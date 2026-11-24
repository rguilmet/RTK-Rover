#include "e220_driver_internal.h"

// =========================================================================
// Sliding-Window Store-and-Verify RTCM3 Gatekeeper State
// Stages incoming Mode 0 bytes, validates 0xD3 header + 24-bit CRC24Q == 0,
// and forwards ONLY verified complete RTCM3 frames atomically to UM982 COM1.
// =========================================================================
uint8_t  rx_stage[RTCM_STAGE_CAPACITY];
size_t   rx_stage_len            = 0;
uint32_t last_stage_rx_ms        = 0;
uint32_t last_1074_rx_ms         = 0;
uint32_t last_terminal_msm_rx_ms = 0;
bool     rtcm_in_resync          = false;

// =========================================================================
// Non-Blocking Mode 0 Live RSSI / Ambient Noise Register Poll State
// Queries C0 C1 C2 C3 00 02 once per 5-second macro-cycle strictly inside
// the Second 0 post-uplink quiet window (no Seconds 1..4 downlink polling).
// Parses both 4-byte (C1 00 <noise> <pkt>) and 5-byte (C1 00 02 <noise> <pkt>)
// E220-900T30S replies (requires REG1 Bit 5 = 0x20).
// =========================================================================
bool     rssi_query_active        = false;
uint32_t last_rssi_poll_ms        = 0;
uint32_t s_last_rssi_poll_utc_sec = 0xFFFFFFFFU;
uint32_t rssi_query_start_ms      = 0;
uint8_t  rssi_reply_buf[5]        = {0};
uint8_t  rssi_reply_len           = 0;
uint8_t  s_rssi_timeout_streak    = 0;
uint32_t s_aux_low_since_ms       = 0;
bool     s_was_radio_online       = false;

// Burst telemetry state
uint32_t burst_bytes          = 0;
uint32_t burst_last_byte_ms   = 0;
uint32_t last_burst_end_ms    = 0;
uint32_t last_burst_aux_edges = 0;

uint32_t e220_get_trunc_timeout_ms(void)
{
    // Subpacket-2 inter-byte gap ceiling up to 400B frames:
    //   4.8k: 847.0 - 442.8 = 404.2ms -> 480ms cleanly quarantines broken heads within 0.5s
    //   9.6k: 527.2 - 282.8 = 244.4ms -> 380ms
    //  19.2k: 336.7 - 186.7 = 150.0ms -> 280ms
    if (s_active_air_rate <= 5.0f)  return 480U;
    if (s_active_air_rate <= 10.0f) return 380U;
    return 280U;
}

extern "C" uint32_t e220_rtcm_crc24q_compute(const uint8_t *data, size_t len)
{
    uint32_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint32_t)data[i] << 16);
        for (int j = 0; j < 8; j++) {
            crc <<= 1;
            if (crc & 0x1000000U) {
                crc ^= 0x1864CFBU;
            }
        }
    }
    return crc & 0xFFFFFFU;
}

static inline bool rtcm_is_plausible_msg_type(uint16_t msg_type)
{
    return (msg_type >= 1001U && msg_type <= 1300U) ||
           (msg_type >= 4001U && msg_type <= 4095U);
}

extern "C" bool e220_rtcm_validate_frame(const uint8_t *frame, size_t len, uint16_t *msg_type_out)
{
    if (!frame || len < 12U || frame[0] != 0xD3U) {
        return false;
    }
    if ((frame[1] & 0xFCU) != 0x00U) {
        return false;
    }
    uint16_t payload_len = ((uint16_t)(frame[1] & 0x03U) << 8) | frame[2];
    if (payload_len < 6U || payload_len > 1023U) {
        return false;
    }
    if (len != ((size_t)payload_len + 6U)) {
        return false;
    }
    uint16_t msg_type = ((uint16_t)frame[3] << 4) | ((frame[4] >> 4) & 0x0FU);
    if (!rtcm_is_plausible_msg_type(msg_type)) {
        return false;
    }
    if (e220_rtcm_crc24q_compute(frame, len) != 0U) {
        return false;
    }
    if (msg_type_out) {
        *msg_type_out = msg_type;
    }
    return true;
}

void e220_shift_stage_one_byte(void)
{
    if (rx_stage_len > 1) {
        memmove(rx_stage, rx_stage + 1, rx_stage_len - 1);
        rx_stage_len--;
    } else {
        rx_stage_len = 0;
    }
}

void e220_process_rx_stage(void)
{
    while (rx_stage_len > 0) {
        // 0. Intercept Downlink Telemetry / Control Frames (0xAA 0x55)
        if (rx_stage_len >= 2 && rx_stage[0] == ROVER_TELEM_SYNC0 && rx_stage[1] == ROVER_TELEM_SYNC1) {
            if (rx_stage_len < 4) {
                break; // Wait for message ID and payload length
            }
            uint8_t msg_id = rx_stage[2];
            uint8_t plen   = rx_stage[3];
            size_t tot_len = (size_t)plen + 6U;
            if (rx_stage_len < tot_len) {
                break; // Wait for complete frame
            }

            if (msg_id == ROVER_MSG_ID_ACK_RATE_SWITCH && tot_len == ROVER_ACK_SWITCH_FRAME_LEN) {
                RoverRateSwitchAck_t ack;
                if (rover_parse_rate_switch_ack(rx_stage, tot_len, &ack)) {
                    if (ack.status == 0 && ack.req_id == s_rate_switch_req_id) {
                        if (!s_rate_switch_ack_received) {
                            s_rate_switch_ack_received = true;
                            uint32_t rx_now_ms  = millis();
                            uint32_t ack_sf_sec = 0;
                            uint32_t ack_sf_ms  = 0;
                            if (e220_get_superframe_clock(rx_now_ms, NULL, &ack_sf_sec, &ack_sf_ms)) {
                                // Align hardware register reconfiguration precisely to top of Second 2 (T0 + 2 @ 0 ms)
                                uint32_t rem_ms = 1000U - ack_sf_ms;
                                if (ack_sf_sec == 0U) {
                                    s_rate_switch_exec_ms = rx_now_ms + rem_ms + 1000U;
                                } else if (ack_sf_sec == 1U) {
                                    s_rate_switch_exec_ms = rx_now_ms + rem_ms;
                                } else {
                                    s_rate_switch_exec_ms = rx_now_ms;
                                }
                            } else {
                                s_rate_switch_exec_ms = rx_now_ms + ack.switch_delay_ms;
                            }
                            Serial.printf("[E220 LORA RX ACK] Rate switch confirmed! (Req #%u Status:%u Delay:%ums)\n",
                                          ack.req_id, ack.status, ack.switch_delay_ms);
                        }
                    } else if (!s_rate_switch_ack_received) {
                        s_rate_switch_requested = false;
                        e220_stats.rate_switch_pending = false;
                    }
                }
            }

            // Consume frame from rx_stage
            if (rx_stage_len > tot_len) {
                memmove(rx_stage, rx_stage + tot_len, rx_stage_len - tot_len);
                rx_stage_len -= tot_len;
            } else {
                rx_stage_len = 0;
            }
            continue;
        }

        // 0b. Intercept Late Mode 0 RSSI Replies (strictly within 250ms of an issued query):
        //     4-byte E220-900T30S reply: 0xC1 0x00 <noise>=0x80..0xE4> <pkt>=0x80..0xF6>
        //     5-byte datasheet reply:    0xC1 0x00 0x02 <noise>=0x80..0xE4> <pkt>=0x80..0xF6>
        bool allow_late_rssi = (last_rssi_poll_ms > 0) && ((millis() - last_rssi_poll_ms) <= 250U);
        if (allow_late_rssi && rx_stage_len >= 2 && rx_stage[0] == 0xC1 && rx_stage[1] == 0x00) {
            if (rx_stage_len < 3) {
                break; // Wait for byte 2 (0x02 or >= 0x80)
            }
            bool is_5b_hdr = (rx_stage[2] == 0x02U);
            bool is_4b_hdr = (rx_stage[2] >= 0x80U && rx_stage[2] <= 0xE4U);
            if (!is_5b_hdr && !is_4b_hdr) {
                e220_stats.rtcm_orphan_bytes++;
                e220_shift_stage_one_byte();
                continue;
            }
            size_t need_len = is_5b_hdr ? 5U : 4U;
            if (rx_stage_len < need_len) {
                break; // Wait for remaining RSSI payload byte(s)
            }
            uint8_t noise_raw = is_5b_hdr ? rx_stage[3] : rx_stage[2];
            uint8_t pkt_raw   = is_5b_hdr ? rx_stage[4] : rx_stage[3];
            if (noise_raw < 0x80U || noise_raw > 0xE4U ||
                (pkt_raw != 0x00U && (pkt_raw < 0x80U || pkt_raw > 0xF6U))) {
                e220_stats.rtcm_orphan_bytes++;
                e220_shift_stage_one_byte();
                continue;
            }
            e220_record_rssi_reply(noise_raw, pkt_raw, millis());
            if (rx_stage_len > need_len) {
                memmove(rx_stage, rx_stage + need_len, rx_stage_len - need_len);
                rx_stage_len -= need_len;
            } else {
                rx_stage_len = 0;
            }
            continue;
        }

        // 1. Strip any leading non-0xD3 orphan bytes before UM982 COM1 ever sees them
        if (rx_stage[0] != 0xD3) {
            size_t sync_pos = 1;
            while (sync_pos < rx_stage_len) {
                if (rx_stage[sync_pos] == 0xD3) {
                    break;
                }
                if (sync_pos + 1 < rx_stage_len &&
                    rx_stage[sync_pos] == ROVER_TELEM_SYNC0 &&
                    rx_stage[sync_pos + 1] == ROVER_TELEM_SYNC1) {
                    break;
                }
                if (allow_late_rssi &&
                    sync_pos + 2 < rx_stage_len &&
                    rx_stage[sync_pos] == 0xC1 &&
                    rx_stage[sync_pos + 1] == 0x00 &&
                    (rx_stage[sync_pos + 2] == 0x02U ||
                     (rx_stage[sync_pos + 2] >= 0x80U && rx_stage[sync_pos + 2] <= 0xE4U))) {
                    break;
                }
                sync_pos++;
            }
            e220_stats.rtcm_orphan_bytes += (uint32_t)sync_pos;
            if (sync_pos < rx_stage_len) {
                memmove(rx_stage, rx_stage + sync_pos, rx_stage_len - sync_pos);
                rx_stage_len -= sync_pos;
            } else {
                rx_stage_len = 0;
                break;
            }
        }

        // Wait for at least 3-byte RTCM3 header (0xD3 + 10-bit length)
        if (rx_stage_len < 3) {
            break;
        }

        // 2. Validate 6 reserved zero bits in byte 1 and plausible payload length [6..1023]
        if ((rx_stage[1] & 0xFC) != 0x00) {
            e220_stats.rtcm_orphan_bytes++;
            e220_shift_stage_one_byte();
            continue;
        }

        uint16_t payload_len = ((uint16_t)(rx_stage[1] & 0x03) << 8) | rx_stage[2];
        if (payload_len < 6 || payload_len > 1023) {
            e220_stats.rtcm_orphan_bytes++;
            e220_shift_stage_one_byte();
            continue;
        }

        // 3. Validate 12-bit RTCM3 message type once 5 bytes are staged
        if (rx_stage_len >= 5) {
            uint16_t cand_type = ((uint16_t)rx_stage[3] << 4) | ((rx_stage[4] >> 4) & 0x0F);
            if (!rtcm_is_plausible_msg_type(cand_type)) {
                e220_stats.rtcm_orphan_bytes++;
                e220_shift_stage_one_byte();
                continue;
            }
        }

        size_t total_len = (size_t)payload_len + 6U;
        if (rx_stage_len < total_len) {
            // Valid header so far; wait for remaining subpacket bytes (up to RTCM_TRUNC_TIMEOUT_MS)
            break;
        }

        // 4. Full candidate frame staged -> verify 24-bit CRC24Q before touching UM982 COM1
        uint16_t msg_type = ((uint16_t)rx_stage[3] << 4) | ((rx_stage[4] >> 4) & 0x0F);
        uint32_t crc = e220_rtcm_crc24q_compute(rx_stage, total_len);
        if (crc == 0) {
            // Verified intact RTCM3 frame -> forward atomically to UM982 COM1!
            um982_forward_rtcm(rx_stage, total_len);

            uint32_t ok_now = millis();
            rtcm_in_resync = false;
            s_last_valid_rtcm_ms = ok_now;
            e220_stats.rtcm_frames_ok++;
            e220_stats.rtcm_bytes_forwarded += (uint32_t)total_len;
            e220_stats.last_msg_type = msg_type;

            if (msg_type == 1005 || msg_type == 1006) {
                e220_stats.msg_1005_count++;
            } else if (msg_type >= 1071 && msg_type <= 1130) {
                e220_stats.msg_msm_count++;
                if (msg_type == 1074) {
                    e220_stats.msg_1074_count++;
                    last_1074_rx_ms = ok_now;
                } else if (msg_type == 1084) {
                    e220_stats.msg_1084_count++;
                } else if (msg_type == 1094) {
                    e220_stats.msg_1094_count++;
                }

                if (msg_type == 1074U || msg_type == 1084U || msg_type == 1094U || msg_type == 1124U) {
                    uint32_t rx_utc_sec = 0;
                    if (e220_get_superframe_clock(ok_now, &rx_utc_sec, NULL, NULL)) {
                        s_last_valid_msm_utc_sec = rx_utc_sec;
                    }
                }

                if ((s_active_air_rate > 10.0f && msg_type == 1094 &&
                     last_1074_rx_ms > 0 && (ok_now - last_1074_rx_ms) <= 850U) ||
                    (s_active_air_rate > 5.0f && s_active_air_rate <= 10.0f &&
                     (msg_type == 1084 || msg_type == 1094) &&
                     last_1074_rx_ms > 0 && (ok_now - last_1074_rx_ms) <= 850U)) {
                    last_terminal_msm_rx_ms = ok_now;
                }
            } else if (msg_type == 1230) {
                e220_stats.msg_1230_count++;
            }

            if (current_settings.enable_serial_logging) {
                Serial.printf("[RTCM OK -> COM1] Type:%u Frame:%uB | OK:%lu Err:%lu(Trunc:%lu) Orph:%luB\n",
                              msg_type,
                              (unsigned int)total_len,
                              (unsigned long)e220_stats.rtcm_frames_ok,
                              (unsigned long)e220_stats.rtcm_crc_errors,
                              (unsigned long)e220_stats.rtcm_trunc_drops,
                              (unsigned long)e220_stats.rtcm_orphan_bytes);
            }

            if (rx_stage_len > total_len) {
                memmove(rx_stage, rx_stage + total_len, rx_stage_len - total_len);
                rx_stage_len -= total_len;
            } else {
                rx_stage_len = 0;
            }
        } else {
            // Corrupted frame or truncated head followed immediately by a new 0xD3 frame!
            // Do NOT forward to UM982 COM1; shift 1 byte so any complete frame inside rx_stage is rescued.
            if (!rtcm_in_resync) {
                rtcm_in_resync = true;
                e220_stats.rtcm_crc_errors++;
                if (current_settings.enable_serial_logging) {
                    Serial.printf("[RTCM DROP CRC] Type:%u Expected:%uB CRC:0x%06lX (NOT sent to COM1) | OK:%lu Err:%lu\n",
                                  msg_type,
                                  (unsigned int)total_len,
                                  (unsigned long)crc,
                                  (unsigned long)e220_stats.rtcm_frames_ok,
                                  (unsigned long)e220_stats.rtcm_crc_errors);
                }
            }
            e220_stats.rtcm_orphan_bytes++;
            e220_shift_stage_one_byte();
        }
    }
}

void e220_flush_burst_log(uint32_t ref_last_byte_ms)
{
    uint32_t gap = (last_burst_end_ms > 0 && ref_last_byte_ms >= last_burst_end_ms)
                   ? (ref_last_byte_ms - last_burst_end_ms)
                   : 0;
    uint32_t cur_edges  = isr_aux_fall_count;
    uint32_t edge_delta = cur_edges - last_burst_aux_edges;
    last_burst_aux_edges = cur_edges;

    if (current_settings.enable_serial_logging) {
        Serial.printf("[E220 RX BURST] %lu B (gap: %lu ms | AUX edges:+%lu low:%.2fms)\n",
                      (unsigned long)burst_bytes,
                      (unsigned long)gap,
                      (unsigned long)edge_delta,
                      (float)isr_aux_last_low_us / 1000.0f);
    }
    last_burst_end_ms = ref_last_byte_ms;
    burst_bytes = 0;
}

void e220_ingest_rx_bytes(const uint8_t *data, size_t len, uint32_t now)
{
    if (len == 0) {
        return;
    }

    if (burst_bytes > 0 && (now - burst_last_byte_ms >= 15)) {
        e220_flush_burst_log(burst_last_byte_ms);
    }

    burst_bytes += (uint32_t)len;
    burst_last_byte_ms = now;

    e220_stats.rx_bytes_total   += (uint32_t)len;
    e220_stats.last_rx_ms        = now;
    e220_stats.radio_online      = true;
    e220_stats.last_radio_rsp_ms = now;
    s_rssi_timeout_streak        = 0;
    if (!s_was_radio_online) {
        s_was_radio_online = true;
        Serial.println("[E220 LORA] Radio link ONLINE (UART RX activity detected).");
    }
    last_stage_rx_ms = now;

    // Preserve raw wire-tap to SD/FFat rtcm-base_*.txt for forensic analysis
    if (rtcm_logger_is_active()) {
        rtcm_logger_feed(data, len);
    }

    // Append into sliding-window Gatekeeper buffer (shed oldest orphan bytes if near capacity)
    if (len >= RTCM_STAGE_CAPACITY) {
        e220_stats.rtcm_orphan_bytes += (uint32_t)(rx_stage_len + (len - RTCM_STAGE_CAPACITY));
        memcpy(rx_stage, data + (len - RTCM_STAGE_CAPACITY), RTCM_STAGE_CAPACITY);
        rx_stage_len = RTCM_STAGE_CAPACITY;
    } else {
        if (rx_stage_len + len > RTCM_STAGE_CAPACITY) {
            size_t drop = (rx_stage_len + len) - RTCM_STAGE_CAPACITY;
            e220_stats.rtcm_orphan_bytes += (uint32_t)drop;
            memmove(rx_stage, rx_stage + drop, rx_stage_len - drop);
            rx_stage_len -= drop;
        }
        memcpy(rx_stage + rx_stage_len, data, len);
        rx_stage_len += len;
    }

    e220_process_rx_stage();
}

void e220_record_rssi_reply(uint8_t noise_raw, uint8_t pkt_raw, uint32_t now)
{
    int16_t cand_noise_dbm = (int16_t)noise_raw - 256;
    int16_t cand_pkt_dbm   = (int16_t)pkt_raw - 256;
    // Reject ambient noise samples contaminated by active close-range RF carrier demodulation
    bool carrier_contaminated = (cand_noise_dbm > -65) &&
                                (cand_pkt_dbm > -65) &&
                                ((cand_noise_dbm - cand_pkt_dbm) >= -8) &&
                                ((cand_noise_dbm - cand_pkt_dbm) <= 8);
    if (noise_raw > 0x80U && noise_raw <= 0xE4U && !carrier_contaminated) {
        e220_stats.ambient_noise_dbm = cand_noise_dbm;
    }
    if (e220_stats.rx_bytes_total > 0 && pkt_raw >= 0x80U && pkt_raw <= 0xF6U) {
        e220_stats.last_pkt_rssi_dbm = cand_pkt_dbm;
        e220_stats.rssi_valid        = true;
    }
    e220_stats.radio_online      = true;
    e220_stats.last_radio_rsp_ms = now;
    s_rssi_timeout_streak        = 0;
    if (!s_was_radio_online) {
        s_was_radio_online = true;
        Serial.println("[E220 LORA] Radio link ONLINE (Mode 0 RSSI response verified).");
        if (!e220_stats.config_valid) {
            pending_cfg_read = true;
        }
    }
    rssi_query_active     = false;
    rssi_reply_len        = 0;
    isr_aux_fall_us       = 0;
    e220_suppress_aux_isr = false;
}
