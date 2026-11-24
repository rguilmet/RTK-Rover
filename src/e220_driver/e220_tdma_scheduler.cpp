#include "e220_driver_internal.h"

// =========================================================================
// Bi-Directional Uplink Telemetry & Remote Air-Rate Switching State
// =========================================================================
RoverUIData_t s_latest_telemetry_state = {};
bool          s_has_telemetry_state    = false;
portMUX_TYPE  s_telem_mux              = portMUX_INITIALIZER_UNLOCKED;
uint32_t      s_last_uplink_tx_ms      = 0;
uint32_t      s_last_uplink_utc_sec    = 0xFFFFFFFFU;
uint8_t       s_uplink_seq             = 0;

volatile bool s_rate_switch_requested    = false;
float         s_requested_rate_kbps      = 0.0f;
uint16_t      s_rate_switch_req_id       = 0;
uint32_t      s_rate_switch_req_tx_ms    = 0;
uint32_t      s_rate_switch_req_start_ms = 0;
bool          s_rate_switch_ack_received = false;
uint32_t      s_rate_switch_exec_ms      = 0;
float         s_active_air_rate          = 19.2f;
float         s_fallback_air_rate        = 19.2f;
uint32_t      s_rate_switch_done_ms      = 0;
uint32_t      s_last_valid_rtcm_ms       = 0;
uint32_t      s_last_valid_msm_utc_sec   = 0xFFFFFFFFU;
bool          s_switched_requires_watchdog = false;
uint32_t      s_tx_air_busy_until_ms     = 0;

static uint32_t e220_estimate_tx_air_ms(size_t n_bytes, float air_rate_kbps)
{
    // Stage 1 silicon-calibrated TX air time (test/_045.._047) + 25ms TX->RX turnaround margin
    float air_ms;
    if (air_rate_kbps <= 3.0f) {
        float eff_bps = air_rate_kbps * 1000.0f * 0.60f;
        if (eff_bps < 1000.0f) eff_bps = 1000.0f;
        air_ms = 22.0f + ((float)(n_bytes + 16U) * 8000.0f) / eff_bps;
    } else if (air_rate_kbps <= 5.0f) {
        air_ms = 70.0f + 1.87f * (float)n_bytes;
    } else if (air_rate_kbps <= 10.0f) {
        air_ms = 45.0f + 1.19f * (float)n_bytes;
    } else if (air_rate_kbps <= 20.0f) {
        air_ms = 29.0f + 0.80f * (float)n_bytes;
    } else {
        float eff_bps = air_rate_kbps * 1000.0f * 0.68f;
        air_ms = 16.0f + ((float)(n_bytes + 16U) * 8000.0f) / eff_bps;
    }
    return (uint32_t)(air_ms + 25.0f);
}

bool e220_get_superframe_clock(uint32_t now_ms,
                               uint32_t *utc_sec_out,
                               uint32_t *superframe_sec_out,
                               uint32_t *ms_in_sec_out)
{
    if (!gps_time_sync.time_synced) {
        return false;
    }
    uint32_t elapsed        = now_ms - gps_time_sync.last_sync_millis;
    uint32_t current_sec    = (gps_time_sync.last_utc_sec + (elapsed / 1000U)) % 86400U;
    uint32_t superframe_sec = current_sec % 5U;
    uint32_t ms_in_sec      = elapsed % 1000U;
    if (utc_sec_out)        *utc_sec_out        = current_sec;
    if (superframe_sec_out) *superframe_sec_out = superframe_sec;
    if (ms_in_sec_out)      *ms_in_sec_out      = ms_in_sec;
    return true;
}

extern "C" bool e220_is_uplink_slot_active(uint32_t now_ms)
{
    uint32_t superframe_sec = 0;
    uint32_t ms_in_sec      = 0;
    if (!e220_get_superframe_clock(now_ms, NULL, &superframe_sec, &ms_in_sec)) {
        return false;
    }
    if (superframe_sec != 0U) {
        return false;
    }
    // Arch 2B consolidated Second 0 uplink quiet slot (aligned to Base DL cutoffs 450/620/680ms):
    // Leaves >= 160ms+ after uplink air TX inside Second 0 for 1x collision-free Mode 0 RSSI query.
    if (s_active_air_rate <= 5.0f) {
        return (ms_in_sec >= 500U && ms_in_sec <= 660U);
    }
    if (s_active_air_rate <= 10.0f) {
        return (ms_in_sec >= 670U && ms_in_sec <= 760U);
    }
    return (ms_in_sec >= 730U && ms_in_sec <= 810U);
}

extern "C" void e220_poll_and_forward_rtcm(void)
{
    e220_stats.aux_pin_high    = (digitalRead(ROVER_PIN_E220_AUX) == HIGH);
    e220_stats.aux_fall_count  = isr_aux_fall_count;
    e220_stats.aux_last_low_us = isr_aux_last_low_us;

    uint32_t poll_now_ms = millis();
    if (e220_stats.aux_pin_high) {
        s_aux_low_since_ms = 0;
    } else if (s_aux_low_since_ms == 0) {
        s_aux_low_since_ms = (poll_now_ms == 0) ? 1U : poll_now_ms;
    }

    if (pending_cfg_write) {
        pending_cfg_write = false;
        e220_write_default_config_registers();
    }
    if (pending_cfg_read) {
        pending_cfg_read = false;
        e220_read_config_registers();
    }

    // 1. Intercept 4-byte (0xC1 0x00 <noise> <pkt>) or 5-byte (0xC1 0x00 0x02 <noise> <pkt>) Mode 0 RSSI reply
    if (rssi_query_active) {
        uint32_t now = millis();
        while (Serial0.available() > 0 && rssi_query_active) {
            uint8_t b = (uint8_t)Serial0.read();
            rssi_reply_buf[rssi_reply_len++] = b;

            bool prefix_mismatch =
                (rssi_reply_len == 1 && rssi_reply_buf[0] != 0xC1) ||
                (rssi_reply_len == 2 && rssi_reply_buf[1] != 0x00) ||
                (rssi_reply_len == 3 && rssi_reply_buf[2] != 0x02U &&
                 (rssi_reply_buf[2] < 0x80U || rssi_reply_buf[2] > 0xE4U)) ||
                (rssi_reply_len == 4 &&
                 ((rssi_reply_buf[2] == 0x02U && (rssi_reply_buf[3] < 0x80U || rssi_reply_buf[3] > 0xE4U)) ||
                  (rssi_reply_buf[2] != 0x02U && rssi_reply_buf[3] != 0x00U &&
                   (rssi_reply_buf[3] < 0x80U || rssi_reply_buf[3] > 0xF6U)))) ||
                (rssi_reply_len == 5 &&
                 rssi_reply_buf[4] != 0x00U && (rssi_reply_buf[4] < 0x80U || rssi_reply_buf[4] > 0xF6U));

            if (prefix_mismatch) {
                // An incoming LoRa packet arrived simultaneously -> cancel RSSI intercept and feed zero-loss to RTCM stage!
                rssi_query_active     = false;
                isr_aux_fall_us       = 0;
                e220_suppress_aux_isr = false;
                uint8_t saved_len     = rssi_reply_len;
                rssi_reply_len        = 0;
                e220_ingest_rx_bytes(rssi_reply_buf, saved_len, now);
                break;
            }

            if (rssi_reply_len == 4 && rssi_reply_buf[2] != 0x02U) {
                // 4-byte E220-900T30S silicon firmware reply: 0xC1 0x00 <noise_raw> <pkt_raw>
                e220_record_rssi_reply(rssi_reply_buf[2], rssi_reply_buf[3], now);
                break;
            }

            if (rssi_reply_len == 5) {
                // 5-byte datasheet reply: 0xC1 0x00 0x02 <noise_raw> <pkt_raw>
                e220_record_rssi_reply(rssi_reply_buf[3], rssi_reply_buf[4], now);
                break;
            }
        }

        if (rssi_query_active) {
            uint32_t q_now_ms = millis();
            uint32_t q_sf_ms  = 0;
            bool     has_q_sf = e220_get_superframe_clock(q_now_ms, NULL, NULL, &q_sf_ms);
            if ((q_now_ms - rssi_query_start_ms > E220_RSSI_REPLY_TIMEOUT_MS) ||
                (has_q_sf && q_sf_ms >= 990U)) {
                rssi_query_active     = false;
                isr_aux_fall_us       = 0;
                e220_suppress_aux_isr = false;
                if (rssi_reply_len > 0 && rssi_reply_buf[0] != 0xC1) {
                    uint8_t saved_len = rssi_reply_len;
                    rssi_reply_len    = 0;
                    e220_ingest_rx_bytes(rssi_reply_buf, saved_len, q_now_ms);
                } else {
                    rssi_reply_len = 0;
                    if (s_rssi_timeout_streak < 255U) {
                        s_rssi_timeout_streak++;
                    }
                    uint32_t rx_silence_ms = (e220_stats.last_rx_ms > 0) ? (q_now_ms - e220_stats.last_rx_ms) : 0xFFFFFFFFU;
                    if (s_rssi_timeout_streak >= 3U && rx_silence_ms > 10000U) {
                        if (e220_stats.radio_online) {
                            Serial.println("[E220 LORA] WARNING: Radio link OFFLINE (3 consecutive Mode 0 poll timeouts & >10s RX silence).");
                        }
                        e220_stats.radio_online = false;
                        e220_stats.rssi_valid   = false;
                        s_was_radio_online      = false;
                    }
                }
            }
        }
    }

    // 2. Drain all available bytes in Serial0 FIFO into the Store-and-Verify RTCM3 Gatekeeper
    uint8_t buf[256];
    int avail;
    while (!rssi_query_active && (avail = Serial0.available()) > 0) {
        if (avail > (int)sizeof(buf)) avail = (int)sizeof(buf);
        int n = Serial0.readBytes((char *)buf, avail);
        if (n <= 0) break;
        e220_ingest_rx_bytes(buf, (size_t)n, millis());
    }

    uint32_t now = millis();

    if (s_aux_low_since_ms > 0 &&
        (now - s_aux_low_since_ms > 3000U) &&
        (e220_stats.last_rx_ms == 0 || (now - e220_stats.last_rx_ms > 3000U))) {
        if (e220_stats.radio_online) {
            Serial.println("[E220 LORA] WARNING: Radio link OFFLINE (AUX held LOW > 3.0s with no UART RX).");
        }
        e220_stats.radio_online = false;
        e220_stats.rssi_valid   = false;
        s_was_radio_online      = false;
    }

    // 3. Flush burst diagnostic log when UART line goes idle for >= 15 ms
    if (burst_bytes > 0 && (now - burst_last_byte_ms >= 15)) {
        e220_flush_burst_log(burst_last_byte_ms);
    }

    // 4. Mid-frame truncation timeout: if tail subpacket did not arrive within rate-adaptive timeout,
    //    quarantine the broken head (never forward to UM982 COM1) and rescan remaining bytes
    //    so any smaller valid frame sitting behind it in rx_stage is rescued.
    uint32_t cur_trunc_timeout_ms = e220_get_trunc_timeout_ms();
    if (rx_stage_len > 0 && (now - last_stage_rx_ms > cur_trunc_timeout_ms)) {
        bool logged_trunc = false;
        while (rx_stage_len > 0) {
            if (!logged_trunc && !rtcm_in_resync) {
                uint16_t payload_len = (rx_stage_len >= 3)
                                       ? (((uint16_t)(rx_stage[1] & 0x03) << 8) | rx_stage[2])
                                       : 0;
                uint16_t msg_type    = (rx_stage_len >= 5)
                                       ? (((uint16_t)rx_stage[3] << 4) | ((rx_stage[4] >> 4) & 0x0F))
                                       : 0;
                e220_stats.rtcm_crc_errors++;
                e220_stats.rtcm_trunc_drops++;
                logged_trunc = true;
                if (current_settings.enable_serial_logging) {
                    Serial.printf("[RTCM DROP TRUNC] Type:%u Got:%u/%uB after %ums (NOT sent to COM1) | OK:%lu Err:%lu(Trunc:%lu)\n",
                                  msg_type,
                                  (unsigned int)rx_stage_len,
                                  (unsigned int)(payload_len + 6U),
                                  (unsigned int)cur_trunc_timeout_ms,
                                  (unsigned long)e220_stats.rtcm_frames_ok,
                                  (unsigned long)e220_stats.rtcm_crc_errors,
                                  (unsigned long)e220_stats.rtcm_trunc_drops);
                }
            }
            rtcm_in_resync = true;
            e220_stats.rtcm_orphan_bytes++;
            e220_shift_stage_one_byte();
            e220_process_rx_stage();
        }
        rtcm_in_resync = false;
    }

    // 5. Synchronized Mode 0 Ambient Noise & Last Packet RSSI query:
    //    - Suppressed while an air-rate switch request/ACK handshake, pending Mode 3 switch/rollback,
    //      or single-subpacket RF TX is actively in flight on the LLCC68.
    //    - In Second 0: poll strictly after uplink RF transmission finishes (810..895ms).
    //    - In Seconds 1..4: poll inside the GPS-disciplined end-of-second quiet micro-slot (855..895ms)
    //      while Base enforces an 850ms downlink TX cutoff, giving 1.0 Hz updates with zero collisions.
    uint32_t sf_utc_sec = 0;
    uint32_t sf_sec     = 0;
    uint32_t sf_ms      = 0;
    bool     has_sf     = e220_get_superframe_clock(now, &sf_utc_sec, &sf_sec, &sf_ms);
    uint32_t ul_block_start_ms = (s_active_air_rate <= 5.0f) ? 440U : ((s_active_air_rate <= 10.0f) ? 610U : 670U);
    bool     in_sec0_tx_block  = has_sf && (sf_sec == 0U) && (sf_ms >= ul_block_start_ms) && (sf_ms <= 860U) &&
                                 (s_last_uplink_utc_sec != sf_utc_sec);
    bool     tx_air_busy     = (s_tx_air_busy_until_ms > 0) && ((int32_t)(now - s_tx_air_busy_until_ms) < 0);
    bool     switch_exec_due = s_rate_switch_ack_received && ((int32_t)(now - s_rate_switch_exec_ms) >= 0);
    bool     watchdog_due    = s_switched_requires_watchdog && (now >= s_rate_switch_done_ms) && ((now - s_rate_switch_done_ms) > 60000U);

    if (!s_rate_switch_requested &&
        !rssi_query_active &&
        !in_sec0_tx_block &&
        !tx_air_busy &&
        !switch_exec_due &&
        !watchdog_due &&
        (!e220_stats.config_valid || ((e220_stats.cfg_reg1 & 0x20) != 0)) &&
        rx_stage_len == 0 &&
        burst_bytes == 0 &&
        (burst_last_byte_ms == 0 || (now - burst_last_byte_ms >= 15U)) &&
        (now - last_rssi_poll_ms >= E220_RSSI_POLL_INTERVAL_MS) &&
        digitalRead(ROVER_PIN_E220_AUX) == HIGH &&
        Serial0.available() == 0) {

        // Consolidated Architecture 2B RSSI polling: run once per macro-cycle in Second 0
        // right after uplink TX completes so Seconds 1..4 remain 100% unbroken downlink.
        bool post_uplink_gap = has_sf && (sf_sec == 0U) &&
                               (s_last_uplink_utc_sec == sf_utc_sec) &&
                               (s_last_rssi_poll_utc_sec != sf_utc_sec) &&
                               (sf_ms >= 680U) && (sf_ms <= 850U);

        bool unsynced_gap    = !has_sf &&
                               (s_active_air_rate > 5.0f) &&
                               (last_terminal_msm_rx_ms > 0) &&
                               (now - last_terminal_msm_rx_ms >= 15U) &&
                               (now - last_terminal_msm_rx_ms <= 45U) &&
                               (now - last_rssi_poll_ms >= 1800U);

        bool idle_link_probe = (e220_stats.last_rx_ms == 0 || (now - e220_stats.last_rx_ms > 3000U)) &&
                               (!has_sf || ((sf_sec != 0U || sf_ms < 390U) && sf_ms <= 810U)) &&
                               (now - last_rssi_poll_ms >= 2000U);

        if (post_uplink_gap || unsynced_gap || idle_link_probe) {
            static const uint8_t kRssiCmd[6] = {0xC0, 0xC1, 0xC2, 0xC3, 0x00, 0x02};
            last_rssi_poll_ms       = now;
            if (has_sf) {
                s_last_rssi_poll_utc_sec = sf_utc_sec;
            }
            rssi_query_start_ms     = now;
            rssi_reply_len          = 0;
            e220_suppress_aux_isr   = true;
            rssi_query_active       = true;
            Serial0.write(kRssiCmd, sizeof(kRssiCmd));
        }
    }

    // 6. Bi-Directional LoRa Uplink & Air-Rate Negotiation Scheduler (GPS-Disciplined 5s Superframe TDMA)
    // Transmits ONLY during the GPS-disciplined Second 0 quiet/uplink slot (max 1 packet per Second 0),
    // provided AUX is HIGH, UART RX is idle, and no RSSI query is in flight.
    if (!rssi_query_active &&
        rx_stage_len == 0 &&
        burst_bytes == 0 &&
        digitalRead(ROVER_PIN_E220_AUX) == HIGH &&
        Serial0.available() == 0) {

        bool uplink_slot_ready = has_sf &&
                                 e220_is_uplink_slot_active(now) &&
                                 (sf_utc_sec != s_last_uplink_utc_sec) &&
                                 (s_last_uplink_tx_ms == 0 || (now - s_last_uplink_tx_ms >= 2000U));

        // A. Priority 1: Transmit Pending Air-Rate Switch Request (12s timeout covers 2 full 5s superframes)
        if (s_rate_switch_requested && !s_rate_switch_ack_received) {
            if (s_rate_switch_req_start_ms > 0 && (now - s_rate_switch_req_start_ms > 12000U)) {
                // Request timeout after 12 seconds of no ACK -> abort and clear pending flag
                Serial.printf("[E220 LORA] Air-rate switch request to %.1fk timed out (no ACK) -> Aborting.\n", s_requested_rate_kbps);
                s_rate_switch_requested        = false;
                e220_stats.rate_switch_pending = false;
                s_rate_switch_req_start_ms     = 0;
            } else if (uplink_slot_ready) {
                uint8_t air_code = ROVER_AIR_CODE_19_2K;
                if (s_requested_rate_kbps < 3.0f)       air_code = ROVER_AIR_CODE_2_4K;
                else if (s_requested_rate_kbps < 7.0f)  air_code = ROVER_AIR_CODE_4_8K;
                else if (s_requested_rate_kbps < 14.0f) air_code = ROVER_AIR_CODE_9_6K;
                else if (s_requested_rate_kbps < 28.0f) air_code = ROVER_AIR_CODE_19_2K;
                else if (s_requested_rate_kbps < 50.0f) air_code = ROVER_AIR_CODE_38_4K;
                else                                    air_code = ROVER_AIR_CODE_62_5K;

                uint8_t chan = (current_settings.e220_channel <= 83) ? current_settings.e220_channel : ROVER_E220_TARGET_CHAN;
                uint8_t tx_buf[ROVER_CMD_SWITCH_FRAME_LEN];
                size_t n = rover_build_rate_switch_req(tx_buf, sizeof(tx_buf), air_code, chan, s_rate_switch_req_id);
                if (n > 0) {
                    e220_suppress_aux_isr = true;
                    Serial0.write(tx_buf, n);
                    Serial0.flush();
                    uint32_t tx_start_ms = millis();
                    s_tx_air_busy_until_ms = tx_start_ms + e220_estimate_tx_air_ms(n, s_active_air_rate);
                    e220_wait_tx_complete(25, 300);
                    now = millis();
                    isr_aux_fall_us = 0;
                    e220_suppress_aux_isr = false;
                    s_rate_switch_req_tx_ms = now;
                    s_last_uplink_tx_ms     = now;
                    s_last_uplink_utc_sec   = sf_utc_sec;
                    Serial.printf("[E220 LORA TX] Sent Rate Switch Req #%u -> air_code 0x%02X (%.1fk)\n",
                                  s_rate_switch_req_id, air_code, s_requested_rate_kbps);
                }
            }
        }
        // B. Priority 2: Periodic Uplink Telemetry (Every 5 seconds in GPS-disciplined Second 0 slot)
        else if (s_has_telemetry_state &&
                 !s_rate_switch_requested &&
                 uplink_slot_ready) {

            RoverUIData_t telem_state;
            portENTER_CRITICAL(&s_telem_mux);
            telem_state = s_latest_telemetry_state;
            portEXIT_CRITICAL(&s_telem_mux);

            uint8_t tx_buf[ROVER_TELEM_FRAME_LEN];
            size_t n = rover_build_telemetry_packet(tx_buf, sizeof(tx_buf), &telem_state, s_uplink_seq);
            if (n > 0) {
                e220_suppress_aux_isr = true;
                Serial0.write(tx_buf, n);
                Serial0.flush();
                uint32_t tx_start_ms = millis();
                s_tx_air_busy_until_ms = tx_start_ms + e220_estimate_tx_air_ms(n, s_active_air_rate);
                e220_wait_tx_complete(25, 300);
                now = millis();
                isr_aux_fall_us = 0;
                e220_suppress_aux_isr = false;
                s_last_uplink_tx_ms   = now;
                s_last_uplink_utc_sec = sf_utc_sec;
                s_uplink_seq++;
                e220_stats.uplink_packets_sent++;
                if (current_settings.enable_serial_logging) {
                    char pwr_str[32];
                    if (telem_state.vbus_present) {
                        snprintf(pwr_str, sizeof(pwr_str), "USB:%.2fV%s",
                                 telem_state.vbus_voltage,
                                 telem_state.is_charging ? "+" : "");
                    } else {
                        snprintf(pwr_str, sizeof(pwr_str), "Bat:%u%%(%.2fV)",
                                 telem_state.battery_percent,
                                 telem_state.battery_voltage);
                    }
                    Serial.printf("[E220 LORA TX #%u] Uplink %uB (FW:%s Fix:%d SLV:%d Tip:%.7f,%.7f,%.2fm BL:%.1fcm P:%+.1fdeg %s)\n",
                                  s_uplink_seq,
                                  (unsigned)n,
                                  ROVER_FIRMWARE_VERSION,
                                  telem_state.fix_type,
                                  telem_state.slave_fix_type,
                                  telem_state.tip_lat,
                                  telem_state.tip_lon,
                                  telem_state.tip_height_m,
                                  telem_state.baseline_length_m * 100.0f,
                                  telem_state.baseline_pitch_deg,
                                  pwr_str);
                }
            }
        }
    }

    // 7. Execute Scheduled Air-Rate Reconfiguration synchronously at Second 2 boundary
    if (s_rate_switch_ack_received && ((int32_t)(now - s_rate_switch_exec_ms) >= 0)) {
        s_rate_switch_ack_received = false;
        s_rate_switch_requested    = false;
        e220_stats.rate_switch_pending = false;

        const float prev_rate = s_active_air_rate;
        Serial.printf("[E220 LORA SWITCH] Reconfiguring E220 registers %.1f -> %.1f kbps...\n",
                      prev_rate, s_requested_rate_kbps);
        current_settings.e220_air_rate_kbps = s_requested_rate_kbps;
        if (e220_write_default_config_registers()) {
            s_fallback_air_rate = prev_rate;
            s_active_air_rate   = s_requested_rate_kbps;
            e220_stats.active_air_rate_kbps = s_active_air_rate;
            e220_stats.uplink_rate_switches++;
            now = millis();
            s_rate_switch_done_ms = now;
            s_switched_requires_watchdog = true;
            mark_settings_dirty();
            Serial.printf("[E220 TDMA] Switched to %.1f kbps (fallback: %.1f kbps). 60s watchdog armed.\n",
                          s_active_air_rate, s_fallback_air_rate);
        } else {
            s_fallback_air_rate = prev_rate;
            e220_stats.active_air_rate_kbps = s_active_air_rate;
            now = millis();
            s_rate_switch_done_ms = now;
            s_switched_requires_watchdog = (fabsf(s_active_air_rate - prev_rate) >= 0.2f);
            mark_settings_dirty();
            Serial.printf("[E220 TDMA] ERROR: Mode 3 write failed during switch to %.1f kbps! Hardware at %.1f kbps.\n",
                          s_requested_rate_kbps, s_active_air_rate);
        }
    }

    // 8. 60-Second Watchdog Rollback Guard
    if (s_switched_requires_watchdog) {
        if (e220_stats.rtcm_frames_ok > 0 && ((int32_t)(s_last_valid_rtcm_ms - s_rate_switch_done_ms) > 0)) {
            // Received valid RTCM strictly AFTER the switch completed -> switch confirmed!
            s_switched_requires_watchdog = false;
            s_fallback_air_rate = s_active_air_rate;
            Serial.printf("[E220 TDMA] Post-switch RTCM verified on %.1fk; rollback watchdog disarmed.\n", s_active_air_rate);
        } else if (now >= s_rate_switch_done_ms && (now - s_rate_switch_done_ms > 60000U)) {
            s_switched_requires_watchdog = false;
            const float target_fallback = (s_fallback_air_rate >= 2.0f && s_fallback_air_rate <= 65.0f) ? s_fallback_air_rate : 19.2f;
            Serial.printf("[E220 LORA WATCHDOG] 60s silence -> Rolling back %.1fk -> %.1fk fallback!\n",
                          s_active_air_rate, target_fallback);
            current_settings.e220_air_rate_kbps = target_fallback;
            if (e220_write_default_config_registers()) {
                s_active_air_rate = target_fallback;
            }
            e220_stats.active_air_rate_kbps = s_active_air_rate;
            s_rate_switch_requested = false;
            e220_stats.rate_switch_pending = false;
            mark_settings_dirty();
        }
    }

    e220_stats.aux_fall_count  = isr_aux_fall_count;
    e220_stats.aux_last_low_us = isr_aux_last_low_us;
}

extern "C" void e220_feed_telemetry_state(const RoverUIData_t *state)
{
    if (state) {
        portENTER_CRITICAL(&s_telem_mux);
        s_latest_telemetry_state = *state;
        s_has_telemetry_state    = true;
        portEXIT_CRITICAL(&s_telem_mux);
    }
}

extern "C" bool e220_request_air_rate_switch(float target_rate_kbps)
{
    if (target_rate_kbps < 2.0f || target_rate_kbps > 65.0f) {
        return false;
    }
    // Reject only once the over-the-air switch request has been transmitted in Second 0
    if (s_rate_switch_requested && (s_rate_switch_req_tx_ms > 0 || s_rate_switch_ack_received)) {
        return false;
    }
    // If rapid cycling brings target back to the currently active rate before Second 0 TX, cancel queued switch
    if (fabsf(target_rate_kbps - s_active_air_rate) < 0.2f) {
        if (s_rate_switch_requested) {
            s_rate_switch_requested        = false;
            s_requested_rate_kbps          = s_active_air_rate;
            e220_stats.rate_switch_pending = false;
            s_rate_switch_req_start_ms     = 0;
            Serial.printf("[E220 TDMA] Queued air-rate switch cancelled (cycled back to %.1fk).\n", s_active_air_rate);
            return true;
        }
        return false;
    }
    s_requested_rate_kbps      = target_rate_kbps;
    s_rate_switch_req_id++;
    if (s_rate_switch_req_id == 0) s_rate_switch_req_id = 1;
    s_rate_switch_requested    = true;
    s_rate_switch_ack_received = false;
    s_rate_switch_req_tx_ms    = 0;
    s_rate_switch_req_start_ms = millis();
    e220_stats.rate_switch_pending = true;
    Serial.printf("[E220 LORA] Queued air-rate switch request -> %.1fk (Req #%u)\n",
                  target_rate_kbps, s_rate_switch_req_id);
    return true;
}

extern "C" bool e220_is_rate_switch_pending(void)
{
    return e220_stats.rate_switch_pending;
}

extern "C" float e220_get_active_air_rate(void)
{
    return s_active_air_rate;
}

extern "C" float e220_get_requested_air_rate(void)
{
    return s_requested_rate_kbps;
}
