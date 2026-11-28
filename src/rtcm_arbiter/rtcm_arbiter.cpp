#include "rtcm_arbiter.h"
#include "../e220_driver/e220_driver.h"
#include "../um982_driver/um982_driver.h"
#include "../settings_manager/settings_manager.h"
#include "../storage/rtcm_logger.h"

#include <Arduino.h>
#include <Field_Instrument_Core.h>
#include <esp_heap_caps.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

extern "C" int rover_printf(const char *format, ...);

static const size_t kNtripRingBytes  = 4096U;
static const size_t kNtripStageBytes = 1536U;

static portMUX_TYPE       s_ring_mux            = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE       s_arb_mux             = portMUX_INITIALIZER_UNLOCKED;
static uint8_t           *s_ntrip_ring_buf      = nullptr;
static FicByteRing_t      s_ntrip_ring          = {};
static uint8_t           *s_ntrip_stage         = nullptr;
static size_t             s_ntrip_stage_len     = 0U;
static uint32_t           s_ntrip_last_rx_ms    = 0U;
static bool               s_ntrip_in_resync     = false;

static RtcmArbiterStats_t s_stats               = {};
static uint32_t           s_ntrip_win_bytes     = 0U;
static uint32_t           s_ntrip_win_start_ms  = 0U;
static bool               s_initialized         = false;

static void *arb_alloc_psram_pref(size_t size)
{
    void *p = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!p) {
        p = heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    return p;
}

static bool rtcm_hdr_plausible_msg(uint16_t msg_type)
{
    return (msg_type >= 1001U && msg_type <= 1300U) ||
           (msg_type >= 4001U && msg_type <= 4095U);
}

static void shift_stage_one_byte(void)
{
    if (s_ntrip_stage_len > 1U) {
        memmove(s_ntrip_stage, s_ntrip_stage + 1U, s_ntrip_stage_len - 1U);
        s_ntrip_stage_len--;
    } else {
        s_ntrip_stage_len = 0U;
    }
}

static bool parse_mode_str(const char *str, RtcmSourceMode_t *mode_out)
{
    if (!str || !mode_out) {
        return false;
    }
    if (strcasecmp(str, "AUTO") == 0) {
        *mode_out = RTCM_MODE_AUTO;
        return true;
    }
    if (strcasecmp(str, "LORA") == 0 || strcasecmp(str, "LORA_ONLY") == 0) {
        *mode_out = RTCM_MODE_LORA_ONLY;
        return true;
    }
    if (strcasecmp(str, "NTRIP") == 0 || strcasecmp(str, "NTRIP_ONLY") == 0) {
        *mode_out = RTCM_MODE_NTRIP_ONLY;
        return true;
    }
    return false;
}

static void process_ntrip_stage(uint32_t now_ms)
{
    while (s_ntrip_stage_len > 0U) {
        if (s_ntrip_stage[0] != 0xD3U) {
            size_t sync_pos = 1U;
            while (sync_pos < s_ntrip_stage_len && s_ntrip_stage[sync_pos] != 0xD3U) {
                sync_pos++;
            }
            portENTER_CRITICAL(&s_arb_mux);
            s_stats.ntrip_orphan_bytes += (uint32_t)sync_pos;
            portEXIT_CRITICAL(&s_arb_mux);
            if (sync_pos < s_ntrip_stage_len) {
                memmove(s_ntrip_stage, s_ntrip_stage + sync_pos, s_ntrip_stage_len - sync_pos);
                s_ntrip_stage_len -= sync_pos;
            } else {
                s_ntrip_stage_len = 0U;
                break;
            }
        }

        if (s_ntrip_stage_len < 3U) {
            break;
        }
        if ((s_ntrip_stage[1] & 0xFCU) != 0x00U) {
            portENTER_CRITICAL(&s_arb_mux);
            s_stats.ntrip_orphan_bytes++;
            portEXIT_CRITICAL(&s_arb_mux);
            shift_stage_one_byte();
            continue;
        }
        uint16_t payload_len = ((uint16_t)(s_ntrip_stage[1] & 0x03U) << 8U) | s_ntrip_stage[2];
        if (payload_len < 6U || payload_len > 1023U) {
            portENTER_CRITICAL(&s_arb_mux);
            s_stats.ntrip_orphan_bytes++;
            portEXIT_CRITICAL(&s_arb_mux);
            shift_stage_one_byte();
            continue;
        }
        if (s_ntrip_stage_len >= 5U) {
            uint16_t cand_type = ((uint16_t)s_ntrip_stage[3] << 4U) | ((s_ntrip_stage[4] >> 4U) & 0x0FU);
            if (!rtcm_hdr_plausible_msg(cand_type)) {
                portENTER_CRITICAL(&s_arb_mux);
                s_stats.ntrip_orphan_bytes++;
                portEXIT_CRITICAL(&s_arb_mux);
                shift_stage_one_byte();
                continue;
            }
        }

        const size_t total_len = (size_t)payload_len + 6U;
        if (total_len > kNtripStageBytes) {
            portENTER_CRITICAL(&s_arb_mux);
            s_stats.ntrip_orphan_bytes++;
            portEXIT_CRITICAL(&s_arb_mux);
            shift_stage_one_byte();
            continue;
        }
        if (s_ntrip_stage_len < total_len) {
            break;
        }

        uint16_t msg_type = 0U;
        if (e220_rtcm_validate_frame(s_ntrip_stage, total_len, &msg_type)) {
            s_ntrip_in_resync = false;
            (void)rtcm_arbiter_submit_frame(RTCM_SRC_BLE_NTRIP,
                                            s_ntrip_stage,
                                            total_len,
                                            msg_type,
                                            now_ms);
            if (s_ntrip_stage_len > total_len) {
                memmove(s_ntrip_stage, s_ntrip_stage + total_len, s_ntrip_stage_len - total_len);
                s_ntrip_stage_len -= total_len;
            } else {
                s_ntrip_stage_len = 0U;
            }
        } else {
            if (!s_ntrip_in_resync) {
                s_ntrip_in_resync = true;
                portENTER_CRITICAL(&s_arb_mux);
                s_stats.ntrip_crc_errors++;
                portEXIT_CRITICAL(&s_arb_mux);
            }
            portENTER_CRITICAL(&s_arb_mux);
            s_stats.ntrip_orphan_bytes++;
            portEXIT_CRITICAL(&s_arb_mux);
            shift_stage_one_byte();
        }
    }
}

extern "C" void rtcm_arbiter_init(void)
{
    if (s_initialized) {
        return;
    }
    s_ntrip_ring_buf = static_cast<uint8_t *>(arb_alloc_psram_pref(kNtripRingBytes));
    s_ntrip_stage    = static_cast<uint8_t *>(arb_alloc_psram_pref(kNtripStageBytes));
    if (s_ntrip_ring_buf) {
        fic_ring_buf_init(&s_ntrip_ring, s_ntrip_ring_buf, kNtripRingBytes);
    }
    memset(&s_stats, 0, sizeof(s_stats));
    RtcmSourceMode_t init_mode = RTCM_MODE_AUTO;
    (void)parse_mode_str(current_settings.rtcm_source_mode, &init_mode);
    s_stats.mode          = init_mode;
    s_stats.active_source = RTCM_SRC_NONE;
    s_ntrip_win_start_ms  = (uint32_t)millis();
    s_initialized         = true;
    rover_printf("[RTCM ARBITER] Initialized (Mode: %s | Ring:%u B PSRAM:%d)\n",
                 rtcm_arbiter_mode_to_str(init_mode),
                 (unsigned)kNtripRingBytes,
                 (s_ntrip_ring_buf && esp_ptr_external_ram(s_ntrip_ring_buf)) ? 1 : 0);
}

extern "C" void rtcm_arbiter_submit_ble_stream(const uint8_t *data, size_t len)
{
    if (!s_initialized || !s_ntrip_ring_buf || !data || len == 0U) {
        return;
    }
    portENTER_CRITICAL(&s_ring_mux);
    (void)fic_ring_buf_push(&s_ntrip_ring, data, len, false);
    portEXIT_CRITICAL(&s_ring_mux);

    portENTER_CRITICAL(&s_arb_mux);
    s_stats.ntrip_bytes_rx += (uint32_t)len;
    s_ntrip_win_bytes      += (uint32_t)len;
    portEXIT_CRITICAL(&s_arb_mux);
}

extern "C" bool rtcm_arbiter_submit_frame(RtcmSourceId_t src,
                                          const uint8_t *frame,
                                          size_t len,
                                          uint16_t msg_type,
                                          uint32_t now_ms)
{
    if (!frame || len == 0U || (src != RTCM_SRC_LORA && src != RTCM_SRC_BLE_NTRIP)) {
        return false;
    }

    bool forward = false;
    bool log_failover_ntrip = false;
    bool log_recover_lora   = false;
    uint32_t cur_failovers  = 0U;

    portENTER_CRITICAL(&s_arb_mux);
    if (src == RTCM_SRC_LORA) {
        s_stats.last_lora_ok_ms = (now_ms == 0U) ? 1U : now_ms;
        if (s_stats.mode == RTCM_MODE_NTRIP_ONLY) {
            s_stats.lora_frames_dropped++;
        } else if (s_stats.mode == RTCM_MODE_LORA_ONLY) {
            s_stats.active_source        = RTCM_SRC_LORA;
            s_stats.lora_recovery_streak = 0U;
            s_stats.lora_frames_ok++;
            forward = true;
        } else {
            // AUTO mode: LoRa primary with 3-frame hysteresis on return from BLE NTRIP
            if (s_stats.active_source == RTCM_SRC_BLE_NTRIP) {
                if (s_stats.lora_recovery_streak < 255U) {
                    s_stats.lora_recovery_streak++;
                }
                if (s_stats.lora_recovery_streak >= RTCM_ARBITER_LORA_RECOVER_COUNT) {
                    s_stats.active_source        = RTCM_SRC_LORA;
                    s_stats.lora_recovery_streak = 0U;
                    s_stats.failover_count++;
                    cur_failovers                = s_stats.failover_count;
                    log_recover_lora             = true;
                    s_stats.lora_frames_ok++;
                    forward = true;
                } else {
                    s_stats.lora_frames_dropped++;
                }
            } else {
                s_stats.active_source        = RTCM_SRC_LORA;
                s_stats.lora_recovery_streak = 0U;
                s_stats.lora_frames_ok++;
                forward = true;
            }
        }
    } else {
        s_stats.last_ntrip_ok_ms    = (now_ms == 0U) ? 1U : now_ms;
        s_stats.last_ntrip_msg_type = msg_type;
        if (s_stats.mode == RTCM_MODE_LORA_ONLY) {
            s_stats.ntrip_frames_dropped++;
        } else if (s_stats.mode == RTCM_MODE_NTRIP_ONLY) {
            s_stats.active_source        = RTCM_SRC_BLE_NTRIP;
            s_stats.lora_recovery_streak = 0U;
            s_stats.ntrip_frames_ok++;
            forward = true;
        } else {
            const bool lora_timed_out = (s_stats.last_lora_ok_ms == 0U) ||
                                        ((now_ms - s_stats.last_lora_ok_ms) > RTCM_ARBITER_LORA_FAILOVER_MS);
            if (s_stats.active_source == RTCM_SRC_BLE_NTRIP) {
                if (lora_timed_out) {
                    s_stats.lora_recovery_streak = 0U;
                }
                s_stats.ntrip_frames_ok++;
                forward = true;
            } else if (lora_timed_out) {
                if (s_stats.active_source == RTCM_SRC_LORA) {
                    s_stats.failover_count++;
                    cur_failovers      = s_stats.failover_count;
                    log_failover_ntrip = true;
                }
                s_stats.active_source        = RTCM_SRC_BLE_NTRIP;
                s_stats.lora_recovery_streak = 0U;
                s_stats.ntrip_frames_ok++;
                forward = true;
            } else {
                s_stats.ntrip_frames_dropped++;
            }
        }
    }
    portEXIT_CRITICAL(&s_arb_mux);

    if (log_failover_ntrip) {
        rover_printf("[RTCM ARBITER] LoRa silent > 3.0s -> Failover to BLE NTRIP! (failovers=%lu)\n",
                     (unsigned long)cur_failovers);
    } else if (log_recover_lora) {
        rover_printf("[RTCM ARBITER] Promoted back to LoRa after %u clean frames (failovers=%lu)\n",
                     (unsigned)RTCM_ARBITER_LORA_RECOVER_COUNT,
                     (unsigned long)cur_failovers);
    }

    if (forward) {
        um982_forward_rtcm(frame, len);
        if (src == RTCM_SRC_BLE_NTRIP && rtcm_logger_is_active()) {
            rtcm_logger_feed(frame, len);
        }
    }
    return forward;
}

extern "C" void rtcm_arbiter_poll(void)
{
    if (!s_initialized || !s_ntrip_ring_buf || !s_ntrip_stage) {
        return;
    }

    uint32_t now_ms = (uint32_t)millis();
    uint8_t chunk[256];
    size_t pulled_total = 0U;

    while (pulled_total < 512U) {
        size_t avail = 0U;
        portENTER_CRITICAL(&s_ring_mux);
        avail = fic_ring_buf_available(&s_ntrip_ring);
        size_t to_pop = (avail > sizeof(chunk)) ? sizeof(chunk) : avail;
        size_t popped = (to_pop > 0U) ? fic_ring_buf_pop(&s_ntrip_ring, chunk, to_pop) : 0U;
        portEXIT_CRITICAL(&s_ring_mux);

        if (popped == 0U) {
            break;
        }
        pulled_total      += popped;
        s_ntrip_last_rx_ms = now_ms;

        if (s_ntrip_stage_len + popped > kNtripStageBytes) {
            size_t drop = (s_ntrip_stage_len + popped) - kNtripStageBytes;
            if (drop >= s_ntrip_stage_len) {
                portENTER_CRITICAL(&s_arb_mux);
                s_stats.ntrip_orphan_bytes += (uint32_t)s_ntrip_stage_len;
                portEXIT_CRITICAL(&s_arb_mux);
                s_ntrip_stage_len = 0U;
            } else {
                memmove(s_ntrip_stage, s_ntrip_stage + drop, s_ntrip_stage_len - drop);
                s_ntrip_stage_len -= drop;
                portENTER_CRITICAL(&s_arb_mux);
                s_stats.ntrip_orphan_bytes += (uint32_t)drop;
                portEXIT_CRITICAL(&s_arb_mux);
            }
        }
        if (popped <= kNtripStageBytes - s_ntrip_stage_len) {
            memcpy(s_ntrip_stage + s_ntrip_stage_len, chunk, popped);
            s_ntrip_stage_len += popped;
            process_ntrip_stage(now_ms);
        }
    }

    if (s_ntrip_stage_len > 0U && s_ntrip_last_rx_ms > 0U &&
        (now_ms - s_ntrip_last_rx_ms) > RTCM_ARBITER_NTRIP_TRUNC_MS) {
        portENTER_CRITICAL(&s_arb_mux);
        s_stats.ntrip_orphan_bytes += (uint32_t)s_ntrip_stage_len;
        portEXIT_CRITICAL(&s_arb_mux);
        s_ntrip_stage_len = 0U;
        s_ntrip_in_resync = false;
    }

    portENTER_CRITICAL(&s_arb_mux);
    if ((now_ms - s_ntrip_win_start_ms) >= 1000U) {
        uint32_t dt = now_ms - s_ntrip_win_start_ms;
        s_stats.ntrip_rx_kbps = ((float)s_ntrip_win_bytes * 8.0f) / (float)dt;
        s_ntrip_win_bytes     = 0U;
        s_ntrip_win_start_ms  = now_ms;
    }
    if (s_stats.active_source == RTCM_SRC_LORA && s_stats.last_lora_ok_ms > 0U &&
        (now_ms - s_stats.last_lora_ok_ms) > RTCM_ARBITER_STALE_MS) {
        s_stats.active_source = RTCM_SRC_NONE;
    } else if (s_stats.active_source == RTCM_SRC_BLE_NTRIP && s_stats.last_ntrip_ok_ms > 0U &&
               (now_ms - s_stats.last_ntrip_ok_ms) > RTCM_ARBITER_STALE_MS) {
        s_stats.active_source = RTCM_SRC_NONE;
    }
    portEXIT_CRITICAL(&s_arb_mux);
}

extern "C" void rtcm_arbiter_set_mode(RtcmSourceMode_t mode)
{
    if (mode > RTCM_MODE_NTRIP_ONLY) {
        mode = RTCM_MODE_AUTO;
    }
    portENTER_CRITICAL(&s_arb_mux);
    s_stats.mode                 = mode;
    s_stats.lora_recovery_streak = 0U;
    if (mode == RTCM_MODE_LORA_ONLY && s_stats.active_source == RTCM_SRC_BLE_NTRIP) {
        s_stats.active_source = RTCM_SRC_NONE;
    } else if (mode == RTCM_MODE_NTRIP_ONLY && s_stats.active_source == RTCM_SRC_LORA) {
        s_stats.active_source = RTCM_SRC_NONE;
    }
    portEXIT_CRITICAL(&s_arb_mux);

    const char *canonical = rtcm_arbiter_mode_to_str(mode);
    if (settings_lock(50U)) {
        snprintf(current_settings.rtcm_source_mode,
                 sizeof(current_settings.rtcm_source_mode),
                 "%s",
                 canonical);
        settings_unlock();
    } else {
        snprintf(current_settings.rtcm_source_mode,
                 sizeof(current_settings.rtcm_source_mode),
                 "%s",
                 canonical);
    }
    rover_printf("[RTCM ARBITER] Mode -> %s\n", canonical);
}

extern "C" bool rtcm_arbiter_set_mode_from_str(const char *mode_str)
{
    RtcmSourceMode_t mode = RTCM_MODE_AUTO;
    if (!parse_mode_str(mode_str, &mode)) {
        return false;
    }
    rtcm_arbiter_set_mode(mode);
    return true;
}

extern "C" RtcmSourceMode_t rtcm_arbiter_get_mode(void)
{
    portENTER_CRITICAL(&s_arb_mux);
    RtcmSourceMode_t m = s_stats.mode;
    portEXIT_CRITICAL(&s_arb_mux);
    return m;
}

extern "C" RtcmSourceId_t rtcm_arbiter_get_active_source(void)
{
    portENTER_CRITICAL(&s_arb_mux);
    RtcmSourceId_t s = s_stats.active_source;
    portEXIT_CRITICAL(&s_arb_mux);
    return s;
}

extern "C" const char *rtcm_arbiter_mode_to_str(RtcmSourceMode_t mode)
{
    if (mode == RTCM_MODE_LORA_ONLY)  return "LORA";
    if (mode == RTCM_MODE_NTRIP_ONLY) return "NTRIP";
    return "AUTO";
}

extern "C" const char *rtcm_arbiter_source_to_str(RtcmSourceId_t src)
{
    if (src == RTCM_SRC_LORA)      return "LORA";
    if (src == RTCM_SRC_BLE_NTRIP) return "NTRIP";
    return "NONE";
}

extern "C" void rtcm_arbiter_get_stats(RtcmArbiterStats_t *out)
{
    if (!out) {
        return;
    }
    portENTER_CRITICAL(&s_arb_mux);
    *out = s_stats;
    portEXIT_CRITICAL(&s_arb_mux);
}
