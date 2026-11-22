#include <Arduino.h>
#include <math.h>
#include <SD_MMC.h>
#include <SD.h>
#include <FFat.h>
#include <SPI.h>
#include <driver/gpio.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

#include "user_config.h"
#include "src/rover_api.h"
#include "src/i2c_bsp/i2c_bsp.h"
#include "src/tca9554/esp_io_expander_tca9554.h"
#include "src/axp2101_bsp/axp2101_bsp.h"
#include "src/rtc_bsp/rtc_bsp.h"
#include "src/imu_bsp/imu_bsp.h"
#include "src/mag_bsp/mmc5983ma_driver.h"
#include "src/beam_imu_bsp/sen0696_driver.h"
#include "src/settings_manager/settings_manager.h"
#include "src/nav_engine/nav_engine.h"
#include "src/waypoint_manager/waypoint_manager.h"
#include "src/e220_driver/e220_driver.h"
#include "src/um982_driver/um982_driver.h"
#include "src/lvgl_port/lvgl_port.h"
#include "src/screenshot/screenshot.h"
#include "src/codec_board/codec_board.h"
#include "src/codec_board/codec_init.h"
#include "src/storage/route_logger.h"
#include "src/storage/nmea_logger.h"
#include "src/storage/rtcm_logger.h"
#include "src/storage/serial_logger.h"
#include "src/web_server/web_server.h"

extern esp_io_expander_handle_t io_expander;
bool sd_card_mounted = false;
bool sd_card_use_spi = false;

typedef struct {
    uint16_t freq1_hz;
    uint16_t dur1_ms;
    uint16_t gap_ms;
    uint16_t freq2_hz;
    uint16_t dur2_ms;
} RoverBeep_t;

static QueueHandle_t roverStateQueue = NULL;
static QueueHandle_t beepQueue       = NULL;
static esp_codec_dev_handle_t playback_handle = NULL;
static volatile bool     pending_tare_level   = false;
static portMUX_TYPE      s_pwr_snapshot_mux   = portMUX_INITIALIZER_UNLOCKED;
static Axp2101Status_t   s_pwr_snapshot       = {};
static portMUX_TYPE      s_imu_snapshot_mux   = portMUX_INITIALIZER_UNLOCKED;
static float             s_snapshot_bubble_x  = 0.0f;
static float             s_snapshot_bubble_y  = 0.0f;
static float             s_snapshot_pole_tilt = 0.0f;

// =========================================================================
// Backlight PWM Control (GPIO 6 on ESP32-S3-Touch-LCD-3.5B)
// =========================================================================
extern "C" void set_backlight_pwm(uint8_t brightness_pct)
{
    if (brightness_pct > 100) brightness_pct = 100;
    uint8_t duty = (uint8_t)((brightness_pct * 255) / 100);
    ledcWrite(ROVER_PIN_BK_LIGHT, duty);
}

extern "C" void init_backlight_pwm(void)
{
    ledcAttach(ROVER_PIN_BK_LIGHT, 5000, 8);
    set_backlight_pwm(current_settings.brightness);
}

extern "C" void set_rover_brightness(uint8_t brightness_pct)
{
    if (brightness_pct > 100) brightness_pct = 100;
    current_settings.brightness = brightness_pct;
    set_backlight_pwm(brightness_pct);
}

extern "C" void set_rover_volume(uint8_t volume_pct)
{
    if (volume_pct > 100) volume_pct = 100;
    current_settings.volume = volume_pct;
    current_settings.is_muted = (volume_pct == 0);
    if (playback_handle) {
        esp_codec_dev_set_out_vol(playback_handle, (int)volume_pct);
    }
}

// =========================================================================
// Audio Beep Synthesis via ES8311 (Optional Feedback for Logging / Screenshot)
// =========================================================================
#define AUDIO_SAMPLE_RATE 24000

static void audio_bsp_init(void)
{
    if (io_expander != NULL) {
        esp_io_expander_set_dir(io_expander, ROVER_EXIO_PA_CTRL, IO_EXPANDER_OUTPUT);
        esp_io_expander_set_level(io_expander, ROVER_EXIO_PA_CTRL, 1);
    }

    set_codec_board_type("S3_LCD_3_5B");
    codec_init_cfg_t codec_cfg = {
        .in_mode = CODEC_I2S_MODE_NONE,
        .out_mode = CODEC_I2S_MODE_STD,
        .in_use_tdm = false,
        .reuse_dev = false,
    };
    if (init_codec(&codec_cfg) == 0) {
        playback_handle = get_playback_handle();
        if (playback_handle) {
            esp_codec_dev_set_out_vol(playback_handle, (int)current_settings.volume);
            esp_codec_dev_sample_info_t fs = {};
            fs.sample_rate = AUDIO_SAMPLE_RATE;
            fs.channel = 2;
            fs.bits_per_sample = 16;
            esp_codec_dev_open(playback_handle, &fs);
        }
    }
}

extern "C" void play_rover_beep(uint16_t freq_hz, uint16_t duration_ms)
{
    if (beepQueue) {
        RoverBeep_t b = { freq_hz, duration_ms, 0, 0, 0 };
        xQueueSend(beepQueue, &b, 0);
    }
}

extern "C" void play_rover_dual_beep(uint16_t freq1_hz, uint16_t dur1_ms, uint16_t gap_ms, uint16_t freq2_hz, uint16_t dur2_ms)
{
    if (beepQueue) {
        RoverBeep_t b = { freq1_hz, dur1_ms, gap_ms, freq2_hz, dur2_ms };
        xQueueSend(beepQueue, &b, 0);
    }
}

extern "C" void trigger_tare_pole_level(void)
{
    pending_tare_level = true;
}

static void audio_write_tone_or_silence(int16_t *pcm_buf, uint16_t freq_hz, uint16_t dur_ms)
{
    if (!playback_handle || !pcm_buf || dur_ms == 0) return;
    uint32_t total_samples = (AUDIO_SAMPLE_RATE * (uint32_t)dur_ms) / 1000U;
    if (total_samples == 0) return;

    float phase = 0.0f;
    float phase_inc = (freq_hz > 0) ? (2.0f * (float)M_PI * (float)freq_hz / (float)AUDIO_SAMPLE_RATE) : 0.0f;
    uint32_t written = 0;

    while (written < total_samples) {
        uint32_t rem   = total_samples - written;
        uint32_t chunk = (rem < 256U) ? rem : 256U;
        if (freq_hz == 0) {
            memset(pcm_buf, 0, chunk * 2U * sizeof(int16_t));
        } else {
            for (uint32_t i = 0; i < chunk; i++) {
                int16_t s = (phase < (float)M_PI) ? 6000 : -6000;
                pcm_buf[i * 2U]      = s;
                pcm_buf[i * 2U + 1U] = s;
                phase += phase_inc;
                if (phase >= 2.0f * (float)M_PI) phase -= 2.0f * (float)M_PI;
            }
        }
        esp_codec_dev_write(playback_handle, pcm_buf, chunk * 2U * sizeof(int16_t));
        written += chunk;
    }
}

// =========================================================================
// Dedicated Core 1 Real-Time Audio Task (Priority 4)
// Immediately synthesizes button-press and proximity tones via I2S DMA so UI
// tactile audio never lags behind LVGL rendering, SD card, or flash I/O.
// =========================================================================
void task_audio_beep(void *pvParameters)
{
    (void)pvParameters;
    static int16_t s_beep_pcm[256 * 2];
    RoverBeep_t beep_req;

    for (;;) {
        if (beepQueue && xQueueReceive(beepQueue, &beep_req, portMAX_DELAY) == pdPASS) {
            if (playback_handle && !current_settings.is_muted) {
                if (beep_req.freq1_hz > 0 && beep_req.dur1_ms > 0) {
                    audio_write_tone_or_silence(s_beep_pcm, beep_req.freq1_hz, beep_req.dur1_ms);
                }
                if (beep_req.gap_ms > 0) {
                    audio_write_tone_or_silence(s_beep_pcm, 0, beep_req.gap_ms);
                }
                if (beep_req.freq2_hz > 0 && beep_req.dur2_ms > 0) {
                    audio_write_tone_or_silence(s_beep_pcm, beep_req.freq2_hz, beep_req.dur2_ms);
                }
                if ((beep_req.freq1_hz > 0 && beep_req.dur1_ms > 0) ||
                    (beep_req.freq2_hz > 0 && beep_req.dur2_ms > 0)) {
                    audio_write_tone_or_silence(s_beep_pcm, 0, 4);
                }
            }
        }
    }
}

// =========================================================================
// Core 0 Task: Real-Time UART Bridge (E220 <-> UM982 COM2) & Geodesic Math
// NOTE: Core 0 performs ZERO I2C bus transactions. All I2C0 peripherals live
// exclusively on Core 1 to eliminate cross-core I2C bus contention.
// =========================================================================
void task_rover_core0(void *pvParameters)
{
    (void)pvParameters;
    esp_task_wdt_add(NULL);
    RoverUIData_t state = {};
    uint32_t last_ui_push_ms   = 0;
    uint32_t last_telem_log_ms = 0;
    uint32_t last_wp_seq       = g_waypoint_change_seq;

    for (;;) {
        // 1. Poll E220 LoRa UART0 (IO41/42) and immediately forward RTCM3 to UM982 COM1 (IO17/18)
        e220_poll_and_forward_rtcm();

        // 2. Poll UM982 COM1 (IO17/18) for NMEA GGA/RMC/GST/GSV, Heading, and Command Responses
        um982_poll_com1();

        uint32_t now = millis();
        uint32_t cur_wp_seq = g_waypoint_change_seq;
        bool wp_changed = (cur_wp_seq != last_wp_seq);

        // 3. At ~30Hz (or immediately within <=2ms when active waypoint/file changes), fuse GNSS kinematics & guidance
        if (wp_changed || (now - last_ui_push_ms >= 33)) {
            last_ui_push_ms = now;
            last_wp_seq     = cur_wp_seq;

            um982_get_latest_state(&state);
            um982_get_port_stats(&state.um982_stats);
            e220_get_stats(&state.e220_stats);

            // Compute Pole Centerline Ground Tip Position (compensating Master->Slave 555mm spacing & pole height)
            nav_compute_pole_tip_position(state.raw_lat,
                                          state.raw_lon,
                                          state.raw_alt_m,
                                          state.heading_valid,
                                          state.heading_source,
                                          state.true_heading_deg,
                                          state.baseline_pitch_deg,
                                          &state.tip_lat,
                                          &state.tip_lon,
                                          &state.tip_height_m);

            // Compute Geodesic Guidance to Active CSV Waypoint
            WaypointRecord_t active_wp;
            state.total_targets     = g_waypoint_count;
            state.active_target_idx = g_active_waypoint_idx;
            if (waypoint_get_active(&active_wp)) {
                state.has_active_target = true;
                strncpy(state.target_label, active_wp.label, WAYPOINT_LABEL_LEN - 1);
                strncpy(state.target_notes, active_wp.notes, WAYPOINT_NOTES_LEN - 1);
                nav_compute_target_guidance(state.tip_lat,
                                            state.tip_lon,
                                            state.heading_valid ? state.true_heading_deg : 0.0f,
                                            active_wp.latitude,
                                            active_wp.longitude,
                                            &state.distance_to_target_m,
                                            &state.bearing_to_target_deg,
                                            &state.rel_bearing_deg,
                                            &state.d_north_m,
                                            &state.d_east_m);
            } else {
                state.has_active_target = false;
            }

            // Attach Core 1 calibrated pole tilt snapshot so survey CSV averaging logs actual tilt
            portENTER_CRITICAL(&s_imu_snapshot_mux);
            state.pole_bubble_x_deg = s_snapshot_bubble_x;
            state.pole_bubble_y_deg = s_snapshot_bubble_y;
            state.pole_tilt_deg     = s_snapshot_pole_tilt;
            portEXIT_CRITICAL(&s_imu_snapshot_mux);

            // Update Multi-Epoch Survey Logger State
            waypoint_update_survey_averaging(&state,
                                             &state.survey_in_progress,
                                             &state.survey_progress_pct,
                                             &state.survey_epochs_collected,
                                             state.last_logged_label);

            if (roverStateQueue) {
                xQueueOverwrite(roverStateQueue, &state);
            }

            // Periodic 5-second diagnostic telemetry log to Serial Monitor / SD Logger
            if (current_settings.enable_telemetry_logging && (now - last_telem_log_ms >= 5000)) {
                last_telem_log_ms = now;
                const char *hdg_mode = "--";
                if (state.heading_valid) {
                    if (state.heading_source == HDG_SRC_DUAL_FIX) hdg_mode = "FIX";
                    else if (state.heading_source == HDG_SRC_DUAL_FLT) hdg_mode = "FLT";
                    else if (state.heading_source == HDG_SRC_DISP) hdg_mode = "WALK";
                    else if (state.heading_source == HDG_SRC_COMP) hdg_mode = "CMP";
                    else if (state.heading_source == HDG_SRC_GYRO) hdg_mode = "GYRO";
                    else if (state.heading_source == HDG_SRC_HOLD) hdg_mode = "HLD";
                    else if (state.heading_source == HDG_SRC_COG) hdg_mode = "COG";
                    else hdg_mode = state.heading_is_fixed ? "FIX" : "FLT";
                }
                char rssi_str[36];
                if (state.e220_stats.rssi_valid) {
                    snprintf(rssi_str, sizeof(rssi_str), "Noise:%ddBm Pkt:%ddBm",
                             (int)state.e220_stats.ambient_noise_dbm,
                             (int)state.e220_stats.last_pkt_rssi_dbm);
                } else {
                    snprintf(rssi_str, sizeof(rssi_str), "Noise:--dBm Pkt:--dBm");
                }
                char pwr_str[32];
                Axp2101Status_t pwr_snap;
                portENTER_CRITICAL(&s_pwr_snapshot_mux);
                pwr_snap = s_pwr_snapshot;
                portEXIT_CRITICAL(&s_pwr_snapshot_mux);
                if (!pwr_snap.online) {
                    snprintf(pwr_str, sizeof(pwr_str), "Pwr:--");
                } else if (pwr_snap.vbus_good) {
                    snprintf(pwr_str, sizeof(pwr_str), "USB:%.2fV%s",
                             pwr_snap.vbus_voltage_v,
                             pwr_snap.is_charging ? "+" : "");
                } else {
                    snprintf(pwr_str, sizeof(pwr_str), "BAT:%u%%(%.2fV)",
                             pwr_snap.battery_percent,
                             pwr_snap.battery_voltage_v);
                }
                Serial.printf("[TELEM] Fix:%d SVs:%u/%u Hdg:%s(%.1f) BL:%.1fcm Spd:%.1fm/s Base:%u Diff:%.1fs | %s | COM1 RX:%luB NMEA:%lu RTCM_TX:%luB | E220 AUX:%d (Edges:%lu) %s RX:%luB Fwd:%luB OK:%lu (1006:%u 1074:%u 1084:%u 1094:%u 1230:%u) Err:%lu (Trunc:%lu) Orph:%luB\n",
                              (int)state.fix_type,
                              state.num_sats,
                              state.sats_in_view,
                              hdg_mode,
                              state.true_heading_deg,
                              state.baseline_length_m * 100.0f,
                              state.speed_m_s,
                              state.base_station_id,
                              state.diff_age_s,
                              pwr_str,
                              (unsigned long)state.um982_stats.com1_rx_bytes,
                              (unsigned long)state.um982_stats.nmea_sentences_ok,
                              (unsigned long)state.um982_stats.rtcm_tx_bytes,
                              state.e220_stats.aux_pin_high ? 1 : 0,
                              (unsigned long)state.e220_stats.aux_fall_count,
                              rssi_str,
                              (unsigned long)state.e220_stats.rx_bytes_total,
                              (unsigned long)state.e220_stats.rtcm_bytes_forwarded,
                              (unsigned long)state.e220_stats.rtcm_frames_ok,
                              state.e220_stats.msg_1005_count,
                              state.e220_stats.msg_1074_count,
                              state.e220_stats.msg_1084_count,
                              state.e220_stats.msg_1094_count,
                              state.e220_stats.msg_1230_count,
                              (unsigned long)state.e220_stats.rtcm_crc_errors,
                              (unsigned long)state.e220_stats.rtcm_trunc_drops,
                              (unsigned long)state.e220_stats.rtcm_orphan_bytes);
            }
        }

        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// =========================================================================
// Core 1 Task: UI Display Update, Onboard I2C Peripherals & Audio Alerts
// =========================================================================
void task_display_and_audio(void *pvParameters)
{
    (void)pvParameters;
    RoverUIData_t    ui_state          = {};
    Axp2101Status_t  pwr               = {};
    Mmc5983maState_t cached_mag         = {};
    uint32_t         last_imu_poll_ms   = 0;
    uint32_t         last_imu_log_ms    = 0;
    uint32_t         last_pwr_poll_ms   = 0;
    uint32_t         last_10dof_poll_ms = millis();
    float            cached_imu_pitch  = 0.0f;
    float            cached_imu_roll   = 0.0f;
    float            cached_bubble_x   = 0.0f;
    float            cached_bubble_y   = 0.0f;
    DisplayPose_t    cached_pose       = DISPLAY_POSE_FLAT;
    float            cached_acc[3]     = {0.0f, 0.0f, 0.0f};
    float            cached_gyr[3]     = {0.0f, 0.0f, 0.0f};
    float            prev_imu_acc[3]   = {0.0f, 0.0f, 0.0f};
    bool             prev_imu_valid    = false;
    uint32_t         last_imu_motion_ms = 0;
    float            prev_10dof_acc[3] = {0.0f, 0.0f, 0.0f};
    bool             prev_10dof_valid  = false;
    double           motion_anchor_lat = 0.0;
    double           motion_anchor_lon = 0.0;
    bool             motion_anchor_valid = false;
    uint32_t         last_gnss_motion_ms = 0;

    uint32_t         s_auto_off_idle_s      = 0;
    uint8_t          s_auto_off_countdown_s = 0;
    bool             s_auto_off_inhibited   = false;
    char             s_auto_off_reason[16]  = "";

    for (;;) {
        uint32_t now = millis();

        // 1..3: Execute Core 1 I2C transactions (RTC sync, 10 Hz IMU + MMC5983MA Compass, 0.5 Hz PMIC).
        // Hardware bus thread-safety across tasks is guaranteed by i2c_bus_mutex in i2c_bsp.
        // No dependency on the LVGL graphics mutex so sensor polling is never starved.
        uint16_t yy; uint8_t mo, dd, hh, mm, ss;
        if (um982_poll_pending_rtc_sync(&yy, &mo, &dd, &hh, &mm, &ss)) {
            i2c_rtc_setTime(yy, mo, dd, hh, mm, ss);
            Serial.printf("RTC auto-synced to GNSS UTC: %04u-%02u-%02u %02u:%02u:%02u\n", yy, mo, dd, hh, mm, ss);
        }

        if (now - last_imu_poll_ms >= 100 || last_imu_poll_ms == 0) {
            last_imu_poll_ms = (now == 0) ? 1 : now;
            int16_t imu_temp = 0;
            imu_read(cached_acc, cached_gyr, &imu_temp);
            imu_compute_pose_and_bubble(cached_acc,
                                        &cached_imu_pitch,
                                        &cached_imu_roll,
                                        &cached_bubble_x,
                                        &cached_bubble_y,
                                        &cached_pose);

            float imu_pole_tilt = sqrtf(cached_bubble_x * cached_bubble_x +
                                        cached_bubble_y * cached_bubble_y);
            portENTER_CRITICAL(&s_imu_snapshot_mux);
            s_snapshot_bubble_x  = cached_bubble_x;
            s_snapshot_bubble_y  = cached_bubble_y;
            s_snapshot_pole_tilt = imu_pole_tilt;
            portEXIT_CRITICAL(&s_imu_snapshot_mux);

            float gyr_mag = sqrtf(cached_gyr[0] * cached_gyr[0] +
                                  cached_gyr[1] * cached_gyr[1] +
                                  cached_gyr[2] * cached_gyr[2]);
            float acc_delta = prev_imu_valid
                ? (fabsf(cached_acc[0] - prev_imu_acc[0]) +
                   fabsf(cached_acc[1] - prev_imu_acc[1]) +
                   fabsf(cached_acc[2] - prev_imu_acc[2]))
                : 0.0f;
            if (gyr_mag >= 8.0f || acc_delta >= 0.08f) {
                last_imu_motion_ms = (now == 0) ? 1 : now;
            }
            prev_imu_acc[0] = cached_acc[0];
            prev_imu_acc[1] = cached_acc[1];
            prev_imu_acc[2] = cached_acc[2];
            prev_imu_valid  = true;

            // Poll outboard MMC5983MA 3-Axis 18-bit AMR Compass (if auto-detected at boot)
            mmc5983ma_poll(cached_bubble_y, cached_bubble_x, &cached_mag);

            if (current_settings.enable_imu_logging && (now - last_imu_log_ms >= 1000)) {
                last_imu_log_ms = now;
                Serial.printf("[IMU POSE] Pitch: %+.1f deg | Roll: %+.1f deg | Acc(g): %+.2f %+.2f %+.2f | Gyr(dps): %+.1f %+.1f %+.1f | Pose: %d\n",
                              cached_imu_pitch, cached_imu_roll,
                              cached_acc[0], cached_acc[1], cached_acc[2],
                              cached_gyr[0], cached_gyr[1], cached_gyr[2],
                              (int)cached_pose);
            }

            if (pending_tare_level) {
                pending_tare_level = false;
                current_settings.pole_tilt_tare_pitch_deg = cached_imu_pitch;
                current_settings.pole_tilt_tare_roll_deg  = cached_imu_roll;
                mark_settings_dirty();
                play_rover_beep(1800, 80);
            }
        }

        if (now - last_pwr_poll_ms >= 2000 || last_pwr_poll_ms == 0) {
            last_pwr_poll_ms = (now == 0) ? 1 : now;
            axp2101_bsp_read(&pwr);

            portENTER_CRITICAL(&s_pwr_snapshot_mux);
            s_pwr_snapshot = pwr;
            portEXIT_CRITICAL(&s_pwr_snapshot_mux);

            static bool s_pwr_init = false;
            static bool s_last_vbus_good = false;
            if (s_pwr_init && (pwr.vbus_good != s_last_vbus_good)) {
                Serial.printf("[POWER] Transition: %s -> %s | VBUS:%.2fV VBAT:%.2fV (%u%%) Chg:%s\n",
                              s_last_vbus_good ? "USB" : "BATTERY",
                              pwr.vbus_good ? "USB" : "BATTERY",
                              pwr.vbus_voltage_v,
                              pwr.battery_voltage_v,
                              pwr.battery_percent,
                              pwr.is_charging ? "YES" : "NO");
            }
            s_last_vbus_good = pwr.vbus_good;
            s_pwr_init       = true;
        }

        // Staggered 1 Hz always-on DFRobot SEN0696 (10DOF IMU) polling across 3 RoverUI ticks (~10.3 ms/tick, 400 kHz)
        static uint8_t s_10dof_step     = 0;
        static bool    s_10dof_in_cycle = false;
        if (sen0696_is_present()) {
            const uint32_t period_ms = (sen0696_get_consec_fails() >= 15U) ? 5000U : 1000U;
            if (!s_10dof_in_cycle && (now - last_10dof_poll_ms >= period_ms)) {
                last_10dof_poll_ms = now;
                s_10dof_in_cycle   = true;
                s_10dof_step       = 0;
            }
            if (s_10dof_in_cycle) {
                sen0696_state_t beam_state;
                bool ok = sen0696_poll_step(s_10dof_step, &beam_state);
                s_10dof_step++;
                if (s_10dof_step >= 3U) {
                    s_10dof_in_cycle = false;
                    s_10dof_step     = 0;
                    if (ok) {
                        float beam_gyr_mag = sqrtf(beam_state.gyro_dps[0] * beam_state.gyro_dps[0] +
                                                   beam_state.gyro_dps[1] * beam_state.gyro_dps[1] +
                                                   beam_state.gyro_dps[2] * beam_state.gyro_dps[2]);
                        float beam_acc_delta = prev_10dof_valid
                            ? (fabsf(beam_state.accel_g[0] - prev_10dof_acc[0]) +
                               fabsf(beam_state.accel_g[1] - prev_10dof_acc[1]) +
                               fabsf(beam_state.accel_g[2] - prev_10dof_acc[2]))
                            : 0.0f;
                        if (beam_gyr_mag >= 8.0f || beam_acc_delta >= 0.08f) {
                            last_imu_motion_ms = (now == 0) ? 1 : now;
                        }
                        prev_10dof_acc[0] = beam_state.accel_g[0];
                        prev_10dof_acc[1] = beam_state.accel_g[1];
                        prev_10dof_acc[2] = beam_state.accel_g[2];
                        prev_10dof_valid  = true;

                        Serial.printf("[10DOF] Acc(g):%+.3f,%+.3f,%+.3f | Gyr(dps):%+.2f,%+.2f,%+.2f | Mag(uT):%+.2f,%+.2f,%+.2f (|B|=%.1f) | P:%.2fhPa\n",
                                      beam_state.accel_g[0], beam_state.accel_g[1], beam_state.accel_g[2],
                                      beam_state.gyro_dps[0], beam_state.gyro_dps[1], beam_state.gyro_dps[2],
                                      beam_state.mag_ut[0], beam_state.mag_ut[1], beam_state.mag_ut[2],
                                      beam_state.mag_norm_ut,
                                      beam_state.pressure_hpa);
                    } else {
                        Serial.printf("[10DOF] READ ERR (acc:%s mag:%s press:%s fails:%u)\n",
                                      beam_state.accel_gyro_valid ? "OK" : "ERR",
                                      beam_state.mag_valid        ? "OK" : "ERR",
                                      beam_state.press_valid      ? "OK" : "ERR",
                                      (unsigned)beam_state.consec_fails);
                    }
                }
            }
        }

        // Auto-shutdown on battery inactivity timeout (motion/logger-safe + 30s warning modal)
        static uint32_t s_battery_since_ms = 0;
        static uint32_t s_last_touch_seen_ms = 0;
        static uint32_t s_last_poweroff_attempt_ms = 0;
        static uint8_t  s_last_warn_sec = 0;
        uint32_t touch_act_ms = rover_get_last_user_activity_ms();
        if (touch_act_ms != s_last_touch_seen_ms) {
            s_last_touch_seen_ms = touch_act_ms;
            s_battery_since_ms = now; // User interacted: reset timeout window
            s_last_warn_sec = 0;
        }

        bool motion_active = (ui_state.speed_m_s >= 0.30f) ||
                             (last_imu_motion_ms != 0 && (now - last_imu_motion_ms) < 3000U) ||
                             (last_gnss_motion_ms != 0 && (now - last_gnss_motion_ms) < 5000U);

        const char *inhibit_reason = NULL;
        if (current_settings.auto_shutdown_battery_min == 0) {
            inhibit_reason = "Disabled";
        } else if (pwr.vbus_good) {
            inhibit_reason = "USB Power";
        } else if (is_wifi_active() || is_wifi_starting()) {
            inhibit_reason = "Wi-Fi ON";
        } else if (ui_state.survey_in_progress) {
            inhibit_reason = "Survey";
        } else if (route_logger_is_active()) {
            inhibit_reason = "REC ON";
        } else if (nmea_logger_is_active()) {
            inhibit_reason = "NMEA Log";
        } else if (rtcm_logger_is_active()) {
            inhibit_reason = "RTCM Log";
        } else if (motion_active) {
            inhibit_reason = "Motion";
        }

        if (inhibit_reason != NULL) {
            s_battery_since_ms = now; // Inhibit active: hold timer reset
            s_last_warn_sec = 0;
            s_auto_off_idle_s = 0;
            s_auto_off_countdown_s = 0;
            s_auto_off_inhibited = true;
            strncpy(s_auto_off_reason, inhibit_reason, sizeof(s_auto_off_reason) - 1);
            s_auto_off_reason[sizeof(s_auto_off_reason) - 1] = '\0';
        } else {
            if (s_battery_since_ms == 0) s_battery_since_ms = now;
            s_auto_off_inhibited = false;
            s_auto_off_reason[0] = '\0';

            uint32_t timeout_ms    = (uint32_t)current_settings.auto_shutdown_battery_min * 60000UL;
            uint32_t warn_start_ms = (timeout_ms > 30000UL) ? (timeout_ms - 30000UL) : 0UL;
            uint32_t idle_ms       = now - s_battery_since_ms;
            s_auto_off_idle_s      = idle_ms / 1000UL;

            if ((idle_ms >= timeout_ms) &&
                (s_last_poweroff_attempt_ms == 0 || (now - s_last_poweroff_attempt_ms >= 30000U))) {
                s_last_poweroff_attempt_ms = (now == 0) ? 1 : now;
                s_auto_off_countdown_s = 0;
                Serial.printf("[POWER] Battery inactivity timeout (%u min) expired -> Flushing loggers & shutting down.\n",
                              current_settings.auto_shutdown_battery_min);
                if (route_logger_is_active()) route_logger_stop();
                if (nmea_logger_is_active())  nmea_logger_stop();
                if (rtcm_logger_is_active())  rtcm_logger_stop();
                if (settings_dirty) {
                    settings_dirty = false;
                    save_settings();
                }
                if (serial_logger_is_active()) serial_logger_stop();
                play_rover_beep(800, 300);
                vTaskDelay(pdMS_TO_TICKS(350));
                axp2101_bsp_power_off();
                s_battery_since_ms = millis(); // Prevent repeated beep/flash loop if PMIC I2C power-off fails
                s_last_warn_sec = 0;
            } else if (idle_ms >= warn_start_ms) {
                uint32_t rem_s = (timeout_ms - idle_ms + 999UL) / 1000UL;
                if (rem_s > 30UL) rem_s = 30UL;
                if (rem_s == 0UL) rem_s = 1UL;
                s_auto_off_countdown_s = (uint8_t)rem_s;
                if (s_auto_off_countdown_s != s_last_warn_sec) {
                    s_last_warn_sec = s_auto_off_countdown_s;
                    if (s_last_warn_sec == 30 || s_last_warn_sec == 20 || s_last_warn_sec == 10 ||
                        (s_last_warn_sec >= 1 && s_last_warn_sec <= 5)) {
                        play_rover_beep((s_last_warn_sec <= 5) ? 1600 : 1200,
                                        (s_last_warn_sec <= 5) ? 80 : 50);
                    }
                }
            } else {
                s_auto_off_countdown_s = 0;
                s_last_warn_sec = 0;
            }
        }

        // 4. Receive latest GNSS/Nav state from Core 0, attach Core 1 IMU/PMIC data, and update UI
        // (Note: Dirty settings persistence runs in background loop() and audio beeps run in task_audio_beep
        //  so UI rendering and IMU updates are never stalled by flash/SD writes or audio synthesis.)
        if (xQueueReceive(roverStateQueue, &ui_state, pdMS_TO_TICKS(33)) == pdPASS) {
            // GNSS tip displacement anchor (>= 1.5 m) keeps Auto-Off reset during slow canopy walking
            if (ui_state.fix_type != GNSS_FIX_NONE && fabs(ui_state.tip_lat) > 0.0001) {
                if (!motion_anchor_valid) {
                    motion_anchor_lat   = ui_state.tip_lat;
                    motion_anchor_lon   = ui_state.tip_lon;
                    motion_anchor_valid = true;
                } else {
                    double disp_m = 0.0;
                    nav_compute_target_guidance(motion_anchor_lat, motion_anchor_lon, 0.0f,
                                                ui_state.tip_lat, ui_state.tip_lon,
                                                &disp_m, NULL, NULL, NULL, NULL);
                    if (disp_m >= 1.5) {
                        motion_anchor_lat   = ui_state.tip_lat;
                        motion_anchor_lon   = ui_state.tip_lon;
                        last_gnss_motion_ms = (now == 0) ? 1 : now;
                    }
                }
            }

            ui_state.imu_pitch_deg     = cached_imu_pitch;
            ui_state.imu_roll_deg      = cached_imu_roll;
            ui_state.pole_bubble_x_deg = cached_bubble_x;
            ui_state.pole_bubble_y_deg = cached_bubble_y;
            ui_state.display_pose      = cached_pose;
            ui_state.imu_acc[0]        = cached_acc[0];
            ui_state.imu_acc[1]        = cached_acc[1];
            ui_state.imu_acc[2]        = cached_acc[2];
            ui_state.imu_gyr[0]        = cached_gyr[0];
            ui_state.imu_gyr[1]        = cached_gyr[1];
            ui_state.imu_gyr[2]        = cached_gyr[2];
            ui_state.battery_voltage     = pwr.battery_voltage_v;
            ui_state.battery_percent     = pwr.battery_percent;
            ui_state.is_charging         = pwr.is_charging;
            ui_state.vbus_present        = pwr.vbus_good;
            ui_state.vbus_voltage        = pwr.vbus_voltage_v;
            ui_state.auto_off_idle_s     = s_auto_off_idle_s;
            ui_state.auto_off_countdown_s = s_auto_off_countdown_s;
            ui_state.auto_off_inhibited  = s_auto_off_inhibited;
            strncpy(ui_state.auto_off_reason, s_auto_off_reason, sizeof(ui_state.auto_off_reason) - 1);
            ui_state.auto_off_reason[sizeof(ui_state.auto_off_reason) - 1] = '\0';

            // Attach MMC5983MA Magnetometer Telemetry
            ui_state.mag_present         = cached_mag.present;
            ui_state.mag_cal_valid       = current_settings.mag_cal_valid;
            ui_state.mag_uT[0]           = cached_mag.mag_uT[0];
            ui_state.mag_uT[1]           = cached_mag.mag_uT[1];
            ui_state.mag_uT[2]           = cached_mag.mag_uT[2];
            ui_state.mag_total_uT        = cached_mag.total_uT;
            ui_state.mag_tare_offset_deg = current_settings.mag_true_north_offset_deg;
            ui_state.mag_die_temp_c      = cached_mag.die_temp_c;
            ui_state.mag_set_reset_count = cached_mag.set_reset_count;

            float pole_tilt_deg = sqrtf(cached_bubble_x * cached_bubble_x + cached_bubble_y * cached_bubble_y);
            ui_state.pole_tilt_deg = pole_tilt_deg;

            // Phase 5 & Phase 8: Deep-Woods Heading Fallback (Kinematic Displacement + 9-DOF Compass + Gyro Yaw)
            nav_heading_9dof_update(ui_state.tip_lat,
                                    ui_state.tip_lon,
                                    cached_gyr[2],
                                    pole_tilt_deg,
                                    cached_mag.present,
                                    cached_mag.valid,
                                    cached_mag.raw_mag_heading_deg,
                                    cached_mag.total_uT,
                                    now,
                                    (ui_state.heading_valid && (ui_state.heading_source == HDG_SRC_DUAL_FIX || ui_state.heading_source == HDG_SRC_DUAL_FLT)),
                                    ui_state.heading_is_fixed,
                                    ui_state.true_heading_deg,
                                    &ui_state.heading_valid,
                                    &ui_state.true_heading_deg,
                                    &ui_state.heading_source,
                                    &ui_state.displacement_dist_m,
                                    &ui_state.mag_heading_deg,
                                    &ui_state.mag_anomaly);
            ui_state.mag_cal_valid       = current_settings.mag_cal_valid;
            ui_state.mag_tare_offset_deg = current_settings.mag_true_north_offset_deg;

            if (ui_state.has_active_target && ui_state.heading_valid) {
                float rel = ui_state.bearing_to_target_deg - ui_state.true_heading_deg;
                while (rel < -180.0f) rel += 360.0f;
                while (rel > 180.0f)  rel -= 360.0f;
                ui_state.rel_bearing_deg = rel;
            }

            // Unified Mode 1 Hysteresis (< 4.8 m Bullseye entry, > 5.3 m Line-Up exit)
            static bool    s_mode1_bullseye   = false;
            static int16_t s_mode1_last_b_idx = -1;
            bool           mode1_entered_bullseye = false;
            bool           has_mode1_line         = false;
            NavLineStakeout_t mode1_stakeout      = {};
            double            mode1_dist_b_m      = 0.0;

            if (rover_get_tile0_nav_mode() == 1) {
                WaypointRecord_t wp_a, wp_b;
                int16_t idx_a = -1, idx_b = -1;
                waypoint_line_stakeout_get_points(&wp_a, &wp_b, &idx_a, &idx_b);
                if (idx_a >= 0 && idx_b >= 0) {
                    has_mode1_line = true;
                    nav_compute_line_stakeout(ui_state.tip_lat, ui_state.tip_lon,
                                              wp_a.latitude, wp_a.longitude,
                                              wp_b.latitude, wp_b.longitude,
                                              &mode1_stakeout);
                    nav_compute_target_guidance(ui_state.tip_lat, ui_state.tip_lon, 0.0f,
                                                wp_b.latitude, wp_b.longitude,
                                                &mode1_dist_b_m, NULL, NULL, NULL, NULL);
                    if (idx_b != s_mode1_last_b_idx) {
                        s_mode1_last_b_idx = idx_b;
                        s_mode1_bullseye   = (mode1_dist_b_m < 4.8);
                    } else if (!s_mode1_bullseye && mode1_dist_b_m < 4.8) {
                        s_mode1_bullseye       = true;
                        mode1_entered_bullseye = true;
                    } else if (s_mode1_bullseye && mode1_dist_b_m > 5.3) {
                        s_mode1_bullseye = false;
                    }
                } else {
                    s_mode1_bullseye   = false;
                    s_mode1_last_b_idx = -1;
                }
            } else {
                s_mode1_bullseye   = false;
                s_mode1_last_b_idx = -1;
            }
            ui_state.mode1_bullseye_active = s_mode1_bullseye;

            // Feed fully augmented & fused state (including Battery, IMU, and Woods Fallback Heading) to E220 Uplink Telemetry
            e220_feed_telemetry_state(&ui_state);

            update_rover_ui(&ui_state);
            route_logger_log_point(&ui_state);

            // 7. Proximity & Directional Line-Follow Audio Synthesis (Geiger Counter or Standard Beeps)
            if (current_settings.audio_nav_mode != AUDIO_NAV_MUTE &&
                !current_settings.is_muted && current_settings.volume > 0 &&
                ui_state.has_active_target &&
                (ui_state.fix_type == GNSS_FIX_RTK_FIXED || ui_state.fix_type == GNSS_FIX_RTK_FLOAT || ui_state.fix_type == GNSS_FIX_DGPS) &&
                (ui_state.distance_to_target_m > 0.0 || has_mode1_line)) {

                static uint32_t s_last_audio_ms = 0;
                static int16_t  s_last_audio_target_idx = -1;
                if (ui_state.active_target_idx != s_last_audio_target_idx) {
                    s_last_audio_target_idx = ui_state.active_target_idx;
                    s_last_audio_ms = 0;
                }

                if (has_mode1_line && !s_mode1_bullseye) {
                    // Signed Directional Left/Right Line-Follow Audio (faster closer to line)
                    double abs_off = fabs(mode1_stakeout.offset_m);
                    if (abs_off <= 5.0) {
                        uint32_t interval_ms = 1500;
                        if (abs_off <= 0.05)      interval_ms = 400;
                        else if (abs_off <= 0.25) interval_ms = 250;
                        else if (abs_off <= 0.75) interval_ms = 450;
                        else if (abs_off <= 1.50) interval_ms = 750;
                        else if (abs_off <= 3.00) interval_ms = 1100;
                        else                      interval_ms = 1500;

                        if (current_settings.audio_nav_mode == AUDIO_NAV_GEIGER) {
                            interval_ms = (interval_ms * 7U) / 10U;
                        }

                        uint32_t now_ms = millis();
                        if (now_ms - s_last_audio_ms >= interval_ms) {
                            s_last_audio_ms = now_ms;
                            if (abs_off <= 0.05) {
                                // On-Line: crisp steady high pip
                                play_rover_beep(1800, 30);
                            } else if (mode1_stakeout.offset_m < -0.05) {
                                // Left of A->B: rising 2-tone chirp -> steer RIGHT
                                play_rover_dual_beep(660, 30, 25, 1100, 40);
                            } else {
                                // Right of A->B: falling 2-tone chirp -> steer LEFT
                                play_rover_dual_beep(1100, 30, 25, 660, 40);
                            }
                        }
                    }
                } else {
                    if (has_mode1_line && mode1_entered_bullseye) {
                        // Acquisition chime on < 4.8 m Mode 1 handoff into Bullseye proximity audio
                        play_rover_dual_beep(1400, 45, 30, 2000, 70);
                        s_last_audio_ms = millis();
                    }

                    double d = has_mode1_line ? mode1_dist_b_m : ui_state.distance_to_target_m;
                    if (d > 0.0) {
                        if (current_settings.audio_nav_mode == AUDIO_NAV_GEIGER) {
                            if (d <= 2.0) {
                                uint32_t interval_ms = 1000;
                                uint16_t freq_hz = 900;
                                uint16_t dur_ms = 15;

                                if (d <= 0.02) {          // Bullseye (<= 2cm / 0.8 in)
                                    interval_ms = 80;
                                    freq_hz     = 2400;
                                    dur_ms      = 35;
                                } else if (d <= 0.10) {   // 2 .. 10 cm
                                    interval_ms = 90;
                                    freq_hz     = 2000;
                                    dur_ms      = 10;
                                } else if (d <= 0.30) {   // 10 .. 30 cm
                                    interval_ms = 160;
                                    freq_hz     = 1750;
                                    dur_ms      = 12;
                                } else if (d <= 0.70) {   // 30 .. 70 cm
                                    interval_ms = 320;
                                    freq_hz     = 1400;
                                    dur_ms      = 15;
                                } else if (d <= 1.20) {   // 70 .. 120 cm
                                    interval_ms = 600;
                                    freq_hz     = 1100;
                                    dur_ms      = 18;
                                } else {                  // 1.2 .. 2.0 m
                                    interval_ms = 1000;
                                    freq_hz     = 900;
                                    dur_ms      = 20;
                                }

                                uint32_t now_ms = millis();
                                if (now_ms - s_last_audio_ms >= interval_ms) {
                                    s_last_audio_ms = now_ms;
                                    play_rover_beep(freq_hz, dur_ms);
                                }
                            }
                        } else if (current_settings.audio_nav_mode == AUDIO_NAV_BEEPS) {
                            if (d <= 5.0) {
                                uint32_t interval_ms = 1200;
                                uint16_t freq_hz = 1200;
                                uint16_t dur_ms = 50;

                                if (d <= 0.10) {          // Very close (<= 10 cm)
                                    interval_ms = 200;
                                    freq_hz     = 2000;
                                    dur_ms      = 40;
                                } else if (d <= 0.50) {   // 10 .. 50 cm
                                    interval_ms = 400;
                                    freq_hz     = 1600;
                                    dur_ms      = 45;
                                } else if (d <= 2.0) {    // 0.5 .. 2.0 m
                                    interval_ms = 800;
                                    freq_hz     = 1400;
                                    dur_ms      = 50;
                                } else {                  // 2.0 .. 5.0 m
                                    interval_ms = 1500;
                                    freq_hz     = 1200;
                                    dur_ms      = 50;
                                }

                                uint32_t now_ms = millis();
                                if (now_ms - s_last_audio_ms >= interval_ms) {
                                    s_last_audio_ms = now_ms;
                                    play_rover_beep(freq_hz, dur_ms);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

/**
 * @brief Initializes DFRobot SEN0696 (10DOF IMU) on I2C0 and logs boot presence + initial readout.
 */
static void run_beam_10dof_boot_check(void)
{
    sen0696_state_t report;
    (void)sen0696_init(ROVER_I2C_ADDR_BEAM_IMU, ROVER_I2C_ADDR_BEAM_IMU_MAX, &report);

    if (report.i2c_addr == 0U) {
        Serial.printf("[10DOF] DFRobot SEN0696 (0x%02X..0x%02X) not detected on I2C0 -- skipping.\n",
                      (unsigned)ROVER_I2C_ADDR_BEAM_IMU,
                      (unsigned)ROVER_I2C_ADDR_BEAM_IMU_MAX);
        return;
    }

    if (!report.id_valid) {
        Serial.printf("[10DOF] Device at 0x%02X failed SEN0696 ID check (VID=0x%04X PID=0x%04X FW=0x%04X).\n",
                      (unsigned)report.i2c_addr,
                      (unsigned)report.vid,
                      (unsigned)report.pid,
                      (unsigned)report.fw_version);
        return;
    }

    Serial.printf("[10DOF] DFRobot SEN0696 found @ 0x%02X | VID=0x%04X%s PID=0x%04X FW=0x%04X | Cfg:%s (1 Hz log enabled)\n",
                  (unsigned)report.i2c_addr,
                  (unsigned)report.vid,
                  (report.vid == SEN0696_EXPECTED_VID) ? "" : "(unexpected)",
                  (unsigned)report.pid,
                  (unsigned)report.fw_version,
                  report.configured ? "OK" : "ERR");

    if (report.accel_gyro_valid) {
        float acc_norm = sqrtf((report.accel_g[0] * report.accel_g[0]) +
                               (report.accel_g[1] * report.accel_g[1]) +
                               (report.accel_g[2] * report.accel_g[2]));
        Serial.printf("[10DOF] Accel: X=%.3fg Y=%.3fg Z=%.3fg (|a|=%.3fg) | Gyro: X=%.2fdps Y=%.2fdps Z=%.2fdps\n",
                      report.accel_g[0], report.accel_g[1], report.accel_g[2], acc_norm,
                      report.gyro_dps[0], report.gyro_dps[1], report.gyro_dps[2]);
    } else {
        Serial.println("[10DOF] Accel/Gyro: READ_ERR");
    }

    if (report.mag_valid) {
        Serial.printf("[10DOF] Mag:   X=%.2fuT Y=%.2fuT Z=%.2fuT (|B|=%.2fuT)\n",
                      report.mag_ut[0], report.mag_ut[1], report.mag_ut[2], report.mag_norm_ut);
    } else {
        Serial.println("[10DOF] Mag: READ_ERR");
    }

    if (report.press_valid) {
        Serial.printf("[10DOF] Press: %.2f hPa (%.0f Pa, BaroAlt=%.1f m) | Temp: N/A (not in SEN0696 I2C bridge reg map)\n",
                      report.pressure_hpa, report.pressure_pa, report.altitude_m);
    } else {
        Serial.println("[10DOF] Press: READ_ERR | Temp: N/A (not in SEN0696 I2C bridge reg map)");
    }
}

// =========================================================================
// Arduino Setup & Loop
// =========================================================================
void setup()
{
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0); // Critical: non-blocking USB CDC writes so app never stalls when USB disconnected
    pinMode(ROVER_PIN_BOOT_BTN, INPUT_PULLUP);

    // Wait for native USB Serial to connect, with a 3 second timeout 
    // so the wand still boots instantly on battery power without a PC!
    unsigned long start_ms = millis();
    while (!Serial && (millis() - start_ms) < 3000) {
        delay(10);
    }

    if (Serial) {
        delay(500); // Give the Arduino IDE Serial Monitor a half-second to fully render
    }

    Serial.printf("\n====================================================\n");
    Serial.printf("   RTK Rover %s -- Power-Up Diagnostics\n", ROVER_FIRMWARE_VERSION);
    Serial.printf("   Reset Reason: %d\n", (int)esp_reset_reason());
    Serial.printf("====================================================\n");

    // 1. Initialize Shared I2C0 Bus (IO7=SCL, IO8=SDA) & TCA9554 IO Expander
    // Match Waveshare 01_factory: leave EXIO3 (ROVER_EXIO_SD_CS) as Input/High-Z initially.
    // Retry with full GPIO SCL bus-clear recovery so a slave holding SDA low across a warm/UPS reset never causes a panic loop.
    i2c_master_Init();
    esp_err_t exio_err = ESP_FAIL;
    for (int attempt = 1; attempt <= 5; attempt++) {
        exio_err = esp_io_expander_new_i2c_tca9554(user_i2c_port0_handle, ROVER_I2C_ADDR_TCA9554, &io_expander);
        if (exio_err == ESP_OK && io_expander != NULL) {
            break;
        }
        Serial.printf("[I2C WARN] TCA9554 init attempt %d/5 failed (err=0x%X) -> running GPIO bus-clear & reinit\n",
                      attempt, (unsigned int)exio_err);
        i2c_bsp_reinit_bus();
        delay(30);
    }
    if (io_expander != NULL) {
        esp_io_expander_set_dir(io_expander, ROVER_EXIO_LCD_RST | ROVER_EXIO_PA_CTRL | ROVER_EXIO_CAM_PWDN, IO_EXPANDER_OUTPUT);
    } else {
        Serial.printf("[I2C ERROR] TCA9554 (0x20) unresponsive after 5 bus-clear attempts (err=0x%X); continuing degraded boot\n",
                      (unsigned int)exio_err);
    }

    // 2. Scan Shared I2C0 Bus (IO7=SCL, IO8=SDA) & Initialize Onboard + Optional Peripherals
    uint8_t beam_imu_addr = sen0696_detect_i2c_addr(ROVER_I2C_ADDR_BEAM_IMU, ROVER_I2C_ADDR_BEAM_IMU_MAX);
    Serial.printf("[I2C0 SCAN] TCA9554(0x20):%s | AXP2101(0x34):%s | Touch(0x3B):%s | RTC(0x51):%s | IMU(0x6B):%s | ES8311(0x18):%s | MAG(0x30):%s | 10DOF(0x%02X):%s\n",
                  i2c_bsp_probe_addr(ROVER_I2C_ADDR_TCA9554)         ? "OK" : "--",
                  i2c_bsp_probe_addr(ROVER_I2C_ADDR_AXP2101)         ? "OK" : "--",
                  i2c_bsp_probe_addr(ROVER_I2C_ADDR_TOUCH)           ? "OK" : "--",
                  i2c_bsp_probe_addr(ROVER_I2C_ADDR_RTC)             ? "OK" : "--",
                  i2c_bsp_probe_addr(ROVER_I2C_ADDR_IMU)             ? "OK" : "--",
                  i2c_bsp_probe_addr(ROVER_I2C_ADDR_ES8311)          ? "OK" : "--",
                  i2c_bsp_probe_optional_addr(ROVER_I2C_ADDR_MAG)    ? "OK" : "--",
                  (unsigned)(beam_imu_addr ? beam_imu_addr : ROVER_I2C_ADDR_BEAM_IMU),
                  beam_imu_addr                                      ? "OK" : "--");

    axp2101_bsp_init();
    rtc_bsp_init();
    imu_init();
    mmc5983ma_init();
    run_beam_10dof_boot_check();

    // 3. Mount Internal FFat & MicroSD Card (CLK=11, CMD=10, D0=9)
    if (!FFat.begin(true)) {
        Serial.println("[STORAGE] FFat Mount Failed");
    } else {
        Serial.printf("[STORAGE] FFat Mounted (Total: %lu KB, Used: %lu KB)\n",
                      (unsigned long)(FFat.totalBytes() / 1024),
                      (unsigned long)(FFat.usedBytes() / 1024));
    }

    // Enable ESP32-S3 internal pull-ups on SD lines (matching SDMMC_SLOT_FLAG_INTERNAL_PULLUP in 01_factory)
    gpio_pullup_en((gpio_num_t)ROVER_PIN_SD_SCLK);
    gpio_pullup_en((gpio_num_t)ROVER_PIN_SD_MOSI);
    gpio_pullup_en((gpio_num_t)ROVER_PIN_SD_MISO);
    vTaskDelay(pdMS_TO_TICKS(50));

    // Stage 1: Native 1-bit SD_MMC with EXIO3 High-Z (Input with 100k internal pull-up, matching Waveshare 01_factory)
    if (io_expander != NULL) {
        esp_io_expander_set_dir(io_expander, ROVER_EXIO_SD_CS, IO_EXPANDER_INPUT);
    }
    vTaskDelay(pdMS_TO_TICKS(20));
    SD_MMC.setPins(ROVER_PIN_SD_SCLK, ROVER_PIN_SD_MOSI, ROVER_PIN_SD_MISO);
    if (SD_MMC.begin("/sdcard", true, false, 20000) && SD_MMC.cardType() != CARD_NONE) {
        sd_card_mounted = true;
        sd_card_use_spi = false;
        Serial.println("[STORAGE] Stage 1 OK: SD_MMC 1-bit mounted (EXIO3=Input/High-Z)");
    } else {
        SD_MMC.end();
        // Stage 2: Native 1-bit SD_MMC with EXIO3 driven LOW (in case EXIO3 drives a P-MOSFET high-side SD power switch)
        if (io_expander != NULL) {
            esp_io_expander_set_dir(io_expander, ROVER_EXIO_SD_CS, IO_EXPANDER_OUTPUT);
            esp_io_expander_set_level(io_expander, ROVER_EXIO_SD_CS, 0);
        }
        vTaskDelay(pdMS_TO_TICKS(60));
        gpio_pullup_en((gpio_num_t)ROVER_PIN_SD_SCLK);
        gpio_pullup_en((gpio_num_t)ROVER_PIN_SD_MOSI);
        gpio_pullup_en((gpio_num_t)ROVER_PIN_SD_MISO);
        SD_MMC.setPins(ROVER_PIN_SD_SCLK, ROVER_PIN_SD_MOSI, ROVER_PIN_SD_MISO);
        if (SD_MMC.begin("/sdcard", true, false, 20000) && SD_MMC.cardType() != CARD_NONE) {
            sd_card_mounted = true;
            sd_card_use_spi = false;
            Serial.println("[STORAGE] Stage 2 OK: SD_MMC 1-bit mounted (EXIO3=LOW)");
        } else {
            SD_MMC.end();
            // Stage 3: SPI SD mode with EXIO3 driven LOW (CS asserted via TCA9554 EXIO3)
            SPI.begin(ROVER_PIN_SD_SCLK, ROVER_PIN_SD_MISO, ROVER_PIN_SD_MOSI, -1);
            if (SD.begin(-1, SPI, 4000000, "/sdcard") && SD.cardType() != CARD_NONE) {
                sd_card_mounted = true;
                sd_card_use_spi = true;
                Serial.println("[STORAGE] Stage 3 OK: SD SPI mounted (EXIO3=LOW)");
            } else {
                SD.end();
                Serial.println("[STORAGE] SD Card not detected across Stage 1..3 (using FFat storage fallback).");
            }
        }
    }

    if (sd_card_mounted) {
        uint8_t cardType = sd_card_use_spi ? SD.cardType() : SD_MMC.cardType();
        const char *typeStr = (cardType == CARD_MMC) ? "MMC" :
                              (cardType == CARD_SD)  ? "SDSC" :
                              (cardType == CARD_SDHC) ? "SDHC" : "UNKNOWN";
        uint64_t cardSizeMB  = (sd_card_use_spi ? SD.cardSize()   : SD_MMC.cardSize())   / (1024ULL * 1024ULL);
        uint64_t totalSizeMB = (sd_card_use_spi ? SD.totalBytes() : SD_MMC.totalBytes()) / (1024ULL * 1024ULL);
        uint64_t usedSizeMB  = (sd_card_use_spi ? SD.usedBytes()  : SD_MMC.usedBytes())  / (1024ULL * 1024ULL);
        Serial.printf("[STORAGE] MicroSD Ready (%s, %s): Size=%llu MB (Used=%llu MB / Total=%llu MB)\n",
                      sd_card_use_spi ? "SPI" : "SD_MMC", typeStr, cardSizeMB, usedSizeMB, totalSizeMB);
    }

    // 4. Load Settings & CSV Waypoints
    init_settings_manager();
    if (current_settings.enable_serial_logging) {
        serial_logger_start();
    }
    if (current_settings.enable_nmea_logging) {
        nmea_logger_start();
    }
    if (current_settings.enable_rtcm_logging) {
        rtcm_logger_start();
    }
    waypoint_manager_init();

    // 5. Initialize Display & LVGL 9 UI (320x480 Portrait)
    lvgl_port_init();
    audio_bsp_init();

    // 6. Initialize & Self-Test E220 LoRa (UART0) and UM982 Dual-Antenna GNSS (UART1 & UART2)
    e220_driver_init();
    e220_run_powerup_selftest();
    um982_driver_init();

    Serial.println("====================================================");
    Serial.println("   Setup Complete -- Launching FreeRTOS Tasks");
    Serial.println("====================================================\n");

    // 7. Launch FreeRTOS Tasks
    roverStateQueue = xQueueCreate(1, sizeof(RoverUIData_t));
    beepQueue       = xQueueCreate(12, sizeof(RoverBeep_t));
    xTaskCreatePinnedToCore(task_audio_beep,        "RoverAudio", 3072, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(task_rover_core0,       "RoverCore0", 8192, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(task_display_and_audio, "RoverUI",    8192, NULL, 2, NULL, 1);

    Serial.printf("[MEM] Post-Setup Heap -> Internal Free: %u B (Largest: %u B, DMA: %u B) | PSRAM Free: %u KB\n",
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                  (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_DMA),
                  (unsigned)(heap_caps_get_free_size(MALLOC_CAP_SPIRAM) / 1024));
}

void loop()
{
    // Process active Web Portal requests (File manager, downloads, uploads)
    handle_wifi_server();

    // Flush pending raw NMEA sentences to active storage medium
    if (nmea_logger_is_active()) {
        nmea_logger_flush();
    }

    // Flush pending raw RTCM packets to active storage medium
    if (rtcm_logger_is_active()) {
        rtcm_logger_flush();
    }

    // Flush pending serial monitor logs to active storage medium
    if (serial_logger_is_active()) {
        serial_logger_flush();
    }

    // Persist dirty settings in Priority-1 background task after 5s quiet debounce
    if (settings_dirty) {
        uint32_t dirty_ts = last_settings_change_ms;
        uint32_t now_ms   = millis();
        if ((int32_t)(now_ms - dirty_ts) >= 5000) {
            settings_dirty = false;
            save_settings();
        }
    }

    // =====================================================================
    // Physical BOOT Button (GPIO 0):
    // - Long-Press (>= 1.5s hold): Save Screenshot to SD Card on ALL screens
    // (Short-press logging removed per user request: on-screen buttons used instead)
    // =====================================================================
    static uint32_t btn_press_start = 0;
    static bool     btn_is_pressed   = false;
    static bool     long_press_fired = false;

    if (digitalRead(ROVER_PIN_BOOT_BTN) == LOW) {
        if (!btn_is_pressed) {
            btn_is_pressed   = true;
            long_press_fired = false;
            btn_press_start  = millis();
            rover_note_user_activity();
        } else if (!long_press_fired && (millis() - btn_press_start >= 1500)) {
            long_press_fired = true;
            Serial.println("BOOT Long-Press: Taking Screenshot...");
            play_rover_beep(2000, 60);
            play_rover_beep(2500, 100);
            take_screenshot_to_sd();
        }
    } else {
        btn_is_pressed = false;
    }

    vTaskDelay(pdMS_TO_TICKS(15));
}
