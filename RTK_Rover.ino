#include <Arduino.h>
#include <math.h>
#include <SD_MMC.h>
#include <SD.h>
#include <FFat.h>
#include <SPI.h>
#include <driver/gpio.h>

#include "user_config.h"
#include "src/rover_api.h"
#include "src/i2c_bsp/i2c_bsp.h"
#include "src/tca9554/esp_io_expander_tca9554.h"
#include "src/axp2101_bsp/axp2101_bsp.h"
#include "src/rtc_bsp/rtc_bsp.h"
#include "src/imu_bsp/imu_bsp.h"
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
    uint16_t freq_hz;
    uint16_t duration_ms;
} RoverBeep_t;

static QueueHandle_t roverStateQueue = NULL;
static QueueHandle_t beepQueue       = NULL;
static esp_codec_dev_handle_t playback_handle = NULL;
static volatile bool     pending_tare_level   = false;

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
        RoverBeep_t b = { freq_hz, duration_ms };
        xQueueSend(beepQueue, &b, 0);
    }
}

extern "C" void trigger_tare_pole_level(void)
{
    pending_tare_level = true;
}

// =========================================================================
// Core 0 Task: Real-Time UART Bridge (E220 <-> UM982 COM2) & Geodesic Math
// NOTE: Core 0 performs ZERO I2C bus transactions. All I2C0 peripherals live
// exclusively on Core 1 to eliminate cross-core I2C bus contention.
// =========================================================================
void task_rover_core0(void *pvParameters)
{
    (void)pvParameters;
    RoverUIData_t state = {};
    uint32_t last_ui_push_ms   = 0;
    uint32_t last_telem_log_ms = 0;

    for (;;) {
        // 1. Poll E220 LoRa UART0 (IO41/42) and immediately forward RTCM3 to UM982 COM2 (IO47/48)
        e220_poll_and_forward_rtcm();

        // 2. Poll UM982 COM1 (IO17/18) for NMEA GGA/RMC/GST/GSV & Heading, and COM2 (IO47/48) for responses
        um982_poll_com1();
        um982_poll_com2();

        uint32_t now = millis();

        // 3. At ~30Hz, fuse GNSS lever-arm kinematics and active target guidance
        if (now - last_ui_push_ms >= 33) {
            last_ui_push_ms = now;

            um982_get_latest_state(&state);
            um982_get_port_stats(&state.um982_stats);
            e220_get_stats(&state.e220_stats);

            // Compute Pole Centerline Ground Tip Position (compensating Master->Slave 555mm spacing & pole height)
            nav_compute_pole_tip_position(state.raw_lat,
                                          state.raw_lon,
                                          state.raw_alt_m,
                                          state.heading_valid,
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

            // Update Multi-Epoch Survey Logger State
            waypoint_update_survey_averaging(&state,
                                             &state.survey_in_progress,
                                             &state.survey_progress_pct,
                                             &state.survey_epochs_collected,
                                             state.last_logged_label);

            e220_feed_telemetry_state(&state);

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
                    else if (state.heading_source == HDG_SRC_COG) hdg_mode = "COG";
                    else if (state.heading_source == HDG_SRC_HOLD) hdg_mode = "HLD";
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
                Serial.printf("[TELEM] Fix:%d SVs:%u/%u Hdg:%s(%.1f) BL:%.1fcm Spd:%.1fm/s Base:%u Diff:%.1fs | COM1 RX:%luB NMEA:%lu | COM2 TX:%luB | E220 AUX:%d (Edges:%lu) %s RX:%luB Fwd:%luB OK:%lu (1006:%u 1074:%u 1084:%u 1094:%u 1230:%u) Err:%lu (Trunc:%lu) Orph:%luB\n",
                              (int)state.fix_type,
                              state.num_sats,
                              state.sats_in_view,
                              hdg_mode,
                              state.true_heading_deg,
                              state.baseline_length_m * 100.0f,
                              state.speed_m_s,
                              state.base_station_id,
                              state.diff_age_s,
                              (unsigned long)state.um982_stats.com1_rx_bytes,
                              (unsigned long)state.um982_stats.nmea_sentences_ok,
                              (unsigned long)state.um982_stats.com2_tx_bytes,
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

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// =========================================================================
// Core 1 Task: UI Display Update, Onboard I2C Peripherals & Audio Alerts
// =========================================================================
void task_display_and_audio(void *pvParameters)
{
    (void)pvParameters;
    RoverUIData_t   ui_state          = {};
    Axp2101Status_t pwr               = {};
    uint32_t        last_imu_poll_ms  = 0;
    uint32_t        last_imu_log_ms   = 0;
    uint32_t        last_pwr_poll_ms  = 0;
    float           cached_imu_pitch  = 0.0f;
    float           cached_imu_roll   = 0.0f;
    float           cached_bubble_x   = 0.0f;
    float           cached_bubble_y   = 0.0f;
    DisplayPose_t   cached_pose       = DISPLAY_POSE_FLAT;
    float           cached_acc[3]     = {0.0f, 0.0f, 0.0f};
    float           cached_gyr[3]     = {0.0f, 0.0f, 0.0f};

    for (;;) {
        uint32_t now = millis();

        // 1..3: Execute Core 1 I2C transactions (RTC sync, 10 Hz IMU, 0.5 Hz PMIC).
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
        }

        // 4. Persist dirty settings after 4s debounce
        if (settings_dirty && (now - last_settings_change_ms > 4000)) {
            save_settings();
            settings_dirty = false;
        }

        // 5. Audio Beep Synthesis on Core 1 (Queue-driven: reliable, never dropped or overwritten)
        RoverBeep_t beep_req;
        while (beepQueue && xQueueReceive(beepQueue, &beep_req, 0) == pdPASS) {
            if (playback_handle && !current_settings.is_muted && beep_req.freq_hz > 0 && beep_req.duration_ms > 0) {
                int16_t buf[256 * 2];
                uint32_t total_samples = (AUDIO_SAMPLE_RATE * (uint32_t)beep_req.duration_ms) / 1000;
                float phase = 0.0f;
                float phase_inc = 2.0f * (float)M_PI * (float)beep_req.freq_hz / (float)AUDIO_SAMPLE_RATE;
                uint32_t written = 0;
                while (written < total_samples) {
                    for (int i = 0; i < 256; i++) {
                        int16_t s = (phase < (float)M_PI) ? 6000 : -6000;
                        buf[i * 2]     = s;
                        buf[i * 2 + 1] = s;
                        phase += phase_inc;
                        if (phase >= 2.0f * (float)M_PI) phase -= 2.0f * (float)M_PI;
                    }
                    esp_codec_dev_write(playback_handle, buf, sizeof(buf));
                    written += 256;
                }
            }
        }

        // 6. Receive latest GNSS/Nav state from Core 0, attach Core 1 IMU/PMIC data, and update UI
        if (xQueueReceive(roverStateQueue, &ui_state, pdMS_TO_TICKS(33)) == pdPASS) {
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
            ui_state.battery_voltage   = pwr.battery_voltage_v;
            ui_state.battery_percent   = pwr.battery_percent;
            ui_state.is_charging       = pwr.is_charging;
            ui_state.vbus_present      = pwr.vbus_good;

            // Phase 5: Deep-Woods Heading Fallback (Kinematic Displacement Azimuth & Gyro Yaw)
            nav_heading_displacement_update(ui_state.tip_lat,
                                            ui_state.tip_lon,
                                            cached_gyr[2],
                                            now,
                                            (ui_state.heading_valid && (ui_state.heading_source == HDG_SRC_DUAL_FIX || ui_state.heading_source == HDG_SRC_DUAL_FLT)),
                                            ui_state.heading_is_fixed,
                                            ui_state.true_heading_deg,
                                            &ui_state.heading_valid,
                                            &ui_state.true_heading_deg,
                                            &ui_state.heading_source,
                                            &ui_state.displacement_dist_m);

            if (ui_state.has_active_target && ui_state.heading_valid) {
                float rel = ui_state.bearing_to_target_deg - ui_state.true_heading_deg;
                while (rel < -180.0f) rel += 360.0f;
                while (rel > 180.0f)  rel -= 360.0f;
                ui_state.rel_bearing_deg = rel;
            }

            update_rover_ui(&ui_state);
            route_logger_log_point(&ui_state);

            // 7. Proximity Audio Synthesis (Geiger Counter or Standard Beeps)
            if (current_settings.audio_nav_mode != AUDIO_NAV_MUTE &&
                !current_settings.is_muted && current_settings.volume > 0 &&
                ui_state.has_active_target &&
                (ui_state.fix_type == GNSS_FIX_RTK_FIXED || ui_state.fix_type == GNSS_FIX_RTK_FLOAT || ui_state.fix_type == GNSS_FIX_DGPS) &&
                ui_state.distance_to_target_m > 0.0) {

                static uint32_t s_last_audio_ms = 0;
                double d = ui_state.distance_to_target_m;
                if (rover_get_tile0_nav_mode() == 1) {
                    WaypointRecord_t wp_a, wp_b;
                    int16_t idx_a = -1, idx_b = -1;
                    waypoint_line_stakeout_get_points(&wp_a, &wp_b, &idx_a, &idx_b);
                    if (idx_a >= 0 && idx_b >= 0) {
                        NavLineStakeout_t stakeout;
                        nav_compute_line_stakeout(ui_state.tip_lat, ui_state.tip_lon,
                                                  wp_a.latitude, wp_a.longitude,
                                                  wp_b.latitude, wp_b.longitude,
                                                  &stakeout);
                        d = fabs(stakeout.offset_m);
                    }
                }

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

// =========================================================================
// Arduino Setup & Loop
// =========================================================================
void setup()
{
    Serial.begin(115200);
    Serial.setTxTimeoutMs(0); // Critical: non-blocking USB CDC writes so app never stalls when USB disconnected
    pinMode(ROVER_PIN_BOOT_BTN, INPUT_PULLUP);

    // 1. Initialize Shared I2C0 Bus (IO7=SCL, IO8=SDA) & TCA9554 IO Expander
    // Match Waveshare 01_factory: leave EXIO3 (ROVER_EXIO_SD_CS) as Input/High-Z initially
    i2c_master_Init();
    ESP_ERROR_CHECK(esp_io_expander_new_i2c_tca9554(user_i2c_port0_handle, ROVER_I2C_ADDR_TCA9554, &io_expander));
    ESP_ERROR_CHECK(esp_io_expander_set_dir(io_expander, ROVER_EXIO_LCD_RST | ROVER_EXIO_PA_CTRL | ROVER_EXIO_CAM_PWDN, IO_EXPANDER_OUTPUT));

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
    Serial.printf("====================================================\n");

    // 2. Scan Shared I2C0 Bus (IO7=SCL, IO8=SDA) & Initialize Onboard Peripherals
    Serial.printf("[I2C0 SCAN] TCA9554(0x20):%s | AXP2101(0x34):%s | Touch(0x3B):%s | RTC(0x51):%s | IMU(0x6B):%s | ES8311(0x18):%s\n",
                  i2c_bsp_probe_addr(ROVER_I2C_ADDR_TCA9554) ? "OK" : "--",
                  i2c_bsp_probe_addr(ROVER_I2C_ADDR_AXP2101) ? "OK" : "--",
                  i2c_bsp_probe_addr(ROVER_I2C_ADDR_TOUCH)   ? "OK" : "--",
                  i2c_bsp_probe_addr(ROVER_I2C_ADDR_RTC)     ? "OK" : "--",
                  i2c_bsp_probe_addr(ROVER_I2C_ADDR_IMU)     ? "OK" : "--",
                  i2c_bsp_probe_addr(ROVER_I2C_ADDR_ES8311)  ? "OK" : "--");

    axp2101_bsp_init();
    rtc_bsp_init();
    imu_init();

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
    ESP_ERROR_CHECK(esp_io_expander_set_dir(io_expander, ROVER_EXIO_SD_CS, IO_EXPANDER_INPUT));
    vTaskDelay(pdMS_TO_TICKS(20));
    SD_MMC.setPins(ROVER_PIN_SD_SCLK, ROVER_PIN_SD_MOSI, ROVER_PIN_SD_MISO);
    if (SD_MMC.begin("/sdcard", true, false, 20000) && SD_MMC.cardType() != CARD_NONE) {
        sd_card_mounted = true;
        sd_card_use_spi = false;
        Serial.println("[STORAGE] Stage 1 OK: SD_MMC 1-bit mounted (EXIO3=Input/High-Z)");
    } else {
        SD_MMC.end();
        // Stage 2: Native 1-bit SD_MMC with EXIO3 driven LOW (in case EXIO3 drives a P-MOSFET high-side SD power switch)
        ESP_ERROR_CHECK(esp_io_expander_set_dir(io_expander, ROVER_EXIO_SD_CS, IO_EXPANDER_OUTPUT));
        ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander, ROVER_EXIO_SD_CS, 0));
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
    beepQueue       = xQueueCreate(8, sizeof(RoverBeep_t));
    xTaskCreatePinnedToCore(task_rover_core0, "RoverCore0", 8192, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(task_display_and_audio, "RoverUI", 8192, NULL, 2, NULL, 1);
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
