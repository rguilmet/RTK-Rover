#include <Arduino.h>
#include <math.h>
#include <SD_MMC.h>
#include <SD.h>
#include <FFat.h>
#include <SPI.h>
#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <esp_task_wdt.h>

#include "user_config.h"
#include "src/rover_api.h"
#include "src/i2c_bsp/i2c_bsp.h"
#include "src/tca9554/esp_io_expander_tca9554.h"
#include "src/axp2101_bsp/axp2101_bsp.h"
#include "src/rtc_bsp/rtc_bsp.h"
#include "src/beam_imu_bsp/sen0696_driver.h"
#include "src/settings_manager/settings_manager.h"
#include "src/nav_engine/nav_engine.h"
#include "src/waypoint_manager/waypoint_manager.h"
#include "src/e220_driver/e220_driver.h"
#include "src/um982_driver/um982_driver.h"
#include "src/lvgl_port/lvgl_port.h"
#include "src/screenshot/screenshot.h"
#include "src/storage/route_logger.h"
#include "src/storage/nmea_logger.h"
#include "src/storage/rtcm_logger.h"
#include "src/storage/serial_logger.h"
#include "src/web_server/web_server.h"
#include "src/rover_audio/rover_audio.h"
#include "src/rover_power/rover_power.h"
#include "src/rover_sensors/rover_sensors.h"

extern esp_io_expander_handle_t io_expander;
bool sd_card_mounted = false;
bool sd_card_use_spi = false;

static QueueHandle_t roverStateQueue = NULL;

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

static bool rover_touch_log_enabled_cb(void)
{
    return current_settings.enable_touch_logging;
}

static int16_t rover_rtc_utc_offset_cb(void)
{
    return current_settings.minutesOffsetToUTC;
}

static bool rover_rtc_allow_sys_sync_cb(void)
{
    return !gps_time_sync.time_synced;
}

static void rover_init_display(void)
{
    BoardDisplayCfg_t cfg = {};
    cfg.h_res                = ROVER_LCD_H_RES;
    cfg.v_res                = ROVER_LCD_V_RES;
    cfg.dma_buf_bytes        = LVGL_DMA_BUFF_LEN;
    cfg.spiram_buf_bytes     = LVGL_SPIRAM_BUFF_LEN;
    cfg.spi_host             = LCD_HOST;
    cfg.pin_cs               = ROVER_PIN_LCD_CS;
    cfg.pin_pclk             = ROVER_PIN_LCD_PCLK;
    cfg.pin_data0            = ROVER_PIN_LCD_DATA0;
    cfg.pin_data1            = ROVER_PIN_LCD_DATA1;
    cfg.pin_data2            = ROVER_PIN_LCD_DATA2;
    cfg.pin_data3            = ROVER_PIN_LCD_DATA3;
    cfg.pin_bk_light         = ROVER_PIN_BK_LIGHT;
    cfg.pclk_hz              = 40U * 1000U * 1000U;
    cfg.tca9554_addr         = ROVER_I2C_ADDR_TCA9554;
    cfg.exio_input_mask      = (uint32_t)(ROVER_EXIO_TP_INT | ROVER_EXIO_RTC_INT | ROVER_EXIO_AXP_IRQ | ROVER_EXIO_SYS_OUT);
    cfg.exio_output_mask     = (uint32_t)(ROVER_EXIO_LCD_RST | ROVER_EXIO_PA_CTRL | ROVER_EXIO_CAM_PWDN);
    cfg.exio_lcd_rst_mask    = (uint32_t)ROVER_EXIO_LCD_RST;
    cfg.exio_cam_pwdn_mask   = (uint32_t)ROVER_EXIO_CAM_PWDN;
    cfg.tick_period_ms       = LVGL_TICK_PERIOD_MS;
    cfg.task_max_delay_ms    = LVGL_TASK_MAX_DELAY_MS;
    cfg.task_min_delay_ms    = LVGL_TASK_MIN_DELAY_MS;
    cfg.task_stack_size      = LVGL_TASK_STACK_SIZE;
    cfg.task_priority        = LVGL_TASK_PRIORITY;
    cfg.init_backlight_cb    = init_backlight_pwm;
    cfg.create_ui_cb         = create_rover_ui;
    cfg.touch_log_enabled_cb = rover_touch_log_enabled_cb;
    cfg.log_printf_cb        = rover_printf;
    lvgl_port_init_with_cfg(&cfg);
}

// =========================================================================
// Core 0 Task: Real-Time UART Bridge (E220 <-> UM982 COM1) & Geodesic Math
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
            rover_sensors_get_pole_tilt_snapshot(&state.pole_bubble_x_deg,
                                                 &state.pole_bubble_y_deg,
                                                 &state.pole_tilt_deg);

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
                rover_power_get_snapshot(&pwr_snap);
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
    RoverUIData_t ui_state            = {};
    double        motion_anchor_lat   = 0.0;
    double        motion_anchor_lon   = 0.0;
    bool          motion_anchor_valid = false;
    uint32_t      last_gnss_motion_ms = 0;

    for (;;) {
        uint32_t now = millis();

        // 1..3: Execute Core 1 I2C transactions (RTC sync, IMU/Mag/10DOF AHRS, PMIC)
        uint16_t yy; uint8_t mo, dd, hh, mm, ss;
        if (um982_poll_pending_rtc_sync(&yy, &mo, &dd, &hh, &mm, &ss)) {
            i2c_rtc_setTime(yy, mo, dd, hh, mm, ss);
            Serial.printf("RTC auto-synced to GNSS UTC: %04u-%02u-%02u %02u:%02u:%02u\n", yy, mo, dd, hh, mm, ss);
        }

        rover_sensors_poll_core1(now);
        rover_power_poll(now);

        bool imu_motion_active  = rover_sensors_is_imu_motion_recent(now, 3000U);
        bool gnss_motion_active = (last_gnss_motion_ms != 0 && (now - last_gnss_motion_ms) < 5000U);
        rover_power_update_auto_shutdown(now,
                                         ui_state.speed_m_s,
                                         ui_state.survey_in_progress,
                                         rover_sensors_is_mag_cal_collecting(),
                                         imu_motion_active,
                                         gnss_motion_active);

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

            rover_sensors_apply_to_ui_state(now, &ui_state);
            rover_power_apply_to_ui_state(&ui_state);

            if (ui_state.has_active_target && ui_state.heading_valid) {
                float rel = ui_state.bearing_to_target_deg - ui_state.true_heading_deg;
                while (rel < -180.0f) rel += 360.0f;
                while (rel > 180.0f)  rel -= 360.0f;
                ui_state.rel_bearing_deg = rel;
            }

            // Unified Guidance Bullseye Hysteresis (< 4.8 m Bullseye entry, > 5.3 m Triangle exit)
            static bool    s_mode1_bullseye   = false;
            static int16_t s_mode1_last_b_idx = -1;
            bool           mode1_entered_bullseye = false;
            bool           has_mode1_line         = false;
            NavLineStakeout_t mode1_stakeout      = {};
            double            mode1_dist_b_m      = 0.0;

            ActiveGuidanceSegment_t active_seg;
            if (waypoint_guidance_get_segment(ui_state.tip_lat, ui_state.tip_lon, &active_seg)) {
                has_mode1_line = true;
                nav_compute_line_stakeout(ui_state.tip_lat, ui_state.tip_lon,
                                          active_seg.lat_a, active_seg.lon_a,
                                          active_seg.lat_b, active_seg.lon_b,
                                          &mode1_stakeout);
                nav_compute_target_guidance(ui_state.tip_lat, ui_state.tip_lon, 0.0f,
                                            active_seg.lat_b, active_seg.lon_b,
                                            &mode1_dist_b_m, NULL, NULL, NULL, NULL);
                if (active_seg.idx_b != s_mode1_last_b_idx) {
                    s_mode1_last_b_idx = active_seg.idx_b;
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
            ui_state.mode1_bullseye_active = s_mode1_bullseye;

            // Feed fully augmented & fused state (including Battery, IMU, and Woods Fallback Heading) to E220 Uplink Telemetry
            e220_feed_telemetry_state(&ui_state);

            update_rover_ui(&ui_state);
            route_logger_log_point(&ui_state);

            // 7. Proximity & Acoustic Fade Directional Line-Follow Audio Synthesis
            rover_audio_update_nav_guidance(&ui_state,
                                            has_mode1_line,
                                            s_mode1_bullseye,
                                            mode1_entered_bullseye,
                                            mode1_stakeout.offset_m,
                                            mode1_dist_b_m);
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

    // 1. Initialize Shared I2C0 Bus (IO7=SCL, IO8=SDA), Isolated I2C1 Bus (IO47=SDA, IO48=SCL) & TCA9554 IO Expander
    // Match Waveshare 01_factory: leave EXIO3 (ROVER_EXIO_SD_CS) as Input/High-Z initially.
    // Retry with full GPIO SCL bus-clear recovery so a slave holding SDA low across a warm/UPS reset never causes a panic loop.
    i2c_master_Init();
    i2c_bsp_i2c1_init();
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

    // 2. Scan Shared I2C0 (IO7/8) & Isolated I2C1 (IO47/48) Buses & Initialize Peripherals
    uint8_t i2c0_beam_addr = sen0696_detect_i2c_addr_on_bus(0U, ROVER_I2C_ADDR_BEAM_IMU, ROVER_I2C_ADDR_BEAM_IMU_MAX);
    uint8_t i2c1_beam_addr = sen0696_detect_i2c_addr_on_bus(1U, ROVER_I2C_ADDR_BEAM_IMU, ROVER_I2C_ADDR_BEAM_IMU_MAX);
    Serial.printf("[I2C0 SCAN] TCA9554(0x20):%s | AXP2101(0x34):%s | Touch(0x3B):%s | RTC(0x51):%s | IMU(0x6B):%s | ES8311(0x18):%s | MAG(0x30):%s | 10DOF(0x%02X):%s\n",
                  i2c_bsp_probe_addr(ROVER_I2C_ADDR_TCA9554)         ? "OK" : "--",
                  i2c_bsp_probe_addr(ROVER_I2C_ADDR_AXP2101)         ? "OK" : "--",
                  i2c_bsp_probe_addr(ROVER_I2C_ADDR_TOUCH)           ? "OK" : "--",
                  i2c_bsp_probe_addr(ROVER_I2C_ADDR_RTC)             ? "OK" : "--",
                  i2c_bsp_probe_addr(ROVER_I2C_ADDR_IMU)             ? "OK" : "--",
                  i2c_bsp_probe_addr(ROVER_I2C_ADDR_ES8311)          ? "OK" : "--",
                  i2c_bsp_probe_optional_addr(ROVER_I2C_ADDR_MAG)    ? "OK" : "--",
                  (unsigned)(i2c0_beam_addr ? i2c0_beam_addr : ROVER_I2C_ADDR_BEAM_IMU),
                  i2c0_beam_addr                                     ? "OK" : "--");
    Serial.printf("[I2C1 SCAN] (IO47 SDA / IO48 SCL @ 400kHz) | 10DOF(0x%02X):%s\n",
                  (unsigned)(i2c1_beam_addr ? i2c1_beam_addr : ROVER_I2C_ADDR_BEAM_IMU),
                  i2c1_beam_addr                                     ? "OK" : "--");

    axp2101_bsp_init();
    rtc_bsp_set_utc_offset_provider(rover_rtc_utc_offset_cb);
    rtc_bsp_set_sys_clock_sync_guard(rover_rtc_allow_sys_sync_cb);
    rtc_bsp_init();
    rover_sensors_hw_init();

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
    if (SD_MMC.begin("/sdcard", true, false, 20000, 10) && SD_MMC.cardType() != CARD_NONE) {
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
        if (SD_MMC.begin("/sdcard", true, false, 20000, 10) && SD_MMC.cardType() != CARD_NONE) {
            sd_card_mounted = true;
            sd_card_use_spi = false;
            Serial.println("[STORAGE] Stage 2 OK: SD_MMC 1-bit mounted (EXIO3=LOW)");
        } else {
            SD_MMC.end();
            // Stage 3: SPI SD mode with EXIO3 driven LOW (CS asserted via TCA9554 EXIO3)
            SPI.begin(ROVER_PIN_SD_SCLK, ROVER_PIN_SD_MISO, ROVER_PIN_SD_MOSI, -1);
            if (SD.begin(-1, SPI, 4000000, "/sdcard", 10) && SD.cardType() != CARD_NONE) {
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

    // 4. Load Settings, Allocate PSRAM Beam AHRS & CSV Waypoints
    init_settings_manager();
    rover_sensors_ahrs_init();
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
    rover_init_display();
    rover_audio_init();

    // 6. Initialize & Self-Test E220 LoRa (UART0) and UM982 Dual-Antenna GNSS (Single-UART COM1)
    e220_driver_init();
    e220_run_powerup_selftest();
    um982_driver_init();

    Serial.println("====================================================");
    Serial.println("   Setup Complete -- Launching FreeRTOS Tasks");
    Serial.println("====================================================\n");

    // 7. Launch FreeRTOS Tasks
    roverStateQueue = xQueueCreate(1, sizeof(RoverUIData_t));
    rover_audio_start_task();
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

    // Flush pending buffered route CSV points to active storage medium
    if (route_logger_is_active()) {
        route_logger_flush();
    }

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
            play_rover_beep(2000, 40);
            if (take_screenshot_to_sd()) {
                play_rover_beep(2600, 100);
            } else {
                play_rover_beep(400, 120);
                play_rover_beep(300, 180);
            }
        }
    } else {
        btn_is_pressed = false;
    }

    vTaskDelay(pdMS_TO_TICKS(15));
}
