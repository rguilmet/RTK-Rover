#include <Arduino.h>
#include <math.h>
#include <SD.h>
#include <FFat.h>
#include <SPI.h>

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

extern esp_io_expander_handle_t io_expander;
bool sd_card_mounted = false;

static QueueHandle_t roverStateQueue = NULL;
static esp_codec_dev_handle_t playback_handle = NULL;
static volatile uint16_t pending_beep_freq_hz = 0;
static volatile uint16_t pending_beep_dur_ms  = 0;
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
            esp_codec_dev_set_out_vol(playback_handle, (float)current_settings.volume);
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
    pending_beep_freq_hz = freq_hz;
    pending_beep_dur_ms  = duration_ms;
}

extern "C" void trigger_tare_pole_level(void)
{
    pending_tare_level = true;
}

// =========================================================================
// Core 0 Task: Real-Time UART Bridge (E220 <-> UM982 COM2) & GNSS/IMU Fusion
// =========================================================================
void task_rover_core0(void *pvParameters)
{
    (void)pvParameters;
    RoverUIData_t state = {};
    Axp2101Status_t pwr = {};
    uint32_t last_pwr_poll_ms = 0;
    uint32_t last_ui_push_ms  = 0;

    for (;;) {
        // 1. Poll E220 LoRa UART0 (IO41/42) and immediately forward RTCM3 to UM982 COM2 (IO47/48)
        e220_poll_and_forward_rtcm();

        // 2. Poll UM982 COM1 (IO17/18) for NMEA GGA/RMC/GST and Dual-Antenna Heading
        um982_poll_com1();

        uint32_t now = millis();

        // 3. At ~30Hz, fuse GNSS lever-arm kinematics, IMU pose, and active target guidance
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

            // Read QMI8658 IMU for Display Orientation Pose & Pole Bubble Level
            float acc[3] = {0}, gyr[3] = {0};
            int16_t imu_temp = 0;
            imu_read(acc, gyr, &imu_temp);
            imu_compute_pose_and_bubble(acc,
                                        &state.imu_pitch_deg,
                                        &state.imu_roll_deg,
                                        &state.pole_bubble_x_deg,
                                        &state.pole_bubble_y_deg,
                                        &state.display_pose);

            if (pending_tare_level) {
                pending_tare_level = false;
                current_settings.pole_tilt_tare_pitch_deg = state.imu_pitch_deg;
                current_settings.pole_tilt_tare_roll_deg  = state.imu_roll_deg;
                mark_settings_dirty();
                play_rover_beep(1800, 80);
            }

            // Poll AXP2101 PMIC Battery Status every 2 seconds
            if (now - last_pwr_poll_ms >= 2000 || last_pwr_poll_ms == 0) {
                last_pwr_poll_ms = now;
                axp2101_bsp_read(&pwr);
                state.battery_voltage = pwr.battery_voltage_v;
                state.battery_percent = pwr.battery_percent;
                state.is_charging     = pwr.is_charging;
                state.vbus_present    = pwr.vbus_good;
            }

            // Update Multi-Epoch Survey Logger State
            waypoint_update_survey_averaging(&state,
                                             &state.survey_in_progress,
                                             &state.survey_progress_pct,
                                             &state.survey_epochs_collected,
                                             state.last_logged_label);

            if (roverStateQueue) {
                xQueueOverwrite(roverStateQueue, &state);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// =========================================================================
// Core 1 Task: UI Display Update & Audio Alerts
// =========================================================================
void task_display_and_audio(void *pvParameters)
{
    (void)pvParameters;
    RoverUIData_t ui_state = {};

    for (;;) {
        if (settings_dirty && (millis() - last_settings_change_ms > 4000)) {
            save_settings();
            settings_dirty = false;
        }

        if (pending_beep_freq_hz > 0 && pending_beep_dur_ms > 0) {
            uint16_t freq = pending_beep_freq_hz;
            uint16_t dur  = pending_beep_dur_ms;
            pending_beep_freq_hz = 0;
            pending_beep_dur_ms  = 0;

            if (playback_handle && !current_settings.is_muted) {
                int16_t buf[256 * 2];
                uint32_t total_samples = (AUDIO_SAMPLE_RATE * dur) / 1000;
                float phase = 0.0f;
                float phase_inc = 2.0f * (float)M_PI * (float)freq / (float)AUDIO_SAMPLE_RATE;
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

        if (xQueueReceive(roverStateQueue, &ui_state, pdMS_TO_TICKS(33)) == pdPASS) {
            update_rover_ui(&ui_state);
        }
    }
}

// =========================================================================
// Arduino Setup & Loop
// =========================================================================
void setup()
{
    Serial.begin(115200);
    pinMode(ROVER_PIN_BOOT_BTN, INPUT_PULLUP);

    // 1. Initialize Shared I2C0 Bus (IO7=SCL, IO8=SDA) & TCA9554 IO Expander
    i2c_master_Init();
    ESP_ERROR_CHECK(esp_io_expander_new_i2c_tca9554(user_i2c_port0_handle, ROVER_I2C_ADDR_TCA9554, &io_expander));
    ESP_ERROR_CHECK(esp_io_expander_set_dir(io_expander, ROVER_EXIO_LCD_RST | ROVER_EXIO_SD_CS | ROVER_EXIO_PA_CTRL | ROVER_EXIO_CAM_PWDN, IO_EXPANDER_OUTPUT));

    // Brief wait for USB-CDC Serial monitor on boot
    unsigned long start_ms = millis();
    while (!Serial && (millis() - start_ms) < 2000) {
        delay(10);
    }

    Serial.printf("\n=== RTK Rover %s ===\n", ROVER_FIRMWARE_VERSION);

    // 2. Initialize Onboard Peripherals (AXP2101 PMIC, PCF85063 RTC, QMI8658 IMU)
    axp2101_bsp_init();
    rtc_init();
    imu_init();

    // 3. Mount Internal FFat & MicroSD Card (asserting SD_CS LOW via TCA9554 EXIO3)
    if (!FFat.begin(true)) {
        Serial.println("FFat Mount Failed");
    } else {
        Serial.println("FFat Mounted");
    }

    ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander, ROVER_EXIO_SD_CS, 0));
    SPI.begin(ROVER_PIN_SD_SCLK, ROVER_PIN_SD_MISO, ROVER_PIN_SD_MOSI, -1);
    if (!SD.begin(-1, SPI, 4000000)) {
        Serial.println("SD Card not detected (using FFat storage fallback).");
    } else {
        sd_card_mounted = true;
        uint64_t cardSize = SD.cardSize() / (1024 * 1024);
        Serial.printf("SD Card Mounted: %llu MB\n", cardSize);
    }

    // 4. Load Settings & CSV Waypoints
    init_settings_manager();
    waypoint_manager_init();

    // 5. Initialize Display & LVGL 9 UI (320x480 Portrait)
    lvgl_port_init();
    audio_bsp_init();

    // 6. Initialize E220 LoRa & UM982 Dual-Antenna GNSS UARTs
    e220_driver_init();
    um982_driver_init();

    // 7. Launch FreeRTOS Tasks
    roverStateQueue = xQueueCreate(1, sizeof(RoverUIData_t));
    xTaskCreatePinnedToCore(task_rover_core0, "RoverCore0", 8192, NULL, 4, NULL, 0);
    xTaskCreatePinnedToCore(task_display_and_audio, "RoverUI", 8192, NULL, 2, NULL, 1);
}

void loop()
{
    // =====================================================================
    // Physical BOOT Button (GPIO 0) State Machine:
    // - Short-Press (< 1.2s release): Log Waypoint ONLY when on Tile 2 (col == 1)
    // - Long-Press  (>= 1.5s hold):   Save Screenshot to SD Card on ALL screens
    // =====================================================================
    static uint32_t btn_press_start = 0;
    static bool     btn_is_pressed  = false;
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
            vTaskDelay(pdMS_TO_TICKS(80));
            play_rover_beep(2500, 100);
            take_screenshot_to_sd();
        }
    } else {
        if (btn_is_pressed) {
            uint32_t held_ms = millis() - btn_press_start;
            if (!long_press_fired && held_ms >= 40 && held_ms < 1200) {
                // Short press released: check if Tile 2 (Field Survey Logger, col == 1) is active
                if (rover_get_active_tile_index() == 1) {
                    Serial.println("BOOT Short-Press on Tile 2: Triggering Survey Waypoint Log!");
                    trigger_survey_waypoint_log();
                } else {
                    Serial.println("BOOT Short-Press ignored (not on Tile 2 Survey Logger).");
                }
            }
            btn_is_pressed = false;
        }
    }

    vTaskDelay(pdMS_TO_TICKS(15));
}
