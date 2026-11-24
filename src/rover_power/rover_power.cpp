#include "rover_power.h"
#include "../rover_audio/rover_audio.h"
#include "../lvgl_port/lvgl_port.h"
#include "../web_server/web_server.h"
#include "../settings_manager/settings_manager.h"
#include "../storage/route_logger.h"
#include "../storage/nmea_logger.h"
#include "../storage/rtcm_logger.h"
#include "../storage/serial_logger.h"
#include <Arduino.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static portMUX_TYPE    s_pwr_snapshot_mux     = portMUX_INITIALIZER_UNLOCKED;
static Axp2101Status_t s_pwr_snapshot         = {};
static Axp2101Status_t s_pwr_cached           = {};
static uint32_t        s_last_pwr_poll_ms     = 0;
static uint32_t        s_auto_off_idle_s      = 0;
static uint8_t         s_auto_off_countdown_s = 0;
static bool            s_auto_off_inhibited   = false;
static char            s_auto_off_reason[16]  = "";

extern "C" void rover_power_poll(uint32_t now_ms)
{
    if (now_ms - s_last_pwr_poll_ms >= 2000 || s_last_pwr_poll_ms == 0) {
        s_last_pwr_poll_ms = (now_ms == 0) ? 1 : now_ms;
        axp2101_bsp_read(&s_pwr_cached);

        portENTER_CRITICAL(&s_pwr_snapshot_mux);
        s_pwr_snapshot = s_pwr_cached;
        portEXIT_CRITICAL(&s_pwr_snapshot_mux);

        static bool s_pwr_init = false;
        static bool s_last_vbus_good = false;
        if (s_pwr_init && (s_pwr_cached.vbus_good != s_last_vbus_good)) {
            Serial.printf("[POWER] Transition: %s -> %s | VBUS:%.2fV VBAT:%.2fV (%u%%) Chg:%s\n",
                          s_last_vbus_good ? "USB" : "BATTERY",
                          s_pwr_cached.vbus_good ? "USB" : "BATTERY",
                          s_pwr_cached.vbus_voltage_v,
                          s_pwr_cached.battery_voltage_v,
                          s_pwr_cached.battery_percent,
                          s_pwr_cached.is_charging ? "YES" : "NO");
        }
        s_last_vbus_good = s_pwr_cached.vbus_good;
        s_pwr_init       = true;
    }
}

extern "C" void rover_power_get_snapshot(Axp2101Status_t *out)
{
    if (!out) return;
    portENTER_CRITICAL(&s_pwr_snapshot_mux);
    *out = s_pwr_snapshot;
    portEXIT_CRITICAL(&s_pwr_snapshot_mux);
}

extern "C" void rover_power_flush_all_logs(void)
{
    if (route_logger_is_active()) route_logger_stop();
    if (nmea_logger_is_active())  nmea_logger_stop();
    if (rtcm_logger_is_active())  rtcm_logger_stop();
    if (settings_dirty) {
        settings_dirty = false;
        save_settings();
    }
    if (serial_logger_is_active()) serial_logger_stop();
}

extern "C" void rover_power_update_auto_shutdown(uint32_t now_ms,
                                                 float speed_m_s,
                                                 bool survey_in_progress,
                                                 bool mag_cal_collecting,
                                                 bool imu_motion_active,
                                                 bool gnss_motion_active)
{
    // Auto-shutdown on battery inactivity timeout (motion/logger-safe + 30s warning modal)
    static uint32_t s_battery_since_ms = 0;
    static uint32_t s_last_touch_seen_ms = 0;
    static uint32_t s_last_poweroff_attempt_ms = 0;
    static uint8_t  s_last_warn_sec = 0;
    uint32_t touch_act_ms = rover_get_last_user_activity_ms();
    if (touch_act_ms != s_last_touch_seen_ms) {
        s_last_touch_seen_ms = touch_act_ms;
        s_battery_since_ms = now_ms; // User interacted: reset timeout window
        s_last_warn_sec = 0;
    }

    bool motion_active = (speed_m_s >= 0.30f) || imu_motion_active || gnss_motion_active;

    const char *inhibit_reason = NULL;
    if (current_settings.auto_shutdown_battery_min == 0) {
        inhibit_reason = "Disabled";
    } else if (s_pwr_cached.vbus_good) {
        inhibit_reason = "USB Power";
    } else if (is_wifi_active() || is_wifi_starting()) {
        inhibit_reason = "Wi-Fi ON";
    } else if (survey_in_progress) {
        inhibit_reason = "Survey";
    } else if (mag_cal_collecting) {
        inhibit_reason = "Mag Cal";
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
        s_battery_since_ms = now_ms; // Inhibit active: hold timer reset
        s_last_warn_sec = 0;
        s_auto_off_idle_s = 0;
        s_auto_off_countdown_s = 0;
        s_auto_off_inhibited = true;
        strncpy(s_auto_off_reason, inhibit_reason, sizeof(s_auto_off_reason) - 1);
        s_auto_off_reason[sizeof(s_auto_off_reason) - 1] = '\0';
    } else {
        if (s_battery_since_ms == 0) s_battery_since_ms = now_ms;
        s_auto_off_inhibited = false;
        s_auto_off_reason[0] = '\0';

        uint32_t timeout_ms    = (uint32_t)current_settings.auto_shutdown_battery_min * 60000UL;
        uint32_t warn_start_ms = (timeout_ms > 30000UL) ? (timeout_ms - 30000UL) : 0UL;
        uint32_t idle_ms       = now_ms - s_battery_since_ms;
        s_auto_off_idle_s      = idle_ms / 1000UL;

        if ((idle_ms >= timeout_ms) &&
            (s_last_poweroff_attempt_ms == 0 || (now_ms - s_last_poweroff_attempt_ms >= 30000U))) {
            s_last_poweroff_attempt_ms = (now_ms == 0) ? 1 : now_ms;
            s_auto_off_countdown_s = 0;
            Serial.printf("[POWER] Battery inactivity timeout (%u min) expired -> Flushing loggers & shutting down.\n",
                          current_settings.auto_shutdown_battery_min);
            rover_power_flush_all_logs();
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
}

extern "C" void rover_power_apply_to_ui_state(RoverUIData_t *ui_state)
{
    if (!ui_state) return;
    ui_state->battery_voltage      = s_pwr_cached.battery_voltage_v;
    ui_state->battery_percent      = s_pwr_cached.battery_percent;
    ui_state->is_charging          = s_pwr_cached.is_charging;
    ui_state->vbus_present         = s_pwr_cached.vbus_good;
    ui_state->vbus_voltage         = s_pwr_cached.vbus_voltage_v;
    ui_state->auto_off_idle_s      = s_auto_off_idle_s;
    ui_state->auto_off_countdown_s = s_auto_off_countdown_s;
    ui_state->auto_off_inhibited   = s_auto_off_inhibited;
    strncpy(ui_state->auto_off_reason, s_auto_off_reason, sizeof(ui_state->auto_off_reason) - 1);
    ui_state->auto_off_reason[sizeof(ui_state->auto_off_reason) - 1] = '\0';
}
