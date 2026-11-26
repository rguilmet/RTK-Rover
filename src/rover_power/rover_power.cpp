#include "rover_power.h"
#include "../rover_audio/rover_audio.h"
#include "../lvgl_port/lvgl_port.h"
#include "../settings_manager/settings_manager.h"
#include "../waypoint_manager/waypoint_manager.h"
#include "../storage/route_logger.h"
#include "../storage/nmea_logger.h"
#include "../storage/rtcm_logger.h"
#include "../storage/serial_logger.h"
#include "../ble_bridge/ble_bridge.h"
#include <Arduino.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

static portMUX_TYPE       s_pwr_snapshot_mux        = portMUX_INITIALIZER_UNLOCKED;
static FicAxp2101Status_t s_pwr_snapshot            = {};
static FicAxp2101Status_t s_pwr_cached              = {};
static uint32_t           s_last_pwr_poll_ms        = 0;
static uint32_t           s_auto_off_idle_s         = 0;
static uint8_t            s_auto_off_countdown_s    = 0;
static bool               s_auto_off_inhibited      = false;
static char               s_auto_off_reason[16]     = "";

static uint32_t           s_low_batt_since_ms       = 0;
static uint8_t            s_low_batt_debounce_s     = 0;
static bool               s_low_batt_active         = false;
static bool               s_low_batt_flushed        = false;
static bool               s_low_batt_modal_dismissed = false;
static uint32_t           s_last_low_batt_beep_ms    = 0;
static volatile bool      s_low_batt_flush_pending   = false;
static volatile bool      s_auto_off_poweroff_pending = false;

extern "C" void rover_power_poll(uint32_t now_ms)
{
    uint32_t poll_period_ms = (s_low_batt_since_ms != 0U && !s_low_batt_active) ? 1000UL : 2000UL;
    if (now_ms - s_last_pwr_poll_ms >= poll_period_ms || s_last_pwr_poll_ms == 0) {
        s_last_pwr_poll_ms = (now_ms == 0) ? 1 : now_ms;
        fic_axp2101_read(&s_pwr_cached);

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

        bool on_batt_low = s_pwr_cached.online &&
                           !s_pwr_cached.vbus_good &&
                           !s_pwr_cached.is_charging &&
                           (s_pwr_cached.battery_voltage_v >= ROVER_LOW_BATT_VALID_MIN_V) &&
                           (s_pwr_cached.battery_voltage_v < ROVER_LOW_BATT_THRESH_V);
        bool recovered   = s_pwr_cached.online &&
                           (s_pwr_cached.vbus_good ||
                            s_pwr_cached.is_charging ||
                            (s_pwr_cached.battery_voltage_v >= ROVER_LOW_BATT_CLEAR_V));

        if (!s_low_batt_active) {
            if (on_batt_low) {
                uint32_t elapsed_ms = 0;
                portENTER_CRITICAL(&s_pwr_snapshot_mux);
                if (s_low_batt_since_ms == 0U) {
                    s_low_batt_since_ms = (now_ms == 0U) ? 1U : now_ms;
                }
                elapsed_ms = now_ms - s_low_batt_since_ms;
                uint32_t sec = (elapsed_ms + 999UL) / 1000UL;
                if (sec == 0UL) sec = 1UL;
                if (sec > 5UL)  sec = 5UL;
                s_low_batt_debounce_s = (uint8_t)sec;
                if (elapsed_ms >= ROVER_LOW_BATT_DEBOUNCE_MS) {
                    s_low_batt_active          = true;
                    s_low_batt_flushed         = false;
                    s_low_batt_modal_dismissed = false;
                    s_last_low_batt_beep_ms    = now_ms;
                    s_low_batt_flush_pending   = true;
                }
                portEXIT_CRITICAL(&s_pwr_snapshot_mux);

                if (s_low_batt_active) {
                    Serial.printf("[POWER CRIT] Low battery %.2fV (%u%%) sustained >= 5s -> Flushing SD logs & settings before UVLO!\n",
                                  s_pwr_cached.battery_voltage_v,
                                  (unsigned int)s_pwr_cached.battery_percent);
                    play_rover_dual_beep(1200, 120, 60, 600, 220);
                }
            } else {
                portENTER_CRITICAL(&s_pwr_snapshot_mux);
                s_low_batt_since_ms   = 0;
                s_low_batt_debounce_s = 0;
                portEXIT_CRITICAL(&s_pwr_snapshot_mux);
            }
        } else {
            if (recovered) {
                portENTER_CRITICAL(&s_pwr_snapshot_mux);
                s_low_batt_active          = false;
                s_low_batt_flushed         = false;
                s_low_batt_flush_pending   = false;
                s_low_batt_modal_dismissed = false;
                s_low_batt_since_ms        = 0;
                s_low_batt_debounce_s      = 0;
                portEXIT_CRITICAL(&s_pwr_snapshot_mux);
                Serial.printf("[POWER] Low-battery warning cleared (VBUS:%d VBAT:%.2fV %u%%).\n",
                              s_pwr_cached.vbus_good ? 1 : 0,
                              s_pwr_cached.battery_voltage_v,
                              (unsigned int)s_pwr_cached.battery_percent);
            } else if ((now_ms - s_last_low_batt_beep_ms) >= 30000UL) {
                s_last_low_batt_beep_ms = now_ms;
                play_rover_dual_beep(1000, 90, 50, 550, 160);
            }
        }
    }
}

extern "C" void rover_power_get_snapshot(FicAxp2101Status_t *out)
{
    if (!out) return;
    portENTER_CRITICAL(&s_pwr_snapshot_mux);
    *out = s_pwr_snapshot;
    portEXIT_CRITICAL(&s_pwr_snapshot_mux);
}

extern "C" void rover_power_flush_all_logs(void)
{
    waypoint_flush_pending_survey_csv();
    if (route_logger_is_active()) route_logger_stop();
    if (nmea_logger_is_active())  nmea_logger_stop();
    if (rtcm_logger_is_active())  rtcm_logger_stop();
    if (settings_dirty) {
        settings_dirty = false;
        save_settings();
    }
    if (serial_logger_is_active()) serial_logger_stop();
}

extern "C" void rover_power_service_deferred_flush(void)
{
    bool do_auto_off = false;
    bool do_low_batt = false;
    portENTER_CRITICAL(&s_pwr_snapshot_mux);
    if (s_auto_off_poweroff_pending) {
        s_auto_off_poweroff_pending = false;
        do_auto_off = true;
    }
    if (s_low_batt_flush_pending) {
        s_low_batt_flush_pending = false;
        do_low_batt = true;
    }
    portEXIT_CRITICAL(&s_pwr_snapshot_mux);

    if (do_auto_off) {
        Serial.println("[POWER] Servicing Auto Power-Off SD flush & PMIC shutdown on loop()...");
        rover_power_flush_all_logs();
        vTaskDelay(pdMS_TO_TICKS(350));
        fic_axp2101_power_off();
        return;
    }

    if (!do_low_batt) return;

    rover_power_flush_all_logs();

    portENTER_CRITICAL(&s_pwr_snapshot_mux);
    s_low_batt_flushed = true;
    portEXIT_CRITICAL(&s_pwr_snapshot_mux);
    Serial.println("[POWER CRIT] Low-battery protection flush complete (SD loggers closed, settings saved).");
}

extern "C" bool rover_power_is_low_batt_active(void)
{
    portENTER_CRITICAL(&s_pwr_snapshot_mux);
    bool active = s_low_batt_active;
    portEXIT_CRITICAL(&s_pwr_snapshot_mux);
    return active;
}

extern "C" bool rover_power_should_show_low_batt_modal(void)
{
    portENTER_CRITICAL(&s_pwr_snapshot_mux);
    bool show = s_low_batt_active && !s_low_batt_modal_dismissed;
    portEXIT_CRITICAL(&s_pwr_snapshot_mux);
    return show;
}

extern "C" void rover_power_dismiss_low_batt_modal(void)
{
    portENTER_CRITICAL(&s_pwr_snapshot_mux);
    s_low_batt_modal_dismissed = true;
    portEXIT_CRITICAL(&s_pwr_snapshot_mux);
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
    } else if (fic_web_portal_is_active() || fic_web_portal_is_starting()) {
        inhibit_reason = "Wi-Fi ON";
    } else if (ble_bridge_is_connected()) {
        inhibit_reason = "BLE Link";
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
            Serial.printf("[POWER] Battery inactivity timeout (%u min) expired -> Deferring log flush & shutdown to loop().\n",
                          current_settings.auto_shutdown_battery_min);
            portENTER_CRITICAL(&s_pwr_snapshot_mux);
            s_auto_off_poweroff_pending = true;
            portEXIT_CRITICAL(&s_pwr_snapshot_mux);
            play_rover_beep(800, 300);
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

    portENTER_CRITICAL(&s_pwr_snapshot_mux);
    ui_state->low_batt_warning      = s_low_batt_active;
    ui_state->low_batt_logs_flushed = s_low_batt_flushed;
    ui_state->low_batt_debounce_s   = s_low_batt_debounce_s;
    portEXIT_CRITICAL(&s_pwr_snapshot_mux);
}
