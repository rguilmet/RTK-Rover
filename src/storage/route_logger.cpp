#include "route_logger.h"
#include "../settings_manager/settings_manager.h"
#include "../rtc_bsp/rtc_bsp.h"
#include <Arduino.h>
#include <FS.h>

static bool s_route_active = false;
static uint32_t s_route_points = 0;
static char s_current_filename[64] = {0};
static File s_route_file;
static uint32_t s_last_log_ms = 0;
static SemaphoreHandle_t s_route_mux = NULL;

extern "C" bool route_logger_start(void)
{
    if (!s_route_mux) {
        s_route_mux = xSemaphoreCreateMutex();
    }
    if (s_route_mux) xSemaphoreTake(s_route_mux, portMAX_DELAY);

    if (s_route_active) {
        if (s_route_mux) xSemaphoreGive(s_route_mux);
        return true;
    }

    if (s_route_file) {
        s_route_file.close();
    }

    // Ensure /routes directory exists
    if (!get_active_fs().exists("/routes")) {
        get_active_fs().mkdir("/routes");
    }

    char ts[64];
    get_formatted_timestamp(ts, sizeof(ts), true);
    snprintf(s_current_filename, sizeof(s_current_filename), "/routes/route_%s.csv", ts);

    s_route_file = get_active_fs().open(s_current_filename, FILE_WRITE);
    if (!s_route_file) {
        // Fallback to flat root filename if /routes directory is not supported
        snprintf(s_current_filename, sizeof(s_current_filename), "/route_%s.csv", ts);
        s_route_file = get_active_fs().open(s_current_filename, FILE_WRITE);
    }
    if (!s_route_file) {
        Serial.printf("[ROUTE LOG] Failed to open %s for write\n", s_current_filename);
        s_route_active = false;
        if (s_route_mux) xSemaphoreGive(s_route_mux);
        return false;
    }

    // CSV Header matching MFS & survey standards
    s_route_file.println("timestamp,tip_lat,tip_lon,tip_alt_m,raw_lat,raw_lon,raw_alt_m,fix_type,sats,hacc_m,true_heading_deg,baseline_pitch_deg,baseline_len_m");
    s_route_file.flush();

    s_route_points = 0;
    s_route_active = true;
    s_last_log_ms = millis();
    Serial.printf("[ROUTE LOG] Started recording to %s\n", s_current_filename);

    if (s_route_mux) xSemaphoreGive(s_route_mux);
    return true;
}

extern "C" void route_logger_stop(void)
{
    if (s_route_mux) xSemaphoreTake(s_route_mux, portMAX_DELAY);
    if (s_route_file) {
        s_route_file.flush();
        s_route_file.close();
    }
    s_route_active = false;
    Serial.printf("[ROUTE LOG] Stopped recording. Saved %lu points to %s\n", (unsigned long)s_route_points, s_current_filename);
    if (s_route_mux) xSemaphoreGive(s_route_mux);
}

extern "C" bool route_logger_is_active(void)
{
    return s_route_active;
}

extern "C" const char* route_logger_get_current_filename(void)
{
    return s_current_filename;
}

extern "C" uint32_t route_logger_get_point_count(void)
{
    return s_route_points;
}

extern "C" void route_logger_log_point(const RoverUIData_t *state)
{
    if (!s_route_active || !state) return;

    // Throttle to 1 Hz (1000 ms)
    uint32_t now = millis();
    if (now - s_last_log_ms < 1000) return;
    s_last_log_ms = now;

    if (s_route_mux) xSemaphoreTake(s_route_mux, portMAX_DELAY);
    if (!s_route_file) {
        if (s_route_mux) xSemaphoreGive(s_route_mux);
        return;
    }

    // Format in-file timestamp with milliseconds (MFS format)
    RtcDateTime_t lt;
    get_local_datetime(&lt);
    uint16_t ms = (uint16_t)(millis() % 1000);
    char sign = (current_settings.minutesOffsetToUTC >= 0) ? '+' : '-';
    int abs_offset = abs(current_settings.minutesOffsetToUTC);

    char ts_buf[48];
    snprintf(ts_buf, sizeof(ts_buf), "%04u-%02u-%02u_%02u-%02u-%02u.%03u%c%d",
             lt.year, lt.month, lt.day, lt.hour, lt.minute, lt.second, ms, sign, abs_offset);

    const char *fix_str = "NONE";
    switch (state->fix_type) {
        case GNSS_FIX_RTK_FIXED: fix_str = "RTK_FIX"; break;
        case GNSS_FIX_RTK_FLOAT: fix_str = "RTK_FLT"; break;
        case GNSS_FIX_DGPS:      fix_str = "DGPS"; break;
        case GNSS_FIX_SINGLE:    fix_str = "SINGLE"; break;
        default:                 fix_str = "NONE"; break;
    }

    s_route_file.printf("%s,%.9f,%.9f,%.3f,%.9f,%.9f,%.3f,%s,%u,%.3f,%.2f,%.2f,%.3f\n",
                        ts_buf,
                        state->tip_lat, state->tip_lon, state->tip_height_m,
                        state->raw_lat, state->raw_lon, state->raw_alt_m,
                        fix_str,
                        (unsigned int)state->num_sats,
                        state->h_acc_m,
                        state->heading_valid ? state->true_heading_deg : -1.0f,
                        state->heading_valid ? state->baseline_pitch_deg : 0.0f,
                        state->heading_valid ? state->baseline_length_m : 0.0f);
    s_route_file.flush();
    s_route_points++;

    if (s_route_mux) xSemaphoreGive(s_route_mux);
}
