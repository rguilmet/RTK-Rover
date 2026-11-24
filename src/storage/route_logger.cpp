#include "route_logger.h"
#include "serial_logger.h"
#include "../../user_config.h"
#include "../settings_manager/settings_manager.h"
#include <Field_Instrument_Core.h>
#include <Arduino.h>
#include <math.h>
#include <stdlib.h>

#define ROUTE_RING_BUF_SIZE 4096U

extern SemaphoreHandle_t fs_mux;

static FicRingLogger_t   s_route_logger = {};
static bool              s_route_inited = false;
static volatile uint32_t s_route_points = 0U;
static uint32_t          s_last_log_ms  = 0U;

static void route_timestamp_cb(char *buf, size_t max_len)
{
    fic_rtc_get_formatted_timestamp(buf, max_len, true);
}

static bool ensure_route_init(void)
{
    if (s_route_inited) {
        s_route_logger.cfg.fs     = &get_active_fs();
        s_route_logger.cfg.fs_mux = fs_mux;
        return true;
    }
    FicRingLoggerCfg_t cfg = {};
    cfg.tag               = "ROUTE LOG";
    cfg.fs                = &get_active_fs();
    cfg.fs_mux            = fs_mux;
    cfg.dir_path          = "/routes";
    cfg.file_prefix       = "route";
    cfg.file_ext          = ".csv";
    cfg.header_line       = "timestamp,firmware_ver,tip_lat,tip_lon,tip_alt_m,raw_lat,raw_lon,raw_alt_m,pos_fix,hdg_fix,sats,slave_sats,hacc_m,vacc_m,hdop,slave_hdop,diff_age_s,base_id,true_heading_deg,baseline_pitch_deg,baseline_len_m,pole_tilt_deg\n";
    cfg.ring_bytes        = ROUTE_RING_BUF_SIZE;
    cfg.flush_interval_ms = 1000U;
    cfg.drop_oldest       = false;
    cfg.timestamp_cb      = route_timestamp_cb;
    cfg.log_cb            = rover_printf;
    s_route_inited = fic_ring_logger_init(&s_route_logger, &cfg);
    return s_route_inited;
}

extern "C" bool route_logger_start(void)
{
    if (!ensure_route_init()) {
        return false;
    }
    if (fic_ring_logger_is_active(&s_route_logger)) {
        return true;
    }
    if (!fic_ring_logger_start(&s_route_logger)) {
        return false;
    }
    s_route_points = 0U;
    s_last_log_ms  = millis();
    return true;
}

extern "C" void route_logger_stop(void)
{
    if (!s_route_inited) {
        return;
    }
    bool was_active = fic_ring_logger_is_active(&s_route_logger);
    uint32_t pts    = s_route_points;
    const char *fn  = fic_ring_logger_get_filename(&s_route_logger);
    fic_ring_logger_stop(&s_route_logger);
    if (was_active) {
        Serial.printf("[ROUTE LOG] Stopped recording. Saved %lu points to %s\n",
                      (unsigned long)pts, fn ? fn : "");
    }
}

extern "C" bool route_logger_is_active(void)
{
    return s_route_inited && fic_ring_logger_is_active(&s_route_logger);
}

extern "C" const char *route_logger_get_current_filename(void)
{
    return s_route_inited ? fic_ring_logger_get_filename(&s_route_logger) : "";
}

extern "C" uint32_t route_logger_get_point_count(void)
{
    return s_route_points;
}

extern "C" void route_logger_log_point(const RoverUIData_t *state)
{
    if (!s_route_inited || !fic_ring_logger_is_active(&s_route_logger) || !state) {
        return;
    }

    // Throttle to 1 Hz (1000 ms)
    uint32_t now = millis();
    if (now - s_last_log_ms < 1000U) {
        return;
    }
    s_last_log_ms = now;

    // Format in-file timestamp with milliseconds (MFS format)
    FicRtcTime_t lt;
    fic_rtc_get_local_datetime(&lt);
    uint16_t ms        = (uint16_t)(now % 1000U);
    int16_t  utc_off   = fic_rtc_get_utc_offset_minutes();
    char     sign      = (utc_off >= 0) ? '+' : '-';
    int      abs_off   = abs((int)utc_off);

    char ts_buf[48];
    snprintf(ts_buf, sizeof(ts_buf), "%04u-%02u-%02u_%02u-%02u-%02u.%03u%c%d",
             lt.year, lt.month, lt.day, lt.hour, lt.minute, lt.second, ms, sign, abs_off);

    const char *pos_fix_str = "NONE";
    switch (state->fix_type) {
        case GNSS_FIX_RTK_FIXED: pos_fix_str = "RTK_FIX"; break;
        case GNSS_FIX_RTK_FLOAT: pos_fix_str = "RTK_FLT"; break;
        case GNSS_FIX_DGPS:      pos_fix_str = "DGPS"; break;
        case GNSS_FIX_SINGLE:    pos_fix_str = "SINGLE"; break;
        default:                 pos_fix_str = "NONE"; break;
    }

    const char *hdg_fix_str = state->heading_valid ? (state->heading_is_fixed ? "HDG_FIX" : "HDG_FLT") : "NONE";

    float pole_tilt = sqrtf(state->pole_bubble_x_deg * state->pole_bubble_x_deg +
                            state->pole_bubble_y_deg * state->pole_bubble_y_deg);

    char row[320];
    int row_len = snprintf(row, sizeof(row),
                           "%s,%s,%.9f,%.9f,%.3f,%.9f,%.9f,%.3f,%s,%s,%u,%u,%.3f,%.3f,%.1f,%.1f,%.1f,%u,%.2f,%.2f,%.3f,%.2f\n",
                           ts_buf,
                           ROVER_FIRMWARE_VERSION,
                           state->tip_lat, state->tip_lon, state->tip_height_m,
                           state->raw_lat, state->raw_lon, state->raw_alt_m,
                           pos_fix_str,
                           hdg_fix_str,
                           (unsigned int)state->num_sats,
                           (unsigned int)state->slave_num_sats,
                           state->h_acc_m,
                           state->v_acc_m,
                           state->hdop,
                           state->slave_hdop,
                           state->diff_age_s,
                           (unsigned int)state->base_station_id,
                           state->heading_valid ? state->true_heading_deg : -1.0f,
                           state->heading_valid ? state->baseline_pitch_deg : 0.0f,
                           state->heading_valid ? state->baseline_length_m : 0.0f,
                           pole_tilt);
    if (row_len <= 0) {
        return;
    }
    size_t n_bytes = ((size_t)row_len < sizeof(row)) ? (size_t)row_len : (sizeof(row) - 1U);
    fic_ring_logger_feed(&s_route_logger, (const uint8_t *)row, n_bytes);
    uint32_t pts = s_route_points + 1U;
    s_route_points = pts;
}

extern "C" void route_logger_flush(void)
{
    if (!s_route_inited) {
        return;
    }
    fic_ring_logger_flush(&s_route_logger);
}
