#include "route_logger.h"
#include "../../user_config.h"
#include "../settings_manager/settings_manager.h"
#include "../rtc_bsp/rtc_bsp.h"
#include <math.h>
#include <Arduino.h>
#include <FS.h>
#include "esp_heap_caps.h"
#include <stdlib.h>

#define ROUTE_RING_BUF_SIZE 4096

extern SemaphoreHandle_t fs_mux;

static volatile bool s_route_active = false;
static volatile uint32_t s_route_points = 0;
static char s_current_filename[64] = {0};
static File s_route_file;
static uint32_t s_last_log_ms = 0;
static uint32_t s_last_sd_flush_ms = 0;
static SemaphoreHandle_t s_route_buf_mux = NULL;
static SemaphoreHandle_t s_route_file_mux = NULL;

static uint8_t *s_ring_buf = NULL;
static volatile size_t s_ring_head = 0;
static volatile size_t s_ring_tail = 0;

static bool ensure_route_ring_buf(void)
{
    if (s_ring_buf) return true;
    s_ring_buf = (uint8_t *)heap_caps_malloc(ROUTE_RING_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_ring_buf) {
        s_ring_buf = (uint8_t *)malloc(ROUTE_RING_BUF_SIZE);
    }
    return (s_ring_buf != NULL);
}

extern "C" bool route_logger_start(void)
{
    if (!s_route_buf_mux) {
        s_route_buf_mux = xSemaphoreCreateMutex();
    }
    if (!s_route_file_mux) {
        s_route_file_mux = xSemaphoreCreateMutex();
    }
    if (s_route_file_mux) xSemaphoreTake(s_route_file_mux, portMAX_DELAY);

    if (!ensure_route_ring_buf()) {
        if (s_route_file_mux) xSemaphoreGive(s_route_file_mux);
        return false;
    }

    if (s_route_active) {
        if (s_route_file_mux) xSemaphoreGive(s_route_file_mux);
        return true;
    }

    if (fs_mux) xSemaphoreTake(fs_mux, portMAX_DELAY);

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
        s_route_active = false;
        if (fs_mux) xSemaphoreGive(fs_mux);
        if (s_route_file_mux) xSemaphoreGive(s_route_file_mux);
        Serial.printf("[ROUTE LOG] Failed to open %s for write\n", s_current_filename);
        return false;
    }

    // CSV Header matching MFS & survey standards
    s_route_file.println("timestamp,firmware_ver,tip_lat,tip_lon,tip_alt_m,raw_lat,raw_lon,raw_alt_m,pos_fix,hdg_fix,sats,slave_sats,hacc_m,vacc_m,hdop,slave_hdop,diff_age_s,base_id,true_heading_deg,baseline_pitch_deg,baseline_len_m,pole_tilt_deg");
    s_route_file.flush();

    if (s_route_buf_mux) xSemaphoreTake(s_route_buf_mux, portMAX_DELAY);
    s_ring_head = 0;
    s_ring_tail = 0;
    s_route_points = 0;
    uint32_t now_ms = millis();
    s_last_log_ms = now_ms;
    s_last_sd_flush_ms = now_ms;
    if (s_route_buf_mux) xSemaphoreGive(s_route_buf_mux);

    s_route_active = true;

    if (fs_mux) xSemaphoreGive(fs_mux);
    if (s_route_file_mux) xSemaphoreGive(s_route_file_mux);
    Serial.printf("[ROUTE LOG] Started recording to %s\n", s_current_filename);
    return true;
}

// Drain ring buffer to SD/FFat WITHOUT holding s_route_buf_mux across slow VFS write/flush calls.
// Caller must already hold s_route_file_mux and (if initialized) fs_mux.
static void route_logger_drain_to_file(bool force_sync)
{
    if (!s_route_file || !s_ring_buf) return;

    uint8_t write_buf[1024];
    bool wrote_any = false;

    for (;;) {
        size_t bytes_to_write = 0;
        if (s_route_buf_mux && xSemaphoreTake(s_route_buf_mux, pdMS_TO_TICKS(20)) == pdTRUE) {
            size_t tail = s_ring_tail;
            size_t head = s_ring_head;
            while (tail != head && bytes_to_write < sizeof(write_buf)) {
                write_buf[bytes_to_write++] = s_ring_buf[tail];
                tail = (tail + 1U) % ROUTE_RING_BUF_SIZE;
            }
            s_ring_tail = tail;
            xSemaphoreGive(s_route_buf_mux);
        } else {
            break;
        }

        if (bytes_to_write == 0) {
            break;
        }

        s_route_file.write(write_buf, bytes_to_write);
        wrote_any = true;
    }

    uint32_t now_ms = millis();
    if (force_sync || (wrote_any && (now_ms - s_last_sd_flush_ms >= 1000))) {
        s_last_sd_flush_ms = now_ms;
        s_route_file.flush();
    }
}

extern "C" void route_logger_stop(void)
{
    if (!s_route_file_mux) return;
    xSemaphoreTake(s_route_file_mux, portMAX_DELAY);
    bool stopped = false;
    uint32_t pts = 0;
    if (s_route_active) {
        if (fs_mux) xSemaphoreTake(fs_mux, portMAX_DELAY);
        route_logger_drain_to_file(true);
        if (s_route_file) {
            s_route_file.flush();
            s_route_file.close();
        }
        s_route_active = false;
        pts = s_route_points;
        stopped = true;
        if (fs_mux) xSemaphoreGive(fs_mux);
    }
    xSemaphoreGive(s_route_file_mux);
    if (stopped) {
        Serial.printf("[ROUTE LOG] Stopped recording. Saved %lu points to %s\n", (unsigned long)pts, s_current_filename);
    }
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
    if (!s_route_active || !s_ring_buf || !state) return;

    // Throttle to 1 Hz (1000 ms)
    uint32_t now = millis();
    if (now - s_last_log_ms < 1000) return;
    s_last_log_ms = now;

    // Format in-file timestamp with milliseconds (MFS format)
    RtcDateTime_t lt;
    get_local_datetime(&lt);
    uint16_t ms = (uint16_t)(now % 1000U);
    int16_t utc_off = rtc_bsp_get_utc_offset_minutes();
    char sign = (utc_off >= 0) ? '+' : '-';
    int abs_offset = abs((int)utc_off);

    char ts_buf[48];
    snprintf(ts_buf, sizeof(ts_buf), "%04u-%02u-%02u_%02u-%02u-%02u.%03u%c%d",
             lt.year, lt.month, lt.day, lt.hour, lt.minute, lt.second, ms, sign, abs_offset);

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
    if (row_len <= 0) return;
    size_t n_bytes = ((size_t)row_len < sizeof(row)) ? (size_t)row_len : (sizeof(row) - 1U);

    if (!s_route_buf_mux) {
        s_route_buf_mux = xSemaphoreCreateMutex();
    }

    if (s_route_buf_mux && xSemaphoreTake(s_route_buf_mux, pdMS_TO_TICKS(5)) == pdTRUE) {
        size_t head = s_ring_head;
        size_t tail = s_ring_tail;
        size_t used = (head >= tail) ? (head - tail) : (ROUTE_RING_BUF_SIZE - tail + head);
        size_t free_bytes = (ROUTE_RING_BUF_SIZE - 1U) - used;
        if (free_bytes >= n_bytes) {
            for (size_t i = 0; i < n_bytes; i++) {
                s_ring_buf[head] = (uint8_t)row[i];
                head = (head + 1U) % ROUTE_RING_BUF_SIZE;
            }
            s_ring_head = head;
            uint32_t pts = s_route_points + 1U;
            s_route_points = pts;
        }
        xSemaphoreGive(s_route_buf_mux);
    }
}

extern "C" void route_logger_flush(void)
{
    if (!s_route_active || !s_ring_buf) return;

    bool has_data = (s_ring_head != s_ring_tail);
    bool needs_periodic_sync = (millis() - s_last_sd_flush_ms >= 1000);
    if (!has_data && !needs_periodic_sync) return;

    if (!s_route_file_mux || xSemaphoreTake(s_route_file_mux, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    if (fs_mux && xSemaphoreTake(fs_mux, pdMS_TO_TICKS(15)) != pdTRUE) {
        xSemaphoreGive(s_route_file_mux);
        return;
    }

    if (s_route_active) {
        route_logger_drain_to_file(false);
    }

    if (fs_mux) xSemaphoreGive(fs_mux);
    xSemaphoreGive(s_route_file_mux);
}
