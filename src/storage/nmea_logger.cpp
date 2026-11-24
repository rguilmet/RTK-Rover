#include "nmea_logger.h"
#include "../settings_manager/settings_manager.h"
#include "../rtc_bsp/rtc_bsp.h"
#include <Arduino.h>
#include <FS.h>
#include "esp_heap_caps.h"
#include <stdlib.h>

#define NMEA_RING_BUF_SIZE 8192

extern SemaphoreHandle_t fs_mux;

static volatile bool s_nmea_active = false;
static uint32_t s_bytes_logged = 0;
static char s_nmea_filename[64] = {0};
static File s_nmea_file;
static SemaphoreHandle_t s_nmea_buf_mux = NULL;
static SemaphoreHandle_t s_nmea_file_mux = NULL;

static uint8_t *s_ring_buf = NULL;
static volatile size_t s_ring_head = 0;
static volatile size_t s_ring_tail = 0;
static uint32_t s_last_sd_flush_ms = 0;

static bool ensure_nmea_ring_buf(void)
{
    if (s_ring_buf) return true;
    s_ring_buf = (uint8_t *)heap_caps_malloc(NMEA_RING_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_ring_buf) {
        s_ring_buf = (uint8_t *)malloc(NMEA_RING_BUF_SIZE);
    }
    return (s_ring_buf != NULL);
}

extern "C" bool nmea_logger_start(void)
{
    if (!s_nmea_buf_mux) {
        s_nmea_buf_mux = xSemaphoreCreateMutex();
    }
    if (!s_nmea_file_mux) {
        s_nmea_file_mux = xSemaphoreCreateMutex();
    }
    if (s_nmea_file_mux) xSemaphoreTake(s_nmea_file_mux, portMAX_DELAY);

    if (!ensure_nmea_ring_buf()) {
        if (s_nmea_file_mux) xSemaphoreGive(s_nmea_file_mux);
        return false;
    }

    if (s_nmea_active) {
        if (s_nmea_file_mux) xSemaphoreGive(s_nmea_file_mux);
        return true;
    }

    if (fs_mux) xSemaphoreTake(fs_mux, portMAX_DELAY);

    if (s_nmea_file) {
        s_nmea_file.close();
    }

    if (!get_active_fs().exists("/nmea")) {
        get_active_fs().mkdir("/nmea");
    }

    char ts[64];
    get_formatted_timestamp(ts, sizeof(ts), true);
    snprintf(s_nmea_filename, sizeof(s_nmea_filename), "/nmea/nema-log_%s.txt", ts);

    s_nmea_file = get_active_fs().open(s_nmea_filename, FILE_WRITE);
    if (!s_nmea_file) {
        // Fallback to flat root filename if /nmea directory is not supported
        snprintf(s_nmea_filename, sizeof(s_nmea_filename), "/nema-log_%s.txt", ts);
        s_nmea_file = get_active_fs().open(s_nmea_filename, FILE_WRITE);
    }
    if (!s_nmea_file) {
        s_nmea_active = false;
        if (fs_mux) xSemaphoreGive(fs_mux);
        if (s_nmea_file_mux) xSemaphoreGive(s_nmea_file_mux);
        Serial.printf("[NMEA LOG] Failed to open %s for write\n", s_nmea_filename);
        return false;
    }

    if (s_nmea_buf_mux) xSemaphoreTake(s_nmea_buf_mux, portMAX_DELAY);
    s_ring_head = 0;
    s_ring_tail = 0;
    s_bytes_logged = 0;
    s_last_sd_flush_ms = millis();
    if (s_nmea_buf_mux) xSemaphoreGive(s_nmea_buf_mux);

    s_nmea_active = true;

    if (fs_mux) xSemaphoreGive(fs_mux);
    if (s_nmea_file_mux) xSemaphoreGive(s_nmea_file_mux);
    Serial.printf("[NMEA LOG] Started recording raw stream to %s\n", s_nmea_filename);
    return true;
}

// Drain ring buffer to SD/FFat WITHOUT holding s_nmea_buf_mux across slow VFS write/flush calls.
// Caller must already hold s_nmea_file_mux and (if initialized) fs_mux.
static void nmea_logger_drain_to_file(bool force_sync)
{
    if (!s_nmea_file || !s_ring_buf) return;

    uint8_t write_buf[1024];
    bool wrote_any = false;

    for (;;) {
        size_t bytes_to_write = 0;
        if (s_nmea_buf_mux && xSemaphoreTake(s_nmea_buf_mux, pdMS_TO_TICKS(20)) == pdTRUE) {
            size_t tail = s_ring_tail;
            size_t head = s_ring_head;
            while (tail != head && bytes_to_write < sizeof(write_buf)) {
                write_buf[bytes_to_write++] = s_ring_buf[tail];
                tail = (tail + 1U) % NMEA_RING_BUF_SIZE;
            }
            s_ring_tail = tail;
            xSemaphoreGive(s_nmea_buf_mux);
        } else {
            break;
        }

        if (bytes_to_write == 0) {
            break;
        }

        s_nmea_file.write(write_buf, bytes_to_write);
        s_bytes_logged += bytes_to_write;
        wrote_any = true;
    }

    uint32_t now_ms = millis();
    if (force_sync || (wrote_any && (now_ms - s_last_sd_flush_ms >= 500))) {
        s_last_sd_flush_ms = now_ms;
        s_nmea_file.flush();
    }
}

extern "C" void nmea_logger_stop(void)
{
    if (!s_nmea_file_mux) return;
    xSemaphoreTake(s_nmea_file_mux, portMAX_DELAY);
    bool stopped = false;
    uint32_t total_logged = 0;
    if (s_nmea_active) {
        if (fs_mux) xSemaphoreTake(fs_mux, portMAX_DELAY);
        nmea_logger_drain_to_file(true);
        if (s_nmea_file) {
            s_nmea_file.flush();
            s_nmea_file.close();
        }
        s_nmea_active = false;
        total_logged = s_bytes_logged;
        stopped = true;
        if (fs_mux) xSemaphoreGive(fs_mux);
    }
    xSemaphoreGive(s_nmea_file_mux);
    if (stopped) {
        Serial.printf("[NMEA LOG] Stopped. Total logged: %lu bytes to %s\n", (unsigned long)total_logged, s_nmea_filename);
    }
}

extern "C" bool nmea_logger_is_active(void)
{
    return s_nmea_active;
}

extern "C" const char* nmea_logger_get_current_filename(void)
{
    return s_nmea_filename;
}

extern "C" uint32_t nmea_logger_get_bytes_logged(void)
{
    return s_bytes_logged;
}

extern "C" void nmea_logger_feed(const uint8_t *data, size_t len)
{
    if (!s_nmea_active || !s_ring_buf || !data || len == 0) return;

    if (!s_nmea_buf_mux) {
        s_nmea_buf_mux = xSemaphoreCreateMutex();
    }

    if (s_nmea_buf_mux && xSemaphoreTake(s_nmea_buf_mux, pdMS_TO_TICKS(5)) == pdTRUE) {
        size_t head = s_ring_head;
        size_t tail = s_ring_tail;
        for (size_t i = 0; i < len; i++) {
            size_t next_head = (head + 1U) % NMEA_RING_BUF_SIZE;
            if (next_head != tail) {
                s_ring_buf[head] = data[i];
                head = next_head;
            } else {
                // Buffer overflow: drop byte to prevent stalling
                break;
            }
        }
        s_ring_head = head;
        xSemaphoreGive(s_nmea_buf_mux);
    }
}

extern "C" void nmea_logger_flush(void)
{
    if (!s_nmea_active || !s_ring_buf) return;

    bool has_data = (s_ring_head != s_ring_tail);
    bool needs_periodic_sync = (millis() - s_last_sd_flush_ms >= 500);
    if (!has_data && !needs_periodic_sync) return;

    if (!s_nmea_file_mux || xSemaphoreTake(s_nmea_file_mux, pdMS_TO_TICKS(10)) != pdTRUE) {
        return;
    }
    if (fs_mux && xSemaphoreTake(fs_mux, pdMS_TO_TICKS(15)) != pdTRUE) {
        xSemaphoreGive(s_nmea_file_mux);
        return;
    }

    if (s_nmea_active) {
        nmea_logger_drain_to_file(false);
    }

    if (fs_mux) xSemaphoreGive(fs_mux);
    xSemaphoreGive(s_nmea_file_mux);
}
