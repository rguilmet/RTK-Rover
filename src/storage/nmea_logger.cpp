#include "nmea_logger.h"
#include "../settings_manager/settings_manager.h"
#include "../rtc_bsp/rtc_bsp.h"
#include <Arduino.h>
#include <FS.h>

#define NMEA_RING_BUF_SIZE 8192

static bool s_nmea_active = false;
static uint32_t s_bytes_logged = 0;
static char s_nmea_filename[64] = {0};
static File s_nmea_file;
static SemaphoreHandle_t s_nmea_mux = NULL;

static uint8_t s_ring_buf[NMEA_RING_BUF_SIZE];
static size_t s_ring_head = 0;
static size_t s_ring_tail = 0;

extern "C" bool nmea_logger_start(void)
{
    if (!s_nmea_mux) {
        s_nmea_mux = xSemaphoreCreateMutex();
    }
    if (s_nmea_mux) xSemaphoreTake(s_nmea_mux, portMAX_DELAY);

    if (s_nmea_active) {
        if (s_nmea_mux) xSemaphoreGive(s_nmea_mux);
        return true;
    }

    if (s_nmea_file) {
        s_nmea_file.close();
    }

    if (!get_active_fs().exists("/nmea")) {
        get_active_fs().mkdir("/nmea");
    }

    char ts[64];
    get_formatted_timestamp(ts, sizeof(ts), true);
    snprintf(s_nmea_filename, sizeof(s_nmea_filename), "/nmea/nmea_%s.log", ts);

    s_nmea_file = get_active_fs().open(s_nmea_filename, FILE_WRITE);
    if (!s_nmea_file) {
        // Fallback to flat root filename if /nmea directory is not supported
        snprintf(s_nmea_filename, sizeof(s_nmea_filename), "/nmea_%s.log", ts);
        s_nmea_file = get_active_fs().open(s_nmea_filename, FILE_WRITE);
    }
    if (!s_nmea_file) {
        Serial.printf("[NMEA LOG] Failed to open %s for write\n", s_nmea_filename);
        s_nmea_active = false;
        if (s_nmea_mux) xSemaphoreGive(s_nmea_mux);
        return false;
    }

    s_ring_head = 0;
    s_ring_tail = 0;
    s_bytes_logged = 0;
    s_nmea_active = true;
    Serial.printf("[NMEA LOG] Started recording raw stream to %s\n", s_nmea_filename);

    if (s_nmea_mux) xSemaphoreGive(s_nmea_mux);
    return true;
}

static void nmea_logger_flush_locked(void)
{
    if (s_nmea_file && s_ring_head != s_ring_tail) {
        uint8_t write_buf[1024];
        while (s_ring_tail != s_ring_head) {
            size_t bytes_to_write = 0;
            while (s_ring_tail != s_ring_head && bytes_to_write < sizeof(write_buf)) {
                write_buf[bytes_to_write++] = s_ring_buf[s_ring_tail];
                s_ring_tail = (s_ring_tail + 1) % NMEA_RING_BUF_SIZE;
            }

            if (bytes_to_write > 0) {
                s_nmea_file.write(write_buf, bytes_to_write);
                s_bytes_logged += bytes_to_write;
            }
        }
        s_nmea_file.flush();
    }
}

extern "C" void nmea_logger_stop(void)
{
    if (s_nmea_mux) xSemaphoreTake(s_nmea_mux, portMAX_DELAY);
    if (s_nmea_active) {
        nmea_logger_flush_locked();
        if (s_nmea_file) {
            s_nmea_file.flush();
            s_nmea_file.close();
        }
        s_nmea_active = false;
        Serial.printf("[NMEA LOG] Stopped. Total logged: %lu bytes to %s\n", (unsigned long)s_bytes_logged, s_nmea_filename);
    }
    if (s_nmea_mux) xSemaphoreGive(s_nmea_mux);
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
    if (!s_nmea_active || !data || len == 0) return;

    if (s_nmea_mux && xSemaphoreTake(s_nmea_mux, pdMS_TO_TICKS(5)) == pdTRUE) {
        for (size_t i = 0; i < len; i++) {
            size_t next_head = (s_ring_head + 1) % NMEA_RING_BUF_SIZE;
            if (next_head != s_ring_tail) {
                s_ring_buf[s_ring_head] = data[i];
                s_ring_head = next_head;
            } else {
                // Buffer overflow: drop byte to prevent stalling
                break;
            }
        }
        xSemaphoreGive(s_nmea_mux);
    }
}

extern "C" void nmea_logger_flush(void)
{
    if (!s_nmea_active) return;

    if (s_nmea_mux && xSemaphoreTake(s_nmea_mux, pdMS_TO_TICKS(5)) == pdTRUE) {
        nmea_logger_flush_locked();
        xSemaphoreGive(s_nmea_mux);
    }
}
