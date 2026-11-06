#define RTK_ROVER_NO_SERIAL_REDIRECT
#include "serial_logger.h"
#include "../settings_manager/settings_manager.h"
#include "../rtc_bsp/rtc_bsp.h"
#include <Arduino.h>
#include <FS.h>
#include "esp_log.h"
#include "esp_heap_caps.h"
#include <stdarg.h>
#include <sys/time.h>
#include <time.h>
#include <stdlib.h>

#define SERIAL_RING_BUF_SIZE 32768

static bool s_serial_active = false;
static uint32_t s_bytes_logged = 0;
static char s_serial_filename[64] = {0};
static File s_serial_file;
static SemaphoreHandle_t s_serial_mux = NULL;

static uint8_t *s_ring_buf = NULL;
static size_t s_ring_head = 0;
static size_t s_ring_tail = 0;
static bool s_in_logger = false;
static bool s_timestamp_enabled = true;
static bool s_at_start_of_line = true;

static bool ensure_serial_ring_buf(void) {
    if (s_ring_buf) return true;
    s_ring_buf = (uint8_t *)heap_caps_malloc(SERIAL_RING_BUF_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_ring_buf) {
        s_ring_buf = (uint8_t *)malloc(SERIAL_RING_BUF_SIZE);
    }
    return (s_ring_buf != NULL);
}

SerialLoggerClass SerialLogger;

SerialLoggerClass::SerialLoggerClass() {}

void SerialLoggerClass::begin(unsigned long baud) {
    ::Serial.begin(baud);
    if (!s_serial_mux) {
        s_serial_mux = xSemaphoreCreateRecursiveMutex();
    }
    ensure_serial_ring_buf();
}

void SerialLoggerClass::end() {
    ::Serial.end();
}

void SerialLoggerClass::setTxTimeoutMs(uint32_t timeout) {
    ::Serial.setTxTimeoutMs(timeout);
}

int SerialLoggerClass::available() {
    return ::Serial.available();
}

int SerialLoggerClass::read() {
    return ::Serial.read();
}

int SerialLoggerClass::peek() {
    return ::Serial.peek();
}

void SerialLoggerClass::flush() {
    ::Serial.flush();
    if (s_serial_active) {
        serial_logger_flush();
    }
}

void SerialLoggerClass::setTimestampEnabled(bool enabled) {
    s_timestamp_enabled = enabled;
}

bool SerialLoggerClass::isTimestampEnabled() const {
    return s_timestamp_enabled;
}

static size_t format_record_timestamp(char *buf, size_t max_len) {
    struct timeval tv;
    gettimeofday(&tv, NULL);

    // If real-world epoch is valid (year >= 2025, epoch >= 1735689600)
    if (tv.tv_sec >= 1735689600) {
        time_t local_sec = tv.tv_sec + (time_t)(current_settings.minutesOffsetToUTC * 60);
        int sec_in_day = (int)(local_sec % 86400);
        if (sec_in_day < 0) sec_in_day += 86400;
        int hh   = sec_in_day / 3600;
        int mm   = (sec_in_day % 3600) / 60;
        int ss   = sec_in_day % 60;
        int msec = (int)(tv.tv_usec / 1000);
        return snprintf(buf, max_len, "[%02d:%02d:%02d.%03d] ", hh, mm, ss, msec);
    } else {
        // Fallback to uptime since boot before RTC/GNSS time sync
        uint32_t now = millis();
        uint32_t ms  = now % 1000;
        uint32_t sec = (now / 1000) % 60;
        uint32_t min = (now / 60000) % 60;
        uint32_t hr  = (now / 3600000);
        return snprintf(buf, max_len, "[%02u:%02u:%02u.%03u] ", hr, min, sec, ms);
    }
}

static void serial_logger_feed_locked(const uint8_t *data, size_t len) {
    if (!data || len == 0 || !ensure_serial_ring_buf()) return;
    for (size_t i = 0; i < len; i++) {
        size_t next_head = (s_ring_head + 1) % SERIAL_RING_BUF_SIZE;
        if (next_head != s_ring_tail) {
            s_ring_buf[s_ring_head] = data[i];
            s_ring_head = next_head;
        } else {
            // Buffer full: drop oldest byte to keep newest diagnostics
            s_ring_tail = (s_ring_tail + 1) % SERIAL_RING_BUF_SIZE;
            s_ring_buf[s_ring_head] = data[i];
            s_ring_head = next_head;
        }
    }
}

static void serial_logger_feed(const uint8_t *data, size_t len) {
    if (!data || len == 0) return;

    if (!s_serial_mux) {
        s_serial_mux = xSemaphoreCreateRecursiveMutex();
    }

    if (s_serial_mux && xSemaphoreTakeRecursive(s_serial_mux, pdMS_TO_TICKS(5)) == pdTRUE) {
        serial_logger_feed_locked(data, len);
        xSemaphoreGiveRecursive(s_serial_mux);
    }
}

size_t SerialLoggerClass::write(uint8_t c) {
    return write(&c, 1);
}

size_t SerialLoggerClass::write(const uint8_t *buffer, size_t size) {
    if (!buffer || size == 0) return 0;

    if (s_in_logger) {
        return ::Serial.write(buffer, size);
    }

    if (!s_serial_mux) {
        s_serial_mux = xSemaphoreCreateRecursiveMutex();
    }

    if (s_serial_mux && xSemaphoreTakeRecursive(s_serial_mux, pdMS_TO_TICKS(20)) == pdTRUE) {
        if (!s_timestamp_enabled) {
            ::Serial.write(buffer, size);
            serial_logger_feed_locked(buffer, size);
            xSemaphoreGiveRecursive(s_serial_mux);
            return size;
        }

        size_t idx = 0;
        while (idx < size) {
            // Prepend timestamp at start of record (skip blank newlines)
            if (s_at_start_of_line) {
                uint8_t first_char = buffer[idx];
                if (first_char != '\r' && first_char != '\n') {
                    char ts_buf[32];
                    size_t ts_len = format_record_timestamp(ts_buf, sizeof(ts_buf));
                    ::Serial.write((const uint8_t*)ts_buf, ts_len);
                    serial_logger_feed_locked((const uint8_t*)ts_buf, ts_len);
                    s_at_start_of_line = false;
                }
            }

            // Find segment up to next '\n' or buffer end
            size_t seg_start = idx;
            while (idx < size && buffer[idx] != '\n') {
                idx++;
            }

            if (idx < size && buffer[idx] == '\n') {
                idx++; // include '\n'
                s_at_start_of_line = true;
            }

            size_t seg_len = idx - seg_start;
            if (seg_len > 0) {
                ::Serial.write(buffer + seg_start, seg_len);
                serial_logger_feed_locked(buffer + seg_start, seg_len);
            }
        }
        xSemaphoreGiveRecursive(s_serial_mux);
    } else {
        ::Serial.write(buffer, size);
    }
    return size;
}

bool SerialLoggerClass::start() {
    return serial_logger_start();
}

void SerialLoggerClass::stop() {
    serial_logger_stop();
}

bool SerialLoggerClass::isActive() const {
    return serial_logger_is_active();
}

void SerialLoggerClass::flushBuffer() {
    serial_logger_flush();
}

const char* SerialLoggerClass::getCurrentFilename() const {
    return serial_logger_get_current_filename();
}

uint32_t SerialLoggerClass::getBytesLogged() const {
    return serial_logger_get_bytes_logged();
}

static void serial_logger_flush_locked(void) {
    if (s_serial_file && s_ring_buf && s_ring_head != s_ring_tail) {
        uint8_t write_buf[1024];
        s_in_logger = true;
        while (s_ring_tail != s_ring_head) {
            size_t bytes_to_write = 0;
            while (s_ring_tail != s_ring_head && bytes_to_write < sizeof(write_buf)) {
                write_buf[bytes_to_write++] = s_ring_buf[s_ring_tail];
                s_ring_tail = (s_ring_tail + 1) % SERIAL_RING_BUF_SIZE;
            }

            if (bytes_to_write > 0) {
                s_serial_file.write(write_buf, bytes_to_write);
                s_bytes_logged += bytes_to_write;
            }
        }
        s_serial_file.flush();
        s_in_logger = false;
    }
}

extern "C" bool serial_logger_start(void) {
    if (!s_serial_mux) {
        s_serial_mux = xSemaphoreCreateRecursiveMutex();
    }
    if (s_serial_mux) xSemaphoreTakeRecursive(s_serial_mux, portMAX_DELAY);

    if (s_serial_active) {
        if (s_serial_mux) xSemaphoreGiveRecursive(s_serial_mux);
        return true;
    }

    if (s_serial_file) {
        s_serial_file.close();
    }

    s_in_logger = true;
    if (!get_active_fs().exists("/serial")) {
        get_active_fs().mkdir("/serial");
    }

    char ts[64];
    get_formatted_timestamp(ts, sizeof(ts), true);
    snprintf(s_serial_filename, sizeof(s_serial_filename), "/serial/serial-log_%s.txt", ts);

    s_serial_file = get_active_fs().open(s_serial_filename, FILE_WRITE);
    if (!s_serial_file) {
        // Fallback to flat root filename if /serial directory is not supported
        snprintf(s_serial_filename, sizeof(s_serial_filename), "/serial-log_%s.txt", ts);
        s_serial_file = get_active_fs().open(s_serial_filename, FILE_WRITE);
    }
    s_in_logger = false;

    if (!s_serial_file) {
        ::Serial.printf("[SERIAL LOG] Failed to open %s for write\n", s_serial_filename);
        s_serial_active = false;
        if (s_serial_mux) xSemaphoreGiveRecursive(s_serial_mux);
        return false;
    }

    s_bytes_logged = 0;
    s_serial_active = true;
    ::Serial.printf("[SERIAL LOG] Started recording to %s\n", s_serial_filename);

    // Flush any power-up messages that were accumulated before logger start
    serial_logger_flush_locked();

    if (s_serial_mux) xSemaphoreGiveRecursive(s_serial_mux);
    return true;
}

extern "C" void serial_logger_stop(void) {
    if (s_serial_mux) xSemaphoreTakeRecursive(s_serial_mux, portMAX_DELAY);
    if (s_serial_active) {
        serial_logger_flush_locked();
        s_in_logger = true;
        if (s_serial_file) {
            s_serial_file.flush();
            s_serial_file.close();
        }
        s_in_logger = false;
        s_serial_active = false;
        ::Serial.printf("[SERIAL LOG] Stopped. Total logged: %lu bytes to %s\n",
                        (unsigned long)s_bytes_logged, s_serial_filename);
    }
    if (s_serial_mux) xSemaphoreGiveRecursive(s_serial_mux);
}

extern "C" bool serial_logger_is_active(void) {
    return s_serial_active;
}

extern "C" const char* serial_logger_get_current_filename(void) {
    return s_serial_filename;
}

extern "C" uint32_t serial_logger_get_bytes_logged(void) {
    return s_bytes_logged;
}

extern "C" void serial_logger_flush(void) {
    if (!s_serial_active) return;
    if (s_serial_mux && xSemaphoreTakeRecursive(s_serial_mux, pdMS_TO_TICKS(5)) == pdTRUE) {
        serial_logger_flush_locked();
        xSemaphoreGiveRecursive(s_serial_mux);
    }
}

extern "C" void serial_logger_set_timestamp_enabled(bool enabled) {
    s_timestamp_enabled = enabled;
}

extern "C" bool serial_logger_is_timestamp_enabled(void) {
    return s_timestamp_enabled;
}

extern "C" int rover_printf(const char *format, ...) {
    char buf[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    if (len > 0) {
        SerialLogger.write((const uint8_t*)buf, len);
    }
    return len;
}
