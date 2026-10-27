#define RTK_ROVER_NO_SERIAL_REDIRECT
#include "serial_logger.h"
#include "../settings_manager/settings_manager.h"
#include "../rtc_bsp/rtc_bsp.h"
#include <Arduino.h>
#include <FS.h>
#include "esp_log.h"
#include <stdarg.h>

#define SERIAL_RING_BUF_SIZE 32768

static bool s_serial_active = false;
static uint32_t s_bytes_logged = 0;
static char s_serial_filename[64] = {0};
static File s_serial_file;
static SemaphoreHandle_t s_serial_mux = NULL;

static uint8_t s_ring_buf[SERIAL_RING_BUF_SIZE];
static size_t s_ring_head = 0;
static size_t s_ring_tail = 0;
static bool s_in_logger = false;
static vprintf_like_t s_orig_vprintf = NULL;

SerialLoggerClass SerialLogger;

static int serial_vprintf_hook(const char *fmt, va_list args) {
    if (s_in_logger) {
        if (s_orig_vprintf) return s_orig_vprintf(fmt, args);
        return 0;
    }
    char buf[256];
    va_list copy;
    va_copy(copy, args);
    int len = vsnprintf(buf, sizeof(buf), fmt, copy);
    va_end(copy);

    if (len > 0) {
        SerialLogger.write((const uint8_t*)buf, len);
    }
    if (s_orig_vprintf) {
        return s_orig_vprintf(fmt, args);
    }
    return len;
}

SerialLoggerClass::SerialLoggerClass() {}

void SerialLoggerClass::begin(unsigned long baud) {
    ::Serial.begin(baud);
    if (!s_serial_mux) {
        s_serial_mux = xSemaphoreCreateMutex();
    }
    if (s_orig_vprintf == NULL) {
        s_orig_vprintf = esp_log_set_vprintf(serial_vprintf_hook);
    }
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

static void serial_logger_feed(const uint8_t *data, size_t len) {
    if (!data || len == 0) return;

    if (!s_serial_mux) {
        s_serial_mux = xSemaphoreCreateMutex();
    }

    if (s_serial_mux && xSemaphoreTake(s_serial_mux, pdMS_TO_TICKS(5)) == pdTRUE) {
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
        xSemaphoreGive(s_serial_mux);
    }
}

size_t SerialLoggerClass::write(uint8_t c) {
    ::Serial.write(c);
    if (!s_in_logger) {
        serial_logger_feed(&c, 1);
    }
    return 1;
}

size_t SerialLoggerClass::write(const uint8_t *buffer, size_t size) {
    ::Serial.write(buffer, size);
    if (!s_in_logger && buffer && size > 0) {
        serial_logger_feed(buffer, size);
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
    if (s_serial_file && s_ring_head != s_ring_tail) {
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
        s_serial_mux = xSemaphoreCreateMutex();
    }
    if (s_serial_mux) xSemaphoreTake(s_serial_mux, portMAX_DELAY);

    if (s_serial_active) {
        if (s_serial_mux) xSemaphoreGive(s_serial_mux);
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
        if (s_serial_mux) xSemaphoreGive(s_serial_mux);
        return false;
    }

    s_bytes_logged = 0;
    s_serial_active = true;
    ::Serial.printf("[SERIAL LOG] Started recording to %s\n", s_serial_filename);

    // Flush any power-up messages that were accumulated before logger start
    serial_logger_flush_locked();

    if (s_serial_mux) xSemaphoreGive(s_serial_mux);
    return true;
}

extern "C" void serial_logger_stop(void) {
    if (s_serial_mux) xSemaphoreTake(s_serial_mux, portMAX_DELAY);
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
    if (s_serial_mux) xSemaphoreGive(s_serial_mux);
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
    if (s_serial_mux && xSemaphoreTake(s_serial_mux, pdMS_TO_TICKS(5)) == pdTRUE) {
        serial_logger_flush_locked();
        xSemaphoreGive(s_serial_mux);
    }
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
