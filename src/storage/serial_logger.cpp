#define RTK_ROVER_NO_SERIAL_REDIRECT
#include "serial_logger.h"
#include <Field_Instrument_Core.h>
#include <Arduino.h>
#include <stdarg.h>

static bool s_core_log_inited   = false;
static bool s_ts_pref           = true;

static void rover_serial_ts_cb(char *buf, size_t max_len)
{
    fic_rtc_get_formatted_timestamp(buf, max_len, true);
}

static void ensure_core_log_init(void)
{
    if (s_core_log_inited) {
        return;
    }
    fic_core_log_attach_stream(&::Serial);
    FicCoreLogCfg_t cfg = {};
    cfg.ring_bytes        = 32768U;
    cfg.flush_interval_ms = 500U;
    cfg.dir_path          = "/serial";
    cfg.file_prefix       = "serial-log";
    cfg.timestamp_enabled = s_ts_pref;
    cfg.capture_esp_log   = true;
    cfg.timestamp_cb      = rover_serial_ts_cb;
    fic_core_log_init(&cfg);
    s_core_log_inited = true;
}

SerialLoggerClass SerialLogger;

SerialLoggerClass::SerialLoggerClass() {}

void SerialLoggerClass::begin(unsigned long baud)
{
    ::Serial.begin(baud);
    ensure_core_log_init();
}

void SerialLoggerClass::end()
{
    ::Serial.end();
}

void SerialLoggerClass::setTxTimeoutMs(uint32_t timeout)
{
    ::Serial.setTxTimeoutMs(timeout);
}

int SerialLoggerClass::available()
{
    return ::Serial.available();
}

int SerialLoggerClass::read()
{
    return ::Serial.read();
}

int SerialLoggerClass::peek()
{
    return ::Serial.peek();
}

void SerialLoggerClass::flush()
{
    ::Serial.flush();
    if (fic_core_log_is_active()) {
        fic_core_log_flush();
    }
}

void SerialLoggerClass::setTimestampEnabled(bool enabled)
{
    s_ts_pref = enabled;
    fic_core_log_set_timestamp_enabled(enabled);
}

bool SerialLoggerClass::isTimestampEnabled() const
{
    return s_core_log_inited ? fic_core_log_is_timestamp_enabled() : s_ts_pref;
}

size_t SerialLoggerClass::write(uint8_t c)
{
    return write(&c, 1U);
}

size_t SerialLoggerClass::write(const uint8_t *buffer, size_t size)
{
    if (!s_core_log_inited) {
        return ::Serial.write(buffer, size);
    }
    return fic_core_log_write(buffer, size);
}

bool SerialLoggerClass::start()
{
    return serial_logger_start();
}

void SerialLoggerClass::stop()
{
    serial_logger_stop();
}

bool SerialLoggerClass::isActive() const
{
    return fic_core_log_is_active();
}

void SerialLoggerClass::flushBuffer()
{
    fic_core_log_flush();
}

const char *SerialLoggerClass::getCurrentFilename() const
{
    return fic_core_log_get_filename();
}

uint32_t SerialLoggerClass::getBytesLogged() const
{
    return fic_core_log_get_bytes_logged();
}

extern "C" bool serial_logger_start(void)
{
    ensure_core_log_init();
    return fic_core_log_start();
}

extern "C" void serial_logger_stop(void)
{
    fic_core_log_stop();
}

extern "C" bool serial_logger_is_active(void)
{
    return fic_core_log_is_active();
}

extern "C" void serial_logger_flush(void)
{
    fic_core_log_flush();
}

extern "C" const char *serial_logger_get_current_filename(void)
{
    return fic_core_log_get_filename();
}

extern "C" uint32_t serial_logger_get_bytes_logged(void)
{
    return fic_core_log_get_bytes_logged();
}

extern "C" void serial_logger_set_timestamp_enabled(bool enabled)
{
    s_ts_pref = enabled;
    fic_core_log_set_timestamp_enabled(enabled);
}

extern "C" bool serial_logger_is_timestamp_enabled(void)
{
    return s_core_log_inited ? fic_core_log_is_timestamp_enabled() : s_ts_pref;
}

extern "C" int rover_printf(const char *format, ...)
{
    char buf[256];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);
    if (len > 0) {
        size_t write_len = ((size_t)len < sizeof(buf)) ? (size_t)len : (sizeof(buf) - 1U);
        SerialLogger.write((const uint8_t *)buf, write_len);
    }
    return len;
}
