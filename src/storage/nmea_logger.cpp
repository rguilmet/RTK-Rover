#include "nmea_logger.h"
#include "serial_logger.h"
#include "../settings_manager/settings_manager.h"
#include <Field_Instrument_Core.h>

#define NMEA_RING_BUF_SIZE 8192U

extern SemaphoreHandle_t fs_mux;

static FicRingLogger_t s_nmea_logger = {};
static bool            s_nmea_inited = false;

static void nmea_timestamp_cb(char *buf, size_t max_len)
{
    fic_rtc_get_formatted_timestamp(buf, max_len, true);
}

static bool ensure_nmea_init(void)
{
    if (s_nmea_inited) {
        s_nmea_logger.cfg.fs     = &get_active_fs();
        s_nmea_logger.cfg.fs_mux = fs_mux;
        return true;
    }
    FicRingLoggerCfg_t cfg = {};
    cfg.tag               = "NMEA LOG";
    cfg.fs                = &get_active_fs();
    cfg.fs_mux            = fs_mux;
    cfg.dir_path          = "/nmea";
    cfg.file_prefix       = "nema-log";
    cfg.file_ext          = ".txt";
    cfg.header_line       = nullptr;
    cfg.ring_bytes        = NMEA_RING_BUF_SIZE;
    cfg.flush_interval_ms = 500U;
    cfg.drop_oldest       = false;
    cfg.timestamp_cb      = nmea_timestamp_cb;
    cfg.log_cb            = rover_printf;
    s_nmea_inited = fic_ring_logger_init(&s_nmea_logger, &cfg);
    return s_nmea_inited;
}

extern "C" bool nmea_logger_start(void)
{
    if (!ensure_nmea_init()) {
        return false;
    }
    return fic_ring_logger_start(&s_nmea_logger);
}

extern "C" void nmea_logger_stop(void)
{
    if (!s_nmea_inited) {
        return;
    }
    fic_ring_logger_stop(&s_nmea_logger);
}

extern "C" bool nmea_logger_is_active(void)
{
    return s_nmea_inited && fic_ring_logger_is_active(&s_nmea_logger);
}

extern "C" const char *nmea_logger_get_current_filename(void)
{
    return s_nmea_inited ? fic_ring_logger_get_filename(&s_nmea_logger) : "";
}

extern "C" uint32_t nmea_logger_get_bytes_logged(void)
{
    return s_nmea_inited ? fic_ring_logger_get_bytes_logged(&s_nmea_logger) : 0U;
}

extern "C" void nmea_logger_feed(const uint8_t *data, size_t len)
{
    if (!s_nmea_inited || !fic_ring_logger_is_active(&s_nmea_logger)) {
        return;
    }
    fic_ring_logger_feed(&s_nmea_logger, data, len);
}

extern "C" void nmea_logger_flush(void)
{
    if (!s_nmea_inited) {
        return;
    }
    fic_ring_logger_flush(&s_nmea_logger);
}
