#include "rtcm_logger.h"
#include "serial_logger.h"
#include "../settings_manager/settings_manager.h"
#include <Field_Instrument_Core.h>

#define RTCM_RING_BUF_SIZE 16384U

extern SemaphoreHandle_t fs_mux;

static FicRingLogger_t s_rtcm_logger = {};
static bool            s_rtcm_inited = false;

static void rtcm_timestamp_cb(char *buf, size_t max_len)
{
    fic_rtc_get_formatted_timestamp(buf, max_len, true);
}

static bool ensure_rtcm_init(void)
{
    if (s_rtcm_inited) {
        s_rtcm_logger.cfg.fs     = &get_active_fs();
        s_rtcm_logger.cfg.fs_mux = fs_mux;
        return true;
    }
    FicRingLoggerCfg_t cfg = {};
    cfg.tag               = "RTCM LOG";
    cfg.fs                = &get_active_fs();
    cfg.fs_mux            = fs_mux;
    cfg.dir_path          = "/rtcm";
    cfg.file_prefix       = "rtcm-base";
    cfg.file_ext          = ".txt";
    cfg.header_line       = nullptr;
    cfg.ring_bytes        = RTCM_RING_BUF_SIZE;
    cfg.flush_interval_ms = 500U;
    cfg.drop_oldest       = false;
    cfg.timestamp_cb      = rtcm_timestamp_cb;
    cfg.log_cb            = rover_printf;
    s_rtcm_inited = fic_ring_logger_init(&s_rtcm_logger, &cfg);
    return s_rtcm_inited;
}

extern "C" bool rtcm_logger_start(void)
{
    if (!ensure_rtcm_init()) {
        return false;
    }
    return fic_ring_logger_start(&s_rtcm_logger);
}

extern "C" void rtcm_logger_stop(void)
{
    if (!s_rtcm_inited) {
        return;
    }
    fic_ring_logger_stop(&s_rtcm_logger);
}

extern "C" bool rtcm_logger_is_active(void)
{
    return s_rtcm_inited && fic_ring_logger_is_active(&s_rtcm_logger);
}

extern "C" const char *rtcm_logger_get_current_filename(void)
{
    return s_rtcm_inited ? fic_ring_logger_get_filename(&s_rtcm_logger) : "";
}

extern "C" uint32_t rtcm_logger_get_bytes_logged(void)
{
    return s_rtcm_inited ? fic_ring_logger_get_bytes_logged(&s_rtcm_logger) : 0U;
}

extern "C" void rtcm_logger_feed(const uint8_t *data, size_t len)
{
    if (!s_rtcm_inited || !fic_ring_logger_is_active(&s_rtcm_logger)) {
        return;
    }
    fic_ring_logger_feed(&s_rtcm_logger, data, len);
}

extern "C" void rtcm_logger_flush(void)
{
    if (!s_rtcm_inited) {
        return;
    }
    fic_ring_logger_flush(&s_rtcm_logger);
}
