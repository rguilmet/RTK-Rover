#ifndef RTCM_LOGGER_H
#define RTCM_LOGGER_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

bool        rtcm_logger_start(void);
void        rtcm_logger_stop(void);
bool        rtcm_logger_is_active(void);
const char* rtcm_logger_get_current_filename(void);
uint32_t    rtcm_logger_get_bytes_logged(void);
void        rtcm_logger_feed(const uint8_t *data, size_t len);
void        rtcm_logger_flush(void);

#ifdef __cplusplus
}
#endif

#endif // RTCM_LOGGER_H
