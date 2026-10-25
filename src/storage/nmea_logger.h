#ifndef NMEA_LOGGER_H
#define NMEA_LOGGER_H

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

bool nmea_logger_start(void);
void nmea_logger_stop(void);
bool nmea_logger_is_active(void);
const char* nmea_logger_get_current_filename(void);
uint32_t nmea_logger_get_bytes_logged(void);
void nmea_logger_feed(const uint8_t *data, size_t len);
void nmea_logger_flush(void);

#ifdef __cplusplus
}
#endif

#endif // NMEA_LOGGER_H
