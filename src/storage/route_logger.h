#ifndef ROUTE_LOGGER_H
#define ROUTE_LOGGER_H

#include "../rover_api.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool route_logger_start(void);
void route_logger_stop(void);
bool route_logger_is_active(void);
const char* route_logger_get_current_filename(void);
uint32_t route_logger_get_point_count(void);
void route_logger_log_point(const RoverUIData_t *state);

#ifdef __cplusplus
}
#endif

#endif // ROUTE_LOGGER_H
