#ifndef UM982_DRIVER_H
#define UM982_DRIVER_H

#include <stddef.h>
#include <stdint.h>
#include "../rover_api.h"

#ifdef __cplusplus
extern "C" {
#endif

void um982_driver_init(void);
void um982_configure_rover_defaults(void);
void um982_run_powerup_selftest(void);
void um982_poll_com1(void);
void um982_poll_com2(void);
void um982_forward_rtcm_to_com2(const uint8_t *data, size_t len);
void um982_get_latest_state(RoverUIData_t *state_out);
void um982_get_port_stats(Um982PortStats_t *stats_out);

#ifdef __cplusplus
}
#endif

#endif // UM982_DRIVER_H
