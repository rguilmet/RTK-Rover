#ifndef ROVER_POWER_H
#define ROVER_POWER_H

#include <stdint.h>
#include <stdbool.h>
#include "../axp2101_bsp/axp2101_bsp.h"
#include "../rover_api.h"

#ifdef __cplusplus
extern "C" {
#endif

void rover_power_poll(uint32_t now_ms);
void rover_power_get_snapshot(Axp2101Status_t *out);
void rover_power_flush_all_logs(void);
void rover_power_update_auto_shutdown(uint32_t now_ms,
                                      float speed_m_s,
                                      bool survey_in_progress,
                                      bool mag_cal_collecting,
                                      bool imu_motion_active,
                                      bool gnss_motion_active);
void rover_power_apply_to_ui_state(RoverUIData_t *ui_state);

#ifdef __cplusplus
}
#endif

#endif // ROVER_POWER_H
