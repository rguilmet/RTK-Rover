#ifndef ROVER_SENSORS_H
#define ROVER_SENSORS_H

#include <stdint.h>
#include <stdbool.h>
#include "../rover_api.h"

#ifdef __cplusplus
extern "C" {
#endif

void trigger_tare_pole_level(void);
void trigger_mag_cal_start_or_finish(void);
void trigger_mag_cal_cancel(void);

void rover_sensors_hw_init(void);
void rover_sensors_ahrs_init(void);
void rover_sensors_poll_core1(uint32_t now_ms);
void rover_sensors_apply_to_ui_state(uint32_t now_ms, RoverUIData_t *ui_state);
void rover_sensors_get_pole_tilt_snapshot(float *bubble_x, float *bubble_y, float *pole_tilt);
bool rover_sensors_is_imu_motion_recent(uint32_t now_ms, uint32_t window_ms);
bool rover_sensors_is_mag_cal_collecting(void);

#ifdef __cplusplus
}
#endif

#endif // ROVER_SENSORS_H
