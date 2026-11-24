#ifndef IMU_BSP_H
#define IMU_BSP_H

#include <stdbool.h>
#include <stdint.h>
#include "../rover_api.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*imu_log_cb_t)(const char *fmt, ...);

void imu_set_log_callback(imu_log_cb_t cb);
void imu_set_debug_logging(bool enable);
void imu_init(void);
void imu_read(float *acc, float *gyr, int16_t *temp);
void imu_compute_pose_and_bubble(const float *acc,
                                 float tare_pitch_deg,
                                 float tare_roll_deg,
                                 float *pitch_deg_out,
                                 float *roll_deg_out,
                                 float *bubble_x_deg_out,
                                 float *bubble_y_deg_out,
                                 DisplayPose_t *pose_out);

#ifdef __cplusplus
}
#endif

#endif // IMU_BSP_H
