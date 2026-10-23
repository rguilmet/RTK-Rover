#ifndef IMU_BSP_H
#define IMU_BSP_H

#include <stdint.h>
#include "../rover_api.h"

#ifdef __cplusplus
extern "C" {
#endif

void imu_init(void);
void imu_read(float *acc, float *gyr, int16_t *temp);
void imu_compute_pose_and_bubble(const float *acc,
                                 float *pitch_deg_out,
                                 float *roll_deg_out,
                                 float *bubble_x_deg_out,
                                 float *bubble_y_deg_out,
                                 DisplayPose_t *pose_out);

#ifdef __cplusplus
}
#endif

#endif // IMU_BSP_H
