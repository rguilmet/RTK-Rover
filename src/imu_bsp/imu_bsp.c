#include "imu_bsp.h"
#include "../i2c_bsp/i2c_bsp.h"
#include "../settings_manager/settings_manager.h"
#include <stdint.h>
#include <math.h>
#include <esp_err.h>

extern i2c_master_dev_handle_t imu_dev_handle;

void imu_init(void)
{
    uint8_t ctrl_buf;
    // Enable Address Auto Increment
    ctrl_buf = 0x60;
    i2c_write_buff(imu_dev_handle, 0x02, &ctrl_buf, 1);

    // Accel Config (4g, 250Hz)
    ctrl_buf = 0x14;
    i2c_write_buff(imu_dev_handle, 0x03, &ctrl_buf, 1);

    // Gyro Config (512dps, 250Hz)
    ctrl_buf = 0x54;
    i2c_write_buff(imu_dev_handle, 0x04, &ctrl_buf, 1);

    // Enable Sensors (SYS_EN=1, ACC_EN=1, GYR_EN=1 -> 0x07)
    ctrl_buf = 0x07;
    i2c_write_buff(imu_dev_handle, 0x08, &ctrl_buf, 1);
}

void imu_read(float *acc, float *gyr, int16_t *temp)
{
    uint8_t raw[14];
    if (i2c_read_buff(imu_dev_handle, 0x33, raw, 14) == ESP_OK) {
        int16_t t_raw = (raw[1] << 8) | raw[0];
        int16_t ax = (raw[3] << 8) | raw[2];
        int16_t ay = (raw[5] << 8) | raw[4];
        int16_t az = (raw[7] << 8) | raw[6];
        int16_t gx = (raw[9] << 8) | raw[8];
        int16_t gy = (raw[11] << 8) | raw[10];
        int16_t gz = (raw[13] << 8) | raw[12];

        *temp = t_raw;

        // 4g range -> 1g = 8192 LSB
        acc[0] = (float)ax / 8192.0f;
        acc[1] = (float)ay / 8192.0f;
        acc[2] = (float)az / 8192.0f;

        // 512dps -> 1dps = 64 LSB
        gyr[0] = (float)gx / 64.0f;
        gyr[1] = (float)gy / 64.0f;
        gyr[2] = (float)gz / 64.0f;
    } else {
        *temp = 0;
        acc[0] = acc[1] = acc[2] = 0.0f;
        gyr[0] = gyr[1] = gyr[2] = 0.0f;
    }
}

void imu_compute_pose_and_bubble(const float *acc,
                                 float *pitch_deg_out,
                                 float *roll_deg_out,
                                 float *bubble_x_deg_out,
                                 float *bubble_y_deg_out,
                                 DisplayPose_t *pose_out)
{
    static float ema_pitch = 0.0f;
    static float ema_roll  = 0.0f;
    static bool  first_run = true;

    float ax = acc[0];
    float ay = acc[1];
    float az = acc[2];
    float norm = sqrtf(ax * ax + ay * ay + az * az);
    if (norm < 0.1f) {
        return;
    }

    // Compute enclosure Pitch (tilt from flat horizontal) and Roll in degrees
    float raw_pitch = atan2f(-ay, sqrtf(ax * ax + az * az)) * (180.0f / (float)M_PI);
    float raw_roll  = atan2f(ax, az) * (180.0f / (float)M_PI);

    if (first_run) {
        ema_pitch = raw_pitch;
        ema_roll  = raw_roll;
        first_run = false;
    } else {
        ema_pitch = 0.85f * ema_pitch + 0.15f * raw_pitch;
        ema_roll  = 0.85f * ema_roll  + 0.15f * raw_roll;
    }

    if (pitch_deg_out) *pitch_deg_out = ema_pitch;
    if (roll_deg_out)  *roll_deg_out  = ema_roll;

    // Subtract user-tared bracket angles so the bubble level works whether clipped vertically or held flat
    if (bubble_x_deg_out) *bubble_x_deg_out = ema_roll  - current_settings.pole_tilt_tare_roll_deg;
    if (bubble_y_deg_out) *bubble_y_deg_out = ema_pitch - current_settings.pole_tilt_tare_pitch_deg;

    if (pose_out) {
        float abs_p = fabsf(ema_pitch);
        if (abs_p < 30.0f) {
            *pose_out = DISPLAY_POSE_FLAT;
        } else if (abs_p < 70.0f) {
            *pose_out = DISPLAY_POSE_ANGLED;
        } else {
            *pose_out = DISPLAY_POSE_VERTICAL;
        }
    }
}
