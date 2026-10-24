#include <stdio.h>
#include "imu_bsp.h"
#include "../i2c_bsp/i2c_bsp.h"
#include "../settings_manager/settings_manager.h"
#include <stdint.h>
#include <math.h>
#include <esp_err.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

extern i2c_master_dev_handle_t imu_dev_handle;

void imu_init(void)
{
    uint8_t who_am_i = 0;
    esp_err_t err = i2c_read_buff(imu_dev_handle, 0x00, &who_am_i, 1);

    uint8_t ctrl_buf;
    // Soft Reset QMI8658 (REG 0x60 = 0xB0 per Waveshare bsp_qmi8658.c)
    ctrl_buf = 0xB0;
    i2c_write_buff(imu_dev_handle, 0x60, &ctrl_buf, 1);
    vTaskDelay(pdMS_TO_TICKS(15));

    // CTRL1 (0x02): Enable Address Auto-Increment, Little-Endian (0x40)
    ctrl_buf = 0x40;
    i2c_write_buff(imu_dev_handle, 0x02, &ctrl_buf, 1);

    // CTRL7 (0x08): Enable Accelerometer & Gyroscope (0x03)
    ctrl_buf = 0x03;
    i2c_write_buff(imu_dev_handle, 0x08, &ctrl_buf, 1);

    // CTRL2 (0x03): Accel Config (4g, 250Hz -> 0x95)
    ctrl_buf = 0x95;
    i2c_write_buff(imu_dev_handle, 0x03, &ctrl_buf, 1);

    // CTRL3 (0x04): Gyro Config (512dps, 250Hz -> 0xD5)
    ctrl_buf = 0xD5;
    i2c_write_buff(imu_dev_handle, 0x04, &ctrl_buf, 1);

    if (err == ESP_OK && who_am_i == 0x05) {
        printf("[IMU] QMI8658 (0x6B) ONLINE — WHO_AM_I: 0x%02X (4g/512dps @ 250Hz)\n", who_am_i);
    } else {
        printf("[IMU] QMI8658 (0x6B) WARN — WHO_AM_I: 0x%02X (err=%d)\n", who_am_i, (int)err);
    }
}

void imu_read(float *acc, float *gyr, int16_t *temp)
{
    static float last_acc[3] = {0.0f, 0.0f, 1.0f};
    uint8_t status = 0;
    // Match Waveshare 01_factory bsp_qmi8658.c: check QMI8658_STATUS0 (0x2E) first, then read 6 bytes from QMI8658_AX_L (0x35)
    if (i2c_read_buff(imu_dev_handle, 0x2E, &status, 1) == ESP_OK && (status & 0x03)) {
        uint8_t raw[6] = {0};
        if (i2c_read_buff(imu_dev_handle, 0x35, raw, 6) == ESP_OK) {
            int16_t ax = (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);
            int16_t ay = (int16_t)(((uint16_t)raw[3] << 8) | raw[2]);
            int16_t az = (int16_t)(((uint16_t)raw[5] << 8) | raw[4]);

            // 4g range -> 1g = 8192 LSB
            last_acc[0] = (float)ax / 8192.0f;
            last_acc[1] = (float)ay / 8192.0f;
            last_acc[2] = (float)az / 8192.0f;
        }
    }

    if (acc) {
        acc[0] = last_acc[0];
        acc[1] = last_acc[1];
        acc[2] = last_acc[2];
    }
    if (gyr) {
        gyr[0] = gyr[1] = gyr[2] = 0.0f;
    }
    if (temp) {
        *temp = 0;
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
