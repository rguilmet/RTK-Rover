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
    // 1. Soft Reset QMI8658 (REG 0x60 = 0xB0)
    ctrl_buf = 0xB0;
    i2c_write_buff(imu_dev_handle, 0x60, &ctrl_buf, 1);
    vTaskDelay(pdMS_TO_TICKS(5));

    // 2. Poll RST_RESULT (0x4D) until reset completion bit (0x80) is asserted
    uint8_t rst_result = 0;
    for (int retry = 0; retry < 10; retry++) {
        if (i2c_read_buff(imu_dev_handle, 0x4D, &rst_result, 1) == 0 && (rst_result & 0x80)) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }

    // 3. CTRL7 (0x08): Disable sensors during configuration (required by silicon errata before writing CTRL2/CTRL3)
    ctrl_buf = 0x00;
    i2c_write_buff(imu_dev_handle, 0x08, &ctrl_buf, 1);

    // 4. CTRL1 (0x02): Enable Address Auto-Increment (Bit 6 = 1 -> 0x40)
    ctrl_buf = 0x40;
    i2c_write_buff(imu_dev_handle, 0x02, &ctrl_buf, 1);

    // 5. CTRL2 (0x03): Accel Config (4g range = 0x10, 250Hz ODR = 0x05 -> 0x15, NO self-test bit 7)
    ctrl_buf = 0x15;
    i2c_write_buff(imu_dev_handle, 0x03, &ctrl_buf, 1);

    // 6. CTRL3 (0x04): Gyro Config (512dps range = 0x50, 250Hz ODR = 0x05 -> 0x55, NO self-test bit 7)
    ctrl_buf = 0x55;
    i2c_write_buff(imu_dev_handle, 0x04, &ctrl_buf, 1);

    // 7. CTRL5 (0x06): Low-Pass Filter (LPF_MODE_0 = 0x00, enable filters for accel & gyro = 0x11)
    ctrl_buf = 0x11;
    i2c_write_buff(imu_dev_handle, 0x06, &ctrl_buf, 1);

    // 8. CTRL6 (0x07): Disable AttitudeEngine Motion On Demand
    ctrl_buf = 0x00;
    i2c_write_buff(imu_dev_handle, 0x07, &ctrl_buf, 1);

    // 9. CTRL8 (0x09): Handshake configuration
    ctrl_buf = 0x80;
    i2c_write_buff(imu_dev_handle, 0x09, &ctrl_buf, 1);

    // 10. CTRL7 (0x08): Enable internal high-speed clock (0x40) + Gyro (0x02) + Accel (0x01) -> 0x43
    ctrl_buf = 0x43;
    i2c_write_buff(imu_dev_handle, 0x08, &ctrl_buf, 1);
    vTaskDelay(pdMS_TO_TICKS(15));

    if (err == 0 && who_am_i == 0x05) {
        printf("[IMU] QMI8658 (0x6B) ONLINE — WHO_AM_I: 0x%02X (4g/512dps @ 250Hz)\n", who_am_i);
    } else {
        printf("[IMU] QMI8658 (0x6B) WARN — WHO_AM_I: 0x%02X (err=%d)\n", who_am_i, (int)err);
    }
}

void imu_read(float *acc, float *gyr, int16_t *temp)
{
    static float last_acc[3] = {0.0f, 0.0f, 1.0f};
    static float last_gyr[3] = {0.0f, 0.0f, 0.0f};
    static uint8_t fail_count = 0;
    static uint32_t last_log_ms = 0;

    // Direct read 12 bytes from QMI8658_AX_L (0x35).
    // Address auto-increment reads AX_L, AX_H, AY_L, AY_H, AZ_L, AZ_H, GX_L, GX_H, GY_L, GY_H, GZ_L, GZ_H in one I2C transaction.
    uint8_t raw[12] = {0};
    uint8_t err = i2c_read_buff(imu_dev_handle, 0x35, raw, 12);
    if (err == 0) {
        int16_t ax = (int16_t)(((uint16_t)raw[1] << 8) | raw[0]);
        int16_t ay = (int16_t)(((uint16_t)raw[3] << 8) | raw[2]);
        int16_t az = (int16_t)(((uint16_t)raw[5] << 8) | raw[4]);
        int16_t gx = (int16_t)(((uint16_t)raw[7] << 8) | raw[6]);
        int16_t gy = (int16_t)(((uint16_t)raw[9] << 8) | raw[8]);
        int16_t gz = (int16_t)(((uint16_t)raw[11] << 8) | raw[10]);

        // Validate non-zero reading
        if (ax != 0 || ay != 0 || az != 0 || gx != 0 || gy != 0 || gz != 0) {
            // 4g range -> 1g = 8192 LSB
            last_acc[0] = (float)ax / 8192.0f;
            last_acc[1] = (float)ay / 8192.0f;
            last_acc[2] = (float)az / 8192.0f;
            // 512 dps range -> 1 dps = 64 LSB
            last_gyr[0] = (float)gx / 64.0f;
            last_gyr[1] = (float)gy / 64.0f;
            last_gyr[2] = (float)gz / 64.0f;
            fail_count = 0;
        } else {
            fail_count++;
        }
    } else {
        fail_count++;
    }

    uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if (current_settings.enable_imu_logging && (now_ms - last_log_ms >= 1000)) {
        last_log_ms = now_ms;
        printf("[IMU] err=%d | Acc(g): %+.2f %+.2f %+.2f | Gyr(dps): %+.1f %+.1f %+.1f | fails=%u\n",
               (int)err,
               last_acc[0], last_acc[1], last_acc[2],
               last_gyr[0], last_gyr[1], last_gyr[2],
               (unsigned int)fail_count);
    }

    // Auto-recovery: only reinitialize if 50 consecutive polls fail (5 continuous seconds of silence)
    if (fail_count >= 50) {
        printf("[IMU] 50 consecutive failed reads -> attempting auto-recovery\n");
        fail_count = 0;
        imu_init();
    }

    if (acc) {
        acc[0] = last_acc[0];
        acc[1] = last_acc[1];
        acc[2] = last_acc[2];
    }
    if (gyr) {
        gyr[0] = last_gyr[0];
        gyr[1] = last_gyr[1];
        gyr[2] = last_gyr[2];
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

    // Compute enclosure Pitch (tilt from flat horizontal) and Roll in degrees.
    // On Waveshare ESP32-S3-Touch-LCD-3.5B, gravity points along -Z (-1.0g) when LCD is face-up flat.
    // Using -az ensures resting flat orientation produces Pitch = 0 deg and Roll = 0 deg.
    // Physical QMI8658 orientation in Portrait mode:
    // - Sensor Y axis aligns with Screen Horizontal (Left/Right Roll / Bubble X)
    // - Sensor X axis aligns with Screen Vertical (Up/Down Pitch / Bubble Y)
    // Tipping left (ay < 0) -> raw_roll < 0 -> bubble moves Left
    // Tipping right (ay > 0) -> raw_roll > 0 -> bubble moves Right
    // Tipping up (ax < 0) -> raw_pitch < 0 -> bubble moves Up
    // Tipping down (ax > 0) -> raw_pitch > 0 -> bubble moves Down
    float raw_roll  = atan2f(ay, -az) * (180.0f / (float)M_PI);
    float raw_pitch = atan2f(ax, sqrtf(ay * ay + az * az)) * (180.0f / (float)M_PI);

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
