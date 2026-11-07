#include "mmc5983ma_driver.h"
#include "../i2c_bsp/i2c_bsp.h"
#include "../../user_config.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>

// =========================================================================
// MMC5983MA Register Map (Datasheet Rev A)
// =========================================================================
#define MMC5983MA_REG_XOUT0       0x00
#define MMC5983MA_REG_XOUT1       0x01
#define MMC5983MA_REG_YOUT0       0x02
#define MMC5983MA_REG_YOUT1       0x03
#define MMC5983MA_REG_ZOUT0       0x04
#define MMC5983MA_REG_ZOUT1       0x05
#define MMC5983MA_REG_XYZOUT2     0x06
#define MMC5983MA_REG_TOUT        0x07
#define MMC5983MA_REG_STATUS      0x08
#define MMC5983MA_REG_CTRL0       0x09
#define MMC5983MA_REG_CTRL1       0x0A
#define MMC5983MA_REG_CTRL2       0x0B
#define MMC5983MA_REG_CTRL3       0x0C
#define MMC5983MA_REG_PROD_ID1    0x2F

#define MMC5983MA_PROD_ID_VAL     0x30

// Internal Control 0 (0x09) Bits
#define MMC5983MA_CTRL0_TM_M      0x01  // Trigger magnetic measurement
#define MMC5983MA_CTRL0_TM_T      0x02  // Trigger temperature measurement
#define MMC5983MA_CTRL0_SET       0x08  // 500ns SET current pulse
#define MMC5983MA_CTRL0_RESET     0x10  // 500ns RESET current pulse
#define MMC5983MA_CTRL0_AUTO_SR   0x20  // Automatic SET/RESET

// Internal Control 1 (0x0A) Bits
#define MMC5983MA_CTRL1_BW_100HZ  0x00  // 8.0 ms measurement time (0.4 mG noise)
#define MMC5983MA_CTRL1_BW_200HZ  0x01  // 4.0 ms measurement time (0.6 mG noise)
#define MMC5983MA_CTRL1_BW_400HZ  0x02  // 2.0 ms measurement time (0.8 mG noise)
#define MMC5983MA_CTRL1_BW_800HZ  0x03  // 0.5 ms measurement time (1.2 mG noise)
#define MMC5983MA_CTRL1_SW_RST    0x80  // Software reset (>10 ms recovery)

// Internal Control 2 (0x0B) Bits
#define MMC5983MA_CTRL2_CM_50HZ   0x04  // 50 Hz continuous mode frequency
#define MMC5983MA_CTRL2_CMM_EN    0x08  // Continuous measurement mode enable

// 18-bit sensitivity: 16,384 LSB/Gauss = 163.84 LSB/uT (Null field = 131072)
#define MMC5983MA_NULL_FIELD_LSB  131072.0f
#define MMC5983MA_LSB_PER_UT      163.84f

static Mmc5983maState_t s_mag_state = {
    .present             = false,
    .valid               = false,
    .mag_uT              = {0.0f, 0.0f, 0.0f},
    .total_uT            = 0.0f,
    .raw_mag_heading_deg = 0.0f,
    .die_temp_c          = 25.0f,
    .bridge_offset_lsb   = {MMC5983MA_NULL_FIELD_LSB, MMC5983MA_NULL_FIELD_LSB, MMC5983MA_NULL_FIELD_LSB},
    .set_reset_count     = 0
};

static uint32_t s_last_set_reset_ms = 0;
static float    s_last_cal_temp_c   = 25.0f;

static bool mmc5983ma_write_reg(uint8_t reg, uint8_t val)
{
    if (mag_dev_handle == NULL) return false;
    return (i2c_write_buff(mag_dev_handle, reg, &val, 1) == 0);
}

static bool mmc5983ma_read_regs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    if (mag_dev_handle == NULL || !buf) return false;
    return (i2c_read_buff(mag_dev_handle, reg, buf, len) == 0);
}

static bool mmc5983ma_read_raw_18bit(uint32_t raw_xyz[3])
{
    uint8_t buf[7] = {0};
    if (!mmc5983ma_read_regs(MMC5983MA_REG_XOUT0, buf, 7)) {
        return false;
    }
    raw_xyz[0] = ((uint32_t)buf[0] << 10) | ((uint32_t)buf[1] << 2) | ((uint32_t)(buf[6] >> 6) & 0x03U);
    raw_xyz[1] = ((uint32_t)buf[2] << 10) | ((uint32_t)buf[3] << 2) | ((uint32_t)(buf[6] >> 4) & 0x03U);
    raw_xyz[2] = ((uint32_t)buf[4] << 10) | ((uint32_t)buf[5] << 2) | ((uint32_t)(buf[6] >> 2) & 0x03U);
    return true;
}

extern "C" bool mmc5983ma_calibrate_bridge_offset(void)
{
    if (!s_mag_state.present || mag_dev_handle == NULL) {
        return false;
    }

    // 1. Pause continuous mode and set BW = 200Hz (4ms measurement time)
    mmc5983ma_write_reg(MMC5983MA_REG_CTRL2, 0x00);
    mmc5983ma_write_reg(MMC5983MA_REG_CTRL1, MMC5983MA_CTRL1_BW_200HZ);
    vTaskDelay(pdMS_TO_TICKS(5));

    // 2. Pulse SET (500ns current pulse restores positive AMR film magnetization)
    if (!mmc5983ma_write_reg(MMC5983MA_REG_CTRL0, MMC5983MA_CTRL0_SET)) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(2));

    // 3. Trigger single magnetic measurement (+H + Offset)
    mmc5983ma_write_reg(MMC5983MA_REG_CTRL0, MMC5983MA_CTRL0_TM_M);
    vTaskDelay(pdMS_TO_TICKS(6));
    uint32_t m_set[3] = {0};
    if (!mmc5983ma_read_raw_18bit(m_set)) {
        return false;
    }

    // 4. Pulse RESET (500ns current pulse flips AMR film magnetization 180 deg)
    mmc5983ma_write_reg(MMC5983MA_REG_CTRL0, MMC5983MA_CTRL0_RESET);
    vTaskDelay(pdMS_TO_TICKS(2));

    // 5. Trigger single magnetic measurement (-H + Offset)
    mmc5983ma_write_reg(MMC5983MA_REG_CTRL0, MMC5983MA_CTRL0_TM_M);
    vTaskDelay(pdMS_TO_TICKS(6));
    uint32_t m_reset[3] = {0};
    if (!mmc5983ma_read_raw_18bit(m_reset)) {
        return false;
    }

    // 6. Restore normal forward SET polarity for continuous mode operation
    mmc5983ma_write_reg(MMC5983MA_REG_CTRL0, MMC5983MA_CTRL0_SET);
    vTaskDelay(pdMS_TO_TICKS(2));

    // 7. Read on-chip die temperature (Tout: -75C at 0x00, 0.8C/LSB)
    mmc5983ma_write_reg(MMC5983MA_REG_CTRL0, MMC5983MA_CTRL0_TM_T);
    vTaskDelay(pdMS_TO_TICKS(2));
    uint8_t t_raw = 0;
    if (mmc5983ma_read_regs(MMC5983MA_REG_TOUT, &t_raw, 1)) {
        s_mag_state.die_temp_c = -75.0f + ((float)t_raw * 0.8f);
        s_last_cal_temp_c      = s_mag_state.die_temp_c;
    }

    // 8. Compute exact null-field bridge offset: Offset = (M_SET + M_RESET) / 2.0
    for (int i = 0; i < 3; i++) {
        float off = 0.5f * ((float)m_set[i] + (float)m_reset[i]);
        // Sanity check: bridge offset must be within +/- 16384 LSB of 131072
        if (fabsf(off - MMC5983MA_NULL_FIELD_LSB) < 16384.0f) {
            s_mag_state.bridge_offset_lsb[i] = off;
        } else {
            s_mag_state.bridge_offset_lsb[i] = MMC5983MA_NULL_FIELD_LSB;
        }
    }

    // 9. Re-enable 50 Hz 18-bit continuous measurement mode (BW=200Hz/4ms, CMM_EN=1, CM_Freq=50Hz)
    mmc5983ma_write_reg(MMC5983MA_REG_CTRL1, MMC5983MA_CTRL1_BW_200HZ);
    mmc5983ma_write_reg(MMC5983MA_REG_CTRL2, MMC5983MA_CTRL2_CMM_EN | MMC5983MA_CTRL2_CM_50HZ);

    s_mag_state.set_reset_count++;
    s_last_set_reset_ms = millis();
    return true;
}

extern "C" bool mmc5983ma_init(void)
{
    s_mag_state.present = false;
    s_mag_state.valid   = false;

    // Quietly probe optional I2C address 0x30 without triggering bus recovery warnings if unplugged
    if (!i2c_bsp_probe_optional_addr(ROVER_I2C_ADDR_MAG)) {
        Serial.println("[MAG] MMC5983MA (0x30) not detected on I2C0 — continuing in 6-DOF IMU mode.");
        return false;
    }

    uint8_t prod_id = 0;
    if (!mmc5983ma_read_regs(MMC5983MA_REG_PROD_ID1, &prod_id, 1) || prod_id != MMC5983MA_PROD_ID_VAL) {
        Serial.printf("[MAG] Device at 0x30 returned Product ID 0x%02X (expected 0x30) — ignoring.\n", prod_id);
        return false;
    }

    s_mag_state.present = true;

    // Issue Software Reset and wait 15 ms per MMC5983MA datasheet
    mmc5983ma_write_reg(MMC5983MA_REG_CTRL1, MMC5983MA_CTRL1_SW_RST);
    vTaskDelay(pdMS_TO_TICKS(15));

    // Run initial SET/RESET degaussing & exact bridge offset calibration
    bool cal_ok = mmc5983ma_calibrate_bridge_offset();

    Serial.printf("[MAG] MMC5983MA (0x30, ID=0x%02X) ONLINE! 18-bit 50Hz | SET/RESET %s (Offsets: %.1f, %.1f, %.1f LSB | Temp: %.1fC)\n",
                  prod_id,
                  cal_ok ? "OK" : "WARN",
                  s_mag_state.bridge_offset_lsb[0],
                  s_mag_state.bridge_offset_lsb[1],
                  s_mag_state.bridge_offset_lsb[2],
                  s_mag_state.die_temp_c);
    return true;
}

extern "C" bool mmc5983ma_is_present(void)
{
    return s_mag_state.present;
}

extern "C" bool mmc5983ma_poll(float pitch_deg, float roll_deg, Mmc5983maState_t *state_out)
{
    if (!s_mag_state.present) {
        if (state_out) *state_out = s_mag_state;
        return false;
    }

    uint32_t now_ms = millis();

    // Periodic hardware SET/RESET every 60 seconds to eliminate thermal bridge offset drift
    if (s_last_set_reset_ms == 0 || (now_ms - s_last_set_reset_ms >= 60000)) {
        mmc5983ma_calibrate_bridge_offset();
    }

    uint32_t raw_xyz[3] = {0};
    if (!mmc5983ma_read_raw_18bit(raw_xyz)) {
        if (state_out) *state_out = s_mag_state;
        return false;
    }

    // Convert 18-bit unsigned counts to microteslas (uT) using SET/RESET bridge offset
    float bx = ((float)raw_xyz[0] - s_mag_state.bridge_offset_lsb[0]) / MMC5983MA_LSB_PER_UT;
    float by = ((float)raw_xyz[1] - s_mag_state.bridge_offset_lsb[1]) / MMC5983MA_LSB_PER_UT;
    float bz = ((float)raw_xyz[2] - s_mag_state.bridge_offset_lsb[2]) / MMC5983MA_LSB_PER_UT;

    s_mag_state.mag_uT[0] = bx;
    s_mag_state.mag_uT[1] = by;
    s_mag_state.mag_uT[2] = bz;
    s_mag_state.total_uT  = sqrtf(bx * bx + by * by + bz * bz);
    s_mag_state.valid     = (s_mag_state.total_uT > 5.0f && s_mag_state.total_uT < 150.0f);

    if (s_mag_state.valid) {
        // Tilt compensation projecting 3D magnetic vector onto local horizontal plane using pole pitch & roll
        float pitch_rad = pitch_deg * (float)(M_PI / 180.0);
        float roll_rad  = roll_deg  * (float)(M_PI / 180.0);
        float cp = cosf(pitch_rad);
        float sp = sinf(pitch_rad);
        float cr = cosf(roll_rad);
        float sr = sinf(roll_rad);

        float b_xh = bx * cp + by * sr * sp + bz * cr * sp;
        float b_yh = by * cr - bz * sr;

        float hdg_deg = atan2f(-b_yh, b_xh) * (float)(180.0 / M_PI);
        if (hdg_deg < 0.0f)   hdg_deg += 360.0f;
        if (hdg_deg >= 360.0f) hdg_deg -= 360.0f;
        s_mag_state.raw_mag_heading_deg = hdg_deg;
    }

    if (state_out) {
        *state_out = s_mag_state;
    }
    return s_mag_state.valid;
}

extern "C" void mmc5983ma_get_state(Mmc5983maState_t *state_out)
{
    if (state_out) {
        *state_out = s_mag_state;
    }
}
