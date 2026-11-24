#include "sen0696_driver.h"

#include <math.h>
#include <string.h>

#include <driver/i2c_master.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "../i2c_bsp/i2c_bsp.h"

// Verified I2C register map from DFRobot_Multi_DOF_IMU.h (16-bit LE register addresses)
namespace {

constexpr uint16_t kRegVid              = 0x0000U;
constexpr uint16_t kRegPid              = 0x0001U;
constexpr uint16_t kRegFwVersion        = 0x0002U;
constexpr uint16_t kRegSensorsMode      = 0x0005U;
constexpr uint16_t kRegAccRangeConf     = 0x0006U;
constexpr uint16_t kRegGyrRangeConf     = 0x0007U;
constexpr uint16_t kRegAccDataX         = 0x0010U; // 12 B: Accel X/Y/Z (int16) + Gyro X/Y/Z (int16)
constexpr uint16_t kRegMagDataXLowWord  = 0x0016U; // 12 B: Mag X/Y/Z (int32, 0.01 uT/LSB)
constexpr uint16_t kRegPressDataLowWord = 0x001CU; //  4 B: Pressure (int32, Pa)

constexpr uint16_t kModeNormal          = 0x0002U; // eNormalMode
constexpr uint16_t kAccelRange2G        = 0x0000U; // +/-2g
constexpr uint16_t kGyroRange250Dps     = 0x0001U; // +/-250 deg/s

constexpr float    kAccelRangeG         = 2.0f;
constexpr float    kGyroRangeDps        = 250.0f;
constexpr int      kI2cXferTimeoutMs    = 40;
constexpr uint32_t kTwoPhaseDelayMs     = 10U; // Bridge MCU needs >= 10 ms between addr write and read
constexpr uint8_t  kReconfigFailMod     = 10U; // Re-apply config every 10 consecutive failed cycles

int16_t parse_s16_le(const uint8_t *buf)
{
    uint16_t raw = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    return (int16_t)raw;
}

uint16_t parse_u16_le(const uint8_t *buf)
{
    return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

int32_t parse_s32_le(const uint8_t *buf)
{
    uint32_t raw = (uint32_t)buf[0]
                 | ((uint32_t)buf[1] << 8)
                 | ((uint32_t)buf[2] << 16)
                 | ((uint32_t)buf[3] << 24);
    return (int32_t)raw;
}

bool sen0696_write_reg16(FicSen0696_t *ctx, uint16_t reg16, uint16_t val16)
{
    if (ctx == nullptr) {
        return false;
    }

    uint8_t tx[4] = {
        (uint8_t)(reg16 & 0xFFU),
        (uint8_t)((reg16 >> 8) & 0xFFU),
        (uint8_t)(val16 & 0xFFU),
        (uint8_t)((val16 >> 8) & 0xFFU),
    };

    const uint8_t bus = ctx->state.i2c_bus;
    if (!i2c_bsp_lock_port(bus, 150)) {
        return false;
    }
    i2c_master_dev_handle_t dev_handle = (ctx->dev_handle_ptr != nullptr) ? *(ctx->dev_handle_ptr) : nullptr;
    if (dev_handle == nullptr) {
        i2c_bsp_unlock_port(bus);
        return false;
    }
    const esp_err_t err = i2c_master_transmit(dev_handle, tx, sizeof(tx), kI2cXferTimeoutMs);
    i2c_bsp_unlock_port(bus);

    if (err == ESP_OK || err == ESP_ERR_TIMEOUT || err == ESP_ERR_INVALID_STATE) {
        i2c_bsp_report_status_port(bus, dev_handle, "Write", (int)err);
    }
    return (err == ESP_OK);
}

bool sen0696_read_reg(FicSen0696_t *ctx, uint16_t reg16, uint8_t *out, size_t len)
{
    if (ctx == nullptr || out == nullptr || len == 0U) {
        return false;
    }

    uint8_t reg_addr[2] = {
        (uint8_t)(reg16 & 0xFFU),
        (uint8_t)((reg16 >> 8) & 0xFFU),
    };

    const uint8_t bus = ctx->state.i2c_bus;
    // Hold bus lock atomically across [write reg -> 10 ms -> read] so another device
    // cannot split an active SEN0696 bridge transaction or trigger mid-transaction bus resets.
    if (!i2c_bsp_lock_port(bus, 150)) {
        return false;
    }
    i2c_master_dev_handle_t dev_handle = (ctx->dev_handle_ptr != nullptr) ? *(ctx->dev_handle_ptr) : nullptr;
    if (dev_handle == nullptr) {
        i2c_bsp_unlock_port(bus);
        return false;
    }

    esp_err_t err = i2c_master_transmit(dev_handle, reg_addr, sizeof(reg_addr), kI2cXferTimeoutMs);
    if (err == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(kTwoPhaseDelayMs));
        err = i2c_master_receive(dev_handle, out, len, kI2cXferTimeoutMs);
    }

    i2c_bsp_unlock_port(bus);

    // Feed only bus stalls (ESP_ERR_TIMEOUT / ESP_ERR_INVALID_STATE) and successes into runtime bus recovery
    if (err == ESP_OK || err == ESP_ERR_TIMEOUT || err == ESP_ERR_INVALID_STATE) {
        i2c_bsp_report_status_port(bus, dev_handle, "Read", (int)err);
    }
    return (err == ESP_OK);
}

bool sen0696_read_u16(FicSen0696_t *ctx, uint16_t reg16, uint16_t *out_val)
{
    uint8_t rx[2] = {0};
    if (!sen0696_read_reg(ctx, reg16, rx, sizeof(rx))) {
        return false;
    }
    *out_val = parse_u16_le(rx);
    return true;
}

bool sen0696_configure_device(FicSen0696_t *ctx)
{
    const bool ok_mode = sen0696_write_reg16(ctx, kRegSensorsMode, kModeNormal);
    if (!ok_mode) {
        return false;
    }
    vTaskDelay(pdMS_TO_TICKS(25));
    const bool ok_acc  = sen0696_write_reg16(ctx, kRegAccRangeConf, kAccelRange2G);
    vTaskDelay(pdMS_TO_TICKS(10));
    const bool ok_gyr  = sen0696_write_reg16(ctx, kRegGyrRangeConf, kGyroRange250Dps);
    vTaskDelay(pdMS_TO_TICKS(10));
    return ok_mode && ok_acc && ok_gyr;
}

void sen0696_read_acc_gyr_block(FicSen0696_t *ctx)
{
    sen0696_state_t *st = &ctx->state;
    st->accel_gyro_valid = false;
    uint8_t imu_raw[12] = {0};
    if (!sen0696_read_reg(ctx, kRegAccDataX, imu_raw, sizeof(imu_raw)) ||
        i2c_bsp_has_sentinel_tail(imu_raw, sizeof(imu_raw), 4U) ||
        i2c_bsp_has_sentinel_run(imu_raw, sizeof(imu_raw), 4U)) {
        return;
    }

    const float acc_scale = kAccelRangeG / 32768.0f;
    const float gyr_scale = kGyroRangeDps / 32768.0f;
    float acc[3];
    float gyr[3];
    for (int i = 0; i < 3; ++i) {
        acc[i] = (float)parse_s16_le(&imu_raw[i * 2]) * acc_scale;
        gyr[i] = (float)parse_s16_le(&imu_raw[6 + (i * 2)]) * gyr_scale;
        if (!isfinite(acc[i]) || fabsf(acc[i]) > 16.0f || !isfinite(gyr[i]) || fabsf(gyr[i]) > 2000.0f) {
            return;
        }
    }

    memcpy(st->accel_g, acc, sizeof(acc));
    memcpy(st->gyro_dps, gyr, sizeof(gyr));
    st->accel_gyro_valid = true;
}

void sen0696_read_mag_block(FicSen0696_t *ctx)
{
    sen0696_state_t *st = &ctx->state;
    st->mag_valid = false;
    uint8_t mag_raw[12] = {0};
    if (!sen0696_read_reg(ctx, kRegMagDataXLowWord, mag_raw, sizeof(mag_raw)) ||
        i2c_bsp_has_sentinel_tail(mag_raw, sizeof(mag_raw), 4U) ||
        i2c_bsp_has_sentinel_run(mag_raw, sizeof(mag_raw), 4U)) {
        return;
    }

    float mag[3];
    for (int i = 0; i < 3; ++i) {
        mag[i] = (float)parse_s32_le(&mag_raw[i * 4]) * 0.01f;
        if (!isfinite(mag[i]) || fabsf(mag[i]) > 2000.0f) {
            return;
        }
    }

    const float norm = sqrtf((mag[0] * mag[0]) + (mag[1] * mag[1]) + (mag[2] * mag[2]));
    if (!isfinite(norm) || norm > 2500.0f) {
        return;
    }

    memcpy(st->mag_ut, mag, sizeof(mag));
    st->mag_norm_ut = norm;
    st->mag_valid   = true;
}

void sen0696_read_press_block(FicSen0696_t *ctx, bool allow_press_retry)
{
    sen0696_state_t *st = &ctx->state;
    st->press_valid = false;
    uint8_t press_raw[4] = {0};
    if (!sen0696_read_reg(ctx, kRegPressDataLowWord, press_raw, sizeof(press_raw)) ||
        i2c_bsp_has_sentinel_tail(press_raw, sizeof(press_raw), 3U)) {
        return;
    }

    int32_t press_pa = parse_s32_le(press_raw);
    if (press_pa <= 0 && allow_press_retry) {
        // Allow one extra barometer conversion cycle immediately after cold wakeup
        vTaskDelay(pdMS_TO_TICKS(50));
        if (sen0696_read_reg(ctx, kRegPressDataLowWord, press_raw, sizeof(press_raw))) {
            press_pa = parse_s32_le(press_raw);
        }
    }

    if (press_pa < 30000 || press_pa > 120000) {
        return;
    }

    st->pressure_pa  = (float)press_pa;
    st->pressure_hpa = st->pressure_pa * 0.01f;
    st->altitude_m   = 44307.7f * (1.0f - powf(st->pressure_pa / 101325.0f, 0.190284f));
    st->press_valid  = true;
}

void copy_state_out(const FicSen0696_t *ctx, sen0696_state_t *out_state)
{
    if (out_state != nullptr) {
        if (ctx != nullptr) {
            *out_state = ctx->state;
        } else {
            memset(out_state, 0, sizeof(*out_state));
        }
    }
}

bool sen0696_probe_and_configure(FicSen0696_t *ctx, sen0696_state_t *out_state)
{
    const bool ok_vid = sen0696_read_u16(ctx, kRegVid, &ctx->state.vid);
    const bool ok_pid = sen0696_read_u16(ctx, kRegPid, &ctx->state.pid);
    (void)sen0696_read_u16(ctx, kRegFwVersion, &ctx->state.fw_version);

    ctx->state.id_valid = ok_vid && ok_pid && (ctx->state.pid >= SEN0696_MIN_10DOF_PID);
    if (!ctx->state.id_valid) {
        if (ctx->owns_global_beam_handle) {
            i2c_bsp_detach_beam_imu();
            ctx->owns_global_beam_handle = false;
        }
        ctx->dev_handle_ptr = nullptr;
        copy_state_out(ctx, out_state);
        return false;
    }

    ctx->state.present    = true;
    ctx->state.configured = sen0696_configure_device(ctx);

    // Allow BMI323 + BMM350 + BMP581 ODR conversion cycles after wakeup before first sample
    vTaskDelay(pdMS_TO_TICKS(120));
    sen0696_read_acc_gyr_block(ctx);
    sen0696_read_mag_block(ctx);
    sen0696_read_press_block(ctx, true);

    ctx->state.valid = ctx->state.accel_gyro_valid && ctx->state.mag_valid && ctx->state.press_valid;
    if (ctx->state.valid) {
        ctx->state.last_sample_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    }

    copy_state_out(ctx, out_state);
    return ctx->state.valid;
}

} // namespace

uint8_t sen0696_detect_i2c_addr_on_bus(uint8_t bus_port, uint8_t addr_min, uint8_t addr_max)
{
    if (bus_port > 1U || addr_min > addr_max || addr_max > 0x77U) {
        return 0U;
    }

    for (uint8_t addr = addr_min; addr <= addr_max; ++addr) {
        if (i2c_bsp_probe_optional_addr_port(bus_port, addr)) {
            return addr;
        }
    }
    return 0U;
}

uint8_t sen0696_detect_i2c_addr(uint8_t addr_min, uint8_t addr_max)
{
    uint8_t addr = sen0696_detect_i2c_addr_on_bus(1U, addr_min, addr_max);
    if (addr != 0U) {
        return addr;
    }
    return sen0696_detect_i2c_addr_on_bus(0U, addr_min, addr_max);
}

void fic_sen0696_reset(FicSen0696_t *ctx)
{
    if (ctx == nullptr) {
        return;
    }
    memset(ctx, 0, sizeof(*ctx));
}

bool fic_sen0696_init_with_device(FicSen0696_t *ctx,
                                  uint8_t i2c_bus,
                                  uint8_t i2c_addr,
                                  i2c_master_dev_handle_t dev_handle,
                                  sen0696_state_t *out_state)
{
    if (ctx == nullptr) {
        copy_state_out(nullptr, out_state);
        return false;
    }
    fic_sen0696_reset(ctx);
    if (i2c_bus > 1U || i2c_addr == 0U || dev_handle == nullptr) {
        copy_state_out(ctx, out_state);
        return false;
    }

    ctx->owned_dev_handle        = dev_handle;
    ctx->dev_handle_ptr          = &ctx->owned_dev_handle;
    ctx->owns_global_beam_handle = false;
    ctx->state.i2c_bus           = i2c_bus;
    ctx->state.i2c_addr          = i2c_addr;
    return sen0696_probe_and_configure(ctx, out_state);
}

bool fic_sen0696_init(FicSen0696_t *ctx, uint8_t addr_min, uint8_t addr_max, sen0696_state_t *out_state)
{
    if (ctx == nullptr) {
        copy_state_out(nullptr, out_state);
        return false;
    }
    fic_sen0696_reset(ctx);

    uint8_t bus = 1U;
    uint8_t addr = (user_i2c_port1_handle != nullptr)
                       ? sen0696_detect_i2c_addr_on_bus(1U, addr_min, addr_max)
                       : 0U;
    if (addr == 0U && user_i2c_port0_handle != nullptr) {
        bus  = 0U;
        addr = sen0696_detect_i2c_addr_on_bus(0U, addr_min, addr_max);
    }
    if (addr == 0U) {
        copy_state_out(ctx, out_state);
        return false;
    }

    ctx->state.i2c_bus  = bus;
    ctx->state.i2c_addr = addr;

    if (!i2c_bsp_attach_beam_imu_port(bus, addr)) {
        copy_state_out(ctx, out_state);
        return false;
    }

    // Track i2c_bsp's managed beam_imu_dev_handle pointer so bus reinit stays transparent
    ctx->dev_handle_ptr          = &beam_imu_dev_handle;
    ctx->owns_global_beam_handle = true;
    return sen0696_probe_and_configure(ctx, out_state);
}

bool fic_sen0696_is_present(const FicSen0696_t *ctx)
{
    return (ctx != nullptr) && ctx->state.present;
}

uint8_t fic_sen0696_get_i2c_bus(const FicSen0696_t *ctx)
{
    return (ctx != nullptr) ? ctx->state.i2c_bus : 0U;
}

uint8_t fic_sen0696_get_consec_fails(const FicSen0696_t *ctx)
{
    return (ctx != nullptr) ? ctx->state.consec_fails : 0U;
}

bool fic_sen0696_poll_step(FicSen0696_t *ctx, uint8_t step, sen0696_state_t *out_state)
{
    if (ctx == nullptr || !ctx->state.present) {
        copy_state_out(ctx, out_state);
        return false;
    }

    sen0696_state_t *st = &ctx->state;
    if (step == 0U) {
        // When repeatedly failing (e.g. unplugged cable), check NACK cleanly before spending 10 ms waits
        ctx->link_suppressed = (st->consec_fails >= 15U) &&
                               !i2c_bsp_probe_optional_addr_port(st->i2c_bus, st->i2c_addr);
        if (ctx->link_suppressed) {
            st->accel_gyro_valid = false;
            st->mag_valid        = false;
            st->press_valid      = false;
        } else {
            sen0696_read_acc_gyr_block(ctx);
        }
        return false;
    }

    if (step == 1U) {
        if (!ctx->link_suppressed) {
            sen0696_read_mag_block(ctx);
        }
        return false;
    }

    // Step 2: finalize cycle with barometer read & health tracking
    if (!ctx->link_suppressed) {
        sen0696_read_press_block(ctx, false);
    }

    if (!ctx->link_suppressed && st->accel_gyro_valid && st->mag_valid && st->press_valid) {
        st->valid          = true;
        st->consec_fails   = 0U;
        st->last_sample_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    } else {
        st->valid = false;
        if (st->consec_fails < 255U) {
            st->consec_fails++;
        }
        if (!ctx->link_suppressed && (st->consec_fails % kReconfigFailMod) == 0U) {
            st->configured = sen0696_configure_device(ctx);
        }
    }

    copy_state_out(ctx, out_state);
    return st->valid;
}

bool fic_sen0696_poll_fast(FicSen0696_t *ctx, bool read_baro, sen0696_state_t *out_state)
{
    if (ctx == nullptr || !ctx->state.present) {
        copy_state_out(ctx, out_state);
        return false;
    }

    sen0696_state_t *st = &ctx->state;
    ctx->link_suppressed = (st->consec_fails >= 15U) &&
                           !i2c_bsp_probe_optional_addr_port(st->i2c_bus, st->i2c_addr);
    if (ctx->link_suppressed) {
        st->accel_gyro_valid = false;
        st->mag_valid        = false;
        st->press_valid      = false;
        st->valid            = false;
        if (st->consec_fails < 255U) {
            st->consec_fails++;
        }
        copy_state_out(ctx, out_state);
        return false;
    }

    sen0696_read_acc_gyr_block(ctx);
    if (read_baro) {
        sen0696_read_press_block(ctx, false);
    } else {
        sen0696_read_mag_block(ctx);
    }

    const bool aux_ok = read_baro ? st->press_valid : st->mag_valid;
    if (st->accel_gyro_valid && aux_ok) {
        st->valid          = true;
        st->consec_fails   = 0U;
        st->last_sample_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    } else {
        st->valid = false;
        if (st->consec_fails < 255U) {
            st->consec_fails++;
        }
        if ((st->consec_fails % kReconfigFailMod) == 0U) {
            st->configured = sen0696_configure_device(ctx);
        }
    }

    copy_state_out(ctx, out_state);
    return st->valid;
}

bool fic_sen0696_poll(FicSen0696_t *ctx, sen0696_state_t *out_state)
{
    (void)fic_sen0696_poll_step(ctx, 0U, nullptr);
    (void)fic_sen0696_poll_step(ctx, 1U, nullptr);
    return fic_sen0696_poll_step(ctx, 2U, out_state);
}
