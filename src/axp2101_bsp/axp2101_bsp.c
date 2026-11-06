#include <stdio.h>
#include "axp2101_bsp.h"
#include "../i2c_bsp/i2c_bsp.h"
#include <esp_err.h>

// AXP2101 Core Telemetry & LDO Registers (per X-Powers AXP2101 Datasheet V1.0 & Waveshare 3.5B BSP)
#define AXP2101_REG_STATUS1       0x00 // Bit 5: VBUS good, Bit 3: BAT present
#define AXP2101_REG_STATUS2       0x01 // Bits [7:5] Charge status (001=charging)
#define AXP2101_REG_IC_TYPE       0x03 // Expected Chip ID = 0x4A
#define AXP2101_REG_COMM_CFG      0x10 // Bit 0: Soft PWROFF
#define AXP2101_REG_ADC_EN        0x30 // Bit 0: VBAT, Bit 1: TS (Disable=0), Bit 2: VBUS, Bit 3: VSYS
#define AXP2101_REG_VBAT_H        0x34 // 14-bit VBAT mV: ((H & 0x3F) << 8) | L
#define AXP2101_REG_VBAT_L        0x35
#define AXP2101_REG_VBUS_H        0x38 // 14-bit VBUS mV: ((H & 0x3F) << 8) | L
#define AXP2101_REG_VBUS_L        0x39
#define AXP2101_REG_ICC_CFG       0x62 // Bits [4:0] Constant-current charge limit (0x0B = 500mA)
#define AXP2101_REG_DC_ONOFF      0x80 // Bits [4:0] = DCDC1..5 Enable
#define AXP2101_REG_LDO_ONOFF0    0x90 // Bits [3:0]=ALDO1..4, [5:4]=BLDO1..2, [6]=CPUSLDO, [7]=DLDO1
#define AXP2101_REG_LDO_ONOFF1    0x91 // Bit 0 = DLDO2 Enable
#define AXP2101_REG_ALDO1_VOL     0x92 // 0x1C = 3300 mV ((3300 - 500) / 100)
#define AXP2101_REG_ALDO2_VOL     0x93
#define AXP2101_REG_ALDO3_VOL     0x94
#define AXP2101_REG_ALDO4_VOL     0x95
#define AXP2101_REG_BLDO1_VOL     0x96 // 0x0A = 1500 mV (or 0x1C = 3300 mV)
#define AXP2101_REG_BLDO2_VOL     0x97 // 0x17 = 2800 mV (or 0x1C = 3300 mV)
#define AXP2101_REG_CPUSLDO_VOL   0x98 // 0x0A = 1000 mV
#define AXP2101_REG_DLDO1_VOL     0x99 // 0x1C = 3300 mV
#define AXP2101_REG_DLDO2_VOL     0x9A // 0x1C = 3300 mV
#define AXP2101_REG_BAT_PERCENT   0xA4 // Fuel gauge percentage (0..100%)

static bool axp_online = false;

void axp2101_bsp_init(void)
{
    if (axp2101_dev_handle == NULL) {
        return;
    }

    uint8_t chip_id = 0;
    if (i2c_read_buff(axp2101_dev_handle, AXP2101_REG_IC_TYPE, &chip_id, 1) == ESP_OK) {
        axp_online = true;
        // Enable VBAT (bit 0), VBUS (bit 2), VSYS (bit 3) ADCs; disable TS pin (bit 1 = 0) per Waveshare BSP
        uint8_t adc_en = 0x0D;
        i2c_write_buff(axp2101_dev_handle, AXP2101_REG_ADC_EN, &adc_en, 1);

        // Configure and enable all LDO rails per Waveshare 01_factory/components/esp_bsp/bsp_axp2101.cpp
        uint8_t vol_3v3  = 0x1C; // 3300 mV
        uint8_t vol_1v5  = 0x0A; // 1500 mV
        uint8_t vol_2v8  = 0x17; // 2800 mV
        uint8_t vol_1v0  = 0x0A; // 1000 mV (CPUSLDO)
        i2c_write_buff(axp2101_dev_handle, AXP2101_REG_ALDO1_VOL,   &vol_3v3, 1);
        i2c_write_buff(axp2101_dev_handle, AXP2101_REG_ALDO2_VOL,   &vol_3v3, 1);
        i2c_write_buff(axp2101_dev_handle, AXP2101_REG_ALDO3_VOL,   &vol_3v3, 1);
        i2c_write_buff(axp2101_dev_handle, AXP2101_REG_ALDO4_VOL,   &vol_3v3, 1);
        i2c_write_buff(axp2101_dev_handle, AXP2101_REG_BLDO1_VOL,   &vol_1v5, 1);
        i2c_write_buff(axp2101_dev_handle, AXP2101_REG_BLDO2_VOL,   &vol_2v8, 1);
        i2c_write_buff(axp2101_dev_handle, AXP2101_REG_CPUSLDO_VOL, &vol_1v0, 1);
        i2c_write_buff(axp2101_dev_handle, AXP2101_REG_DLDO1_VOL,   &vol_3v3, 1);
        i2c_write_buff(axp2101_dev_handle, AXP2101_REG_DLDO2_VOL,   &vol_3v3, 1);

        // Enable ALDO1..4, BLDO1..2, CPUSLDO, DLDO1 (0x90 = 0xFF) and DLDO2 (0x91 |= 0x01)
        uint8_t ldo0 = 0xFF;
        uint8_t ldo1 = 0x01;
        i2c_write_buff(axp2101_dev_handle, AXP2101_REG_LDO_ONOFF0, &ldo0, 1);
        i2c_write_buff(axp2101_dev_handle, AXP2101_REG_LDO_ONOFF1, &ldo1, 1);

        uint8_t dc_en = 0;
        if (i2c_read_buff(axp2101_dev_handle, AXP2101_REG_DC_ONOFF, &dc_en, 1) == ESP_OK) {
            dc_en |= 0x1F; // Ensure DCDC1..5 enabled
            i2c_write_buff(axp2101_dev_handle, AXP2101_REG_DC_ONOFF, &dc_en, 1);
        }

        // Configure safe 500mA charge current (0.5C for 1000mAh battery) per AXP2101 REG 0x62 (01011b = 0x0B)
        uint8_t icc_500ma = 0x0B;
        i2c_write_buff(axp2101_dev_handle, AXP2101_REG_ICC_CFG, &icc_500ma, 1);

        Axp2101Status_t st = {};
        axp2101_bsp_read(&st);
        printf("[PMIC] AXP2101 (0x34) ONLINE — ChipID: 0x%02X | LDO0:0x%02X DC:0x%02X | VBAT: %.2fV (%u%%) | VBUS: %.2fV (%s) | Charging: %s\n",
               chip_id, ldo0, dc_en, st.battery_voltage_v, st.battery_percent,
               st.vbus_voltage_v, st.vbus_good ? "YES" : "NO", st.is_charging ? "YES" : "NO");
    } else {
        printf("[PMIC] AXP2101 (0x34) FAIL — Not responding on I2C0\n");
    }
}

void axp2101_bsp_read(Axp2101Status_t *status)
{
    if (!status) return;
    status->online = axp_online;
    if (!axp_online || axp2101_dev_handle == NULL) {
        status->battery_voltage_v = 0.0f;
        status->battery_percent   = 0;
        status->is_charging       = false;
        status->vbus_good         = false;
        status->vbus_voltage_v    = 0.0f;
        return;
    }

    uint8_t st[2] = {0};
    bool raw_vbus_good = false;
    if (i2c_read_buff(axp2101_dev_handle, AXP2101_REG_STATUS1, st, 2) == ESP_OK) {
        raw_vbus_good       = (st[0] & (1 << 5)) != 0;
        uint8_t chg_state   = (st[1] >> 5) & 0x07;
        status->is_charging = (chg_state == 0x01 || chg_state == 0x02);
    }

    uint8_t vbus_raw[2] = {0};
    float raw_vbus_v = 0.0f;
    if (i2c_read_buff(axp2101_dev_handle, AXP2101_REG_VBUS_H, vbus_raw, 2) == ESP_OK) {
        uint16_t vbus_mv = ((uint16_t)(vbus_raw[0] & 0x3F) << 8) | vbus_raw[1];
        raw_vbus_v = (float)vbus_mv / 1000.0f;
        status->vbus_voltage_v = raw_vbus_v;
    } else {
        raw_vbus_v = status->vbus_voltage_v; // Retain previous valid reading on transient I2C bus contention
    }

    // Debounce VBUS presence:
    // USB power is considered present if hardware bit is set OR ADC measures >= 4.0V
    static uint8_t s_vbus_loss_debounce = 0;
    static bool s_filtered_vbus_good = true;

    bool vbus_detected = raw_vbus_good || (raw_vbus_v >= 4.0f);
    if (vbus_detected) {
        s_vbus_loss_debounce = 0;
        s_filtered_vbus_good = true;
    } else {
        // Require 2 consecutive missing cycles (>= 4s) of BOTH bit=0 and VBUS < 4.0V
        // before declaring VBUS disconnected
        if (s_vbus_loss_debounce < 2) {
            s_vbus_loss_debounce++;
        } else {
            s_filtered_vbus_good = false;
        }
    }
    status->vbus_good = s_filtered_vbus_good;

    uint8_t vbat_raw[2] = {0};
    if (i2c_read_buff(axp2101_dev_handle, AXP2101_REG_VBAT_H, vbat_raw, 2) == ESP_OK) {
        uint16_t mv = ((uint16_t)(vbat_raw[0] & 0x3F) << 8) | vbat_raw[1];
        status->battery_voltage_v = (float)mv / 1000.0f;
    }

    uint8_t pct = 0;
    if (i2c_read_buff(axp2101_dev_handle, AXP2101_REG_BAT_PERCENT, &pct, 1) == ESP_OK) {
        if (pct <= 100) {
            status->battery_percent = pct;
        } else if (status->battery_voltage_v > 3.0f) {
            // Linear fallback if fuel gauge is uncalibrated
            float p = (status->battery_voltage_v - 3.30f) / (4.18f - 3.30f) * 100.0f;
            if (p < 0.0f) p = 0.0f;
            if (p > 100.0f) p = 100.0f;
            status->battery_percent = (uint8_t)p;
        }
    }
}

void axp2101_bsp_power_off(void)
{
    if (axp2101_dev_handle == NULL || !axp_online) return;
    printf("[PMIC] Initiating AXP2101 Soft PWROFF...\n");
    uint8_t reg_val = 0;
    if (i2c_read_buff(axp2101_dev_handle, AXP2101_REG_COMM_CFG, &reg_val, 1) == ESP_OK) {
        reg_val |= 0x01; // Bit 0 = Soft PWROFF
        i2c_write_buff(axp2101_dev_handle, AXP2101_REG_COMM_CFG, &reg_val, 1);
    }
}
