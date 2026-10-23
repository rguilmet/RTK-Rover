#include "axp2101_bsp.h"
#include "../i2c_bsp/i2c_bsp.h"
#include <esp_err.h>

// AXP2101 Core Telemetry & ADC Registers (per X-Powers AXP2101 Datasheet V1.0)
#define AXP2101_REG_STATUS1       0x00 // Bit 5: VBUS good, Bit 3: BAT present
#define AXP2101_REG_STATUS2       0x01 // Bits [7:5] Charge status (001=charging)
#define AXP2101_REG_IC_TYPE       0x03
#define AXP2101_REG_ADC_EN        0x30 // Bit 0: VBAT ADC Enable, Bit 2: VBUS ADC Enable
#define AXP2101_REG_VBAT_H        0x34 // 14-bit VBAT mV: ((H & 0x3F) << 8) | L
#define AXP2101_REG_VBAT_L        0x35
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
        // Enable VBAT, TS, VBUS, VSYS ADC channels
        uint8_t adc_en = 0x0F;
        i2c_write_buff(axp2101_dev_handle, AXP2101_REG_ADC_EN, &adc_en, 1);
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
        return;
    }

    uint8_t st[2] = {0};
    if (i2c_read_buff(axp2101_dev_handle, AXP2101_REG_STATUS1, st, 2) == ESP_OK) {
        status->vbus_good   = (st[0] & (1 << 5)) != 0;
        uint8_t chg_state   = (st[1] >> 5) & 0x07;
        status->is_charging = (chg_state == 0x01 || chg_state == 0x02);
    }

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
