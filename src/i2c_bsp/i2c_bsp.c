#include <stdio.h>
#include "i2c_bsp.h"
#include "../../user_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

i2c_master_bus_handle_t user_i2c_port0_handle = NULL;
i2c_master_dev_handle_t disp_touch_dev_handle = NULL;
i2c_master_dev_handle_t rtc_dev_handle        = NULL;
i2c_master_dev_handle_t imu_dev_handle        = NULL;
i2c_master_dev_handle_t axp2101_dev_handle    = NULL;

static SemaphoreHandle_t i2c_bus_mutex = NULL;
static uint32_t i2c_data_pdMS_TICKS = 0;
static uint32_t i2c_done_pdMS_TICKS = 0;

bool i2c_bsp_lock(uint32_t timeout_ms)
{
    if (i2c_bus_mutex == NULL) {
        return true;
    }
    const TickType_t ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(i2c_bus_mutex, ticks) == pdTRUE;
}

void i2c_bsp_unlock(void)
{
    if (i2c_bus_mutex != NULL) {
        xSemaphoreGiveRecursive(i2c_bus_mutex);
    }
}

void i2c_master_Init(void)
{
    if (user_i2c_port0_handle != NULL) {
        return;
    }
    i2c_bus_mutex = xSemaphoreCreateRecursiveMutex();
    i2c_data_pdMS_TICKS = pdMS_TO_TICKS(50);
    i2c_done_pdMS_TICKS = pdMS_TO_TICKS(50);

    // ESP32-S3-Touch-LCD-3.5B shares I2C_NUM_0 on IO7 (SCL) and IO8 (SDA)
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = ROVER_PIN_I2C0_SDA,
        .scl_io_num = ROVER_PIN_I2C0_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
        },
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &user_i2c_port0_handle));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ROVER_I2C_ADDR_RTC,
        .scl_speed_hz = 400000,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(user_i2c_port0_handle, &dev_cfg, &rtc_dev_handle));

    dev_cfg.device_address = ROVER_I2C_ADDR_IMU;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(user_i2c_port0_handle, &dev_cfg, &imu_dev_handle));

    dev_cfg.device_address = ROVER_I2C_ADDR_TOUCH;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(user_i2c_port0_handle, &dev_cfg, &disp_touch_dev_handle));

    dev_cfg.device_address = ROVER_I2C_ADDR_AXP2101;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(user_i2c_port0_handle, &dev_cfg, &axp2101_dev_handle));
}

bool i2c_bsp_probe_addr(uint8_t addr_7bit)
{
    if (user_i2c_port0_handle == NULL) return false;
    if (!i2c_bsp_lock(100)) return false;
    esp_err_t err = i2c_master_probe(user_i2c_port0_handle, addr_7bit, 50);
    i2c_bsp_unlock();
    return (err == ESP_OK);
}

uint8_t i2c_write_buff(i2c_master_dev_handle_t dev_handle, int reg, uint8_t *buf, uint8_t len)
{
    if (!i2c_bsp_lock(100)) {
        return ESP_ERR_TIMEOUT;
    }
    uint8_t ret = i2c_master_bus_wait_all_done(user_i2c_port0_handle, i2c_done_pdMS_TICKS);
    if (ret == ESP_OK) {
        if (reg == -1) {
            ret = i2c_master_transmit(dev_handle, buf, len, i2c_data_pdMS_TICKS);
        } else {
            uint8_t pbuf[len + 1];
            pbuf[0] = (uint8_t)reg;
            for (uint8_t i = 0; i < len; i++) {
                pbuf[i + 1] = buf[i];
            }
            ret = i2c_master_transmit(dev_handle, pbuf, len + 1, i2c_data_pdMS_TICKS);
        }
    }
    i2c_bsp_unlock();
    return ret;
}

uint8_t i2c_master_write_read_dev(i2c_master_dev_handle_t dev_handle, uint8_t *writeBuf, uint8_t writeLen, uint8_t *readBuf, uint8_t readLen)
{
    if (!i2c_bsp_lock(100)) {
        return ESP_ERR_TIMEOUT;
    }
    uint8_t ret = i2c_master_bus_wait_all_done(user_i2c_port0_handle, i2c_done_pdMS_TICKS);
    if (ret == ESP_OK) {
        ret = i2c_master_transmit_receive(dev_handle, writeBuf, writeLen, readBuf, readLen, i2c_data_pdMS_TICKS);
    }
    i2c_bsp_unlock();
    return ret;
}

uint8_t i2c_read_buff(i2c_master_dev_handle_t dev_handle, int reg, uint8_t *buf, uint8_t len)
{
    if (!i2c_bsp_lock(100)) {
        return ESP_ERR_TIMEOUT;
    }
    uint8_t ret = i2c_master_bus_wait_all_done(user_i2c_port0_handle, i2c_done_pdMS_TICKS);
    if (ret == ESP_OK) {
        if (reg == -1) {
            ret = i2c_master_receive(dev_handle, buf, len, i2c_data_pdMS_TICKS);
        } else {
            uint8_t addr = (uint8_t)reg;
            ret = i2c_master_transmit_receive(dev_handle, &addr, 1, buf, len, i2c_data_pdMS_TICKS);
        }
    }
    i2c_bsp_unlock();
    return ret;
}
