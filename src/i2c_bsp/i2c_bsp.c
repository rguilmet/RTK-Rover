#include <stdio.h>
#include "i2c_bsp.h"
#include "../../user_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"

i2c_master_bus_handle_t user_i2c_port0_handle = NULL;
i2c_master_dev_handle_t disp_touch_dev_handle = NULL;
i2c_master_dev_handle_t rtc_dev_handle        = NULL;
i2c_master_dev_handle_t imu_dev_handle        = NULL;
i2c_master_dev_handle_t axp2101_dev_handle    = NULL;
i2c_master_dev_handle_t mag_dev_handle        = NULL;

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

static const char* i2c_dev_name(i2c_master_dev_handle_t h)
{
    if (h == disp_touch_dev_handle) return "Touch(0x3B)";
    if (h == imu_dev_handle)        return "IMU(0x6B)";
    if (h == rtc_dev_handle)        return "RTC(0x51)";
    if (h == axp2101_dev_handle)    return "AXP2101(0x34)";
    if (h == mag_dev_handle)        return "MAG(0x30)";
    return "I2C_DEV";
}

static void i2c_bus_recover_locked(const char *action, i2c_master_dev_handle_t dev_handle, int reg, esp_err_t err)
{
    static uint32_t last_warn_ms = 0;
    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if (now - last_warn_ms >= 1000) {
        last_warn_ms = now;
        if (reg >= 0) {
            printf("[I2C WARN] %s failed on %s (reg=0x%02X, err=0x%X) -> recovering bus\n",
                   action, i2c_dev_name(dev_handle), (uint8_t)reg, (unsigned int)err);
        } else {
            printf("[I2C WARN] %s failed on %s (err=0x%X) -> recovering bus\n",
                   action, i2c_dev_name(dev_handle), (unsigned int)err);
        }
    }
    if (user_i2c_port0_handle != NULL) {
        i2c_master_bus_reset(user_i2c_port0_handle);
        // Settle delay while holding mutex so subsequent tasks do not collide with recovering lines
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// NXP I2C Spec UM10204 Section 3.1.16 Bus Clear:
// Because the AXP2101 LiPo UPS keeps all 6 I2C0 slave chips continuously powered across
// ESP32 warm resets (esptool flash reset, RTC_SW_CPU_RST, or RST button), a reset mid-read
// leaves a slave holding SDA (GPIO 8) LOW waiting for SCL clocks. Clocking 18 SCL pulses
// with SDA released HIGH followed by an explicit STOP cleanly releases any wedged slave.
void i2c_bsp_gpio_bus_clear(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << ROVER_PIN_I2C0_SCL) | (1ULL << ROVER_PIN_I2C0_SDA),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(ROVER_PIN_I2C0_SCL, 1);
    gpio_set_level(ROVER_PIN_I2C0_SDA, 1);
    esp_rom_delay_us(200);

    // Clock up to 18 SCL pulses (~50 kHz) so any interrupted slave shifts out remaining bits + NACK
    for (int i = 0; i < 18; i++) {
        gpio_set_level(ROVER_PIN_I2C0_SCL, 0);
        esp_rom_delay_us(10);
        gpio_set_level(ROVER_PIN_I2C0_SCL, 1);
        esp_rom_delay_us(10);
    }

    // Generate explicit I2C STOP condition (SDA LOW -> SCL HIGH -> SDA HIGH)
    gpio_set_level(ROVER_PIN_I2C0_SCL, 0);
    esp_rom_delay_us(10);
    gpio_set_level(ROVER_PIN_I2C0_SDA, 0);
    esp_rom_delay_us(10);
    gpio_set_level(ROVER_PIN_I2C0_SCL, 1);
    esp_rom_delay_us(10);
    gpio_set_level(ROVER_PIN_I2C0_SDA, 1);
    esp_rom_delay_us(200);
}

void i2c_bsp_recover_bus(void)
{
    if (!i2c_bsp_lock(250)) {
        return;
    }
    if (user_i2c_port0_handle != NULL) {
        i2c_master_bus_reset(user_i2c_port0_handle);
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    i2c_bsp_unlock();
}

void i2c_master_Init(void)
{
    if (user_i2c_port0_handle != NULL) {
        return;
    }
    if (i2c_bus_mutex == NULL) {
        i2c_bus_mutex = xSemaphoreCreateRecursiveMutex();
    }
    // Fast 25ms timeout prevents noticeable UI stutter if a transient bus stretch occurs
    i2c_data_pdMS_TICKS = 25;
    i2c_done_pdMS_TICKS = 25;

    // Clear any slave holding SDA low across warm/UPS reset before attaching hardware I2C master
    i2c_bsp_gpio_bus_clear();

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
    i2c_master_bus_reset(user_i2c_port0_handle);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Match Waveshare 01_factory 400 kHz across all I2C0 devices so i2c_master never reconfigures bus clock mid-stream
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = ROVER_I2C_ADDR_RTC,
        .scl_speed_hz = 400000,
        .scl_wait_us = 20000, // Allow up to 20ms for slave clock stretching (capacitive touch & IMU DSP)
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(user_i2c_port0_handle, &dev_cfg, &rtc_dev_handle));

    dev_cfg.device_address = ROVER_I2C_ADDR_IMU;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(user_i2c_port0_handle, &dev_cfg, &imu_dev_handle));

    dev_cfg.device_address = ROVER_I2C_ADDR_TOUCH;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(user_i2c_port0_handle, &dev_cfg, &disp_touch_dev_handle));

    dev_cfg.device_address = ROVER_I2C_ADDR_AXP2101;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(user_i2c_port0_handle, &dev_cfg, &axp2101_dev_handle));

    dev_cfg.device_address = ROVER_I2C_ADDR_MAG;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(user_i2c_port0_handle, &dev_cfg, &mag_dev_handle));
}

void i2c_bsp_reinit_bus(void)
{
    if (!i2c_bsp_lock(500)) {
        return;
    }
    if (rtc_dev_handle)        { i2c_master_bus_rm_device(rtc_dev_handle);        rtc_dev_handle = NULL; }
    if (imu_dev_handle)        { i2c_master_bus_rm_device(imu_dev_handle);        imu_dev_handle = NULL; }
    if (disp_touch_dev_handle) { i2c_master_bus_rm_device(disp_touch_dev_handle); disp_touch_dev_handle = NULL; }
    if (axp2101_dev_handle)    { i2c_master_bus_rm_device(axp2101_dev_handle);    axp2101_dev_handle = NULL; }
    if (mag_dev_handle)        { i2c_master_bus_rm_device(mag_dev_handle);        mag_dev_handle = NULL; }
    if (user_i2c_port0_handle) { i2c_del_master_bus(user_i2c_port0_handle);      user_i2c_port0_handle = NULL; }
    vTaskDelay(pdMS_TO_TICKS(10));
    i2c_master_Init();
    i2c_bsp_unlock();
}

bool i2c_bsp_probe_addr(uint8_t addr_7bit)
{
    if (user_i2c_port0_handle == NULL) return false;
    if (!i2c_bsp_lock(200)) return false;
    esp_err_t err = i2c_master_probe(user_i2c_port0_handle, addr_7bit, 50);
    if (err != ESP_OK) {
        i2c_bus_recover_locked("Probe", NULL, addr_7bit, err);
    }
    i2c_bsp_unlock();
    return (err == ESP_OK);
}

bool i2c_bsp_probe_optional_addr(uint8_t addr_7bit)
{
    if (user_i2c_port0_handle == NULL) return false;
    if (!i2c_bsp_lock(200)) return false;
    esp_err_t err = i2c_master_probe(user_i2c_port0_handle, addr_7bit, 25);
    i2c_bsp_unlock();
    return (err == ESP_OK);
}

uint8_t i2c_write_buff(i2c_master_dev_handle_t dev_handle, int reg, uint8_t *buf, uint8_t len)
{
    if (!i2c_bsp_lock(200)) {
        return 1;
    }
    esp_err_t ret;
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
    if (ret != ESP_OK) {
        i2c_bus_recover_locked("Write", dev_handle, reg, ret);
    }
    i2c_bsp_unlock();
    return (ret == ESP_OK) ? 0 : 1;
}

uint8_t i2c_master_write_read_dev(i2c_master_dev_handle_t dev_handle, uint8_t *writeBuf, uint8_t writeLen, uint8_t *readBuf, uint8_t readLen)
{
    if (!i2c_bsp_lock(200)) {
        return 1;
    }
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, writeBuf, writeLen, readBuf, readLen, i2c_data_pdMS_TICKS);
    if (ret != ESP_OK) {
        i2c_bus_recover_locked("WriteRead", dev_handle, -1, ret);
    }
    i2c_bsp_unlock();
    return (ret == ESP_OK) ? 0 : 1;
}

uint8_t i2c_read_buff(i2c_master_dev_handle_t dev_handle, int reg, uint8_t *buf, uint8_t len)
{
    if (!i2c_bsp_lock(200)) {
        return 1;
    }
    esp_err_t ret;
    if (reg == -1) {
        ret = i2c_master_receive(dev_handle, buf, len, i2c_data_pdMS_TICKS);
    } else {
        uint8_t addr = (uint8_t)reg;
        ret = i2c_master_transmit_receive(dev_handle, &addr, 1, buf, len, i2c_data_pdMS_TICKS);
    }
    if (ret != ESP_OK) {
        i2c_bus_recover_locked("Read", dev_handle, reg, ret);
    }
    i2c_bsp_unlock();
    return (ret == ESP_OK) ? 0 : 1;
}
