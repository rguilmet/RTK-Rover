#include <stdio.h>
#include "i2c_bsp.h"
#include "../storage/serial_logger.h"
#include "../../user_config.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h"
#include "esp_rom_sys.h"
#include "soc/gpio_sig_map.h"

i2c_master_bus_handle_t user_i2c_port0_handle = NULL;
i2c_master_bus_handle_t user_i2c_port1_handle = NULL;
i2c_master_dev_handle_t disp_touch_dev_handle = NULL;
i2c_master_dev_handle_t rtc_dev_handle        = NULL;
i2c_master_dev_handle_t imu_dev_handle        = NULL;
i2c_master_dev_handle_t axp2101_dev_handle    = NULL;
i2c_master_dev_handle_t mag_dev_handle        = NULL;
i2c_master_dev_handle_t beam_imu_dev_handle   = NULL;

static uint8_t s_beam_imu_addr = 0;
static uint8_t s_beam_imu_bus  = 0;

typedef struct {
    i2c_port_t              port_num;
    gpio_num_t              scl_pin;
    gpio_num_t              sda_pin;
    uint32_t                scl_out_idx;
    uint32_t                scl_in_idx;
    uint32_t                sda_out_idx;
    uint32_t                sda_in_idx;
    i2c_master_bus_handle_t *bus_ptr;
    SemaphoreHandle_t       mutex;
    uint8_t                 consec_timeouts;
    uint32_t                last_gpio_clear_ms;
    uint32_t                last_reinit_ms;
    uint32_t                last_warn_ms;
    uint32_t                err_count;
} i2c_port_state_t;

static i2c_port_state_t s_ports[2] = {
    {
        .port_num = I2C_NUM_0,
        .scl_pin = ROVER_PIN_I2C0_SCL,
        .sda_pin = ROVER_PIN_I2C0_SDA,
        .scl_out_idx = I2CEXT0_SCL_OUT_IDX,
        .scl_in_idx  = I2CEXT0_SCL_IN_IDX,
        .sda_out_idx = I2CEXT0_SDA_OUT_IDX,
        .sda_in_idx  = I2CEXT0_SDA_IN_IDX,
        .bus_ptr = &user_i2c_port0_handle,
        .mutex = NULL,
        .consec_timeouts = 0,
        .last_gpio_clear_ms = 0,
        .last_reinit_ms = 0,
        .last_warn_ms = 0,
        .err_count = 0,
    },
    {
        .port_num = I2C_NUM_1,
        .scl_pin = ROVER_PIN_I2C1_SCL,
        .sda_pin = ROVER_PIN_I2C1_SDA,
        .scl_out_idx = I2CEXT1_SCL_OUT_IDX,
        .scl_in_idx  = I2CEXT1_SCL_IN_IDX,
        .sda_out_idx = I2CEXT1_SDA_OUT_IDX,
        .sda_in_idx  = I2CEXT1_SDA_IN_IDX,
        .bus_ptr = &user_i2c_port1_handle,
        .mutex = NULL,
        .consec_timeouts = 0,
        .last_gpio_clear_ms = 0,
        .last_reinit_ms = 0,
        .last_warn_ms = 0,
        .err_count = 0,
    }
};

#define I2C_BSP_MAX_BUS_HOOKS 4
typedef struct {
    i2c_bsp_bus_hook_t detach_fn;
    i2c_bsp_bus_hook_t reattach_fn;
    void *ctx;
} i2c_bsp_bus_hook_entry_t;

static i2c_bsp_bus_hook_entry_t s_bus_hooks[I2C_BSP_MAX_BUS_HOOKS] = {0};

static uint32_t i2c_data_pdMS_TICKS = 0;
static uint32_t i2c_done_pdMS_TICKS = 0;

bool i2c_bsp_lock_port(uint8_t port, uint32_t timeout_ms)
{
    if (port > 1U) {
        return false;
    }
    SemaphoreHandle_t mtx = s_ports[port].mutex;
    if (mtx == NULL) {
        return true;
    }
    const TickType_t ticks = (timeout_ms == 0) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTakeRecursive(mtx, ticks) == pdTRUE;
}

void i2c_bsp_unlock_port(uint8_t port)
{
    if (port > 1U) {
        return;
    }
    SemaphoreHandle_t mtx = s_ports[port].mutex;
    if (mtx != NULL) {
        xSemaphoreGiveRecursive(mtx);
    }
}

bool i2c_bsp_lock(uint32_t timeout_ms)
{
    return i2c_bsp_lock_port(0U, timeout_ms);
}

void i2c_bsp_unlock(void)
{
    i2c_bsp_unlock_port(0U);
}

static const char* i2c_dev_name(i2c_master_dev_handle_t h)
{
    if (h == disp_touch_dev_handle) return "Touch(0x3B)";
    if (h == imu_dev_handle)        return "IMU(0x6B)";
    if (h == rtc_dev_handle)        return "RTC(0x51)";
    if (h == axp2101_dev_handle)    return "AXP2101(0x34)";
    if (h == mag_dev_handle)        return "MAG(0x30)";
    if (h == beam_imu_dev_handle)   return "10DOF(0x4A)";
    return "I2C_DEV";
}

static bool is_corrupt_sentinel_byte(uint8_t b)
{
    return (b == 0x61U || b == 0x71U);
}

bool i2c_bsp_has_sentinel_run(const uint8_t *buf, size_t len, size_t min_run)
{
    if (buf == NULL || min_run == 0 || len < min_run) {
        return false;
    }
    if (min_run == 1 && is_corrupt_sentinel_byte(buf[0])) {
        return true;
    }
    size_t run = 1;
    for (size_t i = 1; i < len; i++) {
        if (buf[i] == buf[i - 1] && is_corrupt_sentinel_byte(buf[i])) {
            run++;
            if (run >= min_run) {
                return true;
            }
        } else {
            run = 1;
        }
    }
    return false;
}

bool i2c_bsp_has_sentinel_tail(const uint8_t *buf, size_t len, size_t tail_len)
{
    if (buf == NULL || tail_len == 0 || len < tail_len) {
        return false;
    }
    const uint8_t s = buf[len - tail_len];
    if (!is_corrupt_sentinel_byte(s)) {
        return false;
    }
    for (size_t i = len - tail_len + 1; i < len; i++) {
        if (buf[i] != s) {
            return false;
        }
    }
    return true;
}

bool i2c_bsp_register_bus_hooks(i2c_bsp_bus_hook_t detach_fn, i2c_bsp_bus_hook_t reattach_fn, void *ctx)
{
    if (detach_fn == NULL && reattach_fn == NULL) {
        return false;
    }
    if (!i2c_bsp_lock(200)) {
        return false;
    }
    int free_idx = -1;
    for (int i = 0; i < I2C_BSP_MAX_BUS_HOOKS; i++) {
        if (s_bus_hooks[i].ctx == ctx && s_bus_hooks[i].detach_fn == detach_fn && s_bus_hooks[i].reattach_fn == reattach_fn) {
            i2c_bsp_unlock();
            return true;
        }
        if (free_idx < 0 && s_bus_hooks[i].detach_fn == NULL && s_bus_hooks[i].reattach_fn == NULL) {
            free_idx = i;
        }
    }
    if (free_idx < 0) {
        i2c_bsp_unlock();
        return false;
    }
    s_bus_hooks[free_idx].detach_fn = detach_fn;
    s_bus_hooks[free_idx].reattach_fn = reattach_fn;
    s_bus_hooks[free_idx].ctx = ctx;
    i2c_bsp_unlock();
    return true;
}

void i2c_bsp_unregister_bus_hooks(void *ctx)
{
    if (!i2c_bsp_lock(200)) {
        return;
    }
    for (int i = 0; i < I2C_BSP_MAX_BUS_HOOKS; i++) {
        if (s_bus_hooks[i].ctx == ctx) {
            s_bus_hooks[i].detach_fn = NULL;
            s_bus_hooks[i].reattach_fn = NULL;
            s_bus_hooks[i].ctx = NULL;
        }
    }
    i2c_bsp_unlock();
}

static void i2c_bsp_gpio_bus_clear_pins(gpio_num_t scl_pin, gpio_num_t sda_pin);

static bool i2c_bsp_runtime_gpio_bus_clear_locked(uint8_t port, int *sda_gpio_after_out)
{
    i2c_port_state_t *ps = &s_ports[port];
    // Pre-drive HIGH output latch before detaching I2CEXT so switching to open-drain never glitches LOW
    gpio_set_level(ps->scl_pin, 1);
    gpio_set_level(ps->sda_pin, 1);
    esp_rom_gpio_connect_out_signal(ps->scl_pin, SIG_GPIO_OUT_IDX, false, false);
    esp_rom_gpio_connect_out_signal(ps->sda_pin, SIG_GPIO_OUT_IDX, false, false);

    i2c_bsp_gpio_bus_clear_pins(ps->scl_pin, ps->sda_pin);

    int sda_gpio_after = gpio_get_level(ps->sda_pin);
    int scl_gpio_after = gpio_get_level(ps->scl_pin);
    if (sda_gpio_after_out != NULL) {
        *sda_gpio_after_out = sda_gpio_after;
    }

    // Restore open-drain levels and reattach GPIO pins to hardware I2C peripheral
    gpio_set_level(ps->scl_pin, 1);
    gpio_set_level(ps->sda_pin, 1);
    esp_rom_gpio_connect_out_signal(ps->scl_pin, ps->scl_out_idx, false, false);
    esp_rom_gpio_connect_in_signal(ps->scl_pin, ps->scl_in_idx, false);
    esp_rom_gpio_connect_out_signal(ps->sda_pin, ps->sda_out_idx, false, false);
    esp_rom_gpio_connect_in_signal(ps->sda_pin, ps->sda_in_idx, false);
    return (sda_gpio_after == 1 && scl_gpio_after == 1);
}

static void i2c_bus_recover_port_locked(uint8_t port, const char *action, i2c_master_dev_handle_t dev_handle, int reg, esp_err_t err)
{
    if (port > 1U) {
        return;
    }
    i2c_port_state_t *ps = &s_ports[port];
    uint32_t now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    if (ps->err_count < UINT32_MAX) {
        ps->err_count++;
    }
    if (now - ps->last_warn_ms >= 1000) {
        ps->last_warn_ms = now;
        if (reg >= 0) {
            rover_printf("[I2C%u WARN] %s failed on %s (reg=0x%02X, err=0x%X) -> recovering bus\n",
                         (unsigned)port, action, i2c_dev_name(dev_handle), (uint8_t)reg, (unsigned int)err);
        } else {
            rover_printf("[I2C%u WARN] %s failed on %s (err=0x%X) -> recovering bus\n",
                         (unsigned)port, action, i2c_dev_name(dev_handle), (unsigned int)err);
        }
    }

    const bool is_bus_stall = (err == ESP_ERR_TIMEOUT || err == ESP_ERR_INVALID_STATE);
    if (is_bus_stall && ps->consec_timeouts < 255) {
        ps->consec_timeouts++;
    }

    i2c_master_bus_handle_t bus_handle = *ps->bus_ptr;
    if (bus_handle == NULL) {
        return;
    }

    bool escalate_reinit = false;
    int sda_before = gpio_get_level(ps->sda_pin);
    int scl_before = gpio_get_level(ps->scl_pin);

    // Tier 1: Only run open-drain GPIO clocking if SDA or SCL is actually held LOW; never disturb idle SDA=1/SCL=1
    if ((ps->consec_timeouts >= 3 || sda_before == 0 || scl_before == 0) &&
        (now - ps->last_gpio_clear_ms >= 1500U)) {
        ps->last_gpio_clear_ms = now;
        int sda_gpio_after = sda_before;
        if (sda_before == 0 || scl_before == 0) {
            rover_printf("[I2C%u RECOVER] %u consecutive bus stalls -> running GPIO bus-clear (SDA=%d SCL=%d)\n",
                         (unsigned)port, (unsigned int)ps->consec_timeouts, sda_before, scl_before);
            (void)i2c_bsp_runtime_gpio_bus_clear_locked(port, &sda_gpio_after);
        }
        i2c_master_bus_reset(bus_handle);
        vTaskDelay(pdMS_TO_TICKS(5));
        int sda_after = gpio_get_level(ps->sda_pin);
        int scl_after = gpio_get_level(ps->scl_pin);
        if (sda_before == 0 || scl_before == 0) {
            rover_printf("[I2C%u RECOVER] Runtime bus-clear complete (GPIO SDA=%d | I2C SDA=%d SCL=%d)\n",
                         (unsigned)port, sda_gpio_after, sda_after, scl_after);
        }
        if (sda_after == 0 || scl_after == 0 || ps->consec_timeouts >= 6) {
            escalate_reinit = true;
        }
    } else {
        i2c_master_bus_reset(bus_handle);
        vTaskDelay(pdMS_TO_TICKS(5));
        if (ps->consec_timeouts >= 6) {
            escalate_reinit = true;
        }
    }

    // Tier 2: Persistent stall (or peripheral matrix stuck LOW) -> full ESP-IDF I2C peripheral teardown & reinit
    if (escalate_reinit && (now - ps->last_reinit_ms >= 5000U)) {
        ps->last_reinit_ms = now;
        ps->consec_timeouts = 0;
        rover_printf("[I2C%u RECOVER] Persistent stall (SDA=%d SCL=%d) -> running full I2C%u peripheral reinit\n",
                     (unsigned)port, gpio_get_level(ps->sda_pin), gpio_get_level(ps->scl_pin), (unsigned)port);
        if (port == 0U) {
            i2c_bsp_reinit_bus();
        } else {
            i2c_bsp_i2c1_reinit_bus();
        }
        rover_printf("[I2C%u RECOVER] Full I2C%u reinit complete (SDA=%d SCL=%d)\n",
                     (unsigned)port, (unsigned)port, gpio_get_level(ps->sda_pin), gpio_get_level(ps->scl_pin));
    }
}

// NXP I2C Spec UM10204 Section 3.1.16 Bus Clear:
// Because the AXP2101 LiPo UPS keeps all I2C0 slave chips continuously powered across
// ESP32 warm resets, an interrupted read can leave a slave holding SDA LOW.
// Pre-drive HIGH before configuring open-drain to avoid synthetic START glitches, and
// skip clocking completely if SDA and SCL are already idle HIGH.
static void i2c_bsp_gpio_bus_clear_pins(gpio_num_t scl_pin, gpio_num_t sda_pin)
{
    gpio_set_level(scl_pin, 1);
    gpio_set_level(sda_pin, 1);

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << scl_pin) | (1ULL << sda_pin),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(scl_pin, 1);
    gpio_set_level(sda_pin, 1);
    esp_rom_delay_us(50);

    // Do not clock an already-idle bus (avoids shifting slaves during benign stalls)
    if (gpio_get_level(sda_pin) == 1 && gpio_get_level(scl_pin) == 1) {
        return;
    }

    // Clock up to 3 x 9 SCL pulses (~50 kHz), stopping cleanly once SDA releases HIGH and STOP succeeds
    for (int chunk = 0; chunk < 3; chunk++) {
        if (gpio_get_level(sda_pin) == 1) {
            break;
        }
        for (int i = 0; i < 9; i++) {
            gpio_set_level(scl_pin, 0);
            esp_rom_delay_us(10);
            gpio_set_level(scl_pin, 1);
            for (int w = 0; w < 10 && gpio_get_level(scl_pin) == 0; w++) {
                esp_rom_delay_us(10);
            }
            esp_rom_delay_us(10);
        }
        if (gpio_get_level(sda_pin) == 1) {
            gpio_set_level(scl_pin, 0);
            esp_rom_delay_us(10);
            gpio_set_level(sda_pin, 0);
            esp_rom_delay_us(10);
            gpio_set_level(scl_pin, 1);
            esp_rom_delay_us(10);
            gpio_set_level(sda_pin, 1);
            esp_rom_delay_us(100);
            if (gpio_get_level(sda_pin) == 1) {
                break;
            }
        }
    }

    gpio_set_level(sda_pin, 1);
    gpio_set_level(scl_pin, 1);
    esp_rom_delay_us(50);
}

void i2c_bsp_gpio_bus_clear(void)
{
    i2c_bsp_gpio_bus_clear_pins(ROVER_PIN_I2C0_SCL, ROVER_PIN_I2C0_SDA);
}

void i2c_bsp_i2c1_gpio_bus_clear(void)
{
    i2c_bsp_gpio_bus_clear_pins(ROVER_PIN_I2C1_SCL, ROVER_PIN_I2C1_SDA);
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
    if (s_ports[0].mutex == NULL) {
        s_ports[0].mutex = xSemaphoreCreateRecursiveMutex();
    }
    // 40 ms software timeout clears 20 ms .scl_wait_us slave stretching margin plus RTOS jitter
    i2c_data_pdMS_TICKS = 40;
    i2c_done_pdMS_TICKS = 40;

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

void i2c_bsp_i2c1_init(void)
{
    if (user_i2c_port1_handle != NULL) {
        return;
    }
    if (s_ports[1].mutex == NULL) {
        s_ports[1].mutex = xSemaphoreCreateRecursiveMutex();
    }
    if (i2c_data_pdMS_TICKS == 0) {
        i2c_data_pdMS_TICKS = 40;
        i2c_done_pdMS_TICKS = 40;
    }

    // Release any slave holding external I2C1 SDA LOW across resets
    i2c_bsp_i2c1_gpio_bus_clear();

    i2c_master_bus_config_t i2c1_bus_config = {
        .i2c_port = I2C_NUM_1,
        .sda_io_num = ROVER_PIN_I2C1_SDA,
        .scl_io_num = ROVER_PIN_I2C1_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .intr_priority = 0,
        .trans_queue_depth = 0,
        .flags = {
            .enable_internal_pullup = true,
        },
    };
    esp_err_t err = i2c_new_master_bus(&i2c1_bus_config, &user_i2c_port1_handle);
    if (err != ESP_OK || user_i2c_port1_handle == NULL) {
        user_i2c_port1_handle = NULL;
        rover_printf("[I2C1 WARN] i2c_new_master_bus(I2C1 IO47/48) failed err=0x%X\n", (unsigned int)err);
        return;
    }
    i2c_master_bus_reset(user_i2c_port1_handle);
    vTaskDelay(pdMS_TO_TICKS(10));
}

static bool i2c_bsp_attach_beam_imu_locked(uint8_t port, uint8_t addr_7bit)
{
    if (port > 1U || addr_7bit < 0x08U || addr_7bit > 0x77U) {
        return false;
    }
    i2c_master_bus_handle_t bus_handle = *s_ports[port].bus_ptr;
    if (bus_handle == NULL) {
        return false;
    }
    if (beam_imu_dev_handle != NULL && s_beam_imu_addr == addr_7bit && s_beam_imu_bus == port) {
        return true;
    }
    if (beam_imu_dev_handle != NULL) {
        i2c_master_bus_rm_device(beam_imu_dev_handle);
        beam_imu_dev_handle = NULL;
    }

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr_7bit,
        .scl_speed_hz    = (port == 1U) ? ROVER_I2C1_SPEED_HZ : 400000U,
        .scl_wait_us     = 20000,
    };
    if (i2c_master_bus_add_device(bus_handle, &dev_cfg, &beam_imu_dev_handle) != ESP_OK) {
        beam_imu_dev_handle = NULL;
        return false;
    }
    s_beam_imu_addr = addr_7bit;
    s_beam_imu_bus  = port;
    return true;
}

bool i2c_bsp_attach_beam_imu_port(uint8_t port, uint8_t addr_7bit)
{
    if (port > 1U) {
        return false;
    }
    if (beam_imu_dev_handle != NULL && s_beam_imu_bus != port) {
        i2c_bsp_detach_beam_imu();
    }
    if (!i2c_bsp_lock_port(port, 200)) {
        return false;
    }
    bool ok = i2c_bsp_attach_beam_imu_locked(port, addr_7bit);
    i2c_bsp_unlock_port(port);
    return ok;
}

bool i2c_bsp_attach_beam_imu(uint8_t addr_7bit)
{
    return i2c_bsp_attach_beam_imu_port(0U, addr_7bit);
}

void i2c_bsp_detach_beam_imu(void)
{
    uint8_t bus = s_beam_imu_bus;
    if (!i2c_bsp_lock_port(bus, 200)) {
        return;
    }
    if (beam_imu_dev_handle != NULL) {
        i2c_master_bus_rm_device(beam_imu_dev_handle);
        beam_imu_dev_handle = NULL;
    }
    s_beam_imu_addr = 0;
    s_beam_imu_bus  = 0;
    i2c_bsp_unlock_port(bus);
}

uint8_t i2c_bsp_get_beam_imu_bus(void)
{
    return s_beam_imu_bus;
}

uint32_t i2c_bsp_get_bus_err_count(uint8_t port)
{
    return (port <= 1U) ? s_ports[port].err_count : 0U;
}

void i2c_bsp_reinit_bus(void)
{
    if (!i2c_bsp_lock_port(0U, 500)) {
        return;
    }
    for (int i = 0; i < I2C_BSP_MAX_BUS_HOOKS; i++) {
        if (s_bus_hooks[i].detach_fn != NULL && user_i2c_port0_handle != NULL) {
            s_bus_hooks[i].detach_fn(user_i2c_port0_handle, s_bus_hooks[i].ctx);
        }
    }
    if (rtc_dev_handle)        { i2c_master_bus_rm_device(rtc_dev_handle);        rtc_dev_handle = NULL; }
    if (imu_dev_handle)        { i2c_master_bus_rm_device(imu_dev_handle);        imu_dev_handle = NULL; }
    if (disp_touch_dev_handle) { i2c_master_bus_rm_device(disp_touch_dev_handle); disp_touch_dev_handle = NULL; }
    if (axp2101_dev_handle)    { i2c_master_bus_rm_device(axp2101_dev_handle);    axp2101_dev_handle = NULL; }
    if (mag_dev_handle)        { i2c_master_bus_rm_device(mag_dev_handle);        mag_dev_handle = NULL; }
    if (beam_imu_dev_handle && s_beam_imu_bus == 0U) {
        i2c_master_bus_rm_device(beam_imu_dev_handle);
        beam_imu_dev_handle = NULL;
    }

    esp_err_t del_err = ESP_OK;
    if (user_i2c_port0_handle != NULL) {
        del_err = i2c_del_master_bus(user_i2c_port0_handle);
        if (del_err != ESP_OK) {
            rover_printf("[I2C0 WARN] i2c_del_master_bus returned err=0x%X; falling back to bus_reset\n",
                         (unsigned int)del_err);
            i2c_master_bus_reset(user_i2c_port0_handle);
        } else {
            user_i2c_port0_handle = NULL;
        }
    }

    if (user_i2c_port0_handle == NULL) {
        vTaskDelay(pdMS_TO_TICKS(10));
        i2c_master_Init();
    }
    if (s_beam_imu_addr != 0 && s_beam_imu_bus == 0U) {
        (void)i2c_bsp_attach_beam_imu_locked(0U, s_beam_imu_addr);
    }
    for (int i = 0; i < I2C_BSP_MAX_BUS_HOOKS; i++) {
        if (s_bus_hooks[i].reattach_fn != NULL && user_i2c_port0_handle != NULL) {
            s_bus_hooks[i].reattach_fn(user_i2c_port0_handle, s_bus_hooks[i].ctx);
        }
    }
    s_ports[0].consec_timeouts = 0;
    i2c_bsp_unlock_port(0U);
}

void i2c_bsp_i2c1_reinit_bus(void)
{
    if (!i2c_bsp_lock_port(1U, 500)) {
        return;
    }
    if (beam_imu_dev_handle != NULL && s_beam_imu_bus == 1U) {
        i2c_master_bus_rm_device(beam_imu_dev_handle);
        beam_imu_dev_handle = NULL;
    }
    if (user_i2c_port1_handle != NULL) {
        esp_err_t del_err = i2c_del_master_bus(user_i2c_port1_handle);
        if (del_err != ESP_OK) {
            rover_printf("[I2C1 WARN] i2c_del_master_bus returned err=0x%X; falling back to bus_reset\n",
                         (unsigned int)del_err);
            i2c_master_bus_reset(user_i2c_port1_handle);
        } else {
            user_i2c_port1_handle = NULL;
        }
    }
    if (user_i2c_port1_handle == NULL) {
        vTaskDelay(pdMS_TO_TICKS(10));
        i2c_bsp_i2c1_init();
    }
    if (s_beam_imu_addr != 0 && s_beam_imu_bus == 1U && user_i2c_port1_handle != NULL) {
        (void)i2c_bsp_attach_beam_imu_locked(1U, s_beam_imu_addr);
    }
    s_ports[1].consec_timeouts = 0;
    i2c_bsp_unlock_port(1U);
}

bool i2c_bsp_probe_addr(uint8_t addr_7bit)
{
    if (user_i2c_port0_handle == NULL) return false;
    if (!i2c_bsp_lock_port(0U, 200)) return false;
    esp_err_t err = i2c_master_probe(user_i2c_port0_handle, addr_7bit, 50);
    if (err != ESP_OK) {
        i2c_bus_recover_port_locked(0U, "Probe", NULL, addr_7bit, err);
    }
    i2c_bsp_unlock_port(0U);
    return (err == ESP_OK);
}

bool i2c_bsp_probe_optional_addr_port(uint8_t port, uint8_t addr_7bit)
{
    if (port > 1U) return false;
    i2c_master_bus_handle_t bus_handle = *s_ports[port].bus_ptr;
    if (bus_handle == NULL) return false;
    if (!i2c_bsp_lock_port(port, 200)) return false;
    esp_err_t err = i2c_master_probe(bus_handle, addr_7bit, 25);
    i2c_bsp_unlock_port(port);
    return (err == ESP_OK);
}

bool i2c_bsp_probe_optional_addr(uint8_t addr_7bit)
{
    return i2c_bsp_probe_optional_addr_port(0U, addr_7bit);
}

void i2c_bsp_report_status_port(uint8_t port, i2c_master_dev_handle_t dev_handle, const char *action, int err_code)
{
    if (port > 1U) {
        return;
    }
    if (!i2c_bsp_lock_port(port, 200)) {
        return;
    }
    if (err_code == ESP_OK) {
        s_ports[port].consec_timeouts = 0;
    } else {
        i2c_bus_recover_port_locked(port, action ? action : "Op", dev_handle, -1, (esp_err_t)err_code);
    }
    i2c_bsp_unlock_port(port);
}

void i2c_bsp_report_status(i2c_master_dev_handle_t dev_handle, const char *action, int err_code)
{
    uint8_t port = (dev_handle != NULL && dev_handle == beam_imu_dev_handle) ? s_beam_imu_bus : 0U;
    i2c_bsp_report_status_port(port, dev_handle, action, err_code);
}

uint8_t i2c_write_buff(i2c_master_dev_handle_t dev_handle, int reg, uint8_t *buf, uint8_t len)
{
    if (!i2c_bsp_lock_port(0U, 200)) {
        return 1;
    }
    if (dev_handle == NULL) {
        i2c_bsp_unlock_port(0U);
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
    if (ret == ESP_OK) {
        s_ports[0].consec_timeouts = 0;
    } else {
        i2c_bus_recover_port_locked(0U, "Write", dev_handle, reg, ret);
    }
    i2c_bsp_unlock_port(0U);
    return (ret == ESP_OK) ? 0 : 1;
}

uint8_t i2c_master_write_read_dev(i2c_master_dev_handle_t dev_handle, uint8_t *writeBuf, uint8_t writeLen, uint8_t *readBuf, uint8_t readLen)
{
    if (!i2c_bsp_lock_port(0U, 200)) {
        return 1;
    }
    if (dev_handle == NULL) {
        i2c_bsp_unlock_port(0U);
        return 1;
    }
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, writeBuf, writeLen, readBuf, readLen, i2c_data_pdMS_TICKS);
    if (ret == ESP_OK) {
        s_ports[0].consec_timeouts = 0;
    } else {
        i2c_bus_recover_port_locked(0U, "WriteRead", dev_handle, -1, ret);
    }
    i2c_bsp_unlock_port(0U);
    return (ret == ESP_OK) ? 0 : 1;
}

uint8_t i2c_read_buff(i2c_master_dev_handle_t dev_handle, int reg, uint8_t *buf, uint8_t len)
{
    if (!i2c_bsp_lock_port(0U, 200)) {
        return 1;
    }
    if (dev_handle == NULL) {
        i2c_bsp_unlock_port(0U);
        return 1;
    }
    esp_err_t ret;
    if (reg == -1) {
        ret = i2c_master_receive(dev_handle, buf, len, i2c_data_pdMS_TICKS);
    } else {
        uint8_t addr = (uint8_t)reg;
        ret = i2c_master_transmit_receive(dev_handle, &addr, 1, buf, len, i2c_data_pdMS_TICKS);
    }
    if (ret == ESP_OK) {
        s_ports[0].consec_timeouts = 0;
    } else {
        i2c_bus_recover_port_locked(0U, "Read", dev_handle, reg, ret);
    }
    i2c_bsp_unlock_port(0U);
    return (ret == ESP_OK) ? 0 : 1;
}
