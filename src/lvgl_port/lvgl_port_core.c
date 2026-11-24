#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl_port.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "../axs15231b/esp_lcd_axs15231b.h"
#include "../tca9554/esp_io_expander_tca9554.h"
#include "../i2c_bsp/i2c_bsp.h"

static const char *TAG = "lvgl_port";
static SemaphoreHandle_t lvgl_mux = NULL;
static uint16_t *trans_buf_1 = NULL;
static uint16_t *s_last_flushed_fb = NULL;
static uint8_t * volatile s_fb_capture_dst = NULL;
static SemaphoreHandle_t s_fb_capture_done_sem = NULL;
static SemaphoreHandle_t flush_done_semaphore = NULL;
esp_io_expander_handle_t io_expander = NULL;

static BoardDisplayCfg_t s_disp_cfg = {0};
static size_t s_fb_bytes = 0;

#define LCD_BIT_PER_PIXEL 16
#define BYTES_PER_PIXEL (LV_COLOR_FORMAT_GET_SIZE(LV_COLOR_FORMAT_RGB565))

static const axs15231b_lcd_init_cmd_t lcd_init_cmds[] = {
    {0x11, (uint8_t []){0x00}, 0, 100},
    {0x29, (uint8_t []){0x00}, 0, 100},
};

static bool rover_notify_lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
    (void)panel_io; (void)edata; (void)user_ctx;
    BaseType_t TaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(flush_done_semaphore, &TaskWoken);
    return (TaskWoken == pdTRUE);
}

static void rover_increase_lvgl_tick(void *arg)
{
    (void)arg;
    lv_tick_inc(s_disp_cfg.tick_period_ms);
}

static void rover_lvgl_service_fb_capture(const uint16_t *src_fb)
{
    uint8_t *dst = s_fb_capture_dst;
    if (dst != NULL && src_fb != NULL && s_fb_bytes > 0) {
        memcpy(dst, src_fb, s_fb_bytes);
        s_fb_capture_dst = NULL;
        if (s_fb_capture_done_sem != NULL) {
            xSemaphoreGive(s_fb_capture_done_sem);
        }
    }
}

static void rover_lvgl_flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *color_p)
{
    esp_lcd_panel_handle_t panel_handle = (esp_lcd_panel_handle_t)lv_display_get_user_data(disp);
    lv_draw_sw_rgb565_swap(color_p, lv_area_get_width(area) * lv_area_get_height(area));

    const int flush_count = (s_disp_cfg.dma_buf_bytes > 0)
                                ? (int)(s_disp_cfg.spiram_buf_bytes / s_disp_cfg.dma_buf_bytes)
                                : 1;
    const int offgap      = (flush_count > 0) ? ((int)s_disp_cfg.v_res / flush_count) : (int)s_disp_cfg.v_res;
    const int dmalen      = (int)(s_disp_cfg.dma_buf_bytes / 2U);
    int offsetx1 = 0;
    int offsety1 = 0;
    int offsetx2 = (int)s_disp_cfg.h_res;
    int offsety2 = offgap;

    uint16_t *map = (uint16_t *)color_p;
    xSemaphoreGive(flush_done_semaphore);
    for (int i = 0; i < flush_count; i++) {
        xSemaphoreTake(flush_done_semaphore, portMAX_DELAY);
        memcpy(trans_buf_1, map, s_disp_cfg.dma_buf_bytes);
        esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2, offsety2, trans_buf_1);
        offsety1 += offgap;
        offsety2 += offgap;
        map += dmalen;
    }
    xSemaphoreTake(flush_done_semaphore, portMAX_DELAY);
    if (area->x1 == 0 && area->y1 == 0 &&
        lv_area_get_width(area) == (int32_t)s_disp_cfg.h_res &&
        lv_area_get_height(area) == (int32_t)s_disp_cfg.v_res) {
        s_last_flushed_fb = (uint16_t *)color_p;
        rover_lvgl_service_fb_capture(s_last_flushed_fb);
    }
    lv_disp_flush_ready(disp);
}

static volatile uint32_t s_last_user_activity_ms = 0;

uint32_t rover_get_last_user_activity_ms(void)
{
    return s_last_user_activity_ms;
}

void rover_note_user_activity(void)
{
    s_last_user_activity_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
}

static void TouchInputReadCallback(lv_indev_t *indev, lv_indev_data_t *indevData)
{
    (void)indev;
    static int16_t last_x = 0;
    static int16_t last_y = 0;
    static lv_indev_state_t prev_state = LV_INDEV_STATE_RELEASED;
    static uint32_t last_log_ms = 0;
    static uint8_t  err_streak = 0;
    static uint32_t backoff_until_ms = 0;

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    if (err_streak >= 5 && (int32_t)(now_ms - backoff_until_ms) < 0) {
        indevData->point.x = last_x;
        indevData->point.y = last_y;
        indevData->state   = LV_INDEV_STATE_RELEASED;
        return;
    }

    // Command byte 7 = 0x0E (14 bytes): matches Waveshare ESP32-S3-Touch-LCD-3.5B bsp_touch.c
    uint8_t read_touchpad_cmd[11] = {0xb5, 0xab, 0xa5, 0x5a, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00};
    uint8_t buff[14] = {0};

    bool touch_log_enabled = (s_disp_cfg.touch_log_enabled_cb != NULL) && s_disp_cfg.touch_log_enabled_cb();
    bool corrupt_sentinel = false;
    esp_err_t err = i2c_master_write_read_dev(disp_touch_dev_handle, read_touchpad_cmd, 11, buff, 14);
    if (err == ESP_OK) {
        err_streak = 0;
        // Reject corrupt 0x61 / 0x71 repeated-sentinel I2C glitches in addition to 0xFF invalid frames
        corrupt_sentinel = i2c_bsp_has_sentinel_run(buff, 6U, 2U);
        // buff[0]: gesture (0xFF = invalid), buff[1]: touch points (1..2)
        // buff[2] bits [7:6]: event flag (0x80 = contact/down, 0x40 = release/up)
        uint8_t event_flag = (buff[2] >> 6) & 0x03;
        if (!corrupt_sentinel && buff[0] != 0xFF && buff[1] >= 1 && buff[1] <= 2 && event_flag != 0x01) {
            uint16_t pointX = (((uint16_t)buff[2] & 0x0F) << 8) | (uint16_t)buff[3];
            uint16_t pointY = (((uint16_t)buff[4] & 0x0F) << 8) | (uint16_t)buff[5];
            if (pointX >= s_disp_cfg.h_res) pointX = s_disp_cfg.h_res - 1U;
            if (pointY >= s_disp_cfg.v_res) pointY = s_disp_cfg.v_res - 1U;
            last_x = (int16_t)pointX;
            last_y = (int16_t)pointY;
            indevData->point.x = last_x;
            indevData->point.y = last_y;
            indevData->state   = LV_INDEV_STATE_PRESSED;
            s_last_user_activity_ms = now_ms;

            if (touch_log_enabled && s_disp_cfg.log_printf_cb != NULL) {
                if (prev_state != LV_INDEV_STATE_PRESSED) {
                    s_disp_cfg.log_printf_cb("[TOUCH DOWN] (%d, %d) | buf: %02X %02X %02X %02X %02X %02X | evt:%u pts:%u\n",
                                             last_x, last_y, buff[0], buff[1], buff[2], buff[3], buff[4], buff[5],
                                             event_flag, buff[1]);
                    last_log_ms = now_ms;
                } else if (now_ms - last_log_ms >= 250) {
                    s_disp_cfg.log_printf_cb("[TOUCH HELD] (%d, %d) | buf: %02X %02X %02X %02X | pts:%u\n",
                                             last_x, last_y, buff[0], buff[1], buff[2], buff[3], buff[1]);
                    last_log_ms = now_ms;
                }
            }
            prev_state = LV_INDEV_STATE_PRESSED;
            return;
        } else if (!corrupt_sentinel && buff[1] >= 1 && buff[0] == 0xFF && touch_log_enabled && s_disp_cfg.log_printf_cb != NULL) {
            s_disp_cfg.log_printf_cb("[TOUCH REJECTED: buff[0]==0xFF] pts=%u evt=%u\n", buff[1], event_flag);
        }
    } else {
        if (err_streak < 255) err_streak++;
        if (err_streak >= 5) {
            // Back off 250 ms during I2C0 fault recovery so lvgl_port_task does not thrash lvgl_mux/i2c_bus_mutex
            backoff_until_ms = (uint32_t)(esp_timer_get_time() / 1000ULL) + 250U;
        }
    }

    // Always preserve last_x / last_y on release so LVGL 9 recognizes swipes and clicks
    indevData->point.x = last_x;
    indevData->point.y = last_y;
    indevData->state   = LV_INDEV_STATE_RELEASED;

    if (touch_log_enabled && s_disp_cfg.log_printf_cb != NULL &&
        prev_state == LV_INDEV_STATE_PRESSED && err == ESP_OK && !corrupt_sentinel) {
        uint8_t event_flag = (buff[2] >> 6) & 0x03;
        s_disp_cfg.log_printf_cb("[TOUCH UP]   (%d, %d) | buf: %02X %02X %02X %02X %02X %02X | evt:%u pts:%u\n",
                                 last_x, last_y, buff[0], buff[1], buff[2], buff[3], buff[4], buff[5],
                                 event_flag, buff[1]);
    }
    prev_state = LV_INDEV_STATE_RELEASED;
}

bool rover_lvgl_lock(int timeout_ms)
{
    if (!lvgl_mux) return false;
    const TickType_t timeout_ticks = (timeout_ms == -1) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    return xSemaphoreTake(lvgl_mux, timeout_ticks) == pdTRUE;
}

void rover_lvgl_unlock(void)
{
    if (lvgl_mux) {
        xSemaphoreGive(lvgl_mux);
    }
}

bool rover_lvgl_capture_framebuffer(uint8_t *dst_rgb565_be, size_t dst_bytes)
{
    if (!dst_rgb565_be || s_fb_bytes == 0 || dst_bytes < s_fb_bytes || s_fb_capture_done_sem == NULL) {
        return false;
    }
    // Clear any stale completion token before arming the one-shot copy
    (void)xSemaphoreTake(s_fb_capture_done_sem, 0);
    s_fb_capture_dst = dst_rgb565_be;
    if (xSemaphoreTake(s_fb_capture_done_sem, pdMS_TO_TICKS(2000)) == pdTRUE) {
        return true;
    }
    // Timeout fallback: disarm request cleanly before caller frees dst_rgb565_be
    s_fb_capture_dst = NULL;
    if (xSemaphoreTake(s_fb_capture_done_sem, pdMS_TO_TICKS(100)) == pdTRUE) {
        return true;
    }
    return false;
}

static void rover_lvgl_port_task(void *arg)
{
    (void)arg;
    uint32_t task_delay_ms = s_disp_cfg.task_max_delay_ms;
    for (;;) {
        if (rover_lvgl_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            rover_lvgl_service_fb_capture(s_last_flushed_fb);
            rover_lvgl_unlock();
        }
        if (task_delay_ms > s_disp_cfg.task_max_delay_ms) {
            task_delay_ms = s_disp_cfg.task_max_delay_ms;
        } else if (task_delay_ms < s_disp_cfg.task_min_delay_ms) {
            task_delay_ms = s_disp_cfg.task_min_delay_ms;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

static void rover_lcd_pwm_off_early(void)
{
    if (s_disp_cfg.pin_bk_light < 0) {
        return;
    }
    gpio_config_t gpio_conf = {};
    gpio_conf.intr_type    = GPIO_INTR_DISABLE;
    gpio_conf.mode         = GPIO_MODE_OUTPUT;
    gpio_conf.pin_bit_mask = ((uint64_t)1 << s_disp_cfg.pin_bk_light);
    gpio_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&gpio_conf));
    ESP_ERROR_CHECK(gpio_set_level((gpio_num_t)s_disp_cfg.pin_bk_light, 0));
}

static void rover_lcd_exio_init(void)
{
    if (io_expander == NULL && s_disp_cfg.tca9554_addr != 0) {
        esp_io_expander_new_i2c_tca9554(user_i2c_port0_handle, s_disp_cfg.tca9554_addr, &io_expander);
    }
    if (io_expander == NULL) {
        return;
    }

    // Leave EXIO3 (SD_CS / DAT3) untouched here so SD_MMC / SPI setup remains undisturbed
    esp_io_expander_set_dir(io_expander, s_disp_cfg.exio_input_mask, IO_EXPANDER_INPUT);
    esp_io_expander_set_dir(io_expander, s_disp_cfg.exio_output_mask, IO_EXPANDER_OUTPUT);
    if (s_disp_cfg.exio_cam_pwdn_mask != 0) {
        esp_io_expander_set_level(io_expander, s_disp_cfg.exio_cam_pwdn_mask, 1); // Power down unused camera
    }
    if (s_disp_cfg.exio_lcd_rst_mask != 0) {
        esp_io_expander_set_level(io_expander, s_disp_cfg.exio_lcd_rst_mask, 1);
    }
}

static void rover_lcd_reset(void)
{
    if (io_expander == NULL || s_disp_cfg.exio_lcd_rst_mask == 0) {
        return;
    }
    esp_io_expander_set_level(io_expander, s_disp_cfg.exio_lcd_rst_mask, 1);
    vTaskDelay(pdMS_TO_TICKS(30));
    esp_io_expander_set_level(io_expander, s_disp_cfg.exio_lcd_rst_mask, 0);
    vTaskDelay(pdMS_TO_TICKS(150));
    esp_io_expander_set_level(io_expander, s_disp_cfg.exio_lcd_rst_mask, 1);
    vTaskDelay(pdMS_TO_TICKS(30));
}

void lvgl_port_init_with_cfg(const BoardDisplayCfg_t *cfg)
{
    if (cfg == NULL || cfg->h_res == 0 || cfg->v_res == 0 || cfg->dma_buf_bytes == 0) {
        ESP_LOGE(TAG, "Invalid BoardDisplayCfg_t passed to lvgl_port_init_with_cfg");
        return;
    }

    s_disp_cfg = *cfg;
    s_fb_bytes = (size_t)s_disp_cfg.h_res * (size_t)s_disp_cfg.v_res * BYTES_PER_PIXEL;
    if (s_disp_cfg.spiram_buf_bytes == 0) {
        s_disp_cfg.spiram_buf_bytes = s_fb_bytes;
    }

    flush_done_semaphore = xSemaphoreCreateBinary();
    assert(flush_done_semaphore);
    s_fb_capture_done_sem = xSemaphoreCreateBinary();
    assert(s_fb_capture_done_sem);

    ESP_LOGI(TAG, "Initialize LCD reset and backlight");
    rover_lcd_pwm_off_early();
    rover_lcd_exio_init();

    ESP_LOGI(TAG, "Initialize QSPI bus");
    spi_bus_config_t buscfg = {};
    buscfg.data0_io_num    = s_disp_cfg.pin_data0;
    buscfg.data1_io_num    = s_disp_cfg.pin_data1;
    buscfg.sclk_io_num     = s_disp_cfg.pin_pclk;
    buscfg.data2_io_num    = s_disp_cfg.pin_data2;
    buscfg.data3_io_num    = s_disp_cfg.pin_data3;
    buscfg.max_transfer_sz = (int)s_disp_cfg.dma_buf_bytes;
    ESP_ERROR_CHECK(spi_bus_initialize((spi_host_device_t)s_disp_cfg.spi_host, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t panel_io = NULL;
    esp_lcd_panel_handle_t panel = NULL;

    esp_lcd_panel_io_spi_config_t io_config = {};
    io_config.cs_gpio_num         = s_disp_cfg.pin_cs;
    io_config.dc_gpio_num         = -1;
    io_config.spi_mode            = 3;
    io_config.pclk_hz             = s_disp_cfg.pclk_hz;
    io_config.trans_queue_depth   = 10;
    io_config.on_color_trans_done = rover_notify_lvgl_flush_ready;
    io_config.lcd_cmd_bits        = 32;
    io_config.lcd_param_bits      = 8;
    io_config.flags.quad_mode     = true;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)s_disp_cfg.spi_host, &io_config, &panel_io));

    axs15231b_vendor_config_t vendor_config = {};
    vendor_config.flags.use_qspi_interface = 1;
    vendor_config.init_cmds      = lcd_init_cmds;
    vendor_config.init_cmds_size = sizeof(lcd_init_cmds) / sizeof(lcd_init_cmds[0]);

    esp_lcd_panel_dev_config_t panel_config = {};
    panel_config.reset_gpio_num = -1;
    panel_config.rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB;
    panel_config.bits_per_pixel = LCD_BIT_PER_PIXEL;
    panel_config.vendor_config  = &vendor_config;

    ESP_ERROR_CHECK(esp_lcd_new_panel_axs15231b(panel_io, &panel_config, &panel));

    rover_lcd_reset();
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    if (s_disp_cfg.init_backlight_cb != NULL) {
        s_disp_cfg.init_backlight_cb();
    }

    lv_init();

    lv_display_t *disp = lv_display_create(s_disp_cfg.h_res, s_disp_cfg.v_res);
    lv_display_set_flush_cb(disp, rover_lvgl_flush_cb);

    uint8_t *buffer_1 = (uint8_t *)heap_caps_malloc(s_fb_bytes, MALLOC_CAP_SPIRAM);
    uint8_t *buffer_2 = (uint8_t *)heap_caps_malloc(s_fb_bytes, MALLOC_CAP_SPIRAM);
    assert(buffer_1);
    assert(buffer_2);
    trans_buf_1 = (uint16_t *)heap_caps_malloc(s_disp_cfg.dma_buf_bytes, MALLOC_CAP_DMA);
    assert(trans_buf_1);

    lv_display_set_buffers(disp, buffer_1, buffer_2, s_fb_bytes, LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_user_data(disp, panel);

    lv_indev_t *touch_indev = lv_indev_create();
    lv_indev_set_type(touch_indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(touch_indev, TouchInputReadCallback);

    esp_timer_create_args_t lvgl_tick_timer_args = {};
    lvgl_tick_timer_args.callback = &rover_increase_lvgl_tick;
    lvgl_tick_timer_args.name     = "lvgl_tick";
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, s_disp_cfg.tick_period_ms * 1000U));

    lvgl_mux = xSemaphoreCreateMutex();
    assert(lvgl_mux);

    if (rover_lvgl_lock(-1)) {
        // Route LVGL widget/style/table allocations (>=64B) into Octal PSRAM so internal
        // 8-bit/DMA SRAM remains completely free & unfragmented for the ESP32-S3 Wi-Fi stack
        heap_caps_malloc_extmem_enable(64);
        if (s_disp_cfg.create_ui_cb != NULL) {
            s_disp_cfg.create_ui_cb();
        }
        heap_caps_malloc_extmem_enable(4096);
        rover_lvgl_unlock();
    }

    xTaskCreatePinnedToCore(rover_lvgl_port_task, "LVGL", s_disp_cfg.task_stack_size, NULL, s_disp_cfg.task_priority, NULL, 1);
}
