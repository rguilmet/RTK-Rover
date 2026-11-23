#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl_ui_internal.h"
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

#define LCD_BIT_PER_PIXEL 16
#define BYTES_PER_PIXEL (LV_COLOR_FORMAT_GET_SIZE(LV_COLOR_FORMAT_RGB565))
#define BUFF_SIZE (ROVER_LCD_H_RES * ROVER_LCD_V_RES * BYTES_PER_PIXEL)

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
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static void rover_lvgl_service_fb_capture(const uint16_t *src_fb)
{
    uint8_t *dst = s_fb_capture_dst;
    if (dst != NULL && src_fb != NULL) {
        memcpy(dst, src_fb, BUFF_SIZE);
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

    const int flush_count = (LVGL_SPIRAM_BUFF_LEN / LVGL_DMA_BUFF_LEN);
    const int offgap      = (ROVER_LCD_V_RES / flush_count);
    const int dmalen      = (LVGL_DMA_BUFF_LEN / 2);
    int offsetx1 = 0;
    int offsety1 = 0;
    int offsetx2 = ROVER_LCD_H_RES;
    int offsety2 = offgap;

    uint16_t *map = (uint16_t *)color_p;
    xSemaphoreGive(flush_done_semaphore);
    for (int i = 0; i < flush_count; i++) {
        xSemaphoreTake(flush_done_semaphore, portMAX_DELAY);
        memcpy(trans_buf_1, map, LVGL_DMA_BUFF_LEN);
        esp_lcd_panel_draw_bitmap(panel_handle, offsetx1, offsety1, offsetx2, offsety2, trans_buf_1);
        offsety1 += offgap;
        offsety2 += offgap;
        map += dmalen;
    }
    xSemaphoreTake(flush_done_semaphore, portMAX_DELAY);
    if (area->x1 == 0 && area->y1 == 0 &&
        lv_area_get_width(area) == ROVER_LCD_H_RES &&
        lv_area_get_height(area) == ROVER_LCD_V_RES) {
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
            if (pointX >= ROVER_LCD_H_RES) pointX = ROVER_LCD_H_RES - 1;
            if (pointY >= ROVER_LCD_V_RES) pointY = ROVER_LCD_V_RES - 1;
            last_x = (int16_t)pointX;
            last_y = (int16_t)pointY;
            indevData->point.x = last_x;
            indevData->point.y = last_y;
            indevData->state   = LV_INDEV_STATE_PRESSED;
            s_last_user_activity_ms = now_ms;

            if (current_settings.enable_touch_logging) {
                if (prev_state != LV_INDEV_STATE_PRESSED) {
                    rover_printf("[TOUCH DOWN] (%d, %d) | buf: %02X %02X %02X %02X %02X %02X | evt:%u pts:%u\n",
                                 last_x, last_y, buff[0], buff[1], buff[2], buff[3], buff[4], buff[5],
                                 event_flag, buff[1]);
                    last_log_ms = now_ms;
                } else if (now_ms - last_log_ms >= 250) {
                    rover_printf("[TOUCH HELD] (%d, %d) | buf: %02X %02X %02X %02X | pts:%u\n",
                                 last_x, last_y, buff[0], buff[1], buff[2], buff[3], buff[1]);
                    last_log_ms = now_ms;
                }
            }
            prev_state = LV_INDEV_STATE_PRESSED;
            return;
        } else if (!corrupt_sentinel && buff[1] >= 1 && buff[0] == 0xFF && current_settings.enable_touch_logging) {
            rover_printf("[TOUCH REJECTED: buff[0]==0xFF] pts=%u evt=%u\n", buff[1], event_flag);
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

    if (current_settings.enable_touch_logging && prev_state == LV_INDEV_STATE_PRESSED && err == ESP_OK && !corrupt_sentinel) {
        uint8_t event_flag = (buff[2] >> 6) & 0x03;
        rover_printf("[TOUCH UP]   (%d, %d) | buf: %02X %02X %02X %02X %02X %02X | evt:%u pts:%u\n",
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
    if (!dst_rgb565_be || dst_bytes < (size_t)BUFF_SIZE || s_fb_capture_done_sem == NULL) {
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
    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
    for (;;) {
        if (rover_lvgl_lock(-1)) {
            task_delay_ms = lv_timer_handler();
            rover_lvgl_service_fb_capture(s_last_flushed_fb);
            rover_lvgl_unlock();
        }
        if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
        } else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS) {
            task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

static void rover_lcd_pwm_off_early(void)
{
    gpio_config_t gpio_conf = {};
    gpio_conf.intr_type    = GPIO_INTR_DISABLE;
    gpio_conf.mode         = GPIO_MODE_OUTPUT;
    gpio_conf.pin_bit_mask = ((uint64_t)1 << ROVER_PIN_BK_LIGHT);
    gpio_conf.pull_down_en = GPIO_PULLDOWN_ENABLE;
    gpio_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
    ESP_ERROR_CHECK(gpio_config(&gpio_conf));
    ESP_ERROR_CHECK(gpio_set_level(ROVER_PIN_BK_LIGHT, 0));
}

static void rover_lcd_exio_init(void)
{
    if (io_expander == NULL) {
        esp_io_expander_new_i2c_tca9554(user_i2c_port0_handle, ROVER_I2C_ADDR_TCA9554, &io_expander);
    }
    if (io_expander == NULL) {
        return;
    }

    // Leave EXIO3 (SD_CS / DAT3) untouched here so SD_MMC / SPI setup remains undisturbed
    esp_io_expander_set_dir(io_expander, ROVER_EXIO_TP_INT | ROVER_EXIO_RTC_INT | ROVER_EXIO_AXP_IRQ | ROVER_EXIO_SYS_OUT, IO_EXPANDER_INPUT);
    esp_io_expander_set_dir(io_expander, ROVER_EXIO_LCD_RST | ROVER_EXIO_PA_CTRL | ROVER_EXIO_CAM_PWDN, IO_EXPANDER_OUTPUT);
    esp_io_expander_set_level(io_expander, ROVER_EXIO_CAM_PWDN, 1); // Power down unused camera
    esp_io_expander_set_level(io_expander, ROVER_EXIO_LCD_RST, 1);
}

static void rover_lcd_reset(void)
{
    if (io_expander == NULL) {
        return;
    }
    esp_io_expander_set_level(io_expander, ROVER_EXIO_LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(30));
    esp_io_expander_set_level(io_expander, ROVER_EXIO_LCD_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(150));
    esp_io_expander_set_level(io_expander, ROVER_EXIO_LCD_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(30));
}

void lvgl_port_init(void)
{
    flush_done_semaphore = xSemaphoreCreateBinary();
    assert(flush_done_semaphore);
    s_fb_capture_done_sem = xSemaphoreCreateBinary();
    assert(s_fb_capture_done_sem);

    ESP_LOGI(TAG, "Initialize LCD reset and backlight");
    rover_lcd_pwm_off_early();
    rover_lcd_exio_init();

    ESP_LOGI(TAG, "Initialize QSPI bus");
    spi_bus_config_t buscfg = {};
    buscfg.data0_io_num    = ROVER_PIN_LCD_DATA0;
    buscfg.data1_io_num    = ROVER_PIN_LCD_DATA1;
    buscfg.sclk_io_num     = ROVER_PIN_LCD_PCLK;
    buscfg.data2_io_num    = ROVER_PIN_LCD_DATA2;
    buscfg.data3_io_num    = ROVER_PIN_LCD_DATA3;
    buscfg.max_transfer_sz = LVGL_DMA_BUFF_LEN;
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t panel_io = NULL;
    esp_lcd_panel_handle_t panel = NULL;

    esp_lcd_panel_io_spi_config_t io_config = {};
    io_config.cs_gpio_num         = ROVER_PIN_LCD_CS;
    io_config.dc_gpio_num         = -1;
    io_config.spi_mode            = 3;
    io_config.pclk_hz             = 40 * 1000 * 1000;
    io_config.trans_queue_depth   = 10;
    io_config.on_color_trans_done = rover_notify_lvgl_flush_ready;
    io_config.lcd_cmd_bits        = 32;
    io_config.lcd_param_bits      = 8;
    io_config.flags.quad_mode     = true;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(LCD_HOST, &io_config, &panel_io));

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
    init_backlight_pwm();

    lv_init();

    lv_display_t *disp = lv_display_create(ROVER_LCD_H_RES, ROVER_LCD_V_RES);
    lv_display_set_flush_cb(disp, rover_lvgl_flush_cb);

    uint8_t *buffer_1 = (uint8_t *)heap_caps_malloc(BUFF_SIZE, MALLOC_CAP_SPIRAM);
    uint8_t *buffer_2 = (uint8_t *)heap_caps_malloc(BUFF_SIZE, MALLOC_CAP_SPIRAM);
    assert(buffer_1);
    assert(buffer_2);
    trans_buf_1 = (uint16_t *)heap_caps_malloc(LVGL_DMA_BUFF_LEN, MALLOC_CAP_DMA);
    assert(trans_buf_1);

    lv_display_set_buffers(disp, buffer_1, buffer_2, BUFF_SIZE, LV_DISPLAY_RENDER_MODE_FULL);
    lv_display_set_user_data(disp, panel);

    lv_indev_t *touch_indev = lv_indev_create();
    lv_indev_set_type(touch_indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(touch_indev, TouchInputReadCallback);

    esp_timer_create_args_t lvgl_tick_timer_args = {};
    lvgl_tick_timer_args.callback = &rover_increase_lvgl_tick;
    lvgl_tick_timer_args.name     = "lvgl_tick";
    esp_timer_handle_t lvgl_tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(lvgl_tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    lvgl_mux = xSemaphoreCreateMutex();
    assert(lvgl_mux);

    if (rover_lvgl_lock(-1)) {
        // Route LVGL widget/style/table allocations (>=64B) into Octal PSRAM so internal
        // 8-bit/DMA SRAM remains completely free & unfragmented for the ESP32-S3 Wi-Fi stack
        heap_caps_malloc_extmem_enable(64);
        create_rover_ui();
        heap_caps_malloc_extmem_enable(4096);
        rover_lvgl_unlock();
    }

    xTaskCreatePinnedToCore(rover_lvgl_port_task, "LVGL", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL, 1);
}
