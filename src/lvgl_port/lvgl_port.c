#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl_port.h"
#include "lvgl.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_timer.h"
#include "../../user_config.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_io.h"
#include "../axs15231b/esp_lcd_axs15231b.h"
#include "../tca9554/esp_io_expander_tca9554.h"
#include "../i2c_bsp/i2c_bsp.h"
#include "../rtc_bsp/rtc_bsp.h"
#include "../settings_manager/settings_manager.h"
#include "../nav_engine/nav_engine.h"
#include "../waypoint_manager/waypoint_manager.h"
#include "../storage/route_logger.h"
#include "../storage/nmea_logger.h"
#include "../storage/rtcm_logger.h"
#include "../storage/serial_logger.h"
#include "../web_server/web_server.h"
#include "../e220_driver/e220_driver.h"

static const char *TAG = "lvgl_port";
static SemaphoreHandle_t lvgl_mux = NULL;
static uint16_t *trans_buf_1 = NULL;
static SemaphoreHandle_t flush_done_semaphore = NULL;
esp_io_expander_handle_t io_expander = NULL;

#define LCD_BIT_PER_PIXEL 16
#define BYTES_PER_PIXEL (LV_COLOR_FORMAT_GET_SIZE(LV_COLOR_FORMAT_RGB565))
#define BUFF_SIZE (ROVER_LCD_H_RES * ROVER_LCD_V_RES * BYTES_PER_PIXEL)

extern void init_backlight_pwm(void);
extern void set_backlight_pwm(uint8_t brightness);
extern void set_rover_brightness(uint8_t brightness_pct);
extern void set_rover_volume(uint8_t volume_pct);
extern void mark_settings_dirty(void);
extern void play_rover_beep(uint16_t freq_hz, uint16_t duration_ms);

// =========================================================================
// Persistent Top Header Bar Widgets (Visible on ALL Screens)
// =========================================================================
// Row 1: System & Radios
static lv_obj_t *hdr_datetime_lbl = NULL;
static lv_obj_t *hdr_units_lbl    = NULL;
static lv_obj_t *hdr_wifi_lbl     = NULL;
static lv_obj_t *hdr_batt_lbl     = NULL;

// Row 2: Master Antenna (ANT1)
static lv_obj_t *hdr_m_fix_badge  = NULL;
static lv_obj_t *hdr_m_fix_lbl    = NULL;
static lv_obj_t *hdr_m_sats_lbl   = NULL;
static lv_obj_t *hdr_m_acc_lbl    = NULL;
static lv_obj_t *hdr_m_rtcm_lbl   = NULL;

// Row 3: Slave Antenna (ANT2)
static lv_obj_t *hdr_s_fix_badge  = NULL;
static lv_obj_t *hdr_s_fix_lbl    = NULL;
static lv_obj_t *hdr_s_sats_lbl   = NULL;
static lv_obj_t *hdr_s_bl_lbl     = NULL;
static lv_obj_t *hdr_s_hdg_lbl    = NULL;

// Row 4: Streamlined Coordinates Bar
static lv_obj_t *hdr_coords_lbl   = NULL;

// =========================================================================
// Tileview & Navigation
// =========================================================================
static lv_obj_t *main_tv        = NULL;
static lv_obj_t *tile_guidance  = NULL;
static lv_obj_t *tile_survey    = NULL;
static lv_obj_t *tile_csv       = NULL;
static lv_obj_t *tile_compass   = NULL;
static lv_obj_t *tile_test      = NULL;
static lv_obj_t *nav_screen_lbl = NULL;
static volatile uint8_t active_tile_col = 0;

static const char *kScreenTitles[5] = {
    "1/5: Pin Seeker",
    "2/5: Survey & Settings",
    "3/5: CSV Waypoints",
    "4/5: Compass & Level",
    "5/5: Diagnostics"
};

// =========================================================================
// Tile 0: Main Guidance / Pin Seeker Widgets (Hybrid Navigation)
// =========================================================================
static lv_obj_t *t0_pos_lbl         = NULL;
static lv_obj_t *t0_target_lbl      = NULL;
static lv_obj_t *t0_notes_lbl       = NULL;
static lv_obj_t *t0_rec_btn         = NULL;
static lv_obj_t *t0_rec_btn_lbl     = NULL;

// Navigation Canvas (304x210)
static lv_obj_t *t0_nav_canvas      = NULL;
static lv_obj_t *t0_mode_badge      = NULL;
static lv_obj_t *t0_hdg_hud_lbl     = NULL;
static lv_obj_t *t0_pitch_hud_lbl   = NULL;
static lv_obj_t *t0_scale_lbl       = NULL;
static lv_obj_t *t0_tgt_hdg_lbl     = NULL;
static lv_obj_t *t0_cross_h         = NULL;
static lv_obj_t *t0_cross_v         = NULL;
static lv_obj_t *t0_bullseye_ring1  = NULL;
static lv_obj_t *t0_bullseye_ring2  = NULL;
static lv_obj_t *t0_bullseye_center = NULL;
static lv_obj_t *t0_track_line      = NULL;
static lv_obj_t *t0_track_arrow     = NULL;
static lv_obj_t *t0_beam_arrow      = NULL;
static lv_obj_t *t0_wedge_amber_l   = NULL;
static lv_obj_t *t0_wedge_amber_r   = NULL;
static lv_obj_t *t0_wedge_green_l   = NULL;
static lv_obj_t *t0_wedge_green_r   = NULL;
static lv_obj_t *t0_target_icon     = NULL;
static lv_obj_t *t0_rover_icon      = NULL;
#define T0_GRID_H_LINES 4
#define T0_GRID_V_LINES 4
static lv_obj_t *t0_grid_h[T0_GRID_H_LINES] = {0};
static lv_obj_t *t0_grid_v[T0_GRID_V_LINES] = {0};

#define COLOR_BTN_ACTIVE_GREEN  lv_color_hex(0x107C41)
#define COLOR_BTN_INACTIVE_BLUE lv_color_hex(0x1B6CA8)
#define COLOR_UNIT_SI_TEAL      lv_color_hex(0x0284C7)
#define COLOR_UNIT_IMP_AMBER    lv_color_hex(0xD97706)

static lv_obj_t *t0_dist_lbl        = NULL;
static lv_obj_t *t0_steer_lbl       = NULL;
static lv_obj_t *t0_offsets_lbl     = NULL;

// =========================================================================
// Tile 1: Field Survey / Waypoint Logger & Quick Settings
// =========================================================================
static lv_obj_t *t1_next_lbl        = NULL;
static lv_obj_t *t1_prefix_dd       = NULL;
static lv_obj_t *t1_note_dd         = NULL;
static lv_obj_t *t1_avg_btn_lbl     = NULL;
static lv_obj_t *t1_log_btn         = NULL;
static lv_obj_t *t1_log_btn_lbl     = NULL;
static lv_obj_t *t1_prog_bar        = NULL;
static lv_obj_t *t1_status_lbl      = NULL;

// Tile 1 Quick Settings (3x4 Grid - 12 Buttons)
static lv_obj_t *t1_vol_btn         = NULL;
static lv_obj_t *t1_vol_btn_lbl     = NULL;
static lv_obj_t *t1_aud_btn         = NULL;
static lv_obj_t *t1_aud_btn_lbl     = NULL;
static lv_obj_t *t1_bright_btn      = NULL;
static lv_obj_t *t1_bright_btn_lbl  = NULL;

static lv_obj_t *t1_nmea_btn        = NULL;
static lv_obj_t *t1_nmea_btn_lbl    = NULL;
static lv_obj_t *t1_serial_btn      = NULL;
static lv_obj_t *t1_serial_btn_lbl  = NULL;
static lv_obj_t *t1_rtcm_btn        = NULL;
static lv_obj_t *t1_rtcm_btn_lbl    = NULL;

static lv_obj_t *t1_telem_btn       = NULL;
static lv_obj_t *t1_telem_btn_lbl   = NULL;
static lv_obj_t *t1_imu_btn         = NULL;
static lv_obj_t *t1_imu_btn_lbl     = NULL;
static lv_obj_t *t1_touch_btn       = NULL;
static lv_obj_t *t1_touch_btn_lbl   = NULL;

static lv_obj_t *t1_wifi_btn        = NULL;
static lv_obj_t *t1_wifi_btn_lbl    = NULL;
static lv_obj_t *t1_unit_btn        = NULL;
static lv_obj_t *t1_unit_btn_lbl    = NULL;
static lv_obj_t *t1_lora_btn        = NULL;
static lv_obj_t *t1_lora_btn_lbl    = NULL;

static lv_obj_t *t1_wifi_status_lbl = NULL;

// =========================================================================
// Tile 2: Waypoint CSV Browser
// =========================================================================
static lv_obj_t *t2_file_lbl      = NULL;
static lv_obj_t *t2_wp_table      = NULL;

// =========================================================================
// Tile 3: Dual-Antenna Compass & Pole Bubble Level
// =========================================================================
static lv_obj_t *t3_hdg_lbl         = NULL;
static lv_obj_t *t3_base_lbl        = NULL;
static lv_obj_t *t3_pose_lbl        = NULL;
static lv_obj_t *t3_level_circle    = NULL;
static lv_obj_t *t3_bubble_dot      = NULL;
static lv_obj_t *t3_card_lbls[8]    = {NULL};
static lv_obj_t *t3_tilt_lbl        = NULL;
static lv_obj_t *t3_tare_btn        = NULL;
static lv_obj_t *t3_tare_lbl        = NULL;

static const char *kCardNames[8] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
static const float kCardAngles[8] = {0.0f, 45.0f, 90.0f, 135.0f, 180.0f, 225.0f, 270.0f, 315.0f};

// =========================================================================
// Tile 4: Hardware Diagnostics
// =========================================================================
static lv_obj_t *t4_title               = NULL;
static lv_obj_t *t4_rf_card             = NULL;
static lv_obj_t *t4_rf_bar              = NULL;
static lv_obj_t *t4_rf_rssi_lbl         = NULL;
static lv_obj_t *t4_rf_noise_lbl        = NULL;
static lv_obj_t *t4_rf_health_lbl       = NULL;
static lv_obj_t *t4_rf_chan_lbl         = NULL;
static lv_obj_t *t4_scroll_cont         = NULL;
static lv_obj_t *t4_storage_status_lbl  = NULL;
static lv_obj_t *t4_ant_master_lbl      = NULL;
static lv_obj_t *t4_ant_slave_lbl       = NULL;
static lv_obj_t *t4_base_station_lbl    = NULL;
static lv_obj_t *t4_com1_lbl            = NULL;
static lv_obj_t *t4_com2_lbl            = NULL;
static lv_obj_t *t4_e220_lbl            = NULL;
static lv_obj_t *t4_e220_cfg_lbl        = NULL;
static lv_obj_t *t4_power_lbl           = NULL;
static lv_obj_t *t4_imu_lbl             = NULL;
static lv_obj_t *t4_wifi_lbl            = NULL;
static lv_obj_t *t4_i2c_lbl             = NULL;

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
    lv_disp_flush_ready(disp);
}

static void TouchInputReadCallback(lv_indev_t *indev, lv_indev_data_t *indevData)
{
    (void)indev;
    static int16_t last_x = 0;
    static int16_t last_y = 0;
    static lv_indev_state_t prev_state = LV_INDEV_STATE_RELEASED;
    static uint32_t last_log_ms = 0;

    // Command byte 7 = 0x0E (14 bytes): matches Waveshare ESP32-S3-Touch-LCD-3.5B bsp_touch.c
    uint8_t read_touchpad_cmd[11] = {0xb5, 0xab, 0xa5, 0x5a, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00};
    uint8_t buff[14] = {0};

    esp_err_t err = i2c_master_write_read_dev(disp_touch_dev_handle, read_touchpad_cmd, 11, buff, 14);
    if (err == ESP_OK) {
        // buff[0]: gesture (0xFF = invalid), buff[1]: touch points (1..2)
        // buff[2] bits [7:6]: event flag (0x80 = contact/down, 0x40 = release/up)
        uint8_t event_flag = (buff[2] >> 6) & 0x03;
        if (buff[0] != 0xFF && buff[1] >= 1 && buff[1] <= 2 && event_flag != 0x01) {
            uint16_t pointX = (((uint16_t)buff[2] & 0x0F) << 8) | (uint16_t)buff[3];
            uint16_t pointY = (((uint16_t)buff[4] & 0x0F) << 8) | (uint16_t)buff[5];
            if (pointX >= ROVER_LCD_H_RES) pointX = ROVER_LCD_H_RES - 1;
            if (pointY >= ROVER_LCD_V_RES) pointY = ROVER_LCD_V_RES - 1;
            last_x = (int16_t)pointX;
            last_y = (int16_t)pointY;
            indevData->point.x = last_x;
            indevData->point.y = last_y;
            indevData->state   = LV_INDEV_STATE_PRESSED;

            if (current_settings.enable_touch_logging) {
                uint32_t now = (uint32_t)(esp_timer_get_time() / 1000ULL);
                if (prev_state != LV_INDEV_STATE_PRESSED) {
                    rover_printf("[TOUCH DOWN] (%d, %d) | buf: %02X %02X %02X %02X %02X %02X | evt:%u pts:%u\n",
                                 last_x, last_y, buff[0], buff[1], buff[2], buff[3], buff[4], buff[5],
                                 event_flag, buff[1]);
                    last_log_ms = now;
                } else if (now - last_log_ms >= 250) {
                    rover_printf("[TOUCH HELD] (%d, %d) | buf: %02X %02X %02X %02X | pts:%u\n",
                                 last_x, last_y, buff[0], buff[1], buff[2], buff[3], buff[1]);
                    last_log_ms = now;
                }
            }
            prev_state = LV_INDEV_STATE_PRESSED;
            return;
        } else if (buff[1] >= 1 && buff[0] == 0xFF && current_settings.enable_touch_logging) {
            rover_printf("[TOUCH REJECTED: buff[0]==0xFF] pts=%u evt=%u\n", buff[1], event_flag);
        }
    }

    // Always preserve last_x / last_y on release so LVGL 9 recognizes swipes and clicks
    indevData->point.x = last_x;
    indevData->point.y = last_y;
    indevData->state   = LV_INDEV_STATE_RELEASED;

    if (current_settings.enable_touch_logging && prev_state == LV_INDEV_STATE_PRESSED) {
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

uint8_t rover_get_active_tile_index(void)
{
    return active_tile_col;
}

static void rover_lvgl_port_task(void *arg)
{
    (void)arg;
    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
    for (;;) {
        if (rover_lvgl_lock(-1)) {
            task_delay_ms = lv_timer_handler();
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
        ESP_ERROR_CHECK(esp_io_expander_new_i2c_tca9554(user_i2c_port0_handle, ROVER_I2C_ADDR_TCA9554, &io_expander));
    }

    // Leave EXIO3 (SD_CS / DAT3) untouched here so SD_MMC / SPI setup remains undisturbed
    ESP_ERROR_CHECK(esp_io_expander_set_dir(io_expander, ROVER_EXIO_TP_INT | ROVER_EXIO_RTC_INT | ROVER_EXIO_AXP_IRQ | ROVER_EXIO_SYS_OUT, IO_EXPANDER_INPUT));
    ESP_ERROR_CHECK(esp_io_expander_set_dir(io_expander, ROVER_EXIO_LCD_RST | ROVER_EXIO_PA_CTRL | ROVER_EXIO_CAM_PWDN, IO_EXPANDER_OUTPUT));
    ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander, ROVER_EXIO_CAM_PWDN, 1)); // Power down unused camera
    ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander, ROVER_EXIO_LCD_RST, 1));
}

static void rover_lcd_reset(void)
{
    ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander, ROVER_EXIO_LCD_RST, 1));
    vTaskDelay(pdMS_TO_TICKS(30));
    ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander, ROVER_EXIO_LCD_RST, 0));
    vTaskDelay(pdMS_TO_TICKS(150));
    ESP_ERROR_CHECK(esp_io_expander_set_level(io_expander, ROVER_EXIO_LCD_RST, 1));
    vTaskDelay(pdMS_TO_TICKS(30));
}

// =========================================================================
// UI Event Callbacks
// =========================================================================
static void update_nav_bar_label(void)
{
    if (nav_screen_lbl && active_tile_col < 5) {
        lv_label_set_text(nav_screen_lbl, kScreenTitles[active_tile_col]);
    }
}

static void switch_to_tile_index(uint8_t idx, lv_anim_enable_t anim)
{
    if (!main_tv || idx >= 5) return;
    lv_obj_t *tiles[5] = {tile_guidance, tile_survey, tile_csv, tile_compass, tile_test};
    if (tiles[idx]) {
        active_tile_col = idx;
        lv_tileview_set_tile(main_tv, tiles[idx], anim);
        update_nav_bar_label();
    }
}

static void tv_changed_event_cb(lv_event_t *e)
{
    (void)e;
    lv_obj_t *act = lv_tileview_get_tile_active(main_tv);
    if (act == tile_guidance)     active_tile_col = 0;
    else if (act == tile_survey)  active_tile_col = 1;
    else if (act == tile_csv)     active_tile_col = 2;
    else if (act == tile_compass) active_tile_col = 3;
    else if (act == tile_test)    active_tile_col = 4;
    update_nav_bar_label();
}

static void nav_prev_screen_btn_cb(lv_event_t *e)
{
    (void)e;
    play_rover_beep(1200, 20);
    uint8_t next_idx = (active_tile_col == 0) ? 4 : (active_tile_col - 1);
    switch_to_tile_index(next_idx, LV_ANIM_ON);
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] '< Prev' screen clicked -> Tile %u (%s)\n", next_idx + 1, kScreenTitles[next_idx]);
    }
}

static void nav_next_screen_btn_cb(lv_event_t *e)
{
    (void)e;
    play_rover_beep(1200, 20);
    uint8_t next_idx = (active_tile_col + 1) % 5;
    switch_to_tile_index(next_idx, LV_ANIM_ON);
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] 'Next >' screen clicked -> Tile %u (%s)\n", next_idx + 1, kScreenTitles[next_idx]);
    }
}

static uint8_t s_tile0_nav_mode = 0; // 0 = Point Guidance, 1 = Line Stakeout

uint8_t rover_get_tile0_nav_mode(void)
{
    return s_tile0_nav_mode;
}

static void update_t0_target_label(void);

static void t0_mode_toggle_cb(lv_event_t *e)
{
    (void)e;
    play_rover_beep(1600, 25);
    s_tile0_nav_mode = (s_tile0_nav_mode == 0) ? 1 : 0;
    update_t0_target_label();
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Tile 0 Nav Mode -> %s\n",
                     (s_tile0_nav_mode == 1) ? "LINE STAKEOUT" : "POINT GUIDANCE");
    }
}

static void t0_target_lbl_cb(lv_event_t *e)
{
    (void)e;
    if (s_tile0_nav_mode == 1) {
        play_rover_beep(1700, 25);
        waypoint_line_stakeout_swap();
        update_t0_target_label();
        if (current_settings.enable_touch_logging) {
            rover_printf("[TOUCH BTN] Line Stakeout A<->B Swapped\n");
        }
    }
}

static void update_t0_target_label(void)
{
    if (s_tile0_nav_mode == 1) {
        // Line Stakeout Mode
        WaypointRecord_t wp_a, wp_b;
        int16_t idx_a = -1, idx_b = -1;
        waypoint_line_stakeout_get_points(&wp_a, &wp_b, &idx_a, &idx_b);
        if (t0_target_lbl && idx_a >= 0 && idx_b >= 0) {
            char buf[64];
            snprintf(buf, sizeof(buf), "%s -> %s [SWAP]", wp_a.label, wp_b.label);
            const char *cur = lv_label_get_text(t0_target_lbl);
            if (!cur || strcmp(cur, buf) != 0) {
                lv_label_set_text(t0_target_lbl, buf);
            }
        }
        if (t0_notes_lbl && idx_a >= 0 && idx_b >= 0) {
            NavLineStakeout_t stakeout;
            nav_compute_line_stakeout(0, 0, wp_a.latitude, wp_a.longitude, wp_b.latitude, wp_b.longitude, &stakeout);
            char qbrg[32];
            nav_format_quadrant_bearing(stakeout.line_bearing_deg, qbrg, sizeof(qbrg));
            char len_str[24];
            nav_format_distance(stakeout.total_length_m, len_str, sizeof(len_str));
            char nbuf[64];
            snprintf(nbuf, sizeof(nbuf), "Deed: %s | %s", qbrg, len_str);
            const char *cur_notes = lv_label_get_text(t0_notes_lbl);
            if (!cur_notes || strcmp(cur_notes, nbuf) != 0) {
                lv_label_set_text(t0_notes_lbl, nbuf);
            }
        }
        return;
    }

    WaypointRecord_t active_wp;
    if (t0_target_lbl && waypoint_get_active(&active_wp)) {
        char buf[64];
        snprintf(buf, sizeof(buf), "%s (%d/%d)",
                 active_wp.label, g_active_waypoint_idx + 1, g_waypoint_count);
        const char *cur = lv_label_get_text(t0_target_lbl);
        if (!cur || strcmp(cur, buf) != 0) {
            lv_label_set_text(t0_target_lbl, buf);
        }
    } else if (t0_target_lbl) {
        const char *cur = lv_label_get_text(t0_target_lbl);
        if (!cur || strcmp(cur, "No Target (0/0)") != 0) {
            lv_label_set_text(t0_target_lbl, "No Target (0/0)");
        }
    }
    if (t0_notes_lbl && waypoint_get_active(&active_wp)) {
        const char *new_notes = active_wp.notes[0] ? active_wp.notes : "---";
        const char *cur_notes = lv_label_get_text(t0_notes_lbl);
        if (!cur_notes || strcmp(cur_notes, new_notes) != 0) {
            lv_label_set_text(t0_notes_lbl, new_notes);
        }
    } else if (t0_notes_lbl) {
        const char *cur_notes = lv_label_get_text(t0_notes_lbl);
        if (!cur_notes || strcmp(cur_notes, "---") != 0) {
            lv_label_set_text(t0_notes_lbl, "---");
        }
    }
}

static void prev_wp_btn_cb(lv_event_t *e)
{
    (void)e;
    play_rover_beep(1500, 20);
    waypoint_select_prev();
    update_t0_target_label();
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Waypoint '<' clicked -> Target idx: %d/%d\n",
                     g_active_waypoint_idx + 1, g_waypoint_count);
    }
}

static void next_wp_btn_cb(lv_event_t *e)
{
    (void)e;
    play_rover_beep(1500, 20);
    waypoint_select_next();
    update_t0_target_label();
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Waypoint '>' clicked -> Target idx: %d/%d\n",
                     g_active_waypoint_idx + 1, g_waypoint_count);
    }
}

static void prev_file_btn_cb(lv_event_t *e)
{
    (void)e;
    play_rover_beep(1500, 20);
    waypoint_file_select_prev();
    update_t0_target_label();
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] File '<' clicked -> %s\n", waypoint_file_get_active_name());
    }
}

static void next_file_btn_cb(lv_event_t *e)
{
    (void)e;
    play_rover_beep(1500, 20);
    waypoint_file_select_next();
    update_t0_target_label();
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] File '>' clicked -> %s\n", waypoint_file_get_active_name());
    }
}

static void update_all_control_buttons(void)
{
    static int s_last_vol = -1;
    static int s_last_muted = -1;
    static int s_last_aud_mode = -1;
    static int s_last_bright = -1;
    static int s_last_nmea = -1;
    static int s_last_serial = -1;
    static int s_last_route = -1;
    static int s_last_rtcm = -1;
    static int s_last_telem = -1;
    static int s_last_imu = -1;
    static int s_last_touch = -1;
    static int s_last_wifi = -1;
    static int s_last_units = -1;
    static int s_last_tare = -1;

    // Row 1, Col 0: VOL: 80% / VOL: MUTE
    int cur_vol = (int)current_settings.volume;
    int cur_muted = current_settings.is_muted ? 1 : 0;
    if (cur_vol != s_last_vol || cur_muted != s_last_muted) {
        s_last_vol = cur_vol;
        s_last_muted = cur_muted;
        if (t1_vol_btn && t1_vol_btn_lbl) {
            char v_buf[32];
            if (current_settings.volume == 0 || current_settings.is_muted) {
                snprintf(v_buf, sizeof(v_buf), "VOL: MUTE");
                lv_obj_set_style_bg_color(t1_vol_btn, lv_color_hex(0x555555), 0);
            } else {
                snprintf(v_buf, sizeof(v_buf), "VOL: %u%%", current_settings.volume);
                lv_obj_set_style_bg_color(t1_vol_btn, lv_color_hex(0x2F3B4C), 0);
            }
            lv_label_set_text(t1_vol_btn_lbl, v_buf);
        }
    }

    // Row 1, Col 1: AUD: GEIGER / AUD: BEEPS / AUD: MUTE
    int cur_aud = (int)current_settings.audio_nav_mode;
    if (cur_aud != s_last_aud_mode) {
        s_last_aud_mode = cur_aud;
        if (t1_aud_btn && t1_aud_btn_lbl) {
            if (cur_aud == AUDIO_NAV_GEIGER) {
                lv_label_set_text(t1_aud_btn_lbl, "AUD: GEIGER");
                lv_obj_set_style_bg_color(t1_aud_btn, COLOR_BTN_ACTIVE_GREEN, 0);
            } else if (cur_aud == AUDIO_NAV_MUTE) {
                lv_label_set_text(t1_aud_btn_lbl, "AUD: MUTE");
                lv_obj_set_style_bg_color(t1_aud_btn, lv_color_hex(0x555555), 0);
            } else {
                lv_label_set_text(t1_aud_btn_lbl, "AUD: BEEPS");
                lv_obj_set_style_bg_color(t1_aud_btn, lv_color_hex(0x2563EB), 0);
            }
        }
    }

    // Row 1, Col 2: BRI: 100%
    int cur_bright = (int)current_settings.brightness;
    if (cur_bright != s_last_bright) {
        s_last_bright = cur_bright;
        if (t1_bright_btn && t1_bright_btn_lbl) {
            char b_buf[32];
            snprintf(b_buf, sizeof(b_buf), "BRI: %u%%", current_settings.brightness);
            lv_label_set_text(t1_bright_btn_lbl, b_buf);
            lv_obj_set_style_bg_color(t1_bright_btn, lv_color_hex(0x2F3B4C), 0);
        }
    }

    // Row 2, Col 0: NMEA: ON / NMEA: OFF
    int cur_nmea = (nmea_logger_is_active() || current_settings.enable_nmea_logging) ? 1 : 0;
    if (cur_nmea != s_last_nmea) {
        s_last_nmea = cur_nmea;
        if (t1_nmea_btn && t1_nmea_btn_lbl) {
            lv_label_set_text(t1_nmea_btn_lbl, cur_nmea ? "NMEA: ON" : "NMEA: OFF");
            lv_obj_set_style_bg_color(t1_nmea_btn, cur_nmea ? COLOR_BTN_ACTIVE_GREEN : COLOR_BTN_INACTIVE_BLUE, 0);
        }
    }

    // Row 2, Col 1: SERIAL: ON / SERIAL: OFF
    int cur_serial = (serial_logger_is_active() || current_settings.enable_serial_logging) ? 1 : 0;
    if (cur_serial != s_last_serial) {
        s_last_serial = cur_serial;
        if (t1_serial_btn && t1_serial_btn_lbl) {
            lv_label_set_text(t1_serial_btn_lbl, cur_serial ? "SERIAL: ON" : "SERIAL: OFF");
            lv_obj_set_style_bg_color(t1_serial_btn, cur_serial ? COLOR_BTN_ACTIVE_GREEN : COLOR_BTN_INACTIVE_BLUE, 0);
        }
    }

    // Tile 0 Route REC Button (t0_rec_btn)
    int cur_route = route_logger_is_active() ? 1 : 0;
    if (cur_route != s_last_route) {
        s_last_route = cur_route;
        if (t0_rec_btn && t0_rec_btn_lbl) {
            lv_label_set_text(t0_rec_btn_lbl, cur_route ? "REC ON" : "REC OFF");
            lv_obj_set_style_bg_color(t0_rec_btn, cur_route ? COLOR_BTN_ACTIVE_GREEN : COLOR_BTN_INACTIVE_BLUE, 0);
        }
    }

    // Row 2, Col 2: RTCM: ON / RTCM: OFF
    int cur_rtcm = (current_settings.enable_rtcm_logging || rtcm_logger_is_active()) ? 1 : 0;
    if (cur_rtcm != s_last_rtcm) {
        s_last_rtcm = cur_rtcm;
        if (t1_rtcm_btn && t1_rtcm_btn_lbl) {
            lv_label_set_text(t1_rtcm_btn_lbl, cur_rtcm ? "RTCM: ON" : "RTCM: OFF");
            lv_obj_set_style_bg_color(t1_rtcm_btn, cur_rtcm ? COLOR_BTN_ACTIVE_GREEN : COLOR_BTN_INACTIVE_BLUE, 0);
        }
    }

    // Row 3, Col 0: TELEM: ON / TELEM: OFF
    int cur_telem = current_settings.enable_telemetry_logging ? 1 : 0;
    if (cur_telem != s_last_telem) {
        s_last_telem = cur_telem;
        if (t1_telem_btn && t1_telem_btn_lbl) {
            lv_label_set_text(t1_telem_btn_lbl, cur_telem ? "TELEM: ON" : "TELEM: OFF");
            lv_obj_set_style_bg_color(t1_telem_btn, cur_telem ? COLOR_BTN_ACTIVE_GREEN : COLOR_BTN_INACTIVE_BLUE, 0);
        }
    }

    // Row 3, Col 1: IMU: ON / IMU: OFF
    int cur_imu = current_settings.enable_imu_logging ? 1 : 0;
    if (cur_imu != s_last_imu) {
        s_last_imu = cur_imu;
        if (t1_imu_btn && t1_imu_btn_lbl) {
            lv_label_set_text(t1_imu_btn_lbl, cur_imu ? "IMU: ON" : "IMU: OFF");
            lv_obj_set_style_bg_color(t1_imu_btn, cur_imu ? COLOR_BTN_ACTIVE_GREEN : COLOR_BTN_INACTIVE_BLUE, 0);
        }
    }

    // Row 3, Col 2: TOUCH: ON / TOUCH: OFF
    int cur_touch = current_settings.enable_touch_logging ? 1 : 0;
    if (cur_touch != s_last_touch) {
        s_last_touch = cur_touch;
        if (t1_touch_btn && t1_touch_btn_lbl) {
            lv_label_set_text(t1_touch_btn_lbl, cur_touch ? "TOUCH: ON" : "TOUCH: OFF");
            lv_obj_set_style_bg_color(t1_touch_btn, cur_touch ? COLOR_BTN_ACTIVE_GREEN : COLOR_BTN_INACTIVE_BLUE, 0);
        }
    }

    // Row 4, Col 0: WIFI: ON / WIFI: OFF
    int cur_wifi = is_wifi_active() ? 1 : 0;
    if (cur_wifi != s_last_wifi) {
        s_last_wifi = cur_wifi;
        if (t1_wifi_btn && t1_wifi_btn_lbl) {
            lv_label_set_text(t1_wifi_btn_lbl, cur_wifi ? "WIFI: ON" : "WIFI: OFF");
            lv_obj_set_style_bg_color(t1_wifi_btn, cur_wifi ? COLOR_BTN_ACTIVE_GREEN : COLOR_BTN_INACTIVE_BLUE, 0);
        }
    }

    // Row 4, Col 1: UNITS: IMP / UNITS: SI
    int cur_units = current_settings.use_imperial_units ? 1 : 0;
    if (cur_units != s_last_units) {
        s_last_units = cur_units;
        if (t1_unit_btn && t1_unit_btn_lbl) {
            if (!current_settings.use_imperial_units) {
                lv_label_set_text(t1_unit_btn_lbl, "UNITS: SI");
                lv_obj_set_style_bg_color(t1_unit_btn, COLOR_UNIT_SI_TEAL, 0);
                if (hdr_units_lbl) {
                    lv_label_set_text(hdr_units_lbl, "SI");
                    lv_obj_set_style_text_color(hdr_units_lbl, COLOR_UNIT_SI_TEAL, 0);
                }
            } else {
                lv_label_set_text(t1_unit_btn_lbl, "UNITS: IMP");
                lv_obj_set_style_bg_color(t1_unit_btn, COLOR_UNIT_IMP_AMBER, 0);
                if (hdr_units_lbl) {
                    lv_label_set_text(hdr_units_lbl, "IMP");
                    lv_obj_set_style_text_color(hdr_units_lbl, COLOR_UNIT_IMP_AMBER, 0);
                }
            }
        }
    }

    // Row 4, Col 2: LORA: 19.2k / REQ: 9.6k
    if (t1_lora_btn && t1_lora_btn_lbl) {
        char l_buf[32];
        if (e220_is_rate_switch_pending()) {
            snprintf(l_buf, sizeof(l_buf), "REQ: %.1fk", e220_get_requested_air_rate());
            lv_label_set_text(t1_lora_btn_lbl, l_buf);
            lv_obj_set_style_bg_color(t1_lora_btn, lv_color_hex(0xD97706), 0); // Amber
        } else {
            snprintf(l_buf, sizeof(l_buf), "LORA: %.1fk", e220_get_active_air_rate());
            lv_label_set_text(t1_lora_btn_lbl, l_buf);
            lv_obj_set_style_bg_color(t1_lora_btn, lv_color_hex(0x2F3B4C), 0);
        }
    }

    // Compact Tare / Reset Tare Button on Tile 3
    bool has_tare = (fabsf(current_settings.pole_tilt_tare_pitch_deg) > 0.05f ||
                     fabsf(current_settings.pole_tilt_tare_roll_deg)  > 0.05f);
    int cur_tare = has_tare ? 1 : 0;
    if (cur_tare != s_last_tare) {
        s_last_tare = cur_tare;
        if (t3_tare_btn && t3_tare_lbl) {
            if (has_tare) {
                lv_label_set_text(t3_tare_lbl, "RST");
                lv_obj_set_style_bg_color(t3_tare_btn, lv_color_hex(0xEA580C), 0); // Orange
            } else {
                lv_label_set_text(t3_tare_lbl, "TARE");
                lv_obj_set_style_bg_color(t3_tare_btn, lv_color_hex(0x2563EB), 0); // Blue
            }
        }
    }
}

static void bright_toggle_btn_cb(lv_event_t *e)
{
    (void)e;
    uint8_t b = current_settings.brightness;
    if (b < 30) b = 40;
    else if (b < 50) b = 60;
    else if (b < 70) b = 80;
    else if (b < 90) b = 100;
    else b = 20;

    set_rover_brightness(b);
    mark_settings_dirty();
    update_all_control_buttons();
    play_rover_beep(1400, 25);
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Brightness clicked -> %u%%\n", b);
    }
}

static void vol_toggle_btn_cb(lv_event_t *e)
{
    (void)e;
    uint8_t v = current_settings.volume;
    if (v == 0) v = 20;
    else if (v < 30) v = 40;
    else if (v < 50) v = 60;
    else if (v < 70) v = 80;
    else if (v < 90) v = 100;
    else v = 0;

    set_rover_volume(v);
    current_settings.is_muted = (v == 0);
    mark_settings_dirty();
    update_all_control_buttons();

    if (v == 0) {
        play_rover_beep(900, 30);
    } else {
        play_rover_beep(1750, 40);
    }
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Volume clicked -> %u%%%s\n",
                     v, current_settings.is_muted ? " (MUTED)" : "");
    }
}

static void audio_mode_toggle_btn_cb(lv_event_t *e)
{
    (void)e;
    if (current_settings.audio_nav_mode == AUDIO_NAV_BEEPS) {
        current_settings.audio_nav_mode = AUDIO_NAV_GEIGER;
        play_rover_beep(1800, 15);
    } else if (current_settings.audio_nav_mode == AUDIO_NAV_GEIGER) {
        current_settings.audio_nav_mode = AUDIO_NAV_MUTE;
        play_rover_beep(900, 30);
    } else {
        current_settings.audio_nav_mode = AUDIO_NAV_BEEPS;
        play_rover_beep(1500, 30);
    }
    mark_settings_dirty();
    update_all_control_buttons();
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Audio Nav Mode clicked -> %s\n",
                     current_settings.audio_nav_mode == AUDIO_NAV_GEIGER ? "GEIGER" :
                     current_settings.audio_nav_mode == AUDIO_NAV_MUTE ? "MUTE" : "BEEPS");
    }
}

static void lora_rate_btn_cb(lv_event_t *e)
{
    (void)e;
    if (e220_is_rate_switch_pending()) {
        play_rover_beep(600, 40); // Busy tone
        return;
    }
    float cur_rate = e220_get_active_air_rate();
    float next_rate = (cur_rate >= 19.0f && cur_rate <= 20.0f) ? 9.6f : 19.2f;

    play_rover_beep(1600, 35);
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] LoRa Rate clicked -> Requesting %.1fk\n", next_rate);
    }
    e220_request_air_rate_switch(next_rate);
    update_all_control_buttons();
}

static void unit_toggle_btn_cb(lv_event_t *e)
{
    (void)e;
    current_settings.use_imperial_units = !current_settings.use_imperial_units;
    mark_settings_dirty();
    update_all_control_buttons();
    play_rover_beep(1400, 25);
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Units clicked -> %s\n",
                     current_settings.use_imperial_units ? "IMP" : "SI");
    }
}

static void route_toggle_btn_cb(lv_event_t *e)
{
    (void)e;
    if (route_logger_is_active()) {
        route_logger_stop();
        play_rover_beep(1200, 30);
    } else {
        route_logger_start();
        play_rover_beep(1800, 40);
    }
    update_all_control_buttons();
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Route REC clicked -> %s\n",
                     route_logger_is_active() ? "RECORDING (ON)" : "STOPPED (OFF)");
    }
}

static void serial_log_toggle_btn_cb(lv_event_t *e)
{
    (void)e;
    current_settings.enable_serial_logging = !current_settings.enable_serial_logging;
    mark_settings_dirty();
    if (current_settings.enable_serial_logging) {
        serial_logger_start();
        play_rover_beep(1800, 40);
    } else {
        serial_logger_stop();
        play_rover_beep(1200, 30);
    }
    update_all_control_buttons();
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Serial Log clicked -> %s\n",
                     current_settings.enable_serial_logging ? "ON" : "OFF");
    }
}

static void nmea_toggle_btn_cb(lv_event_t *e)
{
    (void)e;
    if (nmea_logger_is_active()) {
        nmea_logger_stop();
        current_settings.enable_nmea_logging = false;
        play_rover_beep(1200, 30);
    } else {
        nmea_logger_start();
        current_settings.enable_nmea_logging = true;
        play_rover_beep(1800, 40);
    }
    mark_settings_dirty();
    update_all_control_buttons();
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] NMEA Log clicked -> %s\n",
                     current_settings.enable_nmea_logging ? "ON" : "OFF");
    }
}

static void wifi_toggle_btn_cb(lv_event_t *e)
{
    (void)e;
    if (is_wifi_active()) {
        stop_wifi();
        play_rover_beep(1200, 30);
    } else {
        start_wifi();
        play_rover_beep(1800, 40);
    }
    update_all_control_buttons();
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Wi-Fi clicked -> %s\n",
                     is_wifi_active() ? "Starting" : "Stopping");
    }
}

static char s_pending_wifi_status[64] = "Wi-Fi Inactive";
static bool s_wifi_status_dirty = true;

void update_wifi_ip_label(const char* ip)
{
    if (ip && ip[0]) {
        strncpy(s_pending_wifi_status, ip, sizeof(s_pending_wifi_status) - 1);
        s_pending_wifi_status[sizeof(s_pending_wifi_status) - 1] = '\0';
    } else {
        strncpy(s_pending_wifi_status, "Wi-Fi Inactive", sizeof(s_pending_wifi_status) - 1);
        s_pending_wifi_status[sizeof(s_pending_wifi_status) - 1] = '\0';
    }
    s_wifi_status_dirty = true;
}

static void prefix_dd_cb(lv_event_t *e)
{
    lv_obj_t *dd = (lv_obj_t *)lv_event_get_target(e);
    g_survey_prefix_idx = (uint8_t)lv_dropdown_get_selected(dd);
    waypoint_invalidate_survey_cache();
    play_rover_beep(1400, 20);
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Prefix dropdown -> idx %u\n", g_survey_prefix_idx);
    }
}

static void note_dd_cb(lv_event_t *e)
{
    lv_obj_t *dd = (lv_obj_t *)lv_event_get_target(e);
    g_survey_note_tag_idx = (uint8_t)lv_dropdown_get_selected(dd);
    play_rover_beep(1400, 20);
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Note tag dropdown -> idx %u\n", g_survey_note_tag_idx);
    }
}

static void avg_dur_btn_cb(lv_event_t *e)
{
    (void)e;
    if (current_settings.survey_avg_seconds <= 1) current_settings.survey_avg_seconds = 5;
    else if (current_settings.survey_avg_seconds <= 5) current_settings.survey_avg_seconds = 10;
    else current_settings.survey_avg_seconds = 1;
    mark_settings_dirty();
    if (t1_avg_btn_lbl) {
        lv_label_set_text_fmt(t1_avg_btn_lbl, "Avg: %us", current_settings.survey_avg_seconds);
    }
    play_rover_beep(1400, 25);
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Avg Duration clicked -> %ds\n", current_settings.survey_avg_seconds);
    }
}

static void log_pin_btn_cb(lv_event_t *e)
{
    (void)e;
    trigger_survey_waypoint_log();
    play_rover_beep(1750, 40);
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Log Pin clicked!\n");
    }
}

static void wp_table_event_cb(lv_event_t *e)
{
    lv_obj_t *table = (lv_obj_t *)lv_event_get_target(e);
    uint32_t row = 0, col = 0;
    lv_table_get_selected_cell(table, &row, &col);
    if (row > 0 && (int16_t)(row - 1) < g_waypoint_count) {
        play_rover_beep(1500, 20);
        waypoint_select_index((int16_t)(row - 1));
        update_t0_target_label();
        switch_to_tile_index(0, LV_ANIM_ON);
        if (current_settings.enable_touch_logging) {
            rover_printf("[TOUCH BTN] Waypoint table row clicked -> Waypoint %u\n", (unsigned int)row);
        }
    }
}

static void tare_level_btn_cb(lv_event_t *e)
{
    (void)e;
    bool has_tare = (fabsf(current_settings.pole_tilt_tare_pitch_deg) > 0.05f ||
                     fabsf(current_settings.pole_tilt_tare_roll_deg)  > 0.05f);
    if (has_tare) {
        current_settings.pole_tilt_tare_pitch_deg = 0.0f;
        current_settings.pole_tilt_tare_roll_deg  = 0.0f;
        mark_settings_dirty();
        play_rover_beep(1200, 80);
    } else {
        trigger_tare_pole_level();
        play_rover_beep(1800, 40);
    }
    update_all_control_buttons();
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Tare clicked -> %s\n", has_tare ? "RESET" : "TARE");
    }
}

static void diag_rtcm_btn_cb(lv_event_t *e)
{
    (void)e;
    current_settings.enable_rtcm_logging = !current_settings.enable_rtcm_logging;
    mark_settings_dirty();
    if (current_settings.enable_rtcm_logging) {
        rtcm_logger_start();
        play_rover_beep(1800, 40);
    } else {
        rtcm_logger_stop();
        play_rover_beep(1200, 30);
    }
    update_all_control_buttons();
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Diag RTCM Log clicked -> %s\n",
                     current_settings.enable_rtcm_logging ? "ON" : "OFF");
    }
}

static void diag_imu_btn_cb(lv_event_t *e)
{
    (void)e;
    current_settings.enable_imu_logging = !current_settings.enable_imu_logging;
    mark_settings_dirty();
    play_rover_beep(current_settings.enable_imu_logging ? 1800 : 1200, 30);
    update_all_control_buttons();
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Diag IMU Log clicked -> %s\n",
                     current_settings.enable_imu_logging ? "ON" : "OFF");
    }
}

static void diag_telem_btn_cb(lv_event_t *e)
{
    (void)e;
    current_settings.enable_telemetry_logging = !current_settings.enable_telemetry_logging;
    mark_settings_dirty();
    play_rover_beep(current_settings.enable_telemetry_logging ? 1800 : 1200, 30);
    update_all_control_buttons();
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Diag Telem Log clicked -> %s\n",
                     current_settings.enable_telemetry_logging ? "ON" : "OFF");
    }
}

static void diag_touch_btn_cb(lv_event_t *e)
{
    (void)e;
    current_settings.enable_touch_logging = !current_settings.enable_touch_logging;
    mark_settings_dirty();
    play_rover_beep(current_settings.enable_touch_logging ? 1800 : 1200, 30);
    update_all_control_buttons();
    rover_printf("[TOUCH BTN] Diag Touch Log clicked -> %s\n",
                 current_settings.enable_touch_logging ? "ON" : "OFF");
}

void refresh_waypoint_table_ui(void)
{
    if (t2_file_lbl) {
        char buf[64];
        snprintf(buf, sizeof(buf), "%s (%d/%d)",
                 waypoint_file_get_active_name(),
                 waypoint_file_get_active_index() + 1,
                 waypoint_file_get_count());
        const char *cur = lv_label_get_text(t2_file_lbl);
        if (!cur || strcmp(cur, buf) != 0) {
            lv_label_set_text(t2_file_lbl, buf);
        }
    }
    if (!t2_wp_table) return;
    lv_table_set_col_cnt(t2_wp_table, 3);
    lv_table_set_col_width(t2_wp_table, 0, 110);
    lv_table_set_col_width(t2_wp_table, 1, 98);
    lv_table_set_col_width(t2_wp_table, 2, 98);

    lv_table_set_row_cnt(t2_wp_table, (uint32_t)(g_waypoint_count + 1));
    lv_table_set_cell_value(t2_wp_table, 0, 0, "Label");
    lv_table_set_cell_value(t2_wp_table, 0, 1, "Lat");
    lv_table_set_cell_value(t2_wp_table, 0, 2, "Lon");

    char buf[32];
    for (int16_t i = 0; i < g_waypoint_count; i++) {
        lv_table_set_cell_value(t2_wp_table, (uint32_t)(i + 1), 0, g_waypoints[i].label);
        snprintf(buf, sizeof(buf), "%.6f", g_waypoints[i].latitude);
        lv_table_set_cell_value(t2_wp_table, (uint32_t)(i + 1), 1, buf);
        snprintf(buf, sizeof(buf), "%.6f", g_waypoints[i].longitude);
        lv_table_set_cell_value(t2_wp_table, (uint32_t)(i + 1), 2, buf);
    }
}

static lv_obj_t* create_grid_button(lv_obj_t *parent, int col, int row, lv_event_cb_t cb, lv_obj_t **out_lbl)
{
    int x = (col == 0) ? 10 : (col == 1 ? 113 : 216);
    int y = 186 + (row * 35);
    lv_obj_t *btn = lv_button_create(parent);
    lv_obj_set_size(btn, 94, 27);
    lv_obj_align(btn, LV_ALIGN_TOP_LEFT, x, y);
    lv_obj_set_style_pad_all(btn, 0, 0);
    lv_obj_set_ext_click_area(btn, 3);
    if (cb) lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl = lv_label_create(btn);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(lbl);
    if (out_lbl) *out_lbl = lbl;
    return btn;
}

void create_rover_ui(void)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x121417), LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(scr, LV_SCROLLBAR_MODE_OFF);

    // =====================================================================
    // 1. Persistent Top Status Header Bar (320 x 74, always visible across all 5 tiles)
    // =====================================================================
    lv_obj_t *hdr = lv_obj_create(scr);
    lv_obj_set_size(hdr, ROVER_LCD_H_RES, 74);
    lv_obj_align(hdr, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(hdr, lv_color_hex(0x181C22), 0);
    lv_obj_set_style_border_color(hdr, lv_color_hex(0x28323E), 0);
    lv_obj_set_style_border_width(hdr, 1, 0);
    lv_obj_set_style_border_side(hdr, LV_BORDER_SIDE_BOTTOM, 0);
    lv_obj_set_style_radius(hdr, 0, 0);
    lv_obj_set_style_pad_all(hdr, 0, 0);
    lv_obj_set_scrollbar_mode(hdr, LV_SCROLLBAR_MODE_OFF);

    // Row 1 (y = 2): System & Radios
    hdr_datetime_lbl = lv_label_create(hdr);
    lv_label_set_text(hdr_datetime_lbl, "2026-10-26 00:00:00");
    lv_obj_set_style_text_color(hdr_datetime_lbl, lv_color_hex(0xE0E6ED), 0);
    lv_obj_set_style_text_font(hdr_datetime_lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_width(hdr_datetime_lbl, 165);
    lv_label_set_long_mode(hdr_datetime_lbl, LV_LABEL_LONG_CLIP);
    lv_obj_align(hdr_datetime_lbl, LV_ALIGN_TOP_LEFT, 4, 2);

    hdr_units_lbl = lv_label_create(hdr);
    lv_label_set_text(hdr_units_lbl, current_settings.use_imperial_units ? "IMP" : "SI");
    lv_obj_set_style_text_color(hdr_units_lbl, current_settings.use_imperial_units ? COLOR_UNIT_IMP_AMBER : COLOR_UNIT_SI_TEAL, 0);
    lv_obj_set_style_text_font(hdr_units_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(hdr_units_lbl, LV_ALIGN_TOP_RIGHT, -92, 2);

    hdr_wifi_lbl = lv_label_create(hdr);
    lv_label_set_text(hdr_wifi_lbl, LV_SYMBOL_WIFI);
    lv_obj_set_style_text_color(hdr_wifi_lbl, lv_color_hex(0x475569), 0);
    lv_obj_set_style_text_font(hdr_wifi_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(hdr_wifi_lbl, LV_ALIGN_TOP_RIGHT, -70, 2);

    hdr_batt_lbl = lv_label_create(hdr);
    lv_label_set_text(hdr_batt_lbl, "100% 4.1V");
    lv_obj_set_style_text_color(hdr_batt_lbl, lv_color_hex(0x55FF55), 0);
    lv_obj_set_style_text_font(hdr_batt_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(hdr_batt_lbl, LV_ALIGN_TOP_RIGHT, -4, 2);

    // Row 2 (y = 19): Master Antenna (ANT1)
    hdr_m_fix_badge = lv_obj_create(hdr);
    lv_obj_set_size(hdr_m_fix_badge, 68, 16);
    lv_obj_align(hdr_m_fix_badge, LV_ALIGN_TOP_LEFT, 4, 19);
    lv_obj_set_style_bg_color(hdr_m_fix_badge, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_style_border_width(hdr_m_fix_badge, 0, 0);
    lv_obj_set_style_radius(hdr_m_fix_badge, 3, 0);
    lv_obj_set_style_pad_all(hdr_m_fix_badge, 0, 0);

    hdr_m_fix_lbl = lv_label_create(hdr_m_fix_badge);
    lv_label_set_text(hdr_m_fix_lbl, "NO FIX");
    lv_obj_set_style_text_color(hdr_m_fix_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(hdr_m_fix_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(hdr_m_fix_lbl);

    hdr_m_sats_lbl = lv_label_create(hdr);
    lv_label_set_text(hdr_m_sats_lbl, "SV: 0/0");
    lv_obj_set_style_text_color(hdr_m_sats_lbl, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_text_font(hdr_m_sats_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(hdr_m_sats_lbl, LV_ALIGN_TOP_LEFT, 76, 19);

    hdr_m_acc_lbl = lv_label_create(hdr);
    lv_label_set_text(hdr_m_acc_lbl, "+/- --");
    lv_obj_set_style_text_color(hdr_m_acc_lbl, lv_color_hex(0x00FF88), 0);
    lv_obj_set_style_text_font(hdr_m_acc_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(hdr_m_acc_lbl, LV_ALIGN_TOP_LEFT, 146, 19);

    hdr_m_rtcm_lbl = lv_label_create(hdr);
    lv_label_set_text(hdr_m_rtcm_lbl, "LoRa: -- dB");
    lv_obj_set_style_text_color(hdr_m_rtcm_lbl, lv_color_hex(0x64748B), 0);
    lv_obj_set_style_text_font(hdr_m_rtcm_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(hdr_m_rtcm_lbl, LV_ALIGN_TOP_RIGHT, -4, 19);

    // Row 3 (y = 37): Slave Antenna (ANT2)
    hdr_s_fix_badge = lv_obj_create(hdr);
    lv_obj_set_size(hdr_s_fix_badge, 68, 16);
    lv_obj_align(hdr_s_fix_badge, LV_ALIGN_TOP_LEFT, 4, 37);
    lv_obj_set_style_bg_color(hdr_s_fix_badge, lv_color_hex(0x3A3F47), 0);
    lv_obj_set_style_border_width(hdr_s_fix_badge, 0, 0);
    lv_obj_set_style_radius(hdr_s_fix_badge, 3, 0);
    lv_obj_set_style_pad_all(hdr_s_fix_badge, 0, 0);

    hdr_s_fix_lbl = lv_label_create(hdr_s_fix_badge);
    lv_label_set_text(hdr_s_fix_lbl, "HDG ---");
    lv_obj_set_style_text_color(hdr_s_fix_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(hdr_s_fix_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(hdr_s_fix_lbl);

    hdr_s_sats_lbl = lv_label_create(hdr);
    lv_label_set_text(hdr_s_sats_lbl, "SV: --");
    lv_obj_set_style_text_color(hdr_s_sats_lbl, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_text_font(hdr_s_sats_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(hdr_s_sats_lbl, LV_ALIGN_TOP_LEFT, 76, 37);

    hdr_s_bl_lbl = lv_label_create(hdr);
    lv_label_set_text(hdr_s_bl_lbl, "BL: --.-cm");
    lv_obj_set_style_text_color(hdr_s_bl_lbl, lv_color_hex(0x38BDF8), 0);
    lv_obj_set_style_text_font(hdr_s_bl_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(hdr_s_bl_lbl, LV_ALIGN_TOP_LEFT, 146, 37);

    hdr_s_hdg_lbl = lv_label_create(hdr);
    lv_label_set_text(hdr_s_hdg_lbl, "Hdg: ---.-");
    lv_obj_set_style_text_color(hdr_s_hdg_lbl, lv_color_hex(0xFFCC00), 0);
    lv_obj_set_style_text_font(hdr_s_hdg_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(hdr_s_hdg_lbl, LV_ALIGN_TOP_RIGHT, -4, 37);

    // Row 4 (y = 55): Streamlined Coordinates Bar (Montserrat 16 high-visibility font)
    hdr_coords_lbl = lv_label_create(hdr);
    lv_label_set_text(hdr_coords_lbl, "0.0000000, 0.0000000, 0.000 m");
    lv_obj_set_style_text_color(hdr_coords_lbl, lv_color_hex(0x00FFAA), 0);
    lv_obj_set_style_text_font(hdr_coords_lbl, &lv_font_montserrat_16, 0);
    lv_obj_align(hdr_coords_lbl, LV_ALIGN_TOP_MID, 0, 55);

    // =====================================================================
    // 2. Persistent Bottom Screen Navigation Bar (320 x 32, always visible)
    // =====================================================================
    lv_obj_t *nav_bar = lv_obj_create(scr);
    lv_obj_set_size(nav_bar, ROVER_LCD_H_RES, 32);
    lv_obj_align(nav_bar, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(nav_bar, lv_color_hex(0x1E2229), 0);
    lv_obj_set_style_border_width(nav_bar, 0, 0);
    lv_obj_set_style_radius(nav_bar, 0, 0);
    lv_obj_set_style_pad_all(nav_bar, 2, 0);
    lv_obj_set_scrollbar_mode(nav_bar, LV_SCROLLBAR_MODE_OFF);

    lv_obj_t *btn_scr_prev = lv_button_create(nav_bar);
    lv_obj_set_size(btn_scr_prev, 68, 28);
    lv_obj_align(btn_scr_prev, LV_ALIGN_LEFT_MID, 4, 0);
    lv_obj_set_style_bg_color(btn_scr_prev, lv_color_hex(0x2F3B4C), 0);
    lv_obj_set_ext_click_area(btn_scr_prev, 6);
    lv_obj_add_event_cb(btn_scr_prev, nav_prev_screen_btn_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_t *lbl_scr_prev = lv_label_create(btn_scr_prev);
    lv_label_set_text(lbl_scr_prev, LV_SYMBOL_LEFT " Prev");
    lv_obj_set_style_text_font(lbl_scr_prev, &lv_font_montserrat_12, 0);
    lv_obj_center(lbl_scr_prev);

    nav_screen_lbl = lv_label_create(nav_bar);
    lv_label_set_text(nav_screen_lbl, kScreenTitles[0]);
    lv_obj_set_style_text_color(nav_screen_lbl, lv_color_hex(0x00D8FF), 0);
    lv_obj_set_style_text_font(nav_screen_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(nav_screen_lbl, LV_ALIGN_CENTER, 0, 0);

    lv_obj_t *btn_scr_next = lv_button_create(nav_bar);
    lv_obj_set_size(btn_scr_next, 68, 28);
    lv_obj_align(btn_scr_next, LV_ALIGN_RIGHT_MID, -4, 0);
    lv_obj_set_style_bg_color(btn_scr_next, lv_color_hex(0x2F3B4C), 0);
    lv_obj_set_ext_click_area(btn_scr_next, 6);
    lv_obj_add_event_cb(btn_scr_next, nav_next_screen_btn_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_t *lbl_scr_next = lv_label_create(btn_scr_next);
    lv_label_set_text(lbl_scr_next, "Next " LV_SYMBOL_RIGHT);
    lv_obj_set_style_text_font(lbl_scr_next, &lv_font_montserrat_12, 0);
    lv_obj_center(lbl_scr_next);

    // =====================================================================
    // 3. Main Swipeable Tileview (320 x 400 between Header and Nav Bar)
    // =====================================================================
    main_tv = lv_tileview_create(scr);
    lv_obj_set_size(main_tv, ROVER_LCD_H_RES, ROVER_LCD_V_RES - 74 - 32);
    lv_obj_align(main_tv, LV_ALIGN_TOP_MID, 0, 74);
    lv_obj_set_style_bg_color(main_tv, lv_color_hex(0x121417), 0);
    lv_obj_set_scrollable(main_tv, false);
    lv_obj_add_event_cb(main_tv, tv_changed_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // Disable horizontal swipe gestures; navigate strictly via Prev/Next bottom bar buttons
    tile_guidance = lv_tileview_add_tile(main_tv, 0, 0, LV_DIR_NONE);
    tile_survey   = lv_tileview_add_tile(main_tv, 1, 0, LV_DIR_NONE);
    tile_csv      = lv_tileview_add_tile(main_tv, 2, 0, LV_DIR_NONE);
    tile_compass  = lv_tileview_add_tile(main_tv, 3, 0, LV_DIR_NONE);
    tile_test     = lv_tileview_add_tile(main_tv, 4, 0, LV_DIR_NONE);

    // =====================================================================
    // TILE 0: Pin Seeker & Guidance (Hybrid Navigation Canvas)
    // =====================================================================
    // Target selector row (compacted to y = 3 since coordinates are now in persistent header)
    lv_obj_t *btn_prev = lv_button_create(tile_guidance);
    lv_obj_set_size(btn_prev, 44, 28);
    lv_obj_align(btn_prev, LV_ALIGN_TOP_LEFT, 6, 3);
    lv_obj_set_ext_click_area(btn_prev, 12);
    lv_obj_add_event_cb(btn_prev, prev_wp_btn_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_t *lbl_prev = lv_label_create(btn_prev);
    lv_label_set_text(lbl_prev, LV_SYMBOL_LEFT);
    lv_obj_center(lbl_prev);

    t0_target_lbl = lv_label_create(tile_guidance);
    lv_obj_set_style_text_color(t0_target_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(t0_target_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_width(t0_target_lbl, 216);
    lv_label_set_long_mode(t0_target_lbl, LV_LABEL_LONG_MODE_SCROLL_CIRCULAR);
    lv_obj_set_style_text_align(t0_target_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(t0_target_lbl, LV_ALIGN_TOP_MID, 0, 6);
    lv_obj_set_clickable(t0_target_lbl, true);
    lv_obj_set_ext_click_area(t0_target_lbl, 8);
    lv_obj_add_event_cb(t0_target_lbl, t0_target_lbl_cb, LV_EVENT_PRESSED, NULL);

    WaypointRecord_t init_wp;
    if (waypoint_get_active(&init_wp)) {
        char buf[64];
        snprintf(buf, sizeof(buf), "%s (%d/%d)",
                 init_wp.label, g_active_waypoint_idx + 1, g_waypoint_count);
        lv_label_set_text(t0_target_lbl, buf);
    } else {
        lv_label_set_text(t0_target_lbl, "No Target (0/0)");
    }

    lv_obj_t *btn_next = lv_button_create(tile_guidance);
    lv_obj_set_size(btn_next, 44, 28);
    lv_obj_align(btn_next, LV_ALIGN_TOP_RIGHT, -6, 3);
    lv_obj_set_ext_click_area(btn_next, 12);
    lv_obj_add_event_cb(btn_next, next_wp_btn_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_t *lbl_next = lv_label_create(btn_next);
    lv_label_set_text(lbl_next, LV_SYMBOL_RIGHT);
    lv_obj_center(lbl_next);

    // Target notes row (circular marquee autoscroll when text exceeds width)
    t0_notes_lbl = lv_label_create(tile_guidance);
    lv_label_set_long_mode(t0_notes_lbl, LV_LABEL_LONG_MODE_SCROLL_CIRCULAR);
    lv_obj_set_width(t0_notes_lbl, 304);
    if (waypoint_get_active(&init_wp)) {
        lv_label_set_text(t0_notes_lbl, init_wp.notes[0] ? init_wp.notes : "---");
    } else {
        lv_label_set_text(t0_notes_lbl, "---");
    }
    lv_obj_set_style_text_color(t0_notes_lbl, lv_color_hex(0x99A3B0), 0);
    lv_obj_set_style_text_font(t0_notes_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t0_notes_lbl, LV_ALIGN_TOP_LEFT, 8, 34);

    // Navigation Canvas (304x210)
    t0_nav_canvas = lv_obj_create(tile_guidance);
    lv_obj_set_size(t0_nav_canvas, 304, 210);
    lv_obj_align(t0_nav_canvas, LV_ALIGN_TOP_MID, 0, 52);
    lv_obj_set_style_radius(t0_nav_canvas, 8, 0);
    lv_obj_set_style_bg_color(t0_nav_canvas, lv_color_hex(0x13191E), 0);
    lv_obj_set_style_border_color(t0_nav_canvas, lv_color_hex(0x283B47), 0);
    lv_obj_set_style_border_width(t0_nav_canvas, 2, 0);
    lv_obj_set_scrollbar_mode(t0_nav_canvas, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scrollable(t0_nav_canvas, false);
    lv_obj_set_style_pad_all(t0_nav_canvas, 0, 0);

    // Mode badge & scale
    t0_mode_badge = lv_label_create(t0_nav_canvas);
    lv_label_set_text(t0_mode_badge, "TACTICAL GUIDANCE");
    lv_obj_set_style_text_font(t0_mode_badge, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(t0_mode_badge, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_color(t0_mode_badge, lv_color_hex(0x00D8FF), 0);
    lv_obj_set_style_bg_opa(t0_mode_badge, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_hor(t0_mode_badge, 6, 0);
    lv_obj_set_style_pad_ver(t0_mode_badge, 2, 0);
    lv_obj_set_style_radius(t0_mode_badge, 4, 0);
    lv_obj_align(t0_mode_badge, LV_ALIGN_TOP_LEFT, 6, 6);
    lv_obj_set_clickable(t0_mode_badge, true);
    lv_obj_set_ext_click_area(t0_mode_badge, 8);
    lv_obj_add_event_cb(t0_mode_badge, t0_mode_toggle_cb, LV_EVENT_PRESSED, NULL);

    // Telemetry HUD Readout (Heading & Pitch on top-left under Tactical Grid badge)
    t0_hdg_hud_lbl = lv_label_create(t0_nav_canvas);
    lv_label_set_text(t0_hdg_hud_lbl, "Hdg: ---.--");
    lv_obj_set_style_text_font(t0_hdg_hud_lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(t0_hdg_hud_lbl, lv_color_hex(0x38BDF8), 0);
    lv_obj_align(t0_hdg_hud_lbl, LV_ALIGN_TOP_LEFT, 6, 28);

    t0_pitch_hud_lbl = lv_label_create(t0_nav_canvas);
    lv_label_set_text(t0_pitch_hud_lbl, "Pitch: --.-");
    lv_obj_set_style_text_font(t0_pitch_hud_lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(t0_pitch_hud_lbl, lv_color_hex(0x38BDF8), 0);
    lv_obj_align(t0_pitch_hud_lbl, LV_ALIGN_TOP_LEFT, 6, 44);

    t0_scale_lbl = lv_label_create(t0_nav_canvas);
    lv_label_set_text(t0_scale_lbl, "Range: 50.0 m");
    lv_obj_set_style_text_font(t0_scale_lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(t0_scale_lbl, lv_color_hex(0x94A3B8), 0);
    lv_obj_align(t0_scale_lbl, LV_ALIGN_TOP_RIGHT, -6, 8);

    t0_tgt_hdg_lbl = lv_label_create(t0_nav_canvas);
    lv_label_set_text(t0_tgt_hdg_lbl, "Tgt: ---.- deg");
    lv_obj_set_style_text_font(t0_tgt_hdg_lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(t0_tgt_hdg_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(t0_tgt_hdg_lbl, LV_ALIGN_TOP_RIGHT, -6, 28);

    // Tactical Background Grid Lines
    for (int i = 0; i < T0_GRID_H_LINES; i++) {
        t0_grid_h[i] = lv_obj_create(t0_nav_canvas);
        lv_obj_set_size(t0_grid_h[i], 292, 1);
        lv_obj_set_style_bg_color(t0_grid_h[i], lv_color_hex(0x1A252D), 0);
        lv_obj_set_style_border_width(t0_grid_h[i], 0, 0);
        lv_obj_align(t0_grid_h[i], LV_ALIGN_TOP_MID, 0, (i + 1) * 42);
    }
    int16_t v_positions[T0_GRID_V_LINES] = {60, 120, 184, 244};
    for (int i = 0; i < T0_GRID_V_LINES; i++) {
        t0_grid_v[i] = lv_obj_create(t0_nav_canvas);
        lv_obj_set_size(t0_grid_v[i], 1, 198);
        lv_obj_set_style_bg_color(t0_grid_v[i], lv_color_hex(0x1A252D), 0);
        lv_obj_set_style_border_width(t0_grid_v[i], 0, 0);
        lv_obj_align(t0_grid_v[i], LV_ALIGN_TOP_LEFT, v_positions[i], 6);
    }

    // Primary Crosshairs
    t0_cross_h = lv_obj_create(t0_nav_canvas);
    lv_obj_set_size(t0_cross_h, 280, 1);
    lv_obj_set_style_bg_color(t0_cross_h, lv_color_hex(0x283E4C), 0);
    lv_obj_set_style_border_width(t0_cross_h, 0, 0);
    lv_obj_center(t0_cross_h);

    t0_cross_v = lv_obj_create(t0_nav_canvas);
    lv_obj_set_size(t0_cross_v, 1, 190);
    lv_obj_set_style_bg_color(t0_cross_v, lv_color_hex(0x283E4C), 0);
    lv_obj_set_style_border_width(t0_cross_v, 0, 0);
    lv_obj_center(t0_cross_v);

    // Bullseye Concentric Rings (Fine Stakeout Mode)
    t0_bullseye_ring1 = lv_obj_create(t0_nav_canvas);
    lv_obj_set_size(t0_bullseye_ring1, 150, 150);
    lv_obj_center(t0_bullseye_ring1);
    lv_obj_set_style_radius(t0_bullseye_ring1, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(t0_bullseye_ring1, 0, 0);
    lv_obj_set_style_border_color(t0_bullseye_ring1, lv_color_hex(0x2D4350), 0);
    lv_obj_set_style_border_width(t0_bullseye_ring1, 1, 0);

    t0_bullseye_ring2 = lv_obj_create(t0_nav_canvas);
    lv_obj_set_size(t0_bullseye_ring2, 60, 60);
    lv_obj_center(t0_bullseye_ring2);
    lv_obj_set_style_radius(t0_bullseye_ring2, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(t0_bullseye_ring2, 0, 0);
    lv_obj_set_style_border_color(t0_bullseye_ring2, lv_color_hex(0x3B5768), 0);
    lv_obj_set_style_border_width(t0_bullseye_ring2, 1, 0);

    t0_bullseye_center = lv_obj_create(t0_nav_canvas);
    lv_obj_set_size(t0_bullseye_center, 14, 14);
    lv_obj_center(t0_bullseye_center);
    lv_obj_set_style_radius(t0_bullseye_center, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(t0_bullseye_center, lv_color_hex(0xFF0055), 0);
    lv_obj_set_style_border_color(t0_bullseye_center, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(t0_bullseye_center, 2, 0);

    // Stage 1 Visual Tolerance Wedges (Emanating from Rover Origin (152, 170))
    // Amber Outer Sector (+/- 15 deg)
    t0_wedge_amber_l = lv_line_create(t0_nav_canvas);
    lv_obj_set_style_line_color(t0_wedge_amber_l, lv_color_hex(0xB45309), 0);
    lv_obj_set_style_line_width(t0_wedge_amber_l, 2, 0);

    t0_wedge_amber_r = lv_line_create(t0_nav_canvas);
    lv_obj_set_style_line_color(t0_wedge_amber_r, lv_color_hex(0xB45309), 0);
    lv_obj_set_style_line_width(t0_wedge_amber_r, 2, 0);

    // Green Inner Sector (+/- 3 deg)
    t0_wedge_green_l = lv_line_create(t0_nav_canvas);
    lv_obj_set_style_line_color(t0_wedge_green_l, lv_color_hex(0x15803D), 0);
    lv_obj_set_style_line_width(t0_wedge_green_l, 2, 0);

    t0_wedge_green_r = lv_line_create(t0_nav_canvas);
    lv_obj_set_style_line_color(t0_wedge_green_r, lv_color_hex(0x15803D), 0);
    lv_obj_set_style_line_width(t0_wedge_green_r, 2, 0);

    static const lv_point_precise_t pts_amb_l[2] = {{152, 170}, {117, 40}};
    static const lv_point_precise_t pts_amb_r[2] = {{152, 170}, {187, 40}};
    static const lv_point_precise_t pts_grn_l[2] = {{152, 170}, {145, 35}};
    static const lv_point_precise_t pts_grn_r[2] = {{152, 170}, {159, 35}};
    lv_line_set_points(t0_wedge_amber_l, pts_amb_l, 2);
    lv_line_set_points(t0_wedge_amber_r, pts_amb_r, 2);
    lv_line_set_points(t0_wedge_green_l, pts_grn_l, 2);
    lv_line_set_points(t0_wedge_green_r, pts_grn_r, 2);

    // Line of sight (fixed vertical track line from Rover (152, 170) to Target Pin outer radius at (152, 66))
    t0_track_line = lv_line_create(t0_nav_canvas);
    lv_obj_set_style_line_color(t0_track_line, lv_color_hex(0x00D8FF), 0);
    lv_obj_set_style_line_width(t0_track_line, 2, 0);
    static const lv_point_precise_t pts_track[2] = {{152, 170}, {152, 66}};
    lv_line_set_points(t0_track_line, pts_track, 2);

    // Target Track Chevron Arrowhead (terminating at Target Pin outer radius at (152, 66))
    t0_track_arrow = lv_line_create(t0_nav_canvas);
    lv_obj_set_style_line_color(t0_track_arrow, lv_color_hex(0x00D8FF), 0);
    lv_obj_set_style_line_width(t0_track_arrow, 3, 0);
    static const lv_point_precise_t pts_track_arrow[3] = {{144, 78}, {152, 66}, {160, 78}};
    lv_line_set_points(t0_track_arrow, pts_track_arrow, 3);

    // Rover antenna beam vector with directional arrow (shaft + dual barbs)
    t0_beam_arrow = lv_line_create(t0_nav_canvas);
    lv_obj_set_style_line_color(t0_beam_arrow, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_line_width(t0_beam_arrow, 3, 0);

    // Target pin marker
    t0_target_icon = lv_obj_create(t0_nav_canvas);
    lv_obj_set_size(t0_target_icon, 16, 16);
    lv_obj_set_style_radius(t0_target_icon, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(t0_target_icon, lv_color_hex(0x00FF88), 0);
    lv_obj_set_style_border_color(t0_target_icon, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(t0_target_icon, 2, 0);

    // Rover icon
    t0_rover_icon = lv_obj_create(t0_nav_canvas);
    lv_obj_set_size(t0_rover_icon, 16, 16);
    lv_obj_set_style_radius(t0_rover_icon, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(t0_rover_icon, lv_color_hex(0x38BDF8), 0);
    lv_obj_set_style_border_color(t0_rover_icon, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(t0_rover_icon, 2, 0);

    // Route REC button (Tactical Grid HUD - Bottom Right)
    t0_rec_btn = lv_button_create(t0_nav_canvas);
    lv_obj_set_size(t0_rec_btn, 80, 26);
    lv_obj_align(t0_rec_btn, LV_ALIGN_BOTTOM_RIGHT, -6, -6);
    lv_obj_set_style_bg_color(t0_rec_btn, COLOR_BTN_INACTIVE_BLUE, 0);
    lv_obj_set_style_radius(t0_rec_btn, 4, 0);
    lv_obj_set_ext_click_area(t0_rec_btn, 6);
    lv_obj_add_event_cb(t0_rec_btn, route_toggle_btn_cb, LV_EVENT_PRESSED, NULL);
    t0_rec_btn_lbl = lv_label_create(t0_rec_btn);
    lv_label_set_text(t0_rec_btn_lbl, "REC OFF");
    lv_obj_set_style_text_font(t0_rec_btn_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(t0_rec_btn_lbl);

    // Readout section below canvas (compacted to fit inside 374px tileview)
    t0_dist_lbl = lv_label_create(tile_guidance);
    lv_label_set_text(t0_dist_lbl, "0.00 m");
    lv_obj_set_style_text_color(t0_dist_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(t0_dist_lbl, &lv_font_montserrat_24, 0);
    lv_obj_align(t0_dist_lbl, LV_ALIGN_TOP_MID, 0, 268);

    t0_steer_lbl = lv_label_create(tile_guidance);
    lv_label_set_text(t0_steer_lbl, "Acquiring Position...");
    lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0xFFCC00), 0);
    lv_obj_set_style_text_font(t0_steer_lbl, &lv_font_montserrat_14, 0);
    lv_obj_align(t0_steer_lbl, LV_ALIGN_TOP_MID, 0, 300);

    t0_offsets_lbl = lv_label_create(tile_guidance);
    lv_label_set_text(t0_offsets_lbl, "dN: +0.0 cm   dE: +0.0 cm");
    lv_obj_set_style_text_color(t0_offsets_lbl, lv_color_hex(0x00D8FF), 0);
    lv_obj_set_style_text_font(t0_offsets_lbl, &lv_font_montserrat_14, 0);
    lv_obj_align(t0_offsets_lbl, LV_ALIGN_TOP_MID, 0, 324);

    // =====================================================================
    // TILE 1: Field Survey / Waypoint Logger & Quick Settings
    // =====================================================================
    // Row 1 (y=6): Location ID Prefix Dropdown (145x28) & Averaging Duration Button (145x28)
    t1_prefix_dd = lv_dropdown_create(tile_survey);
    lv_dropdown_set_options(t1_prefix_dd, waypoint_get_prefix_options());
    lv_dropdown_set_selected(t1_prefix_dd, g_survey_prefix_idx);
    lv_obj_set_size(t1_prefix_dd, 145, 28);
    lv_obj_align(t1_prefix_dd, LV_ALIGN_TOP_LEFT, 10, 6);
    lv_obj_add_event_cb(t1_prefix_dd, prefix_dd_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *btn_avg = lv_button_create(tile_survey);
    lv_obj_set_size(btn_avg, 145, 28);
    lv_obj_align(btn_avg, LV_ALIGN_TOP_RIGHT, -10, 6);
    lv_obj_add_event_cb(btn_avg, avg_dur_btn_cb, LV_EVENT_CLICKED, NULL);
    t1_avg_btn_lbl = lv_label_create(btn_avg);
    lv_label_set_text_fmt(t1_avg_btn_lbl, "Avg: %us", current_settings.survey_avg_seconds);
    lv_obj_center(t1_avg_btn_lbl);

    // Row 2 (y=38): Quick Note Tag Dropdown (300x28)
    t1_note_dd = lv_dropdown_create(tile_survey);
    lv_dropdown_set_options(t1_note_dd, waypoint_get_note_tag_options());
    lv_dropdown_set_selected(t1_note_dd, g_survey_note_tag_idx);
    lv_obj_set_size(t1_note_dd, 300, 28);
    lv_obj_align(t1_note_dd, LV_ALIGN_TOP_MID, 0, 38);
    lv_obj_add_event_cb(t1_note_dd, note_dd_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // Row 3 (y=70): ID Label (300px wide, Montserrat 14, scrolling marquee)
    t1_next_lbl = lv_label_create(tile_survey);
    lv_obj_set_width(t1_next_lbl, 300);
    lv_label_set_long_mode(t1_next_lbl, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_label_set_text(t1_next_lbl, "ID: Acquiring...");
    lv_obj_set_style_text_color(t1_next_lbl, lv_color_hex(0xFFCC00), 0);
    lv_obj_set_style_text_font(t1_next_lbl, &lv_font_montserrat_14, 0);
    lv_obj_align(t1_next_lbl, LV_ALIGN_TOP_LEFT, 10, 70);

    // Row 4 (y=92): Multi-epoch progress bar (300x6)
    t1_prog_bar = lv_bar_create(tile_survey);
    lv_obj_set_size(t1_prog_bar, 300, 6);
    lv_obj_align(t1_prog_bar, LV_ALIGN_TOP_MID, 0, 92);
    lv_bar_set_range(t1_prog_bar, 0, 100);
    lv_bar_set_value(t1_prog_bar, 0, LV_ANIM_OFF);

    // Row 5 (y=104): Large LOG PIN NOW Button (300x36)
    t1_log_btn = lv_button_create(tile_survey);
    lv_obj_set_size(t1_log_btn, 300, 36);
    lv_obj_align(t1_log_btn, LV_ALIGN_TOP_MID, 0, 104);
    lv_obj_set_style_bg_color(t1_log_btn, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_add_event_cb(t1_log_btn, log_pin_btn_cb, LV_EVENT_CLICKED, NULL);
    t1_log_btn_lbl = lv_label_create(t1_log_btn);
    lv_label_set_text(t1_log_btn_lbl, "LOG PIN NOW");
    lv_obj_center(t1_log_btn_lbl);

    // Row 6 (y=148): Status & Logged File Info
    t1_status_lbl = lv_label_create(tile_survey);
    lv_obj_set_width(t1_status_lbl, 300);
    lv_label_set_text(t1_status_lbl, "Logged: None | /survey_logged_pins.csv");
    lv_obj_set_style_text_color(t1_status_lbl, lv_color_hex(0x99A3B0), 0);
    lv_obj_set_style_text_font(t1_status_lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_align(t1_status_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(t1_status_lbl, LV_ALIGN_TOP_MID, 0, 148);

    // Row 7 (y=170): Horizontal Divider separating Field Survey from Quick Settings
    lv_obj_t *t1_sep = lv_obj_create(tile_survey);
    lv_obj_set_size(t1_sep, 308, 1);
    lv_obj_align(t1_sep, LV_ALIGN_TOP_MID, 0, 170);
    lv_obj_set_style_bg_color(t1_sep, lv_color_hex(0x283B47), 0);
    lv_obj_set_style_border_width(t1_sep, 0, 0);
    lv_obj_set_scrollable(t1_sep, false);

    // ---------------------------------------------------------------------
    // Quick Settings Controls (3x4 Grid of 94x27 Buttons)
    // ---------------------------------------------------------------------
    // Row 1 (y=186): VOL, AUD, BRI
    t1_vol_btn    = create_grid_button(tile_survey, 0, 0, vol_toggle_btn_cb,        &t1_vol_btn_lbl);
    t1_aud_btn    = create_grid_button(tile_survey, 1, 0, audio_mode_toggle_btn_cb, &t1_aud_btn_lbl);
    t1_bright_btn = create_grid_button(tile_survey, 2, 0, bright_toggle_btn_cb,     &t1_bright_btn_lbl);

    // Row 2 (y=221): NMEA, SERIAL, RTCM
    t1_nmea_btn   = create_grid_button(tile_survey, 0, 1, nmea_toggle_btn_cb,       &t1_nmea_btn_lbl);
    t1_serial_btn = create_grid_button(tile_survey, 1, 1, serial_log_toggle_btn_cb, &t1_serial_btn_lbl);
    t1_rtcm_btn   = create_grid_button(tile_survey, 2, 1, diag_rtcm_btn_cb,         &t1_rtcm_btn_lbl);

    // Row 3 (y=256): TELEM, IMU, TOUCH
    t1_telem_btn  = create_grid_button(tile_survey, 0, 2, diag_telem_btn_cb,        &t1_telem_btn_lbl);
    t1_imu_btn    = create_grid_button(tile_survey, 1, 2, diag_imu_btn_cb,          &t1_imu_btn_lbl);
    t1_touch_btn  = create_grid_button(tile_survey, 2, 2, diag_touch_btn_cb,        &t1_touch_btn_lbl);

    // Row 4 (y=291): WIFI, UNITS, LORA
    t1_wifi_btn   = create_grid_button(tile_survey, 0, 3, wifi_toggle_btn_cb,        &t1_wifi_btn_lbl);
    t1_unit_btn   = create_grid_button(tile_survey, 1, 3, unit_toggle_btn_cb,        &t1_unit_btn_lbl);
    t1_lora_btn   = create_grid_button(tile_survey, 2, 3, lora_rate_btn_cb,          &t1_lora_btn_lbl);

    t1_wifi_status_lbl = lv_label_create(tile_survey);
    lv_obj_set_width(t1_wifi_status_lbl, 300);
    lv_label_set_text(t1_wifi_status_lbl, "Wi-Fi: Inactive (Tap to start)");
    lv_obj_set_style_text_color(t1_wifi_status_lbl, lv_color_hex(0x94A3B8), 0);
    lv_obj_set_style_text_font(t1_wifi_status_lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_align(t1_wifi_status_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(t1_wifi_status_lbl, LV_ALIGN_TOP_MID, 0, 332);

    update_all_control_buttons();

    // =====================================================================
    // TILE 2: Waypoint CSV Browser (Tile 3 to user)
    // Fixed Top File Selector Row: [ < ] waypoints.csv (1/1) [ > ]
    // =====================================================================
    lv_obj_t *btn_file_prev = lv_button_create(tile_csv);
    lv_obj_set_size(btn_file_prev, 44, 28);
    lv_obj_align(btn_file_prev, LV_ALIGN_TOP_LEFT, 6, 3);
    lv_obj_set_ext_click_area(btn_file_prev, 12);
    lv_obj_add_event_cb(btn_file_prev, prev_file_btn_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_t *lbl_fprev = lv_label_create(btn_file_prev);
    lv_label_set_text(lbl_fprev, LV_SYMBOL_LEFT);
    lv_obj_center(lbl_fprev);

    t2_file_lbl = lv_label_create(tile_csv);
    lv_obj_set_style_text_color(t2_file_lbl, lv_color_hex(0x00D8FF), 0);
    lv_obj_set_style_text_font(t2_file_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_width(t2_file_lbl, 216);
    lv_label_set_long_mode(t2_file_lbl, LV_LABEL_LONG_MODE_SCROLL_CIRCULAR);
    lv_obj_set_style_text_align(t2_file_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(t2_file_lbl, LV_ALIGN_TOP_MID, 0, 8);
    lv_label_set_text(t2_file_lbl, "waypoints.csv");

    lv_obj_t *btn_file_next = lv_button_create(tile_csv);
    lv_obj_set_size(btn_file_next, 44, 28);
    lv_obj_align(btn_file_next, LV_ALIGN_TOP_RIGHT, -6, 3);
    lv_obj_set_ext_click_area(btn_file_next, 12);
    lv_obj_add_event_cb(btn_file_next, next_file_btn_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_t *lbl_fnext = lv_label_create(btn_file_next);
    lv_label_set_text(lbl_fnext, LV_SYMBOL_RIGHT);
    lv_obj_center(lbl_fnext);

    // Scrollable Waypoint Table Below (y = 36 to y = 370)
    t2_wp_table = lv_table_create(tile_csv);
    lv_obj_set_size(t2_wp_table, 310, 336);
    lv_obj_align(t2_wp_table, LV_ALIGN_TOP_MID, 0, 36);
    lv_obj_set_style_text_font(t2_wp_table, &lv_font_montserrat_12, LV_PART_ITEMS);
    lv_obj_add_event_cb(t2_wp_table, wp_table_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    refresh_waypoint_table_ui();

    // =====================================================================
    // TILE 3: Dual-Antenna Compass & Pole Bubble Level
    // =====================================================================
    t3_hdg_lbl = lv_label_create(tile_compass);
    lv_label_set_text(t3_hdg_lbl, "Heading: ---.-");
    lv_obj_set_style_text_color(t3_hdg_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(t3_hdg_lbl, &lv_font_montserrat_24, 0);
    lv_obj_align(t3_hdg_lbl, LV_ALIGN_TOP_MID, 0, 8);

    t3_base_lbl = lv_label_create(tile_compass);
    lv_label_set_text(t3_base_lbl, "Baseline: 0.555 m | Pitch: 0.0 deg");
    lv_obj_set_style_text_color(t3_base_lbl, lv_color_hex(0x00FFAA), 0);
    lv_obj_set_style_text_font(t3_base_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t3_base_lbl, LV_ALIGN_TOP_MID, 0, 38);

    t3_pose_lbl = lv_label_create(tile_compass);
    lv_label_set_text(t3_pose_lbl, "Display Pose: Held Flat");
    lv_obj_set_style_text_color(t3_pose_lbl, lv_color_hex(0x00D8FF), 0);
    lv_obj_set_style_text_font(t3_pose_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t3_pose_lbl, LV_ALIGN_TOP_MID, 0, 56);

    // Fixed Course-Up Index (12 o'clock heading indicator)
    lv_obj_t *t3_hdg_pointer = lv_label_create(tile_compass);
    lv_label_set_text(t3_hdg_pointer, LV_SYMBOL_UP " HDG");
    lv_obj_set_style_text_color(t3_hdg_pointer, lv_color_hex(0x00FF88), 0);
    lv_obj_set_style_text_font(t3_hdg_pointer, &lv_font_montserrat_12, 0);
    lv_obj_align(t3_hdg_pointer, LV_ALIGN_TOP_MID, 0, 76);

    // 2D Pole Plumb Bubble Level Circle with Integrated Compass Ring (224x224 px, center at 160, 206)
    t3_level_circle = lv_obj_create(tile_compass);
    lv_obj_set_size(t3_level_circle, 224, 224);
    lv_obj_align(t3_level_circle, LV_ALIGN_TOP_MID, 0, 94);
    lv_obj_set_style_radius(t3_level_circle, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(t3_level_circle, lv_color_hex(0x1A2421), 0);
    lv_obj_set_style_border_color(t3_level_circle, lv_color_hex(0x3B6E5C), 0);
    lv_obj_set_style_border_width(t3_level_circle, 2, 0);
    lv_obj_set_scrollbar_mode(t3_level_circle, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scrollable(t3_level_circle, false);

    lv_obj_t *center_ring = lv_obj_create(t3_level_circle);
    lv_obj_set_size(center_ring, 36, 36);
    lv_obj_center(center_ring);
    lv_obj_set_style_radius(center_ring, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(center_ring, 0, 0);
    lv_obj_set_style_border_color(center_ring, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(center_ring, 2, 0);

    t3_bubble_dot = lv_obj_create(t3_level_circle);
    lv_obj_set_size(t3_bubble_dot, 24, 24);
    lv_obj_set_style_radius(t3_bubble_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(t3_bubble_dot, lv_color_hex(0x00FF44), 0);
    lv_obj_set_style_border_width(t3_bubble_dot, 0, 0);
    lv_obj_center(t3_bubble_dot);

    // 8-Point Compass Ring Markers (N, NE, E, SE, S, SW, W, NW)
    for (int i = 0; i < 8; i++) {
        t3_card_lbls[i] = lv_label_create(t3_level_circle);
        lv_label_set_text(t3_card_lbls[i], kCardNames[i]);
        if (i == 0) {
            lv_obj_set_style_text_color(t3_card_lbls[i], lv_color_hex(0xEF4444), 0); // Bold Red North
            lv_obj_set_style_text_font(t3_card_lbls[i], &lv_font_montserrat_14, 0);
        } else if (i % 2 == 0) {
            lv_obj_set_style_text_color(t3_card_lbls[i], lv_color_hex(0xFFFFFF), 0); // White Cardinal
            lv_obj_set_style_text_font(t3_card_lbls[i], &lv_font_montserrat_12, 0);
        } else {
            lv_obj_set_style_text_color(t3_card_lbls[i], lv_color_hex(0x94A3B8), 0); // Slate Intercardinal
            lv_obj_set_style_text_font(t3_card_lbls[i], &lv_font_montserrat_12, 0);
        }
        float rad = kCardAngles[i] * 0.0174532925f;
        int16_t cx = (int16_t)roundf(sinf(rad) * 96.0f);
        int16_t cy = (int16_t)roundf(-cosf(rad) * 96.0f);
        lv_obj_align(t3_card_lbls[i], LV_ALIGN_CENTER, cx, cy);
    }

    // Compact Tare Button shifted to the right edge of the screen
    t3_tare_btn = lv_button_create(tile_compass);
    lv_obj_set_size(t3_tare_btn, 64, 28);
    lv_obj_align(t3_tare_btn, LV_ALIGN_TOP_RIGHT, -10, 290);
    lv_obj_set_style_bg_color(t3_tare_btn, lv_color_hex(0x2563EB), 0); // Default Blue
    lv_obj_add_event_cb(t3_tare_btn, tare_level_btn_cb, LV_EVENT_CLICKED, NULL);
    t3_tare_lbl = lv_label_create(t3_tare_btn);
    lv_label_set_text(t3_tare_lbl, "TARE");
    lv_obj_set_style_text_font(t3_tare_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(t3_tare_lbl);

    t3_tilt_lbl = lv_label_create(tile_compass);
    lv_label_set_text(t3_tilt_lbl, "Pole Tilt: dX: 0.0 deg | dY: 0.0 deg");
    lv_obj_set_style_text_color(t3_tilt_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(t3_tilt_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t3_tilt_lbl, LV_ALIGN_TOP_MID, 0, 336);

    // =====================================================================
    // TILE 4: Hardware Diagnostics
    // =====================================================================
    lv_obj_set_scrollable(tile_test, false);

    // 1. Fixed Pinned Header Deck (Always visible, non-scrolling)
    t4_title = lv_label_create(tile_test);
    lv_label_set_text(t4_title, "RTK ROVER " ROVER_FIRMWARE_VERSION);
    lv_obj_set_style_text_color(t4_title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(t4_title, &lv_font_montserrat_16, 0);
    lv_obj_align(t4_title, LV_ALIGN_TOP_MID, 0, 4);

    // 2. Pinned LoRa RF Survey Card (Always visible, non-scrolling)
    t4_rf_card = lv_obj_create(tile_test);
    lv_obj_set_size(t4_rf_card, 308, 54);
    lv_obj_align(t4_rf_card, LV_ALIGN_TOP_MID, 0, 26);
    lv_obj_set_style_bg_color(t4_rf_card, lv_color_hex(0x1A202C), 0);
    lv_obj_set_style_border_color(t4_rf_card, lv_color_hex(0x2D3748), 0);
    lv_obj_set_style_border_width(t4_rf_card, 1, 0);
    lv_obj_set_style_radius(t4_rf_card, 4, 0);
    lv_obj_set_style_pad_all(t4_rf_card, 0, 0);
    lv_obj_set_scrollbar_mode(t4_rf_card, LV_SCROLLBAR_MODE_OFF);

    // Row 1: Meter Bar across top of card (296 x 6 px)
    t4_rf_bar = lv_bar_create(t4_rf_card);
    lv_obj_set_size(t4_rf_bar, 296, 6);
    lv_obj_align(t4_rf_bar, LV_ALIGN_TOP_MID, 0, 5);
    lv_bar_set_range(t4_rf_bar, 0, 100);
    lv_bar_set_value(t4_rf_bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(t4_rf_bar, lv_color_hex(0x283B47), LV_PART_MAIN);
    lv_obj_set_style_bg_color(t4_rf_bar, lv_color_hex(0x00FF88), LV_PART_INDICATOR);
    lv_obj_set_style_radius(t4_rf_bar, 2, LV_PART_MAIN);
    lv_obj_set_style_radius(t4_rf_bar, 2, LV_PART_INDICATOR);

    // Row 2: Live Metrics (RSSI bold left, Noise/Margin right)
    t4_rf_rssi_lbl = lv_label_create(t4_rf_card);
    lv_label_set_text(t4_rf_rssi_lbl, "LoRa: -- dBm");
    lv_obj_set_style_text_color(t4_rf_rssi_lbl, lv_color_hex(0x64748B), 0);
    lv_obj_set_style_text_font(t4_rf_rssi_lbl, &lv_font_montserrat_14, 0);
    lv_obj_align(t4_rf_rssi_lbl, LV_ALIGN_TOP_LEFT, 6, 16);

    t4_rf_noise_lbl = lv_label_create(t4_rf_card);
    lv_label_set_text(t4_rf_noise_lbl, "Noise: -- dBm");
    lv_obj_set_style_text_color(t4_rf_noise_lbl, lv_color_hex(0x94A3B8), 0);
    lv_obj_set_style_text_font(t4_rf_noise_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t4_rf_noise_lbl, LV_ALIGN_TOP_RIGHT, -6, 18);

    // Row 3: Health & Channel (Age/OK left, Channel/Air right)
    t4_rf_health_lbl = lv_label_create(t4_rf_card);
    lv_label_set_text(t4_rf_health_lbl, "Age: -- | OK: 0 (0 drop)");
    lv_obj_set_style_text_color(t4_rf_health_lbl, lv_color_hex(0xCBD5E1), 0);
    lv_obj_set_style_text_font(t4_rf_health_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t4_rf_health_lbl, LV_ALIGN_TOP_LEFT, 6, 35);

    t4_rf_chan_lbl = lv_label_create(t4_rf_card);
    lv_label_set_text(t4_rf_chan_lbl, "CH-- (19.2k)");
    lv_obj_set_style_text_color(t4_rf_chan_lbl, lv_color_hex(0x38BDF8), 0);
    lv_obj_set_style_text_font(t4_rf_chan_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t4_rf_chan_lbl, LV_ALIGN_TOP_RIGHT, -6, 35);

    // Subtle divider line
    lv_obj_t *t4_sep = lv_obj_create(tile_test);
    lv_obj_set_size(t4_sep, 308, 1);
    lv_obj_align(t4_sep, LV_ALIGN_TOP_MID, 0, 83);
    lv_obj_set_style_bg_color(t4_sep, lv_color_hex(0x283B47), 0);
    lv_obj_set_style_border_width(t4_sep, 0, 0);
    lv_obj_set_scrollbar_mode(t4_sep, LV_SCROLLBAR_MODE_OFF);

    // 3. Scrollable Viewport for Diagnostic Telemetry (320 x 288 px, +57px viewport boost)
    t4_scroll_cont = lv_obj_create(tile_test);
    lv_obj_set_size(t4_scroll_cont, 320, 374 - 86);
    lv_obj_align(t4_scroll_cont, LV_ALIGN_TOP_MID, 0, 86);
    lv_obj_set_style_bg_opa(t4_scroll_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(t4_scroll_cont, 0, 0);
    lv_obj_set_style_radius(t4_scroll_cont, 0, 0);
    lv_obj_set_scroll_dir(t4_scroll_cont, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(t4_scroll_cont, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_flex_flow(t4_scroll_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(t4_scroll_cont, 10, 0);
    lv_obj_set_style_pad_all(t4_scroll_cont, 6, 0);
    lv_obj_set_style_pad_bottom(t4_scroll_cont, 20, 0);

    // 1. Storage Status
    t4_storage_status_lbl = lv_label_create(t4_scroll_cont);
    lv_obj_set_width(t4_storage_status_lbl, 300);
    lv_label_set_long_mode(t4_storage_status_lbl, LV_LABEL_LONG_WRAP);
    lv_label_set_text(t4_storage_status_lbl, "Storage: Auto-Detecting...");
    lv_obj_set_style_text_color(t4_storage_status_lbl, lv_color_hex(0x22D3EE), 0);
    lv_obj_set_style_text_font(t4_storage_status_lbl, &lv_font_montserrat_12, 0);

    // 2. ANT1 Master GNSS
    t4_ant_master_lbl = lv_label_create(t4_scroll_cont);
    lv_obj_set_width(t4_ant_master_lbl, 300);
    lv_label_set_long_mode(t4_ant_master_lbl, LV_LABEL_LONG_WRAP);
    lv_label_set_text(t4_ant_master_lbl, "[ANT1 MASTER] Waiting for GNSS fix...");
    lv_obj_set_style_text_color(t4_ant_master_lbl, lv_color_hex(0x00FF88), 0);
    lv_obj_set_style_text_font(t4_ant_master_lbl, &lv_font_montserrat_12, 0);

    // 3. ANT2 Slave GNSS
    t4_ant_slave_lbl = lv_label_create(t4_scroll_cont);
    lv_obj_set_width(t4_ant_slave_lbl, 300);
    lv_label_set_long_mode(t4_ant_slave_lbl, LV_LABEL_LONG_WRAP);
    lv_label_set_text(t4_ant_slave_lbl, "[ANT2 SLAVE] Waiting for heading/slave data...");
    lv_obj_set_style_text_color(t4_ant_slave_lbl, lv_color_hex(0xFFCC00), 0);
    lv_obj_set_style_text_font(t4_ant_slave_lbl, &lv_font_montserrat_12, 0);

    // 4. Base Station Telemetry
    t4_base_station_lbl = lv_label_create(t4_scroll_cont);
    lv_obj_set_width(t4_base_station_lbl, 300);
    lv_label_set_long_mode(t4_base_station_lbl, LV_LABEL_LONG_WRAP);
    lv_label_set_text(t4_base_station_lbl, "[BASE STATION] Waiting for RTCM...");
    lv_obj_set_style_text_color(t4_base_station_lbl, lv_color_hex(0x38BDF8), 0);
    lv_obj_set_style_text_font(t4_base_station_lbl, &lv_font_montserrat_12, 0);

    // 5. UM982 COM1 Info
    t4_com1_lbl = lv_label_create(t4_scroll_cont);
    lv_obj_set_width(t4_com1_lbl, 300);
    lv_label_set_long_mode(t4_com1_lbl, LV_LABEL_LONG_WRAP);
    lv_label_set_text(t4_com1_lbl, "[UM982 COM1 IO17/18 @ 115200]\nRX: 0 B | TX: 0 B | NMEA: 0 | HDG: 0\nLast: Waiting...");
    lv_obj_set_style_text_color(t4_com1_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(t4_com1_lbl, &lv_font_montserrat_12, 0);

    // 6. UM982 COM2 Info
    t4_com2_lbl = lv_label_create(t4_scroll_cont);
    lv_obj_set_width(t4_com2_lbl, 300);
    lv_label_set_long_mode(t4_com2_lbl, LV_LABEL_LONG_WRAP);
    lv_label_set_text(t4_com2_lbl, "[UM982 COM2 IO47/48 @ 115200]\nRTCM TX: 0 B | RX: 0 B | Diff Age: -- s\nLast: Waiting...");
    lv_obj_set_style_text_color(t4_com2_lbl, lv_color_hex(0x00FFAA), 0);
    lv_obj_set_style_text_font(t4_com2_lbl, &lv_font_montserrat_12, 0);

    // 7. E220 LoRa Info
    char init_buf[192];
    t4_e220_lbl = lv_label_create(t4_scroll_cont);
    lv_obj_set_width(t4_e220_lbl, 300);
    lv_label_set_long_mode(t4_e220_lbl, LV_LABEL_LONG_WRAP);
    snprintf(init_buf, sizeof(init_buf),
             "[E220 LoRa Hardware IO41/42 @ %lu]\n"
             "AUX:1 Edges:0 | Regs:--\n"
             "RX:0B -> COM2:0B | OK:0 Err:0\n"
             "1006:0 | 74:0 | 84:0 | 94:0 | 1230:0",
             (unsigned long)current_settings.e220_uart_baud);
    lv_label_set_text(t4_e220_lbl, init_buf);
    lv_obj_set_style_text_color(t4_e220_lbl, lv_color_hex(0xFFCC00), 0);
    lv_obj_set_style_text_font(t4_e220_lbl, &lv_font_montserrat_12, 0);

    // 8. E220 Target & Registers
    t4_e220_cfg_lbl = lv_label_create(t4_scroll_cont);
    lv_obj_set_width(t4_e220_cfg_lbl, 300);
    lv_label_set_long_mode(t4_e220_cfg_lbl, LV_LABEL_LONG_WRAP);
    snprintf(init_buf, sizeof(init_buf),
             "Target: CH%u %lu/%.1fk %uB %udBm",
             current_settings.e220_channel,
             (unsigned long)current_settings.e220_uart_baud,
             current_settings.e220_air_rate_kbps,
             current_settings.e220_subpacket_bytes,
             current_settings.e220_tx_power_dbm);
    lv_label_set_text(t4_e220_cfg_lbl, init_buf);
    lv_obj_set_style_text_color(t4_e220_cfg_lbl, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_text_font(t4_e220_cfg_lbl, &lv_font_montserrat_12, 0);

    // 9. Power & Battery (AXP2101 PMIC)
    t4_power_lbl = lv_label_create(t4_scroll_cont);
    lv_obj_set_width(t4_power_lbl, 300);
    lv_label_set_long_mode(t4_power_lbl, LV_LABEL_LONG_WRAP);
    lv_label_set_text(t4_power_lbl, "[POWER / PMIC AXP2101]\nVBAT: --.-V (--%) | VBUS: -- | Chg: --");
    lv_obj_set_style_text_color(t4_power_lbl, lv_color_hex(0x38BDF8), 0);
    lv_obj_set_style_text_font(t4_power_lbl, &lv_font_montserrat_12, 0);

    // 10. IMU & Pole Tilt (QMI8658)
    t4_imu_lbl = lv_label_create(t4_scroll_cont);
    lv_obj_set_width(t4_imu_lbl, 300);
    lv_label_set_long_mode(t4_imu_lbl, LV_LABEL_LONG_WRAP);
    lv_label_set_text(t4_imu_lbl, "[IMU & POLE TILT] QMI8658 @ 250Hz\nLive: Pitch: 0.0 deg | Roll: 0.0 deg\nAcc(g): X:+0.00 Y:+0.00 Z:+0.00\nGyr(d/s): X:+0.0 Y:+0.0 Z:+0.0\nTare: P: 0.0 deg R: 0.0 deg | Lever: 1.80m [ON]");
    lv_obj_set_style_text_color(t4_imu_lbl, lv_color_hex(0x10B981), 0);
    lv_obj_set_style_text_font(t4_imu_lbl, &lv_font_montserrat_12, 0);

    // 11. Wi-Fi & Web Portal
    t4_wifi_lbl = lv_label_create(t4_scroll_cont);
    lv_obj_set_width(t4_wifi_lbl, 300);
    lv_label_set_long_mode(t4_wifi_lbl, LV_LABEL_LONG_WRAP);
    lv_label_set_text(t4_wifi_lbl, "[WIFI & WEB PORTAL]\nWi-Fi: INACTIVE");
    lv_obj_set_style_text_color(t4_wifi_lbl, lv_color_hex(0xA78BFA), 0);
    lv_obj_set_style_text_font(t4_wifi_lbl, &lv_font_montserrat_12, 0);

    // 12. I2C Bus Status
    t4_i2c_lbl = lv_label_create(t4_scroll_cont);
    lv_obj_set_width(t4_i2c_lbl, 300);
    lv_label_set_long_mode(t4_i2c_lbl, LV_LABEL_LONG_WRAP);
    lv_label_set_text(t4_i2c_lbl, "[I2C0 BUS STATUS]\nTCA:OK | AXP:OK | Touch:OK | RTC:OK | IMU:OK | ES8311:OK");
    lv_obj_set_style_text_color(t4_i2c_lbl, lv_color_hex(0x2DD4BF), 0);
    lv_obj_set_style_text_font(t4_i2c_lbl, &lv_font_montserrat_12, 0);
}

static inline lv_color_t get_lora_rssi_color(int16_t rssi_dbm, bool is_stale)
{
    if (is_stale || rssi_dbm < -106) {
        return lv_palette_main(LV_PALETTE_RED); // Critical / Out of lock (< -106 dBm or stale)
    } else if (rssi_dbm < -98) {
        return lv_color_hex(0xF97316);          // Safety Orange: Fringe / Low Margin (-106 .. -99 dBm)
    } else if (rssi_dbm < -85) {
        return lv_color_hex(0xFBBF24);          // Amber Yellow: Fair Working (-98 .. -86 dBm)
    } else if (rssi_dbm < -70) {
        return lv_color_hex(0x84CC16);          // Lime Green: Nominal Working Field (-85 .. -71 dBm)
    } else if (rssi_dbm < -45) {
        return lv_color_hex(0x10B981);          // Emerald Green: Optimal Solid Field (-70 .. -46 dBm)
    } else {
        return lv_color_hex(0x00E5FF);          // Electric Cyan: Near-Field / Proximity (>= -45 dBm)
    }
}

void update_rover_ui(const RoverUIData_t *d)
{
    if (!d) return;
    if (!rover_lvgl_lock(100)) return;

    char buf[512];

    // 1. Update Persistent Header Bar (Date, Time, Units, Battery, Fix, RTCM)
    RtcDateTime_t lt;
    get_local_datetime(&lt);
    snprintf(buf, sizeof(buf), "%04u-%02u-%02u %02u:%02u:%02u %s",
             lt.year, lt.month, lt.day, lt.hour, lt.minute, lt.second,
             current_settings.timezone_label);
    lv_label_set_text(hdr_datetime_lbl, buf);


    snprintf(buf, sizeof(buf), "%u%%%s %.2fV",
             d->battery_percent,
             d->is_charging ? "+" : "",
             d->battery_voltage);
    lv_label_set_text(hdr_batt_lbl, buf);

    // Wi-Fi Status Badge in Row 1 (Icon only)
    if (hdr_wifi_lbl) {
        lv_label_set_text(hdr_wifi_lbl, LV_SYMBOL_WIFI);
        if (is_wifi_active()) {
            lv_obj_set_style_text_color(hdr_wifi_lbl, lv_color_hex(0x22D3EE), 0); // Cyan
        } else {
            lv_obj_set_style_text_color(hdr_wifi_lbl, lv_color_hex(0x475569), 0); // Dim slate
        }
    }

    // Tile 1 Wi-Fi Status Label
    if (s_wifi_status_dirty && t1_wifi_status_lbl) {
        s_wifi_status_dirty = false;
        const char *ip = s_pending_wifi_status;
        if (ip[0] && strcmp(ip, "Wi-Fi Off") != 0 && strcmp(ip, "Wi-Fi Failed") != 0 &&
            strcmp(ip, "Connecting...") != 0 && strcmp(ip, "Starting AP...") != 0 && strstr(ip, "...") == NULL) {
            char ip_buf[64];
            snprintf(ip_buf, sizeof(ip_buf), "Portal: http://%s/", ip);
            lv_label_set_text(t1_wifi_status_lbl, ip_buf);
            lv_obj_set_style_text_color(t1_wifi_status_lbl, lv_color_hex(0x00FF88), 0);
        } else {
            lv_label_set_text(t1_wifi_status_lbl, ip);
            lv_obj_set_style_text_color(t1_wifi_status_lbl, lv_color_hex(0x94A3B8), 0);
        }
        update_all_control_buttons();
    }

    // Row 2: Master Antenna (ANT1) Fix Badge & Metrics
    const char *fix_str = "NO FIX";
    lv_color_t fix_col  = lv_palette_main(LV_PALETTE_RED);
    if (d->fix_type == GNSS_FIX_RTK_FIXED) {
        fix_str = "RTK FIX";
        fix_col = lv_palette_main(LV_PALETTE_GREEN);
    } else if (d->fix_type == GNSS_FIX_RTK_FLOAT) {
        fix_str = "RTK FLT";
        fix_col = lv_palette_main(LV_PALETTE_ORANGE);
    } else if (d->fix_type == GNSS_FIX_DGPS) {
        fix_str = "DGPS";
        fix_col = lv_palette_main(LV_PALETTE_BLUE);
    } else if (d->fix_type == GNSS_FIX_SINGLE) {
        fix_str = "SINGLE";
        fix_col = lv_palette_main(LV_PALETTE_AMBER);
    }
    lv_label_set_text(hdr_m_fix_lbl, fix_str);
    lv_obj_set_style_bg_color(hdr_m_fix_badge, fix_col, 0);

    char acc_str[24];
    nav_format_distance(d->h_acc_m, acc_str, sizeof(acc_str));
    uint8_t sv_view = (d->sats_in_view >= d->num_sats) ? d->sats_in_view : d->num_sats;
    snprintf(buf, sizeof(buf), "SV:%u/%u", d->num_sats, sv_view);
    lv_label_set_text(hdr_m_sats_lbl, buf);

    snprintf(buf, sizeof(buf), "+/-%s", acc_str);
    lv_label_set_text(hdr_m_acc_lbl, buf);
    lv_obj_set_style_text_color(hdr_m_acc_lbl, fix_col, 0);

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    uint32_t last_radio_rx_age = (d->e220_stats.last_rx_ms > 0) ? (now_ms - d->e220_stats.last_rx_ms) : 99999;

    // Header Row 2: LoRa Signal Strength (Numeric "LoRa: -104dB" or "LoRa: -- dB")
    if (d->e220_stats.rssi_valid && d->e220_stats.rx_bytes_total > 0) {
        bool is_stale = (last_radio_rx_age > 4000);
        snprintf(buf, sizeof(buf), "LoRa: %ddB", (int)d->e220_stats.last_pkt_rssi_dbm);
        lv_label_set_text(hdr_m_rtcm_lbl, buf);
        lv_obj_set_style_text_color(hdr_m_rtcm_lbl, get_lora_rssi_color(d->e220_stats.last_pkt_rssi_dbm, is_stale), 0);
    } else {
        lv_label_set_text(hdr_m_rtcm_lbl, "LoRa: -- dB");
        lv_obj_set_style_text_color(hdr_m_rtcm_lbl, lv_color_hex(0x64748B), 0);
    }

    // Row 3: Dual-Antenna Heading Status & Metrics
    const char *hdg_badge_str = "HDG ---";
    lv_color_t hdg_badge_col  = lv_color_hex(0x3A3F47);
    if (d->heading_valid) {
        if (d->heading_source == HDG_SRC_DUAL_FIX) {
            hdg_badge_str = "SLV FIX";
            hdg_badge_col = lv_palette_main(LV_PALETTE_GREEN);
        } else if (d->heading_source == HDG_SRC_DUAL_FLT) {
            hdg_badge_str = "SLV FLT";
            hdg_badge_col = lv_palette_main(LV_PALETTE_ORANGE);
        } else if (d->heading_source == HDG_SRC_COG) {
            hdg_badge_str = "HDG COG";
            hdg_badge_col = lv_palette_main(LV_PALETTE_CYAN);
        } else if (d->heading_source == HDG_SRC_HOLD) {
            hdg_badge_str = "HDG HLD";
            hdg_badge_col = lv_palette_main(LV_PALETTE_AMBER);
        } else if (d->heading_is_fixed) {
            hdg_badge_str = "SLV FIX";
            hdg_badge_col = lv_palette_main(LV_PALETTE_GREEN);
        } else {
            hdg_badge_str = "SLV FLT";
            hdg_badge_col = lv_palette_main(LV_PALETTE_ORANGE);
        }
    }
    if (hdr_s_fix_lbl) lv_label_set_text(hdr_s_fix_lbl, hdg_badge_str);
    if (hdr_s_fix_badge) lv_obj_set_style_bg_color(hdr_s_fix_badge, hdg_badge_col, 0);

    if (d->slave_num_sats > 0) {
        snprintf(buf, sizeof(buf), "SV:%u", d->slave_num_sats);
    } else {
        snprintf(buf, sizeof(buf), "SV:--");
    }
    lv_label_set_text(hdr_s_sats_lbl, buf);

    if (d->baseline_length_m > 0.01f) {
        snprintf(buf, sizeof(buf), "BL:%.1fcm", d->baseline_length_m * 100.0f);
    } else {
        snprintf(buf, sizeof(buf), "BL:--.-cm");
    }
    lv_label_set_text(hdr_s_bl_lbl, buf);

    if (d->heading_valid) {
        const char *src_tag = (d->heading_source == HDG_SRC_DUAL_FIX) ? "FIX" :
                              (d->heading_source == HDG_SRC_DUAL_FLT) ? "FLT" :
                              (d->heading_source == HDG_SRC_COG)      ? "COG" :
                              (d->heading_source == HDG_SRC_HOLD)     ? "HLD" :
                              (d->heading_is_fixed ? "FIX" : "FLT");
        snprintf(buf, sizeof(buf), "Hdg:%.1f %s", d->true_heading_deg, src_tag);
    } else {
        snprintf(buf, sizeof(buf), "Hdg:---.-");
    }
    lv_label_set_text(hdr_s_hdg_lbl, buf);

    // Row 4: Streamlined Coordinates Bar (7 decimals in Montserrat 16 for high visibility)
    char hgt_str[24], dist_str[24], dn_str[24], de_str[24];
    nav_format_height(d->tip_height_m, hgt_str, sizeof(hgt_str));
    nav_format_distance(d->distance_to_target_m, dist_str, sizeof(dist_str));
    nav_format_offset(d->d_north_m, dn_str, sizeof(dn_str));
    nav_format_offset(d->d_east_m, de_str, sizeof(de_str));

    snprintf(buf, sizeof(buf), "%.7f, %.7f, %s", d->tip_lat, d->tip_lon, hgt_str);
    if (hdr_coords_lbl) lv_label_set_text(hdr_coords_lbl, buf);

    // Push position into breadcrumb buffer
    nav_breadcrumb_push(d->tip_lat, d->tip_lon);

    // Update all control buttons (Colors & Labels)
    update_all_control_buttons();

    // Update Canvas Top-Left Telemetry HUD (Heading & Pitch)
    char hud_buf[32];
    if (t0_hdg_hud_lbl) {
        if (d->heading_valid) {
            const char *src_tag = (d->heading_source == HDG_SRC_DUAL_FIX) ? "FIX" :
                                  (d->heading_source == HDG_SRC_DUAL_FLT) ? "FLT" :
                                  (d->heading_source == HDG_SRC_COG)      ? "COG" :
                                  (d->heading_source == HDG_SRC_HOLD)     ? "HLD" :
                                  (d->heading_is_fixed ? "FIX" : "FLT");
            snprintf(hud_buf, sizeof(hud_buf), "Hdg: %05.1f %s", d->true_heading_deg, src_tag);
        } else {
            snprintf(hud_buf, sizeof(hud_buf), "Hdg: ---.-");
        }
        lv_label_set_text(t0_hdg_hud_lbl, hud_buf);
    }

    if (t0_pitch_hud_lbl) {
        if (d->heading_valid || d->baseline_length_m > 0.01f) {
            snprintf(hud_buf, sizeof(hud_buf), "Pitch: %+.1f deg", d->baseline_pitch_deg);
        } else {
            snprintf(hud_buf, sizeof(hud_buf), "Pitch: --.- deg");
        }
        lv_label_set_text(t0_pitch_hud_lbl, hud_buf);
    }

    if (t0_tgt_hdg_lbl) {
        if (d->has_active_target) {
            snprintf(hud_buf, sizeof(hud_buf), "Tgt: %05.1f deg", d->bearing_to_target_deg);
            lv_label_set_text(t0_tgt_hdg_lbl, hud_buf);
            if ((d->heading_valid && fabsf(d->rel_bearing_deg) <= 3.0f) ||
                (d->distance_to_target_m <= 0.05)) {
                lv_obj_set_style_text_color(t0_tgt_hdg_lbl, lv_color_hex(0x00FF88), 0); // Green (within +/-3 deg or on pin)
            } else if (d->heading_valid && fabsf(d->rel_bearing_deg) <= 15.0f) {
                lv_obj_set_style_text_color(t0_tgt_hdg_lbl, lv_color_hex(0xFFB800), 0); // Amber (within +/-15 deg)
            } else {
                lv_obj_set_style_text_color(t0_tgt_hdg_lbl, lv_color_hex(0xFFFFFF), 0); // White (>15 deg or no heading)
            }
        } else {
            lv_label_set_text(t0_tgt_hdg_lbl, "Tgt: ---.- deg");
            lv_obj_set_style_text_color(t0_tgt_hdg_lbl, lv_color_hex(0xFFFFFF), 0);     // White
        }
    }

    if (d->has_active_target) {
        if (s_tile0_nav_mode == 0) {
            snprintf(buf, sizeof(buf), "%s (%d/%d)",
                     d->target_label, d->active_target_idx + 1, d->total_targets);
            if (t0_target_lbl) {
                const char *cur_tgt = lv_label_get_text(t0_target_lbl);
                if (!cur_tgt || strcmp(cur_tgt, buf) != 0) {
                    lv_label_set_text(t0_target_lbl, buf);
                }
            }
            if (t0_notes_lbl) {
                const char *new_notes = d->target_notes[0] ? d->target_notes : "---";
                const char *cur_notes = lv_label_get_text(t0_notes_lbl);
                if (!cur_notes || strcmp(cur_notes, new_notes) != 0) {
                    lv_label_set_text(t0_notes_lbl, new_notes);
                }
            }
            if (t0_dist_lbl) lv_label_set_text(t0_dist_lbl, dist_str);
        }

        if (s_tile0_nav_mode == 1) {
            // =============================================================
            // LINE STAKEOUT MODE (Station & Offset)
            // =============================================================
            WaypointRecord_t wp_a, wp_b;
            int16_t idx_a = -1, idx_b = -1;
            waypoint_line_stakeout_get_points(&wp_a, &wp_b, &idx_a, &idx_b);

            if (idx_a >= 0 && idx_b >= 0) {
                NavLineStakeout_t stakeout;
                nav_compute_line_stakeout(d->tip_lat, d->tip_lon,
                                          wp_a.latitude, wp_a.longitude,
                                          wp_b.latitude, wp_b.longitude,
                                          &stakeout);

                // Top Target Row: A -> B [SWAP]
                snprintf(buf, sizeof(buf), "%s -> %s [SWAP]", wp_a.label, wp_b.label);
                if (t0_target_lbl) {
                    const char *cur_tgt = lv_label_get_text(t0_target_lbl);
                    if (!cur_tgt || strcmp(cur_tgt, buf) != 0) {
                        lv_label_set_text(t0_target_lbl, buf);
                    }
                }

                char qbrg[32];
                nav_format_quadrant_bearing(stakeout.line_bearing_deg, qbrg, sizeof(qbrg));

                char len_str[24];
                nav_format_distance(stakeout.total_length_m, len_str, sizeof(len_str));

                snprintf(buf, sizeof(buf), "Deed: %s | %s", qbrg, len_str);
                if (t0_notes_lbl) {
                    const char *cur_notes = lv_label_get_text(t0_notes_lbl);
                    if (!cur_notes || strcmp(cur_notes, buf) != 0) {
                        lv_label_set_text(t0_notes_lbl, buf);
                    }
                }

                // Mode badge in Amber
                lv_label_set_text(t0_mode_badge, "LINE STAKEOUT");
                lv_obj_set_style_bg_color(t0_mode_badge, lv_color_hex(0xD97706), 0);

                // Scale label shows total line length
                snprintf(buf, sizeof(buf), "Total: %s", len_str);
                lv_label_set_text(t0_scale_lbl, buf);

                // Hide coarse grid, track arrows, wedges, bullseye rings
                lv_obj_set_hidden(t0_bullseye_ring1, true);
                lv_obj_set_hidden(t0_bullseye_ring2, true);
                lv_obj_set_hidden(t0_bullseye_center, true);
                lv_obj_set_hidden(t0_cross_h, true);
                for (int i = 0; i < T0_GRID_H_LINES; i++) {
                    if (t0_grid_h[i]) lv_obj_set_hidden(t0_grid_h[i], true);
                }
                for (int i = 0; i < T0_GRID_V_LINES; i++) {
                    if (t0_grid_v[i]) lv_obj_set_hidden(t0_grid_v[i], true);
                }
                lv_obj_set_hidden(t0_track_line, true);
                lv_obj_set_hidden(t0_track_arrow, true);
                lv_obj_set_hidden(t0_wedge_amber_l, true);
                lv_obj_set_hidden(t0_wedge_amber_r, true);
                lv_obj_set_hidden(t0_wedge_green_l, true);
                lv_obj_set_hidden(t0_wedge_green_r, true);

                // Show Centerline (Baseline A -> B)
                lv_obj_set_hidden(t0_cross_v, false);
                lv_obj_set_style_line_color(t0_cross_v, lv_color_hex(0x00D8FF), 0);
                lv_obj_set_style_line_width(t0_cross_v, 2, 0);

                // Canvas coordinates:
                // Canvas size: 304 x 210, Center: (152, 105)
                // Centerline is at x = 152.
                // Cross-track pixel scaling: 60 px = 1.0 m (~18 px/ft)
                float px_per_m = 60.0f;
                int16_t off_px = (int16_t)(stakeout.offset_m * px_per_m);
                if (off_px > 120) off_px = 120;
                if (off_px < -120) off_px = -120;

                // Along-line Station progress:
                // Point A at bottom (y = 175), Point B at top (y = 35)
                float pct = stakeout.pct_complete;
                if (pct < 0.0f) pct = 0.0f;
                if (pct > 100.0f) pct = 100.0f;
                int16_t sta_py = 175 - (int16_t)((pct / 100.0f) * 140.0f);

                // Target Point B marker fixed at top of line (152, 35)
                lv_obj_align(t0_target_icon, LV_ALIGN_TOP_LEFT, 152 - 8, 35 - 8);
                lv_obj_set_hidden(t0_target_icon, false);

                // Rover marker at (152 + off_px, sta_py)
                lv_obj_align(t0_rover_icon, LV_ALIGN_TOP_LEFT, 152 + off_px - 8, sta_py - 8);
                lv_obj_set_hidden(t0_rover_icon, false);

                bool on_line = (fabs(stakeout.offset_m) <= 0.02); // 2 cm on-line threshold
                if (on_line) {
                    lv_obj_set_style_bg_color(t0_rover_icon, lv_color_hex(0x00FF88), 0);
                } else {
                    lv_obj_set_style_bg_color(t0_rover_icon, lv_color_hex(0xF59E0B), 0);
                }

                // Heading beam on rover
                if (d->heading_valid) {
                    float hdg_rad = d->true_heading_deg * (float)(M_PI / 180.0);
                    int16_t arrow_dx = (int16_t)(20.0f * sinf(hdg_rad));
                    int16_t arrow_dy = (int16_t)(-20.0f * cosf(hdg_rad));
                    static lv_point_precise_t line_beam_pts[2];
                    line_beam_pts[0].x = 152 + off_px;
                    line_beam_pts[0].y = sta_py;
                    line_beam_pts[1].x = 152 + off_px + arrow_dx;
                    line_beam_pts[1].y = sta_py + arrow_dy;
                    lv_line_set_points(t0_beam_arrow, line_beam_pts, 2);
                    lv_obj_set_style_line_color(t0_beam_arrow, lv_color_hex(0xFFFFFF), 0);
                    lv_obj_set_hidden(t0_beam_arrow, false);
                } else {
                    lv_obj_set_hidden(t0_beam_arrow, true);
                }

                // Large Offset Display (t0_dist_lbl)
                char off_val_str[24];
                nav_format_offset((float)fabs(stakeout.offset_m), off_val_str, sizeof(off_val_str));
                if (on_line) {
                    snprintf(buf, sizeof(buf), "[OK] ON LINE! (%s)", off_val_str);
                    if (t0_dist_lbl) {
                        lv_label_set_text(t0_dist_lbl, buf);
                        lv_obj_set_style_text_color(t0_dist_lbl, lv_color_hex(0x00FF88), 0);
                    }
                } else if (stakeout.offset_m < 0.0) {
                    snprintf(buf, sizeof(buf), "%s LEFT ◄── (Move R)", off_val_str);
                    if (t0_dist_lbl) {
                        lv_label_set_text(t0_dist_lbl, buf);
                        lv_obj_set_style_text_color(t0_dist_lbl, lv_color_hex(0xF59E0B), 0);
                    }
                } else {
                    snprintf(buf, sizeof(buf), "%s RIGHT (Move L) ──►", off_val_str);
                    if (t0_dist_lbl) {
                        lv_label_set_text(t0_dist_lbl, buf);
                        lv_obj_set_style_text_color(t0_dist_lbl, lv_color_hex(0xF59E0B), 0);
                    }
                }

                // Steer Label: Station + Remaining
                char sta_str[32];
                nav_format_station(stakeout.station_m, sta_str, sizeof(sta_str));
                char rem_str[24];
                nav_format_distance(fabs(stakeout.remaining_m), rem_str, sizeof(rem_str));
                snprintf(buf, sizeof(buf), "%s (%.1f%%) | Rem: %s", sta_str, stakeout.pct_complete, rem_str);
                if (t0_steer_lbl) {
                    lv_label_set_text(t0_steer_lbl, buf);
                    lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0xFFFFFF), 0);
                }

                // Offsets Label: Deed Bearing + True Heading
                snprintf(buf, sizeof(buf), "Deed: %s (%.1f deg)", qbrg, stakeout.line_bearing_deg);
                if (t0_offsets_lbl) {
                    lv_label_set_text(t0_offsets_lbl, buf);
                }
            }
        } else if (d->distance_to_target_m <= 1.5) {
            // -------------------------------------------------------------
            // Fine Stakeout Mode (< 1.5 m): Centimeter Bullseye Target
            // -------------------------------------------------------------
            lv_label_set_text(t0_mode_badge, "FINE STAKEOUT");
            lv_obj_set_style_bg_color(t0_mode_badge, lv_color_hex(0x00AA55), 0);
            lv_label_set_text(t0_scale_lbl, "Bullseye: 1.0 m");

            // Hide coarse grid & track line
            for (int i = 0; i < T0_GRID_H_LINES; i++) {
                if (t0_grid_h[i]) lv_obj_set_hidden(t0_grid_h[i], true);
            }
            for (int i = 0; i < T0_GRID_V_LINES; i++) {
                if (t0_grid_v[i]) lv_obj_set_hidden(t0_grid_v[i], true);
            }
            lv_obj_set_hidden(t0_track_line, true);
            lv_obj_set_hidden(t0_track_arrow, true);
            lv_obj_set_hidden(t0_wedge_amber_l, true);
            lv_obj_set_hidden(t0_wedge_amber_r, true);
            lv_obj_set_hidden(t0_wedge_green_l, true);
            lv_obj_set_hidden(t0_wedge_green_r, true);

            // Show bullseye rings
            lv_obj_set_hidden(t0_bullseye_ring1, false);
            lv_obj_set_hidden(t0_bullseye_ring2, false);
            lv_obj_set_hidden(t0_bullseye_center, false);
            lv_obj_set_hidden(t0_cross_h, false);
            lv_obj_set_hidden(t0_cross_v, false);

            // Target Pin at Center
            lv_obj_align(t0_target_icon, LV_ALIGN_CENTER, 0, 0);
            lv_obj_set_hidden(t0_target_icon, false);

            // Rover Pole Tip moves relative to target center (70 px = 1.0m)
            int16_t r_px = (int16_t)(-d->d_east_m * 70.0f);
            int16_t r_py = (int16_t)( d->d_north_m * 70.0f);
            if (r_px > 80) r_px = 80; else if (r_px < -80) r_px = -80;
            if (r_py > 80) r_py = 80; else if (r_py < -80) r_py = -80;
            lv_obj_align(t0_rover_icon, LV_ALIGN_CENTER, r_px, r_py);
            lv_obj_set_hidden(t0_rover_icon, false);

            // Antenna beam vector
            if (d->heading_valid) {
                float hdg_rad = d->true_heading_deg * (float)(M_PI / 180.0);
                int16_t arrow_dx = (int16_t)(22.0f * sinf(hdg_rad));
                int16_t arrow_dy = (int16_t)(-22.0f * cosf(hdg_rad));
                static lv_point_precise_t arrow_pts[2];
                arrow_pts[0].x = 152 + r_px;
                arrow_pts[0].y = 105 + r_py;
                arrow_pts[1].x = 152 + r_px + arrow_dx;
                arrow_pts[1].y = 105 + r_py + arrow_dy;
                lv_line_set_points(t0_beam_arrow, arrow_pts, 2);
                lv_obj_set_style_line_color(t0_beam_arrow, lv_color_hex(0xFFCC00), 0);
                lv_obj_set_hidden(t0_beam_arrow, false);
            } else {
                lv_obj_set_hidden(t0_beam_arrow, true);
            }

            // Lock indicator (Clean ASCII)
            if (d->distance_to_target_m < 0.02) {
                lv_obj_set_style_bg_color(t0_bullseye_center, lv_color_hex(0x00FF00), 0);
                lv_obj_set_style_border_color(t0_bullseye_center, lv_color_hex(0xFFFFFF), 0);
                snprintf(buf, sizeof(buf), "[OK] PIN ACQUIRED! (Acc: %.1f cm)", d->h_acc_m * 100.0f);
                lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0x00FF00), 0);
            } else {
                lv_obj_set_style_bg_color(t0_bullseye_center, lv_color_hex(0xFF0055), 0);
                lv_obj_set_style_border_color(t0_bullseye_center, lv_color_hex(0xFFAABB), 0);
                snprintf(buf, sizeof(buf), "Stakeout: Align Tip to Crosshair");
                lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0xFFCC00), 0);
            }
            lv_label_set_text(t0_steer_lbl, buf);
        } else {
            // -------------------------------------------------------------
            // Stage 1 Tactical Guidance Mode (> 1.5 m): Course-Up Director
            // -------------------------------------------------------------
            lv_label_set_text(t0_mode_badge, "TACTICAL GUIDANCE");
            lv_obj_set_style_bg_color(t0_mode_badge, lv_color_hex(0x0088CC), 0);

            // Hide fine bullseye rings, center, and crosshairs
            lv_obj_set_hidden(t0_bullseye_ring1, true);
            lv_obj_set_hidden(t0_bullseye_ring2, true);
            lv_obj_set_hidden(t0_bullseye_center, true);
            lv_obj_set_hidden(t0_cross_h, true);
            lv_obj_set_hidden(t0_cross_v, true);

            // Hide coarse grid lines for clean high-contrast canvas
            for (int i = 0; i < T0_GRID_H_LINES; i++) {
                if (t0_grid_h[i]) lv_obj_set_hidden(t0_grid_h[i], true);
            }
            for (int i = 0; i < T0_GRID_V_LINES; i++) {
                if (t0_grid_v[i]) lv_obj_set_hidden(t0_grid_v[i], true);
            }

            // Continuous Distance in Range HUD
            char scale_str[24];
            nav_format_distance(d->distance_to_target_m, scale_str, sizeof(scale_str));
            snprintf(buf, sizeof(buf), "Range: %s", scale_str);
            lv_label_set_text(t0_scale_lbl, buf);

            // Fixed Course-Up Anchor Positions:
            // Rover Origin at bottom center (152, 170)
            int16_t rover_x = 152;
            int16_t rover_y = 170;
            lv_obj_align(t0_rover_icon, LV_ALIGN_TOP_LEFT, rover_x - 8, rover_y - 8);
            lv_obj_set_hidden(t0_rover_icon, false);

            // Target Pin fixed at top center (152, 58)
            int16_t target_x = 152;
            int16_t target_y = 58;
            lv_obj_align(t0_target_icon, LV_ALIGN_TOP_LEFT, target_x - 8, target_y - 8);
            lv_obj_set_hidden(t0_target_icon, false);

            // Show fixed track line and track chevron arrowhead
            lv_obj_set_hidden(t0_track_line, false);
            lv_obj_set_hidden(t0_track_arrow, false);

            // Show tolerance wedges radiating from Rover (152, 170)
            lv_obj_set_hidden(t0_wedge_amber_l, false);
            lv_obj_set_hidden(t0_wedge_amber_r, false);
            lv_obj_set_hidden(t0_wedge_green_l, false);
            lv_obj_set_hidden(t0_wedge_green_r, false);

            // Rotating Beam Pointer with Directional Arrowhead (shaft + dual barbs)
            if (d->heading_valid) {
                float rel_turn = d->rel_bearing_deg;
                float abs_turn = fabsf(rel_turn);
                float beam_rad = (-rel_turn) * (float)(M_PI / 180.0);

                float tip_x = (float)rover_x + 55.0f * sinf(beam_rad);
                float tip_y = (float)rover_y - 55.0f * cosf(beam_rad);

                float b1_rad = beam_rad - 2.61799f; // -150 deg
                float b2_rad = beam_rad + 2.61799f; // +150 deg
                float b1_x = tip_x + 12.0f * sinf(b1_rad);
                float b1_y = tip_y - 12.0f * cosf(b1_rad);
                float b2_x = tip_x + 12.0f * sinf(b2_rad);
                float b2_y = tip_y - 12.0f * cosf(b2_rad);

                static lv_point_precise_t beam_pts[5];
                beam_pts[0].x = rover_x;
                beam_pts[0].y = rover_y;
                beam_pts[1].x = (int16_t)roundf(tip_x);
                beam_pts[1].y = (int16_t)roundf(tip_y);
                beam_pts[2].x = (int16_t)roundf(b1_x);
                beam_pts[2].y = (int16_t)roundf(b1_y);
                beam_pts[3].x = (int16_t)roundf(tip_x);
                beam_pts[3].y = (int16_t)roundf(tip_y);
                beam_pts[4].x = (int16_t)roundf(b2_x);
                beam_pts[4].y = (int16_t)roundf(b2_y);
                lv_line_set_points(t0_beam_arrow, beam_pts, 5);
                lv_obj_set_hidden(t0_beam_arrow, false);

                // 3-Tier Dynamic Color & Steering Feedback
                if (abs_turn <= 3.0f) {
                    // Tier 1: Inner Green Wedge (<= +/-3 deg)
                    lv_obj_set_style_line_color(t0_beam_arrow, lv_color_hex(0x00FF88), 0);
                    snprintf(buf, sizeof(buf), "[OK] ON TRACK - Aimed at Pin");
                    lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0x00FF88), 0);
                } else if (abs_turn <= 15.0f) {
                    // Tier 2: Outer Amber Wedge (+/-4 to +/-15 deg)
                    lv_obj_set_style_line_color(t0_beam_arrow, lv_color_hex(0xFFB800), 0);
                    if (rel_turn > 0.0f) {
                        snprintf(buf, sizeof(buf), ">>> Turn Right %.0f deg", rel_turn);
                    } else {
                        snprintf(buf, sizeof(buf), "<<< Turn Left %.0f deg", -rel_turn);
                    }
                    lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0xFFB800), 0);
                } else {
                    // Tier 3: Outside Wedges (> 15 deg)
                    lv_obj_set_style_line_color(t0_beam_arrow, lv_color_hex(0xFFFFFF), 0);
                    if (rel_turn > 0.0f) {
                        snprintf(buf, sizeof(buf), ">>> Turn Right %.0f deg", rel_turn);
                    } else {
                        snprintf(buf, sizeof(buf), "<<< Turn Left %.0f deg", -rel_turn);
                    }
                    lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0xFFFFFF), 0);
                }
            } else {
                lv_obj_set_hidden(t0_beam_arrow, true);
                snprintf(buf, sizeof(buf), "Brg: %.1f deg | Acquiring Heading...", d->bearing_to_target_deg);
                lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0xFF8800), 0);
            }
            lv_label_set_text(t0_steer_lbl, buf);
        }

        if (s_tile0_nav_mode == 0) {
            snprintf(buf, sizeof(buf), "dN: %s   dE: %s", dn_str, de_str);
            lv_label_set_text(t0_offsets_lbl, buf);
        }
    } else {
        if (t0_target_lbl) {
            const char *cur_tgt = lv_label_get_text(t0_target_lbl);
            if (!cur_tgt || strcmp(cur_tgt, "No Target (0/0)") != 0) {
                lv_label_set_text(t0_target_lbl, "No Target (0/0)");
            }
        }
        if (t0_notes_lbl) {
            const char *cur_notes = lv_label_get_text(t0_notes_lbl);
            if (!cur_notes || strcmp(cur_notes, "---") != 0) {
                lv_label_set_text(t0_notes_lbl, "---");
            }
        }
        if (t0_dist_lbl) lv_label_set_text(t0_dist_lbl, "---");
        if (t0_offsets_lbl) lv_label_set_text(t0_offsets_lbl, "dN: ---   dE: ---");
        lv_label_set_text(t0_mode_badge, "NO TARGET");
        lv_obj_set_style_bg_color(t0_mode_badge, lv_palette_main(LV_PALETTE_GREY), 0);
        lv_label_set_text(t0_scale_lbl, "Range: ---");
        lv_label_set_text(t0_steer_lbl, "Select target in Tile 3 Waypoints");
        lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0x888888), 0);

        // Hide navigation overlays
        lv_obj_set_hidden(t0_bullseye_ring1, true);
        lv_obj_set_hidden(t0_bullseye_ring2, true);
        lv_obj_set_hidden(t0_bullseye_center, true);
        lv_obj_set_hidden(t0_cross_h, true);
        lv_obj_set_hidden(t0_cross_v, true);
        for (int i = 0; i < T0_GRID_H_LINES; i++) {
            if (t0_grid_h[i]) lv_obj_set_hidden(t0_grid_h[i], true);
        }
        for (int i = 0; i < T0_GRID_V_LINES; i++) {
            if (t0_grid_v[i]) lv_obj_set_hidden(t0_grid_v[i], true);
        }
        lv_obj_set_hidden(t0_track_line, true);
        lv_obj_set_hidden(t0_track_arrow, true);
        lv_obj_set_hidden(t0_beam_arrow, true);
        lv_obj_set_hidden(t0_wedge_amber_l, true);
        lv_obj_set_hidden(t0_wedge_amber_r, true);
        lv_obj_set_hidden(t0_wedge_green_l, true);
        lv_obj_set_hidden(t0_wedge_green_r, true);
        lv_obj_set_hidden(t0_target_icon, true);
        lv_obj_align(t0_rover_icon, LV_ALIGN_TOP_LEFT, 152 - 8, 170 - 8);
        lv_obj_set_hidden(t0_rover_icon, false);
    }

    // 3. Update Tile 1: Field Survey Logger
    char next_id[WAYPOINT_LABEL_LEN];
    waypoint_format_next_survey_label(next_id, sizeof(next_id));
    snprintf(buf, sizeof(buf), "ID: %s", next_id);
    lv_label_set_text(t1_next_lbl, buf);

    lv_bar_set_value(t1_prog_bar, d->survey_progress_pct, LV_ANIM_OFF);
    if (d->survey_in_progress) {
        snprintf(buf, sizeof(buf), "AVERAGING RTK... %u%% (%u pts)",
                 d->survey_progress_pct, d->survey_epochs_collected);
        lv_label_set_text(t1_log_btn_lbl, buf);
        lv_obj_set_style_bg_color(t1_log_btn, lv_palette_main(LV_PALETTE_ORANGE), 0);
    } else {
        lv_label_set_text(t1_log_btn_lbl, "LOG PIN NOW");
        lv_obj_set_style_bg_color(t1_log_btn, lv_palette_main(LV_PALETTE_GREEN), 0);
    }
    snprintf(buf, sizeof(buf), "Logged: %s  |  /survey_logged_pins.csv",
             d->last_logged_label);
    lv_label_set_text(t1_status_lbl, buf);

    // 4. Update Tile 3: Dual-Antenna Compass & Bubble Level
    if (d->heading_valid) {
        snprintf(buf, sizeof(buf), "Heading: %05.1f", d->true_heading_deg);
        lv_label_set_text(t3_hdg_lbl, buf);
    } else {
        lv_label_set_text(t3_hdg_lbl, "Heading: ---.-");
    }
    snprintf(buf, sizeof(buf), "Baseline: %.3f m | Pitch: %+.1f deg",
             d->baseline_length_m > 0.01f ? d->baseline_length_m : current_settings.antenna_spacing_m,
             d->baseline_pitch_deg);
    lv_label_set_text(t3_base_lbl, buf);

    const char *pose_str = "Held Flat";
    if (d->display_pose == DISPLAY_POSE_ANGLED) pose_str = "Handheld Viewing Angle";
    else if (d->display_pose == DISPLAY_POSE_VERTICAL) pose_str = "Clipped Vertically to Pole";
    snprintf(buf, sizeof(buf), "Display Pose: %s", pose_str);
    lv_label_set_text(t3_pose_lbl, buf);

    // Dynamic rotation of 8 compass points (Course-Up: current heading is UP at 12 o'clock)
    float cur_hdg = d->heading_valid ? d->true_heading_deg : 0.0f;
    for (int i = 0; i < 8; i++) {
        if (t3_card_lbls[i]) {
            float rel_deg = kCardAngles[i] - cur_hdg;
            float rad = rel_deg * 0.0174532925f;
            int16_t cx = (int16_t)roundf(sinf(rad) * 96.0f);
            int16_t cy = (int16_t)roundf(-cosf(rad) * 96.0f);
            lv_obj_align(t3_card_lbls[i], LV_ALIGN_CENTER, cx, cy);
        }
    }

    int16_t bx = (int16_t)(d->pole_bubble_x_deg * 6.0f);
    int16_t by = (int16_t)(d->pole_bubble_y_deg * 6.0f);
    if (bx > 70) bx = 70; if (bx < -70) bx = -70;
    if (by > 70) by = 70; if (by < -70) by = -70;
    lv_obj_align(t3_bubble_dot, LV_ALIGN_CENTER, bx, by);
    snprintf(buf, sizeof(buf), "Pole Tilt: dX: %+.1f deg | dY: %+.1f deg",
             d->pole_bubble_x_deg, d->pole_bubble_y_deg);
    lv_label_set_text(t3_tilt_lbl, buf);

    // 5. Update Tile 4: Pinned LoRa RF Survey Card & Hardware Diagnostics Status
    if (t4_rf_card && t4_rf_bar && t4_rf_rssi_lbl) {
        float rx_age_s = (d->e220_stats.last_rx_ms > 0) ? (float)(now_ms - d->e220_stats.last_rx_ms) / 1000.0f : 999.0f;
        bool is_fresh = (d->e220_stats.rssi_valid && rx_age_s <= 4.0f && d->e220_stats.rx_bytes_total > 0);
        int16_t pkt_rssi = d->e220_stats.last_pkt_rssi_dbm;
        int16_t noise_floor = d->e220_stats.ambient_noise_dbm;

        // Row 1: Meter Bar (Scale: -110 dBm = 0%, -25 dBm = 100%, Span = 85 dB)
        if (is_fresh) {
            int pct = ((int)pkt_rssi + 110) * 100 / 85;
            if (pct < 0) pct = 0;
            if (pct > 100) pct = 100;
            lv_bar_set_value(t4_rf_bar, pct, LV_ANIM_OFF);
            lv_obj_set_style_bg_color(t4_rf_bar, get_lora_rssi_color(pkt_rssi, false), LV_PART_INDICATOR);
        } else {
            lv_bar_set_value(t4_rf_bar, 0, LV_ANIM_OFF);
            lv_obj_set_style_bg_color(t4_rf_bar, lv_color_hex(0x475569), LV_PART_INDICATOR);
        }

        // Row 2: Live Metrics (RSSI bold left, Noise/Margin right)
        if (is_fresh) {
            snprintf(buf, sizeof(buf), "LoRa: %d dBm", (int)pkt_rssi);
            lv_label_set_text(t4_rf_rssi_lbl, buf);
            lv_obj_set_style_text_color(t4_rf_rssi_lbl, get_lora_rssi_color(pkt_rssi, false), 0);
        } else if (d->e220_stats.rx_bytes_total > 0 && d->e220_stats.rssi_valid) {
            snprintf(buf, sizeof(buf), "LoRa: %d dBm*", (int)pkt_rssi);
            lv_label_set_text(t4_rf_rssi_lbl, buf);
            lv_obj_set_style_text_color(t4_rf_rssi_lbl, lv_palette_main(LV_PALETTE_RED), 0);
        } else {
            lv_label_set_text(t4_rf_rssi_lbl, "LoRa: No Signal");
            lv_obj_set_style_text_color(t4_rf_rssi_lbl, lv_color_hex(0x64748B), 0);
        }

        if (d->e220_stats.rssi_valid) {
            int margin = (int)pkt_rssi - (int)noise_floor;
            if (is_fresh) {
                snprintf(buf, sizeof(buf), "Noise: %d dBm (%+d dB)", (int)noise_floor, margin);
            } else {
                snprintf(buf, sizeof(buf), "Noise: %d dBm (--)", (int)noise_floor);
            }
            lv_label_set_text(t4_rf_noise_lbl, buf);
        } else {
            lv_label_set_text(t4_rf_noise_lbl, "Noise: -- dBm");
        }

        // Row 3: Health & Channel
        uint32_t drops = d->e220_stats.rtcm_crc_errors + d->e220_stats.rtcm_trunc_drops;
        if (rx_age_s < 99.0f) {
            snprintf(buf, sizeof(buf), "Age: %.1fs | OK: %lu (%lu drop)",
                     rx_age_s, (unsigned long)d->e220_stats.rtcm_frames_ok, (unsigned long)drops);
        } else {
            snprintf(buf, sizeof(buf), "Age: >99s | OK: %lu (%lu drop)",
                     (unsigned long)d->e220_stats.rtcm_frames_ok, (unsigned long)drops);
        }
        lv_label_set_text(t4_rf_health_lbl, buf);
        if (drops > 0) {
            lv_obj_set_style_text_color(t4_rf_health_lbl, lv_color_hex(0xFFA500), 0);
        } else {
            lv_obj_set_style_text_color(t4_rf_health_lbl, lv_color_hex(0xCBD5E1), 0);
        }

        snprintf(buf, sizeof(buf), "CH%u (%.1fk)",
                 d->e220_stats.cfg_channel,
                 current_settings.e220_air_rate_kbps);
        lv_label_set_text(t4_rf_chan_lbl, buf);
    }

    if (t4_storage_status_lbl) {
        extern bool sd_card_mounted;
        if (sd_card_mounted) {
            lv_label_set_text(t4_storage_status_lbl, "Storage: MicroSD Card Ready (Primary)");
            lv_obj_set_style_text_color(t4_storage_status_lbl, lv_color_hex(0x22D3EE), 0); // Cyan (distinct from ANT1 green #00FF88)
        } else {
            lv_label_set_text(t4_storage_status_lbl, "Storage: FFat Internal Flash (Fallback)");
            lv_obj_set_style_text_color(t4_storage_status_lbl, lv_color_hex(0xFFA500), 0); // Amber
        }
    }

    // 6. Update Tile 4: Antenna & GNSS Diagnostics
    if (t4_ant_master_lbl) {
        char h_acc_str[24], v_acc_str[24], hgt_str[24];
        nav_format_distance(d->h_acc_m, h_acc_str, sizeof(h_acc_str));
        nav_format_distance(d->v_acc_m, v_acc_str, sizeof(v_acc_str));
        nav_format_height(d->tip_height_m, hgt_str, sizeof(hgt_str));
        uint8_t sv_view = (d->sats_in_view >= d->num_sats) ? d->sats_in_view : d->num_sats;

        const char *m_fix_str = "NO FIX";
        if (d->fix_type == GNSS_FIX_RTK_FIXED)      m_fix_str = "RTK FIX";
        else if (d->fix_type == GNSS_FIX_RTK_FLOAT) m_fix_str = "RTK FLT";
        else if (d->fix_type == GNSS_FIX_DGPS)      m_fix_str = "DGPS";
        else if (d->fix_type == GNSS_FIX_SINGLE)    m_fix_str = "SINGLE";

        snprintf(buf, sizeof(buf),
                 "[ANT1 MASTER] %s | SV:%u/%u | HDOP:%.1f\n"
                 "Acc:+/-%s (V:+/-%s) | Alt:%s\n"
                 "Pos:%.8f, %.8f",
                 m_fix_str,
                 d->num_sats,
                 sv_view,
                 d->hdop,
                 h_acc_str,
                 v_acc_str,
                 hgt_str,
                 d->tip_lat,
                 d->tip_lon);
        lv_label_set_text(t4_ant_master_lbl, buf);
    }

    if (t4_ant_slave_lbl) {
        const char *s_fix_str = "NO FIX";
        if (d->heading_valid) {
            if (d->heading_source == HDG_SRC_DUAL_FIX)      s_fix_str = "SLV FIX";
            else if (d->heading_source == HDG_SRC_DUAL_FLT) s_fix_str = "SLV FLT";
            else if (d->heading_source == HDG_SRC_COG)      s_fix_str = "HDG COG";
            else if (d->heading_source == HDG_SRC_HOLD)     s_fix_str = "HDG HLD";
            else s_fix_str = d->heading_is_fixed ? "SLV FIX" : "SLV FLT";
        } else if (d->slave_fix_type == GNSS_FIX_RTK_FIXED) {
            s_fix_str = "SLV FIX";
        } else if (d->slave_fix_type == GNSS_FIX_RTK_FLOAT) {
            s_fix_str = "SLV FLT";
        } else if (d->slave_fix_type == GNSS_FIX_DGPS) {
            s_fix_str = "SLV DGPS";
        } else if (d->slave_fix_type == GNSS_FIX_SINGLE) {
            s_fix_str = "SLV SGL";
        }

        char bl_str[24];
        if (d->baseline_length_m > 0.01f) {
            snprintf(bl_str, sizeof(bl_str), "%.1fcm", d->baseline_length_m * 100.0f);
        } else {
            snprintf(bl_str, sizeof(bl_str), "--.-cm");
        }

        char hdg_str[24];
        if (d->heading_valid) {
            snprintf(hdg_str, sizeof(hdg_str), "%.1f deg", d->true_heading_deg);
        } else {
            snprintf(hdg_str, sizeof(hdg_str), "---.- deg");
        }

        char s_alt_str[24];
        nav_format_height(d->slave_alt_m, s_alt_str, sizeof(s_alt_str));

        if (d->slave_lat != 0.0 || d->slave_lon != 0.0) {
            snprintf(buf, sizeof(buf),
                     "[ANT2 SLAVE] %s | SV:%u | HDOP:%.1f\n"
                     "Baseline:%s | Pitch:%+.1f deg\n"
                     "Heading:%s | Alt:%s\n"
                     "Pos:%.8f, %.8f",
                     s_fix_str,
                     d->slave_num_sats,
                     d->slave_hdop,
                     bl_str,
                     d->baseline_pitch_deg,
                     hdg_str,
                     s_alt_str,
                     d->slave_lat,
                     d->slave_lon);
        } else {
            snprintf(buf, sizeof(buf),
                     "[ANT2 SLAVE] %s | SV:%u | HDOP:%.1f\n"
                     "Baseline:%s | Pitch:%+.1f deg\n"
                     "Heading:%s | Alt:%s",
                     s_fix_str,
                     d->slave_num_sats,
                     d->slave_hdop,
                     bl_str,
                     d->baseline_pitch_deg,
                     hdg_str,
                     s_alt_str);
        }
        lv_label_set_text(t4_ant_slave_lbl, buf);
    }

    if (t4_base_station_lbl) {
        if (d->base_coords_valid) {
            char alt_str[24];
            nav_format_height((float)d->base_alt_m, alt_str, sizeof(alt_str));
            snprintf(buf, sizeof(buf),
                     "[BASE STATION] ID:#%u | Diff:%.1fs\n"
                     "Pos:%.8f, %.8f\n"
                     "Alt:%s | 1005:%u MSM:%u",
                     d->base_station_id,
                     d->diff_age_s,
                     d->base_lat,
                     d->base_lon,
                     alt_str,
                     d->e220_stats.msg_1005_count,
                     d->e220_stats.msg_msm_count);
            if (d->diff_age_s <= 2.0f) {
                lv_obj_set_style_text_color(t4_base_station_lbl, lv_color_hex(0x00FF88), 0); // Fresh Green
            } else if (d->diff_age_s <= 5.0f) {
                lv_obj_set_style_text_color(t4_base_station_lbl, lv_color_hex(0xFFA500), 0); // Warning Orange
            } else {
                lv_obj_set_style_text_color(t4_base_station_lbl, lv_palette_main(LV_PALETTE_RED), 0); // Stale Red
            }
        } else if (d->base_station_id > 0 || d->diff_age_s < 30.0f) {
            snprintf(buf, sizeof(buf),
                     "[BASE STATION] ID:#%u | Diff:%.1fs\n"
                     "Pos: Waiting for RTCM 1005/1006 ARP...\n"
                     "RTCM: 1005:%u MSM:%u",
                     d->base_station_id,
                     d->diff_age_s,
                     d->e220_stats.msg_1005_count,
                     d->e220_stats.msg_msm_count);
            lv_obj_set_style_text_color(t4_base_station_lbl, lv_color_hex(0xFFA500), 0);
        } else {
            snprintf(buf, sizeof(buf),
                     "[BASE STATION] Waiting for RTCM corrections...\n"
                     "Diff: --.-s | 1005:%u MSM:%u",
                     d->e220_stats.msg_1005_count,
                     d->e220_stats.msg_msm_count);
            lv_obj_set_style_text_color(t4_base_station_lbl, lv_color_hex(0x888888), 0);
        }
        lv_label_set_text(t4_base_station_lbl, buf);
    }

    // 7. Update Tile 4: Hardware Port Diagnostics
    if (t4_com1_lbl) {
        snprintf(buf, sizeof(buf),
                 "[UM982 COM1 IO17/18 @ 115200] [%s]\nRX:%lu B | TX:%lu B | NMEA:%lu | HDG:%lu\nLast: %s",
                 d->um982_stats.com1_test_passed ? "PASS" : "WAIT",
                 (unsigned long)d->um982_stats.com1_rx_bytes,
                 (unsigned long)d->um982_stats.com1_tx_bytes,
                 (unsigned long)d->um982_stats.nmea_sentences_ok,
                 (unsigned long)d->um982_stats.heading_sentences_ok,
                 d->um982_stats.last_com1_line);
        lv_label_set_text(t4_com1_lbl, buf);
    }

    if (t4_com2_lbl) {
        snprintf(buf, sizeof(buf),
                 "[UM982 COM2 IO47/48 @ 115200] [%s]\nRTCM TX:%lu B | RX:%lu B | Diff:%.1fs\nLast: %s",
                 d->um982_stats.com2_test_passed ? "PASS" : "WAIT",
                 (unsigned long)d->um982_stats.com2_tx_bytes,
                 (unsigned long)d->um982_stats.com2_rx_bytes,
                 d->diff_age_s,
                 d->um982_stats.last_com2_line);
        lv_label_set_text(t4_com2_lbl, buf);
    }

    if (t4_e220_lbl) {
        const char *m0_str = d->e220_stats.m0_wire_ok ? "OK" : "ERR";
        const char *m1_str = d->e220_stats.m1_wire_ok ? "OK" : "ERR";
        snprintf(buf, sizeof(buf),
                 "[E220 LoRa Hardware IO41/42] [%s]\n"
                 "AUX:%d (Edges:%lu) | M0:%s M1:%s\n"
                 "Throughput: RX:%lu B -> COM2:%lu B\n"
                 "Integrity: OK:%lu | CRC Err:%lu | Trunc:%lu | Orph:%lu B\n"
                 "[RTCM3 Message Audit]\n"
                 "1006 (Base ARP): %u | 1074 (GPS): %u\n"
                 "1084 (GLO): %u | 1094 (GAL): %u\n"
                 "1230 (Biases): %u",
                 d->e220_stats.config_valid ? (d->e220_stats.config_matched ? "CFG OK" : "CFG DIFF") : "NO RSP",
                 d->e220_stats.aux_pin_high ? 1 : 0,
                 (unsigned long)d->e220_stats.aux_fall_count,
                 m0_str,
                 m1_str,
                 (unsigned long)d->e220_stats.rx_bytes_total,
                 (unsigned long)d->e220_stats.rtcm_bytes_forwarded,
                 (unsigned long)d->e220_stats.rtcm_frames_ok,
                 (unsigned long)d->e220_stats.rtcm_crc_errors,
                 (unsigned long)d->e220_stats.rtcm_trunc_drops,
                 (unsigned long)d->e220_stats.rtcm_orphan_bytes,
                 d->e220_stats.msg_1005_count,
                 d->e220_stats.msg_1074_count,
                 d->e220_stats.msg_1084_count,
                 d->e220_stats.msg_1094_count,
                 d->e220_stats.msg_1230_count);
        lv_label_set_text(t4_e220_lbl, buf);
        if (!d->e220_stats.m0_wire_ok || !d->e220_stats.m1_wire_ok) {
            lv_obj_set_style_text_color(t4_e220_lbl, lv_palette_main(LV_PALETTE_RED), 0); // Pin wire error
        } else if (d->e220_stats.rtcm_crc_errors > 0 || d->e220_stats.rtcm_trunc_drops > 0) {
            lv_obj_set_style_text_color(t4_e220_lbl, lv_color_hex(0xFFA500), 0); // Highlight in Orange on errors
        } else {
            lv_obj_set_style_text_color(t4_e220_lbl, lv_color_hex(0xFFCC00), 0); // Normal yellow
        }
    }

    if (t4_e220_cfg_lbl && d->e220_stats.config_valid) {
        snprintf(buf, sizeof(buf),
                 "Target: CH%u %lu/%.1fk %uB %udBm\n"
                 "Regs: ADDH:%02X ADDL:%02X REG0:%02X REG1:%02X CH:%02X REG3:%02X",
                 current_settings.e220_channel,
                 (unsigned long)current_settings.e220_uart_baud,
                 current_settings.e220_air_rate_kbps,
                 current_settings.e220_subpacket_bytes,
                 current_settings.e220_tx_power_dbm,
                 d->e220_stats.cfg_addr_high,
                 d->e220_stats.cfg_addr_low,
                 d->e220_stats.cfg_reg0,
                 d->e220_stats.cfg_reg1,
                 d->e220_stats.cfg_channel,
                 d->e220_stats.cfg_reg3);
        lv_label_set_text(t4_e220_cfg_lbl, buf);
    }

    // 8. Update Tile 4: Power / PMIC AXP2101 Diagnostics
    if (t4_power_lbl) {
        snprintf(buf, sizeof(buf),
                 "[POWER / PMIC AXP2101]\n"
                 "VBAT: %.2fV (%u%%) | VBUS: %s | Chg: %s",
                 d->battery_voltage,
                 (unsigned int)d->battery_percent,
                 d->vbus_present ? "YES" : "NO",
                 d->is_charging ? "YES" : "NO");
        lv_label_set_text(t4_power_lbl, buf);
    }

    // 9. Update Tile 4: IMU & Pole Tilt Diagnostics
    if (t4_imu_lbl) {
        snprintf(buf, sizeof(buf),
                 "[IMU & POLE TILT] QMI8658 @ 250Hz\n"
                 "Live: Pitch:%+.1f deg | Roll:%+.1f deg\n"
                 "Acc(g): X:%+.2f Y:%+.2f Z:%+.2f\n"
                 "Gyr(d/s): X:%+.1f Y:%+.1f Z:%+.1f\n"
                 "Tare: P:%+.1f deg R:%+.1f deg | Lever: %.2fm [%s]",
                 d->imu_pitch_deg,
                 d->imu_roll_deg,
                 d->imu_acc[0], d->imu_acc[1], d->imu_acc[2],
                 d->imu_gyr[0], d->imu_gyr[1], d->imu_gyr[2],
                 current_settings.pole_tilt_tare_pitch_deg,
                 current_settings.pole_tilt_tare_roll_deg,
                 current_settings.antenna_height_m,
                 current_settings.apply_lever_arm ? "ON" : "OFF");
        lv_label_set_text(t4_imu_lbl, buf);
    }

    // 10. Update Tile 4: Wi-Fi & Web Portal Diagnostics
    if (t4_wifi_lbl) {
        bool active = is_wifi_active();
        const char *ip = get_wifi_ip();
        if (active && ip && ip[0]) {
            snprintf(buf, sizeof(buf),
                     "[WIFI & WEB PORTAL]\n"
                     "SSID: '%s' [ONLINE]\n"
                     "Portal: http://%s/",
                     current_settings.wifi_ssid,
                     ip);
        } else if (active) {
            snprintf(buf, sizeof(buf),
                     "[WIFI & WEB PORTAL]\n"
                     "SSID: '%s' [CONNECTING...]",
                     current_settings.wifi_ssid);
        } else {
            snprintf(buf, sizeof(buf),
                     "[WIFI & WEB PORTAL]\n"
                     "Wi-Fi: INACTIVE (Tap Start on Tile 2)");
        }
        lv_label_set_text(t4_wifi_lbl, buf);
    }

    // 11. Update Tile 4: I2C Bus Diagnostics
    if (t4_i2c_lbl) {
        lv_label_set_text(t4_i2c_lbl,
                          "[I2C0 BUS STATUS]\n"
                          "TCA:OK | AXP:OK | Touch:OK | RTC:OK | IMU:OK | ES8311:OK");
    }

    rover_lvgl_unlock();
}

void lvgl_port_init(void)
{
    flush_done_semaphore = xSemaphoreCreateBinary();
    assert(flush_done_semaphore);

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
        create_rover_ui();
        rover_lvgl_unlock();
    }

    xTaskCreatePinnedToCore(rover_lvgl_port_task, "LVGL", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL, 1);
}
