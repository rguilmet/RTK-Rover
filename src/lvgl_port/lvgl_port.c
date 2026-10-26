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
#include "../web_server/web_server.h"

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
    "2/5: Survey Log",
    "3/5: CSV Waypoints",
    "4/5: Compass & Level",
    "5/5: Settings & Diag"
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
static lv_obj_t *t0_cross_h         = NULL;
static lv_obj_t *t0_cross_v         = NULL;
static lv_obj_t *t0_bullseye_ring1  = NULL;
static lv_obj_t *t0_bullseye_ring2  = NULL;
static lv_obj_t *t0_bullseye_center = NULL;
static lv_obj_t *t0_track_line      = NULL;
static lv_obj_t *t0_beam_arrow      = NULL;
static lv_obj_t *t0_target_icon     = NULL;
static lv_obj_t *t0_rover_icon      = NULL;
#define T0_BREADCRUMB_DOTS 16
static lv_obj_t *t0_bc_dots[T0_BREADCRUMB_DOTS] = {0};
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
// Tile 1: Field Survey / Waypoint Logger (BOOT Button Short-Press Active)
// =========================================================================
static lv_obj_t *t1_live_pos_lbl  = NULL;
static lv_obj_t *t1_next_lbl      = NULL;
static lv_obj_t *t1_match_sw      = NULL;
static lv_obj_t *t1_prefix_dd     = NULL;
static lv_obj_t *t1_note_dd       = NULL;
static lv_obj_t *t1_avg_btn_lbl   = NULL;
static lv_obj_t *t1_log_btn       = NULL;
static lv_obj_t *t1_log_btn_lbl   = NULL;
static lv_obj_t *t1_prog_bar      = NULL;
static lv_obj_t *t1_status_lbl    = NULL;

// =========================================================================
// Tile 2: Waypoint CSV Browser
// =========================================================================
static lv_obj_t *t2_file_lbl      = NULL;
static lv_obj_t *t2_wp_table      = NULL;

// =========================================================================
// Tile 3: Dual-Antenna Compass & Pole Bubble Level
// =========================================================================
static lv_obj_t *t3_hdg_lbl       = NULL;
static lv_obj_t *t3_base_lbl      = NULL;
static lv_obj_t *t3_pose_lbl      = NULL;
static lv_obj_t *t3_level_circle  = NULL;
static lv_obj_t *t3_bubble_dot    = NULL;
static lv_obj_t *t3_tilt_lbl      = NULL;
static lv_obj_t *t3_tare_btn      = NULL;
static lv_obj_t *t3_tare_lbl      = NULL;

// =========================================================================
// Tile 4: Settings & System Info / Hardware Diagnostics
// =========================================================================
static lv_obj_t *t4_title               = NULL;
static lv_obj_t *t4_unit_btn            = NULL;
static lv_obj_t *t4_unit_btn_lbl        = NULL;
static lv_obj_t *t4_wifi_btn            = NULL;
static lv_obj_t *t4_wifi_btn_lbl        = NULL;
static lv_obj_t *t4_wifi_status_lbl     = NULL;
static lv_obj_t *t4_nmea_btn            = NULL;
static lv_obj_t *t4_nmea_btn_lbl        = NULL;
static lv_obj_t *t4_route_btn           = NULL;
static lv_obj_t *t4_route_btn_lbl       = NULL;
static lv_obj_t *t4_storage_status_lbl  = NULL;
static lv_obj_t *t4_com1_lbl            = NULL;
static lv_obj_t *t4_com2_lbl            = NULL;
static lv_obj_t *t4_e220_lbl            = NULL;
static lv_obj_t *t4_e220_cfg_lbl        = NULL;

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

    // Command byte 7 = 0x0E (14 bytes): matches Waveshare ESP32-S3-Touch-LCD-3.5B bsp_touch.c
    uint8_t read_touchpad_cmd[11] = {0xb5, 0xab, 0xa5, 0x5a, 0x00, 0x00, 0x00, 0x0e, 0x00, 0x00, 0x00};
    uint8_t buff[14] = {0};

    if (i2c_master_write_read_dev(disp_touch_dev_handle, read_touchpad_cmd, 11, buff, 14) == ESP_OK) {
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
            return;
        }
    }

    // Always preserve last_x / last_y on release so LVGL 9 recognizes swipes and clicks
    indevData->point.x = last_x;
    indevData->point.y = last_y;
    indevData->state   = LV_INDEV_STATE_RELEASED;
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
    uint8_t next_idx = (active_tile_col == 0) ? 4 : (active_tile_col - 1);
    switch_to_tile_index(next_idx, LV_ANIM_ON);
}

static void nav_next_screen_btn_cb(lv_event_t *e)
{
    (void)e;
    uint8_t next_idx = (active_tile_col + 1) % 5;
    switch_to_tile_index(next_idx, LV_ANIM_ON);
}

static void prev_wp_btn_cb(lv_event_t *e)
{
    (void)e;
    waypoint_select_prev();
}

static void next_wp_btn_cb(lv_event_t *e)
{
    (void)e;
    waypoint_select_next();
}

static void update_all_control_buttons(void)
{
    // 1. Units Button on Tile 4 & Header Badge on Header
    if (t4_unit_btn && t4_unit_btn_lbl) {
        if (!current_settings.use_imperial_units) {
            lv_label_set_text(t4_unit_btn_lbl, "Units: SI (m/cm)");
            lv_obj_set_style_bg_color(t4_unit_btn, COLOR_UNIT_SI_TEAL, 0);
            if (hdr_units_lbl) {
                lv_label_set_text(hdr_units_lbl, "SI");
                lv_obj_set_style_text_color(hdr_units_lbl, COLOR_UNIT_SI_TEAL, 0);
            }
        } else {
            lv_label_set_text(t4_unit_btn_lbl, "Units: IMP (ft/in)");
            lv_obj_set_style_bg_color(t4_unit_btn, COLOR_UNIT_IMP_AMBER, 0);
            if (hdr_units_lbl) {
                lv_label_set_text(hdr_units_lbl, "IMP");
                lv_obj_set_style_text_color(hdr_units_lbl, COLOR_UNIT_IMP_AMBER, 0);
            }
        }
    }

    // 2. Wi-Fi Button on Tile 4
    if (t4_wifi_btn && t4_wifi_btn_lbl) {
        if (is_wifi_active()) {
            lv_label_set_text(t4_wifi_btn_lbl, "Wi-Fi: Stop");
            lv_obj_set_style_bg_color(t4_wifi_btn, COLOR_BTN_ACTIVE_GREEN, 0);
        } else {
            lv_label_set_text(t4_wifi_btn_lbl, "Wi-Fi: Start");
            lv_obj_set_style_bg_color(t4_wifi_btn, COLOR_BTN_INACTIVE_BLUE, 0);
        }
    }

    // 3. NMEA Logging Button on Tile 4
    if (t4_nmea_btn && t4_nmea_btn_lbl) {
        if (nmea_logger_is_active()) {
            lv_label_set_text(t4_nmea_btn_lbl, "NMEA Log: ON");
            lv_obj_set_style_bg_color(t4_nmea_btn, COLOR_BTN_ACTIVE_GREEN, 0);
        } else {
            lv_label_set_text(t4_nmea_btn_lbl, "NMEA Log: OFF");
            lv_obj_set_style_bg_color(t4_nmea_btn, COLOR_BTN_INACTIVE_BLUE, 0);
        }
    }

    // 4. Route Logging Button on Tile 4
    if (t4_route_btn && t4_route_btn_lbl) {
        if (route_logger_is_active()) {
            lv_label_set_text(t4_route_btn_lbl, "Route: ON");
            lv_obj_set_style_bg_color(t4_route_btn, COLOR_BTN_ACTIVE_GREEN, 0);
        } else {
            lv_label_set_text(t4_route_btn_lbl, "Route: OFF");
            lv_obj_set_style_bg_color(t4_route_btn, COLOR_BTN_INACTIVE_BLUE, 0);
        }
    }

    // 5. Route REC Button on Tile 0
    if (t0_rec_btn && t0_rec_btn_lbl) {
        if (route_logger_is_active()) {
            lv_label_set_text(t0_rec_btn_lbl, "REC ON");
            lv_obj_set_style_bg_color(t0_rec_btn, COLOR_BTN_ACTIVE_GREEN, 0);
        } else {
            lv_label_set_text(t0_rec_btn_lbl, "REC OFF");
            lv_obj_set_style_bg_color(t0_rec_btn, COLOR_BTN_INACTIVE_BLUE, 0);
        }
    }

    // 6. Tare / Reset Tare Button on Tile 3
    if (t3_tare_btn && t3_tare_lbl) {
        bool has_tare = (fabsf(current_settings.pole_tilt_tare_pitch_deg) > 0.05f ||
                         fabsf(current_settings.pole_tilt_tare_roll_deg)  > 0.05f);
        if (has_tare) {
            lv_label_set_text(t3_tare_lbl, "Reset Tare (Clear Zero)");
            lv_obj_set_style_bg_color(t3_tare_btn, lv_color_hex(0xD97706), 0); // Amber/Orange
        } else {
            lv_label_set_text(t3_tare_lbl, "Tare Pole Bracket Angle (Zero Level)");
            lv_obj_set_style_bg_color(t3_tare_btn, lv_color_hex(0x2F3B4C), 0); // Blue/Slate
        }
    }
}

static void unit_toggle_btn_cb(lv_event_t *e)
{
    (void)e;
    current_settings.use_imperial_units = !current_settings.use_imperial_units;
    mark_settings_dirty();
    update_all_control_buttons();
}

static void route_toggle_btn_cb(lv_event_t *e)
{
    (void)e;
    if (route_logger_is_active()) {
        route_logger_stop();
    } else {
        route_logger_start();
    }
    update_all_control_buttons();
}

static void nmea_toggle_btn_cb(lv_event_t *e)
{
    (void)e;
    if (nmea_logger_is_active()) {
        nmea_logger_stop();
    } else {
        nmea_logger_start();
    }
    update_all_control_buttons();
}

static void wifi_toggle_btn_cb(lv_event_t *e)
{
    (void)e;
    if (is_wifi_active()) {
        stop_wifi();
    } else {
        start_wifi();
    }
    update_all_control_buttons();
}

void update_wifi_ip_label(const char* ip)
{
    if (t4_wifi_status_lbl) {
        if (ip && ip[0] && strcmp(ip, "Wi-Fi Off") != 0 && strcmp(ip, "Wi-Fi Failed") != 0 && strcmp(ip, "Connecting...") != 0 && strcmp(ip, "Starting AP...") != 0) {
            char buf[64];
            snprintf(buf, sizeof(buf), "Portal: http://%s/", ip);
            lv_label_set_text(t4_wifi_status_lbl, buf);
            lv_obj_set_style_text_color(t4_wifi_status_lbl, lv_color_hex(0x00FF88), 0);
        } else {
            lv_label_set_text(t4_wifi_status_lbl, ip ? ip : "Wi-Fi Inactive");
            lv_obj_set_style_text_color(t4_wifi_status_lbl, lv_color_hex(0x94A3B8), 0);
        }
    }
    update_all_control_buttons();
}

static void match_target_sw_cb(lv_event_t *e)
{
    lv_obj_t *sw = (lv_obj_t *)lv_event_get_target(e);
    g_survey_target_match_mode = lv_obj_has_state(sw, LV_STATE_CHECKED);
}

static void prefix_dd_cb(lv_event_t *e)
{
    lv_obj_t *dd = (lv_obj_t *)lv_event_get_target(e);
    g_survey_prefix_idx = (uint8_t)lv_dropdown_get_selected(dd);
}

static void note_dd_cb(lv_event_t *e)
{
    lv_obj_t *dd = (lv_obj_t *)lv_event_get_target(e);
    g_survey_note_tag_idx = (uint8_t)lv_dropdown_get_selected(dd);
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
}

static void log_pin_btn_cb(lv_event_t *e)
{
    (void)e;
    trigger_survey_waypoint_log();
}

static void wp_table_event_cb(lv_event_t *e)
{
    lv_obj_t *table = (lv_obj_t *)lv_event_get_target(e);
    uint32_t row = 0, col = 0;
    lv_table_get_selected_cell(table, &row, &col);
    if (row > 0 && (int16_t)(row - 1) < g_waypoint_count) {
        waypoint_select_index((int16_t)(row - 1));
        switch_to_tile_index(0, LV_ANIM_ON);
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
    }
    update_all_control_buttons();
}

static void test_com1_ver_btn_cb(lv_event_t *e)
{
    (void)e;
    send_um982_com1_command("VERSIONA");
}

static void test_com1_cfg_btn_cb(lv_event_t *e)
{
    (void)e;
    send_um982_com1_command("CONFIG");
}

static void test_com2_pulse_btn_cb(lv_event_t *e)
{
    (void)e;
    send_um982_com2_test_pulse();
}

static void test_e220_read_btn_cb(lv_event_t *e)
{
    (void)e;
    request_e220_config_read();
}

static void test_e220_write_btn_cb(lv_event_t *e)
{
    (void)e;
    request_e220_config_write();
}

void refresh_waypoint_table_ui(void)
{
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
    lv_obj_align(hdr_units_lbl, LV_ALIGN_TOP_RIGHT, -120, 2);

    hdr_wifi_lbl = lv_label_create(hdr);
    lv_label_set_text(hdr_wifi_lbl, LV_SYMBOL_WIFI " OFF");
    lv_obj_set_style_text_color(hdr_wifi_lbl, lv_color_hex(0x64748B), 0);
    lv_obj_set_style_text_font(hdr_wifi_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(hdr_wifi_lbl, LV_ALIGN_TOP_RIGHT, -74, 2);

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
    lv_label_set_text(hdr_m_rtcm_lbl, "RTCM: --");
    lv_obj_set_style_text_color(hdr_m_rtcm_lbl, lv_color_hex(0xFFAA00), 0);
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
    lv_label_set_text(hdr_s_fix_lbl, "HDG --");
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

    // Row 4 (y = 56): Streamlined Coordinates Bar
    hdr_coords_lbl = lv_label_create(hdr);
    lv_label_set_text(hdr_coords_lbl, "0.00000000, 0.00000000, 0.000 m");
    lv_obj_set_style_text_color(hdr_coords_lbl, lv_color_hex(0x00FFAA), 0);
    lv_obj_set_style_text_font(hdr_coords_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(hdr_coords_lbl, LV_ALIGN_TOP_MID, 0, 56);

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
    lv_obj_set_size(btn_scr_prev, 64, 26);
    lv_obj_align(btn_scr_prev, LV_ALIGN_LEFT_MID, 4, 0);
    lv_obj_set_style_bg_color(btn_scr_prev, lv_color_hex(0x2F3B4C), 0);
    lv_obj_add_event_cb(btn_scr_prev, nav_prev_screen_btn_cb, LV_EVENT_CLICKED, NULL);
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
    lv_obj_set_size(btn_scr_next, 64, 26);
    lv_obj_align(btn_scr_next, LV_ALIGN_RIGHT_MID, -4, 0);
    lv_obj_set_style_bg_color(btn_scr_next, lv_color_hex(0x2F3B4C), 0);
    lv_obj_add_event_cb(btn_scr_next, nav_next_screen_btn_cb, LV_EVENT_CLICKED, NULL);
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
    lv_obj_add_event_cb(main_tv, tv_changed_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    tile_guidance = lv_tileview_add_tile(main_tv, 0, 0, LV_DIR_RIGHT);
    tile_survey   = lv_tileview_add_tile(main_tv, 1, 0, (lv_dir_t)(LV_DIR_LEFT | LV_DIR_RIGHT));
    tile_csv      = lv_tileview_add_tile(main_tv, 2, 0, (lv_dir_t)(LV_DIR_LEFT | LV_DIR_RIGHT));
    tile_compass  = lv_tileview_add_tile(main_tv, 3, 0, (lv_dir_t)(LV_DIR_LEFT | LV_DIR_RIGHT));
    tile_test     = lv_tileview_add_tile(main_tv, 4, 0, LV_DIR_LEFT);

    // =====================================================================
    // TILE 0: Pin Seeker & Guidance (Hybrid Navigation Canvas)
    // =====================================================================
    // Target selector row (compacted to y = 4 since coordinates are now in persistent header)
    lv_obj_t *btn_prev = lv_button_create(tile_guidance);
    lv_obj_set_size(btn_prev, 38, 26);
    lv_obj_align(btn_prev, LV_ALIGN_TOP_LEFT, 6, 4);
    lv_obj_add_event_cb(btn_prev, prev_wp_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl_prev = lv_label_create(btn_prev);
    lv_label_set_text(lbl_prev, LV_SYMBOL_LEFT);
    lv_obj_center(lbl_prev);

    t0_target_lbl = lv_label_create(tile_guidance);
    lv_label_set_text(t0_target_lbl, "PIN_NW_01 (1/4)");
    lv_obj_set_style_text_color(t0_target_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(t0_target_lbl, &lv_font_montserrat_16, 0);
    lv_obj_align(t0_target_lbl, LV_ALIGN_TOP_MID, 0, 6);

    lv_obj_t *btn_next = lv_button_create(tile_guidance);
    lv_obj_set_size(btn_next, 38, 26);
    lv_obj_align(btn_next, LV_ALIGN_TOP_RIGHT, -6, 4);
    lv_obj_add_event_cb(btn_next, next_wp_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl_next = lv_label_create(btn_next);
    lv_label_set_text(lbl_next, LV_SYMBOL_RIGHT);
    lv_obj_center(lbl_next);

    // Target notes row
    t0_notes_lbl = lv_label_create(tile_guidance);
    lv_obj_set_width(t0_notes_lbl, 304);
    lv_label_set_text(t0_notes_lbl, "NW corner iron pipe");
    lv_obj_set_style_text_color(t0_notes_lbl, lv_color_hex(0x99A3B0), 0);
    lv_obj_set_style_text_font(t0_notes_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t0_notes_lbl, LV_ALIGN_TOP_LEFT, 10, 34);

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
    lv_label_set_text(t0_mode_badge, "TACTICAL GRID");
    lv_obj_set_style_text_font(t0_mode_badge, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(t0_mode_badge, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_color(t0_mode_badge, lv_color_hex(0x00D8FF), 0);
    lv_obj_set_style_bg_opa(t0_mode_badge, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_hor(t0_mode_badge, 6, 0);
    lv_obj_set_style_pad_ver(t0_mode_badge, 2, 0);
    lv_obj_set_style_radius(t0_mode_badge, 4, 0);
    lv_obj_align(t0_mode_badge, LV_ALIGN_TOP_LEFT, 6, 6);

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

    // Breadcrumb dots (16 pre-allocated)
    for (int i = 0; i < T0_BREADCRUMB_DOTS; i++) {
        t0_bc_dots[i] = lv_obj_create(t0_nav_canvas);
        lv_obj_set_size(t0_bc_dots[i], 4, 4);
        lv_obj_set_style_radius(t0_bc_dots[i], LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(t0_bc_dots[i], lv_color_hex(0x00E5FF), 0);
        lv_obj_set_style_border_width(t0_bc_dots[i], 0, 0);
        lv_obj_set_hidden(t0_bc_dots[i], true);
    }

    // Line of sight (track line)
    t0_track_line = lv_line_create(t0_nav_canvas);
    lv_obj_set_style_line_color(t0_track_line, lv_color_hex(0x00D8FF), 0);
    lv_obj_set_style_line_width(t0_track_line, 2, 0);

    // Rover antenna beam vector
    t0_beam_arrow = lv_line_create(t0_nav_canvas);
    lv_obj_set_style_line_color(t0_beam_arrow, lv_color_hex(0xFFCC00), 0);
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
    lv_obj_set_size(t0_rec_btn, 76, 24);
    lv_obj_align(t0_rec_btn, LV_ALIGN_BOTTOM_RIGHT, -6, -6);
    lv_obj_set_style_bg_color(t0_rec_btn, COLOR_BTN_INACTIVE_BLUE, 0);
    lv_obj_set_style_radius(t0_rec_btn, 4, 0);
    lv_obj_add_event_cb(t0_rec_btn, route_toggle_btn_cb, LV_EVENT_CLICKED, NULL);
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
    // TILE 1: Field Survey / Waypoint Logger (BOOT Short-Press Active Here)
    // =====================================================================
    lv_obj_t *t1_title = lv_label_create(tile_survey);
    lv_label_set_text(t1_title, "FIELD SURVEY LOGGER (BOOT Key Active)");
    lv_obj_set_style_text_color(t1_title, lv_color_hex(0x00FF88), 0);
    lv_obj_set_style_text_font(t1_title, &lv_font_montserrat_12, 0);
    lv_obj_align(t1_title, LV_ALIGN_TOP_MID, 0, 4);

    t1_live_pos_lbl = lv_label_create(tile_survey);
    lv_label_set_text(t1_live_pos_lbl, "Tip Lat: 0.00000000\nTip Lon: 0.00000000\nGround Hgt: 0.000 m");
    lv_obj_set_style_text_color(t1_live_pos_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(t1_live_pos_lbl, &lv_font_montserrat_14, 0);
    lv_obj_align(t1_live_pos_lbl, LV_ALIGN_TOP_LEFT, 10, 22);

    t1_next_lbl = lv_label_create(tile_survey);
    lv_label_set_text(t1_next_lbl, "Next ID: PIN_NW_01_FND");
    lv_obj_set_style_text_color(t1_next_lbl, lv_color_hex(0xFFCC00), 0);
    lv_obj_set_style_text_font(t1_next_lbl, &lv_font_montserrat_16, 0);
    lv_obj_align(t1_next_lbl, LV_ALIGN_TOP_LEFT, 10, 84);

    // Target Match Switch
    lv_obj_t *match_lbl = lv_label_create(tile_survey);
    lv_label_set_text(match_lbl, "Match Active Target Name:");
    lv_obj_set_style_text_color(match_lbl, lv_color_hex(0xCCCCCC), 0);
    lv_obj_align(match_lbl, LV_ALIGN_TOP_LEFT, 10, 114);

    t1_match_sw = lv_switch_create(tile_survey);
    lv_obj_align(t1_match_sw, LV_ALIGN_TOP_RIGHT, -12, 108);
    if (g_survey_target_match_mode) lv_obj_add_state(t1_match_sw, LV_STATE_CHECKED);
    lv_obj_add_event_cb(t1_match_sw, match_target_sw_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // Prefix Dropdown & Averaging Button
    t1_prefix_dd = lv_dropdown_create(tile_survey);
    lv_dropdown_set_options(t1_prefix_dd, waypoint_get_prefix_options());
    lv_obj_set_size(t1_prefix_dd, 155, 34);
    lv_obj_align(t1_prefix_dd, LV_ALIGN_TOP_LEFT, 10, 144);
    lv_obj_add_event_cb(t1_prefix_dd, prefix_dd_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *btn_avg = lv_button_create(tile_survey);
    lv_obj_set_size(btn_avg, 130, 34);
    lv_obj_align(btn_avg, LV_ALIGN_TOP_RIGHT, -10, 144);
    lv_obj_add_event_cb(btn_avg, avg_dur_btn_cb, LV_EVENT_CLICKED, NULL);
    t1_avg_btn_lbl = lv_label_create(btn_avg);
    lv_label_set_text_fmt(t1_avg_btn_lbl, "Avg: %us", current_settings.survey_avg_seconds);
    lv_obj_center(t1_avg_btn_lbl);

    // Quick Note Tag Dropdown
    t1_note_dd = lv_dropdown_create(tile_survey);
    lv_dropdown_set_options(t1_note_dd, waypoint_get_note_tag_options());
    lv_obj_set_size(t1_note_dd, 300, 34);
    lv_obj_align(t1_note_dd, LV_ALIGN_TOP_MID, 0, 186);
    lv_obj_add_event_cb(t1_note_dd, note_dd_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // Progress bar for multi-epoch averaging
    t1_prog_bar = lv_bar_create(tile_survey);
    lv_obj_set_size(t1_prog_bar, 300, 16);
    lv_obj_align(t1_prog_bar, LV_ALIGN_TOP_MID, 0, 230);
    lv_bar_set_range(t1_prog_bar, 0, 100);
    lv_bar_set_value(t1_prog_bar, 0, LV_ANIM_OFF);

    // Large LOG WAYPOINT Button
    t1_log_btn = lv_button_create(tile_survey);
    lv_obj_set_size(t1_log_btn, 300, 50);
    lv_obj_align(t1_log_btn, LV_ALIGN_TOP_MID, 0, 254);
    lv_obj_set_style_bg_color(t1_log_btn, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_add_event_cb(t1_log_btn, log_pin_btn_cb, LV_EVENT_CLICKED, NULL);
    t1_log_btn_lbl = lv_label_create(t1_log_btn);
    lv_label_set_text(t1_log_btn_lbl, "LOG PIN NOW (or Short-Press BOOT)");
    lv_obj_center(t1_log_btn_lbl);

    t1_status_lbl = lv_label_create(tile_survey);
    lv_label_set_text(t1_status_lbl, "Last Logged: None\nSaved to /survey_logged_pins.csv");
    lv_obj_set_style_text_color(t1_status_lbl, lv_color_hex(0x99A3B0), 0);
    lv_obj_set_style_text_font(t1_status_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t1_status_lbl, LV_ALIGN_BOTTOM_LEFT, 10, -6);

    // =====================================================================
    // TILE 2: Waypoint CSV Browser
    // =====================================================================
    t2_file_lbl = lv_label_create(tile_csv);
    lv_label_set_text_fmt(t2_file_lbl, "CSV File: %s (Tap row to guide)", current_settings.active_waypoint_file);
    lv_obj_set_style_text_color(t2_file_lbl, lv_color_hex(0x00D8FF), 0);
    lv_obj_set_style_text_font(t2_file_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t2_file_lbl, LV_ALIGN_TOP_LEFT, 6, 4);

    t2_wp_table = lv_table_create(tile_csv);
    lv_obj_set_size(t2_wp_table, 310, 368);
    lv_obj_align(t2_wp_table, LV_ALIGN_BOTTOM_MID, 0, -4);
    lv_obj_set_style_text_font(t2_wp_table, &lv_font_montserrat_12, LV_PART_ITEMS);
    lv_obj_add_event_cb(t2_wp_table, wp_table_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    refresh_waypoint_table_ui();

    // =====================================================================
    // TILE 3: Dual-Antenna Compass & Pole Bubble Level
    // =====================================================================
    lv_obj_t *t3_title = lv_label_create(tile_compass);
    lv_label_set_text(t3_title, "DUAL-ANTENNA COMPASS & POLE LEVEL");
    lv_obj_set_style_text_color(t3_title, lv_color_hex(0xFFCC00), 0);
    lv_obj_set_style_text_font(t3_title, &lv_font_montserrat_12, 0);
    lv_obj_align(t3_title, LV_ALIGN_TOP_MID, 0, 4);

    t3_hdg_lbl = lv_label_create(tile_compass);
    lv_label_set_text(t3_hdg_lbl, "Heading: ---.- deg");
    lv_obj_set_style_text_color(t3_hdg_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(t3_hdg_lbl, &lv_font_montserrat_24, 0);
    lv_obj_align(t3_hdg_lbl, LV_ALIGN_TOP_MID, 0, 24);

    t3_base_lbl = lv_label_create(tile_compass);
    lv_label_set_text(t3_base_lbl, "Baseline: 0.555 m | Pitch: 0.0 deg");
    lv_obj_set_style_text_color(t3_base_lbl, lv_color_hex(0x00FFAA), 0);
    lv_obj_set_style_text_font(t3_base_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t3_base_lbl, LV_ALIGN_TOP_MID, 0, 54);

    t3_pose_lbl = lv_label_create(tile_compass);
    lv_label_set_text(t3_pose_lbl, "Display Pose: Held Flat");
    lv_obj_set_style_text_color(t3_pose_lbl, lv_color_hex(0x00D8FF), 0);
    lv_obj_set_style_text_font(t3_pose_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t3_pose_lbl, LV_ALIGN_TOP_MID, 0, 72);

    // 2D Pole Plumb Bubble Level
    t3_level_circle = lv_obj_create(tile_compass);
    lv_obj_set_size(t3_level_circle, 176, 176);
    lv_obj_align(t3_level_circle, LV_ALIGN_TOP_MID, 0, 94);
    lv_obj_set_style_radius(t3_level_circle, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(t3_level_circle, lv_color_hex(0x1A2421), 0);
    lv_obj_set_style_border_color(t3_level_circle, lv_color_hex(0x3B6E5C), 0);
    lv_obj_set_style_border_width(t3_level_circle, 2, 0);
    lv_obj_set_scrollbar_mode(t3_level_circle, LV_SCROLLBAR_MODE_OFF);

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

    t3_tilt_lbl = lv_label_create(tile_compass);
    lv_label_set_text(t3_tilt_lbl, "Pole Tilt: dX: 0.0 deg | dY: 0.0 deg");
    lv_obj_set_style_text_color(t3_tilt_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_align(t3_tilt_lbl, LV_ALIGN_TOP_MID, 0, 278);

    t3_tare_btn = lv_button_create(tile_compass);
    lv_obj_set_size(t3_tare_btn, 260, 38);
    lv_obj_align(t3_tare_btn, LV_ALIGN_BOTTOM_MID, 0, -8);
    lv_obj_add_event_cb(t3_tare_btn, tare_level_btn_cb, LV_EVENT_CLICKED, NULL);
    t3_tare_lbl = lv_label_create(t3_tare_btn);
    lv_label_set_text(t3_tare_lbl, "Tare Pole Bracket Angle (Zero Level)");
    lv_obj_center(t3_tare_lbl);

    // =====================================================================
    // TILE 4: Settings & System Info / Hardware Diagnostics
    // =====================================================================
    t4_title = lv_label_create(tile_test);
    lv_label_set_text(t4_title, "RTK ROVER " ROVER_FIRMWARE_VERSION " - SETTINGS & DIAG");
    lv_obj_set_style_text_color(t4_title, lv_color_hex(0x00FFAA), 0);
    lv_obj_set_style_text_font(t4_title, &lv_font_montserrat_14, 0);
    lv_obj_align(t4_title, LV_ALIGN_TOP_MID, 0, 4);

    // Section 1: User Preferences & Controls
    t4_wifi_btn = lv_button_create(tile_test);
    lv_obj_set_size(t4_wifi_btn, 145, 32);
    lv_obj_align(t4_wifi_btn, LV_ALIGN_TOP_LEFT, 8, 26);
    lv_obj_add_event_cb(t4_wifi_btn, wifi_toggle_btn_cb, LV_EVENT_CLICKED, NULL);
    t4_wifi_btn_lbl = lv_label_create(t4_wifi_btn);
    lv_obj_center(t4_wifi_btn_lbl);

    t4_unit_btn = lv_button_create(tile_test);
    lv_obj_set_size(t4_unit_btn, 145, 32);
    lv_obj_align(t4_unit_btn, LV_ALIGN_TOP_RIGHT, -8, 26);
    lv_obj_add_event_cb(t4_unit_btn, unit_toggle_btn_cb, LV_EVENT_CLICKED, NULL);
    t4_unit_btn_lbl = lv_label_create(t4_unit_btn);
    lv_obj_center(t4_unit_btn_lbl);

    t4_wifi_status_lbl = lv_label_create(tile_test);
    lv_obj_set_width(t4_wifi_status_lbl, 304);
    lv_label_set_text(t4_wifi_status_lbl, "Wi-Fi: Inactive (Tap Start to enable)");
    lv_obj_set_style_text_color(t4_wifi_status_lbl, lv_color_hex(0x94A3B8), 0);
    lv_obj_set_style_text_font(t4_wifi_status_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t4_wifi_status_lbl, LV_ALIGN_TOP_LEFT, 10, 62);

    t4_nmea_btn = lv_button_create(tile_test);
    lv_obj_set_size(t4_nmea_btn, 145, 32);
    lv_obj_align(t4_nmea_btn, LV_ALIGN_TOP_LEFT, 8, 80);
    lv_obj_add_event_cb(t4_nmea_btn, nmea_toggle_btn_cb, LV_EVENT_CLICKED, NULL);
    t4_nmea_btn_lbl = lv_label_create(t4_nmea_btn);
    lv_obj_center(t4_nmea_btn_lbl);

    t4_route_btn = lv_button_create(tile_test);
    lv_obj_set_size(t4_route_btn, 145, 32);
    lv_obj_align(t4_route_btn, LV_ALIGN_TOP_RIGHT, -8, 80);
    lv_obj_add_event_cb(t4_route_btn, route_toggle_btn_cb, LV_EVENT_CLICKED, NULL);
    t4_route_btn_lbl = lv_label_create(t4_route_btn);
    lv_obj_center(t4_route_btn_lbl);

    update_all_control_buttons();

    t4_storage_status_lbl = lv_label_create(tile_test);
    lv_obj_set_width(t4_storage_status_lbl, 304);
    lv_label_set_text(t4_storage_status_lbl, "Storage: Auto-Detecting...");
    lv_obj_set_style_text_color(t4_storage_status_lbl, lv_color_hex(0x38BDF8), 0);
    lv_obj_set_style_text_font(t4_storage_status_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t4_storage_status_lbl, LV_ALIGN_TOP_LEFT, 10, 116);

    // Section 2: UM982 COM1 Diagnostics
    t4_com1_lbl = lv_label_create(tile_test);
    lv_obj_set_width(t4_com1_lbl, 304);
    lv_label_set_long_mode(t4_com1_lbl, LV_LABEL_LONG_CLIP);
    lv_label_set_text(t4_com1_lbl, "[UM982 COM1 IO17/18 @ 115200]\nRX: 0 B | TX: 0 B | NMEA: 0 | HDG: 0\nLast: Waiting...");
    lv_obj_set_style_text_color(t4_com1_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(t4_com1_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t4_com1_lbl, LV_ALIGN_TOP_LEFT, 8, 138);

    lv_obj_t *btn_c1_ver = lv_button_create(tile_test);
    lv_obj_set_size(btn_c1_ver, 145, 30);
    lv_obj_align(btn_c1_ver, LV_ALIGN_TOP_LEFT, 8, 188);
    lv_obj_add_event_cb(btn_c1_ver, test_com1_ver_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l_c1_ver = lv_label_create(btn_c1_ver);
    lv_label_set_text(l_c1_ver, "COM1: VERSIONA");
    lv_obj_center(l_c1_ver);

    lv_obj_t *btn_c1_cfg = lv_button_create(tile_test);
    lv_obj_set_size(btn_c1_cfg, 145, 30);
    lv_obj_align(btn_c1_cfg, LV_ALIGN_TOP_RIGHT, -8, 188);
    lv_obj_add_event_cb(btn_c1_cfg, test_com1_cfg_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l_c1_cfg = lv_label_create(btn_c1_cfg);
    lv_label_set_text(l_c1_cfg, "COM1: CONFIG");
    lv_obj_center(l_c1_cfg);

    // Section 3: UM982 COM2 Diagnostics
    t4_com2_lbl = lv_label_create(tile_test);
    lv_obj_set_width(t4_com2_lbl, 304);
    lv_label_set_long_mode(t4_com2_lbl, LV_LABEL_LONG_CLIP);
    lv_label_set_text(t4_com2_lbl, "[UM982 COM2 IO47/48 @ 115200]\nRTCM TX: 0 B | RX: 0 B | Diff Age: -- s\nLast: Waiting...");
    lv_obj_set_style_text_color(t4_com2_lbl, lv_color_hex(0x00FFAA), 0);
    lv_obj_set_style_text_font(t4_com2_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t4_com2_lbl, LV_ALIGN_TOP_LEFT, 8, 224);

    lv_obj_t *btn_c2_test = lv_button_create(tile_test);
    lv_obj_set_size(btn_c2_test, 304, 30);
    lv_obj_align(btn_c2_test, LV_ALIGN_TOP_MID, 0, 274);
    lv_obj_add_event_cb(btn_c2_test, test_com2_pulse_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l_c2_test = lv_label_create(btn_c2_test);
    lv_label_set_text(l_c2_test, "COM2: Send VERSIONA Test (IO47/48)");
    lv_obj_center(l_c2_test);

    // Section 4: E220 LoRa
    char init_buf[128];
    t4_e220_lbl = lv_label_create(tile_test);
    snprintf(init_buf, sizeof(init_buf),
             "[E220 LoRa IO41/42 @ %lu]\nAUX:1 M0:0 M1:0 | RX: 0 B | RTCM OK: 0\nLast Msg: -- | 1005: 0 | MSM: 0",
             (unsigned long)current_settings.e220_uart_baud);
    lv_label_set_text(t4_e220_lbl, init_buf);
    lv_obj_set_style_text_color(t4_e220_lbl, lv_color_hex(0xFFCC00), 0);
    lv_obj_set_style_text_font(t4_e220_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t4_e220_lbl, LV_ALIGN_TOP_LEFT, 8, 310);

    t4_e220_cfg_lbl = lv_label_create(tile_test);
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
    lv_obj_align(t4_e220_cfg_lbl, LV_ALIGN_TOP_LEFT, 8, 360);

    lv_obj_t *btn_e220_cfg = lv_button_create(tile_test);
    lv_obj_set_size(btn_e220_cfg, 145, 32);
    lv_obj_align(btn_e220_cfg, LV_ALIGN_TOP_LEFT, 8, 380);
    lv_obj_add_event_cb(btn_e220_cfg, test_e220_read_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l_e220_cfg = lv_label_create(btn_e220_cfg);
    lv_label_set_text(l_e220_cfg, "Read E220 Regs");
    lv_obj_center(l_e220_cfg);

    lv_obj_t *btn_e220_wr = lv_button_create(tile_test);
    lv_obj_set_size(btn_e220_wr, 145, 32);
    lv_obj_align(btn_e220_wr, LV_ALIGN_TOP_RIGHT, -8, 380);
    lv_obj_set_style_bg_color(btn_e220_wr, lv_palette_main(LV_PALETTE_ORANGE), 0);
    lv_obj_add_event_cb(btn_e220_wr, test_e220_write_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l_e220_wr = lv_label_create(btn_e220_wr);
    lv_label_set_text(l_e220_wr, "Write E220 Cfg");
    lv_obj_center(l_e220_wr);
}

void update_rover_ui(const RoverUIData_t *d)
{
    if (!d) return;
    if (!rover_lvgl_lock(50)) return;

    char buf[256];

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

    // Wi-Fi Status Badge in Row 1
    if (hdr_wifi_lbl) {
        if (is_wifi_active()) {
            lv_label_set_text(hdr_wifi_lbl, LV_SYMBOL_WIFI " ON");
            lv_obj_set_style_text_color(hdr_wifi_lbl, lv_color_hex(0x00FF88), 0);
        } else {
            lv_label_set_text(hdr_wifi_lbl, LV_SYMBOL_WIFI " OFF");
            lv_obj_set_style_text_color(hdr_wifi_lbl, lv_color_hex(0x64748B), 0);
        }
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

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    uint32_t rtcm_age_ms = (d->e220_stats.last_rx_ms > 0) ? (now_ms - d->e220_stats.last_rx_ms) : 99999;
    if (rtcm_age_ms < 15000) {
        snprintf(buf, sizeof(buf), "RTCM:%u.%us", (unsigned int)(rtcm_age_ms / 1000), (unsigned int)((rtcm_age_ms % 1000) / 100));
        lv_label_set_text(hdr_m_rtcm_lbl, buf);
        lv_obj_set_style_text_color(hdr_m_rtcm_lbl, lv_color_hex(0x00FF88), 0);
    } else {
        lv_label_set_text(hdr_m_rtcm_lbl, "RTCM:Wait");
        lv_obj_set_style_text_color(hdr_m_rtcm_lbl, lv_color_hex(0xFF8800), 0);
    }

    // Row 3: Slave Antenna (ANT2) Status & Metrics
    const char *hdg_badge_str = "HDG --";
    lv_color_t hdg_badge_col  = lv_color_hex(0x3A3F47);
    if (d->heading_valid) {
        if (d->heading_is_fixed) {
            hdg_badge_str = "HDG FIX";
            hdg_badge_col = lv_palette_main(LV_PALETTE_GREEN);
        } else {
            hdg_badge_str = "HDG FLT";
            hdg_badge_col = lv_palette_main(LV_PALETTE_ORANGE);
        }
    } else if (d->slave_fix_type == GNSS_FIX_RTK_FIXED) {
        hdg_badge_str = "SLV FIX";
        hdg_badge_col = lv_palette_main(LV_PALETTE_GREEN);
    } else if (d->slave_fix_type == GNSS_FIX_RTK_FLOAT) {
        hdg_badge_str = "SLV FLT";
        hdg_badge_col = lv_palette_main(LV_PALETTE_ORANGE);
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
        snprintf(buf, sizeof(buf), "Hdg:%.1f", d->true_heading_deg);
    } else {
        snprintf(buf, sizeof(buf), "Hdg:---.-");
    }
    lv_label_set_text(hdr_s_hdg_lbl, buf);

    // Row 4: Streamlined Coordinates Bar
    char hgt_str[24], dist_str[24], dn_str[24], de_str[24];
    nav_format_height(d->tip_height_m, hgt_str, sizeof(hgt_str));
    nav_format_distance(d->distance_to_target_m, dist_str, sizeof(dist_str));
    nav_format_offset(d->d_north_m, dn_str, sizeof(dn_str));
    nav_format_offset(d->d_east_m, de_str, sizeof(de_str));

    snprintf(buf, sizeof(buf), "%.8f, %.8f, %s", d->tip_lat, d->tip_lon, hgt_str);
    if (hdr_coords_lbl) lv_label_set_text(hdr_coords_lbl, buf);

    // Push position into breadcrumb buffer
    nav_breadcrumb_push(d->tip_lat, d->tip_lon);

    // Update all control buttons (Colors & Labels)
    update_all_control_buttons();

    // Update Canvas Top-Left Telemetry HUD (Heading & Pitch)
    char hud_buf[32];
    if (t0_hdg_hud_lbl) {
        if (d->heading_valid) {
            snprintf(hud_buf, sizeof(hud_buf), "Hdg: %06.2f", d->true_heading_deg);
        } else {
            snprintf(hud_buf, sizeof(hud_buf), "Hdg: ---.--");
        }
        lv_label_set_text(t0_hdg_hud_lbl, hud_buf);
    }

    if (t0_pitch_hud_lbl) {
        if (d->heading_valid) {
            snprintf(hud_buf, sizeof(hud_buf), "Pitch: %+.1f", d->baseline_pitch_deg);
        } else if (fabsf(d->imu_pitch_deg) > 0.01f) {
            snprintf(hud_buf, sizeof(hud_buf), "Pitch: %+.1f", d->imu_pitch_deg);
        } else {
            snprintf(hud_buf, sizeof(hud_buf), "Pitch: --.-");
        }
        lv_label_set_text(t0_pitch_hud_lbl, hud_buf);
    }

    if (d->has_active_target) {
        snprintf(buf, sizeof(buf), "%s (%d/%d)",
                 d->target_label, d->active_target_idx + 1, d->total_targets);
        lv_label_set_text(t0_target_lbl, buf);
        lv_label_set_text(t0_notes_lbl, d->target_notes[0] ? d->target_notes : "---");
        lv_label_set_text(t0_dist_lbl, dist_str);

        if (d->distance_to_target_m <= 1.5) {
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

            // Show bullseye rings
            lv_obj_set_hidden(t0_bullseye_ring1, false);
            lv_obj_set_hidden(t0_bullseye_ring2, false);
            lv_obj_set_hidden(t0_bullseye_center, false);
            lv_obj_set_hidden(t0_cross_h, false);
            lv_obj_set_hidden(t0_cross_v, false);

            for (int i = 0; i < T0_BREADCRUMB_DOTS; i++) {
                if (t0_bc_dots[i]) lv_obj_set_hidden(t0_bc_dots[i], true);
            }

            // Target Pin at Center
            lv_obj_align(t0_target_icon, LV_ALIGN_CENTER, 0, 0);

            // Rover Pole Tip moves relative to target center (70 px = 1.0m)
            int16_t r_px = (int16_t)(-d->d_east_m * 70.0f);
            int16_t r_py = (int16_t)( d->d_north_m * 70.0f);
            if (r_px > 80) r_px = 80; else if (r_px < -80) r_px = -80;
            if (r_py > 80) r_py = 80; else if (r_py < -80) r_py = -80;
            lv_obj_align(t0_rover_icon, LV_ALIGN_CENTER, r_px, r_py);

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
            // Coarse Tactical Grid Mode (> 1.5 m): Course-Up Tactical Canvas
            // -------------------------------------------------------------
            lv_label_set_text(t0_mode_badge, "TACTICAL GRID");
            lv_obj_set_style_bg_color(t0_mode_badge, lv_color_hex(0x0088CC), 0);

            // Show tactical background grid & track line
            for (int i = 0; i < T0_GRID_H_LINES; i++) {
                if (t0_grid_h[i]) lv_obj_set_hidden(t0_grid_h[i], false);
            }
            for (int i = 0; i < T0_GRID_V_LINES; i++) {
                if (t0_grid_v[i]) lv_obj_set_hidden(t0_grid_v[i], false);
            }
            lv_obj_set_hidden(t0_track_line, false);

            // Hide fine bullseye rings
            lv_obj_set_hidden(t0_bullseye_ring1, true);
            lv_obj_set_hidden(t0_bullseye_ring2, true);
            lv_obj_set_hidden(t0_bullseye_center, true);
            lv_obj_set_hidden(t0_cross_h, false);
            lv_obj_set_hidden(t0_cross_v, false);

            // Dynamic scaling
            float scale_m = 5.0f;
            if (d->distance_to_target_m > 250.0) scale_m = 500.0f;
            else if (d->distance_to_target_m > 100.0) scale_m = 250.0f;
            else if (d->distance_to_target_m > 50.0) scale_m = 100.0f;
            else if (d->distance_to_target_m > 20.0) scale_m = 50.0f;
            else if (d->distance_to_target_m > 10.0) scale_m = 20.0f;
            else if (d->distance_to_target_m > 5.0) scale_m = 10.0f;

            char scale_str[24];
            nav_format_distance((double)scale_m, scale_str, sizeof(scale_str));
            snprintf(buf, sizeof(buf), "Range: %s", scale_str);
            lv_label_set_text(t0_scale_lbl, buf);

            // Rover icon anchored near bottom (y = 170)
            int16_t rover_x = 152;
            int16_t rover_y = 170;
            lv_obj_align(t0_rover_icon, LV_ALIGN_TOP_LEFT, rover_x - 8, rover_y - 8);

            // Target pin projected along Course-Up forward line
            float norm_dist = (float)d->distance_to_target_m / scale_m;
            if (norm_dist > 1.0f) norm_dist = 1.0f;
            else if (norm_dist < 0.15f) norm_dist = 0.15f;

            int16_t target_x = rover_x;
            int16_t target_y = rover_y - (int16_t)(norm_dist * 135.0f);
            lv_obj_align(t0_target_icon, LV_ALIGN_TOP_LEFT, target_x - 8, target_y - 8);

            // Track line
            static lv_point_precise_t track_pts[2];
            track_pts[0].x = rover_x;
            track_pts[0].y = rover_y;
            track_pts[1].x = target_x;
            track_pts[1].y = target_y;
            lv_line_set_points(t0_track_line, track_pts, 2);

            // Rover antenna beam vector
            if (d->heading_valid) {
                float rel_turn = d->rel_bearing_deg;
                float beam_angle_rad = (-rel_turn) * (float)(M_PI / 180.0);
                int16_t b_dx = (int16_t)(32.0f * sinf(beam_angle_rad));
                int16_t b_dy = (int16_t)(-32.0f * cosf(beam_angle_rad));
                static lv_point_precise_t arrow_pts[2];
                arrow_pts[0].x = rover_x;
                arrow_pts[0].y = rover_y;
                arrow_pts[1].x = rover_x + b_dx;
                arrow_pts[1].y = rover_y + b_dy;
                lv_line_set_points(t0_beam_arrow, arrow_pts, 2);
                lv_obj_set_hidden(t0_beam_arrow, false);

                // Clean ASCII steering cues
                if (fabsf(rel_turn) <= 3.0f) {
                    snprintf(buf, sizeof(buf), "[OK] ON TRACK - Aimed at Pin");
                    lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0x00FF88), 0);
                } else if (rel_turn > 0) {
                    snprintf(buf, sizeof(buf), ">>> Turn Right %.0f deg", rel_turn);
                    lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0xFFCC00), 0);
                } else {
                    snprintf(buf, sizeof(buf), "<<< Turn Left %.0f deg", -rel_turn);
                    lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0xFFCC00), 0);
                }
            } else {
                lv_obj_set_hidden(t0_beam_arrow, true);
                snprintf(buf, sizeof(buf), "Brg: %.1f deg | Acquiring Heading...", d->bearing_to_target_deg);
                lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0xFF8800), 0);
            }
            lv_label_set_text(t0_steer_lbl, buf);

            // Breadcrumb trail dots
            int bc_total = nav_breadcrumb_get_count();
            float brg_rad = d->bearing_to_target_deg * (float)(M_PI / 180.0);
            float cos_b = cosf(brg_rad);
            float sin_b = sinf(brg_rad);

            for (int i = 0; i < T0_BREADCRUMB_DOTS; i++) {
                if (!t0_bc_dots[i]) continue;
                if (i < bc_total) {
                    NavBreadcrumb_t pt;
                    int sample_idx = bc_total - 1 - i;
                    if (nav_breadcrumb_get_point(sample_idx, &pt)) {
                        double dlat = pt.lat - d->tip_lat;
                        double dlon = pt.lon - d->tip_lon;
                        float dn = (float)(dlat * 111132.95);
                        float de = (float)(dlon * 111132.95 * cos(d->tip_lat * 0.0174532925));

                        float u =  de * cos_b - dn * sin_b;
                        float v =  de * sin_b + dn * cos_b;

                        int16_t dot_x = rover_x + (int16_t)((u / scale_m) * 135.0f);
                        int16_t dot_y = rover_y - (int16_t)((v / scale_m) * 135.0f);

                        if (dot_x >= 220 && dot_y >= 174) {
                            lv_obj_set_hidden(t0_bc_dots[i], true);
                        } else if (dot_x >= 4 && dot_x <= 300 && dot_y >= 4 && dot_y <= 206) {
                            lv_obj_align(t0_bc_dots[i], LV_ALIGN_TOP_LEFT, dot_x - 2, dot_y - 2);
                            lv_obj_set_hidden(t0_bc_dots[i], false);
                        } else {
                            lv_obj_set_hidden(t0_bc_dots[i], true);
                        }
                    } else {
                        lv_obj_set_hidden(t0_bc_dots[i], true);
                    }
                } else {
                    lv_obj_set_hidden(t0_bc_dots[i], true);
                }
            }
        }

        snprintf(buf, sizeof(buf), "dN: %s   dE: %s", dn_str, de_str);
        lv_label_set_text(t0_offsets_lbl, buf);
    }

    // 3. Update Tile 1: Field Survey Logger
    snprintf(buf, sizeof(buf),
             "Tip Lat: %.8f\nTip Lon: %.8f\nGround Hgt: %s (%s)",
             d->tip_lat, d->tip_lon, hgt_str, fix_str);
    lv_label_set_text(t1_live_pos_lbl, buf);

    char next_id[WAYPOINT_LABEL_LEN];
    waypoint_format_next_survey_label(next_id, sizeof(next_id));
    snprintf(buf, sizeof(buf), "Next ID: %s", next_id);
    lv_label_set_text(t1_next_lbl, buf);

    lv_bar_set_value(t1_prog_bar, d->survey_progress_pct, LV_ANIM_OFF);
    if (d->survey_in_progress) {
        snprintf(buf, sizeof(buf), "AVERAGING RTK... %u%% (%u pts)",
                 d->survey_progress_pct, d->survey_epochs_collected);
        lv_label_set_text(t1_log_btn_lbl, buf);
        lv_obj_set_style_bg_color(t1_log_btn, lv_palette_main(LV_PALETTE_ORANGE), 0);
    } else {
        lv_label_set_text(t1_log_btn_lbl, "LOG PIN NOW (or Short-Press BOOT)");
        lv_obj_set_style_bg_color(t1_log_btn, lv_palette_main(LV_PALETTE_GREEN), 0);
    }
    snprintf(buf, sizeof(buf), "Last Logged: %s\nFile: /survey_logged_pins.csv",
             d->last_logged_label);
    lv_label_set_text(t1_status_lbl, buf);

    // 4. Update Tile 3: Compass & Bubble Level
    if (d->heading_valid) {
        snprintf(buf, sizeof(buf), "Heading: %05.1f deg (%s)",
                 d->true_heading_deg,
                 d->heading_is_fixed ? "FIX" : "FLT");
        lv_label_set_text(t3_hdg_lbl, buf);
    } else {
        lv_label_set_text(t3_hdg_lbl, "Heading: ---.- deg");
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

    int16_t bx = (int16_t)(d->pole_bubble_x_deg * 6.0f);
    int16_t by = (int16_t)(d->pole_bubble_y_deg * 6.0f);
    if (bx > 70) bx = 70; if (bx < -70) bx = -70;
    if (by > 70) by = 70; if (by < -70) by = -70;
    lv_obj_align(t3_bubble_dot, LV_ALIGN_CENTER, bx, by);
    snprintf(buf, sizeof(buf), "Pole Tilt: dX: %+.1f deg | dY: %+.1f deg",
             d->pole_bubble_x_deg, d->pole_bubble_y_deg);
    lv_label_set_text(t3_tilt_lbl, buf);

    // 5. Update Tile 4: Settings & System Info Status
    if (t4_storage_status_lbl) {
        extern bool sd_card_mounted;
        if (sd_card_mounted) {
            lv_label_set_text(t4_storage_status_lbl, "Storage: MicroSD Card Ready (Primary)");
            lv_obj_set_style_text_color(t4_storage_status_lbl, lv_color_hex(0x00FF88), 0);
        } else {
            lv_label_set_text(t4_storage_status_lbl, "Storage: FFat Internal Flash (Fallback)");
            lv_obj_set_style_text_color(t4_storage_status_lbl, lv_color_hex(0xFFA500), 0);
        }
    }

    // 6. Update Tile 4: Hardware Diagnostics
    snprintf(buf, sizeof(buf),
             "[UM982 COM1 IO17/18 @ 115200] [%s] (M_HDOP:%.1f | S_HDOP:%.1f)\nRX:%lu B | TX:%lu B | NMEA:%lu | HDG:%lu\nLast: %s",
             d->um982_stats.com1_test_passed ? "PASS" : "WAIT",
             d->hdop,
             d->slave_hdop,
             (unsigned long)d->um982_stats.com1_rx_bytes,
             (unsigned long)d->um982_stats.com1_tx_bytes,
             (unsigned long)d->um982_stats.nmea_sentences_ok,
             (unsigned long)d->um982_stats.heading_sentences_ok,
             d->um982_stats.last_com1_line);
    lv_label_set_text(t4_com1_lbl, buf);

    snprintf(buf, sizeof(buf),
             "[UM982 COM2 IO47/48 @ 115200] [%s]\nRTCM TX:%lu B | RX:%lu B | Diff:%.1fs\nLast: %s",
             d->um982_stats.com2_test_passed ? "PASS" : "WAIT",
             (unsigned long)d->um982_stats.com2_tx_bytes,
             (unsigned long)d->um982_stats.com2_rx_bytes,
             d->diff_age_s,
             d->um982_stats.last_com2_line);
    lv_label_set_text(t4_com2_lbl, buf);

    snprintf(buf, sizeof(buf),
             "[E220 LoRa IO41/42 @ %lu] [%s]\nAUX:%d M0:%u M1:%u | RX:%lu B | RTCM OK:%lu\nLast Msg:%u | 1005:%u | MSM:%u",
             (unsigned long)current_settings.e220_uart_baud,
             d->e220_stats.config_valid ? (d->e220_stats.config_matched ? "CFG OK" : "CFG DIFF") : "NO RSP",
             d->e220_stats.aux_pin_high ? 1 : 0,
             d->e220_stats.mode_m0,
             d->e220_stats.mode_m1,
             (unsigned long)d->e220_stats.rx_bytes_total,
             (unsigned long)d->e220_stats.rtcm_frames_ok,
             d->e220_stats.last_msg_type,
             d->e220_stats.msg_1005_count,
             d->e220_stats.msg_msm_count);
    lv_label_set_text(t4_e220_lbl, buf);

    if (d->e220_stats.config_valid) {
        snprintf(buf, sizeof(buf),
                 "E220 Regs: ADDH:%02X ADDL:%02X REG0:%02X REG1:%02X CH:%02X REG3:%02X",
                 d->e220_stats.cfg_addr_high,
                 d->e220_stats.cfg_addr_low,
                 d->e220_stats.cfg_reg0,
                 d->e220_stats.cfg_reg1,
                 d->e220_stats.cfg_channel,
                 d->e220_stats.cfg_reg3);
        lv_label_set_text(t4_e220_cfg_lbl, buf);
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
    xTaskCreatePinnedToCore(rover_lvgl_port_task, "LVGL", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL, 1);

    if (rover_lvgl_lock(-1)) {
        create_rover_ui();
        rover_lvgl_unlock();
    }
}
