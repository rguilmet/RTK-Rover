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
static lv_obj_t *hdr_datetime_lbl = NULL;
static lv_obj_t *hdr_units_lbl    = NULL;
static lv_obj_t *hdr_batt_lbl     = NULL;
static lv_obj_t *hdr_fix_badge    = NULL;
static lv_obj_t *hdr_fix_lbl      = NULL;
static lv_obj_t *hdr_sats_acc_lbl = NULL;
static lv_obj_t *hdr_rtcm_lbl     = NULL;

// =========================================================================
// Tileview & Navigation
// =========================================================================
static lv_obj_t *main_tv = NULL;
static lv_obj_t *tile_guidance = NULL;
static lv_obj_t *tile_survey   = NULL;
static lv_obj_t *tile_csv      = NULL;
static lv_obj_t *tile_compass  = NULL;
static lv_obj_t *tile_test     = NULL;
static volatile uint8_t active_tile_col = 0;

// =========================================================================
// Tile 0: Main Guidance / Pin Seeker Widgets
// =========================================================================
static lv_obj_t *t0_pos_lbl       = NULL;
static lv_obj_t *t0_target_lbl    = NULL;
static lv_obj_t *t0_notes_lbl     = NULL;
static lv_obj_t *t0_radar_circle  = NULL;
static lv_obj_t *t0_ring_mid      = NULL;
static lv_obj_t *t0_target_dot    = NULL;
static lv_obj_t *t0_compass_n     = NULL;
static lv_obj_t *t0_dist_lbl      = NULL;
static lv_obj_t *t0_brg_lbl       = NULL;
static lv_obj_t *t0_offsets_lbl   = NULL;
static lv_obj_t *t0_unit_btn_lbl  = NULL;

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

// =========================================================================
// Tile 4: Hardware Test Modes (UM982 COM1, COM2, E220)
// =========================================================================
static lv_obj_t *t4_com1_lbl      = NULL;
static lv_obj_t *t4_com2_lbl      = NULL;
static lv_obj_t *t4_e220_lbl      = NULL;
static lv_obj_t *t4_e220_cfg_lbl  = NULL;

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
    uint8_t read_touchpad_cmd[11] = {0xb5, 0xab, 0xa5, 0x5a, 0x0, 0x0, 0x0, 0x0e, 0x0, 0x0, 0x0};
    uint8_t buff[32] = {0};
    if (i2c_master_write_read_dev(disp_touch_dev_handle, read_touchpad_cmd, 11, buff, 32) == ESP_OK) {
        uint16_t pointX = (((uint16_t)buff[2] & 0x0f) << 8) | (uint16_t)buff[3];
        uint16_t pointY = (((uint16_t)buff[4] & 0x0f) << 8) | (uint16_t)buff[5];
        if (buff[1] > 0 && buff[1] < 5) {
            indevData->state = LV_INDEV_STATE_PRESSED;
            if (pointX >= ROVER_LCD_H_RES) pointX = ROVER_LCD_H_RES - 1;
            if (pointY >= ROVER_LCD_V_RES) pointY = ROVER_LCD_V_RES - 1;
            indevData->point.x = pointX;
            indevData->point.y = pointY;
            return;
        }
    }
    indevData->state = LV_INDEV_STATE_RELEASED;
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

    ESP_ERROR_CHECK(esp_io_expander_set_dir(io_expander, ROVER_EXIO_TP_INT | ROVER_EXIO_RTC_INT | ROVER_EXIO_AXP_IRQ | ROVER_EXIO_SYS_OUT, IO_EXPANDER_INPUT));
    ESP_ERROR_CHECK(esp_io_expander_set_dir(io_expander, ROVER_EXIO_LCD_RST | ROVER_EXIO_SD_CS | ROVER_EXIO_PA_CTRL | ROVER_EXIO_CAM_PWDN, IO_EXPANDER_OUTPUT));
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
static void tv_changed_event_cb(lv_event_t *e)
{
    (void)e;
    lv_obj_t *act = lv_tileview_get_tile_active(main_tv);
    if (act == tile_guidance)     active_tile_col = 0;
    else if (act == tile_survey)  active_tile_col = 1;
    else if (act == tile_csv)     active_tile_col = 2;
    else if (act == tile_compass) active_tile_col = 3;
    else if (act == tile_test)    active_tile_col = 4;
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

static void unit_toggle_btn_cb(lv_event_t *e)
{
    (void)e;
    current_settings.use_imperial_units = !current_settings.use_imperial_units;
    mark_settings_dirty();
    if (t0_unit_btn_lbl) {
        lv_label_set_text(t0_unit_btn_lbl, current_settings.use_imperial_units ? "Units: IMP (ft/in)" : "Units: SI (m/cm)");
    }
}

static void goto_survey_tile_btn_cb(lv_event_t *e)
{
    (void)e;
    if (main_tv && tile_survey) {
        lv_tileview_set_tile(main_tv, tile_survey, LV_ANIM_ON);
        active_tile_col = 1;
    }
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
        if (main_tv && tile_guidance) {
            lv_tileview_set_tile(main_tv, tile_guidance, LV_ANIM_ON);
            active_tile_col = 0;
        }
    }
}

static void tare_level_btn_cb(lv_event_t *e)
{
    (void)e;
    trigger_tare_pole_level();
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
    // 1. Persistent Top Status Header Bar (320 x 48, always visible)
    // =====================================================================
    lv_obj_t *hdr = lv_obj_create(scr);
    lv_obj_set_size(hdr, ROVER_LCD_H_RES, 48);
    lv_obj_align(hdr, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_set_style_bg_color(hdr, lv_color_hex(0x1E2229), 0);
    lv_obj_set_style_border_width(hdr, 0, 0);
    lv_obj_set_style_radius(hdr, 0, 0);
    lv_obj_set_style_pad_all(hdr, 3, 0);
    lv_obj_set_scrollbar_mode(hdr, LV_SCROLLBAR_MODE_OFF);

    hdr_datetime_lbl = lv_label_create(hdr);
    lv_label_set_text(hdr_datetime_lbl, "2026-10-23 00:00:00");
    lv_obj_set_style_text_color(hdr_datetime_lbl, lv_color_hex(0xE0E6ED), 0);
    lv_obj_set_style_text_font(hdr_datetime_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(hdr_datetime_lbl, LV_ALIGN_TOP_LEFT, 2, 0);

    hdr_units_lbl = lv_label_create(hdr);
    lv_label_set_text(hdr_units_lbl, "SI");
    lv_obj_set_style_text_color(hdr_units_lbl, lv_color_hex(0x00D8FF), 0);
    lv_obj_set_style_text_font(hdr_units_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(hdr_units_lbl, LV_ALIGN_TOP_MID, 28, 0);

    hdr_batt_lbl = lv_label_create(hdr);
    lv_label_set_text(hdr_batt_lbl, "100% 4.1V");
    lv_obj_set_style_text_color(hdr_batt_lbl, lv_color_hex(0x55FF55), 0);
    lv_obj_set_style_text_font(hdr_batt_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(hdr_batt_lbl, LV_ALIGN_TOP_RIGHT, -2, 0);

    hdr_fix_badge = lv_obj_create(hdr);
    lv_obj_set_size(hdr_fix_badge, 84, 20);
    lv_obj_align(hdr_fix_badge, LV_ALIGN_BOTTOM_LEFT, 2, 0);
    lv_obj_set_style_bg_color(hdr_fix_badge, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_set_style_border_width(hdr_fix_badge, 0, 0);
    lv_obj_set_style_radius(hdr_fix_badge, 4, 0);
    lv_obj_set_style_pad_all(hdr_fix_badge, 0, 0);

    hdr_fix_lbl = lv_label_create(hdr_fix_badge);
    lv_label_set_text(hdr_fix_lbl, "NO FIX");
    lv_obj_set_style_text_color(hdr_fix_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(hdr_fix_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(hdr_fix_lbl);

    hdr_sats_acc_lbl = lv_label_create(hdr);
    lv_label_set_text(hdr_sats_acc_lbl, "SV:0  Acc:--");
    lv_obj_set_style_text_color(hdr_sats_acc_lbl, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_text_font(hdr_sats_acc_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(hdr_sats_acc_lbl, LV_ALIGN_BOTTOM_MID, 8, -2);

    hdr_rtcm_lbl = lv_label_create(hdr);
    lv_label_set_text(hdr_rtcm_lbl, "RTCM:--");
    lv_obj_set_style_text_color(hdr_rtcm_lbl, lv_color_hex(0xFFAA00), 0);
    lv_obj_set_style_text_font(hdr_rtcm_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(hdr_rtcm_lbl, LV_ALIGN_BOTTOM_RIGHT, -2, -2);

    // =====================================================================
    // 2. Main Swipeable Tileview (320 x 432 below Header Bar)
    // =====================================================================
    main_tv = lv_tileview_create(scr);
    lv_obj_set_size(main_tv, ROVER_LCD_H_RES, ROVER_LCD_V_RES - 48);
    lv_obj_align(main_tv, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_bg_color(main_tv, lv_color_hex(0x121417), 0);
    lv_obj_add_event_cb(main_tv, tv_changed_event_cb, LV_EVENT_VALUE_CHANGED, NULL);

    tile_guidance = lv_tileview_add_tile(main_tv, 0, 0, LV_DIR_RIGHT);
    tile_survey   = lv_tileview_add_tile(main_tv, 1, 0, (lv_dir_t)(LV_DIR_LEFT | LV_DIR_RIGHT));
    tile_csv      = lv_tileview_add_tile(main_tv, 2, 0, (lv_dir_t)(LV_DIR_LEFT | LV_DIR_RIGHT));
    tile_compass  = lv_tileview_add_tile(main_tv, 3, 0, (lv_dir_t)(LV_DIR_LEFT | LV_DIR_RIGHT));
    tile_test     = lv_tileview_add_tile(main_tv, 4, 0, LV_DIR_LEFT);

    // =====================================================================
    // TILE 0: Pin Seeker & Guidance (Main Screen)
    // =====================================================================
    t0_pos_lbl = lv_label_create(tile_guidance);
    lv_label_set_text(t0_pos_lbl, "Lat:  0.00000000\nLon:  0.00000000   Hgt: 0.00 m");
    lv_obj_set_style_text_color(t0_pos_lbl, lv_color_hex(0x00FFAA), 0);
    lv_obj_set_style_text_font(t0_pos_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t0_pos_lbl, LV_ALIGN_TOP_MID, 0, 4);

    // Target selector row
    lv_obj_t *btn_prev = lv_button_create(tile_guidance);
    lv_obj_set_size(btn_prev, 42, 34);
    lv_obj_align(btn_prev, LV_ALIGN_TOP_LEFT, 6, 40);
    lv_obj_add_event_cb(btn_prev, prev_wp_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl_prev = lv_label_create(btn_prev);
    lv_label_set_text(lbl_prev, LV_SYMBOL_LEFT);
    lv_obj_center(lbl_prev);

    t0_target_lbl = lv_label_create(tile_guidance);
    lv_label_set_text(t0_target_lbl, "PIN_NW_01 (1/4)");
    lv_obj_set_style_text_color(t0_target_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(t0_target_lbl, &lv_font_montserrat_16, 0);
    lv_obj_align(t0_target_lbl, LV_ALIGN_TOP_MID, 0, 42);

    t0_notes_lbl = lv_label_create(tile_guidance);
    lv_label_set_text(t0_notes_lbl, "NW corner iron pipe");
    lv_obj_set_style_text_color(t0_notes_lbl, lv_color_hex(0x99A3B0), 0);
    lv_obj_set_style_text_font(t0_notes_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t0_notes_lbl, LV_ALIGN_TOP_MID, 0, 62);

    lv_obj_t *btn_next = lv_button_create(tile_guidance);
    lv_obj_set_size(btn_next, 42, 34);
    lv_obj_align(btn_next, LV_ALIGN_TOP_RIGHT, -6, 40);
    lv_obj_add_event_cb(btn_next, next_wp_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl_next = lv_label_create(btn_next);
    lv_label_set_text(lbl_next, LV_SYMBOL_RIGHT);
    lv_obj_center(lbl_next);

    // 2D Bullseye / Compass Guidance Circle
    t0_radar_circle = lv_obj_create(tile_guidance);
    lv_obj_set_size(t0_radar_circle, 180, 180);
    lv_obj_align(t0_radar_circle, LV_ALIGN_TOP_MID, 0, 82);
    lv_obj_set_style_radius(t0_radar_circle, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(t0_radar_circle, lv_color_hex(0x182026), 0);
    lv_obj_set_style_border_color(t0_radar_circle, lv_color_hex(0x2F4F5F), 0);
    lv_obj_set_style_border_width(t0_radar_circle, 2, 0);
    lv_obj_set_scrollbar_mode(t0_radar_circle, LV_SCROLLBAR_MODE_OFF);

    t0_ring_mid = lv_obj_create(t0_radar_circle);
    lv_obj_set_size(t0_ring_mid, 90, 90);
    lv_obj_center(t0_ring_mid);
    lv_obj_set_style_radius(t0_ring_mid, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_opa(t0_ring_mid, 0, 0);
    lv_obj_set_style_border_color(t0_ring_mid, lv_color_hex(0x25404D), 0);
    lv_obj_set_style_border_width(t0_ring_mid, 1, 0);

    lv_obj_t *cross_h = lv_obj_create(t0_radar_circle);
    lv_obj_set_size(cross_h, 168, 1);
    lv_obj_set_style_bg_color(cross_h, lv_color_hex(0x25404D), 0);
    lv_obj_set_style_border_width(cross_h, 0, 0);
    lv_obj_center(cross_h);

    lv_obj_t *cross_v = lv_obj_create(t0_radar_circle);
    lv_obj_set_size(cross_v, 1, 168);
    lv_obj_set_style_bg_color(cross_v, lv_color_hex(0x25404D), 0);
    lv_obj_set_style_border_width(cross_v, 0, 0);
    lv_obj_center(cross_v);

    t0_compass_n = lv_label_create(t0_radar_circle);
    lv_label_set_text(t0_compass_n, "N");
    lv_obj_set_style_text_color(t0_compass_n, lv_color_hex(0xFF4444), 0);
    lv_obj_align(t0_compass_n, LV_ALIGN_TOP_MID, 0, 2);

    t0_target_dot = lv_obj_create(t0_radar_circle);
    lv_obj_set_size(t0_target_dot, 16, 16);
    lv_obj_set_style_radius(t0_target_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(t0_target_dot, lv_color_hex(0x00FF66), 0);
    lv_obj_set_style_border_color(t0_target_dot, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(t0_target_dot, 2, 0);
    lv_obj_center(t0_target_dot);

    // Distance & Bearing readouts
    t0_dist_lbl = lv_label_create(tile_guidance);
    lv_label_set_text(t0_dist_lbl, "0.00 m");
    lv_obj_set_style_text_color(t0_dist_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(t0_dist_lbl, &lv_font_montserrat_24, 0);
    lv_obj_align(t0_dist_lbl, LV_ALIGN_TOP_MID, 0, 266);

    t0_brg_lbl = lv_label_create(tile_guidance);
    lv_label_set_text(t0_brg_lbl, "Brg: 0.0 deg | Hdg: --");
    lv_obj_set_style_text_color(t0_brg_lbl, lv_color_hex(0xFFCC00), 0);
    lv_obj_set_style_text_font(t0_brg_lbl, &lv_font_montserrat_14, 0);
    lv_obj_align(t0_brg_lbl, LV_ALIGN_TOP_MID, 0, 296);

    t0_offsets_lbl = lv_label_create(tile_guidance);
    lv_label_set_text(t0_offsets_lbl, "dN: +0.0 cm   dE: +0.0 cm");
    lv_obj_set_style_text_color(t0_offsets_lbl, lv_color_hex(0x00D8FF), 0);
    lv_obj_set_style_text_font(t0_offsets_lbl, &lv_font_montserrat_14, 0);
    lv_obj_align(t0_offsets_lbl, LV_ALIGN_TOP_MID, 0, 318);

    // Footer buttons on Tile 0
    lv_obj_t *btn_unit = lv_button_create(tile_guidance);
    lv_obj_set_size(btn_unit, 145, 38);
    lv_obj_align(btn_unit, LV_ALIGN_BOTTOM_LEFT, 8, -8);
    lv_obj_add_event_cb(btn_unit, unit_toggle_btn_cb, LV_EVENT_CLICKED, NULL);
    t0_unit_btn_lbl = lv_label_create(btn_unit);
    lv_label_set_text(t0_unit_btn_lbl, current_settings.use_imperial_units ? "Units: IMP (ft/in)" : "Units: SI (m/cm)");
    lv_obj_center(t0_unit_btn_lbl);

    lv_obj_t *btn_goto_survey = lv_button_create(tile_guidance);
    lv_obj_set_size(btn_goto_survey, 145, 38);
    lv_obj_align(btn_goto_survey, LV_ALIGN_BOTTOM_RIGHT, -8, -8);
    lv_obj_set_style_bg_color(btn_goto_survey, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_add_event_cb(btn_goto_survey, goto_survey_tile_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl_goto = lv_label_create(btn_goto_survey);
    lv_label_set_text(lbl_goto, "Survey Log ->");
    lv_obj_center(lbl_goto);

    // =====================================================================
    // TILE 1: Field Survey / Waypoint Logger (BOOT Short-Press Active Here)
    // =====================================================================
    lv_obj_t *t1_title = lv_label_create(tile_survey);
    lv_label_set_text(t1_title, "FIELD SURVEY LOGGER (BOOT Key Active)");
    lv_obj_set_style_text_color(t1_title, lv_color_hex(0x00FF88), 0);
    lv_obj_set_style_text_font(t1_title, &lv_font_montserrat_12, 0);
    lv_obj_align(t1_title, LV_ALIGN_TOP_MID, 0, 6);

    t1_live_pos_lbl = lv_label_create(tile_survey);
    lv_label_set_text(t1_live_pos_lbl, "Tip Lat: 0.00000000\nTip Lon: 0.00000000\nGround Hgt: 0.000 m");
    lv_obj_set_style_text_color(t1_live_pos_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(t1_live_pos_lbl, &lv_font_montserrat_14, 0);
    lv_obj_align(t1_live_pos_lbl, LV_ALIGN_TOP_LEFT, 10, 26);

    t1_next_lbl = lv_label_create(tile_survey);
    lv_label_set_text(t1_next_lbl, "Next ID: PIN_NW_01_FND");
    lv_obj_set_style_text_color(t1_next_lbl, lv_color_hex(0xFFCC00), 0);
    lv_obj_set_style_text_font(t1_next_lbl, &lv_font_montserrat_16, 0);
    lv_obj_align(t1_next_lbl, LV_ALIGN_TOP_LEFT, 10, 92);

    // Target Match Switch
    lv_obj_t *match_lbl = lv_label_create(tile_survey);
    lv_label_set_text(match_lbl, "Match Active Target Name:");
    lv_obj_set_style_text_color(match_lbl, lv_color_hex(0xCCCCCC), 0);
    lv_obj_align(match_lbl, LV_ALIGN_TOP_LEFT, 10, 124);

    t1_match_sw = lv_switch_create(tile_survey);
    lv_obj_align(t1_match_sw, LV_ALIGN_TOP_RIGHT, -12, 118);
    if (g_survey_target_match_mode) lv_obj_add_state(t1_match_sw, LV_STATE_CHECKED);
    lv_obj_add_event_cb(t1_match_sw, match_target_sw_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // Prefix Dropdown & Averaging Button
    t1_prefix_dd = lv_dropdown_create(tile_survey);
    lv_dropdown_set_options(t1_prefix_dd, waypoint_get_prefix_options());
    lv_obj_set_size(t1_prefix_dd, 155, 36);
    lv_obj_align(t1_prefix_dd, LV_ALIGN_TOP_LEFT, 10, 156);
    lv_obj_add_event_cb(t1_prefix_dd, prefix_dd_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *btn_avg = lv_button_create(tile_survey);
    lv_obj_set_size(btn_avg, 130, 36);
    lv_obj_align(btn_avg, LV_ALIGN_TOP_RIGHT, -10, 156);
    lv_obj_add_event_cb(btn_avg, avg_dur_btn_cb, LV_EVENT_CLICKED, NULL);
    t1_avg_btn_lbl = lv_label_create(btn_avg);
    lv_label_set_text_fmt(t1_avg_btn_lbl, "Avg: %us", current_settings.survey_avg_seconds);
    lv_obj_center(t1_avg_btn_lbl);

    // Quick Note Tag Dropdown
    t1_note_dd = lv_dropdown_create(tile_survey);
    lv_dropdown_set_options(t1_note_dd, waypoint_get_note_tag_options());
    lv_obj_set_size(t1_note_dd, 300, 36);
    lv_obj_align(t1_note_dd, LV_ALIGN_TOP_MID, 0, 202);
    lv_obj_add_event_cb(t1_note_dd, note_dd_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // Progress bar for multi-epoch averaging
    t1_prog_bar = lv_bar_create(tile_survey);
    lv_obj_set_size(t1_prog_bar, 300, 18);
    lv_obj_align(t1_prog_bar, LV_ALIGN_TOP_MID, 0, 250);
    lv_bar_set_range(t1_prog_bar, 0, 100);
    lv_bar_set_value(t1_prog_bar, 0, LV_ANIM_OFF);

    // Large LOG WAYPOINT Button
    t1_log_btn = lv_button_create(tile_survey);
    lv_obj_set_size(t1_log_btn, 300, 56);
    lv_obj_align(t1_log_btn, LV_ALIGN_TOP_MID, 0, 278);
    lv_obj_set_style_bg_color(t1_log_btn, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_add_event_cb(t1_log_btn, log_pin_btn_cb, LV_EVENT_CLICKED, NULL);
    t1_log_btn_lbl = lv_label_create(t1_log_btn);
    lv_label_set_text(t1_log_btn_lbl, "LOG PIN NOW (or Short-Press BOOT)");
    lv_obj_center(t1_log_btn_lbl);

    t1_status_lbl = lv_label_create(tile_survey);
    lv_label_set_text(t1_status_lbl, "Last Logged: None\nSaved to /survey_logged_pins.csv");
    lv_obj_set_style_text_color(t1_status_lbl, lv_color_hex(0x99A3B0), 0);
    lv_obj_set_style_text_font(t1_status_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t1_status_lbl, LV_ALIGN_BOTTOM_LEFT, 10, -10);

    // =====================================================================
    // TILE 2: Waypoint CSV Browser
    // =====================================================================
    t2_file_lbl = lv_label_create(tile_csv);
    lv_label_set_text_fmt(t2_file_lbl, "CSV File: %s (Tap row to guide)", current_settings.active_waypoint_file);
    lv_obj_set_style_text_color(t2_file_lbl, lv_color_hex(0x00D8FF), 0);
    lv_obj_set_style_text_font(t2_file_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t2_file_lbl, LV_ALIGN_TOP_LEFT, 6, 6);

    t2_wp_table = lv_table_create(tile_csv);
    lv_obj_set_size(t2_wp_table, 310, 390);
    lv_obj_align(t2_wp_table, LV_ALIGN_BOTTOM_MID, 0, -6);
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
    lv_obj_align(t3_title, LV_ALIGN_TOP_MID, 0, 6);

    t3_hdg_lbl = lv_label_create(tile_compass);
    lv_label_set_text(t3_hdg_lbl, "Heading: ---.- deg");
    lv_obj_set_style_text_color(t3_hdg_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(t3_hdg_lbl, &lv_font_montserrat_24, 0);
    lv_obj_align(t3_hdg_lbl, LV_ALIGN_TOP_MID, 0, 30);

    t3_base_lbl = lv_label_create(tile_compass);
    lv_label_set_text(t3_base_lbl, "Baseline: 0.555 m | Pitch: 0.0 deg");
    lv_obj_set_style_text_color(t3_base_lbl, lv_color_hex(0x00FFAA), 0);
    lv_obj_set_style_text_font(t3_base_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t3_base_lbl, LV_ALIGN_TOP_MID, 0, 62);

    t3_pose_lbl = lv_label_create(tile_compass);
    lv_label_set_text(t3_pose_lbl, "Display Pose: Held Flat");
    lv_obj_set_style_text_color(t3_pose_lbl, lv_color_hex(0x00D8FF), 0);
    lv_obj_set_style_text_font(t3_pose_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t3_pose_lbl, LV_ALIGN_TOP_MID, 0, 82);

    // 2D Pole Plumb Bubble Level
    t3_level_circle = lv_obj_create(tile_compass);
    lv_obj_set_size(t3_level_circle, 190, 190);
    lv_obj_align(t3_level_circle, LV_ALIGN_TOP_MID, 0, 108);
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
    lv_obj_align(t3_tilt_lbl, LV_ALIGN_TOP_MID, 0, 310);

    lv_obj_t *btn_tare_lvl = lv_button_create(tile_compass);
    lv_obj_set_size(btn_tare_lvl, 260, 44);
    lv_obj_align(btn_tare_lvl, LV_ALIGN_BOTTOM_MID, 0, -14);
    lv_obj_add_event_cb(btn_tare_lvl, tare_level_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl_tare = lv_label_create(btn_tare_lvl);
    lv_label_set_text(lbl_tare, "Tare Pole Bracket Angle (Zero Level)");
    lv_obj_center(lbl_tare);

    // =====================================================================
    // TILE 4: Hardware Diagnostics & Test Modes (UM982 COM1/COM2 & E220)
    // =====================================================================
    lv_obj_t *t4_title = lv_label_create(tile_test);
    lv_label_set_text(t4_title, "HARDWARE TEST MODES (" ROVER_FIRMWARE_VERSION ")");
    lv_obj_set_style_text_color(t4_title, lv_color_hex(0x00D8FF), 0);
    lv_obj_set_style_text_font(t4_title, &lv_font_montserrat_12, 0);
    lv_obj_align(t4_title, LV_ALIGN_TOP_MID, 0, 4);

    // Section 1: UM982 COM1 (IO17/18)
    t4_com1_lbl = lv_label_create(tile_test);
    lv_obj_set_width(t4_com1_lbl, 304);
    lv_label_set_long_mode(t4_com1_lbl, LV_LABEL_LONG_CLIP);
    lv_label_set_text(t4_com1_lbl, "[UM982 COM1 IO17/18 @ 115200]\nRX: 0 B | TX: 0 B | NMEA: 0 | HDG: 0\nLast: Waiting...");
    lv_obj_set_style_text_color(t4_com1_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(t4_com1_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t4_com1_lbl, LV_ALIGN_TOP_LEFT, 8, 24);

    lv_obj_t *btn_c1_ver = lv_button_create(tile_test);
    lv_obj_set_size(btn_c1_ver, 145, 32);
    lv_obj_align(btn_c1_ver, LV_ALIGN_TOP_LEFT, 8, 80);
    lv_obj_add_event_cb(btn_c1_ver, test_com1_ver_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l_c1_ver = lv_label_create(btn_c1_ver);
    lv_label_set_text(l_c1_ver, "COM1: VERSIONA");
    lv_obj_center(l_c1_ver);

    lv_obj_t *btn_c1_cfg = lv_button_create(tile_test);
    lv_obj_set_size(btn_c1_cfg, 145, 32);
    lv_obj_align(btn_c1_cfg, LV_ALIGN_TOP_RIGHT, -8, 80);
    lv_obj_add_event_cb(btn_c1_cfg, test_com1_cfg_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l_c1_cfg = lv_label_create(btn_c1_cfg);
    lv_label_set_text(l_c1_cfg, "COM1: CONFIG");
    lv_obj_center(l_c1_cfg);

    // Section 2: UM982 COM2 (IO47/48)
    t4_com2_lbl = lv_label_create(tile_test);
    lv_label_set_text(t4_com2_lbl, "[UM982 COM2 IO47/48 @ 115200]\nRTCM Forwarded TX: 0 B | Diff Age: -- s");
    lv_obj_set_style_text_color(t4_com2_lbl, lv_color_hex(0x00FFAA), 0);
    lv_obj_set_style_text_font(t4_com2_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t4_com2_lbl, LV_ALIGN_TOP_LEFT, 8, 122);

    lv_obj_t *btn_c2_test = lv_button_create(tile_test);
    lv_obj_set_size(btn_c2_test, 304, 32);
    lv_obj_align(btn_c2_test, LV_ALIGN_TOP_MID, 0, 160);
    lv_obj_add_event_cb(btn_c2_test, test_com2_pulse_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l_c2_test = lv_label_create(btn_c2_test);
    lv_label_set_text(l_c2_test, "Send Test Pulse on UM982 COM2 (IO48)");
    lv_obj_center(l_c2_test);

    // Section 3: E220 LoRa (IO41/42, AUX:21, M0:39, M1:40, RST:38)
    t4_e220_lbl = lv_label_create(tile_test);
    lv_label_set_text_fmt(t4_e220_lbl,
                          "[E220 LoRa IO41/42 @ %lu]\nAUX:1 M0:0 M1:0 | RX: 0 B | RTCM OK: 0\nLast Msg: -- | 1005: 0 | MSM: 0",
                          (unsigned long)current_settings.e220_uart_baud);
    lv_obj_set_style_text_color(t4_e220_lbl, lv_color_hex(0xFFCC00), 0);
    lv_obj_set_style_text_font(t4_e220_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t4_e220_lbl, LV_ALIGN_TOP_LEFT, 8, 202);

    t4_e220_cfg_lbl = lv_label_create(tile_test);
    lv_label_set_text_fmt(t4_e220_cfg_lbl,
                          "Target: CH%u %lu/%.1fk %uB %udBm",
                          current_settings.e220_channel,
                          (unsigned long)current_settings.e220_uart_baud,
                          current_settings.e220_air_rate_kbps,
                          current_settings.e220_subpacket_bytes,
                          current_settings.e220_tx_power_dbm);
    lv_obj_set_style_text_color(t4_e220_cfg_lbl, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_text_font(t4_e220_cfg_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t4_e220_cfg_lbl, LV_ALIGN_TOP_LEFT, 8, 256);

    lv_obj_t *btn_e220_cfg = lv_button_create(tile_test);
    lv_obj_set_size(btn_e220_cfg, 145, 38);
    lv_obj_align(btn_e220_cfg, LV_ALIGN_TOP_LEFT, 8, 282);
    lv_obj_add_event_cb(btn_e220_cfg, test_e220_read_btn_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *l_e220_cfg = lv_label_create(btn_e220_cfg);
    lv_label_set_text(l_e220_cfg, "Read E220 Regs");
    lv_obj_center(l_e220_cfg);

    lv_obj_t *btn_e220_wr = lv_button_create(tile_test);
    lv_obj_set_size(btn_e220_wr, 145, 38);
    lv_obj_align(btn_e220_wr, LV_ALIGN_TOP_RIGHT, -8, 282);
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

    // 1. Update Persistent Header Bar (Date, Time, Units, Battery, Fix, RTCM)
    RtcDateTime_t lt;
    get_local_datetime(&lt);
    lv_label_set_text_fmt(hdr_datetime_lbl, "%04u-%02u-%02u %02u:%02u:%02u %s",
                          lt.year, lt.month, lt.day, lt.hour, lt.minute, lt.second,
                          current_settings.timezone_label);

    lv_label_set_text(hdr_units_lbl, current_settings.use_imperial_units ? "IMP" : "SI");

    lv_label_set_text_fmt(hdr_batt_lbl, "%u%%%s %.2fV",
                          d->battery_percent,
                          d->is_charging ? "+" : "",
                          d->battery_voltage);

    const char *fix_str = "NO FIX";
    lv_color_t fix_col  = lv_palette_main(LV_PALETTE_RED);
    if (d->fix_type == GNSS_FIX_RTK_FIXED) {
        fix_str = "RTK FIXED";
        fix_col = lv_palette_main(LV_PALETTE_GREEN);
    } else if (d->fix_type == GNSS_FIX_RTK_FLOAT) {
        fix_str = "RTK FLOAT";
        fix_col = lv_palette_main(LV_PALETTE_ORANGE);
    } else if (d->fix_type == GNSS_FIX_DGPS) {
        fix_str = "DGPS";
        fix_col = lv_palette_main(LV_PALETTE_BLUE);
    } else if (d->fix_type == GNSS_FIX_SINGLE) {
        fix_str = "SINGLE";
        fix_col = lv_palette_main(LV_PALETTE_AMBER);
    }
    lv_label_set_text(hdr_fix_lbl, fix_str);
    lv_obj_set_style_bg_color(hdr_fix_badge, fix_col, 0);

    char acc_str[24];
    nav_format_distance(d->h_acc_m, acc_str, sizeof(acc_str));
    lv_label_set_text_fmt(hdr_sats_acc_lbl, "SV:%u ±%s", d->num_sats, acc_str);

    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    uint32_t rtcm_age_ms = (d->e220_stats.last_rx_ms > 0) ? (now_ms - d->e220_stats.last_rx_ms) : 99999;
    if (rtcm_age_ms < 15000) {
        lv_label_set_text_fmt(hdr_rtcm_lbl, "RTCM:%u.%us", rtcm_age_ms / 1000, (rtcm_age_ms % 1000) / 100);
        lv_obj_set_style_text_color(hdr_rtcm_lbl, lv_color_hex(0x00FF88), 0);
    } else {
        lv_label_set_text(hdr_rtcm_lbl, "RTCM:Wait");
        lv_obj_set_style_text_color(hdr_rtcm_lbl, lv_color_hex(0xFF8800), 0);
    }

    // 2. Update Tile 0: Main Guidance Screen
    char hgt_str[24], dist_str[24], dn_str[24], de_str[24];
    nav_format_height(d->tip_height_m, hgt_str, sizeof(hgt_str));
    nav_format_distance(d->distance_to_target_m, dist_str, sizeof(dist_str));
    nav_format_offset(d->d_north_m, dn_str, sizeof(dn_str));
    nav_format_offset(d->d_east_m, de_str, sizeof(de_str));

    lv_label_set_text_fmt(t0_pos_lbl, "Lat: %12.8f\nLon: %12.8f   Hgt: %s",
                          d->tip_lat, d->tip_lon, hgt_str);

    if (d->has_active_target) {
        lv_label_set_text_fmt(t0_target_lbl, "%s (%d/%d)",
                              d->target_label, d->active_target_idx + 1, d->total_targets);
        lv_label_set_text(t0_notes_lbl, d->target_notes[0] ? d->target_notes : "---");
        lv_label_set_text(t0_dist_lbl, dist_str);

        if (d->heading_valid) {
            lv_label_set_text_fmt(t0_brg_lbl, "Brg: %.1f deg | Hdg: %.1f deg",
                                  d->bearing_to_target_deg, d->true_heading_deg);
        } else {
            lv_label_set_text_fmt(t0_brg_lbl, "Brg: %.1f deg | Hdg: No Lock",
                                  d->bearing_to_target_deg);
        }
        lv_label_set_text_fmt(t0_offsets_lbl, "dN: %s   dE: %s", dn_str, de_str);

        // Move target dot inside the 180x180 radar circle (max radius 74px)
        float scale_m = (d->distance_to_target_m < 2.0) ? 2.0f : 20.0f;
        float norm_r  = (float)d->distance_to_target_m / scale_m;
        if (norm_r > 1.0f) norm_r = 1.0f;
        float angle_rad = (d->heading_valid ? d->rel_bearing_deg : d->bearing_to_target_deg) * (float)(M_PI / 180.0);
        int16_t px = (int16_t)(norm_r * 72.0f * sinf(angle_rad));
        int16_t py = (int16_t)(-norm_r * 72.0f * cosf(angle_rad));
        lv_obj_align(t0_target_dot, LV_ALIGN_CENTER, px, py);
    }

    // 3. Update Tile 1: Field Survey Logger
    lv_label_set_text_fmt(t1_live_pos_lbl,
                          "Tip Lat: %.8f\nTip Lon: %.8f\nGround Hgt: %s (%s)",
                          d->tip_lat, d->tip_lon, hgt_str, fix_str);

    char next_id[WAYPOINT_LABEL_LEN];
    waypoint_format_next_survey_label(next_id, sizeof(next_id));
    lv_label_set_text_fmt(t1_next_lbl, "Next ID: %s", next_id);

    lv_bar_set_value(t1_prog_bar, d->survey_progress_pct, LV_ANIM_OFF);
    if (d->survey_in_progress) {
        lv_label_set_text_fmt(t1_log_btn_lbl, "AVERAGING RTK... %u%% (%u pts)",
                              d->survey_progress_pct, d->survey_epochs_collected);
        lv_obj_set_style_bg_color(t1_log_btn, lv_palette_main(LV_PALETTE_ORANGE), 0);
    } else {
        lv_label_set_text(t1_log_btn_lbl, "LOG PIN NOW (or Short-Press BOOT)");
        lv_obj_set_style_bg_color(t1_log_btn, lv_palette_main(LV_PALETTE_GREEN), 0);
    }
    lv_label_set_text_fmt(t1_status_lbl, "Last Logged: %s\nFile: /survey_logged_pins.csv",
                          d->last_logged_label);

    // 4. Update Tile 3: Compass & Bubble Level
    if (d->heading_valid) {
        lv_label_set_text_fmt(t3_hdg_lbl, "Heading: %05.1f deg", d->true_heading_deg);
    } else {
        lv_label_set_text(t3_hdg_lbl, "Heading: ---.- deg");
    }
    lv_label_set_text_fmt(t3_base_lbl, "Baseline: %.3f m | Pitch: %+.1f deg",
                          d->baseline_length_m > 0.01f ? d->baseline_length_m : current_settings.antenna_spacing_m,
                          d->baseline_pitch_deg);

    const char *pose_str = "Held Flat";
    if (d->display_pose == DISPLAY_POSE_ANGLED) pose_str = "Handheld Viewing Angle";
    else if (d->display_pose == DISPLAY_POSE_VERTICAL) pose_str = "Clipped Vertically to Pole";
    lv_label_set_text_fmt(t3_pose_lbl, "Display Pose: %s", pose_str);

    int16_t bx = (int16_t)(d->pole_bubble_x_deg * 6.0f);
    int16_t by = (int16_t)(d->pole_bubble_y_deg * 6.0f);
    if (bx > 70) bx = 70; if (bx < -70) bx = -70;
    if (by > 70) by = 70; if (by < -70) by = -70;
    lv_obj_align(t3_bubble_dot, LV_ALIGN_CENTER, bx, by);
    lv_label_set_text_fmt(t3_tilt_lbl, "Pole Tilt: dX: %+.1f deg | dY: %+.1f deg",
                          d->pole_bubble_x_deg, d->pole_bubble_y_deg);

    // 5. Update Tile 4: Hardware Test Modes
    lv_label_set_text_fmt(t4_com1_lbl,
                          "[UM982 COM1 IO17/18 @ 115200]\nRX:%lu B | TX:%lu B | NMEA:%lu | HDG:%lu\nLast: %s",
                          (unsigned long)d->um982_stats.com1_rx_bytes,
                          (unsigned long)d->um982_stats.com1_tx_bytes,
                          (unsigned long)d->um982_stats.nmea_sentences_ok,
                          (unsigned long)d->um982_stats.heading_sentences_ok,
                          d->um982_stats.last_com1_line);

    lv_label_set_text_fmt(t4_com2_lbl,
                          "[UM982 COM2 IO47/48 @ 115200]\nRTCM Forwarded TX: %lu B | Diff Age: %.1f s",
                          (unsigned long)d->um982_stats.com2_tx_bytes,
                          d->diff_age_s);

    lv_label_set_text_fmt(t4_e220_lbl,
                          "[E220 LoRa IO41/42 @ %lu]\nAUX:%d M0:%u M1:%u | RX:%lu B | RTCM OK:%lu\nLast Msg:%u | 1005:%u | MSM:%u",
                          (unsigned long)current_settings.e220_uart_baud,
                          d->e220_stats.aux_pin_high ? 1 : 0,
                          d->e220_stats.mode_m0,
                          d->e220_stats.mode_m1,
                          (unsigned long)d->e220_stats.rx_bytes_total,
                          (unsigned long)d->e220_stats.rtcm_frames_ok,
                          d->e220_stats.last_msg_type,
                          d->e220_stats.msg_1005_count,
                          d->e220_stats.msg_msm_count);

    if (d->e220_stats.config_valid) {
        lv_label_set_text_fmt(t4_e220_cfg_lbl,
                              "E220 Regs: ADDH:%02X ADDL:%02X REG0:%02X REG1:%02X CH:%02X REG3:%02X",
                              d->e220_stats.cfg_addr_high,
                              d->e220_stats.cfg_addr_low,
                              d->e220_stats.cfg_reg0,
                              d->e220_stats.cfg_reg1,
                              d->e220_stats.cfg_channel,
                              d->e220_stats.cfg_reg3);
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
