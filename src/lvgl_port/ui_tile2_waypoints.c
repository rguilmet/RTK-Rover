#include "lvgl_ui_internal.h"

// =========================================================================
// Tile 2: Waypoint CSV Browser Widgets
// =========================================================================
static lv_obj_t *t2_file_lbl      = NULL;
static lv_obj_t *t2_wp_table      = NULL;

static void prev_file_btn_cb(lv_event_t *e)
{
    (void)e;
    play_rover_beep(1500, 20);
    waypoint_file_select_prev();
    ui_tile0_map_on_file_changed();
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
    ui_tile0_map_on_file_changed();
    update_t0_target_label();
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] File '>' clicked -> %s\n", waypoint_file_get_active_name());
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

void ui_tile2_waypoints_create(lv_obj_t *parent)
{
    // =====================================================================
    // TILE 2: Waypoint CSV Browser (Tile 3 to user)
    // Fixed Top File Selector Row: [ < ] waypoints.csv (1/1) [ > ]
    // =====================================================================
    lv_obj_t *btn_file_prev = lv_button_create(parent);
    lv_obj_set_size(btn_file_prev, 44, 28);
    lv_obj_align(btn_file_prev, LV_ALIGN_TOP_LEFT, 6, 3);
    lv_obj_set_ext_click_area(btn_file_prev, 12);
    lv_obj_add_event_cb(btn_file_prev, prev_file_btn_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_t *lbl_fprev = lv_label_create(btn_file_prev);
    lv_label_set_text(lbl_fprev, LV_SYMBOL_LEFT);
    lv_obj_center(lbl_fprev);

    t2_file_lbl = lv_label_create(parent);
    lv_obj_set_style_text_color(t2_file_lbl, lv_color_hex(0x00D8FF), 0);
    lv_obj_set_style_text_font(t2_file_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_width(t2_file_lbl, 216);
    lv_label_set_long_mode(t2_file_lbl, LV_LABEL_LONG_MODE_SCROLL_CIRCULAR);
    lv_obj_set_style_text_align(t2_file_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(t2_file_lbl, LV_ALIGN_TOP_MID, 0, 8);
    lv_label_set_text(t2_file_lbl, "waypoints.csv");

    lv_obj_t *btn_file_next = lv_button_create(parent);
    lv_obj_set_size(btn_file_next, 44, 28);
    lv_obj_align(btn_file_next, LV_ALIGN_TOP_RIGHT, -6, 3);
    lv_obj_set_ext_click_area(btn_file_next, 12);
    lv_obj_add_event_cb(btn_file_next, next_file_btn_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_t *lbl_fnext = lv_label_create(btn_file_next);
    lv_label_set_text(lbl_fnext, LV_SYMBOL_RIGHT);
    lv_obj_center(lbl_fnext);

    // Scrollable Waypoint Table Below (y = 36 to y = 370)
    t2_wp_table = lv_table_create(parent);
    lv_obj_set_size(t2_wp_table, 310, 336);
    lv_obj_align(t2_wp_table, LV_ALIGN_TOP_MID, 0, 36);
    lv_obj_set_style_text_font(t2_wp_table, &lv_font_montserrat_12, LV_PART_ITEMS);
    lv_obj_add_event_cb(t2_wp_table, wp_table_event_cb, LV_EVENT_VALUE_CHANGED, NULL);
    refresh_waypoint_table_ui();
}
