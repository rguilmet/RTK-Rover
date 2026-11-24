#include "lvgl_ui_internal.h"
#include "esp_timer.h"
#include "../e220_driver/e220_driver.h"

// =========================================================================
// Persistent Top Header Bar Widgets (Visible on ALL Screens)
// =========================================================================
// Row 1: System & Radios
static lv_obj_t *hdr_datetime_lbl = NULL;
lv_obj_t        *hdr_units_lbl    = NULL;
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
static lv_obj_t *tile_settings  = NULL;
static lv_obj_t *nav_screen_lbl = NULL;
static lv_obj_t *auto_off_modal     = NULL;
static lv_obj_t *auto_off_modal_lbl = NULL;
volatile uint8_t active_tile_col = 0;

RoverUIData_t s_last_ui_data = {0};
bool s_has_last_ui_data = false;

static const char *kScreenTitles[6] = {
    "1/6: Pin Seeker",
    "2/6: Field Survey",
    "3/6: CSV Waypoints",
    "4/6: Compass & Level",
    "5/6: Diagnostics",
    "6/6: Quick Settings"
};

uint8_t rover_get_active_tile_index(void)
{
    return active_tile_col;
}

// =========================================================================
// UI Event Callbacks
// =========================================================================
static void update_nav_bar_label(void)
{
    if (nav_screen_lbl && active_tile_col < 6) {
        lv_label_set_text(nav_screen_lbl, kScreenTitles[active_tile_col]);
    }
}

void switch_to_tile_index(uint8_t idx, lv_anim_enable_t anim)
{
    if (!main_tv || idx >= 6) return;
    lv_obj_t *tiles[6] = {tile_guidance, tile_survey, tile_csv, tile_compass, tile_test, tile_settings};
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
    if (act == tile_guidance)      active_tile_col = 0;
    else if (act == tile_survey)   active_tile_col = 1;
    else if (act == tile_csv)      active_tile_col = 2;
    else if (act == tile_compass)  active_tile_col = 3;
    else if (act == tile_test)     active_tile_col = 4;
    else if (act == tile_settings) active_tile_col = 5;
    update_nav_bar_label();
}

static void nav_prev_screen_btn_cb(lv_event_t *e)
{
    (void)e;
    play_rover_beep(1200, 20);
    uint8_t next_idx = (active_tile_col == 0) ? 5 : (active_tile_col - 1);
    switch_to_tile_index(next_idx, LV_ANIM_ON);
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] '< Prev' screen clicked -> Tile %u (%s)\n", next_idx + 1, kScreenTitles[next_idx]);
    }
}

static void nav_next_screen_btn_cb(lv_event_t *e)
{
    (void)e;
    play_rover_beep(1200, 20);
    uint8_t next_idx = (active_tile_col + 1) % 6;
    switch_to_tile_index(next_idx, LV_ANIM_ON);
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] 'Next >' screen clicked -> Tile %u (%s)\n", next_idx + 1, kScreenTitles[next_idx]);
    }
}

static void auto_off_cancel_cb(lv_event_t *e)
{
    (void)e;
    rover_note_user_activity();
    play_rover_beep(1800, 40);
    if (auto_off_modal) {
        lv_obj_set_hidden(auto_off_modal, true);
    }
    if (current_settings.enable_touch_logging) {
        rover_printf("[POWER] Auto Power-Off countdown cancelled by user tap.\n");
    }
}

static const char *get_heading_source_tag(const RoverUIData_t *d)
{
    if (d->heading_source == HDG_SRC_DUAL_FIX) return "FIX";
    if (d->heading_source == HDG_SRC_DUAL_FLT) return "FLT";
    if (d->heading_source == HDG_SRC_DISP)     return "WLK";
    if (d->heading_source == HDG_SRC_COMP)     return "CMP";
    if (d->heading_source == HDG_SRC_GYRO)     return "GYR";
    if (d->heading_source == HDG_SRC_COG)      return "COG";
    if (d->heading_source == HDG_SRC_HOLD)     return "HLD";
    return d->heading_is_fixed ? "FIX" : "FLT";
}

void create_rover_ui(void)
{
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x121417), LV_PART_MAIN);
    lv_obj_set_scrollbar_mode(scr, LV_SCROLLBAR_MODE_OFF);

    // =====================================================================
    // 1. Persistent Top Status Header Bar (320 x 74, always visible across all 6 tiles)
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
    lv_obj_align(hdr_units_lbl, LV_ALIGN_TOP_RIGHT, -145, 2);

    hdr_wifi_lbl = lv_label_create(hdr);
    lv_label_set_text(hdr_wifi_lbl, LV_SYMBOL_WIFI);
    lv_obj_set_style_text_color(hdr_wifi_lbl, lv_color_hex(0x475569), 0);
    lv_obj_set_style_text_font(hdr_wifi_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(hdr_wifi_lbl, LV_ALIGN_TOP_RIGHT, -118, 2);

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
    tile_settings = lv_tileview_add_tile(main_tv, 5, 0, LV_DIR_NONE);

    // Create all screen tiles in symmetric 6-tile ring order
    ui_tile0_create(tile_guidance);
    ui_tile1_survey_create(tile_survey);
    ui_tile2_waypoints_create(tile_csv);
    ui_tile3_compass_create(tile_compass);
    ui_tile4_diagnostics_create(tile_test);
    ui_tile5_settings_create(tile_settings);

    // 4. Top-Layer 30-Second Auto Power-Off Warning Modal (hidden until active)
    auto_off_modal = lv_obj_create(lv_layer_top());
    lv_obj_set_size(auto_off_modal, ROVER_LCD_H_RES, ROVER_LCD_V_RES);
    lv_obj_align(auto_off_modal, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(auto_off_modal, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(auto_off_modal, LV_OPA_70, 0);
    lv_obj_set_style_border_width(auto_off_modal, 0, 0);
    lv_obj_set_style_radius(auto_off_modal, 0, 0);
    lv_obj_set_style_pad_all(auto_off_modal, 0, 0);
    lv_obj_set_scrollbar_mode(auto_off_modal, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scrollable(auto_off_modal, false);
    lv_obj_set_clickable(auto_off_modal, true);
    lv_obj_add_event_cb(auto_off_modal, auto_off_cancel_cb, LV_EVENT_PRESSED, NULL);

    lv_obj_t *ao_card = lv_obj_create(auto_off_modal);
    lv_obj_set_size(ao_card, 296, 124);
    lv_obj_center(ao_card);
    lv_obj_set_style_bg_color(ao_card, lv_color_hex(0x7F1D1D), 0);
    lv_obj_set_style_border_color(ao_card, lv_color_hex(0xF59E0B), 0);
    lv_obj_set_style_border_width(ao_card, 3, 0);
    lv_obj_set_style_radius(ao_card, 10, 0);
    lv_obj_set_style_pad_all(ao_card, 10, 0);
    lv_obj_set_scrollbar_mode(ao_card, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scrollable(ao_card, false);
    lv_obj_set_clickable(ao_card, true);
    lv_obj_add_event_cb(ao_card, auto_off_cancel_cb, LV_EVENT_PRESSED, NULL);

    auto_off_modal_lbl = lv_label_create(ao_card);
    lv_obj_set_width(auto_off_modal_lbl, 272);
    lv_label_set_long_mode(auto_off_modal_lbl, LV_LABEL_LONG_WRAP);
    lv_label_set_text(auto_off_modal_lbl, "AUTO POWER-OFF IN 30s\n\nTAP ANYWHERE TO CANCEL");
    lv_obj_set_style_text_color(auto_off_modal_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(auto_off_modal_lbl, &lv_font_montserrat_16, 0);
    lv_obj_set_style_text_align(auto_off_modal_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(auto_off_modal_lbl);

    lv_obj_set_hidden(auto_off_modal, true);
}

void update_rover_ui(const RoverUIData_t *d)
{
    if (!d) return;
    if (!rover_lvgl_lock(100)) return;

    s_last_ui_data = *d;
    s_has_last_ui_data = true;

    char buf[512];
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);

    // 1. Update Persistent Header Bar (Date, Time, Units, Battery, Fix, RTCM)
    FicRtcTime_t lt;
    fic_rtc_get_local_datetime(&lt);
    snprintf(buf, sizeof(buf), "%04u-%02u-%02u %02u:%02u:%02u %s",
             lt.year, lt.month, lt.day, lt.hour, lt.minute, lt.second,
             current_settings.timezone_label);
    lv_label_set_text(hdr_datetime_lbl, buf);

    if (d->vbus_present) {
        snprintf(buf, sizeof(buf), "%s %.2fV%s",
                 LV_SYMBOL_USB,
                 d->vbus_voltage,
                 d->is_charging ? "+" : "");
        lv_label_set_text(hdr_batt_lbl, buf);
        lv_obj_set_style_text_color(hdr_batt_lbl, lv_color_hex(0x22D3EE), 0); // Cyan
    } else {
        const char *bat_sym = (d->battery_percent >= 80) ? LV_SYMBOL_BATTERY_FULL :
                              (d->battery_percent >= 55) ? LV_SYMBOL_BATTERY_3 :
                              (d->battery_percent >= 30) ? LV_SYMBOL_BATTERY_2 :
                              (d->battery_percent >= 15) ? LV_SYMBOL_BATTERY_1 :
                                                           LV_SYMBOL_BATTERY_EMPTY;
        snprintf(buf, sizeof(buf), "%s %u%% %.2fV",
                 bat_sym,
                 d->battery_percent,
                 d->battery_voltage);
        lv_label_set_text(hdr_batt_lbl, buf);
        lv_color_t bat_col = (d->battery_percent <= 20) ? lv_palette_main(LV_PALETTE_RED) :
                             (d->battery_percent <= 40) ? lv_palette_main(LV_PALETTE_AMBER) :
                                                          lv_palette_main(LV_PALETTE_GREEN);
        lv_obj_set_style_text_color(hdr_batt_lbl, bat_col, 0);
    }

    // Wi-Fi Status Badge in Row 1 (Icon only: Slate=OFF, Amber=Connecting, Cyan=Router STA, Purple=AP)
    if (hdr_wifi_lbl) {
        lv_label_set_text(hdr_wifi_lbl, LV_SYMBOL_WIFI);
        if (fic_web_portal_is_active()) {
            if (fic_web_portal_is_softap()) {
                lv_obj_set_style_text_color(hdr_wifi_lbl, lv_color_hex(0xA855F7), 0); // Purple / Violet (AP)
            } else {
                lv_obj_set_style_text_color(hdr_wifi_lbl, lv_color_hex(0x22D3EE), 0); // Cyan (Router STA)
            }
        } else if (fic_web_portal_is_starting()) {
            lv_obj_set_style_text_color(hdr_wifi_lbl, lv_color_hex(0xF59E0B), 0);     // Amber (Connecting)
        } else {
            lv_obj_set_style_text_color(hdr_wifi_lbl, lv_color_hex(0x475569), 0);     // Dim slate (OFF)
        }
    }

    // Tile 1 Wi-Fi Status Label (synchronized cross-task copy)
    ui_tile5_sync_wifi_status();

    // Row 2: Master Antenna (ANT1) Fix Badge & Metrics + Live LoRa Status (decoupled from VBUS)
    if (!d->um982_stats.com1_online) {
        lv_label_set_text(hdr_m_fix_lbl, "OFF");
        lv_obj_set_style_bg_color(hdr_m_fix_badge, lv_palette_main(LV_PALETTE_RED), 0);
        lv_label_set_text(hdr_m_sats_lbl, "SV:--");
        lv_label_set_text(hdr_m_acc_lbl, "+/- ---");
        lv_obj_set_style_text_color(hdr_m_acc_lbl, lv_color_hex(0x64748B), 0);
    } else {
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
    }

    now_ms = (uint32_t)(esp_timer_get_time() / 1000ULL);
    uint32_t last_radio_rx_age = (d->e220_stats.last_rx_ms > 0) ? (now_ms - d->e220_stats.last_rx_ms) : 99999;

    // Header Row 2: LoRa Signal Strength (Numeric "LoRa: -104dB", "LoRa: -- dB", or "LoRa: OFF")
    if (!d->e220_stats.radio_online) {
        lv_label_set_text(hdr_m_rtcm_lbl, "LoRa: OFF");
        lv_obj_set_style_text_color(hdr_m_rtcm_lbl, lv_color_hex(0x64748B), 0);
    } else if (d->e220_stats.rssi_valid && d->e220_stats.rx_bytes_total > 0) {
        bool is_stale = (last_radio_rx_age > 6000);
        snprintf(buf, sizeof(buf), "LoRa: %ddB", (int)d->e220_stats.last_pkt_rssi_dbm);
        lv_label_set_text(hdr_m_rtcm_lbl, buf);
        lv_obj_set_style_text_color(hdr_m_rtcm_lbl, get_lora_rssi_color(d->e220_stats.last_pkt_rssi_dbm, is_stale), 0);
    } else {
        lv_label_set_text(hdr_m_rtcm_lbl, "LoRa: -- dB");
        lv_obj_set_style_text_color(hdr_m_rtcm_lbl, lv_color_hex(0x64748B), 0);
    }

    // Row 3: Dual-Antenna Heading Status & Metrics (decoupled from VBUS)
    if (!d->um982_stats.com1_online) {
        if (hdr_s_fix_lbl) lv_label_set_text(hdr_s_fix_lbl, "OFF");
        if (hdr_s_fix_badge) lv_obj_set_style_bg_color(hdr_s_fix_badge, lv_palette_main(LV_PALETTE_RED), 0);
        lv_label_set_text(hdr_s_sats_lbl, "SV:--");
        lv_label_set_text(hdr_s_bl_lbl, "BL:--.-cm");
        if (d->heading_valid) {
            snprintf(buf, sizeof(buf), "Hdg:%.1f %s", d->true_heading_deg, get_heading_source_tag(d));
            lv_label_set_text(hdr_s_hdg_lbl, buf);
        } else {
            lv_label_set_text(hdr_s_hdg_lbl, "Hdg:---.-");
        }
    } else {
        const char *hdg_badge_str = "HDG ---";
        lv_color_t hdg_badge_col  = lv_color_hex(0x3A3F47);
        if (d->heading_valid) {
            if (d->heading_source == HDG_SRC_DUAL_FIX) {
                hdg_badge_str = "SLV FIX";
                hdg_badge_col = lv_palette_main(LV_PALETTE_GREEN);
            } else if (d->heading_source == HDG_SRC_DUAL_FLT) {
                hdg_badge_str = "SLV FLT";
                hdg_badge_col = lv_palette_main(LV_PALETTE_CYAN);
            } else if (d->heading_source == HDG_SRC_DISP) {
                hdg_badge_str = "HDG WALK";
                hdg_badge_col = lv_palette_main(LV_PALETTE_AMBER);
            } else if (d->heading_source == HDG_SRC_COMP) {
                hdg_badge_str = "HDG CMP";
                hdg_badge_col = lv_color_hex(0x14B8A6); // Vivid Teal for 9-DOF MMC5983MA Compass
            } else if (d->heading_source == HDG_SRC_GYRO) {
                hdg_badge_str = "HDG GYRO";
                hdg_badge_col = lv_palette_main(LV_PALETTE_AMBER);
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
                                  (d->heading_source == HDG_SRC_DISP)     ? "WLK" :
                                  (d->heading_source == HDG_SRC_COMP)     ? "CMP" :
                                  (d->heading_source == HDG_SRC_GYRO)     ? "GYR" :
                                  (d->heading_source == HDG_SRC_COG)      ? "COG" :
                                  (d->heading_source == HDG_SRC_HOLD)     ? "HLD" :
                                  (d->heading_is_fixed ? "FIX" : "FLT");
            snprintf(buf, sizeof(buf), "Hdg:%.1f %s", d->true_heading_deg, src_tag);
        } else {
            snprintf(buf, sizeof(buf), "Hdg:---.-");
        }
        lv_label_set_text(hdr_s_hdg_lbl, buf);
    }

    // Row 4: Streamlined Coordinates Bar (7 decimals in Montserrat 16 for high visibility)
    char hgt_str[24];
    nav_format_height(d->tip_height_m, hgt_str, sizeof(hgt_str));

    snprintf(buf, sizeof(buf), "%.7f, %.7f, %s", d->tip_lat, d->tip_lon, hgt_str);
    if (hdr_coords_lbl) lv_label_set_text(hdr_coords_lbl, buf);

    // Push position into breadcrumb buffer
    nav_breadcrumb_push(d->tip_lat, d->tip_lon);

    // Update all control buttons (Colors & Labels)
    update_all_control_buttons();

    // Update Tile 0, Tile 1, Tile 3, Tile 4, and Tile 5
    ui_tile0_update(d, buf, sizeof(buf));
    ui_tile1_survey_update(d, buf, sizeof(buf));
    ui_tile3_compass_update(d, buf, sizeof(buf));
    ui_tile4_diagnostics_update(d, now_ms, buf, sizeof(buf));
    ui_tile5_settings_update(d, buf, sizeof(buf));

    if (auto_off_modal && auto_off_modal_lbl) {
        if (d->auto_off_countdown_s > 0) {
            snprintf(buf, sizeof(buf), "AUTO POWER-OFF IN %us\n\nTAP ANYWHERE TO CANCEL",
                     (unsigned int)d->auto_off_countdown_s);
            lv_label_set_text(auto_off_modal_lbl, buf);
            lv_obj_set_hidden(auto_off_modal, false);
        } else {
            lv_obj_set_hidden(auto_off_modal, true);
        }
    }

    rover_lvgl_unlock();
}
