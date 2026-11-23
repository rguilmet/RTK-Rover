#ifndef LVGL_UI_INTERNAL_H
#define LVGL_UI_INTERNAL_H

#include <stdlib.h>
#include <stdio.h>
#include <stddef.h>
#include <string.h>
#include <math.h>
#include "lvgl_port.h"
#include "../../user_config.h"
#include "../settings_manager/settings_manager.h"
#include "../nav_engine/nav_engine.h"
#include "../waypoint_manager/waypoint_manager.h"
#include "../storage/serial_logger.h"

#ifdef __cplusplus
extern "C" {
#endif

// =========================================================================
// Shared Capacities & Button Color Palette
// =========================================================================
#define T0_GRID_V_LINES 4
#define T0_MAP_MAX_TIES 8
#define T0_MAP_MAX_PINS 32

#define T0_CANVAS_W       304
#define T0_CANVAS_H       276
#define T0_CANVAS_CX      152
#define T0_CANVAS_CY      138
#define T0_GUIDE_Y_TOP    50
#define T0_GUIDE_Y_BOT    206
#define T0_GUIDE_CY       128

#define COLOR_BTN_ACTIVE_GREEN  lv_color_hex(0x107C41)
#define COLOR_BTN_INACTIVE_BLUE lv_color_hex(0x1B6CA8)
#define COLOR_UNIT_SI_TEAL      lv_color_hex(0x0284C7)
#define COLOR_UNIT_IMP_AMBER    lv_color_hex(0xD97706)
#define COLOR_BTN_WIFI_CONN     lv_color_hex(0xD97706) // Vivid Amber / Orange (Connecting)
#define COLOR_BTN_WIFI_STA      lv_color_hex(0x107C41) // Emerald Green (Local Router STA)
#define COLOR_BTN_WIFI_AP       lv_color_hex(0x7C3AED) // Vivid Violet / Purple (Hotspot SoftAP)

// =========================================================================
// External Firmware Callbacks
// =========================================================================
extern void init_backlight_pwm(void);
extern void set_backlight_pwm(uint8_t brightness);
extern void set_rover_brightness(uint8_t brightness_pct);
extern void set_rover_volume(uint8_t volume_pct);
extern void mark_settings_dirty(void);
extern void play_rover_beep(uint16_t freq_hz, uint16_t duration_ms);

// =========================================================================
// Shared Inline Helpers
// =========================================================================
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

static inline void rover_label_set_text_if_changed(lv_obj_t *lbl, const char *txt)
{
    if (!lbl || !txt) return;
    const char *cur = lv_label_get_text(lbl);
    if (!cur || strcmp(cur, txt) != 0) {
        lv_label_set_text(lbl, txt);
    }
}

// =========================================================================
// Shared Cross-Module Widget Pointers & UI State
// =========================================================================
// Defined in ui_header_nav.c
extern lv_obj_t *hdr_units_lbl;
extern volatile uint8_t active_tile_col;
extern RoverUIData_t s_last_ui_data;
extern bool s_has_last_ui_data;

// Defined in ui_tile0_guidance.c
extern uint8_t s_tile0_nav_mode;
extern lv_obj_t *t0_target_lbl;
extern lv_obj_t *t0_notes_lbl;
extern lv_obj_t *t0_rec_btn;
extern lv_obj_t *t0_rec_btn_lbl;

extern lv_obj_t *t0_nav_canvas;
extern lv_obj_t *t0_mode_badge;
extern lv_obj_t *t0_hdg_hud_lbl;
extern lv_obj_t *t0_tgt_hdg_lbl;
extern lv_obj_t *t0_cross_h;
extern lv_obj_t *t0_cross_v;
extern lv_obj_t *t0_bullseye_ring1;
extern lv_obj_t *t0_bullseye_ring2;
extern lv_obj_t *t0_bullseye_center;
extern lv_obj_t *t0_beam_arrow;
extern lv_obj_t *t0_target_icon;
extern lv_obj_t *t0_rover_icon;
extern lv_obj_t *t0_lbl_pt_a;
extern lv_obj_t *t0_lbl_pt_b;
extern lv_obj_t *t0_lbl_pt_r;
extern lv_obj_t *t0_grid_v[T0_GRID_V_LINES];
extern lv_obj_t *t0_log_lbl_inner;
extern lv_obj_t *t0_log_lbl_outer;
extern lv_obj_t *t0_anchor_btn;
extern lv_obj_t *t0_anchor_btn_lbl;

extern lv_obj_t *t0_chev_left[3];
extern lv_obj_t *t0_chev_right[3];
extern lv_obj_t *t0_ontrack_bar_l;
extern lv_obj_t *t0_ontrack_bar_r;

extern lv_obj_t *t0_dist_lbl;
extern lv_obj_t *t0_offsets_lbl;

extern lv_obj_t *t0_line_baseline;
extern lv_obj_t *t0_line_ext_top;
extern lv_obj_t *t0_line_ext_bot;
extern lv_obj_t *t0_line_tie;
extern lv_obj_t *t0_line_hyp;
extern lv_obj_t *t0_line_arrow;
extern lv_obj_t *t0_line_pt_a;
extern lv_obj_t *t0_line_chord;

// Defined in ui_tile0_map.c
extern lv_obj_t *t0_map_btn_zin;
extern lv_obj_t *t0_map_btn_zout;
extern lv_obj_t *t0_map_btn_fit;
extern lv_obj_t *t0_map_btn_ctr;
extern lv_obj_t *t0_map_boundary;
extern lv_obj_t *t0_map_active_seg;
extern lv_obj_t *t0_map_trail;
extern lv_obj_t *t0_map_ties[T0_MAP_MAX_TIES];
extern lv_obj_t *t0_map_pins[T0_MAP_MAX_PINS];
extern bool s_map_auto_fit;
extern bool s_map_center_rover;

// Defined in ui_tile3_compass.c
extern lv_obj_t *t3_tare_btn;
extern lv_obj_t *t3_tare_lbl;

// =========================================================================
// Internal Cross-Module Functions
// =========================================================================
// ui_header_nav.c
void switch_to_tile_index(uint8_t idx, lv_anim_enable_t anim);

// ui_tile0_guidance.c
void ui_tile0_create(lv_obj_t *parent);
void ui_tile0_update(const RoverUIData_t *d, char *buf, size_t buf_sz);
void update_t0_target_label(void);
void refresh_t0_guidance_readout_immediately(void);

// ui_tile0_views.c
void ui_tile0_views_update_hud(const RoverUIData_t *d);
void ui_tile0_views_update_guidance(const RoverUIData_t *d, char *buf, size_t buf_sz);

// ui_tile0_map.c
void t0_canvas_drag_cb(lv_event_t *e);
void ui_tile0_map_create(lv_obj_t *canvas);
void ui_tile0_map_update(const RoverUIData_t *d, char *buf, size_t buf_sz);
void ui_tile0_map_on_file_changed(void);

// ui_tile1_survey.c
void ui_tile1_survey_create(lv_obj_t *parent);
void ui_tile1_survey_update(const RoverUIData_t *d, char *buf, size_t buf_sz);

// ui_tile2_waypoints.c
void ui_tile2_waypoints_create(lv_obj_t *parent);

// ui_tile3_compass.c
void ui_tile3_compass_create(lv_obj_t *parent);
void ui_tile3_compass_update(const RoverUIData_t *d, char *buf, size_t buf_sz);

// ui_tile4_diagnostics.c
void ui_tile4_diagnostics_create(lv_obj_t *parent);
void ui_tile4_diagnostics_update(const RoverUIData_t *d, uint32_t now_ms, char *buf, size_t buf_sz);

// ui_tile5_settings.c
void ui_tile5_settings_create(lv_obj_t *parent);
void ui_tile5_settings_update(const RoverUIData_t *d, char *buf, size_t buf_sz);
void ui_tile5_sync_wifi_status(void);
void update_all_control_buttons(void);
void update_wifi_ip_label(const char *ip);

#ifdef __cplusplus
}
#endif

#endif // LVGL_UI_INTERNAL_H
