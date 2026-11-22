#include "lvgl_ui_internal.h"
#include "../storage/route_logger.h"

// =========================================================================
// Tile 0: Main Guidance / Pin Seeker Widgets (Hybrid Navigation)
// =========================================================================
lv_obj_t *t0_target_lbl      = NULL;
lv_obj_t *t0_notes_lbl       = NULL;
lv_obj_t *t0_rec_btn         = NULL;
lv_obj_t *t0_rec_btn_lbl     = NULL;

// Navigation Canvas (304x210)
lv_obj_t *t0_nav_canvas      = NULL;
lv_obj_t *t0_mode_badge      = NULL;
lv_obj_t *t0_hdg_hud_lbl     = NULL;
lv_obj_t *t0_pitch_hud_lbl   = NULL;
lv_obj_t *t0_scale_lbl       = NULL;
lv_obj_t *t0_tgt_hdg_lbl     = NULL;
lv_obj_t *t0_cross_h         = NULL;
lv_obj_t *t0_cross_v         = NULL;
lv_obj_t *t0_bullseye_ring1  = NULL;
lv_obj_t *t0_bullseye_ring2  = NULL;
lv_obj_t *t0_bullseye_center = NULL;
lv_obj_t *t0_track_line      = NULL;
lv_obj_t *t0_track_arrow     = NULL;
lv_obj_t *t0_beam_arrow      = NULL;
lv_obj_t *t0_wedge_amber_l   = NULL;
lv_obj_t *t0_wedge_amber_r   = NULL;
lv_obj_t *t0_wedge_green_l   = NULL;
lv_obj_t *t0_wedge_green_r   = NULL;
lv_obj_t *t0_target_icon     = NULL;
lv_obj_t *t0_rover_icon      = NULL;
lv_obj_t *t0_grid_h[T0_GRID_H_LINES] = {0};
lv_obj_t *t0_grid_v[T0_GRID_V_LINES] = {0};

lv_obj_t *t0_dist_lbl        = NULL;
lv_obj_t *t0_steer_lbl       = NULL;
lv_obj_t *t0_offsets_lbl     = NULL;
lv_obj_t *t0_extra_lbl       = NULL;

// Line Stakeout HUD & Canvas Overlays
lv_obj_t *t0_line_baseline   = NULL;
lv_obj_t *t0_line_tie        = NULL;
lv_obj_t *t0_line_arrow      = NULL;
lv_obj_t *t0_line_pt_a       = NULL;
lv_obj_t *t0_line_chord      = NULL;

uint8_t s_tile0_nav_mode = 0; // 0 = Point Guidance, 1 = Unified Line+Bullseye, 2 = Map View

uint8_t rover_get_tile0_nav_mode(void)
{
    return s_tile0_nav_mode;
}

static void t0_mode_toggle_cb(lv_event_t *e)
{
    (void)e;
    play_rover_beep(1600, 25);
    s_tile0_nav_mode = (s_tile0_nav_mode + 1) % 3;
    if (s_tile0_nav_mode == 2) {
        s_map_auto_fit = true;
        s_map_center_rover = false;
    }
    update_t0_target_label();
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Tile 0 Nav Mode -> %s\n",
                     (s_tile0_nav_mode == 1) ? "LINE STAKEOUT" :
                     (s_tile0_nav_mode == 2) ? "MAP VIEW" : "POINT GUIDANCE");
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

void update_t0_target_label(void)
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
        refresh_t0_guidance_readout_immediately();
        return;
    }

    if (s_tile0_nav_mode == 2) {
        // Mode 2: Map View
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
            const char *new_notes = active_wp.notes[0] ? active_wp.notes : "Touch Map to Pan | Zoom with Buttons";
            const char *cur_notes = lv_label_get_text(t0_notes_lbl);
            if (!cur_notes || strcmp(cur_notes, new_notes) != 0) {
                lv_label_set_text(t0_notes_lbl, new_notes);
            }
        } else if (t0_notes_lbl) {
            const char *cur_notes = lv_label_get_text(t0_notes_lbl);
            if (!cur_notes || strcmp(cur_notes, "Touch Map to Pan | Zoom with Buttons") != 0) {
                lv_label_set_text(t0_notes_lbl, "Touch Map to Pan | Zoom with Buttons");
            }
        }
        refresh_t0_guidance_readout_immediately();
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
    refresh_t0_guidance_readout_immediately();
}

// Recompute and update Tile 0 guidance readouts on the exact same LVGL frame as a button tap
// so distance, bearing, steering, and dN/dE offsets snap simultaneously with the waypoint name.
void refresh_t0_guidance_readout_immediately(void)
{
    if (!s_has_last_ui_data) return;

    char buf[128];
    WaypointRecord_t active_wp;
    bool has_wp = waypoint_get_active(&active_wp);

    double dist_m = 0.0;
    float brg_deg = 0.0f, rel_deg = 0.0f, dn_m = 0.0f, de_m = 0.0f;
    if (has_wp) {
        nav_compute_target_guidance(s_last_ui_data.tip_lat,
                                    s_last_ui_data.tip_lon,
                                    s_last_ui_data.heading_valid ? s_last_ui_data.true_heading_deg : 0.0f,
                                    active_wp.latitude,
                                    active_wp.longitude,
                                    &dist_m,
                                    &brg_deg,
                                    &rel_deg,
                                    &dn_m,
                                    &de_m);
    }

    if (t0_tgt_hdg_lbl) {
        if (s_tile0_nav_mode == 1) {
            WaypointRecord_t wp_a, wp_b;
            int16_t idx_a = -1, idx_b = -1;
            waypoint_line_stakeout_get_points(&wp_a, &wp_b, &idx_a, &idx_b);
            if (idx_a >= 0 && idx_b >= 0) {
                NavLineStakeout_t line_st;
                nav_compute_line_stakeout(s_last_ui_data.tip_lat, s_last_ui_data.tip_lon,
                                          wp_a.latitude, wp_a.longitude,
                                          wp_b.latitude, wp_b.longitude,
                                          &line_st);
                snprintf(buf, sizeof(buf), "Line: %05.1f deg", line_st.line_bearing_deg);
                lv_label_set_text(t0_tgt_hdg_lbl, buf);
                float rel_line = line_st.line_bearing_deg - (s_last_ui_data.heading_valid ? s_last_ui_data.true_heading_deg : 0.0f);
                while (rel_line > 180.0f)  rel_line -= 360.0f;
                while (rel_line < -180.0f) rel_line += 360.0f;
                if (s_last_ui_data.heading_valid && fabsf(rel_line) <= 3.0f) {
                    lv_obj_set_style_text_color(t0_tgt_hdg_lbl, lv_color_hex(0x00FF88), 0);
                } else if (s_last_ui_data.heading_valid && fabsf(rel_line) <= 15.0f) {
                    lv_obj_set_style_text_color(t0_tgt_hdg_lbl, lv_color_hex(0xFFB800), 0);
                } else {
                    lv_obj_set_style_text_color(t0_tgt_hdg_lbl, lv_color_hex(0xFFFFFF), 0);
                }
            } else {
                lv_label_set_text(t0_tgt_hdg_lbl, "Line: ---.- deg");
                lv_obj_set_style_text_color(t0_tgt_hdg_lbl, lv_color_hex(0xFFFFFF), 0);
            }
        } else if (has_wp) {
            snprintf(buf, sizeof(buf), "Tgt: %05.1f deg", brg_deg);
            lv_label_set_text(t0_tgt_hdg_lbl, buf);
            if ((s_last_ui_data.heading_valid && fabsf(rel_deg) <= 3.0f) || (dist_m <= 0.05)) {
                lv_obj_set_style_text_color(t0_tgt_hdg_lbl, lv_color_hex(0x00FF88), 0);
            } else if (s_last_ui_data.heading_valid && fabsf(rel_deg) <= 15.0f) {
                lv_obj_set_style_text_color(t0_tgt_hdg_lbl, lv_color_hex(0xFFB800), 0);
            } else {
                lv_obj_set_style_text_color(t0_tgt_hdg_lbl, lv_color_hex(0xFFFFFF), 0);
            }
        } else {
            lv_label_set_text(t0_tgt_hdg_lbl, "Tgt: ---.- deg");
            lv_obj_set_style_text_color(t0_tgt_hdg_lbl, lv_color_hex(0xFFFFFF), 0);
        }
    }

    if (s_tile0_nav_mode == 1) {
        WaypointRecord_t wp_a, wp_b;
        int16_t idx_a = -1, idx_b = -1;
        waypoint_line_stakeout_get_points(&wp_a, &wp_b, &idx_a, &idx_b);
        if (idx_a >= 0 && idx_b >= 0) {
            NavLineStakeout_t stakeout;
            nav_compute_line_stakeout(s_last_ui_data.tip_lat, s_last_ui_data.tip_lon,
                                      wp_a.latitude, wp_a.longitude,
                                      wp_b.latitude, wp_b.longitude,
                                      &stakeout);
            bool on_line = (fabs(stakeout.offset_m) <= 0.02);
            char off_val_str[24];
            nav_format_distance(fabs(stakeout.offset_m), off_val_str, sizeof(off_val_str));
            if (t0_dist_lbl) {
                lv_obj_set_style_text_font(t0_dist_lbl, &lv_font_montserrat_16, 0);
                if (on_line) {
                    snprintf(buf, sizeof(buf), "[OK] ON LINE (%s)", off_val_str);
                    lv_obj_set_style_text_color(t0_dist_lbl, lv_color_hex(0x00FF88), 0);
                } else if (stakeout.offset_m < 0.0) {
                    snprintf(buf, sizeof(buf), "%s LEFT", off_val_str);
                    lv_obj_set_style_text_color(t0_dist_lbl, lv_color_hex(0xF59E0B), 0);
                } else {
                    snprintf(buf, sizeof(buf), "%s RIGHT", off_val_str);
                    lv_obj_set_style_text_color(t0_dist_lbl, lv_color_hex(0xF59E0B), 0);
                }
                lv_label_set_text(t0_dist_lbl, buf);
            }
            if (t0_steer_lbl) {
                if (on_line) {
                    snprintf(buf, sizeof(buf), "[OK] ON LINE - PROCEED ON BEARING");
                    lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0x00FF88), 0);
                } else if (stakeout.offset_m < 0.0) {
                    snprintf(buf, sizeof(buf), "MOVE RIGHT TO LINE >>>");
                    lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0xF59E0B), 0);
                } else {
                    snprintf(buf, sizeof(buf), "<<< MOVE LEFT TO LINE");
                    lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0xF59E0B), 0);
                }
                lv_label_set_text(t0_steer_lbl, buf);
            }
            if (t0_offsets_lbl) {
                char sta_str[32], rem_str[24];
                nav_format_station(stakeout.station_m, sta_str, sizeof(sta_str));
                nav_format_distance(fabs(stakeout.remaining_m), rem_str, sizeof(rem_str));
                snprintf(buf, sizeof(buf), "%s (%.1f%%) | Rem: %s", sta_str, stakeout.pct_complete, rem_str);
                lv_label_set_text(t0_offsets_lbl, buf);
            }
        }
    } else if (s_tile0_nav_mode == 2) {
        char dist_str[24];
        nav_format_distance(dist_m, dist_str, sizeof(dist_str));
        if (t0_dist_lbl) {
            lv_obj_set_style_text_font(t0_dist_lbl, &lv_font_montserrat_16, 0);
            if (has_wp) {
                snprintf(buf, sizeof(buf), "%s @ %05.1f deg", dist_str, brg_deg);
            } else {
                snprintf(buf, sizeof(buf), "--- m @ --- deg");
            }
            lv_label_set_text(t0_dist_lbl, buf);
        }
        if (t0_steer_lbl) {
            if (has_wp) {
                snprintf(buf, sizeof(buf), "TGT: %s (%d/%d)",
                         active_wp.label, g_active_waypoint_idx + 1, g_waypoint_count);
            } else {
                snprintf(buf, sizeof(buf), "No Target Active");
            }
            lv_label_set_text(t0_steer_lbl, buf);
        }
    } else if (has_wp) {
        char dist_str[24], dn_str[24], de_str[24];
        nav_format_distance(dist_m, dist_str, sizeof(dist_str));
        nav_format_offset(dn_m, dn_str, sizeof(dn_str));
        nav_format_offset(de_m, de_str, sizeof(de_str));

        if (t0_dist_lbl) {
            lv_obj_set_style_text_font(t0_dist_lbl, &lv_font_montserrat_24, 0);
            lv_label_set_text(t0_dist_lbl, dist_str);
            lv_obj_set_style_text_color(t0_dist_lbl, lv_color_hex(0xFFFFFF), 0);
        }
        if (t0_scale_lbl && dist_m > 1.5) {
            snprintf(buf, sizeof(buf), "Range: %s", dist_str);
            lv_label_set_text(t0_scale_lbl, buf);
        }
        if (t0_steer_lbl) {
            if (dist_m <= 1.5) {
                if (dist_m < 0.02) {
                    snprintf(buf, sizeof(buf), "[OK] PIN ACQUIRED! (Acc: %.1f cm)", s_last_ui_data.h_acc_m * 100.0f);
                    lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0x00FF88), 0);
                } else {
                    snprintf(buf, sizeof(buf), "Stakeout: Align Tip to Crosshair");
                    lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0xFFCC00), 0);
                }
            } else if (s_last_ui_data.heading_valid) {
                float abs_turn = fabsf(rel_deg);
                if (abs_turn <= 3.0f) {
                    snprintf(buf, sizeof(buf), "[OK] ON TRACK - Aimed at Pin");
                    lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0x00FF88), 0);
                } else if (abs_turn <= 15.0f) {
                    snprintf(buf, sizeof(buf), (rel_deg > 0.0f) ? ">>> Turn Right %.0f deg" : "<<< Turn Left %.0f deg", abs_turn);
                    lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0xFFB800), 0);
                } else {
                    snprintf(buf, sizeof(buf), (rel_deg > 0.0f) ? ">>> Turn Right %.0f deg" : "<<< Turn Left %.0f deg", abs_turn);
                    lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0xFFFFFF), 0);
                }
            } else {
                snprintf(buf, sizeof(buf), "Brg: %.1f deg | Acquiring Heading...", brg_deg);
                lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0xFF8800), 0);
            }
            lv_label_set_text(t0_steer_lbl, buf);
        }
        if (t0_offsets_lbl) {
            snprintf(buf, sizeof(buf), "dN: %s   dE: %s", dn_str, de_str);
            lv_label_set_text(t0_offsets_lbl, buf);
            lv_obj_set_style_text_color(t0_offsets_lbl, lv_color_hex(0x00D8FF), 0);
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

void ui_tile0_create(lv_obj_t *parent)
{
    // =====================================================================
    // TILE 0: Pin Seeker & Guidance (Hybrid Navigation Canvas)
    // =====================================================================
    // Target selector row (compacted to y = 3 since coordinates are now in persistent header)
    lv_obj_t *btn_prev = lv_button_create(parent);
    lv_obj_set_size(btn_prev, 44, 28);
    lv_obj_align(btn_prev, LV_ALIGN_TOP_LEFT, 6, 3);
    lv_obj_set_ext_click_area(btn_prev, 12);
    lv_obj_add_event_cb(btn_prev, prev_wp_btn_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_t *lbl_prev = lv_label_create(btn_prev);
    lv_label_set_text(lbl_prev, LV_SYMBOL_LEFT);
    lv_obj_center(lbl_prev);

    t0_target_lbl = lv_label_create(parent);
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

    lv_obj_t *btn_next = lv_button_create(parent);
    lv_obj_set_size(btn_next, 44, 28);
    lv_obj_align(btn_next, LV_ALIGN_TOP_RIGHT, -6, 3);
    lv_obj_set_ext_click_area(btn_next, 12);
    lv_obj_add_event_cb(btn_next, next_wp_btn_cb, LV_EVENT_PRESSED, NULL);
    lv_obj_t *lbl_next = lv_label_create(btn_next);
    lv_label_set_text(lbl_next, LV_SYMBOL_RIGHT);
    lv_obj_center(lbl_next);

    // Target notes row (circular marquee autoscroll when text exceeds width)
    t0_notes_lbl = lv_label_create(parent);
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
    t0_nav_canvas = lv_obj_create(parent);
    lv_obj_set_size(t0_nav_canvas, 304, 210);
    lv_obj_align(t0_nav_canvas, LV_ALIGN_TOP_MID, 0, 52);
    lv_obj_set_style_radius(t0_nav_canvas, 8, 0);
    lv_obj_set_style_bg_color(t0_nav_canvas, lv_color_hex(0x13191E), 0);
    lv_obj_set_style_border_color(t0_nav_canvas, lv_color_hex(0x283B47), 0);
    lv_obj_set_style_border_width(t0_nav_canvas, 2, 0);
    lv_obj_set_scrollbar_mode(t0_nav_canvas, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scrollable(t0_nav_canvas, false);
    lv_obj_set_style_pad_all(t0_nav_canvas, 0, 0);
    lv_obj_set_clickable(t0_nav_canvas, true);
    lv_obj_add_event_cb(t0_nav_canvas, t0_canvas_drag_cb, LV_EVENT_ALL, NULL);

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
    lv_obj_set_size(t0_cross_h, 220, 1);
    lv_obj_set_style_bg_color(t0_cross_h, lv_color_hex(0x283E4C), 0);
    lv_obj_set_style_border_width(t0_cross_h, 0, 0);
    lv_obj_center(t0_cross_h);

    t0_cross_v = lv_obj_create(t0_nav_canvas);
    lv_obj_set_size(t0_cross_v, 1, 146);
    lv_obj_set_style_bg_color(t0_cross_v, lv_color_hex(0x283E4C), 0);
    lv_obj_set_style_border_width(t0_cross_v, 0, 0);
    lv_obj_center(t0_cross_v);

    // Bullseye Concentric Rings (Fine Stakeout & Mode 1 Unified Bullseye)
    t0_bullseye_ring1 = lv_obj_create(t0_nav_canvas);
    lv_obj_set_size(t0_bullseye_ring1, 146, 146);
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

    // Property line chord overlay across Bullseye in Unified Line+Bullseye mode
    t0_line_chord = lv_line_create(t0_nav_canvas);
    lv_obj_set_style_line_color(t0_line_chord, lv_color_hex(0x00D8FF), 0);
    lv_obj_set_style_line_width(t0_line_chord, 2, 0);
    lv_obj_set_hidden(t0_line_chord, true);

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

    // Line Stakeout baseline, direction arrow, tie line, and point markers (pulled inward for canvas whitespace)
    t0_line_baseline = lv_line_create(t0_nav_canvas);
    lv_obj_set_style_line_color(t0_line_baseline, lv_color_hex(0x00D8FF), 0);
    lv_obj_set_style_line_width(t0_line_baseline, 2, 0);
    static const lv_point_precise_t pts_baseline[2] = {{152, 178}, {152, 46}};
    lv_line_set_points(t0_line_baseline, pts_baseline, 2);
    lv_obj_set_hidden(t0_line_baseline, true);

    t0_line_arrow = lv_line_create(t0_nav_canvas);
    lv_obj_set_style_line_color(t0_line_arrow, lv_color_hex(0x00D8FF), 0);
    lv_obj_set_style_line_width(t0_line_arrow, 3, 0);
    static const lv_point_precise_t pts_line_arrow[3] = {{146, 58}, {152, 46}, {158, 58}};
    lv_line_set_points(t0_line_arrow, pts_line_arrow, 3);
    lv_obj_set_hidden(t0_line_arrow, true);

    t0_line_tie = lv_line_create(t0_nav_canvas);
    lv_obj_set_style_line_color(t0_line_tie, lv_color_hex(0xF59E0B), 0);
    lv_obj_set_style_line_width(t0_line_tie, 2, 0);
    lv_obj_set_hidden(t0_line_tie, true);

    t0_line_pt_a = lv_obj_create(t0_nav_canvas);
    lv_obj_set_size(t0_line_pt_a, 12, 12);
    lv_obj_set_style_radius(t0_line_pt_a, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(t0_line_pt_a, lv_color_hex(0xF59E0B), 0);
    lv_obj_set_style_border_color(t0_line_pt_a, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(t0_line_pt_a, 2, 0);
    lv_obj_align(t0_line_pt_a, LV_ALIGN_TOP_LEFT, 152 - 6, 178 - 6);
    lv_obj_set_hidden(t0_line_pt_a, true);

    // Mode 2 Map View Layers & HUD Controls (inside t0_nav_canvas before bottom readout labels)
    ui_tile0_map_create(t0_nav_canvas);

    // Readout section below canvas (compacted to fit inside 374px tileview)
    t0_dist_lbl = lv_label_create(parent);
    lv_label_set_text(t0_dist_lbl, "0.00 m");
    lv_obj_set_style_text_color(t0_dist_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(t0_dist_lbl, &lv_font_montserrat_24, 0);
    lv_obj_align(t0_dist_lbl, LV_ALIGN_TOP_MID, 0, 266);

    t0_steer_lbl = lv_label_create(parent);
    lv_label_set_text(t0_steer_lbl, "Acquiring Position...");
    lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0xFFCC00), 0);
    lv_obj_set_style_text_font(t0_steer_lbl, &lv_font_montserrat_14, 0);
    lv_obj_align(t0_steer_lbl, LV_ALIGN_TOP_MID, 0, 292);

    t0_offsets_lbl = lv_label_create(parent);
    lv_label_set_text(t0_offsets_lbl, "dN: +0.0 cm   dE: +0.0 cm");
    lv_obj_set_style_text_color(t0_offsets_lbl, lv_color_hex(0x00D8FF), 0);
    lv_obj_set_style_text_font(t0_offsets_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t0_offsets_lbl, LV_ALIGN_TOP_MID, 0, 314);

    t0_extra_lbl = lv_label_create(parent);
    lv_label_set_text(t0_extra_lbl, "");
    lv_obj_set_style_text_color(t0_extra_lbl, lv_color_hex(0x94A3B8), 0);
    lv_obj_set_style_text_font(t0_extra_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t0_extra_lbl, LV_ALIGN_TOP_MID, 0, 334);
    lv_obj_set_hidden(t0_extra_lbl, true);
}

void ui_tile0_update(const RoverUIData_t *d, char *buf, size_t buf_sz)
{
    ui_tile0_views_update_hud(d);
    if (s_tile0_nav_mode == 1) {
        ui_tile0_views_update_mode1(d, buf, buf_sz);
    } else if (s_tile0_nav_mode == 2) {
        ui_tile0_map_update(d, buf, buf_sz);
    } else {
        ui_tile0_views_update_mode0(d, buf, buf_sz);
    }
}
