#include "lvgl_ui_internal.h"

static float wrap180f(float deg)
{
    while (deg > 180.0f)  deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

static float wrap360f(float deg)
{
    while (deg >= 360.0f) deg -= 360.0f;
    while (deg < 0.0f)    deg += 360.0f;
    return deg;
}

static int16_t calc_log_x_px(float dist_m, float d_max_m)
{
    if (d_max_m < 2.0f) d_max_m = 50.0f;
    float denom = logf(1.0f + d_max_m / 0.25f);
    if (denom <= 1e-4f) return 0;
    float norm = logf(1.0f + fabsf(dist_m) / 0.25f) / denom;
    int16_t px = (int16_t)roundf(104.0f * norm);
    if (px < 0)   px = 0;
    if (px > 104) px = 104;
    return px;
}

static void hide_all_map_layers(void)
{
    if (t0_map_boundary)   lv_obj_set_hidden(t0_map_boundary, true);
    if (t0_map_active_seg) lv_obj_set_hidden(t0_map_active_seg, true);
    if (t0_map_trail)      lv_obj_set_hidden(t0_map_trail, true);
    for (int t = 0; t < T0_MAP_MAX_TIES; t++) {
        if (t0_map_ties[t]) lv_obj_set_hidden(t0_map_ties[t], true);
    }
    for (int i = 0; i < T0_MAP_MAX_PINS; i++) {
        if (t0_map_pins[i]) lv_obj_set_hidden(t0_map_pins[i], true);
    }
    if (t0_map_btn_zin)  lv_obj_set_hidden(t0_map_btn_zin, true);
    if (t0_map_btn_zout) lv_obj_set_hidden(t0_map_btn_zout, true);
    if (t0_map_btn_fit)  lv_obj_set_hidden(t0_map_btn_fit, true);
    if (t0_map_btn_ctr)  lv_obj_set_hidden(t0_map_btn_ctr, true);
}

static void hide_bullseye_layers(void)
{
    if (t0_bullseye_ring1)  lv_obj_set_hidden(t0_bullseye_ring1, true);
    if (t0_bullseye_ring2)  lv_obj_set_hidden(t0_bullseye_ring2, true);
    if (t0_bullseye_center) lv_obj_set_hidden(t0_bullseye_center, true);
    if (t0_cross_h)         lv_obj_set_hidden(t0_cross_h, true);
    if (t0_cross_v)         lv_obj_set_hidden(t0_cross_v, true);
    if (t0_line_chord)      lv_obj_set_hidden(t0_line_chord, true);
}

static void hide_triangle_layers(void)
{
    for (int i = 0; i < T0_GRID_V_LINES; i++) {
        if (t0_grid_v[i]) lv_obj_set_hidden(t0_grid_v[i], true);
    }
    if (t0_log_lbl_inner) lv_obj_set_hidden(t0_log_lbl_inner, true);
    if (t0_log_lbl_outer) lv_obj_set_hidden(t0_log_lbl_outer, true);
    if (t0_line_baseline) lv_obj_set_hidden(t0_line_baseline, true);
    if (t0_line_ext_top)  lv_obj_set_hidden(t0_line_ext_top, true);
    if (t0_line_ext_bot)  lv_obj_set_hidden(t0_line_ext_bot, true);
    if (t0_line_arrow)    lv_obj_set_hidden(t0_line_arrow, true);
    if (t0_line_tie)      lv_obj_set_hidden(t0_line_tie, true);
    if (t0_line_hyp)      lv_obj_set_hidden(t0_line_hyp, true);
    if (t0_line_pt_a)     lv_obj_set_hidden(t0_line_pt_a, true);
}

static void hide_steer_graphics(void)
{
    for (int i = 0; i < 3; i++) {
        if (t0_chev_left[i])  lv_obj_set_hidden(t0_chev_left[i], true);
        if (t0_chev_right[i]) lv_obj_set_hidden(t0_chev_right[i], true);
    }
    if (t0_ontrack_bar_l) lv_obj_set_hidden(t0_ontrack_bar_l, true);
    if (t0_ontrack_bar_r) lv_obj_set_hidden(t0_ontrack_bar_r, true);
}

static void update_steer_graphics(bool on_line, bool pin_acquired,
                                  double offset_m, bool perp_locked_green)
{
    if (pin_acquired || on_line) {
        for (int i = 0; i < 3; i++) {
            if (t0_chev_left[i])  lv_obj_set_hidden(t0_chev_left[i], true);
            if (t0_chev_right[i]) lv_obj_set_hidden(t0_chev_right[i], true);
        }
        lv_color_t bar_col = lv_color_hex(pin_acquired ? 0x00FF00 : 0x00FF88);
        if (t0_ontrack_bar_l) {
            lv_obj_set_style_bg_color(t0_ontrack_bar_l, bar_col, 0);
            lv_obj_set_hidden(t0_ontrack_bar_l, false);
        }
        if (t0_ontrack_bar_r) {
            lv_obj_set_style_bg_color(t0_ontrack_bar_r, bar_col, 0);
            lv_obj_set_hidden(t0_ontrack_bar_r, false);
        }
        return;
    }

    if (t0_ontrack_bar_l) lv_obj_set_hidden(t0_ontrack_bar_l, true);
    if (t0_ontrack_bar_r) lv_obj_set_hidden(t0_ontrack_bar_r, true);

    float fade_m = current_settings.line_audio_fade_m;
    if (fade_m < 1.0f) fade_m = 20.0f;
    uint32_t chev_hex = perp_locked_green ? 0x00FF88 :
                        ((fabs(offset_m) > (double)fade_m) ? 0xFF5522 : 0xF59E0B);
    lv_color_t chev_col = lv_color_hex(chev_hex);

    if (offset_m < 0.0) {
        // Rover LEFT of line -> show >>> chevrons on center-left (Move Right)
        for (int i = 0; i < 3; i++) {
            if (t0_chev_left[i]) {
                lv_obj_set_style_line_color(t0_chev_left[i], chev_col, 0);
                lv_obj_set_hidden(t0_chev_left[i], false);
            }
            if (t0_chev_right[i]) {
                lv_obj_set_hidden(t0_chev_right[i], true);
            }
        }
    } else {
        // Rover RIGHT of line -> show <<< chevrons on center-right (Move Left)
        for (int i = 0; i < 3; i++) {
            if (t0_chev_right[i]) {
                lv_obj_set_style_line_color(t0_chev_right[i], chev_col, 0);
                lv_obj_set_hidden(t0_chev_right[i], false);
            }
            if (t0_chev_left[i]) {
                lv_obj_set_hidden(t0_chev_left[i], true);
            }
        }
    }
}

static void draw_barbed_arrow(int16_t ox, int16_t oy, float rel_deg, float len_px, uint32_t hex_color)
{
    if (!t0_beam_arrow) return;
    float rad = rel_deg * (float)(M_PI / 180.0);
    float tip_x = (float)ox + len_px * sinf(rad);
    float tip_y = (float)oy - len_px * cosf(rad);

    float b1_rad = rad - 2.6179939f; // -150 deg
    float b2_rad = rad + 2.6179939f; // +150 deg
    float b1_x = tip_x + 8.0f * sinf(b1_rad);
    float b1_y = tip_y - 8.0f * cosf(b1_rad);
    float b2_x = tip_x + 8.0f * sinf(b2_rad);
    float b2_y = tip_y - 8.0f * cosf(b2_rad);

    static lv_point_precise_t arrow_pts[5];
    arrow_pts[0].x = ox;
    arrow_pts[0].y = oy;
    arrow_pts[1].x = (int16_t)roundf(tip_x);
    arrow_pts[1].y = (int16_t)roundf(tip_y);
    arrow_pts[2].x = (int16_t)roundf(b1_x);
    arrow_pts[2].y = (int16_t)roundf(b1_y);
    arrow_pts[3].x = (int16_t)roundf(tip_x);
    arrow_pts[3].y = (int16_t)roundf(tip_y);
    arrow_pts[4].x = (int16_t)roundf(b2_x);
    arrow_pts[4].y = (int16_t)roundf(b2_y);
    lv_line_set_points(t0_beam_arrow, arrow_pts, 5);
    lv_obj_set_style_line_color(t0_beam_arrow, lv_color_hex(hex_color), 0);
    lv_obj_set_style_line_width(t0_beam_arrow, 3, 0);
    lv_obj_set_hidden(t0_beam_arrow, false);
}

void ui_tile0_views_update_hud(const RoverUIData_t *d)
{
    char hud_buf[32];
    if (t0_hdg_hud_lbl) {
        char h_str[16], p_str[16];
        if (d->heading_valid) {
            snprintf(h_str, sizeof(h_str), "%05.1f", d->true_heading_deg);
        } else {
            snprintf(h_str, sizeof(h_str), "---.-");
        }
        if (d->heading_valid || d->baseline_length_m > 0.01f) {
            snprintf(p_str, sizeof(p_str), "%+.1f", d->baseline_pitch_deg);
        } else {
            snprintf(p_str, sizeof(p_str), "--.-");
        }
        snprintf(hud_buf, sizeof(hud_buf), "H:%s P:%s", h_str, p_str);
        rover_label_set_text_if_changed(t0_hdg_hud_lbl, hud_buf);
    }

    if (t0_tgt_hdg_lbl) {
        ActiveGuidanceSegment_t seg;
        bool has_seg = waypoint_guidance_get_segment(d->tip_lat, d->tip_lon, &seg);
        if (has_seg) {
            double dist_b_m = 0.0;
            float brg_b_deg = 0.0f, rel_b_deg = 0.0f;
            nav_compute_target_guidance(d->tip_lat, d->tip_lon,
                                        d->heading_valid ? d->true_heading_deg : 0.0f,
                                        seg.lat_b, seg.lon_b,
                                        &dist_b_m, &brg_b_deg, &rel_b_deg, NULL, NULL);
            snprintf(hud_buf, sizeof(hud_buf), "B: %05.1f", brg_b_deg);
            rover_label_set_text_if_changed(t0_tgt_hdg_lbl, hud_buf);
            if ((d->heading_valid && fabsf(rel_b_deg) <= current_settings.line_arrow_green_deg) ||
                (dist_b_m <= 0.05)) {
                lv_obj_set_style_text_color(t0_tgt_hdg_lbl, lv_color_hex(0x00FF88), 0);
            } else if (d->heading_valid && fabsf(rel_b_deg) <= current_settings.line_arrow_amber_deg) {
                lv_obj_set_style_text_color(t0_tgt_hdg_lbl, lv_color_hex(0xFFB800), 0);
            } else {
                lv_obj_set_style_text_color(t0_tgt_hdg_lbl, lv_color_hex(0xFFFFFF), 0);
            }
        } else if (d->has_active_target) {
            snprintf(hud_buf, sizeof(hud_buf), "B: %05.1f", d->bearing_to_target_deg);
            rover_label_set_text_if_changed(t0_tgt_hdg_lbl, hud_buf);
            if ((d->heading_valid && fabsf(d->rel_bearing_deg) <= current_settings.line_arrow_green_deg) ||
                (d->distance_to_target_m <= 0.05)) {
                lv_obj_set_style_text_color(t0_tgt_hdg_lbl, lv_color_hex(0x00FF88), 0);
            } else if (d->heading_valid && fabsf(d->rel_bearing_deg) <= current_settings.line_arrow_amber_deg) {
                lv_obj_set_style_text_color(t0_tgt_hdg_lbl, lv_color_hex(0xFFB800), 0);
            } else {
                lv_obj_set_style_text_color(t0_tgt_hdg_lbl, lv_color_hex(0xFFFFFF), 0);
            }
        } else {
            rover_label_set_text_if_changed(t0_tgt_hdg_lbl, "B: ---.-");
            lv_obj_set_style_text_color(t0_tgt_hdg_lbl, lv_color_hex(0xFFFFFF), 0);
        }
    }
}

void ui_tile0_views_update_guidance(const RoverUIData_t *d, char *buf, size_t buf_sz)
{
    hide_all_map_layers();

    // Sync & show bottom-left Start Anchor button ("A: DEED" vs "A: HERE")
    bool is_direct_mode = waypoint_guidance_is_direct_mode();
    if (t0_anchor_btn && t0_anchor_btn_lbl) {
        const char *want_txt = is_direct_mode ? "A: HERE" : "A: DEED";
        const char *cur_txt = lv_label_get_text(t0_anchor_btn_lbl);
        if (!cur_txt || strcmp(cur_txt, want_txt) != 0) {
            lv_label_set_text(t0_anchor_btn_lbl, want_txt);
        }
        lv_obj_set_style_bg_color(t0_anchor_btn, is_direct_mode ? COLOR_BTN_WIFI_AP : COLOR_UNIT_SI_TEAL, 0);
        lv_obj_set_hidden(t0_anchor_btn, false);
    }

    ActiveGuidanceSegment_t seg;
    bool has_seg = waypoint_guidance_get_segment(d->tip_lat, d->tip_lon, &seg);

    if (!has_seg) {
        hide_bullseye_layers();
        hide_triangle_layers();
        hide_steer_graphics();
        if (t0_beam_arrow) lv_obj_set_hidden(t0_beam_arrow, true);

        if (seg.label_b[0] != '\0') {
            // Single-point / Direct HERE mode waiting for first GNSS coordinate to anchor A
            snprintf(buf, buf_sz, "HERE -> %s [SET A]", seg.label_b);
            if (t0_target_lbl) rover_label_set_text_if_changed(t0_target_lbl, buf);
            if (t0_notes_lbl) {
                const char *n = seg.notes_b[0] ? seg.notes_b : "Waiting for GNSS Fix to Anchor A...";
                rover_label_set_text_if_changed(t0_notes_lbl, n);
            }
            lv_label_set_text(t0_mode_badge, "GUIDANCE");
            lv_obj_set_style_bg_color(t0_mode_badge, COLOR_BTN_WIFI_AP, 0);
            if (t0_dist_lbl) {
                lv_obj_set_style_text_font(t0_dist_lbl, &lv_font_montserrat_16, 0);
                rover_label_set_text_if_changed(t0_dist_lbl, "Acquiring Fix...");
                lv_obj_set_style_text_color(t0_dist_lbl, lv_color_hex(0xF59E0B), 0);
            }
            if (t0_offsets_lbl) {
                rover_label_set_text_if_changed(t0_offsets_lbl, "Start A anchors automatically on GNSS fix");
                lv_obj_set_style_text_color(t0_offsets_lbl, lv_color_hex(0x94A3B8), 0);
            }
            if (t0_target_icon) {
                lv_obj_align(t0_target_icon, LV_ALIGN_TOP_LEFT, T0_CANVAS_CX - 8, T0_GUIDE_Y_TOP - 8);
                lv_obj_set_hidden(t0_target_icon, false);
            }
            if (t0_rover_icon) {
                lv_obj_align(t0_rover_icon, LV_ALIGN_TOP_LEFT, T0_CANVAS_CX - 7, T0_GUIDE_Y_BOT - 7);
                lv_obj_set_style_bg_color(t0_rover_icon, lv_color_hex(0x38BDF8), 0);
                lv_obj_set_hidden(t0_rover_icon, false);
            }
        } else {
            // No CSV waypoints loaded
            if (t0_target_lbl) rover_label_set_text_if_changed(t0_target_lbl, "No Target (0/0)");
            if (t0_notes_lbl)  rover_label_set_text_if_changed(t0_notes_lbl, "---");
            lv_label_set_text(t0_mode_badge, "NO TARGET");
            lv_obj_set_style_bg_color(t0_mode_badge, lv_palette_main(LV_PALETTE_GREY), 0);
            if (t0_dist_lbl) {
                lv_obj_set_style_text_font(t0_dist_lbl, &lv_font_montserrat_16, 0);
                rover_label_set_text_if_changed(t0_dist_lbl, "No Target Active");
                lv_obj_set_style_text_color(t0_dist_lbl, lv_color_hex(0xFFFFFF), 0);
            }
            if (t0_offsets_lbl) {
                rover_label_set_text_if_changed(t0_offsets_lbl, "Select target in Tile 3 Waypoints");
                lv_obj_set_style_text_color(t0_offsets_lbl, lv_color_hex(0x888888), 0);
            }
            if (t0_target_icon) lv_obj_set_hidden(t0_target_icon, true);
            if (t0_rover_icon) {
                lv_obj_align(t0_rover_icon, LV_ALIGN_TOP_LEFT, T0_CANVAS_CX - 7, T0_GUIDE_Y_BOT - 7);
                lv_obj_set_style_bg_color(t0_rover_icon, lv_color_hex(0x38BDF8), 0);
                lv_obj_set_hidden(t0_rover_icon, false);
            }
        }
        return;
    }

    // Active Segment Resolved (DEED A -> B or DIRECT HERE -> B)
    NavLineStakeout_t stakeout;
    nav_compute_line_stakeout(d->tip_lat, d->tip_lon,
                              seg.lat_a, seg.lon_a,
                              seg.lat_b, seg.lon_b,
                              &stakeout);

    double dist_b_m = 0.0;
    float brg_b_deg = 0.0f, rel_b_deg = 0.0f, dn_b_m = 0.0f, de_b_m = 0.0f;
    nav_compute_target_guidance(d->tip_lat, d->tip_lon,
                                d->heading_valid ? d->true_heading_deg : 0.0f,
                                seg.lat_b, seg.lon_b,
                                &dist_b_m, &brg_b_deg, &rel_b_deg, &dn_b_m, &de_b_m);

    // Top Title & Consolidated Marquee Row
    snprintf(buf, buf_sz, "%s -> %s [%s]",
             seg.label_a, seg.label_b, seg.is_direct_here ? "SET A" : "SWAP");
    if (t0_target_lbl) rover_label_set_text_if_changed(t0_target_lbl, buf);

    char qbrg[32], len_str[24], log_span_str[24];
    nav_format_quadrant_bearing(stakeout.line_bearing_deg, qbrg, sizeof(qbrg));
    nav_format_distance(stakeout.total_length_m, len_str, sizeof(len_str));
    float d_max = current_settings.line_log_max_m;
    if (d_max < 2.0f) d_max = 50.0f;
    nav_format_distance((double)d_max, log_span_str, sizeof(log_span_str));

    bool on_line = (fabs(stakeout.offset_m) <= 0.02);
    char off_val_str[24], dist_b_str[24];
    nav_format_distance(fabs(stakeout.offset_m), off_val_str, sizeof(off_val_str));
    nav_format_distance(dist_b_m, dist_b_str, sizeof(dist_b_str));

    if (d->mode1_bullseye_active) {
        // -----------------------------------------------------------------
        // Sub-view A (< 4.8 m / > 5.3 m Hysteresis): Bullseye + A -> B Chord
        // -----------------------------------------------------------------
        hide_triangle_layers();

        snprintf(buf, buf_sz, "%s: %s (%.1f deg) | %s",
                 seg.is_direct_here ? "Direct" : "Deed", qbrg, stakeout.line_bearing_deg, len_str);
        if (t0_notes_lbl) rover_label_set_text_if_changed(t0_notes_lbl, buf);

        lv_obj_set_hidden(t0_bullseye_ring1, false);
        lv_obj_set_hidden(t0_bullseye_ring2, false);
        lv_obj_set_hidden(t0_bullseye_center, false);
        lv_obj_set_hidden(t0_cross_h, false);
        lv_obj_set_hidden(t0_cross_v, false);

        lv_label_set_text(t0_mode_badge, "BULLSEYE");
        lv_obj_set_style_bg_color(t0_mode_badge, lv_color_hex(0x00AA55), 0);

        float px_per_m;
        if (dist_b_m <= 1.5) {
            px_per_m = 90.0f / 1.5f;
            lv_obj_set_size(t0_bullseye_ring2, 60, 60);
        } else {
            px_per_m = 90.0f / 5.0f;
            lv_obj_set_size(t0_bullseye_ring2, 54, 54);
        }

        // Overlaid A -> B chord across the Bullseye centered on Target B
        float line_rad = stakeout.line_bearing_deg * (float)(M_PI / 180.0);
        float ux = sinf(line_rad);
        float uy = -cosf(line_rad);
        static lv_point_precise_t chord_pts[2];
        chord_pts[0].x = T0_CANVAS_CX - (int16_t)roundf(90.0f * ux);
        chord_pts[0].y = T0_CANVAS_CY - (int16_t)roundf(90.0f * uy);
        chord_pts[1].x = T0_CANVAS_CX + (int16_t)roundf(90.0f * ux);
        chord_pts[1].y = T0_CANVAS_CY + (int16_t)roundf(90.0f * uy);
        lv_line_set_points(t0_line_chord, chord_pts, 2);
        lv_obj_set_style_line_color(t0_line_chord, on_line ? lv_color_hex(0x00FF88) : lv_color_hex(0x00D8FF), 0);
        lv_obj_set_hidden(t0_line_chord, false);

        lv_obj_align(t0_target_icon, LV_ALIGN_CENTER, 0, 0);
        lv_obj_set_hidden(t0_target_icon, false);

        if (dist_b_m < 0.02) {
            lv_obj_set_style_bg_color(t0_bullseye_center, lv_color_hex(0x00FF00), 0);
            lv_obj_set_style_border_color(t0_bullseye_center, lv_color_hex(0xFFFFFF), 0);
        } else {
            lv_obj_set_style_bg_color(t0_bullseye_center, lv_color_hex(0xFF0055), 0);
            lv_obj_set_style_border_color(t0_bullseye_center, lv_color_hex(0xFFAABB), 0);
        }

        int16_t r_px = (int16_t)roundf(-de_b_m * px_per_m);
        int16_t r_py = (int16_t)roundf( dn_b_m * px_per_m);
        if (r_px > 94) r_px = 94; else if (r_px < -94) r_px = -94;
        if (r_py > 94) r_py = 94; else if (r_py < -94) r_py = -94;
        lv_obj_align(t0_rover_icon, LV_ALIGN_CENTER, r_px, r_py);
        lv_obj_set_style_bg_color(t0_rover_icon, on_line ? lv_color_hex(0x00FF88) : lv_color_hex(0x38BDF8), 0);
        lv_obj_set_hidden(t0_rover_icon, false);

        if (d->heading_valid) {
            float err_b = fabsf(wrap180f(d->true_heading_deg - brg_b_deg));
            uint32_t arrow_hex = 0xFFFFFF;
            if (err_b <= current_settings.line_arrow_green_deg)      arrow_hex = 0x00FF88;
            else if (err_b <= current_settings.line_arrow_amber_deg) arrow_hex = 0xFFB800;
            draw_barbed_arrow(T0_CANVAS_CX + r_px, T0_CANVAS_CY + r_py, d->true_heading_deg, 22.0f, arrow_hex);
        } else {
            lv_obj_set_hidden(t0_beam_arrow, true);
        }

        update_steer_graphics(on_line, (dist_b_m < 0.02), stakeout.offset_m, false);

        if (t0_dist_lbl) {
            lv_obj_set_style_text_font(t0_dist_lbl, &lv_font_montserrat_16, 0);
            if (dist_b_m < 0.02) {
                snprintf(buf, buf_sz, "[OK] PIN ACQUIRED! (%.1f cm)", d->h_acc_m * 100.0f);
                lv_obj_set_style_text_color(t0_dist_lbl, lv_color_hex(0x00FF00), 0);
            } else if (on_line) {
                snprintf(buf, buf_sz, "B: %s | [OK] ON LINE", dist_b_str);
                lv_obj_set_style_text_color(t0_dist_lbl, lv_color_hex(0x00FF88), 0);
            } else {
                snprintf(buf, buf_sz, "B: %s | %s %s",
                         dist_b_str, off_val_str, (stakeout.offset_m < 0.0) ? "L" : "R");
                lv_obj_set_style_text_color(t0_dist_lbl, lv_color_hex(0xFFFFFF), 0);
            }
            rover_label_set_text_if_changed(t0_dist_lbl, buf);
        }

        char dn_b_str[24], de_b_str[24];
        nav_format_offset(dn_b_m, dn_b_str, sizeof(dn_b_str));
        nav_format_offset(de_b_m, de_b_str, sizeof(de_b_str));
        if (t0_offsets_lbl) {
            snprintf(buf, buf_sz, "dN: %s   dE: %s", dn_b_str, de_b_str);
            rover_label_set_text_if_changed(t0_offsets_lbl, buf);
            lv_obj_set_style_text_color(t0_offsets_lbl, lv_color_hex(0x00D8FF), 0);
        }
        return;
    }

    // ---------------------------------------------------------------------
    // Sub-view B (Outside 5 m): Unified Line-Up Log-Scale Triangle (A-B-R)
    // ---------------------------------------------------------------------
    hide_bullseye_layers();

    lv_label_set_text(t0_mode_badge, "GUIDANCE");
    lv_obj_set_style_bg_color(t0_mode_badge, seg.is_direct_here ? COLOR_BTN_WIFI_AP : COLOR_UNIT_SI_TEAL, 0);

    snprintf(buf, buf_sz, "%s: %s (%.1f deg) | %s | Log: %s",
             seg.is_direct_here ? "Direct" : "Deed",
             qbrg, stakeout.line_bearing_deg, len_str, log_span_str);
    if (t0_notes_lbl) rover_label_set_text_if_changed(t0_notes_lbl, buf);

    // Symmetric Log-X Reference Grid Ticks
    float tick_in_m  = current_settings.use_imperial_units ? 0.9144f : 1.0f;   // 3 ft or 1 m
    float tick_out_m = current_settings.use_imperial_units ? 9.144f  : 10.0f;  // 30 ft or 10 m
    int16_t px_in    = calc_log_x_px(tick_in_m, d_max);
    int16_t px_out   = calc_log_x_px(tick_out_m, d_max);

    bool show_in  = (px_in >= 10 && tick_in_m <= d_max * 0.90f);
    bool show_out = (px_out >= px_in + 18 && tick_out_m <= d_max * 0.95f);

    if (t0_grid_v[1] && t0_grid_v[2]) {
        if (show_in) {
            lv_obj_align(t0_grid_v[1], LV_ALIGN_TOP_LEFT, T0_CANVAS_CX - px_in, 30);
            lv_obj_align(t0_grid_v[2], LV_ALIGN_TOP_LEFT, T0_CANVAS_CX + px_in, 30);
            lv_obj_set_hidden(t0_grid_v[1], false);
            lv_obj_set_hidden(t0_grid_v[2], false);
        } else {
            lv_obj_set_hidden(t0_grid_v[1], true);
            lv_obj_set_hidden(t0_grid_v[2], true);
        }
    }
    if (t0_grid_v[0] && t0_grid_v[3]) {
        if (show_out) {
            lv_obj_align(t0_grid_v[0], LV_ALIGN_TOP_LEFT, T0_CANVAS_CX - px_out, 30);
            lv_obj_align(t0_grid_v[3], LV_ALIGN_TOP_LEFT, T0_CANVAS_CX + px_out, 30);
            lv_obj_set_hidden(t0_grid_v[0], false);
            lv_obj_set_hidden(t0_grid_v[3], false);
        } else {
            lv_obj_set_hidden(t0_grid_v[0], true);
            lv_obj_set_hidden(t0_grid_v[3], true);
        }
    }
    if (t0_log_lbl_inner) {
        if (show_in) {
            lv_label_set_text(t0_log_lbl_inner, current_settings.use_imperial_units ? "3ft" : "1m");
            lv_obj_align(t0_log_lbl_inner, LV_ALIGN_TOP_LEFT, T0_CANVAS_CX + px_in - 14, 220);
            lv_obj_set_hidden(t0_log_lbl_inner, false);
        } else {
            lv_obj_set_hidden(t0_log_lbl_inner, true);
        }
    }
    if (t0_log_lbl_outer) {
        if (show_out) {
            lv_label_set_text(t0_log_lbl_outer, current_settings.use_imperial_units ? "30ft" : "10m");
            lv_obj_align(t0_log_lbl_outer, LV_ALIGN_TOP_LEFT, T0_CANVAS_CX + px_out - 16, 220);
            lv_obj_set_hidden(t0_log_lbl_outer, false);
        } else {
            lv_obj_set_hidden(t0_log_lbl_outer, true);
        }
    }

    // Log cross-track offset (signed left/right)
    int16_t off_mag_px = calc_log_x_px((float)stakeout.offset_m, d_max);
    int16_t off_px = (stakeout.offset_m < 0.0) ? -off_mag_px : off_mag_px;

    // Dynamic Vertical Auto-Framing [36..232] across {0, L, station} so A, B, and R stay inside canvas
    const int16_t y_top      = T0_GUIDE_Y_TOP;
    const int16_t y_bot      = T0_GUIDE_Y_BOT;
    const int16_t y_span     = y_bot - y_top; // 196 px
    const int16_t min_seg_px = 36;
    double seg_len = (stakeout.total_length_m > 0.01) ? stakeout.total_length_m : 0.01;
    double s_m     = stakeout.station_m;

    int16_t y_a = y_bot;
    int16_t y_b = y_top;
    int16_t y_r = y_bot;

    static lv_point_precise_t ext_top_pts[2];
    static lv_point_precise_t ext_bot_pts[2];

    if (s_m < 0.0) {
        // Case 2: Rover behind Start A -> pin B at top, R at bottom, slide A upward + dashed A->Foot
        y_b = y_top;
        y_r = y_bot;
        int16_t seg_px = (int16_t)round((seg_len / (seg_len - s_m)) * (double)y_span);
        if (seg_px < min_seg_px) seg_px = min_seg_px;
        if (seg_px > y_span)     seg_px = y_span;
        y_a = y_b + seg_px;

        lv_obj_set_hidden(t0_line_ext_top, true);
        if (y_r > y_a + 1) {
            ext_bot_pts[0].x = 152;
            ext_bot_pts[0].y = y_a;
            ext_bot_pts[1].x = 152;
            ext_bot_pts[1].y = y_r;
            lv_line_set_points(t0_line_ext_bot, ext_bot_pts, 2);
            lv_obj_set_hidden(t0_line_ext_bot, false);
        } else {
            lv_obj_set_hidden(t0_line_ext_bot, true);
        }
    } else if (s_m > seg_len) {
        // Case 3: Rover past Target B -> pin A at bottom, R at top, slide B downward + dashed B->Foot
        y_a = y_bot;
        y_r = y_top;
        int16_t seg_px = (int16_t)round((seg_len / s_m) * (double)y_span);
        if (seg_px < min_seg_px) seg_px = min_seg_px;
        if (seg_px > y_span)     seg_px = y_span;
        y_b = y_a - seg_px;

        lv_obj_set_hidden(t0_line_ext_bot, true);
        if (y_b > y_r + 1) {
            ext_top_pts[0].x = 152;
            ext_top_pts[0].y = y_b;
            ext_top_pts[1].x = 152;
            ext_top_pts[1].y = y_r;
            lv_line_set_points(t0_line_ext_top, ext_top_pts, 2);
            lv_obj_set_hidden(t0_line_ext_top, false);
        } else {
            lv_obj_set_hidden(t0_line_ext_top, true);
        }
    } else {
        // Case 1: Between A and B -> A at bottom, B at top, R climbs linearly, extensions hidden
        y_a = y_bot;
        y_b = y_top;
        y_r = y_bot - (int16_t)round((s_m / seg_len) * (double)y_span);
        if (y_r < y_top) y_r = y_top;
        if (y_r > y_bot) y_r = y_bot;
        lv_obj_set_hidden(t0_line_ext_top, true);
        lv_obj_set_hidden(t0_line_ext_bot, true);
    }

    // Update dynamic solid baseline A -> B and direction chevron
    static lv_point_precise_t base_pts[2];
    base_pts[0].x = 152;
    base_pts[0].y = y_a;
    base_pts[1].x = 152;
    base_pts[1].y = y_b;
    lv_line_set_points(t0_line_baseline, base_pts, 2);
    lv_obj_set_hidden(t0_line_baseline, false);

    if ((y_a - y_b) >= 22) {
        static lv_point_precise_t arrow_chev_pts[3];
        arrow_chev_pts[0].x = 146;
        arrow_chev_pts[0].y = y_b + 14;
        arrow_chev_pts[1].x = 152;
        arrow_chev_pts[1].y = y_b + 6;
        arrow_chev_pts[2].x = 158;
        arrow_chev_pts[2].y = y_b + 14;
        lv_line_set_points(t0_line_arrow, arrow_chev_pts, 3);
        lv_obj_set_hidden(t0_line_arrow, false);
    } else {
        lv_obj_set_hidden(t0_line_arrow, true);
    }

    int16_t rx = 152 + off_px;
    int16_t ry = y_r;

    // Distinct Circles: Start A (12px Amber/Violet), Target B (16px Green), Rover R (14px Sky Blue)
    lv_obj_align(t0_line_pt_a, LV_ALIGN_TOP_LEFT, 152 - 6, y_a - 6);
    lv_obj_set_style_bg_color(t0_line_pt_a,
                              seg.is_direct_here ? lv_color_hex(0xA855F7) : lv_color_hex(0xF59E0B), 0);
    lv_obj_set_hidden(t0_line_pt_a, false);

    lv_obj_align(t0_target_icon, LV_ALIGN_TOP_LEFT, 152 - 8, y_b - 8);
    lv_obj_set_hidden(t0_target_icon, false);

    lv_obj_align(t0_rover_icon, LV_ALIGN_TOP_LEFT, rx - 7, ry - 7);
    lv_obj_set_style_bg_color(t0_rover_icon,
                              on_line ? lv_color_hex(0x00FF88) : lv_color_hex(0x38BDF8), 0);
    lv_obj_set_hidden(t0_rover_icon, false);

    // Determine Heading Arrow & Active-Leg Color Priority (Hypotenuse R->B first, then Perpendicular Tie)
    uint32_t arrow_hex = 0xFFFFFF;
    uint32_t hyp_hex   = 0x38BDF8;
    int16_t  hyp_width = 2;
    uint32_t tie_hex   = on_line ? 0x00FF88 : 0xF59E0B;
    int16_t  tie_width = 2;

    typedef enum {
        LOCK_NONE = 0,
        LOCK_HYP_GREEN,
        LOCK_HYP_AMBER,
        LOCK_PERP_GREEN,
        LOCK_PERP_AMBER
    } LockState_t;
    LockState_t lock_state = LOCK_NONE;

    if (d->heading_valid) {
        float g_deg = current_settings.line_arrow_green_deg;
        float a_deg = current_settings.line_arrow_amber_deg;
        float err_hyp = fabsf(wrap180f(d->true_heading_deg - brg_b_deg));
        float perp_brg = wrap360f(stakeout.line_bearing_deg + ((stakeout.offset_m < 0.0) ? 90.0f : -90.0f));
        float err_perp = fabsf(wrap180f(d->true_heading_deg - perp_brg));
        bool  allow_perp = (fabs(stakeout.offset_m) > 0.15);

        if (err_hyp <= g_deg) {
            arrow_hex = 0x00FF88;
            hyp_hex   = 0x00FF88;
            hyp_width = 3;
            lock_state = LOCK_HYP_GREEN;
        } else if (err_hyp <= a_deg) {
            arrow_hex = 0xFFB800;
            hyp_hex   = 0xFFB800;
            hyp_width = 3;
            lock_state = LOCK_HYP_AMBER;
        } else if (allow_perp && err_perp <= g_deg) {
            arrow_hex = 0x00FF88;
            tie_hex   = 0x00FF88;
            tie_width = 3;
            lock_state = LOCK_PERP_GREEN;
        } else if (allow_perp && err_perp <= a_deg) {
            arrow_hex = 0xFFB800;
            tie_hex   = 0xFFB800;
            tie_width = 3;
            lock_state = LOCK_PERP_AMBER;
        }

        float rel_hdg_deg = wrap180f(d->true_heading_deg - stakeout.line_bearing_deg);
        draw_barbed_arrow(rx, ry, rel_hdg_deg, 24.0f, arrow_hex);
    } else {
        lv_obj_set_hidden(t0_beam_arrow, true);
    }

    // Draw Hypotenuse R -> B (using dynamic y_b)
    if (dist_b_m > 0.05 && (rx != 152 || ry != y_b)) {
        static lv_point_precise_t hyp_pts[2];
        hyp_pts[0].x = rx;
        hyp_pts[0].y = ry;
        hyp_pts[1].x = 152;
        hyp_pts[1].y = y_b;
        lv_line_set_points(t0_line_hyp, hyp_pts, 2);
        lv_obj_set_style_line_color(t0_line_hyp, lv_color_hex(hyp_hex), 0);
        lv_obj_set_style_line_width(t0_line_hyp, hyp_width, 0);
        lv_obj_set_hidden(t0_line_hyp, false);
    } else {
        lv_obj_set_hidden(t0_line_hyp, true);
    }

    // Draw Perpendicular Tie R -> Baseline
    if (abs(off_px) > 1) {
        static lv_point_precise_t tie_pts[2];
        tie_pts[0].x = rx;
        tie_pts[0].y = ry;
        tie_pts[1].x = 152;
        tie_pts[1].y = ry;
        lv_line_set_points(t0_line_tie, tie_pts, 2);
        lv_obj_set_style_line_color(t0_line_tie, lv_color_hex(tie_hex), 0);
        lv_obj_set_style_line_width(t0_line_tie, tie_width, 0);
        lv_obj_set_hidden(t0_line_tie, false);
    } else {
        lv_obj_set_hidden(t0_line_tie, true);
    }

    // Row 1: Cross-Track Offset + Direct Distance to B (Montserrat 16)
    lv_obj_set_style_text_font(t0_dist_lbl, &lv_font_montserrat_16, 0);
    if (on_line) {
        snprintf(buf, buf_sz, "[OK] ON LINE | B: %s", dist_b_str);
        lv_obj_set_style_text_color(t0_dist_lbl, lv_color_hex(0x00FF88), 0);
    } else if (stakeout.offset_m < 0.0) {
        snprintf(buf, buf_sz, "%s LEFT | B: %s", off_val_str, dist_b_str);
        lv_obj_set_style_text_color(t0_dist_lbl, lv_color_hex(0xF59E0B), 0);
    } else {
        snprintf(buf, buf_sz, "%s RIGHT | B: %s", off_val_str, dist_b_str);
        lv_obj_set_style_text_color(t0_dist_lbl, lv_color_hex(0xF59E0B), 0);
    }
    if (t0_dist_lbl) rover_label_set_text_if_changed(t0_dist_lbl, buf);

    // 3-Tier Vector Graphic Steering Chevrons (>>> / <<<) & On-Track Vertical Bars (|)
    update_steer_graphics(on_line, false, stakeout.offset_m, (lock_state == LOCK_PERP_GREEN));

    // Row 2: Along-Line Station & Remaining (Montserrat 12)
    char sta_str[32], rem_str[24];
    nav_format_station(stakeout.station_m, sta_str, sizeof(sta_str));
    nav_format_distance(fabs(stakeout.remaining_m), rem_str, sizeof(rem_str));
    snprintf(buf, buf_sz, "%s (%.1f%%) | Rem: %s", sta_str, stakeout.pct_complete, rem_str);
    if (t0_offsets_lbl) {
        rover_label_set_text_if_changed(t0_offsets_lbl, buf);
        lv_obj_set_style_text_color(t0_offsets_lbl, lv_color_hex(0x38BDF8), 0);
    }
}
