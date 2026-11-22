#include "lvgl_ui_internal.h"

void ui_tile0_views_update_hud(const RoverUIData_t *d)
{
    // Update Canvas Top-Left Telemetry HUD (Heading & Pitch)
    char hud_buf[32];
    if (t0_hdg_hud_lbl) {
        if (d->heading_valid) {
            const char *src_tag = (d->heading_source == HDG_SRC_DUAL_FIX) ? "FIX" :
                                  (d->heading_source == HDG_SRC_DUAL_FLT) ? "FLT" :
                                  (d->heading_source == HDG_SRC_DISP)     ? "WLK" :
                                  (d->heading_source == HDG_SRC_COMP)     ? "CMP" :
                                  (d->heading_source == HDG_SRC_GYRO)     ? "GYR" :
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
        if (s_tile0_nav_mode == 1) {
            WaypointRecord_t wp_a, wp_b;
            int16_t idx_a = -1, idx_b = -1;
            waypoint_line_stakeout_get_points(&wp_a, &wp_b, &idx_a, &idx_b);
            if (idx_a >= 0 && idx_b >= 0) {
                NavLineStakeout_t line_st;
                nav_compute_line_stakeout(d->tip_lat, d->tip_lon,
                                          wp_a.latitude, wp_a.longitude,
                                          wp_b.latitude, wp_b.longitude,
                                          &line_st);
                snprintf(hud_buf, sizeof(hud_buf), "Line: %05.1f deg", line_st.line_bearing_deg);
                lv_label_set_text(t0_tgt_hdg_lbl, hud_buf);
                float rel_line = line_st.line_bearing_deg - (d->heading_valid ? d->true_heading_deg : 0.0f);
                while (rel_line > 180.0f)  rel_line -= 360.0f;
                while (rel_line < -180.0f) rel_line += 360.0f;
                if (d->heading_valid && fabsf(rel_line) <= 3.0f) {
                    lv_obj_set_style_text_color(t0_tgt_hdg_lbl, lv_color_hex(0x00FF88), 0);
                } else if (d->heading_valid && fabsf(rel_line) <= 15.0f) {
                    lv_obj_set_style_text_color(t0_tgt_hdg_lbl, lv_color_hex(0xFFB800), 0);
                } else {
                    lv_obj_set_style_text_color(t0_tgt_hdg_lbl, lv_color_hex(0xFFFFFF), 0);
                }
            } else {
                lv_label_set_text(t0_tgt_hdg_lbl, "Line: ---.- deg");
                lv_obj_set_style_text_color(t0_tgt_hdg_lbl, lv_color_hex(0xFFFFFF), 0);
            }
        } else if (d->has_active_target) {
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
}

void ui_tile0_views_update_mode1(const RoverUIData_t *d, char *buf, size_t buf_sz)
{
    // =============================================================
    // MODE 1: UNIFIED LINE + BULLSEYE STAKEOUT (Station/Offset & 5m Target)
    // =============================================================
    WaypointRecord_t wp_a, wp_b;
    int16_t idx_a = -1, idx_b = -1;
    waypoint_line_stakeout_get_points(&wp_a, &wp_b, &idx_a, &idx_b);

    // Hide Mode 0 coarse grid/director overlays
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

    // Hide Mode 2 Map View overlays, tie-lines & HUD buttons
    lv_obj_set_hidden(t0_map_boundary, true);
    lv_obj_set_hidden(t0_map_trail, true);
    for (int t = 0; t < T0_MAP_MAX_TIES; t++) {
        if (t0_map_ties[t]) lv_obj_set_hidden(t0_map_ties[t], true);
    }
    for (int i = 0; i < T0_MAP_MAX_PINS; i++) {
        if (t0_map_pins[i]) lv_obj_set_hidden(t0_map_pins[i], true);
    }
    lv_obj_set_hidden(t0_map_btn_zin, true);
    lv_obj_set_hidden(t0_map_btn_zout, true);
    lv_obj_set_hidden(t0_map_btn_fit, true);
    lv_obj_set_hidden(t0_map_btn_ctr, true);

    if (idx_a >= 0 && idx_b >= 0) {
        NavLineStakeout_t stakeout;
        nav_compute_line_stakeout(d->tip_lat, d->tip_lon,
                                  wp_a.latitude, wp_a.longitude,
                                  wp_b.latitude, wp_b.longitude,
                                  &stakeout);

        double dist_b_m = 0.0;
        float brg_b_deg = 0.0f, rel_b_deg = 0.0f, dn_b_m = 0.0f, de_b_m = 0.0f;
        nav_compute_target_guidance(d->tip_lat, d->tip_lon,
                                    d->heading_valid ? d->true_heading_deg : 0.0f,
                                    wp_b.latitude, wp_b.longitude,
                                    &dist_b_m, &brg_b_deg, &rel_b_deg, &dn_b_m, &de_b_m);

        // Top Target Row: A -> B [SWAP]
        snprintf(buf, buf_sz, "%s -> %s [SWAP]", wp_a.label, wp_b.label);
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

        snprintf(buf, buf_sz, "Deed: %s | %s", qbrg, len_str);
        if (t0_notes_lbl) {
            const char *cur_notes = lv_label_get_text(t0_notes_lbl);
            if (!cur_notes || strcmp(cur_notes, buf) != 0) {
                lv_label_set_text(t0_notes_lbl, buf);
            }
        }

        bool on_line = (fabs(stakeout.offset_m) <= 0.02); // 2 cm on-line threshold
        char off_val_str[24];
        nav_format_distance(fabs(stakeout.offset_m), off_val_str, sizeof(off_val_str));

        if (d->mode1_bullseye_active) {
            // ---------------------------------------------------------
            // Sub-view A (< 5.0 m w/ 4.8/5.3 m hysteresis): Bullseye + A->B Chord
            // ---------------------------------------------------------
            lv_obj_set_hidden(t0_line_baseline, true);
            lv_obj_set_hidden(t0_line_arrow, true);
            lv_obj_set_hidden(t0_line_tie, true);
            lv_obj_set_hidden(t0_line_pt_a, true);

            lv_obj_set_hidden(t0_bullseye_ring1, false);
            lv_obj_set_hidden(t0_bullseye_ring2, false);
            lv_obj_set_hidden(t0_bullseye_center, false);
            lv_obj_set_hidden(t0_cross_h, false);
            lv_obj_set_hidden(t0_cross_v, false);

            lv_label_set_text(t0_mode_badge, "LINE BULLSEYE");
            lv_obj_set_style_bg_color(t0_mode_badge, lv_color_hex(0x00AA55), 0);

            float px_per_m;
            if (dist_b_m <= 1.5) {
                px_per_m = 73.0f / 1.5f;
                lv_obj_set_size(t0_bullseye_ring2, 48, 48); // 0.5 m inner ring
                lv_label_set_text(t0_scale_lbl, "Ring: 1.5m / 0.5m");
            } else {
                px_per_m = 73.0f / 5.0f;
                lv_obj_set_size(t0_bullseye_ring2, 44, 44); // 1.5 m inner ring
                lv_label_set_text(t0_scale_lbl, "Ring: 5.0m / 1.5m");
            }

            // Overlaid A -> B property chord drawn across the Bullseye centered on Point B
            float line_rad = stakeout.line_bearing_deg * (float)(M_PI / 180.0);
            float ux = sinf(line_rad);
            float uy = -cosf(line_rad);
            static lv_point_precise_t chord_pts[2];
            chord_pts[0].x = 152 - (int16_t)roundf(73.0f * ux);
            chord_pts[0].y = 105 - (int16_t)roundf(73.0f * uy);
            chord_pts[1].x = 152 + (int16_t)roundf(73.0f * ux);
            chord_pts[1].y = 105 + (int16_t)roundf(73.0f * uy);
            lv_line_set_points(t0_line_chord, chord_pts, 2);
            lv_obj_set_style_line_color(t0_line_chord, on_line ? lv_color_hex(0x00FF88) : lv_color_hex(0x00D8FF), 0);
            lv_obj_set_hidden(t0_line_chord, false);

            // Target B pin centered at Bullseye origin
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
            if (r_px > 78) r_px = 78; else if (r_px < -78) r_px = -78;
            if (r_py > 78) r_py = 78; else if (r_py < -78) r_py = -78;
            lv_obj_align(t0_rover_icon, LV_ALIGN_CENTER, r_px, r_py);
            lv_obj_set_style_bg_color(t0_rover_icon, on_line ? lv_color_hex(0x00FF88) : lv_color_hex(0x38BDF8), 0);
            lv_obj_set_hidden(t0_rover_icon, false);

            if (d->heading_valid) {
                float hdg_rad = d->true_heading_deg * (float)(M_PI / 180.0);
                int16_t arrow_dx = (int16_t)roundf(22.0f * sinf(hdg_rad));
                int16_t arrow_dy = (int16_t)roundf(-22.0f * cosf(hdg_rad));
                static lv_point_precise_t b_beam_pts[2];
                b_beam_pts[0].x = 152 + r_px;
                b_beam_pts[0].y = 105 + r_py;
                b_beam_pts[1].x = 152 + r_px + arrow_dx;
                b_beam_pts[1].y = 105 + r_py + arrow_dy;
                lv_line_set_points(t0_beam_arrow, b_beam_pts, 2);
                lv_obj_set_style_line_color(t0_beam_arrow, lv_color_hex(0xFFCC00), 0);
                lv_obj_set_hidden(t0_beam_arrow, false);
            } else {
                lv_obj_set_hidden(t0_beam_arrow, true);
            }

            char dist_b_str[24];
            nav_format_distance(dist_b_m, dist_b_str, sizeof(dist_b_str));
            if (t0_dist_lbl) {
                lv_obj_set_style_text_font(t0_dist_lbl, &lv_font_montserrat_24, 0);
                lv_label_set_text(t0_dist_lbl, dist_b_str);
                lv_obj_set_style_text_color(t0_dist_lbl, (dist_b_m < 0.02) ? lv_color_hex(0x00FF00) : lv_color_hex(0xFFFFFF), 0);
            }

            if (t0_steer_lbl) {
                if (dist_b_m < 0.02) {
                    snprintf(buf, buf_sz, "[OK] PIN ACQUIRED! (Acc: %.1f cm)", d->h_acc_m * 100.0f);
                    lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0x00FF00), 0);
                } else if (on_line) {
                    snprintf(buf, buf_sz, "Line: [OK] ON LINE (%s)", off_val_str);
                    lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0x00FF88), 0);
                } else if (stakeout.offset_m < 0.0) {
                    snprintf(buf, buf_sz, "Line: %s LEFT (Move R >>>)", off_val_str);
                    lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0xF59E0B), 0);
                } else {
                    snprintf(buf, buf_sz, "Line: %s RIGHT (<<< Move L)", off_val_str);
                    lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0xF59E0B), 0);
                }
                lv_label_set_text(t0_steer_lbl, buf);
            }

            char dn_b_str[24], de_b_str[24];
            nav_format_offset(dn_b_m, dn_b_str, sizeof(dn_b_str));
            nav_format_offset(de_b_m, de_b_str, sizeof(de_b_str));
            if (t0_offsets_lbl) {
                snprintf(buf, buf_sz, "dN: %s   dE: %s", dn_b_str, de_b_str);
                lv_label_set_text(t0_offsets_lbl, buf);
                lv_obj_set_style_text_color(t0_offsets_lbl, lv_color_hex(0x00D8FF), 0);
            }

            if (t0_extra_lbl) {
                snprintf(buf, buf_sz, "Deed: %s (%.1f deg)", qbrg, stakeout.line_bearing_deg);
                lv_label_set_text(t0_extra_lbl, buf);
                lv_obj_set_style_text_color(t0_extra_lbl, lv_color_hex(0x94A3B8), 0);
                lv_obj_set_hidden(t0_extra_lbl, false);
            }
        } else {
            // ---------------------------------------------------------
            // Sub-view B (Outside 5.0 m): Line-Up Baseline A -> B
            // ---------------------------------------------------------
            lv_obj_set_hidden(t0_bullseye_ring1, true);
            lv_obj_set_hidden(t0_bullseye_ring2, true);
            lv_obj_set_hidden(t0_bullseye_center, true);
            lv_obj_set_hidden(t0_cross_h, true);
            lv_obj_set_hidden(t0_cross_v, true);
            lv_obj_set_hidden(t0_line_chord, true);

            lv_obj_set_hidden(t0_line_baseline, false);
            lv_obj_set_hidden(t0_line_arrow, false);
            lv_obj_set_hidden(t0_line_pt_a, false);

            // Mode badge in Amber
            lv_label_set_text(t0_mode_badge, "LINE STAKEOUT");
            lv_obj_set_style_bg_color(t0_mode_badge, lv_color_hex(0xD97706), 0);

            // Scale label shows compact distance to Point B
            char dist_b_str[24];
            nav_format_distance(dist_b_m, dist_b_str, sizeof(dist_b_str));
            snprintf(buf, buf_sz, "To B: %s", dist_b_str);
            lv_label_set_text(t0_scale_lbl, buf);

            // Canvas coordinates (pulled inward for top/bottom/side whitespace):
            // Canvas size: 304 x 210. Baseline runs from Point A (152, 178) to Point B (152, 46) (132 px).
            // Cross-track pixel scaling: 52 px = 1.0 m, horizontal clamp [-104, +104] px.
            float px_per_m = 52.0f;
            int16_t off_px = (int16_t)(stakeout.offset_m * px_per_m);
            if (off_px > 104) off_px = 104;
            if (off_px < -104) off_px = -104;

            // Along-line Station progress:
            // Point A at bottom (y = 178), Point B at top (y = 46)
            float pct = stakeout.pct_complete;
            if (pct < 0.0f) pct = 0.0f;
            if (pct > 100.0f) pct = 100.0f;
            int16_t sta_py = 178 - (int16_t)((pct / 100.0f) * 132.0f);
            if (sta_py < 46) sta_py = 46;
            if (sta_py > 178) sta_py = 178;

            // Target Point B marker fixed at top of line (152, 46)
            lv_obj_align(t0_target_icon, LV_ALIGN_TOP_LEFT, 152 - 8, 46 - 8);
            lv_obj_set_hidden(t0_target_icon, false);

            // Point A marker fixed at bottom of line (152, 178)
            lv_obj_align(t0_line_pt_a, LV_ALIGN_TOP_LEFT, 152 - 6, 178 - 6);
            lv_obj_set_hidden(t0_line_pt_a, false);

            // Rover marker at (152 + off_px, sta_py)
            lv_obj_align(t0_rover_icon, LV_ALIGN_TOP_LEFT, 152 + off_px - 8, sta_py - 8);
            lv_obj_set_hidden(t0_rover_icon, false);

            if (on_line) {
                lv_obj_set_style_bg_color(t0_rover_icon, lv_color_hex(0x00FF88), 0);
            } else {
                lv_obj_set_style_bg_color(t0_rover_icon, lv_color_hex(0xF59E0B), 0);
            }

            // Perpendicular tie line connecting Rover (152 + off_px, sta_py) to Baseline (152, sta_py)
            if (abs(off_px) > 1) {
                static lv_point_precise_t tie_pts[2];
                tie_pts[0].x = 152 + off_px;
                tie_pts[0].y = sta_py;
                tie_pts[1].x = 152;
                tie_pts[1].y = sta_py;
                lv_line_set_points(t0_line_tie, tie_pts, 2);
                lv_obj_set_style_line_color(t0_line_tie, on_line ? lv_color_hex(0x00FF88) : lv_color_hex(0xF59E0B), 0);
                lv_obj_set_hidden(t0_line_tie, false);
            } else {
                lv_obj_set_hidden(t0_line_tie, true);
            }

            // Line-Up heading arrow rotated relative to baseline bearing (true_heading - line_bearing)
            if (d->heading_valid) {
                float rel_hdg_rad = (d->true_heading_deg - stakeout.line_bearing_deg) * (float)(M_PI / 180.0);
                int16_t arrow_dx = (int16_t)roundf(20.0f * sinf(rel_hdg_rad));
                int16_t arrow_dy = (int16_t)roundf(-20.0f * cosf(rel_hdg_rad));
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

            // Row 1: Unsigned Offset Display (t0_dist_lbl in Montserrat 16)
            lv_obj_set_style_text_font(t0_dist_lbl, &lv_font_montserrat_16, 0);
            if (on_line) {
                snprintf(buf, buf_sz, "[OK] ON LINE (%s)", off_val_str);
                lv_obj_set_style_text_color(t0_dist_lbl, lv_color_hex(0x00FF88), 0);
            } else if (stakeout.offset_m < 0.0) {
                snprintf(buf, buf_sz, "%s LEFT", off_val_str);
                lv_obj_set_style_text_color(t0_dist_lbl, lv_color_hex(0xF59E0B), 0);
            } else {
                snprintf(buf, buf_sz, "%s RIGHT", off_val_str);
                lv_obj_set_style_text_color(t0_dist_lbl, lv_color_hex(0xF59E0B), 0);
            }
            if (t0_dist_lbl) lv_label_set_text(t0_dist_lbl, buf);

            // Row 2: Lateral Steer Action Prompt (t0_steer_lbl in Montserrat 14)
            if (on_line) {
                snprintf(buf, buf_sz, "[OK] ON LINE - PROCEED ON BEARING");
                lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0x00FF88), 0);
            } else if (stakeout.offset_m < 0.0) {
                snprintf(buf, buf_sz, "MOVE RIGHT TO LINE >>>");
                lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0xF59E0B), 0);
            } else {
                snprintf(buf, buf_sz, "<<< MOVE LEFT TO LINE");
                lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0xF59E0B), 0);
            }
            if (t0_steer_lbl) lv_label_set_text(t0_steer_lbl, buf);

            // Row 3: Longitudinal Progress (t0_offsets_lbl in Montserrat 12)
            char sta_str[32];
            nav_format_station(stakeout.station_m, sta_str, sizeof(sta_str));
            char rem_str[24];
            nav_format_distance(fabs(stakeout.remaining_m), rem_str, sizeof(rem_str));
            snprintf(buf, buf_sz, "%s (%.1f%%) | Rem: %s", sta_str, stakeout.pct_complete, rem_str);
            if (t0_offsets_lbl) {
                lv_label_set_text(t0_offsets_lbl, buf);
                lv_obj_set_style_text_color(t0_offsets_lbl, lv_color_hex(0x38BDF8), 0);
            }

            // Row 4: Deed Bearing & Angle (t0_extra_lbl in Montserrat 12)
            snprintf(buf, buf_sz, "Deed: %s (%.1f deg)", qbrg, stakeout.line_bearing_deg);
            if (t0_extra_lbl) {
                lv_label_set_text(t0_extra_lbl, buf);
                lv_obj_set_style_text_color(t0_extra_lbl, lv_color_hex(0x94A3B8), 0);
                lv_obj_set_hidden(t0_extra_lbl, false);
            }
        }
    } else {
        // Need at least 2 points to define baseline
        lv_label_set_text(t0_mode_badge, "LINE STAKEOUT");
        lv_obj_set_style_bg_color(t0_mode_badge, lv_color_hex(0xD97706), 0);
        if (t0_dist_lbl) {
            lv_obj_set_style_text_font(t0_dist_lbl, &lv_font_montserrat_16, 0);
            lv_label_set_text(t0_dist_lbl, "Need >= 2 Waypoints");
            lv_obj_set_style_text_color(t0_dist_lbl, lv_color_hex(0xEF4444), 0);
        }
        if (t0_steer_lbl) {
            lv_label_set_text(t0_steer_lbl, "Select Points in Tile 3 CSV Browser");
            lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0x94A3B8), 0);
        }
        if (t0_offsets_lbl) lv_label_set_text(t0_offsets_lbl, "Line Stakeout requires A and B");
        if (t0_extra_lbl) lv_obj_set_hidden(t0_extra_lbl, true);
        lv_obj_set_hidden(t0_bullseye_ring1, true);
        lv_obj_set_hidden(t0_bullseye_ring2, true);
        lv_obj_set_hidden(t0_bullseye_center, true);
        lv_obj_set_hidden(t0_cross_h, true);
        lv_obj_set_hidden(t0_cross_v, true);
        lv_obj_set_hidden(t0_line_baseline, true);
        lv_obj_set_hidden(t0_line_arrow, true);
        lv_obj_set_hidden(t0_line_tie, true);
        lv_obj_set_hidden(t0_line_pt_a, true);
        lv_obj_set_hidden(t0_line_chord, true);
        lv_obj_set_hidden(t0_target_icon, true);
        lv_obj_set_hidden(t0_beam_arrow, true);
    }
}

void ui_tile0_views_update_mode0(const RoverUIData_t *d, char *buf, size_t buf_sz)
{
    if (d->has_active_target) {
        // =============================================================
        // MODE 0: TACTICAL GUIDANCE (Course-Up Director or Bullseye)
        // =============================================================
        char dist_str[24], dn_str[24], de_str[24];
        nav_format_distance(d->distance_to_target_m, dist_str, sizeof(dist_str));
        nav_format_offset(d->d_north_m, dn_str, sizeof(dn_str));
        nav_format_offset(d->d_east_m, de_str, sizeof(de_str));

        // Hide Line Stakeout overlays
        lv_obj_set_hidden(t0_line_baseline, true);
        lv_obj_set_hidden(t0_line_arrow, true);
        lv_obj_set_hidden(t0_line_tie, true);
        lv_obj_set_hidden(t0_line_pt_a, true);
        lv_obj_set_hidden(t0_line_chord, true);

        // Hide Mode 2 Map View overlays & HUD buttons
        lv_obj_set_hidden(t0_map_boundary, true);
        lv_obj_set_hidden(t0_map_trail, true);
        for (int t = 0; t < T0_MAP_MAX_TIES; t++) {
            if (t0_map_ties[t]) lv_obj_set_hidden(t0_map_ties[t], true);
        }
        for (int i = 0; i < T0_MAP_MAX_PINS; i++) {
            if (t0_map_pins[i]) lv_obj_set_hidden(t0_map_pins[i], true);
        }
        lv_obj_set_hidden(t0_map_btn_zin, true);
        lv_obj_set_hidden(t0_map_btn_zout, true);
        lv_obj_set_hidden(t0_map_btn_fit, true);
        lv_obj_set_hidden(t0_map_btn_ctr, true);

        // Hide Row 4 extra label in Mode 0
        if (t0_extra_lbl) lv_obj_set_hidden(t0_extra_lbl, true);

        // Target & Notes Row
        snprintf(buf, buf_sz, "%s (%d/%d)",
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

        // Row 1: Target Distance in Montserrat 24
        lv_obj_set_style_text_font(t0_dist_lbl, &lv_font_montserrat_24, 0);
        if (t0_dist_lbl) {
            lv_label_set_text(t0_dist_lbl, dist_str);
            lv_obj_set_style_text_color(t0_dist_lbl, lv_color_hex(0xFFFFFF), 0);
        }

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
            lv_obj_set_hidden(t0_track_arrow, true);
            lv_obj_set_hidden(t0_wedge_amber_l, true);
            lv_obj_set_hidden(t0_wedge_amber_r, true);
            lv_obj_set_hidden(t0_wedge_green_l, true);
            lv_obj_set_hidden(t0_wedge_green_r, true);

            // Show bullseye rings
            lv_obj_set_size(t0_bullseye_ring2, 60, 60);
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
            lv_obj_set_style_bg_color(t0_rover_icon, lv_color_hex(0x38BDF8), 0);
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
                snprintf(buf, buf_sz, "[OK] PIN ACQUIRED! (Acc: %.1f cm)", d->h_acc_m * 100.0f);
                lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0x00FF00), 0);
            } else {
                lv_obj_set_style_bg_color(t0_bullseye_center, lv_color_hex(0xFF0055), 0);
                lv_obj_set_style_border_color(t0_bullseye_center, lv_color_hex(0xFFAABB), 0);
                snprintf(buf, buf_sz, "Stakeout: Align Tip to Crosshair");
                lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0xFFCC00), 0);
            }
            if (t0_steer_lbl) lv_label_set_text(t0_steer_lbl, buf);
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
            snprintf(buf, buf_sz, "Range: %s", scale_str);
            lv_label_set_text(t0_scale_lbl, buf);

            // Fixed Course-Up Anchor Positions:
            // Rover Origin at bottom center (152, 170)
            int16_t rover_x = 152;
            int16_t rover_y = 170;
            lv_obj_align(t0_rover_icon, LV_ALIGN_TOP_LEFT, rover_x - 8, rover_y - 8);
            lv_obj_set_style_bg_color(t0_rover_icon, lv_color_hex(0x38BDF8), 0);
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
                    snprintf(buf, buf_sz, "[OK] ON TRACK - Aimed at Pin");
                    lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0x00FF88), 0);
                } else if (abs_turn <= 15.0f) {
                    // Tier 2: Outer Amber Wedge (+/-4 to +/-15 deg)
                    lv_obj_set_style_line_color(t0_beam_arrow, lv_color_hex(0xFFB800), 0);
                    if (rel_turn > 0.0f) {
                        snprintf(buf, buf_sz, ">>> Turn Right %.0f deg", rel_turn);
                    } else {
                        snprintf(buf, buf_sz, "<<< Turn Left %.0f deg", -rel_turn);
                    }
                    lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0xFFB800), 0);
                } else {
                    // Tier 3: Outside Wedges (> 15 deg)
                    lv_obj_set_style_line_color(t0_beam_arrow, lv_color_hex(0xFFFFFF), 0);
                    if (rel_turn > 0.0f) {
                        snprintf(buf, buf_sz, ">>> Turn Right %.0f deg", rel_turn);
                    } else {
                        snprintf(buf, buf_sz, "<<< Turn Left %.0f deg", -rel_turn);
                    }
                    lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0xFFFFFF), 0);
                }
            } else {
                lv_obj_set_hidden(t0_beam_arrow, true);
                snprintf(buf, buf_sz, "Brg: %.1f deg | Acquiring Heading...", d->bearing_to_target_deg);
                lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0xFF8800), 0);
            }
            if (t0_steer_lbl) lv_label_set_text(t0_steer_lbl, buf);
        }

        snprintf(buf, buf_sz, "dN: %s   dE: %s", dn_str, de_str);
        if (t0_offsets_lbl) {
            lv_label_set_text(t0_offsets_lbl, buf);
            lv_obj_set_style_text_color(t0_offsets_lbl, lv_color_hex(0x00D8FF), 0);
        }
    } else {
        // Mode 0: No target active
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
        if (t0_dist_lbl) {
            lv_obj_set_style_text_font(t0_dist_lbl, &lv_font_montserrat_24, 0);
            lv_label_set_text(t0_dist_lbl, "---");
        }
        if (t0_offsets_lbl) lv_label_set_text(t0_offsets_lbl, "dN: ---   dE: ---");
        if (t0_extra_lbl) lv_obj_set_hidden(t0_extra_lbl, true);
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

        // Hide Line Stakeout overlays
        lv_obj_set_hidden(t0_line_baseline, true);
        lv_obj_set_hidden(t0_line_arrow, true);
        lv_obj_set_hidden(t0_line_tie, true);
        lv_obj_set_hidden(t0_line_pt_a, true);
        lv_obj_set_hidden(t0_line_chord, true);

        // Hide Mode 2 Map View overlays & HUD buttons
        lv_obj_set_hidden(t0_map_boundary, true);
        lv_obj_set_hidden(t0_map_trail, true);
        for (int t = 0; t < T0_MAP_MAX_TIES; t++) {
            if (t0_map_ties[t]) lv_obj_set_hidden(t0_map_ties[t], true);
        }
        for (int i = 0; i < T0_MAP_MAX_PINS; i++) {
            if (t0_map_pins[i]) lv_obj_set_hidden(t0_map_pins[i], true);
        }
        lv_obj_set_hidden(t0_map_btn_zin, true);
        lv_obj_set_hidden(t0_map_btn_zout, true);
        lv_obj_set_hidden(t0_map_btn_fit, true);
        lv_obj_set_hidden(t0_map_btn_ctr, true);

        lv_obj_align(t0_rover_icon, LV_ALIGN_TOP_LEFT, 152 - 8, 170 - 8);
        lv_obj_set_hidden(t0_rover_icon, false);
    }
}
