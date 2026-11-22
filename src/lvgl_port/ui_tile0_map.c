#include "lvgl_ui_internal.h"

// =========================================================================
// Mode 2 Map View HUD Controls, Layers & State
// =========================================================================
lv_obj_t *t0_map_ties[T0_MAP_MAX_TIES] = {0};

lv_obj_t *t0_map_btn_zin     = NULL;
lv_obj_t *t0_map_btn_zout    = NULL;
lv_obj_t *t0_map_btn_fit     = NULL;
lv_obj_t *t0_map_btn_ctr     = NULL;
lv_obj_t *t0_map_boundary    = NULL;
lv_obj_t *t0_map_trail       = NULL;
lv_obj_t *t0_map_pins[T0_MAP_MAX_PINS] = {0};

static double s_map_center_lat      = 0.0;
static double s_map_center_lon      = 0.0;
static float  s_map_m_per_px        = 0.5f;
bool          s_map_auto_fit        = true;
bool          s_map_center_rover    = false;

static void t0_map_zoom_in_cb(lv_event_t *e) {
    (void)e;
    play_rover_beep(1800, 20);
    s_map_auto_fit = false;
    s_map_m_per_px *= 0.70f;
    if (s_map_m_per_px < 0.05f) s_map_m_per_px = 0.05f;
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Map Zoom In -> %.2f m/px\n", s_map_m_per_px);
    }
}

static void t0_map_zoom_out_cb(lv_event_t *e) {
    (void)e;
    play_rover_beep(1400, 20);
    s_map_auto_fit = false;
    s_map_m_per_px *= 1.40f;
    if (s_map_m_per_px > 30.0f) s_map_m_per_px = 30.0f;
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Map Zoom Out -> %.2f m/px\n", s_map_m_per_px);
    }
}

static void t0_map_fit_cb(lv_event_t *e) {
    (void)e;
    play_rover_beep(1600, 20);
    s_map_auto_fit = true;
    s_map_center_rover = false;
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Map Auto-Fit Bounding Box\n");
    }
}

static void t0_map_center_rover_cb(lv_event_t *e) {
    (void)e;
    play_rover_beep(1600, 20);
    s_map_auto_fit = false;
    s_map_center_rover = true;
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Map Center On Rover\n");
    }
}

void t0_canvas_drag_cb(lv_event_t *e) {
    if (s_tile0_nav_mode != 2) return;
    lv_event_code_t code = lv_event_get_code(e);
    static lv_point_t last_pt = {0, 0};
    static bool dragging = false;

    if (code == LV_EVENT_PRESSED) {
        lv_indev_t *indev = lv_indev_active();
        if (indev) {
            lv_indev_get_point(indev, &last_pt);
            dragging = true;
        }
    } else if (code == LV_EVENT_PRESSING) {
        lv_indev_t *indev = lv_indev_active();
        if (indev && dragging) {
            lv_point_t cur_pt;
            lv_indev_get_point(indev, &cur_pt);
            int16_t dx = cur_pt.x - last_pt.x;
            int16_t dy = cur_pt.y - last_pt.y;
            if (abs(dx) > 1 || abs(dy) > 1) {
                s_map_auto_fit = false;
                s_map_center_rover = false;
                float de_m = -(float)dx * s_map_m_per_px;
                float dn_m =  (float)dy * s_map_m_per_px;
                double lat_rad = s_map_center_lat * (M_PI / 180.0);
                double cos_lat = cos(lat_rad);
                if (fabs(cos_lat) < 0.01) cos_lat = 0.01;
                s_map_center_lat += (double)dn_m / 111132.95;
                s_map_center_lon += (double)de_m / (111132.95 * cos_lat);
                last_pt = cur_pt;
            }
        }
    } else if (code == LV_EVENT_RELEASED) {
        dragging = false;
    }
}

static lv_obj_t* create_map_hud_btn(lv_obj_t *parent, const char *txt, int16_t y_pos, lv_event_cb_t cb)
{
    lv_obj_t *btn = lv_button_create(parent);
    lv_obj_set_size(btn, 36, 26);
    lv_obj_align(btn, LV_ALIGN_TOP_RIGHT, -6, y_pos);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x1E293B), 0);
    lv_obj_set_style_border_color(btn, lv_color_hex(0x334155), 0);
    lv_obj_set_style_border_width(btn, 1, 0);
    lv_obj_set_style_radius(btn, 4, 0);
    lv_obj_set_ext_click_area(btn, 4);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_PRESSED, NULL);
    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, txt);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(lbl);
    lv_obj_set_hidden(btn, true);
    return btn;
}

void ui_tile0_map_create(lv_obj_t *canvas)
{
    // Mode 2 Map View Layers & HUD Controls
    t0_map_boundary = lv_line_create(canvas);
    lv_obj_set_style_line_color(t0_map_boundary, lv_color_hex(0x0284C7), 0);
    lv_obj_set_style_line_width(t0_map_boundary, 2, 0);
    lv_obj_set_hidden(t0_map_boundary, true);

    t0_map_trail = lv_line_create(canvas);
    lv_obj_set_style_line_color(t0_map_trail, lv_color_hex(0xF59E0B), 0);
    lv_obj_set_style_line_width(t0_map_trail, 2, 0);
    lv_obj_set_style_line_dash_width(t0_map_trail, 3, 0);
    lv_obj_set_style_line_dash_gap(t0_map_trail, 3, 0);
    lv_obj_set_hidden(t0_map_trail, true);

    for (int t = 0; t < T0_MAP_MAX_TIES; t++) {
        t0_map_ties[t] = lv_line_create(canvas);
        lv_obj_set_style_line_color(t0_map_ties[t], lv_color_hex(0xF97316), 0);
        lv_obj_set_style_line_width(t0_map_ties[t], 2, 0);
        lv_obj_set_style_line_dash_width(t0_map_ties[t], 4, 0);
        lv_obj_set_style_line_dash_gap(t0_map_ties[t], 4, 0);
        lv_obj_set_hidden(t0_map_ties[t], true);
    }

    for (int i = 0; i < T0_MAP_MAX_PINS; i++) {
        t0_map_pins[i] = lv_obj_create(canvas);
        lv_obj_set_size(t0_map_pins[i], 8, 8);
        lv_obj_set_style_radius(t0_map_pins[i], LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(t0_map_pins[i], lv_color_hex(0xFBBF24), 0);
        lv_obj_set_style_border_color(t0_map_pins[i], lv_color_hex(0x000000), 0);
        lv_obj_set_style_border_width(t0_map_pins[i], 1, 0);
        lv_obj_set_hidden(t0_map_pins[i], true);
    }

    t0_map_btn_zin  = create_map_hud_btn(canvas, "+",   48, t0_map_zoom_in_cb);
    t0_map_btn_zout = create_map_hud_btn(canvas, "-",   78, t0_map_zoom_out_cb);
    t0_map_btn_fit  = create_map_hud_btn(canvas, "FIT", 108, t0_map_fit_cb);
    t0_map_btn_ctr  = create_map_hud_btn(canvas, "CTR", 138, t0_map_center_rover_cb);
}

void ui_tile0_map_update(const RoverUIData_t *d, char *buf, size_t buf_sz)
{
    // =============================================================
    // MODE 2: MAP VIEW (Geodetic Canvas with Zoom & Pan)
    // =============================================================
    // Hide Mode 0 overlays
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
    lv_obj_set_hidden(t0_wedge_amber_l, true);
    lv_obj_set_hidden(t0_wedge_amber_r, true);
    lv_obj_set_hidden(t0_wedge_green_l, true);
    lv_obj_set_hidden(t0_wedge_green_r, true);

    // Hide Mode 1 Line Stakeout overlays
    lv_obj_set_hidden(t0_line_baseline, true);
    lv_obj_set_hidden(t0_line_arrow, true);
    lv_obj_set_hidden(t0_line_tie, true);
    lv_obj_set_hidden(t0_line_pt_a, true);
    lv_obj_set_hidden(t0_line_chord, true);

    // Show Map HUD zoom/pan controls
    lv_obj_set_hidden(t0_map_btn_zin, false);
    lv_obj_set_hidden(t0_map_btn_zout, false);
    lv_obj_set_hidden(t0_map_btn_fit, false);
    lv_obj_set_hidden(t0_map_btn_ctr, false);

    // Mode badge in Emerald Green
    lv_label_set_text(t0_mode_badge, "MAP VIEW");
    lv_obj_set_style_bg_color(t0_mode_badge, lv_color_hex(0x10B981), 0);

    // Scale label shows visible window width & meters/px
    float win_w_m = 304.0f * s_map_m_per_px;
    snprintf(buf, buf_sz, "Win: %.0fm (%.2fm/px)", win_w_m, s_map_m_per_px);
    lv_label_set_text(t0_scale_lbl, buf);

    // Top Target Row
    if (d->has_active_target) {
        snprintf(buf, buf_sz, "%s (%d/%d)",
                 d->target_label, d->active_target_idx + 1, d->total_targets);
        if (t0_target_lbl) lv_label_set_text(t0_target_lbl, buf);
        if (t0_notes_lbl) {
            const char *new_notes = d->target_notes[0] ? d->target_notes : "Touch Map to Pan | Zoom with Buttons";
            lv_label_set_text(t0_notes_lbl, new_notes);
        }
    } else {
        if (t0_target_lbl) lv_label_set_text(t0_target_lbl, "No Target (0/0)");
        if (t0_notes_lbl) lv_label_set_text(t0_notes_lbl, "Touch Map to Pan | Zoom with Buttons");
    }

    // Auto-Fit Bounding Box calculation
    if (s_map_auto_fit) {
        double min_lat = d->tip_lat, max_lat = d->tip_lat;
        double min_lon = d->tip_lon, max_lon = d->tip_lon;
        if (fabs(min_lat) < 0.0001 && g_waypoint_count > 0) {
            min_lat = max_lat = g_waypoints[0].latitude;
            min_lon = max_lon = g_waypoints[0].longitude;
        }
        for (int i = 0; i < g_waypoint_count; i++) {
            if (g_waypoints[i].latitude < min_lat) min_lat = g_waypoints[i].latitude;
            if (g_waypoints[i].latitude > max_lat) max_lat = g_waypoints[i].latitude;
            if (g_waypoints[i].longitude < min_lon) min_lon = g_waypoints[i].longitude;
            if (g_waypoints[i].longitude > max_lon) max_lon = g_waypoints[i].longitude;
        }
        s_map_center_lat = (min_lat + max_lat) * 0.5;
        s_map_center_lon = (min_lon + max_lon) * 0.5;
        double dlat_m = (max_lat - min_lat) * 111132.95;
        double dlon_m = (max_lon - min_lon) * 111132.95 * cos(s_map_center_lat * (M_PI / 180.0));
        float sx = (float)dlon_m / 240.0f;
        float sy = (float)dlat_m / 160.0f;
        float fit_scale = (sx > sy) ? sx : sy;
        if (fit_scale < 0.15f) fit_scale = 0.15f;
        if (fit_scale > 25.0f) fit_scale = 25.0f;
        s_map_m_per_px = fit_scale;
    } else if (s_map_center_rover && fabs(d->tip_lat) > 0.0001) {
        s_map_center_lat = d->tip_lat;
        s_map_center_lon = d->tip_lon;
    }

    // Equirectangular local projection parameters
    double lat_rad = s_map_center_lat * (M_PI / 180.0);
    double cos_lat = cos(lat_rad);
    if (fabs(cos_lat) < 0.01) cos_lat = 0.01;

    // Phase 5 Cadastral Boundary Polygon (connects positive poly_order vertices)
    static lv_point_precise_t s_bnd_pts[MAX_WAYPOINTS + 2];
    int bnd_cnt = 0;
    int16_t bnd_indices[MAX_WAYPOINTS];
    int16_t bnd_poly_cnt = waypoint_get_boundary_polygon(bnd_indices, MAX_WAYPOINTS);

    for (int k = 0; k < bnd_poly_cnt; k++) {
        int16_t idx = bnd_indices[k];
        double dN = (g_waypoints[idx].latitude - s_map_center_lat) * 111132.95;
        double dE = (g_waypoints[idx].longitude - s_map_center_lon) * 111132.95 * cos_lat;
        s_bnd_pts[bnd_cnt].x = 152 + (int16_t)roundf((float)dE / s_map_m_per_px);
        s_bnd_pts[bnd_cnt].y = 105 - (int16_t)roundf((float)dN / s_map_m_per_px);
        bnd_cnt++;
    }
    if (bnd_cnt >= 2) {
        if (bnd_cnt >= 3) {
            s_bnd_pts[bnd_cnt] = s_bnd_pts[0];
            bnd_cnt++;
        }
        lv_line_set_points(t0_map_boundary, s_bnd_pts, bnd_cnt);
        lv_obj_set_hidden(t0_map_boundary, false);
    } else {
        lv_obj_set_hidden(t0_map_boundary, true);
    }

    // Render Waypoint Pins (Gold = Boundary Polygon, Orange = Reference / Tie Pin)
    for (int i = 0; i < g_waypoint_count && i < T0_MAP_MAX_PINS; i++) {
        double dN = (g_waypoints[i].latitude - s_map_center_lat) * 111132.95;
        double dE = (g_waypoints[i].longitude - s_map_center_lon) * 111132.95 * cos_lat;
        int16_t sx = 152 + (int16_t)roundf((float)dE / s_map_m_per_px);
        int16_t sy = 105 - (int16_t)roundf((float)dN / s_map_m_per_px);

        if (i == g_active_waypoint_idx) {
            lv_obj_set_hidden(t0_map_pins[i], true);
            lv_obj_align(t0_target_icon, LV_ALIGN_TOP_LEFT, sx - 8, sy - 8);
            lv_obj_set_hidden(t0_target_icon, false);
        } else {
            if (sx >= -5 && sx <= 309 && sy >= -5 && sy <= 215) {
                lv_obj_align(t0_map_pins[i], LV_ALIGN_TOP_LEFT, sx - 4, sy - 4);
                if (g_waypoints[i].is_boundary) {
                    lv_obj_set_style_bg_color(t0_map_pins[i], lv_color_hex(0xFBBF24), 0); // Gold
                } else {
                    lv_obj_set_style_bg_color(t0_map_pins[i], lv_color_hex(0xF97316), 0); // Orange Ref
                }
                lv_obj_set_hidden(t0_map_pins[i], false);
            } else {
                lv_obj_set_hidden(t0_map_pins[i], true);
            }
        }
    }
    for (int i = g_waypoint_count; i < T0_MAP_MAX_PINS; i++) {
        if (t0_map_pins[i]) lv_obj_set_hidden(t0_map_pins[i], true);
    }
    if (g_waypoint_count == 0) {
        lv_obj_set_hidden(t0_target_icon, true);
    }

    // Render Dashed Tie-Lines from Reference Pins to Target Monuments
    int active_ties = 0;
    static lv_point_precise_t s_tie_pts[T0_MAP_MAX_TIES][2];
    for (int i = 0; i < g_waypoint_count && active_ties < T0_MAP_MAX_TIES; i++) {
        if (g_waypoints[i].tie_to_poly_order > 0) {
            int16_t tgt_idx = waypoint_find_by_poly_order(g_waypoints[i].tie_to_poly_order);
            if (tgt_idx >= 0 && tgt_idx < g_waypoint_count) {
                double dN1 = (g_waypoints[i].latitude - s_map_center_lat) * 111132.95;
                double dE1 = (g_waypoints[i].longitude - s_map_center_lon) * 111132.95 * cos_lat;
                int16_t sx1 = 152 + (int16_t)roundf((float)dE1 / s_map_m_per_px);
                int16_t sy1 = 105 - (int16_t)roundf((float)dN1 / s_map_m_per_px);

                double dN2 = (g_waypoints[tgt_idx].latitude - s_map_center_lat) * 111132.95;
                double dE2 = (g_waypoints[tgt_idx].longitude - s_map_center_lon) * 111132.95 * cos_lat;
                int16_t sx2 = 152 + (int16_t)roundf((float)dE2 / s_map_m_per_px);
                int16_t sy2 = 105 - (int16_t)roundf((float)dN2 / s_map_m_per_px);

                s_tie_pts[active_ties][0].x = sx1;
                s_tie_pts[active_ties][0].y = sy1;
                s_tie_pts[active_ties][1].x = sx2;
                s_tie_pts[active_ties][1].y = sy2;
                lv_line_set_points(t0_map_ties[active_ties], s_tie_pts[active_ties], 2);
                lv_obj_set_hidden(t0_map_ties[active_ties], false);
                active_ties++;
            }
        }
    }
    for (int t = active_ties; t < T0_MAP_MAX_TIES; t++) {
        if (t0_map_ties[t]) lv_obj_set_hidden(t0_map_ties[t], true);
    }

    // Render Breadcrumb Trail Polyline (when enabled in settings)
    int bc_count = nav_breadcrumb_get_count();
    if (current_settings.map_show_route_trail && bc_count > 1) {
        static lv_point_precise_t s_trail_pts[NAV_BREADCRUMB_CAPACITY];
        int trail_cnt = 0;
        for (int i = 0; i < bc_count && i < NAV_BREADCRUMB_CAPACITY; i++) {
            NavBreadcrumb_t bc_pt;
            if (nav_breadcrumb_get_point(i, &bc_pt)) {
                double dN = (bc_pt.lat - s_map_center_lat) * 111132.95;
                double dE = (bc_pt.lon - s_map_center_lon) * 111132.95 * cos_lat;
                s_trail_pts[trail_cnt].x = 152 + (int16_t)roundf((float)dE / s_map_m_per_px);
                s_trail_pts[trail_cnt].y = 105 - (int16_t)roundf((float)dN / s_map_m_per_px);
                trail_cnt++;
            }
        }
        if (trail_cnt > 1) {
            lv_line_set_points(t0_map_trail, s_trail_pts, trail_cnt);
            lv_obj_set_hidden(t0_map_trail, false);
        } else {
            lv_obj_set_hidden(t0_map_trail, true);
        }
    } else {
        lv_obj_set_hidden(t0_map_trail, true);
    }

    // Render Live Rover Position & Heading Arrow
    double r_dN = (d->tip_lat - s_map_center_lat) * 111132.95;
    double r_dE = (d->tip_lon - s_map_center_lon) * 111132.95 * cos_lat;
    int16_t r_sx = 152 + (int16_t)roundf((float)r_dE / s_map_m_per_px);
    int16_t r_sy = 105 - (int16_t)roundf((float)r_dN / s_map_m_per_px);
    lv_obj_align(t0_rover_icon, LV_ALIGN_TOP_LEFT, r_sx - 8, r_sy - 8);
    lv_obj_set_style_bg_color(t0_rover_icon, lv_color_hex(0x38BDF8), 0);
    lv_obj_set_hidden(t0_rover_icon, false);

    if (d->heading_valid) {
        float hdg_rad = d->true_heading_deg * (float)(M_PI / 180.0);
        int16_t arrow_dx = (int16_t)(18.0f * sinf(hdg_rad));
        int16_t arrow_dy = (int16_t)(-18.0f * cosf(hdg_rad));
        static lv_point_precise_t r_beam_pts[2];
        r_beam_pts[0].x = r_sx;
        r_beam_pts[0].y = r_sy;
        r_beam_pts[1].x = r_sx + arrow_dx;
        r_beam_pts[1].y = r_sy + arrow_dy;
        lv_line_set_points(t0_beam_arrow, r_beam_pts, 2);
        lv_obj_set_style_line_color(t0_beam_arrow, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_hidden(t0_beam_arrow, false);
    } else {
        lv_obj_set_hidden(t0_beam_arrow, true);
    }

    // Row 1: Distance & Bearing to Target (t0_dist_lbl in Montserrat 20)
    lv_obj_set_style_text_font(t0_dist_lbl, &lv_font_montserrat_16, 0);
    if (d->has_active_target) {
        char dist_str[24];
        nav_format_distance(d->distance_to_target_m, dist_str, sizeof(dist_str));
        snprintf(buf, buf_sz, "%s @ %05.1f deg", dist_str, d->bearing_to_target_deg);
    } else {
        snprintf(buf, buf_sz, "--- m @ --- deg");
    }
    if (t0_dist_lbl) {
        lv_label_set_text(t0_dist_lbl, buf);
        lv_obj_set_style_text_color(t0_dist_lbl, lv_color_hex(0x10B981), 0);
    }

    // Row 2: Target Name / Status (t0_steer_lbl in Montserrat 14)
    if (d->has_active_target) {
        snprintf(buf, buf_sz, "TGT: %s (%d/%d)",
                 d->target_label, d->active_target_idx + 1, d->total_targets);
    } else {
        snprintf(buf, buf_sz, "No Target Active");
    }
    if (t0_steer_lbl) {
        lv_label_set_text(t0_steer_lbl, buf);
        lv_obj_set_style_text_color(t0_steer_lbl, lv_color_hex(0xFFFFFF), 0);
    }

    // Row 3: Map Center Coordinates (t0_offsets_lbl in Montserrat 12)
    snprintf(buf, buf_sz, "Map Ctr: %.5f, %.5f", s_map_center_lat, s_map_center_lon);
    if (t0_offsets_lbl) {
        lv_label_set_text(t0_offsets_lbl, buf);
        lv_obj_set_style_text_color(t0_offsets_lbl, lv_color_hex(0x38BDF8), 0);
    }

    // Row 4: Scale & Waypoint Count (t0_extra_lbl in Montserrat 12)
    snprintf(buf, buf_sz, "Scale: %.2f m/px | Pts: %d", s_map_m_per_px, g_waypoint_count);
    if (t0_extra_lbl) {
        lv_label_set_text(t0_extra_lbl, buf);
        lv_obj_set_style_text_color(t0_extra_lbl, lv_color_hex(0x94A3B8), 0);
        lv_obj_set_hidden(t0_extra_lbl, false);
    }
}
