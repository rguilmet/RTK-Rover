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
lv_obj_t *t0_map_active_seg  = NULL;
lv_obj_t *t0_map_trail       = NULL;
lv_obj_t *t0_map_pins[T0_MAP_MAX_PINS] = {0};
lv_obj_t *t0_map_base_tri    = NULL;
lv_obj_t *t0_map_base_dot    = NULL;
lv_obj_t *t0_map_base_lbl    = NULL;

static double s_map_center_lat          = 0.0;
static double s_map_center_lon          = 0.0;
static float  s_map_m_per_px            = 0.5f;
static bool   s_map_zoom_loaded         = false;
static bool   s_map_center_initialized  = false;
bool          s_map_auto_fit            = true;
bool          s_map_center_rover        = false;

void ui_tile0_map_on_file_changed(void)
{
    s_map_center_initialized = false;
}

static void t0_map_zoom_in_cb(lv_event_t *e) {
    (void)e;
    play_rover_beep(1800, 20);
    s_map_auto_fit = false;
    s_map_m_per_px *= 0.70f;
    if (s_map_m_per_px < 0.05f) s_map_m_per_px = 0.05f;
    current_settings.map_zoom_m_per_px = s_map_m_per_px;
    mark_settings_dirty();
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
    current_settings.map_zoom_m_per_px = s_map_m_per_px;
    mark_settings_dirty();
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Map Zoom Out -> %.2f m/px\n", s_map_m_per_px);
    }
}

static void t0_map_fit_cb(lv_event_t *e) {
    (void)e;
    play_rover_beep(1600, 20);
    s_map_auto_fit = true;
    s_map_center_rover = false;
    current_settings.map_zoom_m_per_px = 0.0f;
    mark_settings_dirty();
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
    if (s_tile0_nav_mode != 1) return;
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
                s_map_center_initialized = true;
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

static lv_obj_t* create_map_hud_btn(lv_obj_t *parent, const char *txt,
                                    int16_t w, int16_t x_pos,
                                    const lv_font_t *font, lv_event_cb_t cb)
{
    lv_obj_t *btn = lv_button_create(parent);
    lv_obj_set_size(btn, w, 26);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_LEFT, x_pos, -6);
    lv_obj_set_style_bg_color(btn, lv_color_hex(0x1E293B), 0);
    lv_obj_set_style_border_color(btn, lv_color_hex(0x334155), 0);
    lv_obj_set_style_border_width(btn, 1, 0);
    lv_obj_set_style_radius(btn, 4, 0);
    lv_obj_set_ext_click_area(btn, 2);
    lv_obj_add_event_cb(btn, cb, LV_EVENT_PRESSED, NULL);
    lv_obj_t *lbl = lv_label_create(btn);
    lv_label_set_text(lbl, txt);
    lv_obj_set_style_text_font(lbl, font, 0);
    lv_obj_center(lbl);
    lv_obj_set_hidden(btn, true);
    return btn;
}

static void add_unit_ray(float *rx, float *ry, int *cnt, int max_cnt, int16_t dx, int16_t dy)
{
    if (*cnt >= max_cnt) return;
    float mag = hypotf((float)dx, (float)dy);
    if (mag <= 1.0f) return;
    rx[*cnt] = (float)dx / mag;
    ry[*cnt] = (float)dy / mag;
    (*cnt)++;
}

static bool map_pt_in_view(int16_t sx, int16_t sy)
{
    return (sx >= 4 && sx <= (T0_CANVAS_W - 4) && sy >= 24 && sy <= 238);
}

static void add_pin_edge_rays(int16_t px, int16_t py, int16_t wp_idx,
                              const int16_t *bnd_indices, const lv_point_precise_t *bnd_pts, int16_t poly_cnt,
                              const lv_point_precise_t (*tie_pts)[2], int tie_cnt,
                              float *rx, float *ry, int *cnt, int max_cnt)
{
    for (int16_t k = 0; wp_idx >= 0 && k < poly_cnt; k++) {
        if (bnd_indices[k] == wp_idx) {
            int16_t prev_k = (k > 0) ? (k - 1) : ((poly_cnt >= 3) ? (poly_cnt - 1) : -1);
            int16_t next_k = (k + 1 < poly_cnt) ? (k + 1) : ((poly_cnt >= 3) ? 0 : -1);
            if (prev_k >= 0) add_unit_ray(rx, ry, cnt, max_cnt, bnd_pts[prev_k].x - px, bnd_pts[prev_k].y - py);
            if (next_k >= 0) add_unit_ray(rx, ry, cnt, max_cnt, bnd_pts[next_k].x - px, bnd_pts[next_k].y - py);
            break;
        }
    }
    for (int t = 0; t < tie_cnt; t++) {
        if (abs(tie_pts[t][0].x - px) <= 2 && abs(tie_pts[t][0].y - py) <= 2) {
            add_unit_ray(rx, ry, cnt, max_cnt, tie_pts[t][1].x - px, tie_pts[t][1].y - py);
        } else if (abs(tie_pts[t][1].x - px) <= 2 && abs(tie_pts[t][1].y - py) <= 2) {
            add_unit_ray(rx, ry, cnt, max_cnt, tie_pts[t][0].x - px, tie_pts[t][0].y - py);
        }
    }
}

void ui_tile0_map_create(lv_obj_t *canvas)
{
    // Map View Layers & HUD Controls
    t0_map_boundary = lv_line_create(canvas);
    lv_obj_set_style_line_color(t0_map_boundary, lv_color_hex(0x0284C7), 0);
    lv_obj_set_style_line_width(t0_map_boundary, 2, 0);
    lv_obj_set_hidden(t0_map_boundary, true);

    t0_map_active_seg = lv_line_create(canvas);
    lv_obj_set_style_line_color(t0_map_active_seg, lv_color_hex(0x00FF88), 0);
    lv_obj_set_style_line_width(t0_map_active_seg, 4, 0);
    lv_obj_set_hidden(t0_map_active_seg, true);

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

    // Survey-Control Base Station Symbol (Hollow Magenta Triangle + White Center Dot + BS Label)
    t0_map_base_tri = lv_line_create(canvas);
    lv_obj_set_style_line_color(t0_map_base_tri, lv_color_hex(0xFF44AA), 0);
    lv_obj_set_style_line_width(t0_map_base_tri, 2, 0);
    lv_obj_set_style_line_rounded(t0_map_base_tri, true, 0);
    lv_obj_set_hidden(t0_map_base_tri, true);

    t0_map_base_dot = lv_obj_create(canvas);
    lv_obj_set_size(t0_map_base_dot, 4, 4);
    lv_obj_set_style_radius(t0_map_base_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(t0_map_base_dot, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_border_width(t0_map_base_dot, 0, 0);
    lv_obj_set_clickable(t0_map_base_dot, false);
    lv_obj_set_scrollable(t0_map_base_dot, false);
    lv_obj_set_hidden(t0_map_base_dot, true);

    t0_map_base_lbl = lv_label_create(canvas);
    lv_label_set_text(t0_map_base_lbl, "BS");
    lv_obj_set_size(t0_map_base_lbl, 20, 14);
    lv_obj_set_style_text_align(t0_map_base_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_text_font(t0_map_base_lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_color(t0_map_base_lbl, lv_color_hex(0xFF44AA), 0);
    lv_obj_set_clickable(t0_map_base_lbl, false);
    lv_obj_set_hidden(t0_map_base_lbl, true);

    t0_map_btn_zin  = create_map_hud_btn(canvas, "+",   46,   6, &lv_font_montserrat_16, t0_map_zoom_in_cb);
    t0_map_btn_zout = create_map_hud_btn(canvas, "-",   46,  57, &lv_font_montserrat_16, t0_map_zoom_out_cb);
    t0_map_btn_fit  = create_map_hud_btn(canvas, "FIT", 48, 108, &lv_font_montserrat_12, t0_map_fit_cb);
    t0_map_btn_ctr  = create_map_hud_btn(canvas, "CTR", 48, 161, &lv_font_montserrat_12, t0_map_center_rover_cb);
}

void ui_tile0_map_update(const RoverUIData_t *d, char *buf, size_t buf_sz)
{
    // =============================================================
    // MODE 1: MAP VIEW (Geodetic Canvas with Sticky Zoom & Active Segment)
    // =============================================================
    // Hide Bullseye & Unified Triangle Guidance overlays
    lv_obj_set_hidden(t0_bullseye_ring1, true);
    lv_obj_set_hidden(t0_bullseye_ring2, true);
    lv_obj_set_hidden(t0_bullseye_center, true);
    lv_obj_set_hidden(t0_cross_h, true);
    lv_obj_set_hidden(t0_cross_v, true);
    lv_obj_set_hidden(t0_line_chord, true);
    for (int i = 0; i < T0_GRID_V_LINES; i++) {
        if (t0_grid_v[i]) lv_obj_set_hidden(t0_grid_v[i], true);
    }
    if (t0_log_lbl_inner) lv_obj_set_hidden(t0_log_lbl_inner, true);
    if (t0_log_lbl_outer) lv_obj_set_hidden(t0_log_lbl_outer, true);
    if (t0_anchor_btn)    lv_obj_set_hidden(t0_anchor_btn, true);

    lv_obj_set_hidden(t0_line_baseline, true);
    lv_obj_set_hidden(t0_line_ext_top, true);
    lv_obj_set_hidden(t0_line_ext_bot, true);
    lv_obj_set_hidden(t0_line_arrow, true);
    lv_obj_set_hidden(t0_line_tie, true);
    lv_obj_set_hidden(t0_line_hyp, true);
    for (int i = 0; i < 3; i++) {
        if (t0_chev_left[i])  lv_obj_set_hidden(t0_chev_left[i], true);
        if (t0_chev_right[i]) lv_obj_set_hidden(t0_chev_right[i], true);
    }
    if (t0_ontrack_bar_l) lv_obj_set_hidden(t0_ontrack_bar_l, true);
    if (t0_ontrack_bar_r) lv_obj_set_hidden(t0_ontrack_bar_r, true);

    // Show Map HUD zoom/pan controls
    lv_obj_set_hidden(t0_map_btn_zin, false);
    lv_obj_set_hidden(t0_map_btn_zout, false);
    lv_obj_set_hidden(t0_map_btn_fit, false);
    lv_obj_set_hidden(t0_map_btn_ctr, false);

    // Restore persisted sticky zoom level once on first Map render
    if (!s_map_zoom_loaded) {
        s_map_zoom_loaded = true;
        if (current_settings.map_zoom_m_per_px >= 0.05f && current_settings.map_zoom_m_per_px <= 30.0f) {
            s_map_m_per_px = current_settings.map_zoom_m_per_px;
            s_map_auto_fit = false;
        }
    }

    // Mode badge in Emerald Green
    lv_label_set_text(t0_mode_badge, "MAP VIEW");
    lv_obj_set_style_bg_color(t0_mode_badge, lv_color_hex(0x10B981), 0);

    ActiveGuidanceSegment_t seg;
    bool has_seg = waypoint_guidance_get_segment(d->tip_lat, d->tip_lon, &seg);

    // Top Target Row
    if (has_seg) {
        snprintf(buf, buf_sz, "%s -> %s [%s]",
                 seg.label_a, seg.label_b, seg.is_direct_here ? "SET A" : "SWAP");
        if (t0_target_lbl) rover_label_set_text_if_changed(t0_target_lbl, buf);
    } else if (d->has_active_target) {
        snprintf(buf, buf_sz, "%s (%d/%d)",
                 d->target_label, d->active_target_idx + 1, d->total_targets);
        if (t0_target_lbl) rover_label_set_text_if_changed(t0_target_lbl, buf);
    } else {
        if (t0_target_lbl) rover_label_set_text_if_changed(t0_target_lbl, "No Target (0/0)");
    }

    // Center & Auto-Fit Bounding Box calculation
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
        float sx = (float)dlon_m / 250.0f;
        float sy = (float)dlat_m / 184.0f;
        float fit_scale = (sx > sy) ? sx : sy;
        if (fit_scale < 0.15f) fit_scale = 0.15f;
        if (fit_scale > 25.0f) fit_scale = 25.0f;
        s_map_m_per_px = fit_scale;
        if (fabs(s_map_center_lat) > 0.0001 || fabs(s_map_center_lon) > 0.0001) {
            s_map_center_initialized = true;
        }
    } else if (s_map_center_rover && fabs(d->tip_lat) > 0.0001) {
        s_map_center_lat = d->tip_lat;
        s_map_center_lon = d->tip_lon;
        s_map_center_initialized = true;
    } else if (!s_map_center_initialized) {
        // Initialize map camera center onto parcel/target when sticky manual zoom starts at boot or file switch
        if (g_waypoint_count > 0) {
            double min_lat = g_waypoints[0].latitude, max_lat = g_waypoints[0].latitude;
            double min_lon = g_waypoints[0].longitude, max_lon = g_waypoints[0].longitude;
            for (int i = 1; i < g_waypoint_count; i++) {
                if (g_waypoints[i].latitude < min_lat) min_lat = g_waypoints[i].latitude;
                if (g_waypoints[i].latitude > max_lat) max_lat = g_waypoints[i].latitude;
                if (g_waypoints[i].longitude < min_lon) min_lon = g_waypoints[i].longitude;
                if (g_waypoints[i].longitude > max_lon) max_lon = g_waypoints[i].longitude;
            }
            s_map_center_lat = (min_lat + max_lat) * 0.5;
            s_map_center_lon = (min_lon + max_lon) * 0.5;
            s_map_center_initialized = true;
        } else if (fabs(d->tip_lat) > 0.0001 || fabs(d->tip_lon) > 0.0001) {
            s_map_center_lat = d->tip_lat;
            s_map_center_lon = d->tip_lon;
            s_map_center_initialized = true;
        }
    }

    // Consolidated Row 6 Marquee with up-to-date Map scale & waypoint count
    float win_w_m = (float)T0_CANVAS_W * s_map_m_per_px;
    if (has_seg) {
        snprintf(buf, buf_sz, "SEG: %s -> %s | Win: %.0fm (%.2fm/px) | Pts: %d",
                 seg.label_a, seg.label_b, win_w_m, s_map_m_per_px, g_waypoint_count);
    } else if (d->has_active_target) {
        snprintf(buf, buf_sz, "TGT: %s | Win: %.0fm (%.2fm/px) | Pts: %d",
                 d->target_label, win_w_m, s_map_m_per_px, g_waypoint_count);
    } else {
        snprintf(buf, buf_sz, "Touch Map to Pan | Win: %.0fm (%.2fm/px) | Pts: %d",
                 win_w_m, s_map_m_per_px, g_waypoint_count);
    }
    if (t0_notes_lbl) rover_label_set_text_if_changed(t0_notes_lbl, buf);

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
        s_bnd_pts[bnd_cnt].x = T0_CANVAS_CX + (int16_t)roundf((float)dE / s_map_m_per_px);
        s_bnd_pts[bnd_cnt].y = T0_CANVAS_CY - (int16_t)roundf((float)dN / s_map_m_per_px);
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

    // Highlight Active Followed Segment (A -> B) above base boundary polygon
    int16_t a_sx = 0, a_sy = 0, b_sx = 0, b_sy = 0;
    bool    show_pt_a = false;
    if (has_seg) {
        double a_dN = (seg.lat_a - s_map_center_lat) * 111132.95;
        double a_dE = (seg.lon_a - s_map_center_lon) * 111132.95 * cos_lat;
        a_sx = T0_CANVAS_CX + (int16_t)roundf((float)a_dE / s_map_m_per_px);
        a_sy = T0_CANVAS_CY - (int16_t)roundf((float)a_dN / s_map_m_per_px);

        double b_dN = (seg.lat_b - s_map_center_lat) * 111132.95;
        double b_dE = (seg.lon_b - s_map_center_lon) * 111132.95 * cos_lat;
        b_sx = T0_CANVAS_CX + (int16_t)roundf((float)b_dE / s_map_m_per_px);
        b_sy = T0_CANVAS_CY - (int16_t)roundf((float)b_dN / s_map_m_per_px);

        static lv_point_precise_t s_act_seg_pts[2];
        s_act_seg_pts[0].x = a_sx;
        s_act_seg_pts[0].y = a_sy;
        s_act_seg_pts[1].x = b_sx;
        s_act_seg_pts[1].y = b_sy;
        lv_line_set_points(t0_map_active_seg, s_act_seg_pts, 2);
        lv_obj_set_style_line_color(t0_map_active_seg,
                                    seg.is_direct_here ? lv_color_hex(0xC084FC) : lv_color_hex(0x00FF88), 0);
        lv_obj_set_style_line_width(t0_map_active_seg, seg.is_direct_here ? 3 : 4, 0);
        lv_obj_set_style_line_dash_width(t0_map_active_seg, seg.is_direct_here ? 6 : 0, 0);
        lv_obj_set_style_line_dash_gap(t0_map_active_seg, seg.is_direct_here ? 4 : 0, 0);
        lv_obj_set_hidden(t0_map_active_seg, false);

        if (a_sx >= -6 && a_sx <= (T0_CANVAS_W + 6) && a_sy >= -6 && a_sy <= (T0_CANVAS_H + 6)) {
            lv_obj_align(t0_line_pt_a, LV_ALIGN_TOP_LEFT, a_sx - 6, a_sy - 6);
            lv_obj_set_style_bg_color(t0_line_pt_a,
                                      seg.is_direct_here ? lv_color_hex(0xA855F7) : lv_color_hex(0xF59E0B), 0);
            lv_obj_set_hidden(t0_line_pt_a, false);
            show_pt_a = true;
        }
    } else {
        lv_obj_set_hidden(t0_map_active_seg, true);
    }
    if (!show_pt_a) {
        lv_obj_set_hidden(t0_line_pt_a, true);
    }

    // Render Waypoint Pins (Target B = Green 16px, Start A = Amber 12px, Others = Gold / Orange)
    bool target_pin_shown = false;
    for (int i = 0; i < g_waypoint_count && i < T0_MAP_MAX_PINS; i++) {
        double dN = (g_waypoints[i].latitude - s_map_center_lat) * 111132.95;
        double dE = (g_waypoints[i].longitude - s_map_center_lon) * 111132.95 * cos_lat;
        int16_t sx = T0_CANVAS_CX + (int16_t)roundf((float)dE / s_map_m_per_px);
        int16_t sy = T0_CANVAS_CY - (int16_t)roundf((float)dN / s_map_m_per_px);

        if (i == g_active_waypoint_idx) {
            lv_obj_set_hidden(t0_map_pins[i], true);
            b_sx = sx;
            b_sy = sy;
            if (sx >= -8 && sx <= (T0_CANVAS_W + 8) && sy >= -8 && sy <= (T0_CANVAS_H + 8)) {
                lv_obj_align(t0_target_icon, LV_ALIGN_TOP_LEFT, sx - 8, sy - 8);
                lv_obj_set_hidden(t0_target_icon, false);
                target_pin_shown = true;
            }
        } else if (has_seg && !seg.is_direct_here && i == seg.idx_a && show_pt_a) {
            lv_obj_set_hidden(t0_map_pins[i], true);
        } else {
            if (sx >= -5 && sx <= (T0_CANVAS_W + 5) && sy >= -5 && sy <= (T0_CANVAS_H + 5)) {
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
    if (!target_pin_shown) {
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
                int16_t sx1 = T0_CANVAS_CX + (int16_t)roundf((float)dE1 / s_map_m_per_px);
                int16_t sy1 = T0_CANVAS_CY - (int16_t)roundf((float)dN1 / s_map_m_per_px);

                double dN2 = (g_waypoints[tgt_idx].latitude - s_map_center_lat) * 111132.95;
                double dE2 = (g_waypoints[tgt_idx].longitude - s_map_center_lon) * 111132.95 * cos_lat;
                int16_t sx2 = T0_CANVAS_CX + (int16_t)roundf((float)dE2 / s_map_m_per_px);
                int16_t sy2 = T0_CANVAS_CY - (int16_t)roundf((float)dN2 / s_map_m_per_px);

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

    // Render Base Station Survey Control Symbol (excluded from auto-fit parcel zoom)
    int16_t bs_sx = 0, bs_sy = 0;
    bool    show_base = false;
    if (d->base_coords_valid && (fabs(d->base_lat) > 0.0001 || fabs(d->base_lon) > 0.0001)) {
        double bs_dN = (d->base_lat - s_map_center_lat) * 111132.95;
        double bs_dE = (d->base_lon - s_map_center_lon) * 111132.95 * cos_lat;
        bs_sx = T0_CANVAS_CX + (int16_t)roundf((float)bs_dE / s_map_m_per_px);
        bs_sy = T0_CANVAS_CY - (int16_t)roundf((float)bs_dN / s_map_m_per_px);
        if (bs_sx >= -8 && bs_sx <= (T0_CANVAS_W + 8) && bs_sy >= 18 && bs_sy <= 246) {
            static lv_point_precise_t s_base_tri_pts[4];
            s_base_tri_pts[0] = (lv_point_precise_t){bs_sx, (int16_t)(bs_sy - 8)};
            s_base_tri_pts[1] = (lv_point_precise_t){(int16_t)(bs_sx + 7), (int16_t)(bs_sy + 5)};
            s_base_tri_pts[2] = (lv_point_precise_t){(int16_t)(bs_sx - 7), (int16_t)(bs_sy + 5)};
            s_base_tri_pts[3] = s_base_tri_pts[0];
            if (t0_map_base_tri) {
                lv_line_set_points(t0_map_base_tri, s_base_tri_pts, 4);
                lv_obj_set_hidden(t0_map_base_tri, false);
            }
            if (t0_map_base_dot) {
                lv_obj_align(t0_map_base_dot, LV_ALIGN_TOP_LEFT, bs_sx - 2, bs_sy - 2);
                lv_obj_set_hidden(t0_map_base_dot, false);
            }
            show_base = true;
        }
    }
    if (!show_base) {
        if (t0_map_base_tri) lv_obj_set_hidden(t0_map_base_tri, true);
        if (t0_map_base_dot) lv_obj_set_hidden(t0_map_base_dot, true);
    }

    // Render Breadcrumb Trail Polyline (when enabled in settings)
    int bc_count = nav_breadcrumb_get_count();
    int16_t trail_tail_x[2] = {0, 0};
    int16_t trail_tail_y[2] = {0, 0};
    int     trail_tail_cnt  = 0;
    if (current_settings.map_show_route_trail && bc_count > 1) {
        static lv_point_precise_t s_trail_pts[NAV_BREADCRUMB_CAPACITY];
        int trail_cnt = 0;
        for (int i = 0; i < bc_count && i < NAV_BREADCRUMB_CAPACITY; i++) {
            NavBreadcrumb_t bc_pt;
            if (nav_breadcrumb_get_point(i, &bc_pt)) {
                double dN = (bc_pt.lat - s_map_center_lat) * 111132.95;
                double dE = (bc_pt.lon - s_map_center_lon) * 111132.95 * cos_lat;
                s_trail_pts[trail_cnt].x = T0_CANVAS_CX + (int16_t)roundf((float)dE / s_map_m_per_px);
                s_trail_pts[trail_cnt].y = T0_CANVAS_CY - (int16_t)roundf((float)dN / s_map_m_per_px);
                trail_cnt++;
            }
        }
        if (trail_cnt > 1) {
            lv_line_set_points(t0_map_trail, s_trail_pts, trail_cnt);
            lv_obj_set_hidden(t0_map_trail, false);
            trail_tail_x[0] = s_trail_pts[trail_cnt - 1].x;
            trail_tail_y[0] = s_trail_pts[trail_cnt - 1].y;
            trail_tail_x[1] = s_trail_pts[trail_cnt - 2].x;
            trail_tail_y[1] = s_trail_pts[trail_cnt - 2].y;
            trail_tail_cnt  = 2;
        } else {
            lv_obj_set_hidden(t0_map_trail, true);
        }
    } else {
        lv_obj_set_hidden(t0_map_trail, true);
    }

    // Render Live Rover Position & Barbed Heading Arrow
    int16_t r_sx = 0, r_sy = 0;
    bool    show_rover = false;
    if (fabs(d->tip_lat) > 0.0001 || fabs(d->tip_lon) > 0.0001) {
        double r_dN = (d->tip_lat - s_map_center_lat) * 111132.95;
        double r_dE = (d->tip_lon - s_map_center_lon) * 111132.95 * cos_lat;
        r_sx = T0_CANVAS_CX + (int16_t)roundf((float)r_dE / s_map_m_per_px);
        r_sy = T0_CANVAS_CY - (int16_t)roundf((float)r_dN / s_map_m_per_px);
        if (r_sx >= -7 && r_sx <= (T0_CANVAS_W + 7) && r_sy >= 16 && r_sy <= (T0_CANVAS_H + 7)) {
            lv_obj_align(t0_rover_icon, LV_ALIGN_TOP_LEFT, r_sx - 7, r_sy - 7);
            lv_obj_set_style_bg_color(t0_rover_icon, lv_color_hex(0x38BDF8), 0);
            lv_obj_set_hidden(t0_rover_icon, false);
            show_rover = true;
            if (d->heading_valid) {
                ui_tile0_draw_barbed_arrow(r_sx, r_sy, d->true_heading_deg, 22.0f, 0xFFFFFF);
            } else {
                lv_obj_set_hidden(t0_beam_arrow, true);
            }
        } else {
            lv_obj_set_hidden(t0_rover_icon, true);
            lv_obj_set_hidden(t0_beam_arrow, true);
        }
    } else {
        lv_obj_set_hidden(t0_rover_icon, true);
        lv_obj_set_hidden(t0_beam_arrow, true);
    }

    // Dynamic Non-Overlapping Labels (A, B, R, BS) on Map View
    static int8_t s_map_a_dir = 0, s_map_b_dir = 1, s_map_r_dir = 2, s_map_bs_dir = 3;
    int16_t a_lx = 0, a_ly = 0, b_lx = 0, b_ly = 0, r_lx = 0, r_ly = 0;
    bool has_a_lbl = false, has_b_lbl = false, has_r_lbl = false;

    if (t0_lbl_pt_a && show_pt_a && map_pt_in_view(a_sx, a_sy)) {
        float rx[8], ry[8];
        int rcnt = 0;
        if (has_seg) add_unit_ray(rx, ry, &rcnt, 8, b_sx - a_sx, b_sy - a_sy);
        if (!seg.is_direct_here) {
            add_pin_edge_rays(a_sx, a_sy, seg.idx_a, bnd_indices, s_bnd_pts, bnd_poly_cnt,
                              s_tie_pts, active_ties, rx, ry, &rcnt, 8);
        }
        int16_t av_x[4], av_y[4];
        int acnt = 0;
        if (target_pin_shown) { av_x[acnt] = b_sx;  av_y[acnt] = b_sy;  acnt++; }
        if (show_rover)       { av_x[acnt] = r_sx;  av_y[acnt] = r_sy;  acnt++; }
        if (show_base)        { av_x[acnt] = bs_sx; av_y[acnt] = bs_sy; acnt++; }
        ui_tile0_place_point_label(t0_lbl_pt_a, a_sx, a_sy, 15.5f, rx, ry, rcnt,
                                   av_x, av_y, acnt, false, &s_map_a_dir,
                                   seg.is_direct_here ? 0xC084FC : 0xF59E0B, &a_lx, &a_ly);
        has_a_lbl = true;
    } else if (t0_lbl_pt_a) {
        lv_obj_set_hidden(t0_lbl_pt_a, true);
    }

    if (t0_lbl_pt_b && target_pin_shown && map_pt_in_view(b_sx, b_sy)) {
        float rx[8], ry[8];
        int rcnt = 0;
        if (has_seg) add_unit_ray(rx, ry, &rcnt, 8, a_sx - b_sx, a_sy - b_sy);
        add_pin_edge_rays(b_sx, b_sy, g_active_waypoint_idx, bnd_indices, s_bnd_pts, bnd_poly_cnt,
                          s_tie_pts, active_ties, rx, ry, &rcnt, 8);
        int16_t av_x[4], av_y[4];
        int acnt = 0;
        if (show_pt_a)  { av_x[acnt] = a_sx;  av_y[acnt] = a_sy;  acnt++; }
        if (has_a_lbl)  { av_x[acnt] = a_lx;  av_y[acnt] = a_ly;  acnt++; }
        if (show_rover) { av_x[acnt] = r_sx;  av_y[acnt] = r_sy;  acnt++; }
        if (show_base)  { av_x[acnt] = bs_sx; av_y[acnt] = bs_sy; acnt++; }
        ui_tile0_place_point_label(t0_lbl_pt_b, b_sx, b_sy, 17.5f, rx, ry, rcnt,
                                   av_x, av_y, acnt, false, &s_map_b_dir, 0x00FF88, &b_lx, &b_ly);
        has_b_lbl = true;
    } else if (t0_lbl_pt_b) {
        lv_obj_set_hidden(t0_lbl_pt_b, true);
    }

    if (t0_lbl_pt_r && show_rover && map_pt_in_view(r_sx, r_sy)) {
        float rx[6], ry[6];
        int rcnt = 0;
        if (d->heading_valid) {
            float hr = d->true_heading_deg * (float)(M_PI / 180.0);
            rx[rcnt] = sinf(hr);
            ry[rcnt] = -cosf(hr);
            rcnt++;
        }
        if (trail_tail_cnt == 2) {
            int ti = (abs(trail_tail_x[0] - r_sx) <= 2 && abs(trail_tail_y[0] - r_sy) <= 2) ? 1 : 0;
            add_unit_ray(rx, ry, &rcnt, 6, trail_tail_x[ti] - r_sx, trail_tail_y[ti] - r_sy);
        }
        if (show_pt_a && hypotf((float)(a_sx - r_sx), (float)(a_sy - r_sy)) < 35.0f) {
            add_unit_ray(rx, ry, &rcnt, 6, a_sx - r_sx, a_sy - r_sy);
        }
        if (target_pin_shown && hypotf((float)(b_sx - r_sx), (float)(b_sy - r_sy)) < 35.0f) {
            add_unit_ray(rx, ry, &rcnt, 6, b_sx - r_sx, b_sy - r_sy);
        }
        int16_t av_x[5], av_y[5];
        int acnt = 0;
        if (show_pt_a)        { av_x[acnt] = a_sx;  av_y[acnt] = a_sy;  acnt++; }
        if (has_a_lbl)        { av_x[acnt] = a_lx;  av_y[acnt] = a_ly;  acnt++; }
        if (target_pin_shown) { av_x[acnt] = b_sx;  av_y[acnt] = b_sy;  acnt++; }
        if (has_b_lbl)        { av_x[acnt] = b_lx;  av_y[acnt] = b_ly;  acnt++; }
        if (show_base)        { av_x[acnt] = bs_sx; av_y[acnt] = bs_sy; acnt++; }
        ui_tile0_place_point_label(t0_lbl_pt_r, r_sx, r_sy, 16.5f, rx, ry, rcnt,
                                   av_x, av_y, acnt, false, &s_map_r_dir, 0x38BDF8, &r_lx, &r_ly);
        has_r_lbl = true;
    } else if (t0_lbl_pt_r) {
        lv_obj_set_hidden(t0_lbl_pt_r, true);
    }

    if (t0_map_base_lbl && show_base && map_pt_in_view(bs_sx, bs_sy)) {
        float rx[4], ry[4];
        int rcnt = 0;
        if (show_rover && hypotf((float)(r_sx - bs_sx), (float)(r_sy - bs_sy)) < 35.0f) {
            add_unit_ray(rx, ry, &rcnt, 4, r_sx - bs_sx, r_sy - bs_sy);
        }
        if (show_pt_a && hypotf((float)(a_sx - bs_sx), (float)(a_sy - bs_sy)) < 35.0f) {
            add_unit_ray(rx, ry, &rcnt, 4, a_sx - bs_sx, a_sy - bs_sy);
        }
        if (target_pin_shown && hypotf((float)(b_sx - bs_sx), (float)(b_sy - bs_sy)) < 35.0f) {
            add_unit_ray(rx, ry, &rcnt, 4, b_sx - bs_sx, b_sy - bs_sy);
        }
        int16_t av_x[6], av_y[6];
        int acnt = 0;
        if (show_pt_a)        { av_x[acnt] = a_sx; av_y[acnt] = a_sy; acnt++; }
        if (has_a_lbl)        { av_x[acnt] = a_lx; av_y[acnt] = a_ly; acnt++; }
        if (target_pin_shown) { av_x[acnt] = b_sx; av_y[acnt] = b_sy; acnt++; }
        if (has_b_lbl)        { av_x[acnt] = b_lx; av_y[acnt] = b_ly; acnt++; }
        if (show_rover)       { av_x[acnt] = r_sx; av_y[acnt] = r_sy; acnt++; }
        if (has_r_lbl)        { av_x[acnt] = r_lx; av_y[acnt] = r_ly; acnt++; }
        ui_tile0_place_point_label(t0_map_base_lbl, bs_sx, bs_sy, 18.0f, rx, ry, rcnt,
                                   av_x, av_y, acnt, false, &s_map_bs_dir, 0xFF44AA, NULL, NULL);
    } else if (t0_map_base_lbl) {
        lv_obj_set_hidden(t0_map_base_lbl, true);
    }

    // Row 1: Distance & Bearing to Target B (t0_dist_lbl in Montserrat 16)
    lv_obj_set_style_text_font(t0_dist_lbl, &lv_font_montserrat_16, 0);
    if (d->has_active_target) {
        char dist_str[24];
        nav_format_distance(d->distance_to_target_m, dist_str, sizeof(dist_str));
        snprintf(buf, buf_sz, "%s @ %05.1f deg", dist_str, d->bearing_to_target_deg);
    } else {
        snprintf(buf, buf_sz, "--- m @ --- deg");
    }
    if (t0_dist_lbl) {
        rover_label_set_text_if_changed(t0_dist_lbl, buf);
        lv_obj_set_style_text_color(t0_dist_lbl, lv_color_hex(0x10B981), 0);
    }

    // Row 2: Map Center Coordinates (t0_offsets_lbl in Montserrat 12)
    snprintf(buf, buf_sz, "Map Ctr: %.5f, %.5f", s_map_center_lat, s_map_center_lon);
    if (t0_offsets_lbl) {
        rover_label_set_text_if_changed(t0_offsets_lbl, buf);
        lv_obj_set_style_text_color(t0_offsets_lbl, lv_color_hex(0x38BDF8), 0);
    }
}
