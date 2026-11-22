#include "lvgl_ui_internal.h"

// =========================================================================
// Tile 3: Dual-Antenna Compass & Pole Bubble Level Widgets
// =========================================================================
static lv_obj_t *t3_hdg_lbl         = NULL;
static lv_obj_t *t3_base_lbl        = NULL;
static lv_obj_t *t3_pose_lbl        = NULL;
static lv_obj_t *t3_level_circle    = NULL;
static lv_obj_t *t3_bubble_dot      = NULL;
static lv_obj_t *t3_card_lbls[8]    = {NULL};
static lv_obj_t *t3_tilt_lbl        = NULL;
lv_obj_t        *t3_tare_btn        = NULL;
lv_obj_t        *t3_tare_lbl        = NULL;

static const char *kCardNames[8] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
static const float kCardAngles[8] = {0.0f, 45.0f, 90.0f, 135.0f, 180.0f, 225.0f, 270.0f, 315.0f};

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
        play_rover_beep(1800, 40);
    }
    update_all_control_buttons();
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Tare clicked -> %s\n", has_tare ? "RESET" : "TARE");
    }
}

void ui_tile3_compass_create(lv_obj_t *parent)
{
    // =====================================================================
    // TILE 3: Dual-Antenna Compass & Pole Bubble Level
    // =====================================================================
    t3_hdg_lbl = lv_label_create(parent);
    lv_label_set_text(t3_hdg_lbl, "Heading: ---.-");
    lv_obj_set_style_text_color(t3_hdg_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(t3_hdg_lbl, &lv_font_montserrat_24, 0);
    lv_obj_align(t3_hdg_lbl, LV_ALIGN_TOP_MID, 0, 8);

    t3_base_lbl = lv_label_create(parent);
    lv_label_set_text(t3_base_lbl, "Baseline: 0.555 m | Pitch: 0.0 deg");
    lv_obj_set_style_text_color(t3_base_lbl, lv_color_hex(0x00FFAA), 0);
    lv_obj_set_style_text_font(t3_base_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t3_base_lbl, LV_ALIGN_TOP_MID, 0, 38);

    t3_pose_lbl = lv_label_create(parent);
    lv_label_set_text(t3_pose_lbl, "Display Pose: Held Flat");
    lv_obj_set_style_text_color(t3_pose_lbl, lv_color_hex(0x00D8FF), 0);
    lv_obj_set_style_text_font(t3_pose_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t3_pose_lbl, LV_ALIGN_TOP_MID, 0, 56);

    // Fixed Course-Up Index (12 o'clock heading indicator)
    lv_obj_t *t3_hdg_pointer = lv_label_create(parent);
    lv_label_set_text(t3_hdg_pointer, LV_SYMBOL_UP " HDG");
    lv_obj_set_style_text_color(t3_hdg_pointer, lv_color_hex(0x00FF88), 0);
    lv_obj_set_style_text_font(t3_hdg_pointer, &lv_font_montserrat_12, 0);
    lv_obj_align(t3_hdg_pointer, LV_ALIGN_TOP_MID, 0, 76);

    // 2D Pole Plumb Bubble Level Circle with Integrated Compass Ring (224x224 px, center at 160, 206)
    t3_level_circle = lv_obj_create(parent);
    lv_obj_set_size(t3_level_circle, 224, 224);
    lv_obj_align(t3_level_circle, LV_ALIGN_TOP_MID, 0, 94);
    lv_obj_set_style_radius(t3_level_circle, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(t3_level_circle, lv_color_hex(0x1A2421), 0);
    lv_obj_set_style_border_color(t3_level_circle, lv_color_hex(0x3B6E5C), 0);
    lv_obj_set_style_border_width(t3_level_circle, 2, 0);
    lv_obj_set_scrollbar_mode(t3_level_circle, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scrollable(t3_level_circle, false);

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

    // 8-Point Compass Ring Markers (N, NE, E, SE, S, SW, W, NW)
    for (int i = 0; i < 8; i++) {
        t3_card_lbls[i] = lv_label_create(t3_level_circle);
        lv_label_set_text(t3_card_lbls[i], kCardNames[i]);
        if (i == 0) {
            lv_obj_set_style_text_color(t3_card_lbls[i], lv_color_hex(0xEF4444), 0); // Bold Red North
            lv_obj_set_style_text_font(t3_card_lbls[i], &lv_font_montserrat_14, 0);
        } else if (i % 2 == 0) {
            lv_obj_set_style_text_color(t3_card_lbls[i], lv_color_hex(0xFFFFFF), 0); // White Cardinal
            lv_obj_set_style_text_font(t3_card_lbls[i], &lv_font_montserrat_12, 0);
        } else {
            lv_obj_set_style_text_color(t3_card_lbls[i], lv_color_hex(0x94A3B8), 0); // Slate Intercardinal
            lv_obj_set_style_text_font(t3_card_lbls[i], &lv_font_montserrat_12, 0);
        }
        float rad = kCardAngles[i] * 0.0174532925f;
        int16_t cx = (int16_t)roundf(sinf(rad) * 96.0f);
        int16_t cy = (int16_t)roundf(-cosf(rad) * 96.0f);
        lv_obj_align(t3_card_lbls[i], LV_ALIGN_CENTER, cx, cy);
    }

    // Compact Tare Button shifted to the right edge of the screen
    t3_tare_btn = lv_button_create(parent);
    lv_obj_set_size(t3_tare_btn, 64, 28);
    lv_obj_align(t3_tare_btn, LV_ALIGN_TOP_RIGHT, -10, 290);
    lv_obj_set_style_bg_color(t3_tare_btn, lv_color_hex(0x2563EB), 0); // Default Blue
    lv_obj_add_event_cb(t3_tare_btn, tare_level_btn_cb, LV_EVENT_CLICKED, NULL);
    t3_tare_lbl = lv_label_create(t3_tare_btn);
    lv_label_set_text(t3_tare_lbl, "TARE");
    lv_obj_set_style_text_font(t3_tare_lbl, &lv_font_montserrat_12, 0);
    lv_obj_center(t3_tare_lbl);

    t3_tilt_lbl = lv_label_create(parent);
    lv_label_set_text(t3_tilt_lbl, "Pole Tilt: dX: 0.0 deg | dY: 0.0 deg");
    lv_obj_set_style_text_color(t3_tilt_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(t3_tilt_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t3_tilt_lbl, LV_ALIGN_TOP_MID, 0, 336);
}

void ui_tile3_compass_update(const RoverUIData_t *d, char *buf, size_t buf_sz)
{
    // 4. Update Tile 3: Dual-Antenna / 9-DOF Compass & Bubble Level
    if (d->heading_valid) {
        const char *src_tag = (d->heading_source == HDG_SRC_DUAL_FIX) ? "FIX" :
                              (d->heading_source == HDG_SRC_DUAL_FLT) ? "FLT" :
                              (d->heading_source == HDG_SRC_DISP)     ? "WALK" :
                              (d->heading_source == HDG_SRC_COMP)     ? "CMP" :
                              (d->heading_source == HDG_SRC_GYRO)     ? "GYRO" :
                              (d->heading_source == HDG_SRC_COG)      ? "COG" :
                              (d->heading_source == HDG_SRC_HOLD)     ? "HLD" :
                              (d->heading_is_fixed ? "FIX" : "FLT");
        snprintf(buf, buf_sz, "Heading: %05.1f [%s]", d->true_heading_deg, src_tag);
        lv_label_set_text(t3_hdg_lbl, buf);
    } else {
        lv_label_set_text(t3_hdg_lbl, "Heading: WALK TO ALIGN");
    }
    if (d->mag_present) {
        if (d->mag_anomaly) {
            snprintf(buf, buf_sz, "BL: %.2fm | Mag: %.1fuT [IRON ANOMALY]",
                     d->baseline_length_m > 0.01f ? d->baseline_length_m : current_settings.antenna_spacing_m,
                     d->mag_total_uT);
        } else {
            snprintf(buf, buf_sz, "BL: %.2fm | Mag: %.1fuT (Tare:%+.1f%s)",
                     d->baseline_length_m > 0.01f ? d->baseline_length_m : current_settings.antenna_spacing_m,
                     d->mag_total_uT,
                     d->mag_tare_offset_deg,
                     d->mag_cal_valid ? "*" : "");
        }
    } else {
        snprintf(buf, buf_sz, "Baseline: %.3f m | Pitch: %+.1f deg",
                 d->baseline_length_m > 0.01f ? d->baseline_length_m : current_settings.antenna_spacing_m,
                 d->baseline_pitch_deg);
    }
    lv_label_set_text(t3_base_lbl, buf);

    const char *pose_str = "Held Flat";
    if (d->display_pose == DISPLAY_POSE_ANGLED) pose_str = "Handheld Viewing Angle";
    else if (d->display_pose == DISPLAY_POSE_VERTICAL) pose_str = "Clipped Vertically to Pole";
    snprintf(buf, buf_sz, "Display Pose: %s", pose_str);
    lv_label_set_text(t3_pose_lbl, buf);

    // Dynamic rotation of 8 compass points (Course-Up: current heading is UP at 12 o'clock)
    float cur_hdg = d->heading_valid ? d->true_heading_deg : 0.0f;
    for (int i = 0; i < 8; i++) {
        if (t3_card_lbls[i]) {
            float rel_deg = kCardAngles[i] - cur_hdg;
            float rad = rel_deg * 0.0174532925f;
            int16_t cx = (int16_t)roundf(sinf(rad) * 96.0f);
            int16_t cy = (int16_t)roundf(-cosf(rad) * 96.0f);
            lv_obj_align(t3_card_lbls[i], LV_ALIGN_CENTER, cx, cy);
        }
    }

    int16_t bx = (int16_t)(d->pole_bubble_x_deg * 6.0f);
    int16_t by = (int16_t)(d->pole_bubble_y_deg * 6.0f);
    if (bx > 70) bx = 70; if (bx < -70) bx = -70;
    if (by > 70) by = 70; if (by < -70) by = -70;
    lv_obj_align(t3_bubble_dot, LV_ALIGN_CENTER, bx, by);
    snprintf(buf, buf_sz, "Pole Tilt: dX: %+.1f deg | dY: %+.1f deg",
             d->pole_bubble_x_deg, d->pole_bubble_y_deg);
    lv_label_set_text(t3_tilt_lbl, buf);
}
