#include "lvgl_ui_internal.h"

// =========================================================================
// Tile 1: Field Survey / Waypoint Logger Widgets
// =========================================================================
static lv_obj_t *t1_next_lbl        = NULL;
static lv_obj_t *t1_prefix_dd       = NULL;
static lv_obj_t *t1_note_dd         = NULL;
static lv_obj_t *t1_avg_btn_lbl     = NULL;
static lv_obj_t *t1_log_btn         = NULL;
static lv_obj_t *t1_log_btn_lbl     = NULL;
static lv_obj_t *t1_prog_bar        = NULL;
static lv_obj_t *t1_status_lbl      = NULL;
static lv_obj_t *t1_qual_card       = NULL;
static lv_obj_t *t1_qual_fix_lbl    = NULL;
static lv_obj_t *t1_qual_acc_lbl    = NULL;
static lv_obj_t *t1_qual_tilt_lbl   = NULL;
static lv_obj_t *t1_qual_tgt_lbl    = NULL;

static void update_t1_log_btn_idle_style(void)
{
    if (!t1_log_btn || !t1_log_btn_lbl) return;
    const char *tag = waypoint_get_note_tag_label(g_survey_note_tag_idx);
    char lbuf[64];
    lv_color_t target_col = lv_palette_main(LV_PALETTE_GREEN);
    if (tag && tag[0] != '\0') {
        snprintf(lbuf, sizeof(lbuf), "LOG: %s", tag);
        if (strncmp(tag, "NO PIN", 6) == 0) {
            target_col = lv_color_hex(0x0284C7);
        }
    } else {
        snprintf(lbuf, sizeof(lbuf), "LOG PIN NOW");
    }
    const char *cur = lv_label_get_text(t1_log_btn_lbl);
    if (!cur || strcmp(cur, lbuf) != 0) {
        lv_label_set_text(t1_log_btn_lbl, lbuf);
    }
    lv_obj_set_style_bg_color(t1_log_btn, target_col, 0);
}

static void prefix_dd_cb(lv_event_t *e)
{
    lv_obj_t *dd = (lv_obj_t *)lv_event_get_target(e);
    g_survey_prefix_idx = (uint8_t)lv_dropdown_get_selected(dd);
    waypoint_invalidate_survey_cache();
    play_rover_beep(1400, 20);
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Prefix dropdown -> idx %u\n", g_survey_prefix_idx);
    }
}

static void note_dd_cb(lv_event_t *e)
{
    lv_obj_t *dd = (lv_obj_t *)lv_event_get_target(e);
    g_survey_note_tag_idx = (uint8_t)lv_dropdown_get_selected(dd);
    update_t1_log_btn_idle_style();
    play_rover_beep(1400, 20);
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Note tag dropdown -> idx %u\n", g_survey_note_tag_idx);
    }
}

static void avg_dur_btn_cb(lv_event_t *e)
{
    (void)e;
    if (current_settings.survey_avg_seconds <= 1)       current_settings.survey_avg_seconds = 5;
    else if (current_settings.survey_avg_seconds <= 5)  current_settings.survey_avg_seconds = 10;
    else if (current_settings.survey_avg_seconds <= 10) current_settings.survey_avg_seconds = 30;
    else                                                current_settings.survey_avg_seconds = 1;
    mark_settings_dirty();
    if (t1_avg_btn_lbl) {
        lv_label_set_text_fmt(t1_avg_btn_lbl, "Avg: %us", current_settings.survey_avg_seconds);
    }
    play_rover_beep(1400, 25);
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Avg Duration clicked -> %ds\n", current_settings.survey_avg_seconds);
    }
}

static void log_pin_btn_cb(lv_event_t *e)
{
    (void)e;
    trigger_survey_waypoint_log();
    play_rover_beep(1750, 40);
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Log Pin clicked!\n");
    }
}

void ui_tile1_survey_create(lv_obj_t *parent)
{
    // =====================================================================
    // TILE 1: Dedicated Field Survey / Waypoint Logger (320 x 374 px)
    // =====================================================================
    // Row 1 (y=8): Location ID Prefix Dropdown (146x40) & Averaging Duration Button (146x40)
    t1_prefix_dd = lv_dropdown_create(parent);
    lv_dropdown_set_options(t1_prefix_dd, waypoint_get_prefix_options());
    lv_dropdown_set_selected(t1_prefix_dd, g_survey_prefix_idx);
    lv_obj_set_size(t1_prefix_dd, 146, 40);
    lv_obj_align(t1_prefix_dd, LV_ALIGN_TOP_LEFT, 10, 8);
    lv_obj_add_event_cb(t1_prefix_dd, prefix_dd_cb, LV_EVENT_VALUE_CHANGED, NULL);

    lv_obj_t *btn_avg = lv_button_create(parent);
    lv_obj_set_size(btn_avg, 146, 40);
    lv_obj_align(btn_avg, LV_ALIGN_TOP_RIGHT, -10, 8);
    lv_obj_add_event_cb(btn_avg, avg_dur_btn_cb, LV_EVENT_CLICKED, NULL);
    t1_avg_btn_lbl = lv_label_create(btn_avg);
    lv_obj_set_style_text_font(t1_avg_btn_lbl, &lv_font_montserrat_14, 0);
    lv_label_set_text_fmt(t1_avg_btn_lbl, "Avg: %us", current_settings.survey_avg_seconds);
    lv_obj_center(t1_avg_btn_lbl);

    // Row 2 (y=56): Full-Width Monument / Note Tag Dropdown (300x40)
    t1_note_dd = lv_dropdown_create(parent);
    lv_dropdown_set_options(t1_note_dd, waypoint_get_note_tag_options());
    lv_dropdown_set_selected(t1_note_dd, g_survey_note_tag_idx);
    lv_obj_set_size(t1_note_dd, 300, 40);
    lv_obj_align(t1_note_dd, LV_ALIGN_TOP_MID, 0, 56);
    lv_obj_add_event_cb(t1_note_dd, note_dd_cb, LV_EVENT_VALUE_CHANGED, NULL);

    // Row 3 (y=104): ID Label (300px wide, Montserrat 14, scrolling marquee)
    t1_next_lbl = lv_label_create(parent);
    lv_obj_set_width(t1_next_lbl, 300);
    lv_label_set_long_mode(t1_next_lbl, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_label_set_text(t1_next_lbl, "ID: Acquiring...");
    lv_obj_set_style_text_color(t1_next_lbl, lv_color_hex(0xFFCC00), 0);
    lv_obj_set_style_text_font(t1_next_lbl, &lv_font_montserrat_14, 0);
    lv_obj_align(t1_next_lbl, LV_ALIGN_TOP_LEFT, 10, 104);

    // Row 4 (y=128): Multi-epoch progress bar (300x8)
    t1_prog_bar = lv_bar_create(parent);
    lv_obj_set_size(t1_prog_bar, 300, 8);
    lv_obj_align(t1_prog_bar, LV_ALIGN_TOP_MID, 0, 128);
    lv_bar_set_range(t1_prog_bar, 0, 100);
    lv_bar_set_value(t1_prog_bar, 0, LV_ANIM_OFF);

    // Row 5 (y=144): Large 52 px LOG PIN NOW Button (visibly echoes active Note Tag)
    t1_log_btn = lv_button_create(parent);
    lv_obj_set_size(t1_log_btn, 300, 52);
    lv_obj_align(t1_log_btn, LV_ALIGN_TOP_MID, 0, 144);
    lv_obj_set_style_radius(t1_log_btn, 8, 0);
    lv_obj_set_style_bg_color(t1_log_btn, lv_palette_main(LV_PALETTE_GREEN), 0);
    lv_obj_add_event_cb(t1_log_btn, log_pin_btn_cb, LV_EVENT_CLICKED, NULL);
    t1_log_btn_lbl = lv_label_create(t1_log_btn);
    lv_obj_set_width(t1_log_btn_lbl, 288);
    lv_label_set_long_mode(t1_log_btn_lbl, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_obj_set_style_text_font(t1_log_btn_lbl, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_align(t1_log_btn_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(t1_log_btn_lbl);
    update_t1_log_btn_idle_style();

    // Row 6 (y=204): Status & Logged File Info
    t1_status_lbl = lv_label_create(parent);
    lv_obj_set_width(t1_status_lbl, 300);
    lv_label_set_text(t1_status_lbl, "Logged: None | /survey_logged_pins.csv");
    lv_obj_set_style_text_color(t1_status_lbl, lv_color_hex(0x99A3B0), 0);
    lv_obj_set_style_text_font(t1_status_lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_align(t1_status_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(t1_status_lbl, LV_ALIGN_TOP_MID, 0, 204);

    // Row 7 (y=228): Pre-Log Survey Quality Summary Card (300x136 px)
    t1_qual_card = lv_obj_create(parent);
    lv_obj_set_size(t1_qual_card, 300, 136);
    lv_obj_align(t1_qual_card, LV_ALIGN_TOP_MID, 0, 228);
    lv_obj_set_style_bg_color(t1_qual_card, lv_color_hex(0x1A202C), 0);
    lv_obj_set_style_border_color(t1_qual_card, lv_color_hex(0x2D3748), 0);
    lv_obj_set_style_border_width(t1_qual_card, 1, 0);
    lv_obj_set_style_radius(t1_qual_card, 6, 0);
    lv_obj_set_style_pad_all(t1_qual_card, 8, 0);
    lv_obj_set_scrollbar_mode(t1_qual_card, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_scrollable(t1_qual_card, false);

    lv_obj_t *qual_hdr = lv_label_create(t1_qual_card);
    lv_label_set_text(qual_hdr, "PRE-LOG SURVEY QUALITY");
    lv_obj_set_style_text_color(qual_hdr, lv_color_hex(0x38BDF8), 0);
    lv_obj_set_style_text_font(qual_hdr, &lv_font_montserrat_12, 0);
    lv_obj_align(qual_hdr, LV_ALIGN_TOP_LEFT, 0, 0);

    t1_qual_fix_lbl = lv_label_create(t1_qual_card);
    lv_obj_set_width(t1_qual_fix_lbl, 282);
    lv_label_set_text(t1_qual_fix_lbl, "Fix: -- | Sats: 0/0 | HDOP: --.-");
    lv_obj_set_style_text_color(t1_qual_fix_lbl, lv_color_hex(0xE2E8F0), 0);
    lv_obj_set_style_text_font(t1_qual_fix_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t1_qual_fix_lbl, LV_ALIGN_TOP_LEFT, 0, 22);

    t1_qual_acc_lbl = lv_label_create(t1_qual_card);
    lv_obj_set_width(t1_qual_acc_lbl, 282);
    lv_label_set_text(t1_qual_acc_lbl, "H-Acc: -- | V-Acc: --");
    lv_obj_set_style_text_color(t1_qual_acc_lbl, lv_color_hex(0x00FF88), 0);
    lv_obj_set_style_text_font(t1_qual_acc_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t1_qual_acc_lbl, LV_ALIGN_TOP_LEFT, 0, 46);

    t1_qual_tilt_lbl = lv_label_create(t1_qual_card);
    lv_obj_set_width(t1_qual_tilt_lbl, 282);
    lv_label_set_text(t1_qual_tilt_lbl, "Pole Tilt: 0.0 deg (dX:+0.0 dY:+0.0)");
    lv_obj_set_style_text_color(t1_qual_tilt_lbl, lv_color_hex(0xFFCC00), 0);
    lv_obj_set_style_text_font(t1_qual_tilt_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t1_qual_tilt_lbl, LV_ALIGN_TOP_LEFT, 0, 70);

    t1_qual_tgt_lbl = lv_label_create(t1_qual_card);
    lv_obj_set_width(t1_qual_tgt_lbl, 282);
    lv_label_set_long_mode(t1_qual_tgt_lbl, LV_LABEL_LONG_SCROLL_CIRCULAR);
    lv_label_set_text(t1_qual_tgt_lbl, "Target: None active");
    lv_obj_set_style_text_color(t1_qual_tgt_lbl, lv_color_hex(0x94A3B8), 0);
    lv_obj_set_style_text_font(t1_qual_tgt_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t1_qual_tgt_lbl, LV_ALIGN_TOP_LEFT, 0, 94);
}

void ui_tile1_survey_update(const RoverUIData_t *d, char *buf, size_t buf_sz)
{
    // 3. Update Tile 1: Field Survey Logger (only query survey stem when Tile 1 is visible or survey is active)
    if (active_tile_col == 1 || d->survey_in_progress) {
        char next_id[WAYPOINT_LABEL_LEN];
        waypoint_format_next_survey_label(next_id, sizeof(next_id));
        snprintf(buf, buf_sz, "ID: %s", next_id);
        const char *cur_next = lv_label_get_text(t1_next_lbl);
        if (!cur_next || strcmp(cur_next, buf) != 0) {
            lv_label_set_text(t1_next_lbl, buf);
        }

        lv_bar_set_value(t1_prog_bar, d->survey_progress_pct, LV_ANIM_OFF);
        if (d->survey_in_progress) {
            snprintf(buf, buf_sz, "AVERAGING RTK... %u%% (%u pts)",
                     d->survey_progress_pct, d->survey_epochs_collected);
            lv_label_set_text(t1_log_btn_lbl, buf);
            lv_obj_set_style_bg_color(t1_log_btn, lv_palette_main(LV_PALETTE_ORANGE), 0);
        } else {
            update_t1_log_btn_idle_style();
        }
        snprintf(buf, buf_sz, "Logged: %s  |  /survey_logged_pins.csv",
                 d->last_logged_label);
        const char *cur_stat = lv_label_get_text(t1_status_lbl);
        if (!cur_stat || strcmp(cur_stat, buf) != 0) {
            lv_label_set_text(t1_status_lbl, buf);
        }
    }

    if (active_tile_col == 1) {
        if (t1_qual_fix_lbl) {
            const char *fix_str = "NO FIX";
            if (!d->um982_stats.com1_online)            fix_str = "OFFLINE";
            else if (d->fix_type == GNSS_FIX_RTK_FIXED) fix_str = "RTK FIX";
            else if (d->fix_type == GNSS_FIX_RTK_FLOAT) fix_str = "RTK FLT";
            else if (d->fix_type == GNSS_FIX_DGPS)      fix_str = "DGPS";
            else if (d->fix_type == GNSS_FIX_SINGLE)    fix_str = "SINGLE";
            uint8_t sv_view = (d->sats_in_view >= d->num_sats) ? d->sats_in_view : d->num_sats;
            snprintf(buf, buf_sz, "Fix: %s | SV:%u/%u | HDOP:%.1f",
                     fix_str, d->num_sats, sv_view, d->hdop);
            lv_label_set_text(t1_qual_fix_lbl, buf);
        }
        if (t1_qual_acc_lbl) {
            char h_acc_str[24], v_acc_str[24];
            nav_format_distance(d->h_acc_m, h_acc_str, sizeof(h_acc_str));
            nav_format_distance(d->v_acc_m, v_acc_str, sizeof(v_acc_str));
            snprintf(buf, buf_sz, "H-Acc: +/-%s | V-Acc: +/-%s", h_acc_str, v_acc_str);
            lv_label_set_text(t1_qual_acc_lbl, buf);
            lv_obj_set_style_text_color(t1_qual_acc_lbl,
                                        (d->fix_type == GNSS_FIX_RTK_FIXED) ? lv_color_hex(0x00FF88) : lv_color_hex(0xFFCC00), 0);
        }
        if (t1_qual_tilt_lbl) {
            snprintf(buf, buf_sz, "Pole Tilt: %.1f deg (dX:%+.1f dY:%+.1f)",
                     d->pole_tilt_deg, d->pole_bubble_x_deg, d->pole_bubble_y_deg);
            lv_label_set_text(t1_qual_tilt_lbl, buf);
        }
        if (t1_qual_tgt_lbl) {
            if (d->has_active_target) {
                char dn_str[24], de_str[24];
                nav_format_offset(-d->d_north_m, dn_str, sizeof(dn_str));
                nav_format_offset(-d->d_east_m, de_str, sizeof(de_str));
                snprintf(buf, buf_sz, "From %s: dN %s  dE %s",
                         d->target_label, dn_str, de_str);
            } else {
                snprintf(buf, buf_sz, "Target: None active");
            }
            lv_label_set_text(t1_qual_tgt_lbl, buf);
        }
    }
}
