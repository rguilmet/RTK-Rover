#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl_ui_internal.h"
#include "../storage/route_logger.h"
#include "../storage/nmea_logger.h"
#include "../storage/rtcm_logger.h"
#include "../e220_driver/e220_driver.h"
#include "../ble_bridge/ble_bridge.h"
#include "../rtcm_arbiter/rtcm_arbiter.h"

// =========================================================================
// Tile 5: Dedicated Quick Settings Deck (5x3 Grid - 15 Slots, 94x44 px)
// =========================================================================
static lv_obj_t *t1_vol_btn         = NULL;
static lv_obj_t *t1_vol_btn_lbl     = NULL;
static lv_obj_t *t1_aud_btn         = NULL;
static lv_obj_t *t1_aud_btn_lbl     = NULL;
static lv_obj_t *t1_bright_btn      = NULL;
static lv_obj_t *t1_bright_btn_lbl  = NULL;

static lv_obj_t *t1_nmea_btn        = NULL;
static lv_obj_t *t1_nmea_btn_lbl    = NULL;
static lv_obj_t *t1_serial_btn      = NULL;
static lv_obj_t *t1_serial_btn_lbl  = NULL;
static lv_obj_t *t1_rtcm_btn        = NULL;
static lv_obj_t *t1_rtcm_btn_lbl    = NULL;

static lv_obj_t *t1_telem_btn       = NULL;
static lv_obj_t *t1_telem_btn_lbl   = NULL;
static lv_obj_t *t1_imu_btn         = NULL;
static lv_obj_t *t1_imu_btn_lbl     = NULL;
static lv_obj_t *t1_touch_btn       = NULL;
static lv_obj_t *t1_touch_btn_lbl   = NULL;

static lv_obj_t *t5_calmag_btn       = NULL;
static lv_obj_t *t5_calmag_btn_lbl   = NULL;
static lv_obj_t *t1_unit_btn         = NULL;
static lv_obj_t *t1_unit_btn_lbl     = NULL;
static lv_obj_t *t5_autooff_btn      = NULL;
static lv_obj_t *t5_autooff_btn_lbl  = NULL;

static lv_obj_t *t1_lora_btn         = NULL;
static lv_obj_t *t1_lora_btn_lbl     = NULL;
static lv_obj_t *t5_comms_btn        = NULL;
static lv_obj_t *t5_comms_btn_lbl    = NULL;
static lv_obj_t *t5_rtcm_src_btn     = NULL;
static lv_obj_t *t5_rtcm_src_btn_lbl = NULL;

static lv_obj_t *t1_wifi_status_lbl  = NULL;
static lv_obj_t *t5_power_status_lbl = NULL;
static lv_obj_t *t5_lora_status_lbl  = NULL;

static portMUX_TYPE s_wifi_lbl_mux = portMUX_INITIALIZER_UNLOCKED;
static char s_pending_wifi_status[64] = "Wi-Fi Inactive";
static volatile bool s_wifi_status_dirty = true;

void update_all_control_buttons(void)
{
    static int s_last_vol = -1;
    static int s_last_muted = -1;
    static int s_last_aud_mode = -1;
    static int s_last_bright = -1;
    static int s_last_nmea = -1;
    static int s_last_serial = -1;
    static int s_last_route = -1;
    static int s_last_rtcm = -1;
    static int s_last_telem = -1;
    static int s_last_imu = -1;
    static int s_last_touch = -1;
    static int s_last_calmag = -1;
    static int s_last_units = -1;
    static int s_last_autooff = -1;
    static int s_last_comms = -1;
    static int s_last_rtcm_src_key = -1;
    static int s_last_tare = -1;

    // Row 1, Col 0: VOL: 80% / VOL: MUTE
    int cur_vol = (int)current_settings.volume;
    int cur_muted = current_settings.is_muted ? 1 : 0;
    if (cur_vol != s_last_vol || cur_muted != s_last_muted) {
        s_last_vol = cur_vol;
        s_last_muted = cur_muted;
        if (t1_vol_btn && t1_vol_btn_lbl) {
            char v_buf[32];
            if (current_settings.volume == 0 || current_settings.is_muted) {
                snprintf(v_buf, sizeof(v_buf), "VOL: MUTE");
                lv_obj_set_style_bg_color(t1_vol_btn, lv_color_hex(0x555555), 0);
            } else {
                snprintf(v_buf, sizeof(v_buf), "VOL: %u%%", current_settings.volume);
                lv_obj_set_style_bg_color(t1_vol_btn, lv_color_hex(0x2F3B4C), 0);
            }
            lv_label_set_text(t1_vol_btn_lbl, v_buf);
        }
    }

    // Row 1, Col 1: AUD: GEIGER / AUD: BEEPS / AUD: MUTE
    int cur_aud = (int)current_settings.audio_nav_mode;
    if (cur_aud != s_last_aud_mode) {
        s_last_aud_mode = cur_aud;
        if (t1_aud_btn && t1_aud_btn_lbl) {
            if (cur_aud == AUDIO_NAV_GEIGER) {
                lv_label_set_text(t1_aud_btn_lbl, "AUD: GEIGER");
                lv_obj_set_style_bg_color(t1_aud_btn, COLOR_BTN_ACTIVE_GREEN, 0);
            } else if (cur_aud == AUDIO_NAV_MUTE) {
                lv_label_set_text(t1_aud_btn_lbl, "AUD: MUTE");
                lv_obj_set_style_bg_color(t1_aud_btn, lv_color_hex(0x555555), 0);
            } else {
                lv_label_set_text(t1_aud_btn_lbl, "AUD: BEEPS");
                lv_obj_set_style_bg_color(t1_aud_btn, lv_color_hex(0x2563EB), 0);
            }
        }
    }

    // Row 1, Col 2: BRI: 100%
    int cur_bright = (int)current_settings.brightness;
    if (cur_bright != s_last_bright) {
        s_last_bright = cur_bright;
        if (t1_bright_btn && t1_bright_btn_lbl) {
            char b_buf[32];
            snprintf(b_buf, sizeof(b_buf), "BRI: %u%%", current_settings.brightness);
            lv_label_set_text(t1_bright_btn_lbl, b_buf);
            lv_obj_set_style_bg_color(t1_bright_btn, lv_color_hex(0x2F3B4C), 0);
        }
    }

    // Row 2, Col 0: SERIAL: ON / SERIAL: OFF
    int cur_serial = (serial_logger_is_active() || current_settings.enable_serial_logging) ? 1 : 0;
    if (cur_serial != s_last_serial) {
        s_last_serial = cur_serial;
        if (t1_serial_btn && t1_serial_btn_lbl) {
            lv_label_set_text(t1_serial_btn_lbl, cur_serial ? "SERIAL: ON" : "SERIAL: OFF");
            lv_obj_set_style_bg_color(t1_serial_btn, cur_serial ? COLOR_BTN_ACTIVE_GREEN : COLOR_BTN_INACTIVE_BLUE, 0);
        }
    }

    // Row 2, Col 1: NMEA: ON / NMEA: OFF
    int cur_nmea = (nmea_logger_is_active() || current_settings.enable_nmea_logging) ? 1 : 0;
    if (cur_nmea != s_last_nmea) {
        s_last_nmea = cur_nmea;
        if (t1_nmea_btn && t1_nmea_btn_lbl) {
            lv_label_set_text(t1_nmea_btn_lbl, cur_nmea ? "NMEA: ON" : "NMEA: OFF");
            lv_obj_set_style_bg_color(t1_nmea_btn, cur_nmea ? COLOR_BTN_ACTIVE_GREEN : COLOR_BTN_INACTIVE_BLUE, 0);
        }
    }

    // Tile 0 Route REC Button (t0_rec_btn)
    int cur_route = route_logger_is_active() ? 1 : 0;
    if (cur_route != s_last_route) {
        s_last_route = cur_route;
        if (t0_rec_btn && t0_rec_btn_lbl) {
            lv_label_set_text(t0_rec_btn_lbl, cur_route ? "REC ON" : "REC OFF");
            lv_obj_set_style_bg_color(t0_rec_btn, cur_route ? COLOR_BTN_ACTIVE_GREEN : COLOR_BTN_INACTIVE_BLUE, 0);
        }
    }

    // Row 2, Col 2: RTCM: ON / RTCM: OFF
    int cur_rtcm = (current_settings.enable_rtcm_logging || rtcm_logger_is_active()) ? 1 : 0;
    if (cur_rtcm != s_last_rtcm) {
        s_last_rtcm = cur_rtcm;
        if (t1_rtcm_btn && t1_rtcm_btn_lbl) {
            lv_label_set_text(t1_rtcm_btn_lbl, cur_rtcm ? "RTCM: ON" : "RTCM: OFF");
            lv_obj_set_style_bg_color(t1_rtcm_btn, cur_rtcm ? COLOR_BTN_ACTIVE_GREEN : COLOR_BTN_INACTIVE_BLUE, 0);
        }
    }

    // Row 3, Col 0: TELEM: ON / TELEM: OFF
    int cur_telem = current_settings.enable_telemetry_logging ? 1 : 0;
    if (cur_telem != s_last_telem) {
        s_last_telem = cur_telem;
        if (t1_telem_btn && t1_telem_btn_lbl) {
            lv_label_set_text(t1_telem_btn_lbl, cur_telem ? "TELEM: ON" : "TELEM: OFF");
            lv_obj_set_style_bg_color(t1_telem_btn, cur_telem ? COLOR_BTN_ACTIVE_GREEN : COLOR_BTN_INACTIVE_BLUE, 0);
        }
    }

    // Row 3, Col 1: IMU: ON / IMU: OFF
    int cur_imu = current_settings.enable_imu_logging ? 1 : 0;
    if (cur_imu != s_last_imu) {
        s_last_imu = cur_imu;
        if (t1_imu_btn && t1_imu_btn_lbl) {
            lv_label_set_text(t1_imu_btn_lbl, cur_imu ? "IMU: ON" : "IMU: OFF");
            lv_obj_set_style_bg_color(t1_imu_btn, cur_imu ? COLOR_BTN_ACTIVE_GREEN : COLOR_BTN_INACTIVE_BLUE, 0);
        }
    }

    // Row 3, Col 2: TOUCH: ON / TOUCH: OFF
    int cur_touch = current_settings.enable_touch_logging ? 1 : 0;
    if (cur_touch != s_last_touch) {
        s_last_touch = cur_touch;
        if (t1_touch_btn && t1_touch_btn_lbl) {
            lv_label_set_text(t1_touch_btn_lbl, cur_touch ? "TOUCH: ON" : "TOUCH: OFF");
            lv_obj_set_style_bg_color(t1_touch_btn, cur_touch ? COLOR_BTN_ACTIVE_GREEN : COLOR_BTN_INACTIVE_BLUE, 0);
        }
    }

    // Row 4, Col 0: CAL MAG (Reflects 3D Ellipsoid Calibration Status, jumps to Tile 3)
    int cur_calmag = current_settings.mag_ellipsoid_cal_valid ? 1 : 0;
    if (cur_calmag != s_last_calmag) {
        s_last_calmag = cur_calmag;
        if (t5_calmag_btn && t5_calmag_btn_lbl) {
            if (cur_calmag) {
                lv_label_set_text(t5_calmag_btn_lbl, "CAL MAG:\nOK");
                lv_obj_set_style_bg_color(t5_calmag_btn, COLOR_BTN_ACTIVE_GREEN, 0);
            } else {
                lv_label_set_text(t5_calmag_btn_lbl, "CAL MAG");
                lv_obj_set_style_bg_color(t5_calmag_btn, lv_color_hex(0x2563EB), 0);
            }
        }
    }

    // Row 4, Col 1: UNITS: IMP / UNITS: SI
    int cur_units = current_settings.use_imperial_units ? 1 : 0;
    if (cur_units != s_last_units) {
        s_last_units = cur_units;
        if (t1_unit_btn && t1_unit_btn_lbl) {
            if (!current_settings.use_imperial_units) {
                lv_label_set_text(t1_unit_btn_lbl, "UNITS: SI");
                lv_obj_set_style_bg_color(t1_unit_btn, COLOR_UNIT_SI_TEAL, 0);
                if (hdr_units_lbl) {
                    lv_label_set_text(hdr_units_lbl, "SI");
                    lv_obj_set_style_text_color(hdr_units_lbl, COLOR_UNIT_SI_TEAL, 0);
                }
            } else {
                lv_label_set_text(t1_unit_btn_lbl, "UNITS: IMP");
                lv_obj_set_style_bg_color(t1_unit_btn, COLOR_UNIT_IMP_AMBER, 0);
                if (hdr_units_lbl) {
                    lv_label_set_text(hdr_units_lbl, "IMP");
                    lv_obj_set_style_text_color(hdr_units_lbl, COLOR_UNIT_IMP_AMBER, 0);
                }
            }
        }
    }

    // Row 4, Col 2: AUTO-OFF: 5 min / 15 min / 30 min / DISABLED
    int cur_autooff = (int)current_settings.auto_shutdown_battery_min;
    if (cur_autooff != s_last_autooff) {
        s_last_autooff = cur_autooff;
        if (t5_autooff_btn && t5_autooff_btn_lbl) {
            if (cur_autooff == 0) {
                lv_label_set_text(t5_autooff_btn_lbl, "AUTO-OFF\nDISABLED");
                lv_obj_set_style_bg_color(t5_autooff_btn, lv_color_hex(0x555555), 0);
            } else {
                char ao_buf[32];
                snprintf(ao_buf, sizeof(ao_buf), "AUTO-OFF\n%d min", cur_autooff);
                lv_label_set_text(t5_autooff_btn_lbl, ao_buf);
                lv_obj_set_style_bg_color(t5_autooff_btn, lv_color_hex(0x2F3B4C), 0);
            }
        }
    }

    // Row 5, Col 0: LORA: 19.2k / REQ: 9.6k
    if (t1_lora_btn && t1_lora_btn_lbl) {
        char l_buf[32];
        if (e220_is_rate_switch_pending()) {
            snprintf(l_buf, sizeof(l_buf), "REQ: %.1fk", e220_get_requested_air_rate());
            lv_label_set_text(t1_lora_btn_lbl, l_buf);
            lv_obj_set_style_bg_color(t1_lora_btn, lv_color_hex(0xD97706), 0); // Amber
        } else {
            snprintf(l_buf, sizeof(l_buf), "LORA: %.1fk", e220_get_active_air_rate());
            lv_label_set_text(t1_lora_btn_lbl, l_buf);
            lv_obj_set_style_bg_color(t1_lora_btn, lv_color_hex(0x2F3B4C), 0);
        }
    }

    // Row 5, Col 1: Consolidated COMMS Radio Cycle (BLE -> WIFI -> OFF -> BLE)
    int cur_comms = (int)ble_bridge_get_comms_ui_state();
    if (cur_comms != s_last_comms) {
        s_last_comms = cur_comms;
        if (t5_comms_btn && t5_comms_btn_lbl) {
            if (cur_comms == ROVER_COMMS_BLE_SLAVE) {
                lv_label_set_text(t5_comms_btn_lbl, "BLE: SLAVE");
                lv_obj_set_style_bg_color(t5_comms_btn, COLOR_BTN_ACTIVE_GREEN, 0);
            } else if (cur_comms == ROVER_COMMS_BLE_CONN) {
                lv_label_set_text(t5_comms_btn_lbl, "BLE: CONN");
                lv_obj_set_style_bg_color(t5_comms_btn, lv_color_hex(0x0891B2), 0);
            } else if (cur_comms == ROVER_COMMS_BLE_ADV) {
                lv_label_set_text(t5_comms_btn_lbl, "BLE: ADV");
                lv_obj_set_style_bg_color(t5_comms_btn, lv_color_hex(0x0284C7), 0);
            } else if (cur_comms == ROVER_COMMS_BLE_INIT) {
                lv_label_set_text(t5_comms_btn_lbl, "BLE: INIT");
                lv_obj_set_style_bg_color(t5_comms_btn, lv_color_hex(0xD97706), 0);
            } else if (cur_comms == ROVER_COMMS_WIFI_CONN) {
                lv_label_set_text(t5_comms_btn_lbl, "WIFI: CONN");
                lv_obj_set_style_bg_color(t5_comms_btn, COLOR_BTN_WIFI_CONN, 0);
            } else if (cur_comms == ROVER_COMMS_WIFI_STA) {
                lv_label_set_text(t5_comms_btn_lbl, "WIFI: STA");
                lv_obj_set_style_bg_color(t5_comms_btn, COLOR_BTN_WIFI_STA, 0);
            } else if (cur_comms == ROVER_COMMS_WIFI_AP) {
                lv_label_set_text(t5_comms_btn_lbl, "WIFI: AP");
                lv_obj_set_style_bg_color(t5_comms_btn, COLOR_BTN_WIFI_AP, 0);
            } else {
                lv_label_set_text(t5_comms_btn_lbl, "COMMS: OFF");
                lv_obj_set_style_bg_color(t5_comms_btn, COLOR_BTN_INACTIVE_BLUE, 0);
            }
        }
    }

    // Row 5, Col 2: RTCM Correction Source Arbiter Mode (AUTO / LORA / NTRIP)
    RtcmSourceMode_t r_mode = rtcm_arbiter_get_mode();
    RtcmSourceId_t   r_src  = rtcm_arbiter_get_active_source();
    int cur_rtcm_key = (((int)r_mode) << 4) | (int)r_src;
    if (cur_rtcm_key != s_last_rtcm_src_key) {
        s_last_rtcm_src_key = cur_rtcm_key;
        if (t5_rtcm_src_btn && t5_rtcm_src_btn_lbl) {
            if (r_mode == RTCM_MODE_LORA_ONLY) {
                lv_label_set_text(t5_rtcm_src_btn_lbl, "RTCM:\nLORA ONLY");
                lv_obj_set_style_bg_color(t5_rtcm_src_btn, lv_color_hex(0xB45309), 0);
            } else if (r_mode == RTCM_MODE_NTRIP_ONLY) {
                lv_label_set_text(t5_rtcm_src_btn_lbl, "RTCM:\nNTRIP ONLY");
                lv_obj_set_style_bg_color(t5_rtcm_src_btn, lv_color_hex(0x0891B2), 0);
            } else {
                const char *tag = (r_src == RTCM_SRC_BLE_NTRIP) ? "NTRIP" :
                                  (r_src == RTCM_SRC_LORA)      ? "LORA" : "WAIT";
                char r_buf[32];
                snprintf(r_buf, sizeof(r_buf), "RTCM: AUTO\n[%s]", tag);
                lv_label_set_text(t5_rtcm_src_btn_lbl, r_buf);
                lv_obj_set_style_bg_color(t5_rtcm_src_btn,
                                          (r_src == RTCM_SRC_BLE_NTRIP) ? lv_color_hex(0x0891B2) : lv_color_hex(0x2F3B4C),
                                          0);
            }
        }
    }

    // Compact Tare / Reset Tare Button on Tile 3
    bool has_tare = (fabsf(current_settings.pole_tilt_tare_pitch_deg) > 0.05f ||
                     fabsf(current_settings.pole_tilt_tare_roll_deg)  > 0.05f);
    int cur_tare = has_tare ? 1 : 0;
    if (cur_tare != s_last_tare) {
        s_last_tare = cur_tare;
        if (t3_tare_btn && t3_tare_lbl) {
            if (has_tare) {
                lv_label_set_text(t3_tare_lbl, "RST");
                lv_obj_set_style_bg_color(t3_tare_btn, lv_color_hex(0xEA580C), 0); // Orange
            } else {
                lv_label_set_text(t3_tare_lbl, "TARE");
                lv_obj_set_style_bg_color(t3_tare_btn, lv_color_hex(0x2563EB), 0); // Blue
            }
        }
    }
}

static void bright_toggle_btn_cb(lv_event_t *e)
{
    (void)e;
    uint8_t b = current_settings.brightness;
    if (b < 30) b = 40;
    else if (b < 50) b = 60;
    else if (b < 70) b = 80;
    else if (b < 90) b = 100;
    else b = 20;

    set_rover_brightness(b);
    mark_settings_dirty();
    update_all_control_buttons();
    play_rover_beep(1400, 25);
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Brightness clicked -> %u%%\n", b);
    }
}

static void vol_toggle_btn_cb(lv_event_t *e)
{
    (void)e;
    uint8_t v = current_settings.volume;
    if (v == 0) v = 20;
    else if (v < 30) v = 40;
    else if (v < 50) v = 60;
    else if (v < 70) v = 80;
    else if (v < 90) v = 100;
    else v = 0;

    set_rover_volume(v);
    current_settings.is_muted = (v == 0);
    mark_settings_dirty();
    update_all_control_buttons();

    if (v == 0) {
        play_rover_beep(900, 30);
    } else {
        play_rover_beep(1750, 40);
    }
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Volume clicked -> %u%%%s\n",
                     v, current_settings.is_muted ? " (MUTED)" : "");
    }
}

static void audio_mode_toggle_btn_cb(lv_event_t *e)
{
    (void)e;
    if (current_settings.audio_nav_mode == AUDIO_NAV_BEEPS) {
        current_settings.audio_nav_mode = AUDIO_NAV_GEIGER;
        play_rover_beep(1800, 15);
    } else if (current_settings.audio_nav_mode == AUDIO_NAV_GEIGER) {
        current_settings.audio_nav_mode = AUDIO_NAV_MUTE;
        play_rover_beep(900, 30);
    } else {
        current_settings.audio_nav_mode = AUDIO_NAV_BEEPS;
        play_rover_beep(1500, 30);
    }
    mark_settings_dirty();
    update_all_control_buttons();
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Audio Nav Mode clicked -> %s\n",
                     current_settings.audio_nav_mode == AUDIO_NAV_GEIGER ? "GEIGER" :
                     current_settings.audio_nav_mode == AUDIO_NAV_MUTE ? "MUTE" : "BEEPS");
    }
}

static void lora_rate_btn_cb(lv_event_t *e)
{
    (void)e;
    float cur_rate = e220_is_rate_switch_pending()
                         ? e220_get_requested_air_rate()
                         : e220_get_active_air_rate();
    float next_rate;
    if (cur_rate > 14.0f) {
        next_rate = 9.6f;
    } else if (cur_rate > 7.0f) {
        next_rate = 4.8f;
    } else {
        next_rate = 19.2f;
    }

    if (e220_request_air_rate_switch(next_rate)) {
        play_rover_beep(1800, 60);
        if (current_settings.enable_touch_logging) {
            rover_printf("[TOUCH BTN] LoRa Rate clicked -> Requesting %.1fk\n", next_rate);
        }
    } else {
        play_rover_beep(600, 40); // Busy tone: OTA switch handshake already in flight
    }
    update_all_control_buttons();
}

static void unit_toggle_btn_cb(lv_event_t *e)
{
    (void)e;
    current_settings.use_imperial_units = !current_settings.use_imperial_units;
    mark_settings_dirty();
    update_all_control_buttons();
    play_rover_beep(1400, 25);
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Units clicked -> %s\n",
                     current_settings.use_imperial_units ? "IMP" : "SI");
    }
}

static void serial_log_toggle_btn_cb(lv_event_t *e)
{
    (void)e;
    current_settings.enable_serial_logging = !current_settings.enable_serial_logging;
    mark_settings_dirty();
    if (current_settings.enable_serial_logging) {
        serial_logger_start();
        play_rover_beep(1800, 40);
    } else {
        serial_logger_stop();
        play_rover_beep(1200, 30);
    }
    update_all_control_buttons();
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Serial Log clicked -> %s\n",
                     current_settings.enable_serial_logging ? "ON" : "OFF");
    }
}

static void nmea_toggle_btn_cb(lv_event_t *e)
{
    (void)e;
    if (nmea_logger_is_active()) {
        nmea_logger_stop();
        current_settings.enable_nmea_logging = false;
        play_rover_beep(1200, 30);
    } else {
        nmea_logger_start();
        current_settings.enable_nmea_logging = true;
        play_rover_beep(1800, 40);
    }
    mark_settings_dirty();
    update_all_control_buttons();
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] NMEA Log clicked -> %s\n",
                     current_settings.enable_nmea_logging ? "ON" : "OFF");
    }
}

static void comms_cycle_btn_cb(lv_event_t *e)
{
    (void)e;
    ble_bridge_request_comms_cycle();
    RoverCommsUiState_t next_st = ble_bridge_get_comms_ui_state();
    play_rover_beep((next_st == ROVER_COMMS_OFF) ? 1100 : 1800,
                    (next_st == ROVER_COMMS_OFF) ? 30 : 40);
    update_all_control_buttons();
    if (current_settings.enable_touch_logging) {
        const char *tag = (next_st == ROVER_COMMS_OFF)       ? "OFF" :
                          (next_st == ROVER_COMMS_WIFI_CONN) ? "WIFI" : "BLE";
        rover_printf("[TOUCH BTN] COMMS clicked -> %s\n", tag);
    }
}

static void rtcm_src_toggle_btn_cb(lv_event_t *e)
{
    (void)e;
    RtcmSourceMode_t cur = rtcm_arbiter_get_mode();
    RtcmSourceMode_t nxt = (cur == RTCM_MODE_AUTO)      ? RTCM_MODE_LORA_ONLY :
                           (cur == RTCM_MODE_LORA_ONLY) ? RTCM_MODE_NTRIP_ONLY :
                                                          RTCM_MODE_AUTO;
    rtcm_arbiter_set_mode(nxt);
    mark_settings_dirty();
    char jbuf[48];
    snprintf(jbuf, sizeof(jbuf), "{\"rtcm_source_mode\":\"%s\"}", current_settings.rtcm_source_mode);
    ble_bridge_notify_local_ui_change(jbuf, NULL);
    play_rover_beep((nxt == RTCM_MODE_AUTO) ? 1500 : (nxt == RTCM_MODE_LORA_ONLY ? 1650 : 1800), 35);
    update_all_control_buttons();
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] RTCM SRC clicked -> %s\n", current_settings.rtcm_source_mode);
    }
}

void update_wifi_ip_label(const char* ip)
{
    portENTER_CRITICAL(&s_wifi_lbl_mux);
    if (ip && ip[0]) {
        strncpy(s_pending_wifi_status, ip, sizeof(s_pending_wifi_status) - 1);
        s_pending_wifi_status[sizeof(s_pending_wifi_status) - 1] = '\0';
    } else {
        strncpy(s_pending_wifi_status, "Wi-Fi Inactive", sizeof(s_pending_wifi_status) - 1);
        s_pending_wifi_status[sizeof(s_pending_wifi_status) - 1] = '\0';
    }
    s_wifi_status_dirty = true;
    portEXIT_CRITICAL(&s_wifi_lbl_mux);
}

static void diag_rtcm_btn_cb(lv_event_t *e)
{
    (void)e;
    current_settings.enable_rtcm_logging = !current_settings.enable_rtcm_logging;
    mark_settings_dirty();
    if (current_settings.enable_rtcm_logging) {
        rtcm_logger_start();
        play_rover_beep(1800, 40);
    } else {
        rtcm_logger_stop();
        play_rover_beep(1200, 30);
    }
    update_all_control_buttons();
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Diag RTCM Log clicked -> %s\n",
                     current_settings.enable_rtcm_logging ? "ON" : "OFF");
    }
}

static void diag_imu_btn_cb(lv_event_t *e)
{
    (void)e;
    current_settings.enable_imu_logging = !current_settings.enable_imu_logging;
    mark_settings_dirty();
    play_rover_beep(current_settings.enable_imu_logging ? 1800 : 1200, 30);
    update_all_control_buttons();
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Diag IMU Log clicked -> %s\n",
                     current_settings.enable_imu_logging ? "ON" : "OFF");
    }
}

static void diag_telem_btn_cb(lv_event_t *e)
{
    (void)e;
    current_settings.enable_telemetry_logging = !current_settings.enable_telemetry_logging;
    mark_settings_dirty();
    play_rover_beep(current_settings.enable_telemetry_logging ? 1800 : 1200, 30);
    update_all_control_buttons();
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Diag Telem Log clicked -> %s\n",
                     current_settings.enable_telemetry_logging ? "ON" : "OFF");
    }
}

static void diag_touch_btn_cb(lv_event_t *e)
{
    (void)e;
    current_settings.enable_touch_logging = !current_settings.enable_touch_logging;
    mark_settings_dirty();
    play_rover_beep(current_settings.enable_touch_logging ? 1800 : 1200, 30);
    update_all_control_buttons();
    rover_printf("[TOUCH BTN] Diag Touch Log clicked -> %s\n",
                 current_settings.enable_touch_logging ? "ON" : "OFF");
}

static void autooff_toggle_btn_cb(lv_event_t *e)
{
    (void)e;
    uint8_t cur = current_settings.auto_shutdown_battery_min;
    if (cur == 0)      current_settings.auto_shutdown_battery_min = 5;
    else if (cur <= 5) current_settings.auto_shutdown_battery_min = 15;
    else if (cur <= 15) current_settings.auto_shutdown_battery_min = 30;
    else               current_settings.auto_shutdown_battery_min = 0;

    mark_settings_dirty();
    rover_note_user_activity();
    update_all_control_buttons();
    play_rover_beep(current_settings.auto_shutdown_battery_min == 0 ? 1000 : 1500, 30);
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] Auto-Off clicked -> %u min\n",
                     (unsigned int)current_settings.auto_shutdown_battery_min);
    }
}

static void cal_mag_shortcut_btn_cb(lv_event_t *e)
{
    (void)e;
    play_rover_beep(1500, 25);
    switch_to_tile_index(3, LV_ANIM_ON);
    if (current_settings.enable_touch_logging) {
        rover_printf("[TOUCH BTN] CAL MAG shortcut clicked -> Tile 4 (Compass & Level)\n");
    }
}

static lv_obj_t* create_grid_button(lv_obj_t *parent, int col, int row, lv_event_cb_t cb, lv_obj_t **out_lbl)
{
    int x = (col == 0) ? 10 : (col == 1 ? 113 : 216);
    int y = 10 + (row * 54);
    lv_obj_t *btn = lv_button_create(parent);
    lv_obj_set_size(btn, 94, 44);
    lv_obj_align(btn, LV_ALIGN_TOP_LEFT, x, y);
    lv_obj_set_style_radius(btn, 6, 0);
    lv_obj_set_style_pad_all(btn, 0, 0);
    lv_obj_set_ext_click_area(btn, 4);
    if (cb) lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *lbl = lv_label_create(btn);
    lv_obj_set_style_text_font(lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(lbl);
    if (out_lbl) *out_lbl = lbl;
    return btn;
}

void ui_tile5_settings_create(lv_obj_t *parent)
{
    // ---------------------------------------------------------------------
    // Dedicated Quick Settings Deck (5x3 Grid of 94x44 Buttons, 10px row gap)
    // ---------------------------------------------------------------------
    // Row 0 (y=10): VOL, AUD, BRI
    t1_vol_btn     = create_grid_button(parent, 0, 0, vol_toggle_btn_cb,        &t1_vol_btn_lbl);
    t1_aud_btn     = create_grid_button(parent, 1, 0, audio_mode_toggle_btn_cb, &t1_aud_btn_lbl);
    t1_bright_btn  = create_grid_button(parent, 2, 0, bright_toggle_btn_cb,     &t1_bright_btn_lbl);

    // Row 1 (y=64): SERIAL, NMEA, RTCM
    t1_serial_btn  = create_grid_button(parent, 0, 1, serial_log_toggle_btn_cb, &t1_serial_btn_lbl);
    t1_nmea_btn    = create_grid_button(parent, 1, 1, nmea_toggle_btn_cb,       &t1_nmea_btn_lbl);
    t1_rtcm_btn    = create_grid_button(parent, 2, 1, diag_rtcm_btn_cb,         &t1_rtcm_btn_lbl);

    // Row 2 (y=118): TELEM, IMU, TOUCH
    t1_telem_btn   = create_grid_button(parent, 0, 2, diag_telem_btn_cb,        &t1_telem_btn_lbl);
    t1_imu_btn     = create_grid_button(parent, 1, 2, diag_imu_btn_cb,          &t1_imu_btn_lbl);
    t1_touch_btn   = create_grid_button(parent, 2, 2, diag_touch_btn_cb,        &t1_touch_btn_lbl);

    // Row 3 (y=172): CAL MAG, UNITS, AUTO-OFF
    t5_calmag_btn   = create_grid_button(parent, 0, 3, cal_mag_shortcut_btn_cb, &t5_calmag_btn_lbl);
    lv_label_set_text(t5_calmag_btn_lbl, "CAL MAG");
    lv_obj_set_style_bg_color(t5_calmag_btn, lv_color_hex(0x2563EB), 0);

    t1_unit_btn     = create_grid_button(parent, 1, 3, unit_toggle_btn_cb,      &t1_unit_btn_lbl);
    t5_autooff_btn  = create_grid_button(parent, 2, 3, autooff_toggle_btn_cb,   &t5_autooff_btn_lbl);

    // Row 4 (y=226): LORA, COMMS, RTCM SRC
    t1_lora_btn     = create_grid_button(parent, 0, 4, lora_rate_btn_cb,        &t1_lora_btn_lbl);

    t5_comms_btn    = create_grid_button(parent, 1, 4, comms_cycle_btn_cb,      &t5_comms_btn_lbl);
    lv_label_set_text(t5_comms_btn_lbl, "COMMS: OFF");
    lv_obj_set_style_bg_color(t5_comms_btn, COLOR_BTN_INACTIVE_BLUE, 0);

    t5_rtcm_src_btn = create_grid_button(parent, 2, 4, rtcm_src_toggle_btn_cb,  &t5_rtcm_src_btn_lbl);
    lv_label_set_text(t5_rtcm_src_btn_lbl, "RTCM: AUTO\n[WAIT]");
    lv_obj_set_style_bg_color(t5_rtcm_src_btn, lv_color_hex(0x2F3B4C), 0);

    // Horizontal divider below 5x3 grid (y=278)
    lv_obj_t *t5_sep = lv_obj_create(parent);
    lv_obj_set_size(t5_sep, 300, 1);
    lv_obj_align(t5_sep, LV_ALIGN_TOP_MID, 0, 278);
    lv_obj_set_style_bg_color(t5_sep, lv_color_hex(0x283B47), 0);
    lv_obj_set_style_border_width(t5_sep, 0, 0);
    lv_obj_set_scrollable(t5_sep, false);

    // Status panel (y=286..350)
    t1_wifi_status_lbl = lv_label_create(parent);
    lv_obj_set_width(t1_wifi_status_lbl, 300);
    lv_label_set_text(t1_wifi_status_lbl, "Comms: Initializing...");
    lv_obj_set_style_text_color(t1_wifi_status_lbl, lv_color_hex(0x94A3B8), 0);
    lv_obj_set_style_text_font(t1_wifi_status_lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_align(t1_wifi_status_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(t1_wifi_status_lbl, LV_ALIGN_TOP_MID, 0, 286);

    t5_power_status_lbl = lv_label_create(parent);
    lv_obj_set_width(t5_power_status_lbl, 300);
    lv_label_set_text(t5_power_status_lbl, "Auto-Off: Initializing...");
    lv_obj_set_style_text_color(t5_power_status_lbl, lv_color_hex(0x94A3B8), 0);
    lv_obj_set_style_text_font(t5_power_status_lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_align(t5_power_status_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(t5_power_status_lbl, LV_ALIGN_TOP_MID, 0, 308);

    t5_lora_status_lbl = lv_label_create(parent);
    lv_obj_set_width(t5_lora_status_lbl, 300);
    lv_label_set_text(t5_lora_status_lbl, "LoRa CH75 4.8k | RTCM OK:0 | SRC:AUTO");
    lv_obj_set_style_text_color(t5_lora_status_lbl, lv_color_hex(0x38BDF8), 0);
    lv_obj_set_style_text_font(t5_lora_status_lbl, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_align(t5_lora_status_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(t5_lora_status_lbl, LV_ALIGN_TOP_MID, 0, 330);

    update_all_control_buttons();
}

void ui_tile5_settings_update(const RoverUIData_t *d, char *buf, size_t buf_sz)
{
    if (active_tile_col != 5) return;

    if (t5_power_status_lbl) {
        if (d->auto_off_countdown_s > 0) {
            snprintf(buf, buf_sz, "Auto-Off: SHUTDOWN IN %us!", (unsigned int)d->auto_off_countdown_s);
            lv_obj_set_style_text_color(t5_power_status_lbl, lv_palette_main(LV_PALETTE_RED), 0);
        } else if (d->low_batt_warning) {
            snprintf(buf, buf_sz, "LOW BATT %.2fV! LOGS %s",
                     d->battery_voltage,
                     d->low_batt_logs_flushed ? "SAVED" : "FLUSHING");
            lv_obj_set_style_text_color(t5_power_status_lbl, lv_palette_main(LV_PALETTE_RED), 0);
        } else if (d->auto_off_inhibited) {
            snprintf(buf, buf_sz, "Auto-Off: Hold (%s)",
                     d->auto_off_reason[0] ? d->auto_off_reason : "Active");
            lv_obj_set_style_text_color(t5_power_status_lbl, lv_color_hex(0x94A3B8), 0);
        } else {
            uint32_t idle_m = d->auto_off_idle_s / 60U;
            uint32_t idle_s = d->auto_off_idle_s % 60U;
            snprintf(buf, buf_sz, "Auto-Off: Idle %lum%02lus / %um",
                     (unsigned long)idle_m, (unsigned long)idle_s,
                     (unsigned int)current_settings.auto_shutdown_battery_min);
            lv_obj_set_style_text_color(t5_power_status_lbl, lv_color_hex(0xF59E0B), 0);
        }
        lv_label_set_text(t5_power_status_lbl, buf);
    }

    if (t5_lora_status_lbl) {
        if (d->rtcm_arbiter.active_source == RTCM_SRC_BLE_NTRIP) {
            snprintf(buf, buf_sz, "NTRIP BLE %.1fkb/s | OK:%lu | M:%s",
                     d->rtcm_arbiter.ntrip_rx_kbps,
                     (unsigned long)d->rtcm_arbiter.ntrip_frames_ok,
                     current_settings.rtcm_source_mode);
            lv_obj_set_style_text_color(t5_lora_status_lbl, lv_color_hex(0x22D3EE), 0);
        } else {
            uint8_t ch = (d->e220_stats.cfg_channel > 0) ? d->e220_stats.cfg_channel : current_settings.e220_channel;
            snprintf(buf, buf_sz, "LoRa CH%u %.1fk | OK:%lu | M:%s",
                     (unsigned int)ch,
                     current_settings.e220_air_rate_kbps,
                     (unsigned long)d->e220_stats.rtcm_frames_ok,
                     current_settings.rtcm_source_mode);
            lv_obj_set_style_text_color(t5_lora_status_lbl, lv_color_hex(0x38BDF8), 0);
        }
        lv_label_set_text(t5_lora_status_lbl, buf);
    }

    if (t1_wifi_status_lbl && !fic_web_portal_is_active() && !fic_web_portal_is_starting()) {
        ble_bridge_format_status(buf, buf_sz);
        rover_label_set_text_if_changed(t1_wifi_status_lbl, buf);
        RoverCommsUiState_t c_st = ble_bridge_get_comms_ui_state();
        lv_color_t c = (c_st == ROVER_COMMS_BLE_CONN || c_st == ROVER_COMMS_BLE_SLAVE) ? lv_color_hex(0x22D3EE) :
                       (c_st == ROVER_COMMS_BLE_INIT)  ? lv_color_hex(0xF59E0B) :
                       (c_st == ROVER_COMMS_BLE_ADV)   ? lv_color_hex(0x38BDF8) :
                                                         lv_color_hex(0x94A3B8);
        lv_obj_set_style_text_color(t1_wifi_status_lbl, c, 0);
    }
}

void ui_tile5_sync_wifi_status(void)
{
    // Tile 1 Wi-Fi Status Label (synchronized cross-task copy)
    if (s_wifi_status_dirty && t1_wifi_status_lbl) {
        char ip[64];
        portENTER_CRITICAL(&s_wifi_lbl_mux);
        s_wifi_status_dirty = false;
        strncpy(ip, s_pending_wifi_status, sizeof(ip) - 1);
        ip[sizeof(ip) - 1] = '\0';
        portEXIT_CRITICAL(&s_wifi_lbl_mux);

        if (ip[0] && strcmp(ip, "Wi-Fi Off") != 0 && strcmp(ip, "Wi-Fi Failed") != 0 &&
            strcmp(ip, "Wi-Fi Inactive") != 0 && strcmp(ip, "Connecting...") != 0 &&
            strcmp(ip, "Starting AP...") != 0 && strcmp(ip, "0.0.0.0") != 0 &&
            strstr(ip, "...") == NULL) {
            char ip_buf[80];
            snprintf(ip_buf, sizeof(ip_buf), "Portal: http://%s/", ip);
            lv_label_set_text(t1_wifi_status_lbl, ip_buf);
            lv_obj_set_style_text_color(t1_wifi_status_lbl, lv_color_hex(0x00FF88), 0);
        } else {
            lv_label_set_text(t1_wifi_status_lbl, ip);
            lv_obj_set_style_text_color(t1_wifi_status_lbl, lv_color_hex(0x94A3B8), 0);
        }
        update_all_control_buttons();
    }
}
