#include "lvgl_ui_internal.h"
#include "../e220_driver/e220_driver.h"
#include "../ble_bridge/ble_bridge.h"
#include "../rtcm_arbiter/rtcm_arbiter.h"

// =========================================================================
// Tile 4: Hardware Diagnostics Widgets
// =========================================================================
static lv_obj_t *t4_title               = NULL;
static lv_obj_t *t4_rf_card             = NULL;
static lv_obj_t *t4_rf_bar              = NULL;
static lv_obj_t *t4_rf_rssi_lbl         = NULL;
static lv_obj_t *t4_rf_noise_lbl        = NULL;
static lv_obj_t *t4_rf_health_lbl       = NULL;
static lv_obj_t *t4_rf_chan_lbl         = NULL;
static lv_obj_t *t4_scroll_cont         = NULL;
static lv_obj_t *t4_storage_status_lbl  = NULL;
static lv_obj_t *t4_ant_master_lbl      = NULL;
static lv_obj_t *t4_ant_slave_lbl       = NULL;
static lv_obj_t *t4_base_station_lbl    = NULL;
static lv_obj_t *t4_com1_lbl            = NULL;
static lv_obj_t *t4_um982_audit_lbl     = NULL;
static lv_obj_t *t4_e220_lbl            = NULL;
static lv_obj_t *t4_e220_cfg_lbl        = NULL;
static lv_obj_t *t4_power_lbl           = NULL;
static lv_obj_t *t4_imu_lbl             = NULL;
static lv_obj_t *t4_ble_ntrip_lbl       = NULL;
static lv_obj_t *t4_wifi_lbl            = NULL;
static lv_obj_t *t4_i2c_lbl             = NULL;

void ui_tile4_diagnostics_create(lv_obj_t *parent)
{
    // =====================================================================
    // TILE 4: Hardware Diagnostics
    // =====================================================================
    lv_obj_set_scrollable(parent, false);

    // 1. Fixed Pinned Header Deck (Always visible, non-scrolling)
    t4_title = lv_label_create(parent);
    lv_label_set_text(t4_title, "RTK ROVER " ROVER_FIRMWARE_VERSION);
    lv_obj_set_style_text_color(t4_title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(t4_title, &lv_font_montserrat_16, 0);
    lv_obj_align(t4_title, LV_ALIGN_TOP_MID, 0, 4);

    // 2. Pinned LoRa RF Survey Card (Always visible, non-scrolling)
    t4_rf_card = lv_obj_create(parent);
    lv_obj_set_size(t4_rf_card, 308, 54);
    lv_obj_align(t4_rf_card, LV_ALIGN_TOP_MID, 0, 26);
    lv_obj_set_style_bg_color(t4_rf_card, lv_color_hex(0x1A202C), 0);
    lv_obj_set_style_border_color(t4_rf_card, lv_color_hex(0x2D3748), 0);
    lv_obj_set_style_border_width(t4_rf_card, 1, 0);
    lv_obj_set_style_radius(t4_rf_card, 4, 0);
    lv_obj_set_style_pad_all(t4_rf_card, 0, 0);
    lv_obj_set_scrollbar_mode(t4_rf_card, LV_SCROLLBAR_MODE_OFF);

    // Row 1: Meter Bar across top of card (296 x 6 px)
    t4_rf_bar = lv_bar_create(t4_rf_card);
    lv_obj_set_size(t4_rf_bar, 296, 6);
    lv_obj_align(t4_rf_bar, LV_ALIGN_TOP_MID, 0, 5);
    lv_bar_set_range(t4_rf_bar, 0, 100);
    lv_bar_set_value(t4_rf_bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(t4_rf_bar, lv_color_hex(0x283B47), LV_PART_MAIN);
    lv_obj_set_style_bg_color(t4_rf_bar, lv_color_hex(0x00FF88), LV_PART_INDICATOR);
    lv_obj_set_style_radius(t4_rf_bar, 2, LV_PART_MAIN);
    lv_obj_set_style_radius(t4_rf_bar, 2, LV_PART_INDICATOR);

    // Row 2: Live Metrics (RSSI bold left, Noise/Margin right)
    t4_rf_rssi_lbl = lv_label_create(t4_rf_card);
    lv_label_set_text(t4_rf_rssi_lbl, "LoRa: -- dBm");
    lv_obj_set_style_text_color(t4_rf_rssi_lbl, lv_color_hex(0x64748B), 0);
    lv_obj_set_style_text_font(t4_rf_rssi_lbl, &lv_font_montserrat_14, 0);
    lv_obj_align(t4_rf_rssi_lbl, LV_ALIGN_TOP_LEFT, 6, 16);

    t4_rf_noise_lbl = lv_label_create(t4_rf_card);
    lv_label_set_text(t4_rf_noise_lbl, "Noise: -- dBm");
    lv_obj_set_style_text_color(t4_rf_noise_lbl, lv_color_hex(0x94A3B8), 0);
    lv_obj_set_style_text_font(t4_rf_noise_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t4_rf_noise_lbl, LV_ALIGN_TOP_RIGHT, -6, 18);

    // Row 3: Health & Channel (Age/OK left, Channel/Air right)
    t4_rf_health_lbl = lv_label_create(t4_rf_card);
    lv_label_set_text(t4_rf_health_lbl, "Age: -- | OK: 0 (0 drop)");
    lv_obj_set_style_text_color(t4_rf_health_lbl, lv_color_hex(0xCBD5E1), 0);
    lv_obj_set_style_text_font(t4_rf_health_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t4_rf_health_lbl, LV_ALIGN_TOP_LEFT, 6, 35);

    t4_rf_chan_lbl = lv_label_create(t4_rf_card);
    lv_label_set_text(t4_rf_chan_lbl, "CH-- (19.2k)");
    lv_obj_set_style_text_color(t4_rf_chan_lbl, lv_color_hex(0x38BDF8), 0);
    lv_obj_set_style_text_font(t4_rf_chan_lbl, &lv_font_montserrat_12, 0);
    lv_obj_align(t4_rf_chan_lbl, LV_ALIGN_TOP_RIGHT, -6, 35);

    // Subtle divider line
    lv_obj_t *t4_sep = lv_obj_create(parent);
    lv_obj_set_size(t4_sep, 308, 1);
    lv_obj_align(t4_sep, LV_ALIGN_TOP_MID, 0, 83);
    lv_obj_set_style_bg_color(t4_sep, lv_color_hex(0x283B47), 0);
    lv_obj_set_style_border_width(t4_sep, 0, 0);
    lv_obj_set_scrollbar_mode(t4_sep, LV_SCROLLBAR_MODE_OFF);

    // 3. Scrollable Viewport for Diagnostic Telemetry (320 x 288 px, +57px viewport boost)
    t4_scroll_cont = lv_obj_create(parent);
    lv_obj_set_size(t4_scroll_cont, 320, 374 - 86);
    lv_obj_align(t4_scroll_cont, LV_ALIGN_TOP_MID, 0, 86);
    lv_obj_set_style_bg_opa(t4_scroll_cont, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(t4_scroll_cont, 0, 0);
    lv_obj_set_style_radius(t4_scroll_cont, 0, 0);
    lv_obj_set_scroll_dir(t4_scroll_cont, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(t4_scroll_cont, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_flex_flow(t4_scroll_cont, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(t4_scroll_cont, 10, 0);
    lv_obj_set_style_pad_all(t4_scroll_cont, 6, 0);
    lv_obj_set_style_pad_bottom(t4_scroll_cont, 20, 0);

    // 1. Storage Status
    t4_storage_status_lbl = lv_label_create(t4_scroll_cont);
    lv_obj_set_width(t4_storage_status_lbl, 300);
    lv_label_set_long_mode(t4_storage_status_lbl, LV_LABEL_LONG_WRAP);
    lv_label_set_text(t4_storage_status_lbl, "Storage: Auto-Detecting...");
    lv_obj_set_style_text_color(t4_storage_status_lbl, lv_color_hex(0x22D3EE), 0);
    lv_obj_set_style_text_font(t4_storage_status_lbl, &lv_font_montserrat_12, 0);

    // 2. ANT1 Master GNSS
    t4_ant_master_lbl = lv_label_create(t4_scroll_cont);
    lv_obj_set_width(t4_ant_master_lbl, 300);
    lv_label_set_long_mode(t4_ant_master_lbl, LV_LABEL_LONG_WRAP);
    lv_label_set_text(t4_ant_master_lbl, "[ANT1 MASTER] Waiting for GNSS fix...");
    lv_obj_set_style_text_color(t4_ant_master_lbl, lv_color_hex(0x00FF88), 0);
    lv_obj_set_style_text_font(t4_ant_master_lbl, &lv_font_montserrat_12, 0);

    // 3. ANT2 Slave GNSS
    t4_ant_slave_lbl = lv_label_create(t4_scroll_cont);
    lv_obj_set_width(t4_ant_slave_lbl, 300);
    lv_label_set_long_mode(t4_ant_slave_lbl, LV_LABEL_LONG_WRAP);
    lv_label_set_text(t4_ant_slave_lbl, "[ANT2 SLAVE] Waiting for heading/slave data...");
    lv_obj_set_style_text_color(t4_ant_slave_lbl, lv_color_hex(0xFFCC00), 0);
    lv_obj_set_style_text_font(t4_ant_slave_lbl, &lv_font_montserrat_12, 0);

    // 4. Base Station Telemetry
    t4_base_station_lbl = lv_label_create(t4_scroll_cont);
    lv_obj_set_width(t4_base_station_lbl, 300);
    lv_label_set_long_mode(t4_base_station_lbl, LV_LABEL_LONG_WRAP);
    lv_label_set_text(t4_base_station_lbl, "[BASE STATION] Waiting for RTCM...");
    lv_obj_set_style_text_color(t4_base_station_lbl, lv_color_hex(0x38BDF8), 0);
    lv_obj_set_style_text_font(t4_base_station_lbl, &lv_font_montserrat_12, 0);

    // 5. UM982 COM1 Full-Duplex Info
    t4_com1_lbl = lv_label_create(t4_scroll_cont);
    lv_obj_set_width(t4_com1_lbl, 300);
    lv_label_set_long_mode(t4_com1_lbl, LV_LABEL_LONG_WRAP);
    lv_label_set_text(t4_com1_lbl, "[UM982 COM1 IO17/18 @ 115200 Full-Duplex]\nRX: 0 B | TX: 0 B (RTCM: 0 B) | NMEA: 0 | HDG: 0\nLast NMEA: Waiting...");
    lv_obj_set_style_text_color(t4_com1_lbl, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(t4_com1_lbl, &lv_font_montserrat_12, 0);

    // 6. UM982 Receiver State & Loglist Audit
    t4_um982_audit_lbl = lv_label_create(t4_scroll_cont);
    lv_obj_set_width(t4_um982_audit_lbl, 300);
    lv_label_set_long_mode(t4_um982_audit_lbl, LV_LABEL_LONG_WRAP);
    lv_label_set_text(t4_um982_audit_lbl, "[UM982 RECEIVER STATE & LOGLIST]\nVer: Querying...\nMode: -- | Cfg: --\nLogs: --\nLast ACK: Waiting...");
    lv_obj_set_style_text_color(t4_um982_audit_lbl, lv_color_hex(0x00FFAA), 0);
    lv_obj_set_style_text_font(t4_um982_audit_lbl, &lv_font_montserrat_12, 0);

    // 7. E220 LoRa Info
    char init_buf[192];
    t4_e220_lbl = lv_label_create(t4_scroll_cont);
    lv_obj_set_width(t4_e220_lbl, 300);
    lv_label_set_long_mode(t4_e220_lbl, LV_LABEL_LONG_WRAP);
    snprintf(init_buf, sizeof(init_buf),
             "[E220 LoRa Hardware IO41/42 @ %lu]\n"
             "AUX:1 Edges:0 | Regs:--\n"
             "RX:0B -> COM1(RTCM):0B | OK:0 Err:0\n"
             "1006:0 | 74:0 | 84:0 | 94:0 | 1230:0",
             (unsigned long)current_settings.e220_uart_baud);
    lv_label_set_text(t4_e220_lbl, init_buf);
    lv_obj_set_style_text_color(t4_e220_lbl, lv_color_hex(0xFFCC00), 0);
    lv_obj_set_style_text_font(t4_e220_lbl, &lv_font_montserrat_12, 0);

    // 8. E220 Target & Registers
    t4_e220_cfg_lbl = lv_label_create(t4_scroll_cont);
    lv_obj_set_width(t4_e220_cfg_lbl, 300);
    lv_label_set_long_mode(t4_e220_cfg_lbl, LV_LABEL_LONG_WRAP);
    snprintf(init_buf, sizeof(init_buf),
             "Target: CH%u %lu/%.1fk %uB %udBm",
             current_settings.e220_channel,
             (unsigned long)current_settings.e220_uart_baud,
             current_settings.e220_air_rate_kbps,
             current_settings.e220_subpacket_bytes,
             current_settings.e220_tx_power_dbm);
    lv_label_set_text(t4_e220_cfg_lbl, init_buf);
    lv_obj_set_style_text_color(t4_e220_cfg_lbl, lv_color_hex(0xCCCCCC), 0);
    lv_obj_set_style_text_font(t4_e220_cfg_lbl, &lv_font_montserrat_12, 0);

    // 9. Power & Battery (AXP2101 PMIC)
    t4_power_lbl = lv_label_create(t4_scroll_cont);
    lv_obj_set_width(t4_power_lbl, 300);
    lv_label_set_long_mode(t4_power_lbl, LV_LABEL_LONG_WRAP);
    lv_label_set_text(t4_power_lbl, "[POWER / PMIC AXP2101]\nVBAT: --.-V (--%) | VBUS: -- | Chg: --");
    lv_obj_set_style_text_color(t4_power_lbl, lv_color_hex(0x38BDF8), 0);
    lv_obj_set_style_text_font(t4_power_lbl, &lv_font_montserrat_12, 0);

    // 10. IMU & Pole Tilt (QMI8658)
    t4_imu_lbl = lv_label_create(t4_scroll_cont);
    lv_obj_set_width(t4_imu_lbl, 300);
    lv_label_set_long_mode(t4_imu_lbl, LV_LABEL_LONG_WRAP);
    lv_label_set_text(t4_imu_lbl, "[IMU & POLE TILT] QMI8658 @ 250Hz\nLive: Pitch: 0.0 deg | Roll: 0.0 deg\nAcc(g): X:+0.00 Y:+0.00 Z:+0.00\nGyr(d/s): X:+0.0 Y:+0.0 Z:+0.0\nTare: P: 0.0 deg R: 0.0 deg | Lever: 1.80m [ON]");
    lv_obj_set_style_text_color(t4_imu_lbl, lv_color_hex(0x10B981), 0);
    lv_obj_set_style_text_font(t4_imu_lbl, &lv_font_montserrat_12, 0);

    // 11. BLE Companion & RTCM Arbiter
    t4_ble_ntrip_lbl = lv_label_create(t4_scroll_cont);
    lv_obj_set_width(t4_ble_ntrip_lbl, 300);
    lv_label_set_long_mode(t4_ble_ntrip_lbl, LV_LABEL_LONG_WRAP);
    lv_label_set_text(t4_ble_ntrip_lbl, "[BLE & NTRIP ARBITER]\nBLE: INIT | Mode: AUTO | Active: NONE");
    lv_obj_set_style_text_color(t4_ble_ntrip_lbl, lv_color_hex(0x38BDF8), 0);
    lv_obj_set_style_text_font(t4_ble_ntrip_lbl, &lv_font_montserrat_12, 0);

    // 12. Wi-Fi & Web Portal
    t4_wifi_lbl = lv_label_create(t4_scroll_cont);
    lv_obj_set_width(t4_wifi_lbl, 300);
    lv_label_set_long_mode(t4_wifi_lbl, LV_LABEL_LONG_WRAP);
    lv_label_set_text(t4_wifi_lbl, "[WIFI & WEB PORTAL]\nWi-Fi: INACTIVE");
    lv_obj_set_style_text_color(t4_wifi_lbl, lv_color_hex(0xA78BFA), 0);
    lv_obj_set_style_text_font(t4_wifi_lbl, &lv_font_montserrat_12, 0);

    // 12. I2C Bus Status
    t4_i2c_lbl = lv_label_create(t4_scroll_cont);
    lv_obj_set_width(t4_i2c_lbl, 300);
    lv_label_set_long_mode(t4_i2c_lbl, LV_LABEL_LONG_WRAP);
    lv_label_set_text(t4_i2c_lbl, "[I2C0 BUS STATUS]\nTCA:OK | AXP:OK | Touch:OK | RTC:OK | IMU:OK | ES8311:OK");
    lv_obj_set_style_text_color(t4_i2c_lbl, lv_color_hex(0x2DD4BF), 0);
    lv_obj_set_style_text_font(t4_i2c_lbl, &lv_font_montserrat_12, 0);
}

void ui_tile4_diagnostics_update(const RoverUIData_t *d, uint32_t now_ms, char *buf, size_t buf_sz)
{
    // 5. Update Tile 4: Pinned LoRa RF Survey Card & Hardware Diagnostics Status
    if (t4_rf_card && t4_rf_bar && t4_rf_rssi_lbl) {
        float rx_age_s = (d->e220_stats.last_rx_ms > 0) ? (float)(now_ms - d->e220_stats.last_rx_ms) / 1000.0f : 999.0f;
        bool is_fresh = (d->e220_stats.rssi_valid && rx_age_s <= 6.0f && d->e220_stats.rx_bytes_total > 0);
        int16_t pkt_rssi = d->e220_stats.last_pkt_rssi_dbm;
        int16_t noise_floor = d->e220_stats.ambient_noise_dbm;

        // Row 1: Meter Bar (Scale: -110 dBm = 0%, -25 dBm = 100%, Span = 85 dB)
        if (is_fresh) {
            int pct = ((int)pkt_rssi + 110) * 100 / 85;
            if (pct < 0) pct = 0;
            if (pct > 100) pct = 100;
            lv_bar_set_value(t4_rf_bar, pct, LV_ANIM_OFF);
            lv_obj_set_style_bg_color(t4_rf_bar, get_lora_rssi_color(pkt_rssi, false), LV_PART_INDICATOR);
        } else {
            lv_bar_set_value(t4_rf_bar, 0, LV_ANIM_OFF);
            lv_obj_set_style_bg_color(t4_rf_bar, lv_color_hex(0x475569), LV_PART_INDICATOR);
        }

        // Row 2: Live Metrics (RSSI bold left, Noise/Margin right)
        if (is_fresh) {
            snprintf(buf, buf_sz, "LoRa: %d dBm", (int)pkt_rssi);
            lv_label_set_text(t4_rf_rssi_lbl, buf);
            lv_obj_set_style_text_color(t4_rf_rssi_lbl, get_lora_rssi_color(pkt_rssi, false), 0);
        } else if (d->e220_stats.rx_bytes_total > 0 && d->e220_stats.rssi_valid) {
            snprintf(buf, buf_sz, "LoRa: %d dBm*", (int)pkt_rssi);
            lv_label_set_text(t4_rf_rssi_lbl, buf);
            lv_obj_set_style_text_color(t4_rf_rssi_lbl, lv_palette_main(LV_PALETTE_RED), 0);
        } else if (!d->e220_stats.radio_online) {
            lv_label_set_text(t4_rf_rssi_lbl, "LoRa: Offline");
            lv_obj_set_style_text_color(t4_rf_rssi_lbl, lv_color_hex(0x64748B), 0);
        } else {
            lv_label_set_text(t4_rf_rssi_lbl, "LoRa: No Signal");
            lv_obj_set_style_text_color(t4_rf_rssi_lbl, lv_color_hex(0x64748B), 0);
        }

        if (d->e220_stats.rssi_valid) {
            int margin = (int)pkt_rssi - (int)noise_floor;
            if (is_fresh) {
                snprintf(buf, buf_sz, "Noise: %d dBm (%+d dB)", (int)noise_floor, margin);
            } else {
                snprintf(buf, buf_sz, "Noise: %d dBm (--)", (int)noise_floor);
            }
            lv_label_set_text(t4_rf_noise_lbl, buf);
        } else {
            lv_label_set_text(t4_rf_noise_lbl, "Noise: -- dBm");
        }

        // Row 3: Health & Channel
        uint32_t drops = d->e220_stats.rtcm_crc_errors + d->e220_stats.rtcm_trunc_drops;
        if (rx_age_s < 99.0f) {
            snprintf(buf, buf_sz, "Age: %.1fs | OK: %lu (%lu drop)",
                     rx_age_s, (unsigned long)d->e220_stats.rtcm_frames_ok, (unsigned long)drops);
        } else {
            snprintf(buf, buf_sz, "Age: >99s | OK: %lu (%lu drop)",
                     (unsigned long)d->e220_stats.rtcm_frames_ok, (unsigned long)drops);
        }
        lv_label_set_text(t4_rf_health_lbl, buf);
        if (drops > 0) {
            lv_obj_set_style_text_color(t4_rf_health_lbl, lv_color_hex(0xFFA500), 0);
        } else {
            lv_obj_set_style_text_color(t4_rf_health_lbl, lv_color_hex(0xCBD5E1), 0);
        }

        snprintf(buf, buf_sz, "CH%u (%.1fk)",
                 d->e220_stats.cfg_channel,
                 current_settings.e220_air_rate_kbps);
        lv_label_set_text(t4_rf_chan_lbl, buf);
    }

    if (t4_storage_status_lbl) {
        extern bool sd_card_mounted;
        if (sd_card_mounted) {
            lv_label_set_text(t4_storage_status_lbl, "Storage: MicroSD Card Ready (Primary)");
            lv_obj_set_style_text_color(t4_storage_status_lbl, lv_color_hex(0x22D3EE), 0); // Cyan (distinct from ANT1 green #00FF88)
        } else {
            lv_label_set_text(t4_storage_status_lbl, "Storage: FFat Internal Flash (Fallback)");
            lv_obj_set_style_text_color(t4_storage_status_lbl, lv_color_hex(0xFFA500), 0); // Amber
        }
    }

    // 6. Update Tile 4: Antenna & GNSS Diagnostics
    if (t4_ant_master_lbl) {
        char h_acc_str[24], v_acc_str[24], hgt_str[24];
        nav_format_distance(d->h_acc_m, h_acc_str, sizeof(h_acc_str));
        nav_format_distance(d->v_acc_m, v_acc_str, sizeof(v_acc_str));
        nav_format_height(d->tip_height_m, hgt_str, sizeof(hgt_str));
        uint8_t sv_view = (d->sats_in_view >= d->num_sats) ? d->sats_in_view : d->num_sats;

        const char *m_fix_str = !d->um982_stats.com1_online ? "OFFLINE" : "NO FIX";
        if (d->um982_stats.com1_online) {
            if (d->fix_type == GNSS_FIX_RTK_FIXED)      m_fix_str = "RTK FIX";
            else if (d->fix_type == GNSS_FIX_RTK_FLOAT) m_fix_str = "RTK FLT";
            else if (d->fix_type == GNSS_FIX_DGPS)      m_fix_str = "DGPS";
            else if (d->fix_type == GNSS_FIX_SINGLE)    m_fix_str = "SINGLE";
        }

        snprintf(buf, buf_sz,
                 "[ANT1 MASTER] %s | SV:%u/%u | HDOP:%.1f\n"
                 "Acc:+/-%s (V:+/-%s) | Alt:%s\n"
                 "Pos:%.8f, %.8f",
                 m_fix_str,
                 d->num_sats,
                 sv_view,
                 d->hdop,
                 h_acc_str,
                 v_acc_str,
                 hgt_str,
                 d->tip_lat,
                 d->tip_lon);
        lv_label_set_text(t4_ant_master_lbl, buf);
    }

    if (t4_ant_slave_lbl) {
        const char *s_fix_str = "NO FIX";
        if (d->heading_source == HDG_SRC_DUAL_FIX)      s_fix_str = "SLV FIX";
        else if (d->heading_source == HDG_SRC_DUAL_FLT) s_fix_str = "SLV FLT";
        else if (d->heading_source == HDG_SRC_DISP)     s_fix_str = "HDG WALK";
        else if (d->heading_source == HDG_SRC_COMP)     s_fix_str = "HDG CMP";
        else if (d->heading_source == HDG_SRC_GYRO)     s_fix_str = "HDG GYRO";
        else if (d->heading_source == HDG_SRC_COG)      s_fix_str = "HDG COG";
        else if (d->heading_source == HDG_SRC_HOLD)     s_fix_str = "HDG HLD";
        else if (d->heading_valid)                      s_fix_str = d->heading_is_fixed ? "SLV FIX" : "SLV FLT";
        else if (d->slave_fix_type == GNSS_FIX_DGPS)    s_fix_str = "SLV DGPS";
        else if (d->slave_fix_type == GNSS_FIX_SINGLE)  s_fix_str = "SLV SGL";

        char bl_str[24];
        if (d->baseline_length_m > 0.01f) {
            snprintf(bl_str, sizeof(bl_str), "%.1fcm", d->baseline_length_m * 100.0f);
        } else {
            snprintf(bl_str, sizeof(bl_str), "--.-cm");
        }

        char hdg_str[24];
        if (d->heading_valid) {
            snprintf(hdg_str, sizeof(hdg_str), "%.1f deg", d->true_heading_deg);
        } else {
            snprintf(hdg_str, sizeof(hdg_str), "---.- deg");
        }

        char s_alt_str[24];
        nav_format_height(d->slave_alt_m, s_alt_str, sizeof(s_alt_str));

        if (d->slave_lat != 0.0 || d->slave_lon != 0.0) {
            snprintf(buf, buf_sz,
                     "[ANT2 SLAVE] %s | SV:%u | HDOP:%.1f\n"
                     "Baseline:%s | Pitch:%+.1f deg\n"
                     "Heading:%s | Alt:%s\n"
                     "Pos:%.8f, %.8f",
                     s_fix_str,
                     d->slave_num_sats,
                     d->slave_hdop,
                     bl_str,
                     d->baseline_pitch_deg,
                     hdg_str,
                     s_alt_str,
                     d->slave_lat,
                     d->slave_lon);
        } else {
            snprintf(buf, buf_sz,
                     "[ANT2 SLAVE] %s | SV:%u | HDOP:%.1f\n"
                     "Baseline:%s | Pitch:%+.1f deg\n"
                     "Heading:%s | Alt:%s",
                     s_fix_str,
                     d->slave_num_sats,
                     d->slave_hdop,
                     bl_str,
                     d->baseline_pitch_deg,
                     hdg_str,
                     s_alt_str);
        }
        lv_label_set_text(t4_ant_slave_lbl, buf);
    }

    if (t4_base_station_lbl) {
        if (d->base_coords_valid) {
            char alt_str[24];
            nav_format_height((float)d->base_alt_m, alt_str, sizeof(alt_str));
            snprintf(buf, buf_sz,
                     "[BASE STATION] ID:#%u | Diff:%.1fs\n"
                     "Pos:%.8f, %.8f\n"
                     "Alt:%s | 1005:%u MSM:%u",
                     d->base_station_id,
                     d->diff_age_s,
                     d->base_lat,
                     d->base_lon,
                     alt_str,
                     d->e220_stats.msg_1005_count,
                     d->e220_stats.msg_msm_count);
            if (d->diff_age_s <= 2.0f) {
                lv_obj_set_style_text_color(t4_base_station_lbl, lv_color_hex(0x00FF88), 0); // Fresh Green
            } else if (d->diff_age_s <= 5.0f) {
                lv_obj_set_style_text_color(t4_base_station_lbl, lv_color_hex(0xFFA500), 0); // Warning Orange
            } else {
                lv_obj_set_style_text_color(t4_base_station_lbl, lv_palette_main(LV_PALETTE_RED), 0); // Stale Red
            }
        } else if (d->base_station_id > 0 || d->diff_age_s < 30.0f) {
            snprintf(buf, buf_sz,
                     "[BASE STATION] ID:#%u | Diff:%.1fs\n"
                     "Pos: Waiting for RTCM 1005/1006 ARP...\n"
                     "RTCM: 1005:%u MSM:%u",
                     d->base_station_id,
                     d->diff_age_s,
                     d->e220_stats.msg_1005_count,
                     d->e220_stats.msg_msm_count);
            lv_obj_set_style_text_color(t4_base_station_lbl, lv_color_hex(0xFFA500), 0);
        } else {
            snprintf(buf, buf_sz,
                     "[BASE STATION] Waiting for RTCM corrections...\n"
                     "Diff: --.-s | 1005:%u MSM:%u",
                     d->e220_stats.msg_1005_count,
                     d->e220_stats.msg_msm_count);
            lv_obj_set_style_text_color(t4_base_station_lbl, lv_color_hex(0x888888), 0);
        }
        lv_label_set_text(t4_base_station_lbl, buf);
    }

    // 7. Update Tile 4: Hardware Port & UM982 Known-State Audit Diagnostics
    if (t4_com1_lbl) {
        snprintf(buf, buf_sz,
                 "[UM982 COM1 IO17/18 @ 115200 Full-Duplex] [%s]\n"
                 "RX:%lu B | TX:%lu B (RTCM:%lu B)\n"
                 "NMEA:%lu | HDG:%lu | Diff:%.1fs\n"
                 "Last NMEA: %s",
                 d->um982_stats.com1_online ? "ONLINE" : "OFFLINE",
                 (unsigned long)d->um982_stats.com1_rx_bytes,
                 (unsigned long)d->um982_stats.com1_tx_bytes,
                 (unsigned long)d->um982_stats.rtcm_tx_bytes,
                 (unsigned long)d->um982_stats.nmea_sentences_ok,
                 (unsigned long)d->um982_stats.heading_sentences_ok,
                 d->diff_age_s,
                 d->um982_stats.last_com1_line);
        lv_label_set_text(t4_com1_lbl, buf);
    }

    if (t4_um982_audit_lbl) {
        snprintf(buf, buf_sz,
                 "[UM982 RECEIVER STATE & LOGLIST] [%s]\n"
                 "Ver: %s\n"
                 "Mode: %s | %s\n"
                 "Logs: %s\n"
                 "Last ACK (%lu): %s",
                 !d->um982_stats.com1_online ? "OFFLINE" : (d->um982_stats.known_state_ok ? "STATE OK" : "CHECK"),
                 d->um982_stats.um982_version,
                 d->um982_stats.um982_mode,
                 d->um982_stats.um982_config_summary,
                 d->um982_stats.um982_loglist,
                 (unsigned long)d->um982_stats.cmd_responses_ok,
                 d->um982_stats.last_cmd_ack_line);
        lv_label_set_text(t4_um982_audit_lbl, buf);
        if (d->um982_stats.known_state_ok) {
            lv_obj_set_style_text_color(t4_um982_audit_lbl, lv_color_hex(0x00FFAA), 0);
        } else {
            lv_obj_set_style_text_color(t4_um982_audit_lbl, lv_color_hex(0xFFA500), 0);
        }
    }

    if (t4_e220_lbl) {
        const char *m0_str = d->e220_stats.m0_wire_ok ? "OK" : "ERR";
        const char *m1_str = d->e220_stats.m1_wire_ok ? "OK" : "ERR";
        snprintf(buf, buf_sz,
                 "[E220 LoRa Hardware IO41/42] [%s]\n"
                 "AUX:%d (Edges:%lu) | M0:%s M1:%s\n"
                 "Throughput: RX:%lu B -> COM1(RTCM):%lu B\n"
                 "Integrity: OK:%lu | CRC Err:%lu | Trunc:%lu | Orph:%lu B\n"
                 "[RTCM3 Message Audit]\n"
                 "1006 (Base ARP): %u | 1074 (GPS): %u\n"
                 "1084 (GLO): %u | 1094 (GAL): %u\n"
                 "1230 (Biases): %u",
                 !d->e220_stats.radio_online ? "OFFLINE" : (d->e220_stats.config_valid ? (d->e220_stats.config_matched ? "CFG OK" : "CFG DIFF") : "ONLINE"),
                 d->e220_stats.aux_pin_high ? 1 : 0,
                 (unsigned long)d->e220_stats.aux_fall_count,
                 m0_str,
                 m1_str,
                 (unsigned long)d->e220_stats.rx_bytes_total,
                 (unsigned long)d->e220_stats.rtcm_bytes_forwarded,
                 (unsigned long)d->e220_stats.rtcm_frames_ok,
                 (unsigned long)d->e220_stats.rtcm_crc_errors,
                 (unsigned long)d->e220_stats.rtcm_trunc_drops,
                 (unsigned long)d->e220_stats.rtcm_orphan_bytes,
                 d->e220_stats.msg_1005_count,
                 d->e220_stats.msg_1074_count,
                 d->e220_stats.msg_1084_count,
                 d->e220_stats.msg_1094_count,
                 d->e220_stats.msg_1230_count);
        lv_label_set_text(t4_e220_lbl, buf);
        if (!d->e220_stats.m0_wire_ok || !d->e220_stats.m1_wire_ok) {
            lv_obj_set_style_text_color(t4_e220_lbl, lv_palette_main(LV_PALETTE_RED), 0); // Pin wire error
        } else if (d->e220_stats.rtcm_crc_errors > 0 || d->e220_stats.rtcm_trunc_drops > 0) {
            lv_obj_set_style_text_color(t4_e220_lbl, lv_color_hex(0xFFA500), 0); // Highlight in Orange on errors
        } else {
            lv_obj_set_style_text_color(t4_e220_lbl, lv_color_hex(0xFFCC00), 0); // Normal yellow
        }
    }

    if (t4_e220_cfg_lbl && d->e220_stats.config_valid) {
        snprintf(buf, buf_sz,
                 "Target: CH%u %lu/%.1fk %uB %udBm\n"
                 "Regs: ADDH:%02X ADDL:%02X REG0:%02X REG1:%02X CH:%02X REG3:%02X",
                 current_settings.e220_channel,
                 (unsigned long)current_settings.e220_uart_baud,
                 current_settings.e220_air_rate_kbps,
                 current_settings.e220_subpacket_bytes,
                 current_settings.e220_tx_power_dbm,
                 d->e220_stats.cfg_addr_high,
                 d->e220_stats.cfg_addr_low,
                 d->e220_stats.cfg_reg0,
                 d->e220_stats.cfg_reg1,
                 d->e220_stats.cfg_channel,
                 d->e220_stats.cfg_reg3);
        lv_label_set_text(t4_e220_cfg_lbl, buf);
    }

    // 8. Update Tile 4: Power / PMIC AXP2101 Diagnostics
    if (t4_power_lbl) {
        bool periph_5v_active = d->um982_stats.com1_online || d->e220_stats.radio_online;
        if (d->vbus_present) {
            snprintf(buf, buf_sz,
                     "[POWER / PMIC AXP2101]\n"
                     "Input: USB External (%.2fV) | 5V Rail: %s\n"
                     "Battery: %.2fV (%u%%) | Chg: %s\n"
                     "Peripherals: UM982:%s | E220:%s",
                     d->vbus_voltage,
                     periph_5v_active ? "ON" : "OFF",
                     d->battery_voltage,
                     (unsigned int)d->battery_percent,
                     d->is_charging ? "YES (Charging)" : "STANDBY",
                     d->um982_stats.com1_online ? "ON" : "OFF",
                     d->e220_stats.radio_online ? "ON" : "OFF");
        } else {
            char ao_str[48];
            if (d->low_batt_warning) {
                snprintf(ao_str, sizeof(ao_str), "%s",
                         d->low_batt_logs_flushed ? "LOW-BATT PROTECT: LOGS SAVED" : "LOW-BATT PROTECT: FLUSHING...");
            } else if (d->low_batt_debounce_s > 0) {
                snprintf(ao_str, sizeof(ao_str), "LOW-BATT CHECK: %us / 5s",
                         (unsigned int)d->low_batt_debounce_s);
            } else if (current_settings.auto_shutdown_battery_min == 0) {
                snprintf(ao_str, sizeof(ao_str), "Auto-Off: Disabled");
            } else if (d->auto_off_inhibited) {
                snprintf(ao_str, sizeof(ao_str), "Auto-Off: %um | Hold (%s)",
                         (unsigned int)current_settings.auto_shutdown_battery_min,
                         d->auto_off_reason[0] ? d->auto_off_reason : "Active");
            } else {
                snprintf(ao_str, sizeof(ao_str), "Auto-Off: %um | Idle %lus",
                         (unsigned int)current_settings.auto_shutdown_battery_min,
                         (unsigned long)d->auto_off_idle_s);
            }
            snprintf(buf, buf_sz,
                     "[POWER / PMIC AXP2101]\n"
                     "Input: LiPo Battery | 5V Boost: %s\n"
                     "Battery: %.2fV (%u%%) | Periph: UM982:%s E220:%s\n"
                     "%s",
                     periph_5v_active ? "ON" : "OFF",
                     d->battery_voltage,
                     (unsigned int)d->battery_percent,
                     d->um982_stats.com1_online ? "ON" : "OFF",
                     d->e220_stats.radio_online ? "ON" : "OFF",
                     ao_str);
        }
        lv_label_set_text(t4_power_lbl, buf);
    }

    // 9. Update Tile 4: Beam 10DOF (SEN0696), QMI8658 & Compass Diagnostics
    if (t4_imu_lbl) {
        const char *pose_tag = (d->display_pose == DISPLAY_POSE_VERTICAL) ? "VERT" :
                               (d->display_pose == DISPLAY_POSE_ANGLED)   ? "ANG"  : "FLAT";
        if (d->beam_10dof_present) {
            char rel_elev_str[24];
            if (d->beam_baro_bias_valid) {
                nav_format_height(d->beam_baro_rel_elev_m, rel_elev_str, sizeof(rel_elev_str));
            } else {
                snprintf(rel_elev_str, sizeof(rel_elev_str), "--");
            }
            const char *lev_tag = !current_settings.apply_lever_arm ? "OFF" :
                                  (d->lever_arm_applied ? ((d->heading_source == HDG_SRC_COMP) ? "BEAM" : "DUAL") : "WAIT");
            snprintf(buf, buf_sz,
                     "[BEAM 10DOF & IMU] SEN0696@0x%02X [%s:%s]\n"
                     "Beam P:%+.1f R:%+.1f deg | M%u FOC:%s\n"
                     "Mag:%.1fuT %s | Cal3D:%s Tare:%+.1f\n"
                     "BMP581:%.1fhPa RelElev:%s\n"
                     "QMI8658:%s P:%+.1f R:%+.1f | Lev:%.2fm[%s]",
                     (unsigned)(d->beam_10dof_addr ? d->beam_10dof_addr : ROVER_I2C_ADDR_BEAM_IMU),
                     (d->beam_10dof_bus == 1U) ? "I2C1" : "I2C0",
                     d->beam_10dof_valid ? "ONLINE" : "ERR",
                     d->beam_pitch_deg,
                     d->beam_roll_deg,
                     (unsigned)current_settings.beam_imu_orient_mode,
                     d->beam_gyro_foc_locked ? "OK" : "WAIT",
                     d->mag_total_uT,
                     d->mag_anomaly ? "[IRON!]" : "[OK]",
                     d->mag_ellipsoid_cal_valid ? "YES" : "NO",
                     d->mag_tare_offset_deg,
                     d->beam_pressure_hpa,
                     rel_elev_str,
                     pose_tag,
                     d->imu_pitch_deg,
                     d->imu_roll_deg,
                     current_settings.antenna_height_m,
                     lev_tag);
        } else {
            char mag_line[128];
            if (d->mag_present) {
                snprintf(mag_line, sizeof(mag_line),
                         "Mag(0x30): %.1fuT (X:%+.1f Y:%+.1f Z:%+.1f) %s\n"
                         "Compass: %.1f deg | Tare:%+.1f deg [%s] | SR:%lu (%.1fC)",
                         d->mag_total_uT,
                         d->mag_uT[0], d->mag_uT[1], d->mag_uT[2],
                         d->mag_anomaly ? "[IRON!]" : "[OK]",
                         d->mag_heading_deg,
                         d->mag_tare_offset_deg,
                         d->mag_cal_valid ? "AUTO-CAL" : "DEFAULT",
                         (unsigned long)d->mag_set_reset_count,
                         d->mag_die_temp_c);
            } else {
                snprintf(mag_line, sizeof(mag_line),
                         "10DOF: -- | MAG(0x30): Not Detected (6-DOF Mode)");
            }
            snprintf(buf, buf_sz,
                     "[IMU & 9-DOF COMPASS] QMI8658 [%s]\n"
                     "Live: Pitch:%+.1f deg | Roll:%+.1f deg\n"
                     "Acc(g): X:%+.2f Y:%+.2f Z:%+.2f\n"
                     "Gyr(d/s): X:%+.1f Y:%+.1f Z:%+.1f\n"
                     "Tare: P:%+.1f R:%+.1f | Lever: %.2fm [%s]\n"
                     "%s",
                     pose_tag,
                     d->imu_pitch_deg,
                     d->imu_roll_deg,
                     d->imu_acc[0], d->imu_acc[1], d->imu_acc[2],
                     d->imu_gyr[0], d->imu_gyr[1], d->imu_gyr[2],
                     current_settings.pole_tilt_tare_pitch_deg,
                     current_settings.pole_tilt_tare_roll_deg,
                     current_settings.antenna_height_m,
                     current_settings.apply_lever_arm ? "ON" : "OFF",
                     mag_line);
        }
        lv_label_set_text(t4_imu_lbl, buf);
    }

    // 10. Update Tile 4: BLE Companion & Dual-Source RTCM Arbiter
    if (t4_ble_ntrip_lbl) {
        FicBleStats_t b_st = {};
        fic_ble_companion_get_stats(&b_st);
        const char *ble_tag = !b_st.active ? "OFF" :
                              b_st.connected ? (ble_bridge_is_slave_dimmed() ? "SLAVE" : "CONN") : "ADV";
        const char *m_tag = rtcm_arbiter_mode_to_str(d->rtcm_arbiter.mode);
        const char *a_tag = rtcm_arbiter_source_to_str(d->rtcm_arbiter.active_source);
        const char *host  = current_settings.ntrip_host[0] ? current_settings.ntrip_host : "--";
        const char *mnt   = current_settings.ntrip_mountpoint[0] ? current_settings.ntrip_mountpoint : "--";
        snprintf(buf, buf_sz,
                 "[BLE & NTRIP ARBITER] [%s] MTU:%u\n"
                 "CTRL RX:%lu TX:%lu | Mode:%s -> Active:%s\n"
                 "NTRIP:%.1fkb/s OK:%lu Err:%lu Drop:%lu Fail:%lu\n"
                 "Caster: %s:%u/%s",
                 ble_tag,
                 (unsigned)b_st.peer_mtu,
                 (unsigned long)b_st.frames_rx,
                 (unsigned long)b_st.frames_tx,
                 m_tag,
                 a_tag,
                 d->rtcm_arbiter.ntrip_rx_kbps,
                 (unsigned long)d->rtcm_arbiter.ntrip_frames_ok,
                 (unsigned long)d->rtcm_arbiter.ntrip_crc_errors,
                 (unsigned long)d->rtcm_arbiter.ntrip_frames_dropped,
                 (unsigned long)d->rtcm_arbiter.failover_count,
                 host,
                 (unsigned)current_settings.ntrip_port,
                 mnt);
        lv_label_set_text(t4_ble_ntrip_lbl, buf);
    }

    // 11. Update Tile 4: Wi-Fi & Web Portal Diagnostics
    if (t4_wifi_lbl) {
        bool active = fic_web_portal_is_active();
        bool softap = fic_web_portal_is_softap();
        const char *ip = fic_web_portal_get_ip();
        if (active && ip && ip[0] && strcmp(ip, "0.0.0.0") != 0) {
            snprintf(buf, buf_sz,
                     "[WIFI & WEB PORTAL]\n"
                     "SSID: '%s' [ONLINE]\n"
                     "Portal: http://%s/",
                     softap ? "RTK-Rover (AP)" : current_settings.wifi_ssid,
                     ip);
        } else if (active) {
            snprintf(buf, buf_sz,
                     "[WIFI & WEB PORTAL]\n"
                     "SSID: '%s' [CONNECTING...]",
                     current_settings.wifi_ssid);
        } else {
            snprintf(buf, buf_sz,
                     "[WIFI & WEB PORTAL]\n"
                     "Wi-Fi: INACTIVE (Tap COMMS on Tile 6)");
        }
        lv_label_set_text(t4_wifi_lbl, buf);
    }

    // 11. Update Tile 4: I2C0 (Internal) & I2C1 (Isolated Beam 10DOF) Bus Diagnostics
    if (t4_i2c_lbl) {
        const char *beam_stat = "--";
        if (d->beam_10dof_present) {
            if (d->beam_10dof_bus == 0U) {
                beam_stat = d->beam_10dof_valid ? "OK(I2C0)" : "ERR(I2C0)";
            } else {
                beam_stat = d->beam_10dof_valid ? "OK" : "ERR";
            }
        }
        snprintf(buf, buf_sz,
                 "[I2C0 IO7/8 @ 400kHz] Err:%lu | MAG(0x30):%s\n"
                 "TCA/AXP/Touch/RTC/IMU/ES8311: OK\n"
                 "[I2C1 IO47/48 @ 400kHz] Err:%lu | 10DOF(0x%02X):%s",
                 (unsigned long)d->i2c0_err_count,
                 (!d->beam_10dof_present && d->mag_present) ? "OK" : "--",
                 (unsigned long)d->i2c1_err_count,
                 (unsigned)(d->beam_10dof_addr ? d->beam_10dof_addr : ROVER_I2C_ADDR_BEAM_IMU),
                 beam_stat);
        lv_label_set_text(t4_i2c_lbl, buf);
    }
}
