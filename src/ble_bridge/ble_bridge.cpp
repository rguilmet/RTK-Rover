#include "ble_bridge.h"
#include "../../user_config.h"
#include <Field_Instrument_Core.h>
#include "../settings_manager/settings_manager.h"
#include "../waypoint_manager/waypoint_manager.h"
#include "../e220_driver/e220_driver.h"
#include "../rover_power/rover_power.h"
#include "../rover_sensors/rover_sensors.h"
#include "../rover_audio/rover_audio.h"
#include "../lvgl_port/lvgl_port.h"
#include "../storage/route_logger.h"
#include "../storage/serial_logger.h"

#include <SD.h>
#include <SD_MMC.h>
#include <FFat.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

extern bool sd_card_mounted;
extern bool sd_card_use_spi;
extern SemaphoreHandle_t fs_mux;

extern "C" void set_backlight_pwm(uint8_t brightness_pct);
extern "C" void set_rover_brightness(uint8_t brightness_pct);
extern "C" int rover_printf(const char *format, ...);

typedef struct {
    bool          valid;
    double        tip_lat;
    double        tip_lon;
    double        tip_height_m;
    double        base_lat;
    double        base_lon;
    uint16_t      base_id;
    bool          base_valid;
    float         baro_rel_m;
    GnssFixType_t fix_type;
    GnssFixType_t slave_fix_type;
    HeadingSource_t hdg_src;
    bool          hdg_valid;
    uint8_t       sv1;
    uint8_t       sv2;
    float         true_hdg_deg;
    float         bl_len_m;
    float         bl_pitch_deg;
    float         beam_pitch_deg;
    float         beam_roll_deg;
    bool          mag_anomaly;
    bool          lever_arm_applied;
    float         h_acc_m;
    float         v_acc_m;
    float         diff_age_s;
    int16_t       rssi_dbm;
    int16_t       noise_dbm;
    float         air_rate_kbps;
    int16_t       active_wp_idx;
    bool          survey_active;
    uint8_t       survey_pct;
} RoverBleTelemCache_t;

static portMUX_TYPE              s_telem_mux     = portMUX_INITIALIZER_UNLOCKED;
static RoverBleTelemCache_t      s_telem         = {};
static volatile int8_t           s_pending_cmd   = 0;
static volatile bool             s_slave_dimmed  = false;
static volatile bool             s_ui_dirty      = false;
static ble_bridge_stream_sink_cb_t s_stream_sink = nullptr;

static void write_u16_le(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8U) & 0xFFU);
}

static void write_i16_le(uint8_t *p, int16_t v)
{
    write_u16_le(p, (uint16_t)v);
}

static void write_u32_le(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v & 0xFFU);
    p[1] = (uint8_t)((v >> 8U) & 0xFFU);
    p[2] = (uint8_t)((v >> 16U) & 0xFFU);
    p[3] = (uint8_t)((v >> 24U) & 0xFFU);
}

static void write_i32_le(uint8_t *p, int32_t v)
{
    write_u32_le(p, (uint32_t)v);
}

static void write_f64_le(uint8_t *p, double v)
{
    uint64_t raw = 0ULL;
    memcpy(&raw, &v, sizeof(raw));
    for (uint8_t i = 0U; i < 8U; i++) {
        p[i] = (uint8_t)((raw >> (8U * i)) & 0xFFU);
    }
}

static int8_t clamp_i8_dbm(int16_t val)
{
    if (val < -127) return -127;
    if (val > 0)    return 0;
    return (int8_t)val;
}

extern "C" bool rover_is_storage_write_locked(char *reason_out, size_t reason_sz)
{
    if (route_logger_is_active()) {
        if (reason_out && reason_sz > 0U) {
            snprintf(reason_out, reason_sz, "REC ON (route logger active)");
        }
        return true;
    }
    if (waypoint_is_survey_in_progress()) {
        if (reason_out && reason_sz > 0U) {
            snprintf(reason_out, reason_sz, "Survey averaging active");
        }
        return true;
    }
    if (rover_sensors_is_mag_cal_collecting()) {
        if (reason_out && reason_sz > 0U) {
            snprintf(reason_out, reason_sz, "Mag calibration active");
        }
        return true;
    }
    return false;
}

static void rover_ble_snapshot_cb(JsonObject settings_out, JsonObject runtime_out, const char *section)
{
    const bool want_settings = (!section || strcmp(section, "settings") == 0 || strcmp(section, "all") == 0);
    const bool want_runtime  = (!section || strcmp(section, "runtime") == 0 || strcmp(section, "all") == 0);

    SystemSettings snap;
    settings_get_snapshot(&snap);

    if (want_settings) {
        settings_out["antenna_height_m"]   = snap.antenna_height_m;
        settings_out["antenna_spacing_m"]  = snap.antenna_spacing_m;
        settings_out["apply_lever_arm"]    = snap.apply_lever_arm;
        settings_out["apply_lever_arm_on_calibrated_beam"] = snap.apply_lever_arm_on_calibrated_beam;
        settings_out["rtcm_source_mode"]   = snap.rtcm_source_mode;
        settings_out["lora_air_rate_bps"]  = (uint32_t)lroundf(e220_get_active_air_rate() * 1000.0f);
        settings_out["use_imperial_units"] = snap.use_imperial_units;
        settings_out["brightness"]         = snap.brightness;
        settings_out["volume"]             = snap.volume;
        settings_out["is_muted"]           = snap.is_muted;
        settings_out["audio_nav_mode"]     = snap.audio_nav_mode;
        settings_out["survey_avg_seconds"] = snap.survey_avg_seconds;
        settings_out["survey_max_tilt_deg"] = snap.survey_max_tilt_deg;
        settings_out["line_log_max_m"]     = snap.line_log_max_m;
        settings_out["line_audio_fade_m"]  = snap.line_audio_fade_m;
        settings_out["ntrip_host"]         = snap.ntrip_host;
        settings_out["ntrip_host_ip"]      = snap.ntrip_host_ip;
        settings_out["ntrip_port"]         = snap.ntrip_port;
        settings_out["ntrip_mountpoint"]   = snap.ntrip_mountpoint;
        settings_out["ntrip_user_email"]   = snap.ntrip_user_email;
    }

    if (want_runtime) {
        runtime_out["active_tile"]   = rover_get_active_tile_index();
        runtime_out["tile0_mode"]    = rover_get_tile0_nav_mode();
        runtime_out["rec_active"]    = route_logger_is_active();
        runtime_out["survey_active"] = waypoint_is_survey_in_progress();
        runtime_out["wp_idx"]        = g_active_waypoint_idx;
        runtime_out["wp_count"]      = g_waypoint_count;
        runtime_out["wp_file"]       = snap.active_waypoint_file;
        runtime_out["sd_mounted"]    = sd_card_mounted;
    }
}

static bool rover_ble_config_update_cb(JsonObjectConst settings_in,
                                       JsonObject applied_out,
                                       bool persist,
                                       char *err_reason,
                                       size_t err_reason_sz)
{
    bool changed = false;
    bool req_rate = false;
    float req_rate_kbps = 0.0f;
    bool chg_bright = false;
    uint8_t new_bright = 80U;
    bool chg_vol = false;
    uint8_t new_vol = 40U;

    if (!settings_lock(200U)) {
        snprintf(err_reason, err_reason_sz, "Settings lock timeout");
        return false;
    }

    if (settings_in["antenna_height_m"].is<float>()) {
        float v = settings_in["antenna_height_m"].as<float>();
        if (isfinite(v) && v >= 0.2f && v <= 5.0f) {
            current_settings.antenna_height_m = v;
            applied_out["antenna_height_m"] = v;
            changed = true;
        }
    }
    if (settings_in["antenna_spacing_m"].is<float>()) {
        float v = settings_in["antenna_spacing_m"].as<float>();
        if (isfinite(v) && v >= 0.2f && v <= 3.0f) {
            current_settings.antenna_spacing_m = v;
            applied_out["antenna_spacing_m"] = v;
            changed = true;
        }
    }
    if (settings_in["apply_lever_arm"].is<bool>()) {
        current_settings.apply_lever_arm = settings_in["apply_lever_arm"].as<bool>();
        applied_out["apply_lever_arm"] = current_settings.apply_lever_arm;
        changed = true;
    }
    if (settings_in["apply_lever_arm_on_calibrated_beam"].is<bool>()) {
        current_settings.apply_lever_arm_on_calibrated_beam =
            settings_in["apply_lever_arm_on_calibrated_beam"].as<bool>();
        applied_out["apply_lever_arm_on_calibrated_beam"] =
            current_settings.apply_lever_arm_on_calibrated_beam;
        changed = true;
    }
    if (settings_in["use_imperial_units"].is<bool>()) {
        current_settings.use_imperial_units = settings_in["use_imperial_units"].as<bool>();
        applied_out["use_imperial_units"] = current_settings.use_imperial_units;
        changed = true;
    }
    if (settings_in["brightness"].is<int>()) {
        int b = settings_in["brightness"].as<int>();
        if (b < 10) b = 10;
        if (b > 100) b = 100;
        new_bright = (uint8_t)b;
        chg_bright = true;
        applied_out["brightness"] = new_bright;
        changed = true;
    }
    if (settings_in["volume"].is<int>()) {
        int v = settings_in["volume"].as<int>();
        if (v < 0) v = 0;
        if (v > 100) v = 100;
        new_vol = (uint8_t)v;
        chg_vol = true;
        applied_out["volume"] = new_vol;
        changed = true;
    }
    if (settings_in["audio_nav_mode"].is<int>()) {
        int m = settings_in["audio_nav_mode"].as<int>();
        if (m >= AUDIO_NAV_BEEPS && m <= AUDIO_NAV_MUTE) {
            current_settings.audio_nav_mode = (uint8_t)m;
            applied_out["audio_nav_mode"] = current_settings.audio_nav_mode;
            changed = true;
        }
    }
    if (settings_in["survey_avg_seconds"].is<int>()) {
        int s = settings_in["survey_avg_seconds"].as<int>();
        if (s == 1 || s == 5 || s == 10 || s == 30) {
            current_settings.survey_avg_seconds = (uint8_t)s;
            applied_out["survey_avg_seconds"] = current_settings.survey_avg_seconds;
            changed = true;
        }
    }
    if (settings_in["survey_max_tilt_deg"].is<float>()) {
        float t = settings_in["survey_max_tilt_deg"].as<float>();
        if (isfinite(t) && t >= 0.0f && t <= 15.0f) {
            current_settings.survey_max_tilt_deg = t;
            applied_out["survey_max_tilt_deg"] = t;
            changed = true;
        }
    }
    if (settings_in["line_log_max_m"].is<float>()) {
        float l = settings_in["line_log_max_m"].as<float>();
        if (isfinite(l) && l >= 2.0f && l <= 500.0f) {
            current_settings.line_log_max_m = l;
            applied_out["line_log_max_m"] = l;
            changed = true;
        }
    }
    if (settings_in["line_audio_fade_m"].is<float>()) {
        float f = settings_in["line_audio_fade_m"].as<float>();
        if (isfinite(f) && f >= 0.0f && f <= 200.0f) {
            current_settings.line_audio_fade_m = f;
            applied_out["line_audio_fade_m"] = f;
            changed = true;
        }
    }
    if (settings_in["rtcm_source_mode"].is<const char *>()) {
        const char *m = settings_in["rtcm_source_mode"].as<const char *>();
        if (strcmp(m, "AUTO") == 0 || strcmp(m, "LORA") == 0 || strcmp(m, "NTRIP") == 0) {
            snprintf(current_settings.rtcm_source_mode, sizeof(current_settings.rtcm_source_mode), "%s", m);
            applied_out["rtcm_source_mode"] = current_settings.rtcm_source_mode;
            changed = true;
        }
    }
    if (settings_in["lora_air_rate_bps"].is<uint32_t>()) {
        uint32_t bps = settings_in["lora_air_rate_bps"].as<uint32_t>();
        if (bps == 4800U || bps == 9600U || bps == 19200U) {
            req_rate      = true;
            req_rate_kbps = (float)bps / 1000.0f;
            applied_out["lora_air_rate_bps"] = bps;
            changed = true;
        }
    }
    settings_unlock();

    if (chg_bright) {
        if (s_slave_dimmed) {
            current_settings.brightness = new_bright;
        } else {
            set_rover_brightness(new_bright);
        }
    }
    if (chg_vol) {
        set_rover_volume(new_vol);
        if (s_slave_dimmed && fic_ble_companion_is_active()) {
            // Preserve slave audio mute unless follow mode was active
        }
    }
    if (req_rate) {
        (void)e220_request_air_rate_switch(req_rate_kbps);
    }

    if (!changed) {
        snprintf(err_reason, err_reason_sz, "No valid mirrored keys in update");
        return false;
    }

    if (persist) {
        mark_settings_dirty();
    }
    s_ui_dirty = true;
    return true;
}

static void rover_ble_control_mode_cb(FicControlMode_t mode,
                                      uint8_t backlight_pct,
                                      FicLocalAudioMode_t local_audio,
                                      const char *origin)
{
    if (mode == FIC_CTRL_SLAVE_DIMMED) {
        s_slave_dimmed = true;
        set_backlight_pwm(backlight_pct);
        const bool mute = (local_audio == FIC_AUDIO_MUTED);
        fic_audio_beeper_set_muted(mute ? true : (current_settings.is_muted || current_settings.volume == 0U));
    } else {
        s_slave_dimmed = false;
        set_backlight_pwm(current_settings.brightness);
        fic_audio_beeper_set_muted(current_settings.is_muted || current_settings.volume == 0U);
    }
    s_ui_dirty = true;
    rover_printf("[BLE] Control mode -> %s (BL=%u%%, audio=%s, origin=%s)\n",
                 ficp_control_mode_to_str(mode),
                 (unsigned)backlight_pct,
                 ficp_audio_mode_to_str(local_audio),
                 origin ? origin : "?");
}

static void rover_ble_stream_rx_cb(const uint8_t *data, size_t len)
{
    if (s_stream_sink != nullptr && data != nullptr && len > 0U) {
        s_stream_sink(data, len);
    }
}

static size_t rover_ble_telem_pack_cb(uint8_t *out, size_t max_len, uint8_t *msg_id_out)
{
    static const size_t kTelemLen = 83U;
    if (!out || max_len < kTelemLen) {
        return 0U;
    }

    RoverBleTelemCache_t t;
    portENTER_CRITICAL(&s_telem_mux);
    t = s_telem;
    portEXIT_CRITICAL(&s_telem_mux);

    if (!t.valid) {
        return 0U;
    }

    FicAxp2101Status_t pwr = {};
    rover_power_get_snapshot(&pwr);

    uint8_t flags = 0U;
    if (t.mag_anomaly)       flags |= (1U << 0);
    if (t.base_valid)        flags |= (1U << 1);
    if (s_slave_dimmed)      flags |= (1U << 2);
    if (t.lever_arm_applied) flags |= (1U << 3);
    if (t.hdg_valid)         flags |= (1U << 4);

    uint8_t pwr_flags = 0U;
    if (pwr.vbus_good)                    pwr_flags |= (1U << 0);
    if (pwr.is_charging)                  pwr_flags |= (1U << 1);
    if (rover_power_is_low_batt_active()) pwr_flags |= (1U << 2);

    uint8_t sd_flags = 0U;
    if (sd_card_mounted)          sd_flags |= (1U << 0);
    if (route_logger_is_active()) sd_flags |= (1U << 1);
    if (t.survey_active)          sd_flags |= (1U << 2);

    const uint8_t rtcm_mode = (strcmp(current_settings.rtcm_source_mode, "LORA") == 0) ? 1U :
                              (strcmp(current_settings.rtcm_source_mode, "NTRIP") == 0) ? 2U : 0U;
    const uint8_t rate_code = (uint8_t)lroundf(t.air_rate_kbps * 10.0f);

    if (msg_id_out) {
        *msg_id_out = (uint8_t)(FICP_MSG_EXT_BASE + 1U); // 0x81 MSG_ROVER_TELEM
    }

    write_u32_le(&out[0], (uint32_t)millis());
    write_f64_le(&out[4], t.tip_lat);
    write_f64_le(&out[12], t.tip_lon);
    write_f64_le(&out[20], t.base_lat);
    write_f64_le(&out[28], t.base_lon);
    write_u16_le(&out[36], t.base_id);
    write_i32_le(&out[38], (int32_t)llround(t.tip_height_m * 1000.0));
    write_i32_le(&out[42], (int32_t)llroundf(t.baro_rel_m * 1000.0f));
    write_i16_le(&out[46], 0);
    out[48] = (uint8_t)t.fix_type;
    out[49] = (uint8_t)t.slave_fix_type;
    out[50] = (uint8_t)t.hdg_src;
    out[51] = t.sv1;
    out[52] = t.sv2;
    write_u16_le(&out[53], (uint16_t)lroundf(t.true_hdg_deg * 100.0f));
    write_u16_le(&out[55], (uint16_t)lroundf(t.bl_len_m * 1000.0f));
    write_i16_le(&out[57], (int16_t)lroundf(t.bl_pitch_deg * 100.0f));
    write_i16_le(&out[59], (int16_t)lroundf(t.beam_pitch_deg * 100.0f));
    write_i16_le(&out[61], (int16_t)lroundf(t.beam_roll_deg * 100.0f));
    out[63] = flags;
    write_u16_le(&out[64], (uint16_t)lroundf(fminf(fmaxf(t.h_acc_m * 1000.0f, 0.0f), 65535.0f)));
    write_u16_le(&out[66], (uint16_t)lroundf(fminf(fmaxf(t.v_acc_m * 1000.0f, 0.0f), 65535.0f)));
    write_u16_le(&out[68], (uint16_t)lroundf(fminf(fmaxf(t.diff_age_s * 10.0f, 0.0f), 65535.0f)));
    out[70] = 1U; // 1 = LoRa active (Phase 2.4 rtcm_arbiter updates live source)
    out[71] = rtcm_mode;
    out[72] = rate_code;
    out[73] = (uint8_t)clamp_i8_dbm(t.rssi_dbm);
    out[74] = (uint8_t)clamp_i8_dbm(t.noise_dbm);
    write_u16_le(&out[75], (uint16_t)lroundf(fmaxf(pwr.battery_voltage_v * 1000.0f, 0.0f)));
    out[77] = pwr.battery_percent;
    out[78] = pwr_flags;
    write_u16_le(&out[79], (t.active_wp_idx >= 0) ? (uint16_t)t.active_wp_idx : 0xFFFFU);
    out[81] = t.survey_pct;
    out[82] = sd_flags;
    return kTelemLen;
}

static void rover_ble_health_cb(FicBleHealthInfo_t *out)
{
    if (!out) return;
    FicAxp2101Status_t pwr = {};
    rover_power_get_snapshot(&pwr);
    out->vbat_mv   = (uint16_t)lroundf(fmaxf(pwr.battery_voltage_v * 1000.0f, 0.0f));
    out->batt_pct  = pwr.battery_percent;
    out->i2c0_errs = fic_i2c_get_bus_err_count(0U);
    out->i2c1_errs = fic_i2c_get_bus_err_count(1U);
    out->sd_ok     = sd_card_mounted;
}

static bool rover_ble_rtc_set_cb(const FicpUtcCalendar_t *utc)
{
    if (!utc) return false;
    fic_rtc_set_from_utc(utc->year, utc->month, utc->day, utc->hour, utc->minute, utc->second, true);
    return true;
}

static bool rover_ble_ext_rpc_cb(uint8_t msg_id,
                                 uint16_t seq,
                                 uint8_t flags,
                                 const uint8_t *payload,
                                 uint16_t len,
                                 char *ack_json_out,
                                 size_t ack_json_sz,
                                 FicpErrCode_t *err_code_out,
                                 char *err_reason_out,
                                 size_t err_reason_sz)
{
    (void)seq;
    (void)flags;
    (void)payload;
    (void)len;
    (void)ack_json_out;
    (void)ack_json_sz;
    if (err_code_out) *err_code_out = FICP_ERR_UNSUPPORTED_MSG;
    if (err_reason_out && err_reason_sz > 0U) {
        snprintf(err_reason_out, err_reason_sz, "Rover ext 0x%02X scheduled for Phase 2.5", (unsigned)msg_id);
    }
    return false;
}

static void rover_ble_file_committed_cb(const char *fs_key, const char *path)
{
    rover_on_storage_file_updated(path, fs_key);
    s_ui_dirty = true;
}

static void rover_ble_conn_event_cb(bool connected, uint16_t peer_mtu)
{
    (void)peer_mtu;
    if (!connected && s_slave_dimmed) {
        s_slave_dimmed = false;
        set_backlight_pwm(current_settings.brightness);
        fic_audio_beeper_set_muted(current_settings.is_muted || current_settings.volume == 0U);
    }
    s_ui_dirty = true;
}

extern "C" void ble_bridge_init(void)
{
    FicBleCompanionCfg_t cfg = {};
    cfg.device_type        = "rtk_rover";
    cfg.device_name        = "RTK-Rover";
    cfg.fw_version         = ROVER_FIRMWARE_VERSION;
    cfg.board_name         = "waveshare_s3_35b";
    cfg.schema_ver         = ROVER_SETTINGS_SCHEMA_VERSION;
    cfg.auth_mode          = FICP_AUTH_OPEN_GATED;
    cfg.feature_mask       = FICP_FEAT_SETTINGS_SYNC |
                             FICP_FEAT_CONTROL_HANDOFF |
                             FICP_FEAT_FILE_SYNC |
                             FICP_FEAT_STREAM_RX |
                             FICP_FEAT_RTC_SYNC |
                             FICP_FEAT_SCREENSHOT;
    cfg.telem_hz_default   = 5U;
    cfg.enable_stream_rx   = true;
    cfg.enable_stream_tx   = false;
    cfg.ctrl_rx_ring_bytes = 4096U;
    cfg.primary_fs         = sd_card_mounted ? (sd_card_use_spi ? (fs::FS *)&SD : (fs::FS *)&SD_MMC) : nullptr;
    cfg.fallback_fs        = &FFat;
    cfg.fs_mux             = fs_mux;
    cfg.snapshot_cb        = rover_ble_snapshot_cb;
    cfg.config_update_cb   = rover_ble_config_update_cb;
    cfg.control_mode_cb    = rover_ble_control_mode_cb;
    cfg.ext_rpc_cb         = rover_ble_ext_rpc_cb;
    cfg.stream_rx_cb       = rover_ble_stream_rx_cb;
    cfg.telem_pack_cb      = rover_ble_telem_pack_cb;
    cfg.health_cb          = rover_ble_health_cb;
    cfg.rtc_set_cb         = rover_ble_rtc_set_cb;
    cfg.screenshot_cb      = rover_take_screenshot_to;
    cfg.file_lock_check_cb = rover_is_storage_write_locked;
    cfg.file_committed_cb  = rover_ble_file_committed_cb;
    cfg.conn_event_cb      = rover_ble_conn_event_cb;
    cfg.log_cb             = rover_printf;

    fic_ble_companion_configure(&cfg);
    if (current_settings.ble_enabled) {
        (void)fic_ble_companion_start();
    } else {
        rover_printf("[BLE] Disabled by /settings.json (ble_enabled=false).\n");
    }
}

extern "C" void ble_bridge_loop(void)
{
    const int8_t cmd = s_pending_cmd;
    if (cmd < 0) {
        s_pending_cmd = 0;
        if (fic_ble_companion_is_active()) {
            fic_ble_companion_stop();
            s_ui_dirty = true;
        }
    } else if (cmd > 0) {
        if (!fic_web_portal_is_active() && !fic_web_portal_is_starting()) {
            s_pending_cmd = 0;
            (void)fic_ble_companion_start();
            s_ui_dirty = true;
        }
    }
    fic_ble_companion_loop();
}

extern "C" void ble_bridge_feed_state(const RoverUIData_t *state)
{
    if (!state) return;
    portENTER_CRITICAL(&s_telem_mux);
    s_telem.valid             = true;
    s_telem.tip_lat           = state->tip_lat;
    s_telem.tip_lon           = state->tip_lon;
    s_telem.tip_height_m      = state->tip_height_m;
    s_telem.base_lat          = state->base_lat;
    s_telem.base_lon          = state->base_lon;
    s_telem.base_id           = state->base_station_id;
    s_telem.base_valid        = state->base_coords_valid;
    s_telem.baro_rel_m        = state->beam_baro_rel_elev_m;
    s_telem.fix_type          = state->fix_type;
    s_telem.slave_fix_type    = state->slave_fix_type;
    s_telem.hdg_src           = state->heading_source;
    s_telem.hdg_valid         = state->heading_valid;
    s_telem.sv1               = state->num_sats;
    s_telem.sv2               = state->slave_num_sats;
    s_telem.true_hdg_deg      = state->true_heading_deg;
    s_telem.bl_len_m          = state->baseline_length_m;
    s_telem.bl_pitch_deg      = state->baseline_pitch_deg;
    s_telem.beam_pitch_deg    = state->beam_pitch_deg;
    s_telem.beam_roll_deg     = state->beam_roll_deg;
    s_telem.mag_anomaly       = state->mag_anomaly;
    s_telem.lever_arm_applied = state->lever_arm_applied;
    s_telem.h_acc_m           = state->h_acc_m;
    s_telem.v_acc_m           = state->v_acc_m;
    s_telem.diff_age_s        = state->diff_age_s;
    s_telem.rssi_dbm          = state->e220_stats.last_pkt_rssi_dbm;
    s_telem.noise_dbm         = state->e220_stats.ambient_noise_dbm;
    s_telem.air_rate_kbps     = state->e220_stats.active_air_rate_kbps;
    s_telem.active_wp_idx     = state->active_target_idx;
    s_telem.survey_active     = state->survey_in_progress;
    s_telem.survey_pct        = state->survey_progress_pct;
    portEXIT_CRITICAL(&s_telem_mux);
}

extern "C" void ble_bridge_request_toggle(void)
{
    if (fic_ble_companion_is_active() || s_pending_cmd > 0) {
        current_settings.ble_enabled = false;
        s_pending_cmd = -1;
    } else {
        current_settings.ble_enabled = true;
        if (fic_web_portal_is_active() || fic_web_portal_is_starting()) {
            fic_web_portal_stop();
        }
        s_pending_cmd = 1;
    }
    mark_settings_dirty();
    s_ui_dirty = true;
}

extern "C" void ble_bridge_reclaim_local(void)
{
    s_slave_dimmed = false;
    set_backlight_pwm(current_settings.brightness);
    fic_audio_beeper_set_muted(current_settings.is_muted || current_settings.volume == 0U);
    (void)fic_ble_companion_reclaim_local("local_reclaim");
    s_ui_dirty = true;
}

extern "C" RoverBleUiState_t ble_bridge_get_ui_state(void)
{
    if (fic_web_portal_is_active() || fic_web_portal_is_starting()) {
        return current_settings.ble_enabled ? ROVER_BLE_UI_WIFI_HOLD : ROVER_BLE_UI_OFF;
    }
    if (s_pending_cmd > 0) {
        return ROVER_BLE_UI_STARTING;
    }
    if (!fic_ble_companion_is_active()) {
        return ROVER_BLE_UI_OFF;
    }
    if (fic_ble_companion_is_connected()) {
        return s_slave_dimmed ? ROVER_BLE_UI_SLAVE : ROVER_BLE_UI_CONN;
    }
    return ROVER_BLE_UI_ADV;
}

extern "C" bool ble_bridge_is_active(void)
{
    return fic_ble_companion_is_active();
}

extern "C" bool ble_bridge_is_connected(void)
{
    return fic_ble_companion_is_connected();
}

extern "C" bool ble_bridge_is_slave_dimmed(void)
{
    return s_slave_dimmed;
}

extern "C" bool ble_bridge_consume_ui_dirty(void)
{
    if (!s_ui_dirty) return false;
    s_ui_dirty = false;
    return true;
}

extern "C" void ble_bridge_format_status(char *buf, size_t buf_sz)
{
    if (!buf || buf_sz == 0U) return;
    FicBleStats_t st = {};
    fic_ble_companion_get_stats(&st);
    switch (ble_bridge_get_ui_state()) {
        case ROVER_BLE_UI_SLAVE:
            snprintf(buf, buf_sz, "BLE: Slave Mode (MTU %u)", (unsigned)st.peer_mtu);
            break;
        case ROVER_BLE_UI_CONN:
            snprintf(buf, buf_sz, "BLE: Connected (MTU %u)", (unsigned)st.peer_mtu);
            break;
        case ROVER_BLE_UI_ADV:
            snprintf(buf, buf_sz, "BLE: Advertising (RTK-Rover)");
            break;
        case ROVER_BLE_UI_STARTING:
            snprintf(buf, buf_sz, "BLE: Starting...");
            break;
        case ROVER_BLE_UI_WIFI_HOLD:
            snprintf(buf, buf_sz, "BLE: Held while Wi-Fi Active");
            break;
        default:
            snprintf(buf, buf_sz, "Radios: Wi-Fi & BLE Off");
            break;
    }
}

extern "C" void ble_bridge_notify_local_ui_change(const char *settings_json, const char *runtime_json)
{
    fic_ble_companion_notify_local_change("local_ui", settings_json, runtime_json);
}

extern "C" void ble_bridge_set_stream_rx_sink(ble_bridge_stream_sink_cb_t sink_cb)
{
    s_stream_sink = sink_cb;
}
