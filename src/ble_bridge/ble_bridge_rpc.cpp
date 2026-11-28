#include "ble_bridge_internal.h"
#include "../../user_config.h"
#include "../settings_manager/settings_manager.h"
#include "../waypoint_manager/waypoint_manager.h"
#include "../e220_driver/e220_driver.h"
#include "../rover_sensors/rover_sensors.h"
#include "../rtcm_arbiter/rtcm_arbiter.h"
#include "../storage/route_logger.h"
#include "../storage/serial_logger.h"

#include <ArduinoJson.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

#define FICP_MSG_ROVER_SET_TARGET_POINT   ((uint8_t)(FICP_MSG_EXT_BASE + 0x02U))
#define FICP_MSG_ROVER_SET_LINE_STAKEOUT  ((uint8_t)(FICP_MSG_EXT_BASE + 0x03U))
#define FICP_MSG_ROVER_SELECT_WAYPOINT    ((uint8_t)(FICP_MSG_EXT_BASE + 0x04U))
#define FICP_MSG_ROVER_SURVEY_LOG         ((uint8_t)(FICP_MSG_EXT_BASE + 0x05U))
#define FICP_MSG_ROVER_ROUTE_REC_SET      ((uint8_t)(FICP_MSG_EXT_BASE + 0x06U))
#define FICP_MSG_ROVER_RADIO_SET_RATE     ((uint8_t)(FICP_MSG_EXT_BASE + 0x07U))
#define FICP_MSG_ROVER_RTCM_SET_MODE      ((uint8_t)(FICP_MSG_EXT_BASE + 0x08U))
#define FICP_MSG_ROVER_SENSOR_CAL         ((uint8_t)(FICP_MSG_EXT_BASE + 0x09U))
#define FICP_MSG_ROVER_POLE_GEOM_SET      ((uint8_t)(FICP_MSG_EXT_BASE + 0x0AU))
#define FICP_MSG_EVT_ROVER_SURVEY_RESULT  ((uint8_t)(FICP_MSG_EXT_BASE + 0x21U))

typedef struct {
    bool     initialized;
    float    antenna_height_m;
    float    antenna_spacing_m;
    bool     apply_lever_arm;
    bool     apply_lever_arm_on_calibrated_beam;
    char     rtcm_source_mode[16];
    uint32_t lora_air_rate_bps;
    bool     use_imperial_units;
    uint8_t  audio_nav_mode;
    uint8_t  survey_avg_seconds;
    float    survey_max_tilt_deg;
    float    line_log_max_m;
    float    line_audio_fade_m;
    bool     rec_active;
    bool     survey_active;
    int16_t  wp_idx;
    int16_t  wp_count;
    char     wp_file[48];
    bool     remote_active;
    bool     remote_line;
    char     remote_label_b[WAYPOINT_LABEL_LEN];
} RoverReflectedState_t;

static RoverReflectedState_t s_ref = {};
static uint32_t s_last_reflect_poll_ms  = 0U;
static uint32_t s_last_survey_prog_ms   = 0U;
static uint8_t  s_last_survey_prog_pct  = 255U;

static bool fail_rpc(FicpErrCode_t *err_code_out,
                     char *err_reason_out,
                     size_t err_reason_sz,
                     FicpErrCode_t code,
                     const char *reason)
{
    if (err_code_out) *err_code_out = code;
    if (err_reason_out && err_reason_sz > 0U) {
        snprintf(err_reason_out, err_reason_sz, "%s", reason ? reason : "Invalid request");
    }
    return false;
}

static bool valid_lat_lon(double lat, double lon)
{
    return isfinite(lat) && isfinite(lon) &&
           lat >= -90.0 && lat <= 90.0 &&
           lon >= -180.0 && lon <= 180.0 &&
           (fabs(lat) > 0.0001 || fabs(lon) > 0.0001);
}

static void capture_current_reflected_state(RoverReflectedState_t *out)
{
    if (!out) return;
    memset(out, 0, sizeof(*out));
    SystemSettings snap = current_settings;
    if (settings_lock(20U)) {
        settings_get_snapshot(&snap);
        settings_unlock();
    }
    out->initialized                        = true;
    out->antenna_height_m                   = snap.antenna_height_m;
    out->antenna_spacing_m                  = snap.antenna_spacing_m;
    out->apply_lever_arm                    = snap.apply_lever_arm;
    out->apply_lever_arm_on_calibrated_beam = snap.apply_lever_arm_on_calibrated_beam;
    strncpy(out->rtcm_source_mode, snap.rtcm_source_mode, sizeof(out->rtcm_source_mode) - 1U);
    out->lora_air_rate_bps                  = (uint32_t)lroundf(e220_get_active_air_rate() * 1000.0f);
    out->use_imperial_units                 = snap.use_imperial_units;
    out->audio_nav_mode                     = snap.audio_nav_mode;
    out->survey_avg_seconds                 = snap.survey_avg_seconds;
    out->survey_max_tilt_deg                = snap.survey_max_tilt_deg;
    out->line_log_max_m                     = snap.line_log_max_m;
    out->line_audio_fade_m                  = snap.line_audio_fade_m;
    out->rec_active                         = route_logger_is_active();
    out->survey_active                      = waypoint_is_survey_in_progress();
    out->wp_idx                             = g_active_waypoint_idx;
    out->wp_count                           = g_waypoint_count;
    strncpy(out->wp_file, snap.active_waypoint_file, sizeof(out->wp_file) - 1U);
    (void)waypoint_remote_guidance_get_snapshot(&out->remote_active,
                                                &out->remote_line,
                                                NULL, NULL, NULL, 0U,
                                                NULL, NULL,
                                                out->remote_label_b,
                                                sizeof(out->remote_label_b));
}

static void emit_runtime_guidance_event(const char *origin)
{
    if (!fic_ble_companion_is_connected()) return;
    bool r_act = false, r_line = false;
    char lbl_b[WAYPOINT_LABEL_LEN] = "";
    (void)waypoint_remote_guidance_get_snapshot(&r_act, &r_line,
                                                NULL, NULL, NULL, 0U,
                                                NULL, NULL, lbl_b, sizeof(lbl_b));
    JsonDocument doc;
    doc["guidance_mode"] = r_act ? (r_line ? "remote_line" : "remote_point") : "csv";
    doc["wp_idx"]        = g_active_waypoint_idx;
    doc["wp_count"]      = g_waypoint_count;
    doc["wp_file"]       = current_settings.active_waypoint_file;
    if (r_act) {
        doc["target_label"] = lbl_b;
    } else {
        WaypointRecord_t wp;
        if (waypoint_get_active(&wp)) {
            doc["target_label"] = wp.label;
        }
    }
    char rt_buf[224];
    size_t n = serializeJson(doc, rt_buf, sizeof(rt_buf));
    if (n > 0U && n < sizeof(rt_buf)) {
        fic_ble_companion_notify_local_change(origin ? origin : "remote_rpc", nullptr, rt_buf);
    }
}

void ble_bridge_sync_reflected_snapshot(void)
{
    capture_current_reflected_state(&s_ref);
}

void ble_bridge_poll_local_reflection(uint32_t now_ms)
{
    if ((uint32_t)(now_ms - s_last_reflect_poll_ms) < 250U) return;
    s_last_reflect_poll_ms = now_ms;

    if (!s_ref.initialized) {
        capture_current_reflected_state(&s_ref);
        return;
    }

    RoverReflectedState_t cur;
    capture_current_reflected_state(&cur);

    JsonDocument sdoc;
    JsonDocument rdoc;
    bool has_s = false;
    bool has_r = false;

    if (fabsf(cur.antenna_height_m - s_ref.antenna_height_m) > 0.0005f) {
        sdoc["antenna_height_m"] = cur.antenna_height_m;
        has_s = true;
    }
    if (fabsf(cur.antenna_spacing_m - s_ref.antenna_spacing_m) > 0.0005f) {
        sdoc["antenna_spacing_m"] = cur.antenna_spacing_m;
        has_s = true;
    }
    if (cur.apply_lever_arm != s_ref.apply_lever_arm) {
        sdoc["apply_lever_arm"] = cur.apply_lever_arm;
        has_s = true;
    }
    if (cur.apply_lever_arm_on_calibrated_beam != s_ref.apply_lever_arm_on_calibrated_beam) {
        sdoc["apply_lever_arm_on_calibrated_beam"] = cur.apply_lever_arm_on_calibrated_beam;
        has_s = true;
    }
    if (strncmp(cur.rtcm_source_mode, s_ref.rtcm_source_mode, sizeof(cur.rtcm_source_mode)) != 0) {
        sdoc["rtcm_source_mode"] = cur.rtcm_source_mode;
        has_s = true;
    }
    if (cur.lora_air_rate_bps != s_ref.lora_air_rate_bps && cur.lora_air_rate_bps > 0U) {
        sdoc["lora_air_rate_bps"] = cur.lora_air_rate_bps;
        has_s = true;
    }
    if (cur.use_imperial_units != s_ref.use_imperial_units) {
        sdoc["use_imperial_units"] = cur.use_imperial_units;
        has_s = true;
    }
    if (cur.audio_nav_mode != s_ref.audio_nav_mode) {
        sdoc["audio_nav_mode"] = cur.audio_nav_mode;
        has_s = true;
    }
    if (cur.survey_avg_seconds != s_ref.survey_avg_seconds) {
        sdoc["survey_avg_seconds"] = cur.survey_avg_seconds;
        has_s = true;
    }
    if (fabsf(cur.line_log_max_m - s_ref.line_log_max_m) > 0.05f) {
        sdoc["line_log_max_m"] = cur.line_log_max_m;
        has_s = true;
    }
    if (fabsf(cur.line_audio_fade_m - s_ref.line_audio_fade_m) > 0.05f) {
        sdoc["line_audio_fade_m"] = cur.line_audio_fade_m;
        has_s = true;
    }

    if (cur.rec_active != s_ref.rec_active) {
        rdoc["rec_active"] = cur.rec_active;
        has_r = true;
    }
    if (cur.survey_active != s_ref.survey_active) {
        rdoc["survey_active"] = cur.survey_active;
        has_r = true;
    }
    if (cur.wp_idx != s_ref.wp_idx || cur.wp_count != s_ref.wp_count ||
        strncmp(cur.wp_file, s_ref.wp_file, sizeof(cur.wp_file)) != 0 ||
        cur.remote_active != s_ref.remote_active || cur.remote_line != s_ref.remote_line ||
        strncmp(cur.remote_label_b, s_ref.remote_label_b, sizeof(cur.remote_label_b)) != 0) {
        rdoc["guidance_mode"] = cur.remote_active ? (cur.remote_line ? "remote_line" : "remote_point") : "csv";
        rdoc["wp_idx"]        = cur.wp_idx;
        rdoc["wp_count"]      = cur.wp_count;
        rdoc["wp_file"]       = cur.wp_file;
        has_r = true;
    }

    s_ref = cur;

    if ((!has_s && !has_r) || !fic_ble_companion_is_connected()) {
        return;
    }

    char s_buf[256];
    char r_buf[224];
    const char *s_ptr = nullptr;
    const char *r_ptr = nullptr;
    if (has_s) {
        size_t sn = serializeJson(sdoc, s_buf, sizeof(s_buf));
        if (sn > 0U && sn < sizeof(s_buf)) s_ptr = s_buf;
    }
    if (has_r) {
        size_t rn = serializeJson(rdoc, r_buf, sizeof(r_buf));
        if (rn > 0U && rn < sizeof(r_buf)) r_ptr = r_buf;
    }
    if (s_ptr || r_ptr) {
        fic_ble_companion_notify_local_change("local_ui", s_ptr, r_ptr);
    }
}

void ble_bridge_on_survey_result_event(bool ok,
                                       const char *reason,
                                       const char *label,
                                       double lat,
                                       double lon,
                                       float h_sigma_m,
                                       float max_tilt_deg,
                                       uint16_t samples)
{
    s_last_survey_prog_pct = 255U;
    if (ok) {
        ble_bridge_mark_ui_dirty(true);
    } else {
        ble_bridge_mark_ui_dirty(false);
    }

    if (fic_ble_companion_is_connected()) {
        JsonDocument doc;
        doc["status"]       = ok ? "logged" : "aborted";
        doc["ok"]           = ok;
        doc["reason"]       = reason ? reason : (ok ? "OK" : "ABORT");
        doc["label"]        = label ? label : "";
        if (ok) {
            float sig_safe  = isfinite(h_sigma_m) ? h_sigma_m : 0.0f;
            float tilt_safe = isfinite(max_tilt_deg) ? max_tilt_deg : 0.0f;
            char sig_buf[20];
            char tilt_buf[16];
            snprintf(sig_buf, sizeof(sig_buf), "%.4f", sig_safe);
            snprintf(tilt_buf, sizeof(tilt_buf), "%.2f", tilt_safe);
            doc["lat"]          = lat;
            doc["lon"]          = lon;
            doc["h_sigma_m"]    = serialized((const char *)sig_buf);
            doc["max_tilt_deg"] = serialized((const char *)tilt_buf);
            doc["samples"]      = samples;
        }
        char buf[256];
        size_t n = serializeJson(doc, buf, sizeof(buf));
        if (n > 0U && n < sizeof(buf)) {
            (void)fic_ble_companion_send_ext_packet(FICP_MSG_EVT_ROVER_SURVEY_RESULT,
                                                    FICP_FLAG_IS_JSON,
                                                    (const uint8_t *)buf,
                                                    (uint16_t)n);
        }
    }
}

void ble_bridge_poll_survey_progress(uint32_t now_ms, bool survey_active, uint8_t pct)
{
    if (!survey_active) {
        s_last_survey_prog_pct = 255U;
        return;
    }
    if (!fic_ble_companion_is_connected()) return;
    if ((uint32_t)(now_ms - s_last_survey_prog_ms) < 1000U && pct == s_last_survey_prog_pct) {
        return;
    }
    if ((uint32_t)(now_ms - s_last_survey_prog_ms) < 500U) {
        return;
    }
    s_last_survey_prog_ms  = now_ms;
    s_last_survey_prog_pct = pct;

    char buf[72];
    int n = snprintf(buf, sizeof(buf), "{\"status\":\"in_progress\",\"pct\":%u}", (unsigned)pct);
    if (n > 0 && (size_t)n < sizeof(buf)) {
        (void)fic_ble_companion_send_ext_packet(FICP_MSG_EVT_ROVER_SURVEY_RESULT,
                                                FICP_FLAG_IS_JSON,
                                                (const uint8_t *)buf,
                                                (uint16_t)n);
    }
}

static bool handle_set_target_point(const JsonDocument &doc,
                                    char *ack_json_out,
                                    size_t ack_json_sz,
                                    FicpErrCode_t *err_code_out,
                                    char *err_reason_out,
                                    size_t err_reason_sz)
{
    double lat = doc["lat"].is<double>() ? doc["lat"].as<double>() : (doc["lat_deg"] | NAN);
    double lon = doc["lon"].is<double>() ? doc["lon"].as<double>() : (doc["lon_deg"] | NAN);
    if (!valid_lat_lon(lat, lon)) {
        return fail_rpc(err_code_out, err_reason_out, err_reason_sz,
                        FICP_ERR_INVALID_PARAM, "Invalid lat/lon coordinates");
    }
    const char *label = doc["label"] | "REMOTE_PT";
    waypoint_remote_guidance_set_point(lat, lon, label);
    ble_bridge_mark_ui_dirty(false);
    ble_bridge_sync_reflected_snapshot();
    emit_runtime_guidance_event("remote_rpc");
    if (ack_json_out && ack_json_sz > 0U) {
        snprintf(ack_json_out, ack_json_sz,
                 "{\"mode\":\"remote_point\",\"lat\":%.8f,\"lon\":%.8f}", lat, lon);
    }
    return true;
}

static bool handle_set_line_stakeout(const JsonDocument &doc,
                                     char *ack_json_out,
                                     size_t ack_json_sz,
                                     FicpErrCode_t *err_code_out,
                                     char *err_reason_out,
                                     size_t err_reason_sz)
{
    if (doc["swap"] | false) {
        if (!waypoint_remote_guidance_is_line_mode() && g_waypoint_count < 2) {
            return fail_rpc(err_code_out, err_reason_out, err_reason_sz,
                            FICP_ERR_INVALID_PARAM, "Need >= 2 waypoints to swap");
        }
        waypoint_line_stakeout_swap();
        ble_bridge_mark_ui_dirty(false);
        ble_bridge_sync_reflected_snapshot();
        emit_runtime_guidance_event("remote_rpc");
        if (ack_json_out && ack_json_sz > 0U) {
            snprintf(ack_json_out, ack_json_sz, "{\"mode\":\"swap\"}");
        }
        return true;
    }

    if (doc["idx_a"].is<int>() && doc["idx_b"].is<int>()) {
        int idx_a = doc["idx_a"].as<int>();
        int idx_b = doc["idx_b"].as<int>();
        if (idx_a < 0 || idx_a >= g_waypoint_count ||
            idx_b < 0 || idx_b >= g_waypoint_count || idx_a == idx_b) {
            return fail_rpc(err_code_out, err_reason_out, err_reason_sz,
                            FICP_ERR_INVALID_PARAM, "Invalid stakeout index pair");
        }
        waypoint_line_stakeout_set_points((int16_t)idx_a, (int16_t)idx_b);
        ble_bridge_mark_ui_dirty(true);
        ble_bridge_sync_reflected_snapshot();
        emit_runtime_guidance_event("remote_rpc");
        if (ack_json_out && ack_json_sz > 0U) {
            snprintf(ack_json_out, ack_json_sz,
                     "{\"mode\":\"csv_line\",\"idx_a\":%d,\"idx_b\":%d}", idx_a, idx_b);
        }
        return true;
    }

    double lat_a = doc["lat_a"] | NAN;
    double lon_a = doc["lon_a"] | NAN;
    double lat_b = doc["lat_b"] | NAN;
    double lon_b = doc["lon_b"] | NAN;
    if (!valid_lat_lon(lat_a, lon_a) || !valid_lat_lon(lat_b, lon_b)) {
        return fail_rpc(err_code_out, err_reason_out, err_reason_sz,
                        FICP_ERR_INVALID_PARAM, "Invalid line A/B coordinates");
    }
    const char *lbl_a = doc["label_a"] | "REMOTE_A";
    const char *lbl_b = doc["label_b"] | "REMOTE_B";
    waypoint_remote_guidance_set_line(lat_a, lon_a, lbl_a, lat_b, lon_b, lbl_b);
    ble_bridge_mark_ui_dirty(false);
    ble_bridge_sync_reflected_snapshot();
    emit_runtime_guidance_event("remote_rpc");
    if (ack_json_out && ack_json_sz > 0U) {
        snprintf(ack_json_out, ack_json_sz, "{\"mode\":\"remote_line\"}");
    }
    return true;
}

static bool handle_select_waypoint(const JsonDocument &doc,
                                   char *ack_json_out,
                                   size_t ack_json_sz,
                                   FicpErrCode_t *err_code_out,
                                   char *err_reason_out,
                                   size_t err_reason_sz)
{
    bool acted = false;
    if (doc["clear_remote"] | false) {
        waypoint_remote_guidance_clear();
        acted = true;
    }
    const char *fname = doc["wp_file"].is<const char *>() ? doc["wp_file"].as<const char *>() :
                        (doc["filename"].is<const char *>() ? doc["filename"].as<const char *>() :
                         (doc["file"].is<const char *>() ? doc["file"].as<const char *>() : nullptr));
    if (fname && fname[0] != '\0') {
        if (!waypoint_select_by_filename(fname)) {
            return fail_rpc(err_code_out, err_reason_out, err_reason_sz,
                            FICP_ERR_INVALID_PARAM, "Waypoint CSV file not found or empty");
        }
        acted = true;
    }
    if (doc["wp_idx"].is<int>() || doc["index"].is<int>()) {
        int idx = doc["wp_idx"].is<int>() ? doc["wp_idx"].as<int>() : doc["index"].as<int>();
        if (idx < 0 || idx >= g_waypoint_count) {
            return fail_rpc(err_code_out, err_reason_out, err_reason_sz,
                            FICP_ERR_INVALID_PARAM, "Waypoint index out of range");
        }
        waypoint_select_index((int16_t)idx);
        acted = true;
    } else if (doc["next"] | false) {
        waypoint_select_next();
        acted = true;
    } else if (doc["prev"] | false) {
        waypoint_select_prev();
        acted = true;
    }
    if (!acted) {
        return fail_rpc(err_code_out, err_reason_out, err_reason_sz,
                        FICP_ERR_INVALID_PARAM, "Missing wp_idx or wp_file");
    }
    ble_bridge_mark_ui_dirty(true);
    ble_bridge_sync_reflected_snapshot();
    emit_runtime_guidance_event("remote_rpc");
    if (ack_json_out && ack_json_sz > 0U) {
        snprintf(ack_json_out, ack_json_sz,
                 "{\"wp_idx\":%d,\"wp_count\":%d,\"wp_file\":\"%s\"}",
                 (int)g_active_waypoint_idx,
                 (int)g_waypoint_count,
                 current_settings.active_waypoint_file);
    }
    return true;
}

static bool handle_survey_log(const JsonDocument &doc,
                              char *ack_json_out,
                              size_t ack_json_sz,
                              FicpErrCode_t *err_code_out,
                              char *err_reason_out,
                              size_t err_reason_sz)
{
    const char *act = doc["action"] | "start";
    const bool want_abort = (doc["abort"] | false) ||
                            (strcasecmp(act, "abort") == 0) ||
                            (strcasecmp(act, "cancel") == 0);
    if (want_abort) {
        if (!waypoint_survey_cancel("CANCELLED_REMOTE")) {
            return fail_rpc(err_code_out, err_reason_out, err_reason_sz,
                            FICP_ERR_INVALID_PARAM, "No active survey to cancel");
        }
        ble_bridge_mark_ui_dirty(false);
        if (ack_json_out && ack_json_sz > 0U) {
            snprintf(ack_json_out, ack_json_sz, "{\"action\":\"abort\",\"aborted\":true}");
        }
        return true;
    }

    if (waypoint_is_survey_in_progress()) {
        return fail_rpc(err_code_out, err_reason_out, err_reason_sz,
                        FICP_ERR_BUSY_LOCKED, "Survey averaging already in progress");
    }

    int dur = doc["duration_s"] | 0;
    if (dur < 0 || dur > 60) {
        return fail_rpc(err_code_out, err_reason_out, err_reason_sz,
                        FICP_ERR_INVALID_PARAM, "duration_s out of range (0..60)");
    }
    const char *prefix = doc["prefix"] | "";
    const char *note   = doc["note"] | "";
    if (!waypoint_survey_start_remote((uint8_t)dur, prefix, note)) {
        return fail_rpc(err_code_out, err_reason_out, err_reason_sz,
                        FICP_ERR_INVALID_PARAM, "Invalid prefix or survey busy");
    }
    ble_bridge_mark_ui_dirty(false);
    if (ack_json_out && ack_json_sz > 0U) {
        snprintf(ack_json_out, ack_json_sz, "{\"action\":\"start\",\"started\":true}");
    }
    return true;
}

static bool handle_route_rec_set(const uint8_t *payload,
                                 uint16_t len,
                                 char *ack_json_out,
                                 size_t ack_json_sz,
                                 FicpErrCode_t *err_code_out,
                                 char *err_reason_out,
                                 size_t err_reason_sz)
{
    bool have_val = false;
    bool target_active = false;
    if (payload && len == 1U && payload[0] <= 1U) {
        have_val = true;
        target_active = (payload[0] == 1U);
    } else if (payload && len > 0U) {
        JsonDocument doc;
        if (deserializeJson(doc, payload, len) == DeserializationError::Ok) {
            if (doc["active"].is<bool>()) {
                have_val = true;
                target_active = doc["active"].as<bool>();
            } else if (doc["rec_active"].is<bool>()) {
                have_val = true;
                target_active = doc["rec_active"].as<bool>();
            }
        }
    }
    if (!have_val) {
        return fail_rpc(err_code_out, err_reason_out, err_reason_sz,
                        FICP_ERR_INVALID_PARAM, "Expected active boolean");
    }

    if (target_active) {
        if (!route_logger_start()) {
            return fail_rpc(err_code_out, err_reason_out, err_reason_sz,
                            FICP_ERR_BUSY_LOCKED, "Route logger failed (SD unavailable)");
        }
    } else {
        route_logger_stop();
    }
    ble_bridge_mark_ui_dirty(false);
    ble_bridge_sync_reflected_snapshot();

    const char *fname = route_logger_get_current_filename();
    char rbuf[96];
    snprintf(rbuf, sizeof(rbuf), "{\"rec_active\":%s}", route_logger_is_active() ? "true" : "false");
    fic_ble_companion_notify_local_change("remote_rpc", nullptr, rbuf);
    if (ack_json_out && ack_json_sz > 0U) {
        snprintf(ack_json_out, ack_json_sz,
                 "{\"rec_active\":%s,\"file\":\"%s\"}",
                 route_logger_is_active() ? "true" : "false",
                 fname ? fname : "");
    }
    return true;
}

static bool handle_radio_set_rate(const JsonDocument &doc,
                                  char *ack_json_out,
                                  size_t ack_json_sz,
                                  FicpErrCode_t *err_code_out,
                                  char *err_reason_out,
                                  size_t err_reason_sz)
{
    float target_kbps = 0.0f;
    if (doc["lora_air_rate_bps"].is<int>() || doc["bps"].is<int>()) {
        int bps = doc["lora_air_rate_bps"].is<int>() ? doc["lora_air_rate_bps"].as<int>() : doc["bps"].as<int>();
        if (bps == 4800 || bps == 9600 || bps == 19200) {
            target_kbps = (float)bps / 1000.0f;
        }
    } else if (doc["rate_kbps"].is<float>() || doc["rate_kbps"].is<double>()) {
        float k = doc["rate_kbps"].as<float>();
        if (fabsf(k - 4.8f) < 0.1f)       target_kbps = 4.8f;
        else if (fabsf(k - 9.6f) < 0.1f)  target_kbps = 9.6f;
        else if (fabsf(k - 19.2f) < 0.1f) target_kbps = 19.2f;
    }
    if (target_kbps <= 0.0f) {
        return fail_rpc(err_code_out, err_reason_out, err_reason_sz,
                        FICP_ERR_INVALID_PARAM, "Supported air rates: 4800, 9600, 19200 bps");
    }
    if (!e220_request_air_rate_switch(target_kbps)) {
        return fail_rpc(err_code_out, err_reason_out, err_reason_sz,
                        FICP_ERR_BUSY_LOCKED, "E220 air rate switch rejected");
    }
    mark_settings_dirty();
    ble_bridge_mark_ui_dirty(false);
    ble_bridge_sync_reflected_snapshot();
    char sbuf[64];
    snprintf(sbuf, sizeof(sbuf), "{\"lora_air_rate_bps\":%u}", (unsigned)lroundf(target_kbps * 1000.0f));
    fic_ble_companion_notify_local_change("remote_rpc", sbuf, nullptr);
    if (ack_json_out && ack_json_sz > 0U) {
        snprintf(ack_json_out, ack_json_sz,
                 "{\"requested_kbps\":%.1f,\"active_kbps\":%.1f}",
                 target_kbps, e220_get_active_air_rate());
    }
    return true;
}

static bool handle_sensor_cal(const JsonDocument &doc,
                              char *ack_json_out,
                              size_t ack_json_sz,
                              FicpErrCode_t *err_code_out,
                              char *err_reason_out,
                              size_t err_reason_sz)
{
    const char *act = doc["action"] | "";
    if (strcasecmp(act, "tare") == 0 || strcasecmp(act, "tare_level") == 0) {
        trigger_tare_pole_level();
    } else if (strcasecmp(act, "mag_cal_start_or_finish") == 0 || strcasecmp(act, "mag_cal") == 0) {
        trigger_mag_cal_start_or_finish();
    } else if (strcasecmp(act, "mag_cal_cancel") == 0 || strcasecmp(act, "cancel") == 0) {
        trigger_mag_cal_cancel();
    } else {
        return fail_rpc(err_code_out, err_reason_out, err_reason_sz,
                        FICP_ERR_INVALID_PARAM, "Unknown sensor calibration action");
    }
    ble_bridge_mark_ui_dirty(false);
    if (ack_json_out && ack_json_sz > 0U) {
        snprintf(ack_json_out, ack_json_sz,
                 "{\"action\":\"%s\",\"mag_cal_active\":%s}",
                 act, rover_sensors_is_mag_cal_collecting() ? "true" : "false");
    }
    return true;
}

static bool handle_pole_geom_set(const JsonDocument &doc,
                                 char *ack_json_out,
                                 size_t ack_json_sz,
                                 FicpErrCode_t *err_code_out,
                                 char *err_reason_out,
                                 size_t err_reason_sz)
{
    if (!settings_lock(50U)) {
        return fail_rpc(err_code_out, err_reason_out, err_reason_sz,
                        FICP_ERR_BUSY_LOCKED, "Settings lock timeout");
    }
    bool changed = false;
    if (doc["antenna_height_m"].is<float>() || doc["antenna_height_m"].is<double>()) {
        float h = doc["antenna_height_m"].as<float>();
        if (h >= 0.0f && h <= 5.0f) {
            current_settings.antenna_height_m = h;
            changed = true;
        }
    }
    if (doc["antenna_spacing_m"].is<float>() || doc["antenna_spacing_m"].is<double>()) {
        float s = doc["antenna_spacing_m"].as<float>();
        if (s >= 0.2f && s <= 5.0f) {
            current_settings.antenna_spacing_m = s;
            changed = true;
        }
    }
    if (doc["line_log_max_m"].is<float>() || doc["line_log_max_m"].is<double>()) {
        float v = doc["line_log_max_m"].as<float>();
        if (v >= 2.0f && v <= 200.0f) {
            current_settings.line_log_max_m = v;
            changed = true;
        }
    }
    if (doc["line_audio_fade_m"].is<float>() || doc["line_audio_fade_m"].is<double>()) {
        float v = doc["line_audio_fade_m"].as<float>();
        if (v >= 1.0f && v <= 200.0f) {
            current_settings.line_audio_fade_m = v;
            changed = true;
        }
    }
    if (doc["apply_lever_arm"].is<bool>()) {
        current_settings.apply_lever_arm = doc["apply_lever_arm"].as<bool>();
        changed = true;
    }
    if (doc["apply_lever_arm_on_calibrated_beam"].is<bool>()) {
        current_settings.apply_lever_arm_on_calibrated_beam =
            doc["apply_lever_arm_on_calibrated_beam"].as<bool>();
        changed = true;
    }
    float out_h = current_settings.antenna_height_m;
    float out_s = current_settings.antenna_spacing_m;
    settings_unlock();

    if (!changed) {
        return fail_rpc(err_code_out, err_reason_out, err_reason_sz,
                        FICP_ERR_INVALID_PARAM, "No valid pole geometry fields provided");
    }
    mark_settings_dirty();
    ble_bridge_mark_ui_dirty(false);
    ble_bridge_sync_reflected_snapshot();
    if (ack_json_out && ack_json_sz > 0U) {
        snprintf(ack_json_out, ack_json_sz,
                 "{\"antenna_height_m\":%.3f,\"antenna_spacing_m\":%.3f}", out_h, out_s);
    }
    return true;
}

bool ble_bridge_ext_rpc_dispatch(uint8_t msg_id,
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

    if (msg_id == FICP_MSG_ROVER_ROUTE_REC_SET) {
        return handle_route_rec_set(payload, len, ack_json_out, ack_json_sz,
                                    err_code_out, err_reason_out, err_reason_sz);
    }

    if (msg_id == FICP_MSG_ROVER_RTCM_SET_MODE) {
        bool ok = false;
        if (payload && len == 1U && payload[0] <= (uint8_t)RTCM_MODE_NTRIP_ONLY) {
            rtcm_arbiter_set_mode((RtcmSourceMode_t)payload[0]);
            ok = true;
        } else if (payload && len > 0U) {
            JsonDocument doc;
            if (deserializeJson(doc, payload, len) == DeserializationError::Ok &&
                doc["mode"].is<const char *>()) {
                ok = rtcm_arbiter_set_mode_from_str(doc["mode"].as<const char *>());
            }
        }
        if (!ok) {
            return fail_rpc(err_code_out, err_reason_out, err_reason_sz,
                            FICP_ERR_INVALID_PARAM, "Invalid RTCM source mode");
        }
        mark_settings_dirty();
        ble_bridge_mark_ui_dirty(false);
        ble_bridge_sync_reflected_snapshot();
        char sbuf[64];
        snprintf(sbuf, sizeof(sbuf), "{\"rtcm_source_mode\":\"%s\"}", current_settings.rtcm_source_mode);
        fic_ble_companion_notify_local_change("remote_rpc", sbuf, nullptr);
        if (ack_json_out && ack_json_sz > 0U) {
            snprintf(ack_json_out, ack_json_sz, "{\"mode\":\"%s\"}", current_settings.rtcm_source_mode);
        }
        return true;
    }

    JsonDocument doc;
    if (!payload || len == 0U || deserializeJson(doc, payload, len) != DeserializationError::Ok) {
        if (msg_id >= FICP_MSG_ROVER_SET_TARGET_POINT && msg_id <= FICP_MSG_ROVER_POLE_GEOM_SET) {
            return fail_rpc(err_code_out, err_reason_out, err_reason_sz,
                            FICP_ERR_INVALID_PARAM, "Expected valid JSON payload");
        }
        return fail_rpc(err_code_out, err_reason_out, err_reason_sz,
                        FICP_ERR_UNSUPPORTED_MSG, "Unsupported Rover ext RPC");
    }

    switch (msg_id) {
        case FICP_MSG_ROVER_SET_TARGET_POINT:
            return handle_set_target_point(doc, ack_json_out, ack_json_sz,
                                           err_code_out, err_reason_out, err_reason_sz);
        case FICP_MSG_ROVER_SET_LINE_STAKEOUT:
            return handle_set_line_stakeout(doc, ack_json_out, ack_json_sz,
                                            err_code_out, err_reason_out, err_reason_sz);
        case FICP_MSG_ROVER_SELECT_WAYPOINT:
            return handle_select_waypoint(doc, ack_json_out, ack_json_sz,
                                          err_code_out, err_reason_out, err_reason_sz);
        case FICP_MSG_ROVER_SURVEY_LOG:
            return handle_survey_log(doc, ack_json_out, ack_json_sz,
                                     err_code_out, err_reason_out, err_reason_sz);
        case FICP_MSG_ROVER_RADIO_SET_RATE:
            return handle_radio_set_rate(doc, ack_json_out, ack_json_sz,
                                         err_code_out, err_reason_out, err_reason_sz);
        case FICP_MSG_ROVER_SENSOR_CAL:
            return handle_sensor_cal(doc, ack_json_out, ack_json_sz,
                                     err_code_out, err_reason_out, err_reason_sz);
        case FICP_MSG_ROVER_POLE_GEOM_SET:
            return handle_pole_geom_set(doc, ack_json_out, ack_json_sz,
                                        err_code_out, err_reason_out, err_reason_sz);
        default:
            return fail_rpc(err_code_out, err_reason_out, err_reason_sz,
                            FICP_ERR_UNSUPPORTED_MSG, "Unsupported Rover ext RPC");
    }
}
