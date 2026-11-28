#include "waypoint_manager_internal.h"
#include <Arduino.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <strings.h>

static portMUX_TYPE s_remote_mux = portMUX_INITIALIZER_UNLOCKED;
static bool   s_remote_active    = false;
static bool   s_remote_is_line   = false;
static bool   s_remote_a_valid   = false;
static double s_remote_lat_a     = 0.0;
static double s_remote_lon_a     = 0.0;
static double s_remote_lat_b     = 0.0;
static double s_remote_lon_b     = 0.0;
static char   s_remote_label_a[WAYPOINT_LABEL_LEN] = "";
static char   s_remote_label_b[WAYPOINT_LABEL_LEN] = "";
static char   s_remote_notes_b[WAYPOINT_NOTES_LEN] = "";

static bool remote_valid_coord(double lat, double lon)
{
    return isfinite(lat) && isfinite(lon) && (fabs(lat) > 0.0001 || fabs(lon) > 0.0001);
}

static void sanitize_remote_label(const char *src, const char *fallback, char *dst, size_t dst_sz)
{
    if (!dst || dst_sz == 0U) return;
    const char *use = (src && src[0] != '\0') ? src : fallback;
    size_t j = 0U;
    for (size_t i = 0U; use && use[i] != '\0' && j + 1U < dst_sz; ++i) {
        char ch = use[i];
        if ((unsigned char)ch < 0x20U || (unsigned char)ch > 0x7EU) ch = ' ';
        if (ch == ',') ch = ' ';
        dst[j++] = ch;
    }
    dst[j] = '\0';
}

extern "C" void waypoint_remote_guidance_clear(void)
{
    bool changed = false;
    portENTER_CRITICAL(&s_remote_mux);
    if (s_remote_active) {
        s_remote_active  = false;
        s_remote_is_line = false;
        s_remote_a_valid = false;
        s_remote_label_a[0] = '\0';
        s_remote_label_b[0] = '\0';
        s_remote_notes_b[0] = '\0';
        changed = true;
    }
    portEXIT_CRITICAL(&s_remote_mux);
    if (changed) {
        g_waypoint_change_seq = g_waypoint_change_seq + 1;
    }
}

extern "C" void waypoint_remote_guidance_set_point(double lat, double lon, const char *label)
{
    if (!remote_valid_coord(lat, lon)) return;
    char clean_b[WAYPOINT_LABEL_LEN];
    sanitize_remote_label(label, "REMOTE_PT", clean_b, sizeof(clean_b));

    portENTER_CRITICAL(&s_remote_mux);
    s_remote_active  = true;
    s_remote_is_line = false;
    s_remote_a_valid = false;
    s_remote_lat_a   = 0.0;
    s_remote_lon_a   = 0.0;
    s_remote_lat_b   = lat;
    s_remote_lon_b   = lon;
    strncpy(s_remote_label_a, "HERE", sizeof(s_remote_label_a) - 1);
    s_remote_label_a[sizeof(s_remote_label_a) - 1] = '\0';
    strncpy(s_remote_label_b, clean_b, sizeof(s_remote_label_b) - 1);
    s_remote_label_b[sizeof(s_remote_label_b) - 1] = '\0';
    snprintf(s_remote_notes_b, sizeof(s_remote_notes_b), "Remote Target (HERE -> %s)", clean_b);
    portEXIT_CRITICAL(&s_remote_mux);

    g_waypoint_change_seq = g_waypoint_change_seq + 1;
}

extern "C" void waypoint_remote_guidance_set_line(double lat_a,
                                                  double lon_a,
                                                  const char *label_a,
                                                  double lat_b,
                                                  double lon_b,
                                                  const char *label_b)
{
    if (!remote_valid_coord(lat_a, lon_a) || !remote_valid_coord(lat_b, lon_b)) return;
    char clean_a[WAYPOINT_LABEL_LEN];
    char clean_b[WAYPOINT_LABEL_LEN];
    sanitize_remote_label(label_a, "REMOTE_A", clean_a, sizeof(clean_a));
    sanitize_remote_label(label_b, "REMOTE_B", clean_b, sizeof(clean_b));

    portENTER_CRITICAL(&s_remote_mux);
    s_remote_active  = true;
    s_remote_is_line = true;
    s_remote_a_valid = true;
    s_remote_lat_a   = lat_a;
    s_remote_lon_a   = lon_a;
    s_remote_lat_b   = lat_b;
    s_remote_lon_b   = lon_b;
    strncpy(s_remote_label_a, clean_a, sizeof(s_remote_label_a) - 1);
    s_remote_label_a[sizeof(s_remote_label_a) - 1] = '\0';
    strncpy(s_remote_label_b, clean_b, sizeof(s_remote_label_b) - 1);
    s_remote_label_b[sizeof(s_remote_label_b) - 1] = '\0';
    snprintf(s_remote_notes_b, sizeof(s_remote_notes_b), "Remote Line (%s -> %s)", clean_a, clean_b);
    portEXIT_CRITICAL(&s_remote_mux);

    g_waypoint_change_seq = g_waypoint_change_seq + 1;
}

extern "C" bool waypoint_remote_guidance_is_active(void)
{
    portENTER_CRITICAL(&s_remote_mux);
    bool active = s_remote_active;
    portEXIT_CRITICAL(&s_remote_mux);
    return active;
}

extern "C" bool waypoint_remote_guidance_is_line_mode(void)
{
    portENTER_CRITICAL(&s_remote_mux);
    bool line = s_remote_active && s_remote_is_line;
    portEXIT_CRITICAL(&s_remote_mux);
    return line;
}

extern "C" bool waypoint_remote_guidance_swap(void)
{
    bool swapped = false;
    portENTER_CRITICAL(&s_remote_mux);
    if (s_remote_active && s_remote_is_line) {
        double tmp_lat = s_remote_lat_a;
        double tmp_lon = s_remote_lon_a;
        char tmp_lbl[WAYPOINT_LABEL_LEN];
        strncpy(tmp_lbl, s_remote_label_a, sizeof(tmp_lbl));

        s_remote_lat_a = s_remote_lat_b;
        s_remote_lon_a = s_remote_lon_b;
        strncpy(s_remote_label_a, s_remote_label_b, sizeof(s_remote_label_a));

        s_remote_lat_b = tmp_lat;
        s_remote_lon_b = tmp_lon;
        strncpy(s_remote_label_b, tmp_lbl, sizeof(s_remote_label_b));
        snprintf(s_remote_notes_b, sizeof(s_remote_notes_b),
                 "Remote Line (%s -> %s)", s_remote_label_a, s_remote_label_b);
        swapped = true;
    }
    portEXIT_CRITICAL(&s_remote_mux);
    if (swapped) {
        g_waypoint_change_seq = g_waypoint_change_seq + 1;
    }
    return swapped;
}

extern "C" bool waypoint_remote_guidance_reanchor_here(double cur_lat, double cur_lon)
{
    bool updated = false;
    portENTER_CRITICAL(&s_remote_mux);
    if (s_remote_active && !s_remote_is_line && remote_valid_coord(cur_lat, cur_lon)) {
        s_remote_lat_a   = cur_lat;
        s_remote_lon_a   = cur_lon;
        s_remote_a_valid = true;
        updated = true;
    }
    portEXIT_CRITICAL(&s_remote_mux);
    if (updated) {
        g_waypoint_change_seq = g_waypoint_change_seq + 1;
    }
    return updated;
}

extern "C" bool waypoint_remote_guidance_get_target(WaypointRecord_t *wp_out)
{
    bool active = false;
    portENTER_CRITICAL(&s_remote_mux);
    active = s_remote_active;
    if (active && wp_out) {
        memset(wp_out, 0, sizeof(*wp_out));
        wp_out->latitude  = s_remote_lat_b;
        wp_out->longitude = s_remote_lon_b;
        strncpy(wp_out->label, s_remote_label_b, WAYPOINT_LABEL_LEN - 1);
        strncpy(wp_out->notes, s_remote_notes_b, WAYPOINT_NOTES_LEN - 1);
    }
    portEXIT_CRITICAL(&s_remote_mux);
    return active;
}

extern "C" bool waypoint_remote_guidance_get_segment(double cur_lat,
                                                   double cur_lon,
                                                   ActiveGuidanceSegment_t *seg_out)
{
    if (!seg_out) return false;
    memset(seg_out, 0, sizeof(*seg_out));
    seg_out->idx_a = -1;
    seg_out->idx_b = -1;

    bool active   = false;
    bool is_line  = false;
    bool a_valid  = false;
    double lat_a  = 0.0;
    double lon_a  = 0.0;
    double lat_b  = 0.0;
    double lon_b  = 0.0;
    char label_a[WAYPOINT_LABEL_LEN] = "";
    char label_b[WAYPOINT_LABEL_LEN] = "";
    char notes_b[WAYPOINT_NOTES_LEN] = "";

    portENTER_CRITICAL(&s_remote_mux);
    active = s_remote_active;
    if (active) {
        is_line = s_remote_is_line;
        if (!is_line && !s_remote_a_valid && remote_valid_coord(cur_lat, cur_lon)) {
            s_remote_lat_a   = cur_lat;
            s_remote_lon_a   = cur_lon;
            s_remote_a_valid = true;
        }
        a_valid = s_remote_a_valid;
        lat_a   = s_remote_lat_a;
        lon_a   = s_remote_lon_a;
        lat_b   = s_remote_lat_b;
        lon_b   = s_remote_lon_b;
        strncpy(label_a, s_remote_label_a, sizeof(label_a) - 1);
        strncpy(label_b, s_remote_label_b, sizeof(label_b) - 1);
        strncpy(notes_b, s_remote_notes_b, sizeof(notes_b) - 1);
    }
    portEXIT_CRITICAL(&s_remote_mux);

    if (!active) return false;

    seg_out->is_direct_here = !is_line;
    seg_out->idx_a          = is_line ? -3 : -1;
    seg_out->idx_b          = -2;
    seg_out->lat_b          = lat_b;
    seg_out->lon_b          = lon_b;
    strncpy(seg_out->label_b, label_b, WAYPOINT_LABEL_LEN - 1);
    strncpy(seg_out->notes_b, notes_b, WAYPOINT_NOTES_LEN - 1);

    if (!a_valid) {
        return false;
    }

    seg_out->valid = true;
    seg_out->lat_a = lat_a;
    seg_out->lon_a = lon_a;
    strncpy(seg_out->label_a, is_line ? label_a : "HERE", WAYPOINT_LABEL_LEN - 1);
    return true;
}

extern "C" bool waypoint_remote_guidance_get_snapshot(bool *active_out,
                                                      bool *is_line_out,
                                                      double *lat_a_out,
                                                      double *lon_a_out,
                                                      char *label_a_out,
                                                      size_t label_a_sz,
                                                      double *lat_b_out,
                                                      double *lon_b_out,
                                                      char *label_b_out,
                                                      size_t label_b_sz)
{
    bool active = false;
    portENTER_CRITICAL(&s_remote_mux);
    active = s_remote_active;
    if (active_out)  *active_out  = s_remote_active;
    if (is_line_out) *is_line_out = s_remote_is_line;
    if (lat_a_out)   *lat_a_out   = s_remote_lat_a;
    if (lon_a_out)   *lon_a_out   = s_remote_lon_a;
    if (lat_b_out)   *lat_b_out   = s_remote_lat_b;
    if (lon_b_out)   *lon_b_out   = s_remote_lon_b;
    if (label_a_out && label_a_sz > 0U) {
        strncpy(label_a_out, s_remote_label_a, label_a_sz - 1U);
        label_a_out[label_a_sz - 1U] = '\0';
    }
    if (label_b_out && label_b_sz > 0U) {
        strncpy(label_b_out, s_remote_label_b, label_b_sz - 1U);
        label_b_out[label_b_sz - 1U] = '\0';
    }
    portEXIT_CRITICAL(&s_remote_mux);
    return active;
}

static waypoint_survey_result_cb_t s_survey_result_cb = NULL;
static char          s_remote_note[48]      = "";
static uint8_t       s_remote_duration_s    = 0;
static volatile bool s_survey_cancel_req    = false;
static volatile bool s_pending_abort_ready  = false;
static char          s_abort_reason[24]     = "";

extern "C" bool waypoint_select_by_filename(const char *filepath)
{
    if (!filepath || filepath[0] == '\0') return false;
    char norm[48];
    if (filepath[0] == '/') {
        snprintf(norm, sizeof(norm), "%s", filepath);
    } else {
        snprintf(norm, sizeof(norm), "/%s", filepath);
    }

    waypoint_scan_files();
    SemaphoreHandle_t mux = waypoint_fs_mux();
    if (mux) xSemaphoreTake(mux, portMAX_DELAY);
    bool exists = waypoint_primary_fs().exists(norm);
    if (mux) xSemaphoreGive(mux);
    if (!exists) return false;

    waypoint_remote_guidance_clear();
    waypoint_line_stakeout_reset_auto();
    waypoint_file_sync_active_idx(norm);
    strncpy(current_settings.active_waypoint_file, norm, sizeof(current_settings.active_waypoint_file) - 1);
    current_settings.active_waypoint_file[sizeof(current_settings.active_waypoint_file) - 1] = '\0';
    g_active_waypoint_idx = 0;
    current_settings.active_waypoint_index = 0;
    bool ok = waypoint_load_csv(current_settings.active_waypoint_file);
    mark_settings_dirty();
    return ok;
}

extern "C" void waypoint_survey_set_result_callback(waypoint_survey_result_cb_t cb)
{
    s_survey_result_cb = cb;
}

void waypoint_survey_ext_consume_note(char *buf, size_t sz)
{
    if (!buf || sz == 0U) return;
    portENTER_CRITICAL(&s_remote_mux);
    strncpy(buf, s_remote_note, sz - 1U);
    buf[sz - 1U] = '\0';
    s_remote_note[0] = '\0';
    portEXIT_CRITICAL(&s_remote_mux);
}

uint8_t waypoint_survey_ext_get_dur_s(uint8_t fallback_s)
{
    portENTER_CRITICAL(&s_remote_mux);
    uint8_t dur = (s_remote_duration_s > 0U) ? s_remote_duration_s : fallback_s;
    portEXIT_CRITICAL(&s_remote_mux);
    return dur;
}

bool waypoint_survey_ext_take_cancel(void)
{
    bool cancel = false;
    portENTER_CRITICAL(&s_remote_mux);
    if (s_survey_cancel_req) {
        cancel = true;
        s_survey_cancel_req   = false;
        s_remote_duration_s   = 0U;
        s_remote_note[0]      = '\0';
        s_pending_abort_ready = true;
    }
    portEXIT_CRITICAL(&s_remote_mux);
    return cancel;
}

void waypoint_survey_ext_clear_cancel(void)
{
    portENTER_CRITICAL(&s_remote_mux);
    s_survey_cancel_req   = false;
    s_pending_abort_ready = false;
    portEXIT_CRITICAL(&s_remote_mux);
}

void waypoint_survey_ext_reset_overrides(void)
{
    portENTER_CRITICAL(&s_remote_mux);
    s_survey_cancel_req = false;
    s_remote_duration_s = 0U;
    portEXIT_CRITICAL(&s_remote_mux);
}

void waypoint_survey_ext_queue_abort(const char *reason)
{
    portENTER_CRITICAL(&s_remote_mux);
    snprintf(s_abort_reason, sizeof(s_abort_reason), "%s", (reason && reason[0]) ? reason : "ABORT");
    s_pending_abort_ready = true;
    portEXIT_CRITICAL(&s_remote_mux);
}

void waypoint_survey_ext_flush_abort(void)
{
    char reason[24] = "";
    portENTER_CRITICAL(&s_remote_mux);
    if (s_pending_abort_ready) {
        strncpy(reason, s_abort_reason, sizeof(reason) - 1U);
        reason[sizeof(reason) - 1U] = '\0';
        s_pending_abort_ready = false;
    }
    portEXIT_CRITICAL(&s_remote_mux);
    if (reason[0] != '\0' && s_survey_result_cb) {
        s_survey_result_cb(false, reason, "", 0.0, 0.0, 0.0f, 0.0f, 0U);
    }
}

void waypoint_survey_ext_emit_result(bool ok,
                                     const char *reason,
                                     const char *label,
                                     double lat,
                                     double lon,
                                     float h_sigma_m,
                                     float max_tilt_deg,
                                     uint16_t samples)
{
    if (s_survey_result_cb) {
        s_survey_result_cb(ok, reason, label, lat, lon, h_sigma_m, max_tilt_deg, samples);
    }
}

extern "C" bool waypoint_survey_cancel(const char *reason)
{
    if (!waypoint_is_survey_in_progress()) return false;
    portENTER_CRITICAL(&s_remote_mux);
    snprintf(s_abort_reason, sizeof(s_abort_reason), "%s", (reason && reason[0]) ? reason : "CANCELLED");
    portEXIT_CRITICAL(&s_remote_mux);

    if (waypoint_survey_cancel_pending_internal()) {
        portENTER_CRITICAL(&s_remote_mux);
        s_remote_duration_s   = 0U;
        s_remote_note[0]      = '\0';
        s_pending_abort_ready = true;
        portEXIT_CRITICAL(&s_remote_mux);
        return true;
    }
    portENTER_CRITICAL(&s_remote_mux);
    s_survey_cancel_req = true;
    portEXIT_CRITICAL(&s_remote_mux);
    return true;
}

extern "C" bool waypoint_survey_start_remote(uint8_t duration_s, const char *prefix, const char *note)
{
    if (waypoint_survey_is_busy_internal()) return false;

    int16_t prefix_idx = -1;
    if (prefix && prefix[0] != '\0') {
        if (strcasecmp(prefix, "DERIVED") == 0 || strcasecmp(prefix, "<DERIVED>") == 0) {
            prefix_idx = 0;
        } else {
            const char *opts = waypoint_get_prefix_options();
            int16_t idx = 0;
            const char *p = opts;
            while (p && *p != '\0') {
                const char *nl = strchr(p, '\n');
                size_t seg_len = nl ? (size_t)(nl - p) : strlen(p);
                if (strlen(prefix) == seg_len && strncasecmp(prefix, p, seg_len) == 0) {
                    prefix_idx = idx;
                    break;
                }
                if (!nl) break;
                p = nl + 1;
                idx++;
            }
            if (prefix_idx < 0) return false;
        }
    }

    char clean_note[48] = "";
    if (note && note[0] != '\0') {
        size_t j = 0U;
        for (size_t i = 0U; note[i] != '\0' && j + 1U < sizeof(clean_note); ++i) {
            char ch = note[i];
            if ((unsigned char)ch < 0x20U || (unsigned char)ch > 0x7EU) ch = ' ';
            if (ch == ',') ch = ' ';
            clean_note[j++] = ch;
        }
        clean_note[j] = '\0';
    }

    portENTER_CRITICAL(&s_remote_mux);
    s_remote_duration_s = (duration_s > 60U) ? 60U : duration_s;
    strncpy(s_remote_note, clean_note, sizeof(s_remote_note) - 1U);
    s_remote_note[sizeof(s_remote_note) - 1U] = '\0';
    s_survey_cancel_req   = false;
    s_pending_abort_ready = false;
    portEXIT_CRITICAL(&s_remote_mux);

    if (!waypoint_survey_try_start_internal(prefix_idx)) {
        portENTER_CRITICAL(&s_remote_mux);
        s_remote_duration_s = 0U;
        s_remote_note[0]    = '\0';
        portEXIT_CRITICAL(&s_remote_mux);
        return false;
    }
    return true;
}
