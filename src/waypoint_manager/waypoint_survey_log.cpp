#include "waypoint_manager_internal.h"
#include "../../user_config.h"
#include <Field_Instrument_Core.h>
#include "../nav_engine/nav_engine.h"
#include <math.h>
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>

uint8_t  g_survey_prefix_idx        = 0;
uint16_t g_survey_auto_num          = 1;
uint8_t  g_survey_note_tag_idx      = 0;

static uint16_t s_cached_next_num = 1;
static char     s_cached_stem[WAYPOINT_LABEL_LEN] = "";

static const char *kPrefixes[] = {
    "<DERIVED>", "PIN", "PIPE", "MONUMENT", "STAKE", "CORNER", "FENCE", "TREE", "UTILITY", "WP"
};
static const char *kPrefixDropdownStr =
    "<DERIVED>\nPIN\nPIPE\nMONUMENT\nSTAKE\nCORNER\nFENCE\nTREE\nUTILITY\nWP";

static const char *kNoteTags[] = {
    "",
    "Found Iron Pipe",
    "Found Capped Rebar",
    "Found Stone Mon",
    "Found Mag Nail",
    "NO PIN - Search Pt",
    "NO PIN - Marked Spot",
    "Unknown Pin/Pipe",
    "Set Temp Flag",
    "Mag Anomaly Only",
    "Obstructed Sky",
    "Needs Verification"
};
static const size_t kNoteTagCount = sizeof(kNoteTags) / sizeof(kNoteTags[0]);
static const char *kNoteTagDropdownStr =
    "(No Tag)\nFound Iron Pipe\nFound Capped Rebar\nFound Stone Mon\n"
    "Found Mag Nail\nNO PIN - Search Pt\nNO PIN - Marked Spot\n"
    "Unknown Pin/Pipe\nSet Temp Flag\nMag Anomaly Only\n"
    "Obstructed Sky\nNeeds Verification";

extern "C" bool waypoint_is_derived_mode(void) {
    return (g_survey_prefix_idx == 0);
}

extern "C" void waypoint_invalidate_survey_cache(void) {
    s_cached_stem[0] = '\0';
}

// Multi-epoch Survey Averaging State (F-03 hardened)
static portMUX_TYPE  s_survey_mux              = portMUX_INITIALIZER_UNLOCKED;
static volatile bool survey_requested          = false;
static bool          survey_active             = false;
static uint32_t      survey_hard_start_ms      = 0;
static uint32_t      survey_window_start_ms    = 0;
static uint32_t      s_last_survey_gga_seq     = 0;
static uint16_t      survey_epoch_cnt          = 0;
static uint16_t      survey_skipped_fix_cnt    = 0;
static uint16_t      survey_skipped_tilt_cnt   = 0;
static GnssFixType_t survey_best_fix           = GNSS_FIX_NONE;
static char          s_survey_gate_reason[24]  = "";
static float         s_last_survey_h_sigma_m   = 0.0f;
static float         s_last_survey_max_tilt_deg = -1.0f;
static char          last_logged_label[WAYPOINT_LABEL_LEN] = "None";

typedef struct {
    double        lat;
    double        lon;
    double        height_m;
    GnssFixType_t fix;
    char          hdg_fix_str[12];
    float         h_acc_m;
    float         v_acc_m;
    float         hdop;
    uint8_t       sats;
    uint8_t       slave_sats;
    float         diff_age_s;
    uint16_t      base_id;
    float         heading_deg;
    float         pole_tilt_deg;
    float         max_tilt_deg;
    float         h_sigma_m;
    float         v_sigma_m;
    uint16_t      samples;
    uint16_t      gst_samples;
    uint16_t      skipped_fix;
    uint16_t      skipped_tilt;
    bool          untared_warn;
    bool          uncomp_warn;
} PendingSurveyRecord_t;

static PendingSurveyRecord_t s_pending_record       = {};
static volatile bool         s_pending_record_ready = false;

extern "C" const char* waypoint_get_prefix_options(void) {
    return kPrefixDropdownStr;
}

extern "C" const char* waypoint_get_note_tag_options(void) {
    return kNoteTagDropdownStr;
}

extern "C" const char* waypoint_get_note_tag_label(uint8_t idx) {
    return (idx < kNoteTagCount) ? kNoteTags[idx] : "";
}

static void get_current_survey_stem(char *stem_out, size_t max_len)
{
    char base[32];
    if (g_survey_prefix_idx == 0) {
        // <DERIVED> mode: active target label (fallback to "WP")
        if (g_waypoint_count > 0 && g_active_waypoint_idx >= 0 && g_active_waypoint_idx < g_waypoint_count) {
            const char *src = g_waypoints[g_active_waypoint_idx].label;
            size_t src_len = strlen(src);
            if (src_len <= 30) {
                strncpy(base, src, sizeof(base) - 1);
                base[sizeof(base) - 1] = '\0';
            } else {
                // Delimiter-aligned truncation between 20 and 30
                size_t cut = 30;
                for (size_t i = 30; i >= 20; i--) {
                    if (src[i] == '_' || src[i] == '-' || src[i] == ' ') {
                        cut = i;
                        break;
                    }
                }
                strncpy(base, src, cut);
                base[cut] = '\0';
                Serial.printf("[WAYPOINTS] Target label truncated to %u chars for _Lxxx suffix: '%s' -> '%s'\n",
                              (unsigned int)cut, src, base);
            }
        } else {
            // Fallback when no active waypoint is loaded
            strncpy(base, "WP", sizeof(base) - 1);
            base[sizeof(base) - 1] = '\0';
        }
    } else {
        // Standard prefix mode (e.g. PIN, PIPE, MONUMENT, etc.)
        size_t num_prefixes = sizeof(kPrefixes) / sizeof(kPrefixes[0]);
        uint8_t p_idx = (g_survey_prefix_idx < num_prefixes) ? g_survey_prefix_idx : 1;
        strncpy(base, kPrefixes[p_idx], sizeof(base) - 1);
        base[sizeof(base) - 1] = '\0';
    }

    // Defensive sanitation: strip any trailing underscore from base to prevent accidental "__L"
    size_t blen = strlen(base);
    while (blen > 0 && base[blen - 1] == '_') {
        base[--blen] = '\0';
    }

    // Universal rule: ALWAYS append "_L"
    snprintf(stem_out, max_len, "%s_L", base);
}

// Phase 5 In-Memory Fast Stem Number Cache (eliminates slow SD card scans on navigation taps)
#define MAX_STEM_CACHE 64
struct StemCacheEntry {
    char stem[WAYPOINT_LABEL_LEN];
    uint16_t next_num;
};
static StemCacheEntry s_stem_cache[MAX_STEM_CACHE];
static int s_stem_cache_cnt = 0;
static int s_stem_cache_evict_idx = 0;

static uint16_t find_next_available_suffix(const char *stem)
{
    // Fast path: Check RAM cache first (< 0.01 ms)
    for (int i = 0; i < s_stem_cache_cnt; i++) {
        if (strcmp(s_stem_cache[i].stem, stem) == 0) {
            return s_stem_cache[i].next_num;
        }
    }

    uint16_t max_num = 0;
    SemaphoreHandle_t mux = waypoint_fs_mux();
    if (mux) xSemaphoreTake(mux, portMAX_DELAY);
    fs::FS &fs = waypoint_primary_fs();
    if (fs.exists("/survey_logged_pins.csv")) {
        File f = fs.open("/survey_logged_pins.csv", "r");
        if (f) {
            size_t stem_len = strlen(stem);
            uint8_t rbuf[512];
            char line[384];
            size_t line_len = 0;
            bool skip_header = true;

            for (;;) {
                int n = f.read(rbuf, sizeof(rbuf));
                if (n <= 0 && line_len == 0) break;

                int limit = (n > 0) ? n : 1;
                for (int bi = 0; bi < limit; bi++) {
                    char ch = (n > 0) ? (char)rbuf[bi] : '\n';
                    if (ch == '\n') {
                        line[line_len] = '\0';
                        line_len = 0;

                        if (skip_header) {
                            skip_header = false;
                            continue;
                        }

                        // Column 0: timestamp, Column 1: firmware_ver, Column 2: label (0-indexed)
                        char *p = strchr(line, ',');
                        if (!p) continue;
                        p = strchr(p + 1, ',');
                        if (!p) continue;
                        p++; // Start of col 2 (label)

                        char *end_label = strchr(p, ',');
                        if (end_label) *end_label = '\0';

                        if (strncmp(p, stem, stem_len) == 0) {
                            const char *num_part = p + stem_len;
                            if (*num_part >= '0' && *num_part <= '9') {
                                uint16_t num = (uint16_t)atoi(num_part);
                                if (num > max_num) {
                                    max_num = num;
                                }
                            }
                        }
                    } else if (ch != '\r') {
                        if (line_len < sizeof(line) - 1) {
                            line[line_len++] = ch;
                        }
                    }
                }
                if (n <= 0) break;
            }
            f.close();
        }
    }
    if (mux) xSemaphoreGive(mux);

    uint16_t next = max_num + 1;
    int slot = 0;
    if (s_stem_cache_cnt < MAX_STEM_CACHE) {
        slot = s_stem_cache_cnt++;
    } else {
        slot = s_stem_cache_evict_idx;
        s_stem_cache_evict_idx = (s_stem_cache_evict_idx + 1) % MAX_STEM_CACHE;
    }
    strncpy(s_stem_cache[slot].stem, stem, WAYPOINT_LABEL_LEN - 1);
    s_stem_cache[slot].stem[WAYPOINT_LABEL_LEN - 1] = '\0';
    s_stem_cache[slot].next_num = next;
    return next;
}

extern "C" void waypoint_format_next_survey_label(char *buf, size_t max_len)
{
    char stem[WAYPOINT_LABEL_LEN];
    get_current_survey_stem(stem, sizeof(stem));

    if (strcmp(stem, s_cached_stem) != 0) {
        s_cached_next_num = find_next_available_suffix(stem);
        strncpy(s_cached_stem, stem, sizeof(s_cached_stem) - 1);
        s_cached_stem[sizeof(s_cached_stem) - 1] = '\0';
    }

    snprintf(buf, max_len, "%s%03u", stem, s_cached_next_num);
}

extern "C" void trigger_survey_waypoint_log(void)
{
    survey_requested = true;
}

static const char* fix_type_to_str(GnssFixType_t fix)
{
    switch (fix) {
        case GNSS_FIX_RTK_FIXED: return "RTK_FIX";
        case GNSS_FIX_RTK_FLOAT: return "RTK_FLT";
        case GNSS_FIX_DGPS:      return "DGPS";
        case GNSS_FIX_SINGLE:    return "SINGLE";
        default:                 return "NONE";
    }
}

static void save_survey_record_to_csv(const PendingSurveyRecord_t *rec)
{
    if (!rec) return;
    char label[WAYPOINT_LABEL_LEN];
    waypoint_format_next_survey_label(label, sizeof(label));

    SemaphoreHandle_t mux = waypoint_fs_mux();
    if (mux) xSemaphoreTake(mux, portMAX_DELAY);
    fs::FS &fs = waypoint_primary_fs();
    bool exists = fs.exists("/survey_logged_pins.csv");
    File f = fs.open("/survey_logged_pins.csv", FILE_APPEND);
    if (!f) {
        f = fs.open("/survey_logged_pins.csv", FILE_WRITE);
    }
    if (!f) {
        if (mux) xSemaphoreGive(mux);
        Serial.println("Failed to open /survey_logged_pins.csv");
        return;
    }

    // Standardized 19-column header: timestamp is Column 1, followed by firmware_ver
    if (!exists) {
        f.println("timestamp,firmware_ver,label,latitude,longitude,elevation,pos_fix,hdg_fix,h_acc_m,v_acc_m,hdop,sats,slave_sats,diff_age_s,base_id,true_heading_deg,pole_tilt_deg,samples,notes");
    }

    // Standardized ISO / MFS timestamp format matching route logger
    FicRtcTime_t lt;
    fic_rtc_get_local_datetime(&lt);
    uint16_t ms = (uint16_t)(millis() % 1000);
    char sign = (current_settings.minutesOffsetToUTC >= 0) ? '+' : '-';
    int abs_offset = abs(current_settings.minutesOffsetToUTC);

    char ts_buf[48];
    snprintf(ts_buf, sizeof(ts_buf), "%04u-%02u-%02u_%02u-%02u-%02u.%03u%c%03d",
             lt.year, lt.month, lt.day, lt.hour, lt.minute, lt.second, ms, sign, abs_offset);

    uint8_t tag_idx = (g_survey_note_tag_idx < kNoteTagCount) ? g_survey_note_tag_idx : 0;
    char base_note[96] = {0};

    WaypointRecord_t active_wp;
    bool is_derived = (g_survey_prefix_idx == 0);
    if (is_derived && waypoint_get_active(&active_wp)) {
        double dist_m = 0.0;
        float brg = 0.0f, rel_brg = 0.0f, dn = 0.0f, de = 0.0f;
        nav_compute_target_guidance(rec->lat, rec->lon, 0.0f, active_wp.latitude, active_wp.longitude,
                                    &dist_m, &brg, &rel_brg, &dn, &de);
        if (kNoteTags[tag_idx][0] != '\0') {
            snprintf(base_note, sizeof(base_note), "%s (dN:%+.2fm dE:%+.2fm)", kNoteTags[tag_idx], -dn, -de);
        } else {
            snprintf(base_note, sizeof(base_note), "dN:%+.2fm dE:%+.2fm from target", -dn, -de);
        }
    } else {
        snprintf(base_note, sizeof(base_note), "%s", kNoteTags[tag_idx]);
    }

    char qual_note[88];
    snprintf(qual_note, sizeof(qual_note),
             "[hSig:%.3fm vSig:%.3fm maxTilt:%.1fd %s skip:%u/%u%s%s]",
             rec->h_sigma_m,
             rec->v_sigma_m,
             rec->max_tilt_deg,
             (rec->gst_samples > 0U) ? "GST" : "NOM",
             (unsigned int)rec->skipped_fix,
             (unsigned int)rec->skipped_tilt,
             rec->untared_warn ? " UNTARED" : "",
             rec->uncomp_warn ? " UNCOMP" : "");

    char note_buf[184];
    if (base_note[0] != '\0') {
        snprintf(note_buf, sizeof(note_buf), "%s %s", base_note, qual_note);
    } else {
        snprintf(note_buf, sizeof(note_buf), "%s", qual_note);
    }

    char row[420];
    snprintf(row, sizeof(row),
             "%s,%s,%s,%.9f,%.9f,%.3f,%s,%s,%.3f,%.3f,%.1f,%u,%u,%.1f,%u,%.2f,%.2f,%u,%s",
             ts_buf,
             ROVER_FIRMWARE_VERSION,
             label,
             rec->lat, rec->lon, rec->height_m,
             fix_type_to_str(rec->fix),
             rec->hdg_fix_str[0] ? rec->hdg_fix_str : "NONE",
             rec->h_acc_m, rec->v_acc_m, rec->hdop,
             (unsigned int)rec->sats, (unsigned int)rec->slave_sats,
             rec->diff_age_s, (unsigned int)rec->base_id,
             rec->heading_deg, rec->pole_tilt_deg,
             (unsigned int)rec->samples,
             note_buf);
    f.println(row);
    f.close();
    if (mux) xSemaphoreGive(mux);

    portENTER_CRITICAL(&s_survey_mux);
    strncpy(last_logged_label, label, sizeof(last_logged_label) - 1);
    last_logged_label[sizeof(last_logged_label) - 1] = '\0';
    portEXIT_CRITICAL(&s_survey_mux);

    char stem_saved[WAYPOINT_LABEL_LEN];
    get_current_survey_stem(stem_saved, sizeof(stem_saved));
    for (int i = 0; i < s_stem_cache_cnt; i++) {
        if (strcmp(s_stem_cache[i].stem, stem_saved) == 0) {
            s_stem_cache[i].next_num++;
            break;
        }
    }

    waypoint_invalidate_survey_cache();
    Serial.printf("SURVEY LOGGED: %s\n", row);
}

// Survey averaging Welford & quality accumulation variables
static double   s_ref_lat              = 0.0;
static double   s_ref_lon              = 0.0;
static double   s_ref_alt              = 0.0;
static double   s_r_m                  = 6378137.0;
static double   s_r_n                  = 6378137.0;
static double   s_cos_ref_lat          = 1.0;
static double   s_mean_n               = 0.0;
static double   s_mean_e               = 0.0;
static double   s_mean_u               = 0.0;
static double   s_m2_n                 = 0.0;
static double   s_m2_e                 = 0.0;
static double   s_m2_u                 = 0.0;
static float    survey_sum_hacc_all    = 0.0f;
static float    survey_sum_vacc_all    = 0.0f;
static float    survey_sum_hacc_gst    = 0.0f;
static float    survey_sum_vacc_gst    = 0.0f;
static uint16_t survey_gst_cnt         = 0;
static float    survey_sum_hdop        = 0.0f;
static float    survey_sum_tilt        = 0.0f;
static float    survey_max_tilt        = 0.0f;
static bool     survey_untared_warn    = false;
static uint16_t survey_lever_ok_cnt    = 0;
static double   survey_sum_hdg_sin     = 0.0;
static double   survey_sum_hdg_cos     = 0.0;
static uint16_t survey_hdg_valid_cnt   = 0;
static uint16_t survey_hdg_fix_cnt     = 0;
static uint16_t survey_hdg_flt_cnt     = 0;
static uint16_t survey_hdg_cmp_cnt     = 0;
static uint16_t survey_hdg_other_cnt   = 0;
static uint8_t  survey_max_sats        = 0;
static uint8_t  survey_max_slave_sats  = 0;
static float    survey_last_diff_age   = 0.0f;
static uint16_t survey_base_id         = 0;

static uint8_t fix_quality_rank(GnssFixType_t fix)
{
    switch (fix) {
        case GNSS_FIX_RTK_FIXED: return 4U;
        case GNSS_FIX_RTK_FLOAT: return 3U;
        case GNSS_FIX_DGPS:
        case GNSS_FIX_PPS:       return 2U;
        case GNSS_FIX_SINGLE:    return 1U;
        default:                 return 0U;
    }
}

static void reset_survey_accumulators(void)
{
    s_ref_lat              = 0.0;
    s_ref_lon              = 0.0;
    s_ref_alt              = 0.0;
    s_mean_n               = 0.0;
    s_mean_e               = 0.0;
    s_mean_u               = 0.0;
    s_m2_n                 = 0.0;
    s_m2_e                 = 0.0;
    s_m2_u                 = 0.0;
    survey_epoch_cnt       = 0;
    survey_sum_hacc_all    = 0.0f;
    survey_sum_vacc_all    = 0.0f;
    survey_sum_hacc_gst    = 0.0f;
    survey_sum_vacc_gst    = 0.0f;
    survey_gst_cnt         = 0;
    survey_sum_hdop        = 0.0f;
    survey_sum_tilt        = 0.0f;
    survey_max_tilt        = 0.0f;
    survey_untared_warn    = false;
    survey_lever_ok_cnt    = 0;
    survey_sum_hdg_sin     = 0.0;
    survey_sum_hdg_cos     = 0.0;
    survey_hdg_valid_cnt   = 0;
    survey_hdg_fix_cnt     = 0;
    survey_hdg_flt_cnt     = 0;
    survey_hdg_cmp_cnt     = 0;
    survey_hdg_other_cnt   = 0;
    survey_max_sats        = 0;
    survey_max_slave_sats  = 0;
    survey_last_diff_age   = 0.0f;
    survey_base_id         = 0;
}

extern "C" bool waypoint_is_survey_in_progress(void)
{
    return survey_active || survey_requested;
}

extern "C" void waypoint_flush_pending_survey_csv(void)
{
    if (!s_pending_record_ready) return;
    PendingSurveyRecord_t copy;
    portENTER_CRITICAL(&s_survey_mux);
    if (!s_pending_record_ready) {
        portEXIT_CRITICAL(&s_survey_mux);
        return;
    }
    copy = s_pending_record;
    s_pending_record_ready = false;
    portEXIT_CRITICAL(&s_survey_mux);
    save_survey_record_to_csv(&copy);
}

extern "C" void waypoint_get_survey_quality_snapshot(uint16_t *skipped_out,
                                                   char *gate_reason_out,
                                                   size_t gate_reason_sz,
                                                   float *last_h_sigma_out,
                                                   float *last_max_tilt_out)
{
    portENTER_CRITICAL(&s_survey_mux);
    if (skipped_out) *skipped_out = (uint16_t)(survey_skipped_fix_cnt + survey_skipped_tilt_cnt);
    if (gate_reason_out && gate_reason_sz > 0U) {
        strncpy(gate_reason_out, s_survey_gate_reason, gate_reason_sz - 1U);
        gate_reason_out[gate_reason_sz - 1U] = '\0';
    }
    if (last_h_sigma_out) *last_h_sigma_out = s_last_survey_h_sigma_m;
    if (last_max_tilt_out) *last_max_tilt_out = s_last_survey_max_tilt_deg;
    portEXIT_CRITICAL(&s_survey_mux);
}

extern "C" void waypoint_update_survey_averaging(const RoverUIData_t *live_state,
                                                 bool *in_progress_out,
                                                 uint8_t *progress_pct_out,
                                                 uint16_t *epochs_out,
                                                 char *last_logged_out)
{
    if (!live_state) return;

    if (survey_requested && !survey_active && !s_pending_record_ready) {
        uint32_t now_ms        = millis();
        survey_requested       = false;
        survey_active          = true;
        survey_hard_start_ms   = now_ms;
        survey_window_start_ms = now_ms;
        s_last_survey_gga_seq  = (live_state->gga_seq > 0U) ? (live_state->gga_seq - 1U) : 0U;
        survey_best_fix        = GNSS_FIX_NONE;
        reset_survey_accumulators();
        portENTER_CRITICAL(&s_survey_mux);
        survey_skipped_fix_cnt  = 0;
        survey_skipped_tilt_cnt = 0;
        snprintf(s_survey_gate_reason, sizeof(s_survey_gate_reason), "WAIT GGA");
        portEXIT_CRITICAL(&s_survey_mux);
        play_rover_beep(1200, 80);
    }

    if (survey_active) {
        uint32_t now_ms          = millis();
        uint32_t dur_ms          = (uint32_t)current_settings.survey_avg_seconds * 1000UL;
        if (dur_ms < 500UL) dur_ms = 500UL;
        uint32_t hard_timeout_ms = dur_ms + 15000UL;
        char     gate_reason[24];
        portENTER_CRITICAL(&s_survey_mux);
        strncpy(gate_reason, s_survey_gate_reason, sizeof(gate_reason) - 1U);
        gate_reason[sizeof(gate_reason) - 1U] = '\0';
        portEXIT_CRITICAL(&s_survey_mux);

        if (live_state->gga_seq != 0U && live_state->gga_seq != s_last_survey_gga_seq) {
            s_last_survey_gga_seq = live_state->gga_seq;
            uint8_t cur_rank      = fix_quality_rank(live_state->fix_type);

            if (cur_rank == 0U || fabs(live_state->tip_lat) < 0.0001) {
                survey_skipped_fix_cnt++;
                snprintf(gate_reason, sizeof(gate_reason), "NO FIX");
            } else {
                uint8_t best_rank = fix_quality_rank(survey_best_fix);
                if (cur_rank > best_rank) {
                    survey_best_fix        = live_state->fix_type;
                    reset_survey_accumulators();
                    survey_window_start_ms = now_ms;
                }

                if (cur_rank < fix_quality_rank(survey_best_fix)) {
                    survey_skipped_fix_cnt++;
                    snprintf(gate_reason, sizeof(gate_reason), "FIX DIP");
                } else {
                    float pole_tilt = live_state->pole_tilt_deg;
                    if (!isfinite(pole_tilt)) {
                        pole_tilt = sqrtf(live_state->pole_bubble_x_deg * live_state->pole_bubble_x_deg +
                                          live_state->pole_bubble_y_deg * live_state->pole_bubble_y_deg);
                    }
                    bool tilt_qualified = live_state->beam_10dof_valid ||
                                          (fabsf(current_settings.pole_tilt_tare_pitch_deg) > 0.05f) ||
                                          (fabsf(current_settings.pole_tilt_tare_roll_deg)  > 0.05f);
                    float max_tilt_gate = current_settings.survey_max_tilt_deg;
                    if (max_tilt_gate > 0.05f && tilt_qualified && pole_tilt > max_tilt_gate) {
                        survey_skipped_tilt_cnt++;
                        snprintf(gate_reason, sizeof(gate_reason), "TILT %.1fd!", pole_tilt);
                    } else {
                        gate_reason[0] = '\0';
                        if (survey_epoch_cnt == 0U) {
                            const double a  = 6378137.0;
                            const double e2 = 0.00669437999014;
                            s_ref_lat     = live_state->tip_lat;
                            s_ref_lon     = live_state->tip_lon;
                            s_ref_alt     = live_state->tip_height_m;
                            double lat_r  = s_ref_lat * (M_PI / 180.0);
                            double sin_l  = sin(lat_r);
                            double denom  = sqrt(1.0 - e2 * sin_l * sin_l);
                            s_r_m         = a * (1.0 - e2) / (denom * denom * denom);
                            s_r_n         = a / denom;
                            s_cos_ref_lat = cos(lat_r);
                            if (fabs(s_cos_ref_lat) < 1e-6) s_cos_ref_lat = 1e-6;
                        }

                        double dn = (live_state->tip_lat - s_ref_lat) * (M_PI / 180.0) * s_r_m;
                        double de = (live_state->tip_lon - s_ref_lon) * (M_PI / 180.0) * s_r_n * s_cos_ref_lat;
                        double du = live_state->tip_height_m - s_ref_alt;
                        survey_epoch_cnt++;
                        double inv_n = 1.0 / (double)survey_epoch_cnt;
                        double d_n1  = dn - s_mean_n;
                        s_mean_n    += d_n1 * inv_n;
                        s_m2_n      += d_n1 * (dn - s_mean_n);
                        double d_e1  = de - s_mean_e;
                        s_mean_e    += d_e1 * inv_n;
                        s_m2_e      += d_e1 * (de - s_mean_e);
                        double d_u1  = du - s_mean_u;
                        s_mean_u    += d_u1 * inv_n;
                        s_m2_u      += d_u1 * (du - s_mean_u);

                        survey_sum_hacc_all += live_state->h_acc_m;
                        survey_sum_vacc_all += live_state->v_acc_m;
                        if (live_state->h_acc_from_gst) {
                            survey_sum_hacc_gst += live_state->h_acc_m;
                            survey_sum_vacc_gst += live_state->v_acc_m;
                            survey_gst_cnt++;
                        }
                        survey_sum_hdop += live_state->hdop;
                        survey_sum_tilt += pole_tilt;
                        if (pole_tilt > survey_max_tilt) survey_max_tilt = pole_tilt;
                        if (!tilt_qualified && pole_tilt > 2.5f) survey_untared_warn = true;
                        if (live_state->lever_arm_applied) survey_lever_ok_cnt++;

                        if (live_state->heading_valid) {
                            double h_rad = (double)live_state->true_heading_deg * (M_PI / 180.0);
                            survey_sum_hdg_sin += sin(h_rad);
                            survey_sum_hdg_cos += cos(h_rad);
                            survey_hdg_valid_cnt++;
                            if (live_state->heading_source == HDG_SRC_DUAL_FIX)      survey_hdg_fix_cnt++;
                            else if (live_state->heading_source == HDG_SRC_DUAL_FLT) survey_hdg_flt_cnt++;
                            else if (live_state->heading_source == HDG_SRC_COMP)     survey_hdg_cmp_cnt++;
                            else                                                     survey_hdg_other_cnt++;
                        }
                        if (live_state->num_sats > survey_max_sats)             survey_max_sats = live_state->num_sats;
                        if (live_state->slave_num_sats > survey_max_slave_sats) survey_max_slave_sats = live_state->slave_num_sats;
                        survey_last_diff_age = live_state->diff_age_s;
                        if (live_state->base_station_id > 0U) survey_base_id = live_state->base_station_id;
                    }
                }
            }
            portENTER_CRITICAL(&s_survey_mux);
            strncpy(s_survey_gate_reason, gate_reason, sizeof(s_survey_gate_reason) - 1U);
            s_survey_gate_reason[sizeof(s_survey_gate_reason) - 1U] = '\0';
            portEXIT_CRITICAL(&s_survey_mux);
        }

        uint32_t win_elapsed  = now_ms - survey_window_start_ms;
        uint32_t hard_elapsed = now_ms - survey_hard_start_ms;
        uint8_t  pct          = (win_elapsed >= dur_ms) ? 100U : (uint8_t)((win_elapsed * 100UL) / dur_ms);

        bool ready_clean   = (win_elapsed >= dur_ms && survey_epoch_cnt > 0U && gate_reason[0] == '\0');
        bool hit_hard_done = (hard_elapsed >= hard_timeout_ms && survey_epoch_cnt > 0U);
        if (ready_clean || hit_hard_done) {
            double avg_lat  = s_ref_lat + (s_mean_n / s_r_m) * (180.0 / M_PI);
            double avg_lon  = s_ref_lon + (s_mean_e / (s_r_n * s_cos_ref_lat)) * (180.0 / M_PI);
            double avg_alt  = s_ref_alt + s_mean_u;
            float  h_sigma  = (survey_epoch_cnt >= 2U)
                              ? (float)sqrt((s_m2_n + s_m2_e) / (double)(survey_epoch_cnt - 1U))
                              : 0.0f;
            float  v_sigma  = (survey_epoch_cnt >= 2U)
                              ? (float)sqrt(s_m2_u / (double)(survey_epoch_cnt - 1U))
                              : 0.0f;
            float  avg_hacc = (survey_gst_cnt > 0U)
                              ? (survey_sum_hacc_gst / (float)survey_gst_cnt)
                              : (survey_sum_hacc_all / (float)survey_epoch_cnt);
            float  avg_vacc = (survey_gst_cnt > 0U)
                              ? (survey_sum_vacc_gst / (float)survey_gst_cnt)
                              : (survey_sum_vacc_all / (float)survey_epoch_cnt);
            float  avg_hdop = survey_sum_hdop / (float)survey_epoch_cnt;
            float  avg_tilt = survey_sum_tilt / (float)survey_epoch_cnt;
            float  avg_hdg  = -1.0f;
            if (survey_hdg_valid_cnt > 0U) {
                if (fabs(survey_sum_hdg_sin) < 1e-9 && fabs(survey_sum_hdg_cos) < 1e-9) {
                    avg_hdg = 0.0f;
                } else {
                    double h_deg = atan2(survey_sum_hdg_sin, survey_sum_hdg_cos) * (180.0 / M_PI);
                    if (h_deg < 0.0) h_deg += 360.0;
                    avg_hdg = (float)h_deg;
                }
            }
            const char *hdg_fix_str =
                (survey_hdg_fix_cnt > (survey_epoch_cnt / 2U)) ? "HDG_FIX" :
                ((survey_hdg_fix_cnt + survey_hdg_flt_cnt) > 0U) ? "HDG_FLT" :
                (survey_hdg_cmp_cnt > 0U) ? "HDG_CMP" :
                (survey_hdg_other_cnt > 0U) ? "HDG_EST" : "NONE";

            PendingSurveyRecord_t rec = {};
            rec.lat           = avg_lat;
            rec.lon           = avg_lon;
            rec.height_m      = avg_alt;
            rec.fix           = survey_best_fix;
            strncpy(rec.hdg_fix_str, hdg_fix_str, sizeof(rec.hdg_fix_str) - 1U);
            rec.h_acc_m       = avg_hacc;
            rec.v_acc_m       = avg_vacc;
            rec.hdop          = avg_hdop;
            rec.sats          = survey_max_sats;
            rec.slave_sats    = survey_max_slave_sats;
            rec.diff_age_s    = survey_last_diff_age;
            rec.base_id       = survey_base_id;
            rec.heading_deg   = avg_hdg;
            rec.pole_tilt_deg = avg_tilt;
            rec.max_tilt_deg  = survey_max_tilt;
            rec.h_sigma_m     = h_sigma;
            rec.v_sigma_m     = v_sigma;
            rec.samples       = survey_epoch_cnt;
            rec.gst_samples   = survey_gst_cnt;
            rec.skipped_fix   = survey_skipped_fix_cnt;
            rec.skipped_tilt  = survey_skipped_tilt_cnt;
            rec.untared_warn  = survey_untared_warn;
            rec.uncomp_warn   = (survey_lever_ok_cnt == 0U && current_settings.apply_lever_arm);

            portENTER_CRITICAL(&s_survey_mux);
            s_pending_record          = rec;
            s_pending_record_ready    = true;
            s_last_survey_h_sigma_m   = h_sigma;
            s_last_survey_max_tilt_deg = survey_max_tilt;
            s_survey_gate_reason[0]   = '\0';
            portEXIT_CRITICAL(&s_survey_mux);

            survey_active = false;
            play_rover_beep(2200, 160);
        } else if (hard_elapsed >= hard_timeout_ms && survey_epoch_cnt == 0U) {
            survey_active = false;
            portENTER_CRITICAL(&s_survey_mux);
            snprintf(s_survey_gate_reason, sizeof(s_survey_gate_reason), "%s",
                     (survey_skipped_tilt_cnt > 0U) ? "ABORT: TILT" : "ABORT: NO FIX");
            portEXIT_CRITICAL(&s_survey_mux);
            play_rover_dual_beep(400, 120, 40, 300, 180);
            Serial.printf("[SURVEY ABORT] %s (skipFix=%u skipTilt=%u)\n",
                          s_survey_gate_reason,
                          (unsigned)survey_skipped_fix_cnt,
                          (unsigned)survey_skipped_tilt_cnt);
        }

        if (in_progress_out)  *in_progress_out  = survey_active;
        if (progress_pct_out) *progress_pct_out = pct;
        if (epochs_out)       *epochs_out       = survey_epoch_cnt;
    } else {
        if (in_progress_out)  *in_progress_out  = false;
        if (progress_pct_out) *progress_pct_out = 0;
        if (epochs_out)       *epochs_out       = survey_epoch_cnt;
    }

    if (last_logged_out) {
        portENTER_CRITICAL(&s_survey_mux);
        strncpy(last_logged_out, last_logged_label, WAYPOINT_LABEL_LEN - 1);
        last_logged_out[WAYPOINT_LABEL_LEN - 1] = '\0';
        portEXIT_CRITICAL(&s_survey_mux);
    }
}
