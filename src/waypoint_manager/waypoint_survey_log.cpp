#include "waypoint_manager_internal.h"
#include "../../user_config.h"
#include "../rtc_bsp/rtc_bsp.h"
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

// Multi-epoch Survey Averaging State
static volatile bool survey_requested   = false;
static bool          survey_active      = false;
static uint32_t      survey_start_ms    = 0;
static double        survey_sum_lat     = 0.0;
static double        survey_sum_lon     = 0.0;
static double        survey_sum_alt     = 0.0;
static float         survey_sum_hacc    = 0.0f;
static uint16_t      survey_epoch_cnt   = 0;
static GnssFixType_t survey_best_fix    = GNSS_FIX_NONE;
static char          last_logged_label[WAYPOINT_LABEL_LEN] = "None";

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

static void save_survey_record_to_csv(double lat, double lon, double height_m,
                                      GnssFixType_t fix, const char *hdg_fix_str,
                                      float h_acc_m, float v_acc_m, float hdop,
                                      uint8_t sats, uint8_t slave_sats,
                                      float diff_age_s, uint16_t base_id,
                                      float heading_deg, float pole_tilt_deg,
                                      uint16_t samples)
{
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

    // Standardized header: timestamp is Column 1, followed by firmware_ver
    if (!exists) {
        f.println("timestamp,firmware_ver,label,latitude,longitude,elevation,pos_fix,hdg_fix,h_acc_m,v_acc_m,hdop,sats,slave_sats,diff_age_s,base_id,true_heading_deg,pole_tilt_deg,samples,notes");
    }

    // Standardized ISO / MFS timestamp format matching route logger
    RtcDateTime_t lt;
    get_local_datetime(&lt);
    uint16_t ms = (uint16_t)(millis() % 1000);
    char sign = (current_settings.minutesOffsetToUTC >= 0) ? '+' : '-';
    int abs_offset = abs(current_settings.minutesOffsetToUTC);

    char ts_buf[48];
    snprintf(ts_buf, sizeof(ts_buf), "%04u-%02u-%02u_%02u-%02u-%02u.%03u%c%03d",
             lt.year, lt.month, lt.day, lt.hour, lt.minute, lt.second, ms, sign, abs_offset);

    uint8_t tag_idx = (g_survey_note_tag_idx < kNoteTagCount) ? g_survey_note_tag_idx : 0;
    char note_buf[96] = {0};

    WaypointRecord_t active_wp;
    bool is_derived = (g_survey_prefix_idx == 0);
    if (is_derived && waypoint_get_active(&active_wp)) {
        double dist_m = 0.0;
        float brg = 0.0f, rel_brg = 0.0f, dn = 0.0f, de = 0.0f;
        nav_compute_target_guidance(lat, lon, 0.0f, active_wp.latitude, active_wp.longitude,
                                    &dist_m, &brg, &rel_brg, &dn, &de);
        if (kNoteTags[tag_idx][0] != '\0') {
            snprintf(note_buf, sizeof(note_buf), "%s (dN:%+.2fm dE:%+.2fm)", kNoteTags[tag_idx], -dn, -de);
        } else {
            snprintf(note_buf, sizeof(note_buf), "dN:%+.2fm dE:%+.2fm from target", -dn, -de);
        }
    } else {
        snprintf(note_buf, sizeof(note_buf), "%s", kNoteTags[tag_idx]);
    }

    char row[320];
    snprintf(row, sizeof(row),
             "%s,%s,%s,%.9f,%.9f,%.3f,%s,%s,%.3f,%.3f,%.1f,%u,%u,%.1f,%u,%.2f,%.2f,%u,%s",
             ts_buf,
             ROVER_FIRMWARE_VERSION,
             label,
             lat, lon, height_m,
             fix_type_to_str(fix),
             hdg_fix_str ? hdg_fix_str : "NONE",
             h_acc_m, v_acc_m, hdop,
             (unsigned int)sats, (unsigned int)slave_sats,
             diff_age_s, (unsigned int)base_id,
             heading_deg, pole_tilt_deg,
             samples,
             note_buf);
    f.println(row);
    f.close();
    if (mux) xSemaphoreGive(mux);

    strncpy(last_logged_label, label, sizeof(last_logged_label) - 1);
    last_logged_label[sizeof(last_logged_label) - 1] = '\0';

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

// Survey averaging accumulation variables
static float          survey_sum_vacc       = 0.0f;
static float          survey_sum_hdop       = 0.0f;
static float          survey_sum_tilt       = 0.0f;
static double         survey_sum_hdg_sin    = 0.0;
static double         survey_sum_hdg_cos    = 0.0;
static uint16_t       survey_hdg_valid_cnt  = 0;
static uint16_t       survey_hdg_fix_cnt    = 0;
static uint8_t        survey_max_sats       = 0;
static uint8_t        survey_max_slave_sats = 0;
static float          survey_last_diff_age  = 0.0f;
static uint16_t       survey_base_id        = 0;

extern "C" void waypoint_update_survey_averaging(const RoverUIData_t *live_state,
                                                 bool *in_progress_out,
                                                 uint8_t *progress_pct_out,
                                                 uint16_t *epochs_out,
                                                 char *last_logged_out)
{
    if (survey_requested && !survey_active) {
        survey_requested      = false;
        survey_active         = true;
        survey_start_ms       = millis();
        survey_sum_lat        = 0.0;
        survey_sum_lon        = 0.0;
        survey_sum_alt        = 0.0;
        survey_sum_hacc       = 0.0f;
        survey_sum_vacc       = 0.0f;
        survey_sum_hdop       = 0.0f;
        survey_sum_tilt       = 0.0f;
        survey_sum_hdg_sin    = 0.0;
        survey_sum_hdg_cos    = 0.0;
        survey_hdg_valid_cnt  = 0;
        survey_hdg_fix_cnt    = 0;
        survey_max_sats       = 0;
        survey_max_slave_sats = 0;
        survey_last_diff_age  = 0.0f;
        survey_base_id        = 0;
        survey_epoch_cnt      = 0;
        survey_best_fix       = live_state->fix_type;
        play_rover_beep(1200, 80);
    }

    if (survey_active) {
        survey_sum_lat  += live_state->tip_lat;
        survey_sum_lon  += live_state->tip_lon;
        survey_sum_alt  += live_state->tip_height_m;
        survey_sum_hacc += live_state->h_acc_m;
        survey_sum_vacc += live_state->v_acc_m;
        survey_sum_hdop += live_state->hdop;

        float pole_tilt = sqrtf(live_state->pole_bubble_x_deg * live_state->pole_bubble_x_deg +
                                live_state->pole_bubble_y_deg * live_state->pole_bubble_y_deg);
        survey_sum_tilt += pole_tilt;

        if (live_state->heading_valid) {
            double h_rad = (double)live_state->true_heading_deg * (M_PI / 180.0);
            survey_sum_hdg_sin += sin(h_rad);
            survey_sum_hdg_cos += cos(h_rad);
            survey_hdg_valid_cnt++;
            if (live_state->heading_is_fixed) {
                survey_hdg_fix_cnt++;
            }
        }
        if (live_state->num_sats > survey_max_sats) {
            survey_max_sats = live_state->num_sats;
        }
        if (live_state->slave_num_sats > survey_max_slave_sats) {
            survey_max_slave_sats = live_state->slave_num_sats;
        }
        survey_last_diff_age = live_state->diff_age_s;
        if (live_state->base_station_id > 0) {
            survey_base_id = live_state->base_station_id;
        }

        survey_epoch_cnt++;
        if (live_state->fix_type == GNSS_FIX_RTK_FIXED) {
            survey_best_fix = GNSS_FIX_RTK_FIXED;
        }

        uint32_t dur_ms = (uint32_t)current_settings.survey_avg_seconds * 1000UL;
        if (dur_ms < 500) dur_ms = 500;
        uint32_t elapsed = millis() - survey_start_ms;
        uint8_t pct = (elapsed >= dur_ms) ? 100 : (uint8_t)((elapsed * 100UL) / dur_ms);

        if (elapsed >= dur_ms && survey_epoch_cnt > 0) {
            double avg_lat  = survey_sum_lat / (double)survey_epoch_cnt;
            double avg_lon  = survey_sum_lon / (double)survey_epoch_cnt;
            double avg_alt  = survey_sum_alt / (double)survey_epoch_cnt;
            float  avg_hacc = survey_sum_hacc / (float)survey_epoch_cnt;
            float  avg_vacc = survey_sum_vacc / (float)survey_epoch_cnt;
            float  avg_hdop = survey_sum_hdop / (float)survey_epoch_cnt;
            float  avg_tilt = survey_sum_tilt / (float)survey_epoch_cnt;
            float  avg_hdg  = -1.0f;
            if (survey_hdg_valid_cnt > 0) {
                if (fabs(survey_sum_hdg_sin) < 1e-9 && fabs(survey_sum_hdg_cos) < 1e-9) {
                    avg_hdg = 0.0f;
                } else {
                    double h_deg = atan2(survey_sum_hdg_sin, survey_sum_hdg_cos) * (180.0 / M_PI);
                    if (h_deg < 0.0) h_deg += 360.0;
                    avg_hdg = (float)h_deg;
                }
            }
            const char *hdg_fix_str = (survey_hdg_fix_cnt > (survey_epoch_cnt / 2)) ? "HDG_FIX" :
                                      ((survey_hdg_valid_cnt > 0) ? "HDG_FLT" : "NONE");

            save_survey_record_to_csv(avg_lat, avg_lon, avg_alt, survey_best_fix, hdg_fix_str,
                                      avg_hacc, avg_vacc, avg_hdop,
                                      survey_max_sats, survey_max_slave_sats,
                                      survey_last_diff_age, survey_base_id,
                                      avg_hdg, avg_tilt, survey_epoch_cnt);
            survey_active = false;
            play_rover_beep(2200, 160);
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
        strncpy(last_logged_out, last_logged_label, WAYPOINT_LABEL_LEN - 1);
        last_logged_out[WAYPOINT_LABEL_LEN - 1] = '\0';
    }
}
