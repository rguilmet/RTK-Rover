#include "waypoint_manager.h"
#include "../settings_manager/settings_manager.h"
#include "../rtc_bsp/rtc_bsp.h"
#include "../nav_engine/nav_engine.h"
#include <Arduino.h>
#include <SD_MMC.h>
#include <FFat.h>
#include <string.h>
#include <stdlib.h>

WaypointRecord_t g_waypoints[MAX_WAYPOINTS];
int16_t          g_waypoint_count      = 0;
int16_t          g_active_waypoint_idx = 0;

bool     g_survey_target_match_mode = true;
uint8_t  g_survey_prefix_idx        = 0;
uint16_t g_survey_auto_num          = 1;
uint8_t  g_survey_note_tag_idx      = 0;

static const char *kPrefixes[] = {
    "PIN_", "CORNER_", "MONUMENT_", "STAKE_", "FENCE_", "TREE_", "UTIL_", "WP_"
};
static const char *kPrefixDropdownStr =
    "PIN_\nCORNER_\nMONUMENT_\nSTAKE_\nFENCE_\nTREE_\nUTIL_\nWP_";

static const char *kNoteTags[] = {
    "",
    "Found Iron Pipe",
    "Found Rebar",
    "Found Stone Bound",
    "Set Temp Flag",
    "Mag Anomaly Only",
    "Obstructed Sky",
    "Needs Verification"
};
static const char *kNoteTagDropdownStr =
    "(No Tag)\nFound Iron Pipe\nFound Rebar\nFound Stone Bound\nSet Temp Flag\nMag Anomaly Only\nObstructed Sky\nNeeds Verification";

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

static void create_default_sample_waypoints_if_missing(void)
{
    if (get_active_fs().exists("/waypoints.csv")) {
        return;
    }
    File f = get_active_fs().open("/waypoints.csv", "w");
    if (!f) return;

    f.println("label,latitude,longitude,elevation,notes");
    f.println("PIN_NW_01,42.36010000,-71.09420000,12.500,NW corner iron pipe near oak");
    f.println("PIN_NE_02,42.36045000,-71.09350000,13.100,NE corner rebar by stone wall");
    f.println("PIN_SE_03,42.35980000,-71.09340000,11.800,SE corner monument");
    f.println("PIN_SW_04,42.35975000,-71.09415000,12.000,SW corner pin by driveway");
    f.close();
    Serial.println("Created sample /waypoints.csv with 4 property pins.");
}

extern "C" bool waypoint_load_csv(const char *filepath)
{
    File f = get_active_fs().open(filepath, "r");
    if (!f && sd_card_mounted) {
        f = FFat.open(filepath, "r");
    }
    if (!f) {
        Serial.printf("waypoint_load_csv: Could not open %s\n", filepath);
        return false;
    }

    g_waypoint_count = 0;
    bool first_line = true;
    int col_label = 0, col_lat = 1, col_lon = 2, col_elev = 3, col_notes = 4;

    while (f.available() && g_waypoint_count < MAX_WAYPOINTS) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.length() == 0 || line.startsWith("#")) continue;

        // Split CSV columns
        String cols[10];
        int col_cnt = 0;
        int start = 0;
        int len = line.length();
        for (int i = 0; i <= len && col_cnt < 10; i++) {
            if (i == len || line.charAt(i) == ',') {
                cols[col_cnt] = line.substring(start, i);
                cols[col_cnt].trim();
                col_cnt++;
                start = i + 1;
            }
        }

        if (first_line) {
            first_line = false;
            String c0 = cols[0];
            c0.toLowerCase();
            if (c0.indexOf("label") >= 0 || c0.indexOf("name") >= 0 || c0.indexOf("id") >= 0) {
                col_elev = -1;
                col_notes = -1;
                for (int c = 0; c < col_cnt; c++) {
                    String h = cols[c];
                    h.toLowerCase();
                    if (h.indexOf("label") >= 0 || h.indexOf("name") >= 0) col_label = c;
                    else if (h.indexOf("lat") >= 0) col_lat = c;
                    else if (h.indexOf("lon") >= 0) col_lon = c;
                    else if (h.indexOf("elev") >= 0 || h.indexOf("alt") >= 0 || h.indexOf("height") >= 0) col_elev = c;
                    else if (h.indexOf("note") >= 0 || h.indexOf("desc") >= 0) col_notes = c;
                }
                continue;
            }
        }

        if (col_cnt <= col_lon) continue;

        WaypointRecord_t *wp = &g_waypoints[g_waypoint_count];
        memset(wp, 0, sizeof(WaypointRecord_t));
        strncpy(wp->label, cols[col_label].c_str(), WAYPOINT_LABEL_LEN - 1);
        wp->latitude  = strtod(cols[col_lat].c_str(), NULL);
        wp->longitude = strtod(cols[col_lon].c_str(), NULL);

        if (col_elev >= 0 && col_elev < col_cnt && cols[col_elev].length() > 0) {
            wp->elevation_m   = (float)atof(cols[col_elev].c_str());
            wp->has_elevation = true;
        }
        if (col_notes >= 0 && col_notes < col_cnt && cols[col_notes].length() > 0) {
            strncpy(wp->notes, cols[col_notes].c_str(), WAYPOINT_NOTES_LEN - 1);
        }

        g_waypoint_count++;
    }
    f.close();

    if (g_active_waypoint_idx >= g_waypoint_count) {
        g_active_waypoint_idx = 0;
    }
    Serial.printf("Loaded %d waypoints from %s\n", g_waypoint_count, filepath);
    return g_waypoint_count > 0;
}

extern "C" void waypoint_manager_init(void)
{
    create_default_sample_waypoints_if_missing();
    waypoint_load_csv(current_settings.active_waypoint_file);
}

extern "C" void waypoint_select_next(void)
{
    if (g_waypoint_count <= 0) return;
    g_active_waypoint_idx = (g_active_waypoint_idx + 1) % g_waypoint_count;
}

extern "C" void waypoint_select_prev(void)
{
    if (g_waypoint_count <= 0) return;
    g_active_waypoint_idx = (g_active_waypoint_idx - 1 + g_waypoint_count) % g_waypoint_count;
}

extern "C" void waypoint_select_index(int16_t idx)
{
    if (idx >= 0 && idx < g_waypoint_count) {
        g_active_waypoint_idx = idx;
    }
}

extern "C" bool waypoint_get_active(WaypointRecord_t *wp_out)
{
    if (g_waypoint_count <= 0 || g_active_waypoint_idx < 0 || g_active_waypoint_idx >= g_waypoint_count) {
        return false;
    }
    if (wp_out) {
        *wp_out = g_waypoints[g_active_waypoint_idx];
    }
    return true;
}

extern "C" void waypoint_format_next_survey_label(char *buf, size_t max_len)
{
    if (g_survey_target_match_mode && g_waypoint_count > 0) {
        snprintf(buf, max_len, "%s_FND", g_waypoints[g_active_waypoint_idx].label);
    } else {
        uint8_t p_idx = g_survey_prefix_idx < 8 ? g_survey_prefix_idx : 0;
        snprintf(buf, max_len, "%s%03u", kPrefixes[p_idx], g_survey_auto_num);
    }
}

extern "C" void trigger_survey_waypoint_log(void)
{
    survey_requested = true;
}

static const char* fix_type_to_str(GnssFixType_t fix)
{
    switch (fix) {
        case GNSS_FIX_RTK_FIXED: return "RTK_FIXED";
        case GNSS_FIX_RTK_FLOAT: return "RTK_FLOAT";
        case GNSS_FIX_DGPS:      return "DGPS";
        case GNSS_FIX_SINGLE:    return "SINGLE";
        default:                 return "NO_FIX";
    }
}

static void save_survey_record_to_csv(double lat, double lon, double height_m,
                                      GnssFixType_t fix, float h_acc_m, uint16_t samples)
{
    bool exists = get_active_fs().exists("/survey_logged_pins.csv");
    File f = get_active_fs().open("/survey_logged_pins.csv", FILE_APPEND);
    if (!f) {
        f = get_active_fs().open("/survey_logged_pins.csv", FILE_WRITE);
    }
    if (!f) {
        Serial.println("Failed to open /survey_logged_pins.csv");
        return;
    }

    if (!exists) {
        f.println("label,latitude,longitude,elevation,fix_type,h_acc_m,samples,timestamp,notes");
    }

    char label[WAYPOINT_LABEL_LEN];
    waypoint_format_next_survey_label(label, sizeof(label));

    char ts[32];
    get_formatted_timestamp(ts, sizeof(ts), false);

    uint8_t tag_idx = g_survey_note_tag_idx < 8 ? g_survey_note_tag_idx : 0;
    char note_buf[96] = {0};

    WaypointRecord_t active_wp;
    if (g_survey_target_match_mode && waypoint_get_active(&active_wp)) {
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

    char row[256];
    snprintf(row, sizeof(row), "%s,%.8f,%.8f,%.3f,%s,%.3f,%u,%s,%s",
             label, lat, lon, height_m, fix_type_to_str(fix), h_acc_m, samples, ts, note_buf);
    f.println(row);
    f.close();

    strncpy(last_logged_label, label, sizeof(last_logged_label) - 1);
    if (!g_survey_target_match_mode) {
        g_survey_auto_num++;
    }
    Serial.printf("SURVEY LOGGED: %s\n", row);
}

extern "C" void waypoint_update_survey_averaging(const RoverUIData_t *live_state,
                                                 bool *in_progress_out,
                                                 uint8_t *progress_pct_out,
                                                 uint16_t *epochs_out,
                                                 char *last_logged_out)
{
    if (survey_requested && !survey_active) {
        survey_requested = false;
        survey_active    = true;
        survey_start_ms  = millis();
        survey_sum_lat   = 0.0;
        survey_sum_lon   = 0.0;
        survey_sum_alt   = 0.0;
        survey_sum_hacc  = 0.0f;
        survey_epoch_cnt = 0;
        survey_best_fix  = live_state->fix_type;
        play_rover_beep(1200, 80);
    }

    if (survey_active) {
        survey_sum_lat  += live_state->tip_lat;
        survey_sum_lon  += live_state->tip_lon;
        survey_sum_alt  += live_state->tip_height_m;
        survey_sum_hacc += live_state->h_acc_m;
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

            save_survey_record_to_csv(avg_lat, avg_lon, avg_alt, survey_best_fix, avg_hacc, survey_epoch_cnt);
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
