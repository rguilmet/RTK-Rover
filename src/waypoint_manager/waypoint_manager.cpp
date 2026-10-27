#include "waypoint_manager.h"
#include "../../user_config.h"
#include "../settings_manager/settings_manager.h"
#include "../rtc_bsp/rtc_bsp.h"
#include "../nav_engine/nav_engine.h"
#include <math.h>
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

    f.println("timestamp,firmware_ver,label,latitude,longitude,elevation,pos_fix,hdg_fix,h_acc_m,v_acc_m,hdop,sats,slave_sats,diff_age_s,base_id,true_heading_deg,pole_tilt_deg,samples,notes");
    f.println("2026-10-26_00-00-00.000-240,v0.2.10,PIN_NW_01,42.360100000,-71.094200000,12.500,DESIGN,NONE,0.000,0.000,0.0,0,0,0.0,0,0.00,0.00,1,NW corner iron pipe near oak");
    f.println("2026-10-26_00-00-00.000-240,v0.2.10,PIN_NE_02,42.360450000,-71.093500000,13.100,DESIGN,NONE,0.000,0.000,0.0,0,0,0.0,0,0.00,0.00,1,NE corner rebar by stone wall");
    f.println("2026-10-26_00-00-00.000-240,v0.2.10,PIN_SE_03,42.359800000,-71.093400000,11.800,DESIGN,NONE,0.000,0.000,0.0,0,0,0.0,0,0.00,0.00,1,SE corner monument");
    f.println("2026-10-26_00-00-00.000-240,v0.2.10,PIN_SW_04,42.359750000,-71.094150000,12.000,DESIGN,NONE,0.000,0.000,0.0,0,0,0.0,0,0.00,0.00,1,SW corner pin by driveway");
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

        // Split CSV columns (supporting up to 24 columns for extended logs)
        String cols[24];
        int col_cnt = 0;
        int start = 0;
        int len = line.length();
        for (int i = 0; i <= len && col_cnt < 24; i++) {
            if (i == len || line.charAt(i) == ',') {
                cols[col_cnt] = line.substring(start, i);
                cols[col_cnt].trim();
                col_cnt++;
                start = i + 1;
            }
        }

        if (first_line) {
            first_line = false;
            String line_lower = line;
            line_lower.toLowerCase();
            if (line_lower.indexOf("label") >= 0 || line_lower.indexOf("name") >= 0 || line_lower.indexOf("id") >= 0 || line_lower.indexOf("lat") >= 0 || line_lower.indexOf("time") >= 0) {
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
    bool exists = get_active_fs().exists("/survey_logged_pins.csv");
    File f = get_active_fs().open("/survey_logged_pins.csv", FILE_APPEND);
    if (!f) {
        f = get_active_fs().open("/survey_logged_pins.csv", FILE_WRITE);
    }
    if (!f) {
        Serial.println("Failed to open /survey_logged_pins.csv");
        return;
    }

    // Standardized header: timestamp is Column 1, followed by firmware_ver
    if (!exists) {
        f.println("timestamp,firmware_ver,label,latitude,longitude,elevation,pos_fix,hdg_fix,h_acc_m,v_acc_m,hdop,sats,slave_sats,diff_age_s,base_id,true_heading_deg,pole_tilt_deg,samples,notes");
    }

    char label[WAYPOINT_LABEL_LEN];
    waypoint_format_next_survey_label(label, sizeof(label));

    // Standardized ISO / MFS timestamp format matching route logger
    RtcDateTime_t lt;
    get_local_datetime(&lt);
    uint16_t ms = (uint16_t)(millis() % 1000);
    char sign = (current_settings.minutesOffsetToUTC >= 0) ? '+' : '-';
    int abs_offset = abs(current_settings.minutesOffsetToUTC);

    char ts_buf[48];
    snprintf(ts_buf, sizeof(ts_buf), "%04u-%02u-%02u_%02u-%02u-%02u.%03u%c%03d",
             lt.year, lt.month, lt.day, lt.hour, lt.minute, lt.second, ms, sign, abs_offset);

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

    strncpy(last_logged_label, label, sizeof(last_logged_label) - 1);
    if (!g_survey_target_match_mode) {
        g_survey_auto_num++;
    }
    Serial.printf("SURVEY LOGGED: %s\n", row);
}

// Survey averaging accumulation variables
static float          survey_sum_vacc       = 0.0f;
static float          survey_sum_hdop       = 0.0f;
static float          survey_sum_tilt       = 0.0f;
static float          survey_sum_hdg        = 0.0f;
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
        survey_sum_hdg        = 0.0f;
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
            survey_sum_hdg += live_state->true_heading_deg;
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
            float  avg_hdg  = (survey_hdg_valid_cnt > 0) ? (survey_sum_hdg / (float)survey_hdg_valid_cnt) : -1.0f;
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
