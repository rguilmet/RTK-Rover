#include "waypoint_manager.h"
#include "../../user_config.h"
#include "../settings_manager/settings_manager.h"
#include "../rtc_bsp/rtc_bsp.h"
#include "../nav_engine/nav_engine.h"
#include "../lvgl_port/lvgl_port.h"
#include <math.h>
#include <Arduino.h>
#include <SD_MMC.h>
#include <FFat.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include "esp_heap_caps.h"

extern SemaphoreHandle_t fs_mux;

WaypointRecord_t *g_waypoints          = NULL;
int16_t          g_waypoint_count      = 0;
int16_t          g_active_waypoint_idx = 0;
volatile uint32_t g_waypoint_change_seq = 0;

static bool ensure_waypoints_buf(void)
{
    if (g_waypoints) return true;
    g_waypoints = (WaypointRecord_t *)heap_caps_calloc(MAX_WAYPOINTS, sizeof(WaypointRecord_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!g_waypoints) {
        g_waypoints = (WaypointRecord_t *)calloc(MAX_WAYPOINTS, sizeof(WaypointRecord_t));
    }
    return (g_waypoints != NULL);
}

#define MAX_WAYPOINT_FILES 16
static char s_waypoint_files[MAX_WAYPOINT_FILES][48];
static int16_t s_waypoint_file_cnt = 0;
static int16_t s_active_file_idx   = 0;

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
    "Found Rebar",
    "Found Stone Bound",
    "Set Temp Flag",
    "Mag Anomaly Only",
    "Obstructed Sky",
    "Needs Verification"
};
static const char *kNoteTagDropdownStr =
    "(No Tag)\nFound Iron Pipe\nFound Rebar\nFound Stone Bound\nSet Temp Flag\nMag Anomaly Only\nObstructed Sky\nNeeds Verification";

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

static void create_default_sample_waypoints_if_missing(void)
{
    if (get_active_fs().exists("/waypoints.csv")) {
        return;
    }
    File f = get_active_fs().open("/waypoints.csv", "w");
    if (!f) return;

    f.println("timestamp,firmware_ver,label,latitude,longitude,elevation,pos_fix,hdg_fix,h_acc_m,v_acc_m,hdop,sats,slave_sats,diff_age_s,base_id,true_heading_deg,pole_tilt_deg,samples,notes");
    f.println("2026-10-27_12-00-00.000-240,v0.2.17,PIN_NW_01,42.360100000,-71.094200000,12.500,DESIGN,NONE,0.000,0.000,0.0,0,0,0.0,0,0.00,0.00,1,NW corner iron pipe near oak tree (sample waypoint with extended notes to demonstrate circular marquee autoscroll)");
    f.println("2026-10-27_12-00-00.000-240,v0.2.17,PIN_NE_02,42.360450000,-71.093500000,13.100,DESIGN,NONE,0.000,0.000,0.0,0,0,0.0,0,0.00,0.00,1,NE corner rebar by stone wall (sample pin)");
    f.println("2026-10-27_12-00-00.000-240,v0.2.17,PIN_SE_03,42.359800000,-71.093400000,11.800,DESIGN,NONE,0.000,0.000,0.0,0,0,0.0,0,0.00,0.00,1,SE corner granite monument bound");
    f.println("2026-10-27_12-00-00.000-240,v0.2.17,PIN_SW_04,42.359750000,-71.094150000,12.000,DESIGN,NONE,0.000,0.000,0.0,0,0,0.0,0,0.00,0.00,1,SW corner pin by driveway entrance");
    f.close();
    Serial.println("Created sample /waypoints.csv with 4 generic property pins.");
}

static char* csv_trim_in_place(char *s)
{
    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') s++;
    size_t len = strlen(s);
    while (len > 0 && (s[len - 1] == ' ' || s[len - 1] == '\t' || s[len - 1] == '\r' || s[len - 1] == '\n')) {
        s[--len] = '\0';
    }
    if (len >= 2 && s[0] == '"' && s[len - 1] == '"') {
        s[len - 1] = '\0';
        s++;
    }
    return s;
}

static int csv_split_cols_in_place(char *line, char **cols, int max_cols)
{
    int cnt = 0;
    char *p = line;
    while (cnt < max_cols) {
        char *start = p;
        bool in_quotes = false;
        while (*p != '\0') {
            if (*p == '"') {
                in_quotes = !in_quotes;
            } else if (*p == ',' && !in_quotes) {
                break;
            }
            p++;
        }
        bool at_end = (*p == '\0');
        if (!at_end) {
            *p = '\0';
            p++;
        }
        cols[cnt++] = csv_trim_in_place(start);
        if (at_end) break;
    }
    return cnt;
}

static bool str_contains_icase(const char *haystack, const char *needle)
{
    if (!haystack || !needle || !needle[0]) return false;
    size_t nlen = strlen(needle);
    for (const char *p = haystack; *p != '\0'; p++) {
        if (strncasecmp(p, needle, nlen) == 0) return true;
    }
    return false;
}

extern "C" bool waypoint_load_csv(const char *filepath)
{
    if (!ensure_waypoints_buf()) {
        Serial.println("waypoint_load_csv: Failed to allocate waypoint buffer!");
        return false;
    }

    if (fs_mux) xSemaphoreTake(fs_mux, portMAX_DELAY);

    File f = get_active_fs().open(filepath, "r");
    if (!f && sd_card_mounted) {
        f = FFat.open(filepath, "r");
    }
    // Fallback to default /waypoints.csv if specific file not found
    if (!f && strcmp(filepath, "/waypoints.csv") != 0) {
        f = get_active_fs().open("/waypoints.csv", "r");
        if (!f && sd_card_mounted) f = FFat.open("/waypoints.csv", "r");
    }
    if (!f) {
        if (fs_mux) xSemaphoreGive(fs_mux);
        Serial.printf("waypoint_load_csv: Could not open %s\n", filepath);
        return false;
    }

    g_waypoint_count = 0;
    bool first_line = true;
    int col_label = 0, col_lat = 1, col_lon = 2, col_elev = 3, col_notes = 4, col_poly = -1;

    uint8_t rbuf[512];
    char line_buf[512];
    size_t line_len = 0;

    for (;;) {
        int n = f.read(rbuf, sizeof(rbuf));
        if (n <= 0 && line_len == 0) break;

        int limit = (n > 0) ? n : 1;
        for (int bi = 0; bi < limit; bi++) {
            char ch = (n > 0) ? (char)rbuf[bi] : '\n';
            if (ch == '\n') {
                line_buf[line_len] = '\0';
                line_len = 0;

                char *trimmed = csv_trim_in_place(line_buf);
                if (trimmed[0] == '\0' || trimmed[0] == '#') {
                    continue;
                }

                // Check header detection before splitting modifies commas
                bool looks_like_hdr = false;
                if (first_line) {
                    looks_like_hdr = str_contains_icase(trimmed, "label") ||
                                     str_contains_icase(trimmed, "name")  ||
                                     str_contains_icase(trimmed, "id")    ||
                                     str_contains_icase(trimmed, "lat")   ||
                                     str_contains_icase(trimmed, "time");
                }

                char *cols[24];
                int col_cnt = csv_split_cols_in_place(trimmed, cols, 24);

                if (first_line) {
                    first_line = false;
                    if (looks_like_hdr) {
                        col_elev = -1;
                        col_notes = -1;
                        col_poly = -1;
                        for (int c = 0; c < col_cnt; c++) {
                            const char *h = cols[c];
                            if (str_contains_icase(h, "label") || str_contains_icase(h, "name")) col_label = c;
                            else if (str_contains_icase(h, "lat")) col_lat = c;
                            else if (str_contains_icase(h, "lon")) col_lon = c;
                            else if (str_contains_icase(h, "elev") || str_contains_icase(h, "alt") || str_contains_icase(h, "height")) col_elev = c;
                            else if (str_contains_icase(h, "note") || str_contains_icase(h, "desc")) col_notes = c;
                            else if (str_contains_icase(h, "poly") || str_contains_icase(h, "order") || str_contains_icase(h, "seq")) col_poly = c;
                        }
                        continue;
                    }
                }

                if (col_cnt <= col_label || col_cnt <= col_lat || col_cnt <= col_lon) continue;
                if (cols[col_label][0] == '\0' || cols[col_lat][0] == '\0' || cols[col_lon][0] == '\0') continue;

                WaypointRecord_t *wp = &g_waypoints[g_waypoint_count];
                memset(wp, 0, sizeof(WaypointRecord_t));
                strncpy(wp->label, cols[col_label], WAYPOINT_LABEL_LEN - 1);
                wp->latitude  = strtod(cols[col_lat], NULL);
                wp->longitude = strtod(cols[col_lon], NULL);

                if (col_elev >= 0 && col_elev < col_cnt && cols[col_elev][0] != '\0') {
                    wp->elevation_m   = (float)atof(cols[col_elev]);
                    wp->has_elevation = true;
                }
                if (col_notes >= 0 && col_notes < col_cnt && cols[col_notes][0] != '\0') {
                    strncpy(wp->notes, cols[col_notes], WAYPOINT_NOTES_LEN - 1);
                }

                // Phase 5 Cadastral Classification (poly_order):
                if (col_poly >= 0 && col_poly < col_cnt && cols[col_poly][0] != '\0') {
                    const char *pstr = cols[col_poly];
                    if (strncasecmp(pstr, "TIE:", 4) == 0) {
                        int t_val = atoi(pstr + 4);
                        wp->poly_order = (t_val > 0) ? (int16_t)(-t_val) : (int16_t)(-1);
                        wp->is_boundary = false;
                        wp->tie_to_poly_order = (t_val > 0) ? (int16_t)t_val : 1;
                    } else if (strcasecmp(pstr, "TIE") == 0 || strcasecmp(pstr, "REF") == 0 || strcasecmp(pstr, "CONTROL") == 0) {
                        wp->poly_order = 0;
                        wp->is_boundary = false;
                        wp->tie_to_poly_order = 0;
                    } else if (strcasecmp(pstr, "PIN") == 0 || strcasecmp(pstr, "BOUNDARY") == 0) {
                        wp->poly_order = (int16_t)(g_waypoint_count + 1);
                        wp->is_boundary = true;
                        wp->tie_to_poly_order = 0;
                    } else {
                        int val = atoi(pstr);
                        wp->poly_order = (int16_t)val;
                        if (val > 0) {
                            wp->is_boundary = true;
                            wp->tie_to_poly_order = 0;
                        } else if (val < 0) {
                            wp->is_boundary = false;
                            wp->tie_to_poly_order = (int16_t)(-val);
                        } else {
                            wp->is_boundary = false;
                            wp->tie_to_poly_order = 0;
                        }
                    }
                } else {
                    // Backward compatibility when no poly_order column is in CSV:
                    // All points sequentially form the closed boundary polygon
                    wp->poly_order = (int16_t)(g_waypoint_count + 1);
                    wp->is_boundary = true;
                    wp->tie_to_poly_order = 0;
                }

                g_waypoint_count++;
                if (g_waypoint_count >= MAX_WAYPOINTS) break;
            } else if (ch != '\r') {
                if (line_len < sizeof(line_buf) - 1) {
                    line_buf[line_len++] = ch;
                }
            }
        }
        if (n <= 0 || g_waypoint_count >= MAX_WAYPOINTS) break;
    }
    f.close();
    if (fs_mux) xSemaphoreGive(fs_mux);

    if (g_active_waypoint_idx >= g_waypoint_count) {
        g_active_waypoint_idx = 0;
    }
    waypoint_invalidate_survey_cache();
    g_waypoint_change_seq++;
    Serial.printf("Loaded %d waypoints from %s\n", g_waypoint_count, filepath);
    return g_waypoint_count > 0;
}

extern "C" void waypoint_scan_files(void)
{
    s_waypoint_file_cnt = 0;

    // Check files on active filesystem (SD card or FFat)
    fs::FS &fs = get_active_fs();
    File root = fs.open("/");
    if (root && root.isDirectory()) {
        File file = root.openNextFile();
        while (file && s_waypoint_file_cnt < MAX_WAYPOINT_FILES) {
            if (!file.isDirectory()) {
                String fname = file.name();
                if (!fname.startsWith("/")) fname = "/" + fname;
                String fname_lower = fname;
                fname_lower.toLowerCase();
                if (fname_lower.startsWith("/waypoints") && fname_lower.endsWith(".csv")) {
                    bool exists = false;
                    for (int i = 0; i < s_waypoint_file_cnt; i++) {
                        if (strcasecmp(s_waypoint_files[i], fname.c_str()) == 0) {
                            exists = true;
                            break;
                        }
                    }
                    if (!exists) {
                        strncpy(s_waypoint_files[s_waypoint_file_cnt], fname.c_str(), sizeof(s_waypoint_files[0]) - 1);
                        s_waypoint_files[s_waypoint_file_cnt][sizeof(s_waypoint_files[0]) - 1] = '\0';
                        s_waypoint_file_cnt++;
                    }
                }
            }
            file = root.openNextFile();
        }
        root.close();
    }

    // Also check FFat if SD card is mounted
    if (sd_card_mounted && s_waypoint_file_cnt < MAX_WAYPOINT_FILES) {
        File froot = FFat.open("/");
        if (froot && froot.isDirectory()) {
            File file = froot.openNextFile();
            while (file && s_waypoint_file_cnt < MAX_WAYPOINT_FILES) {
                if (!file.isDirectory()) {
                    String fname = file.name();
                    if (!fname.startsWith("/")) fname = "/" + fname;
                    String fname_lower = fname;
                    fname_lower.toLowerCase();
                    if (fname_lower.startsWith("/waypoints") && fname_lower.endsWith(".csv")) {
                        bool exists = false;
                        for (int i = 0; i < s_waypoint_file_cnt; i++) {
                            if (strcasecmp(s_waypoint_files[i], fname.c_str()) == 0) {
                                exists = true;
                                break;
                            }
                        }
                        if (!exists) {
                            strncpy(s_waypoint_files[s_waypoint_file_cnt], fname.c_str(), sizeof(s_waypoint_files[0]) - 1);
                            s_waypoint_files[s_waypoint_file_cnt][sizeof(s_waypoint_files[0]) - 1] = '\0';
                            s_waypoint_file_cnt++;
                        }
                    }
                }
                file = froot.openNextFile();
            }
            froot.close();
        }
    }

    // Fallback if no files on disk
    if (s_waypoint_file_cnt == 0) {
        strncpy(s_waypoint_files[0], "/waypoints.csv", sizeof(s_waypoint_files[0]) - 1);
        s_waypoint_file_cnt = 1;
    }

    // Sort files alphabetically
    for (int i = 0; i < s_waypoint_file_cnt - 1; i++) {
        for (int j = i + 1; j < s_waypoint_file_cnt; j++) {
            if (strcmp(s_waypoint_files[i], s_waypoint_files[j]) > 0) {
                char temp[48];
                strcpy(temp, s_waypoint_files[i]);
                strcpy(s_waypoint_files[i], s_waypoint_files[j]);
                strcpy(s_waypoint_files[j], temp);
            }
        }
    }

    // Match active file to settings
    s_active_file_idx = 0;
    for (int i = 0; i < s_waypoint_file_cnt; i++) {
        if (strcasecmp(s_waypoint_files[i], current_settings.active_waypoint_file) == 0) {
            s_active_file_idx = i;
            break;
        }
    }
}

extern "C" void waypoint_file_select_next(void)
{
    if (s_waypoint_file_cnt <= 0) return;
    s_active_file_idx = (s_active_file_idx + 1) % s_waypoint_file_cnt;
    strncpy(current_settings.active_waypoint_file, s_waypoint_files[s_active_file_idx], sizeof(current_settings.active_waypoint_file) - 1);
    current_settings.active_waypoint_file[sizeof(current_settings.active_waypoint_file) - 1] = '\0';
    current_settings.active_waypoint_index = 0;
    mark_settings_dirty();
    g_active_waypoint_idx = 0;
    waypoint_line_stakeout_reset_auto();
    waypoint_load_csv(current_settings.active_waypoint_file);
    refresh_waypoint_table_ui();
}

extern "C" void waypoint_file_select_prev(void)
{
    if (s_waypoint_file_cnt <= 0) return;
    s_active_file_idx = (s_active_file_idx - 1 + s_waypoint_file_cnt) % s_waypoint_file_cnt;
    strncpy(current_settings.active_waypoint_file, s_waypoint_files[s_active_file_idx], sizeof(current_settings.active_waypoint_file) - 1);
    current_settings.active_waypoint_file[sizeof(current_settings.active_waypoint_file) - 1] = '\0';
    current_settings.active_waypoint_index = 0;
    mark_settings_dirty();
    g_active_waypoint_idx = 0;
    waypoint_line_stakeout_reset_auto();
    waypoint_load_csv(current_settings.active_waypoint_file);
    refresh_waypoint_table_ui();
}

extern "C" const char* waypoint_file_get_active_name(void)
{
    if (s_waypoint_file_cnt <= 0 || s_active_file_idx < 0 || s_active_file_idx >= s_waypoint_file_cnt) {
        return "waypoints.csv";
    }
    const char *p = s_waypoint_files[s_active_file_idx];
    if (*p == '/') p++;
    return p;
}

extern "C" int16_t waypoint_file_get_active_index(void)
{
    return s_active_file_idx;
}

extern "C" int16_t waypoint_file_get_count(void)
{
    return s_waypoint_file_cnt;
}

extern "C" void waypoint_manager_init(void)
{
    ensure_waypoints_buf();
    create_default_sample_waypoints_if_missing();
    waypoint_scan_files();
    g_active_waypoint_idx = current_settings.active_waypoint_index;
    waypoint_load_csv(current_settings.active_waypoint_file);
    if (g_active_waypoint_idx < 0 || g_active_waypoint_idx >= g_waypoint_count) {
        g_active_waypoint_idx = 0;
    }
    current_settings.active_waypoint_index = g_active_waypoint_idx;
    if (g_waypoint_count > 0) {
        Serial.printf("[WAYPOINTS] Restored active waypoint index: %d/%d ('%s')\n",
                      g_active_waypoint_idx + 1, g_waypoint_count, g_waypoints[g_active_waypoint_idx].label);
    }
}

extern "C" void waypoint_select_next(void)
{
    if (g_waypoint_count <= 0) return;
    g_active_waypoint_idx = (g_active_waypoint_idx + 1) % g_waypoint_count;
    current_settings.active_waypoint_index = g_active_waypoint_idx;
    g_waypoint_change_seq++;
    mark_settings_dirty();
    waypoint_invalidate_survey_cache();
}

extern "C" void waypoint_select_prev(void)
{
    if (g_waypoint_count <= 0) return;
    g_active_waypoint_idx = (g_active_waypoint_idx - 1 + g_waypoint_count) % g_waypoint_count;
    current_settings.active_waypoint_index = g_active_waypoint_idx;
    g_waypoint_change_seq++;
    mark_settings_dirty();
    waypoint_invalidate_survey_cache();
}

extern "C" void waypoint_select_index(int16_t idx)
{
    if (idx >= 0 && idx < g_waypoint_count) {
        g_active_waypoint_idx = idx;
        current_settings.active_waypoint_index = g_active_waypoint_idx;
        g_waypoint_change_seq++;
        mark_settings_dirty();
        waypoint_invalidate_survey_cache();
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

// Phase 4: Line Stakeout State & Helpers
int16_t g_stakeout_pt_a_idx  = -1;
int16_t g_stakeout_pt_b_idx  = -1;
bool    g_stakeout_custom_pair = false;

extern "C" bool waypoint_get_at(int16_t idx, WaypointRecord_t *wp_out)
{
    if (idx < 0 || idx >= g_waypoint_count) {
        return false;
    }
    if (wp_out) {
        *wp_out = g_waypoints[idx];
    }
    return true;
}

extern "C" int16_t waypoint_get_boundary_polygon(int16_t *idx_list_out, int16_t max_count)
{
    if (!idx_list_out || max_count <= 0 || g_waypoint_count <= 0) return 0;

    int16_t cnt = 0;
    // Collect all indices where is_boundary == true and poly_order > 0
    for (int16_t i = 0; i < g_waypoint_count && cnt < max_count; i++) {
        if (g_waypoints[i].is_boundary && g_waypoints[i].poly_order > 0) {
            idx_list_out[cnt++] = i;
        }
    }

    // Sort by poly_order ascending (insertion sort)
    for (int16_t i = 1; i < cnt; i++) {
        int16_t key_idx = idx_list_out[i];
        int16_t key_order = g_waypoints[key_idx].poly_order;
        int16_t j = i - 1;
        while (j >= 0 && g_waypoints[idx_list_out[j]].poly_order > key_order) {
            idx_list_out[j + 1] = idx_list_out[j];
            j--;
        }
        idx_list_out[j + 1] = key_idx;
    }
    return cnt;
}

extern "C" int16_t waypoint_find_by_poly_order(int16_t target_order)
{
    if (target_order <= 0 || g_waypoint_count <= 0) return -1;
    for (int16_t i = 0; i < g_waypoint_count; i++) {
        if (g_waypoints[i].poly_order == target_order) {
            return i;
        }
    }
    return -1;
}

extern "C" void waypoint_line_stakeout_reset_auto(void)
{
    g_stakeout_custom_pair = false;
    g_stakeout_pt_a_idx = -1;
    g_stakeout_pt_b_idx = -1;
}

extern "C" void waypoint_line_stakeout_get_points(WaypointRecord_t *wp_a_out,
                                                 WaypointRecord_t *wp_b_out,
                                                 int16_t *idx_a_out,
                                                 int16_t *idx_b_out)
{
    if (g_waypoint_count < 2) {
        if (idx_a_out) *idx_a_out = -1;
        if (idx_b_out) *idx_b_out = -1;
        return;
    }

    int16_t a_idx = 0;
    int16_t b_idx = 0;

    if (g_stakeout_custom_pair && g_stakeout_pt_a_idx >= 0 && g_stakeout_pt_b_idx >= 0 &&
        g_stakeout_pt_a_idx < g_waypoint_count && g_stakeout_pt_b_idx < g_waypoint_count) {
        a_idx = g_stakeout_pt_a_idx;
        b_idx = g_stakeout_pt_b_idx;
    } else {
        // Auto default: Point B is active target; Point A is previous index (closed loop wrap-around)
        b_idx = g_active_waypoint_idx;
        if (b_idx < 0 || b_idx >= g_waypoint_count) b_idx = 0;
        a_idx = (b_idx - 1 + g_waypoint_count) % g_waypoint_count;
        g_stakeout_pt_a_idx = a_idx;
        g_stakeout_pt_b_idx = b_idx;
    }

    if (idx_a_out) *idx_a_out = a_idx;
    if (idx_b_out) *idx_b_out = b_idx;
    if (wp_a_out)  *wp_a_out  = g_waypoints[a_idx];
    if (wp_b_out)  *wp_b_out  = g_waypoints[b_idx];
}

extern "C" void waypoint_line_stakeout_swap(void)
{
    if (g_waypoint_count < 2) return;
    int16_t cur_a = 0, cur_b = 0;
    waypoint_line_stakeout_get_points(NULL, NULL, &cur_a, &cur_b);
    g_stakeout_pt_a_idx = cur_b;
    g_stakeout_pt_b_idx = cur_a;
    g_stakeout_custom_pair = true;
    // Align active target to new destination (Point B)
    g_active_waypoint_idx = cur_a;
    current_settings.active_waypoint_index = g_active_waypoint_idx;
    g_waypoint_change_seq++;
    mark_settings_dirty();
}

extern "C" void waypoint_line_stakeout_set_points(int16_t idx_a, int16_t idx_b)
{
    if (idx_a >= 0 && idx_a < g_waypoint_count && idx_b >= 0 && idx_b < g_waypoint_count) {
        g_stakeout_pt_a_idx = idx_a;
        g_stakeout_pt_b_idx = idx_b;
        g_stakeout_custom_pair = true;
        g_active_waypoint_idx = idx_b;
        current_settings.active_waypoint_index = g_active_waypoint_idx;
        g_waypoint_change_seq++;
        mark_settings_dirty();
    }
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
    if (fs_mux) xSemaphoreTake(fs_mux, portMAX_DELAY);
    if (get_active_fs().exists("/survey_logged_pins.csv")) {
        File f = get_active_fs().open("/survey_logged_pins.csv", "r");
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
    if (fs_mux) xSemaphoreGive(fs_mux);

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

    if (fs_mux) xSemaphoreTake(fs_mux, portMAX_DELAY);
    bool exists = get_active_fs().exists("/survey_logged_pins.csv");
    File f = get_active_fs().open("/survey_logged_pins.csv", FILE_APPEND);
    if (!f) {
        f = get_active_fs().open("/survey_logged_pins.csv", FILE_WRITE);
    }
    if (!f) {
        if (fs_mux) xSemaphoreGive(fs_mux);
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

    uint8_t tag_idx = g_survey_note_tag_idx < 8 ? g_survey_note_tag_idx : 0;
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
    if (fs_mux) xSemaphoreGive(fs_mux);

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
