#include "waypoint_manager_internal.h"
#include "../../user_config.h"
#include "../lvgl_port/lvgl_port.h"
#include <math.h>
#include <Arduino.h>
#include <SD_MMC.h>
#include <FFat.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include "esp_heap_caps.h"

static fs::FS *s_injected_fs = NULL;
static SemaphoreHandle_t s_injected_mux = NULL;
static bool s_has_injected_mux = false;

void waypoint_manager_set_fs(fs::FS *primary_fs, SemaphoreHandle_t mux)
{
    s_injected_fs = primary_fs;
    s_injected_mux = mux;
    s_has_injected_mux = (primary_fs != NULL || mux != NULL);
}

fs::FS &waypoint_primary_fs(void)
{
    return s_injected_fs ? *s_injected_fs : get_active_fs();
}

bool waypoint_has_custom_fs(void)
{
    return s_injected_fs != NULL;
}

SemaphoreHandle_t waypoint_fs_mux(void)
{
    return s_has_injected_mux ? s_injected_mux : fs_mux;
}

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

static void create_default_sample_waypoints_if_missing(void)
{
    fs::FS &fs = waypoint_primary_fs();
    if (fs.exists("/waypoints.csv")) {
        return;
    }
    File f = fs.open("/waypoints.csv", "w");
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

    SemaphoreHandle_t mux = waypoint_fs_mux();
    if (mux) xSemaphoreTake(mux, portMAX_DELAY);

    fs::FS &fs = waypoint_primary_fs();
    bool allow_ffat_fallback = !waypoint_has_custom_fs() && sd_card_mounted;

    File f = fs.open(filepath, "r");
    if (!f && allow_ffat_fallback) {
        f = FFat.open(filepath, "r");
    }
    // Fallback to default /waypoints.csv if specific file not found
    if (!f && strcmp(filepath, "/waypoints.csv") != 0) {
        f = fs.open("/waypoints.csv", "r");
        if (!f && allow_ffat_fallback) f = FFat.open("/waypoints.csv", "r");
    }
    if (!f) {
        if (mux) xSemaphoreGive(mux);
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
    if (mux) xSemaphoreGive(mux);

    if (g_active_waypoint_idx >= g_waypoint_count) {
        g_active_waypoint_idx = 0;
    }
    waypoint_invalidate_survey_cache();
    g_waypoint_change_seq = g_waypoint_change_seq + 1;
    Serial.printf("Loaded %d waypoints from %s\n", g_waypoint_count, filepath);
    return g_waypoint_count > 0;
}

extern "C" void waypoint_scan_files(void)
{
    s_waypoint_file_cnt = 0;

    // Check files on active filesystem (SD card or FFat)
    fs::FS &fs = waypoint_primary_fs();
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
    if (!waypoint_has_custom_fs() && sd_card_mounted && s_waypoint_file_cnt < MAX_WAYPOINT_FILES) {
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
    waypoint_remote_guidance_clear();
    if (g_waypoint_count <= 0) return;
    g_stakeout_custom_pair = false;
    g_active_waypoint_idx = (g_active_waypoint_idx + 1) % g_waypoint_count;
    current_settings.active_waypoint_index = g_active_waypoint_idx;
    g_waypoint_change_seq = g_waypoint_change_seq + 1;
    mark_settings_dirty();
    waypoint_invalidate_survey_cache();
}

extern "C" void waypoint_select_prev(void)
{
    waypoint_remote_guidance_clear();
    if (g_waypoint_count <= 0) return;
    g_stakeout_custom_pair = false;
    g_active_waypoint_idx = (g_active_waypoint_idx - 1 + g_waypoint_count) % g_waypoint_count;
    current_settings.active_waypoint_index = g_active_waypoint_idx;
    g_waypoint_change_seq = g_waypoint_change_seq + 1;
    mark_settings_dirty();
    waypoint_invalidate_survey_cache();
}

extern "C" void waypoint_select_index(int16_t idx)
{
    if (idx >= 0 && idx < g_waypoint_count) {
        waypoint_remote_guidance_clear();
        g_stakeout_custom_pair = false;
        g_active_waypoint_idx = idx;
        current_settings.active_waypoint_index = g_active_waypoint_idx;
        g_waypoint_change_seq = g_waypoint_change_seq + 1;
        mark_settings_dirty();
        waypoint_invalidate_survey_cache();
    }
}

extern "C" bool waypoint_get_active(WaypointRecord_t *wp_out)
{
    if (waypoint_remote_guidance_get_target(wp_out)) {
        return true;
    }
    if (g_waypoint_count <= 0 || g_active_waypoint_idx < 0 || g_active_waypoint_idx >= g_waypoint_count) {
        return false;
    }
    if (wp_out) {
        *wp_out = g_waypoints[g_active_waypoint_idx];
    }
    return true;
}

// Phase 4 & v0.10.0: Line Stakeout & Direct-Here Guidance Anchor State
int16_t g_stakeout_pt_a_idx  = -1;
int16_t g_stakeout_pt_b_idx  = -1;
bool    g_stakeout_custom_pair = false;

static portMUX_TYPE s_guidance_mux = portMUX_INITIALIZER_UNLOCKED;
static bool         s_guidance_direct_mode = false;
static bool         s_direct_a_valid       = false;
static double       s_direct_a_lat         = 0.0;
static double       s_direct_a_lon         = 0.0;
static int16_t      s_direct_target_idx    = -1;

static bool is_valid_coord_pair(double lat, double lon)
{
    return isfinite(lat) && isfinite(lon) && (fabs(lat) > 0.0001 || fabs(lon) > 0.0001);
}

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
    waypoint_remote_guidance_clear();
    g_stakeout_custom_pair = false;
    g_stakeout_pt_a_idx = -1;
    g_stakeout_pt_b_idx = -1;
    portENTER_CRITICAL(&s_guidance_mux);
    s_direct_a_valid    = false;
    s_direct_target_idx = -1;
    portEXIT_CRITICAL(&s_guidance_mux);
}

extern "C" void waypoint_guidance_set_direct_mode(bool direct_here, double cur_lat, double cur_lon)
{
    int16_t b_idx = g_active_waypoint_idx;
    if (b_idx < 0 || b_idx >= g_waypoint_count) b_idx = 0;

    portENTER_CRITICAL(&s_guidance_mux);
    s_guidance_direct_mode = direct_here;
    if (direct_here) {
        if (is_valid_coord_pair(cur_lat, cur_lon)) {
            s_direct_a_lat      = cur_lat;
            s_direct_a_lon      = cur_lon;
            s_direct_a_valid    = true;
            s_direct_target_idx = (g_waypoint_count > 0) ? b_idx : -1;
        } else {
            s_direct_a_valid    = false;
            s_direct_target_idx = -1;
        }
    }
    portEXIT_CRITICAL(&s_guidance_mux);
    g_waypoint_change_seq = g_waypoint_change_seq + 1;
}

extern "C" void waypoint_guidance_reanchor_here(double cur_lat, double cur_lon)
{
    if (waypoint_remote_guidance_reanchor_here(cur_lat, cur_lon)) {
        return;
    }
    int16_t b_idx = g_active_waypoint_idx;
    if (b_idx < 0 || b_idx >= g_waypoint_count) b_idx = 0;

    portENTER_CRITICAL(&s_guidance_mux);
    s_guidance_direct_mode = true;
    if (is_valid_coord_pair(cur_lat, cur_lon)) {
        s_direct_a_lat      = cur_lat;
        s_direct_a_lon      = cur_lon;
        s_direct_a_valid    = true;
        s_direct_target_idx = (g_waypoint_count > 0) ? b_idx : -1;
    } else {
        s_direct_a_valid    = false;
        s_direct_target_idx = -1;
    }
    portEXIT_CRITICAL(&s_guidance_mux);
    g_waypoint_change_seq = g_waypoint_change_seq + 1;
}

extern "C" bool waypoint_guidance_is_direct_mode(void)
{
    if (waypoint_remote_guidance_is_active()) {
        return !waypoint_remote_guidance_is_line_mode();
    }
    portENTER_CRITICAL(&s_guidance_mux);
    bool direct = s_guidance_direct_mode;
    portEXIT_CRITICAL(&s_guidance_mux);
    return direct || (g_waypoint_count == 1);
}

extern "C" bool waypoint_guidance_get_segment(double cur_lat, double cur_lon, ActiveGuidanceSegment_t *seg_out)
{
    if (!seg_out) return false;
    memset(seg_out, 0, sizeof(*seg_out));
    seg_out->idx_a = -1;
    seg_out->idx_b = -1;

    if (waypoint_remote_guidance_is_active()) {
        return waypoint_remote_guidance_get_segment(cur_lat, cur_lon, seg_out);
    }

    if (g_waypoint_count <= 0) {
        return false;
    }

    if (waypoint_guidance_is_direct_mode()) {
        int16_t b_idx = g_active_waypoint_idx;
        if (b_idx < 0 || b_idx >= g_waypoint_count) b_idx = 0;

        bool   a_valid = false;
        double a_lat   = 0.0;
        double a_lon   = 0.0;

        portENTER_CRITICAL(&s_guidance_mux);
        if ((!s_direct_a_valid || s_direct_target_idx != b_idx) && is_valid_coord_pair(cur_lat, cur_lon)) {
            s_direct_a_lat      = cur_lat;
            s_direct_a_lon      = cur_lon;
            s_direct_a_valid    = true;
            s_direct_target_idx = b_idx;
        }
        a_valid = s_direct_a_valid;
        a_lat   = s_direct_a_lat;
        a_lon   = s_direct_a_lon;
        portEXIT_CRITICAL(&s_guidance_mux);

        seg_out->is_direct_here = true;
        seg_out->idx_a          = -1;
        seg_out->idx_b          = b_idx;
        snprintf(seg_out->label_a, sizeof(seg_out->label_a), "HERE");
        seg_out->lat_b = g_waypoints[b_idx].latitude;
        seg_out->lon_b = g_waypoints[b_idx].longitude;
        strncpy(seg_out->label_b, g_waypoints[b_idx].label, WAYPOINT_LABEL_LEN - 1);
        strncpy(seg_out->notes_b, g_waypoints[b_idx].notes, WAYPOINT_NOTES_LEN - 1);

        if (a_valid) {
            seg_out->lat_a = a_lat;
            seg_out->lon_a = a_lon;
            seg_out->valid = true;
            return true;
        }
        return false;
    }

    WaypointRecord_t wp_a, wp_b;
    int16_t idx_a = -1, idx_b = -1;
    waypoint_line_stakeout_get_points(&wp_a, &wp_b, &idx_a, &idx_b);
    if (idx_a < 0 || idx_b < 0) {
        return false;
    }

    seg_out->valid          = true;
    seg_out->is_direct_here = false;
    seg_out->idx_a          = idx_a;
    seg_out->idx_b          = idx_b;
    seg_out->lat_a          = wp_a.latitude;
    seg_out->lon_a          = wp_a.longitude;
    strncpy(seg_out->label_a, wp_a.label, WAYPOINT_LABEL_LEN - 1);
    seg_out->lat_b          = wp_b.latitude;
    seg_out->lon_b          = wp_b.longitude;
    strncpy(seg_out->label_b, wp_b.label, WAYPOINT_LABEL_LEN - 1);
    strncpy(seg_out->notes_b, wp_b.notes, WAYPOINT_NOTES_LEN - 1);
    return true;
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
    if (waypoint_remote_guidance_swap()) return;
    if (g_waypoint_count < 2) return;
    int16_t cur_a = 0, cur_b = 0;
    waypoint_line_stakeout_get_points(NULL, NULL, &cur_a, &cur_b);
    g_stakeout_pt_a_idx = cur_b;
    g_stakeout_pt_b_idx = cur_a;
    g_stakeout_custom_pair = true;
    // Align active target to new destination (Point B)
    g_active_waypoint_idx = cur_a;
    current_settings.active_waypoint_index = g_active_waypoint_idx;
    g_waypoint_change_seq = g_waypoint_change_seq + 1;
    mark_settings_dirty();
}

extern "C" void waypoint_line_stakeout_set_points(int16_t idx_a, int16_t idx_b)
{
    if (idx_a >= 0 && idx_a < g_waypoint_count && idx_b >= 0 && idx_b < g_waypoint_count) {
        waypoint_remote_guidance_clear();
        g_stakeout_pt_a_idx = idx_a;
        g_stakeout_pt_b_idx = idx_b;
        g_stakeout_custom_pair = true;
        g_active_waypoint_idx = idx_b;
        current_settings.active_waypoint_index = g_active_waypoint_idx;
        g_waypoint_change_seq = g_waypoint_change_seq + 1;
        mark_settings_dirty();
    }
}

void waypoint_file_sync_active_idx(const char *filepath)
{
    if (!filepath) return;
    for (int i = 0; i < s_waypoint_file_cnt; i++) {
        if (strcasecmp(s_waypoint_files[i], filepath) == 0) {
            s_active_file_idx = i;
            break;
        }
    }
}
