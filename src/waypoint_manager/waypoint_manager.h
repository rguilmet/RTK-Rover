#ifndef WAYPOINT_MANAGER_H
#define WAYPOINT_MANAGER_H

#include <stddef.h>
#include "../rover_api.h"

#ifdef __cplusplus
extern "C" {
#endif

extern WaypointRecord_t g_waypoints[MAX_WAYPOINTS];
extern int16_t          g_waypoint_count;
extern int16_t          g_active_waypoint_idx;

void waypoint_manager_init(void);
bool waypoint_load_csv(const char *filepath);
void waypoint_select_next(void);
void waypoint_select_prev(void);
void waypoint_select_index(int16_t idx);
bool waypoint_get_active(WaypointRecord_t *wp_out);

// Multi-File Waypoint Browser
void        waypoint_scan_files(void);
void        waypoint_file_select_next(void);
void        waypoint_file_select_prev(void);
const char* waypoint_file_get_active_name(void);
int16_t     waypoint_file_get_active_index(void);
int16_t     waypoint_file_get_count(void);

// Survey Logger (Tile 2 BOOT Button / Touch Trigger)
extern uint8_t  g_survey_prefix_idx;
extern uint16_t g_survey_auto_num;
extern uint8_t  g_survey_note_tag_idx;

// Phase 4: Line Stakeout (Baseline A -> B)
extern int16_t g_stakeout_pt_a_idx;
extern int16_t g_stakeout_pt_b_idx;
extern bool    g_stakeout_custom_pair;

void waypoint_line_stakeout_get_points(WaypointRecord_t *wp_a_out,
                                      WaypointRecord_t *wp_b_out,
                                      int16_t *idx_a_out,
                                      int16_t *idx_b_out);
void waypoint_line_stakeout_swap(void);
void waypoint_line_stakeout_set_points(int16_t idx_a, int16_t idx_b);
void waypoint_line_stakeout_reset_auto(void);
bool waypoint_get_at(int16_t idx, WaypointRecord_t *wp_out);

bool        waypoint_is_derived_mode(void);
const char* waypoint_get_prefix_options(void);
const char* waypoint_get_note_tag_options(void);
void        waypoint_format_next_survey_label(char *buf, size_t max_len);
void        waypoint_invalidate_survey_cache(void);
void        waypoint_update_survey_averaging(const RoverUIData_t *live_state,
                                              bool *in_progress_out,
                                              uint8_t *progress_pct_out,
                                              uint16_t *epochs_out,
                                              char *last_logged_out);

#ifdef __cplusplus
}
#endif

#endif // WAYPOINT_MANAGER_H
