#ifndef WAYPOINT_MANAGER_H
#define WAYPOINT_MANAGER_H

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

// Survey Logger (Tile 2 BOOT Button / Touch Trigger)
extern bool     g_survey_target_match_mode;
extern uint8_t  g_survey_prefix_idx;
extern uint16_t g_survey_auto_num;
extern uint8_t  g_survey_note_tag_idx;

const char* waypoint_get_prefix_options(void);
const char* waypoint_get_note_tag_options(void);
void waypoint_format_next_survey_label(char *buf, size_t max_len);
void waypoint_update_survey_averaging(const RoverUIData_t *live_state,
                                      bool *in_progress_out,
                                      uint8_t *progress_pct_out,
                                      uint16_t *epochs_out,
                                      char *last_logged_out);

#ifdef __cplusplus
}
#endif

#endif // WAYPOINT_MANAGER_H
