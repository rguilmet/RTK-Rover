#ifndef WAYPOINT_MANAGER_INTERNAL_H
#define WAYPOINT_MANAGER_INTERNAL_H

#include "waypoint_manager.h"
#include "../settings_manager/settings_manager.h"
#include <FS.h>
#include <FFat.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

extern bool sd_card_mounted;
extern SemaphoreHandle_t fs_mux;

fs::FS &waypoint_primary_fs(void);
bool waypoint_has_custom_fs(void);
SemaphoreHandle_t waypoint_fs_mux(void);
bool waypoint_survey_try_start_internal(int16_t prefix_idx);
bool waypoint_survey_cancel_pending_internal(void);
bool waypoint_survey_is_busy_internal(void);
void waypoint_survey_ext_consume_note(char *buf, size_t sz);
uint8_t waypoint_survey_ext_get_dur_s(uint8_t fallback_s);
bool waypoint_survey_ext_take_cancel(void);
void waypoint_survey_ext_clear_cancel(void);
void waypoint_survey_ext_reset_overrides(void);
void waypoint_file_sync_active_idx(const char *filepath);
void waypoint_survey_ext_queue_abort(const char *reason);
void waypoint_survey_ext_flush_abort(void);
void waypoint_survey_ext_emit_result(bool ok,
                                     const char *reason,
                                     const char *label,
                                     double lat,
                                     double lon,
                                     float h_sigma_m,
                                     float max_tilt_deg,
                                     uint16_t samples);

#endif // WAYPOINT_MANAGER_INTERNAL_H
