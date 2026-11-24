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

#endif // WAYPOINT_MANAGER_INTERNAL_H
