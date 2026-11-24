#ifndef LVGL_PORT_H
#define LVGL_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <lvgl.h>
#include <Field_Instrument_Core.h>
#include "../rover_api.h"

#ifdef __cplusplus
extern "C" {
#endif

static inline bool rover_lvgl_lock(int timeout_ms)
{
    return fic_lvgl_lock(timeout_ms);
}

static inline void rover_lvgl_unlock(void)
{
    fic_lvgl_unlock();
}

static inline bool rover_lvgl_capture_framebuffer(uint8_t *dst_rgb565_be, size_t dst_bytes)
{
    return fic_lvgl_capture_framebuffer(dst_rgb565_be, dst_bytes);
}

// Returns active tile column index (0 = Pin Seeker, 1 = Field Survey, 2 = CSV Browser, 3 = Compass/Level, 4 = Diagnostics, 5 = Quick Settings)
uint8_t rover_get_active_tile_index(void);

// Returns Tile 0 navigation mode (0 = Unified Guidance, 1 = Map View)
uint8_t rover_get_tile0_nav_mode(void);

void create_rover_ui(void);
void update_rover_ui(const RoverUIData_t *data);
void refresh_waypoint_table_ui(void);

// User Activity Tracking for Battery Timeout Auto-Shutdown
static inline uint32_t rover_get_last_user_activity_ms(void)
{
    return fic_lvgl_get_last_user_activity_ms();
}

static inline void rover_note_user_activity(void)
{
    fic_lvgl_note_user_activity();
}

#ifdef __cplusplus
}
#endif

#endif // LVGL_PORT_H
