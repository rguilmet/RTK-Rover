#ifndef LVGL_PORT_H
#define LVGL_PORT_H

#include <stdbool.h>
#include <stdint.h>
#include <lvgl.h>
#include "../rover_api.h"
#include "../tca9554/esp_io_expander_tca9554.h"

#ifdef __cplusplus
extern "C" {
#endif

extern esp_io_expander_handle_t io_expander;

void lvgl_port_init(void);
bool rover_lvgl_lock(int timeout_ms);
void rover_lvgl_unlock(void);

// Returns active tile column index (0 = Pin Seeker, 1 = Survey Logger, 2 = CSV Browser, 3 = Compass/Level, 4 = Test Modes)
uint8_t rover_get_active_tile_index(void);

// Returns Tile 0 navigation mode (0 = Point Guidance, 1 = Line Stakeout)
uint8_t rover_get_tile0_nav_mode(void);

void create_rover_ui(void);
void update_rover_ui(const RoverUIData_t *data);
void refresh_waypoint_table_ui(void);

#ifdef __cplusplus
}
#endif

#endif // LVGL_PORT_H
