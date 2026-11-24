#ifndef LVGL_PORT_H
#define LVGL_PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <lvgl.h>
#include "../rover_api.h"
#include "../tca9554/esp_io_expander_tca9554.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef void (*lvgl_port_void_cb_t)(void);
typedef bool (*lvgl_port_bool_cb_t)(void);
typedef int  (*lvgl_port_log_cb_t)(const char *fmt, ...);

typedef struct {
    uint16_t h_res;
    uint16_t v_res;
    size_t   dma_buf_bytes;
    size_t   spiram_buf_bytes;
    int      spi_host;
    int      pin_cs;
    int      pin_pclk;
    int      pin_data0;
    int      pin_data1;
    int      pin_data2;
    int      pin_data3;
    int      pin_bk_light;
    uint32_t pclk_hz;
    uint8_t  tca9554_addr;
    uint32_t exio_input_mask;
    uint32_t exio_output_mask;
    uint32_t exio_lcd_rst_mask;
    uint32_t exio_cam_pwdn_mask;
    uint32_t tick_period_ms;
    uint32_t task_max_delay_ms;
    uint32_t task_min_delay_ms;
    uint32_t task_stack_size;
    uint8_t  task_priority;
    lvgl_port_void_cb_t init_backlight_cb;
    lvgl_port_void_cb_t create_ui_cb;
    lvgl_port_bool_cb_t touch_log_enabled_cb;
    lvgl_port_log_cb_t  log_printf_cb;
} BoardDisplayCfg_t;

extern esp_io_expander_handle_t io_expander;

void lvgl_port_init_with_cfg(const BoardDisplayCfg_t *cfg);
bool rover_lvgl_lock(int timeout_ms);
void rover_lvgl_unlock(void);
bool rover_lvgl_capture_framebuffer(uint8_t *dst_rgb565_be, size_t dst_bytes);

// Returns active tile column index (0 = Pin Seeker, 1 = Field Survey, 2 = CSV Browser, 3 = Compass/Level, 4 = Diagnostics, 5 = Quick Settings)
uint8_t rover_get_active_tile_index(void);

// Returns Tile 0 navigation mode (0 = Unified Guidance, 1 = Map View)
uint8_t rover_get_tile0_nav_mode(void);

void create_rover_ui(void);
void update_rover_ui(const RoverUIData_t *data);
void refresh_waypoint_table_ui(void);

// User Activity Tracking for Battery Timeout Auto-Shutdown
uint32_t rover_get_last_user_activity_ms(void);
void rover_note_user_activity(void);

#ifdef __cplusplus
}
#endif

#endif // LVGL_PORT_H
