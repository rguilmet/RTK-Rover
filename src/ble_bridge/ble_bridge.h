#ifndef BLE_BRIDGE_H
#define BLE_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "../rover_api.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ROVER_BLE_UI_OFF       = 0,
    ROVER_BLE_UI_STARTING  = 1,
    ROVER_BLE_UI_ADV       = 2,
    ROVER_BLE_UI_CONN      = 3,
    ROVER_BLE_UI_SLAVE     = 4,
    ROVER_BLE_UI_WIFI_HOLD = 5
} RoverBleUiState_t;

typedef void (*ble_bridge_stream_sink_cb_t)(const uint8_t *data, size_t len);

void ble_bridge_init(void);
void ble_bridge_loop(void);
void ble_bridge_feed_state(const RoverUIData_t *state);

void ble_bridge_request_toggle(void);
void ble_bridge_reclaim_local(void);

RoverBleUiState_t ble_bridge_get_ui_state(void);
bool ble_bridge_is_active(void);
bool ble_bridge_is_connected(void);
bool ble_bridge_is_slave_dimmed(void);
bool ble_bridge_consume_ui_dirty(void);
void ble_bridge_format_status(char *buf, size_t buf_sz);

void ble_bridge_notify_local_ui_change(const char *settings_json, const char *runtime_json);
void ble_bridge_set_stream_rx_sink(ble_bridge_stream_sink_cb_t sink_cb);

// Shared F-11 storage lock & upload hooks (used by both fic_web_portal and FICP BLE)
bool rover_is_storage_write_locked(char *reason_out, size_t reason_sz);
bool rover_take_screenshot_to(char *out_path, size_t out_path_sz);
void rover_on_storage_file_updated(const char *filename, const char *fs_key);

#ifdef __cplusplus
}
#endif

#endif // BLE_BRIDGE_H
