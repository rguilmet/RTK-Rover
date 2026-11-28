#ifndef BLE_BRIDGE_INTERNAL_H
#define BLE_BRIDGE_INTERNAL_H

#include "ble_bridge.h"
#include <Field_Instrument_Core.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

void ble_bridge_mark_ui_dirty(bool refresh_wp_table);
bool ble_bridge_get_telem_survey_snapshot(bool *survey_active_out, uint8_t *pct_out);

bool ble_bridge_ext_rpc_dispatch(uint8_t msg_id,
                                 uint16_t seq,
                                 uint8_t flags,
                                 const uint8_t *payload,
                                 uint16_t len,
                                 char *ack_json_out,
                                 size_t ack_json_sz,
                                 FicpErrCode_t *err_code_out,
                                 char *err_reason_out,
                                 size_t err_reason_sz);

void ble_bridge_on_survey_result_event(bool ok,
                                       const char *reason,
                                       const char *label,
                                       double lat,
                                       double lon,
                                       float h_sigma_m,
                                       float max_tilt_deg,
                                       uint16_t samples);

void ble_bridge_sync_reflected_snapshot(void);
void ble_bridge_poll_local_reflection(uint32_t now_ms);
void ble_bridge_poll_survey_progress(uint32_t now_ms, bool survey_active, uint8_t pct);

#endif // BLE_BRIDGE_INTERNAL_H
