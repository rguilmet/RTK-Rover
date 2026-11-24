#ifndef E220_DRIVER_INTERNAL_H
#define E220_DRIVER_INTERNAL_H

#include "e220_driver.h"
#include "rover_telemetry.h"
#include "../../user_config.h"
#include "../um982_driver/um982_driver.h"
#include "../settings_manager/settings_manager.h"
#include "../storage/rtcm_logger.h"
#include <Arduino.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <math.h>
#include <string.h>

// =========================================================================
// Shared Constants
// =========================================================================
#define RTCM_STAGE_CAPACITY         1536U
#define E220_RSSI_POLL_INTERVAL_MS  750U
#define E220_RSSI_IDLE_POLL_MS      2000U
#define E220_RSSI_REPLY_TIMEOUT_MS  180U

// =========================================================================
// Shared State Across E220 Driver Units
// =========================================================================
// Defined in e220_config_mode3.cpp
extern E220LinkStats_t   e220_stats;
extern volatile bool     pending_cfg_read;
extern volatile bool     pending_cfg_write;
extern volatile bool     e220_suppress_aux_isr;
extern volatile uint32_t isr_aux_fall_count;
extern volatile int64_t  isr_aux_fall_us;
extern volatile uint32_t isr_aux_last_low_us;

// Defined in e220_rtcm_gatekeeper.cpp
extern uint8_t  rx_stage[RTCM_STAGE_CAPACITY];
extern size_t   rx_stage_len;
extern uint32_t last_stage_rx_ms;
extern uint32_t last_1074_rx_ms;
extern uint32_t last_terminal_msm_rx_ms;
extern bool     rtcm_in_resync;

extern bool     rssi_query_active;
extern uint32_t last_rssi_poll_ms;
extern uint32_t s_last_rssi_poll_utc_sec;
extern uint32_t rssi_query_start_ms;
extern uint8_t  rssi_reply_buf[5];
extern uint8_t  rssi_reply_len;
extern uint8_t  s_rssi_timeout_streak;
extern uint32_t s_aux_low_since_ms;
extern bool     s_was_radio_online;

extern uint32_t burst_bytes;
extern uint32_t burst_last_byte_ms;
extern uint32_t last_burst_end_ms;
extern uint32_t last_burst_aux_edges;

// Defined in e220_tdma_scheduler.cpp
extern RoverUIData_t s_latest_telemetry_state;
extern bool          s_has_telemetry_state;
extern portMUX_TYPE  s_telem_mux;
extern uint32_t      s_last_uplink_tx_ms;
extern uint32_t      s_last_uplink_utc_sec;
extern uint8_t       s_uplink_seq;

extern volatile bool s_rate_switch_requested;
extern float         s_requested_rate_kbps;
extern uint16_t      s_rate_switch_req_id;
extern uint32_t      s_rate_switch_req_tx_ms;
extern uint32_t      s_rate_switch_req_start_ms;
extern bool          s_rate_switch_ack_received;
extern uint32_t      s_rate_switch_exec_ms;
extern float         s_active_air_rate;
extern float         s_fallback_air_rate;
extern uint32_t      s_rate_switch_done_ms;
extern uint32_t      s_last_valid_rtcm_ms;
extern uint32_t      s_last_valid_msm_utc_sec;
extern bool          s_switched_requires_watchdog;
extern uint32_t      s_tx_air_busy_until_ms;

// =========================================================================
// Internal Cross-Unit Helpers
// =========================================================================
// e220_config_mode3.cpp
float e220_decode_air_rate_from_reg0(uint8_t reg0);
bool  e220_wait_aux_high(uint32_t timeout_ms);
void  e220_wait_tx_complete(uint32_t fall_wait_ms, uint32_t rise_timeout_ms);

// e220_rtcm_gatekeeper.cpp
uint32_t e220_get_trunc_timeout_ms(void);
void     e220_shift_stage_one_byte(void);
void     e220_process_rx_stage(void);
void     e220_flush_burst_log(uint32_t ref_last_byte_ms);
void     e220_ingest_rx_bytes(const uint8_t *data, size_t len, uint32_t now);
void     e220_record_rssi_reply(uint8_t noise_raw, uint8_t pkt_raw, uint32_t now);

// e220_tdma_scheduler.cpp
bool e220_get_superframe_clock(uint32_t now_ms,
                               uint32_t *utc_sec_out,
                               uint32_t *superframe_sec_out,
                               uint32_t *ms_in_sec_out);

#endif // E220_DRIVER_INTERNAL_H
