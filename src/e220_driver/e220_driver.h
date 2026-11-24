#ifndef E220_DRIVER_H
#define E220_DRIVER_H

#include <stddef.h>
#include "../rover_api.h"

#ifdef __cplusplus
extern "C" {
#endif

void e220_driver_init(void);
void e220_set_mode(uint8_t m0, uint8_t m1);
bool e220_read_config_registers(void);
bool e220_write_default_config_registers(void);
void e220_run_powerup_selftest(void);

// Polls incoming E220 bytes, stages and CRC24Q-verifies complete RTCM3 (0xD3)
// frames before atomically forwarding to UM982 COM1 (Serial1), tracks AUX ISR
// edges, and queries Mode 0 ambient/packet RSSI registers.
void e220_poll_and_forward_rtcm(void);

// Computes 24-bit RTCM3 CRC-24Q over `len` bytes (returns 0 over complete frame + 3-byte CRC).
uint32_t e220_rtcm_crc24q_compute(const uint8_t *data, size_t len);

// Validates a complete RTCM3 frame (0xD3 preamble, length match, plausible msg type, CRC24Q == 0).
bool e220_rtcm_validate_frame(const uint8_t *frame, size_t len, uint16_t *msg_type_out);

void e220_get_stats(E220LinkStats_t *stats_out);

// Feeds latest fused rover state snapshot for uplink telemetry packing
void e220_feed_telemetry_state(const RoverUIData_t *state);

// Initiates over-the-air negotiation to change LoRa air-rate (e.g. 9.6 or 19.2)
bool e220_request_air_rate_switch(float target_rate_kbps);

// Checks if an air-rate switch negotiation is actively in progress
bool e220_is_rate_switch_pending(void);

// Returns current operating air-rate (kbps)
float e220_get_active_air_rate(void);

// Returns target air-rate requested during pending negotiation (kbps)
float e220_get_requested_air_rate(void);

// Returns true when the GPS-disciplined 5-second superframe Second 0 uplink TX slot is active
bool e220_is_uplink_slot_active(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif // E220_DRIVER_H
