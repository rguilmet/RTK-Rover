#ifndef E220_DRIVER_H
#define E220_DRIVER_H

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
// frames before atomically forwarding to UM982 COM2 (Serial2), tracks AUX ISR
// edges, and queries Mode 0 ambient/packet RSSI registers.
void e220_poll_and_forward_rtcm(void);

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

#ifdef __cplusplus
}
#endif

#endif // E220_DRIVER_H
