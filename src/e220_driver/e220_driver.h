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

#ifdef __cplusplus
}
#endif

#endif // E220_DRIVER_H
