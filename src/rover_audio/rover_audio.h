#ifndef ROVER_AUDIO_H
#define ROVER_AUDIO_H

#include <stdint.h>
#include <stdbool.h>
#include "../rover_api.h"

#ifdef __cplusplus
extern "C" {
#endif

void set_rover_volume(uint8_t volume_pct);
void play_rover_beep(uint16_t freq_hz, uint16_t duration_ms);
void play_rover_dual_beep(uint16_t freq1_hz, uint16_t dur1_ms, uint16_t gap_ms, uint16_t freq2_hz, uint16_t dur2_ms);
void play_rover_beep_gain(uint16_t freq_hz, uint16_t duration_ms, uint8_t gain_pct);
void play_rover_dual_beep_gain(uint16_t freq1_hz, uint16_t dur1_ms, uint16_t gap_ms,
                               uint16_t freq2_hz, uint16_t dur2_ms, uint8_t gain_pct);

void rover_audio_init(void);
void rover_audio_start_task(void);
void rover_audio_update_nav_guidance(const RoverUIData_t *ui_state,
                                     bool has_mode1_line,
                                     bool mode1_bullseye,
                                     bool mode1_entered_bullseye,
                                     double mode1_offset_m,
                                     double mode1_dist_b_m);

#ifdef __cplusplus
}
#endif

#endif // ROVER_AUDIO_H
