#include "rover_audio.h"
#include "../../user_config.h"
#include <Field_Instrument_Core.h>
#include "../settings_manager/settings_manager.h"
#include "../storage/serial_logger.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>

extern "C" void set_rover_volume(uint8_t volume_pct)
{
    if (volume_pct > 100) volume_pct = 100;
    current_settings.volume = volume_pct;
    current_settings.is_muted = (volume_pct == 0);
    fic_audio_beeper_set_volume(volume_pct);
    fic_audio_beeper_set_muted(current_settings.is_muted);
}

// =========================================================================
// Audio Beep Synthesis via ES8311 (Optional Feedback for Logging / Screenshot)
// =========================================================================
extern "C" void rover_audio_init(void)
{
    FicAudioBeeperCfg_t cfg;
    fic_audio_beeper_default_cfg(&cfg,
                                 "S3_LCD_3_5B",
                                 fic_lvgl_get_io_expander(),
                                 (uint32_t)ROVER_EXIO_PA_CTRL);
    cfg.initial_volume = current_settings.volume;
    cfg.initial_muted  = current_settings.is_muted;
    cfg.log_cb         = rover_printf;
    (void)fic_audio_beeper_init(&cfg);
}

extern "C" void play_rover_beep(uint16_t freq_hz, uint16_t duration_ms)
{
    fic_audio_beeper_play(freq_hz, duration_ms);
}

extern "C" void play_rover_dual_beep(uint16_t freq1_hz, uint16_t dur1_ms, uint16_t gap_ms, uint16_t freq2_hz, uint16_t dur2_ms)
{
    fic_audio_beeper_play_dual(freq1_hz, dur1_ms, gap_ms, freq2_hz, dur2_ms);
}

extern "C" void play_rover_beep_gain(uint16_t freq_hz, uint16_t duration_ms, uint8_t gain_pct)
{
    fic_audio_beeper_play_gain(freq_hz, duration_ms, gain_pct);
}

extern "C" void play_rover_dual_beep_gain(uint16_t freq1_hz, uint16_t dur1_ms, uint16_t gap_ms,
                                          uint16_t freq2_hz, uint16_t dur2_ms, uint8_t gain_pct)
{
    fic_audio_beeper_play_dual_gain(freq1_hz, dur1_ms, gap_ms, freq2_hz, dur2_ms, gain_pct);
}

extern "C" void rover_audio_start_task(void)
{
    (void)fic_audio_beeper_start_task();
}

extern "C" void rover_audio_update_nav_guidance(const RoverUIData_t *ui_state,
                                                bool has_mode1_line,
                                                bool mode1_bullseye,
                                                bool mode1_entered_bullseye,
                                                double mode1_offset_m,
                                                double mode1_dist_b_m)
{
    if (!ui_state) return;

    // Proximity & Acoustic Fade Directional Line-Follow Audio Synthesis
    if (current_settings.audio_nav_mode != AUDIO_NAV_MUTE &&
        !current_settings.is_muted && current_settings.volume > 0 &&
        ui_state->has_active_target &&
        (ui_state->fix_type == GNSS_FIX_RTK_FIXED || ui_state->fix_type == GNSS_FIX_RTK_FLOAT || ui_state->fix_type == GNSS_FIX_DGPS) &&
        (ui_state->distance_to_target_m > 0.0 || has_mode1_line)) {

        static uint32_t s_last_audio_ms = 0;
        static int16_t  s_last_audio_target_idx = -1;
        if (ui_state->active_target_idx != s_last_audio_target_idx) {
            s_last_audio_target_idx = ui_state->active_target_idx;
            s_last_audio_ms = 0;
        }

        if (has_mode1_line && !mode1_bullseye) {
            // Acoustic Fade-Out Directional Line Audio (faster & louder closer to line, silent beyond fade_m)
            double fade_m  = (double)current_settings.line_audio_fade_m;
            double abs_off = fabs(mode1_offset_m);
            if (fade_m > 0.0 && abs_off <= fade_m) {
                uint32_t interval_ms = 650U;
                uint8_t  gain_pct    = 90U;
                if (abs_off > 0.05) {
                    double denom_log = log(1.0 + fade_m / 0.25);
                    double frac = (denom_log > 1e-4) ? (log(1.0 + abs_off / 0.25) / denom_log) : 1.0;
                    if (frac < 0.0) frac = 0.0;
                    if (frac > 1.0) frac = 1.0;
                    interval_ms = (uint32_t)(220.0 + frac * 2280.0);
                    gain_pct    = (uint8_t)(100.0 - frac * 75.0);
                }

                if (current_settings.audio_nav_mode == AUDIO_NAV_GEIGER) {
                    interval_ms = (interval_ms * 7U) / 10U;
                }

                uint32_t now_ms = millis();
                if (now_ms - s_last_audio_ms >= interval_ms) {
                    s_last_audio_ms = now_ms;
                    if (abs_off <= 0.05) {
                        // On-Line: crisp single high pip
                        play_rover_beep_gain(1800, 30, gain_pct);
                    } else if (mode1_offset_m < -0.05) {
                        // Left of A->B: rising 2-tone chirp -> steer RIGHT
                        play_rover_dual_beep_gain(660, 30, 25, 1100, 40, gain_pct);
                    } else {
                        // Right of A->B: falling 2-tone chirp -> steer LEFT
                        play_rover_dual_beep_gain(1100, 30, 25, 660, 40, gain_pct);
                    }
                }
            }
        } else {
            if (has_mode1_line && mode1_entered_bullseye) {
                // Acquisition chime on < 4.8 m Mode 1 handoff into Bullseye proximity audio
                play_rover_dual_beep(1400, 45, 30, 2000, 70);
                s_last_audio_ms = millis();
            }

            double d = has_mode1_line ? mode1_dist_b_m : ui_state->distance_to_target_m;
            if (d > 0.0) {
                if (current_settings.audio_nav_mode == AUDIO_NAV_GEIGER) {
                    if (d <= 2.0) {
                        uint32_t interval_ms = 1000;
                        uint16_t freq_hz = 900;
                        uint16_t dur_ms = 15;

                        if (d <= 0.02) {          // Bullseye (<= 2cm / 0.8 in)
                            interval_ms = 80;
                            freq_hz     = 2400;
                            dur_ms      = 35;
                        } else if (d <= 0.10) {   // 2 .. 10 cm
                            interval_ms = 90;
                            freq_hz     = 2000;
                            dur_ms      = 10;
                        } else if (d <= 0.30) {   // 10 .. 30 cm
                            interval_ms = 160;
                            freq_hz     = 1750;
                            dur_ms      = 12;
                        } else if (d <= 0.70) {   // 30 .. 70 cm
                            interval_ms = 320;
                            freq_hz     = 1400;
                            dur_ms      = 15;
                        } else if (d <= 1.20) {   // 70 .. 120 cm
                            interval_ms = 600;
                            freq_hz     = 1100;
                            dur_ms      = 18;
                        } else {                  // 1.2 .. 2.0 m
                            interval_ms = 1000;
                            freq_hz     = 900;
                            dur_ms      = 20;
                        }

                        uint32_t now_ms = millis();
                        if (now_ms - s_last_audio_ms >= interval_ms) {
                            s_last_audio_ms = now_ms;
                            play_rover_beep(freq_hz, dur_ms);
                        }
                    }
                } else if (current_settings.audio_nav_mode == AUDIO_NAV_BEEPS) {
                    if (d <= 5.0) {
                        uint32_t interval_ms = 1200;
                        uint16_t freq_hz = 1200;
                        uint16_t dur_ms = 50;

                        if (d <= 0.10) {          // Very close (<= 10 cm)
                            interval_ms = 200;
                            freq_hz     = 2000;
                            dur_ms      = 40;
                        } else if (d <= 0.50) {   // 10 .. 50 cm
                            interval_ms = 400;
                            freq_hz     = 1600;
                            dur_ms      = 45;
                        } else if (d <= 2.0) {    // 0.5 .. 2.0 m
                            interval_ms = 800;
                            freq_hz     = 1400;
                            dur_ms      = 50;
                        } else {                  // 2.0 .. 5.0 m
                            interval_ms = 1500;
                            freq_hz     = 1200;
                            dur_ms      = 50;
                        }

                        uint32_t now_ms = millis();
                        if (now_ms - s_last_audio_ms >= interval_ms) {
                            s_last_audio_ms = now_ms;
                            play_rover_beep(freq_hz, dur_ms);
                        }
                    }
                }
            }
        }
    }
}
