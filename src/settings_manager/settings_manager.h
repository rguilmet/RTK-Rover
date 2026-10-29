#ifndef SETTINGS_MANAGER_H
#define SETTINGS_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
#include <Arduino.h>
#include <FS.h>
#endif

struct SystemSettings {
    int16_t  minutesOffsetToUTC;
    char     timezone_label[16];
    bool     use_imperial_units;
    float    antenna_height_m;
    float    antenna_spacing_m;
    float    baseline_mount_offset_deg;
    bool     apply_lever_arm;
    float    pole_tilt_tare_pitch_deg;
    float    pole_tilt_tare_roll_deg;
    uint32_t e220_uart_baud;
    uint8_t  e220_channel;
    float    e220_air_rate_kbps;
    uint8_t  e220_tx_power_dbm;
    uint16_t e220_subpacket_bytes;
    bool     e220_lbt_enable;
    uint8_t  survey_avg_seconds;
    char     default_label_prefix[16];
    char     active_waypoint_file[48];
    int16_t  active_waypoint_index;
    uint8_t  brightness;
    uint8_t  volume;
    bool     is_muted;
    char     wifi_ssid[33];
    char     wifi_password[65];
    bool     enable_serial_logging;
    bool     enable_nmea_logging;
    bool     enable_rtcm_logging;
    bool     enable_imu_logging;
    bool     enable_telemetry_logging;
    bool     enable_touch_logging;
};

extern struct SystemSettings current_settings;
extern volatile bool settings_dirty;
extern volatile uint32_t last_settings_change_ms;
extern bool sd_card_mounted;

#ifdef __cplusplus
fs::FS& get_active_fs(void);
extern "C" {
#endif

void init_settings_manager(void);
void load_settings(void);
void save_settings(void);
void mark_settings_dirty(void);

#ifdef __cplusplus
}
#endif

#endif // SETTINGS_MANAGER_H
