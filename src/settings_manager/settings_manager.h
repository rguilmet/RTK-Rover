#ifndef SETTINGS_MANAGER_H
#define SETTINGS_MANAGER_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
#include <Arduino.h>
#include <FS.h>
#endif

#define AUDIO_NAV_BEEPS  0
#define AUDIO_NAV_GEIGER 1
#define AUDIO_NAV_MUTE   2

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
    uint8_t  e220_profile_rev;
    uint32_t e220_uart_baud;
    uint8_t  e220_channel;
    float    e220_air_rate_kbps;
    uint8_t  e220_tx_power_dbm;
    uint16_t e220_subpacket_bytes;
    bool     e220_lbt_enable;
    uint8_t  survey_avg_seconds;        // Averaging duration (1, 5, 10, or 30s)
    char     default_label_prefix[16];
    char     active_waypoint_file[48];
    int16_t  active_waypoint_index;
    uint8_t  brightness;
    uint8_t  volume;
    bool     is_muted;
    uint8_t  audio_nav_mode;
    char     wifi_ssid[33];
    char     wifi_password[65];
    bool     enable_serial_logging;
    bool     enable_nmea_logging;
    bool     enable_rtcm_logging;
    bool     enable_imu_logging;
    bool     enable_telemetry_logging;
    bool     enable_touch_logging;
    bool     map_show_route_trail;
    uint8_t  auto_shutdown_battery_min;
    float    mag_true_north_offset_deg; // Auto-calibrated True-North compass tare from UM982 HDG FIX
    float    mag_field_ref_uT;          // Ambient geomagnetic field reference magnitude (uT)
    bool     mag_cal_valid;             // True once UM982 HDG FIX has auto-calibrated the compass
    uint8_t  beam_imu_orient_mode;      // Signed axis permutation 0..7 (default 4: Face-Down 0 deg)
    float    mag_hard_iron_uT[3];       // 3D hard-iron bias center (uT)
    float    mag_soft_iron[9];          // Row-major 3x3 soft-iron ellipsoid correction matrix
    bool     mag_ellipsoid_cal_valid;   // True once 360-deg ellipsoid CAL MAG completed cleanly
    float    mag_bz_ref_uT;             // Learned world-vertical magnetic component (uT)
    float    beam_pitch_tare_deg;       // Learned pitch offset (beam_pitch_deg - baseline_pitch_deg)
    bool     apply_lever_arm_on_calibrated_beam; // Opt-in lever-arm when HDG_SRC_COMP has calibrated beam
    char     rtcm_source_mode[16];      // "AUTO", "LORA", or "NTRIP"
    char     ntrip_host[64];            // e.g. "rtk2go.com"
    char     ntrip_host_ip[32];         // Fallback numeric host IP (e.g. "3.14.70.106")
    uint16_t ntrip_port;                // Default 2101
    char     ntrip_mountpoint[48];      // Private NTRIP mountpoint (empty default in public repo)
    char     ntrip_user_email[64];      // Private NTRIP user/email (empty default in public repo)
    char     ntrip_password[32];        // Default "none"
    uint16_t settings_schema_version;   // settings.json schema version (v2 adds guidance/map fields)
    float    line_log_max_m;            // Tile 0 log cross-track outer span (m, default 50.0)
    float    line_audio_fade_m;         // Off-line directional audio silence cutoff (m, default 20.0)
    float    line_arrow_green_deg;      // Heading lock green threshold (deg, default 4.0)
    float    line_arrow_amber_deg;      // Heading lock orange threshold (deg, default 15.0)
    float    map_zoom_m_per_px;         // Sticky Map View zoom (m/px, 0.0 = Auto-Fit)
    float    survey_max_tilt_deg;       // Max pole tilt gate for survey averaging (deg, 0 = warn-only)
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
bool settings_lock(uint32_t timeout_ms);
void settings_unlock(void);
void settings_get_snapshot(struct SystemSettings *out);

#ifdef __cplusplus
}
#endif

#endif // SETTINGS_MANAGER_H
