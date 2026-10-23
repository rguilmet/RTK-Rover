#ifndef ROVER_API_H
#define ROVER_API_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// =========================================================================
// RTK Fix Quality Enumeration (Matches NMEA GGA Fix Quality)
// =========================================================================
typedef enum {
    GNSS_FIX_NONE      = 0,
    GNSS_FIX_SINGLE    = 1,
    GNSS_FIX_DGPS      = 2,
    GNSS_FIX_PPS       = 3,
    GNSS_FIX_RTK_FIXED = 4,
    GNSS_FIX_RTK_FLOAT = 5,
    GNSS_FIX_DR        = 6
} GnssFixType_t;

// =========================================================================
// Handheld Display Mounting Pose (Derived from QMI8658 Gravity Vector)
// =========================================================================
typedef enum {
    DISPLAY_POSE_FLAT     = 0, // Held flat in hand (0-30 deg pitch)
    DISPLAY_POSE_ANGLED   = 1, // Held at viewing angle (30-70 deg pitch)
    DISPLAY_POSE_VERTICAL = 2  // Clipped vertically to survey pole (70-90 deg pitch)
} DisplayPose_t;

// =========================================================================
// Waypoint Record Structure (Parsed from / Saved to CSV)
// =========================================================================
#define MAX_WAYPOINTS         128
#define WAYPOINT_LABEL_LEN    28
#define WAYPOINT_NOTES_LEN    48

typedef struct {
    char     label[WAYPOINT_LABEL_LEN];
    double   latitude;
    double   longitude;
    float    elevation_m;
    bool     has_elevation;
    char     notes[WAYPOINT_NOTES_LEN];
} WaypointRecord_t;

// =========================================================================
// E220 LoRa & RTCM3 Link Telemetry
// =========================================================================
typedef struct {
    uint32_t rx_bytes_total;
    uint32_t rtcm_frames_ok;
    uint32_t rtcm_crc_errors;
    uint32_t last_rx_ms;
    uint16_t last_msg_type;     // e.g. 1005, 1074, 1084, 1094, 1124
    uint16_t msg_1005_count;    // Station ARP
    uint16_t msg_msm_count;     // GPS/GLO/GAL/BDS MSM4/MSM7
    bool     aux_pin_high;
    uint8_t  mode_m0;
    uint8_t  mode_m1;
    bool     config_valid;
    uint8_t  cfg_addr_high;
    uint8_t  cfg_addr_low;
    uint8_t  cfg_reg0;
    uint8_t  cfg_reg1;
    uint8_t  cfg_channel;
    uint8_t  cfg_reg3;
} E220LinkStats_t;

// =========================================================================
// UM982 COM1 & COM2 Telemetry
// =========================================================================
typedef struct {
    uint32_t com1_rx_bytes;
    uint32_t com1_tx_bytes;
    uint32_t com2_tx_bytes;     // RTCM bytes forwarded to COM2
    uint32_t nmea_sentences_ok;
    uint32_t heading_sentences_ok;
    char     last_com1_line[96];
    char     um982_version[48];
} Um982PortStats_t;

// =========================================================================
// Master Rover State Snapshot (Passed via FreeRTOS Queue to UI at 30Hz)
// =========================================================================
typedef struct {
    // Raw UM982 Master Antenna Position
    double           raw_lat;
    double           raw_lon;
    double           raw_alt_m;

    // Lever-Arm & Pole-Height Compensated Ground Tip Position (Pole Centerline)
    double           tip_lat;
    double           tip_lon;
    double           tip_height_m;

    // GNSS Solution Quality
    GnssFixType_t    fix_type;
    uint8_t          num_sats;
    float            hdop;
    float            h_acc_m;         // Horizontal accuracy estimate (m)
    float            v_acc_m;         // Vertical accuracy estimate (m)
    float            diff_age_s;      // Age of RTCM differential corrections (s)

    // Dual-Antenna Heading & Pitch (from UNIHEADINGA / GPHDT)
    bool             heading_valid;
    float            true_heading_deg;   // 0..360 deg True North
    float            baseline_pitch_deg; // Baseline tilt angle (deg)
    float            baseline_length_m;  // Measured Master->Slave distance (e.g. 0.555m)

    // Active Target Guidance (Computed by nav_engine)
    bool             has_active_target;
    int16_t          active_target_idx;
    int16_t          total_targets;
    char             target_label[WAYPOINT_LABEL_LEN];
    char             target_notes[WAYPOINT_NOTES_LEN];
    double           distance_to_target_m;
    float            bearing_to_target_deg;
    float            rel_bearing_deg;    // Relative to rover heading (-180..+180)
    float            d_north_m;          // Target offset North from Rover Tip (m)
    float            d_east_m;           // Target offset East from Rover Tip (m)

    // IMU & Pole Bubble Level (QMI8658)
    float            imu_pitch_deg;
    float            imu_roll_deg;
    float            pole_bubble_x_deg;  // Tared pole tilt Roll
    float            pole_bubble_y_deg;  // Tared pole tilt Pitch
    DisplayPose_t    display_pose;

    // Power & Battery (AXP2101)
    float            battery_voltage;
    uint8_t          battery_percent;
    bool             is_charging;
    bool             vbus_present;

    // Survey Logging State (Tile 2 BOOT button)
    bool             survey_in_progress;
    uint8_t          survey_progress_pct;
    uint16_t         survey_epochs_collected;
    char             last_logged_label[WAYPOINT_LABEL_LEN];

    // Hardware Link Diagnostics
    E220LinkStats_t  e220_stats;
    Um982PortStats_t um982_stats;
} RoverUIData_t;

// =========================================================================
// Global Action Triggers (Callable from UI or Hardware Tasks)
// =========================================================================
void trigger_survey_waypoint_log(void);
void trigger_tare_pole_level(void);
void request_e220_config_read(void);
void send_um982_com1_command(const char *cmd);
void send_um982_com2_test_pulse(void);
void play_rover_beep(uint16_t freq_hz, uint16_t duration_ms);

#ifdef __cplusplus
}
#endif

#endif // ROVER_API_H
