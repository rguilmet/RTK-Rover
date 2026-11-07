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
#define WAYPOINT_LABEL_LEN    36
#define WAYPOINT_NOTES_LEN    128

typedef struct {
    char     label[WAYPOINT_LABEL_LEN];
    double   latitude;
    double   longitude;
    float    elevation_m;
    bool     has_elevation;
    int16_t  poly_order;        // Positive: polygon vertex sequence (1, 2, ...); Negative: tie to vertex; 0: pin
    bool     is_boundary;       // True if poly_order > 0
    int16_t  tie_to_poly_order; // Vertex sequence number this pin ties to (positive int), or 0 if none
    char     notes[WAYPOINT_NOTES_LEN];
} WaypointRecord_t;

// =========================================================================
// E220 LoRa & RTCM3 Link Telemetry
// =========================================================================
typedef struct {
    uint32_t rx_bytes_total;
    uint32_t rtcm_bytes_forwarded; // Verified CRC24Q-valid bytes forwarded to UM982 COM2
    uint32_t rtcm_frames_ok;
    uint32_t rtcm_crc_errors;      // Total dropped frames (CRC mismatch + mid-frame truncation)
    uint32_t rtcm_trunc_drops;     // Frames aborted due to missing tail subpacket (>220ms silence)
    uint32_t rtcm_orphan_bytes;    // Non-preamble / orphan tail bytes stripped before UM982 COM2
    uint32_t aux_fall_count;       // Mode 0 AUX (IO21) falling-edge interrupt counter
    uint32_t aux_last_low_us;      // Most recent Mode 0 AUX LOW pulse width (microseconds)
    uint32_t last_rx_ms;
    uint16_t last_msg_type;        // e.g. 1006, 1074, 1084, 1094, 1230
    uint16_t msg_1005_count;       // Station ARP (1005/1006)
    uint16_t msg_msm_count;        // Total GPS/GLO/GAL/BDS MSM4/MSM7
    uint16_t msg_1074_count;       // GPS MSM4
    uint16_t msg_1084_count;       // GLONASS MSM4
    uint16_t msg_1094_count;       // Galileo MSM4
    uint16_t msg_1230_count;       // GLONASS Code-Phase Biases
    bool     rssi_valid;           // True once C0 C1 C2 C3 RSSI registers read in Mode 0
    int16_t  ambient_noise_dbm;    // Channel ambient noise floor in dBm (-(256 - raw))
    int16_t  last_pkt_rssi_dbm;    // Last received LoRa packet RSSI in dBm (-(256 - raw))
    bool     aux_pin_high;
    uint8_t  mode_m0;
    uint8_t  mode_m1;
    bool     config_valid;
    bool     config_matched;       // True if read registers match target settings.json
    uint8_t  cfg_addr_high;
    uint8_t  cfg_addr_low;
    uint8_t  cfg_reg0;
    uint8_t  cfg_reg1;
    uint8_t  cfg_channel;
    uint8_t  cfg_reg3;
    bool     m0_wire_ok;           // Boot-time hardware continuity test: M0 line
    bool     m1_wire_ok;           // Boot-time hardware continuity test: M1 line
    uint32_t uplink_packets_sent;  // 39-byte uplink telemetry frames sent to Base
    uint32_t uplink_rate_switches; // Successful air-rate switches completed
    bool     rate_switch_pending;  // True while waiting for Base ACK / switch delay
    float    active_air_rate_kbps; // Current operating air rate (e.g. 19.2, 9.6)
} E220LinkStats_t;

// =========================================================================
// UM982 Single-UART (COM1) & Known-State Audit Telemetry
// =========================================================================
typedef struct {
    uint32_t com1_rx_bytes;
    uint32_t com1_tx_bytes;
    uint32_t com2_rx_bytes;     // Legacy counter (0 in Single-UART COM1 mode)
    uint32_t com2_tx_bytes;     // Verified RTCM3 bytes forwarded to UM982 (via COM1 in v0.8.0+)
    uint32_t nmea_sentences_ok;
    uint32_t heading_sentences_ok;
    uint32_t cmd_responses_ok;  // Count of $command / #VERSIONA / #MODE / $CONFIG / #LOGLIST responses
    bool     com1_test_passed;
    bool     com2_test_passed;  // True when Single-UART COM1 bidirectional self-test & state audit pass
    bool     known_state_ok;    // True when MODE == ROVER SURVEY, COM1 has 7 logs, and COM2 has 0 logs
    char     last_com1_line[96];       // Most recent NMEA / Navigation sentence on COM1
    char     last_com2_line[64];       // Most recent Command Response / ACK on COM1
    char     um982_version[48];        // Parsed from #VERSIONA
    char     um982_mode[32];           // Parsed from #MODE (e.g. "ROVER SURVEY")
    char     um982_config_summary[96]; // Parsed from $CONFIG (e.g. "MULTI | FIXLEN 55.5/12.0 LOWDYN")
    char     um982_loglist[112];       // Parsed from #LOGLIST (e.g. "COM1(7): GGA RMC GST GSV HDG GGAH BASE | COM2:0")
} Um982PortStats_t;

// =========================================================================
// Heading Source Resolution
// =========================================================================
typedef enum {
    HDG_SRC_NONE = 0,
    HDG_SRC_DUAL_FIX,    // 1: Dual-antenna integer carrier phase RTK (NARROW_INT)
    HDG_SRC_DUAL_FLT,    // 2: Dual-antenna float carrier phase with verified baseline
    HDG_SRC_DISP,        // 3: Kinematic geodetic displacement from 2-3 paces (>= 0.70m)
    HDG_SRC_GYRO,        // 4: IMU Gyroscope dead-reckoning hold during pauses (< 60s)
    HDG_SRC_COG,         // 5: Kinematic Course-Over-Ground from walking motion (> 0.4 m/s)
    HDG_SRC_HOLD,        // 6: Stationary hold of last valid heading (< 0.2 m/s)
    HDG_SRC_COMP         // 7: 9-DOF Tilt-Compensated MMC5983MA + QMI8658 Compass Fusion
} HeadingSource_t;

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
    uint8_t          num_sats;        // Satellites used in position solution (GGA)
    uint8_t          sats_in_view;    // Total satellites in view across constellations (GSV)
    float            hdop;
    float            h_acc_m;         // Horizontal accuracy estimate (m)
    float            v_acc_m;         // Vertical accuracy estimate (m)
    float            diff_age_s;      // Age of RTCM differential corrections (s)

    // Dual-Antenna Heading & Pitch (from UNIHEADINGA / GPHDT)
    bool             heading_valid;
    bool             heading_is_fixed;   // True if UNIHEADINGA pos_type == NARROW_INT
    HeadingSource_t  heading_source;     // Active heading origin
    float            true_heading_deg;   // 0..360 deg True North
    float            baseline_pitch_deg; // Baseline tilt angle (deg)
    float            baseline_length_m;  // Measured Master->Slave distance (e.g. 0.555m)
    float            speed_m_s;          // Speed over ground (m/s) from RMC
    float            cog_deg;            // Course over ground (deg) from RMC
    float            displacement_dist_m;// Phase 5: Distance moved from step anchor (m)

    // Slave Antenna (ANT2) Diagnostics (from GNGGAH)
    GnssFixType_t    slave_fix_type;
    uint8_t          slave_num_sats;     // Satellites used by Slave Antenna
    float            slave_hdop;         // HDOP of Slave Antenna
    float            slave_alt_m;        // Altitude of Slave Antenna (m)
    double           slave_lat;          // Latitude of Slave Antenna (deg)
    double           slave_lon;          // Longitude of Slave Antenna (deg)

    // Base Station Telemetry (from GNGGA and BASEINFOA)
    uint16_t         base_station_id;    // Base station ID (e.g. 1)
    bool             base_coords_valid;  // True if BASEINFOA received
    double           base_lat;           // Base Station Latitude (deg)
    double           base_lon;           // Base Station Longitude (deg)
    double           base_alt_m;         // Base Station Altitude (m)
    double           base_ecef_x;        // Base Station ECEF X (m)
    double           base_ecef_y;        // Base Station ECEF Y (m)
    double           base_ecef_z;        // Base Station ECEF Z (m)

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
    float            imu_acc[3];         // Accel X, Y, Z (g)
    float            imu_gyr[3];         // Gyro X, Y, Z (deg/s)

    // 3-Axis AMR Magnetometer / 9-DOF Compass (SparkFun MMC5983MA @ 0x30, Auto-Detected)
    bool             mag_present;        // True if MMC5983MA (0x30, Reg 0x2F == 0x30) detected at boot
    bool             mag_cal_valid;      // True once True-North tare learned from UM982 HDG FIX or settings.json
    bool             mag_anomaly;        // True if |B| deviates > 3.5 uT from ambient reference (iron pipe/fence)
    float            mag_uT[3];          // SET/RESET bridge-offset-cancelled Bx, By, Bz (uT)
    float            mag_total_uT;       // Total magnetic field magnitude ||B|| (uT)
    float            mag_heading_deg;    // Tilt-compensated True-North magnetic heading (0..360 deg)
    float            mag_tare_offset_deg;// Learned True-North tare offset (absorbs declination + mounting angle)
    float            mag_die_temp_c;     // MMC5983MA internal die temperature (deg C)
    uint32_t         mag_set_reset_count;// Number of hardware SET/RESET degaussing pulses executed

    // Power & Battery (AXP2101)
    float            battery_voltage;
    uint8_t          battery_percent;
    bool             is_charging;
    bool             vbus_present;
    float            vbus_voltage;

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
void request_e220_config_write(void);
void send_um982_com1_command(const char *cmd);
void send_um982_com2_test_pulse(void);
void play_rover_beep(uint16_t freq_hz, uint16_t duration_ms);

#ifdef __cplusplus
}
#endif

#endif // ROVER_API_H
