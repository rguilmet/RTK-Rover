#ifndef NAV_ENGINE_H
#define NAV_ENGINE_H

#include "../rover_api.h"
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Project Master antenna WGS84 position along Master->Slave baseline to Pole Centerline
// and subtract pole height to obtain exact Ground Tip coordinates.
void nav_compute_pole_tip_position(double master_lat,
                                   double master_lon,
                                   double master_alt_m,
                                   bool heading_valid,
                                   float true_heading_deg,
                                   float baseline_pitch_deg,
                                   double *tip_lat_out,
                                   double *tip_lon_out,
                                   double *tip_height_m_out);

// Compute geodesic distance (m), true bearing (0..360 deg), relative bearing (-180..+180 deg),
// and local North/East offsets (m) from Rover Ground Tip to Active Target.
void nav_compute_target_guidance(double tip_lat,
                                 double tip_lon,
                                 float rover_heading_deg,
                                 double target_lat,
                                 double target_lon,
                                 double *distance_m_out,
                                 float *bearing_deg_out,
                                 float *rel_bearing_deg_out,
                                 float *d_north_m_out,
                                 float *d_east_m_out);

// Phase 4: Line Stakeout (Station & Offset)
typedef struct {
    double station_m;         // Along-track distance from Point A (meters)
    double offset_m;          // Perpendicular cross-track distance (meters, <0 Left, >0 Right)
    double total_length_m;    // Total line distance from Point A to Point B (meters)
    double remaining_m;       // Remaining distance along line to Point B (meters)
    float  line_bearing_deg;  // Deed bearing from Point A to Point B (0..360 deg)
    float  pct_complete;      // Progress along line (0..100%, can be <0 or >100)
    bool   valid;             // True if points A and B are valid and non-coincident
} NavLineStakeout_t;

// Compute along-track station (m), cross-track offset (m), deed bearing, and remaining distance
// for Rover tip relative to directed baseline Point A -> Point B.
void nav_compute_line_stakeout(double tip_lat,
                               double tip_lon,
                               double lat_a,
                               double lon_a,
                               double lat_b,
                               double lon_b,
                               NavLineStakeout_t *result_out);

// Format bearing as surveyor quadrant (e.g. "S 62°15'12\" E" or "N 45°00' E")
void nav_format_quadrant_bearing(float bearing_deg, char *buf, size_t buf_len);

// Format station in surveying stationing format (e.g. "Sta 4+20.5 ft" or "Sta 1+28.2 m")
void nav_format_station(double station_m, char *buf, size_t buf_len);

// Unit Formatting Helpers (SI vs Imperial based on current_settings.use_imperial_units)
void nav_format_commas(double val, int decimals, const char *unit, char *buf, size_t buf_len);
void nav_format_distance(double meters, char *buf, int buf_len);
void nav_format_height(double meters, char *buf, int buf_len);
void nav_format_offset(float meters, char *buf, int buf_len);

// 64-point distance-filtered breadcrumb ring buffer for on-screen coarse navigation
#define NAV_BREADCRUMB_CAPACITY 64

typedef struct {
    double lat;
    double lon;
} NavBreadcrumb_t;

void nav_breadcrumb_reset(void);
bool nav_breadcrumb_push(double tip_lat, double tip_lon);
int  nav_breadcrumb_get_count(void);
bool nav_breadcrumb_get_point(int index, NavBreadcrumb_t *pt_out);

#ifdef __cplusplus
}
#endif

#endif // NAV_ENGINE_H
