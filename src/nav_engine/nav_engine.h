#ifndef NAV_ENGINE_H
#define NAV_ENGINE_H

#include "../rover_api.h"

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

// Unit Formatting Helpers (SI vs Imperial based on current_settings.use_imperial_units)
void nav_format_distance(double meters, char *buf, int buf_len);
void nav_format_height(double meters, char *buf, int buf_len);
void nav_format_offset(float meters, char *buf, int buf_len);

#ifdef __cplusplus
}
#endif

#endif // NAV_ENGINE_H
