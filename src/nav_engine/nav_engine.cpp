#include "nav_engine.h"
#include "../settings_manager/settings_manager.h"
#include <math.h>
#include <stdio.h>

#define WGS84_A  6378137.0
#define WGS84_E2 0.00669437999014
#ifndef DEG_TO_RAD
#define DEG_TO_RAD (M_PI / 180.0)
#endif
#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0 / M_PI)
#endif
#define METERS_TO_FEET 3.280839895

static void wgs84_radii(double lat_rad, double *r_meridian, double *r_prime_vertical)
{
    double sin_lat = sin(lat_rad);
    double denom   = sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat);
    *r_meridian       = WGS84_A * (1.0 - WGS84_E2) / (denom * denom * denom);
    *r_prime_vertical = WGS84_A / denom;
}

extern "C" void nav_compute_pole_tip_position(double master_lat,
                                              double master_lon,
                                              double master_alt_m,
                                              bool heading_valid,
                                              float true_heading_deg,
                                              float baseline_pitch_deg,
                                              double *tip_lat_out,
                                              double *tip_lon_out,
                                              double *tip_height_m_out)
{
    double tip_lat = master_lat;
    double tip_lon = master_lon;
    double tip_alt = master_alt_m - (double)current_settings.antenna_height_m;

    // Apply Master->Pole-Center lever-arm correction when dual-antenna heading is valid
    if (current_settings.apply_lever_arm && heading_valid && fabs(master_lat) > 0.0001) {
        double half_baseline_m = (double)current_settings.antenna_spacing_m * 0.5;
        double az_rad    = (double)(true_heading_deg + current_settings.baseline_mount_offset_deg) * DEG_TO_RAD;
        double pitch_rad = (double)baseline_pitch_deg * DEG_TO_RAD;

        double horiz_m = half_baseline_m * cos(pitch_rad);
        double d_north = horiz_m * cos(az_rad);
        double d_east  = horiz_m * sin(az_rad);
        double d_up    = half_baseline_m * sin(pitch_rad);

        double lat_rad = master_lat * DEG_TO_RAD;
        double r_m = WGS84_A, r_n = WGS84_A;
        wgs84_radii(lat_rad, &r_m, &r_n);

        tip_lat = master_lat + (d_north / (r_m + master_alt_m)) * RAD_TO_DEG;
        double cos_lat = cos(lat_rad);
        if (fabs(cos_lat) > 1e-6) {
            tip_lon = master_lon + (d_east / ((r_n + master_alt_m) * cos_lat)) * RAD_TO_DEG;
        }
        tip_alt += d_up;
    }

    if (tip_lat_out)      *tip_lat_out      = tip_lat;
    if (tip_lon_out)      *tip_lon_out      = tip_lon;
    if (tip_height_m_out) *tip_height_m_out = tip_alt;
}

extern "C" void nav_compute_target_guidance(double tip_lat,
                                            double tip_lon,
                                            float rover_heading_deg,
                                            double target_lat,
                                            double target_lon,
                                            double *distance_m_out,
                                            float *bearing_deg_out,
                                            float *rel_bearing_deg_out,
                                            float *d_north_m_out,
                                            float *d_east_m_out)
{
    double lat1 = tip_lat * DEG_TO_RAD;
    double lon1 = tip_lon * DEG_TO_RAD;
    double lat2 = target_lat * DEG_TO_RAD;
    double lon2 = target_lon * DEG_TO_RAD;

    double dlat = lat2 - lat1;
    double dlon = lon2 - lon1;

    // Local tangent plane ENU accurate to <1mm at short range, Haversine for long range
    double r_m = WGS84_A, r_n = WGS84_A;
    wgs84_radii((lat1 + lat2) * 0.5, &r_m, &r_n);

    double d_north = dlat * r_m;
    double d_east  = dlon * r_n * cos((lat1 + lat2) * 0.5);
    double dist_m  = sqrt(d_north * d_north + d_east * d_east);

    double brg_deg = atan2(d_east, d_north) * RAD_TO_DEG;
    if (brg_deg < 0.0) brg_deg += 360.0;

    float rel_brg = (float)brg_deg - rover_heading_deg;
    while (rel_brg > 180.0f)  rel_brg -= 360.0f;
    while (rel_brg < -180.0f) rel_brg += 360.0f;

    if (distance_m_out)     *distance_m_out     = dist_m;
    if (bearing_deg_out)    *bearing_deg_out    = (float)brg_deg;
    if (rel_bearing_deg_out)*rel_bearing_deg_out= rel_brg;
    if (d_north_m_out)      *d_north_m_out      = (float)d_north;
    if (d_east_m_out)       *d_east_m_out       = (float)d_east;
}

extern "C" void nav_format_distance(double meters, char *buf, int buf_len)
{
    if (!current_settings.use_imperial_units) {
        if (fabs(meters) < 2.0) {
            snprintf(buf, buf_len, "%.1f cm", meters * 100.0);
        } else if (fabs(meters) < 100.0) {
            snprintf(buf, buf_len, "%.2f m", meters);
        } else {
            snprintf(buf, buf_len, "%.1f m", meters);
        }
    } else {
        double feet = meters * METERS_TO_FEET;
        if (fabs(meters) < 2.0) {
            double inches = feet * 12.0;
            snprintf(buf, buf_len, "%.1f in", inches);
        } else {
            snprintf(buf, buf_len, "%.2f ft", feet);
        }
    }
}

extern "C" void nav_format_height(double meters, char *buf, int buf_len)
{
    if (!current_settings.use_imperial_units) {
        snprintf(buf, buf_len, "%.3f m", meters);
    } else {
        snprintf(buf, buf_len, "%.2f ft", meters * METERS_TO_FEET);
    }
}

extern "C" void nav_format_offset(float meters, char *buf, int buf_len)
{
    if (!current_settings.use_imperial_units) {
        if (fabsf(meters) < 2.0f) {
            snprintf(buf, buf_len, "%+.1f cm", meters * 100.0f);
        } else {
            snprintf(buf, buf_len, "%+.2f m", meters);
        }
    } else {
        float feet = meters * (float)METERS_TO_FEET;
        if (fabsf(meters) < 2.0f) {
            snprintf(buf, buf_len, "%+.1f in", feet * 12.0f);
        } else {
            snprintf(buf, buf_len, "%+.2f ft", feet);
        }
    }
}
