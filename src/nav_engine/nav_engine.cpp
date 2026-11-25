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

extern "C" bool nav_compute_pole_tip_position(double master_lat,
                                              double master_lon,
                                              double master_alt_m,
                                              bool heading_valid,
                                              HeadingSource_t heading_source,
                                              float true_heading_deg,
                                              float baseline_pitch_deg,
                                              double *tip_lat_out,
                                              double *tip_lon_out,
                                              double *tip_height_m_out)
{
    double tip_lat = master_lat;
    double tip_lon = master_lon;
    double tip_alt = master_alt_m - (double)current_settings.antenna_height_m;
    bool   applied = false;

    // Apply Master->Pole-Center lever-arm correction when dual-antenna carrier phase heading is active,
    // or when opt-in calibrated rigid-beam 9-DOF compass lock is explicitly qualified.
    bool dual_antenna_locked = heading_valid &&
                               (heading_source == HDG_SRC_DUAL_FIX ||
                                heading_source == HDG_SRC_DUAL_FLT);
    bool calibrated_beam_locked = current_settings.apply_lever_arm_on_calibrated_beam &&
                                  heading_valid &&
                                  (heading_source == HDG_SRC_COMP) &&
                                  current_settings.mag_cal_valid &&
                                  current_settings.mag_ellipsoid_cal_valid;
    if (current_settings.apply_lever_arm && (dual_antenna_locked || calibrated_beam_locked) && fabs(master_lat) > 0.0001) {
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
        applied = true;
    }

    if (tip_lat_out)      *tip_lat_out      = tip_lat;
    if (tip_lon_out)      *tip_lon_out      = tip_lon;
    if (tip_height_m_out) *tip_height_m_out = tip_alt;
    return applied;
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

extern "C" void nav_compute_line_stakeout(double tip_lat,
                                          double tip_lon,
                                          double lat_a,
                                          double lon_a,
                                          double lat_b,
                                          double lon_b,
                                          NavLineStakeout_t *result_out)
{
    if (!result_out) return;

    double lat_mid = (lat_a + lat_b) * 0.5 * DEG_TO_RAD;
    double r_m = WGS84_A, r_n = WGS84_A;
    wgs84_radii(lat_mid, &r_m, &r_n);

    // Vector from Point A to Point B
    double dlat_ab = (lat_b - lat_a) * DEG_TO_RAD;
    double dlon_ab = (lon_b - lon_a) * DEG_TO_RAD;
    double dn_ab = dlat_ab * r_m;
    double de_ab = dlon_ab * r_n * cos(lat_mid);
    double total_len = sqrt(dn_ab * dn_ab + de_ab * de_ab);

    if (total_len < 0.001) {
        // Coincident points or invalid line
        result_out->valid           = false;
        result_out->station_m       = 0.0;
        result_out->offset_m        = 0.0;
        result_out->total_length_m  = 0.0;
        result_out->remaining_m     = 0.0;
        result_out->line_bearing_deg= 0.0f;
        result_out->pct_complete    = 0.0f;
        return;
    }

    // Unit vector along line A -> B
    double u_n = dn_ab / total_len;
    double u_e = de_ab / total_len;

    // Deed azimuth bearing (0..360 deg)
    double brg = atan2(de_ab, dn_ab) * RAD_TO_DEG;
    if (brg < 0.0) brg += 360.0;

    // Vector from Point A to Rover Tip R
    double dlat_ar = (tip_lat - lat_a) * DEG_TO_RAD;
    double dlon_ar = (tip_lon - lon_a) * DEG_TO_RAD;
    double dn_ar = dlat_ar * r_m;
    double de_ar = dlon_ar * r_n * cos(lat_mid);

    // Station: along-track projection onto line A -> B
    double station = dn_ar * u_n + de_ar * u_e;

    // Offset: perpendicular cross-track error (<0 = Left, >0 = Right)
    double offset = de_ar * u_n - dn_ar * u_e;

    // Remaining along-track distance to Point B
    double remaining = total_len - station;

    // Completion percentage
    float pct = (float)((station / total_len) * 100.0);

    result_out->valid           = true;
    result_out->station_m       = station;
    result_out->offset_m        = offset;
    result_out->total_length_m  = total_len;
    result_out->remaining_m     = remaining;
    result_out->line_bearing_deg= (float)brg;
    result_out->pct_complete    = pct;
}

extern "C" void nav_format_quadrant_bearing(float bearing_deg, char *buf, size_t buf_len)
{
    if (!buf || buf_len == 0) return;
    while (bearing_deg < 0.0f) bearing_deg += 360.0f;
    while (bearing_deg >= 360.0f) bearing_deg -= 360.0f;

    char ns = 'N';
    char ew = 'E';
    float angle = bearing_deg;

    if (bearing_deg >= 0.0f && bearing_deg <= 90.0f) {
        ns = 'N';
        ew = 'E';
        angle = bearing_deg;
    } else if (bearing_deg > 90.0f && bearing_deg <= 180.0f) {
        ns = 'S';
        ew = 'E';
        angle = 180.0f - bearing_deg;
    } else if (bearing_deg > 180.0f && bearing_deg <= 270.0f) {
        ns = 'S';
        ew = 'W';
        angle = bearing_deg - 180.0f;
    } else {
        ns = 'N';
        ew = 'W';
        angle = 360.0f - bearing_deg;
    }

    int deg = (int)angle;
    float rem_min = (angle - (float)deg) * 60.0f;
    int min = (int)rem_min;
    float sec = (rem_min - (float)min) * 60.0f;
    if (sec >= 59.95f) {
        sec = 0.0f;
        min++;
        if (min >= 60) {
            min = 0;
            deg++;
        }
    }

    snprintf(buf, buf_len, "%c %02d deg %02d'%04.1f\" %c", ns, deg, min, sec, ew);
}

extern "C" void nav_format_station(double station_m, char *buf, size_t buf_len)
{
    if (!buf || buf_len == 0) return;
    double dist = current_settings.use_imperial_units ? (station_m * METERS_TO_FEET) : station_m;
    const char *unit = current_settings.use_imperial_units ? "ft" : "m";

    bool neg = (dist < 0.0);
    double abs_d = fabs(dist);
    int hundred = (int)(abs_d / 100.0);
    double remainder = abs_d - (hundred * 100.0);

    snprintf(buf, buf_len, "Sta %s%d+%04.1f %s", neg ? "-" : "", hundred, remainder, unit);
}

static void format_signed_along_1dp(double meters, char *buf, size_t buf_len)
{
    if (!buf || buf_len == 0) return;
    if (current_settings.use_imperial_units) {
        double feet = meters * METERS_TO_FEET;
        if (fabs(feet) < 1.0) {
            double inches = feet * 12.0;
            if (fabs(inches) < 0.05) inches = 0.0;
            snprintf(buf, buf_len, "%+.1f in", inches);
        } else if (fabs(feet) < 1000.0) {
            snprintf(buf, buf_len, "%+.1f ft", feet);
        } else {
            char tmp[32];
            nav_format_commas(feet, 1, "ft", tmp, sizeof(tmp));
            snprintf(buf, buf_len, "%s%s", (feet >= 0.0) ? "+" : "", tmp);
        }
    } else {
        if (fabs(meters) < 1.0) {
            double cm = meters * 100.0;
            if (fabs(cm) < 0.05) cm = 0.0;
            snprintf(buf, buf_len, "%+.1f cm", cm);
        } else if (fabs(meters) < 1000.0) {
            snprintf(buf, buf_len, "%+.1f m", meters);
        } else {
            char tmp[32];
            nav_format_commas(meters, 1, "m", tmp, sizeof(tmp));
            snprintf(buf, buf_len, "%s%s", (meters >= 0.0) ? "+" : "", tmp);
        }
    }
}

extern "C" void nav_format_along_line(double station_m, float pct_complete, double remaining_m, char *buf, size_t buf_len)
{
    if (!buf || buf_len == 0) return;
    char a_str[32], b_str[32];
    format_signed_along_1dp(station_m, a_str, sizeof(a_str));
    format_signed_along_1dp(remaining_m, b_str, sizeof(b_str));
    float pct = (fabsf(pct_complete) < 0.05f) ? 0.0f : pct_complete;
    snprintf(buf, buf_len, "A: %s (%.1f%%) | Line->B: %s", a_str, pct, b_str);
}

extern "C" void nav_format_commas(double val, int decimals, const char *unit, char *buf, size_t buf_len)
{
    if (!buf || buf_len == 0) return;

    bool negative = (val < 0.0);
    double abs_val = fabs(val);

    double scale = pow(10.0, decimals);
    double rounded = round(abs_val * scale) / scale;
    uint64_t int_part = (uint64_t)rounded;
    double frac_part = rounded - (double)int_part;

    char int_str[32];
    snprintf(int_str, sizeof(int_str), "%llu", (unsigned long long)int_part);
    int int_len = strlen(int_str);

    char formatted_int[48];
    int f_idx = 0;
    for (int i = 0; i < int_len; i++) {
        if (i > 0 && ((int_len - i) % 3) == 0) {
            formatted_int[f_idx++] = ',';
        }
        formatted_int[f_idx++] = int_str[i];
    }
    formatted_int[f_idx] = '\0';

    if (decimals > 0) {
        char fmt[16];
        snprintf(fmt, sizeof(fmt), "%%.%df", decimals);
        char tmp_frac[32];
        snprintf(tmp_frac, sizeof(tmp_frac), fmt, frac_part);
        const char *dot_frac = strchr(tmp_frac, '.');
        if (!dot_frac) dot_frac = ".0";

        if (unit && unit[0]) {
            snprintf(buf, buf_len, "%s%s%s %s", negative ? "-" : "", formatted_int, dot_frac, unit);
        } else {
            snprintf(buf, buf_len, "%s%s%s", negative ? "-" : "", formatted_int, dot_frac);
        }
    } else {
        if (unit && unit[0]) {
            snprintf(buf, buf_len, "%s%s %s", negative ? "-" : "", formatted_int, unit);
        } else {
            snprintf(buf, buf_len, "%s%s", negative ? "-" : "", formatted_int);
        }
    }
}

extern "C" void nav_format_distance(double meters, char *buf, int buf_len)
{
    if (!current_settings.use_imperial_units) {
        if (fabs(meters) < 2.0) {
            snprintf(buf, buf_len, "%.1f cm", meters * 100.0);
        } else if (fabs(meters) < 100.0) {
            snprintf(buf, buf_len, "%.2f m", meters);
        } else if (fabs(meters) < 1000.0) {
            snprintf(buf, buf_len, "%.1f m", meters);
        } else {
            nav_format_commas(meters, 1, "m", buf, (size_t)buf_len);
        }
    } else {
        double feet = meters * METERS_TO_FEET;
        if (fabs(meters) < 2.0) {
            double inches = feet * 12.0;
            snprintf(buf, buf_len, "%.1f in", inches);
        } else if (fabs(feet) < 1000.0) {
            snprintf(buf, buf_len, "%.2f ft", feet);
        } else {
            nav_format_commas(feet, 2, "ft", buf, (size_t)buf_len);
        }
    }
}

extern "C" void nav_format_height(double meters, char *buf, int buf_len)
{
    if (!current_settings.use_imperial_units) {
        if (fabs(meters) < 1000.0) {
            snprintf(buf, buf_len, "%.3f m", meters);
        } else {
            nav_format_commas(meters, 3, "m", buf, (size_t)buf_len);
        }
    } else {
        double feet = meters * METERS_TO_FEET;
        if (fabs(feet) < 1000.0) {
            snprintf(buf, buf_len, "%.2f ft", feet);
        } else {
            nav_format_commas(feet, 2, "ft", buf, (size_t)buf_len);
        }
    }
}

extern "C" void nav_format_offset(float meters, char *buf, int buf_len)
{
    if (!current_settings.use_imperial_units) {
        if (fabsf(meters) < 2.0f) {
            snprintf(buf, buf_len, "%+.1f cm", meters * 100.0f);
        } else if (fabsf(meters) < 1000.0f) {
            snprintf(buf, buf_len, "%+.2f m", meters);
        } else {
            char tmp[32];
            nav_format_commas((double)meters, 2, "m", tmp, sizeof(tmp));
            snprintf(buf, buf_len, "%s%s", (meters >= 0.0f) ? "+" : "", tmp);
        }
    } else {
        float feet = meters * (float)METERS_TO_FEET;
        if (fabsf(meters) < 2.0f) {
            snprintf(buf, buf_len, "%+.1f in", feet * 12.0f);
        } else if (fabsf(feet) < 1000.0f) {
            snprintf(buf, buf_len, "%+.2f ft", feet);
        } else {
            char tmp[32];
            nav_format_commas((double)feet, 2, "ft", tmp, sizeof(tmp));
            snprintf(buf, buf_len, "%s%s", (feet >= 0.0f) ? "+" : "", tmp);
        }
    }
}

// -----------------------------------------------------------------------------
// Breadcrumb Ring Buffer
// -----------------------------------------------------------------------------
static NavBreadcrumb_t s_breadcrumbs[NAV_BREADCRUMB_CAPACITY];
static int s_bc_head = 0;
static int s_bc_count = 0;
static double s_bc_last_lat = 0.0;
static double s_bc_last_lon = 0.0;
static bool   s_bc_has_last = false;

extern "C" void nav_breadcrumb_reset(void)
{
    s_bc_head = 0;
    s_bc_count = 0;
    s_bc_has_last = false;
}

extern "C" bool nav_breadcrumb_push(double tip_lat, double tip_lon)
{
    if (fabs(tip_lat) < 0.0001 && fabs(tip_lon) < 0.0001) return false;

    if (s_bc_has_last) {
        // Compute displacement from last recorded breadcrumb point
        double lat1 = s_bc_last_lat * DEG_TO_RAD;
        double lon1 = s_bc_last_lon * DEG_TO_RAD;
        double lat2 = tip_lat * DEG_TO_RAD;
        double lon2 = tip_lon * DEG_TO_RAD;
        double r_m = WGS84_A, r_n = WGS84_A;
        wgs84_radii((lat1 + lat2) * 0.5, &r_m, &r_n);
        double dn = (lat2 - lat1) * r_m;
        double de = (lon2 - lon1) * r_n * cos((lat1 + lat2) * 0.5);
        double dist = sqrt(dn * dn + de * de);

        // Distance threshold: require > 0.5 meters of movement
        if (dist < 0.5) {
            return false;
        }
    }

    s_breadcrumbs[s_bc_head].lat = tip_lat;
    s_breadcrumbs[s_bc_head].lon = tip_lon;
    s_bc_head = (s_bc_head + 1) % NAV_BREADCRUMB_CAPACITY;
    if (s_bc_count < NAV_BREADCRUMB_CAPACITY) {
        s_bc_count++;
    }

    s_bc_last_lat = tip_lat;
    s_bc_last_lon = tip_lon;
    s_bc_has_last = true;
    return true;
}

extern "C" int nav_breadcrumb_get_count(void)
{
    return s_bc_count;
}

extern "C" bool nav_breadcrumb_get_point(int index, NavBreadcrumb_t *pt_out)
{
    if (!pt_out || index < 0 || index >= s_bc_count) return false;
    // index 0 is oldest, index count-1 is newest
    int start_idx = (s_bc_count < NAV_BREADCRUMB_CAPACITY) ? 0 : s_bc_head;
    int actual_idx = (start_idx + index) % NAV_BREADCRUMB_CAPACITY;
    *pt_out = s_breadcrumbs[actual_idx];
    return true;
}

// =========================================================================
// Phase 5 & Phase 8: Deep-Woods Heading Fallback Engine
// (Kinematic Displacement + 9-DOF MMC5983MA/QMI8658 Compass + Gyro Yaw)
// =========================================================================
static double   s_anchor_lat       = 0.0;
static double   s_anchor_lon       = 0.0;
static bool     s_has_anchor       = false;
static float    s_fused_heading    = 0.0f;
static bool     s_fused_valid      = false;
static HeadingSource_t s_fused_src = HDG_SRC_NONE;
static uint32_t s_last_update_ms   = 0;
static uint32_t s_last_disp_ms     = 0;
static uint32_t s_last_tare_save_ms= 0;
static float    s_last_saved_tare  = -999.0f;
static float    s_prev_mag_raw_hdg = 0.0f;
static bool     s_has_prev_mag     = false;
static float    s_innov_deg        = 0.0f;
static uint32_t s_innov_bad_since_ms = 0;
static bool     s_anom_latched     = false;
static uint32_t s_clean_since_ms   = 0;

static float wrap_angle_180(float deg)
{
    while (deg > 180.0f)  deg -= 360.0f;
    while (deg < -180.0f) deg += 360.0f;
    return deg;
}

static float wrap_angle_360(float deg)
{
    while (deg < 0.0f)    deg += 360.0f;
    while (deg >= 360.0f) deg -= 360.0f;
    return deg;
}

extern "C" void nav_heading_displacement_reset(void)
{
    s_has_anchor         = false;
    s_fused_valid        = false;
    s_fused_src          = HDG_SRC_NONE;
    s_last_update_ms     = 0;
    s_last_disp_ms       = 0;
    s_has_prev_mag       = false;
    s_innov_deg          = 0.0f;
    s_innov_bad_since_ms = 0;
    s_anom_latched       = false;
    s_clean_since_ms     = 0;
}

extern "C" void nav_heading_beam_update(const NavHeadingSensorInput_t *in,
                                        bool *heading_valid_out,
                                        float *true_heading_out,
                                        HeadingSource_t *heading_src_out,
                                        float *disp_dist_out,
                                        float *mag_true_heading_out,
                                        bool *mag_anomaly_out)
{
    if (!in) {
        return;
    }

    float dt_s = (s_last_update_ms > 0 && in->now_ms >= s_last_update_ms)
                 ? ((float)(in->now_ms - s_last_update_ms) / 1000.0f)
                 : 0.033f;
    s_last_update_ms = in->now_ms;

    bool  mag_anomaly  = false;
    float mag_true_hdg = 0.0f;

    // 1. Evaluate Triple-Condition Iron Anomaly Gatekeeper & True-North Online Teacher
    if (in->mag_present && in->mag_valid) {
        bool plausible_earth = (in->mag_total_uT >= 25.0f && in->mag_total_uT <= 75.0f);
        float ref_uT = (current_settings.mag_field_ref_uT > 10.0f) ? current_settings.mag_field_ref_uT : 52.5f;
        bool mag_spike = (!plausible_earth) ||
                         (current_settings.mag_cal_valid && fabsf(in->mag_total_uT - ref_uT) > 3.5f);

        bool bz_spike = (in->has_bz_world &&
                         current_settings.mag_cal_valid &&
                         fabsf(current_settings.mag_bz_ref_uT) > 5.0f &&
                         fabsf(in->mag_bz_world_uT - current_settings.mag_bz_ref_uT) > 3.0f);

        bool innov_trip = false;
        if (s_has_prev_mag && dt_s > 0.0f && dt_s < 0.5f && plausible_earth) {
            float d_mag = wrap_angle_180(in->raw_mag_heading_deg - s_prev_mag_raw_hdg);
            float d_gyr = in->yaw_rate_cw_dps * dt_s;
            float innov_step = wrap_angle_180(d_mag - d_gyr);
            s_innov_deg = s_innov_deg * 0.88f + innov_step;
            if (s_innov_deg > 90.0f)  s_innov_deg = 90.0f;
            if (s_innov_deg < -90.0f) s_innov_deg = -90.0f;
            if (fabsf(s_innov_deg) > 4.0f) {
                if (s_innov_bad_since_ms == 0) {
                    s_innov_bad_since_ms = in->now_ms;
                }
                innov_trip = ((in->now_ms - s_innov_bad_since_ms) >= 200U);
            } else {
                s_innov_bad_since_ms = 0;
            }
        } else {
            s_innov_bad_since_ms = 0;
        }
        s_prev_mag_raw_hdg = in->raw_mag_heading_deg;
        s_has_prev_mag     = true;

        bool raw_anom = (mag_spike || bz_spike || innov_trip);
        if (raw_anom) {
            s_anom_latched   = true;
            s_clean_since_ms = 0;
        } else if (s_anom_latched) {
            if (s_clean_since_ms == 0) {
                s_clean_since_ms = in->now_ms;
            } else if ((in->now_ms - s_clean_since_ms) >= 500U) {
                s_anom_latched = false;
                s_innov_deg    = 0.0f;
            }
        }
        mag_anomaly = s_anom_latched;

        // Continuous True-North Auto-Calibration whenever UM982 has Integer Dual-Antenna HDG FIX
        if (in->dual_antenna_valid && in->dual_antenna_fixed &&
            plausible_earth && !mag_spike && in->pole_tilt_deg < 15.0f) {
            float measured_offset = wrap_angle_180(in->dual_heading_deg - in->raw_mag_heading_deg);
            if (!current_settings.mag_cal_valid) {
                current_settings.mag_true_north_offset_deg = measured_offset;
                current_settings.mag_field_ref_uT          = in->mag_total_uT;
                if (in->has_bz_world) {
                    current_settings.mag_bz_ref_uT = in->mag_bz_world_uT;
                }
                if (in->is_beam_10dof && fabsf(in->um982_baseline_pitch_deg) < 25.0f) {
                    current_settings.beam_pitch_tare_deg = in->beam_pitch_deg - in->um982_baseline_pitch_deg;
                }
                current_settings.mag_cal_valid = true;
                s_last_saved_tare              = measured_offset;
                s_last_tare_save_ms            = in->now_ms;
                mark_settings_dirty();
            } else {
                float err_deg = wrap_angle_180(measured_offset - current_settings.mag_true_north_offset_deg);
                // Reject sudden magnetic spikes (>15 deg jump in a single epoch) during auto-tare
                if (fabsf(err_deg) < 15.0f && !raw_anom) {
                    current_settings.mag_true_north_offset_deg =
                        wrap_angle_180(current_settings.mag_true_north_offset_deg + 0.05f * err_deg);
                    current_settings.mag_field_ref_uT =
                        0.95f * current_settings.mag_field_ref_uT + 0.05f * in->mag_total_uT;
                    if (in->has_bz_world) {
                        if (fabsf(current_settings.mag_bz_ref_uT) < 1.0f) {
                            current_settings.mag_bz_ref_uT = in->mag_bz_world_uT;
                        } else {
                            current_settings.mag_bz_ref_uT =
                                0.95f * current_settings.mag_bz_ref_uT + 0.05f * in->mag_bz_world_uT;
                        }
                    }
                    if (in->is_beam_10dof && fabsf(in->um982_baseline_pitch_deg) < 25.0f) {
                        float pitch_err = in->beam_pitch_deg - in->um982_baseline_pitch_deg;
                        current_settings.beam_pitch_tare_deg =
                            0.95f * current_settings.beam_pitch_tare_deg + 0.05f * pitch_err;
                    }

                    if (fabsf(wrap_angle_180(current_settings.mag_true_north_offset_deg - s_last_saved_tare)) >= 0.5f &&
                        (in->now_ms - s_last_tare_save_ms >= 60000U)) {
                        s_last_saved_tare   = current_settings.mag_true_north_offset_deg;
                        s_last_tare_save_ms = in->now_ms;
                        mark_settings_dirty();
                    }
                }
            }
        }

        mag_true_hdg = wrap_angle_360(in->raw_mag_heading_deg + current_settings.mag_true_north_offset_deg);
    } else {
        s_has_prev_mag       = false;
        s_innov_deg          = 0.0f;
        s_innov_bad_since_ms = 0;
        s_anom_latched       = false;
        s_clean_since_ms     = 0;
    }

    if (mag_true_heading_out) *mag_true_heading_out = mag_true_hdg;
    if (mag_anomaly_out)      *mag_anomaly_out      = mag_anomaly;

    // 2. Tier 1: Dual-Antenna Primary Source Available (HDG_SRC_DUAL_FIX / HDG_SRC_DUAL_FLT)
    // Clear s_last_disp_ms = 0 so SLV dropouts switch instantaneously (0.0 s) to [CMP] instead of stale WALK hold.
    bool compass_healthy = (in->mag_present && in->mag_valid && !mag_anomaly);
    bool dual_flt_bl_ok = true;
    if (!in->dual_antenna_fixed && compass_healthy && in->is_beam_10dof &&
        current_settings.antenna_spacing_m > 0.05f) {
        if (in->dual_baseline_length_m <= 0.01f ||
            fabsf(in->dual_baseline_length_m - current_settings.antenna_spacing_m) > 0.10f) {
            dual_flt_bl_ok = false;
        }
    }
    if (in->dual_antenna_valid && dual_flt_bl_ok) {
        s_anchor_lat         = in->tip_lat;
        s_anchor_lon         = in->tip_lon;
        s_has_anchor         = true;
        s_fused_heading      = in->dual_heading_deg;
        s_fused_valid        = true;
        s_fused_src          = in->dual_antenna_fixed ? HDG_SRC_DUAL_FIX : HDG_SRC_DUAL_FLT;
        s_last_disp_ms       = 0;
        s_innov_deg          = 0.0f;
        s_innov_bad_since_ms = 0;
        if (heading_valid_out) *heading_valid_out = true;
        if (true_heading_out)  *true_heading_out  = in->dual_heading_deg;
        if (heading_src_out)   *heading_src_out   = s_fused_src;
        if (disp_dist_out)     *disp_dist_out     = 0.0f;
        return;
    }

    // 3. Tier 2..4: Woods Fallback (Slave Antenna lock lost under canopy)
    if (!s_has_anchor || s_anchor_lat == 0.0 || s_anchor_lon == 0.0) {
        s_anchor_lat   = in->tip_lat;
        s_anchor_lon   = in->tip_lon;
        s_has_anchor   = (in->tip_lat != 0.0 && in->tip_lon != 0.0);
        s_last_disp_ms = 0;
    }

    // Compute displacement distance and geodetic delta from anchor
    double lat1 = s_anchor_lat * DEG_TO_RAD;
    double lat2 = in->tip_lat * DEG_TO_RAD;
    double lon1 = s_anchor_lon * DEG_TO_RAD;
    double lon2 = in->tip_lon * DEG_TO_RAD;
    double r_m = WGS84_A, r_n = WGS84_A;
    wgs84_radii((lat1 + lat2) * 0.5, &r_m, &r_n);
    double dn = (lat2 - lat1) * r_m;
    double de = (lon2 - lon1) * r_n * cos((lat1 + lat2) * 0.5);
    double dist_m = sqrt(dn * dn + de * de);

    // Displacement trigger: Moving >= 0.70m (~2 to 3 paces)
    if (dist_m >= 0.70) {
        double az_rad = atan2(de, dn);
        double az_deg = az_rad * (180.0 / M_PI);
        if (az_deg < 0.0) az_deg += 360.0;
        s_fused_heading = (float)az_deg;
        s_fused_valid   = true;
        s_fused_src     = HDG_SRC_DISP;
        s_last_disp_ms  = in->now_ms;
        s_anchor_lat = in->tip_lat;
        s_anchor_lon = in->tip_lon;
        dist_m = 0.0;
    } else {
        bool in_walk_window = (s_fused_valid && s_last_disp_ms > 0 && (in->now_ms - s_last_disp_ms <= 3000U));
        float rot_thresh_dps = in->is_beam_10dof ? 1.0f : 1.5f;
        bool is_rotating = (fabsf(in->yaw_rate_cw_dps) >= rot_thresh_dps);
        if (in_walk_window) {
            s_fused_src = HDG_SRC_DISP;
        } else if (in->mag_present && in->mag_valid && !mag_anomaly) {
            // Tier 3: 9-DOF Tilt-Compensated Beam (SEN0696) or MMC5983MA Compass + Gyro Fusion
            if (!s_fused_valid) {
                s_fused_heading = mag_true_hdg;
                s_fused_valid   = true;
            } else {
                float pred = s_fused_heading;
                float comp_thresh = in->is_beam_10dof ? 0.25f : 1.5f;
                if (fabsf(in->yaw_rate_cw_dps) >= comp_thresh && dt_s > 0.0f && dt_s < 0.5f) {
                    pred = wrap_angle_360(pred + in->yaw_rate_cw_dps * dt_s);
                }
                float mag_err = wrap_angle_180(mag_true_hdg - pred);
                float mag_alpha = (fabsf(mag_err) > 15.0f) ? 0.20f : (!is_rotating ? 0.12f : 0.05f);
                s_fused_heading = wrap_angle_360(pred + mag_alpha * mag_err);
            }
            s_fused_src = HDG_SRC_COMP;
        } else if (s_fused_valid) {
            // Tier 4: 6-DOF Gyro / Hold Fallback (when magnetometer absent or during iron anomaly)
            if (is_rotating) {
                if (dt_s > 0.0f && dt_s < 0.5f) {
                    s_fused_heading = wrap_angle_360(s_fused_heading + in->yaw_rate_cw_dps * dt_s);
                }
                s_fused_src = HDG_SRC_GYRO;
            } else {
                s_fused_src = HDG_SRC_HOLD;
            }
        }
    }
    if (heading_valid_out) *heading_valid_out = s_fused_valid;
    if (true_heading_out)  *true_heading_out  = s_fused_heading;
    if (heading_src_out)   *heading_src_out   = s_fused_src;
    if (disp_dist_out)     *disp_dist_out     = (float)dist_m;
}

extern "C" void nav_heading_9dof_update(double tip_lat, double tip_lon, float gyr_z_dps, float pole_tilt_deg,
                                        bool mag_present, bool mag_valid, float raw_mag_heading_deg,
                                        float mag_total_uT, uint32_t now_ms, bool dual_antenna_valid,
                                        bool dual_antenna_fixed, float dual_heading_deg,
                                        bool *heading_valid_out, float *true_heading_out,
                                        HeadingSource_t *heading_src_out, float *disp_dist_out,
                                        float *mag_true_heading_out, bool *mag_anomaly_out)
{
    NavHeadingSensorInput_t in = {};
    in.tip_lat             = tip_lat;
    in.tip_lon             = tip_lon;
    in.yaw_rate_cw_dps     = gyr_z_dps;
    in.pole_tilt_deg       = pole_tilt_deg;
    in.mag_present         = mag_present;
    in.mag_valid           = mag_valid;
    in.raw_mag_heading_deg = raw_mag_heading_deg;
    in.mag_total_uT        = mag_total_uT;
    in.now_ms              = now_ms;
    in.dual_antenna_valid  = dual_antenna_valid;
    in.dual_antenna_fixed  = dual_antenna_fixed;
    in.dual_heading_deg    = dual_heading_deg;
    nav_heading_beam_update(&in, heading_valid_out, true_heading_out, heading_src_out,
                            disp_dist_out, mag_true_heading_out, mag_anomaly_out);
}

extern "C" void nav_heading_displacement_update(double tip_lat, double tip_lon, float gyr_z_dps,
                                                uint32_t now_ms, bool dual_antenna_valid,
                                                bool dual_antenna_fixed, float dual_heading_deg,
                                                bool *heading_valid_out, float *true_heading_out,
                                                HeadingSource_t *heading_src_out, float *disp_dist_out)
{
    nav_heading_9dof_update(tip_lat, tip_lon, gyr_z_dps, 0.0f,
                            false, false, 0.0f, 0.0f, now_ms,
                            dual_antenna_valid, dual_antenna_fixed, dual_heading_deg,
                            heading_valid_out, true_heading_out, heading_src_out, disp_dist_out,
                            NULL, NULL);
}
