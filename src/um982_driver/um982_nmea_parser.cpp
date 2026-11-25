#include "um982_driver_internal.h"

RoverUIData_t          gnss_state               = {};
Um982PortStats_t       port_stats               = {};
volatile GpsTimeSync_t gps_time_sync            = {};
bool                   s_com1_rx_backlog_active = false;

static char     com1_line_buf[512];
static uint16_t com1_line_pos      = 0;
static bool     com1_line_overflow = false;

static bool              rtc_synced_from_gnss   = false;
static volatile bool     rtc_sync_pending       = false;
static volatile time_t   s_pending_rtc_epoch    = 0;
static volatile uint32_t s_pending_rtc_sync_ms  = 0;
static uint32_t          s_last_rtc_chip_sync_ms = 0;
static uint32_t          s_last_sysclock_sync_ms = 0;
static uint32_t          s_last_com1_poll_ms     = 0;
static uint32_t          s_last_crc_warn_ms      = 0;

static uint8_t  gsv_sats_gp = 0;
static uint8_t  gsv_sats_gl = 0;
static uint8_t  gsv_sats_ga = 0;
static uint8_t  gsv_sats_gb = 0;
static uint8_t  gsv_sats_gq = 0;
static uint32_t gsv_last_gp_ms = 0;
static uint32_t gsv_last_gl_ms = 0;
static uint32_t gsv_last_ga_ms = 0;
static uint32_t gsv_last_gb_ms = 0;
static uint32_t gsv_last_gq_ms = 0;
static uint32_t s_last_offline_probe_ms = 0;
static bool     s_was_com1_online       = false;
static uint32_t s_gga_seq               = 0;
static uint32_t s_last_gst_ms           = 0;

static int parse_hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return (int)(c - '0');
    if (c >= 'a' && c <= 'f') return (int)(c - 'a' + 10);
    if (c >= 'A' && c <= 'F') return (int)(c - 'A' + 10);
    return -1;
}

static double parse_nmea_deg_min(const char *str, const char *hemisphere)
{
    if (!str || str[0] == '\0') return 0.0;
    double raw = strtod(str, NULL);
    int deg = (int)(raw / 100.0);
    double min = raw - (deg * 100.0);
    double dec = (double)deg + (min / 60.0);
    if (hemisphere && (hemisphere[0] == 'S' || hemisphere[0] == 'W')) {
        dec = -dec;
    }
    return dec;
}

static int split_csv_fields(char *line, char **fields, int max_fields)
{
    if (!line || max_fields == 0) return 0;
    int count = 0;
    char *p = line;
    fields[count++] = p;
    while (*p && count < max_fields) {
        if (*p == ',' || *p == '*') {
            char sep = *p;
            *p = '\0';
            fields[count++] = p + 1;
            if (sep == '*') break;
        }
        p++;
    }
    return count;
}

static void update_gps_utc_sync_from_nmea_field(const char *time_field, uint32_t now_ms)
{
    if (!time_field || strlen(time_field) < 6) return;
    for (int i = 0; i < 6; i++) {
        if (time_field[i] < '0' || time_field[i] > '9') return;
    }

    uint32_t hh = (uint32_t)((time_field[0] - '0') * 10 + (time_field[1] - '0'));
    uint32_t mm = (uint32_t)((time_field[2] - '0') * 10 + (time_field[3] - '0'));
    uint32_t ss = (uint32_t)((time_field[4] - '0') * 10 + (time_field[5] - '0'));
    if (hh >= 24U || mm >= 60U || ss >= 60U) return;

    uint32_t frac_ms = 0;
    if (time_field[6] == '.' && time_field[7] != '\0') {
        double frac = atof(time_field + 6);
        if (frac >= 0.0 && frac < 1.0) {
            frac_ms = (uint32_t)(frac * 1000.0 + 0.5);
            if (frac_ms >= 1000U) frac_ms = 999U;
        }
    }

    // Anchor last_sync_millis to the exact integer UTC second boundary (.000),
    // compensating for fractional epoch timestamp and ~15 ms NMEA serial transit latency.
    gps_time_sync.last_utc_sec     = (hh * 3600U) + (mm * 60U) + ss;
    gps_time_sync.last_sync_millis = now_ms - frac_ms - 15U;
    gps_time_sync.time_synced      = true;
}

static void handle_gga_sentence(char *line)
{
    char *f[16] = {0};
    int n = split_csv_fields(line, f, 16);
    if (n < 14) return;

    uint32_t now_ms = millis();
    gps_time_sync.last_gga_ms = now_ms;
    s_gga_seq++;
    gnss_state.gga_seq = s_gga_seq;

    int qual = f[6] ? atoi(f[6]) : 0;
    if (qual > 0 && f[1] && f[1][0] && !s_com1_rx_backlog_active && Serial1.available() < 180) {
        update_gps_utc_sync_from_nmea_field(f[1], now_ms);
    }
    gnss_state.fix_type = (GnssFixType_t)qual;
    gnss_state.num_sats = f[7] ? (uint8_t)atoi(f[7]) : 0;
    if (f[8] && f[8][0]) gnss_state.hdop = (float)atof(f[8]);

    if (qual > 0 && f[2] && f[2][0] && f[4] && f[4][0]) {
        gnss_state.raw_lat   = parse_nmea_deg_min(f[2], f[3]);
        gnss_state.raw_lon   = parse_nmea_deg_min(f[4], f[5]);
        gnss_state.raw_alt_m = strtod(f[9], NULL);
    }

    if (f[13] && f[13][0]) {
        gnss_state.diff_age_s = (float)atof(f[13]);
    } else {
        gnss_state.diff_age_s = 0.0f;
    }
    if (n > 14 && f[14] && f[14][0]) {
        gnss_state.base_station_id = (uint16_t)atoi(f[14]);
    }

    bool gst_fresh = (s_last_gst_ms != 0U) && ((now_ms - s_last_gst_ms) <= 2500U);
    if (qual == GNSS_FIX_NONE || !gst_fresh || gnss_state.h_acc_m <= 0.0001f) {
        gnss_state.h_acc_from_gst = false;
        if (qual == GNSS_FIX_RTK_FIXED)      gnss_state.h_acc_m = 0.014f;
        else if (qual == GNSS_FIX_RTK_FLOAT) gnss_state.h_acc_m = 0.180f;
        else if (qual == GNSS_FIX_DGPS)      gnss_state.h_acc_m = 0.650f;
        else if (qual == GNSS_FIX_SINGLE)    gnss_state.h_acc_m = 1.500f;
        else                                 gnss_state.h_acc_m = 99.9f;
    }
}

static void handle_ggah_sentence(char *line)
{
    char *f[16] = {0};
    int n = split_csv_fields(line, f, 16);
    if (n < 9) return;

    int qual = f[6] ? atoi(f[6]) : 0;
    // Note: Only record single/DGPS standalone qual; moving-baseline RTK status is governed by UNIHEADINGA
    if (gnss_state.slave_fix_type == GNSS_FIX_NONE) {
        if (qual == 1)      gnss_state.slave_fix_type = GNSS_FIX_SINGLE;
        else if (qual == 2) gnss_state.slave_fix_type = GNSS_FIX_DGPS;
    }
    gnss_state.slave_num_sats = f[7] ? (uint8_t)atoi(f[7]) : 0;
    if (f[8] && f[8][0]) gnss_state.slave_hdop  = (float)atof(f[8]);
    if (f[9] && f[9][0]) gnss_state.slave_alt_m = (float)atof(f[9]);

    if (f[2] && f[2][0] && f[4] && f[4][0]) {
        gnss_state.slave_lat = parse_nmea_deg_min(f[2], f[3]);
        gnss_state.slave_lon = parse_nmea_deg_min(f[4], f[5]);
    }
}

static time_t utc_ymdhms_to_epoch(int year, int month, int day, int hour, int minute, int second)
{
    int y = year - (month <= 2 ? 1 : 0);
    int era = (y >= 0 ? y : y - 399) / 400;
    int yoe = y - era * 400;
    int doy = (153 * (month + (month > 2 ? -3 : 9)) + 2) / 5 + day - 1;
    int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    int64_t days = (int64_t)era * 146097LL + (int64_t)doe - 719468LL;
    return (time_t)(days * 86400LL + (int64_t)hour * 3600LL + (int64_t)minute * 60LL + (int64_t)second);
}

static void handle_rmc_sentence(char *line)
{
    char *f[14] = {0};
    int n = split_csv_fields(line, f, 14);
    if (n < 10) return;

    uint32_t now_ms = millis();
    gps_time_sync.last_rmc_ms = now_ms;

    // Field 2: Status ('A'=Active, 'V'=Void, or 'D'=Differential)
    bool pos_active  = (f[2] && (f[2][0] == 'A' || f[2][0] == 'D'));
    bool clock_fresh = (!s_com1_rx_backlog_active && Serial1.available() < 180);
    if (pos_active && f[1] && f[1][0] && clock_fresh) {
        update_gps_utc_sync_from_nmea_field(f[1], now_ms);
    }

    // Field 7: Speed over ground in knots (1 knot = 0.514444 m/s)
    if (f[7] && f[7][0]) {
        gnss_state.speed_m_s = (float)atof(f[7]) * 0.514444f;
    }

    // Field 8: Track angle / Course Over Ground (COG) in degrees True
    if (pos_active && f[8] && f[8][0]) {
        gnss_state.cog_deg = (float)atof(f[8]);

        // Kinematic Motion Fallback:
        // If dual-antenna baseline heading is invalid or currently steering via COG/HOLD:
        if (!gnss_state.heading_valid || gnss_state.heading_source == HDG_SRC_COG || gnss_state.heading_source == HDG_SRC_HOLD) {
            if (gnss_state.speed_m_s >= 0.40f) {
                // Moving forward at normal walking pace (> 0.40 m/s / ~0.8 knots / ~1.4 km/h)
                gnss_state.true_heading_deg = gnss_state.cog_deg;
                gnss_state.heading_valid    = true;
                gnss_state.heading_is_fixed = false;
                gnss_state.heading_source   = HDG_SRC_COG;
            } else if (gnss_state.heading_source == HDG_SRC_COG && gnss_state.speed_m_s < 0.20f) {
                // Surveyor has stopped walking; hold the last confirmed heading
                gnss_state.heading_source = HDG_SRC_HOLD;
            }
        }
    }

    if (pos_active && clock_fresh && strlen(f[1]) >= 6 && strlen(f[9]) == 6) {
        bool digits_ok = true;
        for (int i = 0; i < 6; i++) {
            if (f[1][i] < '0' || f[1][i] > '9' || f[9][i] < '0' || f[9][i] > '9') {
                digits_ok = false;
                break;
            }
        }
        if (digits_ok) {
            int hh = (f[1][0] - '0') * 10 + (f[1][1] - '0');
            int mm = (f[1][2] - '0') * 10 + (f[1][3] - '0');
            int ss = (f[1][4] - '0') * 10 + (f[1][5] - '0');

            int dd = (f[9][0] - '0') * 10 + (f[9][1] - '0');
            int mo = (f[9][2] - '0') * 10 + (f[9][3] - '0');
            int yy = 2000 + (f[9][4] - '0') * 10 + (f[9][5] - '0');

            if (yy >= 2025 && mo >= 1 && mo <= 12 && dd >= 1 && dd <= 31 &&
                hh >= 0 && hh <= 23 && mm >= 0 && mm <= 59 && ss >= 0 && ss <= 59) {
                time_t rmc_epoch = utc_ymdhms_to_epoch(yy, mo, dd, hh, mm, ss);
                uint32_t cur_ms = millis();
                uint32_t sync_ms = gps_time_sync.last_sync_millis;
                uint32_t elapsed_ms = cur_ms - sync_ms;
                time_t target_sec = rmc_epoch + (time_t)(elapsed_ms / 1000U);
                suseconds_t target_usec = (suseconds_t)((elapsed_ms % 1000U) * 1000U);

                struct timeval cur_tv;
                gettimeofday(&cur_tv, NULL);
                int64_t diff_us = ((int64_t)target_sec - (int64_t)cur_tv.tv_sec) * 1000000LL +
                                  ((int64_t)target_usec - (int64_t)cur_tv.tv_usec);

                if (llabs(diff_us) >= 5000LL || s_last_sysclock_sync_ms == 0U || (cur_ms - s_last_sysclock_sync_ms >= 60000U)) {
                    struct timeval tv = { .tv_sec = target_sec, .tv_usec = target_usec };
                    settimeofday(&tv, NULL);
                    s_last_sysclock_sync_ms = (cur_ms == 0U) ? 1U : cur_ms;
                }

                if (!rtc_synced_from_gnss || llabs(diff_us) >= 2000000LL || (cur_ms - s_last_rtc_chip_sync_ms >= 900000U)) {
                    s_pending_rtc_epoch   = rmc_epoch;
                    s_pending_rtc_sync_ms = sync_ms;
                    __sync_synchronize();
                    rtc_synced_from_gnss    = true;
                    s_last_rtc_chip_sync_ms = (cur_ms == 0U) ? 1U : cur_ms;
                    rtc_sync_pending        = true;
                }
            }
        }
    }
}

static void update_gsv_max_band(uint8_t *slot, uint32_t *last_ms, uint8_t sv_in_view, uint32_t now)
{
    if (sv_in_view >= *slot || (now - *last_ms > 1200)) {
        *slot = sv_in_view;
        *last_ms = now;
    }
}

static void handle_gsv_sentence(char *line)
{
    if (strlen(line) < 7) return;
    char talker1 = line[1];
    char talker2 = line[2];
    char *f[6] = {0};
    int n = split_csv_fields(line, f, 6);
    if (n < 4 || !f[3] || !f[3][0]) return;

    uint32_t now = millis();
    uint8_t sv_in_view = (uint8_t)atoi(f[3]);
    if (talker1 == 'G' && talker2 == 'P')      update_gsv_max_band(&gsv_sats_gp, &gsv_last_gp_ms, sv_in_view, now);
    else if (talker1 == 'G' && talker2 == 'L') update_gsv_max_band(&gsv_sats_gl, &gsv_last_gl_ms, sv_in_view, now);
    else if (talker1 == 'G' && talker2 == 'A') update_gsv_max_band(&gsv_sats_ga, &gsv_last_ga_ms, sv_in_view, now);
    else if ((talker1 == 'G' && talker2 == 'B') || (talker1 == 'B' && talker2 == 'D')) update_gsv_max_band(&gsv_sats_gb, &gsv_last_gb_ms, sv_in_view, now);
    else if (talker1 == 'G' && talker2 == 'Q') update_gsv_max_band(&gsv_sats_gq, &gsv_last_gq_ms, sv_in_view, now);

    uint16_t total_view = (uint16_t)gsv_sats_gp + gsv_sats_gl + gsv_sats_ga + gsv_sats_gb + gsv_sats_gq;
    if (total_view > 255) total_view = 255;
    gnss_state.sats_in_view = (uint8_t)total_view;
}

static void handle_gst_sentence(char *line)
{
    char *f[10] = {0};
    int n = split_csv_fields(line, f, 10);
    if (n >= 9 && f[6] && f[6][0] && f[7] && f[7][0]) {
        float lat_e = (float)atof(f[6]);
        float lon_e = (float)atof(f[7]);
        float alt_e = (float)atof(f[8]);
        float h_err = sqrtf(lat_e * lat_e + lon_e * lon_e);
        if (h_err > 0.001f) {
            gnss_state.h_acc_m = h_err;
            gnss_state.h_acc_from_gst = true;
            uint32_t gst_ms = millis();
            s_last_gst_ms = (gst_ms == 0U) ? 1U : gst_ms;
        }
        if (alt_e > 0.001f) gnss_state.v_acc_m = alt_e;
    }
}

static void handle_hdt_sentence(char *line)
{
    char *f[4] = {0};
    int n = split_csv_fields(line, f, 4);
    if (n >= 2 && f[1] && f[1][0]) {
        gnss_state.true_heading_deg = (float)atof(f[1]);
        gnss_state.heading_valid    = true;
        port_stats.heading_sentences_ok++;
    }
}

static void handle_ths_sentence(char *line)
{
    char *f[4] = {0};
    int n = split_csv_fields(line, f, 4);
    if (n >= 3 && f[1] && f[1][0] && f[2]) {
        // Mode indicator: 'A'=Autonomous, 'D'=Differential, 'E'=Estimated, 'M'=Manual
        if (f[2][0] == 'A' || f[2][0] == 'D' || f[2][0] == 'E' || f[2][0] == 'M') {
            gnss_state.true_heading_deg = (float)atof(f[1]);
            gnss_state.heading_valid    = true;
            port_stats.heading_sentences_ok++;
        }
    }
}

static void handle_uniheadinga_sentence(char *line)
{
    char *semi = strchr(line, ';');
    if (!semi) return;
    char *f[12] = {0};
    int n = split_csv_fields(semi + 1, f, 12);
    if (n >= 5 && f[0] && f[1] && f[2] && f[3] && f[4]) {
        float bl_m = (float)atof(f[2]);
        gnss_state.baseline_length_m = bl_m;

        if (strstr(f[0], "SOL_COMPUTED") != NULL && strstr(f[1], "NONE") == NULL && strstr(f[1], "INSUFFICIENT") == NULL) {
            float expected_bl = (current_settings.antenna_spacing_m > 0.05f) ? current_settings.antenna_spacing_m : 0.555f;
            // Physical sanity gate: Dual-antenna baseline must be within +/- 15cm of expected beam spacing
            bool bl_physically_sane = (fabsf(bl_m - expected_bl) <= 0.150f);
            bool is_int = (strstr(f[1], "INT") != NULL);

            if (is_int && bl_physically_sane) {
                gnss_state.true_heading_deg   = (float)atof(f[3]);
                gnss_state.baseline_pitch_deg = (float)atof(f[4]);
                gnss_state.heading_valid      = true;
                gnss_state.heading_is_fixed   = true;
                gnss_state.heading_source     = HDG_SRC_DUAL_FIX;
                gnss_state.slave_fix_type     = GNSS_FIX_RTK_FIXED;
                gps_time_sync.last_uniheadinga_ms = millis();
                port_stats.heading_sentences_ok++;
            } else if (bl_physically_sane) {
                // Float with sane physical baseline
                gnss_state.true_heading_deg   = (float)atof(f[3]);
                gnss_state.baseline_pitch_deg = (float)atof(f[4]);
                gnss_state.heading_valid      = true;
                gnss_state.heading_is_fixed   = false;
                gnss_state.heading_source     = HDG_SRC_DUAL_FLT;
                gnss_state.slave_fix_type     = GNSS_FIX_RTK_FLOAT;
                gps_time_sync.last_uniheadinga_ms = millis();
                port_stats.heading_sentences_ok++;
            } else {
                // Wild baseline vector (e.g. 4.88m in Float) - reject dual-antenna heading!
                gnss_state.heading_is_fixed = false;
                gnss_state.slave_fix_type   = GNSS_FIX_NONE;
                if (gnss_state.heading_source == HDG_SRC_DUAL_FIX || gnss_state.heading_source == HDG_SRC_DUAL_FLT) {
                    gnss_state.heading_valid  = false;
                    gnss_state.heading_source = HDG_SRC_NONE;
                }
            }
        } else {
            gnss_state.heading_is_fixed = false;
            gnss_state.slave_fix_type   = GNSS_FIX_NONE;
            if (gnss_state.heading_source == HDG_SRC_DUAL_FIX || gnss_state.heading_source == HDG_SRC_DUAL_FLT) {
                gnss_state.heading_valid  = false;
                gnss_state.heading_source = HDG_SRC_NONE;
            }
        }
    }
}

static void ecef_to_wgs84(double x, double y, double z, double *lat_deg, double *lon_deg, double *alt_m)
{
    const double a = 6378137.0;
    const double f = 1.0 / 298.257223563;
    const double b = a * (1.0 - f);
    const double e2 = (a * a - b * b) / (a * a);
    const double ep2 = (a * a - b * b) / (b * b);

    double p = sqrt(x * x + y * y);
    if (p < 1e-6) {
        *lat_deg = (z > 0) ? 90.0 : -90.0;
        *lon_deg = 0.0;
        *alt_m = fabs(z) - b;
        return;
    }

    double th = atan2(a * z, b * p);
    double sin_th = sin(th);
    double cos_th = cos(th);

    double phi = atan2(z + ep2 * b * sin_th * sin_th * sin_th,
                       p - e2 * a * cos_th * cos_th * cos_th);
    double lam = atan2(y, x);

    double sin_phi = sin(phi);
    double N = a / sqrt(1.0 - e2 * sin_phi * sin_phi);

    *lat_deg = phi * (180.0 / M_PI);
    *lon_deg = lam * (180.0 / M_PI);
    *alt_m   = (p / cos(phi)) - N;
}

static void handle_baseinfoa_sentence(char *line)
{
    char *semi = strchr(line, ';');
    if (!semi) return;

    char *f[8] = {0};
    int n = split_csv_fields(semi + 1, f, 8);
    if (n < 4) return;

    double x = (n > 1 && f[1]) ? strtod(f[1], NULL) : 0.0;
    double y = (n > 2 && f[2]) ? strtod(f[2], NULL) : 0.0;
    double z = (n > 3 && f[3]) ? strtod(f[3], NULL) : 0.0;

    if (fabs(x) > 1000.0 && fabs(y) > 1000.0) {
        gnss_state.base_ecef_x = x;
        gnss_state.base_ecef_y = y;
        gnss_state.base_ecef_z = z;

        ecef_to_wgs84(x, y, z, &gnss_state.base_lat, &gnss_state.base_lon, &gnss_state.base_alt_m);
        gnss_state.base_coords_valid = true;

        if (n > 4 && f[4] && f[4][0]) {
            const char *sid_str = f[4];
            if (sid_str[0] == '"') sid_str++;
            gnss_state.base_station_id = (uint16_t)atoi(sid_str);
        }

        Serial.printf("[UM982 BASE] ID:%u Valid:1 Pos:%.8f, %.8f, %.3fm (ECEF: %.2f, %.2f, %.2f)\n",
                      gnss_state.base_station_id,
                      gnss_state.base_lat, gnss_state.base_lon, gnss_state.base_alt_m,
                      x, y, z);
    }
}

extern "C" bool um982_verify_nmea_checksum(const char *line)
{
    if (!line || line[0] != '$') return false;
    const char *star = strrchr(line, '*');
    if (!star || !star[1] || !star[2]) return true; // Allow if unchecksummed
    if (star[3] != '\0') return false;
    int hi = parse_hex_nibble(star[1]);
    int lo = parse_hex_nibble(star[2]);
    if (hi < 0 || lo < 0) return false;
    uint8_t calc = 0;
    for (const char *p = line + 1; p < star; p++) {
        calc ^= (uint8_t)(*p);
    }
    uint8_t expected = (uint8_t)((hi << 4) | lo);
    return (calc == expected);
}

extern "C" bool um982_verify_unicore_crc32(const char *line)
{
    if (!line || line[0] != '#') return false;
    const char *star = strrchr(line, '*');
    if (!star || star <= (line + 1)) return false;

    uint32_t expected = 0UL;
    for (int i = 1; i <= 8; i++) {
        int nib = parse_hex_nibble(star[i]);
        if (nib < 0) return false;
        expected = (expected << 4) | (uint32_t)nib;
    }
    if (star[9] != '\0') return false;

    // Bitwise NovAtel/Unicore OEM ASCII CRC32 (init=0, reflected poly=0xEDB88320, xorout=0 -> 0 B DRAM0)
    uint32_t crc = 0UL;
    for (const char *p = line + 1; p < star; p++) {
        crc ^= (uint32_t)(uint8_t)(*p);
        for (int bit = 0; bit < 8; bit++) {
            if (crc & 1UL) {
                crc = (crc >> 1) ^ 0xEDB88320UL;
            } else {
                crc >>= 1;
            }
        }
    }
    return (crc == expected);
}

static void record_unicore_crc32_drop(const char *line)
{
    port_stats.unicore_crc_errors++;
    uint32_t now = millis();
    if (current_settings.enable_serial_logging &&
        (s_last_crc_warn_ms == 0U || (now - s_last_crc_warn_ms) >= 2000U)) {
        s_last_crc_warn_ms = (now == 0U) ? 1U : now;
        Serial.printf("[UM982 CRC32 DROP] Invalid Unicore CRC32 (errs=%lu): %.48s\n",
                      (unsigned long)port_stats.unicore_crc_errors,
                      line ? line : "");
    }
}

void um982_strip_checksum_in_place(char *str)
{
    if (!str) return;
    char *star = strrchr(str, '*');
    if (star) *star = '\0';
}

void um982_mark_com1_rx_alive(void)
{
    port_stats.last_com1_rx_ms  = millis();
    port_stats.com1_online      = true;
    port_stats.com1_test_passed = true;
}

void um982_process_com1_line(char *line, uint16_t line_len)
{
    if (!line || line[0] == '\0') return;

    // =====================================================================
    // Stream 3: Unicore Command / Query Responses ($command, #VERSIONA, #MODE, $CONFIG, #LOGLIST)
    // Handled first so command responses NEVER pollute .nmea logs or overwrite last_com1_line!
    // =====================================================================
    if (strncmp(line, "#VERSIONA", 9) == 0) {
        if (!um982_verify_unicore_crc32(line)) {
            record_unicore_crc32_drop(line);
            return;
        }
        um982_record_cmd_response_line(line);
        char *semi = strchr(line, ';');
        if (semi) {
            strncpy(port_stats.um982_version, semi + 1, sizeof(port_stats.um982_version) - 1);
            port_stats.um982_version[sizeof(port_stats.um982_version) - 1] = '\0';
            um982_strip_checksum_in_place(port_stats.um982_version);
        } else {
            strncpy(port_stats.um982_version, "UM982_ONLINE", sizeof(port_stats.um982_version) - 1);
        }
        version_ack_received = true;
        Serial.printf("[UM982 CMD RX] %s\n", line);
        return;
    }

    if (strncmp(line, "#MODE", 5) == 0 || strncmp(line, "$MODE", 5) == 0) {
        um982_record_cmd_response_line(line);
        const char *semi = strchr(line, ';');
        const char *m_str = semi ? (semi + 1) : (strchr(line, ',') ? strchr(line, ',') + 1 : line);
        if (strncmp(m_str, "MODE ", 5) == 0) m_str += 5;
        strncpy(port_stats.um982_mode, m_str, sizeof(port_stats.um982_mode) - 1);
        port_stats.um982_mode[sizeof(port_stats.um982_mode) - 1] = '\0';
        um982_strip_checksum_in_place(port_stats.um982_mode);
        um982_rebuild_known_state_summary();
        mode_query_ack_received = true;
        Serial.printf("[UM982 CMD RX] %s\n", line);
        return;
    }

    if (strncmp(line, "$CONFIG,", 8) == 0 || strncmp(line, "#CONFIG,", 8) == 0) {
        um982_record_cmd_response_line(line);
        um982_parse_config_response_line(line);
        config_query_ack_received = true;
        s_last_config_line_ms = millis();
        Serial.printf("[UM982 CMD RX] %s\n", line);
        return;
    }

    if (strncmp(line, "#LOGLIST", 8) == 0 || strncmp(line, "$LOGLIST", 8) == 0) {
        um982_record_cmd_response_line(line);
        um982_parse_loglist_response_line(line);
        loglist_query_ack_received = true;
        Serial.printf("[UM982 CMD RX] %s\n", line);
        return;
    }

    if (strncmp(line, "$command,", 9) == 0) {
        um982_record_cmd_response_line(line);
        if (strstr(line, "SAVECONFIG") != NULL && strstr(line, "response:") != NULL) {
            saveconfig_ack_received = true;
        }
        Serial.printf("[UM982 CMD RX] %s\n", line);
        return;
    }

    // =====================================================================
    // Stream 1 & 2: Periodic NMEA 0183 & Unicore ASCII Navigation Logs
    // Only valid navigation sentences reach nmea_logger_feed() and last_com1_line.
    // =====================================================================
    if (line[0] == '$' && !um982_verify_nmea_checksum(line)) {
        return;
    }
    if ((strncmp(line, "#UNIHEADINGA", 12) == 0 || strncmp(line, "#BASEINFOA", 10) == 0) &&
        !um982_verify_unicore_crc32(line)) {
        record_unicore_crc32_drop(line);
        return;
    }
    if (line[0] == '$' || line[0] == '#') {
        um982_mark_com1_rx_alive();
    }

    nmea_logger_feed((const uint8_t *)line, line_len);
    nmea_logger_feed((const uint8_t *)"\r\n", 2);

    strncpy(port_stats.last_com1_line, line, sizeof(port_stats.last_com1_line) - 1);
    port_stats.last_com1_line[sizeof(port_stats.last_com1_line) - 1] = '\0';

    if (strncmp(line, "$GNGGAH", 7) == 0 || strncmp(line, "$GPGGAH", 7) == 0) {
        port_stats.nmea_sentences_ok++;
        handle_ggah_sentence(line);
    } else if (strncmp(line, "$GNGGA", 6) == 0 || strncmp(line, "$GPGGA", 6) == 0) {
        port_stats.nmea_sentences_ok++;
        handle_gga_sentence(line);
    } else if (strncmp(line, "$GNRMC", 6) == 0 || strncmp(line, "$GPRMC", 6) == 0) {
        port_stats.nmea_sentences_ok++;
        handle_rmc_sentence(line);
    } else if (strncmp(line, "$GNGST", 6) == 0 || strncmp(line, "$GPGST", 6) == 0) {
        port_stats.nmea_sentences_ok++;
        handle_gst_sentence(line);
    } else if (strncmp(line, "$GNHDT", 6) == 0 || strncmp(line, "$GPHDT", 6) == 0) {
        port_stats.nmea_sentences_ok++;
        handle_hdt_sentence(line);
    } else if (strncmp(line, "$GNTHS", 6) == 0 || strncmp(line, "$GPTHS", 6) == 0) {
        port_stats.nmea_sentences_ok++;
        handle_ths_sentence(line);
    } else if (strncmp(line, "$GNGSV", 6) == 0 || strncmp(line, "$GPGSV", 6) == 0) {
        port_stats.nmea_sentences_ok++;
        handle_gsv_sentence(line);
    } else if (strncmp(line, "#UNIHEADINGA", 12) == 0) {
        handle_uniheadinga_sentence(line);
    } else if (strncmp(line, "#BASEINFOA", 10) == 0) {
        handle_baseinfoa_sentence(line);
    }
}

extern "C" void um982_poll_com1(void)
{
    uint32_t entry_ms = millis();
    int entry_avail = Serial1.available();
    bool has_backlog = (s_last_com1_poll_ms > 0U && (entry_ms - s_last_com1_poll_ms) > 60U) ||
                       (entry_avail > 256);
    int backlog_bytes_remaining = has_backlog ? (entry_avail + (int)com1_line_pos + 64) : 0;

    while (Serial1.available() > 0) {
        char c = (char)Serial1.read();
        port_stats.com1_rx_bytes++;
        if (backlog_bytes_remaining > 0) {
            backlog_bytes_remaining--;
        }

        if (c == '\r' || c == '\n') {
            if (com1_line_overflow) {
                com1_line_overflow = false;
                com1_line_pos = 0;
            } else if (com1_line_pos > 0) {
                com1_line_buf[com1_line_pos] = '\0';
                // Suppress GNSS wall-clock discipline while draining stale NMEA queued across blocking Core 0 work
                s_com1_rx_backlog_active = (backlog_bytes_remaining > 0);
                um982_process_com1_line(com1_line_buf, com1_line_pos);
                com1_line_pos = 0;
            }
        } else if (!com1_line_overflow) {
            if (com1_line_pos < sizeof(com1_line_buf) - 1) {
                com1_line_buf[com1_line_pos++] = c;
            } else {
                // Line exceeded 511 bytes: discard remainder until next \r/\n so tail never masquerades as a new line
                com1_line_overflow = true;
                com1_line_pos = 0;
            }
        }
    }
    s_com1_rx_backlog_active = false;

    uint32_t now_ms = millis();
    bool is_online = (port_stats.last_com1_rx_ms > 0) &&
                     ((now_ms - port_stats.last_com1_rx_ms) <= 3000U);
    if (is_online && !s_was_com1_online) {
        s_was_com1_online = true;
        if (s_com1_tx_boot_complete && !port_stats.known_state_ok) {
            Serial.println("[UM982] COM1 link came ONLINE after boot -> Starting non-blocking receiver state check/config.");
            um982_start_audit_or_config_pipeline(true);
        }
    } else if (!is_online) {
        if (s_was_com1_online) {
            Serial.println("[UM982] WARNING: COM1 link went OFFLINE (>3.0s silence).");
            s_was_com1_online = false;
        }
        port_stats.com1_online      = false;
        port_stats.com1_test_passed = false;
        if (s_com1_tx_boot_complete && (now_ms - s_last_offline_probe_ms >= 3000U)) {
            s_last_offline_probe_ms = now_ms;
            // Lightweight non-blocking probe so an unconfigured/reset UM982 responds when powered on
            if (!s_com1_tx_mutex || xSemaphoreTake(s_com1_tx_mutex, pdMS_TO_TICKS(20)) == pdTRUE) {
                Serial1.write((const uint8_t *)"VERSIONA\r\n", 10);
                port_stats.com1_tx_bytes += 10U;
                if (s_com1_tx_mutex) xSemaphoreGive(s_com1_tx_mutex);
            }
        }
    }

    um982_audit_and_config_step(now_ms);
    s_last_com1_poll_ms = millis();
}

extern "C" bool um982_poll_pending_rtc_sync(uint16_t *yy, uint8_t *mo, uint8_t *dd, uint8_t *hh, uint8_t *mm, uint8_t *ss)
{
    if (!rtc_sync_pending) return false;
    time_t latched_epoch = s_pending_rtc_epoch;
    uint32_t latched_ms  = s_pending_rtc_sync_ms;
    rtc_sync_pending     = false;

    uint32_t elapsed_s = (millis() - latched_ms + 500U) / 1000U;
    time_t live_epoch  = latched_epoch + (time_t)elapsed_s;
    struct tm tm_utc;
    if (!gmtime_r(&live_epoch, &tm_utc)) return false;

    if (yy) *yy = (uint16_t)(tm_utc.tm_year + 1900);
    if (mo) *mo = (uint8_t)(tm_utc.tm_mon + 1);
    if (dd) *dd = (uint8_t)tm_utc.tm_mday;
    if (hh) *hh = (uint8_t)tm_utc.tm_hour;
    if (mm) *mm = (uint8_t)tm_utc.tm_min;
    if (ss) *ss = (uint8_t)tm_utc.tm_sec;
    return true;
}
