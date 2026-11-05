#include "um982_driver.h"
#include "../../user_config.h"
#include "../settings_manager/settings_manager.h"
#include "../storage/nmea_logger.h"
#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

static RoverUIData_t   gnss_state = {};
static Um982PortStats_t port_stats = {};
static char            com1_line_buf[256];
static uint16_t        com1_line_pos = 0;
static char            com2_line_buf[256];
static uint16_t        com2_line_pos = 0;
static bool            rtc_synced_from_gnss = false;
static volatile bool   rtc_sync_pending     = false;
static uint16_t        pending_rtc_yy       = 2026;
static uint8_t         pending_rtc_mo       = 1;
static uint8_t         pending_rtc_dd       = 1;
static uint8_t         pending_rtc_hh       = 0;
static uint8_t         pending_rtc_mm       = 0;
static uint8_t         pending_rtc_ss       = 0;
static volatile bool   saveconfig_ack_received = false;
static volatile bool   mode_query_ack_received = false;

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

static void handle_gga_sentence(char *line)
{
    char *f[16] = {0};
    int n = split_csv_fields(line, f, 16);
    if (n < 14) return;

    int qual = f[6] ? atoi(f[6]) : 0;
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

    if (gnss_state.h_acc_m <= 0.0001f || qual != GNSS_FIX_RTK_FIXED) {
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
    gnss_state.slave_fix_type = (GnssFixType_t)qual;
    gnss_state.slave_num_sats = f[7] ? (uint8_t)atoi(f[7]) : 0;
    if (f[8] && f[8][0]) gnss_state.slave_hdop  = (float)atof(f[8]);
    if (f[9] && f[9][0]) gnss_state.slave_alt_m = (float)atof(f[9]);

    if (f[2] && f[2][0] && f[4] && f[4][0]) {
        gnss_state.slave_lat = parse_nmea_deg_min(f[2], f[3]);
        gnss_state.slave_lon = parse_nmea_deg_min(f[4], f[5]);
    }
}

static void handle_rmc_sentence(char *line)
{
    char *f[14] = {0};
    int n = split_csv_fields(line, f, 14);
    if (n < 10) return;

    // Field 2: Status ('A'=Active, 'V'=Void, or 'D'=Differential)
    bool pos_active = (f[2] && (f[2][0] == 'A' || f[2][0] == 'D'));

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

    if (!rtc_synced_from_gnss && pos_active && strlen(f[1]) >= 6 && strlen(f[9]) == 6) {
        int hh = (f[1][0] - '0') * 10 + (f[1][1] - '0');
        int mm = (f[1][2] - '0') * 10 + (f[1][3] - '0');
        int ss = (f[1][4] - '0') * 10 + (f[1][5] - '0');

        int dd = (f[9][0] - '0') * 10 + (f[9][1] - '0');
        int mo = (f[9][2] - '0') * 10 + (f[9][3] - '0');
        int yy = 2000 + (f[9][4] - '0') * 10 + (f[9][5] - '0');

        if (yy >= 2025 && mo >= 1 && mo <= 12 && dd >= 1 && dd <= 31) {
            pending_rtc_yy       = (uint16_t)yy;
            pending_rtc_mo       = (uint8_t)mo;
            pending_rtc_dd       = (uint8_t)dd;
            pending_rtc_hh       = (uint8_t)hh;
            pending_rtc_mm       = (uint8_t)mm;
            pending_rtc_ss       = (uint8_t)ss;
            rtc_synced_from_gnss = true;
            rtc_sync_pending     = true;
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
        if (h_err > 0.001f) gnss_state.h_acc_m = h_err;
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
                port_stats.heading_sentences_ok++;
            } else if (bl_physically_sane) {
                // Float with sane physical baseline
                gnss_state.true_heading_deg   = (float)atof(f[3]);
                gnss_state.baseline_pitch_deg = (float)atof(f[4]);
                gnss_state.heading_valid      = true;
                gnss_state.heading_is_fixed   = false;
                gnss_state.heading_source     = HDG_SRC_DUAL_FLT;
                port_stats.heading_sentences_ok++;
            } else {
                // Wild baseline vector (e.g. 4.88m in Float) - reject dual-antenna heading!
                gnss_state.heading_is_fixed = false;
                if (gnss_state.heading_source == HDG_SRC_DUAL_FIX || gnss_state.heading_source == HDG_SRC_DUAL_FLT) {
                    gnss_state.heading_valid  = false;
                    gnss_state.heading_source = HDG_SRC_NONE;
                }
            }
        } else {
            gnss_state.heading_is_fixed = false;
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

static void process_com1_line(char *line)
{
    if (!line || line[0] == '\0') return;
    port_stats.com1_test_passed = true;
    
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
    } else if (strncmp(line, "#VERSIONA", 9) == 0) {
        char *semi = strchr(line, ';');
        if (semi) {
            strncpy(port_stats.um982_version, semi + 1, sizeof(port_stats.um982_version) - 1);
            port_stats.um982_version[sizeof(port_stats.um982_version) - 1] = '\0';
        } else {
            strncpy(port_stats.um982_version, "UM982_ONLINE", sizeof(port_stats.um982_version) - 1);
        }
        Serial.printf("[UM982 COM1 RX] %s\n", line);
    } else if (strncmp(line, "$command,", 9) == 0 ||
               strncmp(line, "$CONFIG,", 8) == 0 ||
               strncmp(line, "#CONFIG", 7) == 0 ||
               strncmp(line, "#MODE", 5) == 0 ||
               strncmp(line, "$MODE", 5) == 0 ||
               strstr(line, "MODE") != NULL ||
               strstr(line, "CONFIG") != NULL ||
               strstr(line, "response:") != NULL) {
        if (strstr(line, "SAVECONFIG") != NULL && strstr(line, "response:") != NULL) {
            saveconfig_ack_received = true;
        }
        if (strstr(line, "MODE") != NULL) {
            mode_query_ack_received = true;
        }
        Serial.printf("[UM982 COM1 RX] %s\n", line);
    }
}

static void process_com2_line(char *line)
{
    if (!line || line[0] == '\0') return;
    port_stats.com2_test_passed = true;
    strncpy(port_stats.last_com2_line, line, sizeof(port_stats.last_com2_line) - 1);
    port_stats.last_com2_line[sizeof(port_stats.last_com2_line) - 1] = '\0';
    if (Serial) Serial.printf("[UM982 COM2 RX] %s\n", line);
}

extern "C" void send_um982_com1_command(const char *cmd)
{
    if (!cmd) return;
    size_t len = strlen(cmd);
    if (Serial) Serial.printf("[UM982 COM1 TX] %s\n", cmd);
    Serial1.write((const uint8_t *)cmd, len);
    Serial1.write((const uint8_t *)"\r\n", 2);
    port_stats.com1_tx_bytes += (uint32_t)(len + 2);
}

extern "C" void send_um982_com2_test_pulse(void)
{
    const char *cmd = "VERSIONA COM2\r\n";
    size_t len = strlen(cmd);
    Serial.println("[UM982 COM2 TX] VERSIONA COM2 (Testing COM2 IO47/48 @ 115200)");
    Serial2.write((const uint8_t *)cmd, len);
    port_stats.com2_tx_bytes += (uint32_t)len;
}

extern "C" void um982_configure_rover_defaults(void)
{
    send_um982_com1_command("CONFIG COM2 115200");
    vTaskDelay(pdMS_TO_TICKS(40));
	
    send_um982_com1_command("UNLOGALL COM1");
    vTaskDelay(pdMS_TO_TICKS(40));

    send_um982_com1_command("CONFIG PVTALG MULTI");
    vTaskDelay(pdMS_TO_TICKS(40));

    // Put UM982 into precision surveying mode (overrides factory default UAV drone mode)
    send_um982_com1_command("MODE ROVER SURVEY");
    vTaskDelay(pdMS_TO_TICKS(40));

    send_um982_com1_command("CONFIG HEADING LOWDYNAMIC");
    vTaskDelay(pdMS_TO_TICKS(40));

    send_um982_com1_command("CONFIG HEADING FIXLENGTH");
    vTaskDelay(pdMS_TO_TICKS(40));

    char hdg_cmd[64];
    float spacing_m = (current_settings.antenna_spacing_m > 0.05f) ? current_settings.antenna_spacing_m : 0.555f;
    float spacing_cm = spacing_m * 100.0f;
    // Parameter 1: baseline length in centimeters (Table 4-14 in manual: 55.5 cm)
    // Parameter 2: tolerance in centimeters (12 cm tolerance)
    snprintf(hdg_cmd, sizeof(hdg_cmd), "CONFIG HEADING LENGTH %.1f 12", spacing_cm);
    send_um982_com1_command(hdg_cmd);
    vTaskDelay(pdMS_TO_TICKS(40));
  
    send_um982_com1_command("GPGGA COM1 0.5");
    vTaskDelay(pdMS_TO_TICKS(40));
    send_um982_com1_command("GPRMC COM1 1");
    vTaskDelay(pdMS_TO_TICKS(40));
    send_um982_com1_command("GPGST COM1 0.5");
    vTaskDelay(pdMS_TO_TICKS(40));
    send_um982_com1_command("GPGSV COM1 1");
    vTaskDelay(pdMS_TO_TICKS(40));
    // UNIHEADINGA provides heading, pitch, baseline length, and NARROW_INT fix type at 2 Hz
    send_um982_com1_command("UNIHEADINGA COM1 0.5");
    vTaskDelay(pdMS_TO_TICKS(40));
    // GNGGAH provides Slave Antenna (ANT2) fix quality, satellites used, HDOP at 1 Hz
    send_um982_com1_command("GNGGAH COM1 1");
    vTaskDelay(pdMS_TO_TICKS(40));
    // BASEINFOA provides base station ID, health status, and ECEF coordinates
    // Log at 1 Hz so Base Station coordinates are continuously streamed whenever RTCM corrections arrive
    send_um982_com1_command("BASEINFOA COM1 1");
    vTaskDelay(pdMS_TO_TICKS(40));
    saveconfig_ack_received = false;
    send_um982_com1_command("SAVECONFIG");
    // Wait for SAVECONFIG flash writing to complete before querying
    uint32_t t_save = millis();
    while (!saveconfig_ack_received && (millis() - t_save < 1500)) {
        um982_poll_com1();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    // Query active mode to confirm in telemetry
    mode_query_ack_received = false;
    send_um982_com1_command("MODE");
    uint32_t t_mode = millis();
    while (!mode_query_ack_received && (millis() - t_mode < 800)) {
        um982_poll_com1();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    // Query full receiver configuration dump
    send_um982_com1_command("CONFIG");
    uint32_t t_cfg = millis();
    while (millis() - t_cfg < 1200) {
        um982_poll_com1();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

extern "C" void um982_run_powerup_selftest(void)
{
    Serial.println("\n--- [UM982 Dual-Antenna GNSS Power-Up Self-Test] ---");

    Serial.println("[UM982 COM1] Testing UART1 (RX=IO17, TX=IO18 @ 115200) with VERSIONA...");
    send_um982_com1_command("VERSIONA");
    uint32_t t0 = millis();
    while (millis() - t0 < 600) {
        um982_poll_com1();
        if (strncmp(port_stats.um982_version, "Waiting", 7) != 0) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (port_stats.com1_test_passed) {
        Serial.printf("[UM982 COM1 TEST] PASS — RX=%lu B | Version: %s | Last: %s\n",
                      (unsigned long)port_stats.com1_rx_bytes,
                      port_stats.um982_version,
                      port_stats.last_com1_line);
    } else {
        Serial.println("[UM982 COM1 TEST] FAIL — No response on IO17/18 @ 115200");
    }

    Serial.println("[UM982 COM2] Testing UART2 (RX=IO47, TX=IO48 @ 115200) with VERSIONA COM2...");
    while (Serial2.available()) Serial2.read();
    send_um982_com2_test_pulse();
    t0 = millis();
    while (millis() - t0 < 500) {
        um982_poll_com2();
        if (port_stats.com2_test_passed && strstr(port_stats.last_com2_line, "VERSION") != NULL) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (port_stats.com2_test_passed) {
        Serial.printf("[UM982 COM2 TEST] PASS — Bidirectional COM2 verified (RX=%lu B, TX=%lu B) | Last: %s\n",
                      (unsigned long)port_stats.com2_rx_bytes,
                      (unsigned long)port_stats.com2_tx_bytes,
                      port_stats.last_com2_line);
    } else {
        Serial.println("[UM982 COM2] No reply to IO48 TX -> Requesting 'VERSIONA COM2' via COM1 to test IO47 RX...");
        send_um982_com1_command("VERSIONA COM2");
        t0 = millis();
        while (millis() - t0 < 500) {
            um982_poll_com1();
            um982_poll_com2();
            if (port_stats.com2_test_passed) break;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (port_stats.com2_test_passed) {
            Serial.printf("[UM982 COM2 TEST] PARTIAL — IO47 (UM982_TXD1 RX) OK (%lu B), but IO48 (UM982_RXD1 TX) did not trigger COM2\n",
                          (unsigned long)port_stats.com2_rx_bytes);
        } else {
            Serial.println("[UM982 COM2 TEST] WARN — No RX response on IO47 from COM2 (check IO47/IO48 wiring)");
        }
    }
}

extern "C" void um982_driver_init(void)
{
    strncpy(port_stats.um982_version, "Waiting COM1...", sizeof(port_stats.um982_version) - 1);
    port_stats.um982_version[sizeof(port_stats.um982_version) - 1] = '\0';
    strncpy(port_stats.last_com1_line, "No COM1 RX yet", sizeof(port_stats.last_com1_line) - 1);
    port_stats.last_com1_line[sizeof(port_stats.last_com1_line) - 1] = '\0';
    strncpy(port_stats.last_com2_line, "No COM2 RX yet", sizeof(port_stats.last_com2_line) - 1);
    port_stats.last_com2_line[sizeof(port_stats.last_com2_line) - 1] = '\0';

    pinMode(ROVER_PIN_UM982_TXD0, INPUT_PULLDOWN);
    pinMode(ROVER_PIN_UM982_RXD0, INPUT_PULLDOWN);
    pinMode(ROVER_PIN_UM982_TXD1, INPUT_PULLDOWN);
    pinMode(ROVER_PIN_UM982_RXD1, INPUT_PULLDOWN);
    delay(2);
    Serial.printf("[UM982 LINE PROBE] COM1: IO17(ESP_RX)=%d IO18(ESP_TX)=%d | COM2: IO47(ESP_RX)=%d IO48(ESP_TX)=%d\n",
                  digitalRead(ROVER_PIN_UM982_TXD0),
                  digitalRead(ROVER_PIN_UM982_RXD0),
                  digitalRead(ROVER_PIN_UM982_TXD1),
                  digitalRead(ROVER_PIN_UM982_RXD1));

    Serial1.setRxBufferSize(4096);
    Serial1.setTxBufferSize(1024);
    Serial1.begin(ROVER_UM982_COM1_BAUD, SERIAL_8N1, ROVER_PIN_UM982_TXD0, ROVER_PIN_UM982_RXD0);

    Serial2.setRxBufferSize(1024);
    Serial2.setTxBufferSize(4096);
    Serial2.begin(ROVER_UM982_COM2_BAUD, SERIAL_8N1, ROVER_PIN_UM982_TXD1, ROVER_PIN_UM982_RXD1);

    um982_run_powerup_selftest();
    um982_configure_rover_defaults();
}

extern "C" void um982_forward_rtcm_to_com2(const uint8_t *data, size_t len)
{
    if (!data || len == 0) return;
    size_t written = Serial2.write(data, len);
    port_stats.com2_tx_bytes += (uint32_t)written;
}

extern "C" void um982_poll_com1(void)
{
    while (Serial1.available() > 0) {
        char c = (char)Serial1.read();
        port_stats.com1_rx_bytes++;

        if (c == '\r' || c == '\n') {
            if (com1_line_pos > 0) {
                com1_line_buf[com1_line_pos] = '\0';
                nmea_logger_feed((const uint8_t*)com1_line_buf, com1_line_pos);
                nmea_logger_feed((const uint8_t*)"\r\n", 2);
                process_com1_line(com1_line_buf);
                com1_line_pos = 0;
            }
        } else if (com1_line_pos < sizeof(com1_line_buf) - 1) {
            com1_line_buf[com1_line_pos++] = c;
        } else {
            com1_line_pos = 0;
        }
    }
}

extern "C" void um982_poll_com2(void)
{
    while (Serial2.available() > 0) {
        char c = (char)Serial2.read();
        port_stats.com2_rx_bytes++;

        if (c == '\r' || c == '\n') {
            if (com2_line_pos > 0) {
                com2_line_buf[com2_line_pos] = '\0';
                process_com2_line(com2_line_buf);
                com2_line_pos = 0;
            }
        } else if (com2_line_pos < sizeof(com2_line_buf) - 1) {
            com2_line_buf[com2_line_pos++] = c;
        } else {
            com2_line_pos = 0;
        }
    }
}

extern "C" void um982_get_latest_state(RoverUIData_t *state_out)
{
    if (!state_out) return;
    state_out->raw_lat            = gnss_state.raw_lat;
    state_out->raw_lon            = gnss_state.raw_lon;
    state_out->raw_alt_m          = gnss_state.raw_alt_m;
    state_out->fix_type           = gnss_state.fix_type;
    state_out->num_sats           = gnss_state.num_sats;
    state_out->sats_in_view       = gnss_state.sats_in_view;
    state_out->hdop               = gnss_state.hdop;
    state_out->h_acc_m            = gnss_state.h_acc_m;
    state_out->v_acc_m            = gnss_state.v_acc_m;
    state_out->diff_age_s         = gnss_state.diff_age_s;
    state_out->heading_valid      = gnss_state.heading_valid;
    state_out->heading_is_fixed   = gnss_state.heading_is_fixed;
    state_out->heading_source     = gnss_state.heading_source;
    state_out->true_heading_deg   = gnss_state.true_heading_deg;
    state_out->baseline_pitch_deg = gnss_state.baseline_pitch_deg;
    state_out->baseline_length_m  = gnss_state.baseline_length_m;
    state_out->speed_m_s          = gnss_state.speed_m_s;
    state_out->cog_deg            = gnss_state.cog_deg;
    state_out->slave_fix_type     = gnss_state.slave_fix_type;
    state_out->slave_num_sats     = gnss_state.slave_num_sats;
    state_out->slave_hdop         = gnss_state.slave_hdop;
    state_out->slave_alt_m        = gnss_state.slave_alt_m;

    // Slave Antenna (ANT2) Coordinates
    if (gnss_state.slave_lat != 0.0 && gnss_state.slave_lon != 0.0) {
        state_out->slave_lat      = gnss_state.slave_lat;
        state_out->slave_lon      = gnss_state.slave_lon;
    } else if (gnss_state.heading_valid && gnss_state.baseline_length_m > 0.05f && gnss_state.raw_lat != 0.0) {
        // Geodetic projection from ANT1 to ANT2 along verified dual-antenna baseline vector
        double lat_rad = gnss_state.raw_lat * (M_PI / 180.0);
        double hdg_rad = gnss_state.true_heading_deg * (M_PI / 180.0);
        double pitch_rad = gnss_state.baseline_pitch_deg * (M_PI / 180.0);
        double horiz_len = gnss_state.baseline_length_m * cos(pitch_rad);
        double d_north = horiz_len * cos(hdg_rad);
        double d_east  = horiz_len * sin(hdg_rad);
        double m_per_deg_lat = 111132.954 - 559.822 * cos(2.0 * lat_rad) + 1.175 * cos(4.0 * lat_rad);
        double m_per_deg_lon = (111412.84 * cos(lat_rad) - 93.5 * cos(3.0 * lat_rad));
        state_out->slave_lat      = gnss_state.raw_lat + (d_north / m_per_deg_lat);
        state_out->slave_lon      = gnss_state.raw_lon + (d_east / m_per_deg_lon);
    } else {
        state_out->slave_lat      = 0.0;
        state_out->slave_lon      = 0.0;
    }

    // Base Station Telemetry
    state_out->base_station_id    = gnss_state.base_station_id;
    state_out->base_coords_valid  = gnss_state.base_coords_valid;
    state_out->base_lat           = gnss_state.base_lat;
    state_out->base_lon           = gnss_state.base_lon;
    state_out->base_alt_m         = gnss_state.base_alt_m;
    state_out->base_ecef_x        = gnss_state.base_ecef_x;
    state_out->base_ecef_y        = gnss_state.base_ecef_y;
    state_out->base_ecef_z        = gnss_state.base_ecef_z;
}

extern "C" void um982_get_port_stats(Um982PortStats_t *stats_out)
{
	if (stats_out) {
		*stats_out = port_stats;
	}
}

extern "C" bool um982_poll_pending_rtc_sync(uint16_t *yy, uint8_t *mo, uint8_t *dd, uint8_t *hh, uint8_t *mm, uint8_t *ss)
{
	if (!rtc_sync_pending) return false;
	rtc_sync_pending = false;
	if (yy) *yy = pending_rtc_yy;
	if (mo) *mo = pending_rtc_mo;
	if (dd) *dd = pending_rtc_dd;
	if (hh) *hh = pending_rtc_hh;
	if (mm) *mm = pending_rtc_mm;
	if (ss) *ss = pending_rtc_ss;
	return true;
}