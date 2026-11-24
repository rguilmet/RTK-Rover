#include "um982_driver_internal.h"

SemaphoreHandle_t s_com1_tx_mutex         = NULL;
volatile bool     s_com1_tx_boot_complete = false;

volatile bool     version_ack_received       = false;
volatile bool     saveconfig_ack_received    = false;
volatile bool     mode_query_ack_received    = false;
volatile bool     config_query_ack_received  = false;
volatile bool     loglist_query_ack_received = false;
volatile uint32_t s_last_config_line_ms      = 0;

static char    s_cfg_pvtalg[16]            = "--";
static char    s_cfg_hdg_mode[16]          = "--";
char           s_cfg_hdg_len[24]           = "--";
static char    s_cfg_hdg_dyn[16]           = "--";
static char    s_cfg_com1[16]              = "115200";
static uint8_t s_loglist_com1_cnt          = 0;
static uint8_t s_loglist_other_ports_cnt   = 0;
static uint8_t s_loglist_mask              = 0; // Bitmask: 0=GGA,1=RMC,2=GST,3=GSV,4=HDG,5=GGAH,6=BASE

// =========================================================================
// Non-Blocking Receiver Audit & Configuration State Machine (F-14)
// Allows runtime audits and post-boot reconnect reconfigurations on Core 0
// without blocking RTCM3 forwarding > 1 second.
// =========================================================================
typedef enum {
    UM982_PHASE_IDLE = 0,
    UM982_PHASE_AUDIT_VERSIONA,
    UM982_PHASE_AUDIT_MODE,
    UM982_PHASE_AUDIT_CONFIG,
    UM982_PHASE_AUDIT_LOGLIST,
    UM982_PHASE_AUDIT_DRAIN,
    UM982_PHASE_CFG_COMMANDS,
    UM982_PHASE_CFG_SAVECONFIG
} Um982AuditPhase_t;

static Um982AuditPhase_t s_audit_phase     = UM982_PHASE_IDLE;
static uint32_t          s_phase_start_ms  = 0;
static uint32_t          s_cfg_step_last_ms = 0;
static uint8_t           s_cfg_step_idx    = 0;
static bool              s_cfg_after_audit = false;

void um982_rebuild_known_state_summary(void)
{
    snprintf(port_stats.um982_config_summary, sizeof(port_stats.um982_config_summary),
             "PVT:%s | HDG:%s %s %s | C1:%s",
             s_cfg_pvtalg, s_cfg_hdg_mode, s_cfg_hdg_len, s_cfg_hdg_dyn, s_cfg_com1);

    snprintf(port_stats.um982_loglist, sizeof(port_stats.um982_loglist),
             "COM1(%u):%s%s%s%s%s%s%s | COM2:%u",
             (unsigned)s_loglist_com1_cnt,
             (s_loglist_mask & (1U << 0)) ? " GGA"  : "",
             (s_loglist_mask & (1U << 1)) ? " RMC"  : "",
             (s_loglist_mask & (1U << 2)) ? " GST"  : "",
             (s_loglist_mask & (1U << 3)) ? " GSV"  : "",
             (s_loglist_mask & (1U << 4)) ? " HDG"  : "",
             (s_loglist_mask & (1U << 5)) ? " GGAH" : "",
             (s_loglist_mask & (1U << 6)) ? " BASE" : "",
             (unsigned)s_loglist_other_ports_cnt);

    bool survey_ok = (strstr(port_stats.um982_mode, "SURVEY") != NULL);
    bool logs_ok   = (s_loglist_mask == 0x7FU && s_loglist_other_ports_cnt == 0);
    port_stats.known_state_ok = (survey_ok && logs_ok);
}

void um982_parse_config_response_line(const char *line)
{
    if (strstr(line, "CONFIG PVTALG ") != NULL) {
        const char *p = strstr(line, "CONFIG PVTALG ") + 14;
        strncpy(s_cfg_pvtalg, p, sizeof(s_cfg_pvtalg) - 1);
        s_cfg_pvtalg[sizeof(s_cfg_pvtalg) - 1] = '\0';
        um982_strip_checksum_in_place(s_cfg_pvtalg);
    } else if (strstr(line, "CONFIG HEADING FIXLENGTH") != NULL) {
        strncpy(s_cfg_hdg_mode, "FIXLEN", sizeof(s_cfg_hdg_mode) - 1);
    } else if (strstr(line, "CONFIG HEADING VARLENGTH") != NULL) {
        strncpy(s_cfg_hdg_mode, "VARLEN", sizeof(s_cfg_hdg_mode) - 1);
    } else if (strstr(line, "CONFIG HEADING LENGTH ") != NULL) {
        const char *p = strstr(line, "CONFIG HEADING LENGTH ") + 22;
        strncpy(s_cfg_hdg_len, p, sizeof(s_cfg_hdg_len) - 1);
        s_cfg_hdg_len[sizeof(s_cfg_hdg_len) - 1] = '\0';
        um982_strip_checksum_in_place(s_cfg_hdg_len);
    } else if (strstr(line, "CONFIG HEADING LOWDYNAMIC") != NULL) {
        strncpy(s_cfg_hdg_dyn, "LOWDYN", sizeof(s_cfg_hdg_dyn) - 1);
    } else if (strstr(line, "CONFIG COM1 ") != NULL) {
        const char *p = strstr(line, "CONFIG COM1 ") + 12;
        strncpy(s_cfg_com1, p, sizeof(s_cfg_com1) - 1);
        s_cfg_com1[sizeof(s_cfg_com1) - 1] = '\0';
        um982_strip_checksum_in_place(s_cfg_com1);
    }
    um982_rebuild_known_state_summary();
}

void um982_parse_loglist_response_line(const char *line)
{
    // Unicore #LOGLIST format:
    // #LOGLIST,97,GPS,FINE,...;<count>,COM1,GPGGA,ONTIME,0.5,COM1,GPRMC,ONTIME,1,...*XX
    const char *body = strchr(line, ';');
    if (!body) body = line;
    else body++;

    char buf[512];
    strncpy(buf, body, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    um982_strip_checksum_in_place(buf);

    // If this is the header of a #LOGLIST;count,... dump, reset counters before scanning tokens
    if (strncmp(line, "#LOGLIST", 8) == 0) {
        s_loglist_com1_cnt        = 0;
        s_loglist_other_ports_cnt = 0;
        s_loglist_mask            = 0;
    }

    char *tok = strtok(buf, ",");
    while (tok != NULL) {
        if (strcmp(tok, "COM1") == 0) {
            s_loglist_com1_cnt++;
            char *msg = strtok(NULL, ",");
            if (msg) {
                if (strstr(msg, "GGAH") != NULL)           s_loglist_mask |= (1U << 5);
                else if (strstr(msg, "GGA") != NULL)       s_loglist_mask |= (1U << 0);
                else if (strstr(msg, "RMC") != NULL)       s_loglist_mask |= (1U << 1);
                else if (strstr(msg, "GST") != NULL)       s_loglist_mask |= (1U << 2);
                else if (strstr(msg, "GSV") != NULL)       s_loglist_mask |= (1U << 3);
                else if (strstr(msg, "UNIHEADING") != NULL)s_loglist_mask |= (1U << 4);
                else if (strstr(msg, "BASEINFO") != NULL)  s_loglist_mask |= (1U << 6);
            }
        } else if (strcmp(tok, "COM2") == 0 || strcmp(tok, "COM3") == 0) {
            s_loglist_other_ports_cnt++;
        }
        tok = strtok(NULL, ",");
    }
    um982_rebuild_known_state_summary();
}

void um982_record_cmd_response_line(const char *line)
{
    um982_mark_com1_rx_alive();
    port_stats.cmd_responses_ok++;
    strncpy(port_stats.last_cmd_ack_line, line, sizeof(port_stats.last_cmd_ack_line) - 1);
    port_stats.last_cmd_ack_line[sizeof(port_stats.last_cmd_ack_line) - 1] = '\0';
}

extern "C" void send_um982_com1_command(const char *cmd)
{
    if (!cmd) return;
    size_t len = strlen(cmd);
    if (s_com1_tx_mutex) {
        if (xSemaphoreTake(s_com1_tx_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
            port_stats.com1_tx_mutex_timeouts++;
            return;
        }
    }
    if (Serial) Serial.printf("[UM982 COM1 TX] %s\n", cmd);
    Serial1.write((const uint8_t *)cmd, len);
    Serial1.write((const uint8_t *)"\r\n", 2);
    port_stats.com1_tx_bytes += (uint32_t)(len + 2);
    if (s_com1_tx_mutex) xSemaphoreGive(s_com1_tx_mutex);
}

static void um982_send_cfg_command_step(uint8_t step)
{
    float spacing_m  = (current_settings.antenna_spacing_m > 0.05f) ? current_settings.antenna_spacing_m : 0.555f;
    float spacing_cm = spacing_m * 100.0f;
    char hdg_cmd[64];

    switch (step) {
        // Phase 1: Quiet-First Setup (Silence periodic logs on UM982 COM1 & unused receiver COM2)
        case 0: send_um982_com1_command("CONFIG COM1 115200"); break;
        case 1: send_um982_com1_command("UNLOGALL COM1"); break;
        case 2: send_um982_com1_command("UNLOGALL COM2"); break;
        // Phase 2: Receiver Algorithm, Mode & Fixed-Baseline Heading Configuration (while COM1 is quiet)
        case 3: send_um982_com1_command("CONFIG PVTALG MULTI"); break;
        // Put UM982 into precision surveying mode (overrides factory default UAV drone mode)
        case 4: send_um982_com1_command("MODE ROVER SURVEY"); break;
        case 5: send_um982_com1_command("CONFIG HEADING LOWDYNAMIC"); break;
        case 6: send_um982_com1_command("CONFIG HEADING FIXLENGTH"); break;
        case 7:
            // Parameter 1: baseline length in centimeters (Table 4-14 in manual: 55.5 cm)
            // Parameter 2: tolerance in centimeters (12 cm tolerance)
            snprintf(hdg_cmd, sizeof(hdg_cmd), "CONFIG HEADING LENGTH %.1f 12", spacing_cm);
            send_um982_com1_command(hdg_cmd);
            break;
        // Phase 3: Enable the 7 Required Periodic Logs strictly on COM1
        case 8:  send_um982_com1_command("GPGGA COM1 0.5"); break;
        case 9:  send_um982_com1_command("GPRMC COM1 1"); break;
        case 10: send_um982_com1_command("GPGST COM1 0.5"); break;
        case 11: send_um982_com1_command("GPGSV COM1 1"); break;
        // UNIHEADINGA provides heading, pitch, baseline length, and NARROW_INT fix type at 2 Hz
        case 12: send_um982_com1_command("UNIHEADINGA COM1 0.5"); break;
        // GNGGAH provides Slave Antenna (ANT2) fix quality, satellites used, HDOP at 1 Hz
        case 13: send_um982_com1_command("GNGGAH COM1 1"); break;
        // BASEINFOA provides base station ID, health status, and ECEF coordinates at 1 Hz
        case 14: send_um982_com1_command("BASEINFOA COM1 1"); break;
        default: break;
    }
}

bool um982_start_audit_or_config_pipeline(bool cfg_after_audit)
{
    if (s_audit_phase != UM982_PHASE_IDLE) {
        return false;
    }
    Serial.println("\n--- [UM982 Known-State Audit: VERSIONA / MODE / CONFIG / LOGLIST] ---");
    s_cfg_after_audit    = cfg_after_audit;
    version_ack_received = false;
    send_um982_com1_command("VERSIONA");
    s_audit_phase    = UM982_PHASE_AUDIT_VERSIONA;
    s_phase_start_ms = millis();
    return true;
}

void um982_audit_and_config_step(uint32_t now_ms)
{
    switch (s_audit_phase) {
        case UM982_PHASE_IDLE:
            return;

        case UM982_PHASE_AUDIT_VERSIONA:
            if (version_ack_received || (now_ms - s_phase_start_ms >= 450U)) {
                mode_query_ack_received = false;
                send_um982_com1_command("MODE");
                s_audit_phase    = UM982_PHASE_AUDIT_MODE;
                s_phase_start_ms = now_ms;
            }
            break;

        case UM982_PHASE_AUDIT_MODE:
            if (mode_query_ack_received || (now_ms - s_phase_start_ms >= 450U)) {
                config_query_ack_received = false;
                s_last_config_line_ms     = now_ms;
                send_um982_com1_command("CONFIG");
                s_audit_phase    = UM982_PHASE_AUDIT_CONFIG;
                s_phase_start_ms = now_ms;
            }
            break;

        case UM982_PHASE_AUDIT_CONFIG: {
            // CONFIG emits multiple $CONFIG,... lines; advance early once silent for 150ms
            bool quiet_done = config_query_ack_received && ((now_ms - s_last_config_line_ms) >= 150U);
            if (quiet_done || (now_ms - s_phase_start_ms >= 550U)) {
                loglist_query_ack_received = false;
                send_um982_com1_command("LOGLIST");
                s_audit_phase    = UM982_PHASE_AUDIT_LOGLIST;
                s_phase_start_ms = now_ms;
            }
            break;
        }

        case UM982_PHASE_AUDIT_LOGLIST:
            if (loglist_query_ack_received || (now_ms - s_phase_start_ms >= 500U)) {
                s_audit_phase    = UM982_PHASE_AUDIT_DRAIN;
                s_phase_start_ms = now_ms;
            }
            break;

        case UM982_PHASE_AUDIT_DRAIN:
            if (now_ms - s_phase_start_ms >= 80U) {
                port_stats.single_uart_audit_passed = port_stats.com1_test_passed;
                Serial.printf("[UM982 AUDIT SUMMARY] State:%s | Ver:%s | Mode:%s | Cfg:[%s] | Logs:[%s]\n",
                              port_stats.known_state_ok ? "KNOWN STATE OK" : "CHECK LOGS/MODE",
                              port_stats.um982_version,
                              port_stats.um982_mode,
                              port_stats.um982_config_summary,
                              port_stats.um982_loglist);

                if (s_cfg_after_audit) {
                    s_cfg_after_audit = false;
                    float spacing_m  = (current_settings.antenna_spacing_m > 0.05f) ? current_settings.antenna_spacing_m : 0.555f;
                    float spacing_cm = spacing_m * 100.0f;
                    char exp_len_prefix[16];
                    snprintf(exp_len_prefix, sizeof(exp_len_prefix), "%.1f", spacing_cm);

                    // Skip reconfig & SAVECONFIG flash wear on warm boot when receiver is already healthy
                    if (port_stats.known_state_ok &&
                        strstr(port_stats.um982_mode, "SURVEY") != NULL &&
                        strstr(s_cfg_hdg_len, exp_len_prefix) != NULL) {
                        Serial.println("[UM982] Warm boot: Receiver already in KNOWN STATE OK -> Skipping reconfig & SAVECONFIG.");
                        s_audit_phase = UM982_PHASE_IDLE;
                    } else {
                        s_cfg_step_idx     = 0;
                        s_cfg_step_last_ms = 0;
                        s_audit_phase      = UM982_PHASE_CFG_COMMANDS;
                    }
                } else {
                    s_audit_phase = UM982_PHASE_IDLE;
                }
            }
            break;

        case UM982_PHASE_CFG_COMMANDS:
            if (s_cfg_step_last_ms == 0U || (now_ms - s_cfg_step_last_ms >= 40U)) {
                if (s_cfg_step_idx < 15U) {
                    um982_send_cfg_command_step(s_cfg_step_idx);
                    s_cfg_step_idx++;
                    s_cfg_step_last_ms = now_ms;
                } else {
                    // Phase 4: Persist to UM982 Non-Volatile Flash
                    saveconfig_ack_received = false;
                    send_um982_com1_command("SAVECONFIG");
                    s_audit_phase    = UM982_PHASE_CFG_SAVECONFIG;
                    s_phase_start_ms = now_ms;
                }
            }
            break;

        case UM982_PHASE_CFG_SAVECONFIG:
            if ((saveconfig_ack_received && (now_ms - s_phase_start_ms >= 50U)) ||
                (now_ms - s_phase_start_ms >= 1500U)) {
                // Phase 5: Chain full 4-command Known-State Audit (without reconfig loop)
                s_cfg_after_audit    = false;
                version_ack_received = false;
                send_um982_com1_command("VERSIONA");
                s_audit_phase    = UM982_PHASE_AUDIT_VERSIONA;
                s_phase_start_ms = now_ms;
            }
            break;
    }
}

extern "C" bool um982_request_receiver_audit(void)
{
    return um982_start_audit_or_config_pipeline(false);
}

extern "C" bool um982_is_receiver_audit_active(void)
{
    return (s_audit_phase != UM982_PHASE_IDLE);
}

extern "C" void um982_query_receiver_state(void)
{
    if (!um982_request_receiver_audit()) {
        return;
    }
    if (!s_com1_tx_boot_complete) {
        uint32_t t_start = millis();
        while (s_audit_phase != UM982_PHASE_IDLE && (millis() - t_start < 3000U)) {
            um982_poll_com1();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (s_audit_phase != UM982_PHASE_IDLE) {
            s_audit_phase = UM982_PHASE_IDLE;
        }
    }
}

extern "C" void um982_configure_rover_defaults(void)
{
    if (!um982_start_audit_or_config_pipeline(true)) {
        return;
    }
    if (!s_com1_tx_boot_complete) {
        uint32_t t_start = millis();
        while (s_audit_phase != UM982_PHASE_IDLE && (millis() - t_start < 7500U)) {
            um982_poll_com1();
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (s_audit_phase != UM982_PHASE_IDLE) {
            s_audit_phase     = UM982_PHASE_IDLE;
            s_cfg_after_audit = false;
        }
    }
}

extern "C" void um982_run_powerup_selftest(void)
{
    Serial.println("\n--- [UM982 Single-UART (COM1) Power-Up Self-Test] ---");

    // Preserve any warm-boot NMEA/log configuration during self-test so um982_configure_rover_defaults()
    // can audit the live receiver state without unnecessary UNLOGALL/SAVECONFIG flash wear.
    while (Serial1.available()) Serial1.read();

    Serial.println("[UM982 COM1] Testing Full-Duplex UART1 (RX=IO17, TX=IO18 @ 115200) with VERSIONA...");
    version_ack_received = false;
    send_um982_com1_command("VERSIONA");
    uint32_t t0 = millis();
    while (!version_ack_received && (millis() - t0 < 600)) {
        um982_poll_com1();
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (port_stats.com1_test_passed) {
        port_stats.single_uart_audit_passed = true;
        Serial.printf("[UM982 COM1 TEST] PASS -- RX=%lu B | Version: %s | Last ACK: %s\n",
                      (unsigned long)port_stats.com1_rx_bytes,
                      port_stats.um982_version,
                      port_stats.last_cmd_ack_line);
    } else {
        Serial.println("[UM982 COM1 TEST] FAIL -- No response on IO17/18 @ 115200");
    }
}

extern "C" void um982_driver_init(void)
{
    if (s_com1_tx_mutex == NULL) {
        s_com1_tx_mutex = xSemaphoreCreateMutex();
    }
    s_com1_tx_boot_complete = false;

    strncpy(port_stats.um982_version, "Waiting COM1...", sizeof(port_stats.um982_version) - 1);
    port_stats.um982_version[sizeof(port_stats.um982_version) - 1] = '\0';
    strncpy(port_stats.um982_mode, "INIT...", sizeof(port_stats.um982_mode) - 1);
    port_stats.um982_mode[sizeof(port_stats.um982_mode) - 1] = '\0';
    strncpy(port_stats.um982_config_summary, "Querying CONFIG...", sizeof(port_stats.um982_config_summary) - 1);
    port_stats.um982_config_summary[sizeof(port_stats.um982_config_summary) - 1] = '\0';
    strncpy(port_stats.um982_loglist, "Querying LOGLIST...", sizeof(port_stats.um982_loglist) - 1);
    port_stats.um982_loglist[sizeof(port_stats.um982_loglist) - 1] = '\0';
    strncpy(port_stats.last_com1_line, "No COM1 NMEA yet", sizeof(port_stats.last_com1_line) - 1);
    port_stats.last_com1_line[sizeof(port_stats.last_com1_line) - 1] = '\0';
    strncpy(port_stats.last_cmd_ack_line, "No CMD ACK yet", sizeof(port_stats.last_cmd_ack_line) - 1);
    port_stats.last_cmd_ack_line[sizeof(port_stats.last_cmd_ack_line) - 1] = '\0';

    pinMode(ROVER_PIN_UM982_TXD0, INPUT_PULLDOWN);
    pinMode(ROVER_PIN_UM982_RXD0, INPUT_PULLDOWN);
    delay(2);
    Serial.printf("[UM982 LINE PROBE] Single-UART COM1: IO17(ESP_RX)=%d IO18(ESP_TX)=%d | I2C1(IO47/48): SEN0696\n",
                  digitalRead(ROVER_PIN_UM982_TXD0),
                  digitalRead(ROVER_PIN_UM982_RXD0));

    // Expand Serial1 TX buffer to 4096B to absorb full-duplex RTCM3 bursts without blocking
    Serial1.setRxBufferSize(4096);
    Serial1.setTxBufferSize(4096);
    Serial1.begin(ROVER_UM982_COM1_BAUD, SERIAL_8N1, ROVER_PIN_UM982_TXD0, ROVER_PIN_UM982_RXD0);

    um982_run_powerup_selftest();
    um982_configure_rover_defaults();

    // Allow Core 0 to begin forwarding atomic CRC24Q-verified RTCM3 frames onto COM1 (Serial1)
    s_com1_tx_boot_complete = true;
}

extern "C" void um982_forward_rtcm(const uint8_t *data, size_t len)
{
    if (!s_com1_tx_boot_complete || !data || len == 0) return;
    if (s_com1_tx_mutex) {
        if (xSemaphoreTake(s_com1_tx_mutex, pdMS_TO_TICKS(50)) != pdTRUE) {
            port_stats.com1_tx_mutex_timeouts++;
            return;
        }
    }
    size_t written = Serial1.write(data, len);
    gps_time_sync.last_rtcm_forward_ms = millis();
    port_stats.rtcm_tx_bytes += (uint32_t)written; // Tracks RTCM3 bytes forwarded to UM982
    port_stats.com1_tx_bytes += (uint32_t)written;
    if (s_com1_tx_mutex) xSemaphoreGive(s_com1_tx_mutex);
}

extern "C" void um982_get_latest_state(RoverUIData_t *state_out)
{
    if (!state_out) return;

    // Age-of-data expiration: expire stale GGA, GPS UTC sync, and dual-antenna heading on UART/antenna dropout
    uint32_t now_ms = millis();
    bool com1_alive = (port_stats.last_com1_rx_ms > 0) &&
                      ((now_ms - port_stats.last_com1_rx_ms) <= 3000U);
    port_stats.com1_online      = com1_alive;
    port_stats.com1_test_passed = com1_alive;
    if (!com1_alive) {
        gnss_state.fix_type          = GNSS_FIX_NONE;
        gnss_state.num_sats          = 0;
        gnss_state.sats_in_view      = 0;
        gnss_state.h_acc_m           = 99.9f;
        gnss_state.v_acc_m           = 99.9f;
        gnss_state.slave_fix_type    = GNSS_FIX_NONE;
        gnss_state.slave_num_sats    = 0;
        gnss_state.baseline_length_m = 0.0f;
    }
    if (gps_time_sync.last_gga_ms == 0 || (now_ms - gps_time_sync.last_gga_ms > 2000U)) {
        gnss_state.fix_type = GNSS_FIX_NONE;
        gnss_state.h_acc_m  = 99.9f;
        gnss_state.v_acc_m  = 99.9f;
    }
    if (gps_time_sync.time_synced &&
        (now_ms - gps_time_sync.last_gga_ms > 15000U) &&
        (now_ms - gps_time_sync.last_rmc_ms > 15000U)) {
        gps_time_sync.time_synced = false;
    }
    if ((gnss_state.heading_source == HDG_SRC_DUAL_FIX || gnss_state.heading_source == HDG_SRC_DUAL_FLT) &&
        (gps_time_sync.last_uniheadinga_ms == 0 || (now_ms - gps_time_sync.last_uniheadinga_ms > 3000U))) {
        gnss_state.heading_valid    = false;
        gnss_state.heading_is_fixed = false;
        gnss_state.heading_source   = HDG_SRC_NONE;
        gnss_state.slave_fix_type   = GNSS_FIX_NONE;
    }

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
