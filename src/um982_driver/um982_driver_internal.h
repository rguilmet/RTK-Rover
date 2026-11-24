#ifndef UM982_DRIVER_INTERNAL_H
#define UM982_DRIVER_INTERNAL_H

#include "um982_driver.h"
#include "../../user_config.h"
#include "../settings_manager/settings_manager.h"
#include "../storage/nmea_logger.h"
#include <Arduino.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// =========================================================================
// Shared State Across UM982 Driver Units
// =========================================================================
// Defined in um982_nmea_parser.cpp
extern RoverUIData_t    gnss_state;
extern Um982PortStats_t port_stats;
extern bool             s_com1_rx_backlog_active;

// Defined in um982_config_audit.cpp
extern SemaphoreHandle_t s_com1_tx_mutex;
extern volatile bool     s_com1_tx_boot_complete;
extern volatile bool     version_ack_received;
extern volatile bool     saveconfig_ack_received;
extern volatile bool     mode_query_ack_received;
extern volatile bool     config_query_ack_received;
extern volatile bool     loglist_query_ack_received;
extern volatile uint32_t s_last_config_line_ms;
extern char              s_cfg_hdg_len[24];

// =========================================================================
// Internal Cross-Unit Helpers
// =========================================================================
// um982_nmea_parser.cpp
void um982_strip_checksum_in_place(char *str);
void um982_mark_com1_rx_alive(void);
void um982_process_com1_line(char *line, uint16_t line_len);

// um982_config_audit.cpp
void um982_rebuild_known_state_summary(void);
void um982_parse_config_response_line(const char *line);
void um982_parse_loglist_response_line(const char *line);
void um982_record_cmd_response_line(const char *line);
bool um982_start_audit_or_config_pipeline(bool cfg_after_audit);
void um982_audit_and_config_step(uint32_t now_ms);

#endif // UM982_DRIVER_INTERNAL_H
