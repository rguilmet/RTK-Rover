#ifndef ROVER_TELEMETRY_H
#define ROVER_TELEMETRY_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "../rover_api.h"

#ifdef __cplusplus
extern "C" {
#endif

// =========================================================================
// Bi-Directional LoRa Protocol Constants (0xAA 0x55 Framing)
// Completely distinct from RTCM3 (0xD3) to prevent false-framing.
// =========================================================================
#define ROVER_TELEM_SYNC0               0xAAU
#define ROVER_TELEM_SYNC1               0x55U

#define ROVER_MSG_ID_TELEMETRY          0x01U
#define ROVER_MSG_ID_CMD_RATE_SWITCH    0x02U
#define ROVER_MSG_ID_ACK_RATE_SWITCH    0x82U

#define ROVER_TELEM_PAYLOAD_LEN         33U
#define ROVER_TELEM_FRAME_LEN           39U  // 2 sync + 1 id + 1 len + 33 payload + 2 crc16

#define ROVER_CMD_SWITCH_PAYLOAD_LEN    4U
#define ROVER_CMD_SWITCH_FRAME_LEN      10U  // 2 sync + 1 id + 1 len + 4 payload + 2 crc16

#define ROVER_ACK_SWITCH_PAYLOAD_LEN    5U
#define ROVER_ACK_SWITCH_FRAME_LEN      11U  // 2 sync + 1 id + 1 len + 5 payload + 2 crc16

// Air-rate codes for REG0 Bits [2:0]
#define ROVER_AIR_CODE_2_4K             0x02U
#define ROVER_AIR_CODE_4_8K             0x03U
#define ROVER_AIR_CODE_9_6K             0x04U
#define ROVER_AIR_CODE_19_2K            0x05U
#define ROVER_AIR_CODE_38_4K            0x06U
#define ROVER_AIR_CODE_62_5K            0x07U

#pragma pack(push, 1)

// 39-Byte Uplink Telemetry Frame
typedef struct {
    uint8_t  sync[2];        // 0xAA, 0x55
    uint8_t  msg_id;         // 0x01
    uint8_t  payload_len;    // 33
    double   tip_lat;        // Ground-tip Latitude (WGS84 degrees)
    double   tip_lon;        // Ground-tip Longitude (WGS84 degrees)
    float    tip_height_m;   // Ground-tip Ellipsoidal Height (meters)
    uint16_t h_acc_mm;       // Horizontal Accuracy Estimate (millimeters)
    uint8_t  fix_type;       // GnssFixType_t (0=None, 1=Single, 2=DGPS, 4=Fix, 5=Float)
    uint8_t  sats_master;    // Master Antenna (ANT1) Satellites Used
    uint8_t  sats_slave;     // Slave Antenna (ANT2) Satellites Used
    uint16_t heading_cdeg;   // True Heading in centidegrees (0..36000 -> 0.00°..360.00°)
    uint8_t  battery_pct;    // Rover Battery Percentage (0..100%)
    uint16_t battery_mv;     // Battery Voltage in millivolts (e.g. 4020 mV)
    int8_t   rover_rssi_dbm; // Last LoRa packet RSSI measured at Rover (dBm)
    int8_t   rover_noise_dbm;// Live ambient noise measured at Rover (dBm)
    uint8_t  seq_num;        // Rolling Packet Sequence Counter (0..255)
    uint16_t crc16;          // CRC16-CCITT (polynomial 0x1021, init 0xFFFF)
} RoverUplinkTelemetry_t;

// 10-Byte Remote Air-Rate Switch Request Frame
typedef struct {
    uint8_t  sync[2];        // 0xAA, 0x55
    uint8_t  msg_id;         // 0x02
    uint8_t  payload_len;    // 4
    uint8_t  target_air_code;// ROVER_AIR_CODE_*
    uint8_t  target_channel; // e.g. 75
    uint16_t req_id;         // Unique request transaction ID
    uint16_t crc16;          // CRC16-CCITT
} RoverRateSwitchReq_t;

// 11-Byte Remote Air-Rate Switch Acknowledge Frame
typedef struct {
    uint8_t  sync[2];        // 0xAA, 0x55
    uint8_t  msg_id;         // 0x82
    uint8_t  payload_len;    // 5
    uint16_t req_id;         // Matching transaction ID
    uint8_t  status;         // 0 = OK, 1 = Error / Unsupported
    uint16_t switch_delay_ms;// Scheduled delay before both radios reconfigure registers
    uint16_t crc16;          // CRC16-CCITT
} RoverRateSwitchAck_t;

#pragma pack(pop)

// Standard CRC16-CCITT (False: poly 0x1021, init 0xFFFF, no final XOR)
uint16_t rover_crc16_ccitt(const uint8_t *data, size_t len);

// Frame builders
size_t rover_build_telemetry_packet(uint8_t *out_buf,
                                   size_t max_len,
                                   const RoverUIData_t *state,
                                   uint8_t seq_num);

size_t rover_build_rate_switch_req(uint8_t *out_buf,
                                  size_t max_len,
                                  uint8_t air_code,
                                  uint8_t channel,
                                  uint16_t req_id);

// Parser for Downlink ACK
bool rover_parse_rate_switch_ack(const uint8_t *buf,
                                 size_t len,
                                 RoverRateSwitchAck_t *ack_out);

#ifdef __cplusplus
}
#endif

#endif // ROVER_TELEMETRY_H
