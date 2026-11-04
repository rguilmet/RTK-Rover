#include "rover_telemetry.h"
#include <string.h>
#include <math.h>

uint16_t rover_crc16_ccitt(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFFU;
    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint16_t)data[i] << 8);
        for (int j = 0; j < 8; j++) {
            if (crc & 0x8000U) {
                crc = (crc << 1) ^ 0x1021U;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

size_t rover_build_telemetry_packet(uint8_t *out_buf,
                                   size_t max_len,
                                   const RoverUIData_t *state,
                                   uint8_t seq_num)
{
    if (!out_buf || !state || max_len < ROVER_TELEM_FRAME_LEN) {
        return 0;
    }

    RoverUplinkTelemetry_t pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.sync[0]     = ROVER_TELEM_SYNC0;
    pkt.sync[1]     = ROVER_TELEM_SYNC1;
    pkt.msg_id      = ROVER_MSG_ID_TELEMETRY;
    pkt.payload_len = ROVER_TELEM_PAYLOAD_LEN;

    pkt.tip_lat      = state->tip_lat;
    pkt.tip_lon      = state->tip_lon;
    pkt.tip_height_m = (float)state->tip_height_m;

    // Horizontal accuracy in millimeters (clamp to uint16 max)
    float acc_mm = state->h_acc_m * 1000.0f;
    if (acc_mm < 0.0f) acc_mm = 0.0f;
    if (acc_mm > 65535.0f) acc_mm = 65535.0f;
    pkt.h_acc_mm = (uint16_t)acc_mm;

    pkt.fix_type    = (uint8_t)state->fix_type;
    pkt.sats_master = state->num_sats;
    pkt.sats_slave  = state->slave_num_sats;

    // True heading in centidegrees (0.00° - 360.00° -> 0..36000)
    float hdg = state->true_heading_deg;
    if (hdg < 0.0f) hdg = 0.0f;
    if (hdg > 360.0f) hdg = 360.0f;
    pkt.heading_cdeg = (uint16_t)(hdg * 100.0f + 0.5f);

    pkt.battery_pct = state->battery_percent;
    // Voltage in millivolts
    float v_mv = state->battery_voltage * 1000.0f;
    if (v_mv < 0.0f) v_mv = 0.0f;
    if (v_mv > 65535.0f) v_mv = 65535.0f;
    pkt.battery_mv = (uint16_t)v_mv;

    pkt.rover_rssi_dbm  = (int8_t)state->e220_stats.last_pkt_rssi_dbm;
    pkt.rover_noise_dbm = (int8_t)state->e220_stats.ambient_noise_dbm;
    pkt.seq_num         = seq_num;

    // Calculate CRC16 over the 37 bytes preceding the crc16 field
    pkt.crc16 = rover_crc16_ccitt((const uint8_t *)&pkt, ROVER_TELEM_FRAME_LEN - 2);

    memcpy(out_buf, &pkt, sizeof(pkt));
    return ROVER_TELEM_FRAME_LEN;
}

size_t rover_build_rate_switch_req(uint8_t *out_buf,
                                  size_t max_len,
                                  uint8_t air_code,
                                  uint8_t channel,
                                  uint16_t req_id)
{
    if (!out_buf || max_len < ROVER_CMD_SWITCH_FRAME_LEN) {
        return 0;
    }

    RoverRateSwitchReq_t pkt;
    memset(&pkt, 0, sizeof(pkt));

    pkt.sync[0]          = ROVER_TELEM_SYNC0;
    pkt.sync[1]          = ROVER_TELEM_SYNC1;
    pkt.msg_id           = ROVER_MSG_ID_CMD_RATE_SWITCH;
    pkt.payload_len      = ROVER_CMD_SWITCH_PAYLOAD_LEN;
    pkt.target_air_code  = air_code;
    pkt.target_channel   = channel;
    pkt.req_id           = req_id;
    pkt.crc16            = rover_crc16_ccitt((const uint8_t *)&pkt, ROVER_CMD_SWITCH_FRAME_LEN - 2);

    memcpy(out_buf, &pkt, sizeof(pkt));
    return ROVER_CMD_SWITCH_FRAME_LEN;
}

bool rover_parse_rate_switch_ack(const uint8_t *buf,
                                 size_t len,
                                 RoverRateSwitchAck_t *ack_out)
{
    if (!buf || len < ROVER_ACK_SWITCH_FRAME_LEN) {
        return false;
    }

    if (buf[0] != ROVER_TELEM_SYNC0 ||
        buf[1] != ROVER_TELEM_SYNC1 ||
        buf[2] != ROVER_MSG_ID_ACK_RATE_SWITCH ||
        buf[3] != ROVER_ACK_SWITCH_PAYLOAD_LEN) {
        return false;
    }

    uint16_t calc_crc = rover_crc16_ccitt(buf, ROVER_ACK_SWITCH_FRAME_LEN - 2);
    uint16_t pkt_crc  = (uint16_t)buf[9] | ((uint16_t)buf[10] << 8);

    if (calc_crc != pkt_crc) {
        return false;
    }

    if (ack_out) {
        memcpy(ack_out, buf, sizeof(RoverRateSwitchAck_t));
    }
    return true;
}
