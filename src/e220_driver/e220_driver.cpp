#include "e220_driver.h"
#include "../../user_config.h"
#include "../um982_driver/um982_driver.h"
#include <Arduino.h>

static E220LinkStats_t e220_stats = {};
static volatile bool pending_cfg_read  = false;
static volatile bool pending_cfg_write = false;

// RTCM3 State Machine for lightweight packet sniffing
typedef enum {
    RTCM_WAIT_PREAMBLE = 0,
    RTCM_WAIT_LEN_H,
    RTCM_WAIT_LEN_L,
    RTCM_READ_PAYLOAD
} RtcmParseState_t;

static RtcmParseState_t rtcm_state = RTCM_WAIT_PREAMBLE;
static uint16_t rtcm_payload_len = 0;
static uint16_t rtcm_bytes_read  = 0;
static uint8_t  rtcm_hdr_buf[6]  = {0};

static bool e220_wait_aux_high(uint32_t timeout_ms)
{
    uint32_t start = millis();
    while (digitalRead(ROVER_PIN_E220_AUX) == LOW) {
        if (millis() - start >= timeout_ms) {
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return true;
}

extern "C" void e220_set_mode(uint8_t m0, uint8_t m1)
{
    e220_wait_aux_high(200);
    digitalWrite(ROVER_PIN_E220_M0, m0 ? HIGH : LOW);
    digitalWrite(ROVER_PIN_E220_M1, m1 ? HIGH : LOW);
    e220_stats.mode_m0 = m0 ? 1 : 0;
    e220_stats.mode_m1 = m1 ? 1 : 0;
    vTaskDelay(pdMS_TO_TICKS(40));
    e220_wait_aux_high(200);
}

extern "C" void e220_driver_init(void)
{
    pinMode(ROVER_PIN_E220_AUX, INPUT_PULLUP);
    pinMode(ROVER_PIN_E220_RST, OUTPUT);
    pinMode(ROVER_PIN_E220_M0,  OUTPUT);
    pinMode(ROVER_PIN_E220_M1,  OUTPUT);

    digitalWrite(ROVER_PIN_E220_RST, HIGH);

    // Boot directly into Mode 0 (Transparent Mode: M0=0, M1=0) at 115200 8N1
    // Note: E220_TXD (IO41) is ESP32 RX; E220_RXD (IO42) is ESP32 TX
    e220_set_mode(0, 0);
    Serial0.setRxBufferSize(4096);
    Serial0.begin(ROVER_E220_TRANSPARENT_BAUD, SERIAL_8N1, ROVER_PIN_E220_TXD, ROVER_PIN_E220_RXD);
    e220_stats.aux_pin_high = (digitalRead(ROVER_PIN_E220_AUX) == HIGH);
}

extern "C" void request_e220_config_read(void)
{
    pending_cfg_read = true;
}

extern "C" void request_e220_config_write(void)
{
    pending_cfg_write = true;
}

extern "C" bool e220_write_default_config_registers(void)
{
    // 1. Switch E220 to Mode 3 (Config/Sleep: M0=1, M1=1) @ 9600 8N1
    e220_set_mode(1, 1);
    Serial0.updateBaudRate(ROVER_E220_CONFIG_BAUD);
    vTaskDelay(pdMS_TO_TICKS(30));
    while (Serial0.available()) Serial0.read();

    // 2. Write Persistent Registers (0xC0 0x00 0x06):
    //    ADDH=0x00, ADDL=0x00, REG0=0xE5 (115200 8N1, 19.2k),
    //    REG1=0x03 (200B, 21dBm), CHAN=0x37 (CH 55 = 905.125MHz), REG3=0x00 (Transparent, No LBT)
    uint8_t cmd[9] = {
        0xC0, 0x00, 0x06,
        ROVER_E220_TARGET_ADDH,
        ROVER_E220_TARGET_ADDL,
        ROVER_E220_TARGET_REG0,
        ROVER_E220_TARGET_REG1,
        ROVER_E220_TARGET_CHAN,
        ROVER_E220_TARGET_REG3
    };
    Serial0.write(cmd, sizeof(cmd));
    Serial0.flush();

    uint8_t resp[9] = {0};
    int idx = 0;
    uint32_t start = millis();
    while (millis() - start < 400 && idx < 9) {
        if (Serial0.available()) {
            resp[idx++] = (uint8_t)Serial0.read();
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }

    bool ok = false;
    if (idx >= 9 && resp[0] == 0xC1 && resp[1] == 0x00 && resp[2] == 0x06) {
        e220_stats.cfg_addr_high = resp[3];
        e220_stats.cfg_addr_low  = resp[4];
        e220_stats.cfg_reg0      = resp[5];
        e220_stats.cfg_reg1      = resp[6];
        e220_stats.cfg_channel   = resp[7];
        e220_stats.cfg_reg3      = resp[8];
        e220_stats.config_valid  = true;
        ok = true;
    }

    // 3. Return to Mode 0 (Transparent: M0=0, M1=0) @ 115200 8N1
    Serial0.updateBaudRate(ROVER_E220_TRANSPARENT_BAUD);
    e220_set_mode(0, 0);
    return ok;
}

extern "C" bool e220_read_config_registers(void)
{
    // 1. Switch E220 to Mode 3 (Config/Sleep: M0=1, M1=1), where E220 speaks 9600 8N1
    e220_set_mode(1, 1);
    Serial0.updateBaudRate(ROVER_E220_CONFIG_BAUD);
    vTaskDelay(pdMS_TO_TICKS(30));
    while (Serial0.available()) Serial0.read();

    // 2. Send Read Register Command: 0xC1 0x00 0x06 (Read 6 registers starting at 0x00)
    uint8_t cmd[3] = {0xC1, 0x00, 0x06};
    Serial0.write(cmd, 3);
    Serial0.flush();

    uint8_t resp[9] = {0};
    int idx = 0;
    uint32_t start = millis();
    while (millis() - start < 400 && idx < 9) {
        if (Serial0.available()) {
            resp[idx++] = (uint8_t)Serial0.read();
        } else {
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }

    bool ok = false;
    if (idx >= 9 && resp[0] == 0xC1 && resp[1] == 0x00 && resp[2] == 0x06) {
        e220_stats.cfg_addr_high = resp[3];
        e220_stats.cfg_addr_low  = resp[4];
        e220_stats.cfg_reg0      = resp[5];
        e220_stats.cfg_reg1      = resp[6];
        e220_stats.cfg_channel   = resp[7];
        e220_stats.cfg_reg3      = resp[8];
        e220_stats.config_valid  = true;
        ok = true;
    }

    // 3. Return immediately to Mode 0 (Transparent: M0=0, M1=0) at 115200 baud
    Serial0.updateBaudRate(ROVER_E220_TRANSPARENT_BAUD);
    e220_set_mode(0, 0);
    return ok;
}

static void feed_rtcm_sniffer(uint8_t b)
{
    switch (rtcm_state) {
        case RTCM_WAIT_PREAMBLE:
            if (b == 0xD3) {
                rtcm_state = RTCM_WAIT_LEN_H;
            }
            break;

        case RTCM_WAIT_LEN_H:
            if ((b & 0xFC) == 0x00) {
                rtcm_hdr_buf[0] = b;
                rtcm_state = RTCM_WAIT_LEN_L;
            } else if (b != 0xD3) {
                rtcm_state = RTCM_WAIT_PREAMBLE;
            }
            break;

        case RTCM_WAIT_LEN_L:
            rtcm_hdr_buf[1] = b;
            rtcm_payload_len = ((uint16_t)(rtcm_hdr_buf[0] & 0x03) << 8) | b;
            if (rtcm_payload_len > 0 && rtcm_payload_len <= 1023) {
                rtcm_bytes_read = 0;
                rtcm_state = RTCM_READ_PAYLOAD;
            } else {
                rtcm_state = RTCM_WAIT_PREAMBLE;
            }
            break;

        case RTCM_READ_PAYLOAD:
            if (rtcm_bytes_read < 2) {
                rtcm_hdr_buf[2 + rtcm_bytes_read] = b;
            }
            rtcm_bytes_read++;
            // Total frame after length is payload_len + 3 bytes CRC24Q
            if (rtcm_bytes_read >= rtcm_payload_len + 3) {
                uint16_t msg_type = ((uint16_t)rtcm_hdr_buf[2] << 4) | ((rtcm_hdr_buf[3] >> 4) & 0x0F);
                if (msg_type >= 1001 && msg_type <= 1300) {
                    e220_stats.rtcm_frames_ok++;
                    e220_stats.last_msg_type = msg_type;
                    if (msg_type == 1005 || msg_type == 1006) {
                        e220_stats.msg_1005_count++;
                    } else if (msg_type >= 1071 && msg_type <= 1127) {
                        e220_stats.msg_msm_count++;
                    }
                }
                rtcm_state = RTCM_WAIT_PREAMBLE;
            }
            break;
    }
}

extern "C" void e220_poll_and_forward_rtcm(void)
{
    e220_stats.aux_pin_high = (digitalRead(ROVER_PIN_E220_AUX) == HIGH);

    if (pending_cfg_write) {
        pending_cfg_write = false;
        e220_write_default_config_registers();
    }
    if (pending_cfg_read) {
        pending_cfg_read = false;
        e220_read_config_registers();
    }

    uint8_t buf[256];
    int avail = Serial0.available();
    if (avail > 0) {
        if (avail > (int)sizeof(buf)) avail = sizeof(buf);
        int n = Serial0.readBytes(buf, avail);
        if (n > 0) {
            e220_stats.rx_bytes_total += (uint32_t)n;
            e220_stats.last_rx_ms = millis();

            // Forward raw RTCM bytes transparently to UM982 COM2
            um982_forward_rtcm_to_com2(buf, (size_t)n);

            for (int i = 0; i < n; i++) {
                feed_rtcm_sniffer(buf[i]);
            }
        }
    }
}

extern "C" void e220_get_stats(E220LinkStats_t *stats_out)
{
    if (stats_out) {
        *stats_out = e220_stats;
    }
}
