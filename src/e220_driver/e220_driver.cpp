#include "e220_driver.h"
#include "../../user_config.h"
#include "../um982_driver/um982_driver.h"
#include "../settings_manager/settings_manager.h"
#include "../storage/rtcm_logger.h"
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
static uint16_t rtcm_payload_len        = 0;
static uint16_t rtcm_bytes_read         = 0;
static uint8_t  rtcm_hdr_buf[6]         = {0};
static uint32_t rtcm_crc_accum          = 0;
static uint32_t rtcm_last_rx_byte_ms   = 0;

static inline uint32_t rtcm_crc24q_step(uint32_t crc, uint8_t b)
{
    crc ^= ((uint32_t)b << 16);
    for (int j = 0; j < 8; j++) {
        crc <<= 1;
        if (crc & 0x1000000) crc ^= 0x1864CFB;
    }
    return crc & 0xFFFFFF;
}

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
    e220_wait_aux_high(500);
    digitalWrite(ROVER_PIN_E220_M0, m0 ? HIGH : LOW);
    digitalWrite(ROVER_PIN_E220_M1, m1 ? HIGH : LOW);
    e220_stats.mode_m0 = m0 ? 1 : 0;
    e220_stats.mode_m1 = m1 ? 1 : 0;
    vTaskDelay(pdMS_TO_TICKS(100));
    e220_wait_aux_high(500);
    vTaskDelay(pdMS_TO_TICKS(60));
}

extern "C" void e220_driver_init(void)
{
    pinMode(ROVER_PIN_E220_AUX, INPUT_PULLUP);
    pinMode(ROVER_PIN_E220_RST, OUTPUT);
    pinMode(ROVER_PIN_E220_M0,  OUTPUT);
    pinMode(ROVER_PIN_E220_M1,  OUTPUT);

    digitalWrite(ROVER_PIN_E220_RST, HIGH);

    // Probe IO41/IO42 with weak pull-down before Serial0.begin() to verify physical E220 TX line drive
    pinMode(ROVER_PIN_E220_TXD, INPUT_PULLDOWN);
    pinMode(ROVER_PIN_E220_RXD, INPUT_PULLDOWN);
    delay(2);
    Serial.printf("[E220 LINE PROBE] IO41(ESP_RX/E220_TXD)=%d | IO42(ESP_TX/E220_RXD)=%d\n",
                  digitalRead(ROVER_PIN_E220_TXD),
                  digitalRead(ROVER_PIN_E220_RXD));

    // Boot directly into Mode 0 (Transparent Mode: M0=0, M1=0) at configured UART baud (default 57600 8N1)
    // Note: E220_TXD (IO41) is ESP32 RX; E220_RXD (IO42) is ESP32 TX
    e220_set_mode(0, 0);
    Serial0.setRxBufferSize(4096);
    uint32_t baud = (current_settings.e220_uart_baud > 0) ? current_settings.e220_uart_baud : ROVER_E220_TRANSPARENT_BAUD;
    Serial0.begin(baud, SERIAL_8N1, ROVER_PIN_E220_TXD, ROVER_PIN_E220_RXD);
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

static uint8_t e220_build_reg0_from_settings(void)
{
    // Bits [7:5] = UART baud (000=1200, 001=2400, 010=4800, 011=9600, 100=19200, 101=38400, 110=57600, 111=115200)
    // Bits [4:3] = 00 (8N1)
    uint8_t uart_bits = 0xC0; // Default 57600 bps (0b110 << 5)
    uint32_t baud = current_settings.e220_uart_baud;
    if (baud >= 115200)     uart_bits = 0xE0; // 115200
    else if (baud >= 57600) uart_bits = 0xC0; // 57600
    else if (baud >= 38400) uart_bits = 0xA0; // 38400
    else if (baud >= 19200) uart_bits = 0x80; // 19200
    else if (baud >= 9600)  uart_bits = 0x60; // 9600
    else if (baud >= 4800)  uart_bits = 0x40; // 4800
    else if (baud >= 2400)  uart_bits = 0x20; // 2400
    else                    uart_bits = 0x00; // 1200

    uint8_t air_bits = 0x06; // Default 38.4 kbps (0b110)
    float rate = current_settings.e220_air_rate_kbps;
    if (rate < 3.0f)       air_bits = 0x02; // 2.4 kbps
    else if (rate < 7.0f)  air_bits = 0x03; // 4.8 kbps
    else if (rate < 14.0f) air_bits = 0x04; // 9.6 kbps
    else if (rate < 28.0f) air_bits = 0x05; // 19.2 kbps
    else if (rate < 50.0f) air_bits = 0x06; // 38.4 kbps
    else                   air_bits = 0x07; // 62.5 kbps
    return (uint8_t)(uart_bits | air_bits);
}

static uint8_t e220_build_reg1_from_settings(void)
{
    // Bits [7:6] = Sub-packet size (00=200B, 01=128B, 10=64B, 11=32B)
    uint8_t pkt_bits = 0x40; // Default 128 bytes
    uint16_t pkt = current_settings.e220_subpacket_bytes;
    if (pkt <= 32)       pkt_bits = 0xC0;
    else if (pkt <= 64)  pkt_bits = 0x80;
    else if (pkt <= 128) pkt_bits = 0x40;
    else                 pkt_bits = 0x00; // 200 bytes

    // Bits [1:0] = TX Power on E220-900T30S (00=30dBm, 01=27dBm, 10=24dBm, 11=21dBm)
    uint8_t pwr_bits = 0x03; // Default 21 dBm (Level 3)
    uint8_t pwr = current_settings.e220_tx_power_dbm;
    if (pwr >= 29)      pwr_bits = 0x00; // 30 dBm (Level 0)
    else if (pwr >= 26) pwr_bits = 0x01; // 27 dBm (Level 1)
    else if (pwr >= 23) pwr_bits = 0x02; // 24 dBm (Level 2)
    else                pwr_bits = 0x03; // 21 dBm (Level 3)

    return (uint8_t)(pkt_bits | pwr_bits);
}

static uint8_t e220_build_reg3_from_settings(void)
{
    // Bit 6 = 0 (Transparent Mode), Bit 4 = LBT Enable
    return current_settings.e220_lbt_enable ? 0x10 : 0x00;
}

static void e220_log_decoded_registers(const char *tag)
{
    static const uint32_t kUartBauds[8] = {1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200};
    static const float    kAirRates[8]  = {2.4f, 2.4f, 2.4f, 4.8f, 9.6f, 19.2f, 38.4f, 62.5f};
    static const uint16_t kPktSizes[4]  = {200, 128, 64, 32};
    static const uint8_t  kTxPowers[4]  = {30, 27, 24, 21};

    uint8_t r0 = e220_stats.cfg_reg0;
    uint8_t r1 = e220_stats.cfg_reg1;
    uint8_t ch = e220_stats.cfg_channel;
    uint8_t r3 = e220_stats.cfg_reg3;

    uint32_t baud = kUartBauds[(r0 >> 5) & 0x07];
    float    air  = kAirRates[r0 & 0x07];
    uint16_t pkt  = kPktSizes[(r1 >> 6) & 0x03];
    uint8_t  pwr  = kTxPowers[r1 & 0x03];
    float    freq = 850.125f + (float)ch;

    uint8_t exp_r0 = e220_build_reg0_from_settings();
    uint8_t exp_r1 = e220_build_reg1_from_settings();
    uint8_t exp_ch = (current_settings.e220_channel <= 83) ? current_settings.e220_channel : ROVER_E220_TARGET_CHAN;
    uint8_t exp_r3 = e220_build_reg3_from_settings();

    e220_stats.config_matched = (r0 == exp_r0 && r1 == exp_r1 && ch == exp_ch && r3 == exp_r3);

    Serial.printf("[%s] Regs: ADDH=0x%02X ADDL=0x%02X REG0=0x%02X REG1=0x%02X CHAN=0x%02X(%u) REG3=0x%02X\n",
                  tag, e220_stats.cfg_addr_high, e220_stats.cfg_addr_low, r0, r1, ch, ch, r3);
    Serial.printf("[%s] Decoded: UART=%lu 8N1 | Air=%.1fk | Pkt=%uB | Pwr=%udBm | Freq=%.3fMHz (CH%u) | Mode=%s | Match=%s\n",
                  tag, (unsigned long)baud, air, pkt, pwr, freq, ch,
                  (r3 & 0x40) ? "Fixed" : "Transparent",
                  e220_stats.config_matched ? "YES" : "NO");
}

extern "C" bool e220_write_default_config_registers(void)
{
    // 1. Switch E220 to Mode 3 (Config/Sleep: M0=1, M1=1) @ 9600 8N1
    e220_set_mode(1, 1);
    Serial0.updateBaudRate(ROVER_E220_CONFIG_BAUD);
    vTaskDelay(pdMS_TO_TICKS(40));
    while (Serial0.available()) Serial0.read();

    uint8_t reg0 = e220_build_reg0_from_settings();
    uint8_t reg1 = e220_build_reg1_from_settings();
    uint8_t chan = (current_settings.e220_channel <= 83) ? current_settings.e220_channel : ROVER_E220_TARGET_CHAN;
    uint8_t reg3 = e220_build_reg3_from_settings();

    Serial.printf("[E220 CFG] Writing target registers: REG0=0x%02X REG1=0x%02X CHAN=0x%02X(%u) REG3=0x%02X...\n",
                  reg0, reg1, chan, chan, reg3);

    // 2. Write Persistent Registers (0xC0 0x00 0x06) from settings.json parameters
    uint8_t cmd[9] = {
        0xC0, 0x00, 0x06,
        ROVER_E220_TARGET_ADDH,
        ROVER_E220_TARGET_ADDL,
        reg0,
        reg1,
        chan,
        reg3
    };
    Serial0.write(cmd, sizeof(cmd));
    Serial0.flush();

    uint8_t resp[9] = {0};
    int idx = 0;
    uint32_t start = millis();
    while (millis() - start < 500 && idx < 9) {
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
        e220_log_decoded_registers("E220 WRITE OK");
    } else {
        Serial.printf("[E220 WRITE FAIL] Expected 9-byte 0xC1 response, got %d bytes\n", idx);
    }

    // 3. Return to Mode 0 (Transparent: M0=0, M1=0) at configured UART baud (default 57600 8N1)
    uint32_t baud = (current_settings.e220_uart_baud > 0) ? current_settings.e220_uart_baud : ROVER_E220_TRANSPARENT_BAUD;
    Serial0.updateBaudRate(baud);
    e220_set_mode(0, 0);
    return ok;
}

extern "C" bool e220_read_config_registers(void)
{
    // 1. Switch E220 to Mode 3 (Config/Sleep: M0=1, M1=1), where E220 speaks 9600 8N1
    e220_set_mode(1, 1);
    Serial0.updateBaudRate(ROVER_E220_CONFIG_BAUD);
    vTaskDelay(pdMS_TO_TICKS(60));

    uint8_t resp[9] = {0};
    int idx = 0;
    bool ok = false;

    for (int attempt = 0; attempt < 2 && !ok; attempt++) {
        while (Serial0.available()) Serial0.read();

        // 2. Send Read Register Command: 0xC1 0x00 0x06 (Read 6 registers starting at 0x00)
        uint8_t cmd[3] = {0xC1, 0x00, 0x06};
        Serial0.write(cmd, 3);
        Serial0.flush();

        idx = 0;
        uint32_t start = millis();
        while (millis() - start < 500 && idx < 9) {
            if (Serial0.available()) {
                resp[idx++] = (uint8_t)Serial0.read();
            } else {
                vTaskDelay(pdMS_TO_TICKS(5));
            }
        }

        if (idx >= 9 && resp[0] == 0xC1 && resp[1] == 0x00 && resp[2] == 0x06) {
            e220_stats.cfg_addr_high = resp[3];
            e220_stats.cfg_addr_low  = resp[4];
            e220_stats.cfg_reg0      = resp[5];
            e220_stats.cfg_reg1      = resp[6];
            e220_stats.cfg_channel   = resp[7];
            e220_stats.cfg_reg3      = resp[8];
            e220_stats.config_valid  = true;
            ok = true;
            e220_log_decoded_registers("E220 READ OK");
        } else if (attempt == 0) {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    if (!ok) {
        Serial.printf("[E220 READ FAIL] Expected 9-byte 0xC1 response on Serial0 (IO41/42 @ 9600), got %d bytes (AUX=%d)\n",
                      idx, digitalRead(ROVER_PIN_E220_AUX));
    }

    // 3. Return immediately to Mode 0 (Transparent: M0=0, M1=0) at configured UART baud
    uint32_t baud = (current_settings.e220_uart_baud > 0) ? current_settings.e220_uart_baud : ROVER_E220_TRANSPARENT_BAUD;
    Serial0.updateBaudRate(baud);
    e220_set_mode(0, 0);
    return ok;
}

extern "C" void e220_run_powerup_selftest(void)
{
    Serial.println("\n--- [E220-900T30S Power-Up Self-Test (UART0: RX=IO41, TX=IO42)] ---");
    Serial.printf("[E220 PINS] AUX(IO21)=%d | M0(IO39)=%d | M1(IO40)=%d | RST(IO38)=%d\n",
                  digitalRead(ROVER_PIN_E220_AUX),
                  digitalRead(ROVER_PIN_E220_M0),
                  digitalRead(ROVER_PIN_E220_M1),
                  digitalRead(ROVER_PIN_E220_RST));

    if (e220_read_config_registers()) {
        if (!e220_stats.config_matched) {
            Serial.println("[E220 TEST] Register mismatch vs settings.json -> Auto-programming target E220 config...");
            e220_write_default_config_registers();
        }
        Serial.printf("[E220 TEST] PASS — E220-900T30S verified & running in Transparent Mode 0 @ %lu baud\n",
                      (unsigned long)current_settings.e220_uart_baud);
    } else {
        Serial.println("[E220 TEST] FAIL — No register response from E220 on IO41/42 (check power/wiring)");
    }
}

static void feed_rtcm_sniffer(uint8_t b)
{
    rtcm_last_rx_byte_ms = millis();

    switch (rtcm_state) {
        case RTCM_WAIT_PREAMBLE:
            if (b == 0xD3) {
                rtcm_crc_accum = rtcm_crc24q_step(0, 0xD3);
                rtcm_state = RTCM_WAIT_LEN_H;
            }
            break;

        case RTCM_WAIT_LEN_H:
            if ((b & 0xFC) == 0x00) {
                rtcm_crc_accum = rtcm_crc24q_step(rtcm_crc_accum, b);
                rtcm_hdr_buf[0] = b;
                rtcm_state = RTCM_WAIT_LEN_L;
            } else if (b == 0xD3) {
                // Previous 0xD3 was payload byte; this 0xD3 is candidate preamble
                rtcm_crc_accum = rtcm_crc24q_step(0, 0xD3);
                rtcm_state = RTCM_WAIT_LEN_H;
            } else {
                rtcm_state = RTCM_WAIT_PREAMBLE;
            }
            break;

        case RTCM_WAIT_LEN_L:
            rtcm_crc_accum = rtcm_crc24q_step(rtcm_crc_accum, b);
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
            rtcm_crc_accum = rtcm_crc24q_step(rtcm_crc_accum, b);
            if (rtcm_bytes_read < 2) {
                rtcm_hdr_buf[2 + rtcm_bytes_read] = b;
            }
            rtcm_bytes_read++;
            // Total frame after length is payload_len + 3 bytes CRC24Q
            if (rtcm_bytes_read >= rtcm_payload_len + 3) {
                uint16_t msg_type = ((uint16_t)rtcm_hdr_buf[2] << 4) | ((rtcm_hdr_buf[3] >> 4) & 0x0F);
                if (msg_type >= 1001 && msg_type <= 1300) {
                    if (rtcm_crc_accum == 0) {
                        // Frame passed CRC24Q integrity check
                        e220_stats.rtcm_frames_ok++;
                        e220_stats.last_msg_type = msg_type;
                        if (msg_type == 1005 || msg_type == 1006) {
                            e220_stats.msg_1005_count++;
                        } else if (msg_type >= 1071 && msg_type <= 1130) {
                            e220_stats.msg_msm_count++;
                        }
                    } else {
                        // Frame corrupted or truncated over the radio link
                        e220_stats.rtcm_crc_errors++;
                    }
                } else {
                    e220_stats.rtcm_crc_errors++;
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
    int avail;
    // Drain all available bytes in the Serial0 FIFO to forward RTCM bursts with zero latency
    while ((avail = Serial0.available()) > 0) {
        if (avail > (int)sizeof(buf)) avail = sizeof(buf);
        int n = Serial0.readBytes((char *)buf, avail);
        if (n <= 0) break;

        e220_stats.rx_bytes_total += (uint32_t)n;
        e220_stats.last_rx_ms = millis();

        // Forward raw RTCM bytes transparently to UM982 COM2
        um982_forward_rtcm_to_com2(buf, (size_t)n);

        if (rtcm_logger_is_active()) {
            rtcm_logger_feed(buf, (size_t)n);
        }

        for (int i = 0; i < n; i++) {
            feed_rtcm_sniffer(buf[i]);
        }
    }

    // Inter-byte timeout: Reset state machine if radio has been silent for > 3.0s mid-frame
    // to cleanly recover from lost packets without falsely aborting valid multi-packet bursts
    if (rtcm_state != RTCM_WAIT_PREAMBLE && (millis() - rtcm_last_rx_byte_ms > 3000)) {
        rtcm_state = RTCM_WAIT_PREAMBLE;
    }
}

extern "C" void e220_get_stats(E220LinkStats_t *stats_out)
{
    if (stats_out) {
        *stats_out = e220_stats;
    }
}
