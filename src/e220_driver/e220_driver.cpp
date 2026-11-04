#include "e220_driver.h"
#include "../../user_config.h"
#include "../um982_driver/um982_driver.h"
#include "../settings_manager/settings_manager.h"
#include "../storage/rtcm_logger.h"
#include <Arduino.h>
#include <driver/gpio.h>
#include <esp_timer.h>
#include <string.h>

static E220LinkStats_t e220_stats = {};
static volatile bool pending_cfg_read  = false;
static volatile bool pending_cfg_write = false;

// =========================================================================
// Mode 0 AUX (IO21) Hardware Edge Counter ISR
// Counts E220 Mode 0 wakeup/UART-output falling edges and measures LOW width.
// Suppressed during Mode 3 register config and local C0 C1 C2 C3 RSSI polls.
// =========================================================================
static volatile bool     e220_suppress_aux_isr = true;
static volatile uint32_t isr_aux_fall_count    = 0;
static volatile int64_t  isr_aux_fall_us       = 0;
static volatile uint32_t isr_aux_last_low_us   = 0;

static void IRAM_ATTR e220_aux_isr(void)
{
    if (e220_suppress_aux_isr) {
        return;
    }
    int level = gpio_get_level((gpio_num_t)ROVER_PIN_E220_AUX);
    int64_t now_us = esp_timer_get_time();
    if (level == 0) {
        isr_aux_fall_count++;
        isr_aux_fall_us = now_us;
    } else if (isr_aux_fall_us > 0) {
        int64_t dt = now_us - isr_aux_fall_us;
        if (dt >= 0 && dt < 10000000LL) {
            isr_aux_last_low_us = (uint32_t)dt;
        }
        isr_aux_fall_us = 0;
    }
}

// =========================================================================
// Sliding-Window Store-and-Verify RTCM3 Gatekeeper State
// Stages incoming Mode 0 bytes, validates 0xD3 header + 24-bit CRC24Q == 0,
// and forwards ONLY verified complete RTCM3 frames atomically to UM982 COM2.
// =========================================================================
#define RTCM_STAGE_CAPACITY      1536U
#define RTCM_TRUNC_TIMEOUT_MS    260U

static uint8_t  rx_stage[RTCM_STAGE_CAPACITY];
static size_t   rx_stage_len         = 0;
static uint32_t last_stage_rx_ms     = 0;
static uint32_t last_1094_rx_ms      = 0;
static bool     rtcm_in_resync       = false;

// =========================================================================
// Non-Blocking Mode 0 Live RSSI / Ambient Noise Register Poll State
// Queries C0 C1 C2 C3 00 02 in the post-1094 quiet gap (or every 10s when
// idle). Parses both 4-byte (C1 00 <noise> <pkt>) and 5-byte
// (C1 00 02 <noise> <pkt>) E220-900T30S replies (requires REG1 Bit 5 = 0x20).
// =========================================================================
#define E220_RSSI_POLL_INTERVAL_MS  900U
#define E220_RSSI_IDLE_POLL_MS      2000U
#define E220_RSSI_REPLY_TIMEOUT_MS  80U

static bool     rssi_query_active    = false;
static uint32_t last_rssi_poll_ms    = 0;
static uint32_t rssi_query_start_ms  = 0;
static uint8_t  rssi_reply_buf[5]    = {0};
static uint8_t  rssi_reply_len       = 0;

// Burst telemetry state
static uint32_t burst_bytes          = 0;
static uint32_t burst_last_byte_ms   = 0;
static uint32_t last_burst_end_ms    = 0;
static uint32_t last_burst_aux_edges = 0;

static uint32_t rtcm_crc24q_compute(const uint8_t *data, size_t len)
{
    uint32_t crc = 0;
    for (size_t i = 0; i < len; i++) {
        crc ^= ((uint32_t)data[i] << 16);
        for (int j = 0; j < 8; j++) {
            crc <<= 1;
            if (crc & 0x1000000U) {
                crc ^= 0x1864CFBU;
            }
        }
    }
    return crc & 0xFFFFFFU;
}

static inline bool rtcm_is_plausible_msg_type(uint16_t msg_type)
{
    return (msg_type >= 1001U && msg_type <= 1300U) ||
           (msg_type >= 4001U && msg_type <= 4095U);
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
    e220_suppress_aux_isr = true;
    rssi_query_active     = false;
    rssi_reply_len        = 0;

    e220_wait_aux_high(500);
    digitalWrite(ROVER_PIN_E220_M0, m0 ? HIGH : LOW);
    digitalWrite(ROVER_PIN_E220_M1, m1 ? HIGH : LOW);
    e220_stats.mode_m0 = m0 ? 1 : 0;
    e220_stats.mode_m1 = m1 ? 1 : 0;
    vTaskDelay(pdMS_TO_TICKS(100));
    e220_wait_aux_high(500);
    vTaskDelay(pdMS_TO_TICKS(60));

    if (m0 == 0 && m1 == 0) {
        isr_aux_fall_us       = 0;
        e220_suppress_aux_isr = false;
    }
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

    // Boot directly into Mode 0 (Transparent Mode: M0=0, M1=0) at configured UART baud (default 115200 8N1)
    // Note: E220_TXD (IO41) is ESP32 RX; E220_RXD (IO42) is ESP32 TX
    e220_set_mode(0, 0);
    Serial0.setRxBufferSize(4096);
    uint32_t baud = (current_settings.e220_uart_baud > 0) ? current_settings.e220_uart_baud : ROVER_E220_TRANSPARENT_BAUD;
    Serial0.begin(baud, SERIAL_8N1, ROVER_PIN_E220_TXD, ROVER_PIN_E220_RXD);

    attachInterrupt(digitalPinToInterrupt(ROVER_PIN_E220_AUX), e220_aux_isr, CHANGE);
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
    uint8_t uart_bits = 0xE0; // Default 115200 bps (0b111 << 5)
    uint32_t baud = current_settings.e220_uart_baud;
    if (baud >= 115200)     uart_bits = 0xE0; // 115200
    else if (baud >= 57600) uart_bits = 0xC0; // 57600
    else if (baud >= 38400) uart_bits = 0xA0; // 38400
    else if (baud >= 19200) uart_bits = 0x80; // 19200
    else if (baud >= 9600)  uart_bits = 0x60; // 9600
    else if (baud >= 4800)  uart_bits = 0x40; // 4800
    else if (baud >= 2400)  uart_bits = 0x20; // 2400
    else                    uart_bits = 0x00; // 1200

    uint8_t air_bits = 0x06; // Default 19.2 kbps (0b101)
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
    uint8_t pkt_bits = 0x00; // Default 200 bytes
    uint16_t pkt = current_settings.e220_subpacket_bytes;
    if (pkt <= 32)       pkt_bits = 0xC0;
    else if (pkt <= 64)  pkt_bits = 0x80;
    else if (pkt <= 128) pkt_bits = 0x40;
    else                 pkt_bits = 0x00; // 200 bytes

    // Bit 5 (0x20) = Enable Mode 0 Ambient Noise & Last Packet RSSI Register (C0 C1 C2 C3)
    // Note: Does NOT append extra bytes to RX UART frames (unlike REG3 Bit 7).
    const uint8_t rssi_reg_bit = 0x20;

    // Bits [1:0] = TX Power on E220-900T30S (00=30dBm, 01=27dBm, 10=24dBm, 11=21dBm)
    uint8_t pwr_bits = 0x03; // Default 21 dBm (Level 3)
    uint8_t pwr = current_settings.e220_tx_power_dbm;
    if (pwr >= 29)      pwr_bits = 0x00; // 30 dBm (Level 0)
    else if (pwr >= 26) pwr_bits = 0x01; // 27 dBm (Level 1)
    else if (pwr >= 23) pwr_bits = 0x02; // 24 dBm (Level 2)
    else                pwr_bits = 0x03; // 21 dBm (Level 3)

    return (uint8_t)(pkt_bits | rssi_reg_bit | pwr_bits);
}

static uint8_t e220_build_reg3_from_settings(void)
{
    // Bit 7 = 0 (No RSSI byte appended to RX data), Bit 6 = 0 (Transparent Mode), Bit 4 = LBT Enable
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
    Serial.printf("[%s] Decoded: UART=%lu 8N1 | Air=%.1fk | Pkt=%uB | Pwr=%udBm | RSSI_Reg=%s | Freq=%.3fMHz (CH%u) | Mode=%s | Match=%s\n",
                  tag, (unsigned long)baud, air, pkt, pwr,
                  (r1 & 0x20) ? "ON" : "OFF",
                  freq, ch,
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
    if (idx >= 9 && (resp[0] == 0xC1 || resp[0] == 0xC0) && resp[1] == 0x00 && resp[2] == 0x06) {
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
        Serial.printf("[E220 WRITE FAIL] Expected 9-byte 0xC1/0xC0 response, got %d bytes\n", idx);
    }

    // 3. Return to Mode 0 (Transparent: M0=0, M1=0) at configured UART baud
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
        Serial.printf("[E220 TEST] PASS -- E220-900T30S verified & running in Transparent Mode 0 @ %lu baud\n",
                      (unsigned long)current_settings.e220_uart_baud);
    } else {
        Serial.println("[E220 TEST] FAIL -- No register response from E220 on IO41/42 (check power/wiring)");
    }
}

static void e220_shift_stage_one_byte(void)
{
    if (rx_stage_len > 1) {
        memmove(rx_stage, rx_stage + 1, rx_stage_len - 1);
        rx_stage_len--;
    } else {
        rx_stage_len = 0;
    }
}

static void e220_process_rx_stage(void)
{
    while (rx_stage_len > 0) {
        // 1. Strip any leading non-0xD3 orphan bytes before UM982 COM2 ever sees them
        if (rx_stage[0] != 0xD3) {
            size_t d3_pos = 1;
            while (d3_pos < rx_stage_len && rx_stage[d3_pos] != 0xD3) {
                d3_pos++;
            }
            e220_stats.rtcm_orphan_bytes += (uint32_t)d3_pos;
            if (d3_pos < rx_stage_len) {
                memmove(rx_stage, rx_stage + d3_pos, rx_stage_len - d3_pos);
                rx_stage_len -= d3_pos;
            } else {
                rx_stage_len = 0;
                break;
            }
        }

        // Wait for at least 3-byte RTCM3 header (0xD3 + 10-bit length)
        if (rx_stage_len < 3) {
            break;
        }

        // 2. Validate 6 reserved zero bits in byte 1 and plausible payload length [6..1023]
        if ((rx_stage[1] & 0xFC) != 0x00) {
            e220_stats.rtcm_orphan_bytes++;
            e220_shift_stage_one_byte();
            continue;
        }

        uint16_t payload_len = ((uint16_t)(rx_stage[1] & 0x03) << 8) | rx_stage[2];
        if (payload_len < 6 || payload_len > 1023) {
            e220_stats.rtcm_orphan_bytes++;
            e220_shift_stage_one_byte();
            continue;
        }

        // 3. Validate 12-bit RTCM3 message type once 5 bytes are staged
        if (rx_stage_len >= 5) {
            uint16_t cand_type = ((uint16_t)rx_stage[3] << 4) | ((rx_stage[4] >> 4) & 0x0F);
            if (!rtcm_is_plausible_msg_type(cand_type)) {
                e220_stats.rtcm_orphan_bytes++;
                e220_shift_stage_one_byte();
                continue;
            }
        }

        size_t total_len = (size_t)payload_len + 6U;
        if (rx_stage_len < total_len) {
            // Valid header so far; wait for remaining subpacket bytes (up to RTCM_TRUNC_TIMEOUT_MS)
            break;
        }

        // 4. Full candidate frame staged -> verify 24-bit CRC24Q before touching UM982 COM2
        uint16_t msg_type = ((uint16_t)rx_stage[3] << 4) | ((rx_stage[4] >> 4) & 0x0F);
        uint32_t crc = rtcm_crc24q_compute(rx_stage, total_len);
        if (crc == 0) {
            // Verified intact RTCM3 frame -> forward atomically to UM982 COM2!
            um982_forward_rtcm_to_com2(rx_stage, total_len);

            rtcm_in_resync = false;
            e220_stats.rtcm_frames_ok++;
            e220_stats.rtcm_bytes_forwarded += (uint32_t)total_len;
            e220_stats.last_msg_type = msg_type;

            if (msg_type == 1005 || msg_type == 1006) {
                e220_stats.msg_1005_count++;
            } else if (msg_type >= 1071 && msg_type <= 1130) {
                e220_stats.msg_msm_count++;
                if (msg_type == 1074)      e220_stats.msg_1074_count++;
                else if (msg_type == 1084) e220_stats.msg_1084_count++;
                else if (msg_type == 1094) {
                    e220_stats.msg_1094_count++;
                    last_1094_rx_ms = millis();
                }
            } else if (msg_type == 1230) {
                e220_stats.msg_1230_count++;
            }

            if (current_settings.enable_serial_logging) {
                Serial.printf("[RTCM OK -> COM2] Type:%u Frame:%uB | OK:%lu Err:%lu(Trunc:%lu) Orph:%luB\n",
                              msg_type,
                              (unsigned int)total_len,
                              (unsigned long)e220_stats.rtcm_frames_ok,
                              (unsigned long)e220_stats.rtcm_crc_errors,
                              (unsigned long)e220_stats.rtcm_trunc_drops,
                              (unsigned long)e220_stats.rtcm_orphan_bytes);
            }

            if (rx_stage_len > total_len) {
                memmove(rx_stage, rx_stage + total_len, rx_stage_len - total_len);
                rx_stage_len -= total_len;
            } else {
                rx_stage_len = 0;
            }
        } else {
            // Corrupted frame or truncated head followed immediately by a new 0xD3 frame!
            // Do NOT forward to UM982 COM2; shift 1 byte so any complete frame inside rx_stage is rescued.
            if (!rtcm_in_resync) {
                rtcm_in_resync = true;
                e220_stats.rtcm_crc_errors++;
                if (current_settings.enable_serial_logging) {
                    Serial.printf("[RTCM DROP CRC] Type:%u Expected:%uB CRC:0x%06lX (NOT sent to COM2) | OK:%lu Err:%lu\n",
                                  msg_type,
                                  (unsigned int)total_len,
                                  (unsigned long)crc,
                                  (unsigned long)e220_stats.rtcm_frames_ok,
                                  (unsigned long)e220_stats.rtcm_crc_errors);
                }
            }
            e220_stats.rtcm_orphan_bytes++;
            e220_shift_stage_one_byte();
        }
    }
}

static void e220_flush_burst_log(uint32_t ref_last_byte_ms)
{
    uint32_t gap = (last_burst_end_ms > 0 && ref_last_byte_ms >= last_burst_end_ms)
                   ? (ref_last_byte_ms - last_burst_end_ms)
                   : 0;
    uint32_t cur_edges  = isr_aux_fall_count;
    uint32_t edge_delta = cur_edges - last_burst_aux_edges;
    last_burst_aux_edges = cur_edges;

    if (current_settings.enable_serial_logging) {
        Serial.printf("[E220 RX BURST] %lu B (gap: %lu ms | AUX edges:+%lu low:%.2fms)\n",
                      (unsigned long)burst_bytes,
                      (unsigned long)gap,
                      (unsigned long)edge_delta,
                      (float)isr_aux_last_low_us / 1000.0f);
    }
    last_burst_end_ms = ref_last_byte_ms;
    burst_bytes = 0;
}

static void e220_ingest_rx_bytes(const uint8_t *data, size_t len, uint32_t now)
{
    if (len == 0) {
        return;
    }

    if (burst_bytes > 0 && (now - burst_last_byte_ms >= 15)) {
        e220_flush_burst_log(burst_last_byte_ms);
    }

    burst_bytes += (uint32_t)len;
    burst_last_byte_ms = now;

    e220_stats.rx_bytes_total += (uint32_t)len;
    e220_stats.last_rx_ms = now;
    last_stage_rx_ms = now;

    // Preserve raw wire-tap to SD/FFat rtcm-base_*.txt for forensic analysis
    if (rtcm_logger_is_active()) {
        rtcm_logger_feed(data, len);
    }

    // Append into sliding-window Gatekeeper buffer (shed oldest orphan bytes if near capacity)
    if (len >= RTCM_STAGE_CAPACITY) {
        e220_stats.rtcm_orphan_bytes += (uint32_t)(rx_stage_len + (len - RTCM_STAGE_CAPACITY));
        memcpy(rx_stage, data + (len - RTCM_STAGE_CAPACITY), RTCM_STAGE_CAPACITY);
        rx_stage_len = RTCM_STAGE_CAPACITY;
    } else {
        if (rx_stage_len + len > RTCM_STAGE_CAPACITY) {
            size_t drop = (rx_stage_len + len) - RTCM_STAGE_CAPACITY;
            e220_stats.rtcm_orphan_bytes += (uint32_t)drop;
            memmove(rx_stage, rx_stage + drop, rx_stage_len - drop);
            rx_stage_len -= drop;
        }
        memcpy(rx_stage + rx_stage_len, data, len);
        rx_stage_len += len;
    }

    e220_process_rx_stage();
}

extern "C" void e220_poll_and_forward_rtcm(void)
{
    e220_stats.aux_pin_high    = (digitalRead(ROVER_PIN_E220_AUX) == HIGH);
    e220_stats.aux_fall_count  = isr_aux_fall_count;
    e220_stats.aux_last_low_us = isr_aux_last_low_us;

    if (pending_cfg_write) {
        pending_cfg_write = false;
        e220_write_default_config_registers();
    }
    if (pending_cfg_read) {
        pending_cfg_read = false;
        e220_read_config_registers();
    }

    // 1. Intercept 4-byte (0xC1 0x00 <noise> <pkt>) or 5-byte (0xC1 0x00 0x02 <noise> <pkt>) Mode 0 RSSI reply
    if (rssi_query_active) {
        uint32_t now = millis();
        while (Serial0.available() > 0 && rssi_query_active) {
            uint8_t b = (uint8_t)Serial0.read();
            rssi_reply_buf[rssi_reply_len++] = b;

            bool prefix_mismatch =
                (rssi_reply_len == 1 && rssi_reply_buf[0] != 0xC1) ||
                (rssi_reply_len == 2 && rssi_reply_buf[1] != 0x00);

            if (prefix_mismatch) {
                // An incoming LoRa packet arrived simultaneously -> cancel RSSI intercept and feed zero-loss to RTCM stage!
                rssi_query_active     = false;
                e220_suppress_aux_isr = false;
                uint8_t saved_len     = rssi_reply_len;
                rssi_reply_len        = 0;
                e220_ingest_rx_bytes(rssi_reply_buf, saved_len, now);
                break;
            }

            if (rssi_reply_len == 4 && rssi_reply_buf[2] != 0x02) {
                // 4-byte E220-900T30S MCU firmware reply: 0xC1 0x00 <noise_raw> <pkt_raw>
                e220_stats.ambient_noise_dbm = (int16_t)rssi_reply_buf[2] - 256;
                if (e220_stats.rx_bytes_total > 0 && rssi_reply_buf[3] != 0x00) {
                    e220_stats.last_pkt_rssi_dbm = (int16_t)rssi_reply_buf[3] - 256;
                }
                e220_stats.rssi_valid        = true;
                rssi_query_active            = false;
                rssi_reply_len               = 0;
                e220_suppress_aux_isr        = false;
                break;
            }

            if (rssi_reply_len == 5) {
                // 5-byte datasheet reply: 0xC1 0x00 0x02 <noise_raw> <pkt_raw>
                e220_stats.ambient_noise_dbm = (int16_t)rssi_reply_buf[3] - 256;
                if (e220_stats.rx_bytes_total > 0 && rssi_reply_buf[4] != 0x00) {
                    e220_stats.last_pkt_rssi_dbm = (int16_t)rssi_reply_buf[4] - 256;
                }
                e220_stats.rssi_valid        = true;
                rssi_query_active            = false;
                rssi_reply_len               = 0;
                e220_suppress_aux_isr        = false;
                break;
            }
        }

        if (rssi_query_active && (millis() - rssi_query_start_ms > E220_RSSI_REPLY_TIMEOUT_MS)) {
            rssi_query_active     = false;
            e220_suppress_aux_isr = false;
            if (rssi_reply_len > 0 && rssi_reply_buf[0] != 0xC1) {
                uint8_t saved_len = rssi_reply_len;
                rssi_reply_len    = 0;
                e220_ingest_rx_bytes(rssi_reply_buf, saved_len, millis());
            } else {
                rssi_reply_len = 0;
            }
        }
    }

    // 2. Drain all available bytes in Serial0 FIFO into the Store-and-Verify RTCM3 Gatekeeper
    uint8_t buf[256];
    int avail;
    while (!rssi_query_active && (avail = Serial0.available()) > 0) {
        if (avail > (int)sizeof(buf)) avail = (int)sizeof(buf);
        int n = Serial0.readBytes((char *)buf, avail);
        if (n <= 0) break;
        e220_ingest_rx_bytes(buf, (size_t)n, millis());
    }

    uint32_t now = millis();

    // 3. Flush burst diagnostic log when UART line goes idle for >= 15 ms
    if (burst_bytes > 0 && (now - burst_last_byte_ms >= 15)) {
        e220_flush_burst_log(burst_last_byte_ms);
    }

    // 4. Mid-frame truncation timeout: if tail subpacket did not arrive within 260 ms,
    //    quarantine the broken head (never forward to UM982 COM2) and rescan remaining bytes
    //    so any smaller valid frame sitting behind it in rx_stage is rescued.
    if (rx_stage_len > 0 && (now - last_stage_rx_ms > RTCM_TRUNC_TIMEOUT_MS)) {
        bool logged_trunc = false;
        while (rx_stage_len > 0) {
            if (!logged_trunc && !rtcm_in_resync) {
                uint16_t payload_len = (rx_stage_len >= 3)
                                       ? (((uint16_t)(rx_stage[1] & 0x03) << 8) | rx_stage[2])
                                       : 0;
                uint16_t msg_type    = (rx_stage_len >= 5)
                                       ? (((uint16_t)rx_stage[3] << 4) | ((rx_stage[4] >> 4) & 0x0F))
                                       : 0;
                e220_stats.rtcm_crc_errors++;
                e220_stats.rtcm_trunc_drops++;
                logged_trunc = true;
                if (current_settings.enable_serial_logging) {
                    Serial.printf("[RTCM DROP TRUNC] Type:%u Got:%u/%uB after %ums (NOT sent to COM2) | OK:%lu Err:%lu(Trunc:%lu)\n",
                                  msg_type,
                                  (unsigned int)rx_stage_len,
                                  (unsigned int)(payload_len + 6U),
                                  (unsigned int)RTCM_TRUNC_TIMEOUT_MS,
                                  (unsigned long)e220_stats.rtcm_frames_ok,
                                  (unsigned long)e220_stats.rtcm_crc_errors,
                                  (unsigned long)e220_stats.rtcm_trunc_drops);
                }
            }
            rtcm_in_resync = true;
            e220_stats.rtcm_orphan_bytes++;
            e220_shift_stage_one_byte();
            e220_process_rx_stage();
        }
        rtcm_in_resync = false;
    }

    // 5. Synchronized Mode 0 Ambient Noise & Last Packet RSSI query:
    //    - Active stream: fire 30..350ms after 1094 (inside the ~545ms inter-epoch RF quiet gap @ 38.4k)
    //      or 35..300ms after any RX burst if 1094 has not arrived within 8s.
    //    - Idle link: probe once 2.5s after boot, then every 10s when no RF traffic is arriving.
    if (!rssi_query_active &&
        e220_stats.config_valid &&
        ((e220_stats.cfg_reg1 & 0x20) != 0) &&
        rx_stage_len == 0 &&
        burst_bytes == 0 &&
        (now - last_rssi_poll_ms >= E220_RSSI_POLL_INTERVAL_MS) &&
        digitalRead(ROVER_PIN_E220_AUX) == HIGH &&
        Serial0.available() == 0) {

        bool post_1094_gap = (last_1094_rx_ms > 0) &&
                             (now - last_1094_rx_ms >= 30U) &&
                             (now - last_1094_rx_ms <= 350U);

        bool post_any_gap  = (e220_stats.last_rx_ms > 0) &&
                             (last_1094_rx_ms == 0 || (now - last_1094_rx_ms > 8000U)) &&
                             (now - e220_stats.last_rx_ms >= 35U) &&
                             (now - e220_stats.last_rx_ms <= 300U);

        bool boot_probe    = (!e220_stats.rssi_valid) &&
                             (e220_stats.last_rx_ms == 0) &&
                             (now >= 2500U);

        bool idle_probe    = (e220_stats.last_rx_ms == 0 || (now - e220_stats.last_rx_ms >= E220_RSSI_IDLE_POLL_MS)) &&
                             (now - last_rssi_poll_ms >= E220_RSSI_IDLE_POLL_MS);

        if (post_1094_gap || post_any_gap || boot_probe || idle_probe) {
            static const uint8_t kRssiCmd[6] = {0xC0, 0xC1, 0xC2, 0xC3, 0x00, 0x02};
            last_rssi_poll_ms     = now;
            rssi_query_start_ms   = now;
            rssi_reply_len        = 0;
            e220_suppress_aux_isr = true;
            rssi_query_active     = true;
            Serial0.write(kRssiCmd, sizeof(kRssiCmd));
        }
    }

    e220_stats.aux_fall_count  = isr_aux_fall_count;
    e220_stats.aux_last_low_us = isr_aux_last_low_us;
}

extern "C" void e220_get_stats(E220LinkStats_t *stats_out)
{
    if (stats_out) {
        e220_stats.aux_fall_count  = isr_aux_fall_count;
        e220_stats.aux_last_low_us = isr_aux_last_low_us;
        *stats_out = e220_stats;
    }
}
