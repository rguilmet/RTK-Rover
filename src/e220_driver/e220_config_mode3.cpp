#include "e220_driver_internal.h"

E220LinkStats_t e220_stats        = {};
volatile bool   pending_cfg_read  = false;
volatile bool   pending_cfg_write = false;

// =========================================================================
// Mode 0 AUX (IO21) Hardware Edge Counter ISR
// Counts E220 Mode 0 wakeup/UART-output falling edges and measures LOW width.
// Suppressed during Mode 3 register config and local C0 C1 C2 C3 RSSI polls.
// =========================================================================
volatile bool     e220_suppress_aux_isr = true;
volatile uint32_t isr_aux_fall_count    = 0;
volatile int64_t  isr_aux_fall_us       = 0;
volatile uint32_t isr_aux_last_low_us   = 0;

static void IRAM_ATTR e220_aux_isr(void)
{
    if (e220_suppress_aux_isr) {
        return;
    }
    int level = gpio_get_level((gpio_num_t)ROVER_PIN_E220_AUX);
    int64_t now_us = esp_timer_get_time();
    if (level == 0) {
        isr_aux_fall_count = isr_aux_fall_count + 1;
        isr_aux_fall_us = now_us;
    } else if (isr_aux_fall_us > 0) {
        int64_t dt = now_us - isr_aux_fall_us;
        if (dt >= 0 && dt < 10000000LL) {
            isr_aux_last_low_us = (uint32_t)dt;
        }
        isr_aux_fall_us = 0;
    }
}

float e220_decode_air_rate_from_reg0(uint8_t reg0)
{
    static const float kAirRates[8] = {2.4f, 2.4f, 2.4f, 4.8f, 9.6f, 19.2f, 38.4f, 62.5f};
    return kAirRates[reg0 & 0x07U];
}

bool e220_wait_aux_high(uint32_t timeout_ms)
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

void e220_wait_tx_complete(uint32_t fall_wait_ms, uint32_t rise_timeout_ms)
{
    uint32_t t0 = millis();
    while (digitalRead(ROVER_PIN_E220_AUX) == HIGH) {
        if (millis() - t0 >= fall_wait_ms) {
            break;
        }
        delayMicroseconds(500);
    }
    e220_wait_aux_high(rise_timeout_ms);
    isr_aux_fall_us = 0;
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

    s_active_air_rate = (current_settings.e220_air_rate_kbps > 0.0f) ? current_settings.e220_air_rate_kbps : 19.2f;
    s_fallback_air_rate = s_active_air_rate;
    e220_stats.active_air_rate_kbps = s_active_air_rate;
    e220_stats.rate_switch_pending = false;
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
    float    air  = e220_decode_air_rate_from_reg0(r0);
    (void)kAirRates;
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

/**
 * Probe Mode 3 register read (0xC1 0x00 0x06) to drain and align the E220 UART
 * after entering configuration mode before issuing an EEPROM write command.
 */
static bool e220_mode3_sync_read(void)
{
    const uint8_t read_cmd[3] = { 0xC1U, 0x00U, 0x06U };
    while (Serial0.available()) Serial0.read();
    Serial0.write(read_cmd, sizeof(read_cmd));
    Serial0.flush();

    uint8_t raw[24] = {0};
    size_t idx = 0;
    uint32_t start = millis();
    uint32_t last_rx = start;

    while (millis() - start < 180U && idx < sizeof(raw)) {
        if (Serial0.available()) {
            raw[idx++] = (uint8_t)Serial0.read();
            last_rx = millis();
            for (size_t k = 0; k + 9U <= idx; ++k) {
                if (raw[k] == 0xC1U && raw[k + 1] == 0x00U && raw[k + 2] == 0x06U) {
                    return true;
                }
            }
        } else {
            if ((idx >= 9U) || (idx >= 3U && (millis() - last_rx >= 35U))) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(5));
        }
    }
    return false;
}

static void e220_exit_mode3_to_mode0(uint32_t baud)
{
    // Exit Mode 3 -> Mode 0 BEFORE switching UART baud back to operational 115200 bps,
    // waiting 25 ms before baud change, 55 ms after, and draining stale Mode 3 RX bytes.
    e220_set_mode(0, 0);
    vTaskDelay(pdMS_TO_TICKS(25));
    Serial0.updateBaudRate(baud);
    vTaskDelay(pdMS_TO_TICKS(55));
    e220_wait_aux_high(500);
    while (Serial0.available()) Serial0.read();
}

static bool e220_probe_mode0_liveness(uint32_t baud)
{
    // Verify the E220 responds to Mode 0 commands at operational baud; if wedged in Mode 3, bounce once.
    if ((e220_stats.cfg_reg1 & 0x20U) == 0U) {
        return true;
    }
    static const uint8_t kRssiCmd[6] = {0xC0, 0xC1, 0xC2, 0xC3, 0x00, 0x02};
    for (uint8_t probe_try = 0; probe_try < 2U; ++probe_try) {
        vTaskDelay(pdMS_TO_TICKS(5));
        while (Serial0.available()) Serial0.read();
        e220_suppress_aux_isr = true;
        Serial0.write(kRssiCmd, sizeof(kRssiCmd));
        Serial0.flush();

        uint8_t raw[16] = {0};
        size_t idx = 0;
        uint32_t t0 = millis();
        bool alive = false;
        while (millis() - t0 < 80U && idx < sizeof(raw)) {
            if (Serial0.available()) {
                raw[idx++] = (uint8_t)Serial0.read();
                for (size_t k = 0; k + 5U <= idx; ++k) {
                    if (raw[k] == 0xC1U && raw[k + 1] == 0x00U && raw[k + 2] == 0x02U) {
                        alive = true;
                        break;
                    }
                }
                if (alive || raw[0] == 0xD3U) {
                    alive = true;
                    break;
                }
            } else {
                vTaskDelay(pdMS_TO_TICKS(2));
            }
        }
        e220_suppress_aux_isr = false;
        isr_aux_fall_us = 0;
        if (alive) {
            return true;
        }
        if (probe_try == 0U) {
            Serial.printf("[E220 MODE0 WARN] Mode 0 liveness probe timed out (%u B); bouncing Mode 3 -> Mode 0...\n",
                          (unsigned)idx);
            e220_set_mode(1, 1);
            Serial0.updateBaudRate(ROVER_E220_CONFIG_BAUD);
            vTaskDelay(pdMS_TO_TICKS(40));
            e220_exit_mode3_to_mode0(baud);
        }
    }
    Serial.println("[E220 MODE0 WARN] Mode 0 liveness probe did not answer after bounce");
    return false;
}

extern "C" bool e220_write_default_config_registers(void)
{
    // Cancel any active Mode 0 RSSI intercept and wait for any in-flight RF TX before Mode 3
    rssi_query_active     = false;
    rssi_reply_len        = 0;
    e220_suppress_aux_isr = true;
    isr_aux_fall_us       = 0;

    uint32_t air_wait_start = millis();
    while ((int32_t)(millis() - s_tx_air_busy_until_ms) < 0) {
        if (millis() - air_wait_start >= 250U) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    e220_wait_aux_high(300);
    Serial0.flush();

    uint8_t reg0 = e220_build_reg0_from_settings();
    uint8_t reg1 = e220_build_reg1_from_settings();
    uint8_t chan = (current_settings.e220_channel <= 83) ? current_settings.e220_channel : ROVER_E220_TARGET_CHAN;
    uint8_t reg3 = e220_build_reg3_from_settings();

    Serial.printf("[E220 CFG] Writing target registers: REG0=0x%02X REG1=0x%02X CHAN=0x%02X(%u) REG3=0x%02X...\n",
                  reg0, reg1, chan, chan, reg3);

    uint8_t cmd[9] = {
        0xC0, 0x00, 0x06,
        ROVER_E220_TARGET_ADDH,
        ROVER_E220_TARGET_ADDL,
        reg0,
        reg1,
        chan,
        reg3
    };

    uint32_t baud = (current_settings.e220_uart_baud > 0) ? current_settings.e220_uart_baud : ROVER_E220_TRANSPARENT_BAUD;

    // Enter Mode 3 BEFORE switching UART baud so the E220 is off 115200-baud Mode 0 before ESP32 changes to 9600
    e220_set_mode(1, 1);
    Serial0.updateBaudRate(ROVER_E220_CONFIG_BAUD);
    vTaskDelay(pdMS_TO_TICKS(60));
    e220_mode3_sync_read();

    bool ok = false;
    for (uint8_t attempt = 1; attempt <= 4U && !ok; ++attempt) {
        if (attempt == 4U) {
            // Last-resort clean Mode 0 -> Mode 3 bounce with matching baud rates to recover a wedged parser
            e220_set_mode(0, 0);
            Serial0.updateBaudRate(baud);
            vTaskDelay(pdMS_TO_TICKS(30));
            while (Serial0.available()) Serial0.read();
            e220_set_mode(1, 1);
            Serial0.updateBaudRate(ROVER_E220_CONFIG_BAUD);
            vTaskDelay(pdMS_TO_TICKS(60));
            e220_mode3_sync_read();
        } else {
            // Stay in Mode 3 between initial attempts to avoid M0/M1 bounce framing glitches
            vTaskDelay(pdMS_TO_TICKS(35));
        }
        while (Serial0.available()) Serial0.read();

        Serial0.write(cmd, sizeof(cmd));
        Serial0.flush();

        uint8_t raw[24] = {0};
        size_t idx = 0;
        uint32_t start = millis();
        uint32_t last_rx = start;
        bool matched = false;
        size_t match_off = 0;

        while (millis() - start < 280U && idx < sizeof(raw)) {
            if (Serial0.available()) {
                raw[idx++] = (uint8_t)Serial0.read();
                last_rx = millis();
                if (idx >= 9U) {
                    for (size_t k = 0; k + 9U <= idx; ++k) {
                        if ((raw[k] == 0xC1U || raw[k] == 0xC0U) &&
                            raw[k + 1] == 0x00U &&
                            raw[k + 2] == 0x06U &&
                            raw[k + 5] == reg0 &&
                            raw[k + 6] == reg1 &&
                            raw[k + 7] == chan &&
                            raw[k + 8] == reg3) {
                            matched = true;
                            match_off = k;
                            break;
                        }
                    }
                    if (matched) {
                        break;
                    }
                }
            } else {
                if (idx >= 3U && (millis() - last_rx >= 35U)) {
                    break;
                }
                vTaskDelay(pdMS_TO_TICKS(5));
            }
        }

        if (matched) {
            e220_stats.cfg_addr_high     = raw[match_off + 3];
            e220_stats.cfg_addr_low      = raw[match_off + 4];
            e220_stats.cfg_reg0          = raw[match_off + 5];
            e220_stats.cfg_reg1          = raw[match_off + 6];
            e220_stats.cfg_channel       = raw[match_off + 7];
            e220_stats.cfg_reg3          = raw[match_off + 8];
            e220_stats.config_valid      = true;
            e220_stats.radio_online      = true;
            e220_stats.last_radio_rsp_ms = millis();
            s_rssi_timeout_streak        = 0;
            s_was_radio_online           = true;
            ok = true;
            e220_log_decoded_registers("E220 WRITE OK");
        } else {
            char hex_buf[80];
            size_t hex_pos = 0;
            if (idx == 0U) {
                snprintf(hex_buf, sizeof(hex_buf), "none");
            } else {
                for (size_t i = 0; i < idx && hex_pos + 4U < sizeof(hex_buf); ++i) {
                    int n = snprintf(hex_buf + hex_pos, sizeof(hex_buf) - hex_pos,
                                     (i == 0U) ? "%02X" : " %02X", raw[i]);
                    if (n > 0) hex_pos += (size_t)n;
                }
            }
            Serial.printf("[E220 WRITE WARN] Attempt %u/4 failed (got %u bytes, AUX=%d, hex=%s)\n",
                          attempt, (unsigned)idx, digitalRead(ROVER_PIN_E220_AUX), hex_buf);
        }
    }

    // Return to Mode 0 (Transparent: M0=0, M1=0) BEFORE restoring operational UART baud + liveness check
    e220_exit_mode3_to_mode0(baud);

    if (ok) {
        e220_probe_mode0_liveness(baud);
    } else {
        Serial.println("[E220 WRITE FAIL] All 4 Mode 3 write attempts failed! Reading actual hardware REG0...");
        if (e220_read_config_registers()) {
            float actual_rate = e220_decode_air_rate_from_reg0(e220_stats.cfg_reg0);
            s_active_air_rate                   = actual_rate;
            e220_stats.active_air_rate_kbps     = actual_rate;
            current_settings.e220_air_rate_kbps = actual_rate;
            Serial.printf("[E220 CFG] Synced firmware state to hardware REG0 (%.1f kbps)\n", actual_rate);
        }
    }
    return ok;
}

extern "C" bool e220_read_config_registers(void)
{
    rssi_query_active     = false;
    rssi_reply_len        = 0;
    e220_suppress_aux_isr = true;
    isr_aux_fall_us       = 0;
    e220_wait_aux_high(300);
    Serial0.flush();

    // 1. Switch E220 to Mode 3 (Config/Sleep: M0=1, M1=1) BEFORE switching UART to 9600 8N1
    e220_set_mode(1, 1);
    Serial0.updateBaudRate(ROVER_E220_CONFIG_BAUD);
    vTaskDelay(pdMS_TO_TICKS(60));
    e220_mode3_sync_read();

    uint8_t resp[24] = {0};
    int idx = 0;
    bool ok = false;

    for (int attempt = 0; attempt < 2 && !ok; attempt++) {
        vTaskDelay(pdMS_TO_TICKS(20));
        while (Serial0.available()) Serial0.read();

        // 2. Send Read Register Command: 0xC1 0x00 0x06 (Read 6 registers starting at 0x00)
        uint8_t cmd[3] = {0xC1, 0x00, 0x06};
        Serial0.write(cmd, 3);
        Serial0.flush();

        idx = 0;
        int match_off = -1;
        uint32_t start = millis();
        while (millis() - start < 500 && idx < (int)sizeof(resp)) {
            if (Serial0.available()) {
                resp[idx++] = (uint8_t)Serial0.read();
                for (int k = 0; k + 9 <= idx; k++) {
                    if (resp[k] == 0xC1 && resp[k + 1] == 0x00 && resp[k + 2] == 0x06) {
                        match_off = k;
                        break;
                    }
                }
                if (match_off >= 0) {
                    break;
                }
            } else {
                vTaskDelay(pdMS_TO_TICKS(5));
            }
        }

        if (match_off >= 0) {
            e220_stats.cfg_addr_high     = resp[match_off + 3];
            e220_stats.cfg_addr_low      = resp[match_off + 4];
            e220_stats.cfg_reg0          = resp[match_off + 5];
            e220_stats.cfg_reg1          = resp[match_off + 6];
            e220_stats.cfg_channel       = resp[match_off + 7];
            e220_stats.cfg_reg3          = resp[match_off + 8];
            e220_stats.config_valid      = true;
            e220_stats.radio_online      = true;
            e220_stats.last_radio_rsp_ms = millis();
            s_rssi_timeout_streak        = 0;
            s_was_radio_online           = true;
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

    // 3. Exit Mode 3 -> Mode 0 BEFORE restoring operational UART baud + verify Mode 0 liveness
    uint32_t baud = (current_settings.e220_uart_baud > 0) ? current_settings.e220_uart_baud : ROVER_E220_TRANSPARENT_BAUD;
    e220_exit_mode3_to_mode0(baud);
    if (ok) {
        e220_probe_mode0_liveness(baud);
    }
    return ok;
}

static bool e220_verify_pin_continuity(uint8_t pin, const char *pin_name)
{
    // Start with E220 in Mode 3 (M0=1, M1=1) and ensure AUX is steady HIGH
    digitalWrite(ROVER_PIN_E220_M0, HIGH);
    digitalWrite(ROVER_PIN_E220_M1, HIGH);
    vTaskDelay(pdMS_TO_TICKS(40));
    if (!e220_wait_aux_high(300)) {
        Serial.printf("[E220 TEST] %s continuity check skipped (AUX stuck LOW)\n", pin_name);
        return false;
    }

    // Pull the tested pin LOW (transition from Mode 3 to Mode 1 or Mode 2)
    digitalWrite(pin, LOW);

    // Per E220 manual Section 5.2.4, switching modes causes AUX to pulse LOW for 2..15ms.
    // If the wire is intact, E220 detects the transition and pulls AUX LOW.
    // If the wire is open/floating, E220's weak internal pullup keeps the pin HIGH (no mode change).
    bool saw_low = false;
    uint32_t start = millis();
    while (millis() - start < 40) {
        if (digitalRead(ROVER_PIN_E220_AUX) == LOW) {
            saw_low = true;
            break;
        }
        delayMicroseconds(250);
    }

    // Restore pin to HIGH (return to Mode 3)
    digitalWrite(pin, HIGH);
    vTaskDelay(pdMS_TO_TICKS(30));
    e220_wait_aux_high(300);

    Serial.printf("[E220 TEST] %s Continuity: %s (AUX pulse=%s)\n",
                  pin_name, saw_low ? "PASS (OK)" : "FAIL (OPEN/FLOAT)", saw_low ? "DETECTED" : "MISSING");
    return saw_low;
}

extern "C" void e220_run_powerup_selftest(void)
{
    Serial.println("\n--- [E220-900T30S Power-Up Self-Test (UART0: RX=IO41, TX=IO42)] ---");
    Serial.printf("[E220 PINS] AUX(IO21)=%d | M0(IO39)=%d | M1(IO40)=%d | RST(IO38)=%d\n",
                  digitalRead(ROVER_PIN_E220_AUX),
                  digitalRead(ROVER_PIN_E220_M0),
                  digitalRead(ROVER_PIN_E220_M1),
                  digitalRead(ROVER_PIN_E220_RST));

    // 1. Hardware Wire Continuity Self-Test via Mode-Switch AUX Pulses
    e220_stats.m0_wire_ok = e220_verify_pin_continuity(ROVER_PIN_E220_M0, "M0 (IO39)");
    e220_stats.m1_wire_ok = e220_verify_pin_continuity(ROVER_PIN_E220_M1, "M1 (IO40)");

    // 2. Read Configuration Registers in Mode 3
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

extern "C" void e220_get_stats(E220LinkStats_t *stats_out)
{
    if (stats_out) {
        e220_stats.aux_fall_count  = isr_aux_fall_count;
        e220_stats.aux_last_low_us = isr_aux_last_low_us;
        *stats_out = e220_stats;
    }
}
