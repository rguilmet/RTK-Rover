#ifndef USER_CONFIG_H
#define USER_CONFIG_H

#include "driver/gpio.h"

// =========================================================================
// Firmware Identity & Semantic Versioning
// =========================================================================
#define ROVER_FIRMWARE_VERSION "v0.2.14"

// =========================================================================
// Display Configuration (Waveshare ESP32-S3-Touch-LCD-3.5B)
// Controller: AXS15231B QSPI + Capacitive Touch (320x480 Portrait)
// =========================================================================
#define LCD_HOST                  SPI3_HOST

#define ROVER_LCD_H_RES           320
#define ROVER_LCD_V_RES           480
#define LCD_NOROT_HRES            320
#define LCD_NOROT_VRES            480

#define USER_DISP_ROT_90          1
#define USER_DISP_ROT_NONE        0
#define Rotated                   USER_DISP_ROT_NONE

#define LVGL_DMA_BUFF_LEN         (LCD_NOROT_HRES * 48 * 2)
#define LVGL_SPIRAM_BUFF_LEN      (ROVER_LCD_H_RES * ROVER_LCD_V_RES * 2)

#define ROVER_PIN_LCD_CS          (GPIO_NUM_12)
#define ROVER_PIN_LCD_PCLK        (GPIO_NUM_5)
#define ROVER_PIN_LCD_DATA0       (GPIO_NUM_1)
#define ROVER_PIN_LCD_DATA1       (GPIO_NUM_2)
#define ROVER_PIN_LCD_DATA2       (GPIO_NUM_3)
#define ROVER_PIN_LCD_DATA3       (GPIO_NUM_4)
#define ROVER_PIN_LCD_RST         (-1) // Driven via TCA9554 EXIO1
#define ROVER_PIN_BK_LIGHT        (GPIO_NUM_6)

// =========================================================================
// Shared Internal I2C0 Bus (Touch, TCA9554, RTC, IMU, AXP2101, ES8311)
// =========================================================================
#define ROVER_PIN_I2C0_SCL        (GPIO_NUM_7)
#define ROVER_PIN_I2C0_SDA        (GPIO_NUM_8)

#define ROVER_I2C_ADDR_TCA9554    0x20
#define ROVER_I2C_ADDR_AXP2101    0x34
#define ROVER_I2C_ADDR_TOUCH      0x3B
#define ROVER_I2C_ADDR_RTC        0x51
#define ROVER_I2C_ADDR_IMU        0x6B
#define ROVER_I2C_ADDR_ES8311     0x18

// =========================================================================
// TCA9554 IO Expander Pin Bitmasks (ESP32-S3-Touch-LCD-3.5B V2.0 Schematic)
// =========================================================================
#define ROVER_EXIO_CAM_PWDN       (1ULL << 0) // EXIO0 (P0)
#define ROVER_EXIO_LCD_RST        (1ULL << 1) // EXIO1 (P1)
#define ROVER_EXIO_TP_INT         (1ULL << 2) // EXIO2 (P2)
#define ROVER_EXIO_SD_CS          (1ULL << 3) // EXIO3 (P3) - Active LOW SD Chip Select
#define ROVER_EXIO_RTC_INT        (1ULL << 4) // EXIO4 (P4)
#define ROVER_EXIO_AXP_IRQ        (1ULL << 5) // EXIO5 (P5)
#define ROVER_EXIO_SYS_OUT        (1ULL << 6) // EXIO6 (P6) - PWR Button State
#define ROVER_EXIO_PA_CTRL        (1ULL << 7) // EXIO7 (P7) - Speaker Amplifier Enable

// =========================================================================
// SD Card SPI Pins (CS is controlled via ROVER_EXIO_SD_CS)
// =========================================================================
#define ROVER_PIN_SD_MISO         9
#define ROVER_PIN_SD_MOSI         10
#define ROVER_PIN_SD_SCLK         11
#define ROVER_PIN_SD_CS           (-1)

// =========================================================================
// Physical BOOT Button (GPIO 0)
// Short-Press (Tile 2 ONLY) = Log Survey Waypoint
// Long-Press (All Screens)  = Save Screenshot to SD Card
// =========================================================================
#define ROVER_PIN_BOOT_BTN        0

// =========================================================================
// Holybro H-RTK Unicore UM982 Dual-Antenna GNSS UART Mapping
// NOTE: Pin labels in hardware documentation are device-relative:
//   UM982_TXDn = UM982 Transmit Output -> ESP32 UART RX Input
//   UM982_RXDn = UM982 Receive Input   <- ESP32 UART TX Output
// =========================================================================
// UM982 COM1 (Primary Configuration, Commands, NMEA & Dual-Antenna Heading)
#define ROVER_UM982_COM1_BAUD     115200
#define ROVER_PIN_UM982_TXD0      17  // Connected to ESP32 RX (Serial1 RX)
#define ROVER_PIN_UM982_RXD0      18  // Connected to ESP32 TX (Serial1 TX)

// UM982 COM2 (Dedicated RTCM3 Correction Stream Input)
#define ROVER_UM982_COM2_BAUD     115200
#define ROVER_PIN_UM982_TXD1      47  // Connected to ESP32 RX (Serial2 RX)
#define ROVER_PIN_UM982_RXD1      48  // Connected to ESP32 TX (Serial2 TX)

// =========================================================================
// Ebyte E220-900T30S LoRa Module Mapping & Radio Configuration
// NOTE: Device-relative labels:
//   E220_TXD = E220 Transmit Output -> ESP32 UART RX Input
//   E220_RXD = E220 Receive Input   <- ESP32 UART TX Output
//
// Target Radio Configuration (850.125 MHz + CH * 1 MHz):
//   - RF Channel:    55 (0x37) -> 905.125 MHz (avoids EcoWitt Weather Station)
//   - UART Rate:     57600 bps, 8N1  (REG0[7:5]=110, REG0[4:3]=00) -> paces UART to prevent 1000B FIFO overrun on 1400B bursts
//   - Air Data Rate: 38.4 kbps       (REG0[2:0]=110) -> REG0 = 0xC6 (configurable in settings.json)
//   - Packet Size:   128 bytes       (REG1[7:6]=01)
//   - TX Power:      21 dBm (Level 3)(REG1[1:0]=11)  -> REG1 = 0x43
//   - Mode / LBT:    Transparent (REG3[6]=0), LBT Disabled (REG3[4]=0) -> REG3 = 0x00
// =========================================================================
#define ROVER_E220_TRANSPARENT_BAUD        57600
#define ROVER_E220_CONFIG_BAUD             9600
#define ROVER_E220_TARGET_ADDH             0x00
#define ROVER_E220_TARGET_ADDL             0x00
#define ROVER_E220_TARGET_REG0             0xC6  // 57600 8N1 (0xC0), 38.4k Air Rate (0x06)
#define ROVER_E220_TARGET_REG1             0x43  // 128-byte packet (0x40), 21 dBm TX Power (0x03)
#define ROVER_E220_TARGET_CHAN             55    // CH 55 (0x37) = 905.125 MHz
#define ROVER_E220_TARGET_REG3             0x00  // Transparent Mode, LBT Disabled
#define ROVER_E220_DEFAULT_AIR_RATE_KBPS   38.4f // Configurable via settings.json (2.4, 4.8, 9.6, 19.2, 38.4, 62.5)
#define ROVER_E220_DEFAULT_TX_POWER_DBM    21    // Configurable via settings.json (30, 27, 24, 21)
#define ROVER_E220_DEFAULT_SUBPACKET_BYTES 128   // Configurable via settings.json (200, 128, 64, 32)
#define ROVER_PIN_E220_AUX          21    // Input: HIGH = Ready/Idle, LOW = Busy
#define ROVER_PIN_E220_RST          38    // Output: Hardware Reset
#define ROVER_PIN_E220_M0           39    // Output: Mode 0 (0=Transparent, 1=Config)
#define ROVER_PIN_E220_M1           40    // Output: Mode 1 (0=Transparent, 1=Config)
#define ROVER_PIN_E220_TXD          41    // Connected to ESP32 RX (Serial0 RX)
#define ROVER_PIN_E220_RXD          42    // Connected to ESP32 TX (Serial0 TX)

// =========================================================================
// LVGL Task Timing & Priorities
// =========================================================================
#define LVGL_TICK_PERIOD_MS       5
#define LVGL_TASK_MAX_DELAY_MS    500
#define LVGL_TASK_MIN_DELAY_MS    5
#define LVGL_TASK_STACK_SIZE      (10 * 1024)
#define LVGL_TASK_PRIORITY        3

#endif // USER_CONFIG_H
