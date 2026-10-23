# RTK Rover

High-precision handheld & pole-mounted RTK GNSS Waypoint Guidance, Property Pin Seeker, Field Survey Logger, and Dual-Antenna Compass built on the **Waveshare ESP32-S3-Touch-LCD-3.5B**, **Holybro H-RTK Unicore UM982**, and **Ebyte E220-900T30S** LoRa transceiver.

## Bill of Materials (BOM)
- **Display & Main Controller:** Waveshare ESP32-S3-Touch-LCD-3.5B (`320x480` IPS QSPI Display, AXP2101 PMIC, QMI8658 IMU, PCF85063 RTC, ES8311 Audio, TCA9554 IO Expander, MicroSD slot)
- **GNSS Receiver:** Holybro H-RTK Unicore UM982 Dual-Antenna RTK GNSS Module
- **RTCM Radio Link:** Ebyte E220-900T30S 900MHz LoRa Module

## Hardware Pin Mapping (`ESP32-S3-Touch-LCD-3.5B`)

All UART pin labels in the hardware table are **device-relative** (`<DEV>_TXD` is an output from the external module into an ESP32 RX pin):

| Subsystem | Signal Name | ESP32-S3 GPIO | Direction (ESP32 View) | Description |
| :--- | :--- | :--- | :--- | :--- |
| **UM982 COM1** | `UM982_TXD0` | `GPIO 17` | **ESP32 RX** (`Serial1`) | UM982 COM1 TX -> ESP32 RX (`115200 8N1`) |
| **UM982 COM1** | `UM982_RXD0` | `GPIO 18` | **ESP32 TX** (`Serial1`) | ESP32 TX -> UM982 COM1 RX (`115200 8N1`) |
| **UM982 COM2** | `UM982_TXD1` | `GPIO 47` | **ESP32 RX** (`Serial2`) | UM982 COM2 TX -> ESP32 RX (`115200 8N1`) |
| **UM982 COM2** | `UM982_RXD1` | `GPIO 48` | **ESP32 TX** (`Serial2`) | ESP32 TX -> UM982 COM2 RX (`115200 8N1` RTCM3 Forward) |
| **E220 LoRa** | `E220_TXD` | `GPIO 41` | **ESP32 RX** (`Serial0`) | E220 TXD -> ESP32 RX (`115200` Transparent / `9600` Config) |
| **E220 LoRa** | `E220_RXD` | `GPIO 42` | **ESP32 TX** (`Serial0`) | ESP32 TX -> E220 RXD |
| **E220 Control** | `E220_AUX` | `GPIO 21` | **ESP32 Input** | Module Busy (`LOW`) / Ready (`HIGH`) status |
| **E220 Control** | `E220_RST` | `GPIO 38` | **ESP32 Output** | Hardware Reset |
| **E220 Control** | `E220_M0` | `GPIO 39` | **ESP32 Output** | Mode 0 (`LOW` = Transparent, `HIGH` = Config) |
| **E220 Control** | `E220_M1` | `GPIO 40` | **ESP32 Output** | Mode 1 (`LOW` = Transparent, `HIGH` = Config) |
| **Internal I2C0** | `ESP_SCL` / `SDA` | `GPIO 7` / `8` | **I2C Master** | Shared by Touch (`0x3B`), TCA9554 (`0x20`), AXP2101 (`0x34`), RTC (`0x51`), IMU (`0x6B`), ES8311 (`0x18`) |
| **SD Card SPI** | `MISO/MOSI/SCLK` | `GPIO 9/10/11` | **SPI Master** | Chip Select driven via TCA9554 `EXIO3` |
| **BOOT Button** | `BOOT_SWITCH` | `GPIO 0` | **ESP32 Input** | **Short-Press (Tile 2 only):** Log Waypoint \| **Long-Press (All Screens):** Save Screenshot |

## License
Licensed under the [MIT License](LICENSE).
