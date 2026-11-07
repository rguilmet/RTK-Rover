# RTK Rover & Smart Pole — System Hardware Wiring Schematics

**Document Version:** `v1.0.0`  
**Date:** November 7, 2026  
**Firmware Baseline:** `v0.8.0` (Phase 8 Single-UART `UM982 COM1` Consolidation + Auto-Detected `MMC5983MA` 9-DOF Compass)  

This document contains the complete system wiring schematics, RF antenna diagrams, and pin-by-pin interconnect tables for:
1. **Current Generation Prototype (`v0.8.0`):** **Waveshare ESP32-S3-Touch-LCD-3.5B** + **Holybro H-RTK Unicore UM982** (Single-UART `COM1`) + **Ebyte E220-900T30D / E220-900T30S** (1W 900 MHz LoRa) + **SparkFun Qwiic `MMC5983MA`** 3-Axis 18-Bit AMR Compass (`0x30`).
2. **Next-Generation Smart Pole:** **Waveshare ESP32-S3-Touch-LCD-3.49-V2** (`J3` 22-Pin Expansion Header + 18650 Battery) + **Discrete High-Side Switch (`NDP6020P` P-FET + `2N3904` NPN) & `MT3608` 5.0V Boost Stage** + **Holybro UM982** (Single-UART `COM1`) + **Ebyte E220-900T30D/S** + **SparkFun Qwiic `MMC5983MA` Compass (`0x30`)**.

---

## 1. Current Generation Prototype (`Waveshare ESP32-S3-Touch-LCD-3.5B`, Firmware `v0.8.0`)

![Current Generation RTK Rover v0.8.0 Wiring Schematic (Waveshare ESP32-S3-Touch-LCD-3.5B)](wiring_diagram_3_5B_v0_8_0.png)
*Figure 1: Complete Hardware & RF Antenna Wiring Schematic for the Current-Generation Waveshare `ESP32-S3-Touch-LCD-3.5B` Prototype (`v0.8.0` Single-UART `UM982 COM1` + Auto-Detected `MMC5983MA` Compass).*  
*(Vector SVG version: [`wiring_diagram_3_5B_v0_8_0.svg`](wiring_diagram_3_5B_v0_8_0.svg))*

### 1.1 Complete Pin-by-Pin Interconnect Table (`ESP32-S3-Touch-LCD-3.5B`)

| Peripheral Module | Module Pin / Port | Direction | `ESP32-S3-Touch-LCD-3.5B` Pin | Signal / Protocol Specification |
| :--- | :--- | :---: | :--- | :--- |
| **Holybro UM982 GNSS** | `5V_IN` (`VCC`) | $\longleftarrow$ | `5V` / External USB 5V Rail | `+5.0 V` DC Supply (`~320 mA` incl. active LNA bias) |
| **Holybro UM982 GNSS** | `GND` | $\longleftrightarrow$ | `GND` | Common System Ground |
| **Holybro UM982 GNSS** | **`COM1 TX` (`TXD0`)** | $\longrightarrow$ | **`GPIO 17` (`Serial1 RX`)** | `115200 8N1` — `NMEA` (`GGA`, `RMC`, `GST`, `GSV`, `GGAH`) + `#UNIHEADINGA` + `#BASEINFOA` + Command Responses (`VERSIONA`, `MODE`, `CONFIG`, `LOGLIST`) |
| **Holybro UM982 GNSS** | **`COM1 RX` (`RXD0`)** | $\longleftarrow$ | **`GPIO 18` (`Serial1 TX`)** | `115200 8N1` — Multiplexed ASCII Setup/Query Commands + `CRC24Q`-Verified Binary `RTCM3.x` Corrections (`1006, 1074, 1084, 1094, 1230`) |
| **Holybro UM982 GNSS** | `COM2 TX/RX` (`TXD1/RXD1`) | — | **`GPIO 47 / 48` (Freed / NC)** | **Disconnected in `v0.8.0`** (saves 2 wires & ~6 KB internal `DRAM0`) |
| **Holybro UM982 GNSS** | **`ANT1 (MASTER)` MMCX/SMA** | $\longleftrightarrow$ | **Master GNSS Antenna** | $50\,\Omega$ Coax to L1/L2/L5 Master Antenna (Position / PVT) |
| **Holybro UM982 GNSS** | **`ANT2 (SLAVE)` MMCX/SMA** | $\longleftrightarrow$ | **Slave GNSS Antenna** | $50\,\Omega$ Coax to L1/L2/L5 Slave Antenna (Moving-Baseline Heading, $B = 55.5\text{ cm}$) |
| **Ebyte E220-900T30D/S** | `VCC` | $\longleftarrow$ | `5V` / External USB 5V Rail | `+5.0 V` DC Supply (with local `100 uF` electrolytic + `0.1 uF` ceramic decoupling) |
| **Ebyte E220-900T30D/S** | `GND` | $\longleftrightarrow$ | `GND` | Common System Ground |
| **Ebyte E220-900T30D/S** | **`TXD` (Module Out)** | $\longrightarrow$ | **`GPIO 41` (`Serial0 RX`)** | `115200 8N1` (Mode 0 RTCM3 Downlink + RSSI Reply) / `9600 8N1` (Mode 3 Config) |
| **Ebyte E220-900T30D/S** | **`RXD` (Module In)** | $\longleftarrow$ | **`GPIO 42` (`Serial0 TX`)** | `115200 8N1` (45-Byte Uplink Telemetry + Rate-Switch Req + RSSI Query) |
| **Ebyte E220-900T30D/S** | **`AUX`** | $\longrightarrow$ | **`GPIO 21` (ISR Input)** | Hardware Edge Interrupt (`HIGH` = Idle/Ready, `LOW` = RF/UART Busy) |
| **Ebyte E220-900T30D/S** | **`M0`** | $\longleftarrow$ | **`GPIO 39` (Output)** | Mode Select Bit 0 (`0` = Mode 0 Transparent, `1` = Mode 3 Sleep/Config) |
| **Ebyte E220-900T30D/S** | **`M1`** | $\longleftarrow$ | **`GPIO 40` (Output)** | Mode Select Bit 1 (`0` = Mode 0 Transparent, `1` = Mode 3 Sleep/Config) |
| **Ebyte E220-900T30D/S** | `NRST` *(Optional)* | $\longleftarrow$ | `GPIO 38` (Output) | Optional Hardware Reset |
| **Ebyte E220-900T30D/S** | **`SMA / IPEX RF Port`** | $\longleftrightarrow$ | **900 MHz LoRa Antenna** | $50\,\Omega$ Coax to 900 MHz ISM Antenna (`CH 75 = 925.125 MHz`) |
| **SparkFun `MMC5983MA`** | `3.3V` (Qwiic Red) | $\longleftarrow$ | **`3V3` (`+3.3 V` Out)** | `+3.3 V` Regulated Supply (`< 1 mA` average) |
| **SparkFun `MMC5983MA`** | `GND` (Qwiic Black) | $\longleftrightarrow$ | **`GND`** | Common System Ground |
| **SparkFun `MMC5983MA`** | **`SCL` (Qwiic Yellow)** | $\longleftarrow$ | **`GPIO 7` (`I2C0_SCL`)** | `400 kHz` Shared I2C0 Clock (with 18-clock GPIO bus-clear & `i2c_bsp_lock()`) |
| **SparkFun `MMC5983MA`** | **`SDA` (Qwiic Blue)** | $\longleftrightarrow$ | **`GPIO 8` (`I2C0_SDA`)** | `400 kHz` Shared I2C0 Data (I2C Address `0x30`, Auto-Detected via `Reg 0x2F == 0x30`) |

---

## 2. Next-Generation Smart Pole (`Waveshare ESP32-S3-Touch-LCD-3.49-V2`, 1" Aluminum Beam)

![Next-Generation Smart Pole Wiring Schematic (Waveshare ESP32-S3-Touch-LCD-3.49-V2)](wiring_diagram_3_49_V2_smart_pole.png)
*Figure 2: Complete Hardware, Discrete `NDP6020P`/`MT3608` Power Stage, and RF Antenna Wiring Schematic for the Next-Generation `Waveshare ESP32-S3-Touch-LCD-3.49-V2` Smart Pole.*  
*(Vector SVG version: [`wiring_diagram_3_49_V2_smart_pole.svg`](wiring_diagram_3_49_V2_smart_pole.svg))*

### 2.1 Complete `J3` 22-Pin Header & Power Stage Interconnect Table (`ESP32-S3-Touch-LCD-3.49-V2`)

| `J3` Header Pin | `3.49-V2` Signal | Direction | Connected Subsystem / Pin | Signal / Electrical Specification |
| :--- | :--- | :---: | :--- | :--- |
| **`J3 Pin 1`** | **`VBAT`** (`3.5V–4.2V`) | $\longrightarrow$ | **`NDP6020P` Source (Pin 3)** & **`R1` (`10 kΩ`)** | Unswitched 18650 Li-Ion battery positive rail feeding the high-side P-FET switch |
| **`J3 Pin 2`** | **`3V3`** (`+3.3V` Switched) | $\longrightarrow$ | **`R2` (`10 kΩ`) $\to$ `2N3904` Base** & **`MMC5983MA` `3.3V`** | Active-HIGH when board is ON (`+3.3 V`), `0 V` when OFF (`0.00 uA` off-state leak) |
| **`J3 Pins 3, 21, 22`** | **`GND`** | $\longleftrightarrow$ | **Common Star Ground** | Common ground for `2N3904` Emitter, `MT3608`, `UM982`, `E220`, and `MMC5983MA` |
| **`J3 Pin 4`** | **`GPIO 4` (`ADC_INT`)** | Internal | **Onboard `VBAT` Divider (`200k/100k`)** | `3.0x` hardware voltage divider for live 18650 battery voltage & low-battery SD flush |
| **`J3 Pin 5`** | **`GPIO 15` (`Serial1 RX`)** | $\longleftarrow$ | **Holybro UM982 `COM1 TX` (`TXD0`)** | `115200 8N1` — `NMEA` + `#UNIHEADINGA` + `#BASEINFOA` + Command Responses |
| **`J3 Pin 6`** | **`GPIO 16` (`Serial1 TX`)** | $\longrightarrow$ | **Holybro UM982 `COM1 RX` (`RXD0`)** | `115200 8N1` — Multiplexed ASCII Commands + `CRC24Q`-Verified `RTCM3.x` Corrections |
| **`J3 Pin 9`** | `GPIO 6` | — | **Freed / Spare GPIO** | Freed by Phase 8 Single-UART `UM982 COM1` consolidation |
| **`J3 Pin 7`** | **`GPIO 43` (`TXD0`)** | $\longrightarrow$ | **Ebyte E220 `RXD` (Module In)** | `115200 8N1` (Mode 0 Uplink Telemetry / RSSI Query) / `9600 8N1` (Mode 3 Config) |
| **`J3 Pin 8`** | **`GPIO 44` (`RXD0`)** | $\longleftarrow$ | **Ebyte E220 `TXD` (Module Out)** | `115200 8N1` (Mode 0 RTCM3 Downlink + RSSI Reply) / `9600 8N1` (Mode 3 Config) |
| **`J3 Pin 10`** | **`GPIO 48` (ISR Input)** | $\longleftarrow$ | **Ebyte E220 `AUX`** | Hardware Edge Interrupt (`HIGH` = Idle/Ready, `LOW` = Busy) |
| **`J3 Pin 15`** | **`GPIO 2` (Output)** | $\longrightarrow$ | **Ebyte E220 `M0`** | Mode Select Bit 0 (`0` = Mode 0 Transparent, `1` = Mode 3 Sleep/Config) |
| **`J3 Pin 16`** | **`GPIO 1` (Output)** | $\longrightarrow$ | **Ebyte E220 `M1`** | Mode Select Bit 1 (`0` = Mode 0 Transparent, `1` = Mode 3 Sleep/Config) |
| **`J3 Pin 17`** | **`GPIO 18` (`I2C_SCL`)** | $\longrightarrow$ | **SparkFun `MMC5983MA` `SCL` (Yellow)** | `400 kHz` I2C0 Clock to outboard beam compass (`6"`–`10"` along aluminum tube) |
| **`J3 Pin 18`** | **`GPIO 17` (`I2C_SDA`)** | $\longleftrightarrow$ | **SparkFun `MMC5983MA` `SDA` (Blue)** | `400 kHz` I2C0 Data to outboard beam compass (`0x30`, shared with `0x6B` IMU & `0x51` RTC) |
| **`J3 Pin 11`** | `GPIO 47` | $\longleftarrow$ | **SparkFun `MMC5983MA` `INT`** *(Optional)* | Optional hardware Data-Ready interrupt from `MMC5983MA` |

### 2.2 Discrete High-Side Power Switch (`NDP6020P` + `2N3904`) & `MT3608` `+5.00V` Boost Schematic

```text
J3 Pin 1 (VBAT: 3.5V - 4.2V from Rear 18650 Holder)
  │
  ├──────────────────────┬────────────────────────► NDP6020P Pin 3 (SOURCE)
  │                      │
  │                    [R1: 10 kOhm] (Holds Gate at VBAT when OFF -> 0.00 uA leak)
  │                      │
  │                      ├────────────────────────► NDP6020P Pin 1 (GATE)
  │                      │
  │                    2N3904 Collector (Pin 3)
  │                      │
J3 Pin 2 (3V3) ──[R2: 10 kOhm]──► 2N3904 Base (Pin 2)
  │                      │
  │                    2N3904 Emitter (Pin 1) ────► J3 Pin 3 (GND)
  │
  └──► SparkFun MMC5983MA 3.3V (Red Wire)

NDP6020P Pin 2 (DRAIN / Center Pin & Tab)
  │
  └──► Switched VBAT ──► [MT3608 VIN+] ===> [MT3608 VOUT+ (+5.00V)] ──┬──► Holybro UM982 5V_IN
                                                                      ├──► Ebyte E220-900T30D/S VCC (5.0V)
                                                                      └──► [100uF-220uF + 0.1uF Caps] ──► GND
```
