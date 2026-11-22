# RTK Rover & Smart Pole -- System Hardware Wiring Schematics

**Document Version:** `v1.3.0`  
**Date:** November 22, 2026  
**Firmware Baseline:** `v0.9.0` (Single-UART `UM982 COM1` + Direct-JST `1S` LiPo + Collapsed `Switched +5.00V` Power Module + `DFRobot SEN0696` 10-DOF Beam IMU Isolated on `I2C1 GPIO 47/48`)  

This document contains the complete high-level wiring diagram, Switched `+5.00V` Power Module internal schematic, RF antenna diagrams, power budget, and pin-by-pin interconnect tables for:
1. **Current Rover Prototype (`v0.9.0`):** **Waveshare `ESP32-S3-Touch-LCD-3.5B`** (`1S` LiPo plugged directly into onboard **`BAT JST`** using `AXP2101` `500 mA` USB-C charging) + **Collapsed Switched `+5.00V` Power Module (`NDP6020P` P-FET + `2N2222` NPN + `2 x 10 kOhm` + `MT3608` Boost + `470 uF / 0.1 uF` Filter)** feeding **separate `+5.00V / GND` pairs** to **Holybro `UM982`** and **Ebyte `E220-900T30S`** + **DFRobot Gravity `SEN0696`** 10-DOF IMU (`0x4A`, isolated **`I2C1` on `GPIO 47 SDA` / `GPIO 48 SCL`** with automatic `I2C0` fallback on un-rewired bench units and optional Tier-B `MMC5983MA` `0x30` on `I2C0`). Connector wiring guide: [`RTK_Rover_3.5B_Header_Wiring_Guide-rev5.png`](RTK_Rover_3.5B_Header_Wiring_Guide-rev5.png) & [`RTK_Rover_3.5B_Header_Pin_Mapping-rev5.csv`](RTK_Rover_3.5B_Header_Pin_Mapping-rev5.csv).
2. **Legacy / Experimental Smart Pole Reference (`3.49-V2`, Retired):** Archived reference mapping (**`Waveshare 3.49-V2`** migration retired in favor of locked `3.5B` hardware) retained in Section 2 for `J2/J3/J6` electrical cross-reference.

### Schematic & Pinout Reference Index

| Artifact Role | Files | Scope / Source |
| :--- | :--- | :--- |
| **System Wiring Diagrams (`v0.9.0`)** | [`wiring_diagram_3_5B_current.png`](wiring_diagram_3_5B_current.png) ([`.svg`](wiring_diagram_3_5B_current.svg)) & [`wiring_diagram_power_module.png`](wiring_diagram_power_module.png) ([`.svg`](wiring_diagram_power_module.svg)) | Project-wide harness & Switched `+5.00V` Power Module (`Figure 1A / 1B`) |
| **32-Pin Connector Wiring Guide (`rev5`)** | [`RTK_Rover_3.5B_Header_Wiring_Guide-rev5.png`](RTK_Rover_3.5B_Header_Wiring_Guide-rev5.png) & [`RTK_Rover_3.5B_Header_Pin_Mapping-rev5.csv`](RTK_Rover_3.5B_Header_Pin_Mapping-rev5.csv) | **RTK Rover custom project assignments** on the `3.5B` 2x16 header |
| **Stock MCU Reference (Vendor)** | [`Waveshare_3.5B_Stock_GPIO_Allocation_Table.png`](../images/Waveshare_3.5B_Stock_GPIO_Allocation_Table.png), [`Waveshare_3.5B_Stock_Header_Pinout.png`](../images/Waveshare_3.5B_Stock_Header_Pinout.png), [`Waveshare_3.5B_Stock_Board_Callouts.png`](../images/Waveshare_3.5B_Stock_Board_Callouts.png), [`V1.0`](ESP32-S3-Touch-LCD-3.5B_V1.0-schematic.pdf) / [`V2.0`](ESP32-S3-Touch-LCD-3.5B_V2.0-schematic.pdf) PDFs | Unmodified Waveshare `3.5B` internal peripheral & hardware reference |
| **Archived `3.49-V2` Mapping (Retired)** | [`Waveshare ESP32-S3-Touch-LCD-3.49-v2_gpio_pin_mapping-MFS&RSP.csv`](Waveshare%20ESP32-S3-Touch-LCD-3.49-v2_gpio_pin_mapping-MFS%26RSP.csv) | Historical `MFS&RSP` `J2/J3/J6` header allocation (`Section 2`) |

---

## 1. Current Rover Prototype (`Waveshare ESP32-S3-Touch-LCD-3.5B`, Firmware `v0.9.0`)

### Figure 1A: High-Level System Wiring Diagram

![RTK Rover v0.9.0 High-Level Wiring Diagram (Waveshare ESP32-S3-Touch-LCD-3.5B)](wiring_diagram_3_5B_current.png)
*Figure 1A: High-Level Hardware Wiring Diagram for the Current `Waveshare ESP32-S3-Touch-LCD-3.5B` Rover (`v0.9.0`) showing direct `1S` LiPo connection to `BAT JST`, the collapsed Switched `+5.00V` Power Module with separate `+5.00V / GND` harnesses to `UM982` & `E220`, Single-UART `UM982 COM1`, and `DFRobot Gravity SEN0696` (`0x4A`) isolated on `I2C1` (`GPIO 47 SDA / GPIO 48 SCL`).*  
*(Vector SVG version: [`wiring_diagram_3_5B_current.svg`](wiring_diagram_3_5B_current.svg) | 32-Pin Connector Guide: [`RTK_Rover_3.5B_Header_Wiring_Guide-rev5.png`](RTK_Rover_3.5B_Header_Wiring_Guide-rev5.png) & [`rev5.csv`](RTK_Rover_3.5B_Header_Pin_Mapping-rev5.csv))*

### Figure 1B: Switched `+5.00V` Power Module Internal Wiring Diagram

![RTK Rover Switched +5.00V Power Module Wiring Diagram](wiring_diagram_power_module.png)
*Figure 1B: Internal Component Wiring Schematic for the Custom Switched `+5.00V` Power Module (`NDP6020P` TO-220 + `2N2222` TO-92 + `2 x 10 kOhm` + `MT3608` Boost + `470 uF` & `0.1 uF` decoupling capacitors), showing the two separate `(+5.00V, GND)` output harnesses to the Holybro `UM982` and Ebyte `E220-900T30S`.*  
*(Vector SVG version: [`wiring_diagram_power_module.svg`](wiring_diagram_power_module.svg))*

### Figure 1C: Waveshare `3.5B` 32-Pin Connector Wiring Guide (`rev5`)

![RTK Rover 3.5B 32-Pin Header Wiring Guide (rev5)](RTK_Rover_3.5B_Header_Wiring_Guide-rev5.png)
*Figure 1C: Project-specific RTK Rover connector wiring assignments overlaid on the `32-pin` expansion header (supersedes `rev4`: moves `10DOF_SDA` -> `P21 GPIO 47` and `10DOF_SCL` -> `P23 GPIO 48` on isolated `I2C1`). Tabular export: [`RTK_Rover_3.5B_Header_Pin_Mapping-rev5.csv`](RTK_Rover_3.5B_Header_Pin_Mapping-rev5.csv); unmodified vendor allocation matrix: [`Waveshare_3.5B_Stock_GPIO_Allocation_Table.png`](../images/Waveshare_3.5B_Stock_GPIO_Allocation_Table.png).*

### 1.1 Complete Pin-by-Pin Interconnect & Power Stage Table (`ESP32-S3-Touch-LCD-3.5B`)

> **CRITICAL POWER & CABLE COLOR WARNINGS:**  
> * **Direct Battery & Charging:** The `1S` Li-Ion/LiPo battery plugs directly into the Waveshare `3.5B` **`BAT JST`** socket so the onboard `AXP2101` PMIC handles `500 mA` charging over `USB-C VBUS` and live battery telemetry (`0x34`). The Power Module taps `VBAT` from Header **`P1 (BAT)`** without external battery splicing.  
> * **Never tie `MT3608 VOUT+ (+5.00V)` to `3.5B Header P2 (5V)`:** Keeping `+5.00V` isolated on the Power Module outputs prevents backfeeding `VBUS` when running on battery.  
> * **Gravity `SEN0696` vs. Qwiic Color Swap & Isolated `I2C1` Routing:** On the DFRobot `PH2.0-4P` Gravity cable, **Green (`D/T`) = `SDA`** (`P21 GPIO 47`) and **Blue (`C/R`) = `SCL`** (`P23 GPIO 48`) -- opposite to SparkFun Qwiic (**Blue = `SDA`**, **Yellow = `SCL`**). Power `SEN0696` **`+` (Red)** strictly from **`3V3` (`P32`)** and **`-` (Black)** from **`GND` (`P30`)**. External `4.7 kOhm` pull-up resistors to `3V3` on `IO47/IO48` are recommended alongside internal weak pull-ups for long `GX-12` mast cable capacitance. `sen0696_init()` probes `I2C1` (`GPIO 47/48`) first and automatically falls back to `I2C0` (`GPIO 7/8`) for un-rewired bench units, while optional Tier-B `MMC5983MA` (`0x30`) remains auto-detected on `I2C0`.

| Peripheral / Power Stage | Module Pin / Port | Direction | `3.5B` Pin / Source | Wire / Electrical Specification |
| :--- | :--- | :---: | :--- | :--- |
| **`1S` Li-Ion / LiPo Pack** | **`BAT+` / `BAT-` (`JST-PH2.0`)** | `<->` | **Waveshare `BAT JST` Socket** | **Direct Red/Black JST Lead** -- Feeds onboard `AXP2101` PMIC (`0x34`) for `500 mA` USB-C charging & live `VBAT` |
| **Power Module Inputs** | **`VBAT IN` / `GND IN` / `3V3 EN`** | `<-` | **`P1 (BAT)`, `P29 (GND)`, `P31 (3V3)`** | Dark Red (`VBAT`), Black (`GND`), Orange (`3V3` Enable) -- `0.00 uA` off-state leak when powered off |
| **Power Module -> `UM982`** | **`5V_IN` & `GND` (Output Pair 1)** | `->` | **Power Module `OUT 1` (`+5.00V / GND`)** | **Dedicated Red/Black Pair** -- Switched `+5.00 V` (`~320-350 mA` incl. dual LNA bias) + dedicated ground return |
| **Power Module -> `E220`** | **`VCC` & `GND` (Output Pair 2)** | `->` | **Power Module `OUT 2` (`+5.00V / GND`)** | **Dedicated Red/Black Pair** -- Switched `+5.00 V` (`~25-40 mA` avg, `<= 850 mA` TX peaks via `470 uF` cap) + dedicated ground return |
| **Holybro `UM982` GNSS** | **`COM1 TX` (`TXD0`)** | `->` | **`P16` (`GPIO 17` / `Serial1 RX`)** | Teal -- `115200 8N1` (`NMEA` + `#UNIHEADINGA` + `#BASEINFOA` + Cmd ACKs) |
| **Holybro `UM982` GNSS** | **`COM1 RX` (`RXD0`)** | `<-` | **`P18` (`GPIO 18` / `Serial1 TX`)** | Cyan -- `115200 8N1` (Multiplexed ASCII Setup Cmds + `CRC24Q`-Verified `RTCM3.x` Corrections) |
| **Legacy `UM982 COM2`** | `(Consolidated onto COM1)` | `--` | **(Replaced in `v0.9.0`)** | `GPIO 47/48` previously used by `Serial2 COM2` were reassigned to isolated `I2C1` in `v0.9.0` |
| **Holybro `UM982` GNSS** | **`ANT1 (MASTER)` & `ANT2 (SLAVE)`** | `<->` | **Dual L1/L2/L5 Antennas** | `50 Ohm` MMCX/SMA coax on `55.5 cm` rigid crossbar (Pole Midpoint `27.75 cm`, Height `1.80 m`) |
| **Ebyte `E220-900T30S`** | **`TXD` (Module Out)** | `->` | **`P13` (`GPIO 41` / `Serial0 RX`)** | Purple -- `115200 8N1` (Mode 0 `RTCM3` Downlink + RSSI Reply) / `9600 8N1` (Mode 3 Cfg) |
| **Ebyte `E220-900T30S`** | **`RXD` (Module In)** | `<-` | **`P15` (`GPIO 42` / `Serial0 TX`)** | Magenta -- `115200 8N1` (`48 B` Uplink Telemetry + Rate-Switch Cmds + `C0..C3` RSSI Query) |
| **Ebyte `E220-900T30S`** | **`AUX`** | `->` | **`P5` (`GPIO 21` / ISR Input)** | Amber -- Hardware Edge Interrupt (`HIGH` = Idle/Ready, `LOW` = Busy) |
| **Ebyte `E220-900T30S`** | **`M0`** | `<-` | **`P9` (`GPIO 39` / Output)** | Brown -- Mode Select Bit 0 (`0` = Mode 0 Transparent, `1` = Mode 3 Sleep/Config) |
| **Ebyte `E220-900T30S`** | **`M1`** | `<-` | **`P11` (`GPIO 40` / Output)** | Olive -- Mode Select Bit 1 (`0` = Mode 0 Transparent, `1` = Mode 3 Sleep/Config) |
| **Ebyte `E220-900T30S`** | `NRST` *(Optional)* | `<-` | `P7` (`GPIO 38` / Output) | Gray Dashed -- Optional Hardware Reset |
| **Ebyte `E220-900T30S`** | **`SMA / IPEX RF Port`** | `<->` | **900 MHz LoRa Antenna** | `50 Ohm` Coax (`CH 75 = 925.125 MHz`, `30 dBm` / `1 W`, `4.8k/9.6k/19.2k` air-rate) |
| **DFRobot `SEN0696` (`0x4A`)** | **`+` (`VCC`, Gravity Red)** | `<-` | **`P32` (`3V3` Out)** | **Gravity Red** -- Regulated `+3.3 V` (`~1 mA`; `BMI323` + `BMM350` + `BMP581` 10-DOF Beam IMU) |
| **DFRobot `SEN0696` (`0x4A`)** | **`-` (`GND`, Gravity Black)** | `<->` | **`P30` (`GND`)** | **Gravity Black** -- Common System Ground |
| **DFRobot `SEN0696` (`0x4A`)** | **`D/T` (`SDA`, Gravity Green)** | `<->` | **`P21` (`GPIO 47` / `I2C1_SDA`)** | **Gravity Green** -- `400 kHz` **Isolated `I2C1` Data** (`0x4A..0x4D`; 18-clk GPIO bus-clear + `4.7 kOhm` pull-up to `3V3`; auto `I2C0` bench fallback) |
| **DFRobot `SEN0696` (`0x4A`)** | **`C/R` (`SCL`, Gravity Blue)** | `<-` | **`P23` (`GPIO 48` / `I2C1_SCL`)** | **Gravity Blue** -- `400 kHz` **Isolated `I2C1` Clock** (`4.7 kOhm` pull-up to `3V3`; separates external cable capacitance from internal `I2C0` slaves) |
| **Internal `I2C0` + Tier-B Mag** | **`ESP_SCL` / `ESP_SDA`** | `<->` | **`P26` (`GPIO 7`) / `P28` (`GPIO 8`)** | **Shared `400 kHz` `I2C0`** -- Touch (`0x3B`), `TCA9554` (`0x20`), `AXP2101` (`0x34`), `PCF85063` (`0x51`), `QMI8658` (`0x6B`), `ES8311` (`0x18`), opt. legacy `MMC5983MA` (`0x30`) |

---

## 2. Next-Generation Smart Pole (`Waveshare ESP32-S3-Touch-LCD-3.49-V2`, 1" Aluminum Beam)

> **Unified `MFS&RSP` Pin Allocation Reference:** [`Waveshare ESP32-S3-Touch-LCD-3.49-v2_gpio_pin_mapping-MFS&RSP.csv`](Waveshare%20ESP32-S3-Touch-LCD-3.49-v2_gpio_pin_mapping-MFS%26RSP.csv)  
> The `3.49-V2` Smart Pole reuses the shared `Magnetic Field Scanner` (`MFS`) & `Rover Smart Pole` (`RSP`) `J2`/`J3`/`J6` expansion pinout so a single beam enclosure and `I2C0` bus serve both instruments.

### 2.1 Unified `MFS&RSP` Expansion Header & Power Stage Table (`ESP32-S3-Touch-LCD-3.49-V2`)

| Header Pin | `3.49-V2` GPIO / Rail | Direction | Connected Subsystem / Pin | Signal / Electrical Specification |
| :--- | :--- | :---: | :--- | :--- |
| **`J3-11`** | **`VBAT`** (`3.5V–4.2V`) | $\longrightarrow$ | **`NDP6020P` Source (Pin 3)** & **`R1` (`10 kOhm`)** | Unswitched `1S` battery rail feeding the high-side P-FET switch |
| **`J3-10` / `J6-10`** | **`3V3`** (`+3.3V` Switched) | $\longrightarrow$ | **`R2` (`10 kOhm`) $\to$ `2N2222`/`2N3904` Base** & **`10DOF` `+` (`3.3V`)** | Active-HIGH when board is ON (`+3.3 V`), `0 V` when OFF (`0.00 uA` off-state leak) |
| **`J3-9` / `J6-9`** | **`GND`** | $\longleftrightarrow$ | **Common Star Ground** | Common ground for Power Module, `UM982`, `E220`, and `10DOF` |
| **`J6-5`** | **`GPIO 4` (`BAT_ADC`)** | Internal | **Onboard `VBAT` Divider** | Hardware voltage divider for live battery voltage monitoring |
| **`J6-1`** | **`GPIO 0` (`BOOT0`)** | $\longleftarrow$ | **External Pushbutton** *(Optional)* | Optional external pushbutton in parallel with `BOOT` (`GPIO 0`) |
| **`J3-5`** | **`GPIO 43` (`UART TXD`)** | $\longrightarrow$ | **Holybro `UM982` `COM1 RX` (`RXD0`)** | `115200 8N1` — Multiplexed ASCII Commands + `CRC24Q`-Verified `RTCM3.x` Corrections |
| **`J3-6`** | **`GPIO 44` (`UART RXD`)** | $\longleftarrow$ | **Holybro `UM982` `COM1 TX` (`TXD0`)** | `115200 8N1` — `NMEA` + `#UNIHEADINGA` + `#BASEINFOA` + Command Responses |
| **`J2-2`** | **`GPIO 1` (UART/GPIO)** | $\longrightarrow$ | **Ebyte `E220` `RXD` (Module In)** | `115200 8N1` (Mode 0 Uplink Telemetry / RSSI Query) / `9600 8N1` (Mode 3 Cfg) |
| **`J6-3`** | **`GPIO 2` (UART/GPIO)** | $\longleftarrow$ | **Ebyte `E220` `TXD` (Module Out)** | `115200 8N1` (Mode 0 RTCM3 Downlink + RSSI Reply) / `9600 8N1` (Mode 3 Cfg) |
| **`J6-4`** | **`GPIO 3` (ISR Input)** | $\longleftarrow$ | **Ebyte `E220` `AUX`** | Hardware Edge Interrupt (`HIGH` = Idle/Ready, `LOW` = Busy) |
| **`J6-6`** | **`GPIO 5` (Output)** | $\longrightarrow$ | **Ebyte `E220` `M1` / `M0`** | Combined Mode Select (`0` = Mode 0 Transparent, `1` = Mode 3 Config) |
| **`J3-7`** | **`GPIO 47` (`I2C_SDA`)** | $\longleftrightarrow$ | **Outboard `10DOF` `SDA`** (`SEN0696` `0x4A`) | Shared `400 kHz` `I2C0` Data (`TCA9554` `0x20`, `QMI8658` IMU, `PCF85063` RTC, Audio) |
| **`J3-8`** | **`GPIO 48` (`I2C_SCL`)** | $\longrightarrow$ | **Outboard `10DOF` `SCL`** (`SEN0696` `0x4A`) | Shared `400 kHz` `I2C0` Clock along non-ferromagnetic 1" aluminum beam |

### 2.2 Discrete High-Side Power Switch (`NDP6020P` + `2N2222`/`2N3904`) & `MT3608` `+5.00V` Boost Schematic

```text
J3-11 (VBAT: 3.5V - 4.2V from 1S Battery)
  │
  ├──────────────────────┬────────────────────────► NDP6020P Pin 3 (SOURCE)
  │                      │
  │                    [R1: 10 kOhm] (Holds Gate at VBAT when OFF -> 0.00 uA leak)
  │                      │
  │                      ├────────────────────────► NDP6020P Pin 1 (GATE)
  │                      │
  │                    2N2222 / 2N3904 Collector (Pin 3)
  │                      │
J3-10 (3V3) ──[R2: 10 kOhm]──► 2N2222 / 2N3904 Base (Pin 2)
  │                      │
  │                    2N2222 / 2N3904 Emitter (Pin 1) ────► J3-9 (GND)
  │
  └──► Outboard Beam 10DOF 3.3V (Gravity Red Wire)

NDP6020P Pin 2 (DRAIN / Center Pin & Tab)
  │
  └──► Switched VBAT ──► [MT3608 VIN+] ===> [MT3608 VOUT+ (+5.00V)] ──┬──► Holybro UM982 5V_IN (Pair 1)
                                                                      ├──► Ebyte E220-900T30D/S VCC (Pair 2)
                                                                      └──► [470uF + 0.1uF Caps] ──► GND
```

---

## 3. System Power Budget, `MT3608` Boost Analysis & Battery Runtime Estimates

When running untethered from a single-cell `1S` Li-Ion/LiPo battery pack (`3.7 V` nominal) via the high-side switched **`MT3608`** `+5.00 V` boost stage (`VBAT` -> `NDP6020P` -> `MT3608` -> `UM982` & `E220`, with controller/display/sensors powered via the `AXP2101` PMIC), total battery current equals the sum of the reflected `5.0 V` boost load and the `3.3 V / VSYS` controller load.

### 3.1 Subsystem Current & Power Breakdown (`VBAT = 3.7 V` Nominal)

| Subsystem / Rail | Supply Source | Operating Load (`@ Rail`) | Est. `VBAT` Draw (`@ 3.7 V`) | Basis / Notes |
| :--- | :--- | :---: | :---: | :--- |
| **Holybro H-RTK `UM982`** | `MT3608` `+5.0 V` | `320–350 mA` (`1.60–1.75 W`) | **`~510–560 mA`** | Dual-antenna receiver + dual LNA bias (`~320 mA` schematic typ; `~350 mA` Holybro spec; core `180 mA` typ / `300 mA` max @ `3.3 V`) |
| **Ebyte `E220-900T30S`** | `MT3608` `+5.0 V` | `~25–40 mA` avg (`0.13–0.20 W`) | **`~40–75 mA`** | `~17 mA` RX continuous (`14–18 mA`) + `610–850 mA` peak TX bursts (`~50–160 ms` per `5 s` uplink = `1%–3.2%` duty @ `4.8k`–`19.2k`) |
| **`MT3608` Boost Subtotal** | `~85%` typ eff (`83–88%`) | **`345–390 mA` (`~375 mA` typ)** | **`~596 mA` typ** (`530–635 mA`) | $P_{\text{bat}} = P_{5\text{V}} / \eta_{\text{boost}}$; typical `1.88 W / (0.85 * 3.7 V)` |
| **ESP32-S3 + PSRAM + `SD_MMC`** | `AXP2101` `DCDC1` (`3.3 V`) | `~95–130 mA` (`Wi-Fi OFF`) | **`~95–130 mA`** | `240 MHz` dual-core Modem-sleep (`66–108 mA` datasheet) + QSPI/SD logging; **add `~100–140 mA` avg if Wi-Fi AP/STA is ON** |
| **3.5" `320x480` IPS LCD + Touch** | `VSYS` / `AXP2101` (`80%` PWM) | `~85–110 mA` (`80%` default) | **`~85–110 mA`** | Backlight (`R3 = 6.8 Ω` LED chain + driver) + `AXS15231B`; drops to `~45–55 mA` when dimmed to `40%` |
| **10DOF / IMU / RTC / Audio** | `3V3` (`AXP2101`) | `~2–5 mA` total | **`~2–5 mA`** | `SEN0696` (`BMI323` `0.79 mA` + `BMM350` `0.20 mA` + `BMP581` `1.3 uA`) / `MMC5983MA` (`< 1 mA`) + `QMI8658` / `PCF85063` |
| **Controller/Display Subtotal** | `AXP2101` + `VSYS` | **`~180–240 mA`** (`~210 mA` typ) | **`~210 mA` typ** | `Wi-Fi OFF`, `80%` backlight |
| **Total Rover System Load** | **`1S` Li-Ion/LiPo (`3.7 V`)** | — | **`~806 mA` (`~3.0 W`) typ** | Range: `~675 mA` (`2.5 W` Eco) to `~1,005 mA` (`3.7 W` Heavy) |

### 3.2 Expected Battery Runtime (`3,000 mAh` vs. `10,000 mAh`)

Practical estimates below apply an **`85%–90%` usable capacity factor** to account for under-load battery voltage sag reaching the `MT3608` / `AXP2101` dropout knee (`~3.4 V`), alongside the `100%` nominal theoretical runtime:

| Operating Profile | Est. `VBAT` Draw (`@ 3.7 V`) | **3,000 mAh (`11.1 Wh`)** *(85–90% usable)* | **10,000 mAh (`37.0 Wh`)** *(85–90% usable)* |
| :--- | :---: | :---: | :---: |
| **Eco Survey** (`40%` brightness, `Wi-Fi OFF`, `9.6k/19.2k`) | **`~675 mA`** (`2.5 W`) | **~3.8 – 4.0 hrs** *(4.4 hrs nominal)* | **~12.6 – 13.3 hrs** *(14.8 hrs nominal)* |
| **Typical Field** (`80%` brightness, `Wi-Fi OFF`, `4.8k`, SD logs ON) | **`~806 mA`** (`3.0 W`) | **~3.2 – 3.4 hrs** *(3.7 hrs nominal)* | **~10.6 – 11.2 hrs** *(12.4 hrs nominal)* |
| **Heavy Load** (`100%` brightness, **`Wi-Fi ON`**, `4.8k`, SD logs ON) | **`~1,005 mA`** (`3.7 W`) | **~2.5 – 2.7 hrs** *(3.0 hrs nominal)* | **~8.5 – 9.0 hrs** *(10.0 hrs nominal)* |

*(High internal resistance in a single `3,000 mAh` pouch cell or thin `JST MX1.25` leads can cause slightly earlier low-voltage cutoff on `E220` TX bursts, pushing practical pouch runtime toward `~2.8–3.2 hrs`; a `10,000 mAh` pack reduces per-cell stress by $>3\times$.)*

### 3.3 Battery-Only Hardware & Firmware Checklist

1. **Idle Auto-Shutdown Timer (`auto_shutdown_battery_min`):** Defaults to **`5` minutes** without touchscreen input, motion, Wi-Fi, or active loggers when running on battery (`Tile 5 AUTO-OFF` toggles `5m/15m/30m/DISABLED`; automatically inhibited by `REC ON`, `NMEA`/`RTCM` log, survey, `CAL MAG`, or walking motion, with a 30s cancel modal & clean SD logger flush). Set `"auto_shutdown_battery_min": 0` or tap `Tile 5 AUTO-OFF` -> `DISABLED` for continuous unattended logging on bench/tripod.
2. **Instantaneous `E220` TX Surges & `925 MHz` RF Noise Verification:** Each `4.8k` uplink burst pulls `~0.95–1.20 A` peak at `5.0 V` (`~1.8–2.5 A` peak from `VBAT` through the `MT3608`). Place `>= 470 uF` low-ESR electrolytic + `0.1 uF` ceramic decoupling right across `MT3608` `VIN`/`VOUT`, use heavy-gauge battery leads (avoid small `MX1.25` pigtails where practical), and verify Tile 4 `Noise` stays clean (`<= -92 dBm`) to confirm the `MT3608` switching inductor does not couple `925 MHz` ripple into the `E220` (recall `test/_052` saw `~9 dB` lower RF noise on internal battery than on external `USB` cabling).
3. **Recharge Time via Onboard `AXP2101` (`500 mA` Cap):** `axp2101_bsp.c` configures `REG 0x62 = 0x0B` (**`500 mA`** constant-current limit). Charging via USB-C takes **~7 hrs** for `3,000 mAh` and **~20+ hrs** for `10,000 mAh` (prefer an external charger for `10,000 mAh` packs).
