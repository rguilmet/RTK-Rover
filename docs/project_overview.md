# RTK Rover - Project Overview

**Document Version:** `v1.0.3`
**Last Updated:** October 23, 2026
**Firmware Target:** `v0.1.7`
**Python Ecosystem Target:** `v1.0.1`
**License:** GNU General Public License v3.0 (`GPLv3`)

This document serves as the master source of truth for the **RTK Rover** project. It details the mechanical setup, dual-antenna lever-arm kinematics, electrical pinouts, FreeRTOS + LVGL 9 firmware architecture, operational modes, and companion Python scripts.

---

## 1. Project Identity, Versioning & Documentation Rules
* **CRITICAL RULE:** All projects shall have a main Software version identifier as `v[Major].[Minor].[Patch]` and follow industry-standard SemVer rules and practices for bumping the version at EVERY file iteration.
* **Why:** We do not reuse version numbers for each iteration. This allows us to distinctly identify in the AI context the files we are working on. The version string is defined as `ROVER_FIRMWARE_VERSION` in `user_config.h`.
* **Python Scripting Standard:** All Python scripts in `scripts/` maintain an independent `__version__` string and print `=== {__filename__} {__version__} ===` upon execution.

---

## 2. Bill of Materials (BOM) & Mechanical Architecture
1. **Display & Main Processing Unit:** **Waveshare ESP32-S3-Touch-LCD-3.5B**
   * `320x480` Portrait IPS QSPI Display (`AXS15231B` controller + Capacitive Touch `0x3B`).
   * Integrated `AXP2101` PMIC (`0x34`), `TCA9554` IO Expander (`0x20`), `PCF85063` RTC (`0x51`), `QMI8658` 6-axis IMU (`0x6B`), `ES8311` Audio Codec (`0x18`), and MicroSD slot.
   * Housed in a separate handheld enclosure that can be held flat in the hand, tilted at a viewing angle, or clipped vertically to the side of the survey pole.
2. **RTK GNSS Receiver:** **Holybro H-RTK Unicore UM982** (Dual-Antenna Multi-Band RTK GNSS)
   * Mounted on the survey pole T-bar with Master (Position) and Slave (Heading) antennas separated by `0.555 m` (`55.5 cm`, configurable in `settings.json`).
   * The survey pole centerline sits at the **exact midpoint** (`27.75 cm`) between the Master and Slave antennas, at height `antenna_height_m` (default `1.800 m`) above the ground tip.
3. **RTCM Correction Radio:** **Ebyte E220-900T30S** (900MHz LoRa UART Transceiver)
   * Receives RTCM3 differential corrections streamed from the RTK Base Station in transparent mode at `57600` baud (forwarded by the ESP32 to UM982 COM2 at `115200` baud).
   * **Radio Configuration (`v0.1.7`, configurable in `settings.json`):**
     * **RF Channel (`e220_channel`):** `55` (`0x37` $\rightarrow$ $850.125 + 55 = 905.125\text{ MHz}$, selected to avoid conflict with the EcoWitt Weather Station).
     * **UART Rate (`e220_uart_baud`):** `57600` bps, `8N1` (`REG0[7:5] = 110`; paces UART input so 1,400-byte bursts do not overrun the E220's 1,000-byte FIFO).
     * **Air Data Rate (`e220_air_rate_kbps`):** `38.4` kbps (`REG0 = 0xC6`; supports `2.4, 4.8, 9.6, 19.2, 38.4, 62.5` in `settings.json`).
     * **Packet Size (`e220_subpacket_bytes`):** `128 bytes` (`REG1[7:6] = 01`; supports `200, 128, 64, 32`).
     * **Transmit Power (`e220_tx_power_dbm`):** Level 3 (`21 dBm`, `REG1 = 0x43`; supports `30, 27, 24, 21`).
     * **Transmission Mode & LBT (`e220_lbt_enable`):** Transparent Mode, LBT Disabled (`REG3 = 0x00`).

---

## 3. Dual-Antenna Lever-Arm Kinematics & Display Decoupling

### A. Master-to-Pole-Center & Ground Tip Projection (`nav_engine`)
The UM982 reports the WGS84 coordinates (`GPGGA`) of the **Master Antenna** phase center, and outputs the true azimuth ($\psi_{\text{baseline}}$) and pitch ($\theta_{\text{baseline}}$) vector from the Master Antenna to the Slave Antenna (`UNIHEADINGA`).
Because the pole centerline is located at the midpoint between the two antennas ($L = \frac{\text{antenna\_spacing\_m}}{2} = 0.2775\text{ m}$), `nav_compute_pole_tip_position()` projects the Master coordinates along the 3D baseline vector and subtracts `antenna_height_m`:
$$\Delta N = \frac{B}{2}\cos(\theta_{\text{baseline}})\cos(\psi_{\text{baseline}}), \quad \Delta E = \frac{B}{2}\cos(\theta_{\text{baseline}})\sin(\psi_{\text{baseline}}), \quad \Delta U = \frac{B}{2}\sin(\theta_{\text{baseline}}) - H_{\text{pole}}$$
This prevents the displayed or logged ground-tip coordinates from orbiting in a $27.75\text{ cm}$ radius circle when the pole is rotated in place.

### B. Display Orientation Independence (`imu_bsp`)
* **True Heading Source:** Driven by the pole-mounted UM982 dual antennas (`UNIHEADINGA` / `GPHDT`), so rotating or tilting the handheld Waveshare display does not corrupt the navigation compass arrow.
* **Gravity Vector Pose Detection & Bracket Tare:** The onboard `QMI8658` IMU continuously tracks the 3D gravity vector of the Waveshare enclosure to classify its pose (`Held Flat`, `Viewing Angle`, or `Clipped Vertically to Pole`) and provides a **"Tare Pole Bracket Angle"** function on Tile 4 so the screen acts as a 2D plumb bubble level once clipped to the pole.

---

## 4. Electrical Pinouts & Hardware Mapping (`ESP32-S3-Touch-LCD-3.5B`)

> **CRITICAL PIN NAMING CONVENTION:**
> All peripheral pin labels in `Waveshare-ESP32-S3-Touch-LCD-3.5B-GPIO_Pin_Assignment-rev3.csv` are **device-relative**:
> * `<DEV>_TXD` is the **Transmit Output from the external module** $\rightarrow$ wired to the **ESP32 UART RX Input**.
> * `<DEV>_RXD` is the **Receive Input to the external module** $\leftarrow$ wired from the **ESP32 UART TX Output**.

| Subsystem | Signal Label | ESP32-S3 GPIO | ESP32 Direction | Function / Bus Assignment |
| :--- | :--- | :--- | :--- | :--- |
| **UM982 COM1** | `UM982_TXD0` | `GPIO 17` | **ESP32 RX** (`Serial1`) | UM982 COM1 TX $\rightarrow$ ESP32 RX (`115200 8N1`, Config/NMEA/Heading) |
| **UM982 COM1** | `UM982_RXD0` | `GPIO 18` | **ESP32 TX** (`Serial1`) | ESP32 TX $\rightarrow$ UM982 COM1 RX (`115200 8N1`) |
| **UM982 COM2** | `UM982_TXD1` | `GPIO 47` | **ESP32 RX** (`Serial2`) | UM982 COM2 TX $\rightarrow$ ESP32 RX (`115200 8N1`) |
| **UM982 COM2** | `UM982_RXD1` | `GPIO 48` | **ESP32 TX** (`Serial2`) | ESP32 TX $\rightarrow$ UM982 COM2 RX (`115200 8N1`, RTCM3 Stream Forward) |
| **UM982 COM3** | `USB-C` | N/A | Direct PC Link | Reserved for direct PC connection with Unicore **Uprecise** software |
| **E220 LoRa** | `E220_TXD` | `GPIO 41` | **ESP32 RX** (`Serial0`) | E220 TXD $\rightarrow$ ESP32 RX (`57600` Transparent / `9600` Config Mode) |
| **E220 LoRa** | `E220_RXD` | `GPIO 42` | **ESP32 TX** (`Serial0`) | ESP32 TX $\rightarrow$ E220 RXD |
| **E220 Control** | `E220_AUX` | `GPIO 21` | **ESP32 Input** | Busy (`LOW`) / Idle & Ready (`HIGH`) status |
| **E220 Control** | `E220_RST` | `GPIO 38` | **ESP32 Output** | Hardware Reset (`HIGH` = normal operation) |
| **E220 Control** | `E220_M0` | `GPIO 39` | **ESP32 Output** | Mode 0 (`LOW` = Transparent Mode 0, `HIGH` = Config Mode 3) |
| **E220 Control** | `E220_M1` | `GPIO 40` | **ESP32 Output** | Mode 1 (`LOW` = Transparent Mode 0, `HIGH` = Config Mode 3) |
| **Internal I2C0** | `ESP_SCL/SDA` | `GPIO 7 / 8` | **I2C Master** | Shared by Touch (`0x3B`), TCA9554 (`0x20`), AXP2101 (`0x34`), RTC (`0x51`), IMU (`0x6B`), ES8311 (`0x18`) |
| **LCD QSPI** | `CS/CLK/D0..3` | `12, 5, 1..4` | **QSPI Master** | `AXS15231B` `320x480` Portrait Display (`GPIO 6` Backlight PWM) |
| **SD Card SPI** | `MISO/MOSI/CLK`| `9, 10, 11` | **SPI Master** | Chip Select driven active-low via TCA9554 `EXIO3` |
| **BOOT Button** | `BOOT_SWITCH` | `GPIO 0` | **ESP32 Input** | **Short-Press (Tile 2 ONLY):** Log Pin \| **Long-Press (All Screens):** Screenshot |

---

## 5. FreeRTOS & LVGL 9 Firmware Architecture

### Core Separation
* **Core 0 (`task_rover_core0`, Priority 4):**
  * Polls `E220` (`Serial0`) and immediately writes incoming RTCM3 bytes to `UM982 COM2` (`Serial2`) while running a zero-allocation RTCM3 (`0xD3`) frame sniffer (`1005`, `MSM` message counters).
  * Polls `UM982 COM1` (`Serial1`) to parse `$GNGGA`, `$GNRMC`, `$GNGST`, `$GNHDT`, and `#UNIHEADINGA`.
  * Automatically synchronizes the hardware `PCF85063` RTC to GNSS UTC time on first valid `$GNRMC` fix.
  * Runs 30Hz lever-arm kinematic projection, geodesic target guidance (`nav_engine`), `QMI8658` IMU pose/bubble filter, `AXP2101` battery telemetry, and multi-epoch survey averaging.
* **Core 1 (`rover_lvgl_port_task` Priority 3, `task_display_and_audio` Priority 2):**
  * Renders the `320x480` Portrait LVGL 9 interface protected by `rover_lvgl_lock()` / `rover_lvgl_unlock()`.
  * Drives `ES8311` audio confirmation beeps and handles debounced `settings.json` persistence.

### UI Screen Layout (`320x480` Portrait)
* **Persistent Top Header Bar (`320x48`):** Always displays Local Date (`YYYY-MM-DD`), Local Time (`HH:MM:SS TZ` with `minutesOffsetToUTC`), Unit system badge (`SI` / `IMP`), Battery `%` and Voltage, Fix Quality badge (`NO FIX`, `SINGLE`, `DGPS`, `RTK FLOAT`, `RTK FIXED`), Satellite count, horizontal accuracy, and RTCM age.
* **Tile 1 (Col 0) — Pin Seeker & Guidance (Main Screen):** Live Pole-Tip `Lat`, `Lon`, and `Ground Height` (SI or Imperial), active target selector (`Prev`/`Next`), auto-scaling 2D bullseye/compass radar, Distance, Bearing, and $\Delta\text{North}/\Delta\text{East}$ offsets.
* **Tile 2 (Col 1) — Field Survey / Waypoint Logger:** Dedicated survey capture screen where **BOOT Button Short-Press** (or the large on-screen button) triggers multi-epoch (`1s`/`5s`/`10s`) RTK averaging and logs to `/survey_logged_pins.csv`. Supports **Target Match** (`<TARGET>_FND` + automatic $\Delta\text{N}/\Delta\text{E}$ offset note), **Prefix Dropdown + Auto-Increment**, and **Quick Condition Note Tags**.
* **Tile 3 (Col 2) — Waypoint CSV Browser:** Scrollable table of waypoints loaded from `/waypoints.csv`; tapping any row selects it as the active guidance target.
* **Tile 4 (Col 3) — Dual-Antenna Compass & Pole Bubble Level:** High-precision True Heading readout, baseline length/pitch verification, Display Pose indicator, and 2D Pole Plumb Bubble Level with bracket tare.
* **Tile 5 (Col 4) — Hardware Test Modes:** Live diagnostics and interactive test triggers for **Waveshare $\leftrightarrow$ UM982 COM1**, **Waveshare $\leftrightarrow$ UM982 COM2**, and **Waveshare $\leftrightarrow$ E220 LoRa** (including Mode 3 register readback and RTCM3 packet sniffer).
