# RTK Rover - Project Overview

**Document Version:** `v1.2.0`  
**Last Updated:** October 26, 2026  
**Firmware Target:** `v0.2.9`  
**Python Ecosystem Target:** `v1.0.1`  
**License:** GNU General Public License v3.0 (`GPLv3`)  

This document serves as the master source of truth for the **RTK Rover** project. It details the mechanical setup, dual-antenna lever-arm kinematics, electrical pinouts, FreeRTOS + LVGL 9 firmware architecture, operational modes, and companion Python scripts.

---

## 1. Project Identity, Versioning & Documentation Rules
* **CRITICAL RULE:** All projects shall have a main Software version identifier as `v[Major].[Minor].[Patch]` and follow industry-standard SemVer rules and practices for bumping the version at EVERY file iteration.
* **Why:** We do not reuse version numbers for each iteration. This allows us to distinctly identify in the AI context the files we are working on. The version string is defined as `ROVER_FIRMWARE_VERSION` in `user_config.h`.
* **Pure ASCII Requirement:** All user-facing strings, labels, and terminal outputs strictly use standard ASCII characters. Extended Unicode symbols (such as $\pm$, $\circ$, arrows) are avoided in UI strings to guarantee compatibility with LVGL fonts and prevent missing glyph boxes.

---

## 2. Bill of Materials (BOM) & Mechanical Architecture
1. **Display & Main Processing Unit:** **Waveshare ESP32-S3-Touch-LCD-3.5B**
   * `320x480` Portrait IPS QSPI Display (`AXS15231B` controller + Capacitive Touch `0x3B`).
   * Integrated `AXP2101` PMIC (`0x34`), `TCA9554` IO Expander (`0x20`), `PCF85063` RTC (`0x51`), `QMI8658` 6-axis IMU (`0x6B`), `ES8311` Audio Codec (`0x18`), and MicroSD slot.
   * Housed in a handheld enclosure that can be held flat in the hand, tilted at a viewing angle, or clipped vertically to the survey pole.
2. **RTK GNSS Receiver:** **Holybro H-RTK Unicore UM982** (Dual-Antenna Multi-Band RTK GNSS)
   * Mounted on the survey pole T-bar with Master (Position) and Slave (Heading) antennas separated by `0.555 m` (`55.5 cm`, configurable via `antenna_spacing_m` in `settings.json`).
   * The survey pole centerline sits at the **exact midpoint** (`27.75 cm`) between the Master and Slave antennas, at height `antenna_height_m` (default `1.800 m`) above the ground tip.
3. **RTCM Correction Radio:** **Ebyte E220-900T30S** (900MHz LoRa UART Transceiver)
   * Receives RTCM3 differential corrections streamed from the RTK Base Station in transparent mode at `57600` baud (forwarded by the ESP32 to UM982 COM2 at `115200` baud).
   * **Radio Configuration (configured via `settings.json`):**
     * **RF Channel (`e220_channel`):** `55` (`0x37` $\rightarrow$ $850.125 + 55 = 905.125\text{ MHz}$, selected to avoid conflict with local 915MHz weather sensors).
     * **UART Rate (`e220_uart_baud`):** `57600` bps, `8N1` (`REG0 = 0xC6`; paces UART input so 1,400-byte bursts do not overrun the E220's 1,000-byte FIFO).
     * **Air Data Rate (`e220_air_rate_kbps`):** `38.4` kbps (`REG0 = 0xC6`; supports `2.4, 4.8, 9.6, 19.2, 38.4, 62.5`).
     * **Packet Size (`e220_subpacket_bytes`):** `128 bytes` (`REG1 = 0x43`).
     * **Transmit Power (`e220_tx_power_dbm`):** Level 3 (`21 dBm`, `REG1 = 0x43`).
     * **Transmission Mode & LBT (`e220_lbt_enable`):** Transparent Mode, LBT Disabled (`REG3 = 0x00`).

---

## 3. Dual-Antenna Lever-Arm Kinematics & IMU Decoupling

### A. Master-to-Pole-Center & Ground Tip Projection (`nav_engine`)
The UM982 reports the WGS84 coordinates (`GPGGA`) of the **Master Antenna** phase center, and outputs the true azimuth ($\psi_{\text{baseline}}$) and pitch ($\theta_{\text{baseline}}$) vector from the Master Antenna to the Slave Antenna (`UNIHEADINGA`).
Because the pole centerline is located at the midpoint between the two antennas ($L = \frac{\text{antenna\_spacing\_m}}{2} = 0.2775\text{ m}$), `nav_compute_pole_tip_position()` projects the Master coordinates along the 3D baseline vector and subtracts `antenna_height_m`:
$$\Delta N = \frac{B}{2}\cos(\theta_{\text{baseline}})\cos(\psi_{\text{baseline}}), \quad \Delta E = \frac{B}{2}\cos(\theta_{\text{baseline}})\sin(\psi_{\text{baseline}}), \quad \Delta U = \frac{B}{2}\sin(\theta_{\text{baseline}}) - H_{\text{pole}}$$
This prevents the displayed or logged ground-tip coordinates from orbiting in a $27.75\text{ cm}$ radius circle when the pole is rotated in place.

### B. Display Orientation Independence (`imu_bsp`)
* **True Heading Source:** Driven strictly by the pole-mounted UM982 dual antennas (`UNIHEADINGA` / `GPHDT`), so rotating or tilting the handheld display never corrupts the heading compass arrow.
* **Portrait Bubble Level Realignment (`v0.2.9`):**
  * The physical QMI8658 accelerometer axes map to the portrait screen as:
    * **Screen Horizontal (Roll / $b_x$):** Driven by sensor $a_y$ (`raw_roll = atan2f(ay, -az)`). Tipping Left moves the bubble Left; tipping Right moves the bubble Right.
    * **Screen Vertical (Pitch / $b_y$):** Driven by sensor $a_x$ (`raw_pitch = atan2f(ax, sqrtf(ay * ay + az * az))`). Tipping Up moves the bubble Up; tipping Down moves the bubble Down.
    * Resting flat face-up produces $0.0^\circ$ pitch and $0.0^\circ$ roll (centered bubble).
* **Bracket Tare Calibration:** A dedicated on-screen **Tare Pole Bracket Angle** button on Tile 3 records the mounting angle offset into `current_settings.pole_tilt_tare_pitch_deg` and `current_settings.pole_tilt_tare_roll_deg`, allowing the 2D bubble level to zero out when the screen is clipped to an angled pole clamp.

---

## 4. Electrical Pinouts & Hardware Mapping (`ESP32-S3-Touch-LCD-3.5B`)

> **PIN NAMING CONVENTION:**
> All peripheral pin labels are **device-relative**:
> * `<DEV>_TXD` is the **Transmit Output from the external module** $\rightarrow$ wired to the **ESP32 UART RX Input**.
> * `<DEV>_RXD` is the **Receive Input to the external module** $\leftarrow$ wired from the **ESP32 UART TX Output**.

| Subsystem | Signal Label | ESP32-S3 GPIO | ESP32 Direction | Function / Bus Assignment |
| :--- | :--- | :--- | :--- | :--- |
| **UM982 COM1** | `UM982_TXD0` | `GPIO 17` | **ESP32 RX** (`Serial1`) | UM982 COM1 TX $\rightarrow$ ESP32 RX (`115200 8N1`, NMEA/Heading stream) |
| **UM982 COM1** | `UM982_RXD0` | `GPIO 18` | **ESP32 TX** (`Serial1`) | ESP32 TX $\rightarrow$ UM982 COM1 RX (`115200 8N1`, Config commands) |
| **UM982 COM2** | `UM982_TXD1` | `GPIO 47` | **ESP32 RX** (`Serial2`) | UM982 COM2 TX $\rightarrow$ ESP32 RX (`115200 8N1`, Responses) |
| **UM982 COM2** | `UM982_RXD1` | `GPIO 48` | **ESP32 TX** (`Serial2`) | ESP32 TX $\rightarrow$ UM982 COM2 RX (`115200 8N1`, RTCM3 Stream Forward) |
| **E220 LoRa** | `E220_TXD` | `GPIO 41` | **ESP32 RX** (`Serial0`) | E220 TXD $\rightarrow$ ESP32 RX (`57600 8N1` Transparent / `9600` Config) |
| **E220 LoRa** | `E220_RXD` | `GPIO 42` | **ESP32 TX** (`Serial0`) | ESP32 TX $\rightarrow$ E220 RXD |
| **E220 Control** | `E220_AUX` | `GPIO 21` | **ESP32 Input** | Busy (`LOW`) / Idle & Ready (`HIGH`) status |
| **E220 Control** | `E220_RST` | `GPIO 38` | **ESP32 Output** | Hardware Reset (`HIGH` = normal operation) |
| **E220 Control** | `E220_M0` | `GPIO 39` | **ESP32 Output** | Mode 0 (`LOW` = Transparent, `HIGH` = Config) |
| **E220 Control** | `E220_M1` | `GPIO 40` | **ESP32 Output** | Mode 1 (`LOW` = Transparent, `HIGH` = Config) |
| **Internal I2C0** | `ESP_SCL/SDA` | `GPIO 7 / 8` | **I2C Master** | Shared by Touch (`0x3B`), TCA9554 (`0x20`), AXP2101 (`0x34`), RTC (`0x51`), IMU (`0x6B`), ES8311 (`0x18`) at `400 kHz` on Core 1 |
| **LCD QSPI** | `CS/CLK/D0..3` | `12, 5, 1..4` | **QSPI Master** | `AXS15231B` `320x480` Portrait Display (`GPIO 6` Backlight PWM) |
| **SD Card** | `D0/CMD/CLK` | `9, 10, 11` | **1-Bit SD_MMC / SPI** | 3-stage mount fallback (`SD_MMC` 1-bit `EXIO3=High-Z` $\rightarrow$ `SD_MMC` `EXIO3=LOW` $\rightarrow$ `SD` SPI) |
| **BOOT Button** | `BOOT_SWITCH` | `GPIO 0` | **ESP32 Input** | **Long-Press ($\ge 1.5\text{ s}$):** Capture full-screen screenshot to storage |

---

## 5. FreeRTOS & LVGL 9 Firmware Architecture

### Core Separation
* **Core 0 (`task_rover_core0`, Priority 4 — Zero I2C Access):**
  * Polls `E220` (`Serial0`) and immediately writes incoming RTCM3 bytes to `UM982 COM2` (`Serial2`) while running a zero-allocation RTCM3 (`0xD3`) frame sniffer (`1005`, `MSM` message counters).
  * Polls `UM982 COM1` (`Serial1`) to parse `$GPGGA` (2 Hz), `$GPRMC` (1 Hz), `$GPGST` (2 Hz), `$GPGSV` (1 Hz), `$GNGGAH` (1 Hz slave data), and `#UNIHEADINGA` (2 Hz).
  * Runs 30Hz lever-arm kinematic projection and geodesic target guidance (`nav_engine`).
* **Core 1 (`rover_lvgl_port_task` Priority 3, `task_display_and_audio` Priority 2 — Exclusive I2C0 Owner):**
  * Exclusively owns `I2C_NUM_0` (`400 kHz`, serialized via `i2c_bus_mutex` with automatic `i2c_master_bus_reset()` recovery): polls `AXS15231B` touch (`30 Hz`), `QMI8658` IMU (`10 Hz` independent of LVGL mutex), `PCF85063` RTC (`1 Hz` cached + one-shot GNSS UTC sync), and `AXP2101` battery telemetry (`0.5 Hz`).
  * Renders the `320x480` Portrait LVGL 9 interface protected by `rover_lvgl_lock()` / `rover_lvgl_unlock()`.
  * Multi-tone audio beep synthesis via FreeRTOS `beepQueue` driving the ES8311 codec.
  * Debounced `settings.json` persistence (4s debounce after changes).

---

## 6. UI Tile Structure (`320x480` Portrait)

### Persistent Top Header Bar (`320x74`)
* **Row 1:** Local Date (`YYYY-MM-DD`), Local Time (`HH:MM:SS TZ`), Units badge (`SI` / `IMP`), Wi-Fi badge (`ON` / `OFF`), and Battery telemetry (`%`, `+` charging, `V`).
* **Row 2 (Master Antenna ANT1):** Fix badge (`RTK FIX`, `RTK FLT`, `DGPS`, `SINGLE`), Satellites (`SV: used/view`), Precision (`+/- h_acc`), and RTCM differential age (`RTCM: X.Xs`).
* **Row 3 (Slave Antenna ANT2):** Heading Fix badge (`HDG FIX`, `HDG FLT`), Slave satellites (`SV: X`), Baseline length (`BL: 55.5cm`), and True Heading (`Hdg: XXX.X`).
* **Row 4:** Ground-tip Latitude, Longitude, and Ellipsoidal Height (`0.00000000, 0.00000000, 0.000 m`).

### Tile Walkthrough
* **Tile 0 (Col 0) — Pin Seeker & Hybrid Navigation:**
  * Active target selector row with label and notes.
  * Course-Up navigation canvas with dynamic auto-scaling ($500\text{ m} \rightarrow 5\text{ m}$).
  * Antenna crossbar beam arrow, line-of-sight track vector, turn prompt, and 64-point breadcrumb ring buffer.
  * High-precision Bullseye mode automatically active at distance $\le 1.5\text{ m}$ with $1\text{ m}$, $20\text{ cm}$, and $2\text{ cm}$ rings.
  * On-screen Route recording toggle (`REC ON` / `REC OFF`).
* **Tile 1 (Col 1) — Field Survey Logger:**
  * Live pole tip coordinates and next auto-incrementing ID.
  * Prefix dropdown (`PIN`, `BENCHMARK`, `CONTROL`, `CORNER`, etc.) and quick note tags (`IP`, `MAG NAIL`, `REBAR`, `STONE`).
  * Target Match toggle to log design vs. as-found offsets.
  * Visual averaging progress bar and large on-screen `[LOG PIN NOW]` button.
* **Tile 2 (Col 2) — Waypoint CSV Browser:**
  * Scrollable table of waypoints loaded from `/waypoints.csv`.
  * Tapping any row selects it as the active guidance target.
* **Tile 3 (Col 3) — Dual-Antenna Compass & Bubble Level:**
  * High-precision True Heading readout, baseline length/pitch verification.
  * Display pose indicator (`Held Flat`, `Handheld Viewing Angle`, `Clipped Vertically to Pole`).
  * 2D spirit bubble level aligned for portrait LCD with **Tare Pole Bracket Angle** button.
* **Tile 4 (Col 4) — Settings & System Diagnostics:**
  * Preferences: Unit toggle, Wi-Fi portal toggle, NMEA log toggle, Route log toggle.
  * Stepped controls: Backlight Brightness (`25%..100%`) and Audio Volume (`MUTE..100%`).
  * Dedicated **Antenna & GNSS Diagnostics Card**: Full Master and Slave metrics including **HDOP**, satellite counts, precision estimates, and RTCM age.
  * Hardware Port Diagnostics: COM1, COM2, and E220 register query and pulse test triggers.
