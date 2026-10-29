# RTK Rover - Project Overview

**Document Version:** `v1.6.0`  
**Last Updated:** October 28, 2026  
**Firmware Target:** `v0.2.38`  
**Python Ecosystem Target:** `v1.2.0`  
**License:** GNU General Public License v3.0 (`GPLv3`)  

This document serves as the master technical specification for the **RTK Rover** project. It details the mechanical setup, dual-antenna lever-arm kinematics, electrical pinouts, FreeRTOS + LVGL 9 firmware architecture, operational modes, and companion Python scripts.

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
   * Housed in a custom 3D-printed enclosure designed for handheld operation, bracket mounting at an angle, or vertical clipping to a survey pole.
2. **RTK GNSS Receiver:** **Holybro H-RTK Unicore UM982** (Dual-Antenna Multi-Band RTK GNSS)
   * Mounted on the survey pole T-bar with Master (Position) and Slave (Heading) antennas separated by `0.555 m` (`55.5 cm`, configurable via `antenna_spacing_m` in `settings.json`).
   * The survey pole centerline sits at the **exact midpoint** (`27.75 cm`) between Master and Slave antennas, at height `antenna_height_m` (default `1.800 m`) above the ground tip.
3. **RTCM Correction Radio:** **Ebyte E220-900T30S** (900MHz LoRa UART Transceiver)
   * Receives RTCM3 differential corrections streamed from the RTK Base Station in transparent mode at `19200` baud (forwarded by the ESP32 to UM982 COM2 at `115200` baud).
   * **Radio Configuration (configured via `settings.json`):**
     * **RF Channel (`e220_channel`):** `55` (`0x37` $\rightarrow$ $850.125 + 55 = 905.125\text{ MHz}$, selected to avoid conflict with local 915MHz weather sensors).
     * **UART Rate (`e220_uart_baud`):** `19200` bps, `8N1` (`REG0 = 0x86`; paces UART transmission to stay strictly below on-air LoRa throughput).
     * **Air Data Rate (`e220_air_rate_kbps`):** `38.4` kbps (`REG0 = 0x86`; supports `2.4, 4.8, 9.6, 19.2, 38.4, 62.5`).
     * **Packet Size (`e220_subpacket_bytes`):** `200 bytes` (`REG1 = 0x03`).
     * **Transmit Power (`e220_tx_power_dbm`):** Level 3 (`21 dBm`, `REG1 = 0x03`).
     * **Transmission Mode & LBT (`e220_lbt_enable`):** Transparent Mode, LBT Disabled (`REG3 = 0x00`).
   * **Technical Rationale for Radio Defaults (19200 baud, 38.4k air rate, 200B packets):**
     * **Throughput Ratio & Buffer Overflow Prevention:**
       * At `19200` baud (8N1), the UART feed rate is exactly $1,920\text{ bytes/s}$.
       * At `38.4` kbps air data rate, net LoRa channel throughput is approximately $2,400\text{ bytes/s}$.
       * Because the on-air transmission speed ($2,400\text{ B/s}$) exceeds the UART arrival rate ($1,920\text{ B/s}$), the E220 transmitter drains its internal 400-byte FIFO faster than the base station UART can fill it. This fundamentally eliminates TX buffer overflow on dense RTCM observation bursts (such as 650-byte multi-constellation MSM4 epochs).
       * Conversely, when UART operated at `38400` baud ($3,840\text{ B/s}$) with `38.4k` air rate ($2,400\text{ B/s}$), a 650-byte RTCM burst arrived in $169\text{ ms}$, but took $271\text{ ms}$ to transmit over the air, backing up over $240\text{ bytes}$ against the 400-byte FIFO buffer. Coupled with packet fragmentation and preamble delays, this caused buffer overflows, dropped bytes, and CRC24Q failures.
     * **Native 200-Byte Packet Size Justification:**
       * Standard RTCM frames (e.g. GLONASS 1084 at ~180 bytes) fit entirely within a single 200-byte subpacket without fragmentation.
       * GPS 1074 (~240 bytes) requires only 2 packets (200B + 40B). In contrast, a 128-byte subpacket size forces 1084 into 2 packets and 1074 into 3 packets. Each additional packet introduces LoRa physical layer overhead (preamble, sync word, header, CRC) and inter-packet transmission latency, increasing channel occupancy and latency without providing benefit.
     * **Link Budget & Sensitivity Boost:**
       * Operating at `38.4` kbps instead of `62.5` kbps yields a $+2.5\text{ to } +3.0\text{ dBm}$ increase in receiver sensitivity (approximately $-123\text{ dBm}$ vs $-120\text{ dBm}$ on the SX1262 LoRa core).
       * In free space, $+3\text{ dBm}$ represents a $\approx 41\%$ increase in operational range ($\sqrt{2} \times D$), significantly improving packet reception through tree canopy, foliage, and undulating terrain at conservative legal transmit power (`21 dBm` / `125 mW`).

---

## 3. Dual-Antenna Lever-Arm Kinematics & IMU Decoupling

### A. Master-to-Pole-Center & Ground Tip Projection (`nav_engine`)
The UM982 reports the WGS84 coordinates (`GPGGA`) of the **Master Antenna** phase center, and outputs the true azimuth ($\psi_{\text{baseline}}$) and pitch ($\theta_{\text{baseline}}$) vector from Master to Slave (`UNIHEADINGA`).
Because the pole centerline is located at the midpoint between the two antennas ($L = \frac{\text{antenna\_spacing\_m}}{2} = 0.2775\text{ m}$), `nav_compute_pole_tip_position()` projects the Master coordinates along the 3D baseline vector and subtracts `antenna_height_m`:
$$\Delta N = \frac{B}{2}\cos(\theta_{\text{baseline}})\cos(\psi_{\text{baseline}}), \quad \Delta E = \frac{B}{2}\cos(\theta_{\text{baseline}})\sin(\psi_{\text{baseline}}), \quad \Delta U = \frac{B}{2}\sin(\theta_{\text{baseline}}) - H_{\text{pole}}$$
This prevents the displayed or logged ground-tip coordinates from orbiting in a $27.75\text{ cm}$ radius circle when the pole is rotated in place.

### B. Slave Antenna (ANT2) Coordinates & Geodetic Baseline Projection
Slave antenna coordinates are extracted from NMEA `$GNGGAH` / `$GPGGAH` sentences. If the secondary PVT has not solved independent lat/lon but carrier-phase dual-antenna heading/baseline is valid, ANT2 position is accurately computed along the measured 3D baseline vector:
$$\Delta N = L \cdot \cos(\theta_{\text{pitch}}) \cdot \cos(\theta_{\text{hdg}}), \quad \Delta E = L \cdot \cos(\theta_{\text{pitch}}) \cdot \sin(\theta_{\text{hdg}})$$
These coordinates are displayed live under `[ANT2 SLAVE]` on Tile 4.

### C. Unicore Extended Base Station Telemetry (`BASEINFOA`)
* **Autonomous RTCM Decoding:** The UM982 receiver autonomously parses incoming RTCM 1005/1006 reference frames from the Base Station received via LoRa on COM2.
* **`BASEINFOA COM1 1` Stream:** Configured to push 1 Hz updates of `#BASEINFOA` reporting Base Station ID, health status, and ECEF coordinates $(X, Y, Z)$.
* **Closed-Form Geodetic Conversion:** The ESP32 driver converts base ECEF coordinates into WGS84 Latitude, Longitude, and Height via closed-form Bowring equations in $O(1)$ time with sub-millimeter precision.
* **Dedicated Observer Card:** Tile 4 displays Base ID, differential correction age, exact WGS84 coordinates, altitude, ECEF vectors, and RTCM 1005 / MSM message counters.

### D. Full 6-DOF IMU Telemetry & Atomic Burst Read (`imu_bsp`)
* **12-Byte Atomic I2C Read:** Reads registers `0x35..0x40` in a single transaction, capturing 3-axis Accelerometer (`ax`, `ay`, `az`) and 3-axis Gyroscope (`gx`, `gy`, `gz`) simultaneously with zero bus overhead.
* **Calibrated Units:** Accelerometer converted at 8192 LSB/g (4g range); Gyroscope converted at 64 LSB/(deg/s) (512 dps full-scale range).
* **Portrait Bubble Level Realignment:** Sensor $a_y$ drives horizontal roll ($b_x$), sensor $a_x$ drives vertical pitch ($b_y$). Resting flat face-up produces $0.0^\circ$ pitch and $0.0^\circ$ roll.
* **Bracket Tare Calibration:** An on-screen **Tare** button on Tile 3 records mounting angle offsets into `settings.json`, allowing the 2D bubble level to zero out when the screen is clipped to an angled pole clamp.

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
| **BOOT Button** | `BOOT_SWITCH` | `GPIO 0` | **ESP32 Input** | **Long-Press ($\ge 1.5\text{ s}$):** Capture 24-bit uncompressed BMP screenshot |

---

## 5. FreeRTOS & LVGL 9 Firmware Architecture

### Core Separation
* **Core 0 (`task_rover_core0`, Priority 4 — Zero I2C Access):**
  * Polls `E220` (`Serial0`) and immediately writes incoming RTCM3 bytes to `UM982 COM2` (`Serial2`) while running a zero-allocation RTCM3 (`0xD3`) frame sniffer (`1005`, `MSM` message counters).
  * Polls `UM982 COM1` (`Serial1`) to parse `$GPGGA` (2 Hz), `$GPRMC` (1 Hz), `$GPGST` (2 Hz), `$GPGSV` (1 Hz), `$GNGGAH` (1 Hz slave data), `#BASEINFOA` (1 Hz base station data), and `#UNIHEADINGA` (2 Hz).
  * Runs 30Hz lever-arm kinematic projection and geodesic target guidance (`nav_engine`).
* **Core 1 (`rover_lvgl_port_task` Priority 3, `task_display_and_audio` Priority 2 — Exclusive I2C0 Owner):**
  * Exclusively owns `I2C_NUM_0` (`400 kHz`, serialized via `i2c_bus_mutex` with automatic `i2c_master_bus_reset()` recovery): polls `AXS15231B` touch (`30 Hz`), `QMI8658` IMU (`10 Hz` independent of LVGL mutex), `PCF85063` RTC (`1 Hz` cached + one-shot GNSS UTC sync), and `AXP2101` battery telemetry (`0.5 Hz`).
  * Renders the `320x480` Portrait LVGL 9 interface protected by `rover_lvgl_lock()` / `rover_lvgl_unlock()`.
  * Multi-tone audio beep synthesis via FreeRTOS `beepQueue` driving the ES8311 codec.
  * Debounced `settings.json` persistence (4s debounce after changes).
  * Hardware screenshot generation directly to MicroSD card.

---

## 6. UI Tile Structure (`320x480` Portrait)

### Persistent Top Header Bar (`320x74`)
* **Row 1:** Local Date (`YYYY-MM-DD`), Local Time (`HH:MM:SS TZ`), Units badge (`SI` / `IMP`), Wi-Fi badge (`ON` / `OFF`), and Battery telemetry (`%`, `+` charging, `V`).
* **Row 2 (Master Antenna ANT1):** Fix badge (`RTK FIX`, `RTK FLT`, `DGPS`, `SINGLE`), Satellites (`SV: used/view`), Precision (`+/- h_acc`), and RTCM differential age (`RTCM: X.Xs`).
* **Row 3 (Slave Antenna ANT2):** Heading Fix badge (`HDG FIX`, `HDG FLT`), Slave satellites (`SV: X`), Baseline length (`BL: 55.5cm`), and True Heading (`Hdg: XXX.X`).
* **Row 4:** Ground-tip Latitude, Longitude, and Ellipsoidal Height in 14pt Montserrat font (`0.00000000, 0.00000000, 0.000 m`).

### Tile Walkthrough
* **Tile 0 (Screen 1) — Pin Seeker & Tactical Guidance:**
  * Active target selector row with label and notes.
  * Dynamic Course-Up canvas with auto-scaling ($500\text{ m} \rightarrow 5\text{ m}$): rotating crossbar with colored antenna tips (Green ANT1, Amber ANT2) and green field-of-view guidance wedge.
  * Line-of-sight tracking vector, turn prompt, distance readout, cardinal North/East offsets, and 64-point breadcrumb trail.
  * High-precision Bullseye mode automatically active at distance $\le 1.5\text{ m}$ with $1\text{ m}$, $20\text{ cm}$, and $2\text{ cm}$ rings with lock audio cues.
  * Route recording toggle (`REC ON` / `REC OFF`).
* **Tile 1 (Screen 2) — Field Survey Logger & Quick Settings:**
  * **Row 1 (`y = 8`):** Symmetrical Location ID Prefix dropdown (`<DERIVED>`, `PIN`, `PIPE`, `MONUMENT`, `STAKE`, `CORNER`, `FENCE`, `TREE`, `UTILITY`, `WP`) and Averaging duration button (`1s`, `5s`, `10s`, `30s`).
  * **Row 2 (`y = 44`):** Quick Note Tag dropdown (`Found Iron Pipe`, `Found Rebar`, `Found Stone Bound`, `Set Temp Flag`, etc.).
  * **Row 3 (`y = 82`):** Next ID row (`ID: <label>`) in Montserrat 14 `#FFCC00` with circular marquee for long compound labels.
  * **Row 4 (`y = 108`):** Multi-epoch averaging progress bar.
  * **Row 5 (`y = 124`):** Large `[LOG PIN NOW]` button with visual feedback.
  * **Row 6 (`y = 172`):** Last logged pin label and destination CSV path (`/survey_logged_pins.csv`).
  * **Row 7 (`y = 218`):** Subtle horizontal divider line separating survey controls from settings.
  * **Quick Settings (`y = 238..360`):** Row 1 (NMEA & Route log toggles), Row 2 (Brightness & Volume in 20% steps), Row 3 (Wi-Fi & Units), Row 4 (Wi-Fi status & web portal URL).
* **Tile 2 (Screen 3) — Waypoint CSV Browser & Multi-File Selector:**
  * Top navigation bar with `< Prev` and `Next >` file buttons and circular marquee for filenames.
  * Dynamic scanning of `/waypoints.csv` and all `/waypoints_*.csv` with alphabetical sorting.
  * Scrollable table of waypoints; tapping any row selects it as the active navigation target.
  * Remembers and restores the active file and active waypoint index across reboots.
* **Tile 3 (Screen 4) — Dual-Antenna Compass & Bubble Level:**
  * Numerical True Heading readout in prominent 24pt font (`Heading: 162.4`).
  * Baseline length ($55.5\text{ cm}$) and pitch ($^\circ$) verification with display pose indicator.
  * Enlarged $224\text{ px}$ diameter spirit bubble level with Course-Up 8-point compass ring (`N`, `NE`, `E`, `SE`, `S`, `SW`, `W`, `NW`) keeping heading UP (12 o'clock).
  * Compact **Tare** button (`62x28 px`) nested in the bottom-right of the circle's bounding rectangle (`TARE` Blue / `RST` Orange).
  * Centered Pole Tilt angle readout (`dX` / `dY`) below circle.
* **Tile 4 (Screen 5) — Hardware Diagnostics & Live Observers:**
  * **Pinned Top Deck:** Fixed firmware header (`RTK ROVER v0.2.38` in Montserrat 16 White) and dual-row interactive logging controls (`SERIAL`, `IMU`, `TELEM`, `TOUCH`).
  * **Scrollable Container:** Multi-line auto-wrapping across all telemetry blocks.
  * **Observers:** Storage status in Electric Cyan (`#22D3EE`), Antenna Master (ANT1) metrics, Antenna Slave (ANT2) metrics with live coordinates, dedicated Base Station card (ID, RTCM age, WGS84 coordinates, ECEF, RTCM counters), 6-DOF IMU (Pitch, Roll, 3-axis Accel, 3-axis Gyro), UM982 COM1/COM2 metrics, and E220 LoRa registers.

---

## 7. Python Survey & Ingestion Scripts (`scripts/`)

* **`cogo_network_solver.py` (v1.0.0):** Multi-deed Coordinate Geometry (COGO) solver. Merges metes-and-bounds legal descriptions across multiple adjacent deeds, ties relative traverses to surveyed RTK ground pins (`PIN_DRIVEWAY_REF` and `PIN_TREE_BASE_REF`), calculates grid-to-true convergence rotation, verifies boundary closure (sub-inch misclosure over 4,045 ft), and computes true coordinates for intermediate line intersections, corners, and elusive legal anchors (e.g. `MONUMENT_STONE_EAST`).
* **`kml_to_waypoints.py` (v1.0.0):** Ingests GIS or Google Earth KML files and extracts placemarks, linestrings, and polygon boundaries into the standardized 19-column rover CSV format.
* **`validate_waypoints.py` (v1.2.0):** Validates CSV schema compliance, bounds, and computes traverse leg distances, bearings, and total closed perimeter.
