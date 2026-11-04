# RTK Rover

High-precision handheld and pole-mounted RTK GNSS Waypoint Guidance, Property Pin Seeker, Field Survey Logger, and Dual-Antenna Moving-Baseline Compass built on the **Waveshare ESP32-S3-Touch-LCD-3.5B**, **Holybro H-RTK Unicore UM982**, and **Ebyte E220-900T30S** 900MHz LoRa transceiver.

**Firmware Version:** `v0.5.0`  
**License:** GNU General Public License v3.0 ([GPLv3](LICENSE))  
**User Documentation:** [Field User Manual](docs/user_manual.md)  
**Engineering Review:** [Critical Design Review (CDR)](docs/critical_design_review.md) | [Project Retrospective](docs/project_retrospective.md)

---

## 1. Overview & Key Capabilities

* **Geodesic Line Stakeout & Cadastral Guidance (Phase 4):**
  * Real-time along-track Station ($S$) and perpendicular cross-track Offset ($O$) navigation along boundary baselines ($A \rightarrow B$).
  * Interactive Tile 0 mode toggle: `[POINT GUIDANCE]` (Centimeter Bullseye) $\leftrightarrow$ `[LINE STAKEOUT]` (Deed Vector with Left/Right guidance prompts and quadrant bearings e.g. `S 62°15'12" E`).
  * One-tap baseline direction swap (`[ <-> SWAP ]`) and Geiger audio guidance tracking deed line proximity.
* **Bi-Directional LoRa Protocol & Remote Air-Rate Switching (Phase 3):**
  * 39-byte packed binary uplink telemetry (`0xAA 0x55`, CRC16-CCITT) transmitted in the half-duplex quiet window ($t = 0.10\text{ s} \dots 0.45\text{ s}$ post-epoch, $\approx 30\text{ ms}$ airtime) every 5 seconds.
  * Base Station [`e220_pacer.py`](scripts/e220_pacer.py) outputs live telemetry to console, writes atomic JSON to `/run/rtk_rover_status.json` (`tmpfs` in RAM, zero SD wear), and appends to rolling daily CSV with 30-day auto-pruning, gzip compression, and 50MB hard FIFO cap.
  * Over-the-air dynamic air-rate switching (`19.2k` $\leftrightarrow$ `9.6k` deep woods mode) triggered directly from Tile 1 `LORA` button with zero `sudo` passwords, zero service restarts, and 60-second automatic rollback failsafe.
* **Dual-Antenna Moving-Baseline RTK Kinematics:** Powered by the Holybro H-RTK Unicore UM982. Phase centers for Master (Position) and Slave (Heading) are spaced at `55.5 cm` along a rigid crossbar. Kinematic algorithms project coordinates to the exact pole centerline midpoint and down to the ground tip (`1.80 m`), completely eliminating position orbit when rotating the pole.
* **Dual-Mode Hybrid Navigation Engine (Tile 0):**
  * **Coarse Tactical Grid ($> 1.5\text{ m}$):** Dynamic Course-Up grid with antenna crossbar beam arrow, line-of-sight tracking vector, turn prompts, dynamic auto-scaling, and a 64-point on-screen breadcrumb trail.
  * **Fine Bullseye Stakeout ($\le 1.5\text{ m}$):** Seamlessly transitions into a centimeter bullseye with $1\text{ m}$, $20\text{ cm}$, and $2\text{ cm}$ rings, providing real-time North/East and Forward/Right offsets with distance-proportional audio Geiger clicks.
* **4-Row Persistent Top Status Header Bar:**
  * **Row 1:** Local calendar date, 24-hr time with timezone offset, SI/Imperial unit badge, icon-only Wi-Fi indicator, battery percentage, and voltage.
  * **Row 2 (Master Antenna ANT1):** RTK Fix badge (`RTK FIX`, `RTK FLT`, `DGPS`, `SINGLE`), Satellites used/in-view, horizontal accuracy estimate ($+/-\text{cm}$), RTCM differential age, and `"LoRa: "` signal strength with multi-tier color coding.
  * **Row 3 (Slave Antenna ANT2):** Heading Fix badge (`HDG FIX`, `HDG FLT`), Slave satellites, measured baseline length ($55.5\text{ cm}$), baseline pitch ($^\circ$), and true heading ($^\circ$).
  * **Row 4:** Streamlined ground-tip Latitude, Longitude, and Ellipsoidal Height in 14pt font.
* **Field Survey Logger & Consolidated Quick Settings (Tile 1):**
  * Integrated `<DERIVED>` prefix mode and clean prefixes (`PIN`, `PIPE`, `MONUMENT`, `STAKE`, `CORNER`, `FENCE`, `TREE`, `UTILITY`, `WP`).
  * Universal collision-free `_Lxxx` numbering scanning `/survey_logged_pins.csv` with zero UI loop disk I/O latency.
  * Multi-epoch averaging (`1s`, `5s`, `10s`, `30s`), visual progress bar, large `[LOG PIN NOW]` button, and circular marquee ID label.
  * **Consolidated $3\times 4$ Quick Settings Grid (12 Buttons):**
    * Row 1: `VOL: xx%` / `VOL: MUTE`, `AUD: GEIGER / BEEPS / MUTE`, `BRI: xx%`
    * Row 2: `NMEA: ON/OFF`, `ROUTE: ON/OFF`, `RTCM: ON/OFF`
    * Row 3: `TELEM: ON/OFF`, `IMU: ON/OFF`, `TOUCH: ON/OFF`
    * Row 4: `WIFI: ON/OFF`, `UNITS: IMP/SI`, `LORA: 19.2k`
  * Centered Wi-Fi status banner below grid (`Wi-Fi: Inactive (Tap to start)` / `Portal: http://...`).
* **Multi-File Waypoint Browser & Selector (Tile 2):**
  * Multi-file CSV selector scanning `/waypoints.csv` and all `/waypoints_*.csv` on MicroSD card or internal flash.
  * Dynamic circular marquee autoscroll for long waypoint labels and filenames.
  * Remembers and restores the active waypoint file and selected target index across power cycles and reboots.
* **2D Pole Plumb Bubble Level & Course-Up Compass (Tile 3):**
  * Enlarged 224px diameter spirit bubble level with dynamic Course-Up 8-point compass ring (`N`, `NE`, `E`, `SE`, `S`, `SW`, `W`, `NW`) keeping heading oriented at 12 o'clock.
  * Dedicated **Tare** (`TARE` / `RST`) button to zero the level when the display is clipped to an angled pole bracket.
  * Numerical True Heading in prominent 24pt font, baseline pitch, and live pole tilt angles.
* **Dedicated Hardware Diagnostics & Live Observers (Tile 4):**
  * Pinned top deck with firmware version header and calibrated linear LoRa RF survey card (`[-110 dBm .. -25 dBm]`).
  * Expanded $+57\text{ px}$ vertical scrollable viewport with full multi-line auto-wrapping.
  * Live observers for Storage, ANT1 Master, ANT2 Slave (with live coordinates), dedicated Base Station telemetry (ID, RTCM age, WGS84 coordinates, ECEF, RTCM counters), 6-DOF IMU (Pitch, Roll, 3-axis Accel g, 3-axis Gyro dps), UM982 COM1/COM2 metrics, and E220 LoRa hardware pin self-test (`M0:OK M1:OK`).
* **ES8311 Proximity Audio Synthesis Engine:**
  * Distance-proportional Geiger click frequency synthesis for hands-free pin recovery without taking eyes off the ground.
  * Configurable audio navigation modes (`GEIGER`, `BEEPS`, `MUTE`) while strictly preserving button feedback, screenshot chimes, and survey pin logging sounds.
* **Dedicated Hardware Screenshot Engine:**
  * Long-press physical BOOT button (GPIO 0) for $\ge 1.5\text{ s}$ to capture a full 24-bit uncompressed BMP screenshot directly to `/screenshots/screenshot_YYYYMMDD_HHMMSS.bmp`.
* **Wi-Fi Web File Portal:**
  * Integrated HTML5 web server (`http://<rover-ip>/`) for wireless field maintenance from any phone, tablet, or laptop. Browse, download, and delete survey logs (`/survey_logged_pins.csv`), route tracks (`/routes/*.csv`), raw NMEA logs (`/nmea/*.log`), and screenshots.

---

## 2. Visual Interface & Screenshot Gallery

The user interface is optimized for high-contrast sunlight readability on the Waveshare 3.5" $320 \times 480$ IPS portrait display.

### Persistent Top Header Bar
![Header: 4-Row Persistent Status Header](docs/images/screenshot_header_bar.png)
*Figure 1: Persistent 4-row header displaying system time, battery, ANT1 RTK fix, ANT2 heading fix, and ground-tip coordinates.*

### Tile 0: Pin Seeker & Tactical Guidance
| Tactical Course-Up ($> 1.5\text{ m}$) | Bullseye Centimeter Stakeout ($\le 1.5\text{ m}$) |
| :---: | :---: |
| ![Tile 0: Tactical Course-Up Navigation](docs/images/screenshot_tile0_guidance.png) | ![Tile 0: Bullseye Centimeter Stakeout](docs/images/screenshot_tile0_bullseye.png) |
| *Rotating antenna crossbar, LOS vector, and breadcrumb trail.* | *Centimeter concentric rings ($1\text{ m}$, $20\text{ cm}$, $2\text{ cm}$) with lock audio cues.* |

### Tile 1: Field Survey Logger & Quick Settings
![Tile 1: Field Survey Logger and Quick Settings](docs/images/screenshot_tile1_survey_settings.png)
*Figure 2: Row 1 Prefix & Avg duration, Row 2 Note Tag, Row 3 ID with marquee, Row 4 progress bar, Row 5 Log Pin Now, Row 6 Logged info, and Quick Settings deck.*

### Tile 2: Multi-File Waypoint Browser & Selector
![Tile 2: Multi-File Waypoint Browser](docs/images/screenshot_tile2_waypoint_browser.png)
*Figure 3: Multi-file selector bar with circular marquee, waypoint table with tap-to-select target guidance.*

### Tile 3: Dual-Antenna Compass & Plumb Bubble Level
![Tile 3: Dual-Antenna Compass and Bubble Level](docs/images/screenshot_tile3_compass_level.png)
*Figure 4: 24pt True Heading readout, 224px spirit bubble level with Course-Up 8-point compass ring, compact Tare button, and pole tilt metrics.*

### Tile 4: Hardware Diagnostics & Live Observers
![Tile 4: Hardware Diagnostics](docs/images/screenshot_tile4_diagnostics.png)
*Figure 5: Pinned firmware header, dual-row logging buttons, MicroSD storage status, ANT1, ANT2, dedicated Base Station card, 6-DOF IMU, and COM/LoRa metrics.*

---

## 3. Bill of Materials (BOM)

| Component | Model | Description |
| :--- | :--- | :--- |
| **Display & MCU** | **Waveshare ESP32-S3-Touch-LCD-3.5B** | 3.5" $320 \times 480$ IPS portrait capacitive touch LCD (`AXS15231B`), ESP32-S3 (16MB Flash, 8MB Octal PSRAM), `AXP2101` PMIC, `QMI8658` 6-axis IMU, `PCF85063` RTC, `ES8311` audio codec, `TCA9554` IO expander, MicroSD slot. |
| **GNSS Module** | **Holybro H-RTK Unicore UM982** | Dual-antenna multi-band RTK GNSS receiver (GPS L1/L2/L5, GLONASS G1/G2, Galileo E1/E5a/E5b, BeiDou B1I/B2I/B3I). |
| **Antennas** | **Dual Helix / Patch GNSS Antennas** | Separated by $55.5\text{ cm}$ along a lightweight rigid T-bar crossbar. |
| **RTCM Radio** | **Ebyte E220-900T30S** | 900MHz LoRa UART transceiver module (30dBm / 1W max power). |

---

## 4. Hardware Pin Mapping (`ESP32-S3-Touch-LCD-3.5B`)

All peripheral UART pin labels are **device-relative** (`<DEV>_TXD` is an output from the external module into an ESP32 RX pin):

| Subsystem | Signal Name | ESP32-S3 GPIO | Direction (ESP32 View) | Description |
| :--- | :--- | :--- | :--- | :--- |
| **UM982 COM1** | `UM982_TXD0` | `GPIO 17` | **ESP32 RX** (`Serial1`) | UM982 COM1 TX $\rightarrow$ ESP32 RX (`115200 8N1`, NMEA & Heading stream) |
| **UM982 COM1** | `UM982_RXD0` | `GPIO 18` | **ESP32 TX** (`Serial1`) | ESP32 TX $\rightarrow$ UM982 COM1 RX (`115200 8N1`, Configuration commands) |
| **UM982 COM2** | `UM982_TXD1` | `GPIO 47` | **ESP32 RX** (`Serial2`) | UM982 COM2 TX $\rightarrow$ ESP32 RX (`115200 8N1`, Response telemetry) |
| **UM982 COM2** | `UM982_RXD1` | `GPIO 48` | **ESP32 TX** (`Serial2`) | ESP32 TX $\rightarrow$ UM982 COM2 RX (`115200 8N1`, RTCM3 correction stream) |
| **E220 LoRa** | `E220_TXD` | `GPIO 41` | **ESP32 RX** (`Serial0`) | E220 TXD $\rightarrow$ ESP32 RX (`57600 8N1` Transparent / `9600` Config) |
| **E220 LoRa** | `E220_RXD` | `GPIO 42` | **ESP32 TX** (`Serial0`) | ESP32 TX $\rightarrow$ E220 RXD |
| **E220 Control** | `E220_AUX` | `GPIO 21` | **ESP32 Input** | Module Busy (`LOW`) / Idle & Ready (`HIGH`) status |
| **E220 Control** | `E220_RST` | `GPIO 38` | **ESP32 Output** | Hardware Reset (`HIGH` = normal operation) |
| **E220 Control** | `E220_M0` | `GPIO 39` | **ESP32 Output** | Mode 0 (`LOW` = Transparent Mode, `HIGH` = Config Mode) |
| **E220 Control** | `E220_M1` | `GPIO 40` | **ESP32 Output** | Mode 1 (`LOW` = Transparent Mode, `HIGH` = Config Mode) |
| **Internal I2C0** | `ESP_SCL` / `SDA` | `GPIO 7` / `8` | **I2C Master** | Shared by Touch (`0x3B`), TCA9554 (`0x20`), AXP2101 (`0x34`), RTC (`0x51`), IMU (`0x6B`), ES8311 (`0x18`) at `400 kHz` |
| **LCD QSPI** | `CS/CLK/D0..3` | `12, 5, 1..4` | **QSPI Master** | `AXS15231B` $320 \times 480$ Portrait Display (`GPIO 6` Backlight PWM) |
| **SD Card** | `D0/CMD/CLK` | `9, 10, 11` | **1-Bit SD_MMC / SPI** | 3-stage mount fallback (`SD_MMC` 1-bit `EXIO3=High-Z` $\rightarrow$ `EXIO3=LOW` $\rightarrow$ `SD` SPI) |
| **BOOT Button** | `BOOT_SWITCH` | `GPIO 0` | **ESP32 Input** | **Long-Press ($\ge 1.5\text{ s}$):** Capture full-screen BMP screenshot to storage |

---

## 5. FreeRTOS Dual-Core Architecture

```mermaid
flowchart TD
    subgraph Core0["Core 0: High-Throughput Real-Time Task (Priority 4)"]
        E220_RX["E220 LoRa UART0 (IO41/42 @ 57600)"] --> RTCM_FWD["Forward RTCM3 Bytes"]
        RTCM_FWD --> COM2_TX["UM982 COM2 UART2 (IO47/48 @ 115200)"]
        COM1_RX["UM982 COM1 UART1 (IO17/18 @ 115200)"] --> NMEA_PARSE["NMEA & UNIHEADINGA Parser\n(GPGGA, GPRMC, GPGST, GPGSV, GNGGAH, BASEINFOA)"]
        NMEA_PARSE --> KINEMATICS["3D Lever-Arm & Ground Tip Kinematics\n(Midpoint 27.75cm, Height 1.80m)"]
        KINEMATICS --> NAV_ENGINE["Geodesic Nav Engine\n(Range, Bearing, Offsets)"]
        NAV_ENGINE --> QUEUE["roverStateQueue (xQueueOverwrite)"]
    end

    subgraph Core1["Core 1: UI Rendering & Peripheral Task (Priority 2 & 3)"]
        QUEUE --> UI_UPDATE["update_rover_ui()"]
        I2C_POLL["Shared I2C0 Bus Driver (400 kHz)\n• Touch AXS15231B (30 Hz)\n• IMU QMI8658 12-byte burst (10 Hz)\n• RTC PCF85063 (1 Hz)\n• PMIC AXP2101 (0.5 Hz)"]
        UI_UPDATE --> LVGL["LVGL 9 Display Refresh (QSPI @ 40 MHz)"]
        AUDIO_QUEUE["beepQueue (FreeRTOS Queue)"] --> CODEC["ES8311 Audio Synthesizer"]
        SETTINGS["Debounced Settings Persistence"] --> STORAGE["MicroSD Card / FFat Mirror"]
        SCREENSHOT["Screenshot Engine (GPIO 0 Long-Press)"] --> BMP_OUT["BMP Generator -> /screenshots/"]
    end
```

---

## 6. Configuration (`settings.json`)

Settings are loaded from `/settings.json` on the MicroSD card (or internal `FFat` fallback) and automatically mirrored across both storage media:

```json
{
  "minutesOffsetToUTC": -300,
  "timezone_label": "EST",
  "use_imperial_units": false,
  "antenna_height_m": 1.800,
  "antenna_spacing_m": 0.555,
  "baseline_mount_offset_deg": 0.0,
  "apply_lever_arm": true,
  "pole_tilt_tare_pitch_deg": 0.0,
  "pole_tilt_tare_roll_deg": 0.0,
  "e220_uart_baud": 57600,
  "e220_channel": 55,
  "e220_air_rate_kbps": 38.4,
  "e220_tx_power_dbm": 21,
  "e220_subpacket_bytes": 128,
  "e220_lbt_enable": false,
  "survey_avg_seconds": 5,
  "default_label_prefix": "PIN",
  "active_waypoint_file": "/waypoints.csv",
  "active_waypoint_index": 0,
  "brightness": 80,
  "volume": 40,
  "is_muted": false,
  "wifi_ssid": "YourWiFiSSID",
  "wifi_password": "YourWiFiPassword",
  "enable_serial_logging": true,
  "enable_imu_logging": false,
  "enable_telemetry_logging": true,
  "enable_touch_logging": false
}
```

---

## 7. Cadastral, Waypoint & Security Utilities (scripts/)

Python command-line tools in scripts/ automate cadastral COGO network solving, waypoint generation, hardware pacing, and security verification:

* **`cogo_solver.py` (v1.1.0):** Generic schema-driven Cadastral Coordinate Geometry (COGO) & Traverse Engine. Solves metes-and-bounds deed descriptions, circular road curve geometry (PC, PT, center of curvature, arc midpoint), intermediate stationing, and ground-truth RTK control ties. Computes linear traverse misclosure, precision ratios (1 : N), and balances loops via Compass Rule (Bowditch Adjustment). Supports multi-parcel adjoiner boundary snapping. Exports Rover 19-column `waypoints.csv`, Google Earth KML, RFC 7946 GeoJSON, AutoCAD DXF (with dedicated layers), and ASCII closure reports. See the [Cadastral COGO Solver Guide](docs/cogo_solver_guide.md).
* **check_secrets.py (v1.1.0):** Security and privacy verification engine. Scans Git tracked files and unpushed commits for exposed credentials, cryptographic keys, and absolute user paths. Dynamically loads local personal search tokens from an unversioned .secrets.local configuration.
* **install_git_hooks.py (v1.1.0):** Installs the pre-push security verification hook into .git/hooks/pre-push.
* **kml_to_waypoints.py (v1.0.0):** Converts Google Earth or GIS .kml boundary polygons, paths, and placemarks directly into the rover's 19-column waypoints.csv format with automatic label sanitization and multi-line description handling.
* **validate_waypoints.py** (v1.2.0): Checks syntax, column count, bounds, and computes leg-by-leg distances, bearings, and closed-loop perimeter for any waypoint CSV file.
* **e220_pacer.py (v2.8.1):** Base station hardware pacer daemon for RTKBase streams with silicon-calibrated Time-on-Air and Section 5.6.2 two-regime pacing.

---

## 8. License

This project is licensed under the [GNU General Public License v3.0 (GPLv3)](LICENSE).
