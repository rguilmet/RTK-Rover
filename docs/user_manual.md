# RTK Rover - Field Operator's User Manual

**Document Version:** `v1.4.0`  
**Firmware Version:** `v0.4.0`  
**Last Updated:** November 4, 2026  
**Hardware Platform:** Waveshare ESP32-S3-Touch-LCD-3.5B, Holybro H-RTK Unicore UM982, Ebyte E220-900T30S  

---

## Table of Contents
1. [Introduction & System Overview](#1-introduction--system-overview)
2. [Hardware Setup & Power-On Procedure](#2-hardware-setup--power-on-procedure)
3. [Persistent 4-Row Top Status Header Bar](#3-persistent-4-row-top-status-header-bar)
4. [Tile 0: Pin Seeker & Tactical Guidance](#4-tile-0-pin-seeker--tactical-guidance)
5. [Tile 1: Field Survey Logger & Quick Settings](#5-tile-1-field-survey-logger--quick-settings)
6. [Tile 2: Multi-File Waypoint Browser & Selector](#6-tile-2-multi-file-waypoint-browser--selector)
7. [Tile 3: Dual-Antenna Compass & Plumb Bubble Level](#7-tile-3-dual-antenna-compass--plumb-bubble-level)
8. [Tile 4: Hardware Diagnostics & Live Observers](#8-tile-4-hardware-diagnostics--live-observers)
9. [Hardware Screenshot Engine](#9-hardware-screenshot-engine)
10. [Wireless Web File Portal & Field Maintenance](#10-wireless-web-file-portal--field-maintenance)
11. [Configuration Reference (`settings.json`)](#11-configuration-reference-settingsjson)
12. [Troubleshooting & Field Diagnostics FAQ](#12-troubleshooting--field-diagnostics-faq)
13. [Bi-Directional LoRa Protocol & Base Station Integration](#13-bi-directional-lora-protocol--base-station-integration)

---

## 1. Introduction & System Overview

The **RTK Rover** is a dual-antenna, high-precision centimeter GNSS rover engineered for boundary surveying, boundary recovery, property pin seeking, and ground-control point logging.

```
                           +------------------------+
                           |  GNSS Master Antenna   | (ANT1 - Position)
                           +-----------+------------+
                                       |
                           <---- 27.75 cm ---->
                                       |
    +----------------------------------+----------------------------------+
    |                     Rigid T-Bar Crossbar (55.5 cm)                   |
    +----------------------------------+----------------------------------+
                                       |
                                       | <---- 27.75 cm ---->
                                       |
                           +-----------+------------+
                           |   GNSS Slave Antenna   | (ANT2 - True Heading)
                           +------------------------+
                                       |
                                       | (Pole Centerline)
                                       |
                       +---------------+---------------+
                       |  Waveshare ESP32-S3 Display   |
                       |  (Handheld or Pole Bracket)   |
                       +---------------+---------------+
                                       |
                                       | Height: 1.800 m
                                       |
                                       v
                                [Ground Tip]
```

### Key Mechanical & Kinematic Principles
* **Rigid Moving-Baseline:** Two multi-band GNSS antennas are fixed at `55.5 cm` separation along a rigid crossbar. ANT1 (Master) computes carrier-phase RTK position; ANT2 (Slave) solves carrier-phase dual-antenna moving baseline heading.
* **Exact Midpoint Kinematics:** The survey pole centerline is positioned at the exact midpoint (`27.75 cm`) between ANT1 and ANT2. 
* **Zero Position Orbit:** The firmware projects ANT1 coordinates along the 3D baseline vector to the pole centerline and down `1.80 m` to the ground tip. Rotating the pole around its vertical axis produces zero positional orbit.

---

## 2. Hardware Setup & Power-On Procedure

1. **Mounting:** Mount the crossbar securely to the top of the survey pole. Ensure ANT1 (Master) is on the user's left and ANT2 (Slave) is on the right when viewing the display from behind the rover.
2. **Antenna Connections:** Connect the Master antenna to UM982 RF1 and the Slave antenna to UM982 RF2.
3. **LoRa Antenna:** Attach the 900 MHz LoRa antenna to the E220 transceiver SMA connector.
4. **Power-On:** Turn on battery power via the AXP2101 power switch.
5. **Boot Sequence:**
   * Waveshare boots and initializes SPI Flash, MicroSD card, and I2C peripherals.
   * QSPI Display powers up at default brightness (80%).
   * MicroSD card is mounted and `/settings.json` is mirrored to internal `FFat`.
   * FreeRTOS tasks start: Core 0 streams RTCM3 corrections from E220 to UM982 COM2 and parses UM982 COM1 NMEA/heading frames; Core 1 renders LVGL 9 at 30 Hz.
6. **Acquisition Checklist:**
   * Verify battery voltage ($> 3.7\text{ V}$) in Row 1.
   * Verify ANT1 acquires `RTK FIX` (Row 2, Green badge).
   * Verify ANT2 acquires `HDG FIX` (Row 3, Green badge).
   * Verify baseline length reads $\approx 55.5\text{ cm}$ ($\pm 1.5\text{ cm}$).

---

## 3. Persistent 4-Row Top Status Header Bar

The top 74 pixels of the display remain permanently visible across all 5 operational screens.

![Top Status Header Bar](docs/images/screenshot_header_bar.png)
*Figure 1: Persistent 4-row status header bar showing system time, battery, ANT1 fix, ANT2 fix, and ground-tip coordinates.*

### Row 1: System Time, Power & Environment
* **Date & Time:** Local calendar date (`YYYY-MM-DD`) and 24-hr time (`HH:MM:SS TZ`) computed from UTC using `minutesOffsetToUTC` and synchronized with GNSS atomic clocks.
* **Units Badge:** `SI` (meters/centimeters) or `IMP` (feet/inches). Tap to toggle on Tile 1.
* **Wi-Fi Indicator:** Compact icon-only symbol (`LV_SYMBOL_WIFI`) freeing horizontal space for date/time. Colored Electric Cyan (`#22D3EE`) in station mode, Amber (`#FFCC00`) in AP mode, and Slate Gray (`#475569`) when inactive.
* **Battery Telemetry:** Battery percentage (`0..100%`), charging indicator (`+`), and battery voltage (e.g. `4.12V`).

### Row 2: Master Antenna (ANT1 - Position) Metrics
* **Fix Status Badge:**
  * `RTK FIX` (Bright Green `#00FF88`): Centimeter carrier-phase integer fixed solution ($< 2\text{ cm}$).
  * `RTK FLT` (Yellow `#FFCC00`): Carrier-phase float solution ($5\text{ cm}..30\text{ cm}$).
  * `DGPS` (Orange `#FFAA00`): Code-differential solution ($0.5\text{ m}..1.5\text{ m}$).
  * `SINGLE` (Red `#FF4444`): Autonomous uncorrected solution ($1.5\text{ m}..5.0\text{ m}$).
* **Satellites:** Count of satellites used in fix vs. total in view (`SV: 28/34`).
* **Horizontal Precision:** Estimated horizontal precision standard deviation (`+/- 0.015m` / `+/- 0.05ft`).
* **RTCM Differential Age:** Age of incoming RTCM corrections in seconds (`RTCM: 1.0s`).
* **LoRa Signal Strength:** Explicitly labeled `"LoRa: "` prefix (e.g. `LoRa: -85dB`) with multi-tier color coding:
  * Green: Strong signal ($\ge -90\text{ dBm}$)
  * Amber/Yellow: Moderate signal ($-91\text{ dBm} \dots -105\text{ dBm}$)
  * Red: Weak fringe signal ($< -105\text{ dBm}$) or `-- dB` when link is idle.

### Row 3: Slave Antenna (ANT2 - Heading) Metrics
* **Heading Fix Badge:**
  * `HDG FIX` (Bright Green `#00FF88`): Dual-antenna carrier-phase integer ambiguity resolved.
  * `HDG FLT` (Yellow `#FFCC00`): Heading solution converging.
  * `NO HDG` (Red `#FF4444`): Dual-antenna heading unavailable.
* **Slave Satellites:** Number of satellites tracked by secondary RF path (`SV: 26`).
* **Baseline Length:** Measured 3D distance between antenna phase centers (`BL: 55.4cm`).
* **Baseline Pitch:** Crossbar pitch angle relative to the local horizon (`Pitch: +0.2 deg`).
* **True Heading:** True geographic azimuth of antenna baseline (`Hdg: 142.6 deg`).

### Row 4: Ground-Tip Geodetic Coordinates
* **Prominent 14pt Font:** Displays real-time kinematic ground-tip Latitude, Longitude, and Ellipsoidal Height:
  ```
  42.36012480, -71.09421520, 12.450 m
  ```

---

## 4. Tile 0: Pin Seeker & Tactical Guidance

Tile 0 is the primary navigational workspace for locating buried boundary pins, recovering property monuments, and navigating to design coordinates.

| Tactical Course-Up Navigation ($> 1.5\text{ m}$) | Bullseye Centimeter Stakeout ($\le 1.5\text{ m}$) |
| :---: | :---: |
| ![Tile 0: Tactical Course-Up](docs/images/screenshot_tile0_guidance.png) | ![Tile 0: Bullseye Centimeter Stakeout](docs/images/screenshot_tile0_bullseye.png) |
| *Figure 2A: Tactical Course-Up canvas with rotating antenna crossbar, LOS vector, and breadcrumb trail.* | *Figure 2B: Bullseye centimeter canvas with 1m, 20cm, and 2cm rings and audio lock cues.* |

### Tactical Course-Up Canvas ($> 1.5\text{ m}$)
* **Target Selector Bar:** Tap `< Prev` and `Next >` to cycle through targets loaded from the active waypoint CSV file. The target label and notes marquee automatically if text exceeds the container width.
* **Dynamic Course-Up Canvas:** The display rotates in real time so the rover's physical walking direction is always facing **UP** (12 o'clock).
* **Rotating Antenna Crossbar:** A graphical T-bar reflects the true physical orientation of your antenna crossbar:
  * **Left Tip (ANT1):** Green circle (`#00FF88`).
  * **Right Tip (ANT2):** Amber circle (`#FFCC00`).
  * **Green FOV Wedge:** A green triangular wedge projects forward from the crossbar showing your line-of-sight cone.
* **Line-of-Sight (LOS) Tracking Vector:** A line connects the rover's center directly to the target icon, indicating exact steer angle and heading error.
* **Turn Prompt & Distance:** Prominent live readout of distance (`Dist: 24.32 m`) and steering direction (`Steer: 14 deg L`).
* **Cardinal Offsets:** Displays distance decomposed into cardinal axes (`dN: +18.20 m | dE: -16.10 m`).
* **Breadcrumb Trail:** A 64-point circular trail records your approach path in light blue.
* **Route Recording:** Tap `REC ON` / `REC OFF` to toggle continuous track recording to `/routes/route_YYYYMMDD_HHMMSS.csv`.

### Bullseye Centimeter Stakeout Canvas ($\le 1.5\text{ m}$)
When you close within $1.5\text{ m}$ of the target, the navigation engine automatically switches to the high-precision **Bullseye Mode**:
* **Concentric Rings:**
  * **Outer White Ring:** $1.0\text{ m}$ radius ($100\text{ cm}$).
  * **Middle Orange Ring:** $20\text{ cm}$ radius ($0.20\text{ m}$).
  * **Inner Green Bullseye Ring:** $2.0\text{ cm}$ radius ($0.02\text{ m}$) — true property pin lock threshold.
* **Forward / Right Precision Offsets:** Replaces coarse steer prompts with precise surveyor offsets:
  ```
  Fwd: +0.03 m | Right: -0.01 m
  ```
* **ES8311 Proximity Audio Engine:** When approaching a buried property pin or stone bound, the onboard ES8311 codec and physical speaker provide eyes-free acoustic guidance (toggled via `AUD` button on Tile 1):
  * **Geiger Mode (`AUD: GEIGER`):** Synthesizes distance-proportional clicks that accelerate progressively as you close in on the target:
    * $D > 2.0\text{ m}$: Mute / silent.
    * $1.0\text{ m} < D \le 2.0\text{ m}$: Sparse clicks at $1\text{ Hz}$ ($800\text{ Hz}$ tone).
    * $0.3\text{ m} < D \le 1.0\text{ m}$: Medium cadence chirps at $3 - 5\text{ Hz}$ ($1200\text{ Hz}$).
    * $0.05\text{ m} < D \le 0.3\text{ m}$ ($2 - 12\text{ in}$): Rapid Geiger ticks at $8 - 15\text{ Hz}$ ($1800\text{ Hz}$).
    * $D \le 0.02\text{ m}$ ($\le 2\text{ cm}$ / Bullseye Lock): Solid confirmation chime at $2400\text{ Hz}$.
  * **Beeps Mode (`AUD: BEEPS`):** Stepped pitch chimes closing from $5.0\text{ m}$ inward.
  * **Mute Mode (`AUD: MUTE`):** Silences navigation proximity audio while preserving button taps, survey logged tones, and screenshot double-beeps.

---

## 5. Tile 1: Field Survey Logger & Quick Settings

Tile 1 provides tools for recording high-precision RTK averaged ground control points and accessing field system settings.

![Tile 1: Field Survey Logger and Quick Settings](docs/images/screenshot_tile1_survey_settings.png)
*Figure 3: Field Survey point logger and persistent Quick Settings deck.*

### Survey Logger Controls

#### Row 1: Location ID Prefix & Averaging Duration
* **Prefix Dropdown (`145x30 px`):** Selects the category or naming mode for the point:
  * `<DERIVED>` (Index 0): Derives the point name directly from the currently active navigation target label (e.g. `PIN_NW_01` $\rightarrow$ `PIN_NW_01_L001`). If no waypoints are loaded, cleanly falls back to `WP_L001`.
  * `PIN`: Formats as `PIN_L001`, `PIN_L002`, ...
  * `PIPE`: Formats as `PIPE_L001`, `PIPE_L002`, ...
  * `MONUMENT`: Formats as `MONUMENT_L001`, `MONUMENT_L002`, ...
  * `STAKE`: Formats as `STAKE_L001`, ...
  * `CORNER`: Formats as `CORNER_L001`, ...
  * `FENCE`: Formats as `FENCE_L001`, ...
  * `TREE`: Formats as `TREE_L001`, ...
  * `UTILITY`: Formats as `UTILITY_L001`, ...
  * `WP`: Formats as `WP_L001`, ...
* **Averaging Duration Button (`145x30 px`):** Cycles through averaging integration windows: `Avg: 1s` $\rightarrow$ `Avg: 5s` $\rightarrow$ `Avg: 10s` $\rightarrow$ `Avg: 30s`.

#### Row 2: Quick Note Tag Dropdown
* Selects standardized feature notes: `(No Tag)`, `Found Iron Pipe`, `Found Rebar`, `Found Stone Bound`, `Set Temp Flag`, `Mag Anomaly Only`, `Obstructed Sky`, or `Needs Verification`.

#### Row 3: Next Location ID Row
* Displays `ID: <next_label>` in Montserrat 14 `#FFCC00`.
* **Collision-Free Numbering:** Automatically scans `/survey_logged_pins.csv` for the highest existing number for that exact stem and assigns $N_{\text{max}} + 1$.
* **Circular Marquee:** If a compound name exceeds container width, it scrolls continuously across the row.

#### Row 4: Multi-Epoch Averaging Progress Bar
* Fills smoothly from 0% to 100% during RTK multi-epoch averaging.

#### Row 5: `[LOG PIN NOW]` Button (`300x40 px`)
* **Idle:** Green button displaying `LOG PIN NOW`.
* **Averaging:** Orange button displaying `AVERAGING RTK... X% (N pts)`.

#### Row 6: Logged Status
* Displays the most recently saved point label and target storage file:
  ```
  Logged: PIN_NW_01_L001
  File: /survey_logged_pins.csv
  ```

---

### Quick Settings Controls (Consolidated 3x4 Grid - 12 Buttons)

The bottom deck provides instantaneous touch control over hardware preferences in a clean 3-column by 4-row layout ($94 \times 27\text{ px}$ buttons with generous $8\text{ px}$ vertical and $9\text{ px}$ horizontal spacing):

| Column 1 (Left) | Column 2 (Center) | Column 3 (Right) |
| :--- | :--- | :--- |
| **Row 1:** `VOL: 20%..100%` / `MUTE` | **Row 1:** `AUD: GEIGER / BEEPS / MUTE` | **Row 1:** `BRI: 20%..100%` |
| **Row 2:** `NMEA: ON/OFF` | **Row 2:** `ROUTE: ON/OFF` | **Row 2:** `RTCM: ON/OFF` |
| **Row 3:** `TELEM: ON/OFF` | **Row 3:** `IMU: ON/OFF` | **Row 3:** `TOUCH: ON/OFF` |
| **Row 4:** `WIFI: ON/OFF` | **Row 4:** `UNITS: IMP/SI` | **Row 4:** `LORA: 19.2k / 9.6k` |

* **Button Descriptions:**
  * `VOL: xx%` / `VOL: MUTE`: Cycles master speaker output through 20%, 40%, 60%, 80%, 100%, and MUTE.
  * `AUD: GEIGER / BEEPS / MUTE`: Cycles proximity audio navigation modes (saved to `/settings.json`).
  * `BRI: xx%`: Cycles LCD backlight brightness through 20%, 40%, 60%, 80%, and 100%.
  * `NMEA: ON/OFF`: Toggles raw UM982 NMEA logging to `/nmea/nmea_YYYYMMDD_HHMMSS.log`.
  * `ROUTE: ON/OFF`: Toggles 1 Hz trajectory path logging to `/routes/route_YYYYMMDD_HHMMSS.csv`.
  * `RTCM: ON/OFF`: Toggles raw incoming RTCM3 differential stream logging to `/rtcm/`.
  * `TELEM: ON/OFF`: Toggles 5-second `[TELEM]` diagnostic serial output.
  * `IMU: ON/OFF`: Toggles 10 Hz raw IMU serial streaming.
  * `TOUCH: ON/OFF`: Toggles touch coordinate serial debugging.
  * `WIFI: ON/OFF`: Starts or stops the 2.4 GHz Wi-Fi radio and HTML5 Web File Portal.
  * `UNITS: IMP/SI`: Toggles between Imperial (feet/inches) and Metric (meters/centimeters).
  * `LORA: 19.2k` / `LORA: 9.6k`: Toggles remote over-the-air LoRa air-rate switching between standard ($19.2\text{ kbps}$) and deep woods canopy mode ($9.6\text{ kbps}$). Shows amber `REQ: 9.6k` while negotiating with the Base Station.
* **Row 5 (y=332):** Centered Wi-Fi status indicator (`Wi-Fi: Inactive (Tap to start)` / `Portal: http://...`).

---

## 6. Tile 2: Multi-File Waypoint Browser & Selector

Tile 2 enables operators to manage, browse, and switch between multiple project files stored on the MicroSD card.

![Tile 2: Multi-File Waypoint Browser](docs/images/screenshot_tile2_waypoint_browser.png)
*Figure 4: Waypoint browser showing multi-file selector bar and interactive waypoint table.*

### File Navigation Bar
* **`< Prev File` / `Next File >`:** Cycles through all CSV files on disk matching `/waypoints.csv` or `/waypoints_*.csv`.
* **File Label Marquee:** The active file name is displayed with dynamic circular marquee autoscroll for long filenames.
* **Persistent Memory:** The rover remembers both the active waypoint file and the active selected waypoint index across power cycles and reboots.

### Interactive Waypoint Table
* **Table Rows:** Displays point ID, target Latitude, Longitude, Elevation, and feature notes.
* **Tap-to-Target Selection:** Tapping any row instantly selects that waypoint as the active navigation target for Tile 0 and derives the naming stem for Tile 1.

---

## 7. Tile 3: Dual-Antenna Compass & Plumb Bubble Level

Tile 3 is designed for pole plumb alignment, bracket tilt calibration, and heading verification.

![Tile 3: Dual-Antenna Compass and Bubble Level](docs/images/screenshot_tile3_compass_level.png)
*Figure 5: 24pt True Heading readout, 224px spirit bubble level with Course-Up 8-point compass ring, and compact Tare button.*

### True Heading Readout
* Prominent 24pt Montserrat font displays true geographic azimuth (e.g. `Heading: 162.4`).
* Baseline length (`Baseline: 0.554 m`) and pitch (`Pitch: +0.2 deg`) provide real-time hardware integrity checks.
* **Display Pose Indicator:** Classifies physical display orientation (`Held Flat`, `Handheld Viewing Angle`, `Clipped Vertically to Pole`).

### 224px Plumb Bubble Level & Course-Up Compass Ring
* **Dynamic Course-Up 8-Point Compass Ring:** The 8 cardinal points (`N`, `NE`, `E`, `SE`, `S`, `SW`, `W`, `NW`) orbit around the bubble level ring in real time, keeping your physical heading oriented **UP** at 12 o'clock. North is rendered in bright red (`#EF4444`).
* **2D Bubble Motion:** Driven by onboard QMI8658 accelerometer:
  * Tipping pole Left moves bubble Left.
  * Tipping pole Right moves bubble Right.
  * Tipping pole Forward moves bubble Up.
  * Tipping pole Backward moves bubble Down.
* **Compact Tare Button (`62x28 px`):**
  * When clipping the display to an angled pole clamp, hold the survey pole perfectly plumb using a physical bubble level vial, then tap **`TARE`** (Blue).
  * The current display bracket angle is recorded into `settings.json` and zeroed out. The button turns Orange displaying **`RST`**.
  * Tapping **`RST`** clears the calibration and restores factory level alignment.
* **Pole Tilt Angles:** Readout below circle reports angular deflection in degrees (`dX: +0.2 deg | dY: -0.1 deg`).

---

## 8. Tile 4: Hardware Diagnostics & Live Observers

Tile 4 is a comprehensive live diagnostic suite for field troubleshooting and sensor health auditing.

![Tile 4: Hardware Diagnostics](docs/images/screenshot_tile4_diagnostics.png)
*Figure 6: Pinned diagnostic header deck, interactive logging toggles, and live subsystem observer blocks.*

### Pinned Top Control Deck & Elevated RF Survey Card
* **Pinned Firmware Header:** Fixed header displaying `"RTK ROVER v0.4.0"` in Montserrat 16 White (`#FFFFFF`).
* **Pinned LoRa RF Survey Card ($y=26$):** Elevated to the top of the tile to provide permanent visibility of RF link health without scrolling:
  * Live ambient noise floor (`Noise: -102dBm`).
  * Last received packet signal strength (`Pkt: -84dBm`).
  * Mode 0 hardware `AUX` edge interrupt counter and pulse width (`Edges: N low: X.Xms`).
  * RTCM3 frame counters: Station ARP (`1005`/`1006`), GPS MSM4 (`1074`), GLONASS MSM4 (`1084`), Galileo MSM4 (`1094`), and GLONASS Biases (`1230`).
  * Frame error and mid-frame truncation drops (`Err: N Trunc: N`).

### Expanded Scrollable Diagnostic Container (+57 px Viewport)
With the logging buttons consolidated into the Tile 1 Quick Settings grid, the scrollable diagnostic container viewport is expanded from $231\text{ px}$ to **$288\text{ px}$** ($+57\text{ px}$ vertical space):
* **Storage Status:** Displayed in Electric Cyan (`#22D3EE`): MicroSD mount state, card size, used space, free space, and internal FFat health.
* **Antenna 1 (Master):** Fix state, SV used/view, HDOP, VDOP, horizontal accuracy, vertical accuracy, and differential age.
* **Antenna 2 (Slave):** Heading fix, Slave SVs, Slave HDOP, measured baseline length, baseline pitch, and live ANT2 WGS84 coordinates (`Pos: Lat, Lon`, `Alt`).
* **Base Station Observer:** Sky blue (`#38BDF8`) card reporting:
  * Base Station ID (`#<id>`)
  * Differential correction age (`Diff: X.Xs`)
  * Base Station WGS84 Position (`Lat, Lon`, `Alt`)
  * Base Station ECEF Vectors (`X, Y, Z` in meters)
  * RTCM Message Counters (RTCM 1005 Station ARP frames and MSM multi-signal frames).
* **6-DOF IMU Telemetry:** Pitch, Roll, 3-axis Accelerometer (`Acc(g): X, Y, Z`), 3-axis Gyroscope (`Gyr(d/s): X, Y, Z`), calibrated tare offsets, and kinematic lever-arm compensation status.
* **UM982 COM1 / COM2 Metrics:** Active baud rates, byte throughput counters, sentence counts, and checksum error counters.
* **E220 LoRa Configuration & Hardware Wire Self-Test:** RF Channel, frequency (`925.125 MHz`), operating air data rate (`19.2 kbps` / `9.6 kbps`), transmit power (`21 dBm`), UART baud (`115200`), hardware register dump, and boot-time `M0`/`M1` pin continuity self-test (`M0:OK M1:OK`). Alerts in bold red if a jumper wire is loose or floating.

---

## 9. Hardware Screenshot Engine

The rover features a dedicated hardware screenshot engine for capturing field verification images directly to storage.

* **Trigger:** Long-press the physical **BOOT** button (GPIO 0) for $\ge 1.5\text{ seconds}$.
* **Audio Feedback:** The rover emits a distinctive two-tone audio beep (`1800 Hz` for 100 ms) confirming the screenshot capture.
* **Storage Path:** Images are saved directly to the MicroSD card under `/screenshots/`:
  ```
  /screenshots/screenshot_20261028_131500.bmp
  ```
* **Format:** Uncompressed 24-bit TrueColor Windows BMP format ($320 \times 480 \times 3\text{ bytes} \approx 460.8\text{ KB}$), immediately readable on any operating system without conversion.

---

## 10. Wireless Web File Portal & Field Maintenance

When maintenance or data offload is required in the field, the rover hosts an HTML5 web portal.

1. **Activate Wi-Fi:** Tap the `Wi-Fi` button on Tile 1 (Row 3).
2. **Connect Device:** Connect your smartphone, tablet, or laptop to the local Wi-Fi access point configured in `settings.json` (or connect directly to the rover's SoftAP).
3. **Open Browser:** Navigate to `http://<rover-ip>/` (IP address is displayed in Row 4 of Quick Settings on Tile 1).
4. **Web Portal Features:**
   * **Survey Logs:** Download `/survey_logged_pins.csv` directly into your GIS or CAD workflow.
   * **Route Tracks:** Download `/routes/route_*.csv` for trajectory mapping and walking surveys.
   * **NMEA Logs:** Download `/nmea/nmea_*.log` for post-processing kinematic (PPK) analysis.
   * **Screenshots:** View and download BMP screenshots captured in the field.
   * **Waypoint Uploads:** Upload newly created `waypoints.csv` or `waypoints_*.csv` files directly to the MicroSD card without pulling the card.
   * **Settings Management:** Update `settings.json` and sync rover RTC clock to local device time with one tap.

---

## 11. Configuration Reference (`settings.json`)

All operational parameters are stored in `/settings.json` on the MicroSD card and mirrored to internal flash:

| Parameter Key | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `minutesOffsetToUTC` | Integer | `-240` | Local timezone offset from UTC in minutes (`-240` = EDT / UTC-4, `-300` = EST / UTC-5). |
| `timezone_label` | String | `"EDT"` | 3-letter timezone abbreviation for header display. |
| `use_imperial_units` | Boolean | `false` | `false` for Metric (m, cm); `true` for Imperial (ft, in). |
| `antenna_height_m` | Float | `1.800` | Physical distance from crossbar centerline to ground tip in meters. |
| `antenna_spacing_m` | Float | `0.555` | Measured physical distance between Master and Slave antenna phase centers. |
| `apply_lever_arm` | Boolean | `true` | Enables 3D kinematic projection to pole centerline and ground tip. |
| `pole_tilt_tare_pitch_deg` | Float | `0.0` | Recorded bracket pitch angle tare offset. |
| `pole_tilt_tare_roll_deg` | Float | `0.0` | Recorded bracket roll angle tare offset. |
| `e220_channel` | Integer | `75` | LoRa RF channel (`75` = $925.125\text{ MHz}$, clean US ISM band). |
| `e220_uart_baud` | Integer | `115200` | Baud rate between ESP32 and E220 transceiver. |
| `e220_air_rate_kbps` | Float | `19.2` | LoRa over-the-air data transmission rate (`19.2` or `9.6`). |
| `audio_nav_mode` | Integer | `1` | Proximity navigation audio mode (`0` = Beeps, `1` = Geiger Clicks, `2` = Mute). |
| `survey_avg_seconds` | Integer | `5` | Multi-epoch integration duration (`1`, `5`, `10`, or `30`). |
| `default_label_prefix` | String | `"PIN"` | Default prefix when not in `<DERIVED>` mode. |
| `active_waypoint_file` | String | `"/waypoints.csv"` | Active CSV waypoint file path. |
| `active_waypoint_index` | Integer | `0` | Active selected waypoint index (restored on boot). |
| `brightness` | Integer | `80` | Backlight brightness percentage (`20`, `40`, `60`, `80`, `100`). |
| `volume` | Integer | `40` | Audio volume percentage (`0` = Mute, `20`, `40`, `60`, `80`, `100`). |

---

## 12. Troubleshooting & Field Diagnostics FAQ

### Q1: Antenna 1 shows `SINGLE` or `DGPS`, but never achieves `RTK FIX`.
* **Cause 1: Differential Link Down.** Check Row 2 `RTCM: X.Xs`. If age exceeds $5.0\text{ s}$ or reads `None`, the rover is not receiving RTCM packets. Inspect LoRa antenna connection and verify E220 RF channel on Tile 4 matches the Base Station transmitter (`Channel 55` / `905.125 MHz`).
* **Cause 2: Obstructed Sky.** Verify in Row 2 that satellite count used is $\ge 18$. Move away from heavy tree canopy or metal building walls.

### Q2: Row 3 shows `NO HDG` or `HDG FLT` while ANT1 has `RTK FIX`.
* **Cause:** The secondary antenna (ANT2) requires clear sky view to resolve carrier-phase integer ambiguities along the $55.5\text{ cm}$ baseline. Ensure the Slave antenna cable is firmly connected to UM982 RF2 and the crossbar is held relatively level.

### Q3: When rotating the survey pole in place, the coordinates shift.
* **Cause:** Kinematic lever-arm compensation is disabled. Check Tile 4 IMU block to ensure `Lever-Arm: ON`. Verify in `settings.json` that `antenna_spacing_m` is set to `0.555` and `antenna_height_m` matches your physical pole height (`1.800 m`).

### Q4: The spirit bubble is not centered when the pole is held plumb.
* **Cause:** The display is mounted on an angled bracket. Hold the survey pole plumb using a physical bubble vial, navigate to Tile 3, and tap **`TARE`**. The bubble will center at $(0.0^\circ, 0.0^\circ)$ and store the bracket compensation offset permanently.

### Q5: How do I back up my survey data in the field?
* Tap `Wi-Fi` on Tile 1, connect with your phone, open `http://<rover-ip>/`, tap `/survey_logged_pins.csv`, and save to your mobile device. You can also power down the rover and remove the MicroSD card to read it directly on a computer.

---

## 13. Bi-Directional LoRa Protocol & Base Station Integration

The RTK Rover implements a bi-directional telemetry and remote control protocol operating over the Ebyte E220 900 MHz LoRa link.

### 13.1 Half-Duplex Quiet-Window Interleaving
* The Base Station broadcasts RTCM3 corrections in a burst at the beginning of each 1-second UTC epoch ($t = 0.00\text{ s} \dots 0.45\text{ s}$).
* The Rover transmits an uplink telemetry frame strictly within the post-epoch quiet window ($t = 0.10\text{ s} \dots 0.45\text{ s}$) every 5 seconds.
* At $19.2\text{ kbps}$, the 39-byte frame consumes only $\approx 30\text{ ms}$ Time-on-Air ($<1\%$ duty cycle), leaving hundreds of milliseconds of guard margin before the next base epoch.

### 13.2 39-Byte Uplink Telemetry Frame
* **Sync Preamble:** `0xAA 0x55` (distinct from RTCM3 `0xD3` to prevent false-framing).
* **Payload:** Sub-millimeter ground-tip coordinates (`tip_lat`, `tip_lon`, `tip_height_m`), horizontal accuracy (`h_acc_mm`), fix quality, Master/Slave satellite counts, True Heading centidegrees, battery percentage and millivolts, Rover packet RSSI, live ambient noise floor, and sequence counter.
* **Integrity:** Protected by a 16-bit CRC16-CCITT checksum.

### 13.3 Touchscreen Remote Air-Rate Switching
* **Field Problem:** Deep timber or dense foliage can cause fringe RF packet loss at $19.2\text{ kbps}$. Switching down to $9.6\text{ kbps}$ adds $+3.5\text{ dB}$ of link margin.
* **Procedure:** Tap the `LORA: 19.2k` button on Tile 1. The button displays amber `REQ: 9.6k`.
* **Zero-Credential Negotiation:** Rover transmits `CMD_REQ_RADIO_SWITCH` (`0x02`), Base replies with `ACK_RADIO_SWITCH` (`0x82`), and both radios reconfigure their registers in Mode 3 simultaneously without restarting the Base systemd service or entering passwords.
* **Failsafe Rollback:** If communication is silent for 60 seconds after a rate switch, both Rover and Base automatically revert to the $19.2\text{ kbps}$ fallback profile.

### 13.4 Base Station Pacer (`e220_pacer.py` v3.0.0)
* **Real-Time Terminal Output:** Displays live Rover telemetry lines on reception.
* **Atomic RAM State (`/run/rtk_rover_status.json`):** Writes live JSON on RAM tmpfs for instant zero-overhead dashboard querying.
* **Rolling Daily CSV Log (`~/rover_logs/rover_YYYY-MM-DD.csv`):**
  * Auto-prunes files older than 30 days (`--log-retention-days 30`).
  * Caps maximum log directory footprint at $50\text{ MB}$ (`--max-log-mb 50`).
  * Compresses completed logs with `gzip` at midnight (`rover_YYYY-MM-DD.csv.gz`).
