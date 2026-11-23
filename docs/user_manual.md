# RTK Rover - Field Operator's User Manual

**Document Version:** `v1.10.0`  
**Firmware Version:** `v0.10.0`  
**Last Updated:** November 22, 2026  
**Hardware Platform:** Waveshare ESP32-S3-Touch-LCD-3.5B, Holybro H-RTK Unicore UM982, Ebyte E220-900T30S, DFRobot SEN0696 10-DOF Beam IMU  

---

## Table of Contents
1. [Introduction & System Overview](#1-introduction--system-overview)
2. [Hardware Setup & Power-On Procedure](#2-hardware-setup--power-on-procedure)
3. [Persistent 4-Row Top Status Header Bar](#3-persistent-4-row-top-status-header-bar)
4. [Tile 0: Unified Guidance Triangle, Auto-Bullseye & Map View](#4-tile-0-unified-guidance-triangle-auto-bullseye--map-view)
5. [Tile 1: Dedicated Field Survey Logger](#5-tile-1-dedicated-field-survey-logger)
6. [Tile 2: Multi-File Waypoint Browser & Selector](#6-tile-2-multi-file-waypoint-browser--selector)
7. [Tile 3: Dual-Antenna & Beam 9-DOF Compass, Plumb Bubble Level & CAL MAG](#7-tile-3-dual-antenna--beam-9-dof-compass-plumb-bubble-level--cal-mag)
8. [Tile 4: Hardware Diagnostics & Live Dual-Bus (I2C0/I2C1) Observers](#8-tile-4-hardware-diagnostics--live-dual-bus-i2c0i2c1-observers)
9. [Tile 5: Dedicated 5x3 Quick Settings Deck & Safe Auto-Off](#9-tile-5-dedicated-5x3-quick-settings-deck--safe-auto-off)
10. [Hardware Screenshot Engine](#10-hardware-screenshot-engine)
11. [Wireless Web File Portal & Field Maintenance](#11-wireless-web-file-portal--field-maintenance)
12. [Configuration Reference (`settings.json`)](#12-configuration-reference-settingsjson)
13. [Troubleshooting & Field Diagnostics FAQ](#13-troubleshooting--field-diagnostics-faq)
14. [Bi-Directional LoRa Protocol & Base Station Integration](#14-bi-directional-lora-protocol--base-station-integration)

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
4. **Power-On:**
   * **Internal Battery (`MT3608` 5V Boost Mode):** Power on untethered using an internal `1S` Li-Ion/LiPo battery (`~806 mA` / `~3.0 W` typical field draw at `80%` brightness, `Wi-Fi OFF`: expect **~3.2–3.4 hrs** on `3,000 mAh` or **~10.6–11.2 hrs** on `10,000 mAh`; see [Power Budget & Runtime](schematics/system_wiring_schematics.md#3-system-power-budget-mt3608-boost-analysis--battery-runtime-estimates)). Before long battery-only walks, set `auto_shutdown_battery_min` to `0` in `/settings.json` (defaults to `5` minutes idle). Onboard USB-C `AXP2101` charging is capped at $500\text{ mA}$ (~7 hrs for `3,000 mAh`, ~20+ hrs for `10,000 mAh`).
   * **External USB Mode:** Optionally connect an external USB 5V power pack while floating/charging the internal UPS cell at $500\text{ mA}$ via the AXP2101 PMIC.
5. **Boot Sequence:**
   * Waveshare boots and initializes SPI Flash, MicroSD card, and I2C peripherals.
   * QSPI Display powers up at default brightness (80%).
   * MicroSD card is mounted and `/settings.json` is mirrored to internal `FFat`.
   * FreeRTOS tasks start: Core 0 streams RTCM3 corrections from E220 to UM982 COM2 and parses UM982 COM1 NMEA/heading frames; Core 1 renders LVGL 9 at 30 Hz.
6. **Acquisition Checklist:**
   * Verify power widget reads `[USB] 5.04V+` (charging) or `[USB] 5.04V` (float) in Row 1.
   * Verify ANT1 acquires `RTK FIX` (Row 2, Green badge).
   * Verify ANT2 acquires `HDG FIX` (Row 3, Green badge).
   * Verify baseline length reads $\approx 55.5\text{ cm}$ ($\pm 1.5\text{ cm}$).

---

## 3. Persistent 4-Row Top Status Header Bar

The top 74 pixels of the display remain permanently visible across all 6 operational tiles (`[Tile 5: Settings] <-> [Tile 0: Nav] <-> [Tile 1: Survey] <-> [Tile 2: Waypoints] <-> [Tile 3: Compass] <-> [Tile 4: Diag]`).

![Top Status Header Bar](docs/images/screenshot_header_bar.png)
*Figure 1: Persistent 4-row status header bar showing system time, battery, ANT1 fix, ANT2 fix, and ground-tip coordinates.*

### Row 1: System Time, Power & Environment
* **Date & Time:** Local calendar date (`YYYY-MM-DD`) and 24-hr time (`HH:MM:SS TZ`) computed from UTC using `minutesOffsetToUTC` and synchronized with GNSS atomic clocks.
* **Units Badge:** `SI` (meters/centimeters) or `IMP` (feet/inches). Tap `UNITS` on Tile 5 Quick Settings to toggle.
* **Wi-Fi Indicator:** Compact icon-only symbol (`LV_SYMBOL_WIFI`) with distinct multi-state colors:
  * **Electric Cyan (`#22D3EE`):** Connected to local router (`STA`).
  * **Vivid Violet (`#A855F7`):** Access Point hotspot active (`AP`).
  * **Vivid Amber (`#F59E0B`):** Actively connecting (`CONN`).
  * **Slate Gray (`#475569`):** Inactive / radio off (`OFF`).
* **Power Telemetry Widget:** Minimalist dual-mode power indicator:
  * **USB Power Mode:** Displays `[USB] 5.04V+` while charging the backup battery, dropping to `[USB] 5.04V` when floating at 100%.
  * **Battery Mode (Unplugged):** Displays `[Bat] 85% 3.92V` with color-coded battery icon (Green $>60\%$, Amber $20\text{--}60\%$, Red $<20\%$).

### Row 2: Master Antenna (ANT1 - Position) Metrics
* **Fix Status Badge:**
  * `RTK FIX` (Bright Green `#00FF88`): Centimeter carrier-phase integer fixed solution ($< 2\text{ cm}$).
  * `RTK FLT` (Yellow `#FFCC00`): Carrier-phase float solution ($5\text{ cm}..30\text{ cm}$).
  * `DGPS` (Orange `#FFAA00`): Code-differential solution ($0.5\text{ m}..1.5\text{ m}$).
  * `SINGLE` (Red `#FF4444`): Autonomous uncorrected solution ($1.5\text{ m}..5.0\text{ m}$).
  * `[OFF]` (Red `#EF4444`): External USB 5V rail disconnected (receiver unpowered to preserve battery).
* **Satellites:** Count of satellites used in fix vs. total in view (`SV: 28/34`).
* **Horizontal Precision:** Estimated horizontal precision standard deviation (`+/- 0.015m` / `+/- 0.05ft`).
* **RTCM Differential Age:** Age of incoming RTCM corrections in seconds (`RTCM: 1.0s`).
* **LoRa Signal Strength:** Explicitly labeled `"LoRa: "` prefix (e.g. `LoRa: -85dB`) with multi-tier color coding:
  * Green: Strong signal ($\ge -90\text{ dBm}$)
  * Amber/Yellow: Moderate signal ($-91\text{ dBm} \dots -105\text{ dBm}$)
  * Red: Weak fringe signal ($< -105\text{ dBm}$) or `-- dB` when link is idle.
  * Gray (`LoRa: OFF`): External USB 5V rail disconnected.

### Row 3: Slave Antenna (ANT2 - Heading) & Woods Fallback Metrics
* **Heading Fix Badge:**
  * `HDG FIX` (Bright Green `#00FF88`): Dual-antenna carrier-phase integer ambiguity resolved (`0.2 deg` accuracy). Continuously teaches magnetic declination, `B_ref`, `Bz_ref`, and beam pitch tare.
  * `HDG FLT` (Yellow `#FFCC00`): Dual-antenna carrier-phase solution converging.
  * `HDG CMP` / `CMP` (Teal `#14B8A6`): Calibrated Beam 9-DOF AHRS (`SEN0696` `BMI323+BMM350` on `I2C1 GPIO 47/48`) or Tier-B `MMC5983MA` fallback, engaging immediately (`0.0 s` switchover) on Slave dropout.
  * `HDG WALK` / `WLK` (Cyan `#06B6D4`): Deep-woods kinematic displacement heading established from `>= 0.70 m` (2-3 paces) movement with 3.0s walk persistence filter. Velocity-independent.
  * `HDG GYRO` / `GYR` (Blue `#3B82F6`): In-place pivot yaw integration or bias-compensated gyro coasting during localized iron anomalies (`|B|`, `Bz`, or 200 ms gyro-mag mismatch).
  * `HDG HLD` / `HLD` (Slate `#94A3B8`): Stationary zero-drift hold (`0.0 deg/s` drift) freezing heading when stationary.
  * `[OFF]` (Red `#EF4444`): External USB 5V rail disconnected.
  * `NO HDG` (Red `#FF4444`): Heading unavailable; prompts user to walk 2–3 paces to establish displacement azimuth.
* **Slave Satellites:** Number of satellites tracked by secondary RF path (`SV: 26`).
* **Baseline Length:** Measured 3D distance between antenna phase centers (`BL: 55.4cm`).
* **Baseline Pitch:** Crossbar pitch angle relative to the local horizon (`Pitch: +0.2 deg`).
* **True Heading:** True geographic azimuth of antenna baseline or integrated fallback source (`Hdg: 142.6 deg`).

### Row 4: Ground-Tip Geodetic Coordinates
* **Prominent 14pt Font:** Displays real-time kinematic ground-tip Latitude, Longitude, and Ellipsoidal Height:
  ```
  42.36012480, -71.09421520, 12.450 m
  ```

---

## 4. Tile 0: Unified Guidance Triangle, Auto-Bullseye & Map View

Tile 0 (`ui_tile0_guidance.c`, `ui_tile0_views.c`, `ui_tile0_map.c`) is the primary navigational workspace (`v0.10.0`). Tapping the top-left mode badge cycles directly between **`[GUIDANCE]`** and **`[MAP VIEW]`**:

```
[GUIDANCE] (Log-Scale Triangle + Auto < 5 m Bullseye)  <--->  [MAP VIEW] (Sticky Zoom + Active Segment)
```

### Navigation Bar, Anchor Modes & Bottom Controls
* **Target Title Bar:** Displays `Start -> Target [SWAP]` (in Deed mode) or `HERE -> Target [SET A]` (in Direct-Here mode).
  * **Deed Mode (`A: DEED`, Teal):** By default, Point `B` is the active CSV waypoint (`Index`) and Start `A` is `Index - 1` (wrapping around closed boundary loops). Tap the title bar (`[SWAP]`) to reverse `A <-> B` cleanly in place; stepping `<` or `>` resets `[SWAP]` and advances to the next boundary leg.
  * **Direct Mode (`A: HERE`, Violet):** Tap the bottom-left **`[A: DEED / A: HERE]`** button (auto-enabled on 1-waypoint CSVs) to anchor Start `A` at your current live GNSS position (`HERE -> B`). Tap **`[SET A]`** (or **`[A: HERE]`**) anytime after walking around an obstacle to drop a fresh Start `A` from your new position.
* **Route Recording Button (`[REC OFF]` / `[REC ON]`):** Bottom-right canvas button toggles continuous route track CSV logging to `/routes/route_YYYYMMDD_HHMMSS.csv`.

---

### Mode 0: Unified Guidance (Log-Scale Triangle -> `< 5 m` Auto-Bullseye)

#### Phase A — Line-Up Log-Scale Triangle (`> 5.0 m` from Target `B`, exit hysteresis `> 5.3 m`)
* **Fixed Line-Up Frame:** Start `A` (Amber/Violet `12 px`) stays anchored near bottom-center (`y = 174`), Target `B` (Green `16 px`) near top-center (`y = 46`), connected by the `A -> B` baseline with `18 px` dashed undershoot/overshoot aprons below `A` and above `B`.
* **Symmetric Logarithmic Cross-Track Scale (`line_log_max_m = 50.0 m`):** Rover `R` (`14 px` Sky Blue, turning Green within `2 cm` of line) moves horizontally on a logarithmic scale from `0.02 m` out to `line_log_max_m`, keeping centimeter sensitivity near the line (`1m / 10m` SI or `3ft / 30ft` Imperial ticks) without clamping flat when walking around obstacles `5–50 m` off-line.
* **Triangle Geometry & Priority Heading Arrow:** Draws both the perpendicular tie-line (`R -> Baseline`) and direct hypotenuse (`R -> B`). A barbed heading arrow points from `R` at your true relative heading (`true_heading_deg - line_bearing_deg`) with dual-lock color coding prioritizing Target `B`:
  1. **Hypotenuse `R -> B` Priority:** `<= 4 deg` (`line_arrow_green_deg`) -> **Arrow + Hypotenuse Green**; `<= 15 deg` (`line_arrow_amber_deg`) -> **Arrow + Hypotenuse Orange**.
  2. **Perpendicular Tie Return** (only outside `0.15 m` of line when not aimed near `B`): `<= 4 deg` -> **Arrow + Tie Green**; `<= 15 deg` -> **Arrow + Tie Orange**.
  3. **Outside Windows:** **Arrow White**.
* **Acoustic Fade-Out `L/R` Audio (`ES8311`):** Outside Bullseye, left-of-line (`offset < -0.05 m`) plays rising chirps (`660 -> 1100 Hz`, steer RIGHT), right-of-line plays falling chirps (`1100 -> 660 Hz`, steer LEFT), and on-line (`<= 0.05 m`) plays a steady `1800 Hz` pip. Chirp cadence accelerates (`2500 -> 220 ms`) and PCM amplitude ramps smoothly (`25% -> 100%`) as cross-track distance shrinks toward `0`, fading to **complete silence** beyond `line_audio_fade_m` (`20.0 m` default).

#### Phase B — Auto-Bullseye Centimeter Finish (`< 4.8 m` entry chime -> `BULLSEYE`)
* **Hysteresis Handoff:** Crossing inside `4.8 m` of Target `B` emits a `1400 -> 2000 Hz` acquisition chime and transitions into centered concentric rings (`5.0 m / 1.5 m`, auto-zooming to `1.5 m / 0.5 m` inside `1.5 m`) **with the `A -> B` property chord drawn through `B`** and a North-Up barbed heading arrow colored Green/Orange/White relative to `B`.
* **ES8311 Pin Proximity Audio (`Geiger` / `Beeps` / `Mute`):** Accelerating clicks/tones from `2.0 m` (`Geiger`) or `5.0 m` (`Beeps`) down to a `2400 Hz` pin-lock tone at `<= 2 cm`.

---

### Mode 1: Map View (Sticky Zoom, Active Segment & Tie-Lines)

When the mode badge is set to `MAP VIEW` (Emerald Green `#10B981`):
* **Persistent Sticky Zoom (`map_zoom_m_per_px`):** Your manual `[+]`/`[-]` zoom scale is preserved across `GUIDANCE <-> MAP VIEW` switches, CSV parcel switches, and device reboots until you tap `[FIT]` (which restores Auto-Fit `0.0`).
* **Highlighted Active Segment (`t0_map_active_seg`):** Draws the actively followed `A -> B` leg on top of the Sky Blue boundary polygon in **bright `4 px` solid Emerald (`A: DEED`)** or **`3 px` dashed Violet (`A: HERE`)**, with distinct Start `A` (`12 px` Amber/Violet) and Target `B` (`16 px` Green) pins.
* **HUD Zoom & Pan Controls (Canvas Right Edge):**
  * `[+]`: Zoom in (`-30%` m/px down to `0.05 m/px`, saved to `settings.json`).
  * `[-]`: Zoom out (`+40%` m/px up to `30.0 m/px`, saved to `settings.json`).
  * `[FIT]`: Auto-fit bounding box of all loaded waypoints + live rover position and reset sticky manual zoom to `0.0`.
  * `[CTR]`: Center viewport on live rover ground-tip coordinates while keeping current zoom.
* **Cadastral Layers & 4-Row Readout:** Connects positive `poly_order` vertices (`1..N`), dashed Orange tie-lines (`poly_order < 0`), Gold boundary / Orange reference pins, Cyan rover marker + heading beam, optional breadcrumb route trail (`map_show_route_trail`), and bottom target distance/bearing, active segment, viewport center, and scale readouts.

---

## 5. Tile 1: Dedicated Field Survey Logger

Removing the Quick Settings buttons to Tile 5 gives Tile 1 (`ui_tile1_survey.c`) spacious controls optimized for tough field monument logging:

![Tile 1: Field Survey Logger](docs/images/screenshot_tile1_survey_settings.png)
*Figure 3: Dedicated Field Survey point logger with 40 px dropdowns, 12 monument tags, 52 px LOG button, and pre-log quality card.*

### Survey Logger Controls

#### Row 1: Location ID Prefix & Averaging Duration (`40 px` tall)
* **Prefix Dropdown:** `<DERIVED>` (uses active navigation target stem, e.g. `PIN_NW_01 -> PIN_NW_01_L001`), plus `PIN`, `PIPE`, `MONUMENT`, `STAKE`, `CORNER`, `FENCE`, `TREE`, `UTILITY`, `WP`.
* **Averaging Duration Button:** Cycles `Avg: 1s -> Avg: 5s -> Avg: 10s -> Avg: 30s`. Uses circular vector averaging (`atan2(sum_sin, sum_cos)`) for heading and captures real Core 1 `pole_tilt_deg` via lock-free snapshot into `/survey_logged_pins.csv`.

#### Row 2: Monument / Note Tag Dropdown (`40 px` tall, `12` standardized `kNoteTags`)
* `(No Tag)`, `Found Iron Pipe`, `Found Capped Rebar`, `Found Stone Mon`, `Found Mag Nail`, `NO PIN - Search Pt`, `NO PIN - Marked Spot`, `Unknown Pin/Pipe`, `Set Temp Flag`, `Mag Anomaly Only`, `Obstructed Sky`, `Needs Verification`.

#### Row 3 & Row 4: Next ID Marquee & Progress Bar
* Displays collision-free `ID: <next_label>` in Montserrat 14 `#FFCC00` with circular marquee, plus 0-100% multi-epoch averaging bar.

#### Row 5: Active-Tag `[LOG]` Button (`300 x 52 px`) & Pre-Log Quality Card
* **Idle:** Green button echoing the active note tag (or Sky Blue `#0284C7` for `NO PIN` search/marked points so monument misses are unmistakable).
* **Pre-Log Survey Quality Summary Card:** Displays live Fix/SV/HDOP, Horizontal/Vertical accuracy (`H/V-Acc`), real-time `Pole Tilt`, active target deltas (`dN / dE`), and last logged point + `/survey_logged_pins.csv` status.

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
* **Tap-to-Target Selection:** Tapping any row (or stepping with `<` / `>`) selects that waypoint as the active navigation target for Tile 0 and derives the naming stem for Tile 1.
* **Sub-Millisecond Tactile Audio & Same-Frame (`0 ms`) Readout Response (`v0.7.7`):** Powered by a dedicated Priority-4 real-time audio synthesizer task (`RoverAudio`), synchronous same-frame guidance readout calculation (`refresh_t0_guidance_readout_immediately()`), `512 B` chunked CSV loading (`< 5 ms`), a 64-entry RAM stem cache (`s_stem_cache`), and background debounced settings persistence with smart `FFat` flash-wear skip.

### Waypoint CSV Format & Cadastral Ordering Schema
The Rover natively loads standard 19-column and 20-column survey CSV files. The 20-column schema introduces the `poly_order` column for cadastral mapping:

```csv
timestamp,firmware_ver,label,latitude,longitude,elevation,poly_order,pos_fix,hdg_fix,h_acc_m,v_acc_m,hdop,sats,slave_sats,diff_age_s,base_id,true_heading_deg,pole_tilt_deg,samples,notes
2026-10-27_12-00-00.000-240,v0.6.0,PIN_NW_01,42.360100000,-71.094200000,12.500,1,DESIGN,NONE,0.000,0.000,0.0,0,0,0.0,0,0.00,0.00,1,NW corner iron pipe
2026-10-27_12-00-00.000-240,v0.6.0,PIN_NE_02,42.360450000,-71.093500000,13.100,2,DESIGN,NONE,0.000,0.000,0.0,0,0,0.0,0,0.00,0.00,1,NE corner rebar
2026-10-27_12-00-00.000-240,v0.6.0,PIN_SE_03,42.359800000,-71.093400000,11.800,3,DESIGN,NONE,0.000,0.000,0.0,0,0,0.0,0,0.00,0.00,1,SE corner granite bound
2026-10-27_12-00-00.000-240,v0.6.0,PIN_SW_04,42.359750000,-71.094150000,12.000,4,DESIGN,NONE,0.000,0.000,0.0,0,0,0.0,0,0.00,0.00,1,SW corner pin
2026-10-27_12-00-00.000-240,v0.6.0,REF_TREE_TIE,42.360550000,-71.093350000,13.000,-2,DESIGN,NONE,0.000,0.000,0.0,0,0,0.0,0,0.00,0.00,1,Witness tree tie point (ties to vertex 2)
```

* **`poly_order > 0` (e.g. `1, 2, 3, 4`):** Boundary vertices that define the closed property boundary loop in sequence ($1 \rightarrow 2 \dots \rightarrow N \rightarrow 1$). Rendered as Gold pins on the Map View.
* **`poly_order < 0` (e.g. `-2`, `-10`):** Cadastral tie / reference points. The negative number identifies the boundary vertex to which this point connects. Rendered as Orange pins with dashed tie-lines on the Map View.
* **`poly_order = 0` (or `CONTROL`, `REF`, `TIE`):** Standalone reference, road center, or control points not connected to the parcel boundary polygon.
* **Backward Compatibility:** If the `poly_order` column is absent, all waypoints default to a sequential boundary loop ($0 \rightarrow 1 \dots \rightarrow N-1 \rightarrow 0$).

---

## 7. Tile 3: Dual-Antenna & Beam 9-DOF Compass, Plumb Bubble Level & `CAL MAG`

Tile 3 (`ui_tile3_compass.c`) integrates dual-antenna heading, Beam 10-DOF 9-axis fusion (`SEN0696` on `I2C1 GPIO 47/48`), pole plumb alignment, and `360-deg` magnetometer calibration:

![Tile 3: Dual-Antenna Compass and Bubble Level](docs/images/screenshot_tile3_compass_level.png)
*Figure 5: 24pt True Heading readout w/ provenance & iron anomaly warning, 224px spirit bubble level, `CAL MAG`, `TARE`, and `Baro Rel Elev`.*

### True Heading, Provenance & Beam Telemetry
* Prominent 24pt Montserrat readout displays true geographic azimuth (`Heading: 162.4 deg`) alongside active source provenance (`DUAL FIX`, `DUAL FLT`, `BEAM 9DOF`, `WALK`, `GYRO COAST`, `HOLD`) and `[IRON ANOMALY]` orange highlight whenever nearby metal triggers gyro coasting (`|B|`, `Bz`, or 200 ms gyro-mag divergence).
* Baseline length (`BL: 55.4 cm`), crossbar pitch (`Pitch: +0.2 deg`), Beam 10-DOF status (`[BEAM 10DOF I2C1]`), and `RTK_FIX`-disciplined **`Baro Rel Elev`** (`BMP581` relative vertical elevation).
* **Display Pose Indicator:** Onboard `QMI8658` remains dedicated to physical display orientation (`Held Flat`, `Handheld Viewing Angle`, `Clipped Vertically to Pole`) whenever `SEN0696` is online.

### 224px Plumb Bubble Level, `CAL MAG` & `TARE`
* **Dynamic Course-Up 8-Point Compass Ring:** Keeps physical heading UP (`12 o'clock`) with North in bright red.
* **360-Deg Magnetometer Ellipsoid Calibration (`CAL MAG`, Left Side `80x28 px`):**
  1. Tap `CAL MAG` on Tile 3 (or via the Tile 5 Quick Settings shortcut) to start the `300`-sample PSRAM ellipsoid capture.
  2. Rotate the pole `360 deg` smoothly while gently tilting to cover all `16` sectors (`SectorMask` + live RMS coaching).
  3. Tap `FINISH CAL` once sufficient spatial coverage is captured (`FIT OK` saves hard-iron offset & `3x3` soft-iron matrix cleanly to `settings.json`). During `CAL MAG`, battery Auto-Off is automatically inhibited.
* **Compact `TARE` Button (Right Side):** Hold pole physically plumb and tap **`TARE`** (`Blue` -> **`RST`** Orange) to zero display/beam pitch-roll offsets and persist them into `settings.json`.

---

## 8. Tile 4: Hardware Diagnostics & Live Dual-Bus (`I2C0`/`I2C1`) Observers

Tile 4 (`ui_tile4_diagnostics.c`) provides live troubleshooting telemetry across all hardware buses:

![Tile 4: Hardware Diagnostics](docs/images/screenshot_tile4_diagnostics.png)
*Figure 6: Pinned diagnostic header deck, LoRa RF survey card, and live subsystem observer blocks.*

* **Pinned Top Deck:** Fixed firmware header (`RTK ROVER v0.9.0`) + elevated LoRa RF Survey Card (`Noise`, `Pkt`, `AUX` edges, `RTCM3` counters, `Err/Trunc` drops) + `Auto-Off` idle countdown/inhibit status.
* **Dual-Bus & Sensor Health Cards:**
  * **Dual `I2C0 / I2C1` Status (`t4_i2c_lbl`):** Separate `I2C0` (`GPIO 7/8`) & `I2C1` (`GPIO 47/48`) bus error counters and `10DOF` active bus indicator (`0` expected `I2C0` stalls with `SEN0696` isolated on `I2C1`).
  * **Beam 10-DOF & Pose (`t4_imu_lbl`):** Reports `SEN0696` bus/status, `Orient: M4`, 50-sample stationary gyro `FOC`, `3D-Cal` ellipsoid fit status, `BMP581` pressure (`hPa`) + `Baro Rel Elev`, alongside onboard `QMI8658` display pose (`Pitch`/`Roll`).
  * **Storage, `ANT1`, `ANT2`, Base Station (`ECEF` -> `WGS84`), `UM982 COM1`, Dual-Power `UPS` (`VBUS_GOOD` + `MT3608`), and `E220` Pin Self-Test.**

---

## 9. Tile 5: Dedicated `5x3` Quick Settings Deck & Safe Auto-Off

Centered directly `1` tap `< Prev` from Tile 0, **Tile 5 (`ui_tile5_settings.c`)** features a `5x3` grid of large `94 x 44 px` touch buttons (`10 px` row gap) and a live Wi-Fi / Auto-Off / LoRa footer:

| Column 1 (Left) | Column 2 (Center) | Column 3 (Right) |
| :--- | :--- | :--- |
| **Row 1:** `VOL: 20%..100%` / `MUTE` | **Row 1:** `AUD: GEIGER / BEEPS / MUTE` | **Row 1:** `BRI: 20%..100%` |
| **Row 2:** `SERIAL: ON/OFF` | **Row 2:** `NMEA: ON/OFF` | **Row 2:** `RTCM: ON/OFF` |
| **Row 3:** `TELEM: ON/OFF` | **Row 3:** `IMU: ON/OFF` | **Row 3:** `TOUCH: ON/OFF` |
| **Row 4:** `WIFI: CONN/STA/AP/OFF` | **Row 4:** `UNITS: IMP/SI` | **Row 4:** `LORA: 19.2k / 9.6k / 4.8k` |
| **Row 5:** `AUTO-OFF: 5m/15m/30m/OFF` | **Row 5:** `CAL MAG` (`OK` Green / Ready Blue) | **Row 5:** `BLE: PLAN2` (Reserved) |

* **`WIFI: CONN/STA/AP/OFF`:** Amber `CONN` (tap again `<= 100 ms` to cancel) -> Emerald Green `STA` (`http://<local-ip>/`, live immediately on DHCP) -> Violet `AP` (`http://192.168.4.1/`, SSID `RTK-Rover`) -> Navy Blue `OFF`.
* **`LORA: 19.2k / 9.6k / 4.8k`:** Cycles OTA air-rate switching via 4-ACK `Second 1` / Mode 3 `Second 2` sync + 60s rollback failsafe.
* **`AUTO-OFF` (`5m / 15m / 30m / DISABLED`) & Motion/REC-Safe Shutdown Guard:**
  * Idle shutdown on battery is automatically **inhibited** while external `USB` power is present, `Wi-Fi` is active, a `Survey` or `CAL MAG` is active, **`REC ON` route recording**, **`NMEA` log**, or **`RTCM` log** is running, physical motion is detected (`speed >= 0.30 m/s`, IMU delta, or `>= 1.5 m` GNSS displacement), or touch/`BOOT` input occurs.
  * When an uninhibited idle timer expires, a full-screen `lv_layer_top()` **`30 s` countdown modal (`TAP ANYWHERE TO CANCEL`)** alerts with warning beeps (`30, 20, 10, 5..1 s`) and cleanly stops & flushes active loggers (`route`, `nmea`, `rtcm`, `save_settings`, `serial`) before cutting `AXP2101` PMIC power.

---

## 10. Hardware Screenshot Engine

The rover features a dedicated hardware screenshot engine for capturing field verification images directly to storage.

* **Trigger:** Long-press the physical **BOOT** button (GPIO 0) for $\ge 1.5\text{ seconds}$.
* **Audio Feedback:** The rover emits a distinctive two-tone audio beep (`1800 Hz` for 100 ms) confirming the screenshot capture.
* **Storage Path:** Images are saved directly to the MicroSD card under `/screenshots/`:
  ```
  /screenshots/screenshot_20261028_131500.bmp
  ```
* **Format:** Uncompressed 24-bit TrueColor Windows BMP format ($320 \times 480 \times 3\text{ bytes} \approx 460.8\text{ KB}$), immediately readable on any operating system without conversion.

---

## 11. Wireless Web File Portal & Field Maintenance

When maintenance or data offload is required in the field, the rover hosts an HTML5 web portal.

1. **Activate Wi-Fi:** Tap the `WIFI` button on Tile 5 (Row 4).
2. **Connect Device:** Connect your smartphone, tablet, or laptop to the local Wi-Fi access point configured in `settings.json` (or connect directly to the rover's SoftAP).
3. **Open Browser:** Navigate to `http://<rover-ip>/` (IP address is displayed in the bottom status footer on Tile 5).
4. **Web Portal Features:**
   * **Survey Logs:** Download `/survey_logged_pins.csv` directly into your GIS or CAD workflow.
   * **Route Tracks:** Download `/routes/route_*.csv` for trajectory mapping and walking surveys.
   * **NMEA Logs:** Download `/nmea/nmea_*.log` for post-processing kinematic (PPK) analysis.
   * **Screenshots:** View and download BMP screenshots captured in the field.
   * **Waypoint Uploads:** Upload newly created `waypoints.csv` or `waypoints_*.csv` files directly to the MicroSD card without pulling the card.
   * **Settings Management:** Update `settings.json` and sync rover RTC clock to local device time with one tap.

---

## 12. Configuration Reference (`settings.json`)

All operational parameters are stored in `/settings.json` on the MicroSD card and mirrored to internal flash:

| Parameter Key | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `settings_schema_version` | Integer | `2` | `settings.json` schema revision (v2 auto-migrates existing v1 configs cleanly on boot). |
| `minutesOffsetToUTC` | Integer | `-300` | Local timezone offset from UTC in minutes (`-240` = EDT / UTC-4, `-300` = EST / UTC-5). |
| `timezone_label` | String | `"EDT"` | 3-letter timezone abbreviation for header display. |
| `use_imperial_units` | Boolean | `false` | `false` for Metric (m, cm); `true` for Imperial (ft, in). |
| `antenna_height_m` | Float | `1.800` | Physical distance from crossbar centerline to ground tip in meters. |
| `antenna_spacing_m` | Float | `0.555` | Measured physical distance between Master and Slave antenna phase centers. |
| `apply_lever_arm` | Boolean | `true` | Enables 3D kinematic projection to pole centerline and ground tip. |
| `pole_tilt_tare_pitch_deg` | Float | `0.0` | Recorded bracket pitch angle tare offset. |
| `pole_tilt_tare_roll_deg` | Float | `0.0` | Recorded bracket roll angle tare offset. |
| `e220_channel` | Integer | `75` | LoRa RF channel (`75` = `925.125 MHz`, clean US ISM band). |
| `e220_uart_baud` | Integer | `115200` | Baud rate between ESP32 and E220 transceiver. |
| `e220_air_rate_kbps` | Float | `19.2` | LoRa over-the-air data transmission rate (`19.2`, `9.6`, or `4.8` kbps). |
| `audio_nav_mode` | Integer | `1` | Proximity navigation audio mode (`0` = Beeps, `1` = Geiger Clicks, `2` = Mute). |
| `survey_avg_seconds` | Integer | `5` | Multi-epoch integration duration (`1`, `5`, `10`, or `30`). |
| `default_label_prefix` | String | `"PIN"` | Default prefix when not in `<DERIVED>` mode. |
| `active_waypoint_file` | String | `"/waypoints.csv"` | Active CSV waypoint file path. |
| `active_waypoint_index` | Integer | `0` | Active selected waypoint index (restored on boot). |
| `brightness` | Integer | `80` | Backlight brightness percentage (`20`, `40`, `60`, `80`, `100`). |
| `volume` | Integer | `40` | Audio volume percentage (`0` = Mute, `20`, `40`, `60`, `80`, `100`). |
| `auto_shutdown_battery_min` | Integer | `5` | Idle timeout on battery (`0`, `5`, `15`, `30`; inhibited by motion/`REC`/Wi-Fi/USB). |
| `beam_imu_orient_mode` | Integer | `4` | Beam `SEN0696` `3x3` axis alignment index (`0..7`, default `4` = Face-Down `(+X,-Y,-Z)`). |
| `mag_hard_iron_uT` | Float[3] | `[0,0,0]` | Hard-iron `[bx,by,bz]` offset solved by `CAL MAG` 360-deg ellipsoid fit (`uT`). |
| `mag_soft_iron` | Float[9] | Identity | `3x3` soft-iron correction matrix (row-major) from `CAL MAG` fit. |
| `mag_ellipsoid_cal_valid` | Boolean | `false` | `true` once `CAL MAG` 3D ellipsoid calibration has converged. |
| `mag_true_north_offset_deg` | Float | `0.0` | Declination / alignment tare continuously learned from `UM982 HDG FIX`. |
| `mag_field_ref_uT` / `mag_bz_ref_uT` | Float | `45.0 / 0.0` | Learned reference ambient magnetic magnitude and vertical Z component (`uT`). |
| `beam_pitch_tare_deg` | Float | `0.0` | Beam AHRS pitch offset learned against `UM982` moving-baseline pitch. |
| `apply_lever_arm_on_calibrated_beam` | Boolean | `false` | Opt-in flag enabling beam tilt lever-arm projection during single-antenna `RTK FIX`. |
| `rtcm_source_mode` | String | `"AUTO"` | RTCM input source policy (`"AUTO"`, `"LORA"`, `"NTRIP"`). |
| `ntrip_host` / `ntrip_host_ip` | String | `"rtk2go.com"` | NTRIP caster hostname and resolved IP address (Plan 2 BLE bridge). |
| `ntrip_port` | Integer | `2101` | NTRIP caster TCP port. |
| `ntrip_mountpoint` / `ntrip_user_email` / `ntrip_password` | String | `""` / `""` / `"none"` | NTRIP mountpoint and credentials (protected against default overwrite on save). |
| `line_log_max_m` | Float | `50.0` | Outer logarithmic cross-track span on Tile 0 Guidance canvas (`2.0..500.0` m). |
| `line_audio_fade_m` | Float | `20.0` | Off-line directional chirp silence cutoff in meters (`0.0` disables line-follow chirps). |
| `line_arrow_green_deg` | Float | `4.0` | Heading arrow Green lock angular window (`1.0..15.0` deg; `R -> B` prioritized). |
| `line_arrow_amber_deg` | Float | `15.0` | Heading arrow Orange near-lock angular window (`3.0..45.0` deg). |
| `map_zoom_m_per_px` | Float | `0.0` | Persistent Map View zoom in m/px (`0.0` = Auto-Fit; saved automatically on `+`/`-` tap). |

---

## 13. Troubleshooting & Field Diagnostics FAQ

### Q1: Antenna 1 shows `SINGLE` or `DGPS`, but never achieves `RTK FIX`.
* **Cause 1: Differential Link Down.** Check Row 2 `RTCM: X.Xs`. If age exceeds $5.0\text{ s}$ or reads `None`, the rover is not receiving RTCM packets. Inspect LoRa antenna connection and verify E220 RF channel on Tile 4 matches the Base Station transmitter (`Channel 75` / `925.125 MHz`).
* **Cause 2: Obstructed Sky.** Verify in Row 2 that satellite count used is $\ge 18$. Move away from heavy tree canopy or metal building walls.

### Q2: Row 3 shows `NO HDG` or `HDG FLT` while ANT1 has `RTK FIX`.
* **Cause:** The secondary antenna (ANT2) requires clear sky view to resolve carrier-phase integer ambiguities along the $55.5\text{ cm}$ baseline. Ensure the Slave antenna cable is firmly connected to UM982 RF2 and the crossbar is held relatively level.

### Q3: When rotating the survey pole in place, the coordinates shift.
* **Cause:** Kinematic lever-arm compensation is disabled. Check Tile 4 IMU block to ensure `Lever-Arm: ON`. Verify in `settings.json` that `antenna_spacing_m` is set to `0.555` and `antenna_height_m` matches your physical pole height (`1.800 m`).

### Q4: The spirit bubble is not centered when the pole is held plumb.
* **Cause:** The display is mounted on an angled bracket. Hold the survey pole plumb using a physical bubble vial, navigate to Tile 3, and tap **`TARE`**. The bubble will center at $(0.0^\circ, 0.0^\circ)$ and store the bracket compensation offset permanently.

### Q5: How do I back up my survey data in the field?
* Tap `WIFI` on Tile 5, connect with your phone, open `http://<rover-ip>/`, tap `/survey_logged_pins.csv`, and save to your mobile device. You can also power down the rover and remove the MicroSD card to read it directly on a computer.

### Q6: What happens when I tap `WIFI: OFF` on Tile 5, and how do I diagnose router connection issues?
* **Connection Sequence:** Tapping `WIFI: OFF` (Navy Blue) changes the button to `WIFI: CONN` (Amber) while scanning all 2.4 GHz channels for the `wifi_ssid` in `/settings.json` (up to 20 seconds).
  * **Router Found (`STA`):** Button turns Emerald Green (`WIFI: STA`) and displays `Portal: http://<router-ip>/`.
  * **Field / Away from Router (`AP` Fallback):** If the configured router is out of range (or unconfigured), the Rover automatically launches its standalone SoftAP (`SSID: RTK-Rover`, password `12345678`), turns the button Violet (`WIFI: AP`), and serves the portal at `http://192.168.4.1/`.
* **Serial Diagnostics:** Check the USB Serial log (or `/serial_logs/` on MicroSD) for `[MEM] Post-Setup Heap` and `[WIFI] WiFi.mode(WIFI_STA) -> OK`. Note that ESP32-S3 supports **2.4 GHz** (`802.11b/g/n`) networks only (not 5 GHz-only SSIDs).

---

## 14. Bi-Directional LoRa Protocol & Base Station Integration

The RTK Rover implements a bi-directional telemetry and remote control protocol operating over the Ebyte E220 900 MHz LoRa link (`e220_pacer.py v3.7.1` + `v0.9.0`).

### 14.1 Content-Independent Macro-Cycle & Half-Duplex Uplink Slot
* Base Station (`e220_pacer.py v3.7.1`) streams continuously across `Seconds 1..(N-1)` of the GPS-disciplined macro-cycle (`--macro-cycle-s`, default `5 s`, optional `10 s`) with zero per-second RSSI pauses.
* During `Second 0` (`UTC_Second % 5 == 0`), Base enforces a rate-aligned TX cutoff (`450 ms` @ `4.8k`, `620 ms` @ `9.6k`, `680 ms` @ `>= 19.2k`) before opening the half-duplex Rover uplink slot (`500..660 ms` @ `4.8k`, `670..760 ms` @ `9.6k`, `730..810 ms` @ `>= 19.2k`), followed by one post-uplink Mode 0 RSSI query (`680..920 ms`).

### 14.2 48-Byte V3 Uplink Telemetry Frame
* **Sync Preamble:** `0xAA 0x55` (distinct from RTCM3 `0xD3` to prevent false-framing).
* **Payload (42 Bytes):**
  * Sub-millimeter ground-tip coordinates (`tip_lat`, `tip_lon`, `tip_height_m`) and horizontal accuracy (`h_acc_mm`).
  * Master ANT1 fix quality (`RTK_FIX`, `RTK_FLT`, `DGPS`, `SINGLE`, `NONE`).
  * Slave ANT2 fix quality (`SLV_FIX`, `SLV_FLT`, `SLV_DGPS`, `SLV_SGL`, `NONE`).
  * Measured 3D inter-antenna baseline length (`baseline_mm`, nominal $555\text{ mm}$ / $55.4\text{ cm}$).
  * Measured crossbar pitch angle (`baseline_pitch_cdeg`, centidegrees).
  * Explicit heading source tag (`DUAL_FIX`, `DUAL_FLT`, `WALK`, `GYRO`, `COG`, `HOLD`, `COMP`).
  * Master & Slave satellite counts, True Heading centidegrees, battery percentage and millivolts, Rover packet RSSI, live ambient noise floor, sequence counter, and firmware SemVer (`fw_major.fw_minor.fw_patch`).
* **Integrity:** Protected by a 16-bit CRC16-CCITT checksum.

### 14.3 Touchscreen Remote 3-Rate Air-Rate Switching
* **Field Problem:** Deep timber or dense foliage can cause fringe RF packet loss at $19.2\text{ kbps}$. Switching down to $9.6\text{ kbps}$ (`+3 dB`) or $4.8\text{ kbps}$ (`+5 dB`) extends range cleanly.
* **Procedure:** Tap Tile 5 `LORA` button to cycle `19.2k / 9.6k / 4.8k` (or tap again before Second 0 TX to cancel).
* **Zero-Credential 4-ACK Negotiation:** Rover transmits `CMD_REQ_RADIO_SWITCH` (`0x02`) in `Second 0`, Base replies with a 4-slot spaced `ACK_RADIO_SWITCH` train (`80 / 260 / 440 / 620 ms`) in `Second 1`, and both radios reconfigure in Mode 3 at `Second 2` before probing Mode 0 liveness.
* **Dynamic Pre-Switch Failsafe Rollback:** If no valid traffic is verified within 60 seconds after a rate switch, both Rover and Base automatically revert to the pre-switch verified fallback air rate.

### 14.4 Base Station Pacer (`e220_pacer.py v3.7.1`)
* **Real-Time Terminal Output:** Displays live Rover telemetry lines with full ANT1 and ANT2 observables formatted with clean ASCII `deg` units and USB/Battery power state:
  ```text
  [ROVER TEL #042] FW:v0.8.12 | ANT1:RTK_FIX (28M) | ANT2:SLV_FIX (26S, BL:55.4cm, P:+0.2deg) | Hdg:142.6deg [DUAL_FIX] | PWR:USB (4.02V) | RSSI:-84dBm | Pos:43.7319105, -70.6237371, 12.45m
  ```
* **Tri-Format Backward-Compatible Decoder:** Supports 48-byte V3, 45-byte V2, and legacy 39-byte uplink frames seamlessly.
* **Atomic RAM State (`/run/rtk_rover_status.json`):** Writes live JSON on RAM tmpfs including `rover_fw_version`, `ant2_fix_name`, `baseline_length_cm`, `baseline_pitch_deg`, and `heading_source`.
* **Rolling Daily CSV Log (`~/rover_logs/rover_YYYY-MM-DD.csv`):**
  * Logs extended columns (`timestamp_utc, seq, rover_fw, fix_type, fix_name, ant2_fix, baseline_mm, pitch_deg, hdg_source, latitude, longitude, height_m, h_acc_mm, sats_master, sats_slave, heading_deg, battery_pct, battery_mv, rover_rssi_dbm, rover_noise_dbm, air_rate_kbps, channel`).
  * Auto-prunes files older than 30 days (`--log-retention-days 30`), caps total log directory footprint at $50\text{ MB}$ (`--max-log-mb 50`), and compresses yesterday's logs with `gzip` (`rover_YYYY-MM-DD.csv.gz`).

### 14.5 Asynchronous Ping-Pong Mailboxes & Smart `<= 200 B` Coalescing
* **Zero Epoch Queue Latency:** A dedicated background ingress thread continuously drains incoming RTCM3 bytes from RTKBase, verifies `CRC24Q`, drops empty MSMs (`<= 28 B`), and overwrites latest-value ping-pong mailboxes keyed by message type (`2.5 s` dynamic MSM / `15.0 s` static TTL). Saturated links never accumulate stale queued seconds.
* **Starvation-Fair Round Robin & Coalescing:** Prioritizes static base metadata (`1005/1006/1033/1230`), rotates multi-constellation MSM frames (`1074/1084/1094/1124`) cleanly by starvation age (`now - last_tx` with GPS `1074` tie-breaker), and packs compatible small frames into a single `<= 200 B` Regime A burst (e.g., `1006+1230+1084`).
* **Heartbeat & CLI Controls:** Heartbeat displays `Mbox:[Sup:<n> Coal:<n> Stale:<n> MaxAge:<ms>]`. Supports `--macro-cycle-s {5,10}` (default `5`) and `--no-interleave` CLI flags.
