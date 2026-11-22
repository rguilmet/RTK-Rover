# RTK Rover - Field Operator's User Manual

**Document Version:** `v1.9.0`  
**Firmware Version:** `v0.9.0`  
**Last Updated:** November 22, 2026  
**Hardware Platform:** Waveshare ESP32-S3-Touch-LCD-3.5B, Holybro H-RTK Unicore UM982, Ebyte E220-900T30S, DFRobot SEN0696 10-DOF Beam IMU  

---

## Table of Contents
1. [Introduction & System Overview](#1-introduction--system-overview)
2. [Hardware Setup & Power-On Procedure](#2-hardware-setup--power-on-procedure)
3. [Persistent 4-Row Top Status Header Bar](#3-persistent-4-row-top-status-header-bar)
4. [Tile 0: Pin Seeker & 3-Mode Tactical Guidance (Point / Line+Bullseye / Map)](#4-tile-0-pin-seeker--3-mode-tactical-guidance-point--linebullseye--map)
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

### Navigation Modes on Tile 0

Tile 0 features an interactive tri-state navigation badge in the top-left corner of the canvas. Tapping the badge cycles through three specialized navigation modes:
```
[POINT] (Cyan) -> [LINE STAKEOUT / LINE BULLSEYE] (Amber) -> [MAP VIEW] (Emerald Green) -> [POINT]
```

---

### Mode 1: Unified Line + Bullseye Stakeout (Primary Boundary Mode)

![Tile 0: Line Stakeout Mode](docs/images/screenshot_tile0_line_stakeout.png)
*Figure 2C: Mode 1 Line Stakeout showing Point A to Point B deed baseline, perpendicular offset tie-line, stationing, and ASCII quadrant bearing.*

When the mode badge is set to Mode 1 (`LINE STAKEOUT` -> `LINE BULLSEYE`, Amber `#D97706`):
* **Baseline Definition (`A -> B`):** Directs the surveyor along the boundary line between Point A (Station `0+00`) and Point B (Target).
  * **Auto-Pairing & Clean Step Advance:** By default, Point B is the active target (`Index`), and Point A is `Index - 1` (wrapping around to close parcels). Stepping `< Prev` or `Next >` automatically clears `[SWAP]` pairing so `A -> B` advances to the new leg.
  * **One-Tap Swap (`[<-> SWAP]`):** Tap the target row label (`A -> B [SWAP]`) to reverse line direction cleanly in place.
* **Hysteresis Transition Around Target `B` (`5.0 m` / `16.4 ft`):**
  * **Line-Up Phase (`> 5.0 m`, exit hysteresis `> 5.3 m`):** Shows the vertical baseline pulled inward (`y = 46..178 px`, horizontal clamp `[-104, +104] px` leaving `48 px` margins on both sides), perpendicular tie-line, compact distance/bearing cue to `B`, and **Line-Up heading arrow** rotated cleanly to `true_heading_deg - line_bearing_deg`. `t0_tgt_hdg_lbl` displays `Line: XXX.X deg`.
  * **Bullseye Finish Phase (`< 4.8 m` acquisition chime -> `LINE BULLSEYE`):** Automatically transitions to centered concentric target rings (`5.0 m` outer / `1.5 m` inner, auto-zooming to `1.5 m / 0.5 m` inside `1.5 m`) **with the `A -> B` property chord overlay drawn straight through the target**, giving simultaneous pin range and line offset!
* **Signed Directional `L/R` Audio (`ES8311`):**
  * During Line-Up (`> 4.8 m`), left-of-line (`offset_m < -0.05 m`) plays a **rising chirp (`660 -> 1100 Hz`, steer RIGHT `>>>`)**, right-of-line plays a **falling chirp (`1100 -> 660 Hz`, steer LEFT `<<<`)**, and on-line (`<= 0.05 m`) plays a **steady `1800 Hz` pip**, speeding up from `1500 ms` to `250 ms` as you approach the line. Crossing inside `4.8 m` emits a `1400 -> 2000 Hz` acquisition chime and hands off to Bullseye proximity audio.
* **Scaled 4-Row Readout Deck Below Canvas:**
  * **Row 1:** `[OK] ON LINE (0.01 ft)` green or `X.XX ft RIGHT` / `LEFT` amber (unsigned magnitude).
  * **Row 2:** `<<< MOVE LEFT TO LINE` / `MOVE RIGHT TO LINE >>>` / `[OK] ON LINE - PROCEED ON BEARING`.
  * **Row 3:** `Sta: 0+78.6 ft (26.8%) | Rem: 215.1 ft`.
  * **Row 4:** Pure ASCII cadastral bearing (`Deed: N 56d00'12" W (304.0 deg)`).

---

### Mode 2: Map View (Cadastral 2D Canvas with Zoom, Pan & Tie-Lines)

![Tile 0: 2D Cadastral Map View](docs/images/screenshot_tile0_map_view.png)
*Figure 2D: 2D Cadastral Map View rendering closed parcel boundary polygon, dashed reference tie-lines, HUD zoom controls, and rover heading beam.*

When the mode badge is set to `MAP VIEW` (Emerald Green `#10B981`):
* **Geodetic Tangent-Plane Canvas:** Projects real-world WGS84 coordinates onto a 2D local equirectangular plane centered on the map viewport.
* **Interactive Zoom & Pan:**
  * **Single-Touch Drag Pan:** Touch and drag anywhere on the map canvas to smoothly pan the view in real-time.
  * **HUD Zoom Buttons (Canvas Right Edge):**
    * `[+]`: Zoom in (magnifies view by 30%, down to 0.05 m/px).
    * `[-]`: Zoom out (expands view by 40%, up to 30.0 m/px).
    * `[FIT]`: Auto-fit bounding box encompassing all loaded survey waypoints and the current rover location.
    * `[CTR]`: Center map viewport directly on the live rover ground-tip coordinates.
* **Rendered Cadastral Map Layers:**
  * **Closed Boundary Polygon:** Connects all parcel boundary vertices with positive sequence numbers ($1 \rightarrow 2 \dots \rightarrow N \rightarrow 1$) using a solid Sky Blue polyline.
  * **Dashed Cadastral Tie-Lines:** For external reference / tie points with negative sequence numbers ($-k$), draws a distinct dashed line directly connecting the tie point to parcel boundary vertex $k$.
  * **Differentiated Pin Symbology:**
    * **Boundary Vertices:** High-visibility Gold pins (`#FBBF24`).
    * **Reference / Tie Pins:** High-visibility Orange pins (`#F97316`).
    * **Active Target:** Prominent green bullseye icon over the selected waypoint.
    * **Live Rover:** Bright cyan position circle with a forward-projecting true heading beam.
  * **128-Point Route Trail:** Renders the surveyor's historical movement trail across the parcel. Can be enabled or disabled via `"map_show_route_trail"` in `settings.json`.
* **Readouts Below Canvas:**
  * **Row 1:** Target distance and true bearing (`14.28 m @ 042.5 deg`).
  * **Row 2:** Active target ID, index, and note (`TGT: P3 (3/8) - Corner Iron`).
  * **Row 3:** Viewport center geodetic coordinates (`Map Ctr: 43.73248, -70.62408`).
  * **Row 4:** Viewport scale and loaded waypoint count (`Scale: 0.50 m/px | Pts: 8`).

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
| `minutesOffsetToUTC` | Integer | `-240` | Local timezone offset from UTC in minutes (`-240` = EDT / UTC-4, `-300` = EST / UTC-5). |
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
