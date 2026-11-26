# RTK Rover - Field Operator's User Manual

**Document Version:** `v1.12.0`  
**Firmware Version:** `v0.12.0` (`Field_Instrument_Core >= v1.2.1`)  
**Last Updated:** November 25, 2026  
**Hardware Platform:** Waveshare ESP32-S3-Touch-LCD-3.5B, Holybro H-RTK Unicore UM982, Ebyte E220-900T30S, DFRobot SEN0696 10-DOF Beam IMU (`CAD v2.2.4`)  

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

The **RTK Rover** is a dual-antenna, high-precision centimeter GNSS rover engineered for boundary surveying, property monument recovery, line stakeout, and ground-control logging.

```text
                           +------------------------+
                           |  GNSS Master Antenna   | (ANT1 - Position)
                           +-----------+------------+
                                       |
                           <---- 27.75 cm ---->
                                       |
    +----------------------------------+----------------------------------+
    |                     Rigid T-Bar Crossbar (55.5 cm)                  |
    +----------------------------------+----------------------------------+
                                       |
                                       | <---- 27.75 cm ---->
                                       |
                           +-----------+------------+
                           |   GNSS Slave Antenna   | (ANT2 - True Heading)
                           +------------------------+
                                       |
                                       | (Pole Centerline + SEN0696 Beam IMU)
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
* **Rigid Moving-Baseline:** Two multi-band GNSS antennas sit `55.5 cm` apart along a rigid crossbar. ANT1 (Master) solves carrier-phase RTK position; ANT2 (Slave) solves dual-antenna moving-baseline azimuth and beam pitch.
* **Exact Midpoint Kinematics:** The survey pole centerline sits at the midpoint (`27.75 cm`) between ANT1 and ANT2. Firmware projects coordinates along the 3D baseline vector and down `1.800 m` to the ground tip (`< 8 mm` orbit across `360 deg` pole rotation).

---

## 2. Hardware Setup & Power-On Procedure

1. **Mounting:** Clamp the crossbar cleanly to the survey pole (`ANT1` left, `ANT2` right when viewing from behind the display) and connect the external `GX-12` `SEN0696` beam IMU lead cleanly into the lower bulkhead (`I2C1 GPIO 47/48`).
2. **Antenna & LoRa Connections:** Connect Master RF1, Slave RF2, and the 900 MHz LoRa antenna onto E220 SMA before powering on.
3. **Power-On:**
   * **Internal Battery (`MT3608` 5V Boost Mode):** Operates untethered on `1S` Li-Ion/LiPo (`~806 mA` / `~3.0 W` typical draw @ `3.7 V` with `80%` brightness & `Wi-Fi OFF`: ~3.2-3.4 hrs on `3,000 mAh` or ~10.6-11.2 hrs on `10,000 mAh`; see [Power Budget](schematics/system_wiring_schematics.md#3-system-power-budget-mt3608-boost-analysis--battery-runtime-estimates)). For long battery walks without active `REC ON`, set `AUTO-OFF` on Tile 5 (defaults to `5m`, automatically inhibited by motion, `REC ON`, survey, `CAL MAG`, or Wi-Fi).
   * **External USB Mode:** Connect external USB 5V power while floating/charging the internal cell at `500 mA` via `AXP2101`.
4. **Boot Sequence:**
   * Initializes MicroSD (with internal `FFat` fallback), loads/migrates `/settings.json` (Schema v2), runs `E220` `M0`/`M1` continuity checks and `UM982 COM1` state audit (`VERSIONA`, `MODE`, `CONFIG`, `LOGLIST`).
   * Core 0 validates LoRa `CRC24Q` RTCM3 frames and forwards them to **`UM982 COM1` (`GPIO 17/18`)**, while Core 1 polls `I2C0` + isolated `I2C1` (`SEN0696`) and renders LVGL 9 at `30 Hz`.
5. **Acquisition Checklist:**
   * Verify Row 1 power (`[USB] 4.85V+` / `[Bat] ...`), Row 2 `RTK FIX` (Green), Row 3 `SLV FIX` (or calibrated `HDG CMP` under canopy), and baseline `BL: ~55.5cm`.

---

## 3. Persistent 4-Row Top Status Header Bar

The top `74 px` remain visible across all `6` tiles (`[Tile 5: Settings] <-> [Tile 0: Nav] <-> [Tile 1: Survey] <-> [Tile 2: Waypoints] <-> [Tile 3: Compass] <-> [Tile 4: Diag]`) -- capture full-screen reference BMPs via `BOOT >= 1.5 s` (stored under `docs/images/`):

### Row 1: System Time, Power & Environment
* **Date & Time:** Local date (`YYYY-MM-DD`) and 24-hr clock (`HH:MM:SS TZ`) disciplined by GNSS UTC.
* **Units Badge:** `SI` (`m`/`cm`) or `IMP` (`ft`/`in`); toggle via Tile 5 `UNITS`.
* **Wi-Fi Icon:** Cyan (`STA`), Violet (`AP`), Amber (`CONN`), or Slate (`OFF`).
* **Power Telemetry:** `[USB] 4.85V+` (charging), `[USB] 4.85V` (full/float), or `[Bat] 85% 3.92V` (Green `>60%`, Amber `20-60%`, Red `<20%`).

### Row 2: Master Antenna (ANT1) & LoRa Link
* **Fix Badge:** `RTK FIX` (Green `< 2 cm`), `RTK FLT` (Yellow), `DGPS` (Orange), `SINGLE` (Red), or `OFF`.
* **Satellites / Precision / RTCM Age / LoRa:** `SV: used/view`, `+/- h_acc`, `RTCM: X.Xs`, and `LoRa: -XXdB` (Green `>= -90 dBm`, Yellow `-91..-105`, Red `< -105`).

### Row 3: Slave Antenna (ANT2) & Woods Heading Fallback
* **Heading Badge:** `SLV FIX` (dual-antenna fixed `0.2 deg`), `SLV FLT` (float), `HDG CMP` (`SEN0696` `I2C1` 9-DOF or `MMC5983MA`, `0.0 s` handoff), `HDG WALK` (`>= 0.70 m` steps), `HDG GYRO` (iron coast / turn), `HDG HLD` (stationary freeze), `NO HDG`, or `OFF`.
* **Slave SVs, Baseline & Heading:** `SV: XX`, `BL: 55.4cm`, `Hdg: XXX.X SRC`.

### Row 4: Ground-Tip Geodetic Coordinates
* High-contrast `14pt` live Lat, Lon, Height of the pole ground tip (`0.0000000, 0.0000000, 0.000 m`).

---

## 4. Tile 0: Unified Guidance Triangle, Auto-Bullseye & Map View

Tile 0 (`ui_tile0_guidance.c`, `ui_tile0_views.c`, `ui_tile0_map.c`) is the primary field navigation screen (`v0.10.7`). Tap the top-left mode badge inside the `304 x 276 px` canvas to toggle between **`[GUIDANCE]`** and **`[MAP VIEW]`**:

```text
[GUIDANCE] (Log-Scale Triangle + Auto < 5 m Bullseye)  <--->  [MAP VIEW] (Sticky Zoom + Active Segment + BS Base)
```

### Target Bar, Anchor Modes & Single-Row Canvas HUD
* **Target Bar (`A -> B`):** Tap `<` / `>` to step CSV waypoints. Tap the center target label (`[SWAP]`) to reverse `A <-> B` in Deed mode, or tap `[SET A]` in Direct mode to re-anchor Start `A` at your current GNSS spot.
* **Consolidated Marquee Line (`t0_notes_lbl`):** Auto-scrolls circularly below the target bar showing `Deed:`/`Direct:` quadrant bearing (`N/S xx deg xx'xx" E/W`), azimuth, line length, and outer log span (`Log: 164.0ft`) in Guidance, or `SEG: A -> B | Win: ... | Pts: N` in Map View.
* **Single-Row Canvas Top HUD (Shared by Both Modes):**
  * Left: **`[GUIDANCE]`** / **`[MAP VIEW]`** mode toggle badge.
  * Center: **`H:XXX.X P:+X.X`** (True Heading + **Dual-Source Beam Pitch**: displays UM982 `baseline_pitch_deg` when `SLV FIX` or `SLV FLT` is active, automatically falling back to tare-compensated `SEN0696` 10-DOF beam pitch whenever dual-antenna heading is inactive).
  * Right: **`B: XXX.X`** direct azimuth to Target `B` (Green `<= 4 deg`, Amber `<= 15 deg`, White otherwise).

---

### Mode 0: Unified Guidance (Log-Scale Triangle -> `< 5 m` Auto-Bullseye)

#### Phase A -- Line-Up Log-Scale Triangle (`> 5.0 m` from Target `B`, exit hysteresis `> 5.3 m`)
* **Dynamic Vertical Auto-Framing (`y = 50..206 px`):** Baseline `A -> B` stays vertically aligned in center with generous HUD and bottom-button margins:
  * **Between `A` and `B`:** `A` (`12 px` Amber/Violet) pinned near bottom, `B` (`16 px` Green) near top, `R` (`14 px` Sky Blue, Green on-line) climbing linearly.
  * **Behind `A` or Past `B`:** Canvas dynamically slides `A` or `B` inward and renders a **dashed baseline extension** to Rover `R`'s perpendicular foot so the full `A-B-R` triangle stays framed on screen.
* **Symmetric Log Cross-Track Grid (`line_log_max_m = 50.0 m`):** Vertical reference grid lines and tick labels (`3ft` / `30ft` Imperial or `1m` / `10m` SI) keep `2 cm` sensitivity near the line while fitting obstacles up to `50 m` off-line.
* **Triangle Legs & Prioritized Heading Arrow:** Renders perpendicular tie (`R -> Line`), direct hypotenuse (`R -> B`), and a barbed heading arrow from `R` (passing cleanly behind bottom buttons if aimed downward). **Hypotenuse `R -> B` locks take color priority** (`<= 4 deg` Green / `<= 15 deg` Orange), lighting up `R -> B` first or lighting the perpendicular tie if turning back to the line.
* **3-Tier Vector Steering Chevrons & Dynamic `A`/`B`/`R` Labels:**
  * Side margins render graphic **`>>>`** (center-left, steer RIGHT) or **`<<<`** (center-right, steer LEFT) chevrons (`Red-Orange` outside `line_audio_fade_m`, `Amber` closing in, `Green` when aimed squarely at the perpendicular tie), switching to dual **`|`** green bars when `<= 2 cm` on-line.
  * Crisp `A`, `B`, and `R` marker labels automatically keep opposite the crossing baseline side (`A`/`B`) and step into the clearest 8-way sector around `R` away from active lines, chevrons, and neighbor labels.
* **Acoustic Fade-Out `L/R` Audio (`ES8311`):** Left-of-line plays rising chirps (`660 -> 1100 Hz`), right-of-line plays falling chirps (`1100 -> 660 Hz`), speeding up (`2500 -> 220 ms`) and ramping gain (`25% -> 100%`) as you approach center, and fading silent past `line_audio_fade_m` (`20.0 m`).
* **2-Row Bottom Readout (`308 px` circular marquee):**
  * **Row 1:** `<offset> LEFT/RIGHT | B: <straight-line dist to B>` (or `[OK] ON LINE | B: <dist>`).
  * **Row 2 (Signed Along-Line Progress, `v0.10.7`):** `A: +|-x.x <unit> (x.x%) | Line->B: +|-x.x <unit>` showing signed along-line distance from Start `A` (`-` behind `A`, `+` forward from `A`) and remaining along-line distance to Target `B` (`+` approaching `B`, `-` overshot past `B`).

#### Phase B -- Auto-Bullseye Centimeter Finish (`< 4.8 m` entry chime -> `BULLSEYE`)
* Emits a `1400 -> 2000 Hz` chime inside `4.8 m` and switches to centered concentric rings (`5.0 m / 1.5 m`, auto-zooming to `1.5 m / 0.5 m` inside `1.5 m`) **with the `A -> B` property chord overlaid through `B`**, dynamic `B`/`R` labels, North-Up barbed heading arrow, and `Geiger`/`Beeps` (`2400 Hz` pin-lock tone at `<= 2 cm`).
* **Start Anchor Button (`A: DEED` / `A: HERE`):** Bottom-left button switches between CSV leg `A -> B` and Direct anchor `HERE -> B` (`auto` on single-waypoint CSVs).

---

### Mode 1: Map View (Sticky Zoom, `BS` Base Symbol & Bottom Toolbar)

Tap `[GUIDANCE]` -> **`[MAP VIEW]`** (Emerald Green `#10B981`):
* **Unified Bottom Toolbar (`26 px` tall):** Bottom edge houses full-width zoom/pan and logging buttons (`[+]`, `[-]`, `[FIT]`, `[CTR]`, and `[REC OFF/ON]`), leaving the upper parcel area completely unobstructed.
* **Persistent Sticky Zoom (`map_zoom_m_per_px`):** Manual `[+]` (`-30%`) and `[-]` (`+40%`) zoom levels persist across mode switches, CSV switches, and reboots until `[FIT]` resets `0.0`. Drag the map canvas anytime to pan; tap `[CTR]` to center on live Rover `R`.
* **Survey Layers & Dynamic `A`/`B`/`R`/`BS` Labels:**
  * Closed boundary polygon (`poly_order > 0`, Gold pins), dashed Orange tie-lines (`poly_order < 0`, Orange pins), optional breadcrumb trail (`map_show_route_trail`), highlighted active followed segment (`4 px` solid Emerald `DEED` or `3 px` dashed Violet `HERE`), barbed Rover heading arrowhead, and the **Survey-Control Base Station symbol (`BS` Magenta hollow triangle + white center dot plotted from live `BASEINFOA`, excluded from `FIT`)** -- all with dynamic non-overlapping `A`, `B`, `R`, and `BS` labels (`v0.10.6`).
* **Bottom 2-Row Readout:** Row 1 shows direct distance & bearing to Target `B`; Row 2 shows live Map Center coordinates (`Map Ctr: lat, lon`).

---

## 5. Tile 1: Dedicated Field Survey Logger

Tile 1 (`ui_tile1_survey.c`) isolates survey logging from quick settings so field monument logging stays fast and glare-proof (`40 px` dropdowns, `12` monument tags, `52 px` `[LOG]` button, and pre-log quality card):

* **Row 1 (`40 px` tall):** Prefix (`<DERIVED>` uses active target stem e.g. `PIN_NW_01 -> PIN_NW_01_L001`, plus `PIN`, `PIPE`, `MONUMENT`, `STAKE`, `CORNER`, `FENCE`, `TREE`, `UTILITY`, `WP`) and Averaging (`Avg: 1s -> 5s -> 10s -> 30s`, using `atan2` circular heading averaging and real Core 1 `pole_tilt_deg` capture into `/survey_logged_pins.csv`).
* **Row 2 (`40 px` tall, `12` `kNoteTags`):** `(No Tag)`, `Found Iron Pipe`, `Found Capped Rebar`, `Found Stone Mon`, `Found Mag Nail`, `NO PIN - Search Pt`, `NO PIN - Marked Spot`, `Unknown Pin/Pipe`, `Set Temp Flag`, `Mag Anomaly Only`, `Obstructed Sky`, `Needs Verification`.
* **Row 3-5:** Collision-free `ID: <next_label>` marquee, `0-100%` progress bar, `300 x 52 px` active-tag `[LOG]` button (`Sky Blue` for `NO PIN` entries), and live Pre-Log Survey Quality card (`Fix/SV/HDOP/H/V-Acc/Tilt/dN/dE`).

---

## 6. Tile 2: Multi-File Waypoint Browser & Selector

Tile 2 (`ui_tile2_waypoint_browser.c`) scans `/waypoints.csv` and all `/waypoints_*.csv` on MicroSD/`FFat` via a multi-file selector bar with circular marquee and interactive waypoint table:

* Tap `< Prev File` / `Next File >` or tap any waypoint row (`< 5 ms` `512 B` chunked load, `64`-entry stem cache, `< 1 ms` `RoverAudio`, `0 ms` same-frame Tile 0 readout recomputation). Both active file and selected waypoint index persist across reboots.
* Supports `19`-column legacy CSV and **`20`-column cadastral CSV** (`poly_order > 0` boundary vertices Gold, `< 0` tie points Orange `-k`, `0` standalone pins).

---

## 7. Tile 3: Dual-Antenna & Beam 9-DOF Compass, Plumb Bubble Level & `CAL MAG`

Tile 3 (`ui_tile3_compass.c`) displays heading provenance, Beam 10-DOF status (`SEN0696` on `I2C1 GPIO 47/48`), `224 px` spirit bubble level, `CAL MAG`, `TARE`, and `Baro Rel Elev`:

* **Heading Readout:** `24pt` True Heading with source provenance (`DUAL FIX`, `DUAL FLT`, `BEAM 9DOF`, `WALK`, `GYRO COAST`, `HOLD`) and `[IRON ANOMALY]` warning whenever localized metal triggers gyro coasting.
* **360-Deg Ellipsoid `CAL MAG` (Left Button):** Collects `300` samples into PSRAM while smoothly rotating `360 deg` with gentle pole tilt (`16` sectors), falling back cleanly to regularized sphere fit + `BZ_ref` self-heal on flat spins, and persisting `mag_hard_iron_uT` / `mag_soft_iron` to `settings.json`. Auto-Off is inhibited during `CAL MAG`.
* **Plumb `TARE` (Right Button):** Hold pole physically plumb via a physical vial and tap **`TARE`** to zero pitch/roll bubble offsets into `settings.json`.

---

Tile 4 (`ui_tile4_diagnostics.c`) renders the pinned diagnostic header deck, LoRa RF survey card, and live subsystem observer blocks:

* **Pinned Deck:** Fixed firmware version header (`RTK ROVER v0.10.7`), calibrated LoRa RF card (`Noise`, `Pkt`, `AUX` edges, `RTCM3` counters, `Err/Trunc`), and `Auto-Off` idle countdown/inhibit reason.
* **Live Cards:** Dual **`I2C0 / I2C1` error counters**, **`SEN0696` 10-DOF & `QMI8658` Pose** (`Orient`, `FOC`, `3D-Cal`, `BMP581` + `Baro Rel Elev`), `Storage`, `ANT1`, `ANT2`, `Base Station` (`ECEF -> WGS84`), `UM982 COM1`, `UPS` / `MT3608`, and `E220` pin self-test (`M0:OK M1:OK`).

---

## 9. Tile 5: Dedicated `5x3` Quick Settings Deck & Safe Auto-Off

One tap `< Prev` from Tile 0 opens **Tile 5 (`ui_tile5_settings.c`)** with `15` large `94 x 44 px` controls above a live Wi-Fi / Auto-Off / LoRa footer:

| Column 1 (Left) | Column 2 (Center) | Column 3 (Right) |
| :--- | :--- | :--- |
| **Row 1:** `VOL: 20%..100%` / `MUTE` | **Row 1:** `AUD: GEIGER / BEEPS / MUTE` | **Row 1:** `BRI: 20%..100%` |
| **Row 2:** `SERIAL: ON/OFF` | **Row 2:** `NMEA: ON/OFF` | **Row 2:** `RTCM: ON/OFF` |
| **Row 3:** `TELEM: ON/OFF` | **Row 3:** `IMU: ON/OFF` | **Row 3:** `TOUCH: ON/OFF` |
| **Row 4:** `CAL MAG` (`OK` Green / Ready) | **Row 4:** `UNITS: IMP/SI` | **Row 4:** `AUTO-OFF: 5m/15m/30m/OFF` |
| **Row 5:** `LORA: 4.8k / 9.6k / 19.2k` | **Row 5:** `COMMS: BLE / WIFI / OFF` | **Row 5:** `RTCM SRC: PHASE 2.4` *(Reserved)* |

* **Unified `COMMS` 3-State Radio Cycle (`BLE -> WIFI -> OFF -> BLE`):** Field default starts **`BLE: ADV`** (`"RTK-Rover"`, `MTU 517`) with Wi-Fi OFF to conserve internal `DRAM0`. Tapping `COMMS` advances cleanly to **Wi-Fi Web Portal** (suspending NimBLE before `WiFi.mode()`): Amber `WIFI: CONN` (tap `COMMS` again `<= 100 ms` to cancel straight to `COMMS: OFF`) -> Emerald `WIFI: STA` (`http://<local-ip>/`) -> Violet `WIFI: AP` (`http://192.168.4.1/`, SoftAP creds from `/settings.json`). Tapping `COMMS` while Wi-Fi is active stops Wi-Fi cleanly into **`COMMS: OFF`**; tapping again restarts **`BLE: ADV`**. When an Android phone enters `slave_dimmed` mode, a dimmed overlay renders `[LOCAL CONTROL]` for instant one-tap reclaim (with a `5 s` heartbeat watchdog fail-safe).
* **Motion/REC-Safe `AUTO-OFF` Guard:** Automatically inhibited by external USB power, Wi-Fi, active `BLE Link`, active Survey or `CAL MAG`, `REC ON` route track logging, `NMEA`/`RTCM` logging, GNSS/IMU motion, or touch/`BOOT` input. When an idle timer expires, a full-screen `lv_layer_top()` **`30 s` countdown modal (`TAP ANYWHERE TO CANCEL`)** alerts with beeps before stopping/flushing all SD logs cleanly and cutting `AXP2101` power.

---

## 10. Hardware Screenshot Engine (`v0.10.3 - v0.10.4`)

* **Trigger:** Long-press the physical **BOOT** button (`GPIO 0`) for `>= 1.5 seconds` (short press is ignored so screenshots never accidentally trigger waypoint logs).
* **Lock-Free Framebuffer Capture:** Captures the active `320 x 480` PSRAM framebuffer via a non-contending handshake inside `lvgl_port_core.c` and encodes top-down 24-bit TrueColor BMP using a `2`-row (`1,920 B`) internal DMA chunk buffer under `fs_mux`.
* **Storage & Verification:** Writes directly to MicroSD `/screenshots/screenshot_YYYYMMDD_HHMMSS.bmp` (falling back automatically to internal `FFat` if SD fails), then performs mandatory read-only post-`close()` size verification (`460,854 B`).
* **Audio Feedback:** Plays a crisp **`2600 Hz` chime** on verified capture, or a **`400 -> 300 Hz` error tone** (with `errno` + free-space serial log) on failure.

---

## 11. Wireless Web File Portal & Field Maintenance

1. Tap `COMMS` on Tile 5 (Row 5) to cycle from `BLE: ADV` to Wi-Fi (`WIFI: CONN` -> `STA` or `AP`). Connect your phone or laptop to the local STA network (or SoftAP configured by `ap_ssid`/`ap_password` in `/settings.json`, `http://192.168.4.1/`; when `ap_password` is blank, a per-unit `RTK-<MAC>` WPA2 passphrase is logged to serial).
2. Download `/survey_logged_pins.csv`, `/routes/route_*.csv`, `/nmea/*.log`, `/serial_logs/*.txt`, or `/screenshots/*.bmp`, upload new `waypoints*.csv` files, view `settings.json`, or sync the rover RTC clock in one tap. *(Destructive uploads/deletes return HTTP `423` whenever `REC ON`, active Survey averaging, or `CAL MAG` is running.)*

---

## 12. Configuration Reference (`settings.json` Schema v2)

Settings live in `/settings.json` on MicroSD with automatic internal `FFat` mirroring (`non_index_settings_equal()` skips redundant flash erases when stepping waypoints) and v1 -> v2 schema migration:

| Parameter Key | Type | Default | Description |
| :--- | :--- | :--- | :--- |
| `settings_schema_version` | Integer | `2` | Auto-migrates existing v1 configs cleanly to Schema v2 on boot. |
| `minutesOffsetToUTC` / `timezone_label` | Int / Str | `-300` / `"EDT"` | Local UTC offset in minutes and 3-letter header TZ tag. |
| `use_imperial_units` | Boolean | `false` | `false` = Metric (`m`/`cm`); `true` = Imperial (`ft`/`in`). |
| `antenna_height_m` / `antenna_spacing_m` | Float | `1.800` / `0.555` | Pole tip height and Master-to-Slave antenna spacing (`m`). |
| `apply_lever_arm` | Boolean | `true` | Enables 3D midpoint lever-arm projection to ground tip. |
| `pole_tilt_tare_pitch_deg` / `_roll_deg` | Float | `0.0` / `0.0` | Recorded bracket bubble level tare offsets (`deg`). |
| `e220_channel` / `e220_uart_baud` | Integer | `75` / `115200` | LoRa channel (`925.125 MHz`) and UART baud rate. |
| `e220_air_rate_kbps` / `e220_tx_power_dbm` | Float / Int | `4.8` / `30` | Air rate (`4.8`/`9.6`/`19.2`) and TX power (`30 dBm` field / `21` bench). |
| `audio_nav_mode` | Integer | `0` | Audio mode (`0` = Beeps, `1` = Geiger Clicks, `2` = Mute). |
| `survey_avg_seconds` / `default_label_prefix` | Int / Str | `5` / `"PIN"` | Multi-epoch averaging (`1/5/10/30 s`) and default pin prefix. |
| `active_waypoint_file` / `active_waypoint_index` | Str / Int | `"/waypoints.csv"` / `0` | Persisted active CSV path and selected waypoint index. |
| `brightness` / `volume` / `auto_shutdown_battery_min` | Integer | `80` / `40` / `5` | LCD brightness (`%`), ES8311 gain (`%`), battery Auto-Off (`0/5/15/30 min`). |
| `ble_enabled` / `ap_ssid` / `ap_password` | Bool / Str | `true` / `"RTK_Rover_AP"` / `""` | Field BLE companion default (`ON`), SoftAP SSID, and custom `>= 8`-char WPA2 key. |
| `beam_imu_orient_mode` | Integer | `4` | Beam `SEN0696` axis alignment index (`0..7`, `4` = Face-Down). |
| `mag_hard_iron_uT` / `mag_soft_iron` | Float[3/9] | `[0,0,0]` / `I` | `CAL MAG` 360-deg ellipsoid fit parameters (`uT` and `3x3` matrix). |
| `mag_ellipsoid_cal_valid` / `mag_true_north_offset_deg` | Bool / Flt | `false` / `0.0` | Ellipsoid lock flag and `HDG FIX`-learned True-North tare (`deg`). |
| `beam_pitch_tare_deg` / `apply_lever_arm_on_calibrated_beam` | Flt / Bool | `0.0` / `false` | Learned beam pitch tare (`deg`) and opt-in Plan 2 single-ant lever arm. |
| `rtcm_source_mode` / `ntrip_*` | Str / Int | `"AUTO"` | NTRIP caster host/IP/port/mountpoint/creds reserved for Plan 2 BLE bridge. |
| `line_log_max_m` / `line_audio_fade_m` | Float | `50.0` / `20.0` | Guidance log cross-track outer span and L/R audio silence cutoff (`m`). |
| `line_arrow_green_deg` / `line_arrow_amber_deg` | Float | `4.0` / `15.0` | Heading arrow Green and Orange lock windows (`deg`, `R -> B` prioritized). |
| `map_zoom_m_per_px` / `map_show_route_trail` | Flt / Bool | `0.0` / `true` | Persistent Map zoom (`0.0` = Auto-Fit) and breadcrumb trail toggle. |

---

## 13. Troubleshooting & Field Diagnostics FAQ

### Q1: Antenna 1 shows `SINGLE` or `DGPS`, never `RTK FIX`.
* Check Row 2 `RTCM: X.Xs`: if missing/`> 5.0 s`, confirm E220 antenna, verify Channel `75` (`925.125 MHz`) & air rate on Tile 4 match the Base Pacer, and ensure `>= 18` usable SVs outside heavy canopy/walls. Note: keep close indoor bench radios separated or at `21 dBm` (`REG1=0x23`) so `+14 dB` RX LNA saturation doesn't corrupt frames.

### Q2: Row 3 drops `SLV FIX` under canopy; how does `HDG CMP` recover?
* With `SEN0696` connected on `I2C1` and `CAL MAG` + `HDG FIX` tare calibrated outdoors, `HDG CMP` engages in `0.0 s` whenever `SLV` drops out (with automatic gyro coasting around iron culverts/fences).

### Q3: Rotating the survey pole shifts coordinates, or the bubble is off-center.
* Verify `Lever-Arm: ON` on Tile 4 and `antenna_spacing_m = 0.555`, `antenna_height_m = 1.800` in `settings.json`. Hold the pole plumb against a vial and tap **`TARE`** on Tile 3 to zero bracket tilt.

### Q4: How do `WIFI: CONN/STA/AP/OFF` and Web Portal access work?
* Tapping `WIFI` scans 2.4 GHz (`802.11b/g/n` only) for configured `wifi_ssid`: connects to router (`STA`, `http://<router-ip>/`) or falls back to SoftAP (`SSID: RTK-Rover`, `http://192.168.4.1/`). Check Tile 4 / USB serial `[MEM] Post-Setup Heap` if `WiFi.mode()` fails.

---

## 14. Bi-Directional LoRa Protocol & Base Station Integration

Operates over Ebyte E220 900 MHz transparent serial (`e220_pacer.py v3.7.1` + **`v0.10.7`**):
* **Content-Independent Macro-Cycle & `Second 0` Uplink:** `e220_pacer.py` streams uninterrupted RTCM3 across `Seconds 1..(N-1)` (`--macro-cycle-s`, default `5 s`), enforces rate-aligned TX cutoffs (`450 / 620 / 680 ms`) in `Second 0`, receives Rover's **`48 B` V3 telemetry frame** (`0xAA 0x55`, `FW:vX.Y.Z`, ANT1/ANT2/baseline/pitch/heading/battery/RSSI/noise, `CRC16-CCITT`), and runs one Mode 0 `C0 C1 C2 C3 00 02` RSSI probe (`680..920 ms`).
* **3-Rate Synchronized Air Switching (`4.8k / 9.6k / 19.2k`):** Tile 5 `LORA` button triggers `CMD_REQ_RADIO_SWITCH` -> 4 spaced `ACK_SWITCH` frames in `Second 1` -> Mode 3 reconfiguration at `Second 2` (with 60s failsafe rollback).
* **Arch 2B Async Ping-Pong Mailboxes & `<= 200 B` Coalescing:** Background TCP ingress filters `CRC24Q` frames, drops `<= 28 B` empty MSMs, overwrites per-type latest-value mailboxes (`2.5 s` dynamic / `15.0 s` static TTL), prioritizes metadata (`1005/1006/1033/1230`), uses starvation-fair round robin (`1074` anchor), packs compatible small frames (`<= 200 B`), logs atomic JSON (`/run/rtk_rover_status.json`) + rolling gzip CSVs, and passes 7/7 automated checks in `scripts/stage2_bench_verify.py v1.0.3`.
