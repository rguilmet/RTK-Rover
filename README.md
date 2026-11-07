# RTK Rover

High-precision handheld and pole-mounted RTK GNSS Waypoint Guidance, Property Pin Seeker, Field Survey Logger, and Dual-Antenna Moving-Baseline Compass built on the **Waveshare ESP32-S3-Touch-LCD-3.5B**, **Holybro H-RTK Unicore UM982**, and **Ebyte E220-900T30S** 900MHz LoRa transceiver.

**Firmware Version:** `v0.7.7`  
**License:** GNU General Public License v3.0 ([GPLv3](LICENSE))  
**User Documentation:** [Field User Manual](docs/user_manual.md)  
**Engineering Review:** [Critical Design Review (CDR)](docs/critical_design_review.md) | [Project Retrospective](docs/project_retrospective.md)  
**Next-Gen Architecture:** [Plan Smart Pole and Android App](docs/plan_smart_pole_and_android_app.md) | [Phase 7 Cellular NTRIP Bridge Plan](docs/plan_phase7_cellular_ntrip_bridge.md)

---

## 1. Overview & Key Capabilities

* **Real-Time Priority-4 Tactile Audio, Zero-Lag Navigation & UPS Bus Recovery (`v0.7.6` - `v0.7.7`):**
  * **Dedicated Priority-4 Audio Synthesizer (`RoverAudio`):** Decoupled ES8311 I2S DMA tone generation (`beepQueue`) from the LVGL UI thread into a dedicated Core 1 FreeRTOS task at Priority 4 (`task_audio_beep`). Tactile button tones fire within **`< 1 ms`** of touch contact, completely immune to display rendering or storage I/O.
  * **Zero-Lag (`0 ms`) Same-Frame Waypoint/File Selection & `<= 2 ms` Core 0 Wakeup:** Tapping `<` or `>` immediately recomputes target distance, steering prompt, cardinal offsets, scale, and line stakeout geometry on the **exact same LVGL frame** (`refresh_t0_guidance_readout_immediately()`) while waking `task_rover_core0` within `<= 2 ms` (`g_waypoint_change_seq`) to push updated canvas vectors.
  * **Background Settings Persistence & Smart `FFat` Flash Wear Protection:** Moved `save_settings()` out of `RoverUI` into the Priority-1 background `loop()` with a race-free 5-second quiet debounce and smart `FFat` mirror skip (`non_index_settings_equal()`), eliminating `0.8–2.18 s` internal SPI Flash wear-leveling stalls when stepping through waypoints.
  * **Zero-Allocation `512 B` Chunked CSV Loader & Lossless SD Serial Logger:** Replaced 1-byte `readStringUntil` heap allocations in `waypoint_load_csv()` with `512 B` stack chunk parsing (`< 5 ms`) and a 64-entry ring stem cache (`s_stem_cache`), and upgraded `serial_logger` with task-scoped re-entrancy (`s_io_owner_task`) and `500 ms` batched FAT flushes so concurrent tasks never drop `[TOUCH]` or `[WIFI]` log lines.
  * **UPS Warm-Reset Stuck-`SDA` I2C0 Bus Recovery (`v0.7.6`):** Implements an 18-clock GPIO open-drain `SCL` bus-clear + `STOP` sequence (`i2c_bsp_gpio_bus_clear()`) before `i2c_new_master_bus()`, unified `400 kHz` / `20 ms` timeout / `i2c_bsp_lock()` across all 6 I2C0 peripherals (`TCA9554`, `ES8311`, `AXP2101`, `AXS15231B`, `PCF85063`, `QMI8658`), and a 5-attempt `i2c_bsp_reinit_bus()` recovery loop to prevent warm-reset panic loops when the LiPo UPS keeps I2C slaves powered across resets.
* **Dual-Power & Hot-Swap UPS Architecture (Phase 6):**
  * **AXP2101 PMIC Constant-Current Charging:** Configured to $500\text{ mA}$ for safe charging and float maintenance of an internal 1S LiPo buffer.
  * **Hot-Swap VBUS USB Sense:** 14-bit ADC monitoring of external USB line voltage (`vbus_voltage_v >= 4.0V`) with debounce filter. Seamlessly powers the high-current UM982 GNSS and E220 1W LoRa radio from an external 5V pack, while battery keeps the ESP32-S3 and LCD alive across power interruptions with zero rebooting.
  * **Auto-Shutdown on Battery Timeout:** User-configurable idle battery timer (`auto_shutdown_battery_min` in `settings.json`, default 5 min) triggering soft power-off ($<40\,\mu\text{A}$) when external power is removed and the rover is left idle. Any touchscreen tap resets the timer.
  * **Minimalist Row 1 Power Widget & Sensor Offline Badges:** Displays `[USB] 5.04V+` (charging) or `[USB] 5.04V` (float), switching to `[Bat] 85% 3.92V` on battery. When on battery without USB, Row 2/3 fix badges switch to red `[OFF]` and radio displays `LoRa: OFF`.
* **Wi-Fi Multi-State Visual Cues & Internal `DRAM0` / PSRAM Memory Architecture (Phase 6.1 / `v0.7.5` - `v0.7.7`):**
  * **Multi-Color Button & Icon States with Instant Tap-to-Cancel:** Tile 1 Wi-Fi button illuminates Vivid Amber (`WIFI: CONN`, tap again to abort connection within `<= 100 ms`), Emerald Green (`WIFI: STA`, online immediately upon DHCP `GOT_IP` prior to best-effort NTP), Vivid Violet (`WIFI: AP`), or Navy Blue (`WIFI: OFF`). Header icon matches state with Amber, Electric Cyan, Violet, and Slate Gray.
  * **Octal PSRAM Offload & `DRAM0` Radio Headroom (`v0.7.5`):** Migrated `82.9 KB` of static `.dram0.bss` logger/waypoint buffers (`serial_logger`, `g_waypoints`, `rtcm_logger`, `nmea_logger`) and `~50 KB` of LVGL 9 UI widget trees into 8 MB Octal PSRAM (reducing static global dynamic RAM from `44%` to `20%` / `37.1 KB .bss`) and trimmed the QSPI DMA bounce strip to 24 lines (`15.3 KB`), restoring `>150 KB` of contiguous internal 8-bit/DMA SRAM required by `esp_wifi_init()` / `esp_wifi_start()`.
  * **Core v3.3.12 Netif Preservation & Resiliency:** Pinned background `wifi_connect` task to Core 1 (Priority 3), deferred `WiFi.mode(WIFI_OFF)` teardown to single-threaded `loop()`, restored `WIFI_ALL_CHANNEL_SCAN` / `WIFI_AUTH_WPA_PSK`, and added `[MEM] Post-Setup Heap` + `WiFi.mode()` status diagnostics alongside guaranteed `192.168.4.1` SoftAP fallback.
* **Rock-Solid Deep-Woods Heading Fallback & Gyro Spin Elimination (Phase 5 & 6.1):**
  * **Zero-Drift Stationary Hold (`HDG HLD` / `HLD`):** Heading freezes instantly when stationary ($0.0^\circ/\text{s}$ drift) eliminating IMU bias integration.
  * **Active Pivot Tracking (`HDG GYRO` / `GYR` Blue `#3B82F6`):** Smooth yaw rate integration during deliberate in-place turns ($|\omega_z| \ge 1.5^\circ/\text{s}$).
  * **Kinematic Displacement ("Walk 3 Steps" $\ge 0.70\text{ m}$):** Resolves true geographic azimuth even when crawling through thick canopy (`HDG WALK` / `WLK` Cyan `#06B6D4`) with a 3.0-second persistence filter.
* **Cadastral Parcel Mapping & Signed Sequence Ordering (Phase 5):**
  * **Signed Sequence Schema (`poly_order`):** Connects positive sequence numbers ($1 \rightarrow 2 \dots \rightarrow N \rightarrow 1$) into closed parcel boundary polygons (Gold pins `#FBBF24`), draws dashed tie-lines from negative sequence points ($-k$) to boundary vertex $k$ (Orange pins `#F97316`), and supports standalone control/construction points (`0`).
  * **Decluttered Line Stakeout Canvas:** Clean baseline vector, traverse chevron, and perpendicular tie-line with text labels elevated to the top marquee row to prevent overlap with buttons and notes.
  * **Sub-25ms Touch Responsiveness:** Reduced LVGL polling delay to 20 ms and eliminated synchronous SD reads via an in-memory survey stem cache.
  * **128-Point Route Trail:** Toggleable historical surveyor walking trail directly on the Map View canvas (`"map_show_route_trail"` in `settings.json`).
* **Geodesic Line Stakeout & Cadastral Guidance (Phase 4):**
  * Real-time along-track Station ($S$) and perpendicular cross-track Offset ($O$) navigation along boundary baselines ($A \rightarrow B$).
  * Interactive Tile 0 tri-mode toggle: `[TACTICAL GUIDANCE]` (Course-Up & Centimeter Bullseye) $\rightarrow$ `[LINE STAKEOUT]` (Deed Vector with Left/Right guidance prompts and quadrant bearings e.g. `S 62°15'12" W`) $\rightarrow$ `[MAP VIEW]` (2D Geodetic Canvas with Zoom, Pan, Boundary Polygon, Pins, and Breadcrumbs).
  * One-tap baseline direction swap (`[ <-> SWAP ]`) and Geiger audio guidance tracking deed line proximity.
* **Bi-Directional LoRa Protocol & Remote Air-Rate Switching (Phase 3 & Phase 5.1):**
  * 45-byte packed binary uplink telemetry (`0xAA 0x55`, CRC16-CCITT) transmitted in the half-duplex quiet window ($t = 0.10\text{ s} \dots 0.45\text{ s}$ post-epoch, $\approx 31\text{ ms}$ airtime) every 5 seconds. Reports ANT1 position fix, ANT2 slave fix (`SLV_FIX`/`SLV_FLT`), measured 3D baseline length (`BL: 55.4cm`), crossbar pitch, explicit heading source (`DUAL_FIX`, `WALK`, `GYRO`, `HLD`), satellite counts, ground-tip coordinates, and battery/USB health.
  * Base Station [`e220_pacer.py`](scripts/e220_pacer.py) (v3.4.0) features a dual-format auto-detecting unpacker (39B/45B), outputs live telemetry heartbeats to console with clean ASCII `deg` units and `PWR:USB` / `Bat:xx%` power formatting, writes atomic JSON to `/run/rtk_rover_status.json` (`tmpfs` in RAM, zero SD wear), and appends to rolling daily CSV with 30-day auto-pruning.
  * Over-the-air dynamic air-rate switching (`19.2k` $\leftrightarrow$ `9.6k` deep woods mode) triggered directly from Tile 1 `LORA` button with zero `sudo` passwords, zero service restarts, and 60-second automatic rollback failsafe.
* **Dual-Antenna Moving-Baseline RTK Kinematics:** Powered by the Holybro H-RTK Unicore UM982. Phase centers for Master (Position) and Slave (Heading) are spaced at `55.5 cm` along a rigid crossbar. Kinematic algorithms project coordinates to the exact pole centerline midpoint and down to the ground tip (`1.80 m`), completely eliminating position orbit when rotating the pole.
* **Dual-Mode Hybrid Navigation Engine (Tile 0):**
  * **Coarse Tactical Grid ($> 1.5\text{ m}$):** Dynamic Course-Up grid with antenna crossbar beam arrow, line-of-sight tracking vector, turn prompts, dynamic auto-scaling, and a 64-point on-screen breadcrumb trail.
  * **Fine Bullseye Stakeout ($\le 1.5\text{ m}$):** Seamlessly transitions into a centimeter bullseye with $1\text{ m}$, $20\text{ cm}$, and $2\text{ cm}$ rings, providing real-time North/East and Forward/Right offsets with distance-proportional audio Geiger clicks.
* **4-Row Persistent Top Status Header Bar:**
  * **Row 1:** Local calendar date, 24-hr time with timezone offset, SI/Imperial unit badge, multi-state Wi-Fi icon, minimalist battery/USB power widget (`[USB] 5.04V` / `[Bat] 85% 3.92V`).
  * **Row 2 (Master Antenna ANT1):** RTK Fix badge (`RTK FIX`, `RTK FLT`, `DGPS`, `SINGLE`, `OFF`), Satellites used/in-view, horizontal accuracy estimate ($+/-\text{cm}$), RTCM differential age, and `"LoRa: "` signal strength with multi-tier color coding.
  * **Row 3 (Slave Antenna ANT2):** Heading Fix badge (`HDG FIX`, `HDG FLT`, `OFF`), Slave satellites, measured baseline length ($55.5\text{ cm}$), baseline pitch ($^\circ$), and true heading ($^\circ$).
  * **Row 4:** Streamlined ground-tip Latitude, Longitude, and Ellipsoidal Height in 14pt font.
* **Field Survey Logger & Consolidated Quick Settings (Tile 1):**
  * Integrated `<DERIVED>` prefix mode and clean prefixes (`PIN`, `PIPE`, `MONUMENT`, `STAKE`, `CORNER`, `FENCE`, `TREE`, `UTILITY`, `WP`).
  * Universal collision-free `_Lxxx` numbering scanning `/survey_logged_pins.csv` with zero UI loop disk I/O latency.
  * Multi-epoch averaging (`1s`, `5s`, `10s`, `30s`), visual progress bar, large `[LOG PIN NOW]` button, and circular marquee ID label.
  * **Consolidated $3\times 4$ Quick Settings Grid (12 Buttons):**
    * Row 1: `VOL: xx%` / `VOL: MUTE`, `AUD: GEIGER / BEEPS / MUTE`, `BRI: xx%`
    * Row 2: `SERIAL: ON/OFF`, `NMEA: ON/OFF`, `RTCM: ON/OFF`
    * Row 3: `TELEM: ON/OFF`, `IMU: ON/OFF`, `TOUCH: ON/OFF`
    * Row 4: `WIFI: CONN/STA/AP/OFF`, `UNITS: IMP/SI`, `LORA: 19.2k / 9.6k`
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

### Tile 0: Pin Seeker, Line Stakeout & 2D Cadastral Map View
| Tactical Course-Up ($> 1.5\text{ m}$) | Bullseye Centimeter Stakeout ($\le 1.5\text{ m}$) |
| :---: | :---: |
| ![Tile 0: Tactical Course-Up Navigation](docs/images/screenshot_tile0_guidance.png) | ![Tile 0: Bullseye Centimeter Stakeout](docs/images/screenshot_tile0_bullseye.png) |
| *Rotating antenna crossbar, LOS vector, and breadcrumb trail.* | *Centimeter concentric rings ($1\text{ m}$, $20\text{ cm}$, $2\text{ cm}$) with lock audio cues.* |

| Line Stakeout Mode (Station & Offset) | 2D Cadastral Map View (Polygon & Tie-Lines) |
| :---: | :---: |
| ![Tile 0: Line Stakeout Mode](docs/images/screenshot_tile0_line_stakeout.png) | ![Tile 0: 2D Cadastral Map View](docs/images/screenshot_tile0_map_view.png) |
| *Deed baseline vector, perpendicular tie-line, lateral offset, and quadrant bearing.* | *Closed parcel boundary polygon (`poly_order > 0`), dashed tie-lines (`-k`), and HUD zoom/pan.* |

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
| **E220 LoRa** | `E220_TXD` | `GPIO 41` | **ESP32 RX** (`Serial0`) | E220 TXD $\rightarrow$ ESP32 RX (`115200 8N1` Transparent / `9600` Config) |
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
        E220_RX["E220 LoRa UART0 (IO41/42 @ 115200)"] --> RTCM_FWD["Forward RTCM3 Bytes"]
        RTCM_FWD --> COM2_TX["UM982 COM2 UART2 (IO47/48 @ 115200)"]
        COM1_RX["UM982 COM1 UART1 (IO17/18 @ 115200)"] --> NMEA_PARSE["NMEA & UNIHEADINGA Parser\n(GPGGA, GPRMC, GPGST, GPGSV, GNGGAH, BASEINFOA)"]
        NMEA_PARSE --> KINEMATICS["3D Lever-Arm & Ground Tip Kinematics\n(Midpoint 27.75cm, Height 1.80m)"]
        KINEMATICS --> NAV_ENGINE["Geodesic Nav Engine\n(Range, Bearing, Offsets)"]
        NAV_ENGINE --> QUEUE["roverStateQueue (xQueueOverwrite)"]
    end

    subgraph Core1["Core 1: Real-Time Audio, UI Rendering & Background Storage (Priorities 4, 3, 2, 1)"]
        AUDIO_QUEUE["beepQueue (FreeRTOS Queue)"] --> CODEC["RoverAudio Task (Priority 4)\n<1ms ES8311 I2S Tone Synthesizer"]
        QUEUE --> UI_UPDATE["RoverUI Task (Priority 2)\nupdate_rover_ui() & Proximity Geiger"]
        I2C_POLL["Shared I2C0 Bus Driver (400 kHz, 18-clk Bus Clear)\n• Touch AXS15231B (30 Hz)\n• IMU QMI8658 12-byte burst (10-50 Hz)\n• RTC PCF85063 (1 Hz)\n• PMIC AXP2101 (0.5 Hz)"]
        UI_UPDATE --> LVGL["lvgl_port_task (Priority 3)\nLVGL 9 QSPI Refresh & 0ms Same-Frame Callbacks"]
        SETTINGS["loopTask (Priority 1)\n5s Debounced Settings & Smart FFat Skip\nLossless SD Serial/NMEA/RTCM Log Flush"] --> STORAGE["MicroSD Card / FFat Mirror"]
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
  "e220_uart_baud": 115200,
  "e220_channel": 75,
  "e220_air_rate_kbps": 19.2,
  "e220_tx_power_dbm": 21,
  "e220_subpacket_bytes": 200,
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
* **check_secrets.py (v1.1.1):** Security and privacy verification engine. Scans Git tracked files and unpushed commits for exposed credentials, cryptographic keys, and absolute user paths. Dynamically loads local personal search tokens from an unversioned .secrets.local configuration.
* **install_git_hooks.py (v1.1.0):** Installs the pre-push security verification hook into .git/hooks/pre-push.
* **kml_to_waypoints.py (v1.0.0):** Converts Google Earth or GIS .kml boundary polygons, paths, and placemarks directly into the rover's 19-column waypoints.csv format with automatic label sanitization and multi-line description handling.
* **validate_waypoints.py** (v1.2.0): Checks syntax, column count, bounds, and computes leg-by-leg distances, bearings, and closed-loop perimeter for any waypoint CSV file.
* **`e220_pacer.py` (v3.4.0):** Base station hardware pacer daemon for RTKBase streams with silicon-calibrated Time-on-Air, Section 5.6.2 two-regime pacing, 45-byte bi-directional uplink telemetry (`PWR:USB` / `Bat:xx%`), remote air-rate switching, and Adaptive Constellation Interleaving at $\le 9.6\text{ kbps}$.

---

## 8. License

This project is licensed under the [GNU General Public License v3.0 (GPLv3)](LICENSE).
