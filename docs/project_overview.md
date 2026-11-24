# RTK Rover - Project Overview

**Document Version:** `v2.10.7`  
**Last Updated:** November 24, 2026  
**Firmware Target:** `v0.11.0` (depends on `Field_Instrument_Core >= v1.0.1`; standalone snapshot at Git tag `v0.10.10`)  
**Python Ecosystem Target:** `e220_pacer.py v3.7.1` | `stage2_bench_verify.py v1.0.3` | `cogo_solver.py v1.2.0`  
**Mechanical CAD Baseline:** `v2.2.4`  
**License:** GNU General Public License v3.0 (`GPLv3`)  

This document serves as the master technical specification for the **RTK Rover** project. It details the mechanical enclosure, dual-antenna lever-arm kinematics, electrical pinouts, FreeRTOS + LVGL 9 firmware architecture, operational modes, and companion Python scripts.

---

## 1. Project Identity, Versioning & Documentation Rules
* **CRITICAL RULE:** All firmware and Python tools define an explicit SemVer identifier (`v[Major].[Minor].[Patch]`) bumped at every iteration.
* **Mandatory Per-Version Changelog & Git Sync:** Every version bump (`ROVER_FIRMWARE_VERSION` in `user_config.h` or `__version__` in Python scripts) is recorded in `CHANGELOG.md`, verified with `python scripts/check_secrets.py`, and committed + pushed to Git/GitHub (`origin/master`) so any iteration is reproducibly restorable from Git.
* **Why:** Unique version tags let the ESP32 telemetry uplink (`FW:vX.Y.Z`), UI diagnostics header, serial logs, and Git commits map 1:1.
* **Pure ASCII Requirement:** All user-facing UI strings, serial logs, and engineering docs strictly use 7-bit ASCII characters (`deg`, `+/-`, `->`, `<=`, `>=`) to guarantee compatibility with LVGL font glyphs and Windows/POSIX terminals.
* **Sibling Repository Architecture & Code/Data Decoupling:** Public `RTK_Rover` contains generic firmware, drivers, kinematics, and schema-driven mathematical tools. Private land records, deed PDFs, personal surveyed coordinates, raw sensor logs, and local LAN/Wi-Fi configs reside strictly in the independent sibling repository (`RTK_Rover_Private`).

---

## 2. Bill of Materials (BOM) & Mechanical Architecture
1. **Display & Main Processing Unit:** **Waveshare ESP32-S3-Touch-LCD-3.5B**
   * `320x480` Portrait IPS QSPI Display (`AXS15231B` controller + Capacitive Touch `0x3B`).
   * Integrated `AXP2101` PMIC (`0x34`), `TCA9554` IO Expander (`0x20`), `PCF85063` RTC (`0x51`), `QMI8658` 6-axis IMU (`0x6B`), `ES8311` Audio Codec (`0x18`), and MicroSD slot.
   * Housed in the support-free Option A PETG enclosure (**[`CAD v2.2.4`](mechanical/cad/README.md)**, `120.0 x 180.0 x 28.0 mm`) with `Back_Chassis_Frame`, lift-off `Top_Cover`, Option B2 `5,000 / 10,000 mAh` rear pole battery pod, `37.5 deg` open-truss `Pole_Tilt_Wedge_37_5`, downward-D `GX-12` bulkhead, left-wall `3.5 mm` jack L-tab, and slot-in `Slim_Sun_Shield`.
2. **RTK GNSS Receiver:** **Holybro H-RTK Unicore UM982** (Dual-Antenna Multi-Band RTK GNSS)
   * Mounted on the survey pole T-bar with Master (Position) and Slave (Heading) antennas separated by `0.555 m` (`55.5 cm`, configurable via `antenna_spacing_m` in `settings.json`).
   * The survey pole centerline sits at the **exact midpoint** (`27.75 cm`) between Master and Slave antennas, at height `antenna_height_m` (default `1.800 m`) above the ground tip.
3. **RTCM Correction Radio:** **Ebyte E220-900T30S** (900MHz LoRa UART Transceiver)
   * Streams RTCM3 differential corrections in transparent mode at `115200` baud to ESP32 Core 0, which validates `CRC24Q` and forwards intact frames to `UM982 COM1` at `115200` baud via `s_com1_tx_mutex`.
   * **Radio Configuration (Verified on both Base and Rover, Profile Rev 3):**
     * **RF Channel (`e220_channel`):** `75` (`0x4B` -> `850.125 + 75 = 925.125 MHz`, cleanest US 902-928 MHz ISM channel in `test/e220-scan.csv` at `-96.23 dBm` avg, `+10.125 MHz` clear of `915.0 MHz`).
     * **UART Rate (`e220_uart_baud`):** `115200` bps, `8N1` (matching `UM982 COM1`).
     * **Air Data Rate (`e220_air_rate_kbps`):** **`4.8` kbps field default** (`REG0 = 0xE3`) with synchronized 3-rate switching across `4.8k / 9.6k / 19.2k` (`REG0 = 0xE3 / 0xE4 / 0xE5`; also supports `2.4, 38.4, 62.5`).
     * **Packet Size (`e220_subpacket_bytes`):** `200 bytes` (`REG1` bits `[7:6] = 0b00`, utilizing native `400-byte` buffer atomic framing).
     * **Mode 0 Live RSSI Register (`REG1` Bit 5 = `0x20`):** Enabled, allowing on-demand `C0 C1 C2 C3 00 02` ambient noise floor and last-packet RSSI queries (dual 4-byte `C1 00 <noise> <pkt>` and 5-byte `C1 00 02 <noise> <pkt>` parser with `s_tx_air_busy_until_ms` RF TX guard and `180 ms` reply timeout) during `Second 0` `post_uplink_gap` without appending extra bytes to RX UART frames (`REG3 = 0x00`).
     * **Transmit Power (`e220_tx_power_dbm`):** **`30 dBm` (`1 W`) production field default (`REG1 = 0x20`)**; `21 dBm` (`125 mW`, `REG1 = 0x23`) recommended for close indoor bench tests to prevent `E220-900T30S` `+14 dB` RX LNA front-end saturation (`> -35 dBm`).
     * **Target EEPROM Profile (Field Default):** `ADDH=0x00 ADDL=0x00 REG0=0xE3 REG1=0x20 CHAN=0x4B REG3=0x00`.
   * **Base Station Software & Architecture 2B Async Ping-Pong Mailbox Transport (`e220_pacer.py v3.7.1`):**
     * **RTKBase Stream Filtering:** RTKBase port `5016` is filtered via `str2str` to `"1006(5),1074,1084,1094,1230(10)"`, producing ~625-650 Bytes/s. Empty MSM frames (`<= 28 B`) are dropped prior to TX.
     * **Asynchronous Ingress & Ping-Pong Latest-Value Mailboxes (`e220_pacer.py v3.7.1`):** A background ingress thread continuously drains TCP bytes, validates `CRC24Q`, and overwrites per-message-type mailboxes in place (zero epoch backlog buildup), with `2.5 s` dynamic MSM TTL expiration (`15.0 s` static metadata). The TX scheduler prioritizes static metadata (`1005/1006/1033/1230`), rotates MSM constellations via starvation-fair round robin (`now - last_tx` with GPS `1074` tie-breaker), and packs small frames (`<= 200 B`, e.g. `1006+1230` or `+1084`) into a single Regime A UART write.
     * **Two-Regime AUX + Stage-1-Calibrated Time-on-Air Pacing:**
       * Per [`docs/benchmarks/e220_stage1_20261116/`](benchmarks/e220_stage1_20261116/README.md) and [`docs/e220-reference.md`](e220-reference.md) (`v3.2`), `AUX=1` indicates the 400-byte UART buffer emptied into the LLCC68 transceiver, **not** that over-the-air RF transmission finished. Both Base and Rover enforce empirical silicon `ToA` formulas (`4.8k: 70+1.87*N ms`, `9.6k: 45+1.19*N ms`, `19.2k: 29+0.80*N ms`) + `30 ms` (`>=9.6k`) / `35 ms` (`<=5k`) guard delays.
       * **Single-Subpacket Frames (`<= 200 B`):** Paced deterministically by calibrated LoRa `ToA + guard`.
       * **Two-Subpacket Frames (`201..400 B`):** Hold `AUX` LOW while Subpacket 1 transmits (can rise `100-147 ms` early at `<= 9.6 kbps`); paced by waiting for `AUX` rising edge plus `max(ToA(tail), total_ToA - elapsed_since_flush) + guard`.
     * **Content-Independent Consolidated Macro-Cycle & Mode 3 / Mode 0 Hardening:** Anchored to `UTC_Second % macro_cycle_s` (default `5 s`, optional `10 s`): Seconds `1..(N-1)` stay `100%` unbroken downlink, while `Second 0` enforces aligned downlink cutoffs (`450 / 620 / 680 ms`) before Rover's uplink slot (`500..660 / 670..760 / 730..810 ms`) and a single post-uplink `C0 C1 C2 C3 00 02` Mode 0 RSSI query (`680..920 ms`, accepting cold-boot `pkt_raw == 0x00`). Synchronous 3-rate switches (`4.8k / 9.6k / 19.2k`) transmit 4 spaced `ACK_SWITCH` frames in `Second 1` (`80/260/440/620 ms`), reconfigure both E220 modules in Mode 3 at `Second 2` (restoring `M0/M1=0` before switching baud), and arm a 60-second fallback rollback failsafe.
     * Includes `GPIO 22` reset watchdog if `AUX` locks LOW `> 1.5 s`, `TIOCEXCL` exclusive serial port locking, source `CRC24Q` verification, and startup Channel 75 ambient noise probing.
     * **VBAT 5V Boost Power Architecture & Link Budget:** High-side switched `MT3608` boost (`NDP6020P + 2N2222`) powers `UM982` + `E220` cleanly on battery with `com1_online` / `radio_online` activity detection independent of `VBUS` (`~806 mA` / `~3.0 W` typical @ `3.7 V` -> ~3.2-3.4 hrs on `3,000 mAh` or ~10.6-11.2 hrs on `10,000 mAh`; see [Power Budget](schematics/system_wiring_schematics.md#3-system-power-budget-mt3608-boost-analysis--battery-runtime-estimates)).

---

## 3. Dual-Antenna Lever-Arm Kinematics & IMU Decoupling

### A. Master-to-Pole-Center & Ground Tip Projection (`nav_engine`)
The UM982 reports WGS84 coordinates (`GPGGA`) of the **Master Antenna (ANT1)** phase center and outputs moving-baseline azimuth (`line_hdg`) and pitch (`baseline_pitch_deg`) from Master to Slave (`UNIHEADINGA`). Because the pole sits at the midpoint (`B/2 = 0.2775 m`) at height `antenna_height_m` (`1.800 m`), `nav_compute_pole_tip_position()` projects the 3D vector to the ground tip (`dN = (B/2)*cos(pitch)*cos(az)`, `dE = (B/2)*cos(pitch)*sin(az)`, `dU = (B/2)*sin(pitch) - H_pole`), eliminating orbital shift (`< 8 mm` over `360 deg` pole rotation).

### B. Slave Antenna (ANT2) Coordinates & Baseline Projection
Slave coordinates are parsed from `$GNGGAH` / `$GPGGAH` or projected along the 3D dual-antenna baseline vector from Master (`ANT1`) and displayed under `[ANT2 SLAVE]` on Tile 4.

### C. Unicore Extended Base Station Telemetry (`BASEINFOA`)
* **Autonomous RTCM Decoding:** The UM982 decodes incoming RTCM 1005/1006 frames received via LoRa on **`COM1`** (`Serial1` `GPIO 17/18`) and emits 1 Hz `#BASEINFOA` reports (Base Station ID, health, ECEF `X, Y, Z`).
* **Closed-Form Geodetic Conversion & Mapping:** `um982_driver.cpp` converts ECEF to WGS84 Lat/Lon/Alt via closed-form Bowring equations (`O(1)`), populating Tile 4's Base Station card and plotting the **`BS` survey-control triangle symbol** dynamically on Tile 0 `MAP VIEW` (`v0.10.6`).

### D. Deep-Woods Heading Fallback, `SEN0696` Beam Fusion & `CAL MAG` (`v0.9.0..v0.9.2`)
In dense forest canopy when secondary (`ANT2`) carrier lock drops, `nav_heading_beam_update()` + `ahrs_9dof` + `matrix_math` run a zero-lag multi-tier hierarchy:
* **Tier 1 (`HDG_SRC_DUAL_FIX` / `DUAL_FLT`):** Carrier-phase moving-baseline heading (`0.2 deg`), continuously teaching magnetic True-North declination tare, reference field `B_ref`, vertical component `Bz_ref`, and `beam_pitch_tare_deg` via `atan2` circular averaging, with immediate `0.0 s` handoff (`s_last_disp_ms = 0`) on `SLV` dropout.
* **Tier 2A (`SEN0696` Beam 9-DOF AHRS -> `[CMP]` Teal `#14B8A6`):** Isolated on **`I2C1` (`GPIO 47/48` @ `400 kHz`, automatic `I2C0` bench fallback)**, polling `BMI323 + BMM350` at `25-30 Hz` (`~3 Hz BMP581` baro) through instance-safe [`ahrs_9dof.{h,cpp}`](../src/sensor_fusion/ahrs_9dof.cpp) with `beam_imu_orient_mode = 4` (Face-Down `(+X,-Y,-Z)`), 50-sample stationary accel-variance gyro FOC, `360-deg` PSRAM ellipsoid `CAL MAG` fit (`4`-param sphere fallback + `BZ_ref` self-heal on flat spins), and a triple-condition iron anomaly gatekeeper (`|B|`, `Bz`, 200 ms gyro-mag innovation + 500 ms hysteresis). Onboard `QMI8658` stays dedicated to enclosure `DisplayPose_t`.
* **Tier 2B (Fallback Legacy `MMC5983MA 0x30` on `I2C0`):** Retained with `SET`/`RESET` coil bridge offset calibration if `SEN0696` is absent.
* **Tier 3/4 (`HDG_WALK`, `HDG_GYRO`, `HDG_HOLD`):** Kinematic displacement (`>= 0.70 m`, `WLK` Cyan), bias-compensated gyro coasting (`GYR` Blue), and stationary zero-drift hold (`HLD` Slate).

---

## 4. Electrical Pinouts & Hardware Mapping (`ESP32-S3-Touch-LCD-3.5B`)

> **PIN NAMING CONVENTION (Device-Relative):** `<DEV>_TXD` is module Transmit Output -> ESP32 UART RX Input; `<DEV>_RXD` is module Receive Input <- ESP32 UART TX Output.

| Subsystem | Signal Label | ESP32-S3 GPIO | ESP32 Direction | Function / Bus Assignment |
| :--- | :--- | :--- | :--- | :--- |
| **UM982 COM1** | `UM982_TXD0` | `GPIO 17` | **ESP32 RX** (`Serial1`) | UM982 COM1 TX -> ESP32 RX (`115200 8N1`, NMEA, Heading & Responses) |
| **UM982 COM1** | `UM982_RXD0` | `GPIO 18` | **ESP32 TX** (`Serial1`) | ESP32 TX -> UM982 COM1 RX (`115200 8N1`, RTCM3 & Config commands) |
| **Beam 10DOF `I2C1`** | `10DOF_SDA` | `GPIO 47` (`P21`) | **I2C1 Open-Drain** | Isolated `I2C_NUM_1` SDA (`400 kHz` -> `SEN0696 0x4A..0x4D` + `4.7 kOhm` pull-up) |
| **Beam 10DOF `I2C1`** | `10DOF_SCL` | `GPIO 48` (`P23`) | **I2C1 Clock** | Isolated `I2C_NUM_1` SCL (`400 kHz` -> `SEN0696` + `4.7 kOhm` pull-up; auto `I2C0` fallback) |
| **E220 LoRa** | `E220_TXD` | `GPIO 41` | **ESP32 RX** (`Serial0`) | E220 TXD -> ESP32 RX (`115200 8N1` Transparent / `9600` Config) |
| **E220 LoRa** | `E220_RXD` | `GPIO 42` | **ESP32 TX** (`Serial0`) | ESP32 TX -> E220 RXD |
| **E220 Control** | `E220_AUX/RST/M0/M1` | `21 / 38 / 39 / 40` | **In / Out x3** | Busy/Ready (`AUX`), Reset, and Mode 0/3 control |
| **Internal `I2C0`** | `ESP_SCL / SDA` | `GPIO 7 / 8` | **I2C0 Master** | Touch (`0x3B`), TCA9554 (`0x20`), AXP2101 (`0x34`), RTC (`0x51`), QMI8658 (`0x6B`), ES8311 (`0x18`), opt. `MMC5983MA` (`0x30`) |
| **LCD QSPI** | `CS/CLK/D0..3/BL` | `12, 5, 1..4, 6` | **QSPI Master + PWM** | `AXS15231B` `320x480` Portrait Display (`GPIO 6` Backlight PWM) |
| **SD Card** | `D0/CMD/CLK` | `9, 10, 11` | **1-Bit SD_MMC / SPI** | 3-stage mount fallback (`SD_MMC` 1-bit `EXIO3=High-Z` -> `LOW` -> `SD` SPI, `maxOpenFiles = 10`) |
| **BOOT Button** | `BOOT_SWITCH` | `GPIO 0` | **ESP32 Input** | **Long-Press (`>= 1.5 s`):** Capture 24-bit uncompressed BMP screenshot (`460,854 B`) |

---

## 5. FreeRTOS, Core Libraries & LVGL 9 Architecture (`v0.11.0`)

### Core Separation
* **Core 0 (`task_rover_core0` / `"RoverCore0"`, Priority 4 -- Zero I2C Access):**
  * Polls `E220` (`Serial0`), logs raw bytes to `rtcm-base_*.txt`, stages incoming bytes through the `rx_stage` `CRC24Q` Gatekeeper (`280 / 380 / 480 ms` truncation timeouts), forwards intact RTCM3 frames to `UM982 COM1` (`s_com1_tx_mutex`), runs `Second 0` uplink (`48 B` V3) & Mode 0 RSSI polling, and handles 4-ACK 3-rate switching.
  * Polls `UM982 COM1` (`Serial1`), runs 30 Hz midpoint lever-arm + `nav_heading_beam_update()`, and wakes within `<= 2 ms` via `g_waypoint_change_seq`.
* **Core 1 (`RoverAudio` Pri 4, `lvgl_port_task` Pri 3, `RoverUI` Pri 2, `loopTask` Pri 1 -- Exclusive `I2C0` & `I2C1` Owner):**
  * **`task_audio_beep` (`"RoverAudio"`, Priority 4):** `< 1 ms` ES8311 I2S tone/chirp synthesis driven by `fic_audio_beeper` inside `Field_Instrument_Core`.
  * **`fic_lvgl_port_task` (Priority 3):** Renders the modular 6-tile LVGL 9 UI (`src/lvgl_port/`, `9` UI `.c` + `2` `.h` modules `<= 789 L`), runs `0 ms` same-frame button recomputation (`refresh_t0_guidance_readout_immediately()`), and services lock-free one-shot PSRAM framebuffer screenshot requests (`fic_lvgl_capture_framebuffer()`) directly inside flush/idle ticks without Core 1 mutex starvation.
  * **`task_display_and_audio` (`"RoverUI"`, Priority 2):** Exclusive `I2C0` + `I2C1` owner (`400 kHz`, `.scl_wait_us = 20000`, `fic_i2c_lock_port()` mutexes, 18-clock GPIO SCL bus-clear, automatic bus reinit, `0x61`/`0x71` sentinel filters): polls touch, `SEN0696` (`25-30 Hz` / `~3 Hz` baro), `QMI8658`, `PCF85063`, and `AXP2101`; runs audio guidance (`L/R` chirps + Geiger/Beeps), `CAL MAG` capture, and safe Auto-Off countdown/logger flush.
  * **`loopTask` (Priority 1):** Executes 5s debounced `save_settings()` with smart `FFat` skip (`non_index_settings_equal()`), hosts `fic_web_portal_handle()`, flushes PSRAM ring loggers (`fic_core_log` + `fic_ring_logger`), and executes 24-bit BMP screenshot writes (`fic_screenshot_save_bmp()`, `2`-row DMA chunk buffer, `FFat` fallback, `460,854 B` post-`close()` check).
* **Shared BSP & Core Library (`Field_Instrument_Core >= v1.0.1`):**
  * Provides `waveshare_s3_35b.h`, dual-bus `fic_i2c_bsp`, `tca9554`, `fic_lvgl_port`, `fic_screenshot`, `esp_codec_dev`/`codec_board`/`fic_audio_beeper`, `fic_qmi8658`, `fic_sen0696`, `fic_mmc5983ma`, `fic_pcf85063`, `fic_axp2101`, `fic_matrix_math`, `fic_ahrs_9dof`, `fic_fs_manager`/`fic_core_log`/`fic_ring_logger`/`fic_settings_json`, and `fic_web_portal`.

### Memory Partitioning (`v0.11.0` Compiled Output & PSRAM Allocation)
* **Binary Image:** `1,760,414 B` (`55%` of `3,145,728 B` app slot under `app3M_fat9M_16MB`), `0` compiler warnings (`--warnings default`).
* **8 MB Octal PSRAM:** Dual RGB565 framebuffers (`614.4 KB`), `serial`/`rtcm`/`nmea`/`route` ring buffers (`61.4 KB`), `g_waypoints` (`MAX_WAYPOINTS = 128`), `CAL MAG` sample array, and ~400+ LVGL 9 UI objects scoped via `heap_caps_malloc_extmem_enable(64)` -> `4096`.
* **Internal `DRAM0` / DMA SRAM:** Static global dynamic usage = **`68,712 B` (`20%` of `327,680 B`)** (bench-verified `~155 KB` free internal heap on `v0.10.10` prior to `v0.11.0` smoke flash).

---

## 6. UI Tile Structure (`320x480` Portrait, `v0.10.7`)

### Persistent Top Header Bar (`320x74`)
* **Row 1:** Date, 24-hr Time (`TZ`), Units (`SI`/`IMP`), Wi-Fi icon (`STA`/`AP`/`CONN`/`OFF`), and Power (`[USB] 4.85V+` / `[Bat] 85% 3.92V`).
* **Row 2 (`ANT1`):** Fix badge (`RTK FIX`/`FLT`/`DGPS`/`SINGLE`/`OFF`), `SV: used/view`, `+/- h_acc`, RTCM diff age (`RTCM: X.Xs`), and `LoRa: -XXdB`.
* **Row 3 (`ANT2` / Heading):** Heading badge (`SLV FIX`/`FLT`, `HDG CMP`, `HDG WLK`, `HDG GYR`, `HDG HLD`), Slave SVs, Baseline length (`BL: 55.4cm`), and `Hdg: XXX.X SRC`.
* **Row 4:** Ground-tip Lat, Lon, Height (`14pt` Montserrat).

### UI 6-Tile Symmetric Ring
`[Tile 5: Settings] <-> [Tile 0: Nav] <-> [Tile 1: Survey] <-> [Tile 2: Waypoints] <-> [Tile 3: Compass] <-> [Tile 4: Diag]`:

* **Tile 0 (Navigation) -- 2-Mode Unified Guidance Triangle & Map View (`v0.10.0 - v0.10.7`):**
  * Top waypoint bar (`<` / `Start -> Target [SWAP|SET A]` / `>`) + consolidated circular marquee `t0_notes_lbl` (`Deed:`/`Direct:` bearing/length/log-span in Guidance; `SEG: A -> B | Win: ... | Pts: ...` in Map View).
  * Expanded **`304 x 276 px`** canvas with single-row Top HUD: `[GUIDANCE]`/`[MAP VIEW]` toggle flush-left, centered `H:XXX.X P:+X.X` (**automatic dual-source pitch**: UM982 `baseline_pitch_deg` during `SLV FIX/FLT`, falling back to tare-compensated `SEN0696` beam pitch), and flush-right color-coded `B: XXX.X` azimuth (`<= 4 deg` Green / `<= 15 deg` Amber / White).
  * **Guidance (`ui_tile0_views.c`):** Supports **`A: DEED`** and **`A: HERE`** direct anchor. Plots `A` (`12 px`), `B` (`16 px`), `R` (`14 px`) on symmetric log cross-track ticks (`0x385266`, `3ft`/`30ft` or `1m`/`10m` labels `0x94A3B8`) with dynamic vertical auto-framing (`y = 50..206`, dashed extension segments when `R` is behind `A` or past `B`), `R -> B` prioritized barbed heading arrow, 3-tier vector steering chevrons (`>>>`/`<<<` & `|`), and dynamic `A`/`B`/`R` non-overlapping labels. Auto-switches (`< 4.8 m / > 5.3 m` hysteresis) into `BULLSEYE` w/ overlaid `A -> B` chord. Acoustic fade-out `L/R` chirps speed up & scale PCM gain toward line center and mute cleanly beyond `line_audio_fade_m` (`20.0 m`).
  * **Map View (`ui_tile0_map.c`):** Sticky persistent zoom (`map_zoom_m_per_px`), active segment highlight (`4 px` solid Emerald `DEED` / `3 px` dashed Violet `HERE`), dashed tie-lines, Gold/Orange pins, optional breadcrumb trail, barbed Rover heading arrow (`ui_tile0_draw_barbed_arrow()`), **Survey-Control Base Station symbol (`BS` hollow Magenta triangle + white center dot from `BASEINFOA`, excluded from `FIT`)**, dynamic `A`/`B`/`R`/`BS` labels, and a unified **bottom toolbar** (`+`, `-`, `FIT`, `CTR`, `REC OFF/ON`).
  * **Compact 2-Row Lower Readout (`308 px` circular marquee):** Row 1 shows `OFFSET LEFT/RIGHT | B: <dist>`; Row 2 shows **signed along-line progress `A: +|-x.x <unit> (x.x%) | Line->B: +|-x.x <unit>`** (`nav_format_along_line()` in `v0.10.7`, removing ambiguous civil `+` stationing).
* **Tile 1 (Survey) -- Dedicated Field Survey Logger:** `40 px` Prefix + Avg (`1s..30s`) + all `12` monument `kNoteTags`, `52 px` `[LOG]` button, Pre-Log Survey Quality card, Core 1 `pole_tilt_deg` capture, and circular `atan2` heading averaging.
* **Tile 2 (Waypoints) -- Multi-File CSV Browser:** Instant `< 25 ms` RAM stem-cached file/waypoint selector with `poly_order` boundary/tie rendering (`[SWAP]` auto-clear on target step).
* **Tile 3 (Compass) -- Dual-Antenna + Beam 9-DOF Compass & `CAL MAG`:** `224 px` Course-Up bubble level, `TARE`, left-side `CAL MAG` 360-deg spin wizard, heading provenance badges, `[IRON ANOMALY]` warning, `[BEAM 10DOF I2C1]`, and `Baro Rel Elev`.
* **Tile 4 (Diag) -- Dual-Bus `I2C0/I2C1` Observers:** Fixed header (`RTK ROVER v0.10.7`), LoRa RF survey card, Storage, `ANT1/ANT2`, Base Station (`BASEINFOA`), `SEN0696` (`Orient`, `FOC`, `3D-Cal`, `BMP581`) + `QMI8658` pose, dual `I2C0/I2C1` error counters, Auto-Off countdown, `UM982 COM1`, and `E220` pin self-test.
* **Tile 5 (Settings) -- Dedicated `5x3` (`94 x 44 px`) Quick Settings & Status Footer:** `VOL/AUD/BRI`, `SERIAL/NMEA/RTCM`, `TELEM/IMU/TOUCH`, `WIFI/UNITS/LORA`, `AUTO-OFF` (`5m/15m/30m/OFF`), `CAL MAG`, reserved `BLE: PLAN2`, and Wi-Fi / Auto-Off / LoRa footer, backed by the motion/`REC`/logger-safe `lv_layer_top()` 30s Auto-Off modal.

---

## 7. Python Survey & Ingestion Scripts (`scripts/`)

* **`e220_pacer.py` (`v3.7.1`):** Architecture 2B Asynchronous Ping-Pong Mailbox LoRa driver for RTKBase (`scripts/e220-pacer.service`, `scripts/update_e220_pacer.sh`). Uses a background ingress thread with per-type mailboxes, starvation-fair round robin anchored to `1074`, smart `<= 200 B` coalescing, Stage-1 piecewise `ToA + guard` pacing, `Second 0` pause (`--macro-cycle-s`, default `5`), empty MSM filtering (`<= 28 B`), `GPIO 22` watchdog, `TIOCEXCL` locking, Mode 0 noise probe, V3 `48 B` telemetry decoding + atomic JSON / rolling daily CSV logs, and 4-ACK 3-rate air switching (`4.8k / 9.6k / 19.2k`).
* **`stage2_bench_verify.py` (`v1.0.3`):** Standalone Architecture 2B verification suite (`--self-test` 7/7 tests, `--mode pi-feeder`, `--mode pi-bench`, `--mode rover-monitor`, `--from-log`).
* **`bench_e220_multirate.py` (`v1.0.3`) & `set_e220_power.py` (`v1.1.0`):** Stage 1 multi-rate hardware characterization suite and persistent runtime TX power synchronizer (`/etc/default/e220-pacer`).
* **`cogo_solver.py` (`v1.2.0`):** Schema-driven COGO traverse solver ([`docs/schemas/cadastral_schema.json`](schemas/cadastral_schema.json)) with Compass Rule adjustment, road curves, adjoiner reconciliation, and KML/CSV/GeoJSON/AutoCAD DXF export (see [`docs/cogo_solver_guide.md`](cogo_solver_guide.md)).
* **`check_secrets.py` (`v1.1.1`) & `install_git_hooks.py` (`v1.1.0`):** Pre-push privacy/secret scanner against generic patterns and local `.secrets.local` tokens.
* **`kml_to_waypoints.py` (`v1.0.0`) & `validate_waypoints.py` (`v1.2.0`):** KML converter and 19/20-column CSV schema + leg geometry validator.
