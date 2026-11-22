# Plan 0: Field Survey Refinements, 6-Tile Ergonomic UI, Auto-Off & SEN0696 I2C1 Fusion

> [!NOTE]
> **Status:** Active Immediate Execution Roadmap -- Phases 0.1-0.5 Complete in `v0.8.19`, Phase 0.6 (`v0.9.0`) Pending (November 22, 2026)  
> **Document Version:** `1.2.0`  
> **Baseline:** `RTK_Rover v0.8.19` | `CAD v2.2.2` | `e220_pacer.py v3.7.1`  
> **Target Releases:** `v0.8.19` (Released: LVGL 11-File Split, 6-Tile UI, Unified Line+Bullseye, L/R Audio, Safe Auto-Off, NTRIP Config Persistence) -> `v0.9.0` (Phase 0.6: `SEN0696` `I2C1 GPIO 47/48` Isolation & Calibrated Beam 9-DOF Fusion)  
> **Execution Order:** **Plan 0 (`v0.8.19` - `v0.9.0`)** -> [`Plan 1` (`Field_Instrument_Core` & Remaining Splits)](plan_1_shared_core_library_and_token_optimization.md) -> [`Plan 2` (`3.5B` BLE NUS, RTCM Arbiter & Android Slave Mode)](plan_2_rover_enhancements_and_android_companion.md)

---

## 1. Empirical Field Baseline & Audit Findings (`v0.8.18`)

Extended dense-canopy walking evaluation (`2h 40m`, `9,109` 1 Hz epochs, `7,481 ft` walked out to `1,319 ft` from base, `0` crashes) proved strong core RTK/LoRa stability while exposing `6` concrete field ergonomics, audio, sensor-bus, and configuration gaps:

| Metric / Subsystem | Measured Field Performance (`v0.8.18`) | Actionable Finding for Plan 0 |
| :--- | :--- | :--- |
| **GNSS & Baseline** | `92.8% RTK_FIX`, `96.9% Fix+Float`, median `h_acc = 2.7 cm`, `0.552 m` baseline (`15/15` logged pins `RTK_FIX`) | Core RTK fix/projection is rock-solid; fix Core 1 tilt capture & circular heading averaging on survey pins. |
| **LoRa RF Link (`4.8k/30dBm`)** | `14,772` verified RTCM3 frames (`98.94%`), `100.0%` Base uplink PDR, `-75 dBm` med RSSI | Keep Arch 2B RF link unchanged; deep-shadow `131/133` `200 B` truncs cleanly dropped by Gatekeeper. |
| **Canopy Heading Gap** | `42.2%` dual-antenna (`FIX 27.6% + FLT 14.6%`), `19.4% WALK`, **`38.4% HOLD`** | Static heading stalls in brush; pulling `SEN0696` `I2C1` + calibrated 9-DOF fusion into `v0.9.0` fills this gap. |
| **`SEN0696` Passive Data** | `8,639` samples (`Acc Z = -1.046g` inverted beam mount); 4-param hard-iron fit `5.4 deg` med error vs `HDG_FIX` | Huge rigid hard-iron offset `(-28.3, -26.8, +7.9) uT` confirms `360-deg` cal + `beam_imu_orient_mode` needed. |
| **Shared `I2C0` Bus** | `977` `err=0x103` stalls (`~6/min`: `793 IMU`, `116 10DOF`, `68 Touch`), `100%` recovered in `7 ms` | External `GX-12` cable capacitance loads `I2C0`; isolate `SEN0696` onto freed **`I2C1` (`GPIO 47/48`)** in Phase 0.6. |
| **Code Bugs Audited** | 5 verified code defects affecting field stakeout, power-off, and logging | Fix Line-Up heading arrow, `[SWAP]` target step lock, L/R audio sign drop, walking auto-off, & `kNoteTags` UI. |

---

## 2. Phase 0.1 -- `lvgl_port.c` (`4,225 L`) Modular Split First (Completed in `v0.8.19`)

### Why Split `lvgl_port.c` Ahead of Plan 0 UI Changes
1. Avoids editing 5 existing tiles and building a brand-new 6th tile (`Quick Settings`) inside a `4,225-line` / `193 KB` monolith (`5.3x` the `800 L` single-read limit).
2. Unlocks small, single-read `< 770 L` UI modules immediately for Plan 0 (`Tile 0/1/3/4/5`), Plan 1 Core extraction, and Plan 2 BLE/NTRIP indicators.
3. Preserves all `AGENTS.md` invariants untouched: `heap_caps_malloc_extmem_enable(64)` -> `4096` PSRAM widget routing, `LV_USE_FLOAT = 0` (`snprintf()` + `lv_label_set_text()`), `rover_lvgl_lock()/unlock()`, and zero `i2c_bsp` calls inside LVGL locks.

### Verified `11`-File Architecture under `src/lvgl_port/` (Every File `<= 764 L`)
* `lvgl_port_core.c` (`334 L`): QSPI `AXS15231B` init, `24`-line DMA bounce buffers, PSRAM wrapping helpers, LVGL task & mutex, touch read callback.
* `lvgl_ui_internal.h` (`180 L`): Shared widget pointers, tile switching helpers, palette constants, and `update_*` declarations across `6` tiles.
* `ui_header_nav.c` (`597 L`): Persistent `320x74` 4-row status header, `320x32` bottom nav bar (`6` tiles: `< Prev` / `Next >` ring wrapping), plus the 30s Auto-Shutdown modal overlay.
* `ui_tile0_guidance.c` (`727 L`), `ui_tile0_views.c` (`764 L`), `ui_tile0_map.c` (`432 L`): Tile 0 split cleanly across `3` `< 770 L` modules covering Point Guidance, Unified Line+Bullseye Stakeout, and 2D Map View.
* `ui_tile1_survey.c` (`277 L`): Dedicated Field Survey Logger tile + Pre-Log Survey Quality card.
* `ui_tile2_waypoints.c` (`121 L`): Multi-file CSV Waypoint Browser.
* `ui_tile3_compass.c` (`195 L`): Dual-Antenna Compass dial, `224 px` Pole Bubble Level, `TARE` & `CAL MAG`.
* `ui_tile4_diagnostics.c` (`654 L`): Dual-Power UPS, LoRa RF card, UM982 State/LOGLIST, Beam `10DOF` & Auto-Off status.
* `ui_tile5_settings.c` (`707 L`): Dedicated `5x3` Quick Settings Deck + Wi-Fi/Auto-Off/LoRa status footer.

**Phase 0.1 Gate (Verified):** Split completed with zero UI layout drift before Phase 0.2; full Phases 0.1-0.5 release `v0.8.19` compiles with `--warnings default` (`0` warnings, `54%` flash, `20%` static `DRAM0`).

---

## 3. Phase 0.2 -- 6-Tile Ergonomic UI: Dedicated Survey (Tile 1) & Settings (Tile 5)

### 3.1 Ring Navigation Symmetry (`Tile 0` at Center)
Adding a 6th tile (`Tile 5`) places **Quick Settings** directly `1` tap left (`< Prev`) of `Tile 0` Navigation and **Field Survey Logger** `1` tap right (`Next >`) of `Tile 0`:
`[Tile 5: Settings] <-> [Tile 0: Nav] <-> [Tile 1: Survey] <-> [Tile 2: Waypoints] <-> [Tile 3: Compass] <-> [Tile 4: Diag]`

### 3.2 Dedicated Quick Settings (`ui_tile5_settings.c`, `320 x 374 px`)
* **Preserved 3-Column Width & Horizontal Margins:** Keep exact **`94 px`** button width with `10 px` left/right outer margins and `9 px` column gaps (`x = 10, 113, 216`).
* **Larger Touch Targets & Matched Vertical Whitespace:** Increase button height from `27 px` -> **`44 px`** (`+63%` taller for reliable field hits) and vertical row gap from `8 px` -> **`10 px`** (matching horizontal whitespace; `54 px` vertical row pitch, `lv_obj_set_ext_click_area(..., 4)`).
* **`5` Rows x `3` Columns (`15` Button Slots, `y = 10..260`):**
  * Row 0 (`y = 10`): `VOL` | `AUD` | `BRI`
  * Row 1 (`y = 64`): `SERIAL` | `NMEA` | `RTCM`
  * Row 2 (`y = 118`): `TELEM` | `IMU` | `TOUCH`
  * Row 3 (`y = 172`): `WIFI` | `UNITS` | `LORA`
  * Row 4 (`y = 226`): **`AUTO-OFF`** (`5m / 15m / 30m / OFF`) | **`CAL MAG`** (shortcut to `Tile 3` ellipsoid fit) | **`BLE: PLAN2`** (reserved Plan 2 slot, disabled/dimmed until BLE bridge lands)
* **Spacious Status Panel (`y = 278..366`):** `1 px` divider at `y = 278`, moving `t1_wifi_status_lbl` here (`Wi-Fi` SSID/IP/SoftAP state) plus live Auto-Off idle timer & LoRa/RTCM summary. Allocated cleanly inside `heap_caps_malloc_extmem_enable(64)` -> Octal PSRAM (`0 B` internal `DRAM0` impact).

### 3.3 Dedicated Field Survey Logger (`ui_tile1_survey.c`, `320 x 374 px`) & Averaging Fixes
Removing the 12-button grid gives `Tile 1` room for unmistakable monument verification in tough woods:
* **Spacious Controls:** Prefix dropdown (`40 px` tall) + Averaging Duration button (`1s / 5s / 10s / 30s`), full-width Monument / Note Tag dropdown (`40 px` tall), larger **`52 px`** **`LOG PIN NOW`** button visibly echoing the active tag (`LOG: NO PIN - Search Pt` vs `LOG: Found Iron Pipe`), and a pre-log quality card (`Fix`, `H-Acc`, `Tilt`, `Sats`, active Target delta `dN / dE`).
* **Expanded `kNoteTags` ([`waypoint_manager.cpp`](../../src/waypoint_manager/waypoint_manager.cpp#L51-L62)):**
  `"(No Tag)"`, `"Found Iron Pipe"`, `"Found Capped Rebar"`, `"Found Stone Mon"`, `"Found Mag Nail"`, `"NO PIN - Search Pt"`, `"NO PIN - Marked Spot"`, `"Unknown Pin/Pipe"`, `"Set Temp Flag"`, `"Mag Anomaly Only"`, `"Obstructed Sky"`, `"Needs Verification"`.
  Fix the hardcoded `g_survey_note_tag_idx < 8` guard (`waypoint_manager.cpp:848`) to use `kNoteTagCount = sizeof(kNoteTags) / sizeof(kNoteTags[0])`.
* **Survey Math Fixes:**
  1. Forward latest Core 1 `pole_tilt_deg` safely into `RoverData_t` / atomic snapshot so `survey_logged_pins.csv` records actual beam tilt instead of `0.00`.
  2. Replace linear `true_heading_deg` summation with circular `atan2(sum_sin, sum_cos)` vector averaging (prevents `359/0 deg` wrap corruption).

---

## 4. Phase 0.3 -- 3-Mode Tile 0 (`Point` / `Unified Line+Bullseye` / `Map`) + Whitespace & Bugfixes

Cycle `s_tile0_nav_mode` cleanly across **3 views** on `t0_mode_badge` tap:
1. **Mode 0 -- Standalone Point Guidance / Bullseye:** Preserves existing Course-Up arrow (`> 1.5 m`) -> Fine Bullseye (`<= 1.5 m`).
2. **Mode 1 -- Unified Line + Bullseye Stakeout (Primary Field Boundary Mode):**
   * **Outside `5.0 m` (`16.4 ft`) from Target `B` (hysteresis: enter Bullseye `< 4.8 m`, exit back to Line-Up `> 5.3 m`):**
     * Shows the Line-Up vertical baseline `A -> B`, horizontal perpendicular cross-track tie-line, compact `Dist to B` bearing/distance cue, and **Line-Up heading arrow** rotated by **`d->true_heading_deg - stakeout.line_bearing_deg`** (fixing [`lvgl_port.c`](../../src/lvgl_port/lvgl_port.c#L2956-L2970), which currently draws unrotated North-Up heading against the vertical Line-Up track!).
   * **Inside `5.0 m` (`16.4 ft`) from Target `B`:**
     * Automatically transitions to **Bullseye Target View** centered on `B` (`5.0 m` outer ring / `1.5 m` inner ring / `2 cm` center; auto-zooms to `1.5 m` scale once within `1.5 m`) **with the `A -> B` property line chord overlay drawn straight across the Bullseye** at angle `line_bearing_deg`, showing exact pin distance and line offset simultaneously!
3. **Mode 2 -- 2D Map View:** Unchanged pan/zoom/polygon/tie canvas (fix `t0_map_ties` hide on Mode 1 entry).

**Generous Canvas Line-End Whitespace & Navigation Bugfixes:**
* On the `304 x 210 px` canvas (`t0_nav_canvas`), pull vertical baseline `A -> B` endpoints inward from `y = 35..185` to **`y = 46..178`** (`132 px` vertical span -> `46 px` top whitespace clear of HUD labels, `32 px` bottom whitespace clear of `t0_rec_btn`).
* Pull horizontal rover offset clamp inward from `[-120, +120] px` (`60 px/m`, `2.0 m` full-scale) to **`[-104, +104] px`** (`52 px/m`, **`48 px` whitespace** on both left and right canvas edges), and trim Bullseye crosshairs to `220 x 1 px` / `1 x 146 px`.
* **Additional Bugs Fixed:**
  * Clear `g_stakeout_custom_pair = false` inside `waypoint_select_next/prev/index()` ([`waypoint_manager.cpp`](../../src/waypoint_manager/waypoint_manager.cpp#L476-L505)) so stepping targets still advances `A -> B` after `[SWAP]`.
  * Display `stakeout.line_bearing_deg` in `t0_tgt_hdg_lbl` during Mode 1 instead of radial bearing.
  * Replace UTF-8 degree character in `nav_format_quadrant_bearing()` ([`nav_engine.cpp`](../../src/nav_engine/nav_engine.cpp#L222)) with pure ASCII `deg`, and format unsigned `|offset|` cleanly without redundant leading `+` on `"LEFT"/"RIGHT"`.

---

## 5. Phase 0.4 -- Directional Left/Right Line-Follow Audio & Precedence

Upgrade `RoverBeep_t` and Priority-4 `task_audio_beep` ([`RTK_Rover.ino`](../../RTK_Rover.ino#L40-L162)) on the physically mono `ES8311` (`24 kHz`, 16-bit I2S0):
1. **Single-Queue Dual-Tone Struct (`< 1 ms` Dispatch Preserved):**
   * Extend `RoverBeep_t` to `{ uint16_t freq1_hz; uint16_t dur1_ms; uint16_t gap_ms; uint16_t freq2_hz; uint16_t dur2_ms; }` (simple UI clicks set `freq2_hz = 0`), trimming trailing chunks `< 256` frames and outputting clean zero-filled PCM silence during `gap_ms` instead of dropping `freq == 0`.
2. **Signed Cross-Track Directional Contours (Faster Closer to Line):**
   * Remove the `fabs(stakeout.offset_m)` sign loss at `RTK_Rover.ino:555-568`. Preserve signed `offset_m` (`< 0` = Left of `A->B`, move RIGHT `>>>`; `> 0` = Right of `A->B`, move LEFT `<<<`):
     * **Left of Line (`offset_m < -0.05 m`, steer RIGHT):** **Rising 2-tone chirp** (`660 Hz, 30 ms` -> `25 ms` gap -> `1100 Hz, 40 ms`).
     * **Right of Line (`offset_m > +0.05 m`, steer LEFT):** **Falling 2-tone chirp** (`1100 Hz, 30 ms` -> `25 ms` gap -> `660 Hz, 40 ms`).
     * **On Line (`|offset_m| <= 0.05 m` / `~2 in`):** Crisp steady **single high pip** (`1800 Hz, 30 ms`).
   * **Distance-to-Line Cadence (speeds up as `|offset_m|` shrinks):** `<= 0.05 m` -> `400 ms` (`1800 Hz`), `0.05..0.25 m` -> `250 ms`, `0.25..0.75 m` -> `450 ms`, `0.75..1.50 m` -> `750 ms`, `1.50..3.00 m` -> `1100 ms`, `3.00..5.00 m` -> `1500 ms` (respecting `AUDIO_NAV_BEEPS` / `AUDIO_NAV_GEIGER` / `AUDIO_NAV_MUTE`).
3. **Unified Mode 1 Audio Handoff Precedence:**
   * Outside `5.0 m` (`16.4 ft`) from Target `B` (`> 5.3 m` hysteresis): **Line-Follow Left/Right Audio** leads (`|offset_m|` gate replaces outer radial `> 5 m` limit).
   * Crossing inside **`4.8 m` (`15.7 ft`)**: Play a distinct 2-pip acquisition chime (`1400 Hz -> 2000 Hz`) and hand off cleanly to **Bullseye Proximity Audio** (`distance_to_target_m` Geiger/Beeps). Returning outside `5.3 m` hands back to Line-Follow.

---

## 6. Phase 0.5 -- Field-Safe Auto Power-Off & `settings.json` NTRIP Persistence

1. **Field-Safe Auto Power-Off ([`RTK_Rover.ino`](../../RTK_Rover.ino#L440-L471)):**
   * **Expanded Activity Inhibits (reset idle timer whenever ANY holds):**
     * `pwr.vbus_good` (USB power) or `is_wifi_active()`,
     * `ui_state.survey_in_progress`,
     * **`route_logger_is_active()` (`REC ON`)**, **`nmea_logger_is_active()`**, or **`rtcm_logger_is_active()`**,
     * **Physical motion:** `GNSS tip displacement >= 1.5 m` from last anchor OR `QMI8658`/`SEN0696` gyro/accel delta above stationary noise floor (catches slow brush-walking `< 0.30 m/s` and canopy RMC gaps),
     * Touch activity or physical `BOOT` button press (wire `rover_note_user_activity()`).
   * **30-Second Warning Countdown & Graceful SD Flush:** Before cutting `AXP2101` power, raise an unmistakable top banner / modal `"AUTO POWER-OFF IN %us - TAP TO CANCEL"` (`30 -> 0 s`) with warning beeps (`30, 20, 10, 5..1 s`). At `0`, execute `route_logger_stop()`, `nmea_logger_stop()`, `rtcm_logger_stop()`, `serial_logger_stop()`, and `save_settings()` before `axp2101_bsp_power_off()`.
   * Wire Tile 5 **`AUTO-OFF`** button (`5m -> 15m -> 30m -> OFF -> 5m`) to `current_settings.auto_shutdown_battery_min` + `mark_settings_dirty()`.
2. **`settings.json` NTRIP Persistence ([`settings_manager.cpp`](../../src/settings_manager/settings_manager.cpp#L447-L505)):**
   * Extend `SystemSettings` (`~240 B` struct delta, internal static impact `< 250 B`) with:
     `rtcm_source_mode[16]` (default `"AUTO"`), `ntrip_host[64]` (`"rtk2go.com"`), `ntrip_host_ip[32]` (`"3.14.70.106"`), `ntrip_port` (`2101`), `ntrip_mountpoint[48]` (`""`), `ntrip_user_email[64]` (`""`), `ntrip_password[32]` (`"none"`).
   * Add placeholder-overwrite protection (mirroring `is_placeholder_ssid()`) and include all 7 fields in `non_index_settings_equal()` + `load_settings()` + `save_settings()`, so private `/settings.json` schemas deployed to SD survive all runtime saves before Plan 2 adds the BLE NTRIP client. *(Keep real mountpoints/emails strictly in `RTK_Rover_Private/profiles/settings.json`.)*

---

## 7. Phase 0.6 -- `SEN0696` `I2C1` (`GPIO 47/48`) Isolation & Beam 9-DOF Fusion (`v0.9.0`)

*(Pulled forward from Plan 2 Phase 2.1 per field finding: `977` `I2C0` stalls & `38.4% HLD` heading rate.)*
1. **Isolated `I2C1` Bus (`I2C_NUM_1` on Freed `GPIO 47/48`):**
   * Configure optional secondary `I2C1` bus in `user_config.h` and [`i2c_bsp.c`](../../src/i2c_bsp/i2c_bsp.c) (preserving 18-clock GPIO bus-clear, `.scl_wait_us = 20000`, `i2c_bsp_lock()`, and two-tier runtime recovery on both buses; retain `I2C0` fallback auto-probe if `SEN0696` is still strapped on `I2C0` on an un-rewired bench unit).
   * Separates `10.3 ms` `SEN0696` block transactions and external `GX-12` cable capacitance from internal `Touch(0x3B)`, `QMI8658(0x6B)`, `AXP2101(0x34)`, and `ES8311(0x18)`, enabling `20-40 Hz` AHRS polling without `I2C0` stalls.
2. **Pure Math & Instance-Safe 9-DOF AHRS Port (`src/matrix_math/` + `src/sensor_fusion/`):**
   * Port `Magnetic_Field_Scanner`'s `matrix_math.h` (generalized `float`/`double` 9-param ellipsoid fit, `jacobi_eigen_3x3`, `get_calibration_matrices`, `kabsch_align_3x3`) and refactor `MadgwickAHRS` + 50-sample stationary accel-variance gyro FOC into instance-safe `ahrs_9dof` (seeded cleanly for Plan 1 Core extraction!).
3. **Beam Alignment, Ellipsoid `CAL MAG` & Heading Fusion (`nav_engine` + UI):**
   * Add JSON-configurable `3x3` signed `beam_imu_orient_mode` (`settings.json`), supporting the verified `-Z` inverted beam mount (`Acc Z = -1.046g`), plus centripetal accel gate (`| ||a|| - 1g | < 0.12g` & `|omega| < 12 deg/s`).
   * Implement `Tile 3` / `Tile 5` **`CAL MAG`** 360-deg spin ellipsoid wizard solving hard-iron offset + `3x3` soft-iron matrix (persisted in `settings.json`), online `UM982 HDG_FIX` circular True-North + `B_ref`/`Bz_ref` teacher, triple-condition iron anomaly gate (`|B|`, `Bz`, gyro-mag innovation -> bias-compensated `HDG_SRC_GYRO`/`HOLD`), `BMP581` `Baro Rel Elev` (`no beam_temp_c` -- not in `SEN0696` I2C bridge), and Tile 3/4 `Beam 10DOF` status (`MMC5983MA` `0x30` kept as Tier-B fallback).
   * Keep `48 B` `RoverUplinkTelemetry_t` binary-compatible (`heading_source = 7`) and keep `QMI8658` dedicated to enclosure `DisplayPose_t` whenever `SEN0696` is online.
4. **Wiring Diagram & Pin Allocation Documentation Sync (`GPIO 47/48` -> `I2C1`):**
   * Update [`scripts/generate_wiring_diagrams.py`](../../scripts/generate_wiring_diagrams.py) (bump `__version__`) and regenerate `docs/schematics/wiring_diagram_3_5B_current.svg` / `.png` showing the `GX-12` bulkhead `SEN0696` `SDA/SCL` routed to dedicated `I2C1` (`GPIO 47/48`) instead of shared `I2C0` (`GPIO 7/8`), including external pull-up and Gravity 4-pin color-code notes.
   * Synchronize [`docs/schematics/system_wiring_schematics.md`](../schematics/system_wiring_schematics.md), [`docs/project_overview.md`](../project_overview.md), and the `3.5B` header pin-allocation artifacts (`docs/schematics/RTK_Rover_3.5B_Header_Pin_Mapping-rev4.{csv,xlsx}` and `RTK_Rover_3.5B_Header_Wiring_Guide-rev4.png`, bumping to `rev5` or updating in place) alongside `user_config.h` (`ROVER_PIN_I2C1_SDA` / `ROVER_PIN_I2C1_SCL`).

---

## 8. Plan 0 Acceptance Gates (`v0.8.19` -> `v0.9.0`)

1. **Build & Memory Gate:** `arduino-cli --warnings default` = **`0` warnings** after Phase 0.1 and each subsequent phase; static `DRAM0 <= 25%` (`< 80 KB`); `[MEM] Post-Setup Heap` internal free `>= 150 KB`; all files in `src/lvgl_port/` `<= 800 L`.
2. **UI & Ergonomics Gate:** Verify `6`-tile wrap symmetry, `5x3` `94x44 px` Tile 5 touch hit reliability, whitespace margins on `Tile 0` `304x210` Line+Bullseye canvas, Line-Up heading arrow sign against rotated tracks, and `<`/`>` advancement after `[SWAP]`.
3. **Audio & Power Gate:** Verify rising (`Left -> move Right`) vs falling (`Right -> move Left`) vs steady (`On-Line`) tones, `16.4 ft` (`5.0 m` w/ `4.8/5.3 m` hysteresis) audio transition, `REC ON` / motion auto-off inhibit, 30s countdown cancel, and clean SD logger close before PMIC shutdown.
4. **Config, Sensor & Docs Gate:** Verify `settings.json` round-trip preserves private `ntrip_*` fields + `cal` fields without overwrite; verify `0` `I2C0` stalls over 30-min `I2C1` soak + `< 2 deg` static beam compass accuracy (`HDG_FIX` dropout -> `[CMP]` switch in `0.0 s`); visually inspect regenerated wiring diagrams and confirm pin-allocation table parity with `user_config.h` before releasing `v0.9.0`; run `python scripts/check_secrets.py` (`[PASS] 100% CLEAN`) and release sequentially via `RTK_Rover` SemVer tags.
