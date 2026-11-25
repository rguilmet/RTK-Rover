# Plan 2: 3.5B Rover Sensor Enhancements, BLE/NTRIP Bridge & Android Companion ("Slave Mode")

> [!NOTE]
> **Status:** Active Execution Roadmap (Updated November 25, 2026)  
> **Document Version:** `1.4.0`  
> **Baselines:** `RTK_Rover v0.11.0` | `Field_Instrument_Core v1.1.1` | `CAD v2.2.4` | `e220_pacer.py v3.7.1`  
> **Companion Protocol Spec:** [`../../libraries/Field_Instrument_Core/docs/fic_companion_protocol.md`](../../libraries/Field_Instrument_Core/docs/fic_companion_protocol.md) (**FICP `v1.0.0`**)  
> **Prerequisite Plans:** [`archive/plan_0_field_survey_refinements.md`](archive/plan_0_field_survey_refinements.md) (**COMPLETE & ARCHIVED**) -> [`archive/plan_1_shared_core_library_and_token_optimization.md`](archive/plan_1_shared_core_library_and_token_optimization.md) (**COMPLETE & ARCHIVED `2026-11-24`**: `Field_Instrument_Core v1.1.1`, `RTK_Rover v0.11.0`, `Magnetic_Field_Scanner v6.0.1`, `-Target All` CI gate)  
> **Consolidates:** Phases 3-5 of [`archive/plan_beam_10dof_and_modularization.md`](archive/plan_beam_10dof_and_modularization.md), Phases 7.1-7.5 of [`archive/plan_phase7_cellular_ntrip_bridge.md`](archive/plan_phase7_cellular_ntrip_bridge.md), open Rover field/portal findings (`F-03`, `F-11`) from archived `project_review_sonnet.md`, residual Plan 0 outdoor verification (`G1`/`G5`), and Sections 4-6 of [`archive/plan_smart_pole_and_android_app.md`](archive/plan_smart_pole_and_android_app.md) (retiring the `3.49B-v2` hardware migration).

---

## 1. Executive Summary, Hardware Lock & Standalone Preservation

### 1.1 Locked Hardware Decision (No `3.49B-v2` Migration)
Per project direction, the Rover keeps its proven **`Waveshare ESP32-S3-Touch-LCD-3.5B`** hardware platform as-is rather than migrating to `Waveshare 3.49B-v2`:
* **Core Compute & HMI:** `Waveshare ESP32-S3-Touch-LCD-3.5B` (`320x480` `AXS15231B` QSPI display/touch, onboard `QMI8658` `0x6B` IMU, `PCF85063` RTC, `AXP2101` PMIC, `TCA9554`, `ES8311` + speaker, `MicroSD` + `FFat`).
* **GNSS & Radio:** Holybro `UM982` Single-UART **`COM1` (`Serial1`, `GPIO 17/18` @ `115200`; `COM2`/`Serial2` retired, `GPIO 47/48` freed for `I2C1`)** + Ebyte `E220-900T30S/D` 1W LoRa (`4.8 kbps / 30 dBm` CH 75 field default, 3-rate `4.8k/9.6k/19.2k` Arch 2B) + switched `+5.00V` power stage (`NDP6020P + 2N2222 + MT3608`) inside the **`v2.2.4` support-free PETG enclosure** (`120 x 180 x 28 mm`, Option B2 `5,000 / 10,000 mAh` pod, `37.5 deg` open-truss tilt wedge, `GX-12` downward D-flat, left-wall `3.5 mm` jack tab).
* **Beam Orientation Sensors (`GX-12` Bulkhead):** DFRobot Gravity `SEN0696` (`BMI323 + BMM350 + BMP581`, `0x4A..0x4D`), isolated on **`I2C1` (`GPIO 47/48` @ `400 kHz`)** polling at `25-30 Hz` (`~3 Hz` baro) with `ahrs_9dof` (`v0.9.0..v0.9.2`) + automatic `I2C0` bench fallback; legacy `MMC5983MA` (`0x30`) retained as Tier-B fallback.

### 1.2 100% Standalone Guarantee + Bidirectional FICP Companion ("Slave Mode")
All `6` standalone LVGL tiles (`v0.11.0`: 2-mode Unified Tile 0 `GUIDANCE <-> MAP VIEW`, Dedicated Tile 1 Survey Logger, Tile 2 Waypoint Browser, Tile 3 Compass/`CAL MAG`, Tile 4 Diagnostics, and Tile 5 `5x3` Quick Settings), tactile L/R acoustic fade-out & Geiger guidance, LoRa RTK, `1 Hz` route logging, and multi-epoch survey logging MUST remain **100% functional without a phone**. When linked over **BLE 5.0 via FICP `v1.0.0`** ([`fic_companion_protocol.md`](../../libraries/Field_Instrument_Core/docs/fic_companion_protocol.md)), the `3.5B` Rover operates as a smart companion sensor head (**Slave Mode** / **Mirror Mode**) alongside `RTK-Rover-Mobile` (`5-10 Hz` telemetry, bidirectional settings/state reflection, cellular NTRIP RTCM3 over `STREAM_RX`, live stakeout targets, remote survey/route triggers, LoRa rate switches, file sync) with a `5 s` link-loss fail-safe and one-tap `[LOCAL CONTROL]` instant reclaim on the `3.5B` screen.

### 1.3 Verified `v0.11.0` Resource & Coexistence Envelope
* **Flash (`app3M_fat9M_16MB`):** `1,760,822 B` used (`55%` of `3,145,728 B` -> `~1.38 MB` free).
* **Static `DRAM0`:** `68,728 B` (`20%` of `327,680 B` -> well below the `25% / 80 KB` gate).
* **Runtime Internal `DRAM0` (Wi-Fi OFF):** `~155 KB` free (`~110 KB` largest contiguous block, `~147 KB` DMA) + `~7.38 MB` free Octal PSRAM.
* **NimBLE vs. Wi-Fi Allocation Reality:** Arduino-ESP32 `3.3.12` prebuilds NimBLE (`CONFIG_BT_NIMBLE_ENABLED = 1`, `CONFIG_BT_NIMBLE_MEM_ALLOC_MODE_INTERNAL = 1`, consuming `~35-55 KB` internal `DRAM0`), whereas active Wi-Fi STA/HTTP consumes `~41 KB` at mode init and `~65-80 KB` peak. Running both concurrently leaves thin fragmented SRAM (`~30-45 KB`), so **Plan 2 enforces strict BLE vs. Wi-Fi Web Portal mutual exclusion**.

---

## 2. Part A -- Current-Hardware Rover Sensor, Kinematics & Power Enhancements

### Phase 2.1 -- Outdoor `I2C1` / Beam Compass Qualification, Lever-Arm & Survey Quality Hardening
*(Core `SEN0696` `I2C1 GPIO 47/48` isolation, `matrix_math` ellipsoid `CAL MAG`, `ahrs_9dof` w/ stationary gyro FOC, `BMP581` elevation, and dual-source `P:` beam pitch fallback shipped in `v0.9.0..v0.10.5`; Plan 2 completes outdoor qualification & survey precision guards.)*

1. **Outdoor `I2C1` & Beam Compass Validation (Plan 0 Carry-Over -> Gate G1):**
   * Execute a 30-minute outdoor `I2C1` field soak verifying `0` `I2C0` stalls, `< 2 deg` static beam compass accuracy (`HDG_SRC_COMP`) against `UM982 HDG_FIX`, `<= 1.0 deg` pitch agreement, and `0.0 s` `HDG_FIX -> [CMP]` canopy switchover.
2. **Calibrated Beam Lever-Arm Guard (`nav_compute_pole_tip_position()`):**
   * Once outdoor trials verify `< 2 deg` beam compass accuracy under `HDG_SRC_COMP`, qualify enabling opt-in `apply_lever_arm_on_calibrated_beam` so the Master -> Pole-Center lever-arm stays active cleanly when `ANT2` loses `SLV FIX`.
3. **Survey Averaging Quality Hardening (`F-03` from Archived Review):**
   * Track `survey_best_fix` (preventing transient mid-window float dips from degrading a `10 s`/`30 s` `RTK_FIX` pin or misreporting final fix quality), enforce per-GGA epoch deduplication (avoiding duplicate-epoch weight distortion at 30 Hz queue ticks), compute Welford positional standard deviation (`h_sigma_m`), and flag/gate excessive `pole_tilt_deg` during logged monument averages.
4. **Core Refactor Alignment:** Consume Plan 1's shared `Field_Instrument_Core` (`FicSen0696_t`, `FicAhrs9Dof_t`, `fic_matrix_math`).

### Phase 2.2 -- Proactive Low-Battery Data-Loss Protection & `AUTO-OFF` Verification (`AXP2101`)
* Building on Plan 0's clean SD logger stop inside auto-off, debounce `AXP2101` `VBAT < 3.40 V` (`5 s` continuous while discharging on battery) in `rover_power` to flush/close open `route_*.csv`, `rtcm`, `nmea`, and `serial` SD log handles, commit dirty atomic `settings.json`, display a low-battery warning, and preserve all survey data before PMIC under-voltage cutoff.
* Verify battery `AUTO-OFF` motion/`REC`/logger inhibits, `30 s` cancel modal, and clean SD flush on physical battery hardware (Plan 0 Carry-Over -> Gate G5).

---

## 3. Part B -- Core FICP Engine & `3.5B` Firmware: BLE 5.0, RTCM3 Arbiter & Bidirectional "Slave Mode"

### Phase 2.3a -- Generic FICP Engine in `Field_Instrument_Core` (`v1.2.0`, `src/ble/`)
Implement the reusable **Field Instrument Companion Protocol (`FICP v1.0.0`)** per [`libraries/Field_Instrument_Core/docs/fic_companion_protocol.md`](../../libraries/Field_Instrument_Core/docs/fic_companion_protocol.md) so `RTK_Rover`, `Magnetic_Field_Scanner`, and future Waveshare ESP32-S3 instruments share one BLE transport and sync engine:
1. **Custom FIC GATT Service & PSRAM Buffering (`src/ble/fic_ble_companion.h/.cpp`):**
   * Expose Primary Service `5f494350-0001-4b10-a8e1-464943500001` with `CTRL_RX` / `CTRL_TX` (`0xAA 0x55` `CRC-16/CCITT-FALSE` FICP frames, `MTU 517`) and optional isolated `STREAM_RX` / `STREAM_TX` raw binary pipes. Benchmark built-in Arduino-ESP32 `3.3.12` NimBLE vs. `NimBLE-Arduino` and select the smaller internal `DRAM0` footprint.
   * Pin NimBLE controller/host & `STREAM_RX` ingestion to `FIC_CORE_ACQ` (`0`) and execute deferred persistence/file writes on `FIC_CORE_UI` Priority `1` (`loop()`). Allocate all RX/TX/reassembly/stream ring buffers (`>= 1 KB`) strictly in **Octal PSRAM**.
2. **Core Standard Messages (`0x00..0x7F`) & Link-Loss Fail-Safe:**
   * Implement `MSG_SYS_HELLO`/`CAPS`/`HEARTBEAT`/`ACK`/`NACK`/`HEALTH`/`RTC_SET`/`SCREENSHOT_REQ`, monotonic `state_rev` bidirectional settings/runtime reflection (`MSG_STATE_SNAPSHOT_REQ/RSP`, `MSG_CONFIG_UPDATE`, `MSG_STATE_EVENT`), chunked atomic file transfer (`0x30..0x34`), and `MSG_CONTROL_MODE_SET`/`EVENT` (`standalone | companion_mirror | slave_dimmed`) with a **`5 s` heartbeat watchdog** that automatically restores local backlight, audio, and `standalone` mode on disconnect.
3. **Core-Enforced BLE <-> `fic_web_portal` Mutual Exclusion & Host Tests:**
   * Update `fic_web_portal_start()`/`stop()` in `Field_Instrument_Core` to cleanly tear down active NimBLE before `WiFi.mode()` (reclaiming `~35-55 KB` internal `DRAM0`) and restart BLE when the portal stops.
   * Add FICP frame codec, CRC-16, `state_rev`, and watchdog unit tests to `extras/unit_tests/run_tests.py` and verify `-Target All` across `CoreSmoke_35B`, `RTK_Rover`, and `Magnetic_Field_Scanner`.

### Phase 2.3b -- `3.5B` Rover Companion Integration (`src/ble_bridge/`), Radio Toggle & Portal Hardening (`F-11`)
1. **Rover Profile Binding (`"RTK-Rover"` / `device_type = "rtk_rover"`):**
   * Register `RTK_Rover`'s `FicBleCompanionCfg_t` callbacks (`snapshot`, `config_update`, `control_mode`, `ext_rpc`, `stream_rx`, `telem_pack`), enabling `STREAM_RX` for NTRIP `RTCM3` and `CTRL_TX` for `5-10 Hz` telemetry.
2. **Radio Mode Selection & `F-11` Portal Guard:**
   * Field default: **BLE ON, Wi-Fi OFF** (`>= 100 KB` free internal `DRAM0` target after NimBLE init).
   * Wire the reserved Tile 5 Row 5 slot on `ui_tile5_settings.c` cleanly (`BLE` / `WIFI`), leveraging Core's mutual-exclusion teardown/restart on toggle.
   * **F-11 (SoftAP & Portal Guard):** Store SoftAP configured/default credentials strictly inside `/settings.json` (eliminating hardcoded `"12345678"` fallback strings) and reject destructive Wi-Fi portal file writes/deletes while active field operations (`REC ON` / survey averaging) are running.

### Phase 2.4 -- Core 0 Dual-Source RTCM3 Arbiter (`src/rtcm_arbiter/`)
* Hook both `e220_rtcm_gatekeeper.cpp` (replacing direct calls to `um982_forward_rtcm()`) and `fic_ble_companion`'s `STREAM_RX` callback (`rtcm_arbiter_submit(source, buf, len)`) into Core 0's shared **`CRC24Q == 0` Gatekeeper** -> **`um982_forward_rtcm()`** on **`UM982 COM1` (`Serial1`, `GPIO 17/18`)** protected by `s_com1_tx_mutex`. Zero `COM2` references. Using dedicated `STREAM_RX` isolates raw `0xD3` RTCM3 ingestion from `CTRL_RX` command framing.
* **Source Selection Modes (`settings.json` Schema v2 + FICP `MSG_CONFIG_UPDATE` / `0x88`):**
  * `AUTO` (Default): **LoRa primary**. If `now - last_lora_valid_rtcm_ms > 3000 ms`, immediately promote `BLE NTRIP` frames (`NTRIP: BLE`). Switch back to LoRa only after LoRa delivers `>= 3` consecutive valid frames (prevents boundary oscillation).
  * `LORA_ONLY` / `NTRIP_ONLY`: Explicit user override from Tile 5/Tile 4 or Android app (reflected both ways via `MSG_STATE_EVENT`).
* **`settings.json` NTRIP Schema & Persistence (`Schema v2` Shipped):**
  * Consumes the NTRIP keys persisted inside `/settings.json` (`rtcm_source_mode`, `ntrip_host`, `ntrip_host_ip`, `ntrip_port`, `ntrip_mountpoint`, `ntrip_user_email`, `ntrip_password`; private profiles stored strictly in `RTK_Rover_Private/profiles/settings.json`).
* **Non-Destructive `3.5B` UI Badging:** Header Row 1 adds `BLE` icon (Cyan connected / Amber adv / Dim OFF); Header Row 2 displays correction source (`LoRa -84dB` vs. `NTRIP BLE` vs. `NO CORR`) + diff age; `ui_tile4_diagnostics.c` adds `BLE & NTRIP Diagnostics` card (peer state, MTU, RX kbps, failover count).

### Phase 2.5 -- Rover FICP Extension Profile (`0x80..0xFF`) & Bidirectional State Reflection
Per [`fic_companion_protocol.md`](../../libraries/Field_Instrument_Core/docs/fic_companion_protocol.md), generic handshake (`0x01..0x05`), health (`0x06..0x07`), `PCF85063` RTC sync (`0x08`), screenshot (`0x09`), two-way configuration/state sync (`0x10..0x13`), Master-Slave dim/mute/reclaim (`0x20..0x21`), and chunked SD file transfer (`0x30..0x34` for `waypoints.csv`, parcels, and logs) are inherited directly from `Field_Instrument_Core`. Rover implements its `0x80..0xFF` domain extension profile:
* **Uplink (`0x81` `MSG_ROVER_TELEM`, Device -> Android @ `5-10 Hz` over `CTRL_TX`):**
  * Packed Little-Endian payload (`~96 B`, single `MTU 517` notify, `CRC-16/CCITT-FALSE` inside FICP envelope) carrying `3.5B` + `SEN0696` + Base Station + Slave state:

| Offset / Field | Type | Content (`3.5B v0.12.0+`) |
| :--- | :--- | :--- |
| `ts_ms` | `uint32` | Monotonic instrument timestamp (`ms`) inside FICP `0x81` payload |
| `tip_lat_deg, tip_lon_deg` | `double x2` | **64-bit `double`** ground-tip WGS84 (`%.8f` fidelity) |
| `base_lat_deg, base_lon_deg, base_id` | `double x2, uint16` | **64-bit `double`** live `BASEINFOA` Base Station coords + ID (`BS` map symbol) |
| `height_msl_mm, baro_rel_mm, undul_cm` | `int32/32/16` | Ground-tip MSL elevation, `BMP581` delta, geoid |
| `fix_type, ant2_fix, hdg_src, sv1, sv2` | `uint8 x5` | ANT1/ANT2 fix enums, `HDG_SRC_*`, tracked SVs |
| `hdg_cd, bl_mm, bl_pitch_cd` | `uint16/16/int16` | True heading (`0.01 deg`), baseline mm, crossbar pitch |
| `beam_pitch_cd, beam_roll_cd, flags` | `int16/16/uint8` | Attitude (`SEN0696`/`QMI8658`), `mag_anomaly`, `base_valid`, `slave_active` |
| `h_rms_mm, v_rms_mm, diff_age_ds` | `uint16 x3` | Horizontal/vertical 1-sigma RMS mm, RTCM age (`0.1 s`) |
| `rtcm_src, rtcm_mode, air_rate, rssi, noise` | `uint8/int8` | Active correction source/mode, E220 rate, RSSI & noise dBm |
| `vbat_mv, batt_pct, pwr_flags` | `uint16/8/8` | **`AXP2101`** VBAT mV, SOC %, `VBUS_present`/`charging` |
| `wp_idx, survey_sec, sd_flags` | `uint16/8/8` | Active target index, averaging countdown, SD/rec bits (framed by FICP `CRC16`) |

* **Downlink (`0x82..0x8A`, Android -> Rover Domain RPCs) & Domain Events (`0xA1`):**
  * **Bidirectional Reflection Hook:** Whenever local `3.5B` LVGL touch inputs modify mirrored settings (`lora_air_rate_bps`, `rtcm_source_mode`, pole geometry, active `wp_idx`, `REC ON/OFF`), firmware invokes `fic_ble_companion_notify_local_change()` -> `MSG_STATE_EVENT` (`0x13`) so Android updates live; whenever Android sends `MSG_CONFIG_UPDATE` (`0x12`), Rover validates, commits to runtime + debounced Priority-1 `settings.json`, and refreshes visible LVGL widgets cleanly.

| ID | Symbol | Rover Action (`3.5B` Side, Aligned w/ `v0.11.0+` Segment Model) |
| :--- | :--- | :--- |
| `0x82` | `CMD_ROVER_SET_TARGET_POINT` | Push ad-hoc target `(lat_deg, lon_deg, label[24])` directly into `nav_engine` + `ActiveGuidanceSegment_t` (`A: HERE -> Target` on Tile 0) + emit `MSG_STATE_EVENT`. |
| `0x83` | `CMD_ROVER_SET_LINE_STAKEOUT` | Push two endpoints `(A -> B)` for Deed Line Stakeout (`A: DEED` / `[SWAP]`) directly onto `ActiveGuidanceSegment_t` + Tile 0. |
| `0x84` | `CMD_ROVER_SELECT_WAYPOINT` | Synchronize active CSV index (`wp_idx`) / filename bidirectionally with `Tile 2`. |
| `0x85` | `CMD_ROVER_SURVEY_LOG` | Start (`duration_s`, `prefix`, `note`) or abort multi-epoch pin averaging; stream epoch progress & averaged pin result (`0xA1` `EVT_ROVER_SURVEY_RESULT`) back to Android. |
| `0x86` | `CMD_ROVER_ROUTE_REC_SET` | Toggle `1 Hz` `route_*.csv` recording (`ON/OFF`) on Rover SD (triggers `MSG_STATE_EVENT`). |
| `0x87` | `CMD_ROVER_RADIO_SET_RATE` | Trigger Rover's native `4.8k / 9.6k / 19.2k` LoRa dynamic rate switch (returns ACK/result + `MSG_STATE_EVENT`). |
| `0x88` | `CMD_ROVER_RTCM_SET_MODE` | Switch `rtcm_arbiter` (`AUTO / LORA_ONLY / NTRIP_ONLY`, triggers `MSG_STATE_EVENT`). |
| `0x89` | `CMD_ROVER_SENSOR_CAL` | Trigger `TARE` pole level or start/complete `Tile 3` `CAL MAG` ellipsoid routine. |
| `0x8A` | `CMD_ROVER_POLE_GEOM_SET` | Sync `antenna_spacing_m` and `antenna_height_m` (plus `line_log_max_m`/`line_audio_fade_m`) into `settings.json` & `nav_engine` (also supported via `MSG_CONFIG_UPDATE`). |

*(Control handoff `MSG_CONTROL_MODE_SET` `0x20` handles `[LOCAL CONTROL]` instant reclaim, `10-20%` dimmed HUD backlight, speaker `MUTE`/`FOLLOW`, and `5 s` heartbeat fail-safe; `MSG_FILE_*` `0x30..0x34` handles `waypoints.csv` / parcel / `settings.json` transfers so Android reads NTRIP caster settings on connect without needing field Wi-Fi.)*

---

## 4. Part C -- Native Android Companion App (`RTK-Rover-Mobile`)

Built in **Kotlin + Jetpack Compose** targeting sunlight-readable phones (`1,500-2,600 nits`) atop the reusable **`fic-ble-sdk`** FICP client library (`Phase 2.6`), communicating via the custom FIC GATT service (`CTRL_RX/TX` + `STREAM_RX`):

```mermaid
flowchart LR
    subgraph Android["RTK-Rover-Mobile (Android Service + Compose UI)"]
        NTRIP["Cellular NTRIP Client\nrtk2go:2101 + CRC24Q + Whitelist"]
        SDK["fic-ble-sdk (Reusable FICP v1.0.0)\nMTU 517, <1.5s Reconnect, FicStateStore"]
        MAP["Mapping & Parcel GIS\nSat/Ortho + Offline MBTiles\nCSV/KML/GeoJSON/DXF + BS Symbol"]
        STAKE["Unified Guidance + Map Audio\nLog-Triangle / Auto-Bullseye / Map\nTTS Voice + L/R Fade + Geiger"]
        CTRL["Rover Control & Room DB\nCrash-Safe Points + Rate/Cal/Dim"]
    end
    NTRIP -->|Raw RTCM3 via STREAM_RX| SDK
    MAP <-->|Tap-to-Stake / CSV Sync| CTRL
    CTRL <-->|FICP CTRL_RX/TX 0xAA 0x55| SDK
    SDK <-->|BLE 5.0 FIC GATT| ROVER["RTK_Rover 3.5B\nCOM1 + E220 LoRa + SEN0696"]
```

### Phase 2.6 -- `fic-ble-sdk`, Foreground Service & Cellular NTRIP Engine
* **Reusable `fic-ble-sdk` Library Module:** Implement `FicBleClient`, `FicFrameCodec` (`11 B` `0xAA 0x55` + `CRC-16/CCITT-FALSE`), `FicStateStore` (reactive `StateFlow` automatically reflecting both remote `MSG_CONFIG_UPDATE` commits and local touchscreen `MSG_STATE_EVENT` pushes + automatic `state_rev` gap recovery), `FicFileTransferClient`, and `FicStreamChannel`.
* **Android Foreground Service:** Persistent notification keeps BLE NTRIP streaming and point logging alive when pocketed or screen-locked (`WakeLock` option).
* **BLE 5.0 FIC Central Manager:** Auto-scans FICP service UUID `5f494350-0001-4b10-a8e1-464943500001` (`"RTK-Rover"`), requests `MTU 517`, completes `MSG_SYS_HELLO -> CAPS -> SNAPSHOT_REQ -> RTC_SET` startup, sends `1 Hz` heartbeats, and reconnects automatically within `< 1.5 s` after battery swaps or power cycles.
* **Cellular NTRIP Client (RTK2go / SNIP NTRIP v1.0):** Connects over cellular to `rtk2go.com:2101` (direct IP fallback `3.14.70.106:2101`), pre-populated from Rover `/settings.json` via `MSG_STATE_SNAPSHOT_RSP` on BLE connect (`ntrip_host`, `ntrip_host_ip`, `ntrip_port`, `ntrip_mountpoint`, `ntrip_user_email`, `ntrip_password`). Sends `GET /<MOUNTPOINT> HTTP/1.0` with `User-Agent: NTRIP RTK-Rover-Mobile/1.0` and `Authorization: Basic base64("<email>:<password_or_none>")` -- RTK2go requires a **valid email address** as username (fake/blank usernames trigger temporary IP bans) and **no password** for data consumers (`"none"` or empty). Validates `ICY 200 OK`, retries with exponential backoff (`1s -> 2s -> 4s -> 8s -> 15s`), filters strictly valid **`CRC24Q == 0`** RTCM 3.3 streams (`1005/1006, 1074/1077, 1084/1087, 1094/1097, 1124/1127, 1230` configurable whitelist), and streams raw frames over `FicStreamChannel` (`STREAM_RX`).

### Phase 2.7 -- Enhanced Mobile Mapping, Parcel GIS & Bidirectional Unified Stakeout
* **Sunlight Map + Offline Woodlands Support:** Interactive satellite/orthophoto base layers plus **offline `MBTiles` / cached `WMTS` region packs** for zero-cell woods operation.
* **Survey File Interop:** Native lossless parser/exporter for Rover `19/20-col` CSV (`poly_order > 0` closed cadastral boundary polygons, negative `-k` dashed tie-lines, `<DERIVED>` pins) alongside **KML, GeoJSON, and AutoCAD DXF** import/export.
* **Live GIS Overlay & Tap-to-Stake:** Renders sub-cm Ground Tip (`R`), Survey-Control Base Station symbol (`BS`, Magenta triangle from streamed `BASEINFOA` coords), rotating dual-antenna crossbar orientation (`UM982`/`SEN0696` + dual-source pitch), error circle (`h_rms`), active followed segment highlight, and GNSS track breadcrumb. **Tap any map vertex** or **select two vertices for `A -> B` Line Stakeout** and immediately push to the `3.5B` Rover via `CMD_ROVER_SET_TARGET_POINT` / `CMD_ROVER_SET_LINE_STAKEOUT`.
* **Unified Log-Scale Guidance & Audio (Aligned w/ Rover `v0.10.7`):** High-visibility Line-Up Log-Scale Triangle (`A: DEED` / `A: HERE`, `3-tier` steering chevrons `>>>`/`<<<` & `|`, `R -> B` prioritized barbed heading arrow, signed `A: +/-x.x (x%) | Line->B: +/-x.x` progress readout), automatic `< 4.8 m / > 5.3 m` Bullseye handoff with `A -> B` chord, plus Android **TTS spoken distance/direction calls**, acoustic `L/R` fade-out, and low-latency Geiger tone generator (coordinating `local_audio: "muted"` on Rover speaker via FICP `MSG_CONTROL_MODE_SET`).

### Phase 2.8 -- Crash-Safe `Room` Database & Remote Hardware Control Console
* **Crash-Safe Field Book (`Room` SQLite):** Stores every averaged point (`1s..30s` remote trigger, `<DERIVED>` `_Lxxx` naming, notes, fix/RMS/sigma/tilt/source metadata) in transactional SQLite on the phone with instant atomic CSV/KML/GeoJSON/DXF export, while synchronizing with Rover `/survey_logged_pins.csv` via FICP `MSG_FILE_*`.
* **Remote Control & Hardware Diagnostics Tab:** Dedicated phone control deck for `E220` RSSI/noise + `4.8k / 9.6k / 19.2k` rate switch, RTCM arbiter override (`AUTO/LORA/NTRIP`), `AXP2101` battery/charging status, ANT1/ANT2/Base observer, live `SEN0696`/`QMI8658` bubble level + `TARE`/`CAL MAG`, two-way `settings.json` Schema v2 sync (`MSG_CONFIG_UPDATE` / `MSG_STATE_EVENT`), FICP chunked waypoint CSV transfer, and `3.5B` `slave_dimmed` / `companion_mirror` control toggle.

---

## 5. End-to-End Verification Gates & Deferred Backlog (`Phase 2.9`)

| Gate | Quantitative Pass Criteria |
| :--- | :--- |
| **G1 (`3.5B` Outdoor `I2C1` & Compass)** | `0` unresolved `I2C0` errors over 30-min outdoor soak with `SEN0696` on `I2C1` (`GPIO 47/48`); `CAL MAG` `< 2 deg` static accuracy vs. `UM982 HDG_FIX`; `beam_pitch` vs. `baseline_pitch` `<= 1.0 deg`; `HDG_FIX` -> `SLV` dropout `[CMP]` switch in `0.0 s` (`< 2 deg` jump, `< 2 deg / 15 s` gyro iron coast); `stage2_bench_verify.py` `0` Trunc / `0` CRC. |
| **G2 (`DRAM0` & Core Radio Exclusion)** | `ci_compile_all.ps1 -Target All` + `run_tests.py` = `0` warnings; static `DRAM0 <= 25%` (`< 80 KB`); BLE active + Wi-Fi OFF free internal `DRAM0 >= 100 KB`; `20x` consecutive `BLE <-> WIFI` Tile 5 toggles leave `0` leaked tasks, `0` crashes, `F-11` SoftAP creds enforced, and clean reconnect. |
| **G3 (RTCM3 Dual-Source Arbiter)** | Killing LoRa pacer fails over to `BLE STREAM_RX` NTRIP within `<= 3.5 s` with **zero loss of `RTK FIX`** (`UM982 COM1`); resuming LoRa promotes back after `>= 3` clean frames; zero interference with concurrent `CTRL_RX/TX` traffic; verified via `stage2_bench_verify.py` and field ridge-shadow walk. |
| **G4 (Bidirectional FICP & Slave Mode)** | Phone tap-to-stake updates Rover `Tile 0` in `< 150 ms`; phone settings updates refresh Rover LVGL + commit atomically in `loop()`; local `3.5B` Tile 5 / Tile 2 edits reflect on Android via `MSG_STATE_EVENT`; remote `LOG PIN` (`F-03`), `REC ON/OFF`, `4.8k/9.6k/19.2k` rate switch, `CAL MAG`, and BLE CSV push all return verified ACKs; `3.5B` `[LOCAL CONTROL]` tap and `5 s` disconnect watchdog restore standalone control cleanly; Rover passes full field walk with Android phone completely OFF. |
| **G5 (Power & Data Integrity)** | Battery `AUTO-OFF` (`30 s` cancel modal, motion/`REC`/logger inhibits) & `AXP2101 < 3.40 V` test cleanly flush/close all SD logs + commit atomic `settings.json` before shutdown; zero lost points in Android `Room` DB under sudden Rover battery removal. |

### Deferred Optional Backlog (Lowest Priority)
1. **Phone-Hotspot Embedded Wi-Fi STA NTRIP Client:** Direct `WiFiClient` `rtk2go.com:2101` client inside `RTK_Rover` (Path 2 from archived `plan_phase7_cellular_ntrip_bridge.md`) for app-less field backup if an Android device is unavailable.
2. **LoRa `Arch 5` `4.8k` 2-Signal Pruner & `10 s` Rover Parity:** Optional Layer-7 RTCM3 MSM4 2-signal pruning in `scripts/e220_pacer.py` and `current_sec % 10U` macro-cycle parity in `e220_driver.cpp` (only if `> 12 SV` open-sky `4.8k` field surveys ever saturate 1-subpacket budgets).
