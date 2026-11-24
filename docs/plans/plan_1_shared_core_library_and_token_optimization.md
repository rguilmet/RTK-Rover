# Plan 1: Unified Shared Core Library (`Field_Instrument_Core`), Codebase Modularization & Token Optimization

> [!NOTE]
> **Status:** Active Execution Roadmap (Updated November 24, 2026)  
> **Document Version:** `1.2.0`  
> **Baselines:** `RTK_Rover v0.10.7` | `CAD v2.2.4` | `Magnetic_Field_Scanner v5.1.13`  
> **Completed Prerequisite:** [`archive/plan_0_field_survey_refinements.md`](archive/plan_0_field_survey_refinements.md) (`v0.8.19` - `v0.9.0`, refined through `v0.10.7` and archived `2026-11-24`: `lvgl_port.c` 11-file split, `I2C1` dual-bus `i2c_bsp`, `matrix_math`/`ahrs_9dof` port, `rtk-firmware-release`/`rtk-cad-pipeline` skills, and `build/`+`scratch/` purge completed).  
> **Companion Roadmap:** [`plan_2_rover_enhancements_and_android_companion.md`](plan_2_rover_enhancements_and_android_companion.md) (`v1.3.0`)  
> **Supersedes / Consolidates:** [`archive/plan_project_structure_and_token_optimization.md`](archive/plan_project_structure_and_token_optimization.md), Phase 1 of [`archive/plan_beam_10dof_and_modularization.md`](archive/plan_beam_10dof_and_modularization.md), residual F-19 `COM2` naming cleanup from [`archive/plan_phase9_gps_disciplined_tdma.md`](archive/plan_phase9_gps_disciplined_tdma.md), open Core-tier robustness findings (`F-12`, `F-13`, `F-14`, `F-17`) from archived `project_review_sonnet.md`, and Section 7 of [`archive/plan_smart_pole_and_android_app.md`](archive/plan_smart_pole_and_android_app.md).

---

## 1. Executive Summary & Architectural Scope

With the hardware locked on the **Waveshare ESP32-S3-Touch-LCD-3.5B** platform (retiring the `Waveshare 3.49B-v2` Smart Pole migration), this plan has two coupled objectives:

1. **Optimize AI Context & Token Efficiency:** Keep every active firmware module, documentation file, and planning roadmap compact enough to fit inside a single targeted file read (`<= 800 lines` and `< 46,080 bytes`), keep build/scratch clutter at `0 B` (`build/` and `scratch/` purged `2026-11-24`), prune obsolete stubs, and expand on-demand workspace skills.
2. **Extract a Separate Unified Library Repository (`Field_Instrument_Core`):** Eliminate `~65+` duplicated ESP-IDF BSP, display, audio, I2C, IMU, RTC, logger, Wi-Fi portal, and math/fusion files currently copied between **`RTK_Rover` (`3.5B`, `v0.10.7`)** and **`Magnetic_Field_Scanner` (`3.49 v3`, `v5.1.13`)** (and future Waveshare ESP32-S3 field instruments), shipping both consumers' hardest-won stability improvements into one SemVer Arduino library.

**Sequencing Note:** Archived [**Plan 0**](archive/plan_0_field_survey_refinements.md) (`v0.8.19` - `v0.9.0`, refined through `v0.10.7`) already completed the `lvgl_port.c` (`4,225 L`) modular split (`11` files `<= 792 L`), dual-bus `I2C0 + I2C1` `i2c_bsp` (`717 L`), `matrix_math` (`698 L`) / `ahrs_9dof` (`469 L`) port, lock-free framebuffer `screenshot.cpp` (`258 L`), workspace skills `rtk-firmware-release` / `rtk-cad-pipeline`, and `build/` + `scratch/` cleanup. **Plan 1 Phases 1.0 & 1.1** finish the remaining `< 800 L` file splits, Core decoupling, and `F-12`/`F-13`/`F-14`/`F-17`/`F-19` hardening **before** `Field_Instrument_Core` extraction (Phase 1.2) and [`Plan 2`](plan_2_rover_enhancements_and_android_companion.md) (`BLE` / Android Slave Mode).

---

## 2. Measured Codebase & Token Audit (`RTK_Rover v0.10.7`)

### 2.1 Files Exceeding the Single-Read Cap (`800 lines` / `46,080 bytes`)

| Tracked File | Bytes | Lines | Cap Overrun | Status / Disposition in Plan 1 |
| :--- | :---: | :---: | :---: | :--- |
| `CHANGELOG.md` | `247,369 B` | `1,371 L` | `5.37x B / 1.71x L` | Phase 1.0: Rolling archive split (`< v0.8.0` -> archive) |
| `src/lvgl_port/` (`11` files) | `max 34 KB` | `max 792 L` | **Compliant** | **Done in Plan 0 (`11` files <= 792 L)**; decouple `lvgl_port_core.c` in Ph 1.1 |
| `RTK_Rover.ino` | `80,693 B` | `1,531 L` | `1.75x B / 1.91x L` | Phase 1.1: Extract `rover_audio`, `rover_power` + `rover_sensors`/`CAL MAG` (`< 750 L`) |
| `src/e220_driver/e220_driver.cpp` | `68,372 B` | `1,451 L` | `1.48x B / 1.81x L` | Phase 1.1: Split into `3` units + internal header (`< 550 L` each) |
| `src/waypoint_manager/waypoint_manager.cpp` | `45,645 B` | `1,060 L` | `0.99x B / 1.33x L` | Phase 1.1: Split CSV store vs. survey logger (`2` files) |
| `src/um982_driver/um982_driver.cpp` | `47,460 B` | `1,020 L` | `1.03x B / 1.28x L` | Phase 1.1: Split into `2` units + finish F-19 `COM2` cleanup & F-13 CRC32 |
| `src/settings_manager/settings_manager.cpp` | `40,720 B` | `714 L` | Near byte cap | Phase 1.1: Monitor size + add F-12 atomic `.tmp` + `rename()` save |
| `scripts/e220_pacer.py` | `81,545 B` | `1,668 L` | `1.77x B / 2.09x L` | Optional split (`rover_protocol.py` + `update_e220_pacer.sh`) |
| `scripts/generate_wiring_diagrams.py` | `87,745 B` | `1,319 L` | `1.90x B / 1.65x L` | Split lazily when diagrams change next |
| `scripts/bench_e220_multirate.py` | `80,845 B` | `1,725 L` | `1.75x B / 2.16x L` | Keep self-contained CLI bench suite; inspect via offsets |
| `docs/mechanical/cad/generate_rover_enclosure.py` | `79,760 B` | `1,646 L` | `1.73x B / 2.06x L` | Split lazily by CAD sub-assembly when modified next |
| `scripts/cogo_solver.py` | `58,880 B` | `1,165 L` | `1.28x B / 1.46x L` | Optional split (`traverse_math.py` + `csv_exporter.py`) |
| `scripts/stage2_bench_verify.py` | `47,000 B` | `1,028 L` | `1.02x B / 1.29x L` | Optional split if extended |

### 2.2 Workspace, Cache & Tooling Hygiene Findings
1. **In-Tree `build/` & `scratch/` Remnants (`DONE` `2026-11-24`):** Purged `RTK_Rover/build/` (`202.36 MB`, `2,860` items) and `RTK_Rover/scratch/` (`2` stale scripts from Oct 29); `arduino-cli` continues using `%LOCALAPPDATA%/arduino/sketches/`.
2. **`scripts/` & Build-Config Clutter:** `scripts/__pycache__/` contains stale `.pyc` artifacts; two dead `302 B` wrapper stubs (`scripts/setup_um980.py`, `scripts/setup_unicore.py`) remain with zero callers; stale `platformio.ini` references retired `COM2`/old libs (F-19); `scripts/README.md` does not exist.
3. **Workspace Skills Status:** Workspace `.agents/skills/` already provides `rtk-firmware-release/SKILL.md` and `rtk-cad-pipeline/SKILL.md`. Missing: a field log / wiretap triage runbook (`rtk-field-log-analysis`).
4. **Critical `libraries/lv_conf.h` Shadowing Hazard:** `Documents/Arduino/libraries/lv_conf.h` (`26,137 B`, an LVGL `v8.4.0` config) sits next to `libraries/lvgl` (`v9.6.0`), `lv_conf - 9.x.h`, `RTK_Rover/lv_conf.h` (`1,389 L`, `v9.3.0`), and `Magnetic_Field_Scanner/lv_conf.h` (`1,396 L`). Arduino CLI compiles `lvgl` against `libraries/lv_conf.h` first, which is why `LV_USE_FLOAT` compiles as `0` despite `RTK_Rover/lv_conf.h` setting `1`! Must be audited and cleanly reconciled without breaking the mandatory `snprintf()` + `lv_label_set_text()` policy.

---

## 3. Cross-Project Code-Sharing Matrix (`RTK_Rover 3.5B` vs. `Magnetic_Field_Scanner 3.49 v3`)

*(Note: `differential_pressure_meter v1.9.0` uses a `2.8B` `ST7701/GT911` board with an older standalone C++ class hierarchy; keep `Field_Instrument_Core` clean enough to adapt `2.8B` later via a board header, but anchor V1 directly on the shared ESP-IDF/Waveshare BSP lineage of `RTK_Rover` and `Magnetic_Field_Scanner`.)*

| Subsystem | `RTK_Rover v0.10.7` (`3.5B`) | `MFS v5.1.13` (`3.49 v3`) | Verified Overlap & Key Differences | Target Core Module |
| :--- | :--- | :--- | :--- | :--- |
| **`tca9554` IO Expander** | `src/tca9554/` (4 files) | `src/tca9554/` (4 files) | `3/4` files byte-identical. `esp_io_expander_tca9554.c` in Rover adds `i2c_bsp_lock(250)`, `.scl_wait_us=20000`, shadow dir/out cache, and reinit hooks. | `fic_tca9554` (promote Rover version; inject EXIO pin masks) |
| **`esp_codec_dev`** | `src/esp_codec_dev/` (36 files) | `src/esp_codec_dev/` (36 files) | `35/36` files byte-identical. Rover `audio_codec_ctrl_i2c.c` adds `400 kHz`, `i2c_bsp_lock(200)`, status reporting, and bus hooks. | `fic_esp_codec_dev` (promote Rover version; configurable I2C clock) |
| **`codec_board`** | `src/codec_board/` (18 files) | `src/codec_board/` (18 files) | `17/18` identical. Rover `board_cfg.h` is already a strict superset supporting both `S3_LCD_3_49` and `S3_LCD_3_5B`. | `fic_codec_board` + optional Priority-4 `fic_audio_beeper` |
| **`i2c_bsp`** | `717 L` (dual-bus `I2C0`+`I2C1` `s_ports[2]`, 18-clk clear, 2-tier reinit) | `112 L` (`2` buses `I2C0`+`I2C1`, no mutex/recovery) | **Rover ALREADY supports dual-bus `I2C0`+`I2C1` (`v0.9.0`)** w/ lock, warm-reset clear & sentinel recovery. Direct promotion candidate! | `fic_i2c_bsp` (1- or 2-bus config with full Rover lock/clear/reinit) |
| **`axs15231b` + Touch** | `src/axs15231b/` (`345 L`, direct `0x3B` poll + sentinel filter) | `src/axs15231b/` (`486 L`) + `src/touch/` | MFS bundles `esp_lcd_touch`; Rover polls `0x3B` with `i2c_bsp_has_sentinel_run()` & `250 ms` fault backoff. | `fic_axs15231b` + `fic_axs15231b_touch` (accepts `i2c_master_dev_handle_t`) |
| **`lvgl_port` & PSRAM** | `src/lvgl_port/` (`11` files; `lvgl_port_core.c` `377 L` w/ `64->4096` wrap + lock-free FB capture) | `1,239 L` (`64`-line DMA, no `extmem` wrap) | **UI split DONE.** Rover `lvgl_port_core.c` adds PSRAM wrapping (`>150 KB` internal `DRAM0`) & lock-free one-shot FB capture handshake (`v0.10.4`). | `fic_lvgl_port` (board display cfg + `extmem` wrap + `fic_lvgl_lock` + FB capture) |
| **`imu_bsp` (`QMI8658`)** | `220 L` (`dt`-aware EMA, `WHO_AM_I`, glitch filter, pose) | `56 L` (`500 Hz`, no error check) | Rover adds fault recovery + pose/bubble helper, still coupled to `settings_manager.h` + `serial_logger.h`. | `fic_qmi8658` (promote Rover; inject ODR/range & pose consumer) |
| **`rtc_bsp` (`PCF85063`)** | `165 L` (`1 Hz` cache, local TZ, NTP/GNSS sync) | `47 L` (minimal wrapper) | Both wrap `SensorLib` `SensorPCF85063`; Rover couples to `settings_manager.h` and `um982_driver.h`. | `fic_pcf85063` (inject UTC offset min + external epoch sync API) |
| **Power / PMIC / Buttons** | `axp2101_bsp` (`179 L`, PMIC `0x34`) | ADC `IO4` + `SYS_EN` (`P6`) + `button_bsp` | Hardware-divergent power stages across `3.5B` vs. `3.49 v3`. | Selectable `fic_axp2101` and `fic_adc_battery` + `fic_button` |
| **Storage & Ring Loggers** | `storage/` (PSRAM rings, `s_io_owner_task`, decoupled `s_buf_mux`/`s_file_mux`) | `data_logger/` (`322 L`, CSV on SPI SD/`FFat`) | Rover concurrency model prevents RTOS dropouts; loggers still include `settings_manager.h` / `rtc_bsp.h` & `serial_logger.h` hijacks `#define Serial`. | `fic_fs_manager`, `fic_ring_logger`, `fic_core_log` (no `Serial` hijack) |
| **`screenshot` (BMP)** | **`258 L`** (lock-free FB capture + `2`-row DMA buf + `FFat` fallback + close verify) | `89 L` (legacy `lv_snapshot_take`) | **Rewritten in `v0.10.3..v0.10.4`** eliminating Core 1 `lvgl_mux` starvation and PSRAM flush stalls. Promote hardened Rover model into Core. | `fic_screenshot` (`fic_screenshot_save_bmp(...)`) |
| **`web_server` / Wi-Fi / NTP** | `733 L` (Prio-3 connect task, SoftAP fallback, `WIFI_OFF` cleanup) | `247 L` (basic STA + HTML portal) | Rover adds safe deferred teardown, tap-to-cancel, and `DRAM0` diagnostics. | `fic_web_portal` (injected FS handle + NTP/status callbacks) |
| **Math, Fusion & Ext Sensors** | `matrix_math.h` (`698 L`, 0 deps) + `ahrs_9dof` (`469 L`, `Fic` ready) + `sen0696` (`376 L`) + `mmc5983ma` (`259 L`) | `matrix_math.h` (`317 L`) + `MadgwickAHRS` + `SensorFusion` + `RM3100` | **`matrix_math` & `ahrs_9dof` ALREADY ported (`v0.9.0..v0.9.2`)** w/ stationary gyro FOC & ellipsoid cal. `sen0696` needs `FicSen0696_t *` context struct. | `fic_matrix_math`, `fic_ahrs_9dof`, `fic_sen0696`, `fic_mmc5983ma`, `fic_rm3100` |

---

## 4. Phased Implementation Roadmap

### Phase 1.0 -- Immediate Repo, Search & Token Hygiene (Zero Functional Firmware Change)
1. **Purge Stale In-Tree `build/` (`202.36 MB`) & `scratch/` (`DONE` `2026-11-24`):**
   * `RTK_Rover/build/` and `RTK_Rover/scratch/` deleted from disk; `.agents/skills/rtk-firmware-release/SKILL.md` confirms out-of-tree sketch cache (`%LOCALAPPDATA%/arduino/sketches`).
2. **Clean `scripts/` Directory, Stale Configs & Document Utilities:**
   * Delete `scripts/__pycache__/*.pyc` stale artifacts and retire obsolete `302 B` stubs `scripts/setup_um980.py` and `scripts/setup_unicore.py` (confirm zero callers). Remove or sync obsolete `platformio.ini` (per F-19).
   * Create `scripts/README.md` indexing active deployment, Pacer, COGO, Stage 1/2 benchmark, and log-analysis tools (`__filename__` + SemVer header convention).
3. **Split `CHANGELOG.md` (`1,371 L / 247 KB` -> `< 450 L / < 40 KB` active):**
   * Move legacy entries `< v0.8.0` into a historical archive (`docs/changelog_archive_v0_1_to_v0_7.md`), keeping active `CHANGELOG.md` comfortably single-read (`< 800 L / < 46 KB`).
4. **Complete Workspace Skill Coverage:**
   * Keep existing `rtk-firmware-release/SKILL.md` and `rtk-cad-pipeline/SKILL.md`; add `.agents/skills/rtk-field-log-analysis/SKILL.md` covering `analyze_field_test.py`, `stage2_bench_verify.py`, I2C `0x103` triage, and `[MEM] Post-Setup Heap` interpretation without loading full log archives.
5. **Reconcile `libraries/lv_conf.h` (`v8.4.0`) vs. `lvgl` (`v9.6.0`):**
   * Audit which `lv_conf.h` is resolved by `arduino-cli` for both `RTK_Rover` and `Magnetic_Field_Scanner`, clean up the stale `v8.4.0` top-level `libraries/lv_conf.h` (preserving `LV_USE_FLOAT = 0` or explicitly documenting C `snprintf()` discipline), and verify zero compiler warnings on both sketches.

### Phase 1.1 -- In-Place Modularization (`<= 800 L`), Core Decoupling & Robustness Hardening (Prerequisite for Plan 2)

Every C/C++ split and new reusable module MUST enforce four dependency boundaries:
1. **Strict Direction (`App Adapter` -> `Core Module`, Never Reverse):** Core modules receive pins, bus/device handles, display config, signed `3x3` axis-permutation matrices, `StorageFsHandle_t { fs::FS *fs; SemaphoreHandle_t fs_mux; }`, and log callbacks via `*Config_t` / `*Hooks_t` structs -- **zero** `#include "../../user_config.h"`, **zero** `current_settings`, **zero** `RoverUIData_t`, and **zero** `get_active_fs()` globals.
2. **No `#define Serial SerialLogger` in Core:** Keep `#define Serial SerialLogger` strictly inside `RTK_Rover`'s app layer; shared modules log via `fic_log_printf(...)` or injected callbacks.
3. **Instance-Safe State (`extern "C"` Friendly):** Replace static/global filter state (such as `MFS` `MadgwickAHRS.c` `q0..q3` and `sen0696_driver.cpp` `s_state` / `beam_imu_dev_handle`) with explicit context structs (`FicAhrs9Dof_t *`, `FicSen0696_t *`).
4. **Display Core vs. App UI Separation:** `lvgl_port_core.c` owns QSPI/DMA/PSRAM wrapping (`heap_caps_malloc_extmem_enable(64)` -> `4096`), `rover_lvgl_capture_framebuffer()`, and generic locks; app tiles stay in `RTK_Rover`.

**Exact File Splits, Decouplings & Hardening (`v0.10.7` Baseline):**
1. **`src/lvgl_port/` (`11`-File Split COMPLETE in Plan 0 -> Core Decouple Here):**
   * Remove `lvgl_ui_internal.h` include from `lvgl_port_core.c`; parameterize `lvgl_port_core.c` (`BoardDisplayCfg_t`, PSRAM `extmem_enable(64)` wrapper, mutex, lock-free FB capture) so `ui_header_nav.c` and `ui_tile0..tile5*.c` stay strictly sketch-local in `RTK_Rover`.
2. **Split `src/e220_driver/e220_driver.cpp` (`1,451 L / 68 KB` -> `3` Units + `e220_driver_internal.h`):**
   * `e220_config_mode3.cpp` (`~500 L`), `e220_rtcm_gatekeeper.cpp` (`~380 L` -- exposing clean RTCM3 validation hook for Plan 2's `rtcm_arbiter`), `e220_tdma_scheduler.cpp` (`~520 L`).
3. **Split `src/um982_driver/um982_driver.cpp` (`1,020 L / 47.5 KB` -> `2` Units + Finish F-13, F-14, F-19):**
   * `um982_config_audit.cpp` + `um982_nmea_parser.cpp` (`~520 L` each).
   * **F-19 (`COM2` Cleanup):** Remove `~42` residual `COM2` legacy aliases/comments (`um982_forward_rtcm_to_com2`, `um982_poll_com2`, `send_um982_com2_test_pulse`, `com2_rx_bytes`, `ROVER_UM982_COM2_BAUD`, and `COM2` comments across `RTK_Rover.ino`, `rover_api.h`, `user_config.h`, `ui_tile4_diagnostics.c`, `e220_driver`).
   * **F-13 (Unicore CRC32 Check):** Verify trailing 32-bit CRC32 on `#UNIHEADINGA` / `#BASEINFOA` / `#VERSIONA` ASCII lines alongside NMEA XOR `*XX` checksums so corrupted Unicore frames are cleanly dropped.
   * **F-14 (Non-Blocking Receiver Audit Safety):** Bound or state-machine `um982_query_receiver_state()` blocking retries when triggered interactively or ahead of Plan 2 BLE command exposure so Core 0 never drops RTCM3 > 1 s.
4. **Split `src/waypoint_manager/waypoint_manager.cpp` (`1,060 L / 45.6 KB` -> `2` Units):**
   * `waypoint_csv_store.cpp` + `waypoint_survey_log.cpp` (with injected FS + mutex).
5. **Slim `RTK_Rover.ino` (`1,531 L / 80.7 KB` -> `< 750 L / < 42 KB`):**
   * Extract Priority-4 ES8311 `task_audio_beep` + `beepQueue` into `src/rover_audio/`.
   * Extract AXP2101 auto-shutdown / startup power logic into `src/rover_power/` (also staging Plan 2's `< 3.40 V` low-battery SD flush hook).
   * Extract Core 1 `SEN0696` / `QMI8658` / `CAL MAG` sensor-fusion orchestration loop into `src/rover_sensors/` (`~450-550 L`) so `RTK_Rover.ino` drops cleanly below both `800 L` and `46 KB`.
6. **Decouple Core Candidates & Settings Integrity (`F-12`):**
   * Strip `settings_manager.h` + `serial_logger.h` from `imu_bsp.c`; inject UTC offset & remove `um982_driver.h` from `rtc_bsp.cpp`; convert `sen0696_driver.cpp` to `FicSen0696_t *`; align `nmea_logger.cpp` and `route_logger.cpp` to `rtcm_logger`'s decoupled `s_buf_mux`/`s_file_mux`.
   * **F-12 (`settings_manager` Atomicity):** Write settings to `/settings.json.tmp` on SD/`FFat` before atomic `rename()` over `/settings.json` (plus add a read/write snapshot mutex where multi-task reads occur) so brownout mid-save never leaves a 0-byte config.
   * Keep `src/matrix_math/` (`698 L`) and `src/sensor_fusion/` (`469 L`) strictly zero-app-dependency for direct Phase 1.2 adoption.
   * *Optional Python split:* When modifying `scripts/e220_pacer.py` or `scripts/cogo_solver.py`, extract `rover_protocol.py` (updating `scripts/update_e220_pacer.sh` together!) and `cogo_math.py`.

### Phase 1.2 -- Separate Unified Library Repository (`Field_Instrument_Core`)

Create an independent public Git repository (**`Field-Instrument-Core`**), checked out locally at `Documents/Arduino/libraries/Field_Instrument_Core/` (`library.properties`, SemVer `v1.0.0+`). Keep RTK/cadastral-specific code (`um982_driver`, `e220_driver`, `nav_engine`, `waypoint_manager`, `rover_sensors`, `e220_pacer.py`, `cogo_solver.py`) inside `RTK_Rover`, and keep `MFS` magnetic-survey UI/DSP inside `Magnetic_Field_Scanner`.

```text
Documents/Arduino/libraries/Field_Instrument_Core/
|-- library.properties                 # Standard Arduino ESP32 library manifest (SemVer v1.0.0+)
|-- README.md                          # Board profiles, API contracts & consumer matrix
|-- CHANGELOG.md                       # Library-only release log
|-- extras/
|   |-- ci_compile_all.ps1             # Compiles RTK_Rover & Magnetic_Field_Scanner (--warnings default)
|   |-- unit_tests/                    # Host C/C++ unit tests (matrix_math, ahrs_9dof, ring_logger, crc - F-17)
|   `-- board_notes/                   # Shared Waveshare 3.5B & 3.49 v3 pin/errata references
`-- src/
    |-- Field_Instrument_Core.h        # Master umbrella header
    |-- boards/
    |   |-- fic_board_types.h          # I2cBusCfg, DisplayCfg, AudioCfg, PowerCfg structs
    |   |-- waveshare_s3_35b.h         # 3.5B 320x480, 1/2-bus I2C0/I2C1, AXP2101, no-MCLK ES8311
    |   `-- waveshare_s3_349_v3.h      # 3.49 v3 172x640, 2-bus I2C0+I2C1, ADC+SYS_EN, TDM ES8311/ES7210
    |-- bus/       fic_i2c_bsp.{c,h}   # 18-clk GPIO bus-clear, 20ms scl_wait, 1/2-port lock + 2-tier reinit hooks
    |-- io/        fic_tca9554.*       # Locked esp_io_expander + shadow cache + bus hooks
    |-- audio/     fic_esp_codec_dev/, fic_codec_board/, fic_audio_beeper.*
    |-- display/   fic_axs15231b.*, fic_axs15231b_touch.*, fic_lvgl_port.*, fic_screenshot.*
    |-- power/     fic_axp2101.*, fic_adc_battery.*, fic_button.*
    |-- rtc/       fic_pcf85063.*      # Rate-limited RTC + UTC offset + external epoch sync
    |-- sensors/   fic_qmi8658.*, fic_sen0696.*, fic_mmc5983ma.*, fic_rm3100.*
    |-- math/      fic_matrix_math.h   # float/double 9-param ellipsoid fit, Jacobi eigen, Kabsch 3x3
    |-- fusion/    fic_ahrs_9dof.*     # Instance-safe Madgwick 6/9-DOF + stationary accel-variance gyro FOC
    |-- storage/   fic_fs_manager.*, fic_ring_logger.*, fic_core_log.*, fic_settings_json.*
    `-- net/       fic_web_portal.*    # Core 1 Prio-3 Wi-Fi STA/AP + NTP + file server + DRAM0 telemetry
```

**Mandatory Safety & RTOS Invariants Enforced Inside Core:**
* **Internal `DRAM0` Conservation:** `fic_lvgl_port` wraps UI creation with `heap_caps_malloc_extmem_enable(64)` -> `4096`; all loggers and ring buffers `>= 1 KB` allocate strictly in Octal PSRAM (`MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT`); `fic_screenshot` uses lock-free framebuffer capture + `2`-row internal DMA write buffer + `FFat` fallback + post-close verification.
* **Real-Time Audio & Storage Isolation:** `fic_audio_beeper` runs Priority 4 on Core 1; ring loggers use task-scoped `s_io_owner_task` re-entrancy (`fic_core_log`) and decoupled `s_buf_mux` / `s_file_mux` (`fic_ring_logger`), never writing Flash/SD inside UI tasks; `fic_settings_json` writes atomically via `.tmp` + `rename()`.
* **I2C0/I2C1 Fault Recovery:** Every configured I2C port runs `fic_i2c_gpio_bus_clear()`, `.scl_wait_us = 20000`, `fic_i2c_lock()`, `0x61`/`0x71` sentinel detection, and two-tier runtime bus reinit with registered `TCA9554` / `ES8311` / sensor detach-reattach hooks.

### Phase 1.3 -- Consumer Migration, Host Tests (`F-17`) & Dual-Compile Verification

1. **Stage 1 (`RTK_Rover` Migration):**
   * Point `RTK_Rover` includes to `<Field_Instrument_Core.h>` (`waveshare_s3_35b.h`), delete the `~65` duplicated BSP/codec/driver files from `RTK_Rover/src/`, and keep `um982_driver`, `e220_driver`, `nav_engine`, `waypoint_manager`, `rover_sensors`, UI tiles, and [Plan 2](plan_2_rover_enhancements_and_android_companion.md) features sketch-local.
   * **Host C/C++ Unit Tests (`F-17`):** Add native host test suite (`g++` / `pytest` harness in `Field_Instrument_Core/extras/unit_tests/` and `RTK_Rover/scripts/`) exercising `fic_matrix_math` ellipsoid fit, `fic_ahrs_9dof` FOC/heading, `nav_compute_line_stakeout()`, `nav_format_along_line()`, and `um982` NMEA/Unicore CRC parsers.
   * **Gate:** `arduino-cli compile` (`--warnings default`) produces **`0` warnings**, static `DRAM0 <= 25%` (`< 80 KB`), `[MEM] Post-Setup Heap` internal free `>= 150 KB`, and all `stage2_bench_verify.py`, `test_rover_telemetry.py`, host unit tests (`F-17`), and `check_secrets.py` suites pass.
2. **Stage 2 (`Magnetic_Field_Scanner` Migration, `v5.1.x -> v6.0.0`):**
   * Migrate `Magnetic_Field_Scanner` onto `Field_Instrument_Core` (`waveshare_s3_349_v3.h`), replacing `MFS`'s unguarded `i2c_bsp`, `QMI8658`, `tca9554`, `esp_codec_dev`, `screenshot`, `web_server`, and global `MadgwickAHRS.c` with the hardened Core implementations.
   * Preserve `MFS`'s dual-I2C topology (`I2C0` @ `200 kHz` sensors + `I2C1` Touch), TDM `ES8311+ES7210` streaming, dual `RM3100` DRDY interrupts, and existing magnetometer survey workflows (`0` functional regression).
3. **Dual-Compile CI Gate (`extras/ci_compile_all.ps1`):**
   * Every Core release must compile **both** `RTK_Rover` (`3.5B` FQBN) and `Magnetic_Field_Scanner` (`3.49 v3` FQBN) with `--warnings default` (`0` warnings) before tagging SemVer in `Field-Instrument-Core`.

---

## 5. Target Token & Maintainability Outcomes

| Metric | Current (`v0.10.7`) | Target (End of Plan 1) |
| :--- | :---: | :--- |
| C/C++ files exceeding `800 L / 46 KB` | `4` app files (`ui` already split into `11` <= 792 L) | **`0` core/app source files** |
| Duplicated BSP/codec/driver files (`RTK_Rover` vs. `MFS`) | `~65+` files across `2` repos | **`0` (1 shared library)** |
| In-tree `build/` & `scratch/` clutter | **`0 B` (purged `2026-11-24`)** | **`0 B` (permanent)** |
| Active `CHANGELOG.md` size | `1,371 L` / `247 KB` | **`< 450 L` / `< 40 KB`** |
| Active `docs/plans/` documents | `2` active (`Plan 1`, `Plan 2`) + `8` in `archive/` | **`2` sequenced plans + `archive/`** |
| Shared math/fusion & dual-bus I2C readiness | **Done in `src/` (`v0.9.0..v0.9.2`)** | **Promoted into `Field_Instrument_Core` w/ host tests** |
