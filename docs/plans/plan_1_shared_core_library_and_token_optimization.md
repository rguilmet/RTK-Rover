# Plan 1: Unified Shared Core Library (`Field_Instrument_Core`), Codebase Modularization & Token Optimization

> [!NOTE]
> **Status:** Active Execution Roadmap (Updated November 21, 2026)  
> **Document Version:** `1.1.0`  
> **Baselines:** `RTK_Rover v0.8.18` | `CAD v2.1.6` | `Magnetic_Field_Scanner v5.1.13`  
> **Prerequisite Roadmap:** [`plan_0_field_survey_refinements.md`](plan_0_field_survey_refinements.md) (`v0.8.19` - `v0.9.0`: executes `lvgl_port.c` split, `I2C1` dual-bus, & `matrix_math`/`ahrs_9dof` port ahead of Core extraction)  
> **Companion Roadmap:** [`plan_2_rover_enhancements_and_android_companion.md`](plan_2_rover_enhancements_and_android_companion.md)  
> **Supersedes / Consolidates:** [`archive/plan_project_structure_and_token_optimization.md`](archive/plan_project_structure_and_token_optimization.md), Phase 1 of [`archive/plan_beam_10dof_and_modularization.md`](archive/plan_beam_10dof_and_modularization.md), residual F-19 `COM2` naming cleanup from [`archive/plan_phase9_gps_disciplined_tdma.md`](archive/plan_phase9_gps_disciplined_tdma.md), and Section 7 of [`archive/plan_smart_pole_and_android_app.md`](archive/plan_smart_pole_and_android_app.md).

---

## 1. Executive Summary & Architectural Scope

With the hardware locked on the **Waveshare ESP32-S3-Touch-LCD-3.5B** platform (retiring the `Waveshare 3.49B-v2` Smart Pole migration), this plan has two coupled objectives:

1. **Optimize AI Context & Token Efficiency:** Keep every active firmware module, documentation file, and planning roadmap compact enough to fit inside a single targeted file read (`<= 800 lines` and `< 46,080 bytes`), eliminate build-artifact search clutter (`202.36 MB`), prune obsolete stubs, and expand on-demand workspace skills.
2. **Extract a Separate Unified Library Repository (`Field_Instrument_Core`):** Eliminate `~65+` duplicated ESP-IDF BSP, display, audio, I2C, IMU, RTC, logger, Wi-Fi portal, and math/fusion files currently copied between **`RTK_Rover` (`3.5B`)** and **`Magnetic_Field_Scanner` (`3.49 v3`, `v5.1.13`)** (and future Waveshare ESP32-S3 field instruments), shipping both consumers' hardest-won stability improvements into one SemVer Arduino library.

**Sequencing Note:** [**Plan 0**](plan_0_field_survey_refinements.md) (`v0.8.19` - `v0.9.0`) pulls forward the `lvgl_port.c` (`4,225 lines`) modular split (9 files / 6 tiles) and the `I2C1` + `matrix_math` / `ahrs_9dof` port, so **Plan 1 Phases 1.0 & 1.1** finish the remaining file splits and Core decoupling **before** [`Field_Instrument_Core`](plan_2_rover_enhancements_and_android_companion.md) extraction and Plan 2 (`BLE` / Android Slave Mode).

---

## 2. Measured Codebase & Token Audit (`RTK_Rover v0.8.18`)

### 2.1 Files Exceeding the Single-Read Cap (`800 lines` / `46,080 bytes`)

| Tracked File | Bytes | Lines | Cap Overrun | Disposition Across Plan 0 -> Plan 1 |
| :--- | :---: | :---: | :---: | :--- |
| `CHANGELOG.md` | `205,400 B` | `1,497 L` | `4.5x B / 1.9x L` | Plan 1 Phase 1.0: Rolling archive split (`< v0.8.0` -> archive) |
| `src/lvgl_port/lvgl_port.c` | `193,724 B` | `4,225 L` | `4.2x B / 5.3x L` | **Moved to Plan 0 Phase 0.1 (`9` files / `6` tiles)** |
| `src/e220_driver/e220_driver.cpp` | `68,372 B` | `1,596 L` | `1.5x B / 2.0x L` | Phase 1.1: Split into `3` units + internal header |
| `src/um982_driver/um982_driver.cpp` | `47,460 B` | `1,146 L` | `1.03x B / 1.4x L` | Phase 1.1: Split into `2` units + finish F-19 `COM2` cleanup |
| `src/waypoint_manager/waypoint_manager.cpp` | `39,796 B` | `1,017 L` | `1.27x L` | Phase 1.1: Split CSV store vs. survey logger (`2` files) |
| `RTK_Rover.ino` | `44,687 B` | `932 L` | `1.17x L` | Phase 1.1: Extract `rover_audio` + `rover_power` (`< 650 L`) |
| `scripts/e220_pacer.py` | `81,545 B` | `1,874 L` | `1.8x B / 2.3x L` | Optional split (`rover_protocol.py` + `update_e220_pacer.sh`) |
| `scripts/generate_wiring_diagrams.py` | `81,997 B` | `1,429 L` | `1.8x B / 1.8x L` | Split lazily when diagrams change next |
| `scripts/bench_e220_multirate.py` | `80,845 B` | `1,955 L` | `1.8x B / 2.4x L` | Keep self-contained CLI bench suite; inspect via offsets |
| `docs/mechanical/cad/generate_rover_enclosure.py` | `62,562 B` | `1,460 L` | `1.4x B / 1.8x L` | Split lazily by CAD sub-assembly when modified next |
| `scripts/cogo_solver.py` | `57,452 B` | `1,348 L` | `1.25x B / 1.7x L` | Optional split (`traverse_math.py` + `csv_exporter.py`) |
| `scripts/stage2_bench_verify.py` | `46,958 B` | `1,176 L` | `1.02x B / 1.5x L` | Optional split if extended |

### 2.2 Workspace, Cache & Tooling Hygiene Findings
1. **In-Tree Build Clutter (`RTK_Rover/build/`):** Still occupies **`2,613` files / `202.36 MB`** on disk (`.gitignore`'d, but pollutes local recursive searches and tool globs).
2. **`scripts/` Clutter:** `scripts/__pycache__/` contains `23` `.pyc` artifacts (including stale `e220_pacer-v1.4.1.pyc`, `v2.5.0.pyc`); two dead `302 B` wrapper stubs (`scripts/setup_um980.py`, `scripts/setup_unicore.py`) remain with zero callers; `scripts/README.md` does not exist.
3. **Workspace Skills Status:** Workspace `.agents/skills/` already provides `rtk-firmware-release/SKILL.md` and `rtk-cad-pipeline/SKILL.md`. Missing: a field log / wiretap triage runbook (`rtk-field-log-analysis`).
4. **Critical `libraries/lv_conf.h` Shadowing Hazard:** `Documents/Arduino/libraries/lv_conf.h` (`26,137 B`, an LVGL `v8.4.0` config) sits next to `libraries/lvgl` (`v9.6.0`), `lv_conf - 9.x.h`, `RTK_Rover/lv_conf.h` (`1,389 L`, `v9.3.0`), and `Magnetic_Field_Scanner/lv_conf.h` (`1,396 L`, `19`-line diff vs. Rover). Arduino CLI compiles `lvgl` against `libraries/lv_conf.h` first, which is why `LV_USE_FLOAT` compiles as `0` despite `RTK_Rover/lv_conf.h` setting `1`! Must be audited and cleanly reconciled without breaking the mandatory `snprintf()` + `lv_label_set_text()` policy.

---

## 3. Cross-Project Code-Sharing Matrix (`RTK_Rover 3.5B` vs. `Magnetic_Field_Scanner 3.49 v3`)

*(Note: `differential_pressure_meter v1.9.0` uses a `2.8B` `ST7701/GT911` board with an older standalone C++ class hierarchy; keep `Field_Instrument_Core` clean enough to adapt `2.8B` later via a board header, but anchor V1 directly on the shared ESP-IDF/Waveshare BSP lineage of `RTK_Rover` and `Magnetic_Field_Scanner`.)*

| Subsystem | `RTK_Rover v0.8.18` (`3.5B`) | `MFS v5.1.13` (`3.49 v3`) | Verified Overlap & Key Differences | Target Core Module |
| :--- | :--- | :--- | :--- | :--- |
| **`tca9554` IO Expander** | `src/tca9554/` (4 files) | `src/tca9554/` (4 files) | `3/4` files byte-identical. `esp_io_expander_tca9554.c` in Rover adds `i2c_bsp_lock(250)`, `.scl_wait_us=20000`, shadow dir/out cache, and reinit hooks. | `fic_tca9554` (promote Rover version; inject EXIO pin masks) |
| **`esp_codec_dev`** | `src/esp_codec_dev/` (36 files) | `src/esp_codec_dev/` (36 files) | `35/36` files byte-identical. Rover `audio_codec_ctrl_i2c.c` adds `400 kHz`, `i2c_bsp_lock(200)`, status reporting, and bus hooks. | `fic_esp_codec_dev` (promote Rover version; configurable I2C clock) |
| **`codec_board`** | `src/codec_board/` (18 files) | `src/codec_board/` (18 files) | `17/18` identical. Rover `board_cfg.h` is already a strict superset supporting both `S3_LCD_3_49` and `S3_LCD_3_5B`. | `fic_codec_board` + optional Priority-4 `fic_audio_beeper` |
| **`i2c_bsp`** | `578 L` (`1` bus `I2C0`, mutex, 18-clk clear, 2-tier reinit) | `112 L` (`2` buses `I2C0`+`I2C1`, no mutex/recovery) | Major safety evolution in Rover (`AXP2101` warm-reset recovery + sentinel detection). MFS needs 2-bus support (`I2C1` Touch). | `fic_i2c_bsp` (1- or 2-bus config with full Rover lock/clear/reinit) |
| **`axs15231b` + Touch** | `src/axs15231b/` (`345 L`, direct `0x3B` poll + sentinel filter) | `src/axs15231b/` (`486 L`) + `src/touch/` | MFS bundles `esp_lcd_touch`; Rover polls `0x3B` with `i2c_bsp_has_sentinel_run()` & `250 ms` fault backoff. | `fic_axs15231b` + `fic_axs15231b_touch` (accepts `i2c_master_dev_handle_t`) |
| **`lvgl_port` & PSRAM** | `4,225 L` (`24`-line DMA, `extmem_enable(64)` -> `4096`) | `1,239 L` (`64`-line DMA, no `extmem` wrap) | Both mix QSPI/LVGL task init with app UI. Rover PSRAM wrapping protects `> 150 KB` internal `DRAM0`. | `fic_lvgl_port` (board display cfg + `extmem` wrap + `fic_lvgl_lock`) |
| **`imu_bsp` (`QMI8658`)** | `220 L` (reset poll, `WHO_AM_I`, LPF, glitch filter, pose) | `56 L` (`500 Hz`, no error check) | Rover adds fault recovery + pose/bubble helper, currently coupled to `current_settings` (`settings_manager.h`). | `fic_qmi8658` (promote Rover; inject ODR/range & pose consumer) |
| **`rtc_bsp` (`PCF85063`)** | `165 L` (`1 Hz` cache, local TZ, NTP/GNSS sync) | `47 L` (minimal wrapper) | Both wrap `SensorLib` `SensorPCF85063`; Rover couples to `settings_manager.h` and `um982_driver.h`. | `fic_pcf85063` (inject UTC offset min + external epoch sync API) |
| **Power / PMIC / Buttons** | `axp2101_bsp` (`179 L`, PMIC `0x34`) | ADC `IO4` + `SYS_EN` (`P6`) + `button_bsp` | Hardware-divergent power stages across `3.5B` vs. `3.49 v3`. | Selectable `fic_axp2101` and `fic_adc_battery` + `fic_button` |
| **Storage & Ring Loggers** | `storage/` (`955 L`, PSRAM rings, `s_io_owner_task`, decoupled `s_buf_mux`/`s_file_mux`) | `data_logger/` (`322 L`, CSV on SPI SD/`FFat`) | Rover `rtcm_logger` / `serial_logger` concurrency model prevents RTOS dropouts; `serial_logger.h` currently hijacks `#define Serial`. | `fic_fs_manager`, `fic_ring_logger`, `fic_core_log` (no `Serial` hijack) |
| **`screenshot` (BMP)** | `75 L` (RGB565 -> 24-bit BMP) | `89 L` (RGB565 -> 24-bit BMP) | Near-identical code; differs only in mutex fn, `/screenshots/`, and target LVGL obj. | `fic_screenshot` (`fic_screenshot_save_bmp(...)`) |
| **`web_server` / Wi-Fi / NTP** | `733 L` (Prio-3 connect task, SoftAP fallback, `WIFI_OFF` cleanup) | `247 L` (basic STA + HTML portal) | Rover adds safe deferred teardown, tap-to-cancel, and `DRAM0` diagnostics. | `fic_web_portal` (injected FS handle + NTP/status callbacks) |
| **Math, Fusion & Ext Sensors** | `nav_engine` + `sen0696_driver` (`366 L`) + `mmc5983ma_driver` (`259 L`) | `matrix_math.h` (`317 L`, 0 deps) + `MadgwickAHRS` + `SensorFusion` + `RM3100` | MFS brings pure-math 9-param ellipsoid fit, Jacobi eigen, Kabsch, Madgwick, & 50-sample stationary gyro FOC; Rover brings `SEN0696` & `MMC5983MA`. | `fic_matrix_math`, `fic_ahrs_9dof`, `fic_sen0696`, `fic_mmc5983ma`, `fic_rm3100` |

---

## 4. Phased Implementation Roadmap

### Phase 1.0 -- Immediate Repo, Search & Token Hygiene (Zero Functional Firmware Change)
1. **Purge Stale In-Tree Build Output (`202.36 MB`):**
   * Delete `RTK_Rover/build/` from disk and confirm `.gitignore` ignores any build staging folders.
   * Update `.agents/skills/rtk-firmware-release/SKILL.md` to confirm `arduino-cli compile` uses out-of-tree sketch cache (`%LOCALAPPDATA%/arduino/sketches`) or explicit `--build-path` outside the repo.
2. **Clean `scripts/` Directory & Document Utilities:**
   * Delete `scripts/__pycache__/*.pyc` stale artifacts and retire obsolete `302 B` stubs `scripts/setup_um980.py` and `scripts/setup_unicore.py` (confirm zero callers).
   * Create `scripts/README.md` indexing active deployment, Pacer, COGO, Stage 1/2 benchmark, and log-analysis tools (`__filename__` + SemVer header convention).
3. **Split `CHANGELOG.md` (`1,497 L / 205 KB` -> `< 450 L / < 35 KB` active):**
   * Move legacy entries `< v0.8.0` into a historical archive (`docs/changelog_archive_v0_1_to_v0_7.md`), and consider extracting historical CAD `v1.x` logs if needed so the active `CHANGELOG.md` stays comfortably single-read (`< 800 L / < 46 KB`).
4. **Complete Workspace Skill Coverage:**
   * Add `.agents/skills/rtk-field-log-analysis/SKILL.md` (alongside `rtk-firmware-release` and `rtk-cad-pipeline`) covering `analyze_field_test.py`, `stage2_bench_verify.py`, I2C `0x103` triage, and `[MEM] Post-Setup Heap` interpretation without loading full log archives.
5. **Reconcile `libraries/lv_conf.h` (`v8.4.0`) vs. `lvgl` (`v9.6.0`):**
   * Audit which `lv_conf.h` is resolved by `arduino-cli` for both `RTK_Rover` and `Magnetic_Field_Scanner`, clean up the stale `v8.4.0` top-level `libraries/lv_conf.h` (preserving `LV_USE_FLOAT = 0` or explicitly documenting C `snprintf()` discipline), and verify zero compiler warnings on both sketches.

### Phase 1.1 -- In-Place Modularization (`<= 800 L`) & Core Decoupling (Prerequisite for Plan 2)

Every C/C++ split and new reusable module MUST enforce four dependency boundaries:
1. **Strict Direction (`App Adapter` -> `Core Module`, Never Reverse):** Core modules receive pins, bus/device handles, display config, signed `3x3` axis-permutation matrices, `StorageFsHandle_t { fs::FS *fs; SemaphoreHandle_t fs_mux; }`, and log callbacks via `*Config_t` / `*Hooks_t` structs -- **zero** `#include "../../user_config.h"`, **zero** `current_settings`, **zero** `RoverUIData_t`, and **zero** `get_active_fs()` globals.
2. **No `#define Serial SerialLogger` in Core:** Keep `#define Serial SerialLogger` strictly inside `RTK_Rover`'s app layer; shared modules log via `fic_log_printf(...)` or injected callbacks.
3. **Instance-Safe State (`extern "C"` Friendly):** Replace static/global filter state (such as `MFS` `MadgwickAHRS.c` `q0..q3` and `sen0696_driver.cpp` `s_state` / `beam_imu_dev_handle`) with explicit context structs (`FicAhrs9Dof_t *`, `FicSen0696_t *`).
4. **Display Core vs. App UI Separation:** `lvgl_port_core.c` owns QSPI/DMA/PSRAM wrapping (`heap_caps_malloc_extmem_enable(64)` -> `4096`) and generic locks; app tiles stay in `RTK_Rover`.

**Exact File Splits & Cleanups (`v0.8.18`):**
1. **`src/lvgl_port/` (`Done in Plan 0 Phase 0.1` -> Core Decouple Here):**
   * `lvgl_port.c` is split into `9` files (`6` tiles including `ui_tile5_settings.c`) in Plan 0 Phase 0.1. In Phase 1.1, parameterize `lvgl_port_core.c` (`BoardDisplayCfg_t`, PSRAM `extmem_enable(64)` wrapper, mutex) so `ui_header_nav.c` and `ui_tile0..tile5*.c` stay strictly sketch-local in `RTK_Rover`.
2. **Split `src/e220_driver/e220_driver.cpp` (`1,596 L` -> `3` Units + `e220_driver_internal.h`):**
   * `e220_config_mode3.cpp` (`~520 L`), `e220_rtcm_gatekeeper.cpp` (`~400 L` -- exposing clean RTCM3 validation hook for Plan 2's `rtcm_arbiter`), `e220_tdma_scheduler.cpp` (`~550 L`).
3. **Split `src/um982_driver/um982_driver.cpp` (`1,146 L` -> `2` Units + Finish Residual F-19 `COM2` Cleanup):**
   * `um982_config_audit.cpp` + `um982_nmea_parser.cpp` (`~550 L` each).
   * Remove residual F-19 `COM2` legacy aliases/comments (`um982_forward_rtcm_to_com2`, `um982_poll_com2`, `send_um982_com2_test_pulse`, `com2_rx_bytes`, `ROVER_UM982_COM2_BAUD`, and `COM2` comments in `RTK_Rover.ino`, `rover_api.h`, `e220_driver.h/.cpp`).
4. **Split `src/waypoint_manager/waypoint_manager.cpp` (`1,017 L` -> `2` Units):**
   * `waypoint_csv_store.cpp` + `waypoint_survey_log.cpp` (with injected FS + mutex).
5. **Slim `RTK_Rover.ino` (`932 L` -> `< 650 L`):**
   * Extract Priority-4 ES8311 `task_audio_beep` + `beepQueue` into `src/rover_audio/` and AXP2101 auto-shutdown / startup power logic into `src/rover_power/` (also staging Plan 2's `< 3.40 V` low-battery SD flush hook).
6. **Decouple Core Candidates & Finish Math/Fusion Separation:**
   * Strip `settings_manager.h` from `imu_bsp.c`; inject UTC offset & remove `um982_driver.h` from `rtc_bsp.cpp`; convert `sen0696_driver.cpp` to `FicSen0696_t`; align `nmea_logger.cpp` and `route_logger.cpp` to `rtcm_logger`'s decoupled `s_buf_mux`/`s_file_mux`.
   * Ensure `src/matrix_math/` and `src/sensor_fusion/` (`ahrs_9dof`, seeded in Plan 0 Phase 0.6 from `MFS`) have zero app-specific headers so `MFS` adopts them unchanged in Phase 1.3.
   * *Optional Python split:* When modifying `scripts/e220_pacer.py` or `scripts/cogo_solver.py`, extract `rover_protocol.py` (updating `scripts/update_e220_pacer.sh` together!) and `cogo_math.py`.

### Phase 1.2 -- Separate Unified Library Repository (`Field_Instrument_Core`)

Create an independent public Git repository (**`Field-Instrument-Core`**), checked out locally at `Documents/Arduino/libraries/Field_Instrument_Core/` (`library.properties`, SemVer `v1.0.0+`). Keep RTK/cadastral-specific code (`um982_driver`, `e220_driver`, `nav_engine`, `waypoint_manager`, `e220_pacer.py`, `cogo_solver.py`) inside `RTK_Rover`, and keep `MFS` magnetic-survey UI/DSP inside `Magnetic_Field_Scanner`.

```text
Documents/Arduino/libraries/Field_Instrument_Core/
|-- library.properties                 # Standard Arduino ESP32 library manifest (SemVer v1.0.0+)
|-- README.md                          # Board profiles, API contracts & consumer matrix
|-- CHANGELOG.md                       # Library-only release log
|-- extras/
|   |-- ci_compile_all.ps1             # Compiles RTK_Rover & Magnetic_Field_Scanner (--warnings default)
|   `-- board_notes/                   # Shared Waveshare 3.5B & 3.49 v3 pin/errata references
`-- src/
    |-- Field_Instrument_Core.h        # Master umbrella header
    |-- boards/
    |   |-- fic_board_types.h          # I2cBusCfg, DisplayCfg, AudioCfg, PowerCfg structs
    |   |-- waveshare_s3_35b.h         # 3.5B 320x480, 1-bus I2C0 (or 2-bus GPIO 47/48), AXP2101, no-MCLK ES8311
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
* **Internal `DRAM0` Conservation:** `fic_lvgl_port` wraps UI creation with `heap_caps_malloc_extmem_enable(64)` -> `4096`; all loggers and ring buffers `>= 1 KB` allocate strictly in Octal PSRAM (`MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT`).
* **Real-Time Audio & Storage Isolation:** `fic_audio_beeper` runs Priority 4 on Core 1; ring loggers use task-scoped `s_io_owner_task` re-entrancy (`fic_core_log`) and decoupled `s_buf_mux` / `s_file_mux` (`fic_ring_logger`), never writing Flash/SD inside UI tasks.
* **I2C0/I2C1 Fault Recovery:** Every configured I2C port runs `fic_i2c_gpio_bus_clear()`, `.scl_wait_us = 20000`, `fic_i2c_lock()`, `0x61`/`0x71` sentinel detection, and two-tier runtime bus reinit with registered `TCA9554` / `ES8311` / sensor detach-reattach hooks.

### Phase 1.3 -- Consumer Migration & Dual-Compile Verification

1. **Stage 1 (`RTK_Rover` Migration):**
   * Point `RTK_Rover` includes to `<Field_Instrument_Core.h>` (`waveshare_s3_35b.h`), delete the `~65` duplicated BSP/codec/driver files from `RTK_Rover/src/`, and keep `um982_driver`, `e220_driver`, `nav_engine`, `waypoint_manager`, UI tiles, and [Plan 2](plan_2_rover_enhancements_and_android_companion.md) features sketch-local.
   * **Gate:** `arduino-cli compile` (`--warnings default`) produces **`0` warnings**, static `DRAM0 <= 25%` (`< 80 KB`), `[MEM] Post-Setup Heap` internal free `>= 150 KB`, and all `stage2_bench_verify.py` / `test_rover_telemetry.py` / `check_secrets.py` suites pass.
2. **Stage 2 (`Magnetic_Field_Scanner` Migration, `v5.1.x -> v6.0.0`):**
   * Migrate `Magnetic_Field_Scanner` onto `Field_Instrument_Core` (`waveshare_s3_349_v3.h`), replacing `MFS`'s unguarded `i2c_bsp`, `QMI8658`, `tca9554`, `esp_codec_dev`, `screenshot`, `web_server`, and global `MadgwickAHRS.c` with the hardened Core implementations.
   * Preserve `MFS`'s dual-I2C topology (`I2C0` @ `200 kHz` sensors + `I2C1` Touch), TDM `ES8311+ES7210` streaming, dual `RM3100` DRDY interrupts, and existing magnetometer survey workflows (`0` functional regression).
3. **Dual-Compile CI Gate (`extras/ci_compile_all.ps1`):**
   * Every Core release must compile **both** `RTK_Rover` (`3.5B` FQBN) and `Magnetic_Field_Scanner` (`3.49 v3` FQBN) with `--warnings default` (`0` warnings) before tagging SemVer in `Field-Instrument-Core`.

---

## 5. Target Token & Maintainability Outcomes

| Metric | Current (`v0.8.18`) | Target (End of Plan 1) |
| :--- | :---: | :---: |
| C/C++ files exceeding `800 L / 46 KB` | `5` files (up to `4,225 L`) | **`0` core source files** |
| Duplicated BSP/codec/driver files (`RTK_Rover` vs. `MFS`) | `~65+` files across `2` repos | **`0` (1 shared library)** |
| In-tree build clutter (`RTK_Rover/build/`) | `202.36 MB` (`2,613` files) | **`0 B`** |
| Active `CHANGELOG.md` size | `1,497 L` / `205 KB` | **`< 450 L` / `< 35 KB`** |
| Active `docs/plans/` documents | `7` legacy overlapping plans | **`3` sequenced plans (`Plan 0 -> 1 -> 2`) + `archive/`** |
| Cross-project fusion readiness (`matrix_math` + `ahrs_9dof`) | `MFS`-only | **Ported in Plan 0; shared in Core (`MFS` & `RTK_Rover`)** |
