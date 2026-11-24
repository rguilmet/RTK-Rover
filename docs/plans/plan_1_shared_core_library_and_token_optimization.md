# Plan 1: Unified Shared Core Library (`Field_Instrument_Core`), Codebase Modularization & Token Optimization

> [!NOTE]
> **Status:** Active Execution Roadmap -- **Phase 1.0 & Phase 1.1 (`1.1A v0.10.8`, `1.1B v0.10.9`, `1.1C v0.10.10`) COMPLETE (`2026-11-24`)**, Phase 1.2 (`Field_Instrument_Core` Extraction) Next  
> **Document Version:** `1.2.4`  
> **Baselines:** `RTK_Rover v0.10.10` | `CAD v2.2.4` | `Magnetic_Field_Scanner v5.1.13`  
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
| `CHANGELOG.md` | `29,879 B` (`248 KB` pre-split) | `137 L` (`1,663 L` pre-split) | **Compliant** | **Done in Ph 1.0 (`2026-11-24`)**: `< v0.9.0` -> `docs/changelog_archive_v0_1_to_v0_8.md` |
| `src/lvgl_port/` (`11` files) | `max 34 KB` | `max 792 L` | **Compliant** | **Done in Plan 0 & Ph 1.1C (`v0.10.10`)**: `lvgl_port_core.c` (`405 L / 16,262 B`) decoupled via `BoardDisplayCfg_t` |
| `RTK_Rover.ino` + `rover_*` (`7` files) | `max 35,086 B` (`80.7 KB` pre-split) | `max 760 L` (`1,675 L` pre-split) | **Compliant** | **Done in Ph 1.1B/C (`v0.10.10`)**: `.ino` (`631 L / 30,278 B`), `rover_audio` (`321 L`), `rover_power` (`175 L`), `rover_sensors` (`760 L`) |
| `src/e220_driver/` (`5` split files) | `max 25,759 B` (`68.4 KB` pre-split) | `max 626 L` (`1,596 L` pre-split) | **Compliant** | **Done in Ph 1.1A (`v0.10.8`)**: `config_mode3` (`626 L`), `gatekeeper` (`449 L`), `tdma` (`535 L`), `internal.h` (`104 L`) |
| `src/waypoint_manager/` (`4` split files) | `max 28,195 B` (`45.6 KB` pre-split) | `max 748 L` (`1,181 L` pre-split) | **Compliant** | **Done in Ph 1.1B (`v0.10.9`)**: `csv_store` (`748 L`), `survey_log` (`471 L`), `internal.h` (`18 L`), injected FS/mutex hook |
| `src/um982_driver/` (`4` split files) | `max 29,344 B` (`47.5 KB` pre-split) | `max 734 L` (`1,146 L` pre-split) | **Compliant** | **Done in Ph 1.1A (`v0.10.8`)**: `nmea_parser` (`734 L`), `config_audit` (`546 L`), `internal.h` (`53 L`) + F-19/F-13/F-14 |
| `src/settings_manager/settings_manager.cpp` | `39,914 B` (`40,720 B` pre-F12) | `786 L` (`770 L` pre-F12) | **Compliant** | **Done in Ph 1.1C (`v0.10.10`)**: F-12 atomic `.tmp` + verify + `rename()` + `s_settings_mux` |
| `scripts/e220_pacer.py` | `81,545 B` | `1,668 L` | `1.77x B / 2.09x L` | Optional split (`rover_protocol.py` + `update_e220_pacer.sh`) |
| `scripts/generate_wiring_diagrams.py` | `87,745 B` | `1,319 L` | `1.90x B / 1.65x L` | Split lazily when diagrams change next |
| `scripts/bench_e220_multirate.py` | `80,845 B` | `1,725 L` | `1.75x B / 2.16x L` | Keep self-contained CLI bench suite; inspect via offsets |
| `docs/mechanical/cad/generate_rover_enclosure.py` | `79,760 B` | `1,646 L` | `1.73x B / 2.06x L` | Split lazily by CAD sub-assembly when modified next |
| `scripts/cogo_solver.py` | `58,880 B` | `1,165 L` | `1.28x B / 1.46x L` | Optional split (`traverse_math.py` + `csv_exporter.py`) |
| `scripts/stage2_bench_verify.py` | `47,000 B` | `1,028 L` | `1.02x B / 1.29x L` | Optional split if extended |

### 2.2 Workspace, Cache & Tooling Hygiene Findings
1. **In-Tree `build/` & `scratch/` Remnants (`DONE` `2026-11-24`):** Purged `RTK_Rover/build/` (`202.36 MB`, `2,860` items) and `RTK_Rover/scratch/` (`2` stale scripts from Oct 29); `arduino-cli` continues using `%LOCALAPPDATA%/arduino/sketches/`.
2. **`scripts/` & Build-Config Clutter (`DONE` `2026-11-24`):** Deleted `scripts/__pycache__/` (`23` stale `.pyc` files), obsolete `302 B` wrapper stubs (`scripts/setup_um980.py`, `scripts/setup_unicore.py`), and obsolete `platformio.ini`; updated `CONTRIBUTING.md` with the authoritative `arduino-cli` command and added `scripts/README.md` indexing all `22` active utilities.
3. **Workspace Skills Status (`DONE` `2026-11-24`):** Workspace `.agents/skills/` provides `rtk-firmware-release/SKILL.md`, `rtk-cad-pipeline/SKILL.md`, and newly added `rtk-field-log-analysis/SKILL.md` (covering `stage2_bench_verify.py`, `analyze_rtk_log.py`, `analyze_field_test.py`, `I2C` `0x103` triage, `[MEM] Post-Setup Heap`, `[10DOF]`, `[RTCM]`, and `[SCREENSHOT]` diagnostics).
4. **`libraries/lv_conf.h` Resolution Audit & Fallback Reconciliation (`DONE` `2026-11-24`):** Verified via `.d` dependency files and `compile_commands.json` that `arduino-cli` passes `-I<sketch_dir>` to `libraries/lvgl` (`v9.6.0`), so `__has_include("lv_conf.h")` already resolves `RTK_Rover/lv_conf.h` (`0` warnings, `LV_USE_FLOAT 1`, zero `%f` calls in `lv_label_set_text_fmt()`) and `Magnetic_Field_Scanner/lv_conf.h` (`LV_USE_FLOAT 0`) ahead of the shared fallback. Archived the stale LVGL `v8.4.0` `libraries/lv_conf.h` as `libraries/lv_conf - v8.4.0.h` and replaced `libraries/lv_conf.h` with an LVGL `v9.6.0` fallback (`LV_USE_FLOAT 0`); `Magnetic_Field_Scanner/lv_conf.h` (`LV_COLOR_DEPTH`, `LV_ASSERT_HANDLER_INCLUDE`, `LV_CALENDAR_DEFAULT_*`, `LV_BUILD_EXAMPLES/DEMOS=1`) is deferred cleanly to Phase 1.3 Stage 2 (`MFS v5.1.x -> v6.0.0`).

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

### Phase 1.0 -- Immediate Repo, Search & Token Hygiene (Zero Functional Firmware Change) -- `DONE (2026-11-24)`
1. **Purge Stale In-Tree `build/` (`202.36 MB`) & `scratch/` (`DONE` `2026-11-24`):**
   * `RTK_Rover/build/` and `RTK_Rover/scratch/` deleted from disk; `.agents/skills/rtk-firmware-release/SKILL.md` confirms out-of-tree sketch cache (`%LOCALAPPDATA%/arduino/sketches`).
2. **Clean `scripts/` Directory, Stale Configs & Document Utilities (`DONE` `2026-11-24`):**
   * Deleted `scripts/__pycache__/` (`23` `.pyc` files), obsolete `302 B` wrapper stubs (`scripts/setup_um980.py`, `scripts/setup_unicore.py`), and obsolete `platformio.ini`; replaced PlatformIO instructions in `CONTRIBUTING.md` with `arduino-cli` (`custom_large_partitions.csv` retained).
   * Created [`scripts/README.md`](../../scripts/README.md) indexing all `22` active utilities (`__filename__` + SemVer convention) and linked it from `README.md` Section 7.
3. **Split `CHANGELOG.md` (`1,663 L / 248.4 KB` -> `137 L / 29,879 B` active, `DONE` `2026-11-24`):**
   * Moved legacy entries `< v0.9.0` (`v0.1.0 - v0.8.19`, including `CAD v1.0.0 - v2.2.2`, `132` release headings) verbatim into [`docs/changelog_archive_v0_1_to_v0_8.md`](../changelog_archive_v0_1_to_v0_8.md), keeping active `CHANGELOG.md` (`v0.9.0+`, `14` headings) cleanly single-read (`< 450 L / < 40 KB`) with zero lost entries.
4. **Complete Workspace Skill Coverage (`DONE` `2026-11-24`):**
   * Added workspace skill `.agents/skills/rtk-field-log-analysis/SKILL.md` alongside `rtk-firmware-release` and `rtk-cad-pipeline`, documenting `analyze_field_test.py`, `analyze_rtk_log.py`, `stage2_bench_verify.py`, `I2C` `0x103` triage, `[MEM] Post-Setup Heap`, `[10DOF]`, `[RTCM]`, and `[SCREENSHOT]` diagnostics.
5. **Reconcile `libraries/lv_conf.h` (`v8.4.0` -> `v9.6.0` Fallback, `DONE` `2026-11-24`):**
   * Verified `arduino-cli` resolves `RTK_Rover/lv_conf.h` and `Magnetic_Field_Scanner/lv_conf.h` locally via `-I<sketch_dir>` + `__has_include("lv_conf.h")`. Archived `libraries/lv_conf.h` (`v8.4.0`) to `libraries/lv_conf - v8.4.0.h` and installed an LVGL `v9.6.0`-compatible `libraries/lv_conf.h` fallback (`LV_USE_FLOAT 0`, preserving strict C `snprintf()` + `lv_label_set_text()` discipline). Deferred `Magnetic_Field_Scanner/lv_conf.h` deprecation cleanup to Phase 1.3 Stage 2 (`MFS v5.1.x -> v6.0.0`) and verified zero warnings on `RTK_Rover`.

### Phase 1.1 -- In-Place Modularization (`<= 800 L`), Core Decoupling & Robustness Hardening -- `DONE (v0.10.8 - v0.10.10, 2026-11-24)`

Every C/C++ split and new reusable module MUST enforce four dependency boundaries:
1. **Strict Direction (`App Adapter` -> `Core Module`, Never Reverse):** Core modules receive pins, bus/device handles, display config, signed `3x3` axis-permutation matrices, `StorageFsHandle_t { fs::FS *fs; SemaphoreHandle_t fs_mux; }`, and log callbacks via `*Config_t` / `*Hooks_t` structs -- **zero** `#include "../../user_config.h"`, **zero** `current_settings`, **zero** `RoverUIData_t`, and **zero** `get_active_fs()` globals.
2. **No `#define Serial SerialLogger` in Core:** Keep `#define Serial SerialLogger` strictly inside `RTK_Rover`'s app layer; shared modules log via `fic_log_printf(...)` or injected callbacks.
3. **Instance-Safe State (`extern "C"` Friendly):** Replace static/global filter state (such as `MFS` `MadgwickAHRS.c` `q0..q3` and `sen0696_driver.cpp` `s_state` / `beam_imu_dev_handle`) with explicit context structs (`FicAhrs9Dof_t *`, `FicSen0696_t *`).
4. **Display Core vs. App UI Separation:** `lvgl_port_core.c` owns QSPI/DMA/PSRAM wrapping (`heap_caps_malloc_extmem_enable(64)` -> `4096`), `rover_lvgl_capture_framebuffer()`, and generic locks; app tiles stay in `RTK_Rover`.

**Exact File Splits, Decouplings & Hardening (`v0.10.7` Baseline):**
1. **`src/lvgl_port/` (`11`-File Split COMPLETE in Plan 0 -> Core Decouple `DONE` `v0.10.10` `2026-11-24`):**
   * Removed `lvgl_ui_internal.h` include from `lvgl_port_core.c` (`405 L / 16,262 B`); parameterized display/touch/EXIO/task config & callbacks via `BoardDisplayCfg_t` + `lvgl_port_init_with_cfg()`, keeping PSRAM `extmem_enable(64)` wrap, mutex, and lock-free FB capture cleanly separated from app UI tiles.
2. **Split `src/e220_driver/e220_driver.cpp` (`1,596 L / 68.4 KB` -> `3` Units + `e220_driver_internal.h`, `DONE` `v0.10.8` `2026-11-24`):**
   * `e220_config_mode3.cpp` (`626 L / 23,981 B`), `e220_rtcm_gatekeeper.cpp` (`449 L / 18,564 B` -- exporting `e220_rtcm_crc24q_compute()` & `e220_rtcm_validate_frame()` for Plan 2's `rtcm_arbiter`), `e220_tdma_scheduler.cpp` (`535 L / 25,759 B`), and `e220_driver_internal.h` (`104 L / 3,944 B`).
3. **Split `src/um982_driver/um982_driver.cpp` (`1,146 L / 47.5 KB` -> `2` Units + `um982_driver_internal.h` + F-13/F-14/F-19, `DONE` `v0.10.8` `2026-11-24`):**
   * `um982_nmea_parser.cpp` (`734 L / 29,344 B`) + `um982_config_audit.cpp` (`546 L / 24,390 B`) + `um982_driver_internal.h` (`53 L / 1,967 B`).
   * **F-19 (`COM2` Cleanup, `DONE`):** Removed obsolete ESP32 `COM2` legacy macros/stubs/aliases (`um982_forward_rtcm_to_com2`, `um982_poll_com2`, `send_um982_com2_test_pulse`, `com2_rx_bytes` -> `unicore_crc_errors`, `ROVER_UM982_COM2_BAUD`, `ROVER_PIN_UM982_TXD1`/`RXD1`, `t4_com2_lbl` -> `t4_um982_audit_lbl`, and `COM2` comments across `RTK_Rover.ino`, `rover_api.h`, `user_config.h`, `ui_tile4_diagnostics.c`, `e220_driver`), preserving receiver-side `UNLOGALL COM2` and `#LOGLIST` `COM2:0` verification.
   * **F-13 (Unicore CRC32 Check, `DONE`):** Added bitwise NovAtel/Unicore CRC32 (`um982_verify_unicore_crc32()`, `0 B` `DRAM0`) on `#VERSIONA`, `#UNIHEADINGA`, and `#BASEINFOA` before logging or parsing alongside NMEA XOR `*XX` (`um982_verify_nmea_checksum()`), verified against `71,304` captured frames (`71,300/71,300` valid matched, `4/4` corrupted frames rejected).
   * **F-14 (Non-Blocking Receiver Audit Safety, `DONE`):** Replaced runtime blocking audit/reconfig loops with `um982_audit_and_config_step()` state machine (`um982_request_receiver_audit()`, `um982_is_receiver_audit_active()`) so Core 0 never drops RTCM3 > `1 s` on reconnects or runtime queries.
4. **Split `src/waypoint_manager/waypoint_manager.cpp` (`1,181 L / 45,645 B` -> `2` Units + `waypoint_manager_internal.h`, `DONE` `v0.10.9` `2026-11-24`):**
   * `waypoint_csv_store.cpp` (`748 L / 28,195 B`) + `waypoint_survey_log.cpp` (`471 L / 18,379 B`) + `waypoint_manager_internal.h` (`18 L / 461 B`), adding C++ `waypoint_manager_set_fs()` FS/mutex injection while preserving PSRAM `g_waypoints`, `12` `kNoteTags`, `s_stem_cache[64]`, and circular `atan2` heading averaging.
5. **Slim `RTK_Rover.ino` (`1,675 L / 80,693 B` -> `575 L / 28,043 B`, `DONE` `v0.10.9` `2026-11-24`):**
   * Extracted Priority-4 ES8311 `task_audio_beep` + `beepQueue` + `rover_audio_update_nav_guidance()` into `src/rover_audio/` (`rover_audio.cpp` `321 L / 13,111 B`).
   * Extracted AXP2101 polling, `rover_power_get_snapshot()`, `rover_power_flush_all_logs()` (staging Plan 2's `< 3.40 V` low-battery SD flush hook), and 30s auto-shutdown timer into `src/rover_power/` (`rover_power.cpp` `175 L / 7,559 B`).
   * Extracted Core 1 QMI8658/MMC5983MA/SEN0696 10-DOF polling, PSRAM `s_beam_ahrs` init + `|B|` collapse self-heal, `MagCalWizard_t`, and BMP581 baro elevation into `src/rover_sensors/` (`rover_sensors.cpp` `754 L / 34,453 B`), compiling with `0` warnings (`1,754,134 B` / `55%` flash, `69,184 B` / `21%` static `DRAM0`).
6. **Decouple Core Candidates & Settings Integrity (`F-12`, `DONE` `v0.10.10` `2026-11-24`):**
   * Stripped `settings_manager.h` + `serial_logger.h` from `imu_bsp.c` (`253 L`, injected `imu_set_log_callback()`, `imu_set_debug_logging()`, tare args); stripped `settings_manager.h` + `um982_driver.h` + `Serial` from `rtc_bsp.cpp` (`221 L`, injected UTC offset & sys-clock sync guard callbacks); refactored `sen0696_driver.cpp` (`492 L`) onto `FicSen0696_t *`; aligned `nmea_logger.cpp` (`225 L`) and `route_logger.cpp` (`289 L`) to `rtcm_logger`'s decoupled `s_buf_mux`/`s_file_mux` + `fs_mux` PSRAM rings + `loop()` `route_logger_flush()`.
   * **F-12 (`settings_manager` Atomicity, `DONE`):** Added `write_json_atomic()` (`/settings.json.tmp` + read-back length verify + `.bak` + `rename()` + `.tmp` recovery) and `s_settings_mux` snapshot locking (`settings_manager.cpp` `786 L / 39,914 B`), compiling with `0` warnings (`1,755,030 B` / `55%` flash, `69,368 B` / `21%` static `DRAM0`).
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
| C/C++ files exceeding `800 L / 46 KB` | `4` app files (`Ph 1.1A/1.1B DONE`; `nav_engine.cpp` at `804 L`) | **`0` core/app source files** |
| Duplicated BSP/codec/driver files (`RTK_Rover` vs. `MFS`) | `~65+` files across `2` repos | **`0` (1 shared library)** |
| In-tree `build/` & `scratch/` clutter | **`0 B` (purged `2026-11-24`)** | **`0 B` (permanent)** |
| Active `CHANGELOG.md` size | `137 L` / `29.9 KB` (`DONE Ph 1.0`) | **`< 450 L` / `< 40 KB`** |
| Active `docs/plans/` documents | `2` active (`Plan 1`, `Plan 2`) + `8` in `archive/` | **`2` sequenced plans + `archive/`** |
| Shared math/fusion & dual-bus I2C readiness | **Done in `src/` (`v0.9.0..v0.9.2`)** | **Promoted into `Field_Instrument_Core` w/ host tests** |
