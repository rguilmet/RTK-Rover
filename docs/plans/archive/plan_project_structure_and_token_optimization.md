# [ARCHIVED] Project Structure, Model Utilization & Token Optimization Plan

> [!NOTE]
> **[ARCHIVED -- CONSOLIDATED INTO Plan 1 (2026-11-21)]:** All remaining repo hygiene, C/C++/Python modularization, `build/` cleanup, `CHANGELOG.md` rolling archive, and `.agents/skills/` items (paired with `Field_Instrument_Core` shared-library extraction) are consolidated into the active roadmap [`../plan_1_shared_core_library_and_token_optimization.md`](../plan_1_shared_core_library_and_token_optimization.md).

**Document Version:** `1.0.0`  
**Date:** November 15, 2026  
**Firmware Baseline:** `v0.8.7` (`e220_pacer.py v3.6.1`)  
**Scope:** Multi-session roadmap to maximize AI context efficiency (`read_file` single-call fit, progressive skill loading, build-cache search hygiene) and multi-model (`Argon` / `Hydrogen` / Orchestrator) effectiveness across `RTK_Rover`.

---

## 1. Executive Summary & Measured Repository Audit (`v0.8.7`)

Already completed in `v0.8.7`:
* Linked workspace root [`AGENTS.md`](../../AGENTS.md) to [`GEMINI.md`](../../GEMINI.md) via `@[...]` inclusion, consolidated non-negotiable hardware/LVGL/geodesic roadblocks, eliminated duplicate rules, and added no-chat-echo token discipline.
* Isolated AI context, prompts, and review docs (`hyper-context-anchor.md`, `session_handoff.md`, `session_prompt*.md`, `project_review_sonnet.md`) out of `docs/` into `.gitignore`'d `.agents/`.
* Consolidated planning documents into `docs/plans/` and deprecated the completed Phase 9 TDMA plan.

### Remaining Token & Context Bottlenecks (Measured Baseline)

Antigravity's `read_file` returns up to **`800 lines` / `46,080 bytes`** per call. Files far above that threshold force multi-chunk reads or increase full-file search clutter:

| Asset | Current Size | Lines | `read_file` Chunks (800L / 46KB) | Primary Bottleneck |
| :--- | :---: | :---: | :---: | :--- |
| `src/lvgl_port/lvgl_port.c` | `192,855 B` | `3,820` | `~5x` | Monolithic UI file combining QSPI driver setup, header/nav bars, and all 5 LVGL tiles (`Tile 0`–`Tile 4`). |
| `CHANGELOG.md` | `135,983 B` | `952` | `~3x` | Historical entries (`v0.1.0`–`v0.6.x`) consume ~34k tokens when scanning recent history. |
| `scripts/e220_pacer.py` | `77,834 B` | `1,571` | `~2x` | Base station TCP/RTCM3 ingest, E220 hardware pacing, interleaver, telemetry unpacking, and logger in one module. |
| `src/e220_driver/e220_driver.cpp` | `60,480 B` | `1,275` | `~2x` | Mode 3 EEPROM config/readback mixed with Mode 0 superframe TDMA, RTCM CRC24Q gatekeeper, and RSSI polling. |
| `scripts/cogo_solver.py` | `57,452 B` | `1,165` | `~2x` | Monolithic traverse solver, geodesic math, and multi-format exporters (CSV/KML/GeoJSON/DXF). |
| `src/um982_driver/um982_driver.cpp` | `44,182 B` | `964` | `> 1x` | Approaching `800-line` threshold (`COM1` RTCM TX, NMEA/`UNIHEADINGA`/`BASEINFOA` demux, receiver state audit). |
| `src/waypoint_manager/waypoint_manager.cpp` | `39,796 B` | `913` | `> 1x` | Chunked CSV loader, stem cache, survey logging, and file management in one unit. |
| `build/esp32.esp32.esp32s3/sketch/` | `> 650 KB` | — | Duplicate matches | In-tree Arduino CLI build directory mirrors source and docs (`192 KB` `lvgl_port.c`, `124 KB` `CHANGELOG.md`, etc.), doubling `Select-String` / grep hits. |
| `docs/` root & `scripts/` | 25+ files in `docs/`, 21 files in `scripts/` | — | Discovery overhead | Loose hardware pinout CSV/XLSX/PNG files alongside core manuals in `docs/`, and no `scripts/README.md` index. |

---

## 2. Phase 1: Source File Modularization (`< 800 Lines` / `< 46 KB` Target)

Splitting oversized C/C++ and Python modules around clean single-responsibility boundaries enables **1-call file reads** for `Hydrogen` and keeps `Argon` planning scopes tight with zero risk of editing unrelated screens or drivers.

### Phase 1A — Split `src/lvgl_port/lvgl_port.c` (`3,820 lines` $\rightarrow$ 7 Units)
Create an internal module header (`src/lvgl_port/lvgl_ui_internal.h`) declaring shared styles, widget handles, `rover_lvgl_lock()` / `rover_lvgl_unlock()`, and PSRAM allocation helpers, and split `lvgl_port.c` into:

1. `lvgl_port_core.c` (`~450 lines`): QSPI `AXS15231B` + touch driver setup, DMA buffers, `heap_caps_malloc_extmem_enable(64)` wrapper around UI construction, `lvgl_port_task`.
2. `ui_header_nav.c` (`~550 lines`): Persistent 4-row Top Status Header Bar and Bottom Screen Navigation Bar (`< Prev`, `X/5: Title`, `Next >`).
3. `ui_tile0_guidance.c` (`~750 lines`): Multi-Mode Navigation Workspace (Tactical Guidance, Line Stakeout, Mode 2 Map View canvas + HUD zoom/pan).
4. `ui_tile1_logger_settings.c` (`~700 lines`): Field Survey Logger & 3x4 Consolidated Quick Settings Deck (`LORA`, `WIFI`, etc.).
5. `ui_tile2_csv_browser.c` (`~450 lines`): Multi-File CSV Browser and suffix/target switching callbacks.
6. `ui_tile3_compass_level.c` (`~420 lines`): Dual-Antenna Compass, 2D Pole Bubble Level, and `TARE` controls.
7. `ui_tile4_diagnostics.c` (`~500 lines`): Hardware Diagnostics, Dual-Power UPS Card, Pinned LoRa RF Card, and UM982 Receiver State/LOGLIST card.

### Phase 1B — Split `src/e220_driver/e220_driver.cpp` (`1,275 lines` $\rightarrow$ 2 Units)
Create `src/e220_driver/e220_driver_internal.h` and separate hardware Mode 3 configuration from real-time Mode 0 operation:

1. `e220_config_mode3.cpp` (`~450 lines`): Pin init, `REG0`/`REG1`/`REG3` builders, Mode 0$\leftrightarrow$Mode 3 transitions, 3-attempt sync-scanned EEPROM writes, and `REG0` resync readback.
2. `e220_tdma_rtcm.cpp` (`~825 lines`): `rx_stage` `CRC24Q` gatekeeper, superframe TDMA slot scheduling, uplink TX pacing (`s_tx_air_busy_until_ms`), Mode 0 RSSI polling, and 60s dynamic fallback watchdog.

### Phase 1C — Optional Python Module Splits (`scripts/e220_pacer.py` & `scripts/cogo_solver.py`)
* Extract pure protocol packing/unpacking (`FMT_TELEMETRY`, `FMT_RATE_REQ`, `FMT_RATE_ACK`, CRC16) from `scripts/e220_pacer.py` (`1,571 lines`) into `scripts/rover_protocol.py` so `scripts/test_rover_telemetry.py` and log analyzers import a single `< 250-line` protocol module directly.
* Keep `scripts/e220_pacer.py` deployment single-file friendly on the Raspberry Pi if preferred (`update_e220_pacer.sh`), or package via `scripts/update_e220_pacer.sh` syncing both files.

**Verification Gate for Phase 1:** Compile with exact `arduino-cli` command (`--warnings default`, `CDCOnBoot=cdc`), verify 0 compiler warnings, confirm static `DRAM0` stays `<= 25%` (`< 80 KB`), and verify `python scripts/test_rover_telemetry.py` passes `100%`.

---

## 3. Phase 2: Progressive Disclosure via `.agents/skills/` On-Demand Runbooks

Antigravity uses **progressive disclosure** for skills: at turn start, only each skill's 1-sentence description sits in the prompt; the full `SKILL.md` body loads **only when invoked** for that specific task.

Move long, procedural multi-step instructions out of always-on rules or ad-hoc discovery into **workspace skills** under `RTK_Rover/.agents/skills/` (`.gitignore`'d alongside `.agents/`):

1. **`.agents/skills/firmware-build-and-release/SKILL.md`:**
   * Encapsulates the exact `arduino-cli` Windows binary + FQBN flags, `DRAM0` check threshold (`>= 23%` $\rightarrow$ run `xtensa-esp32s3-elf-size -A`), `test_rover_telemetry.py`, `check_secrets.py`, tiered doc sync rules, and commit/push checklist.
2. **`.agents/skills/field-test-log-analysis/SKILL.md`:**
   * Provides a reusable Python analysis template (plus expected superframe/AUX/RSSI baselines from `docs/e220-reference.md v2.5`) for analyzing `test/_0XX` Rover serial logs and Base `pacer_test.log` captures (`CRC24Q` yield, `[RTCM DROP TRUNC]`, `AUX_low` timing, Mode 0 RSSI cadence, rate-switch transitions). Prevents rewriting custom log parsers from scratch in every debug session.

---

## 4. Phase 3: Build Cache Purge & Search Hygiene (`RTK_Rover/build/`)

Currently `c:\...\Arduino\RTK_Rover\build\esp32.esp32.esp32s3\sketch\` contains duplicate snapshots (`192 KB` `lvgl_port.c`, `124 KB` `CHANGELOG.md`, `59 KB` `plan_smart_pole_and_android_app.md`, etc.) generated by prior local builds.

* **Action:** Delete the stale in-tree `RTK_Rover/build/` directory (or move `arduino-cli` `--build-path` outside the workspace, e.g., `%LOCALAPPDATA%\Temp\rtk_rover_build`) so workspace-wide text searches never return duplicate matches from stale compiled copies.
* **Impact:** Eliminates `> 50%` false-positive search noise and prevents agents from accidentally inspecting outdated `build/.../sketch/` files instead of live `src/` files.

---

## 5. Phase 4: `CHANGELOG.md` Rolling Window & Historical Archive

`CHANGELOG.md` has reached **`135,983 B` (`952 lines`)**. Reading or searching it consumes substantial context on routine patch releases.

* **Action:** Archive legacy entries (`v0.1.0` through `v0.6.2`) into `docs/archive/CHANGELOG_v0.1_to_v0.6.md` and keep `v0.7.0+` (and future releases) in root `CHANGELOG.md` with a header link to the archive file.
* **Impact:** Shrinks `CHANGELOG.md` from `~136 KB` to `~25–30 KB` (`< 1` `read_file` chunk), cutting changelog read/grep overhead by `~75–80%`.

---

## 6. Phase 5: `docs/` and `scripts/` Directory Hygiene

### Clean `docs/` Root (Keep Only Core Engineering Docs in `docs/`)
Currently `docs/` root mixes Markdown manuals with 9 hardware spreadsheets/images:
* Move pinout spreadsheets (`Waveshare*.csv`, `Waveshare*.xlsx`, `Expected LoRa Signal Strength.xlsx`) to `docs/hardware/`.
* Move loose setup screenshots (`ESP32-S3-Touch-LCD-3.5-Arduino_Setup.png`, `.webp`) and `Waveshare-ESP32-S3-Touch-LCD-3.5B-GPIO_Pin_Assignment-rev3.png` into `docs/images/`.
* Keep only core Markdown manuals at `docs/` root (`project_overview.md`, `user_manual.md`, `e220-reference.md`, `cogo_solver_guide.md`, `critical_design_review.md`, `project_retrospective.md`), with subfolders `docs/plans/`, `docs/hardware/`, `docs/images/`, `docs/schemas/`, and `docs/archive/`.

### Index `scripts/` & Retire Legacy Stubs
* Add `scripts/README.md` (`~60 lines`) grouping all scripts by purpose (Base Station Pacer/LoRa Setup, Cadastral COGO, Verification & Security, Field Test Analysis) so agents never need to read 5 scripts just to find the right utility.
* Retire or merge the 302-byte wrapper stubs (`setup_um980.py`, `setup_unicore.py`) into `setup_um98x.py` if no longer referenced externally.

---

## 7. Phase 6: Session & Model Utilization Best Practices

* **Clean Session Boundaries:** Start a fresh chat after major milestone boundaries (e.g., before beginning Phase 1A `lvgl_port.c` split or `Field-Instrument-Core` migration). Turn-1 rules load automatically via `AGENTS.md` $\rightarrow$ `GEMINI.md`, and `.agents/session_handoff.md` + `docs/plans/...` preserve continuity with zero prior-chat baggage.
* **Use `/compact` Mid-Milestone:** When a single debugging + test analysis session grows long (e.g., analyzing `test/_038` logs + editing firmware + compiling), use `/compact` after committing a patch to drop raw log transcripts while keeping active architecture decisions.
* **Leverage Orchestrator Split (`Argon` + `Hydrogen`):**
  * Use **`/plan`** when modifying 3+ files, splitting modules (Phase 1), or changing FreeRTOS task/mutex boundaries so `Argon` locks down exact file lists and `Re-plan if:` gates before `Hydrogen` edits code.
  * For small follow-ups (compile, commit, single-line tweak), `Argon` hands off via `exit_plan_mode` in 1–2 sentences without re-reading untouched code.

---

## 8. Recommended Execution Sequence (Across Future Sessions)

1. **Quick Wins (~15 min, Zero Firmware Risk):** Phase 3 (`RTK_Rover/build/` purge + external build path), Phase 4 (`CHANGELOG.md` archive split), and Phase 5 (`docs/hardware/`, `docs/images/`, `scripts/README.md`).
2. **Agent Runbook Setup (~15 min, Zero Code Risk):** Phase 2 (`.agents/skills/firmware-build-and-release/` and `.agents/skills/field-test-log-analysis/`).
3. **Code Modularization (Dedicated Single-Focus Session + Hardware Compile Verify):** Phase 1B (`e220_driver.cpp` split) followed by Phase 1A (`lvgl_port.c` split), compiling and testing after each unit split before merging.
