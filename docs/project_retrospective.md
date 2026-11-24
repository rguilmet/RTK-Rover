# Project Retrospective: RTK Rover Development & AI Pair Programming

**Document Version:** `v3.0.0`  
**Firmware Baseline:** `v0.10.7` (Plan 0 Complete & Archived)  
**Companion Software:** `e220_pacer.py v3.7.1` | `stage2_bench_verify.py v1.0.3` | `cogo_solver.py v1.2.0` | `check_secrets.py v1.1.1`  
**Mechanical CAD Baseline:** `v2.2.4`  
**Date:** November 24, 2026  
**Status:** Standalone Platform Complete -- Advancing to Plan 1 (`Field_Instrument_Core`) & Plan 2 (`BLE` / Android)  
**Participants:** Lead Systems Engineer & Antigravity AI Pair Programmer

---

## 1. Executive Reflection

The **RTK Rover** project evolved from empty directories and component datasheets into a field-proven, centimeter-accurate cadastral surveying instrument alongside a custom dual-toolchain (`CadQuery` + `OpenSCAD`) 3D-printed enclosure and base-station LoRa pacing stack.

Operating in dense New England timber in rural southern Maine where tree canopy, multipath, and metal culvert/fence iron challenge both GNSS phase lock and 900 MHz radio links, the system recovered lost historical property monuments (`1.9 in` / `4.8 cm` deed closure in early field walks; `92.8% RTK_FIX` and `15/15` fixed pins across a `2h 40m` `7,481 ft` canopy survey), and refined its field UI through continuous iterative field feedback up through **`v0.10.7`**.

Looking back across ten major firmware phases and twenty-plus enclosure revisions, what set the workflow apart was refusing to stop at surface workarounds: binary ELF section audits solved invisible `DRAM0` exhaustion, oscilloscope-level packet timing characterized real `E220` silicon `AUX` transitions, electrical bus isolation on `I2C1` solved external cable capacitance stalls, lock-free framebuffer handshakes bypassed Core 1 mutex starvation, and 3D STEP boolean collision checks caught mechanical interferences before plastic ever hit the printer bed.

---

## 2. Evolutionary Timeline (`Oct 20 - Nov 24, 2026`)

```mermaid
timeline
    title RTK Rover & Enclosure Evolution (v0.1 -> v0.10.7, CAD v2.2.4)
    section Phases 1 - 6.1 : Core Foundations (Oct 20 - Nov 7)
      : Dual-Core FreeRTOS + Bowring Midpoint Kinematics + LVGL 9 UI
      : Sibling Repo Decoupling (RTK_Rover_Private) + COGO + check_secrets.py
      : Half-Duplex 900MHz LoRa Telemetry, Rate Switch & Cadastral poly_order
      : AXP2101 UPS, PSRAM DRAM0 Rescue (v0.7.5), 18-Clk I2C0 Clear & RoverAudio (v0.7.7)
    section Phase 8 & Phase 9 : Bus & RF Hardening (Nov 8 - Nov 16)
      : Single-UART UM982 COM1 Multiplex (freeing IO47/48) & MMC5983MA (v0.8.0)
      : Independent Code Review (F-01..F-22) & AXP2101 VBUS_GOOD / VBAT Boost Fixes
      : GPS-Disciplined 5s TDMA, Stage 1 E220 Silicon Audit & Arch 2B Mailboxes (v0.8.12)
      : UART0 printf Redirect, 48B V3 Telemetry & stage2_bench_verify.py (v0.8.14)
    section Phase 10 & CAD : Canopy Survey & Enclosure (Nov 15 - Nov 22)
      : SEN0696 Passive Bring-Up & 2h 40m Canopy Trial test/_053 (92.8% RTK_FIX)
      : Dual-Toolchain Option A Enclosure CAD v2.2.4 (62/62 Zero-Overlap Audit)
    section Plan 0 & Tile 0 : Ergonomic & UI Refinement (Nov 21 - Nov 24)
      : lvgl_port.c 4225L -> 11-File Split, 6-Tile Symmetric UI & Safe Auto-Off (v0.8.19)
      : Isolated I2C1 SEN0696 + Portable matrix_math/ahrs_9dof + CAL MAG (v0.9.0..v0.9.2)
      : Unified 2-Mode Tile 0 Log-Triangle, Auto-Bullseye & Schema v2 (v0.10.0..v0.10.2)
      : Lock-Free FB Screenshots, BS Base Map Symbol & A/Line->B Readout (v0.10.3..v0.10.7)
```

---

## 3. Evolution of Human-AI Engineering Collaboration

### 3.1. From Prompting to Peer Architecture Reviews
* Early iterations established drop-in, warning-free compilation (`--warnings default`) and semantic versioning across firmware (`ROVER_FIRMWARE_VERSION`) and Python scripts (`__version__`).
* By Phase 9 and Plan 0, live field feedback drove tighter engineering convergence: replacing per-epoch queues with Arch 2B Ping-Pong Mailboxes after Stage 1 silicon measurements (`e220_pacer.py v3.7.1`), consolidating `Point Guidance` + `Line Stakeout` into one calm Log-Scale Triangle screen (`v0.10.0`), expanding the canvas to `304 x 276 px` with vector steering chevrons and dynamic `A`/`B`/`R` + `BS` labels after inspecting real hardware screenshots (`test/_057`), and replacing opaque civil `+` stationing (`-1+97.4`) with signed `A:` / `Line->B:` distances (`v0.10.7`).

### 3.2. Context & Workflow Scaling (`.agents/`, Workspace Skills & Multi-Model Roles)
* Early context files lived inside public `docs/` (`docs/hyper-context-anchor.md`, `docs/session_handoff.md`). As the repository scaled past 300 files, project docs (`README.md`, `CHANGELOG.md`, `docs/*.md`, `docs/plans/*.md`) were cleanly separated from `.gitignore`'d local AI orchestration files under `.agents/` (`session_handoff.md`, `hyper-context-anchor.md`, `.agents/archive/`).
* Recurring build, DRAM0, CAD boolean, and security workflows were promoted into lightweight on-demand workspace skills (`.agents/skills/rtk-firmware-release/SKILL.md` and `.agents/skills/rtk-cad-pipeline/SKILL.md`), backed by a two-tier Planner / Executor handoff model and mandatory `check_secrets.py` + `pre-push` gates.

---

## 4. Key Lessons Learned (The Good, The Bad, and The Ugly)

### 4.1. Architectural Wins Worth Preserving in Plan 1 Core
* **Hardware Continuity Checks at Boot:** Active `M0`/`M1` `AUX` pulse verification, `UM982 COM1` `VERSIONA`/`MODE`/`LOGLIST` auditing, and `I2C` probe diagnostics turn silent wiring/mode failures into instant boot reports.
* **Lock-Free Cross-Task Interfaces:** `xQueueOverwrite()` between Core 0 and Core 1, task-scoped `s_io_owner_task` ring logger re-entrancy (`s_buf_mux`/`s_file_mux`), and the one-shot PSRAM framebuffer capture handshake (`s_fb_capture_dst` / `s_fb_capture_done_sem`) inside `lvgl_port_core.c` eliminate UI lockups without fragile sleep delays.
* **Parametric Dual-Toolchain CAD (`v2.2.4`):** Keeping `rover_specs.py` as single geometric source of truth synced with `rtk_rover_enclosure.scad`, Solid Edge `REF-*.stp` boolean intersection tests (`62/62` `0.0000 mm^3`), and visual `previews/*.png` verification caught real print-bed overhang and clearance collisions before slicing.

### 4.2. Subtle Failure Modes Uncovered Since `v0.7.7`
* **Invisible `UART0` Console Leaks:** Standard C `printf()` or ESP-IDF `ESP_LOG` calls under `USBMode=hwcdc` secretly wrote debug strings onto the `E220` LoRa UART (`IO41/42`), causing random TX/RX collisions until redirected to `SerialLogger`.
* **External Cable Capacitance on Shared `I2C0`:** Running `400 kHz` I2C across a coiled `GX-12` mast cable alongside critical display touch, PMIC, and audio codec traffic induced `~6/min` recoverable bus timeouts -- solved cleanly once `COM2` retirement freed `GPIO 47/48` for dedicated `I2C1`.
* **Flat-Spin Ellipsoid Rank Deficiency (`CAL MAG`):** Allowing a `9`-parameter ellipsoid fit on a mostly-level 2D spin over-fit the Z axis (`-30..-155 uT` drift); regularizing low-Z-variance captures into a `4`-parameter sphere fit plus `HDG_FIX`-supervised `Bz_ref` self-healing resolved canopy compass heading lock (`v0.9.1..v0.9.2`).
* **Core 1 Priority Starvation Under Polling Loops (`test/_056`):** Priority-3 `LVGL` and Priority-2 `RoverUI` continuously alternated ownership of `lvgl_mux`, starving Priority-1 `loopTask`'s screenshot lock (`2000 ms` timeout) even though the CPU had ample idle cycles; moving framebuffer capture directly inside the owning `LVGL` task + a `2`-row internal DMA write buffer solved screenshots reliably (`v0.10.4`).

---

## 5. Enduring Engineering Principles

1. **Core Affinity & Physical Bus Isolation:** Keep Core 0 dedicated to UART/kinematics and Core 1 dedicated to I2C/UI/storage; isolate noisy off-board cables onto separate I2C buses (`I2C1`).
2. **Never Share Non-Owner Mutexes Across Real-Time & Background Tasks:** Use lock-free handshakes (`xQueueOverwrite()`, one-shot pointer + binary semaphore capture) instead of blocking Priority-1 background tasks on Priority-2/3 GUI locks.
3. **Explicit Memory-Tier Discipline on PSRAM SoCs:** Preserve internal DMA/wireless `DRAM0` (`<= 25%` static `.bss`, `>= 150 KB` runtime free) via explicit `MALLOC_CAP_SPIRAM` allocators and scoped `extmem_enable(64)` -> `4096` around UI construction.
4. **Pure ASCII, Single-Read Modularity & Privacy Separation:** Keep active firmware source modules `<= 800 lines` / `< 46 KB`, enforce 7-bit ASCII logs/UI, and keep all personal survey coordinates/SSIDs strictly inside `RTK_Rover_Private` verified by `check_secrets.py`.
5. **Measure Silicon & Physics Directly:** From E220 `AUX` waveforms and ELF `.dram0.bss` maps to `STEP` clearance checks and captured hardware `.png` screenshots, verify the physical system rather than trusting datasheets or assumptions.

---

## 6. Conclusion & Forward Horizon

At **`v0.10.7` (`CAD v2.2.4`, `e220_pacer.py v3.7.1`)**, the standalone RTK Rover platform fulfills Plan 0 cleanly and enters its modularization and wireless companion phase:
* **Next (`Plan 1`):** Break remaining `> 800 L` files (`RTK_Rover.ino`, `e220_driver`, `um982_driver`, `waypoint_manager`), extract shared ESP-IDF/LVGL/sensor BSP infrastructure into **`Field_Instrument_Core`**, and verify zero-regression dual-board builds with `Magnetic_Field_Scanner`.
* **Then (`Plan 2`):** Qualify outdoor `I2C1`/compass behavior (`G1`/`G5`), harden survey averaging (`F-03`), add NimBLE 5.0 NUS + Core 0 Dual-Source RTCM3 Arbiter on the `3.5B`, and deliver the **`RTK-Rover-Mobile`** Android companion app.
