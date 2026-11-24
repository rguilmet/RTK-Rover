# Critical Design Review (CDR) - RTK Rover

**Document Version:** `v3.0.0`  
**Firmware Baseline:** `v0.10.7` (Plan 0 Complete & Archived)  
**Companion Software:** `e220_pacer.py v3.7.1` | `stage2_bench_verify.py v1.0.3` | `cogo_solver.py v1.2.0` | `check_secrets.py v1.1.1`  
**Mechanical CAD Baseline:** `v2.2.4` (`8/8` manifold STL parts, `62/62` `0.0000 mm^3` real-STEP clearance checks)  
**Date:** November 24, 2026  
**Status:** Plan 0 Complete -- Approved & Ready for Plan 1 (`Field_Instrument_Core`) & Plan 2 (`BLE` / Android Slave Mode)  
**Author:** Engineering Team (Pair Programming Collaboration)

---

## 1. Executive Summary

The **RTK Rover** project was engineered to deliver an ultra-compact, centimeter-accurate, dual-antenna RTK GNSS receiver, cadastral boundary pin locator, and field survey instrument. Built around the **Waveshare ESP32-S3-Touch-LCD-3.5B**, **Holybro H-RTK Unicore UM982**, **Ebyte E220-900T30S** 1W LoRa transceiver, and pole-beam **DFRobot SEN0696** (`BMI323 + BMM350 + BMP581`) 10-DOF sensor, the system projects dual-antenna moving-baseline kinematics (`0.2 deg` true heading) directly to the survey pole centerline and ground tip (`1.800 m`).

Through Phase 10, Plan 0 (`v0.8.19 -> v0.9.0`), and the `v0.10.0..v0.10.7` Tile 0 refinements, the Rover matured into a deeply instrumented standalone survey platform featuring:
1. **Unified 2-Mode Tile 0 (`GUIDANCE` <-> `MAP VIEW`) & `settings.json` Schema v2:** Consolidates point guidance and line stakeout into a `304 x 276 px` symmetric log-scale guidance triangle (`A: DEED` / `A: HERE`), dynamic vertical auto-framing, `< 4.8 m / > 5.3 m` Auto-Bullseye handoff, `R -> B`-prioritized barbed heading arrow, 3-tier vector steering chevrons (`>>>`/`<<<` & `|`), dynamic `A`/`B`/`R` + Map `BS` Base Station labels, signed `A: +/-x.x (x%) | Line->B: +/-x.x` readout, acoustic `L/R` fade-out chirps (`ES8311`), and persistent sticky map zoom (`v0.10.0..v0.10.7`).
2. **Single-UART `UM982 COM1` & Isolated `I2C1` Beam 9-DOF Canopy Fusion (`v0.8.0`, `v0.9.0..v0.9.2`):** Retired `UM982 COM2` (`v0.8.0`) to free `GPIO 47/48` for an electrically isolated `I2C1` bus (`400 kHz`) dedicated to the external `GX-12` `SEN0696` sensor. Added portable `src/matrix_math/` + `src/sensor_fusion/ahrs_9dof` (`25-30 Hz` Madgwick 9-DOF fusion, stationary accel-variance gyro FOC, `CAL MAG` 360-deg PSRAM ellipsoid fit w/ Z-bias self-heal, `HDG_FIX` circular teacher, triple-condition iron anomaly gatekeeper, `0.0 s` `[CMP]` switchover, and `BMP581` `Baro Rel Elev`).
3. **Architecture 2B Async Ping-Pong Mailbox LoRa Link & 5s GPS-Disciplined TDMA (`v0.8.12`, `e220_pacer.py v3.7.1`):** Replaced stale epoch queues with a background RTCM3 ingress thread feeding latest-value ping-pong mailboxes, `<= 200 B` smart coalescing, Stage-1-calibrated `ToA + guard` pacing, `48 B` V3 telemetry (`FW:vX.Y.Z` + ANT1/ANT2/pitch/RSSI/noise), and synchronous 4-ACK 3-rate touchscreen switching across `4.8k / 9.6k / 19.2k` (`4.8k / 30 dBm` CH 75 field default).
4. **Deterministic 6-Tile Dual-Core Firmware, Lock-Free Screenshots & Mechanical CAD (`v0.7.5..v0.10.7`, `CAD v2.2.4`):** Enforces Octal PSRAM offload (`~154-158 KB` free internal SRAM, `20%` static `DRAM0`), Priority-4 `< 1 ms` `RoverAudio`, motion/`REC`-safe `30 s` Auto-Off guard, lock-free PSRAM framebuffer screenshot capture + 2-row internal DMA flush, and a support-free Option A PETG enclosure verified across `62/62` `0.0000 mm^3` real-STEP collision checks.
5. **Code/Data Decoupling & Security Gates:** Separates open-source firmware (`RTK_Rover`) from personal cadastral records (`RTK_Rover_Private`) protected by `check_secrets.py v1.1.1`.

This Critical Design Review documents what succeeded (**The Good**), architectural traps diagnosed and resolved across hardware and software (**The Bad**), silicon/RTOS physics that required permanent guardrails (**The Ugly**), quantitative field verification metrics, and readiness for Plans 1 and 2.

---

## 2. Technical System Architecture Audit (`v0.10.7`)

```text
+---------------------------------------------------------------------------------------------------+
|                                   DUAL-CORE ARCHITECTURE (v0.10.7)                                |
+-------------------------------------------------+-------------------------------------------------+
| CORE 0: Real-Time Telemetry & Kinematics (Pri 4)| CORE 1: Audio, UI, I2C0/I2C1 & Storage (P 4..1) |
| - E220 LoRa UART0 (IO41/42 @ 115200, CH75 4.8k) | - RoverAudio (Pri 4): <1ms ES8311 beepQueue I2S |
| - rx_stage CRC24Q Sliding-Window Gatekeeper     |   * Dual-tone L/R chirps + Geiger/Beeps/Chimes  |
| - Forward Verified RTCM3 -> UM982 COM1 TX       | - lvgl_port_task (Pri 3): LVGL 9 QSPI @ 40 MHz  |
|   (Serial1 IO17/18 via s_com1_tx_mutex, 4KB buf)|   * 11 modules <= 792L, PSRAM extmem_enable(64) |
| - UM982 COM1 RX 3-Way Line Demuxer              |   * Lock-free FB capture handshake (screenshot) |
|   (GGA, RMC, GST, GSV, GGAH, UNIHEADING, BASE)  |   * 0ms same-frame Tile 0 readout recomputation |
| - 3D Midpoint Lever-Arm + Bowring ECEF->WGS84   |   * 6-Tile ring: [T5]-[T0]-[T1]-[T2]-[T3]-[T4]  |
| - Deep-Woods Heading Fusion (nav_heading_beam)  | - RoverUI (Pri 2): Exclusive I2C0 + I2C1 Owner  |
| - Sec 0 48B V3 Uplink + 4-ACK 3-Rate Switch     |   * I2C0 (IO7/8): Touch, QMI8658, RTC, AXP2101, |
| - xQueueOverwrite to roverStateQueue (<=2ms wk) |     ES8311, opt. MMC5983MA (18-clk clear+reinit)|
| - esp_task_wdt supervision (COM1 TX guarded)    |   * I2C1 (IO47/48): Isolated SEN0696 25-30Hz    |
|   ZERO printf() on UART0 (-> SerialLogger)      |     (ahrs_9dof, CAL MAG wizard, BMP581 baro)    |
|                                                 |   * Auto-Off guard & LV_LAYER_TOP 30s modal     |
|                                                 | - loopTask (Pri 1): 5s Debounced Settings Save  |
|                                                 |   (smart FFat skip), Web Portal, SD Ring Flushes|
|                                                 |   & 2-row DMA BMP Screenshot File Writes        |
+-------------------------------------------------+-------------------------------------------------+
```

---

## 3. The Good: Architectural Triumphs

### 3.1. Strict Core Affinity & Single-UART `UM982 COM1` Consolidation (`v0.8.0`)
* **Asymmetric Core Isolation:** Core 0 owns all high-throughput UART streaming (`E220` + `UM982 COM1`), RTCM3 `CRC24Q` gatekeeping, `BASEINFOA` ECEF conversion, and 30 Hz lever-arm kinematics with zero I2C or LVGL coupling (`xQueueOverwrite` into `roverStateQueue`).
* **Single-UART Multiplexing (`COM1` Only):** Consolidating RTCM3 TX + command TX (`s_com1_tx_mutex`, `4096 B` TX ring) and NMEA/`UNIHEADINGA`/`BASEINFOA` + `#`/`$command` RX onto `Serial1` (`GPIO 17/18`) freed `Serial2` (`GPIO 47/48`) and `~6 KB` internal `DRAM0`, enabling physical electrical isolation of the external `SEN0696` cable onto `I2C1`.

### 3.2. Architecture 2B Ping-Pong Mailboxes & GPS-Disciplined TDMA (`v0.8.12`)
* **Zero Epoch Queue Backlog (`e220_pacer.py v3.7.1`):** A dedicated background ingress thread decodes TCP RTCM3 from `str2str` (`1006(5),1074,1084,1094,1230(10)`), drops empty MSMs (`<= 28 B`), and overwrites per-type ping-pong mailboxes (`2.5 s` dynamic / `15.0 s` static TTL). Starvation-fair constellation round robin anchored to `1074` plus `<= 200 B` frame coalescing saturates channel efficiency cleanly across `4.8k`, `9.6k`, and `19.2k` air rates.
* **Consolidated `Second 0` Window & 3-Rate Switching:** Eliminated `850 ms` per-second RSSI gaps during `Seconds 1..(N-1)` by consolidating the `48 B` V3 uplink (`FW:vX.Y.Z` + ANT1/ANT2/pitch/noise/RSSI) and single Mode 0 RSSI query into `Second 0`, backed by 4-ACK `Second 1` / Mode 3 `Second 2` remote air-rate switching and a 60s failsafe rollback.

### 3.3. Isolated `I2C1` `SEN0696` Fusion, Ellipsoid `CAL MAG` & Midpoint Kinematics (`v0.9.0..v0.9.2`)
* **Midpoint Projection:** Placing the survey pole at the `27.75 cm` midpoint of the `55.5 cm` crossbar yields `< 8 mm` ground-tip orbit across `360 deg` pole rotations (`1.800 m` height).
* **9-DOF Canopy Continuity:** Polling `SEN0696` on isolated `I2C1` (`25-30 Hz`) through portable `matrix_math.h` + `ahrs_9dof.{h,cpp}` fills canopy `HDG_FIX` dropouts instantaneously (`0.0 s` switchover into calibrated `[CMP]` with triple-condition iron anomaly coasting into `[GYR]` and `HDG_FIX`-learned True-North/field/pitch tares).

### 3.4. Modular 11-File LVGL UI, Unified Tile 0 Guidance & Mechanical CAD Toolchain (`v0.8.19..v0.10.7`, `CAD v2.2.4`)
* **Readable 6-Tile Modularization:** Splitting the `4,225 L` LVGL monolith into `11` `< 800 L` files cleanly separated dedicated Tile 5 Quick Settings (`5x3` `94x44 px`) and Tile 1 Survey Logger from Tile 0's `304 x 276 px` Unified Log-Scale Guidance Triangle, Auto-Bullseye, and bottom-toolbar Map View (with dynamic `A`/`B`/`R` + `BS` Base Station labels, dual-source `P:` beam pitch fallback, and signed `A:` / `Line->B:` progress).
* **Zero-Overlap Option A Enclosure (`CAD v2.2.4`):** Dual-toolchain CadQuery + OpenSCAD (`Manifold`) parametric design synchronized against real Solid Edge hardware STEP assemblies (`REF-*.stp`), verifying `8/8` single-solid support-free PETG print parts and `62/62` `0.0000 mm^3` interference checks across the `37.5 deg` open-truss pole tilt wedge, downward-D `GX-12` bulkhead, left-wall `3.5 mm` jack tab, and Option B2 `10000 mAh` pod.

---

## 4. The Bad: Architectural Traps & Resolved Defects

* **Summary of `v0.6.x - v0.7.7` Remediations (Preserved):**
  * Open `M0` weak pull-up placing E220 silently in Mode 1 WOR -> boot `M0`/`M1` continuity pulse self-test.
  * Core 0/1 telemetry desync -> `portMUX`-protected Core 1 post-fusion `e220_feed_telemetry_state()`.
  * `$GPGGAH` vs. `UNIHEADINGA` `SLV_FIX` mismatch -> strict alignment with `heading_source`.
  * UTF-8 mojibake (`0xC2 0xB0`) & `LV_USE_FLOAT = 0` `lv_label_set_text_fmt` crashes -> pure 7-bit ASCII (`deg`) and C `snprintf()` + `lv_label_set_text()`.
  * Unsigned debounce timestamp underflow (`~0.9-1.9 s` UI stall) & global `s_in_logger` dropped `[TOUCH]` logs -> Priority-4 `RoverAudio`, `0 ms` same-frame updates, Priority-1 `loop()` signed 5s debounce w/ smart `FFat` skip, and task-scoped `s_io_owner_task`.
  * Internal `DRAM0` exhaustion (`WL_STOPPED 254` in `v0.7.5`) -> migrated `82.9 KB` static `.dram0.bss` buffers to PSRAM and scoped `heap_caps_malloc_extmem_enable(64)` around `create_rover_ui()`, keeping `~154-158 KB` free internal SRAM.
* **Post-`v0.7.7` Remediations (`v0.8.1 - v0.10.7`):**
  * **UART0 `printf()` / `ESP_LOG` Half-Duplex TX Collisions:** Under `USBMode=hwcdc,CDCOnBoot=cdc`, C `printf()` / ESP-IDF logs mapped to `UART0` (`IO41/42` = E220 LoRa), transmitting raw console text on `CH 75`. Resolved via `rover_printf()` + `esp_log_set_vprintf()` redirection off `UART0` to `SerialLogger`.
  * **`AXP2101` `16.37 V` Sentinel & `STATUS2` Charging Bit (`v0.8.1`):** Masked off unconnected VBUS ADC glitch on USB unplug using PMIC `0x00 VBUS_GOOD` register + proper `STATUS2` decode; added `com1_online` / `radio_online` activity tracking (`v0.8.2`) for `VBAT` 5V boost field runs.
  * **Shared `I2C0` External `GX-12` Cable Stalls (`test/_053`, `v0.9.0`):** `977` `err=0x103` bus stalls over `2h 40m` (~6/min) traced to external `GX-12` cable capacitance on shared `I2C0`. Resolved cleanly by isolating `SEN0696` on freed `I2C1` (`GPIO 47/48`).
  * **`CAL MAG` 2D/3D Z-Bias Collapse & Warm-Gyro Drift (`v0.9.1 - v0.9.2`):** Flat-pole spins underconstrained 9-param Z bias (driving `-30..-155 uT` Z offsets); resolved via covariance-regularized `3D -> 4-param` sphere fallback with online `BZ_ref` self-heal (`v0.9.1`) and a 50-sample stationary accel-variance gyro FOC gate (`v0.9.2`).
  * **Core 1 `lvgl_mux` Starvation Blocking Screenshots (`test/_056`, `v0.10.3 - v0.10.4`):** Priority-3 `LVGL` and Priority-2 `RoverUI` continuously alternated `lvgl_mux`, starving Priority-1 `loopTask`'s `rover_lvgl_lock(2000)` on `BOOT` long-press. Resolved via a lock-free one-shot PSRAM framebuffer capture handshake (`s_fb_capture_dst` / `s_fb_capture_done_sem`) serviced directly inside `lvgl_port_core.c`, paired with a `2`-row (`1,920 B`) internal DMA write chunk buffer, `FFat` fallback, and mandatory post-`close()` size verification.

---

## 5. The Ugly: Silicon Quirks & Physical Compromises

1. **`E220` `AUX` Two-Regime Early Rise & Mode 3 Baud Order:** `AUX` stays `HIGH` during `<= 200 B` transmissions and rises `100-147 ms` *before* Subpacket 2 completes at `<= 9.6 kbps`; Mode 3 register config must restore `M0/M1=0` *before* switching ESP32 UART back from `9600` to `115200` baud to avoid `0xC1` echo corruption (`v0.8.7`). Both Base and Rover enforce silicon-calibrated piecewise `ToA + guard` formulas.
2. **E220 `+14 dB` RX LNA Front-End Saturation:** Bench testing at `30 dBm` within close room proximity (`> -35 dBm` RX) saturates the E220 RX LNA with false CRC drops. Standard policy separates Base/Rover antennas or runs `21 dBm` (`REG1=0x23`) for close indoor bench testing while preserving `30 dBm / 4.8k` (`REG1=0x20`) for field walks.
3. **Prebuilt Arduino-ESP32 `sdkconfig` PSRAM Thresholds (`4096 B` / `RESERVE_INTERNAL = 0`):** Every standard allocation `<= 4 KB` defaults to scarce internal `DRAM0` and `EXT_RAM_BSS_ATTR` is ineffective unless `heap_caps_malloc(..., MALLOC_CAP_SPIRAM)` or scoped `heap_caps_malloc_extmem_enable(64)` is explicitly applied.
4. **UPS-Powered I2C Slaves Holding `SDA` LOW Across Warm Resets:** Always-on `AXP2101` LiPo UPS power keeps internal `I2C0` slaves mid-transaction across ESP32 warm resets; defended permanently by `18`-clock open-drain `SCL` GPIO bus-clear prior to `i2c_new_master_bus()`.
5. **Internal SPI-Flash Erase vs. `115200` UART FIFO:** Internal `FFat`/`NVS` erases stall non-IRAM interrupts (`35-60 ms`); minimized via `non_index_settings_equal()` `FFat` skips and defended by `rx_stage` `CRC24Q` gatekeeping.

---

## 6. Quantitative Performance Verification

Field data collected across rigorous cadastral walks in dense New England timber (rural southern Maine, `test/_053` `v0.8.18`), Stage 1/2 E220 bench suites, and live `v0.10.7` / `CAD v2.2.4` audits:

| Performance Metric | Specification / Target | Measured Result | Source / Status |
| :--- | :--- | :--- | :--- |
| **Extended Woods Survey (`test/_053`, 2h 40m)** | Continuous canopy RTK | **`92.8% RTK_FIX` (`96.9% Fix+Float`), `2.7 cm` med `h_acc`, `15/15` pins `RTK_FIX`** | Field Proven (`9,109` epochs, `7,481 ft` walked to `1,319 ft`) |
| **RTK Fix Reliability (Open Sky)** | `> 95.0%` | **`100.0%` (`359/359` epochs)** | Field Proven |
| **Boundary Ground Truth Closure** | `< 3.0 in` / `100 ft` | **`1.9 in` (`4.8 cm`) misclosure (`0.78 mm` COGO parity across `37` corners)** | Field & COGO Proven |
| **Dual-Antenna Baseline & Heading** | `55.5 cm +/- 2.0 cm`, `< 0.4 deg` | **`55.4 cm` (`sigma = 1.82 cm`), `~0.2 deg` (`HDG FIX`), `< 8 mm` pole-spin orbit** | Field Proven |
| **Arch 2B LoRa RF Link (`4.8k/30dBm`)** | `> 98.0%` RTCM / Uplink | **`98.94%` (`14,772` verified RTCM3 frames), `100.0%` uplink PDR (`-75 dBm` med)** | Field & `stage2_bench_verify` (`7/7`) Proven |
| **Firmware Binary & Internal SRAM (`v0.10.7`)** | `0` warn, `DRAM0 <= 25%`, heap `>= 150 KB` | **`1,753,634 B` (`55%`) flash, `68,672 B` (`20%`) `DRAM0`, `~154-158 KB` internal free (`7.38 MB` PSRAM)** | Verified on `COM3` (`v0.10.7`) |
| **UI Touch, Audio & Screenshot Capture** | `< 50 ms` touch; lossless BMP | **`< 1 ms` audio / `0 ms` same-frame readout / `460,854 B` verified BMP (`FFat` fallback)** | Verified (`test/_057`) |
| **Mechanical CAD (`v2.2.4`)** | 100% support-free, `0` clashes | **`8/8` manifold STLs, `62/62` pairs `0.0000 mm^3` interference** | CadQuery + OpenSCAD Verified |
| **Outdoor `I2C1` Beam Compass Qual** | `< 2 deg` static, `0` `I2C0` stalls | Indoor `I2C1` + `FOC:OK` proven; outdoor 30-min soak & battery cutoff carry to Plan 2 `G1`/`G5` | Tracked in Plan 2 (`G1`, `G5`) |

---

## 7. Conclusion & Next Horizon

With **Plan 0 (`v0.8.19 -> v0.9.0`) and `v0.10.0..v0.10.7`** complete and archived, the standalone **Waveshare 3.5B + UM982 COM1 + E220-900T30S + SEN0696 `I2C1`** hardware/software architecture (`v0.10.7`, `CAD v2.2.4`, `e220_pacer.py v3.7.1`) is cleanly stabilized and field-ready. Development now advances along two sequenced roadmaps:
1. **[Plan 1 (`plan_1_shared_core_library_and_token_optimization.md`)](plans/plan_1_shared_core_library_and_token_optimization.md):** Split remaining `> 800 L` app files (`RTK_Rover.ino`, `e220_driver`, `um982_driver`, `waypoint_manager`), extract shared ESP-IDF/LVGL/sensor BSP layers into the standalone **`Field_Instrument_Core`** repository alongside `Magnetic_Field_Scanner`, and add host unit tests + `F-12`/`F-13`/`F-14` robustness guards.
2. **[Plan 2 (`plan_2_rover_enhancements_and_android_companion.md`)](plans/plan_2_rover_enhancements_and_android_companion.md):** Execute outdoor `G1`/`G5` verification gates, add survey quality guards (`F-03`), implement the `3.5B` NimBLE 5.0 NUS bridge + Core 0 Dual-Source RTCM3 Arbiter (with BLE/Wi-Fi mutual exclusion), and ship the **`RTK-Rover-Mobile`** Android companion app.
