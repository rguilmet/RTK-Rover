# RTK Rover - Project Overview

**Document Version:** `v2.10.0`  
**Last Updated:** November 22, 2026  
**Firmware Target:** `v0.10.0`  
**Python Ecosystem Target:** `e220_pacer.py v3.7.1`  
**License:** GNU General Public License v3.0 (`GPLv3`)  

This document serves as the master technical specification for the **RTK Rover** project. It details the mechanical setup, dual-antenna lever-arm kinematics, electrical pinouts, FreeRTOS + LVGL 9 firmware architecture, operational modes, and companion Python scripts.

---

## 1. Project Identity, Versioning & Documentation Rules
* **CRITICAL RULE:** All projects shall have a main Software version identifier as `v[Major].[Minor].[Patch]` and follow industry-standard SemVer rules and practices for bumping the version at EVERY file iteration.
* **Mandatory Per-Version Changelog & Git Sync:** Every version bump (`ROVER_FIRMWARE_VERSION` in `user_config.h` or `__version__` in Python scripts) must be recorded immediately under a `[vX.Y.Z]` entry in `CHANGELOG.md`, verified with `python scripts/check_secrets.py`, and committed + pushed to Git/GitHub (`origin/master`) so every version can be restored reproducibly from Git without relying on AI context.
* **Why:** We do not reuse version numbers for each iteration. This allows us to distinctly identify in the AI context and Git history the exact files we are working on. The firmware version string is defined as `ROVER_FIRMWARE_VERSION` in `user_config.h` and transmitted in the 48-byte LoRa uplink telemetry frame.
* **Pure ASCII Requirement:** All user-facing strings, labels, and terminal outputs strictly use standard ASCII characters. Extended Unicode symbols (such as $\pm$, $\circ$, arrows) are avoided in UI strings to guarantee compatibility with LVGL fonts and prevent missing glyph boxes.
* **Sibling Repository Architecture & Code/Data Decoupling:** The project strictly enforces the separation of computational logic from domain data. Public `RTK_Rover` contains open-source firmware, drivers, kinematics, and schema-driven mathematical tools. All private land records, deed PDFs, personal surveyed coordinates, raw sensor captures, and personal radio configurations reside in an independent sibling repository (`RTK_Rover_Private`) under local Git version control.

---

## 2. Bill of Materials (BOM) & Mechanical Architecture
1. **Display & Main Processing Unit:** **Waveshare ESP32-S3-Touch-LCD-3.5B**
   * `320x480` Portrait IPS QSPI Display (`AXS15231B` controller + Capacitive Touch `0x3B`).
   * Integrated `AXP2101` PMIC (`0x34`), `TCA9554` IO Expander (`0x20`), `PCF85063` RTC (`0x51`), `QMI8658` 6-axis IMU (`0x6B`), `ES8311` Audio Codec (`0x18`), and MicroSD slot.
   * Housed in a custom 3D-printed enclosure designed for handheld operation, bracket mounting at an angle, or vertical clipping to a survey pole.
2. **RTK GNSS Receiver:** **Holybro H-RTK Unicore UM982** (Dual-Antenna Multi-Band RTK GNSS)
   * Mounted on the survey pole T-bar with Master (Position) and Slave (Heading) antennas separated by `0.555 m` (`55.5 cm`, configurable via `antenna_spacing_m` in `settings.json`).
   * The survey pole centerline sits at the **exact midpoint** (`27.75 cm`) between Master and Slave antennas, at height `antenna_height_m` (default `1.800 m`) above the ground tip.
3. **RTCM Correction Radio:** **Ebyte E220-900T30S** (900MHz LoRa UART Transceiver)
   * Receives RTCM3 differential corrections streamed from the RTK Base Station in transparent mode at `115200` baud (forwarded by the ESP32 Core 0 bridge to `UM982 COM1` at `115200` baud via `s_com1_tx_mutex`).
   * **Radio Configuration (Verified on both Base and Rover, Profile Rev 2):**
     * **RF Channel (`e220_channel`):** `75` (`0x4B` $\rightarrow$ $850.125 + 75 = 925.125\text{ MHz}$, ranked `#1` cleanest US 902-928 MHz ISM channel in `test/e220-scan.csv` at `-96.23 dBm` avg, `+10.125 MHz` clear of `915.0 MHz` weather telemetry).
     * **UART Rate (`e220_uart_baud`):** `115200` bps, `8N1` (`REG0 = 0xE5` @ `19.2k` baseline; matching `UM982 COM1` baud rate).
     * **Air Data Rate (`e220_air_rate_kbps`):** `19.2` kbps default with synchronized 3-rate switching across `19.2k / 9.6k / 4.8k` (`REG0 = 0xE5 / 0xE4 / 0xE3`; also supports `2.4, 38.4, 62.5`).
     * **Packet Size (`e220_subpacket_bytes`):** `200 bytes` (`REG1` bits `[7:6] = 0b00`, utilizing native `400-byte` buffer atomic framing).
     * **Mode 0 Live RSSI Register (`REG1` Bit 5 = `0x20`):** Enabled (`REG1 = 0x23`), allowing on-demand `C0 C1 C2 C3 00 02` ambient noise floor and last-packet RSSI queries (dual 4-byte `C1 00 <noise> <pkt>` and 5-byte `C1 00 02 <noise> <pkt>` parser with `s_tx_air_busy_until_ms` RF TX guard, `180 ms` reply timeout, `1074`-verified post-terminal-MSM `inter_epoch_gap` at `> 5.0 kbps`, and Second 0 `post_uplink_gap`) without appending extra bytes to RX UART frames (`REG3 = 0x00`).
     * **Transmit Power (`e220_tx_power_dbm`):** Level 0 (`30 dBm` / `1 W` field & production default, `REG1 = 0x20`; `21 dBm` / `125 mW` for close bench tests, `REG1 = 0x23`). Note: keep Base and Rover antennas cleanly separated when operating at `30 dBm` to prevent `E220-900T30S` `+14 dB` RX LNA front-end saturation (`> -35 dBm`).
     * **Transmission Mode & LBT (`e220_lbt_enable`):** Transparent Mode, LBT Disabled, Appended RSSI Byte Disabled (`REG3 = 0x00`).
     * **Target EEPROM Profile:** `ADDH=0x00 ADDL=0x00 REG0=0xE5 REG1=0x20 CHAN=0x4B REG3=0x00`.
   * **Base Station Software & Architecture 2B Async Ping-Pong Mailbox Transport (`e220_pacer.py v3.7.0`):**
     * **RTKBase Stream Filtering:** RTKBase port `5016` is filtered via `str2str` to `"1006(5),1074,1084,1094,1230(10)"`, producing a lean, high-fidelity stream of ~625-650 Bytes/s (GPS MSM4, GLONASS MSM4, Galileo MSM4, Base ARP, and GLONASS biases). Empty MSM frames (`<= 28 B`) are dropped prior to TX.
     * **Asynchronous Ingress & Ping-Pong Latest-Value Mailboxes (`e220_pacer.py v3.7.0`):** A background ingress thread continuously drains TCP bytes, validates `CRC24Q`, and overwrites per-message-type mailboxes in place (zero epoch backlog buildup), with `2.5 s` dynamic MSM TTL expiration (`15.0 s` static metadata). The TX scheduler prioritizes static metadata (`1005/1006/1033/1230`), rotates MSM constellations via starvation-fair round robin (`now - last_tx` with GPS `1074` tie-breaker), and packs small frames (`<= 200 B`, e.g. `1006+1230` or `+1084`) into a single Regime A UART write.
     * **Two-Regime AUX + Stage-1-Calibrated Time-on-Air Pacing:**
       * Per E220-900T30S Manual Section 5.6.2 (and Stage 1 hardware benchmarks in [`docs/benchmarks/e220_stage1_20261116/`](benchmarks/e220_stage1_20261116/README.md) / [`docs/e220-reference.md`](e220-reference.md) v3.1), `AUX=1` indicates the 400-byte UART buffer emptied into the LLCC68 transceiver, **not** that over-the-air RF transmission has completed. Both Base and Rover use empirical silicon `ToA` formulas (`4.8k: 70+1.87*N ms`, `9.6k: 45+1.19*N ms`, `19.2k: 29+0.80*N ms`) + `30 ms` (`>=9.6k`) / `35 ms` (`<=5k`) guard delays.
       * **Single-Subpacket Frames (`<= 200B`):** Transfer straight into an idle LLCC68 FIFO without holding `AUX` LOW; paced deterministically by calibrated LoRa `ToA + guard`.
       * **Two-Subpacket Frames (`201..400B`):** Hold `AUX` LOW while Subpacket 1 (`200B`) transmits on air (can rise `100-147 ms` early at `<= 9.6 kbps`); paced by waiting for `AUX` rising edge plus `max(ToA(tail), total_ToA - elapsed_since_flush) + guard`.
     * **Content-Independent Consolidated Macro-Cycle & Mode 3 / Mode 0 Hardening:** Anchored to `UTC_Second % macro_cycle_s` (`--macro-cycle-s`, default `5`, optional `10`): Seconds `1..(N-1)` stay `100%` unbroken downlink (zero end-of-second RSSI pauses), while `Second 0` enforces aligned downlink cutoffs (`450 / 620 / 680 ms`) before Rover's uplink slot (`500..660 / 670..760 / 730..810 ms`) and a single post-uplink `C0 C1 C2 C3 00 02` Mode 0 RSSI query (`680..920 ms`, accepting cold-boot `pkt_raw == 0x00`). Synchronous 3-rate switches (`19.2k / 9.6k / 4.8k`) transmit 4 spaced `ACK_SWITCH` frames in `Second 1` (`80/260/440/620 ms`), reconfigure both E220 modules in Mode 3 at `Second 2` (exiting Mode 3 `M0/M1=0` BEFORE restoring baud + `5 ms` Mode 0 liveness check), and arm a 60-second pre-switch fallback rollback failsafe.
     * Includes `GPIO 22` hardware reset watchdog if `AUX` locks LOW `> 1.5 s`, `TIOCEXCL` exclusive serial port locking, source `CRC24Q` verification, and startup Channel 75 ambient RF noise floor probing.
     * **VBAT 5V Boost Power Architecture & Passive-XTAL Link Budget:**
       * A high-side switched **`MT3608`** `VBAT` 5V boost circuit (`NDP6020P` + NPN driver) powers both the `UM982` (`~320–350 mA` @ `5.0 V`) and `E220` (`~25–40 mA` avg, `610–850 mA` TX peaks) on battery, with `com1_online` and `radio_online` live-activity detection independent of USB (`VBUS`) presence. Combined with the `3.5B` display/controller (`~210 mA` @ `3.7 V`, `80%` backlight, `Wi-Fi OFF`), typical system draw is **`~806 mA` (`~3.0 W`) @ `3.7 V`**, yielding **~3.2–3.4 hrs** on a **`3,000 mAh`** pack and **~10.6–11.2 hrs** on a **`10,000 mAh`** pack (`85–90%` usable capacity; see [`docs/schematics/system_wiring_schematics.md`](schematics/system_wiring_schematics.md#3-system-power-budget-mt3608-boost-analysis--battery-runtime-estimates)). `19.2 kbps` provides `~450 ms` idle margin per 1 Hz epoch for full 4-constellation RTK, while `9.6 kbps` (`+3 dB`) and `4.8 kbps` (`+5 dB`) extend range under heavy canopy.

---

## 3. Dual-Antenna Lever-Arm Kinematics & IMU Decoupling

### A. Master-to-Pole-Center & Ground Tip Projection (`nav_engine`)
The UM982 reports the WGS84 coordinates (`GPGGA`) of the **Master Antenna** phase center, and outputs the true azimuth ($\psi_{\text{baseline}}$) and pitch ($\theta_{\text{baseline}}$) vector from Master to Slave (`UNIHEADINGA`).
Because the pole centerline is located at the midpoint between the two antennas ($L = \frac{\text{antenna\_spacing\_m}}{2} = 0.2775\text{ m}$), `nav_compute_pole_tip_position()` projects the Master coordinates along the 3D baseline vector and subtracts `antenna_height_m`:
$$\Delta N = \frac{B}{2}\cos(\theta_{\text{baseline}})\cos(\psi_{\text{baseline}}), \quad \Delta E = \frac{B}{2}\cos(\theta_{\text{baseline}})\sin(\psi_{\text{baseline}}), \quad \Delta U = \frac{B}{2}\sin(\theta_{\text{baseline}}) - H_{\text{pole}}$$
This prevents the displayed or logged ground-tip coordinates from orbiting in a $27.75\text{ cm}$ radius circle when the pole is rotated in place.

### B. Slave Antenna (ANT2) Coordinates & Geodetic Baseline Projection
Slave antenna coordinates are extracted from NMEA `$GNGGAH` / `$GPGGAH` sentences. If the secondary PVT has not solved independent lat/lon but carrier-phase dual-antenna heading/baseline is valid, ANT2 position is accurately computed along the measured 3D baseline vector:
$$\Delta N = L \cdot \cos(\theta_{\text{pitch}}) \cdot \cos(\theta_{\text{hdg}}), \quad \Delta E = L \cdot \cos(\theta_{\text{pitch}}) \cdot \sin(\theta_{\text{hdg}})$$
These coordinates are displayed live under `[ANT2 SLAVE]` on Tile 4.

### C. Unicore Extended Base Station Telemetry (`BASEINFOA`)
* **Autonomous RTCM Decoding:** The UM982 receiver autonomously parses incoming RTCM 1005/1006 reference frames from the Base Station received via LoRa on COM2.
* **`BASEINFOA COM1 1` Stream:** Configured to push 1 Hz updates of `#BASEINFOA` reporting Base Station ID, health status, and ECEF coordinates $(X, Y, Z)$.
* **Closed-Form Geodetic Conversion:** The ESP32 driver converts base ECEF coordinates into WGS84 Latitude, Longitude, and Height via closed-form Bowring equations in $O(1)$ time with sub-millimeter precision.
* **Dedicated Observer Card:** Tile 4 displays Base ID, differential correction age, exact WGS84 coordinates, altitude, ECEF vectors, and RTCM 1005 / MSM message counters.

### E. Deep-Woods Heading Fallback, `SEN0696` Beam Fusion & `CAL MAG`
In dense forest canopy where secondary (Slave) antenna carrier tracking drops, the firmware executes a zero-lag multi-tier heading hierarchy (`nav_heading_beam_update()` + `ahrs_9dof` + `matrix_math`):
* **Tier 1 (`HDG_SRC_DUAL_FIX` / `DUAL_FLT`):** Carrier-phase moving-baseline heading (`0.2 deg` precision) whenever `UM982` resolves `ANT2`, continuously teaching the magnetic True-North declination tare, reference field magnitude `B_ref`, reference vertical component `Bz_ref`, and crossbar pitch tare (`atan2` circular averaging) with an immediate `0.0 s` switchover (`s_last_disp_ms = 0`) when `SLV` drops out.
* **Tier 2A (`SEN0696` Beam 9-DOF AHRS -> `HDG_SRC_COMPASS`, `[CMP]` Teal `#14B8A6`):** Isolated onto `I2C1` (`GPIO 47/48` at `400 kHz`, automatic `I2C0` bench fallback), polling `BMI323 + BMM350` at `25-30 Hz` (`~3 Hz BMP581` baro) through an instance-safe Madgwick 9-DOF filter (`src/sensor_fusion/ahrs_9dof.{h,cpp}`) with `beam_imu_orient_mode` (default `4` = Face-Down `(+X,-Y,-Z)`), 50-sample stationary accel-variance gyro FOC, `360-deg` PSRAM ellipsoid `CAL MAG` calibration (`matrix_math.h` 9-param fit w/ 4-param sphere fallback), and a triple-condition iron anomaly gatekeeper (`|B|`, `Bz`, 200 ms gyro-mag innovation + 500 ms recovery hysteresis). Onboard `QMI8658` remains dedicated to enclosure `DisplayPose_t` whenever `SEN0696` is active.
* **Tier 2B (Fallback Legacy `MMC5983MA 0x30` on `I2C0`):** Retained as Tier-B fallback with `SET`/`RESET` degaussing coil bridge offset calibration if `SEN0696` is absent.
* **Tier 3/4 (`HDG_WALK`, `HDG_GYRO`, `HDG_HOLD`):** Kinematic displacement (`>= 0.70 m`, `WLK` Cyan), bias-compensated gyro coasting during iron anomalies or pivots (`GYR` Blue), and stationary zero-drift hold (`HLD` Slate).

---

## 4. Electrical Pinouts & Hardware Mapping (`ESP32-S3-Touch-LCD-3.5B`)

> **PIN NAMING CONVENTION:**
> All peripheral pin labels are **device-relative**:
> * `<DEV>_TXD` is the **Transmit Output from the external module** $\rightarrow$ wired to the **ESP32 UART RX Input**.
> * `<DEV>_RXD` is the **Receive Input to the external module** $\leftarrow$ wired from the **ESP32 UART TX Output**.

| Subsystem | Signal Label | ESP32-S3 GPIO | ESP32 Direction | Function / Bus Assignment |
| :--- | :--- | :--- | :--- | :--- |
| **UM982 COM1** | `UM982_TXD0` | `GPIO 17` | **ESP32 RX** (`Serial1`) | UM982 COM1 TX -> ESP32 RX (`115200 8N1`, NMEA, Heading & Responses) |
| **UM982 COM1** | `UM982_RXD0` | `GPIO 18` | **ESP32 TX** (`Serial1`) | ESP32 TX -> UM982 COM1 RX (`115200 8N1`, RTCM3 & Config commands) |
| **Beam 10DOF `I2C1`** | `10DOF_SDA` | `GPIO 47` (`P21`) | **I2C1 Open-Drain** | Isolated `I2C_NUM_1` SDA (`400 kHz`, Gravity Green `D/T` -> `SEN0696 0x4A..0x4D` + `4.7 kOhm` pull-up; freed from legacy COM2) |
| **Beam 10DOF `I2C1`** | `10DOF_SCL` | `GPIO 48` (`P23`) | **I2C1 Clock** | Isolated `I2C_NUM_1` SCL (`400 kHz`, Gravity Blue `C/R` -> `SEN0696` + `4.7 kOhm` pull-up; auto `I2C0` bench fallback) |
| **E220 LoRa** | `E220_TXD` | `GPIO 41` | **ESP32 RX** (`Serial0`) | E220 TXD -> ESP32 RX (`115200 8N1` Transparent / `9600` Config) |
| **E220 LoRa** | `E220_RXD` | `GPIO 42` | **ESP32 TX** (`Serial0`) | ESP32 TX -> E220 RXD |
| **E220 Control** | `E220_AUX` | `GPIO 21` | **ESP32 Input** | Busy (`LOW`) / Idle & Ready (`HIGH`) status |
| **E220 Control** | `E220_RST` | `GPIO 38` | **ESP32 Output** | Hardware Reset (`HIGH` = normal operation) |
| **E220 Control** | `E220_M0` | `GPIO 39` | **ESP32 Output** | Mode 0 (`LOW` = Transparent, `HIGH` = Config) |
| **E220 Control** | `E220_M1` | `GPIO 40` | **ESP32 Output** | Mode 1 (`LOW` = Transparent, `HIGH` = Config) |
| **Internal I2C0** | `ESP_SCL/SDA` | `GPIO 7 / 8` | **I2C Master** | Shared by Touch (`0x3B`), TCA9554 (`0x20`), AXP2101 (`0x34`), RTC (`0x51`), IMU (`0x6B`), ES8311 (`0x18`), optional `MMC5983MA` (`0x30`) at `400 kHz` on Core 1 |
| **LCD QSPI** | `CS/CLK/D0..3` | `12, 5, 1..4` | **QSPI Master** | `AXS15231B` `320x480` Portrait Display (`GPIO 6` Backlight PWM) |
| **SD Card** | `D0/CMD/CLK` | `9, 10, 11` | **1-Bit SD_MMC / SPI** | 3-stage mount fallback (`SD_MMC` 1-bit `EXIO3=High-Z` $\rightarrow$ `SD_MMC` `EXIO3=LOW` $\rightarrow$ `SD` SPI) |
| **BOOT Button** | `BOOT_SWITCH` | `GPIO 0` | **ESP32 Input** | **Long-Press ($\ge 1.5\text{ s}$):** Capture 24-bit uncompressed BMP screenshot |

---

## 5. FreeRTOS, Core Libraries & LVGL 9 Architecture

### Core Separation
* **Core 0 (`task_rover_core0` / `"RoverCore0"`, Priority 4 -- Zero I2C Access):**
  * Polls `E220` (`Serial0`), logs raw wire-tap bytes to `rtcm-base_*.txt`, stages incoming bytes in a 1,536-byte **Sliding-Window Store-and-Verify RTCM3 Gatekeeper** (`0xD3` + `CRC24Q == 0` with rate-adaptive truncation timeouts `280 / 380 / 480 ms`), and forwards **only verified intact RTCM3 frames** atomically to `UM982 COM1` (`Serial1` via `s_com1_tx_mutex`) while tracking `IO21` `AUX` falling-edge interrupts, consolidated `Second 0` uplink (`500..660 / 670..760 / 730..810 ms`) + Mode 0 RSSI polling (`680..920 ms`), and cold-boot `pkt_raw == 0x00` handling.
  * Polls `UM982 COM1` (`Serial1`) through the 3-way line demultiplexer to parse `$GPGGA` (2 Hz), `$GPRMC` (1 Hz), `$GPGST` (2 Hz), `$GPGSV` (1 Hz), `$GNGGAH` (1 Hz slave data), `#BASEINFOA` (1 Hz base station data), `#UNIHEADINGA` (2 Hz), and `#VERSIONA`/`#MODEA`/`#CONFIGA`/`#LOGLISTA` command responses.
  * Runs 30Hz lever-arm kinematic projection and geodesic target guidance (`nav_engine`), waking within `<= 2 ms` via `g_waypoint_change_seq` whenever the operator selects a new waypoint or file.
* **Core 1 (`task_audio_beep` Priority 4, `rover_lvgl_port_task` Priority 3, `task_display_and_audio` Priority 2, `loopTask` Priority 1 -- Exclusive I2C0 & I2C1 Owner):**
  * **`task_audio_beep` (`"RoverAudio"`, Priority 4):** Dedicated real-time audio synthesizer blocking on `beepQueue` (`portMAX_DELAY`) and driving the ES8311 I2S codec within `< 1 ms` of touch contact (supports dual-tone L/R chirps and clean PCM silence gaps), completely decoupled from UI rendering and filesystem writes.
  * **`rover_lvgl_port_task` (Priority 3):** Renders the modular 6-tile `320x480` Portrait LVGL 9 interface (`src/lvgl_port/`, 11 files `<= 764 L` each) protected by `rover_lvgl_lock()` / `rover_lvgl_unlock()`, executing `0 ms` same-frame target guidance readout updates (`refresh_t0_guidance_readout_immediately()`) inside button callbacks.
  * **`task_display_and_audio` (`"RoverUI"`, Priority 2):** Exclusively owns both `I2C_NUM_0` (`GPIO 7/8`) and `I2C_NUM_1` (`GPIO 47/48`) (`400 kHz`, `.scl_wait_us = 20000`, independent `s_ports[2]` mutexes, 18-clock GPIO SCL warm-reset bus-clear, and automatic `i2c_master_bus_reset()` recovery): polls `AXS15231B` touch (`30 Hz`), `SEN0696` (`25-30 Hz` fast IMU+Mag / `~3 Hz` `BMP581`), `QMI8658` display pose (`10-50 Hz`), `PCF85063` RTC (`1 Hz`), and `AXP2101` power (`0.5 Hz`). Synthesizes proximity Geiger/Beep and L/R line-follow chirps, manages `CAL MAG` PSRAM capture + safe Auto-Off countdown/logger flush, and saves hardware BMP screenshots on `GPIO 0` long-press.
  * **`loopTask` (Priority 1):** Executes 5-second quiet-debounced `save_settings()` with smart `FFat` mirror skip (`non_index_settings_equal()`, avoiding 1.5s internal flash erases when stepping `active_waypoint_index` with MicroSD mounted), services the HTTP web portal (`handle_wifi_server()`), and flushes PSRAM ring buffers (`serial_logger`, `nmea_logger`, `rtcm_logger`) to MicroSD with task-scoped `s_io_owner_task` re-entrancy protection.
* **Portable Zero-App-Dependency Math & Fusion Modules (Seeded for Plan 1 `Field_Instrument_Core`):**
  * [`src/matrix_math/matrix_math.h`](../src/matrix_math/matrix_math.h) (`745 L`): Linear solvers, `3x3` Jacobi eigensolver, 9-param ellipsoid + 4-param flat-spin sphere fit w/ RMS validation, and `kabsch_align_3x3_f32`.
  * [`src/sensor_fusion/ahrs_9dof.{h,cpp}`](../src/sensor_fusion/ahrs_9dof.cpp) (`116 / 513 L`): Instance-safe `Ahrs9DofState_t`, 8 right-handed axis orientation modes, stationary accel-variance gyro FOC, and Madgwick 9-DOF fusion.

### Memory Partitioning: Internal `DRAM0` vs. 8 MB Octal PSRAM (`v0.7.5` - `v0.7.7`)
Because prebuilt Arduino-ESP32 (`v3.3.x`, `qio_opi`) compiles with `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL = 4096` and `CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL = 0`, explicit memory-tier partitioning is enforced so `esp_wifi_init()` / `esp_wifi_start()` (and future BLE NUS stacks) always retain $\ge 150\text{ KB}$ of contiguous internal 8-bit/DMA SRAM:
* **8 MB Octal PSRAM (`MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT`):**
  * Dual full-frame LVGL 9 RGB565 framebuffers (`2 x 307,200 B = 614.4 KB`).
  * `serial_logger` (`32,768 B`), `rtcm_logger` (`16,384 B`), and `nmea_logger` (`8,192 B`) ring buffers (`57.3 KB` total).
  * `g_waypoints` survey table (`MAX_WAYPOINTS = 80`, `25,600 B`).
  * ~400+ LVGL 9 UI widget, style, table, and canvas objects (`~50 KB`), routed via scoped `heap_caps_malloc_extmem_enable(64)` strictly around `create_rover_ui()` (immediately restored to `4096` afterward).
* **Internal `DRAM0` / `MALLOC_CAP_DMA` SRAM (Static `.dram0.bss = 37,120 B` / `20%` global dynamic RAM):**
  * 24-line QSPI LCD DMA bounce strip (`LVGL_DMA_BUFF_LEN = 15,360 B`, exact 20-strip divisor of 480 px).
  * `SD_MMC` IDMAC descriptors, `FFat` sector buffers, `ES8311` I2S DMA buffers, FreeRTOS task stacks, and the ESP32-S3 Wi-Fi MAC/PHY + LwIP RX/TX DMA pools.

---

## 6. UI Tile Structure (`320x480` Portrait)

### Persistent Top Header Bar (`320x74`)
* **Row 1:** Local Date (`YYYY-MM-DD`), Local Time (`HH:MM:SS TZ`), Units badge (`SI` / `IMP`), Multi-state Wi-Fi icon (Cyan STA, Violet AP, Amber CONN, Slate OFF), and Minimalist Power Widget (`[USB] 5.04V` / `[Bat] 85% 3.92V`).
* **Row 2 (Master Antenna ANT1):** Fix badge (`RTK FIX`, `RTK FLT`, `DGPS`, `SINGLE`, `OFF`), Satellites (`SV: used/view`), Precision (`+/- h_acc`), and RTCM differential age (`RTCM: X.Xs`).
* **Row 3 (Slave Antenna ANT2):** Heading Fix badge (`HDG FIX`, `HDG FLT`, `OFF`), Slave satellites (`SV: X`), Baseline length (`BL: 55.5cm`), and True Heading (`Hdg: XXX.X`).
* **Row 4:** Ground-tip Latitude, Longitude, and Ellipsoidal Height in 14pt Montserrat font (`0.00000000, 0.00000000, 0.000 m`).

### UI 6-Tile Symmetric Structure (`320x480` Portrait)
Symmetric ring navigation wraps seamlessly around **Tile 0 (Navigation)** (`< Prev` -> `Tile 5 Quick Settings`, `Next >` -> `Tile 1 Survey`): `[Tile 5: Settings] <-> [Tile 0: Nav] <-> [Tile 1: Survey] <-> [Tile 2: Waypoints] <-> [Tile 3: Compass] <-> [Tile 4: Diag]`.

* **Tile 0 (Navigation) -- 2-Mode Unified Log-Scale Guidance Triangle & Persistent Map (`v0.10.0`):**
  * Consolidates Tile 0 into **`[GUIDANCE]`** <-> **`[MAP VIEW]`**. Guidance (`ui_tile0_views.c`) supports **`A: DEED`** (CSV `idx_a -> idx_b` w/ `[SWAP]`) and **`A: HERE`** direct-to GNSS anchor (`[SET A]`, auto on 1-point CSVs), plotting Start `A` (`12 px`), Target `B` (`16 px`), and Rover `R` (`14 px`) on a symmetric log cross-track scale (`line_log_max_m = 50.0 m`) with `18 px` overshoot aprons, perpendicular tie `R -> Foot`, hypotenuse `R -> B`, `R -> B`-prioritized barbed heading arrow (`<= 4 deg` Green / `<= 15 deg` Orange / White lighting up the locked leg), and acoustic fade-out `L/R` chirps (`25% -> 100%` gain, silent beyond `line_audio_fade_m = 20.0 m`). Enters `BULLSEYE` via `< 4.8 m / > 5.3 m` hysteresis with `A -> B` chord overlay. **Map View (`ui_tile0_map.c`)** persists manual zoom (`map_zoom_m_per_px` in `settings.json` Schema v2) and highlights the active followed segment (`4 px` Emerald `DEED` or `3 px` dashed Violet `HERE`).
* **Tile 1 (Survey) -- Dedicated Field Survey Logger:**
  * `40 px` Prefix + Averaging (`1s/5s/10s/30s`) & all `12` field monument `kNoteTags` (`kNoteTagCount`), `52 px` `[LOG]` button echoing the active tag, live Pre-Log Survey Quality card (`Fix/SV/HDOP/H/V-Acc/Tilt/dN/dE`), Core 1 `pole_tilt_deg` snapshot capture, and circular `atan2` heading averaging.
* **Tile 2 (Waypoints) -- Multi-File CSV Browser:** Instant `<25 ms` RAM stem-cached file/waypoint selector with `poly_order` boundary/tie rendering (`[SWAP]` auto-clear on target step).
* **Tile 3 (Compass) -- Dual-Antenna + Beam 9-DOF Compass, Plumb Bubble & `CAL MAG`:** `224 px` Course-Up bubble level, `TARE`, left-side `CAL MAG` 360-deg PSRAM spin wizard, heading provenance badges (`DUAL FIX/FLT`, `BEAM 9DOF`, `WALK`, `GYRO COAST`, `HOLD`), `[IRON ANOMALY]` warning, `[BEAM 10DOF I2C1]` status, and `Baro Rel Elev`.
* **Tile 4 (Diag) -- Dual-Bus `I2C0/I2C1` & Subsystem Observers:** Fixed header (`RTK ROVER v0.9.0`), LoRa RF survey card, Storage, `ANT1/ANT2`, Base Station, `SEN0696` (`Orient`, `FOC`, `3D-Cal`, `BMP581`) + `QMI8658` pose, dual `I2C0/I2C1` error counters, Auto-Off countdown status, UM982 COM1, and E220 self-test.
* **Tile 5 (Settings) -- Dedicated `5x3` (`94 x 44 px`) Quick Settings & Status Footer:** `VOL/AUD/BRI`, `SERIAL/NMEA/RTCM`, `TELEM/IMU/TOUCH`, `WIFI/UNITS/LORA`, `AUTO-OFF` (`5m/15m/30m/OFF`), `CAL MAG` (`CAL MAG: OK` green when calibrated), dimmed `BLE: PLAN2` slot, and PSRAM-routed `Wi-Fi` / `Auto-Off` / `LoRa` footer. Plus motion/REC-safe Auto-Off `lv_layer_top()` 30s countdown modal & clean SD logger flush, and `settings.json` `ntrip_*` + `mag_*` persistence (`Plan 0 COMPLETE`).

---

## 7. Python Survey & Ingestion Scripts (`scripts/`)

* **`e220_pacer.py` (`v3.7.0`):** Architecture 2B Asynchronous Ping-Pong Mailbox LoRa driver for RTKBase (`scripts/e220-pacer.service`, `scripts/update_e220_pacer.sh`). Uses a background ingress thread with per-type latest-value mailboxes (`2.5 s` dynamic / `15.0 s` static TTL), starvation-fair round robin anchored to `1074`, smart `<= 200 B` coalescing, Stage-1-calibrated piecewise `ToA` + `30/35 ms` guard pacing, content-independent consolidated `Second 0` pause (`--macro-cycle-s`, default `5`), empty MSM filtering (`<= 28 B`), `GPIO 22` watchdog, `TIOCEXCL` locking, Mode 0 ambient noise probing, V3 `48 B` telemetry decoding + atomic RAM JSON / rolling daily CSV logging, and 4-ACK synchronized 3-rate air switching (`19.2k / 9.6k / 4.8k`).
* **`stage2_bench_verify.py` (`v1.0.0`):** Standalone Architecture 2B verification suite providing `--self-test` (offline virtual-clock & live-thread simulation across `avg`/`maxsat`/`extreme` profiles), `--mode pi-feeder` (deterministic `1 Hz` TCP RTCM3 generator on `127.0.0.1:5017`), and `--mode rover-monitor` (ESP32 USB-CDC scorecard).
* **`bench_e220_multirate.py` (`v1.0.3`) & `set_e220_power.py` (`v1.1.0`):** Stage 1 hardware-in-the-loop multi-rate silicon characterization suite (archived in [`docs/benchmarks/e220_stage1_20261116/`](benchmarks/e220_stage1_20261116/README.md)) and persistent runtime TX power synchronizer (`/etc/default/e220-pacer`).
* **`cogo_solver.py` (v1.1.0):** Schema-driven Coordinate Geometry (COGO) traverse solver. Ingests standard parcel JSON schemas ([`docs/schemas/cadastral_schema.json`](schemas/cadastral_schema.json)) and control anchor CSV files, performs forward geodesic projections, verifies loop closure precision, applies Bowditch / Compass Rule adjustments, solves circular road curve geometry, reconciles common boundaries, and exports to Google Earth KML, Rover `/waypoints.csv`, GeoJSON, AutoCAD DXF (with layer organization), and text closure audits. Detailed documentation in [`docs/cogo_solver_guide.md`](cogo_solver_guide.md).
* **`check_secrets.py` (v1.1.1):** Pre-push security verification scanner. Scans Git tracked files and unpushed commits for generic credentials, private keys, and local absolute filesystem paths, plus personal signatures defined in an unversioned `.secrets.local` configuration.
* **`install_git_hooks.py` (v1.1.0):** Automatically installs the pre-push security verification hook into `.git/hooks/pre-push`.
* **`kml_to_waypoints.py` (v1.0.0):** Ingests GIS or Google Earth KML files and extracts placemarks, linestrings, and polygon boundaries into the standardized 19-column rover CSV format.
* **`validate_waypoints.py` (v1.2.0):** Validates CSV schema compliance, bounds, and computes traverse leg distances, bearings, and total closed perimeter.
