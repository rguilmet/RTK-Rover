# Plan Phase 8: Single-UART UM982 (`COM1`) Consolidation & Auto-Detected `MMC5983MA` 9-DOF Compass Integration

**Document Version:** `1.0.0`  
**Date:** November 7, 2026  
**Firmware Baseline:** `v0.7.7` $\rightarrow$ Target **`v0.8.0`**  
**Target Hardware:** Current Prototype (**Waveshare ESP32-S3-Touch-LCD-3.5B** + **Holybro UM982** + **Ebyte E220-900T30S** + **SparkFun Qwiic `MMC5983MA` (`SEN-19921`, `0x30`)**)  
**Purpose:** De-risk and field-verify Single-UART `UM982 COM1` communication (NMEA/Heading RX + RTCM3 TX on one UART) and auto-detected `MMC5983MA` 9-DOF Dual-GNSS-calibrated compass fusion on our current `3.5B` prototype **before** migrating to `Field_Instrument_Core` and the `3.49-V2` Smart Pole.

---

## 1. Executive Summary & Why This Intermediate Step Makes Sense

Before splitting the codebase into `Field_Instrument_Core` and building the new `3.49-V2` Smart-Pole hardware, validating these two hardware/firmware changes on our working `v0.7.7` (`3.5B`) prototype gives us a rock-solid baseline:

1. **Single-UART `UM982 COM1` Consolidation (Testable Immediately Today):**
   * On the Unicore Nebulas IV (`UM982`), `COM1` is a full-duplex hardware UART whose receiver state machine natively auto-detects and accepts both **ASCII configuration commands** and **binary `RTCM3.x` (`0xD3` sync + `CRC24Q`) differential frames** on `COM1 RX` while simultaneously streaming high-rate `NMEA` + `UNIHEADINGA` + `BASEINFOA` out of `COM1 TX`.
   * Consolidating `RTCM3` injection onto `COM1` (`Serial1`, `GPIO 17/18` @ `115200` baud) completely eliminates `Serial2` (`COM2`, `GPIO 47/48`), **removes 2 wires from the UM982 harness**, frees **`GPIO 47` and `GPIO 48`**, and reclaims **$\sim 6\text{ KB}$ of internal `DRAM0` SRAM** (UART2 RX/TX ring buffers and driver state).
2. **Boot-Time Auto-Detection & Integration of the `MMC5983MA` 3-Axis Compass (Plug-and-Play When Sensor Arrives Next Week):**
   * By probing I2C address **`0x30`** (`Product ID 1` register `0x2F == 0x30`) on the existing thread-safe `400 kHz` `i2c_bsp` bus (`SCL = GPIO 7`, `SDA = GPIO 8`) at boot:
     * **Before the module arrives (Today):** Firmware detects no device at `0x30`, logs `[MAG] MMC5983MA (0x30) not present — using 6-DOF IMU fallback`, and runs with zero regressions.
     * **The moment you plug in the `MMC5983MA` (Next Week):** Firmware automatically detects `0x30`, executes hardware **`SET/RESET` degaussing pulses** to cancel bridge offset and temperature drift, and activates the **`HDG COMP` 9-DOF AHRS** (continuously auto-calibrated to True North whenever `UM982` `HDG FIX` is active).

```mermaid
flowchart LR
    subgraph Before["Current v0.7.7 (Dual UART + 6-DOF IMU)"]
        E1["ESP32-S3 (3.5B)"] <-->|Serial1: IO17/18\nNMEA RX + Cmd TX| U1_C1["UM982 COM1"]
        E1 -->|Serial2: IO47/48\nRTCM3 TX Only| U1_C2["UM982 COM2"]
        Q1["Onboard QMI8658 (0x6B)"] -->|I2C0 IO7/8| E1
    end

    subgraph After["Target v0.8.0 (Single UART COM1 + Auto-Detected 9-DOF Compass)"]
        E2["ESP32-S3 (3.5B)\n• Serial2 Disabled (+6KB DRAM0)\n• IO47 & IO48 Freed!"] <==>|Serial1: IO17/18 @ 115200\nFull-Duplex: NMEA/Hdg RX\n+ Boot Cmds & RTCM3 TX| U2_C1["UM982 COM1\n(COM2 Disconnected)"]
        Q2["Onboard QMI8658 (0x6B)\n6-DOF Accel + Gyro"] -->|I2C0 IO7/8 (400kHz)| E2
        M2["Outboard MMC5983MA (0x30)\nAuto-Detected at Boot\nSET/RESET Degaussing"] -.->|I2C0 IO7/8 (400kHz)\non T-Bar Crossbar| E2
    end
```

---

## 2. Part 1: Consolidating `UM982` Communications onto Single-UART `COM1`

### 2.1 Protocol & Bandwidth Analysis on `Serial1` (`115200` Baud Full-Duplex)
Because UART `TX` (`ESP32 GPIO 18 -> UM982 COM1 RX`) and `RX` (`UM982 COM1 TX -> ESP32 GPIO 17`) are electrically independent full-duplex lines at `115200` baud (`11,520 Bytes/sec` per direction):

| Direction on `UM982 COM1` | Signals Carried | Peak Throughput | `%` of `115200` Capacity | Collision Risk |
| :--- | :--- | :---: | :---: | :--- |
| **`UM982 COM1 TX` $\to$ `ESP32 RX1` (`GPIO 17`)** | `GPGGA` (2Hz), `GPRMC` (1Hz), `GPGST` (2Hz), `GPGSV` (1Hz), `UNIHEADINGA` (2Hz), `GNGGAH` (1Hz), `BASEINFOA` (1Hz) | `~1,850 B/s` | **`16.1%`** | **Zero** (Dedicated RX wire) |
| **`ESP32 TX1` (`GPIO 18`) $\to$ `UM982 COM1 RX`** | • **Boot (`setup()`):** ASCII setup commands (`MODE ROVER SURVEY`, etc.)<br>• **Runtime (`task_rover_core0`):** Verified `CRC24Q` RTCM3 frames (`1006, 1074, 1084, 1094, 1230`) | `~650 B/s` | **`5.6%`** | **Zero** (Boot commands finish before Core 0 RTCM task starts; guarded by `s_com1_tx_mutex`) |

### 2.2 Multiplexed Protocol Handling on `COM1` (Both Directions)

Consolidating onto a single UART means `COM1` carries mixed protocol traffic in both directions. Both the `UM982` receiver and our `ESP32-S3` driver must handle this deterministically without interleaving or false parsing:

#### A. `ESP32-S3 TX1` $\to$ `UM982 COM1 RX` (ASCII Commands + Binary `RTCM3.x` Frames)
1. **How `UM982` Hardware Handles Mixed Input:**
   * The Unicore Nebulas IV (`UM982`) UART input engine on `COM1` runs a concurrent protocol recognizer that automatically accepts both **ASCII commands** (`...\r\n`) and **binary `RTCM3.x` packets** (`0xD3` preamble + 10-bit length + payload + 24-bit `CRC24Q`) on the same port without switching modes.
2. **Preventing Mid-Frame Interleaving on `ESP32-S3` TX:**
   * While the `UM982` accepts both protocols on `COM1`, an ASCII command **must never be injected in the middle of a multi-byte `RTCM3` binary frame** (which would corrupt the `RTCM3` `CRC24Q` and cause the `UM982` to discard both the RTCM packet and the command).
   * **Safeguard 1 (Boot vs. Runtime Separation):** All boot setup commands (`VERSIONA`, `UNLOGALL`, `MODE ROVER SURVEY`, `CONFIG HEADING ...`, `GPGGA COM1 0.5`, `SAVECONFIG`) execute synchronously in `um982_driver_init()` during `setup()` **before** `task_rover_core0` starts polling the `E220` radio. A `s_com1_tx_boot_complete` gate blocks any RTCM forwarding until boot configuration finishes.
   * **Safeguard 2 (Atomic Frame Forwarding & TX Mutex):** During runtime, [`e220_driver.cpp`](../src/e220_driver/e220_driver.cpp) already stages each complete `RTCM3` frame (`25–220 B`) in `rx_stage[]` and verifies its 24-bit `CRC24Q` before calling `um982_forward_rtcm_to_com2(rx_stage, total_len)` in a single atomic write. By expanding `Serial1.setTxBufferSize(4096)` and guarding both `send_um982_com1_command()` and `um982_forward_rtcm_to_com2()` with a FreeRTOS mutex (`s_com1_tx_mutex`), a complete `RTCM3` frame and a complete `ASCII\r\n` command can **never** interleave byte-for-byte on `Serial1`.

#### B. `UM982 COM1 TX` $\to$ `ESP32-S3 RX1` (Mixed `NMEA`, `#UNIHEADINGA`/`#BASEINFOA` & Command Responses)
When `COM1` is already streaming `10–15 lines/sec` of `NMEA` (e.g., on a warm reboot after `SAVECONFIG`, or after periodic logs are enabled), command responses (`$command,CONFIG,response: OK*54`, `#VERSIONA,...`, `#MODE,...`, `#CONFIG,...`) arrive interleaved line-by-line with `$GNGGA`, `$GNRMC`, `$GNGSV`, `#UNIHEADINGA`, and `#BASEINFOA`. We must harden `um982_poll_com1()` and `process_com1_line()` across five specific areas:

1. **Strict 3-Way Line Demultiplexer in `process_com1_line()`:**
   * **Stream 1 — Standard NMEA 0183 (`$GNGGA`, `$GNGGAH`, `$GNRMC`, `$GNGST`, `$GNGSV`, `$GNHDT`, `$GNTHS`):**
     * Verify NMEA `*XX` XOR checksum before parsing.
     * Increment `port_stats.nmea_sentences_ok` and update `port_stats.last_com1_line`.
   * **Stream 2 — Unicore Periodic ASCII Telemetry (`#UNIHEADINGA`, `#BASEINFOA`):**
     * Parse dual-antenna heading/pitch/baseline and Base Station ECEF coordinates.
     * Increment `port_stats.heading_sentences_ok`.
   * **Stream 3 — Command & Query Responses (`$command,`, `#VERSIONA,`, `#MODE,`, `$MODE,`, `#CONFIG,`, `$CONFIG,`):**
     * Match using strict prefixes (`strncmp`) rather than loose `strstr` substrings.
     * Store the latest command ACK/response in `port_stats.last_com2_line` (repurposed as the **Dedicated Last Command Response / RTCM Status** buffer on Tile 4) so incoming `2 Hz` `$GNGGA` / `$GNGSV` sentences never overwrite the most recent command response!
     * Set specific volatile ACK flags (`version_ack_received`, `saveconfig_ack_received`, `mode_query_ack_received`) so `um982_run_powerup_selftest()` and `um982_configure_rover_defaults()` wait for the exact command ACK line even if pre-existing NMEA lines arrive in between.
2. **Prevent Command Responses from Polluting `.nmea` SD/FFat Log Files:**
   * Currently in [`um982_driver.cpp`](../src/um982_driver/um982_driver.cpp), `nmea_logger_feed()` is called unconditionally on every `COM1` line before classification, which writes `$command,...` and `#CONFIG,...` responses into `.nmea` files.
   * We will move `nmea_logger_feed()` inside the Stream 1 / Stream 2 branch so **only genuine navigation sentences** (and never command responses) are logged to `.nmea` files.
3. **Quiet-First Command Sequencing in `um982_configure_rover_defaults()`:**
   * Currently, `um982_configure_rover_defaults()` enables `GPGGA`, `GPRMC`, `GPGST`, `GPGSV`, `UNIHEADINGA`, `GNGGAH`, and `BASEINFOA` **before** running `MODE` and `CONFIG` queries, causing `15 Hz` of NMEA traffic to interleave with the `MODE` and `CONFIG` responses.
   * We will reorder `um982_configure_rover_defaults()` so that:
     1. `UNLOGALL COM1` and `UNLOGALL COM2` execute **first** (immediately silencing all periodic NMEA traffic on warm boots).
     2. All mode/heading setup commands (`CONFIG PVTALG MULTI`, `MODE ROVER SURVEY`, `CONFIG HEADING ...`) and diagnostic queries (`VERSIONA`, `MODE`, `CONFIG`) execute while `COM1` is **100% quiet** (only command responses on the wire).
     3. Periodic output logs (`GPGGA COM1 0.5`, `GPRMC COM1 1`, `GPGST COM1 0.5`, `GPGSV COM1 1`, `UNIHEADINGA COM1 0.5`, `GNGGAH COM1 1`, `BASEINFOA COM1 1`) and `SAVECONFIG` execute **last**.
4. **Expanded Line Buffer (`com1_line_buf[512]`) with Overflow-Discard State:**
   * Expand `com1_line_buf` from `256` to `512` bytes to accommodate long `#VERSIONA` / `#UNIHEADINGA` / `#CONFIG` lines, and if a corrupt line ever exceeds `511` bytes without `\r\n`, discard characters until the next `\r`/`\n` delimiter instead of resetting `com1_line_pos = 0` mid-line (preventing a long line's tail from being parsed as a new sentence).

### 2.3 `um982_query_receiver_state()` (`VERSIONA`, `MODE`, `CONFIG`, `LOGLIST`) & Known-State Audit

To verify Single-UART `COM1` consolidation and maintain a deterministic known state of the `UM982` at boot, we will add a dedicated query function (`um982_query_receiver_state()`):

1. **Four-Command Audit Sequence (`VERSIONA` $\to$ `MODE` $\to$ `CONFIG` $\to$ `LOGLIST`):**
   * **`VERSIONA`:** Captures model, firmware build, and hardware revision (`#VERSIONA,...;UM982,R4.10Build...`) into `port_stats.um982_version`.
   * **`MODE`:** Captures active RTK operating mode (`#MODE,...;MODE ROVER SURVEY`) into `port_stats.um982_mode`.
   * **`CONFIG`:** Prints every `$CONFIG,...` line to the Serial Monitor and parses key receiver settings (`PVTALG MULTI`, `HEADING FIXLENGTH`, `HEADING LENGTH`, `HEADING LOWDYNAMIC`, `COM1 115200`) into `port_stats.um982_config_summary`.
   * **`LOGLIST`:** Queries `#LOGLIST` to enumerate every active periodic message, port assignment (`COM1`/`COM2`/`COM3`), trigger type (`ONTIME`), and interval (`0.5`, `1.0`).
     * Summarizes active logs into `port_stats.um982_loglist` (e.g., `"COM1(7): GGA(0.5) RMC(1) GST(0.5) GSV(1) HDG(0.5) GGAH(1) BASE(1) | COM2:NONE"`).
     * Sets `port_stats.known_state_ok = true` when `MODE == ROVER SURVEY`, `COM1` has all 7 expected logs, and `COM2` has `0` logs.
2. **Tile 4 Known-State Diagnostic Card (No Extra Button Needed):**
   * Repurposes the obsolete `COM2` card on Tile 4 to display the parsed `VERSIONA`, `MODE`, `CONFIG`, `LOGLIST`, and `KNOWN STATE OK` status alongside the live `RTCM TX (via COM1)` byte counter and `Diff Age`.

### 2.4 Summary of Firmware Changes in `src/um982_driver/` & `user_config.h`
1. **Expand `Serial1` TX Ring Buffer & Disable `Serial2`:**
   * In `um982_driver_init()`:
     * Configure `Serial1.setRxBufferSize(4096)` and **`Serial1.setTxBufferSize(4096)`** (increased from `1024` to `4096` to absorb multi-message RTCM3 bursts without blocking).
     * Remove `Serial2.begin(...)` and `COM2` GPIO pin initialization (`ROVER_PIN_UM982_TXD1` / `RXD1`), freeing `GPIO 47` and `GPIO 48` and saving `~6 KB` of internal `DRAM0`.
2. **Update `um982_configure_rover_defaults()` (Quiet-First Sequence + `um982_query_receiver_state()`):**
   * Issue `UNLOGALL COM1` and `UNLOGALL COM2` first, configure receiver mode/heading, enable the 7 `COM1` logs, run `SAVECONFIG`, and then call **`um982_query_receiver_state()`** (`VERSIONA`, `MODE`, `CONFIG`, `LOGLIST`) to verify and log the saved state.
3. **Route RTCM3 Injection to `COM1` (`Serial1`):**
   * Update `um982_forward_rtcm_to_com2(const uint8_t *data, size_t len)` (and add primary alias `um982_forward_rtcm()`) to write verified RTCM3 frames directly to **`Serial1.write(data, len)`** under `s_com1_tx_mutex` while incrementing `port_stats.com2_tx_bytes` (aliased as `rtcm_tx_bytes` so Tile 4 UI counters continue displaying live forwarded RTCM bytes seamlessly).
4. **Update Power-Up Self-Test (`um982_run_powerup_selftest()`):**
   * Send `UNLOGALL COM1` + `UNLOGALL COM2` + `VERSIONA`, wait specifically for `version_ack_received` (`#VERSIONA,...`) rather than any arbitrary NMEA line, and mark `port_stats.com2_test_passed = port_stats.com1_test_passed`.

---

## 3. Part 2: `MMC5983MA` Auto-Detection, `SET/RESET` Driver & 9-DOF Heading Fusion

### 3.1 Hardware Connection on the Current `3.5B` Prototype (Next Week)
* Mount the SparkFun Qwiic Micro `MMC5983MA` (`SEN-19921`) out on the existing aluminum T-bar crossbar (`6"`–`10"` away from the center pole clamp / battery pack).
* Connect the 4-wire Qwiic cable to the `3.5B` primary I2C0 bus (`400 kHz`):
  * **Black (`GND`)** $\to$ `GND`
  * **Red (`3.3V`)** $\to$ `3V3`
  * **Blue (`SDA`)** $\to$ **`GPIO 8` (`ROVER_PIN_I2C0_SDA`)**
  * **Yellow (`SCL`)** $\to$ **`GPIO 7` (`ROVER_PIN_I2C0_SCL`)**
* **I2C Address Map on `3.5B` (`GPIO 7/8`):**
  * `0x18` = `ES8311` Audio Codec
  * `0x20` = `TCA9554` I/O Expander
  * **`0x30` = `MMC5983MA` 3-Axis AMR Magnetometer (Zero Conflict!)**
  * `0x34` = `AXP2101` PMIC
  * `0x3B` = `AXS15231B` Capacitive Touch
  * `0x51` = `PCF85063A` RTC
  * `0x6B` = `QMI8658C` 6-DOF IMU

### 3.2 New Driver Module: `src/mag_bsp/mmc5983ma_driver.h` & `.cpp`
All I2C transactions use the existing thread-safe `i2c_bsp_lock()` / `i2c_master_transmit_receive()` API (`v0.7.6` hardened `400 kHz` bus):

1. **Boot Auto-Detection (`mmc5983ma_init()`):**
   * Probes I2C address `0x30` and reads register **`0x2F` (`PRODUCT_ID_1`)**.
   * If the bus NACKs or `reg[0x2F] != 0x30`: sets `s_mag_present = false`, logs `[MAG] MMC5983MA (0x30) not detected — 6-DOF mode active`, and returns `false` cleanly.
   * If `reg[0x2F] == 0x30`: sets `s_mag_present = true`, issues a software reset (`CTRL1 = 0x80`, wait `15 ms`), configures **`BW = 01` (`200 Hz` / `4 ms` decimation, `0.6 mG` noise)** and **`CM_Freq = 100` (`50 Hz` continuous measurement mode, `18-bit` resolution = `16,384 LSB/Gauss` = `163.84 LSB/µT`)**, and executes an initial **`SET/RESET` bridge offset calibration**.
2. **Periodic Hardware `SET/RESET` Degaussing & Bridge-Offset Elimination (`mmc5983ma_calibrate_bridge_offset()`):**
   * Per Page 18 of the `MMC5983MA` Datasheet (`Rev A`):
     1. Write `0x08` (`SET` bit) to `Internal Control 0` (`0x09`) $\rightarrow$ pulses `500 ns` restoring current through the AMR film via the `10 µF` `CAP` reservoir capacitor.
     2. Trigger measurement (`TM_M = 0x01`) and read 18-bit $\mathbf{M}_{\text{SET}} = (+\mathbf{H} + \mathbf{Offset})$.
     3. Write `0x10` (`RESET` bit) to `Internal Control 0` (`0x09`) $\rightarrow$ flips AMR magnetization $180^\circ$.
     4. Trigger measurement (`TM_M = 0x01`) and read 18-bit $\mathbf{M}_{\text{RESET}} = (-\mathbf{H} + \mathbf{Offset})$.
     5. Issue final `SET` (`0x08`) to restore normal polarity and compute exact null-field bridge offset:
        $$\mathbf{Offset}_{xyz} = \frac{\mathbf{M}_{\text{SET}} + \mathbf{M}_{\text{RESET}}}{2}, \qquad \mathbf{H}_{xyz} = \frac{\mathbf{M}_{\text{SET}} - \mathbf{M}_{\text{RESET}}}{2}$$
   * Executed automatically at boot, every `60 seconds` in the background, or whenever the on-chip die temperature (`Tout` register `0x07`, $0.8^\circ\text{C/LSB}$) changes by $\ge 2.4^\circ\text{C}$ (3 LSBs).
3. **50 Hz Poll & Tilt-Compensated Magnetic Azimuth (`mmc5983ma_poll()`):**
   * Reads `7` bytes (`0x00..0x06`: `Xout0`, `Xout1`, `Yout0`, `Yout1`, `Zout0`, `Zout1`, `XYZout2`), reconstructs 18-bit unsigned integers (`0..262143`, null field `131072`), subtracts $\mathbf{Offset}_{xyz}$ and hard-iron bias $\mathbf{B}_{\text{bias}}$, and converts to microteslas ($\mu\text{T}$).
   * Using the crossbar pitch ($\theta$) and roll ($\phi$) from the onboard `QMI8658` accelerometer/gyro filter, projects $\mathbf{B}_{xyz}$ into the horizontal local tangent plane:
     $$B_x^h = B_x \cos\theta + B_y \sin\phi \sin\theta + B_z \cos\phi \sin\theta$$
     $$B_y^h = B_y \cos\phi - B_z \sin\phi$$
     $$\psi_{\text{mag}} = \text{atan2}(-B_y^h, B_x^h)$$

---

### 3.3 Dual-GNSS Auto-Calibration & 9-DOF Heading Hierarchy in `nav_engine`

1. **Continuous True-North Tare Learning (Whenever `UM982` Has `HDG FIX`):**
   * Whenever `UNIHEADINGA` reports `HDG_SRC_DUAL_FIX` ($\sigma_\psi \approx 0.20^\circ$), pole tilt is $< 15^\circ$, and $\|\mathbf{B}\|$ is within normal Earth field bounds ($25\text{--}75\,\mu\text{T}$):
     * Computes the circular angular residual $\Delta\psi_{\text{tare}} = \text{wrap180}(\psi_{\text{UM982\_True}} - \psi_{\text{mag}})$.
     * Updates a circular Exponential Moving Average (`EMA`, $\alpha = 0.05$) of `mag_true_north_offset_deg` and records the ambient geomagnetic field magnitude $B_{\text{ref\_uT}} = \|\mathbf{B}\|$.
     * This single learned offset automatically absorbs **local WMM magnetic declination ($\approx -14.2^\circ$ in ME/NH)**, **sensor mounting angle on the crossbar**, and **static radial bias**, aligning the compass to within $< 0.5^\circ$ of True North with zero manual user calibration!
     * Persists `mag_true_north_offset_deg` and `mag_field_ref_uT` in `/settings.json` (via the debounced Priority-1 background save) so the pole wakes up on the next cold boot in deep woods with a pre-calibrated True North offset!
2. **Iron-Pipe / Magnetic Anomaly Gatekeeper:**
   * Compares live $\|\mathbf{B}\|$ against $B_{\text{ref\_uT}}$. If $\big|\|\mathbf{B}\| - B_{\text{ref\_uT}}\big| > 3.5\,\mu\text{T}$ (`35 mG`, e.g., holding the pole next to an iron boundary pipe, steel fence post, or truck), `mag_anomaly_detected = true` and the 9-DOF filter temporarily ignores magnetic yaw updates, coasting purely on the `QMI8658` gyro until the pole moves away from the iron disturbance.
3. **Updated Heading Source Priority Chain (`HeadingSource_t`):**
   * `HDG_SRC_DUAL_FIX` (`1`) — `UM982` narrow-lane integer carrier phase (`0.20°`) + continuous compass auto-calibrator.
   * `HDG_SRC_DUAL_FLT` (`2`) — `UM982` float baseline (if baseline length passes $\pm 15\text{ cm}$ sanity check).
   * `HDG_SRC_WALK` (`3`) — Kinematic displacement ("Walk 3 Steps" $\ge 0.70\text{ m}$) / COG while moving.
   * **`HDG_SRC_COMP` (`5`, New Badge: `HDG CMP` / `CMP` in Vivid Teal `#14B8A6`)** — 9-DOF complementary filter ($\psi_k = 0.98(\psi_{k-1} + \omega_z \Delta t) + 0.02(\psi_{\text{mag}} + \Delta\psi_{\text{tare}})$):
     * Active immediately at cold boot if `UNIHEADINGA` has not locked yet (`0.0 s` wait in deep woods).
     * Active indefinitely when stationary under dense canopy (`no 45-second timeout!`).
   * `HDG_SRC_GYRO` (`4`) / `HDG_SRC_HOLD` (`6`) — Fallback if `MMC5983MA` is not installed (`6-DOF` mode) or during a temporary $\|\mathbf{B}\|$ iron anomaly.

---

## 4. UI & Telemetry Updates (`v0.8.0`)

1. **Row 3 Persistent Status Header:**
   * Displays **`[CMP]`** (Teal `#14B8A6`) when steering via the 9-DOF `MMC5983MA` + `QMI8658` fusion under canopy.
2. **Tile 3 (Compass & Bubble Level):**
   * Shows live `MMC5983MA` status when auto-detected: `Mag: 52.4 uT | Tare: -14.3 deg (Auto-Cal OK)` (or `Mag: IRON ANOMALY` in Amber if near steel).
3. **Tile 4 (Hardware Diagnostics):**
   * Updates the `UM982 COM1` card to show `COM1 Full-Duplex (NMEA RX + RTCM3 TX | COM2 Freed)` and replaces the obsolete `COM2` card with a live **`[UM982 RECEIVER STATE & LOGLIST]`** card (`VERSIONA`, `MODE`, `CONFIG`, `LOGLIST`, and `KNOWN STATE OK` badge).
   * Adds an **`MMC5983MA Compass`** diagnostic line showing `ONLINE (0x30, 18-bit SET/RESET)` or `NOT DETECTED (6-DOF Fallback)`.
4. **Base Station LoRa Telemetry (`45-Byte` Frame & `e220_pacer.py`):**
   * Maps `HDG_SRC_COMP = 5` to `[COMP]` in `e220_pacer.py` so Base Station logs record when the rover is operating in 9-DOF compass mode under canopy.

---

## 5. Step-by-Step Implementation & Verification Plan

### Phase 8A: Build & Verify on Current Hardware (Today — Before `MMC5983MA` Arrives)
1. **Step 1 — Single-UART `UM982 COM1` Consolidation:**
   * Update `src/um982_driver/um982_driver.cpp` and `user_config.h` to send `RTCM3` over `Serial1` (`COM1`, `GPIO 17/18`), disable `Serial2` (`COM2`), and configure `UM982` with `UNLOGALL COM2`.
2. **Step 2 — Auto-Detected `MMC5983MA` Driver & 9-DOF `nav_engine` Integration:**
   * Create `src/mag_bsp/mmc5983ma_driver.h` and `src/mag_bsp/mmc5983ma_driver.cpp`.
   * Integrate boot auto-detection (`0x30`), `SET/RESET` offset cancellation, `UM982` True-North auto-tare learning, and `HDG_SRC_COMP` into `nav_engine.cpp`, `rover_api.h`, `settings_manager.cpp`, and `RTK_Rover.ino`.
3. **Step 3 — Bench & Outdoor Verification (Without Compass Module Attached):**
   * Physically disconnect `UM982 COM2` wires (`GPIO 47` and `GPIO 48`).
   * Boot `RTK_Rover v0.8.0`:
     * Verify `[UM982 COM1 TEST] PASS` and `[MAG] MMC5983MA (0x30) not detected — 6-DOF fallback active`.
     * Verify LoRa RTCM3 corrections flow into `UM982 COM1` (`RTCM: 1.0s`), achieving **`RTK FIX`** and **`HDG FIX`** on a single 4-wire UART connection (`5V`, `GND`, `IO17`, `IO18`).
     * Verify `[MEM] Post-Setup Heap` shows the extra `~6 KB` of reclaimed internal `DRAM0` SRAM.

### Phase 8B: Plug-and-Play Verification (Next Week — When `MMC5983MA` Arrives)
1. **Step 1 — Plug in Qwiic `MMC5983MA` to `GPIO 7 (SCL)` / `GPIO 8 (SDA)` on T-Bar:**
   * Boot `RTK_Rover v0.8.0` with zero code changes needed.
   * Confirm Serial log and Tile 4 report `[MAG] MMC5983MA (0x30) ONLINE — Product ID 0x30, SET/RESET Offset Calibrated`.
2. **Step 2 — Field Auto-Calibration & Canopy Test:**
   * Stand in open sky until `UM982` achieves `HDG FIX`; slowly rotate the pole $90^\circ\text{--}180^\circ$ and verify `Mag Tare` converges automatically on Tile 3.
   * Disconnect `ANT2` (simulating total Slave antenna loss in deep hemlock canopy) and verify:
     * Heading immediately transitions to **`[CMP]`** (`HDG_SRC_COMP`) with $< 1.5^\circ$ step change.
     * Rotating the pole in place tracks smoothly with zero drift over $5+\text{ minutes}$ stationary.
     * Rebooting the rover with `ANT2` still disconnected yields instant valid True North heading (`[CMP]`) at `0.0 seconds` from boot.
