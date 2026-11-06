# RTK Rover - Project Overview

**Document Version:** `v2.4.0`  
**Last Updated:** November 6, 2026  
**Firmware Target:** `v0.7.5`  
**Python Ecosystem Target:** `v3.4.0`  
**License:** GNU General Public License v3.0 (`GPLv3`)  

This document serves as the master technical specification for the **RTK Rover** project. It details the mechanical setup, dual-antenna lever-arm kinematics, electrical pinouts, FreeRTOS + LVGL 9 firmware architecture, operational modes, and companion Python scripts.

---

## 1. Project Identity, Versioning & Documentation Rules
* **CRITICAL RULE:** All projects shall have a main Software version identifier as `v[Major].[Minor].[Patch]` and follow industry-standard SemVer rules and practices for bumping the version at EVERY file iteration.
* **Why:** We do not reuse version numbers for each iteration. This allows us to distinctly identify in the AI context the files we are working on. The version string is defined as `ROVER_FIRMWARE_VERSION` in `user_config.h`.
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
   * Receives RTCM3 differential corrections streamed from the RTK Base Station in transparent mode at `115200` baud (forwarded by the ESP32 Core 0 bridge to UM982 COM2 at `115200` baud).
   * **Radio Configuration (Verified on both Base and Rover, Profile Rev 2):**
     * **RF Channel (`e220_channel`):** `75` (`0x4B` $\rightarrow$ $850.125 + 75 = 925.125\text{ MHz}$, ranked `#1` cleanest US 902-928 MHz ISM channel in `test/e220-scan.csv` at `-96.23 dBm` avg, `+10.125 MHz` clear of `915.0 MHz` weather telemetry).
     * **UART Rate (`e220_uart_baud`):** `115200` bps, `8N1` (`REG0 = 0xE6`; matching UM982 COM2 baud rate).
     * **Air Data Rate (`e220_air_rate_kbps`):** `38.4` kbps (`REG0 = 0xE6`; supports `2.4, 4.8, 9.6, 19.2, 38.4, 62.5`).
     * **Packet Size (`e220_subpacket_bytes`):** `200 bytes` (`REG1` bits `[7:6] = 0b00`, utilizing native `400-byte` buffer atomic framing).
     * **Mode 0 Live RSSI Register (`REG1` Bit 5 = `0x20`):** Enabled (`REG1 = 0x23`), allowing on-demand `C0 C1 C2 C3 00 02` ambient noise floor and last-packet RSSI queries (4-byte and 5-byte reply parser synchronized to post-`1094` inter-epoch quiet gap) without appending extra bytes to RX UART frames (`REG3 = 0x00`).
     * **Transmit Power (`e220_tx_power_dbm`):** Level 3 (`21 dBm` / `125 mW` close/medium-range testing; `30 dBm` / `1 W` long-range field production, `REG1 = 0x23` / `0x20`). Note: keep Base and Rover separated by $\ge 30\text{--}100\text{ ft}$ even at `21 dBm` to prevent `E220-900T30S` `+14 dB` RX LNA front-end saturation (`> -35 dBm`).
     * **Transmission Mode & LBT (`e220_lbt_enable`):** Transparent Mode, LBT Disabled, Appended RSSI Byte Disabled (`REG3 = 0x00`).
     * **Target EEPROM Profile:** `ADDH=0x00 ADDL=0x00 REG0=0xE6 REG1=0x23 CHAN=0x4B REG3=0x00`.
   * **Base Station Software & Datasheet Section 5.6.2 Pacing Architecture:**
     * **RTKBase Stream Filtering:** RTKBase port `5016` is filtered via `str2str` to `"1006(5),1074,1084,1094,1230(10)"`, producing a lean, high-fidelity stream of ~625-650 Bytes/s (GPS MSM4, GLONASS MSM4, Galileo MSM4, Base ARP, and GLONASS biases).
     * **Two-Regime AUX + Hardware-Calibrated Time-on-Air Pacing (`scripts/e220_pacer.py v3.2.0`):**
       * Per E220-900T30S Manual Section 5.6.2, `AUX=1` indicates the 400-byte UART buffer emptied into the LLCC68 transceiver, **not** that over-the-air RF transmission has completed.
       * **Single-Subpacket Frames (`<= 200B`, e.g., `1006`, `1230`, `1084`):** Transfer straight into an idle LLCC68 FIFO without holding `AUX` LOW; paced deterministically by silicon-calibrated LoRa Time-on-Air (`82.2 ms` for `200B`, `58.3 ms` for `122B` @ `38.4k`) plus a `35 ms` receiver UART drain guard delay.
       * **Two-Subpacket Frames (`201..400B`, e.g., `1074`, `1094`):** Hold `AUX` LOW (~82 ms @ 38.4k) while Subpacket 1 (`200B`) transmits on air, then raise `AUX` HIGH the instant Subpacket 2 (`len - 200`) enters the LLCC68 FIFO and **begins** RF TX; paced by waiting for `AUX` rising edge plus `estimate_subpacket_air_s(len - 200) + 35 ms` guard delay.
       * Includes `GPIO 22` hardware reset watchdog if `AUX` locks LOW `> 1.5 s`, `TIOCEXCL` exclusive serial port locking, source `CRC24Q` verification, and startup Channel 75 ambient RF noise floor probing.
     * **Adaptive Constellation Interleaver (at $\le 9.6\text{ kbps}$):**
       * Interleaves secondary constellations across alternating epochs to prevent LoRa channel saturation at slower air rates: GPS 1074 is transmitted every second ($1.0\text{ Hz}$), while GLONASS 1084, Galileo 1094, and BeiDou 1124 alternate ($0.5\text{ Hz}$ each for 2 secondaries, $0.33\text{ Hz}$ for 3).
       * Base ARP (1005/1006), antenna (1033), and GLONASS biases (1230) retain 100% passthrough.
       * Guarantees a $200\text{ ms}$ half-duplex quiet window for receiving Rover uplink telemetry (`0xAA 0x55` 39-byte frames) and remote air-rate switch commands (`0x02` / `0x82`).
       * Supports `--no-interleave` CLI flag and reports `Interleave:[N skp]` in the console heartbeat.
     * **Link Budget & Passive-XTAL Thermal Stability at 38.4k:**
       * Because the `E220-900T30S` (`LLCC68` + 1W PA) uses a passive `32 MHz` crystal (`XTAL`, no `TCXO`), operating at `38.4 kbps` (`500 kHz` LoRa bandwidth, `~37-45%` air duty cycle) provides `2x-4x` wider carrier/thermal drift tolerance on multi-subpacket `201..400B` bursts than `19.2 kbps` (verified across 16 empirical tests: `223` intact `> 200B` frames at `38.4k`/`62.5k` vs `0` at `19.2k`) while maintaining $\approx -123\text{ dBm}$ receiver sensitivity.

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

### E. Deep-Woods Heading Fallback Architecture ("Walk 3 Steps" Displacement + Gyro Yaw)
In dense forest canopy where secondary (Slave) antenna carrier tracking drops, the firmware implements an autonomous multi-tier heading fallback pipeline:
* **Tier 1 (Dual-Antenna RTK Moving Baseline):** `HDG FIX` / `HDG FLT` ($0.2^\circ$ precision) whenever the UM982 resolves carrier phase.
* **Tier 2 (Kinematic Geodetic Displacement Azimuth):** When dual-antenna lock is lost, `nav_heading_displacement_update()` tracks total WGS84 displacement from the last stationary anchor. Upon moving $\ge 0.70\text{ m}$ ($2\text{--}3$ walking paces), it computes geographic azimuth via `nav_compute_target_guidance()`. Unlike receiver Doppler COG, this displacement azimuth is **velocity-independent**, resolving accurate headings even when crawling at $< 0.4\text{ m/s}$ through thick brush. Displayed as `HDG WALK` (`WLK`, Cyan `#06B6D4`).
* **Tier 3 (50 Hz QMI8658 Gyroscope Yaw Integration):** When the surveyor pauses, stands over a pin, or rotates in place, the Z-axis gyroscope integrates angular velocity ($\Delta \psi = \omega_z \cdot \Delta t$) with a 60-second hold window. This preserves compass and map orientation while stationary. Displayed as `HDG GYRO` (`GYR`, Blue `#3B82F6`).
* **UM982 Fallback Protection:** `handle_uniheadinga_sentence()` selectively invalidates only dual-antenna heading when Slave tracking is lost, strictly preserving displacement, gyro, COG, and hold headings instead of zeroing orientation twice per second.

---

## 4. Electrical Pinouts & Hardware Mapping (`ESP32-S3-Touch-LCD-3.5B`)

> **PIN NAMING CONVENTION:**
> All peripheral pin labels are **device-relative**:
> * `<DEV>_TXD` is the **Transmit Output from the external module** $\rightarrow$ wired to the **ESP32 UART RX Input**.
> * `<DEV>_RXD` is the **Receive Input to the external module** $\leftarrow$ wired from the **ESP32 UART TX Output**.

| Subsystem | Signal Label | ESP32-S3 GPIO | ESP32 Direction | Function / Bus Assignment |
| :--- | :--- | :--- | :--- | :--- |
| **UM982 COM1** | `UM982_TXD0` | `GPIO 17` | **ESP32 RX** (`Serial1`) | UM982 COM1 TX $\rightarrow$ ESP32 RX (`115200 8N1`, NMEA/Heading stream) |
| **UM982 COM1** | `UM982_RXD0` | `GPIO 18` | **ESP32 TX** (`Serial1`) | ESP32 TX $\rightarrow$ UM982 COM1 RX (`115200 8N1`, Config commands) |
| **UM982 COM2** | `UM982_TXD1` | `GPIO 47` | **ESP32 RX** (`Serial2`) | UM982 COM2 TX $\rightarrow$ ESP32 RX (`115200 8N1`, Responses) |
| **UM982 COM2** | `UM982_RXD1` | `GPIO 48` | **ESP32 TX** (`Serial2`) | ESP32 TX $\rightarrow$ UM982 COM2 RX (`115200 8N1`, RTCM3 Stream Forward) |
| **E220 LoRa** | `E220_TXD` | `GPIO 41` | **ESP32 RX** (`Serial0`) | E220 TXD $\rightarrow$ ESP32 RX (`115200 8N1` Transparent / `9600` Config) |
| **E220 LoRa** | `E220_RXD` | `GPIO 42` | **ESP32 TX** (`Serial0`) | ESP32 TX $\rightarrow$ E220 RXD |
| **E220 Control** | `E220_AUX` | `GPIO 21` | **ESP32 Input** | Busy (`LOW`) / Idle & Ready (`HIGH`) status |
| **E220 Control** | `E220_RST` | `GPIO 38` | **ESP32 Output** | Hardware Reset (`HIGH` = normal operation) |
| **E220 Control** | `E220_M0` | `GPIO 39` | **ESP32 Output** | Mode 0 (`LOW` = Transparent, `HIGH` = Config) |
| **E220 Control** | `E220_M1` | `GPIO 40` | **ESP32 Output** | Mode 1 (`LOW` = Transparent, `HIGH` = Config) |
| **Internal I2C0** | `ESP_SCL/SDA` | `GPIO 7 / 8` | **I2C Master** | Shared by Touch (`0x3B`), TCA9554 (`0x20`), AXP2101 (`0x34`), RTC (`0x51`), IMU (`0x6B`), ES8311 (`0x18`) at `400 kHz` on Core 1 |
| **LCD QSPI** | `CS/CLK/D0..3` | `12, 5, 1..4` | **QSPI Master** | `AXS15231B` `320x480` Portrait Display (`GPIO 6` Backlight PWM) |
| **SD Card** | `D0/CMD/CLK` | `9, 10, 11` | **1-Bit SD_MMC / SPI** | 3-stage mount fallback (`SD_MMC` 1-bit `EXIO3=High-Z` $\rightarrow$ `SD_MMC` `EXIO3=LOW` $\rightarrow$ `SD` SPI) |
| **BOOT Button** | `BOOT_SWITCH` | `GPIO 0` | **ESP32 Input** | **Long-Press ($\ge 1.5\text{ s}$):** Capture 24-bit uncompressed BMP screenshot |

---

## 5. FreeRTOS & LVGL 9 Firmware Architecture

### Core Separation
* **Core 0 (`task_rover_core0`, Priority 4 — Zero I2C Access):**
  * Polls `E220` (`Serial0`), logs raw wire-tap bytes to `rtcm-base_*.txt`, stages incoming bytes in a 1,536-byte **Sliding-Window Store-and-Verify RTCM3 Gatekeeper** (`0xD3` + `CRC24Q == 0`), and forwards **only verified intact RTCM3 frames** atomically to `UM982 COM2` (`Serial2`) while tracking `IO21` `AUX` falling-edge interrupts and 5-second Mode 0 `Noise/Pkt RSSI` (`dBm`).
  * Polls `UM982 COM1` (`Serial1`) to parse `$GPGGA` (2 Hz), `$GPRMC` (1 Hz), `$GPGST` (2 Hz), `$GPGSV` (1 Hz), `$GNGGAH` (1 Hz slave data), `#BASEINFOA` (1 Hz base station data), and `#UNIHEADINGA` (2 Hz).
  * Runs 30Hz lever-arm kinematic projection and geodesic target guidance (`nav_engine`).
* **Core 1 (`rover_lvgl_port_task` Priority 3, `task_display_and_audio` Priority 2 — Exclusive I2C0 Owner):**
  * Exclusively owns `I2C_NUM_0` (`400 kHz`, serialized via `i2c_bus_mutex` with automatic `i2c_master_bus_reset()` recovery): polls `AXS15231B` touch (`30 Hz`), `QMI8658` IMU (`10 Hz` independent of LVGL mutex), `PCF85063` RTC (`1 Hz` cached + one-shot GNSS UTC sync), and `AXP2101` battery telemetry (`0.5 Hz`).
  * Renders the `320x480` Portrait LVGL 9 interface protected by `rover_lvgl_lock()` / `rover_lvgl_unlock()`.
  * Multi-tone audio beep synthesis via FreeRTOS `beepQueue` driving the ES8311 codec.
  * Debounced `settings.json` persistence (4s debounce after changes).
  * Hardware screenshot generation directly to MicroSD card.

### Memory Partitioning: Internal `DRAM0` vs. 8 MB Octal PSRAM (`v0.7.5`)
Because prebuilt Arduino-ESP32 (`v3.3.x`, `qio_opi`) compiles with `CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL = 4096` and `CONFIG_SPIRAM_MALLOC_RESERVE_INTERNAL = 0`, explicit memory-tier partitioning is enforced so `esp_wifi_init()` / `esp_wifi_start()` (and future BLE NUS stacks) always retain $\ge 150\text{ KB}$ of contiguous internal 8-bit/DMA SRAM:
* **8 MB Octal PSRAM (`MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT`):**
  * Dual full-frame LVGL 9 RGB565 framebuffers (`2 x 307,200 B = 614.4 KB`).
  * `serial_logger` (`32,768 B`), `rtcm_logger` (`16,384 B`), and `nmea_logger` (`8,192 B`) ring buffers (`57.3 KB` total).
  * `g_waypoints` survey table (`MAX_WAYPOINTS = 80`, `25,600 B`).
  * ~400+ LVGL 9 UI widget, style, table, and canvas objects (`~50 KB`), routed via scoped `heap_caps_malloc_extmem_enable(64)` strictly around `create_rover_ui()` (immediately restored to `4096` afterward).
* **Internal `DRAM0` / `MALLOC_CAP_DMA` SRAM (Static `.dram0.bss = 37,104 B` / `18%` global dynamic RAM):**
  * 24-line QSPI LCD DMA bounce strip (`LVGL_DMA_BUFF_LEN = 15,360 B`, exact 20-strip divisor of 480 px).
  * `SD_MMC` IDMAC descriptors, `FFat` sector buffers, `ES8311` I2S DMA buffers, FreeRTOS task stacks, and the ESP32-S3 Wi-Fi MAC/PHY + LwIP RX/TX DMA pools.

---

## 6. UI Tile Structure (`320x480` Portrait)

### Persistent Top Header Bar (`320x74`)
* **Row 1:** Local Date (`YYYY-MM-DD`), Local Time (`HH:MM:SS TZ`), Units badge (`SI` / `IMP`), Multi-state Wi-Fi icon (Cyan STA, Violet AP, Amber CONN, Slate OFF), and Minimalist Power Widget (`[USB] 5.04V` / `[Bat] 85% 3.92V`).
* **Row 2 (Master Antenna ANT1):** Fix badge (`RTK FIX`, `RTK FLT`, `DGPS`, `SINGLE`, `OFF`), Satellites (`SV: used/view`), Precision (`+/- h_acc`), and RTCM differential age (`RTCM: X.Xs`).
* **Row 3 (Slave Antenna ANT2):** Heading Fix badge (`HDG FIX`, `HDG FLT`, `OFF`), Slave satellites (`SV: X`), Baseline length (`BL: 55.5cm`), and True Heading (`Hdg: XXX.X`).
* **Row 4:** Ground-tip Latitude, Longitude, and Ellipsoidal Height in 14pt Montserrat font (`0.00000000, 0.00000000, 0.000 m`).

### Tile Walkthrough
* **Tile 0 (Screen 1) -- Multi-Mode Navigation & Pin Seeker:**
  * Interactive tri-state navigation badge in top-left corner cycles through three specialized navigation modes:
    1. **`[TACTICAL GUIDANCE]` (Cyan):** Course-Up dynamic canvas auto-scaling from $500\text{ m} \rightarrow 5\text{ m}$. Rotating antenna crossbar with colored tips (Green ANT1, Amber ANT2) and green field-of-view guidance wedge. Line-of-sight tracking vector, turn prompt, distance readout, cardinal North/East offsets, 64-point breadcrumb trail, and high-precision Bullseye mode automatically active at $\le 1.5\text{ m}$ ($1\text{ m}$, $20\text{ cm}$, and $2\text{ cm}$ rings with ES8311 proximity audio lock cues).
    2. **`[LINE STAKEOUT]` (Amber):** Directs surveyor along a boundary line between Point A and Point B. Displays vertical centerline baseline with traverse chevron, decluttered vector geometry (text labels moved to top marquee row to eliminate button overlap), perpendicular tie-line, live rover marker, one-tap swap (`[<-> SWAP]`) to reverse line direction, and scaled 4-row readout deck (Row 1: 16pt offset magnitude, Row 2: ASCII lateral steer cues `<<<`/`>>>`, Row 3: station progress `Sta: 0+78.6 ft`, Row 4: cadastral deed bearing).
    3. **`[MAP VIEW]` (Emerald Green):** Cadastral 2D tangent-plane projection canvas with interactive single-touch drag pan and right-edge HUD Zoom buttons (`[+]` in, `[-]` out, `[FIT]` bounding box, `[CTR]` center on rover). Renders closed boundary polygon connecting all points with positive `poly_order` ($1 \rightarrow 2 \dots \rightarrow N \rightarrow 1$), dashed tie-lines for negative tie points ($-k$ connecting to vertex $k$), color-coded pins (Gold for boundary vertices, Orange for reference/tie pins, Cyan for active target), 128-point historical route trail (toggleable via `"map_show_route_trail"`), live rover marker with heading beam, and geodetic center coordinates.
  * Route recording toggle (`REC ON` / `REC OFF`).
* **Tile 1 (Screen 2) -- Field Survey Logger & Quick Settings:**
  * **Row 1 (`y = 8`):** Symmetrical Location ID Prefix dropdown (`<DERIVED>`, `PIN`, `PIPE`, `MONUMENT`, `STAKE`, `CORNER`, `FENCE`, `TREE`, `UTILITY`, `WP`) and Averaging duration button (`1s`, `5s`, `10s`, `30s`).
  * **Row 2 (`y = 44`):** Quick Note Tag dropdown (`Found Iron Pipe`, `Found Rebar`, `Found Stone Bound`, `Set Temp Flag`, etc.).
  * **Row 3 (`y = 82`):** Next ID row (`ID: <label>`) in Montserrat 14 `#FFCC00` with circular marquee for long compound labels.
  * **Row 4 (`y = 108`):** Multi-epoch averaging progress bar.
  * **Row 5 (`y = 124`):** Large `[LOG PIN NOW]` button with visual feedback.
  * **Row 6 (`y = 172`):** Last logged pin label and destination CSV path (`/survey_logged_pins.csv`).
  * **Row 7 (`y = 218`):** Subtle horizontal divider line separating survey controls from settings.
  * **Quick Settings Consolidated 3x4 Grid (`y = 238..360`):**
    * Row 1: `VOL: 20%..100% / MUTE`, `AUD: GEIGER / BEEPS / MUTE`, `BRI: 20%..100%`
    * Row 2: `SERIAL: ON/OFF`, `NMEA: ON/OFF`, `RTCM: ON/OFF`
    * Row 3: `TELEM: ON/OFF`, `IMU: ON/OFF`, `TOUCH: ON/OFF`
    * Row 4: `WIFI: CONN/STA/AP/OFF` (Multi-state color feedback), `UNITS: IMP/SI`, `LORA: 19.2k / 9.6k`
    * Row 5: Centered Wi-Fi status & web portal URL (`http://<ip>/`).
* **Tile 2 (Screen 3) -- Waypoint CSV Browser & Multi-File Selector:**
  * Top navigation bar with `< Prev` and `Next >` file buttons and circular marquee for filenames.
  * Instant sub-25ms touch responsiveness powered by a 20 ms LVGL polling loop and RAM survey stem cache (`s_stem_cache`), eliminating SD card blocking.
  * Ingests standard 19-column and 20-column CSV files with `poly_order` cadastral signed sequence numbering.
  * Dynamic scanning of `/waypoints.csv` and all `/waypoints_*.csv` with alphabetical sorting.
  * Scrollable table of waypoints; tapping any row selects it as the active navigation target.
  * Remembers and restores the active file and active waypoint index across reboots.
* **Tile 3 (Screen 4) -- Dual-Antenna Compass & Bubble Level:**
  * Numerical True Heading readout in prominent 24pt font (`Heading: 162.4`).
  * Baseline length ($55.5\text{ cm}$) and pitch ($^\circ$) verification with display pose indicator.
  * Enlarged $224\text{ px}$ diameter spirit bubble level with Course-Up 8-point compass ring (`N`, `NE`, `E`, `SE`, `S`, `SW`, `W`, `NW`) keeping heading UP (12 o'clock).
  * Compact **Tare** button (`62x28 px`) nested in the bottom-right of the circle's bounding rectangle (`TARE` Blue / `RST` Orange).
  * Centered Pole Tilt angle readout (`dX` / `dY`) below circle.
* **Tile 4 (Screen 5) -- Hardware Diagnostics & Live Observers:**
  * **Pinned Top Header Deck:** Fixed firmware header (`RTK ROVER v0.7.5` in Montserrat 16 White).
  * **Dual-Power & UPS Card:** Displays external VBUS line voltage, 5V rail status, internal battery voltage/percentage, charging/float status, and auto-shutdown timer countdown.
  * **Pinned LoRa RF Survey Card ($y = 26$):** Elevated to the top of the tile for permanent visibility of noise floor, packet RSSI, AUX edges, RTCM message counters, and truncation drops.
  * **Expanded Scrollable Container (+57 px Viewport):** Multi-line auto-wrapping telemetry cards:
    * Storage status in Electric Cyan (`#22D3EE`).
    * Antenna Master (ANT1) and Slave (ANT2) metrics with live coordinates.
    * Base Station observer card (ID, RTCM differential age, WGS84 coordinates, ECEF vectors, message counters).
    * 6-DOF IMU telemetry (Pitch, Roll, 3-axis Accel, 3-axis Gyro, Tare offsets).
    * UM982 COM1/COM2 baud rates, byte throughput, and error counters.
    * E220 LoRa hardware register decode and boot-time `M0`/`M1` pin continuity self-test.

---

## 7. Python Survey & Ingestion Scripts (`scripts/`)

* **`e220_pacer.py` (v3.4.0):** Datasheet Section 5.6.2 hardware-paced LoRa driver for RTKBase. Handles atomic RTCM3 frame pacing, `GPIO 22` hardware watchdog, `TIOCEXCL` locking, Mode 0 ambient noise floor probing, bi-directional uplink telemetry logging (dual-format 39B/45B auto-detecting unpacker recording ANT1 RTK fix, ANT2 slave fix `SLV_FIX`/`SLV_FLT`, 3D baseline length, crossbar pitch, explicit heading source, `PWR:USB` / `Bat:xx%` power status, RAM JSON + rolling daily CSV), clean ASCII `deg` units in console heartbeat, remote air-rate switching (`0x02`/`0x82`), terminal burst quiet-window enforcement, and Adaptive Constellation Interleaving at $\le 9.6\text{ kbps}$ (interleaving GLONASS 1084, Galileo 1094, and BeiDou 1124 across epochs while preserving 1.0 Hz GPS 1074).
* **`cogo_solver.py` (v1.1.0):** Schema-driven Coordinate Geometry (COGO) traverse solver. Ingests standard parcel JSON schemas ([`docs/schemas/cadastral_schema.json`](schemas/cadastral_schema.json)) and control anchor CSV files, performs forward geodesic projections, verifies loop closure precision, applies Bowditch / Compass Rule adjustments, solves circular road curve geometry, reconciles common boundaries, and exports to Google Earth KML, Rover `/waypoints.csv`, GeoJSON, AutoCAD DXF (with layer organization), and text closure audits. Detailed documentation in [`docs/cogo_solver_guide.md`](cogo_solver_guide.md).
* **`check_secrets.py` (v1.1.1):** Pre-push security verification scanner. Scans Git tracked files and unpushed commits for generic credentials, private keys, and local absolute filesystem paths, plus personal signatures defined in an unversioned `.secrets.local` configuration.
* **`install_git_hooks.py` (v1.1.0):** Automatically installs the pre-push security verification hook into `.git/hooks/pre-push`.
* **`kml_to_waypoints.py` (v1.0.0):** Ingests GIS or Google Earth KML files and extracts placemarks, linestrings, and polygon boundaries into the standardized 19-column rover CSV format.
* **`validate_waypoints.py` (v1.2.0):** Validates CSV schema compliance, bounds, and computes traverse leg distances, bearings, and total closed perimeter.
