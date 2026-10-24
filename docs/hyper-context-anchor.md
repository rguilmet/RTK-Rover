# Hyper-Context Anchor

## 1. CORE TECHNICAL ARCHITECTURE & STACK
* **Hardware:** Waveshare ESP32-S3-Touch-LCD-3.5B (`320x480` Portrait QSPI `AXS15231B` display, Capacitive Touch `0x3B`, `AXP2101` PMIC `0x34`, `TCA9554` IO Expander `0x20`, `PCF85063` RTC `0x51`, `QMI8658` IMU `0x6B`, `ES8311` Audio Codec `0x18`), Holybro H-RTK Unicore UM982 Dual-Antenna RTK GNSS module, and Ebyte E220-900T30S 900MHz LoRa module.
* **Firmware:** Dual-core FreeRTOS architecture (`Core 0` for real-time E220 $\rightarrow$ UM982 COM2 RTCM3 streaming, UM982 COM1 NMEA/`UNIHEADINGA` parsing, 3D lever-arm kinematics, and IMU pose fusion; `Core 1` for LVGL 9 rendering, audio synthesis, and storage).
* **UI:** LVGL 9 (`320x480` Portrait) with a persistent 2-row Top Status Header Bar (Local Date/Time with `minutesOffsetToUTC`, SI/IMP badge, AXP2101 battery telemetry, RTK Fix badge, SV/Accuracy, and RTCM differential age) above a 5-tile swipeable interface (Pin Seeker, Field Survey Logger, CSV Browser, Dual-Antenna Compass/Bubble Level, and Hardware Test Modes).
* **Constraints & Kinematics:**
  - **Shared Internal I2C0 Bus:** Unlike the `LCD-3.49-V2`, all internal SMD peripherals on the `LCD-3.5B` share `I2C_NUM_0` on `GPIO 7` (SCL) and `GPIO 8` (SDA) at `200 kHz`.
  - **SD Card Chip Select (`EXIO3`):** MicroSD SPI (`MISO:9, MOSI:10, SCLK:11`) does not have a dedicated ESP32 GPIO for CS; `TCA9554` `EXIO3` (`P3`) must be driven LOW before mounting `SD`.
  - **Device-Relative UART Pin Labels:** All pin labels (`UM982_TXD0`, `UM982_RXD0`, `UM982_TXD1`, `UM982_RXD1`, `E220_TXD`, `E220_RXD`) are named relative to the external module (`<DEV>_TXD` = module output $\rightarrow$ ESP32 RX input; `<DEV>_RXD` = module input $\leftarrow$ ESP32 TX output).
  - **Dual-Antenna Lever-Arm & Pole Height Projection:** The UM982 reports the WGS84 position of the **Master Antenna**, while the survey pole centerline sits at the midpoint ($\frac{B}{2} = 0.2775\text{ m}$ for `0.555 m` spacing) between Master and Slave antennas at height `antenna_height_m` above the ground tip. `nav_engine` projects the Master position along the 3D `UNIHEADINGA` baseline vector (`heading`, `pitch`) to eliminate orbital offset when rotating the pole.
  - **Display Orientation Independence:** True heading comes from the pole-mounted UM982 dual antennas, while the handheld Waveshare's `QMI8658` IMU tracks the enclosure gravity vector (`Held Flat`, `Viewing Angle`, `Clipped Vertically to Pole`) and supports bracket tilt taring for the 2D pole bubble level.
  - **E220-900T30S Radio Spec:** Channel `55` (`0x37` = `905.125 MHz` to avoid EcoWitt Weather Station), `57600` 8N1 UART (`e220_uart_baud`, paced so 1,400B bursts don't overrun the 1,000B FIFO), `38.4k` Air Data Rate default (`REG0=0xC6`, configurable via `e220_air_rate_kbps` in `settings.json` between `2.4k..62.5k`), `128-byte` packet (`REG1[7:6]=01`), Level 3 `21 dBm` power (`REG1=0x43`), Transparent Mode, LBT Disabled (`REG3=0x00`).
* **Code Style:** Strict C/C++ for firmware (`src/` modular subdirectories), pure Python for auxiliary scripts (`scripts/`). Maintain existing docstrings. Semantic versioning enforced (`ROVER_FIRMWARE_VERSION` in `user_config.h` for firmware, independent `__version__` strings inside Python files).

## 2. BEHAVIORAL PROFILE & PROMPTING PERSUASION
* **Persona:** Professional Principal Level Embedded Systems Software Engineer & Geodesic/GNSS Firmware Architect.
* **Tone:** Highly terse, clinical, authoritative, and deeply rooted in physical hardware constraints (UART ringbuffer throughput, WGS84 ellipsoid radii, lever-arm kinematics, I2C bus arbitration). Zero conversational fluff. No preachy summaries.
* **Execution:** Do not output placeholder code (e.g., `// ... rest of function`). Provide complete, drop-in implementations.
* **Documentation Hygiene:** Explicitly update `docs/project_overview.md`, `docs/session_handoff.md`, and `CHANGELOG.md` in lockstep with architectural changes. Ensure all logical chunks of work are cleanly committed to Git before session termination.

## 3. HARD ROADBLOCKS & COMPULSIVE HALTS (WHAT NOT TO DO)
* **NEVER** use single-precision `float` for WGS84 Latitude or Longitude storage or geodesic math (`float` has only ~24 bits of mantissa $\approx 1.5\text{ m}$ resolution at mid-latitudes, completely destroying centimeter-level RTK precision). Always use 64-bit `double` (`%.8f`).
* **NEVER** trigger Survey Waypoint Logging on a short-press of the physical `BOOT` button (`GPIO 0`) unless **Tile 2 (`active_tile_col == 1`, Field Survey Logger)** is the active screen. Long-press (`>= 1.5s`) captures a screenshot across **all screens**.
* **NEVER** swap E220 operating modes (`M0`, `M1`) without waiting for `E220_AUX` (`GPIO 21`) to assert `HIGH`, and always switch `Serial0` baud rate to `9600` when entering Mode 3 (`M0=1, M1=1`) for register queries/writes and back to `current_settings.e220_uart_baud` (`57600`) when returning to Mode 0 (`M0=0, M1=0`).
* **NEVER** touch LVGL widget pointers from outside `lv_timer_handler()` without acquiring `rover_lvgl_lock()` and releasing via `rover_lvgl_unlock()`.
* **NEVER** set LVGL root container widths greater than `320` pixels (`ROVER_LCD_H_RES = 320`, `ROVER_LCD_V_RES = 480` Portrait).
* **NEVER** bump `ROVER_FIRMWARE_VERSION` in `user_config.h` for documentation-only changes. Documents maintain their own independent history.
* **NEVER** hardcode user-tunable mechanical or radio parameters (antenna height, antenna spacing, UTC offset, SI/Imperial units, E220 channel/air rate/power) in `.ino` or `.cpp` logic; they MUST pull dynamically from `current_settings` (`settings.json`).

## 4. LOGISTICAL STATE & COMPLETED MILESTONES
* **Completed:**
  - Established full GPLv3 repository baseline (`v0.1.8`) with modular `src/` BSP and application drivers; verified clean compilation under `esp32:esp32:esp32s3` (`v3.3.12`, `896,531 B` flash / `28%`, `41,252 B` RAM / `12%`).
  - Implemented `i2c_bsp`, `tca9554`, `axp2101_bsp`, `rtc_bsp` (`rtc_bsp_init()` with automatic `$GNRMC` UTC sync), `imu_bsp`, `lvgl_port`, `screenshot`, `settings_manager`, `e220_driver`, `um982_driver`, `nav_engine`, and `waypoint_manager`.
  - Implemented E220-900T30S Mode 3 register read/write driven by `settings.json` (defaulting to Channel 55 `905.125 MHz`, `57600`/`38.4k` air rate `REG0=0xC6`, `128B` packet size `REG1=0x43`, `21 dBm`, Transparent Mode, LBT disabled), plus zero-allocation RTCM3 (`0xD3`) frame sniffer.
  - Built 5-tile `320x480` Portrait LVGL 9 UI and companion Python scripts (`validate_waypoints.py v1.0.1`, `analyze_rtk_log.py v1.0.0`).
* **Current Step:** Firmware `v0.1.8` compiled cleanly and ready for flashing & hardware bring-up.
* **Next Target Milestone:**
  - Hardware verification of Display, Touch, SD card, AXP2101, E220 (`IO41/42`), and UM982 COM1 (`IO17/18`) & COM2 (`IO47/48`) using Tile 5 Test Modes.
