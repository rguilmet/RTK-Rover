# Hyper-Context Anchor

## 1. CORE TECHNICAL ARCHITECTURE & STACK
* **Hardware:** Waveshare ESP32-S3-Touch-LCD-3.5B (`320x480` Portrait QSPI `AXS15231B` display, Capacitive Touch `0x3B`, `AXP2101` PMIC `0x34`, `TCA9554` IO Expander `0x20`, `PCF85063` RTC `0x51`, `QMI8658` 6-DOF IMU `0x6B`, `ES8311` Audio Codec `0x18`), Holybro H-RTK Unicore UM982 Dual-Antenna RTK GNSS module, and Ebyte E220-900T30S 900MHz LoRa module.
* **Firmware:** Dual-core FreeRTOS architecture (`Core 0` for real-time E220 $\rightarrow$ UM982 COM2 RTCM3 streaming, UM982 COM1 NMEA/`UNIHEADINGA`/`BASEINFOA` parsing, and 3D lever-arm kinematics with zero I2C access; `Core 1` for LVGL 9 rendering, exclusive `I2C_NUM_0` peripheral polling, queue-driven audio synthesis via `beepQueue`, screenshot generation, and storage).
* **UI:** LVGL 9 (`320x480` Portrait) with a persistent 4-row Top Status Header Bar (Date/Time with `minutesOffsetToUTC`, SI/IMP badge, Wi-Fi badge, AXP2101 battery telemetry, Master Fix badge, Master SVs, accuracy, RTCM age, Slave Fix badge, Slave SVs, baseline length, true heading, and streamlined ground-tip coordinates in 14pt Montserrat) above 5 functional screens (Pin Seeker & Tactical Guidance, Field Survey Logger & Quick Settings, Multi-File CSV Browser, Dual-Antenna Compass/Bubble Level, and Hardware Diagnostics) navigated strictly via the persistent Bottom Screen Navigation Bar (`< Prev`, `X/5: Title`, `Next >`). Horizontal swipe gestures are disabled.
* **Constraints & Kinematics:**
  - **Shared Internal I2C0 Bus (Core 1 Exclusive):** All internal SMD peripherals share `I2C_NUM_0` on `GPIO 7` (SCL) and `GPIO 8` (SDA) at `400 kHz`, polled exclusively from **Core 1** (`Touch 0x3B` @ 30Hz, `QMI8658 0x6B` 12-byte atomic burst @ 10Hz independent of LVGL graphics mutex, `PCF85063 0x51` @ 1Hz cached, `AXP2101 0x34` @ 0.5Hz) protected by `i2c_bus_mutex` with automatic `i2c_master_bus_reset()` recovery.
  - **Portrait IMU Coordinate Frame & 6-DOF Telemetry:** Accelerometer $a_y$ drives horizontal roll ($b_x$), $a_x$ drives vertical pitch ($b_y$). Gyroscope measures angular rate at 64 LSB/(deg/s). Resting flat produces $(0.0^\circ, 0.0^\circ)$.
  - **Storage Auto-Routing & Settings Mirroring:** MicroSD card is primary (`/sdcard/`) with internal `FFat` as emergency fallback. On boot and whenever settings change, `settings.json` is automatically mirrored from SD card to `FFat`.
  - **Device-Relative UART Pin Labels:** All pin labels (`UM982_TXD0`, `UM982_RXD0`, `UM982_TXD1`, `UM982_RXD1`, `E220_TXD`, `E220_RXD`) are named relative to the external module (`<DEV>_TXD` = module output $\rightarrow$ ESP32 RX input; `<DEV>_RXD` = module input $\leftarrow$ ESP32 TX output).
  - **Dual-Antenna Lever-Arm & Pole Height Projection:** The UM982 reports the WGS84 position of the **Master Antenna**, while the survey pole centerline sits at the midpoint ($\frac{B}{2} = 0.2775\text{ m}$ for `0.555 m` spacing) between Master and Slave antennas at height `antenna_height_m` above the ground tip. `nav_engine` projects the Master position along the 3D `UNIHEADINGA` baseline vector (`heading`, `pitch`) to eliminate orbital offset when rotating the pole.
  - **Display Orientation Independence:** True heading comes strictly from the pole-mounted UM982 dual antennas, while the handheld Waveshare's `QMI8658` IMU tracks the enclosure gravity vector (`Held Flat`, `Viewing Angle`, `Clipped Vertically to Pole`) and supports bracket tilt taring for the 2D pole bubble level.
  - **E220-900T30S Radio Spec:** Channel `55` (`0x37` = `905.125 MHz` to avoid weather sensors), `57600` 8N1 UART (`e220_uart_baud`, paced so 1,400B bursts don't overrun the 1,000B FIFO), `38.4k` Air Data Rate default (`REG0=0xC6`), `128-byte` packet (`REG1=0x43`), Level 3 `21 dBm` power (`REG1=0x43`), Transparent Mode, LBT Disabled (`REG3=0x00`).
* **Code Style:** Strict C/C++ for firmware (`src/` modular subdirectories), pure Python for auxiliary scripts (`scripts/`). Maintain existing docstrings. Semantic versioning enforced (`ROVER_FIRMWARE_VERSION` in `user_config.h` for firmware).

---

## 2. BEHAVIORAL PROFILE & PROMPTING PERSUASION
* **Persona:** Professional Principal Level Embedded Systems Software Engineer & Geodesic/GNSS Firmware Architect.
* **Tone:** Highly terse, clinical, authoritative, and deeply rooted in physical hardware constraints (UART ringbuffer throughput, WGS84 ellipsoid radii, lever-arm kinematics, I2C bus arbitration). Zero conversational fluff. No preachy summaries.
* **Execution:** Do not output placeholder code (e.g., `// ... rest of function`). Provide complete, drop-in implementations.
* **Documentation Hygiene:** Explicitly update `docs/project_overview.md`, `docs/session_handoff.md`, and `CHANGELOG.md` in lockstep with architectural changes. Ensure all logical chunks of work are cleanly committed to Git.

---

## 3. HARD ROADBLOCKS & COMPULSIVE HALTS (WHAT NOT TO DO)
* **NEVER** use single-precision `float` for WGS84 Latitude or Longitude storage or geodesic math (`float` has only ~24 bits of mantissa $\approx 1.5\text{ m}$ resolution at mid-latitudes, completely destroying centimeter-level RTK precision). Always use 64-bit `double` (`%.8f`).
* **NEVER** wrap I2C sensor reads inside the LVGL graphics mutex `rover_lvgl_lock()`. Sensor polling runs on Core 1 independent of LVGL render frame times, protected by `i2c_bus_mutex`.
* **NEVER** touch LVGL widget pointers from outside `lv_timer_handler()` without acquiring `rover_lvgl_lock()` and releasing via `rover_lvgl_unlock()`.
* **NEVER** use `lv_label_set_text_fmt()` with floating-point specifiers (`%f`); because the global Arduino LVGL library compiles with `LV_USE_FLOAT = 0`, `lv_vsnprintf_inner` skips `%f` without advancing `va_arg(va, double)`, causing subsequent arguments to crash (`LoadProhibited`). Always format floats with standard C `snprintf(buf, sizeof(buf), ...)` followed by `lv_label_set_text()`.
* **NEVER** execute filesystem operations (such as scanning `/survey_logged_pins.csv`) inside the 30 Hz UI rendering update loop. Dynamic identifiers must be evaluated only on dirty state changes and served from cached strings.
* **NEVER** use non-ASCII Unicode characters (such as $\pm$, $\circ$, degree symbols, arrows) in UI strings; standard ASCII equivalents (`+/-`, `deg`, `->`) must be used to ensure font glyph compatibility.
* **NEVER** trigger waypoint logging from a short-press of the BOOT button (GPIO 0). Physical BOOT button is reserved exclusively for long-press ($\ge 1.5\text{ s}$) screenshots.

---

## 4. LOGISTICAL STATE & COMPLETED MILESTONES
* **Completed (`v0.2.38`):**
  - **Clean Prefix List & Unified Postfix:** Stripped trailing underscores from prefix list (`<DERIVED>`, `PIN`, `PIPE`, `MONUMENT`, `STAKE`, `CORNER`, `FENCE`, `TREE`, `UTILITY`, `WP`). Unified all modes to the universal `base + "_L" + %03u` format with defensive trailing underscore sanitation.
* **Completed (`v0.2.37`):**
  - **Tile 2 Field Survey Redesign:** Integrated `<DERIVED>` prefix mode at dropdown index 0, eliminating separate Target Match switch. Added collision-free numbering via cached `/survey_logged_pins.csv` scanning. Expanded `WAYPOINT_LABEL_LEN` to 36 bytes with smart delimiter-aligned stem truncation. Added circular marquee on ID row and restructured rows vertically.
* **Completed (`v0.2.36`):**
  - **ANT2 Coordinates & Base Station Telemetry Delivery Fix:** Direct parsing of ANT2 lat/lon from `$GNGGAH` with dual-antenna geodetic projection fallback. Fixed base telemetry state copy and configured UM982 COM1 to push `BASEINFOA` at 1 Hz. Changed MicroSD storage label color to Electric Cyan (`#22D3EE`).
* **Completed (`v0.2.35`):**
  - **Full 6-DOF IMU Telemetry:** Implemented 12-byte atomic burst read on QMI8658 registers `0x35..0x40` capturing 3-axis Accelerometer (g) and 3-axis Gyroscope (dps). Inserted dedicated Base Station telemetry card in sky blue (`#38BDF8`). Auto-wrapped all diagnostic labels to 300 px width. Fixed preprocessor `"Serial" redefined` warning.
* **Completed (`v0.2.34`):**
  - **Pinned Diagnostic Top Deck:** Fixed firmware header in Montserrat 16 White. Pinned dual-row interactive logging controls (`SERIAL`, `IMU`, `TELEM`, `TOUCH`) directly below header with static state change caching.
* **Completed (`v0.2.24`):**
  - **Screenshot Engine:** Long-press GPIO 0 ($\ge 1.5\text{ s}$) captures 24-bit uncompressed BMP screenshot directly to `/screenshots/screenshot_YYYYMMDD_HHMMSS.bmp` with audio beep confirmation.
* **Completed (`v0.2.20`):**
  - **Tactical Guidance Course-Up Canvas:** Rotating crossbar with green/amber antenna tips, green FOV wedge, LOS tracking vector, and 64-point breadcrumb trail.
* **Completed (`v0.2.18`):**
  - **Multi-File Waypoint Browser & Marquee:** Top bar file selector scanning `/waypoints.csv` and `/waypoints_*.csv` with circular marquee autoscroll and boot-time active file/index restoration.
* **Current Status:** Firmware `v0.2.38` active, verified, documented, and fully production ready.
