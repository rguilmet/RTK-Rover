# Hyper-Context Anchor

## 1. CORE TECHNICAL ARCHITECTURE & STACK
* **Hardware:** Waveshare ESP32-S3-Touch-LCD-3.5B (`320x480` Portrait QSPI `AXS15231B` display, Capacitive Touch `0x3B`, `AXP2101` PMIC `0x34`, `TCA9554` IO Expander `0x20`, `PCF85063` RTC `0x51`, `QMI8658` IMU `0x6B`, `ES8311` Audio Codec `0x18`), Holybro H-RTK Unicore UM982 Dual-Antenna RTK GNSS module, and Ebyte E220-900T30S 900MHz LoRa module.
* **Firmware:** Dual-core FreeRTOS architecture (`Core 0` for real-time E220 $\rightarrow$ UM982 COM2 RTCM3 streaming, UM982 COM1 NMEA/`UNIHEADINGA` parsing, and 3D lever-arm kinematics with zero I2C access; `Core 1` for LVGL 9 rendering, exclusive `I2C_NUM_0` peripheral polling, queue-driven audio synthesis via `beepQueue`, and storage).
* **UI:** LVGL 9 (`320x480` Portrait) with a persistent 4-row Top Status Header Bar (Date/Time with `minutesOffsetToUTC`, SI/IMP badge, Wi-Fi badge, AXP2101 battery telemetry, Master Fix badge, Master SVs, accuracy, RTCM age, Slave Fix badge, Slave SVs, baseline length, true heading, and streamlined ground-tip coordinates) above 5 functional screens (Pin Seeker & Hybrid Guidance, Field Survey Logger, CSV Browser, Dual-Antenna Compass/Bubble Level, and Settings & Diagnostics) navigated strictly via the persistent Bottom Screen Navigation Bar (`< Prev`, `X/5: Title`, `Next >`). Horizontal swipe gestures are disabled.
* **Constraints & Kinematics:**
  - **Shared Internal I2C0 Bus (Core 1 Exclusive):** All internal SMD peripherals share `I2C_NUM_0` on `GPIO 7` (SCL) and `GPIO 8` (SDA) at `400 kHz`, polled exclusively from **Core 1** (`Touch 0x3B` @ 30Hz, `QMI8658 0x6B` @ 10Hz independent of LVGL graphics mutex, `PCF85063 0x51` @ 1Hz cached, `AXP2101 0x34` @ 0.5Hz) protected by `i2c_bus_mutex` with automatic `i2c_master_bus_reset()` recovery.
  - **Portrait IMU Coordinate Frame:** On the Waveshare 3.5B portrait LCD ($320 \times 480$), the QMI8658 accelerometer maps as: `raw_roll = atan2f(ay, -az)` (horizontal bubble motion $b_x$) and `raw_pitch = atan2f(ax, sqrtf(ay * ay + az * az))` (vertical bubble motion $b_y$). Resting flat produces $(0.0^\circ, 0.0^\circ)$.
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
* **NEVER** use non-ASCII Unicode characters (such as $\pm$, $\circ$, degree symbols, arrows) in UI strings; standard ASCII equivalents (`+/-`, `deg`, `->`) must be used to ensure font glyph compatibility.
* **NEVER** trigger waypoint logging from a short-press of the BOOT button (GPIO 0). Physical BOOT button is reserved exclusively for long-press ($\ge 1.5\text{ s}$) screenshots.

---

## 4. LOGISTICAL STATE & COMPLETED MILESTONES
* **Completed (`v0.2.9`):**
  - **Dual-Mode Hybrid Navigation Engine (Tile 0):** Dynamic Course-Up grid ($> 1.5\text{ m}$) with antenna beam vector, turn cues, dynamic auto-scaling, and 64-point breadcrumbs; centimeter bullseye mode ($\le 1.5\text{ m}$) with green ring lock audio cue.
  - **Persistent 4-Row Status Header:** Row 1 (Date, Time, Units, Wi-Fi, Battery), Row 2 (Master Antenna ANT1 fix, SVs, precision, RTCM age), Row 3 (Slave Antenna ANT2 fix, SVs, baseline, pitch, heading), Row 4 (Streamlined coordinates).
  - **Portrait Bubble Level Realignment (Tile 3):** Fixed axis swap: $a_y$ drives horizontal roll ($b_x$), $a_x$ drives vertical pitch ($b_y$). Tare bracket angle button zeroed.
  - **Settings & Diagnostics Tile (Tile 4):** Added stepped Backlight Brightness (`25%..100%`) and Audio Volume (`MUTE..100%`) controls with debounced `settings.json` persistence; added dedicated **Antenna & GNSS Diagnostics Card** displaying full Master and Slave metrics including **HDOP**.
  - **Gesture Stabilization:** Disabled horizontal swipe gestures across tiles; navigation strictly via bottom bar `< Prev` and `Next >` buttons.
  - **Wi-Fi HTML5 Web File Portal:** Wireless file manager (`http://<rover-ip>/`) for SD card and FFat with browser-based PNG conversion and NTP time sync.
  - **Dual Logging Subsystems:** Continuous Route track CSV logging (`/routes/`) and raw UM982 NMEA logging (`/nmea/`).
  - **Storage Auto-Routing & FFat Mirroring:** MicroSD primary, FFat fallback, automatic mirror of `settings.json`.
  - **Audio Queue:** FreeRTOS `beepQueue` synthesized on Core 1 for reliable multi-tone chirps.
* **Current Status:** Firmware `v0.2.9` active, compiled, committed, and ready for field operation.
