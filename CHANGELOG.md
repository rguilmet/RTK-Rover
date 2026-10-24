# Changelog

All notable changes to the **RTK Rover** project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v0.1.14] - 2026-10-24

### Changed
- Removed the duplicate `SVs: xx/xx` readout from the Latitude line on Tile 0 (`t0_pos_lbl` in `src/lvgl_port/lvgl_port.c`), keeping `SV:xx/xx` in Row 2 of the persistent Top Header Bar (`hdr_sats_acc_lbl`).

### Fixed
- Replaced the UTF-8 plus-minus character (`±` / `U+00B1`, which rendered as a missing-glyph box in LVGL's built-in ASCII `lv_font_montserrat_12` font) with standard ASCII `+/-` in `hdr_sats_acc_lbl` (`SV:xx/xx +/-1.8 cm`), and expanded `update_rover_ui()` format buffer to `256` bytes to silence `-Wformat-truncation=`.

## [v0.1.13] - 2026-10-24

### Added
- Added electrical GPIO pull-down line probes (`[E220 LINE PROBE]` on `IO41, IO42` and `[UM982 LINE PROBE]` on `IO17, IO18, IO47, IO48`) prior to `Serial.begin()` to detect which ESP32 pins have an active 3.3V UART TX idle-high signal connected versus floating/unconnected lines.
- Added 3-stage automatic MicroSD mount fallback in `RTK_Rover.ino` and `src/settings_manager/settings_manager.cpp`:
  - **Stage 1:** `SD_MMC` 1-bit (`CLK=11, CMD=10, D0=9`) with ESP32 internal pull-ups enabled (`gpio_pullup_en`) and `TCA9554 EXIO3` left as Input/High-Z (matching Waveshare `01_factory`).
  - **Stage 2:** `SD_MMC` 1-bit with `TCA9554 EXIO3` driven `LOW (0)`.
  - **Stage 3:** `SD` SPI mode (`SPI.begin(11, 9, 10, -1)`) with `TCA9554 EXIO3` driven `LOW (0)` as CS.

### Fixed
- Eliminated the remaining `IMU(0x6B) reg=0x33` -> `Touch(0x3B) err=0x103` I2C timeout pairs by:
  1. Serializing all Core 1 I2C polling (`um982_poll_pending_rtc_sync`, `imu_read`, `axp2101_bsp_read`) inside `rover_lvgl_lock()` in `task_display_and_audio()` so `rover_lvgl_port_task` (Priority 3) cannot preempt `task_display_and_audio` (Priority 2) mid-I2C-transfer.
  2. Unifying `I2C_NUM_0` bus speed to `400 kHz` (`scl_speed_hz = 400000`) across all devices and calling `i2c_master_bus_reset()` in `src/i2c_bsp/i2c_bsp.c` if any transfer fails so the bus never latches in `ESP_ERR_INVALID_STATE (0x103)`.
  3. Updating `imu_read()` in `src/imu_bsp/imu_bsp.c` to check `QMI8658_STATUS0 (0x2E)` first and read 6 accelerometer bytes from `QMI8658_AX_L (0x35)` (matching Waveshare's `bsp_qmi8658.c`).
- Updated `axp2101_bsp_init()` in `src/axp2101_bsp/axp2101_bsp.c` to configure and enable all AXP2101 LDOs (`ALDO1..4 = 3.3V`, `BLDO1 = 1.5V`, `BLDO2 = 2.8V`, `CPUSLDO = 1.0V`, `DLDO1..2 = 3.3V`) and DCDC rails (`DC1..5`) per Waveshare's `01_factory/components/esp_bsp/bsp_axp2101.cpp`.
- Updated `$xxGSV` Satellites in View (`sats_in_view`) parser in `src/um982_driver/um982_driver.cpp` (`update_gsv_max_band()`) to retain the maximum SV count across multi-band (`L1`/`L5`) GSV sentences within each 2.2-second window so `sats_in_view` is never overwritten by a lower secondary-band count.

## [v0.1.12] - 2026-10-24

### Added
- Enabled `GPGSV COM1 1` on the UM982 and added multi-constellation `$xxGSV` parsing (`$GPGSV`, `$GLGSV`, `$GAGSV`, `$GBGSV`, `$GQGSV`) in `src/um982_driver/um982_driver.cpp` to track **Satellites In View (`sats_in_view`)** alongside **Satellites Used (`num_sats`)**.
- Added `heading_is_fixed` (`NARROW_INT` vs `NARROW_FLOAT`) parsing from `#UNIHEADINGA` and displayed `SVs: Used/View` and Heading lock mode (`FIX` / `FLT`) in the Top Header Bar, at the top of Tile 0 (Main Screen), on Tile 3, and in the 5-second `[TELEM]` Serial output.

### Fixed
- Eliminated remaining `i2c.master: I2C transaction timeout detected` messages by moving 100% of `I2C_NUM_0` polling (`QMI8658` IMU at `10 Hz`, `PCF85063` RTC cached at `1 Hz`, `AXP2101` PMIC at `0.5 Hz`, and one-shot GNSS-to-RTC sync) onto **Core 1** (`task_display_and_audio`), leaving **Core 0** (`task_rover_core0`) with zero I2C transactions, and setting `I2C_NUM_0` bus clock to `100 kHz` standard mode (`scl_speed_hz = 100000`) with per-device `[I2C WARN]` diagnostics.
- Fixed MicroSD card `sdmmc_init_ocr: send_op_cond (1) returned 0x107` timeout by driving `TCA9554 EXIO3` (`CD/DAT3`) **HIGH (`1`)** prior to `SD_MMC.begin("/sdcard", true)` so the SD card enters native `SD_MMC` mode rather than SPI mode during `CMD0`.
- Updated `um982_run_powerup_selftest()` and `send_um982_com2_test_pulse()` to send `VERSIONA COM2\r\n` (Unicore target-port syntax) and added a `COM1 -> VERSIONA COM2` fallback check to isolate `IO47 (COM2 RX)` vs `IO48 (COM2 TX)`.
- Increased `e220_set_mode()` mode-switch guard delays (`100 ms` + `AUX=HIGH` + `60 ms`) and added a retry in `e220_read_config_registers()`.

## [v0.1.11] - 2026-10-24

### Added
- Added persistent Bottom Screen Navigation Bar (`320x32` with `< Prev`, `X/5: Screen Title`, and `Next >` buttons) visible across all 5 tiles so screens can be navigated via either touch buttons or horizontal swipes.
- Added comprehensive power-up hardware self-tests (`[I2C0 SCAN]`, `[PMIC]`, `[IMU]`, `[STORAGE]`, `e220_run_powerup_selftest()`, and `um982_run_powerup_selftest()` testing both `UM982 COM1` on `IO17/18` and `UM982 COM2` on `IO47/48` with `VERSIONA` commands) plus a 5-second periodic `[TELEM]` Serial status log.
- Added bidirectional `UM982 COM2` (`Serial2`, `RX=IO47 / TX=IO48`) response polling (`um982_poll_com2()`) and updated Tile 5's COM2 test button to send `VERSIONA\r\n` on `COM2` and display `COM2 RX/TX` byte counts and `PASS/WAIT` status.

### Fixed
- Fixed periodic `E (...) i2c.master: I2C transaction timeout detected` errors by adding a FreeRTOS Recursive Mutex (`i2c_bus_mutex`) in `src/i2c_bsp/i2c_bsp.c` to serialize Core 0 (`QMI8658` IMU, `AXP2101` PMIC) and Core 1 (`AXS15231B` Touch, `PCF85063` RTC) access to the shared `I2C_NUM_0` bus (`IO7/IO8`).
- Fixed `AXS15231B` touch controller polling in `TouchInputReadCallback` (`src/lvgl_port/lvgl_port.c`) to read 14 bytes (matching command length `0x0E` and Waveshare's `bsp_touch.c`), decode finger release event bits (`(buff[2] >> 6) == 0x01`), and preserve `last_x`/`last_y` coordinates on release so LVGL 9 swipe gestures and button clicks register reliably.
- Fixed MicroSD card detection on `ESP32-S3-Touch-LCD-3.5B` by switching from SPI `SD` to 1-bit `SD_MMC` mode (`SD_MMC.setPins(11, 10, 9)` + `SD_MMC.begin("/sdcard", true)` with `TCA9554 EXIO3` LOW, matching Waveshare's `07_sd_test.ino` and `bsp_sdcard.c`).
- Updated `axp2101_bsp.c` to enable `ALDO1..4` (`3.3V`) and disable `TS` thermistor pin ADC measurement (`ADC_EN = 0x0D`), and updated `imu_bsp.c` to set `QMI8658 CTRL1 = 0x40` (Little-Endian + auto-increment) per Waveshare's `3.5B` BSP.

## [v0.1.10] - 2026-10-24

### Fixed
- Resolved LVGL 9.6.0 deprecation warnings in `lv_conf.h` by replacing `LV_COLOR_DEPTH 16` with `LV_COLOR_FORMAT_DEFAULT LV_COLOR_FORMAT_RGB565` and `LV_ASSERT_HANDLER_INCLUDE` with `LV_ASSERT_CUSTOM_INCLUDE`.
- Enabled `LV_USE_FLOAT 1` and disabled unused `LV_BUILD_EXAMPLES` / `LV_BUILD_DEMOS` in `lv_conf.h`.
- Replaced deprecated `#include "SensorPCF85063.hpp"` with `#include "RtcDrv.hpp"` in `src/rtc_bsp/rtc_bsp.cpp`.

## [v0.1.9] - 2026-10-24

### Fixed
- Fixed Core 1 `LoadProhibited` crash inside `lv_vsnprintf_inner` / `lv_strnlen` (`update_rover_ui`) caused by `lv_label_set_text_fmt` ignoring `%f`/`double` arguments when `LV_USE_FLOAT` is disabled in the global Arduino `lv_conf.h`. Replaced all `lv_label_set_text_fmt` calls in `src/lvgl_port/lvgl_port.c` with standard C `snprintf()` + `lv_label_set_text()`.

## [v0.1.8] - 2026-10-24

### Fixed
- Removed unused `../touch/esp_lcd_touch.h` include and dead touch wrapper functions from `src/axs15231b/esp_lcd_axs15231b.c` and `esp_lcd_axs15231b.h` (`lvgl_port.c` polls `0x3B` directly via `i2c_bsp`).
- Added C/C++ header guards (`#ifdef __cplusplus`) around `<Arduino.h>` and `<FS.h>` in `src/settings_manager/settings_manager.h` and moved `SensorPCF85063.hpp` include from `rtc_bsp.h` into `rtc_bsp.cpp` so C translation units (`imu_bsp.c`, `lvgl_port.c`) compile cleanly.
- Added `#include "../tca9554/esp_io_expander_tca9554.h"` to `src/lvgl_port/lvgl_port.h`, `#include <stddef.h>` to `src/waypoint_manager/waypoint_manager.h`, and replaced `millis()` in `lvgl_port.c` with `esp_timer_get_time() / 1000ULL`.
- Renamed `rtc_init()` to `rtc_bsp_init()` to resolve linker collision with ESP-IDF's `libesp_hw_support.a(rtc_init.c.obj)`. Verified full `ESP32S3_DEV` compilation (`896,531 bytes` flash / `28%`, `41,252 bytes` RAM / `12%`).

## [v0.1.7] - 2026-10-23

### Changed
- Updated default E220-900T30S Transparent Mode UART baud rate to **`57600` bps (`8N1`)** (`REG0 = 0xC6` for `57600` UART + `38.4k` Air Data Rate) and added `"e220_uart_baud": 57600` to `settings.json`. Pacing the UART at `57600` allows `1,400-byte` multi-constellation RTCM3 bursts to pass cleanly without overrunning the E220's internal 1,000-byte FIFO buffer.

## [v0.1.6] - 2026-10-23

### Changed
- Updated default E220-900T30S sub-packet size to **`128 bytes`** (`REG1 = 0x43` for `128B` + `21 dBm` Level 3 TX power) alongside `38.4k` Air Data Rate (`REG0 = 0xE6`) and `115200` UART in `user_config.h`, `settings.json`, `settings_manager.cpp`, and `e220_driver.cpp`.

## [v0.1.5] - 2026-10-23

### Changed
- Updated default E220-900T30S Air Data Rate from `19.2k` (`0xE5`) to **`38.4k` (`0xE6`)** to accommodate multi-constellation RTCM3 MSM7 bursts while preserving +5 dB RF link margin for heavily wooded lots.
- Added E220 radio configuration keys (`e220_channel`, `e220_air_rate_kbps`, `e220_tx_power_dbm`, `e220_subpacket_bytes`, `e220_lbt_enable`) to `settings.json` and `settings_manager` so `38.4k` vs `62.5k` can be switched directly via `settings.json` and programmed from Tile 5 (`Write E220 Cfg`).

## [v0.1.4] - 2026-10-23

### Added
- Configured Ebyte E220-900T30S target radio parameters (`CH 55` / `905.125 MHz` to avoid EcoWitt Weather Station, `115200` UART, `19.2k` Air Data Rate, `200-byte` packet size, `21 dBm` Level 3 TX Power, Transparent Mode, LBT Disabled -> `REG0=0xE5`, `REG1=0x03`, `CHAN=0x37`, `REG3=0x00`).
- Added one-tap `"Write CH55 Cfg"` button on Tile 5 (Hardware Test Modes) alongside `"Read E220 Regs"`.

## [v0.1.3] - 2026-10-23

### Added
- Added `docs/implementation_plan.md` to the project documentation folder.

## [v0.1.2] - 2026-10-23

### Added
- Created master architectural documentation in `docs/project_overview.md` and session continuity log in `docs/session_handoff.md`.

## [v0.1.1] - 2026-10-23

### Changed
- Updated project license from MIT to **GNU General Public License v3.0 (GPLv3)** to match `Magnetic_Field_Scanner`.

## [v0.1.0] - 2026-10-23

### Added
- Initial project baseline for Waveshare ESP32-S3-Touch-LCD-3.5B, Holybro H-RTK Unicore UM982, and Ebyte E220-900T30S.
- License, `.gitignore`, `.gitattributes`, `platformio.ini`, `custom_large_partitions.csv`, and `settings.json`.
- Hardware BSP drivers for `ESP32-S3-Touch-LCD-3.5B`:
  - Shared I2C0 bus on `IO7` (SCL) and `IO8` (SDA).
  - `TCA9554` IO expander (`EXIO1` LCD_RST, `EXIO2` TP_INT, `EXIO3` SD_CS, `EXIO6` SYS_OUT, `EXIO7` PA_CTRL).
  - `AXP2101` PMIC battery voltage, percentage, and charging status driver.
  - `AXS15231B` QSPI `320x480` Portrait display and capacitive touch (`0x3B`) with LVGL 9.3.0.
  - `PCF85063` RTC with UTC minute offset and automatic GNSS UTC synchronization.
  - `QMI8658` 6-axis IMU for gravity vector pose detection and pole bubble level.
  - `ES8311` audio tone/beep generator and RGB565 BMP screenshot capture (long-press BOOT button on all screens).
- Modular architecture for `um982_driver` (`COM1` on `IO17/18`, `COM2` on `IO47/48`), `e220_driver` (`IO41/42`, `M0/M1/AUX/RST`), `nav_engine` (lever-arm kinematics & geodesic math), and `waypoint_manager` (CSV reader & BOOT-button survey logger on Tile 2).
