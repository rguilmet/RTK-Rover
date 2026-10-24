# Changelog

All notable changes to the **RTK Rover** project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
