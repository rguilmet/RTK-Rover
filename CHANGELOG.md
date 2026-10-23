# Changelog

All notable changes to the **RTK Rover** project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
