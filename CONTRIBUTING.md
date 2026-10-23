# Contributing to RTK Rover

Thank you for your interest in contributing to the RTK Rover project! 
This project is open-source under the MIT License, and we welcome contributions from the community to help improve high-precision RTK GNSS waypoint guidance and surveying.

## Code of Conduct

By participating in this project, you agree to abide by basic open-source community standards:
- Be respectful and constructive in issues and pull requests.
- Focus on the technical goals and engineering quality.

## How to Contribute

1. **Fork the Repository:** Create a personal fork on GitHub.
2. **Create a Branch:** Create a feature branch (`git checkout -b feature/your-feature-name`).
3. **Commit your Changes:** Follow semantic versioning and provide clear commit messages.
4. **Push to the Branch:** Push your changes to your fork.
5. **Open a Pull Request:** Submit a PR against the main branch with a detailed description of your changes.

## Development Setup

### Arduino IDE
- Install Arduino IDE 2.x
- Add ESP32 core via Board Manager (v3.0.0+ required).
- Select **ESP32S3 Dev Module**.
- Set the following critical board settings:
  - USB CDC On Boot: **Enabled**
  - PSRAM: **OPI PSRAM**
  - Flash Mode: **QIO 80MHz**
  - Flash Size: **16MB (128Mb)**
  - Partition Scheme: **16M Flash (3MB APP/9.9MB FATFS)** (or **Custom** using `custom_large_partitions.csv`)

### PlatformIO
A `platformio.ini` is provided for VSCode/PlatformIO users. It automatically resolves dependencies and fetches the correct ESP32 Arduino 3.x core environment.

## Coding Standards

- **RTOS Tasks:** Long-running operations MUST be placed in dedicated FreeRTOS tasks (Core 0 for UARTs/Sensors/SD, Core 1 for LVGL UI/Web).
- **LVGL Thread Safety:** Any interaction with the LVGL UI outside of `lv_timer_handler` must be wrapped in `rover_lvgl_lock()` and `rover_lvgl_unlock()`.
- **Heap Allocations:** Avoid dynamic memory allocation (`malloc`, `new`) inside hot paths (e.g. high-speed UART RTCM/NMEA loops). Use stack allocation or statically sized buffers to prevent fragmentation.
- **Versioning:** Any modification to code should bump `ROVER_FIRMWARE_VERSION` in `user_config.h` per SemVer rules (`vMajor.Minor.Patch`).

## Reporting Bugs

Please use the GitHub Issues tracker to report bugs. Include:
- Hardware revision
- Firmware version (found on the Web Server Portal or top header bar)
- Steps to reproduce
- Serial monitor logs if a crash occurs (Enable Serial Logging in settings).

Thank you for contributing to RTK Rover!
