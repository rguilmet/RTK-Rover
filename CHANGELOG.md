# Changelog

All notable changes to the **RTK Rover** project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v0.2.58] - 2026-11-04

### Added
- **Boot-Time Hardware Pin Continuity Self-Test (`src/e220_driver/e220_driver.cpp`, `src/rover_api.h`):**
  - Integrated dynamic `M0` (IO39) and `M1` (IO40) continuity verification into `e220_run_powerup_selftest()`: toggles mode lines and confirms physical E220 $2..15\text{ ms}$ `AUX` mode-switch falling pulses.
  - Added `m0_wire_ok` and `m1_wire_ok` telemetry fields to `E220LinkStats_t` and surfaced `M0:OK M1:OK` (or red alert on open/floating wire) in Tile 4 Item 7 (`t4_e220_lbl`).

### Changed
- **Header Row 1 Wi-Fi Indicator Simplification (`src/lvgl_port/lvgl_port.c`):**
  - Replaced verbose text `LV_SYMBOL_WIFI " ON"` and `LV_SYMBOL_WIFI " OFF"` with icon-only `LV_SYMBOL_WIFI`, styled in Electric Cyan (`#22D3EE`) when active and Dim Slate (`#475569`) when off, freeing horizontal space for header date, time, and battery voltage.
- **Header Row 2 LoRa Signal Strength Prefix (`src/lvgl_port/lvgl_port.c`):**
  - Added explicit `"LoRa: "` prefix to Row 2 signal strength (`"LoRa: -85dB"`, `"LoRa: -- dB"`) with synchronized 6-tier color ladder.
- **Firmware Version Bump (`user_config.h`):**
  - Bumped `ROVER_FIRMWARE_VERSION` to `v0.2.58`.

## [v0.2.57] - 2026-11-04

### Changed
- **Calibrated Linear LoRa RF Meter Scale (`src/lvgl_port/lvgl_port.c`):**
  - Re-calibrated the Tile 4 live meter bar from `[-125 dBm .. -65 dBm]` to real-world operational bounds `[-110 dBm .. -25 dBm]` ($85\text{ dB}$ span), eliminating top-end saturation in the yard and deadband cutoff below ambient noise floor.
- **Multi-Tier Color Ladder for LoRa Signal Strength (`src/lvgl_port/lvgl_port.c`):**
  - Refined `get_lora_rssi_color()` with distinct color bands: Electric Cyan (`>= -45 dBm`, proximity/antenna test), Emerald Green (`-45 .. -70 dBm`, optimal solid field), Lime Green (`-70 .. -85 dBm`, nominal working field), Amber Yellow (`-85 .. -98 dBm`, fair / approaching fringe), Safety Orange (`-98 .. -106 dBm`, low margin / approaching noise floor), and Crimson Red (`< -106 dBm` or stale, critical/cutoff).
  - Synchronized color coding across Tile 4 meter bar, Tile 4 RSSI text, and Persistent Header Row 2.
- **Firmware Version Bump (`user_config.h`):**
  - Bumped `ROVER_FIRMWARE_VERSION` to `v0.2.57`.

## [v0.2.56] - 2026-11-04

### Added
- **Pinned LoRa RF Survey Card on Tile 4 (`src/lvgl_port/lvgl_port.c`):**
  - Added fixed $296\times 6\text{ px}$ signal strength bar (`lv_bar`), bold RSSI readout, ambient noise floor with SNR margin (`(+X dB)`), packet age in seconds, frame counter, drop counter, and RF channel/rate banner.
  - Simplified Persistent Header Row 2 to color-coded numeric signal strength (e.g. `"-104dB"` / `"-- dB"`).
- **Accelerated E220 Mode 0 Register Polling (`src/e220_driver/e220_driver.cpp`):**
  - Increased register query frequency to $900\text{ ms}$ during active reception and $2000\text{ ms}$ idle.

## [v0.2.55] - 2026-11-01

### Added
- **Hardware AUX Edge Counter & Telemetry Enhancements (`RTK_Rover.ino`, `src/e220_driver/e220_driver.cpp`):**
  - Integrated physical Mode 0 falling edge counter (`aux_fall_count`) and formatted RSSI noise/packet string (`Noise:%ddBm Pkt:%ddBm`) into live `[TELEM]` serial stream.
  - Expanded RTCM message breakdown in telemetry reporting individual counts for `1006`, `1074`, `1084`, `1094`, and `1230`, alongside truncation drops (`rtcm_trunc_drops`) and orphan bytes (`rtcm_orphan_bytes`).
- **Standardized ASCII Diagnostic Messaging (`RTK_Rover.ino`):**
  - Cleaned boot and diagnostic banners to strict ASCII `--` separators to preserve serial integrity and eliminate encoding mismatches.

### Changed
- **Firmware Semantic Version Bump (`user_config.h`):**
  - Bumped `ROVER_FIRMWARE_VERSION` to `v0.2.55`.

## [v0.2.54] - 2026-10-30

### Added
- **One-Time `/settings.json` E220 Radio Profile Auto-Migration (`ROVER_E220_PROFILE_REV = 2`) (`user_config.h`, `settings.json`, `src/settings_manager/settings_manager.h`, `src/settings_manager/settings_manager.cpp`):**
  - Added `e220_profile_rev` schema revisioning so existing SD card / `FFat` `/settings.json` files from `v0.2.53` (`CH 55 / 19.2 kbps`) automatically migrate once on first boot to Rev 2 (`115200 8N1`, `CH 75` = `925.125 MHz`, `38.4 kbps` Air, `200B` subpackets, `21 dBm` TX power) and persist cleanly to SD + `FFat` while preserving Wi-Fi credentials, antenna geometry, and subsequent user edits.
- **Full-Band Spectrum Report & CSV Exporter (`scripts/scan_e220_channels.py v1.1.0`):**
  - Upgraded passive Mode 0 RSSI scanner to report full per-channel statistics (`min`, `avg`, `median`, `p90`, `max`, `std` dBm, zone classification, and Ecowitt `915.0 MHz` offset) across all scanned channels and write plot-ready CSV files (`--csv`) with automatic `SUDO_UID`/`SUDO_GID` ownership restoration.

### Changed
- **Restored `38.4 kbps` Air Data Rate (`REG0 = 0xE6`) & Migrated RF Channel to `CH 75` (`925.125 MHz`) (`user_config.h`, `settings.json`, `scripts/e220_pacer.py v2.8.1`, `scripts/e220-pacer.service`, `scripts/setup_e220.py`):**
  - **16-Test Wire-Tap Audit (`_005`..`_022`):** Proved the `E220-900T30S` native `400-byte` buffer delivered `223` intact `> 200 B` RTCM3 frames across `38.4 kbps` / `62.5 kbps` runs (`_007`, `_009`, `_010_desk-test` with `110/166` intact `> 200 B` frames and `Fix: 4`, `_011_field-test`), versus **`0`** intact `> 200 B` frames across `8` consecutive `19.2 kbps` runs (`_013`..`_022`). Because the `E220-900T30S` (`LLCC68` + 1W PA) uses a passive `32 MHz` crystal (`XTAL`, no `TCXO`), `38.4 kbps` (`500 kHz` LoRa BW, `~37-45%` air duty cycle vs `69%` at `19.2k`) restores `2x-4x` carrier/thermal drift tolerance while preserving native atomic `400-byte` framing (`~ -123 dBm` sensitivity).
  - **Cleanest RF Channel Selection (`test/e220-scan.csv`):** After Base antenna relocation via 4-ft coax, migrated from `CH 55` (`905.125 MHz`) to **`CH 75` (`0x4B` = `925.125 MHz`)**, ranked `#1` cleanest US 902-928 MHz ISM channel (`-96.23 dBm` avg, `-94 dBm` peak, `+10.125 MHz` clear of `915.0 MHz` office Ecowitt weather telemetry).
  - **Aligned Target EEPROM Profile (Base & Rover):** `ADDH=0x00 ADDL=0x00 REG0=0xE6 REG1=0x23 CHAN=0x4B(75) REG3=0x00`.

### Fixed
- **Dual `4-Byte` / `5-Byte` Mode 0 RSSI Parser & Inter-Epoch Synchronized Polling (`src/e220_driver/e220_driver.cpp`, `scripts/e220_pacer.py`):**
  - Test `_022` raw wire-tap (`rtcm-base_2026-10-30_18-05-00-240.txt`) revealed the `E220-900T30S` MCU returns a `4-byte` Mode 0 RSSI reply (`0xC1 0x00 <noise> <pkt>`, omitting the `0x02` length byte; decoded `C1 00 A1 E3` = `-95 dBm` noise / `-29 dBm` packet and `C1 00 9E E0` = `-98 dBm` noise / `-32 dBm` packet, confirming ultra-clean Rover self-noise and close-range indoor `+21 dBm` TX + `+14 dB` RX LNA front-end compression at `~5 m`).
  - Updated `e220_poll_and_forward_rtcm()` (`80 ms` reply timeout, `260 ms` truncation timeout) and `probe_ambient_noise()` to parse both `4-byte` and `5-byte` replies unambiguously, and synchronized Rover Mode 0 RSSI queries to the post-`1094` inter-epoch RF quiet gap (`30..350 ms` after `1094`, plus `2.5 s` boot probe and `10 s` idle fallback) to eliminate mid-burst RF query collisions.
- **Hardware-Calibrated `E220-900T30S` Time-on-Air Model (`scripts/e220_pacer.py v2.8.1`):**
  - Calibrated `estimate_subpacket_air_s()` directly against Test `_022` hardware `AUX_low` measurements (`165.0 ms` constant `t_uart + AUX_low` across `208..322 B` -> `148.1 ms` true `200 B` Time-on-Air @ `19.2k`; `~0.68` net coded efficiency -> `82.2 ms` for `200 B`, `65.0 ms` for `144 B`, `58.3 ms` for `122 B` @ `38.4k`).

## [v0.2.53] - 2026-10-30

### Added
- **Rover Sliding-Window Store-and-Verify RTCM3 Gatekeeper (`src/e220_driver/e220_driver.cpp`, `src/rover_api.h`):**
  - Replaced raw byte-by-byte passthrough to UM982 COM2 with a 1,536-byte sliding-window staging gatekeeper (`rx_stage`).
  - Incoming E220 UART bytes are still recorded verbatim to `rtcm-base_*.txt` for raw forensic wire-tapping, while `um982_forward_rtcm_to_com2()` is called **atomically only after** a complete RTCM3 frame passes `0xD3` header validation and 24-bit `CRC24Q == 0`.
  - Prevents truncated head subpackets (`200B` of a `>200B` frame) from poisoning UM982 COM2's hardware parser and swallowing subsequent valid preambles (which previously destroyed 31% of intact frames in Test `_021-new-pacer`).
  - Performs 1-byte sliding resynchronization on CRC mismatches and 220 ms mid-frame truncation timeouts so intact frames arriving immediately behind a lost tail subpacket are rescued with zero loss.
  - Added granular per-message-type counters (`1006`, `1074`, `1084`, `1094`, `1230`), verified forwarded byte counter (`rtcm_bytes_forwarded`), truncation counter (`rtcm_trunc_drops`), and stripped byte counter (`rtcm_orphan_bytes`).
- **Rover Mode 0 AUX (`IO21`) Hardware Edge Counter ISR (`src/e220_driver/e220_driver.cpp`):**
  - Added `IRAM_ATTR` GPIO `CHANGE` interrupt on `ROVER_PIN_E220_AUX` counting physical Mode 0 falling edges (`aux_fall_count`) and measuring `AUX` LOW pulse width in microseconds (`aux_last_low_us`), gated automatically during Mode 3 register access and local RSSI register reads.
- **Live E220 Mode 0 Ambient Noise Floor & Last Packet RSSI (`user_config.h`, `src/e220_driver/e220_driver.cpp`, `src/lvgl_port/lvgl_port.c`, `RTK_Rover.ino`):**
  - Enabled `REG1` Bit 5 (`0x20` -> target `REG1 = 0x23`) on both Base and Rover E220-900T30S transceivers (`REG3 = 0x00`, keeping RX UART payload 100% untouched without appended bytes).
  - Non-blockingly polls `C0 C1 C2 C3 00 02` every 5 seconds when idle to report Channel 55 (`905.125 MHz`) **Ambient Noise Floor (`dBm`)** and **Last Packet RSSI (`dBm`)** on **Tile 4 (Hardware Diagnostics)** and in `[TELEM]` logs, with automatic zero-loss collision abort if a `0xD3` LoRa packet arrives simultaneously.

### Fixed
- **Base Station Datasheet Section 5.6.2 Two-Regime Pacer v2.7.0 (`scripts/e220_pacer.py`, `scripts/set_e220_power.py`):**
  - **Eliminated False 500ms Single-Subpacket Stalls:** Per E220 Manual Section 5.6.2, `AUX=1` indicates the 400-byte UART buffer emptied into the LLCC68 chip (not RF completion); frames `<= 200B` transfer into an idle LLCC68 FIFO in `<0.5 ms` without holding `AUX` LOW. Replaced the 500 ms `AUX` fall wait on `<= 200B` frames with calibrated LoRa Time-on-Air + 35 ms guard pacing (~130 ms total instead of 695 ms, eliminating the 102-second TCP backlog drift observed in Test `_021`).
  - **Eliminated Subpacket 2 Air Collisions on `201..400B` Frames:** On two-subpacket frames (`1074`, `1094`), `AUX` rises `HIGH` after ~109 ms the instant Subpacket 2 (`len - 200`) enters the LLCC68 FIFO and **begins** RF transmission. Added explicit `estimate_subpacket_air_s(len - 200) + guard_delay_s` wait after `AUX` rising edge so the next message never collides with Subpacket 2.
  - **Hardware Watchdog Reset (`GPIO 22`):** Added automatic 50 ms active-LOW reset pulse on `RST` (`GPIO 22`) whenever `AUX` remains stuck `LOW` for `> 1.5 s` (recovering from the 4.0s radio lockups observed in Test `_021`).
  - **Source RTCM3 `CRC24Q` Gate, Exclusive UART Lock & RF Noise Probe:** Added `TIOCEXCL` (`exclusive=True`) serial port locking, mandatory startup Mode 3 EEPROM verification (`REG1 = 0x23`, accepting `0xC0/0xC1` write acks), startup Channel 55 ambient noise floor probe (`C0 C1 C2 C3 00 01`), source `CRC24Q` validation (never transmitting non-RTCM or corrupt fragments), and stale TCP backlog shedding.
- **Rover Fallback Config Alignment (`user_config.h`, `settings.json`, `src/settings_manager/settings_manager.cpp`):**
  - Aligned fallback/default Rover E220 constants from legacy `57600 / 38.4k` to `115200 baud / 19.2 kbps / 200B / CH 55 / 21 dBm / REG1=0x23`.

## [v0.2.52] - 2026-10-30

### Fixed
- **Base Station RTCM Hardware Pacer v1.4.1 (`scripts/e220_pacer.py`):**
  - **AUX Falling Edge Window & Dwell Fallback:** Expanded post-write AUX falling edge poll window from 20 ms to 80 ms to cleanly accommodate the E220 internal UART idle overtime before keying transmission. Added automatic fallback air dwell time calculation (`calc_air_s = len * 8 / air_rate + 30ms`) in the event AUX does not assert LOW, guaranteeing that consecutive subpackets never collide in the E220 TX FIFO.
  - **Unconditional Per-Frame Journal Logging:** Removed `if self.verbose:` gate on `[TX RTCM]` logs; systemd journal now displays real-time frame types, chunk breakdowns, and individual subpacket wait durations by default (with optional `-q`/`--quiet` flag).
  - **Startup Version Banner:** Added unconditional startup banner outputting `e220_pacer v1.4.1`, hardware serial port, baud rate, RF channel, frequency (MHz), subpacket size, and guard delay.

## [v0.2.51] - 2026-10-30

### Added
- **Base Station RTCM Frame-Aware Pacer v1.4.0 (`scripts/e220_pacer.py`):**
  - Replaced blind 200-byte slicing with true **RTCM 3 Frame-Aware Packetization**:
    - Complete RTCM frames under 200 bytes (Msg 1084 @ 129B, Msg 1006 @ 27B, Msg 1230 @ 18B) are transmitted as **single, complete, intact LoRa packets**. Zero frame fragmentation or splitting.
    - RTCM frames exceeding 200 bytes (Msg 1074 @ ~229B, Msg 1094 @ ~283B) are dynamically split into $N = \lceil\text{len} / 200\rceil$ balanced subpackets (e.g. 115B + 114B, 142B + 141B). Neither subpacket reaches the 200-byte buffer limit, preventing internal module overflow.
  - Increased default inter-packet guard delay to `45 ms` (`--guard-delay 0.045`), providing a generous 32 ms margin for the SX1262 receiver to drain its 128-byte UART FIFO, reset internal registers, and lock onto the next packet's preamble.
  - Added per-frame transmission telemetry: `[TX RTCM] Type:1084 (129B) -> 1 pkt | Air+Wait: 61.2ms`.
  - Added non-RTCM fallback: if non-RTCM bytes or an incomplete frame sits in the buffer for >150-300 ms, the pacer flushes immediately so non-RTCM streams are never stranded.

### Fixed
- **Rover RTCM Sniffer Cascade Recovery (`src/e220_driver/e220_driver.cpp`):**
  - Reduced sniffer inter-byte recovery timeout from 3000 ms to **250 ms**: if a subpacket is lost over the air, the sniffer aborts the partial frame after 250 ms and resets to `RTCM_WAIT_PREAMBLE`, preventing subsequent valid epochs from having their `0xD3` preamble swallowed as payload bytes.
  - Added clean 15 ms burst consolidation: UART reads arriving within 15 ms (such as 120B FIFO threshold + 80B remainder) are coalesced into a single `[E220 RX BURST] 200 B (gap: ...)` log entry.

## [v0.2.50] - 2026-10-29

### Added
- **Base Station E220 Pacer v1.3.0 (`scripts/e220_pacer.py`):**
  - Added mandatory `INTER_PACKET_GUARD_S = 0.025` (25 ms) inter-packet guard delay to allow the receiver E220 to drain its 200-byte UART output (17.36 ms @ 115,200 baud) and settle its SX1262 LoRa core into RX continuous mode before the next packet's 2ms preamble arrives.
  - Immediate residual tail packet flushing (`if buffer: send_packet()`) eliminating inter-packet socket timeout stalling.
- **Rover RTCM Sniffer & Burst Instrumentation (`src/e220_driver/e220_driver.cpp`):**
  - Added real-time serial logging for RTCM frame status: `[RTCM OK]` with message type, payload length, and frame length, and `[RTCM CRC ERR]` with expected length and CRC value.
  - Added burst arrival gap tracking: `[E220 RX BURST] %d B (gap: %lu ms)`.

## [v0.2.49] - 2026-10-29

### Added
- **Base Station E220 Hardware Pacer Daemon (`scripts/e220_pacer.py v1.2.0`, `scripts/e220-pacer.service`):**
  - High-performance standalone Python hardware pacer running on Raspberry Pi 4B (`pi4b-rtkbase`) connecting RTKBase TCP stream (`127.0.0.1:5016`) to `/dev/ttyAMA0` @ 115,200 baud.
  - Automatically verifies and programs E220-900T30S EEPROM registers (`REG0=0xE5, REG1=0x03, CHAN=0x37, REG3=0x00`) via `--auto-config`.
  - Implements true closed-loop post-write AUX handshake: monitors physical AUX pin (BCM GPIO 27) for falling edge (RF transmit start) and rising edge (RF transmit complete over the air) plus 3ms guard delay, completely eliminating UART FIFO overflow and packet collision.
  - 100% byte conservation verified across 389,094 bytes and 2,416 packets in Test 014.
- **Dynamic Power Level Switching Utility (`scripts/set_e220_power.py v1.0.0`):**
  - Interactive/CLI tool to switch transmission power levels (21, 24, 27, 30 dBm) safely while managing the systemd service.

### Fixed
- **Web Portal Sort Button UTF-8 Mojibake Elimination (`src/web_server/web_server.cpp`):**
  - Removed all multi-byte Unicode emoji characters from portal sort button labels and table column headers, strictly adhering to the project's pure ASCII requirement.
  - Added explicit `<meta charset='utf-8'>` in HTML header and `Content-Type: text/html; charset=utf-8` in HTTP response headers to ensure clean cross-browser rendering without font-box or encoding artifacts.
  - Replaced unicode sort arrows with clean ASCII indicators (` [v]`, ` [^]`).

## [v0.2.48] - 2026-10-29

### Changed
- **E220 Default Configuration Set to 19,200 Baud, 38.4k Air Rate, 200B Subpackets (`user_config.h`, `settings.json`, `src/settings_manager/settings_manager.cpp`, `scripts/setup_e220.py`, `scripts/test_e220_link.py`, `docs/project_overview.md`):**
  - **UART Pacing (`baud = 19200`):** Paces serial data intake to $1,920\text{ bytes/s}$, guaranteeing that net on-air transmission throughput (~$2,400\text{ bytes/s}$ at 38.4k air rate) always exceeds the input rate. This eliminates E220 400-byte TX FIFO overflow during multi-constellation RTCM MSM4 bursts (e.g. ~650 bytes).
  - **RF Link Budget Optimization (`air-rate = 38.4 kbps`):** Improves receiver sensitivity by $+2.5\text{ to } +3.0\text{ dBm}$ compared to 62.5 kbps (achieving $\approx -123\text{ dBm}$ on SX1262 LoRa), extending reliable open-air range by up to $40\%$ and enhancing penetration through foliage at legal 21 dBm (125 mW) power.
  - **Packet Size (`packet-size = 200`):** Retains maximum 200-byte subpackets to prevent fragmentation of standard RTCM messages (e.g. GLONASS 1084 fits in 1 packet; GPS 1074 requires 2 packets instead of 3), reducing LoRa physical layer overhead and latency.
  - **Auto-Sync Target Registers:** Target register configuration is now `REG0 = 0x86` (19200 8N1, 38.4k air rate), `REG1 = 0x03` (200B subpacket, 21 dBm power), `CHAN = 0x37` (55 -> 905.125 MHz), `REG3 = 0x00` (Transparent mode, LBT disabled). Automatic boot-time self-test verifies and programs E220 EEPROM when register mismatch is detected.

## [v0.2.44] - 2026-10-29

### Changed
- **E220 Radio UART Baud Rate Aligned to 38,400 bps (`user_config.h`, `settings.json`, `src/settings_manager/settings_manager.cpp`, `scripts/setup_e220.py`, `scripts/test_e220_link.py`):**
  - Updated default E220 UART communication speed to 38,400 baud (`REG0 = 0xA7`, bits `[7:5] = 0b101`, bits `[2:0] = 0b111` for 62.5 kbps air rate).
  - Paces RTCM3 observation bursts entering the E220 transmitter from RTK-Base so the on-air transmission rate (62.5 kbps) exceeds the UART intake rate (38.4 kbps), completely preventing internal 400-byte FIFO buffer overflows and eliminating frame truncation on packets exceeding 200 bytes.
  - Automatically verifies and re-programs E220 configuration to `REG0 = 0xA7` during rover boot self-test.

## [v0.2.42] - 2026-10-28

### Added
- **NMEA Stream Logging Persistence Across Reboots (`src/settings_manager/`, `settings.json`, `src/lvgl_port/lvgl_port.c`, `RTK_Rover.ino`):**
  - Added `enable_nmea_logging` boolean to `SystemSettings` struct and `settings.json`.
  - Tile 2 `NMEA: ON/OFF` button toggle now marks settings dirty and commits to storage, persisting the state across power cycles.
  - Power-up sequence now inspects `current_settings.enable_nmea_logging` and automatically starts `nmea_logger_start()` if previously enabled.

### Changed
- **RTCM Sniffer Telemetry Non-Gating (`src/e220_driver/e220_driver.cpp`):**
  - Updated frame counting so that valid RTCM message types (1001..1300, 1005/1006, MSM) immediately increment `rtcm_frames_ok` and message type counters, while logging CRC mismatches independently without blocking telemetry.

### Fixed
- **E220 RTCM Sniffer Timeout and False Error Resync (`src/e220_driver/e220_driver.cpp`):**
  - **Eliminated False Errors from Premature 250ms Timeout:** Multi-packet LoRa RTCM3 bursts (split into 128-byte sub-packets) routinely have inter-packet or inter-epoch on-air gaps exceeding $250\text{ ms}$. Expanded the silence timeout to $3,000\text{ ms}$ (3.0 seconds) and removed spurious mid-stream error incrementing, preventing the sniffer from desyncing and falsely reporting 100% errors (`Err: 107`).
  - **Zero-Latency Serial0 FIFO Drain:** Replaced single-buffer read (`if (avail > 0)`) with a continuous `while ((avail = Serial0.available()) > 0)` drain loop, immediately forwarding full 2,100-byte bursts to UM982 COM2 without multi-tick queuing delay.
  - **Robust Preamble Hunting:** Removed false error flags in `RTCM_WAIT_LEN_H` and `RTCM_WAIT_LEN_L` when encountering non-preamble binary bytes during resynchronization, ensuring only genuinely received frames that fail CRC24Q or contain invalid message types increment the error counter.

### Changed
- **E220 Default Air Data Rate Upgraded to 62.5 kbps (`user_config.h`, `src/settings_manager/settings_manager.cpp`, `settings.json`):**
  - Updated default E220 LoRa air data rate from 38.4 kbps to 62.5 kbps (`REG0 = 0xC7`, bits `[2:0] = 0b111`).
  - Provides $7,812\text{ bytes/sec}$ raw on-air bandwidth, reducing airtime occupancy of 1 Hz MSM7 multi-constellation RTCM bursts to ~12% and preventing packet collisions or buffer backlog.
  - Automatically reprograms Rover E220 EEPROM to `0xC7` during power-up self-test if previous configuration was `0xC6`.

## [v0.2.39] - 2026-10-28

### Added
- **On-the-Fly RTCM CRC24Q Verification & Dropped Packet Detection (`src/e220_driver/e220_driver.cpp`, `src/rover_api.h`):**
  - Integrated streaming 24-bit Qualcomm CRC (`CRC24Q`, polynomial `0x1864CFB`) verification across all incoming RTCM3 frames. Valid frames with matching CRC increment `rtcm_frames_ok`.
  - Added mid-frame inter-byte timeout ($250\text{ ms}$) and CRC error detection to increment `rtcm_crc_errors` on dropped LoRa packets or corrupted frames.
  - Added `Err:%lu` count to periodic `[TELEM]` diagnostic serial output.

### Changed
- **Coordinated Dynamic UI Color Coding (`src/lvgl_port/lvgl_port.c`):**
  - **Precision Font Color Follows Fix Badge:** In Row 2, the `+/- %s` horizontal accuracy text font now dynamically tracks the Master Fix Badge color (`Green` on `RTK FIX`, `Orange` on `RTK FLT`, `Blue` on `DGPS`, `Amber` on `SINGLE`, `Red` on `NO FIX`), eliminating misleading green precision text during single-point fixes.
  - **Dynamic RTCM Freshness Indicator:** In Row 2, `hdr_m_rtcm_lbl` now tracks true differential age (`diff_age_s`):
    - $\le 2.0\text{s}$: Fresh **Green** (`#00FF88`).
    - $2.1\text{s} ??? 5.0\text{s}$: Warning **Orange** (`#FFA500`).
    - $> 5.0\text{s}$: Stale **Red** (`#FF4444`).
    - Syncing / Waiting: Displays `RTCM:Sync` (Orange) or `RTCM:Wait` (Red).
  - **Tile 4 E220 Packet Diagnostics:** Displays `OK:%lu Err:%lu`, highlighted in glowing Orange (`#FFA500`) whenever packet drops or CRC errors occur.
  - **Tile 4 Base Station Freshness:** Base Station telemetry card now color-codes by differential age ($\le 2\text{s}$ green, $\le 5\text{s}$ orange, $> 5\text{s}$ red).
- **Stale Differential Age Reset (`src/um982_driver/um982_driver.cpp`):**
  - Explicitly resets `gnss_state.diff_age_s = 0.0f` when `$GNGGA` field 13 is empty (Single fix or lost differential), preventing stale differential ages from latching in memory or route logs.

## [v0.2.38] - 2026-10-28

### Changed
- **Unified Location ID Prefix & Postfix Architecture (`src/waypoint_manager/waypoint_manager.cpp`):**
  - **Clean Prefix List Without Underscores:** Stripped trailing underscores from the prefix list to enhance UI clarity in the Tile 2 dropdown. The updated options are: `<DERIVED>`, `PIN`, `PIPE`, `MONUMENT`, `STAKE`, `CORNER`, `FENCE`, `TREE`, `UTILITY`, `WP`.
  - **Universal `_Lxxx` Suffix Generation:** Unified the stem generation logic across all modes. Both `<DERIVED>` (from active waypoint or `WP` fallback) and selected prefix options now universally follow the standard `base + "_L" + %03u` pattern (e.g. `PIN_L001`, `PIPE_L001`, `MONUMENT_L001`, `PIN_NW_01_L001`).
  - **Trailing Underscore Sanitation:** Added defensive sanitation to strip any trailing underscore from `base` prior to appending `_L`, guaranteeing clean identifiers without double underscores (e.g. `__L`).

## [v0.2.37] - 2026-10-28

### Added
- **Redesigned Tile 2 Field Survey / Waypoint Logger (`src/lvgl_port/lvgl_port.c`, `src/waypoint_manager/waypoint_manager.cpp`, `src/waypoint_manager/waypoint_manager.h`, `src/rover_api.h`):**
  - **`<DERIVED>` Location ID Prefix:** Replaced the separate "Match Active Target Name" switch with an integrated `<DERIVED>` option at index 0 of the Location ID Prefix dropdown.
  - **Collision-Free Numbering for Survey Points:**
    - In `<DERIVED>` mode, derives stem from the active navigation waypoint name with `_L` and scans `/survey_logged_pins.csv` for the next available 3-digit zero-padded index (e.g. `PIN_TREE_BASE_L001`, `PIN_TREE_BASE_L002`). If no waypoints are loaded, cleanly falls back to `WP_L001`.
    - In standard prefix mode (e.g. `MONUMENT_`), formats as `<prefix>Lxxx` (e.g. `MONUMENT_L001`), automatically finding the next unused sequence number.
  - **Label Capacity Expansion & Smart Stem Truncation:**
    - Expanded `WAYPOINT_LABEL_LEN` from 28 to 36 bytes in `src/rover_api.h` to fully support derived compound survey identifiers.
    - Implemented delimiter-aware stem truncation ($\le 30$ chars) on `_`, `-`, or space boundaries so `_Lxxx` is always preserved intact without text truncation.
  - **High-Performance Cached Label Generation:**
    - Cached survey label generation in `waypoint_format_next_survey_label()`: CSV scanning executes only when prefix selection changes, active waypoint changes, or a pin is logged. Zero filesystem I/O occurs during 30 Hz LVGL rendering cycles.
  - **Streamlined Vertical Layout & Marquee Support:**
    - Row 1 (`y = 8`): Symmetrical Location ID Prefix dropdown (`145x30`) & Averaging duration button (`145x30`).
    - Row 2 (`y = 44`): Quick Note Tag dropdown (`300x30`).
    - Row 3 (`y = 82`): Concise `"ID:"` row in Montserrat 14 `#FFCC00` with `LV_LABEL_LONG_SCROLL_CIRCULAR` marquee to smoothly handle long names.
    - Row 4 (`y = 108`): Multi-epoch progress bar (`300x8`).
    - Row 5 (`y = 124`): Large `LOG PIN NOW` button (`300x40`).
    - Row 6 (`y = 172`): Concise `"Logged:"` label and target CSV file path.
    - Row 7 (`y = 218`): Subtle horizontal divider (`308x1`, `#283B47`) cleanly demarcating Field Survey from Quick Settings (`y = 238..360`).

## [v0.2.36] - 2026-10-28

### Added
- **Antenna 2 (Slave) Coordinates (`src/rover_api.h`, `src/um982_driver/um982_driver.cpp`, `src/lvgl_port/lvgl_port.c`):**
  - Added `slave_lat` and `slave_lon` to `RoverUIData_t`.
  - Parsed latitude and longitude from NMEA `$GNGGAH` / `$GPGGAH` sentences in `handle_ggah_sentence()`.
  - Added fallback dual-antenna geodetic projection in `um982_get_latest_state()`: if secondary PVT has not solved independent lat/lon but carrier-phase dual-antenna heading/baseline is valid, ANT2 position is accurately computed along the measured 3D baseline vector ($\Delta N = L \cdot \cos(\theta_{\text{pitch}}) \cdot \cos(\theta_{\text{hdg}})$, $\Delta E = L \cdot \cos(\theta_{\text{pitch}}) \cdot \sin(\theta_{\text{hdg}})$).
  - Displayed live `Pos: Lat, Lon` and `Alt` under `[ANT2 SLAVE]` on Tile 5.
- **Base Station Telemetry Delivery Fix (`src/um982_driver/um982_driver.cpp`):**
  - Fixed state-passing in `um982_get_latest_state()`: explicitly copies `base_station_id`, `base_coords_valid`, `base_lat`, `base_lon`, `base_alt_m`, and `base_ecef_{x,y,z}` from `gnss_state` to `state_out`.
  - Configured UM982 COM1 to stream `BASEINFOA COM1 1` (1 Hz interval) so Base Station coordinates decoded from RTCM 1005 Station ARP are continuously pushed to the firmware and displayed.
  - Relaxed `handle_baseinfoa_sentence()` validation to check valid physical ECEF coordinates ($|X| > 1000\,\text{m}$, $|Y| > 1000\,\text{m}$) regardless of status flags.

### Fixed
- **Storage Row Contrast & Visual Distinction (`src/lvgl_port/lvgl_port.c`):**
  - Changed MicroSD storage status label color from Green (`#00FF88`) to Electric Cyan (`#22D3EE`). This eliminates visual confusion where the Storage row appeared merged into the green `[ANT1 MASTER]` block below it.

## [v0.2.35] - 2026-10-28

### Added
- **Full 6-DOF IMU Telemetry (`src/imu_bsp/imu_bsp.c`, `src/rover_api.h`, `RTK_Rover.ino`, `src/lvgl_port/lvgl_port.c`):**
  - **Single 12-Byte Atomic I2C Burst Read:** Expanded `imu_read()` from 6 bytes to 12 bytes starting at register `0x35` with address auto-increment. In one atomic bus transaction, reads Accelerometer X, Y, Z (`0x35..0x3A`) and Gyroscope X, Y, Z (`0x3B..0x40`) with zero additional I2C transactions or bus overhead.
  - **Calibrated Z-Axis & Gyroscope Units:** Accelerometer Z is converted at 8192 LSB/g (4g range); 3-axis Gyroscope (`gx`, `gy`, `gz`) is converted at 64 LSB/(deg/s) (512 dps full-scale range).
  - **Tile 5 Diagnostics Display:** `t4_imu_lbl` now reports live `Pitch` & `Roll` (??), 3-axis `Acc(g): X, Y, Z`, 3-axis `Gyr(d/s): X, Y, Z`, calibrated tare offsets, and lever-arm compensation status.
  - **Diagnostic IMU Serial Logging:** `[IMU]` and `[IMU POSE]` logs include live 3-axis acceleration (g) and angular velocity (??/s).
- **Dedicated Base Station Telemetry Section (`src/lvgl_port/lvgl_port.c`):**
  - Inserted dedicated `[BASE STATION]` observer block directly beneath `[ANT1 MASTER]` and `[ANT2 SLAVE]` in sky blue (`#38BDF8`).
  - Displays Base Station ID (`#<id>`), RTCM differential age (`Diff: X.Xs`), exact WGS84 base coordinates (`Pos: Lat, Lon`, `Alt`) decoded from RTCM 1005 / UM982 `#BASEINFOA`, and RTCM message counters (1005 ARP and MSM).

### Fixed
- **Resolved Compiler Warning `"Serial" redefined` (`src/storage/serial_logger.h`):**
  - Added `#ifdef Serial` / `#undef Serial` guard prior to `#define Serial SerialLogger` to cleanly suppress ESP32 Arduino core 3.x preprocessor warnings when USB CDC-on-boot (`-DARDUINO_USB_CDC_ON_BOOT=1`) is enabled.
- **Multi-Line Auto-Wrapping on Diagnostic Scroll Container (`src/lvgl_port/lvgl_port.c`):**
  - Replaced `LV_LABEL_LONG_CLIP` with `LV_LABEL_LONG_WRAP` across all 12 telemetry sections in `t4_scroll_cont`.
  - Set fixed 300 px width on all labels inside the flex column (`LV_FLEX_FLOW_COLUMN`). Text lines exceeding 300 px now wrap to subsequent lines dynamically, preventing right-edge truncation while LVGL flex flow adjusts block heights and vertical scrolling smoothly.

## [v0.2.34] - 2026-10-28

### Added
- **Pinned Top Control Deck on Tile 4 (`src/lvgl_port/lvgl_port.c`):**
  - **Fixed Firmware Header:** Pinned `"RTK ROVER v0.2.34"` at `y = 4` in **Montserrat 16 White (`#FFFFFF`)**; uncoupled from scrolling so it is permanently visible.
  - **Dual-Row Logging Controls:** Pinned 4 interactive log toggle buttons (`SERIAL`, `IMU`, `TELEM`, `TOUCH`) directly below the header. Buttons remain accessible at all times regardless of viewport scroll position, allowing real-time toggling while watching live telemetry counters.
  - **Static State Caching:** Integrated the 4 diagnostic buttons into `update_all_control_buttons()` with static change caching to guarantee zero style/label churn during the 30 Hz UI update loop.
  - **Acoustic Feedback & Extended Touch:** Integrated 4 px extended click padding and distinct audio chirp confirmation (1800 Hz ON, 1200 Hz OFF) for each log toggle.
- **Enhanced Diagnostic Telemetry Observers (`src/lvgl_port/lvgl_port.c`):**
  - **`[POWER / PMIC AXP2101]`:** Live battery voltage (`VBAT`), percentage (`%`), USB VBUS connection (`YES`/`NO`), and charging status (`YES`/`NO`).
  - **`[IMU & POLE TILT]`:** 250 Hz live pitch & roll angles, calibrated tare offsets, and lever-arm compensation status (`[ON]` / `[OFF]`).
  - **`[WIFI & WEB PORTAL]`:** Configured SSID, live connection state, and active Web Portal URL (`http://<ip>/`).
  - **`[I2C0 BUS STATUS]`:** Health status verification across all 6 onboard I2C peripherals (`TCA9554`, `AXP2101`, `Touch`, `RTC`, `IMU`, `ES8311`).

### Fixed
- **Tile 4 Layout & Modernization (`src/lvgl_port/lvgl_port.c`):**
  - **LVGL Flex Flow Layout:** Refactored `t4_scroll_cont` to use `LV_FLEX_FLOW_COLUMN` with strict 10 px row padding (`pad_row = 10`), standardizing spacing across all telemetry blocks and eliminating dead blank spaces.
  - **Legacy Button Removal & Clean-Up:** Purged all 5 legacy test buttons (`COM1 VERSIONA`, `COM1 CONFIG`, `COM2 Send VERSIONA`, `Read E220 Regs`, `Write E220 Cfg`) and deleted their obsolete UI callback functions.

## [v0.2.33] - 2026-10-28

### Added
- **Remember & Restore Last Selected Waypoint Index (`src/waypoint_manager/waypoint_manager.cpp`, `src/settings_manager/settings_manager.cpp`):**
  - Added `active_waypoint_index` to `SystemSettings` and `/settings.json`.
  - Automatically persists the active waypoint index whenever changed via Tile 0 `<` / `>` buttons or Tile 2 CSV table clicks.
  - Automatically restores `g_active_waypoint_idx` on boot during `waypoint_manager_init()` and safely clamps to 0 if out of bounds or when switching CSV files.
  - Emits serial diagnostic `[WAYPOINTS] Restored active waypoint index: X/Y ('LABEL')` on startup.

### Fixed
- **Circular Marquee Scrolling for Waypoint & File Selectors (`src/lvgl_port/lvgl_port.c`):**
  - **Bounded Control Widths:** Bounded both `t0_target_lbl` (Tile 0 Waypoint Selector) and `t2_file_lbl` (Tile 2 File Selector) to a strict 216 px width between the `<` and `>` buttons with 2 px margins on both sides.
  - **Marquee Autoscroll:** Configured `lv_label_set_long_mode(..., LV_LABEL_LONG_MODE_SCROLL_CIRCULAR)` on both labels. When waypoint labels or CSV filenames exceed 216 px, text smoothly scrolls in a continuous circular loop instead of spreading over the interactive controls.
  - **Centered Shorter Text:** Applied `lv_obj_set_style_text_align(..., LV_TEXT_ALIGN_CENTER, 0)` so standard-length labels remain centered between controls.
  - **Animation Reset Protection:** Added `strcmp` guard checks in `update_t0_target_label()` and `refresh_waypoint_table_ui()` so marquee scroll animations are not restarted on identical text updates.
  - **Immediate Tile 0 Refresh on File Switch / Table Tap:** Synchronously calls `update_t0_target_label()` on CSV file changes and table row clicks.

## [v0.2.32] - 2026-10-28

### Fixed
- **Storage Safeguards & Wi-Fi Credential Protection (`src/settings_manager/settings_manager.cpp`):**
  - **Eliminated Destructive Auto-Save on Read Failure:** Removed `save_settings()` inside `load_settings()` when `/settings.json` could not be opened. Missing or transiently busy storage now safely uses in-memory defaults without writing to or modifying the filesystem.
  - **Multi-Tier Fallback Hierarchy:** Added sequential fallback checking in `load_settings()`: tries primary `/settings.json`, performs a 50 ms retry if SD card was recently mounted, checks active storage `/settings.json.bak`, and checks FFat internal flash mirror `/settings.json` and `/settings.json.bak`.
  - **Read-Merge-Write Wi-Fi Protection:** Implemented credential preservation in `save_settings()`: if in-memory `wifi_ssid` contains a default placeholder (`"Your_SSID"`, `"Your-SSID-Here"`, or empty), `settings_manager` scans active disk and backup files for valid, non-placeholder credentials and preserves them. Real user credentials can never be overwritten by default template values.
  - **Automatic Rolling Backups (`.bak`):** Any write to `/settings.json` (on both SD card and FFat) automatically copies the existing valid configuration to `/settings.json.bak` before saving.
  - **Thread-Safe Mutex Locking:** Enforced `fs_mux` mutex acquisition around all storage read, parse, backup, and write operations in `settings_manager`.

## [v0.2.31] - 2026-10-28

### Added
- **Restored Acoustic Touch Confirmation (Audio Beeps):**
  - Integrated immediate audio chirp feedback on all interactive touch buttons:
    - Screen navigation (`< Prev` / `Next >`): 1200 Hz, 20 ms.
    - Waypoint selection (`<` / `>`): 1500 Hz, 20 ms.
    - Waypoint file selection (`<` / `>`): 1500 Hz, 20 ms.
    - Route recording toggle (`REC ON/OFF`): 1800 Hz, 40 ms on Start, 1200 Hz, 30 ms on Stop.
    - Waypoint table selection: 1500 Hz, 20 ms.
    - Survey Log Pin: 1750 Hz, 40 ms.
    - Quick controls (Wi-Fi, NMEA, Serial Log, Brightness, Volume, Units, Tare): distinct 1400???1800 Hz tones.
- **Instrumented Real-Time Touch Diagnostics to Serial Monitor:**
  - Added `enable_touch_logging` setting to `RoverSettings_t` and `settings.json` (defaults to `true`).
  - Emits real-time diagnostic logs on touch transitions: `[TOUCH DOWN]`, `[TOUCH HELD]` (every 250 ms), and `[TOUCH UP]` reporting raw controller bytes (`buff[0..5]`), touch coordinates (`X, Y`), event flags, and active touch points.
  - Emits `[TOUCH BTN]` button click logs with targeted button name and action.
  - Logs `[TOUCH REJECTED: buff[0]==0xFF]` if a touch point is ever present while gesture byte indicates idle, enabling definitive hardware diagnostics.
- **Instant Waypoint Target Label Refresh:**
  - Added `update_t0_target_label()` called synchronously inside `prev_wp_btn_cb` and `next_wp_btn_cb` so the waypoint name and index change on screen immediately upon touch without waiting for Core 0's navigation frame cycle.

### Fixed
- **Fixed Button Click Interruption from 30 Hz UI Style Churn (`src/lvgl_port/lvgl_port.c`):**
  - `update_all_control_buttons()` previously called `lv_label_set_text()` and `lv_obj_set_style_bg_color()` unconditionally 30 times a second on `t0_rec_btn`, `t1_unit_btn`, `t1_wifi_btn`, `t1_nmea_btn`, and `t1_serial_log_btn`. Calling styling and text APIs while a user's finger is actively pressing a button dirtied the style cache and reset LVGL's internal indev click state.
  - Added static state caching across all control buttons so LVGL styling and label APIs are only invoked when a setting's value actually changes.
- **Expanded Hitboxes & Added Extended Click Areas (`src/lvgl_port/lvgl_port.c`):**
  - Enlarged Tile 0 waypoint selector buttons (`btn_prev`, `btn_next`) to 44x28 with `lv_obj_set_ext_click_area(btn, 8)` (creating a generous 60x44 px touch target).
  - Enlarged `t0_rec_btn` to 80x26 with `lv_obj_set_ext_click_area(t0_rec_btn, 6)`.
  - Enlarged bottom navigation bar buttons (`btn_scr_prev`, `btn_scr_next`) to 68x28 with 6 px extended click padding.
  - Enlarged CSV file selector buttons (`btn_file_prev`, `btn_file_next`) to 44x28 with 8 px extended click padding.
  - Added 4 px extended click padding to all Survey Screen quick toggle buttons.

## [v0.2.30] - 2026-10-28

### Fixed
- **Restored Proven v0.2.24 Touch, Navigation & I2C Baseline:**
  - **Restored Exact `TouchInputReadCallback` (`src/lvgl_port/lvgl_port.c`):** Reverted the experimental touch filter modifications back to the exact working implementation from `v0.2.24`. The touch controller's `buff[0] != 0xFF` and `event_flag != 0x01` checks are critical for detecting finger release on the AXS15231B; removing them caused the controller to appear permanently pressed at the coordinates of the last tap, triggering rapid-fire runaway screen changes.
  - **Restored Exact Button Configurations & Geometry (`src/lvgl_port/lvgl_port.c`):** Reverted `< Prev` and `Next >` navigation buttons, waypoint buttons, and CSV file buttons to their exact `v0.2.24` specifications.
  - **Restored Exact Task & Bus Timing (`user_config.h`, `RTK_Rover.ino`, `src/i2c_bsp/i2c_bsp.c`):** Restored `LVGL_TASK_MAX_DELAY_MS = 500`, `scl_wait_us = 20000`, `i2c_data_pdMS_TICKS = 25`, and removed intermediate task delays.
  - **Preserved Only Surgical Wi-Fi Crash & Boot Fixes:** Retained strictly the two isolated fixes that prevent the `CORRUPT HEAP` Wi-Fi crash and preserve USB CDC power-up diagnostics:
    1. Decoupled Core 0 Wi-Fi status update via static buffer (`s_pending_wifi_status`) applied strictly on Core 1 inside `update_rover_ui()`.
    2. Non-blocking direct `WiFi.begin()` association in `src/web_server/web_server.cpp` with `s_wifi_starting` re-entrancy guard.
    3. Proper `operator bool()` in `src/storage/serial_logger.h` for boot diagnostics over USB CDC.

## [v0.2.29] - 2026-10-27

### Fixed
- **Fixed Spotty Single Taps & Restored Instant Touch Response (`src/lvgl_port/lvgl_port.c`):**
  - **Removed Erroneous `buff[0] != 0xFF` Filter:** On the AXS15231B touch controller, `buff[0]` is the hardware gesture ID, which returns `0xFF` (no gesture detected) during normal, stationary single-finger taps. The check `buff[0] != 0xFF` was discarding single taps that were not swipes/drags. Relying strictly on active touch point count (`buff[1] >= 1 && buff[1] <= 2`) allows instant single-press recognition.
  - **Removed `event_flag != 0x01` Filter:** Prevented premature rejection of release frames when fingers lift off the glass, ensuring clean `LV_INDEV_STATE_RELEASED` events at the exact release coordinates.
  - **Disabled Scrolling on Bottom Navigation Bar:** Added `lv_obj_remove_flag(nav_bar, LV_OBJ_FLAG_SCROLLABLE)` so minor finger micro-movements during a tap cannot trigger container drag tracking or cancel the click event.
  - **Enlarged Touch Target & Added Extended Click Padding:** Increased `< Prev` and `Next >` navigation bar buttons to 68x28 with `lv_obj_set_ext_click_area(btn, 6)`, expanding the touch sensitivity area by 6 pixels on all edges. Also padded the `<` and `>` waypoint and file selector buttons on Tiles 0 and 2.
  - **Added Audio Click Feedback:** Integrated immediate 20 ms sound feedback on screen change (`play_rover_beep(1200, 20)`) and waypoint/file selection (`play_rover_beep(1500, 20)`).
  - **Clamped LVGL Task Max Latency (`user_config.h`):** Reduced `LVGL_TASK_MAX_DELAY_MS` from 500 ms to 30 ms so the FreeRTOS LVGL task polls touch at least ~33 Hz even when no animations are active, eliminating idle sleep lag.

## [v0.2.28] - 2026-10-27

### Fixed
- **Restored Complete I2C Hardware Peripheral Control (`src/i2c_bsp/i2c_bsp.c`):**
  - **Removed Manual `gpio_set_direction` on I2C Lines:** Removed the experimental `i2c_bus_clear_stuck_sda()` routine. In ESP-IDF v5, calling `gpio_set_direction()` directly on pins attached to the `i2c_master` peripheral disconnects the GPIO matrix routing from the hardware controller, permanently breaking the SCL line upon the first timeout and locking up the bus.
  - **Native Peripheral Hardware Bus Reset:** Bus recovery now delegates cleanly to `i2c_master_bus_reset(user_i2c_port0_handle)`, which safely clocks the hardware bus without corrupting peripheral GPIO matrix assignments.
  - **Retained Safe Transfer Headroom:** Maintained `i2c_data_pdMS_TICKS = 100` and `dev_cfg.scl_wait_us = 0` (ESP-IDF default) with rate-limited warning logs.
- **Restored Responsive Native Touch Polling (`src/lvgl_port/lvgl_port.c`):**
  - **Reverted Synthetic Polling Throttling:** Restored `TouchInputReadCallback` to the proven `v0.2.24` implementation. Removed the artificial 16 ms throttle and `last_state` caching that interfered with LVGL's internal event debouncing, click timers, and gesture recognizers.
  - **Instant Button & Swipe Response:** Touch input is sampled directly on LVGL's natural timer schedule, restoring instant single-tap recognition and smooth screen swipe navigation across all tiles.
  - **Guaranteed Wi-Fi Thread Safety:** Kept the decoupled `s_pending_wifi_status` / `s_wifi_status_dirty` buffer architecture, ensuring Core 0 Wi-Fi callbacks never touch LVGL widgets or heap structures.

## [v0.2.27] - 2026-10-27

### Fixed
- **Resolved I2C Bus Timeouts & Cascade Resets (`src/i2c_bsp/i2c_bsp.c`):**
  - **Increased Transfer Timeout:** Raised `i2c_data_pdMS_TICKS` from an aggressive 25 ms to a safe 100 ms. Normal transfers take < 1 ms, preventing premature software timeout aborts while the hardware FSM is actively clocking.
  - **Removed Artificial Clock Stretch Constraint:** Removed `scl_wait_us = 20000` to use standard ESP-IDF defaults, preventing false `I2C bus is still busy but software timeout detected` errors.
  - **Hardware 9-Clock Bus Clear on Recovery (`i2c_bus_clear_stuck_sda`):** Implemented an open-drain 9-clock SCL pulse recovery routine. When a slave device is interrupted mid-byte and pins SDA low, 9 clock pulses allow the slave to finish shifting out and release SDA before resetting the master peripheral.
  - **Rate-Limited I2C Warning Logs:** Throttled `[I2C WARN]` serial prints to at most once per second to prevent serial output flooding during bus recovery.
- **Enhanced Touch Screen Polling & State Stability (`src/lvgl_port/lvgl_port.c`):**
  - **Touch Polling Throttled to ~60 Hz (16 ms):** Prevented the LVGL task from pounding the shared I2C bus every 5 ms (200 Hz), reducing total I2C bus bandwidth consumption by over 70% and completely eliminating bus starvation with the IMU and PMIC.
  - **Preserved Touch State Across Glitches:** When a transient I2C timeout occurs, `TouchInputReadCallback` now preserves `last_state` (`PRESSED`) instead of erroneously asserting `RELEASED`. This prevents dropped button clicks and allows smooth, unbroken finger drags and screen swipe navigation.

## [v0.2.26] - 2026-10-27

### Fixed
- **Restored Power-Up Boot Messages & USB CDC Detection (`src/storage/serial_logger.h`):** Corrected `SerialLoggerClass::operator bool()` to return `(bool)::Serial` instead of unconditionally returning `true`. This restores the initial 3-second USB CDC connection wait during `setup()`, ensuring the entire startup banner, I2C bus scan, PMIC status, IMU self-test, and storage mounting diagnostics are fully rendered to the Serial Monitor.
- **Eliminated UI Freeze & Restored Touch Responsiveness (`src/lvgl_port/lvgl_port.c`):**
  - **Reverted LVGL Mutex to Standard FreeRTOS Mutex:** Restored `lvgl_mux` to a standard mutex (`xSemaphoreCreateMutex()`) with standard priority inheritance.
  - **Zero-Contention Wi-Fi Status Updates:** Decoupled Core 0 Wi-Fi status changes from the LVGL widget tree using a thread-safe pending status buffer (`s_pending_wifi_status` / `s_wifi_status_dirty`). Status changes are now applied exclusively from Core 1 inside `update_rover_ui()`, preventing cross-core mutex contention and FreeRTOS task starvation.
  - **Correct Initialization Order:** Guaranteed `create_rover_ui()` runs and completes before `rover_lvgl_port_task` is scheduled, preventing race conditions against uninitialized UI structures.
  - **Cooperative Time-Slicing on Core 1 (`RTK_Rover.ino`):** Added a 5ms delay in `task_display_and_audio` to yield CPU cycles to `rover_lvgl_port_task`, guaranteeing smooth touch gesture responsiveness.
- **Removed Dangerous `esp_log_set_vprintf` Hook (`src/storage/serial_logger.cpp`):** Removed the global ESP-IDF C-runtime `vprintf` hook that intercepted internal system logs and created re-entrancy risks in USB CDC (`HWCDC`) and file drivers. `SerialLogger` operates purely through standard Arduino `Print`/`Stream` redirection.

## [v0.2.25] - 2026-10-27

### Fixed
- **Resolved Heap Corruption & Crash on Wi-Fi Start:** Fixed a multi-heap corruption assert (`multi_heap_free: Bad head ... assert failed: (head != NULL)`) triggered when pressing "Wi-Fi: Start".
  - **Thread-Safe LVGL Locking:** Converted `lvgl_mux` to a FreeRTOS recursive mutex (`xSemaphoreCreateRecursiveMutex()`) in `src/lvgl_port/lvgl_port.c` and wrapped `update_wifi_ip_label()` in `rover_lvgl_lock(500)`. This prevents Core 0's background Wi-Fi connection task and Core 1's LVGL rendering task from concurrently modifying and freeing widget memory.
  - **Connection Task Re-entrancy Guard:** Added `s_wifi_starting` state flag in `src/web_server/web_server.cpp` to prevent rapid button double-clicks from launching duplicate concurrent Wi-Fi connection tasks.
  - **Streamlined Direct AP Association:** Replaced the heavy, synchronous pre-scan `WiFi.scanNetworks()` with direct `WiFi.begin(ssid, password)` using `WIFI_ALL_CHANNEL_SCAN` and `WIFI_CONNECT_AP_BY_SIGNAL`. Eliminates dynamic heap scan buffer allocations and races with Wi-Fi driver deinitialization while accelerating connection time.

## [v0.2.24] - 2026-10-27

### Added
- **Serial Monitor SD Card Logger (`src/storage/serial_logger.h`, `src/storage/serial_logger.cpp`):** Implemented non-blocking asynchronous Serial logging to the SD Card under `/serial/serial-log_<timestamp>.txt`. Features a 32 KB ring buffer with early power-up buffering so no boot diagnostics are lost. Disconnecting USB on battery power does not stall or lose telemetry diagnostics.
- **Dedicated Screenshots Directory (`src/screenshot/screenshot.cpp`):** Redirected full-screen RGB565 BMP captures into a dedicated `/screenshots/` folder (`/screenshots/screenshot_<timestamp>.bmp`) with automatic fallback to root.
- **Screen 2 `LOG: ON` / `LOG: OFF` Quick Settings Button (`src/lvgl_port/lvgl_port.c`):** Replaced redundant `Route: ON/OFF` button on Screen 2 ("2/5: Survey & Settings") with an interactive `LOG: ON` (Green) / `LOG: OFF` (Blue) button. Route recording remains accessible on Screen 1 ("1/5: Pin Seeker" via `[REC ON]/[REC OFF]`). Toggling this button starts/stops SD serial logging and persists state to `settings.json`.

### Changed
- **Uncoupled Hardware Serial from SD Logging Flag:** Hardware USB Serial transmission continues uninterrupted regardless of whether SD logging is ON or OFF. Periodic telemetry and IMU logs continue streaming to USB Serial and are mirrored to SD when enabled.

## [v0.2.23] - 2026-10-27

### Changed
- **NMEA Stream Logger Filename Standardized to `.txt`:** Simplified NMEA stream logging in `src/storage/nmea_logger.cpp` by replacing `.log` with `.txt` and setting filename prefix to `nema-log_<timestamp>.txt` (e.g. `/nmea/nema-log_2026-10-27_17-25-30-240.txt`).
- **Streamlined Web Portal File Manager:** Removed artificial `.log`/`.txt` duality, extra `.txt safe` badges, and separate `[.log]` download buttons in `src/web_server/web_server.cpp`. All files now use a uniform, direct download and `[View]` inline preview link, completely eliminating browser HTTP warning workarounds.

## [v0.2.22] - 2026-10-27

### Changed
- **Target Pin Vertical Re-Positioning:** Moved Stage 1 Target Pin center down from `(152, 35)` to `(152, 58)` in `src/lvgl_port/lvgl_port.c`. This completely eliminates crowding and visual overlap with the `"TACTICAL GUIDANCE"` mode badge and the top telemetry HUD readouts (`Hdg: ...` and `Range: ...`), establishing an open, clean header space.
- **Perimeter-Terminating Target Arrow & Track Line:** Re-anchored the vertical track line and forward chevron arrowhead to terminate cleanly on the outer radius of the target pin at `(152, 66)` rather than penetrating to the pin's center point. The chevron wings sweep backwards `(144, 78) -> (152, 66) -> (160, 78)`, framing the bottom perimeter of the target circle while preserving all tolerance wedge rays radiating from the Rover origin.

## [v0.2.21] - 2026-10-27

### Fixed
- **UM982 Receiver Configuration Query Syntax:** Corrected configuration query command in `src/um982_driver/um982_driver.cpp` from `CONFIG HEADING` (which failed with `PARSING FAILD GRAMMAR ERROR,Too less field!`) to `CONFIG` per Section 4.1 of the Unicore Reference Commands Manual. Sending `CONFIG` reliably triggers a complete multi-line configuration dump (`$CONFIG,...`) across all ports, algorithms, and baseline parameters.
- **UM982 Mode Query & Serial Output Synchronization:** Resolved missing `#MODE` operating mode response on Serial Monitor. Added an active polling wait loop for `SAVECONFIG` flash erase/write acknowledgment before querying `MODE`, preventing the receiver's UART buffer from dropping `MODE` while busy programming internal non-volatile memory. Broadened UART response pattern matching to ensure `#MODE` queries and all `$CONFIG` dump lines are streamed directly to `[UM982 COM1 RX]` serial logs.

## [v0.2.20] - 2026-10-27

### Added
- **Course-Up Tactical Guidance Director:** Revamped Stage 1 guidance (> 1.5 m from target) into a fixed, normalized Course-Up Flight Director geometry in `src/lvgl_port/lvgl_port.c`. The Rover origin is anchored at bottom center `(152, 170)` and the Target pin is anchored at top center `(152, 35)`. Replaced discrete autoscaling jumps with a continuous Range HUD readout (`Range: xx.x m`).
- **Fixed Visual Tolerance Wedges (Course Deviation Indicator):** Added fixed visual tolerance rays radiating upward directly from the Rover position `(152, 170)` flanking the track line:
  - Inner Green Sector ($\pm 3^\circ$ centered on track line) bounded by `(152, 170) -> (145, 35)` and `(152, 170) -> (159, 35)`.
  - Outer Amber Sector ($\pm 15^\circ$ flanking green sector) bounded by `(152, 170) -> (117, 40)` and `(152, 170) -> (187, 40)`.
- **Rotating Beam Pointer with Directional Arrowhead:** Implemented a 5-point rotating arrow (55 px shaft + dual 12 px barbs angled at $\pm 150^\circ$) anchored at the Rover origin rotating smoothly with relative turn angle (`-d->rel_bearing_deg`).
- **Target Track Line with Chevron Arrowhead:** Added a fixed vertical line from `(152, 170)` to `(152, 35)` terminating with a forward chevron arrowhead `(144, 47) -> (152, 35) -> (160, 47)` pointing into the pin.
- **Harmonized 3-Tier Dynamic Color & Steering Feedback:** Synchronized color state across the Rotating Beam Arrow, Target Heading HUD label (`t0_tgt_hdg_lbl`), and Steering Cue label (`t0_steer_lbl`):
  - **Tier 1 (Vivid Green `#00FF88`):** Inside Green Wedge ($\le \pm 3^\circ$ or on pin $\le 0.05\text{ m}$), displaying `[OK] ON TRACK - Aimed at Pin`.
  - **Tier 2 (Amber `#FFB800`):** Inside Amber Wedge ($\pm 4^\circ$ to $\pm 15^\circ$), displaying turn cues `>>> Turn Right X deg` / `<<< Turn Left X deg`.
  - **Tier 3 (White `#FFFFFF`):** Outside Wedges ($> 15^\circ$), displaying turn cues with neutral pointer styling.

### Changed
- **Removed On-Screen Breadcrumb Trail Dots:** De-allocated the 16 on-screen breadcrumb dots and eliminated their per-frame coordinate projection loop to maintain an uncluttered, high-contrast tactical navigation canvas.
- **Preserved Full Background Route Logging:** MicroSD and internal FFat route logging (`route_*.csv`) continues recording all 64-bit geodetic epochs uninterrupted at full 10 Hz precision.
- **Tile 0 Mode Badge:** Updated default badge from `"TACTICAL GRID"` to `"TACTICAL GUIDANCE"`.

## [v0.2.19] - 2026-10-27

### Added
- **Tactical Grid Target Heading HUD Readout:** Added a live Target Heading display (`t0_tgt_hdg_lbl`) directly under the Range label on the Tactical Grid (aligned to `LV_ALIGN_TOP_RIGHT, -6, 28`, perfectly in line with the UM982 Hdg label on the left). Shows geodetic bearing from Master antenna to active target (`d->bearing_to_target_deg`). Displayed in white while navigating and automatically turns bright green (`#00FF88`) when aimed on target (heading aligned within $\pm 3^\circ$ or when on target pin).

### Fixed
- **Tactical Grid Antenna Pitch Display:** Fixed the Tactical Grid pitch HUD label in `src/lvgl_port/lvgl_port.c` which previously fell back to the QMI8658 accelerometer/IMU pitch. The label now strictly displays the true antenna baseline pitch (`d->baseline_pitch_deg`) from the `#UNIHEADINGA` heading message, showing `Pitch: --.- deg` when baseline pitch is unresolved.
- **UM982 Dual-Antenna Baseline Configuration:** Configured UM982 for surveyor pole dynamics in `src/um982_driver/um982_driver.cpp`. Applied `MODE ROVER SURVEY` to override the factory default `UAV` mode (which expected drone velocities up to 30 m/s vertical). Applied `CONFIG HEADING LOWDYNAMIC` and `CONFIG HEADING FIXLENGTH`. Formatted `CONFIG HEADING LENGTH` using centimeter units per Table 4-14 of the Unicore N4 Reference Manual (`CONFIG HEADING LENGTH 55.5 12` for the 55.5 cm baseline with 12 cm tolerance).
- **Row 3 Heading Status Badge Logic:** Fixed misleading bright green badge displayed in Row 3 when heading was unresolved. Standardized badge nomenclature: displays `SLV FIX` (Green) and `SLV FLT` (Orange) when the dual-antenna baseline solution is within the physical window, `HDG COG` (Cyan) when walking kinematic COG is active, `HDG HLD` (Amber) when stationary hold is active, and `HDG ---` (Dark Gray) when heading is unresolved.
- **Web Portal Log Download (Chrome HTTP Block Bypass):** Fixed Chrome blocking `.log` file downloads over HTTP (`Unconfirmed *.crdownload`). Added safe `.txt` download naming (`.txt safe`) and in-memory Blob conversion to bypass Chrome's download filter. Added a `[View]` link that streams raw log text directly to the browser with `Content-Type: text/plain; charset=utf-8` without attachment disposition, plus an explicit `[.log]` button.

## [v0.2.10] - 2026-10-26

### Added
- **Standardized Log Schemas (Timestamp in Column 1):** Standardized 	imestamp as the primary key in Column 1 and irmware_ver in Column 2 across both 
oute_*.csv and survey_logged_pins.csv using the high-resolution ISO/MFS format (YYYY-MM-DD_HH-MM-SS.mmm[+-]OFFSET).
- **Unicore Extended Base Station Telemetry (BASEINFOA):** Configured UM982 with BASEINFOA COM1 ONCHANGED to autonomously decode broadcast RTCM 1005/1006 reference frames and report Base Station ID and ECEF coordinates. Implemented closed-form Bowring ECEF-to-WGS84 geodetic conversion in um982_driver.cpp.
- **Differential Base Station ID & Link Quality:** Extracted ase_id from $GNGGA (field 14) and #BASEINFOA (field 4) alongside differential correction age (diff_age_s). Streamed Base ID and latency to [TELEM] serial monitor output and Tile 4 Antenna Diagnostics card.
- **Dual-Antenna Moving-Baseline & Slave Telemetry:** Logged moving-baseline heading fix status (hdg_fix: HDG_FIX, HDG_FLT, NONE), Slave Antenna satellites used (slave_sats), Slave HDOP (slave_hdop), and survey pole tilt (pole_tilt_deg) to both route and survey pin CSVs.
- **Robust Multi-Column Waypoint Ingestion:** Expanded waypoint_load_csv() from 10 to 24 columns and generalized header keyword matching (	imestamp, 	ime, label, lat, lon, elev, 
otes) so surveyed pin files can be directly re-loaded as target navigation waypoints.
- **Automated Field Test Analysis Tooling:** Updated scripts/analyze_field_test.py with csv.DictReader to analyze both legacy and 0.2.10 route and pin datasets.
- **Standardized Waypoint Files & Validator Tool (1.1.0):** Updated waypoints.csv (root and sample generator) and 	est/waypoints.csv to the standard 19-column format matching surveyed pins. Upgraded scripts/validate_waypoints.py to 1.1.0 with dynamic header discovery supporting closed-loop perimeter and leg calculations across any column ordering.

## [v0.2.9] - 2026-10-26

### Added
- Added stepped touch controls on Tile 4 for Backlight Brightness (`25% -> 50% -> 75% -> 100%`) and Audio Volume (`MUTE -> 25% -> 50% -> 75% -> 100%`), automatically debounced and saved to `settings.json` on both MicroSD card and internal FFat mirror.
- Added a dedicated, prominent **Antenna & GNSS Diagnostics Card** on Tile 4 displaying Master Antenna (ANT1) metrics (Fix, SVs, **HDOP**, Horiz/Vert precision, Altitude, Lat/Lon), Slave Antenna (ANT2) metrics (Fix, SVs, **Slave HDOP**, Baseline length, Baseline pitch, True heading), and RTCM differential correction age.
- Added FreeRTOS `beepQueue` and `RoverBeep_t` declarations at file scope in `RTK_Rover.ino`.

### Changed
- Corrected the QMI8658 accelerometer coordinate mapping for the 320x480 portrait display: sensor $a_y$ drives horizontal roll ($b_x$), sensor $a_x$ drives vertical pitch ($b_y$). Tipping Left moves the bubble Left; tipping Right moves the bubble Right; tipping Up moves the bubble Up; tipping Down moves the bubble Down. Resting flat produces $(0.0^\circ, 0.0^\circ)$.
- Disabled horizontal touch swiping across tiles (`LV_DIR_NONE` on all 5 tiles and `LV_OBJ_FLAG_SCROLLABLE` removed). Navigation is strictly handled via the persistent bottom navigation bar `< Prev` and `Next >` buttons, completely preventing accidental page turns while scrolling diagnostics.
- Streamlined the physical BOOT button (GPIO 0): removed short-press waypoint logging to eliminate accidental recordings (on-screen `[LOG PIN NOW]` on Tile 1 is used instead); physical button is reserved exclusively for Long-Press ($\ge 1.5\text{ s}$) full-screen screenshot capture.

## [v0.2.8] - 2026-10-26

### Fixed
- Unblocked IMU sensor polling from LVGL graphics mutex starvation by removing `rover_lvgl_lock(50)` from around Core 1 I2C reads in `task_display_and_audio()`. Polling runs independently at 10 Hz with thread safety guaranteed by `i2c_bus_mutex` in `i2c_bsp.c`.
- Corrected accelerometer resting flat calculation using `-az` in `atan2f(..., -az)` so gravity ($-1.0g$ along Z on this board) reads $0.0^\circ$ at true flat rather than $180^\circ$.
- Replaced volatile beep variables with a FreeRTOS `beepQueue` on Core 1 so multi-tone audio chirps are queued and synthesized sequentially without getting dropped.

## [v0.2.7] - 2026-10-26

### Added
- Added pre-connection 2.4GHz Wi-Fi diagnostic scan reporting visible SSIDs, RSSI, channels, and auth types; automatically locks onto the target router's exact channel during `WiFi.begin(...)`.
- Added automatic mirroring of SD Card `settings.json` to internal `FFat` on boot.

### Changed
- Re-aligned persistent Row 1 header bar: right-clustered Battery (-4), Wi-Fi (-74), and Units (-120); expanded Date/Time window to 165px with `LV_LABEL_LONG_CLIP` to eliminate label overlap.
- Configured QMI8658 silicon registers (cleared self-test bit in CTRL2/CTRL3; configured CTRL5 and CTRL7).

## [v0.2.6] - 2026-10-26

### Fixed
- Resolved `snprintf` argument count mismatch causing Core 1 `LoadProhibited` crash.
- Audited 100% of format specifiers across all source files for strict type-safety.

## [v0.2.5] - 2026-10-26

### Added
- Expanded persistent header to 4 rows: Row 1 (Date, Time, Units, Wi-Fi, Battery), Row 2 (Master Antenna ANT1 fix, SVs, precision, RTCM age), Row 3 (Slave Antenna ANT2 fix, SVs, baseline, pitch, heading), Row 4 (Streamlined ground-tip coordinates).
- Added parsing for Unicore `GNGGAH` (Slave antenna NMEA fix data) to track Slave satellites and Slave HDOP.
- Optimized UM982 NMEA message rates: 2 Hz `GPGGA`, 2 Hz `UNIHEADINGA`, 2 Hz `GPGST`, 1 Hz `GPRMC`, 1 Hz `GPGSV`, and 1 Hz `GNGGAH`.

## [v0.2.4] - 2026-10-25

### Changed
- Configured UM982 moving baseline fix length with `CONFIG HEADING FIXLENGTH` and `CONFIG HEADING LENGTH 55.50 12.00`.
- Eliminated LVGL 9 deprecation warnings.

## [v0.2.3] - 2026-10-25

### Added
- Added Header Slave antenna badge, unfroze IMU reading, and added Tare reset button.

## [v0.2.1] - 2026-10-25

### Added
- Added Heading and Pitch telemetry HUD readout under Tactical Grid badge on Tile 0.
- Simplified steer cues to ASCII (`Turn Right XX deg` / `Turn Left XX deg`).

## [v0.2.0] - 2026-10-25

### Added
- Modernized settings, added tactical background grid, SoftAP fallback, and button state color sync.

## [v0.1.16] - 2026-10-25

### Added
- Implemented Dual-Mode Hybrid Navigation Engine (Tile 0): Course-Up field grid ($> 1.5\text{ m}$) with antenna beam vector, steer cue, and dynamic scaling; high-precision centimeter bullseye ($\le 1.5\text{ m}$) with green ring lock audio cue.
- Added 64-point on-screen breadcrumb ring buffer.
- Implemented continuous Route track CSV logging (`/routes/`) with Start/Stop toggle.
- Implemented raw UM982 NMEA stream logging (`/nmea/`) with Start/Stop toggle.
- Ported HTML5 Web File Portal (`web_server.cpp`) with SD Card and FFat directory browsing, file downloading, uploading (`waypoints.csv`, `settings.json`), browser-based PNG conversion for screenshots, and NTP time sync.
- Implemented automatic storage routing (MicroSD primary, FFat fallback).

## [v0.1.15] - 2026-10-25

### Changed
- Omitted redundant periodic transmission of `$GNHDT` and `$GPTHS` over COM1 in `um982_configure_rover_defaults()` to eliminate ~350 B/s of duplicate serial traffic, relying on rich `#UNIHEADINGA` data while keeping both NMEA heading parsers fully intact as passive fallbacks.

### Fixed
- Fixed USB CDC TX blocking when Serial Monitor is not connected by calling `Serial.setTxTimeoutMs(0)` in `setup()` and guarding periodic telemetry with `if (Serial && ...)`, preventing `task_rover_core0` from stalling when operated in the field on battery power.
- Restored dual-antenna moving-baseline heading updates by commanding `UNIHEADINGA COM1 0.2` and `CONFIG HEADING FIXLENGTH` (`55.5 cm` with `5.0 cm` tolerance) in `um982_configure_rover_defaults()`.
- Updated `handle_ths_sentence()` in `src/um982_driver/um982_driver.cpp` to accept differential (`'D'`), autonomous (`'A'`), estimated (`'E'`), and manual (`'M'`) mode indicators so valid RTK heading is never rejected.
- Updated `handle_uniheadinga_sentence()` to accept any integer fix (`NARROW_INT`, `L1_INT`) and valid float heading solutions while filtering out `INSUFFICIENT_OBS`.

## [v0.1.14] - 2026-10-24

### Changed
- Removed the duplicate `SVs: xx/xx` readout from the Latitude line on Tile 0 (`t0_pos_lbl` in `src/lvgl_port/lvgl_port.c`), keeping `SV:xx/xx` in Row 2 of the persistent Top Header Bar (`hdr_sats_acc_lbl`).

### Fixed
- Replaced the UTF-8 plus-minus character (`??` / `U+00B1`, which rendered as a missing-glyph box in LVGL's built-in ASCII `lv_font_montserrat_12` font) with standard ASCII `+/-` in `hdr_sats_acc_lbl` (`SV:xx/xx +/-1.8 cm`), and expanded `update_rover_ui()` format buffer to `256` bytes to silence `-Wformat-truncation=`.

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
