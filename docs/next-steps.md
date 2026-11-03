# RTK Rover: Next Steps & Architectural Roadmap

**Document Version:** 1.0  
**Date:** 2026-11-02  
**Status:** Post-Hardware Repair (Test `_024` Link Recovery Verified)

---

## 1. Executive Summary

With the discovery and resolution of the broken `M0` hardware line in Test `_024` (restoring true Transparent Mode 0 from `500 ms` WOR sleep), the LoRa link budget and pacing layer are now operating at **100.00% packet and byte delivery** (`412` frames, `0` CRC errors, `0` dropped subpackets at `38.4 kbps` on `CH 75` = `925.125 MHz`).

This document outlines:
1. **Immediate Verification & Firmware Hardening (`v0.2.55`)**
2. **LoRa Link & Wi-Fi Header Bar Redesign (Cellular-Style "Bars" & RSSI)**
3. **E220 Diagnostic Card Enhancements (Per-Message Breakdown)**
4. **Bi-Directional Command & Status Protocol (Base <-> Rover Interleaving)**
5. **Oversized RTCM Frames (> 400 Bytes) & BeiDou Constellation Analysis**
6. **Public Repository Hygiene & Privacy Checklist**

---

## 2. Immediate Next Steps

### 2.1 Outdoor RTK Fix & Distance Walk Test
- **Status:** Ready immediately using deployed `RTK_Rover v0.2.55` and `e220_pacer.py v2.8.1`.
- **Objectives:**
  - Verify transition from `Fix: 5` (`RTK Float`) to `Fix: 4` (`RTK Fixed`) under open sky.
  - Verify dual-antenna baseline lock (`Hdg: FIX`) and baseline length stability (`55.5 cm ± 1.2 cm`).
  - Confirm differential age holds steady at `1.0 .. 1.5 s` at range through foliage and property boundaries.

### 2.2 Boot-Time Hardware Continuity Self-Test (`v0.2.55`)
- **Background:** Ebyte E220 Pin 1 (`M0`) and Pin 2 (`M1`) have internal weak pull-ups to `3.3 V`. A broken wire or loose pin header allows `M0` or `M1` to float `HIGH`, masking failure during boot config (Mode 3 `1,1`) but trapping the radio in periodic sleep (Mode 1 `0,1` or Mode 2 `1,0`) during runtime.
- **Implementation in `e220_driver.cpp` (`e220_run_powerup_selftest()`):**
  - Per E220 Manual Section 5.2.4 (Figure 13), switching between any two modes pulls `AUX` (`IO21`) **`LOW` for `9..11 ms`**.
  - At boot in Mode 3 (`M1=1, M0=1`), toggle `M0` to `0` for 25 ms and verify `AUX` pulses `LOW` (verifying `M0` continuity).
  - Return to `(1,1)`, then toggle `M1` to `0` for 25 ms and verify `AUX` pulses `LOW` (verifying `M1` continuity).
  - If either pulse is missing, log `[E220 PINS] M0_WIRE: OPEN` or `M1_WIRE: OPEN` and display a clear hardware alert on Tile 4.

---

## 3. UI Redesign: Header Bar Link Indicators

### 3.1 Current Header Bar vs. Proposed Cellular-Style Layout

**Current Header Row 1 (`320 x 20 px`):**
```text
[2026-10-31 09:30:00]       [SI/IMP]  [WIFI OFF]  [100% 4.1V]
```

**Proposed Modern Header Row 1:**
```text
[2026-10-31 09:30:00]  [SI/IMP]  [WIFI_ICON]  [LORA_BARS]  [100% 4.1V]
```

### 3.2 Wi-Fi Indicator Simplification
- Replace text `LV_SYMBOL_WIFI " OFF"` / `LV_SYMBOL_WIFI " ON"` with standard icon-only behavior:
  - **Wi-Fi Connected (STA mode):** Solid Electric Cyan (`#22D3EE`) or White (`#FFFFFF`) `LV_SYMBOL_WIFI`.
  - **Wi-Fi SoftAP Active:** Amber (`#FFCC00`) `LV_SYMBOL_WIFI`.
  - **Wi-Fi Connecting / Scanning:** Blinking Cyan `LV_SYMBOL_WIFI`.
  - **Wi-Fi Off:** Dim Slate Grey (`#475569`) or hidden to save horizontal space.

### 3.3 LoRa Signal Strength ("Bars") Indicator
Using live `last_pkt_rssi_dbm` polled every 5 seconds via Mode 0 `C0 C1 C2 C3 00 02`:

| RSSI Range | Indicator | Icon / Representation | Color |
| :--- | :---: | :---: | :--- |
| `>= -70 dBm` | 4 Bars | `||||` or `LV_SYMBOL_VOLUME_MAX` | Green (`#00FF88`) — Excellent |
| `-85 .. -71 dBm` | 3 Bars | `|||.` or `LV_SYMBOL_VOLUME_MID` | Green (`#00FF88`) — Good |
| `-100 .. -86 dBm` | 2 Bars | `||..` | Amber (`#FFCC00`) — Fair |
| `-115 .. -101 dBm` | 1 Bar | `|...` | Orange (`#FFA500`) — Fringe |
| `< -115 dBm` or Stale (> 4s) | 0 Bars / Disconnected | `....` or `LV_SYMBOL_WARNING` | Red (`#FF4444`) — Critical |

### 3.4 Uplink vs. Downlink Status Consideration
- **Downlink (Base -> Rover):** Continuous `1 Hz` stream (`1074`, `1084`, `1094`, `1006`). The Rover knows packet RSSI, noise floor, frame loss, and differential age (`Diff: X.Xs`).
- **Uplink (Rover -> Base):** Currently simplex (Base transmits, Rover receives).
  - *Recommendation:* Keep the top header bar icon focused on **Downlink Quality** (the primary metric required for RTK convergence). When bi-directional messaging is active (Section 5), a small up/down arrow (`▲▼`) adjacent to the bars indicates active bidirectional ping/pong acknowledgment.

---

## 4. Tile 4 Diagnostic Enhancements (Per-Message RTCM Breakdown)

### 4.1 Current Tile 4 E220 Display
```text
[E220 LORA LINK & RTCM]
AUX: HIGH (Edges: 651)
Noise: -68 dBm | Pkt: -21 dBm
RX: 77575 B | Fwd: 77232 B
Frames OK: 409 | Drops: 0 (Trunc: 0)
Orphan: 343 B
```

### 4.2 Proposed Detailed RTCM Message Breakdown
Expand `t4_e220_lbl` to report granular per-type counts and rates:
```text
[E220 LORA LINK & RTCM3]
Status: Mode 0 (Transparent @ 115200) | CH 75 (925.125 MHz)
Signal: RSSI -21 dBm | Noise -68 dBm | SNR +47 dB
Throughput: 705 B/s | Total: 77.5 KB | Forwarded: 77.2 KB
Integrity: OK: 409 | CRC Err: 0 | Trunc Drops: 0 | Orphan: 0 B

[RTCM3 MESSAGE AUDIT]
- 1006 (Base ARP Antenna):    24 pkts  (0.1 Hz)
- 1074 (GPS MSM4 Full):      124 pkts  (1.0 Hz)
- 1084 (GLONASS MSM4 Full):  125 pkts  (1.0 Hz)
- 1094 (Galileo MSM4 Full):  124 pkts  (1.0 Hz)
- 1124 (BeiDou MSM4 Full):     0 pkts  (Disabled)
- 1230 (GLONASS Code-Phase):  12 pkts  (0.1 Hz)
```
*(All counters `msg_1005_count`, `msg_1074_count`, `msg_1084_count`, `msg_1094_count`, and `msg_1230_count` already exist in `E220LinkStats_t` in `src/e220_driver/e220_driver.h` and simply need formatting in `src/lvgl_port/lvgl_port.c`.)*

---

## 5. Bi-Directional Messaging over the LoRa Stream

### 5.1 Architecture: Half-Duplex Time-Division Interleaving
The E220 is a half-duplex transceiver. The Base Station transmits RTCM3 in a `~480 ms` burst at the start of every 1-second UTC epoch (`t = 0.0 .. 0.48 s`), leaving a **`~500 ms` quiet window (`t = 0.50 .. 0.95 s`)** before the next epoch begins.

```text
Time (1s Epoch):  0.0s               0.48s     0.60s        0.75s                1.0s
Base E220:        [==== TX RTCM3 ====]----(idle)-----------------[==== TX RTCM3 ====]
Rover E220:       [==== RX RTCM3 ====]----(idle)--[== TX ACK ==]--[==== RX RTCM3 ====]
                                                  ^
                                       Rover transmits telemetry/commands
                                       strictly within the quiet window!
```

### 5.2 Framing: Proprietary Packets Without Corrupting RTCM3
Because the Rover's Store-and-Verify Gatekeeper and the Base Station Pacer already inspect headers:
- **RTCM3 Frames:** Always start with preamble `0xD3` (`0xD3 0x00..0x03`).
- **Telemetry / Command Frames:** Use a dedicated non-RTCM magic byte prefix:
  - Magic Header: `0x24` (`'$'`) or `0xAA 0x55`.
  - Format: `[SYNC 2B][CMD_TYPE 1B][LEN 1B][PAYLOAD NB][CRC16 2B]`.
- **Parsing Behavior:**
  - Base and Rover UART parsers inspect the first byte:
    - If `0xD3`: Process as RTCM3 (Base -> UM982 COM2).
    - If `0xAA 0x55`: Process in software command router (never forward to UM982 COM2).

### 5.3 High-Value Bi-Directional Use Cases
1. **Rover-to-Base Uplink Telemetry (Every 5-10 seconds):**
   - Rover battery percentage & voltage (`AXP2101`).
   - Rover fix quality (`SINGLE`, `FLOAT`, `FIX`) and satellite counts.
   - Rover-measured LoRa packet RSSI and noise floor at rover location.
   - Allows the Base Station web dashboard to monitor field rover battery and link health in real time.
2. **Base-to-Rover Commands:**
   - Base station coordinates update (force survey-in restart).
   - Change radio channel (`CHAN`) or air rate remotely.
   - Base station power level adjustment (`21 dBm -> 30 dBm` if rover reports weak signal).
3. **Rover-to-Base Commands:**
   - "Ping / Echo" for round-trip latency and link-margin testing.
   - Request Base reboot or RTKBase service restart if differential link goes stale.

---

## 6. Oversized Frames (> 400 Bytes) & Adding BeiDou

### 6.1 Does the Current Pacer Handle Messages > 400 Bytes?
**Yes, safely — but with software multi-packet splitting.**

Here is the exact hardware constraint:
- The `E220-900T30S` internal MCU has a **400-byte RAM buffer**.
- If the host UART writes $> 400$ bytes in a single contiguous write, the E220 MCU FIFO overflows, corrupting or dropping bytes.

In `scripts/e220_pacer.py` (lines 454–464):
```python
if total_len > (self.packet_size * 2):  # > 400 bytes
    sub_cnt = 0
    offset = 0
    while offset < total_len:
        part = chunk[offset : offset + self.packet_size]
        offset += len(part)
        c, _, _ = self.send_packet(part)
        sub_cnt += c
    dt = time.monotonic() - t0
    return (sub_cnt, f"chunked>{self.packet_size * 2}B", dt)
```
- A frame $> 400$ bytes (e.g., a 450-byte BeiDou frame) is chunked in software into `200 B + 200 B + 50 B`.
- Each chunk is written to UART, paced by Time-on-Air (`~82 ms`) + Guard Delay (`35 ms`).
- On the Rover side, [`e220_driver.cpp`](../src/e220_driver/e220_driver.cpp) stages incoming bytes in `rx_stage` (`RTCM_STAGE_CAPACITY = 1,536 B`). Because the gap between chunks (`~117 ms`) is less than `RTCM_TRUNC_TIMEOUT_MS` (`260 ms`), the Rover gatekeeper seamlessly reassembles all 450 bytes and validates `CRC24Q` before forwarding to UM982 COM2.

### 6.2 The Real Threat with BeiDou: Airtime Budget Exhaustion
The challenge with adding BeiDou is **not buffer overflow — it is RF airtime**:

| Constellation | RTCM Msg Type | Typical Satellites | Typical Frame Size | Time-on-Air @ 38.4k |
| :--- | :---: | :---: | :---: | :---: |
| **GPS** | `1074` (MSM4) | 10 - 12 | `~245 B` | `~98 ms` (2 subpackets) |
| **GLONASS** | `1084` (MSM4) | 7 - 9 | `~144 B` | `~65 ms` (1 subpacket) |
| **Galileo** | `1094` (MSM4) | 8 - 10 | `~224 B` | `~98 ms` (2 subpackets) |
| **Base ARP** | `1006` | -- | `27 B` | `~29 ms` (1 subpacket) |
| **GLO Bias** | `1230` (10s) | -- | `18 B` | `~28 ms` (1 subpacket) |
| **Current Total** | -- | **25 - 31 SVs** | **`~660 B / s`** | **`~320 ms / s` (`~48%` duty cycle)** |
| **BeiDou (BDS)** | `1124` (MSM4) | 12 - 16 | **`~320 - 450 B`** | **`~165 - 180 ms` (2-3 subpackets)** |
| **Combined 4-Constellation**| -- | **37 - 47 SVs** | **`~1,050 B / s`** | **`~500 - 550 ms / s` (`~65-75%` duty cycle)** |

#### Recommendations for Enabling BeiDou:
1. **Keep MSM4 (Not MSM7):** MSM4 provides full carrier-phase and pseudorange corrections with half the byte overhead of MSM7.
2. **Elevate Satellite Mask at Base Station:** Set Base tracking elevation mask to `>= 12°` or `15°` to prune low-elevation, multipath-degraded BeiDou satellites, capping BeiDou message size under `320 B`.
3. **If Duty Cycle Exceeds 65%:**
   - Stagger constellation epochs (e.g., GPS + GLO at $t = 0.0\text{s}$, GAL + BDS at $t = 0.5\text{s}$, 2 Hz interleaved).
   - Or step air rate to `62.5 kbps` (`REG0 = 0xE7`), which drops transmission time by ~38% at the cost of `2.5 dBm` link sensitivity.

---

## 7. Public Repository Security & Privacy Audit

A comprehensive scan of all tracked Git files, history commits, and untracked files was performed prior to making the repository public.

### 7.1 What Is Clean & Safe
- **Wi-Fi Passwords:** All committed code, headers, and `settings.json` use placeholder credentials (`"Your-SSID-Here"`, `"Your-Password-Here"`). No actual Wi-Fi password was ever committed in Git history.
- **Private Test Logs:** The `test/` directory (containing raw serial logs, `MySpectrumWiFi` SSID strings, and captured NMEA coordinates) is explicitly listed in `.gitignore` and has never been committed.
- **Deed PDFs & Local Data:** The `reference/` directory (containing raw scanned deed PDFs, DXFs, and local survey CSVs) is explicitly listed in `.gitignore` and untracked.

### 7.2 Items Requiring Attention Before Public Push

#### 1. Real Property & Neighbor PII in Tracked Scripts
The following two scripts are currently tracked in Git and contain personal boundary data:
- [`reference/cogo_network_solver.py`](../reference/cogo_network_solver.py)
- [`reference/cogo_adjoiner_solver.py`](../reference/cogo_adjoiner_solver.py)
- **Sensitive Content:** Full legal name ("Roland Guilmet"), street address ("Sample Rd, Sample Town"), county deed book/page numbers, neighbor names (Haslam, Gray, Blake, Webber, D'Agostino, Crouse), and millimeter-accurate WGS84 coordinates of the property boundaries (`43.73191...`, `-70.62373...`).
- **Recommendation:** If you wish to keep your home address and property boundaries private:
  1. Remove these two specific deed-solving scripts from the public repository (or move them to `reference/` which is already `.gitignore`d).
  2. If keeping them as examples, replace the real coordinates, deed book numbers, and personal names with fictitious sample values (similar to the Boston coordinates used in `waypoints.csv`).

#### 2. Local Machine Paths in Documentation
- [`docs/implementation_plan.md`](implementation_plan.md) and [`docs/navigation_and_route_logging_plan.md`](navigation_and_route_logging_plan.md) contain local Windows user paths:
  `../../...`
- **Action:** Convert these local absolute `..//...` links to clean repository-relative markdown links (`[nav_engine.cpp](../src/nav_engine/nav_engine.cpp)`).

#### 3. Callsign & Hostname Attribution
- `scripts/e220_pacer.py` and `scripts/scan_e220_channels.py` list:
  `Author: Roland L. Guilmet`
- Changelog and docs reference: `pi4b-rtkbase`.
- **Note:** FCC amateur callsigns (``) are publicly searchable. If you prefer pseudonymity on GitHub, change to your GitHub handle (`rguilmet`) or a generic name. If you are comfortable with attribution as a licensed ham operator, this is completely fine to keep.

---

## 8. Summary Action Table

| Priority | Task | Target Component | Notes |
| :---: | :--- | :--- | :--- |
| **1** | **Outdoor Walk & RTK Fix Test** | Field Test | Verify open-sky `Fix: 4`, heading lock, and range margin. |
| **2** | **Privacy Scrub for Public GitHub** | `scripts/solve_*.py`, `docs/*.md` | Sanitize deed coordinates & local file paths before public release. |
| **3** | **Header Bar Modernization** | `src/lvgl_port/lvgl_port.c` | Replace `WIFI OFF` text with icon; add 4-bar LoRa RSSI meter. |
| **4** | **Tile 4 Message Breakdown** | `src/lvgl_port/lvgl_port.c` | Display counts for `1006`, `1074`, `1084`, `1094`, `1230`. |
| **5** | **`M0`/`M1` Continuity Self-Test** | `src/e220_driver/e220_driver.cpp` | Boot-time `AUX` pulse check on mode transitions (`v0.2.55`). |
| **6** | **Bi-Directional Telemetry** | `e220_pacer.py` & `e220_driver.cpp`| Interleave Rover battery/fix status into Base quiet window. |
| **7** | **BeiDou Evaluation** | RTKBase & UM982 | Evaluate 4-constellation airtime budget with satellite elevation mask. |
