# Plan Smart Pole and Android App

**Document Version:** `2.1.0`  
**Date:** November 7, 2026  
**Firmware Baseline:** `v0.7.7`  
**System Class:** Modular High-Precision Dual-Antenna RTK Cadastral Platform  
**Smart-Pole Target Hardware:** **Waveshare ESP32-S3-Touch-LCD-3.49-V2** (`172 × 640` 3.49" IPS Display, Integrated 18650 Battery Holder & USB-C Charger, MicroSD Card, QMI8658A 6-DOF IMU, PCF85063A RTC) + **Discrete High-Side Switch (`NDP6020P` + `2N3904`) & `MT3608` 5.0V Boost Converter** + **Holybro H-RTK Unicore UM982** (Dual-Helix GNSS, `24"`–`30"` / `610`–`762 mm` Removable 1" Aluminum Beam) + **Ebyte E220-900T30S** 1W LoRa  
**Field Controller Target:** **Android Smartphone (`RTK-Rover-Mobile`)** via **Bluetooth Low Energy (BLE 5.0 NUS)**  
**Companion Base Station:** `e220_pacer.py v3.4.0` (900 MHz LoRa 1W) & Public NTRIP Caster (`rtk2go.com:2101`)

---

## 1. Executive Summary & Vision

The current `v0.7.7` RTK Rover validated our dual-antenna moving-baseline kinematics, Store-and-Verify RTCM3 gatekeeper, deep-woods heading fallback, and cadastral stakeout workflows on a single handheld **Waveshare ESP32-S3-Touch-LCD-3.5B** unit. Moving to production field operations, this plan transitions the system into a split **Cable-Free Smart Pole + Android Field Controller** architecture:

1. **The Self-Contained Removable Smart-Pole Beam Assembly (Top of Pole):**
   * A rigid **1" square aluminum tube crossbar** (`24"` to `30"` / `610 mm` to `762 mm` antenna separation) that mounts to the top of a standard survey pole with **zero external power or RF cables running down the pole**.
   * Carries **dual multi-band helix GNSS antennas**, the **900 MHz LoRa antenna**, and a **beam-mounted electronics box** housing the **Waveshare ESP32-S3-Touch-LCD-3.49-V2** (with its integrated **18650 battery holder** and onboard USB-C charging circuit), a switched **MT3608 5.0V boost converter** (gated by a through-hole **`NDP6020P` P-FET + `2N3904` NPN** high-side switch), the **Holybro UM982**, and the **Ebyte E220-900T30S**.
   * The wide **`640 × 172` (3.49") IPS touchscreen** mounted along the beam displays high-visibility live status distilled from the top 4 rows of our `v0.7.7` header bar (ANT1 RTK fix & precision, ANT2 heading fix & baseline length, LoRa/NTRIP link status, BLE connection state, 18650 battery voltage, MicroSD health, and ground-tip coordinates).
   * Engineered for **instant power-loss resilience** so turning off the power switch or exhausting/swapping the 18650 cell at any time causes **zero to minimal data loss**.
2. **The Android Field Controller (`RTK-Rover-Mobile` on Pole Clamp):**
   * An Android smartphone mounted at eye level on the survey pole, connected wirelessly to the Smart Pole over **Bluetooth Low Energy (BLE 5.0 Nordic UART Service)**.
   * Runs the primary sunlight-readable user interface (`1,500–2,600 nits`), high-resolution cadastral mapping with satellite/ortho basemaps, cellular NTRIP client (`rtk2go.com:2101`) with automatic LoRa/NTRIP failover, multi-epoch point logging, and hands-free voice/Geiger audio guidance.

```mermaid
flowchart TD
    subgraph Pole["Standard Survey Pole Assembly (100% Cable-Free)"]
        subgraph Beam["Removable 1-Inch Square Aluminum Beam (24 in / 610 mm to 30 in / 762 mm)"]
            ANT1["Master Helix Antenna (ANT1 - Position)"]
            ANT2["Slave Helix Antenna (ANT2 - Heading)"]
            LORA_ANT["900 MHz LoRa Antenna"]

            subgraph Box["Beam-Mounted Smart-Pole Electronics Box"]
                BATT["Integrated 18650 Li-Ion Cell (3500mAh)\nCharged via Waveshare USB-C (ETA6098)"]
                SW["High-Side Power Switch\nNDP6020P P-FET + 2N3904 NPN\n(Gated by J3 Pin 2 3V3)"]
                BOOST["MT3608 5.0V Boost Converter\n+ 100uF Output Capacitor"]
                S3_349["Waveshare ESP32-S3-Touch-LCD-3.49-V2\n• 3.49 in 640x172 IPS Status HUD\n• MicroSD Slot (Crash-Safe Logging)\n• QMI8658A 6-DOF IMU & PCF85063A RTC\n• BLE 5.0 NUS Peripheral"]
                UM982["Holybro UM982 Dual-Antenna GNSS"]
                E220["Ebyte E220-900T30S 1W LoRa"]

                BATT -->|J3 Pin 1 VBAT (3.5V-4.2V)| SW
                S3_349 -->|J3 Pin 2 3V3 Enable| SW
                SW -->|Switched VBAT| BOOST
                BOOST -->|Regulated 5.0V Rail| UM982 & E220
            end

            ANT1 & ANT2 -->|Short Matched Coax| UM982
            LORA_ANT --> E220
            UM982 <-->|UART1 (NMEA/Hdg) & UART2 (RTCM)| S3_349
            E220 <-->|UART0 + AUX ISR| S3_349
        end
    end

    subgraph Controller["Android Field Controller (Pole Clamp at Eye Level)"]
        subgraph App["RTK-Rover-Mobile Native Android App"]
            NTRIP["Cellular NTRIP Client (LTE/5G)"]
            GIS["2D Cadastral Map & Satellite Imagery"]
            STAKE["Tactical, Bullseye & Line Stakeout"]
            LOGGER["Master Survey Point Database & CSV/DXF/KML Export"]
            AUDIO["Natural Voice TTS & Geiger Audio Engine"]
        end
    end

    S3_349 <===>|BLE 5.0 NUS (Telemetry Uplink & RTCM/Cmd Downlink)| Controller
    Caster["RTK2go NTRIP Caster\n(rtk2go.com:2101)"] ==>|Cellular Internet| NTRIP
    BaseLoRa["900 MHz Base Station (1W)"] -.->|LoRa RF 19.2k / 9.6k| E220
```

---

## 2. Architectural Comparison: Handheld Box vs. Smart-Pole + Android System

| Feature / Domain | Current Handheld Rover (`v0.7.7` on 3.5B) | Smart-Pole (`ESP32-S3-Touch-LCD-3.49-V2`) + Android App |
| :--- | :--- | :--- |
| **Top-of-Pole Mechanics** | T-bar with long dual coax cables running down the pole to the handheld display box. | **Self-contained removable 1" square aluminum beam** (`24"`–`30"` / `610`–`762 mm`) with dual helix GNSS antennas, LoRa antenna, and beam-mounted electronics box. **Zero cables down the pole.** |
| **Dual-Antenna Baseline ($B$)** | $555\text{ mm}$ ($21.85\text{ in}$) $\rightarrow \approx 0.20^\circ$ heading accuracy. | **$610\text{ mm}$ ($24\text{ in}$) to $762\text{ mm}$ ($30\text{ in}$)** $\rightarrow \approx 0.13^\circ\text{--}0.16^\circ$ carrier-phase heading accuracy while maintaining brush usability. |
| **Smart-Pole Display** | $3.5"$ $320 \times 480$ primary UI screen. | **$3.49"$ $640 \times 172$ wide IPS status screen** (`AXS15231B` QSPI) aligned along the 1" beam showing the full 4-row status HUD + touch diagnostics. |
| **Primary Operator UI** | Onboard LVGL 9 5-tile touchscreen (~350 nits). | **Android Smartphone (`1,500–2,600 nits`)** on pole clamp running Kotlin / Jetpack Compose `RTK-Rover-Mobile`. |
| **Power Supply & Charging** | External USB pack + internal LiPo UPS buffer. | **Integrated 18650 cell (`3500 mAh`) inside the `3.49-V2` rear holder**, charged via the Waveshare USB-C port (`ETA6098`), feeding an **`NDP6020P` + `2N3904` switched `MT3608` 5.0V boost converter** (`~4.5–5.5 hrs` runtime per cell) with crash-safe data persistence. |
| **Correction Redundancy** | 900 MHz LoRa only. | **Automatic Dual-Source Arbitration:** 900 MHz LoRa (on beam) + Cellular NTRIP (from phone via BLE). |
| **Local & Master Storage** | MicroSD card on 3.5B board. | **Dual Storage:** Master survey point DB on Android phone + **MicroSD card (`SD_MMC` 1-bit) on `ESP32-S3-Touch-LCD-3.49-V2`** for raw NMEA/RTCM/backup logging. |

---

## 3. Smart-Pole Hardware, Power Circuit & Mechanical Beam Design

### 3.1 Removable 1" Square Aluminum Beam & Antenna Geometry
* **Crossbar Structure:** Rigid **1" $\times$ 1" square aluminum tube** mounted perpendicular to the vertical survey pole axis at the exact midpoint ($\frac{B}{2}$) via a quick-release top mount.
* **Configurable Baseline Spacing ($24\text{"}$ to $30\text{"}$ / $610\text{ mm}$ to $762\text{ mm}$):**
  * Balances moving-baseline carrier-phase heading precision ($\sigma_\psi \propto \frac{1}{B}$) against physical maneuverability in New England forest canopy:
    * At **$24\text{"}$ ($610\text{ mm}$, midpoint $305\text{ mm}$):** Compact profile in tight saplings; $\approx 10\%$ tighter heading sigma than $555\text{ mm}$.
    * At **$30\text{"}$ ($762\text{ mm}$, midpoint $381\text{ mm}$):** $\approx 37\%$ tighter heading sigma ($\approx 0.13^\circ$ RMS) and faster integer ambiguity resolution (`HDG FIX`) under partial canopy.
  * Stored in `settings.json` (`antenna_spacing_m: 0.610` .. `0.762`) and configurable from the `RTK-Rover-Mobile` Android app, so `nav_compute_pole_tip_position()` always projects Master (ANT1) coordinates to the exact pole centerline ($\frac{B}{2}$) with zero rotational orbit.
* **Antennas:**
  * **ANT1 (Left / Master) & ANT2 (Right / Slave):** Dual multi-band helix-style GNSS antennas (`L1/L2/L5`, `G1/G2`, `E1/E5a/E5b`, `B1I/B2I/B3I`) mounted at the ends of the 1" aluminum beam for superior phase-center stability and low-elevation multipath rejection in woods.
  * **900 MHz LoRa Antenna:** Mounted directly to the beam assembly with a short low-loss jumper to the E220-900T30S module.

```text
   [ANT1: Master Helix]                            [ANT2: Slave Helix]
     (Left / Position)                               (Right / Heading)
             |                                               |
             | <---------- 24" (610mm) to 30" (762mm) -----> |
   ==========+=======================+=======================+==========
             1" Square Aluminum Tube Removable Crossbar Beam
                     |               |               |
                     |   +-----------+-----------+   |
                     |   | Beam Electronics Box  |   |
                     |   | • ESP32-S3-LCD-3.49-V2|   |
                     |   |   (640x172 Bar LCD +  |   |
                     |   |    18650 Holder + SD) |   |
                     |   | • NDP6020P + 2N3904 + |   |
                     |   |   MT3608 5V Booster   |   |
                     |   | • Holybro UM982 GNSS  |   |
                     |   | • Ebyte E220 1W LoRa  |   |
                     |   +-----------+-----------+   |
                     |               |               |
                  [LoRa Ant]         | (Pole Centerline = B / 2)
                                     |
                           [Quick-Release Mount]
                                     |
                        ===========================
                        Standard Survey Pole (1.80m)
                           (Zero External Cables!)
                        ===========================
                                     |
                     +---------------+---------------+
                     | Android Phone on Pole Clamp   |
                     |     (RTK-Rover-Mobile UI)     |
                     +---------------+---------------+
                                     |
                                [Ground Tip]
```

### 3.2 Beam Electronics Box: `Waveshare ESP32-S3-Touch-LCD-3.49-V2` Hardware & Pinout
Proven previously in the `Magnetic_Field_Scanner` project, the **Waveshare ESP32-S3-Touch-LCD-3.49-V2 (`PCBA V1.1`)** is uniquely suited for the 1" beam enclosure:

1. **Form Factor & `640 × 172` Bar Display:**
   * Its elongated bar profile aligns naturally with the 1" square aluminum beam.
   * The `640 × 172` IPS display (`AXS15231B` QSPI) provides 2x the horizontal pixel width of our `3.5B` board (`320` px), allowing the entire 4-row Smart-Pole Status HUD to render side-by-side in crisp, sunlight-readable typography without wrapping or truncation.
2. **Integrated Peripherals on the `3.49-V2` Board:**
   * **MicroSD Slot (`SD_MMC` 1-bit):** `CLK = GPIO 41`, `CMD = GPIO 39`, `D0 = GPIO 40`.
   * **Internal I2C Bus (`SDA = GPIO 17`, `SCL = GPIO 18`):**
     * `QMI8658A` 6-DOF IMU (`0x6B`) directly on the beam for crossbar pitch/roll tilt compensation and woods gyro heading hold.
     * `PCF85063A` Hardware RTC (`0x51`) for accurate timestamps immediately upon boot before GNSS lock.
     * `TCA9554PWR` I/O Expander (`0x20`) controlling LCD power/backlight (`EXIO1`) and touch reset (`EXIO2`), plus **`ADC_INT` (`GPIO 4`)** with a built-in `200 kΩ / 100 kΩ` (`3.0x`) voltage divider measuring live 18650 `VBAT` voltage.
3. **22-Pin Expansion Header (`J3`) Pin Allocation for Smart-Pole Peripherals:**
   * All required pins for the **Holybro UM982** (`COM1` + `COM2`) and **Ebyte E220-900T30S** (`UART` + `AUX` + `M0/M1`) are broken out on the `J3` 22-pin connector (`8` digital GPIOs required out of `9` free GPIOs on `J3`):

| `J3` Pin(s) | Signal on `3.49-V2` | Smart-Pole Assignment | Function |
| :--- | :--- | :--- | :--- |
| **Pin 1** | **`VBAT`** (`3.5V–4.2V`) | **`NDP6020P` Source (Pin 3)** | Unswitched 18650 battery positive rail (feeds high-side switch). |
| **Pin 2** | **`3V3`** (`+3.3V` Switched) | **`10 kΩ` $\to$ `2N3904` Base** | Active-HIGH board power enable (`3.3V` when ON, `0V` when OFF). |
| **Pins 3, 21, 22** | **`GND`** | **Common Ground** | Star ground for `MT3608`, `UM982`, `E220`, and `2N3904` Emitter. |
| **Pin 5 (`GPIO 15`)** | `GPIO 15` | **UM982 `COM1 TX` $\to$ ESP32 `RX1`** | High-rate `115200` baud NMEA + `UNIHEADINGA` + `BASEINFOA` input. |
| **Pin 6 (`GPIO 16`)** | `GPIO 16` | **ESP32 `TX1` $\to$ UM982 `COM1 RX`** | UM982 configuration commands on boot. |
| **Pin 9 (`GPIO 6`)** | `GPIO 6` | **ESP32 `TX2` $\to$ UM982 `COM2 RX`** | Verified RTCM3 differential corrections into UM982 `COM2`. |
| **Pin 7 (`GPIO 43`)** | `TXD0` (`GPIO 43`) | **ESP32 `TX` $\to$ E220 `RXD`** | LoRa UART TX (`115200` in Mode 0, `9600` in Mode 3). |
| **Pin 8 (`GPIO 44`)** | `RXD0` (`GPIO 44`) | **E220 `TXD` $\to$ ESP32 `RX`** | LoRa UART RX (RTCM3 downlink + RSSI byte). |
| **Pin 10 (`GPIO 48`)**| `GPIO 48` | **E220 `AUX` Interrupt Input** | Hardware rising-edge ISR for quiet-window & sub-packet slicing. |
| **Pin 15 (`GPIO 2`)** | `GPIO 2` | **E220 `M0` Mode Output** | E220 mode control (`LOW` = Mode 0 Normal, `HIGH` = Mode 3 Sleep/Cfg). |
| **Pin 16 (`GPIO 1`)** | `GPIO 1` | **E220 `M1` Mode Output** | E220 mode control (`LOW` = Mode 0 Normal, `HIGH` = Mode 3 Sleep/Cfg). |
| **Pin 4 (`GPIO 4`)** | `ADC_INT` (`GPIO 4`) | **Onboard 18650 `VBAT` Sense** | Built-in `3.0x` divider (`R20=200k / R22=100k`) for live battery `%` & `V`. |

---

### 3.3 18650 Battery, USB-C Charging & Discrete Through-Hole `MT3608` 5.0V Power Circuit

The `Waveshare ESP32-S3-Touch-LCD-3.49-V2` holds a single high-capacity **18650 Li-Ion cell (`3,500 mAh`, e.g., Samsung 35E / Panasonic NCR18650GA)** in its rear battery clip and recharges it directly through the Waveshare **USB-C port** via the onboard **`ETA6098`** charger IC.

#### Why the High-Side Switch (`NDP6020P` + `2N3904`) Is Required
* On the `3.49-V2` schematic (`PCBA V1.1`), **`J3 Pin 1 (VBAT)`** is wired directly to the 18650 battery holder **before** the board's internal power-button latch (`Q2 AO3401`), meaning `J3 Pin 1 (VBAT)` is live `24/7` whenever a battery is installed.
* Meanwhile, **`J3 Pin 2 (3V3)`** is driven by the onboard `SY8089A` buck regulator (`U2`), which is **HIGH (`+3.3 V`) only when the Waveshare board is turned ON**, and **`0 V` when the board is turned OFF** (via the `PWR` button or software shutdown).
* By placing a discrete through-hole **`NDP6020P` P-Channel MOSFET** (TO-220) driven by a **`2N3904` NPN transistor** (TO-92) between `J3 Pin 1 (VBAT)` and the `MT3608` `VIN+`, the entire `5.0V` rail (`MT3608` + `Holybro UM982` + `Ebyte E220-900T30S`) turns **ON and OFF automatically with the Waveshare board's power button**, with **zero quiescent drain (`0.00 µA`)** when powered off.

#### Through-Hole Bill of Materials (Power Stage)
1. **`Q_PFET` — `NDP6020P`** (TO-220 Through-Hole Logic-Level P-Channel MOSFET, `-20 V`, `-24 A`, $R_{DS(\text{on})} \approx 0.05\,\Omega$ at $V_{GS} = -2.5\text{ V}$).
2. **`Q_NPN` — `2N3904`** (TO-92 Through-Hole NPN Transistor).
3. **`R1`, `R2` — Two `10 kΩ` $1/4\text{ W}$ Through-Hole Resistors** (one gate pull-up, one base current limiter).
4. **Boost Module — `MT3608` 2A DC-DC Step-Up Module** (potentiometer pre-adjusted to **`5.00 V`** output **before** connecting the UM982 or E220).
5. **Output Filter Capacitor — `100 µF` to `220 µF` (`10V` or `16V`) Radial Electrolytic Capacitor** across the `MT3608` `5.0V` output to absorb the E220-900T30S `1W` TX current edges.

#### Schematic & Pin-by-Pin Wiring Diagram

```text
J3 Pin 1 (VBAT: 3.5V - 4.2V)
  │
  ├──────────────────────┬────────────────────────► NDP6020P Pin 3 (SOURCE)
  │                      │
  │                    [R1: 10 kΩ] (Holds Gate at VBAT when OFF -> 0 uA leak)
  │                      │
  │                      ├────────────────────────► NDP6020P Pin 1 (GATE)
  │                      │
  │                    2N3904 Collector (Pin 3)
  │                      │
J3 Pin 2 (3V3) ──[R2: 10 kΩ]──► 2N3904 Base (Pin 2)
                         │
                       2N3904 Emitter (Pin 1) ────► J3 Pin 3 (GND)

NDP6020P Pin 2 (DRAIN / Center Pin & Tab)
  │
  └──► Switched VBAT ──► [MT3608 VIN+] ===> [MT3608 VOUT+ (5.00V)] ──┬──► UM982 5V
                                                                     ├──► E220 VCC (5.0V)
                                                                     └──► [100uF Cap (+)] ──► GND
```

* **Note on the `NDP6020P` Metal Tab:** The metal heatsink tab on the `NDP6020P` is internally connected to **Pin 2 (`DRAIN` = Switched `VBAT`)**, so ensure the tab does not touch grounded aluminum inside the enclosure. At `1.8 A` peak current, the `NDP6020P` dissipates only $\sim 0.16\text{ W}$ and needs no heatsink.

#### Power & Runtime Budget (`3,500 mAh` / `12.95 Wh` 18650 Cell)
| Subsystem | Voltage Rail | Average Current | Peak Current (LoRa TX) | Average Power |
| :--- | :--- | :---: | :---: | :---: |
| **`ESP32-S3-Touch-LCD-3.49-V2`** (LCD + BLE + SD + IMU) | `3.3V` (`SY8089A`) | `~180 mA` | `~250 mA` | `~0.65 W` |
| **`Holybro UM982`** (Dual-Antenna RTK + LNA Bias) | `5.0V` (`MT3608`) | `~320 mA` | `~350 mA` | `~1.60 W` |
| **`Ebyte E220-900T30S`** (`97%` RX / `3%` 1W TX Uplink) | `5.0V` (`MT3608`) | `~35 mA` | `~650 mA` (for `25 ms`) | `~0.18 W` |
| **`MT3608` Boost Conversion Loss** ($\eta \approx 88\%$) | `VBAT` $\to$ `5.0V` | — | — | `~0.24 W` |
| **Total Smart-Pole System Load** | **`VBAT` (`3.7V` nom)** | **`~720 mA` from 18650** | **`~1.85 A` from 18650** | **`~2.67 W`** |

* **Continuous Field Runtime:** **$4.5\text{ to }5.2\text{ hours}$** per `3,500 mAh` 18650 cell (and spare 18650 cells can be swapped in seconds in the field).

---

### 3.4 Zero/Minimal Data-Loss Power-Loss Architecture
Whether the operator flips the power switch off, the 18650 cell hits low-voltage cutoff, or a cell is swapped mid-survey, the system guarantees **zero to minimal data loss**:

1. **Master Survey Records Live on the Android Phone (`Zero Survey Point Loss`):**
   * All logged survey points, multi-epoch averaging results, active cadastral waypoint files, and stakeout progress are committed immediately to the Android phone's internal transactional storage (`Room` SQLite + atomic CSV append) as the operator logs them.
   * Even if the Smart Pole loses power mid-traverse, **100% of surveyed points and active navigation state remain safely on the phone**.
2. **Write-Through Atomic Persistence on the Smart-Pole MicroSD Card (`< 0.5 s` Max Diagnostic Loss):**
   * **Read-Mostly `settings.json` with Atomic Rename + `FFat` Mirror:** The Smart Pole only writes `/settings.json` when physical configuration changes (e.g., tare offset, antenna spacing, or LoRa rate switch). Writes use atomic `/settings.json.tmp` $\rightarrow$ `flush()` $\rightarrow$ `rename()` mirrored across MicroSD and internal `FFat`.
   * **Pre-Synced Append Logging (`500 ms` FAT Sync):** Raw diagnostic logs (`nmea`, `rtcm`, `serial_logs`) and local backup survey points on the Smart Pole's MicroSD card flush FAT directory metadata every `500 ms` (and immediately after any logged survey point).
   * **Proactive Low-Battery Flush:** Using `ADC_INT` (`GPIO 4`) to monitor 18650 `VBAT`, if `VBAT` drops below `3.40 V`, the firmware automatically flushes and closes open MicroSD files cleanly before the battery protection circuit trips.
3. **Fast Stateless Reboot & Auto-Reconnect (`< 3.5 s` Full Recovery):**
   * When switched back on (or after a fresh 18650 cell swap), the `ESP32-S3-Touch-LCD-3.49-V2` boots into Mode 0 LoRa bridging and BLE advertising in $< 2\text{ seconds}$ (with immediate valid time from the onboard `PCF85063A` RTC).
   * The `RTK-Rover-Mobile` Android app auto-reconnects over BLE NUS within $1.5\text{ seconds}$, synchronizes active pole parameters (`antenna_spacing_m`, `antenna_height_m`), and resumes RTK streaming automatically.

---

## 4. Smart-Pole Firmware Architecture (`ESP32-S3-Touch-LCD-3.49-V2`)

### 4.1 `640 × 172` Wide Bar Status HUD Layout on the Beam
Mounted in landscape orientation (`640 × 172` pixels) along the 1" aluminum beam, the `3.49-V2` IPS screen displays the top 4 rows of our `v0.7.7` status header with generous horizontal spacing:
* **Row 1 (`Y = 0..42` — Link, Battery & Storage):** `BLE: CONNECTED` (Cyan) / `ADV` (Amber) | Active Correction Source (`LoRa -84dB` or `NTRIP BLE`) | `RTCM: 1.0s` | `SD: OK` | `BAT: 4.08V (88%)`.
* **Row 2 (`Y = 43..85` — ANT1 Master Position):** Color-coded Fix badge (`RTK FIX`, `RTK FLT`, `DGPS`, `SINGLE`) | `ANT1 SV: 28/34` | Horizontal & Vertical RMS (`H: 0.014m / 0.5in`, `V: 0.021m`).
* **Row 3 (`Y = 86..128` — ANT2 Slave Heading & Baseline):** Heading Fix badge (`HDG FIX`, `HDG FLT`, `WALK`, `HLD`) | `ANT2 SV: 26` | Measured Baseline (`BL: 61.0cm` / `76.2cm`) | True Heading (`Hdg: 142.6° SE`).
* **Row 4 (`Y = 129..171` — Ground-Tip Coordinates & Radio/Tilt):** Live projected ground-tip `Lat, Lon, Elev` | Beam Bubble Tilt (`Tilt: 0.4°`) | Active LoRa Rate (`CH75 19.2k`).
* **Touch Interaction:** Tapping the `640 × 172` bar display toggles between the **4-Row Live Survey HUD** and a **Smart-Pole Hardware Diagnostics Card** (UART byte counters, E220 `M0/M1` continuity check, `VBAT` curve, and manual `19.2k <-> 9.6k` LoRa toggle).

### 4.2 Dual-Core FreeRTOS Task Partitioning

```mermaid
flowchart TD
    subgraph Core0["Core 0: Real-Time RTCM Arbiter, UART & Kinematics (Priority 4)"]
        E220_RX["E220 LoRa UART (GPIO 43/44)\n(115200 + AUX ISR GPIO 48)"] --> RTCM_Arbiter
        BLE_RX["BLE 5.0 NUS GATT RX\n(Phone Cellular NTRIP)"] --> RTCM_Arbiter
        RTCM_Arbiter["Dual-Source RTCM3 Arbiter & CRC24Q Gatekeeper\n(Auto-Failover: LoRa <-> BLE NTRIP)"]
        RTCM_Arbiter -->|Serial2 TX (GPIO 6 @ 115200)| UM982_COM2["UM982 COM2 (RTCM In)"]
        UM982_COM1["UM982 COM1 (GPIO 15/16 @ 115200)"] --> NMEA_Parser["NMEA & UNIHEADINGA / BASEINFOA Parser"]
        NMEA_Parser --> KIN["3D Midpoint Lever-Arm Kinematics\n(B = 610..762 mm, H = 1.80 m)"]
    end

    subgraph Core1["Core 1: BLE NUS Streamer, IMU/RTC, 640x172 HUD & Crash-Safe SD"]
        IMU_I2C["Onboard QMI8658A IMU (0x6B)\n& PCF85063A RTC (0x51)"] -->|I2C GPIO 17/18| Tilt_Filter["Beam Tilt & Woods Heading Fallback"]
        VBAT_ADC["Onboard VBAT Sense (GPIO 4)"] --> Telem_Packer
        KIN --> Telem_Packer["Binary Telemetry Packer (5-10 Hz)"]
        Tilt_Filter --> Telem_Packer
        Telem_Packer -->|BLE 5.0 NUS Notify| Phone_Link["Android Phone (RTK-Rover-Mobile)"]
        Telem_Packer -->|45-Byte Quiet-Window Uplink| E220_TX["Base Station Telemetry Uplink"]
        Telem_Packer --> LCD_349["3.49 in (640x172) QSPI Status HUD\n4-Row Top Header Summary"]
        LOGGER["Background Loop (Priority 1)\n500ms Pre-Synced MicroSD Logger & Atomic Settings"]
    end
```

### 4.3 Binary Telemetry Packet Specification (Smart Pole $\to$ Phone over BLE)
The Smart Pole transmits a deterministic binary telemetry frame ($64\text{ bytes}$ at $5\text{--}10\text{ Hz}$ using full 64-bit `double` ground-tip coordinates) over the BLE NUS TX characteristic:

```c
typedef struct __attribute__((packed)) {
    uint8_t  sync[2];          // 0xAA, 0x55
    uint8_t  msg_type;         // 0x01 = High-Rate Smart-Pole Telemetry
    uint32_t timestamp_ms;     // Local Smart-Pole monotonic time
    double   tip_lat_deg;      // Ground-tip Latitude (IEEE-754 64-bit double, sub-mm precision)
    double   tip_lon_deg;      // Ground-tip Longitude (IEEE-754 64-bit double, sub-mm precision)
    int32_t  height_msl_mm;    // Ground-tip Elevation in millimeters
    int16_t  undulation_cm;    // Geoid separation in cm
    uint8_t  fix_type;         // 0=None, 1=Single, 2=DGPS, 4=RTK Fix, 5=RTK Float
    uint8_t  ant2_fix_type;    // 0=None, 4=SLV_FIX, 5=SLV_FLT
    uint8_t  heading_source;   // 1=DUAL_FIX, 2=DUAL_FLT, 3=WALK, 4=GYRO, 6=HOLD
    uint8_t  num_svs_ant1;     // Master antenna tracked SVs
    uint8_t  num_svs_ant2;     // Slave antenna tracked SVs
    uint16_t heading_cd;       // True heading in centidegrees (0-35999)
    uint16_t baseline_mm;      // Measured 3D inter-antenna baseline length in mm (610..762 mm)
    int16_t  baseline_pitch_cd;// Crossbar pitch in centidegrees
    int16_t  tilt_pitch_cd;    // IMU pole tilt pitch in centidegrees (-9000 to +9000)
    int16_t  tilt_roll_cd;     // IMU pole tilt roll in centidegrees (-9000 to +9000)
    uint16_t h_rms_mm;         // Horizontal precision RMS (1 sigma) in mm
    uint16_t v_rms_mm;         // Vertical precision RMS in mm
    uint16_t diff_age_ds;      // Correction differential age in deciseconds (0.1s)
    uint8_t  rtcm_source;      // 0=None, 1=LoRa, 2=BLE NTRIP
    int8_t   lora_rssi_dbm;    // LoRa packet RSSI (-128 to 0 dBm)
    int8_t   lora_noise_dbm;   // LoRa ambient noise floor (-128 to 0 dBm)
    uint16_t vbat_mv;          // Smart-Pole 18650 battery voltage in mV (e.g., 4080 = 4.08V)
    uint8_t  sd_status;        // Bit0=SD Mounted, Bit1=Logging Active, Bit2..7=AirRate enum
    uint16_t crc16;            // CRC-16-CCITT payload checksum
} SmartPoleTelemetryFrame_t;   // 64 Bytes
```

---

## 5. Android Field Application Architecture (`RTK-Rover-Mobile`)

Built with **Kotlin** and **Jetpack Compose**, `RTK-Rover-Mobile` ports all 5 operational tiles of `RTK_Rover v0.7.7` into a high-brightness mobile surveying environment while adding cellular NTRIP bridging and satellite basemaps.

```mermaid
flowchart TD
    subgraph CoreServices["Android Background Foreground Service"]
        BLE_Manager["BLE 5.0 NUS Central Manager\nAuto-Scan, MTU 512 & 1.5s Auto-Reconnect"]
        NTRIP_Engine["Cellular NTRIP Client (rtk2go.com:2101)\nCRC24Q Validator & Whitelist (1006,1074,1084,1094,1230)"]
        Nav_Engine["Geodesic & Stakeout Engine (64-bit Double)\nBowring, Vincenty/Karney, Line Station/Offset"]
        Audio_Synth["Audio Guidance Engine\nLow-Latency Geiger Clicks + Android TTS Voice"]
        Storage_DB["Crash-Safe Local Storage\nRoom SQLite + Instant CSV / GeoJSON / KML / DXF"]
    end

    subgraph ComposeUI["Jetpack Compose Sunlight-Readable UI"]
        Header["Persistent 4-Row Status Header Bar\nANT1 Fix, ANT2 Heading & Baseline, Link Source, Smart-Pole Bat, Tip Coords"]
        Tab0["1. Pin Seeker & Stakeout\nCourse-Up Tactical, 2cm Bullseye & Deed Line Stakeout"]
        Tab1["2. 2D Cadastral & Satellite Map\nClosed Polygons (poly_order > 0), Dashed Ties (-k), Ortho Imagery"]
        Tab2["3. Field Survey Point Logger\nMulti-Epoch RTK Averaging (1s..30s) & Derived _Lxxx Naming"]
        Tab3["4. Waypoint Manager & Compass/Level\nMulti-File CSV Selector, 2D Bubble Level & Beam Tare"]
        Tab4["5. Hardware & Link Diagnostics\nSmart-Pole RF Survey, 18650 Battery Telemetry, Base Observer & NTRIP"]
    end

    BLE_Manager <--> Nav_Engine
    NTRIP_Engine -->|Verified RTCM3 over BLE NUS| BLE_Manager
    Nav_Engine --> Header & Tab0 & Tab1 & Tab2 & Tab3 & Tab4
    Nav_Engine --> Audio_Synth
    Tab2 --> Storage_DB
```

---

## 6. Communication Protocol: BLE 5.0 NUS (Phone $\leftrightarrow$ Smart Pole)

* **Primary Transport — Bluetooth Low Energy (BLE 5.0 Nordic UART Service):**
  1. **Zero Cellular Routing Conflict:** Leaves the Android phone's LTE/5G cellular connection 100% active for `rtk2go.com:2101` NTRIP streaming and cloud map tiles.
  2. **Minimal Power Consumption:** Uses $< 2\%\text{/hr}$ of phone battery and negligible current on the Smart Pole.
  3. **Instant Hands-Free Reconnection:** If the Smart Pole is switched off and back on (or the 18650 battery is swapped), the Android Foreground Service reconnects to the `RTK-Rover` BLE peripheral automatically within $1.5\text{ seconds}$ of boot.
  4. **Ample Bandwidth Margin:** With `MTU = 512` negotiated, BLE 5.0 sustains $> 25\text{ KB/s}$, whereas 10 Hz binary telemetry ($\sim 0.64\text{ KB/s}$) plus filtered RTCM3 downlink ($\sim 0.65\text{ KB/s}$) requires only $\sim 1.3\text{ KB/s}$ ($< 6\%$ of link capacity).

---

## 7. Code Reuse & Migration Matrix from `RTK_Rover v0.7.7` & `Magnetic_Field_Scanner`

| Subsystem | Source Baseline | Smart-Pole (`ESP32-S3-Touch-LCD-3.49-V2`) | Android App (`RTK-Rover-Mobile`) |
| :--- | :--- | :--- | :--- |
| **Board BSP, QSPI LCD (`AXS15231B`), Touch, IMU (`QMI8658A`), RTC (`PCF85063A`) & `VBAT` ADC** | **`Magnetic_Field_Scanner`** | **100% Reused** (`esp_lcd_axs15231b`, `TCA9554` backlight/touch init, `GPIO 4` 18650 `VBAT` divider, `SD_MMC` 1-bit on `GPIO 39/40/41`). | Displays Smart-Pole `VBAT` (`V` and `%`) and IMU tilt status. |
| **`e220_driver`** | **`RTK_Rover v0.7.7`** | **100% Reused** on Core 0 (mapped to `J3` `GPIO 43/44/48/2/1`; Store-and-Verify Gatekeeper, AUX ISR, Mode 0 RSSI, OTA rate switch, 45B Base uplink). | Displays live LoRa RSSI/noise and triggers remote `19.2k <-> 9.6k` switching over BLE command. |
| **`um982_driver`** | **`RTK_Rover v0.7.7`** | **100% Reused** on Core 0 (mapped to `J3` `GPIO 15/16` COM1 & `GPIO 6` COM2). | Displays full ANT1, ANT2, and Base Station observer cards. |
| **`nav_engine`** | **`RTK_Rover v0.7.7`** | **100% Reused** on Core 0/1 (3D midpoint lever-arm projection for `610–762 mm` beam, woods walk/gyro fallback). | **Ported to Kotlin** (64-bit `Double` target guidance, bullseye offsets, and line stakeout station/offset). |
| **`lvgl_port` (UI)** | **`RTK_Rover v0.7.7`** | **Streamlined `640 × 172` Wide Bar Status HUD** rendering the 4-row top header summary + diagnostic card on the `3.49-V2` LCD. | **Full 5-Screen Survey Suite** built in Jetpack Compose with high-contrast outdoor themes. |
| **`waypoint_manager` / Loggers** | **`RTK_Rover v0.7.7`** | **Reused** for crash-safe `500 ms` pre-synced MicroSD logging (`nmea`, `rtcm`, `serial_logs`, backup CSV) + low-`VBAT` auto-flush. | Master CSV / Room SQLite point manager with instant share/export. |

---

## 8. Phased Development Roadmap

```mermaid
flowchart LR
    Stage1["Stage 1: Android App & BLE/NTRIP Bridge\n(Prototype on 3.5B or 3.49-V2)"] --> Stage2["Stage 2: Smart-Pole Firmware on\nWaveshare ESP32-S3-Touch-LCD-3.49-V2"]
    Stage2 --> Stage3["Stage 3: Full Android Cadastral Survey Suite\n(Tactical, Bullseye, Line, Map & Logger)"]
    Stage3 --> Stage4["Stage 4: 1 in Aluminum Beam & Power Box Build\n(NDP6020P + 2N3904 + MT3608 + 24-30 in Helix)"]
```

### Stage 1: Core `RTK-Rover-Mobile` Android App & BLE/NTRIP Bridge
* Scaffold the `RTK-Rover-Mobile` Kotlin / Jetpack Compose Android project.
* Add the lightweight `NimBLE-Arduino` BLE 5.0 NUS GATT service and Core 0 Dual-Source RTCM Arbiter (`LoRa` + `BLE NTRIP`) to the rover firmware (allocating BLE ring buffers in Octal PSRAM per `v0.7.5`–`v0.7.7` `DRAM0` rules).
* Implement the Android Foreground Service managing BLE auto-reconnect (`MTU 512`), `rtk2go.com:2101` NTRIP streaming with `CRC24Q` validation, and live 4-row status header telemetry decoding.

### Stage 2: Smart-Pole Firmware on `Waveshare ESP32-S3-Touch-LCD-3.49-V2`
* Combine the proven `ESP32-S3-Touch-LCD-3.49-V2` board support package from `Magnetic_Field_Scanner` (`AXS15231B` `640 × 172` QSPI display, `TCA9554`, `QMI8658A` IMU, `PCF85063A` RTC, `GPIO 4` 18650 `VBAT` ADC, `SD_MMC` 1-bit) with the Core 0/1 RTK engine from `RTK_Rover v0.7.7`.
* Map `UM982` (`GPIO 15/16` COM1, `GPIO 6` COM2) and `E220-900T30S` (`GPIO 43/44` UART, `GPIO 48` AUX, `GPIO 2/1` `M0/M1`) to the `J3` 22-pin expansion header.
* Implement the high-visibility **`640 × 172` 4-Row Smart-Pole Status HUD** on the onboard bar display and verify crash-safe MicroSD logging (`500 ms` pre-sync + `VBAT < 3.40V` auto-flush).

### Stage 3: Full Android Cadastral Survey Suite (`RTK-Rover-Mobile`)
* Implement the tri-mode stakeout workspace in Jetpack Compose:
  * **Tactical Course-Up & 2 cm Bullseye Stakeout** with rotating `24"`–`30"` antenna beam graphic and low-latency Geiger/TTS audio.
  * **Line Stakeout** with perpendicular offset, stationing, `<<<`/`>>>` steer prompts, and quadrant deed bearings.
  * **2D Cadastral Map View** rendering closed parcel boundary loops (`poly_order > 0`), dashed tie-lines (`-k`), and optional satellite/orthophoto basemap overlays.
* Implement the **Field Survey Point Logger** (`1s`, `5s`, `10s`, `30s` multi-epoch RTK averaging, `<DERIVED>` prefix naming, collision-free `_Lxxx` numbering) and **Multi-File CSV Waypoint Browser**.

### Stage 4: 1" Square Aluminum Beam & Switched 5V Power Box Fabrication
* Wire the through-hole **`NDP6020P` P-FET + `2N3904` NPN + `2 × 10 kΩ`** high-side switch and **`MT3608` `5.0V` boost converter (`+ 100 µF` cap)** to `J3 Pin 1 (VBAT)`, `Pin 2 (3V3)`, and `Pin 3 (GND)` inside the beam-mounted electronics enclosure.
* Assemble the removable **1" square aluminum tube beam** with dual helix multi-band GNSS antennas, 900 MHz LoRa antenna, and the self-contained `3.49-V2` + `18650` + `UM982` + `E220` electronics box.
* Conduct field trials comparing **$24\text{"}$ (`610 mm`) vs. $30\text{"}$ (`762 mm`)** antenna spacing for carrier-phase heading lock time (`HDG FIX`), woods brush clearance, and ground-tip rotational orbit invariance ($360^\circ$ spin test).
