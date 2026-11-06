# Session Kickoff Prompt: RTK-Rover-Mobile

> **Usage:** Copy and paste the prompt below into a new Antigravity session to initiate the **RTK-Rover-Mobile** Android project.

---

```markdown
# Session Goal: RTK-Rover-Mobile (Android Application + ESP32 Hardware Bridge)

## 1. Project Overview & Objective
We are transitioning the user-facing survey experience of our dual-antenna RTK GNSS Rover to a dedicated Android mobile application called **`RTK-Rover-Mobile`**. 

We will use our existing **Waveshare ESP32-S3-Touch-LCD-3.5B** board as the physical **sensor and radio bridge** connected to the GNSS receiver and LoRa transceiver, communicating wirelessly with an Android smartphone running the primary user interface, cadastral mapping, NTRIP client, and survey data management.

---

## 2. System Architecture & Component Roles

```
+-------------------------------------------------------------+
|              ANDROID SMARTPHONE (RTK-Rover-Mobile)          |
|  - Modern Sunlight-Readable UI (Kotlin / Jetpack Compose)   |
|  - High-Resolution Map Engine (Google Maps / MapLibre)      |
|  - Cellular NTRIP Client (rtk2go.com:2101 / Standish Base)  |
|  - Tri-Mode Stakeout (Tactical Bullseye, Line, 2D Cadastral)|
|  - Survey Point Logging, Multi-Epoch Averaging & CSV Export |
|  - Hands-Free Audio Guidance (Proximity Geiger & Voice)     |
+------------------------------+------------------------------+
                               ^
               Bluetooth LE (NUS) / Wi-Fi UDP
                               v
+-------------------------------------------------------------+
|    WAVESHARE ESP32-S3 BRIDGE (Hardware Interface Pod)       |
|  - Holybro UM982 GNSS (COM1 @ 115200 NMEA/Heading, COM2)   |
|  - Ebyte E220 900MHz LoRa (Transparent RTCM / Telemetry)    |
|  - Onboard QMI8658 6-DOF IMU (Pole Plumb Bubble Level)      |
|  - AXP2101 PMIC Power Telemetry & LiPo Battery UPS Buffer   |
|  - 3D Lever-Arm Kinematics (Ground-Tip Projection: 1.80m)   |
|  - Local MicroSD / FFat Backup Raw Logging                  |
+-------------------------------------------------------------+
```

### Component 1: Android Mobile Application (`RTK-Rover-Mobile`)
* **Platform:** Android (Modern Android SDK, Kotlin, Jetpack Compose, Coroutines/Flow).
* **Wireless Connectivity:** BLE Nordic UART Service (NUS) for ultra-low latency, battery-efficient telemetry streaming, with Wi-Fi UDP socket as a high-bandwidth alternative.
* **Cellular NTRIP Client:** Connects over mobile cellular data to RTK2go (`rtk2go.com:2101` or `3.14.70.106:2101`), pulls mountpoint `STANDISH_ME_FN43ur`, verifies RTCM3 frames (`1006, 1074, 1084, 1094, 1230`), and streams them over BLE/Wi-Fi to the ESP32 bridge $\rightarrow$ UM982 COM2.
* **Dual RTCM Source Arbitration:** Automatically manages or falls back between Local 900MHz LoRa (E220) and Cellular NTRIP (RTK2go) based on differential age and link health.
* **Full Survey Navigation Suite (Ported from Firmware v0.7.2):**
  * **Persistent Top Status Header Bar:** Date/time, power, ANT1 fix mode, SVs, precision, RTCM age, ANT2 heading fix, baseline length, true heading, and ground-tip coordinates.
  * **Tactical Guidance & Bullseye Stakeout:** Course-Up vector guidance with concentric centimeter rings ($1\text{ m}$, $20\text{ cm}$, $2\text{ cm}$) and proximity Geiger audio cues.
  * **Cadastral Line Stakeout:** Stationing ($S$), lateral offset ($O$), steer cues (`<<<`/`>>>`), and quadrant deed bearings ($S 62^\circ 15' 12" W$).
  * **2D Cadastral Map View:** Closed boundary polygons (`poly_order > 0`), dashed tie-lines (`-k`), color-coded pins, historical route breadcrumbs, and live heading beam.
  * **Survey Point Logger:** Symmetrical prefix dropdown (`PIN`, `PIPE`, `MONUMENT`, `<DERIVED>`), multi-epoch averaging ($1\text{s}$, $5\text{s}$, $10\text{s}$, $30\text{s}$), and collision-free `_Lxxx` numbering.

### Component 2: ESP32 Hardware Bridge Firmware
* **Current Board:** Waveshare ESP32-S3-Touch-LCD-3.5B (acts as the bridge, with optional status readout on LCD).
* **Future Migration Compatibility:** Waveshare ESP32-S3-LCD-1.47B (21x52mm slim stick) or ESP32-S3-DEV-KIT-N32R16V (headless).
* **Firmware Baseline:** Forked or integrated from `RTK_Rover v0.7.2`. Retains UM982 COM1/COM2 drivers, E220 LoRa driver, lever-arm kinematics, and power management, while adding a BLE NUS / Wi-Fi bidirectional bridge service.

---

## 3. Reference Documentation & Key Artifacts
Before beginning, review these technical specifications in the workspace:
* **Firmware Master Docs:** `RTK_Rover/README.md`, `RTK_Rover/docs/project_overview.md`, `RTK_Rover/docs/session_handoff.md`, `RTK_Rover/docs/user_manual.md`.
* **Smart Pole & App Architecture:** Artifact `plan_headless_smart_pole_and_android_app.md`.
* **Cellular NTRIP Bridge Architecture:** Artifact `plan_phase7_cellular_ntrip_bridge.md`.
* **Cadastral Schema & COGO Engine:** `RTK_Rover/scripts/cogo_solver.py v1.1.0` and `RTK_Rover/docs/schemas/cadastral_schema.json`.

---

## 4. Immediate Tasks for This Session
1. **Initialize Project Scaffolding:** Set up the `RTK-Rover-Mobile` Android project structure (Kotlin, Jetpack Compose, Android Gradle Plugin).
2. **Define Bridge Protocol:** Formulate the compact binary or JSON telemetry and RTCM transfer protocol between ESP32 and Android over BLE NUS (GATT characteristics for Telemetry TX, RTCM RX, Commands).
3. **Develop Android BLE Connection Manager:** Implement scanning, pairing, and automatic reconnection with the ESP32 rover bridge.
4. **Implement Android Cellular NTRIP Client:** Build background NTRIP stream client for RTK2go mountpoint `STANDISH_ME_FN43ur` with CRC24Q verification and bridge forwarding.
5. **Implement Jetpack Compose UI Skeleton:** Construct the persistent 4-row status header bar and core navigation deck.
```
