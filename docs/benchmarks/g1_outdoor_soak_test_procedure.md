# Gate G1: 30-Minute Outdoor `I2C1` & Beam Compass Soak Test Procedure

**Project:** `RTK_Rover` (Plan 2 - Phase 2.1 / Gate G1)  
**Target Firmware:** `v0.11.1+` (`SEN0696` on isolated `I2C1 GPIO 47/48`, `F-03` survey hardening)  
**Scoring Script:** `python scripts/verify_g1_outdoor_soak.py <serial-log.txt>`

| Field | Entry | Field | Entry |
| :--- | :--- | :--- | :--- |
| **Date:** | `____-__-__` | **Start / End Time:** | `____:____  /  ____:____` |
| **Test Rig:** | `[ ] Tray Prototype` `[ ] Final Pole` | **Battery Start / End:** | `____% / ____%` |
| **Firmware Build:** | `v0.11.__` | **Serial Log File:** | `serial-log_________________.txt` |

---

## 1. Pre-Soak Setup Checklist (Outdoors, Clear of Buildings & Vehicles)

1. `[ ]` **Tray Material & Clearance Check (if using Tray Prototype):** Confirm the tray/board is **non-metallic** (plastic or wood; do NOT use a steel tray). Keep phones, watches, belt buckles, and keys at least `0.3-0.6 m` (`1-2 ft`) away from the `SEN0696` beam sensor.
2. `[ ]` **Enable SD Serial & Telemetry Logging:** On **Tile 5 (Settings)**, set **`SERIAL: ON`** and **`TELEM: ON`** (confirm MicroSD mounted on Tile 4).
3. `[ ]` **Prevent Mid-Soak `AUTO-OFF`:** On **Tile 5**, set **`AUTO-OFF`** to **`30 min`** or **`DISABLED`** so static periods do not trigger shutdown.
4. `[ ]` **Lock Stationary Gyro FOC (`FOC:OK`):** Set the rig steady (on a non-metallic box/table or bipod) for `2-3 s` until **Tile 4** (`[BEAM 10DOF & IMU]`) reads **`FOC:OK`**.
5. `[ ]` **Run Outdoor `CAL MAG`:** On **Tile 3 (Compass)**, tap **`CAL MAG`**, slowly rotate `360+ deg` while gently tilting `+/-15-20 deg` across all sectors until **`CAL OK!`** appears. Confirm `|B|` is roughly `48-58 uT` (no `[IRON ANOMALY]`).
6. `[ ]` **Level & Tare Plumb:** Level the beam using the bubble level, then tap **`TARE`** on **Tile 3** so `dX / dY` zero near `0.0 deg`.
7. `[ ]` **Converge Open-Sky `HDG FIX`:** Hold steady under open sky for `15-20 s` until Header Row 2/3 show **`RTK FIX`** and **`SLV FIX`** (`HDG_FIX`) so the online True-North and `beam_pitch_tare_deg` converge.

---

## 2. Timed 30-Minute Soak Protocol

### Phase A (`00:00 - 10:00`): Open-Sky Static & 4-Cardinal Rotation
*Note for Tray Rig:* Rest the tray on a **plastic bucket, wooden table, or cardboard box** and step back `~2 m` so your torso does not shade low-elevation satellites during static `dH` / `dP` capture.

1. `[ ]` **`00:00 - 02:30` North (`~0 deg`):** Hold steady for `>= 45 s`, record Tile 3 & Tile 4 values in Table 3.1.
2. `[ ]` **`02:30 - 05:00` East (`~90 deg`):** Rotate `90 deg` CW, hold steady `>= 45 s`, record Table 3.1.
3. `[ ]` **`05:00 - 07:30` South (`~180 deg`):** Rotate `90 deg` CW, hold steady `>= 45 s`, record Table 3.1.
4. `[ ]` **`07:30 - 10:00` West (`~270 deg`):** Rotate `90 deg` CW, hold steady `>= 45 s`, record Table 3.1, plus gentle `+/-10 deg` pitch/roll tilts to verify `dP` tracks baseline pitch.

### Phase B (`10:00 - 20:00`): Canopy / `ANT2` Dropout `FIX -> [CMP]` Switchover
1. `[ ]` Walk under dense tree canopy (or briefly cover/cup hand over **`ANT2` Slave** only) to force **`SLV FIX -> [CMP]`** (`BEAM 9DOF`) transitions.
2. `[ ]` Execute **3 switchover trials** (note heading right before `SLV` drops and immediately after `[CMP]` takes over) in Table 3.2. Confirm **`0.0 s`** switchover delay and **`< 2.0 deg`** jump.
3. `[ ]` Keep walking/holding in canopy for the rest of Phase B; verify `P:` beam pitch on Tile 0 stays smooth and `I2C0` errors stay `0`.

### Phase C (`20:00 - 30:00`): `F-03` Survey Logger Spot-Check & Final Open-Sky Soak
1. `[ ]` On **Tile 1 (Survey)**, select **`Avg: 5s`** (or `10s`) and tap **`LOG PIN NOW`** while level. Verify `AVG FIX xx% (N ep)` counts `~2` epochs/sec (not `30`/sec) and finishes cleanly with `sig:X.Xcm | tilt:X.Xd`.
2. `[ ]` Start a second pin and deliberately tilt **`> 2.5 deg`** mid-window; verify the button shows **`HOLD: TILT X.Xd!`** in red, then level back to complete (or let it test `FIX DIP` / `ABORT` recovery). Record both in Table 3.3.
3. `[ ]` Return to open sky until **`30:00`** elapsed so `[10DOF I2C1]` records final `dH` / `dP` vs. `HDG_FIX` and confirm `I2C0` error count on Tile 4 remains **`0`**.
4. `[ ]` Turn **`SERIAL: OFF`** on **Tile 5** (or power down cleanly) to flush the log file.

---

## 3. Field Observation Tables

### Table 3.1 - Open-Sky 4-Cardinal Static Comparison (Phase A)
| Heading | `UM982` Hdg (`deg`) | `Mag` / `[CMP]` Hdg (`deg`) | `|B|` (`uT`) | `dH` (`deg`) | `dP` (`deg`) | `FOC` | `I2C0 / I2C1` Err |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: | :---: |
| **N (~0)** | | | | | | | |
| **E (~90)** | | | | | | | |
| **S (~180)** | | | | | | | |
| **W (~270)** | | | | | | | |

### Table 3.2 - `SLV FIX -> [CMP]` Canopy Switchover Trials (Phase B)
| Trial | Last `SLV FIX` Hdg (`deg`) | Immediate `[CMP]` Hdg (`deg`) | Jump `|delta|` (`deg`) | Delay (`0.0 s`?) | `P:` Pitch Smooth? |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **1** | | | | `[ ] Yes  [ ] No` | `[ ] Yes  [ ] No` |
| **2** | | | | `[ ] Yes  [ ] No` | `[ ] Yes  [ ] No` |
| **3** | | | | `[ ] Yes  [ ] No` | `[ ] Yes  [ ] No` |

### Table 3.3 - `F-03` Survey Logger Spot-Checks (Phase C)
| Pin | Test Case | Label Logged | `h_sigma` (`cm`) | `maxTilt` (`deg`) | Gate Triggered (`HOLD:` / `skip`) |
| :---: | :--- | :---: | :---: | :---: | :---: |
| **1** | Plumb (`<= 2.5 deg`) | | | | `[ ] None` `[ ] Other: _____` |
| **2** | Tilt `> 2.5 deg` test | | | | `[ ] HOLD: TILT` `[ ] Other: _____` |

---

## 4. Gate G1 Quantitative Scorecard & Post-Soak Steps

1. Copy `/serial-log_*.txt` (and `/survey_logged_pins.csv`) from MicroSD to **`RTK_Rover_Private`** (keep field logs/coords out of public `RTK_Rover`).
2. Run the automated Gate G1 scorer:
   `python RTK_Rover\scripts\verify_g1_outdoor_soak.py <path-to-serial-log.txt>`

| Gate G1 Criterion | Pass Threshold | Measured Result | Verdict |
| :--- | :--- | :--- | :---: |
| `I2C1` Active (`SEN0696` on `GPIO 47/48`) | `100%` of `[10DOF I2C1]` samples | `_____ / _____` | `[ ] PASS  [ ] FAIL` |
| Unresolved `I2C0` Bus Errors (`30 min`) | `0` errors (`I2C:0/x`) | `_____ errors` | `[ ] PASS  [ ] FAIL` |
| Static Beam Compass Accuracy (`dH`) | `< 2.0 deg` RMS vs. `HDG_FIX` | `_____ deg RMS` | `[ ] PASS  [ ] FAIL` |
| Beam vs. Baseline Pitch Agreement (`dP`) | `<= 1.0 deg` RMS vs. `UM982` | `_____ deg RMS` | `[ ] PASS  [ ] FAIL` |
| `HDG_FIX -> [CMP]` Switchover | `0.0 s` switch, `< 2.0 deg` jump | `_____ deg max` | `[ ] PASS  [ ] FAIL` |
| `F-03` Survey Logging & Tilt Gate | Clean `~2 Hz` epochs, `HOLD: TILT` works | Notes: `_____________` | `[ ] PASS  [ ] FAIL` |

> **Important (Tray -> Final 3D-Printed Pole Transition):** Passing Gate G1 on the dinner-tray rig validates `I2C1` bus stability, AHRS fusion, pitch tracking, and switchover behavior. Once assembled into the final 3D-printed enclosure on the survey pole, **re-run `CAL MAG` and `TARE` outdoors** and verify `dH < 2.0 deg` before setting `"apply_lever_arm_on_calibrated_beam": true` in `/settings.json`.
