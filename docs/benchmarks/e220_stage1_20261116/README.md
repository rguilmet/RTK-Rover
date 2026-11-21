# Ebyte E220-900T30S Multi-Rate Stage 1 Bench Characterization (2026-11-16)

**Harness:** [`scripts/bench_e220_multirate.py`](../../scripts/bench_e220_multirate.py) (`v1.0.2`, Stage 1 Pi GPIO `<->` USB-FTDI `1 ms` latency timer)  
**Channels / Rates:** `CH 75` (`925.125 MHz`), `115200 8N1` operational UART, `200 B` LoRa subpacket, `4.8 / 9.6 / 19.2 kbps`  
**Power Profiles Tested:** `21 dBm` (`125 mW`) and `30 dBm` (`1.0 W`, full unattended runs)  
**Companion Engineering Specs:** [`docs/e220-reference.md`](../../e220-reference.md) | [`docs/plans/archive/plan_async_mailbox_lora_transport_analysis.md`](../../plans/archive/plan_async_mailbox_lora_transport_analysis.md)

---

## 1. Executive Scorecard

| Suite | Scope | `21 dBm` (`150739`) | `30 dBm` (`154634`) | Key Finding |
| :--- | :--- | :---: | :---: | :--- |
| **Test 1 (`toa`)** | `18..456 B` (`4` repeats x `3` rates = `144` probes) | **PASS (`144/144`)** | **PASS (`144/144`)** | `0` CRC err; `4.8k` transfers **`35-37%` faster** than old `0.60` efficiency model (`316 B = 695.4 ms`, `400 B = 847.0 ms`) |
| **Test 2 (`guard`)** | `60..10 ms` guard sweep (`648` trials/run) | **PASS (`648/648`)** | **PASS (`648/648`)** | `100.0%` intact down to `10 ms` above empirical `ToA`; coalescing `1006+1230+1084` (`189 B`) cuts airtime `37-38%` |
| **Test 3 (`per`)** | `60` epochs x `5` modes x `3` rates (`1,620` frames) | **PASS (`1,620/1,620`)** | **PASS (`1,620/1,620`)** | **`100.0%`** across `1pkt`, `2pkt_253`, `2pkt_316`, `coalesced_189B`, and continuous `sliced_200B_stream` (`0` CRC err, `0 B` orphans) |
| **Test 4 (`uplink_rssi`)** | `20/40/60/80/100 ms` turnaround x `6` trials (`90` runs) | **PASS (`90/90`)** | **PASS (`90/90`)** | **`DL1=6/6, UL=6/6, RSSI=6/6, Spills=0`** down to `20 ms`; idle Mode 0 RSSI latency **`4.7-6.0 ms`** |
| **Test 5 (`switch`)** | `30` cycles x `4` legs (`120` switches + RF pings) | **FAIL (`119/120`)** | **PASS (`120/120`)** | `240/240` Mode 3 writes passed on attempt `1`; Cycle 13 (`21 dBm`) dropped 1 post-switch RF ping (`found=0`) |
| **Test 6 (`thermal`)** | `5.0 min/rate` @ `4.8k` & `9.6k` (~85-90% duty) | **PARTIAL (`526/526` @ `4.8k`, `0/764` @ `9.6k`)** | **PASS (`1,303/1,303`)** | **`30 dBm` (`1 W`):** `0` errors, `0` AUX resets across `10 min`; `21 dBm` `9.6k` exposed rare FTDI Mode-0 exit UART desync |

---

## 2. Empirical `E220-900T30S` Time-on-Air & `AUX` Waveform Equations

Measured end-to-end (`TX UART write` to `RX CRC24Q complete` at `1 ms` USB polling, matching across `21 dBm` and `30 dBm`):

| Air Rate | Single-Subpacket (`1..200 B`) Fit | `200 B` Measured | Two-Subpacket (`201..400 B`) Fit | `253 B` / `316 B` / `400 B` (`30 dBm`) | Measured `AUX_low` (`201..400 B`) | Early `AUX` Release Lead (`400 B`) |
| :---: | :--- | :---: | :--- | :---: | :---: | :---: |
| **`4.8 kbps`** | `70 + 1.87 * N ms` | `442.8 ms` | `128 + 1.80 * N ms` | `582.6` / `695.4` / `847.0 ms` | `385.7 - 400.1 ms` | **`~461 ms` early** |
| **`9.6 kbps`** | `45 + 1.19 * N ms` | `282.8 ms` | `80 + 1.12 * N ms` | `362.4` / `433.2` / `527.2 ms` | `226.9 - 242.6 ms` | **`~300 ms` early** |
| **`19.2 kbps`** | `29 + 0.80 * N ms` | `186.7 ms` | `55 + 0.70 * N ms` | `232.8` / `277.8` / `336.7 ms` | `129.1 - 145.5 ms` | **`~208 ms` early** |

- Fit residual across all `18..400 B` lengths and all three air rates is **`<= 1.9 ms`**.
- **Coalesced `<= 200 B` Burst (`1006 27B + 1230 18B + 1084 144B = 189 B`, Test 2 `30 dBm`):**
  - `4.8 kbps`: `separate_3 = 754.8 ms` -> **`coalesced_189 = 499.3 ms`** (`-33.8%`)
  - `9.6 kbps`: `separate_3 = 551.0 ms` -> **`coalesced_189 = 345.4 ms`** (`-37.3%`)
  - `19.2 kbps`: `separate_3 = 394.1 ms` -> **`coalesced_189 = 243.2 ms`** (`-38.3%`)

---

## 3. `30 dBm` High-Duty Thermal & `21 dBm` Mode-0 Exit Findings

1. **`30 dBm` (`1.0 W`) Thermal Stability (`Test 6`):**
   - `4.8 kbps` (`5.0 min`): `526/526` (`100.0%`), `CRC_err=0`, `AUX_Rst=0`, `Noise = -93..-96 dBm`, `Pkt = -30..-32 dBm`.
   - `9.6 kbps` (`5.0 min`): `777/777` (`100.0%`), `CRC_err=0`, `AUX_Rst=0`, `Noise = -73..-79 dBm`, `Pkt = -28..-30 dBm`.
   - Confirms Base Pi `5 V` rail sustains `650-850 mA` burst loads and passive `32 MHz` XTAL self-heating induces zero packet loss at narrow `4.8k`/`9.6k` modulation.
2. **Mode 3 -> Mode 0 Post-Switch Liveness Lesson (`21 dBm` `Test 6 @ 9.6k` & `Test 5 Cycle 13`):**
   - In `21 dBm` `Test 6 @ 9.6k`, `/dev/ttyUSB0` passed Mode 3 register write & readback (`9600` baud), then failed both RF RX (`0/764`) and all `5` local Mode 0 `C0 C1 C2 C3 00 02` UART queries (`Noise = None`) until restored via Mode 3 in `finally:`.
   - **Fix applied in [`bench_e220_multirate.py`](../../scripts/bench_e220_multirate.py) (`v1.0.3`):** Probe local Mode 0 responsiveness (`C0 C1 C2 C3 00 02`, `~5 ms`) immediately upon exiting Mode 3 and bounce `(1,1)->(0,0)` + baud once if silent.

---

## 4. Archived Raw Data Files (Scrubbed)

- **`21 dBm` Dataset (`21dBm/`):**
  - [run_bench_20261116_150739_e220_multirate.txt](21dBm/run_bench_20261116_150739_e220_multirate.txt) *(scrubbed terminal log)*
  - [bench_20261116_150739_test1_toa.csv](21dBm/bench_20261116_150739_test1_toa.csv)
  - [bench_20261116_150739_test2_guard.csv](21dBm/bench_20261116_150739_test2_guard.csv)
  - [bench_20261116_150739_test3_per.csv](21dBm/bench_20261116_150739_test3_per.csv)
  - [bench_20261116_150739_test4_uplink_rssi.csv](21dBm/bench_20261116_150739_test4_uplink_rssi.csv)
  - [bench_20261116_150739_test5_switch.csv](21dBm/bench_20261116_150739_test5_switch.csv)
  - [bench_20261116_150739_test6_thermal.csv](21dBm/bench_20261116_150739_test6_thermal.csv)
- **`30 dBm` Dataset (`30dBm/`):**
  - [run_bench_20261116_154634_e220_multirate.txt](30dBm/run_bench_20261116_154634_e220_multirate.txt) *(scrubbed terminal log)*
  - [bench_20261116_154634_test1_toa.csv](30dBm/bench_20261116_154634_test1_toa.csv)
  - [bench_20261116_154634_test2_guard.csv](30dBm/bench_20261116_154634_test2_guard.csv)
  - [bench_20261116_154634_test3_per.csv](30dBm/bench_20261116_154634_test3_per.csv)
  - [bench_20261116_154634_test4_uplink_rssi.csv](30dBm/bench_20261116_154634_test4_uplink_rssi.csv)
  - [bench_20261116_154634_test5_switch.csv](30dBm/bench_20261116_154634_test5_switch.csv)
  - [bench_20261116_154634_test6_thermal.csv](30dBm/bench_20261116_154634_test6_thermal.csv)
