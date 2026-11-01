# Ebyte E220-900T30S Practical Engineering Reference

**Document Version:** 1.0  
**Last Updated:** 2026-10-31  
**Target Hardware:** Ebyte `E220-900T30S` (`850.125 - 930.125 MHz`, `1 W` / `+30 dBm` SMD UART LoRa Module)  
**Applicability:** Directly applies to `E220-900T30S` / `E220-900T30D` (`1 W`) and `E220-900T22S` / `E220-900T22D` (`160 mW`, noting TX power bit differences in Section 4.4).  
**Reference Implementations in This Repository:**
- **ESP32-S3 Rover Driver & RTCM3 Gatekeeper:** [`src/e220_driver/e220_driver.cpp`](../src/e220_driver/e220_driver.cpp), [`user_config.h`](../user_config.h)
- **Raspberry Pi Hardware-Paced Transmitter Daemon:** [`scripts/e220_pacer.py`](../scripts/e220_pacer.py), [`scripts/e220-pacer.service`](../scripts/e220-pacer.service)
- **Spectrum Scanner & Configuration Utilities:** [`scripts/scan_e220_channels.py`](../scripts/scan_e220_channels.py), [`scripts/setup_e220.py`](../scripts/setup_e220.py), [`scripts/set_e220_power.py`](../scripts/set_e220_power.py)

---

## 1. Why This Document Exists

The official Ebyte manuals (`E220-900T30S_UserManual_EN_v1.0` and `E220-xxxTxxx_UserManual_EN_v1.7`) suffer from severe translation ambiguities, missing timing diagrams, and several outright factual errors regarding `AUX` pin behavior, EEPROM write acknowledgments, and live RSSI response framing.

This guide documents the **actual silicon behavior** verified empirically across 24 progressive bench and field test campaigns (oscilloscope/ISR pulse timing, raw byte-level UART wire-tapping, and spectrum scans) while building a `100.00%` lossless `1 Hz` multi-constellation GNSS `RTCM3` radio link.

---

## 2. Internal Hardware Architecture (What Is Under the Shield)

Understanding the `E220-900T30S` requires recognizing that **your host MCU does not talk to a LoRa chip**. It talks to an internal Ebyte protocol microcontroller sitting in front of a Semtech `LLCC68` LoRa transceiver:

```text
+--------------------+        UART         +-------------------------------------------------------+
|                    |  TX  --------> RXD  |  Internal Ebyte MCU      SPI      Semtech LLCC68      |
|  Host Processor    |  RX  <-------- TXD  |  - 400-Byte RAM FIFO  <=======>  - 200B Max Payload   |
|  (ESP32 / RPi /    |  GPIO -------> M0   |  - Mode 0..3 State                - 32 MHz Passive    |
|   STM32 / RP2040)  |  GPIO -------> M1   |    Machine & AT/Hex                 XTAL (NO TCXO!)   |
|                    |  ISR <-------- AUX  |    Command Parser                         |           |
|                    |  GPIO -------> RST  |  - ~500ms RC Sleep Timer          +-------+-------+   |
+--------------------+                     |    (~320-340 ppm drift)           | 1W PA / +14dB |   |
                                           |                                   | RX LNA Switch |   |
                                           +-----------------------------------+-------+-------+---+
                                                                                       |
                                                                                   50-Ohm RF
```

### Critical Silicon Realities

1. **400-Byte MCU RAM Buffer vs. 200-Byte LoRa Packet Limit:**
   - The internal MCU buffers up to **400 bytes** from `RXD`, while each over-the-air `LLCC68` LoRa packet carries at most **200 bytes** (`REG1[7:6] = 00`).
   - This 400-byte buffer is specifically designed for **zero-dead-air hardware packet chaining**: when you write a `201..400 B` message at `115200` baud with no inter-byte gaps, the MCU loads the first `200 B` into the `LLCC68` immediately and holds bytes `201..N` in RAM, chaining Subpacket 2 onto the air the instant Subpacket 1 completes.
2. **Passive `32 MHz` Crystal (`XTAL`), NOT a `TCXO`:**
   - Unlike industrial `SX1262` modules fitted with a temperature-compensated oscillator (`TCXO`), the `E220-900T30S` uses a **passive `32 MHz` quartz crystal** right next to a `1 W` power amplifier.
   - **Design Rule:** Under sustained streaming (`40-70%` air duty cycle), PA self-heating induces crystal frequency drift. Low air rates (`<= 19.2 kbps`) use narrow LoRa signal bandwidths and longer Time-on-Air (`~69%` duty cycle for `700 B/s` RTCM3), making them vulnerable to thermal carrier drift. **`38.4 kbps` (`REG0[2:0] = 110`)** uses a wider `500 kHz` LoRa bandwidth (`~48%` duty cycle, `~ -123 dBm` sensitivity), giving `2x-4x` better frequency-drift tolerance while still delivering multi-kilometer line-of-sight range.
3. **Uncalibrated Internal RC Sleep Timer (`~330 ppm`):**
   - In Wake-on-Radio (`WOR`) Modes 1 and 2, the sleep/wake period (`500..4000 ms`) runs off an internal low-power RC oscillator measured at **`+320..+340 ppm`** (`~115 ms` drift per 6 minutes relative to GPS UTC).

---

## 3. Pinout, Power Supply & The Hidden `M0`/`M1` Pull-Up Trap

### 3.1 Pin Definitions & Naming Convention Warning

> **WARNING (Device-Relative Pin Labels):** Ebyte labels `TXD` and `RXD` from the **E220 module's perspective**, NOT the host MCU's perspective:
> - `E220 Pin TXD` = **E220 UART Output** -> Connect to **Host UART RX**.
> - `E220 Pin RXD` = **E220 UART Input**  <- Connect to **Host UART TX**.

| Pin Name | Direction (E220 View) | Internal Pull | Electrical / Functional Notes |
| :--- | :---: | :---: | :--- |
| **`M0`** | Input | **Weak Pull-Up (`3.3 V`)** | Mode select bit 0 (`3.3 V` logic; **never leave floating**). |
| **`M1`** | Input | **Weak Pull-Up (`3.3 V`)** | Mode select bit 1 (`3.3 V` logic; **never leave floating**). |
| **`RXD`** | Input | Weak Pull-Up | UART data into E220 (`3.3 V` CMOS; use series `1k` resistor if driven from `5 V` TTL). |
| **`TXD`** | Output | Push-Pull (`3.3 V`) | UART data out of E220 (idles `HIGH = 3.3 V` once powered and initialized). |
| **`AUX`** | Output | Push-Pull / Open-Drain* | `HIGH` = Idle/Buffer ready; `LOW` = Busy / Mode switching / RX UART shifting. Enable `INPUT_PULLUP` on host MCU. |
| **`VCC`** | Power | -- | `3.3 V - 5.5 V` DC (`>= 5.0 V` required to reach full `+30.0 dBm`; `3.3 V` rail caps RF output around `+26..27 dBm`, while `+21 dBm` works cleanly at `3.3 V` or `5.0 V`). |
| **`GND`** | Ground | -- | Low-impedance RF + power ground plane. |
| **`NRST` / `RST`** | Input | Internal Pull-Up | Active-`LOW` hardware reset (`drive LOW >= 50 ms`, release `HIGH`, then wait for `AUX == HIGH`). |

### 3.2 Power Supply & Bulk Decoupling Requirements

- **Peak TX Current (`E220-900T30S`):**
  - `@ 30 dBm (1000 mW)`: **`650 - 850 mA`** instantaneous current spikes on every packet.
  - `@ 21 dBm (125 mW)`: **`200 - 300 mA`** instantaneous current spikes.
  - `@ RX Continuous (Mode 0)`: **`~14 - 18 mA`**.
- Place **`>= 100 uF` to `470 uF` low-ESR electrolytic/tantalum + `100 nF` ceramic** directly across `VCC` and `GND` at the module pads. Never power an `E220-900T30S` at `27-30 dBm` from a weak `3.3 V` LDO on a USB-UART dongle; brownouts mid-burst corrupt the internal MCU state machine and lock `AUX` `LOW`.

### 3.3 Operating Modes Truth Table

| Mode | `M1` | `M0` | Name | UART Baud | RF TX Behavior | RF RX Behavior | `C0 C1 C2 C3` RSSI Cmd? | `9600` EEPROM Cfg? |
| :---: | :---: | :---: | :--- | :---: | :--- | :--- | :---: | :---: |
| **0** | **`0`** | **`0`** | **Transparent / Normal** | `REG0` (`115200`) | Immediate TX (no WOR preamble) | **100% Continuous RX** | **YES (`< 30 ms`)** | No |
| **1** | **`0`** | **`1`** | **WOR Transmitting** | `REG0` (`115200`) | Prepends `WOR_Period` (`500 ms+`) wake preamble | **Periodic `WOR` Sleep/Wake RX!** | **YES (`~500 ms` delay!)** | No |
| **2** | **`1`** | **`0`** | **WOR Receiving** | `REG0` (`115200`) | **TX Disabled** | **Periodic `WOR` Sleep/Wake RX** | **NO** | No |
| **3** | **`1`** | **`1`** | **Deep Sleep / Config** | **Fixed `9600 8N1`** | TX Disabled | RX Disabled | No | **YES (`C0/C1/C2`)** |

### 3.4 TRAP #1: The Silent Open-`M0` Connector Failure ("Fake Mode 0")

Because **both `M0` and `M1` have internal weak pull-ups to `3.3 V` (`1`)**, an open jumper wire, bad crimp, or cold solder joint on **`M0`** creates one of the most deceptive hardware bugs in embedded radio systems:

1. **Boot Self-Test (`Mode 3: M1=1, M0=1`) PASSES 100%:**
   - Your MCU drives `M1 = 1` and `M0 = 1`.
   - `M1` reaches the module as `1`. Even though the `M0` wire is physically broken, the E220's internal pull-up holds `M0 = 1` at the module!
   - The module enters **Mode 3 `(1, 1)`** and answers `C1 00 06` EEPROM reads and `C0 00 06` writes perfectly.
2. **Runtime Transition (`Mode 0: M1=0, M0=0`) Silently Enters `WOR Mode 1 (M1=0, M0=1)`:**
   - Your MCU drives `M1 = 0` and `M0 = 0`.
   - Calling `digitalRead(PIN_M0)` on the MCU reads the **MCU's own output pad (`0`)**, reporting `M0=0, M1=0` in software!
   - At the E220 module, `M1 = 0` while the broken **`M0` pin floats `HIGH (1)`**, placing the radio in **Mode 1 (`500 ms` Wake-on-Radio duty-cycle sleep)**!
3. **Unmistakable Diagnostic Signature of "Fake Mode 0" (Broken `M0` Line):**
   - Receiver still receives packets at strong SNR, **but never receives two subpackets `< 450 ms` apart** (`<= 1` subpacket per second ceiling; `100%` of `> 200 B` multi-packet frames truncate at `200 B`).
   - In a `1 Hz` GPS stream, the single subpacket caught each second slowly slides later through the epoch (`~100-120 ms` every 6 minutes = `~330 ppm` RC timer drift).
   - Local `C0 C1 C2 C3 00 02` RSSI commands still return valid `C1 00 <noise> <pkt>` bytes (proving Mode 1 rather than Mode 2), **but arrive `500..535 ms` late**, phase-locked to a `500 ms` grid!
   - *(Note: If **`M1`** is open while `M0` is intact, the module enters **Mode 2 `(M1=1, M0=0)`**: same `1` subpacket/s WOR sleep ceiling, except TX and `C0 C1 C2 C3` stop working completely.)*

#### Software Self-Test to Detect Broken `M0` or `M1` Wires at Boot

Per Manual Section 5.2.4 (Figure 13), switching between **any** two modes causes the E220 to pull `AUX` **`LOW` for `9..11 ms`** while reinitializing its state machine:

1. Drive `(M1=1, M0=1)` and wait for `AUX == HIGH`.
2. Toggle **only `M0` `LOW`** `(1,1) -> (1,0)` while monitoring `AUX` via GPIO interrupt for `25 ms`:
   - `AUX` pulses `LOW` (`~10 ms`) -> **`M0` physical wire continuity VERIFIED**.
   - `AUX` stays `HIGH` (`0` edges) -> **`M0` WIRE IS PHYSICALLY OPEN** (module stayed at `(1,1)`).
3. Restore `(1,1)`, wait `AUX == HIGH`, then toggle **only `M1` `LOW`** `(1,1) -> (0,1)` while monitoring `AUX` for `25 ms`:
   - `AUX` pulses `LOW` (`~10 ms`) -> **`M1` physical wire continuity VERIFIED**.
   - `AUX` stays `HIGH` (`0` edges) -> **`M1` WIRE IS PHYSICALLY OPEN**.
4. Drive `(M1=0, M0=0)` and wait for `AUX == HIGH` + `60 ms` guard before opening operational traffic.

---

## 4. Complete Register Map & Mode 3 Programming Protocol

### 4.1 Mode 3 Serial Rules

- **Mode 3 (`M1=1, M0=1`) ALWAYS uses `9600 bps, 8N1`**, regardless of what baud rate is programmed into `REG0`.
- Wait `AUX == HIGH` + `>= 40-60 ms` settle time after asserting `M0=1, M1=1` before sending command bytes, and wait `AUX == HIGH` + `>= 60 ms` after returning to `M0=0, M1=0` before switching your host UART back to operational baud (`115200`).

### 4.2 Binary Command Frame Formats

| Operation | Host Command Bytes (Hex) | Expected E220 Reply (Hex) | Persistence |
| :--- | :--- | :--- | :--- |
| **Write EEPROM** | `C0 <START> <LEN> <DATA[0..LEN-1]>` | `C1` *(or `C0`!)* `<START> <LEN> <DATA...>` | Saved to non-volatile EEPROM |
| **Read Registers** | `C1 <START> <LEN>` | `C1 <START> <LEN> <DATA[0..LEN-1]>` | Read-only |
| **Write Temp RAM** | `C2 <START> <LEN> <DATA[0..LEN-1]>` | `C1` *(or `C2`)* `<START> <LEN> <DATA...>` | Lost on power-cycle or `RST` |
| **Format / Mode Err**| *(Invalid length or wrong baud)* | `FF FF FF` | Rejected |

> **TRAP #2 (`0xC0` vs `0xC1` Write Acknowledgment Bug):** The Ebyte datasheet states that writing EEPROM with `0xC0` returns a `0xC1` header. In practice, certain `E220-900T30S` firmware builds echo **`0xC0`** instead of `0xC1`! Always accept `reply[0] in (0xC0, 0xC1)` on write, and **always follow a write with an explicit `0xC1` readback** to verify EEPROM contents.

### 4.3 Register Address Overview (`0x00 .. 0x07` & `0x80 .. 0x86`)

| Addr | Name | Purpose | Recommended (`RTK / High-Speed`) |
| :---: | :--- | :--- | :---: |
| `0x00` | **`ADDH`** | Module Address High Byte (`0x00..0xFF`; `0xFFFF` = broadcast) | `0x00` |
| `0x01` | **`ADDL`** | Module Address Low Byte (`0x00..0xFF`) | `0x00` |
| `0x02` | **`REG0`** | Operational UART Baud `[7:5]`, Parity `[4:3]`, Air Data Rate `[2:0]` | `0xE6` (`115200 8N1`, `38.4k`) |
| `0x03` | **`REG1`** | Subpacket Size `[7:6]`, RSSI Reg Enable `[5]`, TX Power `[1:0]` | `0x23` (`200B`, `RSSI=ON`, `21dBm`) |
| `0x04` | **`CHAN`** | RF Channel Offset (`0..80` -> `850.125 MHz + CHAN * 1.0 MHz`) | `0x4B` (`CH 75` = `925.125 MHz`) |
| `0x05` | **`REG3`** | Append RSSI `[7]`, Fixed/Trans `[6]`, LBT `[4]`, WOR Role/Cycle `[3:0]`| `0x00` (Transparent, LBT Off) |
| `0x06` | **`CRYPT_H`** | Write-Only Encryption Key High Byte (reads back as `0x00`) | `0x00` |
| `0x07` | **`CRYPT_L`** | Write-Only Encryption Key Low Byte (reads back as `0x00`) | `0x00` |
| `0x80..0x86` | **`PID`** | Read-Only 7-Byte Product/Firmware Info (`C1 80 07`) | *(Read-only)* |

### 4.4 Detailed Bitfield Reference

#### `0x02` — `REG0` (UART Speed, Parity & LoRa Air Data Rate)

| Bits `[7:5]` | Operational UART Baud | Bits `[4:3]` | UART Parity | Bits `[2:0]` | LoRa Air Data Rate |
| :---: | :--- | :---: | :--- | :---: | :--- |
| `000` | `1200 bps` | **`00`** | **`8N1` (Default)** | `000` | `2.4 kbps` |
| `001` | `2400 bps` | `01` | `8O1` | `001` | `2.4 kbps` |
| `010` | `4800 bps` | `10` | `8E1` | `010` | `2.4 kbps` |
| `011` | `9600 bps` | `11` | `8N1` (alias of `00`) | `011` | `4.8 kbps` |
| `100` | `19200 bps` | | | `100` | `9.6 kbps` |
| `101` | `38400 bps` | | | `101` | `19.2 kbps` |
| `110` | `57600 bps` | | | **`110`** | **`38.4 kbps` (Recommended)** |
| **`111`** | **`115200 bps` (Recommended)**| | | `111` | `62.5 kbps` |

> **Important:** Both ends of the RF link **MUST** have identical **Air Data Rate (`REG0[2:0]`)**, **Channel (`CHAN`)**, **Subpacket Size (`REG1[7:6]`)**, and **CRYPT Key (`0x06..0x07`)**. Their **UART Baud (`REG0[7:5]`)** may differ if desired, though `115200 8N1` on both ends minimizes UART serialization latency (`17.36 ms` per `200 B`).

#### `0x03` — `REG1` (Subpacket Size, Mode 0 RSSI Register Enable & TX Power)

| Bits | Field | Encoding | Engineering Notes |
| :---: | :--- | :--- | :--- |
| **`[7:6]`** | **Subpacket Size** | `00` = **`200 B`**<br>`01` = `128 B`<br>`10` = `64 B`<br>`11` = `32 B` | Maximum payload per physical LoRa packet. Use **`200 B` (`00`)** to minimize preamble overhead and keep `<= 400 B` frames within the atomic 2-packet buffer. |
| **`[5]`** | **RSSI Register Enable** | `0` = Disabled<br>**`1` = Enabled (`0x20`)** | **Always set to `1` (`0x20`)!** Enables `C0 C1 C2 C3` ambient noise & last-packet RSSI queries in Mode 0 **without** appending extra bytes to your RX UART payload. |
| **`[4:2]`** | Reserved | `000` | Keep `000`. |
| **`[1:0]`** | **TX Power (`T30S`/`T30D`)** | `00` = `30 dBm` (`1000 mW`)<br>`01` = `27 dBm` (`500 mW`)<br>`10` = `24 dBm` (`250 mW`)<br>**`11` = `21 dBm` (`125 mW`)** | *(Note: On `22 dBm` `E220-900T22S/D` variants, `00..11` map to `22 / 17 / 13 / 10 dBm` instead.)* Start with `21 dBm` (`11`) to minimize PA thermal drift and legal ERP issues; step up only if link margin requires it. |

#### `0x04` — `CHAN` (RF Frequency & US FCC 902-928 MHz ISM Band Limits)

For `E220-900Txx` modules, center frequency is:

$$f_{\text{center}} = 850.125\text{ MHz} + (\text{CHAN} \times 1.000\text{ MHz}), \quad \text{CHAN} \in [0..80]$$

- **Hardware Supported Range:** `CHAN = 0` (`850.125 MHz`) to `CHAN = 80` (`930.125 MHz`).
- **US / Canada FCC Part 15 / ISED (`902.0 - 928.0 MHz`) Legal Channel Window:**
  - **Lowest Safe Channel:** **`CHAN = 53` (`0x35`) = `903.125 MHz`**. `CHAN = 52` (`902.125 MHz`) is centered in-band, but assuming a `500 kHz` signal bandwidth its lower edge (`901.875 MHz`) falls outside `902.0 MHz`.
  - **Highest Safe Channel:** **`CHAN = 77` (`0x4D`) = `927.125 MHz`** (upper edge `927.375 MHz < 928.0 MHz`). `CHAN = 78` (`928.125 MHz`) is **out of band in the US**.
  - *Note:* Ebyte does not publish the LoRa bandwidth for each air rate. The `500 kHz` figure is an assumption, so treat these edges as conservative guidance, not a compliance certification.
  - **Recommended Clean US Channels:** **`CHAN = 74, 75, 76` (`924.125 - 926.125 MHz`)** — `+10 MHz` clear of `915.0 MHz` consumer weather/IoT congestion (see Section 7).

#### `0x05` — `REG3` (RSSI Byte Append, Fixed/Transparent, LBT & WOR Cycle)

| Bits | Field | Encoding | Engineering Recommendation |
| :---: | :--- | :--- | :--- |
| **`[7]`** | **Append RX RSSI Byte** | **`0` = Off**<br>`1` = Append 1 byte (`0x80`) | **Leave `0` (`OFF`) for binary protocols!** Setting `1` tacks an extra raw RSSI byte onto the end of *every* received subpacket (including after byte `200` right in the middle of a `340 B` frame!), corrupting framed streams unless reassembled at subpacket level. Use `REG1[5] = 1` (`0x20`) instead. |
| **`[6]`** | **Transmission Mode** | **`0` = Transparent**<br>`1` = Fixed-Point (`0x40`) | Use **`0` (Transparent)** for point-to-point/broadcast streams. *(In Fixed-Point mode, every TX write must begin with a 3-byte `[ADDH, ADDL, CHAN]` header which the E220 strips before airing.)* |
| **`[5]`** | Reserved | `0` | Keep `0` (`E220-900T30S` does not support repeater mode). |
| **`[4]`** | **LBT (Listen-Before-Talk)**| **`0` = Disabled**<br>`1` = Enabled (`0x10`) | Leave **`0` (`OFF`)** on dedicated clean channels with real-time `1 Hz` pacing; LBT injects unpredictable backoff delays up to `2000 ms` that break deterministic subpacket spacing. |
| **`[3]`** | **WOR Role** | `0` = WOR RX<br>`1` = WOR TX (`0x08`) | Ignored in Mode 0 (`0`). |
| **`[2:0]`**| **WOR Wake Period** | `000`=`500ms`, `001`=`1000ms` ... `111`=`4000ms` | `(code + 1) * 500 ms`. Ignored in Mode 0, active in Modes 1 & 2. |

---

## 5. The `AUX` Pin Truth & Hardware-Calibrated Two-Regime Pacing

### 5.1 TRAP #3: `AUX == HIGH` Does NOT Mean RF Transmission Finished!

The single costliest misconception in Ebyte's manual (Section 5.6.2) is assuming `AUX` stays `LOW` while the radio is transmitting over the air. **It does not.**

`AUX` indicates whether the **internal 400-byte UART RAM buffer is ready to accept more bytes without overflowing** — NOT whether the `LLCC68` RF power amplifier is still transmitting!

At `115200` baud (`86.8 us/byte`), the E220 detects end-of-frame after a **3-character UART idle gap (`~0.26 ms`)**. What `AUX` does next depends strictly on whether your write was **`<= 200 B`** or **`201..400 B`**:

### 5.2 Regime 1: Single-Subpacket Writes (`1 .. 200 Bytes`)

```text
Host UART TX:  |=== 144B (12.5ms) ===|---(idle)-------------------------------|
E220 RAM Buf:  [ Filling 0..144B     ][Xfer->LLCC68 (<0.5ms): RAM NOW EMPTY! ]
E220 AUX Pin:  HIGH ---------------------------------------------------------- (NEVER STAYS LOW!)
LLCC68 RF PA:                         |======= On-Air TX: 65.0 ms (@38.4k) =======|
Rover UART RX:                                                                    |== 144B (12.5ms) ==|
                                      |<----------- Must Wait: ToA + 35ms Guard ---------->|
```

- Because all `1..200` bytes move straight into the idle `LLCC68` baseband FIFO in `< 0.5 ms`, the 400-byte RAM buffer is immediately empty again — so **`AUX` stays `HIGH` (or glitches `< 0.5 ms`) while the packet is actively transmitting on the air!**
- **Fatal Mistake 1:** Waiting up to `500 ms` for `AUX` to fall `LOW` after writing a `<= 200 B` frame stalls your transmitter by `500 ms` on every small frame!
- **Fatal Mistake 2:** Seeing `AUX == HIGH` and immediately writing the next frame causes back-to-back air/receiver collisions before the receiver finishes draining its UART!
- **Correct Pacing Rule (`1..200 B`):**
  1. Verify `AUX == HIGH` before writing.
  2. Write complete frame (`<= 200 B`) atomically and `flush()` (`tcdrain()`).
  3. Sleep deterministically for **`ToA(N) + Guard_Delay`** (`Guard_Delay = 35 ms` at `115200` baud).

### 5.3 Regime 2: Two-Subpacket Writes (`201 .. 400 Bytes`)

```text
Host UART TX:  |==== 343B (29.8ms) ====|
E220 RAM Buf:  [0..200B -> LLCC68][ 143B waiting in RAM... ][143B -> LLCC68: RAM EMPTY!]
E220 AUX Pin:  HIGH ---|_______________LOW (68.2ms)_________|----------- HIGH ----------------
LLCC68 RF PA:          |=== Subpkt 1: 200B (98.0ms total) ==|=== Subpkt 2: 143B (64.7ms) ===|
                                                            ^
                                       DANGER! AUX RISES HIGH HERE -- RIGHT AS
                                       SUBPACKET 2 *BEGINS* TRANSMITTING ON AIR!
```

- When you write `201..400 B` (`N` bytes) in one contiguous UART write:
  1. As soon as byte `200` arrives, the E220 MCU loads Subpacket 1 (`200 B`) into the `LLCC68`, **starts RF transmission immediately while UART bytes `201..N` are still arriving**, and pulls **`AUX` `LOW`**.
  2. Therefore, the measured duration **`t_uart_write + t_aux_low` is a hardware constant equal to the exact single-packet Time-on-Air of `200 B`** (`98.5 ms` @ `38.4 kbps`; `165.0 ms` @ `19.2 kbps`), regardless of whether the total frame is `214 B` or `343 B`!
  3. The instant Subpacket 1 finishes airing, the E220 MCU moves Subpacket 2 (`N - 200` bytes) from RAM into the `LLCC68` FIFO and **raises `AUX` `HIGH` while Subpacket 2 is just STARTING its RF transmission!**
- **Fatal Mistake:** Treating `AUX` rising `HIGH` as "transmission complete" and sending the next message immediately collides with Subpacket 2!
- **Correct Pacing Rule (`201..400 B`):**
  1. Verify `AUX == HIGH` before writing.
  2. Write all `N` bytes (`201..400 B`) in one contiguous write (never pre-split `201..400 B` in host software — let the E220 hardware chain Subpacket 1 -> Subpacket 2 seamlessly!).
  3. Wait for **`AUX` falling edge** (typically `< 2 ms`, `50 ms` max timeout; if already `LOW` when `flush()` returns because `t_uart > 18 ms`, proceed directly to step 4).
  4. Wait for **`AUX` rising edge** (`1.5 s` hardware watchdog timeout -> pulse `RST` `LOW` `50 ms` if wedged).
  5. **Mandatory Tail Wait:** Sleep for **`ToA(N - 200) + Guard_Delay`** so Subpacket 2 finishes airing AND the receiver drains its UART before the next frame begins!

### 5.4 Hardware-Calibrated Time-on-Air (`ToA`) Formula

Ebyte does not publish the exact `LLCC68` `(SF, BW, CR, Preamble)` register tuples mapped to each `REG0[2:0]` air rate. Direct hardware calibration using `t_uart + t_aux_low` across `E220-900T30S` modules yields an accurate closed-form model (`~0.68` net coded payload efficiency after LoRa preamble, explicit header, CRC16, and FEC):

$$\text{ToA}(N) = 0.016 + \frac{(N + 16) \times 8}{\text{AirRate}_{\text{bps}} \times 0.68}\ \text{seconds} \quad (1 \le N \le 200)$$

| Subpacket Payload (`N`) | `ToA` @ `19.2 kbps` | `ToA` @ `38.4 kbps` *(Rec.)* | `ToA` @ `62.5 kbps` | Receiver UART Drain @ `115200` | Recommended Guard Delay |
| :---: | :---: | :---: | :---: | :---: | :---: |
| **`27 B`** *(RTCM 1006)* | `42.3 ms` | **`29.2 ms`** | `24.1 ms` | `2.34 ms` | `35 ms` |
| **`128 B`** | `104.2 ms` | **`60.1 ms`** | `43.1 ms` | `11.11 ms` | `35 ms` |
| **`144 B`** *(RTCM 1084 / Tail)* | `114.0 ms` | **`65.0 ms`** | `46.1 ms` | `12.50 ms` | `35 ms` |
| **`200 B`** *(Full Subpacket)* | `148.4 ms` | **`82.2 ms`** | `56.7 ms` | `17.36 ms` | `35 ms` |

*(Note: Hardware `t_uart + t_aux_low` on a `200 B` head subpacket includes `~16 ms` of internal MCU/preamble startup overhead on packet 1, totaling `~98.5 ms` @ `38.4k` and `~165.0 ms` @ `19.2k`.)*

### 5.5 Receiver `AUX` Pin Timing (Mode 0)

On the receiving `E220-900T30S` in Mode 0 (`115200 8N1`):
- `AUX` remains `HIGH` while the `LLCC68` is demodulating a packet over the air.
- Once LoRa CRC passes, `AUX` drops **`LOW` `2..3 ms` before byte `0`** exits `E220_TXD`, stays `LOW` for the exact UART shift-out duration (`~4.5 ms` for `27 B`, `~14.8 ms` for `144 B`, `~19.8 ms` for `200 B`), and returns `HIGH`.
- On a chained 2-subpacket frame (`201..400 B`), `AUX` produces **two falling edges** (`AUX edges: +2`), delivering Subpacket 1 (`200 B`) followed `~60..80 ms` later (@ `38.4k`) by Subpacket 2 (`N - 200 B`).

---

## 6. Live Mode 0 RSSI & Ambient Noise Floor Queries (`C0 C1 C2 C3`)

### 6.1 Enabling Without Corrupting Data (`REG1` Bit 5 vs. `REG3` Bit 7)

- Set **`REG1` Bit 5 = `1` (`0x20`)** and keep **`REG3` Bit 7 = `0`**.
- This enables on-demand Mode 0 register reads (`0x00` = current ambient RF noise floor on `CHAN`; `0x01` = RSSI of last received LoRa packet) while keeping received payload bytes `100%` untouched.

### 6.2 TRAP #4: Undocumented `4-Byte` vs. `5-Byte` Firmware Reply Quirk

The Ebyte manual states that querying 2 registers (`C0 C1 C2 C3 00 02`) returns a **5-byte** frame (`C1 00 02 <noise> <pkt>`). **Actual `E220-900T30S` firmware omits the `<len>` byte (`0x02` / `0x01`)!**

| Query Sent in Mode 0 (Hex) | Datasheet Claimed Reply | **Actual `E220-900T30S` Reply** | Conversion Formula |
| :--- | :--- | :--- | :--- |
| **`C0 C1 C2 C3 00 02`** *(Noise + Last Pkt)* | `C1 00 02 <noise> <pkt>` *(5 B)* | **`C1 00 <noise> <pkt>` *(4 B!)*** | `dBm = int(raw_u8) - 256` |
| **`C0 C1 C2 C3 00 01`** *(Noise Only)* | `C1 00 01 <noise>` *(4 B)* | **`C1 00 <noise>` *(3 B!)*** | `dBm = int(raw_u8) - 256` |

- **Why Dual-Format Parsing Is 100% Unambiguous:** Valid `<noise>` and `<pkt>` raw bytes represent `-128 dBm .. -11 dBm`, which always lie in **`0x80 .. 0xF5`** (`>= 128`), never `0x01` or `0x02`!
  - If `len == 4` and `buf[0..1] == C1 00` and `buf[2] != 0x02`: parse **4-byte** reply (`noise = buf[2] - 256`, `pkt = buf[3] - 256`).
  - If `len == 5` and `buf[0..2] == C1 00 02`: parse **5-byte** reply (`noise = buf[3] - 256`, `pkt = buf[4] - 256`).
  - Example from Test `_023`: `C1 00 8E BF` -> `Noise = 0x8E (142) - 256 = -114 dBm`, `Packet = 0xBF (191) - 256 = -65 dBm` (`+49 dB` SNR).

### 6.3 Collision-Free Polling on an Active Streaming Receiver

Never fire `C0 C1 C2 C3 00 02` blindly on an unsynchronized timer while receiving packets:
1. **Synchronize to the Inter-Burst Quiet Window:** On a `1 Hz` stream (e.g., `1074 -> 1084 -> 1094` occupying `~480 ms`), send `C0 C1 C2 C3 00 02` **`30..350 ms` after the final frame (`1094`) completes**, when `AUX == HIGH` and `Serial.available() == 0`. (In true Mode 0, the 4-byte reply returns in **`< 25..30 ms`**.)
2. **Zero-Loss Prefix-Mismatch Abort:** While awaiting the `C1 00` reply (`80 ms` timeout), inspect each incoming byte immediately: if `byte[0] != 0xC1` or `byte[1] != 0x00` (e.g., `0xD3` from an unexpected LoRa packet arriving simultaneously), immediately cancel RSSI interception and pass the buffered bytes straight into your data stream parser so zero payload bytes are lost.

---

## 7. RF Spectrum Selection & Near-Field Desense (US 902-928 MHz)

1. **Move the Antenna Away from Host SBCs / Digital Clocks (`15-20 dB` Noise Improvement!):**
   - The `E220-900T30S` includes a sensitive `+14 dB` RX LNA. Mounting a `915 MHz` whip directly next to a Raspberry Pi 4B (`USB 3.0`, `HDMI`, `SDRAM`, Ethernet harmonics) severely desenses the receiver.
   - Adding a **4-foot (`1.2 m`) low-loss coaxial lead** to separate the antenna from the Raspberry Pi and desk electronics dropped ambient noise across clean channels to **`-95..-97 dBm`** near the Base and **`-114 dBm`** in adjacent rooms.
2. **Avoid `902 - 916 MHz` Consumer ISM Clutter (`CH 52..66`):**
   - Consumer weather stations (e.g., Ecowitt `915.0 MHz`), utility meters, and LoRaWAN/Meshtastic nodes heavily populate `902-916 MHz`.
   - Use [`scripts/scan_e220_channels.py`](../scripts/scan_e220_channels.py) to sweep `CHAN = 52..77` (`30-50` Mode 0 `C0 C1 C2 C3 00 01` samples per channel) and export a CSV noise profile before deploying at a new site.
   - In our site survey (`test/e220-scan.csv`), **`CH 74, 75, 76` (`924.125 - 926.125 MHz`)** were the quietest channels across the entire US ISM band (`CH 75` = `925.125 MHz`: `-96.2 dBm` avg, `-94 dBm` max).

---

## 8. Framed Binary Stream Architecture & Platform Gotchas

### 8.1 Never Pipe Raw E220 UART Bytes Straight into a Hardware Protocol Parser

In fringe RF conditions (foliage, multipath, extreme range), Subpacket 1 (`200 B`) of a `340 B` frame may pass LoRa CRC while Subpacket 2 (`140 B`) drops.

- **What Goes Wrong with Raw Passthrough:** If you stream those `200` headless/tailless bytes directly into a GNSS receiver (`UM982 COM2`) or MAVLink parser, the downstream parser sees a valid header announcing `Length = 340 B` and **swallows the first `140` bytes of the NEXT valid packet** (including its `0xD3` preamble!) as fake payload, failing CRC and **destroying intact single-packet frames that arrived right behind it** (`31%` collateral loss measured in Test `_021`).
- **The Store-and-Verify Sliding-Window Gatekeeper Pattern ([`e220_driver.cpp`](../src/e220_driver/e220_driver.cpp)):**
  1. Stage incoming UART bytes in a `1,536 B` buffer (`rx_stage`).
  2. Strip any leading non-preamble bytes (`!= 0xD3`).
  3. Validate header reserved bits, plausible length (`6..1023`), and plausible message ID.
  4. Wait until `rx_stage_len >= total_frame_len` (up to a **`260 ms` truncation timeout** — slightly longer than the `~80 ms` Subpacket 1 -> Subpacket 2 gap @ `38.4k`).
  5. Compute full **`CRC24Q`**:
     - **`CRC == 0`:** Forward the verified frame **atomically** to the downstream device!
     - **`CRC != 0` or `260 ms` Timeout:** Never forward to the downstream device; **shift `rx_stage` by `1 byte` and immediately rescan**, rescuing `100%` of any complete frames already sitting behind the truncated head!

### 8.2 ESP32-S3 Internal SPI-Flash (`FFat` / `NVS`) Erase vs. `115200` UART FIFO

- On ESP32-S3, writing or erasing a `4 KB` sector on **internal SPI Flash** (`FFat` or `Preferences/NVS`) disables non-IRAM interrupts (including Arduino `HardwareSerial` `UART0` RX ISR) for **`~35 - 60 ms`**.
- At `115200` baud (`11.52 B/ms`), an incoming `200 B` E220 subpacket overflows the ESP32-S3's `128-byte` hardware UART FIFO during a flash erase, clipping the burst at `128 B` (observed at the exact millisecond UI settings were saved to `FFat` at the end of Test `_024`).
- **Mitigation:** Do not write internal `FFat`/`NVS` periodically during active `115200` radio reception (log high-rate telemetry to `SD_MMC`, and commit settings to `FFat` only on explicit user config changes or idle intervals; the Store-and-Verify Gatekeeper cleanly strips any rare `128 B` clipped burst without downstream corruption).

### 8.3 Linux / Raspberry Pi Transmitter Daemon Checklist ([`e220_pacer.py`](../scripts/e220_pacer.py))

1. **Dedicate Hardware `PL011` UART (`/dev/ttyAMA0`):** Disable `serial-getty@ttyAMA0.service` and add `dtoverlay=disable-bt` in `/boot/firmware/config.txt` so Bluetooth does not steal `PL011`.
2. **Exclusive POSIX Port Lock (`TIOCEXCL`):** Open `serial.Serial(..., exclusive=True)` so manual scripts (`scan_e220_channels.py`, `set_e220_power.py`) cannot interleave bytes with a running `systemd` pacer daemon.
3. **Handle `EINTR` on `ser.flush()` (`termios.tcdrain()`):** When `systemd` sends `SIGTERM` while `ser.flush()` is draining UART bytes, Linux aborts `tcdrain()` with `termios.error: (4, 'Interrupted system call')` (`errno.EINTR`). Wrap `ser.flush()` in an `EINTR`-aware helper (`_safe_flush()`) and run `systemctl stop` before `install` in deployment scripts.
4. **Enforce Unix `LF` (`\n`) Line Endings:** Any `.py`, `.sh`, or `.service` file edited on Windows and copied to Linux **must** use Unix `LF` (`\n`, zero `\r`), or Linux `#!/usr/bin/env python3\r` shebang execution will fail with `env: 'python3\r': No such file or directory`.

---

## 9. Rapid Field Troubleshooting Matrix

| Observed Symptom | Actual Root Cause | 30-Second Verification / Fix |
| :--- | :--- | :--- |
| **`E220 READ OK` passes at boot, strong RSSI (`-65 dBm`), but `0` frames `> 200 B` succeed & bursts arrive `>= 950 ms` apart (`<= 1` pkt/s); `C0 C1 C2 C3` replies arrive `~515 ms` late.** | **Open `M0` connection** between MCU and E220 Pin 1! Internal pull-up floats `M0 = 1`: Mode 3 `(1,1)` works, but Mode 0 `(0,0)` drops into **WOR Mode 1 `(M1=0, M0=1)`** (`500 ms` sleep cycle). | Measure DC voltage **directly on E220 Pin 1 (`M0`)** during normal run: if `~3.3 V` instead of `0.0 V`, repair the connector/solder joint (or run the `AUX` mode-pulse continuity check in Section 3.4). |
| **`E220 READ OK` passes at boot, `<= 1` pkt/s RX ceiling, and `C0 C1 C2 C3` / RF TX never respond at all.** | **Open `M1` connection** between MCU and E220 Pin 2! Internal pull-up holds `M1 = 1` -> **WOR Mode 2 `(M1=1, M0=0)`**. | Measure DC voltage **directly on E220 Pin 2 (`M1`)** during normal run (`must be 0.0 V`). |
| **`E220 READ FAIL` (`0` bytes in Mode 3 @ `9600`).** | `TXD`/`RXD` not crossed (`E220_TXD -> MCU_RX`), wrong Mode 3 baud (must be `9600 8N1`), or `M0`/`M1` stuck `LOW`. | Probe `E220_TXD` idle voltage (`3.3 V`), verify `M0 = 3.3 V` & `M1 = 3.3 V` at module pins in Mode 3, and send `C1 00 06` @ `9600 8N1`. |
| **Transmitter pacing loop stalls `500 ms` on every small (`<= 200 B`) packet and builds huge TCP backlog.** | Waiting for `AUX` to fall `LOW` on `<= 200 B` writes (Regime 1: `<= 200 B` enters `LLCC68` in `< 0.5 ms` without holding `AUX` `LOW`). | Use Two-Regime pacing (Section 5.2): pace `<= 200 B` frames by `ToA(N) + 35 ms` without blocking on `AUX` fall. |
| **Single-packet (`<= 200 B`) frames succeed, but `201..400 B` frames lose Subpacket 2 when followed by another frame.** | Sending the next frame immediately when `AUX` rises `HIGH` on a `201..400 B` write (`AUX` rises when Subpacket 2 *starts* RF TX, not when it ends!). | After `AUX` rises `HIGH` on a `201..400 B` write, sleep `ToA(N - 200) + 35 ms` before writing the next frame (Section 5.3). |
| **Mode 0 `C0 C1 C2 C3 00 02` RSSI poll always times out even though `C1 00 ...` bytes appear in raw RX dump.** | Expecting a `5-byte` `C1 00 02 <noise> <pkt>` reply when `E220-900T30S` firmware returns **`4 bytes` (`C1 00 <noise> <pkt>`)**, OR `REG1` Bit 5 (`0x20`) is `0`. | Verify `REG1 & 0x20 != 0` and accept `4-byte` (`buf[2] != 0x02`) and `5-byte` (`buf[2] == 0x02`) replies (Section 6.2). |
| **Every received subpacket has `1` extra garbage byte at the end (`201 B` instead of `200 B`), breaking CRC.** | `REG3` Bit 7 (`0x80`, Append RSSI Byte) is enabled. | Clear `REG3` Bit 7 (`REG3 = 0x00`) and use `REG1` Bit 5 (`0x20`) for out-of-band RSSI queries instead. |
| **`AUX` locks `LOW` indefinitely (`> 1.5 s`) during continuous high-power TX.** | Supply voltage droop (`VCC` brownout during `800 mA` PA transient) or RF return-current EMI into module MCU. | Add `220..470 uF` bulk capacitor at E220 `VCC/GND`, relocate antenna via coax, and pulse `RST` `LOW` `50 ms` if `AUX` stays `LOW > 1.5 s`. |
