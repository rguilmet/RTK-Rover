#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module: scan_e220_channels.py
Description: Passive 902-928 MHz US ISM Ambient RF Noise Spectrum Scanner for Ebyte E220-900T30S.
             - Pauses e220-pacer.service if running (leaves it stopped if already stopped).
             - Sweeps CH 52..77 (902.125..927.125 MHz, or all CH 0..83 with --all-channels)
               using temporary RAM register writes (0xC2 = ZERO EEPROM wear, 0 mW RF TX).
             - Prints a compact (<79-col, non-wrapping) terminal table showing Min / Avg /
               Med / Max RSSI (dBm) for EVERY scanned channel, plus full ranking of all
               Ecowitt-safe channels.
             - Writes a complete timestamped CSV file (with user ownership preserved under
               sudo) ready for plotting in Excel, Google Sheets, or Matplotlib.
             - Pulses RESET (GPIO 22) in finally: to reload untouched EEPROM and restores
               e220-pacer.service if it was active prior to the scan.

Usage on Raspberry Pi:
  sudo python3 scan_e220_channels.py
  sudo python3 scan_e220_channels.py --passes 3 --samples 10 --csv office_scan.csv
  sudo python3 scan_e220_channels.py --all-channels

Author: Roland L. Guilmet & Antigravity
License: MIT
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import statistics
import subprocess
import sys
import time
from datetime import datetime
from typing import Optional

try:
    import serial
except ImportError:
    serial = None  # type: ignore

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None

__app_name__ = "scan_e220_channels"
__version__ = "1.1.0"
__date__ = "2026-10-30"

DEFAULT_PORT = "/dev/ttyAMA0"
PIN_M0 = 18
PIN_M1 = 17
PIN_AUX = 27
PIN_RST = 22

# FCC Part 15.247 US 902-928 MHz ISM legal channel bounds for E220-900T (850.125 + CH MHz)
US_ISM_CH_MIN = 52  # 902.125 MHz
US_ISM_CH_MAX = 77  # 927.125 MHz

# Protect office Ecowitt 915.000 MHz weather station (+/- 4.0 MHz guard band: 911.125..919.125 MHz)
ECOWITT_GUARD_MIN = 61  # 911.125 MHz (3.875 MHz below 915.0 MHz)
ECOWITT_GUARD_MAX = 69  # 919.125 MHz (4.125 MHz above 915.0 MHz)
ECOWITT_CENTER_CH = 65  # 915.125 MHz (125 kHz from 915.000 MHz Ecowitt FSK carrier)

INV_BAUD = {0: 1200, 1: 2400, 2: 4800, 3: 9600, 4: 19200, 5: 38400, 6: 57600, 7: 115200}
INV_AIR = {0: 2.4, 1: 2.4, 2: 2.4, 3: 4.8, 4: 9.6, 5: 19.2, 6: 38.4, 7: 62.5}


def wait_aux(pin_aux: int, timeout: float = 2.0) -> bool:
    if GPIO is None:
        return True
    t0 = time.monotonic()
    while (time.monotonic() - t0) < timeout:
        if GPIO.input(pin_aux) == GPIO.HIGH:
            time.sleep(0.003)
            return True
        time.sleep(0.001)
    return False


def is_service_active(service_name: str) -> bool:
    try:
        res = subprocess.run(
            ["systemctl", "is-active", service_name],
            capture_output=True,
            text=True,
            check=False,
        )
        return res.stdout.strip() == "active"
    except Exception:
        return False


def format_bar(avg_dbm: float, max_dbm: int, width: int = 18) -> str:
    """Render compact ASCII bar from -112 dBm (empty) to -62 dBm (full), marking peak with '|'."""
    lo, hi = -112.0, -62.0
    avg_norm = max(0.0, min(1.0, (avg_dbm - lo) / (hi - lo)))
    max_norm = max(0.0, min(1.0, (float(max_dbm) - lo) / (hi - lo)))
    avg_len = int(round(avg_norm * width))
    max_pos = min(width - 1, int(round(max_norm * width)))

    chars = ["#"] * avg_len + ["."] * (width - avg_len)
    if max_pos >= avg_len and max_dbm > (avg_dbm + 1.5):
        chars[max_pos] = "|"
    return "".join(chars)


def restore_sudo_file_owner(filepath: str) -> None:
    """If invoked via sudo, chown the generated CSV back to the invoking non-root user."""
    uid_str = os.environ.get("SUDO_UID")
    gid_str = os.environ.get("SUDO_GID")
    if uid_str and gid_str and hasattr(os, "chown"):
        try:
            os.chown(filepath, int(uid_str), int(gid_str))
        except Exception:
            pass


def percentile_90(vals: list[int]) -> float:
    if not vals:
        return 0.0
    s = sorted(vals)
    idx = min(len(s) - 1, int(math.ceil(0.90 * len(s))) - 1)
    return float(s[max(0, idx)])


def classify_zone(ch: int) -> str:
    if ch < US_ISM_CH_MIN or ch > US_ISM_CH_MAX:
        return "OUT_OF_US_ISM"
    if ch == ECOWITT_CENTER_CH:
        return "ECOWITT_915_CENTER"
    if ECOWITT_GUARD_MIN <= ch <= ECOWITT_GUARD_MAX:
        return "ECOWITT_GUARD"
    if ch < ECOWITT_GUARD_MIN:
        return "US_ISM_LOW_WING"
    return "US_ISM_HIGH_WING"


def run_scan(
    port: str,
    ch_start: int,
    ch_end: int,
    samples_per_pass: int,
    passes: int,
    csv_path: str,
    no_service_ctl: bool,
) -> int:
    if GPIO is None or serial is None:
        print("[ERROR] RPi.GPIO and pyserial are required on the Raspberry Pi.")
        return 1

    service_was_active = False
    if not no_service_ctl and is_service_active("e220-pacer"):
        service_was_active = True
        print("[*] Pausing e220-pacer.service for passive RF spectrum scan...", flush=True)
        subprocess.run(["systemctl", "stop", "e220-pacer"], check=False)
        time.sleep(0.25)

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    for p in (PIN_M0, PIN_M1, PIN_RST):
        GPIO.setup(p, GPIO.OUT)
    GPIO.setup(PIN_AUX, GPIO.IN, pull_up_down=GPIO.PUD_UP)
    GPIO.output(PIN_RST, GPIO.HIGH)

    ser: Optional[serial.Serial] = None
    orig_regs: Optional[bytes] = None
    scan_ts_iso = datetime.now().strftime("%Y-%m-%dT%H:%M:%S")

    try:
        try:
            ser = serial.Serial(port, 9600, timeout=0.4, exclusive=True)
        except TypeError:
            ser = serial.Serial(port, 9600, timeout=0.4)

        # 1. Read persistent EEPROM profile in Mode 3 (M0=1, M1=1 @ 9600)
        GPIO.output(PIN_M0, GPIO.HIGH)
        GPIO.output(PIN_M1, GPIO.HIGH)
        time.sleep(0.06)
        wait_aux(PIN_AUX, 2.0)

        for _ in range(3):
            ser.reset_input_buffer()
            ser.write(bytes([0xC1, 0x00, 0x08]))
            ser.flush()
            time.sleep(0.08)
            resp = ser.read(11)
            if len(resp) == 11 and resp[0] == 0xC1:
                orig_regs = resp[3:]
                break
            time.sleep(0.08)

        if not orig_regs or len(orig_regs) != 8:
            print("[ERROR] Could not read E220 registers on /dev/ttyAMA0 in Mode 3.")
            return 1

        orig_baud = INV_BAUD.get((orig_regs[2] >> 5) & 0x07, 115200)
        orig_air = INV_AIR.get(orig_regs[2] & 0x07, 19.2)
        orig_ch = orig_regs[4]
        orig_freq = 850.125 + orig_ch

        print(
            f"[*] {__app_name__} v{__version__} -- Passive Receive-Only Scan (0 mW RF TX, 0xC2 RAM tuning)\n"
            f"[*] Current EEPROM : CH {orig_ch} ({orig_freq:.3f} MHz) | UART={orig_baud} | Air={orig_air}k\n"
            f"[*] Sweep Range    : CH {ch_start} ({850.125+ch_start:.3f} MHz) .. CH {ch_end} ({850.125+ch_end:.3f} MHz) "
            f"[{ch_end - ch_start + 1} channels, {passes} pass(es) x {samples_per_pass} samples]\n"
            f"[*] CSV Output     : {os.path.abspath(csv_path)}",
            flush=True,
        )

        results: dict[int, list[int]] = {ch: [] for ch in range(ch_start, ch_end + 1)}

        for p_idx in range(1, passes + 1):
            sys.stdout.write(f"    Pass {p_idx}/{passes}: [")
            sys.stdout.flush()
            for ch in range(ch_start, ch_end + 1):
                # Enter Mode 3 @ 9600 and write temporary RAM register (0xC2 = zero EEPROM wear)
                GPIO.output(PIN_M0, GPIO.HIGH)
                GPIO.output(PIN_M1, GPIO.HIGH)
                time.sleep(0.030)
                wait_aux(PIN_AUX, 1.0)
                ser.baudrate = 9600

                temp = bytearray(orig_regs)
                temp[3] = temp[3] | 0x20            # Ensure REG1 Bit 5 (Mode 0 RSSI Reg) is ON
                temp[4] = ch & 0xFF                 # Candidate RF channel
                temp[5] = temp[5] & ~(0x80 | 0x40)  # Transparent mode, no extra RX byte

                ser.reset_input_buffer()
                ser.write(bytes([0xC2, 0x00, 0x08]) + bytes(temp))
                ser.flush()
                time.sleep(0.035)
                ack = ser.read(11)
                if len(ack) != 11 or ack[0] not in (0xC0, 0xC1, 0xC2):
                    ser.reset_input_buffer()
                    ser.write(bytes([0xC2, 0x00, 0x08]) + bytes(temp))
                    ser.flush()
                    time.sleep(0.050)
                    ser.read(11)

                # Switch to Mode 0 at operational UART baud to sample live receiver RSSI
                ser.baudrate = orig_baud
                GPIO.output(PIN_M0, GPIO.LOW)
                GPIO.output(PIN_M1, GPIO.LOW)
                time.sleep(0.030)
                wait_aux(PIN_AUX, 1.0)
                time.sleep(0.030)  # Allow LLCC68 synthesizer & RSSI filter to settle

                # Discard 1 warm-up read so register 0x00 strictly reflects the new channel
                ser.reset_input_buffer()
                ser.write(bytes([0xC0, 0xC1, 0xC2, 0xC3, 0x00, 0x01]))
                ser.flush()
                time.sleep(0.015)
                ser.read(4)
                time.sleep(0.010)

                for _ in range(samples_per_pass):
                    ser.reset_input_buffer()
                    ser.write(bytes([0xC0, 0xC1, 0xC2, 0xC3, 0x00, 0x01]))
                    ser.flush()
                    time.sleep(0.015)
                    r = ser.read(4)
                    if len(r) == 4 and r[0] == 0xC1 and r[1] == 0x00 and r[2] == 0x01:
                        results[ch].append(int(r[3]) - 256)
                    time.sleep(0.010)

                sys.stdout.write(".")
                sys.stdout.flush()
            sys.stdout.write("] done\n")
            sys.stdout.flush()

        # Compute full per-channel statistics & rank all Ecowitt-safe US ISM channels
        stats_by_ch: dict[int, dict[str, object]] = {}
        safe_ranking_pool: list[tuple[float, int]] = []

        for ch in range(ch_start, ch_end + 1):
            freq = round(850.125 + ch, 3)
            vals = results[ch]
            zone = classify_zone(ch)
            us_legal = US_ISM_CH_MIN <= ch <= US_ISM_CH_MAX
            eco_guard = ECOWITT_GUARD_MIN <= ch <= ECOWITT_GUARD_MAX
            eco_offset = round(freq - 915.000, 3)

            if vals:
                mn = min(vals)
                mx = max(vals)
                av = sum(vals) / len(vals)
                med = float(statistics.median(vals))
                sd = float(statistics.pstdev(vals)) if len(vals) > 1 else 0.0
                p90 = percentile_90(vals)
                if us_legal and not eco_guard:
                    # Composite score: average floor + 0.35 * peak (penalizes bursty interferers)
                    safe_ranking_pool.append((av + 0.35 * mx, ch))
            else:
                mn, mx, av, med, sd, p90 = 0, 0, 0.0, 0.0, 0.0, 0.0

            stats_by_ch[ch] = {
                "channel": ch,
                "freq_mhz": freq,
                "us_ism_legal": 1 if us_legal else 0,
                "ecowitt_guard": 1 if eco_guard else 0,
                "is_current": 1 if ch == orig_ch else 0,
                "ecowitt_offset_mhz": eco_offset,
                "count": len(vals),
                "min_dbm": mn,
                "avg_dbm": av,
                "med_dbm": med,
                "max_dbm": mx,
                "std_dbm": sd,
                "p90_dbm": p90,
                "zone": zone,
                "vals": vals,
                "safe_rank": None,
            }

        safe_ranking_pool.sort(key=lambda x: x[0])
        for rank_idx, (_, ch) in enumerate(safe_ranking_pool, start=1):
            stats_by_ch[ch]["safe_rank"] = rank_idx

        # 2. Write complete CSV file for plotting
        with open(csv_path, "w", newline="", encoding="ascii") as cf:
            writer = csv.writer(cf)
            writer.writerow([
                "timestamp_iso",
                "channel",
                "freq_mhz",
                "us_ism_legal",
                "ecowitt_guard",
                "is_current_channel",
                "ecowitt_offset_mhz",
                "sample_count",
                "min_dbm",
                "avg_dbm",
                "median_dbm",
                "p90_dbm",
                "max_dbm",
                "std_dbm",
                "safe_rank",
                "zone_label",
                "raw_samples_dbm",
            ])
            for ch in range(ch_start, ch_end + 1):
                st = stats_by_ch[ch]
                vals_list = st["vals"]  # type: ignore[assignment]
                has_data = bool(vals_list)
                writer.writerow([
                    scan_ts_iso,
                    ch,
                    f"{st['freq_mhz']:.3f}",
                    st["us_ism_legal"],
                    st["ecowitt_guard"],
                    st["is_current"],
                    f"{st['ecowitt_offset_mhz']:+.3f}",
                    st["count"],
                    st["min_dbm"] if has_data else "",
                    f"{st['avg_dbm']:.2f}" if has_data else "",
                    f"{st['med_dbm']:.2f}" if has_data else "",
                    f"{st['p90_dbm']:.2f}" if has_data else "",
                    st["max_dbm"] if has_data else "",
                    f"{st['std_dbm']:.2f}" if has_data else "",
                    st["safe_rank"] if st["safe_rank"] is not None else "",
                    st["zone"],
                    ";".join(str(v) for v in vals_list),  # type: ignore[union-attr]
                ])

        restore_sudo_file_owner(csv_path)

        # 3. Print compact (<79-column, zero line-wrap) full channel RSSI table to terminal
        print("\nFULL CHANNEL RSSI SPECTRUM (Every Channel Scanned):")
        print("-" * 78)
        print("CH | Freq MHz |  Min |   Avg |  Med |  Max | Bar(-112..-62dBm)  | Status/Rank")
        print("---+----------+------+-------+------+------+--------------------+-------------")

        for ch in range(ch_start, ch_end + 1):
            st = stats_by_ch[ch]
            freq = float(st["freq_mhz"])  # type: ignore[arg-type]
            vals_list = st["vals"]        # type: ignore[assignment]
            if not vals_list:
                print(f"{ch:2d} | {freq:8.3f} |  N/A |   N/A |  N/A |  N/A | {'?'*18} | NO DATA")
                continue

            mn = int(st["min_dbm"])       # type: ignore[arg-type]
            av = float(st["avg_dbm"])     # type: ignore[arg-type]
            med = float(st["med_dbm"])    # type: ignore[arg-type]
            mx = int(st["max_dbm"])       # type: ignore[arg-type]
            rank = st["safe_rank"]
            bar = format_bar(av, mx, 18)

            if ch == ECOWITT_CENTER_CH:
                tag = "ECOWITT-915"
            elif ECOWITT_GUARD_MIN <= ch <= ECOWITT_GUARD_MAX:
                tag = "ECO-GUARD"
            elif ch < US_ISM_CH_MIN or ch > US_ISM_CH_MAX:
                tag = "NON-US-ISM"
            elif rank is not None:
                tag = f"SAFE #{rank:<2d}"
            else:
                tag = ""

            if ch == orig_ch:
                tag = f"*CUR* {tag}".strip()

            print(f"{ch:2d} | {freq:8.3f} | {mn:4d} | {av:5.1f} | {med:4.0f} | {mx:4d} | {bar} | {tag}")

        print("-" * 78)

        # 4. Print complete ranked list of ALL Ecowitt-safe channels + CSV path
        valid_avgs = [float(stats_by_ch[c]["avg_dbm"]) for c in range(ch_start, ch_end + 1) if stats_by_ch[c]["vals"]]  # type: ignore[arg-type]
        if valid_avgs:
            cur_st = stats_by_ch.get(orig_ch)
            cur_summary = (
                f"CH {orig_ch} ({orig_freq:.3f} MHz) Avg={float(cur_st['avg_dbm']):.1f} dBm "  # type: ignore[arg-type]
                f"(Min={cur_st['min_dbm']}, Max={cur_st['max_dbm']})"
                if (cur_st and cur_st["vals"])
                else f"CH {orig_ch} (not in sweep range)"
            )
            print(f"\n[SUMMARY]")
            print(f"  Band Avg Range  : {min(valid_avgs):.1f} dBm (quietest) .. {max(valid_avgs):.1f} dBm (loudest)")
            print(f"  Current Channel : {cur_summary}")
            print(f"  CSV Saved To    : {os.path.abspath(csv_path)}")

            if safe_ranking_pool:
                print("\n  All Ecowitt-Safe US-ISM Channels Ranked Quietest -> Loudest:")
                for r_idx, (_, ch) in enumerate(safe_ranking_pool, start=1):
                    st = stats_by_ch[ch]
                    f_mhz = float(st["freq_mhz"])    # type: ignore[arg-type]
                    av = float(st["avg_dbm"])        # type: ignore[arg-type]
                    mn = int(st["min_dbm"])          # type: ignore[arg-type]
                    mx = int(st["max_dbm"])          # type: ignore[arg-type]
                    sd = float(st["std_dbm"])        # type: ignore[arg-type]
                    off = float(st["ecowitt_offset_mhz"])  # type: ignore[arg-type]
                    cur_mark = " <-- CURRENT" if ch == orig_ch else ""
                    print(
                        f"    #{r_idx:2d}: CH {ch:2d} ({f_mhz:7.3f} MHz | {off:+5.1f} MHz vs 915) "
                        f"Avg={av:6.1f} dBm [Min={mn:4d}, Max={mx:4d}, Std={sd:3.1f}]{cur_mark}"
                    )

    finally:
        print("\n[*] Pulsing E220 RESET (GPIO 22) to reload untouched persistent EEPROM...", flush=True)
        try:
            if ser and ser.is_open:
                ser.close()
        except Exception:
            pass

        try:
            GPIO.output(PIN_M0, GPIO.LOW)
            GPIO.output(PIN_M1, GPIO.LOW)
            GPIO.output(PIN_RST, GPIO.LOW)
            time.sleep(0.060)
            GPIO.output(PIN_RST, GPIO.HIGH)
            time.sleep(0.100)
            wait_aux(PIN_AUX, 2.0)
            GPIO.cleanup()
        except Exception:
            pass

        if service_was_active and not no_service_ctl:
            print("[*] Resuming e220-pacer.service...", flush=True)
            subprocess.run(["systemctl", "start", "e220-pacer"], check=False)
            time.sleep(0.4)
            state = "ACTIVE" if is_service_active("e220-pacer") else "INACTIVE"
            print(f"[+] e220-pacer.service is {state}.", flush=True)

    return 0


def main() -> int:
    default_csv = f"e220_rssi_scan_{datetime.now().strftime('%Y-%m-%d_%H-%M-%S')}.csv"
    parser = argparse.ArgumentParser(
        description="Passive 902-928 MHz US ISM Channel Noise Scanner + CSV Exporter for Ebyte E220-900T30S.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help="E220 serial port")
    parser.add_argument("--ch-start", type=int, default=US_ISM_CH_MIN, help="Start channel (52 = 902.125 MHz)")
    parser.add_argument("--ch-end", type=int, default=US_ISM_CH_MAX, help="End channel (77 = 927.125 MHz)")
    parser.add_argument("--all-channels", action="store_true", help="Sweep all 84 hardware channels (CH 0..83 = 850.125..933.125 MHz)")
    parser.add_argument("--samples", type=int, default=8, help="RSSI samples per channel per pass")
    parser.add_argument("--passes", type=int, default=2, help="Number of full band passes")
    parser.add_argument("--csv", default=default_csv, help="Output CSV filename for plotting")
    parser.add_argument("--no-service-ctl", action="store_true", help="Do not stop/start e220-pacer.service")
    args = parser.parse_args()

    ch_start = 0 if args.all_channels else args.ch_start
    ch_end = 83 if args.all_channels else args.ch_end

    if ch_start < 0 or ch_end > 83 or ch_start > ch_end:
        print("[ERROR] Channel range must satisfy 0 <= ch-start <= ch-end <= 83.")
        return 1

    return run_scan(
        port=args.port,
        ch_start=ch_start,
        ch_end=ch_end,
        samples_per_pass=max(1, args.samples),
        passes=max(1, args.passes),
        csv_path=args.csv,
        no_service_ctl=args.no_service_ctl,
    )


if __name__ == "__main__":
    sys.exit(main())
