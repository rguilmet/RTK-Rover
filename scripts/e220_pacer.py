#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module: e220_pacer.py
Description: Datasheet-Accurate Hardware-Paced Ebyte E220-900T30S LoRa Driver for RTKBase.
             Parses and CRC24Q-validates raw RTCM3 streams into complete frames and
             transmits each frame intact through the E220 400-byte FIFO using Ebyte
             E220 Manual Section 5.6.2 two-regime AUX + Time-on-Air pacing:
               - Single-subpacket frames (<= 200B): Transfer directly into an idle
                 LLCC68 FIFO without holding AUX LOW; paced deterministically by
                 calibrated LoRa Time-on-Air + receiver UART drain guard delay.
               - Two-subpacket frames (201..400B): Hold AUX LOW while Subpacket 1
                 (200B) transmits over the air, then raise AUX HIGH the instant
                 Subpacket 2 (len - 200) enters the LLCC68 FIFO and begins RF TX;
                 paced by AUX rising edge + Subpacket 2 Time-on-Air + guard delay.
             Includes GPIO 22 hardware reset watchdog if AUX ever locks LOW (>1.5s),
             POSIX exclusive UART locking (TIOCEXCL), startup EEPROM verification,
             and Mode 0 ambient RF noise floor probing (REG1 Bit 5 = 0x20) on CH 75
             (925.125 MHz, Rank #1 cleanest US ISM channel clear of 915 MHz Ecowitt).

Hardware Connections (Raspberry Pi BCM Pins, matching setup_e220.py):
  - E220 M0:    GPIO 18 (BCM) -> Pin 12
  - E220 M1:    GPIO 17 (BCM) -> Pin 11
  - E220 AUX:   GPIO 27 (BCM) -> Pin 13 (Active HIGH = Buffer Empty, LOW = Busy)
  - E220 RESET: GPIO 22 (BCM) -> Pin 15 (Active LOW hardware reset)
  - E220 TX/RX: /dev/ttyAMA0 (UART0 GPIO 14/15)

Author: Roland L. Guilmet & Antigravity
License: MIT
"""

from __future__ import annotations

import argparse
import errno
import gzip
import json
import os
import select
import shutil
import signal
import socket
import struct
import sys
import time
from typing import Optional

try:
    import serial
except ImportError:
    serial = None  # type: ignore

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None

try:
    import pty
except ImportError:
    pty = None

try:
    import termios
    _TERMIOS_ERRORS: tuple[type[BaseException], ...] = (termios.error,)
except ImportError:
    _TERMIOS_ERRORS = ()


__app_name__ = "e220_pacer"
__version__ = "3.1.0"
__date__ = "2026-11-04"

# Force unbuffered output so systemd journalctl displays entries in real time
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(line_buffering=True)
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(line_buffering=True)

# Default Pin Mapping (matches setup_e220.py)
DEFAULT_PIN_M0 = 18
DEFAULT_PIN_M1 = 17
DEFAULT_PIN_AUX = 27
DEFAULT_PIN_RST = 22
DEFAULT_SERIAL_PORT = "/dev/ttyAMA0"
DEFAULT_BAUD = 115200
DEFAULT_PACKET_SIZE = 200

# Inter-frame guard delay after RF completion (allows Rover E220 17.4ms @ 115200 UART drain + 17.6ms margin)
DEFAULT_GUARD_DELAY_S = 0.035  # 35 ms

# Hardware Flow Control & Watchdog Thresholds
AUX_SINGLE_PROBE_S = 0.008     # 8 ms probe for <=200B frames (normally stays HIGH per Sec 5.6.2)
AUX_MULTI_FALL_TIMEOUT_S = 0.050  # 50 ms max wait for AUX falling edge on >200B frames (typical: 2.0 ms)
AUX_RISE_WATCHDOG_S = 1.500    # 1.5 s max AUX LOW before triggering GPIO 22 hardware reset

# Lookups for Auto-Config / Register Decode
BAUD_RATES = {1200: 0b000, 2400: 0b001, 4800: 0b010, 9600: 0b011, 19200: 0b100, 38400: 0b101, 57600: 0b110, 115200: 0b111}
AIR_DATA_RATES = {2.4: 0b010, 4.8: 0b011, 9.6: 0b100, 19.2: 0b101, 38.4: 0b110, 62.5: 0b111}
SUBPACKET_SIZES = {200: 0b00, 128: 0b01, 64: 0b10, 32: 0b11}
POWER_LEVELS = {30: 0b00, 27: 0b01, 24: 0b10, 21: 0b11}

INV_BAUD = {v: k for k, v in BAUD_RATES.items()}
INV_AIR = {0b000: 2.4, 0b001: 2.4, **{v: k for k, v in AIR_DATA_RATES.items()}}
INV_PKT = {v: k for k, v in SUBPACKET_SIZES.items()}
INV_PWR = {v: k for k, v in POWER_LEVELS.items()}

running = True


def sig_handler(signum, frame):
    global running
    running = False


def rtcm_crc24q(data: bytes) -> int:
    """Compute 24-bit RTCM3 CRC24Q over full frame (header + payload + 3-byte CRC). Returns 0 if valid."""
    crc = 0
    for b in data:
        crc ^= (b << 16)
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
        crc &= 0xFFFFFF
    return crc


# =============================================================================
# Bi-Directional LoRa Protocol Definitions (0xAA 0x55 Framing)
# =============================================================================
ROVER_SYNC = b"\xAA\x55"
MSG_ID_TELEMETRY = 0x01
MSG_ID_REQ_SWITCH = 0x02
MSG_ID_ACK_SWITCH = 0x82

# 39-Byte Uplink Telemetry Frame Format:
# sync(2s), mid(B), plen(B), lat(d), lon(d), height(f), h_acc_mm(H), fix(B),
# sats_m(B), sats_s(B), hdg_cdeg(H), bat_pct(B), bat_mv(H), rssi(b), noise(b), seq(B), crc(H)
FMT_TELEMETRY = "<2sBBddfHBBBHBHbbBH"

# 10-Byte Remote Air-Rate Switch Request Format:
# sync(2s), mid(B), plen(B), target_code(B), target_chan(B), req_id(H), crc(H)
FMT_REQ_SWITCH = "<2sBBBBHH"

# 11-Byte Remote Air-Rate Switch Acknowledge Format:
# sync(2s), mid(B), plen(B), req_id(H), status(B), delay_ms(H), crc(H)
FMT_ACK_SWITCH = "<2sBBHBHH"


def crc16_ccitt(data: bytes) -> int:
    """Standard CRC16-CCITT (False: poly 0x1021, init 0xFFFF, no final XOR)."""
    crc = 0xFFFF
    for b in data:
        crc ^= (b << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


class E220Pacer:
    def __init__(
        self,
        port: str = DEFAULT_SERIAL_PORT,
        baud: int = DEFAULT_BAUD,
        packet_size: int = DEFAULT_PACKET_SIZE,
        guard_delay_s: float = DEFAULT_GUARD_DELAY_S,
        pin_m0: int = DEFAULT_PIN_M0,
        pin_m1: int = DEFAULT_PIN_M1,
        pin_aux: int = DEFAULT_PIN_AUX,
        pin_rst: int = DEFAULT_PIN_RST,
        auto_config: bool = False,
        air_rate_kbps: float = 19.2,
        power_dbm: int = 21,
        channel: int = 75,
        verbose: bool = False,
        quiet: bool = False,
        log_dir: str = "~/rover_logs",
        log_retention_days: int = 30,
        max_log_mb: int = 50,
        compress_logs: bool = True,
        no_csv: bool = False,
    ):
        self.port_name = port
        self.baud = baud
        self.packet_size = packet_size
        self.guard_delay_s = guard_delay_s
        self.pin_m0 = pin_m0
        self.pin_m1 = pin_m1
        self.pin_aux = pin_aux
        self.pin_rst = pin_rst
        self.auto_config = auto_config
        self.air_rate_kbps = air_rate_kbps
        self.power_dbm = power_dbm
        self.channel = channel
        self.verbose = verbose
        self.quiet = quiet
        self.ser: Optional[serial.Serial] = None
        self.gpio_available = GPIO is not None
        self.rssi_reg_enabled = False

        # Bi-Directional LoRa Uplink State
        self.uplink_buf = bytearray()
        self.last_rover_telemetry: dict = {}
        self.last_rover_rx_time = 0.0
        self.uplink_packets_received = 0
        self.uplink_crc_errors = 0
        self.rate_switch_count = 0
        self.watchdog_rate_switch_time = 0.0
        self.watchdog_rate_revert_time = 0.0
        self.fallback_air_rate = 19.2
        self.active_air_rate = air_rate_kbps

        # Log Retention & Storage
        resolved_log_dir = os.path.expanduser(log_dir)
        if resolved_log_dir.startswith("/root") or resolved_log_dir == "/root":
            if os.path.isdir("/home/pi"):
                resolved_log_dir = "/home/pi/rover_logs"
            elif os.environ.get("SUDO_USER") and os.path.isdir(f"/home/{os.environ.get('SUDO_USER')}"):
                resolved_log_dir = f"/home/{os.environ.get('SUDO_USER')}/rover_logs"
        self.log_dir = resolved_log_dir
        self.log_retention_days = log_retention_days
        self.max_log_mb = max_log_mb
        self.compress_logs = compress_logs
        self.enable_csv = not no_csv
        self.current_log_date = ""
        self.current_log_file = None

        # Telemetry counters
        self.total_bytes_in = 0
        self.total_bytes_out = 0
        self.total_packets_sent = 0
        self.total_subpackets_sent = 0
        self.total_wait_time_s = 0.0
        self.watchdog_resets = 0
        self.src_crc_errors = 0
        self.src_orphan_bytes = 0
        self.backlog_shed_frames = 0
        self.rtcm_stats: dict[int, int] = {}

    def estimate_subpacket_air_s(self, n_bytes: int) -> float:
        """
        Estimate single-subpacket LoRa Time-on-Air (seconds) for E220-900T30S (LLCC68).
        Calibrated directly against Test _022 hardware AUX_low telemetry (t_uart + AUX_low = 165.0 ms
        -> 148.1 ms true Time-on-Air for 200B @ 19.2 kbps; ~0.68 net payload coding efficiency after
        LoRa preamble, explicit header, CRC16, FEC coding rate, and 3-char UART idle detect):
          - @ 38.4 kbps: 200B -> 82.2 ms, 144B -> 65.0 ms, 122B -> 58.3 ms, 27B -> 29.2 ms
          - @ 19.2 kbps: 200B -> 148.4 ms, 144B -> 114.0 ms, 122B -> 100.6 ms, 27B -> 42.3 ms
        """
        if n_bytes <= 0:
            return 0.0
        rate_bps = max(self.air_rate_kbps, 2.4) * 1000.0
        return 0.016 + ((float(n_bytes) + 16.0) * 8.0) / (rate_bps * 0.68)

    @staticmethod
    def _safe_flush(ser_obj: "serial.Serial") -> bool:
        """Drain UART TX buffer via tcdrain(), returning False cleanly if interrupted by SIGTERM/SIGINT (EINTR)."""
        while True:
            try:
                ser_obj.flush()
                return True
            except _TERMIOS_ERRORS + (OSError,) as e:
                err_no = e.args[0] if getattr(e, "args", None) else None
                if err_no == errno.EINTR:
                    if not running:
                        return False
                    continue
                if not running:
                    return False
                raise

    def _open_operational_uart(self) -> None:
        if serial is None:
            raise RuntimeError("pyserial is not installed.")
        if self.ser and self.ser.is_open:
            self.ser.close()
        try:
            self.ser = serial.Serial(self.port_name, self.baud, timeout=0.05, exclusive=True)
            excl_tag = "exclusive=True (TIOCEXCL locked)"
        except TypeError:
            self.ser = serial.Serial(self.port_name, self.baud, timeout=0.05)
            excl_tag = "exclusive=N/A"
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        print(f"[UART] Opened {self.port_name} @ {self.baud} bps ({excl_tag})", flush=True)

    @staticmethod
    def _decode_registers_str(regs: bytes) -> str:
        if len(regs) < 6:
            return "INVALID"
        addh, addl, r0, r1, ch, r3 = regs[0], regs[1], regs[2], regs[3], regs[4], regs[5]
        u_baud = INV_BAUD.get((r0 >> 5) & 0x07, 0)
        a_rate = INV_AIR.get(r0 & 0x07, 0.0)
        pkt_sz = INV_PKT.get((r1 >> 6) & 0x03, 0)
        tx_pwr = INV_PWR.get(r1 & 0x03, 0)
        rssi_reg = "ON" if (r1 & 0x20) else "OFF"
        mode_s = "Fixed" if (r3 & 0x40) else "Transparent"
        lbt_s = "ON" if (r3 & 0x10) else "OFF"
        rssi_byte = "ON" if (r3 & 0x80) else "OFF"
        freq = 850.125 + ch
        return (
            f"ADD=0x{addh:02X}{addl:02X} REG0=0x{r0:02X} REG1=0x{r1:02X} CHAN=0x{ch:02X}({ch}) REG3=0x{r3:02X} -> "
            f"UART={u_baud} | Air={a_rate}k | Pkt={pkt_sz}B | Pwr={tx_pwr}dBm | "
            f"RSSI_Reg={rssi_reg} | Freq={freq:.3f}MHz | Mode={mode_s} | LBT={lbt_s} | RSSI_Byte={rssi_byte}"
        )

    def inspect_and_configure_eeprom(self) -> bool:
        """Always reads Base E220 EEPROM in Mode 3 on startup; programs target profile if --auto-config is enabled."""
        if not self.gpio_available or serial is None:
            print("[EEPROM] Skipped (GPIO or pyserial not available).", flush=True)
            return False

        baud_bits = BAUD_RATES.get(self.baud, 0b111)
        air_bits = AIR_DATA_RATES.get(self.air_rate_kbps, 0b110)
        pkt_bits = SUBPACKET_SIZES.get(self.packet_size, 0b00)
        pwr_bits = POWER_LEVELS.get(self.power_dbm, 0b11)

        reg0 = (baud_bits << 5) | (0b00 << 3) | air_bits
        # REG1 Bit 5 (0x20) = Enable Mode 0 Ambient Noise & Last Packet RSSI Register (C0 C1 C2 C3)
        reg1 = (pkt_bits << 6) | 0x20 | pwr_bits
        chan = self.channel & 0xFF
        # REG3: Bit 7=0 (No extra byte appended to RX), Bit 6=0 (Transparent Mode), Bit 4=0 (LBT off)
        reg3 = 0x00
        target_regs = bytes([0x00, 0x00, reg0, reg1, chan, reg3, 0x00, 0x00])

        print(f"[EEPROM] Target Profile: {self._decode_registers_str(target_regs)}", flush=True)

        # 1. Enter Configuration Mode 3 (M0=1, M1=1 @ 9600 8N1)
        GPIO.output(self.pin_m0, GPIO.HIGH)
        GPIO.output(self.pin_m1, GPIO.HIGH)
        time.sleep(0.06)
        self.wait_aux(2.0)

        try:
            cfg_ser = serial.Serial(self.port_name, 9600, timeout=0.8, exclusive=True)
        except TypeError:
            cfg_ser = serial.Serial(self.port_name, 9600, timeout=0.8)

        try:
            current_regs = b""
            for _ in range(2):
                cfg_ser.reset_input_buffer()
                cfg_ser.write(bytes([0xC1, 0x00, 0x08]))
                self._safe_flush(cfg_ser)
                time.sleep(0.08)
                res = cfg_ser.read(11)
                if len(res) == 11 and res[0] == 0xC1:
                    current_regs = res[3:]
                    break
                time.sleep(0.08)

            if current_regs:
                print(f"[EEPROM] Current Radio : {self._decode_registers_str(current_regs)}", flush=True)
                self.rssi_reg_enabled = bool(current_regs[3] & 0x20)
            else:
                print("[WARN] Could not read Base E220 registers in Mode 3 (check UART/power/M0/M1).", flush=True)

            needs_update = (current_regs != target_regs)
            if needs_update and self.auto_config:
                write_ok = False
                for attempt in range(1, 4):
                    print(f"[AUTO-CONFIG] Programming target EEPROM profile (attempt {attempt}/3)...", flush=True)
                    cfg_ser.reset_input_buffer()
                    cfg_ser.reset_output_buffer()
                    cfg_ser.write(bytes([0xC0, 0x00, 0x08]) + target_regs)
                    self._safe_flush(cfg_ser)
                    time.sleep(0.10)
                    ack = cfg_ser.read(11)
                    if len(ack) == 11 and ack[0] in (0xC0, 0xC1):
                        write_ok = True
                        break
                    time.sleep(0.15)

                time.sleep(0.05)
                cfg_ser.reset_input_buffer()
                cfg_ser.write(bytes([0xC1, 0x00, 0x08]))
                self._safe_flush(cfg_ser)
                time.sleep(0.08)
                verify = cfg_ser.read(11)
                if len(verify) == 11 and verify[0] == 0xC1 and verify[3:] == target_regs:
                    self.rssi_reg_enabled = bool(verify[6] & 0x20)
                    print(f"[AUTO-CONFIG] VERIFIED! EEPROM matches target: {verify[3:].hex().upper()}", flush=True)
                else:
                    actual = verify[3:].hex().upper() if len(verify) == 11 else f"LEN_{len(verify)}"
                    print(
                        f"[ERROR] EEPROM readback mismatch (write_ack={write_ok}): "
                        f"Actual={actual} vs Target={target_regs.hex().upper()}",
                        flush=True,
                    )
            elif needs_update and not self.auto_config:
                print("[WARN] Base E220 EEPROM differs from CLI target and --auto-config was NOT set!", flush=True)
            else:
                print("[EEPROM] Base E220 EEPROM already matches target profile.", flush=True)
        finally:
            cfg_ser.close()

        # 2. Return to Transparent Mode 0 (M0=0, M1=0)
        GPIO.output(self.pin_m0, GPIO.LOW)
        GPIO.output(self.pin_m1, GPIO.LOW)
        time.sleep(0.06)
        self.wait_aux(2.0)
        time.sleep(0.04)
        return True

    def probe_ambient_noise(self) -> None:
        """Query Channel ambient RF noise floor in Mode 0 via C0 C1 C2 C3 00 01 (requires REG1 Bit 5 = 0x20)."""
        if not self.ser or not self.ser.is_open or not self.rssi_reg_enabled:
            return

        samples_dbm: list[int] = []
        for _ in range(3):
            self.wait_aux(0.5)
            self.ser.reset_input_buffer()
            self.ser.write(bytes([0xC0, 0xC1, 0xC2, 0xC3, 0x00, 0x01]))
            self._safe_flush(self.ser)
            time.sleep(0.030)
            resp = self.ser.read(4)
            if len(resp) == 4 and resp[0] == 0xC1 and resp[1] == 0x00 and resp[2] == 0x01:
                samples_dbm.append(int(resp[3]) - 256)
            elif len(resp) == 3 and resp[0] == 0xC1 and resp[1] == 0x00:
                samples_dbm.append(int(resp[2]) - 256)
            time.sleep(0.035)

        self.ser.reset_input_buffer()
        freq_mhz = 850.125 + self.channel
        if samples_dbm:
            avg_dbm = sum(samples_dbm) / len(samples_dbm)
            print(
                f"[RF PROBE] Base CH {self.channel} ({freq_mhz:.3f}MHz) Ambient Noise Floor: "
                f"avg={avg_dbm:.1f} dBm (samples={samples_dbm} dBm)",
                flush=True,
            )
            if avg_dbm > -88.0:
                print(
                    f"[WARN] High ambient 900MHz RF noise ({avg_dbm:.1f} dBm > -88 dBm) at Base! "
                    f"Check antenna coax/separation from Pi 4B / USB3 / switching PSU.",
                    flush=True,
                )
        else:
            print("[RF PROBE] Mode 0 RSSI register probe returned no C1 reply.", flush=True)

    def hardware_reset_e220(self, reason: str) -> None:
        """Pulse E220 RESET (GPIO 22) LOW to recover from a wedged LLCC68 / stuck-LOW AUX state."""
        self.watchdog_resets += 1
        print(
            f"[ERROR] WATCHDOG #{self.watchdog_resets}: {reason} -> Pulsing E220 RESET (GPIO {self.pin_rst}) LOW for 50ms...",
            flush=True,
        )
        if self.gpio_available:
            GPIO.output(self.pin_m0, GPIO.LOW)
            GPIO.output(self.pin_m1, GPIO.LOW)
            GPIO.output(self.pin_rst, GPIO.LOW)
            time.sleep(0.050)
            GPIO.output(self.pin_rst, GPIO.HIGH)
            time.sleep(0.100)
            ok = self.wait_aux(timeout=2.0)
            aux_state = "HIGH" if (GPIO.input(self.pin_aux) == GPIO.HIGH) else "STILL_LOW"
            print(f"[RECOVERY] E220 hardware reset complete (wait_ok={ok}, AUX={aux_state})", flush=True)
        if self.ser and self.ser.is_open:
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()

    def setup_hardware(self) -> None:
        print(f"[INIT] {__app_name__} v{__version__} ({__date__}) initializing hardware...", flush=True)
        print(f"[LOG] Daily CSV logging directory: {self.log_dir}", flush=True)

        if self.gpio_available:
            GPIO.setwarnings(False)
            GPIO.setmode(GPIO.BCM)
            for pin in (self.pin_m0, self.pin_m1, self.pin_rst):
                GPIO.setup(pin, GPIO.OUT)
            GPIO.setup(self.pin_aux, GPIO.IN, pull_up_down=GPIO.PUD_UP)
            GPIO.output(self.pin_rst, GPIO.HIGH)
            GPIO.output(self.pin_m0, GPIO.LOW)
            GPIO.output(self.pin_m1, GPIO.LOW)
            time.sleep(0.04)
            if not self.wait_aux(timeout=1.5):
                self.hardware_reset_e220("AUX stuck LOW at startup")

            self.inspect_and_configure_eeprom()

            if self.verbose:
                print(
                    f"[HW] GPIO ready: M0={self.pin_m0}, M1={self.pin_m1}, AUX={self.pin_aux}, RST={self.pin_rst}",
                    flush=True,
                )
        else:
            print("[WARN] RPi.GPIO not available - running in calculated Time-on-Air pacing mode.", flush=True)

        self._open_operational_uart()
        self.probe_ambient_noise()

    def cleanup_hardware(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()
        if self.gpio_available:
            GPIO.cleanup()
        if self.current_log_file:
            try:
                self.current_log_file.close()
            except Exception:
                pass
        print("[HW] Hardware cleaned up.", flush=True)

    def wait_aux(self, timeout: float = AUX_RISE_WATCHDOG_S) -> bool:
        """Wait until E220 AUX pin is HIGH (UART FIFO empty) plus 2ms datasheet guard."""
        if not self.gpio_available:
            return True
        start = time.monotonic()
        while (time.monotonic() - start) < timeout:
            if GPIO.input(self.pin_aux) == GPIO.HIGH:
                time.sleep(0.002)
                return True
            time.sleep(0.0005)
        return False

    def send_packet(self, chunk: bytes) -> tuple[int, str, float]:
        """
        Feed an intact RTCM3 frame into the E220 and pace strictly per Ebyte E220 Section 5.6.2.
        Returns (num_subpackets, timing_detail_str, total_elapsed_s).
        """
        if not self.ser or not self.ser.is_open or not chunk:
            return (0, "SKIP", 0.0)

        t0 = time.monotonic()
        total_len = len(chunk)

        # Fallback safety for any oversized (> 2 * packet_size = >400B) frame so 400B FIFO never overflows
        if total_len > (self.packet_size * 2):
            sub_cnt = 0
            offset = 0
            while offset < total_len:
                part = chunk[offset : offset + self.packet_size]
                offset += len(part)
                c, _, _ = self.send_packet(part)
                sub_cnt += c
            dt = time.monotonic() - t0
            return (sub_cnt, f"chunked>{self.packet_size * 2}B", dt)

        # 1. Ensure E220 buffer is idle before writing; reset module if wedged
        if not self.wait_aux(timeout=AUX_RISE_WATCHDOG_S):
            self.hardware_reset_e220(f"AUX stuck LOW pre-TX before {total_len}B")

        # 2. Write complete RTCM3 frame atomically to /dev/ttyAMA0
        self.ser.write(chunk)
        if not self._safe_flush(self.ser) and not running:
            return (0, "STOP", time.monotonic() - t0)
        t_after_flush = time.monotonic()

        # 3. Pace over-the-air completion according to subpacket count
        if total_len <= self.packet_size:
            # -----------------------------------------------------------------
            # REGIME A: Single-Subpacket Frame (<= 200B, e.g. 1006, 1230, 1084)
            # Per E220 Sec 5.6.2, <200B transfers immediately into an idle LLCC68
            # FIFO without holding AUX LOW. Do NOT block 500ms waiting for AUX=0!
            # -----------------------------------------------------------------
            num_subpkts = 1
            est_air_s = self.estimate_subpacket_air_s(total_len)
            aux_dropped = False
            aux_low_s = 0.0

            if self.gpio_available:
                while (time.monotonic() - t_after_flush) < AUX_SINGLE_PROBE_S:
                    if GPIO.input(self.pin_aux) == GPIO.LOW:
                        aux_dropped = True
                        break
                    time.sleep(0.0005)

                if aux_dropped:
                    t_low = time.monotonic()
                    if not self.wait_aux(timeout=AUX_RISE_WATCHDOG_S):
                        self.hardware_reset_e220(f"AUX stuck LOW >{AUX_RISE_WATCHDOG_S}s on 1-pkt {total_len}B")
                    aux_low_s = time.monotonic() - t_low

            elapsed_since_flush = time.monotonic() - t_after_flush
            rem_air_s = est_air_s - elapsed_since_flush
            if rem_air_s > 0.0:
                time.sleep(rem_air_s)

            time.sleep(self.guard_delay_s)
            detail = (
                f"1 pkt | EstAir={est_air_s*1000:.1f}ms"
                + (f"(AUX_low={aux_low_s*1000:.1f}ms)" if aux_dropped else "")
                + f" + Guard={self.guard_delay_s*1000:.0f}ms"
            )
        else:
            # -----------------------------------------------------------------
            # REGIME B: Two-Subpacket Frame (201..400B, e.g. 1074, 1094)
            # Subpacket 1 (200B) starts RF TX while Subpacket 2 (tail_len) sits
            # in the UART buffer holding AUX LOW (~109ms @ 19.2k).
            # CRITICAL (Sec 5.6.2): AUX rises HIGH the instant Subpacket 2 moves
            # into the LLCC68 FIFO and BEGINS transmitting over the air! We MUST
            # wait tail_air_s + guard_delay_s after AUX rises so the next frame
            # does not collide with Subpacket 2.
            # -----------------------------------------------------------------
            num_subpkts = 2
            tail_len = total_len - self.packet_size
            head_est_s = self.estimate_subpacket_air_s(self.packet_size)
            tail_air_s = self.estimate_subpacket_air_s(tail_len)
            aux_dropped = False
            drop_ms = 0.0
            aux_low_s = 0.0

            if self.gpio_available:
                while (time.monotonic() - t_after_flush) < AUX_MULTI_FALL_TIMEOUT_S:
                    if GPIO.input(self.pin_aux) == GPIO.LOW:
                        aux_dropped = True
                        drop_ms = (time.monotonic() - t_after_flush) * 1000.0
                        break
                    time.sleep(0.0005)

                if aux_dropped:
                    t_low = time.monotonic()
                    if not self.wait_aux(timeout=AUX_RISE_WATCHDOG_S):
                        self.hardware_reset_e220(f"AUX stuck LOW >{AUX_RISE_WATCHDOG_S}s on 2-pkt {total_len}B")
                    aux_low_s = time.monotonic() - t_low
                    # Subpacket 2 (tail_len) just started RF transmission when AUX rose HIGH:
                    time.sleep(tail_air_s)
                else:
                    # Fallback if AUX edge was missed: sleep full calculated 2-subpacket air time
                    elapsed_since_flush = time.monotonic() - t_after_flush
                    rem_s = (head_est_s + tail_air_s) - elapsed_since_flush
                    if rem_s > 0.0:
                        time.sleep(rem_s)
            else:
                time.sleep(head_est_s + tail_air_s)

            time.sleep(self.guard_delay_s)
            if aux_dropped:
                detail = (
                    f"2 pkts ({self.packet_size}+{tail_len}B) | "
                    f"AUX_low={aux_low_s*1000:.1f}ms(drop={drop_ms:.1f}ms) + "
                    f"TailAir={tail_air_s*1000:.1f}ms + Guard={self.guard_delay_s*1000:.0f}ms"
                )
            else:
                detail = (
                    f"2 pkts ({self.packet_size}+{tail_len}B) | "
                    f"NO-AUX-DROP EstAir={(head_est_s+tail_air_s)*1000:.1f}ms + Guard={self.guard_delay_s*1000:.0f}ms"
                )

        total_dt = time.monotonic() - t0
        self.total_wait_time_s += total_dt
        self.total_bytes_out += total_len
        self.total_packets_sent += 1
        self.total_subpackets_sent += num_subpkts

        if self.verbose:
            print(f"[TX PKT] {total_len:3d}B | {detail} | Total={total_dt*1000:.1f}ms", flush=True)

        return (num_subpkts, detail, total_dt)

    def send_rtcm_frame(self, frame: bytes, msg_type: int) -> None:
        """Transmit a verified, CRC24Q-valid RTCM3 frame and record telemetry."""
        tot_len = len(frame)
        _, detail, total_dt = self.send_packet(frame)
        self.rtcm_stats[msg_type] = self.rtcm_stats.get(msg_type, 0) + 1

        if not self.quiet:
            print(
                f"[TX RTCM] Type:{msg_type:<4d} ({tot_len:3d}B) -> {detail} | Total={total_dt*1000:5.1f}ms",
                flush=True,
            )

    def _shed_stale_backlog(self, buffer: bytearray) -> None:
        """
        If TCP input ever accumulates >2048B of stale epochs (e.g. after a momentary stall),
        shed older MSM frames on clean RTCM3 boundaries while preserving 1005/1006/1230 ARP/biases.
        """
        preserved = bytearray()
        idx = 0
        shed = 0
        n = len(buffer)
        while idx + 6 <= n and (n - idx) > 1024:
            if buffer[idx] != 0xD3 or (buffer[idx + 1] & 0xFC) != 0:
                idx += 1
                continue
            plen = ((buffer[idx + 1] & 0x03) << 8) | buffer[idx + 2]
            flen = plen + 6
            if plen < 6 or plen > 1023 or (idx + flen) > n:
                break
            candidate = bytes(buffer[idx : idx + flen])
            if rtcm_crc24q(candidate) == 0:
                mtype = (candidate[3] << 4) | (candidate[4] >> 4)
                if mtype in (1005, 1006, 1230):
                    preserved.extend(candidate)
                else:
                    shed += 1
                idx += flen
            else:
                idx += 1

        if idx > 0:
            remaining = bytes(buffer[idx:])
            buffer.clear()
            buffer.extend(preserved)
            buffer.extend(remaining)
            if shed > 0:
                self.backlog_shed_frames += shed
                print(
                    f"[WARN] Shed {shed} stale queued MSM frames to keep RTCM differential age real-time (<1s).",
                    flush=True,
                )

    def _write_atomic_json(self, data: dict) -> None:
        """Writes live rover telemetry atomically to RAM tmpfs (/run or /tmp) with zero SD card wear."""
        for target_dir in ("/run", "/tmp"):
            path = os.path.join(target_dir, "rtk_rover_status.json")
            tmp_path = path + ".tmp"
            try:
                with open(tmp_path, "w", encoding="utf-8") as f:
                    json.dump(data, f, indent=2)
                os.replace(tmp_path, path)
                break
            except (OSError, PermissionError):
                continue

    def _write_csv_record(self, data: dict) -> None:
        """Appends telemetry to daily CSV with rollover, gzip compression of yesterday's log, and retention pruning."""
        if not self.enable_csv:
            return
        today_str = time.strftime("%Y-%m-%d")
        if today_str != self.current_log_date:
            if self.current_log_file:
                try:
                    self.current_log_file.close()
                except Exception:
                    pass
                if self.compress_logs and self.current_log_date:
                    self._compress_log(self.current_log_date)
            self._prune_old_logs()
            self.current_log_date = today_str
            os.makedirs(self.log_dir, exist_ok=True)
            try:
                os.chmod(self.log_dir, 0o777)
            except OSError:
                pass
            csv_path = os.path.join(self.log_dir, f"rover_{today_str}.csv")
            file_exists = os.path.exists(csv_path) and os.path.getsize(csv_path) > 0
            self.current_log_file = open(csv_path, "a", encoding="utf-8", buffering=1)
            if not file_exists:
                try:
                    os.chmod(csv_path, 0o666)
                except OSError:
                    pass
                headers = [
                    "timestamp_utc", "seq", "fix_type", "fix_name",
                    "latitude", "longitude", "height_m", "h_acc_mm",
                    "sats_master", "sats_slave", "heading_deg",
                    "battery_pct", "battery_mv",
                    "rover_rssi_dbm", "rover_noise_dbm",
                    "air_rate_kbps", "channel"
                ]
                self.current_log_file.write(",".join(headers) + "\n")

        row = [
            data["timestamp_utc"],
            str(data["seq"]),
            str(data["fix_type"]),
            data["fix_name"],
            f"{data['latitude']:.8f}",
            f"{data['longitude']:.8f}",
            f"{data['height_m']:.3f}",
            str(data["h_acc_mm"]),
            str(data["sats_master"]),
            str(data["sats_slave"]),
            f"{data['heading_deg']:.2f}",
            str(data["battery_pct"]),
            str(data["battery_mv"]),
            str(data["rover_rssi_dbm"]),
            str(data["rover_noise_dbm"]),
            f"{data['base_air_rate_kbps']:.1f}",
            str(data["channel"])
        ]
        if self.current_log_file:
            self.current_log_file.write(",".join(row) + "\n")

    def _prune_old_logs(self) -> None:
        """Enforces retention policy (deletes files older than N days) and max total directory size (FIFO cap)."""
        if not os.path.exists(self.log_dir):
            return
        now = time.time()
        cutoff_s = self.log_retention_days * 86400.0
        files = []
        for fname in os.listdir(self.log_dir):
            if fname.startswith("rover_") and (fname.endswith(".csv") or fname.endswith(".csv.gz")):
                fpath = os.path.join(self.log_dir, fname)
                try:
                    mtime = os.path.getmtime(fpath)
                    sz = os.path.getsize(fpath)
                    files.append((fpath, mtime, sz))
                    if (now - mtime) > cutoff_s:
                        os.remove(fpath)
                        print(f"[LOG RETENTION] Pruned expired log (> {self.log_retention_days}d): {fname}", flush=True)
                except OSError:
                    pass

        if self.max_log_mb > 0:
            total_sz = sum(sz for p, _, sz in files if os.path.exists(p))
            max_bytes = self.max_log_mb * 1024 * 1024
            if total_sz > max_bytes:
                files.sort(key=lambda x: x[1])
                for fpath, _, sz in files:
                    if total_sz <= max_bytes:
                        break
                    try:
                        if os.path.exists(fpath):
                            os.remove(fpath)
                            total_sz -= sz
                            print(f"[LOG CEILING] Pruned oldest log to keep total < {self.max_log_mb}MB: {os.path.basename(fpath)}", flush=True)
                    except OSError:
                        pass

    def _compress_log(self, date_str: str) -> None:
        """Compresses closed daily CSV using standard library gzip."""
        raw_path = os.path.join(self.log_dir, f"rover_{date_str}.csv")
        gz_path = raw_path + ".gz"
        if os.path.exists(raw_path) and not os.path.exists(gz_path):
            try:
                with open(raw_path, "rb") as f_in, gzip.open(gz_path, "wb") as f_out:
                    shutil.copyfileobj(f_in, f_out)
                os.remove(raw_path)
                print(f"[LOG GZIP] Compressed {os.path.basename(raw_path)} -> {os.path.basename(gz_path)}", flush=True)
            except Exception as e:
                print(f"[WARN] Failed to compress {raw_path}: {e}", flush=True)

    def reconfigure_air_rate(self, new_air_rate: float) -> bool:
        """Dynamically reconfigures Base E220 EEPROM to new air-rate in Mode 3 without restarting service."""
        if not self.gpio_available or serial is None:
            return False
        old_rate = self.air_rate_kbps
        self.air_rate_kbps = new_air_rate
        prev_auto = self.auto_config
        self.auto_config = True

        if self.ser and self.ser.is_open:
            self._safe_flush(self.ser)
            self.wait_aux(1.0)
            self.ser.close()

        ok = self.inspect_and_configure_eeprom()
        self.auto_config = prev_auto
        self._open_operational_uart()
        if ok:
            print(f"[RECONFIG] Base E220 air-rate switched {old_rate}k -> {new_air_rate}k successfully!", flush=True)
        else:
            print(f"[RECONFIG] Base E220 air-rate switch to {new_air_rate}k failed!", flush=True)
        return ok

    def _check_watchdog_revert(self) -> None:
        """Rolls back to 19.2k fallback if 60 seconds elapse after an air-rate switch without rover traffic."""
        if self.watchdog_rate_revert_time > 0:
            now = time.monotonic()
            if self.last_rover_rx_time > self.watchdog_rate_switch_time:
                self.watchdog_rate_revert_time = 0.0
                print("[LORA WATCHDOG] Verified uplink at new air-rate! Rollback watchdog disarmed.", flush=True)
            elif now >= self.watchdog_rate_revert_time:
                self.watchdog_rate_revert_time = 0.0
                print(f"[LORA WATCHDOG] 60s silence after rate switch! Reverting to {self.fallback_air_rate}k fallback...", flush=True)
                self.reconfigure_air_rate(self.fallback_air_rate)

    def _handle_telemetry_packet(self, cand: bytes) -> None:
        self.uplink_packets_received += 1
        (sync, mid, plen, lat, lon, height, h_acc_mm, fix_type, sats_m, sats_s,
         hdg_cdeg, bat_pct, bat_mv, r_rssi, r_noise, seq, crc) = struct.unpack(FMT_TELEMETRY, cand)

        fix_names = {0: "NONE", 1: "SINGLE", 2: "DGPS", 3: "PPS", 4: "RTK_FIX", 5: "RTK_FLOAT", 6: "DR"}
        fix_str = fix_names.get(fix_type, f"TYPE_{fix_type}")
        hdg_deg = hdg_cdeg / 100.0

        # High 2 bits of sats_s: bit 6 = heading_valid, bit 7 = heading_is_fixed
        sats_s_clean = sats_s & 0x3F
        hdg_valid = bool(sats_s & 0x40)
        hdg_fixed = bool(sats_s & 0x80)
        hdg_tag = "FIX" if hdg_fixed else ("VAL" if hdg_valid else "NONE")

        print(
            f"[ROVER TEL #{seq:03d}] Fix:{fix_str} (Sats:{sats_m}M/{sats_s_clean}S, +/-{h_acc_mm/10.0:.1f}cm) | "
            f"Hdg:{hdg_deg:5.1f} deg [{hdg_tag}] | Bat:{bat_pct}% ({bat_mv/1000.0:.2f}V) | "
            f"RSSI:{r_rssi}dBm (Noise:{r_noise}dBm) | Pos:{lat:.7f}, {lon:.7f}, {height:.2f}m",
            flush=True,
        )

        status_data = {
            "timestamp_utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "seq": seq,
            "fix_type": fix_type,
            "fix_name": fix_str,
            "latitude": round(lat, 8),
            "longitude": round(lon, 8),
            "height_m": round(height, 3),
            "h_acc_mm": h_acc_mm,
            "sats_master": sats_m,
            "sats_slave": sats_s_clean,
            "heading_deg": round(hdg_deg, 2),
            "heading_valid": hdg_valid,
            "heading_fixed": hdg_fixed,
            "battery_pct": bat_pct,
            "battery_mv": bat_mv,
            "rover_rssi_dbm": r_rssi,
            "rover_noise_dbm": r_noise,
            "base_air_rate_kbps": self.air_rate_kbps,
            "channel": self.channel,
        }
        self.last_rover_telemetry = status_data
        self._write_atomic_json(status_data)
        self._write_csv_record(status_data)

    def _handle_rate_switch_request(self, cand: bytes) -> None:
        (sync, mid, plen, target_code, target_chan, req_id, crc) = struct.unpack(FMT_REQ_SWITCH, cand)
        code_to_air = {0x02: 2.4, 0x03: 4.8, 0x04: 9.6, 0x05: 19.2, 0x06: 38.4, 0x07: 62.5}
        target_rate = code_to_air.get(target_code, 19.2)

        print(
            f"[LORA SWITCH] Received switch request from Rover: {self.air_rate_kbps}k -> {target_rate}k "
            f"(Req #{req_id}, CH{target_chan})",
            flush=True,
        )

        # Build ACK frame: sync(2), msg_id=0x82(1), plen=5(1), req_id(2), status=0(1), delay=400ms(2), crc(2) = 11B
        ack_body = struct.pack(FMT_ACK_SWITCH, ROVER_SYNC, MSG_ID_ACK_SWITCH, 5, req_id, 0, 400, 0)
        ack_crc = crc16_ccitt(ack_body[:-2])
        ack_frame = ack_body[:-2] + struct.pack("<H", ack_crc)

        # Transmit ACK back to Rover
        if self.ser and self.ser.is_open:
            self.ser.write(ack_frame)
            self._safe_flush(self.ser)
            self.wait_aux(0.2)
            print(f"[LORA SWITCH] Sent ACK to Rover (Req #{req_id}, delay 400ms)", flush=True)

        time.sleep(0.400)
        ok = self.reconfigure_air_rate(target_rate)
        if ok:
            self.rate_switch_count += 1
            self.watchdog_rate_switch_time = time.monotonic()
            self.watchdog_rate_revert_time = self.watchdog_rate_switch_time + 60.0
            print(f"[LORA SWITCH] Reconfigured Base E220 to {target_rate}k! Rollback watchdog armed (60s).", flush=True)
        else:
            print(f"[ERROR] Failed to reconfigure Base E220 to {target_rate}k!", flush=True)

    def poll_uplink(self) -> None:
        """Polls Base E220 serial port for 0xAA 0x55 uplink telemetry or rate switch commands from Rover."""
        if not self.ser or not self.ser.is_open:
            return

        try:
            avail = self.ser.in_waiting
        except Exception:
            return

        if avail > 0:
            try:
                raw = self.ser.read(avail)
                if raw:
                    self.uplink_buf.extend(raw)
            except Exception:
                return

        while len(self.uplink_buf) >= 2:
            if self.uplink_buf[0] != 0xAA or self.uplink_buf[1] != 0x55:
                idx = self.uplink_buf.find(ROVER_SYNC)
                if idx == -1:
                    self.uplink_buf.clear()
                    break
                del self.uplink_buf[:idx]
                if len(self.uplink_buf) < 2:
                    break

            if len(self.uplink_buf) < 4:
                break

            msg_id = self.uplink_buf[2]
            plen = self.uplink_buf[3]
            tot_len = plen + 6
            if len(self.uplink_buf) < tot_len:
                break

            cand = bytes(self.uplink_buf[:tot_len])
            calc_crc = crc16_ccitt(cand[:-2])
            pkt_crc = cand[-2] | (cand[-1] << 8)

            if calc_crc != pkt_crc:
                self.uplink_crc_errors += 1
                del self.uplink_buf[:1]
                continue

            del self.uplink_buf[:tot_len]
            self.last_rover_rx_time = time.monotonic()

            if msg_id == MSG_ID_TELEMETRY and tot_len == 39:
                self._handle_telemetry_packet(cand)
            elif msg_id == MSG_ID_REQ_SWITCH and tot_len == 10:
                self._handle_rate_switch_request(cand)

    def process_stream(self, read_fn) -> None:
        """Main loop: consumes RTKBase stream, verifies RTCM3 CRC24Q, and paces intact frames to E220."""
        buffer = bytearray()
        last_report = time.monotonic()
        last_rx_data_time = time.monotonic()

        while running:
            # Poll half-duplex quiet-window for Rover telemetry and commands
            self.poll_uplink()
            self._check_watchdog_revert()

            data = read_fn()
            now = time.monotonic()
            if data:
                buffer.extend(data)
                self.total_bytes_in += len(data)
                last_rx_data_time = now

            if len(buffer) > 2048:
                self._shed_stale_backlog(buffer)

            while len(buffer) >= 3:
                # 1. Sync strictly to RTCM3 preamble 0xD3 (never transmit non-RTCM orphan bytes over LoRa)
                if buffer[0] != 0xD3:
                    d3_idx = buffer.find(b"\xd3")
                    if d3_idx == -1:
                        self.src_orphan_bytes += len(buffer)
                        buffer.clear()
                        break
                    else:
                        self.src_orphan_bytes += d3_idx
                        del buffer[:d3_idx]
                        if len(buffer) < 3:
                            break

                # 2. Validate 6 zero reserved bits and 10-bit payload length [6..1023]
                if (buffer[1] & 0xFC) != 0x00:
                    self.src_orphan_bytes += 1
                    del buffer[:1]
                    continue

                payload_len = ((buffer[1] & 0x03) << 8) | buffer[2]
                if payload_len < 6 or payload_len > 1023:
                    self.src_orphan_bytes += 1
                    del buffer[:1]
                    continue

                total_frame_len = payload_len + 6

                # 3. Wait for complete frame; if incomplete for >350ms, step past false 0xD3 without TXing garbage
                if len(buffer) < total_frame_len:
                    if (now - last_rx_data_time) >= 0.350:
                        self.src_orphan_bytes += 1
                        del buffer[:1]
                        continue
                    break

                # 4. Verify 24-bit CRC24Q before spending LoRa airtime
                frame = bytes(buffer[:total_frame_len])
                msg_type = (frame[3] << 4) | (frame[4] >> 4)
                if rtcm_crc24q(frame) != 0:
                    self.src_crc_errors += 1
                    print(
                        f"[WARN] Dropped corrupt source RTCM candidate Type:{msg_type} ({total_frame_len}B, bad CRC24Q)",
                        flush=True,
                    )
                    del buffer[:1]
                    continue

                del buffer[:total_frame_len]
                self.send_rtcm_frame(frame, msg_type)

                # Poll uplink and watchdog immediately after each frame transmission
                self.poll_uplink()
                self._check_watchdog_revert()

                # End of MSM constellation epoch: enforce guaranteed 200ms half-duplex uplink quiet window
                if msg_type in (1094, 1124) or (msg_type == 1084 and self.rtcm_stats.get(1094, 0) == 0):
                    quiet_deadline = time.monotonic() + 0.200
                    while running and time.monotonic() < quiet_deadline:
                        self.poll_uplink()
                        self._check_watchdog_revert()
                        time.sleep(0.010)

                    # Shed backlog at lower air rates to keep differential age fresh
                    if len(buffer) > 600 and self.air_rate_kbps <= 9.6:
                        self._shed_stale_backlog(buffer)

                    break  # Yield to outer loop for heartbeat reporting and fresh socket reads

                # Poll socket opportunistically between frames so TCP buffer stays fresh
                more = read_fn()
                if more:
                    buffer.extend(more)
                    self.total_bytes_in += len(more)
                    last_rx_data_time = time.monotonic()

            now = time.monotonic()
            if (now - last_report) >= 5.0:
                dt = now - last_report
                rate_in = self.total_bytes_in / dt
                rate_out = self.total_bytes_out / dt
                duty_pct = min(100.0, (self.total_wait_time_s / dt) * 100.0)
                t_str = time.strftime("%H:%M:%S")
                frames_summary = ", ".join(f"{t}:{c}" for t, c in sorted(self.rtcm_stats.items())) or "None"
                uplink_str = f"Uplink:[{self.uplink_packets_received} pkts, {self.rate_switch_count} sw]"
                print(
                    f"[{t_str}] [PACER HEARTBEAT] In:{self.total_bytes_in}B ({rate_in:.1f}B/s) | "
                    f"Out:{self.total_bytes_out}B ({rate_out:.1f}B/s) | "
                    f"Duty:{duty_pct:.1f}% | RTCM:[{frames_summary}] | "
                    f"{uplink_str} | "
                    f"Frames:{self.total_packets_sent} (Subpkts:{self.total_subpackets_sent} "
                    f"Rst:{self.watchdog_resets} SrcErr:{self.src_crc_errors})",
                    flush=True,
                )
                self.total_bytes_in = 0
                self.total_bytes_out = 0
                self.total_wait_time_s = 0.0
                self.rtcm_stats.clear()
                last_report = now


def _drain_initial_socket_backlog(sock: socket.socket) -> int:
    """Drain any stale buffered bytes queued on TCP server port prior to pacer startup."""
    drained = 0
    sock.setblocking(False)
    try:
        while True:
            chunk = sock.recv(4096)
            if not chunk:
                break
            drained += len(chunk)
            if drained >= 65536:
                break
    except BlockingIOError:
        pass
    except Exception:
        pass
    return drained


def run_tcp_client_mode(pacer: E220Pacer, host: str, port: int) -> None:
    """Connects to RTKBase RTCM TCP Service."""
    pacer.setup_hardware()
    try:
        while running:
            print(f"[TCP] Connecting to RTKBase at {host}:{port}...", flush=True)
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            try:
                s.connect((host, port))
                drained = _drain_initial_socket_backlog(s)
                s.settimeout(0.015)
                print(
                    f"[TCP] Connected to RTKBase (drained {drained}B stale backlog). "
                    f"Streaming verified RTCM3 frames with Section 5.6.2 pacing...",
                    flush=True,
                )

                def read_socket() -> bytes:
                    try:
                        chunk = s.recv(4096)
                        if chunk == b"":
                            raise ConnectionResetError("RTKBase closed TCP socket")
                        return chunk
                    except socket.timeout:
                        return b""

                pacer.process_stream(read_socket)
            except _TERMIOS_ERRORS + (ConnectionRefusedError, ConnectionResetError, OSError) as e:
                if not running:
                    break
                print(f"[TCP] Connection interrupted: {e}. Retrying in 2.0s...", flush=True)
                time.sleep(2.0)
            finally:
                try:
                    s.close()
                except Exception:
                    pass
    finally:
        pacer.cleanup_hardware()


def run_tcp_server_mode(pacer: E220Pacer, listen_port: int) -> None:
    """Listens on a local port for RTKBase to stream to."""
    pacer.setup_hardware()
    server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_sock.bind(("0.0.0.0", listen_port))
    server_sock.listen(1)
    server_sock.settimeout(1.0)
    print(f"[TCP] Listening on port {listen_port} for RTKBase connection...", flush=True)

    try:
        while running:
            try:
                client_sock, addr = server_sock.accept()
                client_sock.settimeout(0.02)
                print(f"[TCP] RTKBase connected from {addr}! Streaming verified RTCM3 frames...", flush=True)

                def read_client() -> bytes:
                    try:
                        chunk = client_sock.recv(4096)
                        if chunk == b"":
                            raise ConnectionResetError("Client disconnected")
                        return chunk
                    except socket.timeout:
                        return b""

                try:
                    pacer.process_stream(read_client)
                except ConnectionResetError:
                    print("[TCP] RTKBase client disconnected.", flush=True)
                finally:
                    client_sock.close()
            except socket.timeout:
                continue
    finally:
        server_sock.close()
        pacer.cleanup_hardware()


def run_pty_mode(pacer: E220Pacer, pty_symlink: str) -> None:
    """Creates a virtual serial port (PTY) for RTKBase."""
    if pty is None or not hasattr(os, "openpty"):
        print("[ERROR] PTY mode is only supported on Linux/Unix.", flush=True)
        sys.exit(1)

    master, slave = os.openpty()
    slave_name = os.ttyname(slave)

    if os.path.islink(pty_symlink) or os.path.exists(pty_symlink):
        os.unlink(pty_symlink)
    os.symlink(slave_name, pty_symlink)
    print(f"[PTY] Created virtual serial device: {pty_symlink} -> {slave_name}", flush=True)

    pacer.setup_hardware()
    try:
        def read_pty() -> bytes:
            r, _, _ = select.select([master], [], [], 0.02)
            if r:
                return os.read(master, 4096)
            return b""

        pacer.process_stream(read_pty)
    finally:
        if os.path.islink(pty_symlink):
            os.unlink(pty_symlink)
        os.close(master)
        os.close(slave)
        pacer.cleanup_hardware()


def run_pipe_mode(pacer: E220Pacer) -> None:
    """Reads directly from STDIN."""
    pacer.setup_hardware()
    print("[PIPE] Reading RTCM from standard input...", flush=True)
    try:
        def read_stdin() -> bytes:
            r, _, _ = select.select([sys.stdin], [], [], 0.02)
            if r:
                return sys.stdin.buffer.read(4096)
            return b""

        pacer.process_stream(read_stdin)
    finally:
        pacer.cleanup_hardware()


def main() -> int:
    signal.signal(signal.SIGINT, sig_handler)
    signal.signal(signal.SIGTERM, sig_handler)

    parser = argparse.ArgumentParser(
        description=f"{__app_name__} v{__version__} - Hardware-Paced Ebyte E220 LoRa Driver for RTKBase",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument("--tcp-connect", metavar="HOST:PORT", help="Connect as client to RTKBase TCP Caster")
    group.add_argument("--tcp-listen", metavar="PORT", type=int, help="Listen as TCP server for RTKBase stream")
    group.add_argument("--pty", metavar="SYMLINK_PATH", help="Create virtual serial port for RTKBase")
    group.add_argument("--pipe", action="store_true", help="Read directly from STDIN")

    parser.add_argument("--port", default=DEFAULT_SERIAL_PORT, help="E220 hardware serial port")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="E220 hardware UART baud rate")
    parser.add_argument("--packet-size", type=int, default=DEFAULT_PACKET_SIZE, help="E220 EEPROM Subpacket Size")
    parser.add_argument("--guard-delay", type=float, default=DEFAULT_GUARD_DELAY_S, help="Inter-frame guard delay in seconds")

    parser.add_argument("--auto-config", action="store_true", help="Verify and program E220 EEPROM to target profile")
    parser.add_argument("--power", type=int, choices=[21, 24, 27, 30], default=21, help="Target TX power in dBm")
    parser.add_argument("--air-rate", type=float, choices=[2.4, 4.8, 9.6, 19.2, 38.4, 62.5], default=38.4, help="Target Air Data Rate in kbps")
    parser.add_argument("--channel", type=int, default=75, help="Target RF channel (75 = 925.125 MHz)")

    parser.add_argument("--pin-m0", type=int, default=DEFAULT_PIN_M0, help="BCM GPIO pin for E220 M0")
    parser.add_argument("--pin-m1", type=int, default=DEFAULT_PIN_M1, help="BCM GPIO pin for E220 M1")
    parser.add_argument("--pin-aux", type=int, default=DEFAULT_PIN_AUX, help="BCM GPIO pin for E220 AUX (Pacing line)")
    parser.add_argument("--pin-rst", type=int, default=DEFAULT_PIN_RST, help="BCM GPIO pin for E220 RESET")
    parser.add_argument("-v", "--verbose", action="store_true", help="Verbose pacing logs")
    parser.add_argument("-q", "--quiet", action="store_true", help="Suppress [TX RTCM] logs")
    parser.add_argument("--log-dir", default="~/rover_logs", help="Directory for daily rolling CSV rover logs")
    parser.add_argument("--log-retention-days", type=int, default=30, help="Days to retain daily CSV logs before auto-pruning")
    parser.add_argument("--max-log-mb", type=int, default=50, help="Maximum total megabytes for rover log directory")
    parser.add_argument("--no-compress", action="store_true", help="Disable automatic gzip compression of yesterday's CSV")
    parser.add_argument("--no-csv", action="store_true", help="Disable CSV disk logging (RAM JSON only)")

    args = parser.parse_args()
    freq_mhz = 850.125 + args.channel

    print(
        f"[INIT] {__app_name__} v{__version__} ({__date__}) starting | Port:{args.port} @ {args.baud} bps | "
        f"Air:{args.air_rate}k | CH:{args.channel} ({freq_mhz:.3f}MHz) | Pwr:{args.power}dBm | "
        f"Subpkt:{args.packet_size}B | Guard:{int(args.guard_delay * 1000)}ms",
        flush=True,
    )

    pacer = E220Pacer(
        port=args.port,
        baud=args.baud,
        packet_size=args.packet_size,
        guard_delay_s=args.guard_delay,
        pin_m0=args.pin_m0,
        pin_m1=args.pin_m1,
        pin_aux=args.pin_aux,
        pin_rst=args.pin_rst,
        auto_config=args.auto_config,
        air_rate_kbps=args.air_rate,
        power_dbm=args.power,
        channel=args.channel,
        verbose=args.verbose,
        quiet=args.quiet,
        log_dir=args.log_dir,
        log_retention_days=args.log_retention_days,
        max_log_mb=args.max_log_mb,
        compress_logs=not args.no_compress,
        no_csv=args.no_csv,
    )

    if args.tcp_connect:
        parts = args.tcp_connect.split(":")
        host = parts[0]
        port = int(parts[1]) if len(parts) > 1 else 5016
        run_tcp_client_mode(pacer, host, port)
    elif args.tcp_listen:
        run_tcp_server_mode(pacer, args.tcp_listen)
    elif args.pty:
        run_pty_mode(pacer, args.pty)
    elif args.pipe:
        run_pipe_mode(pacer)

    return 0


if __name__ == "__main__":
    sys.exit(main())
