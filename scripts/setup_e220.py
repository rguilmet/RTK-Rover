#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module: setup_e220.py
Description: Configuration and hardware control utility for Ebyte E220-900T series.
             Supports direct BCM GPIO control OR USB FTDI modem control lines (DTR/RTS/CTS).
Author: Roland L. Guilmet
License: MIT
"""

from __future__ import annotations

import argparse
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

__app_name__ = "setup_e220"
__version__ = "1.3.0"
__date__ = "2026-10-29"

BASE_FREQ_MHZ = 850.125

BAUD_RATES = {1200: 0b000, 2400: 0b001, 4800: 0b010, 9600: 0b011, 19200: 0b100, 38400: 0b101, 57600: 0b110, 115200: 0b111}
AIR_DATA_RATES = {2.4: 0b010, 4.8: 0b011, 9.6: 0b100, 19.2: 0b101, 38.4: 0b110, 62.5: 0b111}
SUBPACKET_SIZES = {200: 0b00, 128: 0b01, 64: 0b10, 32: 0b11}
POWER_LEVELS_30D = {30: 0b00, 27: 0b01, 24: 0b10, 21: 0b11}


class HardwareControllerBase:
    """Abstract base class for radio hardware manipulation."""
    def __init__(self, port: str, target_baud: int, timeout: float = 1.5):
        if serial is None:
            raise RuntimeError("'pyserial' package is required. Install with: pip install pyserial")
        self.port_name = port
        self.target_baud = target_baud
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None

    def setup(self) -> None: pass
    def cleanup(self) -> None: pass
    def enter_config_mode(self) -> None: pass
    def enter_normal_mode(self) -> None: pass
    def reset_module(self) -> None: pass

    def read_registers(self) -> Optional[bytes]:
        if not self.ser or not self.ser.is_open: return None
        for attempt in range(3):
            self.ser.reset_input_buffer()
            self.ser.write(bytes([0xC1, 0x00, 0x08]))
            self.ser.flush()
            time.sleep(0.08)
            res = self.ser.read(11)
            if len(res) == 11 and res[0] == 0xC1:
                return res[3:]
            time.sleep(0.1)
        return None

    def write_registers(self, reg_bytes: bytes, permanent: bool = True) -> bool:
        if not self.ser or not self.ser.is_open: return False
        header = 0xC0 if permanent else 0xC2
        for attempt in range(3):
            self.ser.reset_input_buffer()
            self.ser.write(bytes([header, 0x00, 0x08]) + reg_bytes)
            self.ser.flush()
            time.sleep(0.08)
            res = self.ser.read(11)
            if len(res) == 11 and res[0] in (0xC0, 0xC1, 0xC2):
                return True
            time.sleep(0.1)
        return False


class GpioHardwareController(HardwareControllerBase):
    """Controls the E220 via Raspberry Pi GPIO pins (Original Mode)."""
    def __init__(self, port: str, target_baud: int):
        super().__init__(port, target_baud)
        self.m0, self.m1, self.aux, self.rst = 18, 17, 27, 22

    def setup(self) -> None:
        if GPIO is None: raise RuntimeError("RPi.GPIO required for GPIO mode.")
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        for pin in (self.m0, self.m1, self.rst): GPIO.setup(pin, GPIO.OUT)
        GPIO.setup(self.aux, GPIO.IN)
        GPIO.output(self.rst, GPIO.HIGH)

    def wait_for_aux(self, timeout_sec: float = 3.0) -> bool:
        start = time.time()
        while time.time() - start < timeout_sec:
            if GPIO.input(self.aux) == GPIO.HIGH:
                time.sleep(0.02)
                return True
            time.sleep(0.01)
        return False

    def enter_config_mode(self) -> None:
        GPIO.output(self.m0, GPIO.HIGH)
        GPIO.output(self.m1, GPIO.HIGH)
        time.sleep(0.05)
        self.wait_for_aux()
        self.ser = serial.Serial(self.port_name, 9600, timeout=self.timeout)

    def enter_normal_mode(self) -> None:
        if self.ser and self.ser.is_open: self.ser.close()
        GPIO.output(self.m0, GPIO.LOW)
        GPIO.output(self.m1, GPIO.LOW)
        time.sleep(0.05)
        self.wait_for_aux()

    def reset_module(self) -> None:
        GPIO.output(self.rst, GPIO.LOW)
        time.sleep(0.1)
        GPIO.output(self.rst, GPIO.HIGH)
        self.wait_for_aux(4.0)


class FtdiHardwareController(HardwareControllerBase):
    """Controls the E220 via USB FTDI Modem Control Lines (DTR/RTS/CTS)."""
    def setup(self) -> None:
        # Open port initially at target baud rate
        self.ser = serial.Serial(self.port_name, self.target_baud, timeout=self.timeout)
        self.ser.dtr = True  # M0 = 0 (Normal)
        self.ser.rts = True  # M1 = 0 (Normal)
        time.sleep(0.05)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def cleanup(self) -> None:
        if self.ser and self.ser.is_open:
            self.ser.close()

    def wait_for_aux(self, timeout_sec: float = 3.0) -> bool:
        """
        Monitors E220 AUX line connected to FTDI CTS# pin.
        On standard FTDI (FT232R/H), CTS# is an active-low modem status line:
          - E220 AUX HIGH (3.3V, Idle/Ready) -> FTDI CTS# deasserted -> ser.cts is False
          - E220 AUX LOW  (0V, Busy)         -> FTDI CTS# asserted   -> ser.cts is True
        """
        start = time.time()
        time.sleep(0.05)  # Allow mode transition pin settlement
        while time.time() - start < timeout_sec:
            try:
                # E220 is ready when AUX is HIGH (ser.cts is False on standard FTDI)
                is_idle = not self.ser.cts
            except Exception:
                is_idle = True
            if is_idle:
                time.sleep(0.04)  # Line stabilization
                return True
            time.sleep(0.01)
        print("[*] Note: AUX line settlement timeout; proceeding with settling delay.")
        time.sleep(0.1)
        return False

    def enter_config_mode(self) -> None:
        # 1. Switch baud to 9600 first to avoid termios driver reset glitching DTR/RTS
        self.ser.baudrate = 9600
        time.sleep(0.05)
        # 2. Drive M0=1, M1=1 for Mode 3 (Sleep/Config): PySerial False = HIGH (3.3V)
        self.ser.dtr = False  # M0 = 1
        self.ser.rts = False  # M1 = 1
        time.sleep(0.1)
        self.wait_for_aux()
        time.sleep(0.05)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def enter_normal_mode(self) -> None:
        # 1. Drive M0=0, M1=0 for Mode 0 (Transparent): PySerial True = LOW (0V)
        self.ser.dtr = True   # M0 = 0
        self.ser.rts = True   # M1 = 0
        time.sleep(0.1)
        self.wait_for_aux()
        # 2. Restore operational baud rate
        self.ser.baudrate = self.target_baud
        time.sleep(0.05)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def reset_module(self) -> None:
        print("[-] Hardware reset (RST) is not mapped on the FTDI interface.")


def decode_and_print_config(raw_regs: bytes, port_name: str) -> None:
    """Decodes and formats the 8 configuration bytes for display."""
    if len(raw_regs) < 8: return
    addh, addl, reg0, reg1, chan, reg3, crypt_h, crypt_l = raw_regs[:8]

    baud_val = {v: k for k, v in BAUD_RATES.items()}.get((reg0 >> 5) & 0b111, "Unknown")
    air_val = {v: k for k, v in AIR_DATA_RATES.items()}.get(reg0 & 0b111, "Unknown")
    pkt_val = {v: k for k, v in SUBPACKET_SIZES.items()}.get((reg1 >> 6) & 0b11, "Unknown")
    power_bits = reg1 & 0b11
    power_dbm = {v: k for k, v in POWER_LEVELS_30D.items()}.get(power_bits, "Unknown")
    
    fixed_mode = "Fixed" if (reg3 & 0x40) else "Transparent"
    lbt_mode = "Enabled" if (reg3 & 0x10) else "Disabled"

    print("\n--- Current E220 Configuration ---")
    print(f"  Interface Port:    {port_name}")
    print(f"  Hardware Model:    E220-900T30S (Static)")
    print(f"  Raw Hex:           {raw_regs.hex().upper()}")
    print(f"  RF Channel:        {chan} ({BASE_FREQ_MHZ + chan:.3f} MHz)")
    print(f"  UART Baud Rate:    {baud_val} bps")
    print(f"  Air Data Rate:     {air_val} kbps")
    print(f"  Packet Size:       {pkt_val} bytes")
    print(f"  Power Setting:     Level {power_bits} ({power_dbm} dBm)")
    print(f"  Transmission Mode: {fixed_mode}")
    print(f"  LBT Function:      {lbt_mode}")
    print("----------------------------------\n")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=f"Configure E220 transceiver (v{__version__})")
    parser.add_argument("--interface", choices=['gpio', 'ftdi'], default='gpio', help="Hardware connection type (default: gpio)")
    parser.add_argument("--port", default="/dev/ttyAMA0", help="Serial port")
    parser.add_argument("--baud", type=int, default=115200, help="Operational Baud (default: 115200)")
    parser.add_argument("--query", action="store_true", help="Read registers only")
    parser.add_argument("--reset-only", action="store_true")
    parser.add_argument("--freq", type=float, default=925.125, help="RF frequency in MHz (default: 925.125 = CH 75)")
    parser.add_argument("--power", type=int, default=21, choices=[21, 24, 27, 30], help="TX power in dBm (default: 21)")
    parser.add_argument("--air-rate", type=float, default=38.4, choices=[2.4, 4.8, 9.6, 19.2, 38.4, 62.5], help="Air Data Rate in kbps (default: 38.4)")
    parser.add_argument("--packet-size", type=int, default=200, choices=[32, 64, 128, 200], help="Subpacket size in bytes (default: 200)")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    print(f"=== {__app_name__} v{__version__} ===")

    if args.interface == 'ftdi':
        controller = FtdiHardwareController(port=args.port, target_baud=args.baud)
    else:
        controller = GpioHardwareController(port=args.port, target_baud=args.baud)

    controller.setup()

    try:
        if args.reset_only:
            controller.reset_module()
            controller.enter_normal_mode()
            return 0

        print("[*] Switching E220 to Configuration mode...")
        controller.enter_config_mode()

        existing_regs = controller.read_registers()
        if not existing_regs:
            print("[-] Failed to read registers.")
            return 1

        decode_and_print_config(existing_regs, args.port)
        if args.query: return 0

        chan = int(round(args.freq - BASE_FREQ_MHZ))
        reg0 = (BAUD_RATES[args.baud] << 5) | (0b00 << 3) | AIR_DATA_RATES[args.air_rate]
        # REG1 Bit 5 (0x20) = Enable Mode 0 Ambient Noise & Last Packet RSSI Register (C0 C1 C2 C3)
        reg1 = (SUBPACKET_SIZES[args.packet_size] << 6) | 0x20 | POWER_LEVELS_30D.get(args.power, 0b11)
        
        # REG3 configuration: 0b00000000 strictly enforces Transparent Transmission Mode
        new_regs = bytes([existing_regs[0], existing_regs[1], reg0, reg1, chan, 0b00000000, existing_regs[6], existing_regs[7]])

        print("[*] Writing configuration...")
        if not controller.write_registers(new_regs):
            print("[-] Write failed.")
            return 1
        print("[+] Write successful.")

        updated = controller.read_registers()
        if updated: decode_and_print_config(updated, args.port)

    finally:
        print("[*] Returning E220 to Normal Mode...")
        controller.enter_normal_mode()
        controller.cleanup()
        print("[+] Ready for transmission.")
    return 0

if __name__ == "__main__":
    sys.exit(main())
