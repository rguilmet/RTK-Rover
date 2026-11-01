#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module: set_e220_power.py
Description: Fast runtime utility to switch E220 transmit power levels (21, 24, 27, 30 dBm)
             on an active RTKBase station. Automatically pauses e220-pacer service,
             programs the E220 EEPROM registers via GPIO M0/M1, and restarts the service.
Author: Roland L. Guilmet & Antigravity
License: MIT
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import time

try:
    import serial
except ImportError:
    serial = None

try:
    import RPi.GPIO as GPIO
except ImportError:
    GPIO = None

# Hardware Pins (matches setup_e220.py)
PIN_M0 = 18
PIN_M1 = 17
PIN_AUX = 27
PIN_RST = 22
PORT = "/dev/ttyAMA0"

POWER_LEVELS = {30: 0b00, 27: 0b01, 24: 0b10, 21: 0b11}


def wait_aux(timeout: float = 3.0) -> bool:
    if GPIO is None:
        return True
    start = time.time()
    while time.time() - start < timeout:
        if GPIO.input(PIN_AUX) == GPIO.HIGH:
            time.sleep(0.04)
            return True
        time.sleep(0.01)
    return False


def set_power(target_power_dbm: int) -> bool:
    if target_power_dbm not in POWER_LEVELS:
        print(f"[ERROR] Invalid power level {target_power_dbm} dBm. Valid choices: 21, 24, 27, 30")
        return False

    print(f"[*] Pausing e220-pacer service...")
    subprocess.run(["systemctl", "stop", "e220-pacer"], check=False)
    time.sleep(0.2)

    if GPIO is None or serial is None:
        print("[ERROR] RPi.GPIO and pyserial are required.")
        return False

    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    for p in (PIN_M0, PIN_M1, PIN_RST):
        GPIO.setup(p, GPIO.OUT)
    GPIO.setup(PIN_AUX, GPIO.IN)
    GPIO.output(PIN_RST, GPIO.HIGH)

    # Enter Configuration Mode (M0=1, M1=1)
    print("[*] Switching E220 to Configuration Mode (M0=1, M1=1)...")
    GPIO.output(PIN_M0, GPIO.HIGH)
    GPIO.output(PIN_M1, GPIO.HIGH)
    time.sleep(0.05)
    wait_aux()

    ser = serial.Serial(PORT, 9600, timeout=1.0)
    try:
        # Read current registers: C1 00 08
        ser.reset_input_buffer()
        ser.write(bytes([0xC1, 0x00, 0x08]))
        ser.flush()
        time.sleep(0.08)
        res = ser.read(11)
        if len(res) != 11 or res[0] != 0xC1:
            print("[ERROR] Failed to read current E220 configuration.")
            return False

        regs = bytearray(res[3:])  # 8 register bytes
        old_power_bits = regs[3] & 0b11
        old_power_dbm = {v: k for k, v in POWER_LEVELS.items()}.get(old_power_bits, "Unknown")

        # Update power bits (REG1 bits [1:0]) and enable Mode 0 RSSI Register (REG1 bit 5 = 0x20)
        new_power_bits = POWER_LEVELS[target_power_dbm]
        regs[3] = ((regs[3] & 0xFC) | 0x20) | new_power_bits
        # Enforce Transparent Mode (bit 6 = 0), LBT Disabled (bit 4 = 0), and no extra RX RSSI byte (bit 7 = 0)
        regs[5] = regs[5] & ~(0x80 | 0x40 | 0x10)

        # Write new registers to EEPROM: C0 00 08 ...
        print(f"[*] Updating power: {old_power_dbm} dBm -> {target_power_dbm} dBm (Level {new_power_bits})...")
        ser.reset_input_buffer()
        ser.write(bytes([0xC0, 0x00, 0x08]) + bytes(regs))
        ser.flush()
        time.sleep(0.10)
        res_write = ser.read(11)
        if len(res_write) != 11 or res_write[0] not in (0xC0, 0xC1):
            print("[ERROR] Failed to write new E220 configuration to EEPROM.")
            return False

        print(f"[+] Successfully programmed E220 EEPROM to {target_power_dbm} dBm!")
    finally:
        ser.close()

    # Return to Normal Mode (M0=0, M1=0)
    print("[*] Returning E220 to Normal Mode (M0=0, M1=0)...")
    GPIO.output(PIN_M0, GPIO.LOW)
    GPIO.output(PIN_M1, GPIO.LOW)
    time.sleep(0.05)
    wait_aux()
    GPIO.cleanup()

    # Restart e220-pacer service
    print("[*] Resuming e220-pacer service...")
    subprocess.run(["systemctl", "start", "e220-pacer"], check=False)
    time.sleep(0.5)

    # Check status
    res_status = subprocess.run(["systemctl", "is-active", "e220-pacer"], capture_output=True, text=True)
    status = res_status.stdout.strip()
    print(f"[+] Service status: {status.upper()}")
    print(f"[+] All done! E220 is actively transmitting at {target_power_dbm} dBm.")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Switch E220 TX power on the fly without breaking RTKBase.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("power", type=int, choices=[21, 24, 27, 30], help="Target TX power in dBm (21=125mW, 24=250mW, 27=500mW, 30=1000mW)")
    args = parser.parse_args()

    success = set_power(args.power)
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())
