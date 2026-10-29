#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module: setup_um980.py
Description: Configuration utility for the Unicore UM980 GNSS receiver.
             Configures serial ports, base station operating mode (autonomous
             survey-in or fixed coordinates), and full-constellation RTCM3
             output streams (MSM4 or MSM7). Handles warm reboots induced
             by signal group updates.
Author: Roland L. Guilmet
License: MIT
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import List, Optional, Tuple
import serial

__app_name__ = "setup_um980"
__version__ = "1.1.0"
__date__ = "2026-10-29"


class UM980Configurator:
    """Manages serial communication and command execution for the UM980."""

    def __init__(self, port: str, baud: int, timeout: float = 1.0) -> None:
        self.port_name = port
        self.baud = baud
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None

    def open(self) -> None:
        """Opens the serial connection."""
        try:
            self.ser = serial.Serial(
                port=self.port_name,
                baudrate=self.baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
            )
            time.sleep(0.2)
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            print(f"[+] Serial port {self.port_name} opened at {self.baud} baud.")
        except serial.SerialException as err:
            print(f"[-] Failed to open {self.port_name}: {err}", file=sys.stderr)
            sys.exit(1)

    def close(self) -> None:
        """Closes the serial connection."""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print(f"[+] Serial port {self.port_name} closed.")

    def send_command(self, cmd: str, delay: float = 0.2) -> str:
        """
        Sends an ASCII command terminated by CRLF and captures the immediate response.
        """
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("Serial port not open.")

        clean_cmd = cmd.strip()
        formatted_cmd = f"{clean_cmd}\r\n".encode("ascii")
        
        print(f"--> {clean_cmd}")
        self.ser.write(formatted_cmd)
        self.ser.flush()
        time.sleep(delay)

        response_lines = []
        while self.ser.in_waiting > 0:
            line = self.ser.readline().decode("ascii", errors="replace").strip()
            if line:
                response_lines.append(line)

        response = "\n".join(response_lines)
        if response:
            for r_line in response_lines:
                print(f"    {r_line}")

        # Check for command acceptance or error notices
        resp_lower = response.lower()
        if "error" in resp_lower or "invalid" in resp_lower or "syntax" in resp_lower:
            print(f"[-] WARNING: UM980 reported error for '{clean_cmd}'", file=sys.stderr)

        return response

    def wait_for_reboot(self, wait_seconds: float = 6.0, max_retries: int = 5) -> None:
        """
        Pauses for the internal firmware reboot after SIGNALGROUP changes,
        then polls until the command interface responds.
        """
        print(f"[*] Waiting {wait_seconds:.1f}s for UM980 DSP core to reboot...")
        time.sleep(wait_seconds)

        if not self.ser:
            return

        self.ser.reset_input_buffer()
        print("[*] Verifying serial communication after reboot...")
        
        for attempt in range(1, max_retries + 1):
            self.ser.write(b"\r\n")
            self.ser.flush()
            time.sleep(0.5)
            
            if self.ser.in_waiting > 0:
                self.ser.reset_input_buffer()
                print(f"[+] UM980 communication restored on attempt {attempt}.")
                return
            
            # Send lightweight query
            self.ser.write(b"VERSION\r\n")
            self.ser.flush()
            time.sleep(0.5)
            response = self.ser.read(self.ser.in_waiting).decode("ascii", errors="replace")
            if "VERSION" in response or "<OK" in response:
                print(f"[+] UM980 communication verified on attempt {attempt}.")
                return

        print("[-] Warning: No prompt received after reboot; continuing anyway.", file=sys.stderr)


def parse_args() -> argparse.Namespace:
    """Parses command line arguments."""
    parser = argparse.ArgumentParser(
        prog=__app_name__,
        description=f"{__app_name__} (v{__version__}): Configure Unicore UM980 base station parameters.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Configure survey-in base mode on /dev/ttyAMA3 with compact MSM4:
  ./setup_um980.py --port /dev/ttyAMA3 --msm 4

  # Configure fixed position base mode with MSM7:
  ./setup_um980.py --port /dev/ttyAMA3 --msm 7 --fixed-pos 43.7051234 -70.5123456 95.42

  # Configure with signal group 2 selection prior to parameters:
  ./setup_um980.py --port /dev/ttyAMA3 --signalgroup2
        """,
    )

    parser.add_argument(
        "--port",
        type=str,
        default="/dev/ttyAMA3",
        help="Serial port connected to UM980 COM1 (default: /dev/ttyAMA3)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate for UM980 COM1 communication (default: 115200)",
    )
    parser.add_argument(
        "--msm",
        type=int,
        choices=[4, 7],
        default=4,
        help="MSM observation message type: 4 (compact MSM4 for radio links) or 7 (full MSM7) (default: 4)",
    )
    parser.add_argument(
        "--ephemeris",
        action="store_true",
        help="Include broadcast satellite ephemeris messages (1019, 1020, 1042, 1044, 1045, 1046) on COM2",
    )
    parser.add_argument(
        "--signalgroup2",
        action="store_true",
        help="Precede configuration with CONFIG SIGNALGROUP 2 (triggers receiver reboot)",
    )
    parser.add_argument(
        "--fixed-pos",
        nargs=3,
        type=float,
        metavar=("LAT", "LON", "HEIGHT"),
        help="Override autonomous survey-in with fixed coordinates (e.g. 43.705 -70.505 92.1)",
    )
    parser.add_argument(
        "--survey-duration",
        type=int,
        default=3600,
        help="Observation time in seconds for survey-in mode (default: 3600)",
    )
    parser.add_argument(
        "--survey-accuracy",
        type=float,
        default=2.0,
        help="3D standard deviation cutoff in meters for survey-in mode (default: 2.0)",
    )
    parser.add_argument(
        "--no-save",
        action="store_true",
        help="Skip executing SAVECONFIG to preserve volatile settings only",
    )
    parser.add_argument(
        "-v", "--version",
        action="version",
        version=f"{__app_name__} {__version__} ({__date__})",
    )

    return parser.parse_args()


def main() -> int:
    """Main execution flow."""
    args = parse_args()
    print(f"=== {__app_name__} v{__version__} ===")

    configurator = UM980Configurator(port=args.port, baud=args.baud)
    configurator.open()

    try:
        # 1. Clear existing message streams
        print("\n[*] Clearing existing message streams...")
        configurator.send_command("UNLOG COM1")
        configurator.send_command("UNLOG COM2")

        # 2. Optional Signal Group configuration
        if args.signalgroup2:
            print("\n[*] Applying CONFIG SIGNALGROUP 2...")
            configurator.send_command("CONFIG SIGNALGROUP 2")
            configurator.wait_for_reboot()

        # 3. Port configurations
        print("\n[*] Configuring port baud rates...")
        configurator.send_command("CONFIG COM1 115200")
        configurator.send_command("CONFIG COM2 115200")

        # 4. Set Base Operating Mode
        print("\n[*] Configuring base mode...")
        if args.fixed_pos:
            lat, lon, height = args.fixed_pos
            mode_cmd = f"MODE BASE {lat:.9f} {lon:.9f} {height:.4f}"
            print(f"[*] Setting fixed position coordinates: Lat={lat}, Lon={lon}, Height={height}m")
        else:
            mode_cmd = f"MODE BASE TIME {args.survey_duration} {args.survey_accuracy:g}"
            print(f"[*] Setting autonomous survey-in: {args.survey_duration}s cutoff, {args.survey_accuracy}m limit")
        configurator.send_command(mode_cmd)

        # 5. Enable RTCM3 stream on COM2
        print(f"\n[*] Configuring RTCM3 message stream on COM2 (MSM{args.msm})...")
        rtcm_commands: List[str] = [
            "RTCM1006 COM2 5",    # Station ARP Position with height (every 5s)
            "RTCM1033 COM2 10",   # Receiver & Antenna descriptor (every 10s)
            "RTCM1230 COM2 10",   # GLONASS code-phase biases (every 10s)
        ]

        if args.msm == 4:
            rtcm_commands.extend([
                "RTCM1074 COM2 1",   # GPS MSM4 (1 Hz)
                "RTCM1084 COM2 1",   # GLONASS MSM4 (1 Hz)
                "RTCM1094 COM2 1",   # Galileo MSM4 (1 Hz)
                "RTCM1124 COM2 1",   # BeiDou MSM4 (1 Hz)
            ])
        else:
            rtcm_commands.extend([
                "RTCM1077 COM2 1",   # GPS MSM7 (1 Hz)
                "RTCM1087 COM2 1",   # GLONASS MSM7 (1 Hz)
                "RTCM1097 COM2 1",   # Galileo MSM7 (1 Hz)
                "RTCM1107 COM2 1",   # SBAS MSM7 (1 Hz)
                "RTCM1117 COM2 1",   # QZSS MSM7 (1 Hz)
                "RTCM1127 COM2 1",   # BeiDou MSM7 (1 Hz)
            ])

        if args.ephemeris:
            print("[*] Including broadcast satellite ephemeris messages...")
            rtcm_commands.extend([
                "RTCM1019 COM2 10",  # GPS Ephemeris
                "RTCM1020 COM2 10",  # GLONASS Ephemeris
                "RTCM1042 COM2 10",  # BeiDou Ephemeris
                "RTCM1044 COM2 10",  # QZSS Ephemeris
                "RTCM1045 COM2 10",  # Galileo F/NAV
                "RTCM1046 COM2 10",  # Galileo I/NAV
            ])

        for rtcm_cmd in rtcm_commands:
            configurator.send_command(rtcm_cmd, delay=0.1)

        # 6. Query receiver state for verification
        print("\n[*] Verifying receiver configuration...")
        configurator.send_command("CONFIG", delay=0.4)
        configurator.send_command("MODE", delay=0.4)

        # 7. Persist to NVRAM
        if not args.no_save:
            print("\n[*] Persisting settings to NVRAM...")
            configurator.send_command("SAVECONFIG", delay=0.5)
        else:
            print("\n[*] Skipped SAVECONFIG (--no-save flag set).")

        print("\n[+] UM980 Base configuration complete.")
        return 0

    except KeyboardInterrupt:
        print("\n[!] Execution interrupted by user.", file=sys.stderr)
        return 130
    finally:
        configurator.close()


if __name__ == "__main__":
    sys.exit(main())
