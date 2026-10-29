#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module: setup_um980.py (and setup_unicore.py)
Description: Comprehensive configuration utility for Unicore GNSS receivers
             (UM980 single-antenna and UM982 dual-antenna).
             Configures UART baud rates, operating modes (Base Station with
             MSM4/MSM7 RTCM3 streams or Dual-Antenna Rover with precision heading),
             and handles signal group allocation and post-reboot synchronization.

Supported Hardware:
  - Unicore UM980 (Single-antenna high precision RTK Base or Rover)
  - Unicore UM982 (Dual-antenna high precision RTK Base or Heading Rover)

Signal Group Guide:
  - UM980:
      Group 1: GPS L1/L2/L5, BDS B1I/B2I/B3I/B1C/B2a/B2b, GLO G1/G2, GAL E1/E5a/E5b, QZSS (Default)
      Group 2: All constellations + frequencies (adds GPS L1C, GLO G3, GAL E6, NavIC L5)
      Group 8: High update rate (up to 50 Hz RTCM/raw data)
  - UM982:
      Group 4 5: Master(4) + Slave(5) - Full multi-frequency RTK + Heading (Factory Default)
      Group 3 6: Master(3) + Slave(6) - Adds BDS B2b-PPP, GAL E6 HAS, QZSS L6 MADOCA, 20 Hz
      Group 7 0: Master(7) + Slave(0) - Dedicated Base Station mode (Slave antenna disabled)
      Group 5 0: Master(5) + Slave(0) - Low-power Base Station mode

RTCM Output Guide (Base Mode):
  - RTCM 1006: Station ARP coordinates + antenna height (5s interval).
               Note: RTCM 1005 is not needed when RTCM 1006 is transmitted.
  - RTCM 1033: Receiver and antenna descriptor (10s interval).
  - RTCM 1230: GLONASS L1/L2 code-phase biases (10s interval). Essential for rovers to
               resolve GLONASS carrier ambiguities over FDMA channels.
  - RTCM 1074/1084/1094/1124: Compact MSM4 observation messages for radio links (1 Hz).
  - RTCM 1077/1087/1097/1107/1117/1127: Full MSM7 observation messages (1 Hz).

Author: Roland L. Guilmet
License: MIT
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import List, Optional
try:
    import serial
except ImportError:
    serial = None  # type: ignore

__app_name__ = "setup_um980"
__version__ = "1.2.0"
__date__ = "2026-10-29"


class UnicoreConfigurator:
    """Manages serial communication and command execution for UM980 / UM982 receivers."""

    def __init__(self, port: str, baud: int, timeout: float = 1.0) -> None:
        self.port_name = port
        self.baud = baud
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None

    def open(self) -> None:
        """Opens the serial connection."""
        if serial is None:
            print("[-] Error: 'pyserial' package is required. Install with: pip install pyserial", file=sys.stderr)
            sys.exit(1)

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
            print(f"[-] WARNING: Receiver reported error for '{clean_cmd}'", file=sys.stderr)

        return response

    def wait_for_reboot(self, wait_seconds: float = 6.0, max_retries: int = 5) -> None:
        """
        Pauses for internal firmware reboot after SIGNALGROUP updates,
        then polls until the command interface responds.
        """
        print(f"[*] Waiting {wait_seconds:.1f}s for receiver DSP core to reboot...")
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
                print(f"[+] Communication restored on attempt {attempt}.")
                return

            self.ser.write(b"VERSION\r\n")
            self.ser.flush()
            time.sleep(0.5)
            response = self.ser.read(self.ser.in_waiting).decode("ascii", errors="replace")
            if "VERSION" in response or "<OK" in response:
                print(f"[+] Communication verified on attempt {attempt}.")
                return

        print("[-] Warning: No prompt received after reboot; continuing anyway.", file=sys.stderr)


def parse_args() -> argparse.Namespace:
    """Parses command line arguments."""
    parser = argparse.ArgumentParser(
        prog=__app_name__,
        description=f"{__app_name__} (v{__version__}): Configure Unicore UM980 / UM982 GNSS receivers.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Configure UM980 as autonomous survey-in Base Station with MSM4 on COM2:
  ./setup_um980.py --device um980 --mode base --msm 4

  # Configure UM980 Base with Signal Group 2 and fixed coordinates:
  ./setup_um980.py --device um980 --mode base --signalgroup "2" --fixed-pos 43.7051234 -70.5123456 95.42

  # Configure UM982 as dedicated Base Station (Signal Group 7 0 disables slave antenna):
  ./setup_um980.py --device um982 --mode base --signalgroup "7 0" --msm 4

  # Configure UM982 as Dual-Antenna Rover with 55.5 cm baseline and Signal Group 3 6:
  ./setup_um980.py --device um982 --mode rover --signalgroup "3 6" --antenna-length 55.5 --antenna-tol 12.0
        """,
    )

    # Hardware & Mode Selection
    parser.add_argument(
        "--device",
        type=str.lower,
        choices=["um980", "um982"],
        default="um980",
        help="Target Unicore GNSS hardware module (default: um980)",
    )
    parser.add_argument(
        "--mode",
        type=str.lower,
        choices=["base", "rover"],
        default="base",
        help="Operating mode: 'base' (RTCM output on COM2) or 'rover' (NMEA/heading on COM1) (default: base)",
    )

    # Serial Port Options
    parser.add_argument(
        "--port",
        type=str,
        default="/dev/ttyAMA3",
        help="Serial port connected to receiver COM1 (default: /dev/ttyAMA3)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=115200,
        help="Baud rate for COM1 communication (default: 115200)",
    )
    parser.add_argument(
        "--com2-baud",
        type=int,
        default=115200,
        help="Baud rate to configure on receiver COM2 (default: 115200)",
    )

    # Signal Group Configuration
    parser.add_argument(
        "--signalgroup",
        type=str,
        default=None,
        metavar="'GROUP...'",
        help="Configure tracking signal group (e.g. '2' for UM980, '3 6' or '4 5' for UM982 Rover, '7 0' for UM982 Base)",
    )
    parser.add_argument(
        "--signalgroup2",
        action="store_true",
        help="Convenience alias: apply 'CONFIG SIGNALGROUP 2' (UM980 all-constellation mode)",
    )

    # Base Station Specific Options
    parser.add_argument(
        "--msm",
        type=int,
        choices=[4, 7],
        default=4,
        help="MSM message type in base mode: 4 (compact MSM4 for radio links) or 7 (full MSM7) (default: 4)",
    )
    parser.add_argument(
        "--ephemeris",
        action="store_true",
        help="Base mode: include broadcast satellite ephemeris messages (1019, 1020, 1042, etc.) on COM2",
    )
    parser.add_argument(
        "--fixed-pos",
        nargs=3,
        type=float,
        metavar=("LAT", "LON", "HEIGHT"),
        help="Base mode: override autonomous survey-in with fixed coordinates (e.g. 43.705 -70.505 92.1)",
    )
    parser.add_argument(
        "--survey-duration",
        type=int,
        default=3600,
        help="Base mode: observation time in seconds for autonomous survey-in (default: 3600)",
    )
    parser.add_argument(
        "--survey-accuracy",
        type=float,
        default=2.0,
        help="Base mode: 3D standard deviation cutoff in meters for survey-in (default: 2.0)",
    )

    # Rover Specific Options
    parser.add_argument(
        "--antenna-length",
        type=float,
        default=55.5,
        metavar="CM",
        help="Rover mode (UM982): fixed baseline distance between master and slave antennas in cm (default: 55.5)",
    )
    parser.add_argument(
        "--antenna-tol",
        type=float,
        default=12.0,
        metavar="CM",
        help="Rover mode (UM982): length tolerance threshold for fixed baseline in cm (default: 12.0)",
    )
    parser.add_argument(
        "--dynamic",
        type=str.lower,
        choices=["low", "normal", "high"],
        default="low",
        help="Rover mode (UM982): heading dynamics mode: low (surveying/walking), normal, or high (default: low)",
    )
    parser.add_argument(
        "--rover-rate",
        type=float,
        default=2.0,
        help="Rover mode: output rate in Hz for primary NMEA sentences (default: 2.0 Hz -> 0.5s period)",
    )

    # General Options
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


def configure_signalgroup(configurator: UnicoreConfigurator, device: str, mode: str, requested_sg: Optional[str]) -> None:
    """Validates and executes the CONFIG SIGNALGROUP command."""
    if not requested_sg:
        return

    tokens = requested_sg.strip().split()

    if device == "um980":
        if len(tokens) > 1:
            print(f"[-] Notice: UM980 only supports single-parameter signal groups. Using '{tokens[0]}'.")
        sg_cmd = f"CONFIG SIGNALGROUP {tokens[0]}"
    else:  # um982
        if len(tokens) == 1:
            if tokens[0] == "2":
                print("[-] Warning: Signal Group 2 is not standard on UM982.")
                print("    For UM982 Rover, standard dual-antenna groups are '3 6' or '4 5'.")
                print("    For UM982 Base, standard single-antenna group is '7 0'.")
            if mode == "base":
                print(f"[*] UM982 Base mode: appending slave disable '0' to group '{tokens[0]}'.")
                sg_cmd = f"CONFIG SIGNALGROUP {tokens[0]} 0"
            else:
                # Rover mode warning: single parameter defaults slave to 0!
                default_slave = "6" if tokens[0] == "3" else "5"
                print(f"[-] Warning: UM982 Rover requires both Master and Slave signal groups.")
                print(f"    Omitting parameter 2 disables the slave antenna and disables heading.")
                print(f"    Auto-completing to: CONFIG SIGNALGROUP {tokens[0]} {default_slave}")
                sg_cmd = f"CONFIG SIGNALGROUP {tokens[0]} {default_slave}"
        else:
            sg_cmd = f"CONFIG SIGNALGROUP {tokens[0]} {tokens[1]}"

    print(f"\n[*] Applying {sg_cmd}...")
    configurator.send_command(sg_cmd)
    configurator.wait_for_reboot()


def configure_base(configurator: UnicoreConfigurator, args: argparse.Namespace) -> None:
    """Configures the receiver for Base Station operation."""
    print("\n[*] Configuring Base Station operating parameters...")

    # Set Base Operating Mode
    if args.fixed_pos:
        lat, lon, height = args.fixed_pos
        mode_cmd = f"MODE BASE {lat:.9f} {lon:.9f} {height:.4f}"
        print(f"[*] Setting fixed position coordinates: Lat={lat}, Lon={lon}, Height={height}m")
    else:
        mode_cmd = f"MODE BASE TIME {args.survey_duration} {args.survey_accuracy:g}"
        print(f"[*] Setting autonomous survey-in: {args.survey_duration}s cutoff, {args.survey_accuracy}m limit")
    configurator.send_command(mode_cmd)

    # Configure RTCM3 output stream on COM2
    print(f"\n[*] Configuring RTCM3 message stream on COM2 (MSM{args.msm})...")
    # Base station messages:
    # 1006: Station ARP coordinates + height (every 5s). Supersedes 1005 (1005 omitted to prevent duplication).
    # 1033: Receiver and antenna descriptor (every 10s).
    # 1230: GLONASS L1/L2 code-phase biases (every 10s). Crucial for rover GLONASS RTK ambiguity resolution.
    rtcm_commands: List[str] = [
        "RTCM1006 COM2 5",
        "RTCM1033 COM2 10",
        "RTCM1230 COM2 10",
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


def configure_rover(configurator: UnicoreConfigurator, args: argparse.Namespace) -> None:
    """Configures the receiver for Rover operation."""
    print(f"\n[*] Configuring {args.device.upper()} Rover operating parameters...")

    # Multi-system PVT algorithm
    configurator.send_command("CONFIG PVTALG MULTI")

    # Survey mode
    configurator.send_command("MODE ROVER SURVEY")

    # Dual antenna heading configuration (for UM982)
    if args.device == "um982":
        dynamic_cmd = {
            "low": "CONFIG HEADING LOWDYNAMIC",
            "normal": "CONFIG HEADING NORMAL",
            "high": "CONFIG HEADING HIGHDYNAMIC",
        }.get(args.dynamic, "CONFIG HEADING LOWDYNAMIC")
        configurator.send_command(dynamic_cmd)

        configurator.send_command("CONFIG HEADING FIXLENGTH")
        configurator.send_command(f"CONFIG HEADING LENGTH {args.antenna_length:.1f} {args.antenna_tol:.1f}")

    # Output rates: period in seconds = 1.0 / rate_hz
    period_sec = max(0.05, 1.0 / max(0.1, args.rover_rate))
    period_str = f"{period_sec:g}"

    print(f"\n[*] Enabling Rover telemetry on COM1 (update rate: {args.rover_rate:g} Hz / {period_str}s period)...")
    configurator.send_command(f"GPGGA COM1 {period_str}", delay=0.05)
    configurator.send_command("GPRMC COM1 1", delay=0.05)
    configurator.send_command(f"GPGST COM1 {period_str}", delay=0.05)
    configurator.send_command("GPGSV COM1 1", delay=0.05)

    if args.device == "um982":
        # Dual-antenna heading and secondary antenna status
        configurator.send_command(f"UNIHEADINGA COM1 {period_str}", delay=0.05)
        configurator.send_command("GNGGAH COM1 1", delay=0.05)

    # Base station tracking info (reports incoming RTCM baseline / ARP)
    configurator.send_command("BASEINFOA COM1 1", delay=0.05)


def main() -> int:
    """Main execution flow."""
    args = parse_args()
    print(f"=== {__app_name__} v{__version__} ({args.device.upper()} - {args.mode.upper()} mode) ===")

    # Handle backwards-compatible --signalgroup2
    target_sg = args.signalgroup
    if args.signalgroup2 and not target_sg:
        target_sg = "2"

    configurator = UnicoreConfigurator(port=args.port, baud=args.baud)
    configurator.open()

    try:
        # 1. Clear existing message streams
        print("\n[*] Clearing existing message streams...")
        configurator.send_command("UNLOG COM1")
        configurator.send_command("UNLOG COM2")

        # 2. Configure Signal Group if requested
        if target_sg:
            configure_signalgroup(configurator, args.device, args.mode, target_sg)

        # 3. Port baud rate configurations
        print(f"\n[*] Configuring port baud rates (COM1={args.baud}, COM2={args.com2_baud})...")
        configurator.send_command(f"CONFIG COM1 {args.baud}")
        configurator.send_command(f"CONFIG COM2 {args.com2_baud}")

        # 4. Configure selected mode
        if args.mode == "base":
            configure_base(configurator, args)
        else:
            configure_rover(configurator, args)

        # 5. Query receiver state for verification
        print("\n[*] Verifying receiver configuration...")
        configurator.send_command("CONFIG", delay=0.4)
        configurator.send_command("MODE", delay=0.4)

        # 6. Persist to NVRAM
        if not args.no_save:
            print("\n[*] Persisting settings to NVRAM...")
            configurator.send_command("SAVECONFIG", delay=0.5)
        else:
            print("\n[*] Skipped SAVECONFIG (--no-save flag set).")

        print(f"\n[+] {args.device.upper()} {args.mode.upper()} configuration successfully applied.")
        return 0

    except KeyboardInterrupt:
        print("\n[!] Execution interrupted by user.", file=sys.stderr)
        return 130
    finally:
        configurator.close()


if __name__ == "__main__":
    sys.exit(main())
