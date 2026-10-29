#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module: setup_um98x.py
Description: Unified configuration and diagnostic utility for Unicore GNSS
             receivers (UM980 single-antenna and UM982 dual-antenna).

Features:
  - Smart Hardware Profiles:
      --device um980: Defaults to /dev/ttyAMA0 (RPi GPIO UART), Base mode, Signal Group 2.
      --device um982: Defaults to /dev/ttyUSB0 (FTDI USB UART), Rover survey mode,
                      Signal Group 3 6, and dual-antenna heading (55.5 cm baseline).
  - Factory Reset (--freset):
      Clears NVM settings and waits for full receiver reboot before configuring.
  - Signal Group Reconfiguration:
      Handles firmware reboots caused by CONFIG SIGNALGROUP and synchronizes UART.
  - Query Diagnostics (--query):
      Dumps VERSIONA, CONFIG, MODE, and UNILOGLIST / LOGLIST.
  - Operating Modes:
      Base Station: Autonomous survey-in or fixed coordinates, MSM4/MSM7 RTCM3 streams.
      Rover: Surveying dynamics, fixed baseline heading, and high-rate NMEA streams.

Author: Roland L. Guilmet
License: MIT
"""

from __future__ import annotations

import argparse
import sys
import time
from typing import Dict, List, Optional

try:
    import serial
except ImportError:
    serial = None  # type: ignore

__app_name__ = "setup_um98x"
__version__ = "1.3.0"
__date__ = "2026-10-29"

# Hardware Profile Presets
DEVICE_PROFILES: Dict[str, Dict[str, str]] = {
    "um980": {
        "port": "/dev/ttyAMA0",
        "mode": "base",
        "signalgroup": "2",
        "desc": "UM980 Single-Antenna (RPi GPIO UART AMA0, Base Mode, SG 2)",
    },
    "um982": {
        "port": "/dev/ttyUSB0",
        "mode": "rover",
        "signalgroup": "3 6",
        "desc": "UM982 Dual-Antenna (USB FTDI ttyUSB0, Rover Survey Mode, SG 3 6, Heading)",
    },
}


class UnicoreConfigurator:
    """Manages serial communication, commands, and reboot synchronization."""

    def __init__(self, port: str, baud: int, timeout: float = 1.0) -> None:
        self.port_name = port
        self.baud = baud
        self.timeout = timeout
        self.ser: Optional[serial.Serial] = None

    def open(self) -> None:
        """Opens and flushes the serial connection."""
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

    def send_command(
        self,
        cmd: str,
        delay: float = 0.2,
        wait_more: bool = False,
        print_cmd: bool = True,
        print_response: bool = True,
    ) -> List[str]:
        """
        Sends an ASCII command terminated by CRLF and captures all response lines.
        Uses adaptive multi-line draining for commands like CONFIG, VERSIONA, or UNILOGLIST.
        """
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("Serial port not open.")

        clean_cmd = cmd.strip()
        formatted_cmd = f"{clean_cmd}\r\n".encode("ascii")

        if print_cmd:
            print(f"--> {clean_cmd}")

        self.ser.write(formatted_cmd)
        self.ser.flush()
        time.sleep(delay)

        response_lines: List[str] = []
        timeout_sec = 3.0 if wait_more else 1.2
        deadline = time.time() + timeout_sec
        last_rx = time.time()

        while time.time() < deadline:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode("ascii", errors="replace").strip()
                if line:
                    response_lines.append(line)
                    if print_response:
                        print(f"    {line}")
                last_rx = time.time()
            else:
                # Stop if data arrived and port was quiet for 250ms
                if response_lines and (time.time() - last_rx > 0.25):
                    break
                time.sleep(0.04)

        # Warn if receiver reported syntax errors
        resp_lower = " ".join(response_lines).lower()
        if "error" in resp_lower or "invalid" in resp_lower or "syntax" in resp_lower:
            print(f"[-] WARNING: Receiver reported error for '{clean_cmd}'", file=sys.stderr)

        return response_lines

    def wait_for_reboot(self, wait_seconds: float = 6.0, max_retries: int = 8) -> bool:
        """
        Pauses while receiver hardware reboots (triggered by FRESET or CONFIG SIGNALGROUP),
        then actively probes until the command processor responds.
        """
        print(f"[*] Waiting {wait_seconds:.1f}s for receiver DSP core to reboot...")
        time.sleep(wait_seconds)

        if not self.ser:
            return False

        self.ser.reset_input_buffer()
        print("[*] Probing serial interface to verify boot completion...")

        for attempt in range(1, max_retries + 1):
            self.ser.write(b"\r\n")
            self.ser.flush()
            time.sleep(0.3)

            # Drain any greeting banner
            if self.ser.in_waiting > 0:
                self.ser.reset_input_buffer()
                print(f"[+] Receiver reboot confirmed on attempt {attempt}.")
                time.sleep(0.2)
                return True

            # Send lightweight VERSION query
            self.ser.write(b"VERSION\r\n")
            self.ser.flush()
            time.sleep(0.5)

            if self.ser.in_waiting > 0:
                rx_bytes = self.ser.read(self.ser.in_waiting).decode("ascii", errors="replace")
                if "VERSION" in rx_bytes or "<OK" in rx_bytes or "#" in rx_bytes:
                    print(f"[+] Receiver online and responsive on attempt {attempt}.")
                    time.sleep(0.2)
                    return True

        print("[-] Warning: No confirmation prompt received after reboot; continuing anyway.", file=sys.stderr)
        return False

    def query_all(self) -> None:
        """Dumps VERSIONA, CONFIG, MODE, and UNILOGLIST / LOGLIST."""
        print("\n==================== [RECEIVER DIAGNOSTIC QUERY] ====================")
        
        print("\n--- 1. VERSIONA ---")
        self.send_command("VERSIONA", delay=0.1, wait_more=True)

        print("\n--- 2. MODE ---")
        self.send_command("MODE", delay=0.1, wait_more=False)

        print("\n--- 3. CONFIG ---")
        self.send_command("CONFIG", delay=0.2, wait_more=True)

        print("\n--- 4. LOG LIST (UNILOGLIST) ---")
        lines = self.send_command("UNILOGLIST", delay=0.1, wait_more=True)
        if not lines or any("error" in l.lower() for l in lines):
            # Fallback to LOGLIST
            self.send_command("LOGLIST", delay=0.1, wait_more=True)

        print("\n=====================================================================")


def parse_args() -> argparse.Namespace:
    """Parses command line arguments with smart device-based defaults."""
    parser = argparse.ArgumentParser(
        prog=__app_name__,
        description=f"{__app_name__} (v{__version__}): Configure and diagnose Unicore UM980 / UM982 GNSS receivers.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Profiles & Defaults:
  --device um980:
      Port: /dev/ttyAMA0 (Raspberry Pi GPIO UART)
      Mode: base (Survey-in autonomous or fixed coordinates)
      Signal Group: 2 (All constellations + frequencies)

  --device um982:
      Port: /dev/ttyUSB0 (USB FTDI serial adapter)
      Mode: rover (Survey dynamic, fixed baseline 55.5cm heading)
      Signal Group: 3 6 (Master: Group 3 / Slave: Group 6)

Examples:
  # Configure UM980 Base with default profile (port /dev/ttyAMA0, Base mode, Signal Group 2, MSM4):
  ./setup_um98x.py --device um980

  # Factory reset UM980 and apply fresh Base configuration:
  ./setup_um98x.py --device um980 --freset

  # Configure UM982 Rover with default profile (port /dev/ttyUSB0, Rover mode, Signal Group 3 6):
  ./setup_um98x.py --device um982

  # Perform full receiver query dump (VERSIONA, CONFIG, MODE, LOGLIST):
  ./setup_um98x.py --device um982 --query

  # Configure UM982 as a Base Station (Signal Group 7 0 disables slave antenna):
  ./setup_um98x.py --device um982 --mode base --signalgroup "7 0" --port /dev/ttyAMA0
        """,
    )

    # Hardware Profile
    parser.add_argument(
        "--device",
        type=str.lower,
        choices=["um980", "um982"],
        default="um980",
        help="Target Unicore GNSS hardware module (default: um980)",
    )

    # Actions
    parser.add_argument(
        "--freset",
        action="store_true",
        help="Execute factory reset (FRESET) and wait for reboot before applying configuration",
    )
    parser.add_argument(
        "--query",
        action="store_true",
        help="Query and display receiver status (VERSIONA, CONFIG, MODE, UNILOGLIST) then exit",
    )

    # Operating Mode
    parser.add_argument(
        "--mode",
        type=str.lower,
        choices=["base", "rover"],
        default=None,
        help="Operating mode: 'base' or 'rover' (defaults to 'base' for um980, 'rover' for um982)",
    )

    # Serial Port Options
    parser.add_argument(
        "--port",
        type=str,
        default=None,
        help="Serial port (defaults to /dev/ttyAMA0 for um980, /dev/ttyUSB0 for um982)",
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

    # Signal Group Options
    parser.add_argument(
        "--signalgroup",
        type=str,
        default=None,
        metavar="'GROUP...'",
        help="Configure tracking signal group (defaults to '2' for um980, '3 6' for um982)",
    )
    parser.add_argument(
        "--no-signalgroup",
        action="store_true",
        help="Skip signal group configuration to avoid receiver reboot",
    )
    parser.add_argument(
        "--signalgroup2",
        action="store_true",
        help="Convenience alias: apply 'CONFIG SIGNALGROUP 2' (UM980 mode)",
    )

    # Base Station Options
    parser.add_argument(
        "--msm",
        type=int,
        choices=[4, 7],
        default=4,
        help="Base mode: MSM message type (4=compact MSM4 for LoRa radio, 7=full MSM7) (default: 4)",
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
        help="Base mode: observation duration in seconds for autonomous survey-in (default: 3600)",
    )
    parser.add_argument(
        "--survey-accuracy",
        type=float,
        default=2.0,
        help="Base mode: 3D standard deviation cutoff in meters for survey-in (default: 2.0)",
    )

    # Rover Options
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

    args = parser.parse_args()

    # Apply Smart Device Defaults
    profile = DEVICE_PROFILES[args.device]
    if args.port is None:
        args.port = profile["port"]
    if args.mode is None:
        args.mode = profile["mode"]

    # Signal group resolution
    if args.signalgroup2:
        args.signalgroup = "2"
    elif args.signalgroup is None and not args.no_signalgroup:
        args.signalgroup = profile["signalgroup"]

    return args


def configure_signalgroup(configurator: UnicoreConfigurator, device: str, mode: str, requested_sg: str) -> None:
    """Validates and executes the CONFIG SIGNALGROUP command."""
    tokens = requested_sg.strip().split()

    if device == "um980":
        if len(tokens) > 1:
            print(f"[-] Notice: UM980 only accepts single-parameter signal groups. Using '{tokens[0]}'.")
        sg_cmd = f"CONFIG SIGNALGROUP {tokens[0]}"
    else:  # um982
        if len(tokens) == 1:
            if tokens[0] == "2":
                print("[-] Notice: Signal Group 2 is single-antenna. On UM982 Rover, dual groups are '3 6' or '4 5'.")
            if mode == "base":
                print(f"[*] UM982 Base mode: appending slave disable '0' to group '{tokens[0]}'.")
                sg_cmd = f"CONFIG SIGNALGROUP {tokens[0]} 0"
            else:
                default_slave = "6" if tokens[0] == "3" else "5"
                print(f"[-] Warning: UM982 Rover requires both Master and Slave signal groups.")
                print(f"    Auto-completing to: CONFIG SIGNALGROUP {tokens[0]} {default_slave}")
                sg_cmd = f"CONFIG SIGNALGROUP {tokens[0]} {default_slave}"
        else:
            sg_cmd = f"CONFIG SIGNALGROUP {tokens[0]} {tokens[1]}"

    print(f"\n[*] Applying {sg_cmd}...")
    configurator.send_command(sg_cmd)
    # Signal group changes trigger an internal receiver restart
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
    # 1230: GLONASS L1/L2 code-phase biases (every 10s). Essential for rover to resolve GLONASS carrier ambiguities.
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

    # Dual-antenna heading configuration (for UM982)
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
        configurator.send_command(f"UNIHEADINGA COM1 {period_str}", delay=0.05)
        configurator.send_command("GNGGAH COM1 1", delay=0.05)

    configurator.send_command("BASEINFOA COM1 1", delay=0.05)


def main() -> int:
    """Main execution flow."""
    args = parse_args()

    print(f"=== {__app_name__} v{__version__} ===")
    print(f"[*] Hardware Profile: {DEVICE_PROFILES[args.device]['desc']}")
    print(f"[*] Serial Connection: {args.port} @ {args.baud} baud | Mode: {args.mode.upper()}")

    configurator = UnicoreConfigurator(port=args.port, baud=args.baud)
    configurator.open()

    try:
        # If user only wants diagnostic query dump:
        if args.query:
            configurator.query_all()
            return 0

        # 1. Factory Reset (if requested)
        if args.freset:
            print("\n[*] Executing Factory Reset (FRESET)...")
            configurator.send_command("FRESET")
            configurator.wait_for_reboot(wait_seconds=6.5)

        # 2. Clear existing message streams
        print("\n[*] Clearing existing message streams...")
        configurator.send_command("UNLOG COM1")
        configurator.send_command("UNLOG COM2")

        # 3. Configure Signal Group (if requested or defaulted)
        if args.signalgroup and not args.no_signalgroup:
            configure_signalgroup(configurator, args.device, args.mode, args.signalgroup)

        # 4. Configure Port Baud Rates
        print(f"\n[*] Configuring port baud rates (COM1={args.baud}, COM2={args.com2_baud})...")
        configurator.send_command(f"CONFIG COM1 {args.baud}")
        configurator.send_command(f"CONFIG COM2 {args.com2_baud}")

        # 5. Apply Operating Mode Configuration
        if args.mode == "base":
            configure_base(configurator, args)
        else:
            configure_rover(configurator, args)

        # 6. Verify with Diagnostic Query
        print("\n[*] Verifying receiver state...")
        configurator.send_command("CONFIG", delay=0.2, wait_more=True)
        configurator.send_command("MODE", delay=0.1)
        configurator.send_command("UNILOGLIST", delay=0.1, wait_more=True)

        # 7. Persist to NVRAM
        if not args.no_save:
            print("\n[*] Persisting settings to NVRAM (SAVECONFIG)...")
            configurator.send_command("SAVECONFIG", delay=0.5)
        else:
            print("\n[*] Skipped SAVECONFIG (--no-save flag active).")

        print(f"\n[+] {args.device.upper()} {args.mode.upper()} configuration successfully applied.")
        return 0

    except KeyboardInterrupt:
        print("\n[!] Execution interrupted by user.", file=sys.stderr)
        return 130
    finally:
        configurator.close()


if __name__ == "__main__":
    sys.exit(main())
