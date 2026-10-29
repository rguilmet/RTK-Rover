#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Module: test_e220_link.py
Description: Bidirectional RF link tester for two E220 transceivers, including 
             a high-volume burst test to validate FIFO buffer capabilities,
             subpacket reassembly, and over-the-air throughput.
Author: Roland L. Guilmet
License: MIT
"""

from __future__ import annotations

import argparse
import sys
import time
import os
try:
    import serial
except ImportError:
    serial = None  # type: ignore

__app_name__ = "test_e220_link"
__version__ = "1.1.0"
__date__ = "2026-10-29"


def perform_basic_test(s1: serial.Serial, s2: serial.Serial, port1: str, port2: str) -> int:
    print("--------------------------------------------------")
    print("[*] Phase 1: Basic Link Verification")
    
    token_a = os.urandom(4).hex().upper()
    token_b = os.urandom(4).hex().upper()
    
    msg_a2b = f"PING_A2B_{token_a}\n".encode('ascii')
    msg_b2a = f"PING_B2A_{token_b}\n".encode('ascii')

    errors = 0

    print(f"[*] Direction 1: {port1} -> {port2}")
    s1.reset_input_buffer()
    s2.reset_input_buffer()
    s1.write(msg_a2b)
    s1.flush()
    time.sleep(0.2)
    rx_2 = s2.read_until(b'\n')
    if msg_a2b in rx_2:
        print(f"[+] PASS: Received {msg_a2b.decode('ascii').strip()}")
    else:
        print(f"[-] FAIL: Received {rx_2!r} (Expected: {msg_a2b!r})")
        errors += 1

    print(f"[*] Direction 2: {port2} -> {port1}")
    s1.reset_input_buffer()
    s2.reset_input_buffer()
    s2.write(msg_b2a)
    s2.flush()
    time.sleep(0.2)
    rx_1 = s1.read_until(b'\n')
    if msg_b2a in rx_1:
        print(f"[+] PASS: Received {msg_b2a.decode('ascii').strip()}")
    else:
        print(f"[-] FAIL: Received {rx_1!r} (Expected: {msg_b2a!r})")
        errors += 1

    return errors


def perform_burst_test(s1: serial.Serial, s2: serial.Serial, port1: str, port2: str, burst_size: int) -> int:
    print("--------------------------------------------------")
    print(f"[*] Phase 2: RTCM Burst Simulation ({burst_size} bytes)")
    print(f"[*] Sending from {port1} -> {port2} to test FIFO buffer...")
    
    # Generate sequential payload
    payload = bytearray()
    seq = 0
    while len(payload) < burst_size:
        chunk = f"SQ:{seq:04d}_ABCDEFGHIJKLMNOPQRSTUVWXYZ\n".encode('ascii')
        if len(payload) + len(chunk) > burst_size:
            chunk = chunk[:burst_size - len(payload)]
        payload.extend(chunk)
        seq += 1

    s1.reset_input_buffer()
    s2.reset_input_buffer()
    
    start_time = time.time()
    s1.write(payload)
    s1.flush()
    
    # Adaptive reception loop with sliding silence window
    rx_data = bytearray()
    max_wait = max(4.0, (burst_size / 2000.0) + 2.0)
    last_byte_time = time.time()
    
    while time.time() - start_time < max_wait:
        avail = s2.in_waiting
        if avail > 0:
            chunk = s2.read(avail)
            rx_data.extend(chunk)
            last_byte_time = time.time()
            if len(rx_data) >= burst_size:
                break
        else:
            # If data has started arriving, 350ms of trailing silence indicates burst completion
            if len(rx_data) > 0 and (time.time() - last_byte_time > 0.35):
                break
            time.sleep(0.01)

    elapsed = time.time() - start_time
    throughput_kbps = (len(rx_data) * 8.0) / (elapsed * 1000.0) if elapsed > 0 else 0.0

    print(f"[*] Sent:    {len(payload)} bytes")
    print(f"[*] Rcvd:    {len(rx_data)} bytes in {elapsed:.2f}s ({throughput_kbps:.1f} kbps effective)")
    
    if payload == rx_data:
        print("[+] PASS: Burst payload received perfectly with 0 dropped bytes.")
        return 0
    else:
        print("[-] FAIL: Burst payload mismatch!")
        drop_index = -1
        for i in range(min(len(payload), len(rx_data))):
            if payload[i] != rx_data[i]:
                drop_index = i
                break
                
        if drop_index == -1:
            drop_index = len(rx_data)
            print("[-] ERROR TYPE: Buffer truncation / partial reception.")
            print(f"[-] EXACT FAILURE: Data stopped flowing at byte count: {drop_index}/{len(payload)}")
        else:
            print("[-] ERROR TYPE: Data corruption mid-stream.")
            print(f"[-] EXACT FAILURE: Corruption started at byte offset: {drop_index}")
            
        print(f"[-] Last good chunk received around byte offset: {max(0, drop_index - 20)}")
        return 1


def perform_test(port1: str, port2: str, baud: int, burst: int) -> int:
    if serial is None:
        print("[-] Error: 'pyserial' package is required. Install with: pip install pyserial", file=sys.stderr)
        return 1
    print(f"=== {__app_name__} v{__version__} ===")
    print(f"[*] Opening {port1} and {port2} at {baud} baud...")
    try:
        s1 = serial.Serial(port1, baud, timeout=2.0)
        s2 = serial.Serial(port2, baud, timeout=2.0)
        
        # Explicitly assert Mode 0 (Transparent Mode: DTR=True, RTS=True) on both interfaces
        try:
            s1.dtr = True
            s1.rts = True
        except Exception:
            pass
        try:
            s2.dtr = True
            s2.rts = True
        except Exception:
            pass

        time.sleep(0.5)
    except serial.SerialException as err:
        print(f"[-] Failed to open serial ports: {err}", file=sys.stderr)
        return 1

    s1.reset_input_buffer()
    s2.reset_input_buffer()

    err1 = perform_basic_test(s1, s2, port1, port2)
    err2 = perform_burst_test(s1, s2, port1, port2, burst)
    
    print("--------------------------------------------------")
    s1.close()
    s2.close()
    
    total_errors = err1 + err2
    if total_errors == 0:
        print("[+] ALL TESTS PASSED. Hardware configuration and RF link are fully stable.")
        return 0
    else:
        print("[-] TESTS FAILED. Review baud mismatch, subpacket sizes, or RF path.")
        return 1


def main() -> int:
    parser = argparse.ArgumentParser(
        description=f"Stress test RF link between two E220 modules (v{__version__})."
    )
    parser.add_argument("--port1", default="/dev/ttyAMA0", help="Transmitter E220 port (default: /dev/ttyAMA0)")
    parser.add_argument("--port2", default="/dev/ttyUSB0", help="Receiver E220 port (default: /dev/ttyUSB0)")
    parser.add_argument("--baud", type=int, default=19200, help="Operational UART baud rate (default: 19200)")
    parser.add_argument("--burst-size", type=int, default=2100, help="Burst payload size in bytes (default: 2100)")
    parser.add_argument("-v", "--version", action="version", version=f"{__app_name__} {__version__} ({__date__})")
    
    args = parser.parse_args()
    return perform_test(args.port1, args.port2, args.baud, args.burst_size)


if __name__ == "__main__":
    sys.exit(main())
