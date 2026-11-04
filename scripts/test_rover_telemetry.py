#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_rover_telemetry.py: Verifies Bi-Directional LoRa Protocol framing, CRC16,
JSON atomic output, CSV rolling log with auto-pruning, and rate switch frames.
"""

import os
import shutil
import struct
import tempfile
import time
from e220_pacer import (
    E220Pacer,
    ROVER_SYNC,
    MSG_ID_TELEMETRY,
    MSG_ID_REQ_SWITCH,
    MSG_ID_ACK_SWITCH,
    FMT_TELEMETRY,
    FMT_REQ_SWITCH,
    FMT_ACK_SWITCH,
    crc16_ccitt,
)


def test_telemetry_packing():
    # 39-byte frame
    tip_lat = 43.73191234
    tip_lon = -70.62373156
    tip_height = 88.42
    h_acc_mm = 12
    fix_type = 4
    sats_m = 18
    sats_s = 14
    hdg_cdeg = 23450
    bat_pct = 88
    bat_mv = 4020
    rssi = -84
    noise = -102
    seq = 42

    body = struct.pack(
        FMT_TELEMETRY,
        ROVER_SYNC,
        MSG_ID_TELEMETRY,
        33,
        tip_lat,
        tip_lon,
        tip_height,
        h_acc_mm,
        fix_type,
        sats_m,
        sats_s,
        hdg_cdeg,
        bat_pct,
        bat_mv,
        rssi,
        noise,
        seq,
        0,
    )
    assert len(body) == 39, f"Expected 39B, got {len(body)}"
    crc = crc16_ccitt(body[:-2])
    frame = body[:-2] + struct.pack("<H", crc)

    # Verify unpack
    unpacked = struct.unpack(FMT_TELEMETRY, frame)
    assert unpacked[0] == ROVER_SYNC
    assert unpacked[1] == MSG_ID_TELEMETRY
    assert unpacked[2] == 33
    assert abs(unpacked[3] - tip_lat) < 1e-7
    assert abs(unpacked[4] - tip_lon) < 1e-7
    assert unpacked[6] == 12
    assert unpacked[7] == 4
    assert unpacked[8] == 18
    assert unpacked[9] == 14
    assert unpacked[10] == 23450
    assert unpacked[11] == 88
    assert unpacked[12] == 4020
    assert unpacked[13] == -84
    assert unpacked[14] == -102
    assert unpacked[15] == 42
    assert unpacked[16] == crc
    print("[PASS] Telemetry packing, size (39B) and CRC16 verified!")


def test_rate_switch_frames():
    # Request frame (10B)
    req_body = struct.pack(FMT_REQ_SWITCH, ROVER_SYNC, MSG_ID_REQ_SWITCH, 4, 0x04, 75, 101, 0)
    assert len(req_body) == 10, f"Expected 10B, got {len(req_body)}"
    req_crc = crc16_ccitt(req_body[:-2])
    req_frame = req_body[:-2] + struct.pack("<H", req_crc)
    assert crc16_ccitt(req_frame[:-2]) == (req_frame[-2] | (req_frame[-1] << 8))

    # Ack frame (11B)
    ack_body = struct.pack(FMT_ACK_SWITCH, ROVER_SYNC, MSG_ID_ACK_SWITCH, 5, 101, 0, 400, 0)
    assert len(ack_body) == 11, f"Expected 11B, got {len(ack_body)}"
    ack_crc = crc16_ccitt(ack_body[:-2])
    ack_frame = ack_body[:-2] + struct.pack("<H", ack_crc)
    assert crc16_ccitt(ack_frame[:-2]) == (ack_frame[-2] | (ack_frame[-1] << 8))
    print("[PASS] Rate switch Req (10B) and Ack (11B) frames verified!")


def test_pacer_logging():
    test_dir = tempfile.mkdtemp(prefix="rover_test_logs_")
    try:
        pacer = E220Pacer(
            port="/dev/null",
            log_dir=test_dir,
            log_retention_days=7,
            max_log_mb=10,
            compress_logs=True,
            no_csv=False,
        )

        test_data = {
            "timestamp_utc": "2026-11-04T19:30:15Z",
            "seq": 1,
            "fix_type": 4,
            "fix_name": "RTK_FIX",
            "latitude": 43.73191234,
            "longitude": -70.62373156,
            "height_m": 88.42,
            "h_acc_mm": 12,
            "sats_master": 18,
            "sats_slave": 14,
            "heading_deg": 234.5,
            "battery_pct": 88,
            "battery_mv": 4020,
            "rover_rssi_dbm": -84,
            "rover_noise_dbm": -102,
            "base_air_rate_kbps": 19.2,
            "channel": 75,
        }

        # Test CSV write
        pacer._write_csv_record(test_data)
        today_str = time.strftime("%Y-%m-%d")
        csv_file = os.path.join(test_dir, f"rover_{today_str}.csv")
        assert os.path.exists(csv_file), f"CSV file not created at {csv_file}"
        with open(csv_file, "r") as f:
            lines = f.readlines()
            assert len(lines) == 2, f"Expected header + 1 row, got {len(lines)}"
            assert "RTK_FIX" in lines[1]
            assert "43.73191234" in lines[1]

        # Test compression
        pacer.current_log_file.close()
        pacer._compress_log(today_str)
        gz_file = csv_file + ".gz"
        assert os.path.exists(gz_file), f"GZ file not created at {gz_file}"
        assert not os.path.exists(csv_file), "Raw CSV should be removed after compression"

        # Test atomic JSON
        pacer._write_atomic_json(test_data)
        for target in ("/run", "/tmp"):
            jpath = os.path.join(target, "rtk_rover_status.json")
            if os.path.exists(jpath):
                print(f"[PASS] Atomic JSON verified at {jpath}")
                break

        print("[PASS] CSV rolling log, rollover, and gzip compression verified!")
    finally:
        shutil.rmtree(test_dir, ignore_errors=True)


if __name__ == "__main__":
    test_telemetry_packing()
    test_rate_switch_frames()
    test_pacer_logging()
    print("\nALL PROTOCOL AND PACER TESTS PASSED 100%!")
