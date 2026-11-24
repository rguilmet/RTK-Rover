#!/usr/bin/env python3
"""
F-17 host unit tests and ESP32-S3 syntax gate for RTK_Rover navigation math,
UM982 NMEA/Unicore frame verification, and file-hygiene invariants.
"""

import glob
import math
import os
import re
import subprocess
import sys
import unittest

__filename__ = os.path.basename(__file__)
__version__ = "v1.0.0"

ROVER_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
CORE_ROOT = os.path.abspath(os.path.join(ROVER_ROOT, "..", "libraries", "Field_Instrument_Core"))

WGS84_A = 6378137.0
WGS84_E2 = 0.00669437999014
METERS_TO_FEET = 3.280839895


def find_xtensa_gpp():
    base = os.path.expandvars(r"%LOCALAPPDATA%\Arduino15\packages\esp32\tools\esp-x32")
    matches = glob.glob(os.path.join(base, "*", "bin", "xtensa-esp32s3-elf-g++.exe"))
    return matches[0] if matches else None


def wgs84_radii(lat_rad):
    sin_lat = math.sin(lat_rad)
    denom = math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    r_m = WGS84_A * (1.0 - WGS84_E2) / (denom ** 3)
    r_n = WGS84_A / denom
    return r_m, r_n


def py_compute_line_stakeout(tip_lat, tip_lon, lat_a, lon_a, lat_b, lon_b):
    lat_mid = (lat_a + lat_b) * 0.5 * (math.pi / 180.0)
    r_m, r_n = wgs84_radii(lat_mid)
    dn_ab = (lat_b - lat_a) * (math.pi / 180.0) * r_m
    de_ab = (lon_b - lon_a) * (math.pi / 180.0) * r_n * math.cos(lat_mid)
    total_len = math.hypot(dn_ab, de_ab)
    if total_len < 0.001:
        return {"valid": False, "station_m": 0.0, "offset_m": 0.0, "total_length_m": 0.0, "remaining_m": 0.0, "pct_complete": 0.0}
    u_n = dn_ab / total_len
    u_e = de_ab / total_len
    brg = math.degrees(math.atan2(de_ab, dn_ab))
    if brg < 0.0:
        brg += 360.0
    dn_ar = (tip_lat - lat_a) * (math.pi / 180.0) * r_m
    de_ar = (tip_lon - lon_a) * (math.pi / 180.0) * r_n * math.cos(lat_mid)
    station = dn_ar * u_n + de_ar * u_e
    offset = de_ar * u_n - dn_ar * u_e
    remaining = total_len - station
    pct = (station / total_len) * 100.0
    return {
        "valid": True,
        "station_m": station,
        "offset_m": offset,
        "total_length_m": total_len,
        "remaining_m": remaining,
        "line_bearing_deg": brg,
        "pct_complete": pct,
    }


def py_format_signed_along_1dp(meters, use_imperial=False):
    if use_imperial:
        feet = meters * METERS_TO_FEET
        if abs(feet) < 1.0:
            inches = feet * 12.0
            if abs(inches) < 0.05:
                inches = 0.0
            return f"{inches:+.1f} in"
        if abs(feet) < 1000.0:
            return f"{feet:+.1f} ft"
        return f"{feet:+,.1f} ft"
    if abs(meters) < 1.0:
        cm = meters * 100.0
        if abs(cm) < 0.05:
            cm = 0.0
        return f"{cm:+.1f} cm"
    if abs(meters) < 1000.0:
        return f"{meters:+.1f} m"
    return f"{meters:+,.1f} m"


def py_format_along_line(station_m, pct_complete, remaining_m, use_imperial=False):
    a_str = py_format_signed_along_1dp(station_m, use_imperial)
    b_str = py_format_signed_along_1dp(remaining_m, use_imperial)
    pct = 0.0 if abs(pct_complete) < 0.05 else pct_complete
    return f"A: {a_str} ({pct:.1f}%) | Line->B: {b_str}"


def verify_nmea_checksum(sentence):
    if not sentence or sentence[0] != "$" or "*" not in sentence:
        return False
    star = sentence.rfind("*")
    if star + 3 > len(sentence):
        return False
    calc = 0
    for ch in sentence[1:star]:
        calc ^= ord(ch)
    try:
        expected = int(sentence[star + 1:star + 3], 16)
    except ValueError:
        return False
    return calc == expected


def unicore_crc32(data_bytes):
    crc = 0
    for b in data_bytes:
        val = (crc ^ b) & 0xFFFFFFFF
        for _ in range(8):
            if val & 1:
                val = ((val >> 1) ^ 0xEDB88320) & 0xFFFFFFFF
            else:
                val = (val >> 1) & 0xFFFFFFFF
        crc = ((crc >> 8) ^ val) & 0xFFFFFFFF
    return crc


def verify_unicore_crc32(line):
    if not line or line[0] != "#" or "*" not in line:
        return False
    star = line.rfind("*")
    hex_part = line[star + 1:].strip()
    if len(hex_part) < 8:
        return False
    try:
        expected = int(hex_part[:8], 16)
    except ValueError:
        return False
    calc = unicore_crc32(line[1:star].encode("ascii"))
    return calc == expected


class RoverCoreMathTests(unittest.TestCase):
    def test_nav_compute_line_stakeout_variants(self):
        lat_a, lon_a = 44.0, -70.0
        r_m, r_n = wgs84_radii(math.radians(lat_a))
        lat_b = lat_a + math.degrees(100.0 / r_m)
        lon_b = lon_a

        # Coincident guard (< 0.001 m)
        zero = py_compute_line_stakeout(lat_a, lon_a, lat_a, lon_a, lat_a, lon_a)
        self.assertFalse(zero["valid"])

        # Behind Point A (negative station) and 2.5 m left
        dlat_neg10 = math.degrees(-10.0 / r_m)
        dlon_left2p5 = math.degrees(-2.5 / (r_n * math.cos(math.radians(lat_a))))
        behind = py_compute_line_stakeout(lat_a + dlat_neg10, lon_a + dlon_left2p5, lat_a, lon_a, lat_b, lon_b)
        self.assertTrue(behind["valid"])
        self.assertAlmostEqual(behind["station_m"], -10.0, places=2)
        self.assertAlmostEqual(behind["offset_m"], -2.5, places=2)
        self.assertAlmostEqual(behind["remaining_m"], 110.0, places=2)

        # Overshoot past Point B (station = 112.0 m)
        dlat_112 = math.degrees(112.0 / r_m)
        over = py_compute_line_stakeout(lat_a + dlat_112, lon_a, lat_a, lon_a, lat_b, lon_b)
        self.assertAlmostEqual(over["remaining_m"], -12.0, places=2)
        self.assertGreater(over["pct_complete"], 110.0)

    def test_nav_format_along_line(self):
        self.assertEqual(py_format_along_line(-0.0001, -0.01, 50.0, use_imperial=False),
                         "A: +0.0 cm (0.0%) | Line->B: +50.0 m")
        self.assertEqual(py_format_along_line(0.3048, 25.0, 0.9144, use_imperial=True),
                         "A: +12.0 in (25.0%) | Line->B: +3.0 ft")
        self.assertEqual(py_format_along_line(0.6096, 50.0, -0.6096, use_imperial=True),
                         "A: +2.0 ft (50.0%) | Line->B: -2.0 ft")

    def test_nmea_xor_and_unicore_crc32_frames(self):
        body = "GNGGA,123519.00,4400.0000,N,07000.0000,W,4,16,0.7,120.5,M,-30.1,M,1.0,0001"
        xor_chk = 0
        for ch in body:
            xor_chk ^= ord(ch)
        valid_gga = f"${body}*{xor_chk:02X}"
        self.assertTrue(verify_nmea_checksum(valid_gga))
        self.assertFalse(verify_nmea_checksum(valid_gga.replace(",4,16,", ",1,16,")))
        self.assertFalse(verify_nmea_checksum("$GNGGA,123*ZZ"))

        ubody = "UNIHEADINGA,COM1,0,60.0,FINESTEERING,2340,123456.000,00000000,0000,1;SOL_COMPUTED,NARROW_INT,0.555,180.0,0.0,0.0,0.5,0.5,\"999\",12,12,12,12,0,0,0,0"
        ucrc = unicore_crc32(ubody.encode("ascii"))
        valid_uhdg = f"#{ubody}*{ucrc:08X}"
        self.assertTrue(verify_unicore_crc32(valid_uhdg))
        self.assertFalse(verify_unicore_crc32(valid_uhdg.replace("180.0", "181.0")))

    def test_nav_and_parser_source_constants(self):
        nav_cpp = os.path.join(ROVER_ROOT, "src", "nav_engine", "nav_engine.cpp")
        with open(nav_cpp, "r", encoding="utf-8") as fh:
            text = fh.read()
        self.assertIn("6378137.0", text)
        self.assertIn("0.00669437999014", text)

        nmea_cpp = os.path.join(ROVER_ROOT, "src", "um982_driver", "um982_nmea_parser.cpp")
        with open(nmea_cpp, "r", encoding="utf-8") as fh:
            ntext = fh.read()
        self.assertIn("0xEDB88320", ntext)

    def test_rover_src_line_byte_and_ascii_limits(self):
        src_dir = os.path.join(ROVER_ROOT, "src")
        checked_ascii = [
            os.path.join(ROVER_ROOT, "RTK_Rover.ino"),
            os.path.join(ROVER_ROOT, "user_config.h"),
        ]
        for r, _, fs in os.walk(src_dir):
            for f in sorted(fs):
                if not f.endswith((".c", ".cpp", ".h")):
                    continue
                fp = os.path.join(r, f)
                with open(fp, "rb") as fh:
                    data = fh.read()
                lines = data.count(b"\n") + (1 if data and not data.endswith(b"\n") else 0)
                self.assertLessEqual(lines, 800, f"{fp} exceeds 800 lines ({lines})")
                self.assertLess(len(data), 46080, f"{fp} exceeds 46,080 bytes ({len(data)})")
                if "e220_driver" not in fp:
                    bad = [b for b in data if b > 127]
                    self.assertFalse(bad, f"{fp} contains non-ASCII byte(s)")
        for fp in checked_ascii:
            with open(fp, "rb") as fh:
                data = fh.read()
            bad = [b for b in data if b > 127]
            self.assertFalse(bad, f"{fp} contains non-ASCII byte(s)")


def run_xtensa_syntax_checks():
    gpp = find_xtensa_gpp()
    if not gpp:
        print("[*] xtensa-esp32s3-elf-g++.exe not found; skipping syntax check.")
        return 0
    cmd = [
        gpp,
        "-std=c++17",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-fsyntax-only",
        "-I", os.path.join(ROVER_ROOT, "src"),
        "-I", os.path.join(CORE_ROOT, "src"),
        "-x", "c++",
        "-",
    ]
    probe_src = "\n".join([
        '#include "rover_api.h"',
        '#include "nav_engine/nav_engine.h"',
        '#include "math/fic_matrix_math.h"',
        '#include "fusion/fic_ahrs_9dof.h"',
        "int rover_host_syntax_probe() { return (int)NAV_BREADCRUMB_CAPACITY; }",
        "",
    ])
    res = subprocess.run(cmd, input=probe_src, text=True, capture_output=True)
    if res.returncode != 0:
        print(res.stdout)
        print(res.stderr)
        return res.returncode
    print("[PASS] ESP32-S3 C++17 -Wall -Wextra -Werror syntax check on Rover headers passed.")
    return 0


if __name__ == "__main__":
    print(f"=== {__filename__} {__version__} ===")
    rc = run_xtensa_syntax_checks()
    if rc != 0:
        sys.exit(rc)
    suite = unittest.defaultTestLoader.loadTestsFromTestCase(RoverCoreMathTests)
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    sys.exit(0 if result.wasSuccessful() else 1)
