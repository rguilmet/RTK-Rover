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
__version__ = "v1.7.0"

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

    def test_f03_welford_dedup_fix_stratification_and_tilt_gate(self):
        # Reference implementation mirroring waypoint_survey_log.cpp F-03 logic
        rank_map = {0: 0, 1: 1, 2: 2, 3: 2, 4: 4, 5: 3}
        lat0, lon0 = 44.0, -70.0
        r_m, r_n = wgs84_radii(math.radians(lat0))
        cos_lat = math.cos(math.radians(lat0))
        dlat_1cm = math.degrees(0.01 / r_m)
        dlon_1cm = math.degrees(0.01 / (r_n * cos_lat))

        # 30 Hz queue ticks across 4 GGA epochs:
        # seq 1: FLT (warm-up), seq 2: FIX (+1cm N), seq 2 duplicate (must be ignored!),
        # seq 3: FLT dip (must be skipped!), seq 4: FIX with 4.0 deg tilt (skipped!),
        # seq 5: FIX (-1cm N)
        ticks = [
            {"gga_seq": 1, "fix": 5, "lat": lat0 + 50 * dlat_1cm, "lon": lon0, "alt": 100.0, "tilt": 0.5},
            {"gga_seq": 2, "fix": 4, "lat": lat0 + dlat_1cm, "lon": lon0, "alt": 100.0, "tilt": 0.6},
            {"gga_seq": 2, "fix": 4, "lat": lat0 + 99 * dlat_1cm, "lon": lon0, "alt": 100.0, "tilt": 0.6},
            {"gga_seq": 3, "fix": 5, "lat": lat0 + 20 * dlat_1cm, "lon": lon0 + dlon_1cm, "alt": 100.5, "tilt": 0.5},
            {"gga_seq": 4, "fix": 4, "lat": lat0 + 30 * dlat_1cm, "lon": lon0, "alt": 101.0, "tilt": 4.0},
            {"gga_seq": 5, "fix": 4, "lat": lat0 - dlat_1cm, "lon": lon0, "alt": 100.2, "tilt": 0.8},
        ]

        best_fix = 0
        last_seq = 0
        skip_fix = 0
        skip_tilt = 0
        n = 0
        mean_n = mean_e = mean_u = 0.0
        m2_n = m2_e = m2_u = 0.0
        ref = None

        for t in ticks:
            if t["gga_seq"] == last_seq:
                continue
            last_seq = t["gga_seq"]
            r = rank_map[t["fix"]]
            if r > rank_map[best_fix]:
                best_fix = t["fix"]
                n = 0
                mean_n = mean_e = mean_u = m2_n = m2_e = m2_u = 0.0
                ref = None
            if r < rank_map[best_fix]:
                skip_fix += 1
                continue
            if t["tilt"] > 2.5:
                skip_tilt += 1
                continue
            if ref is None:
                ref = (t["lat"], t["lon"], t["alt"])
            dn = (t["lat"] - ref[0]) * math.pi / 180.0 * r_m
            de = (t["lon"] - ref[1]) * math.pi / 180.0 * r_n * cos_lat
            du = t["alt"] - ref[2]
            n += 1
            d1 = dn - mean_n
            mean_n += d1 / n
            m2_n += d1 * (dn - mean_n)
            d2 = de - mean_e
            mean_e += d2 / n
            m2_e += d2 * (de - mean_e)
            d3 = du - mean_u
            mean_u += d3 / n
            m2_u += d3 * (du - mean_u)

        self.assertEqual(best_fix, 4)
        self.assertEqual(n, 2)
        self.assertEqual(skip_fix, 1)
        self.assertEqual(skip_tilt, 1)
        avg_lat = ref[0] + math.degrees(mean_n / r_m)
        self.assertAlmostEqual(avg_lat, lat0, places=9)
        h_sigma = math.sqrt((m2_n + m2_e) / (n - 1))
        # Points at +1cm and -1cm have sample std dev = sqrt(2) * 1cm ~= 0.01414 m
        self.assertAlmostEqual(h_sigma, math.sqrt(2.0) * 0.01, places=4)

    def test_survey_and_lever_arm_source_guards(self):
        survey_cpp = os.path.join(ROVER_ROOT, "src", "waypoint_manager", "waypoint_survey_log.cpp")
        with open(survey_cpp, "r", encoding="utf-8") as fh:
            stext = fh.read()
        self.assertIn("s_last_survey_gga_seq", stext)
        self.assertIn("fix_quality_rank", stext)
        self.assertIn("survey_max_tilt_deg", stext)
        self.assertIn("waypoint_flush_pending_survey_csv", stext)

        nmea_cpp = os.path.join(ROVER_ROOT, "src", "um982_driver", "um982_nmea_parser.cpp")
        with open(nmea_cpp, "r", encoding="utf-8") as fh:
            ntext = fh.read()
        self.assertIn("s_gga_seq", ntext)
        self.assertIn("h_acc_from_gst", ntext)

        ino_path = os.path.join(ROVER_ROOT, "RTK_Rover.ino")
        with open(ino_path, "r", encoding="utf-8") as fh:
            itext = fh.read()
        self.assertIn("apply_lever_arm_on_calibrated_beam", itext)
        self.assertIn("waypoint_flush_pending_survey_csv()", itext)

        nav_cpp = os.path.join(ROVER_ROOT, "src", "nav_engine", "nav_engine.cpp")
        with open(nav_cpp, "r", encoding="utf-8") as fh:
            nav_text = fh.read()
        self.assertIn("beam_comp_preferred", nav_text)

        sensors_cpp = os.path.join(ROVER_ROOT, "src", "rover_sensors", "rover_sensors.cpp")
        with open(sensors_cpp, "r", encoding="utf-8") as fh:
            st_text = fh.read()
        self.assertIn("2500U", st_text)

    def test_beam_heading_priority_over_slv_flt(self):
        def select_source(dual_valid, dual_fixed, bl_ok, beam_10dof, mag_ok, mag_anom, mag_cal_valid):
            compass_healthy = mag_ok and not mag_anom
            beam_comp_preferred = compass_healthy and beam_10dof and mag_cal_valid
            dual_flt_ok = (not beam_comp_preferred) and bl_ok
            if dual_valid and (dual_fixed or dual_flt_ok):
                return "DUAL_FIX" if dual_fixed else "DUAL_FLT"
            if compass_healthy:
                return "COMP"
            return "FALLBACK"

        self.assertEqual(select_source(True, True, True, True, True, False, True), "DUAL_FIX")
        self.assertEqual(select_source(True, False, True, True, True, False, True), "COMP")
        self.assertEqual(select_source(True, False, True, True, True, True, True), "DUAL_FLT")
        self.assertEqual(select_source(True, False, True, True, True, False, False), "DUAL_FLT")
        self.assertEqual(select_source(True, False, False, True, True, True, True), "FALLBACK")

    def test_low_battery_debounce_and_hysteresis(self):
        # Reference state machine mirroring rover_power.cpp (< 3.40 V for >= 5 s, clear >= 3.55 V / USB)
        class LowBattSim:
            def __init__(self):
                self.since_ms = 0
                self.debounce_s = 0
                self.active = False
                self.flushed = False
                self.flush_count = 0

            def step(self, now_ms, online, vbus, charging, vbat):
                on_batt_low = online and (not vbus) and (not charging) and (2.0 <= vbat < 3.40)
                recovered = online and (vbus or charging or (vbat >= 3.55))
                if not self.active:
                    if on_batt_low:
                        if self.since_ms == 0:
                            self.since_ms = now_ms if now_ms != 0 else 1
                        elapsed = now_ms - self.since_ms
                        self.debounce_s = max(1, min(5, (elapsed + 999) // 1000))
                        if elapsed >= 5000:
                            self.active = True
                            self.flushed = True
                            self.flush_count += 1
                    else:
                        self.since_ms = 0
                        self.debounce_s = 0
                elif recovered:
                    self.active = False
                    self.flushed = False
                    self.since_ms = 0
                    self.debounce_s = 0

        sim = LowBattSim()
        # Reject 0.00 V / offline & transient < 3.40 V dip < 5 s
        sim.step(1000, False, False, False, 0.0)
        self.assertFalse(sim.active)
        sim.step(2000, True, False, False, 3.35)
        sim.step(5000, True, False, False, 3.38)
        self.assertEqual(sim.debounce_s, 3)
        self.assertFalse(sim.active)
        sim.step(6000, True, False, False, 3.42)
        self.assertEqual(sim.debounce_s, 0)

        # USB / charging inhibits trigger
        sim.step(7000, True, True, False, 3.30)
        sim.step(13000, True, True, False, 3.30)
        self.assertFalse(sim.active)

        # Sustained >= 5 s on battery triggers single-shot flush and holds across 3.48 V hysteresis band
        sim.step(20000, True, False, False, 3.38)
        sim.step(23000, True, False, False, 3.36)
        self.assertFalse(sim.active)
        sim.step(25000, True, False, False, 3.34)
        self.assertTrue(sim.active)
        self.assertEqual(sim.flush_count, 1)
        sim.step(28000, True, False, False, 3.48)
        self.assertTrue(sim.active)
        self.assertEqual(sim.flush_count, 1)

        # Clears at >= 3.55 V (or USB plug-in)
        sim.step(30000, True, False, False, 3.56)
        self.assertFalse(sim.active)
        self.assertFalse(sim.flushed)

        # Source guards across Phase 2.2 files
        with open(os.path.join(ROVER_ROOT, "src", "rover_power", "rover_power.h"), "r", encoding="utf-8") as fh:
            pwr_h = fh.read()
        with open(os.path.join(ROVER_ROOT, "src", "rover_power", "rover_power.cpp"), "r", encoding="utf-8") as fh:
            pwr_cpp = fh.read()
        with open(os.path.join(ROVER_ROOT, "RTK_Rover.ino"), "r", encoding="utf-8") as fh:
            ino_txt = fh.read()
        with open(os.path.join(ROVER_ROOT, "src", "lvgl_port", "ui_header_nav.c"), "r", encoding="utf-8") as fh:
            hdr_txt = fh.read()
        self.assertIn("ROVER_LOW_BATT_THRESH_V", pwr_h)
        self.assertIn("rover_power_service_deferred_flush", pwr_cpp)
        self.assertIn("waypoint_flush_pending_survey_csv()", pwr_cpp)
        self.assertIn("rover_power_service_deferred_flush()", ino_txt)
        self.assertIn("low_batt_modal", hdr_txt)

    def test_phase23b_ble_bridge_and_f11_portal_guards(self):
        import json
        # F-11: zero hardcoded "12345678" fallback password in Core portal or RTK_Rover
        web_cpp = os.path.join(CORE_ROOT, "src", "net", "fic_web_portal.cpp")
        with open(web_cpp, "r", encoding="utf-8") as fh:
            wtext = fh.read()
        self.assertNotIn('"12345678"', wtext)
        self.assertIn("get_ap_creds_cb", wtext)
        self.assertIn("is_write_locked_cb", wtext)

        # Schema defaults in settings.json (public repo: ble_enabled=true, empty ap_password)
        with open(os.path.join(ROVER_ROOT, "settings.json"), "r", encoding="utf-8") as fh:
            sjson = json.load(fh)
        self.assertTrue(sjson["ble_enabled"])
        self.assertEqual(sjson["ap_ssid"], "RTK_Rover_AP")
        self.assertEqual(sjson["ap_password"], "")

        # BLE companion profile & lock guard bindings in ble_bridge.cpp and RTK_Rover.ino
        bridge_cpp = os.path.join(ROVER_ROOT, "src", "ble_bridge", "ble_bridge.cpp")
        with open(bridge_cpp, "r", encoding="utf-8") as fh:
            btext = fh.read()
        self.assertIn('"rtk_rover"', btext)
        self.assertIn('"RTK-Rover"', btext)
        self.assertIn("kTelemLen = 83U", btext)
        self.assertIn("rover_is_storage_write_locked", btext)

        with open(os.path.join(ROVER_ROOT, "RTK_Rover.ino"), "r", encoding="utf-8") as fh:
            ino_txt = fh.read()
        self.assertNotIn('"12345678"', ino_txt)
        self.assertIn("<ble/fic_ble_enable.h>", ino_txt)
        self.assertIn("rover_get_ap_creds_cb", ino_txt)
        self.assertIn("rover_is_storage_write_locked", ino_txt)
        self.assertIn("ble_bridge_init()", ino_txt)
        self.assertIn("ble_bridge_loop()", ino_txt)

    def test_v0121_comms_consolidation_and_dram0_guards(self):
        with open(os.path.join(CORE_ROOT, "src", "net", "fic_web_portal.h"), "r", encoding="utf-8") as fh:
            wph = fh.read()
        with open(os.path.join(CORE_ROOT, "src", "net", "fic_web_portal.cpp"), "r", encoding="utf-8") as fh:
            wpc = fh.read()
        self.assertIn("fic_web_portal_set_ble_auto_resume", wph)
        self.assertIn("fic_web_portal_is_busy", wph)
        self.assertIn("fic_web_portal_set_ble_auto_resume", wpc)
        self.assertIn("fic_web_portal_is_busy", wpc)

        with open(os.path.join(ROVER_ROOT, "src", "ble_bridge", "ble_bridge.h"), "r", encoding="utf-8") as fh:
            bbh = fh.read()
        with open(os.path.join(ROVER_ROOT, "src", "ble_bridge", "ble_bridge.cpp"), "r", encoding="utf-8") as fh:
            bbc = fh.read()
        self.assertIn("ble_bridge_request_comms_cycle", bbh)
        self.assertIn("ble_bridge_get_comms_ui_state", bbh)
        self.assertIn("fic_web_portal_set_ble_auto_resume(false)", bbc)
        self.assertIn("fic_web_portal_is_busy()", bbc)

        with open(os.path.join(ROVER_ROOT, "src", "lvgl_port", "ui_tile5_settings.c"), "r", encoding="utf-8") as fh:
            t5c = fh.read()
        self.assertIn("t5_calmag_btn   = create_grid_button(parent, 0, 3,", t5c)
        self.assertIn("t1_unit_btn     = create_grid_button(parent, 1, 3,", t5c)
        self.assertIn("t5_autooff_btn  = create_grid_button(parent, 2, 3,", t5c)
        self.assertIn("t1_lora_btn     = create_grid_button(parent, 0, 4,", t5c)
        self.assertIn("t5_comms_btn    = create_grid_button(parent, 1, 4,", t5c)
        self.assertIn("t5_rtcm_src_btn = create_grid_button(parent, 2, 4,", t5c)

        with open(os.path.join(ROVER_ROOT, "src", "lvgl_port", "ui_header_nav.c"), "r", encoding="utf-8") as fh:
            hnc = fh.read()
        self.assertIn("hdr_radio_lbl", hnc)
        self.assertIn("lv_obj_align(hdr_radio_lbl, LV_ALIGN_TOP_RIGHT, -118, 2)", hnc)
        self.assertIn("lv_obj_set_width(hdr_datetime_lbl, 165)", hnc)
        self.assertNotIn("hdr_ble_lbl", hnc)
        self.assertNotIn("hdr_wifi_lbl", hnc)

        with open(os.path.join(ROVER_ROOT, "RTK_Rover.ino"), "r", encoding="utf-8") as fh:
            ino_txt = fh.read()
        self.assertIn("xTaskCreatePinnedToCoreWithCaps", ino_txt)
        self.assertIn('create_psram_stack_task(task_rover_core0,       "RoverCore0"', ino_txt)
        self.assertIn('create_psram_stack_task(task_display_and_audio, "RoverUI"', ino_txt)

        with open(os.path.join(ROVER_ROOT, "src", "rover_power", "rover_power.cpp"), "r", encoding="utf-8") as fh:
            pwc = fh.read()
        self.assertIn("s_auto_off_poweroff_pending", pwc)
        self.assertIn("do_auto_off", pwc)

    def test_phase24_rtcm_arbiter_failover_and_hysteresis(self):
        # Reference FSM simulation matching rtcm_arbiter.cpp
        MODE_AUTO, MODE_LORA, MODE_NTRIP = 0, 1, 2
        SRC_NONE, SRC_LORA, SRC_NTRIP = 0, 1, 2
        st = {"mode": MODE_AUTO, "active": SRC_NONE, "last_lora": 0, "streak": 0, "failovers": 0}

        def submit(src, now_ms):
            if st["mode"] == MODE_LORA:
                if src == SRC_LORA:
                    st["last_lora"] = now_ms
                    st["active"] = SRC_LORA
                    return True
                return False
            if st["mode"] == MODE_NTRIP:
                if src == SRC_NTRIP:
                    st["active"] = SRC_NTRIP
                    return True
                return False
            if src == SRC_LORA:
                st["last_lora"] = now_ms
                if st["active"] == SRC_NTRIP:
                    st["streak"] += 1
                    if st["streak"] >= 3:
                        st["active"] = SRC_LORA
                        st["streak"] = 0
                        st["failovers"] += 1
                        return True
                    return False
                st["active"] = SRC_LORA
                st["streak"] = 0
                return True
            timed_out = (st["last_lora"] == 0) or ((now_ms - st["last_lora"]) > 3000)
            if st["active"] == SRC_NTRIP:
                if timed_out:
                    st["streak"] = 0
                return True
            if timed_out:
                if st["active"] == SRC_LORA:
                    st["failovers"] += 1
                st["active"] = SRC_NTRIP
                st["streak"] = 0
                return True
            return False

        self.assertTrue(submit(SRC_LORA, 1000))
        self.assertEqual(st["active"], SRC_LORA)
        self.assertFalse(submit(SRC_NTRIP, 2500))  # LoRa fresh (<= 3000ms) -> suppress NTRIP
        self.assertTrue(submit(SRC_NTRIP, 4500))   # > 3000ms silence -> failover to BLE NTRIP
        self.assertEqual(st["active"], SRC_NTRIP)
        self.assertEqual(st["failovers"], 1)
        self.assertFalse(submit(SRC_LORA, 5000))   # Recovery frame 1/3 held
        self.assertFalse(submit(SRC_LORA, 5200))   # Recovery frame 2/3 held
        self.assertTrue(submit(SRC_LORA, 5400))    # Recovery frame 3/3 promotes back to LoRa
        self.assertEqual(st["active"], SRC_LORA)
        self.assertEqual(st["failovers"], 2)

        st["mode"] = MODE_LORA
        self.assertFalse(submit(SRC_NTRIP, 99999))
        st["mode"] = MODE_NTRIP
        self.assertFalse(submit(SRC_LORA, 100000))
        self.assertTrue(submit(SRC_NTRIP, 100100))

        # Source wiring & PSRAM/gatekeeper assertions
        arb_h = os.path.join(ROVER_ROOT, "src", "rtcm_arbiter", "rtcm_arbiter.h")
        arb_cpp = os.path.join(ROVER_ROOT, "src", "rtcm_arbiter", "rtcm_arbiter.cpp")
        gk_cpp = os.path.join(ROVER_ROOT, "src", "e220_driver", "e220_rtcm_gatekeeper.cpp")
        with open(arb_h, "r", encoding="utf-8") as fh:
            ah = fh.read()
        with open(arb_cpp, "r", encoding="utf-8") as fh:
            ac = fh.read()
        with open(gk_cpp, "r", encoding="utf-8") as fh:
            gk = fh.read()
        self.assertIn("RTCM_ARBITER_LORA_FAILOVER_MS", ah)
        self.assertIn("MALLOC_CAP_SPIRAM", ac)
        self.assertIn("e220_rtcm_validate_frame", ac)
        self.assertIn("rtcm_arbiter_submit_frame(RTCM_SRC_LORA", gk)
        self.assertNotIn("um982_forward_rtcm(", gk)

        with open(os.path.join(ROVER_ROOT, "src", "ble_bridge", "ble_bridge.cpp"), "r", encoding="utf-8") as fh:
            bbc = fh.read()
        with open(os.path.join(ROVER_ROOT, "src", "ble_bridge", "ble_bridge_rpc.cpp"), "r", encoding="utf-8") as fh:
            rpc_c = fh.read()
        self.assertIn("rtcm_arbiter_submit_ble_stream", bbc)
        self.assertIn("FICP_MSG_ROVER_RTCM_SET_MODE", rpc_c)
        self.assertIn("rtcm_arbiter_set_mode_from_str", rpc_c)

        with open(os.path.join(ROVER_ROOT, "src", "lvgl_port", "ui_tile5_settings.c"), "r", encoding="utf-8") as fh:
            t5c = fh.read()
        with open(os.path.join(ROVER_ROOT, "src", "lvgl_port", "ui_tile4_diagnostics.c"), "r", encoding="utf-8") as fh:
            t4c = fh.read()
        with open(os.path.join(ROVER_ROOT, "src", "lvgl_port", "ui_header_nav.c"), "r", encoding="utf-8") as fh:
            hnc = fh.read()
        self.assertIn("rtcm_src_toggle_btn_cb", t5c)
        self.assertIn("t4_ble_ntrip_lbl", t4c)
        self.assertIn("RTCM_SRC_BLE_NTRIP", hnc)

    def test_phase25_ficp_extension_rpcs_and_state_reflection(self):
        rpc_cpp = os.path.join(ROVER_ROOT, "src", "ble_bridge", "ble_bridge_rpc.cpp")
        ble_cpp = os.path.join(ROVER_ROOT, "src", "ble_bridge", "ble_bridge.cpp")
        rem_cpp = os.path.join(ROVER_ROOT, "src", "waypoint_manager", "waypoint_remote_ext.cpp")
        csv_cpp = os.path.join(ROVER_ROOT, "src", "waypoint_manager", "waypoint_csv_store.cpp")
        srv_cpp = os.path.join(ROVER_ROOT, "src", "waypoint_manager", "waypoint_survey_log.cpp")
        with open(rpc_cpp, "r", encoding="utf-8") as fh:
            rpc_txt = fh.read()
        with open(ble_cpp, "r", encoding="utf-8") as fh:
            ble_txt = fh.read()
        with open(rem_cpp, "r", encoding="utf-8") as fh:
            rem_txt = fh.read()
        with open(csv_cpp, "r", encoding="utf-8") as fh:
            csv_txt = fh.read()
        with open(srv_cpp, "r", encoding="utf-8") as fh:
            srv_txt = fh.read()

        for sym in (
            "FICP_MSG_ROVER_SET_TARGET_POINT",
            "FICP_MSG_ROVER_SET_LINE_STAKEOUT",
            "FICP_MSG_ROVER_SELECT_WAYPOINT",
            "FICP_MSG_ROVER_SURVEY_LOG",
            "FICP_MSG_ROVER_ROUTE_REC_SET",
            "FICP_MSG_ROVER_RADIO_SET_RATE",
            "FICP_MSG_ROVER_RTCM_SET_MODE",
            "FICP_MSG_ROVER_SENSOR_CAL",
            "FICP_MSG_ROVER_POLE_GEOM_SET",
            "FICP_MSG_EVT_ROVER_SURVEY_RESULT",
            "ble_bridge_ext_rpc_dispatch",
            "ble_bridge_poll_local_reflection",
            "ble_bridge_poll_survey_progress",
            "ble_bridge_on_survey_result_event",
            "fic_ble_companion_send_ext_packet",
            "fic_ble_companion_notify_local_change",
        ):
            self.assertIn(sym, rpc_txt)

        self.assertIn("ble_bridge_ext_rpc_dispatch", ble_txt)
        self.assertIn("waypoint_survey_set_result_callback", ble_txt)
        self.assertIn("ble_bridge_poll_local_reflection", ble_txt)
        self.assertIn("ble_bridge_poll_survey_progress", ble_txt)
        self.assertIn('runtime_out["guidance_mode"]', ble_txt)

        self.assertIn("waypoint_remote_guidance_set_point", rem_txt)
        self.assertIn("waypoint_remote_guidance_set_line", rem_txt)
        self.assertIn("waypoint_select_by_filename", rem_txt)
        self.assertIn("waypoint_file_sync_active_idx", rem_txt)
        self.assertIn("waypoint_survey_ext_reset_overrides", rem_txt)
        self.assertIn("waypoint_remote_guidance_get_segment", csv_txt)
        self.assertIn("waypoint_remote_guidance_get_target", csv_txt)
        self.assertIn("waypoint_survey_ext_emit_result", srv_txt)
        self.assertIn("waypoint_survey_ext_reset_overrides", srv_txt)


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
        '#include "rtcm_arbiter/rtcm_arbiter.h"',
        '#include "ble_bridge/ble_bridge.h"',
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
