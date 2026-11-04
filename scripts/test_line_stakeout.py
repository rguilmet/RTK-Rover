#!/usr/bin/env python3
"""
Test Harness for Phase 4 Line Stakeout Geodesic Math & Formatting.
Verifies Station, Cross-Track Offset (L/R), Quadrant Bearings, and Stationing.
"""

import math
import sys

WGS84_A = 6378137.0
WGS84_E2 = 0.00669437999014
DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi
METERS_TO_FEET = 3.280839895

def wgs84_radii(lat_rad):
    sin_lat = math.sin(lat_rad)
    denom = math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    r_m = WGS84_A * (1.0 - WGS84_E2) / (denom ** 3)
    r_n = WGS84_A / denom
    return r_m, r_n

def compute_line_stakeout(tip_lat, tip_lon, lat_a, lon_a, lat_b, lon_b):
    lat_mid = (lat_a + lat_b) * 0.5 * DEG_TO_RAD
    r_m, r_n = wgs84_radii(lat_mid)

    dlat_ab = (lat_b - lat_a) * DEG_TO_RAD
    dlon_ab = (lon_b - lon_a) * DEG_TO_RAD
    dn_ab = dlat_ab * r_m
    de_ab = dlon_ab * r_n * math.cos(lat_mid)
    total_len = math.sqrt(dn_ab * dn_ab + de_ab * de_ab)

    if total_len < 0.001:
        return {"valid": False}

    u_n = dn_ab / total_len
    u_e = de_ab / total_len

    brg = math.atan2(de_ab, dn_ab) * RAD_TO_DEG
    if brg < 0.0:
        brg += 360.0

    dlat_ar = (tip_lat - lat_a) * DEG_TO_RAD
    dlon_ar = (tip_lon - lon_a) * DEG_TO_RAD
    dn_ar = dlat_ar * r_m
    de_ar = dlon_ar * r_n * math.cos(lat_mid)

    station = dn_ar * u_n + de_ar * u_e
    offset = de_ar * u_n - dn_ar * u_e
    remaining = total_len - station
    pct = (station / total_len) * 100.0

    return {
        "valid": True,
        "station_m": station,
        "offset_m": offset,
        "total_len_m": total_len,
        "remaining_m": remaining,
        "bearing_deg": brg,
        "pct_complete": pct
    }

def format_quadrant_bearing(brg_deg):
    brg_deg = brg_deg % 360.0
    if 0.0 <= brg_deg <= 90.0:
        ns, ew, angle = 'N', 'E', brg_deg
    elif brg_deg <= 180.0:
        ns, ew, angle = 'S', 'E', 180.0 - brg_deg
    elif brg_deg <= 270.0:
        ns, ew, angle = 'S', 'W', brg_deg - 180.0
    else:
        ns, ew, angle = 'N', 'W', 360.0 - brg_deg

    deg = int(angle)
    rem_min = (angle - deg) * 60.0
    minute = int(rem_min)
    sec = (rem_min - minute) * 60.0
    if sec >= 59.95:
        sec = 0.0
        minute += 1
        if minute >= 60:
            minute = 0
            deg += 1
    return f"{ns} {deg:02d}°{minute:02d}'{sec:04.1f}\" {ew}"

def main():
    print("=== test_line_stakeout.py ===")
    
    # Baseline: Due North line of 100 meters
    lat_a, lon_a = 43.0000000, -71.0000000
    r_m, r_n = wgs84_radii(lat_a * DEG_TO_RAD)
    dlat_100m = (100.0 / r_m) * RAD_TO_DEG
    lat_b, lon_b = lat_a + dlat_100m, lon_a

    # Test 1: Exactly at Point A
    res = compute_line_stakeout(lat_a, lon_a, lat_a, lon_a, lat_b, lon_b)
    assert abs(res["station_m"]) < 0.001, f"Station at A failed: {res['station_m']}"
    assert abs(res["offset_m"]) < 0.001, f"Offset at A failed: {res['offset_m']}"
    assert abs(res["total_len_m"] - 100.0) < 0.01, f"Total len failed: {res['total_len_m']}"
    print(f"[PASS] Point A origin check: Sta={res['station_m']:.3f}m, Off={res['offset_m']:.3f}m, Total={res['total_len_m']:.3f}m")

    # Test 2: Exactly halfway along line (Sta = 50m, Off = 0)
    lat_mid = lat_a + (dlat_100m * 0.5)
    res_mid = compute_line_stakeout(lat_mid, lon_a, lat_a, lon_a, lat_b, lon_b)
    assert abs(res_mid["station_m"] - 50.0) < 0.01
    assert abs(res_mid["offset_m"]) < 0.001
    assert abs(res_mid["pct_complete"] - 50.0) < 0.01
    print(f"[PASS] Midpoint check: Sta={res_mid['station_m']:.3f}m, Off={res_mid['offset_m']:.3f}m, Pct={res_mid['pct_complete']:.1f}%")

    # Test 3: 1 meter to the RIGHT (East) of line
    dlon_1m = (1.0 / (r_n * math.cos(lat_mid * DEG_TO_RAD))) * RAD_TO_DEG
    res_right = compute_line_stakeout(lat_mid, lon_a + dlon_1m, lat_a, lon_a, lat_b, lon_b)
    assert abs(res_right["station_m"] - 50.0) < 0.01
    assert abs(res_right["offset_m"] - 1.0) < 0.01, f"Offset right failed: {res_right['offset_m']}"
    print(f"[PASS] Right offset check: Off={res_right['offset_m']:+.3f}m (Positive = RIGHT)")

    # Test 4: 1.5 meters to the LEFT (West) of line
    dlon_1p5m = (1.5 / (r_n * math.cos(lat_mid * DEG_TO_RAD))) * RAD_TO_DEG
    res_left = compute_line_stakeout(lat_mid, lon_a - dlon_1p5m, lat_a, lon_a, lat_b, lon_b)
    assert abs(res_left["station_m"] - 50.0) < 0.01
    assert abs(res_left["offset_m"] - (-1.5)) < 0.01, f"Offset left failed: {res_left['offset_m']}"
    print(f"[PASS] Left offset check: Off={res_left['offset_m']:+.3f}m (Negative = LEFT)")

    # Test 5: Quadrant Bearings
    q_ne = format_quadrant_bearing(45.25)
    q_se = format_quadrant_bearing(135.5)
    q_sw = format_quadrant_bearing(225.75)
    q_nw = format_quadrant_bearing(315.1)
    assert q_ne.startswith("N 45°15'"), f"NE failed: {q_ne}"
    assert q_se.startswith("S 44°30'"), f"SE failed: {q_se}"
    assert q_sw.startswith("S 45°45'"), f"SW failed: {q_sw}"
    assert q_nw.startswith("N 44°54'"), f"NW failed: {q_nw}"
    print(f"[PASS] Quadrant bearings verified: {q_ne} | {q_se} | {q_sw} | {q_nw}")

    print("\nALL LINE STAKEOUT GEODESIC TESTS PASSED 100%!")

if __name__ == "__main__":
    main()
