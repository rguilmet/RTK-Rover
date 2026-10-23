#!/usr/bin/env python3
"""
RTK Rover - Waypoint CSV Validator & Boundary Distance/Bearing Calculator
Follows GEMINI.md independent SemVer Python scripting standard.
"""

import os
import sys
import csv
import math
import argparse

__filename__ = os.path.basename(__file__)
__version__ = "v1.0.1"

WGS84_A = 6378137.0
WGS84_E2 = 0.00669437999014


def compute_geodesic_segment(lat1_deg, lon1_deg, lat2_deg, lon2_deg):
    """Returns (distance_m, bearing_deg) between two WGS84 coordinates."""
    lat1 = math.radians(lat1_deg)
    lon1 = math.radians(lon1_deg)
    lat2 = math.radians(lat2_deg)
    lon2 = math.radians(lon2_deg)

    mid_lat = 0.5 * (lat1 + lat2)
    sin_lat = math.sin(mid_lat)
    denom = math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    r_m = WGS84_A * (1.0 - WGS84_E2) / (denom ** 3)
    r_n = WGS84_A / denom

    d_north = (lat2 - lat1) * r_m
    d_east = (lon2 - lon1) * r_n * math.cos(mid_lat)
    dist_m = math.hypot(d_north, d_east)
    brg_deg = (math.degrees(math.atan2(d_east, d_north)) + 360.0) % 360.0
    return dist_m, brg_deg


def main():
    print(f"=== {__filename__} {__version__} ===")

    parser = argparse.ArgumentParser(
        description="Validate RTK Rover waypoint CSV files and print boundary legs."
    )
    parser.add_argument("input", help="Path to waypoint CSV file")
    args = parser.parse_args()

    clean_path = os.path.normpath(args.input.strip("'\""))
    if not os.path.exists(clean_path):
        print(f"[ERROR] File not found: {clean_path}")
        sys.exit(1)

    waypoints = []
    with open(clean_path, "r", encoding="utf-8-sig") as f:
        reader = csv.reader(f)
        for row_idx, row in enumerate(reader):
            if not row or row[0].strip().startswith("#"):
                continue
            if row_idx == 0 and any(k in row[0].lower() for k in ("label", "name", "id")):
                continue
            if len(row) < 3:
                print(f"[WARN] Skipping malformed row {row_idx + 1}: {row}")
                continue
            label = row[0].strip()
            lat = float(row[1].strip())
            lon = float(row[2].strip())
            elev = float(row[3].strip()) if len(row) > 3 and row[3].strip() else 0.0
            notes = row[4].strip() if len(row) > 4 else ""
            waypoints.append((label, lat, lon, elev, notes))

    print(f"Loaded {len(waypoints)} waypoints from {clean_path}:\n")
    print(f"{'#':<4} {'Label':<16} {'Latitude':<14} {'Longitude':<14} {'Elev(m)':<9} {'Next Leg':<18} {'Notes'}")
    print("-" * 95)

    for i, (label, lat, lon, elev, notes) in enumerate(waypoints):
        if len(waypoints) > 1:
            nxt = waypoints[(i + 1) % len(waypoints)]
            dist_m, brg_deg = compute_geodesic_segment(lat, lon, nxt[1], nxt[2])
            dist_ft = dist_m * 3.280839895
            leg_str = f"{dist_m:.2f}m ({dist_ft:.1f}ft) @{brg_deg:05.1f}deg"
        else:
            leg_str = "--"
        print(f"{i+1:<4} {label:<16} {lat:<14.8f} {lon:<14.8f} {elev:<9.3f} {leg_str:<18} {notes}")


if __name__ == "__main__":
    main()
