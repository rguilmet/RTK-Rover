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
__version__ = "v1.2.0"

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
        col_label, col_lat, col_lon, col_elev, col_notes = 0, 1, 2, 3, 4
        col_fix = -1
        has_header = False

        for row_idx, row in enumerate(reader):
            if not row or not any(c.strip() for c in row):
                continue
            first_non_empty = next((c.strip() for c in row if c.strip()), "")
            if first_non_empty.startswith("#"):
                continue

            # Header detection
            if row_idx == 0:
                row_lower = [c.strip().lower() for c in row]
                if any(k in row_lower[0] for k in ("timestamp", "label", "name", "id")) or any("lat" in c for c in row_lower):
                    has_header = True
                    for c_idx, h in enumerate(row_lower):
                        if "label" in h or "name" in h: col_label = c_idx
                        elif "lat" in h: col_lat = c_idx
                        elif "lon" in h: col_lon = c_idx
                        elif "elev" in h or "alt" in h or "height" in h: col_elev = c_idx
                        elif "fix" in h: col_fix = c_idx
                        elif "note" in h or "desc" in h: col_notes = c_idx
                    continue

            if len(row) <= max(col_label, col_lat, col_lon):
                print(f"[WARN] Skipping malformed row {row_idx + 1}: {row}")
                continue

            label = row[col_label].strip()
            lat_str = row[col_lat].strip()
            lon_str = row[col_lon].strip()
            if not label or not lat_str or not lon_str:
                continue

            try:
                lat = float(lat_str)
                lon = float(lon_str)
                elev = float(row[col_elev].strip()) if col_elev >= 0 and col_elev < len(row) and row[col_elev].strip() else 0.0
                fix = row[col_fix].strip() if col_fix >= 0 and col_fix < len(row) else ""
                notes = row[col_notes].strip() if col_notes >= 0 and col_notes < len(row) else ""
                waypoints.append((label, lat, lon, elev, fix, notes))
            except ValueError as e:
                print(f"[WARN] Skipping row {row_idx + 1} due to parse error: {e}")
                continue

    print(f"Loaded {len(waypoints)} waypoints from {clean_path}:\n")
    print(f"{'#':<4} {'Label':<24} {'Latitude':<14} {'Longitude':<14} {'Elev(m)':<9} {'Fix':<11} {'Next Leg':<20} {'Notes'}")
    print("-" * 115)

    total_perimeter_m = 0.0
    for i, (label, lat, lon, elev, fix, notes) in enumerate(waypoints):
        leg_str = "--"
        if len(waypoints) > 1:
            nxt = waypoints[(i + 1) % len(waypoints)]
            dist_m, brg_deg = compute_geodesic_segment(lat, lon, nxt[1], nxt[2])
            dist_ft = dist_m * 3.280839895
            total_perimeter_m += dist_m
            leg_str = f"{dist_m:.2f}m ({dist_ft:.1f}ft) @{brg_deg:.1f}d"

        print(f"{i+1:<4} {label:<24} {lat:<14.8f} {lon:<14.8f} {elev:<9.3f} {fix:<11} {leg_str:<20} {notes}")

    if len(waypoints) > 2:
        print("-" * 115)
        print(f"Total Closed Loop Perimeter: {total_perimeter_m:.2f} m ({total_perimeter_m * 3.280839895:.2f} ft)")


if __name__ == "__main__":
    main()
