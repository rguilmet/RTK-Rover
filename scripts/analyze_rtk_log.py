#!/usr/bin/env python3
"""
RTK Rover - Survey Log Analyzer & Target Offset Reporter
Follows GEMINI.md independent SemVer Python scripting standard.
"""

import os
import sys
import csv
import argparse

__filename__ = os.path.basename(__file__)
__version__ = "v1.0.0"


def main():
    print(f"=== {__filename__} {__version__} ===")

    parser = argparse.ArgumentParser(
        description="Inspect RTK Rover logged survey CSV files (/survey_logged_pins.csv)."
    )
    parser.add_argument("input", help="Path to survey_logged_pins.csv")
    args = parser.parse_args()

    clean_path = os.path.normpath(args.input.strip("'\""))
    if not os.path.exists(clean_path):
        print(f"[ERROR] File not found: {clean_path}")
        sys.exit(1)

    with open(clean_path, "r", encoding="utf-8-sig") as f:
        reader = csv.DictReader(f)
        rows = list(reader)

    print(f"Total Logged Survey Points: {len(rows)}\n")
    print(f"{'Label':<18} {'Latitude':<14} {'Longitude':<14} {'Elev(m)':<9} {'Fix':<10} {'Acc(m)':<8} {'N':<5} {'Notes'}")
    print("-" * 105)
    for r in rows:
        print(
            f"{r.get('label',''):<18} "
            f"{r.get('latitude',''):<14} "
            f"{r.get('longitude',''):<14} "
            f"{r.get('elevation',''):<9} "
            f"{r.get('fix_type',''):<10} "
            f"{r.get('h_acc_m',''):<8} "
            f"{r.get('samples',''):<5} "
            f"{r.get('notes','')}"
        )


if __name__ == "__main__":
    main()
