#!/usr/bin/env python3
"""
kml_to_waypoints.py - Ingests a Google Earth / GIS KML file and converts
placemarks into the standardized 19-column RTK Rover waypoints CSV format.

Author: Antigravity
Version: 1.0.0
"""

import os
import sys
import xml.etree.ElementTree as ET


def parse_kml_to_waypoints(kml_path):
    if not os.path.isfile(kml_path):
        raise FileNotFoundError(f"KML file not found: {kml_path}")

    tree = ET.parse(kml_path)
    root = tree.getroot()

    # Support both default namespaces and prefix
    ns = {"kml": "http://www.opengis.net/kml/2.2"}
    # If root tag does not match namespace, fallback to tag stripping
    has_ns = root.tag.startswith("{http://www.opengis.net/kml/2.2}")

    placemarks = []
    if has_ns:
        pms = root.findall(".//kml:Placemark", ns)
    else:
        pms = root.findall(".//Placemark")

    count = 0
    waypoints = []

    for pm in pms:
        name_node = pm.find("kml:name" if has_ns else "name", ns) if has_ns else pm.find("name")
        name = name_node.text.strip() if name_node is not None and name_node.text else f"KML_POINT_{count+1:02d}"

        # Clean label for CSV (replace commas or invalid characters)
        label = name.replace(",", "_").replace(" ", "_").upper()

        desc_node = pm.find("kml:description" if has_ns else "description", ns) if has_ns else pm.find("description")
        desc = desc_node.text.strip().replace("\r", " ").replace("\n", " ").replace(",", ";") if desc_node is not None and desc_node.text else f"Imported from {os.path.basename(kml_path)}"

        # Find Point coordinates
        point_node = pm.find(".//kml:Point/kml:coordinates" if has_ns else ".//Point/coordinates", ns) if has_ns else pm.find(".//Point/coordinates")
        if point_node is not None and point_node.text:
            coords = point_node.text.strip().split(",")
            if len(coords) >= 2:
                try:
                    lon = float(coords[0])
                    lat = float(coords[1])
                    elev = float(coords[2]) if len(coords) >= 3 else 0.0
                    count += 1
                    waypoints.append({
                        "label": label,
                        "lat": lat,
                        "lon": lon,
                        "elev": elev,
                        "notes": desc
                    })
                except ValueError:
                    continue
        else:
            # Check for LineString or Polygon coordinates (take first point or all points)
            coord_nodes = pm.findall(".//kml:coordinates" if has_ns else ".//coordinates", ns) if has_ns else pm.findall(".//coordinates")
            for c_node in coord_nodes:
                if c_node.text:
                    lines = c_node.text.strip().split()
                    for idx, line in enumerate(lines):
                        parts = line.strip().split(",")
                        if len(parts) >= 2:
                            try:
                                lon = float(parts[0])
                                lat = float(parts[1])
                                elev = float(parts[2]) if len(parts) >= 3 else 0.0
                                pt_label = f"{label}_{idx+1:02d}" if len(lines) > 1 else label
                                count += 1
                                waypoints.append({
                                    "label": pt_label,
                                    "lat": lat,
                                    "lon": lon,
                                    "elev": elev,
                                    "notes": desc
                                })
                            except ValueError:
                                continue

    return waypoints


def write_waypoints_csv(waypoints, csv_path):
    with open(csv_path, "w", encoding="utf-8") as f:
        f.write("timestamp,firmware_ver,label,latitude,longitude,elevation,pos_fix,hdg_fix,h_acc_m,v_acc_m,hdop,sats,slave_sats,diff_age_s,base_id,true_heading_deg,pole_tilt_deg,samples,notes\n")
        for wp in waypoints:
            elev_str = f"{wp['elev']:.3f}" if wp['elev'] != 0.0 else ""
            f.write(f",,{wp['label']},{wp['lat']:.9f},{wp['lon']:.9f},{elev_str},KML_DESIGN,NONE,0.000,0.000,0.0,0,0,0.0,0,0.00,0.00,1,{wp['notes']}\n")


def main():
    if len(sys.argv) < 2:
        print("Usage: python kml_to_waypoints.py <input.kml> [output_waypoints.csv]")
        sys.exit(1)

    kml_file = sys.argv[1]
    out_csv = sys.argv[2] if len(sys.argv) >= 3 else "kml_waypoints.csv"

    print(f"=== kml_to_waypoints.py v1.0.0 ===")
    print(f"Parsing KML: {kml_file}")
    pts = parse_kml_to_waypoints(kml_file)
    print(f"Extracted {len(pts)} coordinate points.")

    write_waypoints_csv(pts, out_csv)
    print(f"[SUCCESS] Wrote {len(pts)} waypoints to {out_csv}")


if __name__ == "__main__":
    main()
