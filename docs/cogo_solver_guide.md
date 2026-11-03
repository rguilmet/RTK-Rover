# Cadastral COGO & Traverse Solver Guide

The **Cadastral COGO Solver** (`scripts/cogo_solver.py`) is an open-source, schema-driven computational engine designed for the RTK Rover ecosystem. It translates recorded metes-and-bounds deed descriptions, circular road curve geometry, and high-precision RTK GNSS ground truth observations into unified geodetic survey coordinates (WGS84).

Adhering to state-of-the-art software engineering principles, the solver is completely decoupled from domain data: all algorithms are generic and parameterized by standard JSON schemas and CSV files.

---

## 1. Core Architectural Capabilities

* **Geodesic WGS84 Projections:** High-precision direct and inverse ellipsoidal calculations ($a = 6,378,137.0\text{ m}$, $f = 1/298.257223563$).
* **Flexible Bearing Parsing:** Accepts quadrant bearings (`N 45-00-00 E`, `S 55-47-13 W`, `N 14.5 E`) and decimal azimuth degrees (`[0, 360)`).
* **Baseline Convergence Calibration:** Derives grid-to-true rotation offset automatically from a two-anchor ground truth baseline.
* **Traverse Misclosure Analysis:** Evaluates closed traverses, reporting linear error, misclosure azimuth, and precision ratio ($1 : N$).
* **Compass Rule (Bowditch Adjustment):** Dynamically distributes linear misclosure across traverse legs proportional to call lengths.
* **Circular Road Curve Geometry:** Solves Points of Curvature (PC), Points of Tangency (PT), radius centers, chord azimuths, and arc midpoints for road right-of-ways and cul-de-sacs.
* **Adjoiner Common-Line Snapping:** Reconciles shared boundary corners across adjacent parcels to eliminate rounding gaps or overlaps.
* **Multi-Format GIS Exporters:** Generates Rover 19-column `waypoints.csv`, Google Earth KML, RFC 7946 GeoJSON, and ASCII closure reports.

---

## 2. Cadastral Schema Reference (`cadastral_schema.json`)

Deed networks are defined in standard JSON conforming to [`docs/schemas/cadastral_schema.json`](schemas/cadastral_schema.json).

### Minimal Schema Structure
```json
{
  "project": {
    "name": "Subdivision Boundary",
    "datum": "WGS84",
    "units": "feet"
  },
  "control_anchors": [
    {
      "id": "PIN_101",
      "label": "PIN_101_REF",
      "latitude": 45.123456789,
      "longitude": -69.123456789,
      "elevation_m": 125.40,
      "monument": "5/8in Rebar w/ Yellow Cap 'PLS 2081'",
      "notes": "Found corner pin"
    }
  ],
  "calibration": {
    "anchor_from": "PIN_101",
    "anchor_to": "PIN_102",
    "deed_bearing": "N 45-00-00 W"
  },
  "parcels": [
    {
      "id": "lot_1",
      "name": "Lot 1",
      "adjust": "compass",
      "pob": {
        "anchor_id": "PIN_101",
        "point_id": "LOT1_POB"
      },
      "calls": [
        {
          "type": "line",
          "bearing": "N 45-00-00 E",
          "distance": 250.00,
          "to_point_id": "LOT1_PIN_NE",
          "monument": "5/8in Rebar Set"
        }
      ]
    }
  ]
}
```

### Call Types & Parameters

#### 1. Straight Line Calls (`"type": "line"`)
* `bearing`: Quadrant bearing (`"N 45-00-00 E"`) or decimal degrees (`45.0`).
* `distance`: Length in project units (`feet`, `us_survey_feet`, `meters`).
* `to_point_id`: Unique identifier assigned to the endpoint.
* `stations` *(optional)*: Intermediate monuments or fence posts along the leg (`"station": 150.0`).

#### 2. Circular Curve Calls (`"type": "curve"`)
* `radius`: Radius of circular curve in project units.
* `delta_deg` or `arc_length`: Central angle in degrees or arc distance along curve.
* `direction`: `"left"` (`"ccw"`) or `"right"` (`"cw"`).
* `tangent_in`: `true` if entering curve tangentially from previous leg.
* `center_point_id` *(optional)*: Identifier to export the radius center of curvature.
* `mid_point_id` *(optional)*: Identifier to export the arc midpoint.

---

## 3. CLI Command-Line Interface

```text
usage: cogo_solver.py [-h] --input INPUT [--anchors ANCHORS]
                      [--rover-csv ROVER_CSV] [--kml KML]
                      [--geojson GEOJSON] [--dxf DXF]
                      [--report REPORT]
                      [--adjust {none,compass,bowditch}]
                      [--rotation ROTATION]
```

### Argument Reference
| Flag | Description |
| :--- | :--- |
| `--input`, `-i` | **Required.** Path to input cadastral JSON file. |
| `--anchors`, `-a` | Optional path to external control anchors CSV file. |
| `--rover-csv` | Destination path for Rover 19-column `/waypoints.csv`. |
| `--kml` | Destination path for Google Earth KML representation. |
| `--geojson` | Destination path for RFC 7946 GeoJSON FeatureCollection. |
| `--dxf` | Destination path for AutoCAD DXF CAD linework with layers. |
| `--report` | Destination path for detailed ASCII traverse report (also printed to console). |
| `--adjust` | Override balance adjustment rule (`none`, `compass`, `bowditch`). |
| `--rotation` | Override grid-to-true rotation offset in degrees. |

---

## 4. Quickstart Demonstration

Run the solver against the included fictional 3-lot reference dataset:

```bash
python scripts/cogo_solver.py \
  --input examples/subdivision_demo.json \
  --anchors examples/control_points.sample.csv \
  --rover-csv output/waypoints.csv \
  --kml output/subdivision.kml \
  --geojson output/subdivision.geojson \
  --dxf output/subdivision.dxf \
  --report output/closure_report.txt
```

### Output Formats

1. **Rover Waypoints CSV (`output/waypoints.csv`):**
   ```csv
   timestamp,firmware_ver,label,latitude,longitude,elevation,pos_fix,hdg_fix,h_acc_m,v_acc_m,hdop,sats,slave_sats,diff_age_s,base_id,true_heading_deg,pole_tilt_deg,samples,notes
   ,,PIN_LOT1_POB,45.123456789,-69.123456789,125.400,RTK_FIX,HDG_FIX,0.035,0.032,0.6,26,0,0.0,0,0.00,0.00,1,5/8in Rebar Set w/ Plastic Cap 'PLS 9999'
   ,,PIN_LOT1_NE,45.123992746,-69.122853299,,ADJ_CALC,NONE,0.000,0.000,0.0,0,0,0.0,0,0.00,0.00,1,5/8in Rebar Set w/ Plastic Cap 'PLS 9999'
   ```
2. **Google Earth KML (`output/subdivision.kml`):**
   Interactive visualization with translucent parcel polygons, line boundaries, and color-coded pushpins for pins and curvature centers.
3. **GeoJSON (`output/subdivision.geojson`):**
   Standard spatial features compatible with QGIS, ArcGIS, Mapbox, and web GIS portals.
4. **AutoCAD DXF (`output/subdivision.dxf`):**
   CAD vector geometry in local ground survey coordinates with dedicated layers for control anchors, parcel boundaries, curve arcs, monument symbols, waypoint labels, and bearing/distance call annotations.
5. **ASCII Survey Report (`output/closure_report.txt`):**
   Professional leg-by-leg closure metrics, misclosure vectors, and precision ratios.
