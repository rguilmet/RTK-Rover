#!/usr/bin/env python3
"""
cogo_solver.py - Generic Schema-Driven Cadastral COGO & Traverse Engine

Solves metes-and-bounds deed traverses, circular road curve geometry, and
ground-truth RTK GNSS control ties from standard JSON schemas and CSV files.

Features:
- Decimal degrees, quadrant bearings (e.g. 'N 35-38-56 E'), and azimuth conversion
- Ground baseline calibration and true meridian grid convergence offset
- Sequential traverse computation with intermediate stationing
- Closed loop linear misclosure and precision ratio (1 : N) calculation
- Compass Rule (Bowditch) traverse balance adjustment
- Circular curve geometry (PC, PT, center of curvature, arc midpoint, chord)
- Adjoiner common boundary reconciliation
- Multi-format exports: Rover 19-column waypoints.csv, Google Earth KML, GeoJSON, text reports
"""

import os
import sys
import math
import json
import csv
import argparse
import re
from typing import Dict, List, Tuple, Any, Optional

__filename__ = os.path.basename(__file__)
__version__ = "v1.0.1"

# WGS84 Ellipsoidal Constants
WGS84_A = 6378137.0  # Semi-major axis in meters
WGS84_F = 1.0 / 298.257223563  # Flattening
WGS84_B = WGS84_A * (1.0 - WGS84_F)  # Semi-minor axis in meters
WGS84_E2 = (WGS84_A**2 - WGS84_B**2) / (WGS84_A**2)  # First eccentricity squared

# Linear Unit Conversion Multipliers to Meters
UNIT_TO_METERS = {
    "feet": 0.3048,
    "foot": 0.3048,
    "ft": 0.3048,
    "us_survey_feet": 1200.0 / 3937.0,  # ~0.3048006096 m
    "us_survey_foot": 1200.0 / 3937.0,
    "meter": 1.0,
    "meters": 1.0,
    "m": 1.0,
}


# =============================================================================
# GEODESIC & COORDINATE GEOMETRY (COGO) ALGORITHMS
# =============================================================================

def get_unit_multiplier(units_str: str) -> float:
    """Return conversion multiplier to meters for given unit name."""
    clean = units_str.strip().lower().replace(" ", "_")
    return UNIT_TO_METERS.get(clean, 0.3048)


def parse_quadrant_bearing(bearing_str: str) -> float:
    """
    Parse a quadrant bearing string or decimal azimuth into azimuth degrees [0, 360).
    Supported formats:
      - 'N 45-00-00 E', 'N 45 deg 00 min 00 sec E', 'N 45d 00m 00s E', 'N 45.5 E', 'N45E'
      - 'S 55-47-13 W'
      - Pure decimal azimuth: '304.213', '124.5'
    """
    if isinstance(bearing_str, (int, float)):
        return float(bearing_str) % 360.0

    s = bearing_str.strip().upper()

    # Pattern for Quadrant Bearing: [N|S] <deg> [min] [sec] [E|W]
    m_quad = re.match(
        r"^([NS])\s*(\d+(?:\.\d+)?)(?:[^\d]+(\d+(?:\.\d+)?))?(?:[^\d]+(\d+(?:\.\d+)?))?\s*([EW])$",
        s
    )
    if m_quad:
        ns = m_quad.group(1)
        deg = float(m_quad.group(2))
        minute = float(m_quad.group(3)) if m_quad.group(3) else 0.0
        sec = float(m_quad.group(4)) if m_quad.group(4) else 0.0
        ew = m_quad.group(5)

        val = deg + (minute / 60.0) + (sec / 3600.0)
        if ns == "N" and ew == "E":
            azimuth = val
        elif ns == "S" and ew == "E":
            azimuth = 180.0 - val
        elif ns == "S" and ew == "W":
            azimuth = 180.0 + val
        elif ns == "N" and ew == "W":
            azimuth = 360.0 - val
        else:
            azimuth = val
        return azimuth % 360.0

    # Pattern for simple azimuth decimal number
    try:
        return float(s) % 360.0
    except ValueError:
        raise ValueError(f"Unable to parse bearing string: '{bearing_str}'")


def format_quadrant_bearing(azimuth_deg: float) -> str:
    """Format decimal azimuth [0, 360) into standard quadrant bearing (e.g. 'N 45-00-00 E')."""
    az = azimuth_deg % 360.0
    if 0.0 <= az <= 90.0:
        ns, ew, val = "N", "E", az
    elif 90.0 < az <= 180.0:
        ns, ew, val = "S", "E", 180.0 - az
    elif 180.0 < az <= 270.0:
        ns, ew, val = "S", "W", az - 180.0
    else:
        ns, ew, val = "N", "W", 360.0 - az

    deg = int(val)
    rem_min = (val - deg) * 60.0
    minute = int(rem_min)
    sec = round((rem_min - minute) * 60.0)
    if sec >= 60:
        sec = 0
        minute += 1
    if minute >= 60:
        minute = 0
        deg += 1

    return f"{ns} {deg:02d}-{minute:02d}-{sec:02d} {ew}"


def compute_geodesic_segment(lat1: float, lon1: float, lat2: float, lon2: float) -> Tuple[float, float]:
    """
    Compute geodesic distance (meters) and initial forward azimuth (degrees)
    between two WGS84 positions.
    """
    phi1, lam1 = math.radians(lat1), math.radians(lon1)
    phi2, lam2 = math.radians(lat2), math.radians(lon2)
    dphi = phi2 - phi1
    dlam = lam2 - lam1

    a = math.sin(dphi / 2.0)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2.0)**2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(max(0.0, 1.0 - a)))
    dist_m = WGS84_A * c

    y = math.sin(dlam) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
    bearing_deg = (math.degrees(math.atan2(y, x)) + 360.0) % 360.0

    return dist_m, bearing_deg


def project_point(start_lat: float, start_lon: float, dist_m: float, bearing_deg: float) -> Tuple[float, float]:
    """
    Project point from (start_lat, start_lon) along geodesic vector (dist_m, bearing_deg).
    Guarantees mathematical parity with survey baseline solvers.
    """
    lat1 = math.radians(start_lat)
    lon1 = math.radians(start_lon)
    brg = math.radians(bearing_deg)
    d_div_r = dist_m / WGS84_A

    new_lat = math.asin(
        math.sin(lat1) * math.cos(d_div_r) +
        math.cos(lat1) * math.sin(d_div_r) * math.cos(brg)
    )
    new_lon = lon1 + math.atan2(
        math.sin(brg) * math.sin(d_div_r) * math.cos(lat1),
        math.cos(d_div_r) - math.sin(lat1) * math.sin(new_lat)
    )
    return math.degrees(new_lat), math.degrees(new_lon)


# =============================================================================
# DATA STRUCTURES
# =============================================================================

class SurveyPoint:
    """Represents a computed or surveyed parcel monument/waypoint."""
    def __init__(self, point_id: str, label: str, lat: float, lon: float,
                 elevation_m: Optional[float] = None, monument: str = "",
                 notes: str = "", point_type: str = "DEED_CALC",
                 parcel_id: str = "", station_along_call_m: float = 0.0):
        self.point_id = point_id
        self.label = label if label else point_id
        self.lat = lat
        self.lon = lon
        self.elevation_m = elevation_m
        self.monument = monument
        self.notes = notes
        self.point_type = point_type  # 'RTK_FIX', 'DEED_CALC', 'ADJ_CALC', 'CURVE_PT', etc.
        self.parcel_id = parcel_id
        self.station_along_call_m = station_along_call_m


class LegResult:
    """Represents a solved metes-and-bounds traverse call."""
    def __init__(self, call_index: int, call_type: str, raw_bearing: str,
                 applied_bearing_deg: float, distance_m: float, distance_user: float,
                 from_point_id: str, to_point_id: str, monument: str, notes: str):
        self.call_index = call_index
        self.call_type = call_type
        self.raw_bearing = raw_bearing
        self.applied_bearing_deg = applied_bearing_deg
        self.distance_m = distance_m
        self.distance_user = distance_user
        self.from_point_id = from_point_id
        self.to_point_id = to_point_id
        self.monument = monument
        self.notes = notes


class ParcelResult:
    """Results and metrics for a solved parcel."""
    def __init__(self, parcel_id: str, name: str, book_page: str = ""):
        self.parcel_id = parcel_id
        self.name = name
        self.book_page = book_page
        self.points: List[SurveyPoint] = []
        self.legs: List[LegResult] = []
        self.is_closed: bool = False
        self.closure_error_m: float = 0.0
        self.closure_azimuth_deg: float = 0.0
        self.perimeter_m: float = 0.0
        self.precision_ratio: int = 0
        self.adjustment_applied: str = "none"


# =============================================================================
# COGO TRAVERSE SOLVER ENGINE
# =============================================================================

class CadastralSolver:
    def __init__(self, schema_data: Dict[str, Any], external_anchors: Optional[List[Dict[str, Any]]] = None,
                 manual_rotation_deg: Optional[float] = None, force_adjust: Optional[str] = None):
        self.data = schema_data
        self.project = schema_data.get("project", {})
        self.units_name = self.project.get("units", "feet")
        self.unit_to_m = get_unit_multiplier(self.units_name)
        self.m_to_unit = 1.0 / self.unit_to_m

        self.anchors: Dict[str, SurveyPoint] = {}
        self.solved_points: Dict[str, SurveyPoint] = {}
        self.parcels_results: List[ParcelResult] = []
        self.manual_rotation = manual_rotation_deg
        self.force_adjust = force_adjust

        self._load_anchors(external_anchors)
        self.grid_rotation_deg = self._derive_grid_rotation()

    def _load_anchors(self, external_anchors: Optional[List[Dict[str, Any]]] = None):
        """Load control anchors from JSON and external CSV sources."""
        # 1. Load from schema JSON
        anchors_list = list(self.data.get("control_anchors", []))

        # 2. Add external CSV anchors if provided
        if external_anchors:
            anchors_list.extend(external_anchors)

        for a in anchors_list:
            aid = a.get("id") or a.get("point_id")
            if not aid:
                continue
            lat = float(a["latitude"])
            lon = float(a["longitude"])
            elev = float(a["elevation_m"]) if a.get("elevation_m") is not None else None
            monument = a.get("monument", "")
            notes = a.get("notes", "")
            label = a.get("label", aid)

            pt = SurveyPoint(
                point_id=aid,
                label=label,
                lat=lat,
                lon=lon,
                elevation_m=elev,
                monument=monument,
                notes=notes,
                point_type="RTK_FIX"
            )
            self.anchors[aid] = pt
            self.solved_points[aid] = pt

    def _derive_grid_rotation(self) -> float:
        """Derive grid rotation offset from baseline calibration or explicit parameter."""
        if self.manual_rotation is not None:
            return float(self.manual_rotation)

        calib = self.data.get("calibration")
        if calib:
            af_id = calib.get("anchor_from")
            at_id = calib.get("anchor_to")
            deed_bearing_str = calib.get("deed_bearing")

            if af_id in self.anchors and at_id in self.anchors and deed_bearing_str:
                p_from = self.anchors[af_id]
                p_to = self.anchors[at_id]
                dist_m, ground_brg = compute_geodesic_segment(p_from.lat, p_from.lon, p_to.lat, p_to.lon)
                deed_brg = parse_quadrant_bearing(deed_bearing_str)
                rot_offset = ground_brg - deed_brg
                return rot_offset

        # Fallback to project setting or 0.0
        return float(self.project.get("grid_rotation_deg", 0.0))

    def _resolve_point(self, ref_str: str) -> Optional[SurveyPoint]:
        """Resolve a point by ID or scoped ID (e.g. 'lot_1.LOT1_PIN_NE')."""
        if ref_str in self.solved_points:
            return self.solved_points[ref_str]

        # Check by bare point ID if scoped
        if "." in ref_str:
            _, bare = ref_str.split(".", 1)
            if bare in self.solved_points:
                return self.solved_points[bare]

        return None

    def solve_all(self):
        """Solve all parcels sequentially."""
        parcels = self.data.get("parcels", [])
        for p_def in parcels:
            p_res = self._solve_parcel(p_def)
            self.parcels_results.append(p_res)

    def _solve_parcel(self, p_def: Dict[str, Any]) -> ParcelResult:
        parcel_id = p_def["id"]
        parcel_name = p_def.get("name", parcel_id)
        book_page = p_def.get("book_page", "")
        res = ParcelResult(parcel_id, parcel_name, book_page)

        adjust_rule = self.force_adjust if self.force_adjust else p_def.get("adjust", "none").lower()

        # 1. Resolve Point of Beginning (POB)
        pob_def = p_def.get("pob", {})
        pob_pt = self._resolve_pob(parcel_id, pob_def)
        res.points.append(pob_pt)
        self.solved_points[f"{parcel_id}.{pob_pt.point_id}"] = pob_pt
        self.solved_points[pob_pt.point_id] = pob_pt

        curr_lat, curr_lon = pob_pt.lat, pob_pt.lon
        curr_pt_id = pob_pt.point_id
        prev_tangent_brg: Optional[float] = None

        total_perimeter_m = 0.0
        legs_data = []

        # 2. Traverse sequential calls
        calls = p_def.get("calls", [])
        for idx, call in enumerate(calls, 1):
            call_type = call.get("type", "line").lower()
            to_pid = call["to_point_id"]
            label = call.get("label", to_pid)
            monument = call.get("monument", "")
            notes = call.get("notes", "")

            if call_type == "line":
                raw_bearing = call["bearing"]
                dist_user = float(call["distance"])
                dist_m = dist_user * self.unit_to_m
                total_perimeter_m += dist_m

                azimuth_deg = parse_quadrant_bearing(raw_bearing)
                eff_bearing = (azimuth_deg + self.grid_rotation_deg) % 360.0
                prev_tangent_brg = eff_bearing

                # Solve intermediate stations if specified
                for st in call.get("stations", []):
                    st_dist_user = float(st["station"])
                    st_dist_m = st_dist_user * self.unit_to_m
                    st_pid = st["point_id"]
                    st_label = st.get("label", st_pid)
                    st_lat, st_lon = project_point(curr_lat, curr_lon, st_dist_m, eff_bearing)
                    st_pt = SurveyPoint(
                        point_id=st_pid,
                        label=st_label,
                        lat=st_lat,
                        lon=st_lon,
                        monument=st.get("monument", ""),
                        notes=st.get("notes", f"Station {st_dist_user:.2f} {self.units_name}"),
                        point_type="DEED_STATION",
                        parcel_id=parcel_id,
                        station_along_call_m=st_dist_m
                    )
                    res.points.append(st_pt)
                    self.solved_points[f"{parcel_id}.{st_pid}"] = st_pt
                    self.solved_points[st_pid] = st_pt

                # Project endpoint
                next_lat, next_lon = project_point(curr_lat, curr_lon, dist_m, eff_bearing)

                leg_obj = LegResult(
                    call_index=idx,
                    call_type="line",
                    raw_bearing=raw_bearing,
                    applied_bearing_deg=eff_bearing,
                    distance_m=dist_m,
                    distance_user=dist_user,
                    from_point_id=curr_pt_id,
                    to_point_id=to_pid,
                    monument=monument,
                    notes=notes
                )
                res.legs.append(leg_obj)

            elif call_type == "curve":
                radius_user = float(call["radius"])
                radius_m = radius_user * self.unit_to_m
                direction = call.get("direction", "right").lower()  # 'left' (ccw) or 'right' (cw)

                delta_deg = float(call.get("delta_deg", 0.0))
                arc_len_user = float(call.get("arc_length", 0.0))
                if delta_deg > 0.0:
                    delta_rad = math.radians(delta_deg)
                    arc_len_m = radius_m * delta_rad
                    arc_len_user = arc_len_m * self.m_to_unit
                elif arc_len_user > 0.0:
                    arc_len_m = arc_len_user * self.unit_to_m
                    delta_rad = arc_len_m / radius_m
                    delta_deg = math.degrees(delta_rad)
                else:
                    raise ValueError(f"Curve in parcel '{parcel_id}' requires 'delta_deg' or 'arc_length'.")

                total_perimeter_m += arc_len_m

                # Tangent entry bearing
                tangent_in = call.get("tangent_in", True)
                if tangent_in and prev_tangent_brg is not None:
                    t_in_brg = prev_tangent_brg
                elif "bearing_in" in call:
                    t_in_brg = (parse_quadrant_bearing(call["bearing_in"]) + self.grid_rotation_deg) % 360.0
                else:
                    t_in_brg = 0.0

                # Bearing to Center of Curvature
                if direction in ["left", "ccw"]:
                    center_brg = (t_in_brg - 90.0) % 360.0
                else:
                    center_brg = (t_in_brg + 90.0) % 360.0

                center_lat, center_lon = project_point(curr_lat, curr_lon, radius_m, center_brg)

                # Export Center of Curvature if requested
                center_pid = call.get("center_point_id")
                if center_pid:
                    c_pt = SurveyPoint(
                        point_id=center_pid,
                        label=center_pid,
                        lat=center_lat,
                        lon=center_lon,
                        monument="Center of Curvature",
                        notes=f"R={radius_user:.2f} {self.units_name}",
                        point_type="CURVE_CENTER",
                        parcel_id=parcel_id
                    )
                    res.points.append(c_pt)
                    self.solved_points[f"{parcel_id}.{center_pid}"] = c_pt
                    self.solved_points[center_pid] = c_pt

                # Radial vector from center to entry point
                rad_from_center_to_p1 = (center_brg + 180.0) % 360.0

                # Radial vector from center to exit point
                if direction in ["left", "ccw"]:
                    rad_from_center_to_p2 = (rad_from_center_to_p1 - delta_deg) % 360.0
                    rad_from_center_to_mid = (rad_from_center_to_p1 - (delta_deg / 2.0)) % 360.0
                    prev_tangent_brg = (rad_from_center_to_p2 - 90.0) % 360.0
                else:
                    rad_from_center_to_p2 = (rad_from_center_to_p1 + delta_deg) % 360.0
                    rad_from_center_to_mid = (rad_from_center_to_p1 + (delta_deg / 2.0)) % 360.0
                    prev_tangent_brg = (rad_from_center_to_p2 + 90.0) % 360.0

                # Export Arc Midpoint if requested
                mid_pid = call.get("mid_point_id")
                if mid_pid:
                    mid_lat, mid_lon = project_point(center_lat, center_lon, radius_m, rad_from_center_to_mid)
                    mid_pt = SurveyPoint(
                        point_id=mid_pid,
                        label=mid_pid,
                        lat=mid_lat,
                        lon=mid_lon,
                        monument="Road Arc Midpoint",
                        notes=f"R={radius_user:.2f} Arc={arc_len_user / 2.0:.2f} {self.units_name}",
                        point_type="CURVE_MID",
                        parcel_id=parcel_id
                    )
                    res.points.append(mid_pt)
                    self.solved_points[f"{parcel_id}.{mid_pid}"] = mid_pt
                    self.solved_points[mid_pid] = mid_pt

                # Solve exit point
                next_lat, next_lon = project_point(center_lat, center_lon, radius_m, rad_from_center_to_p2)

                chord_m, chord_brg = compute_geodesic_segment(curr_lat, curr_lon, next_lat, next_lon)
                chord_user = chord_m * self.m_to_unit

                leg_obj = LegResult(
                    call_index=idx,
                    call_type="curve",
                    raw_bearing=f"Curve {direction.upper()} R={radius_user:.2f} Delta={delta_deg:.2f} deg",
                    applied_bearing_deg=chord_brg,
                    distance_m=arc_len_m,
                    distance_user=arc_len_user,
                    from_point_id=curr_pt_id,
                    to_point_id=to_pid,
                    monument=monument,
                    notes=f"{notes} [Chord={chord_user:.2f}{self.units_name} @ {format_quadrant_bearing(chord_brg)}]"
                )
                res.legs.append(leg_obj)
            else:
                raise ValueError(f"Unknown call type '{call_type}' in parcel '{parcel_id}'.")

            # Store the computed point
            end_pt = SurveyPoint(
                point_id=to_pid,
                label=label,
                lat=next_lat,
                lon=next_lon,
                monument=monument,
                notes=notes,
                point_type="DEED_CALC",
                parcel_id=parcel_id
            )
            res.points.append(end_pt)
            self.solved_points[f"{parcel_id}.{to_pid}"] = end_pt
            self.solved_points[to_pid] = end_pt

            curr_lat, curr_lon = next_lat, next_lon
            curr_pt_id = to_pid

        # 3. Check for Closed Traverse
        pob_first = res.points[0]
        end_last = res.points[-1]
        is_nominally_closed = (
            end_last.point_id == pob_first.point_id or
            (len(calls) > 1 and calls[-1]["to_point_id"] == pob_first.point_id)
        )

        res.perimeter_m = total_perimeter_m
        if is_nominally_closed:
            res.is_closed = True
            err_m, err_azimuth = compute_geodesic_segment(end_last.lat, end_last.lon, pob_first.lat, pob_first.lon)
            res.closure_error_m = err_m
            res.closure_azimuth_deg = err_azimuth
            if err_m > 0.0001:
                res.precision_ratio = int(total_perimeter_m / err_m)
            else:
                res.precision_ratio = 9999999

            # Apply Compass Rule (Bowditch Adjustment) if configured
            if adjust_rule in ["compass", "bowditch"]:
                self._apply_compass_rule(res)
                res.adjustment_applied = "compass"

        return res

    def _resolve_pob(self, parcel_id: str, pob_def: Dict[str, Any]) -> SurveyPoint:
        """Resolve starting Point of Beginning coordinates."""
        point_id = pob_def.get("point_id", "POB")
        monument = pob_def.get("monument", "")
        notes = pob_def.get("notes", "")

        # A. Direct Anchor Reference
        anchor_id = pob_def.get("anchor_id")
        if anchor_id:
            if anchor_id not in self.anchors:
                raise KeyError(f"Anchor '{anchor_id}' referenced in parcel '{parcel_id}' POB not found.")
            anc = self.anchors[anchor_id]
            label = pob_def.get("label", anc.label if anchor_id else point_id)
            lat, lon = anc.lat, anc.lon
            # Check for offset from anchor
            if "bearing" in pob_def and "distance" in pob_def:
                off_brg = (parse_quadrant_bearing(pob_def["bearing"]) + self.grid_rotation_deg) % 360.0
                off_dist_m = float(pob_def["distance"]) * self.unit_to_m
                lat, lon = project_point(lat, lon, off_dist_m, off_brg)
            return SurveyPoint(point_id, label, lat, lon, anc.elevation_m, monument or anc.monument, notes or anc.notes, "RTK_FIX", parcel_id)

        # B. Reference to existing point from previous parcel
        from_pt_ref = pob_def.get("from_point")
        if from_pt_ref:
            base_pt = self._resolve_point(from_pt_ref)
            if not base_pt:
                raise KeyError(f"Point '{from_pt_ref}' referenced in parcel '{parcel_id}' POB not found.")
            label = pob_def.get("label", point_id if point_id != "POB" else base_pt.label)
            lat, lon = base_pt.lat, base_pt.lon
            # Check for offset from reference point
            if "bearing" in pob_def and "distance" in pob_def:
                off_brg = (parse_quadrant_bearing(pob_def["bearing"]) + self.grid_rotation_deg) % 360.0
                off_dist_m = float(pob_def["distance"]) * self.unit_to_m
                lat, lon = project_point(lat, lon, off_dist_m, off_brg)
            return SurveyPoint(point_id, label, lat, lon, base_pt.elevation_m, monument or base_pt.monument, notes or base_pt.notes, "DEED_CALC", parcel_id)

        raise ValueError(f"Parcel '{parcel_id}' POB must specify 'anchor_id' or 'from_point'.")

    def _apply_compass_rule(self, res: ParcelResult):
        """
        Apply Compass Rule (Bowditch Adjustment) to balance a closed traverse.
        Distributes linear misclosure proportionally to cumulative leg distance.
        """
        pob_first = res.points[0]
        end_last = res.points[-1]
        err_m, err_azimuth = compute_geodesic_segment(end_last.lat, end_last.lon, pob_first.lat, pob_first.lon)
        total_p = res.perimeter_m

        if total_p <= 0.0 or err_m < 0.0001:
            return

        cum_dist_m = 0.0
        for leg in res.legs:
            cum_dist_m += leg.distance_m
            correction_dist_m = err_m * (cum_dist_m / total_p)

            # Find matching point in res.points
            for pt in res.points[1:]:
                if pt.point_id == leg.to_point_id:
                    adj_lat, adj_lon = project_point(pt.lat, pt.lon, correction_dist_m, err_azimuth)
                    pt.lat = adj_lat
                    pt.lon = adj_lon
                    pt.point_type = "ADJ_CALC"
                    # Update in global solved points
                    self.solved_points[f"{res.parcel_id}.{pt.point_id}"] = pt
                    self.solved_points[pt.point_id] = pt

        # Force exact closure on nominal closing point
        end_last.lat = pob_first.lat
        end_last.lon = pob_first.lon


# =============================================================================
# EXPORTERS (CSV, KML, GEOJSON, REPORT)
# =============================================================================

def export_rover_csv(solver: CadastralSolver, output_path: str):
    """
    Export solved points in standard RTK Rover 19-column CSV format.
    Compatible with Firmware CSV Browser and field pin seeker.
    """
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    header = [
        "timestamp", "firmware_ver", "label", "latitude", "longitude", "elevation",
        "pos_fix", "hdg_fix", "h_acc_m", "v_acc_m", "hdop", "sats", "slave_sats",
        "diff_age_s", "base_id", "true_heading_deg", "pole_tilt_deg", "samples", "notes"
    ]

    seen_ids = set()
    written_count = 0

    with open(output_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(header)

        # 1. Output Control Anchors
        for aid, anc in solver.anchors.items():
            if aid in seen_ids:
                continue
            seen_ids.add(aid)
            elev_str = f"{anc.elevation_m:.3f}" if anc.elevation_m is not None else ""
            notes = anc.notes if anc.notes else anc.monument
            writer.writerow([
                "", "", anc.label, f"{anc.lat:.9f}", f"{anc.lon:.9f}", elev_str,
                "RTK_FIX", "HDG_FIX", "0.035", "0.032", "0.6", "26", "0",
                "0.0", "0", "0.00", "0.00", "1", notes
            ])
            written_count += 1

        # 2. Output Solved Parcel Points
        for p_res in solver.parcels_results:
            for pt in p_res.points:
                # Deduplicate if already written
                pt_key = f"{p_res.parcel_id}.{pt.point_id}"
                if pt.point_id in seen_ids or pt_key in seen_ids or (pt.point_type == "RTK_FIX" and pt.point_id in seen_ids):
                    continue
                seen_ids.add(pt.point_id)
                seen_ids.add(pt_key)

                fix_str = "RTK_FIX" if pt.point_type == "RTK_FIX" else (
                    "ADJ_CALC" if pt.point_type == "ADJ_CALC" else "DEED_CALC"
                )
                hdg_str = "HDG_FIX" if pt.point_type == "RTK_FIX" else "NONE"
                h_acc = "0.035" if pt.point_type == "RTK_FIX" else "0.000"
                v_acc = "0.032" if pt.point_type == "RTK_FIX" else "0.000"
                hdop = "0.6" if pt.point_type == "RTK_FIX" else "0.0"
                sats = "26" if pt.point_type == "RTK_FIX" else "0"
                elev_str = f"{pt.elevation_m:.3f}" if pt.elevation_m is not None else ""
                notes = f"{pt.monument} ({pt.notes})" if pt.monument and pt.notes else (pt.monument or pt.notes)

                writer.writerow([
                    "", "", pt.label, f"{pt.lat:.9f}", f"{pt.lon:.9f}", elev_str,
                    fix_str, hdg_str, h_acc, v_acc, hdop, sats, "0",
                    "0.0", "0", "0.00", "0.00", "1", notes
                ])
                written_count += 1

    print(f"[EXPORT] Wrote {written_count} waypoints to Rover CSV: {output_path}")


def export_kml(solver: CadastralSolver, output_path: str):
    """Export solved parcels, boundary lines, and corner pins to Google Earth KML."""
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    kml_lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<kml xmlns="http://www.opengis.net/kml/2.2">',
        '  <Document>',
        f'    <name>{solver.project.get("name", "Cadastral Survey Network")}</name>',
        f'    <description>{solver.project.get("description", "")}</description>',
        '    <!-- Styles -->',
        '    <Style id="anchorPin">',
        '      <IconStyle><color>ff0000ff</color><scale>1.2</scale>',
        '        <Icon><href>http://maps.google.com/mapfiles/kml/shapes/placemark_circle.png</href></Icon>',
        '      </IconStyle>',
        '    </Style>',
        '    <Style id="deedPin">',
        '      <IconStyle><color>ff00ffff</color><scale>1.0</scale>',
        '        <Icon><href>http://maps.google.com/mapfiles/kml/shapes/shaded_dot.png</href></Icon>',
        '      </IconStyle>',
        '    </Style>',
        '    <Style id="boundaryLine">',
        '      <LineStyle><color>ff0000ff</color><width>3</width></LineStyle>',
        '      <PolyStyle><color>4d00ff00</color></PolyStyle>',
        '    </Style>'
    ]

    # Folder 1: Control Anchors
    kml_lines.extend([
        '    <Folder>',
        '      <name>Control Anchors</name>'
    ])
    for aid, anc in solver.anchors.items():
        kml_lines.extend([
            '      <Placemark>',
            f'        <name>{anc.label}</name>',
            f'        <description>{anc.monument} - {anc.notes}</description>',
            '        <styleUrl>#anchorPin</styleUrl>',
            f'        <Point><coordinates>{anc.lon:.9f},{anc.lat:.9f},0</coordinates></Point>',
            '      </Placemark>'
        ])
    kml_lines.append('    </Folder>')

    # Folder 2: Parcels (Boundary Polygons & Lines)
    kml_lines.extend([
        '    <Folder>',
        '      <name>Parcels & Boundaries</name>'
    ])
    for p_res in solver.parcels_results:
        kml_lines.extend([
            '      <Placemark>',
            f'        <name>{p_res.name}</name>',
            f'        <description>{p_res.book_page}</description>',
            '        <styleUrl>#boundaryLine</styleUrl>',
            '        <Polygon>',
            '          <outerBoundaryIs><LinearRing><coordinates>'
        ])
        for pt in p_res.points:
            kml_lines.append(f'            {pt.lon:.9f},{pt.lat:.9f},0')
        # Close polygon ring
        if p_res.points:
            p0 = p_res.points[0]
            kml_lines.append(f'            {p0.lon:.9f},{p0.lat:.9f},0')
        kml_lines.extend([
            '          </coordinates></LinearRing></outerBoundaryIs>',
            '        </Polygon>',
            '      </Placemark>'
        ])
    kml_lines.append('    </Folder>')

    # Folder 3: Corner Monuments
    kml_lines.extend([
        '    <Folder>',
        '      <name>Survey Pins & Monuments</name>'
    ])
    seen_ids = set()
    for p_res in solver.parcels_results:
        for pt in p_res.points:
            if pt.point_id in seen_ids or pt.point_type == "RTK_FIX":
                continue
            seen_ids.add(pt.point_id)
            kml_lines.extend([
                '      <Placemark>',
                f'        <name>{pt.label}</name>',
                f'        <description>{pt.monument} | {pt.notes}</description>',
                '        <styleUrl>#deedPin</styleUrl>',
                f'        <Point><coordinates>{pt.lon:.9f},{pt.lat:.9f},0</coordinates></Point>',
                '      </Placemark>'
            ])
    kml_lines.append('    </Folder>')

    kml_lines.extend([
        '  </Document>',
        '</kml>'
    ])

    with open(output_path, "w", encoding="utf-8") as f:
        f.write("\n".join(kml_lines) + "\n")
    print(f"[EXPORT] Wrote Google Earth KML: {output_path}")


def export_geojson(solver: CadastralSolver, output_path: str):
    """Export solved parcels and points to standard RFC 7946 GeoJSON."""
    os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
    features = []

    # 1. Point features for all control anchors
    for aid, anc in solver.anchors.items():
        feat = {
            "type": "Feature",
            "geometry": {
                "type": "Point",
                "coordinates": [round(anc.lon, 9), round(anc.lat, 9)]
            },
            "properties": {
                "id": anc.point_id,
                "label": anc.label,
                "type": "control_anchor",
                "monument": anc.monument,
                "notes": anc.notes
            }
        }
        features.append(feat)

    # 2. Point features and Polygon features for parcels
    seen_pts = set()
    for p_res in solver.parcels_results:
        # Polygon feature
        poly_coords = [[round(pt.lon, 9), round(pt.lat, 9)] for pt in p_res.points]
        if poly_coords:
            poly_coords.append(poly_coords[0])  # Close ring
        feat_poly = {
            "type": "Feature",
            "geometry": {
                "type": "Polygon",
                "coordinates": [poly_coords]
            },
            "properties": {
                "parcel_id": p_res.parcel_id,
                "name": p_res.name,
                "book_page": p_res.book_page,
                "is_closed": p_res.is_closed,
                "perimeter_m": round(p_res.perimeter_m, 3),
                "closure_error_m": round(p_res.closure_error_m, 4),
                "precision_ratio": f"1:{p_res.precision_ratio}" if p_res.precision_ratio else "N/A",
                "adjustment": p_res.adjustment_applied
            }
        }
        features.append(feat_poly)

        # Individual monument points
        for pt in p_res.points:
            if pt.point_id in seen_pts or pt.point_type == "RTK_FIX":
                continue
            seen_pts.add(pt.point_id)
            feat_pt = {
                "type": "Feature",
                "geometry": {
                    "type": "Point",
                    "coordinates": [round(pt.lon, 9), round(pt.lat, 9)]
                },
                "properties": {
                    "id": pt.point_id,
                    "label": pt.label,
                    "parcel_id": p_res.parcel_id,
                    "type": pt.point_type,
                    "monument": pt.monument,
                    "notes": pt.notes
                }
            }
            features.append(feat_pt)

    geojson_obj = {
        "type": "FeatureCollection",
        "name": solver.project.get("name", "Cadastral Network"),
        "features": features
    }

    with open(output_path, "w", encoding="utf-8") as f:
        json.dump(geojson_obj, f, indent=2)
    print(f"[EXPORT] Wrote GeoJSON FeatureCollection: {output_path}")


def generate_closure_report(solver: CadastralSolver, output_path: Optional[str] = None) -> str:
    """Generate comprehensive human-readable ASCII survey traverse report."""
    lines = []
    lines.append("=" * 80)
    lines.append(f"CADASTRAL SURVEY & TRAVERSE CLOSURE REPORT")
    lines.append(f"Project: {solver.project.get('name', 'Survey Project')}")
    lines.append(f"Datum:   {solver.project.get('datum', 'WGS84')}")
    lines.append(f"Units:   {solver.units_name}")
    lines.append(f"Derived Grid-to-True Rotation: {solver.grid_rotation_deg:+.4f} deg")
    lines.append("=" * 80)
    lines.append("")

    # Control Anchors Section
    lines.append("1. GROUND TRUTH CONTROL ANCHORS:")
    lines.append("-" * 80)
    lines.append(f"{'Anchor ID':<20} {'Latitude':<15} {'Longitude':<16} {'Monument Description'}")
    lines.append("-" * 80)
    for aid, anc in solver.anchors.items():
        lines.append(f"{anc.point_id:<20} {anc.lat:<15.9f} {anc.lon:<16.9f} {anc.monument}")
    lines.append("")

    # Parcels Section
    for p_idx, p_res in enumerate(solver.parcels_results, 1):
        lines.append(f"2.{p_idx} PARCEL TRAVERSE: {p_res.name} (ID: {p_res.parcel_id})")
        if p_res.book_page:
            lines.append(f"    Legal Record: {p_res.book_page}")
        lines.append("-" * 80)
        lines.append(f"{'Call':<5} {'Type':<6} {'Bearing':<18} {'Length':<12} {'From -> To Point':<26} {'Monument'}")
        lines.append("-" * 80)

        for leg in p_res.legs:
            conn = f"{leg.from_point_id} -> {leg.to_point_id}"
            dist_str = f"{leg.distance_user:.2f} {solver.units_name[:2]}"
            lines.append(
                f"{leg.call_index:<5} {leg.call_type:<6} {leg.raw_bearing:<18} {dist_str:<12} {conn:<26} {leg.monument}"
            )
        lines.append("-" * 80)

        # Misclosure summary if closed
        if p_res.is_closed:
            err_user = p_res.closure_error_m * solver.m_to_unit
            err_inches = err_user * 12.0 if solver.units_name in ["feet", "us_survey_feet"] else err_user * 39.3701
            p_user = p_res.perimeter_m * solver.m_to_unit
            lines.append(f"  Traverse Closure Metrics:")
            lines.append(f"    Perimeter:            {p_user:.2f} {solver.units_name}")
            lines.append(f"    Linear Misclosure:    {err_user:.4f} {solver.units_name} ({err_inches:.2f} inches)")
            lines.append(f"    Misclosure Direction: {format_quadrant_bearing(p_res.closure_azimuth_deg)} ({p_res.closure_azimuth_deg:.2f} deg)")
            lines.append(f"    Precision Ratio:      1 : {p_res.precision_ratio:,}")
            lines.append(f"    Balance Adjustment:   {p_res.adjustment_applied.upper()}")
        else:
            lines.append("  Traverse Status: Open traverse / Non-closing boundary.")
        lines.append("")

    # Solved Coordinates Table
    lines.append("3. SOLVED CORNER MONUMENTS & WAYPOINTS TABLE:")
    lines.append("-" * 80)
    lines.append(f"{'Point Label':<24} {'Latitude':<15} {'Longitude':<16} {'Fix':<10} {'Monument'}")
    lines.append("-" * 80)
    seen = set()
    for p_res in solver.parcels_results:
        for pt in p_res.points:
            if pt.point_id in seen:
                continue
            seen.add(pt.point_id)
            lines.append(f"{pt.label:<24} {pt.lat:<15.9f} {pt.lon:<16.9f} {pt.point_type:<10} {pt.monument}")
    lines.append("=" * 80)

    report_text = "\n".join(lines) + "\n"

    if output_path:
        os.makedirs(os.path.dirname(os.path.abspath(output_path)), exist_ok=True)
        with open(output_path, "w", encoding="utf-8") as f:
            f.write(report_text)
        print(f"[EXPORT] Wrote closure report: {output_path}")

    return report_text


# =============================================================================
# CLI ENTRY POINT
# =============================================================================

def parse_external_anchors_csv(csv_path: str) -> List[Dict[str, Any]]:
    """Parse external control anchors CSV file."""
    if not os.path.isfile(csv_path):
        raise FileNotFoundError(f"Control anchors file not found: {csv_path}")

    anchors = []
    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            pid = row.get("id") or row.get("label") or row.get("point_id")
            if not pid or not row.get("latitude") or not row.get("longitude"):
                continue
            anchors.append({
                "id": pid.strip(),
                "label": row.get("label", pid).strip(),
                "latitude": float(row["latitude"]),
                "longitude": float(row["longitude"]),
                "elevation_m": float(row["elevation_m"]) if row.get("elevation_m") else None,
                "monument": row.get("monument", "").strip(),
                "notes": row.get("notes", "").strip()
            })
    return anchors


def main():
    print(f"=== {__filename__} {__version__} ===")
    parser = argparse.ArgumentParser(
        description="Generic Schema-Driven Cadastral COGO & Traverse Engine for RTK Rover."
    )
    parser.add_argument("--input", "-i", type=str, required=True, help="Path to input cadastral schema JSON file")
    parser.add_argument("--anchors", "-a", type=str, default=None, help="Optional external control anchors CSV file")
    parser.add_argument("--rover-csv", type=str, default=None, help="Export path for Rover 19-column waypoints.csv")
    parser.add_argument("--kml", type=str, default=None, help="Export path for Google Earth KML")
    parser.add_argument("--geojson", type=str, default=None, help="Export path for GeoJSON FeatureCollection")
    parser.add_argument("--report", type=str, default=None, help="Export path for text closure report")
    parser.add_argument("--adjust", choices=["none", "compass", "bowditch"], default=None,
                        help="Override traverse adjustment rule (e.g. 'compass')")
    parser.add_argument("--rotation", type=float, default=None,
                        help="Override grid-to-true rotation offset in degrees")

    args = parser.parse_args()

    if not os.path.isfile(args.input):
        print(f"[ERROR] Input JSON file not found: {args.input}")
        sys.exit(1)

    with open(args.input, "r", encoding="utf-8") as f:
        schema_data = json.load(f)

    external_anchors = None
    if args.anchors:
        external_anchors = parse_external_anchors_csv(args.anchors)

    solver = CadastralSolver(
        schema_data=schema_data,
        external_anchors=external_anchors,
        manual_rotation_deg=args.rotation,
        force_adjust=args.adjust
    )

    print(f"[*] Solving cadastral network '{solver.project.get('name', 'Survey')}'...")
    print(f"[*] Units: {solver.units_name} | Derived Grid Rotation: {solver.grid_rotation_deg:+.4f} deg")

    solver.solve_all()

    # Generate and print summary report
    report_text = generate_closure_report(solver, args.report)
    print("\n" + report_text)

    # Perform requested exports
    if args.rover_csv:
        export_rover_csv(solver, args.rover_csv)
    if args.kml:
        export_kml(solver, args.kml)
    if args.geojson:
        export_geojson(solver, args.geojson)

    print("[DONE] Cadastral network successfully solved.")


if __name__ == "__main__":
    main()
