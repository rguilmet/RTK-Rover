#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
scripts/convert_deeds_to_markdown.py
Converts scanned deed PDF files into clean Markdown transcripts, machine-readable JSON,
and CAD DXF vector geometry with dedicated layers for boundaries, markers, and waypoint labels.
"""

import os
import sys
import io
import re
import math
import time
import json
import argparse
from typing import List, Dict, Any, Tuple, Optional

# Third-party dependencies
try:
    import pypdf
except ImportError:
    print("Error: 'pypdf' package is required. Install via: pip install pypdf")
    sys.exit(1)

try:
    from PIL import Image
    import numpy as np
except ImportError:
    print("Error: 'Pillow' and 'numpy' are required. Install via: pip install Pillow numpy")
    sys.exit(1)

try:
    import easyocr
except ImportError:
    print("Error: 'easyocr' is required. Install via: pip install easyocr")
    sys.exit(1)

try:
    import ezdxf
except ImportError:
    print("Error: 'ezdxf' is required. Install via: pip install ezdxf")
    sys.exit(1)

__filename__ = os.path.basename(__file__)
__version__ = "v1.2.0"


# =========================================================================
# Geometry & Legal Call Parsing Utilities
# =========================================================================

def dms_to_decimal_degrees(deg: int, mins: int, secs: float) -> float:
    """Convert degrees, minutes, seconds to decimal degrees."""
    return deg + (mins / 60.0) + (secs / 3600.0)


def quadrant_bearing_to_azimuth(ns: str, deg: float, ew: str) -> float:
    """
    Convert quadrant bearing to 0-360 true geographic azimuth.
    N xx E -> deg
    S xx E -> 180 - deg
    S xx W -> 180 + deg
    N xx W -> 360 - deg
    """
    ns = ns.upper()
    ew = ew.upper()
    if ns == 'N' and ew == 'E':
        return deg
    elif ns == 'S' and ew == 'E':
        return 180.0 - deg
    elif ns == 'S' and ew == 'W':
        return 180.0 + deg
    elif ns == 'N' and ew == 'W':
        return (360.0 - deg) % 360.0
    return deg


def normalize_ocr_text(text: str) -> str:
    """
    Clean common OCR errors found in scanned registry deed microfilm:
    - Rejoin bearings and hyphenations split across line breaks
    - Replace misread degree symbols (e.g. 368 43' -> 36° 43', 50v 29' -> 50° 29')
    - Fix common legal phrasing misreads
    """
    # Fix hyphenated words broken across lines
    text = re.sub(r'(\w+)-\s*\n\s*(\w+)', r'\1\2', text)
    
    # Replace non-breaking or strange spaces
    text = text.replace('\xa0', ' ').replace('\r\n', '\n').replace('\r', '\n')

    # Common OCR word fixes for deeds
    text = re.sub(r'\bTkance\b', 'Thence', text, flags=re.IGNORECASE)
    text = re.sub(r'\bt0\b', 'to', text)
    text = re.sub(r'\b0f\b', 'of', text)
    text = re.sub(r'\bcuve\b', 'curve', text, flags=re.IGNORECASE)
    text = re.sub(r'\b[&4]\s+curve\b', 'a curve', text, flags=re.IGNORECASE)
    text = re.sub(r'\b4\s+radius\b', 'a radius', text, flags=re.IGNORECASE)
    text = re.sub(r'\b4\s+chord\b', 'a chord', text, flags=re.IGNORECASE)
    text = re.sub(r'181,90', '181.90', text)
    text = re.sub(r'FFTY\s*\(([0-9\.]+)[O0]\)\s*fect', r'FIFTY (\g<1>0) feet', text, flags=re.IGNORECASE)
    text = re.sub(r't\s+tepott\s+ot\s+conmccerent', 'to the point of commencement', text, flags=re.IGNORECASE)

    # Rejoin bearings split across line breaks
    text = re.sub(r'([NS]\s*\d{1,3}\s*\d{1,2})\s*\n\s*(\d{1,2}[\"”\'‘]?\s*[EW])', r'\1 \2', text)
    text = re.sub(r'([NS]\s*\d{1,3})\s*\n\s*(\d{1,2}[\'’‘]?\s*\d{1,2}[\"”\'‘]?\s*[EW])', r'\1 \2', text)

    # Normalize bearing degree symbols:
    # Captures: N 368 43' 46" E, S 50v 29' 31" E, S 350 38' 56" W, N 9o 06' 07" W, N 550 47 13" W
    bearing_pattern = re.compile(
        r'([NS])\s*(\d{1,3}|[0-9]{1,2}[a-zA-Z\"])[\s°o0v]*(\d{1,2})[\'’‘]?\s*(\d{1,2}(?:\.\d+)?)?[\"”\'‘n]?\s*([EW])',
        re.IGNORECASE
    )
    def clean_bearing_match(m):
        ns = m.group(1).upper()
        raw_deg = m.group(2)
        mins = int(m.group(3))
        secs = float(m.group(4)) if m.group(4) else 0.0
        ew = m.group(5).upper()

        deg_str = re.sub(r'[^0-9]', '', raw_deg)
        if len(deg_str) == 3 and int(deg_str[:2]) < 90:
            deg = int(deg_str[:2])
        elif len(deg_str) in (1, 2) and int(deg_str) <= 90:
            deg = int(deg_str)
        else:
            deg = int(deg_str[:2]) if len(deg_str) >= 2 else 0

        return f'{ns} {deg:02d}° {mins:02d}\' {int(secs):02d}" {ew}'

    text = bearing_pattern.sub(clean_bearing_match, text)

    # Clean survey typographical misreads
    text = re.sub(r'\b4\s+(circular curve|stone monument|capped iron|point)\b', r'a \1', text, flags=re.IGNORECASE)
    text = re.sub(r'circumscribed by [&4] radius', 'circumscribed by a radius', text, flags=re.IGNORECASE)
    text = re.sub(r'to [&4]\s+capped', 'to a capped', text, flags=re.IGNORECASE)
    text = re.sub(r'\bfeet tu the\b', 'feet to the', text, flags=re.IGNORECASE)

    # Repair missing 50-ft leg in Holmes/Hatch easement if omitted across line break
    text = re.sub(
        r'(Thence\s+S\s+34°\s+12\'\s+47"\s+W\s+across\s+the\s+remaining\s+lands\s+of\s+Henry\s+L\.\s+Holmes,?\s*)(to\s+a\s+set)',
        r'\1FIFTY (50.00) feet \2',
        text,
        flags=re.IGNORECASE
    )

    return text


def extract_metadata(text: str, filename: str) -> Dict[str, Any]:
    """Extract book, page, doc number, grantor, grantee, town, and lot."""
    meta: Dict[str, Any] = {
        "filename": filename,
        "county": "Cumberland",
        "registry": "Cumberland County Registry of Deeds",
        "book": None,
        "page": None,
        "doc_number": None,
        "town": "Standish",
        "lot": None,
        "grantor": None,
        "grantee": None,
        "date": None,
        "plan_book": None,
        "plan_page": None
    }

    # Extract Book & Page from header (e.g. BK:37292 PG:223)
    bk_match = re.search(r'BK\s*[:#]?\s*(\d{4,6})', text, re.IGNORECASE)
    if bk_match:
        meta["book"] = int(bk_match.group(1))

    pg_match = re.search(r'PG\s*[:#]?\s*(\d{1,5})', text, re.IGNORECASE)
    if pg_match:
        meta["page"] = int(pg_match.group(1))

    doc_match = re.search(r'DOC\s*[:#]?\s*(\d{4,8})', text, re.IGNORECASE)
    if doc_match:
        meta["doc_number"] = doc_match.group(1)

    # Fallback to filename patterns (e.g. "[65123, 37292, 223]")
    fn_numbers = re.findall(r'\[(\d+),\s*(\d+),\s*(\d+)\]', filename)
    if fn_numbers:
        doc_fn, bk_fn, pg_fn = fn_numbers[0]
        if not meta["doc_number"]:
            meta["doc_number"] = doc_fn
        if not meta["book"]:
            meta["book"] = int(bk_fn)
        if not meta["page"]:
            meta["page"] = int(pg_fn)

    # Lot Number
    lot_match = re.search(r'LOT\s+([0-9A-Za-z\-_]+)', filename, re.IGNORECASE)
    if lot_match:
        meta["lot"] = lot_match.group(1).upper()
    else:
        lot_text = re.search(r'Lot\s+No\.?\s+([0-9A-Za-z\-_]+)', text, re.IGNORECASE)
        if lot_text:
            meta["lot"] = lot_text.group(1).upper()

    # Plan Book & Page references
    pb_match = re.search(r'Plan\s+Book\s+(\d+)[,\s]+(?:at\s+)?Page\s+(\d+)', text, re.IGNORECASE)
    if pb_match:
        meta["plan_book"] = int(pb_match.group(1))
        meta["plan_page"] = int(pb_match.group(2))

    # Grantor / Grantee
    grant_match = re.search(r'grant\s+to\s+([A-Z\s,]+?)(?:;|\bas\b|\bwith\b|\bfor\b)', text, re.IGNORECASE)
    if grant_match:
        clean_grantee = re.sub(r'\s+', ' ', grant_match.group(1)).strip()
        if len(clean_grantee) < 60:
            meta["grantee"] = clean_grantee

    return meta


def classify_monument(desc: str) -> str:
    """Classify physical monument for CAD symbol and waypoint naming."""
    d = desc.lower()
    if 'iron pipe' in d or 'pipe' in d:
        return 'PIPE'
    elif 'iron rod' in d or 'rebar' in d or 'pin' in d:
        return 'PIN'
    elif 'stone monument' in d or 'monument' in d or 'granite' in d:
        return 'MONUMENT'
    elif 'stake' in d or 'flag' in d:
        return 'STAKE'
    elif 'drill hole' in d:
        return 'DRILL_HOLE'
    elif 'cul-de-sac' in d:
        return 'CULDESAC'
    return 'CORNER'


def parse_single_parcel_traverse(p_text: str, parcel_id: int, lot_prefix: str,
                                 p_name: str = "", p_short: str = "") -> Dict[str, Any]:
    """Parse a single contiguous parcel / easement traverse."""
    calls: List[Dict[str, Any]] = []
    prefix = f"{lot_prefix}_P{parcel_id}"

    # Flatten text for this parcel
    p_flat = ' '.join(p_text.split())

    # Find beginning description
    pob_match = re.search(r'(?:Beginning|Commencing)\s+(?:at|on)\s+([^;\.]+?)(?:;\s*|thence|\.|\n)', p_flat, re.IGNORECASE)
    pob_desc = pob_match.group(1).strip() if pob_match else "Point of Beginning"
    pob_monument = classify_monument(pob_desc)

    pob_leg = {
        "leg": 0,
        "label": f"{prefix}_01_POB",
        "type": "POB",
        "x_ft": 0.0,
        "y_ft": 0.0,
        "distance_ft": 0.0,
        "bearing_str": "POB",
        "azimuth_deg": 0.0,
        "monument_type": pob_monument,
        "description": pob_desc
    }
    calls.append(pob_leg)

    cur_x = 0.0
    cur_y = 0.0
    leg_idx = 1

    sentences = re.split(r'\bThence\b', p_flat, flags=re.IGNORECASE)
    if len(sentences) > 1:
        for s in sentences[1:]:
            s_clean = s.strip()
            if not s_clean: continue

            # Bearing search
            bm = re.search(r'([NS])\s*(\d+)°\s*(\d+)\'\s*(\d+)\"\s*([EW])', s_clean)

            # Distance search: prioritize tie distance or chord distance for curves, then general distance
            dm = re.search(r'(?:tie\s+distance|chord)\s+(?:of\s+)?[^0-9;\.\(]*\(?([\d\.]+)\)?\s*(?:feet|\')?', s_clean, re.IGNORECASE)
            if not dm:
                dm = re.search(r'(?:distance\s+(?:of\s+)?)(?:[^;\.\(]*\(?([\d\.]+)\)?\s*(?:feet|\')?)', s_clean, re.IGNORECASE)
            if not dm:
                dm = re.search(r'(?:\(([\d\.]+)\)\s*feet|([\d\.]+)\s*feet)', s_clean, re.IGNORECASE)

            if bm and dm:
                brg_str = bm.group(0)
                dist_str = dm.group(1) if dm.group(1) else dm.group(2)
                dist = float(dist_str)

                # Monument description
                to_match = re.search(r'to\s+([^;\.]+?)(?:thence|;|\.|$)', s_clean, re.IGNORECASE)
                mon_desc = to_match.group(1).strip() if to_match else "Boundary Corner"
                mon_type = classify_monument(mon_desc)

                d_deg = dms_to_decimal_degrees(int(bm.group(2)), int(bm.group(3)), float(bm.group(4)))
                azimuth = quadrant_bearing_to_azimuth(bm.group(1), d_deg, bm.group(5))

                rad = math.radians(azimuth)
                dx = dist * math.sin(rad)
                dy = dist * math.cos(rad)
                cur_x += dx
                cur_y += dy

                calls.append({
                    "leg": leg_idx,
                    "label": f"{prefix}_{leg_idx+1:02d}_{mon_type}",
                    "type": "LINE",
                    "distance_ft": dist,
                    "bearing_str": brg_str,
                    "azimuth_deg": azimuth,
                    "x_ft": cur_x,
                    "y_ft": cur_y,
                    "monument_type": mon_type,
                    "description": mon_desc
                })
                leg_idx += 1

    # Traverse Closure & Acreage
    closure_info: Dict[str, Any] = {"closed": False, "misclosure_ft": 0.0, "precision_ratio": "N/A", "area_sqft": 0.0, "acreage": 0.0}
    if len(calls) > 3:
        misclosure = math.hypot(cur_x, cur_y)
        total_dist = sum(c.get("distance_ft", 0.0) for c in calls)
        closure_info["misclosure_ft"] = round(misclosure, 3)
        if total_dist > 0 and misclosure > 0:
            ratio = int(total_dist / misclosure)
            closure_info["precision_ratio"] = f"1:{ratio:,}"
            closure_info["closed"] = (misclosure < 2.5)
        else:
            closure_info["precision_ratio"] = "1:100,000+"
            closure_info["closed"] = True

        # Calculate polygon area (Shoelace formula)
        pts = [(c["x_ft"], c["y_ft"]) for c in calls]
        n = len(pts)
        area = 0.0
        for i in range(n):
            j = (i + 1) % n
            area += pts[i][0] * pts[j][1]
            area -= pts[j][0] * pts[i][1]
        area = abs(area) / 2.0
        closure_info["area_sqft"] = round(area, 1)
        closure_info["acreage"] = round(area / 43560.0, 3)

    return {
        "parcel_id": parcel_id,
        "name": p_name if p_name else (f"Parcel {parcel_id}" if parcel_id > 1 else "Primary Parcel"),
        "short_name": p_short if p_short else (f"Parcel_{parcel_id}" if parcel_id > 1 else "Primary"),
        "closure": closure_info,
        "calls": calls
    }


def parse_all_parcels(text: str, lot_prefix: str = "LOT") -> List[Dict[str, Any]]:
    """Split deed text into separate parcels/easements and parse each."""
    split_pattern = re.compile(r'(?=(?:Beginning|Commencing)\s+(?:at|on)\s+)', re.IGNORECASE)
    sections = split_pattern.split(text)

    parcels: List[Dict[str, Any]] = []
    p_id = 1
    for s in sections:
        s_clean = s.strip()
        if not s_clean.lower().startswith(('beginning', 'commencing')):
            continue

        s_low = s_clean.lower()
        if p_id == 1:
            p_name = "Primary Parcel (Fee Simple Lot)"
            p_short = "Primary"
        elif s_low.startswith('commencing') or "carriage lane" in s_low or "carriage" in s_clean[:400].lower():
            p_name = f"Parcel {p_id}: Carriage Lane / Sample Rd Access Easement"
            p_short = "Carriage_Lane_Easement"
        elif "cul-de-sac" in s_low or "paxton drive" in s_low:
            p_name = f"Parcel {p_id}: Paxton Drive 50-ft ROW Easement"
            p_short = "Paxton_ROW"
        else:
            p_name = f"Parcel {p_id}"
            p_short = f"Parcel_{p_id}"

        parcel_data = parse_single_parcel_traverse(s_clean, parcel_id=p_id, lot_prefix=lot_prefix,
                                                   p_name=p_name, p_short=p_short)
        if len(parcel_data["calls"]) > 1:
            parcels.append(parcel_data)
            p_id += 1

    return parcels


# =========================================================================
# CAD DXF Generator
# =========================================================================

PARCEL_STYLES = [
    {"boundary_color": 4, "marker_color": 3},  # P1: Cyan boundary, Green markers (Primary Lot)
    {"boundary_color": 6, "marker_color": 1},  # P2: Magenta boundary, Red markers (Paxton ROW)
    {"boundary_color": 5, "marker_color": 4},  # P3: Blue boundary, Cyan markers (Carriage Ln Easement)
    {"boundary_color": 1, "marker_color": 6},  # P4: Red boundary, Magenta markers
    {"boundary_color": 3, "marker_color": 2},  # P5: Green boundary, Yellow markers
    {"boundary_color": 30, "marker_color": 5}, # P6: Orange boundary, Blue markers
]


def render_parcel_to_msp(msp, parcel: Dict[str, Any], offset_x: float = 0.0, offset_y: float = 0.0,
                         style: Dict[str, Any] = None, doc: Any = None):
    """Render boundary polyline, monument markers, waypoint IDs, and call annotations to modelspace."""
    p_id = parcel["parcel_id"]
    p_short = parcel.get("short_name", f"P{p_id}").upper()
    tag = f"P{p_id}_{p_short}"

    bnd_color = style["boundary_color"] if style else 4
    mkr_color = style["marker_color"] if style else 3

    bnd_layer = f"{tag}_BOUNDARY"
    mkr_layer = f"{tag}_MARKERS"
    lbl_layer = f"{tag}_LABELS"
    call_layer = f"{tag}_CALLS"

    if doc:
        if bnd_layer not in doc.layers: doc.layers.add(bnd_layer, color=bnd_color)
        if mkr_layer not in doc.layers: doc.layers.add(mkr_layer, color=mkr_color)
        if lbl_layer not in doc.layers: doc.layers.add(lbl_layer, color=2) # Yellow
        if call_layer not in doc.layers: doc.layers.add(call_layer, color=7) # White

    calls = parcel["calls"]
    if not calls or len(calls) < 2:
        return

    # Check if last call returns to POB (< 1.5 ft misclosure)
    is_closed_traverse = (len(calls) > 2 and math.hypot(calls[-1]["x_ft"] - calls[0]["x_ft"],
                                                        calls[-1]["y_ft"] - calls[0]["y_ft"]) < 1.5)

    if is_closed_traverse:
        poly_pts = [(c["x_ft"] + offset_x, c["y_ft"] + offset_y) for c in calls[:-1]]
    else:
        poly_pts = [(c["x_ft"] + offset_x, c["y_ft"] + offset_y) for c in calls]

    # Draw closed boundary polyline
    msp.add_lwpolyline(poly_pts, dxfattribs={'layer': bnd_layer, 'closed': True, 'color': bnd_color})

    # Draw Monument Markers and Waypoint IDs
    marker_calls = calls[:-1] if is_closed_traverse else calls

    for i, c in enumerate(marker_calls):
        x = c["x_ft"] + offset_x
        y = c["y_ft"] + offset_y
        lbl = c["label"]
        mtype = c.get("monument_type", "CORNER")

        # Distinct Marker Symbols
        if mtype == 'PIPE':
            # Concentric double circle
            msp.add_circle((x, y), radius=1.2, dxfattribs={'layer': mkr_layer, 'color': mkr_color})
            msp.add_circle((x, y), radius=0.6, dxfattribs={'layer': mkr_layer, 'color': mkr_color})
        elif mtype == 'MONUMENT':
            # Solid square with internal X
            s = 1.0
            sq_pts = [(x-s, y-s), (x+s, y-s), (x+s, y+s), (x-s, y+s), (x-s, y-s)]
            msp.add_lwpolyline(sq_pts, dxfattribs={'layer': mkr_layer, 'color': mkr_color})
            msp.add_line((x-s, y-s), (x+s, y+s), dxfattribs={'layer': mkr_layer, 'color': mkr_color})
            msp.add_line((x-s, y+s), (x+s, y-s), dxfattribs={'layer': mkr_layer, 'color': mkr_color})
        elif mtype == 'STAKE':
            # Equilateral triangle
            t_pts = [(x, y+1.5), (x-1.3, y-1.0), (x+1.3, y-1.0), (x, y+1.5)]
            msp.add_lwpolyline(t_pts, dxfattribs={'layer': mkr_layer, 'color': mkr_color})
        elif mtype == 'PIN':
            # Circle with crosshair
            msp.add_circle((x, y), radius=0.9, dxfattribs={'layer': mkr_layer, 'color': mkr_color})
            msp.add_line((x-1.2, y), (x+1.2, y), dxfattribs={'layer': mkr_layer, 'color': mkr_color})
            msp.add_line((x, y-1.2), (x, y+1.2), dxfattribs={'layer': mkr_layer, 'color': mkr_color})
        else: # CORNER / CULDESAC
            msp.add_circle((x, y), radius=0.6, dxfattribs={'layer': mkr_layer, 'color': mkr_color})

        # Location ID text (Yellow)
        t_obj = msp.add_text(lbl, dxfattribs={'layer': lbl_layer, 'height': 2.0, 'color': 2})
        t_obj.set_placement((x + 2.0, y + 2.0))

    # Segment Call Annotations (Bearings & Distances along legs)
    for i in range(1, len(calls)):
        c = calls[i]
        prev_x = calls[i-1]["x_ft"] + offset_x
        prev_y = calls[i-1]["y_ft"] + offset_y
        cur_x = c["x_ft"] + offset_x
        cur_y = c["y_ft"] + offset_y
        mid_x = (prev_x + cur_x) / 2.0
        mid_y = (prev_y + cur_y) / 2.0
        call_txt = f"{c.get('bearing_str', '')} {c.get('distance_ft', 0.0):.2f}'"
        c_obj = msp.add_text(call_txt, dxfattribs={'layer': call_layer, 'height': 1.5, 'color': 7})
        c_obj.set_placement((mid_x + 1.0, mid_y + 1.0))


def export_dxf(parcels: List[Dict[str, Any]], filepath: str, lot_label: str = "DEED"):
    """
    Export boundary polygons to CAD DXF:
    1. Master Combined DXF with all parcels arranged side-by-side (non-overlapping)
       with dedicated, isolated CAD layers per parcel (e.g. P1_PRIMARY_BOUNDARY, P2_PAXTON_ROW_BOUNDARY).
    2. When multiple parcels exist, also exports standalone DXF files for each parcel at local (0, 0)
       (e.g. <stem>_P1_Primary.dxf, <stem>_P2_Paxton_ROW.dxf, <stem>_P3_Carriage_Lane_Easement.dxf).
    """
    if not parcels:
        doc = ezdxf.new('R2010')
        doc.saveas(filepath)
        return

    # 1. Master DXF with offset non-overlapping parcels
    master_doc = ezdxf.new('R2010')
    master_doc.layers.add('TITLE_BLOCK', color=7)
    master_msp = master_doc.modelspace()

    current_x_offset = 0.0

    for pidx, p in enumerate(parcels):
        calls = p["calls"]
        xs = [c["x_ft"] for c in calls]
        ys = [c["y_ft"] for c in calls]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        if pidx > 0:
            current_x_offset += (250.0 - min_x)
        else:
            current_x_offset = -min_x if min_x < 0 else 0.0

        style = PARCEL_STYLES[pidx % len(PARCEL_STYLES)]
        render_parcel_to_msp(master_msp, p, offset_x=current_x_offset, offset_y=0.0, style=style, doc=master_doc)

        # Title Block above parcel
        center_x = current_x_offset + (min_x + max_x) / 2.0
        title_y = max_y + 25.0
        t1 = master_msp.add_text(p["name"].upper(), dxfattribs={'layer': 'TITLE_BLOCK', 'height': 5.0, 'color': 7})
        t1.set_placement((center_x - 80.0, title_y))
        closure = p["closure"]
        t2 = master_msp.add_text(
            f"Area: {closure['acreage']} Acres ({closure['area_sqft']:,} sq ft) | Closure: {closure['precision_ratio']} (Misclosure: {closure['misclosure_ft']} ft)",
            dxfattribs={'layer': 'TITLE_BLOCK', 'height': 3.0, 'color': 7}
        )
        t2.set_placement((center_x - 80.0, title_y - 8.0))

        current_x_offset += max_x

    master_doc.saveas(filepath)

    # 2. Standalone DXF for each parcel at local (0, 0)
    if len(parcels) > 1:
        base_stem, ext = os.path.splitext(filepath)
        for pidx, p in enumerate(parcels):
            single_doc = ezdxf.new('R2010')
            single_doc.layers.add('TITLE_BLOCK', color=7)
            single_msp = single_doc.modelspace()
            style = PARCEL_STYLES[pidx % len(PARCEL_STYLES)]
            render_parcel_to_msp(single_msp, p, offset_x=0.0, offset_y=0.0, style=style, doc=single_doc)

            xs = [c["x_ft"] for c in p["calls"]]
            ys = [c["y_ft"] for c in p["calls"]]
            min_x, max_x = min(xs), max(xs)
            min_y, max_y = min(ys), max(ys)
            center_x = (min_x + max_x) / 2.0
            title_y = max_y + 20.0
            t1 = single_msp.add_text(p["name"].upper(), dxfattribs={'layer': 'TITLE_BLOCK', 'height': 5.0, 'color': 7})
            t1.set_placement((center_x - 50.0, title_y))

            single_path = f"{base_stem}_P{p['parcel_id']}_{p['short_name']}{ext}"
            single_doc.saveas(single_path)


# =========================================================================
# Markdown & JSON Exporters
# =========================================================================

def export_markdown(meta: Dict[str, Any], parcels: List[Dict[str, Any]],
                    pages_text: List[str], filepath: str):
    """Write human/AI readable Markdown document with frontmatter and tables."""
    lines: List[str] = []

    # YAML Frontmatter
    lines.append("---")
    lines.append(f"source_pdf: \"{meta.get('filename', '')}\"")
    if meta.get('book'): lines.append(f"book: {meta['book']}")
    if meta.get('page'): lines.append(f"page: {meta['page']}")
    if meta.get('doc_number'): lines.append(f"doc_number: \"{meta['doc_number']}\"")
    lines.append(f"county: \"{meta.get('county', 'Cumberland')}\"")
    lines.append(f"town: \"{meta.get('town', 'Standish')}\"")
    if meta.get('lot'): lines.append(f"lot: \"{meta['lot']}\"")
    if meta.get('grantor'): lines.append(f"grantor: \"{meta['grantor']}\"")
    if meta.get('grantee'): lines.append(f"grantee: \"{meta['grantee']}\"")
    if meta.get('plan_book'): lines.append(f"plan_reference: \"Plan Book {meta['plan_book']}, Page {meta.get('plan_page', '')}\"")
    lines.append("---")
    lines.append("")

    # Title & Header
    lines.append(f"# Legal Deed Description: {meta.get('lot', 'Parcel')} ({meta.get('town', 'Standish')})")
    lines.append("")
    lines.append(f"**Registry Reference:** Book {meta.get('book', '---')}, Page {meta.get('page', '---')} (Doc #{meta.get('doc_number', '---')})  ")
    if meta.get('grantee'): lines.append(f"**Grantee:** {meta['grantee']}  ")
    if meta.get('grantor'): lines.append(f"**Grantor:** {meta['grantor']}  ")
    lines.append("")

    # Sections for each parcel
    for p in parcels:
        lines.append(f"## {p['name']}")
        lines.append("")
        closure = p["closure"]
        if closure.get("acreage", 0.0) > 0.0:
            lines.append(f"**Calculated Area:** `{closure['acreage']} Acres` ({closure['area_sqft']:,} sq ft)  ")
        if closure.get('precision_ratio') != 'N/A':
            lines.append(f"**Traverse Closure:** `{closure['precision_ratio']}` (Misclosure: `{closure['misclosure_ft']} ft`) — **{'CLOSED' if closure['closed'] else 'OPEN'}**  ")
        lines.append("")

        lines.append("| Leg | Waypoint Label | Course / Bearing | Dist (ft) | Local X (ft) | Local Y (ft) | Monument | Description |")
        lines.append("| :-: | :--- | :--- | :-: | :-: | :-: | :--- | :--- |")

        for c in p["calls"]:
            dist_str = f"{c.get('distance_ft', 0.0):.2f}" if c.get('distance_ft') else "—"
            desc = c.get('description', '').replace('|', '/')
            if len(desc) > 50: desc = desc[:47] + "..."
            lines.append(
                f"| **{c['leg']}** | `{c['label']}` | {c.get('bearing_str', '—')} | {dist_str} | "
                f"{c['x_ft']:+.2f} | {c['y_ft']:+.2f} | **{c['monument_type']}** | {desc} |"
            )
        lines.append("")

    # Full Page Transcript
    lines.append("## Full Scanned OCR Transcripts")
    lines.append("")
    for p_num, p_txt in enumerate(pages_text, start=1):
        lines.append(f"### Page {p_num}")
        lines.append("```text")
        lines.append(p_txt.strip())
        lines.append("```")
        lines.append("")

    with open(filepath, 'w', encoding='utf-8') as f:
        f.write('\n'.join(lines))


def export_json(meta: Dict[str, Any], parcels: List[Dict[str, Any]],
                pages_text: List[str], filepath: str):
    """Write machine-readable JSON representation."""
    data = {
        "metadata": meta,
        "parcels": parcels,
        "transcripts": [
            {"page": idx, "text": txt} for idx, txt in enumerate(pages_text, start=1)
        ]
    }
    with open(filepath, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=2)


# =========================================================================
# Single File & Batch Pipeline Runner
# =========================================================================

def process_deed_pdf(pdf_path: str, out_dirs: Dict[str, str], ocr_reader: easyocr.Reader,
                     force: bool = False, verbose: bool = False) -> bool:
    """Process a single deed PDF into Markdown, JSON, and DXF."""
    filename = os.path.basename(pdf_path)
    clean_stem = re.sub(r'[^A-Za-z0-9_\-]+', '_', os.path.splitext(filename)[0]).strip('_')
    if len(clean_stem) > 70:
        clean_stem = clean_stem[:70].rstrip('_')

    md_path = os.path.join(out_dirs['md'], f"{clean_stem}.md")
    json_path = os.path.join(out_dirs['json'], f"{clean_stem}.json")
    dxf_path = os.path.join(out_dirs['dxf'], f"{clean_stem}.dxf")

    # Check if up to date
    if not force and os.path.exists(md_path) and os.path.exists(json_path) and os.path.exists(dxf_path):
        src_mtime = os.path.getmtime(pdf_path)
        if os.path.getmtime(md_path) > src_mtime and os.path.getmtime(json_path) > src_mtime:
            print(f"  [SKIPPED] Outputs already up to date: '{clean_stem}'")
            return False

    print(f"\n[PROCESSING] {filename}")
    t0 = time.time()

    reader = pypdf.PdfReader(pdf_path)
    num_pages = len(reader.pages)
    print(f"  Pages: {num_pages} | Extracting images and running OCR...")

    pages_text: List[str] = []

    for p_idx, page in enumerate(reader.pages):
        p_start = time.time()
        page_lines = []

        if page.images:
            try:
                img_data = page.images[0].data
                pil_img = Image.open(io.BytesIO(img_data)).convert('RGB')
                np_img = np.array(pil_img)
                page_lines = ocr_reader.readtext(np_img, detail=0)
            except Exception as e:
                print(f"    Page {p_idx+1}: Image OCR error ({e}), falling back to text extractor")
                raw = page.extract_text()
                if raw: page_lines = raw.splitlines()
        else:
            raw = page.extract_text()
            if raw: page_lines = raw.splitlines()

        p_time = time.time() - p_start
        clean_page_txt = normalize_ocr_text('\n'.join(page_lines))
        pages_text.append(clean_page_txt)

        print(f"    Page {p_idx+1}/{num_pages}: {len(page_lines):2d} lines detected ({p_time:.1f}s)")

    full_combined_text = normalize_ocr_text('\n\n'.join(pages_text))

    # Metadata & Legal Calls
    meta = extract_metadata(full_combined_text, filename)
    lot_prefix = meta.get('lot') if meta.get('lot') else "LOT"
    parcels = parse_all_parcels(full_combined_text, lot_prefix=lot_prefix)

    # Save Outputs
    export_markdown(meta, parcels, pages_text, md_path)
    export_json(meta, parcels, pages_text, json_path)
    export_dxf(parcels, dxf_path, lot_label=lot_prefix)

    elapsed = time.time() - t0
    total_wps = sum(len(p['calls']) for p in parcels)
    print(f"  [SAVED] Markdown:   {md_path}")
    print(f"  [SAVED] JSON:       {json_path}")
    print(f"  [SAVED] Master DXF: {dxf_path}")
    if len(parcels) > 1:
        base_stem, ext = os.path.splitext(dxf_path)
        for p in parcels:
            p_dxf = f"{base_stem}_P{p['parcel_id']}_{p['short_name']}{ext}"
            print(f"  [SAVED] Parcel DXF: {p_dxf}")
    p1_closure = parcels[0]['closure']['precision_ratio'] if parcels else "N/A"
    print(f"  [COMPLETED] Parsed {len(parcels)} parcel(s), {total_wps} total waypoints in {elapsed:.1f}s (P1 Closure: {p1_closure})")
    return True


def main():
    print(f"=== {__filename__} {__version__} ===")

    parser = argparse.ArgumentParser(
        description="Convert scanned deed PDFs to clean Markdown, structured JSON, and CAD DXF vector geometry."
    )
    parser.add_argument("-f", "--file", type=str, help="Process a single PDF deed file")
    parser.add_argument("-d", "--dir", type=str, default="reference",
                        help="Directory containing PDF deeds to process (default: reference)")
    parser.add_argument("-o", "--out", type=str, default="reference",
                        help="Base output directory (default: reference)")
    parser.add_argument("--flat", action="store_true",
                        help="Store all outputs in a single folder rather than subfolders (markdown, json, dxf)")
    parser.add_argument("-F", "--force", action="store_true",
                        help="Force reprocessing even if output files already exist")
    parser.add_argument("-g", "--gpu", action="store_true",
                        help="Enable GPU acceleration for EasyOCR if CUDA is available")
    parser.add_argument("-v", "--verbose", action="store_true",
                        help="Enable verbose output")

    args = parser.parse_args()

    base_out = os.path.abspath(args.out)
    if args.flat:
        out_dirs = {
            'md': base_out,
            'json': base_out,
            'dxf': base_out
        }
        os.makedirs(base_out, exist_ok=True)
    else:
        out_dirs = {
            'md': os.path.join(base_out, "markdown"),
            'json': os.path.join(base_out, "json"),
            'dxf': os.path.join(base_out, "dxf")
        }
        for p in out_dirs.values():
            os.makedirs(p, exist_ok=True)

    print(f"Initializing EasyOCR engine (GPU={'Enabled' if args.gpu else 'Disabled'})...")
    ocr_reader = easyocr.Reader(['en'], gpu=args.gpu)

    pdf_files: List[str] = []
    if args.file:
        if not os.path.exists(args.file):
            print(f"Error: File not found: {args.file}")
            sys.exit(1)
        pdf_files.append(os.path.abspath(args.file))
    else:
        in_dir = os.path.abspath(args.dir)
        if not os.path.exists(in_dir):
            print(f"Error: Directory not found: {in_dir}")
            sys.exit(1)
        for f in os.listdir(in_dir):
            if f.lower().endswith(".pdf"):
                pdf_files.append(os.path.join(in_dir, f))
        pdf_files.sort()

    total_files = len(pdf_files)
    if total_files == 0:
        print("No PDF deed files found to process.")
        sys.exit(0)

    print(f"Found {total_files} deed PDF(s) to process.")
    start_total = time.time()
    processed_cnt = 0

    for idx, pdf_path in enumerate(pdf_files, start=1):
        print(f"\n------------------------------------------------------------")
        print(f"[{idx}/{total_files}] ({(idx-1)/total_files*100:.1f}%) {os.path.basename(pdf_path)}")
        if process_deed_pdf(pdf_path, out_dirs, ocr_reader, force=args.force, verbose=args.verbose):
            processed_cnt += 1

    total_time = time.time() - start_total
    print(f"\n============================================================")
    print(f"Batch Processing Complete!")
    print(f"Total deeds checked: {total_files} | Processed: {processed_cnt} | Time: {total_time:.1f}s")
    print(f"Outputs saved to:")
    print(f"  Markdown: {out_dirs['md']}")
    print(f"  JSON:     {out_dirs['json']}")
    print(f"  CAD DXF:  {out_dirs['dxf']}")


if __name__ == "__main__":
    main()
