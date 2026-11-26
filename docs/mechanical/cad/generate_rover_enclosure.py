"""Generate parametric 3D STEP/STL CAD models and 2D DXF drawings for RTK Rover v2.2.5.

Architecture:
  - Back_Chassis_Frame: Open-frame rear plate carrying ALL modules (Waveshare 3.5B,
    underneath RTC Coin Cell + holder at Y = 64.0, Holybro UM982, E220 LoRa, Power
    Board w/ 2x2 mm holes & 5.0 mm M2 Ruthex pockets, 4-side slotted Speaker tower
    w/ 2 diagonal M2 Ruthex corner inserts, conical-base reinforced standoffs),
    full-width R6.0 Top L-Bulkhead w/ 50/50 split corner piers at Y = 173.2..180.0
    (Z-centered GX-12 w/ downward D-flat, equispaced E220 SMA w/ downward D-flat,
    UM982 SMAs), partial-height Left Headphone L-Tab (Y = 21.00, full-height inner
    1.50 mm pocket for 1.50 mm Ground-pin solder clearance), Right Button-Plunger
    Flange + low support rail + twin slide-in guide towers, split-Left/continuous-
    Bottom/Right 4.5 mm inner cover-capture rim, 6 open-top full-height M3 bosses
    (Z = -2.65 mm), and relocated horizontal under-Waveshare BAT_SLOT (25.0, 35.5).
  - Top_Cover: Clean lift-off cover with ZERO modules/connectors/plunger-combs;
    flush glass bezel, USB-C tunnel, MicroSD slot + scoop, speaker acoustic slots,
    4x UM982 LED light pipes + SWITCH pinhole, Option H1 Sun Shield U-groove,
    left-wall headphone L-tab notch, and matching lower-half top-corner wrap.
  - Standardized Option B2 Pod (10000 mAh Footprint + Capture Corner + 5000 mAh Adapter):
    One skeletonized Battery_Cradle (w/ integral Bottom-Left Capture Corner opposite
    the Bottom-Right cable exit) + solid Pole_Lid (twin 1/4"-20 Ruthex inserts @ 38.1 mm),
    plus removable skeletonized L-truss Battery_Adapter_5000 insert.
  - Pole_Tilt_Wedge_37_5: 90-deg rotated skeletonized open Warren-truss 37.5-deg
    laser-mount wedge straddling front/back edges of the 60 x 135 mm
    REF-laser_pole_mount.stp shelf (6.0 mm skirts, bottom 1/4"-20 Ruthex bore,
    45-deg lofted spine pocket, and pure round twin 1/4"-20 SHCS holes/tunnels).
  - Slim_Sun_Shield (Option H1): Ultra-thin 1.6 mm PETG slot-in tapered hood.
  - Plunger_Capture_Clip: Slide-in vertical capture bar dropping into Back_Chassis_Frame
    right-flange guide tracks (buttons remain functional w/ Top_Cover removed).
  - Button_Plungers: 3 captive round tactile button plungers (PWR, RESET, BOOT).
"""

import math
import os
from pathlib import Path

import cadquery as cq
import ezdxf

import rover_specs as S

__filename__ = os.path.basename(__file__)
__version__ = "v2.2.5"

CAD_DIR = Path(__file__).resolve().parent
STEP_DIR = CAD_DIR / "step"
STL_DIR = CAD_DIR / "stl"
DXF_DIR = CAD_DIR / "dxf"

REF_FILES = {
    "waveshare_body": STEP_DIR / "REF-Waveshare-ESP32-S3-Touch-LCD-3.5B.stp",
    "waveshare": STEP_DIR / "REF-ASM-Waveshare-esp32-s3-touch-lcd-v3.5B-with-right-facing-header.stp",
    "um982": STEP_DIR / "REF-holybro-um982.stp",
    "e220": STEP_DIR / "REF-e220-module.stp",
    "power": STEP_DIR / "REF_power.stp",
    "speaker": STEP_DIR / "REF-speaker.stp",
    "headphone": STEP_DIR / "REF-headphone-jack.stp",
    "coin_cell": STEP_DIR / "REF-coin-cell.stp",
    "coin_holder": STEP_DIR / "REF-coin-cell-holder.stp",
    "gx12": STEP_DIR / "REF_gx12.stp",
    "sma": STEP_DIR / "REF-sma-female-lora-connector.stp",
    "bat5000": STEP_DIR / "REF-lipo-battery-3.7V-5000mAh.stp",
    "bat10000": STEP_DIR / "REF-lipo-battery-3.7V-10000mAh.stp",
    "laser_mount": STEP_DIR / "REF-laser_pole_mount.stp",
}


# ==============================================================================
# Primitive & Support-Free Feature Helpers
# ==============================================================================
def make_rounded_box(width: float, height: float, depth: float, radius: float,
                     cx: float = 0.0, cy: float = 0.0, z_min: float = 0.0) -> cq.Workplane:
    """Create a vertical rounded-rectangle prism spanning [z_min, z_min + depth]."""
    wp = (
        cq.Workplane("XY")
        .workplane(offset=z_min)
        .center(cx, cy)
        .rect(width, height)
        .extrude(depth)
    )
    if radius > 0.05:
        wp = wp.edges("|Z").fillet(radius)
    return wp


def make_stadium_slot(cx: float, cy: float, width: float, height: float,
                      z_min: float, depth: float) -> cq.Workplane:
    """Create a vertical stadium/rounded-slot prism spanning [z_min, z_min + depth]."""
    r = min(width, height) / 2.0 - 0.05
    return make_rounded_box(width, height, depth, radius=max(0.2, r), cx=cx, cy=cy, z_min=z_min)


def make_csk_cone_through(cx: float, cy: float, z_bed: float, plate_thk: float,
                          hole_d: float, head_d: float, cone_from_bed: bool = True) -> cq.Workplane:
    """Create a 45-degree conical countersunk cutter (100% support-free FDM)."""
    cyl = (
        cq.Workplane("XY")
        .workplane(offset=z_bed - 0.5)
        .center(cx, cy)
        .circle(hole_d / 2.0)
        .extrude(plate_thk + 1.0)
    )
    cone_h = max(0.4, (head_d - hole_d) / 2.0)
    if cone_from_bed:
        cone = (
            cq.Workplane("XY")
            .workplane(offset=z_bed - 0.05)
            .center(cx, cy)
            .circle(head_d / 2.0 + 0.05)
            .workplane(offset=cone_h + 0.05)
            .circle(hole_d / 2.0)
            .loft(combine=True)
        )
    else:
        z_top = z_bed + plate_thk
        cone = (
            cq.Workplane("XY")
            .workplane(offset=z_top + 0.05)
            .center(cx, cy)
            .circle(head_d / 2.0 + 0.05)
            .workplane(offset=-(cone_h + 0.05))
            .circle(hole_d / 2.0)
            .loft(combine=True)
        )
    return cyl.union(cone)


def make_teardrop_y_bore(cx: float, y_start: float, y_len: float, z_ctr: float,
                         dia: float, top_flat_dist: float = None,
                         add_teardrop: bool = True, peak_at_plus_z: bool = True) -> cq.Workplane:
    """Create a horizontal Y-axis bore with top D-flat or 45-deg teardrop gable.

    peak_at_plus_z=True places the self-supporting top gable/flat toward +Z_asm
    (correct when printing back-down on Z_PLATE_OUTER). False places it toward -Z_asm
    (correct when printing face-down on Z_FRONT_OUTER).
    """
    r = dia / 2.0
    sign = 1.0 if peak_at_plus_z else -1.0
    bore = (
        cq.Workplane("XZ")
        .workplane(offset=-y_start)
        .center(cx, z_ctr)
        .circle(r)
        .extrude(-y_len)
    )
    if add_teardrop and top_flat_dist is None:
        tip = r * math.sqrt(2.0)
        pts = [
            (cx - r / math.sqrt(2.0), z_ctr + sign * r / math.sqrt(2.0)),
            (cx, z_ctr + sign * tip),
            (cx + r / math.sqrt(2.0), z_ctr + sign * r / math.sqrt(2.0)),
        ]
        peak = (
            cq.Workplane("XZ")
            .workplane(offset=-y_start)
            .polyline(pts)
            .close()
            .extrude(-y_len)
        )
        bore = bore.union(peak)
    if top_flat_dist is not None:
        flat_z = z_ctr + sign * (top_flat_dist - r)
        slab_h = 3.0
        z_min_clip = flat_z if peak_at_plus_z else (flat_z - slab_h)
        clip = (
            cq.Workplane("XY")
            .workplane(offset=z_min_clip)
            .center(cx, y_start + y_len / 2.0)
            .rect(dia + 2.0, y_len + 2.0)
            .extrude(slab_h)
        )
        bore = bore.cut(clip)
    return bore


def make_arch_x_cutout(x_start: float, x_len: float, cy: float, width_y: float,
                       z_min: float, z_max: float, roof_gable: float = 1.5,
                       peak_at_plus_z: bool = True) -> cq.Workplane:
    """Create a horizontal port cutout through a +-X wall with a 45-deg roof gable."""
    if peak_at_plus_z:
        pts = [
            (cy - width_y / 2.0, z_min),
            (cy + width_y / 2.0, z_min),
            (cy + width_y / 2.0, z_max),
            (cy, z_max + roof_gable),
            (cy - width_y / 2.0, z_max),
        ]
    else:
        pts = [
            (cy - width_y / 2.0, z_max),
            (cy + width_y / 2.0, z_max),
            (cy + width_y / 2.0, z_min),
            (cy, z_min - roof_gable),
            (cy - width_y / 2.0, z_min),
        ]
    return (
        cq.Workplane("YZ")
        .workplane(offset=x_start)
        .polyline(pts)
        .close()
        .extrude(x_len)
    )


def make_ruthex_boss(cx: float, cy: float, z_base: float, boss_h: float,
                     boss_od: float, hole_od: float, hole_depth: float,
                     open_at_top: bool = True) -> tuple[cq.Workplane, cq.Workplane]:
    """Return (boss_solid, insert_bore_cutter) for a Ruthex heat-set insert."""
    boss = (
        cq.Workplane("XY")
        .workplane(offset=z_base)
        .center(cx, cy)
        .circle(boss_od / 2.0)
        .extrude(boss_h)
    )
    if open_at_top:
        bore_z = z_base + boss_h - hole_depth
        bore = (
            cq.Workplane("XY")
            .workplane(offset=bore_z)
            .center(cx, cy)
            .circle(hole_od / 2.0)
            .extrude(hole_depth + 0.3)
        )
    else:
        bore = (
            cq.Workplane("XY")
            .workplane(offset=z_base - 0.3)
            .center(cx, cy)
            .circle(hole_od / 2.0)
            .extrude(hole_depth + 0.3)
        )
    return boss, bore


def make_stepped_cone_boss(cx: float, cy: float, z_base: float, z_floor: float,
                           z_top: float, top_od: float, base_od: float,
                           cone_h: float) -> cq.Workplane:
    """Build cylindrical boss up to z_top with a lower conical buttress above z_floor."""
    cyl = (
        cq.Workplane("XY")
        .workplane(offset=z_base)
        .center(cx, cy)
        .circle(top_od / 2.0)
        .extrude(z_top - z_base)
    )
    if cone_h <= 0.05 or base_od <= top_od + 0.05:
        return cyl
    if z_base < z_floor:
        skirt = (
            cq.Workplane("XY")
            .workplane(offset=z_base)
            .center(cx, cy)
            .circle(base_od / 2.0)
            .extrude(z_floor - z_base)
        )
        cyl = cyl.union(skirt)
    cone = (
        cq.Workplane("XY")
        .workplane(offset=z_floor)
        .center(cx, cy)
        .circle(base_od / 2.0)
        .workplane(offset=cone_h)
        .circle(top_od / 2.0)
        .loft(combine=True)
    )
    return cyl.union(cone)


# ==============================================================================
# Part 1: Back_Chassis_Frame (Prints flat on Z_PLATE_OUTER = -28.0 mm, Support-Free)
# ==============================================================================
def build_back_chassis_frame() -> cq.Workplane:
    """Build Back_Chassis_Frame carrying all modules, Top L-wall, and Right Plunger Flange."""
    case_cy = S.CASE_H / 2.0

    # 1. Base chassis back plate: full outer footprint [Z_PLATE_OUTER=-28.0 .. Z_PLATE_INNER=-25.0]
    plate = make_rounded_box(S.CASE_W, S.CASE_H, S.WALL_BACK, S.CORNER_R,
                             cx=0.0, cy=case_cy, z_min=S.Z_PLATE_OUTER)

    # 2. Integrated Full-Width R6.0 Top L-Wall Bulkhead + 50/50 Split Top-Corner Piers (Y = 173.2..180.0)
    # Wraps the upper 180 deg (+Y) of both top-corner M3 bosses cleanly at Z = -2.65 mm while keeping
    # overhead Dia 14.0 mm soldering-iron access completely unobstructed.
    full_wall_envelope = make_rounded_box(
        S.CASE_W, S.CASE_H, 25.35, S.CORNER_R,
        cx=0.0, cy=case_cy, z_min=S.Z_PLATE_OUTER,
    )
    top_band = (
        cq.Workplane("XY")
        .workplane(offset=S.Z_PLATE_OUTER - 0.5)
        .center(0.0, S.CASE_H - S.WALL_TOP / 2.0 + 0.5)
        .rect(S.CASE_W + 2.0, S.WALL_TOP + 1.0)
        .extrude(26.35)
    )
    wrap_w = (S.CASE_W / 2.0 + 1.0) - S.TOP_CORNER_WRAP_X_IN
    wrap_cx_abs = S.TOP_CORNER_WRAP_X_IN + wrap_w / 2.0
    wrap_h = (S.CASE_H - S.TOP_CORNER_SPLIT_Y) + 1.0
    wrap_cy = S.TOP_CORNER_SPLIT_Y + wrap_h / 2.0
    for sign in (-1.0, 1.0):
        corner_pier = (
            cq.Workplane("XY")
            .workplane(offset=S.Z_PLATE_OUTER - 0.5)
            .center(sign * wrap_cx_abs, wrap_cy)
            .rect(wrap_w, wrap_h)
            .extrude(26.35)
        )
        top_band = top_band.union(corner_pier)
    top_l_wall = full_wall_envelope.intersect(top_band)
    plate = plate.union(top_l_wall)

    # 3. Integrated Right Button-Plunger Flange (X = 57.0..60.0, Y = 20.0..52.0)
    flange_cy = 0.5 * (S.FLANGE_Y0 + S.FLANGE_Y1)
    flange_h = S.FLANGE_Y1 - S.FLANGE_Y0
    right_flange = (
        cq.Workplane("XY")
        .workplane(offset=S.Z_PLATE_OUTER)
        .center(S.CASE_W / 2.0 - S.WALL_SIDE / 2.0, flange_cy)
        .rect(S.WALL_SIDE, flange_h)
        .extrude(25.35)
    )
    plate = plate.union(right_flange)

    # Plunger outer stop block (X=56.0..57.0) + low rail (X=48.58..56.5, Z=-25.0..-11.7) + twin guide towers
    blk_cx = 0.5 * (S.PLUNGER_BLK_X0 + S.PLUNGER_BLK_X1)
    blk_w = S.PLUNGER_BLK_X1 - S.PLUNGER_BLK_X0
    blk_cy = 0.5 * (S.PLUNGER_BLK_Y0 + S.PLUNGER_BLK_Y1)
    blk_h = S.PLUNGER_BLK_Y1 - S.PLUNGER_BLK_Y0
    plunger_blk = (
        cq.Workplane("XY")
        .workplane(offset=S.Z_PLATE_INNER - 0.5)
        .center(blk_cx + 0.25, blk_cy)
        .rect(blk_w + 0.5, blk_h)
        .extrude(S.PLUNGER_BLK_Z_TOP - S.Z_PLATE_INNER + 0.5)
    )
    rail_cx = 0.5 * (S.PLUNGER_RAIL_X0 + S.PLUNGER_BLK_X0 + 0.5)
    rail_w = (S.PLUNGER_BLK_X0 + 0.5) - S.PLUNGER_RAIL_X0
    plunger_rail = (
        cq.Workplane("XY")
        .workplane(offset=S.Z_PLATE_INNER - 0.5)
        .center(rail_cx, blk_cy)
        .rect(rail_w, blk_h)
        .extrude(S.PLUNGER_RAIL_Z_TOP - S.Z_PLATE_INNER + 0.5)
    )
    plate = plate.union(plunger_blk).union(plunger_rail)

    trk_x0 = S.PLUNGER_U_X0 - S.PLUNGER_CLIP_CLEAR - S.PLUNGER_TRACK_LIP
    trk_x1 = S.PLUNGER_U_X1 + S.PLUNGER_CLIP_CLEAR + S.PLUNGER_TRACK_LIP
    for y0, y1 in ((S.FLANGE_Y0, S.PLUNGER_TRACK_BOT_Y1), (S.PLUNGER_TRACK_TOP_Y0, S.FLANGE_Y1)):
        tower = (
            cq.Workplane("XY")
            .workplane(offset=S.Z_PLATE_INNER - 0.5)
            .center(0.5 * (trk_x0 + trk_x1), 0.5 * (y0 + y1))
            .rect(trk_x1 - trk_x0, y1 - y0)
            .extrude(S.PLUNGER_CLIP_Z_TOP - S.Z_PLATE_INNER + 0.5)
        )
        plate = plate.union(tower)
    slot_x0 = S.PLUNGER_U_X0 - S.PLUNGER_CLIP_CLEAR
    slot_x1 = S.PLUNGER_U_X1 + S.PLUNGER_CLIP_CLEAR
    slot_y0 = S.FLANGE_Y0 + S.PLUNGER_TRACK_END_WALL
    slot_y1 = S.FLANGE_Y1 - S.PLUNGER_TRACK_END_WALL
    clip_slot = (
        cq.Workplane("XY")
        .workplane(offset=S.PLUNGER_RAIL_Z_TOP)
        .center(0.5 * (slot_x0 + slot_x1), 0.5 * (slot_y0 + slot_y1))
        .rect(slot_x1 - slot_x0, slot_y1 - slot_y0)
        .extrude(S.PLUNGER_CLIP_Z_TOP - S.PLUNGER_RAIL_Z_TOP + 0.5)
    )
    plate = plate.cut(clip_slot)

    # 3b. Partial-Height Left Headphone L-Tab (X=[-60.00, -57.00], Y=[11.75, 30.25], Z=[-28.0, -13.50])
    # Mounts REF-headphone-jack.stp on Back_Chassis_Frame so all connectors remain bench-serviceable.
    hp_tab = (
        cq.Workplane("XY")
        .workplane(offset=S.Z_PLATE_OUTER + 0.5)
        .center(-S.CASE_W / 2.0 + S.WALL_SIDE / 2.0, S.HP_CY)
        .rect(S.WALL_SIDE, S.HP_TAB_W)
        .extrude(S.HP_TAB_Z_TOP - S.Z_PLATE_OUTER - 0.5)
    )
    plate = plate.union(hp_tab)

    # 4. Continuous Inner Cover-Capture Rim (split Left around L-Tab, continuous Bottom, Right; Z = -25.0..-20.5)
    # Braces Top_Cover's open perimeter lip against inward flex across all non-bulkhead edges.
    x_rim_ctr = (S.CASE_W / 2.0 - S.WALL_SIDE - S.RIM_CLEAR) - S.RIM_THK / 2.0
    y_rim_ctr = (S.WALL_BOT + S.RIM_CLEAR) + S.RIM_THK / 2.0
    y_bot_m3 = S.COVER_M3_POINTS[0][1]
    y_top_m3 = S.COVER_M3_POINTS[4][1]
    x_bot_m3 = abs(S.COVER_M3_POINTS[0][0])
    hp_rim_b = S.HP_CY - S.HP_TAB_W / 2.0 - S.NOTCH_CLEAR
    hp_rim_t = S.HP_CY + S.HP_TAB_W / 2.0 + S.NOTCH_CLEAR
    rim_segs = [
        (-x_rim_ctr, 0.5 * (y_bot_m3 + hp_rim_b), S.RIM_THK, hp_rim_b - y_bot_m3),                               # Left below L-Tab (6.8..11.55)
        (-x_rim_ctr, 0.5 * (hp_rim_t + y_top_m3), S.RIM_THK, y_top_m3 - hp_rim_t),                               # Left above L-Tab (30.45..173.2)
        (0.0,        y_rim_ctr,                   2.0 * x_bot_m3, S.RIM_THK),                                    # Bottom continuous under USB-C (-54.2..+54.2)
        ( x_rim_ctr, 0.5 * (y_bot_m3 + S.PLUNGER_BLK_Y0 + 0.5), S.RIM_THK, (S.PLUNGER_BLK_Y0 + 0.5) - y_bot_m3), # Right lower (6.8..22.5)
        ( x_rim_ctr, 0.5 * (S.PLUNGER_BLK_Y1 - 0.5 + y_top_m3), S.RIM_THK, y_top_m3 - (S.PLUNGER_BLK_Y1 - 0.5)), # Right upper (49.5..173.2)
    ]
    for rx, ry, rw, rh in rim_segs:
        rim = (
            cq.Workplane("XY")
            .workplane(offset=S.Z_PLATE_INNER - 0.5)
            .center(rx, ry)
            .rect(rw, rh)
            .extrude(S.RIM_H + 0.5)
        )
        plate = plate.union(rim)

    # 5. Symmetric 6 x Front-Entry M3 Closure Bosses (full-height to Z=-2.65, 0.25 mm below Top_Cover inner face)
    # All 6 bosses stand flush with Top L-Wall / Right Flange for unobstructed top-level soldering-iron access.
    boss_z_top = S.COVER_BOSS_Z_TOP
    csk_holes = []
    for mx, my in S.COVER_M3_POINTS:
        b, bore = make_ruthex_boss(
            mx, my, z_base=S.Z_PLATE_OUTER + 0.5,
            boss_h=(boss_z_top - S.Z_PLATE_OUTER - 0.5),
            boss_od=S.RUTHEX_M3_BOSS_OD, hole_od=S.RUTHEX_M3_OD,
            hole_depth=S.RUTHEX_M3_DEPTH + 1.3, open_at_top=True,
        )
        plate = plate.union(b)
        csk_holes.append(bore)
    for bore in csk_holes:
        plate = plate.cut(bore)

    # 6. Shared 4 x M3 Option B2 Rear Pod Bosses (Ruthex inserts open to Z_PLATE_OUTER=-28.0)
    pod_bores = []
    for px, py in S.POD_M3_POINTS:
        b, bore = make_ruthex_boss(
            px, py, z_base=S.Z_PLATE_OUTER,
            boss_h=8.0,                 # Reaches Z = -20.0 inside cavity (1.1 mm solid top cap!)
            boss_od=S.RUTHEX_M3_BOSS_OD, hole_od=S.RUTHEX_M3_OD,
            hole_depth=S.RUTHEX_M3_DEPTH + 0.2, open_at_top=False,
        )
        plate = plate.union(b)
        pod_bores.append(bore)
    for bore in pod_bores:
        plate = plate.cut(bore)

    # 7. Waveshare 3.5B Pillars (Conical base OD 9.6 -> shaft OD 6.0 up to -11.50, rear M2 CSK pass-throughs)
    ws_bores = []
    for sx in (-S.WS_STANDOFF_DX / 2.0, S.WS_STANDOFF_DX / 2.0):
        for sy in (-S.WS_STANDOFF_DY / 2.0, S.WS_STANDOFF_DY / 2.0):
            px, py = S.WS_CX + sx, S.WS_CY + sy
            pil = make_stepped_cone_boss(
                px, py, z_base=S.Z_PLATE_INNER - 0.5, z_floor=S.Z_PLATE_INNER,
                z_top=S.WS_STANDOFF_Z_END, top_od=S.WS_STANDOFF_OD,
                base_od=S.WS_STANDOFF_BASE_OD, cone_h=S.WS_STANDOFF_CONE_H,
            )
            plate = plate.union(pil)
            ws_bores.append(make_csk_cone_through(
                px, py, S.Z_PLATE_OUTER, S.WS_STANDOFF_Z_END - S.Z_PLATE_OUTER,
                S.HOLE_CLEAR_M2, 4.6, cone_from_bed=True
            ))
    for bore in ws_bores:
        plate = plate.cut(bore)

    # 8. RTC Coin Cell C-Clip Cradle Underneath Waveshare (exact match to REF-coin-cell-holder.stp)
    # ID 22.5 mm, OD 26.5 mm, H 4.0 mm above floor (-25.0..-21.0), C-opening facing -Y for wires
    rtc_ring = (
        cq.Workplane("XY")
        .workplane(offset=S.Z_PLATE_INNER - 0.4)
        .center(S.RTC_CX, S.RTC_CY + 0.12)
        .circle(13.25)
        .extrude(4.4)
    )
    rtc_inner = (
        cq.Workplane("XY")
        .workplane(offset=S.Z_PLATE_INNER - 0.6)
        .center(S.RTC_CX, S.RTC_CY + 0.12)
        .circle(11.25)
        .extrude(4.8)
    )
    rtc_wire_slot = (
        cq.Workplane("XY")
        .workplane(offset=S.Z_PLATE_INNER - 0.6)
        .center(S.RTC_CX, S.RTC_CY - 10.2)
        .rect(22.0, 9.6)
        .extrude(4.8)
    )
    rtc_holder = rtc_ring.cut(rtc_inner).cut(rtc_wire_slot)
    plate = plate.union(rtc_holder)

    # 9. Power Board Standoffs (4 x M2 Ruthex 16x76 mm w/ OD 8.2->6.2 mm root flare, full 5.0 mm pocket)
    pb_top_z = S.Z_PLATE_INNER + S.PB_STANDOFF_H  # -22.45
    pb_bores = []
    for dx in (-S.PB_HOLE_DX / 2.0, S.PB_HOLE_DX / 2.0):
        for dy in (-S.PB_HOLE_DY / 2.0, S.PB_HOLE_DY / 2.0):
            px, py = S.PB_CX + dx, S.PB_CY + dy
            b = make_stepped_cone_boss(
                px, py, z_base=S.Z_PLATE_OUTER + 0.5, z_floor=S.Z_PLATE_INNER,
                z_top=pb_top_z, top_od=S.RUTHEX_M2_BOSS_OD,
                base_od=S.PB_STANDOFF_BASE_OD, cone_h=S.PB_STANDOFF_CONE_H,
            )
            bore = (
                cq.Workplane("XY")
                .workplane(offset=pb_top_z - S.RUTHEX_M2_DEPTH)
                .center(px, py)
                .circle(S.RUTHEX_M2_OD / 2.0)
                .extrude(S.RUTHEX_M2_DEPTH + 0.3)
            )
            plate = plate.union(b)
            pb_bores.append(bore)
    for bore in pb_bores:
        plate = plate.cut(bore)

    # 10. E220 LoRa Standoffs (4 x M2 Ruthex w/ OD 9.2->6.2 mm cone flare, top -16.50, clears POD M3 by 0.50 mm)
    e2_top_z = S.Z_PLATE_INNER + S.E220_STANDOFF_H  # -16.50
    e2_bores = []
    for dx in (-S.E220_HOLE_DX / 2.0, S.E220_HOLE_DX / 2.0):
        for dy in (-S.E220_HOLE_DY / 2.0, S.E220_HOLE_DY / 2.0):
            px, py = S.E220_CX + dx, S.E220_CY + dy
            b = make_stepped_cone_boss(
                px, py, z_base=S.Z_PLATE_OUTER + 0.5, z_floor=S.Z_PLATE_INNER,
                z_top=e2_top_z, top_od=S.RUTHEX_M2_BOSS_OD,
                base_od=S.E220_STANDOFF_BASE_OD, cone_h=S.E220_STANDOFF_CONE_H,
            )
            bore = (
                cq.Workplane("XY")
                .workplane(offset=e2_top_z - S.RUTHEX_M2_DEPTH)
                .center(px, py)
                .circle(S.RUTHEX_M2_OD / 2.0)
                .extrude(S.RUTHEX_M2_DEPTH + 0.3)
            )
            plate = plate.union(b)
            e2_bores.append(bore)
    for bore in e2_bores:
        plate = plate.cut(bore)

    # 11. Holybro UM982 Standoffs (OD 9.0->5.4 mm conical flare up to Z = -17.20, recessed rear M2 CSK)
    z_um_csk = S.Z_PLATE_OUTER + S.UM_CSK_RECESS
    um_bores = []
    for dx in S.UM_M2_X_OFFS:
        for dy in S.UM_M2_Y_OFFS:
            px, py = S.UM_LEFT_X + dx, S.UM_BOT_Y + dy
            pil = make_stepped_cone_boss(
                px, py, z_base=S.Z_PLATE_INNER - 0.5, z_floor=S.Z_PLATE_INNER,
                z_top=S.UM_BACK_Z, top_od=S.UM_STANDOFF_OD,
                base_od=S.UM_STANDOFF_BASE_OD, cone_h=S.UM_STANDOFF_CONE_H,
            )
            plate = plate.union(pil)
            cbore = (
                cq.Workplane("XY")
                .workplane(offset=S.Z_PLATE_OUTER - 0.5)
                .center(px, py)
                .circle(2.35)
                .extrude(S.UM_CSK_RECESS + 0.46)
            )
            csk = make_csk_cone_through(
                px, py, z_um_csk, S.UM_BACK_Z - z_um_csk,
                S.HOLE_CLEAR_M2, 4.6, cone_from_bed=True
            )
            um_bores.append(cbore.union(csk))
    for bore in um_bores:
        plate = plate.cut(bore)

    # 12. 4-Side Slotted Speaker Corner-Pillar Tower + 2 x Diagonal M2 Ruthex Inserts (Z = -13.50)
    # 15.0 mm wide top-to-bottom slots on all 4 walls leave 4 rigid 8.3 mm L-pillars, saving material
    # while opening the -X side for the headphone jack and +Y side for speaker wires.
    spk_ped_top = S.Z_PLATE_INNER + S.SPK_PEDESTAL_H  # -13.50
    spk_seat_w = S.SPK_W + 0.4
    spk_seat_h = S.SPK_H + 0.4
    spk_outer = make_rounded_box(
        spk_seat_w + 2.0 * S.SPK_WALL,
        spk_seat_h + 2.0 * S.SPK_WALL,
        S.SPK_PEDESTAL_H + 4.5,
        radius=2.5,
        cx=S.SPK_CX, cy=S.SPK_CY, z_min=S.Z_PLATE_INNER - 0.5,
    )
    spk_seat_pocket = (
        cq.Workplane("XY")
        .workplane(offset=spk_ped_top)
        .center(S.SPK_CX, S.SPK_CY)
        .rect(spk_seat_w, spk_seat_h)
        .extrude(5.0)
    )
    spk_slot_ns = (
        cq.Workplane("XY")
        .workplane(offset=S.Z_PLATE_INNER)
        .center(S.SPK_CX, S.SPK_CY)
        .rect(S.SPK_SLOT_W, spk_seat_h + 2.0 * S.SPK_WALL + 2.0)
        .extrude(S.SPK_PEDESTAL_H + 5.5)
    )
    spk_slot_ew = (
        cq.Workplane("XY")
        .workplane(offset=S.Z_PLATE_INNER)
        .center(S.SPK_CX, S.SPK_CY)
        .rect(spk_seat_w + 2.0 * S.SPK_WALL + 2.0, S.SPK_SLOT_W)
        .extrude(S.SPK_PEDESTAL_H + 5.5)
    )
    spk_center_open = make_rounded_box(
        spk_seat_w - 2.0 * S.SPK_RIDGE_W,
        spk_seat_h - 2.0 * S.SPK_RIDGE_W,
        S.SPK_PEDESTAL_H + 5.0,
        radius=2.0,
        cx=S.SPK_CX, cy=S.SPK_CY, z_min=S.Z_PLATE_INNER,
    )
    spk_cradle = (
        spk_outer
        .cut(spk_seat_pocket)
        .cut(spk_slot_ns)
        .cut(spk_slot_ew)
        .cut(spk_center_open)
    )
    spk_m2_bores = []
    for sx_m2, sy_m2 in S.SPK_M2_POINTS:
        m2_boss = (
            cq.Workplane("XY")
            .workplane(offset=S.Z_PLATE_INNER - 0.5)
            .center(sx_m2, sy_m2)
            .circle(S.RUTHEX_M2_BOSS_OD / 2.0)
            .extrude(spk_ped_top - (S.Z_PLATE_INNER - 0.5))
        )
        spk_cradle = spk_cradle.union(m2_boss)
        spk_m2_bores.append(
            cq.Workplane("XY")
            .workplane(offset=spk_ped_top - S.RUTHEX_M2_DEPTH)
            .center(sx_m2, sy_m2)
            .circle(S.RUTHEX_M2_OD / 2.0)
            .extrude(S.RUTHEX_M2_DEPTH + 0.3)
        )
    plate = plate.union(spk_cradle)
    for bore in spk_m2_bores:
        plate = plate.cut(bore)

    # 13. Cutouts on Back Plate & Integral L-Wall & Right Flange
    # 13a. Battery JST-PH wire pass-through slot
    bat_slot = make_stadium_slot(S.BAT_SLOT_CX, S.BAT_SLOT_CY,
                                 S.BAT_SLOT_W, S.BAT_SLOT_H,
                                 S.Z_PLATE_OUTER - 0.5, S.WALL_BACK + 1.0)
    plate = plate.cut(bat_slot)

    # 13b. Top L-Wall Bulkhead Cutouts (UM982 local 2.0 mm roof web, GX-12, E220 SMA, UM982 SMAs)
    # NOTE: peak_at_plus_z=True because Back_Chassis_Frame prints back-down on Z=-28!
    um_rec_x0 = S.UM_LEFT_X - 0.30
    um_rec_x1 = S.UM_LEFT_X + S.UM_W + 0.30
    um_recess = (
        cq.Workplane("XY")
        .workplane(offset=S.Z_PLATE_INNER)
        .center(0.5 * (um_rec_x0 + um_rec_x1), 177.25)
        .rect(um_rec_x1 - um_rec_x0, 1.50)
        .extrude(25.0)
    )
    plate = plate.cut(um_recess)

    # GX-12 M12 bore w/ downward (-Z_asm) anti-rotation D-flat at Z=-19.15 + inner nut relief (2.6 mm web Y = 177.4..180.0)
    gx_bore = make_teardrop_y_bore(
        S.GX12_X, y_start=176.5, y_len=4.5, z_ctr=S.GX12_Z,
        dia=S.GX12_HOLE_DIA, top_flat_dist=S.GX12_FLAT,
        add_teardrop=False, peak_at_plus_z=False,
    )
    gx_rel = make_teardrop_y_bore(
        S.GX12_X, y_start=174.0, y_len=180.0 - S.GX12_WALL - 174.0, z_ctr=S.GX12_Z,
        dia=S.GX12_NUT_RECESS_DIA, add_teardrop=False
    )
    plate = plate.cut(gx_bore).cut(gx_rel)

    # E220 SMA bore w/ downward (-Z_asm) anti-rotation D-flat at Z=-13.930 (leaves 2.6 mm web Y = 177.4..180.0)
    e2_sma_bore = make_teardrop_y_bore(
        S.E220_SMA_X, y_start=176.5, y_len=4.5, z_ctr=S.E220_SMA_Z,
        dia=S.E220_SMA_DIA, top_flat_dist=S.E220_SMA_FLAT,
        add_teardrop=False, peak_at_plus_z=False
    )
    e2_sma_rel = make_teardrop_y_bore(
        S.E220_SMA_X, y_start=174.0, y_len=180.0 - S.E220_SMA_WALL - 174.0, z_ctr=S.E220_SMA_Z,
        dia=11.5, add_teardrop=False
    )
    plate = plate.cut(e2_sma_bore).cut(e2_sma_rel)

    # UM982 dual SMA bores (pure round Dia 10.5 mm access holes through Y=178..180; counterbore Y=176..178)
    for sx in (S.UM_SMA_SLAVE_X, S.UM_SMA_MASTER_X):
        um_bore = make_teardrop_y_bore(
            sx, y_start=177.5, y_len=3.5, z_ctr=S.UM_SMA_Z,
            dia=S.UM_SMA_ACCESS_DIA, add_teardrop=False
        )
        um_pocket = make_teardrop_y_bore(
            sx, y_start=175.0, y_len=3.0, z_ctr=S.UM_SMA_Z,
            dia=11.0, add_teardrop=False
        )
        plate = plate.cut(um_bore).cut(um_pocket)

    # 13c. Headphone Jack M6 Bore + Full-Height Inner 1.50 mm Anti-Rotation Recess (Left L-Tab)
    # Cuts straight to Z=HP_TAB_Z_TOP (-13.50) w/ zero overhang, opening 1.50 mm behind Ground solder pin.
    hp_bore = (
        cq.Workplane("YZ")
        .workplane(offset=-S.CASE_W / 2.0 - 0.5)
        .center(S.HP_CY, S.HP_Z_CTR)
        .circle(S.HP_BORE_DIA / 2.0)
        .extrude(S.WALL_SIDE + 1.0)
    )
    x_in = -S.CASE_W / 2.0 + S.HP_WALL
    x_back = -S.CASE_W / 2.0 + S.WALL_SIDE + 0.5
    hp_recess = (
        cq.Workplane("XY")
        .workplane(offset=S.Z_PLATE_INNER)
        .center(0.5 * (x_in + x_back), S.HP_CY)
        .rect(x_back - x_in, S.HP_RECESS_W)
        .extrude((S.HP_TAB_Z_TOP + 0.5) - S.Z_PLATE_INNER)
    )
    plate = plate.cut(hp_bore).cut(hp_recess)

    # 13d. Right Flange Tactile Button Plunger Outer Bearing Bores (pure round Dia 4.60 mm, open to -X)
    for by in S.WS_BTN_Y_LIST:
        outer_bore = (
            cq.Workplane("YZ")
            .workplane(offset=55.5)
            .center(by, S.WS_BTN_Z_CTR)
            .circle(2.30)
            .extrude(5.3)
        )
        plate = plate.cut(outer_bore)

    return plate


# ==============================================================================
# Part 2: Top_Cover (Prints face-down on Z = 0.0 mm, Support-Free, Zero Modules)
# ==============================================================================
def build_top_cover() -> cq.Workplane:
    """Build Top_Cover (clean shell with glass opening, USB-C, speaker slots, LEDs, U-groove)."""
    case_cy = S.CASE_H / 2.0
    cover_depth = abs(S.Z_PLATE_INNER) - 0.15  # 24.85 mm (leaves 0.15 mm clamping gap above Back Plate)
    z_cover_bot = -cover_depth                 # -24.85

    outer = make_rounded_box(S.CASE_W, S.CASE_H, cover_depth, S.CORNER_R,
                             cx=0.0, cy=case_cy, z_min=z_cover_bot)

    # Hollow interior cavity below faceplate Z_FRONT_INNER (-2.4)
    in_w = S.CASE_W - 2.0 * S.WALL_SIDE
    in_h = S.CASE_H - S.WALL_BOT - S.WALL_TOP
    cavity = make_rounded_box(in_w, in_h, cover_depth - S.WALL_FRONT + 0.5,
                              max(1.5, S.CORNER_R - S.WALL_SIDE),
                              cx=0.0, cy=case_cy, z_min=z_cover_bot - 0.5)
    cover = outer.cut(cavity)

    # Notch out full-width Top L-Wall crown & upper split corners (Y = 173.0..180.5) behind Z_FRONT_INNER (-2.4)
    # so Top_Cover wraps the lower 180 deg of both top-corner posts (Y <= 173.0) and lifts STRAIGHT UP!
    notch_y0 = S.TOP_CORNER_SPLIT_Y - S.NOTCH_CLEAR
    notch_y1 = S.CASE_H + 0.5
    top_notch = (
        cq.Workplane("XY")
        .workplane(offset=z_cover_bot - 0.5)
        .center(0.0, 0.5 * (notch_y0 + notch_y1))
        .rect(S.CASE_W + 2.0, notch_y1 - notch_y0)
        .extrude(cover_depth - S.WALL_FRONT + 0.5)
    )
    cover = cover.cut(top_notch)

    # Notch out Right Button-Plunger Flange window (Y = 19.8..52.2)
    right_notch = (
        cq.Workplane("XY")
        .workplane(offset=z_cover_bot - 0.5)
        .center(S.CASE_W / 2.0 - 1.2, 0.5 * (S.FLANGE_Y0 + S.FLANGE_Y1))
        .rect(S.WALL_SIDE + 2.0, (S.FLANGE_Y1 - S.FLANGE_Y0) + 2.0 * S.NOTCH_CLEAR)
        .extrude(cover_depth - S.WALL_FRONT + 0.5)
    )
    cover = cover.cut(right_notch)

    # Notch out partial-height Left Headphone L-Tab window (X=[-60.5, -56.5], Y=[11.55, 30.45], Z=[-24.85, -13.30])
    # Keeps a continuous 13.30 mm front left skirt & unbroken bottom wall (100% support-free)!
    left_hp_notch_top = S.HP_TAB_Z_TOP + S.NOTCH_CLEAR
    left_hp_notch = (
        cq.Workplane("XY")
        .workplane(offset=z_cover_bot - 0.5)
        .center(-S.CASE_W / 2.0 + 0.5 * S.WALL_SIDE, S.HP_CY)
        .rect(S.WALL_SIDE + 1.0, S.HP_TAB_W + 2.0 * S.NOTCH_CLEAR)
        .extrude((left_hp_notch_top - z_cover_bot) + 0.5)
    )
    cover = cover.cut(left_hp_notch)

    # Reinforcing bezel backing rib behind the Option H1 Sun Shield U-groove (Z = -2.4..-3.4)
    u_cx = 0.5 * (S.SHIELD_U_X_LEFT + S.SHIELD_U_X_RIGHT)
    u_w = S.SHIELD_U_X_RIGHT - S.SHIELD_U_X_LEFT
    for gx, gy, gw, gh in [
        (S.SHIELD_U_X_LEFT, 0.5 * (S.SHIELD_U_Y_BOT + S.SHIELD_U_Y_TOP), 4.0, (S.SHIELD_U_Y_TOP - S.SHIELD_U_Y_BOT) + 2.5),
        (S.SHIELD_U_X_RIGHT, 0.5 * (S.SHIELD_U_Y_BOT + S.SHIELD_U_Y_TOP), 4.0, (S.SHIELD_U_Y_TOP - S.SHIELD_U_Y_BOT) + 2.5),
        (u_cx, S.SHIELD_U_Y_TOP, u_w + 4.0, 4.0),
    ]:
        rib = (
            cq.Workplane("XY")
            .workplane(offset=-3.4)
            .center(gx, gy)
            .rect(gw, gh)
            .extrude(1.4)
        )
        cover = cover.union(rib)

    # Cut Option H1 Recessed U-Groove (1.90 mm wide x 1.80 mm deep from Z = 0.0)
    gw = S.SHIELD_GROOVE_W
    left_grv = (
        cq.Workplane("XY")
        .workplane(offset=-S.SHIELD_GROOVE_D)
        .center(S.SHIELD_U_X_LEFT, 0.5 * (S.SHIELD_U_Y_BOT + S.SHIELD_U_Y_TOP))
        .rect(gw, (S.SHIELD_U_Y_TOP - S.SHIELD_U_Y_BOT) + gw)
        .extrude(S.SHIELD_GROOVE_D + 0.2)
    )
    right_grv = (
        cq.Workplane("XY")
        .workplane(offset=-S.SHIELD_GROOVE_D)
        .center(S.SHIELD_U_X_RIGHT, 0.5 * (S.SHIELD_U_Y_BOT + S.SHIELD_U_Y_TOP))
        .rect(gw, (S.SHIELD_U_Y_TOP - S.SHIELD_U_Y_BOT) + gw)
        .extrude(S.SHIELD_GROOVE_D + 0.2)
    )
    top_grv = (
        cq.Workplane("XY")
        .workplane(offset=-S.SHIELD_GROOVE_D)
        .center(u_cx, S.SHIELD_U_Y_TOP)
        .rect(u_w + gw, gw)
        .extrude(S.SHIELD_GROOVE_D + 0.2)
    )
    cover = cover.cut(left_grv).cut(right_grv).cut(top_grv)

    # 6 x Symmetric Front-Entry M3 Boss Wall Relief Pockets (Z = -24.85..Z_FRONT_INNER=-2.40) + Faceplate CSK
    # All 6 full-height Back_Chassis_Frame bosses seat cleanly up behind the 2.40 mm faceplate (M3 x 8/10 CSK).
    for mx, my in S.COVER_M3_POINTS:
        boss_pocket = (
            cq.Workplane("XY")
            .workplane(offset=z_cover_bot - 0.5)
            .center(mx, my)
            .circle(4.10)
            .extrude((abs(z_cover_bot) - S.WALL_FRONT) + 0.5)
        )
        cover = cover.cut(boss_pocket)

    for mx, my in S.COVER_M3_POINTS:
        csk = make_csk_cone_through(mx, my, z_bed=S.Z_FRONT_INNER, plate_thk=S.WALL_FRONT,
                                    hole_d=S.HOLE_CLEAR_M3, head_d=6.2, cone_from_bed=False)
        cover = cover.cut(csk)

    # Waveshare 3.5B Flush Glass Opening (cut cleanly through faceplate Z = -4.0..0.2)
    ws_open = make_rounded_box(
        S.WS_GLASS_W + 2.0 * S.FIT_CLEAR,
        S.WS_GLASS_H + 2.0 * S.FIT_CLEAR,
        4.5,
        S.WS_GLASS_R + S.FIT_CLEAR,
        cx=S.WS_CX, cy=S.WS_CY, z_min=-4.0,
    )
    cover = cover.cut(ws_open)

    # USB-C Bottom Tunnel (prints face-down -> gable at -Z_asm side!)
    usbc_z0 = S.WS_USBC_Z_CTR + S.WS_USBC_H_OPEN / 2.0
    usbc_z1 = S.WS_USBC_Z_CTR - S.WS_USBC_H_OPEN / 2.0
    usbc_pts = [
        (S.WS_CX - S.WS_USBC_W / 2.0, usbc_z0),
        (S.WS_CX + S.WS_USBC_W / 2.0, usbc_z0),
        (S.WS_CX + S.WS_USBC_W / 2.0, usbc_z1),
        (S.WS_CX, usbc_z1 - 2.0),
        (S.WS_CX - S.WS_USBC_W / 2.0, usbc_z1),
    ]
    usbc_cut = (
        cq.Workplane("XZ")
        .workplane(offset=0.5)
        .polyline(usbc_pts)
        .close()
        .extrude(-(S.WALL_BOT + 1.5))
    )
    cover = cover.cut(usbc_cut)

    # MicroSD slot cutout + external finger scoop on Right Wall (prints face-down -> gable at -Z_asm!)
    sd_cy = 0.5 * (S.WS_SD_Y0 + S.WS_SD_Y1)
    sd_h_y = S.WS_SD_Y1 - S.WS_SD_Y0
    sd_cut = make_arch_x_cutout(
        56.2, 4.5, sd_cy, width_y=sd_h_y,
        z_min=S.WS_SD_Z_CTR - 2.6, z_max=S.WS_SD_Z_CTR + 2.6,
        roof_gable=1.6, peak_at_plus_z=False
    )
    sd_scoop = make_arch_x_cutout(
        58.8, 2.0, sd_cy, width_y=sd_h_y + 3.0,
        z_min=S.WS_SD_Z_CTR - 3.5, z_max=S.WS_SD_Z_CTR + 3.5,
        roof_gable=2.0, peak_at_plus_z=False
    )
    cover = cover.cut(sd_cut).cut(sd_scoop)

    # Speaker 5 x Acoustic Grille Slots over cone (-33.2, 21.0)
    for dy in (-8.0, -4.0, 0.0, 4.0, 8.0):
        slot = make_stadium_slot(S.SPK_CX, S.SPK_CY + dy, 18.0, 2.0, z_min=-3.0, depth=3.5)
        cover = cover.cut(slot)

    # 4 x UM982 Status LED Light-Pipe Bores (Dia 1.95 mm) + SWITCH Access Pinhole (Dia 3.4 mm)
    for lx in S.UM_LED_X_LIST:
        lp = (
            cq.Workplane("XY")
            .workplane(offset=-3.0)
            .center(lx, S.UM_LED_Y)
            .circle(1.95 / 2.0)
            .extrude(3.5)
        )
        cover = cover.cut(lp)

    sw_hole = (
        cq.Workplane("XY")
        .workplane(offset=-3.0)
        .center(S.UM_SW_X, S.UM_SW_Y)
        .circle(1.70)
        .extrude(3.5)
    )
    cover = cover.cut(sw_hole)

    return cover


# ==============================================================================
# Parts 3-5: Standardized Option B2 Pod (10000 mAh Cradle, Pole Lid) + 5000 mAh Adapter
# ==============================================================================
def build_battery_pod() -> tuple[cq.Workplane, cq.Workplane]:
    """Build standardized (Battery_Cradle, Pole_Lid) on the 10000 mAh footprint.

    Battery_Cradle features a 2.0 mm outer frame, 2.0 mm continuous inner LiPo bay,
    4 x Dia 8.6 mm M3 sleeves, a 1.8 mm interlocking Warren-truss web, and an
    integral Bottom-Left Capture Corner opposite the shared Bottom-Right cable exit.
    Pole_Lid stays a solid 3.0 mm plate sealing the truss and carrying twin 1/4"-20
    Ruthex inserts at 38.1 mm pitch.
    """
    bay_w, bay_h, bay_d, pod_h = S.POD_BAY_W, S.POD_BAY_H, S.POD_BAY_D, S.POD_H
    z_cradle_front = S.Z_PLATE_OUTER          # -28.0 (100% flat build-plate face)
    z_cradle_back = z_cradle_front - bay_d    # -40.8 (100% flat mate)
    wall = S.POD_FRAME_WALL
    web_t = S.POD_WEB_THK
    bay_cy = S.POD_CY + 1.0

    # --- Piece A: Skeletonized Battery_Cradle (Outer Frame + Inner Pocket + 4xM3 + Truss Web) ---
    outer_solid = make_rounded_box(S.POD_OUTER_W, pod_h, bay_d, S.POD_OUTER_R,
                                   cx=S.POD_CX, cy=S.POD_CY, z_min=z_cradle_back)
    outer_void = make_rounded_box(S.POD_OUTER_W - 2.0 * wall, pod_h - 2.0 * wall, bay_d + 1.0,
                                  max(1.5, S.POD_OUTER_R - wall),
                                  cx=S.POD_CX, cy=S.POD_CY, z_min=z_cradle_back - 0.5)
    frame = outer_solid.cut(outer_void)

    inner_outer = make_rounded_box(bay_w + 2.0 * wall, bay_h + 2.0 * wall, bay_d, 3.5,
                                   cx=S.POD_CX, cy=bay_cy, z_min=z_cradle_back)
    inner_void = make_rounded_box(bay_w, bay_h, bay_d + 1.0, 2.5,
                                  cx=S.POD_CX, cy=bay_cy, z_min=z_cradle_back - 0.5)
    pocket_wall = inner_outer.cut(inner_void)
    cradle = frame.union(pocket_wall)

    # 4 x Reinforced M3 corner bolt sleeves
    for px, py in S.POD_M3_POINTS:
        boss = (
            cq.Workplane("XY")
            .workplane(offset=z_cradle_back)
            .center(px, py)
            .circle(S.POD_M3_SLEEVE_OD / 2.0)
            .extrude(bay_d)
        )
        cradle = cradle.union(boss)

    # Interlocking Warren-truss web inside the annular gap between frame & pocket centerlines
    annulus = (
        make_rounded_box(S.POD_OUTER_W - wall, pod_h - wall, bay_d,
                         max(1.5, S.POD_OUTER_R - wall / 2.0),
                         cx=S.POD_CX, cy=S.POD_CY, z_min=z_cradle_back)
        .cut(make_rounded_box(bay_w + wall, bay_h + wall, bay_d + 1.0, 3.0,
                              cx=S.POD_CX, cy=bay_cy, z_min=z_cradle_back - 0.5))
    )

    def make_strut(x0: float, y0: float, x1: float, y1: float) -> cq.Workplane:
        dx, dy = x1 - x0, y1 - y0
        length = math.hypot(dx, dy) + web_t
        ang = math.degrees(math.atan2(dy, dx))
        return (
            cq.Workplane("XY")
            .workplane(offset=z_cradle_back)
            .center(0.5 * (x0 + x1), 0.5 * (y0 + y1))
            .transformed(rotate=(0, 0, ang))
            .rect(length, web_t)
            .extrude(bay_d)
        )

    web = None

    def add_web(strut: cq.Workplane) -> None:
        nonlocal web
        web = strut if web is None else web.union(strut)

    in_xl = S.POD_CX - (bay_w / 2.0 + wall / 2.0)
    in_xr = S.POD_CX + (bay_w / 2.0 + wall / 2.0)
    in_yb = bay_cy - (bay_h / 2.0 + wall / 2.0)
    in_yt = bay_cy + (bay_h / 2.0 + wall / 2.0)
    out_xl = S.POD_CX - (S.POD_OUTER_W / 2.0 - wall / 2.0)
    out_xr = S.POD_CX + (S.POD_OUTER_W / 2.0 - wall / 2.0)
    out_yb = S.POD_CY - (pod_h / 2.0 - wall / 2.0)
    out_yt = S.POD_CY + (pod_h / 2.0 - wall / 2.0)

    # Corner M3 radial ties + diagonal gussets into pocket and outer wall
    for px, py in S.POD_M3_POINTS:
        sx = -1.0 if px < S.POD_CX else 1.0
        sy = -1.0 if py < S.POD_CY else 1.0
        ix = in_xl if sx < 0 else in_xr
        ox = out_xl if sx < 0 else out_xr
        oy = out_yb if sy < 0 else out_yt
        add_web(make_strut(ox, py, ix, py))
        add_web(make_strut(px, oy, px, py))
        add_web(make_strut(px, py, ix, max(in_yb, min(in_yt, py - sy * 16.0))))

    # Left & Right side 6-bay Warren-truss zigzag web + spine load-center horizontal tie
    n_bays = 6
    y_lo, y_hi = S.POD_M3_POINTS[0][1], S.POD_M3_POINTS[2][1]
    ys = [y_lo + i * (y_hi - y_lo) / n_bays for i in range(n_bays + 1)]
    for side in (-1, 1):
        ix = in_xl if side < 0 else in_xr
        ox = out_xl if side < 0 else out_xr
        for i in range(n_bays):
            ya, yb = ys[i], ys[i + 1]
            if i % 2 == 0:
                add_web(make_strut(ox, ya, ix, yb))
            else:
                add_web(make_strut(ix, ya, ox, yb))
        add_web(make_strut(ox, S.POD_CY, ix, S.POD_CY))

    # Top & Bottom truss ties
    for tx in (S.POD_CX - 20.0, S.POD_CX, S.POD_CX + 20.0):
        add_web(make_strut(tx, out_yb, tx, in_yb))
        add_web(make_strut(tx, in_yt, tx, out_yt))
    for side in (-1, 1):
        ix = in_xl if side < 0 else in_xr
        ox = out_xl if side < 0 else out_xr
        add_web(make_strut(ox, out_yb + 2.0, ix, y_lo))
        add_web(make_strut(ox, out_yt - 2.0, ix, y_hi))

    cradle = cradle.union(web.intersect(annulus)).cut(inner_void)

    # Integral Bottom-Left Battery Capture Corner (X=[-30.25, -12.0], Y=[31.0, 38.75])
    # Supports bottom-left of both 10000 and 5000 mAh cells while leaving bottom-right open under Waveshare for JST wires.
    cap_x0 = (S.POD_CX - bay_w / 2.0) - 0.5
    cap_x1 = S.POD_CAPTURE_X1
    cap_y0 = (bay_cy - bay_h / 2.0) - 0.5
    cap_y1 = S.POD_CAPTURE_Y1
    capture = make_rounded_box(
        cap_x1 - cap_x0, cap_y1 - cap_y0, bay_d, radius=1.5,
        cx=0.5 * (cap_x0 + cap_x1), cy=0.5 * (cap_y0 + cap_y1), z_min=z_cradle_back
    )
    inner_clip = make_rounded_box(bay_w + 1.0, bay_h + 1.0, bay_d + 0.2, 2.8,
                                  cx=S.POD_CX, cy=bay_cy, z_min=z_cradle_back - 0.1)
    cradle = cradle.union(capture.intersect(inner_clip))

    # 4 x M3 pass-through bores (3.4 mm) aligning identically to POD_M3_POINTS
    for px, py in S.POD_M3_POINTS:
        bore = (
            cq.Workplane("XY")
            .workplane(offset=z_cradle_back - 1.5)
            .center(px, py)
            .circle(S.HOLE_CLEAR_M3 / 2.0)
            .extrude(bay_d + 3.0)
        )
        cradle = cradle.cut(bore)

    # --- Piece B: Pole_Lid (3.0 mm solid flat floor + twin 1/4"-20 Ruthex spine, 2.5 mm safety floor) ---
    # Prints 100% flat on z_lid_front with spine growing upward -> zero overhangs.
    z_lid_front = z_cradle_back - 0.10        # 0.10 mm gap avoids coplanar boolean touch
    z_lid_back = z_lid_front - S.POD_FLOOR_THK
    z_spine_back = z_lid_back - S.POD_SPINE_H

    lid = make_rounded_box(S.POD_OUTER_W, pod_h, S.POD_FLOOR_THK, S.POD_OUTER_R,
                           cx=S.POD_CX, cy=S.POD_CY, z_min=z_lid_back)

    # Central reinforced 24.0 mm Spine carrying twin 1/4"-20 Ruthex inserts (38.1 mm pitch)
    spine = make_rounded_box(S.POD_SPINE_W, 74.0, S.POD_SPINE_H + 0.5, 5.0,
                             cx=S.POD_CX, cy=S.POD_CY, z_min=z_spine_back)
    lid = lid.union(spine)

    # 2 x 1/4"-20 Ruthex bores open to -Z_asm (back), stopping >= 2.5 mm shy of z_lid_front
    for qy in (S.POD_CY - S.POLE_Q20_DY / 2.0, S.POD_CY + S.POLE_Q20_DY / 2.0):
        q_bore = (
            cq.Workplane("XY")
            .workplane(offset=z_spine_back - 0.2)
            .center(S.POD_CX, qy)
            .circle(S.RUTHEX_Q20_OD / 2.0)
            .extrude(S.RUTHEX_Q20_DEPTH + 0.2)
        )
        lid = lid.cut(q_bore)

    # 4 x M3 CSK thru-holes on outer flange (head on rear side z_lid_back)
    for px, py in S.POD_M3_POINTS:
        csk = make_csk_cone_through(px, py, z_bed=z_lid_back, plate_thk=S.POD_FLOOR_THK,
                                    hole_d=S.HOLE_CLEAR_M3, head_d=6.2, cone_from_bed=True)
        lid = lid.cut(csk)

    return cradle, lid


def build_battery_adapter_5000() -> cq.Workplane:
    """Build removable skeletonized L-truss adapter adapting the 10000 mAh bay to 5000 mAh."""
    bay_w, bay_h, bay_d = S.POD_BAY_W, S.POD_BAY_H, S.POD_BAY_D
    bay_cy = S.POD_CY + 1.0
    z_cradle_back = S.Z_PLATE_OUTER - bay_d
    ad_z0 = z_cradle_back + 0.10
    ad_h = bay_d - 0.20

    slip = S.ADAPTER_SLIP
    wall = S.ADAPTER_WALL
    web_t = S.ADAPTER_WEB_THK
    ad_xl = (S.POD_CX - bay_w / 2.0) + slip
    ad_xr = (S.POD_CX + bay_w / 2.0) - slip
    ad_yb = S.POD_CAPTURE_Y1 + slip
    ad_yt = (bay_cy + bay_h / 2.0) - slip
    p5_xl = S.BAT5_POCKET_X0
    p5_yt = S.BAT5_POCKET_Y1

    env_full = make_rounded_box(
        ad_xr - ad_xl, (bay_cy + bay_h / 2.0 - slip) - (bay_cy - bay_h / 2.0 + slip),
        ad_h, radius=2.5, cx=S.POD_CX, cy=bay_cy, z_min=ad_z0
    )
    l_top = (
        cq.Workplane("XY")
        .workplane(offset=ad_z0)
        .center(0.5 * (ad_xl + ad_xr), 0.5 * (p5_yt + ad_yt))
        .rect(ad_xr - ad_xl, ad_yt - p5_yt)
        .extrude(ad_h)
    )
    l_left = (
        cq.Workplane("XY")
        .workplane(offset=ad_z0)
        .center(0.5 * (ad_xl + p5_xl), 0.5 * (ad_yb + p5_yt + wall))
        .rect(p5_xl - ad_xl, (p5_yt + wall) - ad_yb)
        .extrude(ad_h)
    )
    l_solid = l_top.union(l_left).intersect(env_full)

    top_cut = make_rounded_box(
        (ad_xr - ad_xl) - 2.0 * wall, (ad_yt - p5_yt) - 2.0 * wall, ad_h + 1.0, radius=1.5,
        cx=0.5 * (ad_xl + ad_xr), cy=0.5 * (p5_yt + ad_yt), z_min=ad_z0 - 0.5
    )
    left_cut = make_rounded_box(
        (p5_xl - ad_xl) - 2.0 * wall, (p5_yt - ad_yb) - 2.0 * wall, ad_h + 1.0, radius=1.0,
        cx=0.5 * (ad_xl + p5_xl), cy=0.5 * (ad_yb + p5_yt), z_min=ad_z0 - 0.5
    )
    adapter = l_solid.cut(top_cut).cut(left_cut)

    def make_ad_strut(x0: float, y0: float, x1: float, y1: float) -> cq.Workplane:
        length = math.hypot(x1 - x0, y1 - y0) + web_t
        ang = math.degrees(math.atan2(y1 - y0, x1 - x0))
        return (
            cq.Workplane("XY")
            .workplane(offset=ad_z0)
            .center(0.5 * (x0 + x1), 0.5 * (y0 + y1))
            .transformed(rotate=(0, 0, ang))
            .rect(length, web_t)
            .extrude(ad_h)
        )

    tx0, tx1 = ad_xl + wall / 2.0, ad_xr - wall / 2.0
    ty0, ty1 = p5_yt + wall / 2.0, ad_yt - wall / 2.0
    tm = 0.5 * (tx0 + tx1)
    truss = (
        make_ad_strut(tx0, ty0, tm, ty1)
        .union(make_ad_strut(tm, ty1, tx1, ty0))
        .union(make_ad_strut(tx0, ty1, tm, ty0))
        .union(make_ad_strut(tm, ty0, tx1, ty1))
    )
    for ry in (ad_yb + (p5_yt - ad_yb) / 3.0, ad_yb + 2.0 * (p5_yt - ad_yb) / 3.0):
        truss = truss.union(make_ad_strut(ad_xl + wall / 2.0, ry, p5_xl - wall / 2.0, ry))

    return adapter.union(truss.intersect(l_solid))


# ==============================================================================
# Part 7: Slim_Sun_Shield (Option H1 - 1.6 mm PETG U-Channel Slot-In Hood)
# ==============================================================================
def build_slim_sun_shield() -> cq.Workplane:
    """Build Option H1 Slim_Sun_Shield (prints flat on back Z = -1.70 mm, support-free)."""
    xl, xr = S.SHIELD_U_X_LEFT, S.SHIELD_U_X_RIGHT
    yb, yt = S.SHIELD_U_Y_BOT, S.SHIELD_U_Y_TOP
    t = S.SHIELD_WALL_T
    half_t = t / 2.0
    z_base = -(S.SHIELD_GROOVE_D - 0.10)  # -1.70 mm (leaves 0.10 mm bottoming gap inside -1.80 groove)
    # Centerline polygon of outer perimeter vs inner perimeter for clean tapered loft
    outer_pts = [
        (xl - half_t, yb),
        (xl - half_t, yt + half_t),
        (xr + half_t, yt + half_t),
        (xr + half_t, yb),
        (xr - half_t, yb),
        (xr - half_t, yt - half_t),
        (xl + half_t, yt - half_t),
        (xl + half_t, yb),
    ]
    # Extrude max depth (+28 mm) then slice slanted taper from Z_bot (+12 mm) to Z_top (+28 mm)!
    u_full = (
        cq.Workplane("XY")
        .workplane(offset=z_base)
        .polyline(outer_pts)
        .close()
        .extrude(S.SHIELD_DEPTH_TOP - z_base)
    )
    # Taper slice polygon in YZ plane (clears Top visor roof at Y = yt..yt+half_t, slopes cleanly to yb)
    slope_pts = [
        (yb - 5.0, S.SHIELD_DEPTH_BOT),
        (yt - 10.0, S.SHIELD_DEPTH_TOP),
        (yt + 5.0, S.SHIELD_DEPTH_TOP),
        (yt + 5.0, S.SHIELD_DEPTH_TOP + 10.0),
        (yb - 5.0, S.SHIELD_DEPTH_TOP + 10.0),
    ]
    taper_cutter = (
        cq.Workplane("YZ")
        .workplane(offset=xl - 5.0)
        .polyline(slope_pts)
        .close()
        .extrude((xr - xl) + 10.0)
    )
    shield = u_full.cut(taper_cutter)
    # Slight fillet on front top vertical corners so it looks crisp
    return shield


# ==============================================================================
# Part 8: Button_Plungers (3 x Captive Tactile Plungers for Back_Chassis_Frame Flange)
# ==============================================================================
def build_button_plungers() -> cq.Workplane:
    """Build 3 captive round tactile button plungers (PWR, RESET, BOOT), 1.55 mm travel."""
    comp = None
    for by in S.WS_BTN_Y_LIST:
        # 1. Inner guided nose (Dia 3.20 mm, X = +45.65..+53.25; 0.30 mm rest gap to dome +45.35)
        nose = (
            cq.Workplane("YZ")
            .workplane(offset=45.65)
            .center(by, S.WS_BTN_Z_CTR)
            .circle(S.PLUNGER_NOSE_DIA / 2.0)
            .extrude(7.60)
        )
        # 2. Tapered 45-deg captive collar (X = +53.15..+55.85; 1.55 mm clear of U-comb +51.60)
        collar = (
            cq.Workplane("YZ")
            .workplane(offset=53.15)
            .center(by, S.WS_BTN_Z_CTR)
            .rect(5.80, 5.40)
            .workplane(offset=0.80)
            .rect(5.80, 5.40)
            .workplane(offset=1.90)
            .circle(2.05)
            .loft(combine=True)
        )
        # 3. Pure round cylindrical stem sticking 1.60 mm proud of right wall (Dia 4.10 mm, X = +55.80..+61.60)
        stem = (
            cq.Workplane("YZ")
            .workplane(offset=55.80)
            .center(by, S.WS_BTN_Z_CTR)
            .circle(2.05)
            .extrude(5.80)
        )
        one = nose.union(collar).union(stem)
        comp = one if comp is None else comp.add(one)
    return comp


# ==============================================================================
# Part 9: Plunger_Capture_Clip (Slide-In Back_Chassis_Frame Button Capture Bar)
# ==============================================================================
def build_plunger_capture_clip() -> cq.Workplane:
    """Build flat-printed slide-in clip capturing all 3 plungers on Back_Chassis_Frame."""
    clip_cx = 0.5 * (S.PLUNGER_U_X0 + S.PLUNGER_U_X1)
    clip_w = S.PLUNGER_U_X1 - S.PLUNGER_U_X0
    clip_cy = 0.5 * (S.PLUNGER_CLIP_Y0 + S.PLUNGER_CLIP_Y1)
    clip_h = S.PLUNGER_CLIP_Y1 - S.PLUNGER_CLIP_Y0
    clip_d = S.PLUNGER_CLIP_Z_TOP - S.PLUNGER_CLIP_Z_BOT
    clip = (
        cq.Workplane("XY")
        .workplane(offset=S.PLUNGER_CLIP_Z_BOT)
        .center(clip_cx, clip_cy)
        .rect(clip_w, clip_h)
        .extrude(clip_d)
    )
    for by in S.WS_BTN_Y_LIST:
        u_round = (
            cq.Workplane("YZ")
            .workplane(offset=S.PLUNGER_U_X0 - 0.5)
            .center(by, S.WS_BTN_Z_CTR)
            .circle(S.PLUNGER_U_SLOT_W / 2.0)
            .extrude(clip_w + 1.0)
        )
        u_open = (
            cq.Workplane("XY")
            .workplane(offset=S.PLUNGER_CLIP_Z_BOT - 0.5)
            .center(clip_cx, by)
            .rect(clip_w + 1.0, S.PLUNGER_U_SLOT_W)
            .extrude(S.WS_BTN_Z_CTR - (S.PLUNGER_CLIP_Z_BOT - 0.5))
        )
        clip = clip.cut(u_round).cut(u_open)
    # 0.40 mm 45-deg lead-in chamfers on bottom Y ends for effortless track drop-in
    c_ch = 0.40
    for y_end, dy in ((S.PLUNGER_CLIP_Y0, c_ch), (S.PLUNGER_CLIP_Y1, -c_ch)):
        ch_cut = (
            cq.Workplane("YZ")
            .workplane(offset=S.PLUNGER_U_X0 - 0.5)
            .polyline([
                (y_end - 0.2 * (1 if dy > 0 else -1), S.PLUNGER_CLIP_Z_BOT - 0.2),
                (y_end + dy, S.PLUNGER_CLIP_Z_BOT - 0.2),
                (y_end + dy, S.PLUNGER_CLIP_Z_BOT),
                (y_end, S.PLUNGER_CLIP_Z_BOT + c_ch),
                (y_end - 0.2 * (1 if dy > 0 else -1), S.PLUNGER_CLIP_Z_BOT + c_ch),
            ])
            .close()
            .extrude(clip_w + 1.0)
        )
        clip = clip.cut(ch_cut)
    return clip


# ==============================================================================
# Part 9: Pole_Tilt_Wedge_37_5 (Support-Free 37.5-deg Laser Mount Adapter)
# ==============================================================================
def _wedge_spine_datums() -> tuple[float, float, float]:
    """Return Pole_Lid mating Z plane, spine bottom Y, and pod X centerline."""
    z_lid_front = S.Z_PLATE_OUTER - S.POD_BAY_D - 0.10
    z_lid_back = z_lid_front - S.POD_FLOOR_THK
    z_spine = z_lid_back - S.POD_SPINE_H
    z_mate = z_spine - S.WEDGE_MATE_GAP
    y_sp_bot = S.POD_CY - 37.0
    return z_mate, y_sp_bot, S.POD_CX


def _local_wedge_to_asm(shape: cq.Workplane) -> cq.Workplane:
    """Transform wedge local (X_w, V_w, U_w) coordinates into assembly coordinates."""
    z_mate, y_sp_bot, x_cx = _wedge_spine_datums()
    return (
        shape
        .translate((0.0, -S.WEDGE_V_SB, -S.WEDGE_DU_SB))
        .rotate((0, 0, 0), (1, 0, 0), S.WEDGE_TILT_DEG)
        .translate((x_cx, y_sp_bot, z_mate))
    )


def _wedge_truss_geometry() -> tuple[list[tuple[float, float]], list[tuple[float, float]], list[tuple[tuple[float, float], tuple[float, float], float]]]:
    """Compute outer profile, 5-vertex open inner cavity, and Warren-truss struts in (V_w, U_w)."""
    th = math.radians(S.WEDGE_TILT_DEG)
    s, c = math.sin(th), math.cos(th)
    du_sb, v_sb = S.WEDGE_DU_SB, S.WEDGE_V_SB

    def sp_vu(dy: float, dn: float) -> tuple[float, float]:
        return (v_sb + dy * c + dn * s, du_sb - dy * s + dn * c)

    rim = S.FIT_CLEAR + S.WEDGE_POCKET_WALL
    p_bot_lip = sp_vu(-rim, S.WEDGE_POCKET_D)
    p_top_lip = sp_vu(74.0 + rim, S.WEDGE_POCKET_D)
    u_in = S.WEDGE_SHELF_W / 2.0 + S.WEDGE_SADDLE_CLR
    u_out = u_in + S.WEDGE_SKIRT_T

    prof_vu = [
        (-S.WEDGE_SKIRT_DROP, u_out),
        (-S.WEDGE_SKIRT_DROP, u_in),
        (0.0, u_in),
        (0.0, -u_in),
        (-S.WEDGE_SKIRT_DROP, -u_in),
        (-S.WEDGE_SKIRT_DROP, -u_out),
        (p_top_lip[0] - (u_out + p_top_lip[1]), -u_out),
        p_top_lip,
        p_bot_lip,
        (S.WEDGE_FRONT_GUSSET_V, u_out),
    ]

    # Convex open interior cavity bounded by chord_t rear/floor/front chords & SPINE_WEB
    u_rear_in = -u_out + S.WEDGE_CHORD_T
    rhs_sp = v_sb * s + du_sb * c - S.WEDGE_SPINE_WEB
    p_sp_bot_bk = sp_vu(-rim, -S.WEDGE_SPINE_WEB)
    va, ua = p_bot_lip
    vb, ub = S.WEDGE_FRONT_GUSSET_V, u_out
    l_frt = math.hypot(vb - va, ub - ua)
    nv, nu = -(ub - ua) / l_frt, (vb - va) / l_frt
    rhs_frt = va * nv + ua * nu + S.WEDGE_CHORD_T

    cav_pts = [
        (S.WEDGE_CHORD_T, u_rear_in),
        ((rhs_sp - c * u_rear_in) / s, u_rear_in),
        p_sp_bot_bk,
        (p_sp_bot_bk[0], (rhs_frt - nv * p_sp_bot_bk[0]) / nu),
        (S.WEDGE_CHORD_T, (rhs_frt - nv * S.WEDGE_CHORD_T) / nu),
    ]
    struts = [
        (sp_vu(37.0, -6.5), (35.0, -31.5), S.WEDGE_MID_STRUT_T),
        ((11.5, -7.0), (3.5, -21.0), S.WEDGE_TIE_STRUT_T),
        ((14.5, 7.0), (3.5, 24.5), S.WEDGE_TIE_STRUT_T),
    ]
    return prof_vu, cav_pts, struts


def build_pole_tilt_wedge() -> cq.Workplane:
    """Build 90-deg rotated skeletonized open-truss 37.5-deg wedge with pure round holes."""
    prof_vu, cav_pts, struts = _wedge_truss_geometry()

    def extrude_vu(pts: list[tuple[float, float]], half_x: float) -> cq.Workplane:
        return cq.Workplane("YZ").workplane(offset=-half_x).polyline(pts).close().extrude(2.0 * half_x)

    def make_vu_strut(p0: tuple[float, float], p1: tuple[float, float], thk: float) -> cq.Workplane:
        dv, du = p1[0] - p0[0], p1[1] - p0[1]
        length = math.hypot(dv, du)
        nv_s, nu_s = -du / length * (thk / 2.0), dv / length * (thk / 2.0)
        pts = [
            (p0[0] + nv_s, p0[1] + nu_s),
            (p1[0] + nv_s, p1[1] + nu_s),
            (p1[0] - nv_s, p1[1] - nu_s),
            (p0[0] - nv_s, p0[1] - nu_s),
        ]
        return extrude_vu(pts, S.WEDGE_CORE_W / 2.0)

    # 1. Skeletonized open Warren-truss frame extruded across WEDGE_CORE_W (32.0 mm)
    outer_2d = extrude_vu(prof_vu, S.WEDGE_CORE_W / 2.0)
    cav_cut = extrude_vu(cav_pts, S.WEDGE_CORE_W)
    boss_pts = [
        (0.0, -S.WEDGE_BOSS_HALF_U),
        (S.WEDGE_BOSS_V_REAR, -S.WEDGE_BOSS_HALF_U),
        (S.WEDGE_BOSS_V_FRONT, S.WEDGE_BOSS_HALF_U),
        (0.0, S.WEDGE_BOSS_HALF_U),
    ]
    wedge_local = outer_2d.cut(cav_cut).union(extrude_vu(boss_pts, S.WEDGE_CORE_W / 2.0))
    for p0, p1, thk in struts:
        wedge_local = wedge_local.union(make_vu_strut(p0, p1, thk))
    wedge_local = wedge_local.intersect(outer_2d)

    # 2. Pure round base 1/4"-20 Ruthex bore (Dia 8.0 x 13.7 mm along +V_w) + 0.5 mm lead-in cone
    r_q = S.RUTHEX_Q20_OD / 2.0
    base_bore = (
        cq.Workplane("XZ")
        .workplane(offset=0.5)
        .center(0.0, 0.0)
        .circle(r_q)
        .extrude(-(S.RUTHEX_Q20_DEPTH + 0.5))
    )
    lead_cone = (
        cq.Workplane("XY")
        .workplane(offset=-0.2)
        .circle(r_q + 0.7)
        .workplane(offset=0.7)
        .circle(r_q)
        .loft(combine=True)
        .rotate((0, 0, 0), (1, 0, 0), -90)
    )
    wedge_local = wedge_local.cut(base_bore).cut(lead_cone)

    # 3. Transform to Assembly coords & cut lofted 45-deg upper (+X_w) funnel spine pocket
    wedge_asm = _local_wedge_to_asm(wedge_local)
    z_mate, _, x_cx = _wedge_spine_datums()
    pw = S.POD_SPINE_W + 2.0 * S.FIT_CLEAR
    ph = 74.0 + 2.0 * S.FIT_CLEAR
    pr = 5.0 + S.FIT_CLEAR
    s_bot = cq.Sketch().rect(pw, ph).vertices().fillet(pr)
    s_top = (
        cq.Sketch()
        .rect(pw + S.WEDGE_POCKET_D + 1.0, ph)
        .vertices()
        .fillet(pr)
        .moved(cq.Location((0.5 * (S.WEDGE_POCKET_D + 1.0), 0.0, S.WEDGE_POCKET_D + 1.0)))
    )
    pocket_loft = (
        cq.Workplane("XY")
        .workplane(offset=z_mate)
        .center(x_cx, S.POD_CY)
        .placeSketch(s_bot, s_top)
        .loft()
    )
    wedge_asm = wedge_asm.cut(pocket_loft)

    # 4. Pure round twin 1/4"-20 SHCS through-bores & rear wrench access holes (no teardrops)
    for qy in (S.POD_CY - S.POLE_Q20_DY / 2.0, S.POD_CY + S.POLE_Q20_DY / 2.0):
        thru = (
            cq.Workplane("XY")
            .workplane(offset=z_mate - S.WEDGE_SPINE_WEB - 1.0)
            .center(x_cx, qy)
            .circle(S.WEDGE_Q20_CLR / 2.0)
            .extrude(S.WEDGE_SPINE_WEB + S.WEDGE_POCKET_D + 2.0)
        )
        cbore = (
            cq.Workplane("XY")
            .workplane(offset=z_mate - S.WEDGE_SPINE_WEB)
            .center(x_cx, qy)
            .circle(S.WEDGE_Q20_SHCS_CBORE / 2.0)
            .extrude(-80.0)
        )
        wedge_asm = wedge_asm.cut(thru).cut(cbore)

    return wedge_asm


def load_laser_mount_ref(x_screw: float, mode: str = "pole_left", lm_raw: cq.Workplane | None = None) -> cq.Workplane:
    """Import REF-laser_pole_mount.stp and transform it to the 90-deg rotated Rover assembly frame."""
    shp = cq.importers.importStep(str(REF_FILES["laser_mount"])) if lm_raw is None else lm_raw
    shp = shp.translate((-x_screw, 0.0, -S.WEDGE_REF_Z_SHELF))
    if mode == "pole_left":
        shp = shp.rotate((0, 0, 0), (0, 0, 1), 180.0).rotate((0, 0, 0), (1, 0, 0), -90.0)
    else:
        shp = shp.rotate((0, 0, 0), (1, 0, 0), -90.0)
    return _local_wedge_to_asm(shp)


# ==============================================================================
# Real REF-*.stp Model Importers & Normalized Assembly Transformers
# ==============================================================================
def load_normalized_refs() -> dict[str, cq.Workplane]:
    """Load all REF-*.stp hardware models and place them in exact assembly coordinates."""
    refs = {}

    # 1. Waveshare 3.5B (fixed R6.0 body REF-Waveshare...stp + right-facing (+X) 2x16 angled header Solid[4] from ASM)
    ws_body = cq.importers.importStep(str(REF_FILES["waveshare_body"]))
    s_body_bb = ws_body.solids().vals()[3].BoundingBox()
    gx, gy, gz = 0.5 * (s_body_bb.xmin + s_body_bb.xmax), 0.5 * (s_body_bb.ymin + s_body_bb.ymax), s_body_bb.zmax
    ws_body_norm = ws_body.translate((S.WS_CX - gx, S.WS_CY - gy, -gz))

    ws_asm = cq.importers.importStep(str(REF_FILES["waveshare"]))
    s3b = ws_asm.solids().vals()[3].BoundingBox()
    ax, ay, az = 0.5 * (s3b.xmin + s3b.xmax), 0.5 * (s3b.ymin + s3b.ymax), s3b.zmax
    hdr = cq.Workplane("XY").add(ws_asm.solids().vals()[4]).translate((S.WS_CX - ax, S.WS_CY - ay, -az))
    refs["waveshare"] = ws_body_norm.add(hdr)

    # 2. Coin cell + holder (Z = -25.0..-21.0 under Waveshare)
    cc = cq.importers.importStep(str(REF_FILES["coin_cell"]))
    ch = cq.importers.importStep(str(REF_FILES["coin_holder"]))
    cc_bb = cc.val().BoundingBox()
    # Coin cell cylinder center measured at (21.094, 44.234)
    dx_rtc = S.RTC_CX - 21.094
    dy_rtc = S.RTC_CY - 44.234
    dz_rtc = S.Z_PLATE_INNER - cc_bb.zmin
    refs["coin_cell"] = cc.translate((dx_rtc, dy_rtc, dz_rtc))
    refs["coin_holder"] = ch.translate((dx_rtc, dy_rtc, dz_rtc))

    # 3. Power Board (REF_power.stp is pre-aligned in Rover assembly coords, 16x76 mm 2x2mm holes)
    refs["power"] = cq.importers.importStep(str(REF_FILES["power"]))

    # 4. E220 Module (PCB Z=0 -> Z_PLATE_INNER + E220_STANDOFF_H = -16.50)
    e2 = cq.importers.importStep(str(REF_FILES["e220"]))
    ebb = e2.val().BoundingBox()
    e2_cx, e2_cy = 0.5 * (ebb.xmin + ebb.xmax), 0.5 * (ebb.ymin + ebb.ymax)
    refs["e220"] = e2.translate((S.E220_CX - e2_cx, S.E220_CY - e2_cy, S.Z_PLATE_INNER + S.E220_STANDOFF_H))

    # 5. Holybro UM982 (Back Z=0 -> UM_BACK_Z = -17.20; Body top at UM_TOP_Y = 178.00)
    um = cq.importers.importStep(str(REF_FILES["um982"]))
    ubb = um.val().BoundingBox()
    um_cx = 0.5 * (ubb.xmin + ubb.xmax)
    um_bot_raw = ubb.ymin
    refs["um982"] = um.translate((S.UM_CX - um_cx, S.UM_BOT_Y - um_bot_raw, S.UM_BACK_Z - ubb.zmin))

    # 6. Speaker (normalized cone center (0,0), rotated 180 deg Z so wire bump faces +Y, Z = -13.5..-3.5)
    sp = cq.importers.importStep(str(REF_FILES["speaker"]))
    sbb = sp.val().BoundingBox()
    sp_norm = sp.translate((-0.5 * (sbb.xmin + sbb.xmax), -(sbb.ymin + 16.0), -sbb.zmin)).rotate((0, 0, 0), (0, 0, 1), 180)
    refs["speaker"] = sp_norm.translate((S.SPK_CX, S.SPK_CY, S.Z_PLATE_INNER + S.SPK_PEDESTAL_H))

    # 7. GX-12 Compass Bulkhead (REF_gx12.stp is pre-aligned in Rover assembly coordinates w/ downward D-flat)
    refs["gx12"] = cq.importers.importStep(str(REF_FILES["gx12"]))

    # 7b. 3.5mm Headphone Jack (REF-headphone-jack.stp normalized: body front at X=-60+HP_WALL+0.10, bushing out -X, pins UP +Z)
    hp = cq.importers.importStep(str(REF_FILES["headphone"]))
    hbb = hp.val().BoundingBox()
    hp_cx = 0.5 * (hbb.xmin + hbb.xmax)
    hp_y_body_front = hbb.ymax - S.HP_BUSHING_L
    hp_z_ctr = hbb.zmax - S.HP_BODY_H / 2.0
    hp_rot = (
        hp.translate((-hp_cx, -hp_y_body_front, -hp_z_ctr))
        .rotate((0, 0, 0), (1, 0, 0), 180)
        .rotate((0, 0, 0), (0, 0, 1), -90)
    )
    refs["headphone"] = hp_rot.translate((-S.CASE_W / 2.0 + S.HP_WALL + 0.10, S.HP_CY, S.HP_Z_CTR))

    # 8. E220 LoRa Female SMA Bulkhead (rotated -90 deg X; inner hex flange face Z_ref=2.0 at Y=177.40)
    sma = cq.importers.importStep(str(REF_FILES["sma"]))
    smabb = sma.val().BoundingBox()
    sma_cx, sma_cy = 0.5 * (smabb.xmin + smabb.xmax), 0.5 * (smabb.ymin + smabb.ymax)
    sma_rot = sma.translate((-sma_cx, -sma_cy, -2.0)).rotate((0, 0, 0), (1, 0, 0), -90)
    refs["sma"] = sma_rot.translate((S.E220_SMA_X, S.CASE_H - S.E220_SMA_WALL, S.E220_SMA_Z))

    # 9. 5000 mAh LiPo Battery (flipped 180 deg X so wire tab sits bottom-right over BAT_SLOT)
    b5 = cq.importers.importStep(str(REF_FILES["bat5000"]))
    b5b = b5.val().BoundingBox()
    b5_rot = b5.translate((-0.5 * (b5b.xmin + b5b.xmax), -0.5 * (b5b.ymin + b5b.ymax), -0.5 * (b5b.zmin + b5b.zmax))).rotate((0, 0, 0), (1, 0, 0), 180)
    b5rb = b5_rot.val().BoundingBox()
    # Align X_max=+33.50 & Y_body_min=39.25 (2.0 mm wire tab sits X=[28.50, 30.50] Y=[37.25, 39.25] directly over BAT_SLOT)
    refs["bat5000"] = b5_rot.translate((-b5rb.xmax + 33.50, -b5rb.ymin + 37.25, -b5rb.zmax + S.Z_PLATE_OUTER - 1.5))

    # 10. 10000 mAh LiPo Battery (aligned identically at X=-29.50, Y_body_min=39.25, offset 1.5 mm off Back Plate)
    b10 = cq.importers.importStep(str(REF_FILES["bat10000"]))
    b10b = b10.val().BoundingBox()
    refs["bat10000"] = b10.translate((-b10b.xmin - 29.50, -b10b.ymin + 39.25, -b10b.zmax + S.Z_PLATE_OUTER - 1.5))

    # 11. Laser Pole Mount Clamp (REF-laser_pole_mount.stp, raw copy + default inner pole_left placement)
    lm_raw = cq.importers.importStep(str(REF_FILES["laser_mount"]))
    refs["laser_mount_raw"] = lm_raw
    refs["laser_mount"] = load_laser_mount_ref(S.WEDGE_REF_SLOT_INNER, "pole_left", lm_raw=lm_raw)

    return refs


# ==============================================================================
# Print Orientation Helpers & Exporters
# ==============================================================================
def to_print_bed(shape: cq.Workplane, mode: str) -> cq.Workplane:
    """Orient part flat onto Z = 0.0 build plate for support-free PETG printing."""
    if mode == "face_down":
        s = shape.rotate((0, 0, 0), (1, 0, 0), 180)
    elif mode in ("plunger_flat", "clip_flat"):
        s = shape.rotate((0, 0, 0), (0, 1, 0), 90)
    elif mode == "wedge_side_down":
        z_mate, y_sp_bot, x_cx = _wedge_spine_datums()
        s = (
            shape
            .translate((-x_cx, -y_sp_bot, -z_mate))
            .rotate((0, 0, 0), (1, 0, 0), -S.WEDGE_TILT_DEG)
            .translate((0.0, S.WEDGE_V_SB, S.WEDGE_DU_SB))
            .rotate((0, 0, 0), (0, 1, 0), -90.0)
        )
    else:
        s = shape
    bb = s.val().BoundingBox()
    return s.translate((0.0, 0.0, -bb.zmin))


def verify_assembly_and_refs(parts: dict[str, cq.Workplane], refs: dict[str, cq.Workplane]) -> None:
    """Run automated manifold, outer envelope, and 0.00 mm^3 real-model collision audits."""
    print("\n--- [1/4] Manifold & Outer Envelope Checks ---")
    for name, shp in parts.items():
        n_solids = len(shp.solids().vals())
        bb = shp.val().BoundingBox()
        expected = 3 if name == "Button_Plungers" else 1
        assert n_solids == expected, f"{name}: expected {expected} solid(s), got {n_solids}"
        if name in ("Back_Chassis_Frame", "Top_Cover"):
            assert abs(bb.xmin + S.CASE_W / 2.0) < 0.02, f"{name} xmin={bb.xmin:.3f}"
            assert abs(bb.xmax - S.CASE_W / 2.0) < 0.02, f"{name} xmax={bb.xmax:.3f}"
            assert abs(bb.ymin - 0.0) < 0.02, f"{name} ymin={bb.ymin:.3f}"
            assert abs(bb.ymax - S.CASE_H) < 0.02, f"{name} ymax={bb.ymax:.3f}"
        vol_cm3 = sum(s.Volume() for s in shp.solids().vals()) / 1000.0
        print(f"  [OK] {name:22s}: {n_solids} solid(s), vol={vol_cm3:6.2f} cm^3, "
              f"X[{bb.xmin:7.2f},{bb.xmax:7.2f}] Y[{bb.ymin:6.2f},{bb.ymax:6.2f}] Z[{bb.zmin:6.2f},{bb.zmax:6.2f}]")

    print("\n--- [2/4] Printed Part vs Printed Part Interference Audit ---")
    printed_pairs = [
        ("Back_Chassis_Frame", "Top_Cover"),
        ("Top_Cover", "Slim_Sun_Shield"),
        ("Back_Chassis_Frame", "Slim_Sun_Shield"),
        ("Back_Chassis_Frame", "Button_Plungers"),
        ("Top_Cover", "Button_Plungers"),
        ("Back_Chassis_Frame", "Plunger_Capture_Clip"),
        ("Top_Cover", "Plunger_Capture_Clip"),
        ("Button_Plungers", "Plunger_Capture_Clip"),
        ("Back_Chassis_Frame", "Battery_Cradle"),
        ("Battery_Cradle", "Pole_Lid"),
        ("Back_Chassis_Frame", "Pole_Lid"),
        ("Battery_Cradle", "Battery_Adapter_5000"),
        ("Pole_Lid", "Battery_Adapter_5000"),
        ("Back_Chassis_Frame", "Battery_Adapter_5000"),
        ("Pole_Tilt_Wedge_37_5", "Pole_Lid"),
        ("Pole_Tilt_Wedge_37_5", "Battery_Cradle"),
        ("Pole_Tilt_Wedge_37_5", "Back_Chassis_Frame"),
        ("Pole_Tilt_Wedge_37_5", "Top_Cover"),
    ]
    for a_name, b_name in printed_pairs:
        inter = parts[a_name].intersect(parts[b_name])
        vol = sum(s.Volume() for s in inter.solids().vals())
        print(f"  {a_name:22s} vs {b_name:22s}: overlap = {vol:8.4f} mm^3")
        assert vol <= 0.05, f"Interference failure: {a_name} vs {b_name} = {vol:.4f} mm^3"

    print("\n--- [3/4] Printed Enclosure vs Real REF-*.stp Hardware Audit ---")
    ref_checks = [
        ("Back_Chassis_Frame",   "waveshare"),
        ("Top_Cover",            "waveshare"),
        ("Button_Plungers",      "waveshare"),
        ("Plunger_Capture_Clip", "waveshare"),
        ("Back_Chassis_Frame",   "coin_cell"),
        ("Back_Chassis_Frame",   "power"),
        ("Top_Cover",            "power"),
        ("Back_Chassis_Frame",   "e220"),
        ("Top_Cover",            "e220"),
        ("Back_Chassis_Frame",   "um982"),
        ("Top_Cover",            "um982"),
        ("Back_Chassis_Frame",   "speaker"),
        ("Top_Cover",            "speaker"),
        ("Back_Chassis_Frame",   "headphone"),
        ("Top_Cover",            "headphone"),
        ("Back_Chassis_Frame",   "gx12"),
        ("Top_Cover",            "gx12"),
        ("Back_Chassis_Frame",   "sma"),
        ("Top_Cover",            "sma"),
        ("Battery_Cradle",       "bat10000"),
        ("Pole_Lid",             "bat10000"),
        ("Back_Chassis_Frame",   "bat10000"),
        ("Battery_Cradle",       "bat5000"),
        ("Battery_Adapter_5000", "bat5000"),
        ("Pole_Lid",             "bat5000"),
        ("Back_Chassis_Frame",   "bat5000"),
    ]
    for p_name, r_name in ref_checks:
        inter = parts[p_name].intersect(refs[r_name])
        vol = sum(s.Volume() for s in inter.solids().vals())
        print(f"  {p_name:22s} vs REF_{r_name:12s}: overlap = {vol:8.4f} mm^3")
        assert vol <= 0.05, f"Hardware collision: {p_name} vs REF_{r_name} = {vol:.4f} mm^3"
    hp_spk_ov = sum(s.Volume() for s in refs["headphone"].intersect(refs["speaker"]).solids().vals())
    print(f"  REF_headphone          vs REF_speaker     : overlap = {hp_spk_ov:8.4f} mm^3")
    assert hp_spk_ov <= 0.05, f"Hardware collision: REF_headphone vs REF_speaker = {hp_spk_ov:.4f} mm^3"

    print("\n--- [4/4] Tilted Rover vs Real REF-laser_pole_mount.stp Audit ---")
    lm_parts = (
        "Pole_Tilt_Wedge_37_5",
        "Pole_Lid",
        "Battery_Cradle",
        "Back_Chassis_Frame",
        "Top_Cover",
        "Slim_Sun_Shield",
        "Button_Plungers",
        "Plunger_Capture_Clip",
    )
    lm_cases = (
        ("pole_left", S.WEDGE_REF_SLOT_INNER),
        ("pole_left", S.WEDGE_REF_SLOT_OUTER),
        ("pole_right", S.WEDGE_REF_SLOT_INNER),
    )
    for mode, x_screw in lm_cases:
        lm_ref = load_laser_mount_ref(x_screw, mode, lm_raw=refs["laser_mount_raw"])
        for p_name in lm_parts:
            inter = parts[p_name].intersect(lm_ref)
            vol = sum(s.Volume() for s in inter.solids().vals())
            print(f"  REF_laser({mode:10s}, X={x_screw:5.2f}) vs {p_name:22s}: overlap = {vol:8.4f} mm^3")
            assert vol <= 0.05, f"REF_laser({mode}, X={x_screw}) vs {p_name}: {vol:.4f} mm^3"


def export_2d_dxf(path: Path) -> None:
    """Export AutoCAD LT 2010 compatible 2D DXF of chassis, standardized pod, adapter, crown, and tilt wedge."""
    doc = ezdxf.new("R2010")
    doc.header["$INSUNITS"] = 4  # mm
    msp = doc.modelspace()

    layers = {
        "CASE_OUTLINE": 7,
        "MODULES": 3,
        "FASTENERS_M3": 1,
        "SUNSHIELD_GROOVE": 6,
        "POD_B2": 4,
        "CROWN_BULKHEAD": 2,
        "TILT_WEDGE_37_5": 5,
    }
    for lname, color in layers.items():
        doc.layers.add(lname, color=color)

    def add_rect(cx, cy, w, h, layer):
        x0, x1 = cx - w / 2.0, cx + w / 2.0
        y0, y1 = cy - h / 2.0, cy + h / 2.0
        msp.add_lwpolyline([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], close=True, dxfattribs={"layer": layer})

    # Front XY footprint
    add_rect(0.0, S.CASE_H / 2.0, S.CASE_W, S.CASE_H, "CASE_OUTLINE")
    add_rect(S.WS_CX, S.WS_CY, S.WS_GLASS_W, S.WS_GLASS_H, "MODULES")
    add_rect(S.PB_CX, S.PB_CY, S.PB_W, S.PB_H, "MODULES")
    add_rect(S.SPK_CX, S.SPK_CY, S.SPK_W, S.SPK_H, "MODULES")
    add_rect(-S.CASE_W / 2.0 + S.HP_WALL + S.HP_BODY_L / 2.0, S.HP_CY, S.HP_BODY_L, S.HP_BODY_W, "MODULES")
    add_rect(S.E220_CX, S.E220_CY, S.E220_W, S.E220_H, "MODULES")
    add_rect(S.UM_CX, S.UM_CY, S.UM_W, S.UM_BODY_H, "MODULES")
    msp.add_circle((S.RTC_CX, S.RTC_CY), 13.25, dxfattribs={"layer": "MODULES"})

    # Battery slot, unified Option B2 pod, Bottom-Left Capture Corner & 5000 mAh adapter pocket
    bay_cy = S.POD_CY + 1.0
    add_rect(S.BAT_SLOT_CX, S.BAT_SLOT_CY, S.BAT_SLOT_W, S.BAT_SLOT_H, "POD_B2")
    add_rect(S.POD_CX, S.POD_CY, S.POD_OUTER_W, S.POD_H, "POD_B2")
    add_rect(S.POD_CX, bay_cy, S.POD_BAY_W, S.POD_BAY_H, "POD_B2")
    bay_xl = S.POD_CX - S.POD_BAY_W / 2.0
    bay_xr = S.POD_CX + S.POD_BAY_W / 2.0
    cap_y0 = bay_cy - S.POD_BAY_H / 2.0
    msp.add_lwpolyline([
        (bay_xl, cap_y0),
        (S.POD_CAPTURE_X1, cap_y0),
        (S.POD_CAPTURE_X1, S.POD_CAPTURE_Y1),
        (bay_xl, S.POD_CAPTURE_Y1),
    ], close=True, dxfattribs={"layer": "POD_B2"})
    msp.add_lwpolyline([
        (S.BAT5_POCKET_X0, S.POD_CAPTURE_Y1),
        (bay_xr, S.POD_CAPTURE_Y1),
        (bay_xr, S.BAT5_POCKET_Y1),
        (S.BAT5_POCKET_X0, S.BAT5_POCKET_Y1),
    ], close=True, dxfattribs={"layer": "POD_B2"})
    for qy in (S.POD_CY - S.POLE_Q20_DY / 2.0, S.POD_CY + S.POLE_Q20_DY / 2.0):
        msp.add_circle((S.POD_CX, qy), S.RUTHEX_Q20_OD / 2.0, dxfattribs={"layer": "POD_B2"})

    # Sun Shield U-groove centerline
    msp.add_lwpolyline([
        (S.SHIELD_U_X_LEFT, S.SHIELD_U_Y_BOT),
        (S.SHIELD_U_X_LEFT, S.SHIELD_U_Y_TOP),
        (S.SHIELD_U_X_RIGHT, S.SHIELD_U_Y_TOP),
        (S.SHIELD_U_X_RIGHT, S.SHIELD_U_Y_BOT),
    ], close=False, dxfattribs={"layer": "SUNSHIELD_GROOVE"})

    # Fasteners
    for mx, my in S.COVER_M3_POINTS:
        msp.add_circle((mx, my), S.RUTHEX_M3_OD / 2.0, dxfattribs={"layer": "FASTENERS_M3"})
    for px, py in S.POD_M3_POINTS:
        msp.add_circle((px, py), S.RUTHEX_M3_OD / 2.0, dxfattribs={"layer": "FASTENERS_M3"})

    # Top L-Wall crown projection (offset Y + 215 mm)
    y_off = S.CASE_H + 35.0
    add_rect(0.0, y_off - (abs(S.Z_PLATE_OUTER) / 2.0), S.CASE_W, abs(S.Z_PLATE_OUTER), "CROWN_BULKHEAD")
    msp.add_circle((S.GX12_X, y_off + S.GX12_Z), S.GX12_HOLE_DIA / 2.0, dxfattribs={"layer": "CROWN_BULKHEAD"})
    gx_r = S.GX12_HOLE_DIA / 2.0
    gx_dz = -(S.GX12_FLAT - gx_r)
    gx_dx = math.sqrt(max(0.0, gx_r * gx_r - gx_dz * gx_dz))
    msp.add_line(
        (S.GX12_X - gx_dx, y_off + S.GX12_Z + gx_dz),
        (S.GX12_X + gx_dx, y_off + S.GX12_Z + gx_dz),
        dxfattribs={"layer": "CROWN_BULKHEAD"},
    )
    msp.add_circle((S.E220_SMA_X, y_off + S.E220_SMA_Z), S.E220_SMA_DIA / 2.0, dxfattribs={"layer": "CROWN_BULKHEAD"})
    e2_r = S.E220_SMA_DIA / 2.0
    e2_dz = -(S.E220_SMA_FLAT - e2_r)
    e2_dx = math.sqrt(max(0.0, e2_r * e2_r - e2_dz * e2_dz))
    msp.add_line(
        (S.E220_SMA_X - e2_dx, y_off + S.E220_SMA_Z + e2_dz),
        (S.E220_SMA_X + e2_dx, y_off + S.E220_SMA_Z + e2_dz),
        dxfattribs={"layer": "CROWN_BULKHEAD"},
    )
    msp.add_circle((S.UM_SMA_SLAVE_X, y_off + S.UM_SMA_Z), S.UM_SMA_ACCESS_DIA / 2.0, dxfattribs={"layer": "CROWN_BULKHEAD"})
    msp.add_circle((S.UM_SMA_MASTER_X, y_off + S.UM_SMA_Z), S.UM_SMA_ACCESS_DIA / 2.0, dxfattribs={"layer": "CROWN_BULKHEAD"})

    # 37.5-deg Tilt Wedge side profile (offset X = +120 mm, local V_w -> +X_dxf, U_w -> +Y_dxf shifted +60)
    wx_off, wy_off = 120.0, 60.0
    th = math.radians(S.WEDGE_TILT_DEG)
    sw, cw = math.sin(th), math.cos(th)
    wedge_prof, cav_pts, struts = _wedge_truss_geometry()
    msp.add_lwpolyline([(wx_off + vx, wy_off + ux) for vx, ux in wedge_prof], close=True, dxfattribs={"layer": "TILT_WEDGE_37_5"})
    msp.add_lwpolyline([(wx_off + vx, wy_off + ux) for vx, ux in cav_pts], close=True, dxfattribs={"layer": "TILT_WEDGE_37_5"})
    for p0, p1, _ in struts:
        msp.add_line((wx_off + p0[0], wy_off + p0[1]), (wx_off + p1[0], wy_off + p1[1]), dxfattribs={"layer": "TILT_WEDGE_37_5"})
    # Base 1/4"-20 Ruthex center + twin SHCS bolt axes
    msp.add_line((wx_off + 0.0, wy_off), (wx_off + S.RUTHEX_Q20_DEPTH, wy_off), dxfattribs={"layer": "TILT_WEDGE_37_5"})
    for dy_q in (37.0 - S.POLE_Q20_DY / 2.0, 37.0 + S.POLE_Q20_DY / 2.0):
        vq = S.WEDGE_V_SB + dy_q * cw
        uq = S.WEDGE_DU_SB - dy_q * sw
        msp.add_line((wx_off + vq - 20.0 * sw, wy_off + uq - 20.0 * cw),
                     (wx_off + vq + 5.0 * sw, wy_off + uq + 5.0 * cw),
                     dxfattribs={"layer": "TILT_WEDGE_37_5"})

    doc.saveas(str(path))
    print(f"  Exported DXF -> {path.name}")


def main() -> None:
    print(f"=== {__filename__} {__version__} ===")
    STEP_DIR.mkdir(parents=True, exist_ok=True)
    STL_DIR.mkdir(parents=True, exist_ok=True)
    DXF_DIR.mkdir(parents=True, exist_ok=True)

    print(f"Building parametric {__version__} enclosure parts...")
    cradle, lid = build_battery_pod()

    parts = {
        "Back_Chassis_Frame": build_back_chassis_frame(),
        "Top_Cover": build_top_cover(),
        "Battery_Cradle": cradle,
        "Pole_Lid": lid,
        "Battery_Adapter_5000": build_battery_adapter_5000(),
        "Pole_Tilt_Wedge_37_5": build_pole_tilt_wedge(),
        "Slim_Sun_Shield": build_slim_sun_shield(),
        "Button_Plungers": build_button_plungers(),
        "Plunger_Capture_Clip": build_plunger_capture_clip(),
    }

    print("Importing all 14 REF*.stp hardware reference models...")
    refs = load_normalized_refs()

    verify_assembly_and_refs(parts, refs)

    print("\n--- Exporting B-Rep STEP & Print-Ready STL Files ---")
    for name, shp in parts.items():
        step_path = STEP_DIR / f"{name}.step"
        cq.exporters.export(shp, str(step_path))
        print(f"  Exported STEP -> {step_path.name}")

    stl_map = [
        ("Back_Chassis_Frame",   "back_chassis_print.stl",    "back_down"),
        ("Top_Cover",            "top_cover_print.stl",       "face_down"),
        ("Battery_Cradle",       "bat_cradle_print.stl",      "face_down"),
        ("Pole_Lid",             "pole_lid_print.stl",        "face_down"),
        ("Battery_Adapter_5000", "bat_adapter_5000_print.stl","face_down"),
        ("Pole_Tilt_Wedge_37_5", "pole_tilt_wedge_print.stl", "wedge_side_down"),
        ("Slim_Sun_Shield",      "slim_sun_shield_print.stl",      "back_down"),
        ("Button_Plungers",      "button_plungers_print.stl",      "plunger_flat"),
        ("Plunger_Capture_Clip", "plunger_capture_clip_print.stl", "clip_flat"),
    ]
    for name, stl_name, orient in stl_map:
        stl_path = STL_DIR / stl_name
        p_shp = to_print_bed(parts[name], orient)
        cq.exporters.export(p_shp, str(stl_path), tolerance=0.05, angularTolerance=0.1)
        print(f"  Exported STL  -> {stl_name}")

    # Export full SolidEdge B-Rep assemblies (10000 mAh standard & 5000 mAh w/ adapter)
    for variant, bat_key, include_adapter in [
        ("5000mAh", "bat5000", True),
        ("10000mAh", "bat10000", False),
    ]:
        assy = cq.Assembly(name=f"RTK_Rover_Assembly_{variant}")
        assy_parts = [
            ("Back_Chassis_Frame", cq.Color(0.22, 0.25, 0.29, 1.0)),
            ("Top_Cover",          cq.Color(0.28, 0.34, 0.42, 0.92)),
            ("Battery_Cradle",     cq.Color(0.18, 0.22, 0.27, 1.0)),
            ("Pole_Lid",           cq.Color(0.14, 0.17, 0.21, 1.0)),
            ("Pole_Tilt_Wedge_37_5", cq.Color(0.24, 0.30, 0.36, 1.0)),
            ("Slim_Sun_Shield",      cq.Color(0.12, 0.14, 0.17, 1.0)),
            ("Button_Plungers",      cq.Color(0.90, 0.45, 0.10, 1.0)),
            ("Plunger_Capture_Clip", cq.Color(0.85, 0.68, 0.15, 1.0)),
        ]
        if include_adapter:
            assy_parts.insert(3, ("Battery_Adapter_5000", cq.Color(0.15, 0.55, 0.52, 1.0)))
        for pname, color in assy_parts:
            assy.add(parts[pname], name=pname, color=color)
        for rname, rcolor in [
            ("waveshare",   cq.Color(0.08, 0.35, 0.25, 1.0)),
            ("um982",       cq.Color(0.10, 0.10, 0.12, 1.0)),
            ("e220",        cq.Color(0.12, 0.42, 0.22, 1.0)),
            ("power",       cq.Color(0.12, 0.28, 0.55, 1.0)),
            ("speaker",     cq.Color(0.18, 0.18, 0.18, 1.0)),
            ("headphone",   cq.Color(0.15, 0.16, 0.18, 1.0)),
            ("coin_cell",   cq.Color(0.85, 0.72, 0.15, 1.0)),
            ("gx12",        cq.Color(0.75, 0.76, 0.78, 1.0)),
            ("sma",         cq.Color(0.82, 0.65, 0.18, 1.0)),
            (bat_key,       cq.Color(0.85, 0.70, 0.10, 1.0)),
            ("laser_mount", cq.Color(0.18, 0.19, 0.21, 1.0)),
        ]:
            assy.add(refs[rname], name=f"REF_{rname}", color=rcolor)
        assy_path = STEP_DIR / f"RTK_Rover_Assembly_{variant}.step"
        assy.export(str(assy_path))
        print(f"  Exported ASSY -> {assy_path.name}")

    export_2d_dxf(DXF_DIR / "RTK_Rover_Interfaces_2D.dxf")
    print(f"\n=== ALL CAD {__version__} CHECKS & EXPORTS PASSED! ===")


if __name__ == "__main__":
    main()
