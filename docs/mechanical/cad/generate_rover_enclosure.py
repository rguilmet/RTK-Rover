"""Generate parametric 3D STEP/STL CAD models and 2D DXF drawings for RTK Rover v2.0.0.

Architecture:
  - Back_Chassis_Frame: Open-frame rear plate carrying ALL modules (Waveshare 3.5B,
    underneath RTC Coin Cell + holder, Holybro UM982, E220 LoRa, Power Board,
    Speaker pedestal), integral Top L-Bulkhead (GX-12, E220 SMA, UM982 SMAs),
    and integral Right Button-Plunger & MicroSD Flange.
  - Top_Cover: Clean lift-off cover with ZERO modules/connectors mounted on it;
    flush glass bezel, USB-C tunnel, speaker acoustic slots, 4x UM982 LED light
    pipes + SWITCH pinhole, Option H1 Sun Shield U-groove, and 6 symmetric
    front-entry M3 CSK screws.
  - Option B2 Pods (5000 mAh and 10000 mAh): Swappable 2-piece support-free
    Battery_Cradle + Pole_Lid (twin 1/4"-20 Ruthex inserts @ 38.1 mm pitch)
    bolting onto the shared 4 x M3 rear anchor grid + JST pass-through slot.
  - Slim_Sun_Shield (Option H1): Ultra-thin 1.6 mm PETG slot-in tapered hood.
  - Button_Plungers: 3 captive tactile button plungers (PWR, RESET, BOOT).
"""

import math
import os
from pathlib import Path

import cadquery as cq
import ezdxf

import rover_specs as S

__filename__ = os.path.basename(__file__)
__version__ = "v2.0.2"

CAD_DIR = Path(__file__).resolve().parent
STEP_DIR = CAD_DIR / "step"
STL_DIR = CAD_DIR / "stl"
DXF_DIR = CAD_DIR / "dxf"

REF_FILES = {
    "waveshare_body": STEP_DIR / "REF-Waveshare-ESP32-S3-Touch-LCD-3.5B.stp",
    "waveshare": STEP_DIR / "REF-ASM-Waveshare-esp32-s3-touch-lcd-v3.5B-with-facing-header.stp",
    "um982": STEP_DIR / "REF-holybro-um982.stp",
    "e220": STEP_DIR / "REF-e220-module.stp",
    "power": STEP_DIR / "REF-power-module.stp",
    "speaker": STEP_DIR / "REF-speaker.stp",
    "coin_cell": STEP_DIR / "REF-coin-cell.stp",
    "coin_holder": STEP_DIR / "REF-coin-cell-holder.stp",
    "gx12": STEP_DIR / "REF-compass-module-input-connector.stp",
    "sma": STEP_DIR / "REF-sma-female-lora-connector.stp",
    "bat5000": STEP_DIR / "REF-lipo-battery-3.7V-5000mAh.stp",
    "bat10000": STEP_DIR / "REF-lipo-battery-3.7V-10000mAh.stp",
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


# ==============================================================================
# Part 1: Back_Chassis_Frame (Prints flat on Z_PLATE_OUTER = -28.0 mm, Support-Free)
# ==============================================================================
def build_back_chassis_frame() -> cq.Workplane:
    """Build Back_Chassis_Frame carrying all modules, Top L-wall, and Right Plunger Flange."""
    case_cy = S.CASE_H / 2.0

    # 1. Base chassis back plate: full outer footprint [Z_PLATE_OUTER=-28.0 .. Z_PLATE_INNER=-25.0]
    plate = make_rounded_box(S.CASE_W, S.CASE_H, S.WALL_BACK, S.CORNER_R,
                             cx=0.0, cy=case_cy, z_min=S.Z_PLATE_OUTER)

    # 2. Integrated Top L-Wall Bulkhead (Y = 177.0..180.0, Z = -28.0..-2.65)
    # Trim inside Top_Cover outer R6 shape so corners stay dead flush.
    full_wall_envelope = make_rounded_box(S.CASE_W, S.CASE_H, 25.35, S.CORNER_R,
                                          cx=0.0, cy=case_cy, z_min=S.Z_PLATE_OUTER)
    top_band = (
        cq.Workplane("XY")
        .workplane(offset=S.Z_PLATE_OUTER)
        .center(0.0, S.CASE_H - S.WALL_TOP / 2.0)
        .rect(S.CASE_W + 2.0, S.WALL_TOP)
        .extrude(25.35)
    )
    top_l_wall = full_wall_envelope.intersect(top_band)
    plate = plate.union(top_l_wall)

    # Corner L-Wall extensions wrapping Top-Left and Top-Right Cover M3 sockets (Y=169.0..177.0)
    for sign in (-1.0, 1.0):
        corner_blk = (
            cq.Workplane("XY")
            .workplane(offset=S.Z_PLATE_OUTER)
            .center(sign * (S.CASE_W / 2.0 - 4.7), 173.0)
            .rect(9.4, 8.2)
            .extrude(25.35)
        ).intersect(full_wall_envelope)
        plate = plate.union(corner_blk)

    # 3. Integrated Right Button-Plunger & MicroSD Flange (X = 57.0..60.0, Y = 16.0..96.0)
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

    # Plunger guide block extending inward off Right Flange (X=46.6..57.0, Y=22..50, Z=-25.0..-4.2)
    blk_cx = 0.5 * (S.PLUNGER_BLK_X0 + S.PLUNGER_BLK_X1)
    blk_w = S.PLUNGER_BLK_X1 - S.PLUNGER_BLK_X0
    blk_cy = 0.5 * (S.PLUNGER_BLK_Y0 + S.PLUNGER_BLK_Y1)
    blk_h = S.PLUNGER_BLK_Y1 - S.PLUNGER_BLK_Y0
    plunger_blk = (
        cq.Workplane("XY")
        .workplane(offset=S.Z_PLATE_INNER - 0.5)
        .center(blk_cx, blk_cy)
        .rect(blk_w + 0.5, blk_h)
        .extrude(S.PLUNGER_BLK_Z_TOP - S.Z_PLATE_INNER + 0.5)
    )
    plate = plate.union(plunger_blk)

    # 4. Left & Bottom thin perimeter registration tabs (Z = -25.0..-21.2, 0.25 mm inside Top_Cover walls)
    reg_tabs = [
        (-56.0, 46.0, 1.6, 12.0),   # Left lower between Speaker & Mid-Left
        (-56.0, 154.0, 1.6, 12.0),  # Left upper near GX-12
        (-10.0, 4.0, 12.0, 1.6),    # Bottom left of USB-C
        ( 42.0, 4.0, 10.0, 1.6),    # Bottom right of USB-C
    ]
    for tx, ty, tw, th in reg_tabs:
        tab = (
            cq.Workplane("XY")
            .workplane(offset=S.Z_PLATE_INNER - 0.5)
            .center(tx, ty)
            .rect(tw, th)
            .extrude(4.2)
        )
        plate = plate.union(tab)

    # 5. Symmetric 6 x Front-Entry M3 Closure Bosses (Top_Cover columns drop to -13.0 -> boss top -13.2)
    # Bottom & Mid bosses rise Z=-25.0..-13.2 (h=14.8 from -28.0); Top corner sockets bored in L-wall!
    boss_z_top = -13.2
    csk_holes = []
    for mx, my in S.COVER_M3_POINTS:
        is_top_corner = my > 165.0
        if is_top_corner:
            # Cut upper socket cylinder Dia 8.1 mm from Z=-2.4 down to boss_z_top=-13.2 for Top_Cover tube
            sock = (
                cq.Workplane("XY")
                .workplane(offset=boss_z_top)
                .center(mx, my)
                .circle(4.05)
                .extrude(11.5)
            )
            plate = plate.cut(sock)
            # Drill M3 Ruthex insert bore from boss_z_top=-13.2 down 7.0 mm (leaves 7.8 mm closed floor)
            bore = (
                cq.Workplane("XY")
                .workplane(offset=boss_z_top - S.RUTHEX_M3_DEPTH - 0.3)
                .center(mx, my)
                .circle(S.RUTHEX_M3_OD / 2.0)
                .extrude(S.RUTHEX_M3_DEPTH + 0.6)
            )
            csk_holes.append(bore)
        else:
            b, bore = make_ruthex_boss(
                mx, my, z_base=S.Z_PLATE_OUTER + 0.5,
                boss_h=(boss_z_top - S.Z_PLATE_OUTER - 0.5),
                boss_od=S.RUTHEX_M3_BOSS_OD, hole_od=S.RUTHEX_M3_OD,
                hole_depth=S.RUTHEX_M3_DEPTH + 0.3, open_at_top=True,
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

    # 7. Waveshare 3.5B Pillars (4 x Dia 5.0 mm standoffs up to -11.50, rear M2 CSK pass-throughs)
    ws_pillar_h = (S.WS_STANDOFF_Z_END - S.Z_PLATE_INNER) + 0.5
    ws_bores = []
    for sx in (-S.WS_STANDOFF_DX / 2.0, S.WS_STANDOFF_DX / 2.0):
        for sy in (-S.WS_STANDOFF_DY / 2.0, S.WS_STANDOFF_DY / 2.0):
            px, py = S.WS_CX + sx, S.WS_CY + sy
            pil = (
                cq.Workplane("XY")
                .workplane(offset=S.Z_PLATE_INNER - 0.5)
                .center(px, py)
                .circle(2.5)
                .extrude(ws_pillar_h)
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

    # 9. Power Board Standoffs (4 x M2 Ruthex bosses, top at Z = -22.95, flush with -2.0 mm back trace clearance)
    pb_top_z = S.Z_PLATE_INNER + S.PB_STANDOFF_H  # -22.95
    pb_bores = []
    for dx in (-S.PB_HOLE_DX / 2.0, S.PB_HOLE_DX / 2.0):
        for dy in (-S.PB_HOLE_DY / 2.0, S.PB_HOLE_DY / 2.0):
            px, py = S.PB_CX + dx, S.PB_CY + dy
            b, bore = make_ruthex_boss(
                px, py, z_base=S.Z_PLATE_OUTER + 0.5,
                boss_h=(pb_top_z - S.Z_PLATE_OUTER - 0.5),
                boss_od=S.RUTHEX_M2_BOSS_OD, hole_od=S.RUTHEX_M2_OD,
                hole_depth=4.50, open_at_top=True,
            )
            plate = plate.union(b)
            pb_bores.append(bore)
    for bore in pb_bores:
        plate = plate.cut(bore)

    # 10. E220 LoRa Standoffs (4 x M2 Ruthex bosses, top at Z = -21.95, flush with -3.0 mm back trace clearance)
    e2_top_z = S.Z_PLATE_INNER + S.E220_STANDOFF_H  # -21.95
    e2_bores = []
    for dx in (-S.E220_HOLE_DX / 2.0, S.E220_HOLE_DX / 2.0):
        for dy in (-S.E220_HOLE_DY / 2.0, S.E220_HOLE_DY / 2.0):
            px, py = S.E220_CX + dx, S.E220_CY + dy
            b, bore = make_ruthex_boss(
                px, py, z_base=S.Z_PLATE_OUTER + 0.5,
                boss_h=(e2_top_z - S.Z_PLATE_OUTER - 0.5),
                boss_od=S.RUTHEX_M2_BOSS_OD, hole_od=S.RUTHEX_M2_OD,
                hole_depth=S.RUTHEX_M2_DEPTH, open_at_top=True,
            )
            plate = plate.union(b)
            e2_bores.append(bore)
    for bore in e2_bores:
        plate = plate.cut(bore)

    # 11. Holybro UM982 Standoffs (4 x Dia 5.4 mm up to Z = -17.20, rear M2 CSK pass-throughs)
    um_pillar_h = (S.UM_BACK_Z - S.Z_PLATE_INNER) + 0.5
    um_bores = []
    for dx in S.UM_M2_X_OFFS:
        for dy in S.UM_M2_Y_OFFS:
            px, py = S.UM_LEFT_X + dx, S.UM_BOT_Y + dy
            pil = (
                cq.Workplane("XY")
                .workplane(offset=S.Z_PLATE_INNER - 0.5)
                .center(px, py)
                .circle(2.7)
                .extrude(um_pillar_h)
            )
            plate = plate.union(pil)
            um_bores.append(make_csk_cone_through(
                px, py, S.Z_PLATE_OUTER, S.UM_BACK_Z - S.Z_PLATE_OUTER,
                S.HOLE_CLEAR_M2, 4.6, cone_from_bed=True
            ))
    for bore in um_bores:
        plate = plate.cut(bore)

    # 12. Speaker Pedestal + Perimeter Cradle (top support at Z = -13.50, notched +Y for wires)
    spk_ped_top = S.Z_PLATE_INNER + S.SPK_PEDESTAL_H  # -13.50
    spk_outer = (
        cq.Workplane("XY")
        .workplane(offset=S.Z_PLATE_INNER - 0.5)
        .center(S.SPK_CX, S.SPK_CY)
        .rect(S.SPK_W + 3.0, S.SPK_H + 3.0)
        .extrude(S.SPK_PEDESTAL_H + 4.0)
    )
    # Core out speaker cavity above pedestal top (-13.50) AND lighten base (-23.0..-13.50)
    spk_seat_pocket = (
        cq.Workplane("XY")
        .workplane(offset=spk_ped_top)
        .center(S.SPK_CX, S.SPK_CY)
        .rect(S.SPK_W + 0.4, S.SPK_H + 0.4)
        .extrude(5.0)
    )
    spk_wire_notch = (
        cq.Workplane("XY")
        .workplane(offset=spk_ped_top)
        .center(S.SPK_CX, S.SPK_CY + S.SPK_H / 2.0 + 1.0)
        .rect(9.0, 4.0)
        .extrude(5.0)
    )
    spk_hollow = (
        cq.Workplane("XY")
        .workplane(offset=S.Z_PLATE_INNER)
        .center(S.SPK_CX, S.SPK_CY)
        .rect(18.0, 18.0)
        .extrude(S.SPK_PEDESTAL_H - 2.0)
    )
    spk_cradle = spk_outer.cut(spk_seat_pocket).cut(spk_wire_notch).cut(spk_hollow)
    plate = plate.union(spk_cradle)

    # 13. Cutouts on Back Plate & Integral L-Wall & Right Flange
    # 13a. Battery JST-PH wire pass-through slot
    bat_slot = make_stadium_slot(S.BAT_SLOT_CX, S.BAT_SLOT_CY,
                                 S.BAT_SLOT_W, S.BAT_SLOT_H,
                                 S.Z_PLATE_OUTER - 0.5, S.WALL_BACK + 1.0)
    plate = plate.cut(bat_slot)

    # 13b. Top L-Wall Bulkhead Cutouts (UM982 local 2.0 mm roof web, GX-12, E220 SMA, UM982 SMAs)
    # NOTE: peak_at_plus_z=True because Back_Chassis_Frame prints back-down on Z=-28!
    um_recess = (
        cq.Workplane("XY")
        .workplane(offset=S.Z_PLATE_INNER)
        .center(S.UM_CX, 177.25)
        .rect(S.UM_W + 0.6, 1.50)
        .extrude(25.0)
    )
    plate = plate.cut(um_recess)

    # GX-12 M12 bore + inner nut relief (pure round cylindrical bores, leaves 2.6 mm web Y = 177.4..180.0)
    gx_bore = make_teardrop_y_bore(
        S.GX12_X, y_start=176.5, y_len=4.5, z_ctr=S.GX12_Z,
        dia=S.GX12_HOLE_DIA, add_teardrop=False
    )
    gx_rel = make_teardrop_y_bore(
        S.GX12_X, y_start=174.0, y_len=180.0 - S.GX12_WALL - 174.0, z_ctr=S.GX12_Z,
        dia=S.GX12_NUT_RECESS_DIA, add_teardrop=False
    )
    plate = plate.cut(gx_bore).cut(gx_rel)

    # E220 SMA bore (pure round cylindrical bore, leaves 2.6 mm web Y = 177.4..180.0)
    e2_sma_bore = make_teardrop_y_bore(
        S.E220_SMA_X, y_start=176.5, y_len=4.5, z_ctr=S.E220_SMA_Z,
        dia=S.E220_SMA_DIA, add_teardrop=False
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

    # 13c. Right Flange Tactile Button Plunger Bores (pure round cylindrical) + Captive Pockets + MicroSD Slot (gabled)
    for by in S.WS_BTN_Y_LIST:
        # Outer round plunger stem bore (Dia 4.60 mm, X = 50.8..60.8)
        outer_bore = (
            cq.Workplane("YZ")
            .workplane(offset=50.8)
            .center(by, S.WS_BTN_Z_CTR)
            .circle(2.30)
            .extrude(10.0)
        )
        # Inner round plunger nose bore (Dia 3.40 mm, X = 46.2..48.8)
        inner_bore = (
            cq.Workplane("YZ")
            .workplane(offset=46.2)
            .center(by, S.WS_BTN_Z_CTR)
            .circle(1.70)
            .extrude(2.6)
        )
        # Drop-in vertical pocket for captive collar flange (X = 48.2..51.2, open upward to Z=-4.2)
        pocket = (
            cq.Workplane("XY")
            .workplane(offset=S.WS_BTN_Z_CTR - 3.3)
            .center(49.7, by)
            .rect(3.0, 6.8)
            .extrude(10.0)
        )
        plate = plate.cut(outer_bore).cut(inner_bore).cut(pocket)

    # MicroSD slot cutout + external finger scoop chamfer on Right Flange
    sd_cy = 0.5 * (S.WS_SD_Y0 + S.WS_SD_Y1)
    sd_h_y = S.WS_SD_Y1 - S.WS_SD_Y0
    sd_cut = make_arch_x_cutout(
        56.2, 4.5, sd_cy, width_y=sd_h_y,
        z_min=S.WS_SD_Z_CTR - 2.6, z_max=S.WS_SD_Z_CTR + 2.6,
        roof_gable=1.6, peak_at_plus_z=True
    )
    sd_scoop = make_arch_x_cutout(
        58.8, 2.0, sd_cy, width_y=sd_h_y + 3.0,
        z_min=S.WS_SD_Z_CTR - 3.5, z_max=S.WS_SD_Z_CTR + 3.5,
        roof_gable=2.0, peak_at_plus_z=True
    )
    plate = plate.cut(sd_cut).cut(sd_scoop)

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

    # Notch out Top L-Wall crown (Y = 176.8..180.5) & Top Corner sockets (Y = 168.8..177.2)
    # behind Z_FRONT_INNER (-2.4) so Top_Cover lifts STRAIGHT UP off bulkhead connectors!
    top_notch = (
        cq.Workplane("XY")
        .workplane(offset=z_cover_bot - 0.5)
        .center(0.0, 178.7)
        .rect(S.CASE_W + 2.0, 4.0)
        .extrude(cover_depth - S.WALL_FRONT + 0.5)
    )
    cover = cover.cut(top_notch)
    for sign in (-1.0, 1.0):
        corner_notch = (
            cq.Workplane("XY")
            .workplane(offset=z_cover_bot - 0.5)
            .center(sign * (S.CASE_W / 2.0 - 5.2), 173.0)
            .rect(11.0, 8.4)
            .extrude(cover_depth - S.WALL_FRONT + 0.5)
        )
        cover = cover.cut(corner_notch)

    # Notch out Right Button-Plunger & MicroSD Flange window (Y = 15.8..96.2)
    right_notch = (
        cq.Workplane("XY")
        .workplane(offset=z_cover_bot - 0.5)
        .center(S.CASE_W / 2.0 - 1.2, 0.5 * (S.FLANGE_Y0 + S.FLANGE_Y1))
        .rect(S.WALL_SIDE + 2.0, (S.FLANGE_Y1 - S.FLANGE_Y0) + 2.0 * S.NOTCH_CLEAR)
        .extrude(cover_depth - S.WALL_FRONT + 0.5)
    )
    cover = cover.cut(right_notch)

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

    # 6 x Symmetric Front-Entry M3 Screw Landing Columns (extend Z=0.0 down to -13.0 mm)
    # Below Z=-13.0, cut Dia 8.2 mm wall relief around lower/mid Back_Chassis_Frame M3 bosses.
    col_len = 13.0
    for mx, my in S.COVER_M3_POINTS:
        if my < 165.0:
            boss_pocket = (
                cq.Workplane("XY")
                .workplane(offset=z_cover_bot - 0.5)
                .center(mx, my)
                .circle(4.10)
                .extrude((abs(z_cover_bot) - col_len) + 0.5)
            )
            cover = cover.cut(boss_pocket)
        col = (
            cq.Workplane("XY")
            .workplane(offset=-col_len)
            .center(mx, my)
            .circle(3.8)
            .extrude(col_len - 0.5)
        )
        cover = cover.union(col)

    for mx, my in S.COVER_M3_POINTS:
        csk = make_csk_cone_through(mx, my, z_bed=-col_len, plate_thk=col_len,
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
# Parts 3-6: Option B2 2-Piece External Battery & Pole Mount Pods (5000 & 10000 mAh)
# ==============================================================================
def build_battery_pod_pair(variant: str) -> tuple[cq.Workplane, cq.Workplane]:
    """Build (Battery_Cradle, Pole_Lid) for '5000' or '10000' mAh sharing POD_M3_POINTS.

    Battery_Cradle is skeletonized into a 2.0 mm outer frame, 2.0 mm continuous inner
    LiPo containment wall, 4 x Dia 8.6 mm M3 bolt sleeves, and a 1.8 mm interlocking
    Warren-truss web (100% flat-bed support-free extrusion). Pole_Lid stays solid 3.0 mm
    to seal the truss cells and shield the LiPo pouch on a survey pole.
    """
    if variant == "5000":
        bay_w, bay_h, bay_d, pod_h = S.BAT5_BAY_W, S.BAT5_BAY_H, S.BAT5_BAY_D, S.BAT5_POD_H
    elif variant == "10000":
        bay_w, bay_h, bay_d, pod_h = S.BAT10_BAY_W, S.BAT10_BAY_H, S.BAT10_BAY_D, S.BAT10_POD_H
    else:
        raise ValueError(f"Unknown battery pod variant: {variant}")

    z_cradle_front = S.Z_PLATE_OUTER          # -28.0 (100% flat build-plate face)
    z_cradle_back = z_cradle_front - bay_d    # -39.2 (5000) or -40.8 (10000), 100% flat mate
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
        iy = in_yb if sy < 0 else in_yt
        oy = out_yb if sy < 0 else out_yt
        add_web(make_strut(ox, py, ix, py))
        add_web(make_strut(px, oy, px, py))
        add_web(make_strut(px, py, ix, max(in_yb, min(in_yt, py - sy * 16.0))))
        if variant == "5000":
            add_web(make_strut(px, py, px - sx * 14.0, iy))

    # Left & Right side Warren-truss zigzag web + spine load-center horizontal tie
    n_bays = 4 if variant == "5000" else 6
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
    if variant == "5000":
        n_tb = 4
        xs = [in_xl + i * (in_xr - in_xl) / n_tb for i in range(n_tb + 1)]
        for y_in, y_out in ((in_yb, out_yb), (in_yt, out_yt)):
            for i in range(n_tb):
                xa, xb = xs[i], xs[i + 1]
                if i % 2 == 0:
                    add_web(make_strut(xa, y_out, xb, y_in))
                else:
                    add_web(make_strut(xa, y_in, xb, y_out))
            add_web(make_strut(S.POD_CX, y_out, S.POD_CX, y_in))
    else:
        for tx in (S.POD_CX - 20.0, S.POD_CX, S.POD_CX + 20.0):
            add_web(make_strut(tx, out_yb, tx, in_yb))
            add_web(make_strut(tx, in_yt, tx, out_yt))
        for side in (-1, 1):
            ix = in_xl if side < 0 else in_xr
            ox = out_xl if side < 0 else out_xr
            add_web(make_strut(ox, out_yb + 2.0, ix, y_lo))
            add_web(make_strut(ox, out_yt - 2.0, ix, y_hi))

    cradle = cradle.union(web.intersect(annulus)).cut(inner_void)

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
    """Build 3 captive round tactile button plungers (PWR, RESET, BOOT), 45-deg self-supporting."""
    comp = None
    for by in S.WS_BTN_Y_LIST:
        # 1. Inner nose reaching near button dome (X = +45.75..+48.6, 0.40 mm gap to dome +45.35)
        nose = (
            cq.Workplane("YZ")
            .workplane(offset=45.75)
            .center(by, S.WS_BTN_Z_CTR)
            .circle(1.40)
            .extrude(2.85)
        )
        # 2. Tapered 45-deg captive collar transitioning smoothly down onto round stem (zero 90-deg overhang!)
        collar = (
            cq.Workplane("YZ")
            .workplane(offset=48.55)
            .center(by, S.WS_BTN_Z_CTR)
            .rect(6.10, 5.20)
            .workplane(offset=1.00)
            .rect(6.10, 5.20)
            .workplane(offset=1.25)
            .circle(2.05)
            .loft(combine=True)
        )
        # 3. Pure round cylindrical stem sticking 1.2 mm proud of right wall (Dia 4.10 mm, X = +50.75..+61.20)
        stem = (
            cq.Workplane("YZ")
            .workplane(offset=50.75)
            .center(by, S.WS_BTN_Z_CTR)
            .circle(2.05)
            .extrude(10.45)
        )
        one = nose.union(collar).union(stem)
        comp = one if comp is None else comp.add(one)
    return comp


# ==============================================================================
# Real REF-*.stp Model Importers & Normalized Assembly Transformers
# ==============================================================================
def load_normalized_refs() -> dict[str, cq.Workplane]:
    """Load all REF-*.stp hardware models and place them in exact assembly coordinates."""
    refs = {}

    # 1. Waveshare 3.5B (fixed R6.0 body REF-Waveshare...stp + 2x16 angled header Solid[4] from ASM)
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

    # 3. Power Board (PCB Z=0 -> Z_PLATE_INNER + PB_STANDOFF_H = -22.0)
    pm = cq.importers.importStep(str(REF_FILES["power"]))
    pbb = pm.val().BoundingBox()
    pm_cx, pm_cy = 0.5 * (pbb.xmin + pbb.xmax), 0.5 * (pbb.ymin + pbb.ymax)
    refs["power"] = pm.translate((S.PB_CX - pm_cx, S.PB_CY - pm_cy, S.Z_PLATE_INNER + S.PB_STANDOFF_H))

    # 4. E220 Module (PCB Z=0 -> Z_PLATE_INNER + E220_STANDOFF_H = -21.0)
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

    # 7. GX-12 Compass Bulkhead (rotated -90 deg around X so +Z points +Y out crown; shoulder Z_ref=-0.9 at Y=180.0)
    gx = cq.importers.importStep(str(REF_FILES["gx12"]))
    gbb = gx.val().BoundingBox()
    gx_cx, gx_cy = 0.5 * (gbb.xmin + gbb.xmax), 0.5 * (gbb.ymin + gbb.ymax)
    gx_rot = gx.translate((-gx_cx, -gx_cy, 0.90)).rotate((0, 0, 0), (1, 0, 0), -90)
    refs["gx12"] = gx_rot.translate((S.GX12_X, S.CASE_H, S.GX12_Z))

    # 8. E220 LoRa Female SMA Bulkhead (rotated -90 deg X; inner hex flange face Z_ref=2.0 at Y=177.40)
    sma = cq.importers.importStep(str(REF_FILES["sma"]))
    smabb = sma.val().BoundingBox()
    sma_cx, sma_cy = 0.5 * (smabb.xmin + smabb.xmax), 0.5 * (smabb.ymin + smabb.ymax)
    sma_rot = sma.translate((-sma_cx, -sma_cy, -2.0)).rotate((0, 0, 0), (1, 0, 0), -90)
    refs["sma"] = sma_rot.translate((S.E220_SMA_X, S.CASE_H - S.E220_SMA_WALL, S.E220_SMA_Z))

    # 9. 5000 mAh LiPo Battery (rotated 180 deg Z so wire tab sits bottom-left over BAT_SLOT)
    b5 = cq.importers.importStep(str(REF_FILES["bat5000"]))
    b5b = b5.val().BoundingBox()
    b5_rot = b5.translate((-0.5 * (b5b.xmin + b5b.xmax), -0.5 * (b5b.ymin + b5b.ymax), -b5b.zmax)).rotate((0, 0, 0), (0, 0, 1), 180)
    refs["bat5000"] = b5_rot.translate((S.POD_CX, S.POD_CY + 5.0, S.Z_PLATE_OUTER))

    # 10. 10000 mAh LiPo Battery (offset 1.5 mm off Back Plate inside 12.8 mm bay for wire routing)
    b10 = cq.importers.importStep(str(REF_FILES["bat10000"]))
    b10b = b10.val().BoundingBox()
    b10_norm = b10.translate((-0.5 * (b10b.xmin + b10b.xmax), -0.5 * (b10b.ymin + b10b.ymax), -b10b.zmax))
    refs["bat10000"] = b10_norm.translate((S.POD_CX, S.POD_CY + 1.0, S.Z_PLATE_OUTER - 1.5))

    return refs


# ==============================================================================
# Print Orientation Helpers & Exporters
# ==============================================================================
def to_print_bed(shape: cq.Workplane, mode: str) -> cq.Workplane:
    """Orient part flat onto Z = 0.0 build plate for support-free PETG printing."""
    if mode == "face_down":
        s = shape.rotate((0, 0, 0), (1, 0, 0), 180)
    elif mode == "plunger_flat":
        s = shape.rotate((0, 0, 0), (0, 1, 0), 90)
    else:
        s = shape
    bb = s.val().BoundingBox()
    return s.translate((0.0, 0.0, -bb.zmin))


def verify_assembly_and_refs(parts: dict[str, cq.Workplane], refs: dict[str, cq.Workplane]) -> None:
    """Run automated manifold, outer envelope, and 0.00 mm^3 real-model collision audits."""
    print("\n--- [1/3] Manifold & Outer Envelope Checks ---")
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

    print("\n--- [2/3] Printed Part vs Printed Part Interference Audit ---")
    printed_pairs = [
        ("Back_Chassis_Frame", "Top_Cover"),
        ("Top_Cover", "Slim_Sun_Shield"),
        ("Back_Chassis_Frame", "Slim_Sun_Shield"),
        ("Back_Chassis_Frame", "Button_Plungers"),
        ("Top_Cover", "Button_Plungers"),
        ("Back_Chassis_Frame", "Battery_Cradle_5000"),
        ("Battery_Cradle_5000", "Pole_Lid_5000"),
        ("Back_Chassis_Frame", "Pole_Lid_5000"),
        ("Back_Chassis_Frame", "Battery_Cradle_10000"),
        ("Battery_Cradle_10000", "Pole_Lid_10000"),
        ("Back_Chassis_Frame", "Pole_Lid_10000"),
    ]
    for a_name, b_name in printed_pairs:
        inter = parts[a_name].intersect(parts[b_name])
        vol = sum(s.Volume() for s in inter.solids().vals())
        print(f"  {a_name:22s} vs {b_name:22s}: overlap = {vol:8.4f} mm^3")
        assert vol <= 0.05, f"Interference failure: {a_name} vs {b_name} = {vol:.4f} mm^3"

    print("\n--- [3/3] Printed Enclosure vs Real REF-*.stp Hardware Audit ---")
    ref_checks = [
        ("Back_Chassis_Frame", "waveshare"),
        ("Top_Cover",          "waveshare"),
        ("Button_Plungers",    "waveshare"),
        ("Back_Chassis_Frame", "coin_cell"),
        ("Back_Chassis_Frame", "power"),
        ("Top_Cover",          "power"),
        ("Back_Chassis_Frame", "e220"),
        ("Top_Cover",          "e220"),
        ("Back_Chassis_Frame", "um982"),
        ("Top_Cover",          "um982"),
        ("Back_Chassis_Frame", "speaker"),
        ("Top_Cover",          "speaker"),
        ("Back_Chassis_Frame", "gx12"),
        ("Top_Cover",          "gx12"),
        ("Back_Chassis_Frame", "sma"),
        ("Top_Cover",          "sma"),
        ("Battery_Cradle_5000", "bat5000"),
        ("Pole_Lid_5000",       "bat5000"),
        ("Back_Chassis_Frame",  "bat5000"),
        ("Battery_Cradle_10000", "bat10000"),
        ("Pole_Lid_10000",       "bat10000"),
        ("Back_Chassis_Frame",   "bat10000"),
    ]
    for p_name, r_name in ref_checks:
        inter = parts[p_name].intersect(refs[r_name])
        vol = sum(s.Volume() for s in inter.solids().vals())
        print(f"  {p_name:22s} vs REF_{r_name:12s}: overlap = {vol:8.4f} mm^3")
        assert vol <= 0.05, f"Hardware collision: {p_name} vs REF_{r_name} = {vol:.4f} mm^3"


def export_2d_dxf(path: Path) -> None:
    """Export AutoCAD LT 2010 compatible 2D DXF of v2.0.0 chassis, pods, and crown."""
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
    add_rect(S.E220_CX, S.E220_CY, S.E220_W, S.E220_H, "MODULES")
    add_rect(S.UM_CX, S.UM_CY, S.UM_W, S.UM_BODY_H, "MODULES")
    msp.add_circle((S.RTC_CX, S.RTC_CY), 13.25, dxfattribs={"layer": "MODULES"})

    # Battery slot & Option B2 pods
    add_rect(S.BAT_SLOT_CX, S.BAT_SLOT_CY, S.BAT_SLOT_W, S.BAT_SLOT_H, "POD_B2")
    add_rect(S.POD_CX, S.POD_CY, S.POD_OUTER_W, S.BAT5_POD_H, "POD_B2")
    add_rect(S.POD_CX, S.POD_CY, S.POD_OUTER_W, S.BAT10_POD_H, "POD_B2")
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
    msp.add_circle((S.E220_SMA_X, y_off + S.E220_SMA_Z), S.E220_SMA_DIA / 2.0, dxfattribs={"layer": "CROWN_BULKHEAD"})
    msp.add_circle((S.UM_SMA_SLAVE_X, y_off + S.UM_SMA_Z), S.UM_SMA_ACCESS_DIA / 2.0, dxfattribs={"layer": "CROWN_BULKHEAD"})
    msp.add_circle((S.UM_SMA_MASTER_X, y_off + S.UM_SMA_Z), S.UM_SMA_ACCESS_DIA / 2.0, dxfattribs={"layer": "CROWN_BULKHEAD"})

    doc.saveas(str(path))
    print(f"  Exported DXF -> {path.name}")


def main() -> None:
    print(f"=== {__filename__} {__version__} ===")
    STEP_DIR.mkdir(parents=True, exist_ok=True)
    STL_DIR.mkdir(parents=True, exist_ok=True)
    DXF_DIR.mkdir(parents=True, exist_ok=True)

    print("Building parametric v2.0.0 enclosure parts...")
    cradle_5000, lid_5000 = build_battery_pod_pair("5000")
    cradle_10000, lid_10000 = build_battery_pod_pair("10000")

    parts = {
        "Back_Chassis_Frame": build_back_chassis_frame(),
        "Top_Cover": build_top_cover(),
        "Battery_Cradle_5000": cradle_5000,
        "Pole_Lid_5000": lid_5000,
        "Battery_Cradle_10000": cradle_10000,
        "Pole_Lid_10000": lid_10000,
        "Slim_Sun_Shield": build_slim_sun_shield(),
        "Button_Plungers": build_button_plungers(),
    }

    print("Importing all 11 REF-*.stp hardware reference models...")
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
        ("Battery_Cradle_5000",  "bat_cradle_5000_print.stl", "face_down"),
        ("Pole_Lid_5000",        "pole_lid_5000_print.stl",   "face_down"),
        ("Battery_Cradle_10000", "bat_cradle_10000_print.stl","face_down"),
        ("Pole_Lid_10000",       "pole_lid_10000_print.stl",  "face_down"),
        ("Slim_Sun_Shield",      "slim_sun_shield_print.stl", "back_down"),
        ("Button_Plungers",      "button_plungers_print.stl", "plunger_flat"),
    ]
    for name, stl_name, orient in stl_map:
        stl_path = STL_DIR / stl_name
        p_shp = to_print_bed(parts[name], orient)
        cq.exporters.export(p_shp, str(stl_path), tolerance=0.05, angularTolerance=0.1)
        print(f"  Exported STL  -> {stl_name}")

    # Export full SolidEdge B-Rep assemblies (5000 mAh & 10000 mAh) with real REF-*.stp hardware
    for variant, bat_key, cradle_key, lid_key in [
        ("5000mAh", "bat5000", "Battery_Cradle_5000", "Pole_Lid_5000"),
        ("10000mAh", "bat10000", "Battery_Cradle_10000", "Pole_Lid_10000"),
    ]:
        assy = cq.Assembly(name=f"RTK_Rover_Assembly_{variant}")
        for pname, color in [
            ("Back_Chassis_Frame", cq.Color(0.22, 0.25, 0.29, 1.0)),
            ("Top_Cover",          cq.Color(0.28, 0.34, 0.42, 0.92)),
            (cradle_key,           cq.Color(0.18, 0.22, 0.27, 1.0)),
            (lid_key,              cq.Color(0.14, 0.17, 0.21, 1.0)),
            ("Slim_Sun_Shield",    cq.Color(0.12, 0.14, 0.17, 1.0)),
            ("Button_Plungers",    cq.Color(0.90, 0.45, 0.10, 1.0)),
        ]:
            assy.add(parts[pname], name=pname, color=color)
        for rname, rcolor in [
            ("waveshare", cq.Color(0.08, 0.35, 0.25, 1.0)),
            ("um982",     cq.Color(0.10, 0.10, 0.12, 1.0)),
            ("e220",      cq.Color(0.12, 0.42, 0.22, 1.0)),
            ("power",     cq.Color(0.12, 0.28, 0.55, 1.0)),
            ("speaker",   cq.Color(0.18, 0.18, 0.18, 1.0)),
            ("coin_cell", cq.Color(0.85, 0.72, 0.15, 1.0)),
            ("gx12",      cq.Color(0.75, 0.76, 0.78, 1.0)),
            ("sma",       cq.Color(0.82, 0.65, 0.18, 1.0)),
            (bat_key,     cq.Color(0.85, 0.70, 0.10, 1.0)),
        ]:
            assy.add(refs[rname], name=f"REF_{rname}", color=rcolor)
        assy_path = STEP_DIR / f"RTK_Rover_Assembly_{variant}.step"
        assy.export(str(assy_path))
        print(f"  Exported ASSY -> {assy_path.name}")

    export_2d_dxf(DXF_DIR / "RTK_Rover_Interfaces_2D.dxf")
    print(f"\n=== ALL CAD {__version__} CHECKS & EXPORTS PASSED! ===")


if __name__ == "__main__":
    main()
