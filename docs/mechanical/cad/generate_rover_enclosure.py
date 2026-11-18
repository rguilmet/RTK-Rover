"""Generate parametric 3D STEP/STL CAD models and 2D DXF drawings for RTK Rover.

Outputs under <this_dir>/:
  step/  - SolidEdge-compatible B-Rep STEP files (6 parts + full assembly)
  stl/   - Print-ready STL meshes rotated flat on build plate (support-free)
  dxf/   - AutoCAD LT 2010 compatible 2D footprint & bulkhead drawings
"""

import math
import os
from pathlib import Path

import cadquery as cq
import ezdxf

import rover_specs as S

__filename__ = os.path.basename(__file__)
__version__ = "v1.1.0"

CAD_DIR = Path(__file__).resolve().parent
STEP_DIR = CAD_DIR / "step"
STL_DIR = CAD_DIR / "stl"
DXF_DIR = CAD_DIR / "dxf"
STP_WAVESHARE = CAD_DIR.parent / "ESP32-S3-Touch-LCD-3_5B.stp"


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
                         add_teardrop: bool = True) -> cq.Workplane:
    """Create a vertical-wall horizontal Y-axis bore safe for face-down printing.

    In print space (Front_Shell rotated 180 deg around X), -Z_asm points UP (+Z_print).
    A top D-flat or 45-deg teardrop peak sits on the -Z_asm side so bridges stay <= 45 deg.
    """
    r = dia / 2.0
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
            (cx - r / math.sqrt(2.0), z_ctr - r / math.sqrt(2.0)),
            (cx, z_ctr - tip),
            (cx + r / math.sqrt(2.0), z_ctr - r / math.sqrt(2.0)),
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
        flat_z = z_ctr - (top_flat_dist - r)
        slab_h = max(0.5, (z_ctr - r) - flat_z + 2.0)
        clip = (
            cq.Workplane("XY")
            .workplane(offset=flat_z - slab_h)
            .center(cx, y_start + y_len / 2.0)
            .rect(dia + 2.0, y_len + 2.0)
            .extrude(slab_h)
        )
        bore = bore.cut(clip)
    return bore


def make_arch_x_cutout(x_start: float, x_len: float, cy: float, width_y: float,
                       z_front: float, z_back: float, roof_gable: float = 1.6) -> cq.Workplane:
    """Create a horizontal port cutout through a +-X wall with a 45-deg roof gable at -Z_asm."""
    pts = [
        (cy - width_y / 2.0, z_front),
        (cy + width_y / 2.0, z_front),
        (cy + width_y / 2.0, z_back),
        (cy, z_back - roof_gable),
        (cy - width_y / 2.0, z_back),
    ]
    return (
        cq.Workplane("YZ")
        .workplane(offset=x_start)
        .polyline(pts)
        .close()
        .extrude(x_len)
    )


def build_front_shell() -> cq.Workplane:
    """Build Front_Shell (Z = 0.0 outer faceplate down to Z_PARTING = -24.0)."""
    case_cy = S.CASE_H / 2.0
    depth = abs(S.Z_PARTING)  # 24.0 mm
    outer = make_rounded_box(S.CASE_W, S.CASE_H, depth, S.CORNER_R,
                             cx=0.0, cy=case_cy, z_min=S.Z_PARTING)

    # Interior cavity leaving SIDE/BOT/TOP walls and FRONT faceplate (WALL_FRONT=2.4)
    in_w = S.CASE_W - 2.0 * S.WALL_SIDE
    in_y_min = S.WALL_BOT
    in_y_max = S.CASE_H - S.WALL_TOP
    in_h = in_y_max - in_y_min
    in_cy = 0.5 * (in_y_min + in_y_max)
    cavity = make_rounded_box(in_w, in_h, depth - S.WALL_FRONT + 0.5,
                              max(1.5, S.CORNER_R - S.WALL_SIDE),
                              cx=0.0, cy=in_cy, z_min=S.Z_PARTING - 0.5)
    shell = outer.cut(cavity)

    # Perimeter tongue-and-groove lip step at parting rim (1.4 mm outer lap rim)
    lip_h = 1.6
    lip_cutter = make_rounded_box(S.CASE_W - 2.0 * 1.4, S.CASE_H - 2.0 * 1.4, lip_h + 0.2,
                                  max(1.5, S.CORNER_R - 1.4),
                                  cx=0.0, cy=case_cy, z_min=S.Z_PARTING - 0.2)
    shell = shell.cut(lip_cutter)

    # 1. Sun_Shield Magnet + Alignment Pin Pads (4 corners around LCD, zero fragile clips)
    for mx, my in S.SHIELD_MAG_POINTS:
        pad = (
            cq.Workplane("XY")
            .workplane(offset=-5.0)
            .center(mx, my)
            .circle(S.SHIELD_MAG_DIA / 2.0 + 1.6)
            .extrude(5.0)
        )
        shell = shell.union(pad)
        mag_hole = (
            cq.Workplane("XY")
            .workplane(offset=-S.SHIELD_MAG_DEPTH)
            .center(mx, my)
            .circle(S.SHIELD_MAG_DIA / 2.0)
            .extrude(S.SHIELD_MAG_DEPTH + 0.2)
        )
        shell = shell.cut(mag_hole)

    for px, py in S.SHIELD_PIN_POINTS:
        pad = (
            cq.Workplane("XY")
            .workplane(offset=-4.5)
            .center(px, py)
            .circle(2.6)
            .extrude(4.5)
        )
        shell = shell.union(pad)
        dimple = (
            cq.Workplane("XY")
            .workplane(offset=0.05)
            .center(px, py)
            .circle(1.65)
            .workplane(offset=-1.9)
            .circle(0.2)
            .loft(combine=True)
        )
        shell = shell.cut(dimple)

    # 2. Waveshare 3.5B Flush Glass Opening (cut AFTER shield pads -> zero STEP overlap!)
    ws_open = make_rounded_box(S.WS_GLASS_W + 2.0 * S.FIT_CLEAR,
                               S.WS_GLASS_H + 2.0 * S.FIT_CLEAR,
                               6.5,
                               S.WS_GLASS_R + S.FIT_CLEAR,
                               cx=S.WS_CX, cy=S.WS_CY, z_min=-5.5)
    shell = shell.cut(ws_open)

    # 3. USB-C Bottom Tunnel (45-deg self-supporting peak at -Z_asm side)
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
    shell = shell.cut(usbc_cut)

    # 4. Right-Side Tactile Buttons (3 x drop-in plunger bores) + MicroSD Scoop
    right_x_inner = S.CASE_W / 2.0 - S.WALL_SIDE - 0.5
    right_x_len = S.WALL_SIDE + 1.5
    for by in S.WS_BTN_Y_LIST:
        b_cut = make_arch_x_cutout(
            right_x_inner, right_x_len, by, width_y=4.8,
            z_front=S.WS_BTN_Z_CTR + 2.2, z_back=S.WS_BTN_Z_CTR - 2.2, roof_gable=1.5
        )
        shell = shell.cut(b_cut)

    sd_cy = 0.5 * (S.WS_SD_Y0 + S.WS_SD_Y1)
    sd_len_y = (S.WS_SD_Y1 - S.WS_SD_Y0) + 3.0
    sd_cut = make_arch_x_cutout(
        right_x_inner, right_x_len, sd_cy, width_y=sd_len_y,
        z_front=S.WS_SD_Z_CTR + 2.8, z_back=S.WS_SD_Z_CTR - 2.8, roof_gable=1.8
    )
    shell = shell.cut(sd_cut)

    # 5. Lower-Left Front-Firing Speaker Acoustic Grille & Retaining Rim (28 x 28 x 11H)
    for row in (-8.0, -4.0, 0.0, 4.0, 8.0):
        slot = make_rounded_box(18.0, 1.8, S.WALL_FRONT + 1.0, 0.8,
                                cx=S.SPK_CX, cy=S.SPK_CY + row, z_min=S.Z_FRONT_INNER - 0.5)
        shell = shell.cut(slot)

    spk_rim_out = make_rounded_box(S.SPK_W + 3.6, S.SPK_H + 3.6, 5.0, 2.5,
                                   cx=S.SPK_CX, cy=S.SPK_CY, z_min=S.Z_FRONT_INNER - 5.0)
    spk_rim_in = make_rounded_box(S.SPK_W + 2.0 * S.FIT_CLEAR, S.SPK_H + 2.0 * S.FIT_CLEAR, 5.5, 1.5,
                                  cx=S.SPK_CX, cy=S.SPK_CY, z_min=S.Z_FRONT_INNER - 5.2)
    shell = shell.union(spk_rim_out.cut(spk_rim_in))

    # 6. UM982 Front LED Light Pipes (1.75 mm PETG filament -> Dia 1.95 mm) + SWITCH Pinhole
    # (Power Board M2 bosses moved to Rear_Cover in v1.1.0 so Battery & Power remain on one shell)
    for lx_off in S.UM_LED_X_OFFSETS:
        lx = S.UM_LEFT_X + lx_off
        ly = S.UM_TOP_Y - S.UM_LED_TOP_OFFSET
        pipe_hole = (
            cq.Workplane("XY")
            .workplane(offset=S.Z_FRONT_INNER - 0.5)
            .center(lx, ly)
            .circle(1.95 / 2.0)
            .extrude(S.WALL_FRONT + 1.0)
        )
        shell = shell.cut(pipe_hole)

    sw_x = S.UM_LEFT_X + S.UM_SW_X_OFFSET
    sw_y = S.UM_TOP_Y - S.UM_SW_TOP_OFFSET
    sw_hole = (
        cq.Workplane("XY")
        .workplane(offset=S.Z_FRONT_INNER - 0.5)
        .center(sw_x, sw_y)
        .circle(3.4 / 2.0)
        .extrude(S.WALL_FRONT + 1.0)
    )
    shell = shell.cut(sw_hole)

    # 8. Top Bulkhead Local Thinning Pockets (Support-free open to Z_PARTING rear edge!)
    # UM982 roof web thinned to 2.0 mm (Y=180.0..182.0), cutting inner wall Y=178.5..180.0
    um_pocket = (
        cq.Workplane("XY")
        .workplane(offset=S.Z_PARTING - 0.5)
        .center(S.UM_CX, 179.25)
        .rect(S.UM_W + 0.8, 1.50)
        .extrude(abs(S.Z_PARTING) - abs(S.Z_FRONT_INNER) + 0.5)
    )
    shell = shell.cut(um_pocket)

    # GX-12 roof web thinned to 2.6 mm (Y=179.4..182.0), cutting inner wall Y=178.5..179.4
    gx_pocket = (
        cq.Workplane("XY")
        .workplane(offset=S.Z_PARTING - 0.5)
        .center(S.GX12_X, 178.95)
        .rect(S.GX12_NUT_RECESS_DIA + 1.0, 0.90)
        .extrude(abs(S.Z_PARTING) - abs(S.Z_FRONT_INNER) + 0.5)
    )
    shell = shell.cut(gx_pocket)

    # 9. Top Roof Bulkhead Cutouts: GX-12 + E220 SMA + UM982 Slave & Master SMAs
    roof_y0 = S.CASE_H - 4.2
    roof_len = 5.0
    gx_cut = make_teardrop_y_bore(S.GX12_X, roof_y0, roof_len, S.GX12_Z,
                                  S.GX12_HOLE_DIA, top_flat_dist=S.GX12_FLAT_DIST,
                                  add_teardrop=False)
    e220_cut = make_teardrop_y_bore(S.E220_SMA_X, roof_y0, roof_len, S.E220_SMA_Z,
                                    S.E220_SMA_DIA, top_flat_dist=S.E220_SMA_FLAT,
                                    add_teardrop=False)
    um_s_cut = make_teardrop_y_bore(S.UM_SMA_SLAVE_X, roof_y0, roof_len, S.UM_SMA_Z,
                                    S.UM_SMA_ACCESS_DIA, add_teardrop=True)
    um_m_cut = make_teardrop_y_bore(S.UM_SMA_MASTER_X, roof_y0, roof_len, S.UM_SMA_Z,
                                    S.UM_SMA_ACCESS_DIA, add_teardrop=True)
    shell = shell.cut(gx_cut).cut(e220_cut).cut(um_s_cut).cut(um_m_cut)

    # 10. 6 x M3 Ruthex Enclosure Closure Bosses (growing UP from Z=-2.4 to Z=-24.0 + 1.6 rim)
    boss_z_bot = S.Z_PARTING
    boss_len = abs(boss_z_bot - S.Z_FRONT_INNER)
    for bx, by in S.COVER_M3_POINTS:
        boss = (
            cq.Workplane("XY")
            .workplane(offset=boss_z_bot + lip_h)
            .center(bx, by)
            .circle(S.RUTHEX_M3_BOSS_OD / 2.0)
            .extrude(boss_len - lip_h)
        )
        # Gusset web tying boss smoothly to outer wall
        hole = (
            cq.Workplane("XY")
            .workplane(offset=boss_z_bot + lip_h - 0.1)
            .center(bx, by)
            .circle(S.RUTHEX_M3_OD / 2.0)
            .extrude(S.RUTHEX_M3_DEPTH + 0.5)
        )
        cham = (
            cq.Workplane("XY")
            .workplane(offset=boss_z_bot + lip_h - 0.05)
            .center(bx, by)
            .circle(S.RUTHEX_M3_OD / 2.0 + 0.6)
            .workplane(offset=0.65)
            .circle(S.RUTHEX_M3_OD / 2.0)
            .loft(combine=True)
        )
        shell = shell.union(boss).cut(hole).cut(cham)

    return shell


def build_mid_sled() -> cq.Workplane:
    """Build Mid_Frame_Sled (Z = -21.5 front to -24.0 flat back, zero supports)."""
    plate_w = S.CASE_W - 2.0 * S.WALL_SIDE - 1.2
    plate_y0 = S.WALL_BOT + 0.6
    plate_y1 = S.CASE_H - S.WALL_TOP - 0.6
    plate_h = plate_y1 - plate_y0
    plate_cy = 0.5 * (plate_y0 + plate_y1)

    sled = make_rounded_box(plate_w, plate_h, S.Z_SLED_THK, 2.5,
                            cx=0.0, cy=plate_cy, z_min=S.Z_SLED_BACK)

    # Open left C-bay on Left Power Bay (square inner corner -> zero floating fillet sliver)
    pb_win = make_rounded_box(27.0, 90.0, S.Z_SLED_THK + 1.0, 0.0,
                              cx=-46.5, cy=93.5, z_min=S.Z_SLED_BACK - 0.5)
    pb_bot_jst = make_rounded_box(15.0, 11.0, S.Z_SLED_THK + 1.0, 0.0,
                                  cx=-40.5, cy=44.5, z_min=S.Z_SLED_BACK - 0.5)
    sled = sled.cut(pb_win).cut(pb_bot_jst)

    # Clear corner cutouts for the 6 x M3 shell closure bosses (so sled locks flush in seam)
    for bx, by in S.COVER_M3_POINTS:
        c_notch = (
            cq.Workplane("XY")
            .workplane(offset=S.Z_SLED_BACK - 0.5)
            .center(bx, by)
            .circle(S.RUTHEX_M3_BOSS_OD / 2.0 + 0.4)
            .extrude(S.Z_SLED_THK + 1.0)
        )
        sled = sled.cut(c_notch)

    # Wire Routing Slot behind Waveshare right-angle DuPont tails + top Power harness slot
    dupont_slot = make_rounded_box(10.0, 44.0, S.Z_SLED_THK + 1.0, 3.0,
                                   cx=S.WS_DUPONT_TIP_X - 1.0, cy=S.WS_CY,
                                   z_min=S.Z_SLED_BACK - 0.5)
    sled = sled.cut(dupont_slot)

    # 1. Waveshare 4 x M2 Standoff Pillars (Z=-21.5 up to Z=-11.50 = 10.0 mm tall, CSK back)
    ws_pillar_h = S.WS_STANDOFF_Z_END - S.Z_SLED_FRONT  # 10.0 mm
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            wx = S.WS_CX + sx * (S.WS_STANDOFF_DX / 2.0)
            wy = S.WS_CY + sy * (S.WS_STANDOFF_DY / 2.0)
            pil = (
                cq.Workplane("XY")
                .workplane(offset=S.Z_SLED_FRONT)
                .center(wx, wy)
                .circle(S.RUTHEX_M2_BOSS_OD / 2.0)
                .extrude(ws_pillar_h)
            )
            csk = make_csk_cone_through(wx, wy, S.Z_SLED_BACK, S.Z_SLED_THK + ws_pillar_h,
                                        S.HOLE_CLEAR_M2, 4.2, cone_from_bed=True)
            sled = sled.union(pil).cut(csk)

    # 2. UM982 4 x M2 Standoff Pillars (Z=-21.5 up to Z=-17.2 = 4.3 mm tall, CSK back)
    um_pillar_h = S.UM_BACK_Z - S.Z_SLED_FRONT  # 4.3 mm
    for sx in (-1.0, 1.0):
        for sy in (0.0, 1.0):
            ux = S.UM_CX + sx * (S.UM_HOLE_DX / 2.0)
            uy = (S.UM_TOP_Y - S.UM_HOLE_TOP_OFFSET) - sy * S.UM_HOLE_DY
            pil = (
                cq.Workplane("XY")
                .workplane(offset=S.Z_SLED_FRONT)
                .center(ux, uy)
                .circle(2.8)
                .extrude(um_pillar_h)
            )
            csk = make_csk_cone_through(ux, uy, S.Z_SLED_BACK, S.Z_SLED_THK + um_pillar_h,
                                        S.HOLE_CLEAR_M2, 4.2, cone_from_bed=True)
            sled = sled.union(pil).cut(csk)

    # 3. E220 4 x M2 Ruthex Standoff Bosses (35 x 55 mm grid, 3.5 mm solder clearance)
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            ex = S.E220_CX + sx * (S.E220_HOLE_DX / 2.0)
            ey = S.E220_CY + sy * (S.E220_HOLE_DY / 2.0)
            pil = (
                cq.Workplane("XY")
                .workplane(offset=S.Z_SLED_FRONT)
                .center(ex, ey)
                .circle(S.RUTHEX_M2_BOSS_OD / 2.0)
                .extrude(S.E220_STANDOFF_H)
            )
            hole = (
                cq.Workplane("XY")
                .workplane(offset=S.Z_SLED_FRONT + S.E220_STANDOFF_H - S.RUTHEX_M2_DEPTH)
                .center(ex, ey)
                .circle(S.RUTHEX_M2_OD / 2.0)
                .extrude(S.RUTHEX_M2_DEPTH + 0.3)
            )
            sled = sled.union(pil).cut(hole)

    # 4. RTC Coin-Cell Cradle Pocket (Ø21.4 x 4.2 mm at (-20.0, 20.0), open top for wire)
    ring_out = (
        cq.Workplane("XY")
        .workplane(offset=S.Z_SLED_FRONT)
        .center(S.RTC_BAT_CX, S.RTC_BAT_CY)
        .circle(S.RTC_BAT_DIA / 2.0 + 1.8)
        .extrude(S.RTC_BAT_THK + 0.5)
    )
    ring_in = (
        cq.Workplane("XY")
        .workplane(offset=S.Z_SLED_FRONT - 0.2)
        .center(S.RTC_BAT_CX, S.RTC_BAT_CY)
        .circle(S.RTC_BAT_DIA / 2.0)
        .extrude(S.RTC_BAT_THK + 1.0)
    )
    wire_gap = make_rounded_box(8.0, 10.0, S.RTC_BAT_THK + 2.0, 0.5,
                                cx=S.RTC_BAT_CX + 8.0, cy=S.RTC_BAT_CY,
                                z_min=S.Z_SLED_FRONT - 0.2)
    sled = sled.union(ring_out.cut(ring_in).cut(wire_gap))

    return sled


def build_rear_cover() -> cq.Workplane:
    """Build Rear_Cover (Z = -38.8 bed outside up to Z_PARTING = -24.0, support-free)."""
    case_cy = S.CASE_H / 2.0
    depth = S.Z_PARTING - S.Z_REAR_OUTER  # 14.8 mm
    outer = make_rounded_box(S.CASE_W, S.CASE_H, depth, S.CORNER_R,
                             cx=0.0, cy=case_cy, z_min=S.Z_REAR_OUTER)

    # Interior cavity above Z_REAR_INNER (-35.8)
    in_w = S.CASE_W - 2.0 * S.WALL_SIDE
    in_h = S.CASE_H - S.WALL_BOT - S.WALL_TOP
    in_cy = 0.5 * (S.WALL_BOT + (S.CASE_H - S.WALL_TOP))
    cav = make_rounded_box(in_w, in_h, depth - S.WALL_REAR + 0.5,
                           max(1.5, S.CORNER_R - S.WALL_SIDE),
                           cx=0.0, cy=in_cy, z_min=S.Z_REAR_INNER)
    shell = outer.cut(cav)

    # Mating tongue-and-groove outer rim step (1.2 mm lap matching Front_Shell cut)
    lip_h = 1.4
    lip_trim = (
        make_rounded_box(S.CASE_W + 2.0, S.CASE_H + 2.0, lip_h + 0.2, S.CORNER_R,
                         cx=0.0, cy=case_cy, z_min=S.Z_PARTING - lip_h)
        .cut(make_rounded_box(S.CASE_W - 2.0 * 1.6, S.CASE_H - 2.0 * 1.6, lip_h + 0.6,
                              max(1.5, S.CORNER_R - 1.6),
                              cx=0.0, cy=case_cy, z_min=S.Z_PARTING - lip_h - 0.2))
    )
    shell = shell.cut(lip_trim)

    # 1. Centered Battery Containment Walls (123.5 x 64.5 x 11.8 mm pocket)
    bat_wall_out = make_rounded_box(S.BAT_POCKET_W + 3.2, S.BAT_POCKET_H + 3.2, S.BAT_POCKET_THK - 1.0,
                                    3.0, cx=S.BAT_CX, cy=S.BAT_CY, z_min=S.Z_REAR_INNER - 0.5)
    bat_wall_in = make_rounded_box(S.BAT_POCKET_W, S.BAT_POCKET_H, S.BAT_POCKET_THK,
                                   2.0, cx=S.BAT_CX, cy=S.BAT_CY, z_min=S.Z_REAR_INNER - 0.2)
    bat_lead_exit = make_rounded_box(16.0, 8.0, S.BAT_POCKET_THK, 1.0,
                                     cx=S.BAT_CX - 18.0, cy=S.BAT_CY - S.BAT_POCKET_H / 2.0,
                                     z_min=S.Z_REAR_INNER)
    shell = shell.union(bat_wall_out.cut(bat_wall_in).cut(bat_lead_exit))

    # 2. 6 x M3 Closure Screw Pillars with 45-deg CSK entering from Rear exterior
    full_rear_h = S.Z_PARTING - S.Z_REAR_OUTER
    for bx, by in S.COVER_M3_POINTS:
        pil = (
            cq.Workplane("XY")
            .workplane(offset=S.Z_REAR_INNER)
            .center(bx, by)
            .circle(S.RUTHEX_M3_BOSS_OD / 2.0)
            .extrude(S.Z_PARTING - S.Z_REAR_INNER)
        )
        csk = make_csk_cone_through(bx, by, S.Z_REAR_OUTER, full_rear_h,
                                    S.HOLE_CLEAR_M3, 6.2, cone_from_bed=True)
        shell = shell.union(pil).cut(csk)

    # 3. Option 3: 4 x M3 Rear Pole Adapter Ruthex Bosses (80 x 58 mm, flanking Battery!)
    boss_z_h = 8.4  # 3.0 mm rear wall + 5.4 mm inside cavity -> 6.7 mm blind hole leaves 1.7 mm cap
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            ax = S.ADAPTER_CX + sx * (S.ADAPTER_M3_DX / 2.0)
            ay = S.ADAPTER_M3_CY + sy * (S.ADAPTER_M3_DY / 2.0)
            boss = (
                cq.Workplane("XY")
                .workplane(offset=S.Z_REAR_OUTER)
                .center(ax, ay)
                .circle(S.RUTHEX_M3_BOSS_OD / 2.0)
                .extrude(boss_z_h)
            )
            hole = (
                cq.Workplane("XY")
                .workplane(offset=S.Z_REAR_OUTER - 0.1)
                .center(ax, ay)
                .circle(S.RUTHEX_M3_OD / 2.0)
                .extrude(S.RUTHEX_M3_DEPTH + 0.1)
            )
            cham = (
                cq.Workplane("XY")
                .workplane(offset=S.Z_REAR_OUTER - 0.05)
                .center(ax, ay)
                .circle(S.RUTHEX_M3_OD / 2.0 + 0.55)
                .workplane(offset=0.6)
                .circle(S.RUTHEX_M3_OD / 2.0)
                .loft(combine=True)
            )
            shell = shell.union(boss).cut(hole).cut(cham)

    # 4. Power Board 4 x M2 Ruthex Bosses on Rear_Cover (15 x 75 mm grid, Z=-36.3 up to -29.8)
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            bx = S.PB_CX + sx * (S.PB_HOLE_DX / 2.0)
            by = S.PB_CY + sy * (S.PB_HOLE_DY / 2.0)
            boss = (
                cq.Workplane("XY")
                .workplane(offset=S.Z_REAR_INNER)
                .center(bx, by)
                .circle(S.RUTHEX_M2_BOSS_OD / 2.0)
                .extrude(S.PB_STANDOFF_H)
            )
            hole = (
                cq.Workplane("XY")
                .workplane(offset=S.PB_PCB_BACK_Z - S.RUTHEX_M2_DEPTH)
                .center(bx, by)
                .circle(S.RUTHEX_M2_OD / 2.0)
                .extrude(S.RUTHEX_M2_DEPTH + 0.1)
            )
            cham = (
                cq.Workplane("XY")
                .workplane(offset=S.PB_PCB_BACK_Z + 0.05)
                .center(bx, by)
                .circle(S.RUTHEX_M2_OD / 2.0 + 0.5)
                .workplane(offset=-0.55)
                .circle(S.RUTHEX_M2_OD / 2.0)
                .loft(combine=True)
            )
            shell = shell.union(boss).cut(hole).cut(cham)

    return shell


def build_pole_adapter() -> cq.Workplane:
    """Build Option 3 Bolt-On Pole Adapter Plate (Z = -38.8 to -55.0)."""
    z_mount = S.Z_REAR_OUTER  # Flush against rear outer shell
    base = make_rounded_box(S.POLE_ADAPTER_W, S.POLE_ADAPTER_H, S.POLE_ADAPTER_BASE_T, 5.0,
                            cx=S.ADAPTER_CX, cy=S.ADAPTER_M3_CY,
                            z_min=z_mount - S.POLE_ADAPTER_BASE_T)

    spine = make_rounded_box(S.POLE_ADAPTER_SPINE_W, S.POLE_ADAPTER_H - 6.0,
                             S.POLE_ADAPTER_SPINE_T, 6.0,
                             cx=S.ADAPTER_CX, cy=S.ADAPTER_M3_CY,
                             z_min=z_mount - S.POLE_ADAPTER_SPINE_T)
    plate = base.union(spine)

    # 4 x M3 CSK holes (cone from rear face of adapter base plate)
    z_base_rear = z_mount - S.POLE_ADAPTER_BASE_T
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            ax = S.ADAPTER_CX + sx * (S.ADAPTER_M3_DX / 2.0)
            ay = S.ADAPTER_M3_CY + sy * (S.ADAPTER_M3_DY / 2.0)
            csk = make_csk_cone_through(ax, ay, z_base_rear, S.POLE_ADAPTER_BASE_T,
                                        S.HOLE_CLEAR_M3, 6.2, cone_from_bed=True)
            plate = plate.cut(csk)

    # 2 x 1/4"-20 Ruthex Insert Bosses (8.0 mm hole x 13.7 mm deep, >= 2.5 mm floor to case)
    z_spine_back = z_mount - S.POLE_ADAPTER_SPINE_T
    for sy in (-1.0, 1.0):
        qy = S.ADAPTER_M3_CY + sy * (S.POLE_Q20_DY / 2.0)
        q_hole = (
            cq.Workplane("XY")
            .workplane(offset=z_spine_back - 0.1)
            .center(S.ADAPTER_CX, qy)
            .circle(S.RUTHEX_Q20_OD / 2.0)
            .extrude(S.RUTHEX_Q20_DEPTH + 0.1)
        )
        q_cham = (
            cq.Workplane("XY")
            .workplane(offset=z_spine_back - 0.05)
            .center(S.ADAPTER_CX, qy)
            .circle(S.RUTHEX_Q20_OD / 2.0 + 0.65)
            .workplane(offset=0.7)
            .circle(S.RUTHEX_Q20_OD / 2.0)
            .loft(combine=True)
        )
        plate = plate.cut(q_hole).cut(q_cham)

    return plate


def build_sun_shield() -> cq.Workplane:
    """Build Detachable 3-Sided Outdoor Sun Shield (flush X=[-23.5..+58.0], zero wall notches)."""
    x_min, x_max = -23.5, 58.0       # Covers Left (-16.8) magnets & Right (+54.4) pins with >=2.1 mm edge margin
    y_min, y_max = 5.5, 111.5        # Covers Top (Y=105) magnets cleanly while leaving UM982 SWITCH free
    flange_w = x_max - x_min         # 81.5 mm
    flange_h = y_max - y_min         # 106.0 mm
    flange_cx = 0.5 * (x_min + x_max) # 17.25 mm
    flange_cy = 0.5 * (y_min + y_max) # 58.5 mm

    base = make_rounded_box(flange_w, flange_h, 3.0, 5.0,
                            cx=flange_cx, cy=flange_cy, z_min=0.0)
    # Visor walls (top + left + right hood rising forward +Z = 38 mm, open bottom!)
    hood_h = 38.0
    hood_outer = make_rounded_box(flange_w, flange_h, hood_h, 5.0,
                                  cx=flange_cx, cy=flange_cy, z_min=0.0)
    hood_inner = make_rounded_box(S.WS_GLASS_W + 0.6, 88.5, hood_h + 2.0, 4.0,
                                  cx=S.WS_CX, cy=54.75, z_min=-0.5)
    shield = base.union(hood_outer).cut(hood_inner)

    # Trim lower portion to 4.5 mm protective bezel only, tapering upward at 45 deg
    lower_cut = (
        cq.Workplane("YZ")
        .workplane(offset=flange_cx - flange_w / 2.0 - 2.0)
        .polyline([
            (y_min - 2.0, 4.5),
            (58.0, 4.5),
            (92.0, hood_h + 1.0),
            (y_min - 2.0, hood_h + 1.0),
        ])
        .close()
        .extrude(flange_w + 4.0)
    )
    shield = shield.cut(lower_cut)

    # Magnet holes & slim registration pins mating cleanly inside Front_Shell bezel
    for mx, my in S.SHIELD_MAG_POINTS:
        m_hole = (
            cq.Workplane("XY")
            .workplane(offset=-0.1)
            .center(mx, my)
            .circle(S.SHIELD_MAG_DIA / 2.0)
            .extrude(S.SHIELD_MAG_DEPTH + 0.1)
        )
        shield = shield.cut(m_hole)
    for px, py in S.SHIELD_PIN_POINTS:
        pin = (
            cq.Workplane("XY")
            .workplane(offset=0.0)
            .center(px, py)
            .circle(1.45)
            .workplane(offset=-1.6)
            .circle(0.35)
            .loft(combine=True)
        )
        shield = shield.union(pin)

    return shield


def build_button_plungers() -> cq.Workplane:
    """Build 3 captive drop-in tactile plunger caps for PWR, RESET, BOOT (support-free)."""
    # Positioned aligned inside the 3 Front_Shell bores; flange on inside, stem sticks slightly proud
    plungers = None
    for by in S.WS_BTN_Y_LIST:
        flange = (
            cq.Workplane("YZ")
            .workplane(offset=S.WS_BTN_X_EDGE + 0.6)
            .center(by, S.WS_BTN_Z_CTR)
            .circle(3.1)
            .extrude(2.2)
        )
        stem = (
            cq.Workplane("YZ")
            .workplane(offset=S.WS_BTN_X_EDGE + 2.8)
            .center(by, S.WS_BTN_Z_CTR)
            .circle(2.15)
            .extrude(S.CASE_W / 2.0 + 0.8 - (S.WS_BTN_X_EDGE + 2.8))
        )
        cap = flange.union(stem)
        plungers = cap if plungers is None else plungers.union(cap)
    return plungers


def build_hardware_proxies() -> dict:
    """Create bounding-envelope solids of all internal modules for interference audit."""
    mods = {}
    if STP_WAVESHARE.exists():
        ws = cq.importers.importStep(str(STP_WAVESHARE))
        # Shift STEP Z=+1.10 glass face down to Z=0.0, then translate to (WS_CX, WS_CY)
        mods["waveshare_stp"] = ws.translate((S.WS_CX, S.WS_CY, -1.10))
    # Power Board body (Z = PB_PCB_BACK_Z .. +20.0 mm tall parts) + 2.0 mm underside solder envelope (inset inside pads)
    pb_upper = make_rounded_box(S.PB_W, S.PB_H, 20.0, 0.5,
                                cx=S.PB_CX, cy=S.PB_CY, z_min=S.PB_PCB_BACK_Z)
    pb_pins = make_rounded_box(S.PB_W - 8.0, S.PB_H - 12.0, 2.0, 0.2,
                               cx=S.PB_CX, cy=S.PB_CY, z_min=S.PB_PCB_BACK_Z - 2.0)
    mods["power_board"] = pb_upper.union(pb_pins)
    mods["e220"] = make_rounded_box(
        S.E220_W, S.E220_H, S.E220_Z_THK, 0.5,
        cx=S.E220_CX, cy=S.E220_CY, z_min=S.Z_SLED_FRONT + S.E220_STANDOFF_H
    )
    mods["um982"] = make_rounded_box(
        S.UM_W, S.UM_H, S.UM_Z_THK, 1.0,
        cx=S.UM_CX, cy=S.UM_CY, z_min=S.UM_BACK_Z
    )
    mods["battery"] = make_rounded_box(
        S.BAT_CELL_W, S.BAT_CELL_H, 11.0, 1.5,
        cx=S.BAT_CX, cy=S.BAT_CY - 2.0, z_min=S.Z_REAR_INNER + 0.3
    )
    mods["speaker"] = make_rounded_box(
        S.SPK_W, S.SPK_H, S.SPK_Z_THK, 1.0,
        cx=S.SPK_CX, cy=S.SPK_CY, z_min=S.Z_FRONT_INNER - S.SPK_Z_THK - 0.2
    )
    mods["rtc_coin"] = (
        cq.Workplane("XY")
        .workplane(offset=S.Z_SLED_FRONT)
        .center(S.RTC_BAT_CX, S.RTC_BAT_CY)
        .circle(21.0 / 2.0)
        .extrude(4.0)
    )
    return mods


def orient_for_print(part: cq.Workplane, flip_x180: bool = False, rot_y_deg: float = 0.0) -> cq.Workplane:
    """Rotate/translate solid so its flat support-free print face sits at Z = 0, growing +Z."""
    out = part.rotate((0, 0, 0), (1, 0, 0), 180.0) if flip_x180 else part
    if abs(rot_y_deg) > 1e-3:
        out = out.rotate((0, 0, 0), (0, 1, 0), rot_y_deg)
    bb = out.val().BoundingBox()
    return out.translate((0.0, 0.0, -bb.zmin))


def run_audit(parts: dict, mods: dict) -> list:
    """Run automated module-vs-module and module-vs-enclosure clearance audit."""
    issues = []
    checks = [
        ("Front_Shell", "power_board"),
        ("Front_Shell", "e220"),
        ("Front_Shell", "um982"),
        ("Front_Shell", "battery"),
        ("Front_Shell", "speaker"),
        ("Front_Shell", "rtc_coin"),
        ("Mid_Frame_Sled", "power_board"),
        ("Mid_Frame_Sled", "e220"),
        ("Mid_Frame_Sled", "um982"),
        ("Mid_Frame_Sled", "battery"),
        ("Mid_Frame_Sled", "speaker"),
        ("Rear_Cover", "power_board"),
        ("Rear_Cover", "battery"),
        ("Rear_Cover", "e220"),
        ("Rear_Cover", "um982"),
        ("Pole_Adapter_Plate", "Rear_Cover"),
    ]
    for pname, mname in checks:
        target = mods[mname] if mname in mods else parts[mname]
        inter = parts[pname].intersect(target)
        vol = sum(s.Volume() for s in inter.solids().vals())
        if vol > 0.05:
            issues.append(f"[OVERLAP] {pname} vs {mname}: {vol:.2f} mm^3")
        else:
            print(f"[AUDIT PASS] {pname} vs {mname}: 0.00 mm^3 overlap")
    # Verify Front_Shell outer envelope stayed exact (zero protruding bosses)
    fs_bb = parts["Front_Shell"].val().BoundingBox()
    if abs(fs_bb.xmax - S.CASE_W / 2.0) > 0.05 or abs(fs_bb.xmin + S.CASE_W / 2.0) > 0.05:
        issues.append(f"[PROTRUSION] Front_Shell X=[{fs_bb.xmin:.2f}, {fs_bb.xmax:.2f}] exceeds +/-{S.CASE_W/2:.2f}")
    else:
        print(f"[AUDIT PASS] Front_Shell outer X flush: [{fs_bb.xmin:.2f}, {fs_bb.xmax:.2f}] mm")
    return issues


def export_dxf_2d() -> Path:
    """Export AutoCAD LT 2010 compatible R2010 DXF interface layout."""
    DXF_DIR.mkdir(parents=True, exist_ok=True)
    dxf_path = DXF_DIR / "RTK_Rover_Interfaces_2D.dxf"
    doc = ezdxf.new("R2010")
    doc.header["$INSUNITS"] = 4  # mm
    msp = doc.modelspace()

    for layer, col in [("OUTLINE", 7), ("CUTOUTS", 1), ("HOLES_M2_M3", 3), ("MODULES", 4), ("TEXT_NOTES", 2)]:
        doc.layers.add(layer, color=col)

    def rect_poly(x0, y0, x1, y1, layer):
        msp.add_lwpolyline([(x0, y0), (x1, y0), (x1, y1), (x0, y1)], close=True, dxfattribs={"layer": layer})

    # Front & Module View centered around (0, 0..182)
    rect_poly(-S.CASE_W / 2, 0.0, S.CASE_W / 2, S.CASE_H, "OUTLINE")
    rect_poly(S.WS_CX - S.WS_GLASS_W / 2, S.WS_CY - S.WS_GLASS_H / 2,
              S.WS_CX + S.WS_GLASS_W / 2, S.WS_CY + S.WS_GLASS_H / 2, "CUTOUTS")
    rect_poly(S.PB_CX - S.PB_W / 2, S.PB_CY - S.PB_H / 2,
              S.PB_CX + S.PB_W / 2, S.PB_CY + S.PB_H / 2, "MODULES")
    rect_poly(S.E220_CX - S.E220_W / 2, S.E220_CY - S.E220_H / 2,
              S.E220_CX + S.E220_W / 2, S.E220_CY + S.E220_H / 2, "MODULES")
    rect_poly(S.UM_LEFT_X, S.UM_BOT_Y, S.UM_RIGHT_X, S.UM_TOP_Y, "MODULES")
    rect_poly(S.SPK_CX - S.SPK_W / 2, S.SPK_CY - S.SPK_H / 2,
              S.SPK_CX + S.SPK_W / 2, S.SPK_CY + S.SPK_H / 2, "MODULES")
    msp.add_circle((S.RTC_BAT_CX, S.RTC_BAT_CY), S.RTC_BAT_DIA / 2.0, dxfattribs={"layer": "MODULES"})

    # UM982 Front LED light pipes & switch
    for lx_off in S.UM_LED_X_OFFSETS:
        msp.add_circle((S.UM_LEFT_X + lx_off, S.UM_TOP_Y - S.UM_LED_TOP_OFFSET), 1.95 / 2.0,
                       dxfattribs={"layer": "CUTOUTS"})
    msp.add_circle((S.UM_LEFT_X + S.UM_SW_X_OFFSET, S.UM_TOP_Y - S.UM_SW_TOP_OFFSET), 1.7,
                   dxfattribs={"layer": "CUTOUTS"})

    for bx, by in S.COVER_M3_POINTS:
        msp.add_circle((bx, by), S.RUTHEX_M3_OD / 2.0, dxfattribs={"layer": "HOLES_M2_M3"})

    # Offset +150 mm right: Rear Battery & Pole Adapter M3 + 1/4"-20 layout
    ox = 150.0
    rect_poly(ox - S.CASE_W / 2, 0.0, ox + S.CASE_W / 2, S.CASE_H, "OUTLINE")
    rect_poly(ox + S.BAT_CX - S.BAT_POCKET_W / 2, S.BAT_CY - S.BAT_POCKET_H / 2,
              ox + S.BAT_CX + S.BAT_POCKET_W / 2, S.BAT_CY + S.BAT_POCKET_H / 2, "MODULES")
    rect_poly(ox + S.ADAPTER_CX - S.POLE_ADAPTER_W / 2, S.ADAPTER_M3_CY - S.POLE_ADAPTER_H / 2,
              ox + S.ADAPTER_CX + S.POLE_ADAPTER_W / 2, S.ADAPTER_M3_CY + S.POLE_ADAPTER_H / 2, "OUTLINE")
    for sx in (-1.0, 1.0):
        for sy in (-1.0, 1.0):
            msp.add_circle((ox + S.ADAPTER_CX + sx * S.ADAPTER_M3_DX / 2,
                            S.ADAPTER_M3_CY + sy * S.ADAPTER_M3_DY / 2),
                           S.RUTHEX_M3_OD / 2.0, dxfattribs={"layer": "HOLES_M2_M3"})
    for sy in (-1.0, 1.0):
        msp.add_circle((ox + S.ADAPTER_CX, S.ADAPTER_M3_CY + sy * S.POLE_Q20_DY / 2), S.RUTHEX_Q20_OD / 2.0,
                       dxfattribs={"layer": "HOLES_M2_M3"})

    # Offset +300 mm right: Top Crown Bulkhead Cross-Section (Y -> vertical Z_depth)
    tx = 300.0
    rect_poly(tx - S.CASE_W / 2, 0.0, tx + S.CASE_W / 2, abs(S.Z_REAR_OUTER), "OUTLINE")
    msp.add_circle((tx + S.GX12_X, abs(S.GX12_Z)), S.GX12_HOLE_DIA / 2.0, dxfattribs={"layer": "CUTOUTS"})
    msp.add_circle((tx + S.E220_SMA_X, abs(S.E220_SMA_Z)), S.E220_SMA_DIA / 2.0, dxfattribs={"layer": "CUTOUTS"})
    msp.add_circle((tx + S.UM_SMA_SLAVE_X, abs(S.UM_SMA_Z)), S.UM_SMA_ACCESS_DIA / 2.0, dxfattribs={"layer": "CUTOUTS"})
    msp.add_circle((tx + S.UM_SMA_MASTER_X, abs(S.UM_SMA_Z)), S.UM_SMA_ACCESS_DIA / 2.0, dxfattribs={"layer": "CUTOUTS"})

    msp.add_text("FRONT & MODULE PLAN (mm)", dxfattribs={"layer": "TEXT_NOTES", "height": 4.0}).set_placement((-55, -10))
    msp.add_text("REAR & POLE ADAPTER", dxfattribs={"layer": "TEXT_NOTES", "height": 4.0}).set_placement((ox - 55, -10))
    msp.add_text("TOP CROWN BULKHEAD (Z depth from front=0)", dxfattribs={"layer": "TEXT_NOTES", "height": 4.0}).set_placement((tx - 55, -10))

    doc.saveas(str(dxf_path))
    return dxf_path


def main() -> int:
    print(f"=== {__filename__} {__version__} ===")
    STEP_DIR.mkdir(parents=True, exist_ok=True)
    STL_DIR.mkdir(parents=True, exist_ok=True)

    parts = {
        "Front_Shell": build_front_shell(),
        "Mid_Frame_Sled": build_mid_sled(),
        "Rear_Cover": build_rear_cover(),
        "Pole_Adapter_Plate": build_pole_adapter(),
        "Sun_Shield": build_sun_shield(),
        "Button_Plungers": build_button_plungers(),
    }
    mods = build_hardware_proxies()
    issues = run_audit(parts, mods)

    if "waveshare_stp" in mods:
        # Verify real Waveshare STEP geometry vs all 3 main structural parts
        for cname in ("Front_Shell", "Mid_Frame_Sled", "Rear_Cover"):
            inter = parts[cname].intersect(mods["waveshare_stp"])
            vol = sum(s.Volume() for s in inter.solids().vals())
            print(f"[AUDIT STP] {cname} vs Waveshare_3.5B.stp: {vol:.2f} mm^3 overlap")

    for name, solid in parts.items():
        step_p = STEP_DIR / f"{name}.step"
        solid.export(str(step_p))
        print(f"Exported STEP: {step_p.name} ({step_p.stat().st_size} B)")

    # Export full assembled SolidEdge STEP (enclosure + hardware reference volumes)
    assy = cq.Assembly(name="RTK_Rover_OptionA")
    for name, solid in parts.items():
        assy.add(solid, name=name)
    for mname, solid in mods.items():
        if mname != "waveshare_stp":
            assy.add(solid, name=f"REF_{mname}")
    assy_p = STEP_DIR / "RTK_Rover_Assembly.step"
    assy.export(str(assy_p))
    print(f"Exported Assembly STEP: {assy_p.name} ({assy_p.stat().st_size} B)")

    # Export print-ready STLs oriented flat on build plate (zero supports)
    print_map = {
        "Front_Shell": (parts["Front_Shell"], True, 0.0),
        "Mid_Frame_Sled": (parts["Mid_Frame_Sled"], False, 0.0),
        "Rear_Cover": (parts["Rear_Cover"], False, 0.0),
        "Pole_Adapter_Plate": (parts["Pole_Adapter_Plate"], True, 0.0),
        "Sun_Shield": (parts["Sun_Shield"], False, 0.0),
        "Button_Plungers": (parts["Button_Plungers"], False, -90.0),
    }
    for name, (solid, flip, rot_y) in print_map.items():
        pr_solid = orient_for_print(solid, flip_x180=flip, rot_y_deg=rot_y)
        stl_p = STL_DIR / f"{name}_print.stl"
        pr_solid.export(str(stl_p))
        bb = pr_solid.val().BoundingBox()
        print(f"Exported STL: {stl_p.name} ({stl_p.stat().st_size} B) "
              f"size=({bb.xlen:.1f} x {bb.ylen:.1f} x {bb.zlen:.1f} mm)")

    dxf_p = export_dxf_2d()
    print(f"Exported DXF: {dxf_p.name} ({dxf_p.stat().st_size} B)")

    if issues:
        print("INTERFERENCE DETECTED:")
        for i in issues:
            print(" ", i)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
