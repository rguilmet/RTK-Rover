// =============================================================================
// RTK Rover Option A Enclosure v2.2.4 (PETG Support-Free, Snapmaker 2)
// Parametric OpenSCAD 2026.11.23 mirror of rover_specs.py v2.2.4
// Coordinate origin: X = centered [-60..+60], Y = [0..180], Z = 0 front glass
// Units: millimeters (mm)
// =============================================================================

/* [View & Export Selector] */
PART = "all"; // ["all", "back", "top", "cradle", "lid", "adapter5", "wedge", "shield", "plungers", "back_print", "top_print", "cradle_print", "lid_print", "adapter5_print", "wedge_print", "shield_print", "plungers_print"]
BATTERY_OPT = "10000"; // ["5000", "10000"]
SHOW_MODULES = true;
EXPLODE_MM = 0; // [0:1:60]

$fn = 48;

// --- Main Electronics Body (120.0 x 180.0 x 28.0 mm) ---
CASE_W = 120.0;
CASE_H = 180.0;
CORNER_R = 6.0;
WALL_SIDE = 3.0;
WALL_BOT = 3.0;
WALL_TOP = 3.0;
WALL_FRONT = 2.4;
WALL_BACK = 3.0;

Z_FRONT_OUTER = 0.0;
Z_FRONT_INNER = -WALL_FRONT;             // -2.4
Z_PLATE_INNER = -25.0;
Z_PLATE_OUTER = Z_PLATE_INNER - WALL_BACK; // -28.0

FIT_CLEAR = 0.25;
NOTCH_CLEAR = 0.20;
HOLE_CLEAR_M2 = 2.30;
HOLE_CLEAR_M3 = 3.40;
RIM_CLEAR = FIT_CLEAR;
RIM_THK = 1.60;
RIM_H = 4.50;

// --- Ruthex Heat-Set Inserts ---
RUTHEX_M2_OD = 3.2;  RUTHEX_M2_DEPTH = 5.0;  RUTHEX_M2_BOSS = 6.2;
RUTHEX_M3_OD = 4.0;  RUTHEX_M3_DEPTH = 6.7;  RUTHEX_M3_BOSS = 7.8;
RUTHEX_Q20_OD = 8.0; RUTHEX_Q20_DEPTH = 13.7;

// --- Waveshare 3.5B (Portrait, USB-C Down) ---
WS_GLASS_W = 61.00;
WS_GLASS_H = 92.44;
WS_GLASS_R = 6.00;
WS_STANDOFF_DX = 48.50;
WS_STANDOFF_DY = 72.00;
WS_STANDOFF_Z_END = -11.50;
WS_CX = 18.50;
WS_CY = 52.50;

WS_USBC_W = 13.5;
WS_USBC_Z_CTR = -8.72;
WS_USBC_H_OPEN = 8.4;

WS_BTN_Y_LIST = [WS_CY - 25.00, WS_CY - 16.50, WS_CY - 8.00];
WS_BTN_Z_CTR = -8.62;
WS_SD_Y0 = WS_CY + 16.80;
WS_SD_Y1 = WS_CY + 32.80;
WS_SD_Z_CTR = -8.25;

FLANGE_Y0 = 20.0;
FLANGE_Y1 = 52.0;
PLUNGER_BLK_X0 = 56.00;
PLUNGER_BLK_X1 = 57.00;
PLUNGER_BLK_Y0 = 22.0;
PLUNGER_BLK_Y1 = 50.0;
PLUNGER_BLK_Z_TOP = -4.2;
PLUNGER_RAIL_X0 = 49.90;
PLUNGER_RAIL_Z_TOP = -11.70;
PLUNGER_U_X0 = 49.90;
PLUNGER_U_X1 = 51.60;
PLUNGER_U_Z_BOT = -11.50;
PLUNGER_NOSE_DIA = 3.20;
PLUNGER_U_SLOT_W = 3.70;

// --- RTC Coin Cell (Underneath Waveshare) ---
RTC_CX = 18.50;
RTC_CY = 64.00;

// --- Power Board (20 x 80 x 20.57H, Back Plate mounted) ---
PB_W = 20.0; PB_H = 80.0;
PB_HOLE_DX = 15.0; PB_HOLE_DY = 75.0;
PB_CX = -39.00; PB_CY = 91.00;
PB_STANDOFF_H = 2.55;

// --- Speaker (28 x 30 x 10H, 4-Side Slotted Corner-Pillar Tower & Support Ridge) ---
SPK_W = 28.0; SPK_H = 28.0; SPK_Z_THK = 10.0;
SPK_CX = -33.20; SPK_CY = 21.00;
SPK_PEDESTAL_H = 11.5; SPK_WALL = 1.6; SPK_RIDGE_W = 2.0; SPK_SLOT_W = 15.0;

// --- 3.5mm Panel-Mount Headphone Jack (REF-headphone-jack.stp on Left L-Tab) ---
HP_BODY_W = 12.00; HP_BODY_L = 14.00; HP_BODY_H = 6.30;
HP_CY = SPK_CY; HP_Z_CTR = -21.50; HP_BORE_DIA = 6.20; HP_WALL = 1.50;
HP_RECESS_W = HP_BODY_W + 2*FIT_CLEAR; HP_RECESS_H = HP_BODY_H + 2*FIT_CLEAR;
HP_TAB_W = 18.5; HP_TAB_Z_TOP = -13.50;

// --- Holybro UM982 (34.8 x 53.2 x 14.4H) ---
UM_W = 34.80; UM_BODY_H = 53.20; UM_Z_THK = 14.40;
UM_CX = -9.80;
UM_LEFT_X = UM_CX - UM_W / 2.0;
UM_TOP_Y = 178.00;
UM_BOT_Y = UM_TOP_Y - UM_BODY_H;
UM_CY = 0.5 * (UM_BOT_Y + UM_TOP_Y);
UM_BACK_Z = -17.20;
UM_FRONT_Z = UM_BACK_Z + UM_Z_THK;
UM_SMA_Z = UM_BACK_Z + 5.95;
UM_SMA_SLAVE_X = UM_LEFT_X + 7.25;
UM_SMA_MASTER_X = UM_SMA_SLAVE_X + 20.30;
UM_SMA_ACCESS_DIA = 10.5;
UM_M2_X_OFFS = [2.85, 31.95];
UM_M2_Y_OFFS = [5.80, 46.30];
UM_LED_Y = UM_BOT_Y + 30.20;
UM_LED_X_OFFS = [9.90, 14.90, 19.90, 24.90];
UM_SW_X = UM_LEFT_X + 24.90;
UM_SW_Y = UM_BOT_Y + 13.31;
UM_CSK_RECESS = 1.60;                   // Rear counterbore depth (9.20 mm effective span -> 2.80 mm engagement, 1.20 mm margin for M2x12 CSK)

// --- E220 LoRa (40 x 60 x 16H) ---
E220_W = 40.0; E220_H = 60.0;
E220_HOLE_DX = 35.0; E220_HOLE_DY = 55.0;
E220_CX = 29.00; E220_CY = 134.00;
E220_STANDOFF_H = 8.50;
E220_SMA_X = 29.00; E220_SMA_Z = UM_SMA_Z; E220_SMA_DIA = 6.6; E220_SMA_FLAT = 5.98; E220_SMA_WALL = 2.6;

// --- GX-12 Compass Bulkhead & 50/50 Split Top-Corner Crown Span ---
TOP_CORNER_SPLIT_Y = 173.2;
TOP_CORNER_WRAP_X_IN = 50.6;
GX12_X = -39.50; GX12_Z = -14.00; GX12_HOLE_DIA = 12.2; GX12_FLAT = 11.25; GX12_NUT_RECESS_DIA = 17.5; GX12_WALL = 2.6;

// --- Closure (6 Symmetric Front M3) & Rear Pod (4 M3) ---
COVER_M3_POINTS = [
    [-54.2,   6.8], [ 54.2,   6.8],
    [-54.1, 108.0], [ 54.1, 108.0],
    [-54.2, 173.2], [ 54.2, 173.2]
];
COVER_BOSS_Z_TOP = -2.65;
POD_CX = 2.0; POD_CY = 92.0;
POD_M3_DX = 80.0; POD_M3_DY = 100.0;
POD_M3_POINTS = [
    [POD_CX - POD_M3_DX/2, POD_CY - POD_M3_DY/2],
    [POD_CX + POD_M3_DX/2, POD_CY - POD_M3_DY/2],
    [POD_CX - POD_M3_DX/2, POD_CY + POD_M3_DY/2],
    [POD_CX + POD_M3_DX/2, POD_CY + POD_M3_DY/2]
];
BAT_SLOT_CX = 25.0; BAT_SLOT_CY = 35.5; BAT_SLOT_W = 13.0; BAT_SLOT_H = 9.0;

POD_OUTER_W = 92.0; POD_OUTER_R = 5.5;
POD_FRAME_WALL = 2.0; POD_WEB_THK = 1.8; POD_M3_SLEEVE_OD = 8.6;
POD_FLOOR_THK = 3.0; POD_SPINE_W = 24.0; POD_SPINE_H = 16.2; POLE_Q20_DY = 38.1;

POD_BAY_W = 64.5; POD_BAY_H = 124.0; POD_BAY_D = 12.8; POD_H = 132.0;
POD_CAPTURE_X1 = -12.0; POD_CAPTURE_Y1 = 38.75;
ADAPTER_SLIP = 0.25; ADAPTER_WALL = 2.0; ADAPTER_WEB_THK = 1.8;
BAT5_POCKET_X0 = -22.25; BAT5_POCKET_Y1 = 106.00;

// --- Option H1 Slim Sun Shield ---
SHIELD_U_X_LEFT = -15.40;
SHIELD_U_X_RIGHT = 52.20;
SHIELD_U_Y_BOT = 14.00;
SHIELD_U_Y_TOP = 102.00;
SHIELD_GROOVE_W = 1.90;
SHIELD_GROOVE_D = 1.80;
SHIELD_WALL_T = 1.60;
SHIELD_DEPTH_TOP = 28.0;
SHIELD_DEPTH_BOT = 12.0;

// --- 37.5-deg Laser-Mount Pole Tilt Wedge (Pole_Tilt_Wedge_37_5) ---
WEDGE_TILT_DEG = 37.5;
WEDGE_DU_SB = 14.0;
WEDGE_V_SB = 21.0;
WEDGE_MATE_GAP = 0.10;
WEDGE_SPINE_WEB = 8.0;
WEDGE_POCKET_D = 2.5;
WEDGE_POCKET_WALL = 3.0;
WEDGE_CORE_W = 32.0;
WEDGE_SHELF_W = 60.0;
WEDGE_SADDLE_CLR = 0.25;
WEDGE_SKIRT_T = 3.2;
WEDGE_SKIRT_DROP = 6.0;
WEDGE_FRONT_GUSSET_V = 5.5;
WEDGE_CHORD_T = 4.5;
WEDGE_BOSS_HALF_U = 8.0;
WEDGE_BOSS_V_REAR = 16.0;
WEDGE_BOSS_V_FRONT = 16.5;
WEDGE_MID_STRUT_T = 4.5;
WEDGE_TIE_STRUT_T = 4.0;
WEDGE_Q20_CLR = 6.80;
WEDGE_Q20_SHCS_CBORE = 11.40;

// =============================================================================
// Primitives
// =============================================================================
module rrect(w, h, d, r, cx=0, cy=0, zmin=0) {
    rr = min(r, min(w, h) / 2 - 0.05);
    translate([cx, cy, zmin])
        linear_extrude(height=d)
            offset(r=rr) offset(delta=-rr)
                square([w, h], center=true);
}

module csk_through(cx, cy, zbed, thk, hd, headd, cone_from_bed=true) {
    ch = max(0.4, (headd - hd) / 2);
    translate([cx, cy, zbed - 0.2]) cylinder(d=hd, h=thk + 0.4);
    if (cone_from_bed) {
        translate([cx, cy, zbed - 0.05]) cylinder(d1=headd + 0.1, d2=hd, h=ch + 0.05);
    } else {
        translate([cx, cy, zbed + thk - ch]) cylinder(d1=hd, d2=headd + 0.1, h=ch + 0.05);
    }
}

module round_y_bore(cx, y_start, y_len, zc, dia) {
    translate([cx, y_start, zc])
        rotate([-90, 0, 0])
            cylinder(d=dia, h=y_len);
}

module d_flat_y_bore(cx, y_start, y_len, zc, dia, flat_dist, flat_at_plus_z=false) {
    r = dia / 2;
    sign = flat_at_plus_z ? 1.0 : -1.0;
    flat_z = zc + sign * (flat_dist - r);
    slab_h = 3.0;
    z_min_clip = flat_at_plus_z ? flat_z : (flat_z - slab_h);
    difference() {
        round_y_bore(cx, y_start, y_len, zc, dia);
        translate([cx, y_start + y_len / 2, z_min_clip + slab_h / 2])
            cube([dia + 2, y_len + 2, slab_h], center=true);
    }
}

module arch_x_cut(xs, xlen, cy, wy, zmin, zmax, gable=1.5, peak_plus_z=true) {
    translate([xs, 0, 0])
        rotate([90, 0, 90])
            linear_extrude(height=xlen)
                if (peak_plus_z)
                    polygon([[cy-wy/2, zmin], [cy+wy/2, zmin], [cy+wy/2, zmax], [cy, zmax+gable], [cy-wy/2, zmax]]);
                else
                    polygon([[cy-wy/2, zmax], [cy+wy/2, zmax], [cy+wy/2, zmin], [cy, zmin-gable], [cy-wy/2, zmin]]);
}

// =============================================================================
// Parts
// =============================================================================
module back_chassis_frame() {
    wrap_w = (CASE_W/2 + 1.0) - TOP_CORNER_WRAP_X_IN;
    wrap_h = (CASE_H - TOP_CORNER_SPLIT_Y) + 1.0;
    wrap_cx = TOP_CORNER_WRAP_X_IN + wrap_w/2;
    wrap_cy = TOP_CORNER_SPLIT_Y + wrap_h/2;
    x_rim_ctr = (CASE_W/2 - WALL_SIDE - RIM_CLEAR) - RIM_THK/2;
    y_rim_ctr = (WALL_BOT + RIM_CLEAR) + RIM_THK/2;
    y_bot_m3 = COVER_M3_POINTS[0][1];
    y_top_m3 = COVER_M3_POINTS[4][1];
    x_bot_m3 = abs(COVER_M3_POINTS[0][0]);
    hp_rim_b = HP_CY - HP_TAB_W/2 - NOTCH_CLEAR;
    hp_rim_t = HP_CY + HP_TAB_W/2 + NOTCH_CLEAR;

    difference() {
        union() {
            // Base plate
            rrect(CASE_W, CASE_H, WALL_BACK, CORNER_R, 0, CASE_H/2, Z_PLATE_OUTER);
            // Integral Full-Width R6.0 Top L-Wall Bulkhead + 50/50 Split Top-Corner Piers (Y = 173.2..180.0)
            intersection() {
                rrect(CASE_W, CASE_H, 25.35, CORNER_R, 0, CASE_H/2, Z_PLATE_OUTER);
                union() {
                    translate([0, CASE_H - WALL_TOP/2 + 0.5, Z_PLATE_OUTER + 25.35/2])
                        cube([CASE_W + 2.0, WALL_TOP + 1.0, 26.35], center=true);
                    for (sg = [-1, 1])
                        translate([sg * wrap_cx, wrap_cy, Z_PLATE_OUTER + 25.35/2])
                            cube([wrap_w, wrap_h, 26.35], center=true);
                }
            }
            // Right Button-Plunger Flange + outer stop block + low bottom support rail (open to -X)
            translate([CASE_W/2 - WALL_SIDE/2, 0.5*(FLANGE_Y0+FLANGE_Y1), Z_PLATE_OUTER + 25.35/2])
                cube([WALL_SIDE, FLANGE_Y1 - FLANGE_Y0, 25.35], center=true);
            translate([0.5*(PLUNGER_BLK_X0+PLUNGER_BLK_X1) + 0.25, 0.5*(PLUNGER_BLK_Y0+PLUNGER_BLK_Y1),
                       0.5*(Z_PLATE_INNER + PLUNGER_BLK_Z_TOP) - 0.25])
                cube([PLUNGER_BLK_X1 - PLUNGER_BLK_X0 + 0.5, PLUNGER_BLK_Y1 - PLUNGER_BLK_Y0,
                      PLUNGER_BLK_Z_TOP - Z_PLATE_INNER + 0.5], center=true);
            translate([0.5*(PLUNGER_RAIL_X0 + PLUNGER_BLK_X0 + 0.5), 0.5*(PLUNGER_BLK_Y0+PLUNGER_BLK_Y1),
                       0.5*(Z_PLATE_INNER + PLUNGER_RAIL_Z_TOP) - 0.25])
                cube([(PLUNGER_BLK_X0 + 0.5) - PLUNGER_RAIL_X0, PLUNGER_BLK_Y1 - PLUNGER_BLK_Y0,
                      PLUNGER_RAIL_Z_TOP - Z_PLATE_INNER + 0.5], center=true);
            // Partial-height Left Headphone L-Tab (X = -60..-57, Y = 11.75..30.25, Z = -28.0..-13.50)
            translate([-CASE_W/2 + WALL_SIDE/2, HP_CY, 0.5*(Z_PLATE_OUTER + 0.5 + HP_TAB_Z_TOP)])
                cube([WALL_SIDE, HP_TAB_W, HP_TAB_Z_TOP - (Z_PLATE_OUTER + 0.5)], center=true);
            // Split Left around L-Tab, continuous Bottom, and Right inner cover-capture rim (Z = -25.0..-20.5)
            for (t = [[-x_rim_ctr, 0.5*(y_bot_m3 + hp_rim_b), RIM_THK, hp_rim_b - y_bot_m3],
                      [-x_rim_ctr, 0.5*(hp_rim_t + y_top_m3), RIM_THK, y_top_m3 - hp_rim_t],
                      [ 0.0,       y_rim_ctr,                2*x_bot_m3, RIM_THK],
                      [ x_rim_ctr, 0.5*(y_bot_m3 + PLUNGER_BLK_Y0 + 0.5), RIM_THK, (PLUNGER_BLK_Y0 + 0.5) - y_bot_m3],
                      [ x_rim_ctr, 0.5*(PLUNGER_BLK_Y1 - 0.5 + y_top_m3), RIM_THK, y_top_m3 - (PLUNGER_BLK_Y1 - 0.5)]])
                translate([t[0], t[1], Z_PLATE_INNER + (RIM_H - 0.5)/2])
                    cube([t[2], t[3], RIM_H + 0.5], center=true);
            // Symmetric 6 x Cover M3 bosses (full height up to COVER_BOSS_Z_TOP = -2.65)
            for (p = COVER_M3_POINTS)
                translate([p[0], p[1], Z_PLATE_OUTER + 0.5])
                    cylinder(d=RUTHEX_M3_BOSS, h=COVER_BOSS_Z_TOP - Z_PLATE_OUTER - 0.5);
            // Rear Option B2 Pod M3 bosses
            for (p = POD_M3_POINTS)
                translate([p[0], p[1], Z_PLATE_OUTER])
                    cylinder(d=RUTHEX_M3_BOSS, h=8.0);
            // Waveshare pillars
            for (sx = [-1, 1], sy = [-1, 1])
                translate([WS_CX + sx*WS_STANDOFF_DX/2, WS_CY + sy*WS_STANDOFF_DY/2, Z_PLATE_INNER - 0.5])
                    cylinder(d=5.0, h=(WS_STANDOFF_Z_END - Z_PLATE_INNER) + 0.5);
            // RTC Coin Cell cradle
            translate([RTC_CX, RTC_CY + 0.12, Z_PLATE_INNER - 0.4])
                cylinder(d=26.5, h=4.4);
            // Power Board & E220 M2 bosses
            for (sx = [-1, 1], sy = [-1, 1]) {
                translate([PB_CX + sx*PB_HOLE_DX/2, PB_CY + sy*PB_HOLE_DY/2, Z_PLATE_OUTER + 0.5])
                    cylinder(d=RUTHEX_M2_BOSS, h=(Z_PLATE_INNER + PB_STANDOFF_H - Z_PLATE_OUTER - 0.5));
                translate([E220_CX + sx*E220_HOLE_DX/2, E220_CY + sy*E220_HOLE_DY/2, Z_PLATE_OUTER + 0.5])
                    cylinder(d=RUTHEX_M2_BOSS, h=(Z_PLATE_INNER + E220_STANDOFF_H - Z_PLATE_OUTER - 0.5));
            }
            // UM982 pillars
            for (dx = UM_M2_X_OFFS, dy = UM_M2_Y_OFFS)
                translate([UM_LEFT_X + dx, UM_BOT_Y + dy, Z_PLATE_INNER - 0.5])
                    cylinder(d=5.4, h=(UM_BACK_Z - Z_PLATE_INNER) + 0.5);
            // Speaker open-center pocket frame (31.6 x 31.6 mm)
            rrect(SPK_W + 0.4 + 2*SPK_WALL, SPK_H + 0.4 + 2*SPK_WALL, SPK_PEDESTAL_H + 4.5, 2.5,
                  SPK_CX, SPK_CY, Z_PLATE_INNER - 0.5);
        }
        // Cut RTC cradle ID + opening
        translate([RTC_CX, RTC_CY + 0.12, Z_PLATE_INNER - 0.6]) cylinder(d=22.5, h=4.8);
        translate([RTC_CX, RTC_CY - 10.2, Z_PLATE_INNER + 1.8]) cube([22.0, 9.6, 4.8], center=true);
        // Cut Speaker upper seat pocket, 4-side top-to-bottom slots (corner L-pillars), & hollow acoustic center
        translate([SPK_CX, SPK_CY, Z_PLATE_INNER + SPK_PEDESTAL_H + 2.5])
            cube([SPK_W + 0.4, SPK_H + 0.4, 5.0], center=true);
        translate([SPK_CX, SPK_CY, Z_PLATE_INNER + (SPK_PEDESTAL_H + 5.5)/2])
            cube([SPK_SLOT_W, SPK_H + 0.4 + 2*SPK_WALL + 2.0, SPK_PEDESTAL_H + 5.5], center=true);
        translate([SPK_CX, SPK_CY, Z_PLATE_INNER + (SPK_PEDESTAL_H + 5.5)/2])
            cube([SPK_W + 0.4 + 2*SPK_WALL + 2.0, SPK_SLOT_W, SPK_PEDESTAL_H + 5.5], center=true);
        rrect(SPK_W + 0.4 - 2*SPK_RIDGE_W, SPK_H + 0.4 - 2*SPK_RIDGE_W, SPK_PEDESTAL_H + 5.0, 2.0,
              SPK_CX, SPK_CY, Z_PLATE_INNER);
        // Symmetric 6 x M3 Ruthex insert bores in full-height Cover M3 bosses
        for (p = COVER_M3_POINTS)
            translate([p[0], p[1], COVER_BOSS_Z_TOP - (RUTHEX_M3_DEPTH + 1.3)])
                cylinder(d=RUTHEX_M3_OD, h=RUTHEX_M3_DEPTH + 1.6);
        for (p = POD_M3_POINTS)
            translate([p[0], p[1], Z_PLATE_OUTER - 0.3]) cylinder(d=RUTHEX_M3_OD, h=RUTHEX_M3_DEPTH + 0.5);
        // M2 thru CSK holes for Waveshare & UM982 + Ruthex M2 bores for PB & E220
        for (sx = [-1, 1], sy = [-1, 1]) {
            csk_through(WS_CX + sx*WS_STANDOFF_DX/2, WS_CY + sy*WS_STANDOFF_DY/2,
                        Z_PLATE_OUTER, WS_STANDOFF_Z_END - Z_PLATE_OUTER, HOLE_CLEAR_M2, 4.6, true);
            translate([PB_CX + sx*PB_HOLE_DX/2, PB_CY + sy*PB_HOLE_DY/2, Z_PLATE_INNER + PB_STANDOFF_H - RUTHEX_M2_DEPTH])
                cylinder(d=RUTHEX_M2_OD, h=RUTHEX_M2_DEPTH + 0.3);
            translate([E220_CX + sx*E220_HOLE_DX/2, E220_CY + sy*E220_HOLE_DY/2, Z_PLATE_INNER + E220_STANDOFF_H - RUTHEX_M2_DEPTH])
                cylinder(d=RUTHEX_M2_OD, h=RUTHEX_M2_DEPTH + 0.3);
        }
        for (dx = UM_M2_X_OFFS, dy = UM_M2_Y_OFFS) {
            translate([UM_LEFT_X + dx, UM_BOT_Y + dy, Z_PLATE_OUTER - 0.5])
                cylinder(d=4.7, h=UM_CSK_RECESS + 0.46);
            csk_through(UM_LEFT_X + dx, UM_BOT_Y + dy, Z_PLATE_OUTER + UM_CSK_RECESS, UM_BACK_Z - (Z_PLATE_OUTER + UM_CSK_RECESS), HOLE_CLEAR_M2, 4.6, true);
        }
        // Battery JST pass-through slot
        rrect(BAT_SLOT_W, BAT_SLOT_H, WALL_BACK + 1.0, 4.4, BAT_SLOT_CX, BAT_SLOT_CY, Z_PLATE_OUTER - 0.5);
        // UM982 L-wall recess & crown bores (GX-12 & E220 SMA w/ downward -Z_asm D-flats)
        translate([UM_CX, 177.25, Z_PLATE_INNER + 12.5])
            cube([UM_W + 0.60, 1.50, 25.0], center=true);
        d_flat_y_bore(GX12_X, 176.5, 4.5, GX12_Z, GX12_HOLE_DIA, GX12_FLAT, false);
        round_y_bore(GX12_X, 174.0, 180.0 - GX12_WALL - 174.0, GX12_Z, GX12_NUT_RECESS_DIA);
        d_flat_y_bore(E220_SMA_X, 176.5, 4.5, E220_SMA_Z, E220_SMA_DIA, E220_SMA_FLAT, false);
        round_y_bore(E220_SMA_X, 174.0, 180.0 - E220_SMA_WALL - 174.0, E220_SMA_Z, 11.5);
        round_y_bore(UM_SMA_SLAVE_X, 175.0, 6.0, UM_SMA_Z, UM_SMA_ACCESS_DIA);
        round_y_bore(UM_SMA_MASTER_X, 175.0, 6.0, UM_SMA_Z, UM_SMA_ACCESS_DIA);
        // Headphone jack M6 bore + inner 1.50 mm anti-rotation recess w/ 45-deg top bevel (Left L-Tab)
        translate([-CASE_W/2 - 0.5, HP_CY, HP_Z_CTR])
            rotate([0, 90, 0]) cylinder(d=HP_BORE_DIA, h=WALL_SIDE + 1.0);
        translate([0, HP_CY + HP_RECESS_W/2, 0])
            rotate([90, 0, 0])
                linear_extrude(height=HP_RECESS_W)
                    polygon([
                        [-CASE_W/2 + WALL_SIDE + 0.5, Z_PLATE_INNER],
                        [-CASE_W/2 + HP_WALL, Z_PLATE_INNER],
                        [-CASE_W/2 + HP_WALL, HP_Z_CTR + HP_RECESS_H/2],
                        [-CASE_W/2 + WALL_SIDE, HP_Z_CTR + HP_RECESS_H/2 + (WALL_SIDE - HP_WALL)],
                        [-CASE_W/2 + WALL_SIDE + 0.5, HP_Z_CTR + HP_RECESS_H/2 + (WALL_SIDE - HP_WALL)]
                    ]);
        // Pure round outer plunger bearing bores (Dia 4.60 mm, open to -X)
        for (by = WS_BTN_Y_LIST)
            translate([55.5, by, WS_BTN_Z_CTR]) rotate([0, 90, 0]) cylinder(d=4.60, h=5.3);
    }
}

module top_cover() {
    cover_depth = abs(Z_PLATE_INNER) - 0.15;
    z_bot = -cover_depth;
    top_notch_y0 = TOP_CORNER_SPLIT_Y - NOTCH_CLEAR;
    top_notch_y1 = CASE_H + 0.5;
    u_cx = 0.5 * (SHIELD_U_X_LEFT + SHIELD_U_X_RIGHT);
    u_w = SHIELD_U_X_RIGHT - SHIELD_U_X_LEFT;
    u_cy = 0.5 * (SHIELD_U_Y_BOT + SHIELD_U_Y_TOP);
    u_h = SHIELD_U_Y_TOP - SHIELD_U_Y_BOT;
    comb_cx = 0.5 * (PLUNGER_U_X0 + PLUNGER_U_X1);
    comb_w = PLUNGER_U_X1 - PLUNGER_U_X0;
    comb_cy = 0.5 * (PLUNGER_BLK_Y0 + PLUNGER_BLK_Y1);
    comb_h = PLUNGER_BLK_Y1 - PLUNGER_BLK_Y0;

    difference() {
        union() {
            difference() {
                rrect(CASE_W, CASE_H, cover_depth, CORNER_R, 0, CASE_H/2, z_bot);
                // Interior cavity
                rrect(CASE_W - 2*WALL_SIDE, CASE_H - WALL_BOT - WALL_TOP, cover_depth - WALL_FRONT + 0.5,
                      max(1.5, CORNER_R - WALL_SIDE), 0, CASE_H/2, z_bot - 0.5);
                // Full-width Top L-wall crown & split-corner notch (Y = 173.0..180.5; keeps lower-half corner wrap & straight +Z lift-off)
                translate([0, 0.5*(top_notch_y0 + top_notch_y1), 0.5*(z_bot + Z_FRONT_INNER) - 0.25])
                    cube([CASE_W + 2.0, top_notch_y1 - top_notch_y0, cover_depth - WALL_FRONT + 0.5], center=true);
                // Right Flange notch
                translate([CASE_W/2 - 1.2, 0.5*(FLANGE_Y0+FLANGE_Y1), 0.5*(z_bot + Z_FRONT_INNER) - 0.25])
                    cube([WALL_SIDE + 2.0, (FLANGE_Y1 - FLANGE_Y0) + 2*NOTCH_CLEAR, cover_depth - WALL_FRONT + 0.5], center=true);
                // Left Headphone L-Tab notch (keeps continuous 13.30 mm front left skirt & bottom wall support-free)
                translate([-CASE_W/2 + 0.5*WALL_SIDE, HP_CY, 0.5*(z_bot + HP_TAB_Z_TOP + NOTCH_CLEAR) - 0.25])
                    cube([WALL_SIDE + 1.0, HP_TAB_W + 2*NOTCH_CLEAR, (HP_TAB_Z_TOP + NOTCH_CLEAR - z_bot) + 0.5], center=true);
            }
            // U-groove reinforcing backing rib
            translate([SHIELD_U_X_LEFT, u_cy, -2.7]) cube([4.0, u_h + 2.5, 1.4], center=true);
            translate([SHIELD_U_X_RIGHT, u_cy, -2.7]) cube([4.0, u_h + 2.5, 1.4], center=true);
            translate([u_cx, SHIELD_U_Y_TOP, -2.7]) cube([u_w + 4.0, 4.0, 1.4], center=true);
            // Downward inverted-U plunger capture comb (slots open UP when printing face-down)
            translate([comb_cx, comb_cy, 0.5*(PLUNGER_U_Z_BOT - 2.0)])
                cube([comb_w, comb_h, -2.0 - PLUNGER_U_Z_BOT], center=true);
        }
        // Full-height boss relief pockets (Z = -24.85..Z_FRONT_INNER = -2.40) around all 6 Back_Chassis_Frame M3 bosses
        for (p = COVER_M3_POINTS)
            translate([p[0], p[1], z_bot - 0.5]) cylinder(d=8.2, h=cover_depth - WALL_FRONT + 0.5);
        // Downward-open U-slots in capture comb
        for (by = WS_BTN_Y_LIST) {
            translate([PLUNGER_U_X0 - 0.5, by, WS_BTN_Z_CTR])
                rotate([0, 90, 0]) cylinder(d=PLUNGER_U_SLOT_W, h=comb_w + 1.0);
            translate([comb_cx, by, 0.5*(PLUNGER_U_Z_BOT - 0.5 + WS_BTN_Z_CTR)])
                cube([comb_w + 1.0, PLUNGER_U_SLOT_W, WS_BTN_Z_CTR - (PLUNGER_U_Z_BOT - 0.5)], center=true);
        }
        // Front CSK M3 thru holes through 2.40 mm faceplate
        for (p = COVER_M3_POINTS)
            csk_through(p[0], p[1], Z_FRONT_INNER, WALL_FRONT, HOLE_CLEAR_M3, 6.2, false);
        // Recessed Option H1 Sun Shield U-groove
        translate([SHIELD_U_X_LEFT, u_cy, -SHIELD_GROOVE_D/2 + 0.1])
            cube([SHIELD_GROOVE_W, u_h + SHIELD_GROOVE_W, SHIELD_GROOVE_D + 0.2], center=true);
        translate([SHIELD_U_X_RIGHT, u_cy, -SHIELD_GROOVE_D/2 + 0.1])
            cube([SHIELD_GROOVE_W, u_h + SHIELD_GROOVE_W, SHIELD_GROOVE_D + 0.2], center=true);
        translate([u_cx, SHIELD_U_Y_TOP, -SHIELD_GROOVE_D/2 + 0.1])
            cube([u_w + SHIELD_GROOVE_W, SHIELD_GROOVE_W, SHIELD_GROOVE_D + 0.2], center=true);
        // Flush glass opening (R3.0 + FIT_CLEAR)
        rrect(WS_GLASS_W + 2*FIT_CLEAR, WS_GLASS_H + 2*FIT_CLEAR, 4.5, WS_GLASS_R + FIT_CLEAR, WS_CX, WS_CY, -4.0);
        // USB-C gabled cutout
        translate([WS_CX, -0.5, WS_USBC_Z_CTR])
            rotate([-90, 0, 0])
                linear_extrude(height=WALL_BOT + 1.5)
                    polygon([
                        [-WS_USBC_W/2, -WS_USBC_H_OPEN/2], [WS_USBC_W/2, -WS_USBC_H_OPEN/2],
                        [WS_USBC_W/2, WS_USBC_H_OPEN/2], [0, WS_USBC_H_OPEN/2 + 2.0],
                        [-WS_USBC_W/2, WS_USBC_H_OPEN/2]
                    ]);
        // MicroSD cutout + external finger scoop on Right Wall (prints face-down -> gable at -Z_asm!)
        arch_x_cut(56.2, 4.5, 0.5*(WS_SD_Y0+WS_SD_Y1), WS_SD_Y1 - WS_SD_Y0,
                   WS_SD_Z_CTR - 2.6, WS_SD_Z_CTR + 2.6, 1.6, false);
        arch_x_cut(58.8, 2.0, 0.5*(WS_SD_Y0+WS_SD_Y1), WS_SD_Y1 - WS_SD_Y0 + 3.0,
                   WS_SD_Z_CTR - 3.5, WS_SD_Z_CTR + 3.5, 2.0, false);
        // Speaker acoustic grille
        for (dy = [-8, -4, 0, 4, 8])
            rrect(18.0, 2.0, 3.5, 0.9, SPK_CX, SPK_CY + dy, -3.0);
        // 4 UM982 LEDs + SWITCH pinhole
        for (lx = UM_LED_X_OFFS)
            translate([UM_LEFT_X + lx, UM_LED_Y, -3.0]) cylinder(d=1.95, h=3.5);
        translate([UM_SW_X, UM_SW_Y, -3.0]) cylinder(d=3.4, h=3.5);
    }
}

module strut2d(x0, y0, x1, y1, w) {
    dx = x1 - x0; dy = y1 - y0;
    L = sqrt(dx*dx + dy*dy) + w;
    ang = atan2(dy, dx);
    translate([0.5*(x0 + x1), 0.5*(y0 + y1)])
        rotate([0, 0, ang])
            square([L, w], center=true);
}

module rrect2d(w, h, r, cx=0, cy=0) {
    rr = min(r, min(w, h) / 2 - 0.05);
    translate([cx, cy]) offset(r=rr) offset(delta=-rr) square([w, h], center=true);
}

module battery_cradle() {
    bay_w = POD_BAY_W; bay_h = POD_BAY_H; bay_d = POD_BAY_D; pod_h = POD_H;
    z_back = Z_PLATE_OUTER - bay_d;
    bay_cy = POD_CY + 1.0;
    in_xl = POD_CX - (bay_w/2 + POD_FRAME_WALL/2);
    in_xr = POD_CX + (bay_w/2 + POD_FRAME_WALL/2);
    in_yb = bay_cy - (bay_h/2 + POD_FRAME_WALL/2);
    in_yt = bay_cy + (bay_h/2 + POD_FRAME_WALL/2);
    out_xl = POD_CX - (POD_OUTER_W/2 - POD_FRAME_WALL/2);
    out_xr = POD_CX + (POD_OUTER_W/2 - POD_FRAME_WALL/2);
    out_yb = POD_CY - (pod_h/2 - POD_FRAME_WALL/2);
    out_yt = POD_CY + (pod_h/2 - POD_FRAME_WALL/2);
    n_bays = 6;
    y_lo = POD_M3_POINTS[0][1];
    y_hi = POD_M3_POINTS[2][1];
    cap_x0 = (POD_CX - bay_w/2) - 0.5; cap_x1 = POD_CAPTURE_X1;
    cap_y0 = (bay_cy - bay_h/2) - 0.5; cap_y1 = POD_CAPTURE_Y1;

    difference() {
        union() {
            difference() {
                union() {
                    difference() {
                        rrect(POD_OUTER_W, pod_h, bay_d, POD_OUTER_R, POD_CX, POD_CY, z_back);
                        rrect(POD_OUTER_W - 2*POD_FRAME_WALL, pod_h - 2*POD_FRAME_WALL, bay_d + 1.0,
                              max(1.5, POD_OUTER_R - POD_FRAME_WALL), POD_CX, POD_CY, z_back - 0.5);
                    }
                    difference() {
                        rrect(bay_w + 2*POD_FRAME_WALL, bay_h + 2*POD_FRAME_WALL, bay_d, 3.5, POD_CX, bay_cy, z_back);
                        rrect(bay_w, bay_h, bay_d + 1.0, 2.5, POD_CX, bay_cy, z_back - 0.5);
                    }
                    for (p = POD_M3_POINTS)
                        translate([p[0], p[1], z_back]) cylinder(d=POD_M3_SLEEVE_OD, h=bay_d);
                    // Interlocking Warren-truss web
                    translate([0, 0, z_back]) linear_extrude(height=bay_d) intersection() {
                        difference() {
                            rrect2d(POD_OUTER_W - POD_FRAME_WALL, pod_h - POD_FRAME_WALL,
                                    max(1.5, POD_OUTER_R - POD_FRAME_WALL/2), POD_CX, POD_CY);
                            rrect2d(bay_w + POD_FRAME_WALL, bay_h + POD_FRAME_WALL, 3.0, POD_CX, bay_cy);
                        }
                        union() {
                            for (p = POD_M3_POINTS) {
                                sx = (p[0] < POD_CX) ? -1 : 1;
                                sy = (p[1] < POD_CY) ? -1 : 1;
                                ix = (sx < 0) ? in_xl : in_xr;
                                ox = (sx < 0) ? out_xl : out_xr;
                                oy = (sy < 0) ? out_yb : out_yt;
                                strut2d(ox, p[1], ix, p[1], POD_WEB_THK);
                                strut2d(p[0], oy, p[0], p[1], POD_WEB_THK);
                                strut2d(p[0], p[1], ix, max(in_yb, min(in_yt, p[1] - sy*16.0)), POD_WEB_THK);
                            }
                            for (side = [-1, 1]) {
                                ix = (side < 0) ? in_xl : in_xr;
                                ox = (side < 0) ? out_xl : out_xr;
                                for (i = [0 : n_bays - 1]) {
                                    ya = y_lo + i*(y_hi - y_lo)/n_bays;
                                    yb = y_lo + (i+1)*(y_hi - y_lo)/n_bays;
                                    if (i % 2 == 0) strut2d(ox, ya, ix, yb, POD_WEB_THK);
                                    else strut2d(ix, ya, ox, yb, POD_WEB_THK);
                                }
                                strut2d(ox, POD_CY, ix, POD_CY, POD_WEB_THK);
                            }
                            for (tx = [POD_CX - 20.0, POD_CX, POD_CX + 20.0]) {
                                strut2d(tx, out_yb, tx, in_yb, POD_WEB_THK);
                                strut2d(tx, in_yt, tx, out_yt, POD_WEB_THK);
                            }
                            for (side = [-1, 1]) {
                                ix = (side < 0) ? in_xl : in_xr;
                                ox = (side < 0) ? out_xl : out_xr;
                                strut2d(ox, out_yb + 2.0, ix, y_lo, POD_WEB_THK);
                                strut2d(ox, out_yt - 2.0, ix, y_hi, POD_WEB_THK);
                            }
                        }
                    }
                }
                rrect(bay_w, bay_h, bay_d + 1.0, 2.5, POD_CX, bay_cy, z_back - 0.5);
            }
            // Integral Bottom-Left Battery Capture Corner
            intersection() {
                rrect(cap_x1 - cap_x0, cap_y1 - cap_y0, bay_d, 1.5,
                      0.5*(cap_x0+cap_x1), 0.5*(cap_y0+cap_y1), z_back);
                rrect(bay_w + 1.0, bay_h + 1.0, bay_d + 0.2, 2.8,
                      POD_CX, bay_cy, z_back - 0.1);
            }
        }
        for (p = POD_M3_POINTS)
            translate([p[0], p[1], z_back - 1.5]) cylinder(d=HOLE_CLEAR_M3, h=bay_d + 3.0);
    }
}

module pole_lid() {
    bay_d = POD_BAY_D;
    pod_h = POD_H;
    z_front = Z_PLATE_OUTER - bay_d - 0.10;
    z_back = z_front - POD_FLOOR_THK;
    z_spine = z_back - POD_SPINE_H;
    difference() {
        union() {
            rrect(POD_OUTER_W, pod_h, POD_FLOOR_THK, POD_OUTER_R, POD_CX, POD_CY, z_back);
            rrect(POD_SPINE_W, 74.0, POD_SPINE_H + 0.5, 5.0, POD_CX, POD_CY, z_spine);
        }
        for (qy = [POD_CY - POLE_Q20_DY/2, POD_CY + POLE_Q20_DY/2])
            translate([POD_CX, qy, z_spine - 0.2]) cylinder(d=RUTHEX_Q20_OD, h=RUTHEX_Q20_DEPTH + 0.2);
        for (p = POD_M3_POINTS)
            csk_through(p[0], p[1], z_back, POD_FLOOR_THK, HOLE_CLEAR_M3, 6.2, true);
    }
}

module battery_adapter_5000() {
    bay_w = POD_BAY_W; bay_h = POD_BAY_H; bay_d = POD_BAY_D;
    bay_cy = POD_CY + 1.0;
    z_back = Z_PLATE_OUTER - bay_d;
    ad_z0 = z_back + 0.10;
    ad_h = bay_d - 0.20;
    slip = ADAPTER_SLIP; wall = ADAPTER_WALL; web_t = ADAPTER_WEB_THK;
    ad_xl = (POD_CX - bay_w/2) + slip;
    ad_xr = (POD_CX + bay_w/2) - slip;
    ad_yb = POD_CAPTURE_Y1 + slip;
    ad_yt = (bay_cy + bay_h/2) - slip;
    p5_xl = BAT5_POCKET_X0;
    p5_yt = BAT5_POCKET_Y1;
    tx0 = ad_xl + wall/2; tx1 = ad_xr - wall/2;
    ty0 = p5_yt + wall/2; ty1 = ad_yt - wall/2;
    tm = 0.5*(tx0 + tx1);

    union() {
        difference() {
            intersection() {
                rrect(ad_xr - ad_xl, (bay_cy + bay_h/2 - slip) - (bay_cy - bay_h/2 + slip),
                      ad_h, 2.5, POD_CX, bay_cy, ad_z0);
                union() {
                    translate([0.5*(ad_xl+ad_xr), 0.5*(p5_yt+ad_yt), ad_z0 + ad_h/2])
                        cube([ad_xr - ad_xl, ad_yt - p5_yt, ad_h], center=true);
                    translate([0.5*(ad_xl+p5_xl), 0.5*(ad_yb+p5_yt+wall), ad_z0 + ad_h/2])
                        cube([p5_xl - ad_xl, (p5_yt + wall) - ad_yb, ad_h], center=true);
                }
            }
            rrect((ad_xr - ad_xl) - 2*wall, (ad_yt - p5_yt) - 2*wall, ad_h + 1.0, 1.5,
                  0.5*(ad_xl+ad_xr), 0.5*(p5_yt+ad_yt), ad_z0 - 0.5);
            rrect((p5_xl - ad_xl) - 2*wall, (p5_yt - ad_yb) - 2*wall, ad_h + 1.0, 1.0,
                  0.5*(ad_xl+p5_xl), 0.5*(ad_yb+p5_yt), ad_z0 - 0.5);
        }
        intersection() {
            intersection() {
                rrect(ad_xr - ad_xl, (bay_cy + bay_h/2 - slip) - (bay_cy - bay_h/2 + slip),
                      ad_h, 2.5, POD_CX, bay_cy, ad_z0);
                union() {
                    translate([0.5*(ad_xl+ad_xr), 0.5*(p5_yt+ad_yt), ad_z0 + ad_h/2])
                        cube([ad_xr - ad_xl, ad_yt - p5_yt, ad_h], center=true);
                    translate([0.5*(ad_xl+p5_xl), 0.5*(ad_yb+p5_yt+wall), ad_z0 + ad_h/2])
                        cube([p5_xl - ad_xl, (p5_yt + wall) - ad_yb, ad_h], center=true);
                }
            }
            translate([0, 0, ad_z0]) linear_extrude(height=ad_h) union() {
                strut2d(tx0, ty0, tm, ty1, web_t);
                strut2d(tm, ty1, tx1, ty0, web_t);
                strut2d(tx0, ty1, tm, ty0, web_t);
                strut2d(tm, ty0, tx1, ty1, web_t);
                strut2d(ad_xl + wall/2, ad_yb + (p5_yt - ad_yb)/3, p5_xl - wall/2, ad_yb + (p5_yt - ad_yb)/3, web_t);
                strut2d(ad_xl + wall/2, ad_yb + 2*(p5_yt - ad_yb)/3, p5_xl - wall/2, ad_yb + 2*(p5_yt - ad_yb)/3, web_t);
            }
        }
    }
}

module slim_sun_shield() {
    xl = SHIELD_U_X_LEFT; xr = SHIELD_U_X_RIGHT;
    yb = SHIELD_U_Y_BOT; yt = SHIELD_U_Y_TOP;
    ht = SHIELD_WALL_T / 2;
    zb = -(SHIELD_GROOVE_D - 0.10);
    difference() {
        translate([0, 0, zb])
            linear_extrude(height=SHIELD_DEPTH_TOP - zb)
                polygon([
                    [xl - ht, yb], [xl - ht, yt + ht], [xr + ht, yt + ht], [xr + ht, yb],
                    [xr - ht, yb], [xr - ht, yt - ht], [xl + ht, yt - ht], [xl + ht, yb]
                ]);
        translate([xl - 5, 0, 0])
            rotate([90, 0, 90])
                linear_extrude(height=(xr - xl) + 10)
                    polygon([
                        [yb - 5, SHIELD_DEPTH_BOT], [yt - 10, SHIELD_DEPTH_TOP],
                        [yt + 5, SHIELD_DEPTH_TOP], [yt + 5, SHIELD_DEPTH_TOP + 10],
                        [yb - 5, SHIELD_DEPTH_TOP + 10]
                    ]);
    }
}

module button_plungers() {
    for (by = WS_BTN_Y_LIST) {
        translate([45.65, by, WS_BTN_Z_CTR]) rotate([0, 90, 0]) cylinder(d=PLUNGER_NOSE_DIA, h=7.60);
        translate([53.15, by, WS_BTN_Z_CTR]) rotate([0, 90, 0])
            hull() {
                cube([5.40, 5.80, 0.2], center=true);
                translate([0, 0, 0.80]) cube([5.40, 5.80, 0.2], center=true);
                translate([0, 0, 2.70]) cylinder(d=4.10, h=0.2, center=true);
            }
        translate([55.80, by, WS_BTN_Z_CTR]) rotate([0, 90, 0]) cylinder(d=4.10, h=5.80);
    }
}

module pole_tilt_wedge() {
    z_mate = Z_PLATE_OUTER - POD_BAY_D - 0.10 - POD_FLOOR_THK - POD_SPINE_H - WEDGE_MATE_GAP;
    y_sp_bot = POD_CY - 37.0;
    s = sin(WEDGE_TILT_DEG);
    c = cos(WEDGE_TILT_DEG);
    rim = FIT_CLEAR + WEDGE_POCKET_WALL;
    p_bot_lip = [WEDGE_V_SB - rim*c + WEDGE_POCKET_D*s, WEDGE_DU_SB + rim*s + WEDGE_POCKET_D*c];
    p_top_lip = [WEDGE_V_SB + (74.0 + rim)*c + WEDGE_POCKET_D*s, WEDGE_DU_SB - (74.0 + rim)*s + WEDGE_POCKET_D*c];

    u_in = WEDGE_SHELF_W / 2 + WEDGE_SADDLE_CLR;
    u_out = u_in + WEDGE_SKIRT_T;
    u_rear_in = -u_out + WEDGE_CHORD_T;
    rhs_sp = WEDGE_V_SB*s + WEDGE_DU_SB*c - WEDGE_SPINE_WEB;
    p_sp_bot_bk = [WEDGE_V_SB - rim*c - WEDGE_SPINE_WEB*s, WEDGE_DU_SB + rim*s - WEDGE_SPINE_WEB*c];
    va = p_bot_lip[0]; ua = p_bot_lip[1];
    vb = WEDGE_FRONT_GUSSET_V; ub = u_out;
    l_frt = sqrt((vb - va)*(vb - va) + (ub - ua)*(ub - ua));
    nv = -(ub - ua) / l_frt; nu = (vb - va) / l_frt;
    rhs_frt = va*nv + ua*nu + WEDGE_CHORD_T;
    k0 = [WEDGE_CHORD_T, u_rear_in];
    k1 = [(rhs_sp - c*u_rear_in) / s, u_rear_in];
    k_frt_top = [p_sp_bot_bk[0], (rhs_frt - nv*p_sp_bot_bk[0]) / nu];
    k_frt_bot = [WEDGE_CHORD_T, (rhs_frt - nv*WEDGE_CHORD_T) / nu];
    mid_sp_p0 = [WEDGE_V_SB + 37.0*c - 6.5*s, WEDGE_DU_SB - 37.0*s - 6.5*c];

    prof_vu = [
        [-WEDGE_SKIRT_DROP, u_out],
        [-WEDGE_SKIRT_DROP, u_in],
        [0.0, u_in],
        [0.0, -u_in],
        [-WEDGE_SKIRT_DROP, -u_in],
        [-WEDGE_SKIRT_DROP, -u_out],
        [p_top_lip[0] - (u_out + p_top_lip[1]), -u_out],
        p_top_lip,
        p_bot_lip,
        [WEDGE_FRONT_GUSSET_V, u_out]
    ];

    pw = POD_SPINE_W + 2*FIT_CLEAR;
    ph = 74.0 + 2*FIT_CLEAR;
    pr = 5.0 + FIT_CLEAR;

    difference() {
        translate([POD_CX, y_sp_bot, z_mate])
            rotate([WEDGE_TILT_DEG, 0, 0])
                translate([0, -WEDGE_V_SB, -WEDGE_DU_SB])
                    difference() {
                        // Skeletonized 32.0 mm open Warren-truss frame w/ integral skirts & base boss
                        rotate([90, 0, 90])
                            translate([0, 0, -WEDGE_CORE_W/2])
                                linear_extrude(height=WEDGE_CORE_W)
                                    intersection() {
                                        polygon(prof_vu);
                                        union() {
                                            difference() {
                                                polygon(prof_vu);
                                                polygon([k0, k1, p_sp_bot_bk, k_frt_top, k_frt_bot]);
                                            }
                                            polygon([
                                                [0.0, -WEDGE_BOSS_HALF_U],
                                                [WEDGE_BOSS_V_REAR, -WEDGE_BOSS_HALF_U],
                                                [WEDGE_BOSS_V_FRONT, WEDGE_BOSS_HALF_U],
                                                [0.0, WEDGE_BOSS_HALF_U]
                                            ]);
                                            strut2d(mid_sp_p0[0], mid_sp_p0[1], 35.0, -31.5, WEDGE_MID_STRUT_T);
                                            strut2d(11.5, -7.0, 3.5, -21.0, WEDGE_TIE_STRUT_T);
                                            strut2d(14.5, 7.0, 3.5, 24.5, WEDGE_TIE_STRUT_T);
                                        }
                                    }
                        // Pure round base 1/4"-20 Ruthex bore + 0.5 mm lead-in cone
                        translate([0, -0.5, 0]) rotate([-90, 0, 0])
                            cylinder(d=RUTHEX_Q20_OD, h=RUTHEX_Q20_DEPTH + 0.5);
                        translate([0, -0.2, 0]) rotate([-90, 0, 0])
                            cylinder(d1=RUTHEX_Q20_OD + 1.4, d2=RUTHEX_Q20_OD, h=0.7);
                    }
        // Lofted 45-deg upper (+X_w) funnel spine alignment pocket (24.5 x 74.5 mm, R5.25)
        hull() {
            rrect(pw, ph, 0.02, pr, POD_CX, POD_CY, z_mate);
            rrect(pw + WEDGE_POCKET_D + 1.0, ph, 0.02, pr,
                  POD_CX + 0.5*(WEDGE_POCKET_D + 1.0), POD_CY, z_mate + WEDGE_POCKET_D + 1.0);
        }
        // Pure round twin 1/4"-20 SHCS through-bores & rear wrench-access holes
        for (qy = [POD_CY - POLE_Q20_DY/2, POD_CY + POLE_Q20_DY/2]) {
            translate([POD_CX, qy, z_mate - WEDGE_SPINE_WEB - 1.0])
                cylinder(d=WEDGE_Q20_CLR, h=WEDGE_SPINE_WEB + WEDGE_POCKET_D + 2.0);
            translate([POD_CX, qy, z_mate - WEDGE_SPINE_WEB - 80.0])
                cylinder(d=WEDGE_Q20_SHCS_CBORE, h=80.0);
        }
    }
}

// =============================================================================
// Selection & Print Orientations
// =============================================================================
if (PART == "back") back_chassis_frame();
else if (PART == "top") top_cover();
else if (PART == "cradle") battery_cradle();
else if (PART == "lid") pole_lid();
else if (PART == "adapter5") battery_adapter_5000();
else if (PART == "wedge") pole_tilt_wedge();
else if (PART == "shield") slim_sun_shield();
else if (PART == "plungers") button_plungers();
else if (PART == "back_print") translate([0, 0, -Z_PLATE_OUTER]) back_chassis_frame();
else if (PART == "top_print") rotate([180, 0, 0]) top_cover();
else if (PART == "cradle_print") rotate([180, 0, 0]) translate([0, 0, -Z_PLATE_OUTER]) battery_cradle();
else if (PART == "lid_print") rotate([180, 0, 0]) pole_lid();
else if (PART == "adapter5_print") rotate([180, 0, 0]) translate([0, 0, -Z_PLATE_OUTER]) battery_adapter_5000();
else if (PART == "wedge_print")
    translate([0, 0, WEDGE_CORE_W/2])
        rotate([0, -90, 0])
            translate([0, WEDGE_V_SB, WEDGE_DU_SB])
                rotate([-WEDGE_TILT_DEG, 0, 0])
                    translate([-POD_CX, -(POD_CY - 37.0), -(Z_PLATE_OUTER - POD_BAY_D - 0.10 - POD_FLOOR_THK - POD_SPINE_H - WEDGE_MATE_GAP)])
                        pole_tilt_wedge();
else if (PART == "shield_print") translate([0, 0, SHIELD_GROOVE_D - 0.10]) slim_sun_shield();
else if (PART == "plungers_print") rotate([0, -90, 0]) translate([-61.6, 0, 0]) button_plungers();
else {
    color("SlateGray") translate([0, 0, EXPLODE_MM]) top_cover();
    color("SteelBlue") back_chassis_frame();
    color("DarkSlateGray") translate([0, 0, -EXPLODE_MM]) battery_cradle();
    if (BATTERY_OPT == "5000")
        color("Teal") translate([0, 0, -1.5*EXPLODE_MM]) battery_adapter_5000();
    color("DimGray") translate([0, 0, -2*EXPLODE_MM]) pole_lid();
    color("SlateBlue") translate([0, 0, -3*EXPLODE_MM]) pole_tilt_wedge();
    color("Orange") translate([0, 0, 2*EXPLODE_MM]) slim_sun_shield();
    color("Gold") translate([EXPLODE_MM * 0.4, 0, 0]) button_plungers();
    if (SHOW_MODULES) {
        %rrect(WS_GLASS_W, WS_GLASS_H, 11.5, WS_GLASS_R, WS_CX, WS_CY, -11.5);
        %cylinder(d=21.5, h=4.0);
        %rrect(PB_W, PB_H, 20.5, 0.5, PB_CX, PB_CY, Z_PLATE_INNER + PB_STANDOFF_H - 2.0);
        %rrect(E220_W, E220_H, 16.0, 0.5, E220_CX, E220_CY, Z_PLATE_INNER + E220_STANDOFF_H - 3.0);
        %rrect(UM_W, UM_BODY_H, UM_Z_THK, 1.0, UM_CX, UM_CY, UM_BACK_Z);
        %rrect(SPK_W, SPK_H, SPK_Z_THK, 1.0, SPK_CX, SPK_CY, Z_PLATE_INNER + SPK_PEDESTAL_H);
    }
}
