// =============================================================================
// RTK Rover Option A Enclosure v2.0.0 (PETG Support-Free, Snapmaker 2)
// Parametric OpenSCAD 2026.11.18 mirror of rover_specs.py v2.0.0
// Coordinate origin: X = centered [-60..+60], Y = [0..180], Z = 0 front glass
// Units: millimeters (mm)
// =============================================================================

/* [View & Export Selector] */
PART = "all"; // ["all", "back", "top", "cradle5", "lid5", "cradle10", "lid10", "shield", "plungers", "back_print", "top_print", "cradle5_print", "lid5_print", "cradle10_print", "lid10_print", "shield_print", "plungers_print"]
BATTERY_OPT = "5000"; // ["5000", "10000"]
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

// --- Ruthex Heat-Set Inserts ---
RUTHEX_M2_OD = 3.2;  RUTHEX_M2_DEPTH = 5.0;  RUTHEX_M2_BOSS = 6.2;
RUTHEX_M3_OD = 4.0;  RUTHEX_M3_DEPTH = 6.7;  RUTHEX_M3_BOSS = 7.8;
RUTHEX_Q20_OD = 8.0; RUTHEX_Q20_DEPTH = 13.7;

// --- Waveshare 3.5B (Portrait, USB-C Down) ---
WS_GLASS_W = 61.00;
WS_GLASS_H = 92.44;
WS_GLASS_R = 3.00;
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

FLANGE_Y0 = 16.0;
FLANGE_Y1 = 96.0;
PLUNGER_BLK_X0 = 46.60;
PLUNGER_BLK_X1 = 57.00;
PLUNGER_BLK_Y0 = 22.0;
PLUNGER_BLK_Y1 = 50.0;
PLUNGER_BLK_Z_TOP = -4.2;

// --- RTC Coin Cell (Underneath Waveshare) ---
RTC_CX = 18.50;
RTC_CY = 46.00;

// --- Power Board (20 x 80 x 20.57H, Back Plate mounted) ---
PB_W = 20.0; PB_H = 80.0;
PB_HOLE_DX = 15.0; PB_HOLE_DY = 75.0;
PB_CX = -39.00; PB_CY = 91.00;
PB_STANDOFF_H = 3.0;

// --- Speaker (28 x 30 x 10H, Flat on Back Plate Pedestal) ---
SPK_W = 28.0; SPK_H = 28.0; SPK_Z_THK = 10.0;
SPK_CX = -33.20; SPK_CY = 21.00;
SPK_PEDESTAL_H = 11.5;

// --- Holybro UM982 (34.8 x 53.2 x 14.4H) ---
UM_W = 34.80; UM_BODY_H = 53.20; UM_Z_THK = 14.40;
UM_CX = 31.60;
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

// --- E220 LoRa (40 x 60 x 16H) ---
E220_W = 40.0; E220_H = 60.0;
E220_HOLE_DX = 35.0; E220_HOLE_DY = 55.0;
E220_CX = -7.20; E220_CY = 134.00;
E220_STANDOFF_H = 4.0;
E220_SMA_X = -7.20; E220_SMA_Z = UM_SMA_Z; E220_SMA_DIA = 6.6; E220_SMA_WALL = 2.6;

// --- GX-12 Compass Bulkhead ---
GX12_X = -39.50; GX12_Z = UM_SMA_Z; GX12_HOLE_DIA = 12.2; GX12_NUT_RECESS_DIA = 17.5; GX12_WALL = 2.6;

// --- Closure (6 Symmetric Front M3) & Rear Pod (4 M3) ---
COVER_M3_POINTS = [
    [-54.2,   6.8], [ 54.2,   6.8],
    [-54.1, 108.0], [ 54.1, 108.0],
    [-54.4, 173.4], [ 54.4, 173.4]
];
POD_CX = 2.0; POD_CY = 92.0;
POD_M3_DX = 80.0; POD_M3_DY = 100.0;
POD_M3_POINTS = [
    [POD_CX - POD_M3_DX/2, POD_CY - POD_M3_DY/2],
    [POD_CX + POD_M3_DX/2, POD_CY - POD_M3_DY/2],
    [POD_CX - POD_M3_DX/2, POD_CY + POD_M3_DY/2],
    [POD_CX + POD_M3_DX/2, POD_CY + POD_M3_DY/2]
];
BAT_SLOT_CX = -22.5; BAT_SLOT_CY = 54.5; BAT_SLOT_W = 9.0; BAT_SLOT_H = 14.0;

POD_OUTER_W = 92.0; POD_OUTER_R = 5.5;
POD_FLOOR_THK = 3.0; POD_SPINE_W = 24.0; POD_SPINE_H = 16.2; POLE_Q20_DY = 38.1;

BAT5_BAY_W = 56.5;  BAT5_BAY_H = 77.0;  BAT5_BAY_D = 11.2;  BAT5_POD_H = 112.0;
BAT10_BAY_W = 64.5; BAT10_BAY_H = 124.0; BAT10_BAY_D = 12.8; BAT10_POD_H = 132.0;

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

module teardrop_y_bore(cx, y_start, y_len, zc, dia, peak_plus_z=true) {
    r = dia / 2;
    s = peak_plus_z ? 1 : -1;
    translate([cx, y_start, zc])
        rotate([-90, 0, 0])
            linear_extrude(height=y_len)
                polygon(concat(
                    [for (a = [0 : 10 : 350]) [r * cos(a), r * sin(a)]],
                    [[r/sqrt(2), -s*r/sqrt(2)], [0, -s*r*sqrt(2)], [-r/sqrt(2), -s*r/sqrt(2)]]
                ));
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
    difference() {
        union() {
            // Base plate
            rrect(CASE_W, CASE_H, WALL_BACK, CORNER_R, 0, CASE_H/2, Z_PLATE_OUTER);
            // Integral Top L-Wall Bulkhead
            intersection() {
                rrect(CASE_W, CASE_H, 25.35, CORNER_R, 0, CASE_H/2, Z_PLATE_OUTER);
                union() {
                    translate([0, CASE_H - WALL_TOP/2, Z_PLATE_OUTER + 25.35/2])
                        cube([CASE_W + 2, WALL_TOP, 25.35], center=true);
                    for (sg = [-1, 1])
                        translate([sg * (CASE_W/2 - 4.7), 173.0, Z_PLATE_OUTER + 25.35/2])
                            cube([9.4, 8.2, 25.35], center=true);
                }
            }
            // Right Button-Plunger & MicroSD Flange
            translate([CASE_W/2 - WALL_SIDE/2, 0.5*(FLANGE_Y0+FLANGE_Y1), Z_PLATE_OUTER + 25.35/2])
                cube([WALL_SIDE, FLANGE_Y1 - FLANGE_Y0, 25.35], center=true);
            translate([0.5*(PLUNGER_BLK_X0+PLUNGER_BLK_X1), 0.5*(PLUNGER_BLK_Y0+PLUNGER_BLK_Y1),
                       0.5*(Z_PLATE_INNER + PLUNGER_BLK_Z_TOP)])
                cube([PLUNGER_BLK_X1 - PLUNGER_BLK_X0 + 0.5, PLUNGER_BLK_Y1 - PLUNGER_BLK_Y0,
                      PLUNGER_BLK_Z_TOP - Z_PLATE_INNER + 0.5], center=true);
            // Perimeter registration tabs
            for (t = [[-56.0, 46.0, 1.6, 12.0], [-56.0, 154.0, 1.6, 12.0],
                      [-10.0,  4.0, 12.0, 1.6], [ 42.0,   4.0, 10.0, 1.6]])
                translate([t[0], t[1], Z_PLATE_INNER + 1.8])
                    cube([t[2], t[3], 4.2], center=true);
            // Lower/Mid Cover M3 bosses
            for (p = COVER_M3_POINTS)
                if (p[1] < 165.0)
                    translate([p[0], p[1], Z_PLATE_OUTER + 0.5])
                        cylinder(d=RUTHEX_M3_BOSS, h=-13.2 - Z_PLATE_OUTER - 0.5);
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
            // Speaker pedestal block
            translate([SPK_CX, SPK_CY, Z_PLATE_INNER + (SPK_PEDESTAL_H + 3.5)/2 - 0.25])
                cube([SPK_W + 3.0, SPK_H + 3.0, SPK_PEDESTAL_H + 4.0], center=true);
        }
        // Cut RTC cradle ID + opening
        translate([RTC_CX, RTC_CY + 0.12, Z_PLATE_INNER - 0.6]) cylinder(d=22.5, h=4.8);
        translate([RTC_CX, RTC_CY - 10.2, Z_PLATE_INNER + 1.8]) cube([22.0, 9.6, 4.8], center=true);
        // Cut Speaker seat & hollow core
        translate([SPK_CX, SPK_CY, Z_PLATE_INNER + SPK_PEDESTAL_H + 2.5])
            cube([SPK_W + 0.4, SPK_H + 0.4, 5.0], center=true);
        translate([SPK_CX, SPK_CY + SPK_H/2 + 1.0, Z_PLATE_INNER + SPK_PEDESTAL_H + 2.5])
            cube([9.0, 4.0, 5.0], center=true);
        translate([SPK_CX, SPK_CY, Z_PLATE_INNER + (SPK_PEDESTAL_H - 2.0)/2])
            cube([18.0, 18.0, SPK_PEDESTAL_H - 2.0], center=true);
        // Top corner sockets & M3 bores
        for (p = COVER_M3_POINTS) {
            if (p[1] > 165.0) {
                translate([p[0], p[1], -13.2]) cylinder(d=8.1, h=11.5);
                translate([p[0], p[1], -13.2 - RUTHEX_M3_DEPTH - 0.3]) cylinder(d=RUTHEX_M3_OD, h=RUTHEX_M3_DEPTH + 0.6);
            } else {
                translate([p[0], p[1], -13.2 - RUTHEX_M3_DEPTH - 0.3]) cylinder(d=RUTHEX_M3_OD, h=RUTHEX_M3_DEPTH + 0.6);
            }
        }
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
        for (dx = UM_M2_X_OFFS, dy = UM_M2_Y_OFFS)
            csk_through(UM_LEFT_X + dx, UM_BOT_Y + dy, Z_PLATE_OUTER, UM_BACK_Z - Z_PLATE_OUTER, HOLE_CLEAR_M2, 4.6, true);
        // Battery JST pass-through slot
        rrect(BAT_SLOT_W, BAT_SLOT_H, WALL_BACK + 1.0, 4.4, BAT_SLOT_CX, BAT_SLOT_CY, Z_PLATE_OUTER - 0.5);
        // UM982 L-wall recess & crown bores
        translate([UM_CX, 177.25, Z_PLATE_INNER + 12.5]) cube([UM_W + 0.6, 1.50, 25.0], center=true);
        teardrop_y_bore(GX12_X, 176.5, 4.5, GX12_Z, GX12_HOLE_DIA, true);
        teardrop_y_bore(GX12_X, 174.0, 180.0 - GX12_WALL - 174.0, GX12_Z, GX12_NUT_RECESS_DIA, true);
        teardrop_y_bore(E220_SMA_X, 176.5, 4.5, E220_SMA_Z, E220_SMA_DIA, true);
        teardrop_y_bore(E220_SMA_X, 174.0, 180.0 - E220_SMA_WALL - 174.0, E220_SMA_Z, 11.5, true);
        teardrop_y_bore(UM_SMA_SLAVE_X, 175.0, 6.0, UM_SMA_Z, UM_SMA_ACCESS_DIA, true);
        teardrop_y_bore(UM_SMA_MASTER_X, 175.0, 6.0, UM_SMA_Z, UM_SMA_ACCESS_DIA, true);
        // Plunger bores & pockets
        for (by = WS_BTN_Y_LIST) {
            arch_x_cut(50.8, 10.0, by, 4.6, WS_BTN_Z_CTR - 2.1, WS_BTN_Z_CTR + 2.1, 1.5, true);
            arch_x_cut(46.2, 2.6,  by, 3.5, WS_BTN_Z_CTR - 1.7, WS_BTN_Z_CTR + 1.7, 1.1, true);
            translate([49.7, by, WS_BTN_Z_CTR + 1.7]) cube([3.0, 6.8, 10.0], center=true);
        }
        // MicroSD cutout + scoop
        arch_x_cut(56.2, 4.5, 0.5*(WS_SD_Y0+WS_SD_Y1), WS_SD_Y1 - WS_SD_Y0,
                   WS_SD_Z_CTR - 2.6, WS_SD_Z_CTR + 2.6, 1.6, true);
        arch_x_cut(58.8, 2.0, 0.5*(WS_SD_Y0+WS_SD_Y1), WS_SD_Y1 - WS_SD_Y0 + 3.0,
                   WS_SD_Z_CTR - 3.5, WS_SD_Z_CTR + 3.5, 2.0, true);
    }
}

module top_cover() {
    cover_depth = abs(Z_PLATE_INNER) - 0.15;
    z_bot = -cover_depth;
    u_cx = 0.5 * (SHIELD_U_X_LEFT + SHIELD_U_X_RIGHT);
    u_w = SHIELD_U_X_RIGHT - SHIELD_U_X_LEFT;
    u_cy = 0.5 * (SHIELD_U_Y_BOT + SHIELD_U_Y_TOP);
    u_h = SHIELD_U_Y_TOP - SHIELD_U_Y_BOT;

    difference() {
        union() {
            rrect(CASE_W, CASE_H, cover_depth, CORNER_R, 0, CASE_H/2, z_bot);
            // U-groove reinforcing backing rib
            translate([SHIELD_U_X_LEFT, u_cy, -2.7]) cube([4.0, u_h + 2.5, 1.4], center=true);
            translate([SHIELD_U_X_RIGHT, u_cy, -2.7]) cube([4.0, u_h + 2.5, 1.4], center=true);
            translate([u_cx, SHIELD_U_Y_TOP, -2.7]) cube([u_w + 4.0, 4.0, 1.4], center=true);
            // 6 x Cover M3 Screw Columns down to -13.0
            for (p = COVER_M3_POINTS)
                translate([p[0], p[1], -13.0]) cylinder(d=7.6, h=12.5);
        }
        // Interior cavity
        rrect(CASE_W - 2*WALL_SIDE, CASE_H - WALL_BOT - WALL_TOP, cover_depth - WALL_FRONT + 0.5,
              max(1.5, CORNER_R - WALL_SIDE), 0, CASE_H/2, z_bot - 0.5);
        // Boss relief pockets below -13.0 in walls
        for (p = COVER_M3_POINTS)
            if (p[1] < 165.0)
                translate([p[0], p[1], z_bot - 0.5]) cylinder(d=8.2, h=cover_depth - 13.0 + 0.5);
        // Top L-wall crown & corner notches (straight +Z lift-off!)
        translate([0, 178.7, 0.5*(z_bot + Z_FRONT_INNER) - 0.25])
            cube([CASE_W + 2, 4.0, cover_depth - WALL_FRONT + 0.5], center=true);
        for (sg = [-1, 1])
            translate([sg * (CASE_W/2 - 5.2), 173.0, 0.5*(z_bot + Z_FRONT_INNER) - 0.25])
                cube([11.0, 8.4, cover_depth - WALL_FRONT + 0.5], center=true);
        // Right Flange notch
        translate([CASE_W/2 - 1.2, 0.5*(FLANGE_Y0+FLANGE_Y1), 0.5*(z_bot + Z_FRONT_INNER) - 0.25])
            cube([WALL_SIDE + 2.0, (FLANGE_Y1 - FLANGE_Y0) + 2*NOTCH_CLEAR, cover_depth - WALL_FRONT + 0.5], center=true);
        // Front CSK M3 thru holes
        for (p = COVER_M3_POINTS)
            csk_through(p[0], p[1], -13.0, 13.0, HOLE_CLEAR_M3, 6.2, false);
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
        // Speaker acoustic grille
        for (dy = [-8, -4, 0, 4, 8])
            rrect(18.0, 2.0, 3.5, 0.9, SPK_CX, SPK_CY + dy, -3.0);
        // 4 UM982 LEDs + SWITCH pinhole
        for (lx = UM_LED_X_OFFS)
            translate([UM_LEFT_X + lx, UM_LED_Y, -3.0]) cylinder(d=1.95, h=3.5);
        translate([UM_SW_X, UM_SW_Y, -3.0]) cylinder(d=3.4, h=3.5);
    }
}

module battery_cradle(v="5000") {
    bay_w = (v == "5000") ? BAT5_BAY_W : BAT10_BAY_W;
    bay_h = (v == "5000") ? BAT5_BAY_H : BAT10_BAY_H;
    bay_d = (v == "5000") ? BAT5_BAY_D : BAT10_BAY_D;
    pod_h = (v == "5000") ? BAT5_POD_H : BAT10_POD_H;
    z_back = Z_PLATE_OUTER - bay_d;
    difference() {
        rrect(POD_OUTER_W, pod_h, bay_d, POD_OUTER_R, POD_CX, POD_CY, z_back);
        rrect(bay_w, bay_h, bay_d + 1.0, 2.5, POD_CX, POD_CY + 1.0, z_back - 0.5);
        for (p = POD_M3_POINTS)
            translate([p[0], p[1], z_back - 1.5]) cylinder(d=HOLE_CLEAR_M3, h=bay_d + 3.0);
    }
}

module pole_lid(v="5000") {
    bay_d = (v == "5000") ? BAT5_BAY_D : BAT10_BAY_D;
    pod_h = (v == "5000") ? BAT5_POD_H : BAT10_POD_H;
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
        translate([45.75, by, WS_BTN_Z_CTR]) rotate([0, 90, 0]) cylinder(d=2.8, h=2.85);
        translate([48.55, by, WS_BTN_Z_CTR]) rotate([0, 90, 0])
            hull() {
                translate([0, 0, 0]) cube([5.2, 6.1, 0.2], center=true);
                translate([0, 0, 1.0]) cube([5.2, 6.1, 0.2], center=true);
                translate([0, 0, 2.25]) cube([3.6, 4.0, 0.2], center=true);
            }
        translate([50.75, 0, 0])
            rotate([90, 0, 90])
                linear_extrude(height=10.45)
                    polygon([
                        [by - 2.0, WS_BTN_Z_CTR - 1.8], [by + 2.0, WS_BTN_Z_CTR - 1.8],
                        [by + 2.0, WS_BTN_Z_CTR + 1.8], [by, WS_BTN_Z_CTR + 3.0],
                        [by - 2.0, WS_BTN_Z_CTR + 1.8]
                    ]);
    }
}

// =============================================================================
// Selection & Print Orientations
// =============================================================================
if (PART == "back") back_chassis_frame();
else if (PART == "top") top_cover();
else if (PART == "cradle5") battery_cradle("5000");
else if (PART == "lid5") pole_lid("5000");
else if (PART == "cradle10") battery_cradle("10000");
else if (PART == "lid10") pole_lid("10000");
else if (PART == "shield") slim_sun_shield();
else if (PART == "plungers") button_plungers();
else if (PART == "back_print") translate([0, 0, -Z_PLATE_OUTER]) back_chassis_frame();
else if (PART == "top_print") rotate([180, 0, 0]) top_cover();
else if (PART == "cradle5_print") rotate([180, 0, 0]) translate([0, 0, -Z_PLATE_OUTER]) battery_cradle("5000");
else if (PART == "lid5_print") rotate([180, 0, 0]) pole_lid("5000");
else if (PART == "cradle10_print") rotate([180, 0, 0]) translate([0, 0, -Z_PLATE_OUTER]) battery_cradle("10000");
else if (PART == "lid10_print") rotate([180, 0, 0]) pole_lid("10000");
else if (PART == "shield_print") translate([0, 0, SHIELD_GROOVE_D - 0.10]) slim_sun_shield();
else if (PART == "plungers_print") rotate([0, -90, 0]) translate([-61.2, 0, 0]) button_plungers();
else {
    color("SlateGray") translate([0, 0, EXPLODE_MM]) top_cover();
    color("SteelBlue") back_chassis_frame();
    color("DarkSlateGray") translate([0, 0, -EXPLODE_MM]) battery_cradle(BATTERY_OPT);
    color("DimGray") translate([0, 0, -2*EXPLODE_MM]) pole_lid(BATTERY_OPT);
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
