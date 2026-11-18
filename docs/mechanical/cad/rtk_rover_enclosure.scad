// =============================================================================
// RTK Rover Option A Enclosure (Wide Field Controller, PETG Support-Free)
// Parametric OpenSCAD 2026.11.18 mirror of rover_specs.py v1.0.0
// Coordinate origin: X = centered [-59..+59], Y = [0..182], Z = 0 front glass
// Units: millimeters (mm)
// =============================================================================

/* [View & Export Selector] */
PART = "all"; // ["all", "front", "sled", "rear", "adapter", "shield", "buttons", "front_print", "sled_print", "rear_print", "adapter_print", "shield_print", "buttons_print"]
SHOW_MODULES = true;
EXPLODE_MM = 0; // [0:1:60]

$fn = 48;

// --- Overall Enclosure ---
CASE_W = 118.0;
CASE_H = 182.0;
CORNER_R = 6.0;
WALL_SIDE = 3.0;
WALL_BOT = 3.0;
WALL_TOP = 3.2;
WALL_FRONT = 2.4;
WALL_REAR = 3.0;

Z_FRONT_OUTER = 0.0;
Z_FRONT_INNER = -WALL_FRONT;     // -2.4
Z_SLED_FRONT = -22.0;
Z_SLED_THK = 2.5;
Z_SLED_BACK = Z_SLED_FRONT - Z_SLED_THK; // -24.5
Z_PARTING = -24.5;
BAT_BAY_Z = 11.8;
Z_REAR_INNER = Z_SLED_BACK - BAT_BAY_Z;  // -36.3
Z_REAR_OUTER = Z_REAR_INNER - WALL_REAR; // -39.3

FIT_CLEAR = 0.25;
HOLE_CLEAR_M2 = 2.30;
HOLE_CLEAR_M3 = 3.40;

// --- Ruthex Inserts ---
RUTHEX_M2_OD = 3.2;
RUTHEX_M2_DEPTH = 5.0;
RUTHEX_M2_BOSS = 6.2;

RUTHEX_M3_OD = 4.0;
RUTHEX_M3_DEPTH = 6.7;
RUTHEX_M3_BOSS = 7.6;

RUTHEX_Q20_OD = 8.0;
RUTHEX_Q20_DEPTH = 13.7;

// --- Waveshare 3.5B (Portrait, USB-C Down) ---
WS_GLASS_W = 61.00;
WS_GLASS_H = 92.44;
WS_GLASS_R = 6.00;
WS_STANDOFF_DX = 48.50;
WS_STANDOFF_DY = 72.00;
WS_STANDOFF_Z_END = -11.50;
WS_CX = 19.0;
WS_CY = 52.0;

WS_USBC_W = 13.5;
WS_USBC_Z_CTR = -8.72;
WS_USBC_H_OPEN = 8.4;

WS_BTN_X_EDGE = WS_CX + 26.85;
WS_BTN_Y_LIST = [WS_CY - 25.00, WS_CY - 16.50, WS_CY - 8.00];
WS_BTN_Z_CTR = -8.20;

WS_SD_Y0 = WS_CY + 16.60;
WS_SD_Y1 = WS_CY + 32.70;
WS_SD_Z_CTR = -8.15;

WS_DUPONT_TIP_X = WS_CX - 30.50 - 11.60; // -23.10

// --- Power Board (20 x 80 x 22H) ---
PB_W = 20.0; PB_H = 80.0; PB_Z_THK = 22.0;
PB_CX = -42.5; PB_CY = 89.0;
PB_HOLE_DX = 15.0; PB_HOLE_DY = 75.0;
PB_PCB_FRONT_Z = -6.0;

// --- Speaker (28 x 28 x 11H) ---
SPK_W = 28.0; SPK_H = 28.0; SPK_Z_THK = 11.0;
SPK_CX = -37.0; SPK_CY = 24.0;

// --- RTC Coin Battery (21 mm dia x 4.0 mm) ---
RTC_BAT_DIA = 21.4; RTC_BAT_THK = 4.2;
RTC_BAT_CX = -20.0; RTC_BAT_CY = 20.0;

// --- Holybro UM982 (34.8 x 53.2 x 14.4) ---
UM_W = 34.8; UM_H = 53.2; UM_Z_THK = 14.4;
UM_RIGHT_X = 48.0;
UM_LEFT_X = UM_RIGHT_X - UM_W;
UM_CX = 0.5 * (UM_LEFT_X + UM_RIGHT_X);
UM_TOP_Y = 180.0;
UM_BOT_Y = UM_TOP_Y - UM_H;
UM_CY = 0.5 * (UM_BOT_Y + UM_TOP_Y);
UM_FRONT_Z = -2.8;
UM_BACK_Z = UM_FRONT_Z - UM_Z_THK;
UM_SMA_Z = UM_FRONT_Z - 8.45;
UM_SMA_SLAVE_X = UM_LEFT_X + 7.25;
UM_SMA_MASTER_X = UM_SMA_SLAVE_X + 20.3;
UM_SMA_ACCESS_DIA = 10.5;
UM_LED_X_OFFSETS = [9.9, 14.9, 19.9, 24.9];

// --- E220 (40 x 60 x 15H) ---
E220_W = 40.0; E220_H = 60.0; E220_Z_THK = 15.0;
E220_HOLE_DX = 35.0; E220_HOLE_DY = 55.0;
E220_CX = -9.5; E220_CY = 143.0;
E220_STANDOFF_H = 4.0;
E220_SMA_X = -9.5; E220_SMA_Z = UM_SMA_Z;
E220_SMA_DIA = 6.5; E220_SMA_FLAT = 6.0;

// --- GX-12 (Top-Left) ---
GX12_X = -43.5; GX12_Z = UM_SMA_Z;
GX12_HOLE_DIA = 12.2;
GX12_FLAT_DIST = 11.1;
GX12_NUT_RECESS_DIA = 17.5;

// --- 10,000 mAh Battery ---
BAT_POCKET_W = 64.5; BAT_POCKET_H = 123.5;
BAT_CX = 6.0; BAT_CY = 75.0;

// --- Closure & Pole Adapter ---
COVER_M3_POINTS = [
    [-53.5, 6.8], [53.8, 7.2],
    [-54.2, 43.5], [54.0, 112.0],
    [-54.2, 148.0], [53.5, 174.5]
];
ADAPTER_CX = BAT_CX;
ADAPTER_M3_DX = 80.0; ADAPTER_M3_DY = 58.0; ADAPTER_M3_CY = 83.0;
POLE_Q20_DY = 38.1;
POLE_ADAPTER_W = 94.0; POLE_ADAPTER_H = 74.0;
POLE_ADAPTER_BASE_T = 4.5;
POLE_ADAPTER_SPINE_W = 24.0; POLE_ADAPTER_SPINE_T = 16.2;

SHIELD_MAG_POINTS = [[-16.8, 22.0], [-16.8, 84.0], [54.6, 20.0], [54.6, 92.0]];
SHIELD_PIN_POINTS = [[-16.8, 53.0], [54.6, 56.0]];

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

module csk_through(cx, cy, zbed, thk, hd, headd) {
    ch = max(0.4, (headd - hd) / 2);
    translate([cx, cy, zbed - 0.2]) cylinder(d=hd, h=thk + 0.4);
    translate([cx, cy, zbed - 0.05]) cylinder(d1=headd + 0.1, d2=hd, h=ch + 0.05);
}

module roof_bore(cx, zc, dia, flat_dist=0, teardrop=true) {
    r = dia / 2;
    translate([cx, CASE_H - 4.2, zc])
        rotate([-90, 0, 0])
            linear_extrude(height=5.0)
                difference() {
                    union() {
                        circle(r=r);
                        if (teardrop && flat_dist == 0)
                            polygon([[-r/sqrt(2), r/sqrt(2)], [0, r*sqrt(2)], [r/sqrt(2), r/sqrt(2)]]);
                    }
                    if (flat_dist > 0) {
                        flat_y = flat_dist - r;
                        translate([0, flat_y + dia/2 + 1])
                            square([dia + 2, dia + 2], center=true);
                    }
                }
}

// =============================================================================
// Parts
// =============================================================================
module front_shell() {
    difference() {
        union() {
            rrect(CASE_W, CASE_H, abs(Z_PARTING), CORNER_R, 0, CASE_H/2, Z_PARTING);
            for (p = SHIELD_MAG_POINTS)
                translate([p[0], p[1], -5.0]) cylinder(d=6.2 + 3.6, h=5.0);
            for (p = SHIELD_PIN_POINTS)
                translate([p[0], p[1], -4.5]) cylinder(d=7.6, h=4.5);
        }
        // Main cavity
        rrect(CASE_W - 2*WALL_SIDE, CASE_H - WALL_BOT - WALL_TOP,
              abs(Z_PARTING) - WALL_FRONT + 0.5, max(1.5, CORNER_R - WALL_SIDE),
              0, 0.5*(WALL_BOT + CASE_H - WALL_TOP), Z_PARTING - 0.5);
        // Tongue-and-groove rim cut
        rrect(CASE_W - 2.8, CASE_H - 2.8, 1.8, max(1.5, CORNER_R - 1.4),
              0, CASE_H/2, Z_PARTING - 0.2);
        // Flush glass opening (deep trim so shield pads cannot encroach)
        rrect(WS_GLASS_W + 2*FIT_CLEAR, WS_GLASS_H + 2*FIT_CLEAR, 6.5,
              WS_GLASS_R + FIT_CLEAR, WS_CX, WS_CY, -5.5);
        // Shield magnets & pin dimples
        for (p = SHIELD_MAG_POINTS)
            translate([p[0], p[1], -2.2]) cylinder(d=6.2, h=2.4);
        for (p = SHIELD_PIN_POINTS)
            translate([p[0], p[1], -2.05]) cylinder(d1=0.4, d2=3.8, h=2.1);
        // USB-C tunnel
        translate([WS_CX, -0.5, WS_USBC_Z_CTR])
            rotate([-90, 0, 0])
                linear_extrude(height=WALL_BOT + 1.5)
                    polygon([
                        [-WS_USBC_W/2, -WS_USBC_H_OPEN/2], [WS_USBC_W/2, -WS_USBC_H_OPEN/2],
                        [WS_USBC_W/2, WS_USBC_H_OPEN/2], [0, WS_USBC_H_OPEN/2 + 2.0],
                        [-WS_USBC_W/2, WS_USBC_H_OPEN/2]
                    ]);
        // Right buttons & MicroSD
        for (by = WS_BTN_Y_LIST)
            translate([CASE_W/2 - WALL_SIDE - 0.5, by, WS_BTN_Z_CTR])
                rotate([0, 90, 0])
                    linear_extrude(height=WALL_SIDE + 1.5)
                        polygon([[-2.2, -2.4], [2.2, -2.4], [2.2, 2.4], [3.7, 0], [2.2, -2.4]]);
        translate([CASE_W/2 - WALL_SIDE - 0.5, 0.5*(WS_SD_Y0+WS_SD_Y1), WS_SD_Z_CTR])
            rotate([0, 90, 0])
                linear_extrude(height=WALL_SIDE + 1.5)
                    polygon([
                        [-2.8, -(WS_SD_Y1-WS_SD_Y0+3)/2], [2.8, -(WS_SD_Y1-WS_SD_Y0+3)/2],
                        [4.6, 0], [2.8, (WS_SD_Y1-WS_SD_Y0+3)/2],
                        [-2.8, (WS_SD_Y1-WS_SD_Y0+3)/2]
                    ]);
        // Speaker grille slots
        for (ry = [-8, -4, 0, 4, 8])
            rrect(18.0, 1.8, WALL_FRONT + 1.0, 0.8, SPK_CX, SPK_CY + ry, Z_FRONT_INNER - 0.5);
        // UM982 LEDs & SWITCH
        for (lx = UM_LED_X_OFFSETS)
            translate([UM_LEFT_X + lx, UM_TOP_Y - 23.0, Z_FRONT_INNER - 0.5])
                cylinder(d=1.95, h=WALL_FRONT + 1.0);
        translate([UM_LEFT_X + 24.9, UM_TOP_Y - 40.0, Z_FRONT_INNER - 0.5])
            cylinder(d=3.4, h=WALL_FRONT + 1.0);
        // Top roof localized thinning pockets (leaving 2.0 mm UM982 / 2.6 mm GX-12 web)
        translate([UM_CX, 179.25, 0.5*(Z_PARTING + Z_FRONT_INNER) - 0.25])
            cube([UM_W + 0.8, 1.50, abs(Z_PARTING) - abs(Z_FRONT_INNER) + 0.5], center=true);
        translate([GX12_X, 178.95, 0.5*(Z_PARTING + Z_FRONT_INNER) - 0.25])
            cube([GX12_NUT_RECESS_DIA + 1.0, 0.90, abs(Z_PARTING) - abs(Z_FRONT_INNER) + 0.5], center=true);
        // Roof bulkhead bores
        roof_bore(GX12_X, GX12_Z, GX12_HOLE_DIA, GX12_FLAT_DIST, false);
        roof_bore(E220_SMA_X, E220_SMA_Z, E220_SMA_DIA, E220_SMA_FLAT, false);
        roof_bore(UM_SMA_SLAVE_X, UM_SMA_Z, UM_SMA_ACCESS_DIA, 0, true);
        roof_bore(UM_SMA_MASTER_X, UM_SMA_Z, UM_SMA_ACCESS_DIA, 0, true);
    }
    // Speaker rim
    difference() {
        rrect(SPK_W + 3.6, SPK_H + 3.6, 5.0, 2.5, SPK_CX, SPK_CY, Z_FRONT_INNER - 5.0);
        rrect(SPK_W + 2*FIT_CLEAR, SPK_H + 2*FIT_CLEAR, 5.6, 1.5, SPK_CX, SPK_CY, Z_FRONT_INNER - 5.2);
    }
    // Power Board 4 x M2 Ruthex standoffs
    for (sx = [-1, 1], sy = [-1, 1]) {
        bx = PB_CX + sx * PB_HOLE_DX/2;
        by = PB_CY + sy * PB_HOLE_DY/2;
        difference() {
            translate([bx, by, PB_PCB_FRONT_Z]) cylinder(d=RUTHEX_M2_BOSS, h=abs(PB_PCB_FRONT_Z - Z_FRONT_INNER));
            translate([bx, by, PB_PCB_FRONT_Z - 0.1]) cylinder(d=RUTHEX_M2_OD, h=RUTHEX_M2_DEPTH + 0.5);
        }
    }
    // 6 x M3 Closure Bosses
    for (p = COVER_M3_POINTS) {
        difference() {
            translate([p[0], p[1], Z_PARTING + 1.6]) cylinder(d=RUTHEX_M3_BOSS, h=abs(Z_PARTING - Z_FRONT_INNER) - 1.6);
            translate([p[0], p[1], Z_PARTING + 1.5]) cylinder(d=RUTHEX_M3_OD, h=RUTHEX_M3_DEPTH + 0.5);
        }
    }
}

module mid_sled() {
    difference() {
        union() {
            rrect(CASE_W - 2*WALL_SIDE - 1.2, CASE_H - WALL_BOT - WALL_TOP - 1.2, Z_SLED_THK, 2.5,
                  0, 0.5*(WALL_BOT + CASE_H - WALL_TOP), Z_SLED_BACK);
            for (sx = [-1, 1], sy = [-1, 1])
                translate([WS_CX + sx*WS_STANDOFF_DX/2, WS_CY + sy*WS_STANDOFF_DY/2, Z_SLED_FRONT])
                    cylinder(d=RUTHEX_M2_BOSS, h=WS_STANDOFF_Z_END - Z_SLED_FRONT);
            for (sx = [-1, 1], sy = [0, 1])
                translate([UM_CX + sx*29.1/2, (UM_TOP_Y - 6.9) - sy*40.5, Z_SLED_FRONT])
                    cylinder(d=5.6, h=UM_BACK_Z - Z_SLED_FRONT);
            for (sx = [-1, 1], sy = [-1, 1])
                translate([E220_CX + sx*E220_HOLE_DX/2, E220_CY + sy*E220_HOLE_DY/2, Z_SLED_FRONT])
                    cylinder(d=RUTHEX_M2_BOSS, h=E220_STANDOFF_H);
            // RTC Coin Battery C-pocket
            difference() {
                translate([RTC_BAT_CX, RTC_BAT_CY, Z_SLED_FRONT]) cylinder(d=RTC_BAT_DIA + 3.6, h=RTC_BAT_THK + 0.5);
                translate([RTC_BAT_CX, RTC_BAT_CY, Z_SLED_FRONT - 0.2]) cylinder(d=RTC_BAT_DIA, h=RTC_BAT_THK + 1.0);
                rrect(8.0, 10.0, RTC_BAT_THK + 2.0, 0.5, RTC_BAT_CX + 8.0, RTC_BAT_CY, Z_SLED_FRONT - 0.2);
            }
        }
        // Open Left Power Bay C-cutout (square inner corner -> zero floating fillet sliver)
        rrect(28.0, 90.0, Z_SLED_THK + 1.0, 0.1, -44.5, 93.5, Z_SLED_BACK - 0.5);
        rrect(16.5, 11.0, Z_SLED_THK + 1.0, 0.1, -38.75, 44.5, Z_SLED_BACK - 0.5);
        // DuPont routing trough
        rrect(12.0, 44.0, Z_SLED_THK + 1.0, 3.0, WS_DUPONT_TIP_X - 2.0, WS_CY, Z_SLED_BACK - 0.5);
        // 6 x M3 Boss clearance notches
        for (p = COVER_M3_POINTS)
            translate([p[0], p[1], Z_SLED_BACK - 0.5]) cylinder(d=RUTHEX_M3_BOSS + 0.8, h=Z_SLED_THK + 1.0);
        // M2 through holes + 45 deg CSK
        for (sx = [-1, 1], sy = [-1, 1])
            csk_through(WS_CX + sx*WS_STANDOFF_DX/2, WS_CY + sy*WS_STANDOFF_DY/2,
                        Z_SLED_BACK, Z_SLED_THK + WS_STANDOFF_Z_END - Z_SLED_FRONT, HOLE_CLEAR_M2, 4.2);
        for (sx = [-1, 1], sy = [0, 1])
            csk_through(UM_CX + sx*29.1/2, (UM_TOP_Y - 6.9) - sy*40.5,
                        Z_SLED_BACK, Z_SLED_THK + UM_BACK_Z - Z_SLED_FRONT, HOLE_CLEAR_M2, 4.2);
        for (sx = [-1, 1], sy = [-1, 1])
            translate([E220_CX + sx*E220_HOLE_DX/2, E220_CY + sy*E220_HOLE_DY/2,
                       Z_SLED_FRONT + E220_STANDOFF_H - RUTHEX_M2_DEPTH])
                cylinder(d=RUTHEX_M2_OD, h=RUTHEX_M2_DEPTH + 0.3);
    }
}

module rear_cover() {
    depth = Z_PARTING - Z_REAR_OUTER;
    difference() {
        union() {
            rrect(CASE_W, CASE_H, depth, CORNER_R, 0, CASE_H/2, Z_REAR_OUTER);
        }
        // Main hollow cavity
        rrect(CASE_W - 2*WALL_SIDE, CASE_H - WALL_BOT - WALL_TOP, depth - WALL_REAR + 0.5,
              max(1.5, CORNER_R - WALL_SIDE), 0, 0.5*(WALL_BOT + CASE_H - WALL_TOP), Z_REAR_INNER);
        // Lip trim
        difference() {
            rrect(CASE_W + 2, CASE_H + 2, 1.6, CORNER_R, 0, CASE_H/2, Z_PARTING - 1.4);
            rrect(CASE_W - 3.2, CASE_H - 3.2, 2.2, max(1.5, CORNER_R - 1.6), 0, CASE_H/2, Z_PARTING - 1.6);
        }
        for (p = COVER_M3_POINTS)
            csk_through(p[0], p[1], Z_REAR_OUTER, depth, HOLE_CLEAR_M3, 6.2);
    }
    // Add battery walls & bosses cleanly
    difference() {
        union() {
            rrect(BAT_POCKET_W + 3.2, BAT_POCKET_H + 3.2, BAT_BAY_Z - 1.0, 3.0, BAT_CX, BAT_CY, Z_REAR_INNER - 0.5);
            for (p = COVER_M3_POINTS)
                translate([p[0], p[1], Z_REAR_INNER]) cylinder(d=RUTHEX_M3_BOSS, h=Z_PARTING - Z_REAR_INNER);
            for (sx = [-1, 1], sy = [-1, 1])
                translate([ADAPTER_CX + sx*ADAPTER_M3_DX/2, ADAPTER_M3_CY + sy*ADAPTER_M3_DY/2, Z_REAR_OUTER])
                    cylinder(d=RUTHEX_M3_BOSS, h=8.4);
        }
        rrect(BAT_POCKET_W, BAT_POCKET_H, BAT_BAY_Z + 1.0, 2.0, BAT_CX, BAT_CY, Z_REAR_INNER - 0.2);
        rrect(16.0, 8.0, BAT_BAY_Z + 1.0, 1.0, BAT_CX - 18.0, BAT_CY - BAT_POCKET_H/2, Z_REAR_INNER - 0.2);
        for (p = COVER_M3_POINTS)
            csk_through(p[0], p[1], Z_REAR_OUTER, depth, HOLE_CLEAR_M3, 6.2);
        for (sx = [-1, 1], sy = [-1, 1])
            translate([ADAPTER_CX + sx*ADAPTER_M3_DX/2, ADAPTER_M3_CY + sy*ADAPTER_M3_DY/2, Z_REAR_OUTER - 0.1])
                cylinder(d=RUTHEX_M3_OD, h=RUTHEX_M3_DEPTH + 0.1);
    }
}

module pole_adapter() {
    z_mount = Z_REAR_OUTER;
    difference() {
        union() {
            rrect(POLE_ADAPTER_W, POLE_ADAPTER_H, POLE_ADAPTER_BASE_T, 5.0,
                  ADAPTER_CX, ADAPTER_M3_CY, z_mount - POLE_ADAPTER_BASE_T);
            rrect(POLE_ADAPTER_SPINE_W, POLE_ADAPTER_H - 6.0, POLE_ADAPTER_SPINE_T, 6.0,
                  ADAPTER_CX, ADAPTER_M3_CY, z_mount - POLE_ADAPTER_SPINE_T);
        }
        for (sx = [-1, 1], sy = [-1, 1])
            csk_through(ADAPTER_CX + sx*ADAPTER_M3_DX/2, ADAPTER_M3_CY + sy*ADAPTER_M3_DY/2,
                        z_mount - POLE_ADAPTER_BASE_T, POLE_ADAPTER_BASE_T, HOLE_CLEAR_M3, 6.2);
        for (sy = [-1, 1])
            translate([ADAPTER_CX, ADAPTER_M3_CY + sy*POLE_Q20_DY/2, z_mount - POLE_ADAPTER_SPINE_T - 0.1])
                cylinder(d=RUTHEX_Q20_OD, h=RUTHEX_Q20_DEPTH + 0.1);
    }
}

module sun_shield() {
    fw = WS_GLASS_W + 17.0;
    fh = WS_GLASS_H + 14.0;
    fcx = WS_CX - 1.5;
    fcy = WS_CY + 1.0;
    hood_h = 38.0;
    difference() {
        union() {
            rrect(fw, fh, 3.0, 5.0, fcx, fcy, 0.0);
            rrect(fw, fh, hood_h, 5.0, fcx, fcy, 0.0);
        }
        rrect(WS_GLASS_W + 0.6, WS_GLASS_H + 2.0, hood_h + 2.0, 4.0, WS_CX, WS_CY - 2.0, -0.5);
        translate([fcx - fw/2 - 2, 0, 0])
            rotate([90, 0, 90])
                linear_extrude(height=fw + 4)
                    polygon([[fcy - fh/2 - 2, 4.5], [fcy + 5, 4.5],
                             [fcy + 39, hood_h + 1], [fcy - fh/2 - 2, hood_h + 1]]);
        for (p = SHIELD_MAG_POINTS)
            translate([p[0], p[1], -0.1]) cylinder(d=6.2, h=2.3);
    }
    for (p = SHIELD_PIN_POINTS)
        translate([p[0], p[1], -1.8]) cylinder(d1=0.8, d2=3.4, h=1.8);
}

module button_plungers() {
    for (by = WS_BTN_Y_LIST) {
        translate([WS_BTN_X_EDGE + 0.6, by, WS_BTN_Z_CTR]) rotate([0, 90, 0]) cylinder(d=6.2, h=2.2);
        translate([WS_BTN_X_EDGE + 2.8, by, WS_BTN_Z_CTR]) rotate([0, 90, 0])
            cylinder(d=4.3, h=CASE_W/2 + 0.8 - (WS_BTN_X_EDGE + 2.8));
    }
}

// =============================================================================
// Selection & Print Orientations
// =============================================================================
if (PART == "front") front_shell();
else if (PART == "sled") mid_sled();
else if (PART == "rear") rear_cover();
else if (PART == "adapter") pole_adapter();
else if (PART == "shield") sun_shield();
else if (PART == "buttons") button_plungers();
else if (PART == "front_print") rotate([180, 0, 0]) front_shell();
else if (PART == "sled_print") translate([0, 0, -Z_SLED_BACK]) mid_sled();
else if (PART == "rear_print") translate([0, 0, -Z_REAR_OUTER]) rear_cover();
else if (PART == "adapter_print") rotate([180, 0, 0]) translate([0, 0, -Z_REAR_OUTER]) pole_adapter();
else if (PART == "shield_print") sun_shield();
else if (PART == "buttons_print") rotate([0, -90, 0]) button_plungers();
else {
    color("SlateGray") translate([0, 0, EXPLODE_MM]) front_shell();
    color("SteelBlue") mid_sled();
    color("DarkSlateGray") translate([0, 0, -EXPLODE_MM]) rear_cover();
    color("DimGray") translate([0, 0, -2*EXPLODE_MM]) pole_adapter();
    color("Orange") translate([0, 0, 2*EXPLODE_MM]) sun_shield();
    color("Gold") translate([0, 0, EXPLODE_MM]) button_plungers();
    if (SHOW_MODULES) {
        %rrect(WS_GLASS_W, WS_GLASS_H, 11.5, WS_GLASS_R, WS_CX, WS_CY, -11.5);
        %rrect(PB_W, PB_H, PB_Z_THK, 0.5, PB_CX, PB_CY, PB_PCB_FRONT_Z - PB_Z_THK);
        %rrect(E220_W, E220_H, E220_Z_THK, 0.5, E220_CX, E220_CY, Z_SLED_FRONT + E220_STANDOFF_H);
        %rrect(UM_W, UM_H, UM_Z_THK, 1.0, UM_CX, UM_CY, UM_BACK_Z);
        %rrect(63.0, 115.0, 11.0, 1.5, BAT_CX, BAT_CY - 2.0, Z_REAR_INNER + 0.3);
        %rrect(SPK_W, SPK_H, SPK_Z_THK, 1.0, SPK_CX, SPK_CY, Z_FRONT_INNER - SPK_Z_THK - 0.2);
    }
}
