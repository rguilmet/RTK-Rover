"""Shared parametric mechanical dimensions for the RTK Rover Option A enclosure.

Coordinate convention (looking at the front touchscreen in portrait, USB-C down):
  X: horizontal (+X = right, -X = left), centered at X = 0.0 on the case outer box
  Y: vertical   (+Y = top RF crown, -Y = bottom USB-C end), outer box spans [0, CASE_H]
  Z: depth      (Z = 0.0 = outer Front_Shell flat face / Waveshare glass front plane;
                 -Z goes backward into the enclosure toward Rear_Cover)
"""

import os

__filename__ = os.path.basename(__file__)
__version__ = "v1.0.0"

# ==============================================================================
# Overall Outer Enclosure (Option A: Wide Field Controller, PETG, Zero Supports)
# ==============================================================================
CASE_W = 118.0          # X: [-59.0, +59.0]
CASE_H = 182.0          # Y: [0.0, 182.0]
CORNER_R = 6.0

WALL_SIDE = 3.0         # Outer side wall thickness (X_inner = +/-56.0)
WALL_BOT = 3.0          # Bottom wall thickness (Y_inner_bot = 3.0)
WALL_TOP = 3.2          # Top bulkhead structural wall (Y_inner_top = 178.8)
WALL_FRONT = 2.4        # Front faceplate wall thickness (Z = 0.0 .. -2.4)
WALL_REAR = 3.0         # Rear wall behind battery firewall (Z = -35.8 .. -38.8)

# Z Stack Planes (Z = 0.0 at front screen plane / build plate for Front_Shell)
Z_FRONT_OUTER = 0.0
Z_FRONT_INNER = -WALL_FRONT          # -2.4
Z_SLED_FRONT = -22.0                 # 22.0 mm front bay (clears 18 mm DuPont + 4.0 mm)
Z_SLED_THK = 2.5
Z_SLED_BACK = Z_SLED_FRONT - Z_SLED_THK   # -24.5 (flat bed face for Mid_Frame_Sled)
Z_PARTING = -24.5                    # Front_Shell ends here (flush lip overlap with Rear_Cover)

BAT_BAY_Z = 11.8                     # 11.0 mm LiPo + 0.8 mm foam pad tolerance
Z_REAR_INNER = Z_SLED_BACK - BAT_BAY_Z    # -36.3 (Rear_Cover interior battery floor)
Z_REAR_OUTER = Z_REAR_INNER - WALL_REAR   # -39.3 (flat bed face for Rear_Cover)

# Printer Clearance (PETG, 0.4 mm nozzle, 4 perimeters)
FIT_CLEAR = 0.25        # Per-side XY pocket fit margin
HOLE_CLEAR_M2 = 2.30    # M2 screw pass-through hole
HOLE_CLEAR_M3 = 3.40    # M3 screw pass-through hole

# Ruthex Heat-Set Insert Specs (user-supplied)
RUTHEX_M2_OD = 3.2      # Hole diameter for Ruthex M2
RUTHEX_M2_DEPTH = 5.0
RUTHEX_M2_MIN_WALL = 1.3
RUTHEX_M2_BOSS_OD = 6.2 # >= 3.2 + 2*1.3 = 5.8

RUTHEX_M3_OD = 4.0      # Hole diameter for Ruthex M3
RUTHEX_M3_DEPTH = 6.7
RUTHEX_M3_MIN_WALL = 1.6
RUTHEX_M3_BOSS_OD = 7.6 # >= 4.0 + 2*1.6 = 7.2

RUTHEX_Q20_OD = 8.0     # Hole diameter for Ruthex 1/4"-20
RUTHEX_Q20_DEPTH = 13.7
RUTHEX_Q20_MIN_WALL = 3.3
RUTHEX_Q20_BOSS_OD = 15.2 # >= 8.0 + 2*3.3 = 14.6

# ==============================================================================
# 1. Waveshare ESP32-S3-Touch-LCD-3.5B (from ESP32-S3-Touch-LCD-3_5B.stp)
# ==============================================================================
WS_GLASS_W = 61.00
WS_GLASS_H = 92.44
WS_GLASS_R = 6.00
WS_VA_W = 48.96
WS_VA_H = 73.44
WS_PCB_W = 54.52
WS_PCB_H = 78.02
WS_STANDOFF_DX = 48.50  # 4 x M2.0 female brass pillars
WS_STANDOFF_DY = 72.00
WS_STANDOFF_Z_END = -11.50 # Relative to glass front = 0.0 (-10.40 - 1.10)

# Placement in Enclosure
WS_CX = 19.0            # Glass spans X = [-11.50, +49.50]; leaves 9.5 mm right wall/buttons
WS_CY = 52.0            # Glass spans Y = [5.78, 98.22]; leaves 5.78 mm bottom chin

# Port/Feature offsets relative to (WS_CX, WS_CY), verified in STP:
WS_USBC_W = 13.5        # Tunnel width for standard USB-C cable overmold
WS_USBC_Z_CTR = -8.72   # (-9.65..-5.59) - 1.10 = -10.75..-6.69, centered -8.72
WS_USBC_H_OPEN = 8.4    # Cutout opening height in Z
WS_USBC_Y_END = WS_CY - 40.00  # USB-C front shell socket face at Y = 12.00

WS_BTN_X_EDGE = WS_CX + 26.85  # +45.85 tactile button domes
WS_BTN_Y_LIST = [WS_CY - 25.00, WS_CY - 16.50, WS_CY - 8.00] # PWR, RESET, BOOT (27.0, 35.5, 44.0)
WS_BTN_Z_CTR = -8.20    # (-8.20..-6.00) - 1.10 = -9.30..-7.10

WS_SD_Y0 = WS_CY + 16.60       # 68.60
WS_SD_Y1 = WS_CY + 32.70       # 84.70
WS_SD_Z_CTR = -8.15     # (-8.40..-5.80) - 1.10 = -9.50..-6.90

# Right-Angle 32-Pin Header facing LEFT (-X) into wiring corridor:
# Solder row inner edge Xw=-19.69, glass edge Xw=-30.50.
# Measured 22.4 mm total width from inner row -> ends 11.6 mm past glass left edge.
WS_DUPONT_TIP_X = WS_CX - 30.50 - 11.60  # -23.10
WS_DUPONT_Y0 = WS_CY - 21.0              # 31.0
WS_DUPONT_Y1 = WS_CY + 21.0              # 73.0

# ==============================================================================
# 2. Power Board (20.0 x 80.0 x 22H mm, 4 x M2 @ 2.5 mm corner inset)
# ==============================================================================
PB_W = 20.0
PB_H = 80.0
PB_Z_THK = 22.0
PB_CX = -42.5           # Spans X = [-52.5, -32.5]; 9.4 mm clear gap left of DuPont tips!
PB_CY = 89.0            # Spans Y = [49.0, 129.0]; leaves Y=[38..49] & [129..152] for 4p JSTs
PB_HOLE_DX = 15.0
PB_HOLE_DY = 75.0
PB_PCB_FRONT_Z = -6.0   # Front_Shell inner boss length = 3.6 mm (-2.4 to -6.0), components to -28.0 (clears M3 adapter bosses at -30.4!)

# ==============================================================================
# 3. Speaker (28 x 28 x 11H mm, Lower-Left Front-Firing)
# ==============================================================================
SPK_W = 28.0
SPK_H = 28.0
SPK_Z_THK = 11.0
SPK_CX = -37.0          # Spans X = [-51.0, -23.0]
SPK_CY = 24.0           # Spans Y = [10.0, 38.0]; below Power Board + clears Bot-Left M3 boss

# ==============================================================================
# 4. RTC Button Cell (CR2032/wrapped pack: 21 mm dia x 4.0 mm thick, Lower-Center)
# ==============================================================================
RTC_BAT_DIA = 21.4      # 21.0 mm + 0.4 mm slip fit
RTC_BAT_THK = 4.2       # 4.0 mm + 0.2 mm
RTC_BAT_CX = -20.0      # Sits beside Speaker / directly next to SH1.0 RTC header (Y~19)
RTC_BAT_CY = 20.0
RTC_BAT_Z_BASE = Z_SLED_FRONT + 4.5  # Held in C-pocket on Mid_Frame_Sled (-17.0..-21.5)

# ==============================================================================
# 5. Holybro UM982 (34.8 x 53.2 x 14.4 mm, Label Front, Top-Right)
# ==============================================================================
UM_W = 34.8
UM_H = 53.2
UM_Z_THK = 14.4
UM_SMA_PROTRUDE = 5.7   # 58.9 - 53.2 = 5.7 mm brass threads
UM_SMA_PITCH = 20.3
UM_SMA_EDGE_X = 7.25    # Centered in 34.8 mm width
UM_SMA_Z_FROM_FRONT = 8.45
UM_HOLE_TOP_OFFSET = 6.9
UM_HOLE_DX = 29.1
UM_HOLE_DY = 40.5
UM_LED_TOP_OFFSET = 23.0
UM_LED_X_OFFSETS = [9.9, 14.9, 19.9, 24.9]
UM_SW_TOP_OFFSET = 40.0
UM_SW_X_OFFSET = 24.9
UM_LEFT_LENS_X_OFFSET = 9.9
UM_LEFT_LENS_DIA = 5.2

UM_RIGHT_X = 48.0       # X = [13.2, 48.0]; leaves 1.5 mm clear inside Top-Right M3 boss!
UM_LEFT_X = UM_RIGHT_X - UM_W            # 13.2
UM_CX = 0.5 * (UM_LEFT_X + UM_RIGHT_X)   # 30.6
UM_TOP_Y = 180.0        # Sits inside 2.0 mm locally counterbored top roof web (180.0..182.0)
UM_BOT_Y = UM_TOP_Y - UM_H               # 126.8 (28.6 mm clear above Waveshare Y=98.2 for COM1)
UM_CY = 0.5 * (UM_BOT_Y + UM_TOP_Y)      # 153.4
UM_FRONT_Z = -2.8       # 0.4 mm behind Front_Shell inner ceiling Z=-2.4
UM_BACK_Z = UM_FRONT_Z - UM_Z_THK        # -17.2 (4.3 mm standoff down to sled Z=-21.5)
UM_SMA_Z = UM_FRONT_Z - UM_SMA_Z_FROM_FRONT  # -11.25
UM_SMA_SLAVE_X = UM_LEFT_X + UM_SMA_EDGE_X                 # 20.45
UM_SMA_MASTER_X = UM_SMA_SLAVE_X + UM_SMA_PITCH            # 40.75
UM_SMA_ACCESS_DIA = 10.5 # >= 10 mm requested, clears 5/16" hex corners

# ==============================================================================
# 6. E220 Module (40.0 x 60.0 x 15.0 mm, Top-Center)
# ==============================================================================
E220_W = 40.0
E220_H = 60.0
E220_Z_THK = 15.0
E220_HOLE_DX = 35.0
E220_HOLE_DY = 55.0
E220_CX = -9.5          # X = [-29.5, +10.5]; clears Power Board (-32.5) & UM982 (+13.2)
E220_CY = 143.0         # Y = [113.0, 173.0]; 14.8 mm above Waveshare, 6.5 mm top coax room
E220_STANDOFF_H = 4.0   # Solder clearance above Z_SLED_FRONT (-22.0 -> PCB back at -18.0 -> -3.0 peak)
E220_SMA_X = -9.5       # Bulkhead SMA centered above E220 (29.95 mm from UM982 Slave, 34.0 mm from GX-12)
E220_SMA_Z = UM_SMA_Z   # -11.25 aligned on top crown axis
E220_SMA_DIA = 6.5
E220_SMA_FLAT = 6.0

# ==============================================================================
# 7. GX-12 4-Pin Bulkhead (10DOF Connector, Top-Left)
# ==============================================================================
GX12_X = -43.5          # Centered above Power Board channel, far from E220/GNSS SMAs
GX12_Z = UM_SMA_Z       # -11.25
GX12_HOLE_DIA = 12.2    # M12 x 0.75 barrel clearance
GX12_FLAT_DIST = 11.1   # Top flat across M12 circle -> prevents rotation + bridges support-free
GX12_NUT_RECESS_DIA = 17.5
GX12_ROOF_WEB = 2.6     # Max panel wall <= 3.0 mm for 9.1 mm thread barrel + locknut

# ==============================================================================
# 8. 10,000 mAh LiPo Battery (115 x 63 x 11 mm -> 123.5 x 64.5 x 11.8 mm bay)
# ==============================================================================
BAT_CELL_W = 63.0
BAT_CELL_H = 115.0
BAT_POCKET_W = 64.5
BAT_POCKET_H = 123.5    # Includes ~8.5 mm wire bend exit clearance at bottom Kapton PCB
BAT_POCKET_THK = BAT_BAY_Z
BAT_CX = 6.0            # Shifted right behind Waveshare -> X=[-26.25..+38.25], clears Power Board (-32.5) by 4.65 mm!
BAT_CY = 75.0           # Y = [13.25, 136.75]

# ==============================================================================
# 9. Case Closure & Option 3 Pole Adapter Fastener Grids (Ruthex M3 + 1/4"-20)
# ==============================================================================
# 6 x M3 Enclosure Closure Bosses (clearing Speaker, Power Board, Waveshare, GX-12, UM982, R6 corners)
COVER_M3_POINTS = [
    (-53.5, 6.8),       # Bottom-Left (below Speaker Y=10.0)
    ( 53.8, 7.2),       # Bottom-Right (clears Waveshare R6 glass corner)
    (-54.2, 43.5),      # Mid-Left (clear gap between Speaker Y=38 and Power Board Y=49)
    ( 54.0, 112.0),     # Mid-Right (clear gap between Waveshare Y=98.2 and UM982 Y=126.8)
    (-54.2, 148.0),     # Top-Left (clear gap above Power Board Y=129 and below GX-12 nut)
    ( 53.5, 174.5),     # Top-Right (clears UM_RIGHT_X=48.0 and R6 top-right corner)
]

# Option 3: 4 x M3 Rear Pole Adapter Grid (flanks BAT_CX=6.0 cleanly!)
ADAPTER_CX = BAT_CX     # Centered at X = +6.0 behind battery pocket
ADAPTER_M3_DX = 80.0    # X = -34.0 and +46.0 (2.35 mm clear outside battery wall; above Z=-28 Power Board!)
ADAPTER_M3_DY = 58.0    # Y = 54.0, 112.0 (centered around Y = 83.0)
ADAPTER_M3_CY = 83.0

# 2 x 1/4"-20 Ruthex Inserts inside Pole_Adapter_Plate
POLE_Q20_DY = 38.1      # 1.500" vertical pitch
POLE_ADAPTER_W = 94.0
POLE_ADAPTER_H = 74.0
POLE_ADAPTER_BASE_T = 4.5
POLE_ADAPTER_SPINE_W = 24.0
POLE_ADAPTER_SPINE_T = 16.2 # >= 13.7 mm insert + 2.5 mm solid closed floor

# Sun Shield Detachable Alignment + Magnet Pockets (Zero fragile snap hooks in PETG)
SHIELD_MAG_DIA = 6.2
SHIELD_MAG_DEPTH = 2.2
SHIELD_MAG_POINTS = [
    (-16.8, 22.0),      # Left strip beside LCD (clears -11.75 glass opening)
    (-16.8, 84.0),      # Left strip near top of LCD
    ( 54.6, 20.0),      # Right lower strip (clears +49.75 glass opening)
    ( 54.6, 92.0),      # Right upper strip above MicroSD notch
]
SHIELD_PIN_POINTS = [
    (-16.8, 53.0),      # Left conical registration dimple
    ( 54.6, 56.0),      # Right conical registration dimple (between BOOT button & MicroSD)
]
