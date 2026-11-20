"""Shared parametric mechanical dimensions for the RTK Rover v2.0.0 enclosure.

Architecture (v2.0.0 - Open-Frame Back Chassis + Lift-Off Cover + Option B2 Pods):
  1. Back_Chassis_Frame: Flat back plate carrying ALL electronics standoffs/cradles
     (Waveshare 3.5B, underneath RTC Coin Cell holder, Holybro UM982, E220 LoRa,
     Power Board, Speaker pedestal), an integral right-angle Top L-Bulkhead
     (GX-12, E220 SMA, UM982 Slave/Master SMAs), and an integral right-angle
     Right Button-Plunger & MicroSD Flange (captive button plungers).
  2. Top_Cover: Clean lift-off shell with ZERO connectors/modules mounted on it;
     only flush glass opening, USB-C cutout, speaker acoustic grille slots,
     4 x UM982 status LED light pipes + SWITCH pinhole, and Option H1 U-groove.
     Secured by 6 symmetric front-entry M3 CSK screws (4 corners + 2 mid-side).
  3. External Option B2 Battery & Pole Mount Pods (5000 mAh and 10000 mAh):
     2-piece support-free stacked pod (Battery_Cradle + Pole_Lid w/ twin 1/4"-20
     Ruthex inserts) bolting onto the rear via a shared 4 x M3 grid & Bat JST slot.
  4. Slim_Sun_Shield (Option H1): Ultra-thin 1.6 mm PETG tapered hood plugging into
     the recessed U-groove in Top_Cover (zero magnets, zero protruding side bosses).

Coordinate convention (looking at the front touchscreen in portrait, USB-C down):
  X: horizontal (+X = right, -X = left), centered at X = 0.0 on outer case
  Y: vertical   (+Y = top RF crown, -Y = bottom USB-C end), spans [0.0, CASE_H]
  Z: depth      (Z = 0.0 = outer Top_Cover faceplate / Waveshare glass front;
                 -Z goes backward toward Back_Chassis_Frame and rear Battery/Pole Pod)
"""

import os

__filename__ = os.path.basename(__file__)
__version__ = "v2.0.1"

# ==============================================================================
# Overall Main Electronics Body (PETG on Snapmaker 2.0, 100% Support-Free)
# ==============================================================================
CASE_W = 120.0          # X: [-60.0, +60.0]
CASE_H = 180.0          # Y: [0.0, 180.0]
CORNER_R = 6.0

WALL_SIDE = 3.0         # Outer side wall thickness (X_inner = +/-57.0)
WALL_BOT = 3.0          # Bottom wall thickness (Y_inner_bot = 3.0)
WALL_TOP = 3.0          # Top L-wall nominal thickness (Y_inner_top = 177.0)
WALL_FRONT = 2.4        # Top_Cover front faceplate thickness (Z = 0.0..-2.4)
WALL_BACK = 3.0         # Back_Chassis_Frame floor thickness (Z = -25.0..-28.0)

# Z Stack Planes (Z = 0.0 at front screen glass plane)
Z_FRONT_OUTER = 0.0
Z_FRONT_INNER = -WALL_FRONT             # -2.4
Z_PLATE_INNER = -25.0                   # 22.6 mm clear bay (clears 20.57 mm Power Board on 3 mm standoffs)
Z_PLATE_OUTER = Z_PLATE_INNER - WALL_BACK  # -28.0 (flat bed plane for Back_Chassis_Frame)

# Fit Clearances (PETG, 0.4 mm nozzle, 4 perimeters)
FIT_CLEAR = 0.25        # General module XY slip clearance per side
NOTCH_CLEAR = 0.20      # Top_Cover lap clearance around Back_Chassis_Frame L-wall & right flange
HOLE_CLEAR_M2 = 2.30    # M2 pass-through bore
HOLE_CLEAR_M3 = 3.40    # M3 pass-through bore

# Ruthex Heat-Set Insert Specs (user-supplied)
RUTHEX_M2_OD = 3.2
RUTHEX_M2_DEPTH = 5.0
RUTHEX_M2_MIN_WALL = 1.3
RUTHEX_M2_BOSS_OD = 6.2     # >= 3.2 + 2*1.3 = 5.8

RUTHEX_M3_OD = 4.0
RUTHEX_M3_DEPTH = 6.7
RUTHEX_M3_MIN_WALL = 1.6
RUTHEX_M3_BOSS_OD = 7.8     # >= 4.0 + 2*1.6 = 7.2

RUTHEX_Q20_OD = 8.0
RUTHEX_Q20_DEPTH = 13.7
RUTHEX_Q20_MIN_WALL = 3.3
RUTHEX_Q20_BOSS_OD = 15.2   # >= 8.0 + 2*3.3 = 14.6

# ==============================================================================
# 1. Waveshare ESP32-S3-Touch-LCD-3.5B (from REF-Waveshare-ESP32-S3-Touch-LCD-3.5B.stp + header)
# ==============================================================================
# Normalized REF glass front sits at Z = 0.0; datum is center of 61.00 x 92.44 glass.
WS_GLASS_W = 61.00
WS_GLASS_H = 92.44
WS_GLASS_R = 6.00           # Exact Waveshare R6.00 corners matching fixed REF-Waveshare...stp
WS_PCB_W = 54.50
WS_PCB_H = 78.00
WS_STANDOFF_DX = 48.50      # 4 x M2.0 female brass pillars ending at Z = -11.50
WS_STANDOFF_DY = 72.00
WS_STANDOFF_Z_END = -11.50

WS_CX = 18.50               # Glass X: [-12.00, +49.00]
WS_CY = 52.50               # Glass Y: [+6.28, +98.72]

# Measured feature coordinates relative to glass center (0, 0) & front Z = 0.0:
# USB-C (bottom edge)
WS_USBC_W = 13.5
WS_USBC_Z_CTR = -8.72
WS_USBC_H_OPEN = 8.4

# Tactile buttons PWR, RESET, BOOT (Solid[0,1,2] exact centroids relative to glass)
# relY = [-25.00, -16.50, -8.00], Z_rel = -8.62, outer contact dome X_rel = +26.85
WS_BTN_X_DOME = WS_CX + 26.85           # +45.35
WS_BTN_Y_LIST = [WS_CY - 25.00, WS_CY - 16.50, WS_CY - 8.00]  # [27.50, 36.00, 44.50]
WS_BTN_Z_CTR = -8.62

# MicroSD card slot (Solid[3] metal shroud relY = [+17.80, +31.80], relX_max = +30.35 -> +48.85)
WS_SD_Y0 = WS_CY + 16.80                # 69.30
WS_SD_Y1 = WS_CY + 32.80                # 85.30
WS_SD_Z_CTR = -8.25

# Integrated Right Button-Plunger Flange & Plunger Carrier (Back_Chassis_Frame)
FLANGE_Y0 = 16.0                        # Clears Bottom-Right M3 boss (54.4, 6.6)
FLANGE_Y1 = 96.0                        # Clears Mid-Right M3 boss (54.4, 108.0)
PLUNGER_BLK_X0 = 46.60                  # Clears Waveshare PCB/midframe (+45.75) by 0.85 mm & Pod boss (+45.9)
PLUNGER_BLK_X1 = 57.00
PLUNGER_BLK_Y0 = 22.0
PLUNGER_BLK_Y1 = 50.0
PLUNGER_BLK_Z_TOP = -4.2                # Clears Glass (-1.10) & Top_Cover front (-2.4) safely

# ==============================================================================
# 2. RTC Coin Cell + Holder (Underneath Waveshare! From REF-coin-cell*.stp)
# ==============================================================================
RTC_CX = 18.50              # Centered cleanly under Waveshare PCB & between standoffs
RTC_CY = 46.00              # Wires exit -Y (toward Y~29, Z=-25..-21); clears Header Z=-18.13 by 2.87+ mm

# ==============================================================================
# 3. Power Board (from REF-power-module.stp: 20.0 x 80.0 x 20.57H mm)
# ==============================================================================
PB_W = 20.0
PB_H = 80.0
PB_HOLE_DX = 15.0
PB_HOLE_DY = 75.0
PB_CX = -39.00              # X: [-49.00, -29.00]; 1.5 mm clear of Mid-Left M3 (-50.5) & 12.67 mm clear of Header
PB_CY = 91.00               # Y: [51.00, 131.00]; 5.1 mm above Bot-Left Pod M3 (45.9), 7.1 mm below Top-Left Pod M3 (138.1)
PB_STANDOFF_H = 2.05        # Flush with -2.00 mm underside trace protrusion (+0.05 mm clear); peak -4.38 mm (1.98 mm below Top_Cover)

# ==============================================================================
# 4. Speaker (from REF-speaker.stp: 28.0 x 30.0 x 10.0H mm, Flat on Back Plate)
# ==============================================================================
# Normalized with +Z cone upward and 2 mm wire bump at +Y: body [-14, +14] x [-14, +14], bump [+14, +16]
SPK_W = 28.0
SPK_H = 28.0
SPK_Z_THK = 10.0
SPK_CX = -33.20             # Body X: [-47.20, -19.20]; 2.8+ mm clear of Bot-Left M3 (-50.3) & Header (-16.33)
SPK_CY = 21.00              # Body Y: [7.00, 35.00], wire bump Y=[35.0, 37.0]; clears Bot-Left Pod M3 (38.1) by 1.1 mm
SPK_PEDESTAL_H = 11.5       # Pedestal top at Z = -13.50 -> speaker front cone at Z = -3.50 (1.1 mm behind Top_Cover)

# ==============================================================================
# 5. Holybro UM982 (from REF-holybro-um982.stp: 34.8 x 58.9 x 14.4H mm)
# ==============================================================================
UM_W = 34.80
UM_BODY_H = 53.20
UM_TOTAL_H = 58.90          # Includes 5.7 mm SMA protrusion
UM_Z_THK = 14.40
UM_SMA_PITCH = 20.30
UM_SMA_EDGE_X = 7.25        # Relative to UM left edge
UM_SMA_Z_FROM_BACK = 5.95   # Measured Z=5.95 from back Z=0.0 (= 8.45 from front Z=14.40)

UM_CX = 31.60               # X: [+14.20, +49.00]; clears Top-Right M3 (+50.5) by 1.50 mm
UM_LEFT_X = UM_CX - UM_W / 2.0          # +14.20
UM_TOP_Y = 178.00           # Body Y: [124.80, 178.00]; 2.0 mm local roof web (178.0..180.0); 26.08 mm clear above LCD
UM_BOT_Y = UM_TOP_Y - UM_BODY_H         # +124.80
UM_CY = 0.5 * (UM_BOT_Y + UM_TOP_Y)     # +151.40
UM_BACK_Z = -17.20          # Standoffs = 7.80 mm off Back Plate Z=-25.0 (4-M2 rear CSK screws)
UM_FRONT_Z = UM_BACK_Z + UM_Z_THK       # -2.80 (0.40 mm behind Top_Cover inside Z=-2.40)
UM_SMA_Z = UM_BACK_Z + UM_SMA_Z_FROM_BACK  # -11.25

UM_SMA_SLAVE_X = UM_LEFT_X + UM_SMA_EDGE_X              # +21.45
UM_SMA_MASTER_X = UM_SMA_SLAVE_X + UM_SMA_PITCH         # +41.75
UM_SMA_ACCESS_DIA = 10.5

# UM982 rear 4-M2 hole positions & front LED / SWITCH exact offsets from REF-holybro-um982.stp
UM_M2_X_OFFS = [2.85, 31.95]
UM_M2_Y_OFFS = [5.80, 46.30]
UM_LED_Y = UM_BOT_Y + 30.20             # 155.00
UM_LED_X_LIST = [UM_LEFT_X + dx for dx in (9.90, 14.90, 19.90, 24.90)]  # [24.10, 29.10, 34.10, 39.10]
UM_SW_X = UM_LEFT_X + 24.90             # 39.10
UM_SW_Y = UM_BOT_Y + 13.31              # 138.11

# ==============================================================================
# 6. E220 LoRa Module (from REF-e220-module.stp: 40.0 x 60.0 x 16.0H mm)
# ==============================================================================
E220_W = 40.0
E220_H = 60.0
E220_HOLE_DX = 35.0
E220_HOLE_DY = 55.0
E220_CX = -7.20             # X: [-27.20, +12.80]; 1.8 mm clear of PB (-29.0) & 1.4 mm clear of UM982 (+14.2)
E220_CY = 134.00            # Y: [104.00, 164.00]; 5.28 mm above glass, 1.4+ mm clear below LoRa SMA tail (165.4)
E220_STANDOFF_H = 3.05      # Flush with -3.00 mm underside trace protrusion (+0.05 mm clear); top at -8.95 mm

# ==============================================================================
# 7. Bulkhead Connectors on Back_Chassis_Frame Top L-Wall (Aligned at Z = -11.25)
# ==============================================================================
# LoRa Female SMA Pigtail (from REF-sma-female-lora-connector.stp, pure round bore)
E220_SMA_X = -7.20          # 32.3 mm right of GX-12, 28.65 mm left of UM982 Slave
E220_SMA_Z = UM_SMA_Z       # -11.25
E220_SMA_DIA = 6.6
E220_SMA_WALL = 2.6         # Inner flange face at Y = 177.40

# GX-12 5-Pin Compass Bulkhead (from REF-compass-module-input-connector.stp, pure round bore)
GX12_X = -39.50             # Flange [-47.25, -31.75]; clears Top-Left M3 (-50.5) & E220 (-27.2)
GX12_Z = UM_SMA_Z           # -11.25
GX12_HOLE_DIA = 12.2        # Pure round M12 x 0.75 barrel clearance (no teardrop)
GX12_NUT_RECESS_DIA = 17.5
GX12_WALL = 2.6             # Inner wall Y = 177.40 -> outer Y = 180.00

# ==============================================================================
# 8. Cover Closure (6 x Symmetric Front-Entry M3) & Option B2 Rear Pod Mount (4 x M3)
# ==============================================================================
COVER_M3_POINTS = [
    (-54.2,   6.8),         # Bottom-Left corner (leaves >= 1.75 mm side & 1.13 mm R6 corner wall)
    ( 54.2,   6.8),         # Bottom-Right corner
    (-54.1, 108.0),         # Mid-Left edge (1.85 mm outside wall; 1.2 mm clear of PB X=-49)
    ( 54.1, 108.0),         # Mid-Right edge (above Right Flange Y=96, below UM982 Y=124.8)
    (-54.4, 173.4),         # Top-Left corner (inside Top L-wall corner socket)
    ( 54.4, 173.4),         # Top-Right corner (inside Top L-wall corner socket)
]

# Shared Option B2 External Battery & Pole Pod Anchor Pattern (Ruthex M3 in Back Plate)
POD_CX = 2.0                # Centered over main electronics spine
POD_CY = 92.0
POD_M3_DX = 80.0            # X = -38.0 and +42.0 (80 mm horizontal pitch)
POD_M3_DY = 100.0           # Y =  42.0 and 142.0 (100 mm vertical pitch)
POD_M3_POINTS = [
    (POD_CX - POD_M3_DX / 2.0, POD_CY - POD_M3_DY / 2.0),  # (-38.0,  42.0)
    (POD_CX + POD_M3_DX / 2.0, POD_CY - POD_M3_DY / 2.0),  # (+42.0,  42.0)
    (POD_CX - POD_M3_DX / 2.0, POD_CY + POD_M3_DY / 2.0),  # (-38.0, 142.0)
    (POD_CX + POD_M3_DX / 2.0, POD_CY + POD_M3_DY / 2.0),  # (+42.0, 142.0)
]

# Battery JST-PH Pass-Through Slot in Back Plate (inside both 5000 & 10000 pods, beside PB bottom)
BAT_SLOT_CX = -22.5         # X: [-27.0, -18.0]; 2.0 mm right of PB (-29.0), 1.67 mm left of Header (-16.33)
BAT_SLOT_CY = 54.5          # Y: [47.5, 61.5]; aligned directly beside Power Board battery input
BAT_SLOT_W = 9.0
BAT_SLOT_H = 14.0

# Option B2 Pod Variants (both bolt onto POD_M3_POINTS & align twin 1/4"-20 inserts)
POD_OUTER_W = 92.0          # X: [-44.0, +48.0] (4.0 mm radial boss margin outside X=-38/+42)
POD_OUTER_R = 5.5
POD_WALL = 2.4
POD_FLOOR_THK = 3.0         # Lid base floor thickness
POD_SPINE_W = 24.0
POD_SPINE_H = 16.2          # >= 13.7 mm 1/4"-20 insert + 2.5 mm solid closed safety floor
POLE_Q20_DY = 38.1          # 1.500" vertical pitch

# Variant 1: 5000 mAh LiPo (REF-lipo-battery-3.7V-5000mAh.stp: 55.0 x 68.0 x 10.0 mm)
BAT5_CELL_W = 55.0
BAT5_CELL_H = 68.0
BAT5_CELL_T = 10.0
BAT5_BAY_W = 56.5           # +0.75 mm slip per side
BAT5_BAY_H = 77.0           # Includes +9 mm wire exit zone below cell
BAT5_BAY_D = 11.2           # +1.2 mm foam/swelling allowance (Z = -28.0..-39.2)
BAT5_POD_H = 112.0          # Covers POD_M3_DY=100 +/- 6 mm corners comfortably

# Variant 2: 10000 mAh LiPo (REF-lipo-battery-3.7V-10000mAh.stp: 63.0 x 115.0 x 11.0 mm)
BAT10_CELL_W = 63.0
BAT10_CELL_H = 115.0
BAT10_CELL_T = 11.0
BAT10_BAY_W = 64.5          # X: [-30.25, +34.25] (side walls out to -32.65/+36.65 -> clears M3 -38/+42!)
BAT10_BAY_H = 124.0         # Y: [30.0, 154.0] (+9 mm Kapton/lead bend exit)
BAT10_BAY_D = 12.8          # +1.8 mm wire-routing & foam/swelling allowance (Z = -28.0..-40.8)
BAT10_POD_H = 132.0         # Y: [26.0, 158.0] (10 mm flange ears at Y=42 & 142 hold POD_M3_POINTS)

# ==============================================================================
# 9. Option H1 Slim Slot-In Sun Shield (Zero Magnets, Zero Protruding Bosses)
# ==============================================================================
# Continuous 1.90 mm wide x 1.80 mm deep U-groove in Top_Cover faceplate around LCD
SHIELD_U_X_LEFT = -15.40    # 3.40 mm clear of glass (-12.00)
SHIELD_U_X_RIGHT = 52.20    # 3.20 mm clear of glass (+49.00), 4.8 mm inside right outer wall (+60.0)
SHIELD_U_Y_BOT = 14.00      # Stops clear above Bot M3 (6.6)
SHIELD_U_Y_TOP = 102.00     # 3.28 mm clear above glass (98.72), clears Mid M3 (108.0) & LEDs (155.0)
SHIELD_GROOVE_W = 1.90
SHIELD_GROOVE_D = 1.80
SHIELD_WALL_T = 1.60        # 4 perimeters x 0.4 mm nozzle (nominal 0.15 mm clearance each side in groove)
SHIELD_DEPTH_TOP = 28.0     # Forward projection (+Z) above faceplate at top visor roof
SHIELD_DEPTH_BOT = 12.0     # Tapered side-cheek forward projection at bottom (open bottom for thumb access)
