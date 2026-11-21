"""Shared parametric mechanical dimensions for the RTK Rover v2.2.0 enclosure.

Architecture (v2.2.0 - 37.5 deg Laser-Mount Tilt Wedge + Recessed UM982 CSK):
- v2.2.0: Added support-free Pole_Tilt_Wedge_37_5 adapting the twin 1/4"-20
  Pole_Lid spine back 37.5 deg from vertical onto a 60 x 130 x 10 mm min
  sliding laser-mount pole clamp (65..115 mm slide, 92 mm rack tower, 8 mm
  exposed 1/4"-20 stud) with 6.0 mm anti-rotation side skirts and bottom
  USB-C overhang access.
- v2.1.6: Added UM_CSK_RECESS = 1.60 mm rear counterbore at the 4 Holybro UM982 M2
  bores (9.20 mm effective span -> 2.80 mm engagement, 1.20 mm margin for M2x12 CSK).
  1. Back_Chassis_Frame: Flat back plate carrying ALL electronics standoffs/cradles
     (Waveshare 3.5B, underneath RTC Coin Cell holder at Y = 64.0, Holybro UM982,
     E220 LoRa, Power Board w/ 5.0 mm M2 Ruthex blind pockets, open-center Speaker
     pocket frame + ridge), full-width R6.0 Top L-Wall bulkhead w/ 50/50 split
     corner piers at Y = 173.2..180.0 (GX-12 centered, E220 SMA w/ downward D-flat,
     UM982 SMAs), Right Button-Plunger Flange + low rail, continuous Left/Bottom/Right
     4.5 mm inner cover-capture rim, 6 open-top full-height M3 cover bosses
     (Z = -2.65 mm), and relocated under-Waveshare BAT_SLOT (25.0, 35.5).
  2. Top_Cover: Clean lift-off shell with ZERO connectors/modules mounted on it;
     flush glass opening, USB-C tunnel, MicroSD slot + finger scoop, speaker
     acoustic grille slots, 4 x UM982 status LED light pipes + SWITCH pinhole,
     Option H1 U-groove, downward inverted-U comb capturing button plungers, and
     matching lower-half top-corner wrap (Y <= 173.0) for 6 symmetric M3 x 8/10 CSK.
  3. Standardized External Option B2 Pod (10000 mAh Footprint + Removable 5000 mAh Insert):
     Unified skeletonized Battery_Cradle (with integral Bottom-Left Capture Corner
     opposite the shared Bottom-Right cable exit) + solid Pole_Lid (twin 1/4"-20
     Ruthex inserts) bolting onto the 4 x M3 grid & aligned JST slot, plus removable
     skeletonized L-truss Battery_Adapter_5000 dropping cleanly inside the bay.
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
__version__ = "v2.2.0"

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

# Inner Perimeter Capture Rim on Back_Chassis_Frame (captures Top_Cover lip on Left, Bottom, Right)
RIM_CLEAR = FIT_CLEAR   # 0.25 mm PETG slip inside Top_Cover inner walls (X=+/-57.0, Y=3.0)
RIM_THK = 1.60          # 4 solid perimeters @ 0.4 mm nozzle (X=[+/-55.15, +/-56.75], Y=[3.25, 4.85])
RIM_H = 4.50            # Z = -25.0..-20.5 mm (4.35 mm engagement overlap inside Top_Cover Z=-24.85)

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

# Integrated Right Button-Plunger Flange (Back_Chassis_Frame) + U-Capture Comb (Top_Cover)
FLANGE_Y0 = 20.0                        # Clears Bottom-Right M3 boss (54.2, 6.8) with 2.0 mm below plunger block
FLANGE_Y1 = 52.0                        # Button block flange only (MicroSD slot moved to Top_Cover right wall)
PLUNGER_BLK_X0 = 56.00                  # Open to -X for tool-free plunger slide-in; 4.00 mm bearing wall (56..60)
PLUNGER_BLK_X1 = 57.00
PLUNGER_BLK_Y0 = 22.0
PLUNGER_BLK_Y1 = 50.0
PLUNGER_BLK_Z_TOP = -4.2                # Clears Glass (-1.10) & Top_Cover front (-2.4) safely
PLUNGER_RAIL_X0 = 49.90                 # Low support rail under collar/nose (clears Top_Cover U-comb -11.50 by 0.20)
PLUNGER_RAIL_Z_TOP = -11.70
PLUNGER_U_X0 = 49.90                    # Top_Cover downward inverted-U capture wall (clears glass opening 49.25 by 0.65)
PLUNGER_U_X1 = 51.60                    # Allows 1.55 mm travel (0.30 mm rest gap + 1.25 mm active stroke) to collar 53.15
PLUNGER_U_Z_BOT = -11.50
PLUNGER_NOSE_DIA = 3.20
PLUNGER_U_SLOT_W = 3.70                 # 0.25 mm radial slip clearance around Dia 3.20 mm nose

# ==============================================================================
# 2. RTC Coin Cell + Holder (Underneath Waveshare! From REF-coin-cell*.stp)
# ==============================================================================
RTC_CX = 18.50              # Centered cleanly under Waveshare PCB & between standoffs
RTC_CY = 64.00              # Wires exit -Y (toward Y~46.4, Z=-25..-21); clears relocated BAT_SLOT Y_max=40.0 by 6.4 mm & top WS standoffs by 8.6 mm

# ==============================================================================
# 3. Power Board (from REF-power-module.stp: 20.0 x 80.0 x 20.57H mm)
# ==============================================================================
PB_W = 20.0
PB_H = 80.0
PB_HOLE_DX = 15.0
PB_HOLE_DY = 75.0
PB_CX = -39.00              # X: [-49.00, -29.00]; 1.5 mm clear of Mid-Left M3 (-50.5) & 21.73 mm clear of right-facing Header (-7.27)
PB_CY = 91.00               # Y: [51.00, 131.00]; 5.1 mm above Bot-Left Pod M3 (45.9), 7.1 mm below Top-Left Pod M3 (138.1)
PB_STANDOFF_H = 2.55        # +0.55 mm clear of -2.00 mm trace protrusion, full 5.00 mm Ruthex M2 pocket; peak -3.88 mm (1.48 mm below cover)

# ==============================================================================
# 4. Speaker (from REF-speaker.stp: 28.0 x 30.0 x 10.0H mm, Flat on Back Plate)
# ==============================================================================
# Normalized with +Z cone upward and 2 mm wire bump at +Y: body [-14, +14] x [-14, +14], bump [+14, +16]
SPK_W = 28.0
SPK_H = 28.0
SPK_Z_THK = 10.0
SPK_CX = -33.20             # Body X: [-47.20, -19.20]; 2.8+ mm clear of Bot-Left M3 (-50.3) & 10.13 mm clear of right-facing Header (-7.27)
SPK_CY = 21.00              # Body Y: [7.00, 35.00], wire bump Y=[35.0, 37.0]; clears Bot-Left Pod M3 (38.1) by 1.1 mm
SPK_PEDESTAL_H = 11.5       # Seating ridge at Z = -13.50 -> speaker front cone at Z = -3.50 (1.1 mm behind Top_Cover)
SPK_WALL = 1.6              # 4-perimeter pocket wall (31.6 x 31.6 outer)
SPK_RIDGE_W = 2.0           # Continuous inner seating ledge width (24.4 x 24.4 open acoustic chamber)

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

UM_CX = -9.80               # X: [-27.20, +7.60]; center Faraday barrier (1.80 mm right of PB -29.0, 1.40 mm left of E220 +9.0)
UM_LEFT_X = UM_CX - UM_W / 2.0          # -27.20
UM_TOP_Y = 178.00           # Body Y: [124.80, 178.00]; 2.0 mm local roof web (178.0..180.0); 26.08 mm clear above LCD
UM_BOT_Y = UM_TOP_Y - UM_BODY_H         # +124.80
UM_CY = 0.5 * (UM_BOT_Y + UM_TOP_Y)     # +151.40
UM_BACK_Z = -17.20          # Standoffs = 7.80 mm off Back Plate Z=-25.0 (4-M2 rear CSK screws)
UM_FRONT_Z = UM_BACK_Z + UM_Z_THK       # -2.80 (0.40 mm behind Top_Cover inside Z=-2.40)
UM_SMA_Z = UM_BACK_Z + UM_SMA_Z_FROM_BACK  # -11.25

UM_SMA_SLAVE_X = UM_LEFT_X + UM_SMA_EDGE_X              # -19.95
UM_SMA_MASTER_X = UM_SMA_SLAVE_X + UM_SMA_PITCH         # +0.35
UM_SMA_ACCESS_DIA = 10.5

# UM982 rear 4-M2 hole positions & front LED / SWITCH exact offsets from REF-holybro-um982.stp
UM_M2_X_OFFS = [2.85, 31.95]
UM_M2_Y_OFFS = [5.80, 46.30]
UM_LED_Y = UM_BOT_Y + 30.20             # 155.00
UM_LED_X_LIST = [UM_LEFT_X + dx for dx in (9.90, 14.90, 19.90, 24.90)]  # [-17.30, -12.30, -7.30, -2.30]
UM_SW_X = UM_LEFT_X + 24.90             # -2.30
UM_SW_Y = UM_BOT_Y + 13.31              # 138.11
UM_CSK_RECESS = 1.60                    # Rear counterbore depth (9.20 mm effective span -> 2.80 mm engagement, 1.20 mm margin for M2 x 12 mm CSK)

# ==============================================================================
# 6. E220 LoRa Module (from REF-e220-module.stp: 40.0 x 60.0 x 16.0H mm)
# ==============================================================================
E220_W = 40.0
E220_H = 60.0
E220_HOLE_DX = 35.0
E220_HOLE_DY = 55.0
E220_CX = 29.00             # X: [+9.00, +49.00]; 38.0 mm clear of Power Board (-29.0), 1.4 mm right of UM982 (+7.6), 1.2 mm clear of Mid-Right M3 (+50.2)
E220_CY = 134.00            # Y: [104.00, 164.00]; 5.28 mm above glass, 1.4+ mm clear below LoRa SMA tail (165.4)
E220_STANDOFF_H = 8.50      # Underside trace (-3.0 mm) at Z=-19.50 (+0.50 mm clear above Top-Right POD M3 -20.0); top at -3.50 (1.10 mm below cover)

# ==============================================================================
# 7. Bulkhead Connectors on Back_Chassis_Frame Top L-Wall
# ==============================================================================
TOP_CORNER_SPLIT_Y = 173.2  # 50/50 split plane through top-corner M3 bosses (+-54.2, 173.2); Back wraps Y=173.2..180, Top_Cover wraps Y<=173.0
TOP_CORNER_WRAP_X_IN = 50.6 # Inner X edge of upper corner piers (0.30 mm inside Dia 7.8 boss for clean non-tangent union; 1.60 mm clear of E220, 2.35 mm clear of GX-12 relief)

# LoRa Female SMA Pigtail (from REF-sma-female-lora-connector.stp, downward -Z_asm anti-rotation D-flat)
E220_SMA_X = 29.00          # Centered over E220; 28.65 mm right of UM982 Master (+0.35), 15.85 mm wall to Top-Right pier (+50.6)
E220_SMA_Z = UM_SMA_Z       # -11.25
E220_SMA_DIA = 6.6
E220_SMA_FLAT = 5.98        # Across-flat height from +Z crown to -Z flat (flat at Z=-13.930, 0.125 mm clear of -13.805)
E220_SMA_WALL = 2.6         # Inner flange face at Y = 177.40

# GX-12 5-Pin Compass Bulkhead (from REF-compass-module-input-connector.stp, Z-centered in main body)
GX12_X = -39.50             # Flange [-47.25, -31.75]; clears Top-Left M3 (-54.2) & UM982 (-27.2)
GX12_Z = -14.00             # Exact center of 28.0 mm main chassis Z = [0.00, -28.00]
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
    ( 54.1, 108.0),         # Mid-Right edge (above Right Flange Y=52, above MicroSD Y=85.3, below UM982 Y=124.8)
    (-54.2, 173.2),         # Top-Left corner (50/50 split wrap: Back_Chassis_Frame wraps +Y upper half, Top_Cover wraps -Y lower half)
    ( 54.2, 173.2),         # Top-Right corner (50/50 split wrap: Back_Chassis_Frame wraps +Y upper half, Top_Cover wraps -Y lower half)
]
COVER_BOSS_Z_TOP = -2.65    # Full-height boss (0.25 mm clamping gap below Z_FRONT_INNER=-2.40; flush w/ Top L-Wall)

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

# Battery JST-PH Pass-Through Slot in Back Plate (under Waveshare, aligned with bottom-right clear bay pocket)
BAT_SLOT_CX = 25.0          # X: [18.5, 31.5]; under Waveshare, clears R2.5 bay fillet (31.75) & POD M3 (38.10)
BAT_SLOT_CY = 35.5          # Y: [31.0, 40.0]; flush w/ bay bottom (31.0), 8.25 mm open below cell body (39.25)
BAT_SLOT_W = 13.0
BAT_SLOT_H = 9.0

# Standardized Option B2 Pod (10000 mAh Footprint + Capture Corner + Removable 5000 mAh Adapter)
POD_OUTER_W = 92.0          # X: [-44.0, +48.0] (6.0 mm corner margin outside X=-38/+42)
POD_OUTER_R = 5.5
POD_WALL = 2.4
POD_FRAME_WALL = 2.0        # 5 perimeters @ 0.4 mm nozzle for outer perimeter & inner LiPo pocket
POD_WEB_THK = 1.8           # 4-5 solid perimeters for interlocking Warren-truss web struts
POD_M3_SLEEVE_OD = 8.6      # Reinforced M3 corner sleeve around Dia 3.4 mm bolt bore
POD_FLOOR_THK = 3.0         # Lid base floor thickness (solid protection over rear truss)
POD_SPINE_W = 24.0
POD_SPINE_H = 16.2          # >= 13.7 mm 1/4"-20 insert + 2.5 mm solid closed safety floor
POLE_Q20_DY = 38.1          # 1.500" vertical pitch

POD_BAY_W = 64.5            # X: [-30.25, +34.25] (side walls out to -32.65/+36.65 -> clears M3 -38/+42)
POD_BAY_H = 124.0           # Y: [31.0, 155.0] (+8.25 mm bottom wire bend/exit space below cell Y=39.25)
POD_BAY_D = 12.8            # +1.8 mm wire-routing & foam/swelling allowance (Z = -28.0..-40.8)
POD_H = 132.0               # Y: [26.0, 158.0] (10 mm flange ears at Y=42 & 142 hold POD_M3_POINTS)

# Integral Bottom-Left Battery Capture Corner (opposite side from Bottom-Right cable exit)
POD_CAPTURE_X1 = -12.0      # Solid block X=[-30.25, -12.0], Y=[31.0, 38.75] behind Speaker; leaves bottom-right X=[-12.0, +34.25] open for JST leads
POD_CAPTURE_Y1 = 38.75      # Stops both cells cleanly at Y=39.25

# Battery Cell Reference Dimensions (Meshnology 1S LiPo pouches, bottom-right cable exit)
BAT5_CELL_W = 55.0          # Placed X=[-21.50, +33.50], body Y=[39.25, 105.25], wire tab X=[28.50, 30.50] Y=[37.25, 39.25]
BAT5_CELL_H = 68.0
BAT5_CELL_T = 10.0
BAT10_CELL_W = 63.0         # Placed X=[-29.50, +33.50], body Y=[39.25, 154.25]
BAT10_CELL_H = 115.0
BAT10_CELL_T = 11.0

# Removable Skeletonized 5000 mAh Adapter Insert (drops cleanly inside standardized POD_BAY)
ADAPTER_SLIP = 0.25         # Per-side slip clearance inside bay (Y_bot = 38.75 + 0.25 = 39.00)
ADAPTER_WALL = 2.0
ADAPTER_WEB_THK = 1.8
BAT5_POCKET_X0 = -22.25     # -0.75 mm slip left of 5000 mAh left edge (-21.50)
BAT5_POCKET_Y1 = 106.00     # +0.75 mm slip past 5000 mAh top edge (+105.25)

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

# ==============================================================================
# 10. 37.5-deg Laser-Mount Pole Tilt Wedge (Pole_Tilt_Wedge_37_5)
# ==============================================================================
# Local frame (X_w, V_w, U_w) has origin on top flat of laser shelf at the
# 1/4"-20 stud center: +U_w = away from pole, +V_w = up, +X_w = right.
WEDGE_TILT_DEG = 37.5       # Display tipped 37.5 deg back from vertical (52.5 deg screen elevation)
WEDGE_DU_SB = 14.0          # Spine bottom (Y=55.0, Z=-60.20) offset forward (+U_w) of base 1/4"-20 bore
WEDGE_V_SB = 19.0           # Spine bottom elevation (+V_w) above shelf flat (clears Pod & USB-C)
WEDGE_MATE_GAP = 0.10       # Non-coplanar gap behind z_spine_back (-60.10 -> -60.20)

WEDGE_SPINE_WEB = 8.0       # Solid slanted wall under SHCS heads (5/8" -> 7.9 mm, 3/4" -> 11.1 mm eng)
WEDGE_POCKET_D = 3.0        # Spine alignment pocket depth (clears Pod back face Z=-43.9 by 13.2 mm)
WEDGE_POCKET_WALL = 3.0     # Pocket rim wall around 24.5 x 74.5 mm Pole_Lid spine
WEDGE_CORE_W = 32.0         # Central spine column width (X_w=[-16, +16])

WEDGE_SHELF_W = 60.0        # Laser mount flat platform width
WEDGE_SHELF_L = 130.0       # Laser mount flat length from start of rack up-angle
WEDGE_SHELF_THK = 10.0      # Minimum outer shelf thickness (furthest from pole)
WEDGE_RACK_H = 92.0         # Vertical clamp/rack tower height above flat shelf
WEDGE_SLIDE_MIN = 65.0      # Min screw center from start of rack up-angle
WEDGE_SLIDE_MAX = 115.0     # Max screw center from start of rack up-angle (50 mm travel)
WEDGE_RACK_POLE_OFFSET = 55.0  # Rack front face to pole centerline (u_s=65 -> U_pole=120 mm)

WEDGE_SADDLE_CLR = 0.25     # Per-side slip fit over 60.0 mm shelf (60.50 mm inner saddle span)
WEDGE_SKIRT_T = 3.2         # Downward anti-rotation skirt wall thickness (8 perimeters @ 0.4 mm)
WEDGE_SKIRT_DROP = 6.0      # Anti-rotation skirt drop below shelf flat (inside 10.0 mm min shelf)
WEDGE_BASE_THK = 5.5        # Full-width saddle bridge thickness
WEDGE_GUSSET_H = 8.0        # 45-deg side gusset rise into central spine column

WEDGE_U_BACK = -40.0        # Flat rear bed plane for support-free vertical (+U_w) print
WEDGE_U_FRONT = 14.0        # Front toe on top of shelf (u_s <= 129 inside 130 tip)
WEDGE_U_SKIRT_FRONT = 7.0   # Side skirts stop inside rounded outer shelf tip (u_s <= 122)
WEDGE_TOP_BEVEL_DV = 8.5    # Rear top bevel drop in V_w (~42.3 deg self-supporting from U_BACK)

WEDGE_Q20_CLR = 6.80        # 1/4"-20 SHCS shank pass-through bore
WEDGE_Q20_SHCS_CBORE = 11.40  # 1/4"-20 SHCS head (9.53 mm) + 3/16" hex-key access tunnel
WEDGE_WIN_POLY = [(17.5, -34.5), (35.0, -34.5), (42.5, -27.0), (32.5, -16.5)]  # Support-free kite cutout (V_w, U_w)
