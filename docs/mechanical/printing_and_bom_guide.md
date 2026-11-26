# RTK Rover Snapmaker 2.0 PETG Print Guide & Fastener BOM (`CAD v2.2.7`)

Complete 3D-printing profile, slicer overrides, heat-set insert reference, verified fastener stack-up calculations, and assembly order for the **RTK Rover Option A** mechanical enclosure (`CAD v2.2.7`).

* **Enclosure Architecture & CAD Previews:** [`cad/README.md`](cad/README.md)
* **Pre-Oriented Support-Free STL Files (`9` files):** [`cad/stl/`](cad/stl/)
* **Parametric Source Datums:** [`cad/rover_specs.py`](cad/rover_specs.py)
* **Snapmaker Luban `4.15.0` PETG Profiles (A350T, Verified):** [`cad/snapmaker/`](cad/snapmaker/)

All `9` meshes in [`cad/stl/`](cad/stl/) (`*_print.stl`) are already rotated flat onto `Z = 0` and engineered to print **100% support-free** with a standard `0.4 mm` nozzle.

---

## 1. Snapmaker 2.0 PETG Baseline Profile (`0.4 mm` Nozzle)

> **Verified Snapmaker Luban `4.15.0` Profiles (A350T Dual Extruder):** Ready-to-import exported material and quality definitions are saved in [`cad/snapmaker/`](cad/snapmaker/) ([`material.PETG-Blue-Tuned.def.json`](cad/snapmaker/material.PETG-Blue-Tuned.def.json) + [`quality.normal.PETG-Blue-Tuned.def.json`](cad/snapmaker/quality.normal.PETG-Blue-Tuned.def.json)). Tested single-extruder workflow on the **Dual Extrusion Module (Left extruder only)**: `6x6` heated bed leveling (`75 deg C`) -> sensor calibration -> Auto Z-Offset calibration. Field defaults: `0.16 mm` layers (`0.28 mm` layer `0`), `240 deg C` nozzle / `75 deg C` bed, `95%` flow (`120%` layer `0`), `0.5 mm @ 35 mm/s` direct-drive retraction, `0%` fan layer `0` -> `30%` by layer `4`, `4` walls / `6` top & bottom / `30%` gyroid, `25 mm/s` outer wall & layer `0` / `40 mm/s` print, `-0.015 mm` XY offset, supports OFF, and `no_outer_surfaces` combing.

Apply these baseline settings in **Snapmaker Luban**, **PrusaSlicer**, or **OrcaSlicer**:

| Setting | Recommended Value | Rationale (Snapmaker 2.0 + PETG) |
| :--- | :--- | :--- |
| **Nozzle Temp** | `240 deg C` Layer 1 / `235-245 deg C` Print | Ensures strong layer fusion on tall `M3` bosses and the Top L-Wall bulkhead; drop `5 deg C` if stringing appears. |
| **Bed Temp** | `75 deg C` Layer 1 / `70 deg C` Print | Prevents corner warp across the `120 x 180 mm` footprint without softening bottom layers. |
| **Live Z-Offset** | **`+0.04 to +0.06 mm` above PLA** | Avoids PETG over-squish (nozzle buildup, blobs, and sheet tearing); lay beads gently rather than flattening them. |
| **Bed Prep & Cooldown** | **Washable PVA glue stick** on stock matte sheet (or textured PEI) | **Mandatory release barrier:** bare PETG can bond permanently and tear the stock Snapmaker 2.0 sheet. Cool bed **`< 30 deg C`** before part removal. |
| **Part Cooling Fan** | `0%` Layers `1-3`, then `30-40%` (`50%` on bridges/plungers) | High fan weakens PETG Z-adhesion on bosses and bulkheads; moderate airflow keeps arches and plungers sharp. |
| **Print Speeds** | First layer `15-20 mm/s`, Outer wall `30-35 mm/s`, Inner/Infill `45-55 mm/s`, Travel `100-120 mm/s` | Eliminates ringing on Snapmaker linear modules and maintains accurate `0.25 mm` slip seams (`Top_Cover` vs capture rim & `0.20 mm` corner seam). |
| **Retraction (Direct Drive)** | `0.8-1.2 mm` @ `30-35 mm/s` (`Z-hop`: `0.2 mm` optional) | Snapmaker 2.0 uses a direct-drive extruder; retraction `> 2 mm` can pull molten PETG into the heatbreak and jam. |
| **Layer Height & Width** | `0.20 mm` layers, `0.40-0.42 mm` line width (enable Arachne perimeter generator if available) | Matches the `1.60 mm` (`4`-perimeter) capture rim, speaker frame, and sun shield walls cleanly. |
| **Perimeters / Shells** | `4` perimeters (`1.60 mm` min wall), `5` top / `5` bottom layers (`1.00 mm`) | Guarantees solid wall rings around all `M2` / `M3` Ruthex heat-set bosses (`>= 1.3 mm` / `1.6 mm` CAD wall). |
| **Elephant's Foot Comp.** | `0.15-0.20 mm` | Keeps first-layer dimensions true on `Top_Cover`'s `1.90 mm` Sun Shield U-groove, glass bezel, and `M3` CSK bores. |
| **Supports** | **OFF (`None`)** | All horizontal holes (`USB-C`, `MicroSD`, `GX-12` & `E220` D-flats, `3.5mm` `M6` bore + full-height straight jack recess), `CSK` cones, `4`-side slotted speaker L-pillars, the unified hollow plunger housing (side-wall clip slots + centerline-split lower nose U-cradle), `Top_Cover`'s continuous `13.30 mm` front left skirt over the L-tab notch, and `Pole_Tilt_Wedge_37_5`'s open Warren truss, pure round `1/4"-20` bores/wrench holes, and `45-deg` lofted spine pocket are self-supporting. |

### Pre-Print PETG Tips
* **Dry Spool First:** Even sealed PETG absorbs moisture; drying at `60-65 deg C` for `4-6 h` eliminates stringing inside the `Battery_Cradle` Warren truss, `Pole_Tilt_Wedge_37_5` open Warren-truss struts, `1.90 mm` Sun Shield U-groove, and `4` `Dia 1.95 mm` `UM982` LED holes.
* **Clean Brass Nozzle:** Wipe hot residue from the nozzle tip with a brass brush immediately before printing [`back_chassis_print.stl`](cad/stl/back_chassis_print.stl) and [`top_cover_print.stl`](cad/stl/top_cover_print.stl) to avoid dark PETG blobs on outer walls or the front bezel.

---

## 2. Per-Part Slicer Overrides (`9` STL Files)

| STL File | Vol (`cm^3`) | Print Size (`mm`) | Layer / Walls / Infill | Adhesion & Print Notes |
| :--- | :---: | :---: | :--- | :--- |
| [`button_plungers_print.stl`](cad/stl/button_plungers_print.stl) | `0.62` | `5.7 x 23.1 x 15.9` | **`0.12-0.16 mm`**, `4` walls, **`100%` infill** | **Print first (~15 min) as a PETG tuning check.** Slow to `20-25 mm/s`, fan `40-50%`, min layer time `12-15 s`, **`4-5 mm` outer brim** so all `3` `Dia 3.20 / 4.10 mm` stems print cleanly round. |
| [`plunger_capture_clip_print.stl`](cad/stl/plunger_capture_clip_print.stl) | `0.28` | `7.35 x 28.90 x 1.70` | **`0.16 mm`**, `4` walls, **`100%` infill** | Print flat on the bed right beside the plungers (`~5 min`, optional small brim); slides into `Back_Chassis_Frame` twin plunger tracks (`0.12 mm/side` slip). |
| [`back_chassis_print.stl`](cad/stl/back_chassis_print.stl) | `92.43` | `120.0 x 180.0 x 25.35` | `0.20 mm`, `4` walls, `25-30%` gyroid | Add a **`5 mm` outer brim** when printing open-air (prevents corner lift on the `120 x 180 mm` base). Includes conical standoffs, unified hollow plunger housing (`X = 46.00..60.00` w/ clip slots + centerline-split lower U-cradle), `4`-side slotted speaker L-pillars + `2 x M2` corner bosses, Left Headphone L-Tab (full-height `12.50 mm` inner pocket), equispaced `E220` SMA (`+20.65`), and downward `GX-12` D-flat. |
| [`top_cover_print.stl`](cad/stl/top_cover_print.stl) | `64.08` | `120.0 x 180.0 x 24.85` | `0.20 mm`, `4` walls, `25%` gyroid | Prints **face-down** (bed texture becomes front faceplate) with an open-top left notch clearing the headphone L-Tab. Use **outer-only brim** (or skirt only) so brim never fills the flush glass opening or `1.90 mm` U-groove. |
| [`pole_lid_print.stl`](cad/stl/pole_lid_print.stl) | `63.24` | `92.0 x 132.0 x 19.2` | `0.20 mm`, **`5-6` walls**, **`60-100%` gyroid** | Primary structural mount carrying the `1/4"-20` survey pole clamp. Use `100%` infill (or a `100%` infill modifier over the central spine) so clamp torque cannot crush the bosses. |
| [`pole_tilt_wedge_print.stl`](cad/stl/pole_tilt_wedge_print.stl) | `54.12` | `66.9 x 89.8 x 32.0` | `0.20 mm`, **`5-6` walls**, **`40-60%` gyroid** | Skeletonized open Warren-truss bracket (**`32.0 mm` tall, side-down `Z = 0`, `0` supports**) with pure round `1/4"-20` SHCS bores/holes (`3-4 mm` optional outer brim). Setting `6` walls (`2.40 mm` @ `0.40 mm`) makes the `4.5 mm` outer chords & mid-strut, `4.0 mm` tie struts, and `3.2 mm` skirts 100% solid perimeters while `40-60%` gyroid fills the `8.0 mm` spine plate & base boss. |
| [`bat_cradle_print.stl`](cad/stl/bat_cradle_print.stl) | `32.14` | `92.0 x 132.0 x 12.8` | `0.20 mm`, **`5` walls**, `25%` gyroid | Setting `5` walls makes the `2.0 mm` outer/inner walls and `1.8 mm` Warren-truss web print as 100% solid perimeters. |
| [`bat_adapter_5000_print.stl`](cad/stl/bat_adapter_5000_print.stl) | `13.90` | `64.0 x 115.8 x 12.6` | `0.20 mm`, **`5` walls**, `25%` gyroid | Only needed when installing the `5000 mAh` LiPo cell inside the standardized `10000 mAh` bay. |
| [`slim_sun_shield_print.stl`](cad/stl/slim_sun_shield_print.stl) | `9.70` | `69.2 x 88.8 x 29.7` | `0.20 mm`, `4` walls (`100%` solid) | **Use a `5 mm` outer brim** (narrow `1.60 mm` U-channel footprint needs extra bed grip) and keep outer-wall speed at `25-30 mm/s`. |

* **Total Filament Estimate:** `330.51 cm^3` CAD solid volume across all `9` parts (`316.61 cm^3` for the `10000 mAh` build omitting `bat_adapter_5000_print.stl`), using roughly `~275-330 g` of PETG (`~0.3` of a `1 kg` spool at `1.27 g/cm^3` with recommended wall/infill overrides and brims).

---

## 3. Ruthex Heat-Set Insert BOM (`23` Inserts Total)

Install all inserts cleanly before mounting electronics using a soldering iron set to **`250-260 deg C`** (slightly cooler than ABS so PETG does not pool at the bottom of blind `M2` pockets):

| Qty | Thread | Ruthex Part / Spec | CAD Bore (`Dia x Depth`) | Enclosure Location | Closed Floor Below Bore |
| :---: | :--- | :--- | :--- | :--- | :--- |
| **`6`** | `M3` | Ruthex `RX-M3x5.7` (`M3S`, `OD 4.6 mm x L 5.7 mm`) | `Dia 4.0 x 8.0 mm` | `Back_Chassis_Frame` front-entry cover bosses (`4` corners + `2` mid-sides, top `Z = -2.65 mm`) | `17.35 mm` solid column |
| **`4`** | `M3` | Ruthex `RX-M3x5.7` (`M3S`, `OD 4.6 mm x L 5.7 mm`) | `Dia 4.0 x 6.9 mm` | `Back_Chassis_Frame` rear Option B2 pod bosses (`(-38/+42, 42/142)`) | `1.10 mm` closed top cap |
| **`4`** | `M2` | Ruthex `RX-M2x4.0` (`M2S`, `OD 3.6 mm x L 4.0 mm`) | `Dia 3.2 x 5.0 mm` | `Back_Chassis_Frame` -> `Power Board` standoffs (`2.55 mm` standoff height, `16.0 x 76.0 mm`) | `0.55 mm` closed rear skin |
| **`4`** | `M2` | Ruthex `RX-M2x4.0` (`M2S`, `OD 3.6 mm x L 4.0 mm`) | `Dia 3.2 x 5.0 mm` | `Back_Chassis_Frame` -> `E220 LoRa` standoffs (`8.50 mm` standoff height) | `6.50 mm` closed standoff |
| **`2`** | `M2` | Ruthex `RX-M2x4.0` (`M2S`, `OD 3.6 mm x L 4.0 mm`) | `Dia 3.2 x 5.0 mm` | `Back_Chassis_Frame` -> `Speaker` diagonal corner L-pillars (`(-45.2, 33.0)` & `(-21.2, 9.0)`, flush at `Z = -13.50`) | `6.50 mm` solid PETG below bore (`>= 2.90 mm` clear of headphone jack) |
| **`2`** | `1/4"-20` | Ruthex `RX-1/4"x12.7` (`OD ~9.6 mm x L 12.7 mm`) | `Dia 8.0 x 13.7 mm` | `Pole_Lid` central spine (`38.1 mm` / `1.500 in` vertical pitch) | `2.50 mm` spine + `3.00 mm` lid floor |
| **`1`** | `1/4"-20` | Ruthex `RX-1/4"x12.7` (`OD ~9.6 mm x L 12.7 mm`) | `Dia 8.0 x 13.7 mm` | `Pole_Tilt_Wedge_37_5` base saddle center (laser-mount `8.0 mm` stud) | `2.30-2.80 mm` closed boss roof (`>= 4.00 mm` U-wall / `12.00 mm` X-wall; `>= 2.82 mm` clear to lower SHCS path) |

### Insert Seating Tips
* **Cover Bosses (`Z = -2.65 mm`):** Press each `M3` cover insert straight down with light pressure until `~0.2 mm` proud, then immediately press the boss face flat with a cool steel ruler or scraper so melted PETG flashes flush rather than leaving a raised collar inside the `0.25 mm` clamping gap behind `Top_Cover`.
* **`Pole_Tilt_Wedge_37_5` Base Saddle (`V = 0`):** Guide `RX-1/4"x12.7` straight down through the `0.5 mm` lead-in chamfer and press strictly flush with the flat saddle floor (`V = 0`) using a cool metal straightedge so the wedge sits dead-flat on the `60 mm` laser shelf and engages `~7.5 mm` (`~6` threads) of the bare `8.0 mm` stud.

---

## 4. Fastener BOM (`28` Metric + `2` Imperial Screws) & Stack-Up Verification

Standard ISO/DIN length convention applies throughout:
* **Countersunk (`90-deg` CSK / DIN 7991 / ISO 10642):** Length is measured **overall including the conical head**.
* **Pan / Button Head (DIN 7985 / ISO 7380) & SHCS (DIN 912):** Length is measured **under the head**.

### Detailed Fastener Stack-Up Breakdown

| Joint | Qty | Kit Screw | Head Type | Notes / Washer | Stack Span | Thread Engagement | Max Safe Length |
| :--- | :---: | :--- | :--- | :--- | :---: | :---: | :---: |
| **`Top_Cover` -> `Back_Chassis_Frame` (front closure)** | **`6`** | **`M3 x 8 mm`** | `90-deg` Flat CSK | Kit 3 (`M3` CSK; `M3 x 10 mm` also fits) | `2.65 mm` | `5.35 mm` (`~10.7` threads) | `10.65 mm` |
| **`Pole_Lid` + `Battery_Cradle` -> `Back_Chassis_Frame`** | **`4`** | **`M3 x 20 mm`** | `90-deg` Flat CSK | Kit 3 (`M3` CSK; **never use `25 mm`**) | `15.90 mm` (`3.0 + 0.1 + 12.8`) | `4.10 mm` (`~8.2` threads) | `22.80 mm` |
| **`Waveshare 3.5B` -> Rear Pillars** | **`4`** | **`M2 x 20 mm`** | `90-deg` Flat CSK | Kit 1 (`M2` CSK) | `16.50 mm` (`Z=-28.0..-11.50`) | `3.50 mm` into brass pillars | -- |
| **`Holybro UM982` -> Rear Pillars (`1.60 mm` CSK recess)** | **`4`** | **`M2 x 12 mm`** | `90-deg` Flat CSK | Kit 1 (`M2` CSK; seated in `1.60 mm` recess) | `9.20 mm` (`Z=-26.4..-17.20`) | `2.80 mm` (`7.0` threads) | `13.20 mm` (`4.0 mm` hole) |
| **`Power Board` PCB -> Stand-offs** | **`4`** | **`M2 x 6 mm`** | Button Head | Kit 2 (`M2` Button + **`1 x M2` flat washer**) | `~1.57 mm` PCB + `~0.3-0.5 mm` washer | `~3.9-4.1 mm` into `4.0 mm` Ruthex | `~6.9 mm` (`5.0 mm` bore) |
| **`E220 LoRa` PCB -> Stand-offs** | **`4`** | **`M2 x 6 mm`** | Button Head | Kit 2 (`M2` Button + **`1 x M2` flat washer**) | `~1.50 mm` PCB + `~0.3-0.5 mm` washer | `~4.0-4.2 mm` into `4.0 mm` Ruthex | `~6.8 mm` (`5.0 mm` bore) |
| **`Speaker` diagonal ears -> Corner L-Pillar Bosses** | **`2`** | **`M2 x 8 mm`** | Button Head | Kit 2 (`M2` Button + optional `M2` flat washer; lightly snugged) | `3.70 mm` (`1.70 mm` ear + `2.00 mm` step above `-13.50`) + opt. `0.4 mm` washer | `~3.9-4.3 mm` into `4.0 mm` Ruthex | `~8.7 mm` (`5.0 mm` bore) |
| **`Pole_Tilt_Wedge_37_5` -> `Pole_Lid`** | **`2`** | **`1/4"-20 x 3/4"`** *(or `5/8"`)* | Allen SHCS (`3/16"` hex) | No washer (bare `Dia 9.53 mm` SHCS head seats `100%` flush inside pure round `Dia 11.40 mm` rear-chord wrench holes across open Warren truss) | `8.00 mm` (`WEDGE_SPINE_WEB`) | `11.05 mm` (`8.7` thr) w/ `3/4"`; `7.88 mm` (`6.2` thr) w/ `5/8"` | `21.70 mm` (`0.85"`) |

### Consolidated Fastener Shopping / Kit-Picking List (`28` Metric + `2` Imperial Screws, `8-10` Washers)

* **Kit 3 -- `M3` CSK Assortment (`10` pcs total):**
  * `6 x M3 x 8 mm` `90-deg` Flat-Head CSK *(or `M3 x 10 mm` CSK)* -- Top cover closure
  * `4 x M3 x 20 mm` `90-deg` Flat-Head CSK *(**never use `25 mm`**, which exceeds `22.80 mm` max safe length and breaches the `1.10 mm` closed boss cap into internal hardware)* -- Rear battery & pole mount pod
* **Kit 1 -- `M2` CSK Assortment (`8` pcs total):**
  * `4 x M2 x 20 mm` `90-deg` Flat-Head CSK -- Waveshare display mount (rear entry)
  * `4 x M2 x 12 mm` `90-deg` Flat-Head CSK -- Holybro `UM982` mount (rear entry via `1.60 mm` `CAD v2.1.6` recessed CSK pockets, leaving `1.20 mm` bottom margin in `UM982`'s `4.00 mm` tapped holes)
* **Kit 2 -- `M2` Button-Head + Washer Kit (`10` screws + `8-10` flat washers):**
  * `8 x M2 x 6 mm` Button Head + **`8 x M2` flat washers** (`0.3-0.5 mm` thick) -- Power Board (`4`) + E220 LoRa (`4`); the flat washer guarantees `>= 0.8 mm` bottom clearance above the `5.00 mm` blind Ruthex pocket floor
  * `2 x M2 x 8 mm` Button Head *(+ optional `2 x M2` flat washers, lightly snugged)* -- `Speaker` diagonal corner mounting ears (`(-45.2, 33.0)` & `(-21.2, 9.0)`)
* **Ruthex Heat-Set Inserts (`23` pcs total):**
  * `10 x M3` (`RX-M3x5.7`)
  * `10 x M2` (`RX-M2x4.0`) -- `4` `Power Board` + `4` `E220` + `2` `Speaker`
  * `3 x 1/4"-20` (`RX-1/4"x12.7`) -- `2` in `Pole_Lid` spine + `1` in `Pole_Tilt_Wedge_37_5` base saddle
* **Panel Jack, Light Pipes, Pole Clamp & Optional Padding:**
  * `1 x 3.5mm` panel-mount stereo headphone jack (`PJ-324M`, [`REF-headphone-jack.stp`](cad/step/REF-headphone-jack.stp)): `12.0 x 14.0 x 6.3 mm` rectangular body + **`Dia 8.0 mm M6` round knurled panel nut** (`2.00 mm` exposed thread outside `X = -60.0` when seated in the `1.50 mm` inner anti-rotation recess).
  * `4 x Dia 1.75 mm` clear PETG filament stubs (~`12 mm` long), pressed into `Top_Cover`'s `Dia 1.95 mm` `UM982` LED bores and trimmed flush.
  * `2 x 1/4"-20 x 3/4"` (or `5/8"`) stainless **Allen socket-head cap screws** (`SHCS`, `3/16"` hex, **no washer**) securing `Pole_Tilt_Wedge_37_5` to `Pole_Lid`'s twin inserts (`<= 12.7 mm` / `0.50 in` thread engagement).
  * **Mating Laser Pole Clamp ([`REF-laser_pole_mount.stp`](cad/step/REF-laser_pole_mount.stp)):** `60 x 135 x 10 mm min` sliding platform (`53.5 mm` slide travel `125.75..179.25 mm` from survey pole centerline); remove the laser-level adapter sleeve and knurled collar to expose the bare `8.0 mm` `1/4"-20` thumbscrew stud.
  * Optional `1 mm` closed-cell EVA / neoprene foam strip behind the LiPo pouch inside `Battery_Cradle` and around the `Speaker` pocket rim.

---

## 5. Recommended Assembly Sequence

1. **Dry-Fit Check:** Before installing any inserts, test-slide `Top_Cover` over `Back_Chassis_Frame`'s split-Left inner capture rim and Left Headphone L-Tab (`0.20 mm` notch lap clearance), test all `3` `Button_Plungers` in the unified hollow plunger housing (`X = 46.00..60.00`) plus `Plunger_Capture_Clip` sliding cleanly down into the continuous side-wall track slots (`0.12 mm/side` slip, `1.55 mm` travel), test-fit the `PJ-324M` headphone jack into the full-height `12.50 mm` left-wall inner anti-rotation pocket (`0.25 mm/side`) and `REF_gx12.stp` into the downward (`-Z_asm`) `GX-12` D-flat bore (`Z_flat = -19.15 mm`), test-seat `Pole_Lid`'s `24 x 74 mm` spine into `Pole_Tilt_Wedge_37_5`'s `2.5 mm` `45-deg`-funneled alignment pocket, and test-straddle `Pole_Tilt_Wedge_37_5`'s `60.50 mm` front/rear skirts over the `60 mm` laser shelf (`53.5 mm` slot travel `24.80..78.30 mm`).
2. **Install Heat-Set Inserts (`23` total):** Solder `10 x M3`, `10 x M2` (`4` Power Board + `4` E220 + `2` Speaker diagonal corner L-pillars), and `3 x 1/4"-20` inserts while `Back_Chassis_Frame`, `Pole_Lid`, and `Pole_Tilt_Wedge_37_5` are empty on the bench (keep `Pole_Tilt_Wedge_37_5` base `RX-1/4"x12.7` flush with the `V = 0` saddle floor and the `2` Speaker `M2` inserts flush at `Z = -13.50 mm`).
3. **Install Tactile Plungers & Rear-Screwed Modules First (Must Precede Pod Mount):**
   * Slide the `3` `Button_Plungers` from `-X` into the Right Flange (`PWR`, `RESET`, `BOOT`) over the lower U-cradle **before** bolting down `Waveshare 3.5B` (since `15.95 mm` plunger length exceeds the `10.65 mm` gap once `Waveshare` is mounted), then drop `Plunger_Capture_Clip` into the twin vertical guide tracks (optionally tack with a tiny drop of CA glue in the top track).
   * Snap `RTC` coin cell (`+` / wire harness toward `-Y`) into its under-display C-clip cradle (`(18.50, 64.00)`).
   * Mount `Waveshare 3.5B` with `4 x M2 x 20 mm CSK` from the rear (CSK heads flush at `Z = -28.0 mm`).
   * Mount `Holybro UM982` (`(-9.80, 151.40)`) with `4 x M2 x 12 mm CSK` from the rear (CSK heads seat inside the `1.60 mm` recessed pockets on `Back_Chassis_Frame`).
4. **Install Remaining Interior Hardware & Test Captive Plungers:**
   * Solder leads onto the `3.5mm` headphone jack **first**, seat the body into the Left L-Tab's full-height `12.50 mm` inner anti-rotation recess with solder pins pointing **`+Z` UP** toward the speaker cavity, tighten the `Dia 8.0 mm` round knurled nut cleanly outside `X = -60.0` (`2.00 mm` exposed `M6` thread), then press `Speaker` into its `4`-corner slotted tower (`+Y` wires exiting the `+Y` slot; `1.85 mm` clearance above jack pins) and lightly snug the `2 x M2 x 8 mm` button-head speaker screws (`+ optional M2` flat washers) into the diagonal corner `M2` inserts.
   * Fasten [`REF_power.stp`](cad/step/REF_power.stp) (`4 x M2 x 6 mm` Button Head + `M2` flat washer, `16.0 x 76.0 mm`) + `E220 LoRa` (`4 x M2 x 6 mm` Button Head + `M2` flat washer).
   * Fit `GX-12` bulkhead (`-39.50`, indexing its bottom key flat downward `-Z_asm` in the `11.25 mm` D-flat bore) and `E220` D-flat `SMA` pigtail (`+20.65`, equispaced at `20.30 mm` pitch alongside `UM982` SMAs).
   * **Verify all `3` plunger clicks directly on `Waveshare` before closing the cover** (both `Plunger_Capture_Clip` and the integral lower nose U-cradle keep them positively centered and anti-tip).
   * Plug and dress the `2x16` DuPont harness under `Waveshare` (`+X` right-facing header clears the `Speaker` by `10.13 mm` and clears the `RTC` cradle by `2.52+ mm`), and feed the battery `JST-PH` lead through `BAT_SLOT` (`(25.0, 35.5)`).
5. **Close Front Cover:** Lower `Top_Cover` straight down (`-Z`) onto `Back_Chassis_Frame` (positively trapping `Plunger_Capture_Clip` with `0.80 mm` top clearance) and torque `6 x M3 x 8 mm CSK` screws evenly; press `4` clear PETG light pipes into the LED bores and slide `Slim_Sun_Shield` into the front U-groove if used.
6. **Attach Rear Battery & Pole Pod:** Route the LiPo cell (`+ Battery_Adapter_5000` if using `5000 mAh`) against the Bottom-Left Capture Corner with leads exiting Bottom-Right through `BAT_SLOT`, seat `Battery_Cradle` + `Pole_Lid`, and secure with `4 x M3 x 20 mm CSK` screws (never `25 mm`).
7. **Mount `37.5 deg` Tilt Wedge & Laser Clamp:** Seat `Pole_Lid`'s `24 x 74 mm` spine into `Pole_Tilt_Wedge_37_5`'s `2.5 mm` `45-deg`-funneled alignment pocket and tighten `2 x 1/4"-20 x 3/4"` (or `5/8"`) Allen SHCS bolts (**no washers**, `100%` flush head seat) through the pure round rear `Dia 11.40 mm` wrench holes and open Warren-truss interior (`3/16"` ball-end or standard L-key). Remove the laser-level adapter sleeve and knurled collar from the laser pole clamp to expose the bare `8.0 mm` `1/4"-20` stud, orient the Rover sideways on the platform (`pole_left` recommended with pole clamp on the left `-X` side leaving right-side buttons + `MicroSD` open; symmetric for `pole_right`), straddle the `6.0 mm` front/rear skirts over the `60 mm` shelf edges, and tighten the clamp's thumbscrew into the wedge's base `1/4"-20` insert (`~7.5 mm` / `~6` threads engagement, leaving the bottom `USB-C` edge overhanging forward/below the `60 mm` shelf front edge unobstructed across the full `53.5 mm` slide).
