# RTK Rover Snapmaker 2.0 PETG Print Guide & Fastener BOM (`CAD v2.1.5`)

Complete 3D-printing profile, slicer overrides, heat-set insert reference, verified metric screw stack-up calculations, and assembly order for the **RTK Rover Option A** mechanical enclosure (`CAD v2.1.5`).

* **Enclosure Architecture & CAD Previews:** [`cad/README.md`](cad/README.md)
* **Pre-Oriented Support-Free STL Files (`7` files):** [`cad/stl/`](cad/stl/)
* **Parametric Source Datums:** [`cad/rover_specs.py`](cad/rover_specs.py)

All `7` meshes in [`cad/stl/`](cad/stl/) (`*_print.stl`) are already rotated flat onto `Z = 0` and engineered to print **100% support-free** with a standard `0.4 mm` nozzle.

---

## 1. Snapmaker 2.0 PETG Baseline Profile (`0.4 mm` Nozzle)

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
| **Supports** | **OFF (`None`)** | All horizontal holes (`USB-C`, `MicroSD`, `GX-12`, D-flat `SMA`, inverted-U comb) and `CSK` cones are self-supporting. |

### Pre-Print PETG Tips
* **Dry Spool First:** Even sealed PETG absorbs moisture; drying at `60-65 deg C` for `4-6 h` eliminates stringing inside the `Battery_Cradle` Warren truss, `1.90 mm` Sun Shield U-groove, and `4` `Dia 1.95 mm` `UM982` LED holes.
* **Clean Brass Nozzle:** Wipe hot residue from the nozzle tip with a brass brush immediately before printing [`back_chassis_print.stl`](cad/stl/back_chassis_print.stl) and [`top_cover_print.stl`](cad/stl/top_cover_print.stl) to avoid dark PETG blobs on outer walls or the front bezel.

---

## 2. Per-Part Slicer Overrides (`7` STL Files)

| STL File | Vol (`cm^3`) | Layer / Walls / Infill | Adhesion & Print Notes |
| :--- | :---: | :--- | :--- |
| [`button_plungers_print.stl`](cad/stl/button_plungers_print.stl) | `0.62` | **`0.12-0.16 mm`**, `4` walls, **`100%` infill** | **Print first (~15 min) as a PETG tuning check.** Slow to `20-25 mm/s`, fan `40-50%`, min layer time `12-15 s`, **`4-5 mm` outer brim** so all `3` `Dia 3.20 / 4.10 mm` stems print cleanly round. |
| [`back_chassis_print.stl`](cad/stl/back_chassis_print.stl) | `92.93` | `0.20 mm`, `4` walls, `25-30%` gyroid | Add a **`5 mm` outer brim** when printing open-air (prevents corner lift on the `120 x 180 mm` base). Keep seam aligned along a rear/interior corner. |
| [`top_cover_print.stl`](cad/stl/top_cover_print.stl) | `65.04` | `0.20 mm`, `4` walls, `25%` gyroid | Prints **face-down** (bed texture becomes front faceplate). Use **outer-only brim** (or skirt only) so brim never fills the flush glass opening or `1.90 mm` U-groove. |
| [`pole_lid_print.stl`](cad/stl/pole_lid_print.stl) | `63.24` | `0.20 mm`, **`5-6` walls**, **`60-100%` gyroid** | Primary structural mount carrying the `1/4"-20` survey pole clamp. Use `100%` infill (or a `100%` infill modifier over the central spine) so clamp torque cannot crush the bosses. |
| [`bat_cradle_print.stl`](cad/stl/bat_cradle_print.stl) | `32.14` | `0.20 mm`, **`5` walls**, `25%` gyroid | Setting `5` walls makes the `2.0 mm` outer/inner walls and `1.8 mm` Warren-truss web print as 100% solid perimeters. |
| [`bat_adapter_5000_print.stl`](cad/stl/bat_adapter_5000_print.stl) | `13.90` | `0.20 mm`, **`5` walls**, `25%` gyroid | Only needed when installing the `5000 mAh` LiPo cell inside the standardized `10000 mAh` bay. |
| [`slim_sun_shield_print.stl`](cad/stl/slim_sun_shield_print.stl) | `9.70` | `0.20 mm`, `4` walls (`100%` solid) | **Use a `5 mm` outer brim** (narrow `1.60 mm` U-channel footprint needs extra bed grip) and keep outer-wall speed at `25-30 mm/s`. |

---

## 3. Ruthex Heat-Set Insert BOM (`20` Inserts Total)

Install all inserts cleanly before mounting electronics using a soldering iron set to **`250-260 deg C`** (slightly cooler than ABS so PETG does not pool at the bottom of blind `M2` pockets):

| Qty | Thread | Ruthex Part / Spec | CAD Bore (`Dia x Depth`) | Enclosure Location | Closed Floor Below Bore |
| :---: | :--- | :--- | :--- | :--- | :--- |
| **`6`** | `M3` | Ruthex `RX-M3x5.7` (`M3S`, `OD 4.6 mm x L 5.7 mm`) | `Dia 4.0 x 8.0 mm` | `Back_Chassis_Frame` front-entry cover bosses (`4` corners + `2` mid-sides, top `Z = -2.65 mm`) | `17.35 mm` solid column |
| **`4`** | `M3` | Ruthex `RX-M3x5.7` (`M3S`, `OD 4.6 mm x L 5.7 mm`) | `Dia 4.0 x 6.9 mm` | `Back_Chassis_Frame` rear Option B2 pod bosses (`(-38/+42, 42/142)`) | `1.10 mm` closed top cap |
| **`4`** | `M2` | Ruthex `RX-M2x4.0` (`M2S`, `OD 3.6 mm x L 4.0 mm`) | `Dia 3.2 x 5.0 mm` | `Back_Chassis_Frame` -> `Power Board` standoffs (`2.55 mm` standoff height) | `0.55 mm` closed rear skin |
| **`4`** | `M2` | Ruthex `RX-M2x4.0` (`M2S`, `OD 3.6 mm x L 4.0 mm`) | `Dia 3.2 x 5.0 mm` | `Back_Chassis_Frame` -> `E220 LoRa` standoffs (`8.50 mm` standoff height) | `6.50 mm` closed standoff |
| **`2`** | `1/4"-20` | Ruthex `RX-1/4"x12.7` (`OD ~9.6 mm x L 12.7 mm`) | `Dia 8.0 x 13.7 mm` | `Pole_Lid` central spine (`38.1 mm` / `1.500 in` vertical pitch) | `2.50 mm` spine + `3.00 mm` lid floor |

### Insert Seating Tip for Cover Bosses (`Z = -2.65 mm`)
Press each `M3` cover insert straight down with light pressure until `~0.2 mm` proud, then immediately press the boss face flat with a cool steel ruler or scraper so melted PETG flashes flush rather than leaving a raised collar inside the `0.25 mm` clamping gap behind `Top_Cover`.

---

## 4. Metric Screw BOM (`26` Screws Total) & Stack-Up Verification

Standard ISO/DIN length convention applies throughout:
* **Countersunk (`90-deg` CSK / DIN 7991 / ISO 10642):** Length is measured **overall including the conical head**.
* **Pan / Button Head (DIN 7985 / ISO 7380) & SHCS (DIN 912):** Length is measured **under the head**.

### Detailed Fastener Stack-Up Breakdown

| Joint | Qty | Recommended Screw | Head Type | Acceptable Alt. | Stack Span | Thread Engagement | Max Safe Length |
| :--- | :---: | :--- | :--- | :--- | :---: | :---: | :---: |
| **`Top_Cover` -> `Back_Chassis_Frame` (front closure)** | **`6`** | **`M3 x 8 mm`** *(ideal)* | `90-deg` Flat CSK | `M3 x 10 mm` CSK (`0.65 mm` bottom margin) | `2.65 mm` | `5.35 mm` (`8 mm`) / full `5.7 mm` (`10 mm`) | `10.65 mm` |
| **`Pole_Lid` + `Battery_Cradle` -> `Back_Chassis_Frame`** | **`4`** | **`M3 x 22 mm`** *(ideal)* | `90-deg` Flat CSK | `M3 x 20 mm` CSK (`4.10 mm` engagement) | `15.90 mm` (`3.0 + 0.1 + 12.8`) | `6.10 mm` (`22 mm`, full `5.7 mm` Ruthex) | `22.80 mm` |
| **`Waveshare 3.5B` -> Rear Pillars** | **`4`** | **`M2 x 20 mm`** | `90-deg` Flat CSK | -- | `16.50 mm` (`Z=-28.0..-11.50`) | `3.50 mm` into brass pillars | -- |
| **`Holybro UM982` -> Rear Pillars** | **`4`** | **`M2 x 14 mm`** | `90-deg` Flat CSK | -- | `10.80 mm` (`Z=-28.0..-17.20`) | `3.20 mm` into UM982 housing | -- |
| **`Power Board` PCB -> Stand-offs** | **`4`** | **`M2 x 5 mm`** | Pan / Button Head | `M2 x 6 mm` max (`0.6 mm` bottom margin) | `~1.6 mm` PCB | `~3.4 mm` into `4.0 mm` Ruthex | `~6.6 mm` (`5.0 mm` bore) |
| **`E220 LoRa` PCB -> Stand-offs** | **`4`** | **`M2 x 5 mm`** | Pan / Button Head | `M2 x 6 mm` max (`0.6 mm` bottom margin) | `~1.6 mm` PCB | `~3.4 mm` into `4.0 mm` Ruthex | `~6.6 mm` (`5.0 mm` bore) |

### Consolidated Fastener Shopping List (Stainless Steel 304 / A2-70 Recommended)

* **M3 Screws (`10` pcs total):**
  * `6 x M3 x 8 mm` `90-deg` Flat-Head CSK *(or `M3 x 10 mm` CSK)* -- Top cover closure
  * `4 x M3 x 22 mm` `90-deg` Flat-Head CSK *(or `M3 x 20 mm` CSK)* -- Rear battery & pole mount pod
* **M2 Screws (`16` pcs total):**
  * `4 x M2 x 20 mm` `90-deg` Flat-Head CSK -- Waveshare display mount (rear entry)
  * `4 x M2 x 14 mm` `90-deg` Flat-Head CSK -- Holybro UM982 mount (rear entry)
  * `8 x M2 x 5 mm` Pan or Button Head *(max `M2 x 6 mm`)* -- Power Board (`4`) + E220 LoRa (`4`)
* **Ruthex Heat-Set Inserts (`20` pcs total):**
  * `10 x M3` (`RX-M3x5.7`)
  * `8 x M2` (`RX-M2x4.0`)
  * `2 x 1/4"-20` (`RX-1/4"x12.7`)
* **Light Pipes, Pole Clamp & Optional Padding:**
  * `4 x Dia 1.75 mm` clear PETG filament stubs (~`12 mm` long), pressed into `Top_Cover`'s `Dia 1.95 mm` `UM982` LED bores and trimmed flush.
  * `2 x 1/4"-20` stainless bolts matched to your survey pole clamp bracket (max thread engagement into `Pole_Lid` **`<= 12.7 mm` / `0.50 in`**).
  * Optional `1 mm` closed-cell EVA / neoprene foam strip behind the LiPo pouch inside `Battery_Cradle` and around the `Speaker` pocket rim.

---

## 5. Recommended Assembly Sequence

1. **Dry-Fit Check:** Before installing any inserts, test-slide `Top_Cover` over `Back_Chassis_Frame`'s `4.50 mm` inner capture rim and test all `3` `Button_Plungers` in the Right Flange (`1.55 mm` travel).
2. **Install Heat-Set Inserts (`20` total):** Solder `10 x M3`, `8 x M2`, and `2 x 1/4"-20` inserts while `Back_Chassis_Frame` and `Pole_Lid` are empty and flat on the bench.
3. **Install Rear-Screwed Modules First (Must Precede Pod Mount):**
   * Snap `RTC` coin cell (`+` / wire harness toward `-Y`) into its under-display C-clip cradle (`(18.50, 64.00)`).
   * Mount `Waveshare 3.5B` with `4 x M2 x 20 mm CSK` from the rear (CSK heads flush at `Z = -28.0 mm`).
   * Mount `Holybro UM982` (`( -9.80, 151.40 )`) with `4 x M2 x 14 mm CSK` from the rear.
4. **Install Remaining Interior Hardware:**
   * Press `Speaker` into its lower-left pocket (`+Y` wires), and fasten `Power Board` (`4 x M2 x 5 mm`) + `E220 LoRa` (`4 x M2 x 5 mm`).
   * Fit `GX-12` bulkhead (`-39.50`) and `E220` D-flat `SMA` pigtail (`+29.00`).
   * Slide the `3` `Button_Plungers` from `-X` into the Right Flange (`PWR`, `RESET`, `BOOT`).
   * Plug and dress the `2x16` DuPont harness under `Waveshare` (`+X` right-facing header clears the `Speaker` by `10.13 mm` and clears the `RTC` cradle by `2.52+ mm`), and feed the battery `JST-PH` lead through `BAT_SLOT` (`(25.0, 35.5)`).
5. **Close Front Cover:** Align `Top_Cover`'s downward inverted-U plunger comb straight down (`-Z`) onto `Back_Chassis_Frame` and torque `6 x M3 x 8 mm CSK` screws evenly; press `4` clear PETG light pipes into the LED bores and slide `Slim_Sun_Shield` into the front U-groove if used.
6. **Attach Rear Battery & Pole Pod:** Route the LiPo cell (`+ Battery_Adapter_5000` if using `5000 mAh`) against the Bottom-Left Capture Corner with leads exiting Bottom-Right through `BAT_SLOT`, seat `Battery_Cradle` + `Pole_Lid`, and secure with `4 x M3 x 22 mm CSK` screws.
