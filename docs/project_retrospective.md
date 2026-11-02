# Project Retrospective: RTK Rover Development & AI Collaboration

**Document Version:** `v1.0.0`  
**Firmware Baseline:** `v0.2.38`  
**Date:** October 28, 2026  
**Status:** Completed & Field Proven  
**Participants:** Lead Systems Engineer & Antigravity AI Pair Programmer  

---

## 1. Executive Reflection

The RTK Rover project achieved an uncommon engineering milestone: in under two weeks of rapid collaborative development, a complex multi-sensor embedded system evolved from component datasheets and empty repositories into a battle-tested, centimeter-accurate RTK field instrument with an interactive LVGL 9 touch interface, dual-antenna kinematic lever-arm compensation, and sub-inch boundary traverse verification.

Reflecting on the development lifecycle, this initiative was characterized by exceptionally low friction, high creative velocity, and true peer-to-peer technical alignment. Solutions frequently functioned on the first compile, enabling an iterative refinement cycle rather than days spent in catastrophic debugging loops.

This retrospective captures the engineering and collaborative methodologies that drove this success, analyzes the friction points, and formalizes core lessons for future autonomous and pair-programmed embedded projects.

---

## 2. What Made This Collaboration Unique & Effective

### 2.1. First-Try Functional Baselines
* **The Traditional Failure Mode:** Conventional AI interactions often yield pseudo-code, incomplete snippets (`// TODO: implement`), or naive algorithms that fail upon compilation, requiring multiple rounds of prompt correction just to reach a baseline build.
* **What Changed Here:** Every major subsystem (FreeRTOS dual-core dispatching, Bowring geodetic transformation, UM982 sentence parsing, 12-byte IMU burst read, multi-epoch averaging) was delivered as **complete, drop-in, production-ready code** with exact register definitions and header guards. Having working firmware on the very first try allowed the project to immediately pivot to hardware validation and incremental refinement.

### 2.2. Constructive Pushback & Advisory Partnership
* **Transition from "Subservient Coder" to "Peer Engineer":** In legacy AI workflows, agents frequently follow instructions literally even when the requested approach introduces subtle bugs, architectural debt, or UI clutter.
* **The Collaborative Pivot:** When questions arose regarding button placement (inside vs. outside the vertical scroll container on Tile 4) or prefix list structure (`PIN_` vs. clean `PIN` with universal `_Lxxx`), the collaboration operated on technical merit. The agent presented trade-off matrices, flagged UI collisions, and suggested simplified, unified architectures. The user received thoughtful analysis, asked clarifying questions, and directed decisions with minimal friction.

### 2.3. The "Hyper-Context Anchor" & Living Specifications
* **Continuous Context Grounding:** By maintaining `docs/hyper-context-anchor.md` and `docs/project_overview.md`, the AI assistant's memory never degraded across conversation boundaries. Hard physical roadblocks (e.g., zero I2C on Core 0, no `%f` in `lv_label_set_text_fmt`, never touch CAD files, 64-bit `double` for WGS84) were persistently enforced without repetitive reminders.
* **Strict Semantic Versioning (`v0.2.1` $\rightarrow$ `v0.2.38`):** Bumping the firmware version string in `user_config.h` and adding structured entries in `CHANGELOG.md` at every logical step gave both human and AI unambiguous checkpoints in the Git trajectory.

---

## 3. The Good, The Bad, and The Ugly: Development Process Analysis

```
+----------------------------------------------------------------------------------------------------+
|                                      DEVELOPMENT RETROSPECTIVE                                     |
+------------------------------------+----------------------------------+----------------------------+
| THE GOOD                           | THE BAD                          | THE UGLY                   |
+------------------------------------+----------------------------------+----------------------------+
| • High velocity & fast convergence | • I2C bus clock-stretching stalls| • Physical sensor inverted |
| • Zero-orbit lever-arm projection  | • LVGL float format crash trap   | • E220 1,000B FIFO limits  |
| • Sub-inch deed boundary closure   | • Lost base station coordinates  | • 1-bit SD_MMC vs SPI boot |
| • Clean prefix & unified postfix   | • Context truncation management  | • 320x480 screen budgeting |
+------------------------------------+----------------------------------+----------------------------+
```

### 3.1. The Good
* **Predictive Kinematics:** Deriving the 3D lever-arm projection mathematically before writing display code ensured that the ground-tip coordinates never orbited when rotating the pole.
* **COGO Python Verification:** Building `cogo_network_solver.py` allowed legal deed traverses to be solved and closed to $0.010\text{ ft}$ over $4,045\text{ ft}$, creating ground-truth control points (`PIN_DRIVEWAY_REF`, `PIN_TREE_BASE_REF`) that verified real-world RTK positioning to $1.9\text{ inches}$.
* **Clean Architectural Refinement:** The progression of Tile 2 (Screen 2) from a complex dual-switch layout to a unified dropdown with `<DERIVED>`, clean names (`PIN`, `PIPE`, `MONUMENT`), and universal `_Lxxx` suffix was an exemplar of software simplification reducing user cognitive load.

### 3.2. The Bad
* **Debugging Shared-Bus Peripherals Remotely:** Identifying that the capacitive touch controller was holding SCL low required analyzing subtle FreeRTOS task starvation patterns. Remote AI pair programming struggles when physical bus traces (oscilloscope / logic analyzer) are not visible, requiring careful diagnostic deduction.
* **Hidden Global Library Macros:** The LVGL 9 crash caused by `#define LV_USE_FLOAT 0` in the pre-compiled Arduino core was an invisible trap. It appeared as a standard null-pointer dereference rather than a `printf` va_arg misalignment.
* **Missing Field Ingestion:** Early versions missed latitude/longitude in secondary antenna sentences (`$GNGGAH`) and dropped Base Station coordinates during struct copy. Catching these required detailed log reviews of raw NMEA streams.

### 3.3. The Ugly
* **Hardware Realities vs. Ideal Models:**
  * The physical orientation of the QMI8658 chip required non-intuitive axis mapping ($a_y \rightarrow$ Roll, $a_x \rightarrow$ Pitch).
  * The E220 LoRa module's 1,000-byte FIFO choked on 1,200-byte RTCM3 bursts, forcing UART baud pacing ($57600\text{ baud}$).
  * MicroSD SPI bus contention forced a 3-stage fallback mount routine to ensure cold boot stability.
  * Fitting high-density surveyor metrics onto a 3.5" portrait screen ($320 \times 480$) required pixel-by-pixel spatial budgeting.

---

## 4. Key Engineering Learnings Captured for the Future

### Lesson 1: Enforce Core Affinity & Bus Ownership Early
Never allow multiple FreeRTOS cores to touch the same physical bus or peripheral family. Pinning all five I2C devices to Core 1 and giving Core 0 exclusive domain over UART streaming eliminated race conditions and eliminated the need for complex inter-core spinlocks.

### Lesson 2: Decouple Data Ingestion from UI Rendering via Queues
High-rate serial streams (NMEA, RTCM3) must never block on GUI drawing. Using `xQueueOverwrite()` to hand off an immutable snapshot of rover telemetry ensured that even if LVGL dropped frames during heavy touch interaction, the GNSS navigation engine and RTCM forwarder never lost a byte.

### Lesson 3: Zero Disk I/O Inside 30 Hz UI Loops
Reading an SD card file—even a quick existence check or header read—can take $20\text{ ms}..150\text{ ms}$ over SPI. Executing disk queries inside an LVGL widget update loop destroys frame rates and causes touch stutter. All dynamic file-derived labels (e.g. next available survey pin ID) must be computed on dirty triggers and cached.

### Lesson 4: Standardize Telemetry Schemas on Day One
Adopting standard ISO timestamping in Column 1 and semantic firmware version in Column 2 across all log files (`route_*.csv`, `survey_logged_pins.csv`) allowed automated Python post-processing tools to ingest files interchangeably without custom regex parsers.

### Lesson 5: Design Systems Around Physical Constraints, Not Assumptions
Antennas have phase-center offsets; survey poles flex; displays sit at angles in bracket clamps. Integrating features like the **Tare Pole Bracket Angle** button and 3D moving-baseline lever-arm compensation transformed the rover from a bench toy into a practical field surveying instrument.

---

## 5. Potential Future Evolutions (Roadmap)

While the `v0.2.38` baseline is complete and stable, future enhancements could extend its capabilities:

1. **Integrated NTRIP Client:** Implement an on-chip Wi-Fi / cellular hotspot NTRIP client (RTCM over IP), allowing the rover to operate via cellular CORS networks without a local LoRa base station.
2. **Bluetooth LE Mock Location Provider:** Broadcast NMEA `$GPGGA` sentences over BLE to Android GIS applications (e.g., QField, SW Maps, Locus GIS), allowing the rover to drive mobile mapping apps seamlessly.
3. **Real-Time Polygon Boundary Area Computation:** Add a closed-traverse polygon engine that computes parcel acreage, square footage, and perimeter in real time as boundary pins are logged.
4. **Horizontal Error Ellipse Display:** Parse `$GPGST` semi-major and semi-minor error axes and render a real-time 95% confidence covariance ellipse on the Tile 0 Bullseye canvas.

---

## 6. Conclusion

The RTK Rover project exemplifies modern collaborative engineering at its best: clear vision and domain expertise from the engineer, paired with precise, grounded, and proactive code generation from the AI assistant. By maintaining technical rigor, clear architectural boundaries, and mutual respect for physical constraints, the project achieved an exceptional standard of execution.
