# Implementation Plan: Public/Private Code & Data Decoupling (Option B: Sibling Directory)

## Executive Overview
This implementation plan establishes a clean architectural separation between **generic mathematical logic** (which belongs in the open-source public repository) and **personal property data, coordinates, and credentials** (which remain strictly private and local). 

Per the architectural decision, we are adopting **Option B: Dedicated Sibling Repository (`RTK_Rover_Private`)** located outside the public repository tree at `../RTK_Rover_Private`. This eliminates the fragile reliance on `.gitignore` for IP protection, provides full independent Git version control and rollback capability for private assets, and adheres to state-of-the-art software engineering practices.

---

## Architecture Model & Directory Topology

```
..\
│
├── RTK_Rover/                                  <-- PUBLIC GIT REPOSITORY (github.com/rguilmet/RTK-Rover)
│   ├── .git/                                   <-- Public git tracking (clean open-source history)
│   ├── RTK_Rover.ino                           <-- Firmware entry point
│   ├── src/ & include/                         <-- Dual-core RTK & UI modules
│   ├── scripts/                                <-- Generic public tools (parameterized & schema-driven)
│   │   ├── check_secrets.py                    <-- Generic PII/key scanner (reads optional .secrets.local)
│   │   ├── cogo_solver.py                      <-- Generic Cadastral COGO & traverse engine
│   │   ├── e220_pacer.py                       <-- Base station hardware pacer
│   │   └── install_git_hooks.py                <-- Pre-push hook installer
│   ├── examples/                               <-- Fictional reference datasets (for testing & docs)
│   │   ├── subdivision_demo.json               <-- Sample 3-lot survey network
│   │   └── control_points.sample.csv           <-- Fictional anchor coordinates
│   ├── docs/                                   <-- Architecture & user documentation
│   │   ├── project_overview.md                 <-- Master technical specification
│   │   ├── hyper-context-anchor.md             <-- Operational context & constraints
│   │   ├── cogo_solver_guide.md                <-- Cadastral solver documentation
│   │   ├── session_handoff.md                  <-- Unversioned session handoff state
│   │   └── session_startup_prompt.md           <-- Startup brief for new AI sessions
│   ├── settings.sample.json                    <-- Template settings with placeholder credentials
│   └── .secrets.local                          <-- (Optional, .gitignore'd) Local personal scanner patterns
│
└── RTK_Rover_Private/                          <-- PRIVATE SIBLING DIRECTORY (Local Git Repository)
    ├── .git/                                   <-- Independent LOCAL Git repo (offline revision control)
    ├── cadastral/                              <-- Personal deed JSON & control anchor CSVs
    │   ├── homestead_parcels.json              <-- Deed calls for Lots 22A, 22B, Gunter, Holmes
    │   ├── control_anchors.csv                 <-- Real surveyed WGS84 ground truth corner pins
    │   ├── solve_deed_network.py               <-- (Legacy solver archived during Phase 3)
    │   └── solve_adjoiner_lots.py              <-- (Legacy adjoiner solver archived during Phase 3)
    ├── deeds/                                  <-- Registered deed PDFs, deed chain notes, OCR text
    ├── profiles/                               <-- Private base station profiles (callsigns, hostnames)
    ├── logs/                                   <-- Historical wire-tap captures & raw NMEA/RTCM dumps
    └── backup_scripts/                         <-- Local bare remote or USB backup sync scripts
```

---

## State-of-the-Art Software Engineering Principles Enforced

1. **Separation of Concerns (Code vs. Data):**  
   Computational algorithms (traverse loops, Bowditch adjustment, geodetic forward projection) are completely decoupled from domain content. Python scripts take CLI arguments (`--input`, `--anchors`, `--output`) and parse standard JSON/CSV schemas.
2. **True Physical Isolation:**  
   Private deeds, personal coordinates, and callsigns never exist in the directory tree of `RTK_Rover`. A `git push` in the public repository physically cannot publish private files.
3. **Full Independent Version Control for Private Work:**  
   `RTK_Rover_Private` has its own local Git database (`git init`), enabling commit histories, branch experimentation, and instant rollbacks for deed modeling without cloud exposure.
4. **Reproducible Testability:**  
   Public tools ship with clean, fictional datasets in `examples/` that run in automated continuous integration (CI) and serve as working documentation for users.
5. **Living Documentation Synchronization:**  
   Every architectural transition updates [`docs/project_overview.md`](project_overview.md), [`docs/session_handoff.md`](session_handoff.md), [`docs/hyper-context-anchor.md`](hyper-context-anchor.md), [`GEMINI.md`](../GEMINI.md), and [`README.md`](../README.md) in lockstep.

---

## Phased Implementation Roadmap

### Phase 1: Security Scanner Parameterization & Re-integration
**Objective:** Restore `scripts/check_secrets.py` as a generic public tool while keeping personal detection signatures strictly local.

- [x] **1.1 Design Local Configuration Format (`.secrets.local`):**
  - Define a clean JSON/text format for user-specific search patterns in the repository root:
    ```json
    {
      "forbidden_tokens": [
        "MY_CALLSIGN",
        "MY_HOSTNAME",
        "LAT_OR_LON_COORDINATE",
        "MY_WIFI_PASS"
      ]
    }
    ```
- [x] **1.2 Parameterize `scripts/check_secrets.py`:**
  - Strip all hardcoded private strings from Python code.
  - Retain generic security rules: AWS keys, GitHub PATs, private SSH keys, unredacted passwords in sample configs, and absolute Windows paths.
  - Auto-detect `.secrets.local` in repository root. If present, load user-defined tokens into the active scan set.
- [x] **1.3 Git Pre-Push Hook Update:**
  - Add `.secrets.local` to `.gitignore`.
  - Update `.git/hooks/pre-push` to run `python scripts/check_secrets.py`.
  - Update `scripts/install_git_hooks.py` to deploy the unified hook.
- [x] **1.4 Documentation Sync:**
  - Verify synchronization across `GEMINI.md`, `hyper-context-anchor.md`, and `session_handoff.md`.

---

### Phase 2: Private Sibling Repository Setup & Migration
**Objective:** Create `RTK_Rover_Private`, initialize local Git version control, migrate all sensitive assets out of `RTK_Rover/reference/`, and clean up the public repository.

- [ ] **2.1 Initialize Sibling Repository:**
  - Create `../RTK_Rover_Private`.
  - Initialize local Git tracking: `git init`.
  - Configure `.gitignore` for OS artifacts (`Thumbs.db`, `.DS_Store`, `__pycache__`).
- [ ] **2.2 Migrate Private Assets from `reference/`:**
  - Move deed PDFs into `RTK_Rover_Private/deeds/`.
  - Move `solve_deed_network.py` and `solve_adjoiner_lots.py` into `RTK_Rover_Private/cadastral/`.
  - Move private logs (`nmea_*.log`) into `RTK_Rover_Private/logs/`.
  - Move real waypoint CSVs (`waypoints-guilmet*.csv`, `waypoints-haslam.csv`, `waypoints-gray.csv`) into `RTK_Rover_Private/cadastral/`.
  - Initial Git commit in `RTK_Rover_Private` (`git commit -m "feat: initial commit of private survey records and solvers"`).
- [ ] **2.3 Clean Up Public Repository (`RTK_Rover`):**
  - Verify that all necessary files have been safely migrated to `RTK_Rover_Private`.
  - Remove unneeded remnants from `RTK_Rover/reference/`.
  - Remove `reference/check_secrets.py` (superseded by public parameterized `scripts/check_secrets.py`).
- [ ] **2.4 Documentation Sync:**
  - Update `docs/session_handoff.md` with migration completion status and directory paths.

---

### Phase 3: Cadastral Schema Definition & Generic COGO Engine
**Objective:** Create standard JSON schema for deed traverses and develop the generic CLI solver in `RTK_Rover/scripts/cogo_solver.py`.

- [ ] **3.1 Cadastral Schema Definition (`docs/schemas/cadastral_schema.json`):**
  - JSON schema supporting parcel metadata, quadrant bearings (`"N 14-30-00 E"`), distances with units, monument descriptions, and control ties.
- [ ] **3.2 Fictional Test Dataset (`examples/subdivision_demo.json`):**
  - Fictional 3-lot suburban/rural layout for tests and user reference.
- [ ] **3.3 Generic COGO Engine (`scripts/cogo_solver.py`):**
  - Forward geodesic projection, loop misclosure ratio calculation, Compass Rule (Bowditch) adjustment, and adjoiner common-line reconciliation.
  - Exporters: Google Earth KML, Rover `/waypoints.csv`, GeoJSON, and text closure reports.
  - Verified against `examples/subdivision_demo.json`.
- [ ] **3.4 Documentation Sync:**
  - Author [`docs/cogo_solver_guide.md`](cogo_solver_guide.md).
  - Update [`README.md`](../README.md) and [`docs/project_overview.md`](project_overview.md).

---

### Phase 4: Private Data Extraction & Mathematical Parity Validation
**Objective:** Express your property's deed calls in the new JSON format and verify exact mathematical parity with previous results.

- [ ] **4.1 Create Homestead Parcel Data:**
  - Author `RTK_Rover_Private/cadastral/homestead_parcels.json` using the new schema.
  - Author `RTK_Rover_Private/cadastral/control_anchors.csv` with known pin coordinates.
- [ ] **4.2 Parity Verification:**
  - Execute:
    ```bash
    python ../RTK_Rover/scripts/cogo_solver.py \
      --input cadastral/homestead_parcels.json \
      --anchors cadastral/control_anchors.csv \
      --kml output/homestead_solved.kml \
      --rover-csv output/waypoints.csv
    ```
  - Verify that solved coordinates match the historical coordinates to within sub-millimeter precision.
- [ ] **4.3 Archive Legacy Hardcoded Solvers:**
  - Archive `solve_deed_network.py` and `solve_adjoiner_lots.py` to `cadastral/legacy/` in `RTK_Rover_Private`.

---

### Phase 5: Documentation & Open-Source Release
**Objective:** Finalize user documentation and verify public repository cleanliness.

- [ ] **5.1 Master Documentation Audit:**
  - Comprehensive sync across `docs/project_overview.md`, `docs/session_handoff.md`, `docs/hyper-context-anchor.md`, `GEMINI.md`, and `README.md`.
- [ ] **5.2 Final Pre-Push Verification:**
  - Run `python scripts/check_secrets.py` with `.secrets.local` active to confirm 100% clean status.
