# RTK Rover Utilities & Deployment Scripts

This directory houses `RTK_Rover`-specific Python and shell utilities for the RTKBase Pi LoRa pacer, Stage 2 split-desk telemetry verification, cadastral COGO solving, field log analysis, schematic generation, and pre-push security scanning.

> [!NOTE]
> Generic, reusable hardware setup and bench utilities (`setup_e220.py`, `scan_e220_channels.py`, `bench_e220_multirate.py`, `test_e220_link.py`, `setup_um98x.py`) live in [`Field_Instrument_Core/extras/tools/`](https://github.com/rguilmet/field-instrument-core/tree/main/extras/tools).

## 1. Local `RTK_Rover` Script Index (`19` Active Files)

### Base Station LoRa Pacer & Deployment
| Script | Version | Purpose |
| :--- | :---: | :--- |
| [`e220_pacer.py`](e220_pacer.py) | `v3.7.1` | Architecture 2B asynchronous ping-pong mailbox Ebyte `E220-900T30S` LoRa daemon (`<= 200 B` coalescing, Stage-1 calibrated ToA + guard pacing, consolidated `Second 0` macro-cycle, `48 B` V3 telemetry uplink, and 4-ACK 3-rate switching). |
| [`e220-pacer.service`](e220-pacer.service) | `--` | `systemd` unit definition running `/usr/local/bin/e220_pacer.py` against `/etc/default/e220-pacer`. |
| [`update_e220_pacer.sh`](update_e220_pacer.sh) | `--` | Raspberry Pi deployment script that strips CRLF line endings, installs pacer/verifier scripts to `/usr/local/bin/` and `e220-pacer.service`, reloads `systemd`, and tails journal logs (`--no-follow` supported). |
| [`set_e220_power.py`](set_e220_power.py) | `v1.1.0` | Runtime utility to switch E220 TX power (`21`/`24`/`27`/`30 dBm`) via EEPROM Mode 3 + readback, sync `/etc/default/e220-pacer`, and restart `e220-pacer.service`. |

### Bench Verification & Unit Tests
| Script | Version | Purpose |
| :--- | :---: | :--- |
| [`stage2_bench_verify.py`](stage2_bench_verify.py) | `v1.0.3` | Architecture 2B split-desk verifier: `--self-test` (`7/7` offline suite), `--mode pi-bench`, `--mode pi-feeder`, `--mode rover-monitor`, and `--from-log` scorecard. |
| [`verify_g1_outdoor_soak.py`](verify_g1_outdoor_soak.py) | `v1.1.0` | Plan 2 Gate G1 outdoor `I2C1` & beam compass soak scorer (`0` `I2C0` delta errors, `< 2 deg` static heading RMS, `<= 1.0 deg` static pitch RMS, `< 2 deg` `FIX -> CMP` jump, `--self-test`). |
| [`test_rover_telemetry.py`](test_rover_telemetry.py) | `--` | Unit test verifying `48 B` V3 telemetry packing, CRC16-CCITT, atomic JSON output, rolling CSV pruning, and rate-switch control frames. |
| [`test_line_stakeout.py`](test_line_stakeout.py) | `--` | Offline test harness verifying WGS84 line-stakeout stationing, signed cross-track offset (`L/R`), and quadrant bearing formatting. |
| [`test_rover_core_math.py`](test_rover_core_math.py) | `v1.5.0` | Offline unit test covering `RTK_Rover` geodesic/navigation math, `F-03` Welford/gating logic, heading priority, low-battery debounce, `COMMS` consolidation / `F-11` portal guards / `DRAM0` checks, and source/ASCII caps. |

### Shared Hardware Utilities (`Field_Instrument_Core/extras/tools/`)
| Script | Version | Location | Purpose |
| :--- | :---: | :--- | :--- |
| `setup_e220.py` | `v1.3.0` | [`extras/tools/e220/setup_e220.py`](https://github.com/rguilmet/field-instrument-core/blob/main/extras/tools/e220/setup_e220.py) | Mode 3 E220 register configurator (Pi BCM GPIO or USB-FTDI). |
| `scan_e220_channels.py` | `v1.1.0` | [`extras/tools/e220/scan_e220_channels.py`](https://github.com/rguilmet/field-instrument-core/blob/main/extras/tools/e220/scan_e220_channels.py) | Passive `902-928 MHz` RF noise scanner (`0xC2` RAM, zero EEPROM wear). |
| `bench_e220_multirate.py` | `v1.0.3` | [`extras/tools/e220/bench_e220_multirate.py`](https://github.com/rguilmet/field-instrument-core/blob/main/extras/tools/e220/bench_e220_multirate.py) | Stage 1 hardware-in-the-loop multi-rate ToA/guard/PER/switch suite. |
| `test_e220_link.py` | `v1.1.0` | [`extras/tools/e220/test_e220_link.py`](https://github.com/rguilmet/field-instrument-core/blob/main/extras/tools/e220/test_e220_link.py) | Bidirectional RF link and burst throughput tester between two E220 units. |
| `setup_um98x.py` | `v1.3.0` | [`extras/tools/um98x/setup_um98x.py`](https://github.com/rguilmet/field-instrument-core/blob/main/extras/tools/um98x/setup_um98x.py) | Unified configuration CLI for Unicore `UM980` and `UM982` receivers. |

### Cadastral COGO, Deed Conversion & Waypoints
| Script | Version | Purpose |
| :--- | :---: | :--- |
| [`cogo_solver.py`](cogo_solver.py) | `v1.2.0` | Schema-driven Cadastral COGO & Traverse Engine (metes-and-bounds, circular curves, Bowditch adjustment, multi-parcel snapping, `20`-column `waypoints.csv`, KML, GeoJSON, DXF). |
| [`convert_deeds_to_markdown.py`](convert_deeds_to_markdown.py) | `v1.2.0` | Deed PDF transcript, structured JSON, and layered CAD DXF geometry exporter. |
| [`kml_to_waypoints.py`](kml_to_waypoints.py) | `v1.0.0` | Converts Google Earth / GIS `.kml` placemarks, paths, and polygons into `19`-column RTK Rover waypoint CSV files. |
| [`validate_waypoints.py`](validate_waypoints.py) | `v1.2.0` | Validates Rover waypoint CSV syntax/bounds and calculates leg distances, bearings, and loop closure perimeter. |

### Field Log Analysis & RF Mapping
| Script | Version | Purpose |
| :--- | :---: | :--- |
| [`analyze_rtk_log.py`](analyze_rtk_log.py) | `v1.0.0` | Tabular CLI inspector for Rover `/survey_logged_pins.csv` point logs. |
| [`analyze_field_test.py`](analyze_field_test.py) | `--` | Offline report comparing surveyed pins vs. design waypoints, route trajectory track metrics, and raw NMEA stream quality from local `test/` captures. |
| [`generate_rf_heatmap_kml.py`](generate_rf_heatmap_kml.py) | `v1.0.2` | Generates a Google Earth KML RF RSSI (`dBm`) heatmap and cadastral waypoint overlay from Rover serial logs, Base pacer logs, and waypoint CSVs. |

### Hardware Schematics & Wiring Diagrams
| Script | Version | Purpose |
| :--- | :---: | :--- |
| [`generate_wiring_diagrams.py`](generate_wiring_diagrams.py) | `v1.4.0` | Generates vector `.svg` and high-DPI `.png` hardware wiring schematics and `rev5` pin-mapping tables under `docs/schematics/`. |

### Security & Git Hooks
| Script | Version | Purpose |
| :--- | :---: | :--- |
| [`check_secrets.py`](check_secrets.py) | `v1.1.1` | Pre-push/pre-commit privacy & secrets scanner checking working files and Git history against generic patterns and optional `.secrets.local`. |
| [`install_git_hooks.py`](install_git_hooks.py) | `v1.1.0` | Installs `.git/hooks/pre-push` and `.git/hooks/pre-commit` wrappers invoking `scripts/check_secrets.py`. |

---

## 2. Scripting Standards

* **Independent SemVer Headers:** New or updated Python scripts should declare `__filename__ = os.path.basename(__file__)` and an independent `__version__ = "vX.Y.Z"` (decoupled from `ROVER_FIRMWARE_VERSION`), printing `=== {__filename__} {__version__} ===` on CLI entry.
* **Code / Data Separation & Pure ASCII:** Drive tools via CLI flags and external JSON/CSV schemas (ship synthetic datasets in `examples/`); never hardcode private deed coordinates, callsigns, Wi-Fi credentials, or local `C:/Users/<user>/...` filesystem paths. Keep CLI output pure 7-bit ASCII (`deg`, `+/-`, `->`).
* **Pre-Push Security Check:** Run `python scripts/check_secrets.py` before committing/pushing changes.
