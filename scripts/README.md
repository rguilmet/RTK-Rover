# RTK Rover Utilities & Deployment Scripts

This directory houses Python and shell utilities for the RTKBase Pi LoRa pacer, Stage 1/2 RF & telemetry verification, Unicore GNSS setup, cadastral COGO solving, field log analysis, schematic generation, and pre-push security scanning.

## 1. Script Index (`22` Active Files)

### Base Station LoRa Pacer & Deployment
| Script | Version | Purpose |
| :--- | :---: | :--- |
| [`e220_pacer.py`](e220_pacer.py) | `v3.7.1` | Architecture 2B asynchronous ping-pong mailbox Ebyte `E220-900T30S` LoRa daemon (`<= 200 B` coalescing, Stage-1 calibrated ToA + guard pacing, consolidated `Second 0` macro-cycle, `48 B` V3 telemetry uplink, and 4-ACK 3-rate switching). |
| [`e220-pacer.service`](e220-pacer.service) | `--` | `systemd` unit definition running `/usr/local/bin/e220_pacer.py` against `/etc/default/e220-pacer`. |
| [`update_e220_pacer.sh`](update_e220_pacer.sh) | `--` | Raspberry Pi deployment script that strips CRLF line endings, installs pacer/bench utilities to `/usr/local/bin/` and `e220-pacer.service`, reloads `systemd`, and tails journal logs (`--no-follow` supported). |
| [`set_e220_power.py`](set_e220_power.py) | `v1.1.0` | Runtime utility to switch E220 TX power (`21`/`24`/`27`/`30 dBm`) via EEPROM Mode 3 + readback, sync `/etc/default/e220-pacer`, and restart `e220-pacer.service`. |
| [`scan_e220_channels.py`](scan_e220_channels.py) | `v1.1.0` | Passive `902-928 MHz` ambient RF noise scanner using temporary RAM register writes (`0xC2`, zero EEPROM wear) and CSV export. |
| [`setup_e220.py`](setup_e220.py) | `v1.3.0` | Configuration and diagnostic utility for E220-900T modules supporting direct Raspberry Pi BCM GPIO or USB FTDI control lines. |
| [`test_e220_link.py`](test_e220_link.py) | `v1.1.0` | Bidirectional RF link and burst throughput tester between two E220 transceivers. |

### Bench Verification & Unit Tests
| Script | Version | Purpose |
| :--- | :---: | :--- |
| [`stage2_bench_verify.py`](stage2_bench_verify.py) | `v1.0.3` | Architecture 2B split-desk verifier: `--self-test` (`7/7` offline suite), `--mode pi-bench`, `--mode pi-feeder`, `--mode rover-monitor`, and `--from-log` scorecard. |
| [`bench_e220_multirate.py`](bench_e220_multirate.py) | `v1.0.3` | Stage 1 hardware-in-the-loop multi-rate (`4.8k` / `9.6k` / `19.2k`) ToA, guard sweep, PER, and 4-ACK switch characterization suite (also supports `--self-test`). |
| [`test_rover_telemetry.py`](test_rover_telemetry.py) | `--` | Unit test verifying `48 B` V3 telemetry packing, CRC16-CCITT, atomic JSON output, rolling CSV pruning, and rate-switch control frames. |
| [`test_line_stakeout.py`](test_line_stakeout.py) | `--` | Offline test harness verifying WGS84 line-stakeout stationing, signed cross-track offset (`L/R`), and quadrant bearing formatting. |

### GNSS Receiver Configuration
| Script | Version | Purpose |
| :--- | :---: | :--- |
| [`setup_um98x.py`](setup_um98x.py) | `v1.3.0` | Unified configuration and diagnostic CLI for Unicore `UM980` (Base, `--device um980`) and `UM982` (Rover dual-antenna, `--device um982`) receivers (`--freset`, `--query`, signal-group sync). |

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
