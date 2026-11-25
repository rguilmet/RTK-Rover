# Ebyte E220-900T30S Practical Engineering Reference (Moved)

The full **Ebyte E220-900T30S Practical Engineering Reference (`v3.2`)** has been promoted to [`Field_Instrument_Core`](https://github.com/rguilmet/field-instrument-core) so shared LoRa hardware specifications stay available across all future `Field_Instrument_Core` instruments:

* **Full Engineering Reference (`v3.2`):** [`Field_Instrument_Core/docs/e220-reference.md`](https://github.com/rguilmet/field-instrument-core/blob/main/docs/e220-reference.md) *(local library copy: `../../libraries/Field_Instrument_Core/docs/e220-reference.md`)*
* **Official Vendor Manuals & Articles:** [`Field_Instrument_Reference` (`datasheets/EByte E220` & `articles/LoRa_E220`)](https://github.com/rguilmet/field-instrument-reference/tree/main/datasheets/EByte%20E220)

### Local `RTK_Rover` Implementations & Shared Utilities
* **ESP32-S3 Driver & Telemetry:** [`src/e220_driver/e220_driver.cpp`](../src/e220_driver/e220_driver.cpp), [`src/e220_driver/rover_telemetry.h`](../src/e220_driver/rover_telemetry.h), [`user_config.h`](../user_config.h)
* **Raspberry Pi Pacer Daemon & Verifier (`RTK_Rover`):** [`scripts/e220_pacer.py`](../scripts/e220_pacer.py) (`v3.7.1`), [`scripts/e220-pacer.service`](../scripts/e220-pacer.service), [`scripts/set_e220_power.py`](../scripts/set_e220_power.py) (`v1.1.0`), [`scripts/stage2_bench_verify.py`](../scripts/stage2_bench_verify.py) (`v1.0.3`), [`docs/benchmarks/e220_stage1_20261116/README.md`](benchmarks/e220_stage1_20261116/README.md)
* **Shared E220 Setup & Bench Suite (`Field_Instrument_Core`):** [`extras/tools/e220/`](https://github.com/rguilmet/field-instrument-core/tree/main/extras/tools/e220) (`setup_e220.py`, `scan_e220_channels.py`, `bench_e220_multirate.py`, `test_e220_link.py`)
