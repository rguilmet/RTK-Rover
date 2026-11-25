#!/usr/bin/env python3
"""
Gate G1 Outdoor I2C1 & Beam Compass Soak Verifier for RTK_Rover.

Parses Rover serial capture logs containing [10DOF I2C1] (or [10DOF I2C0]) and
[TELEM] lines and evaluates Plan 2 Gate G1 quantitative pass criteria:
  1. 0 unresolved I2C0 errors over the soak window (I2C:0/x)
  2. < 2.0 deg RMS / max static beam compass error vs UM982 HDG_FIX
  3. <= 1.0 deg pitch agreement (|dP| <= 1.0 deg)
  4. 0.0 s HDG_FIX -> [CMP] transition with < 2.0 deg heading jump
"""

import argparse
import math
import os
import re
import sys

__filename__ = os.path.basename(__file__)
__version__ = "v1.0.0"

RE_10DOF = re.compile(
    r"\[10DOF I2C(?P<bus>\d+)\]\s+"
    r"P/R:(?P<pitch>[+-]?\d+(?:\.\d+)?)/(?P<roll>[+-]?\d+(?:\.\d+)?)\s+deg\s+\|\s+"
    r"\|B\|:(?P<btot>\d+(?:\.\d+)?)uT\s+\|\s+"
    r"Hdg:(?P<hdg>\d+(?:\.\d+)?)\[(?P<src>[A-Z-]+)\]\s+"
    r"Mag:(?P<mag>\d+(?:\.\d+)?)(?P<anom>\s+ANOM)?\s+\|\s+"
    r"FOC:(?P<foc>[A-Z]+)\s+\|\s+"
    r"P:(?P<press>\d+(?:\.\d+)?)hPa"
    r"(?:\s+\|\s+dH:(?P<dh>[+-]?\d+(?:\.\d+)?)\s+dP:(?P<dp>[+-]?\d+(?:\.\d+)?))?"
    r"(?:\s+\|\s+I2C:(?P<i2c0>\d+)/(?P<i2c1>\d+))?"
)


def wrap_deg_180(angle_deg: float) -> float:
    return (angle_deg + 180.0) % 360.0 - 180.0


def parse_g1_lines(lines):
    samples = []
    for raw in lines:
        m = RE_10DOF.search(raw)
        if not m:
            continue
        hdg = float(m.group("hdg"))
        mag = float(m.group("mag"))
        dh = float(m.group("dh")) if m.group("dh") is not None else wrap_deg_180(mag - hdg)
        dp = float(m.group("dp")) if m.group("dp") is not None else None
        samples.append({
            "bus": int(m.group("bus")),
            "pitch": float(m.group("pitch")),
            "roll": float(m.group("roll")),
            "btot": float(m.group("btot")),
            "hdg": hdg,
            "src": m.group("src"),
            "mag": mag,
            "anom": bool(m.group("anom")),
            "foc": m.group("foc"),
            "press": float(m.group("press")),
            "dh": dh,
            "dp": dp,
            "i2c0_err": int(m.group("i2c0")) if m.group("i2c0") is not None else None,
            "i2c1_err": int(m.group("i2c1")) if m.group("i2c1") is not None else None,
        })
    return samples


def evaluate_g1_samples(samples, max_hdg_err_deg=2.0, max_pitch_err_deg=1.0, max_jump_deg=2.0):
    if not samples:
        return {
            "passed": False,
            "reason": "No [10DOF I2C*] samples found in log.",
            "sample_count": 0,
        }

    i2c1_count = sum(1 for s in samples if s["bus"] == 1)
    i2c0_errs = [s["i2c0_err"] for s in samples if s["i2c0_err"] is not None]
    i2c1_errs = [s["i2c1_err"] for s in samples if s["i2c1_err"] is not None]
    max_i2c0_err = max(i2c0_errs) if i2c0_errs else 0
    max_i2c1_err = max(i2c1_errs) if i2c1_errs else 0

    fix_dh = [abs(s["dh"]) for s in samples if s["src"] == "FIX" and not s["anom"]]
    rms_dh = math.sqrt(sum(v * v for v in fix_dh) / len(fix_dh)) if fix_dh else None
    max_dh = max(fix_dh) if fix_dh else None

    fix_dp = [abs(s["dp"]) for s in samples if s["src"] == "FIX" and s["dp"] is not None]
    rms_dp = math.sqrt(sum(v * v for v in fix_dp) / len(fix_dp)) if fix_dp else None
    max_dp = max(fix_dp) if fix_dp else None

    transitions = []
    for idx in range(1, len(samples)):
        prev_s = samples[idx - 1]
        cur_s = samples[idx]
        if prev_s["src"] in ("FIX", "FLT") and cur_s["src"] == "CMP":
            transitions.append(abs(wrap_deg_180(cur_s["hdg"] - prev_s["hdg"])))

    max_switch_jump = max(transitions) if transitions else 0.0

    checks = {
        "i2c1_active": i2c1_count == len(samples),
        "zero_i2c0_err": max_i2c0_err == 0,
        "hdg_accuracy": (rms_dh is not None and rms_dh < max_hdg_err_deg),
        "pitch_agreement": (rms_dp is not None and rms_dp <= max_pitch_err_deg),
        "switch_jump": max_switch_jump < max_jump_deg,
    }
    passed = all(checks.values())
    return {
        "passed": passed,
        "checks": checks,
        "sample_count": len(samples),
        "i2c1_count": i2c1_count,
        "max_i2c0_err": max_i2c0_err,
        "max_i2c1_err": max_i2c1_err,
        "fix_samples": len(fix_dh),
        "rms_dh_deg": rms_dh,
        "max_dh_deg": max_dh,
        "rms_dp_deg": rms_dp,
        "max_dp_deg": max_dp,
        "fix_to_cmp_count": len(transitions),
        "max_switch_jump_deg": max_switch_jump,
    }


def run_self_test():
    good_log = [
        "[10DOF I2C1] P/R:+0.4/-0.2 deg | |B|:52.4uT | Hdg:120.0[FIX] Mag:120.8 | FOC:OK | P:1012.30hPa | dH:+0.8 dP:+0.2 | I2C:0/0",
        "[10DOF I2C1] P/R:+0.5/-0.1 deg | |B|:52.3uT | Hdg:120.2[FIX] Mag:119.5 | FOC:OK | P:1012.31hPa | dH:-0.7 dP:+0.3 | I2C:0/0",
        "[10DOF I2C1] P/R:+0.4/-0.1 deg | |B|:52.4uT | Hdg:120.9[CMP] Mag:121.0 | FOC:OK | P:1012.30hPa | I2C:0/0",
    ]
    res_good = evaluate_g1_samples(parse_g1_lines(good_log))
    assert res_good["passed"], f"Expected passing G1 self-test, got {res_good}"
    assert res_good["fix_to_cmp_count"] == 1
    assert res_good["max_switch_jump_deg"] < 1.0

    bad_log = [
        "[10DOF I2C1] P/R:+3.5/-0.2 deg | |B|:52.4uT | Hdg:120.0[FIX] Mag:126.0 | FOC:OK | P:1012.30hPa | dH:+6.0 dP:+2.5 | I2C:2/0",
        "[10DOF I2C1] P/R:+3.5/-0.2 deg | |B|:52.4uT | Hdg:126.0[CMP] Mag:126.0 | FOC:OK | P:1012.30hPa | I2C:2/0",
    ]
    res_bad = evaluate_g1_samples(parse_g1_lines(bad_log))
    assert not res_bad["passed"]
    assert not res_bad["checks"]["zero_i2c0_err"]
    assert not res_bad["checks"]["hdg_accuracy"]
    assert not res_bad["checks"]["pitch_agreement"]
    assert not res_bad["checks"]["switch_jump"]
    print("[PASS] verify_g1_outdoor_soak.py self-test (2/2) passed.")
    return 0


def main():
    print(f"=== {__filename__} {__version__} ===")
    parser = argparse.ArgumentParser(description="Score RTK_Rover Gate G1 outdoor I2C1/compass soak logs.")
    parser.add_argument("logfile", nargs="?", help="Path to Rover serial log file.")
    parser.add_argument("--self-test", action="store_true", help="Run internal parser/evaluator self-tests.")
    args = parser.parse_args()

    if args.self_test:
        return run_self_test()
    if not args.logfile:
        parser.error("Provide a serial log path or --self-test.")

    with open(args.logfile, "r", encoding="utf-8", errors="replace") as fh:
        samples = parse_g1_lines(fh)
    res = evaluate_g1_samples(samples)
    if res["sample_count"] == 0:
        print(f"[FAIL] {res['reason']}")
        return 1

    print(
        f"Samples: {res['sample_count']} (I2C1: {res['i2c1_count']}) | "
        f"I2C Errs (0/1): {res['max_i2c0_err']}/{res['max_i2c1_err']}"
    )
    print(
        f"HDG_FIX RMS dH: {res['rms_dh_deg']:.2f} deg (max {res['max_dh_deg']:.2f}) | "
        f"RMS dP: {res['rms_dp_deg']:.2f} deg (max {res['max_dp_deg']:.2f})"
        if res["rms_dh_deg"] is not None and res["rms_dp_deg"] is not None
        else f"HDG_FIX samples: {res['fix_samples']} (insufficient for dH/dP)"
    )
    print(
        f"FIX/FLT -> CMP switches: {res['fix_to_cmp_count']} | "
        f"Max jump: {res['max_switch_jump_deg']:.2f} deg"
    )
    print(f"Gate G1 Verdict: {'PASS' if res['passed'] else 'FAIL'} ({res['checks']})")
    return 0 if res["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
