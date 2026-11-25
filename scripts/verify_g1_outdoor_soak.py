#!/usr/bin/env python3
"""
Gate G1 Outdoor I2C1 & Beam Compass Soak Verifier for RTK_Rover.

Parses Rover serial capture logs containing [10DOF I2C1] (or [10DOF I2C0]),
[CAL MAG] OK, and [I2C0 RECOVER] lines and evaluates Plan 2 Gate G1 criteria:
  1. I2C1 active across 100% of samples and 0 I2C1 errors
  2. Unresolved I2C0 stall tracking (delta I2C0/I2C1 and runtime bus-clear counts)
  3. < 2.0 deg static RMS beam compass error vs UM982 HDG_FIX (post-CAL MAG)
  4. <= 1.0 deg static RMS pitch agreement (|dP| <= 1.0 deg, post-CAL MAG)
  5. 0.0 s HDG_FIX -> [CMP] transition with < 2.0 deg heading jump
"""

import argparse
import math
import os
import re
import statistics
import sys

__filename__ = os.path.basename(__file__)
__version__ = "v1.1.0"

RE_10DOF = re.compile(
    r"(?:\[(?P<ts>\d{2}:\d{2}:\d{2}\.\d{3})\]\s*)?"
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


def parse_g1_log(lines):
    samples = []
    last_cal_mag_idx = -1
    i2c0_recover_count = 0
    i2c0_warn_count = 0

    for idx, raw in enumerate(lines):
        if "[CAL MAG] OK" in raw:
            last_cal_mag_idx = idx
        if "[I2C0 RECOVER] Runtime bus-clear complete" in raw:
            i2c0_recover_count += 1
        elif "[I2C0 WARN]" in raw:
            i2c0_warn_count += 1

        m = RE_10DOF.search(raw)
        if not m:
            continue
        hdg = float(m.group("hdg"))
        mag = float(m.group("mag"))
        dh = float(m.group("dh")) if m.group("dh") is not None else wrap_deg_180(mag - hdg)
        dp = float(m.group("dp")) if m.group("dp") is not None else None
        samples.append({
            "line_idx": idx,
            "ts": m.group("ts") or "",
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

    return {
        "samples": samples,
        "last_cal_mag_idx": last_cal_mag_idx,
        "i2c0_recover_count": i2c0_recover_count,
        "i2c0_warn_count": i2c0_warn_count,
    }


def parse_g1_lines(lines):
    return parse_g1_log(lines)["samples"]


def _quadrant_stats(static_pairs):
    buckets = [
        ("N (315-45)", 315.0, 45.0),
        ("E (45-135)", 45.0, 135.0),
        ("S (135-225)", 135.0, 225.0),
        ("W (225-315)", 225.0, 315.0),
    ]
    out = {}
    for label, lo, hi in buckets:
        vals_h = []
        vals_p = []
        for s in static_pairs:
            in_q = (s["hdg"] >= lo or s["hdg"] < hi) if lo > hi else (lo <= s["hdg"] < hi)
            if in_q:
                vals_h.append(s["dh"])
                if s["dp"] is not None:
                    vals_p.append(s["dp"])
        if vals_h:
            out[label] = {
                "n": len(vals_h),
                "rms_dh": math.sqrt(sum(v * v for v in vals_h) / len(vals_h)),
                "rms_dp": math.sqrt(sum(v * v for v in vals_p) / len(vals_p)) if vals_p else None,
            }
    return out


def evaluate_g1_samples(
    samples_or_parsed,
    max_hdg_err_deg=2.0,
    max_pitch_err_deg=1.0,
    max_jump_deg=2.0,
):
    if isinstance(samples_or_parsed, dict):
        samples = samples_or_parsed["samples"]
        last_cal_mag_idx = samples_or_parsed.get("last_cal_mag_idx", -1)
        i2c0_recover_count = samples_or_parsed.get("i2c0_recover_count", 0)
        i2c0_warn_count = samples_or_parsed.get("i2c0_warn_count", 0)
    else:
        samples = samples_or_parsed
        last_cal_mag_idx = -1
        i2c0_recover_count = 0
        i2c0_warn_count = 0

    if not samples:
        return {
            "passed": False,
            "reason": "No [10DOF I2C*] samples found in log.",
            "sample_count": 0,
        }

    i2c1_count = sum(1 for s in samples if s["bus"] == 1)
    i2c0_errs = [s["i2c0_err"] for s in samples if s["i2c0_err"] is not None]
    i2c1_errs = [s["i2c1_err"] for s in samples if s["i2c1_err"] is not None]
    delta_i2c0_err = (i2c0_errs[-1] - i2c0_errs[0]) if len(i2c0_errs) >= 2 else (i2c0_errs[0] if i2c0_errs else 0)
    delta_i2c1_err = (i2c1_errs[-1] - i2c1_errs[0]) if len(i2c1_errs) >= 2 else (i2c1_errs[0] if i2c1_errs else 0)
    max_i2c0_err = max(i2c0_errs) if i2c0_errs else 0
    max_i2c1_err = max(i2c1_errs) if i2c1_errs else 0

    # Slice accuracy/switchover evaluation to post-CAL MAG samples if CAL MAG ran in this log
    eval_samples = [s for s in samples if s.get("line_idx", 0) > last_cal_mag_idx] if last_cal_mag_idx >= 0 else samples
    if not eval_samples:
        eval_samples = samples

    all_fix_dh = [abs(s["dh"]) for s in eval_samples if s["src"] == "FIX" and not s["anom"]]
    all_rms_dh = math.sqrt(sum(v * v for v in all_fix_dh) / len(all_fix_dh)) if all_fix_dh else None
    all_fix_dp = [abs(s["dp"]) for s in eval_samples if s["src"] == "FIX" and s["dp"] is not None]
    all_rms_dp = math.sqrt(sum(v * v for v in all_fix_dp) / len(all_fix_dp)) if all_fix_dp else None

    static_samples = []
    for idx in range(1, len(eval_samples)):
        prev_s = eval_samples[idx - 1]
        cur_s = eval_samples[idx]
        if prev_s["src"] == "FIX" and cur_s["src"] == "FIX" and not prev_s["anom"] and not cur_s["anom"]:
            d_hdg = abs(wrap_deg_180(cur_s["hdg"] - prev_s["hdg"]))
            d_pitch = abs(cur_s["pitch"] - prev_s["pitch"])
            if d_hdg < 1.5 and d_pitch < 1.0 and abs(cur_s["pitch"]) < 15.0 and abs(cur_s["roll"]) < 15.0:
                static_samples.append(cur_s)

    if static_samples:
        stat_dh = [abs(s["dh"]) for s in static_samples]
        stat_dp = [abs(s["dp"]) for s in static_samples if s["dp"] is not None]
        rms_dh = math.sqrt(sum(v * v for v in stat_dh) / len(stat_dh))
        med_dh = statistics.median(stat_dh)
        max_dh = max(stat_dh)
        rms_dp = math.sqrt(sum(v * v for v in stat_dp) / len(stat_dp)) if stat_dp else None
        med_dp = statistics.median(stat_dp) if stat_dp else None
        max_dp = max(stat_dp) if stat_dp else None
    else:
        rms_dh = all_rms_dh
        med_dh = statistics.median(all_fix_dh) if all_fix_dh else None
        max_dh = max(all_fix_dh) if all_fix_dh else None
        rms_dp = all_rms_dp
        med_dp = statistics.median(all_fix_dp) if all_fix_dp else None
        max_dp = max(all_fix_dp) if all_fix_dp else None

    fix_to_cmp = []
    flt_to_cmp = []
    for idx in range(1, len(eval_samples)):
        prev_s = eval_samples[idx - 1]
        cur_s = eval_samples[idx]
        if cur_s["src"] == "CMP":
            jump = abs(wrap_deg_180(cur_s["hdg"] - prev_s["hdg"]))
            if prev_s["src"] == "FIX":
                fix_to_cmp.append(jump)
            elif prev_s["src"] == "FLT":
                flt_to_cmp.append(jump)

    max_fix_jump = max(fix_to_cmp) if fix_to_cmp else 0.0
    max_flt_jump = max(flt_to_cmp) if flt_to_cmp else 0.0

    # Gate G1 checks: I2C1 100%, 0 I2C1 errs, 0 I2C0 delta errs, static dH < 2.0, static dP <= 1.0, FIX->CMP jump < 2.0
    checks = {
        "i2c1_active": (i2c1_count == len(samples)) and (delta_i2c1_err == 0),
        "zero_i2c0_err": delta_i2c0_err == 0,
        "hdg_accuracy": (rms_dh is not None and rms_dh < max_hdg_err_deg),
        "pitch_agreement": (rms_dp is not None and rms_dp <= max_pitch_err_deg),
        "switch_jump": max_fix_jump < max_jump_deg,
    }
    passed = all(checks.values())
    return {
        "passed": passed,
        "checks": checks,
        "sample_count": len(samples),
        "eval_sample_count": len(eval_samples),
        "post_cal_mag": last_cal_mag_idx >= 0,
        "i2c1_count": i2c1_count,
        "delta_i2c0_err": delta_i2c0_err,
        "delta_i2c1_err": delta_i2c1_err,
        "max_i2c0_err": max_i2c0_err,
        "max_i2c1_err": max_i2c1_err,
        "i2c0_recover_count": i2c0_recover_count,
        "i2c0_warn_count": i2c0_warn_count,
        "fix_samples": len(all_fix_dh),
        "static_fix_samples": len(static_samples),
        "rms_dh_deg": rms_dh,
        "med_dh_deg": med_dh,
        "max_dh_deg": max_dh,
        "all_rms_dh_deg": all_rms_dh,
        "rms_dp_deg": rms_dp,
        "med_dp_deg": med_dp,
        "max_dp_deg": max_dp,
        "all_rms_dp_deg": all_rms_dp,
        "quadrants": _quadrant_stats(static_samples),
        "fix_to_cmp_count": len(fix_to_cmp),
        "max_fix_jump_deg": max_fix_jump,
        "flt_to_cmp_count": len(flt_to_cmp),
        "max_flt_jump_deg": max_flt_jump,
        "max_switch_jump_deg": max(max_fix_jump, max_flt_jump),
    }


def run_self_test():
    good_log = [
        "[10DOF I2C1] P/R:+0.0/+0.0 deg | |B|:40.0uT | Hdg:120.0[FIX] Mag:145.0 | FOC:OK | P:1012.30hPa | dH:+25.0 dP:+4.0 | I2C:5/0",
        "[CAL MAG] OK [3D] (120 pts): Center=(+1.0,-2.0,+0.5)uT | |B|=52.4uT | RMS=1.20%",
        "[10DOF I2C1] P/R:+0.4/-0.2 deg | |B|:52.4uT | Hdg:120.0[FIX] Mag:120.8 | FOC:OK | P:1012.30hPa | dH:+0.8 dP:+0.2 | I2C:5/0",
        "[10DOF I2C1] P/R:+0.5/-0.1 deg | |B|:52.3uT | Hdg:120.2[FIX] Mag:119.5 | FOC:OK | P:1012.31hPa | dH:-0.7 dP:+0.3 | I2C:5/0",
        # Fast dynamic rotation: excluded from static RMS
        "[10DOF I2C1] P/R:+0.5/-0.1 deg | |B|:52.3uT | Hdg:150.0[FIX] Mag:165.0 | FOC:OK | P:1012.31hPa | dH:+15.0 dP:+0.3 | I2C:5/0",
        "[10DOF I2C1] P/R:+0.4/-0.1 deg | |B|:52.4uT | Hdg:150.8[CMP] Mag:151.0 | FOC:OK | P:1012.30hPa | I2C:5/0",
    ]
    res_good = evaluate_g1_samples(parse_g1_log(good_log))
    assert res_good["passed"], f"Expected passing G1 self-test, got {res_good}"
    assert res_good["post_cal_mag"]
    assert res_good["delta_i2c0_err"] == 0
    assert res_good["static_fix_samples"] == 1
    assert res_good["fix_to_cmp_count"] == 1
    assert res_good["max_fix_jump_deg"] < 1.0

    bad_log = [
        "[10DOF I2C1] P/R:+3.5/-0.2 deg | |B|:52.4uT | Hdg:120.0[FIX] Mag:126.0 | FOC:OK | P:1012.30hPa | dH:+6.0 dP:+2.5 | I2C:1/0",
        "[10DOF I2C1] P/R:+3.5/-0.2 deg | |B|:52.4uT | Hdg:120.2[FIX] Mag:126.2 | FOC:OK | P:1012.30hPa | dH:+6.0 dP:+2.5 | I2C:3/0",
        "[10DOF I2C1] P/R:+3.5/-0.2 deg | |B|:52.4uT | Hdg:126.0[CMP] Mag:126.0 | FOC:OK | P:1012.30hPa | I2C:3/0",
    ]
    res_bad = evaluate_g1_samples(parse_g1_log(bad_log))
    assert not res_bad["passed"]
    assert res_bad["delta_i2c0_err"] == 2
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
        parsed = parse_g1_log(fh)
    res = evaluate_g1_samples(parsed)
    if res["sample_count"] == 0:
        print(f"[FAIL] {res['reason']}")
        return 1

    scope = "post-CAL-MAG" if res["post_cal_mag"] else "full-log"
    print(
        f"Samples: {res['sample_count']} (I2C1: {res['i2c1_count']}, eval {res['eval_sample_count']} {scope}) | "
        f"Delta I2C Errs (0/1): +{res['delta_i2c0_err']}/+{res['delta_i2c1_err']} "
        f"(max {res['max_i2c0_err']}/{res['max_i2c1_err']}, bus-clear={res['i2c0_recover_count']})"
    )
    if res["rms_dh_deg"] is not None and res["rms_dp_deg"] is not None:
        print(
            f"Static FIX (n={res['static_fix_samples']}/{res['fix_samples']}): "
            f"RMS dH={res['rms_dh_deg']:.2f} deg (med {res['med_dh_deg']:.2f}, max {res['max_dh_deg']:.2f}; all={res['all_rms_dh_deg']:.2f}) | "
            f"RMS dP={res['rms_dp_deg']:.2f} deg (med {res['med_dp_deg']:.2f}, max {res['max_dp_deg']:.2f}; all={res['all_rms_dp_deg']:.2f})"
        )
    else:
        print(f"HDG_FIX samples: {res['fix_samples']} (insufficient for dH/dP)")

    if res["quadrants"]:
        q_parts = [f"{q}: dH={v['rms_dh']:.2f} dP={v['rms_dp']:.2f}(n={v['n']})" for q, v in res["quadrants"].items()]
        print("Quadrants: " + " | ".join(q_parts))

    print(
        f"Switches: FIX->CMP={res['fix_to_cmp_count']} (max {res['max_fix_jump_deg']:.2f} deg) | "
        f"FLT->CMP={res['flt_to_cmp_count']} (max {res['max_flt_jump_deg']:.2f} deg)"
    )
    print(f"Gate G1 Verdict: {'PASS' if res['passed'] else 'FAIL'} ({res['checks']})")
    return 0 if res["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
