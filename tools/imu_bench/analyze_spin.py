#!/usr/bin/env python3
"""
analyze_spin.py - HARP Step 1c gyro scale-factor calibration from a multi-turn spin.

Log layout expected (imu_log.py):  >=8 s stationary | spin | >=8 s stationary
Both boards are mounted Z-down, so yaw rate (REP-103, CCW +) = -gz.

Bias = mean gz over the stationary segments (pre and post averaged; their difference
is reported as in-run bias change). Yaw is integrated with the Pi's timestamps
(trapezoid), never an assumed sample period.

Usage:
  python3 analyze_spin.py logs/ccw10_....csv --truth 3598.4
     --truth = true rotation from the floor reference, degrees, CCW positive (CW negative)
  Without --truth it just reports the integrated angles.
"""
import argparse, io
import numpy as np

GYRO_LSB = 65.5
SAT = 32767
YAW_SIGN = {"mpu": -1.0, "icm": -1.0}   # Z-down mounting on both boards
STILL_S = 8.0                            # seconds of each end used for bias


def load(path):
    txt = "".join(l for l in open(path) if not l.startswith("#"))
    d = np.genfromtxt(io.StringIO(txt), delimiter=",", names=True)
    d = d[d["err"] == 0]
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--truth", type=float, help="true rotation, deg, CCW +")
    a = ap.parse_args()
    d = load(a.csv)
    t = (d["t_ns"] - d["t_ns"][0]) * 1e-9
    print(f"File: {a.csv}   duration {t[-1]:.1f} s, {len(t)} good rows")
    pre, post = t <= STILL_S, t >= t[-1] - STILL_S

    for s in ("mpu", "icm"):
        if f"{s}_gz" not in d.dtype.names:
            continue
        raw = d[f"{s}_gz"]
        gz = raw / GYRO_LSB
        b_pre, b_post = gz[pre].mean(), gz[post].mean()
        bias = 0.5 * (b_pre + b_post)
        yaw_rate = YAW_SIGN[s] * (gz - bias)
        # sanity: ends must really be still
        still_pk = max(np.abs(gz[pre] - b_pre).max(), np.abs(gz[post] - b_post).max())
        ang = np.concatenate([[0.0], np.cumsum(0.5 * (yaw_rate[1:] + yaw_rate[:-1]) * np.diff(t))])
        total = ang[-1]
        nsat = int(np.sum(np.abs(raw) >= SAT - 1))
        pk = np.abs(yaw_rate).max()
        # bias uncertainty contribution over the spin duration
        moving = np.abs(yaw_rate) > 5.0
        t_move = (t[moving][-1] - t[moving][0]) if moving.any() else 0.0
        bias_err_deg = abs(b_post - b_pre) * 0.5 * t_move
        print(f"\n[{s.upper()}]")
        print(f"  Bias pre {b_pre:+.4f} / post {b_post:+.4f} dps (change {b_post-b_pre:+.4f}); "
              f"end-segment peak dev {still_pk:.2f} dps {'OK' if still_pk < 1.0 else '<-- ends not still?'}")
        print(f"  Peak yaw rate {pk:.1f} dps; saturated samples {nsat} {'OK' if nsat == 0 else '<-- CLIPPED, result invalid'}")
        print(f"  Moving time {t_move:.1f} s; bias-change error bound ~{bias_err_deg:.2f} deg")
        print(f"  Integrated yaw: {total:+.2f} deg  ({total/360:+.3f} turns)")
        if a.truth is not None:
            if np.sign(total) != np.sign(a.truth):
                print("  !! sign mismatch between gyro and --truth: check direction / YAW_SIGN")
            k = a.truth / total
            print(f"  Truth {a.truth:+.2f} deg  ->  scale factor k = {k:.5f}  (gyro error {(1/k - 1)*100:+.3f}%)")


if __name__ == "__main__":
    main()
