#!/usr/bin/env python3
"""
analyze_soak.py - HARP Step 1b analysis of an imu_log.py stationary soak.

Reports, per sensor (gyro Z only - the heading axis):
  - logging health (rows, I2C errors, effective rate, worst gap)
  - warm-up time (die temperature and bias settling)
  - bias tempco from a linear fit of bias vs die temperature, and drift left after compensation
  - Allan deviation: noise density (ARW) and bias instability, post warm-up
  - predicted heading drift over a 30 s segment
And for the pair: correlation of bias drifts and the resulting fusion gain.

Usage:  python3 analyze_soak.py logs/soak_cold_YYYYMMDD_HHMMSS.csv [--no-plot]
"""
import argparse, os, sys
import numpy as np

GYRO_LSB = 65.5
DATASHEET = {"mpu": {"tempco": 0.16, "nd": 0.005}, "icm": {"tempco": 0.05, "nd": 0.015}}
BIN_S = 60.0        # bias/temperature averaging bin
SEG_S = 30.0        # drive segment length for drift prediction
SEG_DT = 0.5        # assumed die-temperature change over a segment, deg C


def load(path):
    with open(path) as f:
        lines = [ln for ln in f if not ln.startswith("#")]
    hdr = lines[0].strip().split(",")
    rows = [ln.strip().split(",") for ln in lines[1:] if ln.strip()]
    col = {h: i for i, h in enumerate(hdr)}
    err = np.array([int(r[col["err"]]) for r in rows])
    good = [r for r, e in zip(rows, err) if e == 0]
    t = np.array([int(r[col["t_ns"]]) for r in good], dtype=np.float64) * 1e-9
    t_all = np.array([int(r[col["t_ns"]]) for r in rows], dtype=np.float64) * 1e-9
    out = {"t": t - t[0], "t_all": t_all, "n_rows": len(rows), "n_err": int(err.sum())}
    for s, tc in (("mpu", lambda r: r / 340.0 + 36.53), ("icm", lambda r: r / 333.87 + 21.0)):
        if f"{s}_gz" in col:
            gz = np.array([int(r[col[f"{s}_gz"]]) for r in good], dtype=np.float64) / GYRO_LSB
            tr = np.array([int(r[col[f"{s}_temp"]]) for r in good], dtype=np.float64)
            out[s] = {"gz": gz, "T": tc(tr)}
    return out


def binned(t, x, w):
    edges = np.arange(0, t[-1] + w, w)
    idx = np.digitize(t, edges) - 1
    n = len(edges) - 1
    m = np.array([x[idx == i].mean() for i in range(n) if np.any(idx == i)])
    tc = np.array([t[idx == i].mean() for i in range(n) if np.any(idx == i)])
    return tc, m


def settle_time(tc, y, tol):
    """First bin time after which y stays within tol of its final value (mean of last 5 bins)."""
    final = y[-5:].mean()
    ok = np.abs(y - final) <= tol
    for i in range(len(ok)):
        if ok[i:].all():
            return tc[i]
    return float("nan")


def adev(rate, dt, taus):
    theta = np.cumsum(rate) * dt
    N = len(theta)
    res = []
    for tau in taus:
        m = int(round(tau / dt))
        if m < 1 or 2 * m >= N:
            continue
        d = theta[2 * m:] - 2 * theta[m:N - m] + theta[:N - 2 * m]
        res.append((m * dt, np.sqrt(np.mean(d * d) / (2 * (m * dt) ** 2))))
    return np.array(res)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--no-plot", action="store_true")
    a = ap.parse_args()
    d = load(a.csv)
    t = d["t"]
    gaps = np.diff(d["t_all"])
    rate = (len(d["t_all"]) - 1) / (d["t_all"][-1] - d["t_all"][0])
    print(f"File: {os.path.basename(a.csv)}")
    print(f"Duration {t[-1]:.0f} s | rows {d['n_rows']} | I2C errors {d['n_err']} | "
          f"rate {rate:.1f} Hz | worst gap {gaps.max()*1000:.1f} ms")

    sensors = [s for s in ("mpu", "icm") if s in d]
    res = {}
    for s in sensors:
        gz, T = d[s]["gz"], d[s]["T"]
        tc, b = binned(t, gz, BIN_S)
        _, Tb = binned(t, T, BIN_S)
        t_T = settle_time(tc, Tb, 0.2)
        t_b = settle_time(tc, b, 0.01)
        slope, icpt = np.polyfit(Tb, b, 1)
        resid = b - (slope * Tb + icpt)
        r2 = 1 - np.sum(resid ** 2) / np.sum((b - b.mean()) ** 2) if np.ptp(b) > 0 else float("nan")
        warm = t >= max(t_b if np.isfinite(t_b) else 0, 0)
        dt = np.median(np.diff(t))
        taus = np.logspace(np.log10(dt), np.log10(t[warm][-1] - t[warm][0]) - 0.6, 30)
        ad = adev(gz[warm] - gz[warm].mean(), dt, taus)
        i1 = np.argmin(np.abs(np.log(ad[:, 0])))          # tau closest to 1 s
        adev1 = ad[i1, 1] * np.sqrt(ad[i1, 0])            # ADEV scaled to tau = 1 s (= ARW, dps/rt-s)
        nd = adev1 * np.sqrt(2)                           # one-sided noise density, comparable to datasheet
        bi = ad[:, 1].min() / 0.664
        tau_bi = ad[np.argmin(ad[:, 1]), 0]
        drift_raw = abs(slope) * SEG_DT * SEG_S
        drift_comp = resid.std() * SEG_S
        res[s] = dict(slope=slope, resid=resid, tc=tc, b=b, Tb=Tb, ad=ad, r2=r2)
        ds = DATASHEET[s]
        print(f"\n[{s.upper()}]  gyro Z")
        print(f"  Die temp: {Tb[0]:.2f} -> {Tb[-1]:.2f} C (rise {Tb[-1]-Tb[0]:.2f} C); settles (+/-0.2 C) at {t_T/60:.1f} min")
        print(f"  Bias: {b[0]:+.4f} -> {b[-1]:+.4f} dps (span {np.ptp(b):.4f}); settles (+/-0.01 dps) at {t_b/60:.1f} min")
        print(f"  Tempco (linear fit): {slope:+.4f} dps/C  R^2={r2:.2f}   [datasheet +/-{ds['tempco']}]")
        print(f"  Bias left after linear T-compensation: {resid.std():.4f} dps rms")
        print(f"  Noise density (ADEV @1 s): {nd:.4f} dps/rtHz   [datasheet {ds['nd']}]")
        print(f"  Bias instability: {bi:.4f} dps ({bi*3600:.1f} deg/h) at tau={tau_bi:.0f} s")
        print(f"  30 s segment drift: uncompensated {drift_raw:.2f} deg (dT={SEG_DT} C), "
              f"T-compensated {drift_comp:.2f} deg")
        print(f"  ARW: {adev1:.4f} deg/rt-s ({adev1*60:.2f} deg/rt-h)")
        print(f"  ZUPT stop to hold bias to 0.01 dps: {(adev1/0.01)**2:.2f} s")

    if len(sensors) == 2:
        rho_raw = np.corrcoef(res["mpu"]["b"], res["icm"]["b"])[0, 1]
        rho_res = np.corrcoef(res["mpu"]["resid"], res["icm"]["resid"])[0, 1]
        s1, s2 = res["mpu"]["resid"].std(), res["icm"]["resid"].std()
        var_c = (s1**2 * s2**2 * (1 - rho_res**2)) / (s1**2 + s2**2 - 2 * rho_res * s1 * s2)
        gain = min(s1, s2) / np.sqrt(var_c) if var_c > 0 else float("nan")
        ratio = max(s1, s2) / min(s1, s2)
        print(f"\n[PAIR]")
        print(f"  Bias correlation, raw: rho={rho_raw:+.2f}   after T-compensation: rho={rho_res:+.2f}")
        print(f"  Compensated sigma ratio (worse/better): {ratio:.2f}")
        print(f"  Optimal-fusion gain over better sensor alone: {gain:.2f}x")
        verdict = ("FUSE (ratio<1.5 and rho<0.3)" if ratio < 1.5 and abs(rho_res) < 0.3
                   else "USE BETTER SENSOR + T-comp; other as cross-check")
        print(f"  Decision rule -> {verdict}")

    if not a.no_plot:
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            print("\n(matplotlib not installed - skipping plot)")
            return
        fig, ax = plt.subplots(2, 2, figsize=(12, 8))
        for s in sensors:
            r = res[s]
            ax[0, 0].plot(r["tc"] / 60, r["Tb"], label=s)
            ax[0, 1].plot(r["tc"] / 60, r["b"], label=s)
            ax[1, 0].plot(r["Tb"], r["b"], "o", ms=3, label=f"{s} {r['slope']:+.4f} dps/C")
            ax[1, 1].loglog(r["ad"][:, 0], r["ad"][:, 1], label=s)
        ax[0, 0].set(xlabel="min", ylabel="die temp C", title="Temperature")
        ax[0, 1].set(xlabel="min", ylabel="gz bias dps", title=f"Bias ({BIN_S:.0f} s bins)")
        ax[1, 0].set(xlabel="die temp C", ylabel="gz bias dps", title="Bias vs temperature")
        ax[1, 1].set(xlabel="tau s", ylabel="ADEV dps", title="Allan deviation (post warm-up)")
        for x in ax.flat:
            x.grid(True, which="both", alpha=0.3); x.legend()
        fig.tight_layout()
        png = os.path.splitext(a.csv)[0] + ".png"
        fig.savefig(png, dpi=110)
        print(f"\nPlot: {png}")


if __name__ == "__main__":
    sys.exit(main())