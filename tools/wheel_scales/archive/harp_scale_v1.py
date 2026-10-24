#!/usr/bin/env python3
"""
harp_scale.py — HARP wheel-scale host tool, v1 (2026-10-04)

Talks to the wheel-scale ESP32 (harp_scales_fw v1) over USB serial. The
firmware streams raw HX711 counts; tare and calibration live here, in a JSON
file (default scale_cal.json, keyed by scale ID).

Scale IDs: 1 left drive (left front), 2 right drive, 3 left caster, 4 right caster.
Default is --scales 1.

First session, one scale (maps to carry-in §5 acceptance tests):
  python3 harp_scale.py info               # firmware alive, scale detected?
  python3 harp_scale.py monitor            # raw counts live; Ctrl-C to stop
  python3 harp_scale.py polarity           # test 1
  python3 harp_scale.py noise              # feasibility check: ~1 g rms expected
  python3 harp_scale.py cal                # test 3 (calibration + linearity)
  python3 harp_scale.py read               # averaged kg / lb
  python3 harp_scale.py repeat --mass 5    # test 4
  python3 harp_scale.py drift              # test 2 (10 min warm-up + 10 min)
  python3 harp_scale.py read --zero        # zero-return check (within 20 g)
Add --log results.csv --note "text" (before the command) to log any of these.
"""
import argparse
import csv
import datetime
import glob
import json
import os
import statistics
import sys
import time

try:
    import serial
except ImportError:
    sys.exit("pyserial missing: sudo apt install python3-serial   (or: pip install pyserial)")

NAMES = {1: "left drive", 2: "right drive", 3: "left caster", 4: "right caster"}
LB_PER_KG = 2.20462
ZERO_TOL_KG = 0.020       # zero-return check, carry-in §4
REPEAT_TOL_KG = 0.020     # repeatability spread, §5 test 4
LINEARITY_TOL_PCT = 0.5   # §5 test 3
# Design estimate: 128 gain x (1 mV/V per 50 kg) x 2^24 counts per V/V.
# Ratiometric, so independent of the actual excitation voltage.
EXPECTED_CPK = 128 * (1e-3 / 50) * 2**24   # ~42,950 counts/kg
EXPECTED_NOISE_COUNTS = 36                 # 50 nV rms / (11.7 mV / 2^23) at 3.0 V


def now():
    return datetime.datetime.now().isoformat(timespec="seconds")


def mean_std(xs):
    m = statistics.fmean(xs)
    s = statistics.stdev(xs) if len(xs) > 1 else 0.0
    return m, s


def find_port():
    ports = sorted(glob.glob("/dev/ttyUSB*")) + sorted(glob.glob("/dev/ttyACM*"))
    if not ports:
        sys.exit("No /dev/ttyUSB* or /dev/ttyACM* found. Check the cable is data-capable "
                 "(see the ESP32 power record) and `sudo dmesg | tail`.")
    if len(ports) > 1:
        print(f"# several ports {ports}; using {ports[0]} (override with --port)")
    return ports[0]


# ---------------------------------------------------------------- serial link
class Link:
    def __init__(self, port, baud=115200):
        try:
            self.ser = serial.Serial(port, baud, timeout=0.2)
        except serial.SerialException as e:
            sys.exit(f"Cannot open {port}: {e}\n"
                     "Not in the dialout group (groups | grep dialout)? "
                     "Or a PlatformIO serial monitor still has the port open?")
        self.present = None
        self._get_banner()

    def _line(self):
        raw = self.ser.readline()
        return raw.decode(errors="replace").strip() if raw else None

    def _get_banner(self):
        # Opening the port normally resets the DevKitC, so the boot banner
        # arrives on its own. If not (no auto-reset), ask for it with 'i'.
        for wait_s in (6.0, 2.0):
            t_end = time.time() + wait_s
            while time.time() < t_end:
                line = self._line()
                if line and line.startswith("# present:"):
                    self.present = [int(x) for x in line.split(":", 1)[1].split(",")]
                    return
            self.ser.write(b"i\n")
        sys.exit("No banner from the firmware. Is harp_scales_fw flashed? "
                 "(cd harp_scales_fw && pio run -t upload)")

    @staticmethod
    def parse(line):
        """'t_ms,r1,r2,r3,r4' -> (t_ms, {id: raw}); None for anything else."""
        if not line or line.startswith("#"):
            return None
        p = line.split(",")
        if len(p) != 5:
            return None
        try:
            return int(p[0]), {i: int(v) for i, v in enumerate(p[1:], 1) if v != ""}
        except ValueError:
            return None

    def collect(self, ids, seconds):
        """Fresh samples for `seconds`; returns {id: [raw, ...]}."""
        self.ser.reset_input_buffer()
        self._line()  # discard a possible partial line
        out = {i: [] for i in ids}
        t_end = time.time() + seconds
        while time.time() < t_end:
            d = self.parse(self._line())
            if d:
                for i in ids:
                    if i in d[1]:
                        out[i].append(d[1][i])
        for i in ids:
            if not out[i]:
                sys.exit(f"Scale {i}: no samples in {seconds:.1f} s "
                         "(not detected at boot? DT/SCK swapped? try the 'info' command)")
        return out

    def stream(self):
        while True:
            d = self.parse(self._line())
            if d:
                yield d


# ---------------------------------------------------------------- calibration
class Cal:
    def __init__(self, path):
        self.path = path
        self.d = {}
        if os.path.exists(path):
            with open(path) as fh:
                self.d = json.load(fh)

    def get(self, i):
        return self.d.get(str(i), {})

    def has_cal(self, i):
        c = self.get(i)
        return "counts_per_kg" in c and "offset" in c

    def kg(self, i, raw):
        c = self.get(i)
        return (raw - c["offset"]) / c["counts_per_kg"]

    def g_per_count(self, i):
        return 1000.0 / abs(self.get(i)["counts_per_kg"])

    def update(self, i, **kw):
        self.d.setdefault(str(i), {}).update(kw)
        tmp = self.path + ".tmp"
        with open(tmp, "w") as fh:
            json.dump(self.d, fh, indent=2)
        os.replace(tmp, self.path)


def require_cal(C, ids):
    missing = [i for i in ids if not C.has_cal(i)]
    if missing:
        sys.exit(f"Scale(s) {missing} not calibrated. Run: harp_scale.py cal --scales "
                 + " ".join(map(str, missing)))


# ---------------------------------------------------------------- CSV log
class Log:
    HEADER = ["timestamp", "mode", "scale", "kg", "std_g", "n", "raw_mean", "extra", "note"]

    def __init__(self, path, note):
        self.path, self.note = path, note or ""

    def row(self, mode, scale, kg="", std_g="", n="", raw_mean="", extra=""):
        if not self.path:
            return
        new = not os.path.exists(self.path)
        with open(self.path, "a", newline="") as fh:
            w = csv.writer(fh)
            if new:
                w.writerow(self.HEADER)
            w.writerow([now(), mode, scale, kg, std_g, n, raw_mean, extra, self.note])


# ---------------------------------------------------------------- commands
def cmd_info(a, L, C, G):
    for i in range(1, 5):
        c = C.get(i)
        state = "calibrated" if C.has_cal(i) else ("tared only" if "offset" in c else "no cal")
        det = "DETECTED" if L.present[i - 1] else "not detected"
        print(f"scale {i} ({NAMES[i]:12s}): {det:12s}  {state}")
        if C.has_cal(i):
            print(f"    counts/kg {c['counts_per_kg']:.1f}  offset {c['offset']:.0f}  "
                  f"({C.g_per_count(i) * 1000:.1f} mg/count)  cal {c.get('cal_time', '?')}")


def cmd_monitor(a, L, C, G):
    print("Ctrl-C to stop.   t_s | per scale: raw count [kg]")
    try:
        for t, vals in L.stream():
            cols = [f"{t / 1000:9.2f}"]
            for i in a.scales:
                if i in vals:
                    s = f"{i}: {vals[i]:9d}"
                    if C.has_cal(i):
                        s += f" {C.kg(i, vals[i]):8.3f} kg"
                else:
                    s = f"{i}: {'--':>9}"
                cols.append(s)
            print("   ".join(cols))
    except KeyboardInterrupt:
        print()


def cmd_tare(a, L, C, G):
    input(f"Remove all load from scale(s) {a.scales} (saddle only), then Enter...")
    data = L.collect(a.scales, a.seconds)
    for i in a.scales:
        m, s = mean_std(data[i])
        C.update(i, offset=m, tare_time=now())
        extra = f" = {s * C.g_per_count(i):.1f} g" if C.has_cal(i) else ""
        print(f"scale {i}: offset {m:.0f} counts, std {s:.0f} counts{extra}  (n={len(data[i])})")
        G.row("tare", i, raw_mean=f"{m:.1f}", n=len(data[i]))


def cmd_cal(a, L, C, G):
    for i in a.scales:
        print(f"\n=== Calibrate scale {i} ({NAMES[i]}) ===")
        input("Remove ALL load (saddle only), hands off, Enter...")
        z = L.collect([i], a.seconds)[i]
        zm, zs = mean_std(z)
        pts = [(0.0, zm)]
        print(f"  zero: {zm:.0f} counts, std {zs:.0f}")
        while True:
            s = input("Place a known mass, centered, hands off. Mass in kg (blank = done): ").strip()
            if not s:
                break
            try:
                m = float(s)
            except ValueError:
                print("  not a number")
                continue
            r = L.collect([i], a.seconds)[i]
            rm, rs = mean_std(r)
            pts.append((m, rm))
            print(f"  {m:.3f} kg -> {rm - zm:+.0f} counts net "
                  f"({(rm - zm) / m if m else 0:.0f} counts/kg), std {rs:.0f}")
        if len(pts) < 2:
            print("  no masses entered; scale not calibrated")
            continue

        # Least-squares line raw = offset + k * mass, zero point included.
        ms = [p[0] for p in pts]
        rs_ = [p[1] for p in pts]
        mb, rb = statistics.fmean(ms), statistics.fmean(rs_)
        sxx = sum((m - mb) ** 2 for m in ms)
        k = sum((m - mb) * (r - rb) for m, r in zip(ms, rs_)) / sxx
        off = rb - k * mb

        print(f"\n  counts/kg = {k:.1f}   ({1000 / abs(k):.3f} g/count)")
        ratio = abs(k) / EXPECTED_CPK
        if not 0.4 < ratio < 2.0:
            print(f"  WARNING: design estimate is ~{EXPECTED_CPK:.0f} counts/kg "
                  f"(you have {ratio:.2f}x). Check wiring, gain, and that the load "
                  "goes only through the cell's center bump.")
        print("  mass_kg   reads_kg   err_g    err_%")
        worst = 0.0
        for m, r in pts:
            got = (r - off) / k
            err_g = (got - m) * 1000
            pct = err_g / (m * 1000) * 100 if m > 0 else 0.0
            worst = max(worst, abs(pct))
            print(f"  {m:7.3f}  {got:9.3f}  {err_g:+7.1f}  {pct:+6.2f}" if m > 0 else
                  f"  {m:7.3f}  {got:9.3f}  {err_g:+7.1f}     --")
        if len(pts) >= 3:
            verdict = "PASS" if worst <= LINEARITY_TOL_PCT else "FAIL"
            print(f"  linearity: worst {worst:.2f}% vs {LINEARITY_TOL_PCT}% -> {verdict}")
        else:
            print("  (one mass only: exact two-point fit, no linearity check; "
                  "use 3+ masses, e.g. 2, 5, 10, 15 kg)")
        C.update(i, offset=off, counts_per_kg=k, cal_time=now(),
                 cal_points=[[m, r] for m, r in pts])
        G.row("cal", i, extra=f"cpk={k:.1f};worst_pct={worst:.2f};points={len(pts)}")
        print(f"  saved to {C.path}")


def cmd_read(a, L, C, G):
    require_cal(C, a.scales)
    data = L.collect(a.scales, a.seconds)
    total = 0.0
    for i in a.scales:
        kgs = [C.kg(i, r) for r in data[i]]
        m, s = mean_std(kgs)
        total += m
        flag = ""
        if a.zero:
            flag = ("zero OK" if abs(m) <= ZERO_TOL_KG
                    else f"FLAG: {m * 1000:+.0f} g, not back within {ZERO_TOL_KG * 1000:.0f} g")
        print(f"scale {i} ({NAMES[i]:12s}): {m:8.3f} kg  {m * LB_PER_KG:8.2f} lb   "
              f"(std {s * 1000:.1f} g, n={len(kgs)})  {flag}")
        G.row("zero" if a.zero else "read", i, kg=f"{m:.4f}", std_g=f"{s * 1000:.1f}",
              n=len(kgs), raw_mean=f"{statistics.fmean(data[i]):.1f}", extra=flag)
    if len(a.scales) > 1:
        print(f"TOTAL{'':20s}: {total:8.3f} kg  {total * LB_PER_KG:8.2f} lb")


def cmd_noise(a, L, C, G):
    data = L.collect(a.scales, a.seconds)
    for i in a.scales:
        r = data[i]
        m, s = mean_std(r)
        sps = len(r) / a.seconds
        print(f"scale {i}: n={len(r)} ({sps:.1f} SPS)  std {s:.1f} counts per sample "
              f"(design estimate ~{EXPECTED_NOISE_COUNTS})  p-p {max(r) - min(r)} counts")
        if C.has_cal(i):
            g = C.g_per_count(i)
            print(f"         = {s * g:.2f} g rms per sample; "
                  f"a 3 s average ~{s * g / (30 ** 0.5):.2f} g")
        if sps > 40:
            print("         NOTE: ~80 SPS means RATE is high; tie RATE low for 10 SPS (lower noise)")
        G.row("noise", i, std_g=f"{s * C.g_per_count(i):.2f}" if C.has_cal(i) else "",
              n=len(r), raw_mean=f"{m:.1f}", extra=f"std_counts={s:.1f};sps={sps:.1f}")


def cmd_polarity(a, L, C, G):
    for i in a.scales:
        print(f"\n=== Polarity, scale {i} ({NAMES[i]}) ===")
        print("Hands off...")
        time.sleep(1)
        base = statistics.fmean(L.collect([i], 2)[i])

        def press(label):
            print(f"Press and HOLD firmly on the center bump of {label}:")
            for n in (3, 2, 1):
                print(f"  {n}...")
                time.sleep(1)
            print("  sampling, keep holding")
            v = statistics.fmean(L.collect([i], 2)[i])
            print("  release.")
            time.sleep(2)
            return v - base

        dA = press("CELL A (loaded)")
        dB = press("CELL B (dummy)")
        print(f"  cell A delta {dA:+.0f} counts, cell B delta {dB:+.0f} counts")
        if min(abs(dA), abs(dB)) < 5000:   # ~120 g at the design sensitivity
            print("  INCONCLUSIVE: a delta is tiny. Press harder, or check that cell's wiring.")
            verdict = "inconclusive"
        elif (dA > 0) == (dB > 0):
            print("  PASS: both cells move the reading the same direction.")
            verdict = "pass"
        else:
            print("  FAIL: opposite directions. Swap cell B's two END wires (at E+ and E-), "
                  "not its center wire.")
            verdict = "fail"
        G.row("polarity", i, extra=f"dA={dA:.0f};dB={dB:.0f};{verdict}")


def cmd_repeat(a, L, C, G):
    require_cal(C, a.scales)
    for i in a.scales:
        print(f"\n=== Repeatability, scale {i}: {a.mass} kg x {a.n} ===")
        loaded, zeros = [], []
        for k in range(a.n):
            input(f"[{k + 1}/{a.n}] place {a.mass} kg centered, hands off, Enter...")
            loaded.append(statistics.fmean(C.kg(i, r) for r in L.collect([i], a.seconds)[i]))
            input("       remove it, hands off, Enter...")
            zeros.append(statistics.fmean(C.kg(i, r) for r in L.collect([i], a.seconds)[i]))
            print(f"       loaded {loaded[-1]:.3f} kg   zero {zeros[-1] * 1000:+.1f} g")
        sl = (max(loaded) - min(loaded)) * 1000
        sz = (max(zeros) - min(zeros)) * 1000
        err = (statistics.fmean(loaded) - a.mass) * 1000
        verdict = "PASS" if sl <= REPEAT_TOL_KG * 1000 else "FAIL"
        print(f"  loaded spread {sl:.1f} g (limit {REPEAT_TOL_KG * 1000:.0f}) -> {verdict}")
        print(f"  zero spread {sz:.1f} g;  mean error vs {a.mass} kg: {err:+.1f} g")
        G.row("repeat", i, kg=f"{statistics.fmean(loaded):.4f}",
              extra=f"spread_g={sl:.1f};zero_spread_g={sz:.1f};{verdict}")


def cmd_drift(a, L, C, G):
    if a.warmup > 0:
        print(f"Warm-up {a.warmup:g} min, no load, hands off...")
        time.sleep(a.warmup * 60)
    steps = max(1, int(a.minutes * 60 / a.interval))
    unit = {i: ("g", C.g_per_count(i)) if C.has_cal(i) else ("counts", 1.0) for i in a.scales}
    first, hist = None, {i: [] for i in a.scales}
    print(f"Logging {a.minutes:g} min, one {a.interval:g} s average per step")
    for k in range(steps):
        d = L.collect(a.scales, a.interval)
        means = {i: statistics.fmean(d[i]) for i in a.scales}
        if first is None:
            first = means
        line = f"  t={(k + 1) * a.interval / 60:5.1f} min"
        for i in a.scales:
            rel = (means[i] - first[i]) * unit[i][1]
            hist[i].append(rel)
            line += f"   {i}: {rel:+8.1f} {unit[i][0]}"
        print(line)
    for i in a.scales:
        h = hist[i]
        print(f"scale {i}: drift end-start {h[-1] - h[0]:+.1f} {unit[i][0]}, "
              f"max-min {max(h) - min(h):.1f} {unit[i][0]}")
        G.row("drift", i, extra=f"end_start={h[-1] - h[0]:.1f}{unit[i][0]};"
                                f"maxmin={max(h) - min(h):.1f};warmup_min={a.warmup}")


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description="HARP wheel-scale host tool")
    ap.add_argument("--port", help="serial port (default: autodetect /dev/ttyUSB*, /dev/ttyACM*)")
    ap.add_argument("--cal-file", default="scale_cal.json")
    ap.add_argument("--log", help="append results to this CSV")
    ap.add_argument("--note", help="free-text note stored in the CSV")

    common = argparse.ArgumentParser(add_help=False)
    common.add_argument("--scales", type=int, nargs="+", default=[1], choices=[1, 2, 3, 4])

    sub = ap.add_subparsers(dest="cmd", required=True)

    def add(name, fn, seconds=None, **extra):
        p = sub.add_parser(name, parents=[common])
        if seconds is not None:
            p.add_argument("--seconds", type=float, default=seconds)
        for flag, kw in extra.items():
            p.add_argument("--" + flag, **kw)
        p.set_defaults(fn=fn)

    add("info", cmd_info)
    add("monitor", cmd_monitor)
    add("tare", cmd_tare, 3.0)
    add("cal", cmd_cal, 5.0)
    add("read", cmd_read, 3.0, zero=dict(action="store_true", help="zero-return check"))
    add("noise", cmd_noise, 10.0)
    add("polarity", cmd_polarity)
    add("repeat", cmd_repeat, 3.0, mass=dict(type=float, required=True),
        n=dict(type=int, default=5))
    add("drift", cmd_drift, warmup=dict(type=float, default=10.0),
        minutes=dict(type=float, default=10.0), interval=dict(type=float, default=10.0))

    a = ap.parse_args()
    L = Link(a.port or find_port())
    C = Cal(a.cal_file)
    G = Log(a.log, a.note)
    for i in a.scales:
        if not L.present[i - 1] and a.cmd != "info":
            print(f"# warning: scale {i} not detected at boot (send 'p' via monitor or replug)")
    a.fn(a, L, C, G)


if __name__ == "__main__":
    main()
