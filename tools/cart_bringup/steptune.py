import serial, struct, time, argparse, os, statistics as st

ADDR, PORT, BAUD = 0x80, "/dev/serial/by-id/usb-Basicmicro_Inc._USB_Roboclaw_2x15A-if00", 115200
QPPS1, QPPS2, ACC = 2660, 2550, 2360
LOGDIR = os.path.expanduser("~/steplogs")

p = argparse.ArgumentParser()
p.add_argument("mode", choices=["straight", "spin"])
p.add_argument("--p1", type=float); p.add_argument("--i1", type=float)
p.add_argument("--p2", type=float); p.add_argument("--i2", type=float)
p.add_argument("--nopid", action="store_true")
p.add_argument("--dt", type=float, default=0.05)
a = p.parse_args()

def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc

ERRS, T0 = [], time.monotonic()
import atexit
atexit.register(lambda: print(f"Serial errors: {len(ERRS)}" + "".join(
    f"\n  t={t:.3f}s cmd {c} try {k} got {r}" for t, c, k, r in ERRS)))

def _xfer(ser, pkt, n, ok):
    for k in range(4):
        ser.reset_input_buffer(); ser.write(pkt); r = ser.read(n)
        if ok(r): return r
        ERRS.append((time.monotonic() - T0, pkt[1], k, r.hex() or "nothing"))
        time.sleep(0.02)
    raise IOError(f"cmd {pkt[1]}: failed after 4 tries")

def write_cmd(ser, cmd, payload=b""):
    pkt = bytes([ADDR, cmd]) + payload
    _xfer(ser, pkt + struct.pack(">H", crc16(pkt)), 1, lambda r: r == b"\xff")

def read_raw(ser, cmd, n):
    pkt = bytes([ADDR, cmd])
    ok = lambda r: len(r) == n + 2 and crc16(pkt + r[:n]) == struct.unpack(">H", r[n:])[0]
    return _xfer(ser, pkt, n + 2, ok)[:n]

def spd(ser, cmd):
    v, s = struct.unpack(">iB", read_raw(ser, cmd, 5)); return -v if s & 1 else v
def pwms(ser):
    a1, a2 = struct.unpack(">hh", read_raw(ser, 48, 4)); return a1/327.67, a2/327.67
def amps(ser):
    c1, c2 = struct.unpack(">hh", read_raw(ser, 49, 4)); return c1/100, c2/100
def read_pid(ser, cmd):
    P, I, D, Q = struct.unpack(">IIII", read_raw(ser, cmd, 16)); return P/65536, I/65536, D/65536, Q
def set_pid(ser, cmd, P, I, D, Q):
    write_cmd(ser, cmd, struct.pack(">IIII", int(D*65536), int(P*65536), int(I*65536), Q))
def speed_acc(ser, s1, s2):
    write_cmd(ser, 40, struct.pack(">Iii", ACC, s1, s2))

T1, T2, RUN = (1000, 1000, 3.0) if a.mode == "straight" else (600, -600, 4.0)

with serial.Serial(PORT, BAUD, timeout=0.02) as ser:
    try:
        P1, I1, D1, _ = read_pid(ser, 55); P2, I2, D2, _ = read_pid(ser, 56)
        if not a.nopid:
            P1 = a.p1 if a.p1 is not None else P1; I1 = a.i1 if a.i1 is not None else I1
            P2 = a.p2 if a.p2 is not None else P2; I2 = a.i2 if a.i2 is not None else I2
            set_pid(ser, 28, P1, I1, D1, QPPS1); set_pid(ser, 29, P2, I2, D2, QPPS2)
        r1, r2 = read_pid(ser, 55), read_pid(ser, 56)
        print(f"M1 P {r1[0]:.3f} I {r1[1]:.3f} | M2 P {r2[0]:.3f} I {r2[1]:.3f} (RAM only)")
        input(f"{a.mode}: clear area, Enter to run...")
        log, t0 = [], time.time()
        speed_acc(ser, T1, T2)
        while (t := time.time() - t0) < RUN:
            m1, m2 = spd(ser, 18), spd(ser, 19)
            w1, w2 = pwms(ser); c1, c2 = amps(ser)
            log.append((t, m1, m2, w1, w2, c1, c2))
            time.sleep(max(0, a.dt - (time.time() - t0 - t)))
        speed_acc(ser, 0, 0)
        while abs(spd(ser, 18)) > 5 or abs(spd(ser, 19)) > 5:
            time.sleep(0.05)
    finally:
        speed_acc(ser, 0, 0)

fn = os.path.join(LOGDIR, f"steplog_{a.mode}_{time.strftime('%H%M%S')}.csv")
with open(fn, "w") as f:
    f.write("t,m1_cps,m2_cps,m1_pwm_pct,m2_pwm_pct,m1_amps,m2_amps\n")
    for row in log: f.write(",".join(f"{x:.3f}" if isinstance(x, float) else str(x) for x in row) + "\n")

win = [r for r in log if r[0] >= RUN - 1.0]
tgt = abs(T1)
m = lambda k: st.mean(abs(r[k]) for r in win)
print(f"Steady state (last 1 s): M1 {100*(m(1)/tgt-1):+.1f}%  M2 {100*(m(2)/tgt-1):+.1f}%  mismatch {100*(m(1)-m(2))/tgt:+.1f}%")
print(f"Ripple (SD):  M1 {100*st.stdev(abs(r[1]) for r in win)/tgt:.1f}%  M2 {100*st.stdev(abs(r[2]) for r in win)/tgt:.1f}%")
print(f"Overshoot:    M1 {100*(max(abs(r[1]) for r in log)/tgt-1):+.1f}%  M2 {100*(max(abs(r[2]) for r in log)/tgt-1):+.1f}%")
print(f"PWM (mean):   M1 {m(3):.1f}%  M2 {m(4):.1f}%   (peak M1 {max(abs(r[3]) for r in log):.1f}%  M2 {max(abs(r[4]) for r in log):.1f}%)")
print(f"Current:      M1 {m(5):.2f} A  M2 {m(6):.2f} A   (peak M1 {max(abs(r[5]) for r in log):.2f}  M2 {max(abs(r[6]) for r in log):.2f})")
print(f"Log saved: {fn}")
