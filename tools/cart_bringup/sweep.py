import serial, struct, time
ADDR, PORT = 0x80, "/dev/serial/by-id/usb-Basicmicro_Inc._USB_Roboclaw_2x15A-if00"
def crc16(d):
    c = 0
    for b in d:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) if c & 0x8000 else (c << 1); c &= 0xFFFF
    return c
def xfer(s, pkt, n, ok):
    for _ in range(4):
        s.reset_input_buffer(); s.write(pkt); r = s.read(n)
        if ok(r): return r
        time.sleep(0.02)
    raise IOError(f"cmd {pkt[1]} failed")
def duty(s, d1, d2):
    p = bytes([ADDR, 34]) + struct.pack(">hh", d1, d2)
    xfer(s, p + struct.pack(">H", crc16(p)), 1, lambda r: r == b"\xff")
def rd(s, cmd, n):
    p = bytes([ADDR, cmd])
    return xfer(s, p, n + 2, lambda r: len(r) == n + 2 and crc16(p + r[:n]) == struct.unpack(">H", r[n:])[0])[:n]
def spd(s, cmd):
    v, st = struct.unpack(">iB", rd(s, cmd, 5)); return -v if st & 1 else v
with serial.Serial(PORT, 115200, timeout=0.02) as s:
    rows = []
    try:
        input("Cart on blocks, wheels free. Enter to sweep...")
        for pct in (20, 40, 60, 80, 100):
            d = int(pct / 100 * 32767); t0 = time.time(); a, b = [], []
            while time.time() - t0 < 2.5:
                duty(s, d, d)
                if time.time() - t0 > 1.5: a.append(spd(s, 18)); b.append(spd(s, 19))
                time.sleep(0.05)
            v = struct.unpack(">H", rd(s, 24, 2))[0] / 10
            rows.append((pct, sum(a)/len(a), sum(b)/len(b), v))
            print(f"{pct:3d}%  M1 {rows[-1][1]:6.0f}  M2 {rows[-1][2]:6.0f}  bus {v:.1f} V")
    finally:
        duty(s, 0, 0)
for k, name in ((1, "M1"), (2, "M2")):
    xs = [r[0] for r in rows]; ys = [r[k] for r in rows]; n = len(xs)
    mx, my = sum(xs)/n, sum(ys)/n
    m = sum((x-mx)*(y-my) for x, y in zip(xs, ys)) / sum((x-mx)**2 for x in xs)
    print(f"{name}: slope {m:.1f} cps/%   zero-speed duty {mx - my/m:.1f}%   speed@100% {ys[-1]:.0f} cps")
