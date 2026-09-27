import serial, struct, time, sys, math

ADDR, PORT, BAUD = 0x80, "/dev/serial/by-id/usb-Basicmicro_Inc._USB_Roboclaw_2x15A-if00", 115200
D_EFF  = 0.0958                 # m, from straight-line test
TRACK  = 0.430                 # m, nominal
CPR    = 1425.1
REVS   = 1                      # set 3 for finer resolution
CPM    = CPR / (math.pi * D_EFF)
TARGET = REVS * math.pi * TRACK * CPM      # counts per wheel
SPD    = 600
ACC    = 2360
BRAKE  = SPD**2 / (2*ACC)

def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc

def write_cmd(ser, cmd, payload=b""):
    pkt = bytes([ADDR, cmd]) + payload
    ser.reset_input_buffer()
    ser.write(pkt + struct.pack(">H", crc16(pkt)))
    if ser.read(1) != b'\xff':
        raise IOError(f"cmd {cmd}: no ack")

def read_raw(ser, cmd, n):
    ser.reset_input_buffer()
    ser.write(bytes([ADDR, cmd]))
    r = ser.read(n + 2)
    if len(r) != n + 2 or crc16(bytes([ADDR, cmd]) + r[:n]) != struct.unpack(">H", r[n:])[0]:
        raise IOError(f"cmd {cmd}: bad reply")
    return r[:n]

def enc(ser, cmd):  return struct.unpack(">iB", read_raw(ser, cmd, 5))[0]
def spd(ser, cmd):
    v, st = struct.unpack(">iB", read_raw(ser, cmd, 5)); return -v if st & 1 else v
def currents(ser):
    a, b = struct.unpack(">hh", read_raw(ser, 49, 4)); return a/100, b/100

def speed_acc(ser, s1, s2):
    write_cmd(ser, 40, struct.pack(">Iii", ACC, s1, s2))

d = -1 if len(sys.argv) > 1 and sys.argv[1] == "-" else 1   # + : M1 fwd / M2 rev
with serial.Serial(PORT, BAUD, timeout=0.1) as ser:
    try:
        write_cmd(ser, 20)
        input(f"Target {TARGET:.0f} counts/wheel ({REVS} rev). Track = {TRACK:.3f} Align reference line, Enter to spin...")
        speed_acc(ser, d*SPD, -d*SPD)
        braking, still, i1max, i2max = False, 0, 0.0, 0.0
        while True:
            e1, e2 = enc(ser, 16), enc(ser, 17)
            c1, c2 = currents(ser); i1max, i2max = max(i1max, abs(c1)), max(i2max, abs(c2))
            if not braking and (abs(e1) + abs(e2)) / 2 >= TARGET - BRAKE:
                speed_acc(ser, 0, 0); braking = True
            if braking:
                still = still + 1 if abs(spd(ser, 18)) < 5 and abs(spd(ser, 19)) < 5 else 0
                if still >= 6: break
            time.sleep(0.05)
        avg = (abs(e1) + abs(e2)) / 2
        print(f"M1 {e1:+d}  M2 {e2:+d}  avg {avg:.0f}  (target {TARGET:.0f})")
        print(f"Encoder-predicted rotation: {360*REVS*avg/TARGET:.2f} deg")
        print(f"Peak current: M1 {i1max:.2f} A  M2 {i2max:.2f} A")
    finally:
        speed_acc(ser, 0, 0)
