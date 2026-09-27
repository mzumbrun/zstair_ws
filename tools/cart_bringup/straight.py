import serial, struct, time

ADDR, PORT, BAUD = 0x80, "/dev/serial/by-id/usb-Basicmicro_Inc._USB_Roboclaw_2x15A-if00", 115200
CPM   = 4726          # nominal counts per meter (96 mm wheel)
DIST  = 9452          # 2.000 m
SPD   = 1000          # counts/s
ACC   = 2360          # counts/s^2 (0.5 m/s^2)
BRAKE = SPD**2 / (2*ACC)

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

def read_i(ser, cmd):
    ser.reset_input_buffer()
    ser.write(bytes([ADDR, cmd]))
    r = ser.read(7)
    if len(r) != 7 or crc16(bytes([ADDR, cmd]) + r[:5]) != struct.unpack(">H", r[5:])[0]:
        raise IOError(f"cmd {cmd}: bad reply")
    v, st = struct.unpack(">iB", r[:5])
    return v, st

def speed_acc(ser, s1, s2):
    write_cmd(ser, 40, struct.pack(">Iii", ACC, s1, s2))   # SpeedAccelM1M2

with serial.Serial(PORT, BAUD, timeout=0.1) as ser:
    try:
        write_cmd(ser, 20)                       # zero encoders
        input("Mark the start position, then press Enter to drive 2 m...")
        speed_acc(ser, SPD, SPD)
        braking = False
        still = 0
        while True:
            e1, _ = read_i(ser, 16); e2, _ = read_i(ser, 17)
            if not braking and (e1 + e2) / 2 >= DIST - BRAKE:
                speed_acc(ser, 0, 0); braking = True
            if braking:
                s1, st1 = read_i(ser, 18); s2, st2 = read_i(ser, 19)
                still = still + 1 if abs(s1) < 5 and abs(s2) < 5 else 0
                if still >= 6: break
            time.sleep(0.05)
        avg = (e1 + e2) / 2
        print(f"M1 {e1}  M2 {e2}  avg {avg:.0f} counts")
        print(f"Encoder distance at nominal 96 mm: {avg/CPM*1000:.1f} mm")
        print(f"M1-M2 difference: {e1-e2:+d} counts")
    finally:
        speed_acc(ser, 0, 0)
