import serial, struct, time

ADDR, PORT, BAUD = 0x80, "/dev/serial/by-id/usb-Basicmicro_Inc._USB_Roboclaw_2x15A-if00", 115200

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

def read_speed(ser, cmd):
    ser.reset_input_buffer()
    ser.write(bytes([ADDR, cmd]))
    r = ser.read(7)
    if len(r) != 7 or crc16(bytes([ADDR, cmd]) + r[:5]) != struct.unpack(">H", r[5:])[0]:
        raise IOError(f"speed {cmd}: bad reply")
    spd, status = struct.unpack(">iB", r[:5])
    return -spd if status & 1 else spd

def speed(ser, s1, s2):
    write_cmd(ser, 37, struct.pack(">ii", s1, s2))   # SpeedM1M2

with serial.Serial(PORT, BAUD, timeout=0.1) as ser:
    try:
        for target in (1000, 2000):
            t0 = time.time()
            while time.time() - t0 < 2.0:            # 2 s kept alive
                speed(ser, target, target)
                time.sleep(0.05)
            m1, m2 = read_speed(ser, 18), read_speed(ser, 19)
            print(f"target {target}: M1 {m1} ({100*(m1/target-1):+.1f}%)  "
                  f"M2 {m2} ({100*(m2/target-1):+.1f}%)")
    finally:
        speed(ser, 0, 0)
