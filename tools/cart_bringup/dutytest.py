import serial, struct, time

ADDR, PORT, BAUD = 0x80, "/dev/serial/by-id/usb-Basicmicro_Inc._USB_Roboclaw_2x15A-if00", 115200
DUTY = int(0.10 * 32767)   # 10% duty

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

def read_enc(ser):
    out = []
    for cmd in (16, 17):
        ser.reset_input_buffer()
        ser.write(bytes([ADDR, cmd]))
        r = ser.read(7)
        if len(r) != 7 or crc16(bytes([ADDR, cmd]) + r[:5]) != struct.unpack(">H", r[5:])[0]:
            raise IOError(f"enc {cmd}: bad reply")
        out.append(struct.unpack(">i", r[:4])[0])
    return out

def duty(ser, d1, d2):
    write_cmd(ser, 34, struct.pack(">hh", d1, d2))   # DutyM1M2

with serial.Serial(PORT, BAUD, timeout=0.1) as ser:
    try:
        write_cmd(ser, 20)                     # reset encoders
        print("Phase A: 10% duty, kept alive 1 s -- watch direction")
        t0 = time.time()
        while time.time() - t0 < 1.0:
            duty(ser, DUTY, DUTY)
            time.sleep(0.05)
        duty(ser, 0, 0); time.sleep(0.5)
        a = read_enc(ser)
        print(f"  M1 {a[0]:+d}   M2 {a[1]:+d}   (expect ~+150..+250 each)")

        write_cmd(ser, 20)
        print("Phase B: one duty command, then silence 1 s")
        duty(ser, DUTY, DUTY)
        time.sleep(1.0)                        # NO packets here
        b1 = read_enc(ser)
        time.sleep(1.0)
        b2 = read_enc(ser)
        print(f"  after 1 s: M1 {b1[0]:+d}   M2 {b1[1]:+d}   (expect ~40-60 if watchdog works)")
        print(f"  drift next 1 s: M1 {b2[0]-b1[0]:+d}   M2 {b2[1]-b1[1]:+d}   (expect ~0)")
    finally:
        duty(ser, 0, 0)
