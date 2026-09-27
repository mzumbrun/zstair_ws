import serial, struct, time, sys
ADDR, PORT = 0x80, "/dev/serial/by-id/usb-Basicmicro_Inc._USB_Roboclaw_2x15A-if00"
MODE = sys.argv[1] if len(sys.argv) > 1 else "idle"
def crc16(d):
    c = 0
    for b in d:
        c ^= b << 8
        for _ in range(8):
            c = ((c << 1) ^ 0x1021) if c & 0x8000 else (c << 1); c &= 0xFFFF
    return c
def duty(s, d):
    pkt = bytes([ADDR, 34]) + struct.pack(">hh", d, d)
    s.reset_input_buffer(); s.write(pkt + struct.pack(">H", crc16(pkt))); return s.read(1) == b"\xff"
with serial.Serial(PORT, 115200, timeout=0.02) as s:
    print(f"mode={MODE}, 20 s, Ctrl+C to stop early")
    t0 = last = time.time(); good = bad = 0
    try:
        while time.time() - t0 < 20:
            if MODE == "motors" and not duty(s, int(0.2 * 32767)): bad += 1
            s.reset_input_buffer(); s.write(bytes([ADDR, 24])); r = s.read(4)
            if len(r) == 4 and crc16(bytes([ADDR, 24]) + r[:2]) == struct.unpack(">H", r[2:])[0]: good += 1
            else: bad += 1; time.sleep(0.02)
            if time.time() - last >= 1:
                print(f"t={time.time()-t0:4.0f}s  good {good:4d}  bad {bad}"); last = time.time(); good = bad = 0
            time.sleep(0.01)
    finally:
        if MODE == "motors": duty(s, 0)
