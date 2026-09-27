import serial, struct

ADDR, PORT, BAUD = 0x80, "/dev/serial/by-id/usb-Basicmicro_Inc._USB_Roboclaw_2x15A-if00", 115200
EXPECT = 1425.1  # counts per output rev

def crc16(data):
    crc = 0
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if crc & 0x8000 else (crc << 1)
            crc &= 0xFFFF
    return crc

def read_cmd(ser, cmd, nbytes):
    ser.reset_input_buffer()
    ser.write(bytes([ADDR, cmd]))
    resp = ser.read(nbytes + 2)
    if len(resp) != nbytes + 2:
        raise IOError(f"cmd {cmd}: got {len(resp)} bytes")
    data, rx_crc = resp[:-2], struct.unpack(">H", resp[-2:])[0]
    if crc16(bytes([ADDR, cmd]) + data) != rx_crc:
        raise IOError(f"cmd {cmd}: CRC mismatch")
    return data

def reset_encoders(ser):
    pkt = bytes([ADDR, 20])
    ser.reset_input_buffer()
    ser.write(pkt + struct.pack(">H", crc16(pkt)))
    if ser.read(1) != b'\xff':
        raise IOError("reset: no ack")

with serial.Serial(PORT, BAUD, timeout=0.1) as ser:
    reset_encoders(ser)
    revs = float(input("Encoders zeroed. Turn each wheel N revs forward, then enter N: "))
    for name, cmd in (("M1", 16), ("M2", 17)):
        enc, _ = struct.unpack(">iB", read_cmd(ser, cmd, 5))
        err = 100 * (abs(enc) / (EXPECT * revs) - 1)
        print(f"{name}: {enc:+d} counts  ({err:+.2f}% vs {EXPECT*revs:.0f})")
