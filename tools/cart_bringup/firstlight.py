import serial, struct

ADDR, PORT, BAUD = 0x80, "/dev/serial/by-id/usb-Basicmicro_Inc._USB_Roboclaw_2x15A-if00", 115200

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
        raise IOError(f"cmd {cmd}: got {len(resp)} bytes, expected {nbytes+2}")
    data, rx_crc = resp[:-2], struct.unpack(">H", resp[-2:])[0]
    if crc16(bytes([ADDR, cmd]) + data) != rx_crc:
        raise IOError(f"cmd {cmd}: CRC mismatch")
    return data

with serial.Serial(PORT, BAUD, timeout=0.1) as ser:
    v = struct.unpack(">H", read_cmd(ser, 24, 2))[0] / 10.0
    print(f"Main battery: {v:.1f} V")
    enc, status = struct.unpack(">iB", read_cmd(ser, 16, 5))
    print(f"M1 encoder: {enc} counts (status 0x{status:02X})")
