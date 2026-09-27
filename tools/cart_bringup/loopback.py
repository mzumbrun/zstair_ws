import serial
s = serial.Serial('/dev/serial/by-id/usb-Basicmicro_Inc._USB_Roboclaw_2x15A-if00', 115200, timeout=0.5)
msg = b'HARP loopback 0123456789'
s.write(msg)
rx = s.read(len(msg))
print('PASS' if rx == msg else f'FAIL: got {rx!r}')
