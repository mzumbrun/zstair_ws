"""Transport checks against a fake serial port. No hardware, no ROS."""
import struct

from harp_cart_drive.roboclaw_link import RoboClawLink, crc16


def test_crc_reference_vector():
    assert crc16(b"123456789") == 0x31C3    # CRC-16/XMODEM check value


class FakeSerial:
    def __init__(self, replies, **_):
        self.replies, self.writes, self.buf = list(replies), [], b""

    def reset_input_buffer(self):
        self.buf = b""

    def write(self, data):
        self.writes.append(bytes(data))
        self.buf = self.replies.pop(0) if self.replies else b""

    def read(self, n):
        out, self.buf = self.buf[:n], self.buf[n:]
        return out

    def close(self):
        pass


def enc_reply(m1, m2, addr=0x80, bad_crc=False):
    data = struct.pack(">II", m1 & 0xFFFFFFFF, m2 & 0xFFFFFFFF)
    crc = crc16(bytes([addr, 78]) + data) ^ (1 if bad_crc else 0)
    return data + struct.pack(">H", crc)


def make(replies):
    return RoboClawLink("fake", serial_factory=lambda **kw: FakeSerial(replies, **kw))


def test_read_encoders_ok():
    link = make([enc_reply(1234, 2**32 - 7)])
    assert link.read_encoders() == (1234, 2**32 - 7)
    assert link._ser.writes == [bytes([0x80, 78])]
    assert link.stats.transactions == 1 and link.stats.retries == 0


def test_crc_error_then_retry_succeeds():
    link = make([enc_reply(1, 2, bad_crc=True), enc_reply(1, 2)])
    assert link.read_encoders() == (1, 2)
    assert link.stats.crc_errors == 1 and link.stats.retries == 1


def test_exhausted_retries_count_consecutive_failures():
    link = make([b""] * 8)
    assert link.read_encoders() is None and link.read_encoders() is None
    assert link.stats.failures == 2 and link.stats.consecutive_failures == 2
    assert link.stats.short_reads == 8


def test_battery_raw_reading():
    data = struct.pack(">H", 126)
    reply = data + struct.pack(">H", crc16(bytes([0x80, 24]) + data))
    assert make([reply]).read_main_battery_v() == 12.6


def test_version_string():
    text = b"USB Roboclaw 2x15a v4.4.9\n\x00"
    reply = text + struct.pack(">H", crc16(bytes([0x80, 21]) + text))
    assert make([reply]).read_version() == "USB Roboclaw 2x15a v4.4.9"
