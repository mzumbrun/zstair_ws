"""Minimal RoboClaw packet-serial transport for the HARP cart. No ROS imports.

RUNG 2a IS READ-ONLY. This class deliberately has no write/motion methods;
they are added in rung 2b after the read path is proven.

Protocol notes (packet serial):
  * CRC16-CCITT, poly 0x1021, init 0.
  * For read commands the request [addr, cmd] carries NO CRC. The reply CRC
    covers addr + cmd + reply data. (Bring-up 5.4: a bit-flipped request can
    execute as a different command. USB has been error-free; counters stay.)
  * Retry policy from bring-up: 4 tries x 20 ms, well inside the 0.2 s
    RoboClaw serial timeout. Any valid packet, reads included, feeds that
    timeout.
"""

import struct
import time
from dataclasses import dataclass, field

CMD_READ_VERSION = 21
CMD_READ_MAIN_BATTERY = 24
CMD_READ_ENCODERS = 78


def crc16(data: bytes, crc: int = 0) -> int:
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


@dataclass
class LinkStats:
    transactions: int = 0      # completed successfully (after any retries)
    retries: int = 0           # extra attempts beyond the first
    failures: int = 0          # all attempts exhausted
    crc_errors: int = 0
    short_reads: int = 0
    consecutive_failures: int = 0
    latencies_ms: list = field(default_factory=list)  # successful attempts only

    def record_latency(self, ms: float, keep: int = 20000):
        self.latencies_ms.append(ms)
        if len(self.latencies_ms) > keep:
            del self.latencies_ms[: len(self.latencies_ms) - keep]

    def latency_summary(self):
        if not self.latencies_ms:
            return None
        s = sorted(self.latencies_ms)
        n = len(s)

        def pct(p):
            return s[min(n - 1, int(p * (n - 1) + 0.5))]

        return {"n": n, "p50": pct(0.50), "p95": pct(0.95),
                "p99": pct(0.99), "max": s[-1]}


class RoboClawLink:
    def __init__(self, port, address=0x80, baud=115200,
                 tries=4, try_timeout_s=0.020, serial_factory=None):
        if serial_factory is None:
            import serial  # python3-serial
            serial_factory = serial.Serial
        # Baud is ignored by the RoboClaw USB CDC port but must be valid.
        self._ser = serial_factory(port=port, baudrate=baud, timeout=try_timeout_s)
        self.address = address
        self.tries = tries
        self.stats = LinkStats()

    def close(self):
        try:
            self._ser.close()
        except Exception:
            pass

    # ---- core read transaction -------------------------------------------
    def _read_fixed(self, cmd: int, n_data: int):
        """Send [addr, cmd]; expect n_data bytes + 2 CRC. Returns data or None."""
        req = bytes([self.address, cmd])
        for attempt in range(self.tries):
            if attempt:
                self.stats.retries += 1
            self._ser.reset_input_buffer()
            t0 = time.monotonic()
            self._ser.write(req)
            reply = self._ser.read(n_data + 2)
            dt_ms = (time.monotonic() - t0) * 1000.0
            if len(reply) != n_data + 2:
                self.stats.short_reads += 1
                continue
            data, rx_crc = reply[:n_data], struct.unpack(">H", reply[n_data:])[0]
            if crc16(req + data) != rx_crc:
                self.stats.crc_errors += 1
                continue
            self.stats.transactions += 1
            self.stats.consecutive_failures = 0
            self.stats.record_latency(dt_ms)
            return data
        self.stats.failures += 1
        self.stats.consecutive_failures += 1
        return None

    # ---- public read-only API --------------------------------------------
    def read_encoders(self):
        """cmd 78 -> (m1_raw, m2_raw) as unsigned 32-bit, or None.
        Caller differences them with kinematics.wrap_delta()."""
        data = self._read_fixed(CMD_READ_ENCODERS, 8)
        if data is None:
            return None
        return struct.unpack(">II", data)

    def read_main_battery_v(self):
        """cmd 24 -> volts as READ by the RoboClaw (~0.4 V low, bring-up 5.8),
        or None. Correction is applied by the caller."""
        data = self._read_fixed(CMD_READ_MAIN_BATTERY, 2)
        if data is None:
            return None
        return struct.unpack(">H", data)[0] / 10.0

    def read_version(self):
        """cmd 21 -> firmware string, or None. Variable length: text, 0x00, CRC."""
        req = bytes([self.address, CMD_READ_VERSION])
        for attempt in range(self.tries):
            if attempt:
                self.stats.retries += 1
            self._ser.reset_input_buffer()
            self._ser.write(req)
            buf = bytearray()
            while len(buf) < 48:
                b = self._ser.read(1)
                if not b:
                    break
                buf += b
                if b == b"\x00":
                    break
            crc_bytes = self._ser.read(2)
            if not buf or buf[-1] != 0 or len(crc_bytes) != 2:
                self.stats.short_reads += 1
                continue
            if crc16(req + bytes(buf)) != struct.unpack(">H", crc_bytes)[0]:
                self.stats.crc_errors += 1
                continue
            self.stats.transactions += 1
            self.stats.consecutive_failures = 0
            return buf[:-1].decode("ascii", errors="replace").strip()
        self.stats.failures += 1
        self.stats.consecutive_failures += 1
        return None
