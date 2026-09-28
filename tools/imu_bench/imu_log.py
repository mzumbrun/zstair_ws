#!/usr/bin/env python3
"""
imu_log.py - HARP Step 1 dual-IMU bench logger (no ROS).

Reads MPU-6050 (0x68) and ICM-20948 (0x69) on Pi I2C bus 1 at a fixed rate,
stamps every read with the Pi's monotonic clock, and writes RAW counts to CSV.
Host timestamps are used deliberately: the ICM-20948 internal clock can be off
by up to +/-9% without timebase correction, so never integrate with an assumed dt.

Both sensors: gyro +/-500 dps, accel +/-4 g, ~100 Hz internal ODR, DLPF on.
Raw counts are logged (lossless); scale factors are written in the CSV header.

Usage:
  python3 imu_log.py --duration 1800                 # 30 min soak, both sensors
  python3 imu_log.py --duration 120 --only icm       # one sensor
  python3 imu_log.py --duration 0                    # run until Ctrl-C
"""
import argparse, csv, os, struct, sys, time
from datetime import datetime
from smbus2 import SMBus

BUS = 1
MPU_ADDR, ICM_ADDR = 0x68, 0x69
GYRO_LSB_PER_DPS = 65.5      # +/-500 dps, both parts
ACCEL_LSB_PER_G = 8192.0     # +/-4 g, both parts

# ---------------- MPU-6050 ----------------
def mpu_init(bus):
    who = bus.read_byte_data(MPU_ADDR, 0x75)
    if who != 0x68:
        raise RuntimeError(f"MPU-6050 WHO_AM_I=0x{who:02X}, expected 0x68")
    bus.write_byte_data(MPU_ADDR, 0x6B, 0x80); time.sleep(0.1)   # reset
    bus.write_byte_data(MPU_ADDR, 0x6B, 0x01); time.sleep(0.05)  # wake, PLL on X gyro
    bus.write_byte_data(MPU_ADDR, 0x1A, 0x03)   # DLPF cfg 3 (~44 Hz); gyro base rate 1 kHz
    bus.write_byte_data(MPU_ADDR, 0x19, 9)      # SMPLRT_DIV: 1 kHz/(1+9) = 100 Hz
    bus.write_byte_data(MPU_ADDR, 0x1B, 0x08)   # gyro FS_SEL=1, +/-500 dps
    bus.write_byte_data(MPU_ADDR, 0x1C, 0x08)   # accel AFS_SEL=1, +/-4 g

def mpu_read(bus):
    b = bus.read_i2c_block_data(MPU_ADDR, 0x3B, 14)   # ax ay az temp gx gy gz
    ax, ay, az, t, gx, gy, gz = struct.unpack(">7h", bytes(b))
    return ax, ay, az, gx, gy, gz, t

def mpu_temp_c(raw):
    return raw / 340.0 + 36.53

# ---------------- ICM-20948 ----------------
def icm_bank(bus, n):
    bus.write_byte_data(ICM_ADDR, 0x7F, n << 4)

def icm_init(bus):
    icm_bank(bus, 0)
    who = bus.read_byte_data(ICM_ADDR, 0x00)
    if who != 0xEA:
        raise RuntimeError(f"ICM-20948 WHO_AM_I=0x{who:02X}, expected 0xEA")
    bus.write_byte_data(ICM_ADDR, 0x06, 0x80); time.sleep(0.1)   # device reset
    icm_bank(bus, 0)
    bus.write_byte_data(ICM_ADDR, 0x06, 0x01); time.sleep(0.05)  # wake, auto clock (PLL)
    bus.write_byte_data(ICM_ADDR, 0x07, 0x00)                    # accel + gyro on
    icm_bank(bus, 2)
    bus.write_byte_data(ICM_ADDR, 0x00, 10)     # GYRO_SMPLRT_DIV: 1.1 kHz/(1+10) = 100 Hz
    bus.write_byte_data(ICM_ADDR, 0x01, (3 << 3) | (1 << 1) | 1)  # DLPF 3, +/-500 dps, FCHOICE=1
    bus.write_byte_data(ICM_ADDR, 0x10, 0x00)   # ACCEL_SMPLRT_DIV_1
    bus.write_byte_data(ICM_ADDR, 0x11, 10)     # ACCEL_SMPLRT_DIV_2: ~100 Hz
    bus.write_byte_data(ICM_ADDR, 0x14, (3 << 3) | (1 << 1) | 1)  # DLPF 3, +/-4 g, FCHOICE=1
    icm_bank(bus, 0)
    time.sleep(0.05)

def icm_read(bus):
    b = bus.read_i2c_block_data(ICM_ADDR, 0x2D, 14)   # ax ay az gx gy gz temp
    ax, ay, az, gx, gy, gz, t = struct.unpack(">7h", bytes(b))
    return ax, ay, az, gx, gy, gz, t

def icm_temp_c(raw):
    return raw / 333.87 + 21.0

# ---------------- main ----------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=1800, help="seconds; 0 = until Ctrl-C")
    ap.add_argument("--rate", type=float, default=100.0, help="host poll rate, Hz")
    ap.add_argument("--only", choices=["mpu", "icm"], help="log one sensor only")
    ap.add_argument("--tag", default="soak", help="filename tag, e.g. soak, cw10, ccw10")
    ap.add_argument("--outdir", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs"))
    a = ap.parse_args()

    use_mpu = a.only in (None, "mpu")
    use_icm = a.only in (None, "icm")
    os.makedirs(a.outdir, exist_ok=True)
    path = os.path.join(a.outdir, f"{a.tag}_{datetime.now():%Y%m%d_%H%M%S}.csv")

    bus = SMBus(BUS)
    if use_mpu: mpu_init(bus)
    if use_icm: icm_init(bus)

    cols = ["t_ns"]
    if use_mpu: cols += ["mpu_ax", "mpu_ay", "mpu_az", "mpu_gx", "mpu_gy", "mpu_gz", "mpu_temp"]
    if use_icm: cols += ["icm_ax", "icm_ay", "icm_az", "icm_gx", "icm_gy", "icm_gz", "icm_temp"]
    cols += ["err"]

    period = 1.0 / a.rate
    n = errs = 0
    acc = {"mpu": [0.0, 0.0, 0], "icm": [0.0, 0.0, 0]}   # sum gz dps, sum temp C, count
    t0 = time.monotonic()
    next_t = t0
    last_print = t0

    with open(path, "w", newline="") as f:
        f.write(f"# HARP imu_log.py  started {datetime.now().isoformat()}\n")
        f.write(f"# gyro_lsb_per_dps={GYRO_LSB_PER_DPS}  accel_lsb_per_g={ACCEL_LSB_PER_G}\n")
        f.write("# mpu_temp_C = raw/340 + 36.53 ; icm_temp_C = raw/333.87 + 21\n")
        f.write("# t_ns = time.monotonic_ns() after the read completes; err=1 row has blanks\n")
        w = csv.writer(f)
        w.writerow(cols)
        print(f"Logging to {path}  (Ctrl-C to stop)")
        try:
            while a.duration == 0 or (time.monotonic() - t0) < a.duration:
                row, err = [], 0
                try:
                    if use_mpu:
                        r = mpu_read(bus); row += r
                        acc["mpu"][0] += r[5] / GYRO_LSB_PER_DPS; acc["mpu"][1] += mpu_temp_c(r[6]); acc["mpu"][2] += 1
                    if use_icm:
                        r = icm_read(bus); row += r
                        acc["icm"][0] += r[5] / GYRO_LSB_PER_DPS; acc["icm"][1] += icm_temp_c(r[6]); acc["icm"][2] += 1
                except OSError:
                    err = 1; errs += 1
                    row = [""] * (len(cols) - 2)
                w.writerow([time.monotonic_ns()] + row + [err])
                n += 1

                now = time.monotonic()
                if now - last_print >= 5.0:
                    msg = f"t={now - t0:7.1f}s  n={n}  i2c_err={errs}"
                    for k in ("mpu", "icm"):
                        s = acc[k]
                        if s[2]:
                            msg += f"  {k}: gz={s[0]/s[2]:+.3f} dps T={s[1]/s[2]:.2f}C"
                        acc[k] = [0.0, 0.0, 0]
                    print(msg, flush=True)
                    f.flush()
                    last_print = now

                next_t += period
                delay = next_t - time.monotonic()
                if delay > 0:
                    time.sleep(delay)
                else:
                    next_t = time.monotonic()   # fell behind; resync, don't burst
        except KeyboardInterrupt:
            pass
    bus.close()
    print(f"Done: {n} rows, {errs} I2C errors -> {path}")

if __name__ == "__main__":
    sys.exit(main())
