"""Rung 2b test driver. RUN ON rpi4u (uses the node's monotonic_ns stamps in
/cart/wheel_counts, which share this host's clock).

Publishes cmd_vel at 20 Hz for --duration s, measures steady-state wheel speed
from the node's timestamped counts, then ends one of three ways:

  --end stop     publish zero, measure the commanded ramp-down        (default)
  --end abandon  go silent, measure the node's cmd_vel timeout + ramp
  --end kill     keep publishing; YOU kill the node (kill -9). Reports the
                 last counts seen, for the enc_snapshot watchdog check.

  ros2 run harp_cart_drive speed_check --v 0.2112
  ros2 run harp_cart_drive speed_check --w 0.5
  ros2 run harp_cart_drive speed_check --v 0.2112 --end abandon
"""

import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Int64MultiArray

from harp_cart_drive.kinematics import DriveGeometry, inverse, wrap_delta  # noqa: F401

ACCEL = 2360.0
CAP = 2000.0
CMD_TIMEOUT = 0.25
CYCLE = 0.05


class SpeedCheck(Node):
    def __init__(self, a):
        super().__init__("harp_speed_check")
        self.a = a
        self.pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.samples = []  # (t_ns, m1, m2)
        self.create_subscription(Int64MultiArray, "cart/wheel_counts", self.on_counts, 50)

    def on_counts(self, msg):
        if len(msg.data) >= 3:
            self.samples.append((msg.data[2], msg.data[0], msg.data[1]))

    def send(self, v, w):
        t = Twist()
        t.linear.x, t.angular.z = float(v), float(w)
        self.pub.publish(t)

    def spin_for(self, seconds, v=None, w=None):
        """Spin; if v/w given, publish at 20 Hz. Returns monotonic_ns of last publish."""
        end = time.monotonic() + seconds
        last = None
        next_pub = time.monotonic()
        while time.monotonic() < end:
            if v is not None and time.monotonic() >= next_pub:
                self.send(v, w)
                last = time.monotonic_ns()
                next_pub += 0.05
            rclpy.spin_once(self, timeout_sec=0.005)
        return last


def speed_between(samples, t0_ns, t1_ns):
    win = [s for s in samples if t0_ns <= s[0] <= t1_ns]
    if len(win) < 2:
        return None
    (ta, a1, a2), (tb, b1, b2) = win[0], win[-1]
    dt = (tb - ta) / 1e9
    return (b1 - a1) / dt, (b2 - a2) / dt, dt


def stationary_after(samples, t_from_ns):
    """First sample time after t_from where counts then stay unchanged for 2 samples."""
    post = [s for s in samples if s[0] >= t_from_ns]
    for i in range(len(post) - 2):
        if post[i][1:] == post[i + 1][1:] == post[i + 2][1:]:
            return post[i]
    return None


def counts_since(samples, t_from_ns, stop_sample):
    pre = [s for s in samples if s[0] <= t_from_ns]
    if not pre or stop_sample is None:
        return None
    ref = pre[-1]
    return stop_sample[1] - ref[1], stop_sample[2] - ref[2], (stop_sample[0] - ref[0]) / 1e9


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--v", type=float, default=0.0, help="linear m/s")
    ap.add_argument("--w", type=float, default=0.0, help="angular rad/s, CCW +")
    ap.add_argument("--duration", type=float, default=12.0)
    ap.add_argument("--end", choices=["stop", "abandon", "kill"], default="stop")
    a = ap.parse_args()

    geo = DriveGeometry()
    t1, t2 = inverse(geo, a.v, a.w, CAP)
    peak = max(abs(t1), abs(t2))
    settle = peak / ACCEL + 0.5

    rclpy.init()
    n = SpeedCheck(a)
    print(f"cmd v={a.v:+.4f} m/s w={a.w:+.4f} rad/s -> target M1(L) {t1:+d}  M2(R) {t2:+d} counts/s")

    n.spin_for(1.0)  # collect a baseline
    if not n.samples:
        print("No /cart/wheel_counts received: is the node running (rung 2b build)?")
        return
    t_start = time.monotonic_ns()

    if a.end == "kill":
        print(">>> Publishing until the node dies. In another terminal, after a few "
              "seconds at speed:  pkill -9 -f lib/harp_cart_drive/drive_node")
        last_n, quiet_since = 0, None
        while True:
            n.spin_for(0.1, a.v, a.w)
            if len(n.samples) != last_n:
                last_n, quiet_since = len(n.samples), None
            elif quiet_since is None:
                quiet_since = time.monotonic()
            elif time.monotonic() - quiet_since > 0.5:
                break
        tl, m1, m2 = n.samples[-1]
        print(f"Node silent. Last counts seen: M1 {m1}  M2 {m2}")
        print("Now:  ros2 run harp_cart_drive enc_snapshot --compare /tmp/kill.json")
        print("Post-kill counts = snapshot delta - these values. "
              f"Predict ~{0.2 * peak:.0f} (0.2 s timeout) + up to {CYCLE * peak:.0f} "
              "(last msg to kill) + coast. Rules out 0.5 s / 1.0 s timeouts if well "
              f"under {0.5 * peak:.0f}.")
        n.destroy_node()
        rclpy.shutdown()
        return

    t_last_pub = n.spin_for(a.duration, a.v, a.w)
    t_ss0 = t_start + int(settle * 1e9)
    sp = speed_between(n.samples, t_ss0, t_last_pub)

    if sp is None:
        print("Not enough samples in the steady-state window")
    else:
        s1, s2, dt = sp
        e1 = (s1 / t1 - 1) * 100 if t1 else float("nan")
        e2 = (s2 / t2 - 1) * 100 if t2 else float("nan")
        print(f"Steady state over {dt:.2f} s (after {settle:.2f} s settle):")
        print(f"  M1(L) {s1:+9.1f} c/s  vs {t1:+d}  error {e1:+.3f}%")
        print(f"  M2(R) {s2:+9.1f} c/s  vs {t2:+d}  error {e2:+.3f}%")
        if t1 and t2:
            print(f"  ratio M1/M2 achieved {s1 / s2:+.4f}  vs target {t1 / t2:+.4f}")

    if a.end == "stop":
        n.spin_for(2.5, 0.0, 0.0)
        pred = peak ** 2 / (2 * ACCEL)
        lo, hi = pred, pred + 2 * CYCLE * peak
        label = "Commanded stop (zero cmd_vel)"
    else:
        n.spin_for(2.5)  # silent
        pred = CMD_TIMEOUT * peak + peak ** 2 / (2 * ACCEL)
        lo, hi = pred, pred + 2 * CYCLE * peak
        label = "Abandoned cmd_vel (node timeout)"

    st = stationary_after(n.samples, t_last_pub)
    cs = counts_since(n.samples, t_last_pub, st)
    if cs is None:
        print(f"{label}: could not find a stationary point")
    else:
        c1, c2, dur = cs
        print(f"{label}: stationary {dur:.2f} s after last cmd_vel; "
              f"counts M1 {c1:+d}  M2 {c2:+d}")
        print(f"  predicted |counts| {lo:.0f}-{hi:.0f} "
              f"(ramp {peak ** 2 / (2 * ACCEL):.0f}"
              + (f" + timeout {CMD_TIMEOUT * peak:.0f}" if a.end == "abandon" else "")
              + f", +0-{2 * CYCLE * peak:.0f} for cycle phase)")

    n.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
