"""HARP cart drive node. RUNG 2a: READ-ONLY.

Reads encoders (cmd 78) and battery (cmd 24) from the cart RoboClaw over USB,
publishes /odom (+ TF odom->base_link), raw accumulated counts, battery and
/diagnostics, and logs transaction-latency and loop-period statistics.

This rung sends NO motion commands. It subscribes to nothing.

Rung 2a pass criteria (see README):
  1. Signs: left wheel (M1) forward -> +m1 counts; spin CCW -> +yaw.
  2. Identity: odom distance == delta counts / 4735.1 from an independent
     bench read before and after, within 1 count.
  3. Link: 0 failures; latency p99 measured (replaces the 1-3 ms estimate).
  4. Loop: 20 Hz period holds (max period < 2x nominal).
"""

import math
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Int64MultiArray
from tf2_ros import TransformBroadcaster

from harp_cart_drive.kinematics import (
    DriveGeometry, body_increment, integrate_midpoint, wheel_distances, wrap_delta)
from harp_cart_drive.roboclaw_link import RoboClawLink

OK, WARN, ERROR = DiagnosticStatus.OK, DiagnosticStatus.WARN, DiagnosticStatus.ERROR


def yaw_to_quat(yaw):
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


class CartDriveNode(Node):
    def __init__(self):
        super().__init__("harp_cart_drive")

        p = self.declare_parameter
        self.port = p("port", "/dev/serial/by-id/usb-Basicmicro_Inc._USB_Roboclaw_2x15A-if00").value
        self.address = p("address", 0x80).value
        self.loop_hz = float(p("loop_hz", 20.0).value)
        self.battery_hz = float(p("battery_hz", 1.0).value)
        self.battery_offset_v = float(p("battery_offset_v", 0.4).value)
        self.low_voltage_warn_v = float(p("low_voltage_warn_v", 10.8).value)
        self.fault_limit = int(p("fault_consecutive_limit", 3).value)
        self.stats_period_s = float(p("stats_period_s", 10.0).value)
        self.speed_cap_counts_s = float(p("speed_cap_counts_s", 2000.0).value)
        self.publish_tf = bool(p("publish_tf", True).value)
        self.odom_frame = p("odom_frame", "odom").value
        self.base_frame = p("base_frame", "base_link").value
        self.pose_cov = list(p("pose_covariance_diagonal",
                               [1e-4, 1e-4, 1e6, 1e6, 1e6, 1e-1]).value)
        self.geo = DriveGeometry(
            counts_per_rev=float(p("counts_per_rev", 1425.1).value),
            wheel_diameter_m=float(p("wheel_diameter_m", 0.0958).value),
            wheel_separation_m=float(p("wheel_separation_m", 0.430).value),
            left_multiplier=float(p("left_wheel_multiplier", 1.0).value),
            right_multiplier=float(p("right_wheel_multiplier", 1.0).value),
        )

        # Twist variance from count quantization: difference of two quantized
        # reads -> triangular error, sd = 1/sqrt(6) counts per cycle.
        cpm = 0.5 * (self.geo.counts_per_m_left + self.geo.counts_per_m_right)
        sd_v = (1.0 / cpm) * self.loop_hz / math.sqrt(6.0)
        sd_w = math.sqrt(2.0) * sd_v / self.geo.wheel_separation_m
        self.twist_cov = [sd_v ** 2, 1e6, 1e6, 1e6, 1e6, sd_w ** 2]

        self.link = RoboClawLink(self.port, self.address)
        version = self.link.read_version()
        if version is None:
            raise RuntimeError(f"No valid reply from RoboClaw on {self.port} (cmd 21)")
        self.get_logger().info(f"RoboClaw: {version}  |  RUNG 2a READ-ONLY, no motion commands")
        self.get_logger().info(
            f"counts/m L {self.geo.counts_per_m_left:.1f}  R {self.geo.counts_per_m_right:.1f}  "
            f"b {self.geo.wheel_separation_m:.3f} m  loop {self.loop_hz:.0f} Hz  "
            f"twist sd {sd_v * 1000:.2f} mm/s")

        # State
        self.prev_raw = None
        self.prev_t = None
        self.acc_m1 = 0
        self.acc_m2 = 0
        self.x = self.y = self.theta = 0.0
        self.dist_path = 0.0
        self.fault_latched = False
        self.implausible = 0
        self.battery_v = None
        self.cycle = 0
        self.periods_ms = []
        self.last_cb_t = None
        self.battery_every = max(1, int(round(self.loop_hz / self.battery_hz)))

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, "odom", 10)
        self.counts_pub = self.create_publisher(Int64MultiArray, "cart/wheel_counts", 10)
        self.batt_pub = self.create_publisher(BatteryState, "cart/battery", 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "diagnostics", 10)
        self.tf_bc = TransformBroadcaster(self) if self.publish_tf else None

        self.create_timer(1.0 / self.loop_hz, self.on_cycle)
        self.create_timer(1.0, self.publish_diagnostics)
        self.create_timer(self.stats_period_s, self.log_stats)

    # ---------------------------------------------------------------------
    def on_cycle(self):
        now_m = time.monotonic()
        if self.last_cb_t is not None:
            self.periods_ms.append((now_m - self.last_cb_t) * 1000.0)
            if len(self.periods_ms) > 20000:
                del self.periods_ms[:10000]
        self.last_cb_t = now_m
        self.cycle += 1

        if self.cycle % self.battery_every == 0:
            v = self.link.read_main_battery_v()
            if v is not None:
                self.battery_v = v + self.battery_offset_v
                self.publish_battery()

        raw = self.link.read_encoders()
        t = time.monotonic()
        stamp = self.get_clock().now().to_msg()

        if raw is None:
            if self.link.stats.consecutive_failures >= self.fault_limit and not self.fault_latched:
                self.fault_latched = True
                self.get_logger().error(
                    f"LINK FAULT LATCHED: {self.link.stats.consecutive_failures} consecutive "
                    "exhausted transactions. (Rung 2b: stop commanding; RoboClaw 0.2 s "
                    "timeout stops motors.)")
            return

        if self.prev_raw is None:  # baseline from first read; never reset encoders
            self.prev_raw, self.prev_t = raw, t
            self.get_logger().info(f"Encoder baseline M1 {raw[0]}  M2 {raw[1]} (raw uint32)")
            return

        d1 = wrap_delta(raw[0], self.prev_raw[0])
        d2 = wrap_delta(raw[1], self.prev_raw[1])
        dt = t - self.prev_t
        self.prev_raw, self.prev_t = raw, t

        # Plausibility: > 3x speed cap over dt cannot be real wheel motion.
        limit = 3.0 * self.speed_cap_counts_s * dt + 50
        if abs(d1) > limit or abs(d2) > limit:
            self.implausible += 1
            self.get_logger().error(
                f"Implausible encoder jump d1={d1} d2={d2} over {dt * 1000:.1f} ms; "
                "sample NOT integrated. This should never happen: investigate.")
            return

        self.acc_m1 += d1
        self.acc_m2 += d2
        d_left, d_right = wheel_distances(self.geo, d1, d2)
        ds, dth = body_increment(self.geo, d_left, d_right)
        self.x, self.y, self.theta = integrate_midpoint(self.x, self.y, self.theta, ds, dth)
        self.dist_path += ds

        v = ds / dt if dt > 0 else 0.0
        w = dth / dt if dt > 0 else 0.0
        self.publish_odom(stamp, v, w)

        c = Int64MultiArray()
        c.data = [self.acc_m1, self.acc_m2]
        self.counts_pub.publish(c)

    # ---------------------------------------------------------------------
    def publish_odom(self, stamp, v, w):
        qx, qy, qz, qw = yaw_to_quat(self.theta)
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x = self.x
        o.pose.pose.position.y = self.y
        o.pose.pose.orientation.x, o.pose.pose.orientation.y = qx, qy
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        o.twist.twist.linear.x = v
        o.twist.twist.angular.z = w
        for i in range(6):
            o.pose.covariance[i * 7] = self.pose_cov[i]
            o.twist.covariance[i * 7] = self.twist_cov[i]
        self.odom_pub.publish(o)

        if self.tf_bc is not None:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id = self.base_frame
            tf.transform.translation.x = self.x
            tf.transform.translation.y = self.y
            tf.transform.rotation.x, tf.transform.rotation.y = qx, qy
            tf.transform.rotation.z, tf.transform.rotation.w = qz, qw
            self.tf_bc.sendTransform(tf)

    def publish_battery(self):
        b = BatteryState()
        b.header.stamp = self.get_clock().now().to_msg()
        b.voltage = float(self.battery_v)
        b.present = True
        self.batt_pub.publish(b)

    def publish_diagnostics(self):
        s = self.link.stats
        st = DiagnosticStatus()
        st.name = "harp_cart_drive: RoboClaw link"
        st.hardware_id = f"roboclaw@{self.address:#04x}"
        if self.fault_latched or self.implausible:
            st.level, st.message = ERROR, "link fault latched" if self.fault_latched else "implausible encoder jump"
        elif s.failures or s.retries:
            st.level, st.message = WARN, "retries/failures seen"
        elif self.battery_v is not None and self.battery_v < self.low_voltage_warn_v:
            st.level, st.message = WARN, f"battery low {self.battery_v:.2f} V"
        else:
            st.level, st.message = OK, "read-only (rung 2a)"
        lat = s.latency_summary() or {}
        kv = {
            "transactions": s.transactions, "retries": s.retries, "failures": s.failures,
            "crc_errors": s.crc_errors, "short_reads": s.short_reads,
            "implausible_jumps": self.implausible,
            "latency_p99_ms": f"{lat.get('p99', float('nan')):.2f}",
            "battery_v_corrected": "n/a" if self.battery_v is None else f"{self.battery_v:.2f}",
        }
        st.values = [KeyValue(key=k, value=str(v)) for k, v in kv.items()]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.diag_pub.publish(arr)

    def summary_text(self):
        s = self.link.stats
        lat = s.latency_summary()
        lat_txt = ("latency ms p50 {p50:.2f} p95 {p95:.2f} p99 {p99:.2f} max {max:.2f} (n={n})"
                   .format(**lat) if lat else "latency: no data")
        per = sorted(self.periods_ms)
        per_txt = (f"period ms mean {sum(per) / len(per):.2f} max {per[-1]:.2f}"
                   if per else "period: no data")
        return (f"{lat_txt} | {per_txt} | ok {s.transactions} retry {s.retries} "
                f"fail {s.failures} crc {s.crc_errors} short {s.short_reads} "
                f"implausible {self.implausible}")

    def log_stats(self):
        self.get_logger().info(self.summary_text())

    def final_report(self):
        cpm_l, cpm_r = self.geo.counts_per_m_left, self.geo.counts_per_m_right
        self.get_logger().info("==== RUNG 2a FINAL ====")
        self.get_logger().info(self.summary_text())
        self.get_logger().info(
            f"accumulated counts M1(L) {self.acc_m1}  M2(R) {self.acc_m2}")
        self.get_logger().info(
            f"from counts: L {self.acc_m1 / cpm_l:.4f} m  R {self.acc_m2 / cpm_r:.4f} m  "
            f"mean {(self.acc_m1 / cpm_l + self.acc_m2 / cpm_r) / 2:.4f} m")
        self.get_logger().info(
            f"odom: path {self.dist_path:.4f} m  x {self.x:.4f}  y {self.y:.4f}  "
            f"yaw {math.degrees(self.theta):.2f} deg")
        if self.prev_raw is not None:
            self.get_logger().info(f"last raw M1 {self.prev_raw[0]}  M2 {self.prev_raw[1]}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CartDriveNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.final_report()
            node.link.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
