#!/usr/bin/env python3
"""
v_axis_stream_test.py -- HARP V-Axis 1D bench controller stand-in.

Models the OUTPUT CONTRACT of the future stair-climb controller (Pi): it streams
the current screw-mm setpoint on /v_axis/cmd_mm at a fixed rate (default 10 Hz),
whether or not the value changed. That continuous stream IS the liveness signal
the ESP32 1D watchdog keys on. Kill this node (Ctrl-C) to simulate stream loss
and prove the ESP32 trips within the watchdog window.

RE-ARM (1D): this node also owns a persistent /v_axis/reset publisher
(std_msgs/Bool). Press Enter in this terminal to publish {data: true} and re-arm
the ESP32 fail-safe latch INSTANTLY -- because the publisher is discovered once
at startup, there is none of the 3-5 s cold-start discovery lag a fresh
`ros2 topic pub` suffers. This mirrors how the production Pi controller will
re-arm from its own already-running node. Re-arm only holds if the stream is
live (reset stamps liveness; no stream -> the watchdog re-trips in ~500 ms).

Change the held target at runtime by publishing ONE Float32 to /v_axis/set_target.

Params:
  rate_hz     (10.0)  setpoint stream rate -- must comfortably beat V_WATCHDOG_MS
  cmd_topic   (/v_axis/cmd_mm)      streamed setpoint (Float32, screw-mm)
  set_topic   (/v_axis/set_target)  operator target input (Float32, screw-mm)
  reset_topic (/v_axis/reset)       fail-safe re-arm (Bool)
  target_mm   (0.0)   initial held setpoint (0 = storage/home datum)
  slew_mms    (0.0)   0 = instant step; >0 = ramp toward target at mm/s
                      (~1.26 models the RoboClaw move speed for mid-move tests)
"""

import threading

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool


class VAxisStreamTest(Node):
    def __init__(self):
        super().__init__('v_axis_stream_test')

        self.declare_parameter('rate_hz', 10.0)
        self.declare_parameter('cmd_topic', '/v_axis/cmd_mm')
        self.declare_parameter('set_topic', '/v_axis/set_target')
        self.declare_parameter('reset_topic', '/v_axis/reset')
        self.declare_parameter('target_mm', 0.0)
        self.declare_parameter('slew_mms', 0.0)

        self.rate_hz  = float(self.get_parameter('rate_hz').value)
        self.slew_mms = float(self.get_parameter('slew_mms').value)
        cmd_topic     = self.get_parameter('cmd_topic').value
        set_topic     = self.get_parameter('set_topic').value
        reset_topic   = self.get_parameter('reset_topic').value

        self.target  = float(self.get_parameter('target_mm').value)  # operator intent
        self.current = self.target                                   # streamed value

        self.pub       = self.create_publisher(Float32, cmd_topic, 10)
        self.reset_pub = self.create_publisher(Bool, reset_topic, 10)  # discovered once
        self.create_subscription(Float32, set_topic, self._on_set, 10)

        self.dt = 1.0 / self.rate_hz
        self.create_timer(self.dt, self._tick)

        # Background thread: Enter -> publish reset(true). Daemon so Ctrl-C exits.
        self._rearm = threading.Thread(target=self._rearm_loop, daemon=True)
        self._rearm.start()

        self.get_logger().info(
            f'streaming {cmd_topic} @ {self.rate_hz:.1f} Hz | '
            f'target={self.target:.2f} mm | slew={self.slew_mms:.2f} mm/s (0=step)')
        self.get_logger().info(
            f'set new target via {set_topic} | press ENTER here to re-arm '
            f'({reset_topic} Bool true)')

    def _on_set(self, msg: Float32):
        self.target = float(msg.data)
        self.get_logger().info(f'new target = {self.target:.2f} mm')

    def _tick(self):
        if self.slew_mms > 0.0:
            step = self.slew_mms * self.dt
            delta = self.target - self.current
            if abs(delta) <= step:
                self.current = self.target
            else:
                self.current += step if delta > 0 else -step
        else:
            self.current = self.target
        self.pub.publish(Float32(data=self.current))

    def _rearm_loop(self):
        while rclpy.ok():
            try:
                input()  # blocks until Enter
            except EOFError:
                return
            self.reset_pub.publish(Bool(data=True))
            self.get_logger().info('re-arm sent (/v_axis/reset true)')


def main():
    rclpy.init()
    node = VAxisStreamTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
