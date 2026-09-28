"""Independent encoder snapshot for the rung 2a identity check. No ROS.

Run with the drive node STOPPED (it owns the port while running):

  ros2 run harp_cart_drive enc_snapshot --save /tmp/before.json
  # start node, move cart, stop node (Ctrl-C prints the FINAL report)
  ros2 run harp_cart_drive enc_snapshot --compare /tmp/before.json

The compare delta must equal the node's "accumulated counts" within 1 count.
"""

import argparse
import json
import sys

from harp_cart_drive.kinematics import DriveGeometry, wrap_delta
from harp_cart_drive.roboclaw_link import RoboClawLink

DEFAULT_PORT = "/dev/serial/by-id/usb-Basicmicro_Inc._USB_Roboclaw_2x15A-if00"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=DEFAULT_PORT)
    ap.add_argument("--address", type=lambda s: int(s, 0), default=0x80)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--save", metavar="FILE")
    g.add_argument("--compare", metavar="FILE")
    a = ap.parse_args()

    link = RoboClawLink(a.port, a.address)
    raw = link.read_encoders()
    link.close()
    if raw is None:
        sys.exit("No valid encoder reply (cmd 78)")
    print(f"raw M1(L) {raw[0]}  M2(R) {raw[1]}")

    if a.save:
        with open(a.save, "w") as f:
            json.dump({"m1": raw[0], "m2": raw[1]}, f)
        print(f"saved -> {a.save}")
        return

    with open(a.compare) as f:
        b = json.load(f)
    geo = DriveGeometry()
    d1, d2 = wrap_delta(raw[0], b["m1"]), wrap_delta(raw[1], b["m2"])
    dl, dr = d1 / geo.counts_per_m_left, d2 / geo.counts_per_m_right
    print(f"delta counts M1(L) {d1}  M2(R) {d2}")
    print(f"distance L {dl:.4f} m  R {dr:.4f} m  mean {(dl + dr) / 2:.4f} m")
    print("PASS if node 'accumulated counts' match these deltas within 1 count")


if __name__ == "__main__":
    main()
