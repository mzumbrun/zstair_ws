"""HARP cart differential-drive kinematics. Pure Python, no ROS imports.

Conventions (REP-103): +x forward, +y left, yaw CCW positive.
RoboClaw M1 = LEFT wheel (driver side), M2 = RIGHT wheel.
Motor reversal and encoder-2 inversion live in the RoboClaw, so +counts
means forward on both wheels and no sign flips are applied here.

Pose comes from absolute encoder-count differences (exact, jitter-immune).
Twist comes from delta-counts / delta-t (host monotonic clock).
"""

import math
from dataclasses import dataclass

INT32_SPAN = 1 << 32
INT32_HALF = 1 << 31


def wrap_delta(new_raw: int, old_raw: int) -> int:
    """Signed int32 difference of two raw 32-bit counter readings.

    Correct across counter wrap as long as the true change is < 2^31 counts
    between reads (at 2,000 counts/s that is 12.4 days, so always true here).
    """
    return ((new_raw - old_raw + INT32_HALF) % INT32_SPAN) - INT32_HALF


@dataclass(frozen=True)
class DriveGeometry:
    counts_per_rev: float = 1425.1
    wheel_diameter_m: float = 0.0958
    wheel_separation_m: float = 0.430
    # Per-wheel multipliers on the effective diameter. 1.0 = symmetric model
    # (Step 2 acceptance). Later rung: left 1.0021, right 0.9979.
    left_multiplier: float = 1.0
    right_multiplier: float = 1.0

    @property
    def counts_per_m_left(self) -> float:
        return self.counts_per_rev / (math.pi * self.wheel_diameter_m * self.left_multiplier)

    @property
    def counts_per_m_right(self) -> float:
        return self.counts_per_rev / (math.pi * self.wheel_diameter_m * self.right_multiplier)


def wheel_distances(geo: DriveGeometry, d_m1_counts: int, d_m2_counts: int):
    """Encoder deltas -> (left, right) wheel travel in meters."""
    return d_m1_counts / geo.counts_per_m_left, d_m2_counts / geo.counts_per_m_right


def body_increment(geo: DriveGeometry, d_left: float, d_right: float):
    """Wheel travel -> (distance along path, heading change)."""
    ds = 0.5 * (d_left + d_right)
    dtheta = (d_right - d_left) / geo.wheel_separation_m
    return ds, dtheta


def integrate_midpoint(x: float, y: float, theta: float, ds: float, dtheta: float):
    """Second-order (midpoint) pose update. Exact for straight lines."""
    th_mid = theta + 0.5 * dtheta
    x += ds * math.cos(th_mid)
    y += ds * math.sin(th_mid)
    theta = math.atan2(math.sin(theta + dtheta), math.cos(theta + dtheta))
    return x, y, theta


def inverse(geo: DriveGeometry, v: float, omega: float, cap_counts_s: float):
    """cmd_vel (v m/s, omega rad/s) -> (M1, M2) integer counts/s.

    Not used in rung 2a (read-only). Included so it is unit-tested before
    rung 2b sends anything to the motors.
    Saturation scales BOTH wheels by one factor, preserving curvature.
    """
    half_b = 0.5 * geo.wheel_separation_m
    m1 = (v - omega * half_b) * geo.counts_per_m_left
    m2 = (v + omega * half_b) * geo.counts_per_m_right
    peak = max(abs(m1), abs(m2))
    if peak > cap_counts_s > 0:
        k = cap_counts_s / peak
        m1 *= k
        m2 *= k
    return int(round(m1)), int(round(m2))
