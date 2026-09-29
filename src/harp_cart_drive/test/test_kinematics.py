"""Offline checks of rung 2a math. Run: python3 -m pytest test/ (no ROS needed)."""
import math
import random

import pytest

from harp_cart_drive.kinematics import (
    DriveGeometry, body_increment, integrate_midpoint, inverse, wheel_distances, wrap_delta)

GEO = DriveGeometry()


def forward_pose(geo, d1, d2, steps=1):
    x = y = th = 0.0
    for _ in range(steps):
        dl, dr = wheel_distances(geo, d1 / steps, d2 / steps)
        ds, dth = body_increment(geo, dl, dr)
        x, y, th = integrate_midpoint(x, y, th, ds, dth)
    return x, y, th


def test_counts_per_meter():
    assert GEO.counts_per_m_left == pytest.approx(4735.1, abs=0.1)
    assert GEO.counts_per_m_right == pytest.approx(4735.1, abs=0.1)


def test_wrap_delta():
    assert wrap_delta(5, 2**32 - 5) == 10          # forward across wrap
    assert wrap_delta(2**32 - 5, 5) == -10         # reverse across wrap
    assert wrap_delta(1000, 400) == 600


def test_straight_is_exact():
    x, y, th = forward_pose(GEO, 23675, 23675)     # 5 m at 4735.1 counts/m
    assert x == pytest.approx(23675 / GEO.counts_per_m_left, rel=1e-12)
    assert y == 0.0 and th == 0.0


@pytest.mark.parametrize("m1,m2,sign", [
    (-6490, 6342, +1), (-6489, 6365, +1), (-6503, 6344, +1),   # bring-up CCW rows
    (6560, -6272, -1), (6554, -6261, -1), (6557, -6264, -1),   # bring-up CW rows
])
def test_bringup_spin_rows_give_correct_yaw_sign(m1, m2, sign):
    _, _, th = forward_pose(GEO, m1, m2, steps=200)
    # Encoder rotation ~361 deg -> wraps near 0; use unwrapped sum instead
    dl, dr = wheel_distances(GEO, m1, m2)
    _, dth = body_increment(GEO, dl, dr)
    assert math.copysign(1, dth) == sign
    assert abs(math.degrees(dth)) == pytest.approx(361, abs=1.5)


def test_chunked_integration_matches_lump_on_straight():
    random.seed(1)
    total, chunks = 0, []
    while total < 23675:
        c = random.randint(0, 60)
        chunks.append(c)
        total += c
    x = y = th = 0.0
    for c in chunks:
        dl, dr = wheel_distances(GEO, c, c)
        ds, dth = body_increment(GEO, dl, dr)
        x, y, th = integrate_midpoint(x, y, th, ds, dth)
    assert x == pytest.approx(total / GEO.counts_per_m_left, rel=1e-12)


def test_arc_midpoint_accuracy_at_20hz_max_spin_rate():
    # Quarter circle, R = 0.5 m, at 112 deg/s-class per-cycle increments.
    R, b = 0.5, GEO.wheel_separation_m
    arc = math.pi / 2
    n = 20
    dl = (R - b / 2) * arc / n
    dr = (R + b / 2) * arc / n
    x = y = th = 0.0
    for _ in range(n):
        ds, dth = body_increment(GEO, dl, dr)
        x, y, th = integrate_midpoint(x, y, th, ds, dth)
    assert x == pytest.approx(R, abs=1e-3) and y == pytest.approx(R, abs=1e-3)


def test_larger_left_wheel_curves_right():
    geo = DriveGeometry(left_multiplier=96.0 / 95.8, right_multiplier=95.6 / 95.8)
    x, y, th = forward_pose(geo, 23675, 23675, steps=500)   # equal counts, ~5 m
    assert th < 0 and y < 0                                  # CW, to the right
    assert -y * 1000 == pytest.approx(116, rel=0.05)         # predicted 116 mm


def test_inverse_test_speed_and_signs():
    assert inverse(GEO, 0.21118839, 0.0, 2000) == (1000, 1000)
    m1, m2 = inverse(GEO, 0.0, 1.0, 2000)                    # CCW spin
    assert m1 < 0 < m2 and m1 == -m2


def test_saturation_preserves_curvature():
    m1, m2 = inverse(GEO, 0.42, 1.0, 2000)
    assert max(abs(m1), abs(m2)) == 2000
    u1 = (0.42 - 1.0 * 0.215) * GEO.counts_per_m_left
    u2 = (0.42 + 1.0 * 0.215) * GEO.counts_per_m_right
    assert m1 / m2 == pytest.approx(u1 / u2, rel=2e-3)


def test_inverse_forward_round_trip():
    for v, w in [(0.3, 0.0), (0.2, 0.5), (-0.1, -0.8), (0.0, 1.5)]:
        m1, m2 = inverse(GEO, v, w, 1e9)
        dl, dr = wheel_distances(GEO, m1, m2)
        ds, dth = body_increment(GEO, dl, dr)
        assert ds == pytest.approx(v, abs=3e-4) and dth == pytest.approx(w, abs=2e-3)


# ---- rung 2b hand-calculated target table (independent of inverse()) ------
@pytest.mark.parametrize("v,w,m1,m2", [
    (0.2112, 0.0, 1000, 1000),      # 0.2112 x 4735.1 = 1000.1
    (-0.2112, 0.0, -1000, -1000),
    (0.0, 0.5, -509, 509),          # 0.5 x 0.215 = 0.1075 m/s x 4735.1 = 509.0
    (0.42, 1.0, 646, 2000),         # 970.7 / 3006.8 scaled by 2000/3006.8
])
def test_rung2b_target_table(v, w, m1, m2):
    assert inverse(GEO, v, w, 2000) == (m1, m2)
