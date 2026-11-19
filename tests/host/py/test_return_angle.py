# Feature: obstacle return angle (OBST_RETURN_DEG, 19 Nov)
"""The trail before the turn back to the path keeps the back corner clear
for any return angle theta in 30..90 deg.

obstacle_side_trail_mm() / obstacle_side_trail90_mm() give the distance T to
drive on before turning back at theta. With the corner at most C ahead of
the car's centre before the trail (C = A + g tan15, or A cos30 +
(g - A sin30) tan15 for the look-before-turn variant), it is C - T ahead
after it, and the return line passes it at

    g cos(theta) - (C - T) sin(theta)  >=  half + margin.

At 45 deg the result equals the formula used before 19 Nov.
"""

import ctypes
import math

import pytest

F = ctypes.c_float
TAN15 = math.tan(math.radians(15.0))
FLOOR_MM = 50.0
ANGLES = [30.0, 45.0, 60.0, 75.0, 90.0]
GAPS = [100.0, 140.0, 200.0, 240.0]
A_MM, HALF_MM, MARGIN_MM = 80.0, 80.0, 60.0


@pytest.fixture(scope="module")
def trails(lib):
    f1 = lib.obstacle_side_trail_mm
    f2 = lib.obstacle_side_trail90_mm
    for f in (f1, f2):
        f.argtypes = [F, F, F, F, F]
        f.restype = F
    return f1, f2


def corner_ahead(gap, variant):
    if variant == 0:
        return A_MM + gap * TAN15
    return A_MM * math.cos(math.radians(30.0)) + (gap - A_MM * 0.5) * TAN15


@pytest.mark.parametrize("variant", [0, 1])
@pytest.mark.parametrize("theta", ANGLES)
@pytest.mark.parametrize("gap", GAPS)
def test_corner_clear_after_trail(trails, variant, theta, gap):
    t = trails[variant](gap, A_MM, HALF_MM, MARGIN_MM, theta)
    dy = corner_ahead(gap, variant) - t
    th = math.radians(theta)
    clearance = gap * math.cos(th) - dy * math.sin(th)
    assert t >= FLOOR_MM - 1e-3
    assert clearance >= HALF_MM + MARGIN_MM - 0.05


@pytest.mark.parametrize("variant", [0, 1])
@pytest.mark.parametrize("gap", GAPS)
def test_45_matches_old_formula(trails, variant, gap):
    old = corner_ahead(gap, variant) - gap + math.sqrt(2.0) * (HALF_MM + MARGIN_MM)
    old = max(old, FLOOR_MM)
    assert trails[variant](gap, A_MM, HALF_MM, MARGIN_MM, 45.0) == pytest.approx(old, abs=0.05)


def test_steeper_needs_longer_trail(trails):
    t45 = trails[0](140.0, A_MM, HALF_MM, MARGIN_MM, 45.0)
    t60 = trails[0](140.0, A_MM, HALF_MM, MARGIN_MM, 60.0)
    assert t60 > t45
