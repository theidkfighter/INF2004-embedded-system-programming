"""Lift / overturn detector (imu_logic.c lift_step; 16 Nov requirement change).

The IMU collision detection is removed. While driving, the robot stops its
motors when it is lifted (pushed upwards: normal end of the run) or
overturned (tilted 60 deg or more: E-stop). Synthetic
20 ms accelerometer samples (g) in several IMU mountings: flat driving with
noise, the course hump, single jolts, a quick lift, a slow tip, an overturn.
"""

import ctypes
import math
import random

import pytest

F, U8 = ctypes.c_float, ctypes.c_uint8
LIFT_NONE, LIFT_OVER, LIFT_UP = 0, 1, 2


class LiftParams(ctypes.Structure):
    _fields_ = [("tilt_deg", F), ("up_g", F), ("alpha", F), ("up_alpha", F),
                ("tilt_samples", U8), ("up_samples", U8), ("blank_samples", U8)]


class LiftCtx(ctypes.Structure):
    _fields_ = [("ref", F * 3), ("lp", F * 3), ("up_lp", F), ("n_blank", U8), ("n_tilt", U8),
                ("n_up", U8), ("cause", U8)]


PAR = LiftParams(60.0, 0.25, 0.2, 1.0, 10, 3, 20)     # robot_config.h LIFT_*
BLANK = 20


@pytest.fixture(scope="module")
def lf(lib):
    lib.lift_reset.restype, lib.lift_reset.argtypes = None, [ctypes.POINTER(LiftCtx)]
    lib.lift_step.restype = ctypes.c_int
    lib.lift_step.argtypes = [ctypes.POINTER(LiftCtx), ctypes.POINTER(LiftParams), F, F, F]
    lib.lift_tilt_deg.restype, lib.lift_tilt_deg.argtypes = F, [ctypes.POINTER(LiftCtx)]
    return lib


def rot(v, mount):
    """Express a robot-frame vector (x fwd, y left, z up) in an IMU mounting."""
    x, y, z = v
    return {"flat": (x, y, z), "upside": (x, -y, -z), "side": (x, z, -y), "back": (-z, y, x)}[mount]


def tilted(deg, axis="roll"):
    r = math.radians(deg)
    return (0.0, math.sin(r), math.cos(r)) if axis == "roll" else (-math.sin(r), 0.0, math.cos(r))


def run(lf, samples, mount="flat"):
    """Returns (index of the first non-zero cause or None, that cause, ctx)."""
    c = LiftCtx()
    lf.lift_reset(ctypes.byref(c))
    for i, v in enumerate(samples):
        cause = lf.lift_step(ctypes.byref(c), ctypes.byref(PAR), *rot(v, mount))
        if cause != LIFT_NONE:
            return i, cause, c
    return None, LIFT_NONE, c


def floor(n, rng, noise=0.04):
    """Driving on flat floor: vibration on every axis, small fore/aft pulls."""
    return [(rng.gauss(0, noise) + 0.05 * math.sin(i / 7), rng.gauss(0, noise), 1.0 + rng.gauss(0, noise))
            for i in range(n)]


MOUNTS = ["flat", "upside", "side", "back"]


@pytest.mark.parametrize("mount", MOUNTS)
def test_flat_driving_never_stops(lf, mount):
    rng = random.Random(1)
    i, cause, _ = run(lf, floor(3000, rng), mount)      # 60 s
    assert i is None and cause == LIFT_NONE


@pytest.mark.parametrize("mount", MOUNTS)
def test_course_hump_is_not_a_lift(lf, mount):
    """The 1 cm one-sided hump tilts the robot 6-8 deg for about a second."""
    rng = random.Random(2)
    hump = [tilted(8.0 * math.sin(math.pi * k / 49)) for k in range(50)]
    hump = [(x + rng.gauss(0, 0.04), y + rng.gauss(0, 0.04), z + rng.gauss(0, 0.04)) for x, y, z in hump]
    i, _, _ = run(lf, floor(BLANK + 50, rng) + hump + floor(100, rng), mount)
    assert i is None


@pytest.mark.parametrize("mount", MOUNTS)
def test_single_jolts_are_ignored(lf, mount):
    """One-sample spikes (ramp edge, junction, bump): 2.5 g up or sideways."""
    rng = random.Random(3)
    s = floor(BLANK + 40, rng)
    for k in range(10):
        s += [(0.0, 0.0, 3.5 if k % 2 else -1.0)] + floor(30, rng)
        s += [(2.5, 1.5, 1.0)] + floor(30, rng)
    i, _, _ = run(lf, s, mount)
    assert i is None


@pytest.mark.parametrize("mount", MOUNTS)
@pytest.mark.parametrize("up_g", [0.4, 0.8])
def test_quick_lift_stops_within_100_ms(lf, mount, up_g):
    rng = random.Random(4)
    before = floor(BLANK + 30, rng)
    lift = [(0.0, 0.0, 1.0 + up_g)] * 8 + [(0.0, 0.0, 1.0 - up_g)] * 8 + floor(50, rng)
    i, cause, _ = run(lf, before + lift, mount)
    assert cause == LIFT_UP
    assert len(before) <= i <= len(before) + 4          # 3 to 5 samples: 60 to 100 ms


@pytest.mark.parametrize("mount", MOUNTS)
@pytest.mark.parametrize("axis", ["roll", "pitch"])
def test_tip_below_60_deg_is_not_a_stop(lf, mount, axis):
    """Tipped to 45 deg with no upward push: not overturned (known limit for a gentle lift)."""
    rng = random.Random(5)
    tip = [tilted(min(k, 45), axis) for k in range(0, 120)]
    i, cause, _ = run(lf, floor(BLANK + 30, rng) + tip, mount)
    assert i is None and cause == LIFT_NONE


@pytest.mark.parametrize("mount", MOUNTS)
@pytest.mark.parametrize("axis", ["roll", "pitch"])
def test_tip_past_60_deg_is_overturned(lf, mount, axis):
    rng = random.Random(9)
    before = floor(BLANK + 30, rng)
    tip = [tilted(min(k, 80), axis) for k in range(0, 120)]          # 1 deg per sample
    i, cause, c = run(lf, before + tip, mount)
    assert cause == LIFT_OVER
    assert 60 <= lf.lift_tilt_deg(ctypes.byref(c)) < 75
    assert i - len(before) < 75


@pytest.mark.parametrize("mount", MOUNTS)
def test_overturned_stops(lf, mount):
    rng = random.Random(6)
    before = floor(BLANK + 30, rng)
    flip = [tilted(min(180.0, 12.0 * k), "roll") for k in range(40)]     # rolled over in 0.3 s
    i, cause, _ = run(lf, before + flip, mount)
    assert cause == LIFT_OVER and i - len(before) <= 20                  # within 0.4 s of starting to roll


def test_reference_is_the_start_posture(lf):
    """Starting on a slope (or with a tilted IMU) is the reference, not a lift."""
    rng = random.Random(7)
    slope = [tilted(20.0, "pitch")] * (BLANK + 200)
    i, _, _ = run(lf, [(x + rng.gauss(0, 0.03), y, z) for x, y, z in slope])
    assert i is None


def test_no_check_during_blanking_and_latched_once(lf):
    c = LiftCtx()
    lf.lift_reset(ctypes.byref(c))
    for _ in range(BLANK - 1):                          # still learning the reference
        assert lf.lift_step(ctypes.byref(c), ctypes.byref(PAR), 0.0, 0.0, 1.0) == LIFT_NONE
    assert lf.lift_tilt_deg(ctypes.byref(c)) == 0.0
    lf.lift_step(ctypes.byref(c), ctypes.byref(PAR), 0.0, 0.0, 1.0)
    got = [lf.lift_step(ctypes.byref(c), ctypes.byref(PAR), 0.0, 0.0, 1.6) for _ in range(10)]
    assert got.count(LIFT_UP) == 1 and c.cause == LIFT_UP   # reported once, then latched
    lf.lift_reset(ctypes.byref(c))
    assert c.cause == LIFT_NONE and c.n_blank == 0
