"""Tilt hump detector (imu_logic.c, HUMP_TILT_DETECT).

Synthetic 20 ms samples: a one-sided hump under one rear wheel, a
full-width hump (pitch only), flat floor with noise, a single jolt; the
mounting-independent tilt angle; the packed one-byte state.
"""

import ctypes
import math
import random

import pytest

F = ctypes.c_float
LEVER = 100.0                      # HUMP_TILT_LEVER_MM, calibrated on the car 3 Nov


class HumpParams(ctypes.Structure):
    _fields_ = [("start_deg", F), ("level_deg", F), ("min_height_mm", F),
                ("start_samples", ctypes.c_uint32), ("end_samples", ctypes.c_uint32), ("max_len_mm", F)]


class TiltHump(ctypes.Structure):
    _fields_ = [("f", ctypes.c_uint8), ("h_pitch_mm", F), ("lift_mm", F), ("peak_mm", F), ("len_mm", F)]


class HumpOut(ctypes.Structure):
    _fields_ = [("b_start", ctypes.c_bool), ("b_peak", ctypes.c_bool), ("b_end", ctypes.c_bool),
                ("b_valid", ctypes.c_bool), ("height_mm", F), ("len_mm", F)]


PAR = HumpParams(2.5, 1.0, 5.0, 3, 5, 1500.0)


@pytest.fixture(scope="module")
def th(lib):
    lib.tilt_hump_reset.restype, lib.tilt_hump_reset.argtypes = None, [ctypes.POINTER(TiltHump)]
    lib.tilt_hump_step.restype = HumpOut
    lib.tilt_hump_step.argtypes = [ctypes.POINTER(TiltHump), ctypes.POINTER(HumpParams), F, F, F, F]
    return lib


def run(th, samples, ds=3.0, state=None):
    t = state or TiltHump()
    if state is None:
        th.tilt_hump_reset(ctypes.byref(t))
    return [th.tilt_hump_step(ctypes.byref(t), ctypes.byref(PAR), p, tl, LEVER, ds) for p, tl in samples], t


def one_sided(lift_mm, n=40):
    """(pitch, total tilt) for a wheel lift rising then falling (half sine)."""
    out = []
    for i in range(n):
        z = lift_mm * math.sin(math.pi * i / (n - 1))
        tilt = math.degrees(math.asin(z / LEVER))
        out.append((-0.45 * tilt, tilt))              # nose-down share of the tilt, as measured
    return [(0.0, 0.0)] * 10 + out + [(0.0, 0.0)] * 10


@pytest.mark.parametrize("lift", [8.5, 10.0, 30.5])
def test_one_sided(th, lift):
    outs, _ = run(th, one_sided(lift))
    assert sum(o.b_start for o in outs) == 1 and sum(o.b_peak for o in outs) == 1
    ends = [o for o in outs if o.b_end]
    assert len(ends) == 1 and ends[0].b_valid
    assert ends[0].height_mm == pytest.approx(lift, abs=0.5)


def test_bench_readings_give_ruler_heights(lib):
    fn = lib.imu_lift_mm
    fn.restype, fn.argtypes = F, [F, F]
    assert fn(LEVER, 5.8) == pytest.approx(10.1, abs=0.2)       # 1 cm hump, 3 Nov
    assert fn(LEVER, 17.7) == pytest.approx(30.4, abs=0.2)      # 3 cm box, 3 Nov


def test_full_width_hump_uses_pitch(th):
    # 6 deg climb then descent, 30 samples each at 3 mm; the first 3 samples only start it
    s = [(0.0, 0.0)] * 5 + [(6.0, 6.0)] * 30 + [(-6.0, 6.0)] * 30 + [(0.0, 0.0)] * 10
    ends = [o for o in run(th, s)[0] if o.b_end]
    assert len(ends) == 1 and ends[0].b_valid
    assert ends[0].height_mm >= 8.4                              # max(lever x sin 6, pitch height)


def test_flat_noise_no_hump(th):
    rng = random.Random(1)
    s = [(rng.uniform(-0.8, 0.8), abs(rng.uniform(-0.9, 0.9))) for _ in range(2000)]
    assert not any(o.b_start for o in run(th, s)[0])


def test_single_jolt_ignored(th):
    s = [(0.0, 0.0)] * 10 + [(-15.0, 15.0)] + [(0.0, 0.0)] * 10
    assert not any(o.b_start for o in run(th, s)[0])


def test_small_tilt_is_false_hump(th):
    s = [(0.0, 0.0)] * 5 + [(0.0, 2.6)] * 10 + [(0.0, 0.0)] * 10   # 100 x sin 2.6 = 4.5 mm < 5
    ends = [o for o in run(th, s)[0] if o.b_end]
    assert len(ends) == 1 and not ends[0].b_valid and ends[0].height_mm == 0.0


def test_state_is_one_byte_and_counters_saturate(th):
    assert ctypes.sizeof(ctypes.c_uint8) == 1
    _, t = run(th, [(0.0, 0.0)] * 50)
    assert (t.f >> 5) & 7 == 7 and t.f & 1 == 0                  # n_level held at 7, off
    _, t = run(th, [(0.0, 9.0)] * 50)
    assert t.f & 1 == 1 and (t.f >> 2) & 7 == 7                  # on, n_tilt held at 7


def _rot(v, axis, deg):
    c, s = math.cos(math.radians(deg)), math.sin(math.radians(deg))
    x, y, z = v
    if axis == "x":
        return (x, c * y - s * z, s * y + c * z)
    if axis == "y":
        return (c * x + s * z, y, -s * x + c * z)
    return (c * x - s * y, s * x + c * y, z)


@pytest.mark.parametrize("g0", [(0.0, 0.0, 1.0), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0)])   # flat, on its side, on end
@pytest.mark.parametrize("deg", [1.0, 5.8, 17.7])
def test_tilt_any_mounting(lib, g0, deg):
    fn = lib.imu_tilt_deg
    fn.restype, fn.argtypes = F, [ctypes.POINTER(F * 3), ctypes.POINTER(F * 3)]
    for axis in "xyz":
        g = _rot(g0, axis, deg)
        got = fn(ctypes.byref((F * 3)(*g)), ctypes.byref((F * 3)(*g0)))
        along = abs(dict(zip("xyz", g0))[axis]) > 0.5               # rotating about gravity itself: no tilt
        assert got == pytest.approx(0.0 if along else deg, abs=0.05)
    assert fn(ctypes.byref((F * 3)(*g0)), ctypes.byref((F * 3)(0.0, 0.0, 0.0))) == 0.0   # no level yet
