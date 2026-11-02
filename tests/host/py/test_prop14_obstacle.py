# Feature: minimum-bit-data-representation, Property 14: Obstacle planning matches the Baseline_Build
"""Property 14: for the same fine scan and coarse sweep, obstacle_profile,
obstacle_choose_side and bypass_plan of this build (libis02.so) and of the
Baseline_Build (libbase.so) give the same profile, side and plan, bit for bit.

The steps run in the order of the task in tsk_obstacle.c, minimum width
included. On this build's side the status writes and the obstacle message go
in between, as they do on the car, so a write that disturbed the working
floats would show as a difference.

The planner logic left in tsk_obstacle.c is kernel code and is not called here.

**Validates: Requirements 6.7**
"""

import ctypes
import struct
from collections import Counter

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from test_msg_codec import F, ObstProfile, Words6

TITLE = "Obstacle planning matches the Baseline_Build"
B, CH, U32 = ctypes.c_bool, ctypes.c_char, ctypes.c_uint32
PF, EXAMPLES = ctypes.POINTER(F), 1000
OBS_MAX_POINTS, COARSE_N = 64, 5
NO_ECHO = -1.0
OBST_BLOCKED_L, OBST_BLOCKED_R, OBST_CHOOSE_LEGACY = 1, 2, 4


class Scan(ctypes.Structure):
    """scan_t, the same in both builds."""

    _fields_ = [("deg", F * OBS_MAX_POINTS), ("cm", F * OBS_MAX_POINTS), ("n", U32)]


class Plan(ctypes.Structure):
    """bypass_plan_t, the same in both builds (task 10.3 is skipped)."""

    _fields_ = [("side", ctypes.c_int8), ("turn1_deg", F), ("d1_mm", F), ("turn2_deg", F),
                ("d2_mm", F), ("turn3_deg", F), ("shift_mm", F), ("return_max_mm", F),
                ("realign_deg", F)]


P = ctypes.POINTER
SIGNATURES = {
    "obstacle_profile": (B, [P(Scan), PF, F, F, P(ObstProfile)]),
    "obstacle_apply_min_width": (None, [P(ObstProfile), F, PF, PF]),
    "obstacle_choose_side": (CH, [P(ObstProfile), F, F, U32]),
    "obstacle_is_tie": (B, [P(ObstProfile), F, F, U32]),
    "obstacle_side_room": (B, [P(ObstProfile), CH, F, F]),
    "obstacle_shift_mm": (F, [P(ObstProfile), CH, F, F]),
    "obstacle_standoff_mm": (F, [P(ObstProfile), CH, F, F]),
    "bypass_plan": (B, [P(ObstProfile), F, F, F, F, P(Plan)]),
}


class Planner:
    """The planner functions of one build."""

    def __init__(self, mod, prefix):
        for name, (restype, argtypes) in SIGNATURES.items():
            fn = getattr(mod, prefix + name)
            fn.restype, fn.argtypes = restype, argtypes
            setattr(self, name, fn)


@pytest.fixture(scope="module")
def pair(lib, base):
    for name, restype, argtypes in (("msg_obst_pack", None, [P(ObstProfile), P(Words6)]),
                                    ("st_set_obst_cm", None, [F]), ("st_set_scan", None, [U32, F]),
                                    ("st_set_scan_side", None, [CH])):
        fn = getattr(lib, name)
        fn.restype, fn.argtypes = restype, argtypes
    return Planner(base, "base_"), Planner(lib, ""), lib


def bits(x):
    return struct.pack("<f", x)


def fields(s):
    """Every field of a ctypes struct, floats as their bits (padding is not compared)."""
    out = []
    for name, _ in s._fields_:
        v = getattr(s, name)
        if isinstance(v, float):
            out.append((name, bits(v)))
        elif isinstance(v, ctypes.Array):
            out.append((name, [bits(x) for x in v]))
        else:
            out.append((name, v))
    return out


def make_scan(points):
    s = Scan()
    s.n = len(points)
    for i, (deg, cm) in enumerate(points):
        s.deg[i], s.cm[i] = deg, cm
    return s


OUTCOMES = Counter()


def plan_case(pair, points, coarse_cm, half, margin, depth, max_lat, min_width):
    old, new, lib = pair
    scan, sweep = make_scan(points), (F * COARSE_N)(*coarse_cm)
    p_old, p_new = ObstProfile(), ObstProfile()

    ok_old = old.obstacle_profile(ctypes.byref(scan), sweep, half, margin, ctypes.byref(p_old))
    ok_new = new.obstacle_profile(ctypes.byref(scan), sweep, half, margin, ctypes.byref(p_new))
    assert ok_new == ok_old
    assert fields(p_new) == fields(p_old)

    # the status writes and the obstacle message of the task, this build only
    for i, cm in enumerate(coarse_cm):
        lib.st_set_scan(i, cm)
    lib.st_set_obst_cm(p_new.r_min_cm)
    lib.st_set_scan_side(p_new.side)
    lib.msg_obst_pack(ctypes.byref(p_new), ctypes.byref(Words6()))
    assert fields(p_new) == fields(p_old)

    if not ok_old:
        OUTCOMES["no profile"] += 1
        return

    l_old, r_old, l_new, r_new = F(), F(), F(), F()
    old.obstacle_apply_min_width(ctypes.byref(p_old), min_width, ctypes.byref(l_old), ctypes.byref(r_old))
    new.obstacle_apply_min_width(ctypes.byref(p_new), min_width, ctypes.byref(l_new), ctypes.byref(r_new))
    assert (bits(l_new.value), bits(r_new.value)) == (bits(l_old.value), bits(r_old.value))
    assert fields(p_new) == fields(p_old)

    for flags in range(8):
        a = (ctypes.byref(p_old), half, margin, flags)
        b = (ctypes.byref(p_new), half, margin, flags)
        assert new.obstacle_choose_side(*b) == old.obstacle_choose_side(*a)
        assert new.obstacle_is_tie(*b) == old.obstacle_is_tie(*a)
    for side in (b"L", b"R", b"\0"):
        a = (ctypes.byref(p_old), side, half, margin)
        b = (ctypes.byref(p_new), side, half, margin)
        assert new.obstacle_side_room(*b) == old.obstacle_side_room(*a)
        assert bits(new.obstacle_shift_mm(*b)) == bits(old.obstacle_shift_mm(*a))
        assert bits(new.obstacle_standoff_mm(*b)) == bits(old.obstacle_standoff_mm(*a))

    # the chosen side, then either side as a retry with the other side blocked would
    chosen = old.obstacle_choose_side(ctypes.byref(p_old), half, margin, 0)
    OUTCOMES[chosen] += 1
    for side in (chosen, b"L", b"R"):
        p_old.side, p_new.side = side, side
        pl_old, pl_new = Plan(), Plan()
        args = (half, margin, depth, max_lat)
        r_old = old.bypass_plan(ctypes.byref(p_old), *args, ctypes.byref(pl_old))
        r_new = new.bypass_plan(ctypes.byref(p_new), *args, ctypes.byref(pl_new))
        assert r_new == r_old
        assert fields(pl_new) == fields(pl_old)
        OUTCOMES["plan" if r_old else "no plan"] += 1


fin = dict(allow_nan=False, allow_infinity=False, width=32)
cm = st.one_of(st.just(NO_ECHO), st.sampled_from([0.0, 15.0, 200.0, 431.0]), st.floats(0, 431, **fin))
coarse = st.lists(cm, min_size=COARSE_N, max_size=COARSE_N)
half_mm = st.sampled_from([60.0, 75.0, 92.5, 110.0])
margin_mm = st.one_of(st.sampled_from([0.0, 50.0, 100.0]), st.floats(0, 300, **fin))
depth_mm = st.floats(0, 600, **fin)
lateral_mm = st.sampled_from([0.0, 300.0, 600.0])
width_mm = st.sampled_from([0.0, 150.0, 199.0, 400.0])


@pytest.mark.prop(14, TITLE, unit="free scan")
@settings(max_examples=EXAMPLES)
@given(st.lists(st.tuples(st.floats(-90, 90, **fin), cm), max_size=OBS_MAX_POINTS), coarse,
       half_mm, margin_mm, depth_mm, lateral_mm, width_mm)
def test_free_scan(pair, points, coarse_cm, half, margin, depth, max_lat, min_width):
    """Any points in any order, including none and all no-echo."""
    plan_case(pair, points, coarse_cm, half, margin, depth, max_lat, min_width)


# Shaped like the task's fine scan: 2 deg steps, an obstacle in a clear field,
# so the profile is valid and each side gets chosen.
shaped = st.tuples(st.integers(-40, 0), st.integers(5, 41), st.integers(0, 40), st.integers(1, 20),
                   st.floats(8, 120, **fin), st.floats(0, 14, **fin), st.floats(40, 431, **fin),
                   st.booleans())


@pytest.mark.prop(14, TITLE, unit="shaped scan")
@settings(max_examples=EXAMPLES)
@given(shaped, coarse, half_mm, margin_mm, depth_mm, lateral_mm, width_mm)
def test_shaped_scan(pair, shape, coarse_cm, half, margin, depth, max_lat, min_width):
    first, n, at, width, r, spread, far, b_echo = shape
    points = []
    for i in range(n):
        deg = float(first + (2 * i))
        if at <= i < at + width:
            points.append((deg, r + (spread * (i - at) / width)))
        else:
            points.append((deg, far if b_echo else NO_ECHO))
    plan_case(pair, points, coarse_cm, half, margin, depth, max_lat, min_width)


def test_generators_reach_every_outcome():
    """Runs after the two properties: they were not vacuous."""
    for outcome in (b"L", b"R", b"\0", "no profile", "plan", "no plan"):
        assert OUTCOMES[outcome] > 0, f"{outcome} never generated: {dict(OUTCOMES)}"


def test_known_plan(pair):
    """Hand-worked: a face at 50 cm from -4 to 0 deg, left of centre, so the
    right edge is on the axis (x = 0) and the car passes on the right with a
    shift of half + margin = 125 mm."""
    old, new, _ = pair
    scan = make_scan([(-4.0, 50.0), (-2.0, 50.0), (0.0, 50.0), (2.0, NO_ECHO), (4.0, NO_ECHO)])
    coarse_cm = (F * COARSE_N)(NO_ECHO, NO_ECHO, 50.0, NO_ECHO, NO_ECHO)
    for planner in (old, new):
        p, pl = ObstProfile(), Plan()
        assert planner.obstacle_profile(ctypes.byref(scan), coarse_cm, 75.0, 50.0, ctypes.byref(p))
        assert p.b_valid and p.side == b"R" and p.r_min_cm == 50.0
        assert p.clear_left_cm == 200.0 and p.clear_right_cm == 200.0
        assert planner.bypass_plan(ctypes.byref(p), 75.0, 50.0, 200.0, 300.0, ctypes.byref(pl))
        assert pl.side == 1 and (pl.turn1_deg, pl.turn2_deg) == (45.0, -45.0)
        assert pl.shift_mm == pytest.approx(125.0)
        assert pl.d1_mm == pytest.approx(125.0 / 0.7071068, abs=1e-3)
