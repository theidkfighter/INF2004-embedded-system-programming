# Feature: ultrasonic-reverse-turn-refinement, Property 1: Scaled reverse with the smallest clearing turn angle
"""OBST_DYNAMIC_TURN (ultrasonic-reverse-turn-refinement).

Property 1 (bug condition): with OBST_LEGACY = 0 and OBST_DYNAMIC_TURN = 1,
the standoff reverses only OBST_REVERSE_SCALE times the original 45 deg
standoff, r1 = scale min(max(need45, 0), CAP) (0 below 5 mm), and the first
turn is the smallest whole-degree angle in [MIN, MAX] whose leg passes the
near corner by half + margin:

    d(theta, y0 + T) = (y0 + T) sin(theta) - e cos(theta) >= H - 0.01 mm

The ``planner`` helper calls obstacle_dyn_standoff + bypass_plan_dyn when
libis02.so has them (the fix). Otherwise it models the unfixed task code,
standoff() + bypass_plan(): reverse obstacle_standoff_mm - r_min, skipped
below 5 mm and capped at OBST_BACKOFF_MAX_MM, theta = bypass_plan().turn1_deg.
On unfixed code the cases below fail; that is the bug. Task 3.9 re-runs this
file unchanged against the fix.

The ctypes layouts of obst_dyn_cfg_t / obst_dyn_plan_t follow design.md
("Changes Required"); the C types added in task 3.1 must match them.

**Validates: Requirements 1.1, 1.2, 1.3, 1.4, 1.5 (expected: 2.1, 2.2, 2.3, 2.4, 2.5)**
"""

import ctypes
import math

import pytest

from test_msg_codec import F, ObstProfile
from test_prop14_obstacle import Plan

TITLE = "Scaled reverse with the smallest clearing turn angle"
B, CH, I32 = ctypes.c_bool, ctypes.c_char, ctypes.c_int
P = ctypes.POINTER

OBST_BACKOFF_MAX_MM = 250.0       # robot_config.h, CAP
OBST_REVERSE_MAX_MM = 250.0       # robot_config.h, per-obstacle reverse budget
OBST_REVERSE_MIN_MM = 5.0         # obstacle_logic.h, the standoff skip
OBST_US_AHEAD_MM = 80.0           # robot_config.h (placeholder value)
OBST_MARGIN_MM = 60.0             # robot_config.h, m0
OBST_SIDE_MAX_MM = 1000.0         # leg 2 upper clamp for bypass_plan_dyn
DEPTH_MM, MAX_LAT_MM = 200.0, 300.0
EPS_MM = 0.01                     # OBST_DYN_EPS_MM
OBST_REV_FULL, OBST_REV_SHORTENED, OBST_REV_REFUSED = 0, 1, 2
HALF_MM = 80.0                    # ROBOT_HALF_WIDTH_MM (passing left)
WIDTH_CM = 17.0                   # 150 mm min width + 2 x 20 mm edge pad


class DynCfg(ctypes.Structure):
    """obst_dyn_cfg_t (design.md)."""

    _fields_ = [("scale", F), ("min_deg", F), ("max_deg", F), ("cap_mm", F),
                ("us_ahead_mm", F), ("swing_margin_mm", F)]


class DynPlan(ctypes.Structure):
    """obst_dyn_plan_t (design.md); kind is obst_dyn_kind_t."""

    _fields_ = [("theta_deg", F), ("reverse_mm", F), ("base_mm", F), ("orig_mm", F),
                ("clear_mm", F), ("kind", I32)]


def default_cfg(scale=0.5, min_deg=30.0, max_deg=70.0):
    return DynCfg(scale, min_deg, max_deg, OBST_BACKOFF_MAX_MM, OBST_US_AHEAD_MM, OBST_MARGIN_MM)


@pytest.fixture(scope="module")
def fns(lib):
    """The planner functions of libis02.so; the dynamic ones only if built."""
    sigs = {
        "obstacle_standoff_mm": (F, [P(ObstProfile), CH, F, F]),
        "bypass_plan": (B, [P(ObstProfile), F, F, F, F, P(Plan)]),
        "obstacle_reverse_budget": (I32, [F, F, F, B, P(F)]),
        "obstacle_dyn_standoff": (B, [P(ObstProfile), CH, F, F, P(DynCfg), P(DynPlan)]),
        "bypass_plan_dyn": (B, [P(ObstProfile), F, F, F, F, F, F, P(Plan)]),
    }
    out = {}
    for name, (restype, argtypes) in sigs.items():
        if not hasattr(lib, name):
            continue
        fn = getattr(lib, name)
        fn.restype, fn.argtypes = restype, argtypes
        out[name] = fn
    out["dynamic"] = ("obstacle_dyn_standoff" in out) and ("bypass_plan_dyn" in out)
    return out


def make_profile(e_mm, y0_mm, side):
    """A valid profile whose near corner on 'side' is e_mm out from the centre line."""
    p = ObstProfile()
    p.b_valid, p.side, p.r_min_cm = True, side, y0_mm / 10.0
    if side == b"L":
        p.x_l_cm = -e_mm / 10.0
        p.x_r_cm = p.x_l_cm + WIDTH_CM
    else:
        p.x_r_cm = e_mm / 10.0
        p.x_l_cm = p.x_r_cm - WIDTH_CM
    p.width_cm = p.x_r_cm - p.x_l_cm
    p.offset_cm = (p.x_l_cm + p.x_r_cm) / 2.0
    p.clear_left_cm = p.clear_right_cm = 200.0
    return p


def corner_e(p, side):
    return (-10.0 * p.x_l_cm) if side == b"L" else (10.0 * p.x_r_cm)


def clearance(theta_deg, y_mm, e_mm):
    t = math.radians(theta_deg)
    return (y_mm * math.sin(t)) - (e_mm * math.cos(t))


def moved(p, reverse_mm):
    q = ObstProfile.from_buffer_copy(p)
    q.r_min_cm = p.r_min_cm + (reverse_mm / 10.0)
    return q


def current_planner(fns, p, side, half, margin):
    """Unfixed task code: standoff() then bypass_plan()."""
    need = fns["obstacle_standoff_mm"](ctypes.byref(p), side, half, margin) - (p.r_min_cm * 10.0)
    rev = 0.0 if need < OBST_REVERSE_MIN_MM else min(need, OBST_BACKOFF_MAX_MM)
    q, plan = moved(p, rev), Plan()
    assert fns["bypass_plan"](ctypes.byref(q), half, margin, DEPTH_MM, MAX_LAT_MM, ctypes.byref(plan))
    return rev, abs(plan.turn1_deg), plan


def planner(fns, p, side, half, margin, cfg):
    """(reverse_mm, theta_deg, plan): the fix when built, else the unfixed code."""
    if not fns["dynamic"]:
        return current_planner(fns, p, side, half, margin)
    out = DynPlan()
    assert fns["obstacle_dyn_standoff"](ctypes.byref(p), side, half, margin, ctypes.byref(cfg),
                                        ctypes.byref(out))
    q, plan = moved(p, out.reverse_mm), Plan()
    assert fns["bypass_plan_dyn"](ctypes.byref(q), half, margin, DEPTH_MM, MAX_LAT_MM, out.theta_deg,
                                  OBST_SIDE_MAX_MM, ctypes.byref(plan))
    return out.reverse_mm, abs(plan.turn1_deg), plan


def expected_r1(fns, p, side, half, margin, cfg):
    need45 = fns["obstacle_standoff_mm"](ctypes.byref(p), side, half, margin) - (p.r_min_cm * 10.0)
    r1 = cfg.scale * min(max(need45, 0.0), OBST_BACKOFF_MAX_MM)
    return (0.0 if r1 < OBST_REVERSE_MIN_MM else r1), need45


# (name, e_mm, y0_mm, margin_mm, expected reverse_mm, expected theta_deg); H = 80 + 60 = 140
CASES = [
    ("reference", 95.0, 170.0, 60.0, 61.5, 57.0),
    ("shallow", 40.0, 270.0, 60.0, 0.0, 40.0),
    ("band_0_5mm", 95.0, 290.0, 60.0, 0.0, 46.0),
]


@pytest.mark.prop(1, TITLE)
@pytest.mark.parametrize("unit,e_mm,y0_mm,margin,want_rev,want_theta", CASES, ids=[c[0] for c in CASES])
def test_bug_condition_cases(fns, unit, e_mm, y0_mm, margin, want_rev, want_theta):
    """isBugCondition: valid profile, side 'L', OBST_LEGACY = 0, OBST_DYNAMIC_TURN = 1."""
    side, half, cfg = b"L", HALF_MM, default_cfg()
    p = make_profile(e_mm, y0_mm, side)
    h = half + margin
    rev, theta, plan = planner(fns, p, side, half, margin, cfg)
    r1, need45 = expected_r1(fns, p, side, half, margin, cfg)
    e = corner_e(p, side)
    d = clearance(theta, y0_mm + rev, e)
    report = (f"{unit}: e={e:.1f} y0={y0_mm:.1f} H={h:.1f} need45={need45:.2f} -> "
              f"reverse={rev:.2f} mm, theta={theta:.1f} deg, clearance={d:.2f} mm "
              f"(expected reverse={want_rev} theta={want_theta})")

    assert cfg.min_deg <= theta <= cfg.max_deg, report
    assert d >= h - EPS_MM, report                                   # 2.2, 2.3: full clearance
    assert rev == pytest.approx(r1, abs=0.05), report                # 2.1: scaled reverse
    assert theta == pytest.approx(want_theta, abs=1e-3), report      # 2.2, 2.3: smallest clearing angle
    if theta > cfg.min_deg:                                          # rounded up, not more
        assert clearance(theta - 1.0, y0_mm + rev, e) < h - EPS_MM, report
    if (r1 > 0.0) and (e >= 0.0):
        assert theta >= 45.0, report
    assert rev == pytest.approx(want_rev, abs=0.05), report
    assert plan.turn3_deg == pytest.approx(45.0), report             # 3.1: return turn stays 45


@pytest.mark.prop(1, TITLE, unit="wider_retry_budget")
def test_bug_condition_wider_retry_budget(fns):
    """The reference attempt, then a wider retry (H = 240) on the same side,
    against the 250 mm per-obstacle budget (standoff not shortenable)."""
    side, half, cfg = b"L", HALF_MM, default_cfg()
    p = make_profile(95.0, 170.0, side)
    allowed = F()

    rev1, theta1, _ = planner(fns, p, side, half, 60.0, cfg)
    rv1 = fns["obstacle_reverse_budget"](rev1, 0.0, OBST_REVERSE_MAX_MM, False, ctypes.byref(allowed))
    assert rv1 == OBST_REV_FULL
    used = rev1

    retry = moved(p, rev1)                    # the profile moved by the first reverse
    rev2, theta2, _ = planner(fns, retry, side, half, 160.0, cfg)
    rv2 = fns["obstacle_reverse_budget"](rev2, used, OBST_REVERSE_MAX_MM, False, ctypes.byref(allowed))
    report = (f"first: reverse={rev1:.2f} theta={theta1:.1f}; retry H=240: r_min={retry.r_min_cm * 10:.1f} "
              f"reverse={rev2:.2f} theta={theta2:.1f}, used={used:.2f} of {OBST_REVERSE_MAX_MM:.0f} -> "
              f"{'REFUSED (ABR_REVERSE_LIMIT)' if rv2 == OBST_REV_REFUSED else 'accepted'}")

    assert rv2 != OBST_REV_REFUSED, report                           # 1.4, 1.5 -> 2.1, 2.5
    assert rev2 == pytest.approx(101.4, abs=0.2), report


# ---- Property 2: Preservation (task 2, observation-first on unfixed code) ----
import struct

from hypothesis import given, settings
from hypothesis import strategies as st

PRES_TITLE = "Switch off or legacy is the original, bit for bit"
SIN45 = 0.7071068                 # obstacle_logic.c / tsk_obstacle.c literal


def f32(x):
    """Round a Python float to float32 (an f32 / f32 quotient in double, rounded once, is IEEE f32)."""
    return struct.unpack("<f", struct.pack("<f", x))[0]


def fbits(x):
    return struct.pack("<f", x)


def plan_bits(plan):
    return [fbits(getattr(plan, n)) if isinstance(getattr(plan, n), float) else getattr(plan, n)
            for n, _ in plan._fields_]


@pytest.mark.prop(2, PRES_TITLE)
@settings(max_examples=1000, deadline=None)
@given(e_mm=st.floats(-150.0, 400.0), y0_mm=st.floats(50.0, 600.0), side=st.sampled_from([b"L", b"R"]),
       half=st.floats(40.0, 200.0), margin=st.floats(0.0, 500.0), out_mm=st.floats(0.0, 1000.0))
def test_preservation_theta45(lib, fns, e_mm, y0_mm, side, half, margin, out_mm):
    """**Validates: Requirements 2.9, 3.1, 3.2, 3.3, 3.5, 3.7**

    Observed on unfixed code: bypass_plan gives |turn1| = 45, d1 = shift / SIN45,
    turn3 = -side 45, and the run_bypass sidestep is out / 0.7071068f. When the
    fix is built, bypass_plan_dyn(..., 45) and obstacle_leg_for_shift(out, 45)
    must equal them bit for bit."""
    p, plan = make_profile(e_mm, y0_mm, side), Plan()
    assert fns["bypass_plan"](ctypes.byref(p), half, margin, DEPTH_MM, MAX_LAT_MM, ctypes.byref(plan))
    s = float(plan.side)
    assert s in (-1.0, 1.0)
    assert plan.turn1_deg == s * 45.0
    assert abs(plan.turn1_deg) == 45.0
    assert fbits(plan.d1_mm) == fbits(f32(plan.shift_mm / f32(SIN45)))
    assert plan.turn3_deg == -s * 45.0
    out = f32(out_mm)
    sidestep = f32(out / f32(SIN45))

    if not (fns["dynamic"] and hasattr(lib, "obstacle_leg_for_shift")):
        return                                     # unfixed code: identity half skipped
    dyn = Plan()
    assert fns["bypass_plan_dyn"](ctypes.byref(p), half, margin, DEPTH_MM, MAX_LAT_MM, 45.0,
                                  OBST_SIDE_MAX_MM, ctypes.byref(dyn))
    assert plan_bits(dyn) == plan_bits(plan)
    leg = lib.obstacle_leg_for_shift
    leg.restype, leg.argtypes = F, [F, F]
    assert fbits(leg(out, 45.0)) == fbits(sidestep)


# ---- Properties 1, 3, 4, 5, 6 and worked examples (task 3.4) ----
SLACK = 1e-3                      # float32 (C) vs double (Python) rounding
CLEAR_K, EXTRA_K, SWING_K, CAPPED_K = 0, 1, 2, 3
SIDE_SIGN = {b"L": 1.0, b"R": -1.0}


@pytest.fixture(scope="module")
def dyn(lib, fns):
    """obstacle_dyn_standoff as a Python call; skips when the fix is not built."""
    if not fns["dynamic"] or not hasattr(lib, "obstacle_leg_for_shift"):
        pytest.skip("OBST_DYNAMIC_TURN functions not built")
    lib.obstacle_leg_for_shift.restype = F
    lib.obstacle_leg_for_shift.argtypes = [F, F]

    def call(p, side, half, margin, cfg):
        out = DynPlan()
        ok = fns["obstacle_dyn_standoff"](ctypes.byref(p), side, half, margin, ctypes.byref(cfg),
                                          ctypes.byref(out))
        return ok, out
    return call


def swing(theta_deg, w, a):
    t = math.radians(theta_deg)
    if t >= math.atan2(w, a):
        return math.sqrt((w * w) + (a * a)) - a
    return (w * math.sin(t)) + (a * math.cos(t)) - a


def candidates(mn, mx):
    th = mn
    while th <= mx:
        yield th
        n = math.floor(th) + 1.0
        if (n > mx) and (th < mx):
            n = mx
        th = n


def y_req(theta_deg, h, e):
    t = math.radians(theta_deg)
    return (h + (e * math.cos(t))) / math.sin(t)


@st.composite
def dyn_inputs(draw, scale=None):
    side = draw(st.sampled_from([b"L", b"R"]))
    p = make_profile(draw(st.floats(-150.0, 400.0)), draw(st.floats(50.0, 600.0)), side)
    half, margin = draw(st.floats(40.0, 200.0)), draw(st.floats(0.0, 500.0))
    sc = draw(st.one_of(st.just(1.0), st.floats(0.0, 1.0))) if scale is None else scale
    cfg = DynCfg(sc, draw(st.floats(20.0, 45.0)), draw(st.floats(45.0, 80.0)), OBST_BACKOFF_MAX_MM,
                 draw(st.floats(0.0, 200.0)), OBST_MARGIN_MM)
    return p, side, half, margin, cfg


def ctx(fns, p, side, half, margin):
    e, y0 = corner_e(p, side), p.r_min_cm * 10.0
    need45 = fns["obstacle_standoff_mm"](ctypes.byref(p), side, half, margin) - y0
    return e, y0, half + margin, need45


@pytest.mark.prop(1, TITLE)
@settings(max_examples=1000, deadline=None)
@given(x=dyn_inputs())
def test_prop1_smallest_clearing_angle(fns, dyn, x):
    """**Validates: Requirements 2.1, 2.2, 2.3, 2.4**"""
    p, side, half, margin, cfg = x
    ok, out = dyn(p, side, half, margin, cfg)
    assert ok
    e, y0, h, need45 = ctx(fns, p, side, half, margin)
    r1 = cfg.scale * min(max(need45, 0.0), OBST_BACKOFF_MAX_MM)
    r1 = 0.0 if r1 < OBST_REVERSE_MIN_MM else r1
    th, t = out.theta_deg, out.reverse_mm
    assert cfg.min_deg <= th <= cfg.max_deg
    assert out.base_mm == pytest.approx(r1, abs=SLACK)
    if out.kind != CAPPED_K:
        assert clearance(th, y0 + t, e) >= h - EPS_MM - SLACK
    if out.kind == CLEAR_K:
        assert t == pytest.approx(r1, abs=SLACK)
        for c in candidates(cfg.min_deg, cfg.max_deg):
            if c >= th:
                break
            clearly = (clearance(c, y0 + t, e) >= h - EPS_MM + SLACK) and (
                c <= 45.0 or y0 + t >= swing(c, half, cfg.us_ahead_mm) + OBST_MARGIN_MM - EPS_MM + SLACK)
            assert not clearly, f"candidate {c} clears before {th}"
    if e >= 0.0 and r1 > 0.0:
        assert th >= 45.0
    if e >= 0.0 and cfg.scale == 1.0 and OBST_REVERSE_MIN_MM <= need45 <= OBST_BACKOFF_MAX_MM:
        assert th == 45.0


@pytest.mark.prop(3, "Never more reverse than the original")
@settings(max_examples=1000, deadline=None)
@given(x=dyn_inputs(), used=st.floats(0.0, OBST_REVERSE_MAX_MM))
def test_prop3_reverse_bound(fns, dyn, x, used):
    """**Validates: Requirements 2.1, 2.4, 2.5, 3.6**"""
    p, side, half, margin, cfg = x
    ok, out = dyn(p, side, half, margin, cfg)
    assert ok
    _, _, _, need45 = ctx(fns, p, side, half, margin)
    t = out.reverse_mm
    assert t <= min(max(need45, 0.0), OBST_BACKOFF_MAX_MM) + EPS_MM + SLACK
    if not (0.0 < need45 < OBST_REVERSE_MIN_MM):
        orig = 0.0 if need45 < OBST_REVERSE_MIN_MM else min(need45, OBST_BACKOFF_MAX_MM)
        assert t <= orig + EPS_MM + SLACK
        a = F()
        if fns["obstacle_reverse_budget"](orig, used, OBST_REVERSE_MAX_MM, False, ctypes.byref(a)) == OBST_REV_FULL:
            assert fns["obstacle_reverse_budget"](t, used, OBST_REVERSE_MAX_MM, False,
                                                  ctypes.byref(a)) == OBST_REV_FULL


@pytest.mark.prop(4, "A longer reverse never gives a steeper turn")
@settings(max_examples=1000, deadline=None)
@given(x=dyn_inputs(scale=0.0), a=st.floats(0.0, 1.0), b=st.floats(0.0, 1.0))
def test_prop4_monotonic(dyn, x, a, b):
    """**Validates: Requirements 2.2**"""
    p, side, half, margin, cfg = x
    lo, hi = min(a, b), max(a, b)
    cfg.scale = lo
    _, out_lo = dyn(p, side, half, margin, cfg)
    cfg.scale = hi
    _, out_hi = dyn(p, side, half, margin, cfg)
    assert out_hi.theta_deg <= out_lo.theta_deg


@pytest.mark.prop(5, "Legs for theta reach the same shift and forward point")
@settings(max_examples=1000, deadline=None)
@given(e_mm=st.floats(-150.0, 400.0), y0_mm=st.floats(50.0, 600.0), side=st.sampled_from([b"L", b"R"]),
       half=st.floats(40.0, 200.0), margin=st.floats(0.0, 500.0), theta=st.floats(20.0, 80.0),
       side_max=st.floats(50.0, 2000.0), out_mm=st.floats(0.0, 1000.0))
def test_prop5_leg_sizing(lib, fns, dyn, e_mm, y0_mm, side, half, margin, theta, side_max, out_mm):
    """**Validates: Requirements 2.6, 3.1, 3.4**"""
    p, base, plan = make_profile(e_mm, y0_mm, side), Plan(), Plan()
    th = f32(theta)
    assert fns["bypass_plan"](ctypes.byref(p), half, margin, DEPTH_MM, MAX_LAT_MM, ctypes.byref(base))
    assert fns["bypass_plan_dyn"](ctypes.byref(p), half, margin, DEPTH_MM, MAX_LAT_MM, th, side_max,
                                  ctypes.byref(plan))
    s, st_ = float(plan.side), math.sin(math.radians(th))
    for n in ("side", "shift_mm", "turn3_deg", "return_max_mm", "realign_deg"):
        assert getattr(plan, n) == getattr(base, n), n
    if th == 45.0:
        return                                     # bit-identical case: Property 2
    assert plan.turn1_deg == pytest.approx(s * th, abs=1e-4)
    assert plan.turn2_deg == pytest.approx(-s * th, abs=1e-4)
    assert plan.d1_mm * st_ == pytest.approx(plan.shift_mm, abs=1e-3, rel=1e-5)
    d2 = (y0_mm + DEPTH_MM + half + margin) - (plan.shift_mm * math.cos(math.radians(th)) / st_)
    assert 50.0 <= plan.d2_mm <= f32(side_max) + 1e-3
    assert plan.d2_mm == pytest.approx(min(max(d2, 50.0), side_max), abs=1e-2, rel=1e-5)
    leg = lib.obstacle_leg_for_shift(f32(out_mm), th)
    assert leg * st_ == pytest.approx(f32(out_mm), abs=1e-3, rel=1e-5)


@pytest.mark.prop(6, "Body clear of the front while pivoting past 45")
@settings(max_examples=1000, deadline=None)
@given(x=dyn_inputs())
def test_prop6_pivot_swing(fns, dyn, x):
    """**Validates: Requirements 2.7**"""
    p, side, half, margin, cfg = x
    ok, out = dyn(p, side, half, margin, cfg)
    assert ok
    e, y0, h, _ = ctx(fns, p, side, half, margin)
    th, t = out.theta_deg, out.reverse_mm
    sw = swing(th, half, cfg.us_ahead_mm) + OBST_MARGIN_MM
    if th > 45.0 and out.kind != CAPPED_K:
        assert y0 + t >= sw - EPS_MM - SLACK
    if out.kind == EXTRA_K:                        # swing did not add reverse
        assert t == pytest.approx(max(out.base_mm, y_req(th, h, e) - y0), abs=1e-2)
    if out.kind == SWING_K:                        # swing binds and sets T
        assert th > 45.0
        assert t == pytest.approx(sw - y0, abs=1e-2)
        assert t >= max(out.base_mm, y_req(th, h, e) - y0) - 1e-2


# (name, e, y0, margin, scale, want reverse, want theta, want kind); half 80, side 'L'
WORKED = [
    ("far", 0.0, 400.0, 60.0, 0.5, 0.0, 30.0, CLEAR_K),
    ("wider_retry", 95.0, 231.5, 160.0, 0.5, 101.4, 60.0, CLEAR_K),
    ("fallback", 150.0, 170.0, 160.0, 0.5, 140.0, 70.0, EXTRA_K),
    ("capped", 300.0, 100.0, 160.0, 0.5, 250.0, 70.0, CAPPED_K),
    ("scale_1", 95.0, 170.0, 60.0, 1.0, 123.0, 45.0, CLEAR_K),
]


@pytest.mark.prop(1, TITLE, unit="worked_examples")
@pytest.mark.parametrize("name,e_mm,y0_mm,margin,scale,want_rev,want_theta,want_kind", WORKED,
                         ids=[w[0] for w in WORKED])
def test_worked_examples(dyn, name, e_mm, y0_mm, margin, scale, want_rev, want_theta, want_kind):
    ok, out = dyn(make_profile(e_mm, y0_mm, b"L"), b"L", HALF_MM, margin, default_cfg(scale=scale))
    assert ok
    assert out.reverse_mm == pytest.approx(want_rev, abs=0.2)
    assert out.theta_deg == want_theta
    assert out.kind == want_kind


@pytest.mark.prop(6, "Body clear of the front while pivoting past 45", unit="swing_case")
def test_swing_case(dyn):
    """A = 0 and a 200 mm swing margin: the swing term binds at 68 deg."""
    cfg = DynCfg(0.0, 30.0, 70.0, OBST_BACKOFF_MAX_MM, 0.0, 200.0)
    ok, out = dyn(make_profile(300.0, 50.0, b"L"), b"L", HALF_MM, 60.0, cfg)
    assert ok and out.kind == SWING_K and out.theta_deg == 68.0
    assert out.reverse_mm == pytest.approx(swing(68.0, HALF_MM, 0.0) + 200.0 - 50.0, abs=1e-2)


@pytest.mark.prop(1, TITLE, unit="invalid_inputs")
@pytest.mark.parametrize("scale,mn,mx,side,valid", [
    (1.5, 30.0, 70.0, b"L", True), (-0.1, 30.0, 70.0, b"L", True), (0.5, 50.0, 70.0, b"L", True),
    (0.5, 30.0, 40.0, b"L", True), (0.5, 30.0, 70.0, b"\x00", True), (0.5, 30.0, 70.0, b"L", False)])
def test_invalid_returns_false(dyn, scale, mn, mx, side, valid):
    p = make_profile(95.0, 170.0, b"L")
    p.b_valid = valid
    ok, _ = dyn(p, side, HALF_MM, 60.0, default_cfg(scale, mn, mx))
    assert not ok
