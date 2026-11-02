# Feature: minimum-bit-data-representation, Property 10: Inter-task messages round-trip
"""Property 10: packing an inter-task message and unpacking it gives every
field back within half a resolution step (clamped to its range), for every
motion command type, nav events and obstacle profiles including no-echo.

**Validates: Requirements 4.2, 4.4, 6.6, 8.1, 8.6**
"""

import ctypes

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from test_msg_codec import MC_BYPASS, MC_LINE, MC_SPEED_LIMIT, MC_TURN, F, NavEvent, ObstProfile, Words6, motion_rt
from test_msg_codec import mc  # noqa: F401  (fixture)

TITLE = "Inter-task messages round-trip"
fin = dict(allow_nan=False, allow_infinity=False, width=32)


def near(a, b, step, lo, hi):
    return abs(a - min(max(b, lo), hi)) <= step / 2 + 1e-3


@pytest.mark.prop(10, TITLE, unit="motion")
@settings(max_examples=300, deadline=None)
@given(t=st.sampled_from([MC_LINE, MC_TURN, MC_BYPASS, MC_SPEED_LIMIT]), src=st.integers(0, 3),
       cid=st.integers(0, 65535), a=st.floats(-1000, 1000, **fin), b=st.floats(-1000, 1000, **fin),
       to=st.integers(0, 600000), s=st.floats(0, 1, **fin))
def test_motion(mc, t, src, cid, a, b, to, s):  # noqa: F811
    kw = dict(type=t, src=src, id=cid)
    if t == MC_LINE:
        o = motion_rt(mc, left_mm_s=a, right_mm_s=b, **kw)
        assert near(o.left_mm_s, a, 0.1, -819.1, 819.1) and near(o.right_mm_s, b, 0.1, -819.1, 819.1)
    elif t == MC_TURN:
        o = motion_rt(mc, turn_deg=a / 2, speed_mm_s=b, timeout_ms=to, **kw)
        assert near(o.turn_deg, a / 2, 0.1, -360, 360) and near(o.speed_mm_s, b, 0.1, -409.5, 409.5)
        assert o.timeout_ms == min(to, 524287)
    elif t == MC_BYPASS:
        o = motion_rt(mc, dist_mm=a * 3, speed_mm_s=b, timeout_ms=to, **kw)
        assert near(o.dist_mm, a * 3, 0.1, -3000, 3000) and o.timeout_ms == min(to, 524287)
    else:
        o = motion_rt(mc, scale=s, **kw)
        assert near(o.scale, s, 0.01, 0, 1)
    assert (o.type, o.src, o.id) == (t, src, cid)


@pytest.mark.prop(10, TITLE, unit="nav")
@settings(max_examples=200, deadline=None)
@given(t=st.integers(0, 7), letter=st.integers(0, 25))
def test_nav(mc, t, letter):  # noqa: F811
    m = ctypes.c_uint8()
    mc.msg_nav_pack(ctypes.byref(NavEvent(t, bytes([65 + letter]))), ctypes.byref(m))
    out = NavEvent()
    mc.msg_nav_unpack(ctypes.byref(m), ctypes.byref(out))
    assert (out.type, out.code) == (t, bytes([65 + letter]))


cm = st.one_of(st.just(-1.0), st.floats(0, 431, **fin))


@pytest.mark.prop(10, TITLE, unit="obstacle")
@settings(max_examples=300, deadline=None)
@given(valid=st.booleans(), r=cm, th=st.lists(st.floats(-100, 100, **fin), min_size=2, max_size=2),
       xs=st.lists(st.floats(-400, 400, **fin), min_size=5, max_size=5), w=st.floats(0, 800, **fin),
       side=st.sampled_from([b"L", b"R", b"\x00"]), scan=st.lists(cm, min_size=5, max_size=5))
def test_obstacle(mc, valid, r, th, xs, w, side, scan):  # noqa: F811
    p = ObstProfile(valid, r, th[0], th[1], xs[0], xs[1], w, xs[2], xs[3], xs[4], side, (F * 5)(*scan))
    m = Words6()
    mc.msg_obst_pack(ctypes.byref(p), ctypes.byref(m))
    o = ObstProfile()
    mc.msg_obst_unpack(ctypes.byref(m), ctypes.byref(o))
    assert o.b_valid == valid and o.side == side
    assert (o.r_min_cm == -1.0) if r < 0 else near(o.r_min_cm, r, 0.1, 0, 431)
    assert near(o.theta_l_deg, th[0], 0.1, -100, 100) and near(o.theta_r_deg, th[1], 0.1, -100, 100)
    for got, want in zip((o.x_l_cm, o.x_r_cm, o.offset_cm, o.clear_left_cm, o.clear_right_cm), xs):
        assert near(got, want, 0.1, -400, 400)
    assert near(o.width_cm, w, 0.1, 0, 800)
    for got, want in zip(o.scan_cm, scan):
        assert (got == -1.0) if want < 0 else near(got, want, 0.1, 0, 431)
