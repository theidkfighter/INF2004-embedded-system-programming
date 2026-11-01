"""Inter-task message packing (app_program/msg_codec.c): round trips at the
documented resolution, clamping, and no field disturbing another (fields
straddle 32-bit words)."""

import ctypes
import random

import pytest

F = ctypes.c_float


class MotionCmd(ctypes.Structure):
    _fields_ = [("type", ctypes.c_uint8), ("src", ctypes.c_uint8), ("id", ctypes.c_uint16),
                ("left_mm_s", F), ("right_mm_s", F), ("turn_deg", F), ("dist_mm", F),
                ("speed_mm_s", F), ("scale", F), ("timeout_ms", ctypes.c_uint32)]


class NavEvent(ctypes.Structure):
    _fields_ = [("type", ctypes.c_uint8), ("code", ctypes.c_char)]


class ObstProfile(ctypes.Structure):
    _fields_ = [("b_valid", ctypes.c_bool), ("r_min_cm", F), ("theta_l_deg", F), ("theta_r_deg", F),
                ("x_l_cm", F), ("x_r_cm", F), ("width_cm", F), ("offset_cm", F),
                ("clear_left_cm", F), ("clear_right_cm", F), ("side", ctypes.c_char), ("scan_cm", F * 5)]


Words3, Words6 = ctypes.c_uint32 * 3, ctypes.c_uint32 * 6
MC_LINE, MC_TURN, MC_BYPASS, MC_SPEED_LIMIT, MC_STOP = 1, 2, 3, 4, 5


@pytest.fixture(scope="module")
def mc(lib):
    P = ctypes.POINTER
    for n, a in (("msg_motion_pack", [P(MotionCmd), P(Words3)]), ("msg_motion_unpack", [P(Words3), P(MotionCmd)]),
                 ("msg_nav_pack", [P(NavEvent), P(ctypes.c_uint8)]), ("msg_nav_unpack", [P(ctypes.c_uint8), P(NavEvent)]),
                 ("msg_obst_pack", [P(ObstProfile), P(Words6)]), ("msg_obst_unpack", [P(Words6), P(ObstProfile)])):
        getattr(lib, n).restype = None
        getattr(lib, n).argtypes = a
    return lib


def motion_rt(mc, **kw):
    c = MotionCmd(**kw)
    w = Words3()
    mc.msg_motion_pack(ctypes.byref(c), ctypes.byref(w))
    out = MotionCmd()
    mc.msg_motion_unpack(ctypes.byref(w), ctypes.byref(out))
    return out


def test_sizes():
    assert ctypes.sizeof(MotionCmd) == 32 and ctypes.sizeof(Words3) == 12
    assert ctypes.sizeof(ObstProfile) == 64 and ctypes.sizeof(Words6) == 24


@pytest.mark.parametrize("seed", range(20))
def test_motion_round_trips(mc, seed):
    rng = random.Random(seed)
    hdr = dict(src=rng.randint(1, 3), id=rng.randint(0, 65535))
    o = motion_rt(mc, type=MC_LINE, left_mm_s=rng.uniform(-500, 500), right_mm_s=rng.uniform(-500, 500), **hdr)
    assert (o.type, o.src, o.id) == (MC_LINE, hdr["src"], hdr["id"])
    c = MotionCmd(type=MC_LINE, left_mm_s=0, **hdr)
    lw = rng.uniform(-500, 500)
    o = motion_rt(mc, type=MC_LINE, left_mm_s=lw, right_mm_s=-lw, **hdr)
    assert o.left_mm_s == pytest.approx(lw, abs=0.05) and o.right_mm_s == pytest.approx(-lw, abs=0.05)
    deg, sp, to = rng.uniform(-360, 360), rng.uniform(20, 250), rng.randint(0, 400000)
    o = motion_rt(mc, type=MC_TURN, turn_deg=deg, speed_mm_s=sp, timeout_ms=to, **hdr)
    assert o.turn_deg == pytest.approx(deg, abs=0.05) and o.speed_mm_s == pytest.approx(sp, abs=0.05)
    assert o.timeout_ms == to and (o.src, o.id) == (hdr["src"], hdr["id"])
    d = rng.uniform(-3000, 3000)
    o = motion_rt(mc, type=MC_BYPASS, dist_mm=d, speed_mm_s=sp, timeout_ms=to, **hdr)
    assert o.dist_mm == pytest.approx(d, abs=0.05) and o.speed_mm_s == pytest.approx(sp, abs=0.05)
    assert o.timeout_ms == to
    s = rng.uniform(0, 1)
    o = motion_rt(mc, type=MC_SPEED_LIMIT, scale=s, **hdr)
    assert o.scale == pytest.approx(s, abs=0.005)
    o = motion_rt(mc, type=MC_STOP, **hdr)
    assert (o.type, o.id, o.left_mm_s, o.timeout_ms) == (MC_STOP, hdr["id"], 0.0, 0)


def test_motion_clamps(mc):
    o = motion_rt(mc, type=MC_TURN, turn_deg=999.0, speed_mm_s=9999.0, timeout_ms=10 ** 7)
    assert o.turn_deg == pytest.approx(360.0) and o.speed_mm_s == pytest.approx(409.5)
    assert o.timeout_ms == 524287
    o = motion_rt(mc, type=MC_BYPASS, dist_mm=-1e6, speed_mm_s=-50.0)
    assert o.dist_mm == pytest.approx(-3000.0) and o.speed_mm_s == pytest.approx(-50.0)


def test_nav_pack(mc):
    for t in range(1, 8):
        for letter in (b"A", b"K", b"Z"):
            m = ctypes.c_uint8()
            mc.msg_nav_pack(ctypes.byref(NavEvent(t, letter)), ctypes.byref(m))
            out = NavEvent()
            mc.msg_nav_unpack(ctypes.byref(m), ctypes.byref(out))
            assert (out.type, out.code) == (t, letter)


def test_obstacle_round_trip(mc):
    p = ObstProfile(True, 18.27, -32.5, 41.0, -12.3, 15.6, 27.9, 1.65, 200.0, -35.0, b"R",
                    (F * 5)(-1.0, 25.5, 18.3, 431.0, 0.0))
    w = Words6()
    mc.msg_obst_pack(ctypes.byref(p), ctypes.byref(w))
    o = ObstProfile()
    mc.msg_obst_unpack(ctypes.byref(w), ctypes.byref(o))
    assert o.b_valid and o.side == b"R"
    for name in ("r_min_cm", "theta_l_deg", "theta_r_deg", "x_l_cm", "x_r_cm", "width_cm", "offset_cm",
                 "clear_left_cm", "clear_right_cm"):
        assert getattr(o, name) == pytest.approx(getattr(p, name), abs=0.05), name
    assert [round(x, 1) for x in o.scan_cm] == [-1.0, 25.5, 18.3, 431.0, 0.0]
