"""Junction check (line_logic.c junction_step, patch/junction_check): the two line sensors only.

A junction is reported once both line sensors have stayed black (in the
junction's own sense, JUNCTION_BLACK_HIGH) for JUNCTION_MIN_MM of travel,
once per both-black stretch. A real 18 mm cross passes; the edge weaving on
a straight (each sensor flicking black/white, sometimes both black for a
moment) does not. JUNCTION_MIN_MM = 0 reports on the first reading.
"""

import ctypes
import random

import pytest

F = ctypes.c_float


class JuncCtx(ctypes.Structure):
    _fields_ = [("b_both", ctypes.c_bool), ("b_sent", ctypes.c_bool), ("len_mm", F), ("hit_mm", F)]


@pytest.fixture(scope="module")
def ll(lib):
    lib.junction_reset.restype, lib.junction_reset.argtypes = None, [ctypes.POINTER(JuncCtx)]
    lib.junction_step.restype = ctypes.c_bool
    lib.junction_step.argtypes = [ctypes.POINTER(JuncCtx), ctypes.c_bool, ctypes.c_bool, F, F, F]
    return lib


def run(ll, readings, min_mm=6.0, ds=0.8, max_mm=0.0, lens=None):
    """readings: (left_black, right_black). Returns the indexes that reported a junction.
    max_mm 0 = the earlier rule (count on reaching min_mm)."""
    j = JuncCtx()
    ll.junction_reset(ctypes.byref(j))
    hits = []
    for i, (l, r) in enumerate(readings):
        if ll.junction_step(ctypes.byref(j), l, r, ds, min_mm, max_mm):
            hits.append(i)
            if lens is not None:
                lens.append(j.hit_mm)
    return hits


def cross(mm, ds=0.8):
    """White, then both sensors black for mm of travel, then white."""
    n = round(mm / ds)
    return [(False, False)] * 10 + [(True, True)] * n + [(False, False)] * 10


def test_real_cross_reports_once(ll):
    hits = run(ll, cross(18.0))                          # 18 mm cross at 80 mm/s, 10 ms readings
    assert len(hits) == 1
    # 6 mm after the first black reading: index 10 is the first, +8 readings x 0.8 mm = 6.4 mm
    assert hits[0] == 10 + 8


def test_one_reading_flicker_is_not_a_junction(ll):
    # left sensor on the line edge, the right one flickers black for one reading
    readings = [(True, False)] * 20 + [(True, True)] + [(True, False)] * 20
    assert run(ll, readings) == []


def test_edge_weave_on_a_straight_is_not_a_junction(ll):
    # 5 Nov: on a straight both sensors keep switching between black and white,
    # sometimes both black together for a few readings (<= 4 mm here)
    rng = random.Random(5)
    readings = []
    for _ in range(400):
        readings.append((rng.random() < 0.6, rng.random() < 0.4))
        if rng.random() < 0.05:
            readings += [(True, True)] * rng.randint(1, 5)      # up to 5 x 0.8 = 4 mm
    assert run(ll, readings) == []
    # ... and a cross after the weave is still found
    assert len(run(ll, readings + cross(18.0))) == 1


def test_short_both_black_is_not_a_junction(ll):
    assert run(ll, cross(4.0)) == []                      # 4 mm < JUNCTION_MIN_MM 6


def test_zero_reports_on_the_first_reading(ll):
    assert run(ll, cross(18.0), min_mm=0.0) == [10]


def test_each_cross_reports(ll):
    assert len(run(ll, cross(18.0) + cross(18.0))) == 2


def test_stretch_must_restart_after_white(ll):
    # black 5 mm, one white reading, black 5 mm: neither stretch reaches 6 mm
    readings = [(False, False)] * 5 + [(True, True)] * 7 + [(False, False)] + [(True, True)] * 7 + \
               [(False, False)] * 5
    assert run(ll, readings) == []


@pytest.mark.parametrize("ds", [0.3, 0.8, 1.5])            # 30, 80, 150 mm/s
def test_threshold_is_distance_not_readings(ll, ds):
    hits = run(ll, cross(18.0, ds), ds=ds)
    assert len(hits) == 1
    first_black = 10
    travelled = (hits[0] - first_black) * ds
    assert 6.0 <= travelled < 6.0 + ds + 1e-6


# ---- end-of-stretch rule (JUNCTION_MAX_MM > 0) ----

def test_cross_counts_when_the_stretch_ends(ll):
    lens = []
    hits = run(ll, cross(18.0), min_mm=4.0, max_mm=30.0, lens=lens)
    assert hits == [10 + round(18.0 / 0.8)]              # the first reading after the cross
    assert lens[0] == pytest.approx(18.0 - 0.8, abs=1.0)  # its width, for the pivot point


def test_riding_along_the_line_is_not_a_junction(ll):
    # both sensors fit on the line: 200 mm of both-black while following it
    assert run(ll, cross(200.0), min_mm=4.0, max_mm=30.0) == []


def test_too_short_still_ignored(ll):
    assert run(ll, cross(3.0), min_mm=4.0, max_mm=30.0) == []


def test_angled_crossing_still_counts(ll):
    # crossing the 18 mm line at about 30 deg: both sensors black for ~21 mm
    assert len(run(ll, cross(21.0), min_mm=4.0, max_mm=30.0)) == 1


# ---- heading window (JUNCTION_MAX_TURN_DEG) ----

TURN_WIN_SLOTS = 8


class TurnWin(ctypes.Structure):
    _fields_ = [("head", F * TURN_WIN_SLOTS), ("acc_mm", F), ("i", ctypes.c_uint8), ("n", ctypes.c_uint8)]


@pytest.fixture(scope="module")
def tw(lib):
    lib.turn_win_reset.restype, lib.turn_win_reset.argtypes = None, [ctypes.POINTER(TurnWin)]
    lib.turn_win_step.restype, lib.turn_win_step.argtypes = None, [ctypes.POINTER(TurnWin), F, F]
    lib.turn_win_change.restype, lib.turn_win_change.argtypes = F, [ctypes.POINTER(TurnWin), F]
    return lib


def drive(tw, path, ds=0.8):
    """path(distance_mm) -> heading_deg; drives 200 mm and returns the change over the last 40 mm."""
    w = TurnWin()
    tw.turn_win_reset(ctypes.byref(w))
    d = 0.0
    for _ in range(round(200 / ds)):
        d += ds
        tw.turn_win_step(ctypes.byref(w), ds, path(d))
    return tw.turn_win_change(ctypes.byref(w), path(d))


def test_straight_reads_no_turn(tw):
    assert drive(tw, lambda d: 0.0) == pytest.approx(0.0)


def test_line_following_wobble_stays_small(tw):
    import math
    # +/-4 deg weave every 60 mm while following a straight line
    assert drive(tw, lambda d: 4.0 * math.sin(2 * math.pi * d / 60.0)) < 10.0


@pytest.mark.parametrize("radius", [70.0, 80.0, 90.0])
def test_bend_reads_as_turning(tw, radius):
    import math
    turned = drive(tw, lambda d: math.degrees(d / radius))      # constant-radius bend
    assert turned > 15.0                                          # JUNCTION_MAX_TURN_DEG default
    assert turned == pytest.approx(math.degrees(40.0 / radius), abs=3.0)


def test_empty_window_reads_zero(tw):
    w = TurnWin()
    tw.turn_win_reset(ctypes.byref(w))
    assert tw.turn_win_change(ctypes.byref(w), 37.0) == 0.0
