# Feature: minimum-bit-data-representation, Property 15: Hump detection matches the Baseline_Build
"""Property 15: the same pitch and distance sequence fed to the hump profiler
of the Baseline_Build (libbase.so) and of this build (libis02.so) gives
HUMP_START, HUMP_PEAK and HUMP_END at the same sample indices, the same valid
or false-slope result, and heights within 0.1 mm.

The two builds are compared after every sample, so a difference is reported at
the sample it first appears. Beyond the property, the packed unit hump_ctx_t.f
must hold exactly min(baseline counter, 5), which is what catches a counter
that wraps: no sequence can see a wrap through the events.

Also here, as the checks of task 10.2: the motion-event classifier against the
baseline, and every field of both packed units against its Valid_Range
(Requirements 2.11, 2.12).

**Validates: Requirements 7.3, 7.7**
"""

import ctypes
import math
from collections import Counter

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from test_baseline_lib import HumpParams

F, U8, U16, U32 = ctypes.c_float, ctypes.c_uint8, ctypes.c_uint16, ctypes.c_uint32
B = ctypes.c_bool

TITLE = "Hump detection matches the Baseline_Build"
HEIGHT_TOL_MM = 0.1
N_MAX = 5
HUMP_FLAT, HUMP_ASCENT, HUMP_CREST, HUMP_DESCENT = 0, 1, 2, 3
MEV_COUNT = 8
EXAMPLES = 1000


class BaseHumpCtx(ctypes.Structure):
    """Baseline_Build hump_ctx_t."""

    _fields_ = [("state", ctypes.c_int), ("n_up", U32), ("n_down", U32), ("n_level", U32),
                ("h_mm", F), ("peak_mm", F), ("len_mm", F)]


class HumpCtx(ctypes.Structure):
    """This build's hump_ctx_t (design D5)."""

    _fields_ = [("f", U16), ("h_mm", F), ("peak_mm", F), ("len_mm", F)]

    state = property(lambda s: s.f & 3)
    n_up = property(lambda s: (s.f >> 2) & 7)
    n_down = property(lambda s: (s.f >> 5) & 7)
    n_level = property(lambda s: (s.f >> 8) & 7)


class HumpOut(ctypes.Structure):
    """hump_out_t, the same in both builds."""

    _fields_ = [("b_start", B), ("b_peak", B), ("b_end", B), ("b_valid", B),
                ("height_mm", F), ("len_mm", F)]


class BaseMevCtx(ctypes.Structure):
    """Baseline_Build mev_ctx_t."""

    _fields_ = [("current", ctypes.c_int), ("candidate", ctypes.c_int), ("n_candidate", U32),
                ("still_ms", U32)]


class MevCtx(ctypes.Structure):
    """This build's mev_ctx_t (design D5)."""

    _fields_ = [("f", U8), ("still_ms", U32)]

    current = property(lambda s: s.f & 7)
    candidate = property(lambda s: (s.f >> 3) & 7)
    n_candidate = property(lambda s: (s.f >> 6) & 3)


class Pair:
    """One profiler of each build, with the Baseline_Build parameters."""

    def __init__(self, lib, base):
        self.par = HumpParams()
        base.base_bh_hump_params(ctypes.byref(self.par))
        self.fns = []
        for mod, prefix, ctx in ((base, "base_", BaseHumpCtx()), (lib, "", HumpCtx())):
            reset = getattr(mod, prefix + "hump_reset")
            reset.restype, reset.argtypes = None, [ctypes.c_void_p]
            step = getattr(mod, prefix + "hump_step")
            step.restype, step.argtypes = HumpOut, [ctypes.c_void_p, ctypes.c_void_p, F, F]
            self.fns.append((reset, step, ctx))

    def reset(self):
        for reset, _, ctx in self.fns:
            reset(ctypes.byref(ctx))

    def step(self, pitch, ds):
        return [(step(ctypes.byref(ctx), ctypes.byref(self.par), pitch, ds), ctx)
                for _, step, ctx in self.fns]


@pytest.fixture(scope="module")
def pair(lib, base):
    return Pair(lib, base)


OUTCOMES = Counter()


def events(o, ctx):
    """(name, height) of each event reported by one step, as tsk_imu.c reads them."""
    ev = []
    if o.b_start:
        ev.append(("HUMP_START", 0.0))
    if o.b_peak:
        ev.append(("HUMP_PEAK", ctx.peak_mm))
    if o.b_end:
        ev.append(("HUMP_END", o.height_mm))
    return ev


def run(pair, seq):
    """Feeds [(pitch, ds, reset), ...] to both builds; returns the events as
    [(sample index, name, height, valid), ...] of this build."""
    got = []
    pair.reset()
    for i, (pitch, ds, b_reset) in enumerate(seq):
        if b_reset:
            pair.reset()
        (o_old, c_old), (o_new, c_new) = pair.step(pitch, ds)
        where = f"sample {i} of {len(seq)}: pitch={pitch} ds={ds}"
        ev_old, ev_new = events(o_old, c_old), events(o_new, c_new)
        assert [e[0] for e in ev_new] == [e[0] for e in ev_old], where
        for (_, h_old), (_, h_new) in zip(ev_old, ev_new):
            assert abs(h_new - h_old) <= HEIGHT_TOL_MM, where
        assert o_new.b_valid == o_old.b_valid, where
        assert abs(o_new.len_mm - o_old.len_mm) <= HEIGHT_TOL_MM, where
        assert c_new.state == c_old.state, where
        assert abs(c_new.h_mm - c_old.h_mm) <= HEIGHT_TOL_MM, where
        for name in ("n_up", "n_down", "n_level"):
            assert getattr(c_new, name) == min(getattr(c_old, name), N_MAX), f"{name} at {where}"
        for name, height in ev_new:
            got.append((i, name, height, o_new.b_valid))
            OUTCOMES[(name, o_new.b_valid) if name == "HUMP_END" else name] += 1
    return got


fin = dict(allow_nan=False, allow_infinity=False, width=32)
# On and either side of the thresholds (5 deg start, 3 deg level)
edge = [0.0, 3.0, -3.0, 5.0, -5.0, 2.9999998, 3.0000002, 4.9999995, 5.0000005,
        -2.9999998, -3.0000002, -4.9999995, -5.0000005, 8.0, -8.0, 12.0, -12.0, 90.0, -90.0]
pitches = st.one_of(st.sampled_from(edge), st.floats(-90, 90, **fin))
steps = st.one_of(st.sampled_from([0.0, 2.0, 5.0, 20.0, -5.0]), st.floats(-60, 60, **fin))
resets = st.sampled_from([False] * 19 + [True])       # I2C fault or collision in tsk_imu.c


@pytest.mark.prop(15, TITLE, unit="hump_ctx_t.f, held pitch runs")
@settings(max_examples=EXAMPLES)
@given(st.lists(st.tuples(pitches, st.integers(1, 40), steps, resets), min_size=1, max_size=30))
def test_held_pitch_runs(pair, runs):
    """Runs of one pitch: long enough to make real humps and to pass 5 and 7."""
    seq = []
    for pitch, n, ds, b_reset in runs:
        seq += [(pitch, ds, b_reset)] + [(pitch, ds, False)] * (n - 1)
    run(pair, seq)


@pytest.mark.prop(15, TITLE, unit="hump_ctx_t.f, free samples")
@settings(max_examples=EXAMPLES)
@given(st.lists(st.tuples(pitches, steps, resets), min_size=1, max_size=120))
def test_free_samples(pair, seq):
    run(pair, seq)


def test_generators_reach_every_outcome():
    """Runs after the two properties: they were not vacuous."""
    for outcome in ("HUMP_START", "HUMP_PEAK", ("HUMP_END", True), ("HUMP_END", False)):
        assert OUTCOMES[outcome] > 0, f"{outcome} never generated: {dict(OUTCOMES)}"


def hump_seq():
    """Up at 10 deg, flat top, down at -10 deg, level: 20 mm per sample."""
    return ([(10.0, 20.0, False)] * 9 + [(0.0, 20.0, False)] * 2 +
            [(-10.0, 20.0, False)] * 9 + [(0.0, 20.0, False)] * 5)


def test_known_hump(pair):
    """Hand-worked: 3 samples up start it; 6 more at 20 sin(10 deg) = 3.47 mm climb
    20.8 mm; the first level sample is the crest; 3 samples down make DESCENT;
    the fifth level sample ends it."""
    got = run(pair, hump_seq())
    assert [(i, n) for i, n, _, _ in got] == [(2, "HUMP_START"), (9, "HUMP_PEAK"), (24, "HUMP_END")]
    peak = 6 * 20.0 * math.sin(math.radians(10.0))
    assert got[1][2] == pytest.approx(peak, abs=1e-3)
    assert got[2][2] == pytest.approx(peak, abs=1e-3) and got[2][3]


def test_false_slope(pair):
    """Up 3 samples, level at once: under 15 mm, so a false slope with no height."""
    got = run(pair, [(10.0, 2.0, False)] * 4 + [(0.0, 2.0, False)])
    assert [(i, n, h, v) for i, n, h, v in got] == [(2, "HUMP_START", 0.0, False),
                                                    (4, "HUMP_END", 0.0, False)]


def test_hump_given_up_when_too_long(pair):
    got = run(pair, [(10.0, 100.0, False)] * 30)
    assert [n for _, n, _, _ in got][:2] == ["HUMP_START", "HUMP_END"]
    assert not got[1][3] and got[1][2] == 0.0


@pytest.mark.parametrize("pitch,name", [(0.0, "n_level"), (10.0, "n_up"), (-10.0, "n_down")])
def test_counter_holds_at_5(pair, pitch, name):
    """Requirement 7.3: past 7 too, where a 3-bit counter would wrap."""
    pair.reset()
    ctx = pair.fns[1][2]
    seen = []
    for _ in range(300):
        pair.step(pitch, 0.0)
        seen.append(getattr(ctx, name))
    assert seen[:6] == [1, 2, 3, 4, 5, 5] and set(seen[5:]) == {5}


def test_unit_sizes(lib):
    size = {n: getattr(lib, n) for n in ("imx_hump_ctx_size", "imx_mev_ctx_size", "imx_hump_n_max")}
    for fn in size.values():
        fn.restype, fn.argtypes = U32, []
    assert size["imx_hump_ctx_size"]() == ctypes.sizeof(HumpCtx) == 16
    assert size["imx_mev_ctx_size"]() == ctypes.sizeof(MevCtx) == 8
    assert size["imx_hump_n_max"]() == N_MAX


def bind_set(lib, name):
    fn = getattr(lib, "imx_" + name)
    fn.restype, fn.argtypes = B, [ctypes.c_void_p, U32]
    return fn


# setter, Valid_Range max, shift, width
UNITS = [
    (HumpCtx, [("hump_set_state", 3, 0, 2), ("hump_set_n_up", N_MAX, 2, 3),
               ("hump_set_n_down", N_MAX, 5, 3), ("hump_set_n_level", N_MAX, 8, 3)]),
    (MevCtx, [("mev_set_current", MEV_COUNT - 1, 0, 3), ("mev_set_candidate", MEV_COUNT - 1, 3, 3),
              ("mev_set_n_candidate", 3, 6, 2)]),
]
FIELDS = [(ctx, *fld) for ctx, flds in UNITS for fld in flds]


@pytest.mark.parametrize("ctx,name,top,sh,w", FIELDS, ids=[f[1] for f in FIELDS])
def test_setter_range(lib, ctx, name, top, sh, w):
    """Requirements 2.11, 2.12: every valid value reads back with the other
    fields kept, every value above the range changes nothing."""
    fn = bind_set(lib, name)
    mask = (1 << w) - 1
    unit_bits = 8 * ctypes.sizeof(ctx._fields_[0][1])
    for before in (0, (1 << unit_bits) - 1):
        c = ctx(f=before)
        for v in range(top + 1):
            assert fn(ctypes.byref(c), v)
            assert (c.f >> sh) & mask == v
            assert c.f & ~(mask << sh) == before & ~(mask << sh)
        c = ctx(f=before)
        for v in (top + 1, mask + 1, 256, 0xFFFFFFFF):
            assert not fn(ctypes.byref(c), v)
            assert c.f == before


@settings(max_examples=300)
@given(st.lists(st.tuples(st.booleans(), st.sampled_from([HUMP_FLAT, HUMP_ASCENT, HUMP_CREST, HUMP_DESCENT]),
                          st.sampled_from([0.0, 100.0, 300.0, -300.0]), st.sampled_from([0.0, 0.5, -0.5]),
                          st.sampled_from([0.0, 1.0, 200.0]), st.integers(1, 40)),
                min_size=1, max_size=40))
def test_mev_matches_baseline(lib, base, runs):
    """mev_step() with mev_ctx_t packed: same reported event at every sample."""
    fns = []
    for mod, prefix, ctx in ((base, "base_", BaseMevCtx()), (lib, "", MevCtx())):
        reset = getattr(mod, prefix + "mev_reset")
        reset.restype, reset.argtypes = None, [ctypes.c_void_p]
        step = getattr(mod, prefix + "mev_step")
        step.restype = ctypes.c_int
        step.argtypes = [ctypes.c_void_p, B, ctypes.c_int, F, F, F, U32]
        reset(ctypes.byref(ctx))
        fns.append((step, ctx))
    (old, c_old), (new, c_new) = fns
    for b_impact, hump, turn, accel, v, n in runs:
        for _ in range(n):
            args = (b_impact, hump, turn, accel, v, 20)
            assert new(ctypes.byref(c_new), *args) == old(ctypes.byref(c_old), *args)
            assert (c_new.current, c_new.candidate) == (c_old.current, c_old.candidate)
            assert c_new.n_candidate == min(c_old.n_candidate, 3)
            assert c_new.still_ms == c_old.still_ms
