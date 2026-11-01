# Feature: minimum-bit-data-representation, Property 2: Out-of-range and NaN stores saturate and are counted
"""Property 2 (design): for any continuous Packed_Field and any input outside
its Valid_Range (one Resolution step past each limit, the float32 minimum and
maximum, +-infinity) or NaN, the stored code is the nearest non-sentinel limit
(NaN: the sentinel, else the lower limit) and the saturation counter grows by
exactly 1 for that store.

bp_quant (app_program/bitpack.h, through bpx_quant) reports the saturation in
*p_sat; the caller then calls sat_note(1) (design C2). Here the test is that
caller, with sat_note/sat_count from stubs/sat_stub.c. The per-accessor
counting (st_put_*, odo_put, wire encoders) is checked again when those
accessors exist (tasks 6.x and later).

**Validates: Requirements 1.4, 1.9, 4.9, 6.5, 7.6, 18.4**
"""

import ctypes
import math
import struct
from fractions import Fraction

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from conftest import reset_stubs
from field_specs import CONTINUOUS_FIELDS as CONTINUOUS_SPECS

SPEC_IDS = [f.name for f in CONTINUOUS_SPECS]

FLT_MAX = struct.unpack("<f", struct.pack("<I", 0x7F7FFFFF))[0]
FLT_TRUE_MIN = struct.unpack("<f", struct.pack("<I", 0x00000001))[0]
SAT_MAX = 65535


def f32(v):
    """v rounded to the nearest float32 (the value ctypes passes to C)."""
    return struct.unpack("<f", struct.pack("<f", v))[0]


def next_f32(v, up):
    """The float32 next to the float32 v, towards +inf (up) or -inf."""
    if v == 0.0:
        return FLT_TRUE_MIN if up else -FLT_TRUE_MIN
    bits = struct.unpack("<I", struct.pack("<f", v))[0]
    bits += 1 if (v > 0.0) == up else -1
    return struct.unpack("<f", struct.pack("<I", bits))[0]


def ref_quant(x, spec):
    """Reference model of the design D0 rules: (code, saturated)."""
    if math.isnan(x):
        return spec.nan_code, True
    if math.isinf(x):
        return (spec.hi_code if x > 0 else spec.lo_code), True
    d = Fraction(x) * spec.scale                   # exact
    code = math.floor(abs(d) + Fraction(1, 2))     # half away from zero
    code = code if d >= 0 else -code
    if code > spec.hi_code:
        return spec.hi_code, True
    if code < spec.lo_code:
        return spec.lo_code, True
    return code, False


def past_limit(spec, up):
    """The float32 nearest to one Resolution step past the upper (up) or lower
    limit that still quantizes outside the Valid_Range. For wide fields
    (odo.dist_mm) float32 cannot hold limit + 1 step, so this walks to the
    next float32 that does round outside."""
    code = spec.hi_code + 1 if up else spec.lo_code - 1
    x = f32(code / spec.scale)
    while not ref_quant(x, spec)[1]:
        x = next_f32(x, up)
    return x


def edge_inputs(spec):
    """Requirement 18.4 inputs: one step past each limit, the float32 min and
    max, +-infinity, and NaN."""
    return [
        ("hi+1step", past_limit(spec, True)),
        ("lo-1step", past_limit(spec, False)),
        ("+FLT_MAX", FLT_MAX),
        ("-FLT_MAX", -FLT_MAX),
        ("+inf", math.inf),
        ("-inf", -math.inf),
        ("nan", math.nan),
    ]


def store(lib, x, spec):
    """One store as an accessor does it: quantize, then sat_note(1) if the
    value was clamped or NaN. Returns (code, sat flag, counter increase)."""
    sat = ctypes.c_bool(False)
    before = lib.sat_count()
    code = lib.bpx_quant(x, *spec.qspec, ctypes.byref(sat))
    if sat.value:
        lib.sat_note(1)
    return code, sat.value, lib.sat_count() - before


def out_of_range(spec):
    """float32 inputs that quantize outside [lo_code, hi_code]."""
    hi_v = past_limit(spec, True)
    lo_v = past_limit(spec, False)
    above = st.floats(min_value=hi_v, max_value=FLT_MAX, width=32)
    below = st.floats(min_value=-FLT_MAX, max_value=lo_v, width=32)
    # Codes k steps past each limit. float32 rounding can pull a wide field's
    # value back inside the range, so keep at least the first outside float.
    near_above = st.integers(1, 1000).map(
        lambda k: max(f32((spec.hi_code + k) / spec.scale), hi_v))
    near_below = st.integers(1, 1000).map(
        lambda k: min(f32((spec.lo_code - k) / spec.scale), lo_v))
    edges = st.sampled_from([v for _, v in edge_inputs(spec)])
    nans = st.sampled_from([math.nan, -math.nan,
                            struct.unpack("<f", struct.pack("<I", 0x7FC00001))[0],
                            struct.unpack("<f", struct.pack("<I", 0xFFFFFFFF))[0]])
    return st.one_of(edges, nans, above, below, near_above, near_below)


@pytest.mark.prop(2, "Out-of-range and NaN stores saturate and are counted")
@pytest.mark.parametrize("spec", CONTINUOUS_SPECS, ids=SPEC_IDS)
def test_prop02_edge_inputs(lib, spec):
    """Each Requirement 18.4 input saturates to the expected limit."""
    for label, x in edge_inputs(spec):
        reset_stubs(lib)
        code, sat, inc = store(lib, x, spec)
        if math.isnan(x):
            want = spec.nan_code
        elif x > 0:
            want = spec.hi_code
        else:
            want = spec.lo_code
        assert (label, code, sat, inc) == (label, want, True, 1)


@pytest.mark.prop(2, "Out-of-range and NaN stores saturate and are counted")
@pytest.mark.parametrize("spec", CONTINUOUS_SPECS, ids=SPEC_IDS)
@settings(max_examples=1000)
@given(data=st.data())
def test_prop02_out_of_range_saturates(lib, spec, data):
    x = data.draw(out_of_range(spec), label="x")
    reset_stubs(lib)
    want, want_sat = ref_quant(x, spec)
    assert want_sat                                   # generator check
    if math.isnan(x):
        assert want == spec.nan_code
    code, sat, inc = store(lib, x, spec)
    assert code == want
    assert sat is True
    assert inc == 1
    assert lib.crit_stub_errors() == 0


@pytest.mark.prop(2, "Out-of-range and NaN stores saturate and are counted")
@pytest.mark.parametrize("spec", CONTINUOUS_SPECS, ids=SPEC_IDS)
@settings(max_examples=1000)
@given(x=st.floats(width=32, allow_nan=True, allow_infinity=True))
def test_prop02_sat_flag_exactly_when_out_of_range(lib, spec, x):
    """*p_sat is true exactly for out-of-range, infinite and NaN inputs, so the
    counter moves only on those stores."""
    reset_stubs(lib)
    want, want_sat = ref_quant(x, spec)
    code, sat, inc = store(lib, x, spec)
    assert (code, sat, inc) == (want, want_sat, 1 if want_sat else 0)


@pytest.mark.prop(2, "Out-of-range and NaN stores saturate and are counted",
                  unit="g_sat_count")
@settings(max_examples=1000)
@given(start=st.one_of(st.integers(0, SAT_MAX), st.integers(SAT_MAX - 3, SAT_MAX)),
       spec=st.sampled_from(CONTINUOUS_SPECS),
       n=st.integers(1, 8))
def test_prop02_counter_holds_at_max(lib, start, spec, n):
    """Each saturated store adds exactly 1, holding at 65,535 (Requirement 1.10)."""
    reset_stubs(lib)
    lib.sat_stub_set(start)
    for i in range(n):
        x = math.nan if i % 2 else math.inf
        _, sat, _ = store(lib, x, spec)
        assert sat
    assert lib.sat_count() == min(start + n, SAT_MAX)
    assert lib.crit_stub_errors() == 0
