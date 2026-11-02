# Feature: minimum-bit-data-representation, Property 1: Continuous stores round to the nearest step
"""Property 1 over every continuous field spec in field_specs.py.

For any finite float32 value inside a field's Valid_Range, bp_quant (via the
bpx_quant ctypes wrapper) must return the code of the nearest multiple of
the Resolution, with exact halves rounded away from zero, without reporting
saturation. Storing that code in the field's bits of a Storage_Unit and
reading it back (BP_PUT / BP_GET / bp_sext) returns the same code, so the
decoded value differs from the input by at most half the Resolution.

The reference is exact rational arithmetic (fractions.Fraction) on the
float32 input, i.e. the true value the firmware receives.

bp_dequant returns float32, which cannot hold every code / scale exactly for
the widest fields (odometry distance 1,000,000.00 mm at 0.01 mm). The test
therefore checks the half-Resolution bound on the exact decoded value
code / scale, and checks that bp_dequant returns the float32 nearest to it.

**Validates: Requirements 1.8, 7.5, 4.4, 6.6**
"""

import ctypes
import math
import struct
from fractions import Fraction

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from field_specs import CONTINUOUS_FIELDS, FIELDS_BY_NAME

TITLE = "Continuous stores round to the nearest step"
EXAMPLES_PER_FIELD = 200


# ---------------------------------------------------------------------------
# float32 helpers
# ---------------------------------------------------------------------------
def f32(v):
    """Nearest float32 to v, as a Python float."""
    return struct.unpack("<f", struct.pack("<f", v))[0]


def f32_step(v, up):
    """The float32 next to the float32 v, towards +inf if up else -inf."""
    bits = struct.unpack("<I", struct.pack("<f", v))[0]
    if v == 0.0:
        bits = 0x00000001 if up else 0x80000001
    elif (v > 0.0) == up:
        bits += 1
    else:
        bits -= 1
    return struct.unpack("<f", struct.pack("<I", bits))[0]


def f32_inside(limit, up):
    """Nearest float32 to the exact limit that is not outside it
    (rounded up for a lower limit, down for an upper one)."""
    v = f32(float(limit))
    if up and Fraction(v) < limit:
        v = f32_step(v, True)
    if not up and Fraction(v) > limit:
        v = f32_step(v, False)
    return v


def round_half_away(fr):
    """Exact round-half-away-from-zero of a Fraction."""
    n = math.floor(abs(fr) + Fraction(1, 2))
    return -n if fr < 0 else n


# ---------------------------------------------------------------------------
# value strategies (all values float32 and inside [lo, hi])
# ---------------------------------------------------------------------------
def in_range(spec):
    return lambda x: spec.lo <= Fraction(x) <= spec.hi


def values(spec):
    lo32 = f32_inside(spec.lo, up=True)
    hi32 = f32_inside(spec.hi, up=False)

    # 1. anywhere in the range
    uniform = st.floats(min_value=lo32, max_value=hi32, width=32,
                        allow_nan=False, allow_infinity=False)

    # 2. the float32 nearest to a decimal half step (k + 1/2) / scale, and its
    #    float32 neighbours: the inputs where the rounding direction matters
    def near_half(args):
        k, ulps = args
        v = f32(float(Fraction(2 * k + 1, 2 * spec.scale)))
        for _ in range(abs(ulps)):
            v = f32_step(v, ulps > 0)
        return v

    halves = st.tuples(st.integers(spec.lo_code, spec.hi_code - 1),
                       st.integers(-2, 2)).map(near_half)

    # 3. decimal halves that float32 holds exactly. scale = 2^a * 5^b, so
    #    (k + 1/2) / scale is a dyadic fraction exactly when it equals
    #    (2j + 1) / 2^(a + 1); these must round away from zero.
    a = (spec.scale & -spec.scale).bit_length() - 1
    den = 2 ** (a + 1)
    j_lo = math.ceil((spec.lo * den - 1) / 2)
    j_hi = math.floor((spec.hi * den - 1) / 2)
    exact_halves = st.integers(j_lo, j_hi).map(lambda j: (2 * j + 1) / den)

    # 4. grid points k / scale (nearest float32) and the limits themselves
    grid = st.integers(spec.lo_code, spec.hi_code).map(
        lambda k: f32(float(Fraction(k, spec.scale))))
    limits = st.sampled_from([lo32, hi32, 0.0] if spec.lo <= 0 <= spec.hi else [lo32, hi32])

    return st.one_of(uniform, halves, exact_halves, grid, limits).filter(in_range(spec))


# ---------------------------------------------------------------------------
# store and read one field through the bitpack wrappers
# ---------------------------------------------------------------------------
def quantize(lib, spec, x):
    sat = ctypes.c_bool(True)
    code = lib.bpx_quant(x, *spec.qspec, ctypes.byref(sat))
    return code, sat.value


def store_and_read(lib, spec, code, background):
    shift = spec.shift if spec.shift is not None else 0
    raw = lib.bpx_tc(code, spec.width) if spec.signed else code
    unit = lib.bpx_put(background, shift, spec.width, raw)
    got = lib.bpx_get(unit, shift, spec.width)
    return lib.bpx_sext(got, spec.width) if spec.signed else got


def check_store(lib, spec, x, background):
    assert struct.unpack("<f", struct.pack("<f", x))[0] == x, "input is not a float32"
    exact = Fraction(x)
    assert spec.lo <= exact <= spec.hi

    expected = round_half_away(exact * spec.scale)
    code, sat = quantize(lib, spec, x)
    assert code == expected, (
        f"{spec.name}: bp_quant({x!r}) = {code}, nearest step (half away) is {expected}")
    assert not sat, f"{spec.name}: in-range value {x!r} reported as saturated"
    assert spec.lo_code <= code <= spec.hi_code
    assert code != spec.sentinel, f"{spec.name}: value {x!r} stored as the sentinel"

    read = store_and_read(lib, spec, code, background)
    assert read == code, f"{spec.name}: stored code {code}, read back {read}"

    decoded = Fraction(read, spec.scale)
    assert abs(decoded - exact) <= spec.resolution / 2, (
        f"{spec.name}: {x!r} decoded as {decoded} (more than half a step away)")
    assert lib.bpx_dequant(read, spec.scale) == f32(read / spec.scale)


# The parametrize argument is called ``unit`` and holds the field name, so the
# result recorder in conftest.py reports one line per field.
FIELD_NAMES = [f.name for f in CONTINUOUS_FIELDS]


@pytest.mark.prop(1, TITLE)
@pytest.mark.parametrize("unit", FIELD_NAMES)
@settings(max_examples=EXAMPLES_PER_FIELD)
@given(data=st.data())
def test_continuous_store_rounds_to_nearest_step(lib, unit, data):
    spec = FIELDS_BY_NAME[unit]
    x = data.draw(values(spec), label="x")
    background = data.draw(st.integers(0, 0xFFFFFFFF), label="unit_bits")
    check_store(lib, spec, x, background)


# Fixed examples: minimum, maximum and zero of every field (Requirement 4.4)
# and the representable halfway values next to zero.
@pytest.mark.parametrize("unit", FIELD_NAMES)
def test_limits_zero_and_halves(lib, unit):
    spec = FIELDS_BY_NAME[unit]
    xs = [f32_inside(spec.lo, up=True), f32_inside(spec.hi, up=False)]
    if spec.lo <= 0 <= spec.hi:
        xs.append(0.0)
    a = (spec.scale & -spec.scale).bit_length() - 1
    for h in (Fraction(1, 2 ** (a + 1)), -Fraction(1, 2 ** (a + 1))):
        if spec.lo <= h <= spec.hi and (h * spec.scale).denominator == 2:
            xs.append(float(h))
    for x in xs:
        check_store(lib, spec, x, 0xFFFFFFFF)
        check_store(lib, spec, x, 0)
