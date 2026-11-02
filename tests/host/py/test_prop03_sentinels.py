# Feature: minimum-bit-data-representation, Property 3: Sentinel codes never collide with values
"""Property 3 (design "Correctness Properties"): for any field that has a
sentinel and any value, the stored code differs from the sentinel; only the
"no value" input gives the sentinel.

**Validates: Requirements 1.3, 6.2, 11.10**

Continuous fields (field_specs.SENTINEL_FIELDS: the U13@0.1 cm ultrasonic
distances with 8191 = no echo in D2, D6, D8 and D10, and the D10 CM9 integer
cm with 511 = no echo) go through bp_quant (bpx_quant wrapper): every
finite or infinite value, in or out of the Valid_Range, stores a code in
[lo_code, hi_code], never the sentinel; NaN stores the sentinel.

Discrete sentinel: the Code 39 index (C39_NONE = 63, design D2/D8/D10 `C6`)
through c39_to_index / c39_from_index in code39.c.

Not covered here yet (their writers are not in libis02.so at this step):
negative ultrasonic reading -> 8191 (st_put_obst, task 6.2), nav letter
31 = none (msg_nav_encode, task 3.14), presentation of the sentinel as
-1.0 / "" (Laptop_Codec, task 14).
"""

import ctypes
import math

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from field_specs import SENTINEL_FIELDS

TITLE = "Sentinel codes never collide with values"
FIELDS = {f.name: f for f in SENTINEL_FIELDS}
FIELD_NAMES = sorted(FIELDS)


def _quant(lib, x, f):
    sat = ctypes.c_bool(False)
    code = lib.bpx_quant(x, f.scale, f.lo_code, f.hi_code, f.nan_code, ctypes.byref(sat))
    return code, sat.value


def _values(f):
    """Floats around and far beyond the Valid_Range, including the limits,
    the half-steps next to them and +-infinity."""
    s, lo, hi = f.scale, f.lo_code, f.hi_code
    vlo, vhi = lo / s, hi / s
    span = max(vhi - vlo, 1.0)
    near = st.floats(vlo - span, vhi + span, allow_nan=False, width=32)
    edges = st.sampled_from([
        vlo, vhi, (hi + 0.5) / s, (hi + 0.49) / s, (hi + 1) / s,
        (f.sentinel - 0.5) / s, f.sentinel / s, (f.sentinel + 1) / s,
        (lo - 0.5) / s, -0.0, -1.0, math.inf, -math.inf, 3.0e38, -3.0e38,
    ])
    return st.one_of(near, edges, st.floats(allow_nan=False, width=32))


def test_sentinel_fields_present():
    """The ultrasonic distances of D2/D6/D8/D10 and CM9 all have a sentinel."""
    assert "status.obs_a.obst_cm" in FIELDS
    assert "frame.telemetry.obst_cm" in FIELDS
    assert "msg.obstacle.r_min" in FIELDS
    assert "frame.event.CM9" in FIELDS


@pytest.mark.parametrize("unit", FIELD_NAMES)
def test_sentinel_is_reserved_all_ones_code(unit):
    """The sentinel lies outside [lo_code, hi_code] and is the all-ones code
    of the field (design C1 rule 4)."""
    f = FIELDS[unit]
    assert not f.signed
    assert f.sentinel == (1 << f.width) - 1
    assert 0 <= f.lo_code <= f.hi_code < f.sentinel
    assert f.nan_code == f.sentinel


# ---------------------------------------------------------------------------
# Continuous fields
# ---------------------------------------------------------------------------
@pytest.mark.prop(3, TITLE)
@pytest.mark.parametrize("unit", FIELD_NAMES)
@settings(max_examples=1000)
@given(data=st.data())
def test_values_never_store_sentinel(lib, unit, data):
    """Any non-NaN value (in range, out of range, infinite) stores a value
    code in [lo_code, hi_code], never the sentinel, and decodes to a value
    in the Valid_Range."""
    f = FIELDS[unit]
    x = data.draw(_values(f), label="x")
    code, sat = _quant(lib, x, f)
    assert code != f.sentinel, f"{unit}: value {x!r} stored the sentinel {f.sentinel}"
    assert f.lo_code <= code <= f.hi_code
    xf = ctypes.c_float(x).value
    if f.lo_code / f.scale <= xf <= f.hi_code / f.scale:
        assert not sat
    v = lib.bpx_dequant(code, f.scale)
    assert ctypes.c_float(f.lo_code / f.scale).value <= v <= ctypes.c_float(f.hi_code / f.scale).value


@pytest.mark.prop(3, TITLE)
@pytest.mark.parametrize("unit", FIELD_NAMES)
@settings(max_examples=100)
@given(payload=st.integers(0, (1 << 22) - 1), negative=st.booleans(),
       quiet=st.booleans())
def test_nan_stores_sentinel(lib, unit, payload, negative, quiet):
    """Every NaN bit pattern (quiet or signalling, either sign) stores the
    sentinel and reports a saturated store."""
    f = FIELDS[unit]
    mant = (0x400000 | payload) if quiet else (payload or 1)
    bits = 0x7F800000 | mant | (0x80000000 if negative else 0)
    x = ctypes.c_float.from_buffer_copy(ctypes.c_uint32(bits)).value
    assert math.isnan(x)
    assert _quant(lib, x, f) == (f.sentinel, True)


# ---------------------------------------------------------------------------
# Code 39 index (C39_NONE = 63; codes 43..62 never stored)
# ---------------------------------------------------------------------------
C39_COUNT, C39_NONE = 43, 63
C39_CHARS = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%"


def _c39(lib):
    # lib[name] returns a new function object, so these signatures do not
    # touch the shared lib.<name> attributes that other tests may bind.
    to_index = lib["c39_to_index"]
    to_index.restype, to_index.argtypes = ctypes.c_uint32, [ctypes.c_char]
    from_index = lib["c39_from_index"]
    from_index.restype, from_index.argtypes = ctypes.c_char, [ctypes.c_uint32]
    return to_index, from_index


@pytest.mark.prop(3, TITLE, unit="code39_index")
@settings(max_examples=256)
@given(c=st.integers(0, 255))
def test_c39_index_sentinel(lib, c):
    """A Code 39 data character never maps to C39_NONE and round-trips; any
    other byte (0, '*', lower case, ...) maps to C39_NONE."""
    to_index, from_index = _c39(lib)
    ch = bytes([c])
    idx = to_index(ch)
    if ch.decode("latin-1") in C39_CHARS:
        assert idx < C39_COUNT
        assert from_index(idx) == ch
    else:
        assert idx == C39_NONE


@pytest.mark.prop(3, TITLE, unit="code39_index")
@settings(max_examples=200)
@given(idx=st.one_of(st.integers(0, 70), st.integers(0, 0xFFFFFFFF)))
def test_c39_none_reads_back_as_none(lib, idx):
    """C39_NONE, the never-stored codes 43..62 and any larger code read back
    as "none" (0); every value code reads back as its data character."""
    _, from_index = _c39(lib)
    ch = from_index(idx)
    if idx < C39_COUNT:
        assert ch == C39_CHARS[idx].encode("latin-1")
    else:
        assert ch == b"\x00"
