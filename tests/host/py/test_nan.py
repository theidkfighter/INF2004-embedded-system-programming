"""NaN into every continuous field spec (design test table: test_nan.py,
Requirement 1.9; edge cases for Property 2).

NaN stores the field's sentinel code when it has one, otherwise the lower
Valid_Range limit, reports the store as saturated (*p_sat) and so adds 1 to
the saturation counter through the caller's sat_note(1) (stubs/sat_stub.c).
"""

import ctypes
import math
import struct

import pytest

from conftest import reset_stubs
from field_specs import CONTINUOUS_FIELDS as CONTINUOUS_SPECS

SPEC_IDS = [f.name for f in CONTINUOUS_SPECS]


def _f32_bits(bits):
    return struct.unpack("<f", struct.pack("<I", bits))[0]


NANS = [
    ("qnan", math.nan),
    ("-qnan", -math.nan),
    ("nan-payload", _f32_bits(0x7FC00001)),
    ("-nan-all-ones", _f32_bits(0xFFFFFFFF)),
]


@pytest.mark.parametrize("spec", CONTINUOUS_SPECS, ids=SPEC_IDS)
def test_nan_stores_sentinel_or_lower_limit(lib, spec):
    # D0 rule 3: the sentinel if the field has one, otherwise the lower limit.
    want = spec.sentinel if spec.sentinel is not None else spec.lo_code
    if spec.sentinel is not None:
        assert spec.sentinel == (1 << spec.width) - 1  # all-ones code (D0 rule 4)
    for label, x in NANS:
        reset_stubs(lib)
        sat = ctypes.c_bool(False)
        code = lib.bpx_quant(x, *spec.qspec, ctypes.byref(sat))
        if sat.value:
            lib.sat_note(1)
        assert (label, code, sat.value, lib.sat_count()) == (label, want, True, 1)
        assert lib.crit_stub_errors() == 0
