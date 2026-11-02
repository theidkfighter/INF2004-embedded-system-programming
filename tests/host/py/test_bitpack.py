"""Unit tests for app_program/bitpack.h (design C1), called through the
bpx_* wrappers in stubs/bitpack_export.c.

Fixed examples only; the properties over every field spec are in the
Property 1-3 tests."""

import ctypes
import math

import pytest

U32_MAX = 0xFFFFFFFF


def quant(lib, x, scale, lo, hi, nan_code):
    sat = ctypes.c_bool(False)
    code = lib.bpx_quant(x, scale, lo, hi, nan_code, ctypes.byref(sat))
    return code, sat.value


def test_mask(lib):
    assert lib.bpx_mask(0) == 0
    assert lib.bpx_mask(1) == 1
    assert lib.bpx_mask(13) == 0x1FFF
    assert lib.bpx_mask(31) == 0x7FFFFFFF
    assert lib.bpx_mask(32) == U32_MAX


def test_put_get_keeps_other_bits(lib):
    u = lib.bpx_put(U32_MAX, 4, 3, 0b010)
    assert lib.bpx_get(u, 4, 3) == 0b010
    assert u == U32_MAX & ~(0b101 << 4)
    # v wider than w: only the low w bits are written
    assert lib.bpx_put(0, 0, 2, 0xFF) == 0b11
    assert lib.bpx_get(0xDEADBEEF, 0, 32) == 0xDEADBEEF
    assert lib.bpx_put(0, 0, 32, 0x12345678) == 0x12345678


@pytest.mark.parametrize("w", [1, 2, 3, 8, 16, 31, 32])
def test_sext_tc_round_trip_at_limits(lib, w):
    lo, hi = -(1 << (w - 1)), (1 << (w - 1)) - 1
    for v in (lo, -1, 0, 1, hi):
        if lo <= v <= hi:
            code = lib.bpx_tc(v, w)
            assert code <= lib.bpx_mask(w)
            assert lib.bpx_sext(code, w) == v


def test_sext_examples(lib):
    assert lib.bpx_sext(0b11, 2) == -1
    assert lib.bpx_sext(0b01, 2) == 1
    assert lib.bpx_sext(0b10, 2) == -2
    assert lib.bpx_sext(0xF3, 2) == -1         # bits above w are ignored
    assert lib.bpx_sext(0x80000000, 32) == -(1 << 31)
    assert lib.bpx_sext(5, 0) == 0


def test_set_checked_accepts_in_range(lib):
    unit = ctypes.c_uint32(0xFFFF0000)
    assert lib.bpx_set_checked(ctypes.byref(unit), 3, 3, 5, 0, 7)
    assert lib.bpx_get(unit.value, 3, 3) == 5
    assert unit.value & ~(0b111 << 3) == 0xFFFF0000


@pytest.mark.parametrize("v,lo,hi,w", [
    (8, 0, 7, 3),     # above hi
    (0, 1, 5, 3),     # below lo
    (4, 0, 7, 2),     # inside [lo, hi] but wider than the field
])
def test_set_checked_rejects_and_keeps_unit(lib, v, lo, hi, w):
    unit = ctypes.c_uint32(0xA5A5A5A5)
    assert not lib.bpx_set_checked(ctypes.byref(unit), 8, w, v, lo, hi)
    assert unit.value == 0xA5A5A5A5


def test_quant_rounds_half_away_from_zero(lib):
    assert quant(lib, 2.5, 1, -100, 100, -100) == (3, False)
    assert quant(lib, -2.5, 1, -100, 100, -100) == (-3, False)
    assert quant(lib, 0.25, 10, -100, 100, -100) == (3, False)
    assert quant(lib, -0.25, 10, -100, 100, -100) == (-3, False)
    assert quant(lib, 2.4999, 1, -100, 100, -100) == (2, False)
    assert quant(lib, -0.4, 1, -100, 100, -100) == (0, False)


def test_quant_large_codes_are_exact(lib):
    # 6710886.5 is a float; x * 10 = 67108865 needs 27 bits, which float
    # arithmetic would round to 67108864. Codes reach 2^28 (odometry distance).
    assert quant(lib, 6710886.5, 10, 0, (1 << 28) - 1, 0) == (67108865, False)


def test_quant_saturates_to_limits(lib):
    assert quant(lib, 101.0, 1, 0, 100, 8191) == (100, True)
    assert quant(lib, -1.0, 1, 0, 100, 8191) == (0, True)
    assert quant(lib, 100.4, 1, 0, 100, 8191) == (100, False)   # rounds into range
    assert quant(lib, math.inf, 10, -5, 5, -5) == (5, True)
    assert quant(lib, -math.inf, 10, -5, 5, -5) == (-5, True)
    assert quant(lib, 3.0e38, 10000, -5, 5, -5) == (5, True)
    assert quant(lib, -3.0e38, 10000, -5, 5, -5) == (-5, True)
    hi = 0x7FFFFFFF
    assert quant(lib, 3.0e9, 1, -hi - 1, hi, 0) == (hi, True)


def test_quant_nan(lib):
    assert quant(lib, math.nan, 10, 0, 8190, 8191) == (8191, True)   # sentinel
    assert quant(lib, math.nan, 10, -50, 50, -50) == (-50, True)     # lower limit
    assert lib.bpx_quant(math.nan, 10, 0, 10, 0, None) == 0          # p_sat may be NULL


def test_dequant(lib):
    assert lib.bpx_dequant(12345, 10) == pytest.approx(1234.5)
    assert lib.bpx_dequant(-3, 100) == pytest.approx(-0.03)
    assert lib.bpx_dequant((1 << 28) - 1, 10) == ctypes.c_float(((1 << 28) - 1) / 10).value


def test_sat_inc(lib):
    assert lib.bpx_sat_inc(0, 5) == 1
    assert lib.bpx_sat_inc(4, 5) == 5
    assert lib.bpx_sat_inc(5, 5) == 5
    assert lib.bpx_sat_inc(9, 5) == 5
    assert lib.bpx_sat_inc(U32_MAX - 1, U32_MAX) == U32_MAX
    assert lib.bpx_sat_inc(U32_MAX, U32_MAX) == U32_MAX
