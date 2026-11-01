# Feature: minimum-bit-data-representation, Property 9: Code 39 index mapping is a bijection
"""Property 9: c39_to_index and c39_from_index are inverse bijections between
the 43 Code 39 data characters and the indices 0..42.

**Validates: Requirements 5.3**
"""

import ctypes

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

TITLE = "Code 39 index mapping is a bijection"
CHARS = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%"


def _fns(lib):
    to_index = lib["c39_to_index"]
    to_index.restype, to_index.argtypes = ctypes.c_uint32, [ctypes.c_char]
    from_index = lib["c39_from_index"]
    from_index.restype, from_index.argtypes = ctypes.c_char, [ctypes.c_uint32]
    return to_index, from_index


@pytest.mark.prop(9, TITLE, unit="code39")
@settings(max_examples=200)
@given(i=st.integers(0, 42))
def test_index_char_index(lib, i):
    to_index, from_index = _fns(lib)
    assert to_index(from_index(i)) == i


@pytest.mark.prop(9, TITLE, unit="code39")
@settings(max_examples=200)
@given(c=st.sampled_from(CHARS))
def test_char_index_char(lib, c):
    to_index, from_index = _fns(lib)
    assert from_index(to_index(c.encode())) == c.encode()


def test_image_is_exactly_0_to_42(lib):
    to_index, _ = _fns(lib)
    assert sorted(to_index(c.encode()) for c in CHARS) == list(range(43))
