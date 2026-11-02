"""Tasks 12.1, 12.2: index arithmetic and entry layouts of isr_ring.h.

Runs the helpers hal_isr.c uses, through stubs/ring_export.c. These are example
and round-trip checks; the FIFO model of the rings is task 12.5 (Property 12).
"""

import ctypes

import pytest
from hypothesis import given, strategies as st

U, I, B = ctypes.c_uint32, ctypes.c_int32, ctypes.c_bool

BC_RING, BC_IDX_MASK = 64, 0x7F
RAW_RING, RAW_IDX_MASK = 256, 0x1FF
RINGS = [(BC_RING, BC_IDX_MASK), (RAW_RING, RAW_IDX_MASK)]
ODO_MASK = 0x7FFFFFFF
u32s = st.integers(0, 0xFFFFFFFF)


@pytest.fixture(scope="module")
def r(lib):
    sigs = {
        "rx_ring_used": (U, U, U, U),
        "rx_bc_edge_pack": (U, U, B),
        "rx_bc_edge_black": (B, U),
        "rx_bc_edge_odo": (U, U, U),
        "rx_raw_slot_pack": (U, U, U, U, B),
        "rx_raw_delta": (I, U, U),
    }
    for name, (res, *args) in sigs.items():
        fn = getattr(lib, name)
        fn.restype, fn.argtypes = res, args
    return lib


@pytest.mark.parametrize("ring,mask", RINGS, ids=["edge", "raw"])
def test_used_counts_across_the_index_wrap(r, ring, mask):
    for tail in range(mask + 1):
        for used in (0, 1, ring - 1, ring):
            head = (tail + used) & mask
            assert r.rx_ring_used(head, tail, mask) == used


@pytest.mark.parametrize("ring,mask", RINGS, ids=["edge", "raw"])
def test_full_differs_from_empty_and_shares_a_slot(r, ring, mask):
    tail = mask - 2
    head = (tail + ring) & mask
    assert head != tail
    assert head & (ring - 1) == tail & (ring - 1)
    assert r.rx_ring_used(head, tail, mask) == ring
    assert r.rx_ring_used(tail, tail, mask) == 0


@given(u32s, st.booleans(), st.integers(0, ODO_MASK))
def test_edge_round_trip_with_the_odometer_wrap(r, odo, b_black, age):
    slot = r.rx_bc_edge_pack(odo, b_black)
    assert r.rx_bc_edge_black(slot) == b_black
    assert slot & ODO_MASK == odo & ODO_MASK
    assert r.rx_bc_edge_odo(slot, (odo + age) & 0xFFFFFFFF) == odo


@given(st.integers(0, 4095), u32s, u32s, st.booleans())
def test_raw_slot_fields_stay_apart(r, adc, er, el, b_gap):
    slot = r.rx_raw_slot_pack(adc, er, el, b_gap)
    assert slot & 0xFFF == adc
    assert (slot >> 12) & 0x1FF == er & 0x1FF
    assert (slot >> 21) & 0x1FF == el & 0x1FF
    assert bool(slot & (1 << 30)) == b_gap
    assert slot >> 31 == 0


@given(u32s, st.integers(-256, 255))
def test_raw_delta_returns_the_signed_step(r, count, step):
    assert r.rx_raw_delta((count + step) & 0x1FF, count & 0x1FF) == step
