"""Unit tests for the host stubs in tests/host/stubs/ (loaded from libis02.so).

The property tests rely on these stubs to see Critical_Sections, the
Saturation_Counter and posted messages, so the stubs themselves are checked
here with fixed examples."""

import ctypes

from conftest import (
    CRIT_ERR_DSP_NESTED,
    CRIT_ERR_DSP_UNDERFLOW,
    CRIT_ERR_IRQ_TOKEN,
    CRIT_ERR_IRQ_UNDERFLOW,
    OS_STUB_MBX_NAV,
    OS_STUB_MBX_TLM,
    OS_STUB_POOL_BLOCKS,
    captured_messages,
    reset_stubs,
)


def test_dsp_section_balanced(lib):
    reset_stubs(lib)
    lib.crit_dsp_enter()
    assert lib.crit_stub_in_dsp()
    lib.crit_dsp_exit()
    assert not lib.crit_stub_in_dsp()
    assert (lib.crit_stub_dsp_enters(), lib.crit_stub_dsp_exits()) == (1, 1)
    assert lib.crit_stub_dsp_depth() == 0
    assert lib.crit_stub_errors() == 0


def test_dsp_nesting_and_underflow_are_errors(lib):
    reset_stubs(lib)
    lib.crit_dsp_enter()
    lib.crit_dsp_enter()          # tk_dis_dsp() does not nest on the target
    assert lib.crit_stub_dsp_max_depth() == 2
    lib.crit_dsp_exit()
    lib.crit_dsp_exit()
    lib.crit_dsp_exit()           # no section open
    assert lib.crit_stub_errors() == 2
    assert lib.crit_stub_error_flags() == CRIT_ERR_DSP_NESTED | CRIT_ERR_DSP_UNDERFLOW
    assert lib.crit_stub_dsp_depth() == 0


def test_irq_sections_nest_in_lifo_order(lib):
    reset_stubs(lib)
    outer = lib.crit_irq_enter()
    inner = lib.crit_irq_enter()
    assert lib.crit_stub_irq_depth() == 2
    lib.crit_irq_exit(inner)
    lib.crit_irq_exit(outer)
    assert lib.crit_stub_irq_depth() == 0
    assert lib.crit_stub_irq_max_depth() == 2
    assert lib.crit_stub_errors() == 0


def test_irq_wrong_order_and_underflow_are_errors(lib):
    reset_stubs(lib)
    outer = lib.crit_irq_enter()
    inner = lib.crit_irq_enter()
    lib.crit_irq_exit(outer)      # wrong value for the inner section
    lib.crit_irq_exit(inner)
    lib.crit_irq_exit(outer)      # no section open
    assert lib.crit_stub_error_flags() == CRIT_ERR_IRQ_TOKEN | CRIT_ERR_IRQ_UNDERFLOW


def test_sat_note_counts_and_holds_at_max(lib):
    reset_stubs(lib)
    assert lib.sat_count() == 0
    lib.sat_note(1)
    lib.sat_note(2)
    assert lib.sat_count() == 3
    lib.sat_stub_set(65534)
    lib.sat_note(1)
    assert lib.sat_count() == 65535
    lib.sat_note(1)
    lib.sat_note(0xFFFFFFFF)
    assert lib.sat_count() == 65535
    assert lib.sat_stub_calls() == 5
    # each call is one balanced dispatch-disabled section
    assert lib.crit_stub_dsp_enters() == 5
    assert lib.crit_stub_dsp_depth() == 0
    assert lib.crit_stub_errors() == 0


def test_sat_note_inside_a_dsp_section_is_a_nesting_error(lib):
    reset_stubs(lib)
    lib.crit_dsp_enter()
    lib.sat_note(1)
    lib.crit_dsp_exit()
    assert lib.crit_stub_error_flags() & CRIT_ERR_DSP_NESTED


def test_os_stub_captures_posts_in_order(lib):
    reset_stubs(lib)
    assert lib.os_stub_post(OS_STUB_MBX_NAV, b"\x21", 1)
    assert lib.os_stub_post(OS_STUB_MBX_TLM, b"\x01\x02\x03", 3)
    assert captured_messages(lib) == [(OS_STUB_MBX_NAV, b"\x21"), (OS_STUB_MBX_TLM, b"\x01\x02\x03")]
    assert lib.os_stub_count_mbx(OS_STUB_MBX_TLM) == 1


def test_os_stub_pool_limit_and_forced_empty(lib):
    reset_stubs(lib)
    for _ in range(OS_STUB_POOL_BLOCKS):
        assert lib.os_stub_post(OS_STUB_MBX_NAV, b"\x00", 1)
    assert not lib.os_stub_post(OS_STUB_MBX_NAV, b"\x00", 1)   # pool exhausted
    lib.os_stub_free(1)
    assert lib.os_stub_post(OS_STUB_MBX_NAV, b"\x00", 1)
    lib.os_stub_set_pool_empty(True)
    assert not lib.os_stub_post(OS_STUB_MBX_NAV, b"\x00", 1)
    assert lib.os_stub_count() == OS_STUB_POOL_BLOCKS + 1
    assert lib.os_stub_rejected() == 2


def test_flag_take_clears_only_matching_bits(lib):
    reset_stubs(lib)
    got = ctypes.c_uint32(0xFFFF)
    assert not lib.flag_take(0x3, ctypes.byref(got))
    assert got.value == 0
    lib.flag_set(0x5)
    lib.flag_set(0x100)
    assert lib.flag_take(0x3, ctypes.byref(got))
    assert got.value == 0x1
    assert lib.flag_peek() == 0x104
    lib.flag_clear(0x100)
    assert lib.flag_peek() == 0x4
    assert lib.os_stub_flag_set_calls() == 2
