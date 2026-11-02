"""Property 18: critical sections are balanced and cover every shared update.

Runs the real accessors (app_status.c, app_shared.c) against crit_stub.c:

- watches: a byte of a shared unit that changes with no dispatch-disabled
  section open counts as a write outside a section;
- hooks: the enter and exit hooks stand for another task, which on the target
  can run just before or just after a section but never inside it. A read
  that really is inside the section sees only the enter-hook data.

The test_watch_* and test_hook_* cases show that these checks can fail.
"""

import ctypes
import math
from pathlib import Path

import pytest
from hypothesis import given, strategies as st

from conftest import HOST_DIR, reset_stubs

F, U, I, B = ctypes.c_float, ctypes.c_uint32, ctypes.c_int32, ctypes.c_bool
U8, U16 = ctypes.c_uint8, ctypes.c_uint16
HOOK = ctypes.CFUNCTYPE(None)

TITLE = "Critical sections are balanced and cover every shared update"
APP_DIR = HOST_DIR.parent.parent / "app_program"

BUZZ_WARN = 6
N_ESTOP_PENDING, N_ACK_DUE, N_CONFIG_DUE = 1, 2, 4
ODO_WORDS = 5
STATUS_BYTES = 60

SIGS = {
    "st_set_state": (None, U), "st_state": (U,),
    "st_fault_set": (None, U), "st_fault_clear": (None, U), "st_faults": (U,),
    "st_set_stop": (None, U), "st_stop": (U,),
    "st_set_motion_event": (None, U),
    "st_set_motion": (None, F, F, F, F, I, I, F, F), "st_dist_mm": (F,),
    "st_set_line": (None, B, B, U, U), "st_ir_adc": (U,),
    "st_set_bc_levels": (None, U, U, B),
    "st_barcode_read": (None, ctypes.c_char), "st_barcodes_read": (U,),
    "st_set_imu": (None, F, F, F), "st_pitch_deg": (F,),
    "st_set_hump_peak": (None, F), "st_hump_done": (None, F), "st_humps": (U,),
    "st_set_obst_cm": (None, F), "st_obst_cm": (F,), "st_set_scan": (None, U, F),
    "st_set_scan_side": (None, ctypes.c_char),
    "st_obstacle_done": (None,), "st_obstacles": (U,),
    "st_set_vbat": (None, U), "st_vbat": (U,), "st_run_reset": (None,),
    "buzz_put": (B, U), "buzzer_pattern_take": (U,),
    "estop_rx_put": (None, U), "estop_rx_take": (B, ctypes.POINTER(U)),
    "estop_ack_request": (None,), "estop_ack_take": (B,),
    "estop_pwm_off_put": (None, U), "estop_pwm_off": (U16,),
    "config_due_set": (None,), "config_due_clear": (None,), "config_due": (B,),
    "motion_done_put": (None, U16, U), "motion_done_get": (U,),
    "crit_stub_set_hooks": (None, HOOK, HOOK),
}


class Odometry(ctypes.Structure):
    _fields_ = [("dist_mm", F), ("heading_deg", F), ("v_mm_s", F),
                ("a_fwd_g", F), ("turn_dps", F), ("odo_um", U)]


@pytest.fixture(scope="module")
def s(lib):
    for name, (res, *args) in SIGS.items():
        fn = getattr(lib, name)
        fn.restype, fn.argtypes = res, args
    lib.odometry_put.restype, lib.odometry_put.argtypes = None, [ctypes.POINTER(Odometry)]
    lib.odometry_get.restype, lib.odometry_get.argtypes = None, [ctypes.POINTER(Odometry)]
    return lib


def unit(s, name, ctype=U):
    return ctype.in_dll(s, name)


def odo_words(s):
    return (U * ODO_WORDS).in_dll(s, "g_odo_rec")


def start(s, *regions):
    """Reset the stubs, zero the units, watch the given (object, size) regions."""
    reset_stubs(s)
    ctypes.memset(ctypes.addressof(unit(s, "g_status")), 0, STATUS_BYTES)
    unit(s, "g_buzz", U8).value = 0
    unit(s, "g_net_flags", U8).value = 0
    unit(s, "g_estop_rx_us").value = 0
    unit(s, "g_estop_pwm_off_us", U16).value = 0
    unit(s, "g_done").value = 0
    ctypes.memset(ctypes.addressof(odo_words(s)), 0, ctypes.sizeof(odo_words(s)))
    for obj, size in regions:
        assert s.crit_stub_watch(ctypes.addressof(obj), size)


def assert_clean(s):
    """Balanced, not nested, depth 0, nothing written outside a section."""
    assert s.crit_stub_errors() == 0
    assert s.crit_stub_dsp_depth() == 0
    assert s.crit_stub_dsp_enters() == s.crit_stub_dsp_exits()
    assert s.crit_stub_dsp_max_depth() <= 1
    assert s.crit_stub_outside_writes() == 0


# ---- Status_Snapshot units --------------------------------------------------

floats = st.floats(allow_nan=True, allow_infinity=True, width=32)
counts = st.integers(0, 0xFFFFFFFF)
ints = st.integers(-(2 ** 31), 2 ** 31 - 1)
chars = st.sampled_from([b"\x00", b"A", b"L", b"R", b"Z", b"7"])

WRITERS = [
    st.tuples(st.just("st_set_state"), st.integers(0, 7)),
    st.tuples(st.just("st_fault_set"), counts),
    st.tuples(st.just("st_fault_clear"), counts),
    st.tuples(st.just("st_set_stop"), st.integers(0, 9)),
    st.tuples(st.just("st_set_motion_event"), st.integers(0, 7)),
    st.tuples(st.just("st_set_motion"), floats, floats, floats, floats, ints, ints, floats, floats),
    st.tuples(st.just("st_set_line"), st.booleans(), st.booleans(), counts, st.integers(0, 6)),
    st.tuples(st.just("st_set_bc_levels"), counts, counts, st.booleans()),
    st.tuples(st.just("st_barcode_read"), chars),
    st.tuples(st.just("st_set_imu"), floats, floats, floats),
    st.tuples(st.just("st_set_hump_peak"), floats),
    st.tuples(st.just("st_hump_done"), floats),
    st.tuples(st.just("st_set_obst_cm"), floats),
    st.tuples(st.just("st_set_scan"), st.integers(0, 5), floats),
    st.tuples(st.just("st_set_scan_side"), chars),
    st.tuples(st.just("st_obstacle_done")),
    st.tuples(st.just("st_set_vbat"), counts),
    st.tuples(st.just("st_run_reset")),
]
READERS = ["st_state", "st_faults", "st_stop", "st_dist_mm", "st_ir_adc", "st_barcodes_read",
           "st_pitch_deg", "st_humps", "st_obst_cm", "st_obstacles", "st_vbat"]


# Feature: minimum-bit-data-representation, Property 18: Critical sections are balanced and cover every shared update
@pytest.mark.prop(18, TITLE, unit="Status_Snapshot units")
@given(st.lists(st.one_of(WRITERS), min_size=1, max_size=30))
def test_status_units(s, calls):
    start(s, (unit(s, "g_status"), STATUS_BYTES))
    for name, *args in calls:
        before = s.crit_stub_changed_sections()
        getattr(s, name)(*args)
        assert_clean(s)
        assert s.crit_stub_changed_sections() - before <= 1, name
    enters = s.crit_stub_dsp_enters()
    for name in READERS:
        getattr(s, name)()
    assert s.crit_stub_dsp_enters() == enters       # readers: one load, no section


# ---- g_buzz -----------------------------------------------------------------

@pytest.mark.prop(18, TITLE, unit="g_buzz")
@given(st.lists(st.one_of(st.integers(0, 40), st.just(0xFFFFFFFF), st.none()), min_size=1, max_size=30))
def test_buzz(s, ops):
    start(s, (unit(s, "g_buzz", U8), 1))
    model = 0
    for op in ops:
        enters = s.crit_stub_dsp_enters()
        if op is None:
            assert s.buzzer_pattern_take() == model
            model = 0
            assert s.crit_stub_dsp_enters() == enters + 1
        else:
            b_ok = op <= BUZZ_WARN
            assert s.buzz_put(op) == b_ok
            model = op if b_ok else model
        assert unit(s, "g_buzz", U8).value == model
        assert_clean(s)


def test_buzz_take_reads_and_clears_inside_one_section(s):
    start(s, (unit(s, "g_buzz", U8), 1))
    g_buzz = unit(s, "g_buzz", U8)
    seen = []

    def before():
        g_buzz.value = 3

    def after():
        seen.append(g_buzz.value)                   # cleared by the time the section ends
        g_buzz.value = 5

    enter, leave = HOOK(before), HOOK(after)
    s.crit_stub_set_hooks(enter, leave)
    assert s.buzzer_pattern_take() == 3
    s.crit_stub_reset()
    assert seen == [0]


# ---- g_net_flags and g_estop_rx_us ------------------------------------------

NET_OPS = st.one_of(
    st.tuples(st.just("rx"), st.integers(0, 0xFFFFFFFF)),
    st.sampled_from([("take",), ("ack_req",), ("ack_take",), ("cfg_set",), ("cfg_clear",)]),
)


@pytest.mark.prop(18, TITLE, unit="g_net_flags, g_estop_rx_us")
@given(st.lists(NET_OPS, min_size=1, max_size=40))
def test_net_flags(s, ops):
    flags = unit(s, "g_net_flags", U8)
    start(s, (flags, 1), (unit(s, "g_estop_rx_us"), 4))
    m_flags, m_rx = 0, 0
    got = U()
    for op in ops:
        before = s.crit_stub_changed_sections()
        enters = s.crit_stub_dsp_enters()
        if op[0] == "rx":
            s.estop_rx_put(op[1])
            m_rx = op[1]
            m_flags |= N_ESTOP_PENDING | N_ACK_DUE
        elif op[0] == "take":
            pending = bool(m_flags & N_ESTOP_PENDING)
            got.value = 0xDEAD
            assert s.estop_rx_take(ctypes.byref(got)) == pending
            assert got.value == (m_rx if pending else 0xDEAD)
            m_flags &= ~N_ESTOP_PENDING
        elif op[0] == "ack_req":
            s.estop_ack_request()
            m_flags |= N_ACK_DUE
        elif op[0] == "ack_take":
            due = (m_flags & (N_ACK_DUE | N_ESTOP_PENDING)) == N_ACK_DUE
            assert s.estop_ack_take() == due
            m_flags &= ~N_ACK_DUE if due else 0xFF
        elif op[0] == "cfg_set":
            s.config_due_set()
            m_flags |= N_CONFIG_DUE
        else:
            s.config_due_clear()
            m_flags &= ~N_CONFIG_DUE
        assert flags.value == m_flags
        assert s.config_due() == bool(m_flags & N_CONFIG_DUE)
        assert s.crit_stub_dsp_enters() == enters + 1, op          # one section per update
        assert s.crit_stub_changed_sections() - before <= 1, op    # time and flag together
        assert_clean(s)


def test_estop_rx_take_gets_one_consistent_pair(s):
    start(s)
    flags, rx = unit(s, "g_net_flags", U8), unit(s, "g_estop_rx_us")

    def before():
        rx.value, flags.value = 1111, N_ESTOP_PENDING

    def after():
        rx.value, flags.value = 2222, 0

    enter, leave = HOOK(before), HOOK(after)
    s.crit_stub_set_hooks(enter, leave)
    got = U()
    assert s.estop_rx_take(ctypes.byref(got))
    s.crit_stub_reset()
    assert got.value == 1111


@pytest.mark.prop(18, TITLE, unit="g_estop_pwm_off_us")
@given(st.integers(0, 0xFFFFFFFF))
def test_pwm_off_saturates_with_one_store(s, us):
    start(s, (unit(s, "g_estop_pwm_off_us", U16), 2))
    s.estop_pwm_off_put(us)
    assert s.estop_pwm_off() == min(us, 65535)
    assert s.crit_stub_dsp_enters() == 0            # single writer, no section


# ---- odometry ---------------------------------------------------------------

def odo_record(i):
    return Odometry(100.0 * i, 10.0 * i, 1.0 * i, 0.1 * i, 2.0 * i, 1000 * i)


def odo_fields(o):
    return (o.dist_mm, o.heading_deg, o.v_mm_s, o.a_fwd_g, o.turn_dps, o.odo_um)


def in_range(lo, hi):
    return st.floats(lo, hi, allow_nan=False, allow_infinity=False, width=32)


@pytest.mark.prop(18, TITLE, unit="odometry_put, odometry_get")
@given(st.lists(st.tuples(in_range(-1e6, 1e6), in_range(-1e5, 1e5), in_range(-1e3, 1e3),
                          in_range(-4.0, 4.0), in_range(-720.0, 720.0), st.integers(0, 0xFFFFFFFF)),
                min_size=1, max_size=20))
def test_odometry(s, records):
    start(s, (odo_words(s), ODO_WORDS * 4))
    for rec in records:
        s.crit_stub_reset()
        assert s.crit_stub_watch(ctypes.addressof(odo_words(s)), ODO_WORDS * 4)
        s.odometry_put(ctypes.byref(Odometry(*rec)))
        assert s.crit_stub_dsp_enters() == 1            # the whole record in one section
        assert s.crit_stub_changed_sections() <= 1
        assert_clean(s)
        out = Odometry()
        s.odometry_get(ctypes.byref(out))
        assert s.crit_stub_dsp_enters() == 2
        assert_clean(s)
        assert out.odo_um == rec[5]
        for got, want, step in zip(odo_fields(out), rec, (0.01, 0.01, 0.01, 1e-4, 0.01)):
            assert math.isclose(got, want, abs_tol=step * 0.51 + abs(want) * 2e-7)


def test_odometry_put_counts_clamps_after_the_section(s):
    start(s, (odo_words(s), ODO_WORDS * 4))
    s.odometry_put(ctypes.byref(Odometry(0.0, 0.0, 5000.0, 0.0, 0.0, 0)))   # v clamps
    assert s.sat_count() == 1
    assert s.crit_stub_dsp_enters() == 2                # record, then the counter
    assert s.crit_stub_changed_sections() == 1          # only the record is watched
    assert_clean(s)


def test_odometry_get_copies_inside_the_section(s):
    start(s)
    s.odometry_put(ctypes.byref(odo_record(1)))
    a = list(odo_words(s))
    s.odometry_put(ctypes.byref(odo_record(2)))
    b = list(odo_words(s))
    words = odo_words(s)

    def before():
        words[:] = a

    def after():
        words[:] = b

    enter, leave = HOOK(before), HOOK(after)
    s.crit_stub_set_hooks(enter, leave)
    out = Odometry()
    s.odometry_get(ctypes.byref(out))
    s.crit_stub_reset()
    assert odo_fields(out) == pytest.approx(odo_fields(odo_record(1)))


# ---- motion done word -------------------------------------------------------

@pytest.mark.prop(18, TITLE, unit="g_done")
@given(st.lists(st.tuples(st.integers(0, 65535), st.integers(0, 2)), min_size=1, max_size=30))
def test_done_word(s, writes):
    g_done = unit(s, "g_done")
    start(s, (g_done, 4))
    for i, (done_id, res) in enumerate(writes, start=1):
        s.motion_done_put(done_id, res)
        assert s.crit_stub_dsp_enters() == i            # every update in one section
        assert_clean(s)
        assert g_done.value == done_id | (res << 16)    # id [0..15], result [16..17], rest 0
        assert s.motion_done_get() == g_done.value
        assert s.crit_stub_dsp_enters() == i            # the reader is one load


def test_done_word_has_one_store_and_one_load_in_the_source():
    src = (APP_DIR / "tsk_motion.c").read_text(encoding="utf-8")
    assert "g_done_id" not in src and "g_done_result" not in src
    assert src.count("motion_done_put(") == 1
    assert src.count("motion_done_get(") == 1


# ---- the checks themselves can fail -----------------------------------------

def test_watch_catches_a_write_outside_a_section(s):
    start(s, (unit(s, "g_buzz", U8), 1))
    unit(s, "g_buzz", U8).value = 4
    assert s.crit_stub_outside_writes() == 1


def test_watch_catches_a_write_before_a_section_opens(s):
    start(s, (unit(s, "g_status"), STATUS_BYTES))
    unit(s, "g_status").value = 0x55
    s.st_set_state(2)
    assert s.crit_stub_outside_writes() == 1


def test_watch_passes_a_write_inside_a_section(s):
    start(s, (unit(s, "g_buzz", U8), 1))
    s.crit_dsp_enter()
    unit(s, "g_buzz", U8).value = 4
    s.crit_dsp_exit()
    assert s.crit_stub_outside_writes() == 0
    assert s.crit_stub_changed_sections() == 1


def test_hook_shows_a_read_outside_the_section(s):
    """A read after the exit hook sees the other task's data: the odometry test
    above would fail for code that copied outside the section."""
    start(s)
    g_buzz = unit(s, "g_buzz", U8)

    def before():
        g_buzz.value = 1

    def after():
        g_buzz.value = 2

    enter, leave = HOOK(before), HOOK(after)
    s.crit_stub_set_hooks(enter, leave)
    s.crit_dsp_enter()
    assert g_buzz.value == 1
    s.crit_dsp_exit()
    assert g_buzz.value == 2
    s.crit_stub_reset()


@pytest.fixture(autouse=True)
def stubs_clean(s):
    yield
    reset_stubs(s)
