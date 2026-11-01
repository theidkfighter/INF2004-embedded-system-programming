"""Status_Snapshot (app_program/status.h, app_status.c) and Event_Frame
builder (evt.c) tests.

- every writer is one dispatch-disabled Critical_Section, balanced, never
  nested (crit_stub counts them);
- write -> read round trips at the documented resolution;
- clamping, NaN and the "no echo" sentinel;
- saturating run counters and the run reset;
- fields sharing a unit never disturb each other;
- evt_post() refuses a wrong bit count and posts a frame the C decoder
  reads back with the same code and argument bits.
"""

import ctypes
import math

import pytest

import wire_spec  # noqa: E402

F, U, I, B = ctypes.c_float, ctypes.c_uint32, ctypes.c_int32, ctypes.c_bool

SIGS = {
    "st_set_state": (None, U), "st_state": (U,),
    "st_fault_set": (None, U), "st_fault_clear": (None, U), "st_faults": (U,),
    "st_set_stop": (None, U), "st_stop": (U,),
    "st_set_motion_event": (None, U), "st_motion_event": (U,),
    "st_set_motion": (None, F, F, F, F, I, I, F, F), "st_dist_mm": (F,),
    "st_set_line": (None, B, B, U, U), "st_line_l": (B,), "st_line_r": (B,), "st_ir_adc": (U,),
    "st_set_bc_levels": (None, U, U, B), "st_bc_thr_black": (U,), "st_bc_thr_white": (U,),
    "st_bc_enabled": (B,),
    "st_barcode_read": (None, ctypes.c_char), "st_barcode": (ctypes.c_char,), "st_barcodes_read": (U,),
    "st_set_imu": (None, F, F, F), "st_pitch_deg": (F,),
    "st_set_hump_peak": (None, F), "st_hump_done": (None, F), "st_hump_max_mm": (F,), "st_humps": (U,),
    "st_set_obst_cm": (None, F), "st_obst_cm": (F,), "st_set_scan": (None, U, F), "st_scan_cm": (F, U),
    "st_set_scan_side": (None, ctypes.c_char), "st_scan_side": (ctypes.c_char,),
    "st_obstacle_done": (None,), "st_obstacles": (U,),
    "st_set_vbat": (None, U), "st_vbat": (U,), "st_run_reset": (None,),
    "crit_stub_reset": (None,), "crit_stub_dsp_enters": (U,), "crit_stub_dsp_exits": (U,),
    "crit_stub_errors": (U,), "crit_stub_dsp_max_depth": (U,),
    "sat_stub_reset": (None,), "sat_count": (ctypes.c_uint16,),
    "evt_stub_reset": (None, U), "evt_stub_posted": (U,),
}


class Status(ctypes.Structure):
    _fields_ = [(n, U) for n in ("sys", "mot_a", "mot_b", "mot_c", "mot_d", "mot_e", "line_a", "line_b",
                                 "imu_a", "imu_b", "imu_c", "obs_a", "obs_b", "obs_c", "misc")]


@pytest.fixture(scope="module")
def st(lib):
    for name, (res, *args) in SIGS.items():
        fn = getattr(lib, name)
        fn.restype, fn.argtypes = res, args
    return lib


@pytest.fixture(autouse=True)
def clean(st):
    ctypes.memset(ctypes.addressof(Status.in_dll(st, "g_status")), 0, ctypes.sizeof(Status))
    st.crit_stub_reset()
    st.sat_stub_reset()
    yield
    assert st.crit_stub_errors() == 0, "unbalanced or nested Critical_Section"
    assert st.crit_stub_dsp_enters() == st.crit_stub_dsp_exits()
    assert st.crit_stub_dsp_max_depth() <= 1


def test_snapshot_is_60_bytes(st):
    assert ctypes.sizeof(Status) == 60          # was 156 bytes unpacked


def test_every_write_is_one_section(st):
    st.st_set_state(2)
    st.st_fault_set(0x801)
    st.st_set_line(True, False, 2047, 3)
    st.st_barcode_read(b"K")
    assert st.crit_stub_dsp_enters() == 4


def test_state_faults_stop_mev_share_one_unit(st):
    st.st_set_state(6)
    st.st_fault_set(0xFFFF)                     # design D1: 16 fault bits
    st.st_set_stop(9)
    st.st_set_motion_event(7)
    assert (st.st_state(), st.st_faults(), st.st_stop(), st.st_motion_event()) == (6, 0xFFFF, 9, 7)
    st.st_fault_clear(0x0801)
    st.st_set_state(1)
    assert (st.st_state(), st.st_faults(), st.st_stop(), st.st_motion_event()) == (1, 0xF7FE, 9, 7)
    st.st_fault_clear(0xFFFF)
    st.st_fault_set(0xFFFF0000)                 # bits above the 16 faults are ignored
    assert st.st_faults() == 0 and st.st_stop() == 9 and st.st_motion_event() == 7


@pytest.mark.parametrize("value,expect", [(0.0, 0.0), (1234.56, 1234.6), (99999.0, 99999.0),
                                          (1e9, 100000.0), (-5.0, 0.0), (float("nan"), 0.0)])
def test_dist_resolution_and_clamp(st, value, expect):
    st.st_set_motion(0, 0, 0, 0, 0, 0, value, 1.0)
    assert st.st_dist_mm() == pytest.approx(expect, abs=1e-3)


def test_motion_saturation_counted(st):
    st.st_set_motion(2000.0, -2000.0, 1.5, 0.0, 0, 0, 10.0, 1.0)   # 3 fields clamp
    assert st.sat_count() == 3


def test_line_and_barcode_fields(st):
    st.st_set_line(True, True, 4095, 7)
    st.st_set_bc_levels(2600, 1800, True)
    for _ in range(200):
        st.st_barcode_read(b"Z")
    assert st.st_line_l() and st.st_line_r() and st.st_ir_adc() == 4095
    assert (st.st_bc_thr_black(), st.st_bc_thr_white(), st.st_bc_enabled()) == (2600, 1800, True)
    assert st.st_barcode() == b"Z" and st.st_barcodes_read() == 127      # U7 saturates
    st.st_set_line(False, True, 12, 1)                                    # barcode fields kept
    assert st.st_barcode() == b"Z" and st.st_bc_enabled() and not st.st_line_l()


def test_humps_and_run_reset(st):
    st.st_set_imu(-12.34, 25.0, 100.0)
    assert st.st_pitch_deg() == pytest.approx(-12.3)
    st.st_hump_done(31.25)
    st.st_hump_done(20.0)
    assert st.st_humps() == 2 and st.st_hump_max_mm() == pytest.approx(31.3)
    st.st_barcode_read(b"A")
    st.st_obstacle_done()
    st.st_run_reset()
    assert (st.st_humps(), st.st_hump_max_mm(), st.st_barcodes_read(), st.st_obstacles()) == (0, 0.0, 0, 0)
    assert st.st_barcode() == b"\x00"
    assert st.st_pitch_deg() == pytest.approx(-12.3)      # live values untouched


def test_ultrasonic_sentinel_and_scans(st):
    st.st_set_obst_cm(-1.0)
    assert st.st_obst_cm() == -1.0
    st.st_set_obst_cm(18.34)
    assert st.st_obst_cm() == pytest.approx(18.3)
    st.st_set_obst_cm(900.0)                              # beyond 431.0 cm: clamped, not "no echo"
    assert st.st_obst_cm() == pytest.approx(431.0)
    vals = [12.0, -1.0, 431.0, 0.0, 77.7]
    for i, v in enumerate(vals):
        st.st_set_scan(i, v)
    st.st_set_scan_side(b"R")
    assert [round(st.st_scan_cm(i), 1) for i in range(5)] == vals
    assert st.st_scan_side() == b"R" and st.st_obst_cm() == pytest.approx(431.0)
    for _ in range(70):
        st.st_obstacle_done()
    assert st.st_obstacles() == 63 and st.st_scan_cm(4) == pytest.approx(77.7)


# ---- Event_Frame builder ------------------------------------------------------------

class EvtB(ctypes.Structure):
    pass


def _evt_lib(st):
    lib = st
    lib.evt_post1.restype, lib.evt_post1.argtypes = B, [U, U, U]
    lib.evt_post0.restype, lib.evt_post0.argtypes = B, [U]
    lib.evt_stub_last.restype = ctypes.c_void_p
    lib.wire_enc_event.restype = U
    lib.wire_enc_event.argtypes = [U, ctypes.c_void_p, ctypes.c_void_p, U]
    return lib


def _last_frame(lib):
    p = lib.evt_stub_last()
    t_ms = U.from_address(p).value
    out = (ctypes.c_uint8 * 64)()
    n = lib.wire_enc_event(t_ms, p + 4, out, 64)
    return bytes(out[:n])


def _decoded(lib, data):
    """(t_ms, code, arg bits, arg bytes) through the C decoder wire_dec_event."""
    from wire_ctypes import WireEvt
    ev, t = WireEvt(), U()
    lib.wire_dec_event.restype = ctypes.c_int
    lib.wire_dec_event.argtypes = [ctypes.c_char_p, U, ctypes.POINTER(U), ctypes.POINTER(WireEvt)]
    assert lib.wire_dec_event(data, len(data), ctypes.byref(t), ctypes.byref(ev)) == 0
    return t.value, ev.code, ev.nbits, bytes(ev.args[:(ev.nbits + 7) // 8])


def test_evt_post_checks_bit_count(st):
    lib = _evt_lib(st)
    lib.evt_stub_reset(777)
    code = wire_spec.event_code
    assert not lib.evt_post1(code("STATE"), 2, 4)                 # STATE has 3 bits
    assert lib.evt_stub_posted() == 0
    assert lib.evt_post1(code("EMERGENCY_STOP"), 1, 4)
    d = _decoded(lib, _last_frame(lib))
    assert d == (777, code("EMERGENCY_STOP"), 4, bytes([0x10]))      # stop reason 1 = remote E-stop
    assert lib.evt_post0(code("LINE_LOST"))
    assert _decoded(lib, _last_frame(lib)) == (777, code("LINE_LOST"), 0, b"")
    assert not lib.evt_post0(code("STATE"))                       # 0 bits for a 3-bit event


def test_nan_values_never_crash(st):
    nan = float("nan")
    st.st_set_motion(nan, nan, nan, nan, 0, 0, nan, nan)
    st.st_set_imu(nan, nan, nan)
    st.st_set_obst_cm(nan)
    assert st.st_obst_cm() == -1.0                         # NaN = no echo
    assert not math.isnan(st.st_pitch_deg())
