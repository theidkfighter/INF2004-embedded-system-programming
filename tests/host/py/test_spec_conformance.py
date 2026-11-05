"""Conformance with docs/cr1/spec/design.md (the team contract).

- Telemetry_Frame (D8) and Heartbeat_Frame (D9): every field sits at the bit
  offset and width of the design tables (copied below), in frames made by
  the C Wire_Codec.
- Map body (D13): slots forward, left, right, U-turn.
- Parameter_Store (D15): the 16 discrete parameters live in g_params.bits at
  the design positions; whole values in min..max round trip, other values
  are refused with -3; the defaults equal the Baseline_Build's for its 55
  IDs; the table CRC (D15) matches the laptop codec.
"""

import ctypes
import struct

import pytest

import wire_spec
from wire_ctypes import WireHb, WireTlm, bind_wire, enc_heartbeat, enc_telemetry

# D8: (offset, width, field name in WireTlm[, index])
D8 = [(4, 15, "seq"), (19, 24, "t_ms"), (43, 3, "state"), (46, 15, "spd", 0), (61, 15, "spd", 1),
      (76, 11, "duty", 0), (87, 11, "duty", 1), (98, 24, "enc", 0), (122, 24, "enc", 1),
      (146, 20, "dist"), (166, 2, "line_l"), (168, 2, "line_r"), (170, 12, "ir"), (182, 12, "ir_min"),
      (194, 12, "ir_max"), (206, 1, "thr_valid"), (207, 12, "thr_b"), (219, 12, "thr_w"),
      (231, 6, "bc"), (237, 11, "pitch"), (248, 15, "hump_h"), (263, 14, "hump_peak"),
      (277, 14, "hump_max"), (291, 13, "us")] + \
     [(304 + 13 * i, 13, "scan", i) for i in range(5)] + \
     [(369, 2, "side"), (371, 3, "imu_ev"), (374, 14, "turn"), (388, 16, "faults"), (404, 13, "vbat"),
      (417, 8, "rssi"), (425, 7, "buf")]
D9 = [(4, 14, "seq"), (18, 24, "t_ms"), (42, 3, "state"), (45, 8, "rssi"), (53, 7, "buf"),
      (60, 17, "uptime_s"), (77, 16, "reconnects"), (93, 16, "dropped"), (109, 16, "sat"),
      (125, 16, "faults"), (141, 6, "n_params"), (147, 16, "param_crc")] + \
     [(163 + 8 * i, 8, "ip", i) for i in range(4)]

# D15: name -> (bit shift, width, encoding) with encoding F1 / S2 / B2-1 / B3+1
D15 = {"MOTOR_MAPPING_CONFIRMED": (9, 2, "S2"), "ENC_R_REVERSED": (0, 1, "F1"),
       "ENC_L_REVERSED": (1, 1, "F1"), "ENC_DIRECTION_CONFIRMED": (11, 2, "S2"),
       "LINE_DO_BLACK_HIGH": (13, 2, "S2"), "SERVO_CALIBRATED": (15, 2, "S2"),
       "SERVO_LOW_ANGLE_IS_RIGHT": (2, 1, "F1"), "IMU_FORWARD_AXIS": (19, 2, "B2-1"),
       "IMU_FORWARD_SIGN": (17, 2, "S2"), "TURN_SNAP": (3, 1, "F1"), "BARCODE_AT_JUNCTION": (4, 1, "F1"),
       "BARCODE_DETECT": (5, 1, "F1"), "BARCODE_LEVELS": (21, 3, "B3+1"), "BARCODE_DEBUG": (6, 1, "F1"),
       "OBST_LEGACY": (7, 1, "F1"), "OBST_TRUST_MISS": (8, 1, "F1"),
       "JUNCTION_BLACK_HIGH": (24, 1, "F1")}   # patch/junction_check


def bits_at(data, off, width):
    v = int.from_bytes(data, "big")
    return (v >> (len(data) * 8 - off - width)) & ((1 << width) - 1)


def _fill(struct_obj, table):
    """Distinct in-range values per field; returns {(name, idx): code}."""
    want = {}
    for k, row in enumerate(table):
        off, w, name = row[:3]
        idx = row[3] if len(row) > 3 else None
        code = ((0x5A5A5A5A >> (k % 7)) ^ (k * 2654435761)) & ((1 << w) - 1)
        if name == "thr_valid":
            code = 1
        if name == "state":
            code = 5
        signed = name in ("spd", "duty", "enc", "line_l", "line_r", "pitch", "hump_h", "turn")
        val = code - (1 << w) if (signed and code >> (w - 1)) else code
        if name in ("line_l", "line_r") and val == -2:
            val, code = -1, 3
        if name in ("bc",):
            val = code = 20
        if name in ("side",):
            val = code = 2
        if name in ("ir", "ir_min", "ir_max", "thr_b", "thr_w") and code > 4095:
            val = code = 4095
        if name == "dist" and code > 1000000:
            val = code = 999999
        if name in ("hump_peak", "hump_max") and code > 15000:
            val = code = 14999
        if name in ("us", "scan") and code > 4310 and code != 8191:
            val = code = 4300
        if name == "vbat" and code > 6600:
            val = code = 6000
        if name == "buf" and code > 64:
            val = code = 33
        if name == "n_params" and code > 63:
            val = code = 55
        if name == "uptime_s" and code > 131071:
            val = code = 4000
        if idx is None:
            setattr(struct_obj, name, val)
        else:
            getattr(struct_obj, name)[idx] = val
        want[(name, idx)] = code
    return want


@pytest.fixture(scope="module")
def wl(lib):
    bind_wire(lib)
    return lib


def test_telemetry_offsets_match_d8(wl):
    t = WireTlm()
    want = _fill(t, D8)
    data = enc_telemetry(wl, t)
    assert len(data) == 54
    for row in D8:
        off, w, name = row[:3]
        idx = row[3] if len(row) > 3 else None
        assert bits_at(data, off, w) == want[(name, idx)], (name, idx, off)
    assert D8[-1][0] + D8[-1][1] == 432


def test_heartbeat_offsets_match_d9(wl):
    h = WireHb()
    want = _fill(h, D9)
    data = enc_heartbeat(wl, h)
    assert len(data) == 25
    for row in D9:
        off, w, name = row[:3]
        idx = row[3] if len(row) > 3 else None
        assert bits_at(data, off, w) == want[(name, idx)], (name, idx, off)
    assert bits_at(data, 195, 5) == 0


def test_map_body_slot_order_d13():
    # C encoder; fields: forward, left, right, U-turn (present, letter) = A, B, C, D
    data = bytes.fromhex(wire_spec.run_vectors(["map 1 0 1 1 1 2 1 3"])[0]["hex"])
    assert [bits_at(data, 4 + 6 * k, 6) for k in range(4)] == [32 | 0, 32 | 1, 32 | 2, 32 | 3]
    assert bits_at(data, 28, 4) == 0


# ---- Parameter_Store (D15) ----------------------------------------------------

@pytest.fixture(scope="module")
def ps(lib):
    U, F = ctypes.c_uint32, ctypes.c_float
    for name, res, args in (("params_init", None, []), ("params_set_id", ctypes.c_int32, [U, F]),
                            ("params_get_id", ctypes.c_bool, [U, ctypes.POINTER(F)]),
                            ("params_store_width", U, [U]), ("params_store_code", U, [U]),
                            ("params_table_crc", ctypes.c_uint16, []), ("params_count", U, []),
                            ("params_name", ctypes.c_char_p, [U])):
        fn = getattr(lib, name)
        fn.restype, fn.argtypes = res, args
    lib.params_init()
    yield lib
    lib.params_init()


def _get(ps, i):
    v = ctypes.c_float()
    assert ps.params_get_id(i, ctypes.byref(v))
    return v.value


def _bits(ps):
    return ctypes.c_uint32.in_dll(ps, "g_params").value     # `bits` is the first member


def _code(enc, value):
    if enc == "S2":
        return value & 3
    if enc == "B2-1":
        return value + 1
    if enc == "B3+1":
        return value - 1
    return value


def _range(enc):
    return {"F1": range(0, 2), "S2": range(-1, 2), "B2-1": range(-1, 3), "B3+1": range(1, 7)}[enc]


def test_table_crc_and_names(ps):
    assert ps.params_count() == 60
    names = [ps.params_name(i).decode() for i in range(60)]
    assert names == wire_spec.param_names() and set(D15) <= set(names)
    assert ps.params_table_crc() == wire_spec.table_crc(names)


@pytest.mark.parametrize("name", sorted(D15))
def test_packed_positions_and_round_trip(ps, name):
    pid = wire_spec.param_names().index(name)
    sh, w, enc = D15[name]
    assert ps.params_store_width(pid) == w
    for value in _range(enc):
        before = _bits(ps)
        assert ps.params_set_id(pid, float(value)) == 0
        assert _get(ps, pid) == float(value)
        after = _bits(ps)
        assert (after >> sh) & ((1 << w) - 1) == _code(enc, value)
        assert (after & ~(((1 << w) - 1) << sh)) == (before & ~(((1 << w) - 1) << sh))   # others kept
        assert after >> 25 == 0
        assert ps.params_store_code(pid) == _code(enc, value)
    keep = _get(ps, pid)
    for bad in (0.5, -0.25, float(min(_range(enc)) - 1), float(max(_range(enc)) + 1)):
        assert ps.params_set_id(pid, bad) == -3, bad
        assert _get(ps, pid) == keep
    assert ps.params_set_id(pid, float("nan")) == -2
    ps.params_init()


def test_float_params_store_as_f32(ps):
    pid = wire_spec.param_names().index("LINE_STEER")
    assert ps.params_store_width(pid) == 32
    assert ps.params_set_id(pid, 0.35) == 0
    assert ps.params_store_code(pid) == struct.unpack(">I", struct.pack(">f", 0.35))[0]
    ps.params_init()


def test_defaults_equal_baseline(ps, base):
    base.base_params_init()
    ps.params_init()
    # The Baseline_Build has 55; the JUNCTION_* parameters are new. Defaults
    # changed on purpose since: TRACK_WIDTH_MM (measured 3 Nov), TURN_SNAP (on, 5 Nov).
    changed = {"TRACK_WIDTH_MM", "TURN_SNAP"}
    for i in range(55):
        if wire_spec.param_names()[i] in changed:
            continue
        assert _get(ps, i) == base.base_bh_param_get(i), wire_spec.param_names()[i]
