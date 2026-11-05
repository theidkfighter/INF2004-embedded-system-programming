"""Example tests for the Wire_Codec robot-to-laptop frames (app_program/wire.c,
task 3.9): frame lengths, Format_Version nibble, zero padding, and
encode -> decode round trips for one fixed sample of each frame type.

The full Property 19 and the unit tests per command code are tasks 3.12 and
3.13."""

import ctypes

import pytest

from wire_ctypes import (
    EVC_BC_DBG, EVC_FINE_SCAN, EVC_OBSTACLE, EVC_STATE,
    WCMD_BARCODE, WCMD_CONFIG, WCMD_ESTOP, WCMD_GET_MAP, WCMD_GET_PARAMS,
    WCMD_PING, WCMD_SET, WCMD_START, WCMD_UNPARSED,
    WIRE_BAD_COUNT, WIRE_BAD_LENGTH, WIRE_BAD_VALUE, WIRE_BAD_VERSION, WIRE_EVT_HDR_BITS,
    WIRE_HB_BITS, WIRE_HB_LEN, WIRE_OK, WIRE_TLM_LEN,
    Br, Bw, WireAck, WireHb, WireTlm, as_dict, bind_wire, bits_of, check_layout,
    dec_ack, dec_event, dec_heartbeat, dec_telemetry, enc_ack, enc_event,
    enc_heartbeat, enc_telemetry, make_evt,
)


@pytest.fixture(scope="module")
def wl(lib):
    bind_wire(lib)
    check_layout(lib)
    return lib


def test_layout_matches_c(wl):
    check_layout(wl)


def test_bitstream_msb_first_and_overflow(wl):
    buf = (ctypes.c_uint8 * 3)(0xFF, 0xFF, 0xFF)
    w = Bw()
    wl.bw_init(ctypes.byref(w), buf, 3)
    assert bytes(buf) == b"\x00\x00\x00"          # zero-filled
    wl.bw_put(ctypes.byref(w), 0b101, 3)
    wl.bw_put(ctypes.byref(w), 0xABC, 12)          # crosses two byte boundaries
    wl.bw_copy(ctypes.byref(w), b"\x80", 1)
    assert not w.b_overflow
    assert wl.bw_bytes(ctypes.byref(w)) == 2
    assert bits_of(bytes(buf))[:16] == "101" + format(0xABC, "012b") + "1"
    wl.bw_put(ctypes.byref(w), 0x1FF, 9)           # 16 + 9 > 24: does not fit
    assert w.b_overflow and w.pos == 16

    r = Br()
    wl.br_init(ctypes.byref(r), bytes(buf), 3)
    assert wl.br_get(ctypes.byref(r), 3) == 0b101
    assert wl.br_get(ctypes.byref(r), 12) == 0xABC
    assert wl.br_get(ctypes.byref(r), 1) == 1
    assert not r.b_short
    assert wl.br_get(ctypes.byref(r), 9) == 0 and r.b_short


def tlm_sample():
    t = WireTlm()
    t.seq, t.t_ms, t.state = 32767, 0xFEDCBA, 5
    t.spd[0], t.spd[1] = -10000, 9999
    t.duty[0], t.duty[1] = -1000, 1000
    t.enc[0], t.enc[1] = -8388607, 8388607
    t.dist, t.line_l, t.line_r = 1000000, -1, 1
    t.ir, t.ir_min, t.ir_max = 4095, 12, 3000
    t.thr_valid, t.thr_b, t.thr_w = 1, 2500, 800
    t.bc, t.pitch, t.hump_h = 42, -900, -15000
    t.hump_peak, t.hump_max, t.us = 15000, 14999, 8191
    for i, v in enumerate([0, 4310, 8191, 1234, 77]):
        t.scan[i] = v
    t.side, t.imu_ev, t.turn = 2, 7, -7200
    t.faults, t.vbat, t.rssi, t.buf = 0xE001, 6600, 128, 64
    return t


def test_telemetry_frame(wl):
    t = tlm_sample()
    data = enc_telemetry(wl, t)
    assert len(data) == WIRE_TLM_LEN == 54
    assert data[0] >> 4 == 1
    bits = bits_of(data)
    assert bits[4:19] == format(32767, "015b")     # sequence at bit 4
    assert bits[425:432] == format(64, "07b")      # Frame_Ring fill is the last field
    st, back = dec_telemetry(wl, data)
    assert st == WIRE_OK
    assert as_dict(back) == as_dict(t)


def test_telemetry_thresholds_off_sent_as_zero(wl):
    t = tlm_sample()
    t.thr_valid = 0
    bits = bits_of(enc_telemetry(wl, t))
    assert bits[206:231] == "0" * 25               # flag + both thresholds


def test_heartbeat_frame(wl):
    h = WireHb()
    h.seq, h.t_ms, h.state, h.rssi, h.buf = 16383, 0xFFFFFF, 3, 67, 12
    h.uptime_s, h.reconnects, h.dropped, h.sat = 86400, 65535, 7, 1
    h.faults, h.n_params, h.param_crc = 0x6000, 55, 0x29B1
    for i, v in enumerate([192, 168, 1, 23]):
        h.ip[i] = v
    data = enc_heartbeat(wl, h)
    assert len(data) == WIRE_HB_LEN == 25
    assert data[0] >> 4 == 1
    assert bits_of(data)[WIRE_HB_BITS:] == "00000"  # 5 padding bits
    assert bits_of(data)[163:195] == "".join(format(o, "08b") for o in (192, 168, 1, 23))
    st, back = dec_heartbeat(wl, data)
    assert st == WIRE_OK
    assert as_dict(back) == as_dict(h)

    h.uptime_s = 10 ** 6                            # U17 saturates
    assert dec_heartbeat(wl, enc_heartbeat(wl, h))[1].uptime_s == 131071


EVENTS = [
    ("STATE", EVC_STATE, [(4, 3)], 5),
    ("OBSTACLE", EVC_OBSTACLE, [(75, 13), (65535, 16)], 8),
    ("FINE_SCAN", EVC_FINE_SCAN, [(-30, 7), (3, 4), (511, 9), (0, 9), (431, 9)], 9),
    ("BC_DBG_widths", EVC_BC_DBG,
     [(4, 3), (3, 3), (16, 5)] + [(100 * i + 7, 12) for i in range(16)], 30),
]


@pytest.mark.parametrize("name,code,fields,length", EVENTS, ids=[e[0] for e in EVENTS])
def test_event_frame(wl, name, code, fields, length):
    ev = make_evt(code, fields)
    data = enc_event(wl, 0x123456, ev)
    assert len(data) == length
    assert data[0] >> 4 == 1
    bits = bits_of(data)
    assert int(bits[28:34], 2) == code
    assert bits[WIRE_EVT_HDR_BITS + ev.nbits:] == "0" * (len(bits) - WIRE_EVT_HDR_BITS - ev.nbits)
    st, t_ms, back = dec_event(wl, data)
    assert st == WIRE_OK and t_ms == 0x123456
    assert as_dict(back) == as_dict(ev)


def test_event_rejects_wrong_nbits_and_count(wl):
    ev = make_evt(EVC_STATE, [(4, 3), (0, 1)])      # STATE has 3 argument bits
    assert enc_event(wl, 0, ev) == b""
    ev = make_evt(EVC_FINE_SCAN, [(0, 7), (12, 4)] + [(1, 9)] * 12)   # count 12 > 11
    assert enc_event(wl, 0, ev) == b""
    assert dec_event(wl, b"\x10" + bytes(8))[0] == WIRE_BAD_LENGTH   # STATE frame is 5 bytes
    # FINE_SCAN with count 0 (count U4 is 1..11)
    bits = "0001" + "0" * 24 + format(EVC_FINE_SCAN, "06b") + "0" * 7 + "0000" + "0" * 9
    assert dec_event(wl, from_bits(bits))[0] == WIRE_BAD_COUNT


def from_bits(bits):
    bits += "0" * (-len(bits) % 8)
    return bytes(int(bits[i:i + 8], 2) for i in range(0, len(bits), 8))


def ack(cmd, reason, t_ms=0xABCDE):
    a = WireAck()
    a.t_ms, a.cmd, a.reason = t_ms, cmd, reason
    return a


def ack_samples():
    """(name, ack, used bits, member to compare)."""
    out = []
    a = ack(WCMD_ESTOP, 7)
    a.a.pwm_off_us = 812
    out.append(("ESTOP", a, 37 + 16, "pwm_off_us"))
    out.append(("START", ack(WCMD_START, 2), 37, None))
    a = ack(WCMD_BARCODE, 9)
    a.a.letter = 30
    out.append(("BARCODE", a, 37 + 5, "letter"))
    a = ack(WCMD_GET_MAP, 0)
    for i, act in enumerate([1, 2, 3, 0] + [0] * 21 + [7]):
        a.a.map.act[i] = act
    a.a.map.n = 4
    out.append(("GET_MAP", a, 37 + 78 + 5, "map"))
    a = ack(WCMD_PING, 4)
    a.a.ping = 65535
    out.append(("PING", a, 37 + 16, "ping"))
    a = ack(WCMD_SET, 13)
    a.a.param_id = 63
    out.append(("SET", a, 37 + 6, "param_id"))
    a = ack(WCMD_GET_PARAMS, 0)
    a.a.getp.part, a.a.getp.count = 5, 12
    for i in range(12):
        a.a.getp.v[i].id, a.a.getp.v[i].w, a.a.getp.v[i].code = 43 + i, 32, 0x3F800000 + i
    out.append(("GET_PARAMS", a, 37 + 7 + 12 * 38, "getp"))
    a = ack(WCMD_CONFIG, 26)
    a.a.cfg.applied, a.a.cfg.n_rej = 52, 3
    for i, pid in enumerate([0, 31, 54]):
        a.a.cfg.rej[i] = pid
    out.append(("CONFIG", a, 37 + 12 + 18, "cfg"))
    out.append(("UNPARSED", ack(WCMD_UNPARSED, 16), 37, None))
    return out


@pytest.mark.parametrize("name,a,used,member", ack_samples(), ids=[s[0] for s in ack_samples()])
def test_ack_frame(wl, name, a, used, member):
    data = enc_ack(wl, a)
    assert len(data) == -(-used // 8)
    assert len(data) == {"ESTOP": 7, "START": 5, "BARCODE": 6, "GET_MAP": 15, "PING": 7,
                         "SET": 6, "GET_PARAMS": 63, "CONFIG": 9, "UNPARSED": 5}[name]
    assert data[0] >> 4 == 1
    bits = bits_of(data)
    assert int(bits[28:32], 2) == a.cmd and int(bits[32:37], 2) == a.reason
    st, back = dec_ack(wl, data)
    assert st == WIRE_OK
    assert (back.t_ms, back.cmd, back.reason) == (a.t_ms, a.cmd, a.reason)
    if member == "getp":
        n = a.a.getp.count
        assert (back.a.getp.part, back.a.getp.count) == (a.a.getp.part, n)
        assert as_dict(back.a.getp)["v"][:n] == as_dict(a.a.getp)["v"][:n]
    elif member == "cfg":
        n = a.a.cfg.n_rej
        assert (back.a.cfg.applied, back.a.cfg.n_rej) == (a.a.cfg.applied, n)
        assert list(back.a.cfg.rej[:n]) == list(a.a.cfg.rej[:n])
    elif member == "map":
        assert as_dict(back.a.map) == as_dict(a.a.map)
    elif member is not None:
        assert getattr(back.a, member) == getattr(a.a, member)
    # padding after the last field is 0, and a set padding bit is rejected
    assert bits[used:] == "0" * (len(bits) - used)
    if len(bits) > used:
        bad = bytearray(data)
        bad[-1] |= 1
        assert dec_ack(wl, bytes(bad))[0] == WIRE_BAD_VALUE


def test_get_params_packed_widths(wl):
    a = ack(WCMD_GET_PARAMS, 0)
    a.a.getp.part, a.a.getp.count = 1, 3
    for i, (pid, w, code) in enumerate([(0, 1, 1), (1, 32, 0x40490FDB), (2, 3, 5)]):
        a.a.getp.v[i].id, a.a.getp.v[i].w, a.a.getp.v[i].code = pid, w, code
    data = enc_ack(wl, a)
    assert len(data) == 13                          # 37 + 7 + (6+1) + (6+32) + (6+3) = 98 bits
    st, back = dec_ack(wl, data, pbits=lambda pid: {0: 1, 1: 32, 2: 3}.get(pid, 0))
    assert st == WIRE_OK
    assert [(v.id, v.w, v.code) for v in back.a.getp.v[:3]] == [(0, 1, 1), (1, 32, 0x40490FDB), (2, 3, 5)]


def test_decoders_reject_text_and_empty(wl):
    text = b"ESTOP"                                 # 'E' = 0x45 reads as version 4
    assert dec_telemetry(wl, text)[0] == WIRE_BAD_VERSION
    assert dec_heartbeat(wl, text)[0] == WIRE_BAD_VERSION
    assert dec_event(wl, text)[0] == WIRE_BAD_VERSION
    assert dec_ack(wl, text)[0] == WIRE_BAD_VERSION
    assert dec_telemetry(wl, b"")[0] == WIRE_BAD_LENGTH
    assert dec_ack(wl, b"")[0] == WIRE_BAD_LENGTH
    assert dec_telemetry(wl, enc_telemetry(wl, tlm_sample())[:-1])[0] == WIRE_BAD_LENGTH


def test_encoders_reject_out_of_range(wl):
    a = ack(12, 0)                                  # command code 12 is unassigned
    assert enc_ack(wl, a) == b""
    a = ack(WCMD_CONFIG, 0)
    a.a.cfg.n_rej = 60
    assert enc_ack(wl, a) == b""
    a = ack(WCMD_GET_PARAMS, 0)
    a.a.getp.part, a.a.getp.count = 1, 13
    assert enc_ack(wl, a) == b""
    a = ack(WCMD_START, 28)                         # Reason_Code 28 is unassigned
    assert enc_ack(wl, a) == b""
