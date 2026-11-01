"""Cross-codec tests: C Wire_Codec (app_program/wire.c) <-> Python Laptop_Codec
(MQTT/is02_wire.py).

1. The Python tables match the C sources they copy (wire_ids.h, app_params.c,
   app_status.c, bc_multi.c, app_check.c).
2. Robot -> laptop: frames encoded by the C codec decode in Python to the
   same values (telemetry, heartbeat, every Event_Code, acks).
3. Laptop -> robot: Command_, Map_ and Config_Frames encoded in Python decode
   in C to the intended command.
"""

import ctypes
import random
import re
import struct
import sys
from pathlib import Path

import pytest

from wire_ctypes import (WireAck, WireEvt, WireHb, WireTlm, bind_wire, check_layout,
                         enc_ack, enc_event, enc_heartbeat, enc_telemetry)

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "MQTT"))
import is02_wire as lw  # noqa: E402

APP = ROOT / "app_program"


class WireCmd(ctypes.Structure):
    _fields_ = [("cmd", ctypes.c_uint8), ("letter", ctypes.c_uint8), ("ping", ctypes.c_uint16),
                ("param_id", ctypes.c_uint8), ("value_bits", ctypes.c_uint32),
                ("map_present", ctypes.c_uint8 * 4), ("map_letter", ctypes.c_uint8 * 4)]


CFG_FN = ctypes.CFUNCTYPE(None, ctypes.c_uint32, ctypes.c_uint32, ctypes.c_void_p)


@pytest.fixture(scope="module")
def wl(lib):
    bind_wire(lib)
    check_layout(lib)
    lib.wire_dec_cmd.restype = ctypes.c_int
    lib.wire_dec_cmd.argtypes = [ctypes.c_char_p, ctypes.c_uint32, ctypes.POINTER(WireCmd)]
    lib.wire_dec_map.restype = ctypes.c_int
    lib.wire_dec_map.argtypes = [ctypes.c_char_p, ctypes.c_uint32, ctypes.POINTER(WireCmd)]
    lib.wire_dec_config.restype = ctypes.c_int
    lib.wire_dec_config.argtypes = [ctypes.c_char_p, ctypes.c_uint32, CFG_FN, ctypes.c_void_p]
    return lib


# ---- 1. tables ---------------------------------------------------------------

def _src(name):
    return (APP / name).read_text(encoding="utf-8")


def test_event_table_matches_wire_ids():
    rows = re.findall(r"X\(\s*(\d+),\s*(\w+),\s*([\w\s]+?),\s*([01])\)", _src("wire_ids.h"))
    c_events = [(n, None if v == "1" else int(b.rstrip("u"))) for _c, n, b, v in rows]
    assert c_events == lw.EVENTS


def test_reason_cmd_stop_abort_tables_match():
    s = _src("wire_ids.h")
    reasons = re.findall(r"X\(\s*\d+,\s*\w+,\s*\"(.*?)\"\)", s[s.index("#define WIRE_REASON_TABLE(X)"):s.index("WIRE_X_RSN_ENUM")])
    assert reasons == lw.REASONS
    cmds = re.findall(r"X\(\s*\d+,\s*\w+,\s*\"(.*?)\",\s*(\d+)u", s[s.index("#define WIRE_CMD_TABLE(X)"):s.index("WIRE_X_CMD_ENUM")])
    assert [(e, int(n)) for e, n in cmds] == lw.CMDS
    stops = re.findall(r"X\(\s*\d+,\s*\w+,\s*\"(.*?)\"\)", s[s.index("#define WIRE_STOP_TABLE(X)"):s.index("WIRE_X_SR_ENUM")])
    assert stops == lw.STOP_TEXTS
    aborts = re.findall(r"X\(\s*\d+,\s*\w+,\s*\"(.*?)\"\)", s[s.index("#define WIRE_ABORT_TABLE(X)"):s.index("WIRE_X_ABR_ENUM")])
    assert aborts == lw.ABORT_TEXTS


def test_param_and_name_tables_match():
    p = _src("app_params.c")
    table = p[p.index("PARAMS[] ="):p.index("};", p.index("PARAMS[] ="))]
    assert re.findall(r'\{ "(\w+)"', table) == lw.PARAM_NAMES
    st = _src("app_status.c")
    assert re.findall(r'"([A-Z_]+)"', st[st.index("STATE_NAMES"):st.index("MEV_NAMES")]) == lw.STATE_NAMES
    assert re.findall(r'"([A-Z_]+)"', st[st.index("MEV_NAMES"):st.index("const char *sys_state_name")]) == lw.MEV_NAMES
    bc = _src("bc_multi.c")
    assert re.findall(r'"([^"]+)"', bc[bc.index("NAME[BCM_MAX_LEVELS]"):bc.index(";", bc.index("NAME[BCM_MAX_LEVELS]"))]) == lw.BCM_LEVELS
    ck = _src("app_check.c")
    body = ck[ck.index("bool config_check("):]
    assert re.findall(r'"([A-Z_]+)", &missing', body) == lw.CONFIG_ITEMS


# ---- 2. robot -> laptop ---------------------------------------------------------

def test_telemetry_c_to_python(wl):
    t = WireTlm()
    t.seq, t.t_ms, t.state = 1234, 987654, 2
    t.spd[0], t.spd[1] = 1205, -305
    t.duty[0], t.duty[1] = 455, -1000
    t.enc[0], t.enc[1] = 100000, -5
    t.dist, t.line_l, t.line_r = 123456, 1, 0
    t.ir, t.ir_min, t.ir_max = 2000, 300, 3100
    t.thr_valid, t.thr_b, t.thr_w = 1, 1700, 1300
    t.bc = 20                                   # D8: Code 39 index of "K"
    t.side, t.pitch, t.hump_h = 2, -35, 251
    t.hump_peak, t.hump_max, t.us = 312, 400, 8191
    for i, v in enumerate([1234, 8191, 4310, 0, 99]):
        t.scan[i] = v
    t.imu_ev, t.rssi, t.turn = 4, 62, -455
    t.faults, t.vbat, t.buf = 0x0801, 5100, 3
    f = lw.decode("is02/telemetry", enc_telemetry(wl, t))
    assert f["seq"] == 1234 and f["t"] == 987654 and f["state"] == "LINE_FOLLOWING"
    assert f["spd"] == [120.5, -30.5] and f["duty"] == [45.5, -100.0] and f["enc"] == [100000, -5]
    assert f["dist"] == 12345.6
    assert f["line"] == {"l": 1, "r": 0, "ir": 2000, "irmn": 300, "irmx": 3100, "thr": [1700, 1300]}
    assert f["bc"] == "K"
    assert f["hump"] == {"pitch": -3.5, "h": 25.1, "peak": 31.2, "max": 40.0}
    assert f["obst"] == {"cm": -1.0, "scan": [123.4, -1.0, 431.0, 0.0, 9.9], "side": "R"}
    assert f["imu"] == {"ev": "TURNING", "tr": -45.5}
    assert (f["faults"], f["vbat"], f["rssi"], f["buf"]) == (0x0801, 5100, -62, 3)


def test_heartbeat_c_to_python(wl):
    h = WireHb()
    h.seq, h.state, h.rssi, h.t_ms = 77, 1, 55, 4242
    h.uptime_s, h.reconnects, h.dropped, h.sat = 3600, 2, 9, 0
    h.faults, h.param_crc, h.buf, h.n_params = 0x10, 0xBEEF, 0, 55
    for i, v in enumerate([172, 20, 10, 3]):
        h.ip[i] = v
    d = lw.decode("is02/status", enc_heartbeat(wl, h))
    assert d == {"seq": 77, "state": "IDLE", "rssi": -55, "t": 4242, "up": 3600, "reconnects": 2,
                 "dropped": 9, "sat": 0, "faults": 0x10, "param_crc": 0xBEEF, "buf": 0,
                 "n_params": 55, "broker": "172.20.10.3"}


def _rand_event_fields(code, rng):
    """Random valid argument fields [(value, width)] for Event_Code code."""
    name, fixed = lw.EVENTS[code]
    if name == "FINE_SCAN":
        n = rng.randint(1, 11)
        return [(rng.randint(-60, 60), 7), (n, 4)] + [(rng.choice([511, rng.randint(0, 431)]), 9) for _ in range(n)]
    if name == "BC_DBG":
        v = rng.choice([0, 1, 2, 3, 4])
        if v == 0:
            return [(0, 3)]
        win = [(rng.randint(0, 96), 7), (rng.randint(0, 3), 2), (rng.choice([63, rng.randint(0, 42)]), 6), (rng.randint(0, 1), 1),
               (rng.randint(0, 4095), 12), (rng.randint(0, 4095), 12), (rng.randint(0, 65535), 16)]
        if v == 1:
            return [(1, 3)] + win
        if v == 2:
            return [(2, 3)] + win + [(rng.randint(0, 5), 3), (rng.randint(0, 1), 1)]
        if v == 3:
            k = rng.randint(1, 6)                      # D10: always 6 entries
            return [(3, 3), (k, 3)] + [x for _ in range(6) for x in
                                       ((rng.randint(0, 3), 2), (rng.choice([63, rng.randint(0, 42)]), 6),
                                        (rng.randint(0, 96), 7))]
        k = rng.randint(0, 16)
        return [(4, 3), (rng.randint(1, 6), 3), (k, 5)] + [(rng.randint(0, 4095), 12) for _ in range(k)]
    if fixed == 0:
        return []
    # fixed layout: random bits of the right length, in 1-bit fields
    return [(rng.randint(0, 1), 1) for _ in range(fixed)]


@pytest.mark.parametrize("code", range(len(lw.EVENTS)), ids=[n for n, _ in lw.EVENTS])
def test_every_event_c_to_python(wl, code):
    from wire_ctypes import make_evt
    rng = random.Random(code)
    for _ in range(50):
        ev = make_evt(code, _rand_event_fields(code, rng))
        data = enc_event(wl, 0xABCDE, ev)
        assert data, f"C codec refused event {code}"
        d = lw.decode("is02/event", data)
        assert d["event"] == lw.EVENTS[code][0] and d["t"] == 0xABCDE
        assert isinstance(d["detail"], str)


def test_event_texts_match_baseline():
    """Spot checks: the Python detail text is the Baseline_Build text."""
    w = lw.BitWriter().u(1, 4).u(5, 24).u(lw.EVENT_CODE["OBSTACLE"], 6).u(183, 13).u(412, 16)
    assert lw.decode_event(w.bytes())["detail"] == "at 18.3 cm, STOP issued in 412 us"
    w = lw.BitWriter().u(1, 4).u(5, 24).u(lw.EVENT_CODE["HUMP_END"], 6).u(1, 1).u(312, 14).u(400, 14)
    assert lw.decode_event(w.bytes())["detail"] == "height 31.2 mm, run max 40.0 mm"
    w = lw.BitWriter().u(1, 4).u(5, 24).u(lw.EVENT_CODE["BARCODE"], 6).u(35, 6).u(1, 1)   # C6 index of Z
    assert lw.decode_event(w.bytes())["detail"] == "Z (read backwards)"
    w = lw.BitWriter().u(1, 4).u(5, 24).u(lw.EVENT_CODE["EMERGENCY_STOP"], 6).u(1, 4)
    assert lw.decode_event(w.bytes())["detail"] == "remote E-stop (MQTT)"


def test_acks_c_to_python(wl):
    a = WireAck()
    a.t_ms, a.cmd, a.reason = 100, lw.WCMD["ESTOP"], 7
    a.a.pwm_off_us = 812
    d = lw.decode("is02/ack", enc_ack(wl, a))
    assert d == {"t": 100, "cmd": "ESTOP", "result": "motors off (press BTN1 on the robot to recover)",
                 "pwm_off_us": 812}
    a = WireAck()
    a.t_ms, a.cmd, a.reason = 5, lw.WCMD["GET:MAP"], 0
    for letter, act in (("F", 1), ("K", 3), ("R", 4), ("U", 2)):
        a.a.map.act[ord(letter) - 65] = act
    a.a.map.n = 4
    d = lw.decode("is02/ack", enc_ack(wl, a))
    assert d["map"] == "F=FWD,K=LEFT,R=RIGHT,U=UTURN" and d["n"] == 4 and d["cmd"] == "GET:MAP"
    a = WireAck()
    a.t_ms, a.cmd, a.reason = 5, lw.WCMD["GET:PARAMS"], 0
    a.a.getp.part, a.a.getp.count = 2, 2
    for i, (pid, val) in enumerate([(22, 120.0), (23, 0.3)]):
        a.a.getp.v[i].id, a.a.getp.v[i].w = pid, 32
        a.a.getp.v[i].code = struct.unpack(">I", struct.pack(">f", val))[0]
    d = lw.decode("is02/ack", enc_ack(wl, a))
    assert d["part"] == 2 and d["params"] == {"LINE_BASE_MM_S": 120.0, "LINE_STEER": 0.3}
    a = WireAck()
    a.t_ms, a.cmd, a.reason = 5, lw.WCMD["CONFIG"], lw.RSN_CONFIG_REJECTED
    a.a.cfg.applied, a.a.cfg.n_rej = 3, 1
    a.a.cfg.rej[0] = 18
    d = lw.decode("is02/ack", enc_ack(wl, a))
    assert d["result"] == "IR_WHITE_ADC" and d["applied"] == 3 and d["rejected"] == 1


def test_text_payload_is_rejected():
    with pytest.raises(lw.WireError):
        lw.decode("is02/telemetry", b'{"seq":1}')
    with pytest.raises(lw.WireError):
        lw.decode("is02/event", b"ESTOP")


# ---- 3. laptop -> robot ------------------------------------------------------------

def _dec_cmd(wl, data):
    c = WireCmd()
    st = wl.wire_dec_cmd(data, len(data), ctypes.byref(c))
    return st, c


@pytest.mark.parametrize("text,cmd", [("ESTOP", "ESTOP"), ("RESUME", "RESUME"), ("START", "START"),
                                      ("STOP", "STOP"), ("MAP:CLEAR", "MAP:CLEAR"),
                                      ("GET:MAP", "GET:MAP"), ("GET:PARAMS", "GET:PARAMS")])
def test_simple_commands_python_to_c(wl, text, cmd):
    data = lw.encode_cmd(text)
    assert len(data) == 1
    st, c = _dec_cmd(wl, data)
    assert st == 0 and c.cmd == lw.WCMD[cmd]
    assert lw.describe_cmd(data) == cmd


def test_argument_commands_python_to_c(wl):
    st, c = _dec_cmd(wl, lw.encode_cmd("PING:4242"))
    assert st == 0 and c.cmd == lw.WCMD["PING"] and c.ping == 4242
    st, c = _dec_cmd(wl, lw.encode_cmd("BARCODE:K"))
    assert st == 0 and c.cmd == lw.WCMD["BARCODE"] and c.letter == 10
    st, c = _dec_cmd(wl, lw.encode_cmd("SET:LINE_STEER=0.35"))
    assert st == 0 and c.param_id == lw.PARAM_ID["LINE_STEER"]
    assert struct.unpack(">f", struct.pack(">I", c.value_bits))[0] == pytest.approx(0.35)
    st, c = _dec_cmd(wl, lw.encode_cmd("MAP:F=FWD,K=LEFT,R=RIGHT,U=UTURN"))
    assert st == 0 and list(c.map_present) == [1, 1, 1, 1]
    assert [chr(65 + x) for x in c.map_letter] == ["F", "K", "R", "U"]   # D13: FWD, LEFT, RIGHT, UTURN
    st, c = _dec_cmd(wl, lw.encode_cmd("MAP:Q,W,E,T"))
    # positional text is FWD,UTURN,LEFT,RIGHT (baseline); the binary slots are D13 order
    assert st == 0 and [chr(65 + x) for x in c.map_letter] == ["Q", "E", "T", "W"]


def test_map_frame_python_to_c(wl):
    data = lw.encode_map("A=LEFT,B=RIGHT,C=FWD,D=UTURN")
    assert len(data) == 4
    c = WireCmd()
    assert wl.wire_dec_map(data, len(data), ctypes.byref(c)) == 0
    assert [chr(65 + x) for x in c.map_letter] == ["C", "A", "B", "D"]   # D13: FWD, LEFT, RIGHT, UTURN
    with pytest.raises(lw.WireError):
        lw.encode_map("A=LEFT,A=RIGHT")


def test_config_frame_python_to_c(wl):
    data = lw.encode_config("IR_WHITE_ADC=410\n# comment\nIR_BLACK_ADC=2650\nLINE_BASE_MM_S=90")
    got = []

    @CFG_FN
    def cb(pid, bits, _ctx):
        got.append((pid, struct.unpack(">f", struct.pack(">I", bits))[0]))

    assert wl.wire_dec_config(data, len(data), cb, None) == 0
    assert got == [(lw.PARAM_ID["IR_WHITE_ADC"], 410.0), (lw.PARAM_ID["IR_BLACK_ADC"], 2650.0),
                   (lw.PARAM_ID["LINE_BASE_MM_S"], 90.0)]
    with pytest.raises(lw.WireError):
        lw.encode_config("NOT_A_PARAM=1")


def test_text_command_rejected_by_c(wl):
    st, _c = _dec_cmd(wl, b"ESTOP")      # 'E' = 0x45: version 4
    assert st == 1                        # WIRE_BAD_VERSION
