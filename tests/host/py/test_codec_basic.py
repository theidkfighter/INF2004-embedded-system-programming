"""Quick self-tests of the Python Laptop_Codec (MQTT/is02_codec.py, tasks 14.1 and
14.2): hand-made records for each payload kind, hand-built frames, the CRC,
the code tables against wire_ids.h and the parameter table against wire_dump, and the event texts
against the Baseline_Build strings.

The property tests and the cross-codec vectors are LT-04."""

import json
import logging
import math
import re
import struct

import pytest

import wire_spec
from codec_helpers import REPO_DIR, is02_codec as codec

WIRE_IDS = (REPO_DIR / "app_program" / "wire_ids.h").read_text(encoding="utf-8")

TELEMETRY = {
    "seq": 32767, "t": 16777215, "state": 2, "spd_l": -1234, "spd_r": 1000, "duty_l": -1000, "duty_r": 455,
    "enc_l": -8388607, "enc_r": 8388607, "dist": 1000000, "line_l": -1, "line_r": 1, "ir": 4095, "ir_min": 12,
    "ir_max": 3000, "thr_valid": 1, "thr_b": 2000, "thr_w": 900, "bc": 20, "pitch": -900, "hump_h": -15000,
    "hump_peak": 15000, "hump_max": 1234, "us": 8191, "scan": [8191, 4310, 0, 123, 8191], "side": 2, "imu_ev": 7,
    "turn": -7200, "faults": 0xA005, "vbat": 6600, "rssi": 128, "buf": 64,
}

HEARTBEAT = {
    "seq": 16383, "t": 123456, "state": 1, "rssi": 57, "buf": 3, "up": 131071, "reconnects": 65535,
    "dropped": 2, "sat": 7, "faults": 0x6000, "ptab_count": 55, "ptab_crc": 0xBEEF, "broker": [192, 168, 1, 20],
}

MAP_ACTS = [0] * 26
MAP_ACTS[ord("F") - 65], MAP_ACTS[ord("K") - 65] = 1, 3
MAP_ACTS[ord("R") - 65], MAP_ACTS[ord("U") - 65] = 4, 2
MAP_TEXT = "F=FWD,K=LEFT,R=RIGHT,U=UTURN"

LEVELS = dict(n_levels=6)
for i, (res, code, n) in enumerate([(1, 35, 29), (2, 0, 27), (0, 0, 3), (1, 35, 28), (2, 0, 26), (0, 0, 0)]):
    LEVELS.update({f"res{i}": res, f"code{i}": code, f"n{i}": n})
WINDOW = {"n": 29, "res": 1, "code": 20, "rev": 1, "thr_b": 1500, "thr_w": 900, "ovr": 2}

# (Event name, args, the Baseline_Build detail text)
EVENT_CASES = [
    ("STATE", {"state": 2}, "LINE_FOLLOWING"),
    ("IMU_OK", {"chip": 1, "addr": 0x19}, "LSM303DLHC at 0x19"),
    ("IMU_OK", {"chip": 2, "addr": 0x1E}, "LSM303D at 0x1E"),
    ("IMU_OK", {"chip": 0, "addr": 0}, "none"),
    ("FAULT_IMU_I2C", {}, "I2C1 timeout: bus recovery"),
    ("FAULT_IMU", {}, "3 failed recoveries: driving without the IMU"),
    ("HUMP_START", {"valid": 1, "h": 0, "max": 1234}, "height 0.0 mm, run max 123.4 mm"),
    ("HUMP_PEAK", {"valid": 1, "h": 505, "max": 1234}, "height 50.5 mm, run max 123.4 mm"),
    ("HUMP_END", {"valid": 1, "h": 505, "max": 1234}, "height 50.5 mm, run max 123.4 mm"),
    ("HUMP_END", {"valid": 0, "h": 0, "max": 0}, "false slope (no height reported)"),
    ("MAP", {"map": MAP_ACTS}, MAP_TEXT),
    ("MAP", {"map": [0] * 26}, "cleared"),
    ("BATTERY_LOW", {}, "below VBAT_LOW_MV"),
    ("START_REFUSED", {}, "calibration missing (see console)"),
    ("MAP_WARNING", {"map": MAP_ACTS}, MAP_TEXT),
    ("MAP_WARNING", {"map": [0] * 26}, "no barcode map yet (send MAP:...)"),
    ("MISSION_COMPLETE", {"t": 95, "dist": 1205, "bc": 4, "humps": 2, "max": 305, "obst": 1},
     "t 95s dist 12.05m bc 4 humps 2 max 30.5mm obst 1"),
    ("TURN_SNAP", {"not_seen": 0}, "stopped on the line"),
    ("TURN_SNAP", {"not_seen": 1}, "line not seen, encoder angle kept"),
    ("NAV_ACTION", {"letter": 10, "action": 3}, "K -> LEFT"),
    ("NAV_ACTION", {"letter": 25, "action": 0}, "Z -> NONE (not in the MQTT map): straight on"),
    ("NAV", {"variant": 0, "letter": 0, "action": 4}, "RIGHT"),
    ("NAV", {"variant": 1, "letter": 0, "action": 0}, "turn did not complete"),
    ("NAV", {"variant": 2, "letter": 10, "action": 3}, "K stored for the next junction (LEFT)"),
    ("NAV", {"variant": 3, "letter": 2, "action": 0}, "barcode fault: going straight (C)"),
    ("EMERGENCY_STOP", {"reason": 1}, "remote E-stop (MQTT)"),
    ("ERROR", {"reason": 9}, "fault reported (see faults in telemetry)"),
    ("RECOVERED", {}, "re-check passed"),
    ("RECOVERY_REFUSED", {"why": 0b11}, "obstacle closer than 8 cm. IMU INT1 still active. "),
    ("RECOVERY_REFUSED", {"why": (1 << 2) | (1 << 14)}, "MOTOR_MAPPING_CONFIRMED, IMU_FORWARD_SIGN"),
    ("IMPACT", {}, "while not driving"),
    ("BARCODE", {"code": 20, "rev": 0}, "K (read forwards)"),
    ("BARCODE", {"code": 20, "rev": 1}, "K (read backwards)"),
    ("BARCODE_UNKNOWN", {"code": 7, "rev": 0}, "7 (read forwards)"),
    ("BARCODE_FAULT", {}, "3 ambiguous barcodes: going straight (C)"),
    ("BARCODE_FAILED", {}, "window did not decode"),
    ("LINE_FOUND", {}, "search"),
    ("LINE_LOST", {}, "search found nothing"),
    ("LINE_SEARCH", {"right": 0}, "left first"),
    ("LINE_SEARCH", {"right": 1}, "right first"),
    ("FINE_SCAN", {"first": -60, "count": 3, "cm": [41, 511, 40]}, "-60deg+2: 41 -1 40"),
    ("SIDE_FOLLOW", {"n60": 12, "seen": 9, "near": 225, "limit": 330, "leg": 412, "x": 180},
     "60deg pings 12 seen 9 near 22.5cm (<=33.0) leg 412 x 180"),
    ("SIDE_FOLLOW", {"n60": 3, "seen": 0, "near": 8191, "limit": 330, "leg": 40, "x": 0},
     "60deg pings 3 seen 0 near -1.0cm (<=33.0) leg 40 x 0"),
    ("SIDE_LOOK", {"d90": 305, "d60": 8191, "gone": 1}, "pivot look 90deg 30.5 cm, 60deg -1.0 cm: side gone"),
    ("SIDE_LOOK", {"d90": 305, "d60": 400, "gone": 0}, "pivot look 90deg 30.5 cm, 60deg 40.0 cm: side still there"),
    ("BYPASS_EXTEND", {"variant": 0, "cm": 25}, "obstacle still beside the car (25 cm): 100 mm further"),
    ("BYPASS_EXTEND", {"variant": 1, "cm": 511}, "return blocked (-1 cm): back to the turn, 100 mm further"),
    ("BYPASS_SIDESTEP", {"mm": 60}, "too close beside the obstacle: back, then 60 mm further out"),
    ("BYPASS_RETURN", {}, "line not seen crossing the original path: back to it, then search"),
    ("OBSTACLE_PROFILE", {"r": 22, "w": 30, "off": -4, "clr_l": 12, "clr_r": -3, "side": 1},
     "r 22cm w 30cm off -4 clrL 12 clrR -3 side L"),
    ("OBSTACLE_PROFILE", {"r": 511, "w": 0, "off": 0, "clr_l": 0, "clr_r": 0, "side": 0},
     "r -1cm w 0cm off 0 clrL 0 clrR 0 side -"),
    ("AVOIDANCE_ABORT", {"reason": 10}, "too many attempts"),
    ("OBST_REVERSE_CAP", {"variant": 0, "n": 90, "used": 300, "cap": 350, "total": 700},
     "standoff 90 mm refused: 300 of 350 mm used, 700 mm total"),
    ("OBST_REVERSE_CAP", {"variant": 1, "n": 0, "used": 300, "cap": 350, "total": 700},
     "rescan skipped: 300 of 350 mm used, 700 mm total"),
    ("OBST_REVERSE_CAP", {"variant": 2, "n": 50, "used": 300, "cap": 350, "total": 700},
     "rescan reverse cut to 50 mm: 300 of 350 mm used, 700 mm total"),
    ("OBSTACLE_BACKOFF", {"need": 75, "right": 0}, "reversing 75 mm: a 45 deg turn would clip the left corner"),
    ("OBSTACLE_BACKOFF", {"need": 75, "right": 1}, "reversing 75 mm: a 45 deg turn would clip the right corner"),
    ("OBSTACLE_RESCAN", {"blocked": 0}, "no room: reversing 100 mm and scanning again"),
    ("OBSTACLE_RESCAN", {"blocked": 1}, "blocked: reversing 100 mm and scanning again"),
    ("OBSTACLE_TIE", {"dl": 200, "dr": 34, "side": 2}, "tie: 90 deg look L 200 cm, R 34 cm -> side R"),
    ("OBSTACLE_CLEARED", {}, "coarse sweep clear: resume line following"),
    ("BYPASS_COMPLETE", {"right": 1, "search": 0}, "passed on the right"),
    ("BYPASS_COMPLETE", {"right": 0, "search": 0}, "passed on the left"),
    ("BYPASS_COMPLETE", {"right": 1, "search": 1}, "passed on the right; on the path, searching for the line"),
    ("BYPASS_COMPLETE", {"right": 0, "search": 1}, "passed on the left; on the path, searching for the line"),
    ("BYPASS_BLOCKED", {"right": 1, "no_end": 0, "cm": 18, "deg": -30},
     "right side: something at 18 cm, -30 deg; backing out"),
    ("BYPASS_BLOCKED", {"right": 0, "no_end": 1, "cm": 0, "deg": 0},
     "left side: obstacle side did not end in 1 m; backing out"),
    ("BYPASS_RETRY", {"right": 1, "wider": 0}, "trying the right side (same scan)"),
    ("BYPASS_RETRY", {"right": 0, "wider": 0}, "trying the left side (same scan)"),
    ("BYPASS_RETRY", {"right": 1, "wider": 1}, "right side again, 100 mm wider"),
    ("BYPASS_RETRY", {"right": 0, "wider": 1}, "left side again, 100 mm wider"),
    ("OBSTACLE", {"cm": 198, "us": 412}, "at 19.8 cm, STOP issued in 412 us"),
    ("BC_DBG", {"variant": 0}, "barcode sampling OFF: IR_WHITE_ADC / IR_BLACK_ADC unset or too close"),
    ("BC_DBG", dict(WINDOW, variant=1), "n=29 decoded K (backwards), thr 1500/900, ovr 2"),
    ("BC_DBG", dict(WINDOW, variant=1, n=5, res=0), "n=5 stray mark (fewer than 9), thr 1500/900, ovr 2"),
    ("BC_DBG", dict(WINDOW, variant=1, n=9, res=0), "n=9 closed, thr 1500/900, ovr 2"),
    ("BC_DBG", dict(WINDOW, variant=1, res=2), "n=29 NOT decoded, thr 1500/900, ovr 2"),
    ("BC_DBG", dict(WINDOW, variant=2, rev=0, level=5, conflict=0),
     "n=29 decoded K (forwards) at adapt, thr 1500/900, ovr 2"),
    ("BC_DBG", dict(WINDOW, variant=2, n=5, res=0, level=0, conflict=0),
     "n=5 stray mark (fewer than 9), thr 1500/900, ovr 2"),
    ("BC_DBG", dict(WINDOW, variant=2, res=2, level=0, conflict=1),
     "n=29 NOT decoded: levels disagree, thr 1500/900, ovr 2"),
    ("BC_DBG", dict(WINDOW, variant=2, res=2, level=0, conflict=0), "n=29 NOT decoded, thr 1500/900, ovr 2"),
    ("BC_DBG", dict(LEVELS, variant=3), "levels 50%:Z29 35%:x27 65%:-3 25%:Z28 75%:x26 adapt:-0"),
    ("BC_DBG", {"variant": 4, "part": 2, "count": 3, "w": [12, 345, 4095]}, "w2 (0.1mm, bar first): 12 345 4095"),
    ("BC_DBG", {"variant": 4, "part": 1, "count": 0, "w": []}, "w1 (0.1mm, bar first):"),
]


def event_frame(name, args, t=1000):
    return codec.encode_fields("event", {"t": t, "code": codec.EVC[name], "args": args})


def round_trip(kind, record):
    payload = codec.encode_fields(kind, record)
    decoded = codec.decode_fields(kind, payload)
    assert decoded == record
    assert codec.encode_fields(kind, decoded) == payload
    assert payload[0] >> 4 == codec.FORMAT_VERSION
    return payload


def layout_bits(layout, upto=None):
    bits = 4
    for field in layout:
        if field[0] == upto:
            break
        count = field[2] if len(field) == 3 else 1
        bits += abs(field[1]) * count
    return bits


# ---------------- tables ----------------

def table_rows(name):
    """The rows of one X-macro table of wire_ids.h, as lists of text fields."""
    m = re.search(rf"#define {name}\(X\) \\\n(.*?)\n\n", WIRE_IDS, re.S)
    assert m, name
    rows = re.findall(r"X\((.*?)\) *(?:\\|/\*|$)", m.group(1), re.M)
    return [[f.strip() for f in re.findall(r'"[^"]*"|[^,\s][^,]*', row)] for row in rows]


def test_format_version_matches_wire_ids():
    assert codec.FORMAT_VERSION == int(re.search(r"WIRE_FORMAT_VERSION\s+\((\d+)u\)", WIRE_IDS).group(1))


def test_event_table_matches_wire_ids():
    rows = table_rows("WIRE_EVENT_TABLE")
    assert len(rows) == len(codec.EVENTS) == 45
    for code, (row, event) in enumerate(zip(rows, codec.EVENTS)):
        name, bits, variable = event
        assert [int(row[0]), row[1], int(row[3])] == [code, name, int(variable)]
        if not variable:
            assert int(row[2].rstrip("u")) == bits


def test_text_tables_match_wire_ids():
    reasons = table_rows("WIRE_REASON_TABLE")
    assert [(r[1], r[2].strip('"')) for r in reasons] == codec.REASONS
    commands = table_rows("WIRE_CMD_TABLE")
    assert [(c[1], c[2].strip('"'), int(c[3].rstrip("u")), int(c[4].rstrip("u"))) for c in commands] == codec.COMMANDS
    assert [r[2].strip('"') for r in table_rows("WIRE_STOP_TABLE")] == codec.STOP_REASONS
    assert [r[2].strip('"') for r in table_rows("WIRE_ABORT_TABLE")] == codec.ABORT_REASONS


def test_param_table_matches_app_params():
    """Names, order, store widths and the table CRC against the C side
    (wire_dump). The limits are not in wire_dump, so they are not checked here."""
    dump = wire_spec.dump()
    assert len(codec.PARAMS) == len(dump["params"]) == 55
    for i, (param, c_param) in enumerate(zip(codec.PARAMS, dump["params"])):
        assert c_param["id"] == i
        assert (param[0], codec.PARAM_KINDS[param[1]]) == (c_param["name"], c_param["store_width"]), i
    assert codec.PARAM_CRC == dump["param_table_crc"]


def test_packed_params_match_d15():
    packed = {i: p[1] for i, p in enumerate(codec.PARAMS) if p[1] != "F32"}
    assert packed == {0: "S2", 1: "F1", 2: "F1", 3: "S2", 4: "S2", 7: "S2", 8: "F1", 10: "B2-1", 11: "S2",
                      33: "F1", 34: "F1", 36: "F1", 39: "B3+1", 41: "F1", 49: "F1", 51: "F1"}


def test_crc_check_value():
    assert codec.crc16_ccitt_false(b"123456789") == 0x29B1


def test_param_table_version_follows_names():
    assert codec.param_table_version() == (55, codec.PARAM_CRC)
    names = [p[0] for p in codec.PARAMS]
    names[0], names[1] = names[1], names[0]
    assert codec.param_table_version(names)[1] != codec.PARAM_CRC


# ---------------- layouts and lengths ----------------

def test_telemetry_offsets_match_d8():
    offsets = {"seq": 4, "t": 19, "state": 43, "spd_l": 46, "duty_l": 76, "enc_l": 98, "dist": 146, "line_l": 166,
               "ir": 170, "thr_valid": 206, "thr_b": 207, "bc": 231, "pitch": 237, "hump_h": 248, "hump_peak": 263,
               "us": 291, "scan": 304, "side": 369, "imu_ev": 371, "turn": 374, "faults": 388, "vbat": 404,
               "rssi": 417, "buf": 425}
    for name, offset in offsets.items():
        assert layout_bits(codec.TELEMETRY, name) == offset, name
    assert layout_bits(codec.TELEMETRY) == codec.TLM_BITS


def test_heartbeat_offsets_match_d9():
    offsets = {"seq": 4, "t": 18, "state": 42, "rssi": 45, "buf": 53, "up": 60, "reconnects": 77, "dropped": 93,
               "sat": 109, "faults": 125, "ptab_count": 141, "ptab_crc": 147, "broker": 163}
    for name, offset in offsets.items():
        assert layout_bits(codec.HEARTBEAT, name) == offset, name
    assert layout_bits(codec.HEARTBEAT) == codec.HB_BITS


@pytest.mark.parametrize("name,event", [(e[0], e) for e in codec.EVENTS if not e[2]])
def test_fixed_event_bits_match_table(name, event):
    assert sum(abs(f[1]) * (f[2] if len(f) == 3 else 1) for f in codec.EVENT_ARGS.get(name, ())) == event[1]


def test_variable_event_lengths():
    assert len(event_frame("BC_DBG", {"variant": 4, "part": 1, "count": 16, "w": [1] * 16})) == codec.EVT_MAX_LEN
    assert len(event_frame("FINE_SCAN", {"first": 0, "count": 1, "cm": [1]})) == codec.evt_len(20)
    assert len(event_frame("BC_DBG", dict(WINDOW, variant=1))) == codec.evt_len(59)
    assert len(event_frame("BC_DBG", dict(WINDOW, variant=2, level=0, conflict=0))) == codec.evt_len(63)
    assert len(event_frame("BC_DBG", dict(LEVELS, variant=3))) == codec.evt_len(96)


def test_config_lengths():
    assert codec.cfg_len(0) == 2
    assert codec.cfg_len(55) == 263
    assert codec.cfg_ack_len(55) == 48


# ---------------- round trips, one per payload kind ----------------

def test_round_trip_telemetry():
    assert len(round_trip("telemetry", TELEMETRY)) == codec.TLM_LEN


def test_round_trip_heartbeat():
    assert len(round_trip("heartbeat", HEARTBEAT)) == codec.HB_LEN


@pytest.mark.parametrize("name,args,text", EVENT_CASES, ids=[f"{c[0]}-{i}" for i, c in enumerate(EVENT_CASES)])
def test_round_trip_event(name, args, text):
    round_trip("event", {"t": 77, "code": codec.EVC[name], "args": args})


ACK_CASES = [
    ("ESTOP", {"pwm_off_us": 812}, 7),
    ("RESUME", {}, 5),
    ("START", {}, 5),
    ("STOP", {}, 5),
    ("BARCODE", {"letter": 31}, 6),
    ("MAP", {"map": MAP_ACTS, "n": 4}, 15),
    ("MAP_CLEAR", {"map": [0] * 26, "n": 0}, 15),
    ("GET_MAP", {"map": MAP_ACTS, "n": 4}, 15),
    ("PING", {"ping": 65535}, 7),
    ("SET", {"param": 63}, 6),
    ("CONFIG", {"applied": 54, "n_rej": 1, "rej": [17]}, codec.cfg_ack_len(1)),
    ("CONFIG", {"applied": 0, "n_rej": 55, "rej": list(range(55))}, 48),
]


@pytest.mark.parametrize("name,args,length", ACK_CASES)
def test_round_trip_ack(name, args, length):
    assert len(round_trip("ack", {"t": 5, "cmd": codec.WCMD[name], "reason": 0, "args": args})) == length


def test_round_trip_ack_unparsed():
    assert len(round_trip("ack", {"t": 5, "cmd": codec.WCMD_UNPARSED, "reason": 12, "args": {}})) == 5


def test_round_trip_ack_get_params():
    bits = codec.f32_to_bits(0.35)
    args = {"part": 1, "count": 4, "values": [[0, 3], [1, 1], [10, 2], [23, bits]]}
    payload = round_trip("ack", {"t": 5, "cmd": codec.WCMD["GET_PARAMS"], "reason": 0, "args": args})
    assert len(payload) == codec.frame_bytes(37 + 7 + (6 + 2) + (6 + 1) + (6 + 2) + (6 + 32))


def test_longest_get_params_part_is_63_bytes():
    values = [[i, 0] for i in (5, 6, 9, 12, 13, 14, 15, 16, 17, 18, 19, 20)]
    args = {"part": 1, "count": 12, "values": values}
    payload = round_trip("ack", {"t": 0, "cmd": codec.WCMD["GET_PARAMS"], "reason": 0, "args": args})
    assert len(payload) == codec.GETP_MAX_LEN


@pytest.mark.parametrize("record,length", [
    ({"code": 0}, 1), ({"code": 1}, 1), ({"code": 2}, 1), ({"code": 3}, 1), ({"code": 6}, 1), ({"code": 7}, 1),
    ({"code": 10}, 1), ({"code": 4, "letter": 25}, 2), ({"code": 8, "ping": 65535}, 3),
    ({"code": 9, "param": 54, "value": 0x7FC00000}, 6),
    (dict(codec.map_body({"fwd": 5, "uturn": 20}), code=5), 4),
])
def test_round_trip_command(record, length):
    assert len(round_trip("command", record)) == length


def test_round_trip_map():
    assert len(round_trip("map", codec.map_body({"fwd": 5, "left": 10, "right": 17, "uturn": 20}))) == codec.MAP_LEN
    assert round_trip("map", codec.map_body({})) == bytes([0x10, 0, 0, 0])


def test_round_trip_config():
    assert len(round_trip("config", {"count": 0, "pairs": []})) == 2
    pairs = [[i, 0x3F800000 + i] for i in range(55)]
    assert len(round_trip("config", {"count": 55, "pairs": pairs})) == 263


@pytest.mark.parametrize("kind,payload", [
    ("telemetry", bytes([0x20]) + bytes(53)),                       # version 2
    ("telemetry", bytes([0x10]) + bytes(52)),                       # 53 bytes
    ("heartbeat", bytes([0x10]) + bytes(25)),                       # 26 bytes
    ("event", bytes([0x10])),                                       # no header
    ("event", codec.encode_fields("event", {"t": 0, "code": 0, "args": {"state": 1}}) + b"\0"),
    ("ack", bytes([0x10, 0, 0, 0])),
    ("command", b""),
    ("command", b"ESTOP"),                                          # baseline text: version 4
    ("command", bytes([0x1B])),                                     # unassigned code 11
    ("command", bytes([0x18, 0])),                                  # PING one byte short
    ("map", bytes([0x10, 0, 0])),
    ("config", bytes([0x10, 0, 0])),                                # 0 pairs, 3 bytes
    ("config", bytes([0x1F, 0xC0])),                                # count 63
])
def test_bad_version_or_length_raises(kind, payload):
    with pytest.raises(codec.CodecError):
        codec.decode_fields(kind, payload)


def test_fine_scan_count_must_be_1_to_11():
    with pytest.raises(codec.CodecError):
        event_frame("FINE_SCAN", {"first": 0, "count": 0, "cm": []})
    with pytest.raises(codec.CodecError):
        event_frame("FINE_SCAN", {"first": 0, "count": 12, "cm": [0] * 12})


def test_field_too_wide_raises():
    with pytest.raises(codec.CodecError, match="seq"):
        codec.encode_fields("telemetry", dict(TELEMETRY, seq=32768))
    with pytest.raises(codec.CodecError, match="spd_l"):
        codec.encode_fields("telemetry", dict(TELEMETRY, spd_l=20000))
    missing = dict(TELEMETRY)
    del missing["seq"]
    with pytest.raises(codec.CodecError, match="seq"):
        codec.encode_fields("telemetry", missing)


# ---------------- hand-built frames: the bit order itself ----------------

def test_command_bytes_by_hand():
    assert codec.encode_command_text("ESTOP") == bytes([0x10])
    assert codec.encode_command_text("RESUME") == bytes([0x11])
    assert codec.encode_command_text("START") == bytes([0x12])
    assert codec.encode_command_text("STOP") == bytes([0x13])
    assert codec.encode_command_text("MAP:CLEAR") == bytes([0x16])
    assert codec.encode_command_text("GET:MAP") == bytes([0x17])
    assert codec.encode_command_text("GET:PARAMS") == bytes([0x1A])
    assert codec.encode_command_text("PING:7") == bytes([0x18, 0x00, 0x07])
    assert codec.encode_command_text("PING") == bytes([0x18, 0x00, 0x00])
    assert codec.encode_command_text("BARCODE:K") == bytes([0x14, 0b01010_000])


def test_set_bytes_by_hand():
    param, value = 23, struct.unpack(">I", struct.pack(">f", 0.35))[0]
    bits = (1 << 44) | (9 << 40) | (param << 34) | (value << 2)
    assert codec.encode_command_text("SET:LINE_STEER=0.35") == bits.to_bytes(6, "big")


def test_map_bytes_by_hand():
    body = 0
    for letter in (5, 10, 17, 20):                                      # F, K, R, U
        body = (body << 6) | 0b100000 | letter
    assert codec.encode_map(MAP_TEXT) == (((1 << 24) | body) << 4).to_bytes(4, "big")
    assert codec.encode_command_text("MAP:" + MAP_TEXT) == bytes([0x15]) + body.to_bytes(3, "big")


def test_heartbeat_bits_by_hand():
    payload = codec.encode_fields("heartbeat", HEARTBEAT)
    value = int.from_bytes(payload, "big")
    assert value & 0b11111 == 0                                         # 5 padding bits
    assert (value >> 5) & 0xFFFFFFFF == 0xC0A80114                      # broker
    assert (value >> 37) & 0xFFFF == 0xBEEF                             # CRC


def test_event_header_by_hand():
    payload = event_frame("OBSTACLE", {"cm": 198, "us": 412}, t=0x123456)
    value = int.from_bytes(payload, "big")
    bits = len(payload) * 8
    assert value >> (bits - 28) == 0x1123456
    assert (value >> (bits - 34)) & 0x3F == 44


# ---------------- presentation ----------------

def test_decode_telemetry_shape():
    m = codec.decode("is02/telemetry", codec.encode_fields("telemetry", TELEMETRY))
    assert set(m) == {"seq", "t", "state", "spd", "duty", "enc", "dist", "line", "bc", "hump", "obst", "imu",
                      "faults", "vbat", "rssi", "buf"}
    assert set(m["line"]) == {"l", "r", "ir", "irmn", "irmx", "thr"}
    assert set(m["hump"]) == {"pitch", "h", "peak", "max"}
    assert set(m["obst"]) == {"cm", "scan", "side"}
    assert set(m["imu"]) == {"ev", "tr"}
    assert m["seq"] == 32767 and m["t"] == 16777215
    assert m["state"] == "LINE_FOLLOWING"
    assert m["spd"] == [-123.4, 100.0]
    assert m["duty"] == [-100.0, 45.5]
    assert m["enc"] == [-8388607, 8388607]
    assert m["dist"] == 100000.0
    assert m["line"] == {"l": -1, "r": 1, "ir": 4095, "irmn": 12, "irmx": 3000, "thr": [2000, 900]}
    assert m["bc"] == "K"
    assert m["hump"] == {"pitch": -90.0, "h": -1500.0, "peak": 1500.0, "max": 123.4}
    assert m["obst"] == {"cm": -1.0, "scan": [-1.0, 431.0, 0.0, 12.3, -1.0], "side": "R"}
    assert m["imu"] == {"ev": "SUDDEN_IMPACT", "tr": -720.0}
    assert (m["faults"], m["vbat"], m["rssi"], m["buf"]) == (0xA005, 6600, -128, 64)


def test_decode_telemetry_sentinels():
    rec = dict(TELEMETRY, thr_valid=0, thr_b=0, thr_w=0, bc=63, side=0)
    m = codec.decode("is02/telemetry", codec.encode_fields("telemetry", rec))
    assert m["line"]["thr"] == [0, 0]
    assert m["bc"] == ""
    assert m["obst"]["side"] == ""


def test_decode_telemetry_keeps_monitor_columns():
    """The JSON keys telemetry_monitor.py reads for its CSV are all there."""
    source = (REPO_DIR / "MQTT" / "telemetry_monitor.py").read_text(encoding="utf-8")
    required = re.search(r"REQUIRED = \{(.*?)\n\}", source, re.S).group(1)
    m = codec.decode("is02/telemetry", codec.encode_fields("telemetry", TELEMETRY))
    assert set(re.findall(r'"(\w+)":', required)) <= set(m)


def test_decode_status_shape():
    m = codec.decode("is02/status", codec.encode_fields("heartbeat", HEARTBEAT))
    assert m == {"seq": 16383, "t": 123456, "state": "IDLE", "rssi": -57, "buf": 3, "up": 131071,
                 "reconnects": 65535, "dropped": 2, "faults": 0x6000, "broker": "192.168.1.20", "sat": 7,
                 "ptab": {"count": 55, "crc": 0xBEEF}}


@pytest.mark.parametrize("name,args,text", EVENT_CASES, ids=[f"{c[0]}-{i}" for i, c in enumerate(EVENT_CASES)])
def test_event_detail_text(name, args, text):
    m = codec.decode("is02/event", event_frame(name, args))
    assert m == {"t": 1000, "event": name, "detail": text, "args": args}


def test_every_event_has_a_text():
    assert {e[0] for e in codec.EVENTS} == set(codec.FIXED_DETAIL) | set(codec.EVENT_TEXT)
    assert {c[0] for c in EVENT_CASES} == {e[0] for e in codec.EVENTS}


@pytest.mark.parametrize("name,args,cmd,result,extras", [
    ("ESTOP", {"pwm_off_us": 812}, "ESTOP", 7, {"pwm_off_us": 812}),
    ("RESUME", {}, "RESUME", 8, {}),
    ("START", {}, "START", 2, {}),
    ("STOP", {}, "STOP", 3, {}),
    ("BARCODE", {"letter": 10}, "BARCODE:K", 1, {}),
    ("MAP", {"map": MAP_ACTS, "n": 4}, "MAP", 0, {"map": MAP_TEXT, "n": 4}),
    ("MAP_CLEAR", {"map": [0] * 26, "n": 0}, "MAP:CLEAR", 5, {"map": "", "n": 0}),
    ("GET_MAP", {"map": MAP_ACTS, "n": 4}, "GET:MAP", 0, {"map": MAP_TEXT, "n": 4}),
    ("PING", {"ping": 7}, "PING:7", 4, {}),
    ("SET", {"param": 23}, "SET:LINE_STEER", 0, {}),
])
def test_decode_ack(name, args, cmd, result, extras):
    payload = codec.encode_fields("ack", {"t": 9, "cmd": codec.WCMD[name], "reason": result, "args": args})
    assert codec.decode("is02/ack", payload) == dict({"t": 9, "cmd": cmd, "result": codec.REASONS[result][1]}, **extras)


def test_decode_ack_get_params():
    values = [[0, 3], [1, 1], [10, 0], [23, codec.f32_to_bits(0.35)], [39, 5], [5, codec.f32_to_bits(-1.0)]]
    args = {"part": 2, "count": 6, "values": values}
    m = codec.decode("is02/ack", codec.encode_fields("ack", {"t": 1, "cmd": 10, "reason": 0, "args": args}))
    assert m == {"t": 1, "cmd": "GET:PARAMS", "result": "ok", "part": 2,
                 "params": {"MOTOR_MAPPING_CONFIRMED": -1.0, "ENC_R_REVERSED": 1.0, "IMU_FORWARD_AXIS": -1.0,
                            "LINE_STEER": 0.35, "BARCODE_LEVELS": 6.0, "SERVO_MIN_US": -1.0}}


def test_decode_ack_config():
    ok = {"t": 1, "cmd": 11, "reason": 0, "args": {"applied": 55, "n_rej": 0, "rej": []}}
    assert codec.decode("is02/ack", codec.encode_fields("ack", ok)) == {
        "t": 1, "cmd": "CONFIG", "result": "ok", "applied": 55, "rejected": 0}
    bad = {"t": 1, "cmd": 11, "reason": 26, "args": {"applied": 53, "n_rej": 2, "rej": [23, 0]}}
    assert codec.decode("is02/ack", codec.encode_fields("ack", bad)) == {
        "t": 1, "cmd": "CONFIG", "result": "LINE_STEER MOTOR_MAPPING_CONFIRMED", "applied": 53, "rejected": 2}


def test_decode_ack_unparsed():
    payload = codec.encode_fields("ack", {"t": 1, "cmd": 15, "reason": 16, "args": {}})
    assert codec.decode("is02/ack", payload) == {"t": 1, "cmd": "", "result": "nack: unknown format version"}


def test_decode_laptop_to_robot_topics():
    assert codec.decode("is02/cmd", codec.encode_command_text("PING:7")) == {"cmd": "PING:7"}
    assert codec.decode("is02/map", codec.encode_map(MAP_TEXT)) == {"map": MAP_TEXT, "n": 4}
    m = codec.decode("is02/config", codec.encode_config(["LINE_STEER=0.5"]))
    assert m == {"count": 1, "params": {"LINE_STEER": 0.5}}


# ---------------- bad payloads ----------------

def logged():
    lines = []
    return lines, lines.append


def test_bad_version_is_logged_and_dropped():
    lines, log = logged()
    assert codec.decode("is02/telemetry", bytes([0x20]) + bytes(53), log=log) is None
    assert len(lines) == 1
    assert "is02/telemetry" in lines[0] and "version 2" in lines[0] and "length 54" in lines[0]


def test_bad_length_is_logged_and_dropped():
    lines, log = logged()
    assert codec.decode("is02/status", bytes([0x10]) + bytes(3), log=log) is None
    assert "is02/status" in lines[0] and "version 1" in lines[0] and "length 4" in lines[0]


def test_default_log_is_the_logger(caplog):
    with caplog.at_level(logging.WARNING, logger="is02_codec"):
        assert codec.decode("is02/event", b"") is None
    assert "is02/event" in caplog.text


def test_empty_retained_payload_is_silent():
    lines, log = logged()
    assert codec.decode("is02/map", b"", log=log) is None
    assert codec.decode("is02/config", b"", log=log) is None
    assert lines == []


def test_unknown_event_code_shows_the_number():
    lines, log = logged()
    payload = (((1 << 24 | 500) << 6 | 45) << 6).to_bytes(5, "big")
    m = codec.decode("is02/event", payload, log=log)
    assert m == {"t": 500, "event": 45, "detail": "", "args": {}}
    assert "45" in lines[0]
    assert codec.encode_fields("event", codec.decode_fields("event", payload)) == payload


def test_unknown_reason_code_shows_the_number():
    lines, log = logged()
    payload = codec.encode_fields("ack", {"t": 1, "cmd": 2, "reason": 28, "args": {}})
    assert codec.decode("is02/ack", payload, log=log) == {"t": 1, "cmd": "START", "result": 28}
    assert "28" in lines[0]


def test_unknown_command_code_shows_the_number():
    lines, log = logged()
    payload = codec.encode_fields("ack", {"t": 1, "cmd": 2, "reason": 12, "args": {}})
    payload = bytes([payload[0], payload[1], payload[2], (payload[3] & 0xF0) | 13, payload[4]])
    assert codec.decode("is02/ack", payload, log=log) == {"t": 1, "cmd": 13, "result": "nack: unknown command"}
    assert "13" in lines[0]
    assert codec.encode_fields("ack", codec.decode_fields("ack", payload)) == payload


@pytest.mark.parametrize("name,args,text", [
    ("EMERGENCY_STOP", {"reason": 12}, "12"),
    ("AVOIDANCE_ABORT", {"reason": 11}, "11"),
    ("NAV_ACTION", {"letter": 0, "action": 6}, "A -> 6"),
    ("BARCODE", {"code": 50, "rev": 0}, "50 (read forwards)"),
    ("MAP", {"map": [7] + [0] * 25}, "A=7"),
])
def test_unknown_code_inside_an_event_shows_the_number(name, args, text):
    lines, log = logged()
    assert codec.decode("is02/event", event_frame(name, args), log=log)["detail"] == text
    assert lines


# ---------------- legacy JSON ----------------

def test_legacy_json_pass_through():
    baseline = {"seq": 3, "t": 100, "state": "IDLE", "spd": [0.0, 0.0]}
    text = json.dumps(baseline).encode()
    assert codec.decode("is02/telemetry", text, legacy_json=True) == baseline


def test_legacy_json_off_rejects_text():
    lines, log = logged()
    assert codec.decode("is02/telemetry", b'{"seq":3}', log=log) is None
    assert "version 7" in lines[0]                                      # '{' = 0x7B


def test_legacy_json_still_decodes_binary():
    m = codec.decode("is02/status", codec.encode_fields("heartbeat", HEARTBEAT), legacy_json=True)
    assert m["broker"] == "192.168.1.20"


def test_legacy_json_invalid_is_logged_and_dropped():
    lines, log = logged()
    assert codec.decode("is02/telemetry", b"{not json", legacy_json=True, log=log) is None
    assert lines


# ---------------- user input ----------------

@pytest.mark.parametrize("text,code", [
    ("ESTOP", 0), ("RESUME", 1), ("START", 2), ("STOP", 3), ("BARCODE:A", 4), ("MAP:F=FWD,K=LEFT,R=RIGHT,U=UTURN", 5),
    ("MAP:CLEAR", 6), ("GET:MAP", 7), ("PING:7", 8), ("SET:LINE_STEER=0.35", 9), ("GET:PARAMS", 10), ("  STOP\n", 3),
])
def test_every_baseline_command_text(text, code):
    assert codec.decode_fields("command", codec.encode_command_text(text))["code"] == code


def test_set_keeps_the_value():
    rec = codec.decode_fields("command", codec.encode_command_text("SET:BARCODE_LEVELS=6"))
    assert rec["param"] == 39
    assert codec.f32_from_bits(rec["value"]) == 6.0


@pytest.mark.parametrize("text,words", [
    ("FLY", ["FLY"]),
    ("PING:Dabc", ["PING:Dabc"]),
    ("PING:65536", ["PING:65536"]),
    ("BARCODE:k", ["BARCODE:k"]),
    ("BARCODE:7", ["BARCODE:7"]),
    ("SET:NOPE=1", ["SET:NOPE", "NOPE"]),
    ("SET:LINE_STEER", ["LINE_STEER"]),
    ("SET:LINE_STEER=abc", ["LINE_STEER", "abc"]),
    ("SET:LINE_STEER=nan", ["LINE_STEER"]),
    ("SET:LINE_STEER=inf", ["LINE_STEER"]),
    ("SET:LINE_STEER=1e-1", ["LINE_STEER"]),
    ("SET:LINE_STEER=1.5", ["LINE_STEER", "out of range"]),
    ("SET:LINE_BASE_MM_S=200", ["LINE_BASE_MM_S", "out of range"]),
    ("SET:BARCODE_LEVELS=2.5", ["BARCODE_LEVELS", "whole"]),
    ("SET:SERVO_MAX_US=1e40", ["SERVO_MAX_US"]),
    ("MAP:", ["empty map"]),
])
def test_bad_command_text_names_the_argument(text, words):
    with pytest.raises(codec.CodecError) as e:
        codec.encode_command_text(text)
    for word in words:
        assert word in str(e.value)


def test_float32_limits_are_inclusive():
    assert codec.encode_command_text("SET:SPEED_KP=0.1")
    assert codec.encode_command_text("SET:SPEED_FF_MM_S_PER_PCT=0.1")
    assert codec.encode_command_text("SET:BARCODE_DET_HYS=0.005")


@pytest.mark.parametrize("pairs", [
    MAP_TEXT,
    "F=FWD;K=LEFT R=RIGHT\nU=UTURN",
    "MAP:" + MAP_TEXT,
    ["F=FWD", "K=LEFT", "R=RIGHT", "U=UTURN"],
    [("F", "FWD"), ("K", "LEFT"), ("R", "RIGHT"), ("U", "UTURN")],
    "f=forward,k=l,r=right,u=u-turn",
    "F=STRAIGHT,K=LEFT,R=R,U=BACK",
    "F,U,K,R",                                                          # positional: FWD,UTURN,LEFT,RIGHT
])
def test_encode_map_forms(pairs):
    assert codec.encode_map(pairs) == codec.encode_map(MAP_TEXT)


def test_encode_map_partial_and_clear():
    rec = codec.decode_fields("map", codec.encode_map("K=LEFT"))
    assert (rec["left_on"], rec["left"], rec["fwd_on"], rec["fwd"]) == (1, 10, 0, 0)
    assert codec.encode_map("CLEAR") == bytes([0x10, 0, 0, 0])


@pytest.mark.parametrize("pairs,word", [
    ("", "empty map"),
    ([], "empty map"),
    ("7=FWD", "letters must be A-Z"),
    ("K=JUMP", "unknown movement"),
    ("K=LEFT,X", "do not mix"),
    ("K,L,M", "exactly 4 letters"),
    ("K,L,M,N,O", "exactly 4 letters"),
    ("KLM", "L=ACTION"),
    ("K,K,M,N", "given twice"),
    ("K=LEFT,K=RIGHT", "given twice"),
    ("K=LEFT,Q=LEFT", "one letter per movement"),
])
def test_encode_map_errors(pairs, word):
    with pytest.raises(codec.CodecError, match=word):
        codec.encode_map(pairs)


def test_r4_error_names_both_letters():
    with pytest.raises(codec.CodecError) as e:
        codec.encode_map("K=LEFT,Q=LEFT")
    assert "Q" in str(e.value) and "K" in str(e.value) and "LEFT" in str(e.value)


def test_encode_config_lines():
    payload = codec.encode_config(["# comment", "", "LINE_STEER=0.35", "  SPEED_KP = 0.02  # tuned", "SET:TURN_SNAP=1"])
    rec = codec.decode_fields("config", payload)
    assert [p[0] for p in rec["pairs"]] == [23, 20, 33]
    assert codec.f32_from_bits(rec["pairs"][2][1]) == 1.0
    assert len(payload) == codec.cfg_len(3)


def test_encode_config_text_and_separators():
    assert codec.encode_config("LINE_STEER=0.35\nSPEED_KP=0.02") == codec.encode_config(["LINE_STEER=0.35;SPEED_KP=0.02"])


def test_encode_config_empty():
    assert codec.encode_config(["# nothing"]) == bytes([0x10, 0x00])


def test_encode_config_reports_every_bad_line():
    lines = ["LINE_STEER=0.35", "", "NOPE=1", "SPEED_KP=abc", "TURN_SNAP", "LINE_STEER=9"]
    with pytest.raises(codec.CodecError) as e:
        codec.encode_config(lines)
    assert [(line, name) for line, name, _ in e.value.errors] == [
        (3, "NOPE"), (4, "SPEED_KP"), (5, "TURN_SNAP"), (6, "LINE_STEER")]
    assert "line 3: NOPE" in str(e.value) and "line 6: LINE_STEER" in str(e.value)


def test_encode_config_too_many_pairs():
    with pytest.raises(codec.CodecError, match="at most 55"):
        codec.encode_config(["LINE_STEER=0.5"] * 56)


# ---------------- jb_fixed ----------------

@pytest.mark.parametrize("value,decimals,text", [
    (0.0, 1, "0.0"),
    (12.34, 1, "12.3"),
    (12.35, 1, "12.4"),                         # float32 12.35 is 12.3500004
    (0.25, 1, "0.3"),                           # exact half: away from zero
    (-0.25, 1, "-0.3"),
    (-0.04, 1, "0.0"),                          # no "-0.0"
    (2.5, 0, "3"),
    (-1.0, 0, "-1"),
    (0.35, 4, "0.3500"),
    (0.0001, 4, "0.0001"),
    (5.0e9, 1, "400000000.0"),                  # capped inside uint32
])
def test_jb_fixed(value, decimals, text):
    assert codec.jb_fixed(value, decimals) == text


def test_jb_fixed_nan():
    assert codec.jb_fixed(math.nan, 1) == "null"


def test_helper_finds_the_repo_codec():
    assert codec.__file__.replace("\\", "/").endswith("MQTT/is02_codec.py")
