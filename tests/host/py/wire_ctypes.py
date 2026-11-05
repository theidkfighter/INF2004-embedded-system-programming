"""ctypes view of the Wire_Codec (app_program/wire.h) in build/libis02.so.

Structures mirror the C structs member for member; ``check_layout(lib)``
compares their sizes and some offsets with stubs/wire_export.c before a test
relies on them. ``bind_wire(lib)`` sets the C signatures once.

Used by the Wire_Codec example tests (test_wire_basic.py) and by the later
property and unit tests (tasks 3.12, 3.13)."""

import ctypes

from conftest import bind

_u8, _u16, _u32 = ctypes.c_uint8, ctypes.c_uint16, ctypes.c_uint32
_i8, _i16, _i32 = ctypes.c_int8, ctypes.c_int16, ctypes.c_int32
_bool = ctypes.c_bool

# Values from app_program/wire_ids.h and wire.h
WIRE_FORMAT_VERSION = 1
WIRE_TLM_BITS, WIRE_TLM_LEN = 432, 54
WIRE_HB_BITS, WIRE_HB_LEN = 195, 25
WIRE_EVT_HDR_BITS = 34
WIRE_ACK_HDR_BITS = 37
WIRE_EVT_ARG_BYTES = 26
WIRE_GETP_PER_PART = 12
WIRE_CFG_REJ_MAX = 58
WIRE_MAP_LETTERS = 26
WIRE_EVT_MAX_LEN = 30
WIRE_GETP_MAX_LEN = 63

(WIRE_OK, WIRE_BAD_VERSION, WIRE_BAD_CODE, WIRE_BAD_LENGTH, WIRE_BAD_LETTER,
 WIRE_DUP_LETTER, WIRE_BAD_COUNT, WIRE_BAD_VALUE) = range(8)

# Command codes (wire_ids.h WIRE_CMD_TABLE)
(WCMD_ESTOP, WCMD_RESUME, WCMD_START, WCMD_STOP, WCMD_BARCODE, WCMD_MAP,
 WCMD_MAP_CLEAR, WCMD_GET_MAP, WCMD_PING, WCMD_SET, WCMD_GET_PARAMS,
 WCMD_CONFIG) = range(12)
WCMD_UNPARSED = 15

# Event codes used by the examples (wire_ids.h WIRE_EVENT_TABLE)
EVC_STATE, EVC_BC_DBG, EVC_FINE_SCAN, EVC_OBSTACLE = 0, 20, 28, 44


class WireTlm(ctypes.Structure):
    _fields_ = [
        ("seq", _u16), ("t_ms", _u32), ("state", _u8), ("thr_valid", _u8),
        ("spd", _i16 * 2), ("duty", _i16 * 2), ("enc", _i32 * 2), ("dist", _u32),
        ("line_l", _i8), ("line_r", _i8), ("ir", _u16), ("ir_min", _u16),
        ("ir_max", _u16), ("thr_b", _u16), ("thr_w", _u16), ("bc", _u8),
        ("side", _u8), ("pitch", _i16), ("hump_h", _i16), ("hump_peak", _u16),
        ("hump_max", _u16), ("us", _u16), ("scan", _u16 * 5), ("imu_ev", _u8),
        ("rssi", _u8), ("turn", _i16), ("faults", _u16), ("vbat", _u16),
        ("buf", _u8),
    ]


class WireHb(ctypes.Structure):
    _fields_ = [
        ("seq", _u16), ("state", _u8), ("rssi", _u8), ("t_ms", _u32),
        ("uptime_s", _u32), ("reconnects", _u16), ("dropped", _u16),
        ("sat", _u16), ("faults", _u16), ("param_crc", _u16), ("buf", _u8),
        ("n_params", _u8), ("ip", _u8 * 4),
    ]


class WireEvt(ctypes.Structure):
    _fields_ = [("code", _u8), ("nbits", _u8), ("args", _u8 * WIRE_EVT_ARG_BYTES)]


class WirePval(ctypes.Structure):
    _fields_ = [("code", _u32), ("id", _u8), ("w", _u8)]


class _AckMap(ctypes.Structure):
    _fields_ = [("act", _u8 * WIRE_MAP_LETTERS), ("n", _u8)]


class _AckGetp(ctypes.Structure):
    _fields_ = [("part", _u8), ("count", _u8), ("v", WirePval * WIRE_GETP_PER_PART)]


class _AckCfg(ctypes.Structure):
    _fields_ = [("applied", _u8), ("n_rej", _u8), ("rej", _u8 * WIRE_CFG_REJ_MAX)]


class _AckArgs(ctypes.Union):
    _fields_ = [
        ("pwm_off_us", _u16), ("ping", _u16), ("letter", _u8), ("param_id", _u8),
        ("map", _AckMap), ("getp", _AckGetp), ("cfg", _AckCfg),
    ]


class WireAck(ctypes.Structure):
    _fields_ = [("t_ms", _u32), ("cmd", _u8), ("reason", _u8), ("a", _AckArgs)]


class Bw(ctypes.Structure):
    _fields_ = [("p", ctypes.POINTER(_u8)), ("cap_bits", _u32), ("pos", _u32),
                ("b_overflow", _bool)]


class Br(ctypes.Structure):
    _fields_ = [("p", ctypes.POINTER(_u8)), ("len_bits", _u32), ("pos", _u32),
                ("b_short", _bool)]


PBITS_FN = ctypes.CFUNCTYPE(_u32, _u32)

_P = ctypes.POINTER
_buf = ctypes.c_char_p   # const uint8_t * input (bytes)
_out = ctypes.c_void_p   # uint8_t * output buffer

_SIGNATURES = {
    "bw_init": (None, _P(Bw), _out, _u32),
    "bw_put": (None, _P(Bw), _u32, _u32),
    "bw_copy": (None, _P(Bw), _buf, _u32),
    "bw_bytes": (_u32, _P(Bw)),
    "br_init": (None, _P(Br), _buf, _u32),
    "br_get": (_u32, _P(Br), _u32),
    "wire_enc_telemetry": (_u32, _P(WireTlm), _out),
    "wire_dec_telemetry": (ctypes.c_int, _buf, _u32, _P(WireTlm)),
    "wire_enc_heartbeat": (_u32, _P(WireHb), _out),
    "wire_dec_heartbeat": (ctypes.c_int, _buf, _u32, _P(WireHb)),
    "wire_evt_arg_bits": (ctypes.c_int, _u32, _buf, _u32, _P(_u32)),
    "wire_enc_event": (_u32, _u32, _P(WireEvt), _out, _u32),
    "wire_dec_event": (ctypes.c_int, _buf, _u32, _P(_u32), _P(WireEvt)),
    "wire_enc_ack": (_u32, _P(WireAck), _out, _u32),
    "wire_dec_ack": (ctypes.c_int, _buf, _u32, PBITS_FN, _P(WireAck)),
    "wirex_sizeof": (_u32, _u32),
    "wirex_offsetof": (_u32, _u32),
}

_bound = set()


def bind_wire(lib):
    """Bind the Wire_Codec signatures on lib (once) and return lib."""
    if id(lib) not in _bound:
        for name, (restype, *argtypes) in _SIGNATURES.items():
            bind(lib, name, restype, *argtypes)
        _bound.add(id(lib))
    return lib


def check_layout(lib):
    """Raise AssertionError if a ctypes Structure differs from the C layout."""
    sizes = [WireTlm, WireHb, WireEvt, WireAck, WirePval, Bw, Br]
    for kind, cls in enumerate(sizes):
        assert ctypes.sizeof(cls) == lib.wirex_sizeof(kind), cls.__name__
    offsets = [
        WireTlm.buf.offset, WireTlm.scan.offset, WireHb.ip.offset, WireAck.a.offset,
        WireAck.a.offset + _AckArgs.getp.offset + _AckGetp.v.offset,
        WireAck.a.offset + _AckArgs.cfg.offset + _AckCfg.rej.offset,
        WireTlm.turn.offset, WireHb.uptime_s.offset,
    ]
    for which, off in enumerate(offsets):
        assert off == lib.wirex_offsetof(which), f"offset {which}"


def enc_telemetry(lib, tlm):
    out = (ctypes.c_uint8 * WIRE_TLM_LEN)()
    n = lib.wire_enc_telemetry(ctypes.byref(tlm), out)
    return bytes(out[:n])


def dec_telemetry(lib, data):
    t = WireTlm()
    st = lib.wire_dec_telemetry(data, len(data), ctypes.byref(t))
    return st, t


def enc_heartbeat(lib, hb):
    out = (ctypes.c_uint8 * WIRE_HB_LEN)()
    n = lib.wire_enc_heartbeat(ctypes.byref(hb), out)
    return bytes(out[:n])


def dec_heartbeat(lib, data):
    h = WireHb()
    st = lib.wire_dec_heartbeat(data, len(data), ctypes.byref(h))
    return st, h


def make_evt(code, fields):
    """WireEvt for code with the argument fields [(value, width), ...]."""
    bits = "".join(format(v & ((1 << w) - 1), f"0{w}b") for v, w in fields)
    ev = WireEvt()
    ev.code = code
    ev.nbits = len(bits)
    padded = bits + "0" * (-len(bits) % 8)
    for i in range(0, len(padded), 8):
        ev.args[i // 8] = int(padded[i:i + 8], 2)
    return ev


def enc_event(lib, t_ms, ev, cap=64):
    out = (ctypes.c_uint8 * cap)()
    n = lib.wire_enc_event(t_ms, ctypes.byref(ev), out, cap)
    return bytes(out[:n])


def dec_event(lib, data):
    ev = WireEvt()
    t = _u32()
    st = lib.wire_dec_event(data, len(data), ctypes.byref(t), ctypes.byref(ev))
    return st, t.value, ev


def enc_ack(lib, ack, cap=64):
    out = (ctypes.c_uint8 * cap)()
    n = lib.wire_enc_ack(ctypes.byref(ack), out, cap)
    return bytes(out[:n])


def dec_ack(lib, data, pbits=None):
    a = WireAck()
    fn = PBITS_FN(pbits) if pbits is not None else ctypes.cast(None, PBITS_FN)
    st = lib.wire_dec_ack(data, len(data), fn, ctypes.byref(a))
    return st, a


def as_dict(s):
    """A ctypes Structure as nested dicts and lists (unions excluded), for
    equality checks that ignore padding bytes."""
    if isinstance(s, ctypes.Array):
        return [as_dict(x) for x in s]
    if isinstance(s, ctypes.Structure):
        return {name: as_dict(getattr(s, name)) for name, *_ in s._fields_
                if not isinstance(getattr(s, name), ctypes.Union)}
    return s


def bits_of(data):
    """The frame as a '0'/'1' string, bit 0 = MSB of byte 0 (D7)."""
    return "".join(format(b, "08b") for b in data)
