"""Property 16 helpers (spec task 16.4): the firmware's event builders (evt.c in
build/libis02.so, called through ctypes) and the reference formatter that
prints an event detail text the way the Baseline_Build did, with the C
jb_fixed() for every number.

Standard library only (cross_cases.py runs under any python3); it does not
import conftest.
"""

import ctypes

import wire_spec
from codec_helpers import is02_codec as codec

LIB_PATH = wire_spec.BUILD / "libis02.so"

_u8, _u32 = ctypes.c_uint8, ctypes.c_uint32
_vp = ctypes.c_void_p


class WireEvt(ctypes.Structure):
    _fields_ = [("code", _u8), ("nbits", _u8), ("args", _u8 * 26)]


class JBuf(ctypes.Structure):
    _fields_ = [("p_buf", ctypes.c_char_p), ("size", _u32), ("len", _u32), ("b_overflow", ctypes.c_bool)]


_lib = None


def lib():
    """build/libis02.so with the event builders, the jb_fixed text builder and the os stub bound."""
    global _lib
    if _lib is None:
        _lib = ctypes.CDLL(str(LIB_PATH))
        sig = {
            "evt_stub_reset": (None, _u32), "evt_begin": (None, _vp, _u32), "evt_u": (None, _vp, _u32, _u32),
            "evt_s": (None, _vp, ctypes.c_int32, _u32), "evt_cm13": (None, _vp, ctypes.c_float),
            "evt_cm9": (None, _vp, ctypes.c_float), "evt_map26": (None, _vp, _vp),
            "evt_post": (ctypes.c_bool, _vp),
            "os_stub_get": (_u32, _u32, ctypes.POINTER(_u32), _vp, _u32),
            "wire_enc_event": (_u32, _u32, ctypes.POINTER(WireEvt), _vp, _u32),
            "jb_init": (None, ctypes.POINTER(JBuf), _vp, _u32),
            "jb_fixed": (None, ctypes.POINTER(JBuf), ctypes.c_float, _u32),
        }
        for name, (res, *args) in sig.items():
            fn = getattr(_lib, name)
            fn.restype, fn.argtypes = res, args
    return _lib


def jf(value, decimals):
    """jb_fixed() of the firmware: value (as float32) with the given decimals."""
    text = ctypes.create_string_buffer(48)
    jb = JBuf()
    lib().jb_init(ctypes.byref(jb), text, 48)
    lib().jb_fixed(ctypes.byref(jb), value, decimals)
    return text.value.decode("ascii")


# ---------------- the builders ----------------

CM13_FIELDS = {("SIDE_LOOK", "d90"), ("SIDE_LOOK", "d60")}
CM9_FIELDS = {("FINE_SCAN", "cm"), ("BYPASS_EXTEND", "cm"), ("OBSTACLE_PROFILE", "r"), ("BYPASS_BLOCKED", "cm")}


def build_ops(rec):
    """Builder calls ([op, ...]) that make the event of a record, in physical units where the firmware
    uses them: ("u", v, width), ("s", v, width), ("cm9", cm), ("cm13", cm), ("map26", [26 codes])."""
    import codec_gen as gen
    name = codec.EVENTS[rec["code"]][0]
    ops = []
    for field in gen.event_layout_of(rec):
        fname, width = field[0], field[1]
        vals = rec["args"][fname] if len(field) == 3 else [rec["args"][fname]]
        if name in ("MAP", "MAP_WARNING"):
            ops.append(["map26", list(vals)])
            continue
        for v in vals:
            if (name, fname) in CM13_FIELDS:
                ops.append(["cm13", -1.0 if v == 8191 else v / 10])
            elif (name, fname) in CM9_FIELDS:
                ops.append(["cm9", -1.0 if v == 511 else float(v)])
            elif width < 0:
                ops.append(["s", v, -width])
            else:
                ops.append(["u", v, width])
    return ops


def build_event(code, ops, t_ms):
    """Runs the builders, posts, and returns (posted, bytes of the Event_Frame the codec makes of the
    captured message). posted is False when evt_post() refused the event."""
    c = lib()
    c.evt_stub_reset(t_ms)
    buf = ctypes.create_string_buffer(256)
    c.evt_begin(buf, code)
    for op in ops:
        if op[0] == "u":
            c.evt_u(buf, op[1], op[2])
        elif op[0] == "s":
            c.evt_s(buf, op[1], op[2])
        elif op[0] == "cm13":
            c.evt_cm13(buf, op[1])
        elif op[0] == "cm9":
            c.evt_cm9(buf, op[1])
        else:
            acts = (_u8 * 26)(*op[1])
            c.evt_map26(buf, acts)
    if not c.evt_post(buf):
        return False, b""
    body = ctypes.create_string_buffer(128)
    mbx = _u32()
    n = c.os_stub_get(0, ctypes.byref(mbx), body, 128)
    assert n >= 32, "the event was not captured"
    t = int.from_bytes(body.raw[0:4], "little")
    ev = WireEvt.from_buffer_copy(body.raw[4:4 + ctypes.sizeof(WireEvt)])
    out = ctypes.create_string_buffer(64)
    n = c.wire_enc_event(t, ctypes.byref(ev), out, 64)
    return True, out.raw[:n]


# ---------------- the reference formatter ----------------

def c39(code):
    return codec.C39_CHARS[code] if code < len(codec.C39_CHARS) else str(code)


def cm13(code):
    return -1.0 if code == 8191 else code / 10


def cm9(code):
    return -1.0 if code == 511 else float(code)


def side_word(right):
    return "right" if right else "left"


def bc_tail(a):
    return f", thr {a['thr_b']}/{a['thr_w']}, ovr {a['ovr']}"


def bc_dbg_text(a):
    v = a["variant"]
    if v == 0:
        return "barcode sampling OFF: IR_WHITE_ADC / IR_BLACK_ADC unset or too close"
    if v in (1, 2):
        text = f"n={a['n']}"
        way = "backwards" if a["rev"] else "forwards"
        if a["res"] == 1:
            text += f" decoded {c39(a['code'])} ({way})" + (f" at {codec.BC_LEVELS[a['level']]}" if v == 2 else "")
        elif a["res"] == 0:
            text += " stray mark (fewer than 9)" if (v == 2 or a["n"] < 9) else " closed"
        else:
            text += " NOT decoded" + (": levels disagree" if v == 2 and a["conflict"] else "")
        return text + bc_tail(a)
    if v == 3:
        text = "levels"
        for i in range(a["n_levels"]):
            res = a[f"res{i}"]
            mark = c39(a[f"code{i}"]) if res == 1 else ("-" if res == 0 else "x")
            text += f" {codec.BC_LEVELS[i]}:{mark}{a[f'n{i}']}"
        return text
    return f"w{a['part']} (0.1mm, bar first):" + "".join(f" {w}" for w in a["w"])


def describe_map(acts):
    return ",".join(f"{chr(65 + i)}={codec.ACTIONS[act]}" for i, act in enumerate(acts) if act)


def ref_text(rec):
    """The Baseline_Build detail text of the event the record holds (from its field values)."""
    name, a = codec.EVENTS[rec["code"]][0], rec["args"]
    fixed = codec.FIXED_DETAIL
    if name in fixed:
        return fixed[name]
    if name == "STATE":
        return codec.STATES[a["state"]]
    if name == "IMU_OK":
        return "none" if a["chip"] == 0 else f"{codec.IMU_CHIPS[a['chip']]} at 0x{a['addr']:02X}"
    if name in ("HUMP_START", "HUMP_PEAK", "HUMP_END"):
        if not a["valid"]:
            return "false slope (no height reported)"
        return f"height {jf(a['h'] / 10, 1)} mm, run max {jf(a['max'] / 10, 1)} mm"
    if name in ("MAP", "MAP_WARNING"):
        text = describe_map(a["map"])
        return text or ("cleared" if name == "MAP" else "no barcode map yet (send MAP:...)")
    if name == "MISSION_COMPLETE":
        return (f"t {a['t']}s dist {jf(a['dist'] / 100, 2)}m bc {a['bc']} humps {a['humps']} "
                f"max {jf(a['max'] / 10, 1)}mm obst {a['obst']}")
    if name == "TURN_SNAP":
        return "line not seen, encoder angle kept" if a["not_seen"] else "stopped on the line"
    if name == "NAV_ACTION":
        text = f"{chr(65 + a['letter'])} -> {codec.ACTIONS[a['action']]}"
        return text + (" (not in the MQTT map): straight on" if a["action"] == 0 else "")
    if name == "NAV":
        return ([codec.ACTIONS[a["action"]], "turn did not complete",
                 f"{chr(65 + a['letter'])} stored for the next junction ({codec.ACTIONS[a['action']]})",
                 "barcode fault: going straight (C)"])[a["variant"]]
    if name in ("EMERGENCY_STOP", "ERROR"):
        return codec.STOP_REASONS[a["reason"]]
    if name == "RECOVERY_REFUSED":
        text = ("obstacle closer than 8 cm. " if a["why"] & 1 else "") + ("IMU INT1 still active. " if a["why"] & 2 else "")
        return text + ", ".join(n for i, n in enumerate(codec.CONFIG_ITEMS) if a["why"] >> (2 + i) & 1)
    if name in ("BARCODE", "BARCODE_UNKNOWN"):
        return f"{c39(a['code'])} (read {'backwards' if a['rev'] else 'forwards'})"
    if name == "LINE_SEARCH":
        return "right first" if a["right"] else "left first"
    if name == "FINE_SCAN":
        return f"{jf(float(a['first']), 0)}deg+2:" + "".join(f" {jf(cm9(c), 0)}" for c in a["cm"])
    if name == "SIDE_FOLLOW":
        return (f"60deg pings {a['n60']} seen {a['seen']} near {jf(cm13(a['near']), 1)}cm "
                f"(<={jf(a['limit'] / 10, 1)}) leg {a['leg']} x {a['x']}")
    if name == "SIDE_LOOK":
        return (f"pivot look 90deg {jf(cm13(a['d90']), 1)} cm, 60deg {jf(cm13(a['d60']), 1)}"
                + (" cm: side gone" if a["gone"] else " cm: side still there"))
    if name == "BYPASS_EXTEND":
        cm = jf(cm9(a["cm"]), 0)
        return (f"return blocked ({cm} cm): back to the turn, 100 mm further" if a["variant"]
                else f"obstacle still beside the car ({cm} cm): 100 mm further")
    if name == "BYPASS_SIDESTEP":
        return f"too close beside the obstacle: back, then {a['mm']} mm further out"
    if name == "OBSTACLE_PROFILE":
        return (f"r {jf(cm9(a['r']), 0)}cm w {a['w']}cm off {a['off']} clrL {a['clr_l']} clrR {a['clr_r']} "
                f"side {codec.SIDES[a['side']] or '-'}")
    if name == "AVOIDANCE_ABORT":
        return codec.ABORT_REASONS[a["reason"]]
    if name == "OBST_REVERSE_CAP":
        head = [f"standoff {a['n']} mm refused: ", "rescan skipped: ", f"rescan reverse cut to {a['n']} mm: "][a["variant"]]
        return head + f"{a['used']} of {a['cap']} mm used, {a['total']} mm total"
    if name == "OBSTACLE_BACKOFF":
        return f"reversing {a['need']} mm: a 45 deg turn would clip the {side_word(a['right'])} corner"
    if name == "OBSTACLE_RESCAN":
        return ("blocked" if a["blocked"] else "no room") + ": reversing 100 mm and scanning again"
    if name == "OBSTACLE_TIE":
        return f"tie: 90 deg look L {a['dl']} cm, R {a['dr']} cm -> side {codec.SIDES[a['side']] or '-'}"
    if name == "BYPASS_COMPLETE":
        return (f"passed on the {side_word(a['right'])}"
                + ("; on the path, searching for the line" if a["search"] else ""))
    if name == "BYPASS_BLOCKED":
        if a["no_end"]:
            return f"{side_word(a['right'])} side: obstacle side did not end in 1 m; backing out"
        return (f"{side_word(a['right'])} side: something at {jf(cm9(a['cm']), 0)} cm, "
                f"{jf(float(a['deg']), 0)} deg; backing out")
    if name == "BYPASS_RETRY":
        side = side_word(a["right"])
        return f"{side} side again, 100 mm wider" if a["wider"] else f"trying the {side} side (same scan)"
    if name == "OBSTACLE":
        return f"at {jf(a['cm'] / 10, 1)} cm, STOP issued in {a['us']} us"
    if name == "BC_DBG":
        return bc_dbg_text(a)
    raise AssertionError(f"no reference text for {name}")
