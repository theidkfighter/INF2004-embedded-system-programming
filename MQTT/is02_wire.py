"""is02_wire.py - Laptop_Codec (Python) for the IS-02 binary MQTT payloads.

Change Requirement 1 replaced the robot's JSON/text payloads with compact
binary frames (app_program/wire.h, evt.h). This module is the laptop side:

    decode(topic, payload) -> dict in the SAME shape the Baseline_Build JSON
                              had, so the tools keep working unchanged
    encode_cmd(text)       -> Command_Frame bytes from the old text command
                              ("ESTOP", "PING:5", "BARCODE:K", "SET:NAME=1.5",
                              "MAP:F=FWD,K=LEFT", "MAP:CLEAR", "GET:MAP", ...)
    encode_map(text)       -> retained Map_Frame for is02/map
    encode_config(text)    -> retained Config_Frame for is02/config
                              ("NAME=VALUE" lines, # comments)

Bit order: MSB-first; byte 0 bits 7..4 = Format_Version (1). Any payload
whose version nibble is not 1 (for example old JSON text) raises WireError.

The tables below must match app_program/wire_ids.h, app_status.c,
app_params.c and bc_multi.c; tests/host/py/test_laptop_codec.py checks
that they do.
"""

import math
import struct

FORMAT_VERSION = 1

STATE_NAMES = ["INITIALISING", "IDLE", "LINE_FOLLOWING", "MANEUVER", "OBSTACLE_AVOIDANCE",
               "MISSION_COMPLETE", "EMERGENCY_STOP", "ERROR"]
MEV_NAMES = ["STATIONARY", "CRUISING", "ACCELERATING", "DECELERATING", "TURNING",
             "CLIMBING_HUMP", "DESCENDING_HUMP", "SUDDEN_IMPACT"]
ACTION_NAMES = ["NONE", "FWD", "UTURN", "LEFT", "RIGHT"]
BCM_LEVELS = ["50%", "35%", "65%", "25%", "75%", "adapt"]
FAULT_NAMES = ["IMU_I2C", "IMU_DEAD", "ENCODER_LOSS", "MOTION_TIMEOUT", "ULTRASONIC", "LINE_LOST",
               "BYPASS_ABORT", "WATCHDOG", "BATTERY_LOW", "BATTERY_CRIT", "CONFIG", "BARCODE",
               "TELEMETRY", "WIFI_LOSS", "MQTT_LOSS", "US_NO_ECHO"]   # design D1 (16 bits)
CONFIG_ITEMS = ["MOTOR_MAPPING_CONFIRMED", "ENC_DIRECTION_CONFIRMED", "ENC_A_EDGES_PER_REV",
                "WHEEL_DIAMETER_MM", "TRACK_WIDTH_MM", "SPEED_FF_DEADBAND_PCT",
                "SPEED_FF_MM_S_PER_PCT", "LINE_DO_BLACK_HIGH", "IR_WHITE_ADC", "IR_BLACK_ADC",
                "SERVO_CALIBRATED", "IMU_FORWARD_AXIS", "IMU_FORWARD_SIGN"]

# Parameter_ID = position in app_params.c PARAMS[] (55 entries)
PARAM_NAMES = [
    "MOTOR_MAPPING_CONFIRMED", "ENC_R_REVERSED", "ENC_L_REVERSED", "ENC_DIRECTION_CONFIRMED",
    "LINE_DO_BLACK_HIGH", "SERVO_MIN_US", "SERVO_MAX_US", "SERVO_CALIBRATED",
    "SERVO_LOW_ANGLE_IS_RIGHT", "SERVO_TRIM_DEG", "IMU_FORWARD_AXIS", "IMU_FORWARD_SIGN",
    "ROBOT_HALF_WIDTH_MM", "ENC_A_EDGES_PER_REV", "WHEEL_DIAMETER_MM", "TRACK_WIDTH_MM",
    "SPEED_FF_DEADBAND_PCT", "SPEED_FF_MM_S_PER_PCT", "IR_WHITE_ADC", "IR_BLACK_ADC",
    "SPEED_KP", "SPEED_KI", "LINE_BASE_MM_S", "LINE_STEER", "LINE_STEER_RAMP",
    "LINE_HOLD_STEER", "LINE_HOLD_MM", "LINE_LOST_MM", "LINE_CURVE_SLOW",
    "LINE_MIN_WHEEL_MM_S", "LINE_BIAS_KI", "LINE_BIAS_DECAY_MM", "OBST_MAX_TURN_DPS",
    "TURN_SNAP", "BARCODE_AT_JUNCTION", "BARCODE_HYS", "BARCODE_DETECT", "BARCODE_DET_HYS",
    "BARCODE_SENSOR_Y_MM", "BARCODE_LEVELS", "BARCODE_ADAPT_SWING", "BARCODE_DEBUG",
    "TURN_MM_S", "BYPASS_MM_S", "OBSTACLE_STOP_CM", "OBST_MARGIN_MM", "OBST_DEPTH_MM",
    "OBST_LEG_STOP_CM", "OBST_REVERSE_MAX_MM", "OBST_LEGACY", "OBST_MIN_WIDTH_MM",
    "OBST_TRUST_MISS", "OBST_EDGE_PAD_MM", "ROBOT_LEFT_HALF_MM", "IR_TO_AXLE_MM",
]
PARAM_ID = {n: i for i, n in enumerate(PARAM_NAMES)}


# Parameter_Store representation (design D15): packed discrete parameters
# {Parameter_ID: (width, encoding)}; every other ID is F32. GET:PARAMS acks
# carry each value in this representation.
PE_FLAG, PE_TRI, PE_AXIS, PE_LEVELS = 1, 2, 3, 4
PARAM_STORE = {0: (2, PE_TRI), 1: (1, PE_FLAG), 2: (1, PE_FLAG), 3: (2, PE_TRI), 4: (2, PE_TRI),
               7: (2, PE_TRI), 8: (1, PE_FLAG), 10: (2, PE_AXIS), 11: (2, PE_TRI), 33: (1, PE_FLAG),
               34: (1, PE_FLAG), 36: (1, PE_FLAG), 39: (3, PE_LEVELS), 41: (1, PE_FLAG),
               49: (1, PE_FLAG), 51: (1, PE_FLAG)}


def param_store_width(pid):
    return PARAM_STORE[pid][0] if pid in PARAM_STORE else 32


def param_from_store(pid, code):
    if pid not in PARAM_STORE:
        return _f32(code)
    enc = PARAM_STORE[pid][1]
    if enc == PE_TRI:
        return float(code - 4 if code >= 2 else code)
    if enc == PE_AXIS:
        return float(code - 1)
    if enc == PE_LEVELS:
        return float(code + 1)
    return float(code)


def crc16_ccitt_false(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


# Parameter table version (design D15): CRC over every NAME + '\n' in ID order.
# A heartbeat whose param_crc or n_params differs means the laptop's table is stale.
PARAM_TABLE_CRC = crc16_ccitt_false("".join(n + "\n" for n in PARAM_NAMES).encode("ascii"))

# wire_ids.h WIRE_EVENT_TABLE: (name, fixed argument bits or None = variable)
EVENTS = [
    ("STATE", 3), ("IMU_OK", 9), ("FAULT_IMU_I2C", 0), ("FAULT_IMU", 0), ("HUMP_START", 29),
    ("HUMP_PEAK", 29), ("HUMP_END", 29), ("MAP", 78), ("BATTERY_LOW", 0), ("START_REFUSED", 0),
    ("MAP_WARNING", 78), ("MISSION_COMPLETE", 59), ("TURN_SNAP", 1), ("NAV_ACTION", 8),
    ("NAV", 10), ("EMERGENCY_STOP", 4), ("ERROR", 4), ("RECOVERED", 0), ("RECOVERY_REFUSED", 15),
    ("IMPACT", 0), ("BC_DBG", None), ("BARCODE", 7), ("BARCODE_UNKNOWN", 7),
    ("BARCODE_FAULT", 0), ("BARCODE_FAILED", 0), ("LINE_FOUND", 0), ("LINE_LOST", 0),
    ("LINE_SEARCH", 1), ("FINE_SCAN", None), ("SIDE_FOLLOW", 64), ("SIDE_LOOK", 27),
    ("BYPASS_EXTEND", 10), ("BYPASS_SIDESTEP", 10), ("BYPASS_RETURN", 0),
    ("OBSTACLE_PROFILE", 51), ("AVOIDANCE_ABORT", 4), ("OBST_REVERSE_CAP", 44),
    ("OBSTACLE_BACKOFF", 9), ("OBSTACLE_RESCAN", 1), ("OBSTACLE_TIE", 20),
    ("OBSTACLE_CLEARED", 0), ("BYPASS_COMPLETE", 2), ("BYPASS_BLOCKED", 19),
    ("BYPASS_RETRY", 2), ("OBSTACLE", 29),
]
EVENT_CODE = {n: i for i, (n, _b) in enumerate(EVENTS)}

REASONS = [
    "ok", "queued", "starting", "ending the run", "pong", "cleared", "ok (not 4 letters)",
    "motors off (press BTN1 on the robot to recover)",
    "refused: clear the hazard and press BTN1 on the robot", "nack: use BARCODE:<A-Z>",
    "nack: only while LINE_FOLLOWING", "nack: only from IDLE", "nack: unknown command",
    "nack: unknown parameter", "nack: bad number", "nack: out of range",
    "nack: unknown format version", "nack: bad length", "nack: letters must be A-Z",
    "nack: unknown movement (use FWD, LEFT, RIGHT, UTURN)",
    "nack: positional form takes exactly 4 letters", "nack: expected L=ACTION or a single letter",
    "nack: do not mix the two forms",
    "nack: positional form takes exactly 4 letters: FWD,UTURN,LEFT,RIGHT", "nack: empty map",
    "nack: the same letter was given twice", "", "nack: count larger than the parameter table",
]
RSN_CONFIG_REJECTED = 26

# command code -> (echo text, Command_Frame length)
CMDS = [("ESTOP", 1), ("RESUME", 1), ("START", 1), ("STOP", 1), ("BARCODE", 2), ("MAP", 4),
        ("MAP:CLEAR", 1), ("GET:MAP", 1), ("PING", 3), ("SET", 6), ("GET:PARAMS", 1),
        ("CONFIG", 0)]
WCMD = {n: i for i, (n, _l) in enumerate(CMDS)}
WCMD_UNPARSED = 15

STOP_TEXTS = ["", "remote E-stop (MQTT)", "BTN2 stop", "collision (IMU)", "obstacle closer than 8 cm",
              "watchdog: a control task stopped running", "calibration missing",
              "IMU fault (3 failed I2C recoveries)", "battery critical",
              "fault reported (see faults in telemetry)"]
ABORT_TEXTS = ["bypass leg failed (timeout, E-stop or proximity)", "realign failed",
               "reverse onto the original path failed", "reverse limit reached",
               "reverse failed (E-stop, proximity or timeout)",
               "profile failed (no echo in the fine scan)", "no room on either side",
               "no bypass plan", "backing out failed (E-stop, proximity or timeout)",
               "blocked on both sides", "too many attempts"]

TLM_LEN, HB_LEN = 54, 25
CM13_NONE, CM9_NONE = 8191, 511


class WireError(ValueError):
    """A payload that is not a valid frame of the expected type."""


# --------------------------------------------------------------------------
# bit reader / writer (MSB-first)
# --------------------------------------------------------------------------

class BitReader:
    def __init__(self, data):
        self.data = bytes(data)
        self.pos = 0
        self.nbits = len(self.data) * 8

    def u(self, w):
        if self.pos + w > self.nbits:
            raise WireError("payload too short")
        v = 0
        for _ in range(w):
            v = (v << 1) | ((self.data[self.pos >> 3] >> (7 - (self.pos & 7))) & 1)
            self.pos += 1
        return v

    def s(self, w):
        v = self.u(w)
        return v - (1 << w) if v & (1 << (w - 1)) else v

    def rest_zero(self):
        while self.pos < self.nbits:
            if self.u(1):
                return False
        return True


class BitWriter:
    def __init__(self):
        self.bits = []

    def u(self, v, w):
        if v < 0 or v >= (1 << w):
            raise WireError(f"value {v} does not fit U{w}")
        self.bits.extend((v >> (w - 1 - i)) & 1 for i in range(w))
        return self

    def s(self, v, w):
        if v < -(1 << (w - 1)) or v >= (1 << (w - 1)):
            raise WireError(f"value {v} does not fit S{w}")
        return self.u(v & ((1 << w) - 1), w)

    def bytes(self):
        bits = self.bits + [0] * (-len(self.bits) % 8)
        return bytes(int("".join(map(str, bits[i:i + 8])), 2) for i in range(0, len(bits), 8))


def _check_version(data):
    if len(data) == 0:
        raise WireError("empty payload")
    if data[0] >> 4 != FORMAT_VERSION:
        raise WireError(f"unknown format version {data[0] >> 4} (old JSON/text payload?)")


def _f32(bits):
    return struct.unpack(">f", struct.pack(">I", bits))[0]


def _f32_bits(x):
    return struct.unpack(">I", struct.pack(">f", float(x)))[0]


C39_CHARS = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%"
C39_NONE = 63


def _char6(c):
    """C6 (design D10): Code 39 index 0..42, 63 = none (\"\")."""
    return C39_CHARS[c] if c < len(C39_CHARS) else ""


def _cm13(v):
    return -1.0 if v == CM13_NONE else v / 10.0


def _cm9(v):
    return -1 if v == CM9_NONE else v


def _r1(x):
    return round(x, 1)


# --------------------------------------------------------------------------
# robot -> laptop
# --------------------------------------------------------------------------

def decode_telemetry(data):
    _check_version(data)
    if len(data) != TLM_LEN:
        raise WireError(f"Telemetry_Frame must be {TLM_LEN} bytes, got {len(data)}")
    r = BitReader(data)
    r.u(4)
    f = {"seq": r.u(15), "t": r.u(24)}
    f["state"] = STATE_NAMES[r.u(3)]
    spd = [r.s(15) / 10.0, r.s(15) / 10.0]
    duty = [r.s(11) / 10.0, r.s(11) / 10.0]
    enc = [r.s(24), r.s(24)]
    dist = r.u(20) / 10.0
    line_l, line_r = r.s(2), r.s(2)
    ir, irmn, irmx = r.u(12), r.u(12), r.u(12)
    thr_valid, thr_b, thr_w = r.u(1), r.u(12), r.u(12)
    bc = _char6(r.u(6))
    pitch, hump_h = r.s(11) / 10.0, r.s(15) / 10.0
    peak, hmax = r.u(14) / 10.0, r.u(14) / 10.0
    us = _cm13(r.u(13))
    scan = [_cm13(r.u(13)) for _ in range(5)]
    side = ["", "L", "R", "?"][r.u(2)]
    ev = r.u(3)
    tr = r.s(14) / 10.0
    faults, vbat = r.u(16), r.u(13)
    rssi = -r.u(8)
    buf = r.u(7)
    f.update({
        "spd": spd, "duty": duty, "enc": enc, "dist": dist,
        "line": {"l": line_l, "r": line_r, "ir": ir, "irmn": irmn, "irmx": irmx,
                 "thr": [thr_b, thr_w] if thr_valid else [0, 0]},
        "bc": bc,
        "hump": {"pitch": pitch, "h": hump_h, "peak": peak, "max": hmax},
        "obst": {"cm": us, "scan": scan, "side": side},
        "imu": {"ev": MEV_NAMES[ev], "tr": tr},
        "faults": faults, "vbat": vbat, "rssi": rssi, "buf": buf,
    })
    return f


def decode_heartbeat(data):
    _check_version(data)
    if len(data) != HB_LEN:
        raise WireError(f"Heartbeat_Frame must be {HB_LEN} bytes, got {len(data)}")
    r = BitReader(data)
    r.u(4)
    h = {"seq": r.u(14), "t": r.u(24)}
    h["state"] = STATE_NAMES[r.u(3)]
    h["rssi"] = -r.u(8)
    h["buf"] = r.u(7)
    h["up"] = r.u(17)
    h["reconnects"], h["dropped"], h["sat"] = r.u(16), r.u(16), r.u(16)
    h["faults"] = r.u(16)
    h["n_params"], h["param_crc"] = r.u(6), r.u(16)
    h["broker"] = ".".join(str(r.u(8)) for _ in range(4))
    if not r.rest_zero():
        raise WireError("padding bits set")
    return h


def _map26(r):
    return [r.u(3) for _ in range(26)]


def _map_text(act):
    pairs = [f"{chr(65 + i)}={ACTION_NAMES[a] if a < len(ACTION_NAMES) else a}"
             for i, a in enumerate(act) if a]
    return ",".join(pairs)


def _event_args(code, r):
    """Reads the arguments of event code; returns (detail text, extra fields)."""
    name = EVENTS[code][0]
    if name == "STATE":
        return STATE_NAMES[r.u(3)], {}
    if name == "IMU_OK":
        chip, addr = r.u(2), r.u(7)
        return (["none", "LSM303DLHC", "LSM303D"][chip] if chip < 3 else "?") + f" at 0x{addr:02X}", {}
    if name in ("FAULT_IMU_I2C",):
        return "I2C1 timeout: bus recovery", {}
    if name == "FAULT_IMU":
        return "3 failed recoveries: driving without the IMU", {}
    if name in ("HUMP_START", "HUMP_PEAK", "HUMP_END"):
        valid, h, mx = r.u(1), r.u(14) / 10.0, r.u(14) / 10.0
        if not valid:
            return "false slope (no height reported)", {"valid": 0}
        return f"height {h:.1f} mm, run max {mx:.1f} mm", {"h": h, "max": mx}
    if name in ("MAP", "MAP_WARNING"):
        act = _map26(r)
        text = _map_text(act)
        if name == "MAP":
            return text or "cleared", {"map": text}
        return text or "no barcode map yet (send MAP:...)", {"map": text}
    if name == "BATTERY_LOW":
        return "below VBAT_LOW_MV", {}
    if name == "START_REFUSED":
        return "calibration missing (see console)", {}
    if name == "MISSION_COMPLETE":
        t, d, bc, humps, mx, ob = r.u(17), r.u(14) / 100.0, r.u(6), r.u(4), r.u(14) / 10.0, r.u(4)
        return (f"t {t}s dist {d:.2f}m bc {bc} humps {humps} max {mx:.1f}mm obst {ob}",
                {"time_s": t, "dist_m": d, "barcodes": bc, "humps": humps, "hump_max_mm": mx,
                 "obstacles": ob})
    if name == "TURN_SNAP":
        return ("line not seen, encoder angle kept" if r.u(1) else "stopped on the line"), {}
    if name == "NAV_ACTION":
        letter, act = chr(65 + r.u(5)), r.u(3)
        an = ACTION_NAMES[act] if act < len(ACTION_NAMES) else str(act)
        return f"{letter} -> {an}" + (" (not in the MQTT map): straight on" if act == 0 else ""), {}
    if name == "NAV":
        kind, letter, act = r.u(2), chr(65 + r.u(5)), r.u(3)
        an = ACTION_NAMES[act] if act < len(ACTION_NAMES) else str(act)
        return [an, "turn did not complete", f"{letter} stored for the next junction ({an})",
                "barcode fault: going straight (C)"][kind], {}
    if name in ("EMERGENCY_STOP", "ERROR"):
        c = r.u(4)
        return STOP_TEXTS[c] if c < len(STOP_TEXTS) else f"reason {c}", {}
    if name == "RECOVERED":
        return "re-check passed", {}
    if name == "RECOVERY_REFUSED":
        b = r.u(15)
        parts = []
        if b & 1:
            parts.append("obstacle closer than 8 cm.")
        if b & 2:
            parts.append("IMU INT1 still active.")
        missing = [CONFIG_ITEMS[i] for i in range(13) if b & (1 << (i + 2))]
        if missing:
            parts.append(", ".join(missing))
        return " ".join(parts), {}
    if name == "IMPACT":
        return "while not driving", {}
    if name == "BC_DBG":
        return _bc_dbg(r), {}
    if name in ("BARCODE", "BARCODE_UNKNOWN"):
        c, rev = _char6(r.u(6)), r.u(1)
        return f"{c} (read {'backwards' if rev else 'forwards'})", {"code": c}
    if name == "BARCODE_FAULT":
        return "3 ambiguous barcodes: going straight (C)", {}
    if name == "BARCODE_FAILED":
        return "window did not decode", {}
    if name == "LINE_FOUND":
        return "search", {}
    if name == "LINE_LOST":
        return "search found nothing", {}
    if name == "LINE_SEARCH":
        return ("right first" if r.u(1) else "left first"), {}
    if name == "FINE_SCAN":
        first, n = r.s(7), r.u(4)
        if not 1 <= n <= 11:
            raise WireError("FINE_SCAN count outside 1..11")
        return f"{first}deg+2: " + " ".join(str(_cm9(r.u(9))) for _ in range(n)), {}
    if name == "SIDE_FOLLOW":
        n60, seen, near, lim, leg, x = r.u(8), r.u(8), r.u(13) / 10.0, r.u(13) / 10.0, r.u(11), r.u(11)
        return f"60deg pings {n60} seen {seen} near {near:.1f}cm (<={lim:.1f}) leg {leg} x {x}", {}
    if name == "SIDE_LOOK":
        d90, d60, gone = _cm13(r.u(13)), _cm13(r.u(13)), r.u(1)
        return (f"pivot look 90deg {d90:.1f} cm, 60deg {d60:.1f} cm: "
                + ("side gone" if gone else "side still there")), {}
    if name == "BYPASS_EXTEND":
        kind, cm = r.u(1), _cm9(r.u(9))
        if kind == 0:
            return f"obstacle still beside the car ({cm} cm): 100 mm further", {}
        return f"return blocked ({cm} cm): back to the turn, 100 mm further", {}
    if name == "BYPASS_SIDESTEP":
        return f"too close beside the obstacle: back, then {r.u(10)} mm further out", {}
    if name == "BYPASS_RETURN":
        return "line not seen crossing the original path: back to it, then search", {}
    if name == "OBSTACLE_PROFILE":
        rr, w, off, cl, cr, side = _cm9(r.u(9)), r.u(10), r.s(10), r.s(10), r.s(10), r.u(2)
        return f"r {rr}cm w {w}cm off {off} clrL {cl} clrR {cr} side {'-LR?'[side]}", {}
    if name == "AVOIDANCE_ABORT":
        c = r.u(4)
        return ABORT_TEXTS[c] if c < len(ABORT_TEXTS) else f"reason {c}", {}
    if name == "OBST_REVERSE_CAP":
        kind, x, used, cap, tot = r.u(2), r.u(10), r.u(10), r.u(10), r.u(12)
        head = [f"standoff {x} mm refused: ", "rescan skipped: ",
                f"rescan reverse cut to {x} mm: ", ""][kind]
        return f"{head}{used} of {cap} mm used, {tot} mm total", {}
    if name == "OBSTACLE_BACKOFF":
        mm, right = r.u(8), r.u(1)
        return f"reversing {mm} mm: a 45 deg turn would clip the {'right' if right else 'left'} corner", {}
    if name == "OBSTACLE_RESCAN":
        return ("blocked" if r.u(1) else "no room") + ": reversing 100 mm and scanning again", {}
    if name == "OBSTACLE_TIE":
        dl, dr, side = r.u(9), r.u(9), r.u(2)
        return f"tie: 90 deg look L {dl} cm, R {dr} cm -> side {'-LR?'[side]}", {}
    if name == "OBSTACLE_CLEARED":
        return "coarse sweep clear: resume line following", {}
    if name == "BYPASS_COMPLETE":
        right, search = r.u(1), r.u(1)
        return (f"passed on the {'right' if right else 'left'}"
                + ("; on the path, searching for the line" if search else "")), {}
    if name == "BYPASS_BLOCKED":
        right, long_, cm, deg = r.u(1), r.u(1), _cm9(r.u(9)), r.s(8)
        sd = "right" if right else "left"
        if long_:
            return f"{sd} side: obstacle side did not end in 1 m; backing out", {}
        return f"{sd} side: something at {cm} cm, {deg} deg; backing out", {}
    if name == "BYPASS_RETRY":
        right, wider = r.u(1), r.u(1)
        sd = "right" if right else "left"
        return (f"{sd} side again, 100 mm wider" if wider else f"trying the {sd} side (same scan)"), {}
    if name == "OBSTACLE":
        cm, us = _cm13(r.u(13)), r.u(16)
        return f"at {cm:.1f} cm, STOP issued in {us} us", {}
    return "", {}


def _bc_window(r):
    n, res, code, rev, tb, tw, ovr = r.u(7), r.u(2), _char6(r.u(6)), r.u(1), r.u(12), r.u(12), r.u(16)
    return n, res, code, rev, tb, tw, ovr


def _bc_dbg(r):
    v = r.u(3)
    if v == 0:
        return "barcode sampling OFF: IR_WHITE_ADC / IR_BLACK_ADC unset or too close"
    if v in (1, 2):
        n, res, code, rev, tb, tw, ovr = _bc_window(r)
        text = f"n={n}"
        if v == 2:
            level, conflict = r.u(3), r.u(1)
        if res == 1:
            text += f" decoded {code} ({'backwards' if rev else 'forwards'})"
            if v == 2:
                text += f" at {BCM_LEVELS[level] if level < 6 else level}"
        elif res == 0:
            text += " stray mark (fewer than 9)" if (n < 9 or v == 2) else " closed"
        else:
            text += " NOT decoded: levels disagree" if (v == 2 and conflict) else " NOT decoded"
        return text + f", thr {tb}/{tw}, ovr {ovr}"
    if v == 3:
        k = r.u(3)
        parts = []
        entries = [(r.u(2), _char6(r.u(6)), r.u(7)) for _ in range(6)]    # D10: always 6
        for i in range(min(k, 6)):
            res, code, n = entries[i]
            mark = code if res == 1 else ("-" if res == 0 else "x")
            parts.append(f"{BCM_LEVELS[i] if i < 6 else i}:{mark}{n}")
        return "levels " + " ".join(parts)
    if v == 4:
        part, k = r.u(3), r.u(5)
        return f"w{part} (0.1mm, bar first): " + " ".join(str(r.u(12)) for _ in range(k))
    raise WireError(f"unknown BC_DBG variant {v}")


def decode_event(data):
    _check_version(data)
    r = BitReader(data)
    r.u(4)
    t, code = r.u(24), r.u(6)
    if code >= len(EVENTS):
        raise WireError(f"unknown Event_Code {code}")
    name, fixed = EVENTS[code]
    start = r.pos
    detail, extra = _event_args(code, r)
    used = r.pos - start
    if fixed is not None and used != fixed:
        raise WireError(f"{name}: {used} argument bits read, {fixed} expected")
    if len(data) != (34 + used + 7) // 8:
        raise WireError(f"{name}: wrong length {len(data)}")
    if not r.rest_zero():
        raise WireError("padding bits set")
    out = {"t": t, "event": name, "detail": detail}
    out.update(extra)
    return out


def decode_ack(data):
    _check_version(data)
    r = BitReader(data)
    r.u(4)
    t, cmd, reason = r.u(24), r.u(4), r.u(5)
    if reason >= len(REASONS):
        raise WireError(f"unknown Reason_Code {reason}")
    echo = "" if cmd == WCMD_UNPARSED else (CMDS[cmd][0] if cmd < len(CMDS) else None)
    if echo is None:
        raise WireError(f"unknown command code {cmd}")
    a = {"t": t, "cmd": echo, "result": REASONS[reason]}
    if echo == "ESTOP":
        a["pwm_off_us"] = r.u(16)
    elif echo == "BARCODE":
        a["cmd"] = f"BARCODE:{chr(65 + r.u(5))}"
    elif echo in ("MAP", "MAP:CLEAR", "GET:MAP"):
        act = _map26(r)
        a["map"] = _map_text(act)
        a["n"] = r.u(5)
    elif echo == "PING":
        a["cmd"] = f"PING:{r.u(16)}"
    elif echo == "SET":
        pid = r.u(6)
        a["cmd"] = f"SET:{PARAM_NAMES[pid] if pid < len(PARAM_NAMES) else pid}"
    elif echo == "GET:PARAMS":
        part, count = r.u(3), r.u(4)
        params = {}
        for _ in range(count):
            pid = r.u(6)
            v = param_from_store(pid, r.u(param_store_width(pid)))
            params[PARAM_NAMES[pid] if pid < len(PARAM_NAMES) else str(pid)] = round(v, 4)
        a["part"], a["params"] = part, params
    elif echo == "CONFIG":
        applied, n_rej = r.u(6), r.u(6)
        rej = [r.u(6) for _ in range(n_rej)]
        names = [PARAM_NAMES[i] if i < len(PARAM_NAMES) else str(i) for i in rej]
        a["applied"], a["rejected"] = applied, n_rej
        if reason == RSN_CONFIG_REJECTED:
            a["result"] = " ".join(names)
    if len(data) != (r.pos + 7) // 8 or not r.rest_zero():
        raise WireError("ack length or padding wrong")
    return a


def decode(topic, payload):
    """Decodes a payload of is02/<topic> into the Baseline_Build JSON shape."""
    sub = topic.rsplit("/", 1)[-1]
    if sub == "telemetry":
        return decode_telemetry(payload)
    if sub == "status":
        return decode_heartbeat(payload)
    if sub == "event":
        return decode_event(payload)
    if sub == "ack":
        return decode_ack(payload)
    if sub == "cmd":
        return {"cmd": describe_cmd(payload)}
    if sub == "map":
        return {"map": describe_map(payload)} if payload else {"map": ""}
    if sub == "config":
        return {"config": describe_config(payload)} if payload else {"config": ""}
    raise WireError(f"unknown topic {topic}")


# --------------------------------------------------------------------------
# laptop -> robot
# --------------------------------------------------------------------------

_ACT_WORDS = {"FWD": 1, "FORWARD": 1, "STRAIGHT": 1, "F": 1, "UTURN": 2, "U": 2, "BACK": 2,
              "LEFT": 3, "L": 3, "RIGHT": 4, "R": 4}
# binary map slots (design D13): forward, left, right, U-turn
_SLOT_OF_ACTION = {1: 0, 3: 1, 4: 2, 2: 3}
_POSITIONAL_ACTIONS = (1, 2, 3, 4)               # text "F,U,K,R" = FWD,UTURN,LEFT,RIGHT (baseline)


def _map_body(text):
    """'F=FWD,K=LEFT,...' or 'F,U,K,R' (FWD,UTURN,LEFT,RIGHT) -> 24-bit body writer."""
    items = [x for x in text.replace(";", ",").replace(" ", ",").split(",") if x.strip()]
    slots = [None] * 4
    if items and all("=" not in i for i in items):
        if len(items) != 4:
            raise WireError("positional form takes exactly 4 letters: FWD,UTURN,LEFT,RIGHT")
        for k, i in enumerate(items):
            slots[_SLOT_OF_ACTION[_POSITIONAL_ACTIONS[k]]] = i.strip().upper()
    else:
        for i in items:
            if "=" not in i:
                raise WireError("do not mix the two forms")
            letter, word = (x.strip().upper() for x in i.split("=", 1))
            act = _ACT_WORDS.get(word.replace("-", "").replace("_", ""))
            if act is None:
                raise WireError("unknown movement (use FWD, LEFT, RIGHT, UTURN)")
            k = _SLOT_OF_ACTION[act]
            if slots[k] is not None:
                raise WireError("binary map: one letter per movement (FWD, UTURN, LEFT, RIGHT)")
            slots[k] = letter
    seen = set()
    w = BitWriter()
    for letter in slots:
        if letter is None:
            w.u(0, 1).u(0, 5)
            continue
        if len(letter) != 1 or not "A" <= letter <= "Z":
            raise WireError("letters must be A-Z")
        if letter in seen:
            raise WireError("the same letter was given twice")
        seen.add(letter)
        w.u(1, 1).u(ord(letter) - 65, 5)
    if not seen:
        raise WireError("empty map")
    return w


def encode_cmd(text):
    """Old text command -> Command_Frame bytes."""
    t = text.strip()
    up = t.upper()
    w = BitWriter().u(FORMAT_VERSION, 4)
    if up in ("ESTOP", "RESUME", "START", "STOP", "MAP:CLEAR", "GET:MAP", "GET:PARAMS"):
        return w.u(WCMD[up], 4).bytes()
    if up.startswith("PING"):
        n = int(t[5:]) if len(t) > 5 else 0
        return w.u(WCMD["PING"], 4).u(n & 0xFFFF, 16).bytes()
    if up.startswith("BARCODE:"):
        c = up[8:9]
        if not c or not "A" <= c <= "Z":
            raise WireError("use BARCODE:<A-Z>")
        return w.u(WCMD["BARCODE"], 4).u(ord(c) - 65, 5).u(0, 3).bytes()
    if up.startswith("SET:"):
        name, _, value = t[4:].partition("=")
        name = name.strip().upper()
        if name not in PARAM_ID:
            raise WireError(f"unknown parameter {name}")
        v = float(value)
        return w.u(WCMD["SET"], 4).u(PARAM_ID[name], 6).u(_f32_bits(v), 32).u(0, 2).bytes()
    if up.startswith("MAP:"):
        body = _map_body(t[4:])
        w.u(WCMD["MAP"], 4)
        w.bits.extend(body.bits)
        return w.bytes()
    raise WireError(f"unknown command {text!r}")


def encode_map(text):
    """Retained Map_Frame for is02/map ('F=FWD,K=LEFT,R=RIGHT,U=UTURN')."""
    w = BitWriter().u(FORMAT_VERSION, 4)
    w.bits.extend(_map_body(text).bits)
    return w.bytes()


def encode_config(text):
    """'NAME=VALUE' lines (also ';' or ',' separated, '#' comments) -> Config_Frame.
    Raises WireError naming every unknown parameter or bad number."""
    pairs, bad = [], []
    for raw in text.replace(";", "\n").replace(",", "\n").splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        if line.upper().startswith("SET:"):
            line = line[4:]
        name, _, value = line.partition("=")
        name = name.strip().upper()
        try:
            v = float(value)
            if name not in PARAM_ID or not math.isfinite(v):
                raise ValueError
            pairs.append((PARAM_ID[name], v))
        except ValueError:
            bad.append(line)
    if bad:
        raise WireError("bad config lines: " + "; ".join(bad))
    if len(pairs) > 55:
        raise WireError("more than 55 parameters")
    w = BitWriter().u(FORMAT_VERSION, 4).u(len(pairs), 6)
    for pid, v in pairs:
        w.u(pid, 6).u(_f32_bits(v), 32)
    return w.bytes()


def describe_cmd(data):
    _check_version(data)
    r = BitReader(data)
    r.u(4)
    code = r.u(4)
    if code >= len(CMDS) - 1:
        raise WireError(f"unknown command code {code}")
    name = CMDS[code][0]
    if name == "PING":
        return f"PING:{r.u(16)}"
    if name == "BARCODE":
        return f"BARCODE:{chr(65 + r.u(5))}"
    if name == "SET":
        pid, v = r.u(6), _f32(r.u(32))
        return f"SET:{PARAM_NAMES[pid] if pid < len(PARAM_NAMES) else pid}={v:g}"
    if name == "MAP":
        return "MAP:" + _describe_body(r)
    return name


def _describe_body(r):
    out = []
    for word in ("FWD", "LEFT", "RIGHT", "UTURN"):       # D13 slot order
        present, letter = r.u(1), r.u(5)
        if present:
            out.append(f"{chr(65 + letter)}={word}")
    return ",".join(out)


def describe_map(data):
    _check_version(data)
    r = BitReader(data)
    r.u(4)
    return _describe_body(r)


def describe_config(data):
    _check_version(data)
    r = BitReader(data)
    r.u(4)
    n = r.u(6)
    lines = []
    for _ in range(n):
        pid, v = r.u(6), _f32(r.u(32))
        lines.append(f"{PARAM_NAMES[pid] if pid < len(PARAM_NAMES) else pid}={v:g}")
    return "\n".join(lines)


# --------------------------------------------------------------------------
# helpers for the laptop tools
# --------------------------------------------------------------------------

def to_text(topic, payload):
    """Payload as the Baseline_Build JSON text. A binary frame is decoded; an
    old JSON/text payload (Baseline_Build robot) is passed through unchanged,
    so the tools work with both builds."""
    import json
    try:
        return json.dumps(decode(topic, payload), separators=(",", ":"))
    except WireError:
        return payload.decode("utf-8", errors="replace")


def cmd_payload(text, legacy=False):
    """Bytes to publish on is02/cmd: a Command_Frame, or the text itself for
    a Baseline_Build robot (legacy=True)."""
    return text.encode() if legacy else encode_cmd(text)


def map_payload(text, legacy=False):
    return text.encode() if legacy else encode_map(text)


def config_payload(text, legacy=False):
    return text.encode() if legacy else encode_config(text)
