"""
IS-02 Laptop_Codec (Python): the binary MQTT payloads of the robot.

Two layers, the same as MQTT/is02_codec.js:

    decode_fields(kind, payload) -> record     raw field codes; encode_fields
    encode_fields(kind, record)  -> bytes      gives the input bytes back

    decode(topic, payload)       -> dict       the Baseline_Build JSON shape
    encode_command_text(text)    -> bytes      "ESTOP", "PING:7", "SET:LINE_STEER=0.35", ...
    encode_map(pairs)            -> bytes      the Map_Frame
    encode_config(lines)         -> bytes      the Config_Frame

    read_payload(topic, payload) -> (decoded, text)    for the tools: JSON, text or binary
    ptab_mismatch(status)        -> warning or None    from a decoded heartbeat

The kinds are telemetry, heartbeat, event, ack, command, map and config.

The code tables and frame lengths are copies of app_program/wire_ids.h, and
PARAMS is PARAMS[] of app_program/app_params.c in the same order: change them
together. Standard library only.
"""

import json
import logging
import math
import re
import struct

LOG = logging.getLogger("is02_codec")

FORMAT_VERSION = 1                      # WIRE_FORMAT_VERSION


class CodecError(ValueError):
    """A payload or a user input that cannot be decoded or encoded.

    errors lists (line number, NAME, reason) for each bad encode_config line.
    """

    def __init__(self, message, errors=None):
        super().__init__(message)
        self.errors = errors or []


# ---------------- code tables (wire_ids.h) ----------------

# (NAME, largest argument bits, variable length)
EVENTS = [
    ("STATE", 3, False), ("IMU_OK", 9, False), ("FAULT_IMU_I2C", 0, False), ("FAULT_IMU", 0, False),
    ("HUMP_START", 29, False), ("HUMP_PEAK", 29, False), ("HUMP_END", 29, False), ("MAP", 78, False),
    ("BATTERY_LOW", 0, False), ("START_REFUSED", 0, False), ("MAP_WARNING", 78, False),
    ("MISSION_COMPLETE", 59, False), ("TURN_SNAP", 1, False), ("NAV_ACTION", 8, False), ("NAV", 10, False),
    ("EMERGENCY_STOP", 4, False), ("ERROR", 4, False), ("RECOVERED", 0, False), ("RECOVERY_REFUSED", 15, False),
    ("IMPACT", 0, False), ("BC_DBG", 203, True), ("BARCODE", 7, False), ("BARCODE_UNKNOWN", 7, False),
    ("BARCODE_FAULT", 0, False), ("BARCODE_FAILED", 0, False), ("LINE_FOUND", 0, False), ("LINE_LOST", 0, False),
    ("LINE_SEARCH", 1, False), ("FINE_SCAN", 110, True), ("SIDE_FOLLOW", 64, False), ("SIDE_LOOK", 27, False),
    ("BYPASS_EXTEND", 10, False), ("BYPASS_SIDESTEP", 10, False), ("BYPASS_RETURN", 0, False),
    ("OBSTACLE_PROFILE", 51, False), ("AVOIDANCE_ABORT", 4, False), ("OBST_REVERSE_CAP", 44, False),
    ("OBSTACLE_BACKOFF", 9, False), ("OBSTACLE_RESCAN", 1, False), ("OBSTACLE_TIE", 20, False),
    ("OBSTACLE_CLEARED", 0, False), ("BYPASS_COMPLETE", 2, False), ("BYPASS_BLOCKED", 19, False),
    ("BYPASS_RETRY", 2, False), ("OBSTACLE", 29, False),
]
EVC = {e[0]: code for code, e in enumerate(EVENTS)}

# (NAME, text); RSN_CONFIG_REJECTED has no fixed text, it lists the rejected NAMEs
REASONS = [
    ("OK", "ok"),
    ("QUEUED", "queued"),
    ("STARTING", "starting"),
    ("ENDING_RUN", "ending the run"),
    ("PONG", "pong"),
    ("CLEARED", "cleared"),
    ("OK_NOT_4", "ok (not 4 letters)"),
    ("MOTORS_OFF", "motors off (press BTN1 on the robot to recover)"),
    ("RESUME_REFUSED", "refused: clear the hazard and press BTN1 on the robot"),
    ("BAD_LETTER", "nack: use BARCODE:<A-Z>"),
    ("ONLY_LINE_FOLLOW", "nack: only while LINE_FOLLOWING"),
    ("ONLY_FROM_IDLE", "nack: only from IDLE"),
    ("UNKNOWN_COMMAND", "nack: unknown command"),
    ("UNKNOWN_PARAM", "nack: unknown parameter"),
    ("BAD_NUMBER", "nack: bad number"),
    ("OUT_OF_RANGE", "nack: out of range"),
    ("BAD_VERSION", "nack: unknown format version"),
    ("BAD_LENGTH", "nack: bad length"),
    ("MAP_LETTERS", "nack: letters must be A-Z"),
    ("MAP_MOVEMENT", "nack: unknown movement (use FWD, LEFT, RIGHT, UTURN)"),
    ("MAP_POS_COUNT", "nack: positional form takes exactly 4 letters"),
    ("MAP_SYNTAX", "nack: expected L=ACTION or a single letter"),
    ("MAP_MIXED", "nack: do not mix the two forms"),
    ("MAP_POS_4", "nack: positional form takes exactly 4 letters: FWD,UTURN,LEFT,RIGHT"),
    ("MAP_EMPTY", "nack: empty map"),
    ("MAP_DUP_LETTER", "nack: the same letter was given twice"),
    ("CONFIG_REJECTED", ""),
    ("BAD_COUNT", "nack: count larger than the parameter table"),
]
RSN_CONFIG_REJECTED = 26

# (NAME, cmd echo, Command_Frame length, Ack_Frame length); length 0 = not on
# is02/cmd for a command, variable for an ack
COMMANDS = [
    ("ESTOP", "ESTOP", 1, 7), ("RESUME", "RESUME", 1, 5), ("START", "START", 1, 5), ("STOP", "STOP", 1, 5),
    ("BARCODE", "BARCODE", 2, 6), ("MAP", "MAP", 4, 15), ("MAP_CLEAR", "MAP:CLEAR", 1, 15),
    ("GET_MAP", "GET:MAP", 1, 15), ("PING", "PING", 3, 7), ("SET", "SET", 6, 6),
    ("GET_PARAMS", "GET:PARAMS", 1, 0), ("CONFIG", "CONFIG", 0, 0),
]
WCMD = {c[0]: code for code, c in enumerate(COMMANDS)}
WCMD_UNPARSED = 15

STOP_REASONS = [
    "", "remote E-stop (MQTT)", "BTN2 stop", "collision (IMU)", "obstacle closer than 8 cm",
    "watchdog: a control task stopped running", "calibration missing",
    "IMU fault (3 failed I2C recoveries)", "battery critical", "fault reported (see faults in telemetry)",
]

ABORT_REASONS = [
    "bypass leg failed (timeout, E-stop or proximity)", "realign failed",
    "reverse onto the original path failed", "reverse limit reached",
    "reverse failed (E-stop, proximity or timeout)", "profile failed (no echo in the fine scan)",
    "no room on either side", "no bypass plan", "backing out failed (E-stop, proximity or timeout)",
    "blocked on both sides", "too many attempts",
]

STATES = ["INITIALISING", "IDLE", "LINE_FOLLOWING", "MANEUVER", "OBSTACLE_AVOIDANCE",
          "MISSION_COMPLETE", "EMERGENCY_STOP", "ERROR"]
MOTION_EVENTS = ["STATIONARY", "CRUISING", "ACCELERATING", "DECELERATING", "TURNING",
                 "CLIMBING_HUMP", "DESCENDING_HUMP", "SUDDEN_IMPACT"]
ACTIONS = ["NONE", "FWD", "UTURN", "LEFT", "RIGHT"]     # nav_action_t
ACT_NONE = 0
C39_CHARS = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ-. $/+%"
C39_NONE = 63
BC_LEVELS = ["50%", "35%", "65%", "25%", "75%", "adapt"]
IMU_CHIPS = [None, "LSM303DLHC", "LSM303D"]             # hal_imu_chip_t
SIDES = ["", "L", "R"]
BARCODE_MIN_ELEMENTS = 9
# config_check() order: bits 2..14 of RECOVERY_REFUSED
CONFIG_ITEMS = ["MOTOR_MAPPING_CONFIRMED", "ENC_DIRECTION_CONFIRMED", "ENC_A_EDGES_PER_REV", "WHEEL_DIAMETER_MM",
                "TRACK_WIDTH_MM", "SPEED_FF_DEADBAND_PCT", "SPEED_FF_MM_S_PER_PCT", "LINE_DO_BLACK_HIGH",
                "IR_WHITE_ADC", "IR_BLACK_ADC", "SERVO_CALIBRATED", "IMU_FORWARD_AXIS", "IMU_FORWARD_SIGN"]

# ---------------- frame lengths (wire_ids.h) ----------------

TLM_BITS, TLM_LEN = 432, 54
HB_BITS, HB_LEN = 195, 25
MAP_LEN = 4
EVT_HDR_BITS = 34
EVT_MAX_LEN = 30
ACK_HDR_BITS = 37
ACK_UNPARSED_LEN = 5
GETP_PER_PART = 12
GETP_MAX_LEN = 63
FINE_SCAN_MAX = 11
BC_WIDTHS_MAX = 16


def frame_bytes(bits):
    """Length in bytes of a frame of this many bits (WIRE_BYTES)."""
    return (bits + 7) // 8


def evt_len(arg_bits):
    return frame_bytes(EVT_HDR_BITS + arg_bits)


def cfg_len(count):
    return frame_bytes(10 + 38 * count)


def cfg_ack_len(rejected):
    return frame_bytes(ACK_HDR_BITS + 12 + 6 * rejected)


# ---------------- Parameter_IDs (app_params.c, design D15) ----------------

# Parameter_Store representation -> code width in a GET:PARAMS part. F32 stays
# a float; the others are packed in g_params.bits.
PARAM_KINDS = {"F32": 32, "F1": 1, "S2": 2, "B2-1": 2, "B3+1": 3}

# (NAME, representation, min, max), the index is the Parameter_ID
PARAMS = [
    ("MOTOR_MAPPING_CONFIRMED", "S2", -1.0, 1.0),
    ("ENC_R_REVERSED", "F1", 0.0, 1.0),
    ("ENC_L_REVERSED", "F1", 0.0, 1.0),
    ("ENC_DIRECTION_CONFIRMED", "S2", -1.0, 1.0),
    ("LINE_DO_BLACK_HIGH", "S2", -1.0, 1.0),
    ("SERVO_MIN_US", "F32", 300.0, 1500.0),
    ("SERVO_MAX_US", "F32", 1500.0, 2800.0),
    ("SERVO_CALIBRATED", "S2", -1.0, 1.0),
    ("SERVO_LOW_ANGLE_IS_RIGHT", "F1", 0.0, 1.0),
    ("SERVO_TRIM_DEG", "F32", -45.0, 45.0),
    ("IMU_FORWARD_AXIS", "B2-1", -1.0, 2.0),
    ("IMU_FORWARD_SIGN", "S2", -1.0, 1.0),
    ("ROBOT_HALF_WIDTH_MM", "F32", 40.0, 200.0),
    ("ENC_A_EDGES_PER_REV", "F32", 10.0, 20000.0),
    ("WHEEL_DIAMETER_MM", "F32", 20.0, 150.0),
    ("TRACK_WIDTH_MM", "F32", 50.0, 300.0),
    ("SPEED_FF_DEADBAND_PCT", "F32", 0.0, 90.0),
    ("SPEED_FF_MM_S_PER_PCT", "F32", 0.1, 100.0),
    ("IR_WHITE_ADC", "F32", 0.0, 4095.0),
    ("IR_BLACK_ADC", "F32", 0.0, 4095.0),
    ("SPEED_KP", "F32", 0.0, 0.1),
    ("SPEED_KI", "F32", 0.0, 1.0),
    ("LINE_BASE_MM_S", "F32", 30.0, 150.0),         # LINE_BASE_SAFE_MIN_MM_S..MAX
    ("LINE_STEER", "F32", 0.0, 1.0),
    ("LINE_STEER_RAMP", "F32", 0.0, 20.0),
    ("LINE_HOLD_STEER", "F32", 0.0, 1.0),
    ("LINE_HOLD_MM", "F32", 0.0, 300.0),
    ("LINE_LOST_MM", "F32", 50.0, 3000.0),
    ("LINE_CURVE_SLOW", "F32", 0.0, 0.8),
    ("LINE_MIN_WHEEL_MM_S", "F32", 0.0, 80.0),
    ("LINE_BIAS_KI", "F32", 0.0, 10.0),
    ("LINE_BIAS_DECAY_MM", "F32", 0.0, 3000.0),
    ("OBST_MAX_TURN_DPS", "F32", 0.0, 720.0),
    ("TURN_SNAP", "F1", 0.0, 1.0),
    ("BARCODE_AT_JUNCTION", "F1", 0.0, 1.0),
    ("BARCODE_HYS", "F32", 0.01, 0.30),
    ("BARCODE_DETECT", "F1", 0.0, 1.0),
    ("BARCODE_DET_HYS", "F32", 0.005, 0.30),
    ("BARCODE_SENSOR_Y_MM", "F32", -150.0, 150.0),
    ("BARCODE_LEVELS", "B3+1", 1.0, 6.0),
    ("BARCODE_ADAPT_SWING", "F32", 0.05, 0.60),
    ("BARCODE_DEBUG", "F1", 0.0, 1.0),
    ("TURN_MM_S", "F32", 30.0, 250.0),              # SPEED_MAX_MM_S
    ("BYPASS_MM_S", "F32", 30.0, 250.0),
    ("OBSTACLE_STOP_CM", "F32", 10.0, 60.0),
    ("OBST_MARGIN_MM", "F32", 0.0, 300.0),
    ("OBST_DEPTH_MM", "F32", 20.0, 600.0),
    ("OBST_LEG_STOP_CM", "F32", 0.0, 40.0),
    ("OBST_REVERSE_MAX_MM", "F32", 0.0, 1000.0),
    ("OBST_LEGACY", "F1", 0.0, 1.0),
    ("OBST_MIN_WIDTH_MM", "F32", 0.0, 600.0),
    ("OBST_TRUST_MISS", "F1", 0.0, 1.0),
    ("OBST_EDGE_PAD_MM", "F32", 0.0, 100.0),
    ("ROBOT_LEFT_HALF_MM", "F32", 0.0, 250.0),
    ("IR_TO_AXLE_MM", "F32", 0.0, 200.0),
    ("JUNCTION_MIN_MM", "F32", 0.0, 30.0),
    ("JUNCTION_BLANK_MM", "F32", 0.0, 1000.0),
]
PARAM_IDS = {p[0]: i for i, p in enumerate(PARAMS)}
PARAM_NOT_FOUND = 63                    # SET ack of a legacy text with an unknown name


def crc16_ccitt_false(data):
    """CRC-16/CCITT-FALSE: polynomial 0x1021, start 0xFFFF, no reflection, no final XOR."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def param_table_version(names=None):
    """(count, CRC) of the parameter table: the NAMEs in ID order, each followed by a newline."""
    names = [p[0] for p in PARAMS] if names is None else list(names)
    return len(names), crc16_ccitt_false("".join(n + "\n" for n in names).encode("ascii"))


PARAM_COUNT, PARAM_CRC = param_table_version()


# ---------------- bitstream (design D7) ----------------

class BitReader:
    """Most significant bit first; signed fields are two's complement of their width."""

    def __init__(self, data):
        self.bits = len(data) * 8
        self.value = int.from_bytes(data, "big")
        self.pos = 0

    def get(self, width):
        end = self.pos + width
        if end > self.bits:
            raise CodecError(f"payload too short: needs more than {self.bits // 8} bytes")
        self.pos = end
        return (self.value >> (self.bits - end)) & ((1 << width) - 1)

    def get_signed(self, width):
        code = self.get(width)
        return code - (1 << width) if code >> (width - 1) else code


class BitWriter:
    def __init__(self):
        self.value = 0
        self.pos = 0

    def put(self, name, code, width):
        if not isinstance(code, int) or not 0 <= code < (1 << width):
            raise CodecError(f"{name}: {code!r} does not fit {width} bits")
        self.value = (self.value << width) | code
        self.pos += width

    def put_signed(self, name, code, width):
        if not isinstance(code, int) or not -(1 << (width - 1)) <= code < (1 << (width - 1)):
            raise CodecError(f"{name}: {code!r} does not fit a signed {width} bits")
        self.put(name, code & ((1 << width) - 1), width)

    def to_bytes(self):
        pad = -self.pos % 8
        return (self.value << pad).to_bytes((self.pos + pad) // 8, "big")


# ---------------- frame layouts (design D8 to D14) ----------------

# A layout lists (name, width) in stream order. A negative width is a signed
# field. A third item makes the field a list: of that many values, or of as
# many as the earlier field it names.

TELEMETRY = (
    ("seq", 15), ("t", 24), ("state", 3), ("spd_l", -15), ("spd_r", -15), ("duty_l", -11), ("duty_r", -11),
    ("enc_l", -24), ("enc_r", -24), ("dist", 20), ("line_l", -2), ("line_r", -2), ("ir", 12), ("ir_min", 12),
    ("ir_max", 12), ("thr_valid", 1), ("thr_b", 12), ("thr_w", 12), ("bc", 6), ("pitch", -11), ("hump_h", -15),
    ("hump_peak", 14), ("hump_max", 14), ("us", 13), ("scan", 13, 5), ("side", 2), ("imu_ev", 3), ("turn", -14),
    ("faults", 16), ("vbat", 13), ("rssi", 8), ("buf", 7),
)

HEARTBEAT = (
    ("seq", 14), ("t", 24), ("state", 3), ("rssi", 8), ("buf", 7), ("up", 17), ("reconnects", 16),
    ("dropped", 16), ("sat", 16), ("faults", 16), ("ptab_count", 6), ("ptab_crc", 16), ("broker", 8, 4),
)

MAP78 = (("map", 3, 26),)
HUMP_ARGS = (("valid", 1), ("h", 14), ("max", 14))
BARCODE_ARGS = (("code", 6), ("rev", 1))

# Arguments per Event_Code; BC_DBG goes through BC_DBG_ARGS
EVENT_ARGS = {
    "STATE": (("state", 3),),
    "IMU_OK": (("chip", 2), ("addr", 7)),
    "HUMP_START": HUMP_ARGS,
    "HUMP_PEAK": HUMP_ARGS,
    "HUMP_END": HUMP_ARGS,
    "MAP": MAP78,
    "MAP_WARNING": MAP78,
    "MISSION_COMPLETE": (("t", 17), ("dist", 14), ("bc", 6), ("humps", 4), ("max", 14), ("obst", 4)),
    "TURN_SNAP": (("not_seen", 1),),
    "NAV_ACTION": (("letter", 5), ("action", 3)),
    "NAV": (("variant", 2), ("letter", 5), ("action", 3)),
    "EMERGENCY_STOP": (("reason", 4),),
    "ERROR": (("reason", 4),),
    "RECOVERY_REFUSED": (("why", 15),),
    "BARCODE": BARCODE_ARGS,
    "BARCODE_UNKNOWN": BARCODE_ARGS,
    "LINE_SEARCH": (("right", 1),),
    "FINE_SCAN": (("first", -7), ("count", 4), ("cm", 9, "count")),
    "SIDE_FOLLOW": (("n60", 8), ("seen", 8), ("near", 13), ("limit", 13), ("leg", 11), ("x", 11)),
    "SIDE_LOOK": (("d90", 13), ("d60", 13), ("gone", 1)),
    "BYPASS_EXTEND": (("variant", 1), ("cm", 9)),
    "BYPASS_SIDESTEP": (("mm", 10),),
    "OBSTACLE_PROFILE": (("r", 9), ("w", 10), ("off", -10), ("clr_l", -10), ("clr_r", -10), ("side", 2)),
    "AVOIDANCE_ABORT": (("reason", 4),),
    "OBST_REVERSE_CAP": (("variant", 2), ("n", 10), ("used", 10), ("cap", 10), ("total", 12)),
    "OBSTACLE_BACKOFF": (("need", 8), ("right", 1)),
    "OBSTACLE_RESCAN": (("blocked", 1),),
    "OBSTACLE_TIE": (("dl", 9), ("dr", 9), ("side", 2)),
    "BYPASS_COMPLETE": (("right", 1), ("search", 1)),
    "BYPASS_BLOCKED": (("right", 1), ("no_end", 1), ("cm", 9), ("deg", -8)),
    "BYPASS_RETRY": (("right", 1), ("wider", 1)),
    "OBSTACLE": (("cm", 13), ("us", 16)),
}

BC_WINDOW = (("n", 7), ("res", 2), ("code", 6), ("rev", 1), ("thr_b", 12), ("thr_w", 12), ("ovr", 16))
BC_LEVEL_ARGS = tuple((f"{field}{i}", width) for i in range(6)
                      for field, width in (("res", 2), ("code", 6), ("n", 7)))
# variant U3 comes first, then:
BC_DBG_ARGS = {
    0: (),                                                  # off
    1: BC_WINDOW,                                           # window
    2: BC_WINDOW + (("level", 3), ("conflict", 1)),         # pass
    3: (("n_levels", 3),) + BC_LEVEL_ARGS,                  # levels
    4: (("part", 3), ("count", 5), ("w", 12, "count")),     # widths
}
RES_NONE, RES_DECODED = 0, 1                                # RES2; 2 = not decoded

MAP_ACK = MAP78 + (("n", 5),)
# Arguments per command code, except GET:PARAMS
ACK_ARGS = {
    WCMD["ESTOP"]: (("pwm_off_us", 16),),
    WCMD["RESUME"]: (),
    WCMD["START"]: (),
    WCMD["STOP"]: (),
    WCMD["BARCODE"]: (("letter", 5),),
    WCMD["MAP"]: MAP_ACK,
    WCMD["MAP_CLEAR"]: MAP_ACK,
    WCMD["GET_MAP"]: MAP_ACK,
    WCMD["PING"]: (("ping", 16),),
    WCMD["SET"]: (("param", 6),),
    WCMD["CONFIG"]: (("applied", 6), ("n_rej", 6), ("rej", 6, "n_rej")),
    WCMD_UNPARSED: (),
}

# forward, left, right, U-turn: present F1, letter U5
MAP_BODY = (("fwd_on", 1), ("fwd", 5), ("left_on", 1), ("left", 5),
            ("right_on", 1), ("right", 5), ("uturn_on", 1), ("uturn", 5))
MAP_SLOTS = {1: "fwd", 3: "left", 4: "right", 2: "uturn"}   # nav_action_t -> movement
MAP_ORDER = ("fwd", "left", "right", "uturn")

COMMAND_ARGS = {
    WCMD["ESTOP"]: (),
    WCMD["RESUME"]: (),
    WCMD["START"]: (),
    WCMD["STOP"]: (),
    WCMD["BARCODE"]: (("letter", 5),),
    WCMD["MAP"]: MAP_BODY,
    WCMD["MAP_CLEAR"]: (),
    WCMD["GET_MAP"]: (),
    WCMD["PING"]: (("ping", 16),),
    WCMD["SET"]: (("param", 6), ("value", 32)),             # value: F32 bit pattern
    WCMD["GET_PARAMS"]: (),
}


def read_layout(r, layout, rec):
    for field in layout:
        name, width = field[0], field[1]
        get = r.get if width > 0 else r.get_signed
        if len(field) == 2:
            rec[name] = get(abs(width))
        else:
            count = rec[field[2]] if isinstance(field[2], str) else field[2]
            rec[name] = [get(abs(width)) for _ in range(count)]
    return rec


def write_layout(w, layout, rec):
    for field in layout:
        name, width = field[0], field[1]
        put = w.put if width > 0 else w.put_signed
        if len(field) == 2:
            put(name, rec[name], abs(width))
            continue
        count = rec[field[2]] if isinstance(field[2], str) else field[2]
        if len(rec[name]) != count:
            raise CodecError(f"{name}: {len(rec[name])} values, expected {count}")
        for code in rec[name]:
            put(name, code, abs(width))


# ---------------- field layer ----------------

def check_end(kind, r, data):
    """The frame length must be exactly what its fields imply."""
    want = frame_bytes(r.pos)
    if len(data) != want:
        raise CodecError(f"{kind}: {len(data)} bytes, expected {want}")


def check_event_counts(code, args):
    """The two array counts that set a frame's length."""
    if code == EVC["FINE_SCAN"] and not 1 <= args["count"] <= FINE_SCAN_MAX:
        raise CodecError(f"FINE_SCAN: count {args['count']} outside 1..{FINE_SCAN_MAX}")
    if code == EVC["BC_DBG"] and args["variant"] == 4 and args["count"] > BC_WIDTHS_MAX:
        raise CodecError(f"BC_DBG: count {args['count']} above {BC_WIDTHS_MAX}")


def bc_dbg_layout(variant):
    if variant not in BC_DBG_ARGS:
        raise CodecError(f"BC_DBG: unknown variant {variant}")
    return BC_DBG_ARGS[variant]


def event_layout(code):
    return EVENT_ARGS.get(EVENTS[code][0], ())


def decode_event(data):
    r = BitReader(data)
    r.get(4)
    rec = {"t": r.get(24), "code": r.get(6)}
    if rec["code"] >= len(EVENTS):
        rec["raw"] = data.hex()         # unknown arguments: kept so the bytes survive
        return rec
    args = {}
    if rec["code"] == EVC["BC_DBG"]:
        args["variant"] = r.get(3)
        read_layout(r, bc_dbg_layout(args["variant"]), args)
    else:
        read_layout(r, event_layout(rec["code"]), args)
    check_event_counts(rec["code"], args)
    check_end("event", r, data)
    rec["args"] = args
    return rec


def encode_event(rec):
    if "raw" in rec:
        return bytes.fromhex(rec["raw"])
    code = rec["code"]
    if not 0 <= code < len(EVENTS):
        raise CodecError(f"unknown Event_Code {code}")
    args = rec.get("args", {})
    w = BitWriter()
    w.put("version", FORMAT_VERSION, 4)
    w.put("t", rec["t"], 24)
    w.put("code", code, 6)
    if code == EVC["BC_DBG"]:
        w.put("variant", args["variant"], 3)
        write_layout(w, bc_dbg_layout(args["variant"]), args)
    else:
        write_layout(w, event_layout(code), args)
    check_event_counts(code, args)
    return w.to_bytes()


def getp_code_width(param_id):
    if param_id >= len(PARAMS):
        raise CodecError(f"GET:PARAMS: unknown Parameter_ID {param_id}")
    return PARAM_KINDS[PARAMS[param_id][1]]


def decode_ack(data):
    r = BitReader(data)
    r.get(4)
    rec = {"t": r.get(24), "cmd": r.get(4), "reason": r.get(5)}
    args = {}
    if rec["cmd"] == WCMD["GET_PARAMS"]:
        args["part"] = r.get(3)
        args["count"] = r.get(4)
        if args["count"] > GETP_PER_PART:
            raise CodecError(f"GET:PARAMS: count {args['count']} above {GETP_PER_PART}")
        args["values"] = []
        for _ in range(args["count"]):
            param_id = r.get(6)
            args["values"].append([param_id, r.get(getp_code_width(param_id))])
    elif rec["cmd"] in ACK_ARGS:
        read_layout(r, ACK_ARGS[rec["cmd"]], args)
    else:
        rec["raw"] = data.hex()         # unknown arguments: kept so the bytes survive
        return rec
    check_end("ack", r, data)
    rec["args"] = args
    return rec


def encode_ack(rec):
    if "raw" in rec:
        return bytes.fromhex(rec["raw"])
    args = rec.get("args", {})
    w = BitWriter()
    w.put("version", FORMAT_VERSION, 4)
    w.put("t", rec["t"], 24)
    w.put("cmd", rec["cmd"], 4)
    w.put("reason", rec["reason"], 5)
    if rec["cmd"] == WCMD["GET_PARAMS"]:
        w.put("part", args["part"], 3)
        w.put("count", args["count"], 4)
        if len(args["values"]) != args["count"]:
            raise CodecError(f"GET:PARAMS: {len(args['values'])} values, count {args['count']}")
        for param_id, code in args["values"]:
            w.put("param", param_id, 6)
            w.put("value", code, getp_code_width(param_id))
    elif rec["cmd"] in ACK_ARGS:
        write_layout(w, ACK_ARGS[rec["cmd"]], args)
    else:
        raise CodecError(f"unknown command code {rec['cmd']}")
    return w.to_bytes()


def decode_command(data):
    r = BitReader(data)
    r.get(4)
    rec = {"code": r.get(4)}
    if rec["code"] not in COMMAND_ARGS:
        raise CodecError(f"unassigned command code {rec['code']}")
    read_layout(r, COMMAND_ARGS[rec["code"]], rec)
    check_end("command", r, data)
    return rec


def encode_command(rec):
    if rec["code"] not in COMMAND_ARGS:
        raise CodecError(f"unassigned command code {rec['code']}")
    w = BitWriter()
    w.put("version", FORMAT_VERSION, 4)
    w.put("code", rec["code"], 4)
    write_layout(w, COMMAND_ARGS[rec["code"]], rec)
    return w.to_bytes()


def decode_config(data):
    r = BitReader(data)
    r.get(4)
    count = r.get(6)
    if count > PARAM_COUNT:
        raise CodecError(f"config: count {count} above {PARAM_COUNT}")
    rec = {"count": count, "pairs": []}
    for _ in range(count):
        param_id = r.get(6)
        rec["pairs"].append([param_id, r.get(32)])  # value: F32 bit pattern
    check_end("config", r, data)
    return rec


def encode_config_record(rec):
    if not 0 <= rec["count"] <= PARAM_COUNT or len(rec["pairs"]) != rec["count"]:
        raise CodecError(f"config: count {rec['count']}, {len(rec['pairs'])} pairs, at most {PARAM_COUNT}")
    w = BitWriter()
    w.put("version", FORMAT_VERSION, 4)
    w.put("count", rec["count"], 6)
    for param_id, code in rec["pairs"]:
        w.put("param", param_id, 6)
        w.put("value", code, 32)
    return w.to_bytes()


def decode_fixed(kind, layout, data):
    r = BitReader(data)
    r.get(4)
    rec = read_layout(r, layout, {})
    check_end(kind, r, data)
    return rec


def encode_fixed(layout, rec):
    w = BitWriter()
    w.put("version", FORMAT_VERSION, 4)
    write_layout(w, layout, rec)
    return w.to_bytes()


DECODERS = {
    "telemetry": lambda data: decode_fixed("telemetry", TELEMETRY, data),
    "heartbeat": lambda data: decode_fixed("heartbeat", HEARTBEAT, data),
    "event": decode_event,
    "ack": decode_ack,
    "command": decode_command,
    "map": lambda data: decode_fixed("map", MAP_BODY, data),
    "config": decode_config,
}

ENCODERS = {
    "telemetry": lambda rec: encode_fixed(TELEMETRY, rec),
    "heartbeat": lambda rec: encode_fixed(HEARTBEAT, rec),
    "event": encode_event,
    "ack": encode_ack,
    "command": encode_command,
    "map": lambda rec: encode_fixed(MAP_BODY, rec),
    "config": encode_config_record,
}


def decode_fields(kind, payload):
    """Payload bytes -> record of raw field codes. Raises CodecError on a bad version or length."""
    if kind not in DECODERS:
        raise CodecError(f"unknown payload kind {kind!r}")
    data = bytes(payload)
    if not data:
        raise CodecError(f"{kind}: empty payload")
    if data[0] >> 4 != FORMAT_VERSION:
        raise CodecError(f"{kind}: unknown Format_Version {data[0] >> 4}")
    return DECODERS[kind](data)


def encode_fields(kind, record):
    """Record of raw field codes -> payload bytes."""
    if kind not in ENCODERS:
        raise CodecError(f"unknown payload kind {kind!r}")
    try:
        return ENCODERS[kind](record)
    except KeyError as e:
        raise CodecError(f"{kind}: record has no field {e}") from None


# ---------------- presentation helpers ----------------

def f32(value):
    """value as the nearest float32 (a float32 store on the robot)."""
    return struct.unpack(">f", struct.pack(">f", value))[0]


def f32_from_bits(bits):
    return struct.unpack(">f", bits.to_bytes(4, "big"))[0]


def f32_to_bits(value):
    return int.from_bytes(struct.pack(">f", value), "big")


def jb_fixed(value, decimals):
    """The text jb_fixed() prints for a float32 value, computed in float32 as jfmt.c does."""
    value = f32(value)
    if math.isnan(value):
        return "null"
    scale = 10 ** decimals
    mag = min(abs(value), f32(4.0e9 / scale))
    total = int(f32(f32(mag * scale) + 0.5))
    sign = "-" if value < 0.0 and total > 0 else ""
    return sign + fixed_text(total, decimals)


def fixed_text(code, decimals):
    """A fixed-point code as decimal text, without going through a float."""
    sign = "-" if code < 0 else ""
    mag = abs(code)
    if decimals == 0:
        return f"{sign}{mag}"
    scale = 10 ** decimals
    return f"{sign}{mag // scale}.{mag % scale:0{decimals}d}"


def cm9_text(code):
    return "-1" if code == 511 else str(code)


def cm13_text(code):
    return "-1.0" if code == 8191 else fixed_text(code, 1)


def cm13_value(code):
    return -1.0 if code == 8191 else code / 10


def lookup(table, code, what, emit):
    """table[code], or the number itself (logged) when the table has no such code."""
    if 0 <= code < len(table) and table[code] is not None:
        return table[code]
    emit(f"unknown {what} {code}")
    return code


def letter_text(code, emit):
    return chr(ord("A") + code) if code < 26 else str(lookup((), code, "letter", emit))


def bc_char(code, emit):
    """Code 39 index -> character; the number for an index no table lists."""
    return lookup(C39_CHARS, code, "barcode index", emit)


def describe_map(acts, emit):
    """The text of nav_map_describe(): 26 action codes, letter A first."""
    return ",".join(f"{chr(ord('A') + i)}={lookup(ACTIONS, act, 'action', emit)}"
                    for i, act in enumerate(acts) if act != ACT_NONE)


# ---------------- event detail texts (design D10) ----------------

FIXED_DETAIL = {
    "FAULT_IMU_I2C": "I2C1 timeout: bus recovery",
    "FAULT_IMU": "3 failed recoveries: driving without the IMU",
    "BATTERY_LOW": "below VBAT_LOW_MV",
    "START_REFUSED": "calibration missing (see console)",
    "RECOVERED": "re-check passed",
    "IMPACT": "while not driving",
    "BARCODE_FAULT": "3 ambiguous barcodes: going straight (C)",
    "BARCODE_FAILED": "window did not decode",
    "LINE_FOUND": "search",
    "LINE_LOST": "search found nothing",
    "BYPASS_RETURN": "line not seen crossing the original path: back to it, then search",
    "OBSTACLE_CLEARED": "coarse sweep clear: resume line following",
}


def side_word(right):
    return "right" if right else "left"


def side_char(code, emit):
    """Side letter, or "-" for no side, as the obstacle events print it."""
    return lookup(SIDES, code, "side", emit) or "-"


def text_imu_ok(a, emit):
    if a["chip"] == 0:
        return "none"
    return f"{lookup(IMU_CHIPS, a['chip'], 'IMU chip', emit)} at 0x{a['addr']:02X}"


def text_hump(a, emit):
    return f"height {fixed_text(a['h'], 1)} mm, run max {fixed_text(a['max'], 1)} mm"


def text_hump_end(a, emit):
    return text_hump(a, emit) if a["valid"] else "false slope (no height reported)"


def text_mission(a, emit):
    return (f"t {a['t']}s dist {fixed_text(a['dist'], 2)}m bc {a['bc']} humps {a['humps']} "
            f"max {fixed_text(a['max'], 1)}mm obst {a['obst']}")


def text_nav_action(a, emit):
    text = f"{letter_text(a['letter'], emit)} -> {lookup(ACTIONS, a['action'], 'action', emit)}"
    return text + (" (not in the MQTT map): straight on" if a["action"] == ACT_NONE else "")


def text_nav(a, emit):
    action = lookup(ACTIONS, a["action"], "action", emit)
    return [str(action),
            "turn did not complete",
            f"{letter_text(a['letter'], emit)} stored for the next junction ({action})",
            "barcode fault: going straight (C)"][a["variant"]]


def text_refused(a, emit):
    text = "obstacle closer than 8 cm. " if a["why"] & 1 else ""
    text += "IMU INT1 still active. " if a["why"] & 2 else ""
    return text + ", ".join(name for i, name in enumerate(CONFIG_ITEMS) if (a["why"] >> (i + 2)) & 1)


def bc_outcome(a, emit):
    """The " decoded K (forwards)" part shared by the window and pass texts."""
    return f" decoded {bc_char(a['code'], emit)}" + (" (backwards)" if a["rev"] else " (forwards)")


def bc_tail(a):
    return f", thr {a['thr_b']}/{a['thr_w']}, ovr {a['ovr']}"


def text_bc_window(a, emit):
    if a["res"] == RES_DECODED:
        outcome = bc_outcome(a, emit)
    elif a["res"] == RES_NONE:
        outcome = " stray mark (fewer than 9)" if a["n"] < BARCODE_MIN_ELEMENTS else " closed"
    else:
        outcome = " NOT decoded"
    return f"n={a['n']}{outcome}{bc_tail(a)}"


def text_bc_pass(a, emit):
    if a["res"] == RES_DECODED:
        outcome = f"{bc_outcome(a, emit)} at {lookup(BC_LEVELS, a['level'], 'barcode level', emit)}"
    elif a["res"] == RES_NONE:
        outcome = " stray mark (fewer than 9)"
    else:
        outcome = " NOT decoded: levels disagree" if a["conflict"] else " NOT decoded"
    return f"n={a['n']}{outcome}{bc_tail(a)}"


def text_bc_levels(a, emit):
    parts = ["levels"]
    for i in range(min(a["n_levels"], len(BC_LEVELS))):
        if a[f"res{i}"] == RES_DECODED:
            mark = bc_char(a[f"code{i}"], emit)
        else:
            mark = "-" if a[f"res{i}"] == RES_NONE else "x"
        parts.append(f"{BC_LEVELS[i]}:{mark}{a[f'n{i}']}")
    return " ".join(parts)


def text_bc_widths(a, emit):
    return f"w{a['part']} (0.1mm, bar first):" + "".join(f" {w}" for w in a["w"])


def text_bc_dbg(a, emit):
    if a["variant"] == 0:
        return "barcode sampling OFF: IR_WHITE_ADC / IR_BLACK_ADC unset or too close"
    return {1: text_bc_window, 2: text_bc_pass, 3: text_bc_levels, 4: text_bc_widths}[a["variant"]](a, emit)


def text_barcode(a, emit):
    return f"{bc_char(a['code'], emit)} (read {'backwards' if a['rev'] else 'forwards'})"


def text_side_follow(a, emit):
    return (f"60deg pings {a['n60']} seen {a['seen']} near {cm13_text(a['near'])}cm "
            f"(<={fixed_text(a['limit'], 1)}) leg {a['leg']} x {a['x']}")


def text_side_look(a, emit):
    return (f"pivot look 90deg {cm13_text(a['d90'])} cm, 60deg {cm13_text(a['d60'])} cm: "
            f"side {'gone' if a['gone'] else 'still there'}")


def text_extend(a, emit):
    if a["variant"]:
        return f"return blocked ({cm9_text(a['cm'])} cm): back to the turn, 100 mm further"
    return f"obstacle still beside the car ({cm9_text(a['cm'])} cm): 100 mm further"


def text_profile(a, emit):
    return (f"r {cm9_text(a['r'])}cm w {a['w']}cm off {a['off']} clrL {a['clr_l']} clrR {a['clr_r']} "
            f"side {side_char(a['side'], emit)}")


def text_reverse_cap(a, emit):
    starts = [f"standoff {a['n']} mm refused: ", "rescan skipped: ", f"rescan reverse cut to {a['n']} mm: "]
    start = lookup(starts, a["variant"], "OBST_REVERSE_CAP variant", emit)
    return f"{start}{a['used']} of {a['cap']} mm used, {a['total']} mm total"


def text_complete(a, emit):
    return f"passed on the {side_word(a['right'])}" + ("; on the path, searching for the line" if a["search"] else "")


def text_blocked(a, emit):
    start = f"{side_word(a['right'])} side: "
    if a["no_end"]:
        return start + "obstacle side did not end in 1 m; backing out"
    return f"{start}something at {cm9_text(a['cm'])} cm, {a['deg']} deg; backing out"


def text_retry(a, emit):
    if a["wider"]:
        return f"{side_word(a['right'])} side again, 100 mm wider"
    return f"trying the {side_word(a['right'])} side (same scan)"


EVENT_TEXT = {
    "STATE": lambda a, emit: STATES[a["state"]],
    "IMU_OK": text_imu_ok,
    "HUMP_START": text_hump,
    "HUMP_PEAK": text_hump,
    "HUMP_END": text_hump_end,
    "MAP": lambda a, emit: describe_map(a["map"], emit) or "cleared",
    "MAP_WARNING": lambda a, emit: describe_map(a["map"], emit) or "no barcode map yet (send MAP:...)",
    "MISSION_COMPLETE": text_mission,
    "TURN_SNAP": lambda a, emit: "line not seen, encoder angle kept" if a["not_seen"] else "stopped on the line",
    "NAV_ACTION": text_nav_action,
    "NAV": text_nav,
    "EMERGENCY_STOP": lambda a, emit: str(lookup(STOP_REASONS, a["reason"], "stop reason", emit)),
    "ERROR": lambda a, emit: str(lookup(STOP_REASONS, a["reason"], "stop reason", emit)),
    "RECOVERY_REFUSED": text_refused,
    "BC_DBG": text_bc_dbg,
    "BARCODE": text_barcode,
    "BARCODE_UNKNOWN": text_barcode,
    "LINE_SEARCH": lambda a, emit: "right first" if a["right"] else "left first",
    "FINE_SCAN": lambda a, emit: f"{a['first']}deg+2:" + "".join(f" {cm9_text(c)}" for c in a["cm"]),
    "SIDE_FOLLOW": text_side_follow,
    "SIDE_LOOK": text_side_look,
    "BYPASS_EXTEND": text_extend,
    "BYPASS_SIDESTEP": lambda a, emit: f"too close beside the obstacle: back, then {a['mm']} mm further out",
    "OBSTACLE_PROFILE": text_profile,
    "AVOIDANCE_ABORT": lambda a, emit: str(lookup(ABORT_REASONS, a["reason"], "abort reason", emit)),
    "OBST_REVERSE_CAP": text_reverse_cap,
    "OBSTACLE_BACKOFF": lambda a, emit: (f"reversing {a['need']} mm: a 45 deg turn would clip the "
                                         f"{side_word(a['right'])} corner"),
    "OBSTACLE_RESCAN": lambda a, emit: f"{'blocked' if a['blocked'] else 'no room'}: reversing 100 mm and scanning again",
    "OBSTACLE_TIE": lambda a, emit: (f"tie: 90 deg look L {a['dl']} cm, R {a['dr']} cm -> "
                                     f"side {side_char(a['side'], emit)}"),
    "BYPASS_COMPLETE": text_complete,
    "BYPASS_BLOCKED": text_blocked,
    "BYPASS_RETRY": text_retry,
    "OBSTACLE": lambda a, emit: f"at {cm13_text(a['cm'])} cm, STOP issued in {a['us']} us",
}


# ---------------- presentation layer ----------------

def present_telemetry(rec, emit):
    bc = "" if rec["bc"] == C39_NONE else bc_char(rec["bc"], emit)
    return {
        "seq": rec["seq"],
        "t": rec["t"],
        "state": STATES[rec["state"]],
        "spd": [rec["spd_l"] / 10, rec["spd_r"] / 10],
        "duty": [rec["duty_l"] / 10, rec["duty_r"] / 10],
        "enc": [rec["enc_l"], rec["enc_r"]],
        "dist": rec["dist"] / 10,
        "line": {
            "l": rec["line_l"], "r": rec["line_r"], "ir": rec["ir"], "irmn": rec["ir_min"], "irmx": rec["ir_max"],
            "thr": [rec["thr_b"], rec["thr_w"]] if rec["thr_valid"] else [0, 0],
        },
        "bc": bc,
        "hump": {"pitch": rec["pitch"] / 10, "h": rec["hump_h"] / 10,
                 "peak": rec["hump_peak"] / 10, "max": rec["hump_max"] / 10},
        "obst": {"cm": cm13_value(rec["us"]), "scan": [cm13_value(c) for c in rec["scan"]],
                 "side": lookup(SIDES, rec["side"], "side", emit)},
        "imu": {"ev": MOTION_EVENTS[rec["imu_ev"]], "tr": rec["turn"] / 10},
        "faults": rec["faults"],
        "vbat": rec["vbat"],
        "rssi": -rec["rssi"],
        "buf": rec["buf"],
    }


def present_status(rec, emit):
    return {
        "seq": rec["seq"],
        "t": rec["t"],
        "state": STATES[rec["state"]],
        "rssi": -rec["rssi"],
        "buf": rec["buf"],
        "up": rec["up"],
        "reconnects": rec["reconnects"],
        "dropped": rec["dropped"],
        "faults": rec["faults"],
        "broker": ".".join(str(octet) for octet in rec["broker"]),
        "sat": rec["sat"],
        "ptab": {"count": rec["ptab_count"], "crc": rec["ptab_crc"]},
    }


def present_event(rec, emit):
    if rec["code"] >= len(EVENTS):
        emit(f"unknown Event_Code {rec['code']}")
        return {"t": rec["t"], "event": rec["code"], "detail": "", "args": {}}
    name = EVENTS[rec["code"]][0]
    text = FIXED_DETAIL.get(name)
    if text is None:
        text = EVENT_TEXT[name](rec["args"], emit)
    return {"t": rec["t"], "event": name, "detail": text, "args": rec["args"]}


def param_value(param_id, code):
    """Parameter_Store code -> the value, as GET:PARAMS printed it (jb_fixed, 4 decimals)."""
    kind = PARAMS[param_id][1]
    if kind == "F32":
        text = jb_fixed(f32_from_bits(code), 4)
        return None if text == "null" else float(text)
    if kind == "S2":
        return float(code - 4 if code >= 2 else code)
    return float(code + {"F1": 0, "B2-1": -1, "B3+1": 1}[kind])


def param_name(param_id, emit):
    return lookup([p[0] for p in PARAMS], param_id, "Parameter_ID", emit)


def ack_echo(code, args, emit):
    """The Baseline_Build "cmd" text."""
    if code == WCMD_UNPARSED:
        return ""
    if not 0 <= code < len(COMMANDS):
        emit(f"unknown command code {code}")
        return code
    echo = COMMANDS[code][1]
    if code == WCMD["PING"]:
        return f"{echo}:{args['ping']}"
    if code == WCMD["BARCODE"]:
        return f"{echo}:{letter_text(args['letter'], emit)}"
    if code == WCMD["SET"]:
        return f"{echo}:{param_name(args['param'], emit)}"
    return echo


def present_ack(rec, emit):
    args = rec.get("args", {})
    code = rec["cmd"]
    if code == WCMD["CONFIG"] and rec["reason"] == RSN_CONFIG_REJECTED:
        result = " ".join(str(param_name(i, emit)) for i in args["rej"])
    elif 0 <= rec["reason"] < len(REASONS):
        result = REASONS[rec["reason"]][1]
    else:
        emit(f"unknown Reason_Code {rec['reason']}")
        result = rec["reason"]
    out = {"t": rec["t"], "cmd": ack_echo(code, args, emit), "result": result}
    if code == WCMD["ESTOP"]:
        out["pwm_off_us"] = args["pwm_off_us"]
    elif code in (WCMD["MAP"], WCMD["MAP_CLEAR"], WCMD["GET_MAP"]):
        out["map"] = describe_map(args["map"], emit)
        out["n"] = args["n"]
    elif code == WCMD["GET_PARAMS"]:
        out["part"] = args["part"]
        out["params"] = {PARAMS[i][0]: param_value(i, value) for i, value in args["values"]}
    elif code == WCMD["CONFIG"]:
        out["applied"] = args["applied"]
        out["rejected"] = args["n_rej"]
    return out


def slots_text(rec, emit):
    """A map body as "F=FWD,K=LEFT,...", letter A first (nav_map_describe order)."""
    words = {"fwd": "FWD", "left": "LEFT", "right": "RIGHT", "uturn": "UTURN"}
    pairs = sorted((rec[slot], words[slot]) for slot in MAP_ORDER if rec[f"{slot}_on"])
    return ",".join(f"{letter_text(letter, emit)}={word}" for letter, word in pairs)


def present_command(rec, emit):
    out = {"cmd": ack_echo(rec["code"], rec, emit)}
    if rec["code"] == WCMD["MAP"]:
        out["map"] = slots_text(rec, emit)
    elif rec["code"] == WCMD["SET"]:
        out["value"] = f32_from_bits(rec["value"])
    return out


def present_map(rec, emit):
    return {"map": slots_text(rec, emit), "n": sum(rec[f"{slot}_on"] for slot in MAP_ORDER)}


def present_config(rec, emit):
    return {"count": rec["count"],
            "params": {param_name(i, emit): f32_from_bits(code) for i, code in rec["pairs"]}}


# topic -> (payload kind, presentation)
TOPICS = {
    "is02/telemetry": ("telemetry", present_telemetry),
    "is02/status": ("heartbeat", present_status),
    "is02/event": ("event", present_event),
    "is02/ack": ("ack", present_ack),
    "is02/cmd": ("command", present_command),
    "is02/map": ("map", present_map),
    "is02/config": ("config", present_config),
}
RETAINED_TOPICS = ("is02/map", "is02/config")
# Topics the tools fill with baseline text while legacy mode is on (flag R7)
TEXT_TOPICS = ("is02/cmd", "is02/map", "is02/config")


def decode(topic, payload, legacy_json=False, log=None):
    """The Baseline_Build JSON shape of a payload, or None if it was dropped.

    Bad versions, lengths and unknown codes go to log (a function taking one
    line of text), by default the "is02_codec" logger. An empty retained
    payload is the broker deleting it, so it returns None without a log line.
    With legacy_json=True a payload starting with "{" is parsed as baseline JSON.
    """
    emit = log or LOG.warning
    data = payload.encode("utf-8") if isinstance(payload, str) else bytes(payload)
    if topic not in TOPICS:
        emit(f"{topic}: no decoder for this topic, length {len(data)}")
        return None
    if legacy_json and data[:1] == b"{":
        try:
            return json.loads(data.decode("utf-8"))
        except ValueError as e:
            emit(f"{topic}: dropped, invalid JSON: {e}")
            return None
    if not data and topic in RETAINED_TOPICS:
        return None
    kind, present = TOPICS[topic]
    try:
        rec = decode_fields(kind, data)
    except CodecError as e:
        version = data[0] >> 4 if data else "none"
        emit(f"{topic}: dropped, version {version}, length {len(data)}: {e}")
        return None
    return present(rec, emit)


def read_payload(topic, payload, log=None):
    """What the tools do with a received payload, in both modes: (decoded, text).

    "{" is baseline JSON, anything else binary (flag R7). On TEXT_TOPICS and on a
    topic with no decoder, a payload without our Format_Version is text, and
    decoded is None. text is the payload as received, or the decoded JSON of a
    binary frame, or None if the frame was dropped.
    """
    data = payload.encode("utf-8") if isinstance(payload, str) else bytes(payload)
    received = data.decode("utf-8", errors="replace")
    if topic not in TOPICS or (topic in TEXT_TOPICS and (not data or data[0] >> 4 != FORMAT_VERSION)):
        return None, received
    decoded = decode(topic, data, legacy_json=True, log=log)
    if data[:1] == b"{":
        return decoded, received
    return decoded, None if decoded is None else json.dumps(decoded, separators=(",", ":"))


def ptab_mismatch(status):
    """The warning for a decoded heartbeat whose parameter table differs from PARAMS, else None.

    A baseline JSON heartbeat has no "ptab", so it gives None.
    """
    ptab = status.get("ptab")
    if ptab is None or (ptab["count"], ptab["crc"]) == (PARAM_COUNT, PARAM_CRC):
        return None
    return (f"parameter table mismatch: robot {ptab['count']} / 0x{ptab['crc']:04X}, "
            f"tool {PARAM_COUNT} / 0x{PARAM_CRC:04X}")


# ---------------- user input -> frames ----------------

# the robot's words (nav_map.c), compared without case, "-" and "_"
MOVES = {"FWD": 1, "FORWARD": 1, "STRAIGHT": 1, "F": 1, "UTURN": 2, "U": 2, "BACK": 2,
         "LEFT": 3, "L": 3, "RIGHT": 4, "R": 4}
POSITIONAL = (1, 2, 3, 4)               # MAP:F,U,K,R gives FWD, UTURN, LEFT, RIGHT
NUMBER = re.compile(r"[+-]?(\d+\.?\d*|\.\d+)")  # what jf_parse_float accepts: no exponent


def parse_map(text):
    """Map text -> {movement: letter code}; empty for CLEAR. Same rules as nav_map_parse()."""
    text = re.sub(r"^MAP:", "", text.strip(" \t\r\n,;")).lstrip(" \t\r\n,;")
    if text[:5].upper() == "CLEAR":
        return {}
    pairs = []
    n_pos = 0
    n_named = 0
    for item in re.split(r"[,; \t\r\n]+", text):
        if not item:
            continue
        letter = item[0].upper()
        if not "A" <= letter <= "Z":
            raise CodecError(f"map: {item}: letters must be A-Z")
        if len(item) >= 3 and item[1] == "=":
            action = MOVES.get(item[2:].replace("-", "").replace("_", "").upper())
            if action is None:
                raise CodecError(f"map: {item}: unknown movement (use FWD, LEFT, RIGHT, UTURN)")
            n_named += 1
        elif len(item) == 1:
            if n_pos >= 4:
                raise CodecError(f"map: {item}: positional form takes exactly 4 letters")
            action = POSITIONAL[n_pos]
            n_pos += 1
        else:
            raise CodecError(f"map: {item}: expected L=ACTION or a single letter")
        pairs.append((letter, action))
    if n_pos and n_named:
        raise CodecError("map: do not mix the two forms")
    if n_pos and n_pos != 4:
        raise CodecError("map: positional form takes exactly 4 letters: FWD,UTURN,LEFT,RIGHT")
    if not pairs:
        raise CodecError("map: empty map")
    slots = {}
    seen = set()
    for letter, action in pairs:
        if letter in seen:
            raise CodecError(f"map: {letter}: the same letter was given twice")
        seen.add(letter)
        slot = MAP_SLOTS[action]
        if slot in slots:
            raise CodecError(f"map: {letter}: {ACTIONS[action]} already has letter "
                             f"{chr(ord('A') + slots[slot])}: the Map_Frame holds one letter per movement")
        slots[slot] = ord(letter) - ord("A")
    return slots


def map_body(slots):
    rec = {}
    for slot in MAP_ORDER:
        rec[f"{slot}_on"] = 1 if slot in slots else 0
        rec[slot] = slots.get(slot, 0)
    return rec


def encode_map(pairs):
    """The Map_Frame. pairs: map text ("K=LEFT,Q=RIGHT") or a list of "K=LEFT" or ("K", "LEFT")."""
    if not isinstance(pairs, str):
        pairs = ",".join(p if isinstance(p, str) else f"{p[0]}={p[1]}" for p in pairs)
    return encode_fields("map", map_body(parse_map(pairs)))


def parse_param(item):
    """"NAME=VALUE" -> (name, Parameter_ID, F32 bits). CodecError names the NAME."""
    if item.startswith("SET:"):
        item = item[4:]
    name, sep, value = item.partition("=")
    name, value = name.strip(), value.strip()
    if not sep:
        raise CodecError(f"{name}: expected NAME=VALUE")
    if name not in PARAM_IDS:
        raise CodecError(f"{name}: unknown parameter")
    if not NUMBER.fullmatch(value):
        raise CodecError(f"{name}: bad number {value!r}")
    param_id = PARAM_IDS[name]
    number = float(value)
    _, kind, low, high = PARAMS[param_id]
    try:
        store = f32(number)
    except OverflowError:
        raise CodecError(f"{name}: {value} out of range {low}..{high}") from None
    if store < f32(low) or store > f32(high):      # the robot compares float32 values
        raise CodecError(f"{name}: {value} out of range {low}..{high}")
    if kind != "F32" and number != int(number):
        raise CodecError(f"{name}: {value} is not a whole number")
    return name, param_id, f32_to_bits(number)


def encode_command_text(text):
    """A Baseline_Build command text -> its Command_Frame. CodecError names the argument."""
    cmd = text.strip()
    simple = {"ESTOP": "ESTOP", "RESUME": "RESUME", "START": "START", "STOP": "STOP",
              "GET:MAP": "GET_MAP", "GET:PARAMS": "GET_PARAMS"}
    if cmd in simple:
        return encode_fields("command", {"code": WCMD[simple[cmd]]})
    if cmd == "PING" or cmd.startswith("PING:"):
        number = cmd[5:]
        if cmd != "PING" and not (number.isascii() and number.isdigit() and int(number) < 65536):
            raise CodecError(f"argument {cmd}: PING takes a number 0..65535")
        return encode_fields("command", {"code": WCMD["PING"], "ping": int(number or 0)})
    if cmd.startswith("BARCODE:"):
        letter = cmd[8:]
        if len(letter) != 1 or not "A" <= letter <= "Z":
            raise CodecError(f"argument {cmd}: use BARCODE:<A-Z>")
        return encode_fields("command", {"code": WCMD["BARCODE"], "letter": ord(letter) - ord("A")})
    if cmd.startswith("SET:"):
        try:
            _, param_id, bits = parse_param(cmd)
        except CodecError as e:
            raise CodecError(f"argument {cmd}: {e}") from None
        return encode_fields("command", {"code": WCMD["SET"], "param": param_id, "value": bits})
    if cmd.startswith("MAP:"):
        slots = parse_map(cmd)
        if not slots:
            return encode_fields("command", {"code": WCMD["MAP_CLEAR"]})
        return encode_fields("command", dict(map_body(slots), code=WCMD["MAP"]))
    raise CodecError(f"argument {cmd}: unknown command")


def parse_config(lines):
    """NAME=VALUE lines -> [[Parameter_ID, F32 bits], ...].

    Blank lines and # comments are ignored, and one line may hold several items
    split by ; or , as on the robot. Every bad line is listed in the CodecError,
    with its line number (from 1) and NAME.
    """
    if isinstance(lines, str):
        lines = lines.splitlines()
    pairs = []
    errors = []
    for number, raw in enumerate(lines, 1):
        for item in re.split(r"[;,]", raw.split("#", 1)[0]):
            item = item.strip()
            if not item:
                continue
            try:
                _, param_id, bits = parse_param(item)
                pairs.append([param_id, bits])
            except CodecError as e:
                name, _, reason = str(e).partition(": ")
                errors.append((number, name, reason))
    if errors:
        raise CodecError("\n".join(f"line {n}: {name}: {why}" for n, name, why in errors), errors)
    if len(pairs) > PARAM_COUNT:
        raise CodecError(f"{len(pairs)} values: the Config_Frame holds at most {PARAM_COUNT}")
    return pairs


def encode_config(lines):
    """robot_params.txt lines (or its whole text) -> the Config_Frame."""
    pairs = parse_config(lines)
    return encode_fields("config", {"count": len(pairs), "pairs": pairs})
