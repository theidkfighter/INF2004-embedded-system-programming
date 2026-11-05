"""LT-04 helpers shared by the cross-codec tests, make_vectors.py and
cross_cases.py: seeded generators of field records for every payload kind, the
field-list form of a record (the line format of tests/host/tools/wire_vectors.c)
and a running wire_vectors process (the C Wire_Codec).

Records are the field layer of the Laptop_Codec (decode_fields / encode_fields).
Every generated field is inside its Valid_Range; the generators lean on the
edge values of each width. Standard library only, so cross_cases.py can run
under any python3.
"""

import atexit
import json
import shutil
import subprocess

import wire_spec
from codec_helpers import is02_codec as codec

KINDS = ("telemetry", "heartbeat", "event", "ack", "command", "map", "config")
C_KIND = {"command": "cmd"}                 # the name wire_vectors uses
TOPIC = {kind: topic for topic, (kind, _) in codec.TOPICS.items()}
ACK_CODES = list(range(13)) + [codec.WCMD_UNPARSED]
COMMAND_CODES = [code for code, c in enumerate(codec.COMMANDS) if c[2] > 0]   # all but CONFIG
SLOTS = ("fwd", "left", "right", "uturn")


# ---------------- random fields ----------------

def rnd(rng, width, signed=False):
    """A value of the field, with the edge values over-represented."""
    lo, hi = (-(1 << (width - 1)), (1 << (width - 1)) - 1) if signed else (0, (1 << width) - 1)
    r = rng.random()
    if r < 0.10:
        return lo
    if r < 0.20:
        return hi
    if r < 0.25:
        return min(max(0, lo), hi)
    if r < 0.28 and signed:
        return -1
    return rng.randint(lo, hi)


def pick_range(n):
    return lambda rng: rng.randrange(n)


def pick_between(lo, hi):
    return lambda rng: rng.randint(lo, hi)


def pick_cm(sentinel, top):
    """An ultrasonic distance code: the no-echo sentinel or 0..top."""
    return lambda rng: rng.choice([sentinel, 0, top, rng.randint(0, top)])


CM9 = pick_cm(511, 431)
CM13 = pick_cm(8191, 4310)
ANY_CODE39 = pick_between(0, 42)


def gen_layout(rng, layout, special=None, rec=None):
    """A record for a layout: one value per field, a list for the array fields."""
    rec = {} if rec is None else rec
    special = special or {}
    for field in layout:
        name, width = field[0], field[1]
        make = special.get(name) or (lambda r, w=abs(width), s=width < 0: rnd(r, w, s))
        if len(field) == 2:
            rec[name] = make(rng)
        else:
            count = rec[field[2]] if isinstance(field[2], str) else field[2]
            rec[name] = [make(rng) for _ in range(count)]
    return rec


def finite_f32_bits(rng):
    """The bit pattern of a finite float32 (a value a tool can send)."""
    bits = rng.getrandbits(32)
    if (bits >> 23) & 0xFF == 0xFF:
        bits &= ~(1 << 23)
    return bits


# ---------------- records per kind ----------------

def gen_telemetry(rng):
    rec = gen_layout(rng, codec.TELEMETRY, {
        "state": pick_range(len(codec.STATES)), "line_l": pick_between(-1, 1), "line_r": pick_between(-1, 1),
        "bc": lambda r: r.choice([63, r.randint(0, 42)]), "us": CM13, "scan": CM13,
        "side": pick_range(len(codec.SIDES)), "imu_ev": pick_range(len(codec.MOTION_EVENTS)),
    })
    if not rec["thr_valid"]:
        rec["thr_b"] = rec["thr_w"] = 0     # the decoder rejects thresholds that are not valid
    return rec


def gen_heartbeat(rng):
    return gen_layout(rng, codec.HEARTBEAT, {"state": pick_range(len(codec.STATES))})


EVENT_SPECIAL = {
    "STATE": {"state": pick_range(len(codec.STATES))},
    "IMU_OK": {"chip": pick_range(len(codec.IMU_CHIPS))},
    "MAP": {"map": pick_range(len(codec.ACTIONS))},
    "MAP_WARNING": {"map": pick_range(len(codec.ACTIONS))},
    "NAV_ACTION": {"letter": pick_range(26), "action": pick_range(len(codec.ACTIONS))},
    "NAV": {"variant": pick_range(4), "letter": pick_range(26), "action": pick_range(len(codec.ACTIONS))},
    "EMERGENCY_STOP": {"reason": pick_range(len(codec.STOP_REASONS))},
    "ERROR": {"reason": pick_range(len(codec.STOP_REASONS))},
    "BARCODE": {"code": ANY_CODE39},
    "BARCODE_UNKNOWN": {"code": ANY_CODE39},
    "FINE_SCAN": {"first": pick_between(-60, 60), "count": pick_between(1, 11), "cm": CM9},
    "SIDE_FOLLOW": {"near": CM13},
    "SIDE_LOOK": {"d90": CM13, "d60": CM13},
    "BYPASS_EXTEND": {"cm": CM9},
    "OBSTACLE_PROFILE": {"r": CM9, "side": pick_range(len(codec.SIDES))},
    "AVOIDANCE_ABORT": {"reason": pick_range(len(codec.ABORT_REASONS))},
    "OBST_REVERSE_CAP": {"variant": pick_range(3)},
    "OBSTACLE_TIE": {"side": pick_range(len(codec.SIDES))},
    "BYPASS_BLOCKED": {"cm": CM9},
    "OBSTACLE": {"cm": pick_between(0, 4310)},
}


def bc_dbg_special(rng_name):
    """Choosers for the BC_DBG argument names (the level and window fields repeat per entry)."""
    if rng_name.startswith("res"):
        return pick_range(3)
    if rng_name.startswith("code"):
        return lambda r: r.choice([63, r.randint(0, 42)])
    return {"part": pick_between(1, 6), "count": pick_between(0, 16), "n_levels": pick_between(1, 6),
            "level": pick_range(6)}.get(rng_name)


def gen_event_args(rng, code):
    name = codec.EVENTS[code][0]
    if name != "BC_DBG":
        args = gen_layout(rng, codec.event_layout(code), EVENT_SPECIAL.get(name))
        if name in ("HUMP_START", "HUMP_PEAK"):
            args["valid"] = 1               # only HUMP_END reports a false slope
        return args
    variant = rng.randrange(5)
    layout = codec.BC_DBG_ARGS[variant]
    special = {field[0]: bc_dbg_special(field[0]) for field in layout}
    return gen_layout(rng, layout, {k: v for k, v in special.items() if v}, {"variant": variant})


def gen_event(rng, code=None):
    code = rng.randrange(len(codec.EVENTS)) if code is None else code
    return {"t": rnd(rng, 24), "code": code, "args": gen_event_args(rng, code)}


def gen_map_body(rng, count=None):
    """The four movement slots; a present slot names a distinct letter, an absent one has letter 0."""
    count = rng.randrange(5) if count is None else count
    slots = rng.sample(SLOTS, count)
    letters = rng.sample(range(26), count)
    rec = {}
    for slot in SLOTS:
        rec[slot + "_on"], rec[slot] = 0, 0
    for slot, letter in zip(slots, letters):
        rec[slot + "_on"], rec[slot] = 1, letter
    return {name: rec[name] for name, _ in codec.MAP_BODY}


def gen_ack(rng, cmd=None):
    cmd = rng.choice(ACK_CODES) if cmd is None else cmd
    rec = {"t": rnd(rng, 24), "cmd": cmd, "reason": rng.randrange(len(codec.REASONS))}
    if cmd == codec.WCMD["GET_PARAMS"]:
        count = rng.choice([0, 1, codec.GETP_PER_PART, rng.randint(0, codec.GETP_PER_PART)])
        values = []
        for _ in range(count):
            pid = rng.randrange(codec.PARAM_COUNT)
            values.append([pid, rnd(rng, codec.PARAM_KINDS[codec.PARAMS[pid][1]])])
        rec["args"] = {"part": rng.randint(1, 7), "count": count, "values": values}
    elif cmd == codec.WCMD["CONFIG"]:
        n_rej = rng.choice([0, 1, codec.PARAM_COUNT, rng.randint(0, codec.PARAM_COUNT)])
        rec["args"] = {"applied": rnd(rng, 6), "n_rej": n_rej, "rej": [rng.randrange(codec.PARAM_COUNT)
                                                                      for _ in range(n_rej)]}
    else:
        special = {"map": pick_range(len(codec.ACTIONS)), "n": pick_between(0, 26)}
        rec["args"] = gen_layout(rng, codec.ACK_ARGS[cmd], special)
    return rec


def gen_command(rng, code=None):
    code = rng.choice(COMMAND_CODES) if code is None else code
    rec = {"code": code}
    if code == codec.WCMD["MAP"]:
        rec.update(gen_map_body(rng))
    else:
        special = {"letter": pick_range(26), "param": pick_range(codec.PARAM_COUNT), "value": finite_f32_bits}
        gen_layout(rng, codec.COMMAND_ARGS[code], special, rec)
    return rec


def gen_config(rng):
    count = rng.choice([0, 1, codec.PARAM_COUNT, rng.randint(0, codec.PARAM_COUNT)])
    ids = rng.sample(range(codec.PARAM_COUNT), count)
    return {"count": count, "pairs": [[pid, finite_f32_bits(rng)] for pid in ids]}


def gen_record(kind, rng, code=None):
    """A valid record of the kind; code picks the Event_Code, ack command code or command code."""
    return {
        "telemetry": lambda: gen_telemetry(rng),
        "heartbeat": lambda: gen_heartbeat(rng),
        "event": lambda: gen_event(rng, code),
        "ack": lambda: gen_ack(rng, code),
        "command": lambda: gen_command(rng, code),
        "map": lambda: gen_map_body(rng),
        "config": lambda: gen_config(rng),
    }[kind]()


# ---------------- records as wire_vectors lines ----------------

def event_layout_of(rec):
    if rec["code"] == codec.EVC["BC_DBG"]:
        return (("variant", 3),) + codec.BC_DBG_ARGS[rec["args"]["variant"]]
    return codec.event_layout(rec["code"])


def event_fields(rec):
    """[(code, width), ...] of the argument fields, in stream order."""
    out = []
    for field in event_layout_of(rec):
        width = abs(field[1])
        values = rec["args"][field[0]] if len(field) == 3 else [rec["args"][field[0]]]
        out.extend((v & ((1 << width) - 1), width) for v in values)
    return out


def event_bits(rec):
    """(argument bit count, argument bytes as hex, MSB first, padded with zeros)."""
    bits = "".join(format(v, f"0{w}b") for v, w in event_fields(rec))
    padded = bits + "0" * (-len(bits) % 8)
    return len(bits), "".join(format(int(padded[i:i + 8], 2), "02x") for i in range(0, len(padded), 8))


def values(layout, rec):
    out = []
    for field in layout:
        value = rec[field[0]]
        out.extend(value if len(field) == 3 else [value])
    return out


def flat(kind, rec):
    """The numbers of a record in the order of its wire_vectors line (events: t, code, bit count)."""
    if kind == "telemetry":
        return values(codec.TELEMETRY, rec)
    if kind == "heartbeat":
        return values(codec.HEARTBEAT, rec)
    if kind == "map":
        return values(codec.MAP_BODY, rec)
    if kind == "command":
        return [rec["code"]] + values(codec.COMMAND_ARGS[rec["code"]], rec)
    if kind == "config":
        return [rec["count"]] + [x for pair in rec["pairs"] for x in pair]
    if kind == "event":
        return [rec["t"], rec["code"], event_bits(rec)[0]]
    args = rec["args"]
    out = [rec["t"], rec["cmd"], rec["reason"]]
    if rec["cmd"] == codec.WCMD["GET_PARAMS"]:
        out += [args["part"], args["count"]]
        for pid, code in args["values"]:
            out += [pid, codec.PARAM_KINDS[codec.PARAMS[pid][1]], code]
    else:
        out += values(codec.ACK_ARGS[rec["cmd"]], args)
    return out


def line_of(kind, rec):
    """The wire_vectors input line that encodes the record."""
    if kind == "event":
        nbits, hexed = event_bits(rec)
        return f"event {rec['t']} {rec['code']} {nbits}" + (f" {hexed}" if nbits else "")
    return f"{C_KIND.get(kind, kind)} " + " ".join(str(x) for x in flat(kind, rec))


def decode_line(kind, data):
    return f"decode {C_KIND.get(kind, kind)} {bytes(data).hex()}".rstrip()


# ---------------- the C Wire_Codec ----------------

class CVectors:
    """One wire_vectors process, asked one line at a time (stdout kept line-buffered with stdbuf)."""

    def __init__(self):
        exe = str(wire_spec.tools() / "wire_vectors")
        stdbuf = shutil.which("stdbuf")
        self.live = stdbuf is not None
        self.proc = None
        if self.live:
            self.proc = subprocess.Popen([stdbuf, "-oL", exe], stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                         text=True, bufsize=1)
            atexit.register(self.close)

    def run(self, line):
        if not self.live:
            return wire_spec.run_vectors([line])[0]
        self.proc.stdin.write(line + "\n")
        self.proc.stdin.flush()
        out = self.proc.stdout.readline()
        if not out:
            raise RuntimeError(f"wire_vectors stopped on: {line}")
        return json.loads(out)

    def close(self):
        if self.proc is not None and self.proc.poll() is None:
            self.proc.stdin.close()
            self.proc.wait(timeout=5)


_c = None


def c():
    global _c
    if _c is None:
        _c = CVectors()
    return _c


def c_encode(kind, rec):
    """The bytes the C Wire_Codec makes of a record (b"" when it refuses the fields)."""
    return bytes.fromhex(c().run(line_of(kind, rec))["hex"])


def c_decode(kind, data):
    """(status, numbers, argument hex or None) of the C Wire_Codec decoding the bytes."""
    out = c().run(decode_line(kind, data))
    return out["status"], out["fields"], out.get("args")


def c_lines(lines):
    """Many lines through one process, in order."""
    return wire_spec.run_vectors(lines)
