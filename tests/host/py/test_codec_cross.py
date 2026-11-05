"""Cross-codec properties 25 to 28 (spec tasks 14.11 to 14.14): the C Wire_Codec
(build/wire_vectors, one process), the Python Laptop_Codec and, in
js/cross.test.js, the JavaScript Laptop_Codec, on the same kinds of frames.

Records are generated from a seed by codec_gen.py; each example draws a seed
(Hypothesis' own --hypothesis-seed makes a run reproducible).
"""

import json
import random

import pytest
from hypothesis import example, given, settings
from hypothesis import strategies as st

import codec_gen as gen
import make_vectors
from codec_helpers import REPO_DIR, is02_codec as codec

SEEDS = st.integers(0, 2 ** 32 - 1)
VECTORS_JSON = REPO_DIR / "tests" / "host" / "vectors" / "vectors.json"

T25 = "Robot-to-laptop frames round-trip through both Laptop_Codecs"
T26 = "Laptop-to-robot frames round-trip into the Wire_Codec"
T27 = "Decode then encode is the identity on bytes"
T28 = "The three codecs agree on the shared vectors"

EVENT_NAMES = [e[0] for e in codec.EVENTS]
COMMAND_NAMES = [c[0] for c in codec.COMMANDS if c[2] > 0]      # every is02/cmd command


def cm(code, none):
    return -1.0 if code == none else code / 10


def expected_telemetry(rec):
    """The Baseline_Build JSON values of a Telemetry_Frame, from the field codes and the D8 resolutions."""
    return {
        "seq": rec["seq"], "t": rec["t"], "spd": [rec["spd_l"] / 10, rec["spd_r"] / 10],
        "duty": [rec["duty_l"] / 10, rec["duty_r"] / 10], "enc": [rec["enc_l"], rec["enc_r"]],
        "dist": rec["dist"] / 10, "faults": rec["faults"], "vbat": rec["vbat"], "rssi": -rec["rssi"],
        "buf": rec["buf"],
        "hump": {"pitch": rec["pitch"] / 10, "h": rec["hump_h"] / 10, "peak": rec["hump_peak"] / 10,
                 "max": rec["hump_max"] / 10},
        "obst": {"cm": cm(rec["us"], 8191), "scan": [cm(c, 8191) for c in rec["scan"]]},
        "imu": {"tr": rec["turn"] / 10},
        "line": {"l": rec["line_l"], "r": rec["line_r"], "ir": rec["ir"], "irmn": rec["ir_min"], "irmx": rec["ir_max"],
                 "thr": [rec["thr_b"], rec["thr_w"]] if rec["thr_valid"] else [0, 0]},
    }


def subset(got, want):
    """Every key of want is in got with an equal value (nested dicts compared the same way)."""
    for key, value in want.items():
        if isinstance(value, dict):
            subset(got[key], value)
        else:
            assert got[key] == value, f"{key}: {got[key]!r} != {value!r}"


def expected_status(rec):
    return {"seq": rec["seq"], "t": rec["t"], "rssi": -rec["rssi"], "buf": rec["buf"], "up": rec["up"],
            "reconnects": rec["reconnects"], "dropped": rec["dropped"], "faults": rec["faults"], "sat": rec["sat"],
            "broker": ".".join(str(o) for o in rec["broker"]),
            "ptab": {"count": rec["ptab_count"], "crc": rec["ptab_crc"]}}


# ---------------- Property 25 ----------------

def round_trip_25(kind, rec):
    data = gen.c_encode(kind, rec)
    assert data, f"the C Wire_Codec refused {kind} {rec}"
    assert codec.decode_fields(kind, data) == rec
    if kind == "telemetry":
        subset(codec.decode(gen.TOPIC[kind], data), expected_telemetry(rec))
    elif kind == "heartbeat":
        subset(codec.decode(gen.TOPIC[kind], data), expected_status(rec))


# Feature: minimum-bit-data-representation, Property 25: Robot-to-laptop frames round-trip through both Laptop_Codecs
@pytest.mark.prop(25, T25)
@pytest.mark.parametrize("unit", ["telemetry", "heartbeat", "ack"])
@settings(max_examples=1000)
@given(seed=SEEDS)
def test_p25_telemetry_heartbeat_ack(unit, seed):
    rng = random.Random(seed)
    code = rng.choice(gen.ACK_CODES) if unit == "ack" else None
    round_trip_25(unit, gen.gen_record(unit, rng, code))


# Feature: minimum-bit-data-representation, Property 25: Robot-to-laptop frames round-trip through both Laptop_Codecs
@pytest.mark.prop(25, T25)
@pytest.mark.parametrize("unit", EVENT_NAMES)
@settings(max_examples=100)
@given(seed=SEEDS)
def test_p25_events(unit, seed):
    round_trip_25("event", gen.gen_event(random.Random(seed), codec.EVC[unit]))


# ---------------- Property 26 ----------------

def round_trip_26(kind, rec):
    data = codec.encode_fields(kind, rec)
    status, fields, _ = gen.c_decode(kind, data)
    assert status == 0, f"the C Wire_Codec rejects {data.hex()} (status {status}) for {rec}"
    assert fields == gen.flat(kind, rec)


# Feature: minimum-bit-data-representation, Property 26: Laptop-to-robot frames round-trip into the Wire_Codec
@pytest.mark.prop(26, T26)
@pytest.mark.parametrize("unit", COMMAND_NAMES)
@settings(max_examples=1000)
@given(seed=SEEDS)
def test_p26_commands(unit, seed):
    round_trip_26("command", gen.gen_command(random.Random(seed), codec.WCMD[unit]))


# Feature: minimum-bit-data-representation, Property 26: Laptop-to-robot frames round-trip into the Wire_Codec
@pytest.mark.prop(26, T26)
@pytest.mark.parametrize("unit", ["map", "config"])
@settings(max_examples=1000)
@given(seed=SEEDS)
def test_p26_map_config(unit, seed):
    round_trip_26(unit, gen.gen_record(unit, random.Random(seed)))


# ---------------- Property 27 ----------------

def c_line_of_fields(kind, fields, args):
    if kind == "event":
        return f"event {fields[0]} {fields[1]} {fields[2]}" + (f" {args}" if fields[2] else "")
    return f"{gen.C_KIND.get(kind, kind)} " + " ".join(str(x) for x in fields)


# Feature: minimum-bit-data-representation, Property 27: Decode then encode is the identity on bytes
@pytest.mark.prop(27, T27)
@pytest.mark.parametrize("unit", gen.KINDS)
@settings(max_examples=1000)
@given(seed=SEEDS)
def test_p27_identity(unit, seed):
    data = gen.c_encode(unit, gen.gen_record(unit, random.Random(seed)))
    assert data
    status, fields, args = gen.c_decode(unit, data)
    assert status == 0
    assert bytes.fromhex(gen.c().run(c_line_of_fields(unit, fields, args))["hex"]) == data, "C Wire_Codec"
    assert codec.encode_fields(unit, codec.decode_fields(unit, data)) == data, "Python Laptop_Codec"


# ---------------- Property 28 ----------------

VECTORS = json.loads(VECTORS_JSON.read_text(encoding="utf-8"))["vectors"]


def check_vector(vec):
    kind, data = vec["kind"], bytes.fromhex(vec["hex"])
    if "reject" in vec:
        status, _, _ = gen.c_decode(kind, data)
        assert status != 0, f"C Wire_Codec accepted a {vec['reject']} {kind} payload"
        with pytest.raises(codec.CodecError):
            codec.decode_fields(kind, data)
        return
    rec = vec["record"]
    assert bytes.fromhex(gen.c().run(vec["line"])["hex"]) == data, "C encode"
    assert codec.encode_fields(kind, rec) == data, "Python encode"
    status, fields, args = gen.c_decode(kind, data)
    assert (status, fields) == (0, gen.flat(kind, rec)), "C decode"
    if kind == "event":
        assert args == gen.event_bits(rec)[1], "C decode of the event arguments"
    assert codec.decode_fields(kind, data) == rec, "Python decode"


def every_vector(test):
    for index in range(len(VECTORS)):
        test = example(index=index)(test)
    return test


# Feature: minimum-bit-data-representation, Property 28: The three codecs agree on the shared vectors
@pytest.mark.prop(28, T28, unit="shared vectors")
@every_vector
@settings(max_examples=100)
@given(index=st.integers(0, len(VECTORS) - 1))
def test_p28_shared_vectors(index):
    check_vector(VECTORS[index])


def test_vectors_file_is_current():
    """vectors.json and vectors.txt are what make_vectors.py writes from the recorded seed."""
    recorded = json.loads(VECTORS_JSON.read_text(encoding="utf-8"))
    assert make_vectors.build(recorded["seed"]) == recorded["vectors"], \
        "vectors.json is stale: run `python3 tests/host/py/make_vectors.py` and commit it"
    kinds = {v["kind"] for v in VECTORS if "record" in v}
    assert kinds == set(gen.KINDS)
    assert {v["record"]["code"] for v in VECTORS if v["kind"] == "event" and "record" in v} == set(range(45))
