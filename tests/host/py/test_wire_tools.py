"""The protocol host tools (spec tasks 3.11 and 14.5 C half).

- build/wire_dump prints the C tables and frame layouts as JSON; counts,
  texts and store widths follow the design tables, and the telemetry /
  heartbeat layouts equal design D8 / D9 (test_spec_conformance.D8, D9).
- build/wire_vectors encodes and decodes vector lines; encode -> decode
  gives the input back, and the bytes carry the fields at the D8 / D9 / D11
  offsets.
"""

import json
import random
import struct
import subprocess
from pathlib import Path

import pytest

import wire_spec
from test_spec_conformance import D15, D8, D9, bits_at

HOST = Path(__file__).resolve().parents[1]
BUILD = HOST / "build"

# design D2 stop reasons and D10 abort reasons (the Baseline_Build texts)
STOP_TEXTS = ["", "remote E-stop (MQTT)", "BTN2 stop", "collision (IMU)", "obstacle closer than 8 cm",
              "watchdog: a control task stopped running", "calibration missing",
              "IMU fault (3 failed I2C recoveries)", "battery critical", "fault reported (see faults in telemetry)"]
ABORT_TEXTS = ["bypass leg failed (timeout, E-stop or proximity)", "realign failed",
               "reverse onto the original path failed", "reverse limit reached",
               "reverse failed (E-stop, proximity or timeout)", "profile failed (no echo in the fine scan)",
               "no room on either side", "no bypass plan", "backing out failed (E-stop, proximity or timeout)",
               "blocked on both sides", "too many attempts"]


@pytest.fixture(scope="module")
def tools():
    subprocess.run(["make", "-s", "-C", str(HOST), "tools"], check=True, capture_output=True)
    return BUILD


@pytest.fixture(scope="module")
def dump(tools):
    out = subprocess.run([str(tools / "wire_dump")], check=True, capture_output=True, text=True).stdout
    return json.loads(out)


def run_vectors(tools, lines):
    out = subprocess.run([str(tools / "wire_vectors")], input="\n".join(lines) + "\n", check=True,
                         capture_output=True, text=True).stdout
    return [json.loads(x) for x in out.splitlines()]


def test_dump_tables_match_design(dump):
    """Counts and texts from design D2, D10, D11, D12, D15 (the laptop
    track's table test compares its codecs with this same JSON)."""
    assert dump["format_version"] == 1
    assert len(dump["events"]) == 45 and [e["code"] for e in dump["events"]] == list(range(45))
    assert len(dump["reasons"]) == 28 and dump["reasons"][16]["text"] == "nack: unknown format version"
    assert [c["code"] for c in dump["commands"]][:11] == list(range(11))
    assert [(c["cmd_len"]) for c in dump["commands"][:11]] == [1, 1, 1, 1, 2, 4, 1, 1, 3, 6, 1]
    names = [p["name"] for p in dump["params"]]
    assert len(names) == 61 and dump["param_table_crc"] == wire_spec.table_crc(names)
    assert {names[p["id"]]: p["store_width"] for p in dump["params"] if p["store_width"] != 32} == \
        {n: w for n, (_sh, w, _e) in D15.items()}
    assert [s["text"] for s in dump["stop_reasons"]] == STOP_TEXTS
    assert [a["text"] for a in dump["abort_reasons"]] == ABORT_TEXTS


def test_dump_layouts_match_design(dump):
    tlm = dump["frames"]["telemetry"]
    assert (tlm["bits"], tlm["len"]) == (432, 54)
    assert [(f["offset"], f["width"]) for f in tlm["fields"][1:]] == [(r[0], r[1]) for r in D8]
    hb = dump["frames"]["heartbeat"]
    assert (hb["bits"], hb["len"]) == (195, 25)
    assert [(f["offset"], f["width"]) for f in hb["fields"][1:-1]] == [(r[0], r[1]) for r in D9]
    mp = dump["frames"]["map"]["fields"]
    assert [f["name"] for f in mp[1:9:2]] == ["fwd_present", "left_present", "right_present", "uturn_present"]


def _rand_tlm(rng):
    s = lambda w: rng.randint(-(1 << (w - 1)) + 1, (1 << (w - 1)) - 1)   # noqa: E731
    thr = rng.randint(0, 1)
    return [rng.randint(0, 32767), rng.randint(0, (1 << 24) - 1), rng.randint(0, 7), s(15), s(15), s(11),
            s(11), s(24), s(24), rng.randint(0, 1000000), rng.choice([-1, 0, 1]), rng.choice([-1, 0, 1]),
            rng.randint(0, 4095), rng.randint(0, 4095), rng.randint(0, 4095), thr,
            rng.randint(0, 4095) * thr, rng.randint(0, 4095) * thr, rng.choice([63, rng.randint(0, 42)]),
            s(11), s(15), rng.randint(0, 15000), rng.randint(0, 15000)] + \
           [rng.choice([8191, rng.randint(0, 4310)]) for _ in range(6)] + \
           [rng.randint(0, 2), rng.randint(0, 7), s(14), rng.randint(0, 65535), rng.randint(0, 6600),
            rng.randint(0, 255), rng.randint(0, 64)]


def _rand_hb(rng):
    return [rng.randint(0, 16383), rng.randint(0, (1 << 24) - 1), rng.randint(0, 7), rng.randint(0, 255),
            rng.randint(0, 64), rng.randint(0, 131071)] + [rng.randint(0, 65535) for _ in range(4)] + \
           [rng.randint(0, 63), rng.randint(0, 65535)] + [rng.randint(0, 255) for _ in range(4)]


def test_vectors_round_trip(tools):
    rng = random.Random(2004)
    enc = []
    for _ in range(200):
        enc.append(("telemetry", _rand_tlm(rng)))
        enc.append(("heartbeat", _rand_hb(rng)))
        p, used = [], set()
        for _k in range(4):
            if rng.random() < 0.7:
                letter = rng.choice([x for x in range(26) if x not in used])
                used.add(letter)
                p += [1, letter]
            else:
                p += [0, 0]
        enc.append(("map", p))
        n = rng.randint(0, 5)
        cfg = [n]
        for _k in range(n):
            cfg += [rng.randint(0, 54), struct.unpack(">I", struct.pack(">f", rng.uniform(-5, 5)))[0]]
        enc.append(("config", cfg))
        enc.append(("cmd", [8, rng.randint(0, 65535)]))
        enc.append(("cmd", [4, rng.randint(0, 25)]))
        enc.append(("ack", [rng.randint(0, 1 << 23), 8, 4, rng.randint(0, 65535)]))
    lines = [f"{k} " + " ".join(str(v) for v in f) for k, f in enc]
    res = run_vectors(tools, lines)
    assert all(r["hex"] for r in res), [r for r in res if not r["hex"]][:3]
    of = {"telemetry": "telemetry", "heartbeat": "heartbeat", "map": "map", "config": "config",
          "cmd": "cmd", "ack": "ack"}
    dec = run_vectors(tools, [f"decode {of[k]} {r['hex']}" for (k, _f), r in zip(enc, res)])
    for (k, f), r, d in zip(enc, res, dec):
        assert d["status"] == 0 and d["fields"] == f, (k, f, d)
        data = bytes.fromhex(r["hex"])
        if k == "telemetry":
            assert bits_at(data, 4, 15) == f[0]
            assert bits_at(data, 369, 2) == f[29] and bits_at(data, 417, 8) == f[34]
        elif k == "heartbeat":
            assert (bits_at(data, 4, 14), bits_at(data, 141, 6), bits_at(data, 147, 16)) == (f[0], f[10], f[11])
        elif k == "ack":
            assert (bits_at(data, 28, 4), bits_at(data, 32, 5), bits_at(data, 37, 16)) == (8, 4, f[3])


def test_vectors_event_and_rejects(tools):
    res = run_vectors(tools, ["event 777 15 4 10",                 # EMERGENCY_STOP, stop reason 1
                              "event 5 0 3 40",                    # STATE 2
                              "event 5 0 4 40"])                   # wrong bit count -> refused
    assert res[0]["hex"] and res[1]["hex"] and res[2]["hex"] == ""
    ev = bytes.fromhex(res[0]["hex"])
    assert (bits_at(ev, 4, 24), bits_at(ev, 28, 6), bits_at(ev, 34, 4)) == (777, 15, 1)
    d = run_vectors(tools, [f"decode event {res[0]['hex']}", "decode cmd 41", "decode telemetry 10"])
    assert d[0]["status"] == 0 and d[0]["fields"] == [777, 15, 4]
    assert d[1]["status"] == 1                                     # 'A' = text: unknown version
    assert d[2]["status"] == 3                                     # short telemetry: bad length


def test_dump_param_limits(dump, lib, base):
    """min / max in wire_dump equal PARAMS[] (float32) and the Baseline_Build limits."""
    import ctypes
    for name in ("params_min", "params_max"):
        fn = lib[name]
        fn.restype, fn.argtypes = ctypes.c_float, [ctypes.c_uint32]
    pmin, pmax = lib["params_min"], lib["params_max"]
    pmin.restype = pmax.restype = ctypes.c_float
    pmin.argtypes = pmax.argtypes = [ctypes.c_uint32]
    f32 = lambda v: struct.unpack("<f", struct.pack("<f", v))[0]    # noqa: E731
    for p in dump["params"]:
        i = p["id"]
        assert f32(p["min"]) == pmin(i) and f32(p["max"]) == pmax(i), p["name"]
        assert (pmin(i), pmax(i)) == (base.base_bh_param_min(i), base.base_bh_param_max(i)), p["name"]
