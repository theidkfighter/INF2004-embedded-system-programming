"""The protocol host tools (spec tasks 3.11 and 14.5 C half).

- build/wire_dump prints the C tables and frame layouts as JSON; the tables
  equal the Python Laptop_Codec's, and the telemetry / heartbeat layouts
  equal design D8 / D9 (test_spec_conformance.D8, D9).
- build/wire_vectors encodes and decodes vector lines; encode -> decode
  gives the input back, and the Python codec reads the same bytes.
"""

import json
import random
import struct
import subprocess
import sys
from pathlib import Path

import pytest

from test_spec_conformance import D8, D9, bits_at

HOST = Path(__file__).resolve().parents[1]
ROOT = HOST.parents[1]
sys.path.insert(0, str(ROOT / "MQTT"))
import is02_wire as lw  # noqa: E402

BUILD = HOST / "build"


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


def test_dump_tables_match_python(dump):
    assert dump["format_version"] == 1
    assert [(e["name"], e["arg_bits"] if not e["variable"] else None) for e in dump["events"]] == \
        [(n, b) for n, b in lw.EVENTS]
    # 26 = rejected Parameter_ID names (built at run time); the rest are fixed texts
    texts = [r["text"] for r in dump["reasons"]]
    assert texts[:26] + texts[27:] == lw.REASONS[:26] + lw.REASONS[27:]
    assert [p["name"] for p in dump["params"]] == lw.PARAM_NAMES
    assert {p["id"]: p["store_width"] for p in dump["params"] if p["store_width"] != 32} == \
        {k: v[0] for k, v in lw.PARAM_STORE.items()}
    assert dump["param_table_crc"] == lw.PARAM_TABLE_CRC
    assert [s["text"] for s in dump["stop_reasons"]] == lw.STOP_TEXTS
    assert [a["text"] for a in dump["abort_reasons"]] == lw.ABORT_TEXTS


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


def test_vectors_round_trip_and_python_agrees(tools):
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
            assert lw.decode("is02/telemetry", data)["seq"] == f[0]
            assert bits_at(data, 369, 2) == f[29] and bits_at(data, 417, 8) == f[34]
        elif k == "heartbeat":
            h = lw.decode("is02/status", data)
            assert (h["seq"], h["n_params"], h["param_crc"]) == (f[0], f[10], f[11])
        elif k == "ack":
            assert lw.decode("is02/ack", data)["cmd"] == f"PING:{f[3]}"


def test_vectors_event_and_rejects(tools):
    res = run_vectors(tools, ["event 777 15 4 10",                 # EMERGENCY_STOP, stop reason 1
                              "event 5 0 3 40",                    # STATE 2
                              "event 5 0 4 40"])                   # wrong bit count -> refused
    assert res[0]["hex"] and res[1]["hex"] and res[2]["hex"] == ""
    assert lw.decode("is02/event", bytes.fromhex(res[0]["hex"]))["detail"] == "remote E-stop (MQTT)"
    d = run_vectors(tools, [f"decode event {res[0]['hex']}", "decode cmd 41", "decode telemetry 10"])
    assert d[0]["status"] == 0 and d[0]["fields"] == [777, 15, 4]
    assert d[1]["status"] == 1                                     # 'A' = text: unknown version
    assert d[2]["status"] == 3                                     # short telemetry: bad length
