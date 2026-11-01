"""Firmware-track test helper: the protocol tables as the C side defines
them (build/wire_dump JSON), with no dependency on the laptop codecs in
MQTT/ (owned by the laptop track). Used by test_status_snapshot,
test_spec_conformance, test_wire_tools and test_prop19.
"""

import json
import subprocess
from functools import lru_cache
from pathlib import Path

HOST = Path(__file__).resolve().parents[1]
BUILD = HOST / "build"


@lru_cache(maxsize=1)
def tools():
    subprocess.run(["make", "-s", "-C", str(HOST), "tools"], check=True, capture_output=True)
    return BUILD


@lru_cache(maxsize=1)
def dump():
    out = subprocess.run([str(tools() / "wire_dump")], check=True, capture_output=True, text=True).stdout
    return json.loads(out)


def event_code(name):
    return next(e["code"] for e in dump()["events"] if e["name"] == name)


def events():
    """[(name, fixed_arg_bits or None for variable), ...] in code order."""
    return [(e["name"], None if e["variable"] else e["arg_bits"]) for e in dump()["events"]]


def param_names():
    return [p["name"] for p in dump()["params"]]


def run_vectors(lines):
    out = subprocess.run([str(tools() / "wire_vectors")], input="\n".join(lines) + "\n", check=True,
                         capture_output=True, text=True).stdout
    return [json.loads(x) for x in out.splitlines()]


def crc16_ccitt_false(data, crc=0xFFFF):
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc


def table_crc(names):
    """Parameter table version (design D15): CRC over every NAME + '\\n'."""
    return crc16_ccitt_false("".join(n + "\n" for n in names).encode("ascii"))


def rand_event_fields(code, rng):
    """Random valid argument fields [(value, width)] for Event_Code code (D10)."""
    name, fixed = events()[code]
    if name == "FINE_SCAN":
        n = rng.randint(1, 11)
        return [(rng.randint(-60, 60), 7), (n, 4)] + [(rng.choice([511, rng.randint(0, 431)]), 9) for _ in range(n)]
    if name == "BC_DBG":
        v = rng.choice([0, 1, 2, 3, 4])
        if v == 0:
            return [(0, 3)]
        win = [(rng.randint(0, 96), 7), (rng.randint(0, 3), 2), (rng.choice([63, rng.randint(0, 42)]), 6),
               (rng.randint(0, 1), 1), (rng.randint(0, 4095), 12), (rng.randint(0, 4095), 12),
               (rng.randint(0, 65535), 16)]
        if v == 1:
            return [(1, 3)] + win
        if v == 2:
            return [(2, 3)] + win + [(rng.randint(0, 5), 3), (rng.randint(0, 1), 1)]
        if v == 3:
            return [(3, 3), (rng.randint(1, 6), 3)] + [x for _ in range(6) for x in
                                                       ((rng.randint(0, 3), 2), (rng.choice([63, rng.randint(0, 42)]), 6),
                                                        (rng.randint(0, 96), 7))]
        k = rng.randint(0, 16)
        return [(4, 3), (rng.randint(1, 6), 3), (k, 5)] + [(rng.randint(0, 4095), 12) for _ in range(k)]
    return [(rng.randint(0, 1), 1) for _ in range(fixed or 0)]
