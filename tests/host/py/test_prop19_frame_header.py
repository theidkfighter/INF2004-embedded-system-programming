# Feature: minimum-bit-data-representation, Property 19: Every encoded frame has the fixed header and length
"""Property 19: every frame the C Wire_Codec encodes starts with
Format_Version 1 in byte 0 bits 7..4, has length ceil(bits / 8) with all
padding bits 0, and its length matches the design (fixed frames: 54, 25, 4,
command lengths per code; events: ceil((34 + argument bits) / 8)).

**Validates: Requirements 11.4, 11.5, 12.7, 16.1**
"""

import ctypes
import random

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

from test_wire_tools import _rand_hb, _rand_tlm, run_vectors

TITLE = "Every encoded frame has the fixed header and length"
CMD_LEN = {0: 1, 1: 1, 2: 1, 3: 1, 4: 2, 5: 4, 6: 1, 7: 1, 8: 3, 9: 6, 10: 1}


@pytest.fixture(scope="module")
def tools():
    import subprocess
    from pathlib import Path
    host = Path(__file__).resolve().parents[1]
    subprocess.run(["make", "-s", "-C", str(host), "tools"], check=True, capture_output=True)
    return host / "build"


def _check(data, bits, length):
    assert len(data) == length
    assert data[0] >> 4 == 1
    pad = length * 8 - bits
    assert pad < 8
    assert (data[-1] & ((1 << pad) - 1)) == 0


@pytest.mark.prop(19, TITLE, unit="telemetry+heartbeat")
@settings(max_examples=50, deadline=None)
@given(seed=st.integers(0, 2 ** 32 - 1))
def test_fixed_frames(tools, seed):
    rng = random.Random(seed)
    t, h = _rand_tlm(rng), _rand_hb(rng)
    res = run_vectors(tools, ["telemetry " + " ".join(map(str, t)), "heartbeat " + " ".join(map(str, h))])
    _check(bytes.fromhex(res[0]["hex"]), 432, 54)
    _check(bytes.fromhex(res[1]["hex"]), 195, 25)


@pytest.mark.prop(19, TITLE, unit="cmd+map+config")
@settings(max_examples=50, deadline=None)
@given(code=st.sampled_from(sorted(CMD_LEN)), n=st.integers(0, 55), seed=st.integers(0, 2 ** 32 - 1))
def test_laptop_frames(tools, code, n, seed):
    rng = random.Random(seed)
    args = {4: [rng.randint(0, 25)], 8: [rng.randint(0, 65535)], 9: [rng.randint(0, 54), rng.randint(0, 2 ** 32 - 1)],
            5: [1, 0, 1, 1, 0, 0, 1, 3]}.get(code, [])
    cfg = [n] + [x for i in range(n) for x in (i, rng.randint(0, 2 ** 32 - 1))]
    res = run_vectors(tools, [f"cmd {code} " + " ".join(map(str, args)), "map 1 0 0 0 1 5 0 0",
                              "config " + " ".join(map(str, cfg))])
    cmd_bits = {4: 13, 5: 32, 8: 24, 9: 46}.get(code, 8)
    _check(bytes.fromhex(res[0]["hex"]), cmd_bits, CMD_LEN[code])
    _check(bytes.fromhex(res[1]["hex"]), 28, 4)
    _check(bytes.fromhex(res[2]["hex"]), 10 + 38 * n, (10 + 38 * n + 7) // 8)


@pytest.mark.prop(19, TITLE, unit="event")
@settings(max_examples=100, deadline=None)
@given(code=st.integers(0, 44), seed=st.integers(0, 2 ** 32 - 1))
def test_events(lib, code, seed):
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "MQTT"))
    import is02_wire as lw
    from test_laptop_codec import _rand_event_fields
    from wire_ctypes import bind_wire, enc_event, make_evt
    bind_wire(lib)
    fields = _rand_event_fields(code, random.Random(seed))
    bits = sum(w for _, w in fields)
    data = enc_event(lib, seed & 0xFFFFFF, make_evt(code, fields))
    _check(data, 34 + bits, (34 + bits + 7) // 8)
    assert lw.decode("is02/event", data)["event"] == lw.EVENTS[code][0]
