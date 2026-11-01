"""Properties 24, 29 and 30 (spec tasks 14.10, 14.15, 14.16): the parameter table
version, the tool inputs, and unknown codes, for the Python Laptop_Codec and the
C Wire_Codec. The JavaScript halves are in js/params.test.js.
"""

import ctypes
import random
import string

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

import codec_gen as gen
import wire_spec
from codec_helpers import is02_codec as codec
from conftest import bind, load_is02

SEEDS = st.integers(0, 2 ** 32 - 1)

T24 = "The parameter table version detects table edits"
T29 = "Tool inputs parse to Parameter_IDs or report the bad line"
T30 = "Unknown codes are shown numerically"

NAME = st.text(alphabet=string.ascii_uppercase + string.digits + "_", min_size=1, max_size=24)


# ---------------- Property 24 ----------------

@pytest.fixture(scope="module")
def c_table():
    """(count, CRC) of the firmware's Parameter_Store table (params_table_crc, S2)."""
    lib = load_is02()
    bind(lib, "params_init", None)
    bind(lib, "params_table_crc", ctypes.c_uint16)
    bind(lib, "params_count", ctypes.c_uint32)
    lib.params_init()
    return lib.params_count(), lib.params_table_crc()


def edits(names, rng):
    """Single edits of a list: one added name, one removed, two different names swapped."""
    extra = "NEW_" + "".join(rng.choice(string.ascii_uppercase) for _ in range(6))
    added = names[:]
    added.insert(rng.randint(0, len(names)), extra)
    out = [added]
    if len(names) > 1:
        i = rng.randrange(len(names))
        out.append(names[:i] + names[i + 1:])
    pairs = [(i, j) for i in range(len(names)) for j in range(i + 1, len(names)) if names[i] != names[j]]
    if pairs:
        i, j = rng.choice(pairs)
        swapped = names[:]
        swapped[i], swapped[j] = swapped[j], swapped[i]
        out.append(swapped)
    return out


# Feature: minimum-bit-data-representation, Property 24: The parameter table version detects table edits
@pytest.mark.prop(24, T24, unit="parameter table")
@settings(max_examples=1000)
@given(names=st.lists(NAME, min_size=1, max_size=60, unique=True), seed=SEEDS)
def test_p24_table_version(c_table, names, seed):
    real = [p[0] for p in codec.PARAMS]
    assert c_table == codec.param_table_version(real) == (len(real), wire_spec.table_crc(real)), "real table"
    version = codec.param_table_version(names)
    assert version == (len(names), wire_spec.table_crc(names))
    for edited in edits(names, random.Random(seed)):
        assert codec.param_table_version(edited) != version, f"{names} -> {edited}"


# ---------------- Property 29 ----------------

def value_text(rng, kind, low, high):
    if kind != "F32":
        return str(rng.randint(int(low), int(high)))
    x = rng.choice([low, high, low + (high - low) * rng.random()])
    return repr(min(max(round(x, 4), low), high))


def valid_lines(rng):
    """(lines, expected pairs): NAME=VALUE lines with blank lines and comments between them."""
    lines, pairs = [], []
    for _ in range(rng.randint(0, 20)):
        for filler in rng.choice([[], [""], ["# a comment"], ["", "  # note: A=1"]]):
            lines.append(filler)
        name, kind, low, high = codec.PARAMS[rng.randrange(codec.PARAM_COUNT)]
        text = value_text(rng, kind, low, high)
        lines.append(rng.choice(["{}={}", "{} = {}", "{}={}  # kept"]).format(name, text))
        pairs.append([codec.PARAM_IDS[name], codec.f32_to_bits(float(text))])
    return lines, pairs


def bad_line(rng):
    name, kind, low, high = codec.PARAMS[rng.randrange(codec.PARAM_COUNT)]
    return rng.choice([
        ("NOT_A_PARAM", "NOT_A_PARAM=1"),
        (name, f"{name}=abc"),
        (name, f"{name}={high + 1:g}"),
        (name, f"{name}={low - 1:g}"),
    ])


# Feature: minimum-bit-data-representation, Property 29: Tool inputs parse to Parameter_IDs or report the bad line
@pytest.mark.prop(29, T29, unit="robot_params.txt")
@settings(max_examples=500)
@given(seed=SEEDS)
def test_p29_valid_content(seed):
    lines, pairs = valid_lines(random.Random(seed))
    assert codec.parse_config(lines) == pairs
    data = codec.encode_config("\n".join(lines))
    status, fields, _ = gen.c_decode("config", data)
    assert status == 0 and fields == [len(pairs)] + [x for p in pairs for x in p]


# Feature: minimum-bit-data-representation, Property 29: Tool inputs parse to Parameter_IDs or report the bad line
@pytest.mark.prop(29, T29, unit="robot_params.txt with a bad line")
@settings(max_examples=500)
@given(seed=SEEDS)
def test_p29_bad_line(seed):
    rng = random.Random(seed)
    lines, _ = valid_lines(rng)
    name, text = bad_line(rng)
    at = rng.randint(0, len(lines))
    lines.insert(at, text)
    with pytest.raises(codec.CodecError) as caught:
        codec.encode_config("\n".join(lines))
    assert [e[:2] for e in caught.value.errors] == [(at + 1, name)], caught.value.errors


# ---------------- Property 30 ----------------

def header_bits(*fields):
    """Version 1 then the (value, width) fields, as bytes padded with zeros."""
    bits = format(codec.FORMAT_VERSION, "04b") + "".join(format(v, f"0{w}b") for v, w in fields)
    bits += "0" * (-len(bits) % 8)
    return int(bits, 2).to_bytes(len(bits) // 8, "big")


# Feature: minimum-bit-data-representation, Property 30: Unknown codes are shown numerically
@pytest.mark.prop(30, T30, unit="event")
@settings(max_examples=200)
@given(code=st.integers(len(codec.EVENTS), 63), t=st.integers(0, 2 ** 24 - 1))
def test_p30_unknown_event_code(code, t):
    logs = []
    shown = codec.decode("is02/event", header_bits((t, 24), (code, 6)), log=logs.append)
    assert shown is not None and shown["event"] == code and shown["t"] == t
    assert any(str(code) in line for line in logs)


# Feature: minimum-bit-data-representation, Property 30: Unknown codes are shown numerically
@pytest.mark.prop(30, T30, unit="ack reason")
@settings(max_examples=200)
@given(reason=st.integers(len(codec.REASONS), 31), cmd=st.sampled_from([1, 2, 3, codec.WCMD_UNPARSED]),
       t=st.integers(0, 2 ** 24 - 1))
def test_p30_unknown_reason_code(reason, cmd, t):
    logs = []
    shown = codec.decode("is02/ack", header_bits((t, 24), (cmd, 4), (reason, 5)), log=logs.append)
    assert shown is not None and shown["result"] == reason
    assert any(str(reason) in line for line in logs)


# Feature: minimum-bit-data-representation, Property 30: Unknown codes are shown numerically
@pytest.mark.prop(30, T30, unit="ack command")
@settings(max_examples=200)
@given(cmd=st.integers(len(codec.COMMANDS), 14), reason=st.integers(0, len(codec.REASONS) - 1),
       t=st.integers(0, 2 ** 24 - 1))
def test_p30_unknown_command_code(cmd, reason, t):
    logs = []
    shown = codec.decode("is02/ack", header_bits((t, 24), (cmd, 4), (reason, 5)), log=logs.append)
    assert shown is not None and shown["cmd"] == cmd
    assert any(str(cmd) in line for line in logs)
