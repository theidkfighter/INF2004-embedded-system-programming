"""Drift guard (INTEGRATION.md 1.4): the Python Laptop_Codec's tables against the
C side (build/wire_dump JSON). The JavaScript codec's tables are compared with
the same JSON in js/tables.test.js. A difference fails the test and names the
codec and the table.
"""

import pytest

import wire_spec
from codec_helpers import is02_codec as codec

CODEC = "Python Laptop_Codec (MQTT/is02_codec.py)"


@pytest.fixture(scope="module")
def dump():
    return wire_spec.dump()


def same(what, got, want):
    assert got == want, f"{CODEC}: {what} differs from wire_dump:\n  codec     {got}\n  wire_dump {want}"


def test_format_version(dump):
    same("format_version", codec.FORMAT_VERSION, dump["format_version"])


def test_event_table(dump):
    same("events", [list(e) for e in codec.EVENTS], [[e["name"], e["arg_bits"], e["variable"]] for e in dump["events"]])
    same("event codes", list(range(len(codec.EVENTS))), [e["code"] for e in dump["events"]])


def test_reason_table(dump):
    same("reasons", [list(r) for r in codec.REASONS], [[r["name"], r["text"]] for r in dump["reasons"]])
    same("RSN_CONFIG_REJECTED", codec.RSN_CONFIG_REJECTED, next(r["code"] for r in dump["reasons"] if r["name"] == "CONFIG_REJECTED"))


def test_command_table(dump):
    same("commands", [list(c) for c in codec.COMMANDS],
         [[c["name"], c["echo"], c["cmd_len"], c["ack_len"]] for c in dump["commands"]])


def test_stop_and_abort_texts(dump):
    same("stop reasons", codec.STOP_REASONS, [s["text"] for s in dump["stop_reasons"]])
    same("abort reasons", codec.ABORT_REASONS, [s["text"] for s in dump["abort_reasons"]])


def test_parameter_table(dump):
    same("parameter names and store widths", [(p[0], codec.PARAM_KINDS[p[1]]) for p in codec.PARAMS],
         [(p["name"], p["store_width"]) for p in dump["params"]])
    same("parameter table version", (codec.PARAM_COUNT, codec.PARAM_CRC), (len(dump["params"]), dump["param_table_crc"]))


def expand(layout):
    """[(width, bit offset)] of a layout's fields, version nibble first, array fields one by one."""
    out, pos = [], 4
    for field in layout:
        for _ in range(1 if len(field) == 2 else field[2]):
            out.append((abs(field[1]), pos))
            pos += abs(field[1])
    return out, pos


@pytest.mark.parametrize("name,layout,bits,length", [
    ("telemetry", codec.TELEMETRY, codec.TLM_BITS, codec.TLM_LEN),
    ("heartbeat", codec.HEARTBEAT, codec.HB_BITS, codec.HB_LEN),
    ("map", codec.MAP_BODY, 28, codec.MAP_LEN),
])
def test_frame_layouts(dump, name, layout, bits, length):
    frame = dump["frames"][name]
    fields = [(f["width"], f["offset"]) for f in frame["fields"][1:] if f["kind"] != "PAD"]
    got, end = expand(layout)
    same(f"{name} field widths and offsets", got, fields)
    same(f"{name} bits", (end, bits), (frame["bits"], frame["bits"]))
    same(f"{name} length", length, frame["len"])


def test_header_bits(dump):
    same("event header bits", codec.EVT_HDR_BITS, dump["frames"]["event_header"]["bits"])
    same("ack header bits", codec.ACK_HDR_BITS, dump["frames"]["ack_header"]["bits"])
    same("config header bits", 10, dump["frames"]["config"]["bits"])
