#!/usr/bin/env python3
"""Writes the generated cases that tests/host/js/cross.test.js replays against the
JavaScript Laptop_Codec (Properties 16, 25, 26 and 27).

    python3 cross_cases.py --seed N --out cases.json

The JavaScript codec cannot call the C Wire_Codec, so this script does it: every
case holds a record and the bytes, made by the C Wire_Codec (robot-to-laptop
frames, Property 25, and valid payloads, Property 27) or by the Python codec
and then decoded back by the C Wire_Codec (laptop-to-robot frames, Property 26;
a mismatch stops this script). The Python tests (test_codec_cross.py) draw their
own cases, so a JavaScript failure is a JavaScript failure.

Standard library only; needs build/libis02.so and build/wire_vectors.
"""

import argparse
import json
import os
import random
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

import codec_gen as gen  # noqa: E402
import event_ref  # noqa: E402
import wire_spec  # noqa: E402
from codec_helpers import is02_codec as codec  # noqa: E402

GOLDEN = HERE.parent / "vectors" / "event_golden.json"
PER_UNIT = 1000                 # telemetry, heartbeat, ack; per command code; map; config; valid payloads
PER_EVENT = 100


def c_encode_many(kind, recs):
    out = gen.c_lines([gen.line_of(kind, rec) for rec in recs])
    return [bytes.fromhex(o["hex"]) for o in out]


def check_c_decodes(kind, recs, datas):
    """The C Wire_Codec reads the bytes of the Python codec back to the fields of the records."""
    out = gen.c_lines([gen.decode_line(kind, d) for d in datas])
    for rec, data, res in zip(recs, datas, out):
        if res["status"] != 0 or res["fields"] != gen.flat(kind, rec):
            raise SystemExit(f"C decodes the Python {kind} frame {data.hex()} differently: {res} for {rec}")


def build(seed):
    rng = random.Random(seed)
    cases = {"seed": seed, "p25": {}, "p26": {}, "p27": {}, "p16": [], "golden": []}

    for kind in ("telemetry", "heartbeat", "ack"):
        recs = [gen.gen_record(kind, rng, gen.ACK_CODES[i % len(gen.ACK_CODES)] if kind == "ack" else None)
                for i in range(PER_UNIT)]
        cases["p25"][kind] = [{"rec": r, "hex": d.hex()} for r, d in zip(recs, c_encode_many(kind, recs))]
    events = {}
    for code in range(len(codec.EVENTS)):
        recs = [gen.gen_event(rng, code) for _ in range(PER_EVENT)]
        events[codec.EVENTS[code][0]] = [{"rec": r, "hex": d.hex()} for r, d in zip(recs, c_encode_many("event", recs))]
    cases["p25"]["event"] = events

    commands = {}
    for code in gen.COMMAND_CODES:
        recs = [gen.gen_command(rng, code) for _ in range(PER_UNIT)]
        datas = [codec.encode_fields("command", r) for r in recs]
        check_c_decodes("command", recs, datas)
        commands[str(code)] = [{"rec": r, "hex": d.hex()} for r, d in zip(recs, datas)]
    cases["p26"]["command"] = commands
    for kind in ("map", "config"):
        recs = [gen.gen_record(kind, rng) for _ in range(PER_UNIT)]
        datas = [codec.encode_fields(kind, r) for r in recs]
        check_c_decodes(kind, recs, datas)
        cases["p26"][kind] = [{"rec": r, "hex": d.hex()} for r, d in zip(recs, datas)]

    for kind in gen.KINDS:
        recs = [gen.gen_record(kind, rng) for _ in range(PER_UNIT)]
        cases["p27"][kind] = [d.hex() for d in c_encode_many(kind, recs)]

    for code in range(len(codec.EVENTS)):
        for _ in range(PER_EVENT):
            rec = gen.gen_event(rng, code)
            posted, data = event_ref.build_event(code, event_ref.build_ops(rec), rec["t"])
            if not posted:
                raise SystemExit(f"evt_post refused {rec}")
            cases["p16"].append({"event": codec.EVENTS[code][0], "hex": data.hex(), "text": event_ref.ref_text(rec)})

    for entry in json.loads(GOLDEN.read_text(encoding="utf-8"))["events"]:
        code = codec.EVC[entry["event"]]
        posted, data = event_ref.build_event(code, entry["build"], 1000)
        if not posted:
            raise SystemExit(f"evt_post refused the golden entry {entry}")
        cases["golden"].append({"event": entry["event"], "hex": data.hex(), "text": entry["text"]})
    return cases


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--seed", type=int, required=True)
    parser.add_argument("--out", required=True)
    args = parser.parse_args()
    subprocess.run(["make", "-s", "-C", str(wire_spec.HOST), "all"], check=True, capture_output=True)
    cases = build(args.seed)
    out = Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    tmp = out.with_name(out.name + f".{os.getpid()}.tmp")
    tmp.write_text(json.dumps(cases, separators=(",", ":")), encoding="utf-8")
    os.replace(tmp, out)


if __name__ == "__main__":
    main()
