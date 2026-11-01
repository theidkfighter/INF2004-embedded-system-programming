#!/usr/bin/env python3
"""Telemetry_Frame budget from wire_dump JSON (spec task 3.11).

    make -C tests/host tools
    tests/host/build/wire_dump | python tools/evidence/frame_budget.py

Prints the sum of field widths (Required_Bits), the frame length in bytes
and the padding bits, for the telemetry and heartbeat frames.
"""
import json
import sys


def main():
    d = json.load(open(sys.argv[1]) if len(sys.argv) > 1 else sys.stdin)
    for name in ("telemetry", "heartbeat"):
        f = d["frames"][name]
        bits = sum(x["width"] for x in f["fields"] if x["kind"] != "PAD")
        length = (bits + 7) // 8
        print(f"{name}: {len(f['fields'])} fields, {bits} bits, {length} bytes, {length * 8 - bits} padding bits"
              + ("" if length == f["len"] else f"  (MISMATCH: wire_ids says {f['len']})"))


if __name__ == "__main__":
    main()
