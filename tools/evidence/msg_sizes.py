#!/usr/bin/env python3
"""MQTT payload sizes per topic (Requirement 23.2, 23.5, 23.6).

Reads one or more recorded MQTT logs of one build and prints a Markdown
table with, for each is02 topic, the number of payloads, the minimum, the
arithmetic mean (to 0.1 byte) and the maximum MQTT application payload size
in bytes (the payload only: no MQTT fixed header, topic name or packet id).

Accepted log formats (detected from the first line):

  new     `python is02_tool.py log run.csv` of the Revised_Build:
          CSV with header  pc_time,topic,len,hex,decoded_json
          The size is the byte length of `hex`; `len` is checked against it.
  csv     `python is02_tool.py log run.csv` of the Baseline_Build:
          CSV with header  pc_time,topic,payload
          The size is the UTF-8 length of `payload` (the JSON text).
  monitor `telemetry_monitor.py` messages_<stamp>.log of the Baseline_Build:
          lines "<ISO time> <topic> <payload text>". This log has no
          is02/telemetry payloads (they go to the telemetry CSV, decoded),
          and is02/cmd may be logged twice (when sent and when echoed by the
          broker), so prefer an is02_tool.py log for size evidence.

Usage:
  python tools/evidence/msg_sizes.py run.csv [more.csv ...] [--commit HASH] [--label TEXT]

Exit status: 0 table printed; 1 a `new` row had a `len` that differs from its
`hex` length (the table uses the hex length); 2 usage or format error.
"""
import argparse
import csv
import re
import sys
from decimal import ROUND_HALF_UP, Decimal

TOPICS = ["is02/telemetry", "is02/status", "is02/event", "is02/ack", "is02/cmd", "is02/map", "is02/config"]

MONITOR_LINE = re.compile(r"^(\d{4}-\d\d-\d\dT\S+) (is02/\S+)(?: (.*))?$")


class LogError(Exception):
    pass


def read_new(rows, path):
    """Rows of a pc_time,topic,len,hex,decoded_json log -> [(topic, size)], len mismatches."""
    out, bad = [], 0
    for n, row in enumerate(rows, start=2):
        if not row.get("topic"):
            continue
        hex_text = (row.get("hex") or "").strip()
        try:
            size = len(bytes.fromhex(hex_text))
        except ValueError as e:
            raise LogError(f"{path} data row {n - 1}: bad hex column: {e}") from None
        stated = (row.get("len") or "").strip()
        if stated and not stated.isdigit():
            raise LogError(f"{path} data row {n - 1}: bad len column: {stated!r}")
        if stated and int(stated) != size:
            print(f"msg_sizes: {path} data row {n - 1}: len={stated} but hex has {size} bytes; using {size}", file=sys.stderr)
            bad += 1
        out.append((row["topic"], size))
    return out, bad


def read_baseline_csv(rows):
    return [(row["topic"], len((row.get("payload") or "").encode("utf-8"))) for row in rows if row.get("topic")]


def read_monitor(f):
    out = []
    for line in f:
        m = MONITOR_LINE.match(line.rstrip("\r\n"))
        if m:
            out.append((m.group(2), len((m.group(3) or "").encode("utf-8"))))
    return out


def read_log(path):
    """Returns (format name, [(topic, size)], len mismatch count)."""
    with open(path, newline="", encoding="utf-8", errors="replace") as f:
        first = f.readline()
        f.seek(0)
        if first.lstrip("\ufeff").startswith("pc_time"):
            reader = csv.DictReader(f)
            cols = [c.strip().lstrip("\ufeff") for c in (reader.fieldnames or [])]
            reader.fieldnames = cols
            if {"topic", "len", "hex"} <= set(cols):
                sizes, bad = read_new(reader, path)
                return "new (pc_time, topic, len, hex, decoded_json)", sizes, bad
            if {"topic", "payload"} <= set(cols):
                return "csv (pc_time, topic, payload)", read_baseline_csv(reader), 0
            raise LogError(f"{path}: unknown CSV columns {cols}")
        if MONITOR_LINE.match(first.rstrip("\r\n")):
            return "monitor (telemetry_monitor.py messages log)", read_monitor(f), 0
        raise LogError(f"{path}: not an is02_tool.py log or a telemetry_monitor.py messages log")


def mean_1dp(total, count):
    return (Decimal(total) / Decimal(count)).quantize(Decimal("0.1"), rounding=ROUND_HALF_UP)


def main(argv=None):
    ap = argparse.ArgumentParser(description="MQTT payload sizes per topic, as a Markdown table.")
    ap.add_argument("logs", nargs="+", help="is02_tool.py log CSV or telemetry_monitor.py messages log")
    ap.add_argument("--commit", default="", help="git commit of the build that produced the log")
    ap.add_argument("--label", default="", help="heading text, for example 'Baseline_Build track run 1'")
    args = ap.parse_args(argv)

    sizes = {}
    formats = []
    bad = 0
    try:
        for path in args.logs:
            fmt, rows, b = read_log(path)
            formats.append((path, fmt, len(rows)))
            bad += b
            for topic, size in rows:
                sizes.setdefault(topic, []).append(size)
    except (OSError, LogError) as e:
        print(f"msg_sizes: {e}", file=sys.stderr)
        return 2

    title = "MQTT payload sizes" + (f": {args.label}" if args.label else "")
    print(f"# {title}\n")
    print("| Record | Value |")
    print("|---|---|")
    for path, fmt, n in formats:
        print(f"| Log | `{path}`: {fmt}, {n} payloads |")
    print(f"| Git commit of the build | {('`' + args.commit + '`') if args.commit else 'not given (pass --commit)'} |")
    print()
    print("Payload bytes only (no MQTT fixed header, topic name or packet identifier).\n")
    print("| Topic | Payloads | Min (bytes) | Mean (bytes) | Max (bytes) |")
    print("|---|---|---|---|---|")
    empty = []
    for topic in TOPICS + sorted(t for t in sizes if t not in TOPICS):
        v = sizes.get(topic, [])
        if v:
            print(f"| `{topic}` | {len(v)} | {min(v)} | {mean_1dp(sum(v), len(v))} | {max(v)} |")
        else:
            print(f"| `{topic}` | 0 | - | - | - |")
            empty.append(topic)
    if empty:
        print()
        print("No payloads in this log for: " + ", ".join(f"`{t}`" for t in empty) + ". "
              "Give their sizes from a separate recorded session of the same build that contains at least "
              "one payload of each message type on the topic, and state that the values come from that "
              "session (Requirement 23.6).")
    if any(fmt.startswith("monitor") for _, fmt, _ in formats):
        print()
        print("Note: a telemetry_monitor.py messages log has no `is02/telemetry` payloads and may count "
              "`is02/cmd` twice (sent and echoed).")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
