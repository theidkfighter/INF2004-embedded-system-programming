#!/usr/bin/env python3
"""Timing probe tables for the Evidence_Report (Requirements 20.1, 20.3, 20.4,
20.6, 24.4, 24.5, 24.6, 24.7).

Reads a USB console log of a build made with APP_PERF=1 and prints Markdown
tables of its `[perf]` lines with a pass or fail result per probe:

    [perf] <name> n=<count> max=<us> avg=<us> over=<count>
    [perf] tlm_bench n=1000 max=<us> avg=<us>

perf_report() prints the cumulative counters of every probe every 2 s, and
perf_reset() (called when a run starts) clears them. A drop in the n of any
probe therefore starts a new segment; the last line of each probe in a
segment is that segment's result. Pick the segments that are the track runs
with --runs (default: all segments).

Probe names (the perf.h probe ids without PERF_, lower case) and limits:

    isr_gpio, isr_timer       worst case <= max(110 % of Baseline_Build, Baseline_Build + 1 us),
                              n >= 10,000 (Req 20.3; needs --baseline)
    tsk_safety, tsk_line      response <= 10 ms and over = 0 (Req 20.1)
    tsk_motion, tsk_imu       response <= 20 ms and over = 0 (Req 20.1)
    tlm_encode                n >= 100 frames (Req 24.7)
    cs_dsp                    <= 10 us (Req 9.6)
    cs_irq                    <= 10 us, n >= 1,000 (Req 10.2)
    tlm_bench                 mean and max <= Baseline_Build, n >= 1,000 (Req 20.4; needs --baseline)

Usage:
  python tools/evidence/perf_report.py revised_console.log --baseline baseline_console.log [--runs 2,3,4]

Exit status: 0 every probe with a limit passed; 1 a probe failed or has too
few samples; 2 usage error or no [perf] lines.
"""
import argparse
import re
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional


@dataclass(frozen=True)
class Probe:
    label: str
    kind: str                 # isr | task | tlm | cs | bench
    limit_us: Optional[int]   # fixed limit, None if relative to the baseline or none
    min_n: int
    req: str


PROBES = {
    "isr_gpio":   Probe("`hal_gpio_isr`", "isr", None, 10000, "20.3"),
    "isr_timer":  Probe("`hal_timer_isr`", "isr", None, 10000, "20.3"),
    "tsk_safety": Probe("TSK_SAFETY_MON cycle", "task", 10000, 0, "20.1"),
    "tsk_line":   Probe("TSK_LINE_BARCODE cycle", "task", 10000, 0, "20.1"),
    "tsk_motion": Probe("TSK_MOTION_CTRL cycle", "task", 20000, 0, "20.1"),
    "tsk_imu":    Probe("TSK_TERRAIN_IMU cycle", "task", 20000, 0, "20.1"),
    "tlm_encode": Probe("Telemetry frame build", "tlm", None, 100, "24.7"),
    "cs_dsp":     Probe("Critical_Section, dispatch disabled", "cs", 10, 0, "9.6"),
    "cs_irq":     Probe("Critical_Section, interrupts masked", "cs", 10, 1000, "10.2"),
    "tlm_bench":  Probe("Telemetry encode benchmark", "bench", None, 1000, "20.4"),
}

ALIASES = {
    "gpio_isr": "isr_gpio", "hal_gpio_isr": "isr_gpio",
    "timer_isr": "isr_timer", "hal_timer_isr": "isr_timer",
    "safety": "tsk_safety", "tsk_safety_mon": "tsk_safety",
    "line": "tsk_line", "tsk_line_barcode": "tsk_line",
    "motion": "tsk_motion", "tsk_motion_ctrl": "tsk_motion",
    "imu": "tsk_imu", "tsk_terrain_imu": "tsk_imu",
    "tlm": "tlm_encode", "tlm_build": "tlm_encode",
    "dsp": "cs_dsp", "irq": "cs_irq",
}

PERF_LINE = re.compile(r"\[perf\]\s+(\S+)((?:\s+\w+=[-+]?\d+(?:\.\d+)?)+)")
KEY_VALUE = re.compile(r"(\w+)=([-+]?\d+(?:\.\d+)?)")


@dataclass
class Stat:
    n: int
    max_us: float
    avg_us: float
    over: Optional[int]
    line: int


@dataclass
class Segment:
    first_line: int
    last_line: int
    probes: Dict[str, Stat]


@dataclass
class Log:
    path: str
    lines: int
    segments: List[Segment]
    bench: Optional[Stat]


def canonical(name):
    n = name.lower()
    if n.startswith("perf_"):
        n = n[5:]
    return ALIASES.get(n, n)


def parse(path):
    segments, current, last_n = [], {}, {}
    bench, count, first, last = None, 0, 0, 0
    with open(path, encoding="utf-8", errors="replace") as f:
        for no, text in enumerate(f, start=1):
            m = PERF_LINE.search(text)
            if not m:
                continue
            kv = {k: float(v) for k, v in KEY_VALUE.findall(m.group(2))}
            if not {"n", "max", "avg"} <= kv.keys():
                continue
            count += 1
            name = canonical(m.group(1))
            st = Stat(int(kv["n"]), kv["max"], kv["avg"], int(kv["over"]) if "over" in kv else None, no)
            if name == "tlm_bench":
                bench = st
                continue
            if name in last_n and st.n < last_n[name]:
                segments.append(Segment(first, last, current))
                current, last_n = {}, {}
            if not current:
                first = no
            current[name] = st
            last_n[name] = st.n
            last = no
    if current:
        segments.append(Segment(first, last, current))
    return Log(path, count, segments, bench)


def combine(segments):
    out = {}
    for seg in segments:
        for name, s in seg.probes.items():
            c = out.get(name)
            if c is None:
                out[name] = Stat(s.n, s.max_us, s.avg_us, s.over, s.line)
                continue
            total = c.n + s.n
            avg = (c.avg_us * c.n + s.avg_us * s.n) / total if total else 0.0
            over = None if c.over is None and s.over is None else (c.over or 0) + (s.over or 0)
            out[name] = Stat(total, max(c.max_us, s.max_us), avg, over, s.line)
    return out


def select(log, spec, what):
    if not log.segments:
        return [], []
    if not spec:
        idx = list(range(1, len(log.segments) + 1))
    else:
        try:
            idx = [int(x) for x in spec.split(",") if x.strip()]
        except ValueError:
            print(f"perf_report: {what}: segment list must be numbers like 2,3,4", file=sys.stderr)
            raise SystemExit(2)
        bad = [i for i in idx if not 1 <= i <= len(log.segments)]
        if bad:
            print(f"perf_report: {what}: no segment {bad[0]} (the log has {len(log.segments)})", file=sys.stderr)
            raise SystemExit(2)
    return idx, [log.segments[i - 1] for i in idx]


def fmt_us(v):
    return f"{v:,.0f}" if float(v).is_integer() else f"{v:,.1f}"


def evaluate(name, s, base, base_bench):
    """Returns (limit text, result, failure text or None)."""
    p = PROBES.get(name)
    if p is None:
        return "-", "N/A", None
    few = s.n < p.min_n
    few_text = f"n = {s.n:,} is below the {p.min_n:,} samples Requirement {p.req} needs"
    if p.kind == "task":
        limit = f"<= {p.limit_us:,} us, over = 0 (Req {p.req})"
        over = s.over or 0
        if s.max_us > p.limit_us or over > 0:
            return limit, "FAIL", f"max {fmt_us(s.max_us)} us, over = {over}; limit {p.limit_us:,} us with over = 0"
        return limit, "PASS", None
    if p.kind == "cs":
        limit = f"<= {p.limit_us} us" + (f", n >= {p.min_n:,}" if p.min_n else "") + f" (Req {p.req})"
        if s.max_us > p.limit_us:
            return limit, "FAIL", f"max {fmt_us(s.max_us)} us; limit {p.limit_us} us"
        return limit, ("INCOMPLETE" if few else "PASS"), (few_text if few else None)
    if p.kind == "tlm":
        limit = f"n >= {p.min_n} frames (Req {p.req})"
        return limit, ("INCOMPLETE" if few else "PASS"), (few_text if few else None)
    if p.kind == "isr":
        b = base.get(name) if base else None
        if b is None:
            return f"needs --baseline (Req {p.req})", "N/A", None
        lim = max(1.10 * b.max_us, b.max_us + 1.0)
        limit = f"<= {fmt_us(lim)} us = max(110 % of {fmt_us(b.max_us)}, {fmt_us(b.max_us)} + 1), n >= {p.min_n:,} (Req {p.req})"
        if s.max_us > lim:
            return limit, "FAIL", f"max {fmt_us(s.max_us)} us; limit {fmt_us(lim)} us"
        return limit, ("INCOMPLETE" if few else "PASS"), (few_text if few else None)
    if p.kind == "bench":
        if base_bench is None:
            return f"needs --baseline (Req {p.req})", "N/A", None
        limit = (f"avg <= {fmt_us(base_bench.avg_us)} us and max <= {fmt_us(base_bench.max_us)} us "
                 f"(Baseline_Build), n >= {p.min_n:,} (Req {p.req})")
        if s.avg_us > base_bench.avg_us or s.max_us > base_bench.max_us:
            return limit, "FAIL", (f"avg {fmt_us(s.avg_us)} us, max {fmt_us(s.max_us)} us; "
                                   f"limit avg {fmt_us(base_bench.avg_us)} us, max {fmt_us(base_bench.max_us)} us")
        return limit, ("INCOMPLETE" if few else "PASS"), (few_text if few else None)
    return "-", "N/A", None


def order(names):
    known = [n for n in PROBES if n in names]
    return known + sorted(n for n in names if n not in PROBES)


def table(stats, base, base_bench, failures, where):
    print("| Probe | n | max (us) | avg (us) | over | Limit | Result |")
    print("|---|---|---|---|---|---|---|")
    for name in order(stats):
        s = stats[name]
        limit, result, why = evaluate(name, s, base, base_bench)
        label = PROBES[name].label if name in PROBES else f"`{name}` (unknown probe)"
        over = "-" if s.over is None else f"{s.over:,}"
        print(f"| {label} (`{name}`) | {s.n:,} | {fmt_us(s.max_us)} | {s.avg_us:,.1f} | {over} | {limit} | {result} |")
        if why:
            failures.append((result, f"{where}, {label}", why, PROBES[name].req))


def main(argv=None):
    ap = argparse.ArgumentParser(description="Markdown tables of the [perf] console lines.")
    ap.add_argument("log", help="USB console log of an APP_PERF=1 build")
    ap.add_argument("--baseline", help="console log of the Baseline_Build (for the ISR and tlm_bench limits)")
    ap.add_argument("--runs", default="", help="segments of LOG to use, e.g. 2,3,4 (default: all)")
    ap.add_argument("--baseline-runs", default="", help="segments of the baseline log to use (default: all)")
    ap.add_argument("--label", default="", help="heading text, for example 'Revised_Build'")
    args = ap.parse_args(argv)

    try:
        log = parse(args.log)
        blog = parse(args.baseline) if args.baseline else None
    except OSError as e:
        print(f"perf_report: {e}", file=sys.stderr)
        return 2
    if log.lines == 0:
        print(f"perf_report: no [perf] lines in {args.log} (was the build made with APP_PERF=1?)", file=sys.stderr)
        return 2
    if blog is not None and blog.lines == 0:
        print(f"perf_report: no [perf] lines in {args.baseline}", file=sys.stderr)
        return 2

    idx, used = select(log, args.runs, "--runs")
    stats = combine(used)
    base, base_bench, bidx = None, None, []
    if blog is not None:
        bidx, bused = select(blog, args.baseline_runs, "--baseline-runs")
        base, base_bench = combine(bused), blog.bench

    failures = []
    title = "Timing probes" + (f": {args.label}" if args.label else "")
    print(f"# {title}\n")
    print("| Record | Value |")
    print("|---|---|")
    print(f"| Console log | `{args.log}` ({log.lines} `[perf]` lines) |")
    print(f"| Segments found (a drop in n starts a new one) | {len(log.segments)}: "
          + ", ".join(f"{i} = lines {s.first_line}-{s.last_line}" for i, s in enumerate(log.segments, 1)) + " |")
    print(f"| Segments used | {', '.join(map(str, idx)) or '-'} |")
    if blog is not None:
        print(f"| Baseline_Build console log | `{args.baseline}` ({blog.lines} `[perf]` lines), "
              f"segments used: {', '.join(map(str, bidx)) or '-'} |")
    print()

    if used:
        for i, seg in zip(idx, used):
            print(f"## Segment {i} (lines {seg.first_line}-{seg.last_line})\n")
            table(seg.probes, base, base_bench, failures, f"segment {i}")
            print()
        if len(used) > 1:
            print(f"## Combined (segments {', '.join(map(str, idx))})\n")
            print("n and over are summed, max is the largest, avg is weighted by n.\n")
            table(stats, base, base_bench, failures, "combined")
            print()

    print("## Telemetry encode benchmark (Requirement 20.4)\n")
    if log.bench is None:
        print("No `[perf] tlm_bench` line in this log.\n")
    else:
        table({"tlm_bench": log.bench}, base, base_bench, failures, "benchmark")
        print()

    if base is not None:
        print("## ISR worst case against the Baseline_Build (Requirement 24.6)\n")
        print("| ISR | Baseline_Build max (us) | Baseline_Build n | This build max (us) | This build n | % of Baseline_Build |")
        print("|---|---|---|---|---|---|")
        for name in ("isr_gpio", "isr_timer"):
            b, s = base.get(name), stats.get(name)
            if b is None or s is None:
                print(f"| {PROBES[name].label} | {'-' if b is None else fmt_us(b.max_us)} | {'-' if b is None else f'{b.n:,}'} "
                      f"| {'-' if s is None else fmt_us(s.max_us)} | {'-' if s is None else f'{s.n:,}'} | - |")
                continue
            pct = f"{100.0 * s.max_us / b.max_us:.1f} %" if b.max_us else "-"
            print(f"| {PROBES[name].label} | {fmt_us(b.max_us)} | {b.n:,} | {fmt_us(s.max_us)} | {s.n:,} | {pct} |")
        print()

    print("## Outside a limit (Requirement 24.5)\n")
    if not failures:
        print("None.")
    else:
        for result, where, why, req in failures:
            print(f"- {result}: {where}: {why} (Requirement {req}). Cause: _to be identified_.")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
