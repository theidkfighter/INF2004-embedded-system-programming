#!/usr/bin/env python3
"""Prints the Host_Test_Suite summary from the JSON-lines result files.

Called by run_all.sh. One line per property test (Requirement 18.8):

    P<n> <title> | <payload type or Storage_Unit> | cases=<n> | failed=<n> | seed=<s>

then, for every failing case (Requirement 18.9), the property, the payload
type or Storage_Unit, the input values and the seed. Exit status 1 if any
property or other test failed, 0 otherwise.

Usage: summary.py <seed> <results.jsonl> [<results.jsonl> ...]
"""

import json
import os
import sys
from pathlib import Path


def load(paths):
    records = []
    for p in paths:
        path = Path(p)
        if not path.is_file():
            continue
        for line in path.read_text(encoding="utf-8").splitlines():
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def prop_key(r):
    n = r.get("n")
    return (n if isinstance(n, int) else 1 << 30, str(r.get("title")), str(r.get("unit")), r.get("runner", ""))


def main(argv):
    if len(argv) < 2:
        print(__doc__.strip().splitlines()[-1], file=sys.stderr)
        return 2
    seed, paths = argv[0], argv[1:]
    records = load(paths)
    props = sorted((r for r in records if r.get("kind") == "prop" and not r.get("skipped")), key=prop_key)
    others = [r for r in records if r.get("kind") != "prop"]

    print("== Host_Test_Suite summary ==")
    if not props:
        print("(no property tests yet)")
    for r in props:
        print(f"P{r.get('n')} {r.get('title')} | {r.get('unit')} | cases={r.get('cases')} "
              f"| failed={r.get('failed')} | seed={r.get('seed')}")

    failing = [r for r in props if r.get("failed")]
    other_failed = [r for r in others if r.get("failed")]
    other_passed = [r for r in others if not r.get("failed") and not r.get("skipped")]
    other_skipped = [r for r in others if r.get("skipped")]
    print(f"other tests: passed={len(other_passed)} failed={len(other_failed)} skipped={len(other_skipped)}")

    for r in failing:
        print()
        print(f"FAILED property P{r.get('n')} {r.get('title')}")
        print(f"  payload type / Storage_Unit: {r.get('unit')}")
        print(f"  seed: {r.get('seed')}")
        print(f"  test: {r.get('runner')} {r.get('nodeid')}")
        print("  input:")
        for line in (r.get("input") or "(not reported)").splitlines():
            print(f"    {line}")
    for r in other_failed:
        print()
        print(f"FAILED test {r.get('runner')} {r.get('nodeid')} ({r.get('phase')})")
        if r.get("input"):
            for line in r["input"].splitlines():
                print(f"    {line}")

    n_fail = len(failing) + len(other_failed)
    print()
    print(f"properties={len(props)} failed={len(failing)} | other tests failed={len(other_failed)} | seed={seed}")
    if n_fail:
        # IS02_RERUN: extra run_all.sh options to repeat, e.g. "--selfcheck "
        print(f"re-run with: tests/host/run_all.sh {os.environ.get('IS02_RERUN', '')}--seed {seed}")
    return 1 if n_fail else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
