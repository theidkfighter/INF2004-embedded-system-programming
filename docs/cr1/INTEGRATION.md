# Two tracks, one protocol: integration rules

CR1 is split into two tracks that can run at the same time, in different sessions, by different people or tools:

| Track | Steps | Guides | Ledger |
|---|---|---|---|
| **Firmware (FW)** | spec Steps 1 to 6, 8, 9 (robot side) and 10 | `steps/firmware/FW-*.md` | `PROGRESS_FW.md` |
| **Laptop (LT)** | spec Step 7 (codecs and tools), the tool half of Step 9, and the laptop tests | `steps/laptop/LT-*.md` | `PROGRESS_LT.md` |
| **Integration (INT)** | final cross-codec table check and the final checkpoint | `steps/INT-01_final.md` | both |

Splitting is safe because the two tracks touch different files and meet only at the binary protocol. That protocol is fixed in writing before either side codes against it: in `spec/design.md` D7 to D15 and in `app_program/wire_ids.h`, which is already committed. The rules below keep it that way.

## 1. The contract

1. **What it is:**
   - `spec/design.md` sections D7 to D15 (frame layouts, codes, parameter IDs);
   - `app_program/wire_ids.h` (Format_Version, event, reason, command, stop and abort codes, frame lengths). It already exists, from task 3.8.
2. **Who owns it:** the firmware track owns `wire_ids.h`. The laptop track reads it and copies its tables into `MQTT/is02_codec.py` and `MQTT/is02_codec.js`, and must not edit it.
3. **Changing it:** a contract change (any field, width, order, code or length) needs all of:
   - a new item in `REVIEW_FLAGS.md` with a team decision;
   - `WIRE_FORMAT_VERSION` + 1 in `wire_ids.h` (Requirement 16.4);
   - a line in both PROGRESS files saying what changed.

   The other track then updates its side before the next merge.
4. **Checking drift:** the drift check is automatic from LT-04 on. The laptop tests compare the Python and JS tables with the `wire_dump` JSON from the C side (task 3.11, done in FW-01b; merged with S1). A difference fails the host tests on whichever branch merges second.

## 2. File ownership

| Path | Owner | The other track may |
|---|---|---|
| `app_program/**` (including `wire_ids.h`, `wire.c/.h`) | FW | read only |
| `build_make/**`, `tools/evidence/build_check.sh`, `ram_report.sh`, `type_sizes.*` | FW | read only |
| `tests/host/Makefile`, `run_all.sh`, `conftest.py`, `stubs/`, `baseline/`, `tools/*.c`, `test_static.sh` | FW | read only. Ask FW (PROGRESS note) if a change is needed |
| `tests/host/py/test_*.py` for firmware properties (all except P16 and P24 to P30) | FW | read only |
| `MQTT/**` (dashboard, monitor, tools, new `is02_codec.py`, `is02_codec.js`, `README.md`) | LT | read only |
| `tests/host/py/test_codec_*.py`, `test_tools.py`, `test_presentation.py`, `py/make_vectors.py`, `py/codec_helpers.py` | LT | read only |
| `tests/host/js/*.test.js` (except `prop_helper.test.js`), `tests/host/vectors/` | LT | read only |
| `tools/evidence/msg_sizes.py`, `perf_report.py` | LT (if a change is needed) | read only |
| `docs/evidence/**` | FW writes the templates; the user fills in measurements | add a section only in your own heading |
| `docs/cr1/PROGRESS_FW.md` / `PROGRESS_LT.md` | FW / LT | read only |
| `docs/cr1/REVIEW_FLAGS.md` | either: **append only**, new item at the end | — |

`run_all.sh` already picks up every `py/test_*.py` and every `js/*.test.js` automatically, so the laptop track adds test files without touching shared runner files. LT code that needs the C library imports `conftest`'s loader; it doesn't edit `conftest.py`. Laptop tests find the codec with `sys.path.insert(0, <repo>/MQTT)` in `py/codec_helpers.py`.

## 3. Sync points

| Id | Condition | Blocks |
|---|---|---|
| **S1** | FW-01c merged into `Poh-Loon` (spec Step 1 complete): `wire.c` encodes and decodes every frame, `wire_dump` and the `wire_vectors` harness exist, and the event builders `evt.c` exist | LT-04 (cross-codec vectors and properties) |
| **S2** | FW-05b merged: `params_table_crc` exists | LT-04 task 14.10 only (Property 24). The rest of LT-04 can go ahead; 14.10 waits |
| **S3** | LT-03 merged: all five tools use the codec, `--legacy-json` on, dashboard loads `is02_codec.js` | **Flashing** any FW-08 build on the car (the robot then sends binary; old tools can't read it). Coding FW-08 isn't blocked |
| **S4** | FW-09 merged | LT-05 (tools send binary by default) |
| **S5** | LT-05 merged | **Flashing** an FW-09 build for the checkpoint 19 bench check, and INT-01 |
| **S6** | All FW and LT steps merged | INT-01 |

The laptop track can start **now**, in parallel with FW-01a: LT-01, LT-02 and LT-03 need only the contract, which is already written.

## 4. Branches and merging

1. Each track works on its branch: `cr1/fw` or `cr1/laptop`.
2. At the end of each step, the track's gate passes on its branch, then the user merges the branch into `Poh-Loon` (pull request or `git merge --no-ff`).
3. Before starting a step, merge `Poh-Loon` back into your track branch (`git merge origin/Poh-Loon`), so you have the other track's latest work and the contract.
4. **Expected conflicts:** none in code, because files don't overlap. If `git merge` reports one, stop and record it; don't resolve it by editing the other track's files.
5. After every merge into `Poh-Loon`, run the **full** gate there: firmware build + all host tests. That's the integration test.

## 5. Known integration risks and how they're handled

| Risk | Handling |
|---|---|
| Tables drift (a code added on one side only) | `wire_ids.h` is the single source; LT-04's table test against `wire_dump` fails on drift; Format_Version rule (1.3) |
| The firmware switches to binary before the tools can read it | S3: no FW-08 build is flashed before LT-03 is merged; until then the tools stay on JSON |
| Tools send binary commands to a firmware that only accepts text | In LT-03, `--legacy-json` (default on) keeps commands as **text**. LT-05 switches to binary only after FW-09 (S4). Spec ambiguity recorded as review flag R7 |
| Both tracks edit the shared test runner | Ownership table: FW owns the runner; LT only adds files the runner picks up automatically |
| Ledger conflicts | One PROGRESS file per track |
| Event detail texts (Property 16) need both sides | 16.4 is in the laptop track (LT-04). It uses the C event builders and `wire_enc_event` from Step 1 (S1); it doesn't need FW-08a |
| Final document tables cover both codecs (20.8) | Moved to INT-01, after both tracks are merged |
