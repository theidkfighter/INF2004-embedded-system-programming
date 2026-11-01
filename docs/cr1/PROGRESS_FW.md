# Firmware track: progress ledger

**This is the single source of truth for the firmware track.**
- The tick marks in `spec/tasks.md`, and in Kiro's `.kiro/specs/.../tasks.md`, aren't kept up to date.
- Every session reads this file first and updates it last, in the same commit as the code.
- Kiro sessions also tick here.

Status values:
- `todo`;
- `present-unverified`: code exists, but the gate hasn't passed with it;
- `done`: the gate passed, with a commit;
- `car-open`: code done, car check pending;
- `car-ok`;
- `n/a`.

## Current state (1 Nov 2026, verified in a cloud workspace)

- **Commit:** `Poh-Loon` at `542368f`.
- **Firmware:** builds with **0 warnings** with `APP_PERF=0` and with `APP_PERF=1`. `tools/evidence/build_check.sh` prints PASS.
- **Host tests:** **fail to build.** `tests/host/Makefile` still lists `wire.c`, but `wire.c` / `wire.h` were reverted. That breaks `run_all.sh` for everything. FW-01a repairs it.
- **Stray files committed:** `tests/host/p3_full.log`, `tests/host/p3_ps.txt` (a process listing). FW-01a removes them.

## Ledger

| Guide | Task (spec/tasks.md) | Status | Commit | Note |
|---|---|---|---|---|
| (Step 0) | 1.1 build switches | done | 542368f | builds verified 1 Nov |
| (Step 0) | 1.2 build_check.sh | done | 542368f | PASS verified 1 Nov; `baseline_warnings.txt` (0) committed with these guides |
| (Step 0) | 1.3 perf probes | done | 542368f | `APP_PERF=1` builds, 0 warnings |
| (Step 0) | 1.4 host test scaffolding | present-unverified | 542368f | suite broken by `wire.c` (see above) |
| (Step 0) | 1.5 baseline differential harness | present-unverified | 542368f | |
| (Step 0) | 1.6 evidence scripts | done | 542368f | |
| (Step 0) | 1.7 evidence templates | done | 542368f | |
| (Step 0) | 1.8 static checks | present-unverified | 542368f | |
| (Step 0) | 1.9 selfcheck | present-unverified | 542368f | |
| FW-00 | checkpoint 2: baseline measurements on the car (`APP_PERF=1`) | todo | | **needs the car** |
| FW-01a | repair: Makefile `wire.c`, park the wire tests, remove stray logs | todo | | |
| FW-01a | 3.1 bitpack.h | present-unverified | 542368f | |
| FW-01a | 3.2 Property 1 | present-unverified | 542368f | `test_prop01_rounding.py` exists |
| FW-01a | 3.3 Property 2 + test_nan | present-unverified | 542368f | files exist |
| FW-01a | 3.4 Property 3 | present-unverified | 542368f | file exists |
| FW-01a | 3.5 crit.h, sat counter | present-unverified | 542368f | commit message: "present but unverified" |
| FW-01a | 3.6 Code 39 index helpers | done | 542368f | |
| FW-01a | 3.7 Property 9 | todo | | |
| FW-01a | 3.8 wire_ids.h | done | 542368f | the contract (see INTEGRATION.md) |
| FW-01b | 3.9 Wire_Codec robot-to-laptop | todo | | was reverted; `test_wire_basic.py`, `wire_ctypes.py`, `wire_export.c` remain from that attempt |
| FW-01b | 3.10 laptop-to-robot frames | todo | | |
| FW-01b | 3.11 wire_dump + frame_budget.py | todo | | |
| FW-01b | 3.12 Property 19 | todo | | |
| FW-01b | 3.13 Wire_Codec unit tests | todo | | |
| FW-01b | 14.5 (C half) `tests/host/tools/wire_vectors.c` + Makefile target | todo | | moved here from Step 7 (C side of the shared vectors) |
| FW-01c | 3.14 msg_codec | todo | | |
| FW-01c | 3.15 Property 10 | todo | | |
| FW-01c | 3.16 evt builders | todo | | |
| FW-01c | checkpoint 4 | todo | | merge = sync point **S1** |
| FW-02 | 5.1–5.4 Fault_Mask | todo | | decide flag R1 first |
| FW-03 | 6.1–6.8 Status_Snapshot | todo | | flag R5 |
| FW-03 | checkpoint 7 (bench) | todo | | **needs the car** |
| FW-04 | 8.1–8.8 shared scalars, messages | todo | | flags R2, R3 |
| FW-04 | checkpoint 9 (bench: turn, bypass) | todo | | **needs the car** |
| FW-05a | 10.1–10.7 controller contexts | todo | | |
| FW-05b | 10.8–10.12 Parameter_Store, map | todo | | merge = sync point **S2** |
| FW-05b | checkpoint 11 | todo | | |
| FW-06 | 12.1–12.6 ISR_Shared_Data | todo | | |
| FW-06 | checkpoint 13 (ISR timing) | todo | | **needs the car** |
| FW-08a | 16.1–16.3 binary events | todo | | 16.4 is in LT-04 |
| FW-08b | 16.5–16.7 acks, heartbeat | todo | | |
| FW-08c | 16.8–16.11 telemetry frame | todo | | |
| FW-08c | checkpoint 17 (dashboard) | todo | | **needs the car**; flash only after **S3** |
| FW-09 | 18.1–18.3, 18.5–18.7 binary commands | todo | | merge = sync point **S4** |
| FW-09 | 18.8 (optional) barcode context bools | todo | | flag R2 |
| FW-09 | checkpoint 19 (commands, ESTOP) | todo | | **needs the car**; flash only after **S5** |
| FW-10 | 20.1–20.7 docs and evidence | todo | | 20.8 is in INT-01 |
| INT-01 | 20.8, final checkpoint 21 | todo | | after **S6** |

## Car checks waiting

| Check | From | Done | Result |
|---|---|---|---|
| Baseline measurements (`APP_PERF=1`, `docs/evidence/README.md` procedures) | FW-00 | no | |
| S5 met: FW-09 builds may be flashed (LT-05 merged: the tools send binary by default). After flashing, run `robot_params.py` and send the map once so the retained `is02/config` and `is02/map` are binary | LT-05 | no | |

## Session log

Newest first. One entry per session: date, guide, tool (Claude chat / Kiro + model), commit, gate results (host totals, `arm-none-eabi-size` line), open issues, car checks for the user.

- **2026-11-01, guides written (Claude chat).** Verified on `542368f`:
  - firmware builds with 0 warnings (`APP_PERF=0`: text 398820, data 2808, bss 107972; `APP_PERF=1`: 400324 / 2808 / 108896);
  - `build_check.sh` PASS;
  - host suite fails to build (`wire.c` missing).
