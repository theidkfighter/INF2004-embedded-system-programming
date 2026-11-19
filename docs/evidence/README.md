# Week 13 evidence: Minimum-Bit Data Representation

This folder holds the Evidence_Report for Change Requirement 1: the RAM usage comparison, the message size comparison, and the correctness and performance evidence. Each file is a template. The tables are filled from the output of the scripts named next to them. Hardware measurements are taken by the team on the robot.

## Builds compared

| Build | Commit | Branch | Notes |
|---|---|---|---|
| Baseline_Build | `bca79643d6f9bdd3938c353fa5f6fe791fc68f94` | `Poh-Loon` | HEAD before any feature code was committed (2026-11-01, "Merge remote-tracking branch 'origin/codex/ultrasonic-refinement' into Poh-Loon") |
| Baseline_Build with timing probes (Step 0 commit) | `542368fee64defa2c6a089db6a9769fc1819a36a` | `Poh-Loon` | Baseline code plus `perf.h` probes, build switches, test scaffolding and these templates |
| Revised_Build | `36a380bc3d8daf0f945e6c6853db1ab53914633e` (firmware; RAM, `sizeof`, host tests and build check were run on it) | `cr1/fw-rest` | The Week 13 documents that follow it change `docs/` and `tools/evidence/` only. Record the final submitted hash here if it differs |

Machine-readable form, used by `tests/host/baseline/fetch_baseline.sh` (via the `tests/host/Makefile` baseline target):

```
BASELINE_COMMIT=bca79643d6f9bdd3938c353fa5f6fe791fc68f94
```

### Which commit each comparison uses

| Comparison | Baseline side | Revised side | Why |
|---|---|---|---|
| RAM (`ram_usage.md`) | Baseline_Build commit above, `APP_PERF=0` | Revised_Build, `APP_PERF=0` | The Step 0 commit adds probe code. Measuring RAM on the parent commit keeps the baseline free of any change made for this feature |
| Message sizes (`message_sizes.md`) | Baseline_Build commit above, `APP_PERF=0` | Revised_Build, `APP_PERF=0` | Same reason as RAM |
| Timing (`performance.md`) | Step 0 commit, `APP_PERF=1` | Revised_Build, `APP_PERF=1` | Both builds must carry the same probes so they are measured the same way. With `APP_PERF=0` the Step 0 build is identical in behaviour to its parent |
| Track runs, L1 to L6, functional checklist, barcode, ultrasonic and fault checks (`correctness.md`) | Baseline_Build commit above, `APP_PERF=0`, laptop tools with `--legacy-json` (from Step 7 on; the unchanged tools read JSON directly) | Revised_Build, `APP_PERF=0` | Same track, barcode map and `robot_params.txt` for both builds |
| Encoder course at `SPEED_MAX_MM_S` (`correctness.md` 6.1) | Step 0 commit, `APP_PERF=1` | Revised_Build, `APP_PERF=1` | The same runs also give the critical-section maxima at `SPEED_MAX_MM_S` for `performance.md` |
| Concurrency self-test (`correctness.md`) | none | Revised_Build, `APP_SELFTEST=1` | Self-test build only. Never used for the RAM, size or track evidence |

## How each build is made

Build each measured commit after `make clean` in a working tree with no uncommitted changes (Requirement 22.7). A separate worktree keeps the main checkout untouched:

```sh
git worktree add ../is02_baseline bca79643d6f9bdd3938c353fa5f6fe791fc68f94
cd ../is02_baseline/build_make
git status --short          # must print nothing
make clean && make APP_ROBOT=1
```

Record the exact command lines here. Baseline and Revised builds must use the same command line and options for each comparison.

| Purpose | Directory | Command line |
|---|---|---|
| RAM and size, Baseline_Build | `build_make/` (worktree of `bca7964`) | `make clean && make APP_ROBOT=1 APP_PERF=0 -j8` (measured 2 Nov 2026) |
| RAM and size, Revised_Build | `build_make/` (worktree of `36a380b`) | `make clean && make APP_ROBOT=1 APP_PERF=0 -j8` (measured 2 Nov 2026) |
| Timing, Step 0 commit | `build_make/` | `make clean && make APP_ROBOT=1 APP_PERF=1` (planned, not run yet) |
| Timing, Revised_Build | `build_make/` | `make clean && make APP_ROBOT=1 APP_PERF=1` (planned, not run yet) |
| Track runs, Revised_Build | `build_make/` | `make clean && make APP_ROBOT=1` (planned, not run yet) |
| Self-test, Revised_Build | `build_make/` | `make clean && make APP_ROBOT=1 APP_SELFTEST=1` (planned, not run yet) |

The linked image is `build_make/mtkernel_3.elf`, with the map file `build_make/mtkernel_3.map` and the flashable `build_make/mtkernel_3.uf2`.

## Toolchain

Paste the first line of `arm-none-eabi-gcc --version` for each machine used. All measured builds must use the same version.

| Item | Value |
|---|---|
| `arm-none-eabi-gcc --version` | `arm-none-eabi-gcc (15:13.2.rel1-2) 13.2.1 20231009` |
| `arm-none-eabi-size --version` | `GNU size (2.42-1ubuntu1+23)` (Ubuntu 24.04 binutils for arm-none-eabi) |
| Host OS / shell used for the build | WSL Ubuntu 24.04.3 LTS, bash; Pico SDK 2.2.0 |
| Python version (host tests, scripts) | Python 3.12.3 (`~/is02venv`) |
| Node version (JS host tests) | v18.19.1 |

## Scripts that fill the tables

| Script | Fills | Run against |
|---|---|---|
| `tools/evidence/build_check.sh` | Build result and warning count (`correctness.md` section 1) | Each build |
| `tools/evidence/ram_report.sh <elf> <map>` | `ram_usage.md` | `mtkernel_3.elf` and `mtkernel_3.map` of each build |
| `tools/evidence/type_sizes.sh` | Target `sizeof` and pool block size (`message_sizes.md` section 1) | Headers of each build |
| `tools/evidence/msg_sizes.py <log.csv>` | MQTT payload sizes per topic (`message_sizes.md` section 2) | `is02_tool.py log` file of one track run per build |
| `tools/evidence/frame_budget.py` | Telemetry_Frame bit budget (`message_sizes.md` section 3) | `wire_dump` output of the Revised_Build |
| `tools/evidence/gen_docs.py` | [../data_representation.md](../data_representation.md) and [../protocol.md](../protocol.md) | headers, drivers, `wire_dump` and the target compiler (WSL) |
| `tools/evidence/perf_report.py <console.log>` | `performance.md` timing tables | USB console log of each `APP_PERF=1` build |
| `tests/host/run_all.sh` | Host_Test_Suite summary (`correctness.md` section 2) | Revised_Build sources |

## Index of evidence files

| File | Content | Requirements |
|---|---|---|
| [ram_usage.md](ram_usage.md) | `.data`/`.bss` comparison, per-symbol RAM table, saving | 22.1 to 22.7 |
| [message_sizes.md](message_sizes.md) | Inter-task message `sizeof`, pool block size, MQTT payload sizes per topic, Telemetry_Frame bit budget, reductions | 23.1 to 23.6 |
| [correctness.md](correctness.md) | Build check, Host_Test_Suite results, L1 to L6 summaries, functional checklist, hardware check procedures and results, self-test | 3.3, 3.7, 3.8, 9.8, 10.7, 10.9, 10.10, 19.5, 19.7, 19.9, 19.10, 24.1 to 24.3, 24.5, 24.8 |
| [performance.md](performance.md) | Task, ISR, critical-section and telemetry encode timing, E-stop PWM-off times, watchdog count, frame rate | 9.6, 10.2, 20.1 to 20.6, 24.4 to 24.7 |

Raw inputs (console logs, `is02_tool.py log` CSV files, `telemetry_monitor.py` CSV and log files, script outputs) go in `docs/evidence/raw/`, named `<build>_<what>_<date>.<ext>`, for example `baseline_console_perf_2026-11-10.log`.

## Status (2 Nov 2026)

| File | State |
|---|---|
| [ram_usage.md](ram_usage.md) | measured |
| [message_sizes.md](message_sizes.md) | sections 1 and 3 measured; section 2 and the 25 % check (section 4) wait for the car run |
| [correctness.md](correctness.md) | sections 1 and 2 (build check, host tests) measured; sections 3 to 7 wait for the car |
| [performance.md](performance.md) | waits for the car; section 4 has the instruction counts from the ledger, marked as counted |
