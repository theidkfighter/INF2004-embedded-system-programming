# Correctness evidence (Requirements 3.3, 3.7, 3.8, 9.8, 10.7, 10.9, 10.10, 19, 24.1 to 24.3, 24.5, 24.8)

Builds and commits are listed in [README.md](README.md). Unless a procedure says otherwise, the Baseline_Build is the recorded baseline commit and the Revised_Build is the Week 13 commit, both built with `APP_PERF=0` and `APP_SELFTEST=0`. For Baseline_Build runs, start the laptop tools with `--legacy-json`.

Use the same track layout, barcode map and `robot_params.txt` for both builds. Put every raw file (console logs, monitor CSV and log files, `is02_tool.py log` files) in `docs/evidence/raw/`.

The `--legacy-json` option exists only after the laptop tools are switched to the Laptop_Codec (Step 7). Before that, the unchanged tools read the Baseline_Build JSON directly.

Where to read values: the `telemetry_monitor.py` CSV has `enc_l`, `enc_r`, `barcode` and `obst_cm` but no `faults` column. Read `faults` from a parallel `python MQTT/is02_tool.py --broker <ip> log <file>.csv` recording (the `faults` key of each `is02/telemetry` and `is02/status` payload, decoded for the Revised_Build), or from the dashboard.

## 1. Firmware build check (Requirement 19.5)

Filled by: `tools/evidence/build_check.sh` (runs `make clean && make APP_ROBOT=1` in `build_make/`, counts warnings, compares with the Baseline_Build count and lists warnings in new or changed files). Run on 2 Nov 2026 in WSL Ubuntu 24.04 on a clean `git worktree` of the commit below, with placeholder `config/*.h` files. The full make log is about 730 KB and is not committed.

| Item | Value |
|---|---|
| Commit checked | `36a380bc3d8daf0f945e6c6853db1ab53914633e` |
| Build exit status | 0 (`build: ok`) |
| Compiler warnings, Baseline_Build clean build | 0 (`tools/evidence/baseline_warnings.txt`) |
| Compiler warnings, Revised_Build clean build | 0 |
| Warnings that name a file added or changed since the baseline commit | 0 (187 files changed since `bca7964`) |
| Script exit status | 0 |
| Result | **pass** |

Command: `cd <worktree> && PICO_SDK_PATH=$HOME/pico/pico-sdk bash tools/evidence/build_check.sh -- -j8`. Earlier sessions also built with `APP_PERF=1`, `APP_LEGACY_TEXT_CMD=1` and `APP_SELFTEST=1` through the same script, each with 0 warnings (ledger sessions 1 to 4); those variants were not repeated for this document.

## 2. Host_Test_Suite results (Requirements 24.1, 24.8)

Filled by: `tests/host/run_all.sh` (run in WSL from the repository root; optional `--seed <s>`). The summary lines `P<n> <title> | <payload type or Storage_Unit> | cases=<n> | failed=<n> | seed=<s>` are in the raw output file; the tables below add them up per property. Python rows are the C-library side run by pytest, JavaScript rows the Node side (`node --test`).

| Item | Value |
|---|---|
| Commit tested | `36a380bc3d8daf0f945e6c6853db1ab53914633e` (the sources at the start of this session) |
| Command line | `rm -rf tests/host/build && PYTHON=~/is02venv/bin/python bash tests/host/run_all.sh --seed 12345` |
| Seed | 12345 |
| Exit status | 0, last line `Host_Test_Suite: PASSED (seed=12345)` |
| Totals | properties=543 failed=0; other tests passed=1489 failed=0 skipped=0 (pytest 1,924 passed in 139 s; Node 262 pass; `test_static.sh` 24 pass; `test_compile_fail.sh` 8 pass) |
| Raw output file | `docs/evidence/raw/revised_host_tests_2026-11-02.log` |

### 2.1 Requirement 18 properties and codec pairs (Requirement 24.1)

The Python rows exercise the C Wire_Codec against the Python Laptop_Codec; the JavaScript rows exercise the C Wire_Codec against the JavaScript Laptop_Codec (`node --test`).

| Requirement 18 criterion | Property | Codec pairs exercised | Payload types / Storage_Units | Cases | Failed | Result |
|---|---|---|---|---|---|---|
| 18.1 robot-to-laptop round trip | P25 | C Wire_Codec → Python Laptop_Codec (7,500); C Wire_Codec → JavaScript Laptop_Codec (7,500) | Telemetry, Heartbeat, Ack (1,000 each per pair) and 45 Event_Codes (100 each per pair): 48 units | 15,000 | 0 | pass |
| 18.2 laptop-to-robot round trip | P26 | Python Laptop_Codec → C Wire_Codec (13,000); JavaScript Laptop_Codec → C Wire_Codec (13,000) | 13 units: each command code, Map, Config (1,000 each per pair) | 26,000 | 0 | pass |
| 18.3 decode then encode is identity | P27 | C Wire_Codec, Python Laptop_Codec (7,000); C Wire_Codec, JavaScript Laptop_Codec (7,000) | all seven payload types (1,000 each per pair) | 14,000 | 0 | pass |
| 18.4 saturation | P2 | C packed accessors and Wire_Codec encoders (Python/ctypes) | 72 fields of the Status_Snapshot, odometry record, messages and frames | 143,071 | 0 | pass |
| 18.5 field independence | P4 | C packed accessors | **no property result**: `test_status_snapshot.py` has example checks only (ledger 6.4, `partly`) | n/a | n/a | not met as a property |
| 18.6 cross-codec shared vectors | P28 | C Wire_Codec, Python Laptop_Codec (445); C Wire_Codec, JavaScript Laptop_Codec (345) | shared vector set | 790 | 0 | pass |
| 18.7 malformed payload rejection | P21 | C Wire_Codec, both Laptop_Codecs | **no property result**: skipped (minimum scope, ledger 18.5); the reject paths run inside P19 and P26 to P28 | n/a | n/a | not run |

### 2.2 All property summary lines

One row per property that printed summary lines. Cases are summed over the rows of the property (one row per payload type, field or Storage_Unit, and per runner). `Units` is the number of distinct payload types, fields or Storage_Units. Every property has at least 100 cases except P9, which is exhaustive over the 43 Code 39 characters (86 cases = 43 characters × 2 checks).

| Property | Title | Units | Cases, C with Python | Cases, C with JavaScript | Failed | Seed | Result |
|---|---|---|---|---|---|---|---|
| P1 | Continuous stores round to the nearest step | 71 | 14200 (71 rows) | - | 0 | 12345 | pass |
| P2 | Out-of-range and NaN stores saturate and are counted | 72 | 143071 (214 rows) | - | 0 | 12345 | pass |
| P3 | Sentinel codes never collide with values | 22 | 23724 (44 rows) | - | 0 | 12345 | pass |
| P9 | Code 39 index mapping is a bijection | 1 | 86 (2 rows) | - | 0 | 12345 | pass |
| P10 | Inter-task messages round-trip | 3 | 800 (3 rows) | - | 0 | 12345 | pass |
| P14 | Obstacle planning matches the Baseline_Build | 2 | 2000 (2 rows) | - | 0 | 12345 | pass |
| P15 | Hump detection matches the Baseline_Build | 2 | 2000 (2 rows) | - | 0 | 12345 | pass |
| P16 | Event detail text can be rebuilt | 46 | 9000 (45 rows) | 4500 (1 rows) | 0 | 12345 | pass |
| P18 | Critical sections are balanced and cover every shared update | 6 | 600 (6 rows) | - | 0 | 12345 | pass |
| P19 | Every encoded frame has the fixed header and length | 3 | 200 (3 rows) | - | 0 | 12345 | pass |
| P24 | The parameter table version detects table edits | 1 | 1000 (1 rows) | 1000 (1 rows) | 0 | 12345 | pass |
| P25 | Robot-to-laptop frames round-trip through both Laptop_Codecs | 48 | 7500 (48 rows) | 7500 (48 rows) | 0 | 12345 | pass |
| P26 | Laptop-to-robot frames round-trip into the Wire_Codec | 13 | 13000 (13 rows) | 13000 (13 rows) | 0 | 12345 | pass |
| P27 | Decode then encode is the identity on bytes | 7 | 7000 (7 rows) | 7000 (7 rows) | 0 | 12345 | pass |
| P28 | The three codecs agree on the shared vectors | 1 | 445 (1 rows) | 345 (1 rows) | 0 | 12345 | pass |
| P29 | Tool inputs parse to Parameter_IDs or report the bad line | 2 | 1000 (2 rows) | 1000 (2 rows) | 0 | 12345 | pass |
| P30 | Unknown codes are shown numerically | 3 | 600 (3 rows) | 600 (3 rows) | 0 | 12345 | pass |

Properties with no summary line (no result is claimed for them):

| Property | Status | Cases |
|---|---|---|
| P4 | partly: example checks only (ledger 6.4) | - |
| P5 | skipped (minimum scope, ledger 6.5) | - |
| P6 | skipped (minimum scope, ledger 6.6) | - |
| P7 | todo (ledger 5.3) | - |
| P8 | skipped (minimum scope, ledger 6.7) | - |
| P11 | skipped (minimum scope, ledger 8.6) | - |
| P12 | todo (ledger 12.5) | - |
| P13 | differential test `test_diff_barcode.py` runs (313 tests, all pass) but prints no property line | - |
| P17 | skipped (minimum scope, ledger 16.3) | - |
| P20 | skipped (minimum scope, ledger 16.7) | - |
| P21 | skipped (minimum scope, ledger 18.5) | - |
| P22 | skipped (minimum scope, ledger 10.10) | - |
| P23 | skipped (minimum scope, ledger 18.6) | - |

### 2.3 Example, static and compile-fail tests

Filled by: `tests/host/run_all.sh` (non-property test results). pytest counts below are the number of test records per file; every one passed.

| Test | Result | Notes |
|---|---|---|
| `test_static.sh` | pass | `static checks: passed=24 failed=0 skipped=0` |
| `test_compile_fail.sh` | pass | 8 PASS lines: 7 narrow-field header edits fail to compile with the field's name, the unchanged headers compile |
| `test_layout_doc.py` | not written | task 20.7 skipped; `docs/data_representation.md` is generated by `tools/evidence/gen_docs.py`, which stops when a computed Required_Bits differs from the code |
| `test_tables.py` | covered by `test_codec_tables.py` | 10 tests pass, plus `tables.test.js` in Node (task 20.8 skipped as a separate file) |
| `test_fingerprint.py` | not written | the parameter table CRC is checked by P24 (1,000 cases Python, 1,000 JavaScript) and `test_spec_conformance.py` |
| `test_resolution.py` | skipped | task 3.13 rest; P1 covers the rounding |
| `test_nan.py` | pass | 71 tests |
| `test_adc_range.py` | not written | task 12.6 `todo` |
| `test_presentation.py` | pass | 10 tests, plus `presentation.test.js` in Node |
| `test_estop_classify.py` | not written | skipped (ledger 18.7) |
| `test_legacy_text.py` | not written | skipped (ledger 18.7) |
| `test_pool_empty.py` | not written | skipped (ledger 8.8) |
| `test_tools.py` | pass | 97 tests |
| `run_all.sh --selfcheck` | not run in this session | `test_selfcheck.sh` is part of the checked-in suite; the gate command above does not call it |

Other pytest files in the run (tests, all passed): `test_baseline_lib.py` 6, `test_bitpack.py` 20, `test_codec_basic.py` 376, `test_codec_cross.py` 1, `test_codec_events.py` 85, `test_diff_barcode.py` 313, `test_isr_ring.py` 7, `test_msg_codec.py` 24, `test_nav_map.py` 243, `test_prop01_rounding.py` 71, `test_prop03_sentinels.py` 22, `test_prop09_c39_bijection.py` 1, `test_prop14_obstacle.py` 2, `test_prop15_hump.py` 16, `test_prop18_critical_sections.py` 9, `test_spec_conformance.py` 22, `test_status_snapshot.py` 15, `test_stubs.py` 9, `test_wire_basic.py` 22, `test_wire_tools.py` 5.

## 3. Track runs (Requirement 19.7)

> **To be measured on the car.** Nothing in this section has been recorded yet; the tables stay empty until the run. Fill from: `python MQTT/telemetry_monitor.py --broker <LAPTOP_IP>` (add `--legacy-json` for the Baseline_Build); procedure below.

Do 3 full track runs with each build on the same track, with the same barcode map and the same `robot_params.txt`. A run starts when the robot enters LINE_FOLLOWING at the track start and ends when it stops at the track end.

Procedure for each run:

1. Flash the build and let it boot to IDLE with the hotspot and broker up.
2. Start the monitor: `python MQTT/telemetry_monitor.py --broker <ip>` (add `--legacy-json` for the Baseline_Build). Note the CSV and log file names it prints.
3. Publish the barcode map and `robot_params.txt` (`robot_map.py`, `robot_params.py`) the same way for both builds.
4. Start the run (START command or BTN1). During the run press `p` once (PING, for check L5) and, after the robot has driven at least half the track, press `e` (ESTOP, for check L6), then recover the robot the way the Baseline_Build requires and continue the run.
5. When the robot stops at the track end, press `q` and keep the printed summary.
6. Read the barcode letters, turn directions, obstacle result, hump count and final state from the monitor log (`is02/event` lines) and CSV.

| Build | Run | Decoded barcode letters (in order) | Turn direction at each junction | Obstacle avoidance result (bypass complete / abort) | Hump count | Final system state | L1 to L6 (pass/fail each) | Monitor CSV / log files |
|---|---|---|---|---|---|---|---|---|
| Baseline_Build | 1 | | | | | | | |
| Baseline_Build | 2 | | | | | | | |
| Baseline_Build | 3 | | | | | | | |
| Revised_Build | 1 | | | | | | | |
| Revised_Build | 2 | | | | | | | |
| Revised_Build | 3 | | | | | | | |

Result (every Revised_Build run matches the Baseline_Build runs in every column except the file names): `<pass/fail>`

## 4. Telemetry_Monitor summaries, checks L1 to L6 (Requirement 24.2)

> **To be measured on the car.** Nothing in this section has been recorded yet; the tables stay empty until the run. Fill from: the summary `telemetry_monitor.py` prints when you press `q` in the runs of section 3.

Filled by: the summary that `telemetry_monitor.py` prints when it quits (also written at the end of its log file). Use one full track run per build from section 3. Each run must include at least one command answered by an Ack_Frame (L5) and at least one ESTOP (L6).

### Baseline_Build (run `<n>`)

| Check | Result | Detail text |
|---|---|---|
| L1 telemetry rate | | |
| L2 message structure | | |
| L3 no missing frames | | |
| L4 heartbeat | | |
| L5 commands | | |
| L6 E-stop | | |

### Revised_Build (run `<n>`)

| Check | Result | Detail text |
|---|---|---|
| L1 telemetry rate | | |
| L2 message structure | | |
| L3 no missing frames | | |
| L4 heartbeat | | |
| L5 commands | | |
| L6 E-stop | | |

## 5. Functional checklist (Requirement 24.3)

> **To be measured on the car.** Nothing in this section has been recorded yet; the tables stay empty until the run. Fill from: fill the expected outcomes before the runs, the observed ones from the monitor log (`is02/event` lines) and CSV of section 3.

Fill in the expected outcome on the track before the runs. A feature passes on a build only if the observed outcome equals the expected outcome.

| Feature | Expected outcome on the track | Observed, Baseline_Build | Result, Baseline_Build | Observed, Revised_Build | Result, Revised_Build |
|---|---|---|---|---|---|
| Line following | | | | | |
| Barcode decoding | | | | | |
| Junction navigation | | | | | |
| Obstacle avoidance | | | | | |
| Hump detection | | | | | |
| E-stop | | | | | |
| Recovery | | | | | |
| Telemetry | | | | | |

## 6. Hardware checks

> **To be measured on the car.** Every check in this section needs the robot. The command for each check is at the top of its subsection.

### 6.1 Encoder counts on a 1,000 mm course at `SPEED_MAX_MM_S` (Requirement 10.7)

> **To be measured on the car.** Fill from: drive method and `hal_enc_errors()` read over SWD as described below; `python MQTT/telemetry_monitor.py --broker <LAPTOP_IP>` gives `enc_l`, `enc_r`; builds `make clean && make APP_ROBOT=1 APP_PERF=1`.

Builds: the Step 0 commit and the Revised_Build, both with `APP_PERF=1`, so the same runs also give the critical-section maxima at `SPEED_MAX_MM_S` for [performance.md](performance.md) section 5.

Set-up:

1. Mark a straight 1,000 mm course on the floor with a start line and an end line. Use the same course and surface for both builds.
2. Choose one drive method that moves the robot straight over the course at `SPEED_MAX_MM_S` (250 mm/s) and use it unchanged for both builds. The firmware has no laptop command for a straight leg at this speed, and line following is capped at 150 mm/s. Record the method: `<fill in>`.
3. Read the encoder error counter `hal_enc_errors()` (variable `g_enc_err` in `hal_isr.c`). No baseline console or MQTT output prints it, so read it over SWD with a debugger, for example `arm-none-eabi-gdb build_make/mtkernel_3.elf`, then `print 'hal_isr.c'::g_enc_err`. If no debugger is available, write "not measured" and record the reason in section 7.

Procedure, 5 runs per build:

1. Place the robot with both wheel axles on the start line.
2. Record the telemetry `enc` values `[left, right]` (monitor CSV columns `enc_l`, `enc_r`) and the encoder error count before the run.
3. Drive the course. Stop with the axles on the end line.
4. Record `enc` and the encoder error count again once the robot is stationary.
5. Per-wheel transition count = after − before (absolute value). Error increase = after − before.

| Build | Run | Left transitions | Right transitions | `hal_enc_errors` increase |
|---|---|---|---|---|
| Step 0 commit | 1 | | | |
| Step 0 commit | 2 | | | |
| Step 0 commit | 3 | | | |
| Step 0 commit | 4 | | | |
| Step 0 commit | 5 | | | |
| Revised_Build | 1 | | | |
| Revised_Build | 2 | | | |
| Revised_Build | 3 | | | |
| Revised_Build | 4 | | | |
| Revised_Build | 5 | | | |

| Item | Left wheel | Right wheel |
|---|---|---|
| Baseline mean transitions | | |
| Revised mean transitions | | |
| Difference (% of Baseline mean) | | |
| Limit | within 1 % | within 1 % |
| Largest per-run error increase, Baseline | | |
| Largest per-run error increase, Revised | | |
| Result (mean within 1 % and Revised largest increase ≤ Baseline largest increase) | | |

### 6.2 Barcode passes (Requirement 10.9)

> **To be measured on the car.** Fill from: `BARCODE_DEBUG=1` (`python MQTT/robot_params.py` or `SET`), then the `BC_DBG` events and the `bc` field in `python MQTT/telemetry_monitor.py --broker <LAPTOP_IP>`.

Set-up: one printed Code 39 barcode on the track, the same `LINE_BASE_MM_S` for both builds, and `BARCODE_DEBUG=1` so each `BC_DBG` event reports the overrun count (`ovr`). The overrun count can also be read over SWD (`print 'hal_isr.c'::g_bc_overruns`).

Procedure, 5 passes per build:

1. Record the overrun count before the first pass.
2. Line-follow over the barcode at the set speed. Record the decoded character from the `BARCODE` event or the telemetry `bc` field.
3. Repeat for 5 passes, then record the overrun count again.

| Build | Pass | Expected character | Decoded character | Overrun count after the pass |
|---|---|---|---|---|
| Baseline_Build | 1 | | | |
| Baseline_Build | 2 | | | |
| Baseline_Build | 3 | | | |
| Baseline_Build | 4 | | | |
| Baseline_Build | 5 | | | |
| Revised_Build | 1 | | | |
| Revised_Build | 2 | | | |
| Revised_Build | 3 | | | |
| Revised_Build | 4 | | | |
| Revised_Build | 5 | | | |

| Item | Baseline_Build | Revised_Build |
|---|---|---|
| Line speed (`LINE_BASE_MM_S`) | | |
| Overrun count before pass 1 | | |
| Total overrun increase over 5 passes | | |
| Same character as the Baseline_Build on every pass | | |
| Result (same characters, Revised increase ≤ Baseline increase) | | |

### 6.3 Ultrasonic medians at 20 cm and 100 cm (Requirement 10.10)

> **To be measured on the car.** Fill from: `python MQTT/telemetry_monitor.py --broker <LAPTOP_IP>`; column `obst_cm` of its CSV.

The obstacle task pings only in LINE_FOLLOWING and OBSTACLE AVOIDANCE, so the robot is held stationary on a stand while it is in LINE_FOLLOWING.

Set-up:

1. Put the robot on a stand so the wheels turn freely and the robot body does not move. Place a strip of the track line under the line sensors as when the robot is centred on the line.
2. Set `OBSTACLE_STOP_CM` to 10 (its minimum) on both builds, so a target at 20 cm does not start obstacle avoidance. Record the value used.
3. Place a flat target square to the sensor, with the servo at the forward position, at 20.0 cm from the sensor face. Measure with a ruler.

Procedure, for each build and each distance:

1. Start the monitor and send START. Check that the state is LINE_FOLLOWING. If it leaves LINE_FOLLOWING during the measurement, discard the readings and repeat.
2. Take 20 consecutive telemetry frames and read `obst.cm` (monitor CSV column `obst_cm`).
3. Send STOP. Move the target to 100.0 cm and repeat.
4. Median = mean of the 10th and 11th values after sorting the 20 readings.

| Build | Distance | 20 readings (cm) | Median (cm) |
|---|---|---|---|
| Baseline_Build | 20 cm | | |
| Baseline_Build | 100 cm | | |
| Revised_Build | 20 cm | | |
| Revised_Build | 100 cm | | |

| Distance | Baseline median (cm) | Revised median (cm) | Difference (cm) | Limit | Result |
|---|---|---|---|---|---|
| 20 cm | | | | ±1 cm | |
| 100 cm | | | | ±1 cm | |

### 6.4 Wi-Fi link loss and MQTT session loss bits, offline buffering (Requirements 3.7, 3.9, 19.9)

> **To be measured on the car.** Fill from: `python MQTT/telemetry_monitor.py --broker <LAPTOP_IP>` plus `python MQTT/is02_tool.py --broker <LAPTOP_IP> log docs/evidence/raw/<build>_faults_<date>.csv` for `faults`.

Fault_Mask bit values: Wi-Fi link loss = bit 13 (8192), MQTT session loss = bit 14 (16384). The Baseline_Build has no such bits; record its state and console text as the reference.

Procedure, for each build:

1. Boot with the hotspot and broker up. Start the monitor. Before the first connection the Revised_Build heartbeat and telemetry `faults` value has bits 13 and 14 set; record the first `faults` value seen after the connection (both bits clear).
2. MQTT session loss only: stop the Mosquitto broker for about 10 s while the hotspot stays up, then start it again. Expected on the Revised_Build: frames buffered during the outage show bit 14 set and bit 13 clear; the first live frames after reconnection show both clear.
3. Wi-Fi link loss: with the robot in IDLE, turn the hotspot off for about 20 s (less than 32 s, so the 64-frame Frame_Ring does not overflow), then on. Expected on the Revised_Build: buffered frames show bits 13 and 14 set; the console status line reads `wifi down` during the outage; after reconnection the buffered frames are published oldest first, at most 4 per radio poll, and the first live frames show both bits clear.
4. Repeat step 3 while the robot is line following (on the stand or on the track). The state decisions must be the same as on the Baseline_Build: the robot keeps line following.
5. Check L3 and L4 in the monitor summary: no missing frames, and one heartbeat gap for the outage.

| Build | Step | Console status text | `faults` during outage (from buffered frames) | `faults` after reconnection | Buffered frames replayed in order (L3) | State during outage | Result |
|---|---|---|---|---|---|---|---|
| Baseline_Build | MQTT loss | | | | | | |
| Baseline_Build | Wi-Fi loss, IDLE | | | | | | |
| Baseline_Build | Wi-Fi loss, line following | | | | | | |
| Revised_Build | MQTT loss | | | | | | |
| Revised_Build | Wi-Fi loss, IDLE | | | | | | |
| Revised_Build | Wi-Fi loss, line following | | | | | | |

### 6.5 Encoder loss fault (Requirement 3.3)

> **To be measured on the car.** Fill from: same two commands as 6.4.

`FAULT_ENCODER_LOSS` is bit 2 (value 4). The check sets it when a wheel has duty above 0.30 (`ENC_LOSS_DUTY`) and zero encoder ticks for at least 200 ms (`ENC_LOSS_MS`).

Procedure, for each build:

1. Put the robot on a stand and start line following, so both wheels are driven.
2. Hold the right wheel still by hand for at least 1 s. Watch the telemetry `duty` value rise above 30 %.
3. Expected: the robot enters ERROR, the motors stop, an `ERROR` event with the encoder loss reason is published, and `faults` has bit 2 set. On the Revised_Build no other motor or encoder bit is set.
4. Recover the way the Baseline_Build requires (BTN1 re-check) and repeat with the left wheel.

| Build | Wheel held | Duty before stop (%) | State reached | `faults` value | Event and reason text | Recovery action and result | Result |
|---|---|---|---|---|---|---|---|
| Baseline_Build | Right | | | | | | |
| Baseline_Build | Left | | | | | | |
| Revised_Build | Right | | | | | | |
| Revised_Build | Left | | | | | | |

### 6.6 Ultrasonic no-echo bit (Requirement 3.8)

> **To be measured on the car.** Fill from: same two commands as 6.4, robot on the stand.

Ultrasonic no-echo is Fault_Mask bit 15 (32768). The obstacle task pings only while line following, so use the stand set-up from section 6.3.

Procedure, for each build:

1. Robot on the stand in LINE_FOLLOWING, sensor pointing at open space (no surface within about 4 m, or at a soft absorbing surface).
2. Expected on the Revised_Build: telemetry `obst.cm` reads −1.0 and `faults` has bit 15 set. On the Baseline_Build: `obst.cm` reads −1.0 (no bit).
3. Place a target at about 50 cm. Expected on the Revised_Build: `obst.cm` reads about 50 and bit 15 clears within one ping (70 ms, so by the next telemetry frame).
4. The state must stay LINE_FOLLOWING in both cases on both builds.

| Build | Condition | `obst.cm` | `faults` value | Bit 15 | State | Result |
|---|---|---|---|---|---|---|
| Baseline_Build | open space | | | n/a | | |
| Baseline_Build | target at 50 cm | | | n/a | | |
| Revised_Build | open space | | | | | |
| Revised_Build | target at 50 cm | | | | | |

### 6.7 E-stop triggers (Requirement 19.10)

> **To be measured on the car.** Fill from: `python MQTT/telemetry_monitor.py --broker <LAPTOP_IP>` (key `e` for the remote ESTOP) and the USB console.

For each trigger, the motors must turn off, the state must become EMERGENCY STOP (or ERROR where the Baseline_Build enters ERROR for that cause), the Revised_Build must publish an Event_Frame and Ack_Frame carrying the same information as the Baseline_Build event and ack, and the robot must leave the state only through the same user action as on the Baseline_Build.

| Trigger | How to cause it | Build | Motors off | State | Event (and ack) published | Reason text | Action that left the state | Result |
|---|---|---|---|---|---|---|---|---|
| BTN2 local stop | Press BTN2 while line following | Baseline_Build | | | | | | |
| BTN2 local stop | | Revised_Build | | | | | | |
| Remote ESTOP | Monitor key `e` while line following | Baseline_Build | | | | | | |
| Remote ESTOP | | Revised_Build | | | | | | |
| Proximity < 8 cm | Hand at about 5 cm in front of the sensor while line following | Baseline_Build | | | | | | |
| Proximity < 8 cm | | Revised_Build | | | | | | |
| IMU collision | Firm tap on the front of the chassis while line following | Baseline_Build | | | | | | |
| IMU collision | | Revised_Build | | | | | | |
| Battery critical | Bench supply lowered below the critical threshold (only if VBAT sensing is on) | Baseline_Build | | | | | | |
| Battery critical | | Revised_Build | | | | | | |
| Heartbeat watchdog (300 ms) | No user action causes it on a working build; record "not triggered" and the reason if it cannot be caused | Baseline_Build | | | | | | |
| Heartbeat watchdog (300 ms) | | Revised_Build | | | | | | |

### 6.8 On-target concurrency self-test, `APP_SELFTEST=1` (Requirement 9.8)

> **To be measured on the car.** Fill from: `cd build_make && make clean && make APP_ROBOT=1 APP_SELFTEST=1`, then read the `[selftest]` line on the USB console.

Build: Revised_Build with `make clean && make APP_ROBOT=1 APP_SELFTEST=1`. This build is used only for this check.

Procedure:

1. Build and flash the self-test image.
2. Open the USB serial console before or right after reset, with logging to `docs/evidence/raw/revised_selftest_<date>.log`.
3. At boot, before the application tasks start, the self-test runs two tasks at different priorities (the higher one woken every 1 ms). Each updates a different Packed_Field of one shared Storage_Unit 10,000 times.
4. Read the `[selftest] PASS` or `[selftest] FAIL` line and the final field values it prints. Expected: PASS, each updated field equal to the last value its task wrote, every other field equal to its value before the test.
5. Check that the robot then boots normally to IDLE.
6. Rebuild and flash the normal image (`APP_SELFTEST=0`) afterwards.

| Item | Value |
|---|---|
| Commit | `<fill in>` |
| `make` command line | `<fill in>` |
| Console line printed | `<paste the [selftest] line>` |
| Final value, field written by the higher-priority task (expected / observed) | |
| Final value, field written by the lower-priority task (expected / observed) | |
| Other fields unchanged | |
| Normal boot to IDLE afterwards | |
| Result | `<pass/fail>` |

## 7. Failures and identified causes (Requirements 24.5, 24.8)

> **To be measured on the car.** Nothing in this section has been recorded yet; the tables stay empty until the run. Fill from: fill from sections 2 to 6 once the car runs are in. The host-test section has no failure.

Fill in one row for every failing Host_Test_Suite property (with its failing input and seed), every failing check L1 to L6, every functional checklist feature marked fail on either build, and every hardware check above that fails its limit or could not be measured. Leave the table empty if everything passes.

| Failing item | Build | Failing input (property) or observed outcome (check, feature, measurement) | Limit or expected outcome | Identified cause |
|---|---|---|---|---|
| | | | | |
