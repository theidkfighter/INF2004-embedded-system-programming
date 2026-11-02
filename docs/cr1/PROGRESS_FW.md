# Firmware track: progress ledger

**This is the single source of truth for the firmware track.**
- Rebuilt on 2 Nov 2026 from `docs/CR1_FW_STATUS.md`, and checked against the code on `cr1/fw-rest`.
- The tick marks in `spec/tasks.md`, and in Kiro's `.kiro/specs/.../tasks.md`, aren't kept up to date.
- Every session reads this file first and updates it last, in the same commit as the code.

Status values:
- `done`: the code or test is in and the gate passed with it;
- `partly`: some of the task is in; the note says what is missing;
- `todo`: not started;
- `skipped (minimum scope)`: left out on purpose, with a one-line reason. These reasons go into the data representation table;
- `car-open`: code done, car check pending.

## Current state (2 Nov 2026)

- **Branch:** `cr1/fw-rest` (from `cr1/laptop`) is the only CR1 firmware branch from now on. `Wei-En` gets no more commits and isn't merged again. Its last work is already in `cr1/laptop` (`5393d66`, merge `aa2fab7`).
- **Scope:** the minimum Change Requirement 1 work. The `skipped (minimum scope)` rows below aren't planned.
- **Binary only:** the robot accepts binary commands only. `APP_LEGACY_TEXT_CMD=1` builds but changes nothing. The laptop tools send binary by default (LT-05).
- **Nothing has run on a robot yet** (see "Car checks waiting").

## Ledger

Commit column: `Wei-En` means the work now in `cr1/laptop` (`5393d66` or earlier), which the integration gate on `71a0eeb` passed.

### Step 0: baseline, switches, scaffolding

| Task (spec/tasks.md) | Status | Commit | Note |
|---|---|---|---|
| 1.1 build switches | done | 542368f | |
| 1.2 build_check.sh | done | 542368f | `baseline_warnings.txt` (0) committed |
| 1.3 perf probes | done | 542368f | `tlm_bench` times the binary encode since `5393d66` |
| 1.4 host test scaffolding | done | 542368f, repaired on Wei-En | |
| 1.5 baseline differential harness | done | 542368f | `libbase.so` builds in `run_all.sh` |
| 1.6 evidence scripts | done | 542368f | |
| 1.7 evidence templates | done | 542368f | tables stay empty until the car measurements |
| 1.8 static checks | done | 542368f | |
| 1.9 selfcheck | done | 542368f | |
| checkpoint 2: baseline measurements (`APP_PERF=1`) | car-open | | **needs the car** |

### Step 1: plain C foundation

| Task | Status | Commit | Note |
|---|---|---|---|
| 3.1 bitpack.h | done | 542368f | |
| 3.2 Property 1 | done | 542368f | |
| 3.3 Property 2 + test_nan | done | 542368f | |
| 3.4 Property 3 | done | 542368f | |
| 3.5 crit.h, saturation counter | done | 542368f | |
| 3.6 Code 39 index helpers | done | 542368f | |
| 3.7 Property 9 | done | Wei-En | |
| 3.8 wire_ids.h | done | 542368f | the contract (see INTEGRATION.md) |
| 3.9 Wire_Codec robot-to-laptop | done | Wei-En | all 7 payload types |
| 3.10 laptop-to-robot frames | done | Wei-En | inverse encoders `wire_enc_cmd`, `wire_enc_map`, `wire_enc_config` |
| 3.11 wire_dump + frame_budget.py | done | Wei-En | |
| 3.12 Property 19 | done | Wei-En | |
| 3.13 Wire_Codec unit tests | partly | Wei-En | encode-then-decode examples are in `test_wire_basic.py` |
| 3.13 (rest) `test_resolution.py` | skipped (minimum scope) | | Each field's resolution is fixed by the D8 layout, which `test_spec_conformance.py` reads, and Property 1 covers the rounding |
| 3.14 msg_codec | done | Wei-En | `obst_msg_t` is 24 B (spec 28); encoders return no clamp count |
| 3.15 Property 10 | done | Wei-En | |
| 3.16 evt builders | done | Wei-En | one generic builder, not one function per Event_Code |
| 14.5 (C half) wire_vectors.c | done | Wei-En | |
| checkpoint 4 | done | Wei-En | sync point S1 met |

### Step 2: Fault_Mask

| Task | Status | Commit | Note |
|---|---|---|---|
| 5.1 16-bit Fault_Mask | done | Wei-En | `fault_mask.h` |
| 5.2 replace duplicate fault variables | partly | Wei-En | `tsk_imu.c` still keeps its local `b_dead` mirror (same transitions) |
| 5.3 Property 7 | todo | | no Property 7 test; `test_status_snapshot.py` has example checks only |
| 5.4 Fault_Mask edge-case unit tests | skipped (minimum scope) | | The boot value and recovery mask are constants next to their use, and the car check covers ESTOP + BTN1 recovery |

### Step 3: Status_Snapshot

| Task | Status | Commit | Note |
|---|---|---|---|
| 6.1 status.h | partly | Wei-En | 60 B; units grouped differently from the D2 table (internal only). The enum-width and field-fit `_Static_assert`s are missing (Requirement 2.13) |
| 6.2 accessors in app_status.c | done | Wei-En | every writer uses a dispatch-disabled section |
| 6.3 convert writers and readers | done | Wei-En | |
| 6.4 Property 4 | partly | | example checks in `test_status_snapshot.py`, no 1,000-sequence property |
| 6.5 Property 5 (invalid discrete writes) | skipped (minimum scope) | | Discrete fields come from enums and fixed call sites, so out-of-range input is not reachable at run time |
| 6.6 Property 6 (saturating counters) | skipped (minimum scope) | | All counters use the one helper `bp_sat_inc`, and `test_status_snapshot.py` shows them holding at the limit |
| 6.7 Property 8 (line sensor codes) | skipped (minimum scope) | | One line of polarity logic, checked on the car by line following |
| 6.8 compile-fail test for narrow enums | skipped (minimum scope) | | The enums are fixed for CR1, so none can outgrow its field; the test also needs the asserts 6.1 lacks |
| checkpoint 7 (bench) | car-open | | **needs the car** |

### Step 4: shared scalars, odometry, done word, mailboxes

| Task | Status | Commit | Note |
|---|---|---|---|
| 8.1 system word and stop reason | done | Wei-En | |
| 8.2 buzzer, heartbeats, re-arm counter, command ids, E-stop flags | done | this session | `g_next_id` was already 16-bit in a section: unchanged. The rest is new, in `shared.h` / `app_shared.c` |
| 8.3 motion done word | done | this session | `g_done`, Requirement 4.5; one `str` and one `ldr` confirmed in the disassembly |
| 8.3 odometry record | done | Wei-En | 5 words (138 bits), not the 6 words of D3.4 |
| 8.4 packed mailboxes | done | Wei-En | |
| 8.5 on-target concurrency self-test | skipped (minimum scope) | | Property 18 checks the same section rule on the host; `APP_SELFTEST=1` only has to build |
| 8.6 Property 11 (motion targets) | skipped (minimum scope) | | CR1 did not change the target maths, and Property 10 covers the packed motion fields |
| 8.7 Property 18 | done | this session | `test_prop18_critical_sections.py`; 14 of 14 mutants caught (a store or copy moved out of its section, an unbalanced section, the reject removed) |
| 8.8 stop-reason and pool-empty unit tests | skipped (minimum scope) | | The texts live in one `wire_ids.h` table that the laptop table tests read, and the pool path keeps the baseline `TMO_POL` logic |
| checkpoint 9 (bench: turn, bypass) | car-open | | **needs the car** |

### Step 5: controller contexts, Parameter_Store, barcode map

| Task | Status | Commit | Note |
|---|---|---|---|
| 10.1 line contexts and `tsk_line.c` flags | partly | Wei-En | barcode part only; `line_ctx_t`, `search_ctx_t`, `g_searching`, `g_det_on` aren't packed |
| 10.2 IMU contexts | todo | | |
| 10.3 obstacle and motion contexts | skipped (minimum scope) | | Private to one task, so no race risk, and leaving them keeps the obstacle code unchanged |
| 10.4 Property 15 (hump) | todo | | |
| 10.5 Property 14 (obstacle planning) | todo | | |
| 10.6 Property 13 (barcode) | done | Wei-En | `test_diff_barcode.py` |
| 10.7 field independence for contexts | skipped (minimum scope) | | Extends a test to contexts that are not all packed; `bitpack.h` itself is tested |
| 10.8 Parameter_Store | done | Wei-En | D15 |
| 10.9 discrete parameter reads | done | Wei-En | |
| 10.10 Property 22 (Parameter_Store value model) | skipped (minimum scope) | | `test_spec_conformance.py` compares all 55 defaults with the Baseline_Build library, and Property 24 guards the table |
| 10.11 barcode map `g_map_w[3]` | todo | | still `g_map[26]` |
| 10.12 packed map unit tests | todo | | |
| checkpoint 11 | todo | | |

### Step 6: ISR_Shared_Data

| Task | Status | Commit | Note |
|---|---|---|---|
| 12.1 barcode edge ring | done | Wei-En | |
| 12.2 raw ring, barcode flags, thresholds | done | Wei-En | |
| 12.3 ADC range, echo, battery | partly | Wei-En | `g_vbat_div` not done: `g_sample_n` is still 32-bit |
| 12.4 driver flags | todo | | `g_motor_killed`, servo range and `g_imu_cfg` not packed |
| 12.5 Property 12 (ring FIFO) | todo | | |
| 12.6 ADC range unit test | todo | | |
| checkpoint 13 (ISR timing) | car-open | | **needs the car** |

### Step 7: laptop track

All done: see `PROGRESS_LT.md` (14.1 to 14.18). Checkpoint 15 is `car-open` there.

### Step 8: robot-to-laptop binary payloads

| Task | Status | Commit | Note |
|---|---|---|---|
| 16.1 Event_Frames | done | Wei-En | |
| 16.2 typed event calls | done | Wei-En | |
| 16.3 Property 17 (detail splitting) | skipped (minimum scope) | | Splitting is a short counted loop, and Property 16 and the laptop vectors exercise the split events |
| 16.4 Property 16 | done | LT-04 | laptop track |
| 16.5 Ack_Frames | done | Wei-En | |
| 16.6 Heartbeat_Frame | done | Wei-En | |
| 16.7 Property 20 (GET:PARAMS parts) | skipped (minimum scope) | | One loop over the table; the laptop round-trip properties (Property 25) cover the ack format |
| 16.8 Frame_Ring byte payloads | done | Wei-En | |
| 16.9 Telemetry_Frame | done | Wei-En | |
| 16.10 ring test for Frame_Ring | skipped (minimum scope) | | Property 12 isn't written yet, and the ring keeps the baseline interrupt-masked logic |
| 16.11 saturating test for network counters | skipped (minimum scope) | | Same saturating add as the other counters, and the values are shown in telemetry |
| checkpoint 17 (dashboard) | car-open | | **needs the car** |

### Step 9: laptop-to-robot binary payloads

| Task | Status | Commit | Note |
|---|---|---|---|
| 18.1 incoming payload path, ESTOP fast path | done | Wei-En | |
| 18.2 Command_Frames | partly | Wei-En | binary path done |
| 18.2 (rest) `APP_LEGACY_TEXT_CMD=1` text route | skipped (minimum scope) | | The tools send binary by default since LT-05, so nothing needs text commands; the switch still builds |
| 18.3 Map_Frames and Config_Frames | done | Wei-En | |
| 18.4 tools binary by default | done | LT-05 | laptop track |
| 18.5 Property 21 (malformed payloads) | skipped (minimum scope) | | The decoders' reject paths are covered by Properties 19 and 26 to 28, including the reject vectors |
| 18.6 Property 23 (whole-map replace) | skipped (minimum scope) | | `nav_map_set_slots` overwrites all 26 entries in one loop, and the ack shows the result |
| 18.7 ESTOP classifier and legacy text tests | skipped (minimum scope) | | The classifier is one comparison, and the legacy route isn't built (18.2 rest) |
| 18.8 barcode context flags (optional) | done | Wei-En | Property 13 still passes |
| checkpoint 19 (commands, ESTOP) | car-open | | **needs the car** |

### Step 10: documentation and evidence

| Task | Status | Commit | Note |
|---|---|---|---|
| 20.1 layout_dump | todo | | |
| 20.2 data_representation.md | todo | | include every `skipped` reason above |
| 20.3 protocol.md | todo | | |
| 20.4 README link, host test README | todo | | |
| 20.5 host-side evidence sections | todo | | |
| 20.6 remove unused JSON helpers | todo | | |
| 20.7 data representation document test | skipped (minimum scope) | | The table is checked by hand against the headers, which are the single source |
| 20.8 protocol table and fingerprint tests | skipped (minimum scope) | | The drift guards `test_codec_tables.py` and `tables.test.js` already compare the codecs with `wire_dump` |
| final checkpoint 21 | todo | | |

## Known differences from the spec text (internal, not on the wire)

- `msg_codec`: `obst_msg_t` is 24 bytes (spec 28); encoders do not return a clamp count; `msg_nav_pack` has no bool result.
- MC_LINE wheel speeds are S14 @0.1 mm/s (spec S13): `line_wheels()` can ask up to 2 × base speed before the clamp. To raise as a review flag.
- Event builders are one generic builder plus per-call-site field lists, not one function per Event_Code.
- Status_Snapshot units are grouped differently from the D2 table; the Telemetry_Frame converts the barcode to the D8 Code 39 index.
- The odometry record is 5 words (138 bits), not the 6 words of D3.4.
- `g_mpf_buf` is not used by the kernel (flag R3, open): the pool is created without `TA_USERBUF`. For `ram_usage.md`.
- `g_done`: the one store sits inside a dispatch-disabled section. D3.5 says no section is needed; the session task asked for every update of it to be inside one. Readers still use one load with no section.
- `buzzer_pattern_take`, `odometry_t`, `odometry_get`, `odometry_put` and the `BUZZ_*` constants moved from `os_glue.c/.h` to the plain `shared.h` / `app_shared.c`, so the host tests run the same code. `os_glue.h` includes `shared.h`, so callers didn't change.
- `g_estop_rx_us` holds the low 32 bits of the time, and `estop_pending` replaces the old "0 = none" convention (D3).

## Car checks waiting

| Check | From | Done | Result |
|---|---|---|---|
| Baseline measurements (`APP_PERF=1`, `docs/evidence/README.md` procedures) | FW-00 | no | |
| Session 1 build: boot, START, line following, ESTOP + BTN1 recovery, one barcode turn. Console and dashboard should look as before; the ESTOP ack's `pwm_off_us` should be a small number | FW session 1 | no | |
| S5 met: FW-09 builds may be flashed (LT-05 merged: the tools send binary by default). After flashing, run `robot_params.py` and send the map once so the retained `is02/config` and `is02/map` are binary | LT-05 | no | |

## Session log

Newest first. One entry per session: date, tool, commit, gate results (host totals, `arm-none-eabi-size` line), open issues, car checks for the user.

- **2026-11-02, minimum CR1 session 1 (Claude Code, local clone, Opus 5.5).** Branch `cr1/fw-rest`, commit `CR1 FW session 1: shared scalars, done word, Property 18`.
  - **Done:** this ledger rebuilt; 8.2; motion done word; 8.7. No item from `REVIEW_FLAGS.md` applied (R2: `g_estop_reported` left as it is; R3: pool unchanged).
  - **Gate (WSL, `PICO_SDK_PATH=$HOME/pico/pico-sdk`), all passed:**
    - `tools/evidence/build_check.sh -- -j8`: PASS, 0 warnings.
    - `APP_PERF=1`, `APP_LEGACY_TEXT_CMD=1`, `APP_SELFTEST=1`: each builds with 0 warnings.
    - `rm -rf tests/host/build && PYTHON=~/is02venv/bin/python bash tests/host/run_all.sh --seed 12345`: `Host_Test_Suite: PASSED (seed=12345)`; properties=539 failed=0, other tests passed=1212 failed=0 skipped=0. Before this session: 533 and 1202.
  - **`arm-none-eabi-size`:** text 409356, data 2796, bss 60184, dec 472336. Before this session: 409216 / 2796 / 60212.
  - **Audit of 8.2:** `g_next_id` already met it. `g_buzz` (was 32-bit, no section), `g_hb[]`, `g_reacq_seq` (32-bit), `g_estop_rx_us` (64-bit), `g_estop_pwm_off_us` (32-bit), and the separate E-stop and config `bool`s all needed the change. The heartbeat check is the same compare; `tsk_line.c` now loads `g_reacq_seq` once per use.
  - **Radio path:** `app_net_command` reaches `crit_dsp_*` from `cyw43_poll()` in the radio-owner task, with interrupts unmasked (`cyw43_thread_enter` is empty), so `tk_dis_dsp` takes effect there.
  - **Test stubs:** `crit_stub.c` gained watches and hooks. `sat_stub.c` opened a section for `n == 0` where the firmware returns early; it now follows the firmware.
  - **Open issues:**
    - Section lengths aren't measured. Every new section is a few loads and stores, and the longest is the unchanged 5-word odometry copy. The `cs_dsp` line of an `APP_PERF=1` run on the car gives the number (limit 10 µs).
    - 6.1 lacks its `_Static_assert`s (Requirement 2.13). Found while rebuilding this ledger, not in this session's tasks.
    - `docs/CR1_FW_STATUS.md` now lags this ledger; this file is the current record. `CONTEXT.md` and `INTEGRATION.md` still name `cr1/fw` and `Poh-Loon` as the working branches.
  - **Car check (nothing above ran on a robot):** boot, START, line following, ESTOP + BTN1 recovery, one barcode turn.
- **2026-11-02, Wei-En integration (see `PROGRESS_LT.md`).** `build_check.sh` PASS, 0 warnings; host suite PASSED (properties=533, other tests passed=1202).
- **2026-11-01, guides written (Claude chat).** Verified on `542368f`:
  - firmware builds with 0 warnings (`APP_PERF=0`: text 398820, data 2808, bss 107972; `APP_PERF=1`: 400324 / 2808 / 108896);
  - `build_check.sh` PASS;
  - host suite fails to build (`wire.c` missing). Repaired on Wei-En.
