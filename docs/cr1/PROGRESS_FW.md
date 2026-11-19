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
| 6.1 status.h | done | Wei-En, session 2 | 60 B; units grouped differently from the D2 table (internal only). Session 2 added the size, enum-width and field-fit `_Static_assert`s (Requirement 2.13), and `app_status.c` now uses the same named shifts and widths |
| 6.2 accessors in app_status.c | done | Wei-En | every writer uses a dispatch-disabled section |
| 6.3 convert writers and readers | done | Wei-En | |
| 6.4 Property 4 | partly | | example checks in `test_status_snapshot.py`, no 1,000-sequence property |
| 6.5 Property 5 (invalid discrete writes) | skipped (minimum scope) | | Discrete fields come from enums and fixed call sites, so out-of-range input is not reachable at run time |
| 6.6 Property 6 (saturating counters) | skipped (minimum scope) | | All counters use the one helper `bp_sat_inc`, and `test_status_snapshot.py` shows them holding at the limit |
| 6.7 Property 8 (line sensor codes) | skipped (minimum scope) | | One line of polarity logic, checked on the car by line following |
| 6.8 compile-fail test for narrow enums | done | session 2 | `tests/host/test_compile_fail.sh`, run by `run_all.sh`: 7 edited header sets each fail with the field's name, and the unchanged headers compile |
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
| 10.2 IMU contexts | done | session 2 | `hump_ctx_t.f` 16-bit, `mev_ctx_t.f` 8-bit, range-checked get/set in `imu_logic.h`; continuous members unchanged (D16) |
| 10.3 obstacle and motion contexts | skipped (minimum scope) | | Private to one task, so no race risk, and leaving them keeps the obstacle code unchanged |
| 10.4 Property 15 (hump) | done | session 2 | `test_prop15_hump.py`, 2,000 cases against `libbase.so`; also the counters, `mev_step` and every setter's range |
| 10.5 Property 14 (obstacle planning) | done | session 2 | `test_prop14_obstacle.py`, 2,000 cases against `libbase.so`, bit for bit. Covers `obstacle_logic.c`; the task-side planner in `tsk_obstacle.c` is kernel code, so its diff was read by hand |
| 10.6 Property 13 (barcode) | done | Wei-En | `test_diff_barcode.py` |
| 10.7 field independence for contexts | skipped (minimum scope) | | Extends a test to contexts that are not all packed; `bitpack.h` itself is tested |
| 10.8 Parameter_Store | done | Wei-En | D15 |
| 10.9 discrete parameter reads | done | Wei-En | |
| 10.10 Property 22 (Parameter_Store value model) | skipped (minimum scope) | | `test_spec_conformance.py` compares all 55 defaults with the Baseline_Build library, and Property 24 guards the table |
| 10.11 barcode map `g_map_w[3]` | done | session 3 | The three words are stored in one dispatch-disabled section (see "Known differences"); `nav_map_get` is one load |
| 10.12 packed map unit tests | done | session 3 | `test_nav_map.py`, 243 tests; 23 of 23 mutants caught |
| checkpoint 11 | todo | | |

### Step 6: ISR_Shared_Data

| Task | Status | Commit | Note |
|---|---|---|---|
| 12.1 barcode edge ring | done | Wei-En, session 4 | The entry was packed on Wei-En. Session 4 added the 8-bit M7 indices and the plain header `isr_ring.h` |
| 12.2 raw ring, barcode flags, thresholds | done | Wei-En, session 4 | Session 4 added `g_bc_flags`, `g_bc_thr`, the 16-bit M9 indices and the `crit_irq` sections. `g_raw` keeps its one-word slot (see "Known differences") |
| 12.3 ADC range, echo, battery | done | Wei-En, session 3 | Session 3 added `g_adc_rng`, the D4 echo layout with `hal_us_arm` inside `crit_irq`, and `g_vbat_div`. The old note named only `g_vbat_div` |
| 12.4 driver flags | done | session 3 | `g_enc_rev`, `g_motor_flags`, `g_servo_rng`, `g_imu_cfg`; own-unit reasons are in `hal_isr.c` comments |
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
| 20.2 data_representation.md | done | session 5 | generated by `tools/evidence/gen_docs.py` from the headers, the drivers, `wire_dump` and the target compiler; 264 field rows, all 55 parameters, private variables, every skipped task, every known difference, R1 to R8. The script stops if a computed Required_Bits differs from a width in the code |
| 20.3 protocol.md | done | session 5 | generated from the `wire_dump` JSON: Format_Version, the 7 frame tables, event, reason, command, stop and abort codes, Parameter_IDs and the table CRC |
| 20.4 README link, host test README | done | session 5 | `README.md` 11.4 links `docs/data_representation.md`, `docs/protocol.md` and `docs/evidence/README.md`; `tests/host/README.md` unchanged |
| 20.5 host-side evidence sections | partly | session 5 | measured: `ram_usage.md`, `message_sizes.md` sections 1 and 3, `correctness.md` sections 1 and 2. Waiting for the car: `message_sizes.md` section 2 and the 25 % check, `correctness.md` sections 3 to 7, all of `performance.md` (each marked "to be measured" with its command) |
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
- The Status_Snapshot type is still `robot_status_t` (spec: `status_snapshot_t`). The 60-byte assert and the field asserts apply to it.
- `mev_ctx_t.still_ms` stays `uint32_t` as in the Baseline_Build (D5 says int32).
- `g_map_w`: D3 and the strategy table list it as single writer with one store per unit. The setters also put the three stores in one dispatch-disabled section, and `nav_map_actions` and `nav_map_describe` copy the three words in one, because TSK_NAV_PLANNER can preempt TSK_TELEMETRY between stores (Requirement 9.7). `nav_map_get` stays one load with no section.
- `g_map_w` is not `static`: `nav_map.h` declares it for the host tests, as with `g_done` and `g_buzz`.
- `g_motor_flags`: D3 names TSK_SAFETY_MON as the only writer. `app_main` also calls `hal_motor_kill` if start-up fails, before any task runs. Every write stores the whole unit, so this is safe.
- `g_enc_prev` is still `uint8_t [2]` (D4: one 8-bit unit). It is private to the GPIO ISR and was not in the 12.4 list.
- `g_raw[256]`: D4 and task 12.2 name three arrays (`g_raw_er`, `g_raw_el`, `g_raw_adc`). The ring keeps the one-word slot from Wei-En, which meets Requirement 5.5 (U12 ADC field, 256 entries) and uses 1,024 bytes instead of 2,560. The slot holds only the low 9 bits of each wheel count, so the reader rebuilds full counts, and after a ring overflow it counts no movement for one sample. The Baseline_Build kept exact counts there. Composing the slot is also what makes the raw path slower than the Baseline_Build (see session 4). Kept by the user's decision of 2 Nov 2026.
- `hal_barcode_config` clamps thresholds to 4095 to fit U12. The Baseline_Build stored them unclamped. No difference on the car: both thresholds lie between `IR_WHITE_ADC` and `IR_BLACK_ADC`, which are 0..4095, because the hysteresis parameters are at most 0.30.
- `isr_ring.h` is new (task 12.1 asks for a plain header but names none). `test_isr_ring.py` checks its helpers with examples and round trips; the FIFO model is still 12.5.

## Car checks waiting

| Check | From | Done | Result |
|---|---|---|---|
| Baseline measurements (`APP_PERF=1`, `docs/evidence/README.md` procedures) | FW-00 | no | |
| Session 1 build: boot, START, line following, ESTOP + BTN1 recovery, one barcode turn. Console and dashboard should look as before; the ESTOP ack's `pwm_off_us` should be a small number | FW session 1 | no | |
| Session 2 build: one hump (START, PEAK, END events, hump count and height in telemetry; a short slope gives a false-slope END), motion events on the dashboard as before, one obstacle bypass | FW session 2 | no | |
| Session 3 build: `APP_PERF=1` interrupt timing compared with the baseline; ultrasonic medians at 20 cm and 100 cm; servo sweep; barcode passes; a map sent from the dashboard and used at a junction | FW session 3 | no | |
| Session 4 build: barcode passes with `BARCODE_DETECT` 1 and 0 (same characters, no new `ovr` count); `APP_PERF=1` `isr_timer` worst case against the baseline, limit in Requirement 20.3: the larger of 110 % and baseline + 1 µs; `cs_irq` line at or below 10 µs | FW session 4 | no | |
| S5 met: FW-09 builds may be flashed (LT-05 merged: the tools send binary by default). After flashing, run `robot_params.py` and send the map once so the retained `is02/config` and `is02/map` are binary | LT-05 | no | |

## Session log

Newest first. One entry per session: date, tool, commit, gate results (host totals, `arm-none-eabi-size` line), open issues, car checks for the user.

- **2026-11-02, minimum CR1 session 5, Week 13 documents (Claude Code, local clone, Sonnet 5.5).** Branch `cr1/fw-rest`, commit `CR1 Week 13: data representation, protocol, RAM and size evidence`.
  - **Done:** 20.2, 20.3, 20.4 and 20.5 (partly). No firmware or `MQTT/` file changed; no item from `REVIEW_FLAGS.md` applied (their current decisions are listed in the data representation document).
  - **Generated:** `docs/data_representation.md` and `docs/protocol.md` by `tools/evidence/gen_docs.py` (new; run in WSL after `wire_dump` is built). Raw reports in `docs/evidence/raw/`.
  - **Gate (WSL):** `tools/evidence/build_check.sh -- -j8` on a clean worktree of `36a380b`: PASS, 0 warnings. `rm -rf tests/host/build && PYTHON=~/is02venv/bin/python bash tests/host/run_all.sh --seed 12345`: `Host_Test_Suite: PASSED (seed=12345)`; properties=543 failed=0, other tests passed=1489 failed=0 skipped=0 (same totals as session 4, no code changed).
  - **RAM (Baseline_Build `bca7964` against `36a380b`, both `make clean && make APP_ROBOT=1 APP_PERF=0 -j8`, gcc 13.2.1, worktrees with placeholder config):**

    | | text | data | bss | data + bss |
    |---|---|---|---|---|
    | Baseline_Build | 398,788 | 2,808 | 107,972 | 110,780 |
    | Revised_Build | 409,668 | 2,788 | 60,160 | 62,948 |
    | Saving | | 20 | 47,812 | **47,832 bytes, 43.2 %** |

    The revised `text` is 44 bytes above session 4's 409,624: the worktree sits in a different directory. This was not investigated; `text` is not part of Requirement 22.

    Items (baseline to revised): `g_status` 140 to 60, `g_mpf_buf` 4,000 to 1,600, `g_frames` 32,896 to 3,520, `g_msgs` 6,432 to 528, `g_bc_ring` 768 to 256, raw ring 2,560 to 1,024, `g_cmds` 768 to 192, `g_params` 220 to 160.
  - **R3:** `g_mpf_buf` is not the real pool. The kernel pool is 40 x 96 = 3,840 bytes before and 40 x 36 = 1,440 after (2,400 bytes saved, outside `.bss`), and the 2,400 bytes of `g_mpf_buf` in the `.bss` saving is a dead buffer. Both are in `ram_usage.md` section 6.
  - **Message sizes:** `motion` 32 to 12, `nav` 2 to 1, `obstacle` 64 to 24, `event` 92 to 32, pool block 96 to 36 bytes. Telemetry_Frame 432 bits, 54 bytes, 0 padding bits (`frame_budget.py`).
  - **Found while writing the documents (in the data representation, section 13):** `bc_ctx_t` uses C bit-fields; `det_t`, `adt_t` and `bcm_t` still hold `bool` members (R2); the message-slot consumer does `g_m_tail++` unmasked in `flush()` (same as the Baseline_Build); the property summary prints no line for P4, P5 to P8, P11 to P13, P17, P20 to P23.
  - **Waiting for the car (all marked "to be measured" in the files):** MQTT payload sizes per topic and the 25 % check (`message_sizes.md` 2 and 4); track runs, L1 to L6, functional checklist, encoder, barcode, ultrasonic, fault and E-stop checks, self-test (`correctness.md` 3 to 7); every table of `performance.md`.
  - **Open issues:** `performance.md` still shows its template's mis-encoded micro sign in the table headers (not touched). `ram_symbols.md` is marked superseded; its older numbers are not regenerated.
- **2026-11-02, minimum CR1 session 4 (Claude Code, local clone, Opus 5.5).** Branch `cr1/fw-rest`, commit `CR1 FW session 4: edge and raw rings, barcode flags and thresholds`.
  - **Done:** the rest of 12.1 and 12.2, which session 3 found marked `done` but missing from the code. No item from `REVIEW_FLAGS.md` applied.
  - **Gate (WSL, `PICO_SDK_PATH=$HOME/pico/pico-sdk`), all passed on the committed sources:**
    - `tools/evidence/build_check.sh -- -j8`: PASS, 0 warnings.
    - `APP_PERF=1`, `APP_LEGACY_TEXT_CMD=1`, `APP_SELFTEST=1`: each PASS with 0 warnings (through `build_check.sh`).
    - `rm -rf tests/host/build && PYTHON=~/is02venv/bin/python bash tests/host/run_all.sh --seed 12345`: `Host_Test_Suite: PASSED (seed=12345)`; properties=543 failed=0, other tests passed=1489 failed=0 skipped=0. Before this session: 543 and 1481. New: 7 tests in `test_isr_ring.py` and the `plain_header:isr_ring.h` static check.
  - **`arm-none-eabi-size`:** text 409624, data 2788, bss 60160, dec 472572. Before this session: 409580 / 2788 / 60176.
  - **Host trace of the drivers (not committed, as in session 3):** on 4 seeds, with `VBAT_ENABLED` 0 and 1, the new `hal_isr.c` gave output identical to the previous commit and to the Baseline_Build. Added for this session: all threshold settings on both polarities, 7,600 to 8,400 edges with random pops, edge-ring and raw-ring overflow, runs past the 7-bit, 8-bit and 9-bit index ranges, and `hal_barcode_raw` with `hal_barcode_config` in both orders. After an overflow the trace compares ADC values only, since the counts differ on purpose (see "Known differences"). An out-of-range threshold of 5000 did differ from the Baseline_Build, which is how the clamp was found; the firmware can't request one.
  - **Timer ISR against the Baseline_Build, counted from the disassembly (`hal_isr.c` at `bca7964` compiled with the firmware's command line; `BARCODE_DETECT = 1` is the default, so raw mode is the normal case; cycles from the instruction timings, not measured). From the ADC read to the return:**

    | Tick | Baseline_Build | Now |
    |---|---|---|
    | Threshold mode, no edge | 67 | 59 |
    | Threshold mode, edge | includes a 64-bit division call | that call is gone, and the path is shorter than at the session start |
    | Raw mode, ADC in range | 82 | **100** |
    | Raw mode, tick raises the ADC maximum | 83 | **108** |

  - **So in raw mode the timer ISR does more work than the Baseline_Build: 18 cycles on a normal tick (144 ns), 25 on a tick that raises the maximum (200 ns).** That breaks the "no more work" rule of the session 3 and 4 instructions, though by this count it is inside the Requirement 20.3 limit (the larger of 110 % and baseline + 1 µs). Almost all of it comes from composing the packed `g_raw` slot and the packed `g_adc_rng` word. Going back to the three arrays of D4 would remove the slot cost and add 1,536 bytes of RAM. **Decided by the user on 2 Nov 2026: keep the one-word `g_raw` slot.**
  - **Correction to session 3:** its "1 cycle less on a tick inside the ADC range" counted the `g_sample_n` saving against the ADC section only. Taken over the whole tick, and with the wrong base (the session start, not the Baseline_Build), it understated the cost. The table above replaces it. This session also made the ADC word cheaper, testing `rng == 0` in place of the `have` mask (2 cycles less per tick).
  - **Masked sections:** `hal_barcode_config` holds about 15 instructions and `hal_barcode_raw` about 35, loads, stores and mask operations only, with no loop or division (under 1 µs by count). `hal_barcode_config` takes its thresholds before the section, and `hal_barcode_raw` now does its whole reader reset inside one section.
  - **GPIO ISR:** this session did not touch it.
  - **Open issues:**
    - The `isr_timer` and `cs_irq` numbers above are counted, not measured, so the `APP_PERF=1` run on the car decides them.
    - Checkpoint 11 and tasks 12.5, 12.6 stay open.
  - **Car check (nothing above ran on a robot):** barcode passes with `BARCODE_DETECT` 1 and 0, `APP_PERF=1` `isr_timer` and `cs_irq` against the baseline. See "Car checks waiting".
- **2026-11-02, minimum CR1 session 3 (Claude Code, local clone, Opus 5.5).** Branch `cr1/fw-rest`, commit `CR1 FW session 3: ISR shared data, packed barcode map, map tests`.
  - **Done:** 12.3, 12.4, 10.11, 10.12. No item from `REVIEW_FLAGS.md` applied.
  - **Gate (WSL, `PICO_SDK_PATH=$HOME/pico/pico-sdk`), all passed on the committed sources:**
    - `tools/evidence/build_check.sh -- -j8`: PASS, 0 warnings.
    - `APP_PERF=1`, `APP_LEGACY_TEXT_CMD=1`, `APP_SELFTEST=1`: each PASS with 0 warnings (through `build_check.sh`).
    - `rm -rf tests/host/build && PYTHON=~/is02venv/bin/python bash tests/host/run_all.sh --seed 12345`: `Host_Test_Suite: PASSED (seed=12345)`; properties=543 failed=0, other tests passed=1481 failed=0 skipped=0. Before this session: 543 and 1238. The 243 new tests are all in `test_nav_map.py`.
    - One gate run stopped with exit code 4 during the `APP_SELFTEST=1` build because WSL restarted. It was re-run in full from the first build.
  - **`arm-none-eabi-size`:** text 409580, data 2788, bss 60176, dec 472544. Before this session: 409616 / 2796 / 60184.
  - **What 12.3 lacked:** the ledger named only `g_vbat_div`. In the code `g_adc_min` / `g_adc_max` were also separate and reset with no section, `hal_us_arm` had no section and did not clear `g_echo_rise_us`, and the echo bits were not the D4 ones.
  - **Echo:** `hal_us_read_cm` still returns exactly `(float)us / 58.0f` and -1 above 25,000 µs. Widths above 25,000 µs are now stored as 25,001 (was 32,767).
  - **Host trace of the drivers (not committed):** `hal_isr.c` from the Baseline_Build and from this commit were compiled against stand-in SDK headers and run on the same scripted edges and samples. Echo widths (including 24,999 to 25,002, 32,767 and 32,768, about 20,000 random widths, no echo, a stray fall), 5,000 ADC windows (including empty ones and the values 0 and 4095), all four encoder reversal settings with the raw ring, and the VBAT reading ticks and values gave identical output on 4 seeds, with `VBAT_ENABLED` 0 and 1. This shows the logic only, not timing or hardware.
  - **Interrupt work, counted from the Cortex-M0+ disassembly (`APP_PERF=0`, cycles from the instruction timings, not measured). The timer ISR line below was too low: see the correction in session 4.**
    - Timer ISR: 1 cycle less on a tick inside the ADC range (22 against 23), which is the usual tick. **A tick that raises the maximum costs 4 cycles more** (28 against 24, about 32 ns), and the raw-ring path about 1 more. So the worst single tick is about 5 cycles above the baseline. That is the one case where the interrupt does more than before; `g_sample_n` and its division are gone, which covers the rest.
    - GPIO ISR: the encoder path is unchanged; the echo paths are the same or shorter. `hal_gpio_isr` is the same length as before.
    - `hal_imu_int1_used`: 2 instructions shorter, because of the chip codes only the DLHC has bit 0 set.
    - Masked sections: `hal_us_arm` and `hal_ir_adc_range` hold only loads and stores, with no loop or division.
  - **Test strength:** 23 mutants of `nav_map.c` were run against `test_nav_map.py`. 22 failed the tests and 1 did not compile. They include storing a word outside the section, copying outside it, storing before the final checks, and OR-ing an action in place of replacing it.
  - **Open issues:**
    - **12.1 and 12.2 are marked `done` above, but the code does not match them.** `g_bc_head` / `g_bc_tail` and `g_raw_head` / `g_raw_tail` are still 32-bit. `g_bc_enabled`, `g_bc_black_high`, `g_bc_black` and `g_raw_enabled` are separate `bool`s, not `g_bc_flags`. The thresholds are two units, not `g_bc_thr`. `hal_barcode_config` and `hal_barcode_raw` use no `crit_irq` section (Requirements 5.6, 5.7, 10.1). There is no plain ring header. `g_raw` is one packed word per sample, unlike D4's three arrays. Not in this session's tasks, so left unchanged and the rows not edited.
    - Interrupt timing is only counted, not measured: the `isr_timer`, `isr_gpio` and `cs_irq` lines of an `APP_PERF=1` run give the real numbers.
    - Checkpoint 11 and tasks 12.5, 12.6 stay open.
  - **Car check (nothing above ran on a robot):** `APP_PERF=1` interrupt timing compared with the baseline; ultrasonic medians at 20 cm and 100 cm; servo sweep; barcode passes; a map sent from the dashboard and used at a junction. See "Car checks waiting".
- **2026-11-02, minimum CR1 session 2 (Claude Code, local clone, Opus 5.5).** Branch `cr1/fw-rest`, commit `CR1 FW session 2: IMU contexts, Properties 14 and 15, status asserts`.
  - **Done:** 10.2, 10.4, 10.5, 6.1, and 6.8 (it was quick). No item from `REVIEW_FLAGS.md` applied. R8 did not arise: `obstacle_logic.c/.h` and `imu_logic.c/.h` matched the Baseline_Build exactly before this session.
  - **Gate (WSL, `PICO_SDK_PATH=$HOME/pico/pico-sdk`), all passed:**
    - `tools/evidence/build_check.sh -- -j8`: PASS, 0 warnings.
    - `APP_PERF=1`, `APP_LEGACY_TEXT_CMD=1`, `APP_SELFTEST=1`: each PASS with 0 warnings (through `build_check.sh`).
    - `rm -rf tests/host/build && PYTHON=~/is02venv/bin/python bash tests/host/run_all.sh --seed 12345`: `Host_Test_Suite: PASSED (seed=12345)`; properties=543 failed=0, other tests passed=1238 failed=0 skipped=0. Before this session: 539 and 1212.
    - Property 14: 2,000 cases (1,000 free scans, 1,000 shaped scans), 0 failed. Property 15: 2,000 cases (1,000 held pitch runs, 1,000 free samples), 0 failed. No failure, so nothing to trace in the harness or the code.
  - **`arm-none-eabi-size`:** text 409616, data 2796, bss 60184, dec 472596. Before this session: 409356 / 2796 / 60184. `.bss` doesn't move because both contexts are locals of `tsk_imu`, so the saving is on its stack (host sizes: `hump_ctx_t` 28 to 16 bytes, `mev_ctx_t` 16 to 8).
  - **Test strength:** 24 mutants of `imu_logic.c/.h` and `obstacle_logic.c` were run against the two new test files. 23 were caught. The other, a state setter limit of 7, is equivalent: `bp_set_checked` still refuses anything above the 2-bit field.
  - **Counters:** no event sequence can show a counter wrap, because the thresholds are met first, so Property 15 also compares each packed counter with `min(baseline, 5)`.
  - **`app_status.c`:** the literal shifts became the `ST_SH_*` / `ST_W_*` constants, so the asserts check the real layout. The object code for the Cortex-M0+ is identical to the previous commit (`objdump -d` compared).
  - **`status.h`:** now includes `line_logic.h` and `wire_ids.h` for `LINE_LOST` and `STOP_REASON_COUNT`, both plain headers. The stale `faults U13` comments there and in `app_types.h` were corrected.
  - **Added beyond the list, to check 10.2:** `stubs/imu_export.c`, the `mev_step` differential and the setter range tests (Requirements 2.11, 2.12).
  - **Open issues:**
    - Property 14 does not run `side_view()` or the leg logic in `tsk_obstacle.c`, which are kernel code. The diff from the Baseline_Build there is events, status accessors, parameter reads and `o.dist_mm`; `st_*` values are only written, never read back into the planner.
    - The task reads `o.dist_mm` from the odometry record, quantized to 0.01 mm (Wei-En work, D3.4), where the Baseline_Build averaged two floats. It feeds the bypass dead reckoning. Not changed here, and small against the 0.081 mm per count.
  - **Car check (nothing above ran on a robot):** one hump, motion events, one obstacle bypass; see "Car checks waiting".
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
