# FW-03: Status_Snapshot (spec Step 3)

| | |
|---|---|
| Track | Firmware (branch `cr1/fw`) |
| Tasks | 6.1–6.8, checkpoint 7 |
| Needs first | FW-02 done. Flag R5 decided (rounding), or follow the spec |
| Model | **Opus** (touches every task file in one commit) |
| Budget | about 1 Claude window / 150–250 Kiro credits |
| Car | **yes**: checkpoint 7 bench check, which also covers FW-02 |

## Read

- `spec/design.md`: "C4. `status.h`", "D2. Status_Snapshot" (the units, shifts and widths), "Critical-section strategy", Properties 4, 5, 6, 8.
- `spec/requirements.md`: Requirements 5.1–5.3, 6.1–6.3, 6.8–6.9, 7.1–7.6, 9.1–9.5.
- The code: `app_types.h` (`robot_status_t`), `app_status.c`, every `g_status.` user (`grep -n "g_status\." app_program/*.c`).

## Work

1. **6.1 `app_program/status.h`:**
   - `status_snapshot_t` with the D2 Storage_Units (`mot_a`..`mot_e`, `line_a`, `line_b`, `run`, `imu_a`, `imu_b`, `obs_a`..`obs_c`, `imu_c`, `vbat`, `fault_mask`, `sys`), the shift and width constants, the setters and getters (C4), and `st_view_t`.
   - Static asserts: size 60, each field fits, `LINE_LOST < 8`, `MEV_COUNT <= 8`, `SYS_STATE_COUNT <= 8`, `STOP_REASON_COUNT <= 16`, each naming its field.
   - Plain C only.
2. **6.2 accessors in `app_status.c`:**
   - Single-writer setters compose the unit in a local and store it once.
   - The read-modify-write setters (`st_run_*`, `st_put_imu`, `st_set_mev`, `st_put_hump_peak`, `st_hump_end`, `st_hump_max_reset`) work inside `crit_dsp_*`.
   - Quantise with `bp_quant` and call `sat_note(n)`.
   - `st_put_obst` / `st_put_scan` store 8191 for a negative reading. `st_put_side` accepts only 0, `'L'`, `'R'`.
   - Counters use `bp_sat_inc` at 63 / 15 / 15.
   - Getters do one load per unit; `st_read()` copies every unit with one load each.
   - Add `app_status.c` to the host library.
3. **6.3 convert every writer and reader** in `tsk_motion.c`, `tsk_line.c`, `tsk_imu.c`, `tsk_obstacle.c`, `tsk_nav.c`, `tsk_safety.c`, `tsk_telemetry.c` and `os_glue.c`.
   - Line sensors store −1 when `LINE_DO_BLACK_HIGH < 0`.
   - The JSON frame builder reads through `st_read()`. Safety recheck uses `st_obst_cm()`. `complete_run()` uses the counter getters. `BC_DBG` uses `st_thr_*()`.
   - Remove `robot_status_t`. No direct `g_status.` access may remain outside `app_status.c`.
4. **Tests:**
   - 6.4 Property 4 (field independence, at least 1,000 sequences per unit);
   - 6.5 Property 5 (invalid discrete writes);
   - 6.6 Property 6 (saturating counters);
   - 6.7 Property 8 (line sensor codes);
   - 6.8 `test_compile_fail.sh`.
5. **Checkpoint 7:** gate passes. Write the bench check below into "Car checks waiting".

## Watch out

- **The JSON telemetry must stay the same** within the published decimals (Requirement 19 / design Step 3). If R5 is undecided, follow D0 (double rounding) and list any value that prints differently from the baseline in the session log.
- **`obst_cm` / `scan_cm` with no echo:** the getter must return −1.0, so `recheck()`'s `cm < 8` test behaves exactly as before.
- **`app_status.c` is pure logic.** It may include `crit.h` but no kernel header.
- **Flag R8:** obstacle maths unchanged. Only the status writes in `tsk_obstacle.c` change.

## Gate

The firmware gate in `CONTEXT.md` section 5. In the session log, also give `grep -c "g_status\." app_program/*.c` (only `app_status.c` may match).

## Car check (user, before FW-05a starts)

Flash an `APP_PERF=0` build with your real config and check:
- it boots to IDLE;
- START, then line following for one lap;
- dashboard ESTOP, then BTN1 recovery;
- the dashboard telemetry values look the same as before. The only difference allowed is the `faults` value, which shows bits 13–14 at boot until MQTT is up, and bit 15 with no echo.

Record the result in `PROGRESS_FW.md`.

## Hand-over

- **Commit:** `CR1 FW-03: Status_Snapshot`.
- Session log, including the size line. bss should drop by about 80 bytes.
