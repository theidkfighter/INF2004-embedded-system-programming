# FW-04: Shared scalars, odometry, done word, packed mailboxes (spec Step 4)

| | |
|---|---|
| Track | Firmware (branch `cr1/fw`) |
| Tasks | 8.1–8.8, checkpoint 9 |
| Needs first | FW-03 done, and its car check done or explicitly postponed by the user. Flags R2 (the `tsk_safety.c` / `app_net.c` bools) and R3 (`TA_USERBUF`) decided, or follow the spec |
| Model | **Opus** (concurrency, motion path) |
| Budget | about 1 Claude window / 150–250 Kiro credits |
| Car | **yes**: checkpoint 9 (bench: a turn and an obstacle bypass) |

## Read

- `spec/design.md`: "C5", "C6", "D3. Other Task_Shared_Data" (with D3.4 and D3.5), "D6" (the motion, nav and obstacle bodies and the Message_Pool), Properties 11 and 18.
- `spec/requirements.md`: Requirements 2.1, 2.5, 2.9, 2.10, 4.1–4.6, 4.9, 4.10, 8.1, 8.4–8.8, 9.1–9.8.
- `REVIEW_FLAGS.md` R2, R3.
- The code: `os_glue.c/.h` (`sys_set_state`, `g_stop_reason`, `buzzer_*`, `g_hb`, `g_reacq_seq`, the message pool), `tsk_safety.c` (`enter_estop`, `enter_error`, `g_estop_*`), `tsk_telemetry.c` (`g_estop_ack_due`, `g_config_due`), `tsk_motion.c` (`finish`, `motion_wait`, odometry), the readers of `odometry_get` in `tsk_line.c`, `tsk_imu.c`, `tsk_obstacle.c`.

## Work

1. **8.1 system word and stop reasons:**
   - `sys` holds state E3 + stop_reason E4.
   - Add `stop_reason_t`, `stop_reason_set/get`, and `stop_reason_text()` with the **exact** baseline strings. There are 9 today: `grep -n 'enter_estop(\|enter_error(' app_program/tsk_safety.c`.
   - `enter_estop` / `enter_error` take the code; the console prints the text.
2. **8.2:**
   - `g_buzz`: 8-bit, with `buzzer_pattern_take` inside one critical section.
   - `g_hb[]` and `g_reacq_seq`: `volatile uint8_t`, with the same safety-check meaning.
   - `g_next_id`: 16-bit.
   - `g_net_flags` (estop_pending, estop_ack_due, config_due) and `g_estop_rx_us`, written together in one dispatch-disabled section.
   - `g_estop_pwm_off_us`: U16, saturating.
3. **8.3:**
   - `g_done` (id [0..15], result [16..17]) is written with one store and read with one load.
   - `odo_rec_t` (D3.4): `odometry_put` / `odometry_get` copy it inside one dispatch-disabled section.
   - `odometry_t` drops `enc_r` / `enc_l`, and `dist_r_mm` / `dist_l_mm` become `dist_mm`. Update the readers.
4. **8.4 mailboxes:**
   - `app_msg_body_t` holds `motion_msg_t`, `nav_msg_t`, `obst_msg_t` (plus `tlm_event_t` until FW-08a). Renumber `cmd_src_t` from 0.
   - The post functions keep their signatures and encode inside; receivers decode.
   - `g_prof` stays unchanged.
   - Keep the four mailboxes, 40 blocks, `TMO_POL` and release-after-copy.
5. **8.5:** the `APP_SELFTEST` concurrency self-test in `os_glue.c`.
6. **Tests:**
   - 8.6 Property 11: motion targets vs `libbase.so`, within ±1 count and ±1 ms;
   - 8.7 Property 18: critical sections balanced and covering every shared update;
   - 8.8: the `stop_reason_text` table and `test_pool_empty.py`.

## Watch out

- **Odometry readers in `tsk_obstacle.c`:**
  - **Problem:** `pose_update()` / `odo_fwd_mm()` use `0.5 * (dist_r_mm + dist_l_mm)` today.
  - **Fix:** after 8.3 they read `dist_mm`. It must be numerically the same mean, so the obstacle planner output stays identical (Requirement 6.7, Property 14, flag R8).
  - **Check:** Property 14 runs in FW-05a, so check it by hand here with a short differential run.
- **Flag R3:** if accepted, create the pool with `TA_USERBUF` on `g_mpf_buf`, sized exactly `MSG_BLOCKS × block size`, a multiple of `sizeof(FREEL)`. If not accepted, leave the pool creation as it is and add a sentence to the session log that `g_mpf_buf` is unused (for `ram_usage.md`).
- **Flag R2:** if accepted, also pack `g_estop_reported` (`tsk_safety.c`) and the `app_net.c` file-scope bools (`g_led`, `g_led_applied`, `g_connecting`, `g_ever_connected`, `g_join_pending`, `g_broker_set`). Different writers must not share a unit without a critical section (Requirement 9.1).
- **`stop_reason_text`** must reproduce each string byte for byte. The event texts depend on it later.

## Gate

The firmware gate in `CONTEXT.md` section 5, plus one build with `APP_SELFTEST=1` (0 warnings).

## Car check (user, before FW-05b starts)

- **`APP_SELFTEST=1` build** (USB console, wheels off the ground): the console prints `[selftest] PASS`.
- **`APP_PERF=0` build:**
  - boot, START, a barcode turn (`BARCODE:<letter>` from the dashboard on a junction), an obstacle bypass on a straight, ESTOP and recovery;
  - the console stop-reason line reads exactly as before.

## Hand-over

- **Commit:** `CR1 FW-04: shared scalars, odometry, done word, packed mailboxes`.
- Log the size line. bss should drop by a few hundred bytes; the pool shrinks only if R3 was applied.
