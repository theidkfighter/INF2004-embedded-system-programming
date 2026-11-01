# FW-05a: Controller contexts and task-file flags (spec Step 5, first half)

| | |
|---|---|
| Track | Firmware (branch `cr1/fw`) |
| Tasks | 10.1–10.7 |
| Needs first | FW-04 done |
| Model | Sonnet (Opus if Properties 13–15 fail and need debugging) |
| Budget | about 1 Claude window / 100–180 Kiro credits |
| Car | no |

## Read

- `spec/design.md`: "D5. Controller contexts and task-file flags", "D16. Private_Control_Variables", Properties 4, 13, 14, 15.
- `spec/requirements.md`: Requirements 2.2–2.4, 2.6–2.8, 2.12, 5.9, 6.7, 7.3, 7.7.
- `REVIEW_FLAGS.md` R2 (barcode context bools), R8 (obstacle).
- The code: `line_logic.h/.c`, `imu_logic.h/.c`, `obstacle_logic.h/.c`, `tsk_line.c` (`g_searching`, `g_det_on`, `g_path_init`), `tsk_obstacle.c` (`g_legacy`, `g_why`), `tsk_motion.c` (`primitive_t`); `tests/host/baseline/`.

## Work

1. **10.1:**
   - `line_ctx_t.f` (state, side, `b_last_curve`, `b_both_prev`) and `search_ctx_t.f` (dir, `b_second`), with inline get/set that reject out-of-range values. Update `line_logic.c`.
   - `g_line_flags` replaces `g_searching`, `g_det_on` and `g_path_init`.
2. **10.2:**
   - `hump_ctx_t.f`: state E2, plus `n_up` / `n_down` / `n_level` U3 saturating at 5.
   - `mev_ctx_t.f`.
   - Static assert `max(HUMP_START_SAMPLES, HUMP_END_SAMPLES) <= 7`.
3. **10.3:**
   - `us_filter_t.f` and `bypass_plan_t.f` (side S2).
   - `g_obs_flags` and `g_why` (an abort-reason code E4, using the 11 abort reasons in `wire_ids.h`). `abort_avoidance()` takes the code.
   - `primitive_t.f`.
   - **Continuous members stay float32** (D16).
4. **The differential tests against the Baseline_Build** (`libbase.so`):
   - 10.4 Property 15 (hump);
   - 10.5 Property 14 (obstacle planning);
   - 10.6 Property 13 (barcode: all 43 characters forward and reversed, edge and raw modes, plus random sequences).
5. **10.7:** add the new context units to the Property 4 test.

## Watch out

- **Property 14 compares with the frozen Baseline_Build** (`bca7964`). It must pass unchanged. Don't "fix" obstacle behaviour here (flag R8).
- **`us_filter_t` is in `obstacle_logic.h`.** Its `n` / `below` change to packed fields, but `usf_*` must return identical values.
- **Flag R2:** the barcode decoder contexts (`bc_ctx_t`, `det_t`, `adt_t`, `bcm_t`) are **not** in this step; they're optional task 18.8 (FW-09). If R2 option (a) is accepted, FW-09 does them; don't start them here.
- **`bypass_plan_t.side`** is only ±1 in practice, but the field is S2, per Requirement 2.7.

## Gate

The firmware gate in `CONTEXT.md` section 5. Properties 13, 14 and 15 must be in the summary with 0 failures.

## Hand-over

- **Commit:** `CR1 FW-05a: packed controller contexts and task flags`.
- Session log with the Property 13–15 case counts.
