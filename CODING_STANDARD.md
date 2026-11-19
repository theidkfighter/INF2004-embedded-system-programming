# Coding standard

The application firmware in `app_program/` follows the **Barr Group Embedded C Coding Standard (BARR-C:2018)**. This file lists every place where it deliberately does not, with the reason. Anything not listed here is meant to comply; a violation that is not listed is a bug in the code or in this file.

Scope: `app_program/` (application, drivers, tasks). Not in scope: the µT-Kernel sources (`kernel/`, `include/`, `etc/`, `device/`), the Pico SDK, lwIP and the MQTT client (`MQTT/`), and generated or tool code (`tools/`, `tests/`). Those keep their own style.

Counts below were taken on the `barr-c-cleanup` branch on 19 Nov 2026.

## How the code is checked

- `make APP_ROBOT=1` in `build_make/` with `APP_PERF=0` and `APP_PERF=1`: 0 errors, 0 warnings (`-Wall -Wextra` and the project flags).
- Host tests (`tests/host`): pytest + Hypothesis properties and `node --test`, run against the same sources with `HOST_TEST` defined.
- Behaviour-neutral clean-ups (constants, renames, declarations) are checked by identical `arm-none-eabi-size` text/data/bss before and after.

## Project conventions that support the standard

| Convention | Where | Rule served |
|---|---|---|
| `TEST_VISIBLE` (`test_visible.h`): `static` in the firmware, empty when `HOST_TEST` is defined, so the host tests can reach a unit the firmware keeps file-local. Functions that only the host tests call are built only under `#ifdef HOST_TEST`. | `app_shared.c`, `nav_map.c`, `wire.c`, `os_glue.c`, `imu_logic.c`, `obstacle_logic.c`, and the `#ifdef HOST_TEST` blocks in `wire.c`, `app_params.c`, `app_status.c`, `code39.c`, `msg_codec.c`, `nav_map.c`, `imu_logic.c` | 1.8.a, 4.2.c |
| `crit_irq_enter()`/`crit_irq_exit()` and `crit_dsp_enter()`/`crit_dsp_exit()` (`crit.h`) for every interrupt-masked or dispatch-disabled section; they also record `PERF_CS_IRQ` / `PERF_CS_DSP` when `APP_PERF=1`. | all shared data | 8.x / CR1 |
| Task entry points end in `_task` (`app_tasks.h`), interrupt handlers end in `_isr` (`hal_isr.h`, `cyc_tick_isr`). | `tsk_*.c`, `hal_isr.c`, `os_glue.c` | 6.4.a, 6.5 |
| `bp_is_nan()` (`bitpack.h`) for NaN tests. | 8 call sites | 5.4.b.v (see deviation 12) |

## Deviations

| # | Rule | Deviation | Reason |
|---|---|---|---|
| 1 | 1.2.a | Lines up to 120 columns, not 80 (1,432 lines over 80; 64 over 120). | Most long lines carry an explanatory trailing comment. Wrapping them all would change most files shortly before submission. |
| 2 | 6.2.a | 12 functions over 100 lines: `tsk_imu_task` 339, `avoid` 231, `run_bypass` 210, `tsk_safety_task` 156, `tsk_motion_task` 138, `follow` 127, `hal_timer_isr` 120, `leg_side_follow` 119, `obstacle_profile` 113, `wire_dec_ack` 107, `tsk_line_task` 106, `nav_map_parse` 104 (host tests only). | Task bodies and the bypass state machine keep their full sequence in one place for review against the design. Splitting them is a separate change that needs its own track runs. |
| 3 | 8.6.a | The variable is on the left of `==` / `!=` (about 180 comparisons against constants). | Consistent across the project. `-Wall` (`-Wparentheses`) warns about `=` used where `==` was meant, which is the risk the rule guards against. |
| 4 | 5.2.a | µT-Kernel types (`INT`, `UB`, `UW`, `ID`, `TMO`, `ER`, `FP`) and Pico SDK `int` / `uint` at API boundaries. | Required by those APIs. Application data uses `<stdint.h>` types. |
| 5 | 6.5, MISRA 11.1 | `(FP)` casts when registering handlers in `os_glue.c`. | Required by the µT-Kernel `T_DINT` / `T_CCYC` API and ABI-identical on arm-none-eabi. The "tell the compiler it is an ISR" part is met by Cortex-M hardware stacking and the kernel's `TA_HLNG` wrapper. |
| 6 | 3.5.a and others | `usb_console_compat.h` (tabs, stub returns). | Third-party template code, kept in its original style. |
| 7 | 6.2.d | `usermain` has no prototype in an `app_program/` header. | It is declared by the µT-Kernel template. |
| 8 | 6.1.i | Most public functions do not carry their module's exact name as a prefix (`buzz_put`, `post_motion`, `st_*`, `hal_*` spread over three files, ...). | Stable API used across all tasks; renaming would touch most files with no functional gain. |
| 9 | 1.6.a, 2.2.h, 5.4.b.i, 6.2.c, 1.7.d, 7.1.e, 7.1.f, 7.1.j, 4.2.a, 4.2.c.i | Casts without a comment; no per-function Doxygen block (each file has an `@file` header and most functions a plain comment); `float` rather than a `float32_t` typedef; functions with several `return`s; some `continue`; short loop and pointer names (`i`, `n`, `p`); upper-case `const` tables (`PARAMS`, `Q_*`, `T_*`), treated as named constants like macros; `.c` files without a matching `.h` (`tsk_*.c` share `app_tasks.h`, `app_shared.c` uses `shared.h`, `hal_*.c` share `hal.h`) and headers without a `.c`; `extern` variables in headers for the 12 true globals (`g_params`, `g_status`, the 8 kernel object IDs, `g_hb`, `g_reacq_seq`). | Each would touch most of the code base shortly before submission. The globals are justified one by one in the review notes: one writer each, or written once before the tasks start. |
| 10 | 5.4.b.iv | Exact float comparisons: `obstacle_logic.c` `theta_deg == 45.0f` (twice), `speed_ctl.c` `target == 0.0f`, `tsk_nav.c` `deg == 0.0f`, `tsk_obstacle.c` `!= 0.0f` (twice), `app_params.c` `(float)i != v`. | Each compares against a value that was assigned exactly that literal, or tests exactness on purpose. `theta_deg == 45.0f` is the deliberate bit-identical short-circuit of `OBST_DYNAMIC_TURN`. Changing them would change behaviour. |
| 11 | 1.1.d, 3.1.j, 3.3.c, 1.3.b | `LOCAL` / `IMPORT` / `EXPORT` from the µT-Kernel template (180 `LOCAL`, 5 `IMPORT`/`EXPORT`); no space between a function name and `(` in definitions; no `/*** end of file ***/` comment; some one-line getter bodies (`app_status.c`, `app_net.c`, `bc_detect.c`). | Template convention and whitespace only (Phase D of the clean-up plan, not done). `LOCAL` is `#define LOCAL static` in `tk/typedef.h`, so linkage is correct. |
| 12 | 5.4.b.v | NaN is tested with `bp_is_nan()` (`__builtin_isnan`), not `isfinite()`. | The existing checks reject NaN only. `isfinite()` would also reject ±infinity and so change which inputs are accepted; the clean-up keeps behaviour unchanged. `<math.h>` cannot be included in the kernel-side files (its `size_t` clashes with µT-Kernel's), so the GCC builtin is used, wrapped in one inline function in `bitpack.h`. |
| 13 | 6.3.a | Function-like macros kept: `PERF_BEGIN`, `PERF_END`, `PERF_CYCLE_END`, `PERF_BARRIER` (`perf.h`); `BP_GET`, `BP_PUT`, `BP_MASK`, `BP_STATIC_ASSERT` (`bitpack.h`); `ST_FITS`; `PRM_DESC`; `SERVO_RNG`; `IMU_CFG`; `BC_THR`; `ENC_REV_BIT`; the length macros `WIRE_BYTES`, `WIRE_CFG_LEN`, `WIRE_CFG_ACK_LEN`, `WIRE_EVT_LEN`, `WIRE_CMD_LEN_OF`, `WIRE_ACK_LEN_OF` (`wire.h`); the X-macros `WIRE_*_TABLE` / `WIRE_X_*` (`wire_ids.h`). | They are used where C needs a constant expression (array sizes, static initialisers, `case` labels, `_Static_assert`), compile to nothing when `APP_PERF=0`, or generate parallel tables from one list. Helpers that did not need this were turned into `static inline` functions during the clean-up (`done_id`, `done_result`, `map_sh`, `adc_rng`, `bp_is_nan`). |
| 14 | 1.8.c, 8.x | `__compiler_memory_barrier()` (Pico SDK) in the single-producer / single-consumer rings of `hal_isr.c`, next to `volatile` indices. | `volatile` alone does not order the stores to the slot before the store to the index. The barrier is a compiler barrier only: the producer (interrupt) and the consumer (task) run on the same core, so no hardware barrier is needed. |
| 15 | 8.4.a and magic numbers | Loop limits are named constants or the array's size constant. Literal `2.0f` / `2u` remain where they mean "twice" or "half" (the reacquire sweep table in `tsk_obstacle.c`, the 1-2-1 smoothing in `bc_detect.c`, the 1.5 x wide/narrow ratio `* 2u < * 3u` in `code39.c`), as do `0`, `1` and `0.5f` rounding. | These are arithmetic, not tuning values; a name would hide the formula. |

## Changing this file

Add a row when new code has to deviate, with the reason, in the same commit as the code. Remove a row when the deviation is fixed.
