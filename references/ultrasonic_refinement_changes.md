# Ultrasonic obstacle bypass refinement (branch `final_ultrasonic_refinement`)

Changes compared with `main` (`b5a907b`). Tested on the track on 4 Nov 2026:
runs logged in `is02_dashboard_2026-11-04081329`, `...083616` and `...085515`
(with `MQTT/logs/telemetry_20261104_154153`, `_163403`, `_165232`).

## Summary

| Area | `main` | This branch |
|---|---|---|
| Standoff reverse | full 45 deg standoff (e.g. 138 mm) | 0.75 x that (e.g. 104 mm) |
| First bypass turn | always 45 deg | smallest angle in 30-70 deg that still clears the near corner (about 51 deg in the test runs) |
| Legs after the turn | sized for 45 deg | sized for the chosen angle (same sideways shift as before, about 23 cm) |
| Return to the line | accepts any line from the start of the return leg | ignores the line until the car is back within max(50 mm, 25 % of the distance out) of the original path |
| After turning parallel | hands over to line following | if the left sensor is off the line: pivot up to 25 deg towards the path side, then 25 deg the other way, stopping on the line |

Everything new is behind `OBST_DYNAMIC_TURN` in `app_program/robot_config.h`.
With `OBST_DYNAMIC_TURN (0)` (or `OBST_LEGACY` = 1) the robot runs the `main`
code path exactly: the original functions (`standoff()`, `obstacle_standoff_mm`,
`bypass_plan`, the 45 deg sidestep) are not edited.

## New settings (`app_program/robot_config.h`, compile time)

| Define | Value | Meaning |
|---|---|---|
| `OBST_DYNAMIC_TURN` | 1 | 1 = everything below, 0 = `main` behaviour |
| `OBST_REVERSE_SCALE` | 0.75f | fraction of the 45 deg standoff reverse to drive (0.5 gave 57-59 deg turns) |
| `OBST_TURN_MIN_DEG` / `OBST_TURN_MAX_DEG` | 30 / 70 | range of the first turn angle |
| `OBST_RETURN_GATE_MIN_MM` / `OBST_RETURN_GATE_FRAC` | 50 / 0.25 | return gate window: max(min, frac x distance out) |
| `OBST_REACQ_SWEEP_DEG` | 25.0f | pivot search after turning parallel; 0 = off |

`OBST_US_AHEAD_MM` (80 mm) is still a placeholder; the planner also uses it for
the pivot-swing guard on turns above 45 deg. Measure it on the car.

## How it works

1. **Reverse and angle** (`obstacle_dyn_standoff`, `app_program/obstacle_logic.c`):
   the original 45 deg standoff `need45` is still computed from the measured
   profile; the robot reverses `OBST_REVERSE_SCALE x need45` (skipped under 5 mm,
   capped at `OBST_BACKOFF_MAX_MM`). It then tries whole degrees from 30 to 70
   and takes the first whose first leg passes the near corner by half the car's
   width + margin (`y sin(theta) - e cos(theta) >= half + margin`). If none does,
   it reverses only the extra the best angle needs, never more than `main` would.
   Steep turns also check that the car's front corner clears the obstacle while
   pivoting. The total reverse is checked once against `OBST_REVERSE_MAX_MM`.
2. **Legs** (`bypass_plan_dyn`, `obstacle_leg_for_shift`): leg 1 = shift / sin(theta),
   the parallel leg allows for the forward distance leg 1 covered
   (shift x cot(theta)), the sidestep re-entry uses the same angle. At 45 deg both
   return the `main` values bit for bit.
3. **Task wiring** (`app_program/tsk_obstacle.c`): `g_dyn` is read at the start of
   each avoidance; `standoff_dyn()` replaces `standoff()` and `bypass_plan_dyn()`
   replaces `bypass_plan()` only when it is set (three marked call sites).
4. **Return gate**: the return leg is split in two. The first part drives (with
   the obstacle watch on) until the dead-reckoned sideways offset is inside the
   gate window; then the line watch is re-armed for the rest. A dark mark early
   on the return is no longer taken as the line.
5. **Line search after realigning**: once parallel again, the left sensor (the
   one line following tracks) is checked. On this car it reads 1 when it is not
   on the line. If so, the car pivots towards the path side (where the usual
   short miss leaves the line), then the other way, and stops as soon as a
   sensor crosses the line, leaving the small angle to line following.

Tried and removed: driving an extra 25 mm onto the line before turning parallel
(it overshot on a wider pass).

## Files

| File | Change |
|---|---|
| `app_program/obstacle_logic.h/.c` | new planner functions and types, in a delimited `OBST_DYNAMIC_TURN` block (no existing line edited) |
| `app_program/robot_config.h` | the new defines above |
| `app_program/tsk_obstacle.c` | `g_dyn`, `standoff_dyn()`, the marked call sites, the return gate, the line search |
| `tests/host/py/test_obst_dynamic_turn.py` | host property tests for the planner (clearance, reverse bound, monotonic angle, leg sizing, pivot swing, 45 deg = `main`) |
| `tests/host/baseline/base_motion.c`, `fetch_baseline.sh` | harness fixes so the Baseline_Build library builds against `main` (and on Windows/MinGW) |

Host tests: `test_obst_dynamic_turn.py` + `test_prop14_obstacle.py` pass (26 tests,
Property 14 against `main` unchanged).

## Reverting

- Compile time: `#define OBST_DYNAMIC_TURN (0)` in `app_program/robot_config.h`, rebuild.
- Only the line search: `OBST_REACQ_SWEEP_DEG (0.0f)`.
- Git: revert the commits on `final_ultrasonic_refinement`.
