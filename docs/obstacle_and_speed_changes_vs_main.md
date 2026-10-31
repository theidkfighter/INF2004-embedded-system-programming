# Changes on `codex/ultrasonic-refinement` compared with `main`

This document is for someone who knows the `main` branch and wants to understand what this branch changes. It is written so that it can be read on its own, or handed to an AI assistant together with questions about the code.

- **Branch:** `codex/ultrasonic-refinement`.
- **Compared with:** `main` at commit 8b16259 ("add setup.md"). This is the commit the branch was made from.
- **Newer `main` commit:** `origin/main` has one newer commit, 4bec241 ("added setup along with creating flash file"). It changes `README.md`, `.gitignore`, removes the Eclipse files (`.cproject`, `.project`, `.settings/`) and adds `build_make/tools/.gitkeep`. It does not touch any firmware code, so everything below applies to it too. See "Merging into main" at the end.
- **State:** the obstacle avoidance and servo code is frozen at the version used in the last test run on 31 Oct 2026 (dashboard log `is02_dashboard_2026-10-31153101`), the best run so far. Only the track tests at the end are still open.

## 1. Summary in plain words

1. **Obstacle avoidance was reworked** after real test runs showed four problems on `main`:
   - the car picked the wrong side to pass on;
   - after passing the obstacle it drove too far forward before turning back;
   - it reversed far too much, sometimes all the way back to where it started;
   - it misjudged the size of flat-faced obstacles, such as a 19.9 cm wide plastic container.
2. **The car's left side is wider** than its right (the barcode reader sticks out), and the bypass now knows this.
3. **New diagnostic telemetry events** show what the ultrasonic sensor saw during each avoidance. They do not change how the car moves.
4. **The dashboard has a line-following speed box** in cm/s with 2 decimals. The robot itself refuses any line speed outside 3.00 to 15.00 cm/s, whatever sends it.
5. **Every new behaviour can be tuned or switched off at run time** with `SET:NAME=VALUE`, and `SET:OBST_LEGACY=1` brings back most of the `main` behaviour without reflashing.

`OBSTACLE_STOP_CM` (20 cm stop distance) and `PROXIMITY_STOP_CM` (8 cm E-stop) are unchanged.

## 2. Files changed

| File | What changed |
|---|---|
| `app_program/obstacle_logic.c`, `obstacle_logic.h` | Pure logic (also unit-tested on the PC): side choice, tie test, reverse budget, obstacle width floor, trail geometry |
| `app_program/tsk_obstacle.c` | The obstacle task: bypass sequence, local recovery, fine-scan extension, reverse cap, diagnostic events |
| `app_program/robot_config.h` | New defaults: obstacle settings, left half width, line-speed safety window |
| `app_program/app_params.c`, `app_params.h` | 6 new run-time parameters; the `LINE_BASE_MM_S` range is narrowed |
| `app_program/tsk_line.c` | Hard cap on the line-following base speed |
| `MQTT/dashboard.html` | Line-speed box in cm/s |
| `MQTT/robot_params.txt` | Comment only: the allowed `LINE_BASE_MM_S` range |
| `.gitignore` | Ignores `references`, `robot_firmware`, `.kiro`, `build_make/build.log`, `build_make/tools/elf2uf2.exe`, `is02_mosquitto.db` |
| `docs/obstacle_and_speed_changes_vs_main.md` | This document |

Nothing in `config/` changed. The example files `config/wifi_credentials.example.h` and `config/mqtt_config.example.h` are identical to `main`. Your own `config/wifi_credentials.h` and `config/mqtt_config.h` are gitignored and are not affected.

## 3. Background: how obstacle avoidance works (same on both branches)

The overall sequence did not change. The stages, from `tsk_obstacle.c`:

- **STAGE0, watching.** While line following, the ultrasonic sensor looks straight ahead. Two filtered readings under `OBSTACLE_STOP_CM` (20 cm) stop the car and start avoidance.
- **STAGE1, coarse sweep.** The servo pans to −60, −30, 0, 30 and 60 degrees. If everything is over 25 cm, the car goes back to line following.
- **STAGE2, fine scan.** 2 degree steps around the obstacle, ±30 degrees.
- **STAGE3, profile and side choice.** `obstacle_profile()` turns the scan into an obstacle profile (`obstacle_profile_t`):
  - `x_l_cm` / `x_r_cm`: left and right edge, sideways from the car's centre line (cm, + = right);
  - `r_min_cm`: nearest distance;
  - `clear_left_cm` / `clear_right_cm`: free space beside each edge. "No echo" counts as 200 cm.

  A side "has room" when its clearance is at least `2 × half width + margin` (`obstacle_side_room`). If neither side has room, the car reverses 100 mm and scans again (once). Before the first leg it may reverse a little (the "standoff", `obstacle_standoff_mm`) so the first 45 degree leg does not clip the obstacle's near corner.
- **STAGE4, bypass.**
  1. Pivot 45 degrees away from the obstacle.
  2. **Leg 1:** drive diagonally until the car is shifted sideways enough (`shift_mm`).
  3. Pivot back parallel to the original path.
  4. **Leg 2:** drive alongside the obstacle. The servo can only reach 60 degrees, so it follows the obstacle's side at 60 degrees ("side-follow") and keeps going until the side has ended.
  5. Drive a short extra distance (the "trail") so the rear corner is cleared.

  During every forward leg the sensor sweeps 0 / 30 / 0 / 60 degrees towards the obstacle. Two pings at the same angle inside the car's path and closer than `OBST_LEG_STOP_CM` (15 cm) mean the leg is blocked (the "leg watch").
- **STAGE5, return.** Pivot 45 degrees back towards the original path and drive diagonally until the line sensors find the line. If the line isn't seen, the car reverses onto the path and line following searches for it.
- **Blocked or failed.** On `main` a blocked leg always backed out along every leg to the scan position, then tried the other side or rescanned. Anything that fails ends in `AVOIDANCE_ABORT`.

**Terms used below:**
- **half (H):** the car's half width on the side facing the obstacle;
- **margin (M):** `OBST_MARGIN_MM`, 60 mm of extra side clearance;
- **gap (g):** the planned sideways distance from the car's centre line to the obstacle's side during leg 2, H + M (+ any extra after a retry);
- **A:** `OBST_US_AHEAD_MM`, the ultrasonic sensor's distance ahead of the wheel axle, 80 mm (a placeholder value that was never measured);
- **shift:** how far sideways the car centre must move to pass on a side.

## 4. Problems found in testing on `main`, and what fixes each

The tests used a box and a 19.9 cm wide, 11.1 cm deep plastic container, placed centred, to the left and to the right of the car's path, on a plain floor.

| Problem seen on `main` | Cause | Fixed by (section 5) |
|---|---|---|
| Passed on the wrong side, e.g. went round the long way when the obstacle was offset | "No echo" counted as 200 cm, so the side that saw nothing always "had more clearance" and won | 5.1, 5.2 |
| Drove too far forward after the obstacle before turning back | Two trails were driven one after the other: one after leg 2, then a pivot look and a second trail | 5.6, 5.7 |
| Reversed a long way, aborted, or ended far behind the stop point | Standoff and rescan reverses had no limit; with a blocked leg the car also backed out to the very start | 5.8, 5.10 |
| Misjudged flat-faced obstacles (container read 11 to 20 cm wide and seemed centred wherever it stood) | A flat face only echoes where the beam hits it square-on, so the scan saw only part of the face | 5.4, 5.5 |
| Side-follow never saw the side of a shallow obstacle, then drove the full planned depth plus a pivot look | The side was already behind the 60 degree beam when leg 2 started | 5.6 (`OBST_TRUST_MISS`) |
| Passing on the right swung wider or closer than expected | The car is wider on the left (barcode reader), but one half width was used for both sides | 5.3 |

## 5. Behaviour changes in detail

### 5.1 Side choice (`obstacle_choose_side`, `obstacle_shift_mm`)
- **`main`:** among sides with room, the one with more clearance. The smaller shift only decided when the clearances were within 10 cm (`OBST_SIDE_TIE_CM`).
- **Now:** clearance only decides whether a side has room. Among sides with room, the car takes the one needing the **smaller sideways shift**, which means passing away from the obstacle's offset. Equal shifts: left.
  - shift left = H + M − 10·`x_l_cm`
  - shift right = H + M + 10·`x_r_cm`
- Sides already blocked twice in this avoidance are excluded, as before.

### 5.2 Tie look (`obstacle_is_tie`)
When both sides are equally good and nothing has been tried yet, the car pivots 30 degrees each way with the servo at 60 to look 90 degrees sideways, then decides. This is the same as `main`.
- **`main`:** a tie meant clearances within 10 cm.
- **Now:** a tie means shifts within 10 mm (`OBST_SHIFT_TIE_MM`), which is an obstacle offset within ±0.5 cm of the car's axis.

### 5.3 Wider left side (`ROBOT_LEFT_HALF_MM`, `left_half_mm`, `side_view`, `g_half_obs`)
- `ROBOT_HALF_WIDTH_MM` (80 mm) is now the right half. `ROBOT_LEFT_HALF_MM` (90 mm, measured) is the distance from the centre line to the furthest point on the left.
- When the car passes on the right, the obstacle is on its left. The standoff, the bypass plan, both trails and the leg watch then use 90 mm instead of 80 mm.
- The side choice works on a copy of the profile (`side_view`) in which passing right needs 10 mm more shift and both sides need 10 mm more room.
- `ROBOT_LEFT_HALF_MM=0` means "same as the right side" (the `main` behaviour).

### 5.4 Obstacle width floor and edge pad (`obstacle_apply_min_width`)
- If the scanned width is under `OBST_MIN_WIDTH_MM` (150 mm), both edges are moved out to the worst case: the real left edge could be as far left as `x_r − 150 mm`, and the right edge as far right as `x_l + 150 mm`.
  - The obstacle's centre stays the same, so this does not change which side is chosen. It changes how far the car shifts.
  - It was 200 mm (the container's width) in an earlier test build. That was too cautious for unknown obstacles, so it is 150 mm now.
- Then `OBST_EDGE_PAD_MM` (20 mm) is added to each edge. A flat face stops echoing at about 25 degrees off square, so its ends are always seen a bit short.
- How far each edge was moved is remembered (`g_ext_mm`). It widens the "seen" limits in side-follow and in the pivot look, because the real side may be that much further out.

### 5.5 Fine-scan extension (`fine_scan`)
- **`main`:** the fine scan always stopped at ±30 degrees, and an obstacle that filled the window was assumed to end there.
- **Now:** if the last point at either end still hits the obstacle (within 15 cm of the nearest echo, `FINE_EXT_CLUSTER_CM`), the scan carries on in 2 degree steps up to the servo limit of ±60 degrees, so the real edge is measured.

### 5.6 Side-follow, leg 2 (`leg_side_follow`)
- **Trail start point:**
  - **`main`:** the trail was measured from the last ping that saw the side.
  - **Now:** it is measured from the **first 60 degree ping that missed** (`miss_at`). That miss already proves the rear corner has left the beam, so the old method drove up to one extra ping step forward.
  - The trail formula (`obstacle_side_trail_mm`) is A + g·tan15 − g + √2·(H + M), at least 50 mm. The header comment in `obstacle_logic.h` gives the geometric proof that this clears the corner both on the 45 degree return and during the pivot, whatever the car's length.
- **"Seen" tolerance** (`obstacle_side_seen`): a 60 degree echo counts as the side if r·sin60 ≤ gap + 150 mm. It was + 100 mm; real sides were missed at 28.8 and 29.7 cm against a 27.7 cm limit.
- **`OBST_TRUST_MISS=1` (default):** two misses in a row end the leg even if the side was never seen. Without a sighting the corner is less certain, so the trail gets g × 0.3094 extra (tan30 − tan15). This also skips the planned-depth leg (`OBST_DEPTH_MM`) and the pivot look. With `0`, a miss only counts after the side has been seen, and an unseen side drives the full planned depth.
- **"Too close" stop:** if the side is seen more than 30 mm closer than planned (`OBST_SIDE_CLOSE_MM`), the leg stops as blocked and the car sidesteps (5.8).
- The leg reports whether the side was seen and then seen to end (`p_ended`). That decides the next step (5.7).

### 5.7 Pivot look before turning back (`look_before_turn`)
- **`main`:** after leg 2 the car always pivoted 30 degrees towards the obstacle, looked square at its side, drove on in 100 mm steps while it was still there, then drove a second trail. Together with the first trail this was the "extra forward drive".
- **Now:** the look is **skipped** when side-follow saw the side and then saw it end, because the first trail already clears the corner. It still runs when the side was never seen and `OBST_TRUST_MISS=0`.
- When it runs, the second trail (`obstacle_side_trail90_mm`) uses a corrected formula: A·cos30 + (g − A·sin30)·tan15 − g + √2·(H + M), at least 50 mm. `main` used A·cos30 + 27 mm in place of the middle term, which was right for only one gap value.

### 5.8 Local recovery instead of backing out to the start (`run_bypass`, `undo_legs`)
- **`main`:** any blocked leg meant backing out along every leg to the scan position.
- **Now**, before that:
  - **Leg 2 blocked or too close:** the car reverses along leg 2 only, pivots 45 degrees out, moves further out and turns parallel again, then carries on following the side (`BYPASS_SIDESTEP` event).
    - It moves out by the shortfall + 30 mm, at least 40 mm.
    - If the block was straight ahead, it moves out by `OBST_RETRY_WIDER_MM` (100 mm).
    - At most 2 times (`OBST_SIDESTEP_MAX`).
  - **Return leg blocked** (the rear corner is further back than expected): the car reverses to where it turned, drives 100 mm further along, and turns back again (`BYPASS_EXTEND "return blocked (...)"`). At most 2 times.
  - Only after that does it back out fully. `back_out()` was replaced by `undo_legs(keep)`, which undoes legs down to `keep` (0 = everything).
- The leg log (`LEG_LOG_MAX`) grew from 8 to 16 entries for the extra legs.

### 5.9 Other side after a block
- **`main`:** after backing out, the other side was tried only if the scan had shown room there.
- **Now:** the other side is tried if it hasn't been tried yet, even without room in the scan. The block shows the obstacle reaches further towards the side tried, and the leg watch still protects the new side.

### 5.10 Reverse cap (`obstacle_reverse_budget`, `rescan_reverse`, `standoff`)
- Standoff + rescan reverses together are limited to `OBST_REVERSE_MAX_MM` (250 mm) per obstacle. This limits how far behind the stop point the car can end up.
  - A **standoff** that does not fit is refused, not shortened, because a short standoff could clip the near corner. The avoidance aborts with "reverse limit reached".
  - A **rescan reverse** is cut to what is left. Under 5 mm left (`OBST_REVERSE_MIN_MM`), it is skipped and the avoidance aborts.
  - Backing out along the legs and the reverse onto the original path are never cut, because they retrace ground already driven. They are counted in the total (`g_rev_total_mm`) shown in the `OBST_REVERSE_CAP` event.
- An earlier test build used 150 mm, which caused needless aborts.

### 5.11 Leg watch corridor (`OBST_PATH_SIDE_MM`)
The leg watch corridor is H + `OBST_PATH_SIDE_MM` on each side. It is now 25 mm (was 20). 40 mm was tried and caused false blocks. Being too near the side is now handled by the "too close" stop and sidestep (5.6, 5.8) instead.

## 6. New run-time parameters

They are set like every other parameter:
- the dashboard Parameters card (`NAME=VALUE`, then Set);
- `python is02_tool.py set NAME=VALUE`;
- or kept across restarts in `MQTT/robot_params.txt` + `py robot_params.py`.

Plain `SET` values are lost when the robot restarts. A change takes effect from the next obstacle.

| Parameter | Default | Range | Meaning | Value that matches `main` |
|---|---|---|---|---|
| `OBST_REVERSE_MAX_MM` | 250 | 0–1000 | Standoff + rescan reverse limit per obstacle (5.10) | `0` (no cap) |
| `OBST_LEGACY` | 0 | 0–1 | 1 = `main` side choice, trail and block rules (section 9) | `1` |
| `OBST_MIN_WIDTH_MM` | 150 | 0–600 | Width floor for a scanned obstacle (5.4) | `0` (off) |
| `OBST_TRUST_MISS` | 1 | 0–1 | Two misses end leg 2 even if the side was never seen (5.6) | `0` |
| `OBST_EDGE_PAD_MM` | 20 | 0–100 | Added to each measured edge (5.4) | `0` (off) |
| `ROBOT_LEFT_HALF_MM` | 90 | 0–250 | Centre line to the furthest point on the left (5.3) | `0` (= `ROBOT_HALF_WIDTH_MM`, 80) |

`GET:PARAMS` now reports 55 parameters.

## 7. Changed or new compile-time constants

| Constant | File | `main` | Now | Purpose |
|---|---|---|---|---|
| `OBST_PATH_SIDE_MM` | robot_config.h | 20 | 25 | Leg watch corridor (5.11) |
| `LINE_BASE_SAFE_MIN_MM_S` | robot_config.h | – | 30 | Line speed lower limit (section 10) |
| `LINE_BASE_SAFE_MAX_MM_S` | robot_config.h | – | 150 | Line speed upper limit (section 10) |
| `OBST_SHIFT_TIE_MM` | obstacle_logic.h | – | 10 | Tie when shifts differ by this or less (5.2) |
| `OBST_REVERSE_MIN_MM` | obstacle_logic.h | – | 5 | Smallest useful rescan reverse (5.10) |
| `OBST_CHOOSE_LEGACY` | obstacle_logic.h | – | flag `4u` | Selects the `main` side rule |
| "seen" tolerance in `obstacle_side_seen` | obstacle_logic.c | gap + 100 mm | gap + 150 mm | 5.6 |
| `LEG_LOG_MAX` | tsk_obstacle.c | 8 | 16 | Room for sidestep / retry legs |
| `OBST_SIDESTEP_MAX` | tsk_obstacle.c | – | 2 | Local recoveries per leg (5.8) |
| `OBST_SIDESTEP_MIN_MM` | tsk_obstacle.c | – | 40 | Smallest sidestep (5.8) |
| `OBST_SIDESTEP_EXTRA_MM` | tsk_obstacle.c | – | 30 | Added to the shortfall (5.8) |
| `OBST_SIDE_CLOSE_MM` | tsk_obstacle.c | – | 30 | "Too close" threshold (5.6) |
| `FINE_EXT_CLUSTER_CM` | tsk_obstacle.c | – | 15 | Fine-scan extension cluster distance (5.5) |

`robot_config.h` also gives the defaults of the new run-time parameters in section 6 (`OBST_REVERSE_MAX_MM`, `OBST_LEGACY`, `OBST_EDGE_PAD_MM`, `OBST_TRUST_MISS`, `OBST_MIN_WIDTH_MM`, `ROBOT_LEFT_HALF_MM`).

## 8. New and changed functions

**`obstacle_logic.c` / `.h`** (pure logic, no hardware):
- New:
  - `obstacle_shift_mm(prof, side, half, margin)`: shift needed for a side.
  - `obstacle_is_tie(prof, half, margin, flags)`: tie test.
  - `obstacle_reverse_budget(want, used, cap, shortenable, &allowed)`: returns `OBST_REV_FULL`, `OBST_REV_SHORTENED` or `OBST_REV_REFUSED`.
  - `obstacle_apply_min_width(prof, min_width, &ext_l, &ext_r)`: width floor.
- Changed:
  - `obstacle_choose_side`: its last parameter is now `flags` (`OBST_BLOCKED_L`, `OBST_BLOCKED_R`, `OBST_CHOOSE_LEGACY`), and the default rule is the smaller shift.
  - `obstacle_side_seen`: tolerance + 150 mm.
  - `obstacle_side_trail90_mm`: new formula.
  - `obstacle_side_trail_mm`: same formula, but it is now applied from the first miss. Its comment has the geometric proof.

**`tsk_obstacle.c`:**
- New:
  - `rev_count`: adds each reverse to the total.
  - `rescan_reverse`: rescan reverse within the cap.
  - `rev_cap_event`: posts `OBST_REVERSE_CAP`.
  - `standoff_res_t`: `STANDOFF_OK`, `STANDOFF_FAILED` or `STANDOFF_CAPPED`.
  - `left_half_mm`, `side_view`: wider left side.
  - `fine_scan_events`, `side_follow_event`, `side_look_event`: diagnostics.
- Replaced: `back_out()` became `undo_legs(keep)`.
- Changed:
  - `leg_side_follow`: new `p_ended` output, `miss_at`, trust-miss, "too close" stop, diagnostics.
  - `run_bypass`: sidestep loop around leg 2; retry loop around the return.
  - `standoff`: returns `standoff_res_t` and checks the cap.
  - `look_before_turn`: skipped after a seen-and-ended side; uses the extra edge width; posts `SIDE_LOOK`.
  - `fine_scan`: extension past ±30 degrees.
  - `avoid`: reads `OBST_LEGACY` and the cap at the start, applies the width floor and edge pad, uses `side_view` for the side choice, sets the half width facing the obstacle, tries the other side after a block.

**New module-level variables in `tsk_obstacle.c`:** `g_rev_total_mm`, `g_rev_cap`, `g_legacy`, `g_ext_mm[2]`, `g_seen_extra_mm`, `g_half_obs`, `g_view`. `g_backed_mm` now counts only standoff and rescan reverses.

## 9. Telemetry events (dashboard log)

These are new or changed. They only report what happened.

| Event | When | Detail |
|---|---|---|
| `FINE_SCAN` | After every fine scan | Echoes, 11 per event: `<first angle>deg+2: cm cm ...` (−1 = no echo) |
| `SIDE_FOLLOW` | End of leg 2 | 60 degree pings, how many saw the side, nearest echo against the "seen" limit, leg length, sideways distance from the path |
| `SIDE_LOOK` | Each pivot look | 90 and 60 degree readings, "side gone" or "side still there" |
| `OBST_REVERSE_CAP` | Standoff refused, rescan cut or skipped | Cap used (of `OBST_REVERSE_MAX_MM`) and total reversed |
| `BYPASS_SIDESTEP` | Local sidestep (5.8) | How far further out |
| `BYPASS_EXTEND` | New detail | "return blocked (N cm): back to the turn, 100 mm further" |
| `AVOIDANCE_ABORT` | New reason | "reverse limit reached" |

## 10. Line-following speed control

### Dashboard (`MQTT/dashboard.html`, Parameters card)
- A **Line speed** box in cm/s (2 decimals, e.g. 8.25), a **Set speed** button (Enter also works), and the robot's current value ("robot: 8.00 cm/s").
- Only 3.00 to 15.00 cm/s with at most 2 decimals is sent. Anything else is refused in the log and not sent.
- It sends `SET:LINE_BASE_MM_S=<cm/s × 10>`; for example, 3.33 cm/s is sent as `SET:LINE_BASE_MM_S=33.3`. 2 decimals in cm/s is 1 decimal in mm/s, so nothing is rounded away.
- The shown value updates from the robot's "ok" ack, or with "Read from robot" (`GET:PARAMS`).
- The button is disabled while the dashboard is not connected to the broker, like the other command buttons.

### Robot (the safety threshold)
- `LINE_BASE_MM_S` is accepted only from 30 to 150 mm/s (3.00 to 15.00 cm/s). On `main` it was 30 to 250.
  - This applies to every source: the speed box, a typed SET, `is02_tool.py`, and the retained `is02/config` calibration.
  - A value outside the window gets "nack: out of range" and the previous value stays.
- 150 mm/s is 60 % of the robot's hard cap (`SPEED_MAX_MM_S`, 250 mm/s, NFR18). Below 30 mm/s the wheels stall in the motor's feed-forward deadband.
- `tsk_line.c` also clamps the base speed to 150 mm/s at the point of use, whatever set the parameter.
- The build stops with `#error` if someone moves the default or the window outside these rules (window max over 60 % of the cap, or the default outside the window).

### Unchanged
- The default line speed (80 mm/s = 8.00 cm/s).
- The pivot-turn speed (`TURN_MM_S`, 90 mm/s) and the bypass speed (`BYPASS_MM_S`, 60 mm/s).

### Precision and persistence
- The setpoint is exact to 0.01 cm/s, but the real wheel speed varies by more than that.
- A SET is lost at restart. To keep a speed, set `LINE_BASE_MM_S=<mm/s>` in `MQTT/robot_params.txt` and run `py robot_params.py`.

## 11. Going back to the `main` behaviour

- **At run time, mostly:** `SET:OBST_LEGACY=1`, `SET:OBST_REVERSE_MAX_MM=0`, `SET:ROBOT_LEFT_HALF_MM=0`.
  - **`OBST_LEGACY=1` restores:**
    - the clearance-first side choice and the clearance tie;
    - the trail from the last sighting, and the pivot look plus second trail every time;
    - no width floor and no edge pad;
    - no "too close" stop, sidestep or return retry;
    - the other side tried only when the scan saw room;
    - one half width (`ROBOT_HALF_WIDTH_MM`, 80 mm) for both sides;
    - `OBST_TRUST_MISS` ignored (as if 0).
  - **It does not restore** these compile-time changes: corridor 25 mm, "seen" tolerance + 150 mm, the corrected trail90 formula, the fine-scan extension, the diagnostic events, and the line-speed window.
- **Fully, with git:**

  ```
  git checkout main -- app_program/obstacle_logic.c app_program/obstacle_logic.h app_program/tsk_obstacle.c app_program/robot_config.h app_program/app_params.c app_program/app_params.h app_program/tsk_line.c MQTT/dashboard.html MQTT/robot_params.txt
  ```

  Revert these files together. `tsk_line.c` and the dashboard use `LINE_BASE_SAFE_MAX_MM_S` from `robot_config.h`, so reverting only some of them breaks the build.

## 12. Building, flashing and testing this branch

- **Build:** the same as `main` (`build_make`, `make` with the Pico SDK 2.2.0 and the Arm GNU toolchain). The output is `build_make/is02_robot_smp0.uf2`; copy it to the Pico's RPI-RP2 drive. The build has no warnings.
- **Host tests:** the PC unit tests are not part of the repository on either branch.
  - On the developer's machine they live in `robot_firmware/tests/test_logic.c`, inside a local, gitignored copy of the project. The firmware build does not use that folder.
  - They were run against `app_program/` with `make APP=../../app_program CC=gcc`: 283 passed, 0 failed.
  - They cover the new side choice, tie, reverse budget, width floor and trail geometry (property-style tests), plus 9 checks of the line-speed window (30, 33.3 and 150 accepted; 29.9, 150.1 and 250 refused; a refused value keeps the last good one).
- **Wi-Fi and broker:** unchanged. Set your own `config/wifi_credentials.h` and `config/mqtt_config.h`, as on `main`. The phone hotspot used for MQTT must let its clients see each other; one phone (POCO X7 Pro) blocked this, another (POCO F5) worked.

### Still to do on the real track
1. Start about 40 cm from the obstacle. Most runs so far started too close, so the car stopped nearer than the 20 cm stop distance.
2. Check that the line is found again after the return leg, on a white mat with a black line. The dark test floor could not show this.
3. Try the line-speed box at 3.00, 8.00 and 15.00 cm/s while following, and check that 15.01 and 2.99 are refused.
4. Repeat left, centred and right passes with obstacles of unknown size. Watch `FINE_SCAN`, `SIDE_FOLLOW`, `SIDE_LOOK` and `OBST_REVERSE_CAP` in the log, and tune with the section 6 parameters if needed.

## 13. Merging into `main`

`origin/main` (4bec241) and this branch both added lines at the end of `.gitignore`, so a merge stops on that file. Keep both sets of lines. `main`'s new `/pico_sdk_import.cmake` ignore line also explains why that leftover file was deleted here rather than committed. No firmware source file conflicts.