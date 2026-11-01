# LT-05: Tools publish binary by default (task 18.4)

| | |
|---|---|
| Track | Laptop (branch `cr1/laptop`) |
| Tasks | 18.4, checkpoint 19 (laptop part) |
| Needs first | **Sync point S4** (FW-09 merged: the robot accepts binary commands) |
| Model | Auto |
| Budget | about ½ Claude window / 30–60 Kiro credits |
| Car | **yes**: checkpoint 19, together with FW-09's car check |
| Unblocks | merge = sync point **S5** (FW-09 builds may be flashed for the checkpoint; INT-01) |

## Read

- `PROGRESS_LT.md`, `PROGRESS_FW.md` (to confirm S4).
- `spec/design.md`: "C15", "Build switches" (`APP_LEGACY_TEXT_CMD`).
- `spec/requirements.md`: 17.5.
- `REVIEW_FLAGS.md` R7.

## Work

1. **18.4:** switch the default of `--legacy-json` to **off** in all five tools, and default the dashboard checkbox to off.
   - Legacy mode stays available for Baseline_Build comparison runs. In legacy mode the tools still send text and read JSON (R7 rule from LT-03).
   - With legacy off, commands, map and config go out binary; received payloads are still auto-detected.
2. **Update `MQTT/README.md`:**
   - the new default;
   - when to use `--legacy-json` (only against a Baseline_Build robot);
   - **the retained-message step** after flashing an FW-09 build: run `robot_params.py` and send the map once, so the retained `is02/config` and `is02/map` are binary.
3. **Update `test_tools.py`** for the new default. Add a test that a binary ESTOP is a 1-byte frame (Requirement 13.3).

## Gate

The laptop gate in `CONTEXT.md` section 5.

## Check (user, checkpoint 19, with FW-09's car check)

Follow the "Car check" in `steps/firmware/FW-09_binary_commands.md`, with these tools at their new defaults. Also run `is02_tool.py cmd PING:1`, `is02_tool.py set LINE_STEER=0.35`, `robot_map.py`, and a monitor ESTOP (`e` key).

## Hand-over

- **Commit:** `CR1 LT-05: tools send binary by default`.
- **Merging = sync point S5.** Add a line to `PROGRESS_FW.md` "Car checks waiting": "S5 met: FW-09 builds may be flashed".
