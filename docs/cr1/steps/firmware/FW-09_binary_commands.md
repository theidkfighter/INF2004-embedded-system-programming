# FW-09: Binary commands, map and config, legacy text switch (spec Step 9, robot side)

| | |
|---|---|
| Track | Firmware (branch `cr1/fw`) |
| Tasks | 18.1, 18.2, 18.3, 18.5, 18.6, 18.7, optional 18.8, checkpoint 19 (robot part). Task 18.4 (tools default to binary) is in the laptop track, LT-05 |
| Needs first | FW-08c done, and its car check done |
| Model | **Opus** (ESTOP fast path, input validation) |
| Budget | about 1 Claude window / 150–250 Kiro credits (+50–80 for 18.8) |
| Car | **yes**: checkpoint 19, **only after sync point S5** (LT-05 merged) |
| Unblocks | merge = sync point **S4** (LT-05 can start) |

## Read

- `spec/design.md`: "Command path (laptop to robot)", "Build switches", "D12. Command_Frame", "D13. Map_Frame", "D14. Config_Frame", Properties 21 and 23 (and 13 for 18.8).
- `spec/requirements.md`: Requirements 10.4, 13 (all), 14 (1–4, 6, 7).
- `REVIEW_FLAGS.md` R2 (for 18.8), R4 (map).
- The code: `app_net.c` (`on_in_data`, `IN_MAX`, the map wrapping); `tsk_telemetry.c` (`app_net_command`, `app_net_config`, the command ring `g_cmds`, `handle_command`, `map_apply`, `g_config`); `tsk_safety.c` (`EVF_REMOTE_ESTOP`, `g_estop_rx_us`).

## Work

1. **18.1 incoming path:**
   - **Binary payloads:** no trimming and no reliance on a NUL terminator. Trim only on the legacy text path.
   - Zero-length payloads on `is02/map` / `is02/config` are ignored.
   - `IN_MAX` is 264 bytes (1024 with `APP_LEGACY_TEXT_CMD=1`).
   - The callbacks take `(p, len, ...)`.
   - **ESTOP fast path:** a valid 1-byte ESTOP frame (or, with the legacy option, text starting `ESTOP`) sets `estop_pending` and `g_estop_rx_us` in one dispatch-disabled section, sets `EVF_REMOTE_ESTOP` and wakes TSK_SAFETY_MON. Other payloads go to the 8 × 7-byte command slots.
2. **18.2 Command_Frames:**
   - Decode with `wire_dec_command` and map each status to its Reason_Code (`UNPARSED` = 15; BARCODE letter 26–31 → Reason_Code 9).
   - Perform the baseline action for each command and state. SET goes through `params_set_id`; GET:PARAMS goes out in parts.
   - With `APP_LEGACY_TEXT_CMD=1`, printable text starting 'A'..'Z' goes to the existing text parser and is answered with Ack_Frames. With 0, text reads as an unknown version.
3. **18.3:**
   - **Map:** reject the whole frame on any cause, with its Reason_Code. Otherwise replace the whole map (all present bits 0 clears it).
   - **Config:** reject the whole frame on a bad version, count or length, with applied = 0. Otherwise apply each valid pair in order and ack the applied count and the rejected IDs.
   - `g_config`: 264 bytes (1024 legacy).
4. **Tests:**
   - 18.5 Property 21 (malformed payloads leave state, Fault_Mask, parameters and map unchanged);
   - 18.6 Property 23 (whole-map replacement);
   - 18.7 `test_estop_classify.py` and `test_legacy_text.py` (built with `-DAPP_LEGACY_TEXT_CMD=1`).
5. **18.8 (optional):** pack the barcode decoder context bools and `bar[96]`. Keep it only if Property 13 still passes; otherwise revert. **Do it** if flag R2 option (a) was accepted.
6. **Checkpoint 19 (robot part):** both gates pass (default and `APP_LEGACY_TEXT_CMD=1`).

## Watch out

- **NFR21:** the ESTOP must turn the PWM off within 10 ms of arrival. Don't add work to the fast path.
- **Retained text config and map:** after flashing this build, a retained **text** `is02/config` or `is02/map` on the broker reads as an unknown version and is rejected. The robot would then run without its calibration. The car check below re-publishes both in binary.
- **Flag R4:** the map is the 4-slot D13 frame unless R4 is accepted.

## Gate

The firmware gate in `CONTEXT.md` section 5, including `tools/evidence/build_check.sh -- APP_LEGACY_TEXT_CMD=1 -j8`.

## Car check (user, only after S5)

1. Flash an `APP_PERF=0` build.
2. **Straight away**, run `py MQTT/robot_params.py --broker <ip>` and send the map from the dashboard, so the retained messages are binary. The CONFIG ack must show `applied` = the number of active lines and `rejected` 0.
3. **Check:**
   - dashboard START / STOP / PING / SET / GET:PARAMS / map;
   - remote ESTOP from the dashboard and the monitor (`pwm_off_us` ≤ 10,000);
   - BTN1 recovery;
   - one barcode turn and one obstacle bypass.

## Hand-over

- **Commit:** `CR1 FW-09: binary commands, map, config, legacy text switch`.
- **`PROGRESS_LT.md`:** add "S4 ready when merged".
