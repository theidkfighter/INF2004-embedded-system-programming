# FW-08a: Binary events (spec Step 8a)

| | |
|---|---|
| Track | Firmware (branch `cr1/fw`) |
| Tasks | 16.1, 16.2, 16.3. Task 16.4 (Property 16) belongs to the laptop track, LT-04 |
| Needs first | FW-06 done |
| Model | Sonnet (Opus if the event cross-check gets hard) |
| Budget | about 1 Claude window / 120–200 Kiro credits |
| Car | no separate check (FW-08c's checkpoint 17 covers all of Step 8). **Don't flash for the team until sync point S3** (laptop tools merged) |

## Read

- `spec/design.md`: "Telemetry path (robot to laptop)", "C8. `evt.h/.c`", "C10. `app_net.h/.c`", "D6" (event body, Message_Pool), "D10. Event_Frame", Property 17.
- `spec/requirements.md`: Requirements 8.2–8.5, 8.7, 8.9, 12.2, 12.3, 12.8.
- `INTEGRATION.md` sync point S3.
- The code: every `post_event(` and `event2(` call (`grep -n "post_event(\|event2(" app_program/*.c`); `os_glue.c` (`post_event`, the pool); `tsk_telemetry.c` (`publish_event`); `app_net.c` (`net_post`).

## Work

1. **16.1:**
   - Implement `post_evt` in `os_glue.c`: pool + `g_mbx_tlm`, `TMO_POL`.
   - Replace `tlm_event_t` with `evt_msg_t` in `app_msg_body_t`. Set `MSG_BLK_WORDS` to 8 (32 bytes), with static asserts.
   - Remove `post_event`, `TLM_EVENT_NAME_LEN` and `TLM_EVENT_DETAIL_LEN`.
   - Add the byte-length `net_post(topic, p, len)`. TSK_TELEMETRY encodes each `evt_msg_t` with `wire_enc_event` and publishes it on `is02/event`.
2. **16.2:**
   - Replace **every** `post_event` / `event2` call with its D10 builder, called with the typed values the text was built from.
   - Remove text buffers that were used only for events. **Keep `log_line()` text unchanged.**
   - `FINE_SCAN`: at most 11 readings per event. `BC_DBG` widths: at most 6 parts of 16.
   - **Completeness check:** after 16.1 removes `post_event`, any call left over fails to compile.
3. **16.3 Property 17:** variable-length details split without loss.

## Watch out

- **Every `abort_avoidance(...)` reason and `g_why` text maps to the 11 abort codes** (done in FW-05a). Check that each event's values match today's detail text at the same decimals. The laptop track's Property 16 (LT-04) will compare the rebuilt texts against golden texts, so record any doubtful mapping in the session log.
- **After this commit the robot publishes binary events.** Until LT-03 is merged (S3), the dashboard can't show them. That's expected; don't flash this for normal use.
- **Flag R8:** obstacle events keep their meaning; no new event codes.

## Gate

The firmware gate in `CONTEXT.md` section 5. Also give `grep -c "post_event(" app_program/*.c` (must be 0) in the session log.

## Hand-over

**Commit:** `CR1 FW-08a: binary Event_Frames`.
