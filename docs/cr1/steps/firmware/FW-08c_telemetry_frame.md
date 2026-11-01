# FW-08c: Binary telemetry frame and byte payload rings (spec Step 8c)

| | |
|---|---|
| Track | Firmware (branch `cr1/fw`) |
| Tasks | 16.8, 16.9, 16.10, 16.11, checkpoint 17 |
| Needs first | FW-08b done. **Sync point S3 (LT-03 merged) before flashing for the car check** |
| Model | **Opus** (`app_net.c` ring rework, offline buffering) |
| Budget | about 1 Claude window / 100–180 Kiro credits |
| Car | **yes**: checkpoint 17 (dashboard and monitor against the binary robot-to-laptop payloads) |

## Read

- `spec/design.md`: "C10. `app_net.h/.c`", "D8. Telemetry_Frame", "D17" (static RAM budget), Properties 6 and 12.
- `spec/requirements.md`: Requirements 11 (all), 19.4, 19.8, 19.9, 20.4, 20.5.
- The code: `app_net.c` (`g_frames`, `g_msgs`, `net_post_frame`, the replay in the poll loop, `g_tx_frame`); `tsk_telemetry.c` (`frame()`, `frame_build`, `tlm_bench` under `APP_PERF`).

## Work

1. **16.8 rings (`app_net.h/.c`):**
   - `net_post_frame(p, len)` with `len == WIRE_TLM_LEN`.
   - 64 slots of exactly 54 bytes. When full, overwrite the oldest and increment the saturating `g_dropped`. The index update and copy run with interrupts masked.
   - 8 message slots of 64 bytes, each with a 16-bit `hdr` (topic, len).
   - Replay oldest first, at up to 4 frames per poll.
   - **The discovery beacon and the back-off stay unchanged.**
2. **16.9 telemetry:**
   - Fill `wire_tlm_t` from the `st_read()` codes, plus sequence (M15), time (M24), ADC range, RSSI and ring fill. Encode it with `wire_enc_telemetry` and post with `net_post_frame`.
   - Keep the 500 ms period, skipping missed periods.
   - Move the `PERF_TLM_ENCODE` probe and `tlm_bench` to the binary encoder.
   - Remove the JSON builders for telemetry, status, event and ack. Keep `jfmt` for console lines and the legacy text parser.
3. **Tests:**
   - 16.10: add the overwrite-oldest rule, the exact +1 dropped count, and byte-identical replay to Property 12.
   - 16.11: add the network counters and the heartbeat uptime to Property 6.
4. **Checkpoint 17:** gate. Write the car check below into "Car checks waiting".

## Watch out

- **Requirement 20.4:** the binary encode's mean and max time must be ≤ the JSON build's (FW-00 `tlm_bench`). Keep `wire_enc_telemetry` free of float maths: the snapshot already holds codes.
- **Requirement 23.4:** the frame must be ≤ 25% of the mean baseline JSON size. 54 bytes is about 14% of about 385.
- **The JSON fallback for comparison runs is the Baseline_Build itself.** No JSON telemetry stays in this build (design D2 decision).

## Gate

The firmware gate in `CONTEXT.md` section 5. The session log must show the size line: bss should drop by about 35 KB (`g_frames` 32,896 → 3,456 bytes; `g_msgs` 6,432 → 528 bytes).

## Car check (user, only after S3)

- Flash an `APP_PERF=0` build. Run the dashboard and `telemetry_monitor.py` (with `--legacy-json`, from LT-03).
- **Check:**
  - every panel updates;
  - monitor L1–L4 PASS;
  - events show the same texts as before;
  - an offline period (hotspot off for 20 s) replays the buffered frames;
  - E-stop acks show `pwm_off_us`.
- Commands are still text in this step, so the tools' legacy text command path is in use (flag R7).

## Hand-over

**Commit:** `CR1 FW-08c: binary Telemetry_Frame, byte payload rings (Step 8 complete)`.
