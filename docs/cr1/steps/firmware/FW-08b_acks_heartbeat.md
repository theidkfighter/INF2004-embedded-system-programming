# FW-08b: Binary acks and heartbeat (spec Step 8b)

| | |
|---|---|
| Track | Firmware (branch `cr1/fw`) |
| Tasks | 16.5, 16.6, 16.7 |
| Needs first | FW-08a done, and FW-06's car check done |
| Model | Sonnet |
| Budget | ½–1 Claude window / 80–150 Kiro credits |
| Car | covered by FW-08c's checkpoint 17 |

## Read

- `spec/design.md`: "D9. Heartbeat_Frame", "D11. Ack_Frame" (Reason_Codes, GET:PARAMS parts, CONFIG, ESTOP extra), "C10", Property 20.
- `spec/requirements.md`: Requirements 12.1, 12.4–12.7, 13.8, 14.5, 19.10.
- The code: `tsk_telemetry.c` (`ack()`, `map_ack()`, `estop_ack()`, the GET:PARAMS loop, the CONFIG ack, `heartbeat()`); `app_net.c` (accessors).

## Work

1. **16.5:**
   - Build every ack with `wire_enc_ack` and its Reason_Code (D11).
   - That covers the E-stop ack with `g_estop_pwm_off_us`, GET:PARAMS in numbered parts of 12 IDs using `params_put_wire`, and CONFIG with the applied count and rejected IDs.
   - **Commands still arrive as text in this step.**
2. **16.6:**
   - Fill `wire_hb_t`: sequence M14, time M24, state, RSSI, ring fill, uptime U17 (saturating), reconnects, dropped, `sat_count()`, Fault_Mask, parameter count and CRC, broker IPv4.
   - Publish it on `is02/status`.
   - Add `net_broker_ipv4()`, `net_reconnects()` and `net_frames_dropped()` (16-bit saturating).
3. **16.7 Property 20:** GET:PARAMS parts cover every parameter exactly once.

## Watch out

- **The ESTOP fast path and its PWM-off timing must not change.** Only the ack payload format changes.
- **Every baseline ack result text needs a Reason_Code** (Requirement 12.4). Compare with today's `ack(` calls one by one.
- **Each GET:PARAMS part must fit one `app_net.c` message slot** (64 bytes after FW-08c; check against the current slot size too).

## Gate

The firmware gate in `CONTEXT.md` section 5.

## Hand-over

**Commit:** `CR1 FW-08b: binary Ack_Frames and Heartbeat_Frame`.
