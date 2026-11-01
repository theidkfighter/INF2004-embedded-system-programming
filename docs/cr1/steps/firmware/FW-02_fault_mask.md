# FW-02: Fault_Mask (spec Step 2)

| | |
|---|---|
| Track | Firmware (branch `cr1/fw`) |
| Tasks | 5.1, 5.2, 5.3, 5.4 |
| Needs first | FW-01c done. **Flag R1 decided** (accepted or rejected) in `REVIEW_FLAGS.md`. If it's still open, ask the user before starting |
| Model | Sonnet |
| Budget | about ½ Claude window / 50–90 Kiro credits |
| Car | the bench check is done together with FW-03 (checkpoint 7) |

## Read

- `spec/design.md`: "C3. `fault_mask.h`", "D1. Fault_Mask", "Critical-section strategy", Property 7.
- `spec/requirements.md`: Requirement 3 (all criteria).
- `REVIEW_FLAGS.md` R1, and its decision.
- The code: `app_types.h` (`FAULT_*`), `os_glue.c` (`fault_set`/`fault_clear`), `tsk_safety.c` (search `FAULT_CONFIG`, `try_recover`), `tsk_imu.c` (`b_dead`), `app_net.c` (`g_wifi_up`, `g_mqtt_up`, `manage_wifi`, the MQTT callbacks), `tsk_obstacle.c` (`ping()`), `tsk_motion.c` (encoder-loss check, about line 468).

## Work: if R1 is rejected or still open, follow the spec exactly

1. **5.1:**
   - Create `app_program/fault_mask.h` with positions 0–12 unchanged, `FAULT_WIFI_LOSS` 13, `FAULT_MQTT_LOSS` 14, `FAULT_US_NO_ECHO` 15, `FAULT_BASE_MASK` (0x1FFF), `FAULT_BOOT_VALUE` (Wi-Fi loss and MQTT loss set).
   - The status `faults` member becomes `volatile uint16_t fault_mask`, initialised to `FAULT_BOOT_VALUE`.
   - `fault_set` / `fault_clear` work inside `crit_dsp_enter/exit`; `fault_mask()` is one 16-bit load.
2. **5.2:**
   - Safety predicate: `(fault_mask() & FAULT_BASE_MASK) == FAULT_CONFIG`.
   - `try_recover()` clears `FAULT_BASE_MASK & ~FAULT_IMU_DEAD`.
   - Remove `b_dead`, `g_wifi_up` and `g_mqtt_up`. `net_wifi_up()` / `net_mqtt_up()` return the inverse of the loss bits.
   - The no-echo bit is set or cleared in `ping()` before the reading is used.
   - The console prints only the 13 baseline names; the JSON `faults` prints the 16-bit mask.
   - **Leave the encoder-loss check unchanged.**
3. **5.3 Property 7** (bit-set model, every update inside one dispatch-disabled section, checked with `crit_stub.c`) and **5.4 unit tests:**
   - the boot value;
   - the recovery mask keeps the link and no-echo bits;
   - the no-echo rule for −1, 0 and a positive reading.

## Work: if R1 is accepted, use the flag's layout instead

- Bit 4 `FAULT_ULTRASONIC` = no echo; bit 12 `FAULT_TELEMETRY` = MQTT session loss; bit 13 = Wi-Fi loss; bit 14 = `FAULT_MOTOR_STALL`; bit 15 free.
- `FAULT_BASE_MASK` must still cover only the conditions that existed in the Baseline_Build *as set conditions*. Write the exact predicate in the session log, and keep the safety behaviour identical (Requirement 3.9).
- Motor stall: both wheels over `ENC_LOSS_DUTY` with no ticks for `ENC_LOSS_MS` → `FAULT_MOTOR_STALL`; otherwise `FAULT_ENCODER_LOSS`. Both still request ERROR at the same place.
- **This changes the contract.** Increment `WIRE_FORMAT_VERSION` only if a frame layout changed. Fault *names* aren't layout, but the laptop codecs must learn the new names, so add a line to both PROGRESS files.

## Watch out

- `app_net.c` runs in the radio-owner task, a kernel task, so `crit_dsp_*` is allowed there. Check the include layering: `app_net.c` is on the SDK side, so call the plain `fault_set()` API, not kernel functions.
- Behaviour must not change (Requirement 3.9): only telemetry shows the new bits.

## Gate

The firmware gate in `CONTEXT.md` section 5.

## Hand-over

- **Commit:** `CR1 FW-02: Fault_Mask`.
- **Car:** no separate check. It's covered by FW-03's bench check (checkpoint 7). Note in "Car checks waiting".
