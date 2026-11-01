# FW-01c: Packed mailbox bodies and typed event builders (end of spec Step 1)

| | |
|---|---|
| Track | Firmware (branch `cr1/fw`) |
| Tasks | 3.14, 3.15, 3.16, checkpoint 4 |
| Needs first | FW-01b done |
| Model | **Opus** for 3.16 (45 event builders must match the baseline texts); Sonnet is fine for 3.14/3.15 |
| Budget | about 1 Claude window / 100–180 Kiro credits |
| Car | no. Nothing is called by the tasks yet |
| Unblocks | merge into `Poh-Loon` = sync point **S1**: the laptop track's LT-04 can start |

## Read

- `spec/design.md`: "C7. `msg_codec.h/.c`", "C8. `evt.h/.c`", "D6. Inter_Task_Messages and the Message_Pool", "D10. Event_Frame" (event catalogue, argument layouts, detail-text rules), Properties 10 and 17.
- `spec/requirements.md`: Requirement 4 (1–4, 9), 6 (4–6), 8 (1–3, 6, 8, 9).
- `app_program/wire_ids.h` (event and abort codes), and today's event texts: `grep -n 'post_event(\|event2(' app_program/*.c`.

## Work

1. **3.14 `app_program/msg_codec.h/.c`:**
   - `motion_msg_t` (12 bytes), `nav_msg_t` (1 byte), `obst_msg_t` (28 bytes), `evt_msg_t` (28 bytes), with their encode/decode functions.
   - Encoders clamp to the nearest bound and return the clamp count.
   - `msg_nav_encode` returns false for a bad type or letter.
   - Unused bits are 0.
   - `_Static_assert` on each body size.
2. **3.15 Property 10:** inter-task round trip for every motion command type, nav events, obstacle profiles (including the no-echo codes) and `evt_msg_t` bodies. Add `msg_codec.c` to `LIB_SRCS`.
3. **3.16 `app_program/evt.h/.c`:**
   - `evt_begin`, `evt_u`, `evt_q`, `evt0`, and **one builder per Event_Code** (D10). That includes `evt_state`, `evt_stop`, `evt_hump`, `evt_fine_scan` (posts `ceil(n/11)` events) and `evt_bc_widths` (at most 6 parts of 16).
   - `post_evt` is a prototype only. Its implementation comes in FW-08a; on the host, `os_stub.c` captures it.
   - Each builder's arguments must carry exactly the values today's detail text prints, at the decimals printed (Requirement 8.3). Cross-check against today's `post_event` / `event2` calls, one by one.
4. **Checkpoint 4:** firmware gate passes; new code unused by the tasks; `arm-none-eabi-size` shows only `text` growth (data and bss unchanged from `542368f`, or explain why).

## Watch out

- **Flag R8:** don't change any obstacle event text or behaviour.
- **Events added by the obstacle refinements** don't exist yet. Don't invent codes. New codes need a contract change (`INTEGRATION.md` 1.3).
- **`FINE_SCAN` and `BC_DBG` splitting must match the baseline limits** (11 points; 6 parts per window).

## Gate

The firmware gate in `CONTEXT.md` section 5.

## Hand-over

- **Commit:** `CR1 FW-01c: msg_codec, event builders (Step 1 complete)`.
- **`PROGRESS_FW.md`:** checkpoint 4 done.
- **`PROGRESS_LT.md`:** add a line "S1 ready when this commit is merged into Poh-Loon". You may write to the LT ledger for this one sync note only.
