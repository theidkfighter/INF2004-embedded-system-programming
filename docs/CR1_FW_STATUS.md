# CR1 firmware track: status against the spec (branch `Wei-En`)

> **Superseded (2 Nov 2026):** `docs/cr1/PROGRESS_FW.md` is now the record of the firmware track, on branch `cr1/fw-rest`. This file is kept for history and is no longer updated.

Base: `Poh-Loon` at `542368f`. Contract: `docs/cr1/spec/design.md` D7 to D15 and
`app_program/wire_ids.h` (unchanged, Format_Version 1). This file sits outside
`docs/cr1/` so it does not clash with the team's copy of the guides; copy the
relevant lines into `docs/cr1/PROGRESS_FW.md` once that folder is committed.

## Sync point S1 (needed by laptop step LT-04)

| S1 item | Status | Where |
|---|---|---|
| `app_program/wire.c` encodes and decodes all 7 payload types | done | telemetry, heartbeat, event, ack, command, map, config; inverse encoders `wire_enc_cmd`, `wire_enc_map`, `wire_enc_config` for the tools |
| `tests/host/tools/wire_dump.c` | done | `make -C tests/host tools && tests/host/build/wire_dump` (JSON: version, events, reasons, commands, stop and abort reasons, 55 parameters with store width, min and max, table CRC, frame layouts) |
| `tests/host/tools/wire_vectors.c` | done | line format documented at the top of the file; `make -C tests/host vectors-c` runs `tests/host/vectors/vectors.txt` when LT-04 has written it |
| `app_program/evt.c` event builders | done (generic builder) | `evt_begin` / `evt_u` / `evt_s` / `evt_cm9` / `evt_cm13` / `evt_post`; `evt_post` refuses a wrong bit count per Event_Code |
| Host test build repaired, `run_all.sh` passes | done | from a clean build: `rm -rf tests/host/build && bash tests/host/run_all.sh --seed 12345` → PASSED (needs `npm ci` in `tests/host/js` once); `post_evt` defined once (os_stub.c); stray `p3_full.log`, `p3_ps.txt` removed |

Conformance checks: `tests/host/py/test_spec_conformance.py` reads the
telemetry and heartbeat bits at the D8 / D9 offsets, checks the D13 map slot
order and the D15 Parameter_Store positions, and compares all 55 parameter
defaults with the Baseline_Build library. `test_wire_tools.py` checks the
`wire_dump` tables against the design tables and the layouts against D8 / D9.
The firmware tests use only the C side (`tests/host/py/wire_spec.py` reads
`wire_dump`); they do not depend on any laptop codec.

**`MQTT/` is the laptop track's.** This branch leaves it exactly as on
`542368f` (an earlier Python codec `MQTT/is02_wire.py` and its test were
removed); the laptop codecs come from `cr1/laptop`.

## Spec tasks done on this branch

| Step | Tasks | Notes |
|---|---|---|
| 1 | 3.9, 3.10, 3.11 (+ `tools/evidence/frame_budget.py`), 3.12 (P19), 3.13 (partly), 3.14, 3.15 (P10), 3.16, 3.7 (P9) | 3.1 to 3.6 and 3.8 are Poh Loon's |
| 2 | 5.1, 5.2 | 16-bit Fault_Mask in `fault_mask.h`; `tsk_imu.c` still keeps its local `b_dead` mirror (same transitions) |
| 3 | 6.1 to 6.3 | Status_Snapshot 60 B; unit layout differs from the D2 table (internal only) |
| 4 | 8.1, 8.3, 8.4 | stop reason, odometry record, packed mailboxes |
| 5 | 10.1 (barcode part), 10.6 (barcode differential test), 10.8, 10.9 | packed Parameter_Store (D15) |
| 6 | 12.1, 12.2, 12.3 (most) | edge ring, raw sample ring, echo word, ADC / thresholds / VBAT halfwords |
| 8 | 16.1, 16.2, 16.5, 16.6, 16.8, 16.9 | binary events, acks, heartbeat, telemetry |
| 9 | 18.1, 18.2, 18.3, 18.8 (barcode context flags) | binary commands, ESTOP fast path, map and config frames |

## Not done yet

- Step 2–3 tests: 5.3, 5.4, 6.4 to 6.8 as listed (some unit tests exist in `test_status_snapshot.py`).
- Step 4: 8.2 (buzzer, heartbeats, re-arm counter, command ids, E-stop flags), motion done word, 8.5 to 8.8.
- Step 5: 10.2, 10.3 (IMU, obstacle and motion contexts), 10.4, 10.5, 10.7, 10.10, 10.11, 10.12 (barcode map `g_map_w[3]`).
- Step 6: 12.4 to 12.6.
- Step 8: 16.3, 16.7, 16.10, 16.11.
- Step 9: 18.5 to 18.7; `APP_LEGACY_TEXT_CMD=1` (robot accepts text commands) is not implemented. **With this firmware the robot accepts binary only**, so the laptop tools must send binary (LT-05 mode) to command it.
- Step 10: 20.1 to 20.7.
- Every car checkpoint (FW-00, cp 7, 9, 13, 17, 19).

## Known differences from the spec text (internal, not on the wire)

- `msg_codec`: `obst_msg_t` is 24 bytes (spec 28); encoders do not return a clamp count; `msg_nav_pack` has no bool result.
- MC_LINE wheel speeds are S14 @0.1 mm/s (spec S13): `line_wheels()` can ask up to 2 × base speed before the clamp. To raise as a review flag.
- Event builders are one generic builder plus per-call-site field lists, not one function per Event_Code.
- Status_Snapshot units are grouped differently from the D2 table; the Telemetry_Frame converts the barcode to the D8 Code 39 index.
