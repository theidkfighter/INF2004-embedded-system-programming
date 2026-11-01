# FW-01b: Wire_Codec (all seven payload types), wire_dump, C vector harness

| | |
|---|---|
| Track | Firmware (branch `cr1/fw`) |
| Tasks | 3.9, 3.10, 3.11, 3.12, 3.13, and the **C half of 14.5** (`tests/host/tools/wire_vectors.c` + Makefile target) |
| Needs first | FW-01a done |
| Model | **Opus** (bit-exact codec, the base of the whole protocol) |
| Budget | about 1 Claude window / 150–250 Kiro credits |
| Car | no. The codec isn't called by the tasks yet |

## Read

- **`spec/design.md`:**
  - "C9. `wire.h/.c` (Wire_Codec)";
  - "D7. Wire conventions" through "D15. Parameter_IDs" (all frame layouts);
  - "Telemetry path" and "Command path" in the Architecture section;
  - Properties 19 and 28 (for the vector format).
- **`spec/requirements.md`:** Requirements 11, 12, 13 (criteria 1, 2, 5), 14 (1–3, 7), 16, and 18.6/18.7.
- **The code:**
  - `app_program/wire_ids.h`: **the contract.** Use its constants; don't redefine them;
  - `tests/host/py/wire_ctypes.py`, `test_wire_basic.py`, `stubs/wire_export.c`, left from the reverted first attempt. Reuse what matches the design, rewrite what doesn't.

## Work

1. **3.9 `app_program/wire.h` / `wire.c`, robot-to-laptop:**
   - The bit writer/reader, MSB first, with an overflow flag: `bw_init` zero-fills, plus `bw_put`, `bw_copy`, `bw_bytes`, `br_init`, `br_get`.
   - The types `wire_tlm_t`, `wire_hb_t`, `wire_ack_t`.
   - The encoders `wire_enc_telemetry` (D8, 54 bytes), `wire_enc_heartbeat` (D9, 25 bytes), `wire_enc_event` (D10 header + `bw_copy` of the argument bits), `wire_enc_ack` (D11, including GET:PARAMS parts and CONFIG), and the inverse decoders (used by the host tests).
   - Plain C only: standard headers, `bitpack.h`, `wire_ids.h`. No kernel or SDK header.
2. **3.10 laptop-to-robot:**
   - The decoders `wire_dec_command` (D12), `wire_dec_map` (D13: reject letter 26–31 when present, reject a duplicate letter), `wire_dec_config` (D14: reject a bad count or length), returning `wire_status_t`.
   - The inverse encoders, for the tests.
   - Format_Version 4 and 5 (text 'A'..'Z') always read as unknown; a zero length gives `WIRE_BAD_LENGTH`.
3. **3.11:**
   - `tests/host/tools/wire_dump.c`: prints the Format_Version, every code table and every frame layout (field, offset, width, Valid_Range, Resolution, sentinel) as JSON.
   - `tools/evidence/frame_budget.py`: reads that JSON and prints the telemetry bit sum (must be 432), the length (54) and the padding bits (0).
   - **The laptop track's table test reads this JSON, so its format is part of the contract from now on.** Describe the format in a comment at the top of `wire_dump.c`.
4. **Tests:**
   - 3.12, Property 19: every encoded frame has the header and the fixed length.
   - 3.13: `bw_put` past the end sets the overflow flag; encode-then-decode for one example of each payload type and command code; `test_resolution.py` compares each telemetry field's resolution with the Baseline_Build JSON decimals.
5. **14.5 (C half):**
   - `tests/host/tools/wire_vectors.c`, built from `wire.c`: reads `tests/host/vectors/vectors.json` (field records per payload kind and command code), encodes and decodes each one, and writes the results as JSON.
   - Add a Makefile target (`make vectors-c`) that builds and runs it when `vectors/vectors.json` exists, and skips cleanly otherwise.
   - The laptop track (LT-04) writes the generator `py/make_vectors.py` and the vectors. Agree the input format in a comment at the top of `wire_vectors.c`, matching design C16 if it says anything.
6. Re-enable `wire.c` / `wire_export.c` in the Makefile (FW-01a made them conditional, so they come back automatically) and un-skip `test_wire_basic.py`.

## Watch out

- **Flag R4 (map frame):** follow D13 as written (4 movements). Don't widen it unless R4 is accepted.
- **Signed fields** are two's complement of their width. **Sentinels** are the all-ones code unless D8–D15 say otherwise. The thresholds use the shared "valid" flag (D8).
- **No dynamic memory**; all buffers are the caller's.
- `-Wdouble-promotion`: suffix float literals.

## Gate

The firmware gate in `CONTEXT.md` section 5, plus `python3 tools/evidence/frame_budget.py <(tests/host/build/wire_dump)`, which must print 432 bits, 54 bytes and 0 padding. Use the actual path of the built `wire_dump`.

## Hand-over

- **Commit:** `CR1 FW-01b: Wire_Codec, wire_dump, C vector harness`.
- **In both PROGRESS files,** note that the `wire_dump` JSON and `wire_vectors` formats are now fixed. Merging FW-01c later completes sync point S1.
