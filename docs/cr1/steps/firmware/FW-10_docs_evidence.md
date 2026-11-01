# FW-10: Documentation, evidence tooling and cleanup (spec Step 10)

| | |
|---|---|
| Track | Firmware (branch `cr1/fw`) |
| Tasks | 20.1, 20.2, 20.3, 20.4, 20.5, 20.6, 20.7. Task 20.8 (tables vs the Python/JS codecs) is in INT-01 |
| Needs first | FW-09 done |
| Model | Auto or Sonnet (documents generated from code) |
| Budget | about 1 Claude window / 80–150 Kiro credits |
| Car | the user then fills in the after measurements (Requirements 20, 22–24) |

## Read

- `spec/design.md`: "Documentation and Evidence Deliverables", "C16", "C17", "D16", "D17".
- `spec/requirements.md`: Requirements 16.2–16.5, 21 (all), 22, 23, 24.
- `REVIEW_FLAGS.md`: **every item and its decision.** The data table and `correctness.md` must state each accepted change and each known difference (for example R4, if recorded as an allowed difference).

## Work

1. **20.1:** `tests/host/tools/layout_dump.c` prints every Storage_Unit, field offset and width, packed type size and protection as JSON.
2. **20.2 `docs/data_representation.md`:**
   - One section per category, one row per Application_Field, with the fields Requirement 21.2 lists. Include the writers and protection per shared unit, the Private_Control_Variables with their reasons, and all 55 parameters.
   - Follow each table with a fenced `json` block for the doc test.
   - Include the "not packed, and why" rows: the barcode width arrays (D16), the 32-bit encoder counters, `g_raw_er` / `g_raw_el`, and float set-points where kept.
3. **20.3 `docs/protocol.md`:**
   - The Format_Version and its change log.
   - D7 conventions and the bit tables for all seven payload types and every command code.
   - The Event_Code catalogue with argument layouts and detail-text rules.
   - The Reason / command / stop / abort codes with their baseline texts.
   - The 55 Parameter_IDs.
   - The CRC definition with a worked example, and the layout fingerprint.
4. **20.4:** add a "Data representation (Change Requirement 1)" link item to `README.md`; finish `tests/host/README.md`.
5. **20.5:** `run_all.sh --report` writes the per-property summary into `docs/evidence/correctness.md`. Fill in the frame-budget section of `message_sizes.md` from `frame_budget.py`.
6. **20.6:** remove the `jfmt` JSON helpers **only** if nothing references them.
7. **20.7 `test_layout_doc.py`:** `layout_dump` equals the JSON blocks; Required_Bits = ceil(log2 N) for every row; every frame field has a row; the README links resolve.

## Watch out

- **RAM evidence and `g_mpf_buf`** (flag R3): if R3 wasn't applied, `ram_usage.md` must say that `g_mpf_buf` is unused and that the kernel pool lives outside `.bss`, with its computed size before and after.
- **The tables must match the code at the submitted commit** (Requirement 21.7). Generate them; don't type them.

## Gate

The firmware gate in `CONTEXT.md` section 5, with `test_layout_doc.py` included in it.

## Hand-over

- **Commit:** `CR1 FW-10: data representation table, protocol spec, evidence tooling`.
- **Tell the user which tables are left for them**, with the script that fills each (`docs/evidence/README.md`):
  - RAM on both builds;
  - MQTT sizes over a track run;
  - timing;
  - 3 track runs;
  - L1–L6;
  - the functional checklist.
