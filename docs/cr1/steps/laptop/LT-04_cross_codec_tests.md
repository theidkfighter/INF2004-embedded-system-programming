# LT-04: Shared vectors and cross-codec property tests

| | |
|---|---|
| Track | Laptop (branch `cr1/laptop`) |
| Tasks | 14.5 (Python half), 14.10, 14.11, 14.12, 14.13, 14.14, 14.15, 14.16, 14.17, 16.4 |
| Needs first | LT-03 done, and **sync point S1** (FW-01c merged: `wire.c`, `wire_dump`, `wire_vectors`, `evt.c`). Task 14.10 also needs **S2** (FW-05b merged); if S2 isn't met yet, leave 14.10 `waiting` and do the rest |
| Model | Sonnet |
| Budget | about 1 Claude window / 120–220 Kiro credits |
| Car | no |

## Read

- `PROGRESS_LT.md` and `PROGRESS_FW.md` (to confirm S1/S2, and the `wire_dump` / `wire_vectors` formats written by FW-01b).
- `spec/design.md`: "C16. Host_Test_Suite", "Property-based testing configuration", Properties 16 and 24–30.
- `spec/requirements.md`: Requirements 18 (all), 16.6.
- `REVIEW_FLAGS.md` R6 (optional test duplicates).
- The C side, read only: `tests/host/tools/wire_dump.c`, `wire_vectors.c`, `tests/host/py/conftest.py` (the `ctypes` loader), `app_program/evt.c`.

## Work

1. **14.5 (Python half):** `tests/host/py/make_vectors.py` generates `tests/host/vectors/vectors.json` from a recorded seed: field records per payload kind and command code, in the input format of `wire_vectors.c`. Commit the generated file.
2. **The table check.** This isn't a separate task, but it's the drift guard from `INTEGRATION.md` 1.4: `tests/host/py/test_codec_tables.py` compares the Python and JS tables (codes, texts, frame lengths, Format_Version) with the `wire_dump` JSON. A difference fails, naming the codec.
3. **The properties** (tag each test `# Feature: minimum-bit-data-representation, Property <n>: <title>`):
   - **14.10, P24:** the parameter-table version detects edits. C (`params_table_crc` via ctypes), Python and JS agree. **Needs S2.**
   - **14.11, P25:** robot-to-laptop round trips, C encode → Python and JS decode. At least 1,000 telemetry, heartbeat and ack sets, and at least 100 per Event_Code.
   - **14.12, P26:** laptop-to-robot, Python and JS encode → C decode. At least 1,000 per command code, map and config.
   - **14.13, P27:** decode-then-encode is the identity on bytes, for C, Python and JS (flag R6: the JS part may become optional if the team accepts R6).
   - **14.14, P28:** the three codecs agree on the shared vectors.
   - **14.15, P29:** tool inputs parse to Parameter_IDs or report the bad line.
   - **14.16, P30:** unknown codes are shown numerically.
   - **16.4, P16:** event detail text can be rebuilt. The C event builders from `evt.c` (via ctypes, with `post_evt` captured by `os_stub.c`) → `wire_enc_event` → the Python and JS formatters, compared with a reference formatter using `jb_fixed` rounding and with **hand-written golden texts** in `tests/host/vectors/event_golden.json`. Write the golden texts from the baseline `post_event` / `event2` calls (`git show bca7964:app_program/<file>`), at least one per event and variant.
4. **14.17:** `test_presentation.py` and `presentation.test.js`:
   - decoded telemetry has every baseline key;
   - the sentinel presentation;
   - bad-version, short, long and zero-length payloads are rejected without fields.

## Watch out

- **Use the C library only through `conftest.py`'s loader.** If you need a new exported symbol or a Makefile target, write the request in `PROGRESS_FW.md` (the session log, under "requests from LT") rather than editing FW files.
- **Seeds:** read `IS02_SEED` in JS and use pytest's `--hypothesis-seed`, so `run_all.sh --seed` reproduces a run.
- **Summary lines:** each property must print `P<n> <title> | <payload> | cases=<n> | failed=<n> | seed=<s>`, the format `run_all.sh` uses.

## Gate

The laptop gate in `CONTEXT.md` section 5, run on a branch that includes `Poh-Loon` with S1 merged. Run `make vectors-c` (C side) too.

## Hand-over

- **Commit:** `CR1 LT-04: shared vectors and cross-codec properties`.
- **If 14.10 waited for S2,** a short follow-up session does only 14.10. Mark it in `PROGRESS_LT.md`.
