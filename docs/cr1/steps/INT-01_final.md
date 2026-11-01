# INT-01: Final cross-check and final checkpoint

| | |
|---|---|
| Track | Integration (on `Poh-Loon` after both tracks are merged) |
| Tasks | 20.8, final checkpoint 21 |
| Needs first | **Sync point S6:** every FW and LT guide done and merged, and FW-10 done |
| Model | Sonnet |
| Budget | about ½ Claude window / 40–80 Kiro credits |
| Car | the user's after measurements (`docs/evidence/`) |

## Read

- `PROGRESS_FW.md`, `PROGRESS_LT.md`, `REVIEW_FLAGS.md` (all decisions).
- `spec/design.md`: "Documentation and Evidence Deliverables", "Risks and Open Points".
- `spec/requirements.md`: 2.10, 8.2, 12.4, 13.2, 16.2–16.6, 21, 24.

## Work

1. **20.8:**
   - `tests/host/py/test_tables.py` compares `wire_dump` with the Python and JS tables and with `docs/protocol.md`:
     - unique codes per category;
     - 45 events, 11 commands, 55 Parameter_IDs;
     - the reason and stop texts equal the baseline strings;
     - the frame lengths.

     A differing version fails, naming the codec.
   - `tests/host/py/test_fingerprint.py`: the `wire_dump` layout hash matches the fingerprint stored for the current Format_Version.
2. **Final checkpoint 21:**
   - On `Poh-Loon`: `tools/evidence/build_check.sh -- -j8` PASS, the same with `APP_LEGACY_TEXT_CMD=1`, and `bash tests/host/run_all.sh --report` PASS with the summary written into `docs/evidence/correctness.md`.
   - Check that `docs/data_representation.md` and `docs/protocol.md` match the code (`test_layout_doc.py`, `test_tables.py`).
   - List every `REVIEW_FLAGS.md` item with its final decision in `docs/evidence/correctness.md` ("Known differences and decisions").
3. **The user's measurements, Week 13:** list what's still empty in `docs/evidence/*.md` and the command that fills each:
   - `ram_report.sh` on both builds;
   - `type_sizes.sh`;
   - `msg_sizes.py` on a track-run log;
   - `perf_report.py` on an `APP_PERF=1` console log;
   - 3 track runs per build;
   - L1–L6;
   - the functional checklist.

## Gate

Everything in step 2 passes on `Poh-Loon`.

## Hand-over

- **Commit:** `CR1 INT-01: table and fingerprint tests, final checkpoint`.
- **Both PROGRESS files:** "CR1 code complete; evidence measurements pending/complete".
