# FW-01a: Repair the host test build, verify bitpack and critical sections

| | |
|---|---|
| Track | Firmware (branch `cr1/fw`) |
| Tasks | repair (below), then verify or finish 3.1, 3.2, 3.3, 3.4, 3.5, and do 3.7 |
| Needs first | nothing |
| Model | Auto or Sonnet (mostly checking and small fixes) |
| Budget | about ½ Claude window / 40–80 Kiro credits |
| Car | no |

## Read

- `CONTEXT.md`, `INTEGRATION.md`, `PROGRESS_FW.md`.
- `spec/design.md`: "C1. `bitpack.h`", "C2. `crit.h` and the saturation counter", "D0. Encoding conventions", "C16. Host_Test_Suite", Properties 1, 2, 3, 9.
- `spec/requirements.md`: Requirement 1 (criteria 1.4, 1.8–1.10), 2.12, 9.5, 9.6, 10.1, 10.2, 18.8, 18.9.
- The existing code: `app_program/bitpack.h`, `crit.h`, `os_glue.c` (search `crit_dsp_`, `sat_note`), `hal_board.c` (search `crit_irq_`), `tests/host/`.

## State at the start (verified 1 Nov)

- `bash tests/host/run_all.sh` **fails at the build**: `make: No rule to make target 'build/obj/app/wire.o'`. `tests/host/Makefile` lists `LIB_SRCS += wire.c` and `STUB_SRCS += wire_export.c`, but `wire.c` was reverted.
- `tests/host/py/test_wire_basic.py` and `py/wire_ctypes.py` (from the reverted attempt) need `wire.c`.
- `tests/host/p3_full.log` and `tests/host/p3_ps.txt` are committed by mistake (a test log and a process listing).
- `tasks.md` marks don't match the code (`PROGRESS_FW.md` has the real state).

## Work

1. **Repair the host build without losing the wire tests:**
   - In `tests/host/Makefile`, include `wire.c` and `wire_export.c` only when the file exists:
     `ifneq ($(wildcard $(APP)/wire.c),)` ... `endif`.
   - In `py/test_wire_basic.py`, skip the module cleanly when the library has no `wire_*` symbols (`pytest.skip(..., allow_module_level=True)`). The tests then come back by themselves in FW-01b.
   - Check `test_static.sh` for wire assumptions, and guard them the same way.
2. **Remove the stray files:** `git rm tests/host/p3_full.log tests/host/p3_ps.txt`, and add `tests/host/*.log` and `tests/host/p3_*.txt` to `tests/host/.gitignore`.
3. **Run `bash tests/host/run_all.sh --seed 12345`** and fix what fails in 3.1–3.4 (bitpack and Properties 1–3). Keep the behaviour the design specifies. Don't change `bp_quant`'s rounding (flag R5 is undecided), but if a Property 1 case shows a difference from `jb_fixed` rounding, list it in the session log for R5.
4. **Task 3.5 (crit.h, sat counter):** the code is present but unverified.
   - Check that `crit_dsp_enter/exit` and `crit_irq_enter/exit` match design C2.
   - Check that `sat_note` holds at 65,535.
   - Check that `crit_stub.c` / `sat_stub.c` implement the same prototypes and count nesting.
   - Check that both firmware builds still have 0 warnings.
   - Add a small pytest for `sat_note` saturation and stub nesting if none exists.
5. **Task 3.7:** Property 9 (Code 39 index mapping is a bijection over the 43 characters; any other character gives 63; 63 maps back to 0), using `c39_to_index` / `c39_from_index` via ctypes. Add `code39.c` to the library list only if it's not there (it is: `LIB_SRCS` has it).
6. **Update `PROGRESS_FW.md`:**
   - 3.1–3.7 → `done` with the commit;
   - 1.4, 1.5, 1.8, 1.9 → `done` if the suite and the static checks pass;
   - session log.

## Watch out

- Don't create `wire.c` here. That's FW-01b.
- Hypothesis seeds: keep `--seed` reproducible. A flaky property is a bug, not bad luck.

## Gate

The firmware gate in `CONTEXT.md` section 5. All three commands must pass. Record the host totals (properties, other tests) in the session log.

## Hand-over

- **Commit:** `CR1 FW-01a: repair host test build, verify bitpack/crit, Property 9`.
- **Session log:** what was fixed in 3.1–3.5, the host totals, and any R5 rounding cases found.
