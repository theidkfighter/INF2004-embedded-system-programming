# IS-02 Host_Test_Suite

Tests that build and run on the laptop with no robot hardware. They cover the
firmware's plain C modules (compiled for the PC), the Wire_Codec and the
Laptop_Codec. This is a first draft; later tasks add the property tests, the
Baseline_Build differential library and the static checks.

Everything here runs in WSL (or any Linux). Nothing in this directory is part
of the firmware build: the sources stay outside `app_program/`.

## One-time set-up in WSL

From a WSL shell (Ubuntu or Debian):

```bash
sudo apt update
sudo apt install -y build-essential python3 python3-venv nodejs npm

# go to this directory (quote the path: it contains spaces)
cd "/mnt/c/Users/<you>/OneDrive/Desktop/SIT MODS/YEAR 2/INF2004 - Embedded Systems/Assignment/INF2004-embedded-system-programming/tests/host"

python3 -m venv .venv
.venv/bin/pip install pytest hypothesis

(cd js && npm install)        # installs fast-check into js/node_modules
```

Check that WSL uses its own Node.js, not the Windows one on the `PATH`
(`/mnt/c/Program Files/nodejs`): `which node` must print `/usr/bin/node`, and
`node --version` must be 18 or later (`node --test` is needed). Run
`npm install` from WSL too, so `node_modules` matches the WSL Node.js.

`run_all.sh` uses `.venv/bin/python` when it exists, otherwise `python3`.
Override with `PYTHON=...`, `NODE=...` or `MAKE=...` if needed.

## Running

```bash
bash run_all.sh               # random seed
bash run_all.sh --seed 12345  # repeat a run exactly
bash test_selfcheck.sh        # check the failure report (see below)
```

`run_all.sh`:

1. builds `build/libis02.so` with `make` (host `gcc -std=gnu11 -fPIC`, the
   firmware warning flags `-Wall -Wextra -Wshadow -Wdouble-promotion` and
   `-Werror`) and the Baseline_Build `build/libbase.so` with `make base`
   (needs `git`, `objcopy` and `nm`; rebuilt only when `baseline/` or the
   recorded commit changes);
2. runs the pytest + Hypothesis tests in `py/` with `--hypothesis-seed=<seed>`;
3. runs the fast-check tests in `js/` (`node --test`) with `IS02_SEED=<seed>`;
4. runs the static structure checks (`test_static.sh`, see below);
5. prints one summary line per property and exits non-zero on any failure.

Summary line format (Requirement 18.8):

```
P<n> <title> | <payload type or Storage_Unit> | cases=<n> | failed=<n> | seed=<s>
```

`cases` counts every input the property was run with. For a failing
Hypothesis property it also counts the shrink steps. On a failure
(Requirement 18.9) the script prints the property, the payload type or
Storage_Unit, the shrunk input values and the seed, then the command that
repeats the run. Full runner output is in `build/results/pytest.log` and
`build/results/node.log`.

To build only the library: `make -C tests/host` (`make -C tests/host clean`
removes `build/`).

## Failure-report self-check (`--selfcheck`)

`bash test_selfcheck.sh` checks that a failing property is reported in full
and fails the run (Requirements 18.8, 18.9):

```bash
bash test_selfcheck.sh              # seed 12345
bash test_selfcheck.sh --seed 777
```

It runs `run_all.sh --selfcheck --seed <s>`. In that mode `run_all.sh` runs
only two properties that fail on purpose, through the same pytest,
`node --test` and `summary.py` path as a normal run:

| File | Runner | Unit | Shrunk input |
|---|---|---|---|
| `py/selfcheck_fail.py` | Hypothesis | `selfcheck.u8` | `v=200` |
| `js/selfcheck_fail.js` | fast-check | `selfcheck.u16` | `[1000]` |

Neither file matches the default test patterns (`test_*.py`, `*.test.js`), so
a normal `run_all.sh` never runs them. `--selfcheck` skips the library build
and the static checks, writes to `build/selfcheck/` (the last real run in
`build/results/` is kept), and always exits non-zero: 1 when the properties
failed as intended, 2 when they unexpectedly passed. The JavaScript property
is skipped when `node` or fast-check is not installed.

`test_selfcheck.sh` prints the `run_all.sh` output, then one `PASS`, `FAIL`
or `SKIP` line per check: the exit status is non-zero, and for each runner
the summary line has `failed=` of 1 or more and the seed, and the
`FAILED property P0 ...` block names the unit, the seed and the shrunk input.
It exits 0 only when every check passed. It is not part of `run_all.sh`; run
it after changing `summary.py`, `py/conftest.py`, `js/prop.js` or
`run_all.sh`.

## Static structure checks

`bash test_static.sh` reads the `app_program/` sources (comments removed) and
prints one `PASS`, `FAIL` or `SKIP` line per check (Requirements 9.5, 19.1,
19.2, 19.3, 19.6):

- `pure_logic:<module>`: the nine pure-logic modules include, directly or
  through other `app_program/` headers, none of `hal.h`, `hal_internal.h`,
  `pins.h`, `tk/tkernel.h` or a Pico SDK header;
- `plain_header:<name>`: each accessor header (`bitpack.h`, `crit.h`,
  `fault_mask.h`, `status.h`, `msg_codec.h`, `evt.h`, `wire_ids.h`, `wire.h`,
  `perf.h`) includes only C standard headers and `app_program/` headers that
  in turn include no micro T-Kernel or Pico SDK header, and compiles on its own with the host compiler (`APP_PERF=0` and `1`); a
  header that does not exist yet is skipped;
- `no_dynamic_alloc`: no `malloc`, `calloc`, `realloc` or `free` call;
- `memory_pools`: one `tk_cre_mpf` call (as in the Baseline_Build), no
  `tk_cre_mpl`;
- `tasks`: seven `start_task` calls, one per baseline task, priorities 2, 4,
  5, 6, 7, 8, 10, and `tk_cre_tsk` only inside `start_task`;
- `evf_bits`: the 18 `EVF_*` bits at the baseline positions and no other
  single-bit `EVF_*` macro.

The baseline values are written in the script (Baseline_Build commit
`bca7964`). `run_all.sh` runs it, writes its records to
`build/results/static.jsonl` and its output to `build/results/static.log`;
the checks are counted under "other tests" in the summary.

## Layout

| Path | Contents |
|---|---|
| `Makefile` | Builds `build/libis02.so`. `LIB_SRCS` lists the `app_program/` sources; later tasks append to it. `STUB_SRCS` lists the host stubs. `make base` builds `build/libbase.so` with the `BASELINE_COMMIT=` hash from `docs/evidence/README.md` (override with `BASELINE_COMMIT=<hash>`). |
| `baseline/fetch_baseline.sh <commit>` | Extracts the Baseline_Build pure-logic sources and headers with `git show` into `build/base/src/`, compiles them with the harness files below, prefixes every symbol with `base_` (`objcopy --prefix-symbols`) and links `build/libbase.so`, used by the differential Properties 11, 13, 14, 15 and 22. The script header lists exactly what is extracted and how. |
| `baseline/base_motion.c`, `shim/tk/tkernel.h` | Compiles the Baseline_Build `tsk_motion.c` on the PC (no-op kernel shim, captured `post_motion`) and exports the MC_TURN / MC_BYPASS timeout, path length and encoder count to done. |
| `baseline/base_params.c` | Reads the Baseline_Build `PARAMS[]` float table by index (name, min, max, value). |
| `baseline/base_hump.c` | Baseline_Build hump profiler parameters, copied from the `tsk_imu.c` task body at build time. |
| `stubs/host_stubs.h` | Prototypes of the stubs and their test control functions. Uses `app_program/crit.h` and `evt.h` once they exist. |
| `stubs/crit_stub.c` | `crit_dsp_enter/exit`, `crit_irq_enter/exit`: counts entries and exits, tracks the depth, records nesting errors (a nested dispatch-disabled section, an exit without an entry, IRQ sections left out of order). |
| `stubs/sat_stub.c` | `sat_note`, `sat_count`: the Saturation_Counter rule (holds at 65,535), each update in one dispatch-disabled section. |
| `stubs/os_stub.c` | Captures posted message bodies in order, models the 40-block Message_Pool (a post fails when no block is free, or when forced empty), event flag helpers `flag_set/clear/peek/take`, and `post_evt` once `evt.h` exists. |
| `py/conftest.py` | `ctypes` loaders (`lib` fixture, `load_is02()`, `bind()`, `reset_stubs()`, `captured_messages()`; `base` fixture, `load_base()`, `base_motion_target()`, `base_commit()` for `libbase.so`; the `base` fixture skips the test when `libbase.so` was not built), the Hypothesis profile (no example database, no deadline), and the result recorder used by the summary. |
| `py/test_*.py` | pytest tests. |
| `js/package.json` | Depends on `fast-check`; `npm test` runs `node --test`. |
| `js/prop.js` | `checkProperty(meta, property, params)`: runs a fast-check property with the suite seed and records it for the summary. |
| `js/*.test.js` | `node --test` tests. |
| `summary.py` | Turns the result records into the summary. |
| `run_all.sh` | Single entry point. |
| `test_static.sh` | Static structure checks (see above). |
| `test_selfcheck.sh`, `py/selfcheck_fail.py`, `js/selfcheck_fail.js` | Failure-report self-check and its deliberately failing properties (see above). |

## Writing a property test

Python (one test per design property, at least 100 examples):

```python
# Feature: minimum-bit-data-representation, Property 4: Packed accessors round-trip and keep fields independent
@pytest.mark.prop(4, "Packed accessors round-trip and keep fields independent", unit="status.mot_a")
@settings(max_examples=1000)
@given(...)
def test_p4_mot_a(lib, ...):
    reset_stubs(lib)          # once per example
    ...
```

JavaScript:

```js
// Feature: minimum-bit-data-representation, Property 25: <title>
test('P25 telemetry', () => {
  checkProperty({ n: 25, title: '<title>', unit: 'Telemetry_Frame' },
                fc.property(arb, (v) => { /* ... */ }), { numRuns: 1000 });
});
```

Tests that are not tagged with `prop` still run and are counted under
"other tests" in the summary.
