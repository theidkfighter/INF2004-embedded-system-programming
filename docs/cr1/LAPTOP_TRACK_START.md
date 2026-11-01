# CR1 laptop track: start here

You're doing the **laptop track** of Change Requirement 1 (minimum-bit data representation): the Python and JavaScript Laptop_Codecs, the five laptop tools, and the laptop tests. The firmware track runs in parallel. The two tracks meet only at the binary protocol, which is already fixed in `spec/design.md` (D7–D15) and `app_program/wire_ids.h`.

## 1. What's in `cr1_laptop_track.zip`

Unzip it into the root of your clone. It creates `docs/cr1/...`.

| File | Why you need it |
|---|---|
| `docs/cr1/README.md` | Order of the sessions, the starting prompt, budgets |
| `docs/cr1/CONTEXT.md` | Setup, the laptop gate (section 5), session rules |
| `docs/cr1/INTEGRATION.md` | Which files you may touch, and the sync points you wait on |
| `docs/cr1/PROGRESS_LT.md` | Your ledger: read it first, update it last |
| `docs/cr1/PROGRESS_FW.md` | Read only. Tells you when sync points S1, S2 and S4 are met (needed for LT-04 and LT-05) |
| `docs/cr1/REVIEW_FLAGS.md` | Flags R1, R4, R5, R6 and R7 affect your work |
| `docs/cr1/spec/requirements.md`, `design.md`, `tasks.md` | The spec snapshot the guides refer to |
| `docs/cr1/steps/laptop/LT-01` to `LT-05` | Your five sessions |
| `docs/cr1/LAPTOP_TRACK_START.md` | This file |

You don't need the firmware guides or `INT-01_final.md`.

## 2. Setup (WSL)

You **don't** need the ARM toolchain or the Pico SDK. The laptop track only needs:

- Python 3 with `pytest` and `hypothesis`;
- Node 18+, with `npm ci` in `tests/host/js`;
- host `gcc`, `git` and `objcopy` (`build-essential`), for the C test library LT-04 uses.

```sh
sudo apt update && sudo apt install -y build-essential python3 python3-venv nodejs npm
cd "<your clone>/tests/host"
python3 -m venv .venv && .venv/bin/pip install pytest hypothesis
(cd js && npm ci)
which node        # must print /usr/bin/node (WSL's Node, not the Windows one)
```

The project path may contain spaces: keep it in quotes.

## 3. How to run each session

1. Work on branch `cr1/laptop`, created from `Poh-Loon`.
2. Before each session, merge `Poh-Loon` into your branch (`git merge origin/Poh-Loon`).
3. Use a **fresh Claude Code session per guide** (`/clear` or a new session), opened in the repo root.
4. Start with LT-01. **LT-01 to LT-03 can start right away**; LT-04 and LT-05 wait for firmware sync points (see `INTEGRATION.md` section 3).

First message for each session (change the guide name):

> Read `docs/cr1/README.md`, `docs/cr1/CONTEXT.md`, `docs/cr1/INTEGRATION.md` and `docs/cr1/PROGRESS_LT.md`, then `docs/cr1/steps/laptop/LT-01_python_codec.md`, and carry out that step only. You are running locally in my clone: commit on `cr1/laptop` and push; ignore the patch instructions in CONTEXT.md section 3. Do not apply any item from REVIEW_FLAGS.md unless its decision says accepted.

## 4. Models

| Guide | Model |
|---|---|
| LT-01 Python codec | **Opus 5.5**, or `/model opusplan` (Opus plans, Sonnet edits; uses less of the plan) |
| LT-02 JS codec | Sonnet |
| LT-03 tools | Sonnet |
| LT-04 cross-codec tests | Sonnet |
| LT-05 binary default | Sonnet |

`/model` opens the picker, so you can see which versions your plan offers. Opus uses the Pro allowance noticeably faster than Sonnet.

## 5. Rules

- **Don't edit:**
  - `app_program/`;
  - the test runner files (`tests/host/Makefile`, `run_all.sh`, `py/conftest.py`, `stubs/`, `baseline/`, `tools/*.c`, `test_static.sh`);
  - `docs/cr1/PROGRESS_FW.md`.

  If you need something from the firmware side, write it as a note in your session log in `PROGRESS_LT.md`.
- **Your own files:** `MQTT/**`, new test files `tests/host/py/test_codec_*.py`, `test_tools.py`, `test_presentation.py`, `py/make_vectors.py`, `py/codec_helpers.py`, `tests/host/js/*.test.js`, `tests/host/vectors/`. `run_all.sh` picks up new `py/test_*.py` and `js/*.test.js` files by itself.
- **While legacy mode is on** (the default until LT-05), the tools send commands as **text**, because the robot only accepts text commands until firmware step FW-09 (flag R7).
- **Never commit** `config/wifi_credentials.h`, `config/mqtt_config.h` or any `.uf2`.

## 6. One thing to agree with your teammate

The guides aren't on GitHub yet, so both of you have copies of `docs/cr1/`. You each update only your own PROGRESS file, so merges won't clash. **Commit `docs/cr1/` to `Poh-Loon` once, from one of you, before you both start.** Otherwise the two copies will drift apart.
