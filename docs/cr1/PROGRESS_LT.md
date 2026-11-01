# Laptop track: progress ledger

**This is the single source of truth for the laptop track** (spec Step 7, the tool half of Step 9, and the laptop tests). Every session reads this file first and updates it last, in the same commit as the code. Kiro sessions also tick here.

Status values: `todo`, `present-unverified`, `done` (gate passed, with a commit), `car-open`, `car-ok`, `waiting` (blocked by a sync point).

## Current state (2 Nov 2026)

LT-05 is done: all five tools send binary by default (`--legacy-json` / dashboard box ticked = baseline text, only for a Baseline_Build robot). Checkpoints 15 and 19 need the car (`car-open`). Merging `cr1/laptop` is S3, and S5 once LT-05 is in.

LT-01 to LT-04 are done on `cr1/laptop` (with `origin/Wei-En` merged: S1 and S2 met). The full gate passes: `rm -rf tests/host/build && bash tests/host/run_all.sh --seed 12345` ends `Host_Test_Suite: PASSED`.
- The five tools use `MQTT/is02_codec.py` and `MQTT/is02_codec.js`. Received payloads are auto-detected. Commands, map and config stay baseline text by default (`--legacy-json`, dashboard box ticked).
- **The firmware on this branch accepts binary commands only** (`docs/CR1_FW_STATUS.md`). LT-05 makes binary the tools' default, so they can command it.
- The C-side drift guard is in: `py/test_codec_tables.py` and `js/tables.test.js` compare both codecs' tables with `build/wire_dump`.
- Checkpoint 15 needs a robot (`car-open`). Merging `cr1/laptop` into `Poh-Loon` is sync point **S3**.

## Ledger

| Guide | Task (spec/tasks.md) | Status | Commit | Note |
|---|---|---|---|---|
| LT-01 | 14.1 Python field layer (`MQTT/is02_codec.py`) | done | `CR1 LT-01` on `cr1/laptop` | `test_codec_basic.py` passes; `run_all.sh` passes (LT-04 gate, 2 Nov) |
| LT-01 | 14.2 Python presentation layer | done | `CR1 LT-01` on `cr1/laptop` | flags R4, R7 open: spec followed as written; `run_all.sh` passes (LT-04 gate, 2 Nov) |
| LT-02 | 14.3 JavaScript field layer (`MQTT/is02_codec.js`) | done | `CR1 LT-02` on `cr1/laptop` | `codec_basic.test.js` passes; `run_all.sh` passes (LT-04 gate, 2 Nov) |
| LT-02 | 14.4 JavaScript presentation layer | done | `CR1 LT-02` on `cr1/laptop` | same choices as the LT-01 design readings; `run_all.sh` passes (LT-04 gate, 2 Nov) |
| LT-03 | 14.6 telemetry_monitor.py | done | `CR1 LT-03` on `cr1/laptop` | `test_tools.py` passes; 6,135 JSON messages through old and new monitor: CSV identical; `run_all.sh` passes (LT-04 gate, 2 Nov) |
| LT-03 | 14.7 is02_tool.py, robot_params.py, robot_map.py | done | `CR1 LT-03` on `cr1/laptop` | `test_tools.py` passes; run as real processes with paho-mqtt against a throwaway broker; `run_all.sh` passes (LT-04 gate, 2 Nov) |
| LT-03 | 14.8 dashboard.html | done | `CR1 LT-03` on `cr1/laptop` | line-speed box and its 3.00–15.00 check unchanged; checked in headless Chrome from `file://`; `run_all.sh` passes (LT-04 gate, 2 Nov) |
| LT-03 | 14.9 MQTT/README.md | done | `CR1 LT-03` on `cr1/laptop` | `run_all.sh` passes (LT-04 gate, 2 Nov) |
| LT-03 | 14.18 tool unit tests | done | `CR1 LT-03` on `cr1/laptop` | `test_tools.py`: 96 passed; `run_all.sh` passes (LT-04 gate, 2 Nov) |
| LT-03 | checkpoint 15 (tools vs current JSON robot) | car-open | | **needs the car**; merge = sync point **S3** |
| LT-04 | 14.5 (Python half) `py/make_vectors.py`, `vectors/vectors.json` | done | `CR1 LT-04` on `cr1/laptop` | `py/make_vectors.py` writes `vectors/vectors.json` and `vectors.txt` (seed 20261101, 345 vectors, committed); `make vectors-c` runs them through C |
| LT-04 | 14.10 Property 24 (parameter table version) | done | `CR1 LT-04` on `cr1/laptop` | S2 met: C (`params_table_crc`), Python and JS agree on (55, 9469) |
| LT-04 | 14.11 Property 25 | done | `CR1 LT-04` on `cr1/laptop` | passes in `run_all.sh` |
| LT-04 | 14.12 Property 26 | done | `CR1 LT-04` on `cr1/laptop` | passes in `run_all.sh` |
| LT-04 | 14.13 Property 27 | done | `CR1 LT-04` on `cr1/laptop` | JS part kept (flag R6 still open) |
| LT-04 | 14.14 Property 28 | done | `CR1 LT-04` on `cr1/laptop` | passes in `run_all.sh` |
| LT-04 | 14.15 Property 29 | done | `CR1 LT-04` on `cr1/laptop` | passes in `run_all.sh` |
| LT-04 | 14.16 Property 30 | done | `CR1 LT-04` on `cr1/laptop` | passes in `run_all.sh` |
| LT-04 | 14.17 presentation unit tests | done | `CR1 LT-04` on `cr1/laptop` | passes in `run_all.sh` |
| LT-04 | 16.4 Property 16 (event detail text) | done | `CR1 LT-04` on `cr1/laptop` | builders via ctypes + `event_golden.json` (84 hand-written texts, every event and variant) |
| LT-05 | 18.4 tools publish binary by default | done | `CR1 LT-05` on `cr1/laptop` | S4 treated as met; `test_tools.py`: 97 passed; `run_all.sh` passes (LT-05 gate, 2 Nov); merge = sync point **S5** |
| LT-05 | checkpoint 19, laptop part | car-open | | **needs the car**, together with FW-09's car check (table below) |

## Car / bench checks waiting

| Check | From | Done | Result |
|---|---|---|---|
| Checkpoint 19 (laptop part), with FW-09's car check, tools at their new binary defaults. **Needs a robot flashed with an FW-09 build.** 1. Flash an `APP_PERF=0` build. 2. **Straight after flashing**, re-publish the retained messages in binary: `py MQTT/robot_params.py --broker <ip>` (the CONFIG ack must show `applied` = the number of active lines and `rejected` 0), then send the map once (dashboard Barcode mapping panel, `robot_map.py` or `is02_tool.py map`), so the retained `is02/config` and `is02/map` are binary. 3. Check: dashboard START / STOP / PING / SET / GET:PARAMS / map; `is02_tool.py cmd PING:1`; `is02_tool.py set LINE_STEER=0.35`; `robot_map.py`; remote ESTOP from the dashboard and from the monitor (`e` key), `pwm_off_us` ≤ 10,000; BTN1 recovery; one barcode turn and one obstacle bypass | LT-05 / FW-09 | no | car-open |
| Checkpoint 15: dashboard, `telemetry_monitor.py`, `is02_tool.py cmd PING:1`, `robot_map.py`, `robot_params.py` in the default legacy mode against the current JSON firmware. Everything as before, including dashboard ESTOP and the line-speed box | LT-03 | no | car-open |

## Session log

Newest first. One entry per session: date, guide, tool (Claude chat / Kiro + model), commit, gate results (host totals), open issues, what the user must check.

- **2026-11-02, LT-05 (Claude Code, local clone, Sonnet 5.5).** Commit `CR1 LT-05: tools send binary by default` on `cr1/laptop`, on top of `CR1 LT-04`.
  - **S4 treated as met, confirmed in the firmware (read only):** `app_net.c` decodes with `wire_dec_cmd` and `wire_dec_map` (and the Config frame); `tsk_telemetry.c` handles the one-byte E-stop fast path (`(WIRE_FORMAT_VERSION << 4) | WCMD_ESTOP`) and the queued binary commands; `APP_LEGACY_TEXT_CMD` defaults to 0 in `build_make/robot_defaults.mk` and `docs/CR1_FW_STATUS.md` says the robot accepts binary only. No firmware-track file was touched.
  - **Gate: passed.** `rm -rf tests/host/build && PYTHON=~/is02venv/bin/python bash tests/host/run_all.sh --seed 12345` (WSL): `properties=533 failed=0 | other tests passed=1201 failed=0`, `Host_Test_Suite: PASSED`. `python3 -m py_compile MQTT/*.py` and `node -e "require('./MQTT/is02_codec.js')"` passed.
  - **Changes:** `--legacy-json` defaults to off in `telemetry_monitor.py`, `is02_tool.py`, `robot_params.py`, `robot_map.py` (`--no-legacy-json` is still accepted and does nothing new); the dashboard box is unticked (still not remembered, unticked at every load); docstrings and help texts updated. `MQTT/README.md` section 6: new default, `--legacy-json` only against a Baseline_Build robot, the retained-message step after flashing (`robot_params.py`, then the map once), troubleshooting row. In legacy mode the tools still send text and read JSON (R7 rule from LT-03).
  - **Tests (`test_tools.py`, 97 passed):** default flipped in the fixtures; legacy tests now pass `legacy_json=True` / `--legacy-json`; new `test_binary_estop_is_a_one_byte_frame` (1 byte, `(version << 4) | ESTOP`, from the monitor and from `is02_tool.py cmd`); the source checks now expect `default=False` and an unticked dashboard box.
  - **Flags:** none applied. R7 is now resolved in practice (legacy = text, default = binary) but its decision stays `open` for the team.
  - **Must do (car, checkpoint 19):** see the table above, including re-publishing the retained `is02/config` and `is02/map` in binary right after flashing. Merging `cr1/laptop` = S5 (and S3 if not merged before); `PROGRESS_FW.md` "Car checks waiting" has the S5 line.

- **2026-11-02, LT-04 (Claude Code, local clone, Sonnet 5.5).** Commit `CR1 LT-04: shared vectors and cross-codec properties` on `cr1/laptop`, on top of the merge of `origin/Wei-En` (S1 and S2 met, `params_table_crc` = 9469).
  - **Gate: passed.** `rm -rf tests/host/build && PYTHON=<venv python> bash tests/host/run_all.sh --seed 12345`: `properties=533 failed=0 | other tests passed=1200 failed=0`, `Host_Test_Suite: PASSED`; seed 777 also passed. `python3 -m py_compile MQTT/*.py` and `node -e "require('./MQTT/is02_codec.js')"` passed; `make -C tests/host vectors-c` runs the 345 vectors (28 are the reject vectors, status != 0 as intended). Run in WSL (Ubuntu 24.04, Node 18.19, `npm ci` done). WSL had no pip and no `python3-venv`, so a venv without pip was made in `~/is02venv` and pip came from the Ubuntu `python3-pip-whl` package; `tests/host/.venv` in the clone exists but has no pytest, and run_all.sh picks it first, so the gate was started with `PYTHON=~/is02venv/bin/python`. 14.1 to 14.9 and 14.18 are `done`.
  - **First fix:** `test_codec_basic.py::test_param_table_matches_app_params` now compares names, order, store widths and the CRC with the `wire_dump` JSON (the 16 packed rows no longer match the old `PARAMS[]` row regex). **The limits (min, max) of the parameters are no longer checked against C**: `wire_dump` has no limits. Add them to `wire_dump` (a request below) if the team wants that check back.
  - **What was added** (all LT-owned; no firmware-track file was touched):
    - `py/codec_gen.py` (seeded generators for the 7 payload kinds, the `wire_vectors` line of a record, a running `wire_vectors` process), `py/event_ref.py` (the `evt.c` builders through ctypes and the reference formatter, which uses the firmware's own `jb_fixed`), `py/make_vectors.py`, `py/cross_cases.py` (cases for the JS tests).
    - `vectors/vectors.json` and `vectors.txt` (seed 20261101, committed; `test_vectors_file_is_current` fails when they are stale) and `vectors/event_golden.json` (84 texts from the Baseline_Build `post_event` / `event2` calls, one per event and variant, with the `evt.c` builder calls).
    - Python: `test_codec_tables.py` (drift guard), `test_codec_cross.py` (P25 to P28), `test_codec_params.py` (P24, P29, P30), `test_codec_events.py` (P16), `test_presentation.py` (14.17). JavaScript: `tables.test.js`, `cross.test.js` (P16, P25 to P28), `params.test.js` (P24, P29, P30), `presentation.test.js`, helper `cross_helper.js`.
    - Each property prints its `P<n> ... | cases | failed | seed` lines: 1,000 telemetry, heartbeat and ack sets, 100 or more per Event_Code (200 for P16), 1,000 per command code, map and config, 1,000 per kind for P27, 345 shared vectors (+100 random picks in Python). Checked once by hand that the tests can fail: a changed event text and changed field widths in either codec each failed the expected tests.
  - **How the JavaScript half gets C bytes:** the JS codec cannot call C, so `js/cross_helper.js` runs `py/cross_cases.py --seed <IS02_SEED>` (stdlib only, `$PYTHON` or `python3`), which writes `build/cross/cases_<seed>.json` with bytes from the C Wire_Codec (P25, P27, P16) or bytes from the Python codec that C was shown to decode to the same fields (P26). The Python tests draw their own cases with Hypothesis, so a JS failure is a JS failure. P28 replays the committed vectors in all three.
  - **Where the guide was unclear, and how it was read:**
    - **P27 valid payloads:** the bytes are C-encoded frames of generated records (padding 0, codes in range, thresholds consistent: the C telemetry decoder rejects `thr_valid=0` with non-zero thresholds, so generated frames honour that). R6 is open, so the JS part of P27 is implemented.
    - **P24:** `params_table_crc` takes no list, so C is compared on the real table (via ctypes, 55 and 9469); for random name lists Python and JS are compared with an independent CRC written in the test, and an added, removed or swapped name must change the version.
    - **P28 reject vectors:** one frame per kind with a wrong version, one byte short, one byte long and zero length; all three decoders must reject them (the guide only requires the laptop codecs for the telemetry-side kinds; C rejects them too).
    - **P29:** the tool input is `parse_config` / `encodeConfig` (the shared parsing of `robot_params.py` and `is02_tool.py config`); the bad line is reported as (line number, NAME) and nothing is encoded.
    - **P30:** an Event_Code 45..63, a Reason_Code 28..31 and an ack command code 12..14 come back as the number and are logged. An unknown code in the Command_Frame is not "shown": `decode_fields` refuses it.
  - **Requests from LT to the firmware track (C-side; not done here):**
    1. **`SIDE_FOLLOW` near distance:** `tsk_obstacle.c` `side_follow_event()` sends `nr > 0 ? ... : 0` for the `near` field, so the baseline's "no echo" (`near_cm = -1.0f`, printed `near -1.0cm`) arrives as 0 and prints `near 0.0cm`. The codec (and the LT-01 log entry) read 8191 as -1.0. FW should send 8191 when `near_cm < 0` (the golden entry for this is built with `evt_u(8191, 13)`). Behaviour difference on the laptop only, visible only in that event text.
    2. `app_program/evt.h` comment: the argument lists for `SIDE_FOLLOW`, `OBSTACLE_PROFILE`, `OBST_REVERSE_CAP`, `OBSTACLE_TIE` and `BYPASS_BLOCKED` disagree with design D10 (the call sites follow D10; the comment does not).
    3. Optional: add each parameter's min and max to `wire_dump` so the limits can be checked against C again.
  - **Flags:** none applied, all open (R1, R4, R6, R7).
  - **Must do:** merge `cr1/laptop` into `Poh-Loon` only after checkpoint 15 (car) if the team wants it first; the merge is S3. At that merge add to `PROGRESS_FW.md` "Car checks waiting": "S3 met: FW-08 builds may be flashed". LT-05 waits for S4 (FW-09); the firmware already accepts binary commands only.
- **2026-11-01, LT-03 (Claude Code, local clone, Opus 5.5).** Commit `CR1 LT-03: tools use the Laptop_Codec (legacy JSON default)` on `cr1/laptop`, on top of `CR1 LT-02`.
  - **Gate: not passed.**
    - `bash tests/host/run_all.sh --seed 12345` was **not run**: it still fails before any test at the host library build (`wire.c` in `tests/host/Makefile`). No firmware-track file was touched. FW-01a repairs it.
    - Run directly instead, all passed: `py -m pytest tests\host\py\test_codec_basic.py tests\host\py\test_tools.py -q` (472 passed: 376 + 96); `node --test tests/host/js/codec_basic.test.js` (163 passed: 161 + 2, with the Node from LT-02); `node -e "require('./MQTT/is02_codec.js')"` (exit 0); `py -m py_compile MQTT/*.py` (exit 0).
    - **Host totals** aren't available until `run_all.sh` works. 14.6 to 14.9 and 14.18 stay `present-unverified`.
    - `test_tools.py` brings its own fake `paho.mqtt.client`, so it needs no paho-mqtt in the host-test venv.
  - **One-off checks, not committed:**
    - **Dashboard:** headless Chrome through Playwright 1.50.1 (the copy in `%LOCALAPPDATA%\ms-playwright-go`, with the installed Chrome; nothing installed), `MQTT/dashboard.html` from `file://`, 0 failed checks. **No console error or warning with a broker.** With no broker the only console error is Chrome's own refused WebSocket, as on the baseline. The broker was a mock on the page's WebSocket, so the real `MiniMqtt` path ran. One JSON telemetry message (the README sample with the extra firmware fields; **no real recording exists in the repo**) and the same values as a codec frame gave the same 23 panel values. Also covered: text and binary retained map and config, bad frames dropped with the display unchanged, the banner, fault chips, the buttons in both modes, the speed box limits (3.00, 15.00 sent; 2.99, 15.01 refused), the box ticked again after a reload.
    - **Monitor, old against new:** `70a14fc` and this commit on the same 6,135 random JSON messages (ints, `8.0`, nulls, missing fields, gaps, restarts): CSV byte-identical (752,648 bytes), console lines identical. The summary differs only in the L2 wording below.
    - **Real processes:** `is02_tool.py` (`cmd`, `set`, `map`), `robot_map.py`, `robot_params.py` and `telemetry_monitor.py` with paho-mqtt 2.1.0 against a throwaway Python broker and a scripted robot, in both modes. Text went out in legacy mode, binary with `--no-legacy-json`, and the acks were read in both. Not a substitute for checkpoint 15.
  - **Where the guide was unclear, and how it was read:**
    - **Receiving (R7):** every tool calls the codec with legacy JSON on in both modes. The flag only changes what is sent.
    - **Text on `is02/cmd`, `is02/map`, `is02/config`:** the broker can hold baseline text, which `decode` alone would drop as a bad version. On these topics a payload without Format_Version 1 is shown as text.
    - **Two helpers added to both codecs:** `read_payload` / `readPayload` for the two points above, and `ptab_mismatch` / `ptabMismatch`, instead of copies in each tool. Nothing else in the codecs changed. `test_tools.py` and `codec_basic.test.js` cover them.
    - **Validation in legacy mode:** `set`, `is02_tool.py config` and `robot_params.py` check NAME and value in both modes (Req 17.9; the guide doesn't limit it to binary). Valid input is sent as the same baseline text. So **a bad `set` or config line now stops at the laptop, where it used to reach the robot and get a nack**. `cmd` and `map` aren't checked in legacy mode: `PING:D...` and two letters for one movement (R4) are legal text.
    - **Config refused on mismatch:** in both modes, also in `robot_params.py` (the guide names only `is02_tool.py`, but Req 14.8 covers every tool). A one-shot tool has no "last heartbeat", so it waits up to 1.5 s for one; with none it publishes as before, which adds 1.5 s while the robot is offline. A JSON heartbeat has no `ptab`, so it neither raises nor clears a warning.
    - **L2:** counts rejected telemetry payloads only, as the baseline did. The summary line now says "rejected by the codec: N" where it said "invalid JSON: N". The pass rule is unchanged.
    - **Sequence on JSON:** the 15-bit rule applies to JSON too, as C15 says. A restart from above 16,383 frames (about 2.3 h of telemetry) now counts as a gap, not a restart.
    - **Dashboard pings:** legacy keeps `PING:D<base36>` exactly. Binary uses 49152 and up, clear of the monitor's 1, 2, 3...
    - **Dashboard speed after a binary SET:** the binary ack has no value, so the dashboard remembers what it sent. The box and its check are unchanged.
    - **Legacy box:** deliberately not remembered (`autocomplete="off"`, no `localStorage`). A stale "off" would turn ESTOP into a frame an old robot ignores.
    - **Dashboard sequence check:** left as it was, C15 doesn't list it. In binary it logs a false "robot restarted" once when the 15-bit `seq` wraps, after about 4.5 h.
  - **Flags:** none applied, all open. R7 is followed as the guide and the user's instructions say. R1: spec fault names added (`WIFI_LOSS`, `MQTT_LOSS`, `US_NO_ECHO`).
  - **Must do:** get FW-01a merged, merge `Poh-Loon` into `cr1/laptop`, run the gate, and only then mark 14.1 to 14.9 and 14.18 `done` and merge. Run checkpoint 15 on the robot. At that merge, add to `PROGRESS_FW.md` "Car checks waiting": "S3 met: FW-08 builds may be flashed". It isn't written yet, because S3 isn't met until the merge.
- **2026-11-01, LT-02 (Claude Code, local clone, Sonnet 5.5).** Commit `CR1 LT-02: JavaScript Laptop_Codec` on `cr1/laptop`, on top of `CR1 LT-01`.
  - **Gate: not passed.**
    - `bash tests/host/run_all.sh --seed 12345` was **not run**: it fails before any test, at the host library build (`wire.c` in `tests/host/Makefile`), as logged for LT-01. `tests/host/Makefile` is FW-owned and wasn't touched. FW-01a repairs it.
    - Run directly instead, all passed: `py -m pytest tests\host\py\test_codec_basic.py -q` (376 passed); `node --test tests/host/js/codec_basic.test.js` (161 passed); `node -e "require('./MQTT/is02_codec.js')"` (exit 0); `py -m py_compile MQTT/*.py` (exit 0).
    - **Host totals** aren't available until `run_all.sh` works. 14.3 and 14.4 stay `present-unverified` until it passes with FW-01a merged.
  - **Node:** this laptop has no Node on PATH. The commands above ran with the `node.exe` (v22.13.1) that the Playwright Go package ships in `%LOCALAPPDATA%\ms-playwright-go\1.50.1\`. Nothing was installed. WSL has no Node either. `npm ci` in `tests/host/js/` wasn't run: `codec_basic.test.js` needs only `node:test`, not `fast-check`.
  - **One-off parity check, not committed:** 31,239 comparisons of `is02_codec.js` against `is02_codec.py`, 0 mismatches. It used 4,800 random valid records of every kind (all 45 events, all ack and command codes, with edge values) for bytes both ways and for `decodeFields`; 6,000 random and truncated payloads for `decode` (the object and every log line, with and without legacy JSON); 4,064 command texts, 4,068 map inputs and 1,507 config inputs for the bytes or the exact error text (including `CodecError.errors`).
  - **Design:** a UMD file with no packages. `<script src="is02_codec.js">` gives the global `IS02`; `require()` gives the same object. The record and object field names are the Python ones (`spd_l`, `n_rej`, ...), so decoded objects are equal. The one test `require()`s the file and also runs it in a `vm` context with only `self`, to check the browser path.
  - **Same readings as LT-01:** all the design readings of the LT-01 entry are kept, and the parity check covers them (the `args` dict, IMU chip codes, `SIDE_FOLLOW` −1 as 8191, the `BC_DBG` variants, unknown codes as numbers, acks that depend on the command code only, 4-decimal `GET:PARAMS` values through `jbFixed`, the map text rules, the numbers grammar, `encodeConfig`).
  - **Differences from Python that can't be removed:**
    - Argument shapes are JavaScript ones: `decode(topic, payload, {legacyJson, log})` takes an options object, with `log` defaulting to `console.warn`. `encode*` return a `Uint8Array`. `decode` takes a `Uint8Array`, a Node `Buffer`, an `ArrayBuffer`, an array or a string (UTF-8).
    - Python `1.0` and JavaScript `1` are one Number. `JSON.stringify` of a decoded object prints `1`, where Python prints `1.0`. Compare parsed values, not text (LT-04).
    - Invalid-JSON log lines end with each engine's own error text. (`rssi` is computed as `0 - x` so that 0 never becomes `-0`; that one matches Python.)
    - `\d` in the numbers grammar is ASCII-only in JavaScript. Python's also accepts other Unicode digits.
  - **Flags:** none applied, all open (R1, R4, R7 as in LT-01).
  - **For LT-03:** the dashboard calls `IS02.decode(topic, bytes, {legacyJson: true})` with the bytes from the MQTT client (`message.payloadBytes` in Paho). Everything in the LT-01 "For LT-03" list holds for the JS side too.
  - **Must do first (as for LT-01):** get FW-01a merged, merge `Poh-Loon` into `cr1/laptop`, run the gate, and only then mark 14.1 to 14.4 `done` and merge. No car check for this step.
- **2026-11-01, LT-01 (Claude Code, local clone, Opus 5.5).** Commit `CR1 LT-01: Python Laptop_Codec` on `cr1/laptop`, from `Poh-Loon` `542368f`.
  - **Gate: not passed.**
    - `bash tests/host/run_all.sh --seed 12345` fails before any test runs: `No rule to make target 'build/obj/app/wire.o'`. It fails the same way on the untouched `542368f`. FW-01a repairs it (`PROGRESS_FW.md`, "Current state"). The README's "LT-01 needs nothing first" missed this, so the first laptop step that can pass the gate is the one after FW-01a is merged.
    - `python3 -m py_compile MQTT/*.py`: passed.
    - `node -e "require('./MQTT/is02_codec.js')"`: not applicable, the JS codec is LT-02. Node isn't installed in WSL yet, and LT-02 needs it.
  - **Tests run directly:** `python3 -m pytest py/test_codec_basic.py`: 376 passed (host totals aren't available until `run_all.sh` works). They also check the event, reason, command, stop and abort tables against `wire_ids.h`, and the parameter table against `app_params.c`.
  - **One-off checks, not committed:** `jb_fixed()` against the real `jfmt.c` compiled with host gcc, 199,905 random and exact-half float32 values, 0 mismatches. 150,000 random frames through `decode()`: 70,935 accepted, the rest dropped cleanly, no exception.
  - **Setup:** WSL has no `python3-venv`, `pytest`, `hypothesis` or `node`, and installing them needs `sudo` (`LAPTOP_TRACK_START.md` section 2). This session's test packages were kept outside the repo.
  - **Flags:** none applied, all open. R4: `encode_map` raises `CodecError` when two letters name one movement. R7: `encode_command_text` always returns binary; keeping commands as text in legacy mode is the tools' job in LT-03. R1: no fault names are in the codec; `faults` stays the number, as in the baseline.
  - **Where the design was unclear, and how it was read. LT-02 must make the same choices:**
    - **`"args"` in decoded events:** C14 doesn't define it. It is the dict of raw argument fields, from `decode_fields`. `MISSION_COMPLETE` has its own `t` inside `args`.
    - **`IMU_OK` chip E2:** D10 gives no codes. Used `hal_imu_chip_t`: 0 none, 1 LSM303DLHC, 2 LSM303D. The address prints as two uppercase hex digits.
    - **`SIDE_FOLLOW` near:** D10 gives no sentinel, but the baseline sends −1 when nothing was seen. Read 8191 as "-1.0", like CM13. **FW-01c must encode −1 as 8191.** `limit` has no sentinel.
    - **`BC_DBG` argument bits:** D10's bit counts leave out the 3-bit variant, which comes first. `wire_ids.h` confirms 203 for 16 widths. An unknown variant (5..7) has no known length, so it raises `CodecError` and the frame is dropped.
    - **`BC_DBG` window text:** result "none" prints " stray mark (fewer than 9)" when n < 9, else " closed". The pass text always prints the stray-mark text, as in the baseline.
    - **Unknown codes:** `event` or `result` is the integer, not a string. Inside a detail text the number replaces the word, for example "A -> 6". Unknown cmd code: `cmd` is the integer. `decode_fields` keeps an unknown Event_Code or command code frame whole as `raw` hex, so re-encoding gives the same bytes.
    - **Acks:** the arguments depend on the command code only, so a rejected MAP ack still carries `map` and `n`, where the baseline nack had neither. BARCODE letter 26..31 echoes the number (`BARCODE:27`); unknown or not-found Parameter_ID too (`SET:63`).
    - **`GET:PARAMS` values:** rounded to 4 decimals with `jb_fixed`, as the baseline printed them, so float32 0.35 reads 0.35. The packed ones are floats (1.0).
    - **`is02/cmd`, `is02/map`, `is02/config`:** C14 gives no shape. `decode` returns `{"cmd",...}`, `{"map","n"}` and `{"count","params"}`. An empty retained payload returns `None` silently.
    - **Map text:** same rules as `nav_map_parse()`, including the positional order FWD, UTURN, LEFT, RIGHT, which differs from the frame's order forward, left, right, U-turn. Two changes, because the frame can't carry them: any letter given twice raises `CodecError` (the baseline pair form kept the last), and so do two letters for one movement (R4). `encode_map("")` raises "empty map", and `encode_map("CLEAR")` gives the all-absent frame.
    - **Numbers:** same grammar as `jf_parse_float`: no exponent, no NaN or infinity. Limits are compared as float32, as on the robot, so `SPEED_KP=0.1` passes.
    - **`encode_config`:** ignores `#` comments and blank lines, and also splits items on `;` and `,` as `params_set_many` does. It reports every bad line in `CodecError.errors`.
  - **For LT-03:**
    - The dashboard pings with `PING:D<base36>`, which a U16 ping number can't carry, so `encode_command_text` rejects it. The dashboard needs a numeric id, and its ack match in `onMessage` needs changing with it.
    - `robot_map.py` matches acks on `MAP:<payload>`; the decoded echo is `MAP`.
    - `robot_map.py` also accepts `LEFT=K`, which `encode_map` doesn't: normalise it first.
    - `decode` with `legacy_json=True` parses only payloads starting with `{`, as C14 says. Text commands seen on `is02/cmd` log as version 4 or 5.
  - **Must do first:** get FW-01a merged, merge `Poh-Loon` into `cr1/laptop`, run the gate, and only then mark 14.1 and 14.2 `done` and merge. No car check for this step.
- **2026-11-01, guides written (Claude chat).** No laptop-track work yet.
