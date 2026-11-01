# Laptop track: progress ledger

**This is the single source of truth for the laptop track** (spec Step 7, the tool half of Step 9, and the laptop tests). Every session reads this file first and updates it last, in the same commit as the code. Kiro sessions also tick here.

Status values: `todo`, `present-unverified`, `done` (gate passed, with a commit), `car-open`, `car-ok`, `waiting` (blocked by a sync point).

## Current state (1 Nov 2026)

LT-01 and LT-02 are coded on `cr1/laptop`, but **their gate can't pass yet**: `run_all.sh` stops at the host library build, because `tests/host/Makefile` still lists the reverted `wire.c`. That was already so on `542368f`. FW-01a repairs it. So 14.1 to 14.4 stay `present-unverified` until `run_all.sh` passes with FW-01a merged. Don't merge `cr1/laptop` into `Poh-Loon` before that.
- `MQTT/is02_codec.py` and `MQTT/is02_codec.js` are new. The five tools aren't switched to them yet (LT-03), so they still use JSON.
- `tests/host/js/package.json` already depends on `fast-check`.

## Ledger

| Guide | Task (spec/tasks.md) | Status | Commit | Note |
|---|---|---|---|---|
| LT-01 | 14.1 Python field layer (`MQTT/is02_codec.py`) | present-unverified | `CR1 LT-01` on `cr1/laptop` | `test_codec_basic.py` passes; `run_all.sh` blocked until FW-01a |
| LT-01 | 14.2 Python presentation layer | present-unverified | `CR1 LT-01` on `cr1/laptop` | flags R4, R7 open: spec followed as written; `run_all.sh` blocked until FW-01a |
| LT-02 | 14.3 JavaScript field layer (`MQTT/is02_codec.js`) | present-unverified | `CR1 LT-02` on `cr1/laptop` | `codec_basic.test.js` passes; `run_all.sh` blocked until FW-01a |
| LT-02 | 14.4 JavaScript presentation layer | present-unverified | `CR1 LT-02` on `cr1/laptop` | same choices as the LT-01 design readings; `run_all.sh` blocked until FW-01a |
| LT-03 | 14.6 telemetry_monitor.py | todo | | |
| LT-03 | 14.7 is02_tool.py, robot_params.py, robot_map.py | todo | | |
| LT-03 | 14.8 dashboard.html | todo | | keep the line-speed box |
| LT-03 | 14.9 MQTT/README.md | todo | | |
| LT-03 | 14.18 tool unit tests | todo | | |
| LT-03 | checkpoint 15 (tools vs current JSON robot) | todo | | **needs the car** (or a broker + recorded payloads); merge = sync point **S3** |
| LT-04 | 14.5 (Python half) `py/make_vectors.py`, `vectors/vectors.json` | waiting | | needs **S1** |
| LT-04 | 14.10 Property 24 (parameter table version) | waiting | | needs **S2** (`params_table_crc`) |
| LT-04 | 14.11 Property 25 | waiting | | needs **S1** |
| LT-04 | 14.12 Property 26 | waiting | | needs **S1** |
| LT-04 | 14.13 Property 27 | waiting | | needs **S1**; flag R6 |
| LT-04 | 14.14 Property 28 | waiting | | needs **S1** |
| LT-04 | 14.15 Property 29 | todo | | no C dependency |
| LT-04 | 14.16 Property 30 | todo | | no C dependency |
| LT-04 | 14.17 presentation unit tests | todo | | no C dependency |
| LT-04 | 16.4 Property 16 (event detail text) | waiting | | needs **S1** (C event builders) |
| LT-05 | 18.4 tools publish binary by default | waiting | | needs **S4** (FW-09 merged); merge = sync point **S5** |
| LT-05 | checkpoint 19, laptop part | waiting | | **needs the car** |

## Car / bench checks waiting

| Check | From | Done | Result |
|---|---|---|---|

## Session log

Newest first. One entry per session: date, guide, tool (Claude chat / Kiro + model), commit, gate results (host totals), open issues, what the user must check.

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
