# LT-03: Switch the five laptop tools to the codec (`--legacy-json` on)

| | |
|---|---|
| Track | Laptop (branch `cr1/laptop`) |
| Tasks | 14.6, 14.7, 14.8, 14.9, 14.18, checkpoint 15 |
| Needs first | LT-02 done |
| Model | Sonnet |
| Budget | about 1 Claude window / 120–200 Kiro credits |
| Car | **yes**: checkpoint 15 (the tools against the current robot, which still speaks JSON) |
| Unblocks | merge = sync point **S3** (FW-08 builds may then be flashed) |

## Read

- `PROGRESS_LT.md`, `INTEGRATION.md` sections 3 and 5.
- `spec/design.md`: "C15. Laptop tool changes", "C14" (the legacy mode).
- `spec/requirements.md`: 14.8, 17.3–17.6, 17.8, 17.9.
- **`REVIEW_FLAGS.md` R7: important for this step.**
- The tools: `MQTT/telemetry_monitor.py` (`CSV_COLS`, the L1–L6 checks, `keyboard`), `MQTT/is02_tool.py` (`show`, `send_and_wait_ack`, `cmd_*`), `MQTT/robot_params.py`, `MQTT/robot_map.py`, and `MQTT/dashboard.html` (`MiniMqtt`, `send`, `onMessage`, `FAULTS`, the map buttons, **the line-speed box**).

## Work

1. **14.6 `telemetry_monitor.py`:**
   - `telemetry()`, `status()` and `ack()` take decoded dicts.
   - Keep `CSV_COLS` and the L1–L6 thresholds.
   - Sequence gaps use the 15-bit modular difference; the L1 rate uses the 24-bit modular time.
   - L2 counts codec-rejected payloads.
   - Match acks by command code plus ping number or letter.
   - Show a parameter-table mismatch warning from the heartbeat.
   - Add `--legacy-json` (default on).
2. **14.7:**
   - **`is02_tool.py`:**
     - `show()` prints decoded dicts;
     - `cmd`, `set`, `map` and `config` encode before publishing, with the same topics, retain flags and ack waits (4 s, 3 s for `set`);
     - `log` writes `pc_time, topic, len, hex, decoded_json`;
     - a config publish is refused while the parameter-table version differs.
   - **`robot_params.py`:** ignore blank and `#` lines; NAME → Parameter_ID; a validation error prints the line number and NAME, and nothing is published; `--clear` publishes the empty retained payload.
   - **`robot_map.py`:** builds the Map_Frame and waits for the MAP ack; `--clear` publishes the empty retained payload and MAP:CLEAR.
   - Add `--legacy-json` to each (default on).
3. **14.8 `dashboard.html`:**
   - Load `is02_codec.js`.
   - `MiniMqtt` passes the raw `Uint8Array` (no `TextDecoder` on the payload); `publish()` accepts bytes or text.
   - `onMessage` uses `IS02.decode`.
   - Add the fault names `WIFI_LOSS`, `MQTT_LOSS`, `US_NO_ECHO` (or R1's names if accepted).
   - Add the parameter-table mismatch banner and the "legacy JSON" checkbox (default on).
   - Labels and units stay unchanged. **Keep the line-speed box and its 3.00–15.00 cm/s check.**
4. **14.9:** write `MQTT/README.md`: how the tools use the codecs, the `--legacy-json` option, the dependencies.
5. **14.18 `tests/host/py/test_tools.py`** with a mock MQTT client:
   - each action publishes the right topic, retain flag and frame;
   - an ack timeout prints the baseline message;
   - the CSV header equals `CSV_COLS`;
   - the thresholds are unchanged;
   - the mismatch warning appears and clears;
   - a config publish is refused on mismatch.

## The rule for commands in legacy mode (flag R7)

**While `--legacy-json` / the checkbox is on, every outgoing command, map and config is sent as baseline TEXT** (`SET:...`, `MAP:...`, `NAME=VALUE` lines), exactly as today. Binary frames are sent only when legacy mode is off. **Received** payloads are auto-detected in both modes: `{` means JSON, otherwise binary.

The reason: until FW-09 the robot accepts only text commands. A dashboard that sent a binary ESTOP to such a robot would not stop it. LT-05 switches the default after FW-09.

## Watch out

- **The dashboard must keep working from `file://`** with no server, and the tools with Python 3 + `paho-mqtt` only.
- **Robot-side files are out of bounds:** don't edit anything under `app_program/`.

## Gate

The laptop gate in `CONTEXT.md` section 5. Also open `MQTT/dashboard.html` in a headless browser if one is available (Playwright is pre-installed in the cloud workspace), and confirm the page loads with no console errors.

## Check (user, checkpoint 15)

1. Flash the current `Poh-Loon` firmware (still JSON at this point), or use a robot running the Baseline_Build.
2. Run the dashboard, `telemetry_monitor.py`, `is02_tool.py cmd PING:1`, `robot_map.py` and `robot_params.py` with the default legacy mode.
3. **Check:** everything works as before, including dashboard ESTOP and the line-speed box.

## Hand-over

- **Commit:** `CR1 LT-03: tools use the Laptop_Codec (legacy JSON default)`.
- **Merging into `Poh-Loon` = sync point S3.** Add a line to `PROGRESS_FW.md` "Car checks waiting": "S3 met: FW-08 builds may be flashed". The LT track may write this one line.
