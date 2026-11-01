# Message size comparison (Requirement 23)

Builds and commits are listed in [README.md](README.md). Both builds use `APP_PERF=0` and `APP_SELFTEST=0`.

Reduction percentage everywhere in this file = (Baseline_Build size − Revised_Build size) ÷ Baseline_Build size × 100, rounded to 0.1 % (Requirement 23.5). For MQTT topics the mean payload size is used.

## 1. Inter-task message bodies and Message_Pool block (Requirements 23.1, 23.5)

Filled by: `tools/evidence/type_sizes.sh`, run once against the Baseline_Build headers and once against the Revised_Build headers. All values come from the RP2040 target compiler (`arm-none-eabi-gcc -c`, sizes read with `arm-none-eabi-nm -S`), not from a host build.

| Message | Baseline_Build type | Baseline_Build `sizeof` (bytes) | Revised_Build type | Revised_Build `sizeof` (bytes) | Reduction (%) |
|---|---|---|---|---|---|
| Motion command (`g_mbx_motion`) | `motion_cmd_t` | | `motion_msg_t` | | |
| Navigation event (`g_mbx_nav`) | `nav_event_t` | | `nav_msg_t` | | |
| Obstacle profile (`g_mbx_obstacle`) | `obstacle_profile_t` | | `obst_msg_t` | | |
| Telemetry event (`g_mbx_tlm`) | `tlm_event_t` | | `evt_msg_t` | | |
| Message_Pool block (header + largest body, rounded to 4) | `g_mpf_msg` block | | `g_mpf_msg` block | | |

Compiler used: `<arm-none-eabi-gcc --version, first line>`. Commits: Baseline_Build `bca79643d6f9bdd3938c353fa5f6fe791fc68f94`, Revised_Build `<fill in>`.

## 2. MQTT application payload sizes per topic (Requirements 23.2, 23.5, 23.6)

Filled by: `tools/evidence/msg_sizes.py <log.csv>`, run on one `is02_tool.py log` file per build recorded during one full track run. Sizes are the MQTT application payload only (no fixed header, topic name or packet identifier). The mean is given to 0.1 byte.

### How to record the log

1. Start the broker and the robot as for a normal track run.
2. In a second terminal, start the logger before the run: `python MQTT/is02_tool.py --broker <ip> log docs/evidence/raw/<build>_mqtt_<date>.csv`. It subscribes to `is02/#`, so it also records `is02/cmd`, `is02/map` and `is02/config` sent by the laptop.
3. During the run, send at least one command (for example a PING or a BARCODE override) and one ESTOP so `is02/cmd` and `is02/ack` carry payloads. Publish the retained map and config once (`robot_map.py`, `robot_params.py` or `is02_tool.py map` / `config`) so `is02/map` and `is02/config` appear in the log.
4. Stop the logger with Ctrl+C after the robot stops at the track end, then run `python tools/evidence/msg_sizes.py docs/evidence/raw/<build>_mqtt_<date>.csv`.

For the Baseline_Build run, the laptop tools use `--legacy-json` once they have been switched to the Laptop_Codec (Step 7). Before that, the unchanged tools read the JSON directly. `msg_sizes.py` accepts both the Baseline_Build log format (`pc_time, topic, payload`) and the new format (`pc_time, topic, len, hex, decoded_json`).

### Baseline_Build (commit `bca79643d6f9bdd3938c353fa5f6fe791fc68f94`)

Log file: `<fill in>`

| Topic | Count | Min (bytes) | Mean (bytes, 0.1) | Max (bytes) | Source (track run / separate session) |
|---|---|---|---|---|---|
| `is02/telemetry` | | | | | |
| `is02/status` | | | | | |
| `is02/event` | | | | | |
| `is02/ack` | | | | | |
| `is02/cmd` | | | | | |
| `is02/map` | | | | | |
| `is02/config` | | | | | |

### Revised_Build (commit `<fill in>`)

Log file: `<fill in>`

| Topic | Count | Min (bytes) | Mean (bytes, 0.1) | Max (bytes) | Source (track run / separate session) |
|---|---|---|---|---|---|
| `is02/telemetry` | | | | | |
| `is02/status` | | | | | |
| `is02/event` | | | | | |
| `is02/ack` | | | | | |
| `is02/cmd` | | | | | |
| `is02/map` | | | | | |
| `is02/config` | | | | | |

### Topics with no payload during the track run (Requirement 23.6)

If a topic carried no payload during the recorded track run, take its values from a separate recorded session of the same build that contains at least one payload of each message type sent on that topic, and mark the row "separate session" above.

| Build | Topic | Separate session log file | Message types covered | Statement |
|---|---|---|---|---|
| | | | | Values taken from a separate session, not from the track run |

### Reduction per topic (Requirement 23.5)

Filled by: computed from the two tables above (mean payload size).

| Topic | Baseline_Build mean (bytes) | Revised_Build mean (bytes) | Reduction (%) |
|---|---|---|---|
| `is02/telemetry` | | | |
| `is02/status` | | | |
| `is02/event` | | | |
| `is02/ack` | | | |
| `is02/cmd` | | | |
| `is02/map` | | | |
| `is02/config` | | | |

## 3. Telemetry_Frame bit budget (Requirement 23.3)

Filled by: `tools/evidence/frame_budget.py`, which reads the `wire_dump` JSON of the Revised_Build (`tests/host/build/wire_dump`).

| Item | Value |
|---|---|
| Sum of Required_Bits of every field, including the Format_Version header | |
| Frame length (bytes) | |
| Padding bits (8 × frame length − sum of Required_Bits) | |

## 4. Telemetry_Frame size limit (Requirement 23.4)

Filled by: computed from section 2 (Baseline_Build `is02/telemetry` mean) and section 3 (frame length).

| Item | Value |
|---|---|
| Baseline_Build `is02/telemetry` mean payload (bytes) | |
| Limit: 0.25 × Baseline_Build mean (bytes) | |
| Revised_Build Telemetry_Frame length (bytes) | |
| Result (length ≤ limit) | `<pass/fail>` |

## 5. Notes and deviations

Record here any value that could not be measured as described, with the reason (Requirement 24.5).
