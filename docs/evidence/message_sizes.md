# Message size comparison (Requirement 23)

Builds and commits are listed in [README.md](README.md). Both builds use `APP_PERF=0` and `APP_SELFTEST=0`.

Reduction percentage everywhere in this file = (Baseline_Build size − Revised_Build size) ÷ Baseline_Build size × 100, rounded to 0.1 % (Requirement 23.5). For MQTT topics the mean payload size is used.

## 1. Inter-task message bodies and Message_Pool block (Requirements 23.1, 23.5)

Filled by: `tools/evidence/type_sizes.sh`, run once against the Baseline_Build headers and once against the Revised_Build headers (2 Nov 2026, WSL). All values come from the RP2040 target compiler (`arm-none-eabi-gcc -c`, sizes read with `arm-none-eabi-nm -S`), not from a host build. Commands:

```sh
tools/evidence/type_sizes.sh -c bca79643d6f9bdd3938c353fa5f6fe791fc68f94 -e <baseline worktree>/build_make/is02_robot_smp0.elf
tools/evidence/type_sizes.sh -c 36a380bc3d8daf0f945e6c6853db1ab53914633e -e <revised worktree>/build_make/is02_robot_smp0.elf
```

The `-c` option reads the headers of that commit (not the working tree). The raw reports are in `docs/evidence/raw/baseline_type_sizes_2026-11-02.md` and `docs/evidence/raw/revised_type_sizes_2026-11-02.md`.

| Message | Baseline_Build type | Baseline_Build `sizeof` (bytes) | Revised_Build type | Revised_Build `sizeof` (bytes) | Reduction (%) |
|---|---|---|---|---|---|
| Motion command (`g_mbx_motion`) | `motion_cmd_t` | 32 | `motion_msg_t` | 12 | 62.5 |
| Navigation event (`g_mbx_nav`) | `nav_event_t` | 2 | `nav_msg_t` | 1 | 50.0 |
| Obstacle profile (`g_mbx_obstacle`) | `obstacle_profile_t` | 64 | `obst_msg_t` | 24 | 62.5 |
| Telemetry event (`g_mbx_tlm`) | `tlm_event_t` | 92 | `evt_msg_t` | 32 | 65.2 |
| Message_Pool block (header + largest body, rounded to 4) | `g_mpf_msg` block | 96 | `g_mpf_msg` block | 36 | 62.5 |
| Message_Pool, whole (`MSG_BLOCKS` 40 × block; kernel-allocated, flag R3) | 40 × 96 | 3840 | 40 × 36 | 1440 | 62.5 |

Cross-check with the linked ELF (the `-e` option): `g_mpf_buf` is 4000 bytes in the Baseline_Build ELF and 1600 bytes in the Revised_Build ELF, which is `MSG_BLOCKS` 40 × (block words + 1 word) and gives the same block sizes (96 and 36 bytes) as the headers. The kernel does not use `g_mpf_buf` (flag R3), so the last row of the table is the pool the kernel really allocates: see [ram_usage.md](ram_usage.md) section 6.

Compiler used: arm-none-eabi-gcc (15:13.2.rel1-2) 13.2.1 20231009. Commits: Baseline_Build `bca79643d6f9bdd3938c353fa5f6fe791fc68f94`, Revised_Build `36a380bc3d8daf0f945e6c6853db1ab53914633e` (firmware of the Week 13 submission; the later commits change documents and `tools/evidence/` only).

## 2. MQTT application payload sizes per topic (Requirements 23.2, 23.5, 23.6)

> **Status: to be measured on the car.** No track run has been recorded yet, so every table in this section is empty on purpose. Record one `is02_tool.py log` file per build during one full track run, then fill the tables from the output of `msg_sizes.py`:
>
> ```sh
> python MQTT/is02_tool.py --legacy-json --broker <LAPTOP_IP> log docs/evidence/raw/baseline_mqtt_<date>.csv   # Baseline_Build run
> python tools/evidence/msg_sizes.py docs/evidence/raw/baseline_mqtt_<date>.csv --commit bca79643d6f9bdd3938c353fa5f6fe791fc68f94
> python MQTT/is02_tool.py --broker <LAPTOP_IP> log docs/evidence/raw/revised_mqtt_<date>.csv                  # Revised_Build run
> python tools/evidence/msg_sizes.py docs/evidence/raw/revised_mqtt_<date>.csv --commit 36a380bc3d8daf0f945e6c6853db1ab53914633e
> ```

Filled by: `tools/evidence/msg_sizes.py <log.csv>`, run on one `is02_tool.py log` file per build recorded during one full track run. Sizes are the MQTT application payload only (no fixed header, topic name or packet identifier). The mean is given to 0.1 byte.

### How to record the log

1. Start the broker and the robot as for a normal track run.
2. In a second terminal, start the logger before the run: `python MQTT/is02_tool.py --broker <ip> log docs/evidence/raw/<build>_mqtt_<date>.csv`. It subscribes to `is02/#`, so it also records `is02/cmd`, `is02/map` and `is02/config` sent by the laptop.
3. During the run, send at least one command (for example a PING or a BARCODE override) and one ESTOP so `is02/cmd` and `is02/ack` carry payloads. Publish the retained map and config once (`robot_map.py`, `robot_params.py` or `is02_tool.py map` / `config`) so `is02/map` and `is02/config` appear in the log.
4. Stop the logger with Ctrl+C after the robot stops at the track end, then run `python tools/evidence/msg_sizes.py docs/evidence/raw/<build>_mqtt_<date>.csv`.

For the Baseline_Build run, the laptop tools use `--legacy-json` once they have been switched to the Laptop_Codec (Step 7). Before that, the unchanged tools read the JSON directly. `msg_sizes.py` accepts both the Baseline_Build log format (`pc_time, topic, payload`) and the new format (`pc_time, topic, len, hex, decoded_json`).

### Baseline_Build (commit `bca79643d6f9bdd3938c353fa5f6fe791fc68f94`)

Log file: to be measured

| Topic | Count | Min (bytes) | Mean (bytes, 0.1) | Max (bytes) | Source (track run / separate session) |
|---|---|---|---|---|---|
| `is02/telemetry` | | | | | |
| `is02/status` | | | | | |
| `is02/event` | | | | | |
| `is02/ack` | | | | | |
| `is02/cmd` | | | | | |
| `is02/map` | | | | | |
| `is02/config` | | | | | |

### Revised_Build (commit `36a380bc3d8daf0f945e6c6853db1ab53914633e`)

Log file: to be measured

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

Filled by: `tools/evidence/frame_budget.py`, which reads the `wire_dump` JSON of the Revised_Build (`tests/host/build/wire_dump`, built from the commit above). Command and output (2 Nov 2026):

```sh
tests/host/build/wire_dump | python3 tools/evidence/frame_budget.py
telemetry: 37 fields, 432 bits, 54 bytes, 0 padding bits
heartbeat: 18 fields, 195 bits, 25 bytes, 5 padding bits
```

| Item | Value |
|---|---|
| Sum of Required_Bits of every field, including the Format_Version header | 432 |
| Frame length (bytes) | 54 |
| Padding bits (8 × frame length − sum of Required_Bits) | 0 (8 × 54 − 432) |

Each field's width equals its Required_Bits: `tools/evidence/gen_docs.py` computes `ceil(log2 N)` from the Valid_Range and Resolution of every Telemetry_Frame field and stops if it differs from the width in `wire_dump` (see [../data_representation.md](../data_representation.md) section 9). The `faults` field is 16 independent flags, so its Required_Bits is 16. `wire_ids.h` fixes the frame at `WIRE_TLM_BITS` = 432 and `WIRE_TLM_LEN` = 54 with `_Static_assert`s.

The Heartbeat_Frame (`is02/status`) is 195 bits, 25 bytes, 5 padding bits.

## 4. Telemetry_Frame size limit (Requirement 23.4)

Filled by: computed from section 2 (Baseline_Build `is02/telemetry` mean) and section 3 (frame length). The Baseline_Build mean needs the car run, so this table is **to be measured**; the Revised_Build length is already known.

| Item | Value |
|---|---|
| Baseline_Build `is02/telemetry` mean payload (bytes) | to be measured (section 2) |
| Limit: 0.25 × Baseline_Build mean (bytes) | to be measured |
| Revised_Build Telemetry_Frame length (bytes) | 54 |
| Result (length ≤ limit) | to be measured. It passes if the Baseline_Build mean is at least 216 bytes (54 ÷ 0.25) |

## 5. Notes and deviations

Record here any value that could not be measured as described, with the reason (Requirement 24.5).

- Section 2 and the 25 % check in section 4 wait for the car run (marked above). Sections 1 and 3 are measured.
- `is02/status` is the Heartbeat_Frame and `is02/event`, `is02/ack` are variable length, so their per-topic rows come from the recorded run, not from the table of `wire_dump`.
