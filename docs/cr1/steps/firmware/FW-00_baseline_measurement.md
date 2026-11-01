# FW-00: Baseline measurements on the car (checkpoint 2)

| | |
|---|---|
| Track | Firmware |
| Who | **The user, on the car**, not a chat. A chat can help read the results afterwards |
| Spec | tasks.md checkpoint 2; Requirements 20, 22, 23, 24; `docs/evidence/README.md` |
| Needs first | nothing. Can be done any time, but **before FW-02 is flashed** (the baseline must come from unchanged behaviour) |
| Blocks | nothing in code. All "before" numbers in the Week 13 evidence come from here |

## Why this matters

Every "before vs after" comparison needs numbers from the unchanged firmware. RAM and message sizes can be measured later from the Baseline_Build commit without the car. **Timing, track runs, L1–L6 and the functional checklist can't**: they need the car running the baseline firmware.

## Which builds

| Measurement | Build |
|---|---|
| Timing (task and ISR probes, telemetry encode time) | `542368f` with `APP_PERF=1` |
| Track runs, monitor L1–L6, functional checklist, barcode, ultrasonic, encoder course | Baseline_Build `bca7964` with `APP_PERF=0`, or `542368f` with `APP_PERF=0`: identical behaviour |
| RAM, `sizeof`, MQTT payload sizes | Baseline_Build `bca7964`, `APP_PERF=0` (`docs/evidence/README.md` explains why) |

Build on your laptop (WSL) with your real `config/wifi_credentials.h` and `config/mqtt_config.h` (`README.md` sections 3 and 5). For the Baseline_Build, use a worktree as `docs/evidence/README.md` shows (`git worktree add ../is02_baseline bca7964...`).

## Procedure

Follow the procedures already written in `docs/evidence/correctness.md` and `docs/evidence/performance.md` (task 1.7). In short:

1. **Timing build** (`APP_PERF=1`), USB console connected, laptop powered (lab rule: never USB and the power bank at the same time):
   - Boot, wait 10 s in IDLE: the `tlm_bench` runs, 1,000 JSON frame builds.
   - Then drive the track.
   - Save the console log (`[perf]` lines every 2 s) as `docs/evidence/raw/baseline_console_perf_<date>.log`.
2. **Track runs** (`APP_PERF=0`, power bank):
   - Run 3 full runs with the same map and `robot_params.txt`.
   - Have `py MQTT/telemetry_monitor.py --broker <ip>` and `py MQTT/is02_tool.py log` running.
   - Include at least one command with an ack (L5) and one dashboard ESTOP (L6).
   - Save the monitor CSV, log and summary, and the `is02_tool.py log` CSV, in `docs/evidence/raw/`.
3. **Hardware checks** (`correctness.md`):
   - 1,000 mm encoder course × 5 runs at `SPEED_MAX_MM_S`;
   - 5 barcode passes;
   - ultrasonic medians at 20 cm and 100 cm (20 readings each);
   - link loss, encoder loss and no-echo checks.
4. **Fill in the "Baseline_Build" columns** of `docs/evidence/performance.md` and `correctness.md` with `tools/evidence/perf_report.py` and `msg_sizes.py`, as the templates say.

## Done when

- The raw files are in `docs/evidence/raw/`.
- The baseline columns are filled.
- The "Car checks waiting" table in `PROGRESS_FW.md` has a row ticked with the date.

Commit: `CR1 FW-00: baseline measurements`.

## If the car isn't available

Coding can continue: FW-01a to FW-01c don't change robot behaviour. **Don't flash any build from FW-02 on as "baseline"**: from FW-02 on, telemetry carries the new fault bits.
