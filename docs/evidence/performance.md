# Performance evidence (Requirements 9.6, 10.2, 20, 24.4 to 24.7)

Builds and commits are listed in [README.md](README.md). Timing is compared between the Step 0 commit and the Revised_Build, both built with `APP_PERF=1` and otherwise the same command line, so both carry the same probes.

Filled by: `tools/evidence/perf_report.py <console.log>`, run on the USB console log of each build. It turns the `[perf] <name> n=<count> max=<us> avg=<us> over=<count>` lines into the tables below and marks each limit pass or fail.

## 1. How to record the timing runs

1. Build and flash the Step 0 commit with `make clean && make APP_ROBOT=1 APP_PERF=1` (exact line in [README.md](README.md)).
2. Open the USB serial console with logging to a file, for example `docs/evidence/raw/step0_console_perf_<date>.log`. Keep it open for all runs of this build.
3. Leave the robot in IDLE for at least 15 s after boot. TSK_TELEMETRY runs the encode benchmark 10 s after boot and prints `[perf] tlm_bench n=1000 max=.. avg=..` once.
4. Start `telemetry_monitor.py` (with `--legacy-json` for the Step 0 build once the tools use the Laptop_Codec, Step 7 on) so check L1 and the watchdog count are recorded for the same runs.
5. Do 3 consecutive full track runs. A run starts when the robot enters LINE_FOLLOWING at the track start (START command or BTN1) and ends when it stops at the track end. `start_run()` resets the probe counters, so the last `[perf]` block printed before the next START holds the totals of that run. Do not send ESTOP during these runs, so each run stays a full track run. The E-stop PWM-off times come from the track runs in [correctness.md](correctness.md) section 3, which include one remote ESTOP while driving.
6. Note for each run: track layout, start and end time, and whether any run had to be repeated.
7. Repeat steps 1 to 6 with the Revised_Build (`make clean && make APP_ROBOT=1 APP_PERF=1`, no `--legacy-json`).
8. Run `python tools/evidence/perf_report.py <console.log>` on each log and paste the output below.

Critical-section maxima (section 5) must include driving at `SPEED_MAX_MM_S` (250 mm/s). Line following is capped at 150 mm/s (`LINE_BASE_SAFE_MAX_MM_S` in `robot_config.h`), so the `[perf] cs_dsp` and `cs_irq` lines are also taken from the encoder course runs in [correctness.md](correctness.md) section 6.1, which drive at `SPEED_MAX_MM_S` with the same `APP_PERF=1` builds.

## 2. Test conditions (Requirement 20.6)

> **To be measured on the car.** Fill when the runs are done: commits `542368f` (Step 0) and the Week 13 commit, the exact `make` lines of [README.md](README.md), the track layout and the file names of the logs.

| Item | Step 0 commit (Baseline_Build timing) | Revised_Build |
|---|---|---|
| Commit | `<fill in>` | `<fill in>` |
| `make` command line | `<fill in>` | `<fill in>` |
| Track layout | `<fill in>` | `<fill in>` |
| Barcode map and `robot_params.txt` | `<fill in>` | `<fill in>` |
| Number of full track runs | 3 | 3 |
| Console log file | `<fill in>` | `<fill in>` |
| Monitor CSV and log files | `<fill in>` | `<fill in>` |

## 3. Task cycle response times (Requirements 20.1, 24.4)

> **To be measured on the car.** Fill from: `python tools/evidence/perf_report.py docs/evidence/raw/<build>_console_perf_<date>.log` on the USB console log of each `APP_PERF=1` build.

Filled by: `perf_report.py` (`[perf] tsk_safety`, `tsk_line`, `tsk_motion`, `tsk_imu`). The time runs from the cyclic handler release to the end of the cycle. `over` counts cycles above the limit and must be 0 in every run.

### Step 0 commit (Baseline_Build timing)

| Task | Run | Cycles (n) | Worst case (µs) | Average (µs) | Over limit | Limit | Result |
|---|---|---|---|---|---|---|---|
| TSK_SAFETY_MON | 1 | | | | | 10 ms | |
| TSK_SAFETY_MON | 2 | | | | | 10 ms | |
| TSK_SAFETY_MON | 3 | | | | | 10 ms | |
| TSK_LINE_BARCODE | 1 | | | | | 10 ms | |
| TSK_LINE_BARCODE | 2 | | | | | 10 ms | |
| TSK_LINE_BARCODE | 3 | | | | | 10 ms | |
| TSK_MOTION_CTRL | 1 | | | | | 20 ms | |
| TSK_MOTION_CTRL | 2 | | | | | 20 ms | |
| TSK_MOTION_CTRL | 3 | | | | | 20 ms | |
| TSK_TERRAIN_IMU | 1 | | | | | 20 ms | |
| TSK_TERRAIN_IMU | 2 | | | | | 20 ms | |
| TSK_TERRAIN_IMU | 3 | | | | | 20 ms | |

### Revised_Build

| Task | Run | Cycles (n) | Worst case (µs) | Average (µs) | Over limit | Limit | Result |
|---|---|---|---|---|---|---|---|
| TSK_SAFETY_MON | 1 | | | | | 10 ms | |
| TSK_SAFETY_MON | 2 | | | | | 10 ms | |
| TSK_SAFETY_MON | 3 | | | | | 10 ms | |
| TSK_LINE_BARCODE | 1 | | | | | 10 ms | |
| TSK_LINE_BARCODE | 2 | | | | | 10 ms | |
| TSK_LINE_BARCODE | 3 | | | | | 10 ms | |
| TSK_MOTION_CTRL | 1 | | | | | 20 ms | |
| TSK_MOTION_CTRL | 2 | | | | | 20 ms | |
| TSK_MOTION_CTRL | 3 | | | | | 20 ms | |
| TSK_TERRAIN_IMU | 1 | | | | | 20 ms | |
| TSK_TERRAIN_IMU | 2 | | | | | 20 ms | |
| TSK_TERRAIN_IMU | 3 | | | | | 20 ms | |

Requirement 20.1 result (zero cycles over the limit across the 3 Revised_Build runs): `<pass/fail>`

## 4. Interrupt handler execution times (Requirements 20.3, 24.6)

> **To be measured on the car.** Fill from: `python tools/evidence/perf_report.py docs/evidence/raw/revised_console_perf_<date>.log --baseline docs/evidence/raw/step0_console_perf_<date>.log`.

Filled by: `perf_report.py` (`[perf] isr_gpio`, `isr_timer`). Each build needs at least 10,000 invocations of each handler over the 3 runs. Limit for the Revised_Build worst case: the larger of 110 % of the Baseline_Build worst case and the Baseline_Build worst case + 1 µs.

| Handler | Build | Invocations over 3 runs (n) | Worst case (µs) | Average (µs) |
|---|---|---|---|---|
| `hal_gpio_isr` | Step 0 commit | | | |
| `hal_gpio_isr` | Revised_Build | | | |
| `hal_timer_isr` | Step 0 commit | | | |
| `hal_timer_isr` | Revised_Build | | | |

| Handler | Baseline_Build worst case (µs) | Limit (µs) | Revised_Build worst case (µs) | Revised as % of Baseline worst case | Result |
|---|---|---|---|---|---|
| `hal_gpio_isr` | | | | | |
| `hal_timer_isr` | | | | | |

Per-run values (Requirement 24.6 asks for one full track run per build; give run 1 here and keep the other runs in the raw log):

| Handler | Build | Run | Invocations (n) | Worst case (µs) | Average (µs) |
|---|---|---|---|---|---|
| `hal_gpio_isr` | Step 0 commit | 1 | | | |
| `hal_gpio_isr` | Revised_Build | 1 | | | |
| `hal_timer_isr` | Step 0 commit | 1 | | | |
| `hal_timer_isr` | Revised_Build | 1 | | | |

### Counted from the disassembly (not measured)

These numbers come from the ledger, session 4 (`docs/cr1/PROGRESS_FW.md`); they are instruction counts of the Cortex-M0+ code from the `bca7964` and current `hal_isr.c` compiled with the firmware's command line, not timings. The car measurement above decides the Requirement 20.3 result.

| `hal_timer_isr` path (ADC read to return) | Baseline_Build cycles | Revised_Build cycles |
|---|---|---|
| threshold mode, no edge | 67 | 59 |
| raw mode, ADC in range | 82 | 100 |
| raw mode, tick raises the ADC maximum | 83 | 108 |

At 125 MHz (`clk_sys`), 18 more cycles is 144 ns and 25 more is 200 ns on those two raw-mode paths. The raw-mode paths are slower than in the Baseline_Build because the one-word `g_raw` slot and the packed `g_adc_rng` word are composed in the ISR (kept by decision of 2 Nov 2026, see the ledger).

## 5. Critical-section durations (Requirements 9.6, 10.2)

> **To be measured on the car.** Fill from: `python tools/evidence/perf_report.py docs/evidence/raw/<build>_console_perf_<date>.log` (the `cs_dsp` and `cs_irq` lines), including the encoder course of `correctness.md` 6.1.

Filled by: `perf_report.py` (`[perf] cs_dsp`, `cs_irq`). Each section type needs at least 1,000 entries in a run that includes driving at `SPEED_MAX_MM_S`. Limit: 10 µs, measured from just before dispatch or interrupts are disabled to just after they are enabled again.

| Section | Build | Run (track run or encoder course) | Entries (n) | Worst case (µs) | Average (µs) | Limit | Result |
|---|---|---|---|---|---|---|---|
| Dispatch disabled (`cs_dsp`) | Step 0 commit | | | | | 10 µs | |
| Dispatch disabled (`cs_dsp`) | Revised_Build | | | | | 10 µs | |
| Interrupts masked (`cs_irq`) | Step 0 commit | | | | | 10 µs | |
| Interrupts masked (`cs_irq`) | Revised_Build | | | | | 10 µs | |

## 6. Telemetry frame production time (Requirements 20.4, 24.7)

> **To be measured on the car.** Fill from: `python tools/evidence/perf_report.py <console.log>` (`tlm_bench` and `tlm_encode` lines).

Filled by: `perf_report.py` (`[perf] tlm_bench` for the benchmark, `[perf] tlm_encode` for frames built during a track run). The time runs from reading the Status_Snapshot to having the complete payload in the output buffer and excludes the MQTT publish. Baseline_Build: JSON frame build. Revised_Build: Wire_Codec encode.

### Benchmark: 1,000 encodes of one fixed snapshot, robot IDLE (Requirement 20.4)

| Build | Encodes (n) | Mean (µs) | Maximum (µs) |
|---|---|---|---|
| Step 0 commit (JSON build) | | | |
| Revised_Build (Wire_Codec encode) | | | |

Requirement 20.4 result (Revised mean ≤ Baseline mean and Revised maximum ≤ Baseline maximum): `<pass/fail>`

### During one full track run (Requirement 24.7, at least 100 frames)

| Build | Run | Frames (n) | Worst case (µs) | Average (µs) |
|---|---|---|---|---|
| Step 0 commit (JSON build) | 1 | | | |
| Revised_Build (Wire_Codec encode) | 1 | | | |

## 7. E-stop PWM-off times (Requirements 13.4, 24.7)

> **To be measured on the car.** Fill from: check L6 of the `telemetry_monitor.py` summary in each track run of `correctness.md` section 3.

Filled by: `telemetry_monitor.py` summary, check L6 (`robot PWM off <n> us after the command arrived`), from the track runs in [correctness.md](correctness.md) section 3. One row per remote ESTOP sent while the motors were driven. Limit: 10 ms (10,000 µs).

| Build | Run | Round trip laptop to ack (ms) | PWM off after command arrived (µs) | Limit | Result |
|---|---|---|---|---|---|
| Baseline_Build | 1 | | | 10,000 µs | |
| Baseline_Build | 2 | | | 10,000 µs | |
| Baseline_Build | 3 | | | 10,000 µs | |
| Revised_Build | 1 | | | 10,000 µs | |
| Revised_Build | 2 | | | 10,000 µs | |
| Revised_Build | 3 | | | 10,000 µs | |

## 8. Watchdog E-stops (Requirements 20.2, 24.7)

> **To be measured on the car.** Fill from: the watchdog count in the `telemetry_monitor.py` summary (and the `FAULT_WATCHDOG` bit) of each track run.

Filled by: counting `EMERGENCY_STOP` events with the watchdog reason in the monitor log file of each run (`is02/event` lines). Limit: 0 per run.

| Build | Run | Watchdog E-stops | Result |
|---|---|---|---|
| Step 0 commit | 1 | | |
| Step 0 commit | 2 | | |
| Step 0 commit | 3 | | |
| Revised_Build | 1 | | |
| Revised_Build | 2 | | |
| Revised_Build | 3 | | |

## 9. Telemetry frame rate (Requirements 20.5, 24.7)

> **To be measured on the car.** Fill from: check L1 of the `telemetry_monitor.py` summary: `python MQTT/telemetry_monitor.py --broker <LAPTOP_IP>`.

Filled by: `telemetry_monitor.py` summary, check L1 (`<hz> Hz over <n> frames (robot clock; target 2 Hz)`). The window must be at least 60 s with at least 120 live frames, excluding frames replayed from the Frame_Ring. Limit: 1.8 Hz to 2.2 Hz.

| Build | Run | Window (s) | Live frames (n) | Rate (Hz) | Limit | Result |
|---|---|---|---|---|---|---|
| Step 0 commit | 1 | | | | 1.8 to 2.2 Hz | |
| Revised_Build | 1 | | | | 1.8 to 2.2 Hz | |

## 10. Summary against the Requirement 20 limits (Requirement 20.6)

> **To be measured on the car.** Fill from: fill from sections 3 to 9 after the car runs.

| Criterion | Measurement | Baseline_Build value | Revised_Build value | Samples | Conditions | Limit | Result |
|---|---|---|---|---|---|---|---|
| 20.1 | Task cycles over the 10 ms / 20 ms limit | | | | 3 full track runs | 0 | |
| 20.2 | Watchdog E-stops | | | | 3 full track runs | 0 per run | |
| 20.3 | `hal_gpio_isr` worst case | | | | 3 full track runs | max(110 %, +1 µs) | |
| 20.3 | `hal_timer_isr` worst case | | | | 3 full track runs | max(110 %, +1 µs) | |
| 20.4 | Telemetry frame build, mean | | | | 1,000 encodes, IDLE | ≤ Baseline_Build | |
| 20.4 | Telemetry frame build, maximum | | | | 1,000 encodes, IDLE | ≤ Baseline_Build | |
| 20.5 | Live telemetry rate | | | | ≥ 60 s, ≥ 120 frames | 1.8 to 2.2 Hz | |
| 9.6 / 10.2 | Critical section worst case (dispatch / interrupts) | | | | ≥ 1,000 entries, incl. `SPEED_MAX_MM_S` | 10 µs | |
| L6 | E-stop PWM-off time, worst | | | | one ESTOP per track run in `correctness.md` section 3 | 10 ms | |

## 11. Measurements outside a limit (Requirement 24.5)

> **To be measured on the car.** Fill from: fill from section 10 after the car runs.

Fill in one row for every value above that fails its limit, and for every E-stop PWM-off time above 10 ms. Leave the table empty if every result passes.

| Measurement | Measured value | Limit | Criterion (20.x or L6) | Identified cause |
|---|---|---|---|---|
| | | | | |
