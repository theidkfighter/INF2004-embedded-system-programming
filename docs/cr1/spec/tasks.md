# Implementation Plan: Minimum-Bit Data Representation

## Overview

This plan implements Change Requirement 1 in the order of the design's rollout (Step 0 to Step 10). Firmware code is C (gnu11, Cortex-M0+, built by `build_make/robot.mk` with `APP_ROBOT=1`). The Laptop_Codec and tools are Python and JavaScript. The Host_Test_Suite uses pytest + Hypothesis (driving the C sources through `ctypes`) and node + fast-check.

Rules that apply to every task group:

- Each group must leave the firmware buildable with `make clean && make APP_ROBOT=1` in `build_make/` with no new warnings (`-Wall -Wextra -Wshadow -Wdouble-promotion`), and with the same robot behaviour as before the group.
- New firmware sources go in `app_program/` (picked up by the existing wildcard in `build_make/mtkernel_3/app_program/subdir.mk`). Host test sources stay outside `app_program/`. No change to `build_make/makefile` or `pico_rp2040.mk`.
- Keep the seven tasks, their priorities, the kernel objects, the module boundaries and the MQTT topic names unchanged (Requirement 19).
- Hardware measurements (track runs, on-target timing, `arm-none-eabi-size` comparisons, bench checks) are done by the user. Tasks here only produce the code, instrumentation, scripts and templates for them.
- Property tests carry the tag comment `# Feature: minimum-bit-data-representation, Property <n>: <title>` (`//` in JavaScript), run at least 100 cases (1,000 where the property says so), and report through the `run_all.sh` summary line format.

## Tasks

- [x] 1. Step 0: Baseline freeze, build switches, timing probes and test scaffolding
  - [x] 1.1 Add the build switches and the option guard
    - Add `APP_LEGACY_TEXT_CMD ?= 0`, `APP_PERF ?= 0`, `APP_SELFTEST ?= 0` to `build_make/robot_defaults.mk`
    - Pass `-DAPP_LEGACY_TEXT_CMD=$(APP_LEGACY_TEXT_CMD) -DAPP_PERF=$(APP_PERF) -DAPP_SELFTEST=$(APP_SELFTEST)` in `CFLAGS` in `build_make/robot.mk`
    - Add the `.robot_opts` guard modelled on the profile guard in `pico_rp2040.mk`: write `legacy$(APP_LEGACY_TEXT_CMD)-perf$(APP_PERF)-self$(APP_SELFTEST)` and delete `mtkernel_3/app_program/*.o` when the recorded value differs
    - _Requirements: 13.7, 13.9, 19.5_

  - [x] 1.2 Add the firmware build check script
    - Create `tools/evidence/build_check.sh`: runs `make clean && make APP_ROBOT=1` in `build_make/`, saves the log, counts compiler warnings, compares the count with a recorded Baseline_Build count, and lists any warning that names a file added or changed since the baseline commit
    - Exit non-zero on build failure, on more warnings than the baseline, or on any warning in a new or changed file
    - _Requirements: 19.5_

  - [x] 1.3 Add `perf.h` and the timing probes to the Baseline_Build code
    - Create `app_program/perf.h` with `perf_stat_t`, the probe ids (`PERF_ISR_GPIO` .. `PERF_CS_IRQ`), `PERF_BEGIN`/`PERF_END`, `perf_add`, `perf_release`, `perf_reset`, `perf_report`; with `APP_PERF=0` every macro expands to nothing and the functions are empty inline functions
    - Add `uint32_t hal_time_us32(void)` (one `timerawl` read) to `hal_board.c` and its prototype to `hal.h`
    - Define `g_perf[PERF_COUNT]`, `perf_add`, `perf_reset`, `perf_report` in `os_glue.c` under `#if APP_PERF`; call `perf_release()` from `cyc_tick` for the four periodic tasks
    - Add probe calls at entry/exit of `hal_gpio_isr` and `hal_timer_isr`, at the end of each cycle of TSK_SAFETY_MON, TSK_LINE_BARCODE, TSK_MOTION_CTRL, TSK_TERRAIN_IMU (10 ms / 20 ms limits counted in `over`), around the JSON telemetry frame build in `tsk_telemetry.c`, and around the existing `tk_dis_dsp`/`save_and_disable_interrupts` sections
    - Add the `tlm_bench` run in `tsk_telemetry.c` under `#if APP_PERF` (1,000 builds of one fixed snapshot, 10 s after boot while IDLE) and call `perf_report()` from `console_status()` every 2 s; call `perf_reset()` from `start_run()` in `tsk_nav.c`
    - _Requirements: 9.6, 10.2, 20.1, 20.3, 20.4, 24.4, 24.6, 24.7_

  - [x] 1.4 Create the Host_Test_Suite scaffolding
    - Create `tests/host/Makefile` that builds `build/libis02.so` from the pure-logic modules (`line_logic`, `imu_logic`, `obstacle_logic`, `barcode_logic`, `bc_detect`, `bc_multi`, `code39`, `nav_map`, `speed_ctl`) with host `gcc -std=gnu11 -fPIC`, the firmware warning flags and `-Werror`, using a source list that later tasks extend
    - Create `tests/host/stubs/` with `crit_stub.c` (counts enter/exit, checks nesting), `sat_stub.c` and `os_stub.c` (captures posted messages, stubs flags)
    - Create `tests/host/py/conftest.py` with the `ctypes` loader for `libis02.so`, and `tests/host/js/` with a `package.json` that depends on `fast-check` and runs `node --test`
    - Create `tests/host/run_all.sh`: builds the libraries, picks a seed (or takes `--seed`), passes it to pytest (`--hypothesis-seed`) and to the JS tests, prints one summary line per property `P<n> <title> | <payload type or Storage_Unit> | cases=<n> | failed=<n> | seed=<s>`, prints the failing property, unit/payload, input and seed on failure, and exits non-zero on any failure
    - Create a first draft of `tests/host/README.md` with the WSL set-up commands
    - _Requirements: 18.8, 18.9, 19.3_

  - [x] 1.5 Add the Baseline_Build differential harness
    - Create `tests/host/baseline/fetch_baseline.sh <commit>` that extracts the Baseline_Build pure-logic sources (and the motion target and hump profiler code needed by Properties 11, 13, 14, 15, 22) with `git show`, compiles them into `build/libbase.so` and prefixes every symbol with `base_` using `objcopy --prefix-symbols=base_`
    - Add the `libbase.so` loader to `tests/host/py/conftest.py` and a Makefile target that runs the script with the commit recorded in `docs/evidence/README.md`
    - _Requirements: 4.10, 5.9, 6.7, 7.7, 15.5_

  - [x] 1.6 Add the evidence scripts
    - `tools/evidence/ram_report.sh <elf> <map>`: unedited `arm-none-eabi-size` output, `.data`, `.bss`, sum, per-symbol sizes from `arm-none-eabi-nm -S` for `g_status`, `g_mpf_buf`, `g_frames`, `g_msgs`, `g_bc_ring`, `g_raw_er`+`g_raw_el`+`g_raw_adc`, `g_cmds`, `g_params` (with a replacement-symbol list for renamed items), git hash, make command line and `arm-none-eabi-gcc --version`, as a Markdown table
    - `tools/evidence/type_sizes.c` + `type_sizes.sh`: compiled with `arm-none-eabi-gcc -c` against either build's headers; prints target `sizeof` of each message body type and the Message_Pool block size from `arm-none-eabi-nm -S`
    - `tools/evidence/msg_sizes.py <log.csv>`: per topic count, min, mean (0.1 byte) and max payload bytes; accepts the Baseline_Build log format and the new `pc_time, topic, len, hex, decoded_json` format
    - `tools/evidence/perf_report.py <console.log>`: Markdown tables of the `[perf]` lines (n, max, avg, over) with pass/fail against the 10 ms / 20 ms / 10 µs / 110 % limits
    - _Requirements: 20.6, 22.1, 22.2, 22.3, 22.4, 22.6, 22.7, 23.1, 23.2, 23.5, 23.6, 24.4, 24.6, 24.7_

  - [x] 1.7 Create the evidence document templates
    - `docs/evidence/README.md`: Baseline_Build commit hash (recorded now), placeholders for the Revised_Build hash, make command lines, compiler version, index of evidence files, and which commit each comparison uses (RAM and size: parent commit; timing: this commit with `APP_PERF=1`)
    - `docs/evidence/ram_usage.md`, `docs/evidence/message_sizes.md`, `docs/evidence/correctness.md`, `docs/evidence/performance.md`: section headings and empty tables for every item of Requirements 20.6, 22, 23 and 24, with the script that fills each table named next to it
    - In `correctness.md`, write the user procedures for the hardware checks: 1,000 mm encoder course × 5 runs, 5 barcode passes, ultrasonic medians at 20 cm and 100 cm, 3 track runs per build, link loss / encoder loss / no-echo checks, `APP_SELFTEST=1` run
    - _Requirements: 3.3, 3.7, 3.8, 9.8, 10.7, 10.9, 10.10, 19.7, 19.10, 20.6, 22.1, 23.2, 24.1, 24.2, 24.3, 24.5, 24.8_

  - [x] 1.8 Write the static structure checks
    - `tests/host/test_static.sh`: pure-logic sources include none of `hal.h`, `hal_internal.h`, `pins.h`, `tk/tkernel.h` or Pico SDK headers; plain accessor headers include only standard and plain firmware headers; no `malloc`/`calloc`/`realloc`/`free` and no new `tk_cre_mpf`; seven `start_task` calls with priorities 2, 4, 5, 6, 7, 8, 10; 18 `EVF_*` bits at the baseline positions
    - _Requirements: 9.5, 19.1, 19.2, 19.3, 19.6_

  - [x] 1.9 Write the `run_all.sh --selfcheck` test
    - A deliberately failing property must print property, unit, input and seed and make `run_all.sh` exit non-zero
    - _Requirements: 18.8, 18.9_

- [~] 2. Checkpoint: Step 0 complete
  - Ensure `tools/evidence/build_check.sh` passes (build with `APP_PERF=0` matches the parent commit) and all tests pass, ask the user if questions arise. The user takes the baseline measurements from this commit with `APP_PERF=1`.

- [ ] 3. Step 1: Plain C foundation (bitpack, critical sections, Wire_Codec, message codec, event builders; not yet used by the tasks)
  - [-] 3.1 Create `app_program/bitpack.h`
    - `BP_MASK`, `BP_GET`, `BP_PUT`, `BP_STATIC_ASSERT`, `bp_sext`, `bp_tc`, `bp_set_checked` (rejects out-of-range values, leaves the unit unchanged), `bp_qspec_t`, `bp_quant` (double arithmetic, round half away from zero, clamp to the nearest non-sentinel limit, NaN to sentinel or lower limit, reports saturation), `bp_dequant`, `bp_sat_inc`
    - Only C standard headers; all literals suffixed so `-Wdouble-promotion` stays quiet; add to the host library source list
    - _Requirements: 1.1, 1.3, 1.4, 1.8, 1.9, 2.12, 9.5_

  - [~] 3.2 Write property test for continuous rounding
    - **Property 1: Continuous stores round to the nearest step**
    - Hypothesis over every `bp_qspec_t` used by the design tables (via `ctypes`)
    - **Validates: Requirements 1.8, 7.5, 4.4, 6.6**

  - [~] 3.3 Write property test for saturation and NaN
    - **Property 2: Out-of-range and NaN stores saturate and are counted**
    - Include one step past each limit, the input type min/max, ±infinity and NaN; also add `test_nan.py` covering NaN for every continuous field spec
    - **Validates: Requirements 1.4, 1.9, 4.9, 6.5, 7.6, 18.4**

  - [~] 3.4 Write property test for sentinel codes
    - **Property 3: Sentinel codes never collide with values**
    - **Validates: Requirements 1.3, 6.2, 11.10**

  - [~] 3.5 Add the critical-section hooks and the saturation counter
    - Create `app_program/crit.h` with `crit_dsp_enter/exit`, `crit_irq_enter/exit`, `sat_note`, `sat_count`
    - Implement `crit_dsp_*` (`tk_dis_dsp`/`tk_ena_dsp`, plus the `PERF_CS_DSP` probe), `sat_note` (dispatch-disabled add holding at 65,535) and `sat_count` with `volatile uint16_t g_sat_count` in `os_glue.c`
    - Implement `crit_irq_*` (`save_and_disable_interrupts`/`restore_interrupts`, plus the `PERF_CS_IRQ` probe) in `hal_board.c`
    - Make the host stubs `crit_stub.c` and `sat_stub.c` implement the same prototypes
    - _Requirements: 1.10, 9.2, 9.5, 9.6, 10.1, 10.2_

  - [x] 3.6 Add the Code 39 index helpers
    - Add `c39_to_index(char)` (0..42 in the Data_Representation_Table order, 63 for any other character) and `c39_from_index(uint32_t)` (63 → 0) to `code39.h/.c`
    - _Requirements: 5.3_

  - [~] 3.7 Write property test for the Code 39 index mapping
    - **Property 9: Code 39 index mapping is a bijection**
    - **Validates: Requirements 5.3**

  - [x] 3.8 Create `app_program/wire_ids.h`
    - X-macro tables: Format_Version (= 1), the 45 Event_Codes (D10), the 28 Reason_Codes with their baseline texts (D11), the 11 command codes and `UNPARSED` = 15 (D11, D12), the 10 stop reasons (D2), the 11 abort reasons (D10), fixed frame lengths (`WIRE_TLM_LEN` = 54, `WIRE_HB_LEN` = 25, map 4, command lengths per code) and `_Static_assert`s that each table fits its field width
    - _Requirements: 2.9, 8.2, 12.2, 12.4, 13.2, 16.1, 16.3_

  - [~] 3.9 Implement the Wire_Codec bitstream and robot-to-laptop frames in `wire.h/.c`
    - `bw_t`/`br_t` with `bw_init` (zero-fill), `bw_put` (MSB first, overflow flag), `bw_copy`, `bw_bytes`, `br_init`, `br_get`
    - `wire_tlm_t`, `wire_hb_t`, `wire_ack_t` holding field codes; `wire_enc_telemetry` (D8), `wire_enc_heartbeat` (D9), `wire_enc_event` (D10 header + `bw_copy` of the argument bits), `wire_enc_ack` (D11, including GET:PARAMS parts and CONFIG); and their inverse decoders for the host tests
    - Format_Version at byte 0 bits 7..4 in every frame; all padding bits 0
    - _Requirements: 11.1, 11.2, 11.3, 11.4, 11.5, 11.10, 12.1, 12.2, 12.3, 12.4, 12.6, 12.7, 16.1_

  - [~] 3.10 Implement the laptop-to-robot frames in `wire.c`
    - `wire_dec_command` (D12), `wire_dec_map` (D13, rejects bad letter and duplicate letter), `wire_dec_config` (D14, rejects bad count and length) returning `wire_status_t`; and the inverse encoders `wire_enc_command`, `wire_enc_map`, `wire_enc_config` for the host tests
    - Version 4 and 5 (text 'A'..'Z') always read as unknown version; zero length → `WIRE_BAD_LENGTH`
    - _Requirements: 13.1, 13.2, 13.5, 14.1, 14.2, 14.3, 14.7, 16.1_

  - [~] 3.11 Add the `wire_dump` host tool and the frame budget script
    - `tests/host/tools/wire_dump.c`: prints the Format_Version, every code table and every frame layout (field, offset, width, Valid_Range, Resolution, sentinel) from `wire_ids.h`/`wire.c` as JSON
    - `tools/evidence/frame_budget.py`: reads `wire_dump` JSON and prints the Telemetry_Frame Required_Bits sum, length in bytes and padding bits
    - _Requirements: 16.2, 16.6, 23.3_

  - [~] 3.12 Write property test for frame header and length
    - **Property 19: Every encoded frame has the fixed header and length**
    - **Validates: Requirements 11.4, 11.5, 12.7, 16.1**

  - [~] 3.13 Write unit tests for the Wire_Codec C side
    - `bw_put` past the buffer end sets `b_overflow`; C encode-then-decode for one example of each payload type and command code; `test_resolution.py` checks each telemetry field's resolution against the Baseline_Build JSON decimals
    - _Requirements: 1.2, 11.3, 13.1_

  - [~] 3.14 Implement `msg_codec.h/.c` (packed mailbox bodies)
    - `motion_msg_t` (12 bytes, D6), `nav_msg_t` (1 byte), `obst_msg_t` (28 bytes), `evt_msg_t` (28 bytes), and the encode/decode functions; encoders clamp to the nearest bound and return the clamp count; `msg_nav_encode` returns false for a bad type or letter; unused bits are 0
    - `_Static_assert`s for each body size
    - _Requirements: 1.7, 4.1, 4.2, 4.3, 4.9, 6.1, 6.3, 6.4, 6.5, 6.6, 8.1, 8.8_

  - [~] 3.15 Write property test for inter-task message round trip
    - **Property 10: Inter-task messages round-trip**
    - Cover every motion command type, nav events, obstacle profiles (including no-echo codes) and `evt_msg_t` bodies
    - **Validates: Requirements 4.2, 4.4, 6.6, 8.1, 8.6**

  - [~] 3.16 Implement the typed event builders `evt.h/.c`
    - `evt_begin`, `evt_u`, `evt_q`, `evt0` and one builder per Event_Code (D10) including `evt_state`, `evt_stop`, `evt_hump`, `evt_fine_scan` (posts `ceil(n / 11)` events) and `evt_bc_widths` (at most 6 parts of 16); `post_evt` prototype only (implemented in Step 8, captured by `os_stub.c` on the host)
    - _Requirements: 8.2, 8.3, 8.9, 12.2, 12.3, 12.8_

- [~] 4. Checkpoint: Step 1 complete
  - Ensure the firmware builds with `build_make/robot.mk` with no new warnings (new code is unused by the tasks) and all tests pass, ask the user if questions arise.

- [ ] 5. Step 2: Fault_Mask
  - [~] 5.1 Create `fault_mask.h` and the 16-bit Fault_Mask
    - Move the `FAULT_*` constants (positions 0..12 unchanged) from `app_types.h` into `app_program/fault_mask.h`, add `FAULT_WIFI_LOSS` (13), `FAULT_MQTT_LOSS` (14), `FAULT_US_NO_ECHO` (15), `FAULT_BASE_MASK`, `FAULT_BOOT_VALUE`, and the `fault_set/clear/mask/has` prototypes; `app_types.h` includes it
    - Change the status `faults` member to `volatile uint16_t fault_mask` initialised to `FAULT_BOOT_VALUE`; implement `fault_set`/`fault_clear` in `os_glue.c` inside `crit_dsp_enter/exit` and `fault_mask()` as one 16-bit load
    - _Requirements: 3.1, 3.2, 3.4, 3.5_

  - [~] 5.2 Replace duplicate fault variables and update the call sites
    - `tsk_safety.c`: auto-recovery test `(fault_mask() & FAULT_BASE_MASK) == FAULT_CONFIG`; `try_recover()` clears `FAULT_BASE_MASK & ~FAULT_IMU_DEAD`
    - `tsk_imu.c`: remove `b_dead`, read `fault_has(FAULT_IMU_DEAD)`; keep the existing set/clear calls at the same places
    - `app_net.c`: delete `g_wifi_up`/`g_mqtt_up`; `manage_wifi()` and the MQTT callbacks set or clear `FAULT_WIFI_LOSS`/`FAULT_MQTT_LOSS` when the condition changes; `net_wifi_up()`/`net_mqtt_up()` return the inverse of the bits
    - `tsk_obstacle.c` `ping()`: set `FAULT_US_NO_ECHO` when `hal_us_read_cm()` < 0, else clear it, before the reading is used
    - Console status line prints only the 13 baseline fault names; the JSON `faults` value prints the 16-bit mask; leave the encoder-loss check in `tsk_motion.c` unchanged
    - _Requirements: 3.3, 3.6, 3.7, 3.8, 3.9_

  - [~] 5.3 Write property test for the Fault_Mask model
    - **Property 7: The Fault_Mask follows the bit-set model**
    - Use `crit_stub.c` to check that every update ran inside one dispatch-disabled section
    - **Validates: Requirements 3.5, 3.9**

  - [~] 5.4 Write unit tests for Fault_Mask edge cases
    - Boot value has bits 13 and 14 set and bit 15 clear; recovery mask keeps the link and no-echo bits; no-echo set/clear rule for results −1, 0 and positive
    - _Requirements: 3.2, 3.8, 3.9_

- [ ] 6. Step 3: Status_Snapshot
  - [~] 6.1 Create `app_program/status.h`
    - `status_snapshot_t` with the D2 Storage_Units (`mot_a`..`mot_e`, `line_a`, `line_b`, `run`, `imu_a`, `imu_b`, `obs_a`..`obs_c`, `imu_c`, `vbat`, `fault_mask`, `sys`), shift/width constants per field, the setter and getter prototypes of C4, and `st_view_t`
    - `_Static_assert`s: `sizeof(status_snapshot_t) == 60`, each `shift + width <= unit width`, `LINE_LOST < 8`, `MEV_COUNT <= 8`, `SYS_STATE_COUNT <= 8`, `STOP_REASON_COUNT <= 16`, each with a message naming the field (for example `"status.line_a.line_state too narrow"`)
    - Plain C only (standard headers, `bitpack.h`, `crit.h`, `fault_mask.h`, `app_types.h`)
    - _Requirements: 1.1, 1.7, 2.1, 2.2, 2.4, 2.13, 5.1, 5.2, 5.3, 6.1, 6.3, 6.8, 7.1, 7.2, 7.4, 9.5_

  - [~] 6.2 Implement the Status_Snapshot accessors in `app_status.c`
    - Single-writer setters compose the unit in a local and do one aligned store; `st_run_*`, `st_put_imu`, `st_set_mev`, `st_put_hump_peak`, `st_hump_end`, `st_hump_max_reset` do the read-modify-write between `crit_dsp_enter()` and `crit_dsp_exit()`
    - Quantize with `bp_quant`, call `sat_note(n)` with the clamped/NaN count; `st_put_obst`/`st_put_scan` store 8191 for a negative reading; `st_put_side` rejects anything except 0, `'L'`, `'R'`; counters use `bp_sat_inc` at 63/15/15
    - Getters do one aligned load per unit; `st_read()` copies every unit with one load each
    - Add `app_status.c` to the host library
    - _Requirements: 1.4, 1.8, 1.9, 1.10, 6.2, 6.9, 7.5, 7.6, 9.1, 9.2, 9.3, 9.4_

  - [~] 6.3 Convert every `g_status` writer and reader
    - Replace direct `g_status` field writes in `tsk_motion.c`, `tsk_line.c`, `tsk_imu.c`, `tsk_obstacle.c`, `tsk_nav.c`, `tsk_safety.c`, `tsk_telemetry.c` and `os_glue.c` with the C4 setters; line sensors store −1 when LINE_DO_BLACK_HIGH < 0
    - Readers: the JSON frame builder in `tsk_telemetry.c` reads through `st_read()`; safety recheck uses `st_obst_cm()`; `complete_run()` uses the counter getters; `BC_DBG` text uses `st_thr_black()`/`st_thr_white()`
    - Remove `robot_status_t` from `app_types.h`
    - _Requirements: 5.1, 6.1, 6.8, 7.1, 7.2, 7.4, 9.1, 9.3, 9.4, 19.7_

  - [~] 6.4 Write property test for field independence
    - **Property 4: Packed accessors round-trip and keep fields independent**
    - At least 1,000 sequences of 1 to 16 writes per Status_Snapshot unit and per synthetic `bitpack` unit, values including 0 and 2^Required_Bits − 1
    - **Validates: Requirements 2.7, 2.11, 5.2, 5.7, 15.7, 18.5**

  - [~] 6.5 Write property test for invalid discrete writes
    - **Property 5: Invalid discrete writes have no effect**
    - Cover `bp_set_checked` enum fields, `st_put_side` and `msg_nav_encode`
    - **Validates: Requirements 2.12, 6.3, 8.8**

  - [~] 6.6 Write property test for saturating counters
    - **Property 6: Saturating counters hold at their maximum**
    - Cover `sat_note`, `st_run_barcode`, `st_run_hump_inc`, `st_run_obstacle_inc`
    - **Validates: Requirements 1.10, 6.9, 7.3**

  - [~] 6.7 Write property test for line sensor codes
    - **Property 8: Line sensor codes follow the polarity rule**
    - **Validates: Requirements 5.1**

  - [~] 6.8 Write the compile-fail test for narrow enums
    - `tests/host/test_compile_fail.sh`: a translation unit that enlarges an enum past its field width must fail to compile with the field's name in the message
    - _Requirements: 2.13_

- [~] 7. Checkpoint: Steps 2 and 3 complete
  - Ensure `tools/evidence/build_check.sh` passes and all tests pass, ask the user if questions arise. The user runs the bench check (boot to IDLE, START, line following, E-stop, recovery) and confirms the JSON telemetry values are unchanged apart from the three new fault bits.

- [ ] 8. Step 4: Shared scalars, odometry, motion done word and packed mailbox bodies
  - [~] 8.1 Pack the system word and the stop reason
    - `sys` unit holds `state` E3 and `stop_reason` E4; `sys_set_state`/`sys_state` keep their signatures; add `stop_reason_t`, `stop_reason_set`, `stop_reason`, `stop_reason_text` (the exact Baseline_Build strings) in `os_glue.h/.c`
    - `enter_estop()`/`enter_error()` in `tsk_safety.c` take a `stop_reason_t`; `console_status()` prints `stop_reason_text(stop_reason())`
    - _Requirements: 2.1, 2.9, 2.10, 9.2_

  - [~] 8.2 Pack the buzzer, heartbeats, re-arm counter, command ids and E-stop flags
    - `g_buzz` 8-bit with `buzzer_request` (rejects > `BUZZ_WARN`) and `buzzer_pattern_take` in one Critical_Section; `g_hb[]` and `g_reacq_seq` as `volatile uint8_t` modular counters with unchanged safety check semantics; `g_next_id` 16-bit
    - `g_net_flags` (estop_pending, estop_ack_due, config_due), `g_estop_rx_us` written together in one dispatch-disabled section, and `g_estop_pwm_off_us` (U16 saturating) written by TSK_SAFETY_MON
    - _Requirements: 2.5, 9.1, 9.2, 9.3, 9.4, 9.7, 12.6_

  - [~] 8.3 Pack the motion done word and the odometry record
    - `g_done` 32-bit (`done_id` [0..15], `done_result` [16..17]) written by `finish()` with one store and read by `motion_wait()` with one load in `tsk_motion.c`/`os_glue.c`
    - `odo_rec_t` (D3.4) with `odometry_put`/`odometry_get` copying all six words in one dispatch-disabled section; `odometry_t` drops `enc_r`/`enc_l` and replaces `dist_r_mm`/`dist_l_mm` with `dist_mm`; update the readers in `tsk_line.c`, `tsk_imu.c`, `tsk_obstacle.c`
    - _Requirements: 4.5, 4.6, 9.7_

  - [~] 8.4 Switch the motion, nav and obstacle mailboxes to packed bodies
    - `app_msg_body_t` union holds `motion_msg_t`, `nav_msg_t`, `obst_msg_t` and (until Step 8) the existing `tlm_event_t`; renumber `cmd_src_t` from 0
    - `post_motion`, `post_nav`, `post_obstacle` keep their signatures and encode inside, passing clamp counts to `sat_note`; `post_nav` returns false for a bad letter; receivers in `tsk_motion.c` and `tsk_nav.c` decode before use; `g_prof` in `tsk_obstacle.c` stays unchanged
    - Keep the four mailboxes, the 40-block pool, `TMO_POL` allocation and release-after-copy
    - _Requirements: 4.1, 4.2, 4.3, 4.4, 4.9, 6.4, 6.5, 6.6, 8.1, 8.5, 8.7, 8.8_

  - [~] 8.5 Add the on-target concurrency self-test
    - Under `#if APP_SELFTEST` in `os_glue.c`: before the application tasks start, run two tasks at different priorities (the higher one woken every 1 ms), each updating a different Packed_Field of one shared unit 10,000 times through the dispatch-disabled accessor; print `[selftest] PASS/FAIL` with the final field values, then continue the normal boot
    - _Requirements: 9.8_

  - [~] 8.6 Write property test for motion targets
    - **Property 11: Motion targets match the Baseline_Build**
    - Differential against `libbase.so` with the `robot_config.h` calibration
    - **Validates: Requirements 4.10**

  - [~] 8.7 Write property test for critical-section coverage
    - **Property 18: Critical sections are balanced and cover every shared update**
    - Cover the shared Status_Snapshot units, `g_buzz`, `g_net_flags`, `odometry_put`/`odometry_get` with `crit_stub.c`
    - **Validates: Requirements 9.2, 9.7**

  - [~] 8.8 Write unit tests for stop reasons and pool exhaustion
    - `stop_reason_text` returns the Baseline_Build string for each of the 10 codes; `test_pool_empty.py`: with `msg_alloc` returning NULL every post function returns false and sends nothing
    - _Requirements: 2.10, 8.7_

- [~] 9. Checkpoint: Step 4 complete
  - Ensure `tools/evidence/build_check.sh` passes and all tests pass, ask the user if questions arise. The user runs the bench check, including a turn and an obstacle bypass.

- [ ] 10. Step 5: Controller contexts, Parameter_Store and barcode map
  - [~] 10.1 Pack the line controller contexts and `tsk_line.c` flags
    - `line_ctx_t.f` (state, side, `b_last_curve`, `b_both_prev`) and `search_ctx_t.f` (dir, `b_second`) in `line_logic.h` with inline get/set functions that reject out-of-range values; update `line_logic.c`
    - `g_line_flags` in `tsk_line.c` replaces `g_searching`, `g_det_on` and the path-init flag
    - _Requirements: 2.2, 2.7, 2.8, 2.12_

  - [~] 10.2 Pack the IMU contexts
    - `hump_ctx_t.f` (state E2, `n_up`/`n_down`/`n_level` U3 saturating at 5 with `bp_sat_inc`) and `mev_ctx_t.f` in `imu_logic.h/.c`; `_Static_assert(max(HUMP_START_SAMPLES, HUMP_END_SAMPLES) <= 7)`
    - _Requirements: 2.3, 2.4, 7.3_

  - [~] 10.3 Pack the obstacle and motion contexts
    - `us_filter_t.f` and `bypass_plan_t.f` (side S2) in `obstacle_logic.h/.c`; `g_obs_flags` and `g_why` (abort reason E4) in `tsk_obstacle.c`; `primitive_t.f` (`b_active`, type E3) in `tsk_motion.c`; continuous members stay float32 (D16)
    - _Requirements: 2.6, 2.7, 2.8, 6.7_

  - [~] 10.4 Write property test for hump detection equivalence
    - **Property 15: Hump detection matches the Baseline_Build**
    - **Validates: Requirements 7.7**

  - [~] 10.5 Write property test for obstacle planning equivalence
    - **Property 14: Obstacle planning matches the Baseline_Build**
    - **Validates: Requirements 6.7**

  - [~] 10.6 Write property test for barcode decoding equivalence
    - **Property 13: Barcode decoding matches the Baseline_Build**
    - Generate each of the 43 Code 39 data characters forward and reversed as edge sequences (BARCODE_DETECT = 0) and raw sample sequences (BARCODE_DETECT = 1), plus random non-barcode sequences
    - **Validates: Requirements 5.9**

  - [~] 10.7 Extend the field independence test to the controller contexts
    - Add `line_ctx_t.f`, `search_ctx_t.f`, `hump_ctx_t.f`, `mev_ctx_t.f`, `us_filter_t.f`, `bypass_plan_t.f` to the Property 4 test
    - _Requirements: 2.7, 2.8, 2.11, 18.5_

  - [~] 10.8 Implement the Parameter_Store in `app_params.h/.c`
    - `param_id_t` in `PARAMS[]` order (55 IDs), `PRM_ID_UNKNOWN`; the 16 discrete parameters move into `g_params.bits` (D15), the 39 float members keep their names
    - `PARAMS[]` descriptors gain kind, shift, width, encoding; `params_get`, `prm_flag`, `prm_tri`, `prm_int`, `params_set_id` (0 ok, −1 unknown, −2 NaN/inf, −3 out of range or not whole, one composed 32-bit store), `params_put_wire`, `params_table_crc` (CRC-16/CCITT-FALSE over the NAMEs each followed by `'\n'`, computed in `params_init`)
    - Keep `params_set` (text) and the JSON SET/GET:PARAMS replies working for now
    - _Requirements: 14.5, 15.1, 15.2, 15.3, 15.4, 15.5, 15.6, 15.7_

  - [~] 10.9 Update the discrete parameter reads at their call sites
    - Replace reads such as `g_params.enc_r_reversed > 0.5f`, `g_params.line_do_black_high >= 0.0f`, `(uint32_t)(g_params.barcode_levels + 0.5f)` with `prm_flag`, `prm_tri`, `prm_int` in every `tsk_*.c`, `hal_board.c` and pure-logic caller
    - _Requirements: 15.5_

  - [~] 10.10 Write property test for the Parameter_Store value model
    - **Property 22: The Parameter_Store matches the Baseline_Build value model**
    - Differential against a Baseline_Build float table from `libbase.so`
    - **Validates: Requirements 13.6, 14.4, 15.5, 15.6**

  - [~] 10.11 Pack the barcode map into `g_map_w[3]`
    - `nav_map.c/.h`: letter `i` in word `i / 10`, bits `[3·(i mod 10) .. +2]`, single-writer composed stores; replaces `uint8_t g_map[26]`; `nav_map_describe()` output unchanged
    - _Requirements: 2.6, 9.1, 9.3, 9.4_

  - [~] 10.12 Write unit tests for the packed barcode map
    - Every letter/action pair round-trips; `nav_map_describe()` text equals the Baseline_Build text for the same map
    - _Requirements: 2.6, 2.11_

- [~] 11. Checkpoint: Step 5 complete
  - Ensure `tools/evidence/build_check.sh` passes and all tests pass, including Properties 13, 14, 15 and 22 against the Baseline_Build sources, ask the user if questions arise.

- [ ] 12. Step 6: ISR_Shared_Data
  - [~] 12.1 Pack the barcode edge ring
    - `hal_bc_edge_t` becomes `{ uint32_t odo; bool b_black; }`; `g_bc_ring[64]` stores one `uint32_t` per edge (`odo` mod 2^31 [0..30], `black` [31]); remove `t_ms` and its 64-bit division from the 1 kHz ISR
    - `g_bc_head`/`g_bc_tail` 8-bit, masked modular arithmetic; full ring drops the new entry and increments `g_bc_overruns`
    - Put the ring index and entry pack/unpack helpers in a plain header so the host tests use the same code; update the consumer in `tsk_line.c`
    - _Requirements: 5.4, 10.4, 10.8_

  - [~] 12.2 Pack the raw sample ring, barcode flags and thresholds
    - `g_raw_adc[256]` as `uint16_t` (U12), `g_raw_head`/`g_raw_tail` 16-bit; `g_bc_flags` 8-bit (`enabled`, `black_high`, `black`, `raw_enabled`) with the task read-modify-writes in `hal_barcode_config`/`hal_barcode_raw` inside `crit_irq_enter/exit`; `g_bc_thr` 32-bit stored once while `enabled = 0`
    - _Requirements: 5.5, 5.6, 5.7, 10.1, 10.3, 10.4, 10.8_

  - [~] 12.3 Pack the ADC range, echo timing and battery reading
    - `g_adc_last` 16-bit; `g_adc_rng` (`min`, `max`, `have`) with `hal_ir_adc_range` read-and-reset inside `crit_irq`, returning the last value as min and max when `have = 0`
    - `g_echo` (`echo_us` U15, `done`, `rise_seen`) and `g_echo_rise_us`; `hal_us_arm` stores inside `crit_irq`; `hal_us_read_cm` reads with one load and returns −1 for no echo or > 25,000 µs
    - `g_vbat_mv` 16-bit and `g_vbat_div` 8-bit replacing `g_sample_n`
    - _Requirements: 5.8, 6.1, 6.2, 10.1, 10.5_

  - [~] 12.4 Pack the remaining driver flags and keep the own-unit counters
    - `g_enc_rev` 8-bit written only by `hal_enc_set_reversed` with one store; `g_motor_flags` replaces `g_motor_killed`; `g_servo_rng` 32-bit; `g_imu_cfg` 16-bit (`hal_imu.c`)
    - Keep `g_enc[]`, `g_odo`, `g_enc_err`, `g_bc_overruns` in their own 32-bit units with a comment giving the reason
    - _Requirements: 4.7, 10.3, 10.5, 10.6_

  - [~] 12.5 Write property test for the ring FIFO model
    - **Property 12: Rings behave like a FIFO model**
    - Cover the barcode edge ring, the raw sample ring and the command slot index arithmetic (drop-new rule and overrun count)
    - **Validates: Requirements 5.4, 5.5, 10.4, 10.8, 11.8, 11.9**

  - [~] 12.6 Write unit tests for the ADC range reset
    - `test_adc_range.py`: host model of `g_adc_rng` returns the last value as min and max when no sample was taken, and never stores a value above 4095
    - _Requirements: 5.8_

- [~] 13. Checkpoint: Step 6 complete
  - Ensure `tools/evidence/build_check.sh` passes and all tests pass, ask the user if questions arise. The user compares the `APP_PERF=1` ISR probe values with the Step 0 numbers before continuing.

- [ ] 14. Step 7: Laptop_Codec and laptop tools (with `--legacy-json` on by default)
  - [~] 14.1 Implement the Python field layer in `MQTT/is02_codec.py`
    - `FORMAT_VERSION`, the event, reason, command, stop/abort reason and Parameter_ID tables (NAME, kind, min, max), frame lengths, the CRC-16/CCITT-FALSE parameter table version
    - `decode_fields(kind, payload)` and `encode_fields(kind, record)` for all seven payload kinds over a MSB-first bit reader/writer; version and length checks raise `CodecError`
    - _Requirements: 14.5, 16.6, 17.1, 17.2, 17.7_

  - [~] 14.2 Implement the Python presentation layer
    - `decode(topic, payload)` returns the Baseline_Build JSON shape per topic (C14), with baseline units, enum names, sentinels as `-1.0`, `""`, `[0,0]`, per-event detail text formatters (D10) and ack extras; unknown codes show the number and are logged; bad version/length are logged with topic, version, length and dropped
    - `encode_command_text(text)` for every Baseline_Build command text, `encode_map(pairs)`, `encode_config(lines)` with `CodecError` naming the line or argument and NAME; `legacy_json=True` parses payloads starting with `{` as JSON
    - _Requirements: 8.3, 12.2, 17.1, 17.6, 17.7, 17.9, 17.10_

  - [~] 14.3 Implement the JavaScript field layer in `MQTT/is02_codec.js`
    - UMD wrapper (`<script>` in the browser from `file://`, `require()` in Node); same tables, CRC, `decodeFields`/`encodeFields`, version and length checks throwing `Error`
    - _Requirements: 14.5, 16.6, 17.1, 17.2, 17.7_

  - [~] 14.4 Implement the JavaScript presentation layer
    - `IS02.decode(topic, bytes)`, `IS02.encodeCommandText`, `IS02.encodeMap`, `IS02.encodeConfig`, legacy JSON mode, unknown-code and bad-payload handling matching the Python codec
    - _Requirements: 8.3, 12.2, 17.1, 17.3, 17.7, 17.9, 17.10_

  - [~] 14.5 Add the shared test vectors and the C vector harness
    - `tests/host/py/make_vectors.py` generates `tests/host/vectors/vectors.json` from a recorded seed (field records per payload kind and command code)
    - `tests/host/tools/wire_vectors.c` built from `wire.c`: encodes and decodes every vector and writes the results as JSON
    - _Requirements: 18.6_

  - [~] 14.6 Switch `telemetry_monitor.py` to the codec
    - `telemetry()`, `status()`, `ack()` take decoded dicts; keep `CSV_COLS` and the L1 to L6 thresholds; 15-bit modular sequence gaps; 24-bit modular time for the L1 rate; L2 counts codec-rejected payloads; acks matched by command code plus ping number or letter; commands sent with `encode_command_text`; parameter-table mismatch warning from the heartbeat; `--legacy-json` (default on for this step)
    - _Requirements: 14.8, 17.4, 17.5, 17.8_

  - [~] 14.7 Switch `is02_tool.py`, `robot_params.py` and `robot_map.py` to the codec
    - `is02_tool.py`: `show()` prints decoded dicts; `cmd`, `set`, `map`, `config` encode before publishing with the baseline topics, retain flags and ack waits (4 s, 3 s for `set`); `log` writes `pc_time, topic, len, hex, decoded_json`; config publish refused while the parameter table version differs
    - `robot_params.py`: blank and `#` lines ignored, NAME → Parameter_ID, validation errors print line number and NAME and publish nothing; `--clear` publishes the empty retained payload
    - `robot_map.py`: builds the Map_Frame, waits for the MAP ack; `--clear` publishes the empty retained payload and the MAP:CLEAR command
    - Add `--legacy-json` to each tool (default on for this step)
    - _Requirements: 14.8, 17.5, 17.6, 17.8, 17.9_

  - [~] 14.8 Switch `dashboard.html` to the codec
    - Load `is02_codec.js`; `MiniMqtt` passes the raw `Uint8Array`; `publish()` accepts bytes or text; `onMessage` uses `IS02.decode`; `send(cmd)` publishes `IS02.encodeCommandText(cmd)`; map buttons publish `IS02.encodeMap(...)` retained; add `WIFI_LOSS`, `MQTT_LOSS`, `US_NO_ECHO` fault names; parameter-table mismatch banner; legacy JSON checkbox (default on); labels and units unchanged
    - _Requirements: 14.8, 17.3, 17.5, 17.8_

  - [~] 14.9 Write `MQTT/README.md`
    - How the tools use `is02_codec.py`/`is02_codec.js`, the `--legacy-json` option, and the dependencies
    - _Requirements: 17.5_

  - [~] 14.10 Write property test for the parameter table version
    - **Property 24: The parameter table version detects table edits**
    - C (`params_table_crc` via `ctypes`), Python and JavaScript agree; any single add, remove or reorder changes the version
    - **Validates: Requirements 14.5**

  - [~] 14.11 Write property test for robot-to-laptop round trips
    - **Property 25: Robot-to-laptop frames round-trip through both Laptop_Codecs**
    - At least 1,000 Telemetry, Heartbeat and Ack sets and at least 100 sets per Event_Code; C encode, Python and JavaScript decode
    - **Validates: Requirements 11.1, 17.1, 18.1**

  - [~] 14.12 Write property test for laptop-to-robot round trips
    - **Property 26: Laptop-to-robot frames round-trip into the Wire_Codec**
    - At least 1,000 sets per command code, Map_Frame and Config_Frame; Python and JavaScript encode, C decode
    - **Validates: Requirements 13.1, 13.10, 14.1, 14.3, 17.2, 18.2**

  - [~] 14.13 Write property test for decode-then-encode identity
    - **Property 27: Decode then encode is the identity on bytes**
    - At least 1,000 valid payloads per type for each of the C, Python and JavaScript codecs
    - **Validates: Requirements 18.3**

  - [~] 14.14 Write property test for cross-codec agreement
    - **Property 28: The three codecs agree on the shared vectors**
    - **Validates: Requirements 18.6**

  - [~] 14.15 Write property test for tool input parsing
    - **Property 29: Tool inputs parse to Parameter_IDs or report the bad line**
    - **Validates: Requirements 17.6, 17.9**

  - [~] 14.16 Write property test for unknown codes
    - **Property 30: Unknown codes are shown numerically**
    - **Validates: Requirements 17.10**

  - [~] 14.17 Write unit tests for the decoded presentation shape
    - `test_presentation.py` and a matching JS test: decoded telemetry has every Baseline_Build key; sentinel presentation; bad-version, short, long and zero-length robot-to-laptop payloads are rejected without fields
    - _Requirements: 11.2, 11.10, 17.7, 18.7_

  - [~] 14.18 Write unit tests for the laptop tools
    - `test_tools.py` with a mock MQTT client: each tool action publishes the right topic, retain flag and frame; ack timeout prints the Baseline_Build message; monitor CSV header equals `CSV_COLS`; L1 to L6 thresholds unchanged; mismatch warning appears and clears; config publish refused on mismatch
    - _Requirements: 14.8, 17.4, 17.5, 17.8_

- [~] 15. Checkpoint: Step 7 complete
  - Ensure all tests pass, including Properties 24 to 30 with the C, Python and JavaScript codecs, ask the user if questions arise. The user confirms the tools still work against the Step 6 JSON firmware with `--legacy-json`.

- [ ] 16. Step 8: Robot-to-laptop binary payloads (events, acks, heartbeat, telemetry)
  - [~] 16.1 Publish events as Event_Frames (Step 8a, part 1)
    - Implement `post_evt` in `os_glue.c` (pool + `g_mbx_tlm`, `TMO_POL`); replace `tlm_event_t` with `evt_msg_t` in `app_msg_body_t`; set `MSG_BLK_WORDS` to 8 (32 bytes) with `_Static_assert`s on body sizes and block size; remove `post_event`, `TLM_EVENT_NAME_LEN`, `TLM_EVENT_DETAIL_LEN`
    - Add the byte-length `net_post(topic, p, len)` in `app_net.h/.c`; TSK_TELEMETRY encodes each received `evt_msg_t` with `wire_enc_event` and publishes it on `is02/event`
    - _Requirements: 8.2, 8.4, 8.5, 8.7, 12.2_

  - [~] 16.2 Replace every `post_event` call with typed builders (Step 8a, part 2)
    - In `tsk_safety.c`, `tsk_motion.c`, `tsk_line.c`, `tsk_obstacle.c`, `tsk_imu.c`, `tsk_nav.c`, `tsk_telemetry.c`: call the D10 builder with the typed values the text was built from; remove text buffers used only for events; keep `log_line()` text unchanged
    - `FINE_SCAN` posts at most 11 readings per event; `BC_DBG` widths post at most 6 parts of 16
    - _Requirements: 8.3, 8.9, 12.2, 12.3, 12.8_

  - [~] 16.3 Write property test for variable-length detail splitting
    - **Property 17: Variable-length details split without loss**
    - **Validates: Requirements 8.9, 12.3, 12.8**

  - [~] 16.4 Write property test for event detail text
    - **Property 16: Event detail text can be rebuilt**
    - Compare the Python and JavaScript rebuilt texts with a reference formatter using `jb_fixed` rounding and golden texts in `tests/host/vectors/`
    - **Validates: Requirements 8.3, 12.2**

  - [~] 16.5 Publish acks as Ack_Frames (Step 8b, part 1)
    - In `tsk_telemetry.c`, build every ack with `wire_enc_ack` and its Reason_Code (D11), including the E-stop ack with `g_estop_pwm_off_us`, GET:PARAMS in numbered parts of 12 IDs using `params_put_wire`, CONFIG with applied count and rejected IDs; commands still arrive as text in this step
    - _Requirements: 12.4, 12.5, 12.6, 13.8, 19.10_

  - [~] 16.6 Publish the heartbeat as a Heartbeat_Frame (Step 8b, part 2)
    - Fill `wire_hb_t` (sequence M14, time M24, state, RSSI, Frame_Ring fill, uptime U17 saturating, reconnects, dropped, `sat_count()`, Fault_Mask, parameter count and CRC, broker IPv4) and publish on `is02/status`
    - Add `net_broker_ipv4()`, `net_reconnects()`, `net_frames_dropped()` (16-bit saturating) to `app_net.h/.c`
    - _Requirements: 12.1, 12.7, 14.5_

  - [~] 16.7 Write property test for GET:PARAMS parts
    - **Property 20: GET:PARAMS parts cover every parameter once**
    - **Validates: Requirements 12.5**

  - [~] 16.8 Convert the Frame_Ring and message slots to byte payloads (Step 8c, part 1)
    - `app_net.h/.c`: `net_post_frame(p, len)` with `len == WIRE_TLM_LEN`, 64 slots of exactly 54 bytes, overwrite oldest and saturating `g_dropped` when full, interrupt-masked index and copy; 8 message slots of 64 bytes with the 16-bit `hdr` (topic, len); oldest-first replay at up to 4 frames per poll; discovery beacon and back-off unchanged
    - _Requirements: 11.6, 11.8, 11.9, 19.4, 19.9_

  - [~] 16.9 Publish telemetry as the Telemetry_Frame (Step 8c, part 2)
    - In `tsk_telemetry.c`, fill `wire_tlm_t` from `st_read()` codes plus sequence (M15), time (M24), ADC range, RSSI and buffer fill; encode with `wire_enc_telemetry` and post with `net_post_frame`; keep the 500 ms period with skipped missed periods
    - Move the `PERF_TLM_ENCODE` probe and `tlm_bench` to the binary encoder; remove the JSON builders for telemetry, status, event and ack (keep `jfmt` for console lines and legacy text)
    - _Requirements: 11.1, 11.2, 11.3, 11.4, 11.5, 11.7, 11.10, 19.8, 20.4, 20.5_

  - [~] 16.10 Extend the ring FIFO test to the Frame_Ring and message slots
    - Add the overwrite-oldest rule, the exact +1 dropped count and byte-identical replay to the Property 12 test
    - _Requirements: 11.8, 11.9_

  - [~] 16.11 Extend the saturating counter test to the network counters
    - Add the reconnect and dropped-frame counts and the heartbeat uptime to the Property 6 test
    - _Requirements: 12.1_

- [~] 17. Checkpoint: Step 8 complete
  - Ensure `tools/evidence/build_check.sh` passes and all tests pass, ask the user if questions arise. The user checks the dashboard and monitor (with `--legacy-json`) against the binary robot-to-laptop payloads.

- [ ] 18. Step 9: Laptop-to-robot binary payloads and the legacy text switch
  - [~] 18.1 Rework the incoming payload path and the ESTOP fast path
    - `app_net.c` `on_in_data()`: no trimming or NUL reliance for binary; trim only on the legacy text path; zero-length payloads on `is02/map`/`is02/config` ignored; `IN_MAX` 264 bytes (1024 with `APP_LEGACY_TEXT_CMD=1`); call `app_net_command/map/config(p, len, ...)`
    - `app_net_command` in `tsk_telemetry.c`: a valid 1-byte ESTOP frame (or, with the legacy option, text starting `ESTOP`) sets `estop_pending` and `g_estop_rx_us` in one dispatch-disabled section, sets `EVF_REMOTE_ESTOP` and wakes TSK_SAFETY_MON; other payloads go to the 8 × 7-byte command slots (SPSC indices)
    - _Requirements: 10.4, 13.3, 13.4, 13.7, 13.9_

  - [~] 18.2 Handle Command_Frames in TSK_TELEMETRY
    - Decode with `wire_dec_command`, map each `wire_status_t` to its Reason_Code (`UNPARSED` = 15 for bad version, code or length; BARCODE letter 26..31 → Reason_Code 9); perform the Baseline_Build action for each command and state; SET through `params_set_id`; GET:PARAMS in parts
    - With `APP_LEGACY_TEXT_CMD=1` route printable text starting 'A'..'Z' to the existing text parser and reply with Ack_Frames; with 0, text reads as an unknown version
    - _Requirements: 13.1, 13.2, 13.5, 13.6, 13.7, 13.8, 13.9_

  - [~] 18.3 Handle Map_Frames and Config_Frames
    - `is02/map` and MAP commands: `wire_dec_map`, reject the whole frame on any cause with its Reason_Code, otherwise replace the whole map (all present bits 0 clears it)
    - `is02/config`: `wire_dec_config`, reject the whole frame on bad version, count or length with applied 0; otherwise apply each valid pair in order through `params_set_id` and ack the applied count and rejected IDs; `g_config` 264 bytes (1024 with the legacy option)
    - _Requirements: 14.1, 14.2, 14.3, 14.4, 14.6, 14.7_

  - [~] 18.4 Make the tools publish binary by default
    - Switch `--legacy-json` (and the dashboard checkbox) default to off in all five tools; keep the option for Baseline_Build comparison runs
    - _Requirements: 17.5_

  - [~] 18.5 Write property test for malformed payload rejection
    - **Property 21: Malformed payloads are rejected without side effects**
    - Check system state, Fault_Mask, Parameter_Store and map are unchanged after each rejected frame
    - **Validates: Requirements 13.5, 13.9, 14.2, 14.7, 17.7, 18.7**

  - [~] 18.6 Write property test for whole-map replacement
    - **Property 23: Accepting a Map_Frame replaces the whole map**
    - **Validates: Requirements 14.6**

  - [~] 18.7 Write unit tests for the ESTOP classifier and legacy text commands
    - `test_estop_classify.py`: only a valid 1-byte ESTOP frame (and `ESTOP` text with the legacy option) takes the fast path
    - `test_legacy_text.py` built with `-DAPP_LEGACY_TEXT_CMD=1`: each Baseline_Build command text gives the same command code, arguments and Reason_Code as the Baseline_Build ack text
    - _Requirements: 13.3, 13.7_

  - [~] 18.8 Pack the barcode decoder context flags (optional Step 9b)
    - Optional per design D5: pack the `bool` members and `bar[96]` of `bc_ctx_t`, `det_t`, `adt_t`, `bcm_t` into one flag word plus a 96-bit bitmap per context; keep only if Property 13 still passes, otherwise revert
    - _Requirements: 2.8, 5.9_

- [~] 19. Checkpoint: Step 9 complete
  - Ensure `tools/evidence/build_check.sh` passes with default options and with `APP_LEGACY_TEXT_CMD=1`, and all tests pass, ask the user if questions arise. The user runs the bench check with binary commands from the dashboard and `is02_tool.py`, including a remote ESTOP.

- [ ] 20. Step 10: Documentation, evidence tooling and cleanup
  - [~] 20.1 Add the `layout_dump` host tool
    - `tests/host/tools/layout_dump.c`: prints every Storage_Unit, field offset and width, packed type size and protection (from constants in the firmware headers) as JSON
    - _Requirements: 21.1, 21.2, 21.6, 21.7_

  - [~] 20.2 Write `docs/data_representation.md`
    - One section per category (motion control and encoder data; line and barcode sensing; obstacle detection; hump estimation; vehicle and controller states; Task_Shared_Data; ISR_Shared_Data; Inter_Task_Messages; MQTT telemetry and command payloads), one row per Application_Field with Valid_Range and sentinels, Resolution, Required_Bits, Baseline_Build type and bits, Revised_Build representation, Storage_Unit name and width, offset and width
    - Writers and protection per shared unit; consistent multi-unit sets; packed type sizes; Private_Control_Variables with task, type and reason; all 55 parameters with representation and float reasons; own-unit ISR fields with reasons; per-run counter maxima; the Code 39 index order
    - A fenced `json` block after each table with the same data for the doc test
    - _Requirements: 1.1, 1.3, 1.6, 1.7, 5.3, 6.8, 9.1, 9.7, 10.6, 15.4, 21.1, 21.2, 21.3, 21.5, 21.6, 21.7_

  - [~] 20.3 Write `docs/protocol.md`
    - Current Format_Version and change log; D7 conventions; bit tables for all seven payload types and every command code (length, byte and bit order, field order, offset, width, Valid_Range, Resolution, Required_Bits, sentinels); Event_Code catalog with argument layouts and detail-text rules; Reason_Codes, command codes, stop and abort reasons with baseline texts; the 55 Parameter_IDs with NAME, kind, min, max, store and wire encodings; CRC definition with a worked example; layout fingerprint
    - _Requirements: 11.5, 12.3, 13.2, 16.2, 16.3, 16.4, 16.5_

  - [~] 20.4 Add the README link and finish the host test README
    - Add a "Data representation (Change Requirement 1)" item to `README.md` with relative links to `docs/data_representation.md`, `docs/protocol.md` and `docs/evidence/README.md`
    - Finish `tests/host/README.md` (WSL set-up, `run_all.sh` options, baseline fetch)
    - _Requirements: 21.4_

  - [~] 20.5 Generate the host-side evidence sections
    - Add `run_all.sh --report` that writes the per-property summary (cases, failures, seeds, codec pairs C+Python and C+JavaScript) into the Host_Test_Suite section of `docs/evidence/correctness.md`
    - Fill the Telemetry_Frame budget section of `docs/evidence/message_sizes.md` from `frame_budget.py`; leave the hardware tables for the user with the command that fills each
    - _Requirements: 23.3, 24.1_

  - [~] 20.6 Remove unused JSON helpers
    - Remove `jfmt` JSON helpers only if nothing references them; keep `jfmt` for console lines and the legacy text parser
    - _Requirements: 19.5_

  - [~] 20.7 Write the data representation document test
    - `test_layout_doc.py`: `layout_dump` equals the JSON blocks in `docs/data_representation.md` (every unit, offset, width, size, protection); Required_Bits = ceil(log2 N) for every row; every frame field has a row; the `README.md` relative links resolve
    - _Requirements: 1.1, 1.6, 1.7, 21.1, 21.2, 21.3, 21.4, 21.5, 21.6, 21.7_

  - [~] 20.8 Write the protocol table and fingerprint tests
    - `test_tables.py`: `wire_dump` vs the Python and JavaScript tables and `docs/protocol.md` (unique codes per category, 45 events, 11 commands, 55 Parameter_IDs, reason and stop texts equal the baseline strings, frame lengths); a differing version fails naming the codec
    - `test_fingerprint.py`: the `wire_dump` layout hash must match the fingerprint stored for the current Format_Version
    - _Requirements: 2.10, 8.2, 12.4, 13.2, 16.2, 16.3, 16.4, 16.5, 16.6_

- [~] 21. Final checkpoint
  - Ensure `tools/evidence/build_check.sh` passes, `tests/host/run_all.sh` passes with the summary written, and all tests pass, ask the user if questions arise. The user then performs the hardware measurements in `docs/evidence/` (RAM, message sizes, timing, track runs) with the provided scripts.

## Notes

- Tasks marked with `*` are optional and can be skipped for a faster MVP. Task 18.8 is optional because the design marks Step 9b optional; all other `*` tasks are tests.
- Each task references requirement clauses for traceability; hardware-only criteria (9.8, 10.7, 10.9, 10.10, 13.4, 19.7, 19.10, 20.1 to 20.5, 22, 23.1, 23.2, 24.2 to 24.7) are covered by the instrumentation, scripts and templates the tasks produce, and the user takes the measurements.
- Checkpoints follow the design's rule that every step leaves the firmware buildable with `build_make/robot.mk` and behaving as before; the bench checks on the robot are the user's.
- Property tests validate the 30 correctness properties of the design; unit tests cover specific examples, tables and edge cases.

## Task Dependency Graph

```json
{
  "waves": [
    { "id": 0, "tasks": ["1.1", "1.2", "1.4", "1.6", "1.7"] },
    { "id": 1, "tasks": ["1.3", "1.5", "1.8"] },
    { "id": 2, "tasks": ["1.9", "3.1", "3.6", "3.8"] },
    { "id": 3, "tasks": ["3.2", "3.3", "3.4", "3.5", "3.7", "3.9"] },
    { "id": 4, "tasks": ["3.10", "3.14"] },
    { "id": 5, "tasks": ["3.12", "3.13", "3.15", "3.16"] },
    { "id": 6, "tasks": ["3.11", "5.1"] },
    { "id": 7, "tasks": ["5.2", "5.3"] },
    { "id": 8, "tasks": ["5.4", "6.1"] },
    { "id": 9, "tasks": ["6.2"] },
    { "id": 10, "tasks": ["6.3"] },
    { "id": 11, "tasks": ["6.4", "6.5", "6.6", "6.7", "6.8"] },
    { "id": 12, "tasks": ["8.1"] },
    { "id": 13, "tasks": ["8.2"] },
    { "id": 14, "tasks": ["8.3"] },
    { "id": 15, "tasks": ["8.4"] },
    { "id": 16, "tasks": ["8.5", "8.6", "8.7", "8.8"] },
    { "id": 17, "tasks": ["10.1", "10.2", "10.3", "10.8", "10.11"] },
    { "id": 18, "tasks": ["10.4", "10.5", "10.6", "10.7", "10.9", "10.10", "10.12"] },
    { "id": 19, "tasks": ["12.1"] },
    { "id": 20, "tasks": ["12.2"] },
    { "id": 21, "tasks": ["12.3"] },
    { "id": 22, "tasks": ["12.4", "12.5", "12.6"] },
    { "id": 23, "tasks": ["14.1", "14.3"] },
    { "id": 24, "tasks": ["14.2", "14.4", "14.5"] },
    { "id": 25, "tasks": ["14.6", "14.7", "14.8", "14.9", "14.10", "14.11", "14.12", "14.13", "14.14", "14.15", "14.16", "14.17"] },
    { "id": 26, "tasks": ["14.18"] },
    { "id": 27, "tasks": ["16.1"] },
    { "id": 28, "tasks": ["16.2"] },
    { "id": 29, "tasks": ["16.3", "16.4", "16.5"] },
    { "id": 30, "tasks": ["16.6"] },
    { "id": 31, "tasks": ["16.7", "16.8"] },
    { "id": 32, "tasks": ["16.9"] },
    { "id": 33, "tasks": ["16.10", "16.11"] },
    { "id": 34, "tasks": ["18.1"] },
    { "id": 35, "tasks": ["18.2"] },
    { "id": 36, "tasks": ["18.3"] },
    { "id": 37, "tasks": ["18.4", "18.5", "18.6", "18.7"] },
    { "id": 38, "tasks": ["18.8"] },
    { "id": 39, "tasks": ["20.1", "20.3", "20.6"] },
    { "id": 40, "tasks": ["20.2", "20.8"] },
    { "id": 41, "tasks": ["20.4", "20.5"] },
    { "id": 42, "tasks": ["20.7"] }
  ]
}
```
