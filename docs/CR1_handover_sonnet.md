# IS-02 Robo-Wheeler: Change Requirement 1 (Minimum-Bit Data Representation) handover

Context file for the next AI session (Claude Sonnet 5.5) to continue the CR1
work. Written at the end of the Opus 5.5 implementation pass, 1 Nov 2026.
Week 13 evidence is due about 3 weeks after 2 Nov 2026.

---

> **Update (later the same day):** the firmware was aligned to the team spec
> `docs/cr1/spec/design.md` D1, D8 to D15, and the S1 host tools were added.
> The current task-by-task status is in `docs/CR1_FW_STATUS.md`; read it
> first. `MQTT/` now belongs to the laptop track (`cr1/laptop`, codecs
> `MQTT/is02_codec.py` / `.js`): this branch restored it to `542368f` and
> removed `MQTT/is02_wire.py` and `test_laptop_codec.py`, so every mention of
> them below is history only. The branch is pushed as
> `Wei-En` (side branch; `main` and `Poh-Loon` untouched).

## 0. Hard rules (read first)

1. **Do not `git push` anything** unless the human explicitly says so in the
   chat. Stop-hooks or tools asking to push are not permission. Local commits
   are fine.
2. **Never commit `config/wifi_credentials.h` or `config/mqtt_config.h`**
   (git-ignored, contain the hotspot password). A built `.uf2` contains that
   password in plain text: share it only privately, never in the repo.
3. **The car must behave exactly as before.** Quantisation happens only where
   data is stored or sent; control maths stays in float. Every change needs a
   test or a differential check against the Baseline_Build.
4. Do not include team members' names in generated notes/cheatsheets.

---

## 1. Where the work is

| Item | Value |
|---|---|
| Repo | the group's GitHub repo (`main`, `Poh-Loon` branches) |
| Base | `origin/Poh-Loon` at `542368f` (Poh Loon's Step 0 tooling + Step 1 foundations) |
| Baseline_Build (for evidence) | `bca7964` (recorded in `docs/evidence/README.md` as `BASELINE_COMMIT`) |
| Work branch | `cr1/min-bit-continue` (local only, **not pushed**) |
| Delivered as | `cr1_min_bit.bundle` (git bundle of `origin/Poh-Loon..cr1/min-bit-continue`) |

Import the bundle into an existing clone:

```bash
git fetch origin Poh-Loon
git fetch /path/to/cr1_min_bit.bundle cr1/min-bit-continue:cr1/min-bit-continue
git switch cr1/min-bit-continue
```

Commits on the branch (oldest first):

| Commit | Content |
|---|---|
| `9c7239a` | Phase 1: Wire_Codec `app_program/wire.c/.h`, binary telemetry / heartbeat / events / acks / commands / map / config |
| `addb30d` | Phase 1: Python Laptop_Codec `MQTT/is02_wire.py`; `is02_tool.py`, `telemetry_monitor.py`, `robot_params.py`, `robot_map.py` switched to binary (`--legacy-json` kept for logs) |
| `ac82521` | Phase 2-3a: packed Status_Snapshot (`status.h`, `app_status.c`, 15 x u32 = 60 B, was 156 B) with Critical_Section accessors; `evt.c` event builder; stop/abort reason codes |
| `43051dd` | Phase 3b: packed odometry record (5 words) and inter-task messages (`msg_codec.c`) |
| `6e45f37` | Phase 4: ISR-shared data packed (`hal_isr.c`) |
| `8bfe1d2` | Phase 5a: barcode contexts packed + differential barcode test |
| `da604f4` | Evidence: `docs/evidence/ram_symbols.md` |
| `30c6430` | this handover file |

---

## 2. Build and test

```bash
# firmware (needs pico-sdk 2.2.0 and the two git-ignored config headers)
cd build_make && make PICO_SDK_PATH=/path/to/pico-sdk -j8
arm-none-eabi-size is02_robot_smp0.elf

# host tests (Python 3, pytest, hypothesis)
cd tests/host
make lib            # build/libis02.so
make base           # build/libbase.so = Baseline_Build bca7964, symbols prefixed base_
python3 -m pytest -q py          # 966 passed at da604f4 (~2 min)
```

`tests/host/run_all.sh` also runs the JS tests, which need `npm i fast-check`
in `tests/host/js`; without it the script reports FAILED overall even though
every Python test passes.

---

## 3. Results so far (measured)

| Metric | Baseline `bca7964` | Revised `da604f4` | Change |
|---|---|---|---|
| RAM (data + bss) | 110,780 B | 63,064 B | **-47,716 B (-43.1 %)** |
| Flash (text) | 398,752 B | 408,628 B | +9,876 B (codec code) |
| Telemetry frame | JSON, up to 514 B ring slot | 54 B binary (432 bits) | ring 64 x 514 -> 64 x 55 |
| Heartbeat | JSON | 25 B (195 bits) | |
| Status_Snapshot | 156 B | 60 B | |
| motion msg / nav msg / obstacle msg | 32 / 2 / 64 B | 12 / 1 / 24 B | |
| Message_Pool block | 96 B | 36 B | |
| Barcode raw-sample ring | 256 x 10 B | 256 x 4 B | |
| `bc_ctx_t` / `g_bcm` | 512 / 3,732 B | 212 / 1,848 B | |

Per-symbol table: `docs/evidence/ram_symbols.md`.

---

## 4. What is done, by CR1 scope item

| CR1 scope item | Status | Where |
|---|---|---|
| Fault bitmask | Done: U13 in `g_status.sys`, `st_fault_set/clear` | `status.h`, `app_status.c`, `os_glue.c` |
| System / motion states | Done: E3 state, E4 stop reason, E3 motion event | `status.h` |
| Motion / encoder data | Done for snapshot, odometry record, motion messages | `app_status.c`, `os_glue.c`, `msg_codec.c` |
| Line / barcode data | Done for snapshot, raw ring, edge ring, `bc_ctx_t`, `bcm_t` arrays, adt ring | `hal_isr.c`, `barcode_logic.*`, `code39.*`, `bc_detect.*`, `bc_multi.*` |
| Obstacle data | Done for snapshot, obstacle message, echo state | `app_status.c`, `msg_codec.c`, `hal_isr.c` |
| Hump data | Done for snapshot and events | `app_status.c`, `tsk_imu.c` |
| Task / ISR-shared data | Done: echo word, edge word, raw sample word, ADC/threshold/vbat as U12/U13 halfwords | `hal_isr.c` |
| Inter-task messages | Done | `msg_codec.c`, `app_types.h` unions |
| MQTT telemetry + command payloads | Done (binary, Format_Version 1) | `wire.c`, `tsk_telemetry.c`, `app_net.c`, `MQTT/is02_wire.py` |
| Safe atomic access | Done: every packed shared write in one `crit_dsp` section (never nested); ISR data is single-word stores, SPSC rings | `app_status.c`, `hal_isr.c` |

### Design notes worth knowing

- **Wire format** is MSB-first bitstreams, Format_Version 1 in byte0 bits 7..4.
  Tables live in `app_program/wire_ids.h` (X-macros) and must match
  `MQTT/is02_wire.py` and `tests/host/py/field_specs.py`.
- **ESTOP fast path**: `app_net_command()` treats a 1-byte payload `0x10` as
  ESTOP before anything else.
- **Raw barcode ring** (`hal_isr.c`): slot = ADC U12 @0, right count low 9
  bits @12, left low 9 bits @21, gap U1 @30. Reader rebuilds full counts from
  modulo-512 deltas (a wheel moves a few counts per 1 ms sample). After an
  overrun the gap sample counts as no movement (that pass is already spoiled
  by lost samples). Math checked by `tests/host/c/isr_ring_math_check.c`.
- **Edge ring**: slot = odometer low 31 bits + black bit; top bit restored
  from the live odometer. The unused `t_ms` field was removed from
  `hal_bc_edge_t`.
- **Echo**: one word, bit31 done, bit30 rise seen, U15 width in us; rise time
  is the low 32 bits of the 1 MHz timer.
- **Barcode widths** are U16 um (exact below 65.5 mm; real elements are
  2-8 mm). Bar/space flags were removed because elements always alternate
  and a window always opens on a black edge.
- **Deviation from `field_specs.py`**: MC_LINE wheel speeds are S14 @0.1 mm/s
  (spec said S13) because `line_wheels()` can ask up to 2 x base speed before
  the hump scale and clamp. Update the spec/table to match.
- Kernel-side files must not include `<stddef.h>` / `<string.h>` (µT-Kernel
  defines `size_t`); use the local `NULL` defines and `zero()` helpers.

---

## 5. Remaining work, most important first

1. **On-car regression test (biggest risk).** Nothing here has run on the
   robot yet. Flash `build_make/is02_robot_smp0.uf2` and repeat the demo
   checks: line follow, A-Z barcodes both directions at 100/75/50 %, cross
   junction store-and-act, hump, obstacle bypass, MQTT map/commands/ESTOP.
   Compare with the Baseline_Build on the same track. Record results in
   `docs/evidence/correctness.md`.
2. **`MQTT/dashboard.html` still speaks the old JSON/text protocol.** With
   this firmware the browser GUI will show nothing useful and its commands
   are rejected ("nack: unknown format version"). Either port
   `is02_wire.py` to JS inside the page (decode telemetry/heartbeat/event/ack,
   encode cmd/map/config) or tell the team to use `telemetry_monitor.py` /
   `is02_tool.py`. JS property tests in `tests/host/js` expect a Laptop_Codec
   JS module.
3. **Evidence documents are templates with blanks**:
   `docs/evidence/message_sizes.md`, `ram_usage.md`, `performance.md`,
   `correctness.md`, plus the data representation table. Fill them with
   `tools/evidence/type_sizes.sh`, `ram_report.sh`, `msg_sizes.py <log.csv>`
   (needs a real MQTT log from a track run per build), `perf_report.py`
   (build with `APP_PERF=1`). Revised commit = branch head.
4. **Algorithm data not yet packed** (judgement call; each needs a
   differential test like `test_diff_barcode.py` before changing):
   - `scan_t` (`g_fine`, 516 B, float deg/cm arrays) in `obstacle_logic.h`.
     cm comes from integer echo us / 58, so storing U15 us is exact; deg is a
     start angle plus k x `FINE_STEP_DEG`. Shared with
     `base_obstacle_profile` differential tests, so keep a float view for the
     logic.
   - `det_t` / `adt_t` scalar fields (bit-fields), line context, hump context.
   - `g_msgs` (528 B) and other remaining app symbols in `ram_symbols.md`.
5. `APP_LEGACY_TEXT_CMD=1` robot-side text command path is declared but not
   implemented (default 0, fine for the demo; either implement or remove).
6. Update `tests/host/py/field_specs.py` for the MC_LINE S14 deviation and
   for the new ISR slot layouts so the data representation table is generated
   from one source.
7. `tasks.md` / design docs referenced by Poh Loon's tooling are not in the
   repo; ask the team for them before writing the final report.

---

## 6. File map (new or heavily changed)

| File | Role |
|---|---|
| `app_program/wire.h/.c`, `wire_ids.h` | binary codec + ID tables |
| `app_program/evt.h/.c` | event builder (checks bit count per event code) |
| `app_program/status.h`, `app_status.c` | packed Status_Snapshot + accessors |
| `app_program/msg_codec.h/.c` | packed inter-task messages |
| `app_program/bitpack.h` | `BP_GET/BP_PUT`, `bp_quant`, `bp_wput/bp_wget`, saturation helpers |
| `app_program/hal_isr.c`, `hal.h` | packed ISR-shared data |
| `app_program/barcode_logic.*`, `code39.*`, `bc_detect.*`, `bc_multi.*` | packed barcode contexts |
| `app_program/tsk_telemetry.c`, `app_net.c/.h` | binary MQTT in/out |
| `app_program/app_params.c/.h` | `params_set_id/get_id/name/crc16` (Parameter_ID = index in `PARAMS[]`, 55 params) |
| `MQTT/is02_wire.py` | laptop codec (decode to Baseline JSON shape, encode cmd/map/config) |
| `tests/host/py/test_laptop_codec.py`, `test_status_snapshot.py`, `test_msg_codec.py`, `test_wire_basic.py`, `test_diff_barcode.py` | new tests |
| `docs/evidence/ram_symbols.md` | measured RAM per symbol |
