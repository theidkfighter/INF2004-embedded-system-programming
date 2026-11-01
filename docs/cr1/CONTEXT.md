# CR1 context for every session

Read this once at the start of each session. It does not change from step to step. What does change is in `PROGRESS_FW.md` / `PROGRESS_LT.md`.

## 1. The project in one paragraph

IS-02 Robo-Wheeler (INF2004, team IS-02) is an autonomous line-following car.

- **Hardware and OS:** Raspberry Pi Pico W (RP2040, Cortex-M0+, no FPU) under µT-Kernel 3.0, built single-core (`SMP=0`) with Pico SDK 2.2.0 drivers and an lwIP MQTT client.
- **Seven tasks:**

  | Task | Priority |
  |---|---|
  | TSK_SAFETY_MON | 2 |
  | TSK_MOTION_CTRL | 4 |
  | TSK_LINE_BARCODE | 5 |
  | TSK_OBSTACLE_SCAN | 6 |
  | TSK_TERRAIN_IMU | 7 |
  | TSK_NAV_PLANNER | 8 |
  | TSK_TELEMETRY | 10 |

- **Where things are:** the firmware is in `app_program/`; the laptop tools (dashboard, monitor, scripts) are in `MQTT/`.

**Change Requirement 1 (CR1)** asks for four things:
- every application data field uses only the bits its valid range needs (bit-fields, or masks and shifts);
- faults go in one shared bitmask;
- packed data shared between tasks, or with an ISR, is race-free;
- MQTT payloads become compact binary.

The Week 13 evidence is: the data representation table, RAM before/after, message sizes before/after, and correctness and timing evidence. The original text is summarised in `spec/requirements.md` (Introduction).

The spec in `spec/` (requirements, design, tasks) is the plan. **Hard rule from the spec: the robot's behaviour must not change** (Requirement 19), apart from the differences the spec itself allows.

## 2. Commits and branches

| What | Commit / branch |
|---|---|
| Repository | `github.com/theidkfighter/INF2004-embedded-system-programming` |
| Integration branch | `Poh-Loon` |
| Firmware track branch | `cr1/fw` (from `Poh-Loon`) |
| Laptop track branch | `cr1/laptop` (from `Poh-Loon`) |
| Baseline_Build (RAM, message-size and behaviour "before") | `bca79643d6f9bdd3938c353fa5f6fe791fc68f94` (recorded in `docs/evidence/README.md`) |
| Step 0 + first part of Step 1 (timing "before": probes) | `542368f` ("Add minimum-bit data representation: Step 0 tooling and Step 1 foundations") |
| Obstacle-avoidance work already merged into Poh-Loon | `codex/ultrasonic-refinement` (`45dfa2c`) and `main` (`af2f71e`) |

`INTEGRATION.md` gives the branch and merge rules. Never rebase or force-push `Poh-Loon`.

## 3. Where a Claude chat works

The chat works on its own clone of the repo in its cloud workspace, not on the user's laptop. Git commits inside the user's connected laptop folders leave `.lock` files, because deleting is blocked there.

- **Pushing:**
  - It needs push access. The chat asks for it with its "add repository" tool (`access: push`). That needs the user's GitHub account linked to Claude, with write access to the team repo.
  - Without push access, the chat commits locally, makes a patch (`git format-patch origin/<branch>..HEAD`) and gives it to the user, who applies it in their clone with `git am <file>`. **On 1 Nov 2026 no GitHub account was linked, so patches are the default.**
- **Which branch:** push to your track branch (`cr1/fw` or `cr1/laptop`), never straight to `Poh-Loon`. The user merges into `Poh-Loon` as `INTEGRATION.md` says.

## 4. Environment set-up (cloud workspace, Ubuntu 24.04)

**Checked on 1 Nov 2026:** with these commands, `Poh-Loon` at `542368f` builds with 0 warnings both with `APP_PERF=0` and with `APP_PERF=1`, and `tools/evidence/build_check.sh` prints PASS.

```sh
# compiler (13.2.1, the version the README requires)
sudo apt-get update && sudo apt-get install -y gcc-arm-none-eabi libnewlib-arm-none-eabi build-essential
arm-none-eabi-gcc --version                       # 13.2.1

# Pico SDK 2.2.0 with the three submodules the build needs
git clone -b 2.2.0 --depth 1 https://github.com/raspberrypi/pico-sdk.git ~/pico/pico-sdk
git -C ~/pico/pico-sdk submodule update --init --depth 1 lib/tinyusb lib/cyw43-driver lib/lwip
export PICO_SDK_PATH=~/pico/pico-sdk

# placeholder config, for compiling only (both files are gitignored, never commit them)
cd <repo>
cp config/wifi_credentials.example.h config/wifi_credentials.h
cp config/mqtt_config.example.h config/mqtt_config.h
sed -i 's/UTK_WIFI_CREDENTIALS_SET *0/UTK_WIFI_CREDENTIALS_SET   1/' config/wifi_credentials.h
sed -i 's/MQTT_CONFIG_SET *(0)/MQTT_CONFIG_SET        (1)/' config/mqtt_config.h

# host test tools
pip install --break-system-packages pytest hypothesis    # or a venv in tests/host/.venv
(cd tests/host/js && npm ci)                              # fast-check; needs Node 18+
```

- **Flashable builds:** a `.uf2` built in the cloud has **placeholder Wi-Fi settings** and can't join the hotspot. The user builds the flashable file on their own laptop (WSL, `README.md` sections 3 and 5) with their own `config/` files.
- **On the user's laptop (WSL)** the same commands work; see `README.md` sections 3 and 5 and `tests/host/README.md`.

**Reference sizes** (`arm-none-eabi-size`, measured 1 Nov 2026 with the commands above):

| Build | text | data | bss | data + bss |
|---|---|---|---|---|
| Baseline_Build `bca7964` | 398,788 | 2,808 | 107,972 | 110,780 |
| `542368f`, `APP_PERF=0` | 398,820 | 2,808 | 107,972 | 110,780 |
| `542368f`, `APP_PERF=1` | 400,324 | 2,808 | 108,896 | 111,704 |

## 5. The gate (run before every commit)

**Firmware track:**

```sh
tools/evidence/build_check.sh -- -j8                    # clean build vs Baseline_Build warnings: must print PASS
(cd build_make && make clean && make APP_PERF=1 -j8)    # timing-probe build: 0 warnings
bash tests/host/run_all.sh --seed 12345                 # must end "Host_Test_Suite: PASSED"
# from FW-09 on, also:
tools/evidence/build_check.sh -- APP_LEGACY_TEXT_CMD=1 -j8
```

`tools/evidence/baseline_warnings.txt` (Baseline_Build: 0 warnings) is committed. If it's missing, run `tools/evidence/build_check.sh --record-baseline -- -j8` once.

**Laptop track:**

```sh
bash tests/host/run_all.sh --seed 12345                 # must end "Host_Test_Suite: PASSED"
python3 -m py_compile MQTT/*.py                         # every tool still imports
node -e "require('./MQTT/is02_codec.js')"               # once the JS codec exists
```

A step is done only when its gate passes. If a check can't pass, stop, write why in the session log of your PROGRESS file, and don't merge half-finished code into `Poh-Loon`.

## 6. Coding rules (from the codebase and the spec)

- **Style:** Barr-C: fixed-width types, `g_` / `p_` / `b_` prefixes, braces everywhere, `LOCAL` for file-local functions in kernel-side files. No dynamic memory: `malloc`/`free` are forbidden (Requirement 19.6).
- **Layering:**
  - `hal_*.c` and `app_net.c` include Pico SDK headers; `tsk_*.c` and `os_glue.c` include `tk/tkernel.h`; never both in one file.
  - Pure-logic modules (`*_logic.c`, `bc_*.c`, `code39.c`, `nav_map.c`, `speed_ctl.c`) and the new plain headers (`bitpack.h`, `crit.h`, `status.h`, `wire*.h`, `msg_codec.h`, `evt.h`) include no kernel or SDK header, so the host tests can compile them.
- **Packing:** only through `bitpack.h` masks and shifts (design C1). Never C bit-fields across a boundary; never `memcpy` a struct onto the wire.
- **Concurrency** (Requirements 9 and 10, design "Critical-section strategy"):
  - Each Storage_Unit has either one writer making one aligned store, or a read-modify-write inside `crit_dsp_enter/exit` (between tasks) or `crit_irq_enter/exit` (task vs ISR).
  - No blocking kernel call inside a critical section. Keep each one under 10 µs.
- **Warnings:** the flags are `-Wall -Wextra -Wshadow -Wdouble-promotion`; new or changed files must produce no warning.
- **Obstacle maths must not change:** `obstacle_logic.c` and the planner in `tsk_obstacle.c` must give plans identical to the Baseline_Build (Requirement 6.7, Property 14). Change only the boundaries the tasks name.
- **Laptop tools** need only Python 3 + `paho-mqtt`, and the dashboard must keep working from `file://`.

## 7. Session rules

1. **Start:** read your track's PROGRESS file and `INTEGRATION.md` (sync points) first. Start only if the previous step of your track is ticked done and every sync point your step depends on is met.
2. **Read only what the step guide lists** (sections of `spec/design.md` and `spec/requirements.md`, found by heading). The spec files are long; reading them whole wastes usage.
3. **Do exactly the tasks of the step.** Don't start the next step, even if time is left. Don't edit files the other track owns (`INTEGRATION.md`, ownership table).
4. **Review flags:** don't apply an item from `REVIEW_FLAGS.md` unless its decision there says **accepted**. If the work runs into a flagged item, follow the spec as written and note it in the session log.
5. **Don't edit `spec/requirements.md` or `spec/design.md`.** If the spec is wrong or impossible, add an item to `REVIEW_FLAGS.md` and stop at that task.
6. **End:**
   - Run the gate.
   - Tick the tasks in your PROGRESS file.
   - Add a session-log entry: date, commit, gate results with the host-test totals and (firmware track) the `arm-none-eabi-size` line, open issues, and what the user must check on the car.
   - Commit with the message the guide gives, then push or make a patch as section 3 says.
7. **Keep answers to the user short:** what was done, what failed, what they must do next.
