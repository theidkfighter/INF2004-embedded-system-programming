# RAM usage comparison (Requirement 22)

Builds and commits are listed in [README.md](README.md). Both builds use `APP_PERF=0` and `APP_SELFTEST=0`, the same `make` command line and the same `arm-none-eabi-gcc` version, and each is built after `make clean` from a working tree with no uncommitted changes (Requirement 22.7).

Filled by: `tools/evidence/ram_report.sh` run once per build (2 Nov 2026). The two full reports are in `docs/evidence/raw/baseline_ram_report_2026-11-02.md` and `docs/evidence/raw/revised_ram_report_2026-11-02.md`. Static RAM only: the kernel's own message pool is outside `.bss` (see section 6, flag R3).

## 1. Build record (Requirements 22.4, 22.7)

Filled by: `ram_report.sh` (header lines).

| Item | Baseline_Build | Revised_Build |
|---|---|---|
| Git commit | `bca79643d6f9bdd3938c353fa5f6fe791fc68f94` | `36a380bc3d8daf0f945e6c6853db1ab53914633e` |
| Working tree clean (`git status --short` empty) | yes (before and after the build; the placeholder `config/*.h` files are gitignored) | yes (same) |
| `make` command line | `make clean && make APP_ROBOT=1 APP_PERF=0 -j8` in `build_make/` | `make clean && make APP_ROBOT=1 APP_PERF=0 -j8` in `build_make/` |
| Build options (`APP_ROBOT`, `APP_LEGACY_TEXT_CMD`, `APP_PERF`, `APP_SELFTEST`) | `APP_ROBOT=1`, `APP_PERF=0` as given; the Baseline_Build `build_make/` has no `APP_PERF`, `APP_LEGACY_TEXT_CMD` or `APP_SELFTEST` option (the extra variable is ignored by `make`) and writes no `.robot_opts` file | `APP_ROBOT=1`, `APP_PERF=0` as given; `.robot_opts` = `legacy0-perf0-self0` (`APP_LEGACY_TEXT_CMD=0`, `APP_PERF=0`, `APP_SELFTEST=0`) |
| `arm-none-eabi-gcc --version` (first line) | arm-none-eabi-gcc (15:13.2.rel1-2) 13.2.1 20231009 | arm-none-eabi-gcc (15:13.2.rel1-2) 13.2.1 20231009 |
| Same command line, options and compiler for both builds | yes | |

Environment: WSL Ubuntu 24.04, `PICO_SDK_PATH=$HOME/pico/pico-sdk` (Pico SDK 2.2.0). Each build is a `git worktree` of its commit with placeholder `config/wifi_credentials.h` and `config/mqtt_config.h` made from the `.example.h` files as in `docs/cr1/CONTEXT.md` section 4. The ELF is `build_make/is02_robot_smp0.elf` and the map `build_make/is02_robot_smp0.map`.

Revised_Build commit: the firmware (`app_program/`, `MQTT/`, `build_make/`) at `36a380b` is the firmware of the Week 13 submission. The later commits of CR1 Week 13 change documents and `tools/evidence/` only.

## 2. Unedited `arm-none-eabi-size` output (Requirement 22.1)

Filled by: `ram_report.sh` (verbatim `arm-none-eabi-size` block). Do not edit the output.

Baseline_Build:

```
   text	   data	    bss	    dec	    hex	filename
 398788	   2808	 107972	 509568	  7c680	is02_robot_smp0.elf
```

Revised_Build:

```
   text	   data	    bss	    dec	    hex	filename
 409668	   2788	  60160	 472616	  73628	is02_robot_smp0.elf
```

The `arm-none-eabi-size -A` output of each build is in the raw reports. `.data` and `.bss` there are 2808 / 107972 bytes (Baseline_Build) and 2788 / 60160 bytes (Revised_Build), the same as the columns above.

## 3. Static RAM totals (Requirements 22.1, 22.2, 22.5)

Filled by: `ram_report.sh` (`.data`, `.bss` and sum). The saving and percentage are computed from the two rows.

| Section | Baseline_Build (bytes) | Revised_Build (bytes) | Saving (bytes) | Saving (% of Baseline_Build, 0.1) |
|---|---|---|---|---|
| `.data` | 2,808 | 2,788 | 20 | 0.7 |
| `.bss` | 107,972 | 60,160 | 47,812 | 44.3 |
| `.data` + `.bss` | 110,780 | 62,948 | 47,832 | 43.2 |

Saving = Baseline_Build `.data` + `.bss` − Revised_Build `.data` + `.bss` = 110,780 − 62,948 = 47,832 bytes. Percentage = 47,832 ÷ 110,780 × 100 = 43.2 %.

Requirement 22.5 check (Revised_Build sum ≤ Baseline_Build sum): **pass** (62,948 ≤ 110,780).

## 4. Per-item RAM (Requirements 22.3, 22.6)

Filled by: `ram_report.sh` (per-symbol sizes from `arm-none-eabi-nm -S`, cross-checked against the linker map file: both gave the same size for every item in both builds). The raw sample ring is the sum of its replacing symbols.

| Item | Baseline_Build symbol(s) | Baseline_Build (bytes) | Revised_Build symbol(s) | Revised_Build (bytes) | Saving (bytes) |
|---|---|---|---|---|---|
| Status_Snapshot | `g_status` | 140 | `g_status` | 60 | 80 |
| Message_Pool buffer | `g_mpf_buf` | 4000 | `g_mpf_buf` | 1600 | 2400 |
| Frame_Ring | `g_frames` | 32896 | `g_frames` | 3520 | 29376 |
| Message slots | `g_msgs` | 6432 | `g_msgs` | 528 | 5904 |
| Barcode edge ring | `g_bc_ring` | 768 | `g_bc_ring` | 256 | 512 |
| Raw sample ring | `g_raw_er` 1,024 + `g_raw_el` 1,024 + `g_raw_adc` 512 | 2560 | `g_raw` 1,024 | 1024 | 1536 |
| Command slots | `g_cmds` | 768 | `g_cmds` | 192 | 576 |
| Parameter_Store | `g_params` | 220 | `g_params` | 160 | 60 |
| Sum of the eight items | | 47,784 | | 7,340 | 40,444 |

## 5. Renamed, split or merged symbols (Requirement 22.6)

Filled by: `ram_report.sh -r g_raw=g_raw` for the Revised_Build.

| Baseline_Build symbol | Revised_Build replacing symbol(s) | Sizes of each replacing symbol (bytes) | Sum (bytes) |
|---|---|---|---|
| `g_raw_er` (1,024), `g_raw_el` (1,024), `g_raw_adc` (512) | `g_raw` (one 32-bit word per sample, 256 entries) | `g_raw` 1,024 | 1,024 |

## 6. Notes and deviations

- **The message pool is not in `.bss` (flag R3, open).** `os_glue.c` creates the pool with `.mpfatr = TA_TFIFO | TA_RNG3`, without `TA_USERBUF`. With `USE_IMALLOC = 1` the kernel allocates the pool itself with `knl_Imalloc()` (`kernel/tkernel/mempfix.c`), so `g_mpf_buf` is never used and the pool does not appear in `arm-none-eabi-size`. The `g_mpf_buf` row above is therefore a dead buffer. The real pool is `MSG_BLOCKS × block size` from `tools/evidence/type_sizes.sh` (see [message_sizes.md](message_sizes.md) section 1): Baseline_Build 40 × 96 = **3,840** bytes, Revised_Build 40 × 36 = **1,440** bytes, a saving of **2,400** bytes (62.5 %), taken from the kernel's own memory area, not from `.data` + `.bss`. Block sizes are multiples of `sizeof(FREEL)` = 4, so the kernel does not round them.
- **Effect on the totals.** The `.bss` saving in section 3 includes 2,400 bytes from the dead `g_mpf_buf`. Without it the static saving is 45,432 bytes. The real pool shrinks by the same 2,400 bytes in the kernel memory area, so the total RAM saving is unchanged.
- **`g_mpf_buf` is 1,600 bytes, not the 1,440 of the design estimate.** The buffer is declared `MSG_BLOCKS × (MSG_BLK_WORDS + 1)` words, which is 40 bytes a block (36 + 4).
- **Largest other changes** (symbols outside the eight items; see the older [ram_symbols.md](ram_symbols.md), which was made at an earlier commit and is superseded by this file): `g_bcm` 3,732 → 1,848, `g_buf` 800 → 0, `g_in` and `g_config` 1,024 → 263, `s_extra` 680 → 0, `s_params` 600 → 0, `g_tx_frame` 512 → 54, `g_bc` 512 → 212 bytes. Run `ram_report.sh` or `arm-none-eabi-nm -S` on the two ELF files for the current numbers.
- Task stacks are not part of `.bss` here, and no stack measurement was made. The smaller local contexts (`hump_ctx_t`, `mev_ctx_t`) save stack that these numbers do not show.
- `text` is not part of Requirement 22. The Revised_Build `text` here is 409,668 bytes; the ledger's session 4 line says 409,624 for the same firmware built in the main checkout. The worktree was in a different directory, and the 44-byte difference was not investigated.
- Both builds used `-j8`; the job count does not change the output.
