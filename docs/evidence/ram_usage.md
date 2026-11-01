# RAM usage comparison (Requirement 22)

Builds and commits are listed in [README.md](README.md). Both builds use `APP_PERF=0` and `APP_SELFTEST=0`, the same `make` command line and the same `arm-none-eabi-gcc` version, and each is built after `make clean` from a working tree with no uncommitted changes (Requirement 22.7).

Filled by: `tools/evidence/ram_report.sh build_make/mtkernel_3.elf build_make/mtkernel_3.map`, run once per build. Paste its Markdown output into the sections below.

## 1. Build record (Requirements 22.4, 22.7)

Filled by: `ram_report.sh` (header lines).

| Item | Baseline_Build | Revised_Build |
|---|---|---|
| Git commit | `bca79643d6f9bdd3938c353fa5f6fe791fc68f94` | `<fill in>` |
| Working tree clean (`git status --short` empty) | `<yes/no>` | `<yes/no>` |
| `make` command line | `<fill in>` | `<fill in>` |
| Build options (`APP_ROBOT`, `APP_LEGACY_TEXT_CMD`, `APP_PERF`, `APP_SELFTEST`) | `<fill in>` | `<fill in>` |
| `arm-none-eabi-gcc --version` (first line) | `<fill in>` | `<fill in>` |
| Same command line, options and compiler for both builds | `<yes/no>` | |

## 2. Unedited `arm-none-eabi-size` output (Requirement 22.1)

Filled by: `ram_report.sh` (verbatim `arm-none-eabi-size` block). Do not edit the output.

Baseline_Build:

```
<paste unedited arm-none-eabi-size output>
```

Revised_Build:

```
<paste unedited arm-none-eabi-size output>
```

## 3. Static RAM totals (Requirements 22.1, 22.2, 22.5)

Filled by: `ram_report.sh` (`.data`, `.bss` and sum). The saving and percentage are computed from the two rows.

| Section | Baseline_Build (bytes) | Revised_Build (bytes) | Saving (bytes) | Saving (% of Baseline_Build, 0.1) |
|---|---|---|---|---|
| `.data` | | | | |
| `.bss` | | | | |
| `.data` + `.bss` | | | | |

Saving = Baseline_Build `.data` + `.bss` − Revised_Build `.data` + `.bss`. Percentage = saving ÷ Baseline_Build sum × 100, rounded to one decimal place.

Requirement 22.5 check (Revised_Build sum ≤ Baseline_Build sum): `<pass/fail>`

## 4. Per-item RAM (Requirements 22.3, 22.6)

Filled by: `ram_report.sh` (per-symbol sizes from `arm-none-eabi-nm -S`, cross-checked against the linker map file). Where the Revised_Build renames, splits or merges a symbol, list every replacing symbol and give their summed size as the Revised_Build size.

| Item | Baseline_Build symbol(s) | Baseline_Build (bytes) | Revised_Build symbol(s) | Revised_Build (bytes) | Saving (bytes) |
|---|---|---|---|---|---|
| Status_Snapshot | `g_status` | | `<fill in>` | | |
| Message_Pool buffer | `g_mpf_buf` | | `<fill in>` | | |
| Frame_Ring | `g_frames` | | `<fill in>` | | |
| Message slots | `g_msgs` | | `<fill in>` | | |
| Barcode edge ring | `g_bc_ring` | | `<fill in>` | | |
| Raw sample ring | `g_raw_er` + `g_raw_el` + `g_raw_adc` | | `<fill in>` | | |
| Command slots | `g_cmds` | | `<fill in>` | | |
| Parameter_Store | `g_params` | | `<fill in>` | | |

## 5. Renamed, split or merged symbols (Requirement 22.6)

Filled by: `ram_report.sh` (replacement-symbol list). Leave the table empty if no symbol in section 4 changed name.

| Baseline_Build symbol | Revised_Build replacing symbol(s) | Sizes of each replacing symbol (bytes) | Sum (bytes) |
|---|---|---|---|
| | | | |

## 6. Notes and deviations

Record here any measurement that could not be taken as described above, with the reason (Requirement 24.5).
