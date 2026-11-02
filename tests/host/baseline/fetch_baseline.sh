#!/usr/bin/env bash
################################################################################
# IS-02 Host_Test_Suite: Baseline_Build differential library
#
#   tests/host/baseline/fetch_baseline.sh <commit>
#   make -C tests/host base        # same, with the commit recorded in
#                                  # docs/evidence/README.md (BASELINE_COMMIT=)
#
# Builds tests/host/build/libbase.so from the Baseline_Build sources at
# <commit>, with every symbol defined by those sources prefixed "base_", so the
# differential property tests (design Properties 11, 13, 14, 15 and 22;
# Requirements 4.10, 5.9, 6.7, 7.7, 15.5) can load it next to build/libis02.so
# and run both versions on identical inputs in one process.
#
# What goes in
#   * Pure-logic modules, unchanged (git show <commit>:app_program/<file>):
#       line_logic.c imu_logic.c obstacle_logic.c barcode_logic.c bc_detect.c
#       bc_multi.c code39.c nav_map.c speed_ctl.c
#     plus their plain-C support files app_status.c (g_status) and jfmt.c.
#   * Every app_program/*.h at <commit> (the baseline headers; the working
#     tree's app_program/ is NOT on the include path).
#   * Motion targets (Property 11): baseline/base_motion.c #includes the
#     extracted tsk_motion.c, so the LOCAL functions timeout_for(),
#     handle_cmd() and run_primitive() run as committed. The kernel calls it
#     makes come from baseline/shim/tk/tkernel.h (no-op dispatch control,
#     waits time out at once); the os_glue/hal functions it references are
#     host replacements defined in base_motion.c (post_motion() captures the
#     command, time and encoders read 0). The task body is never called.
#   * Parameter_Store float table (Property 22): baseline/base_params.c
#     #includes the extracted app_params.c (plain C) to read the static
#     PARAMS[] table by index; params_set() & co. are the baseline functions.
#   * Hump profiler (Property 15): hump_step() is the baseline imu_logic.c.
#     Its parameters are set in the tsk_imu.c task body (kernel code), so the
#     "hp.<field> = <value>;" lines are copied out of the baseline tsk_imu.c
#     into build/base/src/base_hump_params.inc, included by baseline/base_hump.c.
#     The script fails if any hump_params_t field is not found there.
#
# How the prefix is applied
#   1. compile every file (-fPIC, host gcc) and combine them: gcc -r
#   2. objcopy --prefix-symbols=base_ (renames defined AND undefined symbols)
#   3. objcopy --redefine-syms: put the original names back on the undefined
#      ones (libc / libm: memcpy, sinf, ...), listed with nm -u before step 2
#   4. gcc -shared -Wl,--no-undefined -lm  ->  build/libbase.so
#   5. check that every exported symbol starts with base_
#   So base_hump_step(), base_params_set(), base_g_params, base_bh_motion_turn()...
#
# Output (all under tests/host/build/, git-ignored):
#   base/src/      extracted sources and base_hump_params.inc
#   base/obj/      objects
#   base/COMMIT    full hash of <commit>
#   libbase.so
#
# Needs: git, gcc, objcopy, nm (binutils). Run in WSL or Linux.
################################################################################
set -euo pipefail

if [ $# -ne 1 ] || [ -z "$1" ]; then
    echo "usage: fetch_baseline.sh <commit>" >&2
    exit 2
fi

HERE="$(cd "$(dirname "$0")" && pwd)"
HOST="$(cd "$HERE/.." && pwd)"
cd "$HOST"

CC="${HOST_CC:-gcc}"
OBJCOPY="${OBJCOPY:-objcopy}"
NM="${NM:-nm}"
PREFIX=base_

if ! COMMIT="$(git rev-parse --verify --quiet "$1^{commit}")"; then
    echo "fetch_baseline.sh: '$1' is not a commit in this repository" >&2
    exit 2
fi

PURE_SRCS="line_logic.c imu_logic.c obstacle_logic.c barcode_logic.c bc_detect.c
           bc_multi.c code39.c nav_map.c speed_ctl.c app_status.c jfmt.c"
# extracted for #include by the harness files, not compiled on their own
INCL_SRCS="tsk_motion.c app_params.c"
HARNESS_SRCS="base_motion.c base_params.c base_hump.c"
HUMP_FIELDS="start_deg level_deg min_height_mm start_samples end_samples max_len_mm"

OUT=build/base
SRC=$OUT/src
OBJ=$OUT/obj
rm -rf "$OUT" build/libbase.so
mkdir -p "$SRC" "$OBJ"

# ---- 1. extract ---------------------------------------------------------------
show() {   # <file in app_program/> <destination>
    if ! git show "$COMMIT:app_program/$1" > "$2"; then
        echo "fetch_baseline.sh: app_program/$1 not found at $COMMIT" >&2
        exit 1
    fi
}

headers="$(git ls-tree --name-only --full-tree "$COMMIT" app_program/ | sed -n 's|^app_program/\(.*\.h\)$|\1|p')"
[ -n "$headers" ] || { echo "fetch_baseline.sh: no app_program/*.h at $COMMIT" >&2; exit 1; }
for h in $headers; do
    show "$h" "$SRC/$h"
done
for f in $PURE_SRCS $INCL_SRCS tsk_imu.c; do
    show "$f" "$SRC/$f"
done

# hump profiler parameters from the TSK_TERRAIN_IMU task body
sed -n 's/^[[:space:]]*\(hp\.[a-z_]*[[:space:]]*=[^;]*;\).*$/    \1/p' "$SRC/tsk_imu.c" \
    > "$SRC/base_hump_params.inc"
for fld in $HUMP_FIELDS; do
    n="$(grep -c "hp\.$fld[[:space:]]*=" "$SRC/base_hump_params.inc" || true)"
    if [ "$n" != "1" ]; then
        echo "fetch_baseline.sh: expected one 'hp.$fld = ...;' in the baseline tsk_imu.c, found $n" >&2
        exit 1
    fi
done
rm -f "$SRC/tsk_imu.c"
echo "$COMMIT" > "$OUT/COMMIT"

# ---- 2. compile ---------------------------------------------------------------
# Same language level as the firmware; the frozen baseline is not built with
# -Werror (its warnings are not this build's to fix). Paths are relative to
# tests/host on purpose: the repository path contains spaces.
CFLAGS="-std=gnu11 -fPIC -O2 -g -Wall -Wextra -Wshadow -Wdouble-promotion -Wno-unused-parameter
        -Ibaseline/shim -I$SRC"
objs=""
for f in $PURE_SRCS; do
    o="$OBJ/${f%.c}.o"
    $CC $CFLAGS -c -o "$o" "$SRC/$f"
    objs="$objs $o"
done
for f in $HARNESS_SRCS; do
    o="$OBJ/${f%.c}.o"
    $CC $CFLAGS -Werror -c -o "$o" "baseline/$f"
    objs="$objs $o"
done

# ---- 3. prefix every symbol with base_ ------------------------------------------
# shellcheck disable=SC2086
$CC -r -o "$OBJ/base_all.o" $objs
$NM -u "$OBJ/base_all.o" | awk -v p="$PREFIX" '{ print p $2 " " $2 }' > "$OBJ/undefined.map"
$OBJCOPY --prefix-symbols="$PREFIX" "$OBJ/base_all.o" "$OBJ/base_pfx.o"
$OBJCOPY --redefine-syms="$OBJ/undefined.map" "$OBJ/base_pfx.o"

# ---- 4. link ------------------------------------------------------------------
$CC -shared -Wl,--no-undefined -o build/libbase.so "$OBJ/base_pfx.o" -lm

# ---- 5. check -----------------------------------------------------------------
bad="$($NM -D --defined-only build/libbase.so | awk -v p="$PREFIX" \
        '$3 != "" && index($3, p) != 1 && $3 !~ /^_(init|fini|edata|end|_bss_start)$/ { print $3 }')"
if [ -n "$bad" ]; then
    echo "fetch_baseline.sh: symbols without the $PREFIX prefix in libbase.so:" >&2
    echo "$bad" | sed 's/^/    /' >&2
    exit 1
fi
for sym in hump_step bc_edge bcm_sample obstacle_choose_side params_set bh_motion_turn \
           bh_param_get bh_hump_params; do
    if ! $NM -D --defined-only build/libbase.so | awk '{ print $3 }' | grep -qx "$PREFIX$sym"; then
        echo "fetch_baseline.sh: $PREFIX$sym missing from libbase.so" >&2
        exit 1
    fi
done

echo "libbase.so: Baseline_Build $COMMIT -> tests/host/build/libbase.so (symbols prefixed $PREFIX)"
