#!/usr/bin/env bash
################################################################################
# IS-02 Host_Test_Suite: compile-fail test for narrow enums (Requirement 2.13).
#
#   bash tests/host/test_compile_fail.sh
#
# Copies the app_program/ headers to a temporary directory, enlarges one enum
# (or widens one field) past what its Packed_Field holds, and compiles a file
# that includes the header. The compile must fail, and the error must name the
# field. The unchanged copy must compile, so a failure can only come from the
# edit. The firmware sources are not touched.
#
# Prints one PASS / FAIL line per check and, when IS02_RESULTS is set, writes
# one JSON-lines record per check to that file for summary.py (kind "test",
# runner "static"). Exit status 0 when nothing failed, 1 otherwise.
#
# Environment: HOST_CC (default gcc), IS02_RESULTS (optional result file)
################################################################################
set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
APP="$HERE/../../app_program"
HOST_CC="${HOST_CC:-gcc}"
RESULTS="${IS02_RESULTS:-}"
[ -z "$RESULTS" ] || : > "$RESULTS"

TMP="$(mktemp -d)" || exit 2
trap 'rm -rf "$TMP"' EXIT

N_FAIL=0

report() {   # <check> <pass|fail> [detail]
    local failed=false input=null
    if [ "$2" = pass ]; then
        echo "PASS $1"
    else
        echo "FAIL $1: $3"
        N_FAIL=$((N_FAIL + 1))
        failed=true
        input="\"$(printf '%s' "$3" | tr -d '"\\\n')\""
    fi
    if [ -n "$RESULTS" ]; then
        printf '{"runner": "static", "kind": "test", "n": null, "title": "%s", "unit": null, "cases": 1, "failed": %s, "skipped": false, "seed": null, "phase": "call", "nodeid": "test_compile_fail.sh::%s", "input": %s}\n' \
            "$1" "$failed" "$1" "$input" >> "$RESULTS"
    fi
}

compile() {   # <header> <include dir>: prints the compiler output, sets the status
    printf '#include "%s"\n' "$1" |
        "$HOST_CC" -std=gnu11 -fsyntax-only -I"$2" -x c - 2>&1
}

# check <check name> <header to include> <message> <file> <sed expression>
check() {
    local dir="$TMP/$1" out
    mkdir -p "$dir"
    cp "$APP"/*.h "$dir"/
    sed -i "$5" "$dir/$4"
    if cmp -s "$dir/$4" "$APP/$4"; then
        report "$1" fail "the edit changed nothing in $4"
    elif out="$(compile "$2" "$dir")"; then
        report "$1" fail "compiled with the enlarged field"
    elif ! printf '%s' "$out" | grep -qF "$3"; then
        report "$1" fail "error does not name the field: $out"
    else
        report "$1" pass
    fi
}

if out="$(compile status.h "$APP")" && out2="$(compile imu_logic.h "$APP")"; then
    report compile_fail:unchanged pass
else
    report compile_fail:unchanged fail "the unchanged headers do not compile: ${out:-} ${out2:-}"
fi

check compile_fail:sys_state status.h "status.sys.state too narrow" app_types.h \
    's/^\( *\)SYS_STATE_COUNT\b/\1SYS_EXTRA,\n\1SYS_STATE_COUNT/'
check compile_fail:mev status.h "status.sys.mev too narrow" app_types.h \
    's/^\( *\)MEV_COUNT\b/\1MEV_EXTRA,\n\1MEV_COUNT/'
check compile_fail:line_state status.h "status.line_a.line_state too narrow" line_logic.h \
    's/^\( *\)LINE_LOST\b/\1LINE_EXTRA1,\n\1LINE_EXTRA2,\n\1LINE_LOST/'
check compile_fail:stop_reason status.h "status.sys.stop_reason too narrow" wire_ids.h \
    's/^\( *\)STOP_REASON_COUNT\b/\1STOP_X1, STOP_X2, STOP_X3, STOP_X4, STOP_X5, STOP_X6, STOP_X7,\n\1STOP_REASON_COUNT/'
check compile_fail:field_fit status.h "status.sys.mev does not fit its unit" status.h \
    's/^#define ST_SH_MEV .*/#define ST_SH_MEV (30u)/'
check compile_fail:hump_state imu_logic.h "hump_ctx.state too narrow" imu_logic.h \
    's/^\( *\)HUMP_DESCENT\b/\1HUMP_EXTRA,\n\1HUMP_DESCENT/'
check compile_fail:hump_counters imu_logic.h "hump_ctx.n_up, n_down, n_level too narrow" robot_config.h \
    's/^#define HUMP_END_SAMPLES .*/#define HUMP_END_SAMPLES (8)/'

[ "$N_FAIL" -eq 0 ]
