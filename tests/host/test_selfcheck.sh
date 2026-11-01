#!/usr/bin/env bash
################################################################################
# IS-02 Host_Test_Suite: self-check of the failure report (Requirements 18.8,
# 18.9). Run in WSL or Linux.
#
#   bash tests/host/test_selfcheck.sh               # seed 12345
#   bash tests/host/test_selfcheck.sh --seed 777
#
# Runs `run_all.sh --selfcheck --seed <s>`, which runs only the deliberately
# failing properties py/selfcheck_fail.py (Hypothesis, unit selfcheck.u8,
# shrinks to v=200) and js/selfcheck_fail.js (fast-check, unit selfcheck.u16,
# shrinks to [1000]). Neither file is collected by a normal run_all.sh.
#
# PASS needs all of:
#   exit_status        run_all.sh --selfcheck exits non-zero;
#   <runner>:summary   the summary line
#                        P0 <title> | <unit> | cases=<n> | failed=<n> | seed=<s>
#                      with n >= 1 and the seed passed in;
#   <runner>:property  a "FAILED property P0 <title>" block, which contains
#   <runner>:unit        "payload type / Storage_Unit: <unit>",
#   <runner>:seed        "seed: <s>" and
#   <runner>:input       the shrunk input value.
# The JavaScript checks are reported as SKIP when run_all.sh says node or
# fast-check is not installed.
#
# Prints the run_all.sh output, then one PASS / FAIL / SKIP line per check.
# Exit status 0 when nothing failed, 1 when a check failed, 2 on bad usage.
#
# Environment: as for run_all.sh (PYTHON, NODE).
################################################################################
set -u

SEED=12345
while [ $# -gt 0 ]; do
    case "$1" in
        --seed)
            [ $# -ge 2 ] || { echo "test_selfcheck.sh: --seed needs a value" >&2; exit 2; }
            SEED="$2"; shift 2 ;;
        --seed=*)
            SEED="${1#--seed=}"; shift ;;
        -h|--help)
            echo "usage: test_selfcheck.sh [--seed <0..2147483647>]"; exit 0 ;;
        *)
            echo "test_selfcheck.sh: unknown option '$1'" >&2; exit 2 ;;
    esac
done

HERE="$(cd "$(dirname "$0")" && pwd)"

out="$(bash "$HERE/run_all.sh" --selfcheck --seed "$SEED" 2>&1)"
rc=$?
printf '%s\n' "$out" | sed 's/^/  | /'
echo

FAILS=0
result() {   # <PASS|FAIL|SKIP> <check> <detail>
    echo "$1 $2: $3"
    [ "$1" = FAIL ] && FAILS=$((FAILS + 1))
    return 0
}

if [ "$rc" -ne 0 ]; then
    result PASS exit_status "run_all.sh --selfcheck exited with $rc"
else
    result FAIL exit_status "run_all.sh --selfcheck exited with 0"
fi

# Lines of the "FAILED property P0 <title>" block, up to the next blank line.
failed_block() {   # <title>
    printf '%s\n' "$out" | awk -v h="FAILED property P0 $1" \
        '$0 == h { on = 1; print; next } on && $0 == "" { exit } on { print }'
}

has_line() {   # <text> <glob>: does a line of <text> match <glob>?
    local line
    while IFS= read -r line; do
        # shellcheck disable=SC2053
        [[ "$line" == $2 ]] && return 0
    done <<< "$1"
    return 1
}

check_runner() {   # <runner> <title> <unit> <input glob>
    local runner="$1" title="$2" unit="$3" input="$4" block
    if has_line "$out" "P0 $title | $unit | cases=[1-9]* | failed=[1-9]* | seed=$SEED"; then
        result PASS "$runner:summary" "P0 $title | $unit | cases=... | failed=... | seed=$SEED"
    else
        result FAIL "$runner:summary" "no summary line 'P0 $title | $unit | cases=<n> | failed=<n>=1+ | seed=$SEED'"
    fi
    block="$(failed_block "$title")"
    if [ -z "$block" ]; then
        result FAIL "$runner:property" "no 'FAILED property P0 $title' block"
        result FAIL "$runner:unit" "no failure block to check"
        result FAIL "$runner:seed" "no failure block to check"
        result FAIL "$runner:input" "no failure block to check"
        return
    fi
    result PASS "$runner:property" "FAILED property P0 $title"
    if has_line "$block" "*payload type / Storage_Unit: $unit"; then
        result PASS "$runner:unit" "$unit"
    else
        result FAIL "$runner:unit" "failure block does not name unit $unit"
    fi
    if has_line "$block" "*seed: $SEED"; then
        result PASS "$runner:seed" "$SEED"
    else
        result FAIL "$runner:seed" "failure block does not give seed $SEED"
    fi
    if has_line "$block" "$input"; then
        result PASS "$runner:input" "shrunk input reported"
    else
        result FAIL "$runner:input" "failure block does not show the shrunk input ($input)"
    fi
}

check_runner python "Selfcheck deliberately failing property (Python)" selfcheck.u8 "*v=200*"

if has_line "$out" "(selfcheck: JavaScript part skipped: *"; then
    for c in summary property unit seed input; do
        result SKIP "js:$c" "node or fast-check not installed"
    done
else
    check_runner js "Selfcheck deliberately failing property (JavaScript)" selfcheck.u16 "*\[1000\]*"
fi

echo
if [ "$FAILS" -ne 0 ]; then
    echo "test_selfcheck.sh: FAILED ($FAILS check(s), seed=$SEED)"
    exit 1
fi
echo "test_selfcheck.sh: PASSED (seed=$SEED)"
exit 0
