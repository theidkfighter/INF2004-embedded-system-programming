#!/usr/bin/env bash
################################################################################
# IS-02 Host_Test_Suite: single entry point (run in WSL or Linux).
#
#   tests/host/run_all.sh                 # random seed
#   tests/host/run_all.sh --seed 12345    # reproduce a run
#   tests/host/run_all.sh --selfcheck     # failure-reporting self-check
#
# --selfcheck runs only the deliberately failing properties
# py/selfcheck_fail.py and js/selfcheck_fail.js (the JavaScript one only when
# node and fast-check are installed) through the same pytest / node --test /
# summary path, with results in build/selfcheck/ instead of build/results/.
# It skips the library build and the static checks, and always exits
# non-zero. test_selfcheck.sh runs it and checks the report.
#
# 1. Builds the host libraries (make -C tests/host all base: libis02.so and
#    the Baseline_Build libbase.so).
# 2. Picks a seed (or takes --seed) and passes it to pytest/Hypothesis
#    (--hypothesis-seed) and to the fast-check tests (IS02_SEED).
# 3. Prints one line per property:
#      P<n> <title> | <payload type or Storage_Unit> | cases=<n> | failed=<n> | seed=<s>
#    and, for a failure, the property, payload type or Storage_Unit, input and
#    seed (Requirements 18.8, 18.9).
#    The static structure checks (test_static.sh) and the compile-fail test
#    (test_compile_fail.sh) run after the property tests; each check is
#    counted under "other tests".
# 4. Exits non-zero if anything failed.
#
# Environment: PYTHON (default .venv/bin/python if present, else python3),
#              NODE (default node), MAKE (default make).
################################################################################
set -u

usage() {
    echo "usage: run_all.sh [--seed <0..2147483647>] [--selfcheck]"
    exit "${1:-0}"
}

SEED=""
SELFCHECK=0
while [ $# -gt 0 ]; do
    case "$1" in
        --selfcheck)
            SELFCHECK=1; shift ;;
        --seed)
            [ $# -ge 2 ] || { echo "run_all.sh: --seed needs a value" >&2; usage 2; }
            SEED="$2"; shift 2 ;;
        --seed=*)
            SEED="${1#--seed=}"; shift ;;
        -h|--help)
            usage 0 ;;
        *)
            echo "run_all.sh: unknown option '$1'" >&2; usage 2 ;;
    esac
done

if [ -z "$SEED" ]; then
    # 31-bit so the same value is valid for Hypothesis and fast-check
    SEED=$(( $(od -An -N4 -tu4 /dev/urandom | tr -d ' ') & 0x7fffffff ))
fi
case "$SEED" in
    ''|*[!0-9]*) echo "run_all.sh: seed must be a non-negative integer, got '$SEED'" >&2; exit 2 ;;
esac
if [ "$SEED" -gt 2147483647 ]; then
    echo "run_all.sh: seed must be below 2^31 (fast-check limit), got $SEED" >&2
    exit 2
fi

HERE="$(cd "$(dirname "$0")" && pwd)"
cd "$HERE" || exit 2

MAKE="${MAKE:-make}"
NODE="${NODE:-node}"
if [ -z "${PYTHON:-}" ]; then
    if [ -x .venv/bin/python ]; then PYTHON=.venv/bin/python; else PYTHON=python3; fi
fi

STATUS=0
if [ "$SELFCHECK" -eq 1 ]; then
    # Separate directory so a self-check never overwrites the last real run.
    RES=build/selfcheck
    PY_TESTS=py/selfcheck_fail.py   # named explicitly: not a test_*.py file
    JS_TESTS=selfcheck_fail.js      # named explicitly: not a *.test.js file
    RERUN="--selfcheck "
else
    RES=build/results
    PY_TESTS=py
    JS_TESTS=""                     # node --test default patterns
    RERUN=""
fi

# Also forward the seed and result file if NODE is a Windows node.exe
# started from WSL (WSL passes only the variables listed in WSLENV).
export WSLENV="${WSLENV:+$WSLENV:}IS02_SEED:IS02_RESULTS"

if [ "$SELFCHECK" -eq 1 ]; then
    echo "== Host_Test_Suite --selfcheck (seed=$SEED): the properties below fail on purpose =="
else
    echo "== Host_Test_Suite (seed=$SEED) =="
fi

# ---- 1. build ---------------------------------------------------------------
# "base" = build/libbase.so, the Baseline_Build differential library
# (rebuilt only when baseline/ or the recorded commit changes).
# The self-check properties do not use the libraries, so --selfcheck skips it.
if [ "$SELFCHECK" -eq 0 ]; then
    if ! "$MAKE" -s all base > build.log 2>&1; then
        cat build.log
        echo "FAILED: host library build (make -C tests/host), see tests/host/build.log"
        exit 1
    fi
    rm -f build.log
fi
mkdir -p "$RES"
rm -f "$RES"/*.jsonl "$RES"/*.log

show_log() {   # <runner> <log> <status>
    echo "FAILED: $1 exited with status $3; last lines of tests/host/$2:"
    tail -n 40 "$2" | sed 's/^/    /'
}

# ---- 2. Python / Hypothesis ---------------------------------------------------
if [ "$SELFCHECK" -eq 1 ] || ls py/test_*.py > /dev/null 2>&1; then
    IS02_SEED="$SEED" IS02_RESULTS="$RES/py.jsonl" \
        "$PYTHON" -m pytest $PY_TESTS -q -p no:cacheprovider --hypothesis-seed="$SEED" \
        > "$RES/pytest.log" 2>&1
    rc=$?
    # 5 = no tests collected
    if [ "$rc" -ne 0 ] && [ "$rc" -ne 5 ]; then
        show_log pytest "$RES/pytest.log" "$rc"
        STATUS=1
    fi
else
    echo "(no Python tests in py/)"
fi

# ---- 3. JavaScript / fast-check ----------------------------------------------
if [ "$SELFCHECK" -eq 1 ] && { ! command -v "$NODE" > /dev/null 2>&1 || [ ! -d js/node_modules/fast-check ]; }; then
    # test_selfcheck.sh looks for this exact line.
    echo "(selfcheck: JavaScript part skipped: node or fast-check not installed)"
elif [ "$SELFCHECK" -eq 1 ] || ls js/*.test.js > /dev/null 2>&1; then
    if [ ! -d js/node_modules/fast-check ]; then
        echo "FAILED: fast-check is not installed; run: (cd tests/host/js && npm install)"
        STATUS=1
    else
        ( cd js && IS02_SEED="$SEED" IS02_RESULTS="../$RES/js.jsonl" "$NODE" --test $JS_TESTS ) \
            > "$RES/node.log" 2>&1
        rc=$?
        if [ "$rc" -ne 0 ]; then
            show_log "node --test" "$RES/node.log" "$rc"
            STATUS=1
        fi
    fi
else
    echo "(no JavaScript tests in js/)"
fi

# ---- 4. static structure checks ----------------------------------------------
if [ "$SELFCHECK" -eq 0 ]; then
    IS02_RESULTS="$RES/static.jsonl" bash ./test_static.sh > "$RES/static.log" 2>&1
    rc=$?
    if [ "$rc" -ne 0 ]; then
        show_log test_static.sh "$RES/static.log" "$rc"
        STATUS=1
    fi
    IS02_RESULTS="$RES/compile_fail.jsonl" bash ./test_compile_fail.sh > "$RES/compile_fail.log" 2>&1
    rc=$?
    if [ "$rc" -ne 0 ]; then
        show_log test_compile_fail.sh "$RES/compile_fail.log" "$rc"
        STATUS=1
    fi
fi

# ---- 5. summary ---------------------------------------------------------------
echo
IS02_RERUN="$RERUN" \
    "$PYTHON" summary.py "$SEED" "$RES/py.jsonl" "$RES/js.jsonl" "$RES/static.jsonl" \
        "$RES/compile_fail.jsonl" || STATUS=1

if [ "$SELFCHECK" -eq 1 ]; then
    if [ "$STATUS" -eq 0 ]; then
        # The properties are written to fail; a pass means the reporting path
        # is broken. Never exit 0 from a self-check.
        echo "Host_Test_Suite --selfcheck: BROKEN, the deliberately failing properties did not fail (seed=$SEED)"
        exit 2
    fi
    echo "Host_Test_Suite --selfcheck: FAILED as intended (seed=$SEED); test_selfcheck.sh checks the report"
    exit 1
fi

if [ "$STATUS" -ne 0 ]; then
    echo "Host_Test_Suite: FAILED (seed=$SEED)"
else
    echo "Host_Test_Suite: PASSED (seed=$SEED)"
fi
exit "$STATUS"
