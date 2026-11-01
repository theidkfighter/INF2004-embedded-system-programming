#!/usr/bin/env bash
# Firmware build check (Requirement 19.5)
#
# Builds the robot firmware from clean and checks its compiler warnings
# against the Baseline_Build:
#   1. runs `make clean && make APP_ROBOT=1` in build_make/ and saves the log
#   2. counts the warnings in the log
#   3. compares the count with the recorded Baseline_Build count
#   4. lists every warning that names a file added or changed since the
#      Baseline_Build commit (committed, staged, unstaged or untracked)
#
# Exit status: 0 pass; 1 check failed (build failed, more warnings than the
# Baseline_Build, or a warning in a new or changed file); 2 usage or setup
# error (unknown commit, no recorded warning count, ...).
#
# Usage:
#   tools/evidence/build_check.sh [options] [-- MAKE_ARGS...]
#
# Options:
#   --baseline-commit REV  Baseline_Build commit. Default: $BASELINE_COMMIT,
#                          else the first commit hash on a line mentioning
#                          the baseline in docs/evidence/README.md, else HEAD
#                          (then only uncommitted and untracked files count
#                          as changed).
#   --baseline-warnings N  Baseline_Build warning count. Default:
#                          $BASELINE_WARNINGS, else the warnings= line of the
#                          baseline file.
#   --baseline-file FILE   Recorded count. Default:
#                          tools/evidence/baseline_warnings.txt
#   --record-baseline      Build the Baseline_Build commit in a temporary git
#                          worktree, write its warning count to the baseline
#                          file and exit.
#   --log FILE             Build log. Default: build_make/build_check.log
#   -h, --help             Show this text.
#
# MAKE_ARGS are passed to both make calls, for example `-- APP_PERF=0 -j8`.
# Set MAKE to use another make program and PICO_SDK_PATH as for a normal
# build.

set -u
set -o pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
if ! REPO=$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null); then
    echo "build_check: $SCRIPT_DIR is not inside a git work tree" >&2
    exit 2
fi
BUILD_DIR="$REPO/build_make"
README="$REPO/docs/evidence/README.md"
MAKE=${MAKE:-make}
# gcc prints "warning:" only in the C locale; translated messages would not
# be counted.
export LC_ALL=C

baseline_commit=${BASELINE_COMMIT:-}
baseline_warnings=${BASELINE_WARNINGS:-}
baseline_file="$SCRIPT_DIR/baseline_warnings.txt"
log_file="$BUILD_DIR/build_check.log"
record=0
make_args=()

usage() { sed -n '2,/^$/s/^# \{0,1\}//p' "${BASH_SOURCE[0]}"; }
die_usage() { echo "build_check: $1" >&2; usage >&2; exit 2; }
die_setup() { echo "build_check: $1" >&2; exit 2; }

abs_path() { realpath -m -- "$1"; }

while [ $# -gt 0 ]; do
    case "$1" in
        --baseline-commit)
            [ $# -ge 2 ] || die_usage "$1 needs a value"
            baseline_commit=$2; shift 2 ;;
        --baseline-warnings)
            [ $# -ge 2 ] || die_usage "$1 needs a value"
            baseline_warnings=$2; shift 2 ;;
        --baseline-file)
            [ $# -ge 2 ] || die_usage "$1 needs a value"
            baseline_file=$(abs_path "$2"); shift 2 ;;
        --log)
            [ $# -ge 2 ] || die_usage "$1 needs a value"
            log_file=$(abs_path "$2"); shift 2 ;;
        --record-baseline)
            record=1; shift ;;
        -h|--help)
            usage; exit 0 ;;
        --)
            shift; make_args=("$@"); break ;;
        *)
            die_usage "unknown option: $1" ;;
    esac
done

case "$baseline_warnings" in
    ''|*[!0-9]*) [ -z "$baseline_warnings" ] || die_usage "baseline warning count must be a whole number, got '$baseline_warnings'" ;;
esac

git_() { git -C "$REPO" -c core.quotepath=off "$@"; }

# First token of 7..40 hex digits, on a README line that mentions the
# baseline, that git resolves to a commit (placeholders are skipped).
readme_commit() {
    [ -f "$README" ] || return 1
    local tok
    for tok in $(grep -i 'baseline' "$README" | grep -oE '\b[0-9a-fA-F]{7,40}\b'); do
        if git_ rev-parse -q --verify "$tok^{commit}" >/dev/null 2>&1; then
            echo "$tok"
            return 0
        fi
    done
    return 1
}

baseline_source="--baseline-commit / BASELINE_COMMIT"
if [ -z "$baseline_commit" ]; then
    if baseline_commit=$(readme_commit); then
        baseline_source="docs/evidence/README.md"
    else
        baseline_commit=HEAD
        baseline_source="fallback: no Baseline_Build hash found in docs/evidence/README.md"
    fi
fi
base_hash=$(git_ rev-parse -q --verify "$baseline_commit^{commit}") \
    || die_setup "'$baseline_commit' is not a commit in this repository"

# The make command line, as written to the log and the baseline file.
make_cmdline() {
    local extra=""
    [ ${#make_args[@]} -eq 0 ] || extra=" ${make_args[*]}"
    echo "$MAKE clean$extra && $MAKE APP_ROBOT=1$extra"
}

compiler_version() {
    arm-none-eabi-gcc --version 2>/dev/null | head -n 1 || true
}

# run_build DIR LOG: make clean && make APP_ROBOT=1 in DIR, output to the
# terminal and to LOG. Returns the make exit status.
run_build() {
    local dir=$1 log=$2 rev dirty=""
    rev=$(git -C "$dir" rev-parse HEAD 2>/dev/null || echo unknown)
    [ -z "$(git -C "$dir" status --porcelain --untracked-files=no 2>/dev/null)" ] || dirty=" (with uncommitted changes)"
    mkdir -p "$(dirname "$log")"
    {
        echo "# build_check $(date -u +%Y-%m-%dT%H:%M:%SZ) commit $rev$dirty"
        echo "# compiler: $(compiler_version)"
        echo "# cd $dir && $(make_cmdline)"
    } > "$log"
    ( cd "$dir" \
        && "$MAKE" clean ${make_args[@]+"${make_args[@]}"} \
        && "$MAKE" APP_ROBOT=1 ${make_args[@]+"${make_args[@]}"} ) 2>&1 | tee -a "$log"
}

# Compiler and linker warnings in a log: "file:line[:col]: warning: ...",
# "cc1: warning: ...", ".../ld: warning: ...". The script's own header lines
# and make's own warnings are not compiler warnings.
warning_lines() {
    grep -E '(^|: )warning: ' "$1" | grep -vE '^(# |make(\[[0-9]+\])?: )' || true
}

count_warnings() {
    warning_lines "$1" | wc -l | tr -d ' '
}

# ---------------------------------------------------------------- record mode
if [ "$record" -eq 1 ]; then
    tmp=$(mktemp -d "${TMPDIR:-/tmp}/build_check.XXXXXX") || die_setup "mktemp failed"
    wt="$tmp/baseline"
    cleanup() { git_ worktree remove --force "$wt" >/dev/null 2>&1; rm -rf "$tmp"; }
    trap cleanup EXIT
    echo "build_check: building the Baseline_Build $base_hash ($baseline_source) in $wt"
    git_ worktree add --detach "$wt" "$base_hash" >/dev/null || die_setup "git worktree add failed"
    # Local, git-ignored config headers (broker IP, Wi-Fi credentials) the
    # build needs; they are not in any commit.
    for f in "$REPO"/config/*.h; do
        [ -f "$f" ] || continue
        [ -e "$wt/config/$(basename "$f")" ] || cp -- "$f" "$wt/config/"
    done
    # The makefile's default SDK path is relative to build_make/; resolve it
    # from the real checkout so the temporary worktree finds the same SDK.
    if [ -z "${PICO_SDK_PATH:-}" ]; then
        PICO_SDK_PATH=$(abs_path "$BUILD_DIR/../../sdk/pico-sdk")
        export PICO_SDK_PATH
    fi
    base_log="${log_file%.log}_baseline.log"
    run_build "$wt/build_make" "$base_log"
    status=$?
    if [ "$status" -ne 0 ]; then
        echo "build_check: FAIL - the Baseline_Build did not build (make exit $status), log: $base_log" >&2
        exit 1
    fi
    n=$(count_warnings "$base_log")
    mkdir -p "$(dirname "$baseline_file")"
    {
        echo "# Baseline_Build warning count, written by tools/evidence/build_check.sh --record-baseline"
        echo "commit=$base_hash"
        echo "warnings=$n"
        echo "make=$(make_cmdline)"
        echo "compiler=$(compiler_version)"
        echo "date=$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    } > "$baseline_file"
    echo "build_check: Baseline_Build $base_hash has $n warning(s); recorded in $baseline_file"
    echo "build_check: log: $base_log"
    exit 0
fi

# ----------------------------------------------------------------- check mode
recorded_commit=""
if [ -z "$baseline_warnings" ]; then
    [ -f "$baseline_file" ] || die_setup "no Baseline_Build warning count: pass --baseline-warnings N, set BASELINE_WARNINGS, or run with --record-baseline to write $baseline_file"
    baseline_warnings=$(sed -n 's/^warnings=\([0-9][0-9]*\)[[:space:]]*$/\1/p' "$baseline_file" | head -n 1)
    [ -n "$baseline_warnings" ] || die_setup "$baseline_file has no warnings=N line"
    recorded_commit=$(sed -n 's/^commit=\([0-9a-fA-F]*\).*/\1/p' "$baseline_file" | head -n 1)
fi

run_build "$BUILD_DIR" "$log_file"
build_status=$?

n=$(count_warnings "$log_file")

# Files added or changed since the baseline commit, relative to the repo
# root. --ignore-cr-at-eol with the 0/0 filter drops files whose only
# difference is line endings (a Windows checkout built from WSL); binary
# files show "-" and are kept.
changed_files() {
    git_ diff --numstat --ignore-cr-at-eol --diff-filter=d "$base_hash" -- \
        | awk -F'\t' '$1 != "0" || $2 != "0" { print $3 }'
    git_ ls-files --others --exclude-standard
}
changed=$(changed_files | sort -u)

# Path as printed by the compiler (relative to build_make/ or absolute) ->
# path relative to the repo root ("../x" when outside the repo).
repo_path() {
    local p=$1
    case "$p" in
        /*) realpath -m --relative-to="$REPO" -- "$p" ;;
        *)  realpath -m --relative-to="$REPO" -- "$BUILD_DIR/$p" ;;
    esac
}

new_warnings=()
while IFS= read -r line; do
    file=$(printf '%s\n' "$line" | sed -nE 's/^([^:]+):[0-9]+(:[0-9]+)?: warning: .*/\1/p')
    [ -n "$file" ] || continue
    rel=$(repo_path "$file")
    if grep -qxF -- "$rel" <<<"$changed"; then
        new_warnings+=("$line")
    fi
done < <(warning_lines "$log_file")

fail=0
echo
echo "== build_check summary =="
echo "log:               $log_file"
echo "make:              cd build_make && $(make_cmdline)"
echo "baseline commit:   $base_hash ($baseline_source)"
if [ -n "$recorded_commit" ] && [ "$(git_ rev-parse -q --verify "$recorded_commit^{commit}" 2>/dev/null)" != "$base_hash" ]; then
    echo "NOTE: the recorded count in $baseline_file is for commit $recorded_commit, not $base_hash"
fi
if [ "$build_status" -ne 0 ]; then
    echo "FAIL: build failed (make exit status $build_status)"
    fail=1
else
    echo "build:             ok"
fi
echo "warnings:          $n (Baseline_Build: $baseline_warnings)"
if [ "$n" -gt "$baseline_warnings" ]; then
    echo "FAIL: $n warnings is more than the Baseline_Build's $baseline_warnings"
    fail=1
fi
echo "changed files:     $(grep -c . <<<"$changed" || true) since $(git_ rev-parse --short "$base_hash")"
if [ ${#new_warnings[@]} -gt 0 ]; then
    echo "FAIL: ${#new_warnings[@]} warning(s) in files added or changed since the baseline:"
    printf '  %s\n' "${new_warnings[@]}"
    fail=1
fi

if [ "$fail" -eq 0 ]; then
    echo "PASS"
    exit 0
fi
exit 1
