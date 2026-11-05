#!/usr/bin/env bash
################################################################################
# IS-02 Host_Test_Suite: static structure checks (run in WSL or Linux).
#
#   bash tests/host/test_static.sh
#
# Checks the app_program/ sources without building them:
#
#   pure_logic:<module>   The pure-logic modules (line_logic, imu_logic,
#                         obstacle_logic, barcode_logic, bc_detect, bc_multi,
#                         code39, nav_map, speed_ctl) exist, and neither their
#                         .c nor their .h includes, directly or through another
#                         app_program header, hal.h, hal_internal.h, pins.h,
#                         tk/tkernel.h or a Pico SDK header (Requirement 19.3).
#   plain_header:<name>   Each packed-field accessor header (bitpack.h, crit.h,
#                         fault_mask.h, status.h, shared.h, msg_codec.h, evt.h,
#                         wire_ids.h, wire.h, perf.h, isr_ring.h) includes only C standard
#                         headers and app_program headers that in turn include
#                         no micro T-Kernel or Pico SDK header (checked
#                         recursively), and compiles on its own with the host
#                         compiler for APP_PERF=0 and APP_PERF=1
#                         (Requirement 9.5). A header that does not exist yet
#                         is reported as skipped, not failed.
#   no_dynamic_alloc      No call to malloc, calloc, realloc or free in
#                         app_program/ (Requirement 19.6).
#   memory_pools          tk_cre_mpf is called exactly as often as in the
#                         Baseline_Build (once: the 40-block Message_Pool) and
#                         tk_cre_mpl is never called (Requirements 19.2, 19.6).
#   tasks                 Exactly seven start_task calls, one per baseline task,
#                         with the baseline priorities 2, 4, 5, 6, 7, 8, 10, and
#                         tk_cre_tsk called only inside start_task
#                         (Requirement 19.1).
#   evf_bits              The 18 EVF_* bits of g_flg_sys are defined at the
#                         baseline positions (bit 0 EVF_COLLISION to bit 17
#                         EVF_LOCAL_STOP) and no other single-bit EVF_* macro
#                         exists (Requirement 19.2).
#
# Comments are removed before any check, and string literals are removed before
# the call checks, so text in comments or log messages never matches.
#
# Prints one PASS / FAIL / SKIP line per check and, when IS02_RESULTS is set,
# writes one JSON-lines record per check to that file for summary.py
# (kind "test", runner "static"). Exit status 0 when nothing failed, 1 when a
# check failed, 2 when the script could not run.
#
# Environment: APP_DIR  (default ../../app_program from this script)
#              HOST_CC  (default gcc; the compile part of plain_header is
#                        skipped with a note when it is not installed)
#              IS02_RESULTS (optional JSON-lines result file)
#
# Baseline values below are taken from the Baseline_Build, commit
# bca79643d6f9bdd3938c353fa5f6fe791fc68f94 (see docs/evidence/README.md):
# app_program/os_glue.h (PRI_*, EVF_*) and app_program/os_glue.c
# (start_task calls, one tk_cre_mpf, one tk_cre_tsk).
################################################################################
set -u

HERE="$(cd "$(dirname "$0")" && pwd)"
APP="${APP_DIR:-$HERE/../../app_program}"
HOST_CC="${HOST_CC:-gcc}"
RESULTS="${IS02_RESULTS:-}"

# ---- Baseline_Build values ---------------------------------------------------
PURE_LOGIC=(line_logic imu_logic obstacle_logic barcode_logic bc_detect bc_multi
            code39 nav_map speed_ctl)
ACCESSOR_HEADERS=(bitpack.h crit.h fault_mask.h status.h shared.h msg_codec.h
                  evt.h wire_ids.h wire.h perf.h isr_ring.h)

# EVF_* names in bit order: index = bit position in g_flg_sys.
EVF_BASE=(EVF_COLLISION EVF_PROXIMITY EVF_REMOTE_ESTOP EVF_BATTERY_CRITICAL
          EVF_BUZZER_REQ EVF_ESTOP EVF_MOTION_DONE EVF_LINE_FOUND EVF_LINE_LOST
          EVF_OBSTACLE_TRIGGER EVF_BYPASS_COMPLETE EVF_HUMP_START EVF_HUMP_PEAK
          EVF_HUMP_END EVF_IMU_FAULT EVF_BYPASS_ABORT EVF_ERROR_REQ EVF_LOCAL_STOP)

# Task name -> priority (smaller number = higher priority).
declare -A TASK_PRI=(
    [TSK_SAFETY_MON]=2 [TSK_MOTION_CTRL]=4 [TSK_LINE_BARCODE]=5
    [TSK_OBSTACLE_SCAN]=6 [TSK_TERRAIN_IMU]=7 [TSK_NAV_PLANNER]=8
    [TSK_TELEMETRY]=10
)
TASK_ORDER=(TSK_SAFETY_MON TSK_MOTION_CTRL TSK_LINE_BARCODE TSK_OBSTACLE_SCAN
            TSK_TERRAIN_IMU TSK_NAV_PLANNER TSK_TELEMETRY)

BASE_MPF=1      # tk_cre_mpf calls (the 40-block Message_Pool)
BASE_MPL=0      # tk_cre_mpl calls
BASE_TSK=1      # tk_cre_tsk calls (inside start_task)

# Host compile flags: the firmware warning flags, as in tests/host/Makefile.
WARN=(-Wall -Wextra -Wshadow -Wdouble-promotion -Wno-unused-parameter -Werror)

# ---- set-up ------------------------------------------------------------------
if [ ! -d "$APP" ]; then
    echo "test_static.sh: app_program directory not found: $APP" >&2
    exit 2
fi
APP="$(cd "$APP" && pwd)"
if ! command -v perl > /dev/null 2>&1; then
    echo "test_static.sh: perl is required (sudo apt install perl)" >&2
    exit 2
fi
if [ -n "$RESULTS" ]; then
    : > "$RESULTS" || { echo "test_static.sh: cannot write $RESULTS" >&2; exit 2; }
fi

TMP="$(mktemp -d)" || exit 2
trap 'rm -rf "$TMP"' EXIT
mkdir -p "$TMP/str" "$TMP/nostr"

# strip_src <file> <keep_strings 0|1>: the file with comments removed (each
# comment becomes a space, newlines kept so line numbers stay right). With
# keep_strings=0, string literals become "" and character literals become 0.
strip_src() {
    KEEP_STR="$2" perl -0777 -pe '
        s{(/\*.*?\*/)|(//[^\n]*)|("(?:\\.|[^"\\\n])*")|(\x27(?:\\.|[^\x27\\\n])*\x27)}{
            defined $1 ? do { (my $t = $1) =~ s/[^\n]//g; " $t" }
          : defined $2 ? " "
          : $ENV{KEEP_STR} ? (defined $3 ? $3 : $4)
          : (defined $3 ? q("") : q(0))
        }gse' -- "$1"
}

SRC_NAMES=()
for f in "$APP"/*.c "$APP"/*.h; do
    [ -f "$f" ] || continue
    n="$(basename "$f")"
    SRC_NAMES+=("$n")
    strip_src "$f" 1 > "$TMP/str/$n"
    strip_src "$f" 0 > "$TMP/nostr/$n"
done
if [ "${#SRC_NAMES[@]}" -eq 0 ]; then
    echo "test_static.sh: no .c or .h files in $APP" >&2
    exit 2
fi

# ---- result reporting --------------------------------------------------------
N_PASS=0
N_FAIL=0
N_SKIP=0
PROBLEMS=""

add_problem() {
    if [ -z "$PROBLEMS" ]; then PROBLEMS="$1"; else PROBLEMS="$PROBLEMS"$'\n'"$1"; fi
}

json_str() {   # <text>: the text as a JSON string
    if [ -z "$1" ]; then printf '""'; return; fi
    printf '%s' "$1" | perl -0777 -ne '
        s/\\/\\\\/g; s/"/\\"/g; s/\n/\\n/g; s/\t/\\t/g;
        s/([\x00-\x1f])/sprintf("\\u%04x", ord($1))/ge;
        print "\"$_\""'
}

emit() {   # <check> <pass|fail|skip> [detail]
    local name="$1" st="$2" detail="${3:-}" failed=false skipped=false input=null
    case "$st" in
        pass) echo "PASS $name"; N_PASS=$((N_PASS + 1)) ;;
        fail) echo "FAIL $name"
              printf '%s\n' "$detail" | sed 's/^/    /'
              N_FAIL=$((N_FAIL + 1)); failed=true; input="$(json_str "$detail")" ;;
        skip) echo "SKIP $name ($detail)"; N_SKIP=$((N_SKIP + 1)); skipped=true ;;
    esac
    if [ -n "$RESULTS" ]; then
        printf '{"runner": "static", "kind": "test", "n": null, "title": %s, "unit": null, "cases": 1, "failed": %s, "skipped": %s, "seed": null, "phase": "call", "nodeid": %s, "input": %s}\n' \
            "$(json_str "$name")" "$failed" "$skipped" \
            "$(json_str "test_static.sh::$name")" "$input" >> "$RESULTS"
    fi
}

finish() {   # <check>: report PASS or FAIL from PROBLEMS, then clear it
    if [ -z "$PROBLEMS" ]; then emit "$1" pass; else emit "$1" fail "$PROBLEMS"; fi
    PROBLEMS=""
}

# ---- include analysis --------------------------------------------------------
is_std_header() {   # C11 standard library headers
    case "$1" in
        assert.h|complex.h|ctype.h|errno.h|fenv.h|float.h|inttypes.h|iso646.h|\
        limits.h|locale.h|math.h|setjmp.h|signal.h|stdalign.h|stdarg.h|\
        stdatomic.h|stdbool.h|stddef.h|stdint.h|stdio.h|stdlib.h|stdnoreturn.h|\
        string.h|tgmath.h|threads.h|time.h|uchar.h|wchar.h|wctype.h) return 0 ;;
    esac
    return 1
}

# Hardware access headers: forbidden only in the pure-logic modules
# (Requirement 19.3). hal.h itself is plain, so an accessor header may use it
# (Requirement 9.5 forbids only micro T-Kernel and Pico SDK headers there).
FORBID_HW=1

is_forbidden_header() {   # micro T-Kernel and Pico SDK headers (+ hardware access when FORBID_HW=1)
    case "$1" in
        tk/*|tm/*|pico.h|pico/*|hardware/*|cyw43*|lwip/*|boards/*|tusb*) return 0 ;;
        hal.h|hal_internal.h|pins.h) [ "$FORBID_HW" -eq 1 ] && return 0 ;;
    esac
    return 1
}

includes_of() {   # <name in app_program>: one #include operand per line
    sed -nE 's/^[[:space:]]*#[[:space:]]*include[[:space:]]*//p' "$TMP/str/$1" |
        sed -E 's/[[:space:]]+$//'
}

declare -A SEEN=()

# plain_walk <name in app_program> <include chain>: adds a problem for every
# #include, direct or through app_program headers, that is not a C standard
# header or a plain app_program header.
plain_walk() {
    local rel="$1" via="$2" inc name kind
    [ -n "${SEEN[$rel]:-}" ] && return
    SEEN[$rel]=1
    while IFS= read -r inc; do
        case "$inc" in
            \<*\>) name="${inc#<}"; name="${name%>}"; kind=angle ;;
            \"*\") name="${inc#\"}"; name="${name%\"}"; kind=quote ;;
            *) add_problem "$via$rel: #include $inc cannot be checked (not <...> or \"...\")"
               continue ;;
        esac
        if is_forbidden_header "$name"; then
            if [ "$FORBID_HW" -eq 1 ]; then
                add_problem "$via$rel includes $inc (hardware access, micro T-Kernel or Pico SDK header)"
            else
                add_problem "$via$rel includes $inc (micro T-Kernel or Pico SDK header)"
            fi
        elif [ "$kind" = quote ] && [ -f "$APP/$name" ]; then
            plain_walk "$name" "$via$rel -> "
        elif is_std_header "$name"; then
            :
        else
            add_problem "$via$rel includes $inc, which is neither a C standard header nor a plain app_program header"
        fi
    done < <(includes_of "$rel")
}

# ---- 1. pure-logic modules (Requirement 19.3) --------------------------------
FORBID_HW=1
for m in "${PURE_LOGIC[@]}"; do
    SEEN=()
    for f in "$m.c" "$m.h"; do
        if [ ! -f "$APP/$f" ]; then
            add_problem "$f is missing (every Baseline_Build module must stay)"
        else
            plain_walk "$f" ""
        fi
    done
    finish "pure_logic:$m"
done

# ---- 2. plain accessor headers (Requirement 9.5) -----------------------------
HAVE_CC=0
command -v "$HOST_CC" > /dev/null 2>&1 && HAVE_CC=1
CC_NOTE_SHOWN=0
FORBID_HW=0
for h in "${ACCESSOR_HEADERS[@]}"; do
    if [ ! -f "$APP/$h" ]; then
        emit "plain_header:$h" skip "not created yet"
        continue
    fi
    SEEN=()
    plain_walk "$h" ""
    if [ "$HAVE_CC" -eq 1 ]; then
        for p in 0 1; do
            if ! out="$(printf '#include "%s"\n' "$h" |
                        "$HOST_CC" -std=gnu11 -fsyntax-only "${WARN[@]}" \
                            -DAPP_PERF="$p" -DAPP_SELFTEST=0 \
                            -I"$APP" -x c - 2>&1)"; then
                add_problem "$h does not compile on its own with $HOST_CC (APP_PERF=$p):"$'\n'"$out"
            fi
        done
    elif [ "$CC_NOTE_SHOWN" -eq 0 ]; then
        echo "note: $HOST_CC not found, plain_header checks only the #include lines"
        CC_NOTE_SHOWN=1
    fi
    finish "plain_header:$h"
done

# ---- call scanning -----------------------------------------------------------
# calls_of <identifier>: "<file>:<line>: <text>" for each call in app_program/
# (comments and string literals removed).
calls_of() {
    local n
    for n in "${SRC_NAMES[@]}"; do
        grep -nE "(^|[^[:alnum:]_])$1[[:space:]]*\(" "$TMP/nostr/$n" |
            sed -E "s/^([0-9]+):[[:space:]]*/$n:\1: /"
    done
}

count_calls() {   # <identifier>: number of calls in app_program/
    local n total=0 c
    for n in "${SRC_NAMES[@]}"; do
        c=$(grep -oE "(^|[^[:alnum:]_])$1[[:space:]]*\(" "$TMP/nostr/$n" | wc -l)
        total=$((total + c))
    done
    echo "$total"
}

# ---- 3. no dynamic allocation (Requirement 19.6) -----------------------------
for fn in malloc calloc realloc free; do
    hits="$(calls_of "$fn")"
    [ -n "$hits" ] && add_problem "call to $fn:"$'\n'"$hits"
done
finish "no_dynamic_alloc"

# ---- 4. kernel memory pools (Requirements 19.2, 19.6) ------------------------
c=$(count_calls tk_cre_mpf)
if [ "$c" -ne "$BASE_MPF" ]; then
    add_problem "tk_cre_mpf is called $c time(s), the Baseline_Build calls it $BASE_MPF time(s):"$'\n'"$(calls_of tk_cre_mpf)"
fi
c=$(count_calls tk_cre_mpl)
if [ "$c" -ne "$BASE_MPL" ]; then
    add_problem "tk_cre_mpl is called $c time(s), the Baseline_Build calls it $BASE_MPL time(s):"$'\n'"$(calls_of tk_cre_mpl)"
fi
finish "memory_pools"

# ---- 5. tasks and priorities (Requirement 19.1) ------------------------------
# resolve_pri <token>: numeric priority of a start_task argument (a number or a
# macro defined as a number in app_program/), or nothing.
resolve_pri() {
    local tok="$1" vals
    tok="$(printf '%s' "$tok" | tr -d ' ()')"
    case "$tok" in
        ''|*[!0-9]*) ;;
        *) echo "$tok"; return ;;
    esac
    case "$tok" in
        [A-Za-z_]*) ;;
        *) return ;;
    esac
    case "$tok" in
        *[!A-Za-z0-9_]*) return ;;
    esac
    vals="$(cd "$TMP/str" && cat -- "${SRC_NAMES[@]}" |
            sed -nE "s/^[[:space:]]*#[[:space:]]*define[[:space:]]+$tok[[:space:]]+\(?[[:space:]]*([0-9]+)[[:space:]]*\)?[[:space:]]*$/\1/p" |
            sort -u)"
    [ "$(printf '%s\n' "$vals" | grep -c .)" -eq 1 ] && echo "$vals"
}

# Calls whose first argument is a string literal: "<name>\t<priority arg>".
CALLS="$(cd "$TMP/str" && perl -0777 -ne '
    while (/(?<![A-Za-z0-9_])start_task\s*\(\s*"([^"]*)"\s*,([^;{]*)\)\s*;/gs) {
        my ($name, $rest) = ($1, $2);
        my @a = split /,/, $rest;
        my $pri = defined $a[1] ? $a[1] : "";
        $pri =~ s/^\s+|\s+$//g;
        print "$name\t$pri\n";
    }' -- "${SRC_NAMES[@]}")"

# All calls, without the definition and prototypes (their first parameter is
# "const char *").
TOTAL_CALLS=$(cd "$TMP/nostr" && cat -- "${SRC_NAMES[@]}" |
              perl -0777 -ne '$n = () = /(?<![A-Za-z0-9_])start_task\s*\((?!\s*const\b)/g; print $n + 0')
PARSED_CALLS=$(printf '%s\n' "$CALLS" | grep -c .)

if [ "$TOTAL_CALLS" -ne 7 ]; then
    add_problem "found $TOTAL_CALLS start_task calls, the Baseline_Build has 7"
fi
if [ "$PARSED_CALLS" -ne "$TOTAL_CALLS" ]; then
    add_problem "$((TOTAL_CALLS - PARSED_CALLS)) start_task call(s) do not name the task with a string literal"
fi
declare -A FOUND=()
while IFS=$'\t' read -r tname targ; do
    [ -n "$tname" ] || continue
    if [ -n "${FOUND[$tname]:-}" ]; then
        add_problem "task $tname is started more than once"
        continue
    fi
    FOUND[$tname]=1
    if [ -z "${TASK_PRI[$tname]:-}" ]; then
        add_problem "start_task(\"$tname\") is not a Baseline_Build task"
        continue
    fi
    pri="$(resolve_pri "$targ")"
    if [ -z "$pri" ]; then
        add_problem "priority '$targ' of $tname is not a number or a macro defined once as a number"
    elif [ "$pri" -ne "${TASK_PRI[$tname]}" ]; then
        add_problem "$tname has priority $pri ($targ), the Baseline_Build uses ${TASK_PRI[$tname]}"
    fi
done <<< "$CALLS"
for t in "${TASK_ORDER[@]}"; do
    [ -n "${FOUND[$t]:-}" ] || add_problem "no start_task call for $t (priority ${TASK_PRI[$t]})"
done
c=$(count_calls tk_cre_tsk)
if [ "$c" -ne "$BASE_TSK" ]; then
    add_problem "tk_cre_tsk is called $c time(s), the Baseline_Build calls it $BASE_TSK time (inside start_task):"$'\n'"$(calls_of tk_cre_tsk)"
fi
finish "tasks"

# ---- 6. EVF_* bits (Requirement 19.2) ----------------------------------------
# "<name>\t<bit position or ->\t<file>" for every #define EVF_*; "-" when the
# value is not a single bit (a mask built from other EVF_* names, for example).
EVF_DEFS="$(cd "$TMP/str" && perl -ne '
    if (/^\s*#\s*define\s+(EVF_\w+)\s+(.*?)\s*$/) {
        my ($n, $v, $pos) = ($1, $2, "-");
        if ($v =~ /^\(?\s*1[uUlL]*\s*<<\s*(\d+)[uUlL]*\s*\)?$/) {
            $pos = $1;
        } elsif ($v =~ /^\(?\s*(0[xX][0-9a-fA-F]+|\d+)[uUlL]*\s*\)?$/) {
            my $lit = $1;
            my $x = ($lit =~ /^0[xX]/) ? hex($lit) : $lit;
            if ($x > 0 && ($x & ($x - 1)) == 0) { $pos = 0; $pos++ while ($x >>= 1); }
        }
        print "$n\t$pos\t$ARGV\n";
    }' -- "${SRC_NAMES[@]}")"

declare -A EVF_POS=()
for i in "${!EVF_BASE[@]}"; do EVF_POS[${EVF_BASE[$i]}]=$i; done
declare -A EVF_SEEN=()
while IFS=$'\t' read -r ename epos efile; do
    [ -n "$ename" ] || continue
    [ "$epos" = "-" ] && continue
    if [ -z "${EVF_POS[$ename]:-}" ]; then
        add_problem "$efile defines $ename at bit $epos, which is not a Baseline_Build EVF_* bit"
    elif [ "$epos" -ne "${EVF_POS[$ename]}" ]; then
        add_problem "$efile defines $ename at bit $epos, the Baseline_Build uses bit ${EVF_POS[$ename]}"
    fi
    EVF_SEEN[$ename]=1
done <<< "$EVF_DEFS"
for i in "${!EVF_BASE[@]}"; do
    [ -n "${EVF_SEEN[${EVF_BASE[$i]}]:-}" ] ||
        add_problem "${EVF_BASE[$i]} (bit $i) is not defined as a single bit"
done
finish "evf_bits"

# ---- summary -----------------------------------------------------------------
echo "static checks: passed=$N_PASS failed=$N_FAIL skipped=$N_SKIP"
[ "$N_FAIL" -eq 0 ]
