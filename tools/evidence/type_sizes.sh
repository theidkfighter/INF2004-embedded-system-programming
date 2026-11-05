#!/usr/bin/env bash
# Target sizes of the Inter_Task_Message bodies and the Message_Pool block
# (Requirement 23.1)
#
# Compiles tools/evidence/type_sizes.c with `arm-none-eabi-gcc -c` against
# the app_program/ headers of one build and prints, as a Markdown report,
# the target sizeof of each app_msg_body_t member type, of the union, and
# the Message_Pool block size, all read with `arm-none-eabi-nm -S`. With
# -e it also reads `g_mpf_buf` from the linked ELF and derives the block
# size from it, as a cross-check.
#
# Exit status: 0 report printed; 1 the ELF block size differs from the
# header block size; 2 usage, setup or compile error.
#
# Usage:
#   tools/evidence/type_sizes.sh [options]
#   e.g. tools/evidence/type_sizes.sh -c <baseline-commit> > types_baseline.md
#        tools/evidence/type_sizes.sh -e build_make/is02_robot_smp0.elf > types_revised.md
#
# Options:
#   -c COMMIT     use the app_program/ headers of COMMIT (extracted with
#                 git archive into a temporary folder)
#   -d DIR        use the headers in DIR. Default: app_program/ of this
#                 working tree
#   -e ELF        also read g_mpf_buf from ELF (the same build as the
#                 headers) and derive the block size from it
#   -n BLOCKS     Message_Pool block count. Default: MSG_BLOCKS in
#                 os_glue.c of the chosen headers
#   -h, --help    Show this text.
#
# Set CROSS to use another tool prefix (default arm-none-eabi-), TS_CFLAGS
# to replace the target flags (default -mcpu=cortex-m0plus -mthumb, as in
# build_make/pico_rp2040.mk).

set -u
set -o pipefail
export LC_ALL=C

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO=$(git -C "$SCRIPT_DIR" rev-parse --show-toplevel 2>/dev/null || (cd "$SCRIPT_DIR/../.." && pwd))
CROSS=${CROSS-arm-none-eabi-}
GCC="${CROSS}gcc"
NM="${CROSS}nm"
TS_CFLAGS=${TS_CFLAGS:--mcpu=cortex-m0plus -mthumb}

usage() { sed -n '2,/^$/s/^# \{0,1\}//p' "${BASH_SOURCE[0]}"; }
die_usage() { echo "type_sizes: $1" >&2; usage >&2; exit 2; }
die_setup() { echo "type_sizes: $1" >&2; exit 2; }

commit=""
app_dir=""
elf=""
blocks=""

while [ $# -gt 0 ]; do
    case "$1" in
        -c) [ $# -ge 2 ] || die_usage "$1 needs a value"; commit=$2; shift 2 ;;
        -d) [ $# -ge 2 ] || die_usage "$1 needs a value"; app_dir=$2; shift 2 ;;
        -e) [ $# -ge 2 ] || die_usage "$1 needs a value"; elf=$2; shift 2 ;;
        -n) [ $# -ge 2 ] || die_usage "$1 needs a value"; blocks=$2; shift 2 ;;
        -h|--help) usage; exit 0 ;;
        *) die_usage "unknown argument: $1" ;;
    esac
done
[ -z "$commit" ] || [ -z "$app_dir" ] || die_usage "-c and -d cannot be used together"
case "$blocks" in
    ''|*[!0-9]*) [ -z "$blocks" ] || die_usage "-n needs a whole number" ;;
esac
command -v "$GCC" >/dev/null 2>&1 || die_setup "$GCC not found on PATH"
command -v "$NM" >/dev/null 2>&1 || die_setup "$NM not found on PATH"
[ -z "$elf" ] || [ -f "$elf" ] || die_setup "ELF not found: $elf"

tmp=$(mktemp -d "${TMPDIR:-/tmp}/type_sizes.XXXXXX") || die_setup "mktemp failed"
trap 'rm -rf "$tmp"' EXIT

if [ -n "$commit" ]; then
    hash=$(git -C "$REPO" rev-parse -q --verify "$commit^{commit}") || die_setup "'$commit' is not a commit"
    git -C "$REPO" archive "$hash" app_program | tar -x -C "$tmp" || die_setup "git archive $hash app_program failed"
    app_dir="$tmp/app_program"
    source_desc="\`app_program/\` at commit \`$hash\`"
else
    [ -n "$app_dir" ] || app_dir="$REPO/app_program"
    [ -d "$app_dir" ] || die_setup "header folder not found: $app_dir"
    app_dir=$(cd "$app_dir" && pwd)
    if hash=$(git -C "$app_dir" rev-parse HEAD 2>/dev/null); then
        changes=$(git -C "$app_dir" status --porcelain -- . 2>/dev/null | wc -l | tr -d ' ')
        state="clean"
        [ "$changes" -eq 0 ] || state="$changes uncommitted or untracked path(s)"
        source_desc="\`$app_dir\` (working tree at \`$hash\`, $state)"
    else
        source_desc="\`$app_dir\`"
    fi
fi
[ -f "$app_dir/app_types.h" ] || die_setup "no app_types.h in $app_dir"

if [ -z "$blocks" ]; then
    blocks=$(sed -nE 's/^[[:space:]]*#[[:space:]]*define[[:space:]]+MSG_BLOCKS[[:space:]]+\(?([0-9]+)u?\)?.*/\1/p' \
             "$app_dir/os_glue.c" 2>/dev/null | head -n 1)
fi

# ------------------------------------------------------------------ compile
obj="$tmp/type_sizes.o"
# shellcheck disable=SC2086  # TS_CFLAGS is a list of flags
cflags=($TS_CFLAGS -std=gnu11 -O2 -ffreestanding -fno-common
        -DAPP_PERF=0 -DAPP_SELFTEST=0
        -I "$app_dir")
if ! "$GCC" "${cflags[@]}" -c "$SCRIPT_DIR/type_sizes.c" -o "$obj" 2>"$tmp/cc.log"; then
    cat "$tmp/cc.log" >&2
    die_setup "compiling type_sizes.c against $app_dir failed"
fi

# nm -S: "addr size type name"; names ts_<label>__<type>
ts_rows=$("$NM" -S "$obj" | awk 'NF == 4 && $4 ~ /^ts_/ { print $4, $2 }')
[ -n "$ts_rows" ] || die_setup "no ts_ symbols in $obj"

ts_size() {   # ts_size LABEL -> "type bytes"
    local name hex
    while read -r name hex; do
        local rest=${name#ts_}
        if [ "${rest%%__*}" = "$1" ]; then
            echo "${rest#*__} $((16#$hex))"
            return 0
        fi
    done <<<"$ts_rows"
    return 1
}

build="Baseline_Build (no msg_codec.h)"
[ ! -f "$app_dir/msg_codec.h" ] || build="Revised_Build (msg_codec.h present)"
gcc_version=$("$GCC" --version | head -n 1)

rows=""
for m in motion nav obstacle event; do
    read -r type bytes < <(ts_size "$m") || die_setup "no size for member $m"
    rows+="| \`$m\` | \`$type\` | $bytes |"$'\n'
done
read -r _ union_b < <(ts_size union)
read -r _ tmsg_b < <(ts_size tmsg)
read -r _ block_b < <(ts_size block)

cat <<EOF
# Inter_Task_Message sizes (target)

| Record | Value |
|---|---|
| Headers | $source_desc |
| Build | $build |
| Compiler (\`$GCC --version\`) | $gcc_version |
| Flags | \`${cflags[*]}\` |
| Size source | \`$NM -S\` on \`type_sizes.o\` |

## Message body types (Requirement 23.1)

| \`app_msg_body_t\` member | Body type | sizeof (bytes) |
|---|---|---|
${rows%$'\n'}
| (union) | \`app_msg_body_t\` | $union_b |

## Message_Pool block

| Item | Bytes |
|---|---|
| \`T_MSG\` header | $tmsg_b |
| Block size: \`(sizeof(T_MSG) + sizeof(app_msg_body_t) + 3) / 4 * 4\` (\`MSG_BLK_WORDS * 4\` in os_glue.c) | $block_b |
EOF

status=0
if [ -n "$elf" ]; then
    mpf_hex=$("$NM" -S "$elf" | awk 'NF == 4 && $4 == "g_mpf_buf" && $3 ~ /^[bBdD]$/ { print $2; exit }')
    if [ -z "$mpf_hex" ]; then
        echo "| \`g_mpf_buf\` in \`$(basename "$elf")\` | not found |"
        status=1
    elif [ -z "$blocks" ] || [ "$blocks" -eq 0 ]; then
        echo "| \`g_mpf_buf\` in \`$(basename "$elf")\` (\`$NM -S\`) | $((16#$mpf_hex)) |"
        echo "| Block size from the ELF | MSG_BLOCKS unknown: pass -n |"
        status=1
    else
        mpf_b=$((16#$mpf_hex))
        # os_glue.c: UW g_mpf_buf[(MSG_BLOCKS * MSG_BLK_WORDS) + MSG_BLOCKS]
        elf_block=$((mpf_b / blocks - 4))
        echo "| \`g_mpf_buf\` in \`$(basename "$elf")\` (\`$NM -S\`) | $mpf_b |"
        echo "| MSG_BLOCKS | $blocks |"
        echo "| Block size from the ELF: \`g_mpf_buf\` bytes / MSG_BLOCKS - 4 (buffer is MSG_BLOCKS x (MSG_BLK_WORDS + 1) words) | $elf_block |"
        if [ "$elf_block" -ne "$block_b" ] || [ $((mpf_b % blocks)) -ne 0 ]; then
            echo
            echo "**Mismatch:** the ELF block size ($elf_block) differs from the header block size ($block_b). Check that the ELF was built from the same headers."
            status=1
        fi
    fi
fi
exit "$status"
