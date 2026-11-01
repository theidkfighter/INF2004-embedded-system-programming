#!/usr/bin/env bash
# Static RAM report for one firmware build (Requirement 22)
#
# Prints a Markdown report for the Evidence_Report:
#   - the build record: git commit (and whether the working tree had
#     uncommitted changes), the make command line, the arm-none-eabi-gcc
#     version and the robot options in build_make/.robot_opts
#   - the unedited `arm-none-eabi-size` output of the ELF, plus the
#     per-section `arm-none-eabi-size -A` output
#   - .data, .bss and their sum in bytes (the data and bss columns of the
#     default `arm-none-eabi-size` output)
#   - the RAM size of each Requirement 22.3 item from `arm-none-eabi-nm -S`,
#     with the input-section size from the linker map file next to it
#
# Run it once per build (Baseline_Build and Revised_Build, each after
# `make clean` at a clean commit) and put both reports in the
# Evidence_Report; the saving is the Baseline_Build value minus the
# Revised_Build value.
#
# Exit status: 0 report complete; 1 report printed but an item symbol was
# not found in the ELF (give its replacement symbols with -r); 2 usage or
# setup error.
#
# Usage:
#   tools/evidence/ram_report.sh [options] <elf> <map>
#   e.g. tools/evidence/ram_report.sh -m "make APP_ROBOT=1" \
#        build_make/is02_robot_smp0.elf build_make/is02_robot_smp0.map > ram_revised.md
#
# Options:
#   -m CMDLINE        make command line used for the build, as recorded in
#                     the report (Requirement 22.4). Default: $MAKE_CMDLINE,
#                     else "not recorded".
#   -r ITEM=SYM[,SYM...]
#                     Revised_Build symbols that replace a renamed, split or
#                     merged item (Requirement 22.6); their sizes are summed.
#                     ITEM is one of g_status g_mpf_buf g_frames g_msgs
#                     g_bc_ring g_raw g_cmds g_params. Repeat for each item.
#                     Example: -r g_status=g_st_motion,g_st_line
#   -h, --help        Show this text.
#
# Set CROSS to use another tool prefix (default arm-none-eabi-).

set -u
set -o pipefail
export LC_ALL=C

CROSS=${CROSS-arm-none-eabi-}
NM="${CROSS}nm"
SIZE="${CROSS}size"
GCC="${CROSS}gcc"

usage() { sed -n '2,/^$/s/^# \{0,1\}//p' "${BASH_SOURCE[0]}"; }
die_usage() { echo "ram_report: $1" >&2; usage >&2; exit 2; }
die_setup() { echo "ram_report: $1" >&2; exit 2; }

# Requirement 22.3 items: key, description, default symbols
ITEMS=(g_status g_mpf_buf g_frames g_msgs g_bc_ring g_raw g_cmds g_params)
declare -A DESC=(
    [g_status]="Status_Snapshot"
    [g_mpf_buf]="Message_Pool buffer"
    [g_frames]="Frame_Ring"
    [g_msgs]="message slots"
    [g_bc_ring]="barcode edge ring"
    [g_raw]="raw sample ring"
    [g_cmds]="command slots"
    [g_params]="Parameter_Store"
)
declare -A SYMS=(
    [g_status]="g_status"
    [g_mpf_buf]="g_mpf_buf"
    [g_frames]="g_frames"
    [g_msgs]="g_msgs"
    [g_bc_ring]="g_bc_ring"
    [g_raw]="g_raw_er g_raw_el g_raw_adc"
    [g_cmds]="g_cmds"
    [g_params]="g_params"
)
declare -A REPLACED=()

make_cmdline=${MAKE_CMDLINE:-}

while [ $# -gt 0 ]; do
    case "$1" in
        -m)
            [ $# -ge 2 ] || die_usage "$1 needs a value"
            make_cmdline=$2; shift 2 ;;
        -r)
            [ $# -ge 2 ] || die_usage "$1 needs a value"
            key=${2%%=*}
            list=${2#*=}
            [ "$key" != "$2" ] && [ -n "$list" ] || die_usage "-r needs ITEM=SYM[,SYM...], got '$2'"
            [ -n "${DESC[$key]+x}" ] || die_usage "unknown item '$key' (use one of: ${ITEMS[*]})"
            SYMS[$key]=${list//,/ }
            REPLACED[$key]=1
            shift 2 ;;
        -h|--help)
            usage; exit 0 ;;
        -*)
            die_usage "unknown option: $1" ;;
        *)
            break ;;
    esac
done

[ $# -eq 2 ] || die_usage "need <elf> and <map>"
elf=$1
map=$2
[ -f "$elf" ] || die_setup "ELF not found: $elf"
[ -f "$map" ] || die_setup "map file not found: $map"
command -v "$NM" >/dev/null 2>&1 || die_setup "$NM not found on PATH"
command -v "$SIZE" >/dev/null 2>&1 || die_setup "$SIZE not found on PATH"

if [ -z "$make_cmdline" ]; then
    make_cmdline="not recorded (pass -m)"
    echo "ram_report: warning: no make command line given; pass -m \"make APP_ROBOT=1 ...\" (Requirement 22.4)" >&2
fi

# ------------------------------------------------------------- build record
elf_dir=$(cd "$(dirname "$elf")" && pwd)
if commit=$(git -C "$elf_dir" rev-parse HEAD 2>/dev/null); then
    changes=$(git -C "$elf_dir" status --porcelain 2>/dev/null | wc -l | tr -d ' ')
    if [ "$changes" -eq 0 ]; then
        tree="clean"
    else
        tree="$changes uncommitted or untracked path(s): not a valid evidence build (Requirement 22.7)"
    fi
else
    commit="unknown (ELF is not inside a git work tree)"
    tree="unknown"
fi
gcc_version=$("$GCC" --version 2>/dev/null | head -n 1)
[ -n "$gcc_version" ] || gcc_version="unknown ($GCC not found)"
robot_opts=$(cat "$elf_dir/.robot_opts" 2>/dev/null || echo "not found")

size_out=$("$SIZE" "$elf") || die_setup "$SIZE failed on $elf"
size_a_out=$("$SIZE" -A "$elf") || die_setup "$SIZE -A failed on $elf"
# Berkeley format: text data bss dec hex filename
read -r data_b bss_b < <(awk 'NR == 2 { print $2, $3 }' <<<"$size_out")
case "$data_b$bss_b" in
    ''|*[!0-9]*) die_setup "cannot read the data and bss columns from: $size_out" ;;
esac
sum_b=$((data_b + bss_b))

# ------------------------------------------------------------ symbol sizes
# nm -S: "addr size type name"; RAM objects have type b/B (bss) or d/D (data)
nm_out=$("$NM" -S "$elf") || die_setup "$NM -S failed on $elf"

# Hex parsing is done in bash: the awk on the build host may be mawk, which
# has no strtonum().

# nm_size SYM -> "total count" over every RAM symbol called SYM
nm_size() {
    local t=0 n=0 hex
    while read -r hex; do
        t=$((t + 16#$hex))
        n=$((n + 1))
    done < <(awk -v s="$1" 'NF == 4 && $4 == s && $3 ~ /^[bBdD]$/ { print $2 }' <<<"$nm_out")
    echo "$t $n"
}

# Input sections ".bss.SYM" / ".data.SYM" in the memory map part of the map
# file (after "Linker script and memory map", so discarded sections are not
# counted). GNU ld puts address, size and object on the next line when the
# section name is long. Prints "total object..." or "-" when there is none.
map_size() {
    local t=0 n=0 hex obj objs=""
    while read -r hex obj; do
        t=$((t + hex))
        n=$((n + 1))
        objs+=" $obj"
    done < <(awk -v s="$1" '
        function emit(sz) { o = $NF; sub(/.*\//, "", o); print sz, o }
        /^Linker script and memory map/ { mm = 1; next }
        !mm { next }
        pend { if (NF >= 3 && $1 ~ /^0x/) emit($2); pend = 0; next }
        $1 == ".bss." s || $1 == ".data." s { if (NF >= 4) emit($3); else pend = 1 }' "$map")
    if [ "$n" -eq 0 ]; then echo "-"; else echo "$t$objs"; fi
}

missing=0
rows=""
for key in "${ITEMS[@]}"; do
    nm_total=0
    map_total=0
    map_ok=1
    syms_cell=""
    notes=""
    for s in ${SYMS[$key]}; do
        read -r sz cnt < <(nm_size "$s")
        m=$(map_size "$s")
        if [ "$cnt" -eq 0 ]; then
            syms_cell+="\`$s\` (not found) "
            missing=1
        else
            syms_cell+="\`$s\` $sz "
            [ "$cnt" -eq 1 ] || notes+="$cnt symbols named \`$s\` (sizes summed). "
        fi
        nm_total=$((nm_total + sz))
        if [ "$m" = "-" ]; then
            map_ok=0
        else
            map_total=$((map_total + ${m%% *}))
            notes+="\`$s\` in ${m#* }. "
        fi
    done
    if [ "$map_ok" -eq 1 ]; then
        map_cell=$map_total
        [ "$map_total" -eq "$nm_total" ] || notes+="map and nm differ. "
    else
        map_cell="no .bss/.data.SYM section"
    fi
    [ -z "${REPLACED[$key]+x}" ] || notes="replacement symbols (Requirement 22.6). $notes"
    rows+="| ${DESC[$key]} (\`$key\`) | ${syms_cell% } | $nm_total | $map_cell | ${notes% } |"$'\n'
done

# ------------------------------------------------------------------ report
cat <<EOF
# Static RAM report: $(basename "$elf")

| Record | Value |
|---|---|
| ELF | \`$elf\` |
| Map file | \`$map\` |
| Git commit | \`$commit\` |
| Working tree | $tree |
| make command line | \`$make_cmdline\` |
| Robot options (\`.robot_opts\`) | \`$robot_opts\` |
| Compiler (\`$GCC --version\`) | $gcc_version |
| Report date (UTC) | $(date -u +%Y-%m-%dT%H:%M:%SZ) |

## \`${SIZE}\` output (unedited)

\`\`\`text
$size_out
\`\`\`

## \`${SIZE} -A\` output (unedited)

\`\`\`text
$size_a_out
\`\`\`

## Static RAM (Requirement 22.1)

| Section | Bytes |
|---|---|
| .data | $data_b |
| .bss | $bss_b |
| .data + .bss | $sum_b |

## RAM items (Requirement 22.3)

Sizes in bytes. "nm -S" is the symbol size from \`${NM} -S\`; "Map file"
is the size of the item's \`.bss.SYM\` / \`.data.SYM\` input section in the
linker map file.

| Item | Symbols (bytes each) | nm -S bytes | Map file bytes | Notes |
|---|---|---|---|---|
${rows%$'\n'}
EOF

if [ "$missing" -ne 0 ]; then
    echo "ram_report: an item symbol was not found in $elf; give its replacements with -r ITEM=SYM,..." >&2
    exit 1
fi
exit 0
