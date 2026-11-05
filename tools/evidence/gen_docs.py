#!/usr/bin/env python3
"""Generates docs/data_representation.md and docs/protocol.md (tasks 20.2, 20.3).

Run in WSL from the repository root, after `make -C tests/host tools` (or
run_all.sh) has built tests/host/build/wire_dump:

    python3 tools/evidence/gen_docs.py

Nothing about a layout is typed here. Shifts, widths, offsets and sizes come from
  - the named shift / width constants in the headers and drivers,
  - `arm-none-eabi-gcc -c` plus `arm-none-eabi-nm -S` on a probe file (sizeof and
    enum counts, the same trick as type_sizes.c),
  - the JSON that tests/host/build/wire_dump prints,
  - docs/cr1/PROGRESS_FW.md and docs/cr1/REVIEW_FLAGS.md for the scope section.
What is typed here is the Valid_Range, Resolution and Baseline_Build type of each
field (the specification, not the layout). The script checks every Required_Bits it
computes against the width it read from the code and stops on a mismatch.
"""
import json
import math
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
APP = ROOT / "app_program"
CROSS = os.environ.get("CROSS", "arm-none-eabi-")


def rd(name):
    return (APP / name).read_text(encoding="utf-8", errors="replace")


# ---------------------------------------------------------------------------
# constants from the sources
# ---------------------------------------------------------------------------
CONST = {}
BITIDX = {}
for fn in ["status.h", "isr_ring.h", "imu_logic.h", "hal_isr.c", "hal_board.c", "hal_imu.c",
           "app_shared.c", "nav_map.c", "app_params.h", "shared.h", "wire_ids.h", "app_net.h"]:
    txt = rd(fn)
    for m in re.finditer(r"^\s*#\s*define\s+(\w+)\s+\(?\s*(0x[0-9a-fA-F]+|\d+)[uU]?\s*\)?", txt, re.M):
        CONST[m.group(1)] = int(m.group(2), 0)
    for m in re.finditer(r"^\s*#\s*define\s+(\w+)\s+\(\s*1u\s*<<\s*(\d+)\s*\)", txt, re.M):
        BITIDX[m.group(1)] = int(m.group(2))


def C(name):
    if name in CONST:
        return CONST[name]
    if name in BITIDX:
        return BITIDX[name]
    sys.exit("gen_docs: constant %s not found in the sources" % name)


def mask_width(name):
    return CONST[name].bit_length()


# ---------------------------------------------------------------------------
# probe: sizeof and enum counts from the target compiler
# ---------------------------------------------------------------------------
PROBE = [
    ("sz_robot_status_t", "sizeof(robot_status_t)"), ("sz_motion_msg_t", "sizeof(motion_msg_t)"),
    ("sz_nav_msg_t", "sizeof(nav_msg_t)"), ("sz_obst_msg_t", "sizeof(obst_msg_t)"),
    ("sz_evt_msg_t", "sizeof(evt_msg_t)"), ("sz_wire_evt_t", "sizeof(wire_evt_t)"),
    ("sz_app_msg_body_t", "sizeof(app_msg_body_t)"), ("sz_app_params_t", "sizeof(app_params_t)"),
    ("sz_hump_ctx_t", "sizeof(hump_ctx_t)"), ("sz_mev_ctx_t", "sizeof(mev_ctx_t)"),
    ("sz_bc_ctx_t", "sizeof(bc_ctx_t)"), ("sz_odometry_t", "sizeof(odometry_t)"),
    ("sz_wire_cmd_t", "sizeof(wire_cmd_t)"),
    ("n_sys_state", "SYS_STATE_COUNT"), ("n_stop", "STOP_REASON_COUNT"), ("n_mev", "MEV_COUNT"),
    ("n_line", "LINE_LOST + 1"), ("n_hump", "HUMP_DESCENT + 1"), ("n_act", "ACT_RIGHT + 1"),
    ("n_bc_el", "BC_MAX_ELEMENTS"), ("n_bcm_levels", "BCM_MAX_LEVELS"),
    ("n_buzz", "BUZZ_WARN + 1"), ("n_motion_result", "MOTION_TIMEOUT + 1"),
]
PROBE_C = """#include <stdbool.h>
#include <stdint.h>
#include "app_types.h"
#include "status.h"
#include "shared.h"
#include "msg_codec.h"
#include "imu_logic.h"
#include "barcode_logic.h"
#include "bc_detect.h"
#include "bc_multi.h"
#include "nav_map.h"
#include "app_params.h"
#include "line_logic.h"
#include "wire.h"
""" + "".join("char pv_%s[%s];\n" % (n, e) for n, e in PROBE)


def run_probe():
    with tempfile.TemporaryDirectory() as td:
        src = Path(td) / "pv.c"
        obj = Path(td) / "pv.o"
        src.write_text(PROBE_C)
        cmd = [CROSS + "gcc", "-mcpu=cortex-m0plus", "-mthumb", "-std=gnu11", "-O2", "-ffreestanding",
               "-fno-common", "-DAPP_PERF=0", "-DAPP_SELFTEST=0",
               "-I", str(APP), "-c", str(src), "-o", str(obj)]
        r = subprocess.run(cmd, capture_output=True, text=True)
        if r.returncode:
            sys.exit("gen_docs: probe compile failed\n" + r.stderr)
        out = subprocess.run([CROSS + "nm", "-S", str(obj)], capture_output=True, text=True).stdout
    vals = {}
    for line in out.splitlines():
        p = line.split()
        if len(p) == 4 and p[3].startswith("pv_"):
            vals[p[3][3:]] = int(p[1], 16)
    gcc_v = subprocess.run([CROSS + "gcc", "--version"], capture_output=True, text=True).stdout.splitlines()[0]
    return vals, gcc_v


PV, GCC_VERSION = run_probe()

WD = json.loads(subprocess.run([str(ROOT / "tests/host/build/wire_dump")], capture_output=True, text=True,
                               check=True).stdout)
PARAM_COUNT = len(WD["params"])


# ---------------------------------------------------------------------------
# row model
# ---------------------------------------------------------------------------
def req_bits(lo, hi, res, extra=0, n=None):
    if n is None:
        n = int(round((hi - lo) / res)) + 1 + extra
    return max(1, (n - 1).bit_length()), n


ROWS = []   # one dict per Application_Field


def fld(cat, field, member, rng, res, base, unit, sh, w, lo=None, hi=None, rs=1, extra=0, n=None,
        note="", rb=None):
    if rb is None:
        rb, nn = req_bits(lo, hi, rs, extra, n)
    else:
        nn = n
    if rb != w:
        sys.exit("gen_docs: %s: Required_Bits %d differs from the width %d read from the code" % (field, rb, w))
    ROWS.append(dict(cat=cat, field=field, member=member, rng=rng, res=res, rb=rb, base=base, unit=unit,
                     sh=sh, w=w, note=note))


def st(cat, unit, field, rng, res, base, shc, wc, **kw):
    sh = C(shc)
    w = C(wc) if isinstance(wc, str) else wc
    fld(cat, field, "`g_status.%s`" % unit, rng, res, base, "`%s` (32)" % unit, sh, w, **kw)


F32 = "`float` (32)"
U32 = "`uint32_t` (32)"
I32 = "`int32_t` (32)"

CAT_MOT = "Motion control and encoder data"
CAT_LINE = "Line and barcode sensing"
CAT_OBS = "Obstacle detection"
CAT_HUMP = "Hump estimation"
CAT_VEH = "Vehicle and controller states"
CAT_TASK = "Task_Shared_Data"
CAT_ISR = "ISR_Shared_Data"
CAT_MSG = "Inter_Task_Messages"
CAT_MQTT = "MQTT telemetry and command payloads"
CATS = [CAT_MOT, CAT_LINE, CAT_OBS, CAT_HUMP, CAT_VEH, CAT_TASK, CAT_ISR, CAT_MSG, CAT_MQTT]

# ---- motion control and encoder data (Status_Snapshot) --------------------------
st(CAT_MOT, "mot_a", "spd_l (left wheel speed)", "-1000.0 .. +1000.0 mm/s", "0.1 mm/s", F32, "ST_SH_SPD_L", "ST_W_SPD",
   lo=-1000, hi=1000, rs=0.1)
st(CAT_MOT, "mot_a", "spd_r (right wheel speed)", "-1000.0 .. +1000.0 mm/s", "0.1 mm/s", F32, "ST_SH_SPD_R", "ST_W_SPD",
   lo=-1000, hi=1000, rs=0.1)
st(CAT_MOT, "mot_b", "duty_l (left duty)", "-100.0 .. +100.0 %", "0.1 %", F32 + ", -1..+1", "ST_SH_DUTY_L", "ST_W_DUTY",
   lo=-100, hi=100, rs=0.1)
st(CAT_MOT, "mot_b", "duty_r (right duty)", "-100.0 .. +100.0 %", "0.1 %", F32 + ", -1..+1", "ST_SH_DUTY_R", "ST_W_DUTY",
   lo=-100, hi=100, rs=0.1)
st(CAT_MOT, "mot_b", "speed_scale (hump speed limit)", "0.00 .. 1.00", "0.01", F32, "ST_SH_SCALE", "ST_W_SCALE",
   lo=0, hi=1, rs=0.01)
st(CAT_MOT, "mot_c", "enc_l (left encoder count)", "-8,388,607 .. +8,388,607 counts (saturates)", "1 count", I32,
   "ST_SH_ENC", "ST_W_ENC", lo=-8388607, hi=8388607, rs=1)
st(CAT_MOT, "mot_d", "enc_r (right encoder count)", "-8,388,607 .. +8,388,607 counts (saturates)", "1 count", I32,
   "ST_SH_ENC", "ST_W_ENC", lo=-8388607, hi=8388607, rs=1)
st(CAT_MOT, "mot_e", "dist (distance travelled)", "0.0 .. 100,000.0 mm", "0.1 mm", F32, "ST_SH_DIST", "ST_W_DIST",
   lo=0, hi=100000, rs=0.1)

# odometry record (shared.h / app_shared.c): widths are the differences of the named offsets
ODO = [("odo_um", "ODO_P_UM"), ("dist", "ODO_P_DIST"), ("heading", "ODO_P_HEAD"), ("v", "ODO_P_V"),
       ("a_fwd", "ODO_P_A"), ("turn", "ODO_P_TURN")]
m_last = re.search(r"ODO_P_TURN\s*\+\s*(\d+)u\)\s*<=\s*\(ODO_WORDS", rd("app_shared.c"))
odo_end = C("ODO_P_TURN") + int(m_last.group(1))
ODO_W = {}
for i, (nm, c) in enumerate(ODO):
    nxt = C(ODO[i + 1][1]) if i + 1 < len(ODO) else odo_end
    ODO_W[nm] = nxt - C(c)
ODO_BITS = odo_end
ODO_SPEC = {   # name: (range, res, base, lo, hi, rs)
    "odo_um": ("0 .. 4,294,967,295 um (monotonic, wraps)", "1 um", "`uint32_t` (32)", None),
    "dist": ("-1,342,177.27 .. +1,342,177.27 mm", "0.01 mm", "2 x `float` (64): `dist_r_mm`, `dist_l_mm`", (-134217727, 134217727, 1)),
    "heading": ("-100,000.00 .. +100,000.00 deg", "0.01 deg", F32, (-10000000, 10000000, 1)),
    "v": ("-1000.00 .. +1000.00 mm/s", "0.01 mm/s", F32, (-100000, 100000, 1)),
    "a_fwd": ("-4.0000 .. +4.0000 g", "0.0001 g", F32, (-40000, 40000, 1)),
    "turn": ("-720.00 .. +720.00 deg/s", "0.01 deg/s", F32, (-72000, 72000, 1)),
}
for nm, c in ODO:
    rng, res, base, q = ODO_SPEC[nm]
    off = C(c)
    w = ODO_W[nm]
    word, sh = off // 32, off % 32
    straddle = " (crosses into word %d)" % (word + 1) if sh + w > 32 else ""
    if q is None:
        rb = 32
    else:
        rb = req_bits(q[0], q[1], q[2])[0]
    fld(CAT_MOT, "odometry `%s`" % nm, "`g_odo_rec[%d]` via `odometry_t`" % word, rng, res, base,
        "`g_odo_rec[5]` (5 x 32)", sh, w,
        rb=rb, note="record bit %d%s" % (off, straddle))

# ---- line and barcode sensing ------------------------------------------------------
st(CAT_LINE, "line_a", "line_l (left line sensor)", "0 white, 1 black", "1", "`int32_t` (32)", "ST_SH_LINE_L", "ST_W_LINE", n=2)
st(CAT_LINE, "line_a", "line_r (right line sensor)", "0 white, 1 black", "1", "`int32_t` (32)", "ST_SH_LINE_R", "ST_W_LINE", n=2)
st(CAT_LINE, "line_a", "ir (barcode ADC reading)", "0 .. 4095", "1", U32, "ST_SH_IR", "ST_W_IR", lo=0, hi=4095)
st(CAT_LINE, "line_a", "line_state", "0 .. %d (`line_state_t`, %d states)" % (PV["n_line"] - 1, PV["n_line"]), "1", U32,
   "ST_SH_LINE_STATE", "ST_W_LINE_STATE", n=PV["n_line"])
st(CAT_LINE, "line_a", "barcode (last decoded)", "0 none; 1 .. 63 = ASCII - 32 of a Code 39 character (43 used)", "1",
   "`char` (8)", "ST_SH_BARCODE", "ST_W_BARCODE", n=44)
st(CAT_LINE, "line_a", "bc_enabled", "0 sampling off, 1 on", "1", U32, "ST_SH_BC_ON", "ST_W_BC_ON", n=2)
st(CAT_LINE, "line_a", "barcodes_read", "0 .. 127 (saturates)", "1", U32, "ST_SH_BC_N", "ST_W_BC_N", lo=0, hi=127)
st(CAT_LINE, "line_b", "bc_thr_black", "0 .. 4095 ADC", "1", U32, "ST_SH_THR_BLACK", "ST_W_THR", lo=0, hi=4095)
st(CAT_LINE, "line_b", "bc_thr_white", "0 .. 4095 ADC", "1", U32, "ST_SH_THR_WHITE", "ST_W_THR", lo=0, hi=4095)

# bc_ctx_t bit-fields (private to TSK_LINE_BARCODE; read from the header)
bc_txt = rd("barcode_logic.h")
bc_struct = re.search(r"typedef struct\s*\{[^}]*uint32_t n\s*:\s*7;.*?\}\s*bc_ctx_t;", bc_txt, re.S).group(0)
BCF = [(m.group(1), int(m.group(2))) for m in re.finditer(r"uint32_t\s+(\w+)\s*:\s*(\d+);", bc_struct)]
off = 0
BC_RANGE = {"n": ("0 .. 96 elements", 97, "`uint32_t` (32)"), "last_n": ("0 .. 96 elements", 97, "`uint32_t` (32)"),
            "fails": ("0 .. 2 failed passes (`BC_FAULT_COUNT` - 1)", 3, "`uint32_t` (32)"),
            "b_open": ("0 closed, 1 window open", 2, "`bool` (8)"),
            "b_black": ("0 white, 1 black", 2, "`bool` (8)")}
for nm, w in BCF:
    rng, nn, base = BC_RANGE[nm]
    fld(CAT_LINE, "bc_ctx_t `%s`" % nm, "`bc_ctx_t.%s` (C bit-field)" % nm, rng, "1", base,
        "`uint32_t` bit-field word (32)", off, w, n=nn)
    off += w
fld(CAT_LINE, "bc_ctx_t `w[i]` (element width)", "`bc_ctx_t.w[%d]`" % PV["n_bc_el"],
    "0 .. 65,535 um (saturates; a Code 39 element is 2 to 8 mm)", "1 um", "`uint32_t` (32) each", "`uint16_t` array (16)", 0, 16,
    lo=0, hi=65535, note="array of %d; the baseline `bool bar[%d]` flags are gone (element i is a bar when i is even)" % (PV["n_bc_el"], PV["n_bc_el"]))

# ---- obstacle detection -------------------------------------------------------------
for nm, shc, unit, lab in [("obst_cm (last forward reading)", "ST_SH_OBST", "obs_a", None),
                           ("scan[0] (coarse sweep, -60 deg)", "ST_SH_SCAN0", "obs_a", None),
                           ("scan[1]", "ST_SH_SCAN1", "obs_b", None), ("scan[2]", "ST_SH_SCAN2", "obs_b", None),
                           ("scan[3]", "ST_SH_SCAN3", "obs_c", None), ("scan[4] (+60 deg)", "ST_SH_SCAN4", "obs_c", None)]:
    st(CAT_OBS, unit, nm, "0.0 .. 431.0 cm; 8191 = no echo", "0.1 cm", F32 + ", < 0 = no echo", shc, "ST_W_US",
       lo=0, hi=431.0, rs=0.1, extra=1)
st(CAT_OBS, "obs_a", "scan_side (bypass side)", "0 none, 1 left, 2 right", "1", "`char` (8)", "ST_SH_SCAN_SIDE",
   "ST_W_SCAN_SIDE", n=3)
st(CAT_OBS, "obs_c", "obstacles (avoided this run)", "0 .. 63 (saturates)", "1", U32, "ST_SH_OBSTACLES",
   "ST_W_OBSTACLES", lo=0, hi=63)

# ---- hump estimation -----------------------------------------------------------------
st(CAT_HUMP, "imu_a", "pitch", "-90.0 .. +90.0 deg", "0.1 deg", F32, "ST_SH_PITCH", "ST_W_PITCH", lo=-90, hi=90, rs=0.1)
st(CAT_HUMP, "imu_a", "hump_h (height on the hump)", "-1500.0 .. +1500.0 mm", "0.1 mm", F32, "ST_SH_HUMP_H",
   "ST_W_HUMP_H", lo=-1500, hi=1500, rs=0.1)
st(CAT_HUMP, "imu_b", "hump_peak", "0.0 .. 1500.0 mm", "0.1 mm", F32, "ST_SH_HUMP_PEAK", "ST_W_HUMP_U", lo=0, hi=1500, rs=0.1)
st(CAT_HUMP, "imu_b", "hump_max (highest peak of the run)", "0.0 .. 1500.0 mm", "0.1 mm", F32, "ST_SH_HUMP_MAX",
   "ST_W_HUMP_U", lo=0, hi=1500, rs=0.1)
st(CAT_HUMP, "imu_c", "turn (turn rate)", "-720.0 .. +720.0 deg/s", "0.1 deg/s", F32, "ST_SH_TURN", "ST_W_TURN",
   lo=-720, hi=720, rs=0.1)
st(CAT_HUMP, "imu_c", "humps (counted this run)", "0 .. 63 (saturates)", "1", U32, "ST_SH_HUMPS", "ST_W_HUMPS", lo=0, hi=63)
fld(CAT_HUMP, "hump_ctx_t `state`", "`hump_ctx_t.f`", "0 .. %d (`hump_state_t`)" % (PV["n_hump"] - 1), "1",
    "`hump_state_t` enum (32)", "`f` (16)", C("HUMP_SH_STATE"), C("HUMP_W_STATE"), n=PV["n_hump"])
for nm, shc in [("n_up", "HUMP_SH_N_UP"), ("n_down", "HUMP_SH_N_DOWN"), ("n_level", "HUMP_SH_N_LEVEL")]:
    fld(CAT_HUMP, "hump_ctx_t `%s` (sample counter)" % nm, "`hump_ctx_t.f`", "0 .. 5 (holds at `HUMP_N_MAX`)", "1",
        "`uint32_t` (32)", "`f` (16)", C(shc), C("HUMP_W_N"), lo=0, hi=5)

# ---- vehicle and controller states -------------------------------------------------------
st(CAT_VEH, "sys", "state", "0 .. %d (`sys_state_t`, %d states)" % (PV["n_sys_state"] - 1, PV["n_sys_state"]), "1", U32,
   "ST_SH_STATE", "ST_W_STATE", n=PV["n_sys_state"])
st(CAT_VEH, "sys", "faults (Fault_Mask)", "16 independent flags (bit table below)", "1 flag per bit",
   U32 + " (13 bits used)", "ST_SH_FAULTS", "ST_W_FAULTS", n=1 << 16)
st(CAT_VEH, "sys", "stop reason", "0 .. %d (`stop_reason_t`, %d codes)" % (PV["n_stop"] - 1, PV["n_stop"]), "1",
   "`const char *` text (32)", "ST_SH_STOP", "ST_W_STOP", n=PV["n_stop"])
st(CAT_VEH, "sys", "motion event", "0 .. %d (`motion_event_t`, %d values)" % (PV["n_mev"] - 1, PV["n_mev"]), "1", U32,
   "ST_SH_MEV", "ST_W_MEV", n=PV["n_mev"])
st(CAT_VEH, "misc", "vbat (battery)", "0 .. 6600 mV", "1 mV", U32, "ST_SH_VBAT", "ST_W_VBAT", lo=0, hi=6600)
fld(CAT_VEH, "mev_ctx_t `current`", "`mev_ctx_t.f`", "0 .. %d (`motion_event_t`)" % (PV["n_mev"] - 1), "1",
    "`motion_event_t` enum (32)", "`f` (8)", C("MEV_SH_CURRENT"), C("MEV_W_EV"), n=PV["n_mev"])
fld(CAT_VEH, "mev_ctx_t `candidate`", "`mev_ctx_t.f`", "0 .. %d (`motion_event_t`)" % (PV["n_mev"] - 1), "1",
    "`motion_event_t` enum (32)", "`f` (8)", C("MEV_SH_CAND"), C("MEV_W_EV"), n=PV["n_mev"])
fld(CAT_VEH, "mev_ctx_t `n_candidate`", "`mev_ctx_t.f`", "0 .. 3 (holds at `MEV_HOLD`)", "1", "`uint32_t` (32)",
    "`f` (8)", C("MEV_SH_N_CAND"), C("MEV_W_N_CAND"), lo=0, hi=3)

# ---- Task_Shared_Data -------------------------------------------------------------------------
fld(CAT_TASK, "buzzer pattern", "`g_buzz`", "0 .. 6 (`BUZZ_NONE` .. `BUZZ_WARN`)", "1", "`uint32_t` (32)", "`g_buzz` (8)", 0, 3,
    n=PV["n_buzz"])
for nm, bit in [("estop_pending", 0), ("estop_ack_due", 1), ("config_due", 2)]:
    fld(CAT_TASK, nm, "`g_net_flags`", "0, 1", "1", "`volatile bool` (8)", "`g_net_flags` (8)", bit, 1, n=2)
fld(CAT_TASK, "E-stop receive time", "`g_estop_rx_us`", "0 .. 2^32 - 1 us (low 32 bits of the timer, wraps)", "1 us",
    "`uint64_t` (64)", "`g_estop_rx_us` (32)", 0, 32, rb=32, n=None)
fld(CAT_TASK, "E-stop PWM-off time", "`g_estop_pwm_off_us`", "0 .. 65,535 us (saturates)", "1 us", "`uint32_t` (32)",
    "`g_estop_pwm_off_us` (16)", 0, 16, lo=0, hi=65535)
done_shift = int(re.search(r"BP_PUT\(\(uint32_t\)id,\s*(\d+)u,\s*DONE_W_RESULT", rd("app_shared.c")).group(1))
fld(CAT_TASK, "motion done id", "`g_done`", "0 .. 65,535", "1", "`uint16_t` (16) + separate result", "`g_done` (32)", 0,
    done_shift, lo=0, hi=65535)
fld(CAT_TASK, "motion done result", "`g_done`", "0 .. 2 (`motion_result_t`)", "1", "`uint32_t` (32)", "`g_done` (32)",
    done_shift, C("DONE_W_RESULT"), n=PV["n_motion_result"])
fld(CAT_TASK, "map action of letter i (26 letters)", "`g_map_w[i / %d]`" % C("MAP_PER_WORD"),
    "0 .. 4 (`nav_action_t`, 0 = not mapped)", "1", "`uint8_t g_map[26]` (8 each)", "`g_map_w[3]` (3 x 32)",
    "3 x (i mod %d)" % C("MAP_PER_WORD"), C("MAP_ACT_W"), n=PV["n_act"], note="10 letters per word, 26 letters in 3 words")
for i, nm in enumerate(["HB_MOTION", "HB_LINE", "HB_IMU"]):
    fld(CAT_TASK, "heartbeat of %s" % ["TSK_MOTION_CTRL", "TSK_LINE_BARCODE", "TSK_TERRAIN_IMU"][i], "`g_hb[%d]`" % i,
        "0 .. 255 (counts up, wraps)", "1", "`uint32_t` (32)", "`g_hb[%d]` (8)" % i, 0, 8, n=256)
fld(CAT_TASK, "line re-arm counter", "`g_reacq_seq`", "0 .. 255 (counts up, wraps)", "1", "`uint32_t` (32)",
    "`g_reacq_seq` (8)", 0, 8, n=256)
fld(CAT_TASK, "motion command id", "`g_next_id` (tsk_motion.c)", "1 .. 65,535 (0 skipped)", "1", "`uint16_t` (16)",
    "`g_next_id` (16)", 0, 16, lo=0, hi=65535)
fld(CAT_TASK, "Saturation_Counter", "`g_sat_count` (os_glue.c)", "0 .. 65,535 (saturates)", "1", "none", "`g_sat_count` (16)", 0, 16,
    lo=0, hi=65535)

# ---- ISR_Shared_Data ---------------------------------------------------------------------------
fld(CAT_ISR, "encoder count, wheel 0 (right) and 1 (left)", "`g_enc[2]` (hal_isr.c)", "modulo 2^32, 4x counts", "1 count",
    "`int32_t` (32) each", "`g_enc[w]` (32) each", 0, 32, rb=32, note="own unit per wheel")
fld(CAT_ISR, "odometer", "`g_odo`", "modulo 2^32 counts", "1 count", "`uint32_t` (32)", "`g_odo` (32)", 0, 32, rb=32)
fld(CAT_ISR, "encoder error count", "`g_enc_err`", "modulo 2^32", "1", "`uint32_t` (32)", "`g_enc_err` (32)", 0, 32, rb=32)
fld(CAT_ISR, "encoder reversed, right (bit 0) / left (bit 1)", "`g_enc_rev`", "0, 1 each", "1", "`bool[2]` (2 x 8)",
    "`g_enc_rev` (8)", "wheel number", 1, n=2, note="two flags, bit = wheel (`ENC_REV_BIT`)")
fld(CAT_ISR, "quadrature state of one wheel", "`g_enc_prev[2]`", "0 .. 3", "1", "`uint8_t` (8) each", "`g_enc_prev[w]` (8)",
    0, 2, n=4, note="private to the GPIO ISR")
fld(CAT_ISR, "echo width", "`g_echo`", "0 .. 25,000 us; 25,001 = longer than the timeout", "1 us", "`uint32_t` (32)",
    "`g_echo` (32)", 0, mask_width("ECHO_US_MASK"), lo=0, hi=25000, extra=1)
fld(CAT_ISR, "echo done", "`g_echo`", "0, 1", "1", "`bool` (8)", "`g_echo` (32)", BITIDX["ECHO_DONE"], 1, n=2)
fld(CAT_ISR, "echo rise seen", "`g_echo`", "0, 1", "1", "`bool` (8, within the echo state)", "`g_echo` (32)", BITIDX["ECHO_RISE"], 1, n=2)
fld(CAT_ISR, "echo rise time", "`g_echo_rise_us`", "low 32 bits of the 1 MHz timer (modulo 2^32)", "1 us",
    "`uint64_t` (64)", "`g_echo_rise_us` (32)", 0, 32, rb=32)
fld(CAT_ISR, "last ADC sample", "`g_adc_last`", "0 .. 4095", "1", "`uint32_t` (32)", "`g_adc_last` (16)", 0, C("ADC_W"),
    lo=0, hi=4095)
fld(CAT_ISR, "ADC minimum since the last read", "`g_adc_rng`", "0 .. 4095", "1", "`uint32_t` (32)", "`g_adc_rng` (32)",
    C("ADC_MIN_SH"), C("ADC_W"), lo=0, hi=4095)
fld(CAT_ISR, "ADC maximum since the last read", "`g_adc_rng`", "0 .. 4095", "1", "`uint32_t` (32)", "`g_adc_rng` (32)",
    C("ADC_MAX_SH"), C("ADC_W"), lo=0, hi=4095)
fld(CAT_ISR, "ADC range has a sample", "`g_adc_rng`", "0 none yet, 1 have", "1", "none (min = 0xFFFF as the marker)",
    "`g_adc_rng` (32)", BITIDX["ADC_HAVE"], 1, n=2)
for nm, c in [("sampling enabled", "BC_F_ENABLED"), ("black is the high ADC level", "BC_F_BLACK_HIGH"),
              ("current level black", "BC_F_BLACK"), ("raw sampling on", "BC_F_RAW")]:
    fld(CAT_ISR, nm, "`g_bc_flags`", "0, 1", "1", "`bool` (8)", "`g_bc_flags` (8)", BITIDX[c], 1, n=2)
fld(CAT_ISR, "threshold black", "`g_bc_thr`", "0 .. 4095 ADC", "1", "`uint32_t` (32)", "`g_bc_thr` (32)", C("BC_THR_BLACK_SH"),
    C("BC_THR_W"), lo=0, hi=4095)
fld(CAT_ISR, "threshold white", "`g_bc_thr`", "0 .. 4095 ADC", "1", "`uint32_t` (32)", "`g_bc_thr` (32)", C("BC_THR_WHITE_SH"),
    C("BC_THR_W"), lo=0, hi=4095)
fld(CAT_ISR, "edge slot: black level", "`g_bc_ring[%d]`" % C("BC_RING"), "0 white, 1 black", "1", "`bool` (8) in `hal_bc_edge_t`",
    "`g_bc_ring[i]` (32)", BITIDX["BC_EDGE_BLACK"], 1, n=2)
fld(CAT_ISR, "edge slot: odometer low bits", "`g_bc_ring[%d]`" % C("BC_RING"), "odometer modulo 2^31", "1 count",
    "`uint32_t` position + time in `hal_bc_edge_t` (96 total)", "`g_bc_ring[i]` (32)", 0, mask_width("BC_ODO_MASK"),
    rb=31)
fld(CAT_ISR, "edge ring head / tail", "`g_bc_head`, `g_bc_tail`", "modulo %d" % (2 * C("BC_RING")), "1",
    "`uint32_t` (32) each", "`g_bc_head`, `g_bc_tail` (8) each", 0, (2 * C("BC_RING") - 1).bit_length(), n=2 * C("BC_RING"))
fld(CAT_ISR, "edge ring overruns", "`g_bc_overruns`", "modulo 2^32", "1", "`uint32_t` (32)", "`g_bc_overruns` (32)", 0, 32, rb=32)
fld(CAT_ISR, "raw slot: ADC", "`g_raw[%d]`" % C("RAW_RING"), "0 .. 4095", "1", "`uint16_t` (16) in `g_raw_adc`",
    "`g_raw[i]` (32)", 0, mask_width("RAW_ADC_MASK"), lo=0, hi=4095)
fld(CAT_ISR, "raw slot: right count, low bits", "`g_raw[%d]`" % C("RAW_RING"), "modulo %d" % (1 << C("RAW_CNT_BITS")), "1 count",
    "`int32_t` (32) in `g_raw_er`", "`g_raw[i]` (32)", C("RAW_ER_SHIFT"), C("RAW_CNT_BITS"), n=1 << C("RAW_CNT_BITS"))
fld(CAT_ISR, "raw slot: left count, low bits", "`g_raw[%d]`" % C("RAW_RING"), "modulo %d" % (1 << C("RAW_CNT_BITS")), "1 count",
    "`int32_t` (32) in `g_raw_el`", "`g_raw[i]` (32)", C("RAW_EL_SHIFT"), C("RAW_CNT_BITS"), n=1 << C("RAW_CNT_BITS"))
fld(CAT_ISR, "raw slot: gap", "`g_raw[%d]`" % C("RAW_RING"), "0, 1 (samples dropped just before)", "1", "none",
    "`g_raw[i]` (32)", 30, 1, n=2, note="`RAW_GAP` is `1u << 30`; bit 31 unused")
fld(CAT_ISR, "raw ring head / tail", "`g_raw_head`, `g_raw_tail`", "modulo %d" % (2 * C("RAW_RING")), "1",
    "`uint32_t` (32) each", "`g_raw_head`, `g_raw_tail` (16) each", 0, (2 * C("RAW_RING") - 1).bit_length(), n=2 * C("RAW_RING"))
fld(CAT_ISR, "battery voltage", "`g_vbat_mv`", "0 .. 6600 mV", "1 mV", "`uint32_t` (32)", "`g_vbat_mv` (16)", 0, C("ST_W_VBAT"),
    lo=0, hi=6600)
fld(CAT_ISR, "VBAT divider count", "`g_vbat_div`", "0 .. %d" % (C("VBAT_EVERY") - 1), "1", "`uint32_t` (32) `g_sample_n`",
    "`g_vbat_div` (8)", 0, 8, n=C("VBAT_EVERY"), note="only when `VBAT_ENABLED` is 1")
fld(CAT_ISR, "motor killed", "`g_motor_flags`", "0, 1", "1", "`bool` (8)", "`g_motor_flags` (8)", BITIDX["MOTOR_F_KILLED"], 1, n=2)
fld(CAT_ISR, "servo minimum pulse", "`g_servo_rng`", "0 .. 4095 us", "1 us", "`uint32_t` (32)", "`g_servo_rng` (32)",
    C("SERVO_MIN_SH"), C("SERVO_US_W"), lo=0, hi=4095)
fld(CAT_ISR, "servo maximum pulse", "`g_servo_rng`", "0 .. 4095 us", "1 us", "`uint32_t` (32)", "`g_servo_rng` (32)",
    C("SERVO_MAX_SH"), C("SERVO_US_W"), lo=0, hi=4095)
fld(CAT_ISR, "IMU chip", "`g_imu_cfg`", "0 .. 2 (`hal_imu_chip_t`)", "1", "`hal_imu_chip_t` enum (32)", "`g_imu_cfg` (16)",
    C("IMU_CHIP_SH"), C("IMU_CHIP_W"), n=3)
fld(CAT_ISR, "IMU I2C address", "`g_imu_cfg`", "0 .. 127", "1", "`uint8_t` (8) in `hal_imu.c`", "`g_imu_cfg` (16)",
    C("IMU_ADDR_SH"), C("IMU_ADDR_W"), lo=0, hi=127)

# ---- Inter_Task_Messages (offsets read from the pack functions of msg_codec.c) --------------------
mc = rd("msg_codec.c")
P_HDR = int(re.search(r"#define P_HDR\s+\((\d+)u\)", mc).group(1))


def pack_body(name):
    return re.search(r"void %s\(.*?\n\}" % name, mc, re.S).group(0)


def wputs(body):
    return [(m.group(1).strip(), int(m.group(2))) for m in
            re.finditer(r"bp_wput\(w, ([^,]+), (\d+)u,", body)]


def off_val(expr):
    expr = expr.replace("P_HDR", str(P_HDR)).replace("u", "")
    return eval(expr, {"__builtins__": {}})


mp = wputs(pack_body("msg_motion_pack"))
# header fields, then the per-type fields: the code lists them in this order
MOT_NAMES = [("type", "E3", "1 .. 5 (`motion_cmd_type_t`)", 5, None), ("src", "E2", "1 .. 4 (`cmd_src_t`)", 4, None),
             ("id", "U16", "0 .. 65,535", None, (0, 65535, 1)),
             ("MC_LINE left", "S14", "-819.1 .. +819.1 mm/s", None, (-8191, 8191, 1)),
             ("MC_LINE right", "S14", "-819.1 .. +819.1 mm/s", None, (-8191, 8191, 1)),
             ("MC_TURN turn_deg", "S13", "-360.0 .. +360.0 deg", None, (-3600, 3600, 1)),
             ("MC_TURN speed", "S13", "-409.5 .. +409.5 mm/s", None, (-4095, 4095, 1)),
             ("MC_TURN timeout_ms", "U19", "0 .. 524,287 ms (saturates)", None, (0, 524287, 1)),
             ("MC_BYPASS dist", "S16", "-3000.0 .. +3000.0 mm", None, (-30000, 30000, 1)),
             ("MC_BYPASS speed", "S13", "-409.5 .. +409.5 mm/s", None, (-4095, 4095, 1)),
             ("MC_BYPASS timeout_ms", "U19", "0 .. 524,287 ms (saturates)", None, (0, 524287, 1)),
             ("MC_SPEED_LIMIT scale", "U7", "0.00 .. 1.00", None, (0, 100, 1))]
MOT_RES = {"MC_LINE left": "0.1 mm/s", "MC_LINE right": "0.1 mm/s", "MC_TURN turn_deg": "0.1 deg",
           "MC_TURN speed": "0.1 mm/s", "MC_BYPASS dist": "0.1 mm", "MC_BYPASS speed": "0.1 mm/s",
           "MC_SPEED_LIMIT scale": "0.01"}
MOT_BASE = {"type": "`uint8_t` (8)", "src": "`uint8_t` (8)", "id": "`uint16_t` (16)", "timeout_ms": "`uint32_t` (32)",
            "scale": "`float` (32)"}
if len(mp) != len(MOT_NAMES):
    sys.exit("gen_docs: msg_motion_pack has %d bp_wput calls, expected %d" % (len(mp), len(MOT_NAMES)))
for (expr, w), (nm, kind, rng, nn, q) in zip(mp, MOT_NAMES):
    o = off_val(expr)
    base = MOT_BASE.get(nm.split()[-1], "`float` (32)")
    if nn is not None:
        fld(CAT_MSG, "motion_msg_t `%s`" % nm, "`motion_msg_t.w[%d]`" % (o // 32), rng, "1", base,
            "`w[3]` (3 x 32)", o, w, n=nn, note="message bit %d" % o)
    else:
        fld(CAT_MSG, "motion_msg_t `%s`" % nm, "`motion_msg_t.w[%d]`" % (o // 32), rng, MOT_RES.get(nm, "1"), base,
            "`w[3]` (3 x 32)", o, w, lo=q[0], hi=q[1], rs=q[2], note="message bit %d" % o)
nav_pack = pack_body("msg_nav_pack")
fld(CAT_MSG, "nav_msg_t `type`", "`nav_msg_t.b`", "1 .. 7 (`nav_event_type_t`)", "1", "`uint8_t` (8)", "`b` (8)", 0, 3, n=8)
fld(CAT_MSG, "nav_msg_t `letter`", "`nav_msg_t.b`", "0 .. 25 (A .. Z)", "1", "`char` (8)", "`b` (8)", 3, 5, n=26,
    note="shift 3 from `msg_nav_pack`")
op = pack_body("msg_obst_pack")
sizes_obst = [int(x) for x in re.findall(r"bp_wput\(w, pos, (\d+)u,", op)]
n_scan = int(re.findall(r"i < (\d+)u; i\+\+", op)[-1])
sizes_obst = sizes_obst[:-1] + [sizes_obst[-1]] * n_scan     # the last call sits in the scan loop
OBST = [("valid", "0, 1", "1", "`bool` (8)", dict(n=2)),
        ("side", "0 none, 1 L, 2 R", "1", "`char` (8)", dict(n=3)),
        ("r_min", "0.0 .. 431.0 cm; 8191 = none", "0.1 cm", F32, dict(lo=0, hi=431, rs=0.1, extra=1)),
        ("theta_l", "-100.0 .. +100.0 deg", "0.1 deg", F32, dict(lo=-100, hi=100, rs=0.1)),
        ("theta_r", "-100.0 .. +100.0 deg", "0.1 deg", F32, dict(lo=-100, hi=100, rs=0.1)),
        ("x_l", "-400.0 .. +400.0 cm", "0.1 cm", F32, dict(lo=-400, hi=400, rs=0.1)),
        ("x_r", "-400.0 .. +400.0 cm", "0.1 cm", F32, dict(lo=-400, hi=400, rs=0.1)),
        ("width", "0.0 .. 800.0 cm", "0.1 cm", F32, dict(lo=0, hi=800, rs=0.1)),
        ("offset", "-400.0 .. +400.0 cm", "0.1 cm", F32, dict(lo=-400, hi=400, rs=0.1)),
        ("clear_left", "-400.0 .. +400.0 cm", "0.1 cm", F32, dict(lo=-400, hi=400, rs=0.1)),
        ("clear_right", "-400.0 .. +400.0 cm", "0.1 cm", F32, dict(lo=-400, hi=400, rs=0.1))]
OBST += [("scan[%d]" % i, "0.0 .. 431.0 cm; 8191 = no echo", "0.1 cm", F32, dict(lo=0, hi=431, rs=0.1, extra=1))
         for i in range(5)]
if len(sizes_obst) != len(OBST):
    sys.exit("gen_docs: msg_obst_pack has %d bp_wput calls, expected %d" % (len(sizes_obst), len(OBST)))
pos = 0
for (nm, rng, res, base, kw), w in zip(OBST, sizes_obst):
    fld(CAT_MSG, "obst_msg_t `%s`" % nm, "`obst_msg_t.w[%d]`" % (pos // 32), rng, res, base, "`w[6]` (6 x 32)", pos, w,
        note="message bit %d" % pos, **kw)
    pos += w
OBST_BITS = pos
fld(CAT_MSG, "evt_msg_t `t_ms`", "`evt_msg_t.t_ms`", "robot time, modulo 2^32 ms", "1 ms", "none (text event)",
    "`t_ms` (32)", 0, 32, rb=32)
fld(CAT_MSG, "evt_msg_t `e.code`", "`wire_evt_t.code`", "0 .. 44 (`evt_code_t`)", "1", "`char name[20]` (160)",
    "`code` (8)", 0, 6, n=len(WD["events"]), note="stored in a byte; 6 bits on the wire")
fld(CAT_MSG, "evt_msg_t `e.nbits`", "`wire_evt_t.nbits`", "0 .. 208 argument bits", "1", "`char detail[72]` (576)",
    "`nbits` (8)", 0, 8, lo=0, hi=208, note="`WIRE_EVT_ARG_CAP_BITS`")
fld(CAT_MSG, "evt_msg_t `e.args`", "`wire_evt_t.args[%d]`" % (PV["sz_wire_evt_t"] - 2), "argument bits, MSB first", "1", "(in `detail`)",
    "`args[]` (8 each)", 0, 8, rb=8, note="widths per event in `evt.h`")

# ---- MQTT payload rows (from wire_dump) ----------------------------------------------------------
# range / resolution / baseline text per telemetry and heartbeat field name; widths and offsets from wire_dump
TLM_RANGE = {
    "version": ("Format_Version 1 (nibble 0 .. 15)", 16, "none"), "seq": ("modulo 32,768", 1 << 15, "`seq` JSON number"),
    "t": ("modulo 16,777,216 ms", 1 << 24, "`t` JSON number"),
    "state": ("0 .. 7 (`sys_state_t`)", 8, "`state` name text"),
    "spd_l": ("-1000.0 .. +1000.0 mm/s", 20001, "`spd[0]` JSON number"), "spd_r": ("-1000.0 .. +1000.0 mm/s", 20001, "`spd[1]` JSON number"),
    "duty_l": ("-100.0 .. +100.0 %", 2001, "`duty[0]` JSON number"), "duty_r": ("-100.0 .. +100.0 %", 2001, "`duty[1]` JSON number"),
    "enc_l": ("-8,388,607 .. +8,388,607", 16777215, "`enc[0]` JSON number"), "enc_r": ("-8,388,607 .. +8,388,607", 16777215, "`enc[1]` JSON number"),
    "dist": ("0.0 .. 100,000.0 mm", 1000001, "`dist` JSON number"),
    "line_l": ("-1, 0, 1", 3, "`line.l` JSON number"), "line_r": ("-1, 0, 1", 3, "`line.r` JSON number"),
    "ir": ("0 .. 4095", 4096, "`line.ir` JSON number"), "ir_min": ("0 .. 4095", 4096, "`line.irmn` JSON number"),
    "ir_max": ("0 .. 4095", 4096, "`line.irmx` JSON number"),
    "thr_valid": ("0 sampling off (sentinel for both thresholds), 1", 2, "none"),
    "thr_black": ("0 .. 4095 (0 when off)", 4096, "`line.thr[0]`"), "thr_white": ("0 .. 4095 (0 when off)", 4096, "`line.thr[1]`"),
    "bc": ("0 .. 42 Code 39 index; 63 none", 44, "`bc` one-character text"),
    "pitch": ("-90.0 .. +90.0 deg", 1801, "`hump.pitch`"), "hump_h": ("-1500.0 .. +1500.0 mm", 30001, "`hump.h`"),
    "hump_peak": ("0.0 .. 1500.0 mm", 15001, "`hump.peak`"), "hump_max": ("0.0 .. 1500.0 mm", 15001, "`hump.max`"),
    "us": ("0.0 .. 431.0 cm; 8191 no echo", 4312, "`obst.cm` (-1.0 no echo)"),
    "side": ("0 none, 1 L, 2 R", 3, "`obst.side` text"), "imu_ev": ("0 .. 7 (`motion_event_t`)", 8, "`imu.ev` name text"),
    "turn": ("-720.0 .. +720.0 deg/s", 14401, "`imu.tr`"), "faults": ("16 flags (Fault_Mask)", 1 << 16, "`faults` JSON number"),
    "vbat": ("0 .. 6600 mV", 6601, "`vbat`"), "rssi": ("0 .. 128 (code = -dBm)", 129, "`rssi`"),
    "buf": ("0 .. 64 frames", 65, "`buf`"),
    "uptime": ("0 .. 131,071 s (saturates)", 131072, "`uptime` JSON number"), "reconnects": ("0 .. 65,535 (saturates)", 65536, "JSON number"),
    "dropped": ("0 .. 65,535 (saturates)", 65536, "JSON number"), "sat": ("0 .. 65,535 (saturates)", 65536, "none"),
    "n_params": ("0 .. 63 (55 now)", 64, "none"), "crc": ("0 .. 65,535 (CRC-16/CCITT-FALSE)", 65536, "none"),
    "param_crc": ("0 .. 65,535 (CRC-16/CCITT-FALSE)", 65536, "none"),
}
for k in range(4):
    TLM_RANGE["ip%d" % k] = ("octet %d of the broker IPv4, 0 .. 255" % k, 256, "dotted-decimal text")
for k in range(5):
    TLM_RANGE["scan%d" % k] = ("0.0 .. 431.0 cm; 8191 no echo", 4312, "`obst.scan[%d]`" % k)


def wire_rows(fname, frame, label):
    for f in WD["frames"][frame]["fields"]:
        if f["kind"] == "PAD":
            continue
        nm = f["name"]
        rng, n, base = TLM_RANGE.get(nm, (None, None, None))
        if frame == "heartbeat" and nm == "seq":
            rng, n = "modulo 16,384", 1 << 14
        ROWS.append(dict(cat=CAT_MQTT, field="%s `%s`" % (label, nm), member="%s bit %d" % (label, f["offset"]), rng=rng or "see `evt.h` / D10",
                         res=f["resolution"] or "1", rb=(max(1, (n - 1).bit_length()) if n else f["width"]), base=base or "JSON text",
                         unit=label, sh=f["offset"], w=f["width"], note=f["kind"] + (" sentinel %s" % f["sentinel"] if f.get("sentinel", -1) not in (-1, None) else "")))
        r = ROWS[-1]
        if n and r["rb"] != f["width"] and f["kind"] not in ("F32", "F"):
            sys.exit("gen_docs: wire field %s %s: Required_Bits %d, width %d" % (frame, nm, r["rb"], f["width"]))


wire_rows("telemetry", "telemetry", "Telemetry_Frame")
wire_rows("heartbeat", "heartbeat", "Heartbeat_Frame")
for fr, lab in [("event_header", "Event_Frame header"), ("ack_header", "Ack_Frame header"), ("command_header", "Command_Frame header"),
                ("map", "Map_Frame"), ("config", "Config_Frame")]:
    for f in WD["frames"][fr]["fields"]:
        if f["kind"] == "PAD":
            continue
        nm = f["name"]
        extra = {"version": ("Format_Version 1", "none"), "t": ("modulo 16,777,216 ms", "none"),
                 "code": ("0 .. 44 Event_Code", "`name`, `detail` text"), "cmd": ("0 .. 15 command code", "`cmd` text"),
                 "reason": ("0 .. 27 Reason_Code", "`result` text"), "count": ("0 .. 55 (`WIRE_PARAMS_MAX`)", "none"),
                 "pair_id": ("0 .. 54 Parameter_ID, repeated `count` times", "`NAME=` text"),
                 "pair_value": ("IEEE-754 float32, repeated `count` times", "`=VALUE` text")}.get(nm)
        rng, base = extra if extra else ("present 0, 1" if nm.endswith("present") else "0 .. 25 letter (A = 0)", "`MAP:` text")
        ROWS.append(dict(cat=CAT_MQTT, field="%s `%s`" % (lab, nm), member="%s bit %d" % (lab, f["offset"]), rng=rng, res=f["resolution"] or "1",
                         rb=f["width"], base=base, unit=lab, sh=f["offset"], w=f["width"], note=f["kind"]))
for e in WD["events"]:
    ROWS.append(dict(cat=CAT_MQTT, field="Event_Frame arguments of `%s` (code %d)" % (e["name"], e["code"]),
                     member="Event_Frame bit %d" % WD["frames"]["event_header"]["bits"], rng="argument layout in `evt.h` / D10", res="per argument",
                     rb=e["arg_bits"], base="`detail` text (72 bytes max)", unit="Event_Frame", sh=WD["frames"]["event_header"]["bits"],
                     w=e["arg_bits"], note="variable length" if e["variable"] else "fixed"))
CMD_HDR = WD["frames"]["command_header"]["bits"]       # arguments follow the 8-bit header
cmd_extra = [("Command_Frame `BARCODE` letter", CMD_HDR, 5, "0 .. 25", "`BARCODE:<letter>` text"),
             ("Command_Frame `PING` number", CMD_HDR, 16, "0 .. 65,535", "`PING:<n>` text"),
             ("Command_Frame `SET` Parameter_ID", CMD_HDR, 6, "0 .. 54", "`SET:NAME` text"),
             ("Command_Frame `SET` value", CMD_HDR + 6, 32, "IEEE-754 float32", "`=VALUE` text")]
for lab, o, w, rng, base in cmd_extra:
    ROWS.append(dict(cat=CAT_MQTT, field=lab, member="Command_Frame bit %d" % o, rng=rng, res="1", rb=w, base=base, unit="Command_Frame", sh=o, w=w,
                     note="offset = header bits from `wire_dump` (+ the Parameter_ID width for the value); widths from the command table of `wire_ids.h` / `wire.c`"))

# ---------------------------------------------------------------------------
# rendering
# ---------------------------------------------------------------------------
def num(v):
    return "%.6g" % v


def esc(s):
    return str(s).replace("|", "\\|")


def render_rows(cat):
    out = ["| Field | Revised_Build C member | Valid_Range (sentinels) | Resolution | Required_Bits | Baseline_Build type (bits) | "
           "Storage_Unit (bits) | Bit offset | Bit width | Notes |", "|---|---|---|---|---|---|---|---|---|---|"]
    for r in ROWS:
        if r["cat"] != cat:
            continue
        out.append("| %s | %s | %s | %s | %d | %s | %s | %s | %d | %s |" % tuple(
            esc(x) if not isinstance(x, int) else x for x in
            (r["field"], r["member"], r["rng"], r["res"], r["rb"], r["base"], r["unit"], r["sh"], r["w"], r["note"])))
    return "\n".join(out)


def src(name):
    return "`app_program/%s`" % name


def sizes_table():
    t = [("`robot_status_t` (Status_Snapshot)", "sz_robot_status_t", "app_types.h", "156"),
         ("`motion_msg_t`", "sz_motion_msg_t", "msg_codec.h", "32 (`motion_cmd_t`)"),
         ("`nav_msg_t`", "sz_nav_msg_t", "msg_codec.h", "2 (`nav_event_t`)"),
         ("`obst_msg_t`", "sz_obst_msg_t", "msg_codec.h", "64 (`obstacle_profile_t`)"),
         ("`evt_msg_t`", "sz_evt_msg_t", "evt.h", "92 (`tlm_event_t`)"),
         ("`app_msg_body_t` (union)", "sz_app_msg_body_t", "app_types.h", "92"),
         ("`app_params_t` (Parameter_Store)", "sz_app_params_t", "app_params.h", "220"),
         ("`hump_ctx_t`", "sz_hump_ctx_t", "imu_logic.h", "28"),
         ("`mev_ctx_t`", "sz_mev_ctx_t", "imu_logic.h", "16"),
         ("`bc_ctx_t`", "sz_bc_ctx_t", "barcode_logic.h", "512"),
         ("`odometry_t` (caller-side float form)", "sz_odometry_t", "shared.h", "36"),
         ("`wire_cmd_t` (decoded Command_Frame)", "sz_wire_cmd_t", "wire.h", "none")]
    out = ["| Packed type | Size now (bytes, target compiler) | Baseline_Build (bytes) | Declared in |", "|---|---|---|---|"]
    for lab, key, fn, base in t:
        out.append("| %s | %d | %s | %s |" % (lab, PV[key], base, src(fn)))
    out.append("| `g_odo_rec` (packed odometry record, %d bits used) | %d | 36 (`odometry_t`) | %s |" % (ODO_BITS, C("ODO_WORDS") * 4, src("app_shared.c")))
    out.append("| `g_map_w` (packed map) | %d | 26 (`g_map[26]`) | %s |" % (C("MAP_WORDS") * 4, src("nav_map.c")))
    return "\n".join(out)


STATUS_WRITERS = {
    "sys": "state: TSK_NAV_PLANNER, TSK_SAFETY_MON; faults: any task; stop reason: TSK_SAFETY_MON; motion event: TSK_TERRAIN_IMU",
    "mot_a": "TSK_MOTION_CTRL", "mot_b": "TSK_MOTION_CTRL", "mot_c": "TSK_MOTION_CTRL", "mot_d": "TSK_MOTION_CTRL",
    "mot_e": "TSK_MOTION_CTRL",
    "line_a": "TSK_LINE_BARCODE; TSK_NAV_PLANNER resets `barcode` and `barcodes_read` (`st_run_reset`)",
    "line_b": "TSK_LINE_BARCODE",
    "imu_a": "TSK_TERRAIN_IMU", "imu_b": "TSK_TERRAIN_IMU; TSK_NAV_PLANNER resets `hump_max`",
    "imu_c": "TSK_TERRAIN_IMU; TSK_NAV_PLANNER resets `humps`",
    "obs_a": "TSK_OBSTACLE_SCAN", "obs_b": "TSK_OBSTACLE_SCAN",
    "obs_c": "TSK_OBSTACLE_SCAN; TSK_NAV_PLANNER resets `obstacles`", "misc": "TSK_TELEMETRY",
}
DSP = "Critical_Section, dispatch disabled (`crit_dsp_enter/exit`)"
IRQ = "Critical_Section, interrupts masked"
ONE = "single writer, one aligned store"
SPSC = "single-producer single-consumer ring index"
UNITS = [
    ("g_buzz", "8", "any task, TSK_SAFETY_MON (`buzz_put`, `buzzer_pattern_take`)", DSP),
    ("g_net_flags", "8", "radio-owner task, TSK_SAFETY_MON, TSK_TELEMETRY", DSP),
    ("g_estop_rx_us", "32", "radio-owner task, inside the section that sets `estop_pending`", DSP),
    ("g_estop_pwm_off_us", "16", "TSK_SAFETY_MON", ONE),
    ("g_done", "32", "TSK_MOTION_CTRL (one composed store inside a section; readers one load)", DSP),
    ("g_odo_rec[5]", "5 x 32", "TSK_MOTION_CTRL (`odometry_put`); readers copy the record in one section", DSP),
    ("g_map_w[3]", "3 x 32", "TSK_TELEMETRY (setters, three stores in one section); TSK_NAV_PLANNER reads (`nav_map_get` is one load)", DSP),
    ("g_hb[0..2]", "8 each", "TSK_MOTION_CTRL, TSK_LINE_BARCODE, TSK_TERRAIN_IMU (one element each); TSK_SAFETY_MON reads", ONE),
    ("g_reacq_seq", "8", "TSK_OBSTACLE_SCAN; TSK_LINE_BARCODE reads", ONE),
    ("g_sat_count", "16", "any task through `sat_note()`", DSP),
    ("g_params.bits", "32", "TSK_TELEMETRY only (one composed store)", ONE),
    ("g_estop_reported", "8 (`bool`, not packed: R2)", "TSK_SAFETY_MON; TSK_TELEMETRY reads", ONE),
    ("g_next_id", "16", "`new_id()` in `tsk_motion.c`, called by every task that posts a motion command (read-modify-write inside the section)", DSP),
]
for k in STATUS_WRITERS:
    UNITS.append(("g_status.%s" % k, "32", STATUS_WRITERS[k], DSP))
ISR_UNITS = [
    ("g_enc[0..1], g_odo, g_enc_err", "32 each", "GPIO ISR (`hal_gpio_isr`)", ONE),
    ("g_enc_rev", "8", "task (`hal_enc_set_reversed`, one composed store)", ONE),
    ("g_echo", "32", "GPIO ISR (echo edges); `hal_us_arm` (TSK_OBSTACLE_SCAN)", IRQ),
    ("g_echo_rise_us", "32", "GPIO ISR", ONE),
    ("g_adc_last", "16", "timer ISR (`hal_timer_isr`)", ONE),
    ("g_adc_rng", "32", "timer ISR; `hal_ir_adc_range` (TSK_LINE_BARCODE)", IRQ),
    ("g_bc_flags", "8", "timer ISR (black level); `hal_barcode_config`, `hal_barcode_raw` (TSK_LINE_BARCODE)", IRQ),
    ("g_bc_thr", "32", "`hal_barcode_config` (TSK_LINE_BARCODE), one store with sampling off", IRQ),
    ("g_bc_ring[64]", "32 each", "timer ISR (producer)", SPSC),
    ("g_bc_head", "8", "timer ISR", SPSC),
    ("g_bc_tail", "8", "TSK_LINE_BARCODE (consumer); reset inside a `crit_irq` section", SPSC),
    ("g_bc_overruns", "32", "timer ISR", ONE),
    ("g_raw[256]", "32 each", "timer ISR (producer)", SPSC),
    ("g_raw_head", "16", "timer ISR", SPSC),
    ("g_raw_tail", "16", "TSK_LINE_BARCODE (consumer)", SPSC),
    ("g_vbat_mv", "16", "timer ISR", ONE),
    ("g_vbat_div", "8", "timer ISR", ONE),
    ("g_motor_flags", "8", "TSK_SAFETY_MON; `app_main` start-up failure path (whole-unit store)", ONE),
    ("g_servo_rng", "32", "`hal_servo_range` (task)", ONE),
    ("g_imu_cfg", "16", "`hal_imu.c` probe and init functions (TSK_TERRAIN_IMU only, composed store); GPIO ISR reads", ONE),
]


def unit_table(items):
    out = ["| Storage_Unit | Width (bits) | Writing contexts | Protection |", "|---|---|---|---|"]
    for u, w, wr, pr in items:
        out.append("| `%s` | %s | %s | %s |" % (u, w, esc(wr), pr))
    return "\n".join(out)


# fault bit names from fault_mask.h
fm = rd("fault_mask.h")
FAULTS = [(m.group(1), int(m.group(2)), (m.group(3) or "").strip(" /*")) for m in
          re.finditer(r"#define (FAULT_\w+)\s+\(1u << (\d+)\)\s*(/\*.*?\*/)?", fm)]


def faults_table():
    out = ["| Bit | Name | Note |", "|---|---|---|"]
    for n, b, note in FAULTS:
        out.append("| %d | `%s` | %s |" % (b, n, note))
    return "\n".join(out)


# parameters
pp = rd("app_params.c")
PARAM_ROWS = []
for m in re.finditer(r'\{ "(\w+)",\s*(\(float \*\)0|&g_params\.\w+),\s*([^,]+),\s*([^,]+),\s*(\w+) \}', pp):
    PARAM_ROWS.append((m.group(1), m.group(2), m.group(3).strip(), m.group(4).strip(), m.group(5)))
if len(PARAM_ROWS) != PARAM_COUNT:
    sys.exit("gen_docs: parsed %d parameters from app_params.c, wire_dump has %d" % (len(PARAM_ROWS), PARAM_COUNT))
ph = rd("app_params.h")
PRM = {m.group(1): (int(m.group(2)), int(m.group(3)), m.group(4)) for m in
       re.finditer(r"#define PRM_(\w+)\s+PRM_DESC\(\s*(\d+)u,\s*(\d+)u,\s*(PE_\w+)\)", ph)}
rc = rd("robot_config.h")
CAL = set(m.group(1) for m in re.finditer(r"#define (\w+)\s+\(?CAL_UNSET\)?", rc))
GAINS = {"SPEED_KP", "SPEED_KI", "LINE_STEER", "LINE_STEER_RAMP", "LINE_HOLD_STEER", "LINE_CURVE_SLOW", "LINE_BIAS_KI",
         "BARCODE_HYS", "BARCODE_DET_HYS", "BARCODE_ADAPT_SWING"}


def float_reason(name):
    if name in GAINS:
        return "control gain or ratio: needs fine resolution over several decades and is used directly in float arithmetic"
    if name in CAL or name + "_X10" in CAL or name in ("SPEED_FF_MM_S_PER_PCT",):
        return "calibration value typed as an arbitrary decimal; may hold `CAL_UNSET` (-1) outside its min..max"
    return "tuning or geometry value typed as an arbitrary decimal and read in float arithmetic in a control loop"


ENC_NAME = {"PE_FLAG": "F1 (0..1)", "PE_TRI": "S2 (-1..1)", "PE_AXIS": "B2 + (-1) (-1..2)", "PE_LEVELS": "B3 + 1 (1..6)"}


def params_table():
    out = ["| ID | NAME | Min | Max | Representation | Storage (bit offset, width in `g_params.bits`) | Reason it stays float32 |",
           "|---|---|---|---|---|---|---|"]
    n_packed = 0
    for i, (name, pv, mn, mx, desc) in enumerate(PARAM_ROWS):
        w = WD["params"][i]
        if w["name"] != name:
            sys.exit("gen_docs: parameter %d is %s in app_params.c but %s in wire_dump" % (i, name, w["name"]))
        if pv.startswith("&"):
            out.append("| %d | `%s` | %s | %s | `float` (32), member `%s` | not packed | %s |" % (i, name, num(w["min"]), num(w["max"]), pv[10:], float_reason(name)))
        else:
            sh, wd, enc = PRM[name]
            if w["store_width"] != wd:
                sys.exit("gen_docs: %s store width differs" % name)
            n_packed += 1
            out.append("| %d | `%s` | %s | %s | %s | bit %d, width %d | packed |" % (i, name, num(w["min"]), num(w["max"]), ENC_NAME[enc], sh, wd))
    return "\n".join(out), n_packed


PRIVATE = [
    ("`wheel_ctl_t` `target`, `speed`, `integ`, `duty` (`g_w[2]`)", "TSK_MOTION_CTRL", "`float`",
     "PI loop state: the integrator and filtered speed add small increments every 20 ms; quantizing them adds limit-cycle error (Requirement 4.8)"),
    ("`primitive_t` (`g_prim`): `target_mm`, `speed`, `dir`, `start_r`, `start_l`, `t_start`, `timeout_ms`", "TSK_MOTION_CTRL", "`float`, `int32_t`, `uint32_t`",
     "primitive progress, computed from encoder differences each cycle"),
    ("`g_line_l`, `g_line_r`, `g_line_t`, `g_scale`, odometry accumulators, `a_filt`, `v_prev`", "TSK_MOTION_CTRL", "`float`, `uint32_t`",
     "integrated values; quantized only when published in `g_odo_rec` and `g_status`"),
    ("32-bit encoder counters read by the motion task (`hal_enc_count`, `hal_odometer`)", "TSK_MOTION_CTRL", "`int32_t`, `uint32_t`",
     "full modular 32-bit counters, one field per unit (Requirements 10.5, 10.6); a narrower field needs wrap handling in the GPIO ISR"),
    ("`line_ctx_t` (`g_line`): `black_s`, `episode_mm`, `hold_left_mm`, `since_black_mm`, `bias`, `b_last_curve`, `b_both_prev`; `search_ctx_t` (`g_search`) angles and `b_second`; `g_searching`, `g_det_on`", "TSK_LINE_BARCODE",
     "`float`, `bool`", "steering integrators and distance accumulators fed every 10 ms; the `bool`s are not packed (ledger 10.1, R2)"),
    ("`g_path_init`, `g_path_r`, `g_path_l`, `g_path_um`, `g_path_frac`", "TSK_LINE_BARCODE", "`bool`, `int32_t`, `uint32_t`, `float`",
     "odometer path accumulators in um (the barcode widths are differences of these)"),
    ("barcode decoder working storage: `det_t`, `adt_t`, `bcm_t` (sums, positions, `b_inv`, `b_started`, `b_black`, `b_have_prev`, `rev[]`, `b_pass`, `b_settle`, `b_conflict`), `bc_ctx_t` `start_um`, `edge_um`, `min_um`, `max_um`, `bc_params_t`", "TSK_LINE_BARCODE",
     "`uint32_t`, `int32_t`, `float`, `bool`", "algorithm working storage in um and ADC sums; Requirement 5.9 requires identical decode results; the `bool`s are not packed (R2)"),
    ("barcode width arrays: `bc_ctx_t.w[96]` is U16 (see the line and barcode section); `bcm_t` `last_n[]`, `res[]`, `last_res[]` are `uint8_t` per level", "TSK_LINE_BARCODE", "`uint16_t`, `uint8_t`",
     "per-element widths in um, one array per detector level"),
    ("`hump_ctx_t` `h_mm`, `peak_mm`, `len_mm`; pitch filter, `theta0`, `off_sum`, `lp[3]`", "TSK_TERRAIN_IMU", "`float`",
     "hump height integrator (`h += sin(theta) * ds`) and filter states; Requirement 7.7 needs the same event sample indices"),
    ("`mev_ctx_t` `still_ms`", "TSK_TERRAIN_IMU", "`uint32_t`", "timer, kept as in the Baseline_Build (design D5 says `int32_t`)"),
    ("`g_prof`, `g_fine`, `g_view`, `g_pose`, `g_legs[]`, `g_n_legs`, `g_block_cm`, `g_block_deg`, `g_backed_mm`, `g_rev_*`, `g_ext_mm[2]`, `g_seen_extra_mm`, `g_half_obs`, `g_prox_n`, `g_servo_now`, `us_filter_t` `r[3]`", "TSK_OBSTACLE_SCAN", "`float`, `uint32_t`",
     "bypass planner geometry and pose; Requirement 6.7 needs identical plans for identical scans, and the task is private so there is no race (ledger 10.3)"),
    ("`g_pending`, `g_t_start`", "TSK_NAV_PLANNER", "`char`, `uint32_t`", "navigation bookkeeping, one task"),
    ("`g_tone_left_ms`, `g_t_estop`", "TSK_SAFETY_MON", "`uint32_t`", "timers private to the task"),
    ("`g_raw_full_r/l`, `g_raw_ref_r/l`, `g_raw_gap`, `g_alarm_target`, `g_adc_*` reader state", "TSK_LINE_BARCODE / timer ISR", "`int32_t`, `uint32_t`, `bool`",
     "rebuilt full counts and the next alarm time; private to the reader or the ISR"),
    ("`app_net.c`: `g_led`, `g_led_applied`, `g_connecting`, `g_ever_connected`, `g_join_pending`, `g_broker_set`, `g_rssi`, `g_dropped`, `g_reconnects`, `g_published`, `g_pub_errors`, `g_last_status`, timers and back-offs", "radio-owner task", "`bool`, `uint32_t`, `int32_t`",
     "link bookkeeping; the `bool`s are not packed (R2). `g_dropped`, `g_reconnects`, `g_rssi` are shown in telemetry as U16 / U8 fields"),
    ("`g_f_head`, `g_f_tail`, `g_m_head`, `g_m_tail`, `g_cmd_head`, `g_cmd_tail`, `g_config_len`", "ring producers and consumers", "`uint32_t`",
     "free-running ring indices; the frame and message producers mask interrupts (`save_and_disable_interrupts`); see the scope section for the unmasked `g_m_tail++` in `flush()`"),
    ("`g_frames[64]` slots, `g_msgs[8]` slots, `g_cmds[8]` slots", "TSK_TELEMETRY, radio-owner task", "`uint8_t[]` byte payloads",
     "the slots already hold packed binary payloads (Frame_Ring 64 x 55 bytes, message slot 66 bytes, command slot 24 bytes)"),
]


def read_ledger():
    txt = (ROOT / "docs/cr1/PROGRESS_FW.md").read_text(encoding="utf-8")
    skipped = []
    for line in txt.splitlines():
        if line.startswith("|") and "skipped (minimum scope)" in line:
            cells = [c.strip() for c in line.strip().strip("|").split("|")]
            skipped.append((cells[0], cells[3] if len(cells) > 3 else ""))
    m = re.search(r"## Known differences from the spec text[^\n]*\n(.*?)\n## ", txt, re.S)
    known = [l[2:].strip() for l in m.group(1).splitlines() if l.startswith("- ")]
    return skipped, known


def read_flags():
    txt = (ROOT / "docs/cr1/REVIEW_FLAGS.md").read_text(encoding="utf-8")
    flags = []
    for m in re.finditer(r"^## (R\d+)\. (.*?)\n(.*?)(?=^## R\d+\.|\Z)", txt, re.S | re.M):
        dec = re.search(r"\*\*Decision:\*\*\s*(.*)", m.group(3))
        flags.append((m.group(1), m.group(2).strip(), dec.group(1).strip() if dec else "?"))
    return flags


FLAG_CODE = {
    "R1": "Spec followed as written: `FAULT_ENCODER_LOSS` is the only encoder / stall bit; bits 4 (`FAULT_ULTRASONIC`) and 12 (`FAULT_TELEMETRY`) are defined and never set; bits 13..15 are new (`fault_mask.h`).",
    "R2": "Partly applied. `bc_ctx_t` flags are packed (ledger 18.8). `det_t`, `adt_t`, `bcm_t` still hold `bool` members, and the `app_net.c` and `g_estop_reported` flags are plain `bool`s. They are listed in the private table.",
    "R3": "Spec followed (pool created without `TA_USERBUF`), so `g_mpf_buf` is not the real pool. The real pool sizes are in `docs/evidence/ram_usage.md`.",
    "R4": "Spec followed: the Map_Frame has four slots, one letter per movement. The pair form (several letters for one movement) is a recorded difference.",
    "R5": "Spec followed: `bp_quant` rounds in double (`bitpack.h`). `jb_fixed` rounds in float, so a rare boundary value can differ by one step from the Baseline_Build JSON.",
    "R6": "No task marked optional; the minimum-scope ledger instead lists every skipped task above.",
    "R7": "Resolved: the tools send and read binary frames only (the legacy JSON mode was removed on 5 Nov 2026). The robot accepts binary commands only.",
    "R8": "Did not arise: `obstacle_logic.c/.h` are identical to the Baseline_Build, and Property 14 compares them with the frozen Baseline_Build. `tsk_obstacle.c` changed for events, status accessors and the odometry record (ledger session 2); no obstacle refinement was made.",
}


def build_data_repr():
    skipped, known = read_ledger()
    flags = read_flags()
    params_md, n_packed = params_table()
    commit = subprocess.run(["git", "-C", str(ROOT), "rev-parse", "HEAD"], capture_output=True, text=True).stdout.strip()
    o = []
    o.append("# Data representation (Change Requirement 1)\n")
    o.append("Generated by `tools/evidence/gen_docs.py` from the headers, the drivers, `tests/host/build/wire_dump` and the target compiler. "
             "Every shift, width, offset and size below was read from the code, never typed. The Valid_Range, Resolution and Baseline_Build type columns are the "
             "specification; the script stops if a Required_Bits value it computes (`ceil(log2 N)`) differs from the width in the code.\n")
    o.append("| Record | Value |\n|---|---|\n| Firmware commit | `%s` |\n| Compiler | %s |\n| Baseline_Build | `bca79643d6f9bdd3938c353fa5f6fe791fc68f94` |\n| Format_Version | %d |\n"
             % (commit, GCC_VERSION, WD["format_version"]))
    o.append("Related: [protocol.md](protocol.md) (wire formats and code tables), [evidence/README.md](evidence/README.md) (RAM, message size and correctness evidence).\n")
    o.append("How to read the tables: `Bit offset` is the position of the field's least significant bit inside its Storage_Unit (for messages and the odometry record, "
             "inside the whole record; for MQTT payloads, the bit offset from the start of the frame, most significant bit first). "
             "Resolution `1` is used for integer, enumerated and flag fields. `Required_Bits = ceil(log2 N)` where N is the number of distinct values in the Valid_Range at the Resolution, sentinels included. "
             "Types: `S` signed two's complement, `U` unsigned, `E` enumeration, `F` flag, `M` modular counter.\n")
    for i, cat in enumerate(CATS, 1):
        o.append("## %d. %s\n" % (i, cat))
        if cat == CAT_VEH:
            o.append("Fault_Mask bits (`fault_mask.h`):\n\n" + faults_table() + "\n")
        o.append(render_rows(cat) + "\n")
        if cat == CAT_TASK:
            o.append("### Storage_Units shared between tasks: writers and protection\n")
            o.append(unit_table(UNITS) + "\n")
            o.append("`g_f_head/tail` (Frame_Ring) and `g_m_head/tail` (message slots): the producers are `net_post_frame()` and `net_post()`, which mask interrupts "
                     "(`save_and_disable_interrupts`) around the index and slot update; the consumer is the radio-owner task (`flush()` in `app_net.c`). "
                     "Both rings overwrite the oldest entry, so the tail has two writers; this is the baseline arrangement (design D3 note).\n")
        if cat == CAT_ISR:
            o.append("### Storage_Units shared with interrupt handlers: writers and protection\n")
            o.append(unit_table(ISR_UNITS) + "\n")
            o.append("Own-unit fields: `g_enc[]`, `g_odo`, `g_enc_err`, `g_bc_overruns` and `g_echo_rise_us` stay full 32-bit units, one field per unit "
                     "(Requirements 10.5, 10.6): the GPIO ISR adds to them on every encoder edge, and a narrower field would need wrap handling there, "
                     "while a unit shared with another field would need a masked section around each task write. See the comments above them in `hal_isr.c`.\n")
        if cat == CAT_MSG:
            o.append("The mailbox bodies are owned by one task at a time (sender until `tk_snd_mbx`, receiver after `tk_rcv_mbx`), so they need no protection. "
                     "`obst_msg_t` uses %d of its 192 bits (the spec's estimate was 181 and 28 bytes; the code rounds to 24 bytes).\n" % OBST_BITS)
        if cat == CAT_LINE:
            o.append("Line steering contexts (`line_ctx_t`, `search_ctx_t`) and the detector contexts keep unpacked members; see the private table (R2).\n")
        if cat == CAT_VEH:
            o.append("Task_Shared_Data and the Status_Snapshot units are in section 6; the Status_Snapshot writers are listed there.\n")
    o.append("## 10. Packed type sizes\n")
    o.append("Target sizes (`arm-none-eabi-gcc -c`, `arm-none-eabi-nm -S`), the same values the compile-time assertions check (`robot_status_t` 60 bytes in `status.h`).\n")
    o.append(sizes_table() + "\n")
    o.append("## 11. Parameters (all %d)\n" % PARAM_COUNT)
    o.append("The Parameter_Store keeps %d discrete parameters in one 32-bit word, `g_params.bits`, and %d as `float` members (design D15). "
             "Parameter_ID is the table position. The wire value of every parameter in SET and Config_Frame is float32; the Store representation shown here is what the robot keeps.\n"
             % (n_packed, PARAM_COUNT - n_packed))
    o.append(params_md + "\n")
    o.append("## 12. Private calculation variables\n")
    o.append("Kept at their computation type: each is private to one task (or one ISR), so there is no race, and each is either an accumulator, filter state or planner geometry "
             "whose quantization would change the behaviour (Requirement 19). They have no Required_Bits row.\n")
    o.append("| Variable | Task | Type | Reason |\n|---|---|---|---|")
    for v, t, ty, rs in PRIVATE:
        o.append("| %s | %s | %s | %s |" % (esc(v), t, ty, esc(rs)))
    o.append("")
    o.append("## 13. Scope decisions and known differences\n")
    o.append("### Skipped tasks (ledger `docs/cr1/PROGRESS_FW.md`, `skipped (minimum scope)`)\n")
    o.append("| Task | Reason |\n|---|---|")
    for t, r in skipped:
        o.append("| %s | %s |" % (esc(t), esc(r)))
    o.append("")
    o.append("### Known differences from the spec text (ledger)\n")
    for k in known:
        o.append("- " + k)
    o.append("")
    o.append("Differences found while writing this document (not in the ledger list above):\n")
    o.append("- `bc_ctx_t` uses C bit-fields (`n:7`, `last_n:7`, `fails:2`, `b_open:1`, `b_black:1`) for the in-task context, not the masks and shifts of `bitpack.h`. It is private to TSK_LINE_BARCODE, so the layout does not cross a boundary.")
    o.append("- `det_t`, `adt_t` and `bcm_t` (barcode decoder contexts) still hold `bool` members; ledger 18.8 packed only `bc_ctx_t` (R2).")
    o.append("- The message-slot consumer in `flush()` (`app_net.c`) does `g_m_tail++` outside a masked section while `net_post()` may also increment `g_m_tail` on overflow in a masked section. This is the Baseline_Build code and CR1 did not change it.")
    o.append("- The Telemetry_Frame is built by `wire.c` from the Status_Snapshot; `line_l` and `line_r` are S2 on the wire (-1, 0, 1) and U1 in the Status_Snapshot.")
    o.append("- The `json` blocks of the design D-table (one per category, for a document test) are not generated: task 20.7 is skipped, and the table is checked by hand against the headers.")
    o.append("")
    o.append("### Review flags R1 to R8 (`docs/cr1/REVIEW_FLAGS.md`)\n")
    o.append("| Flag | Subject | Decision in REVIEW_FLAGS.md | What the code does |\n|---|---|---|---|")
    for fid, title, dec in flags:
        o.append("| %s | %s | %s | %s |" % (fid, esc(title), esc(dec), esc(FLAG_CODE.get(fid, ""))))
    o.append("")
    return "\n".join(o)


def build_protocol():
    o = []
    o.append("# Wire protocol (Change Requirement 1)\n")
    o.append("Generated by `tools/evidence/gen_docs.py` from the JSON that `tests/host/build/wire_dump` prints (`wire_ids.h` tables and the frame layouts). "
             "The robot and both laptop codecs are compared with the same output by the host tests.\n")
    o.append("**Format_Version: %d.** Byte 0 bits 7..4 carry it in all seven payload types. Versions 4 and 5 are never assigned, because the Baseline_Build text commands start with 'A'..'Z'. "
             "Bits are written most significant first, a field may cross a byte boundary, multi-byte values are big-endian by that order, `F32` is IEEE-754 MSB first, padding bits are 0. "
             "Robot time is M24 ms. Field widths and ranges per field: [data_representation.md](data_representation.md).\n" % WD["format_version"])
    o.append("| Payload | Topic | Bits | Length (bytes) |\n|---|---|---|---|")
    topics = [("telemetry", "Telemetry_Frame", "`is02/telemetry`"), ("heartbeat", "Heartbeat_Frame", "`is02/status`"),
              ("event_header", "Event_Frame header", "`is02/event`"), ("ack_header", "Ack_Frame header", "`is02/ack`"),
              ("command_header", "Command_Frame header", "`is02/cmd`"), ("map", "Map_Frame", "`is02/map`"),
              ("config", "Config_Frame header", "`is02/config`")]
    for k, lab, tp in topics:
        f = WD["frames"][k]
        o.append("| %s | %s | %d | %s |" % (lab, tp, f["bits"], f["len"] if f["len"] else "variable"))
    o.append("")
    for k, lab, tp in topics:
        f = WD["frames"][k]
        o.append("## %s (%s)\n" % (lab, tp))
        o.append("| Bit offset | Width | Field | Kind | Resolution | Sentinel |\n|---|---|---|---|---|---|")
        for x in f["fields"]:
            s = x.get("sentinel", -1)
            o.append("| %d | %d | `%s` | %s | %s | %s |" % (x["offset"], x["width"], x["name"], x["kind"], x["resolution"] or "", "" if s in (-1, None) else s))
        o.append("")
    o.append("## Event_Codes\n")
    o.append("Arguments start at bit %d of the Event_Frame. Argument layouts per code: `app_program/evt.h` (authoritative widths are `WIRE_EVENT_TABLE` in `wire_ids.h`). "
             "The detail text of each event is rebuilt by the laptop codecs.\n" % WD["frames"]["event_header"]["bits"])
    o.append("| Code | Event | Argument bits | Variable |\n|---|---|---|---|")
    for e in WD["events"]:
        o.append("| %d | `%s` | %d | %s |" % (e["code"], e["name"], e["arg_bits"], "yes" if e["variable"] else "no"))
    o.append("")
    o.append("## Reason_Codes (Ack_Frame, E5)\n")
    o.append("| Code | Name | Baseline_Build text |\n|---|---|---|")
    for r in WD["reasons"]:
        o.append("| %d | `%s` | %s |" % (r["code"], r["name"], esc(r["text"])))
    o.append("")
    o.append("## Command codes (E4)\n")
    o.append("| Code | Name | Echo text | Command_Frame length | Ack_Frame length |\n|---|---|---|---|---|")
    for c in WD["commands"]:
        o.append("| %d | `%s` | `%s` | %s | %s |" % (c["code"], c["name"], c["echo"], c["cmd_len"] or "none", c["ack_len"] or "variable / none"))
    o.append("")
    o.append("## Stop reasons (E4)\n")
    o.append("| Code | Name | Baseline_Build text |\n|---|---|---|")
    for r in WD["stop_reasons"]:
        o.append("| %d | `%s` | %s |" % (r["code"], r["name"], esc(r["text"]) or "(empty)"))
    o.append("")
    o.append("## Abort reasons (E4)\n")
    o.append("| Code | Name | Baseline_Build text |\n|---|---|---|")
    for r in WD["abort_reasons"]:
        o.append("| %d | `%s` | %s |" % (r["code"], r["name"], esc(r["text"])))
    o.append("")
    o.append("## Parameter_IDs\n")
    o.append("%d parameters; the parameter table CRC (CRC-16/CCITT-FALSE over the NAMEs in ID order, each followed by `'\\n'`) is **%d** (0x%04X). "
             "Store width 32 means a float32 member; the SET and Config_Frame value is float32 for every parameter.\n" % (PARAM_COUNT, WD["param_table_crc"], WD["param_table_crc"]))
    o.append("| ID | NAME | Store width (bits) | Min | Max |\n|---|---|---|---|---|")
    for p in WD["params"]:
        o.append("| %d | `%s` | %d | %s | %s |" % (p["id"], p["name"], p["store_width"], num(p["min"]), num(p["max"])))
    o.append("")
    return "\n".join(o)


def main():
    (ROOT / "docs/data_representation.md").write_text(build_data_repr(), encoding="utf-8", newline="\n")
    (ROOT / "docs/protocol.md").write_text(build_protocol(), encoding="utf-8", newline="\n")
    print("wrote docs/data_representation.md and docs/protocol.md (%d field rows)" % len(ROWS))


if __name__ == "__main__":
    main()
