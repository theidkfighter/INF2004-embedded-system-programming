"""Continuous Packed_Field specs from the design tables (shared test data).

One ``FieldSpec`` per continuous field (fixed-point ``U<w>@r`` / ``S<w>@r``
and the integer-resolution fields quantized from a float) in:

- D2   Status_Snapshot           (name prefix ``status.<unit>.``)
- D3.4 odometry record           (``odo.``)
- D6   inter-task messages       (``msg.motion.<type>.``, ``msg.obstacle.``)
- D8   Telemetry_Frame           (``frame.telemetry.``)
- D10  Event_Frame arguments     (``frame.event.<event>.``)

Each spec carries the ``bp_qspec_t`` values (scale, lo_code, hi_code,
nan_code) that the firmware passes to ``bp_quant`` plus the field width,
signedness, sentinel and, for 32-bit Storage_Units, the bit shift.

Used by (keep the names and fields stable):
- test_prop01_rounding.py   (task 3.2, Property 1)
- test_prop02_saturation.py and test_nan.py (task 3.3, Property 2)
- the Property 3 sentinel test (task 3.4)

Rules from design D0: Resolution = 1 / scale; codes outside
[lo_code, hi_code] other than the sentinel are never stored; NaN stores the
sentinel when the field has one, otherwise lo_code.

Fields whose Valid_Range the design does not state are marked with a
``note``; their range is taken as the whole field width (unsigned 0 ..
2^w - 1), never using a code that the design reserves.
"""

from dataclasses import dataclass
from fractions import Fraction
from typing import Optional

__all__ = ["FieldSpec", "CONTINUOUS_FIELDS", "FIELDS_BY_NAME", "SENTINEL_FIELDS"]


@dataclass(frozen=True)
class FieldSpec:
    name: str                 # "<context>.<unit>.<field>", used as the test unit
    scale: int                # codes per unit of the value; Resolution = 1 / scale
    lo_code: int              # lower non-sentinel limit (code)
    hi_code: int              # upper non-sentinel limit (code)
    nan_code: int             # code stored for NaN (sentinel, else lo_code)
    width: int                # field width in bits
    signed: bool              # two's complement field
    sentinel: Optional[int] = None   # reserved code ("no echo"), None if none
    shift: Optional[int] = None      # bit shift in its 32-bit Storage_Unit (LSB = 0)
    unit: str = ""            # physical unit of the value (mm/s, cm, ...)
    note: str = ""            # assumption when the design leaves something open

    @property
    def resolution(self) -> Fraction:
        return Fraction(1, self.scale)

    @property
    def lo(self) -> Fraction:
        """Lower Valid_Range limit as an exact value."""
        return Fraction(self.lo_code, self.scale)

    @property
    def hi(self) -> Fraction:
        """Upper Valid_Range limit as an exact value."""
        return Fraction(self.hi_code, self.scale)

    @property
    def qspec(self):
        """(scale, lo_code, hi_code, nan_code), the bp_qspec_t / bpx_quant arguments."""
        return (self.scale, self.lo_code, self.hi_code, self.nan_code)


def _s(name, scale, limit_code, width, shift=None, unit="", note=""):
    """Signed symmetric field -limit..+limit, no sentinel (NaN -> lower limit)."""
    return FieldSpec(name, scale, -limit_code, limit_code, -limit_code, width, True,
                     None, shift, unit, note)


def _u(name, scale, hi_code, width, shift=None, unit="", sentinel=None, note=""):
    """Unsigned field 0..hi, NaN -> sentinel if any, else 0."""
    nan = sentinel if sentinel is not None else 0
    return FieldSpec(name, scale, 0, hi_code, nan, width, False, sentinel, shift, unit, note)


# Ultrasonic distance U13@0.1 cm, 0.0..431.0 cm, 8191 = no echo.
_US_HI, _US_NONE = 4310, 8191


def _us(name, shift=None):
    return _u(name, 10, _US_HI, 13, shift, "cm", sentinel=_US_NONE)


CONTINUOUS_FIELDS = (
    # ---- D2 Status_Snapshot ------------------------------------------------
    _s("status.mot_a.spd_l", 10, 10000, 15, 0, "mm/s"),
    _s("status.mot_a.spd_r", 10, 10000, 15, 15, "mm/s"),
    _s("status.mot_b.duty_l", 10, 1000, 11, 0, "%"),
    _s("status.mot_b.duty_r", 10, 1000, 11, 11, "%"),
    _u("status.mot_b.speed_scale", 100, 100, 7, 22, ""),
    _u("status.mot_e.dist_mm", 10, 1000000, 20, 0, "mm"),
    _s("status.imu_a.pitch", 10, 900, 11, 0, "deg"),
    _s("status.imu_a.hump_h", 10, 15000, 15, 11, "mm"),
    _u("status.imu_b.hump_peak", 10, 15000, 14, 0, "mm"),
    _u("status.imu_b.hump_max", 10, 15000, 14, 14, "mm"),
    _us("status.obs_a.obst_cm", 0),
    _us("status.obs_a.scan0", 15),
    _us("status.obs_b.scan1", 0),
    _us("status.obs_b.scan2", 13),
    _us("status.obs_c.scan3", 0),
    _us("status.obs_c.scan4", 13),
    _s("status.imu_c.turn_dps", 10, 7200, 14, 0, "deg/s"),
    # ---- D3.4 odometry record ------------------------------------------------
    _s("odo.dist_mm", 100, 100000000, 28, 0, "mm"),
    _s("odo.heading_deg", 100, 10000000, 25, 0, "deg"),
    _s("odo.v_mm_s", 100, 100000, 18, 0, "mm/s"),
    _s("odo.a_fwd_g", 10000, 40000, 17, 0, "g"),
    _s("odo.turn_dps", 100, 72000, 18, 0, "deg/s"),
    # ---- D6 motion command (motion_msg_t) -------------------------------------
    _s("msg.motion.MC_LINE.left", 10, 2500, 13, 0, "mm/s"),
    _s("msg.motion.MC_LINE.right", 10, 2500, 13, 13, "mm/s"),
    _s("msg.motion.MC_TURN.turn_deg", 10, 3600, 13, 0, "deg"),
    _s("msg.motion.MC_TURN.speed", 10, 2500, 13, 13, "mm/s"),
    _u("msg.motion.MC_TURN.timeout_ms", 1, 300000, 19, 0, "ms"),
    _s("msg.motion.MC_BYPASS.dist", 10, 30000, 16, 0, "mm"),
    _s("msg.motion.MC_BYPASS.speed", 10, 2500, 13, 16, "mm/s"),
    _u("msg.motion.MC_BYPASS.timeout_ms", 1, 300000, 19, 0, "ms"),
    _u("msg.motion.MC_SPEED_LIMIT.scale", 100, 100, 7, 0, ""),
    # ---- D6 obstacle profile (obst_msg_t) -------------------------------------
    _us("msg.obstacle.r_min", 3),
    _s("msg.obstacle.theta_l", 10, 600, 11, 16, "deg"),
    _s("msg.obstacle.theta_r", 10, 600, 11, 0, "deg"),
    _s("msg.obstacle.x_l", 10, 4000, 13, 11, "cm"),
    _s("msg.obstacle.x_r", 10, 4000, 13, 0, "cm"),
    _u("msg.obstacle.width", 10, 8000, 13, 13, "cm"),
    _s("msg.obstacle.offset", 10, 4000, 13, 0, "cm"),
    _s("msg.obstacle.clear_left", 10, 4000, 13, 13, "cm"),
    _s("msg.obstacle.clear_right", 10, 4000, 13, 0, "cm"),
    _us("msg.obstacle.scan0", 13),
    _us("msg.obstacle.scan1", 0),
    _us("msg.obstacle.scan2", 13),
    _us("msg.obstacle.scan3", 0),
    _us("msg.obstacle.scan4", 13),
    # ---- D8 Telemetry_Frame (MSB-first bitstream, no Storage_Unit shift) ------
    _s("frame.telemetry.spd_l", 10, 10000, 15, None, "mm/s"),
    _s("frame.telemetry.spd_r", 10, 10000, 15, None, "mm/s"),
    _s("frame.telemetry.duty_l", 10, 1000, 11, None, "%"),
    _s("frame.telemetry.duty_r", 10, 1000, 11, None, "%"),
    _u("frame.telemetry.dist", 10, 1000000, 20, None, "mm"),
    _s("frame.telemetry.pitch", 10, 900, 11, None, "deg"),
    _s("frame.telemetry.hump_h", 10, 15000, 15, None, "mm"),
    _u("frame.telemetry.hump_peak", 10, 15000, 14, None, "mm"),
    _u("frame.telemetry.hump_max", 10, 15000, 14, None, "mm"),
    _us("frame.telemetry.obst_cm"),
    _us("frame.telemetry.scan0"),
    _us("frame.telemetry.scan1"),
    _us("frame.telemetry.scan2"),
    _us("frame.telemetry.scan3"),
    _us("frame.telemetry.scan4"),
    _s("frame.telemetry.turn_dps", 10, 7200, 14, None, "deg/s"),
    # ---- D10 Event_Frame arguments ---------------------------------------------
    # HUMP_START / HUMP_PEAK / HUMP_END share one layout.
    _u("frame.event.HUMP.h", 10, 15000, 14, None, "mm"),
    _u("frame.event.HUMP.max", 10, 15000, 14, None, "mm"),
    _u("frame.event.MISSION_COMPLETE.dist", 100, 16383, 14, None, "m",
       note="Valid_Range not stated in D10; whole U14 width 0..163.83 m"),
    _u("frame.event.MISSION_COMPLETE.max", 10, 15000, 14, None, "mm"),
    _u("frame.event.BC_DBG.width", 10, 4095, 12, None, "mm",
       note="Valid_Range not stated in D10; whole U12 width 0..409.5 mm"),
    _u("frame.event.SIDE_FOLLOW.near", 10, 8191, 13, None, "cm",
       note="Valid_Range and sentinel not stated in D10; whole U13 width 0..819.1 cm"),
    _u("frame.event.SIDE_FOLLOW.limit", 10, 8191, 13, None, "cm",
       note="Valid_Range and sentinel not stated in D10; whole U13 width 0..819.1 cm"),
    _us("frame.event.CM13"),          # SIDE_LOOK d90 / d60
    _us("frame.event.OBSTACLE.cm"),
    # CM9: integer cm U9 0..431, 511 = no echo (FINE_SCAN, BYPASS_*, OBSTACLE_PROFILE r)
    _u("frame.event.CM9", 1, 431, 9, None, "cm", sentinel=511),
)

FIELDS_BY_NAME = {f.name: f for f in CONTINUOUS_FIELDS}
SENTINEL_FIELDS = tuple(f for f in CONTINUOUS_FIELDS if f.sentinel is not None)


def _check():
    """Table self-consistency: codes fit the width, shifts fit 32 bits."""
    assert len(FIELDS_BY_NAME) == len(CONTINUOUS_FIELDS), "duplicate field name"
    for f in CONTINUOUS_FIELDS:
        if f.signed:
            lo_min, hi_max = -(1 << (f.width - 1)), (1 << (f.width - 1)) - 1
        else:
            lo_min, hi_max = 0, (1 << f.width) - 1
        assert f.scale >= 1 and f.lo_code <= f.hi_code, f.name
        assert lo_min <= f.lo_code and f.hi_code <= hi_max, f"{f.name}: codes exceed U/S{f.width}"
        if f.sentinel is not None:
            assert lo_min <= f.sentinel <= hi_max, f.name
            assert not (f.lo_code <= f.sentinel <= f.hi_code), f"{f.name}: sentinel inside range"
            assert f.nan_code == f.sentinel, f.name
        else:
            assert f.nan_code == f.lo_code, f.name
        if f.shift is not None:
            assert f.shift + f.width <= 32, f.name


_check()
