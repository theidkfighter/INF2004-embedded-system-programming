"""Checks of the Baseline_Build differential library build/libbase.so.

These are not property tests: they check that the library loads, that its
symbols carry the base_ prefix, that it was built from the commit recorded in
docs/evidence/README.md, and that the harness entry points return the
Baseline_Build values for a few hand-computed cases. The differential
properties (11, 13, 14, 15, 22) build on these entry points.
"""

import ctypes
import math
import re

import pytest

from conftest import (
    BASE_PARAM_BAD_NUMBER,
    BASE_PARAM_OK,
    BASE_PARAM_RANGE,
    BASE_PARAM_UNKNOWN,
    HOST_DIR,
    base_commit,
    base_motion_target,
)

EVIDENCE_README = HOST_DIR.parent.parent / "docs" / "evidence" / "README.md"

# Baseline_Build robot_config.h defaults
TRACK_WIDTH_MM = 130.0
MM_PER_COUNT = math.pi * 65.0 / (2.0 * 1260.0)
RAD_TO_DEG = 57.29578


class HumpParams(ctypes.Structure):
    """Baseline_Build hump_params_t (imu_logic.h)."""

    _fields_ = [
        ("start_deg", ctypes.c_float),
        ("level_deg", ctypes.c_float),
        ("min_height_mm", ctypes.c_float),
        ("start_samples", ctypes.c_uint32),
        ("end_samples", ctypes.c_uint32),
        ("max_len_mm", ctypes.c_float),
    ]


def test_built_from_recorded_commit(base):
    text = EVIDENCE_README.read_text(encoding="utf-8")
    m = re.search(r"^BASELINE_COMMIT=([0-9a-fA-F]{7,40})\s*$", text, re.MULTILINE)
    assert m, "no BASELINE_COMMIT=<hash> line in docs/evidence/README.md"
    assert base_commit() is not None
    assert base_commit().startswith(m.group(1).lower())


def test_symbols_are_prefixed(base):
    for name in ("hump_step", "bc_edge", "bc_poll", "bcm_sample", "obstacle_profile",
                 "obstacle_choose_side", "bypass_plan", "params_set", "g_params"):
        assert hasattr(base, "base_" + name), name
        with pytest.raises(AttributeError):
            getattr(base, name)


def test_turn_target_and_timeout(base):
    timeout, target, counts = base_motion_target(base, "turn", -90.0, 100.0)
    arc = 90.0 / RAD_TO_DEG * 0.5 * TRACK_WIDTH_MM
    assert target == pytest.approx(arc, abs=1e-3)
    assert timeout == int(2000.0 * target / 100.0) + 2000
    assert (counts - 1) * MM_PER_COUNT < target <= counts * MM_PER_COUNT + 1e-4


def test_bypass_target_and_timeout(base):
    timeout, target, counts = base_motion_target(base, "move", -300.0, 10.0)
    assert target == pytest.approx(300.0)
    assert timeout == int(2000.0 * 300.0 / 20.0) + 2000      # speed floor 20 mm/s
    assert (counts - 1) * MM_PER_COUNT < 300.0 <= counts * MM_PER_COUNT + 1e-4
    assert base.base_bh_timeout_for(300.0, -10.0) == timeout


def test_param_table_and_set(base):
    base.base_params_init()
    try:
        assert base.base_params_count() == 55
        i = base.base_bh_param_find(b"TRACK_WIDTH_MM")
        assert base.base_bh_param_name(i) == b"TRACK_WIDTH_MM"
        assert (base.base_bh_param_min(i), base.base_bh_param_max(i)) == (50.0, 300.0)
        assert base.base_bh_param_get(i) == TRACK_WIDTH_MM
        assert base.base_bh_param_find(b"NO_SUCH_PARAM") == -1
        assert base.base_bh_param_name(55) is None

        assert base.base_params_set(b"NO_SUCH_PARAM=1") == BASE_PARAM_UNKNOWN
        assert base.base_params_set(b"TRACK_WIDTH_MM=abc") == BASE_PARAM_BAD_NUMBER
        assert base.base_params_set(b"TRACK_WIDTH_MM=500") == BASE_PARAM_RANGE
        assert base.base_bh_param_get(i) == TRACK_WIDTH_MM
        assert base.base_params_set(b"SET:TRACK_WIDTH_MM=140") == BASE_PARAM_OK
        assert base.base_bh_param_get(i) == 140.0
        j = base.base_bh_param_find(b"WHEEL_DIAMETER_MM")
        assert base.base_bh_param_get(j) == 65.0
    finally:
        base.base_params_init()


def test_hump_params(base):
    p = HumpParams()
    base.base_bh_hump_params(ctypes.byref(p))
    assert (p.start_deg, p.level_deg, p.min_height_mm) == (5.0, 3.0, 15.0)
    assert (p.start_samples, p.end_samples, p.max_len_mm) == (3, 5, 1500.0)
