"""Host_Test_Suite pytest set-up.

- ctypes loader for build/libis02.so (the firmware's plain C modules plus the
  host stubs, built by tests/host/Makefile).
- ctypes loader for build/libbase.so (the Baseline_Build modules, symbols
  prefixed base_, built by `make -C tests/host base`) for the differential
  properties 11, 13, 14, 15 and 22; the ``base`` fixture skips when it is
  missing.
- Hypothesis profile used by every property test.
- Result recorder: each test writes one JSON line to the file named by the
  IS02_RESULTS environment variable; run_all.sh turns those lines into the
  per-property summary (Requirements 18.8, 18.9).

Property tests mark themselves with

    # Feature: minimum-bit-data-representation, Property <n>: <title>
    @pytest.mark.prop(<n>, "<title>", unit="<payload type or Storage_Unit>")
    @settings(max_examples=1000)
    @given(...)
    def test_...(...):

If ``unit`` is omitted, a parametrize argument called ``unit`` (or the
parametrize id) names the payload type or Storage_Unit.
"""

import ctypes
import functools
import json
import os
from pathlib import Path

import pytest
from hypothesis import settings

HOST_DIR = Path(__file__).resolve().parent.parent
BUILD_DIR = HOST_DIR / "build"
LIBIS02_PATH = BUILD_DIR / "libis02.so"

# ---------------------------------------------------------------------------
# Hypothesis profile
# ---------------------------------------------------------------------------
# No example database: a run depends only on the seed passed by run_all.sh
# (--hypothesis-seed), so a printed seed reproduces the run.
settings.register_profile(
    "is02",
    max_examples=100,
    deadline=None,
    database=None,
    print_blob=False,
)
settings.load_profile(os.environ.get("IS02_HYPOTHESIS_PROFILE", "is02"))

# ---------------------------------------------------------------------------
# ctypes loader
# ---------------------------------------------------------------------------
_libs = {}


def load_library(path):
    """Load a host-built shared library once per session and return it."""
    path = Path(path)
    key = str(path)
    if key not in _libs:
        if not path.is_file():
            raise FileNotFoundError(
                f"{path} not found: build it with `make -C tests/host` "
                "(or run tests/host/run_all.sh)"
            )
        _libs[key] = ctypes.CDLL(key)
    return _libs[key]


def bind(lib, name, restype, *argtypes):
    """Set the C signature of lib.<name> and return the function."""
    fn = getattr(lib, name)
    fn.restype = restype
    fn.argtypes = list(argtypes)
    return fn


_u8, _u16, _u32 = ctypes.c_uint8, ctypes.c_uint16, ctypes.c_uint32
_bool, _vp = ctypes.c_bool, ctypes.c_void_p

# Signatures of the host stubs (stubs/host_stubs.h).
_STUB_SIGNATURES = {
    # crit_stub.c
    "crit_dsp_enter": (None,),
    "crit_dsp_exit": (None,),
    "crit_irq_enter": (_u32,),
    "crit_irq_exit": (None, _u32),
    "crit_stub_reset": (None,),
    "crit_stub_dsp_enters": (_u32,),
    "crit_stub_dsp_exits": (_u32,),
    "crit_stub_dsp_depth": (_u32,),
    "crit_stub_dsp_max_depth": (_u32,),
    "crit_stub_irq_enters": (_u32,),
    "crit_stub_irq_exits": (_u32,),
    "crit_stub_irq_depth": (_u32,),
    "crit_stub_irq_max_depth": (_u32,),
    "crit_stub_errors": (_u32,),
    "crit_stub_error_flags": (_u32,),
    "crit_stub_in_dsp": (_bool,),
    "crit_stub_in_irq": (_bool,),
    "crit_stub_watch": (_bool, _vp, _u32),
    "crit_stub_outside_writes": (_u32,),
    "crit_stub_changed_sections": (_u32,),
    # sat_stub.c
    "sat_note": (None, _u32),
    "sat_count": (_u16,),
    "sat_stub_reset": (None,),
    "sat_stub_set": (None, _u16),
    "sat_stub_calls": (_u32,),
    # os_stub.c
    "os_stub_reset": (None,),
    "os_stub_set_pool_empty": (None, _bool),
    "os_stub_post": (_bool, _u32, _vp, _u32),
    "os_stub_count": (_u32,),
    "os_stub_count_mbx": (_u32, _u32),
    "os_stub_rejected": (_u32,),
    "os_stub_get": (_u32, _u32, ctypes.POINTER(_u32), _vp, _u32),
    "os_stub_free": (None, _u32),
    "flag_set": (None, _u32),
    "flag_clear": (None, _u32),
    "flag_peek": (_u32,),
    "flag_take": (_bool, _u32, ctypes.POINTER(_u32)),
    "os_stub_flags_load": (None, _u32),
    "os_stub_flag_set_calls": (_u32,),
    # bitpack_export.c (wrappers of app_program/bitpack.h)
    "bpx_mask": (_u32, _u32),
    "bpx_get": (_u32, _u32, _u32, _u32),
    "bpx_put": (_u32, _u32, _u32, _u32, _u32),
    "bpx_sext": (ctypes.c_int32, _u32, _u32),
    "bpx_tc": (_u32, ctypes.c_int32, _u32),
    "bpx_set_checked": (_bool, ctypes.POINTER(_u32), _u32, _u32, _u32, _u32, _u32),
    "bpx_quant": (ctypes.c_int32, ctypes.c_float, ctypes.c_int32, ctypes.c_int32,
                  ctypes.c_int32, ctypes.c_int32, ctypes.POINTER(_bool)),
    "bpx_dequant": (ctypes.c_float, ctypes.c_int32, ctypes.c_int32),
    "bpx_sat_inc": (_u32, _u32, _u32),
}

# Values from stubs/host_stubs.h
CRIT_ERR_DSP_NESTED = 1 << 0
CRIT_ERR_DSP_UNDERFLOW = 1 << 1
CRIT_ERR_IRQ_UNDERFLOW = 1 << 2
CRIT_ERR_IRQ_TOKEN = 1 << 3
OS_STUB_MBX_MOTION, OS_STUB_MBX_NAV, OS_STUB_MBX_OBSTACLE, OS_STUB_MBX_TLM = 0, 1, 2, 3
OS_STUB_POOL_BLOCKS = 40
OS_STUB_MAX_BODY = 128


def load_is02():
    """Load build/libis02.so with the stub signatures bound."""
    first = str(LIBIS02_PATH) not in _libs
    lib = load_library(LIBIS02_PATH)
    if first:
        for name, (restype, *argtypes) in _STUB_SIGNATURES.items():
            bind(lib, name, restype, *argtypes)
    return lib


def reset_stubs(lib):
    """Put every host stub back to its start-up state.

    Call this at the start of each Hypothesis example (function-scoped
    fixtures run once per test, not once per example)."""
    lib.crit_stub_reset()
    lib.sat_stub_reset()
    lib.os_stub_reset()


def captured_messages(lib):
    """Return [(mbx, body_bytes), ...] posted through os_stub, oldest first."""
    out = []
    buf = ctypes.create_string_buffer(OS_STUB_MAX_BODY)
    mbx = _u32()
    for i in range(lib.os_stub_count()):
        n = lib.os_stub_get(i, ctypes.byref(mbx), buf, OS_STUB_MAX_BODY)
        out.append((mbx.value, buf.raw[:n]))
    return out


@pytest.fixture(scope="session")
def lib():
    """build/libis02.so, signatures bound. Stubs are not reset per example:
    call reset_stubs(lib) inside the test."""
    return load_is02()


# ---------------------------------------------------------------------------
# Baseline_Build differential library (build/libbase.so)
# ---------------------------------------------------------------------------
# Built by `make -C tests/host base` (baseline/fetch_baseline.sh with the
# BASELINE_COMMIT recorded in docs/evidence/README.md). Every symbol is
# prefixed base_: base_hump_step, base_bc_edge, base_bcm_sample,
# base_obstacle_profile, base_obstacle_choose_side, base_bypass_plan,
# base_params_set, base_g_params, ... Bind those in the differential tests
# with bind(base, "base_<name>", ...), using the Baseline_Build struct layouts.
LIBBASE_PATH = BUILD_DIR / "libbase.so"
LIBBASE_COMMIT_PATH = BUILD_DIR / "base" / "COMMIT"
LIBBASE_MISSING = (
    f"{LIBBASE_PATH} not found: build it with `make -C tests/host base` "
    "(Baseline_Build commit from docs/evidence/README.md, needs git)"
)

_f32, _i32, _cp = ctypes.c_float, ctypes.c_int32, ctypes.c_char_p

# Harness functions (baseline/base_*.c) and the Baseline_Build parameter API.
_BASE_SIGNATURES = {
    # base_motion.c (Property 11)
    "base_bh_motion_reset": (None,),
    "base_bh_timeout_for": (_u32, _f32, _f32),
    "base_bh_motion_turn": (_i32, _f32, _f32, ctypes.POINTER(_u32),
                            ctypes.POINTER(_f32), ctypes.POINTER(_i32)),
    "base_bh_motion_move": (_i32, _f32, _f32, ctypes.POINTER(_u32),
                            ctypes.POINTER(_f32), ctypes.POINTER(_i32)),
    # base_params.c + app_params.c (Property 22)
    "base_bh_param_name": (_cp, _u32),
    "base_bh_param_min": (_f32, _u32),
    "base_bh_param_max": (_f32, _u32),
    "base_bh_param_get": (_f32, _u32),
    "base_bh_param_find": (_i32, _cp),
    "base_params_init": (None,),
    "base_params_set": (_i32, _cp),
    "base_params_count": (_u32,),
    "base_params_mm_per_count": (_f32,),
    # base_hump.c (Property 15): argument is a hump_params_t *
    "base_bh_hump_params": (None, _vp),
}

# base_params_set() results (Baseline_Build app_params.h)
BASE_PARAM_OK, BASE_PARAM_UNKNOWN, BASE_PARAM_BAD_NUMBER, BASE_PARAM_RANGE = 0, -1, -2, -3


def base_commit():
    """Full hash of the Baseline_Build commit libbase.so was built from, or None."""
    try:
        return LIBBASE_COMMIT_PATH.read_text(encoding="ascii").strip() or None
    except OSError:
        return None


def load_base():
    """Load build/libbase.so with the harness signatures bound.

    Raises FileNotFoundError (with the build command) if it was not built."""
    if not LIBBASE_PATH.is_file():
        raise FileNotFoundError(LIBBASE_MISSING)
    first = str(LIBBASE_PATH) not in _libs
    base = load_library(LIBBASE_PATH)
    if first:
        for name, (restype, *argtypes) in _BASE_SIGNATURES.items():
            bind(base, name, restype, *argtypes)
    return base


def base_motion_target(base, kind, value, speed_mm_s):
    """Baseline_Build MC_TURN ("turn", value = degrees clockwise) or MC_BYPASS
    ("move", value = signed mm) for the default parameters.

    Returns (timeout_ms, target_mm, counts): counts is the encoder count at
    which the Baseline_Build primitive reports MOTION_OK (-1 if never)."""
    fn = {"turn": base.base_bh_motion_turn, "move": base.base_bh_motion_move}[kind]
    timeout, target, counts = _u32(), _f32(), _i32()
    base.base_bh_motion_reset()
    if not fn(value, speed_mm_s, ctypes.byref(timeout), ctypes.byref(target), ctypes.byref(counts)):
        raise RuntimeError(f"Baseline_Build motion_start_{kind} posted no command")
    return timeout.value, target.value, counts.value


@pytest.fixture(scope="session")
def base():
    """build/libbase.so (Baseline_Build, symbols prefixed base_). Skips the
    test with the build command if the library was not built."""
    try:
        return load_base()
    except FileNotFoundError as exc:
        pytest.skip(str(exc))


# ---------------------------------------------------------------------------
# Result recorder for run_all.sh
# ---------------------------------------------------------------------------
def pytest_configure(config):
    config.addinivalue_line(
        "markers",
        "prop(n, title, unit=None): property-based test for design Property n; "
        "unit names the payload type or Storage_Unit",
    )


def _seed(config):
    try:
        seed = config.getoption("--hypothesis-seed")
    except ValueError:
        seed = None
    return str(seed if seed is not None else os.environ.get("IS02_SEED", "-"))


def _unit(item, marker):
    unit = marker.kwargs.get("unit") if marker is not None else None
    if unit is None and marker is not None and len(marker.args) > 2:
        unit = marker.args[2]
    callspec = getattr(item, "callspec", None)
    if unit is None and callspec is not None:
        unit = callspec.params.get("unit", callspec.id)
    return str(unit) if unit is not None else "-"


def _leaf_errors(exc):
    """Flatten an exception group into its leaf exceptions."""
    subs = getattr(exc, "exceptions", None)
    if isinstance(exc, BaseException) and subs:
        leaves = []
        for sub in subs:
            leaves.extend(_leaf_errors(sub))
        return leaves
    return [exc]


def _failing_input(exc):
    """The shrunk failing example(s) Hypothesis attached to the exception."""
    parts = []
    for leaf in _leaf_errors(exc):
        notes = [n for n in getattr(leaf, "__notes__", []) if n.strip()]
        # "Falsifying example: ..." (older Hypothesis) or "Failing test case: ..."
        examples = [n for n in notes if n.startswith(("Falsifying", "Failing test case"))]
        text = "\n".join(examples or notes) or f"{type(leaf).__name__}: {leaf}"
        parts.append(text)
    return "\n".join(parts)


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_call(item):
    """Count the examples Hypothesis runs (every call of the inner test,
    including shrink steps after a failure)."""
    item._is02_cases = 0
    handle = getattr(item.obj, "hypothesis", None)
    inner = getattr(handle, "inner_test", None)
    if inner is not None:
        @functools.wraps(inner)
        def counted(*args, **kwargs):
            item._is02_cases += 1
            return inner(*args, **kwargs)

        handle.inner_test = counted
        try:
            yield
        finally:
            handle.inner_test = inner
    else:
        item._is02_cases = 1
        yield


@pytest.hookimpl(hookwrapper=True)
def pytest_runtest_makereport(item, call):
    outcome = yield
    report = outcome.get_result()
    path = os.environ.get("IS02_RESULTS")
    if not path:
        return
    # One record per test: from the call phase, or from setup if setup failed.
    if report.when == "setup" and not report.failed:
        return
    if report.when == "teardown" and not report.failed:
        return

    marker = item.get_closest_marker("prop")
    failed = 0
    failing_input = ""
    if report.failed and call.excinfo is not None:
        failed = len(_leaf_errors(call.excinfo.value))
        failing_input = _failing_input(call.excinfo.value)
    elif report.failed:
        failed = 1

    record = {
        "runner": "python",
        "kind": "prop" if marker is not None else "test",
        "n": marker.args[0] if marker is not None and marker.args else None,
        "title": marker.args[1] if marker is not None and len(marker.args) > 1 else item.name,
        "unit": _unit(item, marker),
        "cases": getattr(item, "_is02_cases", 0),
        "failed": failed,
        "skipped": bool(report.skipped),
        "seed": _seed(item.config),
        "phase": report.when,
        "nodeid": item.nodeid,
        "input": failing_input,
    }
    with open(path, "a", encoding="utf-8") as fh:
        fh.write(json.dumps(record) + "\n")
