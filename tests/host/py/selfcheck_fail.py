"""Deliberately failing property for `run_all.sh --selfcheck` (Requirements 18.8, 18.9).

This file does not match pytest's ``test_*.py`` pattern, so a normal
``pytest py`` run (and so a normal ``run_all.sh``) never collects it.
``run_all.sh --selfcheck`` names it on the command line, which makes pytest
collect it; ``test_selfcheck.sh`` then checks that the summary reports the
property, the Storage_Unit, the shrunk input (``v=200``) and the seed, and
that ``run_all.sh`` exits non-zero.

Do not "fix" the assertion: the failure is the point.
"""

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

SELFCHECK_TITLE = "Selfcheck deliberately failing property (Python)"
SELFCHECK_UNIT = "selfcheck.u8"


@pytest.mark.prop(0, SELFCHECK_TITLE, unit=SELFCHECK_UNIT)
@settings(max_examples=100)
@given(v=st.integers(min_value=0, max_value=255))
def test_selfcheck_python(v):
    # False for 200..255; Hypothesis shrinks the counterexample to v=200.
    assert v < 200
