"""Property 16 (spec task 16.4): the detail text of an Event_Frame can be rebuilt.

The firmware's event builders (evt.c in build/libis02.so, posted to the os stub)
make the Event_Frame; the Python Laptop_Codec turns it back into the detail text,
which must equal the reference formatter (event_ref.py: Baseline_Build rules, the
firmware's own jb_fixed) and, for the golden table, the Baseline_Build texts
written down in vectors/event_golden.json. The JavaScript codec is checked on the
same frames in js/cross.test.js.
"""

import json
import random

import pytest
from hypothesis import given, settings
from hypothesis import strategies as st

import codec_gen as gen
import event_ref
from codec_helpers import REPO_DIR, is02_codec as codec

T16 = "Event detail text can be rebuilt"
GOLDEN = json.loads((REPO_DIR / "tests" / "host" / "vectors" / "event_golden.json").read_text(encoding="utf-8"))["events"]
EVENT_NAMES = [e[0] for e in codec.EVENTS]


# Feature: minimum-bit-data-representation, Property 16: Event detail text can be rebuilt
@pytest.mark.prop(16, T16)
@pytest.mark.parametrize("unit", EVENT_NAMES)
@settings(max_examples=200)
@given(seed=st.integers(0, 2 ** 32 - 1))
def test_p16_detail_text(unit, seed):
    rec = gen.gen_event(random.Random(seed), codec.EVC[unit])
    posted, data = event_ref.build_event(rec["code"], event_ref.build_ops(rec), rec["t"])
    assert posted, f"evt_post refused {rec}"
    assert data == codec.encode_fields("event", rec), "the builders and the codec disagree on the bytes"
    shown = codec.decode("is02/event", data, log=lambda line: None)
    assert shown["event"] == unit and shown["t"] == rec["t"]
    assert shown["detail"] == event_ref.ref_text(rec)


@pytest.mark.parametrize("entry", GOLDEN, ids=[f"{i}-{e['event']}" for i, e in enumerate(GOLDEN)])
def test_golden_texts(entry):
    """The Baseline_Build texts, written by hand from the post_event / event2 calls of commit bca7964."""
    posted, data = event_ref.build_event(codec.EVC[entry["event"]], entry["build"], 1000)
    assert posted
    shown = codec.decode("is02/event", data)
    assert (shown["event"], shown["detail"]) == (entry["event"], entry["text"])


def test_golden_table_covers_every_event_and_variant():
    assert {e["event"] for e in GOLDEN} == set(EVENT_NAMES)
    variants = {}
    for e in GOLDEN:
        variants.setdefault(e["event"], set()).add(e["text"])
    for name in ("NAV", "BC_DBG", "BYPASS_COMPLETE", "BYPASS_RETRY", "BYPASS_BLOCKED", "OBST_REVERSE_CAP",
                 "BYPASS_EXTEND", "OBSTACLE_BACKOFF", "OBSTACLE_RESCAN", "LINE_SEARCH", "TURN_SNAP"):
        assert len(variants[name]) >= 2, name
    assert len(variants["BC_DBG"]) >= 10          # off, window x4, pass x2 more, levels, widths x2
