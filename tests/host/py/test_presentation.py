"""Presentation layer of the Laptop_Codec against the Baseline_Build JSON (spec task
14.17, Requirements 11.2, 11.10, 18.7): every baseline key is there, the sentinels
show as the baseline showed them, and a payload with a wrong version or length is
dropped without any field. js/presentation.test.js does the same for JavaScript.
"""

import random

import pytest

import codec_gen as gen
from codec_helpers import is02_codec as codec

# Keys of the Baseline_Build telemetry and heartbeat JSON (bca7964 tsk_telemetry.c)
TELEMETRY_KEYS = {
    "seq": None, "t": None, "state": None, "spd": None, "duty": None, "enc": None, "dist": None,
    "line": {"l": None, "r": None, "ir": None, "irmn": None, "irmx": None, "thr": None},
    "bc": None,
    "hump": {"pitch": None, "h": None, "peak": None, "max": None},
    "obst": {"cm": None, "scan": None, "side": None},
    "imu": {"ev": None, "tr": None},
    "faults": None, "vbat": None, "rssi": None, "buf": None,
}
STATUS_KEYS = {"seq": None, "t": None, "state": None, "rssi": None, "buf": None, "up": None, "reconnects": None,
               "dropped": None, "faults": None, "broker": None}


def missing(shown, keys, path=""):
    out = []
    for key, sub in keys.items():
        if key not in shown:
            out.append(path + key)
        elif sub:
            out += missing(shown[key], sub, path + key + ".")
    return out


def telemetry(**changes):
    rec = gen.gen_telemetry(random.Random(3))
    rec.update(changes)
    return codec.decode("is02/telemetry", codec.encode_fields("telemetry", rec))


def test_telemetry_has_every_baseline_key():
    for seed in range(50):
        rec = gen.gen_telemetry(random.Random(seed))
        shown = codec.decode("is02/telemetry", codec.encode_fields("telemetry", rec))
        assert missing(shown, TELEMETRY_KEYS) == []
        assert isinstance(shown["state"], str) and isinstance(shown["bc"], str)


def test_status_has_every_baseline_key():
    for seed in range(50):
        rec = gen.gen_heartbeat(random.Random(seed))
        shown = codec.decode("is02/status", codec.encode_fields("heartbeat", rec))
        assert missing(shown, STATUS_KEYS) == []
        assert shown["broker"] == ".".join(str(o) for o in rec["broker"])


def test_sentinels_show_as_the_baseline_showed_them():
    shown = telemetry(us=8191, scan=[8191, 0, 4310, 8191, 12], bc=63, side=0, thr_valid=0, thr_b=0, thr_w=0)
    assert shown["obst"]["cm"] == -1.0
    assert shown["obst"]["scan"] == [-1.0, 0.0, 431.0, -1.0, 1.2]
    assert shown["bc"] == ""
    assert shown["obst"]["side"] == ""
    assert shown["line"]["thr"] == [0, 0]
    shown = telemetry(thr_valid=1, thr_b=2000, thr_w=900, bc=20, side=2)
    assert shown["line"]["thr"] == [2000, 900]
    assert shown["bc"] == "K"
    assert shown["obst"]["side"] == "R"


def valid_payloads():
    rng = random.Random(11)
    return [(kind, gen.TOPIC[kind], codec.encode_fields(kind, gen.gen_record(kind, rng))) for kind in gen.KINDS]


def bad_payloads(data):
    return [("bad version", bytes([(data[0] & 0x0F) | 0x20]) + data[1:]), ("one byte short", data[:-1]),
            ("one byte long", data + b"\x00"), ("zero length", b"")]


@pytest.mark.parametrize("kind,topic,data", valid_payloads(), ids=gen.KINDS)
def test_wrong_version_or_length_is_dropped_without_fields(kind, topic, data):
    assert codec.decode(topic, data, log=lambda line: None) is not None
    for why, bad in bad_payloads(data):
        logs = []
        with pytest.raises(codec.CodecError):
            codec.decode_fields(kind, bad)
        shown = codec.decode(topic, bad, log=logs.append)
        if why == "zero length" and topic in codec.RETAINED_TOPICS:
            assert shown is None and logs == []       # an empty retained payload is the broker deleting it
        else:
            assert shown is None, f"{kind} {why}"
            assert len(logs) == 1 and topic in logs[0], logs
