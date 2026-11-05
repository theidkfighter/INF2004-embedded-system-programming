"""Unit tests of the laptop tools (task 14.18) with a mock MQTT client, so they
need no broker and no paho-mqtt: each action publishes the right topic, retain
flag and payload in both modes (flag R7), ack waits and messages, the monitor's
CSV and L1 to L6, the parameter-table mismatch warning.
"""

import argparse
import csv
import json
import sys
import threading
import types
from unittest import mock

import pytest

from codec_helpers import REPO_DIR, is02_codec as codec


class FakeInfo:
    def wait_for_publish(self, timeout=None):
        return None


class FakeClient:
    """Records every publish; the test's replies are delivered on publish."""

    last = None
    replies = []                        # (topic, payload) sent after each publish
    on_subscribe_replies = []           # (topic, payload) sent once connected

    def __init__(self, *args, **kwargs):
        self.published = []
        self.subscribed = []
        self.on_connect = None
        self.on_message = None
        self.on_disconnect = None
        FakeClient.last = self

    def connect(self, host, port=1883, keepalive=60):
        return 0

    def subscribe(self, topic, qos=0):
        self.subscribed.append(topic)

    def loop_start(self):
        if self.on_connect:
            self.on_connect(self, None, None, 0)
        self.deliver(FakeClient.on_subscribe_replies)

    def loop_stop(self):
        pass

    def disconnect(self):
        pass

    def deliver(self, messages):
        for topic, payload in messages:
            if self.on_message:
                self.on_message(self, None, types.SimpleNamespace(topic=topic, payload=payload))

    def publish(self, topic, payload=None, qos=0, retain=False):
        self.published.append((topic, payload, retain))
        self.deliver(FakeClient.replies)
        return FakeInfo()


def fake_paho():
    client = types.ModuleType("paho.mqtt.client")
    client.Client = FakeClient
    client.CallbackAPIVersion = types.SimpleNamespace(VERSION2=2)
    paho = types.ModuleType("paho")
    paho.mqtt = types.ModuleType("paho.mqtt")
    paho.mqtt.client = client
    return {"paho": paho, "paho.mqtt": paho.mqtt, "paho.mqtt.client": client}


with mock.patch.dict(sys.modules, fake_paho()):
    import is02_tool
    import robot_map
    import robot_params
    import telemetry_monitor as monitor


class FakeEvent(threading.Event):
    """An Event that never really waits, and records what it was asked to wait for."""

    waits = []

    def wait(self, timeout=None):
        FakeEvent.waits.append(timeout)
        return self.is_set()


@pytest.fixture(autouse=True)
def no_waiting(monkeypatch):
    FakeClient.last = None
    FakeClient.replies = []
    FakeClient.on_subscribe_replies = []
    FakeEvent.waits = []
    monkeypatch.setattr(is02_tool.threading, "Event", FakeEvent)
    monkeypatch.setattr(robot_params.threading, "Event", FakeEvent)
    for tool in (is02_tool, robot_params, robot_map):
        monkeypatch.setattr(tool.time, "sleep", lambda s: None)


def tool_args(**kw):
    kw.setdefault("legacy_json", False)
    return argparse.Namespace(broker="localhost", port=1883, **kw)


JSON_TELEMETRY = (b'{"seq":7,"t":3500,"state":"LINE_FOLLOWING","spd":[8.0,150.3],"duty":[41.2,40.9],'
                  b'"enc":[1234,1229],"dist":512.3,"line":{"l":0,"r":1,"ir":1790},"bc":"A",'
                  b'"hump":{"pitch":1.2,"h":0.0,"peak":0.0},"obst":{"cm":35.2,'
                  b'"scan":[-1.0,-1.0,20.0,-1.0,-1.0],"side":"R"},"rssi":-48,"buf":0}')
# what the Baseline_Build monitor wrote for it, after rx_time
JSON_ROW = ["7", "3500", "LINE_FOLLOWING", "8.0", "150.3", "41.2", "40.9", "1234", "1229", "512.3", "0", "1", "1790",
            "A", "1.2", "0.0", "0.0", "35.2", "-1.0", "-1.0", "20.0", "-1.0", "-1.0", "R", "-48", "0"]

TELEMETRY = {
    "seq": 7, "t": 3500, "state": 2, "spd_l": 80, "spd_r": 1503, "duty_l": 412, "duty_r": 409,
    "enc_l": 1234, "enc_r": 1229, "dist": 5123, "line_l": 0, "line_r": 1, "ir": 1790, "ir_min": 12,
    "ir_max": 3000, "thr_valid": 1, "thr_b": 2000, "thr_w": 900, "bc": 10, "pitch": 12, "hump_h": 0,
    "hump_peak": 0, "hump_max": 0, "us": 352, "scan": [8191, 8191, 200, 8191, 8191], "side": 2, "imu_ev": 1,
    "turn": 0, "faults": 0, "vbat": 7400, "rssi": 48, "buf": 0,
}

HEARTBEAT = {
    "seq": 1, "t": 1000, "state": 1, "rssi": 48, "buf": 0, "up": 1, "reconnects": 0, "dropped": 0, "sat": 0,
    "faults": 0, "ptab_count": codec.PARAM_COUNT, "ptab_crc": codec.PARAM_CRC, "broker": [10, 0, 0, 1],
}


def telemetry_frame(**fields):
    return codec.encode_fields("telemetry", dict(TELEMETRY, **fields))


def heartbeat_frame(**fields):
    return codec.encode_fields("heartbeat", dict(HEARTBEAT, **fields))


def ack_frame(cmd, reason=0, **args):
    return codec.encode_fields("ack", {"t": 1, "cmd": codec.WCMD[cmd], "reason": reason, "args": args})


def json_ack(cmd, result="ok", **extra):
    return json.dumps(dict({"t": 1, "cmd": cmd, "result": result}, **extra)).encode()


def bad_heartbeat():
    return heartbeat_frame(ptab_count=codec.PARAM_COUNT - 1)


def published():
    return FakeClient.last.published


# ---------------- telemetry_monitor.py ----------------

@pytest.fixture
def mon(tmp_path):
    def make(**kw):
        args = argparse.Namespace(broker="localhost", port=1883, out=str(tmp_path), quiet=True,
                                  legacy_json=kw.get("legacy_json", False))
        m = monitor.Monitor(args)
        m.connected = True
        return m
    return make


def rx(m, topic, payload):
    m.on_message(m.client, None, types.SimpleNamespace(topic=topic, payload=payload))


def csv_rows(m):
    m.csv_file.flush()
    with open(m.csv_path, newline="", encoding="utf-8") as f:
        return list(csv.reader(f))


def test_csv_header_is_the_baseline_one(mon):
    assert monitor.CSV_COLS == [
        "rx_time", "seq", "t_ms", "state", "spd_l", "spd_r", "duty_l", "duty_r", "enc_l", "enc_r", "dist_mm",
        "line_l", "line_r", "ir", "barcode", "pitch_deg", "hump_mm", "hump_peak_mm", "obst_cm", "scan_m60",
        "scan_m30", "scan_0", "scan_p30", "scan_p60", "scan_side", "rssi", "buf"]
    assert csv_rows(mon())[0] == monitor.CSV_COLS


def test_csv_row_of_json_input_is_unchanged(mon):
    m = mon()
    rx(m, monitor.T_TELEM, JSON_TELEMETRY)
    assert csv_rows(m)[1][1:] == JSON_ROW


def test_csv_row_of_binary_input_matches_the_json_one(mon):
    m = mon()
    rx(m, monitor.T_TELEM, telemetry_frame())
    assert csv_rows(m)[1][1:] == JSON_ROW


@pytest.mark.parametrize("legacy", [True, False])
def test_binary_and_json_are_read_in_both_modes(mon, legacy):
    m = mon(legacy_json=legacy)
    rx(m, monitor.T_TELEM, JSON_TELEMETRY)
    rx(m, monitor.T_TELEM, telemetry_frame(seq=8, t=4000))
    assert m.frames == 2 and m.rejected == 0 and m.gaps == 0


def test_commands_are_text_in_legacy_mode(mon):
    m = mon(legacy_json=True)
    for key in "erabcd":
        m.send(monitor.KEYS[key])
    m.ping()
    assert FakeClient.last.published == [("is02/cmd", c, False) for c in (
        "ESTOP", "RESUME", "BARCODE:A", "BARCODE:B", "BARCODE:C", "BARCODE:D", "PING:1")]


def test_commands_are_binary_by_default(mon):
    m = mon()
    m.send("ESTOP")
    m.ping()
    assert FakeClient.last.published == [("is02/cmd", codec.encode_command_text("ESTOP"), False),
                                         ("is02/cmd", codec.encode_command_text("PING:1"), False)]


def test_sequence_gaps_are_15_bit_modular(mon):
    m = mon()
    for seq in (32766, 32767, 0, 2, 2):                 # wrap, then one missing, then a duplicate
        rx(m, monitor.T_TELEM, telemetry_frame(seq=seq))
    assert (m.gaps, m.dups, m.restarts) == (1, 1, 0)
    rx(m, monitor.T_TELEM, telemetry_frame(seq=32000 + 2))   # difference above 16383: not a forward step
    assert m.restarts == 1


def test_robot_restart_still_counts_on_json(mon):
    m = mon()
    for seq in (100, 101, 3):
        rx(m, monitor.T_TELEM, JSON_TELEMETRY.replace(b'"seq":7', b'"seq":%d' % seq))
    assert (m.gaps, m.restarts, m.rate_frames) == (0, 1, 1)


def test_rate_uses_24_bit_modular_time(mon):
    m = mon()
    top = (1 << 24) - 500
    for i in range(5):                                  # 2 Hz across the wrap of t
        rx(m, monitor.T_TELEM, telemetry_frame(seq=i, t=(top + i * 500) % (1 << 24)))
    assert "[PASS] L1 telemetry rate: 2.00 Hz over 5 frames" in m.results()[0]


def l_result(m, name):
    return next(line for line in m.results() if name in line)


@pytest.mark.parametrize("period, verdict", [(556, "FAIL"), (555, "PASS"), (455, "PASS"), (454, "FAIL")])
def test_l1_limits(mon, period, verdict):
    m = mon()
    for i in range(101):
        rx(m, monitor.T_TELEM, telemetry_frame(seq=i, t=i * period))
    assert l_result(m, "L1").startswith(f"[{verdict}]")


def test_l2_counts_rejected_payloads_and_missing_fields(mon):
    m = mon()
    rx(m, monitor.T_TELEM, telemetry_frame())
    assert l_result(m, "L2").startswith("[PASS]")
    rx(m, monitor.T_TELEM, telemetry_frame()[:-1])      # wrong length
    rx(m, monitor.T_TELEM, b"{not json")
    rx(m, monitor.T_TELEM, b"hello")
    assert m.rejected == 3
    assert l_result(m, "L2").startswith("[FAIL]") and "rejected by the codec: 3" in l_result(m, "L2")
    m = mon()
    rx(m, monitor.T_TELEM, b'{"seq":1,"t":1}')
    assert l_result(m, "L2").startswith("[FAIL]")


def test_l3_fails_on_a_gap(mon):
    m = mon()
    for seq in (1, 2, 4):
        rx(m, monitor.T_TELEM, telemetry_frame(seq=seq))
    assert l_result(m, "L3").startswith("[FAIL]") and "1 missing" in l_result(m, "L3")


def test_l4_heartbeat_gap_threshold(mon, capsys):
    m = mon()
    rx(m, monitor.T_TELEM, telemetry_frame())
    m.status(100.0, {}, "{}")
    m.status(103.0, {}, "{}")
    assert m.hb_gaps == []
    m.status(106.1, {}, "{}")
    assert len(m.hb_gaps) == 1 and "heartbeat was missing" in capsys.readouterr().out
    assert l_result(m, "L4").startswith("[PASS]")


@pytest.mark.parametrize("payload", [json_ack("PING:3", "pong"), ack_frame("PING", 4, ping=3)])
def test_acks_match_on_the_ping_number(mon, payload):
    m = mon()
    m.hb_last_rx = monitor.time.time()
    m.send("PING:3")
    m.send("PING:4")
    rx(m, monitor.T_ACK, payload)
    assert m.acked == 1 and list(m.pending) == ["PING:4"]


@pytest.mark.parametrize("payload", [json_ack("BARCODE:C", "queued"), ack_frame("BARCODE", 1, letter=2)])
def test_acks_match_on_the_letter(mon, payload):
    m = mon()
    m.hb_last_rx = monitor.time.time()
    m.send("BARCODE:C")
    m.send("BARCODE:D")
    rx(m, monitor.T_ACK, payload)
    assert m.acked == 1 and list(m.pending) == ["BARCODE:D"]


@pytest.mark.parametrize("us, verdict", [(10000, "PASS"), (10001, "FAIL")])
def test_l5_and_l6(mon, us, verdict):
    m = mon()
    rx(m, monitor.T_TELEM, telemetry_frame())
    m.hb_last_rx = monitor.time.time()
    m.send("ESTOP")
    rx(m, monitor.T_ACK, ack_frame("ESTOP", 7, pwm_off_us=us))
    assert l_result(m, "L5").startswith("[PASS]")
    assert l_result(m, "L6").startswith(f"[{verdict}]")


def test_mismatch_warning_appears_and_clears(mon, capsys):
    m = mon()
    rx(m, monitor.T_TELEM, telemetry_frame())
    rx(m, monitor.T_STATUS, bad_heartbeat())
    out = capsys.readouterr().out
    assert "parameter table mismatch" in out
    assert f"robot {codec.PARAM_COUNT - 1} / 0x{codec.PARAM_CRC:04X}" in out
    assert f"tool {codec.PARAM_COUNT} / 0x{codec.PARAM_CRC:04X}" in out
    assert any(line.startswith("[WARN]") for line in m.results())
    rx(m, monitor.T_STATUS, bad_heartbeat())            # not repeated at 1 Hz
    assert capsys.readouterr().out == ""
    rx(m, monitor.T_STATUS, b'{"seq":1,"t":1,"state":"IDLE"}')      # a JSON heartbeat says nothing
    assert m.ptab_warning is not None
    rx(m, monitor.T_STATUS, heartbeat_frame())
    assert m.ptab_warning is None and "matches again" in capsys.readouterr().out
    assert not any(line.startswith("[WARN]") for line in m.results())


def test_text_and_binary_on_the_command_topics_are_logged(mon):
    m = mon()
    rx(m, monitor.T_CMD, b"PING:1")
    rx(m, monitor.T_CMD, codec.encode_command_text("PING:2"))
    rx(m, "is02/map", b"")
    m.log_file.flush()
    with open(m.log_path, encoding="utf-8") as f:
        text = f.read()
    assert "is02/cmd PING:1\n" in text and '"cmd":"PING:2"' in text


def test_dropped_frames_do_not_reach_the_handlers(mon, capsys):
    m = mon()
    rx(m, monitor.T_STATUS, heartbeat_frame()[:-1])
    rx(m, monitor.T_ACK, ack_frame("START")[:-1] + b"\x00\x00\x00")
    assert m.hb == 0
    assert "is02/status: dropped, version 1, length 24" in capsys.readouterr().out


def test_keyboard_help_still_splits(mon):
    assert "e  ESTOP" in monitor.__doc__.split("Keys", 1)[1].split("Needs:", 1)[0]


# ---------------- is02_tool.py ----------------

@pytest.mark.parametrize("text", ["START", "ESTOP", "PING:1", "GET:MAP", "BARCODE:K", "PING:Dabc", "NONSENSE"])
def test_cmd_is_text_in_legacy_mode(text):
    FakeClient.replies = [(is02_tool.T_ACK, json_ack(text))]
    is02_tool.cmd_cmd(tool_args(text=[text], legacy_json=True))
    assert published() == [("is02/cmd", text, False)]
    assert FakeEvent.waits[-1] == 4.0


@pytest.mark.parametrize("text", ["START", "ESTOP", "PING:1", "GET:MAP", "BARCODE:K", "GET:PARAMS"])
def test_cmd_is_binary_by_default(text):
    is02_tool.cmd_cmd(tool_args(text=[text]))
    assert published() == [("is02/cmd", codec.encode_command_text(text), False)]


def test_cmd_with_legacy_off_refuses_what_a_frame_cannot_carry():
    with pytest.raises(SystemExit, match="PING:Dabc"):
        is02_tool.cmd_cmd(tool_args(text=["PING:Dabc"]))
    assert FakeClient.last is None


def test_ack_timeout_prints_the_baseline_message(capsys):
    is02_tool.cmd_cmd(tool_args(text=["START"]))
    assert "no ack from the robot (is it on the hotspot and connected? check `monitor`)" in capsys.readouterr().out


@pytest.mark.parametrize("ack", [json_ack("START", "starting"), ack_frame("START", 2)])
def test_ack_prints_for_json_and_binary(ack, capsys):
    FakeClient.replies = [(is02_tool.T_ACK, ack)]
    is02_tool.cmd_cmd(tool_args(text=["START"]))
    out = capsys.readouterr().out
    assert "ACK  " in out and "starting" in out and "no ack" not in out


def test_ack_for_another_command_is_not_taken(capsys):
    FakeClient.replies = [(is02_tool.T_ACK, ack_frame("STOP", 3))]
    is02_tool.cmd_cmd(tool_args(text=["START"]))
    assert "no ack from the robot" in capsys.readouterr().out


def test_set_in_both_modes():
    is02_tool.cmd_set(tool_args(pairs=["LINE_BASE_MM_S=100"], legacy_json=True))
    assert published() == [("is02/cmd", "SET:LINE_BASE_MM_S=100", False)]
    assert FakeEvent.waits[-1] == 3
    is02_tool.cmd_set(tool_args(pairs=["LINE_BASE_MM_S=100"]))
    assert published() == [("is02/cmd", codec.encode_command_text("SET:LINE_BASE_MM_S=100"), False)]


@pytest.mark.parametrize("legacy", [True, False])
@pytest.mark.parametrize("bad, name", [("NOPE=1", "NOPE"), ("LINE_BASE_MM_S=abc", "LINE_BASE_MM_S"),
                                       ("LINE_BASE_MM_S=9999", "LINE_BASE_MM_S")])
def test_set_validation_names_the_argument_and_publishes_nothing(bad, name, legacy):
    with pytest.raises(SystemExit, match=name):
        is02_tool.cmd_set(tool_args(pairs=["TURN_MM_S=80", bad], legacy_json=legacy))
    assert FakeClient.last is None


def test_map_in_legacy_mode_keeps_baseline_text_even_where_a_frame_could_not():
    is02_tool.cmd_map(tool_args(pairs=["f=fwd", "K=LEFT"], clear=False, legacy_json=True))
    assert published() == [("is02/map", "F=FWD,K=LEFT", True)]
    is02_tool.cmd_map(tool_args(pairs=["A=FWD", "B=FWD"], clear=False, legacy_json=True))     # flag R4: legal as text
    assert published() == [("is02/map", "A=FWD,B=FWD", True)]


def test_map_binary():
    is02_tool.cmd_map(tool_args(pairs=["F=FWD", "K=LEFT", "R=RIGHT", "U=UTURN"], clear=False))
    assert published() == [("is02/map", codec.encode_map("F=FWD,K=LEFT,R=RIGHT,U=UTURN"), True)]


@pytest.mark.parametrize("legacy, cmd", [(True, "MAP:CLEAR"), (False, codec.encode_command_text("MAP:CLEAR"))])
def test_map_clear(monkeypatch, legacy, cmd):
    clients = []
    monkeypatch.setattr(is02_tool.mqtt, "Client", lambda *a, **k: clients.append(FakeClient()) or clients[-1])
    is02_tool.cmd_map(tool_args(pairs=[], clear=True, legacy_json=legacy))
    assert clients[0].published == [("is02/map", b"", True)]
    assert clients[1].published == [("is02/cmd", cmd, False)]


def write_config(tmp_path, text):
    path = tmp_path / "cal.txt"
    path.write_text(text, encoding="utf-8")
    return str(path)


CONFIG = "# calibration\n\nLINE_BASE_MM_S=80   # slow\nTURN_MM_S=70\n"


def test_config_in_both_modes(tmp_path):
    path = write_config(tmp_path, CONFIG)
    is02_tool.cmd_config(tool_args(file=path, legacy_json=True))
    assert published() == [("is02/config", CONFIG.strip(), True)]
    is02_tool.cmd_config(tool_args(file=path))
    assert published() == [("is02/config", codec.encode_config(CONFIG), True)]


@pytest.mark.parametrize("legacy", [True, False])
def test_config_validation_names_the_line(tmp_path, legacy):
    path = write_config(tmp_path, "\n\nLINE_BASE_MM_S=80\nNOPE=1\n")
    with pytest.raises(SystemExit, match="line 4: NOPE"):
        is02_tool.cmd_config(tool_args(file=path, legacy_json=legacy))
    assert FakeClient.last is None


@pytest.mark.parametrize("legacy", [True, False])
def test_config_refused_on_mismatch(tmp_path, legacy):
    FakeClient.on_subscribe_replies = [(is02_tool.T_STATUS, bad_heartbeat())]
    with pytest.raises(SystemExit, match="parameter table mismatch"):
        is02_tool.cmd_config(tool_args(file=write_config(tmp_path, CONFIG), legacy_json=legacy))
    assert published() == []


def test_config_published_when_the_table_matches_or_no_heartbeat_comes(tmp_path):
    path = write_config(tmp_path, CONFIG)
    FakeClient.on_subscribe_replies = [(is02_tool.T_STATUS, heartbeat_frame())]
    is02_tool.cmd_config(tool_args(file=path))
    assert len(published()) == 1
    FakeClient.on_subscribe_replies = [(is02_tool.T_STATUS, b'{"seq":1,"t":1}')]    # JSON robot
    is02_tool.cmd_config(tool_args(file=path))
    assert len(published()) == 1
    FakeClient.on_subscribe_replies = []
    is02_tool.cmd_config(tool_args(file=path))
    assert len(published()) == 1
    assert is02_tool.HEARTBEAT_WAIT in FakeEvent.waits


@pytest.mark.parametrize("payload, want", [
    (json_ack("START", "starting"), "starting"),
    (ack_frame("START", 2), "starting"),
])
def test_show_prints_acks(capsys, payload, want):
    is02_tool.show(is02_tool.T_ACK, payload, False)
    assert want in capsys.readouterr().out


@pytest.mark.parametrize("payload", [JSON_TELEMETRY, telemetry_frame()])
def test_show_prints_telemetry_the_same_for_json_and_binary(capsys, payload):
    is02_tool.show(is02_tool.T_TLM, payload, True)
    out = capsys.readouterr().out
    assert "LINE_FOLLOWING" in out and "spd [8.0, 150.3] duty [41.2, 40.9]" in out and "bc 'A' obst 35.2cm" in out


def test_show_drops_a_bad_frame_with_a_line(capsys):
    is02_tool.show(is02_tool.T_EVENT, b"\x10\x00", False)
    out = capsys.readouterr().out
    assert "is02/event: dropped, version 1, length 2" in out and "EVT" not in out


def test_log_columns(tmp_path, monkeypatch):
    path = tmp_path / "run.csv"
    monkeypatch.setattr(is02_tool.time, "sleep", mock.Mock(side_effect=KeyboardInterrupt))
    frame = ack_frame("START", 2)
    FakeClient.on_subscribe_replies = [(is02_tool.T_ACK, frame), (is02_tool.T_STATUS, b'{"seq":1}')]
    is02_tool.cmd_log(tool_args(file=str(path)))
    with open(path, newline="", encoding="utf-8") as f:
        rows = list(csv.reader(f))
    assert rows[0] == ["pc_time", "topic", "len", "hex", "decoded_json"]
    assert rows[1][1:4] == ["is02/ack", str(len(frame)), frame.hex()]
    assert json.loads(rows[1][4])["result"] == "starting"
    assert rows[2][1:] == ["is02/status", "9", b'{"seq":1}'.hex(), '{"seq":1}']


# ---------------- robot_params.py ----------------

def run_params(monkeypatch, *argv):
    monkeypatch.setattr(sys, "argv", ["robot_params.py", "--broker", "b", "--wait", "0", *argv])
    robot_params.main()


def test_params_text_is_the_baseline_text(tmp_path, monkeypatch):
    run_params(monkeypatch, "--file", write_config(tmp_path, CONFIG), "--legacy-json")
    assert published() == [("is02/config", "LINE_BASE_MM_S=80\nTURN_MM_S=70", True)]


def test_params_binary(tmp_path, monkeypatch):
    run_params(monkeypatch, "--file", write_config(tmp_path, CONFIG))
    assert published() == [("is02/config", codec.encode_config(CONFIG), True)]


def test_params_real_file_is_valid(monkeypatch):
    run_params(monkeypatch, "--legacy-json")
    assert published()[0][0] == "is02/config" and published()[0][2] is True
    assert "LINE_DO_BLACK_HIGH=1" in published()[0][1]
    run_params(monkeypatch)
    assert published()[0][0] == "is02/config" and isinstance(published()[0][1], bytes)


@pytest.mark.parametrize("legacy", [[], ["--legacy-json"]])
def test_params_validation_prints_line_and_name_and_sends_nothing(tmp_path, monkeypatch, legacy):
    path = write_config(tmp_path, "# c\nTURN_MM_S=70\nNOPE=1\nLINE_BASE_MM_S=9999\nnot a pair\n")
    with pytest.raises(SystemExit) as e:
        run_params(monkeypatch, "--file", path, *legacy)
    assert "line 3: NOPE" in str(e.value) and "line 4: LINE_BASE_MM_S" in str(e.value) and "line 5" in str(e.value)
    assert FakeClient.last is None


def test_params_clear(monkeypatch):
    run_params(monkeypatch, "--clear")
    assert published() == [("is02/config", "", True)]


@pytest.mark.parametrize("ack", [json_ack("CONFIG", "ok", applied=2, rejected=0),
                                 ack_frame("CONFIG", 0, applied=2, n_rej=0, rej=[])])
def test_params_ack(tmp_path, monkeypatch, capsys, ack):
    FakeClient.replies = [("is02/ack", ack)]
    monkeypatch.setattr(sys, "argv", ["robot_params.py", "--broker", "b", "--file", write_config(tmp_path, CONFIG)])
    robot_params.main()
    assert "robot: applied 2, rejected 0" in capsys.readouterr().out


def test_params_no_ack_prints_the_baseline_message(tmp_path, monkeypatch, capsys):
    run_params(monkeypatch, "--file", write_config(tmp_path, CONFIG))
    assert "no answer from the robot (offline?). It will apply the values when it connects." in capsys.readouterr().out


def test_params_refused_on_mismatch(tmp_path, monkeypatch):
    FakeClient.on_subscribe_replies = [("is02/status", bad_heartbeat())]
    with pytest.raises(SystemExit, match="parameter table mismatch"):
        run_params(monkeypatch, "--file", write_config(tmp_path, CONFIG))
    assert published() == []


# ---------------- robot_map.py ----------------

MAP_TEXT = "X=FWD,K=LEFT,Q=RIGHT,M=UTURN"
MAP_ARGS = ["K=LEFT", "Q=RIGHT", "X=FWD", "M=UTURN"]


def run_map(monkeypatch, *argv):
    monkeypatch.setattr(sys, "argv", ["robot_map.py", "--broker", "b", *argv])
    robot_map.main()


def test_map_text_is_the_baseline_text(monkeypatch):
    run_map(monkeypatch, "--wait", "0", "--legacy-json", *MAP_ARGS)
    assert published() == [("is02/map", MAP_TEXT, True)]


def test_map_binary_normalises_and_accepts_move_first(monkeypatch):
    run_map(monkeypatch, "--wait", "0", "LEFT=K", "Q=RIGHT", "X=FWD", "M=UTURN")
    assert published() == [("is02/map", codec.encode_map(MAP_TEXT), True)]


@pytest.mark.parametrize("argv, ack", [
    (["--legacy-json"] + MAP_ARGS, json_ack("MAP:" + MAP_TEXT, "ok", map="K=LEFT,M=UTURN,Q=RIGHT,X=FWD", n=4)),
    (MAP_ARGS, None),
])
def test_map_waits_for_the_map_ack(monkeypatch, capsys, argv, ack):
    acts = [0] * 26
    for letter, action in (("X", 1), ("M", 2), ("K", 3), ("Q", 4)):
        acts[ord(letter) - 65] = action
    FakeClient.replies = [("is02/ack", ack or ack_frame("MAP", 0, map=acts, n=4))]
    run_map(monkeypatch, *argv)
    assert "robot is using: K=LEFT,M=UTURN,Q=RIGHT,X=FWD  (4 letters)" in capsys.readouterr().out


def test_map_ack_with_nack(monkeypatch, capsys):
    FakeClient.replies = [("is02/ack", json_ack("MAP:" + MAP_TEXT, "nack: empty map"))]
    run_map(monkeypatch, "--legacy-json", *MAP_ARGS)
    assert "robot refused it: nack: empty map" in capsys.readouterr().out


def test_map_no_ack_prints_the_baseline_message(monkeypatch, capsys):
    run_map(monkeypatch, "--wait", "0", "--legacy-json", *MAP_ARGS)
    assert "no answer from the robot (offline?). It will get the mapping when it connects." in capsys.readouterr().out


@pytest.mark.parametrize("flag, cmd", [(["--clear"], "MAP:CLEAR"), (["--show"], "GET:MAP")])
def test_map_clear_and_show(monkeypatch, flag, cmd):
    run_map(monkeypatch, "--wait", "0", "--legacy-json", *flag)
    assert published()[-1] == ("is02/cmd", cmd, False)
    run_map(monkeypatch, "--wait", "0", *flag)
    assert published()[-1] == ("is02/cmd", codec.encode_command_text(cmd), False)
    if cmd == "MAP:CLEAR":
        assert published()[0] == ("is02/map", "", True)


# ---------------- binary by default ----------------

@pytest.mark.parametrize("name", ["telemetry_monitor.py", "is02_tool.py", "robot_params.py", "robot_map.py"])
def test_every_tool_has_legacy_off_by_default(name):
    text = (REPO_DIR / "MQTT" / name).read_text(encoding="utf-8")
    assert '"--legacy-json", dest="legacy_json", action="store_true", default=False' in text
    assert '"--no-legacy-json", dest="legacy_json", action="store_false"' in text


def test_binary_estop_is_a_one_byte_frame(mon):
    """Requirement 13.3: the E-stop is one byte, version 1 in the high nibble and the command code in the low."""
    expected = bytes([(codec.FORMAT_VERSION << 4) | codec.WCMD["ESTOP"]])
    assert codec.encode_command_text("ESTOP") == expected and len(expected) == 1
    m = mon()
    m.send("ESTOP")
    is02_tool.cmd_cmd(tool_args(text=["ESTOP"]))
    assert m.client.published == [("is02/cmd", expected, False)]
    assert published() == [("is02/cmd", expected, False)]


# ---------------- dashboard.html (it runs in a browser; this checks the source) ----------------

DASHBOARD = (REPO_DIR / "MQTT" / "dashboard.html").read_text(encoding="utf-8")


def test_dashboard_loads_the_codec_from_a_relative_path():
    assert '<script src="is02_codec.js"></script>' in DASHBOARD


def test_dashboard_keeps_the_line_speed_box():
    assert '<input id="spdTxt" type="number" min="3.00" max="15.00" step="0.01"' in DASHBOARD
    assert "const SPEED_MIN_CM_S = 3.00, SPEED_MAX_CM_S = 15.00;" in DASHBOARD


def test_dashboard_legacy_box_is_off_and_not_remembered():
    assert '<input type="checkbox" id="legacyJson" autocomplete="off">' in DASHBOARD
    assert 'saveSetting("legacy' not in DASHBOARD


def test_dashboard_fault_names_follow_the_mask():
    names = DASHBOARD.split("const FAULTS = [", 1)[1].split("]", 1)[0].replace("\n", " ").replace('"', "").split(",")
    assert [n.strip() for n in names][12:] == ["TELEMETRY", "WIFI_LOSS", "MQTT_LOSS", "US_NO_ECHO"]


# ---------------- codec helpers for the tools ----------------

def test_read_payload():
    assert codec.read_payload("is02/ack", b'{"cmd": "PING:1"}') == ({"cmd": "PING:1"}, '{"cmd": "PING:1"}')
    assert codec.read_payload("is02/cmd", b"PING:1") == (None, "PING:1")
    assert codec.read_payload("is02/map", MAP_TEXT) == (None, MAP_TEXT)
    assert codec.read_payload("is02/config", b"") == (None, "")
    assert codec.read_payload("is02/other", b"\xffhello") == (None, "�hello")
    decoded, text = codec.read_payload("is02/map", codec.encode_map(MAP_TEXT))
    assert decoded == {"map": "K=LEFT,M=UTURN,Q=RIGHT,X=FWD", "n": 4} and json.loads(text) == decoded
    lines = []
    assert codec.read_payload("is02/map", b"\x10", log=lines.append) == (None, None)
    assert len(lines) == 1
    assert codec.read_payload("is02/telemetry", b"hello", log=lines.append) == (None, None)
    assert "version 6" in lines[1]
    assert codec.read_payload("is02/telemetry", b"{bad", log=lines.append) == (None, "{bad")


def test_ptab_mismatch():
    good = {"ptab": {"count": codec.PARAM_COUNT, "crc": codec.PARAM_CRC}}
    assert codec.ptab_mismatch({}) is None and codec.ptab_mismatch(good) is None
    assert "0xBEEF" in codec.ptab_mismatch({"ptab": {"count": 3, "crc": 0xBEEF}})
