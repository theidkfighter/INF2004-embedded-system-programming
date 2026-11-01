"""
IS-02 telemetry monitor (laptop side).

Subscribes to the robot's MQTT topics, shows the telemetry live, checks it
(rate, message structure, missing frames, heartbeat) and saves everything to
logs/ (CSV of telemetry + a log of events, status and acks). Lets you send
commands to the robot from the keyboard.

    python telemetry_monitor.py --broker <laptop IP on the hotspot>

Keys (type the letter, then Enter):
    e  ESTOP          r  RESUME
    a/b/c/d  BARCODE:A..D override (what each does depends on the mapping on is02/map)
    p  PING (round-trip time)      s  show the checks so far      q  quit (prints the summary)

Needs: pip install paho-mqtt
"""

import argparse
import csv
import datetime as dt
import json
import os
import socket
import sys
import threading
import time

try:
    import paho.mqtt.client as mqtt
except ImportError as e:
    sys.exit(f"paho-mqtt is not installed for THIS Python ({sys.executable}).\n"
             f"Install it into the same Python with:\n"
             f"    \"{sys.executable}\" -m pip install paho-mqtt\n"
             f"(import error: {e})")

T_TELEM = "is02/telemetry"
T_STATUS = "is02/status"
T_EVENT = "is02/event"
T_CMD = "is02/cmd"
T_ACK = "is02/ack"

# Every field the brief lists for Buddy 1's telemetry, and where it is in the frame
REQUIRED = {
    "seq": "sequence number",
    "t": "robot time (ms)",
    "state": "current robot state",
    "spd": "motor speed [left, right] mm/s",
    "duty": "motor duty [left, right] %",
    "enc": "encoder counts [left, right]",
    "dist": "distance travelled (mm)",
    "line": "line sensor status {l, r, ir}: left/right 1 = black, barcode sensor ADC",
    "bc": "barcode detected",
    "hump": "hump measurements {pitch, h, peak}",
    "obst": "obstacle scan results {cm, scan[5], side}",
    "rssi": "Wi-Fi signal (dBm)",
    "buf": "frames waiting in the robot's offline buffer",
}
KEYS = {"e": "ESTOP", "r": "RESUME", "a": "BARCODE:A", "b": "BARCODE:B", "c": "BARCODE:C", "d": "BARCODE:D"}

CSV_COLS = ["rx_time", "seq", "t_ms", "state", "spd_l", "spd_r", "duty_l", "duty_r", "enc_l", "enc_r",
            "dist_mm", "line_l", "line_r", "ir", "barcode", "pitch_deg", "hump_mm", "hump_peak_mm", "obst_cm",
            "scan_m60", "scan_m30", "scan_0", "scan_p30", "scan_p60", "scan_side", "rssi", "buf"]


class Monitor:
    def __init__(self, args):
        self.args = args
        self.lock = threading.Lock()
        os.makedirs(args.out, exist_ok=True)
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.csv_path = os.path.join(args.out, f"telemetry_{stamp}.csv")
        self.log_path = os.path.join(args.out, f"messages_{stamp}.log")
        self.csv_file = open(self.csv_path, "w", newline="", encoding="utf-8")
        self.csv = csv.writer(self.csv_file)
        self.csv.writerow(CSV_COLS)
        self.log_file = open(self.log_path, "w", encoding="utf-8")

        # telemetry checks
        self.frames = 0
        self.bad_json = 0
        self.missing_fields = {}            # field -> count of frames without it
        self.last_seq = None
        self.gaps = 0                       # missing frames
        self.dups = 0
        self.restarts = 0
        self.first_t = None
        self.last_t = None
        self.rate_frames = 0
        self.max_buf = 0
        # heartbeat
        self.hb = 0
        self.hb_last_rx = None
        self.hb_max_gap = 0.0
        self.hb_gaps = []                   # (start, seconds) for gaps > 3 s
        # commands
        self.pending = {}                   # cmd -> send time
        self.sent = 0                       # sent while the robot was online
        self.sent_offline = 0               # sent while no heartbeat: not counted
        self.acked = 0
        self.rtts = []
        self.estop = []                     # (rtt_ms, pwm_off_us, wheels_stopped_ms)
        self.ping_n = 0
        self.connected = False

        self.client = self._make_client()

    # ---------------- MQTT ----------------
    def _make_client(self):
        try:
            client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                                 client_id=f"is02-laptop-{os.getpid()}")
        except AttributeError:              # paho-mqtt 1.x
            client = mqtt.Client(client_id=f"is02-laptop-{os.getpid()}")
        client.on_connect = self.on_connect
        client.on_message = self.on_message
        client.on_disconnect = self.on_disconnect
        return client

    def on_connect(self, client, userdata, flags, rc, properties=None):
        self.connected = True
        client.subscribe("is02/#")
        self.out(f"connected to broker {self.args.broker}:{self.args.port}, listening on is02/#")

    def on_disconnect(self, client, userdata, *rest):
        self.connected = False
        self.out("!! disconnected from the broker (is mosquitto still running?)")

    def on_message(self, client, userdata, msg):
        now = time.time()
        text = msg.payload.decode("utf-8", errors="replace")
        with self.lock:
            if msg.topic == T_TELEM:
                self.telemetry(now, text)
            elif msg.topic == T_STATUS:
                self.status(now, text)
            elif msg.topic == T_ACK:
                self.ack(now, text)
            elif msg.topic == T_EVENT:
                self.log(now, msg.topic, text)
                self.out(f"EVENT  {text}")
            elif msg.topic == T_CMD:
                self.log(now, msg.topic, text)
            else:
                self.log(now, msg.topic, text)

    # ---------------- handlers ----------------
    def telemetry(self, now, text):
        try:
            f = json.loads(text)
        except ValueError:
            self.bad_json += 1
            self.out(f"!! telemetry is not valid JSON: {text[:80]}")
            return
        self.frames += 1
        for k in REQUIRED:
            if k not in f:
                self.missing_fields[k] = self.missing_fields.get(k, 0) + 1

        seq = f.get("seq")
        t_ms = f.get("t")
        if isinstance(seq, int):
            if self.last_seq is not None:
                if seq == self.last_seq:
                    self.dups += 1
                elif seq < self.last_seq:
                    self.restarts += 1          # robot was reset: start counting again
                    self.first_t, self.rate_frames = None, 0
                elif seq > self.last_seq + 1:
                    self.gaps += seq - self.last_seq - 1
                    self.out(f"!! {seq - self.last_seq - 1} frame(s) missing before seq {seq}")
            self.last_seq = seq
        if isinstance(t_ms, (int, float)):
            if self.first_t is None:
                self.first_t = t_ms
                self.rate_frames = 0
            self.last_t = t_ms
            self.rate_frames += 1
        buf = f.get("buf", 0)
        if isinstance(buf, int):
            self.max_buf = max(self.max_buf, buf)

        spd = f.get("spd", [None, None])
        duty = f.get("duty", [None, None])
        enc = f.get("enc", [None, None])
        line = f.get("line", {})
        hump = f.get("hump", {})
        obst = f.get("obst", {})
        scan = (obst.get("scan") or [None] * 5) + [None] * 5
        self.csv.writerow([f"{now:.3f}", seq, t_ms, f.get("state"), spd[0], spd[1], duty[0], duty[1], enc[0], enc[1],
                           f.get("dist"), line.get("l"), line.get("r"), line.get("ir"), f.get("bc"), hump.get("pitch"), hump.get("h"),
                           hump.get("peak"), obst.get("cm"), scan[0], scan[1], scan[2], scan[3], scan[4],
                           obst.get("side"), f.get("rssi"), buf])
        self.csv_file.flush()
        if not self.args.quiet:
            def fmt(v, p=1):
                return "-" if v is None else (f"{v:.{p}f}" if isinstance(v, float) else str(v))
            self.out(f"#{seq:<5} {str(f.get('state')):<15} spd L{fmt(spd[0]):>6} R{fmt(spd[1]):>6}  "
                     f"enc {fmt(enc[0])}/{fmt(enc[1])}  dist {fmt(f.get('dist'), 0)}  "
                     f"LR {fmt(line.get('l'))}/{fmt(line.get('r'))}  ir {fmt(line.get('ir'))}  "
                     f"bc {f.get('bc') or '-'}  pitch {fmt(hump.get('pitch'))}  "
                     f"obst {fmt(obst.get('cm'))}  rssi {fmt(f.get('rssi'))}  buf {buf}")

    def status(self, now, text):
        self.log(now, T_STATUS, text)
        if self.hb_last_rx is not None:
            gap = now - self.hb_last_rx
            self.hb_max_gap = max(self.hb_max_gap, gap)
            if gap > 3.0:
                self.hb_gaps.append((self.hb_last_rx, gap))
                self.out(f"!! heartbeat was missing for {gap:.1f} s (robot offline?)")
        self.hb_last_rx = now
        self.hb += 1

    def ack(self, now, text):
        self.log(now, T_ACK, text)
        try:
            a = json.loads(text)
        except ValueError:
            self.out(f"ACK (not JSON): {text}")
            return
        cmd = a.get("cmd", "")
        sent = self.pending.pop(cmd, None)
        if sent is None:
            self.out(f"ACK    {text}")
            return
        rtt = (now - sent) * 1000.0
        self.acked += 1
        self.rtts.append(rtt)
        extra = ""
        if cmd == "ESTOP":
            self.estop.append((rtt, a.get("pwm_off_us")))
            extra = f"  robot: PWM off {a.get('pwm_off_us')} us after the command arrived"
        self.out(f"ACK    {cmd}: round trip {rtt:.0f} ms{extra}  {text}")

    # ---------------- output ----------------
    def log(self, now, topic, text):
        self.log_file.write(f"{dt.datetime.fromtimestamp(now).isoformat(timespec='milliseconds')} {topic} {text}\n")
        self.log_file.flush()

    def out(self, line):
        print(f"{dt.datetime.now().strftime('%H:%M:%S')} {line}", flush=True)

    def robot_online(self):
        return self.hb_last_rx is not None and (time.time() - self.hb_last_rx) < 3.0

    def send(self, cmd):
        if not self.connected:
            self.out("!! not connected to the broker")
            return
        with self.lock:
            if self.robot_online():
                self.pending[cmd] = time.time()
                self.sent += 1
            else:
                self.sent_offline += 1
                self.out("   (robot looks offline - no heartbeat in 3 s; this command is not counted)")
        self.client.publish(T_CMD, cmd, qos=1)
        self.log(time.time(), T_CMD, cmd)
        self.out(f"SENT   {cmd}")

    def ping(self):
        self.ping_n += 1
        self.send(f"PING:{self.ping_n}")

    # ---------------- summary ----------------
    def results(self):
        lines = []

        def res(ok, name, detail):
            lines.append(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")

        if self.frames == 0:
            res(False, "L1 telemetry received", f"no frames on {T_TELEM} - is the robot connected to this broker?")
            return lines
        if self.first_t is not None and self.last_t is not None and self.rate_frames > 1 and self.last_t > self.first_t:
            hz = (self.rate_frames - 1) / ((self.last_t - self.first_t) / 1000.0)
            res(1.8 <= hz <= 2.2, "L1 telemetry rate", f"{hz:.2f} Hz over {self.rate_frames} frames (robot clock; target 2 Hz)")
        else:
            res(False, "L1 telemetry rate", "not enough frames to measure")
        missing = ", ".join(f"{k} ({n})" for k, n in self.missing_fields.items())
        res(not self.missing_fields and self.bad_json == 0, "L2 message structure",
            f"{self.frames} frames, all {len(REQUIRED)} fields present" if not self.missing_fields and self.bad_json == 0
            else f"missing: {missing or '-'}; invalid JSON: {self.bad_json}")
        res(self.gaps == 0, "L3 no missing frames",
            f"{self.gaps} missing, {self.dups} duplicated, {self.restarts} robot restart(s); "
            f"largest offline buffer seen {self.max_buf} frames")
        res(self.hb > 0, "L4 heartbeat",
            f"{self.hb} received, longest gap {self.hb_max_gap:.1f} s, {len(self.hb_gaps)} gap(s) over 3 s "
            f"(expected only while the hotspot was off)")
        if self.sent:
            # unanswered commands sent during a link outage (heartbeat gap) or in the last 2 s don't count
            now = time.time()
            excused = 0
            for t_sent in self.pending.values():
                in_gap = any(start <= t_sent <= start + gap for start, gap in self.hb_gaps)
                if in_gap or now - t_sent < 2.0:
                    excused += 1
            expected = self.sent - excused
            avg = sum(self.rtts) / len(self.rtts) if self.rtts else 0.0
            res(self.acked >= expected, "L5 commands",
                f"{self.sent} sent, {self.acked} answered, {excused} lost during an outage or too late to answer; "
                f"round trip avg {avg:.0f} ms, max {max(self.rtts) if self.rtts else 0:.0f} ms")
        for rtt, pwm_us in self.estop:
            ok = isinstance(pwm_us, (int, float)) and pwm_us <= 10000
            res(ok, "L6 E-stop", f"laptop -> robot -> ack {rtt:.0f} ms; robot PWM off {pwm_us} us after the "
                                 f"command arrived (NFR21 <= 10 ms)")
        return lines

    def summary(self):
        with self.lock:
            lines = self.results()
        print("\n==================================================")
        print(" IS-02 telemetry monitor - summary")
        for line in lines:
            print(" " + line)
        print(f" CSV: {self.csv_path}")
        print(f" Log: {self.log_path}")
        print("==================================================")
        self.log_file.write("\n".join(lines) + "\n")
        self.csv_file.close()
        self.log_file.close()


def keyboard(mon, stop):
    print(__doc__.split("Keys", 1)[1].split("Needs:", 1)[0].replace("(type the letter, then Enter):", "Keys (type the letter, then Enter):"))
    while not stop.is_set():
        try:
            key = input().strip().lower()
        except EOFError:
            break
        if key == "q":
            stop.set()
        elif key in KEYS:
            mon.send(KEYS[key])
        elif key == "p":
            mon.ping()
        elif key == "s":
            for line in mon.results():
                print(" " + line)
        elif key:
            print(" keys: e r a b c d p s q")


def beacon(args, stop):
    """Broadcast "IS02_BROKER <port>" on UDP 47002 every 2 s. The robot firmware
    (robot_firmware/app_program/app_net.c) switches to this laptop if the broker
    IP it was built with stops answering, so a new laptop IP needs no rebuild."""
    msg = f"IS02_BROKER {args.port}".encode()
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        if args.broker not in ("localhost", "127.0.0.1"):
            s.bind((args.broker, 0))      # send on the hotspot interface
    except OSError as e:
        print(f"(broker beacon off: {e})", flush=True)
        return
    while not stop.is_set():
        try:
            s.sendto(msg, ("255.255.255.255", 47002))
        except OSError:
            pass
        stop.wait(2.0)


def main():
    ap = argparse.ArgumentParser(description="IS-02 telemetry monitor")
    ap.add_argument("--broker", default="localhost", help="MQTT broker IP (the laptop's IP on the hotspot)")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--out", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs"),
                    help="folder for the CSV and log files")
    ap.add_argument("--ping-every", type=float, default=5.0, help="send PING every N seconds (0 = off)")
    ap.add_argument("--duration", type=float, default=0.0, help="stop after N seconds (0 = until q)")
    ap.add_argument("--quiet", action="store_true", help="don't print every telemetry frame")
    ap.add_argument("--no-keyboard", action="store_true", help="no keyboard commands (for scripts)")
    ap.add_argument("--no-beacon", action="store_true",
                    help="don't broadcast the broker beacon (the robot uses it to find a moved broker)")
    args = ap.parse_args()

    mon = Monitor(args)
    try:
        mon.client.connect(args.broker, args.port, keepalive=30)
    except OSError as e:
        sys.exit(f"cannot reach the broker at {args.broker}:{args.port}: {e}\n"
                 "Is mosquitto running and listening on the network? See laptop_tools/README.md")
    mon.client.loop_start()

    stop = threading.Event()
    if not args.no_beacon:
        threading.Thread(target=beacon, args=(args, stop), daemon=True).start()
    if not args.no_keyboard:
        threading.Thread(target=keyboard, args=(mon, stop), daemon=True).start()
    t0 = time.time()
    t_ping = t0
    try:
        while not stop.is_set():
            time.sleep(0.2)
            now = time.time()
            if args.ping_every > 0 and now - t_ping >= args.ping_every and mon.connected and mon.robot_online():
                t_ping = now
                mon.ping()
            if args.duration > 0 and now - t0 >= args.duration:
                break
    except KeyboardInterrupt:
        pass
    mon.client.loop_stop()
    mon.client.disconnect()
    mon.summary()


if __name__ == "__main__":
    main()
