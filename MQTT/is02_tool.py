"""
is02_tool.py - laptop side of the IS-02 Robo-Wheeler MQTT link.

Needs:  pip install paho-mqtt        (Python 3.8+)
Broker: Mosquitto on this laptop (see mosquitto_is02.conf next to this file).

Commands (run from a terminal; --broker defaults to localhost):

  python is02_tool.py monitor            live view of events, acks, status
  python is02_tool.py monitor --tlm      ... plus every telemetry frame
  python is02_tool.py map F=FWD K=LEFT R=RIGHT U=UTURN
                                          send the barcode map (retained, so
                                          the robot gets it again after a
                                          reboot or Wi-Fi drop), waits for the ack
  python is02_tool.py map --clear        forget the map (clears the retained copy)
  python is02_tool.py cmd START          any is02/cmd command: START, STOP, ESTOP,
  python is02_tool.py cmd GET:MAP        PING:1, GET:PARAMS, SET:LINE_STEER=0.35,
  python is02_tool.py cmd BARCODE:K      BARCODE:<A-Z> (pretend a barcode was read)
  python is02_tool.py set LINE_BASE_MM_S=100 TURN_MM_S=80
                                          change parameters now (not kept after reboot)
  python is02_tool.py config cal.txt     publish a retained calibration file
                                          ("NAME=VALUE" lines), re-applied at every connect
  python is02_tool.py beacon             broadcast "IS02_BROKER" on UDP 47002 so the
                                          robot finds this laptop even if its IP changed
  python is02_tool.py log run1.csv       record every message to a CSV file

(Windows: run it with "py is02_tool.py ...". The "#!/usr/bin/env python3"
first line was removed: with it, the py launcher picks the first python3 on
PATH, which on some laptops is an MSYS2 Python without paho-mqtt.)
"""
import argparse
import csv
import json
import socket
import sys
import threading
import time

try:
    import paho.mqtt.client as mqtt
except ImportError:
    sys.exit("paho-mqtt is missing: run  pip install paho-mqtt")

T_TLM, T_STATUS, T_EVENT, T_ACK = "is02/telemetry", "is02/status", "is02/event", "is02/ack"
T_CMD, T_CONFIG, T_MAP = "is02/cmd", "is02/config", "is02/map"
BEACON_PORT = 47002


def make_client(args, on_message=None, subscribe=()):
    try:
        c = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=f"is02-laptop-{int(time.time())}")
    except AttributeError:                      # paho-mqtt 1.x
        c = mqtt.Client(client_id=f"is02-laptop-{int(time.time())}")
    connected = threading.Event()

    def on_connect(client, userdata, flags, rc, *extra):
        for t in subscribe:
            client.subscribe(t, qos=1)
        connected.set()

    c.on_connect = on_connect
    if on_message:
        c.on_message = on_message
    try:
        c.connect(args.broker, args.port, keepalive=30)
    except OSError as e:
        sys.exit(f"cannot reach the broker at {args.broker}:{args.port} ({e}). Is Mosquitto running?")
    c.loop_start()
    if not connected.wait(5):
        sys.exit("broker did not answer the MQTT connect")
    return c


def stamp():
    return time.strftime("%H:%M:%S")


def show(topic, payload, show_tlm):
    text = payload.decode(errors="replace")
    if topic == T_TLM:
        if not show_tlm:
            return
        try:
            f = json.loads(text)
            line = f.get("line", {})
            print(f"{stamp()} TLM  {f.get('state',''):<18} spd {f.get('spd')} duty {f.get('duty')} "
                  f"line L{line.get('l')} R{line.get('r')} ir {line.get('ir')} "
                  f"bc {f.get('bc')!r} obst {f.get('obst',{}).get('cm')}cm faults {f.get('faults')}")
        except ValueError:
            print(f"{stamp()} TLM  {text}")
    elif topic == T_EVENT:
        print(f"{stamp()} EVT  {text}")
    elif topic == T_ACK:
        print(f"{stamp()} ACK  {text}")
    elif topic == T_STATUS:
        pass                                    # 1 Hz heartbeat: too chatty, see --tlm
    else:
        print(f"{stamp()} {topic}  {text}")


def cmd_monitor(args):
    seen_status = [0.0]

    def on_msg(client, userdata, msg):
        if msg.topic == T_STATUS:
            seen_status[0] = time.time()
        show(msg.topic, msg.payload, args.tlm)

    make_client(args, on_msg, subscribe=("is02/#",))
    print(f"monitoring is02/# on {args.broker}:{args.port}  (Ctrl+C to stop)")
    try:
        while True:
            time.sleep(5)
            if seen_status[0] and time.time() - seen_status[0] > 5:
                print(f"{stamp()} !!   no heartbeat from the robot for {time.time() - seen_status[0]:.0f} s")
    except KeyboardInterrupt:
        pass


def send_and_wait_ack(args, topic, payload, retain=False, want_prefix=None, timeout=4.0):
    """Publish and print the robot's ack(s) for up to `timeout` seconds."""
    got = threading.Event()

    def on_msg(client, userdata, msg):
        text = msg.payload.decode(errors="replace")
        if msg.topic == T_ACK and (want_prefix is None or want_prefix in text):
            print(f"{stamp()} ACK  {text}")
            got.set()
        elif msg.topic == T_EVENT:
            print(f"{stamp()} EVT  {text}")

    c = make_client(args, on_msg, subscribe=(T_ACK, T_EVENT))
    time.sleep(0.3)                             # let the subscriptions settle
    c.publish(topic, payload, qos=1, retain=retain).wait_for_publish(3)
    print(f"{stamp()} SENT {topic}{' (retained)' if retain else ''}: {payload!r}")
    if not got.wait(timeout):
        print("no ack from the robot (is it on the hotspot and connected? check `monitor`)")
    time.sleep(0.5)                             # GET:PARAMS answers in several parts
    c.loop_stop()
    c.disconnect()


def cmd_map(args):
    if args.clear:
        # an empty retained message deletes the retained copy on the broker
        c = make_client(args)
        c.publish(T_MAP, b"", qos=1, retain=True).wait_for_publish(3)
        c.loop_stop()
        send_and_wait_ack(args, T_CMD, "MAP:CLEAR", want_prefix="MAP")
        return
    if not args.pairs:
        sys.exit("give the map, e.g.  map F=FWD K=LEFT R=RIGHT U=UTURN   (or 4 letters: FWD UTURN LEFT RIGHT)")
    payload = ",".join(args.pairs).upper()
    # retained on is02/map: delivered now AND again at every reconnect
    send_and_wait_ack(args, T_MAP, payload, retain=True, want_prefix="MAP")


def cmd_cmd(args):
    text = " ".join(args.text)
    send_and_wait_ack(args, T_CMD, text, want_prefix=text.split(":")[0][:8])


def cmd_set(args):
    for item in args.pairs:
        send_and_wait_ack(args, T_CMD, "SET:" + item, want_prefix="SET:", timeout=3)


def cmd_config(args):
    with open(args.file, encoding="utf-8") as f:
        text = f.read().strip()
    send_and_wait_ack(args, T_CONFIG, text, retain=True, want_prefix="CONFIG")


def cmd_beacon(args):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    print(f"broadcasting IS02_BROKER on UDP {BEACON_PORT} every 2 s (Ctrl+C to stop)")
    try:
        while True:
            s.sendto(f"IS02_BROKER {args.port}".encode(), ("255.255.255.255", BEACON_PORT))
            time.sleep(2)
    except KeyboardInterrupt:
        pass


def cmd_log(args):
    f = open(args.file, "w", newline="", encoding="utf-8")
    w = csv.writer(f)
    w.writerow(["pc_time", "topic", "payload"])

    def on_msg(client, userdata, msg):
        w.writerow([f"{time.time():.3f}", msg.topic, msg.payload.decode(errors="replace")])
        f.flush()
        if msg.topic in (T_EVENT, T_ACK):
            show(msg.topic, msg.payload, False)

    make_client(args, on_msg, subscribe=("is02/#",))
    print(f"logging is02/# to {args.file}  (Ctrl+C to stop)")
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        f.close()


def main():
    p = argparse.ArgumentParser(description="IS-02 laptop MQTT tool")
    p.add_argument("--broker", default="localhost", help="broker address (default localhost)")
    p.add_argument("--port", type=int, default=1883)
    sub = p.add_subparsers(dest="what", required=True)

    s = sub.add_parser("monitor"); s.add_argument("--tlm", action="store_true"); s.set_defaults(fn=cmd_monitor)
    s = sub.add_parser("map"); s.add_argument("pairs", nargs="*"); s.add_argument("--clear", action="store_true")
    s.set_defaults(fn=cmd_map)
    s = sub.add_parser("cmd"); s.add_argument("text", nargs="+"); s.set_defaults(fn=cmd_cmd)
    s = sub.add_parser("set"); s.add_argument("pairs", nargs="+"); s.set_defaults(fn=cmd_set)
    s = sub.add_parser("config"); s.add_argument("file"); s.set_defaults(fn=cmd_config)
    s = sub.add_parser("beacon"); s.set_defaults(fn=cmd_beacon)
    s = sub.add_parser("log"); s.add_argument("file"); s.set_defaults(fn=cmd_log)
    args = p.parse_args()
    args.fn(args)


if __name__ == "__main__":
    main()
