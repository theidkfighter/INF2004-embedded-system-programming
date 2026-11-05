"""IS-02: send robot_params.txt to the robot as a retained MQTT message.

The broker keeps the message on is02/config and re-sends it every time the
robot connects, so the robot gets its calibration without recompiling.

    python robot_params.py --broker 10.21.203.18            # send robot_params.txt
    python robot_params.py --broker 10.21.203.18 --file other.txt
    python robot_params.py --broker 10.21.203.18 --clear    # remove the retained message

If the robot is online it answers on is02/ack (cmd "CONFIG") with how many
values it applied and the ones it rejected.

Every line is checked against the parameter table first: one bad line prints
its line number and NAME and nothing is sent. The file goes out as a
binary Config_Frame. Nothing is sent
while the robot's heartbeat shows a different parameter table.
"""
import argparse
import os
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

import is02_codec as codec

HEARTBEAT_WAIT = 1.5                    # s; the heartbeat is 1 Hz


def make_client():
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="is02-params")
    except AttributeError:                      # paho-mqtt 1.x
        return mqtt.Client(client_id="is02-params")


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    ap = argparse.ArgumentParser(description="send the IS-02 robot calibration (retained)")
    ap.add_argument("--broker", required=True, help="MQTT broker IP (this laptop's IP on the hotspot)")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--file", default=os.path.join(here, "robot_params.txt"))
    ap.add_argument("--clear", action="store_true", help="delete the retained configuration")
    ap.add_argument("--wait", type=float, default=8.0, help="seconds to wait for the robot's ack")
    args = ap.parse_args()

    if args.clear:
        payload = ""
    else:
        with open(args.file, encoding="utf-8") as f:
            raw_lines = f.read().splitlines()
        try:
            frame = codec.encode_config(raw_lines)
        except codec.CodecError as e:
            sys.exit(f"bad lines in {args.file}, nothing sent:\n{e}")
        lines = [line for line in (raw.split("#", 1)[0].strip() for raw in raw_lines) if line]
        payload = frame
        print(f"{len(lines)} values from {args.file}:")
        for line in lines:
            print("   " + line)

    acks = []
    heartbeat = threading.Event()
    mismatch = []

    def on_message(cl, ud, msg):
        decoded = codec.decode(msg.topic, msg.payload, log=print)
        if decoded is None:
            return
        if msg.topic == "is02/ack":
            acks.append(decoded)
        else:
            mismatch[:] = [codec.ptab_mismatch(decoded)]
            heartbeat.set()

    c = make_client()
    c.on_message = on_message
    c.connect(args.broker, args.port, keepalive=30)
    c.subscribe("is02/ack")
    c.subscribe("is02/status")
    c.loop_start()
    if not args.clear:
        heartbeat.wait(HEARTBEAT_WAIT)          # no heartbeat: offline, it gets the values when it connects
        if mismatch and mismatch[0]:
            c.loop_stop()
            sys.exit(f"not sent: {mismatch[0]}")
    c.publish("is02/config", payload, qos=1, retain=True).wait_for_publish()
    print("retained configuration " + ("cleared" if args.clear else "stored on the broker"))
    if not args.clear:
        t_end = time.time() + args.wait
        while time.time() < t_end:
            for j in acks:
                if j.get("cmd") == "CONFIG":
                    print(f"robot: applied {j.get('applied')}, rejected {j.get('rejected')}"
                          + ("" if j.get("result") == "ok" else f"  -> {j.get('result')}"))
                    c.loop_stop()
                    return
            time.sleep(0.2)
        print("no answer from the robot (offline?). It will apply the values when it connects.")
    c.loop_stop()


if __name__ == "__main__":
    main()
