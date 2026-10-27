"""IS-02: send robot_params.txt to the robot as a retained MQTT message.

The broker keeps the message on is02/config and re-sends it every time the
robot connects, so the robot gets its calibration without recompiling.

    python robot_params.py --broker 10.21.203.18            # send robot_params.txt
    python robot_params.py --broker 10.21.203.18 --file other.txt
    python robot_params.py --broker 10.21.203.18 --clear    # remove the retained message

If the robot is online it answers on is02/ack (cmd "CONFIG") with how many
values it applied and the ones it rejected.
"""
import argparse
import json
import os
import sys
import time

try:
    import paho.mqtt.client as mqtt
except ImportError as e:
    sys.exit(f"paho-mqtt is not installed for THIS Python ({sys.executable}).\n"
             f"Install it into the same Python with:\n"
             f"    \"{sys.executable}\" -m pip install paho-mqtt\n"
             f"(import error: {e})")


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
        lines = []
        with open(args.file, encoding="utf-8") as f:
            for raw in f:
                line = raw.split("#", 1)[0].strip()
                if line:
                    if "=" not in line:
                        sys.exit(f"bad line in {args.file}: {raw.strip()}")
                    lines.append(line)
        payload = "\n".join(lines)
        print(f"{len(lines)} values from {args.file}:")
        for line in lines:
            print("   " + line)
        if len(payload) > 1000:
            sys.exit("too long for the robot (1000 characters max): remove some tuning lines")

    acks = []
    c = make_client()
    c.on_message = lambda cl, ud, msg: acks.append(msg.payload.decode(errors="replace"))
    c.connect(args.broker, args.port, keepalive=30)
    c.subscribe("is02/ack")
    c.loop_start()
    c.publish("is02/config", payload, qos=1, retain=True).wait_for_publish()
    print("retained configuration " + ("cleared" if args.clear else "stored on the broker"))
    if not args.clear:
        t_end = time.time() + args.wait
        while time.time() < t_end:
            for a in acks:
                try:
                    j = json.loads(a)
                except ValueError:
                    continue
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
