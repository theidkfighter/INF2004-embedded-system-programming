"""IS-02: send the barcode character -> movement mapping to the robot.

Just before the demo the instructor names four characters for the four
movements. Send them like this (any order, letters A-Z; FWD or FORWARD):

    py robot_map.py --broker localhost K=LEFT Q=RIGHT X=FWD M=UTURN
    py robot_map.py --broker localhost --show      # what the robot is using
    py robot_map.py --broker localhost --clear     # remove it from the broker

The mapping is published retained on is02/map, so the broker re-sends it
every time the robot (re)connects. If the robot is online it answers on
is02/ack (cmd "MAP:...") with the map it now uses and "n" (number of
letters), or "nack: <reason>". The dashboard's "Barcode mapping" panel and
`is02_tool.py map ...` do the same thing.

The map goes out as a Map_Frame by default. --legacy-json sends baseline text
instead, only for a Baseline_Build robot.
"""
import argparse
import re
import sys
import time

try:
    import paho.mqtt.client as mqtt
except ImportError as e:
    sys.exit(f"paho-mqtt is not installed for THIS Python ({sys.executable}).\n"
             f"Install it into the same Python with:\n"
             f"    \"{sys.executable}\" -m pip install paho-mqtt\n"
             f"(import error: {e})")

import is02_codec as codec

# the robot's words (nav_map.c); everything is sent as FWD / LEFT / RIGHT / UTURN
MOVES = {"FWD": "FWD", "FORWARD": "FWD", "STRAIGHT": "FWD", "F": "FWD",
         "LEFT": "LEFT", "L": "LEFT", "RIGHT": "RIGHT", "R": "RIGHT",
         "UTURN": "UTURN", "U-TURN": "UTURN", "U_TURN": "UTURN", "U": "UTURN", "BACK": "UTURN"}


def make_client():
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="is02-map")
    except AttributeError:                      # paho-mqtt 1.x
        return mqtt.Client(client_id="is02-map")


def parse(pairs):
    """['K=LEFT', ...] -> 'X=FWD,K=LEFT,Q=RIGHT,M=UTURN' (4 different letters, all 4 movements)."""
    got = {}
    for p in re.split(r"[,\s;]+", " ".join(pairs).strip()):
        if not p:
            continue
        m = re.fullmatch(r"([^=:]+)[=:]([^=:]+)", p)
        if not m:
            sys.exit(f"not a LETTER=MOVE pair: {p}")
        a, b = m.group(1).upper(), m.group(2).upper()
        if re.fullmatch(r"[A-Z]", a) and b in MOVES:
            ch, mv = a, MOVES[b]
        elif re.fullmatch(r"[A-Z]", b) and a in MOVES:
            ch, mv = b, MOVES[a]
        else:
            sys.exit(f"not a LETTER=MOVE pair: {p}  (letters A-Z; moves FWD, LEFT, RIGHT, UTURN)")
        if mv in got:
            sys.exit(f"{mv} given twice")
        if ch in got.values():
            sys.exit(f"character {ch} used twice")
        got[mv] = ch
    missing = [m for m in ("FWD", "LEFT", "RIGHT", "UTURN") if m not in got]
    if missing:
        sys.exit("missing: " + ", ".join(missing))
    return f"{got['FWD']}=FWD,{got['LEFT']}=LEFT,{got['RIGHT']}=RIGHT,{got['UTURN']}=UTURN"


def main():
    ap = argparse.ArgumentParser(description="send the IS-02 barcode mapping (retained on is02/map)")
    ap.add_argument("pairs", nargs="*", help="LETTER=MOVE, e.g. K=LEFT Q=RIGHT X=FWD M=UTURN")
    ap.add_argument("--broker", default="localhost", help="MQTT broker IP (default: localhost)")
    ap.add_argument("--port", type=int, default=1883)
    ap.add_argument("--show", action="store_true", help="ask the robot which mapping it is using")
    ap.add_argument("--clear", action="store_true", help="delete the retained map and empty the robot's map")
    ap.add_argument("--wait", type=float, default=8.0, help="seconds to wait for the robot's ack")
    ap.add_argument("--legacy-json", dest="legacy_json", action="store_true", default=False,
                    help="send baseline text (only for a Baseline_Build robot)")
    ap.add_argument("--no-legacy-json", dest="legacy_json", action="store_false",
                    help="send binary frames (the default)")
    args = ap.parse_args()

    if not (args.pairs or args.show or args.clear):
        ap.error("give the mapping (e.g. K=LEFT Q=RIGHT X=FWD M=UTURN), --show or --clear")
    text = parse(args.pairs) if args.pairs else None

    def command(cmd):
        return cmd if args.legacy_json else codec.encode_command_text(cmd)

    acks = []
    c = make_client()
    c.on_message = lambda cl, ud, msg: acks.append(codec.decode(msg.topic, msg.payload, legacy_json=True, log=print))
    c.connect(args.broker, args.port, keepalive=30)
    c.subscribe("is02/ack")
    c.loop_start()
    time.sleep(0.3)

    if args.clear:
        c.publish("is02/map", "", qos=1, retain=True).wait_for_publish()
        print("retained map removed from the broker")
        c.publish("is02/cmd", command("MAP:CLEAR"), qos=1).wait_for_publish()
        want = ("MAP:CLEAR",)
    elif text is not None:
        payload = text if args.legacy_json else codec.encode_map(text)
        c.publish("is02/map", payload, qos=1, retain=True).wait_for_publish()
        print("map stored on the broker: " + text)
        want = ("MAP:" + text, "MAP")         # the robot acks the retained map as "MAP:<payload>", a binary ack as "MAP"
    else:
        c.publish("is02/cmd", command("GET:MAP"), qos=1).wait_for_publish()
        want = ("GET:MAP",)

    t_end = time.time() + args.wait
    while time.time() < t_end:
        for j in acks:
            if j is not None and j.get("cmd") in want:
                res = str(j.get("result", ""))
                if res.startswith("nack"):
                    print("robot refused it: " + res + "  (it keeps its previous map)")
                else:
                    n = j.get("n", 0)
                    print(f"robot is using: {j.get('map') or '(empty)'}  ({n} letters)"
                          + ("" if n == 4 or args.clear else "  <- expected 4"))
                c.loop_stop()
                return
        time.sleep(0.2)
    print("no answer from the robot (offline?). It will get the mapping when it connects.")
    c.loop_stop()


if __name__ == "__main__":
    main()
