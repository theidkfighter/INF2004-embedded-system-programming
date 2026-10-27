# IS-02 laptop tools: MQTT broker, dashboard, monitor and robot tools

| File | Use |
| --- | --- |
| `mosquitto_is02.conf` | Mosquitto config: port 1883 for the robot, 9001 for the dashboard, retained messages kept in `is02_mosquitto.db` |
| `dashboard.html` | browser dashboard: live data, commands, barcode map, parameters |
| `is02_tool.py` | teammate's command-line tool: `monitor`, `map`, `cmd`, `set`, `config`, `beacon`, `log` |
| `telemetry_monitor.py` | console monitor with pass/fail checks, CSV and log files, broker beacon |
| `robot_params.py` + `robot_params.txt` | calibration, retained on `is02/config` |
| `robot_map.py` | barcode map, retained on `is02/map` (same as `is02_tool.py map`, with checks for 4 different letters) |

Run every script with `py <script>.py ...` on Windows (not `python`: on some laptops that is an MSYS2 Python without paho-mqtt).

Used by `part_tests/test_wifi_mqtt` (Buddy 1, steps W3–W5) and `full_tests/ft_telemetry` (T1–T6).

The robot and the laptop must be on the **same 2.4 GHz hotspot** (the Pico W has no 5 GHz).

## 1. One-time setup (Windows)

1. **Python 3** from python.org, then in a terminal: (use the same `python` command you will run the scripts with; a plain `pip` can belong to a different Python)
   ```
   python -m pip install paho-mqtt 
   py -c "import paho.mqtt, sys; print('OK', sys.executable)"
   ```
2. **Mosquitto** (the MQTT broker) from https://mosquitto.org/download (64-bit Windows installer). Default folder: `C:\Program Files\mosquitto`.
3. **Stop Mosquitto Windows Service** The installer starts a background service that only listens to the laptop itself, and it holds port 1883, so the robot can’t reach it. Open PowerShell as Administrator and run:
   ```
   Stop-Service mosquitto
   Set-Service mosquitto -StartupType Manual
   ```
4. **Firewall:** the first time Mosquitto starts, Windows asks whether to allow it: tick **Private networks** and allow. If you missed it, run in an **administrator** terminal:
   ```
   netsh advfirewall firewall add rule name="mosquitto 1883" dir=in action=allow protocol=TCP localport=1883
   ```

## 2. Every session

1. Connect the laptop to the hotspot. Find its IP: `ipconfig` → "Wireless LAN adapter Wi-Fi" → **IPv4 Address** (e.g. `172.20.10.3`). This is the broker IP; see "Which IP is the broker IP?" below.
2. Put that IP, the hotspot name and password into `include/wifi_config.h` of the test you're building (`MQTT_BROKER_IP`, `WIFI_SSID`, `WIFI_PASSWORD`), then rebuild and flash.
3. Start the broker in this folder in VSC (keep the window open):
   ```
   & "C:\Program Files\mosquitto\mosquitto.exe" -c mosquitto_is02.conf -v
   ```
   Mosquitto 2.x only accepts connections from the laptop itself unless it is started with this config file.
4. Start the monitor in a second terminal in this folder:
   ```
   py telemetry_monitor.py --broker localhost
   ```
   (`--broker localhost` also works on the laptop running Mosquitto.)
5. Flash and start the test on the robot.

### Which IP is the broker IP?

The broker is Mosquitto, and it runs on the **laptop**, so `MQTT_BROKER_IP` is the **laptop's** IP address on the hotspot. It is **not** the hotspot's (phone's) address: the hotspot is only the network that the robot and the laptop both join.

To find it on Windows, connect the laptop to the hotspot, open a terminal, run `ipconfig`, and under **"Wireless LAN adapter Wi-Fi"** take the **IPv4 Address**. Don't use the "Default Gateway": that's the phone/hotspot.

```
Wireless LAN adapter Wi-Fi:
   IPv4 Address. . . . . . . . . . . : 172.20.10.3    <- broker IP (laptop)
   Default Gateway . . . . . . . . . : 172.20.10.1    <- the phone hotspot, NOT this
```

| Hotspot | Hotspot (gateway) | Laptop = broker IP |
| --- | --- | --- |
| iPhone | 172.20.10.1 | 172.20.10.x (e.g. .2, .3) |
| Android | often 192.168.43.1 or 10.x.x.1 | same range, different last number |
| The laptop's own Windows "Mobile hotspot" (laptop *is* the hotspot) | 192.168.137.1 | 192.168.137.1 (the laptop itself) |

What goes where in `include/wifi_config.h`:

```c
#define WIFI_SSID       "My iPhone"      // hotspot name
#define WIFI_PASSWORD   "........"       // hotspot password
#define MQTT_BROKER_IP  "172.20.10.3"    // laptop's IPv4 from ipconfig
```

- **The laptop's IP can change** each time it reconnects to the hotspot. Run `ipconfig` at the start of every session; if it changed, update `MQTT_BROKER_IP`, rebuild and flash again (otherwise W3 fails with `[FAIL] W3 MQTT connect`).
- **Quick check before flashing:** start Mosquitto, then run `python telemetry_monitor.py --broker 172.20.10.3` (your IP) on the laptop. If it prints "connected to broker", the IP is right and Mosquitto is listening on the network.
- Don't commit the real hotspot password to the group repo.

## 3. Using the monitor

Each telemetry frame is printed on one line, for example:

```
15:42:36 #12    LINE_FOLLOWING  spd L 149.8 R 150.3  enc 1234/1229  dist 512  LR 0/1  ir 1790  bc A  pitch 1.2  obst 35.2  rssi -48  buf 0
```

(`LR` = left/right line sensors, 1 = that sensor is over black, -1 = polarity not set yet; `ir` = barcode sensor ADC.) Events, acks and warnings are printed as they arrive:

```
15:42:35 SENT   ESTOP
15:42:35 ACK    ESTOP: round trip 38 ms  robot: PWM off 7 us after the command arrived  {...}
15:42:48 !! heartbeat was missing for 9.0 s (robot offline?)
15:42:52 EVENT  {"t":52310,"event":"BARCODE","detail":"A"}
```

| Key + Enter | Sends | Use |
| --- | --- | --- |
| `e` | `ESTOP` | Stops the motors (latched until `r`) |
| `r` | `RESUME` | Releases the E-stop latch |
| `a` / `b` / `c` / `d` | `BARCODE:A` … `D` | Remote barcode override; what it does depends on the mapping on `is02/map` |
| `p` | `PING:<n>` | Round-trip time check (also sent automatically every 5 s while the robot is online) |
| `s` | — | Show the checks so far |
| `q` | — | Quit and print the summary |

### Broker beacon

While it runs, the monitor broadcasts `IS02_BROKER 1883` on UDP port 47002 every 2 s; `--no-beacon` turns this off. The robot firmware (`robot_firmware`) uses the beacon only after 2 failed connects to the broker IP it was built with. It then follows the beacon to this laptop, so a new laptop IP does not need a rebuild.

### Robot calibration: `robot_params.py`

The robot firmware reads its calibration and tuning values from the retained MQTT message `is02/config`, so the prebuilt `.uf2` works without recompiling.

1. Edit `robot_params.txt`: one `NAME=VALUE` per line, and remove the `#` once you have a value.
2. Send it:

   ```
   python robot_params.py --broker <laptop IP>
   ```

Mosquitto keeps the message (`persistence true` in `mosquitto_is02.conf`) and re-sends it every time the robot connects. If the robot is online it answers with how many values it applied and which it rejected. `--clear` deletes the retained message.

### Barcode map: dashboard, `is02_tool.py map` or `robot_map.py`

Just before the demo the instructor names the four letters for forward, left, right and U-turn. Send them to the robot over MQTT; nothing is hard-coded in the firmware. Any of these does it:

- dashboard: **Barcode mapping** panel → **Send mapping** ("Robot is using" must show 4 letters)
- `py is02_tool.py map X=FWD K=LEFT Q=RIGHT M=UTURN`
- `py robot_map.py X=FWD K=LEFT Q=RIGHT M=UTURN` (refuses a missing movement or a repeated letter before sending)

```
py robot_map.py --show          # or: py is02_tool.py cmd GET:MAP
py robot_map.py --clear         # or: py is02_tool.py map --clear   (dashboard: Clear map)
```

The map is retained on `is02/map`, so the robot gets it again after a reset. The robot acks with the map it now uses and `"n"` (letters mapped), or `nack: <reason>` (it then keeps the old one). Words: FWD / FORWARD / STRAIGHT, LEFT, RIGHT, UTURN / U-TURN / BACK; letters A-Z only. A letter that is not in the map is reported (`NAV_ACTION ... not in the MQTT map`) and the car carries straight on.

### Browser dashboard: `dashboard.html`

A live dashboard in the browser, next to (or instead of) the monitor. It talks to Mosquitto directly over WebSockets on port 9001 (already in `mosquitto_is02.conf`), so it needs no Python and no internet.

1. Start Mosquitto with the config file as in section 2, step 3. Its window must show `Opening websockets listen socket on port 9001`.
2. Double-click `dashboard.html` (opens in Chrome/Edge). The top bar should show **broker: connected**, then **robot: online** once frames arrive.

It shows the robot state, speed and distance (60 s chart), line sensors, the last barcode and the one stored for the next junction, ultrasonic distance and the last scan, hump pitch/height, Wi-Fi RSSI, frame rate, missing frames, faults and an event/ack log (**Save log** downloads it). Buttons send E-STOP (also the **Esc** key), Start, Stop run, a barcode override (A-Z, acts at once: for bench tests) and Ping; the **Barcode mapping** panel sends the instructor's mapping; **Parameters** reads the robot's values (`GET:PARAMS`) and sends `SET:NAME=VALUE`.

To watch from a teammate's laptop on the same hotspot: open `dashboard.html?host=<broker laptop IP>`, and on the broker laptop allow port 9001 once (administrator terminal):
```
netsh advfirewall firewall add rule name="mosquitto 9001" dir=in action=allow protocol=TCP localport=9001
```

## 4. What the monitor checks

On `q` (or Ctrl+C) it prints a summary like this and saves two files in `logs/`:

```
==================================================
 IS-02 telemetry monitor - summary
 [PASS] L1 telemetry rate: 2.00 Hz over 67 frames (robot clock; target 2 Hz)
 [PASS] L2 message structure: 67 frames, all 13 fields present
 [PASS] L3 no missing frames: 0 missing, 0 duplicated, 0 robot restart(s); largest offline buffer seen 16 frames
 [PASS] L4 heartbeat: 25 received, longest gap 9.0 s, 1 gap(s) over 3 s (expected only while the hotspot was off)
 [PASS] L5 commands: 8 sent, 7 answered, 1 lost during an outage or too late to answer; round trip avg 31 ms, max 44 ms
 [PASS] L6 E-stop: laptop -> robot -> ack 38 ms; robot PWM off 7 us after the command arrived (NFR21 <= 10 ms)
 CSV: logs/telemetry_20261021_154231.csv
 Log: logs/messages_20261021_154231.log
==================================================
```

| Check | Pass when | Brief / DR |
| --- | --- | --- |
| L1 telemetry rate | 1.8–2.2 Hz, measured with the robot's own timestamps | DR: 2 Hz telemetry |
| L2 message structure | Every frame is valid JSON with all 13 fields | Brief: "define telemetry message structures" |
| L3 no missing frames | 0 missing, even across a hotspot outage (the robot buffers ~32 s) | Brief: "connection recovery"; NFR7 / A5 |
| L4 heartbeat | About 1 per second; gaps over 3 s only while the hotspot was off | Brief: "heartbeat/status reporting" |
| L5 commands | Every command sent while the robot was online got an ack | Brief: "command subscription"; FR19 |
| L6 E-stop | Robot reports PWM off ≤ 10 ms after the command arrived | NFR17 + NFR21 |

The CSV has one row per telemetry frame (open it in Excel to plot speed, distance or pitch). The `.log` file has every status, event, command and ack with a timestamp. Keep both as evidence.

## 5. Topics and message format

| Topic | Direction | Content |
| --- | --- | --- |
| `is02/telemetry` | robot → laptop | JSON frame, 2 Hz (buffered while offline) |
| `is02/status` | robot → laptop | Heartbeat, 1 Hz: `{"seq","t","state","rssi","buf","up"}` |
| `is02/event` | robot → laptop | `{"t","event","detail"}`, e.g. `ONLINE`, `BARCODE`, `OBSTACLE_SCAN`, `EMERGENCY_STOP`, `OVERRIDE` |
| `is02/cmd` | laptop → robot | `ESTOP`, `START`, `STOP`, `BARCODE:<A-Z>`, `MAP:...`, `MAP:CLEAR`, `GET:MAP`, `PING:<n>`, `SET:NAME=VALUE`, `GET:PARAMS` |
| `is02/map` | laptop → robot | Barcode map, retained, e.g. `X=FWD,K=LEFT,Q=RIGHT,M=UTURN` |
| `is02/ack` | robot → laptop | `{"t","cmd",...}` reply to each command |

Telemetry frame (every field the brief lists for Buddy 1):

```json
{"seq":7,"t":3500,"state":"LINE_FOLLOWING",
 "spd":[149.8,150.3],"duty":[41.2,40.9],"enc":[1234,1229],"dist":512.3,
 "line":{"l":0,"r":1,"ir":1790},"bc":"A",
 "hump":{"pitch":1.2,"h":0.0,"peak":0.0},
 "obst":{"cm":35.2,"scan":[-1.0,-1.0,-1.0,-1.0,-1.0],"side":"R"},
 "rssi":-48,"buf":0}
```

| Field | Meaning | Brief item |
| --- | --- | --- |
| `state` | Current robot state | Current robot state |
| `spd` / `duty` | Wheel speed mm/s and motor duty %, `[left, right]` | Motor speed |
| `enc` | Encoder counts `[left, right]` | Encoder counts |
| `dist` | Distance travelled, mm | Distance travelled |
| `line` | `l`, `r`: left/right line sensors (1 = black, 0 = white, -1 = not calibrated); `ir`: barcode sensor ADC value | Line sensor status |
| `bc` | Last barcode letter (`""` = none) | Barcode detected |
| `hump` | Pitch °, current and peak hump height mm | Hump measurements |
| `obst` | Ultrasonic cm (`-1` = no echo), last coarse scan −60…+60°, bypass side | Obstacle scan results |
| `rssi` / `buf` | Wi-Fi signal dBm, frames waiting in the offline buffer | Connection recovery |

The robot side is `tlm_link.c` / `tlm_link.h` (same file in both test folders): Wi-Fi, MQTT, the topics above, heartbeat, command subscription, reconnect and the offline buffer. It can be reused as the starting point of `TSK_TELEMETRY`.

## Troubleshooting

| Symptom | Fix |
| --- | --- |
| Monitor: "cannot reach the broker" | Mosquitto isn't running, or wrong IP / port |
| Robot: `[FAIL] W3 MQTT connect` / `T1 MQTT connect`, status 256 | The laptop refused the connection: Mosquitto not running, or started without `-c mosquitto_is02.conf` (then it only listens on 127.0.0.1). `netstat -an` must list `0.0.0.0:1883 LISTENING` |
| Robot: same, status -1 (no answer) | Wrong `MQTT_BROKER_IP` (use the laptop's IPv4 from `ipconfig`), laptop not on the same hotspot, or the firewall rule for port 1883 is missing |
| `[FAIL] W2 Wi-Fi connect` | Wrong SSID/password, or the hotspot is 5 GHz only |
| Dashboard: "cannot reach ... (retrying)" | Mosquitto not running, or started without `-c mosquitto_is02.conf` (no port 9001) |
| Frames arrive but no acks | The robot is subscribed but the monitor sent while it was offline; press `p` again |
