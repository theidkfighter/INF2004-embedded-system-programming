# IS-02 Robo-Wheeler

Autonomous line-following robot car for INF2004 (team IS-02): Raspberry Pi Pico W on a Cytron Robo Pico, running **µT-Kernel 3.0** (course template [`sirfonzie/mtk3smp-rp2040`](https://github.com/sirfonzie/mtk3smp-rp2040)). It follows a black line, reads Code 39 barcodes and turns at the next junction as told by an MQTT map, measures humps, bypasses obstacles and sends live telemetry to a laptop dashboard over Wi-Fi.

This README is the one place for setting up, building the flash file (`.uf2`) from this repo, and running the car. Follow sections 1–6 once; after that you only need section 7 each session.

**Contents:** 1 What you need · 2 Placeholders · 3 Install WSL and tools · 4 Get the code · 5 Build the flash file · 6 Laptop setup · 7 Every session · 8 Running the car · 9 Reference · 10 Troubleshooting · 11 Repo layout and rules

---

## 1. What you need

| Item | Notes |
|---|---|
| Windows laptop with **WSL 2 (Ubuntu 24.04)** and **VS Code** | Builds the firmware and runs the MQTT broker and dashboard |
| The car with a **Pico W** | |
| Micro-USB **data** cable | For flashing and the serial console |
| Power bank | For floor runs. **Never connect USB and the power bank at the same time** (lab rule) |
| Phone hotspot on **2.4 GHz, WPA2** (WPA2/WPA3 mixed is fine) | The Pico W cannot join 5 GHz or WPA3-only networks. The robot and the laptop must join the **same** hotspot |

---

## 2. Placeholders used in this README

Replace these with your own values wherever they appear. Never commit the real values.

| Placeholder | Meaning | Example | Where it goes |
|---|---|---|---|
| `<HOTSPOT_SSID>` | Your phone hotspot's name | `MyPhone` | `config/wifi_credentials.h` |
| `<HOTSPOT_PASSWORD>` | Its password | | `config/wifi_credentials.h` |
| `<LAPTOP_IP>` | The laptop's IPv4 on the hotspot (`ipconfig` → *Wireless LAN adapter Wi-Fi* → **IPv4 Address**, not the Default Gateway) | `172.20.10.3` | `config/mqtt_config.h`, dashboard link for teammates |
| `<WINDOWS_USER>` | Your Windows user name | `user` | Paths under `/mnt/c/Users/...` |
| `<REPO_DIR>` | Where you cloned this repo | `~/repos/INF2004-embedded-system-programming` | Terminal commands |
| `<YOUR_BRANCH>` | Your working branch | `feature/obstacle` | Git commands |
| `<LETTER_FWD>`, `<LETTER_LEFT>`, `<LETTER_RIGHT>`, `<LETTER_UTURN>` | The four barcode letters the instructor names on demo day | `X`, `K`, `Q`, `M` | Barcode map (section 7.5) |

---

## 3. Install WSL and the build tools (once per laptop)

This is a **GNU make** project, not a CMake project: the µT-Kernel template brings its own startup code, linker script and makefiles, and uses the Pico SDK only for the USB, Wi-Fi and lwIP drivers. **The VS Code Raspberry Pi Pico extension's Import / Build / CMake buttons cannot build it.** Build from a WSL terminal as below.

### 3.1 WSL with Ubuntu 24.04

In **PowerShell as Administrator**:

```powershell
wsl --install -d Ubuntu-24.04
```

Restart when asked, open **Ubuntu 24.04** from the Start menu and create a Linux user name and password. Check it is version 2 with `wsl -l -v`.

### 3.2 Compiler and tools (Ubuntu terminal)

```sh
sudo apt update
sudo apt install -y gcc-arm-none-eabi build-essential git
arm-none-eabi-gcc --version          # must say 13.2.1
```

### 3.3 Pico SDK 2.2.0 (Ubuntu terminal)

```sh
mkdir -p ~/pico
git clone -b 2.2.0 https://github.com/raspberrypi/pico-sdk.git ~/pico/pico-sdk
cd ~/pico/pico-sdk
git submodule update --init lib/tinyusb lib/cyw43-driver lib/lwip
git describe --tags                  # must say 2.2.0
echo 'export PICO_SDK_PATH=$HOME/pico/pico-sdk' >> ~/.bashrc
source ~/.bashrc
```

Use **2.2.0 exactly**: 2.1.1 and 2.3.1 do not compile this project. Only these three submodules are needed. The `export` line lets `make` and `make clean` find the SDK without typing the path.

### 3.4 VS Code

1. Install the **WSL** extension (Microsoft, `ms-vscode-remote.remote-wsl`).
2. If VS Code offers to import the folder as a Raspberry Pi Pico project or to configure CMake, choose **No**. In the Extensions view, set **Raspberry Pi Pico** and **CMake Tools** to **Disable (Workspace)** for this repo.

---

## 4. Get the code

**Option A: clone inside WSL (recommended, faster builds).** In the Ubuntu terminal:

```sh
mkdir -p ~/repos && cd ~/repos
git clone https://github.com/theidkfighter/INF2004-embedded-system-programming.git
cd INF2004-embedded-system-programming
git checkout -b <YOUR_BRANCH>
code .                               # opens VS Code connected to WSL
```

**Option B: use an existing Windows clone.** Open it in VS Code, then click the blue button at the bottom left → **Reopen Folder in WSL**. In WSL it is at `"/mnt/c/Users/<WINDOWS_USER>/<path to the clone>"` (keep the quotes if the path has a space). Builds are slower from `/mnt/c` but work.

Either way, VS Code's bottom-left button should say **WSL: Ubuntu-24.04**, and terminals opened with **Ctrl+`** are Linux bash terminals.

---

## 5. Build the flash file (`.uf2`)

### 5.1 Your two local config files (once per clone)

From `<REPO_DIR>` in the WSL terminal:

```sh
cd config
cp wifi_credentials.example.h wifi_credentials.h
cp mqtt_config.example.h      mqtt_config.h
```

Edit the copies in VS Code:

`config/wifi_credentials.h`

```c
#define UTK_WIFI_CREDENTIALS_SET   1                     /* was 0 */
#define UTK_WIFI_SSID       "<HOTSPOT_SSID>"
#define UTK_WIFI_PASSWORD   "<HOTSPOT_PASSWORD>"
```

`config/mqtt_config.h`

```c
#define MQTT_CONFIG_SET        (1)                       /* was (0) */
#define MQTT_BROKER_IP         "<LAPTOP_IP>"
```

Both files are **git-ignored**: each person makes their own, and they are never committed. If you forget to change the `0` to `1`, the build stops with an `#error` on purpose.

### 5.2 Build

```sh
cd <REPO_DIR>/build_make
make -j8
```

A good build ends like this (the numbers change with the code):

```text
arm-none-eabi-size is02_robot_smp0.elf
   text    data     bss     dec     hex filename
 394412    2808  107796  505016   7b4b8 is02_robot_smp0.elf
tools/elf2uf2 is02_robot_smp0.elf is02_robot_smp0.uf2
```

The flash file is **`build_make/is02_robot_smp0.uf2`** (about 776 KB). A bare `make` already uses the robot profile from `build_make/robot_defaults.mk` (single core, USB console, Wi-Fi + DHCP); don't override it.

**The `.uf2` contains your hotspot password.** Share it only directly; never commit it (`*.uf2` is git-ignored).

### 5.3 Flash the Pico W

1. In the WSL terminal in `build_make/`, build and copy the flash file to your Windows Downloads folder:
   ```sh
   make -j8
   cp is02_robot_smp0.uf2 /mnt/c/Users/<WINDOWS_USER>/Downloads/
   ```
2. Hold **BOOTSEL** on the Pico W, plug in the USB cable, release. A drive called `RPI-RP2` appears.
3. Drag `is02_robot_smp0.uf2` from **Downloads** onto `RPI-RP2`. The Pico restarts by itself.
4. Check: the NeoPixels go dim white, then **blue** (ready, MQTT connected) or **magenta** (ready, no MQTT yet: do section 6–7). A serial monitor on the robot's COM port (115200 baud) shows the boot log, e.g. `[init] IMU: LSM303D at 0x1E ...`, and a `[status]` line every 2 s.

### 5.4 Building again

Every time you change the code or a config file (for example a new `<LAPTOP_IP>`), run the same command in `build_make/`:

```sh
make -j8
```

It compiles only what changed. Then copy and flash as in 5.3. If the build acts strangely, or after a `git pull`, rebuild from scratch with `make clean && rm -f *.uf2 && make -j8`.

Calibration and tuning values sent over MQTT (section 7.5) need no rebuild.

---

## 6. Laptop setup (once per laptop)

The broker (Mosquitto), dashboard and scripts run on **Windows**, from the repo's `MQTT/` folder.

### 6.1 Python and paho-mqtt

1. Install **Python 3 from python.org** and tick **Add python.exe to PATH**. Don't use the MSYS2 Python.
2. In a Windows terminal:
   ```
   py -m pip install paho-mqtt
   py -c "import paho.mqtt, sys; print('OK', sys.executable)"
   ```
3. Always run the scripts with **`py`**, from a terminal (not the VS Code ▶ button).

### 6.2 Mosquitto and the firewall

1. Install the 64-bit Windows build from <https://mosquitto.org/download> (default folder `C:\Program Files\mosquitto`).
2. In **PowerShell as Administrator**, stop the background service (it only listens to the laptop itself and blocks port 1883) and open the firewall:
   ```powershell
   Stop-Service mosquitto
   Set-Service mosquitto -StartupType Manual
   netsh advfirewall firewall add rule name="mosquitto 1883" dir=in action=allow protocol=TCP localport=1883
   netsh advfirewall firewall add rule name="mosquitto 9001" dir=in action=allow protocol=TCP localport=9001
   ```
3. When the laptop joins the hotspot, set that network's profile to **Private** (Settings → Network → Wi-Fi → the hotspot).

---

## 7. Every session

### 7.1 Find `<LAPTOP_IP>`

Join the hotspot and run `ipconfig`. If the IPv4 address differs from the one in `config/mqtt_config.h`, either rebuild and flash, or just keep the monitor (7.4) running: the robot follows its broadcast beacon after two failed connects.

### 7.2 Start the broker (Terminal 1, keep it open)

In a Windows terminal in `MQTT/` (PowerShell, note the `&`):

```powershell
& "C:\Program Files\mosquitto\mosquitto.exe" -c mosquitto_is02.conf -v
```

It must print `Opening ipv4 listen socket on port 1883.` and `Opening websockets listen socket on port 9001.`

### 7.3 Open the dashboard

Double-click `MQTT/dashboard.html` (Chrome or Edge). The top bar shows **broker: connected**, then **robot: online** once the car is on. It shows state, speeds, line sensors, barcode, ultrasonic scan, hump, Wi-Fi and the event log (**Save log** downloads it), and has E-STOP (also **Esc**), Start, Stop run, barcode override, ping, the **Barcode mapping** panel and **Parameters**.

A teammate on the same hotspot can watch with `dashboard.html?host=<LAPTOP_IP>`.

### 7.4 Monitor (Terminal 2, recommended)

```
py telemetry_monitor.py --broker localhost
```

It prints every frame and event, broadcasts the broker beacon (UDP 47002), and on `q` prints a pass/fail summary and saves CSV and log files in `MQTT/logs/` (git-ignored).

### 7.5 Calibration and the barcode map

**Calibration** (once per broker, and again whenever a value changes): edit `MQTT/robot_params.txt` (remove the `#` in front of each measured value), then:

```
py robot_params.py --broker localhost
```

The broker keeps the values (retained on `is02/config`) and the robot applies them every time it connects. No rebuild needed. Values still to measure on each car are listed in section 9.4.

**Barcode map** (demo day, just before the run): in the dashboard's **Barcode mapping** panel, or

```
py robot_map.py <LETTER_FWD>=FWD <LETTER_LEFT>=LEFT <LETTER_RIGHT>=RIGHT <LETTER_UTURN>=UTURN
py is02_tool.py cmd GET:MAP          # check: "n":4
```

The map is retained on the broker, so a robot restart keeps it. A letter not in the map is reported and the car carries straight on.

---

## 8. Running the car

1. **Power:** USB alone on the bench (motors run slowly). On the floor, **power bank only**.
2. **Wait for boot** (5–20 s): **blue** = ready and on MQTT; **magenta** = ready, no MQTT; **flashing red** = ERROR (read the `reason:` in the serial `[status]` line).
3. **Check the dashboard:** robot online, state IDLE, line sensors change over black/white, mapping shown under "Robot is using".
4. **Place the car** with both line sensors either side of the black line (both read white).
5. **Start:** **BTN1** (GP20) or **Start** on the dashboard. Rising chirp, LEDs green.
6. **During the run** it follows the line, reads each barcode (short beep) and stores the letter, does the mapped turn at the next cross-junction (cyan), bypasses obstacles (orange) and slows to 60 % on humps.
7. **End:** **BTN1** again or **Stop run**. Three-note tune, LEDs white, a `MISSION_COMPLETE` summary (time, distance, barcodes, highest hump, obstacles). BTN1 once more returns to IDLE.

**Emergency stop:** **BTN2** (GP21), the dashboard **E-STOP** / **Esc**, or `e` in the monitor. The car also stops itself on a collision (IMU), an obstacle closer than 8 cm or a stalled task. **To recover:** remove the hazard and press **BTN1**; the car re-checks itself and returns to IDLE. `RESUME` over MQTT is refused on purpose.

**Line lost:** it searches left and right; if it can't find the line it stops, beeps and returns to IDLE. Put it back and press BTN1.

---

## 9. Reference

### 9.1 Lights and sounds

| NeoPixel | Meaning |
|---|---|
| Dim white | Booting |
| **Blue** | IDLE, MQTT connected |
| **Magenta** | IDLE, no MQTT (check hotspot, broker, firewall) |
| **Yellow blinking** (with blue/magenta) | IDLE, IMU not answering: no hump or collision detection. Check the I2C wiring; it retries every 2 s |
| **Green** | Line following |
| **Cyan** | Barcode turn at a junction |
| **Orange** | Obstacle avoidance |
| **White** | Mission complete |
| **Solid red** | Emergency stop (latched; clear the hazard, press BTN1) |
| **Flashing red** | ERROR (e.g. calibration missing). Read the serial `[status] ... reason:` line |

| Buzzer | Meaning |
|---|---|
| Rising two-tone chirp | Run started |
| Short high beep | Barcode decoded |
| Three rising notes | Run complete |
| Continuous 3 s alarm | Emergency stop |
| Two long low tones | ERROR |
| One medium beep | Warning (line lost, or recovery refused) |

The Pico W's own LED blinks about once a second while the firmware runs.

### 9.2 Commands (topic `is02/cmd`)

| Command | Effect |
|---|---|
| `ESTOP` | Emergency stop (latched) |
| `START` / `STOP` | Same as BTN1: start from IDLE / end the run |
| `BARCODE:<A-Z>` | Bench test: do the mapped movement **now** (only while line following) |
| `MAP:X=FWD,K=LEFT,...` / `MAP:CLEAR` / `GET:MAP` | Set / clear / read the barcode map (the retained `is02/map` topic is preferred) |
| `PING:<n>` | Round-trip check |
| `SET:NAME=VALUE` / `GET:PARAMS` | Change / read a run-time parameter (lost on restart; use `robot_params.txt` to keep it) |

`is02_tool.py` wraps these: `py is02_tool.py monitor | map ... | cmd ... | set NAME=VALUE ... | config robot_params.txt | beacon | log <file>`.

### 9.3 Topics

| Topic | Direction | Content |
|---|---|---|
| `is02/telemetry` | robot → laptop | 2 Hz frame: state, speeds, encoders, distance, line sensors, barcode, hump, obstacle scan, RSSI. Buffered (64 frames, ≈32 s) while offline |
| `is02/status` | robot → laptop | 1 Hz heartbeat |
| `is02/event` | robot → laptop | Barcode, `NAV` / `NAV_ACTION`, map, hump, obstacle and bypass events, E-stop, error, mission summary |
| `is02/ack` | robot → laptop | Reply to every command |
| `is02/config` | laptop → robot | Calibration, retained (`robot_params.py`) |
| `is02/map` | laptop → robot | Barcode map, retained |

### 9.4 Calibration values (per car)

Defaults are in `app_program/robot_config.h`; send measured values with `MQTT/robot_params.txt` (section 7.5). Marked **placeholder** = not measured yet on our car.

| Value | Default | How to measure |
|---|---|---|
| `ENC_A_EDGES_PER_REV` | 1260 (placeholder) | Encoder edges per wheel turn |
| `WHEEL_DIAMETER_MM` | 65 (placeholder) | Ruler |
| `TRACK_WIDTH_MM` | 130 (placeholder) | Ruler, tyre centre to tyre centre. Affects turn angles and the return to the line after a bypass |
| `SPEED_FF_DEADBAND_PCT`, `SPEED_FF_MM_S_PER_PCT` | 6 / 11.0 (estimated) | Speed step test |
| `IR_WHITE_ADC`, `IR_BLACK_ADC` | 200 / 3900 | Barcode sensor ADC over white paper and a wide black bar, at the final sensor height |
| `BARCODE_DET_HYS` | 0.02 | From the sensor noise |
| `BARCODE_SENSOR_Y_MM` | 35 | Barcode sensor distance left of the car's centre line |
| `IR_TO_AXLE_MM` | 60 (placeholder) | Line sensors to wheel axle; sets where the car pivots at a junction |
| `ROBOT_HALF_WIDTH_MM` | 80 (placeholder) | Ruler |
| `OBST_US_AHEAD_MM` | 80 (placeholder) | Ultrasonic sensor ahead of the axle |
| `SERVO_TRIM_DEG` | 20.5 | Added to every pan angle (+ = right) so the sensor looks straight ahead at 0° |
| `SERVO_MIN_US`, `SERVO_MAX_US` | 500 / 2500 (placeholder) | Servo pulse range |
| `IMU_FORWARD_AXIS`, `IMU_FORWARD_SIGN` | 1 (X) / +1 | Tilt the car nose-up and check the pitch sign |

The IMU firmware detects either GY-511 chip: LSM303DLHC (accel 0x19) or **LSM303D** (0x1E, which our board has).

### 9.5 Main tuning parameters (`SET:` or `robot_params.txt`)

| Parameter | Default | Effect |
|---|---|---|
| `LINE_BASE_MM_S` | 80 | Line-following speed |
| `LINE_STEER`, `LINE_STEER_RAMP` | 0.40 / 3.0 | Steering strength / how fast it grows in a curve (weaving: lower `LINE_STEER`; running wide: raise the ramp) |
| `LINE_CURVE_SLOW` | 0.6 | Slows down in tight curves |
| `BARCODE_AT_JUNCTION` | 1 | 1 = act at the next junction (demo rule) |
| `BARCODE_LEVELS` | 6 | Each barcode is read 5 threshold levels + 1 adaptive read, for pale prints |
| `BYPASS_MM_S` | 60 | Speed during obstacle bypass |
| `OBST_MARGIN_MM`, `OBST_DEPTH_MM` | 60 / 150 | Side margin and assumed depth of an obstacle |
| `OBST_LEG_STOP_CM` | 15 | A bypass leg counts as blocked if something is closer than this in the car's path |
| `OBST_MAX_TURN_DPS` | 15 | Ultrasonic is ignored while turning faster than this |
| `TURN_SNAP` | 0 | 1 = finish junction turns by creeping onto the new line |

---

## 10. Troubleshooting

| Symptom | Fix |
|---|---|
| Build: `#error "config/wifi_credentials.h: placeholder values..."` (or `mqtt_config.h`) | Section 5.1: fill in the file and change `_SET` to 1 |
| Build: `The robot needs config/mqtt_config.h` / `WIFI_JOIN=1 needs config/wifi_credentials.h` | The file is missing in this clone: section 5.1 |
| Build or `make clean`: `TinyUSB not found at ../../sdk/pico-sdk/lib/tinyusb` | `PICO_SDK_PATH` not set: section 3.3, then open a new terminal |
| Build: `pico/...h: No such file` or other SDK errors | SDK is not 2.2.0 or a submodule is missing (3.3) |
| `arm-none-eabi-gcc` / `make`: command not found | The terminal is PowerShell, not WSL. VS Code's bottom-left must say WSL |
| VS Code keeps offering to import / configure CMake | Section 3.4, and delete the import leftovers (section 11.3) |
| `No module named 'paho'` | Use `py`, not `python` (6.1) |
| Mosquitto: *Only one usage of each socket address* | The Windows service is running: `Stop-Service mosquitto` (Administrator) |
| Dashboard: *cannot reach ... (retrying)* | Broker not running, or started without `-c mosquitto_is02.conf` |
| Robot joins the hotspot but stays **magenta** | `netstat -an \| findstr 1883` must show `0.0.0.0:1883 LISTENING`; hotspot network set to **Private**; firewall rules (6.2); `<LAPTOP_IP>` still correct; some hotspots isolate their clients (test with another device) |
| `START_REFUSED` or flashing red at power-on | Calibration missing (the serial console lists the names): section 7.5 |
| Barcode read but no turn | No map / letter not in the map / no junction yet. Check "Robot is using" and the `NAV_ACTION` events |
| Car turns before or after the crossing | `IR_TO_AXLE_MM` is off |

---

## 11. Repo layout and rules

### 11.1 Layout

```text
app_program/        robot firmware: tasks (tsk_*.c), logic (*_logic.c), drivers (hal_*.c), robot_config.h
MQTT/               laptop tools: mosquitto_is02.conf, dashboard.html, telemetry_monitor.py,
                    is02_tool.py, robot_params.py + robot_params.txt, robot_map.py
config/             *.example.h templates; your own wifi_credentials.h / mqtt_config.h (git-ignored)
build_make/         makefiles; the .uf2 is built here (robot_defaults.mk, robot.mk = robot profile)
tools/elf2uf2/      source of the .uf2 packer (built automatically by make)
docs/               template documentation (GETTING_STARTED.md, PORT_RP2040.md)
kernel/ include/ lib/ device/ etc/   µT-Kernel 3.0 template: kernel, headers, libraries, drivers, linker scripts
```

Don't delete the other boards' files (`iote_*`, `nucleo_*`, `rx*`, `stm32*` ...) under `build_make/`, `kernel/`, `device/`, `include/`, `lib/` and `etc/linker/`: they belong to the template the brief requires, and its makefiles refer to them.

### 11.2 Rules

- Work on your own branch and open a pull request; don't push straight to `main`.
- **Never commit** `config/wifi_credentials.h`, `config/mqtt_config.h`, any `*.uf2`, `MQTT/logs/` or `MQTT/*.db`.
- `git status --short` should show nothing new after a build (build output is git-ignored).
- If you change a default in `app_program/robot_config.h`, say so in the pull request so others can update `MQTT/robot_params.txt`.

### 11.3 If the Pico extension tried to import the repo

Delete these leftovers (they are not part of the repo): `pico_sdk_import.cmake` and `build/` in the repo root, `.vscode/` if the extension created it, and `build_make/tools/elf2uf2.exe` from a Windows (MSYS2) build. If `git status` lists dozens of `build_make/mtkernel_3/**/subdir.mk` files as modified, their line endings were rewritten: undo with `git restore build_make/mtkernel_3`.
