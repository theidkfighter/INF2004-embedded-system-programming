# IS-02 Robo-Wheeler: setup guide

How to get this repo building and running on **your own device**. Follow it top to bottom once. Afterwards you only need section 6 (every session).

| Part | Where | Runs on |
|---|---|---|
| Robot firmware (µT-Kernel 3.0, GNU make) | `app_program/`, `build_make/`, `config/` | the Pico W on the car |
| Laptop tools (broker config, dashboard, scripts) | `MQTT/` | your Windows laptop |

More detail: `MQTT/README.md` (laptop tools).

---

## 1. What you need

- Windows laptop (the steps use **WSL Ubuntu** for building)
- Pico W on the car, USB cable (data cable, not charge-only), power bank for floor runs
- A phone hotspot on **2.4 GHz** with **WPA2** (WPA2/WPA3 mixed is fine). The Pico W cannot join 5 GHz or WPA3-only networks.
- The robot and your laptop must join the **same** hotspot.

## 2. Get the code

```sh
git clone https://github.com/theidkfighter/INF2004-embedded-system-programming.git
cd INF2004-embedded-system-programming
git checkout -b feature/<your-name>-<topic>      # work on a branch, not on main
```

## 3. Build tools (once)

Open an **Ubuntu (WSL)** terminal. Your Windows drives are under `/mnt/c/...`, so you can clone the repo on Windows and build from WSL.

```sh
sudo apt install gcc-arm-none-eabi build-essential git
git clone -b 2.2.0 https://github.com/raspberrypi/pico-sdk.git ~/pico/pico-sdk
cd ~/pico/pico-sdk
git submodule update --init lib/tinyusb lib/cyw43-driver lib/lwip
```

**Use Pico SDK 2.2.0 exactly.** SDK 2.1.1 and 2.3.1 do not compile this project. It is a plain `make` project, so VS Code's Pico/CMake build buttons will not work. Always build from the terminal.

## 4. Your local config file

Two files hold your own network details. They are git-ignored, so each person creates their own.

```sh
cd config
cp wifi_credentials.example.h wifi_credentials.h
cp mqtt_config.example.h      mqtt_config.h
```

Edit them:

| File | Set | Notes |
|---|---|---|
| `config/wifi_credentials.h` | `UTK_WIFI_SSID`, `UTK_WIFI_PASSWORD`, and change `UTK_WIFI_CREDENTIALS_SET` from 0 to 1 | Your hotspot name and password |
| `config/mqtt_config.h` | `MQTT_BROKER_IP`, and change `MQTT_CONFIG_SET` from 0 to 1 | The **laptop's** IPv4 address on the hotspot (see 6.1), not the phone's gateway address |

If you forget to flip the `0` to `1`, the build stops with an `#error` on purpose.

> **Never commit** `wifi_credentials.h`, `mqtt_config.h` or any `.uf2` file. A built `.uf2` contains the hotspot password.

## 5. Build and flash

```sh
cd build_make
make PICO_SDK_PATH=~/pico/pico-sdk -j8
```

The result is `build_make/is02_robot_smp0.uf2`. A bare `make` already uses the robot profile (`build_make/robot_defaults.mk`: single core, USB console, Wi-Fi + DHCP). Do not override it.

To flash: hold **BOOTSEL** on the Pico W, plug in USB, release. A drive called `RPI-RP2` appears. Copy the `.uf2` onto it and the board restarts. In WSL, `explorer.exe .` opens the build folder in Windows.

The robot's console is the Pico's USB port (115200 baud, any serial monitor). Boot messages and the `[status]` line appear there.

## 6. Laptop side

### 6.0 One-time install

1. **Python 3 from python.org**, tick *Add python.exe to PATH*. Do not use the MSYS2 Python. Then:
   ```
   py -m pip install paho-mqtt
   py -c "import paho.mqtt, sys; print('OK', sys.executable)"
   ```
   Run all scripts with **`py`**, from a terminal (not the VS Code play button).
2. **Mosquitto** (64-bit Windows build) from <https://mosquitto.org/download>, default folder.
3. In **PowerShell as Administrator**, stop the background Mosquitto service (it blocks port 1883 for our config) and open the firewall:
   ```
   Stop-Service mosquitto
   Set-Service mosquitto -StartupType Manual
   netsh advfirewall firewall add rule name="mosquitto 1883" dir=in action=allow protocol=TCP localport=1883
   netsh advfirewall firewall add rule name="mosquitto 9001" dir=in action=allow protocol=TCP localport=9001
   ```

### 6.1 Every session

**Find your laptop's IP.** Join the hotspot, run `ipconfig`, and read **IPv4 Address** under *Wireless LAN adapter Wi-Fi*. That is `MQTT_BROKER_IP`. It can change every time you reconnect. If it changed, update `config/mqtt_config.h`, rebuild and flash again (or keep the monitor in 6.3 running: the robot can also find the laptop through its broadcast beacon after two failed connects).

**Terminal 1: broker** (from the repo's `MQTT/` folder, keep it open):

```
& "C:\Program Files\mosquitto\mosquitto.exe" -c mosquitto_is02.conf -v
```

It must print `Opening ipv4 listen socket on port 1883.` and `Opening websockets listen socket on port 9001.` If it says *Only one usage of each socket address*, the Windows service from 6.0 is still running.

**Dashboard:** double-click `MQTT/dashboard.html` (Chrome or Edge). The top bar shows *broker: connected*, then *robot: online* once the car is on. Commands, barcode mapping and parameters can all be done here.

**Terminal 2: monitor (optional but recommended):**

```
py telemetry_monitor.py --broker localhost
```

It prints every frame and event and saves CSV/log files in `MQTT/logs/` (git-ignored) when you press `q`.

### 6.2 Calibration values

The robot refuses to start (`START_REFUSED`) until its calibration values are present. They are kept on the broker, so you send them once per broker session:

1. Open `MQTT/robot_params.txt` and remove the `#` in front of each measured value (use the team's agreed values).
2. Run `py robot_params.py --broker localhost`.

No rebuild is needed. The robot applies them every time it connects. Defaults and meanings are listed in `README_ROBOT.md` section 3.

### 6.3 Barcode map (demo day)

Four letters for **forward, left, right, U-turn**. In the dashboard's *Barcode mapping* panel, or:

```
py robot_map.py X=FWD K=LEFT Q=RIGHT M=UTURN
py is02_tool.py cmd GET:MAP          # check what the robot is using
```

## 7. Running the car

1. Power: USB alone is fine on the bench. On the floor use the **power bank only**, never USB and power bank together.
2. Wait 5 to 20 s. LEDs: **blue** = ready and on MQTT, **magenta** = ready but no MQTT, **flashing red** = error (read the `reason:` on the serial console).
3. Put the car on the line with both line sensors over white either side of the black line.
4. **BTN1** (or *Start* on the dashboard) starts the run. **BTN1** again ends it.
5. Emergency stop: **BTN2**, the dashboard E-STOP button or **Esc**. To recover, clear the hazard and press BTN1.

Full LED, buzzer and command tables are in `README_ROBOT.md`.

## 8. Host unit tests (no robot needed)

```sh
cd tests
make          # builds and runs the logic checks
```

Needs a host C compiler (`gcc`, already installed in step 3). Run this before every push; all checks must pass.

## 9. Troubleshooting

| Symptom | Fix |
|---|---|
| Build stops with `#error ... placeholder values` | Section 4: fill in the file and flip `..._SET` to 1 |
| Build fails with SDK errors | Check `git -C ~/pico/pico-sdk describe --tags` says `2.2.0` |
| `No module named 'paho'` | You ran `python` instead of `py`, or the wrong Python. Redo 6.0 |
| Mosquitto: *Only one usage of each socket address* | `Stop-Service mosquitto` (Administrator) |
| Dashboard: *cannot reach ... (retrying)* | Broker not running, or started without `-c mosquitto_is02.conf` |
| LEDs magenta / dashboard shows no robot | Hotspot must be 2.4 GHz WPA2, laptop on the same hotspot, `MQTT_BROKER_IP` matches `ipconfig`, firewall rule from 6.0. `netstat -an \| findstr 1883` should show `0.0.0.0:1883 LISTENING` |
| Yellow blinking LED | The IMU is not answering. Check the I2C wiring (the GY-511 may be an LSM303D at address 0x1E; the firmware detects both chips) |
| `START_REFUSED` | Calibration values missing: section 6.2 |

## 10. Team rules

- Work on a branch and open a pull request. Do not push straight to `main`.
- Never commit `config/wifi_credentials.h`, `config/mqtt_config.h`, `*.uf2`, `MQTT/logs/` or `MQTT/*.db`.
- Run `make` in `tests/` before pushing.
- If you change a tuning value in `app_program/robot_config.h`, say so in the pull request so the others can update `MQTT/robot_params.txt`.
