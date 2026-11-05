# Smart desk controller

ESP-IDF firmware for a Waveshare ESP32-C6-Touch-LCD-1.47 that drives an Otto Chatham sit-stand desk by
simulating handset button presses through PC817 optocouplers. The full plan is in
[desk-controller-spec.md](desk-controller-spec.md); progress is in [CHANGELOG.md](CHANGELOG.md).

**Current stage: 1 (Wi-Fi and web interface, simulated desk).** The opto GPIOs (3 and 7) are not
configured or driven by this firmware yet.

## Build and flash

One-time toolchain install (ESP-IDF v5.5.6):

```sh
mkdir -p ~/esp && cd ~/esp
git clone -b v5.5.6 --recursive --depth 1 --shallow-submodules https://github.com/espressif/esp-idf.git esp-idf-v5.5.6
cd esp-idf-v5.5.6 && ./install.sh esp32c6
python3 tools/idf_tools.py install cmake ninja
```

Build, flash and watch the log:

```sh
. ~/esp/esp-idf-v5.5.6/export.sh
cd ~/smart-desk
idf.py build
idf.py -p /dev/ttyACM1 flash monitor     # Ctrl+] leaves the monitor
```

Options live under `idf.py menuconfig` → **Desk motion**, **Desk UI** and **Desk app** (simulator speed and range,
preset defaults, duty budget, auto-lock/dim timeouts, units, backlight).

## First-boot setup

1. Power the desk controller. With no Wi-Fi saved, it starts an open network called
   `Desk-Setup-XXXX` and shows that name and `192.168.4.1` at the top of its screen.
2. Join that network from a phone. The setup page should open by itself; if not, browse to
   `http://192.168.4.1`.
3. Settings → Wi-Fi → **Scan**, tap your network, enter its password, set a hostname, **Save**.
4. Rejoin your normal network and open `http://<hostname>.local` (default `http://desk.local`).
   The screen shows the desk's IP address.
5. Optionally set an admin password (Settings → Admin password) to protect Settings and System.

If the saved network is unreachable for 60 s, the setup network comes back so you can fix it.
Factory reset (System tab) erases all settings.

## Web interface

- **Control**: live height and state, hold ▲/▼ to move (stops on release, tab close or lost
  connection), preset buttons, STOP.
- **Settings**: Wi-Fi, hostname, time server, presets, display units and timeouts, backlight, MQTT
  (used from stage 6), and **Allow remote moves**. With it off (the default), the web page can only move
  the desk while the desk's screen is unlocked. STOP always works.
- **System**: firmware and network info, last fault, duty budget, reboot, factory reset, firmware
  update (upload `build/smart_desk.bin`; a firmware that fails to start rolls back automatically).

The clock is set by NTP: first a server offered by DHCP, then the **Time server** setting (blank means
the router/gateway), then pool.ntp.org. It is only used for log timestamps.

## Using the touchscreen

- Boots to the **lock screen**: current height, status icons (Wi-Fi/MQTT placeholders, fault warning),
  and a slide-to-unlock knob. Drag it at least 85 % of the way; a shorter slide springs back.
- **Control screen**: hold ▲ / ▼ to move the simulated desk (it stops on release), tap SIT / STAND to
  seek. While seeking, a touch anywhere stops it. "Lock" returns to the lock screen.
- Auto-locks after 30 s without a touch; dims after 60 s locked. The first touch on a dim screen only
  wakes it.
- **Cooldown** (duty budget used) shows a countdown; **Fault** shows the reason and needs Acknowledge.

### Serial console

The board's USB port gives a `desk>` prompt (e.g. through `idf.py monitor`):

| Command | Does |
| --- | --- |
| `status` | State, height, target, duty budget, fault, last error, heap |
| `stop` | Stop any movement |
| `goto <mm>` | Seek to a height |
| `scan` | I2C bus scan |
| `time` | Clock and NTP server status |
| `fault [text]` | Debug: raise a fault (test the fault screen) |
| `duty <s>` / `duty clear` | Debug: use up duty budget / reset the ledger (test the cooldown screen) |
| `crash` | Debug: null-pointer crash (test reset behaviour) |

The duty ledger is kept in RTC memory, so it survives resets and crashes but not a power cycle.

## Wiring

Pin numbers come from [main/board_config.h](main/board_config.h).

| From | To | Notes |
| --- | --- | --- |
| 29 V supply | Desk control box DC input | Y-split on the barrel lead |
| 29 V supply | Buck converter input | Buck rated 36 V input or more |
| Buck 5 V / GND | ESP32 board 5V / GND | |
| Handset cable UP line | PC817 module U1 output | In parallel with the handset button |
| Handset cable DOWN line | PC817 module U2 output | |
| Handset cable GND | PC817 module output G | Desk GND |
| ESP32 GPIO3 (UP) | PC817 module IN1 | Source mode; 10 kΩ pull-down on the GPIO |
| ESP32 GPIO7 (DOWN) | PC817 module IN2 | 10 kΩ pull-down on the GPIO |
| ESP32 GND | PC817 module input G | |
| ESP32 3V3 / GND | VL53L0X VIN / GND | Stage 3 |
| ESP32 GPIO18 / GPIO19 | VL53L0X SDA / SCL | Shared onboard I2C bus (stage 3) |

In sink mode the module's input + goes to 5 V and input − to the GPIO, with a pull-up instead of the
pull-down. GPIO3 is also the TF-card MISO line; the TF card is never used and its CS (GPIO4) is held high.

Onboard I2C devices found at boot: 0x63 (AXS5106L touch), 0x6b (QMI8658 IMU, unused).

## Layout

```
main/            app_main, board_config.h, serial console
components/
  board/         buses, display, touch, backlight
  jd9853/        LCD panel driver
  axs5106/       touch driver
  motion/        motion state machine, safety invariants, duty ledger (simulator in stage 0)
  ui/            LVGL screens
  config/        settings in NVS
  net/           Wi-Fi, setup AP + captive DNS, mDNS, SNTP
  web/           HTTP server, REST API, WebSocket, OTA
web/index.html   the web app (gzipped into the firmware at build time)
tools/serlog.py  serial log capture for scripts
```
