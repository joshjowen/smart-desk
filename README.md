# Smart desk controller

ESP-IDF firmware for a Waveshare ESP32-C6-Touch-LCD-1.47 that drives an Otto Chatham sit-stand desk by
simulating handset button presses through PC817 optocouplers. The full plan is in
[desk-controller-spec.md](desk-controller-spec.md); progress is in [CHANGELOG.md](CHANGELOG.md).

**Current stage: 2 (real desk, hold-to-run only, no height sensor).** The optos are driven in source
mode (GPIO high = button pressed). Presets are disabled until the height sensor is calibrated.

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

## Stage 2 bench test (do this before trusting the desk connection)

1. Flash, then put a multimeter in continuity or diode mode across each opto output (V1–G, V2–G).
   It must conduct only while the matching ▲ / ▼ button is held on the touchscreen.
2. Reset the board, and power it up while holding BOOT. Neither output may conduct at any point
   (watch the module's indicator LEDs too).
3. Hold ▲ on the web page and close the tab: the output must drop within 300 ms.
4. With the desk connected: it moves while ▲ / ▼ is held (touchscreen and web) and stops on release;
   the original handset still works; no movement during power-up, reset, flashing or the `crash`
   console command.
5. Duty budget: `duty 110` on the console, then jog for ~10 s; the desk must stop and show the cooldown
   countdown. `duty clear` resets it.

Console commands and the log show every output change with time, source and reason
(`OUT OFF -> UP src=TOUCH reason=jog`).

Polarity and pins are Kconfig options (`Desk motion`): `DESK_OPTO_ACTIVE_LOW` (sink mode),
`DESK_OPTO_UP_GPIO`, `DESK_OPTO_DOWN_GPIO`. `DESK_SIMULATOR` brings back the simulated desk (outputs
held off). `DESK_HANDSET_SENSE` (off) reads the handset lines to detect physical presses.

The 10 kΩ pull-downs on GPIO3/GPIO7 are not fitted yet. The firmware enables the internal pull-downs
once it is running, but nothing holds the pins low while the chip is in reset.

## Wiring

Pin numbers come from [main/board_config.h](main/board_config.h).

| From | To | Notes |
| --- | --- | --- |
| 29 V supply | Desk control box DC input | Y-split on the barrel lead |
| 29 V supply | Buck converter input | Buck rated 36 V input or more |
| Buck 5 V / GND | ESP32 board 5V / GND | |
| Handset cable UP line | PC817 module V1 | In parallel with the handset button |
| Handset cable DOWN line | PC817 module V2 | |
| Handset cable GND | PC817 module output G | Desk GND |
| ESP32 GPIO3 (UP) | PC817 module I1 | Source mode; 10 kΩ pull-down on the GPIO (recommended, not yet fitted) |
| ESP32 GPIO7 (DOWN) | PC817 module I2 | 10 kΩ pull-down on the GPIO (recommended, not yet fitted) |
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
