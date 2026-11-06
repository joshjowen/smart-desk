# Smart desk controller

ESP-IDF firmware for a Waveshare ESP32-C6-Touch-LCD-1.47 that drives an Otto Chatham sit-stand desk by
simulating handset button presses through PC817 optocouplers. The full plan is in
[desk-controller-spec.md](desk-controller-spec.md); progress is in [CHANGELOG.md](CHANGELOG.md).

**Current stage: 3 (real desk, hold-to-run only; height sensor readings shown but not used yet).** The
optos are driven in source mode (GPIO high = button pressed). The VL53L0X readings are raw distances;
presets stay disabled until the sensor is calibrated (stage 4).

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
- **Sensor**: VL53L0X diagnostics: filtered and raw distance, range status, signal and ambient rate,
  samples per second, standard deviation over 2 s, a plot of the last 30 s (raw dots, filtered line,
  bad samples in red), a stillness summary over the last 10 s, CSV download of the last 60 s, and the
  ranging profile (default 50 ms / high accuracy 200 ms) and filter alpha. `GET /api/sensor?since=<n>`
  and `GET /api/sensor.csv` give the same data to scripts.
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
- The line under the height shows the sensor's filtered distance ("sensor 80 mm (uncalibrated)"), or
  "sensor offline".
- **Long-press the status row** (top of the screen) for the sensor debug screen: filtered and raw mm,
  range status, rate, standard deviation and span over 2 s, signal / ambient rate, profile, alpha and
  error counts. Back returns; it also times out (auto-lock time if opened unlocked, 5 min if locked).
- **Cooldown** (duty budget used) shows a countdown; **Fault** shows the reason and needs Acknowledge.

### Serial console

The board's USB port gives a `desk>` prompt (e.g. through `idf.py monitor`):

| Command | Does |
| --- | --- |
| `status` | State, height, target, duty budget, fault, last error, heap |
| `stop` | Stop any movement |
| `goto <mm>` | Seek to a height |
| `sensor` | Height sensor readings, noise, profile and error counts |
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

## Height sensor (stage 3)

The VL53L0X shares the onboard I2C bus with the touch controller, behind a bus mutex. The `height`
task polls it every 10 ms in continuous mode, drops samples with a bad range status, takes the median
of the last 5 good samples and smooths that with an EMA (alpha 0.3 by default). The filtered reading
goes invalid after 500 ms without a good sample. If the sensor stops answering it is marked offline
straight away and re-initialised with back-off (0.5 s up to 5 s), so it can be unplugged and plugged
back in without a reboot. A touch read that fails on the shared bus counts as "not touched" instead of
crashing the UI.

Kconfig (`Desk height sensor`): `DESK_HEIGHT_BUDGET_MS` (50), `DESK_HEIGHT_BUDGET_ACCURATE_MS` (200),
`DESK_HEIGHT_EMA_PCT` (30). The profile and alpha can also be changed live from the Sensor tab (saved in
settings).

If the screen goes black and the board keeps rebooting with `probe device timeout` in the log, the
sensor is holding the bus low: check its VIN/GND (an unpowered breakout pulls SDA/SCL down) and that
SDA/SCL go to GPIO18/19. The boot scan now reports "bus not responding" in that case.

**Stillness noise** (desk still for 10 s, default profile, measured 2026-11-06 with the sensor mounted):

| Position | Filtered mean | Filtered std dev | Filtered min/max | Raw std dev | Raw min/max | Signal |
| --- | --- | --- | --- | --- | --- | --- |
| Bottom | 50.2 mm | 0.44 mm | 49.1 / 51.0 | 1.08 mm | 46 / 54 | 27.3 MCPS |
| Middle | 292.9 mm | 0.76 mm | 291.2 / 294.4 | 1.72 mm | 287 / 297 | 9.2 MCPS |
| Top | 529.7 mm | 1.10 mm | 527.4 / 533.3 | 3.40 mm | 517 / 539 | 2.8 MCPS |

All within ±5 mm filtered (worst ±3 mm at the top, where the return signal is weakest). No bad
samples in any window. The reading rises as the desk goes up: about 480 mm of travel at ~15.5 mm/s
in both directions, so a full end-to-end run takes ~31 s, longer than the 25 s uncalibrated max move
time (stage 4 must allow for that).

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
| ESP32 3V3 / GND | VL53L0X VIN / GND | |
| ESP32 GPIO18 / GPIO19 | VL53L0X SDA / SCL | Shared onboard I2C bus; leave XSHUT / GPIO1 unconnected |

In sink mode the module's input + goes to 5 V and input − to the GPIO, with a pull-up instead of the
pull-down. GPIO3 is also the TF-card MISO line; the TF card is never used and its CS (GPIO4) is held high.

I2C devices found at boot: 0x29 (VL53L0X), 0x63 (AXS5106L touch), 0x6b (QMI8658 IMU, unused).

## Layout

```
main/            app_main, board_config.h, serial console
components/
  board/         buses, display, touch, backlight
  jd9853/        LCD panel driver
  axs5106/       touch driver
  motion/        motion state machine, safety invariants, duty ledger (simulator in stage 0)
  vl53l0x/       VL53L0X driver (C port of Pololu's library)
  height/        sensor task, filter, 60 s history
  ui/            LVGL screens
  config/        settings in NVS
  net/           Wi-Fi, setup AP + captive DNS, mDNS, SNTP
  web/           HTTP server, REST API, WebSocket, OTA
web/index.html   the web app (gzipped into the firmware at build time)
tools/serlog.py  serial log capture for scripts
```
