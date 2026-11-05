# CLAUDE.md

ESP-IDF firmware for a Waveshare ESP32-C6-Touch-LCD-1.47 that drives an Otto Chatham sit-stand desk
by simulating handset UP/DOWN presses through PC817 optocouplers. The authoritative plan is
[desk-controller-spec.md](desk-controller-spec.md) — read it before changing behaviour.

## Working rules (from the spec, plus what Josh has asked for)

- Work strictly one stage at a time (0 → 6). Each stage ends with a flashable build, its acceptance
  checklist, a `CHANGELOG.md` entry and an updated `README.md`.
- **Stop at the end of every stage and wait for Josh to test on hardware** before starting the next.
- ESP-IDF only (no Arduino / ESPHome). C, LVGL 9, esp_lvgl_port.
- Never weaken the safety invariants in the spec's architecture section. If a feature seems to need
  that, stop and ask.
- Unknown hardware facts go behind Kconfig or a single constant in `main/board_config.h`, never a guess
  baked into code.
- Git: commit at the end of each stage (and for fixes once Josh confirms them). Commit message trailer:
  `Co-Authored-By: Claude Opus 5.5 <noreply@anthropic.com>`. Author must be
  `Josh Owen <joshjowen@hotmail.com>` (set in the repo's local git config). Remote:
  `github.com/joshjowen/smart-desk` (public, GPL-3.0), branch `main`. Never commit private data
  (Wi-Fi names/passwords, IPs of Josh's network, `sdkconfig`).
- Web-side tests can be run from the PC with curl / python `websockets` (IDF env) against the desk.

## Current status

- **Stage 0 (prototype GUI): done and accepted on hardware.** Landscape UI, touch working.
- **Stage 1 (Wi-Fi, settings, web UI): done; all acceptance checks passed on hardware** (see CHANGELOG).
  Waiting for Josh to say go before stage 2.
- Open question raised with Josh: the setup AP is open (no password). Offered to add one.
- Stages 2–6 not started. The desk is physically wired to the optos; only the VL53L0X is not connected.

## Build, flash, monitor

```sh
. ~/esp/esp-idf-v5.5.6/export.sh          # every new shell
idf.py build
idf.py -p /dev/ttyACM0 flash               # port moves between ACM0/ACM1 (a USB speakerphone is also ACM): check with udevadm
idf.py -p /dev/ttyACM0 monitor             # interactive only (needs a TTY)
```

From Claude Code (no TTY), capture logs and drive the console with:

```sh
python tools/serlog.py /dev/ttyACM0 15 --reset "status" "goto 900"
```

Fresh machine setup (ESP-IDF v5.5.6 — `install.sh` did not fetch cmake/ninja, so install them too):

```sh
mkdir -p ~/esp && cd ~/esp
git clone -b v5.5.6 --recursive --depth 1 --shallow-submodules https://github.com/espressif/esp-idf.git esp-idf-v5.5.6
cd esp-idf-v5.5.6 && ./install.sh esp32c6 && python3 tools/idf_tools.py install cmake ninja
```

The user must be able to open the serial port (`dialout` group or a udev ACL). The board enumerates as
Espressif "USB JTAG/serial debug unit" (VID 303a, PID 1001).

`sdkconfig` is generated from `sdkconfig.defaults` and is git-ignored. After changing
`sdkconfig.defaults`, delete `sdkconfig` and rebuild. Changing the bootloader options (e.g. rollback)
needs a USB flash, not OTA.

## Layout

```
main/                 app_main (init order), board_config.h (ALL pin numbers), console.c, Kconfig.projbuild
components/board      buses, JD9853 display, AXS5106L touch, backlight, TF CS
components/jd9853     esp_lcd panel driver (init sequence from Waveshare demo)
components/axs5106    esp_lcd_touch driver
components/motion     motion state machine + safety invariants + duty ledger (duty.c); simulator in stages 0-1
components/config     typed settings blob in NVS (versioned), admin password (PBKDF2), listeners
components/net        Wi-Fi STA + setup AP fallback, captive DNS (dns_captive.c), mDNS, SNTP
components/web        httpd: page + WebSocket (web.c), REST API + OTA (web_api.c)
components/ui         LVGL screens (single ui.c)
web/index.html        the whole web app (inline CSS/JS); gzipped and embedded at build time
tools/serlog.py       serial capture helper
```

## Architecture notes

- `motion` is the only code allowed to touch the opto GPIOs. Everything else sends `motion_request()`
  (queue) and reads `motion_get_status()` (spinlock copy). STOP always wins within a queue batch.
- Request sources: TOUCH, WEB, MQTT, HANDSET, SYSTEM (console). A heartbeat only extends a jog from the
  same source.
- Duty ledger lives in `RTC_NOINIT` memory with a checksum: survives resets/panics, not power cycles.
  Debug console: `duty <s>`, `duty clear`, `fault [text]`, `crash`.
- Config changes go through `config_update()` (validates, saves, then calls listeners in `main.c`, which
  push presets to motion, display settings to the UI, Wi-Fi/hostname to net).
- Web moves are allowed only if `allow_remote_moves` is on or the display is unlocked (`ui_is_locked()`).
  STOP is always accepted. A web jog also stops when its WebSocket closes.
- OTA: two slots, rollback enabled; `main.c` marks the image valid after `CONFIG_DESK_OTA_VALID_AFTER_S`
  (30 s). Uploads are checked for the ESP image magic and matching project name.

## Hardware gotchas learned so far

- **GPIO3 is opto UP and also TF-card MISO.** The LCD SPI bus is initialised with `miso_io_num = -1`;
  never init the TF card. GPIO4 (TF CS) is driven high first thing in `app_main`.
- **Opto pins are deliberately untouched while `CONFIG_DESK_SIMULATOR=y`.** Their safe "off" level
  depends on source vs sink wiring (`DESK_OPTO_ACTIVE_LOW`, stage 2), which is not confirmed yet.
- **Touch must be polled.** Passing the INT pin to esp_lvgl_port switches it to interrupt-driven reads
  and touch goes dead. `board_touch_init()` passes `GPIO_NUM_NC`.
- AXS5106L reads use separate write then read (STOP between), as in Waveshare's demo.
- Display is landscape 320x172: swap_xy + mirror_x on the panel (via lvgl_port rotation), gap (0, 34).
  Touch: swap_xy only, with x_max/y_max given in native portrait (172/320).
- Onboard I2C devices: 0x63 (touch), 0x6b (QMI8658 IMU, unused). 0x29 (VL53L0X) is free. One boot showed
  intermittent `i2c.master: probe device timeout` errors during the boot scan; not reproduced since.
- ESP-IDF 5.5 httpd does NOT call the WebSocket URI handler for the handshake; use
  `ws_post_handshake_cb` (`CONFIG_HTTPD_WS_POST_HANDSHAKE_CB_SUPPORT`).
- With `server_from_dhcp`, SNTP must be started explicitly (`esp_netif_sntp_start`).
- Config struct: only append fields and bump `CONFIG_VERSION`; older blobs load as a prefix.
- Library versions: LVGL 9.6, esp_lvgl_port 2.9, esp_lcd_touch 1.2 (older pins didn't compile together).
  LVGL 9.6 deprecates `lv_obj_add_flag`/`remove_flag` — warnings only, cleanup pending.
