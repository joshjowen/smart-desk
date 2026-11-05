# Changelog

## Stage 2: Up and down control without the sensor (0.2.0) — 2026-11-05

- The real desk is driven through the PC817 optos in source mode (GPIO high = button pressed).
  Kconfig: `DESK_OPTO_ACTIVE_LOW`, `DESK_OPTO_UP_GPIO` (3), `DESK_OPTO_DOWN_GPIO` (7).
  `DESK_SIMULATOR` (now off by default) keeps the simulated desk for desk-free development.
- Invariant 1: `opto_force_off()` is the first line of `app_main`: output latch set to off before the
  driver is enabled, internal pull-down on. Polarity and pins logged at boot.
- Invariant 9: a `--wrap=esp_panic_handler` hook drops both outputs inside the panic handler (crash,
  abort, task and interrupt watchdogs) before the reset; a shutdown handler covers `esp_restart`.
- Hold-to-run jog only; height shows `--`, presets disabled ("needs calibration") on screen, web and
  API. Max move time fixed at 25 s.
- Optional `DESK_HANDSET_SENSE`: debounced handset inputs, ignored while our own output drives that
  line; a press stops automatic moves and counts against the duty budget.
- Tested on the desk: moves while held from touchscreen and web, stops on release; original handset
  works; no movement on reset, crash or reflash (without the external pull-downs); duty budget stops
  a running jog and shows the cooldown countdown.

## Stage 1: Web interface for configuration (0.1.0) — 2026-11-05

- `config`: typed, versioned settings in NVS (Wi-Fi, hostname, time server, presets, display, MQTT,
  remote-move switch, admin password as PBKDF2-SHA256). Fields are only appended, so older settings
  migrate without loss (v1 → v2 tested).
- `net`: Wi-Fi station; setup AP `Desk-Setup-XXXX` (open) when nothing is saved or the connection fails
  for 60 s, with a captive DNS so phones pop up the setup page; AP stops once Wi-Fi is back and nobody
  is on it. mDNS `<hostname>.local`. SNTP from a DHCP-offered server, the configured server or the
  gateway, then pool.ntp.org (works on internet-less VLANs if the router serves NTP).
- `web`: single-page app (7.7 KB gzipped) with Control (live height/state, hold-to-run with 100 ms
  heartbeats, presets, STOP), Settings (Wi-Fi with scan list, presets, display, MQTT, remote moves,
  admin password) and System (info, reboot, factory reset, OTA upload). REST API + WebSocket.
- Remote-move rule: web moves only while the display is unlocked unless "Allow remote moves" is on;
  STOP always accepted. A web jog stops when its WebSocket closes, or 300 ms after the last heartbeat.
- OTA: two slots with bootloader rollback; images are checked for the ESP magic and project name;
  a new image is marked valid after 30 s. `CONFIG_DESK_DEBUG_BOOT_CRASH` builds a test-bad image.
- Display: status row shows the IP, or the setup AP name and 192.168.4.1. Settings apply live.
- Console: `time` command (clock and NTP server status).
- Tested on hardware: phone setup via the AP, invalid settings rejected, settings survive reboot,
  web jog stop on tab close (~10 ms), frozen tab (300 ms) and dropped connection, OTA over Wi-Fi
  (1.5 MB in ~6 s), bad image rolls back, junk upload rejected.

## Stage 0: Prototype GUI (0.0.1) — 2026-11-05

- ESP-IDF v5.5.6 project for the Waveshare ESP32-C6-Touch-LCD-1.47 (LVGL 9.6, esp_lvgl_port 2.9).
- `jd9853` esp_lcd panel driver and `axs5106` touch driver, ported from Waveshare's ESP-IDF demo.
  The LCD SPI bus is write-only, so GPIO3 (TF MISO / opto UP) is never claimed.
- `board`: shared I2C bus with a boot-time scan, display, touch, PWM backlight with fade, and TF CS (GPIO4) held high.
- `motion`: the full request/state API (`JOG_UP/DOWN`, `HEARTBEAT`, `STOP`, `GOTO_HEIGHT`, `GOTO_PRESET`,
  `CALIBRATE`, `CLEAR_FAULT`) backed by a simulated desk (25 mm/s, 650–1150 mm). Already enforced:
  heartbeat timeout (300 ms), direction-change off gap (300 ms), max move time (25 s → FAULT),
  duty-cycle ledger in RTC memory (120 s per 1200 s; seeks refused under 10 s; COOLDOWN at 0),
  STOP always wins, task watchdog. The opto GPIOs are not configured at all in this stage.
- `ui`: lock screen with slide-to-unlock (≥85 % travel), control screen with hold-to-run UP/DOWN,
  SIT/STAND presets, SEEK info, COOLDOWN countdown and FAULT overlays (with Acknowledge), auto-lock
  (30 s), dimming (60 s locked), wake-only first touch, any-touch-stops-an-automatic-move.
- Heap and frame rate logged every 10 s.
- Serial console (`desk>` prompt) with status, stop, goto, scan, and debug fault/duty/crash commands.
- Two-slot OTA partition table set up from the start (ota_0/ota_1 3 MB each, 1.9 MB storage).
- Landscape UI (320 x 172).
- Touch is polled (no INT pin passed to esp_lvgl_port, whose interrupt mode left touch dead).
- Open item confirmed: onboard I2C devices are 0x63 (AXS5106L) and 0x6b (QMI8658); 0x29 is free.
