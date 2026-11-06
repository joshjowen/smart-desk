# Changelog

## Stage 6: MQTT interface (0.6.0) — 2026-11-06

- `mqtt_ha`: esp-mqtt client (broker, login, optional TLS CA or the built-in CA bundle), last will
  `offline`, reconnect back-off 1-60 s and an immediate retry when Wi-Fi comes back.
- Retained state JSON (2 Hz while moving, on change, 60 s keep-alive), Home Assistant discovery for
  Height, Target height, preset buttons, Stop, State, Moving, Fault, Duty budget remaining, Remote moves
  allowed and Last error; republished on `homeassistant/status` = `online` and when presets or the
  calibration change (disabled presets are removed).
- Commands: stop (always), preset, goto. Moves need "Allow remote moves", one per 2 s, no jog, and
  start after an on-screen countdown (settings v4: 0-10 s, default 3) that a tap or STOP cancels. The
  countdown is held in `motion`. Refused commands show as Last error (`motion_note_error`).
- Display: MQTT indicator lit while connected; countdown overlay. Web: countdown and CA fields, MQTT
  status on System.
- Fix: settings migration now sets new fields' defaults explicitly (the v4 countdown landed in the v3
  struct's padding and loaded as 0).
- Tested with Josh's Home Assistant (discovery, HA restart, countdown, tap cancel, Stop, remote moves
  off, reconnect).

## Stage 5: Enable presets (0.5.0) — 2026-11-06

- Seeks: release early by the learned coast + 2 mm, settle (< 2 mm over 500 ms, max 2 s), at most one
  correction pulse (error / speed, ≥ 150 ms), report final height and error (`last_seek`). Refused when
  uncalibrated, cooling down, faulted, sensor invalid, < 10 s budget, or out of range (with the range in
  the message).
- Coast learning: EMA (alpha 0.2) after approaches ≥ 2 s, bounded to ±50 % of the calibrated coast,
  persisted at most hourly (`coast` NVS record), reset by a new calibration, kept by Correct height.
- Invariant 6: stall check for JOG and SEEK (< 5 mm in 1.5 s once the output has been on 2 s) → FAULT;
  a jog held into an end stop just stops ("At the top / bottom"). Plausibility check for seeks
  (> 1.5 × speed) → FAULT. Lost reading during a seek → FAULT.
- Display: presets enabled; long-press a preset to save the current height; a message shows where the
  desk landed. Web: preset buttons enabled; "Here" button per preset row in Settings; landing message.
- Presets updated for the calibrated range: Sit 730 mm (was 720, below the 730 mm usable minimum),
  Stand 1100 mm.
- Kconfig: `DESK_SEEK_MARGIN_MM` (2), `DESK_COAST_LEARN_PCT` (20).
- Stage 3's deferred hot-unplug test done by Josh: offline shown, recovers without a reboot.
- Tested on the desk: 10 Sit/Stand seeks (web) all within ±5 mm without corrections; a 12 mm seek
  corrected to +2 mm. Twice the control box locked out after heavy use (needed a power cycle); the
  stall check stopped and faulted within 1.5 s both times. The longer accuracy and back-to-back
  duty runs were stopped there at Josh's call.

## Stage 4: Sensor calibration routine (0.4.0) — 2026-11-06

- Calibration routine in `motion` (`calib.c`): budget check (waits and opens by itself when there is
  enough), prompt, probe for the sign, top stop, 2 coast tests down, bottom stop, 2 coast tests up,
  back to the bottom (repeatability check), tape-measure entry, summary, Save / Discard. Speeds by
  least squares over the steady part of each run. Runs under all the invariants; long runs are cut
  into ≤ 20 s segments. Order changed from the spec to fit the duty budget (see README).
- Height model `h_min + sign × (reading − raw_min)`; calibrated range and max move time
  (travel / slowest speed × 1.3) replace the uncalibrated defaults. Stored in NVS with a version and
  timestamp. Presets stay off until stage 5.
- Display: gear button → Settings (calibration status, Calibrate desk, Sensor details); calibration
  screen with progress, keypad for the height, summary, result messages.
- Web: calibration panel on the Control tab, Calibration card on System (start, export / import JSON).
  `POST /api/cal`, `GET` / `PUT /api/calibration`; status JSON gains `calibrated`, `min_mm`, `max_mm`, `cal`.
- Settings: presets are range-checked only when changed, so a new calibration doesn't block saving
  other settings; the web form outlines out-of-range presets.
- Console: `cal` command.
- Addition to the spec: **Correct height** (display, web, `cal correct <mm>`): a second tape
  measurement sets a sensor scale factor (or the offset near the bottom). Calibration record v2 adds
  `scale` in what was padding, so v1 records load unchanged (scale 1). The VL53L0X here reads ~7 %
  long; with scale 0.931 the display matches the tape within 1 mm at bottom, middle and top.
- The duty ledger restarts its 20 min expiry after a reflash/reset (RTC time restarts and the
  ledger keeps the recorded on-time, conservatively). Noted, unchanged.
- Tested on the desk: full routine in 78 s; cancel by web STOP (step 2) and web Cancel (coast test):
  output off within 1 ms, calibration unchanged; export / import round trip; bad imports rejected;
  start refused while the display is locked. Josh: display flow, touch-to-cancel, Correct height and
  a full run from the screen (end stops repeat within 1 mm of the first run).

## Stage 3: Sensor test with raw readings (0.3.0) — 2026-11-06

- `vl53l0x`: register-level driver ported from Pololu's library (ST init sequence: SPAD setup, tuning
  settings, VHV and phase reference calibration), back-to-back continuous ranging, non-blocking
  sample reads with range status, signal and ambient rate.
- `height`: sensor task (10 ms poll). Bad range status dropped; median of 5, then EMA (alpha 0.3);
  invalid after 500 ms without a good sample. 60 s sample history. Sensor errors mark it offline at
  once and re-init with back-off (0.5–5 s), so hot-unplug/replug recovers without a reboot.
  Readings are not used for control yet.
- Settings v3: ranging profile (default 50 ms, high accuracy 200 ms) and filter alpha, both
  applied live. Kconfig: `DESK_HEIGHT_BUDGET_MS`, `DESK_HEIGHT_BUDGET_ACCURATE_MS`, `DESK_HEIGHT_EMA_PCT`.
- Shared I2C bus guarded by a recursive mutex. Fix: a failed touch read made esp_lvgl_port abort, so a
  sensor fault on the bus rebooted the board in a loop. Touch reads now go through the mutex and an
  error reads as "not touched" (which also ends a touch jog). The boot scan gives up after 3 timeouts
  with a wiring hint and resets the bus.
- Display: sensor distance or "sensor offline" under the height; long-press the status row for a
  sensor debug screen.
- Web: Sensor tab (live stats, 30 s plot, 10 s stillness summary, CSV of 60 s, profile/alpha).
  `GET /api/sensor?since=<seq>`, `GET /api/sensor.csv`; status JSON gains `sensor {online, valid, mm}`.
- Console: `sensor` command.
- Tested on the desk: readings track smoothly both ways (~15.5 mm/s, ~480 mm of travel, reading rises
  going up); stillness within ±1 mm (bottom), ±2 mm (middle), ±3 mm (top) filtered, no bad samples.
  Touch and display stay responsive with the sensor running; the debug screen works. The hot-unplug
  test is deferred until the wiring is permanent (the jumper wires are fragile).

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
