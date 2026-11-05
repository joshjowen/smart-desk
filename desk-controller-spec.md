# Desk Controller Firmware Spec

Nov 2, 2026 · @Josh Owen

## Overview

Build ESP-IDF firmware that turns an Otto Chatham single-motor sit-stand desk into a networked, touch-controlled desk with height presets and Home Assistant integration over MQTT. The desk's own control box stays in charge of the motor; the firmware only simulates presses of the handset's UP and DOWN buttons through optocouplers and measures height with a VL53L0X time-of-flight sensor.

**How to use this spec (for Claude Code)**

- Work strictly one stage at a time, in order 0 to 6. Each stage ends with a working, flashable build and its acceptance checklist ticked.
- Stop at the end of every stage and wait for Josh to test on hardware before starting the next.
- Do not use ESPHome or Arduino. Use ESP-IDF (latest stable v5.x) with its component manager.
- Hardware details marked **confirm** in the Open items section are unknown until measured. Put each behind a Kconfig option or a single `board_config.h` constant rather than hard-coding a guess.
- Never weaken the safety invariants in the architecture section to get a feature working. If a stage seems to need that, stop and ask.
- Keep a `CHANGELOG.md` with one entry per stage and a `README.md` covering build, flash, first-boot setup and wiring.

## Hardware

The controller is a Waveshare ESP32-C6-Touch-LCD-1.47 driving two PC817 optocoupler channels wired across the handset buttons, plus a VL53L0X on I2C.

| Part | Detail | Notes for firmware |
| --- | --- | --- |
| MCU board | Waveshare ESP32-C6-Touch-LCD-1.47 (ESP32-C6FH8, single-core RISC-V 160 MHz, 512 KB SRAM, 8 MB flash, no PSRAM) | Memory is tight for LVGL + Wi-Fi + HTTP + MQTT. Use partial LVGL draw buffers in internal RAM and keep web assets small. |
| Display | 1.47" IPS, 172 x 320, JD9853 controller, 4-wire SPI | No stock esp\_lcd driver for JD9853. Port the init sequence from Waveshare's ESP-IDF demo into a small esp\_lcd panel driver. Note the 172 px panel needs a column offset. |
| Touch | AXS5106L capacitive controller on I2C | Port from Waveshare's demo. Expose it as an LVGL pointer input. |
| Onboard I2C devices | Touch controller and QMI8658 IMU share the board I2C bus | The IMU is unused. Leave it idle. |
| Desk control box | Otto/JS single-motor box, 29 V 1 A supply (29 W), duty cycle 2 min on / 18 min off | Firmware enforces the duty cycle (see safety invariants). |
| Handset interface | 4-pin connector, likely UP / GND / NC / DOWN; each button shorts its line to GND | Pin mapping and line pull-up voltage: **confirm**. |
| Button switching | Senzooe 2-channel PC817 module; output transistors across UP and DOWN, shared output GND to desk GND | Drive polarity depends on the 3.3 V bench test: active-high (GPIO sources current) or active-low (GPIO sinks from 5 V). Make it a Kconfig option. **confirm** |
| Optional button sense | UP and DOWN lines read through a divider (or a third opto) | Detects physical handset presses. Optional, behind Kconfig. |
| Height sensor | VL53L0X (I2C, address 0x29) | Mounted at the top of the fixed leg column, aimed at a matte white target on the moving frame, so it measures travel only (roughly 0 to 500 mm). Shares the onboard I2C bus (GPIO18/19); 0x29 does not clash with the documented onboard addresses. |
| Power | 29 V tapped from the control box supply, bucked to 5 V (buck rated 36 V input or more) into the board's 5 V input | No firmware impact. |

**Pin allocation (fixed; put these in `main/board_config.h`).** Opto UP is on GPIO3 and opto DOWN is on GPIO7; the sensor shares the onboard I2C bus.

| Function | GPIO | Notes |
| --- | --- | --- |
| Opto UP drive | GPIO3 | This is also the TF card MISO line. The TF card is not used: never initialise the TF SPI device, leave the slot empty, and drive GPIO4 (TF CS) high right after boot so the card can never be selected. |
| Opto DOWN drive | GPIO7 | Free header pin, not a strapping pin. |
| VL53L0X SDA / SCL | GPIO18 / GPIO19 | Shared onboard I2C bus with the AXS5106L touch and QMI8658A IMU. Guard the bus with a mutex, and log an I2C scan at boot from stage 3 on. |
| LCD | SCK 1, MOSI 2, CS 14, DC 15, RST 22, backlight 23 | JD9853 on SPI. |
| Touch | SDA 18, SCL 19, RST 20, INT 21 | AXS5106L. |
| Do not use | 0, 4, 5, 6, 8, 9, 12, 13, 15, 16, 17 | 0 = battery ADC divider; 4, 5, 8, 9, 15 = strapping; 6 = IMU interrupt; 12/13 = USB; 16/17 = UART0. GPIO4 is only driven high as above. |

Set the console to USB Serial/JTAG (`CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG`). The BOOT button is documented as GPIO8 but reported on hardware as GPIO9; don't use either. Optional handset sense has no pin allocated; GPIO17 is the candidate if it's added later.

### Wiring diagram

| From | To | Notes |
| --- | --- | --- |
| 29 V supply | Desk control box DC input | Y-split on the barrel lead |
| 29 V supply | Buck converter input | Buck rated 36 V input or more |
| Buck 5 V / GND | ESP32 board 5V / GND | |
| Handset cable UP line | PC817 module U1 | Pass-through adapter, in parallel with the handset button |
| Handset cable DOWN line | PC817 module U2 | |
| Handset cable GND | PC817 module output G | Desk GND |
| ESP32 GPIO3 (UP) | PC817 module IN1 | Source mode shown; 10 kOhm pull-down on the GPIO |
| ESP32 GPIO7 (DOWN) | PC817 module IN2 | 10 kOhm pull-down on the GPIO |
| ESP32 GND | PC817 module input G | |
| ESP32 3V3 / GND | VL53L0X VIN / GND | |
| ESP32 GPIO18 / GPIO19 | VL53L0X SDA / SCL | Shared onboard I2C bus |

The handset cable passes through a small adapter so the opto outputs sit in parallel with the original buttons, and the handset keeps working. Shown in source mode; in sink mode the module's input + goes to 5 V and its input − goes to the GPIO. Check the module's input-side silkscreen, fit a 10 kΩ pull-down on each opto GPIO (pull-up in sink mode), and take GPIO numbers from `board_config.h`.

## Software architecture

A single motion controller task owns the two opto outputs; every other part of the system (display, web, MQTT) only sends it requests through a queue.

**Toolchain and libraries**

- ESP-IDF v5.x (latest stable), target `esp32c6`, C (C++ only where a dependency needs it).
- `esp_lcd` + `esp_lvgl_port` + LVGL 9 for the UI.
- `esp_http_server` with WebSocket support for the web interface; static assets gzipped and embedded with `EMBED_FILES` or served from LittleFS.
- `esp-mqtt` (`mqtt_client`) for MQTT.
- NVS for all persistent configuration.
- VL53L0X: no official IDF component. Port a compact register-level driver (a C port of the Pololu library is fine) into `components/vl53l0x`.

**Components**

| Component | Responsibility |
| --- | --- |
| `board` | GPIO, I2C buses, SPI, backlight. Sets opto outputs to the safe (off) state as the very first action in `app_main`. |
| `motion` | Motion state machine, safety invariants, duty-cycle ledger, preset seeking. The only code that touches the opto GPIOs. |
| `height` | VL53L0X driver, filtering, raw-to-height conversion using calibration data. Publishes the latest height and validity. |
| `config` | Typed settings in NVS (Wi-Fi, presets, calibration, MQTT, UI options) with defaults and a version number. |
| `ui` | LVGL screens: lock screen, control screen, status, calibration prompts. |
| `net` | Wi-Fi station, SoftAP fallback, mDNS (`desk.local`), SNTP. |
| `web` | HTTP server, REST config API, WebSocket live control and status. |
| `mqtt_ha` | MQTT client, Home Assistant discovery, command handling. |

**Motion requests.** Every request carries its source (`TOUCH`, `WEB`, `MQTT`, `HANDSET`) and is one of: `JOG_UP`, `JOG_DOWN` (hold-to-run, needs heartbeats), `STOP`, `GOTO_HEIGHT(mm)`, `GOTO_PRESET(id)`, `CALIBRATE`. Newer requests replace older ones; `STOP` always wins.

**Motion states:** `IDLE`, `JOG` (manual, heartbeat-driven), `SEEK` (moving to a target), `SETTLE` (waiting for the desk to coast to a stop and the reading to steady), `CALIBRATING`, `COOLDOWN` (duty budget exhausted), `FAULT`. A fault shows on screen and over web/MQTT, and clears only through an explicit user action.

**Safety invariants (enforced in `motion`, never bypassed)**

1. Opto outputs are forced off before anything else starts at boot, and on any reset, crash or task watchdog. Configure the pins with pulls that hold the off state while the chip is in reset or the pins are high-impedance.
2. UP and DOWN are never active at the same time. Switching direction requires both off for at least 300 ms.
3. Hold-to-run: jog moves continue only while heartbeats arrive (touch held, or a web/MQTT heartbeat every 100 ms). No heartbeat for 300 ms means stop.
4. Maximum single move time (default: full travel time + 30 %, before calibration a fixed 25 s). Exceeding it stops the move and raises `FAULT`.
5. Duty-cycle ledger: rolling 1200 s window, at most 120 s of motor-on time. When fewer than 10 s remain, refuse new seeks; at zero, stop and enter `COOLDOWN` with a countdown shown. Keep the ledger in RTC memory so a soft reset does not clear it.
6. Stall/sensor check (once the sensor is in use): if a move is commanded and the height does not change by at least 5 mm within 1.5 s, stop and raise `FAULT`. Invalid or out-of-range readings during a seek stop the move.
7. Any touch anywhere on the screen, a physical handset press (if sensed) or a `STOP` from any source during an automatic move stops it immediately.
8. Remote-initiated moves (web or MQTT) are disabled by default and gated by the settings described in stages 1 and 6.
9. A task watchdog covers the motion task; if it trips, the outputs drop before the reset.

**Repository layout:** `main/` (app\_main, board\_config.h), `components/{board,motion,height,vl53l0x,config,ui,net,web,mqtt_ha}`, `web/` (HTML/JS/CSS source), `docs/`.

## Stage 0: Prototype GUI

Get the display, touch and LVGL running with the full UI flow, driven by a simulated desk so no hardware beyond the board is needed.

**Scope**

- Bring up the JD9853 panel, backlight (PWM, adjustable) and AXS5106L touch through `esp_lvgl_port`. Portrait 172 x 320.
- `motion` exists as a stub with the real request/state API, backed by a simulator: height moves at 25 mm/s between 650 and 1150 mm while a jog or seek is active. No GPIO is touched in this stage.
- **Lock screen:** large current height (mm or cm, configurable later), small status row (Wi-Fi, MQTT, fault icons as placeholders), and a slide-to-unlock track along the bottom. The slider must travel at least 85 % of the track to unlock and snaps back otherwise.
- **Control screen:** hold-to-run UP and DOWN buttons (move only while pressed; release stops), SIT and STAND preset buttons (simulated seek), current height and target height while seeking, and a STOP behaviour where any touch during a seek stops it.
- Auto-lock after 30 s without touch (configurable later). Dim the backlight after 60 s locked, wake on touch; the first touch on a dimmed screen only wakes it.
- Simple visual states for `SEEK`, `COOLDOWN` (countdown) and `FAULT` (message + acknowledge button) so later stages only wire them up.

**Acceptance**

- [ ] Boots straight to the lock screen with no visual tearing or wrong offset at the screen edges.
- [ ] Slide-to-unlock works and a partial slide does not unlock.
- [ ] Holding UP or DOWN changes the simulated height; releasing stops it within one frame.
- [ ] SIT and STAND animate the simulated height to their targets; a touch mid-move stops it.
- [ ] Auto-lock and dimming work; touch stays responsive (no missed presses at normal tapping speed).
- [ ] Free heap and LVGL frame rate are logged once per 10 s.

## Stage 1: Web interface for configuration

Add Wi-Fi, persistent settings and a small web app for configuration and live control, still against the simulated desk.

**Networking**

- No saved Wi-Fi or connection failing for 60 s: start a SoftAP `Desk-Setup-XXXX` (last 4 hex of MAC) with a captive-portal redirect to the setup page. Show the AP name and `192.168.4.1` on the display.
- Station mode once configured, with mDNS hostname `desk` (configurable) so the UI is at `http://desk.local`. Show the IP on the status screen.
- SNTP for timestamps in logs.

**Web app (single page, plain HTML/JS/CSS, no framework, gzipped, total under about 60 KB)**

- **Control tab:** live height and state over WebSocket; hold-to-run UP/DOWN buttons that send heartbeats every 100 ms while held (mouse and touch events, stop on release, blur or page hide); preset buttons; a large STOP button.
- **Settings tab:** Wi-Fi SSID/password, hostname; preset names and heights (Sit, Stand, plus two custom, each enable/disable); display units (mm/cm), auto-lock and dim timeouts, backlight level; MQTT broker settings (used in stage 6); `Allow remote moves` toggle (default off).
- **System tab:** firmware version, uptime, free heap, Wi-Fi RSSI, last fault, duty-cycle budget remaining, reboot, factory reset (confirm dialog), OTA update by firmware upload (`esp_https_ota`-style flow with a two-slot OTA partition table and rollback on failed boot).
- Optional single admin password (stored as a salted hash in NVS) gating the Settings and System tabs; off by default, prompt to set one on first setup.

**REST and WebSocket**

- `GET /api/status`, `GET/PUT /api/config` (JSON, validated server-side with sane ranges), `POST /api/reboot`, `POST /api/factory-reset`, `POST /api/ota`.
- `WS /ws`: server pushes `{height_mm, state, target_mm, fault, duty_remaining_s}` at 5 Hz while moving and 1 Hz idle; client sends `{cmd: "jog", dir}`, `{cmd: "hb"}`, `{cmd: "stop"}`, `{cmd: "preset", id}`.
- Web moves are motion requests with source `WEB`. When `Allow remote moves` is off, web control works only while the display is unlocked; otherwise the API returns an error and the page explains why.

**Acceptance**

- [ ] Fresh flash comes up as a SoftAP, and Wi-Fi can be set from a phone.
- [ ] Settings survive reboot; invalid values are rejected with a clear message.
- [ ] Closing the browser tab mid-jog stops the simulated desk within 300 ms.
- [ ] Display and web show the same height and state at the same time.
- [ ] OTA upload works and a deliberately bad image rolls back.

## Stage 2: Up and down control without the sensor

Replace the simulator with the real opto outputs so the desk moves from the touchscreen and web, using hold-to-run only.

**Scope**

- `motion` drives the two opto GPIOs. Kconfig: `DESK_OPTO_ACTIVE_LOW` (sink mode) or active-high (source mode), and the two pin numbers.
- Boot sequence: set both pins to their off level and enable the matching hold pull before any other init. Log the configured polarity at boot.
- Only `JOG` is available. Presets are hidden or disabled in the UI and API with "needs calibration".
- Height display shows `--` (no sensor yet). The simulator stays available behind a Kconfig option for desk-free development.
- Enforce invariants 1 to 5, 7, 8 and 9 now (all except the sensor check). The max move time is a fixed 25 s until calibration exists.
- Optional, behind Kconfig `DESK_HANDSET_SENSE`: read the UP/DOWN lines as inputs (via divider) and report physical presses as `HANDSET` events; a press during any automatic move stops it. Debounce 20 ms.
- Log every output transition with timestamp, source and reason, to make bench testing easy.

**Bench test before connecting the desk (put this in the README)**

1. Flash, then with a multimeter in continuity or diode mode across each opto output, confirm it conducts only while the matching button is held.
2. Reset the board and hold BOOT during power-up; confirm neither output conducts at any point.
3. Hold UP on the web page and close the tab; confirm the output drops within 300 ms.

**Acceptance**

- [ ] The desk moves up and down while held from the touchscreen and from the web page, and stops on release.
- [ ] The original handset still works normally.
- [ ] No movement during power-up, reset, flashing or a forced crash (add a debug-only crash command for this test).
- [ ] The duty-cycle ledger counts on-time and blocks movement once the budget is used (test with a temporarily shortened budget).

## Stage 3: Sensor test with raw readings

Bring up the VL53L0X and show its raw and filtered readings everywhere, without using them for control yet.

**Scope**

- `vl53l0x` component: init (including SPAD and reference calibration from the ST sequence), continuous ranging, range status per sample. I2C port and pins from `board_config.h`; if the sensor shares the touch bus, protect the bus with a mutex.
- Ranging profile configurable: default timing budget 50 ms (about 20 Hz); a high-accuracy option at 200 ms for testing.
- `height` filter: discard samples with a bad range status; median of the last 5 valid samples, then an EMA (alpha configurable, default 0.3). Mark the reading invalid after 500 ms without a valid sample.
- Diagnostics page on the web (new tab) and a debug screen on the display (reached by a long press on the status row): raw mm, filtered mm, range status, signal rate, ambient rate, samples per second, and standard deviation over the last 2 s.
- The web diagnostics page plots the last 30 s of raw and filtered values and offers a CSV download of the last 60 s.
- Log a warning when a sensor I2C error occurs and retry init with back-off; a dead sensor must never block the UI or motion task.

**Acceptance**

- [ ] Readings track the desk while jogging, and stay within about ±5 mm (filtered) when the desk is still.
- [ ] Unplugging the sensor shows "sensor offline" within 1 s, and plugging it back in recovers without a reboot.
- [ ] Touch and display stay responsive with the sensor running.
- [ ] Josh records stillness noise at the bottom, middle and top of travel from the diagnostics page and notes it in the README.

## Stage 4: Sensor calibration routine

A guided routine finds the desk's end stops, speeds and coast distances, and maps the sensor reading to real desktop height using one tape-measure entry.

**Height model:** `height_mm = h_min + s * (raw_mm - raw_min)`, where `s` is +1 or -1 (detected from which way the reading moves when going up), `raw_min` is the reading at the bottom stop and `h_min` is the desktop height at the bottom, entered by the user. Store the calibration with a version and timestamp in NVS.

**Routine (started from the display settings or the web System tab; requires the screen unlocked; user can cancel at any step, which stops the desk)**

1. Check the duty-cycle budget. The routine needs about 70 s of motor time; if less remains, show the wait time and do not start.
2. Prompt: "Clear the area under and around the desk, then press Start."
3. Jog DOWN until the reading stops changing (less than 2 mm over 1 s while commanded) or 25 s pass; that is the bottom stop. Release, wait 1 s, average 20 samples: `raw_min`.
4. Jog UP for 2 s and check the reading changed by at least 20 mm; set the sign `s`. If it did not change, abort with "sensor not seeing the target".
5. Continue UP to the top stop in the same way: `raw_max`. Record the time and distance of the steady part of the move: `speed_up` (mm/s).
6. Jog DOWN to the bottom stop again, recording `speed_down`.
7. Coast test: from the bottom, run UP for 3 s, release, and measure how far it travels after release until the reading settles: `coast_up`. Repeat DOWN: `coast_down`. Do each twice and average.
8. Prompt: "Measure the desktop height from the floor now (desk is at its lowest) and enter it in mm." The web form and an on-screen number entry both work. That gives `h_min`.
9. Show a summary (range in mm, speeds, coast distances) and Save or Discard.

**Derived values:** max move time becomes travel / min(speed) x 1.3. Usable height range is `h_min + 5` to `h_max - 5` mm, so seeks never deliberately run into an end stop.

**Acceptance**

- [ ] The routine completes on the real desk in under 2 minutes of wall time and saves.
- [ ] Displayed height matches a tape measure to within ±10 mm at the bottom, middle and top.
- [ ] Cancelling at any step stops the desk immediately and leaves the old calibration intact.
- [ ] Calibration can be exported and imported as JSON from the web System tab.

## Stage 5: Enable presets

With calibration saved, the desk drives itself to Sit, Stand and the custom presets, stopping early by the measured coast distance and correcting once if needed.

**Seek algorithm**

1. Reject the request if uncalibrated, in `COOLDOWN` or `FAULT`, if the sensor is invalid, or if the target is outside the usable range (clamp is not allowed; report the error).
2. If already within the tolerance of the target (default ±8 mm), do nothing.
3. Start moving towards the target. Release when `distance_remaining <= coast_dir + margin` (margin default 2 mm, configurable).
4. `SETTLE`: wait until the reading is steady (change under 2 mm over 500 ms, max 2 s).
5. If the error is beyond tolerance, do at most one corrective nudge: a short pulse in the needed direction (pulse length from the error and speed, minimum 150 ms), then settle again. Never more than one correction.
6. Learn: after each completed seek, nudge the stored coast value for that direction towards the observed one (EMA, alpha 0.2, bounded to ±50 % of the calibrated value). Persist at most once per hour to save flash wear.
7. Report the final height and error.

**Rules**

- Plausibility: a filtered reading that jumps faster than 1.5 x the calibrated speed (for example a hand or object in the beam) is treated as invalid and stops a seek.
- Invariant 6 (stall check) applies to `JOG` and `SEEK` from this stage on. In `CALIBRATING`, a stalled reading is the expected end-stop signal and is handled by the routine instead.
- Presets on the touchscreen are single taps once unlocked; any touch during the seek stops it.
- "Save current height as preset" from a long press on a preset button (display) and from the web Settings tab.
- Web preset buttons follow the remote-move rules from stage 1.

**Acceptance**

- [ ] Sit and Stand land within ±8 mm of target in at least 9 of 10 runs each, without a correction in most runs.
- [ ] Stopping by touch, web STOP, or the handset mid-seek works every time.
- [ ] Blocking the sensor beam mid-seek stops the desk and raises a fault.
- [ ] Several back-to-back seeks hit the duty-cycle budget and stop cleanly with a countdown.

## Stage 6: MQTT interface

The desk appears in Home Assistant automatically through MQTT discovery, reports height and state, and accepts preset, target-height and stop commands, but never jogs remotely.

**Connection**

- `esp-mqtt` with broker URI, username, password and optional TLS CA from the stage 1 settings. Reconnect with exponential back-off (1 s up to 60 s).
- Base topic `desk/<device_id>` (device ID = hostname, configurable). Availability on `desk/<device_id>/status`, retained, `online` / `offline` via last will.
- Subscribe to `homeassistant/status`; on `online`, republish discovery and state.

**State**

- `desk/<device_id>/state`, retained JSON: `height_mm`, `state`, `target_mm`, `fault`, `duty_remaining_s`, `locked`, `remote_moves_allowed`, `last_error`.
- Publish at 2 Hz while moving, then on any change (height changes of 5 mm or more when idle) and every 60 s as a keep-alive.

**Discovery entities (one HA device: name, model, firmware version, configuration URL `http://desk.local`)**

| Entity | Type | Behaviour |
| --- | --- | --- |
| Height | sensor, device class distance, unit mm, state class measurement | From state JSON. |
| Target height | number, min/max from calibration, step 10 mm | Sends `GOTO_HEIGHT`. |
| Sit, Stand, custom presets | button (one per enabled preset) | Sends `GOTO_PRESET`. |
| Stop | button | Sends `STOP`; always accepted, even when remote moves are disabled. |
| State | sensor (enum) | idle, jog, seek, settle, calibrating, cooldown, fault. |
| Moving | binary sensor, device class moving |  |
| Fault | binary sensor, device class problem | Fault text as an attribute. |
| Duty budget remaining | sensor, unit s, diagnostic |  |
| Remote moves allowed | binary sensor, diagnostic | Read-only in HA; can only be changed on the device or web UI. |

Use per-entity discovery topics (`homeassistant/<component>/<device_id>_<object>/config`, retained) with `availability_topic` and `value_template` reading the shared state JSON. Commands go to `desk/<device_id>/cmd` as JSON (`{"cmd":"preset","id":"stand"}`, `{"cmd":"goto","mm":1080}`, `{"cmd":"stop"}`).

**Remote move rules**

- If remote moves are disabled, reject move commands, set `last_error`, and leave the desk still.
- If enabled, show a full-screen countdown (default 3 s, configurable 0 to 10 s) with "Tap to cancel" before an MQTT-started move. A tap cancels; so does the handset.
- No MQTT jog. Rate-limit move commands to one per 2 s.

**Acceptance**

- [ ] The device and all entities appear in HA without YAML, and survive an HA restart.
- [ ] A preset button in HA moves the desk after the countdown; tapping the screen cancels it.
- [ ] With remote moves disabled, HA commands do nothing except Stop, and the reason is visible in HA.
- [ ] Broker restart or Wi-Fi drop: the entity goes unavailable, then recovers by itself, and a move in progress is unaffected by the network loss.

## Open items to confirm on hardware

These are unknown until measured; each must be a config value, not a hard-coded guess.

- [ ] Handset connector pin order (UP / GND / NC / DOWN assumed) and pull-up voltage on UP and DOWN with buttons released.
- [ ] Whether the PC817 module switches reliably from a 3.3 V GPIO, or needs sink mode from 5 V; what its red jumpers do. Sets `DESK_OPTO_ACTIVE_LOW`.
- [ ] GPIO3 and GPIO7 stay idle (no pulses) through power-up, reset and flashing, checked with a meter or scope before the optos are connected.
- [ ] I2C addresses on the board bus, to confirm the VL53L0X (0x29) does not clash.
- [ ] JD9853 / AXS5106L init code from Waveshare's ESP-IDF demo for this board (the pin map is fixed in Hardware).
- [ ] Sensor mounting position, target size and distance at both ends of travel (target should fill the cone at full travel).
- [ ] Desk travel range, speed and coast distance (measured by stage 4).
