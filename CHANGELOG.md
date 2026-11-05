# Changelog

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
- Open item confirmed: onboard I2C devices are 0x63 (AXS5106L) and 0x6b (QMI8658); 0x29 is free.
