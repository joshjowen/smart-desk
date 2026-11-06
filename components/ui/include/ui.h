#pragma once

#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch.h"
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Starts LVGL (esp_lvgl_port), builds the lock and control screens and
 * boots to the lock screen. Backlight must already be initialised. */
esp_err_t ui_init(esp_lcd_panel_io_handle_t io, esp_lcd_panel_handle_t panel, esp_lcd_touch_handle_t touch);

/* Config listener: units, timeouts, backlight, preset names. Safe from any task. */
void ui_apply_config(const desk_config_t *cfg);

/* True while the lock screen is showing (web moves need the display unlocked
 * unless remote moves are allowed). */
bool ui_is_locked(void);

/* MQTT indicator in the status row: 0 = not configured (hidden), 1 = connecting, 2 = connected. */
void ui_set_mqtt_state(int state);

#ifdef __cplusplus
}
#endif
