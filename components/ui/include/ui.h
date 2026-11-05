#pragma once

#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Starts LVGL (esp_lvgl_port), builds the lock and control screens and
 * boots to the lock screen. Backlight must already be initialised. */
esp_err_t ui_init(esp_lcd_panel_io_handle_t io, esp_lcd_panel_handle_t panel, esp_lcd_touch_handle_t touch);

#ifdef __cplusplus
}
#endif
