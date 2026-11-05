/*
 * esp_lcd_touch driver for the AXS5106L capacitive controller.
 * Ported from Waveshare's ESP-IDF demo.
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "esp_lcd_touch.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ESP_LCD_TOUCH_AXS5106_ADDRESS 0x63

/* dev must be an i2c_master device at ESP_LCD_TOUCH_AXS5106_ADDRESS. */
esp_err_t esp_lcd_touch_new_i2c_axs5106(i2c_master_dev_handle_t dev,
                                        const esp_lcd_touch_config_t *config,
                                        esp_lcd_touch_handle_t *out_touch);

#ifdef __cplusplus
}
#endif
