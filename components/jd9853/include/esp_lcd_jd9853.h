/*
 * esp_lcd panel driver for the JD9853 (Waveshare ESP32-C6-Touch-LCD-1.47).
 * Init sequence ported from Waveshare's ESP-IDF demo.
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "esp_lcd_panel_vendor.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    int cmd;
    const void *data;
    size_t data_bytes;
    unsigned int delay_ms;
} jd9853_lcd_init_cmd_t;

typedef struct {
    const jd9853_lcd_init_cmd_t *init_cmds; /* NULL = built-in Waveshare sequence */
    uint16_t init_cmds_size;
} jd9853_vendor_config_t;

esp_err_t esp_lcd_new_panel_jd9853(const esp_lcd_panel_io_handle_t io,
                                   const esp_lcd_panel_dev_config_t *panel_dev_config,
                                   esp_lcd_panel_handle_t *ret_panel);

#ifdef __cplusplus
}
#endif
