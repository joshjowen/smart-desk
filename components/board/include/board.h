#pragma once

#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch.h"
#include "driver/i2c_master.h"
#include "board_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* First call in app_main. Parks pins that must never float (TF CS high).
 * Does NOT touch the opto pins in stage 0. */
void board_early_init(void);

/* Shared onboard I2C bus (touch, IMU, VL53L0X). */
esp_err_t board_i2c_init(void);
i2c_master_bus_handle_t board_i2c_bus(void);
void board_i2c_scan(void);
/* Bus mutex (recursive) shared by touch and the VL53L0X. */
bool board_i2c_lock(uint32_t timeout_ms);
void board_i2c_unlock(void);
/* Clocks out a stuck bus (after timeouts). */
void board_i2c_recover(void);
/* Touch reads that failed and were reported as "released". */
uint32_t board_touch_errors(void);

esp_err_t board_display_init(esp_lcd_panel_io_handle_t *io, esp_lcd_panel_handle_t *panel, size_t max_transfer_bytes);
esp_err_t board_touch_init(esp_lcd_touch_handle_t *touch);

esp_err_t board_backlight_init(void);
/* 0..100 % */
void board_backlight_set(uint8_t percent);
uint8_t board_backlight_get(void);

#ifdef __cplusplus
}
#endif
