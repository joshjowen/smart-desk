/*
 * Board pin allocation for the Waveshare ESP32-C6-Touch-LCD-1.47 desk controller.
 * Single source of truth for GPIO numbers. See the Hardware section of the spec.
 */
#pragma once

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"

/* Opto outputs (handset UP / DOWN). Only the motion component may touch these.
 * Stage 0 never configures them: their safe level depends on the opto wiring
 * (CONFIG_DESK_OPTO_ACTIVE_LOW, added in stage 2). */
#define BOARD_PIN_OPTO_UP       GPIO_NUM_3   /* also TF card MISO: TF is never initialised */
#define BOARD_PIN_OPTO_DOWN     GPIO_NUM_7

/* TF card chip select: driven high right after boot so the card is never selected. */
#define BOARD_PIN_TF_CS         GPIO_NUM_4

/* Shared onboard I2C bus: AXS5106L touch, QMI8658 IMU (idle), VL53L0X (stage 3). */
#define BOARD_I2C_PORT          I2C_NUM_0
#define BOARD_PIN_I2C_SDA       GPIO_NUM_18
#define BOARD_PIN_I2C_SCL       GPIO_NUM_19
#define BOARD_I2C_HZ            400000

/* LCD: JD9853 on 4-wire SPI, write-only (no MISO, so GPIO3 is left alone). */
#define BOARD_LCD_SPI_HOST      SPI2_HOST
#define BOARD_PIN_LCD_SCK       GPIO_NUM_1
#define BOARD_PIN_LCD_MOSI      GPIO_NUM_2
#define BOARD_PIN_LCD_CS        GPIO_NUM_14
#define BOARD_PIN_LCD_DC        GPIO_NUM_15
#define BOARD_PIN_LCD_RST       GPIO_NUM_22
#define BOARD_PIN_LCD_BL        GPIO_NUM_23
#define BOARD_LCD_PCLK_HZ       (80 * 1000 * 1000)
#define BOARD_LCD_H_RES         172
#define BOARD_LCD_V_RES         320
#define BOARD_LCD_X_GAP         34   /* 172 px panel sits at columns 34..205 of the 240-wide controller */
#define BOARD_LCD_Y_GAP         0

/* Touch: AXS5106L */
#define BOARD_PIN_TP_RST        GPIO_NUM_20
#define BOARD_PIN_TP_INT        GPIO_NUM_21
#define BOARD_TP_I2C_ADDR       0x63

/* Do not use: 0 (battery ADC), 4/5/8/9/15 (strapping; 4 = TF CS, 15 = LCD DC),
 * 6 (IMU INT), 12/13 (USB), 16/17 (UART0). Handset sense candidate: GPIO17. */
