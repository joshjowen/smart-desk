/*
 * Compact register-level VL53L0X driver (C port of Pololu's VL53L0X Arduino
 * library, which follows ST's API init sequence: SPAD setup, tuning settings,
 * VHV and phase reference calibration).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VL53L0X_ADDR 0x29

/* Device range status (bits 6:3 of RESULT_RANGE_STATUS). 11 = range complete. */
#define VL53L0X_STATUS_OK 11

typedef struct {
    i2c_master_bus_handle_t bus;
    uint32_t scl_hz;
    /* Optional bus guard, called around every transaction (take = true / false). */
    void (*bus_lock)(bool take);
} vl53l0x_config_t;

typedef struct vl53l0x *vl53l0x_handle_t;

typedef struct {
    uint16_t range_mm;
    uint8_t status;          /* device range status, see vl53l0x_status_name() */
    float signal_mcps;       /* return signal rate */
    float ambient_mcps;      /* ambient rate */
} vl53l0x_sample_t;

/* Adds the device to the bus (does not talk to it). */
esp_err_t vl53l0x_new(const vl53l0x_config_t *cfg, vl53l0x_handle_t *out);
void vl53l0x_del(vl53l0x_handle_t dev);

/* Full init + reference calibration. ESP_ERR_NOT_FOUND if the model ID is wrong. */
esp_err_t vl53l0x_init(vl53l0x_handle_t dev);

esp_err_t vl53l0x_set_timing_budget_us(vl53l0x_handle_t dev, uint32_t budget_us);

/* Back-to-back continuous ranging (a new sample every timing budget). */
esp_err_t vl53l0x_start_continuous(vl53l0x_handle_t dev);
esp_err_t vl53l0x_stop_continuous(vl53l0x_handle_t dev);

/* Non-blocking: ESP_ERR_NOT_FINISHED if no new sample is ready yet. */
esp_err_t vl53l0x_read_sample(vl53l0x_handle_t dev, vl53l0x_sample_t *out);

const char *vl53l0x_status_name(uint8_t status);

#ifdef __cplusplus
}
#endif
