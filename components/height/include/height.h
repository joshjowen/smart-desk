/*
 * Height sensor service: owns the VL53L0X, filters its readings and keeps a
 * short history for diagnostics. Stage 3: readings are shown only, motion does
 * not use them yet.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    HEIGHT_PROFILE_DEFAULT = 0,    /* CONFIG_DESK_HEIGHT_BUDGET_MS (50 ms, ~20 Hz) */
    HEIGHT_PROFILE_ACCURATE = 1,   /* CONFIG_DESK_HEIGHT_BUDGET_ACCURATE_MS (200 ms, ~5 Hz) */
} height_profile_t;

#define HEIGHT_HISTORY_S    60
#define HEIGHT_INVALID_MS   500     /* filtered reading invalid after this long without a good sample */

typedef struct {
    bool online;             /* sensor initialised and answering */
    bool valid;              /* filtered reading usable */
    bool raw_ok;             /* last sample had a good range status */
    uint16_t raw_mm;         /* last sample */
    uint8_t range_status;    /* last sample's device range status (11 = ok) */
    float filtered_mm;
    float signal_mcps;
    float ambient_mcps;
    float rate_hz;           /* samples per second over the last 2 s */
    float std_raw_mm;        /* standard deviation over the last 2 s (good samples) */
    float std_filt_mm;
    float span_filt_mm;      /* max - min of the filtered value over the last 2 s */
    uint32_t samples;        /* total samples since boot */
    uint32_t errors;         /* I2C errors since boot */
    uint32_t inits;          /* successful inits since boot */
    uint32_t retry_in_ms;    /* while offline: time to the next init attempt */
    uint8_t profile;
    uint8_t ema_pct;
    uint16_t budget_ms;
} height_status_t;

typedef struct {
    uint32_t t_ms;           /* since boot */
    uint16_t raw_mm;
    uint16_t filt_dmm;       /* filtered, 0.1 mm; HEIGHT_FILT_NONE if invalid */
    uint16_t signal_x128;    /* MCPS * 128 */
    uint16_t ambient_x128;
    uint8_t status;
} height_sample_t;

#define HEIGHT_FILT_NONE 0xFFFF

esp_err_t height_init(uint8_t profile, uint8_t ema_pct);
void height_get(height_status_t *out);
bool height_status_ok(uint8_t range_status, uint16_t raw_mm);

/* Applied by the sensor task within one poll. */
void height_set_params(uint8_t profile, uint8_t ema_pct);

/* Copies history samples with sequence number >= *seq (oldest first, at most
 * `max`), and advances *seq past the last one copied. Pass *seq = 0 for the
 * whole history. Returns the number copied. */
size_t height_history(uint32_t *seq, height_sample_t *buf, size_t max);

#ifdef __cplusplus
}
#endif
