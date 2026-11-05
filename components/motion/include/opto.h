/*
 * Opto outputs (handset UP / DOWN). Only motion and app_main's very first line
 * may call these.
 */
#pragma once

#include <stdbool.h>
#include "motion.h"

#ifdef __cplusplus
extern "C" {
#endif

/* First call in app_main: drive both outputs to their off level, then make
 * them outputs with a pull towards off. Safe to call more than once. */
void opto_force_off(void);

/* Exactly one direction (or none) on. Both are switched off before either is switched on. */
void opto_set(motion_dir_t dir);

/* Logs the configured polarity and pins. */
void opto_log_config(void);

#ifdef __cplusplus
}
#endif
