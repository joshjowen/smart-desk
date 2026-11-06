/* Calibration routine, driven by the motion task (private to the motion component). */
#pragma once

#include "motion.h"

/* Handles a calibration command in the motion task. Returns NULL if accepted,
 * otherwise the reason to reject it. `idle` = motion is IDLE. */
const char *calib_cmd(motion_cal_cmd_t cmd, bool idle, uint32_t duty_remaining_ms);
const char *calib_set_height(int32_t mm);

/* While running: the output the routine wants this tick. `out` is the actual
 * output (it lags `want` across the 300 ms direction-change gap). */
motion_dir_t calib_tick(int64_t now_us, motion_dir_t out);

/* Something outside the routine stopped it (STOP, fault, cooldown). */
void calib_abort(const char *why, bool cancelled);

/* Non-running phases time out (prompt / result messages). Call every tick. */
void calib_idle_tick(int64_t now_us, uint32_t duty_wait_s);

bool calib_running(void);
/* A session is open in a phase where the desk must not be moved by hand. */
bool calib_blocks_moves(void);
void calib_get_status(motion_cal_status_t *out);
/* After a successful SAVE: the new calibration (once). */
bool calib_take_saved(motion_cal_t *out);
/* Scale of the current calibration, carried into a new routine result (it is a property of the sensor mounting). */
void calib_set_scale(float scale);
