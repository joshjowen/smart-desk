/*
 * Motion controller: the only code allowed to drive the desk outputs.
 * Everything else sends requests through motion_request() and reads
 * motion_get_status().
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MOTION_SRC_TOUCH,
    MOTION_SRC_WEB,
    MOTION_SRC_MQTT,
    MOTION_SRC_HANDSET,
    MOTION_SRC_SYSTEM,   /* console, internal */
} motion_source_t;

typedef enum {
    MOTION_REQ_JOG_UP,       /* hold-to-run: needs MOTION_REQ_HEARTBEAT (or a repeat) every <300 ms */
    MOTION_REQ_JOG_DOWN,
    MOTION_REQ_HEARTBEAT,
    MOTION_REQ_STOP,         /* always wins */
    MOTION_REQ_GOTO_HEIGHT,  /* value = mm */
    MOTION_REQ_GOTO_PRESET,  /* value = motion_preset_t */
    MOTION_REQ_CALIBRATE,    /* value = motion_cal_cmd_t */
    MOTION_REQ_CLEAR_FAULT,  /* explicit user acknowledge */
    MOTION_REQ_CAL_HEIGHT,   /* calibration: desktop height at the bottom, value = mm */
    MOTION_REQ_CAL_CORRECT,  /* tape-measured height right now, value = mm: corrects scale (or offset near the bottom) */
} motion_req_type_t;

typedef enum {
    MOTION_CAL_BEGIN,        /* open the routine: budget check, then the "clear the area" prompt */
    MOTION_CAL_START,        /* user pressed Start */
    MOTION_CAL_CANCEL,       /* stop and drop the session (old calibration kept) */
    MOTION_CAL_SAVE,
    MOTION_CAL_DISCARD,
    MOTION_CAL_CLOSE,        /* dismiss a result / budget message */
} motion_cal_cmd_t;

typedef enum {
    MOTION_CAL_NONE,
    MOTION_CAL_NEED_BUDGET,  /* not enough duty budget: wait_s tells how long */
    MOTION_CAL_PROMPT,       /* "clear the area, then Start" */
    MOTION_CAL_RUNNING,
    MOTION_CAL_ENTER_HEIGHT, /* tape measure at the bottom */
    MOTION_CAL_SUMMARY,      /* Save or Discard */
    MOTION_CAL_SAVED,
    MOTION_CAL_FAILED,
    MOTION_CAL_CANCELLED,
} motion_cal_phase_t;

#define MOTION_CAL_VERSION 2   /* v2 adds scale (v1 records load with scale 1) */

/* Height model: height_mm = h_min + sign * scale * (raw - raw_min).
 * raw_*, speeds and coasts are in sensor mm (multiply by scale for desk mm).
 * scale corrects the sensor's distance scale (1 until corrected with a second
 * tape measurement). Coasts are measured on the filtered reading (from the
 * value at release to the settled value), so they include the filter lag
 * that a seek will see. */
typedef struct {
    uint32_t version;
    bool valid;
    int8_t sign;
    float raw_min, raw_max;
    int32_t h_min_mm;
    float speed_up, speed_down;
    float coast_up, coast_down;
    float scale;               /* fills what was padding in v1, so the record size is unchanged */
    int64_t timestamp;         /* unix time, 0 if the clock was not set */
} motion_cal_t;

typedef struct {
    motion_cal_phase_t phase;
    uint8_t step, steps;       /* progress while running */
    char text[96];             /* current step, or the failure reason */
    uint32_t wait_s;           /* NEED_BUDGET: wait until enough budget */
    uint32_t need_s;           /* motor time the routine needs */
    uint32_t elapsed_s;
    motion_cal_t result;       /* values measured so far (complete in SUMMARY) */
} motion_cal_status_t;

typedef enum {
    MOTION_PRESET_SIT,
    MOTION_PRESET_STAND,
    MOTION_PRESET_CUSTOM1,
    MOTION_PRESET_CUSTOM2,
    MOTION_PRESET_COUNT,
} motion_preset_t;

typedef enum {
    MOTION_IDLE,
    MOTION_JOG,
    MOTION_SEEK,
    MOTION_SETTLE,
    MOTION_CALIBRATING,
    MOTION_COOLDOWN,
    MOTION_FAULT,
} motion_state_t;

typedef enum {
    MOTION_DIR_NONE = 0,
    MOTION_DIR_UP,
    MOTION_DIR_DOWN,
} motion_dir_t;

typedef struct {
    motion_req_type_t type;
    motion_source_t source;
    int32_t value;
} motion_req_t;

typedef struct {
    motion_state_t state;
    motion_dir_t output;          /* what the outputs are doing right now */
    motion_source_t source;       /* who started the current move */
    bool height_valid;
    int32_t height_mm;
    int32_t target_mm;            /* valid in SEEK/SETTLE */
    bool presets_available;       /* calibrated (always true with the simulator) */
    int32_t min_mm, max_mm;       /* usable range */
    int32_t preset_mm[MOTION_PRESET_COUNT];
    uint32_t duty_remaining_s;
    uint32_t cooldown_remaining_s;
    char fault[48];               /* empty when no fault */
    char last_fault[48];          /* most recent fault since boot, kept after acknowledge */
    char last_error[48];          /* last rejected request, for the UI / API */
    uint32_t last_error_seq;      /* increments with each new last_error */
    bool calibrated;
    motion_cal_t cal;             /* active calibration (valid if calibrated) */
    motion_cal_status_t cal_session;
    uint32_t max_move_s;
} motion_status_t;

esp_err_t motion_init(void);

/* Non-blocking. Returns ESP_ERR_TIMEOUT if the queue is full (STOP retries briefly). */
esp_err_t motion_request(const motion_req_t *req);

static inline esp_err_t motion_stop(motion_source_t src)
{
    motion_req_t r = { .type = MOTION_REQ_STOP, .source = src };
    return motion_request(&r);
}

static inline esp_err_t motion_jog(motion_dir_t dir, motion_source_t src)
{
    motion_req_t r = { .type = dir == MOTION_DIR_UP ? MOTION_REQ_JOG_UP : MOTION_REQ_JOG_DOWN, .source = src };
    return motion_request(&r);
}

static inline esp_err_t motion_heartbeat(motion_source_t src)
{
    motion_req_t r = { .type = MOTION_REQ_HEARTBEAT, .source = src };
    return motion_request(&r);
}

static inline esp_err_t motion_goto_preset(motion_preset_t p, motion_source_t src)
{
    motion_req_t r = { .type = MOTION_REQ_GOTO_PRESET, .source = src, .value = p };
    return motion_request(&r);
}

void motion_get_status(motion_status_t *out);

static inline esp_err_t motion_cal_cmd(motion_cal_cmd_t cmd, motion_source_t src)
{
    motion_req_t r = { .type = MOTION_REQ_CALIBRATE, .source = src, .value = cmd };
    return motion_request(&r);
}

/* Checks an imported / stored calibration. Returns NULL if OK, else the reason. */
const char *motion_cal_check(const motion_cal_t *c);
/* Installs a calibration (boot load or import). Applied by the motion task
 * within a tick; refused (returns ESP_ERR_INVALID_STATE) while moving or
 * during a calibration session. */
esp_err_t motion_set_calibration(const motion_cal_t *c);
/* Boot: installs the stored calibration directly (before motion_init). */
void motion_load_calibration(const motion_cal_t *c);
/* Called from the motion task when a calibration is saved by the routine or installed by import. */
void motion_set_cal_saved_cb(void (*cb)(const motion_cal_t *c));

/* Preset heights in mm, indexed by motion_preset_t; 0 = disabled. */
void motion_set_presets(const int32_t mm[MOTION_PRESET_COUNT]);

/* Usable height range for seeks and presets. */
void motion_get_range(int32_t *min_mm, int32_t *max_mm);

/* True while the desk is under automatic control (SEEK / SETTLE / CALIBRATING). */
bool motion_is_automatic(void);

const char *motion_state_name(motion_state_t s);
const char *motion_source_name(motion_source_t s);

#if CONFIG_DESK_DEBUG_COMMANDS
void motion_debug_fault(const char *text);
/* seconds < 0 clears the ledger */
void motion_debug_use_duty(int32_t seconds);
#endif

#ifdef __cplusplus
}
#endif
