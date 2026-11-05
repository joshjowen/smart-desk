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
    MOTION_REQ_CALIBRATE,    /* stage 4 */
    MOTION_REQ_CLEAR_FAULT,  /* explicit user acknowledge */
} motion_req_type_t;

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
    bool height_valid;
    int32_t height_mm;
    int32_t target_mm;            /* valid in SEEK/SETTLE */
    bool presets_available;       /* calibrated (always true with the simulator) */
    int32_t min_mm, max_mm;       /* usable range */
    int32_t preset_mm[MOTION_PRESET_COUNT];
    uint32_t duty_remaining_s;
    uint32_t cooldown_remaining_s;
    char fault[48];               /* empty when no fault */
    char last_error[48];          /* last rejected request, for the UI / API */
    uint32_t last_error_seq;      /* increments with each new last_error */
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
