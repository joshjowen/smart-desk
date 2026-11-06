/*
 * Motion controller task. Owns the desk outputs and enforces the safety
 * invariants from the spec:
 *   1  outputs off at boot (opto_force_off), shutdown handler, panic hook
 *   2  never UP and DOWN together; 300 ms off before a direction change
 *   3  hold-to-run: jog stops 300 ms after the last heartbeat
 *   4  max single move time -> FAULT
 *   5  duty-cycle ledger (RTC memory) -> refuse seeks / COOLDOWN
 *   6  stall check (JOG / SEEK with a calibrated sensor) and plausibility (SEEK)
 *   7  STOP from any source wins
 *   9  task watchdog on this task
 * Calibration (stage 4) runs inside this task through calib.c.
 */
#include "motion.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include "duty.h"
#include "opto.h"
#include "calib.h"
#include "height.h"

static const char *TAG = "motion";

#define TICK_MS              20
#define HEARTBEAT_TIMEOUT_US (300 * 1000)
#define DIR_CHANGE_OFF_US    (300 * 1000)
#define SETTLE_STEADY_MS     500    /* settle: change < SETTLE_MM over this long */
#define SETTLE_MM            2
#define SETTLE_MAX_MS        2000
#define NUDGE_MIN_MS         150
#define STALL_WINDOW_MS      1500   /* invariant 6: >= 5 mm within 1.5 s ... */
#define STALL_MIN_MM         5
#define STALL_AFTER_MS       2000   /* ... checked once the output has been on 2 s (soft start + filter lag) */
#define END_ZONE_MM          15     /* a jog stalling this close to an end stop just stopped there */
#define PLAUS_WINDOW_MS      500    /* plausibility: no faster than 1.5 x speed (+3 mm noise) over 0.5 s */
#define PLAUS_NOISE_MM       3
#define LEARN_PERSIST_US     (3600LL * 1000000)
#define HIST_N               128    /* heights at each 20 ms tick: 2.5 s */
#define SEEK_MIN_BUDGET_MS   (10 * 1000)
#define COOLDOWN_EXIT_MS     (10 * 1000)
#define END_STOP_MARGIN_MM   5
#define QUEUE_LEN            8

/* Internal request types (debug only), outside the public enum range. */
#define REQ_DBG_FAULT   100
#define REQ_DBG_DUTY    101

static QueueHandle_t s_q;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static motion_status_t s_pub;        /* published copy, guarded by s_lock */
static char s_dbg_fault[48];

/* ---- task-private state ---- */
static motion_state_t s_state = MOTION_IDLE;
static motion_dir_t s_out = MOTION_DIR_NONE;      /* actual output */
static motion_dir_t s_last_dir = MOTION_DIR_NONE; /* last direction that was on */
static int64_t s_off_since_us;
static int64_t s_on_since_us;
static motion_source_t s_src = MOTION_SRC_SYSTEM; /* source of the current move */
static motion_dir_t s_jog_dir;
static int64_t s_jog_deadline_us;
static int32_t s_target_mm;

/* remote (MQTT) move waiting for its countdown */
static struct {
    bool active;
    int32_t target_mm;
    int64_t at_us;
    motion_req_t req;
} s_pending_move;
static volatile uint32_t s_remote_countdown_s = 3;

/* seek sub-state (stage 5) */
typedef enum { SK_APPROACH, SK_SETTLE1, SK_NUDGE, SK_SETTLE2 } seek_phase_t;
static seek_phase_t s_sk;
static motion_dir_t s_sk_dir;          /* direction of the current approach / nudge */
static int32_t s_sk_release_h;         /* height when the approach was released */
static int64_t s_sk_t_us;              /* phase start (settle) or nudge output-on time */
static uint32_t s_sk_nudge_ms;
static bool s_sk_corrected;
static struct { uint32_t seq; int32_t target_mm, final_mm, error_mm; bool corrected; } s_last_seek;

/* height history for settle / stall / plausibility */
static int32_t s_hist_h[HIST_N];
static int64_t s_hist_t[HIST_N];
static int s_hist_n, s_hist_i;

/* learned coast (sensor mm) */
static float s_coast[2];               /* [0] up, [1] down */
static bool s_coast_dirty;
static int64_t s_coast_saved_us = -LEARN_PERSIST_US;
static void (*s_learn_cb)(float up, float down);
static char s_fault[48];
static char s_last_fault[48];
static portMUX_TYPE s_preset_lock = portMUX_INITIALIZER_UNLOCKED;
static char s_last_error[48];
static uint32_t s_last_error_seq;
static int32_t s_preset_mm[MOTION_PRESET_COUNT] = {
    CONFIG_DESK_PRESET_SIT_MM, CONFIG_DESK_PRESET_STAND_MM, 0, 0,
};

/* ---- height source / output backend ---- */
#if CONFIG_DESK_SIMULATOR
/* Simulated desk. The real outputs were forced off by opto_force_off() and stay off. */
static float s_sim_mm = CONFIG_DESK_SIM_START_MM;
static volatile motion_dir_t s_sim_dir = MOTION_DIR_NONE;
static const int32_t s_min_mm = CONFIG_DESK_SIM_MIN_MM + END_STOP_MARGIN_MM;
static const int32_t s_max_mm = CONFIG_DESK_SIM_MAX_MM - END_STOP_MARGIN_MM;
static const bool s_calibrated = true;

static void hw_set(motion_dir_t d)
{
    s_sim_dir = d;
}

static void hw_tick(uint32_t dt_ms)
{
    if (s_sim_dir == MOTION_DIR_NONE) {
        return;
    }
    float step = CONFIG_DESK_SIM_SPEED_MM_S * dt_ms / 1000.0f;
    s_sim_mm += s_sim_dir == MOTION_DIR_UP ? step : -step;
    if (s_sim_mm > CONFIG_DESK_SIM_MAX_MM) s_sim_mm = CONFIG_DESK_SIM_MAX_MM;
    if (s_sim_mm < CONFIG_DESK_SIM_MIN_MM) s_sim_mm = CONFIG_DESK_SIM_MIN_MM;
}

static bool height_read(int32_t *mm)
{
    *mm = (int32_t)lroundf(s_sim_mm);
    return true;
}
#else
/* Real desk through the optos, height from the VL53L0X through the calibration.
 * Until calibrated, height is unknown and only hold-to-run jogging is possible;
 * the range is then only used to sanity-check preset values in the settings. */
#define UNCAL_MIN_MM 500
#define UNCAL_MAX_MM 1400
static volatile int32_t s_min_mm = UNCAL_MIN_MM;
static volatile int32_t s_max_mm = UNCAL_MAX_MM;
static bool s_calibrated = false;
static motion_cal_t s_cal;

static void hw_set(motion_dir_t d)
{
    opto_set(d);
}

static void hw_tick(uint32_t dt_ms)
{
}

static bool height_read(int32_t *mm)
{
    *mm = 0;
    if (!s_calibrated) {
        return false;
    }
    height_status_t h;
    height_get(&h);
    if (!h.valid) {
        return false;
    }
    *mm = (int32_t)lroundf(s_cal.h_min_mm + s_cal.sign * s_cal.scale * (h.filtered_mm - s_cal.raw_min));
    return true;
}
#endif

/* Presets / seeks are enabled in stage 5. */
#define SEEKS_ENABLED 1

/* ---- optional handset sense ---- */
#if CONFIG_DESK_HANDSET_SENSE
#define HS_IGNORE_AFTER_OFF_US (150 * 1000)   /* our own opto pulls the line low too */
static int s_hs_pins[2] = { CONFIG_DESK_HANDSET_UP_GPIO, CONFIG_DESK_HANDSET_DOWN_GPIO };
static bool s_hs_raw[2], s_hs_pressed[2];

static void handset_init(void)
{
    for (int i = 0; i < 2; i++) {
        if (s_hs_pins[i] < 0) {
            continue;
        }
        const gpio_config_t c = { .pin_bit_mask = BIT64(s_hs_pins[i]), .mode = GPIO_MODE_INPUT };
        gpio_config(&c);
    }
    ESP_LOGI(TAG, "handset sense on UP=GPIO%d DOWN=GPIO%d", s_hs_pins[0], s_hs_pins[1]);
}
#endif

static void handle(const motion_req_t *r);
static void enter_fault(const char *text);

/* Invariant 4: fixed before calibration, travel / slowest speed x 1.3 after. */
static int64_t s_max_move_us = (int64_t)CONFIG_DESK_MAX_MOVE_S_UNCAL * 1000000;

static void (*s_cal_saved_cb)(const motion_cal_t *c);
static portMUX_TYPE s_cal_lock = portMUX_INITIALIZER_UNLOCKED;
static motion_cal_t s_pending_cal;
static bool s_pending;

/* ---- helpers ---- */

static const char *dir_name(motion_dir_t d)
{
    return d == MOTION_DIR_UP ? "UP" : d == MOTION_DIR_DOWN ? "DOWN" : "OFF";
}

const char *motion_state_name(motion_state_t s)
{
    static const char *n[] = { "idle", "jog", "seek", "settle", "calibrating", "cooldown", "fault" };
    return (unsigned)s < sizeof(n) / sizeof(n[0]) ? n[s] : "?";
}

const char *motion_source_name(motion_source_t s)
{
    static const char *n[] = { "TOUCH", "WEB", "MQTT", "HANDSET", "SYSTEM" };
    return (unsigned)s < sizeof(n) / sizeof(n[0]) ? n[s] : "?";
}

static portMUX_TYPE s_err_lock = portMUX_INITIALIZER_UNLOCKED;

static void set_last_error(const char *why)
{
    portENTER_CRITICAL(&s_err_lock);
    strlcpy(s_last_error, why, sizeof(s_last_error));
    s_last_error_seq++;
    portEXIT_CRITICAL(&s_err_lock);
}

void motion_note_error(const char *why)
{
    set_last_error(why);
    ESP_LOGW(TAG, "error noted: %s", why);
}

static void reject(const motion_req_t *r, const char *why)
{
    set_last_error(why);
    ESP_LOGW(TAG, "rejected req %d from %s: %s", r->type, motion_source_name(r->source), why);
}

static void set_state(motion_state_t st, const char *reason)
{
    if (st != s_state) {
        ESP_LOGI(TAG, "state %s -> %s (%s)", motion_state_name(s_state), motion_state_name(st), reason);
        s_state = st;
    }
}

/* Drive the outputs towards `want`, honouring the direction-change interlock. */
static void apply_output(motion_dir_t want, const char *reason)
{
    int64_t now = esp_timer_get_time();
    if (want == s_out) {
        return;
    }
    if (s_out != MOTION_DIR_NONE) {
        hw_set(MOTION_DIR_NONE);
        ESP_LOGI(TAG, "OUT %s -> OFF src=%s reason=%s (on %lld ms)", dir_name(s_out),
                 motion_source_name(s_src), reason, (now - s_on_since_us) / 1000);
        s_last_dir = s_out;
        s_out = MOTION_DIR_NONE;
        s_off_since_us = now;
    }
    if (want == MOTION_DIR_NONE) {
        return;
    }
    if (want != s_last_dir && s_last_dir != MOTION_DIR_NONE && now - s_off_since_us < DIR_CHANGE_OFF_US) {
        return; /* wait for the 300 ms off gap; retried next tick */
    }
    hw_set(want);
    s_out = want;
    s_on_since_us = now;
    ESP_LOGI(TAG, "OUT OFF -> %s src=%s reason=%s", dir_name(want), motion_source_name(s_src), reason);
}

static void enter_fault(const char *text)
{
    calib_abort(text, false);
    apply_output(MOTION_DIR_NONE, "fault");
    strlcpy(s_fault, text, sizeof(s_fault));
    strlcpy(s_last_fault, text, sizeof(s_last_fault));
    ESP_LOGE(TAG, "FAULT: %s", text);
    set_state(MOTION_FAULT, text);
}

static bool moving_state(void)
{
    return s_state == MOTION_JOG || s_state == MOTION_SEEK || s_state == MOTION_SETTLE ||
           s_state == MOTION_CALIBRATING;
}

/* ---- height history ---- */

static void hist_clear(void)
{
    s_hist_n = s_hist_i = 0;
}

static void hist_add(int64_t now, int32_t h)
{
    s_hist_h[s_hist_i] = h;
    s_hist_t[s_hist_i] = now;
    s_hist_i = (s_hist_i + 1) % HIST_N;
    if (s_hist_n < HIST_N) s_hist_n++;
}

/* Height `ago_ms` ago (newest sample at least that old). */
static bool hist_at(int64_t now, uint32_t ago_ms, int32_t *h)
{
    for (int k = 1; k <= s_hist_n; k++) {
        int i = (s_hist_i - k + HIST_N) % HIST_N;
        if (now - s_hist_t[i] >= (int64_t)ago_ms * 1000) {
            *h = s_hist_h[i];
            return true;
        }
    }
    return false;
}

/* max - min over the last win_ms; false if the history doesn't cover it. */
static bool hist_span(int64_t now, uint32_t win_ms, int32_t *span)
{
    int32_t lo = INT32_MAX, hi = INT32_MIN;
    bool covered = false;
    for (int k = 1; k <= s_hist_n; k++) {
        int i = (s_hist_i - k + HIST_N) % HIST_N;
        if (now - s_hist_t[i] > (int64_t)win_ms * 1000) {
            covered = true;
            break;
        }
        lo = s_hist_h[i] < lo ? s_hist_h[i] : lo;
        hi = s_hist_h[i] > hi ? s_hist_h[i] : hi;
    }
    *span = hi - lo;
    return covered;
}

/* ---- desk model in desk mm ---- */

static float speed_mm_s(motion_dir_t d)
{
#if CONFIG_DESK_SIMULATOR
    return CONFIG_DESK_SIM_SPEED_MM_S;
#else
    return s_cal.scale * (d == MOTION_DIR_UP ? s_cal.speed_up : s_cal.speed_down);
#endif
}

static float coast_mm(motion_dir_t d)
{
#if CONFIG_DESK_SIMULATOR
    return 0;
#else
    return s_cal.scale * s_coast[d == MOTION_DIR_UP ? 0 : 1];
#endif
}

#if !CONFIG_DESK_SIMULATOR
static void coast_bounds(int i, float *lo, float *hi)
{
    float cal = i == 0 ? s_cal.coast_up : s_cal.coast_down;
    *lo = cal * 0.5f;
    *hi = cal * 1.5f > 1.0f ? cal * 1.5f : 1.0f;
}

static void learn_coast(motion_dir_t d, float observed_desk_mm)
{
    int i = d == MOTION_DIR_UP ? 0 : 1;
    float obs = observed_desk_mm / s_cal.scale, lo, hi;
    coast_bounds(i, &lo, &hi);
    float before = s_coast[i];
    float v = before + CONFIG_DESK_COAST_LEARN_PCT / 100.0f * (obs - before);
    s_coast[i] = v < lo ? lo : v > hi ? hi : v;
    s_coast_dirty |= s_coast[i] != before;
    ESP_LOGI(TAG, "coast %s: observed %.1f, learned %.1f -> %.1f (sensor mm)", i ? "down" : "up", obs, before, s_coast[i]);
}
#endif

static void seek_done(int32_t h)
{
    s_last_seek.seq++;
    s_last_seek.target_mm = s_target_mm;
    s_last_seek.final_mm = h;
    s_last_seek.error_mm = h - s_target_mm;
    s_last_seek.corrected = s_sk_corrected;
    ESP_LOGI(TAG, "seek done: %ld mm (target %ld, error %+ld%s)", (long)h, (long)s_target_mm,
             (long)(h - s_target_mm), s_sk_corrected ? ", after one correction" : "");
    set_state(MOTION_IDLE, "seek done");
}

/* dry_run: only check (reject() on failure); returns true if a seek would start. */
static bool start_seek_ex(const motion_req_t *r, int32_t target, bool dry_run)
{
    int32_t h;
    if (s_state == MOTION_FAULT) { reject(r, "Fault: acknowledge first"); return false; }
    if (s_state == MOTION_COOLDOWN) { reject(r, "Cooling down"); return false; }
    if (calib_blocks_moves()) { reject(r, "Finish or cancel calibration first"); return false; }
    if (!s_calibrated) { reject(r, "Needs calibration"); return false; }
    if (!SEEKS_ENABLED) { reject(r, "Presets arrive in stage 5"); return false; }
    if (!height_read(&h)) { reject(r, "Height sensor invalid"); return false; }
    if (target < s_min_mm || target > s_max_mm) {
        char m[48];
        snprintf(m, sizeof(m), "Target %ld mm is outside %ld-%ld mm", (long)target, (long)s_min_mm, (long)s_max_mm);
        reject(r, m);
        return false;
    }
    if (duty_remaining_ms() < SEEK_MIN_BUDGET_MS) { reject(r, "Duty budget low"); return false; }
    if (abs(target - h) <= CONFIG_DESK_SEEK_TOLERANCE_MM) {
        ESP_LOGI(TAG, "seek to %ld: already there (%ld)", (long)target, (long)h);
        if (!dry_run && moving_state()) {
            apply_output(MOTION_DIR_NONE, "seek: at target");
            set_state(MOTION_IDLE, "at target");
        }
        return false;
    }
    if (dry_run) {
        return true;
    }
    s_src = r->source;
    s_target_mm = target;
    s_sk = SK_APPROACH;
    s_sk_dir = target > h ? MOTION_DIR_UP : MOTION_DIR_DOWN;
    s_sk_corrected = false;
    set_state(MOTION_SEEK, "seek request");
    ESP_LOGI(TAG, "seek %ld -> %ld mm src=%s (release %.1f mm early)", (long)h, (long)target,
             motion_source_name(r->source), coast_mm(s_sk_dir) + CONFIG_DESK_SEEK_MARGIN_MM);
    return true;
}

static void pending_cancel(const char *why)
{
    if (s_pending_move.active) {
        s_pending_move.active = false;
        ESP_LOGI(TAG, "remote move to %ld mm cancelled (%s)", (long)s_pending_move.target_mm, why);
    }
}

/* Seeks from MQTT start after an on-screen countdown (tap or STOP cancels). */
static void start_seek(const motion_req_t *r, int32_t target)
{
    if (r->source != MOTION_SRC_MQTT) {
        pending_cancel("replaced");
        start_seek_ex(r, target, false);
        return;
    }
    uint32_t cd = s_remote_countdown_s;
    if (cd == 0) {
        start_seek_ex(r, target, false);
        return;
    }
    if (moving_state()) {
        reject(r, "Desk is moving");
        return;
    }
    if (start_seek_ex(r, target, true)) {
        s_pending_move.active = true;
        s_pending_move.target_mm = target;
        s_pending_move.req = *r;
        s_pending_move.at_us = esp_timer_get_time() + (int64_t)cd * 1000000;
        ESP_LOGI(TAG, "remote move to %ld mm in %lu s (tap to cancel)", (long)target, (unsigned long)cd);
    }
}

static void apply_calibration(const motion_cal_t *c, bool reset_learned);

/* A tape-measured height at the current position. Far enough from the bottom it
 * corrects the scale (keeping h_min); near the bottom it corrects h_min. */
#define CORRECT_MIN_SPAN_MM 150
static void correct_height(const motion_req_t *r)
{
#if CONFIG_DESK_SIMULATOR
    reject(r, "No calibration on the simulated desk");
#else
    if (!s_calibrated) { reject(r, "Calibrate first"); return; }
    if (moving_state() || calib_blocks_moves()) { reject(r, "Wait until the desk is still"); return; }
    height_status_t h;
    height_get(&h);
    if (!h.valid) { reject(r, "Height sensor invalid"); return; }
    motion_cal_t c = s_cal;
    float d = c.sign * (h.filtered_mm - c.raw_min);   /* sensor mm above the bottom */
    if (d >= CORRECT_MIN_SPAN_MM) {
        float k = (r->value - c.h_min_mm) / d;
        if (!(k >= 0.5f && k <= 1.5f)) { reject(r, "That height doesn't fit the calibration"); return; }
        ESP_LOGI(TAG, "height correction at %.1f mm above the bottom: scale %.4f -> %.4f", d, c.scale, k);
        c.scale = k;
    } else {
        int32_t hmin = (int32_t)lroundf(r->value - c.scale * d);
        if (hmin < 300 || hmin > 1500) { reject(r, "That height doesn't fit the calibration"); return; }
        ESP_LOGI(TAG, "height correction near the bottom: h_min %ld -> %ld mm", (long)c.h_min_mm, (long)hmin);
        c.h_min_mm = hmin;
    }
    apply_calibration(&c, false);   /* coasts are in sensor mm: still valid */
    if (s_cal_saved_cb) {
        s_cal_saved_cb(&s_cal);
    }
#endif
}

static void handle(const motion_req_t *r)
{
    int64_t now = esp_timer_get_time();
    switch ((int)r->type) {
    case MOTION_REQ_STOP:
        pending_cancel(motion_source_name(r->source));
        if (s_state == MOTION_CALIBRATING) {
            calib_abort("Calibration cancelled. The old calibration is kept.", true);
        }
        if (moving_state()) {
            s_src = r->source;
            apply_output(MOTION_DIR_NONE, "stop request");
            set_state(MOTION_IDLE, "stop");
        }
        break;

    case MOTION_REQ_JOG_UP:
    case MOTION_REQ_JOG_DOWN: {
        motion_dir_t d = r->type == MOTION_REQ_JOG_UP ? MOTION_DIR_UP : MOTION_DIR_DOWN;
        if (s_state == MOTION_FAULT) { reject(r, "Fault: acknowledge first"); break; }
        if (s_state == MOTION_COOLDOWN) { reject(r, "Cooling down"); break; }
        if (s_state == MOTION_CALIBRATING || calib_blocks_moves()) { reject(r, "Finish or cancel calibration first"); break; }
        pending_cancel("jog");
        if (s_state != MOTION_JOG || s_jog_dir != d || s_src != r->source) {
            ESP_LOGI(TAG, "jog %s src=%s", dir_name(d), motion_source_name(r->source));
        }
        s_src = r->source;
        s_jog_dir = d;
        s_jog_deadline_us = now + HEARTBEAT_TIMEOUT_US;
        set_state(MOTION_JOG, "jog request");
        break;
    }

    case MOTION_REQ_HEARTBEAT:
        if (s_state == MOTION_JOG && r->source == s_src) {
            s_jog_deadline_us = now + HEARTBEAT_TIMEOUT_US;
        }
        break;

    case MOTION_REQ_GOTO_HEIGHT:
        start_seek(r, r->value);
        break;

    case MOTION_REQ_GOTO_PRESET:
    {
        int32_t mm = 0;
        if (r->value >= 0 && r->value < MOTION_PRESET_COUNT) {
            portENTER_CRITICAL(&s_preset_lock);
            mm = s_preset_mm[r->value];
            portEXIT_CRITICAL(&s_preset_lock);
        }
        if (mm <= 0) {
            reject(r, "Preset not set");
        } else {
            start_seek(r, mm);
        }
    }
        break;

    case MOTION_REQ_CALIBRATE: {
#if CONFIG_DESK_SIMULATOR
        reject(r, "No calibration on the simulated desk");
#else
        motion_cal_cmd_t cmd = (motion_cal_cmd_t)r->value;
        if (cmd == MOTION_CAL_START && (s_state == MOTION_FAULT || s_state == MOTION_COOLDOWN)) {
            reject(r, s_state == MOTION_FAULT ? "Fault: acknowledge first" : "Cooling down");
            break;
        }
        const char *err = calib_cmd(cmd, s_state == MOTION_IDLE, duty_remaining_ms());
        if (err) {
            reject(r, err);
        } else if (calib_running() && s_state != MOTION_CALIBRATING) {
            s_src = r->source;
            set_state(MOTION_CALIBRATING, "calibration started");
        }
#endif
        break;
    }

    case MOTION_REQ_CAL_CORRECT:
        correct_height(r);
        break;

    case MOTION_REQ_CAL_HEIGHT: {
        const char *err = calib_set_height(r->value);
        if (err) {
            reject(r, err);
        }
        break;
    }

    case MOTION_REQ_CLEAR_FAULT:
        if (s_state == MOTION_FAULT) {
            ESP_LOGI(TAG, "fault acknowledged by %s", motion_source_name(r->source));
            s_fault[0] = 0;
            set_state(MOTION_IDLE, "fault cleared");
        }
        break;

#if CONFIG_DESK_DEBUG_COMMANDS
    case REQ_DBG_FAULT:
        enter_fault(s_dbg_fault);
        break;
    case REQ_DBG_DUTY:
        if (r->value < 0) {
            duty_clear();
            ESP_LOGW(TAG, "debug: duty ledger cleared");
        } else {
            duty_add_ms((uint32_t)r->value * 1000);
            ESP_LOGW(TAG, "debug: added %ld s to duty ledger", (long)r->value);
        }
        break;
#endif
    default:
        reject(r, "Unknown request");
        break;
    }
}

/* Drain the queue. STOP always wins: once a STOP is seen, later move requests in the same batch are dropped. */
static void process_queue(void)
{
    motion_req_t r;
    bool stopped = false;
    while (xQueueReceive(s_q, &r, 0) == pdTRUE) {
        if (r.type == MOTION_REQ_STOP) {
            stopped = true;
            handle(&r);
        } else if (stopped && (r.type == MOTION_REQ_JOG_UP || r.type == MOTION_REQ_JOG_DOWN ||
                               r.type == MOTION_REQ_HEARTBEAT || r.type == MOTION_REQ_GOTO_HEIGHT ||
                               r.type == MOTION_REQ_GOTO_PRESET)) {
            continue;
        } else {
            handle(&r);
        }
    }
}

const char *motion_cal_check(const motion_cal_t *c)
{
    if (c->version != 1 && c->version != MOTION_CAL_VERSION) return "Unsupported calibration version";
    if (c->version >= 2 && !(c->scale >= 0.5f && c->scale <= 1.5f)) return "Scale must be 0.5-1.5";
    if (c->sign != 1 && c->sign != -1) return "Sign must be +1 or -1";
    if (!(c->raw_min >= 0 && c->raw_min <= 2000 && c->raw_max >= 0 && c->raw_max <= 2000)) return "Raw readings out of range";
    if (c->sign * (c->raw_max - c->raw_min) < 100) return "Travel under 100 mm";
    if (c->h_min_mm < 300 || c->h_min_mm > 1500) return "Bottom height must be 300-1500 mm";
    if (!(c->speed_up >= 3 && c->speed_up <= 100 && c->speed_down >= 3 && c->speed_down <= 100)) return "Speeds must be 3-100 mm/s";
    if (!(c->coast_up >= 0 && c->coast_up <= 100 && c->coast_down >= 0 && c->coast_down <= 100)) return "Coasts must be 0-100 mm";
    return NULL;
}

static void apply_calibration(const motion_cal_t *c, bool reset_learned)
{
#if CONFIG_DESK_SIMULATOR
    (void)c;
#else
    if (!c->valid || motion_cal_check(c)) {
        s_calibrated = false;
        s_min_mm = UNCAL_MIN_MM;
        s_max_mm = UNCAL_MAX_MM;
        s_max_move_us = (int64_t)CONFIG_DESK_MAX_MOVE_S_UNCAL * 1000000;
        ESP_LOGW(TAG, "no valid calibration: hold-to-run only");
        return;
    }
    s_cal = *c;
    if (s_cal.version < 2) {
        s_cal.version = MOTION_CAL_VERSION;
        s_cal.scale = 1.0f;
    }
    c = &s_cal;
    calib_set_scale(c->scale);
    float travel = c->sign * (c->raw_max - c->raw_min);   /* sensor mm */
    int32_t h_max = (int32_t)lroundf(c->h_min_mm + c->scale * travel);
    s_min_mm = c->h_min_mm + END_STOP_MARGIN_MM;
    s_max_mm = h_max - END_STOP_MARGIN_MM;
    float slow = fminf(c->speed_up, c->speed_down);
    s_max_move_us = (int64_t)(travel / slow * 1.3f * 1000000);
    s_calibrated = true;
    if (reset_learned) {
        s_coast[0] = c->coast_up;
        s_coast[1] = c->coast_down;
        s_coast_dirty = true;   /* persist the reset too */
    }
    ESP_LOGI(TAG, "calibration: %ld-%ld mm (usable %ld-%ld), scale %.4f, speed up %.1f down %.1f mm/s, coast up %.1f "
             "down %.1f mm, max move %.1f s", (long)c->h_min_mm, (long)h_max, (long)s_min_mm, (long)s_max_mm, c->scale,
             c->speed_up * c->scale, c->speed_down * c->scale, c->coast_up * c->scale, c->coast_down * c->scale,
             s_max_move_us / 1e6);
#endif
}

esp_err_t motion_set_calibration(const motion_cal_t *c)
{
    if (moving_state() || calib_blocks_moves()) {
        return ESP_ERR_INVALID_STATE;
    }
    portENTER_CRITICAL(&s_cal_lock);
    s_pending_cal = *c;
    s_pending = true;
    portEXIT_CRITICAL(&s_cal_lock);
    return ESP_OK;
}

void motion_load_calibration(const motion_cal_t *c)
{
    apply_calibration(c, true);
    s_coast_dirty = false;   /* boot load: nothing new to persist */
}

void motion_load_learned_coast(float up, float down)
{
#if !CONFIG_DESK_SIMULATOR
    if (!s_calibrated) {
        return;
    }
    float v[2] = { up, down };
    for (int i = 0; i < 2; i++) {
        float lo, hi;
        coast_bounds(i, &lo, &hi);
        if (v[i] >= lo && v[i] <= hi) {
            s_coast[i] = v[i];
        }
    }
    ESP_LOGI(TAG, "learned coast: up %.1f down %.1f (sensor mm)", s_coast[0], s_coast[1]);
#endif
}

void motion_set_learn_cb(void (*cb)(float up, float down))
{
    s_learn_cb = cb;
}

void motion_set_cal_saved_cb(void (*cb)(const motion_cal_t *c))
{
    s_cal_saved_cb = cb;
}

static void step(uint32_t dt_ms)
{
    int64_t now = esp_timer_get_time();
    int32_t h = 0;
    bool h_ok = height_read(&h);

    if (s_out != MOTION_DIR_NONE) {
        duty_add_ms(dt_ms);
    }

#if CONFIG_DESK_HANDSET_SENSE
    /* Debounced over two 20 ms ticks. A line is ignored while our own output for
     * that direction is on (and briefly after), since the opto pulls it too. */
    for (int i = 0; i < 2; i++) {
        if (s_hs_pins[i] < 0) {
            continue;
        }
        motion_dir_t d = i == 0 ? MOTION_DIR_UP : MOTION_DIR_DOWN;
        bool ours = s_out == d || (s_last_dir == d && now - s_off_since_us < HS_IGNORE_AFTER_OFF_US);
        bool raw = gpio_get_level(s_hs_pins[i]) == (CONFIG_DESK_HANDSET_ACTIVE_LOW ? 0 : 1);
        bool stable = raw == s_hs_raw[i];
        s_hs_raw[i] = raw;
        if (!stable || ours || raw == s_hs_pressed[i]) {
            continue;
        }
        s_hs_pressed[i] = raw;
        ESP_LOGI(TAG, "HANDSET %s %s", dir_name(d), raw ? "pressed" : "released");
        if (raw && moving_state() && s_state != MOTION_JOG) {
            motion_req_t stop = { .type = MOTION_REQ_STOP, .source = MOTION_SRC_HANDSET };
            handle(&stop);
        }
    }
    /* The handset moves the motor too: count it against the duty budget. */
    if (s_out == MOTION_DIR_NONE && (s_hs_pressed[0] || s_hs_pressed[1])) {
        duty_add_ms(dt_ms);
    }
#endif

    /* Invariant 4: max single move time. */
    if (s_out != MOTION_DIR_NONE && now - s_on_since_us > s_max_move_us) {
        enter_fault("Move took too long");
    }

    /* Invariant 5: duty budget. */
    uint32_t rem = duty_remaining_ms();
    if (rem == 0 && s_state != MOTION_COOLDOWN && s_state != MOTION_FAULT) {
        calib_abort("Motor time ran out. Calibration stopped; try again after the cooldown.", false);
        apply_output(MOTION_DIR_NONE, "duty budget used");
        set_state(MOTION_COOLDOWN, "duty budget used");
    } else if (s_state == MOTION_COOLDOWN && rem >= COOLDOWN_EXIT_MS) {
        set_state(MOTION_IDLE, "cooldown over");
    }

    /* Height history for settle / stall / plausibility. */
    if (h_ok) {
        hist_add(now, h);
    } else {
        hist_clear();
    }

    /* Plausibility (seeks): the reading can't change faster than 1.5 x the desk's speed. */
    if (h_ok && (s_state == MOTION_SEEK || s_state == MOTION_SETTLE)) {
        int32_t old;
        float vmax = fmaxf(speed_mm_s(MOTION_DIR_UP), speed_mm_s(MOTION_DIR_DOWN));
        if (hist_at(now, PLAUS_WINDOW_MS, &old) &&
            abs(h - old) > 1.5f * vmax * PLAUS_WINDOW_MS / 1000 + PLAUS_NOISE_MM) {
            ESP_LOGW(TAG, "implausible height change %ld -> %ld mm in %d ms", (long)old, (long)h, PLAUS_WINDOW_MS);
            enter_fault("Height jumped: something in the sensor beam?");
        }
    }

    /* Invariant 6: stall check while driving JOG / SEEK with a calibrated sensor. */
    if (h_ok && s_out != MOTION_DIR_NONE && (s_state == MOTION_JOG || s_state == MOTION_SEEK) &&
        now - s_on_since_us >= (int64_t)STALL_AFTER_MS * 1000) {
        int32_t old;
        if (hist_at(now, STALL_WINDOW_MS, &old) && abs(h - old) < STALL_MIN_MM) {
            bool at_end = (s_out == MOTION_DIR_UP && h >= s_max_mm + END_STOP_MARGIN_MM - END_ZONE_MM) ||
                          (s_out == MOTION_DIR_DOWN && h <= s_min_mm - END_STOP_MARGIN_MM + END_ZONE_MM);
            if (s_state == MOTION_JOG && at_end) {
                /* Holding a jog into the end stop: the desk has simply arrived. */
                motion_req_t r = { .type = MOTION_REQ_JOG_UP, .source = s_src };
                reject(&r, s_out == MOTION_DIR_UP ? "At the top" : "At the bottom");
                apply_output(MOTION_DIR_NONE, "end stop");
                set_state(MOTION_IDLE, "end stop");
            } else {
                ESP_LOGW(TAG, "stall: %ld -> %ld mm in %d ms while driving %s", (long)old, (long)h, STALL_WINDOW_MS,
                         dir_name(s_out));
                enter_fault("Desk not moving (stall)");
            }
        }
    }

    /* A remote move whose countdown has run out: re-check everything and go. */
    if (s_pending_move.active) {
        if (s_state != MOTION_IDLE) {
            pending_cancel(motion_state_name(s_state));
        } else if (now >= s_pending_move.at_us) {
            s_pending_move.active = false;
            start_seek_ex(&s_pending_move.req, s_pending_move.target_mm, false);
        }
    }

    motion_dir_t want = MOTION_DIR_NONE;
    switch (s_state) {
    case MOTION_JOG:
        if (now > s_jog_deadline_us) {
            apply_output(MOTION_DIR_NONE, "heartbeat timeout");
            set_state(MOTION_IDLE, "heartbeat timeout");
        } else {
            want = s_jog_dir;
        }
        break;
    case MOTION_SEEK: {
        if (!h_ok) {
            enter_fault("Height reading lost during a move");
            break;
        }
        if (s_sk == SK_APPROACH) {
            /* Release early by the coast distance (+ margin) so the desk coasts onto the target. */
            int32_t remaining = s_sk_dir == MOTION_DIR_UP ? s_target_mm - h : h - s_target_mm;
            if (remaining <= coast_mm(s_sk_dir) + CONFIG_DESK_SEEK_MARGIN_MM) {
                s_sk_nudge_ms = s_out == s_sk_dir ? (uint32_t)((now - s_on_since_us) / 1000) : 0;  /* on-time, for learning */
                apply_output(MOTION_DIR_NONE, "seek: release");
                s_sk_release_h = h;
                s_sk = SK_SETTLE1;
                s_sk_t_us = now;
                set_state(MOTION_SETTLE, "released");
            } else {
                want = s_sk_dir;
            }
        } else if (s_sk == SK_NUDGE) {
            if (s_out == s_sk_dir && s_sk_t_us == 0) {
                s_sk_t_us = now;   /* pulse length counts from when the output is really on */
            }
            if (s_sk_t_us && now - s_sk_t_us >= (int64_t)s_sk_nudge_ms * 1000) {
                apply_output(MOTION_DIR_NONE, "seek: nudge done");
                s_sk = SK_SETTLE2;
                s_sk_t_us = now;
                set_state(MOTION_SETTLE, "nudged");
            } else {
                want = s_sk_dir;
            }
        }
        break;
    }
    case MOTION_CALIBRATING:
        want = calib_tick(now, s_out);
        if (!calib_running()) {
            apply_output(MOTION_DIR_NONE, "calibration ended");
            set_state(MOTION_IDLE, "calibration ended");
        }
        break;
    case MOTION_SETTLE: {
        if (!h_ok) {
            enter_fault("Height reading lost during a move");
            break;
        }
        /* Steady: change < 2 mm over 500 ms (after release), or 2 s at most. */
        int32_t span;
        uint32_t since = (uint32_t)((now - s_sk_t_us) / 1000);
        bool steady = since >= SETTLE_STEADY_MS && hist_span(now, SETTLE_STEADY_MS, &span) && span < SETTLE_MM;
        if (!steady && since < SETTLE_MAX_MS) {
            break;
        }
        int32_t err = s_target_mm - h;
        if (s_sk == SK_SETTLE1) {
#if !CONFIG_DESK_SIMULATOR
            /* Learn from full-speed approaches only (short ones coast less). */
            if (s_sk_nudge_ms >= 2000) {
                int32_t moved = s_sk_dir == MOTION_DIR_UP ? h - s_sk_release_h : s_sk_release_h - h;
                learn_coast(s_sk_dir, moved);
            }
#endif
            if (abs(err) > CONFIG_DESK_SEEK_TOLERANCE_MM) {
                /* One corrective nudge, sized from the error and speed. */
                s_sk_dir = err > 0 ? MOTION_DIR_UP : MOTION_DIR_DOWN;
                uint32_t ms = (uint32_t)(abs(err) / speed_mm_s(s_sk_dir) * 1000);
                s_sk_nudge_ms = ms < NUDGE_MIN_MS ? NUDGE_MIN_MS : ms;
                s_sk = SK_NUDGE;
                s_sk_t_us = 0;
                s_sk_corrected = true;
                ESP_LOGI(TAG, "seek: %ld mm (error %+ld), nudge %s %lu ms", (long)h, (long)-err,
                         s_sk_dir == MOTION_DIR_UP ? "up" : "down", (unsigned long)s_sk_nudge_ms);
                set_state(MOTION_SEEK, "correction");
                break;
            }
        }
        seek_done(h);
        break;
    }
    default:
        break;
    }
    apply_output(want, s_state == MOTION_JOG ? "jog" : s_state == MOTION_SEEK ? "seek" :
                       s_state == MOTION_CALIBRATING ? "calibrate" : "idle");

    calib_idle_tick(now, 0);
    motion_cal_t nc;
    bool apply_new = calib_take_saved(&nc);
    if (!apply_new && s_pending && !moving_state() && !calib_blocks_moves()) {
        portENTER_CRITICAL(&s_cal_lock);
        nc = s_pending_cal;
        s_pending = false;
        portEXIT_CRITICAL(&s_cal_lock);
        apply_new = true;
    }
    if (apply_new) {
        apply_calibration(&nc, true);
        if (s_cal_saved_cb) {
            s_cal_saved_cb(&nc);
        }
    }
    /* Persist learned coasts at most once an hour (flash wear). */
    if (s_coast_dirty && s_learn_cb && !moving_state() && now - s_coast_saved_us >= LEARN_PERSIST_US) {
        s_learn_cb(s_coast[0], s_coast[1]);
        s_coast_dirty = false;
        s_coast_saved_us = now;
    }

    motion_status_t st = {
        .state = s_state,
        .output = s_out,
        .source = s_src,
        .height_valid = h_ok,
        .height_mm = h,
        .target_mm = (s_state == MOTION_SEEK || s_state == MOTION_SETTLE) ? s_target_mm : 0,
        .coast_up_mm = coast_mm(MOTION_DIR_UP),
        .coast_down_mm = coast_mm(MOTION_DIR_DOWN),
        .presets_available = s_calibrated && SEEKS_ENABLED,
        .calibrated = s_calibrated,
        .max_move_s = (uint32_t)(s_max_move_us / 1000000),
        .min_mm = s_min_mm,
        .max_mm = s_max_mm,
        .duty_remaining_s = rem / 1000,
        .cooldown_remaining_s = s_state == MOTION_COOLDOWN ? duty_wait_s_for(COOLDOWN_EXIT_MS) : 0,
    };
    portENTER_CRITICAL(&s_preset_lock);
    memcpy(st.preset_mm, s_preset_mm, sizeof(st.preset_mm));
    portEXIT_CRITICAL(&s_preset_lock);
    strlcpy(st.last_fault, s_last_fault, sizeof(st.last_fault));
    strlcpy(st.fault, s_state == MOTION_FAULT ? s_fault : "", sizeof(st.fault));
    portENTER_CRITICAL(&s_err_lock);
    strlcpy(st.last_error, s_last_error, sizeof(st.last_error));
    st.last_error_seq = s_last_error_seq;
    portEXIT_CRITICAL(&s_err_lock);
    st.pending_active = s_pending_move.active;
    st.pending_target_mm = s_pending_move.target_mm;
    st.pending_remaining_ms = s_pending_move.active && s_pending_move.at_us > now ?
                              (uint32_t)((s_pending_move.at_us - now) / 1000) : 0;
#if !CONFIG_DESK_SIMULATOR
    st.cal = s_cal;
#endif
    calib_get_status(&st.cal_session);
    st.last_seek.seq = s_last_seek.seq;
    st.last_seek.target_mm = s_last_seek.target_mm;
    st.last_seek.final_mm = s_last_seek.final_mm;
    st.last_seek.error_mm = s_last_seek.error_mm;
    st.last_seek.corrected = s_last_seek.corrected;
    portENTER_CRITICAL(&s_lock);
    s_pub = st;
    portEXIT_CRITICAL(&s_lock);
}

static void motion_task(void *arg)
{
    ESP_ERROR_CHECK(esp_task_wdt_add(NULL));
    TickType_t last_wake = xTaskGetTickCount();
    int64_t last_us = esp_timer_get_time();
    for (;;) {
        process_queue();
        int64_t now = esp_timer_get_time();
        uint32_t dt = (uint32_t)((now - last_us) / 1000);
        last_us += (int64_t)dt * 1000;
        hw_tick(dt);
        step(dt);
        esp_task_wdt_reset();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TICK_MS));
    }
}

static void shutdown_outputs_off(void)
{
    opto_set(MOTION_DIR_NONE);
}

esp_err_t motion_init(void)
{
    opto_force_off();   /* already done first thing in app_main; repeat to be sure */
    hw_set(MOTION_DIR_NONE);
    opto_log_config();
#if CONFIG_DESK_SIMULATOR
    ESP_LOGW(TAG, "SIMULATED desk: real outputs held off");
#else
    ESP_LOGW(TAG, "REAL desk: outputs on GPIO, height from the VL53L0X via calibration");
#endif
#if CONFIG_DESK_HANDSET_SENSE
    handset_init();
#endif
    duty_init();
    s_q = xQueueCreate(QUEUE_LEN, sizeof(motion_req_t));
    if (!s_q) {
        return ESP_ERR_NO_MEM;
    }
    esp_register_shutdown_handler(shutdown_outputs_off);
    step(0); /* publish an initial status */
    BaseType_t ok = xTaskCreate(motion_task, "motion", 4096, NULL, 6, NULL);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

esp_err_t motion_request(const motion_req_t *req)
{
    /* A STOP must get through: wait briefly if the queue is full. */
    TickType_t wait = req->type == MOTION_REQ_STOP ? pdMS_TO_TICKS(50) : 0;
    return xQueueSend(s_q, req, wait) == pdTRUE ? ESP_OK : ESP_ERR_TIMEOUT;
}

void motion_get_status(motion_status_t *out)
{
    portENTER_CRITICAL(&s_lock);
    *out = s_pub;
    portEXIT_CRITICAL(&s_lock);
}

void motion_set_presets(const int32_t mm[MOTION_PRESET_COUNT])
{
    portENTER_CRITICAL(&s_preset_lock);
    memcpy(s_preset_mm, mm, sizeof(s_preset_mm));
    portEXIT_CRITICAL(&s_preset_lock);
}

void motion_get_range(int32_t *min_mm, int32_t *max_mm)
{
    *min_mm = s_min_mm;
    *max_mm = s_max_mm;
}

bool motion_is_automatic(void)
{
    portENTER_CRITICAL(&s_lock);
    motion_state_t s = s_pub.state;
    bool pending = s_pub.pending_active;
    portEXIT_CRITICAL(&s_lock);
    return pending || s == MOTION_SEEK || s == MOTION_SETTLE || s == MOTION_CALIBRATING;
}

void motion_set_remote_countdown(uint32_t seconds)
{
    s_remote_countdown_s = seconds > 10 ? 10 : seconds;
}

#if CONFIG_DESK_DEBUG_COMMANDS
void motion_debug_fault(const char *text)
{
    strlcpy(s_dbg_fault, text, sizeof(s_dbg_fault));
    motion_req_t r = { .type = (motion_req_type_t)REQ_DBG_FAULT, .source = MOTION_SRC_SYSTEM };
    motion_request(&r);
}

void motion_debug_use_duty(int32_t seconds)
{
    motion_req_t r = { .type = (motion_req_type_t)REQ_DBG_DUTY, .source = MOTION_SRC_SYSTEM, .value = seconds };
    motion_request(&r);
}
#endif
