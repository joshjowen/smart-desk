/*
 * Calibration routine (stage 4). Runs inside the motion task: it only says
 * which way it wants the outputs, and motion applies that with all the
 * invariants still in force (interlock, max single move time, duty budget,
 * STOP wins).
 *
 * Order (changed from the spec so it fits the duty budget on a desk with
 * ~31 s full travel; see README):
 *   1  probe: UP until the reading has moved 20 mm (2-3 s) -> sign. If the
 *      desk is already at the top, probe DOWN instead (that run doubles as
 *      the first coast-down test).
 *   2  continue UP to the top stop (stall: < 2 mm over 1 s) -> raw_max
 *   3,4 coast tests DOWN (3 s run, release, wait for the reading to settle)
 *   5  DOWN to the bottom stop -> raw_min, speed_down
 *   6,7 coast tests UP
 *   8  back DOWN to the bottom stop (checks raw_min repeats)
 *   then the tape-measure entry at the bottom, summary, Save / Discard.
 * Long runs are split into segments of at most 20 s with a 1 s pause, so no
 * single move gets near the max move time.
 */
#include "calib.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "sdkconfig.h"
#include "duty.h"
#include "height.h"

static const char *TAG = "calib";

#define SEG_MAX_MS           20000
#define SEG_PAUSE_MS         1000
#define LEG_MAX_MS           60000
#define STALL_AFTER_MS       1500
#define STALL_WINDOW_MS      1000
#define STALL_MM             2.0f
#define PROBE_MIN_MS         2000
#define PROBE_MAX_MS         3000
#define PROBE_MM             20.0f
#define COAST_RUN_MS         3000
#define REST_MS              1000
#define AVG_N                20
#define AVG_TIMEOUT_MS       4000
#define SETTLE_MIN_MS        700
#define SETTLE_WINDOW_MS     500
#define SETTLE_MAX_MS        3000
#define SETTLE_AVG_N         10
#define SPEED_SKIP_MS        1000
#define SPEED_STALL_TAIL_MS  1500
#define SPEED_MIN_WINDOW_MS  1500
#define TOTAL_MAX_MS         180000
#define REPEAT_TOL_MM        15.0f
#define MIN_TRAVEL_MM        100.0f
#define PROMPT_TIMEOUT_US    (5LL * 60 * 1000000)
#define ENTRY_TIMEOUT_US     (30LL * 60 * 1000000)
#define STEPS                8

typedef enum {
    K_REST0, K_PROBE_UP, K_PROBE_DOWN, K_LEG_UP, K_AVG_TOP, K_COAST_DN1, K_COAST_DN2,
    K_LEG_DOWN, K_AVG_BOTTOM, K_COAST_UP1, K_COAST_UP2, K_LEG_DOWN2, K_AVG_BOTTOM2,
} step_t;

typedef enum { SUB_RUN, SUB_PAUSE, SUB_AVG, SUB_SETTLE } sub_t;

#define RUN_BUF 512
#define REC_N   64

static struct {
    motion_cal_phase_t phase;
    int64_t phase_since_us;
    char text[96];
    uint8_t step;
    motion_cal_t r;
    bool saved_pending;

    step_t k;
    sub_t sub;
    int64_t t0_us, sub_us;
    uint32_t seq;
    float last_f;          /* latest filtered sample */
    bool have_f;

    /* run */
    motion_dir_t run_dir;
    int64_t run_on_us;
    uint32_t run_max_ms;
    bool run_stall_check;
    float run_start_f;
    uint32_t leg_on_ms;

    /* average */
    float avg_sum;
    int avg_n, avg_want;

    /* settle */
    float rel_f;
    bool settle_avg;

    float start_raw;
    float coast_sum[2];    /* [0] up, [1] down */
    int coast_n[2];
    uint32_t best_win_ms[2];
} C;

static float s_scale = 1.0f;

void calib_set_scale(float scale)
{
    s_scale = scale;
}

static uint32_t s_run_t[RUN_BUF];
static float s_run_f[RUN_BUF];
static int s_run_n;
static uint32_t s_rec_t[REC_N];
static float s_rec_f[REC_N];
static int s_rec_n, s_rec_i;

static uint32_t ms_now(void)
{
    return (uint32_t)(esp_timer_get_time() / 1000);
}

static const char *step_text(step_t k)
{
    switch (k) {
    case K_REST0: case K_PROBE_UP: case K_PROBE_DOWN: return "Checking the sensor";
    case K_LEG_UP: case K_AVG_TOP: return "Finding the top";
    case K_COAST_DN1: return "Coast test down (1 of 2)";
    case K_COAST_DN2: return "Coast test down (2 of 2)";
    case K_LEG_DOWN: case K_AVG_BOTTOM: return "Finding the bottom";
    case K_COAST_UP1: return "Coast test up (1 of 2)";
    case K_COAST_UP2: return "Coast test up (2 of 2)";
    default: return "Returning to the bottom";
    }
}

static uint8_t step_no(step_t k)
{
    static const uint8_t n[] = { 1, 1, 1, 2, 2, 3, 4, 5, 5, 6, 7, 8, 8 };
    return n[k];
}

static void set_phase(motion_cal_phase_t p, const char *text)
{
    C.phase = p;
    C.phase_since_us = esp_timer_get_time();
    strlcpy(C.text, text ? text : "", sizeof(C.text));
}

static uint32_t need_ms(void)
{
    return (uint32_t)CONFIG_DESK_CAL_MOTOR_S * 1000;
}

/* ---------------- sample bookkeeping ---------------- */

static void rec_add(uint32_t t, float f)
{
    s_rec_t[s_rec_i] = t;
    s_rec_f[s_rec_i] = f;
    s_rec_i = (s_rec_i + 1) % REC_N;
    if (s_rec_n < REC_N) s_rec_n++;
}

/* Span (max - min) of the filtered reading since t_from; returns the sample count. */
static int rec_span(uint32_t t_from, float *span)
{
    float lo = INFINITY, hi = -INFINITY;
    int n = 0;
    for (int i = 0; i < s_rec_n; i++) {
        if (s_rec_t[i] >= t_from) {
            lo = fminf(lo, s_rec_f[i]);
            hi = fmaxf(hi, s_rec_f[i]);
            n++;
        }
    }
    *span = n ? hi - lo : 0;
    return n;
}

static void begin_sub(sub_t sub)
{
    C.sub = sub;
    C.sub_us = esp_timer_get_time();
}

static void start_run(motion_dir_t dir, uint32_t max_ms, bool stall_check)
{
    begin_sub(SUB_RUN);
    C.run_dir = dir;
    C.run_on_us = 0;
    C.run_max_ms = max_ms;
    C.run_stall_check = stall_check;
    s_run_n = 0;
}

static void start_avg(void)
{
    begin_sub(SUB_AVG);
    C.avg_sum = 0;
    C.avg_n = 0;
    C.avg_want = AVG_N;
}

static void start_settle(void)
{
    begin_sub(SUB_SETTLE);
    C.rel_f = C.last_f;
    C.settle_avg = false;
    C.avg_sum = 0;
    C.avg_n = 0;
    C.avg_want = SETTLE_AVG_N;
}

static void goto_step(step_t k)
{
    C.k = k;
    C.step = step_no(k);
    strlcpy(C.text, step_text(k), sizeof(C.text));
    ESP_LOGI(TAG, "step %u/%d: %s", C.step, STEPS, C.text);
}

static void fail(const char *why)
{
    ESP_LOGW(TAG, "calibration failed: %s", why);
    set_phase(MOTION_CAL_FAILED, why);
}

/* Speed from a finished run: least squares over the steady part. */
static void run_speed(bool stalled, uint32_t end_ms)
{
    uint32_t from = (uint32_t)(C.run_on_us / 1000) + SPEED_SKIP_MS;
    uint32_t to = end_ms - (stalled ? SPEED_STALL_TAIL_MS : 0);
    double n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
    uint32_t first = 0, last = 0;
    for (int i = 0; i < s_run_n; i++) {
        if (s_run_t[i] < from || s_run_t[i] > to) continue;
        double x = (s_run_t[i] - from) / 1000.0, y = s_run_f[i];
        if (n == 0) first = s_run_t[i];
        last = s_run_t[i];
        n++; sx += x; sy += y; sxx += x * x; sxy += x * y;
    }
    uint32_t win = last - first;
    if (n < 8 || win < SPEED_MIN_WINDOW_MS) return;
    double den = n * sxx - sx * sx;
    if (den <= 0) return;
    float slope = fabsf((float)((n * sxy - sx * sy) / den));   /* mm per s */
    int d = C.run_dir == MOTION_DIR_UP ? 0 : 1;
    if (win > C.best_win_ms[d]) {
        C.best_win_ms[d] = win;
        if (d == 0) C.r.speed_up = slope;
        else C.r.speed_down = slope;
        ESP_LOGI(TAG, "speed %s %.1f mm/s over %.1f s", d ? "down" : "up", slope, win / 1000.0f);
    }
}

/* ---------------- step transitions ---------------- */

static void finish_measurements(void)
{
    float travel = C.r.sign * (C.r.raw_max - C.r.raw_min);
    if (travel < MIN_TRAVEL_MM) {
        fail("Travel too short: the sensor may not be seeing the target.");
        return;
    }
    if (C.best_win_ms[0] == 0 || C.best_win_ms[1] == 0) {
        fail("Could not measure the desk speed.");
        return;
    }
    C.r.coast_up = C.coast_n[0] ? C.coast_sum[0] / C.coast_n[0] : 0;
    C.r.coast_down = C.coast_n[1] ? C.coast_sum[1] / C.coast_n[1] : 0;
    ESP_LOGI(TAG, "measured: raw %.1f..%.1f (sign %+d, travel %.0f mm), speed up %.1f down %.1f mm/s, "
             "coast up %.1f down %.1f mm, %lu s", C.r.raw_min, C.r.raw_max, C.r.sign, travel, C.r.speed_up,
             C.r.speed_down, C.r.coast_up, C.r.coast_down,
             (unsigned long)((esp_timer_get_time() - C.t0_us) / 1000000));
    set_phase(MOTION_CAL_ENTER_HEIGHT, "Measure the desktop height from the floor now (the desk is at its "
                                       "lowest) and enter it in mm.");
}

static void after_avg(float value)
{
    switch (C.k) {
    case K_REST0:
        C.start_raw = value;
        ESP_LOGI(TAG, "start reading %.1f mm", value);
        goto_step(K_PROBE_UP);
        start_run(MOTION_DIR_UP, PROBE_MAX_MS, true);
        break;
    case K_AVG_TOP:
        C.r.raw_max = value;
        ESP_LOGI(TAG, "top: raw %.1f mm", value);
        goto_step(K_COAST_DN1);
        start_run(MOTION_DIR_DOWN, COAST_RUN_MS, true);
        break;
    case K_AVG_BOTTOM:
        C.r.raw_min = value;
        ESP_LOGI(TAG, "bottom: raw %.1f mm", value);
        goto_step(K_COAST_UP1);
        start_run(MOTION_DIR_UP, COAST_RUN_MS, true);
        break;
    case K_AVG_BOTTOM2:
        ESP_LOGI(TAG, "bottom again: raw %.1f mm (first %.1f)", value, C.r.raw_min);
        if (fabsf(value - C.r.raw_min) > REPEAT_TOL_MM) {
            fail("The bottom reading did not repeat. Check the sensor mounting.");
            return;
        }
        C.r.raw_min = (C.r.raw_min + value) / 2;
        finish_measurements();
        break;
    default:
        break;
    }
}

static void after_settle(float final)
{
    int d = C.run_dir == MOTION_DIR_UP ? 0 : 1;
    /* Height change after release, positive in the direction of travel. */
    float moved = C.r.sign * (final - C.rel_f) * (d == 0 ? 1 : -1);
    if (moved < 0) moved = 0;
    C.coast_sum[d] += moved;
    C.coast_n[d]++;
    ESP_LOGI(TAG, "coast %s: %.1f mm (release %.1f, settled %.1f)", d ? "down" : "up", moved, C.rel_f, final);
    switch (C.k) {
    case K_PROBE_DOWN:   /* probe from the top doubled as coast test 1 */
    case K_COAST_DN1:
        goto_step(K_COAST_DN2);
        start_run(MOTION_DIR_DOWN, COAST_RUN_MS, true);
        break;
    case K_COAST_DN2:
        goto_step(K_LEG_DOWN);
        C.leg_on_ms = 0;
        start_run(MOTION_DIR_DOWN, SEG_MAX_MS, true);
        break;
    case K_COAST_UP1:
        goto_step(K_COAST_UP2);
        start_run(MOTION_DIR_UP, COAST_RUN_MS, true);
        break;
    case K_COAST_UP2:
        goto_step(K_LEG_DOWN2);
        C.leg_on_ms = 0;
        start_run(MOTION_DIR_DOWN, SEG_MAX_MS, true);
        break;
    default:
        break;
    }
}

/* A run ended (time up or stalled). Returns the next sub-step already started. */
static void after_run(bool stalled, uint32_t on_ms)
{
    uint32_t end = ms_now();
    run_speed(stalled, end);
    float moved = C.last_f - C.run_start_f;

    switch (C.k) {
    case K_PROBE_UP:
        /* No 20 mm change within 3 s (or no change at all): maybe already at the top. */
        ESP_LOGI(TAG, "probe up: moved %.1f mm in %lu ms, trying down", moved, (unsigned long)on_ms);
        goto_step(K_PROBE_DOWN);
        start_run(MOTION_DIR_DOWN, PROBE_MAX_MS, true);
        break;
    case K_PROBE_DOWN:
        if (stalled || fabsf(moved) < PROBE_MM) {
            fail("Sensor not seeing the target: the reading did not change when the desk moved.");
            return;
        }
        C.r.sign = moved < 0 ? 1 : -1;
        C.r.raw_max = C.start_raw;   /* we started at the top */
        ESP_LOGI(TAG, "probe down: moved %.1f mm, sign %+d, started at the top (raw %.1f)", moved, C.r.sign,
                 C.start_raw);
        start_settle();
        break;
    case K_LEG_UP:
    case K_LEG_DOWN:
    case K_LEG_DOWN2:
        C.leg_on_ms += on_ms;
        if (stalled) {
            goto_step(C.k == K_LEG_UP ? K_AVG_TOP : C.k == K_LEG_DOWN ? K_AVG_BOTTOM : K_AVG_BOTTOM2);
            start_avg();
        } else if (C.leg_on_ms >= LEG_MAX_MS) {
            fail("End stop not found.");
        } else {
            begin_sub(SUB_PAUSE);   /* segment limit: pause, then carry on */
        }
        break;
    case K_COAST_DN1:
    case K_COAST_DN2:
    case K_COAST_UP1:
    case K_COAST_UP2:
        if (stalled) {
            fail("Hit an end stop during a coast test.");
            return;
        }
        start_settle();
        break;
    default:
        break;
    }
}

/* ---------------- tick ---------------- */

motion_dir_t calib_tick(int64_t now_us, motion_dir_t out)
{
    if (C.phase != MOTION_CAL_RUNNING) {
        return MOTION_DIR_NONE;
    }
    height_status_t hs;
    height_get(&hs);
    if (!hs.valid) {
        fail("Sensor reading lost.");
        return MOTION_DIR_NONE;
    }
    if (now_us - C.t0_us > (int64_t)TOTAL_MAX_MS * 1000) {
        fail("Took too long.");
        return MOTION_DIR_NONE;
    }

    /* New samples */
    height_sample_t buf[8];
    size_t n;
    bool run_on = C.sub == SUB_RUN && C.run_on_us != 0;
    while ((n = height_history(&C.seq, buf, 8)) > 0) {
        for (size_t i = 0; i < n; i++) {
            const height_sample_t *s = &buf[i];
            if (s->filt_dmm != HEIGHT_FILT_NONE) {
                C.last_f = s->filt_dmm / 10.0f;
                C.have_f = true;
                rec_add(s->t_ms, C.last_f);
                if (run_on && s_run_n < RUN_BUF) {
                    s_run_t[s_run_n] = s->t_ms;
                    s_run_f[s_run_n] = C.last_f;
                    s_run_n++;
                }
            }
            bool good = height_status_ok(s->status, s->raw_mm);
            bool collecting = (C.sub == SUB_AVG && now_us - C.sub_us >= REST_MS * 1000) ||
                              (C.sub == SUB_SETTLE && C.settle_avg);
            if (collecting && good && C.avg_n < C.avg_want) {
                C.avg_sum += s->raw_mm;
                C.avg_n++;
            }
        }
    }
    if (!C.have_f) {
        return MOTION_DIR_NONE;
    }
    uint32_t sub_ms = (uint32_t)((now_us - C.sub_us) / 1000);

    switch (C.sub) {
    case SUB_RUN: {
        if (C.run_on_us == 0) {
            if (out != C.run_dir) {
                return C.run_dir;   /* waiting for the output (direction-change gap) */
            }
            C.run_on_us = now_us;
            C.run_start_f = C.last_f;
        }
        uint32_t on_ms = (uint32_t)((now_us - C.run_on_us) / 1000);
        if (out != C.run_dir) {
            fail("Output dropped unexpectedly.");
            return MOTION_DIR_NONE;
        }
        if (C.k == K_PROBE_UP && on_ms >= PROBE_MIN_MS && fabsf(C.last_f - C.run_start_f) >= PROBE_MM) {
            float moved = C.last_f - C.run_start_f;
            C.r.sign = moved > 0 ? 1 : -1;
            ESP_LOGI(TAG, "probe up: moved %.1f mm in %lu ms, sign %+d", moved, (unsigned long)on_ms, C.r.sign);
            /* Same run carries on as the leg to the top. */
            C.k = K_LEG_UP;
            goto_step(K_LEG_UP);
            C.leg_on_ms = 0;
            C.run_max_ms = SEG_MAX_MS;
        }
        bool stalled = false;
        if (C.run_stall_check && on_ms >= STALL_AFTER_MS) {
            float span;
            if (rec_span(ms_now() - STALL_WINDOW_MS, &span) >= 3 && span < STALL_MM) {
                stalled = true;
            }
        }
        if (stalled || on_ms >= C.run_max_ms) {
            if (stalled) {
                ESP_LOGI(TAG, "stall after %lu ms (reading %.1f)", (unsigned long)on_ms, C.last_f);
            }
            after_run(stalled, on_ms);
            return MOTION_DIR_NONE;
        }
        return C.run_dir;
    }
    case SUB_PAUSE:
        if (sub_ms >= SEG_PAUSE_MS) {
            start_run(C.run_dir, SEG_MAX_MS, true);
            return C.run_dir;
        }
        return MOTION_DIR_NONE;
    case SUB_AVG:
        if (C.avg_n >= C.avg_want) {
            after_avg(C.avg_sum / C.avg_n);
        } else if (sub_ms > REST_MS + AVG_TIMEOUT_MS) {
            fail("Not enough good readings at the end stop.");
        }
        return MOTION_DIR_NONE;
    case SUB_SETTLE:
        if (!C.settle_avg && sub_ms >= SETTLE_MIN_MS) {
            float span;
            if ((rec_span(ms_now() - SETTLE_WINDOW_MS, &span) >= 3 && span < STALL_MM) || sub_ms >= SETTLE_MAX_MS) {
                C.settle_avg = true;
            }
        }
        if (C.settle_avg && C.avg_n >= C.avg_want) {
            after_settle(C.avg_sum / C.avg_n);
        } else if (sub_ms > SETTLE_MAX_MS + AVG_TIMEOUT_MS) {
            fail("The reading did not settle after a coast test.");
        }
        return MOTION_DIR_NONE;
    }
    return MOTION_DIR_NONE;
}

/* ---------------- commands ---------------- */

static bool result_phase(void)
{
    return C.phase == MOTION_CAL_SAVED || C.phase == MOTION_CAL_FAILED || C.phase == MOTION_CAL_CANCELLED ||
           C.phase == MOTION_CAL_NEED_BUDGET;
}

static void enter_prompt_or_wait(uint32_t duty_rem)
{
    if (duty_rem < need_ms()) {
        char t[96];
        snprintf(t, sizeof(t), "Not enough motor time left: calibration needs %d s.", CONFIG_DESK_CAL_MOTOR_S);
        set_phase(MOTION_CAL_NEED_BUDGET, t);
    } else {
        set_phase(MOTION_CAL_PROMPT, "Clear the area under and around the desk, then press Start.");
    }
}

const char *calib_cmd(motion_cal_cmd_t cmd, bool idle, uint32_t duty_rem)
{
    switch (cmd) {
    case MOTION_CAL_BEGIN:
        if (C.phase != MOTION_CAL_NONE && !result_phase()) return "Calibration already open";
        if (!idle) return "Desk is busy";
        memset(&C.r, 0, sizeof(C.r));
        enter_prompt_or_wait(duty_rem);
        ESP_LOGI(TAG, "calibration opened (%s)", C.phase == MOTION_CAL_PROMPT ? "prompt" : "waiting for budget");
        return NULL;
    case MOTION_CAL_START:
        if (C.phase != MOTION_CAL_PROMPT) return "Calibration not ready to start";
        if (!idle) return "Desk is busy";
        if (duty_rem < need_ms()) {
            enter_prompt_or_wait(duty_rem);
            return "Not enough motor time";
        }
        memset(&C.r, 0, sizeof(C.r));
        memset(C.coast_sum, 0, sizeof(C.coast_sum));
        memset(C.coast_n, 0, sizeof(C.coast_n));
        memset(C.best_win_ms, 0, sizeof(C.best_win_ms));
        C.have_f = false;
        s_rec_n = s_rec_i = 0;
        C.seq = 0xFFFFFFFF;   /* clamped to "now" by height_history */
        height_sample_t dummy;
        height_history(&C.seq, &dummy, 0);
        C.t0_us = esp_timer_get_time();
        set_phase(MOTION_CAL_RUNNING, "");
        goto_step(K_REST0);
        start_avg();
        return NULL;
    case MOTION_CAL_CANCEL:
        if (C.phase == MOTION_CAL_RUNNING) {
            calib_abort("Calibration cancelled. The old calibration is kept.", true);
        } else if (C.phase != MOTION_CAL_NONE) {
            ESP_LOGI(TAG, "calibration closed");
            set_phase(MOTION_CAL_NONE, NULL);
        }
        return NULL;
    case MOTION_CAL_SAVE:
        if (C.phase != MOTION_CAL_SUMMARY) return "Nothing to save";
        C.saved_pending = true;
        set_phase(MOTION_CAL_SAVED, "Calibration saved.");
        return NULL;
    case MOTION_CAL_DISCARD:
        if (C.phase != MOTION_CAL_SUMMARY && C.phase != MOTION_CAL_ENTER_HEIGHT) return "Nothing to discard";
        ESP_LOGI(TAG, "calibration discarded");
        set_phase(MOTION_CAL_NONE, NULL);
        return NULL;
    case MOTION_CAL_CLOSE:
        if (result_phase()) set_phase(MOTION_CAL_NONE, NULL);
        return NULL;
    }
    return "Unknown calibration command";
}

const char *calib_set_height(int32_t mm)
{
    if (C.phase != MOTION_CAL_ENTER_HEIGHT) return "Not waiting for a height";
    if (mm < 300 || mm > 1500) return "Height must be 300-1500 mm";
    C.r.h_min_mm = mm;
    C.r.scale = s_scale;
    C.r.version = MOTION_CAL_VERSION;
    C.r.valid = true;
    time_t now = time(NULL);
    C.r.timestamp = now > 1700000000 ? (int64_t)now : 0;
    const char *err = motion_cal_check(&C.r);
    if (err) {
        fail(err);
        return NULL;
    }
    float travel = C.r.sign * (C.r.raw_max - C.r.raw_min);
    char t[96];
    snprintf(t, sizeof(t), "Range %ld-%ld mm", (long)mm, (long)lroundf(mm + C.r.scale * travel));
    set_phase(MOTION_CAL_SUMMARY, t);
    ESP_LOGI(TAG, "bottom height %ld mm: %s", (long)mm, t);
    return NULL;
}

void calib_abort(const char *why, bool cancelled)
{
    if (C.phase != MOTION_CAL_RUNNING) {
        return;
    }
    ESP_LOGW(TAG, "calibration stopped: %s", why);
    set_phase(cancelled ? MOTION_CAL_CANCELLED : MOTION_CAL_FAILED, why);
}

void calib_idle_tick(int64_t now_us, uint32_t duty_wait_s)
{
    (void)duty_wait_s;
    int64_t age = now_us - C.phase_since_us;
    switch (C.phase) {
    case MOTION_CAL_NEED_BUDGET:
        if (duty_remaining_ms() >= need_ms()) {
            set_phase(MOTION_CAL_PROMPT, "Clear the area under and around the desk, then press Start.");
            ESP_LOGI(TAG, "enough motor time now: ready to start");
        } else if (age > ENTRY_TIMEOUT_US) {
            set_phase(MOTION_CAL_NONE, NULL);
        }
        break;
    case MOTION_CAL_PROMPT:
    case MOTION_CAL_SAVED:
    case MOTION_CAL_FAILED:
    case MOTION_CAL_CANCELLED:
        if (age > PROMPT_TIMEOUT_US) set_phase(MOTION_CAL_NONE, NULL);
        break;
    case MOTION_CAL_ENTER_HEIGHT:
    case MOTION_CAL_SUMMARY:
        if (age > ENTRY_TIMEOUT_US) {
            ESP_LOGW(TAG, "calibration entry timed out: discarded");
            set_phase(MOTION_CAL_NONE, NULL);
        }
        break;
    default:
        break;
    }
}

bool calib_running(void)
{
    return C.phase == MOTION_CAL_RUNNING;
}

bool calib_blocks_moves(void)
{
    return C.phase == MOTION_CAL_RUNNING || C.phase == MOTION_CAL_ENTER_HEIGHT || C.phase == MOTION_CAL_SUMMARY;
}

void calib_get_status(motion_cal_status_t *out)
{
    memset(out, 0, sizeof(*out));
    out->phase = C.phase;
    out->step = C.phase == MOTION_CAL_RUNNING ? C.step : 0;
    out->steps = STEPS;
    strlcpy(out->text, C.text, sizeof(out->text));
    out->need_s = CONFIG_DESK_CAL_MOTOR_S;
    out->wait_s = C.phase == MOTION_CAL_NEED_BUDGET ? duty_wait_s_for(need_ms()) : 0;
    out->elapsed_s = C.phase == MOTION_CAL_RUNNING ? (uint32_t)((esp_timer_get_time() - C.t0_us) / 1000000) : 0;
    out->result = C.r;
}

bool calib_take_saved(motion_cal_t *out)
{
    if (!C.saved_pending) {
        return false;
    }
    C.saved_pending = false;
    *out = C.r;
    return true;
}
