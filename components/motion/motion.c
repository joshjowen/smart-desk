/*
 * Motion controller task. Owns the desk outputs and enforces the safety
 * invariants from the spec:
 *   1  outputs off at boot (opto_force_off), shutdown handler, panic hook
 *   2  never UP and DOWN together; 300 ms off before a direction change
 *   3  hold-to-run: jog stops 300 ms after the last heartbeat
 *   4  max single move time -> FAULT
 *   5  duty-cycle ledger (RTC memory) -> refuse seeks / COOLDOWN
 *   7  STOP from any source wins
 *   9  task watchdog on this task
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

static const char *TAG = "motion";

#define TICK_MS              20
#define HEARTBEAT_TIMEOUT_US (300 * 1000)
#define DIR_CHANGE_OFF_US    (300 * 1000)
#define SETTLE_US            (300 * 1000)
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
static int64_t s_settle_until_us;
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
/* Real desk through the optos. No height sensor until stage 3 / calibration in
 * stage 4: height is unknown and only hold-to-run jogging is possible. The range
 * is only used to sanity-check preset values entered in the settings. */
static const int32_t s_min_mm = 500;
static const int32_t s_max_mm = 1400;
static const bool s_calibrated = false;

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
    return false;
}
#endif

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

static const int64_t s_max_move_us = (int64_t)CONFIG_DESK_MAX_MOVE_S_UNCAL * 1000000;

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

static void reject(const motion_req_t *r, const char *why)
{
    strlcpy(s_last_error, why, sizeof(s_last_error));
    s_last_error_seq++;
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

static void start_seek(const motion_req_t *r, int32_t target)
{
    int32_t h;
    if (s_state == MOTION_FAULT) { reject(r, "Fault: acknowledge first"); return; }
    if (s_state == MOTION_COOLDOWN) { reject(r, "Cooling down"); return; }
    if (!s_calibrated) { reject(r, "Needs calibration"); return; }
    if (!height_read(&h)) { reject(r, "Height sensor invalid"); return; }
    if (target < s_min_mm || target > s_max_mm) { reject(r, "Target out of range"); return; }
    if (duty_remaining_ms() < SEEK_MIN_BUDGET_MS) { reject(r, "Duty budget low"); return; }
    if (abs(target - h) <= CONFIG_DESK_SEEK_TOLERANCE_MM) {
        ESP_LOGI(TAG, "seek to %ld: already there (%ld)", (long)target, (long)h);
        if (moving_state()) {
            apply_output(MOTION_DIR_NONE, "seek: at target");
            set_state(MOTION_IDLE, "at target");
        }
        return;
    }
    s_src = r->source;
    s_target_mm = target;
    set_state(MOTION_SEEK, "seek request");
    ESP_LOGI(TAG, "seek %ld -> %ld mm src=%s", (long)h, (long)target, motion_source_name(r->source));
}

static void handle(const motion_req_t *r)
{
    int64_t now = esp_timer_get_time();
    switch ((int)r->type) {
    case MOTION_REQ_STOP:
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
        if (s_state == MOTION_CALIBRATING) { reject(r, "Calibrating"); break; }
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

    case MOTION_REQ_CALIBRATE:
        reject(r, "Calibration arrives in stage 4");
        break;

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
        apply_output(MOTION_DIR_NONE, "duty budget used");
        set_state(MOTION_COOLDOWN, "duty budget used");
    } else if (s_state == MOTION_COOLDOWN && rem >= COOLDOWN_EXIT_MS) {
        set_state(MOTION_IDLE, "cooldown over");
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
            apply_output(MOTION_DIR_NONE, "height invalid");
            set_state(MOTION_IDLE, "height invalid");
            break;
        }
        motion_dir_t d = s_target_mm > h ? MOTION_DIR_UP : MOTION_DIR_DOWN;
        /* Simulator has no coast: release once the target is reached or crossed. */
        bool reached = (s_out == MOTION_DIR_UP && h >= s_target_mm) ||
                       (s_out == MOTION_DIR_DOWN && h <= s_target_mm) ||
                       abs(s_target_mm - h) <= 1;
        if (reached) {
            apply_output(MOTION_DIR_NONE, "seek: release");
            s_settle_until_us = now + SETTLE_US;
            set_state(MOTION_SETTLE, "target reached");
        } else {
            want = d;
        }
        break;
    }
    case MOTION_SETTLE:
        if (now >= s_settle_until_us) {
            ESP_LOGI(TAG, "seek done: %ld mm (target %ld, error %+ld)", (long)h, (long)s_target_mm,
                     (long)(h - s_target_mm));
            set_state(MOTION_IDLE, "settled");
        }
        break;
    default:
        break;
    }
    apply_output(want, s_state == MOTION_JOG ? "jog" : s_state == MOTION_SEEK ? "seek" : "idle");

    motion_status_t st = {
        .state = s_state,
        .output = s_out,
        .source = s_src,
        .height_valid = h_ok,
        .height_mm = h,
        .target_mm = (s_state == MOTION_SEEK || s_state == MOTION_SETTLE) ? s_target_mm : 0,
        .presets_available = s_calibrated,
        .min_mm = s_min_mm,
        .max_mm = s_max_mm,
        .duty_remaining_s = rem / 1000,
        .cooldown_remaining_s = s_state == MOTION_COOLDOWN ? duty_wait_s_for(COOLDOWN_EXIT_MS) : 0,
        .last_error_seq = s_last_error_seq,
    };
    portENTER_CRITICAL(&s_preset_lock);
    memcpy(st.preset_mm, s_preset_mm, sizeof(st.preset_mm));
    portEXIT_CRITICAL(&s_preset_lock);
    strlcpy(st.last_fault, s_last_fault, sizeof(st.last_fault));
    strlcpy(st.fault, s_state == MOTION_FAULT ? s_fault : "", sizeof(st.fault));
    strlcpy(st.last_error, s_last_error, sizeof(st.last_error));
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
    ESP_LOGW(TAG, "REAL desk: hold-to-run only, max move %d s, no height sensor", CONFIG_DESK_MAX_MOVE_S_UNCAL);
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
    portEXIT_CRITICAL(&s_lock);
    return s == MOTION_SEEK || s == MOTION_SETTLE || s == MOTION_CALIBRATING;
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
