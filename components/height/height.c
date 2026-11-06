/*
 * Height sensor task. Polls the VL53L0X in continuous mode, filters good
 * samples (median of 5, then EMA) and records every sample in a 60 s ring.
 * A missing or failing sensor is retried with back-off; it never blocks
 * anything else (touch shares the bus, guarded by board_i2c_lock).
 */
#include "height.h"

#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "board.h"
#include "vl53l0x.h"
#include "sdkconfig.h"

static const char *TAG = "height";

#define POLL_MS             10
#define MEDIAN_N            5
#define STATS_WINDOW_MS     2000
#define ERRORS_TO_OFFLINE   3
#define BACKOFF_MIN_MS      500
#define BACKOFF_MAX_MS      5000
#define MAX_RATE_HZ         25
#define HISTORY_N           (HEIGHT_HISTORY_S * MAX_RATE_HZ)

static vl53l0x_handle_t s_dev;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static height_status_t s_pub;

static volatile uint8_t s_want_profile, s_want_ema;

static height_sample_t *s_hist;
static uint32_t s_head;      /* sequence number of the next sample */

/* filter state (task only) */
static uint16_t s_med[MEDIAN_N];
static int s_med_n, s_med_i;
static float s_ema;
static bool s_ema_init;
static int64_t s_last_good_us;

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

static uint16_t budget_ms(uint8_t profile)
{
    return profile == HEIGHT_PROFILE_ACCURATE ? CONFIG_DESK_HEIGHT_BUDGET_ACCURATE_MS : CONFIG_DESK_HEIGHT_BUDGET_MS;
}

bool height_status_ok(uint8_t st, uint16_t raw)
{
    return st == VL53L0X_STATUS_OK && raw > 0 && raw < 8190;
}

static void bus_lock(bool take)
{
    if (take) {
        while (!board_i2c_lock(1000)) {
        }
    } else {
        board_i2c_unlock();
    }
}

static void filter_reset(void)
{
    s_med_n = s_med_i = 0;
    s_ema_init = false;
}

static float median(void)
{
    uint16_t v[MEDIAN_N];
    memcpy(v, s_med, sizeof(v));
    for (int i = 1; i < s_med_n; i++) {
        for (int j = i; j > 0 && v[j - 1] > v[j]; j--) {
            uint16_t t = v[j]; v[j] = v[j - 1]; v[j - 1] = t;
        }
    }
    return s_med_n & 1 ? v[s_med_n / 2] : (v[s_med_n / 2 - 1] + v[s_med_n / 2]) / 2.0f;
}

static void history_add(const height_sample_t *s)
{
    portENTER_CRITICAL(&s_lock);
    s_hist[s_head % HISTORY_N] = *s;
    s_head++;
    portEXIT_CRITICAL(&s_lock);
}

size_t height_history(uint32_t *seq, height_sample_t *buf, size_t max)
{
    size_t n = 0;
    portENTER_CRITICAL(&s_lock);
    uint32_t first = s_head > HISTORY_N ? s_head - HISTORY_N : 0;
    uint32_t i = *seq < first ? first : *seq > s_head ? s_head : *seq;
    for (; i < s_head && n < max; i++) {
        buf[n++] = s_hist[i % HISTORY_N];
    }
    portEXIT_CRITICAL(&s_lock);
    *seq = i;
    return n;
}

/* Rate, standard deviations and span over the last 2 s. Called after each sample. */
static void update_stats(height_status_t *st, uint32_t now)
{
    double sr = 0, sr2 = 0, sf = 0, sf2 = 0;
    int nr = 0, nf = 0, total = 0;
    float lo = INFINITY, hi = -INFINITY;
    portENTER_CRITICAL(&s_lock);
    uint32_t first = s_head > HISTORY_N ? s_head - HISTORY_N : 0;
    for (uint32_t i = s_head; i > first; i--) {
        const height_sample_t *h = &s_hist[(i - 1) % HISTORY_N];
        if (now - h->t_ms > STATS_WINDOW_MS) {
            break;
        }
        total++;
        if (height_status_ok(h->status, h->raw_mm)) {
            sr += h->raw_mm; sr2 += (double)h->raw_mm * h->raw_mm; nr++;
        }
        if (h->filt_dmm != HEIGHT_FILT_NONE) {
            float f = h->filt_dmm / 10.0f;
            sf += f; sf2 += (double)f * f; nf++;
            lo = fminf(lo, f); hi = fmaxf(hi, f);
        }
    }
    portEXIT_CRITICAL(&s_lock);
    st->rate_hz = total * 1000.0f / STATS_WINDOW_MS;
    st->std_raw_mm = nr >= 2 ? sqrt(fmax(0, sr2 / nr - (sr / nr) * (sr / nr))) : NAN;
    st->std_filt_mm = nf >= 2 ? sqrt(fmax(0, sf2 / nf - (sf / nf) * (sf / nf))) : NAN;
    st->span_filt_mm = nf >= 2 ? hi - lo : NAN;
}

static void publish(const height_status_t *st)
{
    portENTER_CRITICAL(&s_lock);
    s_pub = *st;
    portEXIT_CRITICAL(&s_lock);
}

void height_get(height_status_t *out)
{
    portENTER_CRITICAL(&s_lock);
    *out = s_pub;
    portEXIT_CRITICAL(&s_lock);
}

void height_set_params(uint8_t profile, uint8_t ema_pct)
{
    s_want_profile = profile;
    s_want_ema = ema_pct;
}

static esp_err_t sensor_start(uint8_t profile)
{
    /* Probe first: a missing sensor gives a quiet NACK instead of a full failed init. */
    if (!board_i2c_lock(1000)) {
        return ESP_ERR_TIMEOUT;
    }
    esp_err_t e = i2c_master_probe(board_i2c_bus(), VL53L0X_ADDR, 20);
    board_i2c_unlock();
    if (e != ESP_OK) {
        return e;
    }
    if ((e = vl53l0x_init(s_dev)) != ESP_OK) return e;
    if ((e = vl53l0x_set_timing_budget_us(s_dev, budget_ms(profile) * 1000)) != ESP_OK) return e;
    return vl53l0x_start_continuous(s_dev);
}

static void height_task(void *arg)
{
    height_status_t st = { 0 };
    uint32_t backoff = BACKOFF_MIN_MS;
    int64_t next_try = 0, last_sample_ms = 0;
    int consecutive_err = 0;
    int64_t last_warn = -60000;

    for (;;) {
        int64_t now = now_ms();
        st.ema_pct = s_want_ema;

        if (!st.online) {
            st.retry_in_ms = now < next_try ? (uint32_t)(next_try - now) : 0;
            if (now >= next_try) {
                st.profile = s_want_profile;
                st.budget_ms = budget_ms(st.profile);
                esp_err_t e = sensor_start(st.profile);
                if (e == ESP_OK) {
                    st.online = true;
                    st.inits++;
                    consecutive_err = 0;
                    backoff = BACKOFF_MIN_MS;
                    last_sample_ms = now_ms();
                    ESP_LOGI(TAG, "sensor online (%u ms budget)", st.budget_ms);
                } else {
                    if (now - last_warn >= 60000) {   /* first failure, then once a minute */
                        ESP_LOGW(TAG, "sensor init failed (%s): retrying with back-off", esp_err_to_name(e));
                        last_warn = now;
                    }
                    if (e == ESP_ERR_TIMEOUT) {
                        board_i2c_recover();
                    }
                    next_try = now + backoff;
                    backoff = backoff * 2 > BACKOFF_MAX_MS ? BACKOFF_MAX_MS : backoff * 2;
                }
            }
        } else if (s_want_profile != st.profile) {
            st.profile = s_want_profile;
            st.budget_ms = budget_ms(st.profile);
            esp_err_t e = vl53l0x_stop_continuous(s_dev);
            if (e == ESP_OK) e = vl53l0x_set_timing_budget_us(s_dev, st.budget_ms * 1000);
            if (e == ESP_OK) e = vl53l0x_start_continuous(s_dev);
            ESP_LOGI(TAG, "ranging profile %s (%u ms): %s", st.profile ? "high accuracy" : "default",
                     st.budget_ms, esp_err_to_name(e));
            if (e != ESP_OK) {
                st.online = false;
            }
            filter_reset();
        } else {
            vl53l0x_sample_t smp;
            esp_err_t e = vl53l0x_read_sample(s_dev, &smp);
            if (e == ESP_OK) {
                consecutive_err = 0;
                last_sample_ms = now;
                bool good = height_status_ok(smp.status, smp.range_mm);
                if (good) {
                    s_med[s_med_i] = smp.range_mm;
                    s_med_i = (s_med_i + 1) % MEDIAN_N;
                    if (s_med_n < MEDIAN_N) s_med_n++;
                    float m = median();
                    float a = st.ema_pct / 100.0f;
                    s_ema = s_ema_init ? s_ema + a * (m - s_ema) : m;
                    s_ema_init = true;
                    s_last_good_us = esp_timer_get_time();
                }
                st.raw_mm = smp.range_mm;
                st.raw_ok = good;
                st.range_status = smp.status;
                st.signal_mcps = smp.signal_mcps;
                st.ambient_mcps = smp.ambient_mcps;
                st.samples++;
                bool valid = s_ema_init && esp_timer_get_time() - s_last_good_us < HEIGHT_INVALID_MS * 1000;
                const height_sample_t h = {
                    .t_ms = (uint32_t)now,
                    .raw_mm = smp.range_mm,
                    .filt_dmm = valid ? (uint16_t)lroundf(s_ema * 10) : HEIGHT_FILT_NONE,
                    .signal_x128 = (uint16_t)fminf(smp.signal_mcps * 128, 65535),
                    .ambient_x128 = (uint16_t)fminf(smp.ambient_mcps * 128, 65535),
                    .status = smp.status,
                };
                history_add(&h);
                update_stats(&st, (uint32_t)now);
            } else if (e != ESP_ERR_NOT_FINISHED) {
                st.errors++;
                if (++consecutive_err >= ERRORS_TO_OFFLINE) {
                    ESP_LOGW(TAG, "sensor I2C error (%s): offline", esp_err_to_name(e));
                    st.online = false;
                }
            } else if (now - last_sample_ms > 4 * st.budget_ms + 500) {
                ESP_LOGW(TAG, "sensor stopped producing samples: re-initialising");
                st.online = false;
            }
            if (!st.online) {
                if (e == ESP_ERR_TIMEOUT) {
                    board_i2c_recover();
                }
                next_try = now + BACKOFF_MIN_MS;
                backoff = BACKOFF_MIN_MS * 2;
                last_warn = now;   /* already logged why */
            }
        }

        if (!st.online) {
            st.raw_ok = false;
            st.rate_hz = 0;
            st.std_raw_mm = st.std_filt_mm = st.span_filt_mm = NAN;
            filter_reset();
        }
        if (s_ema_init && esp_timer_get_time() - s_last_good_us >= HEIGHT_INVALID_MS * 1000) {
            filter_reset();   /* start fresh once the reading is back */
        }
        st.valid = st.online && s_ema_init;
        st.filtered_mm = st.valid ? s_ema : 0;
        publish(&st);
        vTaskDelay(pdMS_TO_TICKS(POLL_MS));
    }
}

esp_err_t height_init(uint8_t profile, uint8_t ema_pct)
{
    s_want_profile = profile;
    s_want_ema = ema_pct;
    s_hist = heap_caps_calloc(HISTORY_N, sizeof(height_sample_t), MALLOC_CAP_INTERNAL);
    if (!s_hist) {
        return ESP_ERR_NO_MEM;
    }
    s_pub.std_raw_mm = s_pub.std_filt_mm = s_pub.span_filt_mm = NAN;
    const vl53l0x_config_t cfg = { .bus = board_i2c_bus(), .scl_hz = BOARD_I2C_HZ, .bus_lock = bus_lock };
    esp_err_t e = vl53l0x_new(&cfg, &s_dev);
    if (e != ESP_OK) {
        return e;
    }
    BaseType_t ok = xTaskCreate(height_task, "height", 3584, NULL, 5, NULL);
    return ok == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}
