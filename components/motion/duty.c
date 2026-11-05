#include "duty.h"

#include <string.h>
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_rtc_time.h"
#include "sdkconfig.h"

static const char *TAG = "duty";

#define N_BUCKETS   120
#define BUCKET_MS   ((uint64_t)CONFIG_DESK_DUTY_WINDOW_S * 1000 / N_BUCKETS)
#define MAX_ON_MS   ((uint32_t)CONFIG_DESK_DUTY_MAX_ON_S * 1000)
#define MAGIC       0x44555432u  /* "DUT2": bump when the layout changes */

/* Survives soft resets, panics and watchdog resets; lost on power-off. */
typedef struct {
    uint32_t magic;
    uint32_t window_s, max_on_s;   /* config the ledger was built with */
    int64_t  newest;               /* absolute bucket number of bucket_ms[newest % N] */
    uint32_t bucket_ms[N_BUCKETS];
    uint32_t check;
} ledger_t;

static RTC_NOINIT_ATTR ledger_t s_l;

static uint32_t checksum(void)
{
    uint32_t c = 0x9E3779B9u;
    const uint8_t *p = (const uint8_t *)&s_l;
    for (size_t i = 0; i < offsetof(ledger_t, check); i++) {
        c = (c ^ p[i]) * 16777619u;
    }
    return c;
}

static int64_t now_bucket(void)
{
    /* RTC time keeps counting across soft resets (not across power cycles). */
    return (int64_t)(esp_rtc_get_time_us() / 1000 / BUCKET_MS);
}

/* Advance the ring to the current bucket, zeroing expired buckets. */
static void roll(void)
{
    int64_t nb = now_bucket();
    if (nb < s_l.newest) {
        /* Time base went backwards (RTC reset). Be conservative: keep the
         * recorded on-time and treat no time as having passed. */
        s_l.newest = nb;
        return;
    }
    int64_t steps = nb - s_l.newest;
    if (steps >= N_BUCKETS) {
        memset(s_l.bucket_ms, 0, sizeof(s_l.bucket_ms));
    } else {
        for (int64_t i = 1; i <= steps; i++) {
            s_l.bucket_ms[(s_l.newest + i) % N_BUCKETS] = 0;
        }
    }
    s_l.newest = nb;
}

void duty_init(void)
{
    if (s_l.magic == MAGIC && s_l.check == checksum() &&
        s_l.window_s == CONFIG_DESK_DUTY_WINDOW_S && s_l.max_on_s == CONFIG_DESK_DUTY_MAX_ON_S) {
        roll();
        s_l.check = checksum();
        ESP_LOGI(TAG, "ledger restored from RTC memory: %lu ms used in window",
                 (unsigned long)duty_used_ms());
    } else {
        memset(&s_l, 0, sizeof(s_l));
        s_l.magic = MAGIC;
        s_l.window_s = CONFIG_DESK_DUTY_WINDOW_S;
        s_l.max_on_s = CONFIG_DESK_DUTY_MAX_ON_S;
        s_l.newest = now_bucket();
        s_l.check = checksum();
        ESP_LOGI(TAG, "ledger reset (cold boot)");
    }
    ESP_LOGI(TAG, "budget %d s per %d s window", CONFIG_DESK_DUTY_MAX_ON_S, CONFIG_DESK_DUTY_WINDOW_S);
}

void duty_add_ms(uint32_t ms)
{
    roll();
    s_l.bucket_ms[s_l.newest % N_BUCKETS] += ms;
    s_l.check = checksum();
}

uint32_t duty_used_ms(void)
{
    roll();
    s_l.check = checksum();
    uint32_t sum = 0;
    for (int i = 0; i < N_BUCKETS; i++) {
        sum += s_l.bucket_ms[i];
    }
    return sum;
}

uint32_t duty_remaining_ms(void)
{
    uint32_t used = duty_used_ms();
    return used >= MAX_ON_MS ? 0 : MAX_ON_MS - used;
}

uint32_t duty_wait_s_for(uint32_t want_ms)
{
    uint32_t rem = duty_remaining_ms();
    if (rem >= want_ms) {
        return 0;
    }
    uint32_t need = want_ms - rem, freed = 0;
    uint64_t now_ms = esp_rtc_get_time_us() / 1000;
    /* Oldest bucket is newest+1; bucket b expires when the window passes its end. */
    for (int64_t b = s_l.newest - N_BUCKETS + 1; b <= s_l.newest; b++) {
        freed += s_l.bucket_ms[((b % N_BUCKETS) + N_BUCKETS) % N_BUCKETS];
        if (freed >= need) {
            uint64_t expiry_ms = (uint64_t)(b + N_BUCKETS + 1) * BUCKET_MS;
            return expiry_ms > now_ms ? (uint32_t)((expiry_ms - now_ms + 999) / 1000) : 0;
        }
    }
    return CONFIG_DESK_DUTY_WINDOW_S;
}

void duty_clear(void)
{
    memset(s_l.bucket_ms, 0, sizeof(s_l.bucket_ms));
    s_l.check = checksum();
}
