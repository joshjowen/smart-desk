/*
 * VL53L0X driver: C port of Pololu's VL53L0X library (MIT licence,
 * https://github.com/pololu/vl53l0x-arduino), which is itself derived from
 * ST's VL53L0X API. Register sequences are kept in the same order as Pololu's
 * so they can be compared line by line.
 */
#include "vl53l0x.h"

#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"

static const char *TAG = "vl53l0x";

/* Registers */
#define SYSRANGE_START                              0x00
#define SYSTEM_SEQUENCE_CONFIG                      0x01
#define SYSTEM_INTERMEASUREMENT_PERIOD              0x04
#define SYSTEM_INTERRUPT_CONFIG_GPIO                0x0A
#define SYSTEM_INTERRUPT_CLEAR                      0x0B
#define RESULT_INTERRUPT_STATUS                     0x13
#define RESULT_RANGE_STATUS                         0x14
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44
#define MSRC_CONFIG_TIMEOUT_MACROP                  0x46
#define PRE_RANGE_CONFIG_VCSEL_PERIOD               0x50
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          0x51
#define MSRC_CONFIG_CONTROL                         0x60
#define FINAL_RANGE_CONFIG_VCSEL_PERIOD             0x70
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        0x71
#define GPIO_HV_MUX_ACTIVE_HIGH                     0x84
#define VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           0x89
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0            0xB0
#define GLOBAL_CONFIG_REF_EN_START_SELECT           0xB6
#define DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         0x4E
#define DYNAMIC_SPAD_REF_EN_START_OFFSET            0x4F
#define IDENTIFICATION_MODEL_ID                     0xC0

#define IO_TIMEOUT_MS   50
#define WAIT_TIMEOUT_MS 500

struct vl53l0x {
    i2c_master_dev_handle_t dev;
    void (*lock)(bool take);
    uint8_t stop_variable;
    uint32_t budget_us;
};

typedef struct {
    bool tcc, msrc, dss, pre_range, final_range;
} seq_enables_t;

typedef struct {
    uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
    uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
    uint32_t msrc_dss_tcc_us, pre_range_us, final_range_us;
} seq_timeouts_t;

/* ---------------- register access ---------------- */

static esp_err_t xfer(vl53l0x_handle_t d, const uint8_t *w, size_t wn, uint8_t *r, size_t rn)
{
    if (d->lock) d->lock(true);
    esp_err_t e = r ? i2c_master_transmit_receive(d->dev, w, wn, r, rn, IO_TIMEOUT_MS)
                    : i2c_master_transmit(d->dev, w, wn, IO_TIMEOUT_MS);
    if (d->lock) d->lock(false);
    return e;
}

/* Init does dozens of accesses; the first error is kept and the rest skipped. */
#define CHK(expr) do { esp_err_t _e = (expr); if (_e != ESP_OK) return _e; } while (0)

static esp_err_t wr8(vl53l0x_handle_t d, uint8_t reg, uint8_t v)
{
    uint8_t b[2] = { reg, v };
    return xfer(d, b, 2, NULL, 0);
}

static esp_err_t wr16(vl53l0x_handle_t d, uint8_t reg, uint16_t v)
{
    uint8_t b[3] = { reg, v >> 8, v & 0xFF };
    return xfer(d, b, 3, NULL, 0);
}


static esp_err_t wr_multi(vl53l0x_handle_t d, uint8_t reg, const uint8_t *src, size_t n)
{
    uint8_t b[8];
    b[0] = reg;
    memcpy(&b[1], src, n);
    return xfer(d, b, n + 1, NULL, 0);
}

static esp_err_t rd_multi(vl53l0x_handle_t d, uint8_t reg, uint8_t *dst, size_t n)
{
    return xfer(d, &reg, 1, dst, n);
}

static esp_err_t rd8(vl53l0x_handle_t d, uint8_t reg, uint8_t *v)
{
    return rd_multi(d, reg, v, 1);
}

static esp_err_t rd16(vl53l0x_handle_t d, uint8_t reg, uint16_t *v)
{
    uint8_t b[2];
    CHK(rd_multi(d, reg, b, 2));
    *v = ((uint16_t)b[0] << 8) | b[1];
    return ESP_OK;
}

static esp_err_t set_bits(vl53l0x_handle_t d, uint8_t reg, uint8_t set, uint8_t clear)
{
    uint8_t v;
    CHK(rd8(d, reg, &v));
    return wr8(d, reg, (v | set) & ~clear);
}

/* Sequence of {reg, value} writes. */
typedef struct { uint8_t reg, val; } regval_t;

static esp_err_t wr_list(vl53l0x_handle_t d, const regval_t *l, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        CHK(wr8(d, l[i].reg, l[i].val));
    }
    return ESP_OK;
}

/* ---------------- timing helpers ---------------- */

#define decode_vcsel_period(v)   ((((uint16_t)(v)) + 1) << 1)
#define calc_macro_period(pclks) ((((uint32_t)2304 * (pclks) * 1655) + 500) / 1000)

static uint16_t decode_timeout(uint16_t v)
{
    return (uint16_t)((v & 0x00FF) << (uint16_t)((v & 0xFF00) >> 8)) + 1;
}

static uint16_t encode_timeout(uint32_t mclks)
{
    if (mclks == 0) {
        return 0;
    }
    uint32_t ls = mclks - 1;
    uint16_t ms = 0;
    while ((ls & 0xFFFFFF00) > 0) {
        ls >>= 1;
        ms++;
    }
    return (ms << 8) | (ls & 0xFF);
}

static uint32_t mclks_to_us(uint16_t mclks, uint16_t pclks)
{
    uint32_t macro_ns = calc_macro_period(pclks);
    return ((mclks * macro_ns) + 500) / 1000;
}

static uint32_t us_to_mclks(uint32_t us, uint16_t pclks)
{
    uint32_t macro_ns = calc_macro_period(pclks);
    return ((us * 1000) + (macro_ns / 2)) / macro_ns;
}

static esp_err_t get_seq_enables(vl53l0x_handle_t d, seq_enables_t *e)
{
    uint8_t c;
    CHK(rd8(d, SYSTEM_SEQUENCE_CONFIG, &c));
    e->tcc = (c >> 4) & 1;
    e->dss = (c >> 3) & 1;
    e->msrc = (c >> 2) & 1;
    e->pre_range = (c >> 6) & 1;
    e->final_range = (c >> 7) & 1;
    return ESP_OK;
}

static esp_err_t get_seq_timeouts(vl53l0x_handle_t d, const seq_enables_t *e, seq_timeouts_t *t)
{
    uint8_t v;
    uint16_t v16;
    CHK(rd8(d, PRE_RANGE_CONFIG_VCSEL_PERIOD, &v));
    t->pre_range_vcsel_period_pclks = decode_vcsel_period(v);
    CHK(rd8(d, MSRC_CONFIG_TIMEOUT_MACROP, &v));
    t->msrc_dss_tcc_mclks = v + 1;
    t->msrc_dss_tcc_us = mclks_to_us(t->msrc_dss_tcc_mclks, t->pre_range_vcsel_period_pclks);
    CHK(rd16(d, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, &v16));
    t->pre_range_mclks = decode_timeout(v16);
    t->pre_range_us = mclks_to_us(t->pre_range_mclks, t->pre_range_vcsel_period_pclks);
    CHK(rd8(d, FINAL_RANGE_CONFIG_VCSEL_PERIOD, &v));
    t->final_range_vcsel_period_pclks = decode_vcsel_period(v);
    CHK(rd16(d, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, &v16));
    t->final_range_mclks = decode_timeout(v16);
    if (e->pre_range) {
        t->final_range_mclks -= t->pre_range_mclks;
    }
    t->final_range_us = mclks_to_us(t->final_range_mclks, t->final_range_vcsel_period_pclks);
    return ESP_OK;
}

#define START_OVERHEAD       1910
#define END_OVERHEAD         960
#define MSRC_OVERHEAD        660
#define TCC_OVERHEAD         590
#define DSS_OVERHEAD         690
#define PRE_RANGE_OVERHEAD   660
#define FINAL_RANGE_OVERHEAD 550

static uint32_t fixed_budget(const seq_enables_t *e, const seq_timeouts_t *t)
{
    uint32_t us = START_OVERHEAD + END_OVERHEAD;
    if (e->tcc) us += t->msrc_dss_tcc_us + TCC_OVERHEAD;
    if (e->dss) us += 2 * (t->msrc_dss_tcc_us + DSS_OVERHEAD);
    else if (e->msrc) us += t->msrc_dss_tcc_us + MSRC_OVERHEAD;
    if (e->pre_range) us += t->pre_range_us + PRE_RANGE_OVERHEAD;
    return us;
}

static esp_err_t get_timing_budget(vl53l0x_handle_t d, uint32_t *out)
{
    seq_enables_t e;
    seq_timeouts_t t;
    CHK(get_seq_enables(d, &e));
    CHK(get_seq_timeouts(d, &e, &t));
    uint32_t us = fixed_budget(&e, &t);
    if (e.final_range) {
        us += t.final_range_us + FINAL_RANGE_OVERHEAD;
    }
    *out = us;
    return ESP_OK;
}

esp_err_t vl53l0x_set_timing_budget_us(vl53l0x_handle_t d, uint32_t budget_us)
{
    seq_enables_t e;
    seq_timeouts_t t;
    CHK(get_seq_enables(d, &e));
    CHK(get_seq_timeouts(d, &e, &t));
    if (!e.final_range) {
        return ESP_OK;
    }
    uint32_t used = fixed_budget(&e, &t) + FINAL_RANGE_OVERHEAD;
    if (used > budget_us) {
        return ESP_ERR_INVALID_ARG;
    }
    uint32_t mclks = us_to_mclks(budget_us - used, t.final_range_vcsel_period_pclks);
    if (e.pre_range) {
        mclks += t.pre_range_mclks;
    }
    CHK(wr16(d, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encode_timeout(mclks)));
    d->budget_us = budget_us;
    return ESP_OK;
}

/* ---------------- init steps ---------------- */

static esp_err_t wait_reg(vl53l0x_handle_t d, uint8_t reg, uint8_t mask, bool want_nonzero)
{
    int64_t until = esp_timer_get_time() + WAIT_TIMEOUT_MS * 1000;
    for (;;) {
        uint8_t v;
        CHK(rd8(d, reg, &v));
        if (((v & mask) != 0) == want_nonzero) {
            return ESP_OK;
        }
        if (esp_timer_get_time() > until) {
            return ESP_ERR_TIMEOUT;
        }
        vTaskDelay(1);
    }
}

static esp_err_t get_spad_info(vl53l0x_handle_t d, uint8_t *count, bool *is_aperture)
{
    static const regval_t a[] = { {0x80, 0x01}, {0xFF, 0x01}, {0x00, 0x00}, {0xFF, 0x06} };
    CHK(wr_list(d, a, 4));
    CHK(set_bits(d, 0x83, 0x04, 0));
    static const regval_t b[] = { {0xFF, 0x07}, {0x81, 0x01}, {0x80, 0x01}, {0x94, 0x6b}, {0x83, 0x00} };
    CHK(wr_list(d, b, 5));
    CHK(wait_reg(d, 0x83, 0xFF, true));
    CHK(wr8(d, 0x83, 0x01));
    uint8_t tmp;
    CHK(rd8(d, 0x92, &tmp));
    *count = tmp & 0x7F;
    *is_aperture = (tmp >> 7) & 1;
    CHK(wr8(d, 0x81, 0x00));
    CHK(wr8(d, 0xFF, 0x06));
    CHK(set_bits(d, 0x83, 0, 0x04));
    static const regval_t c[] = { {0xFF, 0x01}, {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00} };
    return wr_list(d, c, 4);
}

static esp_err_t single_ref_calibration(vl53l0x_handle_t d, uint8_t vhv_init_byte)
{
    CHK(wr8(d, SYSRANGE_START, 0x01 | vhv_init_byte));
    CHK(wait_reg(d, RESULT_INTERRUPT_STATUS, 0x07, true));
    CHK(wr8(d, SYSTEM_INTERRUPT_CLEAR, 0x01));
    return wr8(d, SYSRANGE_START, 0x00);
}

/* DefaultTuningSettings from ST's vl53l0x_tuning.h */
static const regval_t s_tuning[] = {
    {0xFF, 0x01}, {0x00, 0x00}, {0xFF, 0x00}, {0x09, 0x00}, {0x10, 0x00}, {0x11, 0x00},
    {0x24, 0x01}, {0x25, 0xFF}, {0x75, 0x00}, {0xFF, 0x01}, {0x4E, 0x2C}, {0x48, 0x00},
    {0x30, 0x20}, {0xFF, 0x00}, {0x30, 0x09}, {0x54, 0x00}, {0x31, 0x04}, {0x32, 0x03},
    {0x40, 0x83}, {0x46, 0x25}, {0x60, 0x00}, {0x27, 0x00}, {0x50, 0x06}, {0x51, 0x00},
    {0x52, 0x96}, {0x56, 0x08}, {0x57, 0x30}, {0x61, 0x00}, {0x62, 0x00}, {0x64, 0x00},
    {0x65, 0x00}, {0x66, 0xA0}, {0xFF, 0x01}, {0x22, 0x32}, {0x47, 0x14}, {0x49, 0xFF},
    {0x4A, 0x00}, {0xFF, 0x00}, {0x7A, 0x0A}, {0x7B, 0x00}, {0x78, 0x21}, {0xFF, 0x01},
    {0x23, 0x34}, {0x42, 0x00}, {0x44, 0xFF}, {0x45, 0x26}, {0x46, 0x05}, {0x40, 0x40},
    {0x0E, 0x06}, {0x20, 0x1A}, {0x43, 0x40}, {0xFF, 0x00}, {0x34, 0x03}, {0x35, 0x44},
    {0xFF, 0x01}, {0x31, 0x04}, {0x4B, 0x09}, {0x4C, 0x05}, {0x4D, 0x04}, {0xFF, 0x00},
    {0x44, 0x00}, {0x45, 0x20}, {0x47, 0x08}, {0x48, 0x28}, {0x67, 0x00}, {0x70, 0x04},
    {0x71, 0x01}, {0x72, 0xFE}, {0x76, 0x00}, {0x77, 0x00}, {0xFF, 0x01}, {0x0D, 0x01},
    {0xFF, 0x00}, {0x80, 0x01}, {0x01, 0xF8}, {0xFF, 0x01}, {0x8E, 0x01}, {0x00, 0x01},
    {0xFF, 0x00}, {0x80, 0x00},
};

esp_err_t vl53l0x_init(vl53l0x_handle_t d)
{
    uint8_t id;
    CHK(rd8(d, IDENTIFICATION_MODEL_ID, &id));
    if (id != 0xEE) {
        ESP_LOGW(TAG, "unexpected model ID 0x%02x", id);
        return ESP_ERR_NOT_FOUND;
    }

    /* DataInit: 2V8 I/O mode (breakout boards), standard I2C mode, read stop variable */
    CHK(set_bits(d, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV, 0x01, 0));
    CHK(wr8(d, 0x88, 0x00));
    static const regval_t a[] = { {0x80, 0x01}, {0xFF, 0x01}, {0x00, 0x00} };
    CHK(wr_list(d, a, 3));
    CHK(rd8(d, 0x91, &d->stop_variable));
    static const regval_t b[] = { {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00} };
    CHK(wr_list(d, b, 3));

    /* Disable SIGNAL_RATE_MSRC and SIGNAL_RATE_PRE_RANGE limit checks */
    CHK(set_bits(d, MSRC_CONFIG_CONTROL, 0x12, 0));
    /* Final range signal rate limit 0.25 MCPS (9.7 fixed point) */
    CHK(wr16(d, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, (uint16_t)(0.25f * (1 << 7))));
    CHK(wr8(d, SYSTEM_SEQUENCE_CONFIG, 0xFF));

    /* StaticInit: reference SPADs */
    uint8_t spad_count;
    bool aperture;
    CHK(get_spad_info(d, &spad_count, &aperture));
    uint8_t map[6];
    CHK(rd_multi(d, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, map, 6));
    CHK(wr8(d, 0xFF, 0x01));
    CHK(wr8(d, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00));
    CHK(wr8(d, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C));
    CHK(wr8(d, 0xFF, 0x00));
    CHK(wr8(d, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4));
    uint8_t first = aperture ? 12 : 0, enabled = 0;
    for (uint8_t i = 0; i < 48; i++) {
        if (i < first || enabled == spad_count) {
            map[i / 8] &= ~(1 << (i % 8));
        } else if ((map[i / 8] >> (i % 8)) & 1) {
            enabled++;
        }
    }
    CHK(wr_multi(d, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, map, 6));

    CHK(wr_list(d, s_tuning, sizeof(s_tuning) / sizeof(s_tuning[0])));

    /* Interrupt on new sample ready, active low */
    CHK(wr8(d, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04));
    CHK(set_bits(d, GPIO_HV_MUX_ACTIVE_HIGH, 0, 0x10));
    CHK(wr8(d, SYSTEM_INTERRUPT_CLEAR, 0x01));

    CHK(get_timing_budget(d, &d->budget_us));
    /* Disable MSRC and TCC, then recalculate the timing budget */
    CHK(wr8(d, SYSTEM_SEQUENCE_CONFIG, 0xE8));
    CHK(vl53l0x_set_timing_budget_us(d, d->budget_us));

    /* Reference calibration: VHV, then phase */
    CHK(wr8(d, SYSTEM_SEQUENCE_CONFIG, 0x01));
    CHK(single_ref_calibration(d, 0x40));
    CHK(wr8(d, SYSTEM_SEQUENCE_CONFIG, 0x02));
    CHK(single_ref_calibration(d, 0x00));
    CHK(wr8(d, SYSTEM_SEQUENCE_CONFIG, 0xE8));

    ESP_LOGI(TAG, "init ok: %u ref SPADs (%s), budget %lu us", spad_count, aperture ? "aperture" : "non-aperture",
             (unsigned long)d->budget_us);
    return ESP_OK;
}

esp_err_t vl53l0x_start_continuous(vl53l0x_handle_t d)
{
    static const regval_t a[] = { {0x80, 0x01}, {0xFF, 0x01}, {0x00, 0x00} };
    CHK(wr_list(d, a, 3));
    CHK(wr8(d, 0x91, d->stop_variable));
    static const regval_t b[] = { {0x00, 0x01}, {0xFF, 0x00}, {0x80, 0x00} };
    CHK(wr_list(d, b, 3));
    return wr8(d, SYSRANGE_START, 0x02);  /* back-to-back mode */
}

esp_err_t vl53l0x_stop_continuous(vl53l0x_handle_t d)
{
    static const regval_t a[] = { {SYSRANGE_START, 0x01}, {0xFF, 0x01}, {0x00, 0x00}, {0x91, 0x00},
                                  {0x00, 0x01}, {0xFF, 0x00} };
    return wr_list(d, a, 6);
}

esp_err_t vl53l0x_read_sample(vl53l0x_handle_t d, vl53l0x_sample_t *out)
{
    uint8_t irq;
    CHK(rd8(d, RESULT_INTERRUPT_STATUS, &irq));
    if ((irq & 0x07) == 0) {
        return ESP_ERR_NOT_FINISHED;
    }
    uint8_t b[12];
    CHK(rd_multi(d, RESULT_RANGE_STATUS, b, sizeof(b)));
    CHK(wr8(d, SYSTEM_INTERRUPT_CLEAR, 0x01));
    out->status = (b[0] & 0x78) >> 3;
    out->signal_mcps = (((uint16_t)b[6] << 8) | b[7]) / 128.0f;   /* 9.7 fixed point */
    out->ambient_mcps = (((uint16_t)b[8] << 8) | b[9]) / 128.0f;
    out->range_mm = ((uint16_t)b[10] << 8) | b[11];
    return ESP_OK;
}

const char *vl53l0x_status_name(uint8_t s)
{
    static const char *names[16] = {
        "none", "VCSEL continuity", "VCSEL watchdog", "no VHV value", "no target (MSRC)", "SNR check",
        "phase out of range", "sigma too high", "TCC", "phase consistency", "min clip", "ok",
        "algo underflow", "algo overflow", "below threshold", "?",
    };
    return names[s & 0x0F];
}

esp_err_t vl53l0x_new(const vl53l0x_config_t *cfg, vl53l0x_handle_t *out)
{
    ESP_RETURN_ON_FALSE(cfg && cfg->bus && out, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    struct vl53l0x *d = calloc(1, sizeof(*d));
    ESP_RETURN_ON_FALSE(d, ESP_ERR_NO_MEM, TAG, "no mem");
    const i2c_device_config_t dc = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VL53L0X_ADDR,
        .scl_speed_hz = cfg->scl_hz ? cfg->scl_hz : 400000,
    };
    esp_err_t e = i2c_master_bus_add_device(cfg->bus, &dc, &d->dev);
    if (e != ESP_OK) {
        free(d);
        return e;
    }
    d->lock = cfg->bus_lock;
    *out = d;
    return ESP_OK;
}

void vl53l0x_del(vl53l0x_handle_t d)
{
    if (d) {
        i2c_master_bus_rm_device(d->dev);
        free(d);
    }
}
