#include "config.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_random.h"
#include "mbedtls/pkcs5.h"
#include "mbedtls/md.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

static const char *TAG = "config";

#define NVS_NS      "desk"
#define NVS_KEY     "cfg"
#define MAX_LISTENERS 6
#define PBKDF2_ITER 4000

static desk_config_t s_cfg;
static SemaphoreHandle_t s_mutex;
static config_listener_t s_listeners[MAX_LISTENERS];
static int s_n_listeners;
static int32_t s_min_mm = 0, s_max_mm = 3000;

#ifdef CONFIG_DESK_UI_UNITS_CM
#define DEF_UNITS_CM true
#else
#define DEF_UNITS_CM false
#endif

static void defaults(desk_config_t *c)
{
    memset(c, 0, sizeof(*c));
    c->version = CONFIG_VERSION;
    strlcpy(c->hostname, "desk", sizeof(c->hostname));
    const char *names[CFG_PRESET_COUNT] = { "Sit", "Stand", "Custom 1", "Custom 2" };
    const int32_t mm[CFG_PRESET_COUNT] = { CONFIG_DESK_PRESET_SIT_MM, CONFIG_DESK_PRESET_STAND_MM, 900, 1000 };
    for (int i = 0; i < CFG_PRESET_COUNT; i++) {
        strlcpy(c->presets[i].name, names[i], sizeof(c->presets[i].name));
        c->presets[i].mm = mm[i];
        c->presets[i].enabled = i < 2;
    }
    c->units_cm = DEF_UNITS_CM;
    c->autolock_s = CONFIG_DESK_UI_AUTOLOCK_S;
    c->dim_s = CONFIG_DESK_UI_DIM_S;
    c->backlight = CONFIG_DESK_UI_BACKLIGHT;
    c->allow_remote_moves = false;
    c->sensor_profile = 0;
    c->sensor_ema_pct = CONFIG_DESK_HEIGHT_EMA_PCT;
}

static esp_err_t save(const desk_config_t *c)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    err = nvs_set_blob(h, NVS_KEY, c, sizeof(*c));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

esp_err_t config_init(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS partition needs erase (%s)", esp_err_to_name(err));
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        return err;
    }
    s_mutex = xSemaphoreCreateMutex();

    /* Older versions are a prefix of the current struct (fields are only ever
     * appended), so load what is stored and default the rest. */
    nvs_handle_t h;
    bool loaded = false;
    defaults(&s_cfg);
    if (nvs_open(NVS_NS, NVS_READONLY, &h) == ESP_OK) {
        size_t len = 0;
        if (nvs_get_blob(h, NVS_KEY, NULL, &len) == ESP_OK && len >= sizeof(uint32_t) && len <= sizeof(s_cfg)) {
            desk_config_t stored;
            defaults(&stored);
            if (nvs_get_blob(h, NVS_KEY, &stored, &len) == ESP_OK && stored.version >= 1 &&
                stored.version <= CONFIG_VERSION) {
                if (stored.version != CONFIG_VERSION) {
                    ESP_LOGW(TAG, "migrating settings v%lu -> v%d", (unsigned long)stored.version, CONFIG_VERSION);
                }
                s_cfg = stored;
                s_cfg.version = CONFIG_VERSION;
                loaded = true;
            }
        }
        nvs_close(h);
    }
    if (!loaded) {
        ESP_LOGW(TAG, "no valid settings: using defaults");
    }
    save(&s_cfg);
    ESP_LOGI(TAG, "settings v%lu loaded: hostname '%s', wifi '%s', remote moves %s, admin password %s",
             (unsigned long)s_cfg.version, s_cfg.hostname, s_cfg.wifi_ssid[0] ? s_cfg.wifi_ssid : "(none)",
             s_cfg.allow_remote_moves ? "ON" : "off", s_cfg.admin_set ? "set" : "not set");
    return ESP_OK;
}

void config_get(desk_config_t *out)
{
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    *out = s_cfg;
    xSemaphoreGive(s_mutex);
}

void config_add_listener(config_listener_t fn)
{
    if (s_n_listeners < MAX_LISTENERS) {
        s_listeners[s_n_listeners++] = fn;
    }
}

void config_set_height_range(int32_t min_mm, int32_t max_mm)
{
    s_min_mm = min_mm;
    s_max_mm = max_mm;
}

static bool valid_hostname(const char *h)
{
    size_t n = strlen(h);
    if (n < 1 || n > 32 || h[0] == '-' || h[n - 1] == '-') {
        return false;
    }
    for (size_t i = 0; i < n; i++) {
        if (!(islower((unsigned char)h[i]) || isdigit((unsigned char)h[i]) || h[i] == '-')) {
            return false;
        }
    }
    return true;
}

#define FAIL(...) do { snprintf(err, err_len, __VA_ARGS__); return false; } while (0)

static bool validate(const desk_config_t *c, char *err, size_t err_len)
{
    size_t pl = strnlen(c->wifi_pass, sizeof(c->wifi_pass));
    if (strnlen(c->wifi_ssid, sizeof(c->wifi_ssid)) > 32) FAIL("Wi-Fi SSID: at most 32 characters");
    if (pl != 0 && (pl < 8 || pl > 63)) FAIL("Wi-Fi password: 8 to 63 characters (or empty for an open network)");
    if (!valid_hostname(c->hostname)) FAIL("Hostname: 1-32 characters, lowercase letters, digits and '-'");
    for (int i = 0; i < CFG_PRESET_COUNT; i++) {
        const cfg_preset_t *p = &c->presets[i];
        if (strnlen(p->name, sizeof(p->name)) == 0 || strnlen(p->name, sizeof(p->name)) > 15)
            FAIL("Preset %d name: 1-15 characters", i + 1);
        if (p->enabled && (p->mm < s_min_mm || p->mm > s_max_mm))
            FAIL("Preset '%s': height must be %ld-%ld mm", p->name, (long)s_min_mm, (long)s_max_mm);
    }
    if (c->autolock_s < CFG_AUTOLOCK_MIN || c->autolock_s > CFG_AUTOLOCK_MAX)
        FAIL("Auto-lock: %d-%d s", CFG_AUTOLOCK_MIN, CFG_AUTOLOCK_MAX);
    if (c->dim_s < CFG_DIM_MIN || c->dim_s > CFG_DIM_MAX) FAIL("Dim timeout: %d-%d s", CFG_DIM_MIN, CFG_DIM_MAX);
    if (c->backlight < CFG_BL_MIN || c->backlight > CFG_BL_MAX) FAIL("Backlight: %d-%d %%", CFG_BL_MIN, CFG_BL_MAX);
    if (c->mqtt_uri[0] && strncmp(c->mqtt_uri, "mqtt://", 7) != 0 && strncmp(c->mqtt_uri, "mqtts://", 8) != 0)
        FAIL("MQTT broker: must start with mqtt:// or mqtts://");
    for (const char *p = c->ntp_server; *p; p++) {
        if (!(isalnum((unsigned char)*p) || *p == '.' || *p == '-')) FAIL("Time server: a hostname or IP address");
    }
    if (c->sensor_profile > 1) FAIL("Sensor profile: default or high accuracy");
    if (c->sensor_ema_pct < CFG_EMA_MIN || c->sensor_ema_pct > CFG_EMA_MAX)
        FAIL("Filter alpha: %.2f-%.2f", CFG_EMA_MIN / 100.0, CFG_EMA_MAX / 100.0);
    if (c->mqtt_device_id[0] && !valid_hostname(c->mqtt_device_id))
        FAIL("MQTT device ID: lowercase letters, digits and '-'");
    return true;
}

esp_err_t config_update(const desk_config_t *next, char *err, size_t err_len)
{
    if (!validate(next, err, err_len)) {
        ESP_LOGW(TAG, "rejected settings: %s", err);
        return ESP_ERR_INVALID_ARG;
    }
    desk_config_t copy;
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    s_cfg = *next;
    s_cfg.version = CONFIG_VERSION;
    esp_err_t e = save(&s_cfg);
    copy = s_cfg;
    xSemaphoreGive(s_mutex);
    if (e != ESP_OK) {
        snprintf(err, err_len, "Saving failed: %s", esp_err_to_name(e));
        return e;
    }
    ESP_LOGI(TAG, "settings saved");
    for (int i = 0; i < s_n_listeners; i++) {
        s_listeners[i](&copy);
    }
    return ESP_OK;
}

static void hash_pw(const char *pw, const uint8_t salt[16], uint8_t out[32])
{
    mbedtls_pkcs5_pbkdf2_hmac_ext(MBEDTLS_MD_SHA256, (const unsigned char *)pw, strlen(pw), salt, 16,
                                  PBKDF2_ITER, 32, out);
}

bool config_admin_check(const char *password)
{
    desk_config_t c;
    config_get(&c);
    if (!c.admin_set) {
        return true;
    }
    uint8_t h[32];
    hash_pw(password ? password : "", c.admin_salt, h);
    uint8_t diff = 0;
    for (int i = 0; i < 32; i++) {
        diff |= h[i] ^ c.admin_hash[i];
    }
    return diff == 0;
}

esp_err_t config_admin_set(const char *password)
{
    desk_config_t c;
    config_get(&c);
    if (password && password[0]) {
        esp_fill_random(c.admin_salt, sizeof(c.admin_salt));
        hash_pw(password, c.admin_salt, c.admin_hash);
        c.admin_set = true;
    } else {
        memset(c.admin_salt, 0, sizeof(c.admin_salt));
        memset(c.admin_hash, 0, sizeof(c.admin_hash));
        c.admin_set = false;
    }
    c.admin_prompt_dismissed = true;
    char err[64];
    return config_update(&c, err, sizeof(err));
}

esp_err_t config_factory_reset(void)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS, NVS_READWRITE, &h);
    if (err != ESP_OK) {
        return err;
    }
    nvs_erase_all(h);
    err = nvs_commit(h);
    nvs_close(h);
    ESP_LOGW(TAG, "factory reset: settings erased");
    return err;
}
