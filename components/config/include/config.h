/*
 * Typed, versioned settings persisted in NVS.
 * config_get() returns a copy; config_update() validates, saves and notifies.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CONFIG_VERSION      1
#define CFG_PRESET_COUNT    4   /* sit, stand, custom1, custom2 (same order as motion_preset_t) */

typedef struct {
    char name[16];
    int32_t mm;
    bool enabled;
} cfg_preset_t;

typedef struct {
    uint32_t version;

    /* Network */
    char wifi_ssid[33];
    char wifi_pass[65];
    char hostname[33];

    /* Presets */
    cfg_preset_t presets[CFG_PRESET_COUNT];

    /* Display */
    bool units_cm;
    uint16_t autolock_s;
    uint16_t dim_s;
    uint8_t backlight;

    /* MQTT (stage 6) */
    char mqtt_uri[128];
    char mqtt_user[64];
    char mqtt_pass[64];
    char mqtt_device_id[33];   /* empty = hostname */

    /* Safety */
    bool allow_remote_moves;

    /* Admin password (PBKDF2-SHA256). Empty hash = no password. */
    uint8_t admin_salt[16];
    uint8_t admin_hash[32];
    bool admin_set;
    bool admin_prompt_dismissed;
} desk_config_t;

typedef void (*config_listener_t)(const desk_config_t *cfg);

/* Initialises NVS and loads settings (defaults on first boot or version change). */
esp_err_t config_init(void);
void config_get(desk_config_t *out);

/* Validates `next` and, if valid, saves it and notifies listeners.
 * On error returns ESP_ERR_INVALID_ARG and fills `err` with a message. */
esp_err_t config_update(const desk_config_t *next, char *err, size_t err_len);

void config_add_listener(config_listener_t fn);

/* Admin password */
bool config_admin_check(const char *password);
esp_err_t config_admin_set(const char *password);   /* NULL or "" clears it */

/* Erases all settings (factory reset). Caller reboots afterwards. */
esp_err_t config_factory_reset(void);

/* Validation limits, shared with the web UI */
#define CFG_AUTOLOCK_MIN 5
#define CFG_AUTOLOCK_MAX 600
#define CFG_DIM_MIN      5
#define CFG_DIM_MAX      3600
#define CFG_BL_MIN       5
#define CFG_BL_MAX       100

/* Presets must lie in this range (set from motion at boot). */
void config_set_height_range(int32_t min_mm, int32_t max_mm);

#ifdef __cplusplus
}
#endif
