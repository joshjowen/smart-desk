/*
 * MQTT client with Home Assistant discovery (stage 6).
 * Base topic desk/<device_id>; availability, retained state JSON and commands.
 * Remote moves follow the remote-move rules: off unless "Allow remote moves",
 * an on-screen countdown before each move (in motion), rate limited, no jog.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    MQTT_HA_OFF,          /* no broker configured */
    MQTT_HA_CONNECTING,
    MQTT_HA_CONNECTED,
} mqtt_ha_state_t;

typedef struct {
    mqtt_ha_state_t state;
    char broker[128];
    char base_topic[48];
    char error[80];          /* last connection error, empty if none */
    uint32_t retry_in_s;     /* while disconnected */
    uint32_t connects;       /* successful connections since boot */
} mqtt_ha_status_t;

esp_err_t mqtt_ha_start(const desk_config_t *cfg);
/* Config listener: reconnects if the broker settings, device ID or CA changed. */
void mqtt_ha_apply_config(const desk_config_t *cfg);
void mqtt_ha_get_status(mqtt_ha_status_t *out);

#ifdef __cplusplus
}
#endif
