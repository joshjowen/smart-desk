#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "esp_wifi_types.h"
#include "config.h"

#ifdef __cplusplus
extern "C" {
#endif

#define NET_AP_IP "192.168.4.1"

typedef struct {
    bool sta_configured;
    bool sta_connected;
    bool ap_active;
    bool time_synced;
    int8_t rssi;
    char ip[16];          /* station IP, empty when not connected */
    char sta_ssid[33];
    char ap_ssid[33];     /* Desk-Setup-XXXX */
    char hostname[33];
} net_status_t;

/* Starts Wi-Fi: station if configured, otherwise the setup AP. */
esp_err_t net_start(const desk_config_t *cfg);
/* Config listener: reconnects when Wi-Fi settings change, renames on hostname change. */
void net_apply_config(const desk_config_t *cfg);
void net_get_status(net_status_t *out);
/* Blocking scan (about 2 s). `n` in: capacity, out: count. */
esp_err_t net_scan(wifi_ap_record_t *out, uint16_t *n);

#ifdef __cplusplus
}
#endif
