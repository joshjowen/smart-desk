/*
 * Wi-Fi station with SoftAP fallback, mDNS and SNTP.
 *  - No saved SSID: setup AP only (APSTA so the setup page can scan).
 *  - Saved SSID: station; if not connected for 60 s, the setup AP starts too.
 *    Once the station is connected and nobody is on the AP, the AP stops.
 */
#include "net.h"

#include <string.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_timer.h"
#include "esp_wifi.h"
#include "mdns.h"
#include "dns_captive.h"

static const char *TAG = "net";

#define FALLBACK_AP_US      (60LL * 1000 * 1000)
#define RECONNECT_US        (5LL * 1000 * 1000)
#define AP_STOP_CHECK_US    (10LL * 1000 * 1000)

static esp_netif_t *s_sta, *s_ap;
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;
static net_status_t s_st;
static esp_timer_handle_t s_fallback_timer, s_reconnect_timer, s_ap_check_timer;
static bool s_sntp_started;
static bool s_mdns_ok;

static void set_status(void (*fn)(net_status_t *, const void *), const void *arg)
{
    portENTER_CRITICAL(&s_lock);
    fn(&s_st, arg);
    portEXIT_CRITICAL(&s_lock);
}

void net_get_status(net_status_t *out)
{
    portENTER_CRITICAL(&s_lock);
    *out = s_st;
    portEXIT_CRITICAL(&s_lock);
    if (out->sta_connected) {
        wifi_ap_record_t ap;
        if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) {
            out->rssi = ap.rssi;
        }
    }
    out->time_synced = time(NULL) > 1700000000;
}

static void st_ap(net_status_t *s, const void *a) { s->ap_active = *(const bool *)a; }
static void st_conn(net_status_t *s, const void *a)
{
    s->sta_connected = a != NULL;
    if (a) {
        strlcpy(s->ip, a, sizeof(s->ip));
    } else {
        s->ip[0] = 0;
    }
}

static void ap_start(void)
{
    if (s_st.ap_active) {
        return;
    }
    wifi_config_t ap = { 0 };
    strlcpy((char *)ap.ap.ssid, s_st.ap_ssid, sizeof(ap.ap.ssid));
    ap.ap.ssid_len = strlen(s_st.ap_ssid);
    ap.ap.channel = 1;
    ap.ap.max_connection = 4;
    ap.ap.authmode = WIFI_AUTH_OPEN;
    esp_wifi_set_mode(WIFI_MODE_APSTA);
    esp_wifi_set_config(WIFI_IF_AP, &ap);
    bool on = true;
    set_status(st_ap, &on);
    dns_captive_start();
    ESP_LOGW(TAG, "setup AP '%s' up at http://%s", s_st.ap_ssid, NET_AP_IP);
}

static void ap_stop(void)
{
    if (!s_st.ap_active) {
        return;
    }
    dns_captive_stop();
    esp_wifi_set_mode(WIFI_MODE_STA);
    bool off = false;
    set_status(st_ap, &off);
    ESP_LOGI(TAG, "setup AP stopped");
}

static void fallback_cb(void *arg)
{
    if (!s_st.sta_connected) {
        ESP_LOGW(TAG, "no Wi-Fi connection for 60 s: starting setup AP");
        ap_start();
    }
}

static void reconnect_cb(void *arg)
{
    if (s_st.sta_configured && !s_st.sta_connected) {
        esp_wifi_connect();
    }
}

static void ap_check_cb(void *arg)
{
    if (!s_st.ap_active || !s_st.sta_connected) {
        return;
    }
    wifi_sta_list_t list;
    if (esp_wifi_ap_get_sta_list(&list) == ESP_OK && list.num == 0) {
        ap_stop();
    }
}

static void start_services(void)
{
    if (!s_sntp_started) {
        esp_sntp_config_t c = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
        if (esp_netif_sntp_init(&c) == ESP_OK) {
            s_sntp_started = true;
        }
    }
}

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT) {
        switch (id) {
        case WIFI_EVENT_STA_START:
            if (s_st.sta_configured) {
                esp_wifi_connect();
            }
            break;
        case WIFI_EVENT_STA_DISCONNECTED: {
            wifi_event_sta_disconnected_t *d = data;
            bool was = s_st.sta_connected;
            set_status(st_conn, NULL);
            if (was) {
                ESP_LOGW(TAG, "Wi-Fi lost (reason %d)", d->reason);
                esp_timer_stop(s_fallback_timer);
                esp_timer_start_once(s_fallback_timer, FALLBACK_AP_US);
            } else {
                ESP_LOGD(TAG, "connect failed (reason %d)", d->reason);
            }
            if (s_st.sta_configured) {
                esp_timer_stop(s_reconnect_timer);
                esp_timer_start_once(s_reconnect_timer, RECONNECT_US);
            }
            break;
        }
        case WIFI_EVENT_AP_STACONNECTED:
            ESP_LOGI(TAG, "client joined setup AP");
            break;
        default:
            break;
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = data;
        char ip[16];
        snprintf(ip, sizeof(ip), IPSTR, IP2STR(&e->ip_info.ip));
        set_status(st_conn, ip);
        esp_timer_stop(s_fallback_timer);
        ESP_LOGI(TAG, "connected to '%s', IP %s, http://%s.local", s_st.sta_ssid, ip, s_st.hostname);
        start_services();
    }
}

static void apply_mdns(const char *hostname)
{
    if (!s_mdns_ok) {
        if (mdns_init() != ESP_OK) {
            ESP_LOGE(TAG, "mDNS init failed");
            return;
        }
        mdns_instance_name_set("Desk controller");
        mdns_service_add(NULL, "_http", "_tcp", 80, NULL, 0);
        s_mdns_ok = true;
    }
    mdns_hostname_set(hostname);
}

static void configure_sta(const desk_config_t *cfg)
{
    wifi_config_t w = { 0 };
    strlcpy((char *)w.sta.ssid, cfg->wifi_ssid, sizeof(w.sta.ssid));
    strlcpy((char *)w.sta.password, cfg->wifi_pass, sizeof(w.sta.password));
    w.sta.threshold.authmode = cfg->wifi_pass[0] ? WIFI_AUTH_WPA_PSK : WIFI_AUTH_OPEN;
    w.sta.pmf_cfg.capable = true;
    w.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    w.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    esp_wifi_set_config(WIFI_IF_STA, &w);
}

static void st_names(net_status_t *s, const void *a)
{
    const desk_config_t *c = a;
    strlcpy(s->sta_ssid, c->wifi_ssid, sizeof(s->sta_ssid));
    strlcpy(s->hostname, c->hostname, sizeof(s->hostname));
    s->sta_configured = c->wifi_ssid[0] != 0;
}

esp_err_t net_start(const desk_config_t *cfg)
{
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_sta = esp_netif_create_default_wifi_sta();
    s_ap = esp_netif_create_default_wifi_ap();
    esp_netif_set_hostname(s_sta, cfg->hostname);

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));
    /* Credentials live in our own config; don't let the driver keep a copy in NVS. */
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, on_event, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, on_event, NULL));

    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP);
    snprintf(s_st.ap_ssid, sizeof(s_st.ap_ssid), "Desk-Setup-%02X%02X", mac[4], mac[5]);
    set_status(st_names, cfg);

    const esp_timer_create_args_t t1 = { .callback = fallback_cb, .name = "wifi_fb" };
    const esp_timer_create_args_t t2 = { .callback = reconnect_cb, .name = "wifi_rc" };
    const esp_timer_create_args_t t3 = { .callback = ap_check_cb, .name = "ap_chk" };
    esp_timer_create(&t1, &s_fallback_timer);
    esp_timer_create(&t2, &s_reconnect_timer);
    esp_timer_create(&t3, &s_ap_check_timer);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    if (s_st.sta_configured) {
        configure_sta(cfg);
        esp_timer_start_once(s_fallback_timer, FALLBACK_AP_US);
        ESP_LOGI(TAG, "connecting to '%s'", cfg->wifi_ssid);
    }
    ESP_ERROR_CHECK(esp_wifi_start());
    if (!s_st.sta_configured) {
        ESP_LOGW(TAG, "no Wi-Fi configured");
        ap_start();
    }
    esp_timer_start_periodic(s_ap_check_timer, AP_STOP_CHECK_US);
    apply_mdns(cfg->hostname);
    return ESP_OK;
}

void net_apply_config(const desk_config_t *cfg)
{
    net_status_t old;
    net_get_status(&old);
    bool wifi_changed = strcmp(old.sta_ssid, cfg->wifi_ssid) != 0;
    /* Password changes can't be compared from status; compare against the driver's copy. */
    wifi_config_t cur;
    if (esp_wifi_get_config(WIFI_IF_STA, &cur) == ESP_OK &&
        strncmp((char *)cur.sta.password, cfg->wifi_pass, sizeof(cur.sta.password)) != 0) {
        wifi_changed = true;
    }
    bool host_changed = strcmp(old.hostname, cfg->hostname) != 0;
    set_status(st_names, cfg);

    if (host_changed) {
        esp_netif_set_hostname(s_sta, cfg->hostname);
        apply_mdns(cfg->hostname);
        ESP_LOGI(TAG, "hostname now '%s'", cfg->hostname);
    }
    if (wifi_changed) {
        ESP_LOGI(TAG, "Wi-Fi settings changed: connecting to '%s'", cfg->wifi_ssid);
        esp_wifi_disconnect();
        set_status(st_conn, NULL);
        if (cfg->wifi_ssid[0]) {
            configure_sta(cfg);
            esp_wifi_connect();
            esp_timer_stop(s_fallback_timer);
            esp_timer_start_once(s_fallback_timer, FALLBACK_AP_US);
        } else {
            ap_start();
        }
    }
}

esp_err_t net_scan(wifi_ap_record_t *out, uint16_t *n)
{
    wifi_scan_config_t sc = { .show_hidden = false };
    esp_err_t err = esp_wifi_scan_start(&sc, true);
    if (err != ESP_OK) {
        *n = 0;
        return err;
    }
    return esp_wifi_scan_get_ap_records(n, out);
}
