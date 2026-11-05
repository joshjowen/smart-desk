/*
 * REST API:
 *   GET  /api/status                 live state (open)
 *   POST /api/login                  {password} -> {token}
 *   GET  /api/config  PUT /api/config
 *   GET  /api/system
 *   GET  /api/wifi/scan
 *   POST /api/password               {password} sets/clears, {dismiss:true} hides the prompt
 *   POST /api/reboot  /api/factory-reset  /api/ota (raw .bin body)
 * Everything except status and login needs the admin token once a password is set.
 */
#include <stdio.h>
#include <string.h>
#include <time.h>
#include "esp_app_desc.h"
#include "esp_app_format.h"
#include "esp_heap_caps.h"
#include "esp_image_format.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_random.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "config.h"
#include "motion.h"
#include "net.h"
#include "ui.h"
#include "web_priv.h"

static const char *TAG = "web_api";

#define MAX_BODY      4096
#define N_TOKENS      4

static char s_tokens[N_TOKENS][33];
static int s_next_token;
static esp_timer_handle_t s_restart_timer;
static bool s_factory_pending;

static const char *preset_ids[MOTION_PRESET_COUNT] = { "sit", "stand", "custom1", "custom2" };

/* ---------------- helpers ---------------- */

esp_err_t web_send_json(httpd_req_t *req, int status, cJSON *obj)
{
    char *s = cJSON_PrintUnformatted(obj);
    cJSON_Delete(obj);
    if (status != 200) {
        char st[16];
        snprintf(st, sizeof(st), "%d", status);
        httpd_resp_set_status(req, status == 400 ? "400 Bad Request" : status == 401 ? "401 Unauthorized" :
                                   status == 403 ? "403 Forbidden" : status == 409 ? "409 Conflict" :
                                   "500 Internal Server Error");
    }
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    esp_err_t e = httpd_resp_sendstr(req, s ? s : "{}");
    cJSON_free(s);
    return e;
}

esp_err_t web_send_error(httpd_req_t *req, int status, const char *msg)
{
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "error", msg);
    return web_send_json(req, status, o);
}

static esp_err_t send_ok(httpd_req_t *req)
{
    cJSON *o = cJSON_CreateObject();
    cJSON_AddBoolToObject(o, "ok", true);
    return web_send_json(req, 200, o);
}

/* Reads a JSON body (max MAX_BODY). Returns NULL and sends an error on failure. */
static cJSON *read_json(httpd_req_t *req)
{
    if (req->content_len <= 0 || req->content_len > MAX_BODY) {
        web_send_error(req, 400, "Request body missing or too large");
        return NULL;
    }
    char *buf = malloc(req->content_len + 1);
    if (!buf) {
        web_send_error(req, 500, "Out of memory");
        return NULL;
    }
    int got = 0;
    while (got < req->content_len) {
        int r = httpd_req_recv(req, buf + got, req->content_len - got);
        if (r == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (r <= 0) {
            free(buf);
            return NULL;
        }
        got += r;
    }
    buf[got] = 0;
    cJSON *o = cJSON_Parse(buf);
    free(buf);
    if (!o) {
        web_send_error(req, 400, "Invalid JSON");
    }
    return o;
}

bool web_authorized(httpd_req_t *req)
{
    desk_config_t c;
    config_get(&c);
    if (!c.admin_set) {
        return true;
    }
    char hdr[48];
    if (httpd_req_get_hdr_value_str(req, "Authorization", hdr, sizeof(hdr)) != ESP_OK ||
        strncmp(hdr, "Bearer ", 7) != 0) {
        return false;
    }
    for (int i = 0; i < N_TOKENS; i++) {
        if (s_tokens[i][0] && strcmp(s_tokens[i], hdr + 7) == 0) {
            return true;
        }
    }
    return false;
}

#define REQUIRE_AUTH(req) do { if (!web_authorized(req)) return web_send_error(req, 401, "Login required"); } while (0)

bool web_moves_allowed(void)
{
    desk_config_t c;
    config_get(&c);
    return c.allow_remote_moves || !ui_is_locked();
}

static void restart_cb(void *arg)
{
    if (s_factory_pending) {
        config_factory_reset();
    }
    esp_restart();
}

static void schedule_restart(bool factory)
{
    motion_stop(MOTION_SRC_WEB);
    s_factory_pending = factory;
    if (!s_restart_timer) {
        const esp_timer_create_args_t t = { .callback = restart_cb, .name = "restart" };
        esp_timer_create(&t, &s_restart_timer);
    }
    esp_timer_start_once(s_restart_timer, 800 * 1000);
}

/* ---------------- status ---------------- */

cJSON *web_status_json(void)
{
    motion_status_t st;
    motion_get_status(&st);
    desk_config_t c;
    config_get(&c);

    cJSON *o = cJSON_CreateObject();
    if (st.height_valid) {
        cJSON_AddNumberToObject(o, "height_mm", st.height_mm);
    } else {
        cJSON_AddNullToObject(o, "height_mm");
    }
    cJSON_AddStringToObject(o, "state", motion_state_name(st.state));
    cJSON_AddStringToObject(o, "output", st.output == MOTION_DIR_UP ? "up" : st.output == MOTION_DIR_DOWN ? "down" : "off");
    if (st.state == MOTION_SEEK || st.state == MOTION_SETTLE) {
        cJSON_AddNumberToObject(o, "target_mm", st.target_mm);
    } else {
        cJSON_AddNullToObject(o, "target_mm");
    }
    cJSON_AddStringToObject(o, "fault", st.fault);
    cJSON_AddNumberToObject(o, "duty_remaining_s", st.duty_remaining_s);
    cJSON_AddNumberToObject(o, "cooldown_s", st.cooldown_remaining_s);
    cJSON_AddBoolToObject(o, "locked", ui_is_locked());
    cJSON_AddBoolToObject(o, "remote_moves_allowed", c.allow_remote_moves);
    cJSON_AddBoolToObject(o, "presets_available", st.presets_available);
    cJSON_AddStringToObject(o, "last_error", st.last_error);
    cJSON_AddNumberToObject(o, "last_error_seq", st.last_error_seq);
    cJSON_AddStringToObject(o, "units", c.units_cm ? "cm" : "mm");
    cJSON *ps = cJSON_AddArrayToObject(o, "presets");
    for (int i = 0; i < MOTION_PRESET_COUNT; i++) {
        if (!c.presets[i].enabled) {
            continue;
        }
        cJSON *p = cJSON_CreateObject();
        cJSON_AddStringToObject(p, "id", preset_ids[i]);
        cJSON_AddStringToObject(p, "name", c.presets[i].name);
        cJSON_AddNumberToObject(p, "mm", c.presets[i].mm);
        cJSON_AddItemToArray(ps, p);
    }
    return o;
}

static esp_err_t status_get(httpd_req_t *req)
{
    return web_send_json(req, 200, web_status_json());
}

/* ---------------- login / password ---------------- */

static esp_err_t login_post(httpd_req_t *req)
{
    cJSON *o = read_json(req);
    if (!o) {
        return ESP_OK;
    }
    const cJSON *pw = cJSON_GetObjectItem(o, "password");
    bool ok = cJSON_IsString(pw) && config_admin_check(pw->valuestring);
    cJSON_Delete(o);
    if (!ok) {
        vTaskDelay(pdMS_TO_TICKS(500));  /* slow down guessing */
        return web_send_error(req, 401, "Wrong password");
    }
    uint8_t rnd[16];
    esp_fill_random(rnd, sizeof(rnd));
    char *t = s_tokens[s_next_token];
    for (int i = 0; i < 16; i++) {
        sprintf(t + i * 2, "%02x", rnd[i]);
    }
    s_next_token = (s_next_token + 1) % N_TOKENS;
    cJSON *r = cJSON_CreateObject();
    cJSON_AddStringToObject(r, "token", t);
    return web_send_json(req, 200, r);
}

static esp_err_t password_post(httpd_req_t *req)
{
    REQUIRE_AUTH(req);
    cJSON *o = read_json(req);
    if (!o) {
        return ESP_OK;
    }
    esp_err_t err;
    if (cJSON_IsTrue(cJSON_GetObjectItem(o, "dismiss"))) {
        desk_config_t c;
        config_get(&c);
        c.admin_prompt_dismissed = true;
        char msg[96];
        err = config_update(&c, msg, sizeof(msg));
    } else {
        const cJSON *pw = cJSON_GetObjectItem(o, "password");
        if (!cJSON_IsString(pw) || (pw->valuestring[0] && strlen(pw->valuestring) < 6) ||
            strlen(pw->valuestring) > 64) {
            cJSON_Delete(o);
            return web_send_error(req, 400, "Password: 6 to 64 characters (empty removes it)");
        }
        err = config_admin_set(pw->valuestring);
        memset(s_tokens, 0, sizeof(s_tokens));  /* log everyone out */
    }
    cJSON_Delete(o);
    return err == ESP_OK ? send_ok(req) : web_send_error(req, 500, "Saving failed");
}

/* ---------------- config ---------------- */

static cJSON *config_json(const desk_config_t *c)
{
    cJSON *o = cJSON_CreateObject();
    cJSON_AddNumberToObject(o, "version", c->version);
    cJSON *w = cJSON_AddObjectToObject(o, "wifi");
    cJSON_AddStringToObject(w, "ssid", c->wifi_ssid);
    cJSON_AddBoolToObject(w, "password_set", c->wifi_pass[0] != 0);
    cJSON_AddStringToObject(o, "hostname", c->hostname);
    cJSON *ps = cJSON_AddArrayToObject(o, "presets");
    for (int i = 0; i < CFG_PRESET_COUNT; i++) {
        cJSON *p = cJSON_CreateObject();
        cJSON_AddStringToObject(p, "id", preset_ids[i]);
        cJSON_AddStringToObject(p, "name", c->presets[i].name);
        cJSON_AddNumberToObject(p, "mm", c->presets[i].mm);
        cJSON_AddBoolToObject(p, "enabled", c->presets[i].enabled);
        cJSON_AddItemToArray(ps, p);
    }
    cJSON *d = cJSON_AddObjectToObject(o, "display");
    cJSON_AddStringToObject(d, "units", c->units_cm ? "cm" : "mm");
    cJSON_AddNumberToObject(d, "autolock_s", c->autolock_s);
    cJSON_AddNumberToObject(d, "dim_s", c->dim_s);
    cJSON_AddNumberToObject(d, "backlight", c->backlight);
    cJSON *m = cJSON_AddObjectToObject(o, "mqtt");
    cJSON_AddStringToObject(m, "uri", c->mqtt_uri);
    cJSON_AddStringToObject(m, "username", c->mqtt_user);
    cJSON_AddBoolToObject(m, "password_set", c->mqtt_pass[0] != 0);
    cJSON_AddStringToObject(m, "device_id", c->mqtt_device_id);
    cJSON_AddBoolToObject(o, "allow_remote_moves", c->allow_remote_moves);
    cJSON_AddBoolToObject(o, "admin_password_set", c->admin_set);
    cJSON_AddBoolToObject(o, "admin_prompt", !c->admin_set && !c->admin_prompt_dismissed);
    int32_t lo, hi;
    motion_get_range(&lo, &hi);
    cJSON *lim = cJSON_AddObjectToObject(o, "limits");
    cJSON_AddNumberToObject(lim, "min_mm", lo);
    cJSON_AddNumberToObject(lim, "max_mm", hi);
    return o;
}

static esp_err_t config_get_h(httpd_req_t *req)
{
    REQUIRE_AUTH(req);
    desk_config_t c;
    config_get(&c);
    return web_send_json(req, 200, config_json(&c));
}

/* JSON merge helpers: absent keys keep the current value; wrong types are errors. */
static bool take_str(const cJSON *parent, const char *key, char *dst, size_t n, const char *label, char *err, size_t el)
{
    const cJSON *v = cJSON_GetObjectItem(parent, key);
    if (!v) return true;
    if (!cJSON_IsString(v)) { snprintf(err, el, "%s: expected text", label); return false; }
    if (strlen(v->valuestring) >= n) { snprintf(err, el, "%s: at most %u characters", label, (unsigned)(n - 1)); return false; }
    strlcpy(dst, v->valuestring, n);
    return true;
}

static bool take_int(const cJSON *parent, const char *key, int32_t *dst, int32_t lo, int32_t hi, const char *label, char *err, size_t el)
{
    const cJSON *v = cJSON_GetObjectItem(parent, key);
    if (!v) return true;
    if (!cJSON_IsNumber(v)) { snprintf(err, el, "%s: expected a number", label); return false; }
    if (v->valuedouble < lo || v->valuedouble > hi) { snprintf(err, el, "%s: must be %ld-%ld", label, (long)lo, (long)hi); return false; }
    *dst = (int32_t)v->valuedouble;
    return true;
}

static bool take_bool(const cJSON *parent, const char *key, bool *dst, const char *label, char *err, size_t el)
{
    const cJSON *v = cJSON_GetObjectItem(parent, key);
    if (!v) return true;
    if (!cJSON_IsBool(v)) { snprintf(err, el, "%s: expected true/false", label); return false; }
    *dst = cJSON_IsTrue(v);
    return true;
}

static bool merge_config(const cJSON *o, desk_config_t *c, char *err, size_t el)
{
    const cJSON *w = cJSON_GetObjectItem(o, "wifi");
    if (w) {
        if (!take_str(w, "ssid", c->wifi_ssid, sizeof(c->wifi_ssid), "Wi-Fi SSID", err, el)) return false;
        if (!take_str(w, "password", c->wifi_pass, sizeof(c->wifi_pass), "Wi-Fi password", err, el)) return false;
    }
    if (!take_str(o, "hostname", c->hostname, sizeof(c->hostname), "Hostname", err, el)) return false;

    const cJSON *ps = cJSON_GetObjectItem(o, "presets");
    if (ps) {
        if (!cJSON_IsArray(ps) || cJSON_GetArraySize(ps) != CFG_PRESET_COUNT) {
            snprintf(err, el, "Presets: expected %d entries", CFG_PRESET_COUNT);
            return false;
        }
        for (int i = 0; i < CFG_PRESET_COUNT; i++) {
            const cJSON *p = cJSON_GetArrayItem(ps, i);
            int32_t mm = c->presets[i].mm;
            char label[24];
            snprintf(label, sizeof(label), "Preset %d", i + 1);
            if (!take_str(p, "name", c->presets[i].name, sizeof(c->presets[i].name), label, err, el)) return false;
            if (!take_int(p, "mm", &mm, 0, 3000, label, err, el)) return false;
            if (!take_bool(p, "enabled", &c->presets[i].enabled, label, err, el)) return false;
            c->presets[i].mm = mm;
        }
    }

    const cJSON *d = cJSON_GetObjectItem(o, "display");
    if (d) {
        const cJSON *u = cJSON_GetObjectItem(d, "units");
        if (u) {
            if (!cJSON_IsString(u) || (strcmp(u->valuestring, "mm") && strcmp(u->valuestring, "cm"))) {
                snprintf(err, el, "Units: mm or cm");
                return false;
            }
            c->units_cm = strcmp(u->valuestring, "cm") == 0;
        }
        int32_t a = c->autolock_s, di = c->dim_s, b = c->backlight;
        if (!take_int(d, "autolock_s", &a, CFG_AUTOLOCK_MIN, CFG_AUTOLOCK_MAX, "Auto-lock (s)", err, el)) return false;
        if (!take_int(d, "dim_s", &di, CFG_DIM_MIN, CFG_DIM_MAX, "Dim timeout (s)", err, el)) return false;
        if (!take_int(d, "backlight", &b, CFG_BL_MIN, CFG_BL_MAX, "Backlight (%)", err, el)) return false;
        c->autolock_s = a;
        c->dim_s = di;
        c->backlight = b;
    }

    const cJSON *m = cJSON_GetObjectItem(o, "mqtt");
    if (m) {
        if (!take_str(m, "uri", c->mqtt_uri, sizeof(c->mqtt_uri), "MQTT broker", err, el)) return false;
        if (!take_str(m, "username", c->mqtt_user, sizeof(c->mqtt_user), "MQTT username", err, el)) return false;
        if (!take_str(m, "password", c->mqtt_pass, sizeof(c->mqtt_pass), "MQTT password", err, el)) return false;
        if (!take_str(m, "device_id", c->mqtt_device_id, sizeof(c->mqtt_device_id), "MQTT device ID", err, el)) return false;
    }
    return take_bool(o, "allow_remote_moves", &c->allow_remote_moves, "Allow remote moves", err, el);
}

static esp_err_t config_put(httpd_req_t *req)
{
    REQUIRE_AUTH(req);
    cJSON *o = read_json(req);
    if (!o) {
        return ESP_OK;
    }
    desk_config_t c;
    config_get(&c);
    char err[128] = "";
    bool ok = merge_config(o, &c, err, sizeof(err));
    cJSON_Delete(o);
    if (!ok || config_update(&c, err, sizeof(err)) != ESP_OK) {
        return web_send_error(req, 400, err);
    }
    config_get(&c);
    return web_send_json(req, 200, config_json(&c));
}

/* ---------------- system ---------------- */

static const char *reset_reason(void)
{
    switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "power-on";
    case ESP_RST_SW: return "software";
    case ESP_RST_PANIC: return "panic";
    case ESP_RST_TASK_WDT: return "task watchdog";
    case ESP_RST_INT_WDT: return "interrupt watchdog";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_USB: return "USB";
    default: return "other";
    }
}

static esp_err_t system_get(httpd_req_t *req)
{
    REQUIRE_AUTH(req);
    const esp_app_desc_t *app = esp_app_get_description();
    motion_status_t st;
    motion_get_status(&st);
    net_status_t n;
    net_get_status(&n);
    const esp_partition_t *run = esp_ota_get_running_partition();

    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "version", app->version);
    cJSON_AddStringToObject(o, "idf", app->idf_ver);
    cJSON_AddStringToObject(o, "build", app->date);
    cJSON_AddNumberToObject(o, "uptime_s", (double)(esp_timer_get_time() / 1000000));
    cJSON_AddNumberToObject(o, "free_heap", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    cJSON_AddNumberToObject(o, "min_free_heap", heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));
    cJSON_AddStringToObject(o, "reset_reason", reset_reason());
    cJSON_AddStringToObject(o, "partition", run ? run->label : "?");
    cJSON_AddStringToObject(o, "hostname", n.hostname);
    cJSON_AddStringToObject(o, "ssid", n.sta_ssid);
    cJSON_AddBoolToObject(o, "connected", n.sta_connected);
    cJSON_AddStringToObject(o, "ip", n.ip);
    if (n.sta_connected) {
        cJSON_AddNumberToObject(o, "rssi", n.rssi);
    } else {
        cJSON_AddNullToObject(o, "rssi");
    }
    cJSON_AddBoolToObject(o, "ap_active", n.ap_active);
    cJSON_AddStringToObject(o, "ap_ssid", n.ap_ssid);
    cJSON_AddBoolToObject(o, "time_synced", n.time_synced);
    cJSON_AddStringToObject(o, "last_fault", st.last_fault);
    cJSON_AddNumberToObject(o, "duty_remaining_s", st.duty_remaining_s);
    return web_send_json(req, 200, o);
}

static esp_err_t scan_get(httpd_req_t *req)
{
    REQUIRE_AUTH(req);
    uint16_t n = 20;
    wifi_ap_record_t *aps = calloc(n, sizeof(*aps));
    if (!aps) {
        return web_send_error(req, 500, "Out of memory");
    }
    esp_err_t err = net_scan(aps, &n);
    if (err != ESP_OK) {
        free(aps);
        return web_send_error(req, 409, "Scan not possible right now; try again");
    }
    cJSON *arr = cJSON_CreateArray();
    for (int i = 0; i < n; i++) {
        bool dup = false;
        for (int j = 0; j < i; j++) {
            dup |= strcmp((char *)aps[i].ssid, (char *)aps[j].ssid) == 0;
        }
        if (dup || !aps[i].ssid[0]) {
            continue;
        }
        cJSON *a = cJSON_CreateObject();
        cJSON_AddStringToObject(a, "ssid", (char *)aps[i].ssid);
        cJSON_AddNumberToObject(a, "rssi", aps[i].rssi);
        cJSON_AddBoolToObject(a, "secure", aps[i].authmode != WIFI_AUTH_OPEN);
        cJSON_AddItemToArray(arr, a);
    }
    free(aps);
    cJSON *o = cJSON_CreateObject();
    cJSON_AddItemToObject(o, "networks", arr);
    return web_send_json(req, 200, o);
}

static esp_err_t reboot_post(httpd_req_t *req)
{
    REQUIRE_AUTH(req);
    ESP_LOGW(TAG, "reboot requested from web");
    schedule_restart(false);
    return send_ok(req);
}

static esp_err_t factory_post(httpd_req_t *req)
{
    REQUIRE_AUTH(req);
    ESP_LOGW(TAG, "factory reset requested from web");
    schedule_restart(true);
    return send_ok(req);
}

/* ---------------- OTA ---------------- */

static esp_err_t ota_post(httpd_req_t *req)
{
    REQUIRE_AUTH(req);
    motion_stop(MOTION_SRC_WEB);

    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    if (!part) {
        return web_send_error(req, 500, "No OTA partition");
    }
    if (req->content_len <= 0 || req->content_len > part->size) {
        return web_send_error(req, 400, "Firmware file missing or too large");
    }
    ESP_LOGW(TAG, "OTA: receiving %d bytes into %s", req->content_len, part->label);

    char *buf = malloc(4096);
    if (!buf) {
        return web_send_error(req, 500, "Out of memory");
    }
    esp_ota_handle_t ota = 0;
    bool begun = false, header_checked = false;
    const char *fail = NULL;
    int remaining = req->content_len;
    int fill = 0;   /* bytes buffered before the header check */

    while (remaining > 0 && !fail) {
        int r = httpd_req_recv(req, buf + fill, MIN(remaining, 4096 - fill));
        if (r == HTTPD_SOCK_ERR_TIMEOUT) {
            continue;
        }
        if (r <= 0) {
            fail = "Upload interrupted";
            break;
        }
        remaining -= r;
        fill += r;

        if (!header_checked) {
            size_t need = sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t) + sizeof(esp_app_desc_t);
            if (fill < need && remaining > 0) {
                continue;
            }
            const esp_image_header_t *ih = (const esp_image_header_t *)buf;
            const esp_app_desc_t *d = (const esp_app_desc_t *)(buf + sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t));
            if (fill < need || ih->magic != ESP_IMAGE_HEADER_MAGIC || d->magic_word != ESP_APP_DESC_MAGIC_WORD) {
                fail = "Not an ESP32 firmware image";
                break;
            }
            if (strcmp(d->project_name, esp_app_get_description()->project_name) != 0) {
                fail = "Firmware is for a different project";
                break;
            }
            ESP_LOGI(TAG, "OTA: image version %s", d->version);
            header_checked = true;
            if (esp_ota_begin(part, OTA_WITH_SEQUENTIAL_WRITES, &ota) != ESP_OK) {
                fail = "OTA begin failed";
                break;
            }
            begun = true;
        }
        if (esp_ota_write(ota, buf, fill) != ESP_OK) {
            fail = "Flash write failed";
            break;
        }
        fill = 0;
    }
    free(buf);

    if (!fail && begun) {
        esp_err_t e = esp_ota_end(ota);
        begun = false;
        if (e == ESP_ERR_OTA_VALIDATE_FAILED) {
            fail = "Image failed verification (corrupt upload?)";
        } else if (e != ESP_OK) {
            fail = "OTA finish failed";
        } else if (esp_ota_set_boot_partition(part) != ESP_OK) {
            fail = "Could not select new firmware";
        }
    }
    if (begun) {
        esp_ota_abort(ota);
    }
    if (fail) {
        ESP_LOGE(TAG, "OTA failed: %s", fail);
        return web_send_error(req, 400, fail);
    }
    ESP_LOGW(TAG, "OTA complete: rebooting into %s", part->label);
    schedule_restart(false);
    return send_ok(req);
}

/* ---------------- registration ---------------- */

esp_err_t web_api_register(httpd_handle_t h)
{
    const httpd_uri_t uris[] = {
        { .uri = "/api/status", .method = HTTP_GET, .handler = status_get },
        { .uri = "/api/login", .method = HTTP_POST, .handler = login_post },
        { .uri = "/api/password", .method = HTTP_POST, .handler = password_post },
        { .uri = "/api/config", .method = HTTP_GET, .handler = config_get_h },
        { .uri = "/api/config", .method = HTTP_PUT, .handler = config_put },
        { .uri = "/api/system", .method = HTTP_GET, .handler = system_get },
        { .uri = "/api/wifi/scan", .method = HTTP_GET, .handler = scan_get },
        { .uri = "/api/reboot", .method = HTTP_POST, .handler = reboot_post },
        { .uri = "/api/factory-reset", .method = HTTP_POST, .handler = factory_post },
        { .uri = "/api/ota", .method = HTTP_POST, .handler = ota_post },
    };
    for (size_t i = 0; i < sizeof(uris) / sizeof(uris[0]); i++) {
        esp_err_t e = httpd_register_uri_handler(h, &uris[i]);
        if (e != ESP_OK) {
            ESP_LOGE(TAG, "register %s failed: %s", uris[i].uri, esp_err_to_name(e));
        }
    }
    return ESP_OK;
}
