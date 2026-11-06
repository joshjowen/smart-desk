/*
 * MQTT + Home Assistant discovery (stage 6).
 *
 *   desk/<id>/status   retained "online" / "offline" (last will)
 *   desk/<id>/state    retained JSON, 2 Hz while moving, on change, every 60 s
 *   desk/<id>/cmd      {"cmd":"preset","id":"stand"} {"cmd":"goto","mm":1080} {"cmd":"stop"}
 *   homeassistant/<component>/<id>_<object>/config   retained discovery
 *
 * Moves (preset / goto) need "Allow remote moves", are rate limited to one per
 * 2 s and start after the on-screen countdown (motion). STOP is always
 * accepted. There is no MQTT jog. A move in progress does not depend on the
 * connection.
 */
#include "mqtt_ha.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "cJSON.h"
#include "esp_app_desc.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "mqtt_client.h"
#include "motion.h"
#include "net.h"
#include "ui.h"

static const char *TAG = "mqtt_ha";

#define BACKOFF_MIN_S        1
#define BACKOFF_MAX_S        60
#define MOVING_PERIOD_US     (500 * 1000)
#define KEEPALIVE_US         (60LL * 1000000)
#define IDLE_HEIGHT_STEP_MM  5
#define MOVE_RATE_US         (2LL * 1000000)

static const char *s_preset_ids[MOTION_PRESET_COUNT] = { "sit", "stand", "custom1", "custom2" };

static SemaphoreHandle_t s_mutex;           /* guards client (re)creation and settings */
static esp_mqtt_client_handle_t s_client;
static esp_timer_handle_t s_retry_timer;
static portMUX_TYPE s_st_lock = portMUX_INITIALIZER_UNLOCKED;
static mqtt_ha_status_t s_st;
static uint32_t s_backoff_s = BACKOFF_MIN_S;
static int64_t s_retry_at_us;

static char s_uri[128], s_user[64], s_pass[64], s_id[33], s_host[33];
static char *s_ca;
static char s_base[48], s_t_status[64], s_t_state[64], s_t_cmd[64];

static volatile bool s_connected, s_republish, s_force_state;
static int64_t s_last_move_us = -MOVE_RATE_US;
static char s_disc_sig[256];

/* ---------------- status ---------------- */

static void set_state(mqtt_ha_state_t st, const char *err)
{
    portENTER_CRITICAL(&s_st_lock);
    s_st.state = st;
    if (err) {
        strlcpy(s_st.error, err, sizeof(s_st.error));
    }
    if (st == MQTT_HA_CONNECTED) {
        s_st.connects++;
        s_st.error[0] = 0;
    }
    portEXIT_CRITICAL(&s_st_lock);
    ui_set_mqtt_state(st == MQTT_HA_OFF ? 0 : st == MQTT_HA_CONNECTED ? 2 : 1);
}

void mqtt_ha_get_status(mqtt_ha_status_t *out)
{
    portENTER_CRITICAL(&s_st_lock);
    *out = s_st;
    portEXIT_CRITICAL(&s_st_lock);
    int64_t now = esp_timer_get_time();
    out->retry_in_s = out->state == MQTT_HA_CONNECTING && s_retry_at_us > now ? (uint32_t)((s_retry_at_us - now + 999999) / 1000000) : 0;
}

/* ---------------- state JSON ---------------- */

static bool is_moving(motion_state_t s)
{
    return s == MOTION_JOG || s == MOTION_SEEK || s == MOTION_SETTLE || s == MOTION_CALIBRATING;
}

/* Builds the state JSON. `cmp` gets the same JSON without height / duty (for change detection). */
static char *state_json(const motion_status_t *st, const desk_config_t *c, char **cmp)
{
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "state", motion_state_name(st->state));
    if (st->state == MOTION_SEEK || st->state == MOTION_SETTLE) {
        cJSON_AddNumberToObject(o, "target_mm", st->target_mm);
    } else {
        cJSON_AddNullToObject(o, "target_mm");
    }
    cJSON_AddStringToObject(o, "fault", st->fault);
    cJSON_AddBoolToObject(o, "locked", ui_is_locked());
    cJSON_AddBoolToObject(o, "remote_moves_allowed", c->allow_remote_moves);
    cJSON_AddStringToObject(o, "last_error", st->last_error);
    cJSON_AddBoolToObject(o, "calibrated", st->calibrated);
    if (st->pending_active) {
        cJSON_AddNumberToObject(o, "pending_target_mm", st->pending_target_mm);
    } else {
        cJSON_AddNullToObject(o, "pending_target_mm");
    }
    *cmp = cJSON_PrintUnformatted(o);
    if (st->height_valid) {
        cJSON_AddNumberToObject(o, "height_mm", st->height_mm);
    } else {
        cJSON_AddNullToObject(o, "height_mm");
    }
    cJSON_AddNumberToObject(o, "duty_remaining_s", st->duty_remaining_s);
    char *s = cJSON_PrintUnformatted(o);
    cJSON_Delete(o);
    return s;
}

static void publish(const char *topic, const char *payload, int qos, bool retain)
{
    esp_mqtt_client_publish(s_client, topic, payload, 0, qos, retain);
}

/* ---------------- discovery ---------------- */

static cJSON *entity(const char *name, const char *obj)
{
    cJSON *o = cJSON_CreateObject();
    char uid[64];
    snprintf(uid, sizeof(uid), "%s_%s", s_id, obj);
    cJSON_AddStringToObject(o, "name", name);
    cJSON_AddStringToObject(o, "unique_id", uid);
    cJSON_AddStringToObject(o, "availability_topic", s_t_status);
    cJSON *d = cJSON_AddObjectToObject(o, "device");
    cJSON *ids = cJSON_AddArrayToObject(d, "identifiers");
    cJSON_AddItemToArray(ids, cJSON_CreateString(s_id));
    cJSON_AddStringToObject(d, "name", s_host);
    cJSON_AddStringToObject(d, "model", "Otto Chatham desk controller (ESP32-C6)");
    cJSON_AddStringToObject(d, "manufacturer", "smart-desk");
    cJSON_AddStringToObject(d, "sw_version", esp_app_get_description()->version);
    char url[64];
    snprintf(url, sizeof(url), "http://%s.local", s_host);
    cJSON_AddStringToObject(d, "configuration_url", url);
    cJSON *orig = cJSON_AddObjectToObject(o, "origin");
    cJSON_AddStringToObject(orig, "name", "smart-desk");
    return o;
}

static void with_state(cJSON *o, const char *tmpl)
{
    cJSON_AddStringToObject(o, "state_topic", s_t_state);
    cJSON_AddStringToObject(o, "value_template", tmpl);
}

static void send_config(const char *component, const char *obj, cJSON *o)
{
    char topic[96];
    snprintf(topic, sizeof(topic), "homeassistant/%s/%s_%s/config", component, s_id, obj);
    char *s = o ? cJSON_PrintUnformatted(o) : NULL;
    publish(topic, s ? s : "", 1, true);   /* empty retained payload removes the entity */
    cJSON_free(s);
    cJSON_Delete(o);
}

static void publish_discovery(const motion_status_t *st, const desk_config_t *c)
{
    cJSON *o;

    o = entity("Height", "height");
    with_state(o, "{{ value_json.height_mm }}");
    cJSON_AddStringToObject(o, "device_class", "distance");
    cJSON_AddStringToObject(o, "unit_of_measurement", "mm");
    cJSON_AddStringToObject(o, "state_class", "measurement");
    send_config("sensor", "height", o);

    if (st->calibrated) {
        o = entity("Target height", "target");
        with_state(o, "{{ value_json.target_mm if value_json.target_mm is not none else value_json.height_mm }}");
        cJSON_AddStringToObject(o, "command_topic", s_t_cmd);
        cJSON_AddStringToObject(o, "command_template", "{\"cmd\":\"goto\",\"mm\":{{ value | int }}}");
        cJSON_AddNumberToObject(o, "min", st->min_mm);
        cJSON_AddNumberToObject(o, "max", st->max_mm);
        cJSON_AddNumberToObject(o, "step", 10);
        cJSON_AddStringToObject(o, "unit_of_measurement", "mm");
        cJSON_AddStringToObject(o, "mode", "box");
        cJSON_AddStringToObject(o, "icon", "mdi:desk");
        send_config("number", "target", o);
    } else {
        send_config("number", "target", NULL);
    }

    for (int i = 0; i < MOTION_PRESET_COUNT; i++) {
        char obj[24];
        snprintf(obj, sizeof(obj), "preset_%s", s_preset_ids[i]);
        if (!c->presets[i].enabled || !st->calibrated) {
            send_config("button", obj, NULL);
            continue;
        }
        o = entity(c->presets[i].name, obj);
        char press[64];
        snprintf(press, sizeof(press), "{\"cmd\":\"preset\",\"id\":\"%s\"}", s_preset_ids[i]);
        cJSON_AddStringToObject(o, "command_topic", s_t_cmd);
        cJSON_AddStringToObject(o, "payload_press", press);
        cJSON_AddStringToObject(o, "icon", "mdi:desk");
        send_config("button", obj, o);
    }

    o = entity("Stop", "stop");
    cJSON_AddStringToObject(o, "command_topic", s_t_cmd);
    cJSON_AddStringToObject(o, "payload_press", "{\"cmd\":\"stop\"}");
    cJSON_AddStringToObject(o, "icon", "mdi:stop-circle");
    send_config("button", "stop", o);

    o = entity("State", "state");
    with_state(o, "{{ value_json.state }}");
    cJSON_AddStringToObject(o, "device_class", "enum");
    cJSON *opts = cJSON_AddArrayToObject(o, "options");
    static const char *states[] = { "idle", "jog", "seek", "settle", "calibrating", "cooldown", "fault" };
    for (size_t i = 0; i < sizeof(states) / sizeof(states[0]); i++) {
        cJSON_AddItemToArray(opts, cJSON_CreateString(states[i]));
    }
    send_config("sensor", "state", o);

    o = entity("Moving", "moving");
    with_state(o, "{{ 'ON' if value_json.state in ['jog', 'seek', 'settle', 'calibrating'] else 'OFF' }}");
    cJSON_AddStringToObject(o, "device_class", "moving");
    send_config("binary_sensor", "moving", o);

    o = entity("Fault", "fault");
    with_state(o, "{{ 'ON' if value_json.fault else 'OFF' }}");
    cJSON_AddStringToObject(o, "device_class", "problem");
    cJSON_AddStringToObject(o, "json_attributes_topic", s_t_state);
    cJSON_AddStringToObject(o, "json_attributes_template", "{{ {'fault': value_json.fault} | tojson }}");
    send_config("binary_sensor", "fault", o);

    o = entity("Duty budget remaining", "duty");
    with_state(o, "{{ value_json.duty_remaining_s }}");
    cJSON_AddStringToObject(o, "unit_of_measurement", "s");
    cJSON_AddStringToObject(o, "device_class", "duration");
    cJSON_AddStringToObject(o, "entity_category", "diagnostic");
    send_config("sensor", "duty", o);

    o = entity("Remote moves allowed", "remote");
    with_state(o, "{{ 'ON' if value_json.remote_moves_allowed else 'OFF' }}");
    cJSON_AddStringToObject(o, "entity_category", "diagnostic");
    cJSON_AddStringToObject(o, "icon", "mdi:remote");
    send_config("binary_sensor", "remote", o);

    o = entity("Last error", "last_error");
    with_state(o, "{{ value_json.last_error }}");
    cJSON_AddStringToObject(o, "entity_category", "diagnostic");
    cJSON_AddStringToObject(o, "icon", "mdi:alert-circle-outline");
    send_config("sensor", "last_error", o);

    ESP_LOGI(TAG, "discovery published");
}

/* What the discovery depends on: republish when it changes. */
static void disc_signature(const motion_status_t *st, const desk_config_t *c, char *out, size_t n)
{
    int len = snprintf(out, n, "%d:%ld:%ld:%s", st->calibrated, (long)st->min_mm, (long)st->max_mm, s_host);
    for (int i = 0; i < MOTION_PRESET_COUNT && len < (int)n; i++) {
        len += snprintf(out + len, n - len, "|%d%s", c->presets[i].enabled, c->presets[i].name);
    }
}

/* ---------------- commands ---------------- */

static void refuse(const char *why)
{
    char m[48];
    snprintf(m, sizeof(m), "MQTT: %s", why);
    motion_note_error(m);
    s_force_state = true;
}

static void handle_cmd(const char *data, int len)
{
    char buf[257];
    if (len <= 0 || len >= (int)sizeof(buf)) {
        return;
    }
    memcpy(buf, data, len);
    buf[len] = 0;
    cJSON *o = cJSON_Parse(buf);
    const cJSON *cmd = cJSON_GetObjectItem(o, "cmd");
    if (!cJSON_IsString(cmd)) {
        refuse("bad command");
        cJSON_Delete(o);
        return;
    }
    const char *c = cmd->valuestring;
    ESP_LOGI(TAG, "command: %s", buf);
    if (strcmp(c, "stop") == 0) {
        motion_stop(MOTION_SRC_MQTT);   /* always accepted */
    } else if (strcmp(c, "jog") == 0) {
        refuse("no jogging over MQTT");
    } else if (strcmp(c, "preset") == 0 || strcmp(c, "goto") == 0) {
        desk_config_t cfg;
        config_get(&cfg);
        int64_t now = esp_timer_get_time();
        if (!cfg.allow_remote_moves) {
            refuse("remote moves are off on the desk");
        } else if (now - s_last_move_us < MOVE_RATE_US) {
            refuse("one move command per 2 s");
        } else {
            motion_req_t r = { .source = MOTION_SRC_MQTT };
            bool ok = false;
            if (c[0] == 'p') {
                const cJSON *id = cJSON_GetObjectItem(o, "id");
                for (int i = 0; cJSON_IsString(id) && i < MOTION_PRESET_COUNT; i++) {
                    if (strcmp(id->valuestring, s_preset_ids[i]) == 0) {
                        r.type = MOTION_REQ_GOTO_PRESET;
                        r.value = i;
                        ok = true;
                    }
                }
                if (!ok) refuse("unknown preset");
            } else {
                const cJSON *mm = cJSON_GetObjectItem(o, "mm");
                if (cJSON_IsNumber(mm)) {
                    r.type = MOTION_REQ_GOTO_HEIGHT;
                    r.value = (int32_t)lround(mm->valuedouble);
                    ok = true;
                } else {
                    refuse("goto needs mm");
                }
            }
            if (ok) {
                s_last_move_us = now;
                motion_request(&r);
                s_force_state = true;
            }
        }
    } else {
        refuse("unknown command");
    }
    cJSON_Delete(o);
}

/* ---------------- connection ---------------- */

static void schedule_retry(void)
{
    if (esp_timer_is_active(s_retry_timer)) {
        return;
    }
    s_retry_at_us = esp_timer_get_time() + (int64_t)s_backoff_s * 1000000;
    esp_timer_start_once(s_retry_timer, (uint64_t)s_backoff_s * 1000000);
    ESP_LOGI(TAG, "reconnecting in %lu s", (unsigned long)s_backoff_s);
    s_backoff_s = s_backoff_s * 2 > BACKOFF_MAX_S ? BACKOFF_MAX_S : s_backoff_s * 2;
}

static void retry_cb(void *arg)
{
    if (s_client) {
        esp_mqtt_client_reconnect(s_client);
    }
}

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    esp_mqtt_event_handle_t e = data;
    switch ((esp_mqtt_event_id_t)id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "connected to %s as %s", s_uri, s_base);
        s_connected = true;
        s_backoff_s = BACKOFF_MIN_S;
        publish(s_t_status, "online", 1, true);
        esp_mqtt_client_subscribe(s_client, s_t_cmd, 1);
        esp_mqtt_client_subscribe(s_client, "homeassistant/status", 1);
        s_republish = true;
        set_state(MQTT_HA_CONNECTED, NULL);
        break;
    case MQTT_EVENT_DISCONNECTED:
        if (s_connected) {
            ESP_LOGW(TAG, "disconnected");
        }
        s_connected = false;
        set_state(MQTT_HA_CONNECTING, NULL);
        schedule_retry();
        break;
    case MQTT_EVENT_ERROR: {
        char err[80] = "connection failed";
        if (e->error_handle->error_type == MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
            int rc = e->error_handle->connect_return_code;
            snprintf(err, sizeof(err), "%s", rc == MQTT_CONNECTION_REFUSE_BAD_USERNAME ||
                     rc == MQTT_CONNECTION_REFUSE_NOT_AUTHORIZED ? "broker refused the login" :
                     "broker refused the connection");
        } else if (e->error_handle->error_type == MQTT_ERROR_TYPE_TCP_TRANSPORT) {
            if (e->error_handle->esp_tls_last_esp_err) {
                snprintf(err, sizeof(err), "can't reach the broker (%s)", esp_err_to_name(e->error_handle->esp_tls_last_esp_err));
            } else {
                snprintf(err, sizeof(err), "can't reach the broker (errno %d)", e->error_handle->esp_transport_sock_errno);
            }
        }
        ESP_LOGW(TAG, "%s", err);
        set_state(s_connected ? MQTT_HA_CONNECTED : MQTT_HA_CONNECTING, err);
        if (!s_connected) {
            schedule_retry();
        }
        break;
    }
    case MQTT_EVENT_DATA:
        if (e->topic_len == (int)strlen(s_t_cmd) && strncmp(e->topic, s_t_cmd, e->topic_len) == 0) {
            handle_cmd(e->data, e->data_len);
        } else if (e->topic_len == 20 && strncmp(e->topic, "homeassistant/status", 20) == 0 &&
                   e->data_len == 6 && strncmp(e->data, "online", 6) == 0) {
            ESP_LOGI(TAG, "Home Assistant restarted: republishing");
            s_republish = true;
        }
        break;
    default:
        break;
    }
}

/* Called with s_mutex held. */
static void client_stop(void)
{
    if (s_retry_timer) {
        esp_timer_stop(s_retry_timer);
    }
    if (s_client) {
        if (s_connected) {
            publish(s_t_status, "offline", 1, true);
        }
        esp_mqtt_client_stop(s_client);
        esp_mqtt_client_destroy(s_client);
        s_client = NULL;
    }
    s_connected = false;
}

static void load_settings(const desk_config_t *c)
{
    strlcpy(s_uri, c->mqtt_uri, sizeof(s_uri));
    strlcpy(s_user, c->mqtt_user, sizeof(s_user));
    strlcpy(s_pass, c->mqtt_pass, sizeof(s_pass));
    strlcpy(s_host, c->hostname, sizeof(s_host));
    strlcpy(s_id, c->mqtt_device_id[0] ? c->mqtt_device_id : c->hostname, sizeof(s_id));
    snprintf(s_base, sizeof(s_base), "desk/%s", s_id);
    snprintf(s_t_status, sizeof(s_t_status), "%s/status", s_base);
    snprintf(s_t_state, sizeof(s_t_state), "%s/state", s_base);
    snprintf(s_t_cmd, sizeof(s_t_cmd), "%s/cmd", s_base);
    free(s_ca);
    s_ca = malloc(CFG_MQTT_CA_MAX);
    if (s_ca && (config_text_load("mqtt_ca", s_ca, CFG_MQTT_CA_MAX) != ESP_OK || !s_ca[0])) {
        free(s_ca);
        s_ca = NULL;
    }
    portENTER_CRITICAL(&s_st_lock);
    strlcpy(s_st.broker, s_uri, sizeof(s_st.broker));
    strlcpy(s_st.base_topic, s_base, sizeof(s_st.base_topic));
    portEXIT_CRITICAL(&s_st_lock);
}

/* Called with s_mutex held. */
static void client_start(void)
{
    if (!s_uri[0]) {
        set_state(MQTT_HA_OFF, "");
        ESP_LOGI(TAG, "no broker configured");
        return;
    }
    char client_id[48];
    snprintf(client_id, sizeof(client_id), "desk-%s", s_id);
    esp_mqtt_client_config_t mc = {
        .broker.address.uri = s_uri,
        .credentials.username = s_user[0] ? s_user : NULL,
        .credentials.authentication.password = s_pass[0] ? s_pass : NULL,
        .credentials.client_id = client_id,
        .session.last_will = { .topic = s_t_status, .msg = "offline", .qos = 1, .retain = 1 },
        .session.keepalive = 30,
        .network.disable_auto_reconnect = true,
        .network.timeout_ms = 10000,
        .buffer.size = 1024,
        .buffer.out_size = 2048,
        .task.stack_size = 6144,
    };
    if (strncmp(s_uri, "mqtts://", 8) == 0) {
        if (s_ca) {
            mc.broker.verification.certificate = s_ca;
        } else {
            mc.broker.verification.crt_bundle_attach = esp_crt_bundle_attach;
        }
    }
    s_backoff_s = BACKOFF_MIN_S;
    s_client = esp_mqtt_client_init(&mc);
    if (!s_client) {
        set_state(MQTT_HA_CONNECTING, "client init failed");
        return;
    }
    esp_mqtt_client_register_event(s_client, ESP_EVENT_ANY_ID, on_event, NULL);
    set_state(MQTT_HA_CONNECTING, "");
    esp_mqtt_client_start(s_client);
    ESP_LOGI(TAG, "connecting to %s (%s, %s)", s_uri, s_base, s_ca ? "own CA" : "no CA / bundle");
}

/* ---------------- publisher task ---------------- */

static void publisher_task(void *arg)
{
    char *last_cmp = NULL;
    int32_t last_h = INT32_MIN;
    uint32_t last_duty = 0;
    int64_t last_pub = 0;
    bool net_up = false;
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(100));
        xSemaphoreTake(s_mutex, portMAX_DELAY);
        /* Wi-Fi (re)connected: don't sit out the back-off, try the broker now. */
        net_status_t ns;
        net_get_status(&ns);
        if (ns.sta_connected && !net_up && s_client && !s_connected) {
            ESP_LOGI(TAG, "Wi-Fi up: connecting to the broker now");
            esp_timer_stop(s_retry_timer);
            s_backoff_s = BACKOFF_MIN_S;
            esp_mqtt_client_reconnect(s_client);
        }
        net_up = ns.sta_connected;
        if (!s_connected || !s_client) {
            xSemaphoreGive(s_mutex);
            free(last_cmp);
            last_cmp = NULL;
            continue;
        }
        motion_status_t st;
        motion_get_status(&st);
        desk_config_t c;
        config_get(&c);
        char sig[256];
        disc_signature(&st, &c, sig, sizeof(sig));
        bool force = s_force_state;
        if (s_republish || strcmp(sig, s_disc_sig) != 0) {
            s_republish = false;
            strlcpy(s_disc_sig, sig, sizeof(s_disc_sig));
            publish_discovery(&st, &c);
            force = true;
        }
        char *cmp = NULL;
        char *js = state_json(&st, &c, &cmp);
        int64_t now = esp_timer_get_time();
        bool moving = is_moving(st.state) || st.pending_active;
        int32_t h = st.height_valid ? st.height_mm : INT32_MIN;
        bool changed = !last_cmp || !cmp || strcmp(cmp, last_cmp) != 0 ||
                       (h == INT32_MIN) != (last_h == INT32_MIN) ||
                       (h != INT32_MIN && abs(h - last_h) >= IDLE_HEIGHT_STEP_MM) ||
                       abs((int)st.duty_remaining_s - (int)last_duty) >= 10;
        if (js && (force || changed || (moving && now - last_pub >= MOVING_PERIOD_US) || now - last_pub >= KEEPALIVE_US)) {
            publish(s_t_state, js, 0, true);
            s_force_state = false;
            last_pub = now;
            last_h = h;
            last_duty = st.duty_remaining_s;
            free(last_cmp);
            last_cmp = cmp;
            cmp = NULL;
        }
        cJSON_free(js);
        cJSON_free(cmp);
        xSemaphoreGive(s_mutex);
    }
}

/* ---------------- public ---------------- */

esp_err_t mqtt_ha_start(const desk_config_t *cfg)
{
    s_mutex = xSemaphoreCreateMutex();
    const esp_timer_create_args_t t = { .callback = retry_cb, .name = "mqtt_retry" };
    esp_timer_create(&t, &s_retry_timer);
    xSemaphoreTake(s_mutex, portMAX_DELAY);
    load_settings(cfg);
    client_start();
    xSemaphoreGive(s_mutex);
    return xTaskCreate(publisher_task, "mqtt_pub", 4096, NULL, 3, NULL) == pdPASS ? ESP_OK : ESP_ERR_NO_MEM;
}

void mqtt_ha_apply_config(const desk_config_t *cfg)
{
    const char *id = cfg->mqtt_device_id[0] ? cfg->mqtt_device_id : cfg->hostname;
    char ca[64] = "";
    /* Only the first bytes of the CA are compared: enough to notice a change. */
    char *full = malloc(CFG_MQTT_CA_MAX);
    if (full) {
        config_text_load("mqtt_ca", full, CFG_MQTT_CA_MAX);
    }
    size_t ca_len = full ? strlen(full) : 0;
    snprintf(ca, sizeof(ca), "%u:%.40s", (unsigned)ca_len, full && ca_len > 60 ? full + ca_len - 40 : (full ? full : ""));
    free(full);
    static char last_ca[64];
    static bool have_last;

    xSemaphoreTake(s_mutex, portMAX_DELAY);
    bool changed = strcmp(cfg->mqtt_uri, s_uri) || strcmp(cfg->mqtt_user, s_user) || strcmp(cfg->mqtt_pass, s_pass) ||
                   strcmp(id, s_id) || strcmp(cfg->hostname, s_host) || (have_last && strcmp(ca, last_ca));
    strlcpy(last_ca, ca, sizeof(last_ca));
    have_last = true;
    if (changed) {
        ESP_LOGI(TAG, "MQTT settings changed: reconnecting");
        client_stop();
        load_settings(cfg);
        s_disc_sig[0] = 0;
        client_start();
    }
    xSemaphoreGive(s_mutex);
}
