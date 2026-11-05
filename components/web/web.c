/*
 * HTTP server: the embedded single-page app, the WebSocket live channel and
 * the captive-portal redirect. REST endpoints live in web_api.c.
 */
#include "web.h"

#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include "esp_log.h"
#include "esp_timer.h"
#include "motion.h"
#include "net.h"
#include "web_priv.h"

static const char *TAG = "web";

extern const uint8_t index_gz_start[] asm("_binary_index_html_gz_start");
extern const uint8_t index_gz_end[] asm("_binary_index_html_gz_end");

#define PUSH_PERIOD_US     (200 * 1000)   /* 5 Hz while moving */
#define IDLE_PUSH_TICKS    5              /* 1 Hz idle */

static httpd_handle_t s_server;
static esp_timer_handle_t s_push_timer;

/* Sockets that completed a WebSocket handshake. The server can't tell us this
 * reliably from inside close_fn, so we track it ourselves. */
#define MAX_WS 10
static int s_ws_fds[MAX_WS] = { -1, -1, -1, -1, -1, -1, -1, -1, -1, -1 };
static portMUX_TYPE s_ws_lock = portMUX_INITIALIZER_UNLOCKED;

static void ws_track(int fd, bool add)
{
    portENTER_CRITICAL(&s_ws_lock);
    for (int i = 0; i < MAX_WS; i++) {
        if (add ? s_ws_fds[i] < 0 : s_ws_fds[i] == fd) {
            s_ws_fds[i] = add ? fd : -1;
            break;
        }
    }
    portEXIT_CRITICAL(&s_ws_lock);
}

static bool ws_tracked(int fd)
{
    bool found = false;
    portENTER_CRITICAL(&s_ws_lock);
    for (int i = 0; i < MAX_WS; i++) {
        found |= s_ws_fds[i] == fd;
    }
    portEXIT_CRITICAL(&s_ws_lock);
    return found;
}

/* ---------------- page ---------------- */

static esp_err_t index_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html; charset=utf-8");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    return httpd_resp_send(req, (const char *)index_gz_start, index_gz_end - index_gz_start);
}

/* Any unknown URL: in setup-AP mode send phones' connectivity checks to the setup page. */
static esp_err_t not_found(httpd_req_t *req, httpd_err_code_t err)
{
    net_status_t n;
    net_get_status(&n);
    if (n.ap_active && strncmp(req->uri, "/api/", 5) != 0) {
        httpd_resp_set_status(req, "302 Found");
        httpd_resp_set_hdr(req, "Location", "http://" NET_AP_IP "/");
        return httpd_resp_send(req, NULL, 0);
    }
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Not found");
    return ESP_FAIL;
}

/* ---------------- WebSocket ---------------- */

static void ws_send_text(httpd_req_t *req, const char *txt)
{
    httpd_ws_frame_t f = { .type = HTTPD_WS_TYPE_TEXT, .payload = (uint8_t *)txt, .len = strlen(txt) };
    httpd_ws_send_frame(req, &f);
}

static void ws_error(httpd_req_t *req, const char *msg)
{
    cJSON *o = cJSON_CreateObject();
    cJSON_AddStringToObject(o, "type", "error");
    cJSON_AddStringToObject(o, "msg", msg);
    char *s = cJSON_PrintUnformatted(o);
    cJSON_Delete(o);
    if (s) {
        ws_send_text(req, s);
        cJSON_free(s);
    }
}

static int preset_from_json(const cJSON *id)
{
    static const char *names[MOTION_PRESET_COUNT] = { "sit", "stand", "custom1", "custom2" };
    if (cJSON_IsNumber(id)) {
        return id->valueint;
    }
    if (cJSON_IsString(id)) {
        for (int i = 0; i < MOTION_PRESET_COUNT; i++) {
            if (strcmp(id->valuestring, names[i]) == 0) {
                return i;
            }
        }
    }
    return -1;
}

static void ws_handle_cmd(httpd_req_t *req, const char *txt)
{
    cJSON *o = cJSON_Parse(txt);
    const cJSON *cmd = cJSON_GetObjectItem(o, "cmd");
    if (!cJSON_IsString(cmd)) {
        cJSON_Delete(o);
        return;
    }
    const char *c = cmd->valuestring;
    if (strcmp(c, "stop") == 0) {
        motion_stop(MOTION_SRC_WEB);              /* always accepted */
    } else if (strcmp(c, "hb") == 0) {
        motion_heartbeat(MOTION_SRC_WEB);
    } else if (strcmp(c, "jog") == 0 || strcmp(c, "preset") == 0) {
        if (!web_moves_allowed()) {
            ws_error(req, WEB_REMOTE_MOVE_ERR);
        } else if (c[0] == 'j') {
            const cJSON *dir = cJSON_GetObjectItem(o, "dir");
            if (cJSON_IsString(dir) && (strcmp(dir->valuestring, "up") == 0 || strcmp(dir->valuestring, "down") == 0)) {
                motion_jog(dir->valuestring[0] == 'u' ? MOTION_DIR_UP : MOTION_DIR_DOWN, MOTION_SRC_WEB);
            }
        } else {
            int p = preset_from_json(cJSON_GetObjectItem(o, "id"));
            if (p < 0 || p >= MOTION_PRESET_COUNT) {
                ws_error(req, "Unknown preset");
            } else {
                motion_goto_preset((motion_preset_t)p, MOTION_SRC_WEB);
            }
        }
    }
    cJSON_Delete(o);
}

/* Called by httpd once the WebSocket handshake is done (IDF 5.5 no longer calls
 * the URI handler for the handshake itself). */
static esp_err_t ws_post_handshake(httpd_req_t *req)
{
    int fd = httpd_req_to_sockfd(req);
    ESP_LOGI(TAG, "WebSocket client connected (fd %d)", fd);
    if (!ws_tracked(fd)) {
        ws_track(fd, true);
    }
    /* Send the current state right away. */
    cJSON *st = web_status_json();
    char *s = cJSON_PrintUnformatted(st);
    cJSON_Delete(st);
    if (s) {
        ws_send_text(req, s);
        cJSON_free(s);
    }
    return ESP_OK;
}

static esp_err_t ws_handler(httpd_req_t *req)
{
    if (req->method == HTTP_GET) {
        return ESP_OK;  /* older IDF behaviour; handshake work is in ws_post_handshake */
    }
    int fd = httpd_req_to_sockfd(req);
    if (!ws_tracked(fd)) {
        ws_track(fd, true);
    }
    httpd_ws_frame_t f = { 0 };
    esp_err_t err = httpd_ws_recv_frame(req, &f, 0);
    if (err != ESP_OK || f.len == 0 || f.len > 256 || f.type != HTTPD_WS_TYPE_TEXT) {
        return err;
    }
    char buf[257];
    f.payload = (uint8_t *)buf;
    err = httpd_ws_recv_frame(req, &f, f.len);
    if (err != ESP_OK) {
        return err;
    }
    buf[f.len] = 0;
    ws_handle_cmd(req, buf);
    return ESP_OK;
}

/* A socket closed. If a web jog was running, stop it now rather than waiting
 * for the heartbeat timeout. */
static void on_close(httpd_handle_t hd, int fd)
{
    if (ws_tracked(fd)) {
        ws_track(fd, false);
        motion_status_t st;
        motion_get_status(&st);
        if (st.state == MOTION_JOG && st.source == MOTION_SRC_WEB) {
            ESP_LOGW(TAG, "WebSocket closed during web jog: STOP");
            motion_stop(MOTION_SRC_WEB);
        }
    }
    close(fd);
}

/* ---------------- status push ---------------- */

static void push_work(void *arg)
{
    static int idle_ticks;
    static char *last;
    motion_status_t st;
    motion_get_status(&st);
    bool moving = st.state == MOTION_JOG || st.state == MOTION_SEEK || st.state == MOTION_SETTLE ||
                  st.state == MOTION_CALIBRATING;

    cJSON *o = web_status_json();
    char *s = cJSON_PrintUnformatted(o);
    cJSON_Delete(o);
    if (!s) {
        return;
    }
    bool changed = !last || strcmp(s, last) != 0;
    if (!moving && !changed && ++idle_ticks < IDLE_PUSH_TICKS) {
        cJSON_free(s);
        return;
    }
    idle_ticks = 0;

    size_t n = 10;
    int fds[10];
    if (httpd_get_client_list(s_server, &n, fds) == ESP_OK) {
        httpd_ws_frame_t f = { .type = HTTPD_WS_TYPE_TEXT, .payload = (uint8_t *)s, .len = strlen(s) };
        for (size_t i = 0; i < n; i++) {
            if (httpd_ws_get_fd_info(s_server, fds[i]) == HTTPD_WS_CLIENT_WEBSOCKET) {
                httpd_ws_send_frame_async(s_server, fds[i], &f);
            }
        }
    }
    if (last) {
        cJSON_free(last);
    }
    last = s;
}

static void push_timer_cb(void *arg)
{
    httpd_queue_work(s_server, push_work, NULL);
}

/* ---------------- start ---------------- */

esp_err_t web_start(void)
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_uri_handlers = 20;
    cfg.max_open_sockets = 10;
    cfg.lru_purge_enable = true;
    cfg.stack_size = 8192;
    cfg.close_fn = on_close;
    cfg.recv_wait_timeout = 10;
    cfg.send_wait_timeout = 10;
    esp_err_t err = httpd_start(&s_server, &cfg);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "httpd start failed: %s", esp_err_to_name(err));
        return err;
    }

    const httpd_uri_t index = { .uri = "/", .method = HTTP_GET, .handler = index_get };
    const httpd_uri_t ws = { .uri = "/ws", .method = HTTP_GET, .handler = ws_handler, .is_websocket = true,
                             .ws_post_handshake_cb = ws_post_handshake };
    httpd_register_uri_handler(s_server, &index);
    httpd_register_uri_handler(s_server, &ws);
    web_api_register(s_server);
    httpd_register_err_handler(s_server, HTTPD_404_NOT_FOUND, not_found);

    const esp_timer_create_args_t t = { .callback = push_timer_cb, .name = "ws_push" };
    esp_timer_create(&t, &s_push_timer);
    esp_timer_start_periodic(s_push_timer, PUSH_PERIOD_US);
    ESP_LOGI(TAG, "web server on port 80 (page %u bytes gzipped)", (unsigned)(index_gz_end - index_gz_start));
    return ESP_OK;
}
