#pragma once

#include <stdbool.h>
#include "esp_http_server.h"
#include "cJSON.h"

#define WEB_REMOTE_MOVE_ERR "Unlock the desk display to control it from here, or turn on 'Allow remote moves' in Settings."

/* web_api.c */
esp_err_t web_api_register(httpd_handle_t h);
cJSON *web_status_json(void);
bool web_moves_allowed(void);
bool web_authorized(httpd_req_t *req);
esp_err_t web_send_json(httpd_req_t *req, int status, cJSON *obj);   /* takes ownership */
esp_err_t web_send_error(httpd_req_t *req, int status, const char *msg);
