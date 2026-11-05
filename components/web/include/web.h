#pragma once
#include "esp_err.h"

/* Starts the HTTP server on port 80 (page, REST API, WebSocket, OTA). */
esp_err_t web_start(void);
