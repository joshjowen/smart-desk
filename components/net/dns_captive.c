#include "dns_captive.h"

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "lwip/sockets.h"
#include "net.h"

static const char *TAG = "dns";
static volatile bool s_run;
static TaskHandle_t s_task;

static void dns_task(void *arg)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in addr = { .sin_family = AF_INET, .sin_port = htons(53), .sin_addr.s_addr = htonl(INADDR_ANY) };
    if (sock < 0 || bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        ESP_LOGE(TAG, "socket/bind failed");
        if (sock >= 0) close(sock);
        s_task = NULL;
        vTaskDelete(NULL);
    }
    struct timeval tv = { .tv_sec = 1 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    uint32_t ip = inet_addr(NET_AP_IP);
    ESP_LOGI(TAG, "captive DNS running");

    uint8_t buf[512];
    while (s_run) {
        struct sockaddr_in from;
        socklen_t fl = sizeof(from);
        int n = recvfrom(sock, buf, sizeof(buf) - 16, 0, (struct sockaddr *)&from, &fl);
        if (n < 12) {
            continue;
        }
        /* Only standard queries with one question. */
        if ((buf[2] & 0x80) || (buf[2] & 0x78) || buf[4] != 0 || buf[5] != 1) {
            continue;
        }
        /* Walk the question name. */
        int p = 12;
        while (p < n && buf[p] != 0) {
            p += buf[p] + 1;
        }
        if (p + 5 > n) {
            continue;
        }
        uint16_t qtype = (buf[p + 1] << 8) | buf[p + 2];
        int end = p + 5;
        buf[2] = 0x84 | (buf[2] & 0x01);   /* response, authoritative, keep RD */
        buf[3] = 0x80;                     /* RA, no error */
        buf[6] = 0; buf[7] = 0;            /* ancount */
        buf[8] = buf[9] = buf[10] = buf[11] = 0;
        if (qtype == 1) {
            buf[7] = 1;
            uint8_t ans[] = { 0xC0, 0x0C, 0, 1, 0, 1, 0, 0, 0, 30, 0, 4 };
            memcpy(buf + end, ans, sizeof(ans));
            memcpy(buf + end + sizeof(ans), &ip, 4);
            end += sizeof(ans) + 4;
        }
        sendto(sock, buf, end, 0, (struct sockaddr *)&from, fl);
    }
    close(sock);
    ESP_LOGI(TAG, "captive DNS stopped");
    s_task = NULL;
    vTaskDelete(NULL);
}

void dns_captive_start(void)
{
    if (s_task) {
        return;
    }
    s_run = true;
    xTaskCreate(dns_task, "dns", 3072, NULL, 3, &s_task);
}

void dns_captive_stop(void)
{
    s_run = false;
}
