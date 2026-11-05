/*
 * esp_lcd_touch driver for the AXS5106L.
 * Register layout from Waveshare's ESP32-C6-Touch-LCD-1.47 ESP-IDF demo.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <string.h>
#include <sys/cdefs.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "esp_lcd_touch_axs5106.h"

static const char *TAG = "axs5106";

#define REG_TOUCH_DATA  0x01  /* [0] gesture, [1] point count, then 6 bytes per point */
#define MAX_POINTS      2

typedef struct {
    esp_lcd_touch_t base;   /* must be first: freed via the base pointer */
    i2c_master_dev_handle_t dev;
} axs5106_t;

static esp_err_t read_data(esp_lcd_touch_handle_t tp)
{
    axs5106_t *t = __containerof(tp, axs5106_t, base);
    uint8_t reg = REG_TOUCH_DATA;
    uint8_t buf[2 + 6 * MAX_POINTS] = {0};

    /* Separate write and read (STOP between), as in Waveshare's demo. */
    ESP_RETURN_ON_ERROR(i2c_master_transmit(t->dev, &reg, 1, 50), TAG, "i2c write failed");
    ESP_RETURN_ON_ERROR(i2c_master_receive(t->dev, buf, sizeof(buf), 50), TAG, "i2c read failed");

    uint8_t points = buf[1] & 0x0F;
    if (points > MAX_POINTS) {
        points = MAX_POINTS;
    }

    portENTER_CRITICAL(&tp->data.lock);
    tp->data.points = points;
    for (int i = 0; i < points; i++) {
        const uint8_t *p = &buf[2 + i * 6];
        tp->data.coords[i].x = ((uint16_t)(p[0] & 0x0F) << 8) | p[1];
        tp->data.coords[i].y = ((uint16_t)(p[2] & 0x0F) << 8) | p[3];
        tp->data.coords[i].strength = 0;
    }
    portEXIT_CRITICAL(&tp->data.lock);
    return ESP_OK;
}

static bool get_xy(esp_lcd_touch_handle_t tp, uint16_t *x, uint16_t *y, uint16_t *strength,
                   uint8_t *point_num, uint8_t max_point_num)
{
    portENTER_CRITICAL(&tp->data.lock);
    *point_num = tp->data.points > max_point_num ? max_point_num : tp->data.points;
    for (size_t i = 0; i < *point_num; i++) {
        x[i] = tp->data.coords[i].x;
        y[i] = tp->data.coords[i].y;
        if (strength) {
            strength[i] = tp->data.coords[i].strength;
        }
    }
    tp->data.points = 0;
    portEXIT_CRITICAL(&tp->data.lock);
    return *point_num > 0;
}

static esp_err_t del(esp_lcd_touch_handle_t tp)
{
    if (tp->config.int_gpio_num != GPIO_NUM_NC) {
        gpio_reset_pin(tp->config.int_gpio_num);
    }
    if (tp->config.rst_gpio_num != GPIO_NUM_NC) {
        gpio_reset_pin(tp->config.rst_gpio_num);
    }
    free(__containerof(tp, axs5106_t, base));
    return ESP_OK;
}

esp_err_t esp_lcd_touch_new_i2c_axs5106(i2c_master_dev_handle_t dev,
                                        const esp_lcd_touch_config_t *config,
                                        esp_lcd_touch_handle_t *out_touch)
{
    ESP_RETURN_ON_FALSE(dev && config && out_touch, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    esp_err_t ret = ESP_OK;

    axs5106_t *t = heap_caps_calloc(1, sizeof(axs5106_t), MALLOC_CAP_DEFAULT);
    ESP_RETURN_ON_FALSE(t, ESP_ERR_NO_MEM, TAG, "no mem");
    t->dev = dev;
    esp_lcd_touch_handle_t tp = &t->base;
    tp->read_data = read_data;
    tp->get_xy = get_xy;
    tp->del = del;
    tp->data.lock.owner = portMUX_FREE_VAL;
    memcpy(&tp->config, config, sizeof(esp_lcd_touch_config_t));

    if (config->int_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t c = {
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_ENABLE,
            .pin_bit_mask = BIT64(config->int_gpio_num),
        };
        ESP_GOTO_ON_ERROR(gpio_config(&c), err, TAG, "int gpio config failed");
    }

    if (config->rst_gpio_num != GPIO_NUM_NC) {
        const gpio_config_t c = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = BIT64(config->rst_gpio_num),
        };
        ESP_GOTO_ON_ERROR(gpio_config(&c), err, TAG, "rst gpio config failed");
        gpio_set_level(config->rst_gpio_num, config->levels.reset);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(config->rst_gpio_num, !config->levels.reset);
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    *out_touch = tp;
    return ESP_OK;

err:
    free(t);
    return ret;
}
