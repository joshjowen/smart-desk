/*
 * esp_lcd panel driver for the JD9853.
 * Structure follows Espressif's esp_lcd panel drivers; the vendor init
 * sequence is from Waveshare's ESP32-C6-Touch-LCD-1.47 ESP-IDF demo.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdlib.h>
#include <sys/cdefs.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lcd_panel_interface.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_commands.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_lcd_jd9853.h"

static const char *TAG = "jd9853";

typedef struct {
    esp_lcd_panel_t base;
    esp_lcd_panel_io_handle_t io;
    int reset_gpio_num;
    bool reset_level;
    int x_gap;
    int y_gap;
    uint8_t fb_bits_per_pixel;
    uint8_t madctl_val;
    uint8_t colmod_val;
    const jd9853_lcd_init_cmd_t *init_cmds;
    uint16_t init_cmds_size;
} jd9853_panel_t;

static const jd9853_lcd_init_cmd_t s_init_default[] = {
    {0x11, NULL, 0, 120},
    {0xDF, (uint8_t[]){0x98, 0x53}, 2, 0},
    {0xDF, (uint8_t[]){0x98, 0x53}, 2, 0},
    {0xB2, (uint8_t[]){0x23}, 1, 0},
    {0xB7, (uint8_t[]){0x00, 0x47, 0x00, 0x6F}, 4, 0},
    {0xBB, (uint8_t[]){0x1C, 0x1A, 0x55, 0x73, 0x63, 0xF0}, 6, 0},
    {0xC0, (uint8_t[]){0x44, 0xA4}, 2, 0},
    {0xC1, (uint8_t[]){0x16}, 1, 0},
    {0xC3, (uint8_t[]){0x7D, 0x07, 0x14, 0x06, 0xCF, 0x71, 0x72, 0x77}, 8, 0},
    {0xC4, (uint8_t[]){0x00, 0x00, 0xA0, 0x79, 0x0B, 0x0A, 0x16, 0x79, 0x0B, 0x0A, 0x16, 0x82}, 12, 0},
    {0xC8, (uint8_t[]){0x3F, 0x32, 0x29, 0x29, 0x27, 0x2B, 0x27, 0x28, 0x28, 0x26, 0x25, 0x17, 0x12, 0x0D, 0x04, 0x00,
                       0x3F, 0x32, 0x29, 0x29, 0x27, 0x2B, 0x27, 0x28, 0x28, 0x26, 0x25, 0x17, 0x12, 0x0D, 0x04, 0x00}, 32, 0},
    {0xD0, (uint8_t[]){0x04, 0x06, 0x6B, 0x0F, 0x00}, 5, 0},
    {0xD7, (uint8_t[]){0x00, 0x30}, 2, 0},
    {0xE6, (uint8_t[]){0x14}, 1, 0},
    {0xDE, (uint8_t[]){0x01}, 1, 0},
    {0xB7, (uint8_t[]){0x03, 0x13, 0xEF, 0x35, 0x35}, 5, 0},
    {0xC1, (uint8_t[]){0x14, 0x15, 0xC0}, 3, 0},
    {0xC2, (uint8_t[]){0x06, 0x3A}, 2, 0},
    {0xC4, (uint8_t[]){0x72, 0x12}, 2, 0},
    {0xBE, (uint8_t[]){0x00}, 1, 0},
    {0xDE, (uint8_t[]){0x02}, 1, 0},
    {0xE5, (uint8_t[]){0x00, 0x02, 0x00}, 3, 0},
    {0xE5, (uint8_t[]){0x01, 0x02, 0x00}, 3, 0},
    {0xDE, (uint8_t[]){0x00}, 1, 0},
    {0x35, (uint8_t[]){0x00}, 1, 0},
    {0x3A, (uint8_t[]){0x05}, 1, 0},                   /* RGB565 */
    {0x2A, (uint8_t[]){0x00, 0x22, 0x00, 0xCD}, 4, 0}, /* columns 34..205 */
    {0x2B, (uint8_t[]){0x00, 0x00, 0x01, 0x3F}, 4, 0}, /* rows 0..319 */
    {0xDE, (uint8_t[]){0x02}, 1, 0},
    {0xE5, (uint8_t[]){0x00, 0x02, 0x00}, 3, 0},
    {0xDE, (uint8_t[]){0x00}, 1, 0},
    {0x29, NULL, 0, 0},
};

static esp_err_t tx(esp_lcd_panel_io_handle_t io, int cmd, const void *data, size_t len)
{
    return esp_lcd_panel_io_tx_param(io, cmd, data, len);
}

static esp_err_t panel_del(esp_lcd_panel_t *panel)
{
    jd9853_panel_t *p = __containerof(panel, jd9853_panel_t, base);
    if (p->reset_gpio_num >= 0) {
        gpio_reset_pin(p->reset_gpio_num);
    }
    free(p);
    return ESP_OK;
}

static esp_err_t panel_reset(esp_lcd_panel_t *panel)
{
    jd9853_panel_t *p = __containerof(panel, jd9853_panel_t, base);
    if (p->reset_gpio_num >= 0) {
        gpio_set_level(p->reset_gpio_num, p->reset_level);
        vTaskDelay(pdMS_TO_TICKS(10));
        gpio_set_level(p->reset_gpio_num, !p->reset_level);
        vTaskDelay(pdMS_TO_TICKS(120));
    } else {
        ESP_RETURN_ON_ERROR(tx(p->io, LCD_CMD_SWRESET, NULL, 0), TAG, "swreset failed");
        vTaskDelay(pdMS_TO_TICKS(120));
    }
    return ESP_OK;
}

static esp_err_t panel_init(esp_lcd_panel_t *panel)
{
    jd9853_panel_t *p = __containerof(panel, jd9853_panel_t, base);
    ESP_RETURN_ON_ERROR(tx(p->io, LCD_CMD_SLPOUT, NULL, 0), TAG, "slpout failed");
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_RETURN_ON_ERROR(tx(p->io, LCD_CMD_MADCTL, (uint8_t[]){p->madctl_val}, 1), TAG, "madctl failed");
    ESP_RETURN_ON_ERROR(tx(p->io, LCD_CMD_COLMOD, (uint8_t[]){p->colmod_val}, 1), TAG, "colmod failed");

    const jd9853_lcd_init_cmd_t *cmds = p->init_cmds ? p->init_cmds : s_init_default;
    uint16_t n = p->init_cmds ? p->init_cmds_size : sizeof(s_init_default) / sizeof(s_init_default[0]);
    for (int i = 0; i < n; i++) {
        if (cmds[i].cmd == LCD_CMD_MADCTL) {
            p->madctl_val = ((const uint8_t *)cmds[i].data)[0];
        } else if (cmds[i].cmd == LCD_CMD_COLMOD) {
            p->colmod_val = ((const uint8_t *)cmds[i].data)[0];
        }
        ESP_RETURN_ON_ERROR(tx(p->io, cmds[i].cmd, cmds[i].data, cmds[i].data_bytes), TAG, "init cmd 0x%02x failed", cmds[i].cmd);
        if (cmds[i].delay_ms) {
            vTaskDelay(pdMS_TO_TICKS(cmds[i].delay_ms));
        }
    }
    return ESP_OK;
}

static esp_err_t panel_draw_bitmap(esp_lcd_panel_t *panel, int x0, int y0, int x1, int y1, const void *color)
{
    jd9853_panel_t *p = __containerof(panel, jd9853_panel_t, base);
    assert(x0 < x1 && y0 < y1);
    x0 += p->x_gap; x1 += p->x_gap;
    y0 += p->y_gap; y1 += p->y_gap;
    ESP_RETURN_ON_ERROR(tx(p->io, LCD_CMD_CASET, (uint8_t[]){
        (x0 >> 8) & 0xFF, x0 & 0xFF, ((x1 - 1) >> 8) & 0xFF, (x1 - 1) & 0xFF}, 4), TAG, "caset failed");
    ESP_RETURN_ON_ERROR(tx(p->io, LCD_CMD_RASET, (uint8_t[]){
        (y0 >> 8) & 0xFF, y0 & 0xFF, ((y1 - 1) >> 8) & 0xFF, (y1 - 1) & 0xFF}, 4), TAG, "raset failed");
    size_t len = (size_t)(x1 - x0) * (y1 - y0) * p->fb_bits_per_pixel / 8;
    return esp_lcd_panel_io_tx_color(p->io, LCD_CMD_RAMWR, color, len);
}

static esp_err_t panel_invert_color(esp_lcd_panel_t *panel, bool invert)
{
    jd9853_panel_t *p = __containerof(panel, jd9853_panel_t, base);
    return tx(p->io, invert ? LCD_CMD_INVON : LCD_CMD_INVOFF, NULL, 0);
}

static esp_err_t panel_mirror(esp_lcd_panel_t *panel, bool mx, bool my)
{
    jd9853_panel_t *p = __containerof(panel, jd9853_panel_t, base);
    p->madctl_val = mx ? (p->madctl_val | LCD_CMD_MX_BIT) : (p->madctl_val & ~LCD_CMD_MX_BIT);
    p->madctl_val = my ? (p->madctl_val | LCD_CMD_MY_BIT) : (p->madctl_val & ~LCD_CMD_MY_BIT);
    return tx(p->io, LCD_CMD_MADCTL, (uint8_t[]){p->madctl_val}, 1);
}

static esp_err_t panel_swap_xy(esp_lcd_panel_t *panel, bool swap)
{
    jd9853_panel_t *p = __containerof(panel, jd9853_panel_t, base);
    p->madctl_val = swap ? (p->madctl_val | LCD_CMD_MV_BIT) : (p->madctl_val & ~LCD_CMD_MV_BIT);
    return tx(p->io, LCD_CMD_MADCTL, (uint8_t[]){p->madctl_val}, 1);
}

static esp_err_t panel_set_gap(esp_lcd_panel_t *panel, int x_gap, int y_gap)
{
    jd9853_panel_t *p = __containerof(panel, jd9853_panel_t, base);
    p->x_gap = x_gap;
    p->y_gap = y_gap;
    return ESP_OK;
}

static esp_err_t panel_disp_on_off(esp_lcd_panel_t *panel, bool on)
{
    jd9853_panel_t *p = __containerof(panel, jd9853_panel_t, base);
    return tx(p->io, on ? LCD_CMD_DISPON : LCD_CMD_DISPOFF, NULL, 0);
}

static esp_err_t panel_sleep(esp_lcd_panel_t *panel, bool sleep)
{
    jd9853_panel_t *p = __containerof(panel, jd9853_panel_t, base);
    ESP_RETURN_ON_ERROR(tx(p->io, sleep ? LCD_CMD_SLPIN : LCD_CMD_SLPOUT, NULL, 0), TAG, "sleep cmd failed");
    vTaskDelay(pdMS_TO_TICKS(120));
    return ESP_OK;
}

esp_err_t esp_lcd_new_panel_jd9853(const esp_lcd_panel_io_handle_t io,
                                   const esp_lcd_panel_dev_config_t *cfg,
                                   esp_lcd_panel_handle_t *ret_panel)
{
    ESP_RETURN_ON_FALSE(io && cfg && ret_panel, ESP_ERR_INVALID_ARG, TAG, "invalid argument");
    esp_err_t ret = ESP_OK;
    jd9853_panel_t *p = calloc(1, sizeof(jd9853_panel_t));
    ESP_RETURN_ON_FALSE(p, ESP_ERR_NO_MEM, TAG, "no mem");

    if (cfg->reset_gpio_num >= 0) {
        gpio_config_t io_conf = {
            .mode = GPIO_MODE_OUTPUT,
            .pin_bit_mask = 1ULL << cfg->reset_gpio_num,
        };
        ESP_GOTO_ON_ERROR(gpio_config(&io_conf), err, TAG, "rst gpio config failed");
    }

    switch (cfg->rgb_ele_order) {
    case LCD_RGB_ELEMENT_ORDER_RGB: p->madctl_val = 0; break;
    case LCD_RGB_ELEMENT_ORDER_BGR: p->madctl_val = LCD_CMD_BGR_BIT; break;
    default: ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported color order");
    }

    switch (cfg->bits_per_pixel) {
    case 16: p->colmod_val = 0x55; p->fb_bits_per_pixel = 16; break;
    case 18: p->colmod_val = 0x66; p->fb_bits_per_pixel = 24; break;
    default: ESP_GOTO_ON_FALSE(false, ESP_ERR_NOT_SUPPORTED, err, TAG, "unsupported pixel width");
    }

    p->io = io;
    p->reset_gpio_num = cfg->reset_gpio_num;
    p->reset_level = cfg->flags.reset_active_high;
    if (cfg->vendor_config) {
        const jd9853_vendor_config_t *vc = cfg->vendor_config;
        p->init_cmds = vc->init_cmds;
        p->init_cmds_size = vc->init_cmds_size;
    }
    p->base.del = panel_del;
    p->base.reset = panel_reset;
    p->base.init = panel_init;
    p->base.draw_bitmap = panel_draw_bitmap;
    p->base.invert_color = panel_invert_color;
    p->base.set_gap = panel_set_gap;
    p->base.mirror = panel_mirror;
    p->base.swap_xy = panel_swap_xy;
    p->base.disp_on_off = panel_disp_on_off;
    p->base.disp_sleep = panel_sleep;
    *ret_panel = &p->base;
    return ESP_OK;

err:
    if (cfg->reset_gpio_num >= 0) {
        gpio_reset_pin(cfg->reset_gpio_num);
    }
    free(p);
    return ret;
}
