#include "board.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "driver/ledc.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_lcd_jd9853.h"
#include "esp_lcd_touch_axs5106.h"

static const char *TAG = "board";

#define BL_LEDC_MODE    LEDC_LOW_SPEED_MODE
#define BL_LEDC_TIMER   LEDC_TIMER_0
#define BL_LEDC_CH      LEDC_CHANNEL_0
#define BL_LEDC_RES     LEDC_TIMER_10_BIT
#define BL_LEDC_MAX     ((1 << 10) - 1)
#define BL_LEDC_HZ      5000

static i2c_master_bus_handle_t s_i2c_bus;
static uint8_t s_backlight;

void board_early_init(void)
{
    /* TF card is never used. Hold its CS high so the card can never be selected
     * (GPIO3, the TF MISO line, is the opto UP output). */
    const gpio_config_t cs = {
        .pin_bit_mask = BIT64(BOARD_PIN_TF_CS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
    };
    gpio_set_level(BOARD_PIN_TF_CS, 1);
    gpio_config(&cs);
    gpio_set_level(BOARD_PIN_TF_CS, 1);
}

esp_err_t board_i2c_init(void)
{
    const i2c_master_bus_config_t cfg = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = BOARD_I2C_PORT,
        .scl_io_num = BOARD_PIN_I2C_SCL,
        .sda_io_num = BOARD_PIN_I2C_SDA,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    return i2c_new_master_bus(&cfg, &s_i2c_bus);
}

i2c_master_bus_handle_t board_i2c_bus(void)
{
    return s_i2c_bus;
}

void board_i2c_scan(void)
{
    char line[96];
    int n = 0, found = 0;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (i2c_master_probe(s_i2c_bus, addr, 20) == ESP_OK) {
            n += snprintf(line + n, sizeof(line) - n, " 0x%02x", addr);
            found++;
        }
    }
    ESP_LOGI(TAG, "I2C scan: %d device(s):%s", found, found ? line : " none");
}

esp_err_t board_display_init(esp_lcd_panel_io_handle_t *io, esp_lcd_panel_handle_t *panel, size_t max_transfer_bytes)
{
    /* Write-only bus: MISO is not routed so GPIO3 (TF MISO / opto UP) stays untouched. */
    const spi_bus_config_t bus = {
        .sclk_io_num = BOARD_PIN_LCD_SCK,
        .mosi_io_num = BOARD_PIN_LCD_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = max_transfer_bytes,
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(BOARD_LCD_SPI_HOST, &bus, SPI_DMA_CH_AUTO), TAG, "spi bus");

    const esp_lcd_panel_io_spi_config_t io_cfg = {
        .cs_gpio_num = BOARD_PIN_LCD_CS,
        .dc_gpio_num = BOARD_PIN_LCD_DC,
        .spi_mode = 0,
        .pclk_hz = BOARD_LCD_PCLK_HZ,
        .trans_queue_depth = 10,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)BOARD_LCD_SPI_HOST, &io_cfg, io), TAG, "panel io");

    const esp_lcd_panel_dev_config_t dev = {
        .reset_gpio_num = BOARD_PIN_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_jd9853(*io, &dev, panel), TAG, "panel");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(*panel), TAG, "panel reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(*panel), TAG, "panel init");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(*panel, true), TAG, "invert");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_mirror(*panel, false, false), TAG, "mirror");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_set_gap(*panel, BOARD_LCD_X_GAP, BOARD_LCD_Y_GAP), TAG, "gap");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(*panel, true), TAG, "disp on");
    ESP_LOGI(TAG, "JD9853 %dx%d ready (x gap %d)", BOARD_LCD_H_RES, BOARD_LCD_V_RES, BOARD_LCD_X_GAP);
    return ESP_OK;
}

esp_err_t board_touch_init(esp_lcd_touch_handle_t *touch)
{
    i2c_master_dev_handle_t dev;
    const i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BOARD_TP_I2C_ADDR,
        .scl_speed_hz = BOARD_I2C_HZ,
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(s_i2c_bus, &dev_cfg, &dev), TAG, "touch dev");

    const esp_lcd_touch_config_t cfg = {
        .x_max = BOARD_LCD_H_RES,
        .y_max = BOARD_LCD_V_RES,
        .rst_gpio_num = BOARD_PIN_TP_RST,
        .int_gpio_num = BOARD_PIN_TP_INT,
        .levels = { .reset = 0, .interrupt = 0 },
        .flags = { .swap_xy = 0, .mirror_x = 1, .mirror_y = 0 },  /* portrait, from Waveshare demo */
    };
    ESP_RETURN_ON_ERROR(esp_lcd_touch_new_i2c_axs5106(dev, &cfg, touch), TAG, "touch");
    ESP_LOGI(TAG, "AXS5106L touch ready");
    return ESP_OK;
}

esp_err_t board_backlight_init(void)
{
    const ledc_timer_config_t t = {
        .speed_mode = BL_LEDC_MODE,
        .timer_num = BL_LEDC_TIMER,
        .duty_resolution = BL_LEDC_RES,
        .freq_hz = BL_LEDC_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&t), TAG, "ledc timer");
    const ledc_channel_config_t c = {
        .speed_mode = BL_LEDC_MODE,
        .channel = BL_LEDC_CH,
        .timer_sel = BL_LEDC_TIMER,
        .gpio_num = BOARD_PIN_LCD_BL,
        .duty = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&c), TAG, "ledc channel");
    ESP_RETURN_ON_ERROR(ledc_fade_func_install(0), TAG, "ledc fade");
    return ESP_OK;
}

void board_backlight_set(uint8_t percent)
{
    if (percent > 100) {
        percent = 100;
    }
    s_backlight = percent;
    uint32_t duty = (uint32_t)percent * BL_LEDC_MAX / 100;
    ledc_set_fade_time_and_start(BL_LEDC_MODE, BL_LEDC_CH, duty, 150, LEDC_FADE_NO_WAIT);
}

uint8_t board_backlight_get(void)
{
    return s_backlight;
}
