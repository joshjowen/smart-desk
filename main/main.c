#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_system.h"
#include "board.h"
#include "motion.h"
#include "ui.h"
#include "console.h"

static const char *TAG = "main";

static const char *reset_reason_str(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON:  return "power-on";
    case ESP_RST_SW:       return "software";
    case ESP_RST_PANIC:    return "panic";
    case ESP_RST_INT_WDT:  return "interrupt watchdog";
    case ESP_RST_TASK_WDT: return "task watchdog";
    case ESP_RST_WDT:      return "other watchdog";
    case ESP_RST_BROWNOUT: return "brownout";
    case ESP_RST_USB:      return "USB";
    case ESP_RST_JTAG:     return "JTAG";
    default:               return "other";
    }
}

void app_main(void)
{
    /* Must stay the very first action. In stage 2 this also forces the opto
     * outputs off; in stage 0 the opto pins are deliberately left untouched. */
    board_early_init();

    const esp_app_desc_t *app = esp_app_get_description();
    ESP_LOGI(TAG, "Desk controller %s (IDF %s), reset reason: %s", app->version, app->idf_ver,
             reset_reason_str(esp_reset_reason()));

    ESP_ERROR_CHECK(motion_init());

    ESP_ERROR_CHECK(board_i2c_init());
    board_i2c_scan();

    ESP_ERROR_CHECK(board_backlight_init());
    board_backlight_set(0);

    esp_lcd_panel_io_handle_t io;
    esp_lcd_panel_handle_t panel;
    ESP_ERROR_CHECK(board_display_init(&io, &panel, BOARD_LCD_H_RES * 40 * 2));

    esp_lcd_touch_handle_t touch = NULL;
    if (board_touch_init(&touch) != ESP_OK) {
        ESP_LOGE(TAG, "touch init failed: continuing without touch");
        touch = NULL;
    }

    ESP_ERROR_CHECK(ui_init(io, panel, touch));
    console_start();
}
