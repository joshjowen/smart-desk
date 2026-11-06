#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_app_desc.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_ota_ops.h"
#include "esp_timer.h"
#include "config.h"
#include "net.h"
#include "web.h"
#include "board.h"
#include "motion.h"
#include "height.h"
#include "opto.h"
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

static void apply_presets(const desk_config_t *c)
{
    int32_t mm[MOTION_PRESET_COUNT];
    for (int i = 0; i < MOTION_PRESET_COUNT; i++) {
        mm[i] = c->presets[i].enabled ? c->presets[i].mm : 0;
    }
    motion_set_presets(mm);
}

static void on_config_changed(const desk_config_t *c)
{
    apply_presets(c);
    ui_apply_config(c);
    net_apply_config(c);
    height_set_params(c->sensor_profile, c->sensor_ema_pct);
}

static void mark_valid_cb(void *arg)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(run, &state) == ESP_OK && state == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_ota_mark_app_valid_cancel_rollback();
        ESP_LOGW(TAG, "new firmware in %s marked valid", run->label);
    }
}

#if CONFIG_DESK_DEBUG_BOOT_CRASH
static void boot_crash_cb(void *arg)
{
    ESP_LOGE(TAG, "DEBUG_BOOT_CRASH: crashing on purpose to test OTA rollback");
    abort();
}
#endif

void app_main(void)
{
    /* Invariant 1: the opto outputs are forced off before anything else runs. */
    opto_force_off();
    board_early_init();

    const esp_app_desc_t *app = esp_app_get_description();
    ESP_LOGI(TAG, "Desk controller %s (IDF %s), reset reason: %s", app->version, app->idf_ver,
             reset_reason_str(esp_reset_reason()));

    ESP_ERROR_CHECK(config_init());
    ESP_ERROR_CHECK(motion_init());

    desk_config_t cfg;
    config_get(&cfg);
    int32_t lo, hi;
    motion_get_range(&lo, &hi);
    config_set_height_range(lo, hi);
    apply_presets(&cfg);

    ESP_ERROR_CHECK(board_i2c_init());
    board_i2c_scan();
    /* Height sensor (stage 3: shown only, not used for control). A missing
     * sensor is retried in the background and never blocks boot. */
    if (height_init(cfg.sensor_profile, cfg.sensor_ema_pct) != ESP_OK) {
        ESP_LOGE(TAG, "height service failed to start");
    }

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
    ui_apply_config(&cfg);

    ESP_ERROR_CHECK(net_start(&cfg));
    ESP_ERROR_CHECK(web_start());
    config_add_listener(on_config_changed);

    /* OTA probation: mark this image good once it has run for a while. */
    esp_timer_handle_t t;
    const esp_timer_create_args_t ta = { .callback = mark_valid_cb, .name = "ota_ok" };
    esp_timer_create(&ta, &t);
    esp_timer_start_once(t, (uint64_t)CONFIG_DESK_OTA_VALID_AFTER_S * 1000000);
#if CONFIG_DESK_DEBUG_BOOT_CRASH
    esp_timer_handle_t c;
    const esp_timer_create_args_t ca = { .callback = boot_crash_cb, .name = "crash" };
    esp_timer_create(&ca, &c);
    esp_timer_start_once(c, 5 * 1000000);
#endif

    console_start();
}
