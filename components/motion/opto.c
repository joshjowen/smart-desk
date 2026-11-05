#include "opto.h"

#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "hal/gpio_ll.h"
#include "board_config.h"
#include "sdkconfig.h"

static const char *TAG = "opto";

#if CONFIG_DESK_OPTO_ACTIVE_LOW
#define LVL_ON   0
#define LVL_OFF  1
#else
#define LVL_ON   1
#define LVL_OFF  0
#endif

void opto_force_off(void)
{
    const gpio_num_t pins[2] = { BOARD_PIN_OPTO_UP, BOARD_PIN_OPTO_DOWN };
    for (int i = 0; i < 2; i++) {
        /* Set the output latch to off before enabling the driver, so the pin
         * never drives the on level, not even for an instant. */
        gpio_set_level(pins[i], LVL_OFF);
        const gpio_config_t c = {
            .pin_bit_mask = BIT64(pins[i]),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = LVL_OFF ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE,
            .pull_down_en = LVL_OFF ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&c);
        gpio_set_level(pins[i], LVL_OFF);
    }
}

void opto_set(motion_dir_t dir)
{
    gpio_set_level(BOARD_PIN_OPTO_UP, LVL_OFF);
    gpio_set_level(BOARD_PIN_OPTO_DOWN, LVL_OFF);
    if (dir == MOTION_DIR_UP) {
        gpio_set_level(BOARD_PIN_OPTO_UP, LVL_ON);
    } else if (dir == MOTION_DIR_DOWN) {
        gpio_set_level(BOARD_PIN_OPTO_DOWN, LVL_ON);
    }
}

void opto_log_config(void)
{
    ESP_LOGW(TAG, "opto outputs: UP=GPIO%d DOWN=GPIO%d, %s (on = %s), both forced OFF",
             BOARD_PIN_OPTO_UP, BOARD_PIN_OPTO_DOWN,
             LVL_ON ? "active-high / source mode" : "active-low / sink mode", LVL_ON ? "high" : "low");
}

/* Invariant 9: drop the outputs inside the panic handler, before the reset.
 * Covers crashes, aborts and the task/interrupt watchdogs (they all panic).
 * Linked with -Wl,--wrap=esp_panic_handler. Register writes only. */
void __real_esp_panic_handler(void *info);

void IRAM_ATTR __wrap_esp_panic_handler(void *info)
{
    gpio_ll_set_level(&GPIO, BOARD_PIN_OPTO_UP, LVL_OFF);
    gpio_ll_set_level(&GPIO, BOARD_PIN_OPTO_DOWN, LVL_OFF);
    __real_esp_panic_handler(info);
}
