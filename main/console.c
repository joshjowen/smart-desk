#include "console.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_console.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include <time.h>
#include "esp_sntp.h"
#include "board.h"
#include "motion.h"
#include "height.h"
#include "vl53l0x.h"

static const char *TAG = "console";

static int cmd_status(int argc, char **argv)
{
    motion_status_t s;
    motion_get_status(&s);
    printf("state=%s output=%s height=%s%ld target=%ld duty_remaining=%lus cooldown=%lus fault='%s' last_error='%s'\n",
           motion_state_name(s.state),
           s.output == MOTION_DIR_UP ? "UP" : s.output == MOTION_DIR_DOWN ? "DOWN" : "OFF",
           s.height_valid ? "" : "(invalid)", (long)s.height_mm, (long)s.target_mm,
           (unsigned long)s.duty_remaining_s, (unsigned long)s.cooldown_remaining_s, s.fault, s.last_error);
    printf("heap free=%u min=%u\n", (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
           (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL));
    return 0;
}

static int cmd_stop(int argc, char **argv)
{
    motion_stop(MOTION_SRC_SYSTEM);
    return 0;
}

static int cmd_goto(int argc, char **argv)
{
    if (argc != 2) {
        printf("usage: goto <mm>\n");
        return 1;
    }
    motion_req_t r = { .type = MOTION_REQ_GOTO_HEIGHT, .source = MOTION_SRC_SYSTEM, .value = atoi(argv[1]) };
    motion_request(&r);
    return 0;
}

static int cmd_time(int argc, char **argv)
{
    time_t now = time(NULL);
    printf("time %lld (%s)", (long long)now, ctime(&now));
    for (int i = 0; i < SNTP_MAX_SERVERS; i++) {
        const char *name = esp_sntp_getservername(i);
        const ip_addr_t *ip = esp_sntp_getserver(i);
        printf("server %d: %s ip=%s reach=0x%02x\n", i, name ? name : "-",
               ip ? ipaddr_ntoa(ip) : "-", esp_sntp_getreachability(i));
    }
    printf("sntp enabled=%d sync status=%d\n", esp_sntp_enabled(), sntp_get_sync_status());
    return 0;
}

static int cmd_sensor(int argc, char **argv)
{
    height_status_t h;
    height_get(&h);
    if (!h.online) {
        printf("sensor offline (next try in %lu ms), errors=%lu inits=%lu\n", (unsigned long)h.retry_in_ms,
               (unsigned long)h.errors, (unsigned long)h.inits);
        return 0;
    }
    printf("raw=%u mm status=%u (%s) filtered=%s%.1f mm signal=%.2f ambient=%.2f MCPS\n", h.raw_mm,
           h.range_status, vl53l0x_status_name(h.range_status), h.valid ? "" : "(invalid) ", h.filtered_mm,
           h.signal_mcps, h.ambient_mcps);
    printf("rate=%.1f Hz std2s raw=%.2f filt=%.2f span=%.1f mm | profile=%s (%u ms) alpha=%.2f samples=%lu errors=%lu "
           "inits=%lu touch_errors=%lu\n",
           h.rate_hz, h.std_raw_mm, h.std_filt_mm, h.span_filt_mm, h.profile ? "accurate" : "default", h.budget_ms,
           h.ema_pct / 100.0, (unsigned long)h.samples, (unsigned long)h.errors, (unsigned long)h.inits,
           (unsigned long)board_touch_errors());
    return 0;
}

static int cmd_scan(int argc, char **argv)
{
    board_i2c_scan();
    return 0;
}

#if CONFIG_DESK_DEBUG_COMMANDS
static int cmd_fault(int argc, char **argv)
{
    motion_debug_fault(argc > 1 ? argv[1] : "Test fault");
    return 0;
}

static int cmd_duty(int argc, char **argv)
{
    if (argc != 2) {
        printf("usage: duty <seconds> | duty clear\n");
        return 1;
    }
    motion_debug_use_duty(strcmp(argv[1], "clear") == 0 ? -1 : atoi(argv[1]));
    return 0;
}

static int cmd_crash(int argc, char **argv)
{
    ESP_LOGW(TAG, "deliberate crash requested");
    volatile int *p = NULL;
    *p = 1;
    return 0;
}
#endif

void console_start(void)
{
    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t cfg = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    cfg.prompt = "desk>";
    cfg.max_cmdline_length = 128;
    esp_console_dev_usb_serial_jtag_config_t dev = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    if (esp_console_new_repl_usb_serial_jtag(&dev, &cfg, &repl) != ESP_OK) {
        ESP_LOGE(TAG, "console start failed");
        return;
    }
    esp_console_register_help_command();
    const esp_console_cmd_t cmds[] = {
        { .command = "status", .help = "Show motion status", .func = cmd_status },
        { .command = "stop", .help = "Stop any movement", .func = cmd_stop },
        { .command = "goto", .help = "Seek to height: goto <mm>", .func = cmd_goto },
        { .command = "sensor", .help = "Show height sensor readings", .func = cmd_sensor },
        { .command = "scan", .help = "Scan the I2C bus", .func = cmd_scan },
        { .command = "time", .help = "Show clock and NTP server status", .func = cmd_time },
#if CONFIG_DESK_DEBUG_COMMANDS
        { .command = "fault", .help = "Debug: raise a fault: fault [text]", .func = cmd_fault },
        { .command = "duty", .help = "Debug: use duty budget: duty <s> | duty clear", .func = cmd_duty },
        { .command = "crash", .help = "Debug: crash (null write) to test reset behaviour", .func = cmd_crash },
#endif
    };
    for (size_t i = 0; i < sizeof(cmds) / sizeof(cmds[0]); i++) {
        esp_console_cmd_register(&cmds[i]);
    }
    esp_console_start_repl(repl);
}
