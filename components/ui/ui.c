/*
 * Touch UI: lock screen, control screen and the SEEK / COOLDOWN / FAULT overlays.
 * The UI never drives the desk itself; it only sends motion requests.
 */
#include "ui.h"

#include <stdio.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include "sdkconfig.h"

#ifdef CONFIG_DESK_UI_UNITS_CM
#define UNITS_CM_DEFAULT true
#else
#define UNITS_CM_DEFAULT false
#endif
#include "board.h"
#include "motion.h"
#include "config.h"
#include "net.h"
#include "height.h"
#include "vl53l0x.h"

static const char *TAG = "ui";

#define DRAW_BUF_LINES      40
#define JOG_REPEAT_MS       100
#define UNLOCK_FRACTION     0.85f
#define TOAST_MS            2500
#define DEBUG_LOCKED_TIMEOUT_MS (5 * 60 * 1000)
#define SENSOR_REFRESH_MS   250

#define COL_BG       lv_color_hex(0x000000)
#define COL_PANEL    lv_color_hex(0x1C1C1E)
#define COL_PANEL_HI lv_color_hex(0x3A3A3C)
#define COL_TEXT     lv_color_hex(0xF2F2F7)
#define COL_DIM      lv_color_hex(0x8E8E93)
#define COL_OFF      lv_color_hex(0x48484A)
#define COL_ACCENT   lv_color_hex(0x0A84FF)
#define COL_SEEK     lv_color_hex(0xFF9F0A)
#define COL_FAULT    lv_color_hex(0xFF453A)
#define COL_COOL     lv_color_hex(0x64D2FF)

/* Display settings, updated from config by ui_apply_config(). */
static struct {
    bool units_cm;
    uint32_t autolock_ms;
    uint32_t dim_ms;
    uint8_t backlight;
    uint8_t backlight_dim;
    char preset_name[2][16];
} s_cfg = {
    .preset_name = { "Sit", "Stand" },
    .units_cm = UNITS_CM_DEFAULT,
    .autolock_ms = CONFIG_DESK_UI_AUTOLOCK_S * 1000,
    .dim_ms = CONFIG_DESK_UI_DIM_S * 1000,
    .backlight = CONFIG_DESK_UI_BACKLIGHT,
    .backlight_dim = CONFIG_DESK_UI_BACKLIGHT_DIM,
};

static lv_display_t *s_disp;
static lv_indev_t *s_indev;
static lv_indev_read_cb_t s_orig_read;

/* Activity / power */
static int64_t s_last_touch_ms;
static int64_t s_locked_at_ms;
static bool s_dimmed;
static bool s_swallow;        /* current touch was used to stop a move or wake: ignore until release */
static bool s_was_pressed;

/* Perf */
static uint32_t s_frames;

/* Widgets */
typedef struct {
    lv_obj_t *wifi, *net, *mqtt, *fault;
} status_row_t;

/* Sensor debug screen */
static lv_obj_t *s_scr_dbg, *s_dbg_big, *s_dbg_vals_l, *s_dbg_vals_r;
static bool s_debug_from_lock;
static lv_obj_t *s_lock_sensor, *s_ctrl_sensor;

/* Settings + calibration screens */
static lv_obj_t *s_scr_set, *s_set_info;
static lv_obj_t *s_scr_cal, *s_cal_prev, *s_cal_title, *s_cal_text, *s_cal_prog, *s_cal_value, *s_cal_kb;
static lv_obj_t *s_cal_b1, *s_cal_b1l, *s_cal_b2, *s_cal_b2l;
static motion_cal_cmd_t s_cal_b1_cmd, s_cal_b2_cmd;
static char s_cal_entry[6];
static motion_cal_phase_t s_cal_shown = MOTION_CAL_NONE;
static lv_obj_t *s_set_correct, *s_scr_num, *s_num_value;
static char s_num_entry[6];

static volatile bool s_locked = true;
static net_status_t s_net;
static bool s_have_net;

static lv_obj_t *s_scr_lock, *s_scr_ctrl;
static status_row_t s_lock_row, s_ctrl_row;
static lv_obj_t *s_lock_height, *s_lock_unit, *s_lock_info;
static lv_obj_t *s_track, *s_knob, *s_track_hint;
static lv_obj_t *s_ctrl_height, *s_ctrl_unit, *s_ctrl_info;
static lv_obj_t *s_btn_up, *s_btn_down, *s_btn_sit, *s_btn_stand;
static lv_obj_t *s_lbl_sit, *s_lbl_stand;
static lv_obj_t *s_ov_cool, *s_ov_cool_time;
static lv_obj_t *s_ov_fault, *s_ov_fault_text;
static lv_obj_t *s_toast;
static lv_timer_t *s_toast_timer;

static motion_status_t s_last;
static bool s_have_last;

static int64_t now_ms(void)
{
    return esp_timer_get_time() / 1000;
}

/* ---------------- helpers ---------------- */

static void fmt_height(char *buf, size_t n, bool valid, int32_t mm)
{
    if (!valid) {
        snprintf(buf, n, "--");
    } else if (s_cfg.units_cm) {
        snprintf(buf, n, "%ld.%ld", (long)(mm / 10), (long)(mm % 10));
    } else {
        snprintf(buf, n, "%ld", (long)mm);
    }
}

static const char *unit_str(void)
{
    return s_cfg.units_cm ? "cm" : "mm";
}

static lv_obj_t *make_screen(void)
{
    lv_obj_t *scr = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(scr, COL_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_set_style_text_color(scr, COL_TEXT, 0);
    lv_obj_set_style_pad_all(scr, 0, 0);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
    return scr;
}

static lv_obj_t *make_label(lv_obj_t *parent, const lv_font_t *font, lv_color_t color, const char *text)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, color, 0);
    lv_label_set_text(l, text);
    return l;
}

/* Status row: Wi-Fi icon + address on the left, MQTT / fault on the right
 * (`right_inset` leaves room for the Lock button on the control screen). */
static void show_debug(void);

static void status_row_long_cb(lv_event_t *e)
{
    show_debug();
}

static void make_status_row(lv_obj_t *scr, status_row_t *row, int right_inset)
{
    /* Invisible hit area: a long press on the status row opens the sensor debug screen. */
    lv_obj_t *hit = lv_obj_create(scr);
    lv_obj_remove_style_all(hit);
    lv_obj_set_size(hit, BOARD_LCD_H_RES - right_inset - (right_inset ? 8 : 0), 28);
    lv_obj_set_pos(hit, 0, 0);
    lv_obj_add_flag(hit, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(hit, status_row_long_cb, LV_EVENT_LONG_PRESSED, NULL);

    row->wifi = make_label(scr, &lv_font_montserrat_14, COL_OFF, LV_SYMBOL_WIFI);
    lv_obj_align(row->wifi, LV_ALIGN_TOP_LEFT, 10, 6);
    row->net = make_label(scr, &lv_font_montserrat_14, COL_DIM, "");
    lv_obj_align(row->net, LV_ALIGN_TOP_LEFT, 32, 6);
    row->mqtt = make_label(scr, &lv_font_montserrat_14, COL_OFF, "MQTT");
    lv_obj_align(row->mqtt, LV_ALIGN_TOP_RIGHT, -10 - right_inset, 6);
    row->fault = make_label(scr, &lv_font_montserrat_14, COL_FAULT, LV_SYMBOL_WARNING);
    lv_obj_align_to(row->fault, row->mqtt, LV_ALIGN_OUT_LEFT_MID, -8, 0);
    lv_obj_add_flag(row->fault, LV_OBJ_FLAG_HIDDEN);
}

static lv_obj_t *make_button(lv_obj_t *parent, int w, int h, lv_color_t bg)
{
    lv_obj_t *b = lv_button_create(parent);
    lv_obj_set_size(b, w, h);
    lv_obj_set_style_bg_color(b, bg, 0);
    lv_obj_set_style_bg_color(b, COL_PANEL_HI, LV_STATE_PRESSED);
    lv_obj_set_style_bg_color(b, COL_PANEL, LV_STATE_DISABLED);
    lv_obj_set_style_text_color(b, COL_OFF, LV_STATE_DISABLED);
    lv_obj_set_style_radius(b, 14, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_remove_flag(b, LV_OBJ_FLAG_SCROLL_ON_FOCUS);
    return b;
}

static void toast_hide_cb(lv_timer_t *t)
{
    lv_obj_add_flag(s_toast, LV_OBJ_FLAG_HIDDEN);
    lv_timer_pause(t);
}

static void toast(const char *text)
{
    lv_label_set_text(s_toast, text);
    lv_obj_remove_flag(s_toast, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(s_toast);
    lv_timer_reset(s_toast_timer);
    lv_timer_resume(s_toast_timer);
}

/* ---------------- lock / unlock / dim ---------------- */

static void set_dimmed(bool dim)
{
    if (dim == s_dimmed) {
        return;
    }
    s_dimmed = dim;
    board_backlight_set(dim ? s_cfg.backlight_dim : s_cfg.backlight);
    ESP_LOGI(TAG, "backlight %s", dim ? "dimmed" : "on");
}

static void knob_reset(bool animate);

static void show_lock(void)
{
    if (lv_screen_active() == s_scr_lock) {
        return;
    }
    knob_reset(false);
    lv_screen_load(s_scr_lock);
    s_locked = true;
    s_locked_at_ms = now_ms();
    ESP_LOGI(TAG, "locked");
}

static void show_control(void)
{
    lv_screen_load(s_scr_ctrl);
    s_locked = false;
    s_last_touch_ms = now_ms();
    ESP_LOGI(TAG, "unlocked");
}

/* ---------------- slide to unlock ---------------- */

static int32_t s_knob_x, s_knob_x0, s_press_x0;

static int32_t knob_travel(void)
{
    return lv_obj_get_content_width(s_track) - lv_obj_get_width(s_knob);
}

static void knob_set_x(void *obj, int32_t x)
{
    s_knob_x = x;
    lv_obj_set_x(obj, x);
    int32_t travel = knob_travel();
    lv_opa_t opa = travel > 0 ? (lv_opa_t)(LV_OPA_COVER - (LV_OPA_COVER * x / travel)) : LV_OPA_COVER;
    lv_obj_set_style_text_opa(s_track_hint, opa, 0);
}

static void knob_reset(bool animate)
{
    lv_anim_delete(s_knob, knob_set_x);
    if (!animate) {
        knob_set_x(s_knob, 0);
        return;
    }
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_knob);
    lv_anim_set_exec_cb(&a, knob_set_x);
    lv_anim_set_values(&a, s_knob_x, 0);
    lv_anim_set_duration(&a, 200);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_start(&a);
}

static void knob_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_point_t p;
    lv_indev_t *indev = lv_indev_active();
    if (indev) {
        lv_indev_get_point(indev, &p);
    }
    if (code == LV_EVENT_PRESSED) {
        lv_anim_delete(s_knob, knob_set_x);
        s_knob_x0 = s_knob_x;
        s_press_x0 = p.x;
    } else if (code == LV_EVENT_PRESSING) {
        int32_t x = s_knob_x0 + (p.x - s_press_x0);
        int32_t travel = knob_travel();
        knob_set_x(s_knob, x < 0 ? 0 : x > travel ? travel : x);
    } else if (code == LV_EVENT_RELEASED || code == LV_EVENT_PRESS_LOST) {
        if (s_knob_x >= (int32_t)(UNLOCK_FRACTION * knob_travel())) {
            show_control();
        } else {
            knob_reset(true);
        }
    }
}

/* ---------------- control buttons ---------------- */

static int64_t s_jog_sent_ms;

static void jog_event_cb(lv_event_t *e)
{
    motion_dir_t dir = (motion_dir_t)(intptr_t)lv_event_get_user_data(e);
    switch (lv_event_get_code(e)) {
    case LV_EVENT_PRESSED:
        motion_jog(dir, MOTION_SRC_TOUCH);
        s_jog_sent_ms = now_ms();
        break;
    case LV_EVENT_PRESSING:
        if (now_ms() - s_jog_sent_ms >= JOG_REPEAT_MS) {
            motion_heartbeat(MOTION_SRC_TOUCH);
            s_jog_sent_ms = now_ms();
        }
        break;
    case LV_EVENT_RELEASED:
    case LV_EVENT_PRESS_LOST:
        motion_stop(MOTION_SRC_TOUCH);
        break;
    default:
        break;
    }
}

static void preset_event_cb(lv_event_t *e)
{
    motion_preset_t p = (motion_preset_t)(intptr_t)lv_event_get_user_data(e);
    motion_goto_preset(p, MOTION_SRC_TOUCH);
}

static void lock_btn_cb(lv_event_t *e)
{
    show_lock();
}

static void settings_btn_cb(lv_event_t *e)
{
    lv_screen_load(s_scr_set);
    s_last_touch_ms = now_ms();
}

static void fault_ack_cb(lv_event_t *e)
{
    motion_req_t r = { .type = MOTION_REQ_CLEAR_FAULT, .source = MOTION_SRC_TOUCH };
    motion_request(&r);
}

/* ---------------- input wrapper ----------------
 * Runs on every touch read, before LVGL sees the data:
 *  - a new touch during an automatic move sends STOP (invariant 7) and is swallowed
 *  - the first touch on a dimmed screen only wakes it */
static void touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    s_orig_read(indev, data);
    bool pressed = data->state == LV_INDEV_STATE_PRESSED;

    if (pressed != s_was_pressed) {
        ESP_LOGD(TAG, "touch %s at %ld,%ld", pressed ? "down" : "up", (long)data->point.x, (long)data->point.y);
    }
    if (pressed && !s_was_pressed) {
        if (motion_is_automatic()) {
            motion_stop(MOTION_SRC_TOUCH);
            s_swallow = true;
            ESP_LOGI(TAG, "touch during automatic move: STOP");
        }
        if (s_dimmed) {
            set_dimmed(false);
            s_swallow = true;
        }
    }
    if (pressed) {
        s_last_touch_ms = now_ms();
    }
    s_was_pressed = pressed;

    if (s_swallow) {
        if (!pressed) {
            s_swallow = false;
        }
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

/* ---------------- screens ---------------- */

/* Landscape layout, 320 x 172. */

static void build_lock(void)
{
    s_scr_lock = make_screen();
    make_status_row(s_scr_lock, &s_lock_row, 0);

    s_lock_height = make_label(s_scr_lock, &lv_font_montserrat_48, COL_TEXT, "--");
    lv_obj_align(s_lock_height, LV_ALIGN_TOP_MID, -16, 30);
    s_lock_unit = make_label(s_scr_lock, &lv_font_montserrat_20, COL_DIM, unit_str());
    lv_obj_align_to(s_lock_unit, s_lock_height, LV_ALIGN_OUT_RIGHT_BOTTOM, 6, -8);
    s_lock_info = make_label(s_scr_lock, &lv_font_montserrat_14, COL_SEEK, "");
    lv_obj_set_width(s_lock_info, 300);
    lv_obj_set_style_text_align(s_lock_info, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_lock_info, LV_ALIGN_TOP_MID, 0, 92);
    s_lock_sensor = make_label(s_scr_lock, &lv_font_montserrat_14, COL_DIM, "");
    lv_obj_set_width(s_lock_sensor, 300);
    lv_obj_set_style_text_align(s_lock_sensor, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_lock_sensor, LV_ALIGN_TOP_MID, 0, 92);

    s_track = lv_obj_create(s_scr_lock);
    lv_obj_set_size(s_track, 288, 50);
    lv_obj_align(s_track, LV_ALIGN_BOTTOM_MID, 0, -10);
    lv_obj_set_style_radius(s_track, 25, 0);
    lv_obj_set_style_bg_color(s_track, COL_PANEL, 0);
    lv_obj_set_style_border_width(s_track, 0, 0);
    lv_obj_set_style_pad_all(s_track, 4, 0);
    lv_obj_remove_flag(s_track, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(s_track, LV_OBJ_FLAG_CLICKABLE);

    s_track_hint = make_label(s_track, &lv_font_montserrat_14, COL_DIM, "slide to unlock");
    lv_obj_align(s_track_hint, LV_ALIGN_CENTER, 20, 0);

    s_knob = lv_obj_create(s_track);
    lv_obj_set_size(s_knob, 56, 42);
    lv_obj_set_style_radius(s_knob, 21, 0);
    lv_obj_set_style_bg_color(s_knob, COL_ACCENT, 0);
    lv_obj_set_style_border_width(s_knob, 0, 0);
    lv_obj_remove_flag(s_knob, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(s_knob, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_set_align(s_knob, LV_ALIGN_LEFT_MID);
    /* Bigger hit area so the knob is easy to grab. */
    lv_obj_set_ext_click_area(s_knob, 8);
    lv_obj_t *arrow = make_label(s_knob, &lv_font_montserrat_20, COL_TEXT, LV_SYMBOL_RIGHT);
    lv_obj_center(arrow);
    lv_obj_add_event_cb(s_knob, knob_event_cb, LV_EVENT_ALL, NULL);
}

static lv_obj_t *make_preset(lv_obj_t *parent, const char *name, motion_preset_t id, lv_obj_t **lbl)
{
    lv_obj_t *b = make_button(parent, 74, 46, COL_PANEL);
    *lbl = make_label(b, &lv_font_montserrat_14, COL_TEXT, name);
    lv_obj_set_style_text_align(*lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(*lbl);
    lv_obj_add_event_cb(b, preset_event_cb, LV_EVENT_CLICKED, (void *)(intptr_t)id);
    return b;
}

static void build_control(void)
{
    s_scr_ctrl = make_screen();
    make_status_row(s_scr_ctrl, &s_ctrl_row, 104);

    lv_obj_t *gear = make_button(s_scr_ctrl, 34, 26, COL_PANEL);
    lv_obj_set_style_radius(gear, 13, 0);
    lv_obj_align(gear, LV_ALIGN_TOP_RIGHT, -70, 2);
    lv_obj_t *gl = make_label(gear, &lv_font_montserrat_14, COL_DIM, LV_SYMBOL_SETTINGS);
    lv_obj_center(gl);
    lv_obj_add_event_cb(gear, settings_btn_cb, LV_EVENT_CLICKED, NULL);

    lv_obj_t *lock = make_button(s_scr_ctrl, 56, 26, COL_PANEL);
    lv_obj_set_style_radius(lock, 13, 0);
    lv_obj_align(lock, LV_ALIGN_TOP_RIGHT, -8, 2);
    lv_obj_t *ll = make_label(lock, &lv_font_montserrat_14, COL_DIM, "Lock");
    lv_obj_center(ll);
    lv_obj_add_event_cb(lock, lock_btn_cb, LV_EVENT_CLICKED, NULL);

    /* Left column (x 0..160): height, info, presets */
    s_ctrl_height = make_label(s_scr_ctrl, &lv_font_montserrat_48, COL_TEXT, "--");
    lv_obj_align(s_ctrl_height, LV_ALIGN_TOP_LEFT, 10, 26);
    s_ctrl_unit = make_label(s_scr_ctrl, &lv_font_montserrat_14, COL_DIM, unit_str());
    lv_obj_align_to(s_ctrl_unit, s_ctrl_height, LV_ALIGN_OUT_RIGHT_BOTTOM, 4, -8);
    s_ctrl_info = make_label(s_scr_ctrl, &lv_font_montserrat_14, COL_SEEK, "");
    lv_obj_set_width(s_ctrl_info, 152);
    lv_obj_align(s_ctrl_info, LV_ALIGN_TOP_LEFT, 10, 82);
    s_ctrl_sensor = make_label(s_scr_ctrl, &lv_font_montserrat_14, COL_DIM, "");
    lv_obj_set_width(s_ctrl_sensor, 152);
    lv_obj_align(s_ctrl_sensor, LV_ALIGN_TOP_LEFT, 10, 82);

    s_btn_sit = make_preset(s_scr_ctrl, "SIT", MOTION_PRESET_SIT, &s_lbl_sit);
    lv_obj_align(s_btn_sit, LV_ALIGN_BOTTOM_LEFT, 8, -8);
    s_btn_stand = make_preset(s_scr_ctrl, "STAND", MOTION_PRESET_STAND, &s_lbl_stand);
    lv_obj_align(s_btn_stand, LV_ALIGN_BOTTOM_LEFT, 88, -8);

    /* Right column: hold-to-run */
    s_btn_up = make_button(s_scr_ctrl, 144, 62, COL_PANEL);
    lv_obj_align(s_btn_up, LV_ALIGN_TOP_RIGHT, -8, 34);
    lv_obj_t *lu = make_label(s_btn_up, &lv_font_montserrat_28, COL_TEXT, LV_SYMBOL_UP);
    lv_obj_center(lu);
    lv_obj_add_event_cb(s_btn_up, jog_event_cb, LV_EVENT_ALL, (void *)(intptr_t)MOTION_DIR_UP);

    s_btn_down = make_button(s_scr_ctrl, 144, 62, COL_PANEL);
    lv_obj_align(s_btn_down, LV_ALIGN_BOTTOM_RIGHT, -8, -8);
    lv_obj_t *ld = make_label(s_btn_down, &lv_font_montserrat_28, COL_TEXT, LV_SYMBOL_DOWN);
    lv_obj_center(ld);
    lv_obj_add_event_cb(s_btn_down, jog_event_cb, LV_EVENT_ALL, (void *)(intptr_t)MOTION_DIR_DOWN);
}

static void debug_back_cb(lv_event_t *e)
{
    if (s_debug_from_lock) {
        show_lock();
    } else {
        lv_screen_load(s_scr_ctrl);
        s_last_touch_ms = now_ms();
    }
}

static void build_debug(void)
{
    s_scr_dbg = make_screen();
    lv_obj_t *t = make_label(s_scr_dbg, &lv_font_montserrat_14, COL_DIM, "Height sensor (raw, uncalibrated)");
    lv_obj_align(t, LV_ALIGN_TOP_LEFT, 10, 6);

    lv_obj_t *back = make_button(s_scr_dbg, 56, 26, COL_PANEL);
    lv_obj_set_style_radius(back, 13, 0);
    lv_obj_align(back, LV_ALIGN_TOP_RIGHT, -8, 2);
    lv_obj_t *bl = make_label(back, &lv_font_montserrat_14, COL_DIM, "Back");
    lv_obj_center(bl);
    lv_obj_add_event_cb(back, debug_back_cb, LV_EVENT_CLICKED, NULL);

    s_dbg_big = make_label(s_scr_dbg, &lv_font_montserrat_28, COL_TEXT, "--");
    lv_obj_align(s_dbg_big, LV_ALIGN_TOP_LEFT, 10, 32);

    static const char *keys_l = "Raw\nStatus\nRate\nStd 2 s\nSpan 2 s";
    static const char *keys_r = "Signal\nAmbient\nProfile\nAlpha\nErrors";
    lv_obj_t *kl = make_label(s_scr_dbg, &lv_font_montserrat_14, COL_DIM, keys_l);
    lv_obj_align(kl, LV_ALIGN_TOP_LEFT, 10, 72);
    s_dbg_vals_l = make_label(s_scr_dbg, &lv_font_montserrat_14, COL_TEXT, "");
    lv_obj_align(s_dbg_vals_l, LV_ALIGN_TOP_LEFT, 74, 72);
    lv_obj_t *kr = make_label(s_scr_dbg, &lv_font_montserrat_14, COL_DIM, keys_r);
    lv_obj_align(kr, LV_ALIGN_TOP_LEFT, 176, 72);
    s_dbg_vals_r = make_label(s_scr_dbg, &lv_font_montserrat_14, COL_TEXT, "");
    lv_obj_align(s_dbg_vals_r, LV_ALIGN_TOP_LEFT, 240, 72);
}

static void show_debug(void)
{
    lv_obj_t *cur = lv_screen_active();
    if (cur == s_scr_dbg) {
        return;
    }
    s_debug_from_lock = cur == s_scr_lock;
    lv_screen_load(s_scr_dbg);
    s_last_touch_ms = now_ms();
    ESP_LOGI(TAG, "sensor debug screen");
}

static void fmt_opt(char *b, size_t n, float v, const char *unit)
{
    if (isnan(v)) {
        snprintf(b, n, "--");
    } else {
        snprintf(b, n, "%.1f%s", v, unit);
    }
}

static void render_sensor(void)
{
    height_status_t h;
    height_get(&h);

    /* One-line summary on the lock / control screens, where the motion info line is free. */
    char line[48];
    lv_color_t c = COL_DIM;
    if (!h.online) {
        snprintf(line, sizeof(line), "sensor offline");
        c = COL_SEEK;
    } else if (!h.valid) {
        snprintf(line, sizeof(line), "sensor: no reading (%s)", vl53l0x_status_name(h.range_status));
        c = COL_SEEK;
    } else if (s_have_last && s_last.calibrated) {
        line[0] = 0;   /* the big number is the real height now */
    } else {
        snprintf(line, sizeof(line), "sensor %.0f mm (uncalibrated)", h.filtered_mm);
    }
    lv_obj_t *pairs[2][2] = { { s_lock_sensor, s_lock_info }, { s_ctrl_sensor, s_ctrl_info } };
    for (int i = 0; i < 2; i++) {
        lv_label_set_text(pairs[i][0], line);
        lv_obj_set_style_text_color(pairs[i][0], c, 0);
        const char *info = lv_label_get_text(pairs[i][1]);
        if (info && info[0]) {
            lv_obj_add_flag(pairs[i][0], LV_OBJ_FLAG_HIDDEN);
        } else {
            lv_obj_remove_flag(pairs[i][0], LV_OBJ_FLAG_HIDDEN);
        }
    }

    if (lv_screen_active() != s_scr_dbg) {
        return;
    }
    char big[32], l[128], r[128], sr[12], sf[12], sp[12];
    if (!h.online) {
        snprintf(big, sizeof(big), "sensor offline");
        lv_obj_set_style_text_color(s_dbg_big, COL_SEEK, 0);
    } else if (!h.valid) {
        snprintf(big, sizeof(big), "-- mm");
        lv_obj_set_style_text_color(s_dbg_big, COL_SEEK, 0);
    } else {
        snprintf(big, sizeof(big), "%.1f mm", h.filtered_mm);
        lv_obj_set_style_text_color(s_dbg_big, COL_TEXT, 0);
    }
    lv_label_set_text(s_dbg_big, big);
    fmt_opt(sr, sizeof(sr), h.std_raw_mm, "");
    fmt_opt(sf, sizeof(sf), h.std_filt_mm, "");
    fmt_opt(sp, sizeof(sp), h.span_filt_mm, " mm");
    if (h.online) {
        snprintf(l, sizeof(l), "%u mm\n%u %s\n%.1f Hz\n%s / %s\n%s", h.raw_mm, h.range_status,
                 vl53l0x_status_name(h.range_status), h.rate_hz, sr, sf, sp);
    } else {
        snprintf(l, sizeof(l), "--\n--\n--\n--\nretry %.1f s", h.retry_in_ms / 1000.0f);
    }
    snprintf(r, sizeof(r), "%.2f MCPS\n%.2f MCPS\n%u ms\n%.2f\n%lu (%lu init)", h.signal_mcps, h.ambient_mcps,
             h.budget_ms, h.ema_pct / 100.0f, (unsigned long)h.errors, (unsigned long)h.inits);
    lv_label_set_text(s_dbg_vals_l, l);
    lv_label_set_text(s_dbg_vals_r, r);
}

/* ---------------- settings / calibration ---------------- */

static void set_back_cb(lv_event_t *e)
{
    lv_screen_load(s_scr_ctrl);
    s_last_touch_ms = now_ms();
}

static void set_cal_cb(lv_event_t *e)
{
    motion_cal_cmd(MOTION_CAL_BEGIN, MOTION_SRC_TOUCH);
}

static void set_sensor_cb(lv_event_t *e)
{
    show_debug();
}

static lv_obj_t *make_text_button(lv_obj_t *parent, int w, int h, lv_color_t bg, const char *text, lv_obj_t **lbl)
{
    lv_obj_t *b = make_button(parent, w, h, bg);
    lv_obj_t *l = make_label(b, &lv_font_montserrat_14, COL_TEXT, text);
    lv_obj_center(l);
    if (lbl) *lbl = l;
    return b;
}

static void num_show(void)
{
    char v[24];
    snprintf(v, sizeof(v), "%s mm", s_num_entry[0] ? s_num_entry : "____");
    lv_label_set_text(s_num_value, v);
}

static void set_correct_cb(lv_event_t *e)
{
    s_num_entry[0] = 0;
    num_show();
    lv_screen_load(s_scr_num);
    s_last_touch_ms = now_ms();
}

static void num_kb_cb(lv_event_t *e)
{
    lv_obj_t *kb = lv_event_get_target(e);
    const char *txt = lv_buttonmatrix_get_button_text(kb, lv_buttonmatrix_get_selected_button(kb));
    if (!txt) return;
    size_t n = strlen(s_num_entry);
    if (strcmp(txt, "OK") == 0 || strcmp(txt, "Cancel") == 0) {
        if (txt[0] == 'O' && n) {
            motion_req_t r = { .type = MOTION_REQ_CAL_CORRECT, .source = MOTION_SRC_TOUCH, .value = atoi(s_num_entry) };
            motion_request(&r);
            toast("Height corrected");
        }
        lv_screen_load(s_scr_set);
        s_last_touch_ms = now_ms();
        return;
    } else if (strcmp(txt, LV_SYMBOL_BACKSPACE) == 0) {
        if (n) s_num_entry[n - 1] = 0;
    } else if (n < 4 && txt[0] >= '0' && txt[0] <= '9') {
        s_num_entry[n] = txt[0];
        s_num_entry[n + 1] = 0;
    }
    num_show();
}

static lv_obj_t *make_keypad(lv_obj_t *parent, lv_event_cb_t cb)
{
    static const char *map[] = { "1", "2", "3", "4", "5", "6", "7", "\n",
                                 "8", "9", "0", LV_SYMBOL_BACKSPACE, "Cancel", "OK", "" };
    lv_obj_t *kb = lv_buttonmatrix_create(parent);
    lv_buttonmatrix_set_map(kb, map);
    lv_obj_set_size(kb, 308, 84);
    lv_obj_align(kb, LV_ALIGN_BOTTOM_MID, 0, -4);
    lv_obj_set_style_bg_opa(kb, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(kb, 0, 0);
    lv_obj_set_style_pad_all(kb, 2, 0);
    lv_obj_set_style_pad_gap(kb, 4, 0);
    lv_obj_set_style_bg_color(kb, COL_PANEL, LV_PART_ITEMS);
    lv_obj_set_style_text_color(kb, COL_TEXT, LV_PART_ITEMS);
    lv_obj_set_style_text_font(kb, &lv_font_montserrat_14, LV_PART_ITEMS);
    lv_obj_add_event_cb(kb, cb, LV_EVENT_VALUE_CHANGED, NULL);
    return kb;
}

static void build_number_entry(void)
{
    s_scr_num = make_screen();
    lv_obj_t *t = make_label(s_scr_num, &lv_font_montserrat_14, COL_TEXT,
                             "Measure floor to desktop now and enter it. Higher up corrects the scale, at the bottom the offset.");
    lv_obj_set_width(t, 300);
    lv_label_set_long_mode(t, LV_LABEL_LONG_WRAP);
    lv_obj_align(t, LV_ALIGN_TOP_LEFT, 10, 4);
    s_num_value = make_label(s_scr_num, &lv_font_montserrat_28, COL_ACCENT, "");
    lv_obj_align(s_num_value, LV_ALIGN_TOP_RIGHT, -10, 46);
    make_keypad(s_scr_num, num_kb_cb);
}

static void build_settings(void)
{
    s_scr_set = make_screen();
    lv_obj_t *t = make_label(s_scr_set, &lv_font_montserrat_20, COL_TEXT, "Settings");
    lv_obj_align(t, LV_ALIGN_TOP_LEFT, 10, 4);
    lv_obj_t *back = make_text_button(s_scr_set, 56, 26, COL_PANEL, "Back", NULL);
    lv_obj_set_style_radius(back, 13, 0);
    lv_obj_align(back, LV_ALIGN_TOP_RIGHT, -8, 2);
    lv_obj_add_event_cb(back, set_back_cb, LV_EVENT_CLICKED, NULL);

    s_set_info = make_label(s_scr_set, &lv_font_montserrat_14, COL_DIM, "");
    lv_obj_set_width(s_set_info, 300);
    lv_label_set_long_mode(s_set_info, LV_LABEL_LONG_WRAP);
    lv_obj_align(s_set_info, LV_ALIGN_TOP_LEFT, 10, 36);

    lv_obj_t *cal = make_text_button(s_scr_set, 98, 46, COL_ACCENT, "Calibrate", NULL);
    lv_obj_align(cal, LV_ALIGN_BOTTOM_LEFT, 8, -8);
    lv_obj_add_event_cb(cal, set_cal_cb, LV_EVENT_CLICKED, NULL);
    s_set_correct = make_text_button(s_scr_set, 98, 46, COL_PANEL, "Correct\nheight", NULL);
    lv_obj_align(s_set_correct, LV_ALIGN_BOTTOM_MID, 0, -8);
    lv_obj_add_event_cb(s_set_correct, set_correct_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_t *sen = make_text_button(s_scr_set, 98, 46, COL_PANEL, "Sensor", NULL);
    lv_obj_align(sen, LV_ALIGN_BOTTOM_RIGHT, -8, -8);
    lv_obj_add_event_cb(sen, set_sensor_cb, LV_EVENT_CLICKED, NULL);
}

static void render_settings_info(const motion_status_t *st)
{
    char t[200];
    if (st->calibrated) {
        float k = st->cal.scale;
        int32_t h_max = (int32_t)lroundf(st->cal.h_min_mm + k * st->cal.sign * (st->cal.raw_max - st->cal.raw_min));
        char when[24] = "";
        if (st->cal.timestamp) {
            time_t ts = (time_t)st->cal.timestamp;
            struct tm tm;
            localtime_r(&ts, &tm);
            strftime(when, sizeof(when), " on %Y-%m-%d", &tm);
        }
        snprintf(t, sizeof(t), "Calibrated%s: %ld-%ld mm.\nSpeed up %.1f / down %.1f mm/s, coast up %.0f / down %.0f mm.",
                 when, (long)st->cal.h_min_mm, (long)h_max, k * st->cal.speed_up, k * st->cal.speed_down,
                 k * st->cal.coast_up, k * st->cal.coast_down);
    } else {
        snprintf(t, sizeof(t), "Not calibrated. Calibration finds the end stops and maps the sensor to desk height "
                               "(needs about %u s of motor time).", (unsigned)st->cal_session.need_s);
    }
    lv_label_set_text(s_set_info, t);
    lv_obj_set_state(s_set_correct, LV_STATE_DISABLED, !st->calibrated);
}

static void cal_btn_cb(lv_event_t *e)
{
    const motion_cal_cmd_t *cmd = lv_event_get_user_data(e);
    motion_cal_cmd(*cmd, MOTION_SRC_TOUCH);
}

static void cal_entry_show(void)
{
    char v[24];
    snprintf(v, sizeof(v), "%s mm", s_cal_entry[0] ? s_cal_entry : "____");
    lv_label_set_text(s_cal_value, v);
}

static void cal_kb_cb(lv_event_t *e)
{
    lv_obj_t *kb = lv_event_get_target(e);
    const char *txt = lv_buttonmatrix_get_button_text(kb, lv_buttonmatrix_get_selected_button(kb));
    if (!txt) return;
    size_t n = strlen(s_cal_entry);
    if (strcmp(txt, "OK") == 0) {
        if (n) {
            motion_req_t r = { .type = MOTION_REQ_CAL_HEIGHT, .source = MOTION_SRC_TOUCH, .value = atoi(s_cal_entry) };
            motion_request(&r);
        }
    } else if (strcmp(txt, "Cancel") == 0) {
        motion_cal_cmd(MOTION_CAL_DISCARD, MOTION_SRC_TOUCH);
    } else if (strcmp(txt, LV_SYMBOL_BACKSPACE) == 0) {
        if (n) s_cal_entry[n - 1] = 0;
    } else if (n < 4 && txt[0] >= '0' && txt[0] <= '9') {
        s_cal_entry[n] = txt[0];
        s_cal_entry[n + 1] = 0;
    }
    cal_entry_show();
}

static void build_calibration(void)
{
    s_scr_cal = make_screen();
    s_cal_title = make_label(s_scr_cal, &lv_font_montserrat_20, COL_TEXT, "Calibration");
    lv_obj_align(s_cal_title, LV_ALIGN_TOP_LEFT, 10, 4);
    s_cal_text = make_label(s_scr_cal, &lv_font_montserrat_14, COL_TEXT, "");
    lv_obj_set_width(s_cal_text, 300);
    lv_label_set_long_mode(s_cal_text, LV_LABEL_LONG_WRAP);
    lv_obj_align(s_cal_text, LV_ALIGN_TOP_LEFT, 10, 32);
    s_cal_prog = make_label(s_scr_cal, &lv_font_montserrat_14, COL_DIM, "");
    lv_obj_align(s_cal_prog, LV_ALIGN_TOP_LEFT, 10, 96);
    s_cal_value = make_label(s_scr_cal, &lv_font_montserrat_28, COL_ACCENT, "");
    lv_obj_align(s_cal_value, LV_ALIGN_TOP_RIGHT, -10, 4);

    s_cal_kb = make_keypad(s_scr_cal, cal_kb_cb);

    s_cal_b1 = make_text_button(s_scr_cal, 146, 46, COL_ACCENT, "", &s_cal_b1l);
    lv_obj_align(s_cal_b1, LV_ALIGN_BOTTOM_LEFT, 8, -8);
    lv_obj_add_event_cb(s_cal_b1, cal_btn_cb, LV_EVENT_CLICKED, (void *)&s_cal_b1_cmd);
    s_cal_b2 = make_text_button(s_scr_cal, 146, 46, COL_PANEL, "", &s_cal_b2l);
    lv_obj_align(s_cal_b2, LV_ALIGN_BOTTOM_RIGHT, -8, -8);
    lv_obj_add_event_cb(s_cal_b2, cal_btn_cb, LV_EVENT_CLICKED, (void *)&s_cal_b2_cmd);
}

static void cal_buttons(const char *b1, motion_cal_cmd_t c1, const char *b2, motion_cal_cmd_t c2)
{
    s_cal_b1_cmd = c1;
    s_cal_b2_cmd = c2;
    lv_obj_set_flag(s_cal_b1, LV_OBJ_FLAG_HIDDEN, b1 == NULL);
    lv_obj_set_flag(s_cal_b2, LV_OBJ_FLAG_HIDDEN, b2 == NULL);
    if (b1) lv_label_set_text(s_cal_b1l, b1);
    if (b2) lv_label_set_text(s_cal_b2l, b2);
}

static void render_calibration(const motion_status_t *st)
{
    const motion_cal_status_t *c = &st->cal_session;
    lv_obj_t *cur = lv_screen_active();
    if (c->phase == MOTION_CAL_NONE) {
        if (cur == s_scr_cal) {
            lv_screen_load(s_cal_prev ? s_cal_prev : s_scr_ctrl);
            s_last_touch_ms = now_ms();
        }
        s_cal_shown = MOTION_CAL_NONE;
        return;
    }
    if (cur != s_scr_cal) {
        s_cal_prev = cur == s_scr_lock ? s_scr_lock : cur == s_scr_set ? s_scr_set : s_scr_ctrl;
        set_dimmed(false);
        lv_screen_load(s_scr_cal);
    }
    if (c->phase != s_cal_shown && c->phase == MOTION_CAL_ENTER_HEIGHT) {
        s_cal_entry[0] = 0;
    }
    s_cal_shown = c->phase;

    char text[200] = "", prog[64] = "";
    const char *title = "Calibration";
    lv_color_t tc = COL_TEXT;
    bool entry = c->phase == MOTION_CAL_ENTER_HEIGHT;
    lv_obj_set_flag(s_cal_kb, LV_OBJ_FLAG_HIDDEN, !entry);
    lv_obj_set_flag(s_cal_value, LV_OBJ_FLAG_HIDDEN, !entry);

    switch (c->phase) {
    case MOTION_CAL_NEED_BUDGET:
        snprintf(text, sizeof(text), "%s", c->text);
        snprintf(prog, sizeof(prog), "Ready in %lu:%02lu", (unsigned long)(c->wait_s / 60), (unsigned long)(c->wait_s % 60));
        cal_buttons(NULL, 0, "Close", MOTION_CAL_CLOSE);
        break;
    case MOTION_CAL_PROMPT:
        snprintf(text, sizeof(text), "%s\nThe desk will move to the top and bottom on its own. Touch anywhere to stop it.", c->text);
        cal_buttons("Start", MOTION_CAL_START, "Cancel", MOTION_CAL_CANCEL);
        break;
    case MOTION_CAL_RUNNING:
        title = "Calibrating";
        tc = COL_SEEK;
        snprintf(text, sizeof(text), "%s\nTouch anywhere to stop.", c->text);
        snprintf(prog, sizeof(prog), "Step %u of %u  |  %lu:%02lu", c->step, c->steps,
                 (unsigned long)(c->elapsed_s / 60), (unsigned long)(c->elapsed_s % 60));
        cal_buttons(NULL, 0, "Cancel", MOTION_CAL_CANCEL);
        break;
    case MOTION_CAL_ENTER_HEIGHT:
        snprintf(text, sizeof(text), "Desk is at its lowest. Measure floor to desktop and enter it:");
        cal_buttons(NULL, 0, NULL, 0);
        cal_entry_show();
        break;
    case MOTION_CAL_SUMMARY: {
        const motion_cal_t *r = &c->result;
        float k = r->scale > 0 ? r->scale : 1;
        float travel = k * r->sign * (r->raw_max - r->raw_min);
        snprintf(text, sizeof(text), "%s (%.0f mm of travel)\nSpeed up %.1f / down %.1f mm/s\nCoast up %.0f / down %.0f mm",
                 c->text, travel, k * r->speed_up, k * r->speed_down, k * r->coast_up, k * r->coast_down);
        cal_buttons("Save", MOTION_CAL_SAVE, "Discard", MOTION_CAL_DISCARD);
        break;
    }
    case MOTION_CAL_SAVED:
        title = "Calibrated";
        snprintf(text, sizeof(text), "%s", c->text);
        cal_buttons(NULL, 0, "OK", MOTION_CAL_CLOSE);
        break;
    case MOTION_CAL_FAILED:
        title = "Calibration failed";
        tc = COL_FAULT;
        snprintf(text, sizeof(text), "%s\nThe old calibration is kept.", c->text);
        cal_buttons(NULL, 0, "OK", MOTION_CAL_CLOSE);
        break;
    case MOTION_CAL_CANCELLED:
        title = "Cancelled";
        snprintf(text, sizeof(text), "%s", c->text);
        cal_buttons(NULL, 0, "OK", MOTION_CAL_CLOSE);
        break;
    default:
        break;
    }
    lv_label_set_text(s_cal_title, title);
    lv_obj_set_style_text_color(s_cal_title, tc, 0);
    lv_label_set_text(s_cal_text, text);
    lv_label_set_text(s_cal_prog, prog);
}

static lv_obj_t *make_overlay(lv_color_t accent)
{
    lv_obj_t *ov = lv_obj_create(lv_layer_top());
    lv_obj_set_size(ov, BOARD_LCD_H_RES, BOARD_LCD_V_RES);
    lv_obj_set_pos(ov, 0, 0);
    lv_obj_set_style_radius(ov, 0, 0);
    lv_obj_set_style_bg_color(ov, COL_BG, 0);
    lv_obj_set_style_bg_opa(ov, LV_OPA_90, 0);
    lv_obj_set_style_border_color(ov, accent, 0);
    lv_obj_set_style_border_width(ov, 3, 0);
    lv_obj_set_style_text_color(ov, COL_TEXT, 0);
    lv_obj_remove_flag(ov, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(ov, LV_OBJ_FLAG_CLICKABLE); /* absorb touches */
    lv_obj_add_flag(ov, LV_OBJ_FLAG_HIDDEN);
    return ov;
}

static void build_overlays(void)
{
    /* Cooldown */
    s_ov_cool = make_overlay(COL_COOL);
    lv_obj_t *t = make_label(s_ov_cool, &lv_font_montserrat_20, COL_COOL, "Cooling down");
    lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 14);
    s_ov_cool_time = make_label(s_ov_cool, &lv_font_montserrat_48, COL_TEXT, "0:00");
    lv_obj_align(s_ov_cool_time, LV_ALIGN_CENTER, 0, 0);
    lv_obj_t *m = make_label(s_ov_cool, &lv_font_montserrat_14, COL_DIM, "Motor duty limit reached. The desk is resting.");
    lv_obj_align(m, LV_ALIGN_BOTTOM_MID, 0, -14);

    /* Fault */
    s_ov_fault = make_overlay(COL_FAULT);
    lv_obj_t *ft = make_label(s_ov_fault, &lv_font_montserrat_28, COL_FAULT, LV_SYMBOL_WARNING " FAULT");
    lv_obj_align(ft, LV_ALIGN_TOP_MID, 0, 10);
    s_ov_fault_text = make_label(s_ov_fault, &lv_font_montserrat_20, COL_TEXT, "");
    lv_obj_set_width(s_ov_fault_text, 290);
    lv_label_set_long_mode(s_ov_fault_text, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(s_ov_fault_text, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_ov_fault_text, LV_ALIGN_CENTER, 0, -8);
    lv_obj_t *ack = make_button(s_ov_fault, 170, 46, COL_FAULT);
    lv_obj_align(ack, LV_ALIGN_BOTTOM_MID, 0, -12);
    lv_obj_t *al = make_label(ack, &lv_font_montserrat_20, COL_TEXT, "Acknowledge");
    lv_obj_center(al);
    lv_obj_add_event_cb(ack, fault_ack_cb, LV_EVENT_CLICKED, NULL);

    /* Toast */
    s_toast = make_label(lv_layer_top(), &lv_font_montserrat_14, COL_TEXT, "");
    lv_obj_set_style_bg_color(s_toast, COL_PANEL_HI, 0);
    lv_obj_set_style_bg_opa(s_toast, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_hor(s_toast, 10, 0);
    lv_obj_set_style_pad_ver(s_toast, 6, 0);
    lv_obj_set_style_radius(s_toast, 10, 0);
    lv_obj_align(s_toast, LV_ALIGN_TOP_MID, 0, 30);
    lv_obj_add_flag(s_toast, LV_OBJ_FLAG_HIDDEN);
    s_toast_timer = lv_timer_create(toast_hide_cb, TOAST_MS, NULL);
    lv_timer_pause(s_toast_timer);
}

/* ---------------- periodic update ---------------- */

static void update_row(status_row_t *row, const motion_status_t *st)
{
    if (st->state == MOTION_FAULT) {
        lv_obj_remove_flag(row->fault, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(row->fault, LV_OBJ_FLAG_HIDDEN);
    }
}

static void render_status(const motion_status_t *st)
{
    char h[16], info_lock[64] = "", info_ctrl[64] = "";
    fmt_height(h, sizeof(h), st->height_valid, st->height_mm);
    lv_label_set_text(s_lock_height, h);
    lv_label_set_text(s_ctrl_height, h);
    lv_obj_align_to(s_lock_unit, s_lock_height, LV_ALIGN_OUT_RIGHT_BOTTOM, 6, -8);
    lv_obj_align_to(s_ctrl_unit, s_ctrl_height, LV_ALIGN_OUT_RIGHT_BOTTOM, 4, -8);

    bool automatic = st->state == MOTION_SEEK || st->state == MOTION_SETTLE;
    if (automatic) {
        char t[16];
        fmt_height(t, sizeof(t), true, st->target_mm);
        snprintf(info_lock, sizeof(info_lock), LV_SYMBOL_RIGHT " %s %s  -  tap anywhere to stop", t, unit_str());
        snprintf(info_ctrl, sizeof(info_ctrl), LV_SYMBOL_RIGHT " %s %s\ntap anywhere to stop", t, unit_str());
    } else if (st->state == MOTION_JOG) {
        const char *m = st->output == MOTION_DIR_UP ? "moving up" : st->output == MOTION_DIR_DOWN ? "moving down" : "";
        snprintf(info_lock, sizeof(info_lock), "%s", m);
        snprintf(info_ctrl, sizeof(info_ctrl), "%s", m);
    }
    lv_label_set_text(s_lock_info, info_lock);
    lv_label_set_text(s_ctrl_info, info_ctrl);

    update_row(&s_lock_row, st);
    update_row(&s_ctrl_row, st);
    render_settings_info(st);
    render_calibration(st);

    /* Preset labels and availability */
    char pl[40];
    bool presets_ok = st->presets_available && st->state != MOTION_COOLDOWN && st->state != MOTION_FAULT;
    lv_obj_t *btns[2] = { s_btn_sit, s_btn_stand }, *lbls[2] = { s_lbl_sit, s_lbl_stand };
    for (int i = 0; i < 2; i++) {
        int32_t mm = st->preset_mm[i];
        fmt_height(h, sizeof(h), mm > 0, mm);
        if (st->presets_available) {
            snprintf(pl, sizeof(pl), "%s\n%s", s_cfg.preset_name[i], h);
        } else {
            snprintf(pl, sizeof(pl), "%s\nneeds cal.", s_cfg.preset_name[i]);
        }
        lv_label_set_text(lbls[i], pl);
        lv_obj_set_state(btns[i], LV_STATE_DISABLED, !presets_ok || mm <= 0);
    }

    /* Highlight the jog button that is actually driving. */
    lv_obj_set_style_border_width(s_btn_up, st->output == MOTION_DIR_UP ? 2 : 0, 0);
    lv_obj_set_style_border_color(s_btn_up, COL_ACCENT, 0);
    lv_obj_set_style_border_width(s_btn_down, st->output == MOTION_DIR_DOWN ? 2 : 0, 0);
    lv_obj_set_style_border_color(s_btn_down, COL_ACCENT, 0);

    /* Overlays */
    if (st->state == MOTION_COOLDOWN) {
        char c[16];
        snprintf(c, sizeof(c), "%lu:%02lu", (unsigned long)(st->cooldown_remaining_s / 60),
                 (unsigned long)(st->cooldown_remaining_s % 60));
        lv_label_set_text(s_ov_cool_time, c);
        lv_obj_remove_flag(s_ov_cool, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_ov_cool, LV_OBJ_FLAG_HIDDEN);
    }
    if (st->state == MOTION_FAULT) {
        lv_label_set_text(s_ov_fault_text, st->fault);
        lv_obj_remove_flag(s_ov_fault, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_ov_fault, LV_OBJ_FLAG_HIDDEN);
    }
}

static void render_net_row(status_row_t *row, const net_status_t *n)
{
    char txt[64] = "";
    lv_color_t c = COL_OFF;
    if (n->sta_connected) {
        snprintf(txt, sizeof(txt), "%s", n->ip);
        c = COL_TEXT;
    } else if (n->ap_active) {
        snprintf(txt, sizeof(txt), "%s  " NET_AP_IP, n->ap_ssid);
        c = COL_SEEK;
    } else if (n->sta_configured) {
        snprintf(txt, sizeof(txt), "connecting...");
    }
    lv_label_set_text(row->net, txt);
    lv_obj_set_style_text_color(row->wifi, c, 0);
}

static void status_timer_cb(lv_timer_t *t)
{
    motion_status_t st;
    motion_get_status(&st);

    net_status_t n;
    net_get_status(&n);
    n.rssi = 0; /* don't re-render on RSSI jitter */
    if (!s_have_net || memcmp(&n, &s_net, sizeof(n)) != 0) {
        render_net_row(&s_lock_row, &n);
        render_net_row(&s_ctrl_row, &n);
        s_net = n;
        s_have_net = true;
    }

    if (!s_have_last || memcmp(&st, &s_last, sizeof(st)) != 0) {
        if (s_have_last && st.last_error_seq != s_last.last_error_seq) {
            toast(st.last_error);
        }
        /* Wake the screen when something needs attention. */
        if (s_have_last && st.state != s_last.state &&
            (st.state == MOTION_FAULT || st.state == MOTION_SEEK || st.state == MOTION_COOLDOWN)) {
            set_dimmed(false);
            s_last_touch_ms = s_locked_at_ms = now_ms();
        }
        render_status(&st);
        s_last = st;
        s_have_last = true;
    }

    int64_t now = now_ms();
    static int64_t last_sensor_ms;
    if (now - last_sensor_ms >= SENSOR_REFRESH_MS) {
        render_sensor();
        last_sensor_ms = now;
    }
    bool moving = st.state == MOTION_JOG || st.state == MOTION_SEEK || st.state == MOTION_SETTLE ||
                  st.state == MOTION_CALIBRATING;
    if ((lv_screen_active() == s_scr_ctrl || lv_screen_active() == s_scr_set || lv_screen_active() == s_scr_num) && !moving &&
        now - s_last_touch_ms > s_cfg.autolock_ms) {
        show_lock();
    }
    if (lv_screen_active() == s_scr_dbg && !moving &&
        now - s_last_touch_ms > (s_debug_from_lock ? DEBUG_LOCKED_TIMEOUT_MS : s_cfg.autolock_ms)) {
        show_lock();
    }
    if (lv_screen_active() == s_scr_lock && !s_dimmed && !moving) {
        int64_t since = s_last_touch_ms > s_locked_at_ms ? s_last_touch_ms : s_locked_at_ms;
        if (now - since > s_cfg.dim_ms) {
            set_dimmed(true);
        }
    }
}

static void render_ready_cb(lv_event_t *e)
{
    s_frames++;
}

static void perf_timer_cb(lv_timer_t *t)
{
    static int64_t last_ms;
    int64_t now = now_ms();
    uint32_t frames = s_frames;
    s_frames = 0;
    float secs = last_ms ? (now - last_ms) / 1000.0f : 10.0f;
    last_ms = now;
    ESP_LOGI(TAG, "heap free %u (min %u, largest %u) | frames %lu in %.1fs = %.1f fps",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
             (unsigned long)frames, secs, frames / secs);
}

/* ---------------- settings ---------------- */

bool ui_is_locked(void)
{
    return s_locked;
}

void ui_apply_config(const desk_config_t *c)
{
    if (!s_disp || !lvgl_port_lock(1000)) {
        return;
    }
    bool units_changed = s_cfg.units_cm != c->units_cm;
    s_cfg.units_cm = c->units_cm;
    s_cfg.autolock_ms = (uint32_t)c->autolock_s * 1000;
    s_cfg.dim_ms = (uint32_t)c->dim_s * 1000;
    s_cfg.backlight = c->backlight;
    strlcpy(s_cfg.preset_name[0], c->presets[0].name, sizeof(s_cfg.preset_name[0]));
    strlcpy(s_cfg.preset_name[1], c->presets[1].name, sizeof(s_cfg.preset_name[1]));
    if (units_changed) {
        lv_label_set_text(s_lock_unit, unit_str());
        lv_label_set_text(s_ctrl_unit, unit_str());
    }
    if (!s_dimmed) {
        board_backlight_set(s_cfg.backlight);
    }
    s_have_last = false;  /* force a full re-render */
    lvgl_port_unlock();
}

/* ---------------- init ---------------- */

esp_err_t ui_init(esp_lcd_panel_io_handle_t io, esp_lcd_panel_handle_t panel, esp_lcd_touch_handle_t touch)
{
    const lvgl_port_cfg_t port_cfg = {
        .task_priority = 4,
        .task_stack = 6144,
        .task_affinity = -1,
        .task_max_sleep_ms = 500,
        .timer_period_ms = 5,
    };
    ESP_RETURN_ON_ERROR(lvgl_port_init(&port_cfg), TAG, "lvgl port");

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io,
        .panel_handle = panel,
        .buffer_size = BOARD_LCD_H_RES * DRAW_BUF_LINES,
        .double_buffer = true,
        .hres = BOARD_LCD_H_RES,
        .vres = BOARD_LCD_V_RES,
        .monochrome = false,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation = { .swap_xy = BOARD_LCD_SWAP_XY, .mirror_x = BOARD_LCD_MIRROR_X, .mirror_y = BOARD_LCD_MIRROR_Y },
        .flags = { .buff_dma = true, .swap_bytes = true },
    };
    s_disp = lvgl_port_add_disp(&disp_cfg);
    ESP_RETURN_ON_FALSE(s_disp, ESP_FAIL, TAG, "add disp");

    if (touch) {
        const lvgl_port_touch_cfg_t touch_cfg = { .disp = s_disp, .handle = touch };
        s_indev = lvgl_port_add_touch(&touch_cfg);
        ESP_RETURN_ON_FALSE(s_indev, ESP_FAIL, TAG, "add touch");
    }

    ESP_RETURN_ON_FALSE(lvgl_port_lock(0), ESP_FAIL, TAG, "lvgl lock");
    if (s_indev) {
        s_orig_read = lv_indev_get_read_cb(s_indev);
        lv_indev_set_read_cb(s_indev, touch_read_cb);
    }

    build_lock();
    build_control();
    build_debug();
    build_settings();
    build_number_entry();
    build_calibration();
    build_overlays();
    lv_screen_load(s_scr_lock);
    s_locked_at_ms = s_last_touch_ms = now_ms();

    lv_display_add_event_cb(s_disp, render_ready_cb, LV_EVENT_RENDER_READY, NULL);
    lv_timer_create(status_timer_cb, 50, NULL);
    lv_timer_create(perf_timer_cb, 10000, NULL);
    lvgl_port_unlock();

    board_backlight_set(s_cfg.backlight);
    ESP_LOGI(TAG, "UI ready (units %s, auto-lock %lus, dim %lus)", unit_str(),
             (unsigned long)(s_cfg.autolock_ms / 1000), (unsigned long)(s_cfg.dim_ms / 1000));
    return ESP_OK;
}
