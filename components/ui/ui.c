/*
 * Touch UI: lock screen, control screen and the SEEK / COOLDOWN / FAULT overlays.
 * The UI never drives the desk itself; it only sends motion requests.
 */
#include "ui.h"

#include <stdio.h>
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

static const char *TAG = "ui";

#define DRAW_BUF_LINES      40
#define JOG_REPEAT_MS       100
#define UNLOCK_FRACTION     0.85f
#define TOAST_MS            2500

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

/* Settings that become configurable in stage 1. */
static struct {
    bool units_cm;
    uint32_t autolock_ms;
    uint32_t dim_ms;
    uint8_t backlight;
    uint8_t backlight_dim;
} s_cfg = {
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
    lv_obj_t *wifi, *mqtt, *fault;
} status_row_t;

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

static void make_status_row(lv_obj_t *scr, status_row_t *row)
{
    row->wifi = make_label(scr, &lv_font_montserrat_14, COL_OFF, LV_SYMBOL_WIFI);
    lv_obj_align(row->wifi, LV_ALIGN_TOP_LEFT, 10, 6);
    row->mqtt = make_label(scr, &lv_font_montserrat_14, COL_OFF, "MQTT");
    lv_obj_align_to(row->mqtt, row->wifi, LV_ALIGN_OUT_RIGHT_MID, 8, 0);
    row->fault = make_label(scr, &lv_font_montserrat_14, COL_FAULT, LV_SYMBOL_WARNING);
    lv_obj_align_to(row->fault, row->mqtt, LV_ALIGN_OUT_RIGHT_MID, 8, 0);
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
    s_locked_at_ms = now_ms();
    ESP_LOGI(TAG, "locked");
}

static void show_control(void)
{
    lv_screen_load(s_scr_ctrl);
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

static void build_lock(void)
{
    s_scr_lock = make_screen();
    make_status_row(s_scr_lock, &s_lock_row);

    s_lock_height = make_label(s_scr_lock, &lv_font_montserrat_48, COL_TEXT, "--");
    lv_obj_align(s_lock_height, LV_ALIGN_TOP_MID, 0, 86);
    s_lock_unit = make_label(s_scr_lock, &lv_font_montserrat_20, COL_DIM, unit_str());
    lv_obj_align_to(s_lock_unit, s_lock_height, LV_ALIGN_OUT_BOTTOM_MID, 0, 2);
    s_lock_info = make_label(s_scr_lock, &lv_font_montserrat_14, COL_SEEK, "");
    lv_obj_set_width(s_lock_info, 160);
    lv_obj_set_style_text_align(s_lock_info, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_lock_info, LV_ALIGN_TOP_MID, 0, 180);

    s_track = lv_obj_create(s_scr_lock);
    lv_obj_set_size(s_track, 156, 56);
    lv_obj_align(s_track, LV_ALIGN_BOTTOM_MID, 0, -18);
    lv_obj_set_style_radius(s_track, 28, 0);
    lv_obj_set_style_bg_color(s_track, COL_PANEL, 0);
    lv_obj_set_style_border_width(s_track, 0, 0);
    lv_obj_set_style_pad_all(s_track, 4, 0);
    lv_obj_remove_flag(s_track, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(s_track, LV_OBJ_FLAG_CLICKABLE);

    s_track_hint = make_label(s_track, &lv_font_montserrat_14, COL_DIM, "slide to unlock");
    lv_obj_align(s_track_hint, LV_ALIGN_RIGHT_MID, -10, 0);

    s_knob = lv_obj_create(s_track);
    lv_obj_set_size(s_knob, 48, 48);
    lv_obj_set_style_radius(s_knob, 24, 0);
    lv_obj_set_style_bg_color(s_knob, COL_ACCENT, 0);
    lv_obj_set_style_border_width(s_knob, 0, 0);
    lv_obj_remove_flag(s_knob, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(s_knob, LV_OBJ_FLAG_GESTURE_BUBBLE);
    lv_obj_set_align(s_knob, LV_ALIGN_LEFT_MID);
    /* Bigger hit area so the knob is easy to grab. */
    lv_obj_set_ext_click_area(s_knob, 6);
    lv_obj_t *arrow = make_label(s_knob, &lv_font_montserrat_20, COL_TEXT, LV_SYMBOL_RIGHT);
    lv_obj_center(arrow);
    lv_obj_add_event_cb(s_knob, knob_event_cb, LV_EVENT_ALL, NULL);
}

static lv_obj_t *make_preset(lv_obj_t *parent, const char *name, motion_preset_t id, lv_obj_t **lbl)
{
    lv_obj_t *b = make_button(parent, 74, 52, COL_PANEL);
    *lbl = make_label(b, &lv_font_montserrat_14, COL_TEXT, name);
    lv_obj_set_style_text_align(*lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_center(*lbl);
    lv_obj_add_event_cb(b, preset_event_cb, LV_EVENT_CLICKED, (void *)(intptr_t)id);
    return b;
}

static void build_control(void)
{
    s_scr_ctrl = make_screen();
    make_status_row(s_scr_ctrl, &s_ctrl_row);

    lv_obj_t *lock = make_button(s_scr_ctrl, 52, 26, COL_PANEL);
    lv_obj_set_style_radius(lock, 13, 0);
    lv_obj_align(lock, LV_ALIGN_TOP_RIGHT, -6, 2);
    lv_obj_t *ll = make_label(lock, &lv_font_montserrat_14, COL_DIM, "Lock");
    lv_obj_center(ll);
    lv_obj_add_event_cb(lock, lock_btn_cb, LV_EVENT_CLICKED, NULL);

    s_ctrl_height = make_label(s_scr_ctrl, &lv_font_montserrat_48, COL_TEXT, "--");
    lv_obj_align(s_ctrl_height, LV_ALIGN_TOP_MID, 0, 34);
    s_ctrl_unit = make_label(s_scr_ctrl, &lv_font_montserrat_14, COL_DIM, unit_str());
    lv_obj_align_to(s_ctrl_unit, s_ctrl_height, LV_ALIGN_OUT_BOTTOM_MID, 0, 0);
    s_ctrl_info = make_label(s_scr_ctrl, &lv_font_montserrat_14, COL_SEEK, "");
    lv_obj_set_width(s_ctrl_info, 168);
    lv_obj_set_style_text_align(s_ctrl_info, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_ctrl_info, LV_ALIGN_TOP_MID, 0, 106);

    s_btn_up = make_button(s_scr_ctrl, 156, 64, COL_PANEL);
    lv_obj_align(s_btn_up, LV_ALIGN_TOP_MID, 0, 128);
    lv_obj_t *lu = make_label(s_btn_up, &lv_font_montserrat_28, COL_TEXT, LV_SYMBOL_UP);
    lv_obj_center(lu);
    lv_obj_add_event_cb(s_btn_up, jog_event_cb, LV_EVENT_ALL, (void *)(intptr_t)MOTION_DIR_UP);

    s_btn_down = make_button(s_scr_ctrl, 156, 64, COL_PANEL);
    lv_obj_align(s_btn_down, LV_ALIGN_TOP_MID, 0, 200);
    lv_obj_t *ld = make_label(s_btn_down, &lv_font_montserrat_28, COL_TEXT, LV_SYMBOL_DOWN);
    lv_obj_center(ld);
    lv_obj_add_event_cb(s_btn_down, jog_event_cb, LV_EVENT_ALL, (void *)(intptr_t)MOTION_DIR_DOWN);

    s_btn_sit = make_preset(s_scr_ctrl, "SIT", MOTION_PRESET_SIT, &s_lbl_sit);
    lv_obj_align(s_btn_sit, LV_ALIGN_BOTTOM_LEFT, 8, -8);
    s_btn_stand = make_preset(s_scr_ctrl, "STAND", MOTION_PRESET_STAND, &s_lbl_stand);
    lv_obj_align(s_btn_stand, LV_ALIGN_BOTTOM_RIGHT, -8, -8);
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
    lv_obj_align(t, LV_ALIGN_TOP_MID, 0, 70);
    s_ov_cool_time = make_label(s_ov_cool, &lv_font_montserrat_48, COL_TEXT, "0:00");
    lv_obj_align(s_ov_cool_time, LV_ALIGN_CENTER, 0, -10);
    lv_obj_t *m = make_label(s_ov_cool, &lv_font_montserrat_14, COL_DIM, "Motor duty limit reached.\nThe desk is resting.");
    lv_obj_set_style_text_align(m, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(m, LV_ALIGN_CENTER, 0, 50);

    /* Fault */
    s_ov_fault = make_overlay(COL_FAULT);
    lv_obj_t *ft = make_label(s_ov_fault, &lv_font_montserrat_28, COL_FAULT, LV_SYMBOL_WARNING " FAULT");
    lv_obj_align(ft, LV_ALIGN_TOP_MID, 0, 50);
    s_ov_fault_text = make_label(s_ov_fault, &lv_font_montserrat_20, COL_TEXT, "");
    lv_obj_set_width(s_ov_fault_text, 150);
    lv_label_set_long_mode(s_ov_fault_text, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_align(s_ov_fault_text, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_ov_fault_text, LV_ALIGN_CENTER, 0, -20);
    lv_obj_t *ack = make_button(s_ov_fault, 140, 52, COL_FAULT);
    lv_obj_align(ack, LV_ALIGN_BOTTOM_MID, 0, -24);
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
    lv_obj_align(s_toast, LV_ALIGN_TOP_MID, 0, 28);
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
    char h[16], info[64] = "";
    fmt_height(h, sizeof(h), st->height_valid, st->height_mm);
    lv_label_set_text(s_lock_height, h);
    lv_label_set_text(s_ctrl_height, h);

    bool automatic = st->state == MOTION_SEEK || st->state == MOTION_SETTLE;
    if (automatic) {
        char t[16];
        fmt_height(t, sizeof(t), true, st->target_mm);
        snprintf(info, sizeof(info), LV_SYMBOL_RIGHT " %s %s\ntap anywhere to stop", t, unit_str());
    } else if (st->state == MOTION_JOG) {
        snprintf(info, sizeof(info), "%s", st->output == MOTION_DIR_UP ? "moving up" :
                                          st->output == MOTION_DIR_DOWN ? "moving down" : "");
    }
    lv_label_set_text(s_lock_info, info);
    lv_label_set_text(s_ctrl_info, info);

    update_row(&s_lock_row, st);
    update_row(&s_ctrl_row, st);

    /* Preset labels and availability */
    char pl[24];
    fmt_height(h, sizeof(h), true, st->preset_mm[MOTION_PRESET_SIT]);
    snprintf(pl, sizeof(pl), "SIT\n%s", h);
    lv_label_set_text(s_lbl_sit, pl);
    fmt_height(h, sizeof(h), true, st->preset_mm[MOTION_PRESET_STAND]);
    snprintf(pl, sizeof(pl), "STAND\n%s", h);
    lv_label_set_text(s_lbl_stand, pl);
    bool presets_ok = st->presets_available && st->state != MOTION_COOLDOWN && st->state != MOTION_FAULT;
    lv_obj_set_state(s_btn_sit, LV_STATE_DISABLED, !presets_ok);
    lv_obj_set_state(s_btn_stand, LV_STATE_DISABLED, !presets_ok);

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

static void status_timer_cb(lv_timer_t *t)
{
    motion_status_t st;
    motion_get_status(&st);

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
    bool moving = st.state == MOTION_JOG || st.state == MOTION_SEEK || st.state == MOTION_SETTLE ||
                  st.state == MOTION_CALIBRATING;
    if (lv_screen_active() == s_scr_ctrl && !moving && now - s_last_touch_ms > s_cfg.autolock_ms) {
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
        .rotation = { .swap_xy = false, .mirror_x = false, .mirror_y = false },
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
