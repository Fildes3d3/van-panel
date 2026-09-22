/*
 * Dethleffs Globebus control panel - STAGE 1 diagnostic firmware
 * Board : Waveshare ESP32-S3-Touch-LCD-4.3B (M-053)
 *
 * Purpose: prove toolchain + flashing + RGB LCD + GT911 touch.
 * Nothing external is connected. No I2C devices are added. No Toptron.
 *
 * Screen shows:
 *   - board profile name compiled in (must say "...4.3-B")
 *   - chip / flash / PSRAM / heap figures (flash must be 16 MB, PSRAM ~8 MB)
 *   - uptime (proves the LVGL task keeps running)
 *   - live touch X/Y + press counter + a dot that follows your finger
 *   - four coloured corner markers (orientation check)
 *   - a button (proves widget-level touch events)
 */

#include <Arduino.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>
#include "lvgl_v8_port.h"

using namespace esp_panel::drivers;
using namespace esp_panel::board;

static const char *g_board_name = "?";
static lv_obj_t *lbl_uptime  = nullptr;
static lv_obj_t *lbl_mem     = nullptr;
static lv_obj_t *lbl_touch   = nullptr;
static lv_obj_t *lbl_btn     = nullptr;
static lv_obj_t *touch_dot   = nullptr;
static uint32_t  press_count = 0;
static uint32_t  btn_count   = 0;

// ---------- helpers ----------------------------------------------------------

static lv_obj_t *make_label(lv_obj_t *parent, const lv_font_t *font, lv_color_t color)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, color, 0);
    return l;
}

static void make_corner(lv_align_t align, lv_color_t color)
{
    lv_obj_t *r = lv_obj_create(lv_scr_act());
    lv_obj_remove_style_all(r);
    lv_obj_set_size(r, 24, 24);
    lv_obj_set_style_bg_color(r, color, 0);
    lv_obj_set_style_bg_opa(r, LV_OPA_COVER, 0);
    lv_obj_align(r, align, 0, 0);
    lv_obj_clear_flag(r, LV_OBJ_FLAG_CLICKABLE);
}

// ---------- callbacks (run inside the LVGL task, lock already held) ----------

static void screen_touch_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    lv_indev_t *indev = lv_indev_get_act();
    if (!indev) return;

    lv_point_t p;
    lv_indev_get_point(indev, &p);

    if (code == LV_EVENT_PRESSED) {
        press_count++;
        Serial.printf("[touch] PRESSED  x=%d y=%d  count=%lu\n", p.x, p.y, (unsigned long)press_count);
    }
    lv_label_set_text_fmt(lbl_touch, "Touch: x=%3d  y=%3d   presses=%lu",
                          p.x, p.y, (unsigned long)press_count);
    lv_obj_set_pos(touch_dot, p.x - 10, p.y - 10);
    lv_obj_clear_flag(touch_dot, LV_OBJ_FLAG_HIDDEN);
}

static void button_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    btn_count++;
    lv_label_set_text_fmt(lbl_btn, "Button clicks: %lu", (unsigned long)btn_count);
    Serial.printf("[touch] button clicked, total=%lu\n", (unsigned long)btn_count);
}

static void status_timer_cb(lv_timer_t *)
{
    uint32_t s = millis() / 1000;
    lv_label_set_text_fmt(lbl_uptime, "Uptime: %02lu:%02lu:%02lu",
                          (unsigned long)(s / 3600), (unsigned long)((s / 60) % 60),
                          (unsigned long)(s % 60));
    lv_label_set_text_fmt(lbl_mem, "Free heap: %lu B    Free PSRAM: %lu B",
                          (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getFreePsram());
}

// ---------- UI ---------------------------------------------------------------

static void build_ui()
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x101418), 0);
    lv_obj_add_flag(scr, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_event_cb(scr, screen_touch_cb, LV_EVENT_PRESSED, nullptr);
    lv_obj_add_event_cb(scr, screen_touch_cb, LV_EVENT_PRESSING, nullptr);

    // Orientation markers: TL red, TR green, BL blue, BR yellow
    make_corner(LV_ALIGN_TOP_LEFT,     lv_color_hex(0xE53935));
    make_corner(LV_ALIGN_TOP_RIGHT,    lv_color_hex(0x43A047));
    make_corner(LV_ALIGN_BOTTOM_LEFT,  lv_color_hex(0x1E88E5));
    make_corner(LV_ALIGN_BOTTOM_RIGHT, lv_color_hex(0xFDD835));

    lv_obj_t *title = make_label(scr, &lv_font_montserrat_30, lv_color_white());
    lv_label_set_text(title, "DETHLEFFS PANEL  -  STAGE 1 DIAGNOSTIC");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 34);

    const lv_color_t grey = lv_color_hex(0xB0BEC5);

    lv_obj_t *lbl_board = make_label(scr, &lv_font_montserrat_20, lv_color_hex(0x80DEEA));
    lv_label_set_text_fmt(lbl_board, "Board profile: %s", g_board_name);
    lv_obj_align(lbl_board, LV_ALIGN_TOP_LEFT, 40, 90);

    lv_obj_t *lbl_chip = make_label(scr, &lv_font_montserrat_16, grey);
    lv_label_set_text_fmt(lbl_chip,
        "Chip: %s rev %d  @ %lu MHz    Flash: %lu MB    PSRAM: %lu KB\n"
        "Arduino-ESP32 %d.%d.%d   IDF %s   ESP32_Display_Panel %d.%d.%d   LVGL %d.%d.%d\n"
        "Built: " __DATE__ " " __TIME__,
        ESP.getChipModel(), ESP.getChipRevision(), (unsigned long)ESP.getCpuFreqMHz(),
        (unsigned long)(ESP.getFlashChipSize() / (1024 * 1024)),
        (unsigned long)(ESP.getPsramSize() / 1024),
        ESP_ARDUINO_VERSION_MAJOR, ESP_ARDUINO_VERSION_MINOR, ESP_ARDUINO_VERSION_PATCH,
        ESP.getSdkVersion(),
        ESP_PANEL_VERSION_MAJOR, ESP_PANEL_VERSION_MINOR, ESP_PANEL_VERSION_PATCH,
        LVGL_VERSION_MAJOR, LVGL_VERSION_MINOR, LVGL_VERSION_PATCH);
    lv_obj_align(lbl_chip, LV_ALIGN_TOP_LEFT, 40, 124);

    // Hard sanity checks, shown in red if the build config does not match the 4.3B
    if (ESP.getPsramSize() < 4 * 1024 * 1024 || ESP.getFlashChipSize() < 16 * 1024 * 1024) {
        lv_obj_t *warn = make_label(scr, &lv_font_montserrat_20, lv_color_hex(0xFF5252));
        lv_label_set_text(warn, "WARNING: PSRAM/flash size does not match N16R8 - check board JSON");
        lv_obj_align(warn, LV_ALIGN_TOP_LEFT, 40, 196);
    }

    lbl_uptime = make_label(scr, &lv_font_montserrat_20, lv_color_white());
    lv_obj_align(lbl_uptime, LV_ALIGN_TOP_LEFT, 40, 232);

    lbl_mem = make_label(scr, &lv_font_montserrat_16, grey);
    lv_obj_align(lbl_mem, LV_ALIGN_TOP_LEFT, 40, 264);

    lbl_touch = make_label(scr, &lv_font_montserrat_24, lv_color_hex(0xFFD54F));
    lv_label_set_text(lbl_touch, "Touch: (touch anywhere)");
    lv_obj_align(lbl_touch, LV_ALIGN_TOP_LEFT, 40, 300);

    lv_obj_t *btn = lv_btn_create(scr);
    lv_obj_set_size(btn, 220, 70);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_RIGHT, -50, -50);
    lv_obj_add_event_cb(btn, button_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *btn_txt = make_label(btn, &lv_font_montserrat_20, lv_color_white());
    lv_label_set_text(btn_txt, "TEST BUTTON");
    lv_obj_center(btn_txt);

    lbl_btn = make_label(scr, &lv_font_montserrat_20, lv_color_white());
    lv_label_set_text(lbl_btn, "Button clicks: 0");
    lv_obj_align(lbl_btn, LV_ALIGN_BOTTOM_LEFT, 40, -70);

    touch_dot = lv_obj_create(scr);
    lv_obj_remove_style_all(touch_dot);
    lv_obj_set_size(touch_dot, 20, 20);
    lv_obj_set_style_radius(touch_dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(touch_dot, lv_color_hex(0xFF4081), 0);
    lv_obj_set_style_bg_opa(touch_dot, LV_OPA_COVER, 0);
    lv_obj_clear_flag(touch_dot, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_add_flag(touch_dot, LV_OBJ_FLAG_HIDDEN);

    lv_timer_create(status_timer_cb, 1000, nullptr);
    status_timer_cb(nullptr);
}

// ---------- Arduino entry points ---------------------------------------------

void setup()
{
    Serial.begin(115200);
    delay(1500);   // give USB-CDC time to enumerate so early logs are not lost
    Serial.println("\n=== Dethleffs panel STAGE 1 ===");
    Serial.printf("Flash %lu MB, PSRAM %lu KB\n",
                  (unsigned long)(ESP.getFlashChipSize() / (1024 * 1024)),
                  (unsigned long)(ESP.getPsramSize() / 1024));

    Board *board = new Board();
    g_board_name = board->getConfig().name;
    Serial.printf("Initializing board (%s)\n", g_board_name);
    board->init();
    if (!board->begin()) {
        Serial.println("FATAL: board->begin() failed - LCD/touch/IO-expander init error");
        while (true) { delay(1000); }
    }

    Serial.println("Initializing LVGL");
    lvgl_port_init(board->getLCD(), board->getTouch());

    lvgl_port_lock(-1);
    build_ui();
    lvgl_port_unlock();

    Serial.println("Setup done. Touch the screen; coordinates are logged here.");
}

void loop()
{
    delay(1000);   // all UI work happens in the LVGL port task
}
