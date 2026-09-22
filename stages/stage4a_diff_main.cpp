/*
 * Dethleffs Globebus control panel - STAGE 4 firmware: differential Toptron input (A0 - A1)
 * Board : Waveshare ESP32-S3-Touch-LCD-4.3B (M-053)
 *
 * Hardware (CLAUDE.md, "Planned interface", revised differential):
 *   SIG+ -47k- node -47k- SIG-;  node -4.7k- A0, 100 nF A0->board GND;
 *   SIG- -4.36k- A1, 100 nF A1->board GND.  A2, A3 unused (not read).
 *   Bench: SIG+ = 3.3 V, SIG- = board GND.  Van: SIG+/SIG- = the old meter's two measurement wires.
 *
 * Measurements each cycle:
 *   A0-A1  differential (MUX 000) = divider output referenced to signal -  <- the real measurement
 *   A0     single-ended  (MUX 100) = diagnostic
 *   A1     single-ended  (MUX 101) = signal - relative to board GND (ground offset, 0..~0.3 V in the van)
 *   Input estimate = (A0-A1) x 2.000, NOMINAL divider ratio, uncalibrated (Stage 5 calibrates).
 *
 * I2C access: same rules as Stage 2 (see CLAUDE.md "I2C bus access").
 *   ESP32_Display_Panel installed the ESP-IDF legacy driver on port 0 (GPIO8/9, 400 kHz).
 *   We use that installed driver's helpers (i2c_master_write_to_device /
 *   i2c_master_write_read_device), which serialise with the touch reads via the
 *   driver lock. No Wire, no second driver. Timeout 1000 ms (loop() can be starved
 *   ~80 ms by the LVGL task).
 *
 * ADS1115 (TI SBAS444): register 0x00 = conversion (16-bit signed, big-endian),
 * 0x01 = config. Each reading is a single-shot conversion:
 *   OS=1 (start) | MUX (000 = AIN0-AIN1, 100/101 = AIN0/AIN1 vs GND) | PGA=001 (+-4.096 V FS, 125 uV/LSB) |
 *   MODE=1 (single-shot) | DR=100 (128 SPS, ~7.8 ms) | comparator disabled (COMP_QUE=11).
 * Completion is detected by polling OS (reads 1 when idle), not by a fixed delay.
 * The +-4.096 V range covers 0..VDD (3.31 V); inputs must never exceed VDD + 0.3 V.
 *
 * Bench expectation: A0-A1 ~ 1.645 V, A0 ~ 1.645 V, A1 ~ 0 V, input estimate ~ 3.29 V.
 *
 * Screen shows: board profile, uptime, boot I2C scan summary, the three measurements
 * (raw, volts, min/max since reset), the input estimate, read error count, touch, RESCAN.
 */

#include <Arduino.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>
#include <driver/i2c.h>
#include "lvgl_v8_port.h"

using namespace esp_panel::drivers;
using namespace esp_panel::board;

// ---------- configuration ----------------------------------------------------

static constexpr i2c_port_t I2C_PORT = I2C_NUM_0;   // port installed by the display library
#ifdef ESP_PANEL_BOARD_TOUCH_I2C_HOST_ID
static_assert(ESP_PANEL_BOARD_TOUCH_I2C_HOST_ID == 0, "touch I2C host is not port 0");
#endif
#ifdef ESP_PANEL_BOARD_EXPANDER_I2C_HOST_ID
static_assert(ESP_PANEL_BOARD_EXPANDER_I2C_HOST_ID == 0, "expander I2C host is not port 0");
#endif

// Must stay well above the ~80 ms loop() starvation measured in Stage 2
static constexpr uint32_t I2C_TIMEOUT_MS = 1000;

static constexpr uint8_t ADS_ADDR     = 0x48;
static constexpr uint8_t ADS_REG_CONV = 0x00;
static constexpr uint8_t ADS_REG_CFG  = 0x01;

static constexpr uint16_t ADS_OS_START    = 0x8000;       // write: start conversion; read: 1 = idle
static constexpr uint16_t ADS_PGA_4V096   = 0x0200;       // PGA = 001
static constexpr uint16_t ADS_MODE_SINGLE = 0x0100;
static constexpr uint16_t ADS_DR_128SPS   = 0x0080;       // DR = 100
static constexpr uint16_t ADS_COMP_OFF    = 0x0003;       // COMP_QUE = 11
static constexpr float    ADS_LSB_V       = 4.096f / 32768.0f;   // 125 uV

static constexpr uint32_t CONV_POLL_TIMEOUT_MS = 50;      // 128 SPS conversion is ~7.8 ms
static constexpr uint32_t READ_PERIOD_MS       = 500;
static constexpr uint32_t LOG_EVERY            = 4;       // serial line every N read cycles (2 s)

static constexpr uint8_t ADDR_FIRST = 0x08;
static constexpr uint8_t ADDR_LAST  = 0x77;

// ---------- state ------------------------------------------------------------

static const char *g_board_name = "?";
static lv_obj_t *lbl_uptime   = nullptr;
static lv_obj_t *lbl_scan     = nullptr;
static lv_obj_t *lbl_ads_hdr  = nullptr;
static lv_obj_t *lbl_ch[4]    = {};
static lv_obj_t *lbl_errors   = nullptr;
static lv_obj_t *lbl_touch    = nullptr;
static lv_obj_t *touch_dot    = nullptr;
static uint32_t  press_count  = 0;

static volatile bool g_rescan_requested = false;

struct Channel {
    bool     valid = false;
    int16_t  raw   = 0;
    int16_t  min   = INT16_MAX;
    int16_t  max   = INT16_MIN;
    uint32_t reads = 0;
};
static constexpr int N_MEAS = 3;
static const uint8_t MEAS_MUX[N_MEAS]   = {0x0, 0x4, 0x5};   // AIN0-AIN1, AIN0-GND, AIN1-GND
static const char   *MEAS_NAME[N_MEAS]  = {"A0-A1", "A0", "A1"};
static const float   DIVIDER_NOMINAL    = 2.000f;             // 47k/47k, uncalibrated
static Channel     g_ch[N_MEAS];
static uint32_t    g_read_cycles    = 0;
static uint32_t    g_read_errors    = 0;
static esp_err_t   g_last_err       = ESP_OK;
static const char *g_last_err_where = "";

static const char *CH_NOTE[N_MEAS] = {"differential = Toptron signal at ADC", "vs board GND (diagnostic)",
                                      "vs board GND = signal- offset"};

// ---------- I2C scan (as Stage 2, run at boot and on RESCAN) -----------------

static esp_err_t probe(uint8_t addr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) return ESP_ERR_NO_MEM;
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(I2C_PORT, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return err;
}

static bool is_known(int a)
{
    return (a >= 0x20 && a <= 0x27) || (a >= 0x30 && a <= 0x3F) || a == 0x51 || a == 0x5D || a == ADS_ADDR;
}

static void run_scan()
{
    int n_found = 0, n_err = 0;
    bool ack[128] = {};
    for (uint8_t a = ADDR_FIRST; a <= ADDR_LAST; a++) {
        esp_err_t err = probe(a);
        if (err == ESP_OK) {
            ack[a] = true;
            n_found++;
        } else if (err != ESP_FAIL) {
            n_err++;
        }
    }
    int n_exp = 0, n_other = 0;
    for (int a = ADDR_FIRST; a <= ADDR_LAST; a++) {
        if (!ack[a]) continue;
        if ((a >= 0x20 && a <= 0x27) || (a >= 0x30 && a <= 0x3F)) n_exp++;
        if (!is_known(a)) n_other++;
    }
    bool pass = n_exp == 24 && ack[0x51] && ack[0x5D] && ack[ADS_ADDR] && n_other == 0 && n_err == 0;

    Serial.printf("[i2c] scan: %d found (CH422G %d/24, RTC 0x51 %s, GT911 0x5D %s, ADS1115 0x%02X %s, other %d), "
                  "%d bus errors -> %s\n",
                  n_found, n_exp, ack[0x51] ? "yes" : "NO", ack[0x5D] ? "yes" : "NO", ADS_ADDR,
                  ack[ADS_ADDR] ? "yes" : "NO", n_other, n_err, pass ? "PASS" : "CHECK");
    for (int a = ADDR_FIRST; a <= ADDR_LAST; a++) {
        if (ack[a] && !is_known(a)) Serial.printf("[i2c]   unexpected device at 0x%02X\n", a);
    }

    lvgl_port_lock(-1);
    lv_label_set_text_fmt(lbl_scan, "I2C scan: %d devices  |  CH422G %d/24  RTC %s  touch %s  ADS1115 0x48 %s  "
                          "other %d  bus err %d  ->  %s",
                          n_found, n_exp, ack[0x51] ? "ok" : "MISSING", ack[0x5D] ? "ok" : "MISSING",
                          ack[ADS_ADDR] ? "ok" : "MISSING", n_other, n_err, pass ? "PASS" : "CHECK");
    lv_obj_set_style_text_color(lbl_scan, pass ? lv_color_hex(0x66BB6A) : lv_color_hex(0xFF5252), 0);
    lvgl_port_unlock();
}

// ---------- ADS1115 ----------------------------------------------------------

static esp_err_t ads_write_reg(uint8_t reg, uint16_t value)
{
    uint8_t buf[3] = {reg, (uint8_t)(value >> 8), (uint8_t)(value & 0xFF)};
    return i2c_master_write_to_device(I2C_PORT, ADS_ADDR, buf, sizeof(buf), pdMS_TO_TICKS(I2C_TIMEOUT_MS));
}

static esp_err_t ads_read_reg(uint8_t reg, uint16_t *value)
{
    uint8_t buf[2] = {};
    esp_err_t err = i2c_master_write_read_device(I2C_PORT, ADS_ADDR, &reg, 1, buf, sizeof(buf),
                                                 pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err == ESP_OK) *value = ((uint16_t)buf[0] << 8) | buf[1];
    return err;
}

static void note_error(esp_err_t err, const char *where)
{
    g_read_errors++;
    g_last_err = err;
    g_last_err_where = where;
}

// Single-shot conversion with the given MUX[2:0] setting. Returns true and the raw count on success.
static bool ads_read_single(uint8_t mux, int16_t *raw)
{
    uint16_t cfg = ADS_OS_START | (uint16_t)((mux & 0x7) << 12) | ADS_PGA_4V096 | ADS_MODE_SINGLE |
                   ADS_DR_128SPS | ADS_COMP_OFF;
    esp_err_t err = ads_write_reg(ADS_REG_CFG, cfg);
    if (err != ESP_OK) {
        note_error(err, "config write");
        return false;
    }

    // Poll OS until the conversion is done. The OS bit reads 0 while converting.
    uint32_t t0 = millis();
    uint16_t status = 0;
    do {
        delay(2);
        err = ads_read_reg(ADS_REG_CFG, &status);
        if (err != ESP_OK) {
            note_error(err, "status read");
            return false;
        }
    } while (!(status & ADS_OS_START) && millis() - t0 < CONV_POLL_TIMEOUT_MS);
    if (!(status & ADS_OS_START)) {
        note_error(ESP_ERR_TIMEOUT, "conversion not finished");
        return false;
    }

    uint16_t value = 0;
    err = ads_read_reg(ADS_REG_CONV, &value);
    if (err != ESP_OK) {
        note_error(err, "conversion read");
        return false;
    }
    *raw = (int16_t)value;
    return true;
}

static void read_all_channels()
{
    g_read_cycles++;
    for (uint8_t ch = 0; ch < N_MEAS; ch++) {
        int16_t raw;
        Channel &c = g_ch[ch];
        c.valid = ads_read_single(MEAS_MUX[ch], &raw);
        if (!c.valid) continue;
        c.raw = raw;
        c.reads++;
        if (raw < c.min) c.min = raw;
        if (raw > c.max) c.max = raw;
    }

    if (g_read_cycles % LOG_EVERY == 1) {
        Serial.printf("[ads] cycle %lu:", (unsigned long)g_read_cycles);
        for (uint8_t ch = 0; ch < N_MEAS; ch++) {
            const Channel &c = g_ch[ch];
            if (c.valid) {
                Serial.printf("  %s %6d = %.4f V [%d..%d]", MEAS_NAME[ch], c.raw, c.raw * ADS_LSB_V, c.min, c.max);
            } else {
                Serial.printf("  %s ERR", MEAS_NAME[ch]);
            }
        }
        if (g_ch[0].valid) Serial.printf("  | input est %.4f V", g_ch[0].raw * ADS_LSB_V * DIVIDER_NOMINAL);
        Serial.printf("  | errors %lu\n", (unsigned long)g_read_errors);
    }
    if (g_last_err != ESP_OK) {
        Serial.printf("[ads] error: %s during %s (total %lu)\n", esp_err_to_name(g_last_err), g_last_err_where,
                      (unsigned long)g_read_errors);
        g_last_err = ESP_OK;   // log each cycle's last error once
    }

    lvgl_port_lock(-1);
    for (uint8_t ch = 0; ch < N_MEAS; ch++) {
        const Channel &c = g_ch[ch];
        if (c.valid) {
            // LVGL's printf is built with LV_SPRINTF_USE_FLOAT 0: never pass %f to lv_*_fmt
            // (it skips the double and misreads every later argument). Format with newlib instead.
            char line[128];
            snprintf(line, sizeof(line), "%-5s  %6d  %7.4f V  min %6d  max %6d  %s", MEAS_NAME[ch], c.raw,
                     c.raw * ADS_LSB_V, c.min, c.max, CH_NOTE[ch]);
            lv_label_set_text(lbl_ch[ch], line);
            lv_obj_set_style_text_color(lbl_ch[ch], ch == 0 ? lv_color_white() : lv_color_hex(0x90A4AE), 0);
        } else {
            lv_label_set_text_fmt(lbl_ch[ch], "%s   read error   %s", MEAS_NAME[ch], CH_NOTE[ch]);
            lv_obj_set_style_text_color(lbl_ch[ch], lv_color_hex(0xFF5252), 0);
        }
    }
    {
        char line[128];
        if (g_ch[0].valid) {
            snprintf(line, sizeof(line), "Input estimate (A0-A1) x %.3f = %.3f V   (nominal ratio, uncalibrated)",
                     DIVIDER_NOMINAL, g_ch[0].raw * ADS_LSB_V * DIVIDER_NOMINAL);
        } else {
            snprintf(line, sizeof(line), "Input estimate: --");
        }
        lv_label_set_text(lbl_ch[3], line);
    }
    lv_label_set_text_fmt(lbl_errors, "Read cycles: %lu    I2C/ADC errors: %lu", (unsigned long)g_read_cycles,
                          (unsigned long)g_read_errors);
    lv_obj_set_style_text_color(lbl_errors, g_read_errors ? lv_color_hex(0xFF5252) : lv_color_hex(0xB0BEC5), 0);
    lvgl_port_unlock();
}

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

static void rescan_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    Serial.println("[touch] RESCAN pressed");
    lv_label_set_text(lbl_scan, "I2C scan: running...");
    g_rescan_requested = true;   // picked up by loop(); never do I2C while holding the LVGL lock
}

static void status_timer_cb(lv_timer_t *)
{
    uint32_t s = millis() / 1000;
    lv_label_set_text_fmt(lbl_uptime, "Uptime: %02lu:%02lu:%02lu    Free heap: %lu B",
                          (unsigned long)(s / 3600), (unsigned long)((s / 60) % 60), (unsigned long)(s % 60),
                          (unsigned long)ESP.getFreeHeap());
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

    const lv_color_t grey = lv_color_hex(0xB0BEC5);

    lv_obj_t *title = make_label(scr, &lv_font_montserrat_30, lv_color_white());
    lv_label_set_text(title, "DETHLEFFS PANEL  -  STAGE 4  DIFFERENTIAL");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 30);

    lv_obj_t *lbl_board = make_label(scr, &lv_font_montserrat_16, lv_color_hex(0x80DEEA));
    lv_label_set_text_fmt(lbl_board, "Board profile: %s    Built: " __DATE__ " " __TIME__, g_board_name);
    lv_obj_align(lbl_board, LV_ALIGN_TOP_LEFT, 40, 78);

    lbl_uptime = make_label(scr, &lv_font_montserrat_16, grey);
    lv_obj_align(lbl_uptime, LV_ALIGN_TOP_LEFT, 40, 102);

    lbl_scan = make_label(scr, &lv_font_montserrat_16, grey);
    lv_label_set_long_mode(lbl_scan, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_scan, 720);
    lv_label_set_text(lbl_scan, "I2C scan: waiting...");
    lv_obj_align(lbl_scan, LV_ALIGN_TOP_LEFT, 40, 130);

    lbl_ads_hdr = make_label(scr, &lv_font_montserrat_16, lv_color_hex(0x80DEEA));
    lv_label_set_text_fmt(lbl_ads_hdr, "ADS1115 @0x%02X   A0-A1 differential + A0/A1 vs GND, PGA +-4.096 V (125 uV/LSB), 128 SPS, "
                          "every %lu ms", ADS_ADDR, (unsigned long)READ_PERIOD_MS);
    lv_obj_align(lbl_ads_hdr, LV_ALIGN_TOP_LEFT, 40, 180);

    for (int ch = 0; ch < 4; ch++) {
        lbl_ch[ch] = make_label(scr, &lv_font_montserrat_20, lv_color_white());
        lv_label_set_text(lbl_ch[ch], "--");
        lv_obj_align(lbl_ch[ch], LV_ALIGN_TOP_LEFT, 40, 208 + ch * 32);
    }

    lbl_errors = make_label(scr, &lv_font_montserrat_16, grey);
    lv_label_set_text(lbl_errors, "Read cycles: 0    I2C/ADC errors: 0");
    lv_obj_align(lbl_errors, LV_ALIGN_TOP_LEFT, 40, 344);

    lbl_touch = make_label(scr, &lv_font_montserrat_16, lv_color_hex(0xFFD54F));
    lv_label_set_text(lbl_touch, "Touch: (touch anywhere)");
    lv_obj_align(lbl_touch, LV_ALIGN_BOTTOM_LEFT, 40, -40);

    lv_obj_t *btn = lv_btn_create(scr);
    lv_obj_set_size(btn, 200, 64);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_RIGHT, -50, -40);
    lv_obj_add_event_cb(btn, rescan_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *btn_txt = make_label(btn, &lv_font_montserrat_20, lv_color_white());
    lv_label_set_text(btn_txt, "RESCAN I2C");
    lv_obj_center(btn_txt);

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
    Serial.println("\n=== Dethleffs panel STAGE 4 (differential A0-A1) ===");
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

    Serial.println("Setup done. Boot I2C scan, then A0-A1 / A0 / A1 every 500 ms (log every 2 s).");
    g_rescan_requested = true;
}

void loop()
{
    static uint32_t last_read_ms = 0;
    if (g_rescan_requested) {
        g_rescan_requested = false;
        run_scan();
    }
    if (millis() - last_read_ms >= READ_PERIOD_MS) {
        last_read_ms = millis();
        read_all_channels();
    }
    delay(5);
}
