/*
 * Dethleffs Globebus control panel - firmware: Toptron gauge reader + Globebus UI
 * Board : Waveshare ESP32-S3-Touch-LCD-4.3B (M-053)
 *
 * Toptron facts (CLAUDE.md, Stage 4 notes):
 *   - The gauge output exists ONLY while a momentary selector is held (starter/leisure rocker, fresh/grey rocker).
 *   - It is a ~mA gauge drive, not a stiff voltage. The original gauge is 85.8 ohm.
 *   - Battery channels and the clean tank are steady while held; grey decays (dirty sender suspected).
 *   - The original gauge dial: ONE needle, two aligned linear scales - 8/10/12/14/16 V on top and
 *     0/(1/4)/(1/2)/(3/4)/(1/1) below. So tank % = (V_scale - 8) / 8 * 100; one battery calibration gives both.
 *
 * Hardware (perfboard, van configuration):
 *   SIG+ --+-- 4.7k -- A0,  100 nF A0 -> board GND
 *        R_burden (499 ohm interim; final ~86 ohm)
 *   SIG- --+-- 4.36k -- A1, 100 nF A1 -> board GND
 *   Board GND on its own wire to vehicle ground (NOT the Toptron ground / signal -).
 *
 * Measurement (unchanged from the tested Stage 4c v4):
 *   - Polls A0-A1 (differential) every ~50 ms. |V| > PRESS_THRESHOLD_MV = a selector is held.
 *   - While held: ~100 samples/s. Displayed value = max |mV| in 0.1-0.5 s after press start (steady value for
 *     batteries/clean, early value for grey before its decay); held on screen for DISPLAY_HOLD_MS.
 *   - The board does NOT know which selector is pressed (user decision): every press is shown both as volts and as
 *     percent, same size; the user reads the one that applies. Volts turn red in the dial's red zones.
 *
 * UI (user-approved mockup v2, 2026-09-22):
 *   Main screen: "Globebus", clock (PCF85063 RTC), Toptron status dot; volts left / percent right (88 px digits);
 *   battery scale 8-16 V with the dial's colour zones, water scale (blue wedge) directly below, aligned so both
 *   markers sit above each other like the single needle; icons in front of each scale; footer with state,
 *   "provisional" badge and a settings button.
 *   Settings screen: clock set with +/- (hours, minutes), diagnostics (raw mV, press history, I2C, idle A0/A1).
 *
 * PCF85063A RTC (0x51), register map per NXP datasheet (from memory - the running clock verifies it):
 *   00h Control_1 (bit1 12_24: 0 = 24 h), 04h Seconds (bit7 OS = oscillator stopped / time invalid), 05h Minutes,
 *   06h Hours (24 h BCD). Setting the time writes seconds = 0 (clears OS), minutes, hours.
 *
 * ADS1115 (TI SBAS444): single-shot, OS-bit polling, comparator off. PGA bits 11:9: 001 +-4.096, 011 +-1.024,
 *   101 +-0.256 V. Signal at +-1.024 V with the 499 ohm interim burden; +-0.256 V once the ~86 ohm burden is fitted.
 *
 * I2C: the display library's legacy driver on port 0, i2c_master_* helpers only, no Wire, timeout 1000 ms.
 * LVGL printf has no float support: floats are formatted with snprintf before being handed to LVGL.
 * Fonts in src/fonts/ generated with lv_font_conv 1.5.2 from LVGL's bundled Montserrat-Medium / FontAwesome 5.
 */

#include <Arduino.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>
#include <driver/i2c.h>
#include "lvgl_v8_port.h"

using namespace esp_panel::drivers;
using namespace esp_panel::board;

LV_FONT_DECLARE(font_num_88);
LV_FONT_DECLARE(font_clock_44);
LV_FONT_DECLARE(font_icon_44);

// ---------- configuration ----------------------------------------------------

static constexpr i2c_port_t I2C_PORT = I2C_NUM_0;   // port installed by the display library
#ifdef ESP_PANEL_BOARD_TOUCH_I2C_HOST_ID
static_assert(ESP_PANEL_BOARD_TOUCH_I2C_HOST_ID == 0, "touch I2C host is not port 0");
#endif
#ifdef ESP_PANEL_BOARD_EXPANDER_I2C_HOST_ID
static_assert(ESP_PANEL_BOARD_EXPANDER_I2C_HOST_ID == 0, "expander I2C host is not port 0");
#endif

static constexpr uint32_t I2C_TIMEOUT_MS = 1000;    // well above the ~80 ms loop() starvation measured in Stage 2

static constexpr uint8_t ADS_ADDR     = 0x48;
static constexpr uint8_t ADS_REG_CONV = 0x00;
static constexpr uint8_t ADS_REG_CFG  = 0x01;
static constexpr uint8_t RTC_ADDR     = 0x51;

static constexpr uint16_t ADS_OS_START    = 0x8000;       // write: start conversion; read: 1 = idle
static constexpr uint16_t ADS_MODE_SINGLE = 0x0100;
static constexpr uint16_t ADS_DR_128SPS   = 0x0080;       // DR = 100
static constexpr uint16_t ADS_COMP_OFF    = 0x0003;       // COMP_QUE = 11

static constexpr uint8_t MUX_DIFF_01 = 0x0;               // AIN0 - AIN1
static constexpr uint8_t MUX_AIN0    = 0x4;               // AIN0 - GND
static constexpr uint8_t MUX_AIN1    = 0x5;               // AIN1 - GND

struct Pga {
    uint16_t bits;
    float    lsb_v;
};
static constexpr Pga PGA_4V096  = {0x0200, 4.096f / 32768.0f};  // PGA = 001
static constexpr Pga PGA_1V024  = {0x0600, 1.024f / 32768.0f};  // PGA = 011
static constexpr Pga PGA_SIGNAL = PGA_1V024;                    // 499 ohm interim burden
static constexpr const char *PGA_SIGNAL_TEXT = "+-1.024 V";
static constexpr float BURDEN_OHM = 499.0f;

static constexpr uint32_t CONV_POLL_TIMEOUT_MS = 50;      // 128 SPS conversion is ~7.8 ms

static constexpr float    PRESS_THRESHOLD_MV   = 5.0f;    // "none" reads ~0 through the burden
static constexpr int      RELEASE_SAMPLES      = 3;       // consecutive samples below threshold = released
static constexpr uint32_t IDLE_POLL_MS         = 50;
static constexpr uint32_t DIAG_PERIOD_MS       = 1000;
static constexpr uint32_t CLOCK_PERIOD_MS      = 1000;
static constexpr uint32_t LOG_FAST_MS          = 3000;    // serial: every sample during the first 3 s of a press
static constexpr uint32_t LOG_SLOW_MS          = 100;

// Display value capture window and hold
static constexpr uint32_t WIN_START_MS    = 100;
static constexpr uint32_t WIN_END_MS      = 500;
static constexpr uint32_t DISPLAY_HOLD_MS = 60000;

// PROVISIONAL volts calibration, 499 ohm burden: V = VOLT_OFFSET + VOLT_PER_MV * mV
// from (265.3 mV, 12.82 V leisure, 2026-09-22) and (242.6 mV, 12.62 V car; battery V from 2026-09-21).
// Checked against a 3rd point: ~240 mV -> 12.59 V vs 12.6 V measured. Redo in Stage 5 with the final ~86 ohm burden.
static constexpr bool  VOLT_CALIBRATED = false;
static constexpr float VOLT_PER_MV     = (12.82f - 12.62f) / (265.3f - 242.6f);   // ~0.00881 V/mV
static constexpr float VOLT_OFFSET     = 12.82f - 265.3f * VOLT_PER_MV;           // ~10.48 V at 0 mV

// Gauge dial: one needle over two aligned linear scales
static constexpr float DIAL_V_MIN      = 8.0f;    // left end: 8 V / empty
static constexpr float DIAL_V_MAX      = 16.0f;   // right end: 16 V / full
static constexpr float RED_LOW_V       = 10.5f;   // dial red zone below (approx., from the dial photo)
static constexpr float RED_HIGH_V      = 15.0f;   // dial red zone above

static constexpr int HISTORY_ROWS      = 6;
static constexpr int END_BACK_SAMPLES  = 20;      // ~0.2 s at ~100 samples/s

static constexpr uint8_t ADDR_FIRST = 0x08;
static constexpr uint8_t ADDR_LAST  = 0x77;

// ---------- UI geometry (800 x 480) ------------------------------------------

static constexpr int SCALE_X     = 92;            // bar left edge
static constexpr int SCALE_W     = 684;           // bar width
static constexpr int SCALE_H     = 22;
static constexpr int BAT_BAR_Y   = 262;
static constexpr int WAT_BAR_Y   = 348;
static constexpr int MARKER_W    = 20;
static constexpr int MARKER_H    = 13;

static const lv_color_t COL_BG      = lv_color_hex(0x141A20);
static const lv_color_t COL_TEXT    = lv_color_hex(0xE8EDF1);
static const lv_color_t COL_MUTED   = lv_color_hex(0x8FA1AD);
static const lv_color_t COL_DIM     = lv_color_hex(0x4E5D68);
static const lv_color_t COL_RED     = lv_color_hex(0xB33A3A);
static const lv_color_t COL_GREEN   = lv_color_hex(0x4F7D1F);
static const lv_color_t COL_BLUE    = lv_color_hex(0x2F6FB0);
static const lv_color_t COL_TRACK   = lv_color_hex(0x26313A);
static const lv_color_t COL_WARN    = lv_color_hex(0xFF5A52);
static const lv_color_t COL_OK      = lv_color_hex(0x639922);

// ---------- state ------------------------------------------------------------

static const char *g_board_name = "?";

// main screen
static lv_obj_t *scr_main    = nullptr;
static lv_obj_t *lbl_clock   = nullptr;
static lv_obj_t *dot_status  = nullptr;
static lv_obj_t *lbl_volt    = nullptr;
static lv_obj_t *lbl_pct     = nullptr;
static lv_obj_t *mk_bat      = nullptr;
static lv_obj_t *mk_wat      = nullptr;
static lv_obj_t *lbl_state   = nullptr;
static lv_obj_t *pill_prov   = nullptr;

// settings screen
static lv_obj_t *scr_set     = nullptr;
static lv_obj_t *lbl_set_hh  = nullptr;
static lv_obj_t *lbl_set_mm  = nullptr;
static lv_obj_t *lbl_rtc     = nullptr;
static lv_obj_t *lbl_scan    = nullptr;
static lv_obj_t *lbl_hist    = nullptr;
static lv_obj_t *lbl_diag    = nullptr;

static volatile bool g_rescan_requested = false;
static volatile bool g_rtc_write_req    = false;
static volatile int  g_set_hh = 12, g_set_mm = 0;
static uint32_t      g_read_errors      = 0;

struct Press {
    uint32_t n           = 0;      // press counter
    uint32_t start_ms    = 0;
    uint32_t samples     = 0;
    float    first_mv    = 0;
    float    peak_mv     = 0;
    float    last_mv     = 0;
    float    at_mv[4]    = {NAN, NAN, NAN, NAN};   // value at 200 / 500 / 1000 / 2000 ms
    uint32_t last_log_ms = 0;
    int      below       = 0;
    bool     active      = false;
    float    win_mv      = NAN;    // max |mV| inside the capture window
    bool     shown       = false;  // display value already published for this press
};
static constexpr uint32_t AT_MS[4] = {200, 500, 1000, 2000};
static Press g_press;

static float g_recent[END_BACK_SAMPLES + 1];      // to report the value shortly before release
static int   g_recent_count = 0;
static int   g_recent_head  = 0;

struct HistoryRow {
    uint32_t n;
    float    dur_s, first, at[4], peak, end;
};
static HistoryRow g_hist[HISTORY_ROWS];
static int        g_hist_count = 0;

static uint32_t g_display_ms = 0;                 // when the displayed value was published (0 = nothing shown)

// ---------- small helpers ----------------------------------------------------

static uint8_t bcd2bin(uint8_t v) { return (v >> 4) * 10 + (v & 0x0F); }
static uint8_t bin2bcd(uint8_t v) { return ((v / 10) << 4) | (v % 10); }

static float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

static lv_obj_t *make_label(lv_obj_t *parent, const lv_font_t *font, lv_color_t color)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, color, 0);
    return l;
}

static lv_obj_t *make_rect(lv_obj_t *parent, int x, int y, int w, int h, lv_color_t color)
{
    lv_obj_t *r = lv_obj_create(parent);
    lv_obj_remove_style_all(r);
    lv_obj_set_pos(r, x, y);
    lv_obj_set_size(r, w, h);
    lv_obj_set_style_bg_color(r, color, 0);
    lv_obj_set_style_bg_opa(r, LV_OPA_COVER, 0);
    lv_obj_clear_flag(r, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    return r;
}

// ---------- I2C scan (run at boot and on RESCAN) -----------------------------

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

    lvgl_port_lock(-1);
    lv_label_set_text_fmt(lbl_scan, "I2C scan: %d devices | CH422G %d/24  RTC %s  touch %s  ADS1115 %s  other %d  "
                          "bus err %d -> %s",
                          n_found, n_exp, ack[0x51] ? "ok" : "MISSING", ack[0x5D] ? "ok" : "MISSING",
                          ack[ADS_ADDR] ? "ok" : "MISSING", n_other, n_err, pass ? "PASS" : "CHECK");
    lv_obj_set_style_text_color(lbl_scan, pass ? COL_OK : COL_WARN, 0);
    lv_obj_set_style_bg_color(dot_status, (pass && g_read_errors == 0) ? COL_OK : COL_WARN, 0);
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
    Serial.printf("[ads] error: %s during %s (total %lu)\n", esp_err_to_name(err), where,
                  (unsigned long)g_read_errors);
}

// Single-shot conversion. Returns true and the value in volts on success.
static bool ads_read_volts(uint8_t mux, const Pga &pga, float *volts)
{
    uint16_t cfg = ADS_OS_START | (uint16_t)((mux & 0x7) << 12) | pga.bits | ADS_MODE_SINGLE | ADS_DR_128SPS |
                   ADS_COMP_OFF;
    esp_err_t err = ads_write_reg(ADS_REG_CFG, cfg);
    if (err != ESP_OK) {
        note_error(err, "config write");
        return false;
    }

    // Poll OS until the conversion is done. The OS bit reads 0 while converting.
    uint32_t t0 = millis();
    uint16_t status = 0;
    do {
        delay(1);
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
    *volts = (int16_t)value * pga.lsb_v;
    return true;
}

// ---------- RTC (PCF85063A) --------------------------------------------------

// Returns true with valid time; false if the read failed or the oscillator-stop flag says the time is invalid.
static bool rtc_read(int *hh, int *mm, bool *osc_stopped)
{
    uint8_t reg = 0x04, buf[3] = {};
    esp_err_t err = i2c_master_write_read_device(I2C_PORT, RTC_ADDR, &reg, 1, buf, sizeof(buf),
                                                 pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (err != ESP_OK) return false;
    *osc_stopped = buf[0] & 0x80;
    *mm = bcd2bin(buf[1] & 0x7F);
    *hh = bcd2bin(buf[2] & 0x3F);
    return !*osc_stopped;
}

static bool rtc_write(int hh, int mm)
{
    // Make sure the hours register is in 24 h mode (Control_1 bit1 = 0), keep the other bits.
    uint8_t reg = 0x00, c1 = 0;
    if (i2c_master_write_read_device(I2C_PORT, RTC_ADDR, &reg, 1, &c1, 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != ESP_OK)
        return false;
    if (c1 & 0x02) {
        uint8_t w[2] = {0x00, (uint8_t)(c1 & ~0x02)};
        if (i2c_master_write_to_device(I2C_PORT, RTC_ADDR, w, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != ESP_OK) return false;
    }
    uint8_t buf[4] = {0x04, 0x00 /* seconds = 0, clears OS */, bin2bcd(mm), bin2bcd(hh)};
    return i2c_master_write_to_device(I2C_PORT, RTC_ADDR, buf, sizeof(buf), pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == ESP_OK;
}

static void update_clock()
{
    int hh = 0, mm = 0;
    bool osc_stopped = false;
    bool ok = rtc_read(&hh, &mm, &osc_stopped);
    char t[8];
    if (ok) snprintf(t, sizeof(t), "%02d:%02d", hh, mm);
    else    snprintf(t, sizeof(t), "--:--");
    lvgl_port_lock(-1);
    lv_label_set_text(lbl_clock, t);
    lv_label_set_text(lbl_rtc, ok ? "RTC running" : (osc_stopped ? "RTC not set (oscillator was stopped)"
                                                                 : "RTC read error"));
    lvgl_port_unlock();
}

// ---------- main display -----------------------------------------------------

static void place_marker(lv_obj_t *mk, int bar_y, float frac)
{
    int x = SCALE_X + (int)(clampf(frac, 0, 1) * SCALE_W) - MARKER_W / 2;
    lv_obj_set_pos(mk, x, bar_y - MARKER_H - 2);
    lv_obj_clear_flag(mk, LV_OBJ_FLAG_HIDDEN);
}

static void show_value(float mv)
{
    float volts = VOLT_OFFSET + VOLT_PER_MV * mv;
    float frac  = (volts - DIAL_V_MIN) / (DIAL_V_MAX - DIAL_V_MIN);
    float pct   = clampf(frac, 0, 1) * 100.0f;
    bool  red   = volts < RED_LOW_V || volts > RED_HIGH_V;
    char v[16], p[16];
    snprintf(v, sizeof(v), "%.1f V", volts);
    snprintf(p, sizeof(p), "%.0f %%", pct);

    lvgl_port_lock(-1);
    lv_label_set_text(lbl_volt, v);
    lv_label_set_text(lbl_pct, p);
    lv_obj_set_style_text_color(lbl_volt, red ? COL_WARN : COL_TEXT, 0);
    lv_obj_set_style_text_color(lbl_pct, COL_TEXT, 0);
    place_marker(mk_bat, BAT_BAR_Y, frac);
    place_marker(mk_wat, WAT_BAR_Y, frac);
    lvgl_port_unlock();

    g_display_ms = millis();
    if (g_display_ms == 0) g_display_ms = 1;
    Serial.printf("[display] %.2f mV -> %.2f V / %.0f %%%s\n", mv, volts, pct, red ? " (red zone)" : "");
}

static void clear_value()
{
    lvgl_port_lock(-1);
    lv_label_set_text(lbl_volt, "-- V");
    lv_label_set_text(lbl_pct, "-- %");
    lv_obj_set_style_text_color(lbl_volt, COL_DIM, 0);
    lv_obj_set_style_text_color(lbl_pct, COL_DIM, 0);
    lv_obj_add_flag(mk_bat, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(mk_wat, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(lbl_state, "Hold a selector on the panel");
    lvgl_port_unlock();
    g_display_ms = 0;
}

static void set_state_text(const char *text)
{
    lvgl_port_lock(-1);
    lv_label_set_text(lbl_state, text);
    lvgl_port_unlock();
}

// ---------- press handling ---------------------------------------------------

static void fmt_opt(char *buf, size_t n, float mv)
{
    if (isnan(mv)) snprintf(buf, n, "--");
    else           snprintf(buf, n, "%.2f", mv);
}

static void press_start(float mv)
{
    g_press.n++;
    g_press.active      = true;
    g_press.start_ms    = millis();
    g_press.samples     = 0;
    g_press.first_mv    = mv;
    g_press.peak_mv     = mv;
    g_press.last_mv     = mv;
    g_press.below       = 0;
    g_press.last_log_ms = 0;
    g_press.win_mv      = NAN;
    g_press.shown       = false;
    for (float &v : g_press.at_mv) v = NAN;
    g_recent_count = 0;
    g_recent_head  = 0;
    Serial.printf("\n[press] #%lu START  first %.3f mV\n", (unsigned long)g_press.n, mv);
    set_state_text("Reading...");
}

static void press_sample(float mv)
{
    uint32_t t = millis() - g_press.start_ms;
    g_press.samples++;
    g_press.last_mv = mv;
    g_recent[g_recent_head] = mv;
    g_recent_head = (g_recent_head + 1) % (END_BACK_SAMPLES + 1);
    if (g_recent_count < END_BACK_SAMPLES + 1) g_recent_count++;
    if (fabsf(mv) > fabsf(g_press.peak_mv)) g_press.peak_mv = mv;
    for (int i = 0; i < 4; i++) {
        if (isnan(g_press.at_mv[i]) && t >= AT_MS[i]) g_press.at_mv[i] = mv;
    }
    if (t >= WIN_START_MS && t <= WIN_END_MS && (isnan(g_press.win_mv) || fabsf(mv) > fabsf(g_press.win_mv))) {
        g_press.win_mv = mv;
    }
    if (!g_press.shown && t > WIN_END_MS && !isnan(g_press.win_mv)) {
        g_press.shown = true;
        show_value(g_press.win_mv);
    }
    if (t < LOG_FAST_MS || t - g_press.last_log_ms >= LOG_SLOW_MS) {
        Serial.printf("[press] #%lu t=%5lu ms  %8.3f mV\n", (unsigned long)g_press.n, (unsigned long)t, mv);
        g_press.last_log_ms = t;
    }
}

static void press_end()
{
    g_press.active = false;
    if (!g_press.shown) {
        show_value(!isnan(g_press.win_mv) ? g_press.win_mv : g_press.peak_mv);
        g_press.shown = true;
    }
    uint32_t dur = millis() - g_press.start_ms;
    float end_mv = g_recent_count > END_BACK_SAMPLES ? g_recent[g_recent_head] : g_press.first_mv;

    char a[4][12];
    for (int i = 0; i < 4; i++) fmt_opt(a[i], sizeof(a[i]), g_press.at_mv[i]);
    Serial.printf("[press] #%lu END  %.2f s, %lu samples | first %.2f  @0.2s %s  @0.5s %s  @1s %s  @2s %s  "
                  "peak %.2f  end(-0.2s) %.2f  window %.2f mV\n",
                  (unsigned long)g_press.n, dur / 1000.0f, (unsigned long)g_press.samples, g_press.first_mv, a[0],
                  a[1], a[2], a[3], g_press.peak_mv, end_mv, g_press.win_mv);

    for (int i = HISTORY_ROWS - 1; i > 0; i--) g_hist[i] = g_hist[i - 1];
    HistoryRow &r = g_hist[0];
    r.n     = g_press.n;
    r.dur_s = dur / 1000.0f;
    r.first = g_press.first_mv;
    for (int i = 0; i < 4; i++) r.at[i] = g_press.at_mv[i];
    r.peak  = g_press.peak_mv;
    r.end   = end_mv;
    if (g_hist_count < HISTORY_ROWS) g_hist_count++;

    static char text[HISTORY_ROWS * 96 + 128];
    size_t len = snprintf(text, sizeof(text), "#    dur    first    @0.2s    @0.5s     @1s      @2s     end   (mV)\n");
    for (int i = 0; i < g_hist_count && len < sizeof(text) - 96; i++) {
        const HistoryRow &h = g_hist[i];
        char b[4][12];
        for (int k = 0; k < 4; k++) fmt_opt(b[k], sizeof(b[k]), h.at[k]);
        len += snprintf(text + len, sizeof(text) - len, "%-4lu %4.1fs  %7.2f  %7s  %7s  %7s  %7s  %7.2f\n",
                        (unsigned long)h.n, h.dur_s, h.first, b[0], b[1], b[2], b[3], h.end);
    }
    snprintf(text + len, sizeof(text) - len, "burden %.0f ohm, PGA %s, newest on top", BURDEN_OHM, PGA_SIGNAL_TEXT);

    lvgl_port_lock(-1);
    lv_label_set_text(lbl_hist, text);
    lvgl_port_unlock();
}

static void run_diagnostics()
{
    float a0 = NAN, a1 = NAN;
    bool ok0 = ads_read_volts(MUX_AIN0, PGA_4V096, &a0);
    bool ok1 = ads_read_volts(MUX_AIN1, PGA_4V096, &a1);
    char s0[16], s1[16], line[256];
    if (ok0) snprintf(s0, sizeof(s0), "%.4f", a0); else snprintf(s0, sizeof(s0), "ERR");
    if (ok1) snprintf(s1, sizeof(s1), "%.4f", a1); else snprintf(s1, sizeof(s1), "ERR");
    uint32_t s = millis() / 1000;
    snprintf(line, sizeof(line),
             "Idle: A0 %s V   A1 %s V (signal- vs board GND)   I2C/ADC errors: %lu\n"
             "Uptime %02lu:%02lu:%02lu   presses %lu   free heap %lu B   built " __DATE__ " " __TIME__ "\n"
             "Volts %s: V = %.3f + %.5f x mV   %% = (V - 8) / 8 per gauge dial",
             s0, s1, (unsigned long)g_read_errors, (unsigned long)(s / 3600), (unsigned long)((s / 60) % 60),
             (unsigned long)(s % 60), (unsigned long)g_press.n, (unsigned long)ESP.getFreeHeap(),
             VOLT_CALIBRATED ? "calibrated" : "PROVISIONAL", VOLT_OFFSET, VOLT_PER_MV);
    lvgl_port_lock(-1);
    lv_label_set_text(lbl_diag, line);
    lv_obj_set_style_text_color(lbl_diag, g_read_errors ? COL_WARN : COL_MUTED, 0);
    if (g_read_errors) lv_obj_set_style_bg_color(dot_status, COL_WARN, 0);
    lvgl_port_unlock();
}

// ---------- UI construction --------------------------------------------------

static lv_obj_t *make_marker(lv_obj_t *parent)
{
    static lv_color_t buf[LV_CANVAS_BUF_SIZE_TRUE_COLOR_ALPHA(MARKER_W, MARKER_H)];
    static bool drawn = false;
    lv_obj_t *c = lv_canvas_create(parent);
    lv_canvas_set_buffer(c, buf, MARKER_W, MARKER_H, LV_IMG_CF_TRUE_COLOR_ALPHA);
    if (!drawn) {
        lv_canvas_fill_bg(c, lv_color_black(), LV_OPA_TRANSP);
        lv_draw_rect_dsc_t d;
        lv_draw_rect_dsc_init(&d);
        d.bg_color = lv_color_white();
        d.bg_opa   = LV_OPA_COVER;
        lv_point_t pts[3] = {{0, 0}, {MARKER_W - 1, 0}, {MARKER_W / 2, MARKER_H - 1}};
        lv_canvas_draw_polygon(c, pts, 3, &d);
        drawn = true;
    }
    lv_obj_add_flag(c, LV_OBJ_FLAG_HIDDEN);
    return c;
}

static void make_ticks(lv_obj_t *parent, int bar_y, const char *const labels[5])
{
    for (int i = 0; i < 5; i++) {
        lv_obj_t *l = make_label(parent, &lv_font_montserrat_16, COL_MUTED);
        lv_label_set_text(l, labels[i]);
        lv_obj_update_layout(l);
        int w = lv_obj_get_width(l);
        int x = SCALE_X + i * SCALE_W / 4 - w / 2;
        if (i == 0) x = SCALE_X;
        if (i == 4) x = SCALE_X + SCALE_W - w;
        lv_obj_set_pos(l, x, bar_y + SCALE_H + 5);
    }
}

static void build_battery_scale(lv_obj_t *scr)
{
    // Zones per the gauge dial: red 8-10.5, red/green hatched 10.5-12, green 12-15, red 15-16 V
    auto xv = [](float v) { return SCALE_X + (int)((v - DIAL_V_MIN) / (DIAL_V_MAX - DIAL_V_MIN) * SCALE_W); };
    int x1 = xv(10.5f), x2 = xv(12.0f), x3 = xv(15.0f), xe = SCALE_X + SCALE_W;
    make_rect(scr, SCALE_X, BAT_BAR_Y, x1 - SCALE_X, SCALE_H, COL_RED);
    for (int x = x1, i = 0; x < x2; x += 8, i++) {
        make_rect(scr, x, BAT_BAR_Y, (x + 8 <= x2 ? 8 : x2 - x), SCALE_H, (i & 1) ? COL_GREEN : COL_RED);
    }
    make_rect(scr, x2, BAT_BAR_Y, x3 - x2, SCALE_H, COL_GREEN);
    make_rect(scr, x3, BAT_BAR_Y, xe - x3, SCALE_H, COL_RED);

    lv_obj_t *icon = make_label(scr, &font_icon_44, COL_MUTED);
    lv_label_set_text(icon, "\xEF\x89\x82");       // FontAwesome battery-half U+F242
    lv_obj_set_pos(icon, 22, BAT_BAR_Y - 12);

    static const char *const t[5] = {"8", "10", "12", "14", "16"};
    make_ticks(scr, BAT_BAR_Y, t);
    mk_bat = make_marker(scr);
}

static void build_water_scale(lv_obj_t *scr)
{
    make_rect(scr, SCALE_X, WAT_BAR_Y, SCALE_W, SCALE_H, COL_TRACK);
    // Blue wedge like the dial: thin on the left, full height on the right
    const int n = 57;
    for (int i = 0; i < n; i++) {
        int x0 = SCALE_X + i * SCALE_W / n, x1 = SCALE_X + (i + 1) * SCALE_W / n;
        int h  = (int)(SCALE_H * (0.15f + 0.85f * i / (n - 1)) + 0.5f);
        make_rect(scr, x0, WAT_BAR_Y + SCALE_H - h, x1 - x0, h, COL_BLUE);
    }
    lv_obj_t *icon = make_label(scr, &font_icon_44, COL_MUTED);
    lv_label_set_text(icon, "\xEF\x81\x83");       // FontAwesome tint (droplet) U+F043
    lv_obj_set_pos(icon, 34, WAT_BAR_Y - 12);

    static const char *const t[5] = {"0", "1/4", "1/2", "3/4", "1/1"};
    make_ticks(scr, WAT_BAR_Y, t);
    mk_wat = make_marker(scr);
}

static void open_settings_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    lv_scr_load(scr_set);
}

static void back_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    lv_scr_load(scr_main);
}

static void rescan_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    lv_label_set_text(lbl_scan, "I2C scan: running...");
    g_rescan_requested = true;   // picked up by loop(); never do I2C while holding the LVGL lock
}

static void refresh_set_labels()
{
    lv_label_set_text_fmt(lbl_set_hh, "%02d", g_set_hh);
    lv_label_set_text_fmt(lbl_set_mm, "%02d", g_set_mm);
}

static void adj_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    intptr_t k = (intptr_t)lv_event_get_user_data(e);   // 0 h+, 1 h-, 2 m+, 3 m-
    if (k == 0) g_set_hh = (g_set_hh + 1) % 24;
    if (k == 1) g_set_hh = (g_set_hh + 23) % 24;
    if (k == 2) g_set_mm = (g_set_mm + 1) % 60;
    if (k == 3) g_set_mm = (g_set_mm + 59) % 60;
    refresh_set_labels();
}

static void set_clock_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    g_rtc_write_req = true;      // written by loop()
    lv_label_set_text(lbl_rtc, "Setting clock...");
}

static lv_obj_t *make_button(lv_obj_t *parent, const char *text, int w, int h, lv_event_cb_t cb, void *ud)
{
    lv_obj_t *b = lv_btn_create(parent);
    lv_obj_set_size(b, w, h);
    lv_obj_set_style_bg_color(b, lv_color_hex(0x2A3640), 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
    lv_obj_t *l = make_label(b, &lv_font_montserrat_24, COL_TEXT);
    lv_label_set_text(l, text);
    lv_obj_center(l);
    return b;
}

static void build_main_screen()
{
    scr_main = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(scr_main, COL_BG, 0);
    lv_obj_clear_flag(scr_main, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *name = make_label(scr_main, &lv_font_montserrat_26, COL_TEXT);
    lv_label_set_text(name, "Globebus");
    lv_obj_set_pos(name, 24, 18);

    lbl_clock = make_label(scr_main, &font_clock_44, COL_TEXT);
    lv_label_set_text(lbl_clock, "--:--");
    lv_obj_align(lbl_clock, LV_ALIGN_TOP_MID, 0, 18);

    lv_obj_t *st = make_label(scr_main, &lv_font_montserrat_20, COL_MUTED);
    lv_label_set_text(st, "Toptron");
    lv_obj_align(st, LV_ALIGN_TOP_RIGHT, -24, 20);
    lv_obj_update_layout(scr_main);   // align_to needs the label's final coordinates
    dot_status = make_rect(scr_main, 0, 0, 10, 10, COL_OK);
    lv_obj_set_style_radius(dot_status, LV_RADIUS_CIRCLE, 0);
    lv_obj_align_to(dot_status, st, LV_ALIGN_OUT_LEFT_MID, -8, 0);

    lbl_volt = make_label(scr_main, &font_num_88, COL_DIM);
    lv_obj_set_width(lbl_volt, 400);
    lv_obj_set_style_text_align(lbl_volt, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(lbl_volt, "-- V");
    lv_obj_set_pos(lbl_volt, 0, 118);

    lbl_pct = make_label(scr_main, &font_num_88, COL_DIM);
    lv_obj_set_width(lbl_pct, 400);
    lv_obj_set_style_text_align(lbl_pct, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(lbl_pct, "-- %");
    lv_obj_set_pos(lbl_pct, 400, 118);

    build_battery_scale(scr_main);
    build_water_scale(scr_main);

    lbl_state = make_label(scr_main, &lv_font_montserrat_18, COL_MUTED);
    lv_label_set_text(lbl_state, "Hold a selector on the panel");
    lv_obj_set_pos(lbl_state, 24, 444);

    lv_obj_t *gear = lv_btn_create(scr_main);
    lv_obj_set_size(gear, 56, 44);
    lv_obj_align(gear, LV_ALIGN_BOTTOM_RIGHT, -16, -10);
    lv_obj_set_style_bg_opa(gear, LV_OPA_TRANSP, 0);
    lv_obj_set_style_shadow_width(gear, 0, 0);
    lv_obj_add_event_cb(gear, open_settings_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *gl = make_label(gear, &lv_font_montserrat_26, COL_MUTED);
    lv_label_set_text(gl, LV_SYMBOL_SETTINGS);
    lv_obj_center(gl);

    pill_prov = lv_obj_create(scr_main);
    lv_obj_remove_style_all(pill_prov);
    lv_obj_set_style_bg_color(pill_prov, lv_color_hex(0x3A2F12), 0);
    lv_obj_set_style_bg_opa(pill_prov, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(pill_prov, 12, 0);
    lv_obj_set_style_pad_hor(pill_prov, 12, 0);
    lv_obj_set_style_pad_ver(pill_prov, 3, 0);
    lv_obj_set_size(pill_prov, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_t *pl = make_label(pill_prov, &lv_font_montserrat_16, lv_color_hex(0xF0C565));
    lv_label_set_text(pl, "provisional");
    lv_obj_update_layout(scr_main);   // gear and pill sizes/positions final before align_to
    lv_obj_align_to(pill_prov, gear, LV_ALIGN_OUT_LEFT_MID, -8, 0);
    if (VOLT_CALIBRATED) lv_obj_add_flag(pill_prov, LV_OBJ_FLAG_HIDDEN);
}

static void build_settings_screen()
{
    scr_set = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(scr_set, COL_BG, 0);

    lv_obj_t *back = make_button(scr_set, LV_SYMBOL_LEFT "  Back", 150, 50, back_cb, nullptr);
    lv_obj_set_pos(back, 16, 12);
    lv_obj_t *title = make_label(scr_set, &lv_font_montserrat_28, COL_TEXT);
    lv_label_set_text(title, "Settings");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 22);

    // Clock set: [+] HH [-]  :  [+] MM [-]   [Set clock]
    lv_obj_t *cl = make_label(scr_set, &lv_font_montserrat_20, COL_MUTED);
    lv_label_set_text(cl, "Clock");
    lv_obj_set_pos(cl, 24, 84);

    lv_obj_t *bhp = make_button(scr_set, LV_SYMBOL_PLUS, 70, 50, adj_cb, (void *)0);
    lv_obj_set_pos(bhp, 24, 116);
    lbl_set_hh = make_label(scr_set, &font_clock_44, COL_TEXT);
    lv_obj_set_pos(lbl_set_hh, 108, 118);
    lv_obj_t *bhm = make_button(scr_set, LV_SYMBOL_MINUS, 70, 50, adj_cb, (void *)1);
    lv_obj_set_pos(bhm, 180, 116);

    lv_obj_t *colon = make_label(scr_set, &font_clock_44, COL_TEXT);
    lv_label_set_text(colon, ":");
    lv_obj_set_pos(colon, 266, 116);

    lv_obj_t *bmp = make_button(scr_set, LV_SYMBOL_PLUS, 70, 50, adj_cb, (void *)2);
    lv_obj_set_pos(bmp, 296, 116);
    lbl_set_mm = make_label(scr_set, &font_clock_44, COL_TEXT);
    lv_obj_set_pos(lbl_set_mm, 380, 118);
    lv_obj_t *bmm = make_button(scr_set, LV_SYMBOL_MINUS, 70, 50, adj_cb, (void *)3);
    lv_obj_set_pos(bmm, 452, 116);

    lv_obj_t *bset = make_button(scr_set, "Set clock", 180, 50, set_clock_cb, nullptr);
    lv_obj_set_pos(bset, 560, 116);
    refresh_set_labels();

    lbl_rtc = make_label(scr_set, &lv_font_montserrat_16, COL_MUTED);
    lv_label_set_text(lbl_rtc, "RTC: --");
    lv_obj_set_pos(lbl_rtc, 24, 176);

    // Diagnostics
    lv_obj_t *dl = make_label(scr_set, &lv_font_montserrat_20, COL_MUTED);
    lv_label_set_text(dl, "Diagnostics");
    lv_obj_set_pos(dl, 24, 210);

    lv_obj_t *brs = make_button(scr_set, "Rescan I2C", 170, 40, rescan_cb, nullptr);
    lv_obj_set_pos(brs, 610, 202);

    lbl_scan = make_label(scr_set, &lv_font_montserrat_14, COL_MUTED);
    lv_label_set_long_mode(lbl_scan, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_scan, 752);
    lv_label_set_text(lbl_scan, "I2C scan: waiting...");
    lv_obj_set_pos(lbl_scan, 24, 250);

    lbl_hist = make_label(scr_set, &lv_font_montserrat_14, lv_color_hex(0xB0BEC5));
    lv_label_set_text(lbl_hist, "No presses yet.");
    lv_obj_set_pos(lbl_hist, 24, 276);

    lbl_diag = make_label(scr_set, &lv_font_montserrat_14, COL_MUTED);
    lv_label_set_text(lbl_diag, "Idle: --");
    lv_obj_set_pos(lbl_diag, 24, 412);
}

// ---------- Arduino entry points ---------------------------------------------

void setup()
{
    Serial.begin(115200);
    delay(1500);   // give USB-CDC time to enumerate so early logs are not lost
    Serial.println("\n=== Dethleffs panel - Globebus UI (Toptron via burden resistor) ===");

    Board *board = new Board();
    g_board_name = board->getConfig().name;
    Serial.printf("Initializing board (%s)\n", g_board_name);
    board->init();
    if (!board->begin()) {
        Serial.println("FATAL: board->begin() failed - LCD/touch/IO-expander init error");
        while (true) { delay(1000); }
    }

    lvgl_port_init(board->getLCD(), board->getTouch());
    lvgl_port_lock(-1);
    build_main_screen();
    build_settings_screen();
    lv_scr_load(scr_main);
    lvgl_port_unlock();

    Serial.printf("Setup done. Burden %.0f ohm, press threshold %.1f mV, volts %s.\n", BURDEN_OHM,
                  PRESS_THRESHOLD_MV, VOLT_CALIBRATED ? "calibrated" : "PROVISIONAL");
    g_rescan_requested = true;
}

void loop()
{
    static uint32_t last_poll_ms = 0, last_diag_ms = 0, last_clock_ms = 0, last_hold_ui_s = 0;

    if (g_press.active) {
        float v;
        if (!ads_read_volts(MUX_DIFF_01, PGA_SIGNAL, &v)) return;
        float mv = v * 1000.0f;
        if (fabsf(mv) < PRESS_THRESHOLD_MV) {
            if (++g_press.below >= RELEASE_SAMPLES) press_end();
            return;
        }
        g_press.below = 0;
        press_sample(mv);
        return;   // sample again immediately
    }

    if (g_rescan_requested) {
        g_rescan_requested = false;
        run_scan();
    }
    if (g_rtc_write_req) {
        g_rtc_write_req = false;
        bool ok = rtc_write(g_set_hh, g_set_mm);
        Serial.printf("[rtc] set %02d:%02d -> %s\n", g_set_hh, g_set_mm, ok ? "ok" : "FAILED");
        last_clock_ms = 0;   // refresh immediately
    }

    uint32_t now = millis();
    if (now - last_poll_ms >= IDLE_POLL_MS) {
        last_poll_ms = now;
        float v;
        if (ads_read_volts(MUX_DIFF_01, PGA_SIGNAL, &v) && fabsf(v * 1000.0f) >= PRESS_THRESHOLD_MV) {
            press_start(v * 1000.0f);
            press_sample(v * 1000.0f);
            return;
        }
    }

    if (g_display_ms) {
        uint32_t age = now - g_display_ms;
        if (age >= DISPLAY_HOLD_MS) {
            clear_value();
        } else {
            uint32_t left = (DISPLAY_HOLD_MS - age + 999) / 1000;
            if (left != last_hold_ui_s) {
                last_hold_ui_s = left;
                char t[48];
                snprintf(t, sizeof(t), "Held reading - clears in %lu s", (unsigned long)left);
                set_state_text(t);
            }
        }
    }

    if (now - last_clock_ms >= CLOCK_PERIOD_MS || last_clock_ms == 0) {
        last_clock_ms = now;
        update_clock();
    }
    if (now - last_diag_ms >= DIAG_PERIOD_MS) {
        last_diag_ms = now;
        run_diagnostics();
    }
    delay(5);
}
