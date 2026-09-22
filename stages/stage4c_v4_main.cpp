/*
 * Dethleffs Globebus control panel - STAGE 4c firmware: read the Toptron gauge output through a burden resistor
 * Board : Waveshare ESP32-S3-Touch-LCD-4.3B (M-053)
 *
 * Toptron facts (CLAUDE.md, Stage 4 notes):
 *   - The gauge output exists ONLY while a momentary selector is held (starter/leisure rocker, fresh/grey rocker).
 *   - It is a ~mA gauge drive with high source resistance, not a stiff voltage. The original gauge is 85.8 ohm.
 *   - Battery channels are steady while held; tank channels decay slowly while held.
 *
 * Hardware (perfboard, van configuration):
 *   SIG+ --+-- 4.7k -- A0,  100 nF A0 -> board GND
 *          |
 *        R_burden (499 ohm interim; final 82-100 ohm)
 *          |
 *   SIG- --+-- 4.36k -- A1, 100 nF A1 -> board GND
 *   Board GND on its own wire to vehicle ground (NOT the Toptron ground / signal -).
 *
 * What this firmware does:
 *   - Polls A0-A1 (differential) every ~50 ms. |V| > PRESS_THRESHOLD_MV = a selector is held.
 *   - While held: samples as fast as the ADS1115 allows at 128 SPS (~100 samples/s), logs every sample for the first
 *     LOG_FAST_MS, then every 100 ms. Records the value at 0.2 / 0.5 / 1 / 2 s, the peak and the last value.
 *   - On release: prints a one-line summary; the screen keeps a table of the last HISTORY_ROWS presses so one
 *     photo covers a whole series. It does NOT know WHICH selector was pressed (selector sensing comes later) -
 *     the user notes the order. "end" = value ~0.2 s before release (the release edge itself is discarded).
 *   - When idle: every second reads A0 and A1 single-ended (diagnostics; A1 = signal- offset vs board GND).
 *   - Main display (user decision): the board does NOT know which selector is pressed, so every press is shown
 *     BOTH as battery volts and as tank percent, side by side, same size. The user reads the one that applies.
 *     Displayed value = max |mV| in the window 0.1-0.5 s after press start (steady value for batteries/clean,
 *     early value for grey before its decay); kept on screen for DISPLAY_HOLD_MS after release.
 *     Volts: PROVISIONAL 2-point line (499 ohm burden).
 *     Percent: from the ORIGINAL GAUGE DIAL - one needle, two aligned linear scales: 8/10/12/14/16 V on top,
 *     0/(1/4)/(1/2)/(3/4)/(1/1) below -> % = (V_scale - 8) / 8 * 100. So one battery calibration gives both.
 *
 * ADS1115 (TI SBAS444): single-shot conversions, OS-bit polling, comparator off.
 *   PGA field (bits 11:9): 000 +-6.144, 001 +-4.096, 010 +-2.048, 011 +-1.024, 100 +-0.512, 101 +-0.256 V.
 *   Differential A0-A1 at PGA +-1.024 V (31.25 uV/LSB): with 499 ohm the signal reached >0.512 V (clipped at
 *   +-0.512 V on one channel, >1.03 mA). With the final ~86-100 ohm burden, switch to PGA +-0.256 V.
 *   Single-ended diagnostics at PGA +-4.096 V (125 uV/LSB).
 *
 * I2C access: same rules as Stage 2 (CLAUDE.md "I2C bus access"): the display library's legacy driver on port 0,
 * i2c_master_* helpers only, no Wire, timeout 1000 ms (loop() can be starved ~80 ms by the LVGL task).
 * LVGL printf has no float support: floats are formatted with snprintf before being handed to LVGL.
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

static constexpr uint32_t I2C_TIMEOUT_MS = 1000;    // well above the ~80 ms loop() starvation measured in Stage 2

static constexpr uint8_t ADS_ADDR     = 0x48;
static constexpr uint8_t ADS_REG_CONV = 0x00;
static constexpr uint8_t ADS_REG_CFG  = 0x01;

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
static constexpr uint32_t LOG_FAST_MS          = 3000;    // log every sample during the first 3 s of a press
static constexpr uint32_t LOG_SLOW_MS          = 100;
static constexpr uint32_t UI_PERIOD_MS         = 100;

// Display value capture window and hold
static constexpr uint32_t WIN_START_MS    = 100;
static constexpr uint32_t WIN_END_MS      = 500;
static constexpr uint32_t DISPLAY_HOLD_MS = 60000;

// PROVISIONAL volts calibration, 499 ohm burden: V = VOLT_OFFSET + VOLT_PER_MV * mV
// from (265.3 mV, 12.82 V leisure, 2026-09-22) and (242.6 mV, 12.62 V car; battery V from 2026-09-21).
// Redo in Stage 5 with the final ~86 ohm burden.
static constexpr bool  VOLT_CALIBRATED = false;
static constexpr float VOLT_PER_MV     = (12.82f - 12.62f) / (265.3f - 242.6f);   // ~0.00881 V/mV
static constexpr float VOLT_OFFSET     = 12.82f - 265.3f * VOLT_PER_MV;           // ~10.48 V at 0 mV

// Tank percent from the gauge dial: 8 V = empty (0), 16 V = full (1/1), linear in between.
static constexpr float DIAL_V_EMPTY    = 8.0f;
static constexpr float DIAL_V_FULL     = 16.0f;

static constexpr int      HISTORY_ROWS         = 6;
static constexpr int      END_BACK_SAMPLES     = 20;      // ~0.2 s at ~100 samples/s

static constexpr uint8_t ADDR_FIRST = 0x08;
static constexpr uint8_t ADDR_LAST  = 0x77;

// ---------- state ------------------------------------------------------------

static const char *g_board_name = "?";
static lv_obj_t *lbl_uptime  = nullptr;
static lv_obj_t *lbl_scan    = nullptr;
static lv_obj_t *lbl_live    = nullptr;
static lv_obj_t *lbl_volt    = nullptr;
static lv_obj_t *lbl_pct     = nullptr;
static lv_obj_t *lbl_valnote = nullptr;
static lv_obj_t *lbl_summary = nullptr;
static lv_obj_t *lbl_diag    = nullptr;

static volatile bool g_rescan_requested = false;
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

// Recent samples, to report the value shortly before release instead of the release edge
static float g_recent[END_BACK_SAMPLES + 1];
static int   g_recent_count = 0;
static int   g_recent_head  = 0;

struct HistoryRow {
    uint32_t n;
    float    dur_s, first, at[4], peak, end;
};
static HistoryRow g_hist[HISTORY_ROWS];
static int        g_hist_count = 0;

static uint32_t g_display_ms = 0;      // when the displayed value was published (0 = nothing shown)

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

// ---------- press handling ---------------------------------------------------

static void fmt_opt(char *buf, size_t n, float mv)
{
    if (isnan(mv)) snprintf(buf, n, "--");
    else           snprintf(buf, n, "%.2f", mv);
}

static void show_live(float mv, uint32_t t_ms)
{
    char line[96];
    snprintf(line, sizeof(line), "PRESSED  live %.2f mV   (%.3f mA)   t = %.1f s", mv, mv / BURDEN_OHM,
             t_ms / 1000.0f);
    lvgl_port_lock(-1);
    lv_label_set_text(lbl_live, line);
    lv_obj_set_style_text_color(lbl_live, lv_color_hex(0xFFD54F), 0);   // live line only; big values = window
    lvgl_port_unlock();
}

static void show_value(float mv)
{
    char v[24], p[24], note[160];
    float volts = VOLT_OFFSET + VOLT_PER_MV * mv;
    snprintf(v, sizeof(v), "%.1f V", volts);
    float pct = (volts - DIAL_V_EMPTY) / (DIAL_V_FULL - DIAL_V_EMPTY) * 100.0f;
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    snprintf(p, sizeof(p), "%.0f %%", pct);
    snprintf(note, sizeof(note), "from %.2f mV (max 0.1-0.5 s)   %s   %% = (V-8)/8 per gauge dial", mv,
             VOLT_CALIBRATED ? "calibrated" : "PROVISIONAL (uncal)");
    lvgl_port_lock(-1);
    lv_label_set_text(lbl_volt, v);
    lv_label_set_text(lbl_pct, p);
    lv_label_set_text(lbl_valnote, note);
    lv_obj_set_style_text_color(lbl_volt, lv_color_white(), 0);
    lv_obj_set_style_text_color(lbl_pct, lv_color_white(), 0);
    lvgl_port_unlock();
    g_display_ms = millis();
    if (g_display_ms == 0) g_display_ms = 1;
}

static void clear_value()
{
    lvgl_port_lock(-1);
    lv_label_set_text(lbl_volt, "-- V");
    lv_label_set_text(lbl_pct, "-- %");
    lv_label_set_text(lbl_valnote, "press a selector on the Toptron panel");
    lv_obj_set_style_text_color(lbl_volt, lv_color_hex(0x607D8B), 0);
    lv_obj_set_style_text_color(lbl_pct, lv_color_hex(0x607D8B), 0);
    lvgl_port_unlock();
    g_display_ms = 0;
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
    for (float &v : g_press.at_mv) v = NAN;
    g_press.win_mv = NAN;
    g_press.shown  = false;
    g_recent_count = 0;
    g_recent_head  = 0;
    Serial.printf("\n[press] #%lu START  first %.3f mV\n", (unsigned long)g_press.n, mv);
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
        // Short press: use whatever the window caught, else the peak
        show_value(!isnan(g_press.win_mv) ? g_press.win_mv : g_press.peak_mv);
        g_press.shown = true;
    }
    uint32_t dur = millis() - g_press.start_ms;
    // Oldest sample in the ring = ~END_BACK_SAMPLES before the last one above threshold
    float end_mv = g_recent_count > END_BACK_SAMPLES ? g_recent[g_recent_head] : g_press.first_mv;

    char a[4][12];
    for (int i = 0; i < 4; i++) fmt_opt(a[i], sizeof(a[i]), g_press.at_mv[i]);
    Serial.printf("[press] #%lu END  %.2f s, %lu samples | first %.2f  @0.2s %s  @0.5s %s  @1s %s  @2s %s  "
                  "peak %.2f  end(-0.2s) %.2f  edge %.2f mV\n",
                  (unsigned long)g_press.n, dur / 1000.0f, (unsigned long)g_press.samples, g_press.first_mv, a[0],
                  a[1], a[2], a[3], g_press.peak_mv, end_mv, g_press.last_mv);

    // Shift history down, newest on top
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
    lv_label_set_text(lbl_summary, text);
    lv_label_set_text(lbl_live, "No selector pressed");
    lv_obj_set_style_text_color(lbl_live, lv_color_hex(0x90A4AE), 0);
    lvgl_port_unlock();
}

static void run_diagnostics()
{
    float a0 = NAN, a1 = NAN;
    bool ok0 = ads_read_volts(MUX_AIN0, PGA_4V096, &a0);
    bool ok1 = ads_read_volts(MUX_AIN1, PGA_4V096, &a1);
    char s0[16], s1[16], line[160];
    if (ok0) snprintf(s0, sizeof(s0), "%.4f", a0); else snprintf(s0, sizeof(s0), "ERR");
    if (ok1) snprintf(s1, sizeof(s1), "%.4f", a1); else snprintf(s1, sizeof(s1), "ERR");
    snprintf(line, sizeof(line), "Idle diagnostics: A0 %s V   A1 %s V (signal- vs board GND)   I2C/ADC errors: %lu",
             s0, s1, (unsigned long)g_read_errors);
    lvgl_port_lock(-1);
    lv_label_set_text(lbl_diag, line);
    lv_obj_set_style_text_color(lbl_diag, g_read_errors ? lv_color_hex(0xFF5252) : lv_color_hex(0xB0BEC5), 0);
    lvgl_port_unlock();
}

// ---------- UI ---------------------------------------------------------------

static lv_obj_t *make_label(lv_obj_t *parent, const lv_font_t *font, lv_color_t color)
{
    lv_obj_t *l = lv_label_create(parent);
    lv_obj_set_style_text_font(l, font, 0);
    lv_obj_set_style_text_color(l, color, 0);
    return l;
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
    lv_label_set_text_fmt(lbl_uptime, "Uptime: %02lu:%02lu:%02lu    Free heap: %lu B    Presses: %lu",
                          (unsigned long)(s / 3600), (unsigned long)((s / 60) % 60), (unsigned long)(s % 60),
                          (unsigned long)ESP.getFreeHeap(), (unsigned long)g_press.n);
}

static void build_ui()
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x101418), 0);
    const lv_color_t grey = lv_color_hex(0xB0BEC5);

    lv_obj_t *title = make_label(scr, &lv_font_montserrat_30, lv_color_white());
    lv_label_set_text(title, "STAGE 4c  -  TOPTRON OUTPUT");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 24);

    lv_obj_t *lbl_board = make_label(scr, &lv_font_montserrat_16, lv_color_hex(0x80DEEA));
    lv_label_set_text_fmt(lbl_board, "Board profile: %s    Built: " __DATE__ " " __TIME__, g_board_name);
    lv_obj_align(lbl_board, LV_ALIGN_TOP_LEFT, 40, 70);

    lbl_uptime = make_label(scr, &lv_font_montserrat_16, grey);
    lv_obj_align(lbl_uptime, LV_ALIGN_TOP_LEFT, 40, 94);

    lbl_scan = make_label(scr, &lv_font_montserrat_16, grey);
    lv_label_set_long_mode(lbl_scan, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_scan, 720);
    lv_label_set_text(lbl_scan, "I2C scan: waiting...");
    lv_obj_align(lbl_scan, LV_ALIGN_TOP_LEFT, 40, 118);

    lbl_volt = make_label(scr, &lv_font_montserrat_48, lv_color_hex(0x607D8B));
    lv_obj_set_width(lbl_volt, 340);
    lv_obj_set_style_text_align(lbl_volt, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(lbl_volt, "-- V");
    lv_obj_align(lbl_volt, LV_ALIGN_TOP_LEFT, 40, 162);

    lv_obj_t *sep = make_label(scr, &lv_font_montserrat_48, lv_color_hex(0x455A64));
    lv_label_set_text(sep, "|");
    lv_obj_align(sep, LV_ALIGN_TOP_MID, 0, 162);

    lbl_pct = make_label(scr, &lv_font_montserrat_48, lv_color_hex(0x607D8B));
    lv_obj_set_width(lbl_pct, 340);
    lv_obj_set_style_text_align(lbl_pct, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(lbl_pct, "-- %");
    lv_obj_align(lbl_pct, LV_ALIGN_TOP_RIGHT, -40, 162);

    lbl_valnote = make_label(scr, &lv_font_montserrat_14, lv_color_hex(0x90A4AE));
    lv_label_set_text(lbl_valnote, "press a selector on the Toptron panel");
    lv_obj_align(lbl_valnote, LV_ALIGN_TOP_MID, 0, 220);

    lbl_live = make_label(scr, &lv_font_montserrat_16, lv_color_hex(0x90A4AE));
    lv_label_set_text(lbl_live, "No selector pressed");
    lv_obj_align(lbl_live, LV_ALIGN_TOP_LEFT, 40, 244);

    lbl_summary = make_label(scr, &lv_font_montserrat_14, lv_color_hex(0xB0BEC5));
    lv_label_set_long_mode(lbl_summary, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_summary, 720);
    lv_label_set_text(lbl_summary, "Hold a selector on the Toptron panel.");
    lv_obj_align(lbl_summary, LV_ALIGN_TOP_LEFT, 40, 268);

    lbl_diag = make_label(scr, &lv_font_montserrat_16, grey);
    lv_label_set_text(lbl_diag, "Idle diagnostics: --");
    lv_obj_align(lbl_diag, LV_ALIGN_TOP_LEFT, 40, 428);

    lv_obj_t *btn = lv_btn_create(scr);
    lv_obj_set_size(btn, 150, 44);
    lv_obj_align(btn, LV_ALIGN_TOP_RIGHT, -30, 20);
    lv_obj_add_event_cb(btn, rescan_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *btn_txt = make_label(btn, &lv_font_montserrat_20, lv_color_white());
    lv_label_set_text(btn_txt, "RESCAN I2C");
    lv_obj_center(btn_txt);

    lv_timer_create(status_timer_cb, 1000, nullptr);
    status_timer_cb(nullptr);
}

// ---------- Arduino entry points ---------------------------------------------

void setup()
{
    Serial.begin(115200);
    delay(1500);   // give USB-CDC time to enumerate so early logs are not lost
    Serial.println("\n=== Dethleffs panel STAGE 4c (Toptron output via burden resistor) ===");

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
    build_ui();
    lvgl_port_unlock();

    Serial.printf("Setup done. Burden %.0f ohm, press threshold %.1f mV.\n", BURDEN_OHM, PRESS_THRESHOLD_MV);
    g_rescan_requested = true;
}

void loop()
{
    static uint32_t last_poll_ms = 0, last_diag_ms = 0, last_ui_ms = 0;

    if (g_rescan_requested && !g_press.active) {
        g_rescan_requested = false;
        run_scan();
    }

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
        if (millis() - last_ui_ms >= UI_PERIOD_MS) {
            last_ui_ms = millis();
            show_live(mv, millis() - g_press.start_ms);
        }
        return;   // sample again immediately
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
    if (g_display_ms && now - g_display_ms >= DISPLAY_HOLD_MS) clear_value();
    if (now - last_diag_ms >= DIAG_PERIOD_MS) {
        last_diag_ms = now;
        run_diagnostics();
    }
    delay(5);
}
