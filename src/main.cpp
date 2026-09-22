/*
 * Dethleffs Globebus control panel - firmware: Toptron gauge reader + Globebus UI + Wi-Fi/weather/clock
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
 *   - While held: ~100 samples/s. Displayed value = max |mV| in 0.1-0.5 s after press start; held DISPLAY_HOLD_MS.
 *   - The board does NOT know which selector is pressed (user decision): every press is shown both as volts and as
 *     percent, same size; the user reads the one that applies. Volts turn red in the dial's red zones.
 *     How long that value stays after the selector is released is set in Settings > Display ("Keep reading":
 *     until release / 5 / 15 / 60 s, default 5) - a held value must not look like a live one.
 *
 * UI (user-approved mockups, 2026-09-22) - three pages, swipe left/right (tileview), page dots at the bottom:
 *   1. Gauges: "Globebus", clock, Wi-Fi/BT icons, Toptron dot; volts left / percent right; battery scale 8-16 V with
 *      the dial's zones and the water wedge scale below, markers aligned like one needle; state, badge, settings.
 *   2. Weather (open-meteo.com via net.cpp): current (icon, temp, condition, feels, wind, humidity, min/max),
 *      today + 3 days, "updated" footer; offline message when no data.
 *   3. Big clock: HH:MM and "Tuesday, 22 Sep 2026".
 *   Settings (gear on page 1): tabs Clock (time + date with +/-, internet-time status), Wi-Fi (switch, scan, list,
 *   password keyboard), Bluetooth (switch), Weather (automatic IP location / manual place), Diagnostics.
 *
 * Display timeout (Stage 6 requirement, image retention + power): after the configured minutes without touch or
 * Toptron press, a BLACK screen is loaded AND the backlight is switched off (CH422G EXIO2 via the library's
 * Backlight; on/off only). Black, not just dark: the LCD would otherwise keep driving the static image. Wake: any
 * touch (swallowed by the black screen, so no button fires) or a Toptron press (value shown at once).
 * Timeout in Settings > Display (Never/1/2/5/10/30 min, default 5), saved in NVS "globebus"/"disp_to".
 *
 * Time: TZ Europe/Bucharest (GLOBEBUS_TZ). At boot the system clock is set from the RTC; NTP (when on Wi-Fi) sets the
 * system clock and loop() then writes it to the RTC. No coin cell is fitted yet -> RTC loses time at power-off.
 *
 * PCF85063A RTC (0x51), register map per NXP datasheet (from memory; confirmed by the running clock):
 *   00h Control_1 (bit1 12_24: 0 = 24 h), 04h Seconds (bit7 OS = time invalid), 05h Minutes, 06h Hours,
 *   07h Days, 08h Weekdays (0-6), 09h Months (1-12), 0Ah Years (0-99, 2000-based). All BCD.
 *
 * ADS1115 (TI SBAS444): single-shot, OS-bit polling, comparator off. PGA bits 11:9: 001 +-4.096, 011 +-1.024,
 *   101 +-0.256 V. Signal at +-1.024 V with the 499 ohm interim burden; +-0.256 V once the ~86 ohm burden is fitted.
 *
 * I2C: the display library's legacy driver on port 0, i2c_master_* helpers only, no Wire, timeout 1000 ms.
 * LVGL is only touched from loop()/LVGL callbacks (core 1); the network task never calls LVGL.
 * LVGL printf has no float support: floats are formatted with snprintf before being handed to LVGL.
 * Fonts in src/fonts/ generated with lv_font_conv 1.5.2 from LVGL's bundled Montserrat-Medium / FontAwesome 5.
 */

#include <Arduino.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>
#include <driver/i2c.h>
#include <sys/time.h>
#include <esp_heap_caps.h>
#include "lvgl_v8_port.h"
#include "net.h"
#include <Preferences.h>

using namespace esp_panel::drivers;
using namespace esp_panel::board;

LV_FONT_DECLARE(font_num_88);
LV_FONT_DECLARE(font_clock_44);
LV_FONT_DECLARE(font_clock_170);
LV_FONT_DECLARE(font_icon_44);
LV_FONT_DECLARE(font_wx_80);
LV_FONT_DECLARE(font_wx_34);
LV_FONT_DECLARE(font_sym_22);

// FontAwesome 5 glyphs (UTF-8)
#define FA_BATTERY  "\xEF\x89\x82"   // U+F242
#define FA_TINT     "\xEF\x81\x83"   // U+F043
#define FA_SUN      "\xEF\x86\x85"   // U+F185
#define FA_MOON     "\xEF\x86\x86"   // U+F186
#define FA_CLOUD    "\xEF\x83\x82"   // U+F0C2
#define FA_CLOUDSUN "\xEF\x9B\x84"   // U+F6C4
#define FA_CLOUDMN  "\xEF\x9B\x83"   // U+F6C3
#define FA_RAIN     "\xEF\x9C\xBD"   // U+F73D
#define FA_SHOWERS  "\xEF\x9D\x80"   // U+F740
#define FA_SNOW     "\xEF\x8B\x9C"   // U+F2DC
#define FA_BOLT     "\xEF\x83\xA7"   // U+F0E7
#define FA_SMOG     "\xEF\x9D\x9F"   // U+F75F
#define FA_WIND     "\xEF\x9C\xAE"   // U+F72E
#define FA_ARROWSV  "\xEF\x8C\xB8"   // U+F338
#define FA_WIFI     "\xEF\x87\xAB"   // U+F1EB
#define FA_BT       "\xEF\x8A\x94"   // U+F294

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
static constexpr int      CONFIRM_SAMPLES      = 3;       // consecutive positive samples before a press counts
static constexpr uint32_t MIN_PRESS_MS         = 150;     // shorter "presses" are noise: logged, not displayed
static constexpr uint32_t BOOT_IGNORE_MS       = 3000;    // power-up transient: the board's inrush current dips the
                                                          // shared Toptron supply (seen in the van: 16.8 V "reading"
                                                          // with nothing pressed, ~717 mV, right after boot)
static constexpr int      IDLE_CONFIRM_SAMPLES = 3;       // quiet readings required before a new press may start
static constexpr float    PLAUSIBLE_MAX_V      = 16.5f;   // above the dial's top (16 V) nothing real can be shown
// Toptron output is POSITIVE (SIG+ on A0). Negative values are pickup on loose/floating leads (seen on the bench
// with Wi-Fi active: 63 phantom presses of -5...-184 mV) and are never treated as a press.
static constexpr int      RELEASE_SAMPLES      = 3;       // consecutive samples below threshold = released
static constexpr uint32_t IDLE_POLL_MS         = 50;
static constexpr uint32_t DIAG_PERIOD_MS       = 1000;
static constexpr uint32_t CLOCK_PERIOD_MS      = 1000;
static constexpr uint32_t NET_UI_PERIOD_MS     = 300;
static constexpr uint32_t LOG_FAST_MS          = 3000;    // serial: every sample during the first 3 s of a press
static constexpr uint32_t LOG_SLOW_MS          = 100;

// Display value capture window and hold
static constexpr uint32_t WIN_START_MS    = 100;
static constexpr uint32_t WIN_END_MS      = 500;
static const uint16_t HOLD_SECONDS[] = {0, 5, 15, 60};        // 0 = clear as soon as the selector is released
static const char    *HOLD_OPTS      = "Until release\n5 s\n15 s\n60 s";
static constexpr int  HOLD_DEFAULT   = 1;                      // 5 s
static constexpr uint32_t ERROR_FRESH_MS = 5UL * 60UL * 1000UL;  // status dot shows recent errors, not a latch

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

static constexpr time_t TIME_VALID_AFTER = 1704067200;

static const uint16_t TIMEOUT_MIN[]   = {0, 1, 2, 5, 10, 30};           // 0 = never
static const char    *TIMEOUT_OPTS    = "Never\n1 min\n2 min\n5 min\n10 min\n30 min";
static constexpr int  TIMEOUT_DEFAULT = 3;                              // index: 5 min
static constexpr uint32_t SCREEN_CHECK_MS = 250;   // 2024-01-01: anything earlier = clock not set

// ---------- UI geometry (800 x 480) ------------------------------------------

static constexpr int SCALE_X     = 92;            // bar left edge
static constexpr int SCALE_W     = 684;           // bar width
static constexpr int SCALE_H     = 22;
static constexpr int BAT_BAR_Y   = 262;
static constexpr int WAT_BAR_Y   = 348;
static constexpr int MARKER_W    = 20;
static constexpr int MARKER_H    = 13;

static const lv_color_t COL_BG      = lv_color_hex(0x141A20);
static const lv_color_t COL_PANEL   = lv_color_hex(0x1C242C);
static const lv_color_t COL_TEXT    = lv_color_hex(0xE8EDF1);
static const lv_color_t COL_MUTED   = lv_color_hex(0x8FA1AD);
static const lv_color_t COL_DIM     = lv_color_hex(0x4E5D68);
static const lv_color_t COL_RED     = lv_color_hex(0xB33A3A);
static const lv_color_t COL_GREEN   = lv_color_hex(0x4F7D1F);
static const lv_color_t COL_BLUE    = lv_color_hex(0x2F6FB0);
static const lv_color_t COL_TRACK   = lv_color_hex(0x26313A);
static const lv_color_t COL_WARN    = lv_color_hex(0xFF5A52);
static const lv_color_t COL_OK      = lv_color_hex(0x639922);
static const lv_color_t COL_BTN     = lv_color_hex(0x2A3640);

// ---------- state ------------------------------------------------------------

static const char *g_board_name = "?";
static Board      *g_board      = nullptr;

// display timeout
static lv_obj_t *scr_blank     = nullptr;
static lv_obj_t *scr_prev      = nullptr;
static lv_obj_t *dd_timeout    = nullptr;
static bool      g_screen_off  = false;
static int       g_timeout_idx = TIMEOUT_DEFAULT;
static lv_obj_t *dd_hold       = nullptr;
static int       g_hold_idx    = HOLD_DEFAULT;

// page 1 (gauges)
static lv_obj_t *scr_main    = nullptr;
static lv_obj_t *lbl_clock   = nullptr;
static lv_obj_t *lbl_icons   = nullptr;
static lv_obj_t *lbl_toptron = nullptr;
static lv_obj_t *dot_status  = nullptr;
static lv_obj_t *lbl_volt    = nullptr;
static lv_obj_t *lbl_pct     = nullptr;
static lv_obj_t *mk_bat      = nullptr;
static lv_obj_t *mk_wat      = nullptr;
static lv_obj_t *lbl_state   = nullptr;
static lv_obj_t *pill_prov   = nullptr;

// page 2 (weather)
static lv_obj_t *lbl_wx_clock = nullptr;
static lv_obj_t *lbl_wx_place = nullptr;
static lv_obj_t *lbl_wx_icon  = nullptr;
static lv_obj_t *lbl_wx_temp  = nullptr;
static lv_obj_t *lbl_wx_cond  = nullptr;
static lv_obj_t *lbl_wx_det[3] = {};
static lv_obj_t *lbl_wx_day[WX_DAYS][4] = {};   // label, icon, temps, pop
static lv_obj_t *lbl_wx_foot  = nullptr;
static lv_obj_t *lbl_wx_msg   = nullptr;
static lv_obj_t *wx_content   = nullptr;

// page 3 (big clock)
static lv_obj_t *lbl_big_clock = nullptr;
static lv_obj_t *lbl_big_date  = nullptr;

// settings
static lv_obj_t *scr_set     = nullptr;
static lv_obj_t *lbl_set_f[5] = {};             // hh, mm, dd, mon, yyyy
static lv_obj_t *lbl_rtc     = nullptr;
static lv_obj_t *sw_wifi     = nullptr;
static lv_obj_t *lbl_wifi    = nullptr;
static lv_obj_t *list_wifi   = nullptr;
static lv_obj_t *sw_ble      = nullptr;
static lv_obj_t *lbl_ble     = nullptr;
static lv_obj_t *sw_loc      = nullptr;
static lv_obj_t *lbl_loc     = nullptr;
static lv_obj_t *btn_place   = nullptr;
static lv_obj_t *lbl_scan    = nullptr;
static lv_obj_t *lbl_hist    = nullptr;
static lv_obj_t *lbl_diag    = nullptr;
static lv_obj_t *lbl_netst   = nullptr;

// keyboard modal
static lv_obj_t *kb_modal    = nullptr;
static lv_obj_t *kb_ta       = nullptr;
static void (*kb_done)(const char *) = nullptr;
static char g_pending_ssid[33];

static volatile bool g_rescan_requested = false;
static volatile bool g_rtc_write_req    = false;
static volatile bool g_ntp_synced       = false;
static int           g_set[5] = {12, 0, 22, 9, 2026};   // hh, mm, dd, mon, yyyy (manual clock set)
static uint32_t      g_read_errors      = 0;
static uint32_t      g_last_error_ms    = 0;       // millis() of the most recent I2C/ADC error (0 = none yet)
static time_t        g_last_error_time  = 0;       // wall clock of that error, for the diagnostics page

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

static int      g_idle_ok    = 0;                 // consecutive quiet readings (see IDLE_CONFIRM_SAMPLES)
static uint32_t g_rejected   = 0;                 // implausible values not displayed
static uint32_t g_display_ms = 0;                 // when the displayed value was published (0 = nothing shown)
static NetState g_ns;                             // last copy of the network state (loop task only)

static const char *const MONTHS[12] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                                       "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
static const char *const WEEKDAYS[7] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

// ---------- small helpers ----------------------------------------------------

static uint8_t bcd2bin(uint8_t v) { return (v >> 4) * 10 + (v & 0x0F); }
static uint8_t bin2bcd(uint8_t v) { return ((v / 10) << 4) | (v % 10); }
static float   clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

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

static lv_obj_t *make_button(lv_obj_t *parent, const char *text, int w, int h, lv_event_cb_t cb, void *ud)
{
    lv_obj_t *b = lv_btn_create(parent);
    lv_obj_set_size(b, w, h);
    lv_obj_set_style_bg_color(b, COL_BTN, 0);
    lv_obj_set_style_shadow_width(b, 0, 0);
    lv_obj_add_event_cb(b, cb, LV_EVENT_CLICKED, ud);
    lv_obj_t *l = make_label(b, &lv_font_montserrat_22, COL_TEXT);
    lv_label_set_text(l, text);
    lv_obj_center(l);
    return b;
}

static const char *wx_icon(int code, bool day)
{
    switch (code) {
    case 0: case 1: return day ? FA_SUN : FA_MOON;
    case 2: return day ? FA_CLOUDSUN : FA_CLOUDMN;
    case 3: return FA_CLOUD;
    case 45: case 48: return FA_SMOG;
    case 65: case 82: return FA_SHOWERS;
    case 71: case 73: case 75: case 77: case 85: case 86: return FA_SNOW;
    case 95: case 96: case 99: return FA_BOLT;
    default: return (code >= 51 && code <= 81) ? FA_RAIN : FA_CLOUD;
    }
}

static const char *compass(int deg)
{
    static const char *const d[8] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
    if (deg < 0) return "";
    return d[((deg + 22) % 360) / 45];
}

static bool time_valid() { return time(nullptr) > TIME_VALID_AFTER; }

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
    bool recent_error = g_last_error_ms && (millis() - g_last_error_ms) < ERROR_FRESH_MS;
    lv_obj_set_style_bg_color(dot_status, (pass && !recent_error) ? COL_OK : COL_WARN, 0);
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
    g_last_error_ms = millis();
    if (g_last_error_ms == 0) g_last_error_ms = 1;
    g_last_error_time = time(nullptr);
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

// ---------- RTC (PCF85063A) and system time ----------------------------------

// Reads date + time. Returns false if the read failed or the oscillator-stop flag marks the time invalid.
static bool rtc_read(struct tm *t, bool *osc_stopped)
{
    uint8_t reg = 0x04, b[7] = {};
    if (i2c_master_write_read_device(I2C_PORT, RTC_ADDR, &reg, 1, b, sizeof(b), pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != ESP_OK)
        return false;
    *osc_stopped = b[0] & 0x80;
    memset(t, 0, sizeof(*t));
    t->tm_sec   = bcd2bin(b[0] & 0x7F);
    t->tm_min   = bcd2bin(b[1] & 0x7F);
    t->tm_hour  = bcd2bin(b[2] & 0x3F);
    t->tm_mday  = bcd2bin(b[3] & 0x3F);
    t->tm_wday  = b[4] & 0x07;
    t->tm_mon   = bcd2bin(b[5] & 0x1F) - 1;
    t->tm_year  = bcd2bin(b[6]) + 100;
    t->tm_isdst = -1;
    return !*osc_stopped;
}

static bool rtc_write(const struct tm *t)
{
    // Keep the hours register in 24 h mode (Control_1 bit1 = 0); other Control_1 bits untouched.
    uint8_t reg = 0x00, c1 = 0;
    if (i2c_master_write_read_device(I2C_PORT, RTC_ADDR, &reg, 1, &c1, 1, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != ESP_OK)
        return false;
    if (c1 & 0x02) {
        uint8_t w[2] = {0x00, (uint8_t)(c1 & ~0x02)};
        if (i2c_master_write_to_device(I2C_PORT, RTC_ADDR, w, 2, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != ESP_OK) return false;
    }
    uint8_t buf[8] = {0x04,
                      bin2bcd(t->tm_sec) /* bit7 OS = 0 */, bin2bcd(t->tm_min), bin2bcd(t->tm_hour),
                      bin2bcd(t->tm_mday), (uint8_t)t->tm_wday, bin2bcd(t->tm_mon + 1),
                      bin2bcd((t->tm_year - 100) % 100)};
    return i2c_master_write_to_device(I2C_PORT, RTC_ADDR, buf, sizeof(buf), pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == ESP_OK;
}

static void set_system_time(struct tm *local)
{
    time_t e = mktime(local);       // local time (TZ set) -> epoch; also normalises tm_wday
    struct timeval tv = {e, 0};
    settimeofday(&tv, nullptr);
}

static void system_time_from_rtc()
{
    struct tm t;
    bool osc = false;
    bool ok = rtc_read(&t, &osc);
    if (ok && t.tm_year >= 124) {
        set_system_time(&t);
        Serial.printf("[rtc] system time set from RTC: %04d-%02d-%02d %02d:%02d:%02d\n", t.tm_year + 1900,
                      t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec);
    } else {
        Serial.printf("[rtc] no valid RTC time (%s)\n",
                      ok ? "date before 2024 - date never set" : (osc ? "oscillator was stopped - no coin cell?"
                                                                       : "read error"));
    }
}

static void rtc_from_system_time(const char *why)
{
    time_t now = time(nullptr);
    struct tm t;
    localtime_r(&now, &t);
    bool ok = rtc_write(&t);
    Serial.printf("[rtc] %s: RTC <- %04d-%02d-%02d %02d:%02d:%02d %s\n", why, t.tm_year + 1900, t.tm_mon + 1,
                  t.tm_mday, t.tm_hour, t.tm_min, t.tm_sec, ok ? "ok" : "FAILED");
}

static void on_ntp_synced() { g_ntp_synced = true; }   // runs in the network task: just flag it

static void update_clocks()
{
    char hm[8], date[40];
    if (time_valid()) {
        time_t now = time(nullptr);
        struct tm t;
        localtime_r(&now, &t);
        snprintf(hm, sizeof(hm), "%02d:%02d", t.tm_hour, t.tm_min);
        snprintf(date, sizeof(date), "%s, %d %s %d", WEEKDAYS[t.tm_wday], t.tm_mday, MONTHS[t.tm_mon], t.tm_year + 1900);
    } else {
        snprintf(hm, sizeof(hm), "--:--");
        snprintf(date, sizeof(date), "Set the clock in Settings");
    }
    lvgl_port_lock(-1);
    lv_label_set_text(lbl_clock, hm);
    lv_label_set_text(lbl_wx_clock, hm);
    lv_label_set_text(lbl_big_clock, hm);
    lv_label_set_text(lbl_big_date, date);
    lvgl_port_unlock();
}

// ---------- page 1: gauge display --------------------------------------------

static void place_marker(lv_obj_t *mk, int bar_y, float frac)
{
    int x = SCALE_X + (int)(clampf(frac, 0, 1) * SCALE_W) - MARKER_W / 2;
    lv_obj_set_pos(mk, x, bar_y - MARKER_H - 2);
    lv_obj_clear_flag(mk, LV_OBJ_FLAG_HIDDEN);
}

static void set_state_text(const char *text);

static void show_value(float mv)
{
    float volts = VOLT_OFFSET + VOLT_PER_MV * mv;
    if (volts > PLAUSIBLE_MAX_V) {                // cannot be a real gauge reading - do not display it
        g_rejected++;
        Serial.printf("[display] REJECTED %.2f mV -> %.2f V (above %.1f V; total %lu)\n", mv, volts,
                      PLAUSIBLE_MAX_V, (unsigned long)g_rejected);
        set_state_text("Ignored an implausible reading");
        return;
    }
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

// ---------- display timeout --------------------------------------------------

static void screen_off()
{
    if (g_screen_off) return;
    lvgl_port_lock(-1);
    scr_prev = lv_scr_act();
    lv_scr_load(scr_blank);
    lvgl_port_unlock();
    delay(60);                                    // let the black frame reach the panel before the light goes out
    if (g_board && g_board->getBacklight()) g_board->getBacklight()->off();
    g_screen_off = true;
    Serial.println("[screen] off (timeout)");
}

static void screen_on(const char *why)
{
    if (!g_screen_off) return;
    lvgl_port_lock(-1);
    lv_scr_load(scr_prev ? scr_prev : scr_main);
    lv_disp_trig_activity(nullptr);
    lvgl_port_unlock();
    if (g_board && g_board->getBacklight()) g_board->getBacklight()->on();
    g_screen_off = false;
    Serial.printf("[screen] on (%s)\n", why);
}

static void check_screen_timeout()
{
    lvgl_port_lock(-1);
    uint32_t idle = lv_disp_get_inactive_time(nullptr);
    lvgl_port_unlock();
    if (g_screen_off) {
        if (idle < 1000) screen_on("touch");     // a touch on the black screen resets LVGL's inactivity timer
        return;
    }
    uint16_t mins = TIMEOUT_MIN[g_timeout_idx];
    if (mins && idle >= mins * 60000UL && !g_press.active) screen_off();
}

static void hold_dd_cb(lv_event_t *e)
{
    g_hold_idx = lv_dropdown_get_selected(dd_hold);
    Preferences p;
    p.begin("globebus", false);
    p.putUChar("hold", (uint8_t)g_hold_idx);
    p.end();
    Serial.printf("[display] keep reading: %u s (0 = until release)\n", HOLD_SECONDS[g_hold_idx]);
}

static void timeout_dd_cb(lv_event_t *e)
{
    g_timeout_idx = lv_dropdown_get_selected(dd_timeout);
    Preferences p;
    p.begin("globebus", false);
    p.putUChar("disp_to", (uint8_t)g_timeout_idx);
    p.end();
    Serial.printf("[screen] timeout set to %u min (0 = never)\n", TIMEOUT_MIN[g_timeout_idx]);
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
    screen_on("Toptron press");
    lvgl_port_lock(-1);
    lv_disp_trig_activity(nullptr);              // a press counts as activity for the timeout
    lvgl_port_unlock();
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
    uint32_t dur = millis() - g_press.start_ms;
    if (!g_press.shown) {
        if (dur >= MIN_PRESS_MS) {
            show_value(!isnan(g_press.win_mv) ? g_press.win_mv : g_press.peak_mv);
        } else {
            Serial.printf("[press] #%lu ignored: %lu ms < %lu ms\n", (unsigned long)g_press.n, (unsigned long)dur,
                          (unsigned long)MIN_PRESS_MS);
            set_state_text(g_display_ms ? "Held reading" : "Hold a selector on the panel");
        }
        g_press.shown = true;
    }
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
    char s0[16], s1[16], line[300];
    if (ok0) snprintf(s0, sizeof(s0), "%.4f", a0); else snprintf(s0, sizeof(s0), "ERR");
    if (ok1) snprintf(s1, sizeof(s1), "%.4f", a1); else snprintf(s1, sizeof(s1), "ERR");
    uint32_t s = millis() / 1000;
    char err_when[40] = "";
    if (g_last_error_ms) {
        uint32_t ago = (millis() - g_last_error_ms) / 1000;
        if (g_last_error_time > TIME_VALID_AFTER) {
            struct tm e;
            localtime_r(&g_last_error_time, &e);
            snprintf(err_when, sizeof(err_when), " (last at %02d:%02d, %lu s ago)", e.tm_hour, e.tm_min,
                     (unsigned long)ago);
        } else {
            snprintf(err_when, sizeof(err_when), " (last %lu s ago)", (unsigned long)ago);
        }
    }
    snprintf(line, sizeof(line),
             "Idle: A0 %s V   A1 %s V (signal- vs board GND)   I2C/ADC errors: %lu%s\n"
             "Uptime %02lu:%02lu:%02lu   presses %lu (rejected %lu)   free heap %lu B (internal %lu B)   built " __DATE__ " " __TIME__
             "\nVolts %s: V = %.3f + %.5f x mV   %% = (V - 8) / 8 per gauge dial",
             s0, s1, (unsigned long)g_read_errors, err_when, (unsigned long)(s / 3600), (unsigned long)((s / 60) % 60),
             (unsigned long)(s % 60), (unsigned long)g_press.n, (unsigned long)g_rejected,
             (unsigned long)ESP.getFreeHeap(),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             VOLT_CALIBRATED ? "calibrated" : "PROVISIONAL", VOLT_OFFSET, VOLT_PER_MV);
    lvgl_port_lock(-1);
    lv_label_set_text(lbl_diag, line);
    bool recent_error = g_last_error_ms && (millis() - g_last_error_ms) < ERROR_FRESH_MS;
    lv_obj_set_style_text_color(lbl_diag, recent_error ? COL_WARN : COL_MUTED, 0);
    lv_obj_set_style_bg_color(dot_status, recent_error ? COL_WARN : COL_OK, 0);
    lvgl_port_unlock();
}

// ---------- keyboard modal ---------------------------------------------------

static void kb_close()
{
    if (kb_modal) {
        lv_obj_del(kb_modal);
        kb_modal = nullptr;
        kb_ta = nullptr;
    }
}

static void kb_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_READY) {
        char text[65];
        strlcpy(text, lv_textarea_get_text(kb_ta), sizeof(text));
        void (*done)(const char *) = kb_done;
        kb_close();
        if (done) done(text);
    } else if (code == LV_EVENT_CANCEL) {
        kb_close();
    }
}

static void open_keyboard(const char *title, const char *initial, bool password, void (*done)(const char *))
{
    kb_close();
    kb_done = done;
    kb_modal = lv_obj_create(lv_layer_top());
    lv_obj_remove_style_all(kb_modal);
    lv_obj_set_size(kb_modal, 800, 480);
    lv_obj_set_style_bg_color(kb_modal, COL_BG, 0);
    lv_obj_set_style_bg_opa(kb_modal, LV_OPA_COVER, 0);
    lv_obj_add_flag(kb_modal, LV_OBJ_FLAG_CLICKABLE);

    lv_obj_t *t = make_label(kb_modal, &lv_font_montserrat_22, COL_TEXT);
    lv_label_set_text(t, title);
    lv_obj_set_pos(t, 24, 20);

    kb_ta = lv_textarea_create(kb_modal);
    lv_textarea_set_one_line(kb_ta, true);
    lv_textarea_set_password_mode(kb_ta, password);
    lv_textarea_set_text(kb_ta, initial ? initial : "");
    lv_textarea_set_max_length(kb_ta, 63);
    lv_obj_set_style_text_font(kb_ta, &lv_font_montserrat_24, 0);
    lv_obj_set_size(kb_ta, 752, 56);
    lv_obj_set_pos(kb_ta, 24, 64);

    lv_obj_t *hint = make_label(kb_modal, &lv_font_montserrat_16, COL_MUTED);
    lv_label_set_text(hint, LV_SYMBOL_OK " = done      " LV_SYMBOL_KEYBOARD " = cancel");
    lv_obj_set_pos(hint, 24, 132);

    lv_obj_t *kb = lv_keyboard_create(kb_modal);
    lv_obj_set_size(kb, 800, 300);
    lv_obj_align(kb, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_text_font(kb, &lv_font_montserrat_22, 0);
    lv_keyboard_set_textarea(kb, kb_ta);
    lv_obj_add_event_cb(kb, kb_event_cb, LV_EVENT_READY, nullptr);
    lv_obj_add_event_cb(kb, kb_event_cb, LV_EVENT_CANCEL, nullptr);
}

// ---------- settings callbacks ----------------------------------------------

static void refresh_set_labels()
{
    lv_label_set_text_fmt(lbl_set_f[0], "%02d", g_set[0]);
    lv_label_set_text_fmt(lbl_set_f[1], "%02d", g_set[1]);
    lv_label_set_text_fmt(lbl_set_f[2], "%02d", g_set[2]);
    lv_label_set_text(lbl_set_f[3], MONTHS[g_set[3] - 1]);
    lv_label_set_text_fmt(lbl_set_f[4], "%d", g_set[4]);
}

static void load_set_from_clock()
{
    if (!time_valid()) return;
    time_t now = time(nullptr);
    struct tm t;
    localtime_r(&now, &t);
    g_set[0] = t.tm_hour;
    g_set[1] = t.tm_min;
    g_set[2] = t.tm_mday;
    g_set[3] = t.tm_mon + 1;
    g_set[4] = t.tm_year + 1900;
}

static void open_settings_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    load_set_from_clock();
    refresh_set_labels();
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

static void adj_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    intptr_t k = (intptr_t)lv_event_get_user_data(e);     // field*2 + (0 = plus, 1 = minus)
    int f = k / 2, d = (k & 1) ? -1 : 1;
    static const int lo[5] = {0, 0, 1, 1, 2024}, hi[5] = {23, 59, 31, 12, 2099};
    int span = hi[f] - lo[f] + 1;
    g_set[f] = lo[f] + ((g_set[f] - lo[f] + d) % span + span) % span;
    refresh_set_labels();
}

static void set_clock_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    g_rtc_write_req = true;      // done by loop()
    lv_label_set_text(lbl_rtc, "Setting clock...");
}

static void wifi_sw_cb(lv_event_t *e)
{
    net_set_wifi(lv_obj_has_state(sw_wifi, LV_STATE_CHECKED));
}

static void ble_sw_cb(lv_event_t *e)
{
    net_set_ble(lv_obj_has_state(sw_ble, LV_STATE_CHECKED));
}

static void loc_sw_cb(lv_event_t *e)
{
    net_set_location_auto(lv_obj_has_state(sw_loc, LV_STATE_CHECKED));
}

static void wifi_scan_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    net_request_scan();
}

static void wifi_pass_done(const char *pass) { net_request_connect(g_pending_ssid, pass); }

static void wifi_item_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    int i = (int)(intptr_t)lv_event_get_user_data(e);
    if (i < 0 || i >= g_ns.scan_count) return;
    strlcpy(g_pending_ssid, g_ns.scan_ssid[i], sizeof(g_pending_ssid));
    if (g_ns.scan_open[i]) {
        net_request_connect(g_pending_ssid, "");
    } else {
        char title[64];
        snprintf(title, sizeof(title), "Password for %s", g_pending_ssid);
        open_keyboard(title, "", true, wifi_pass_done);
    }
}

static void place_done(const char *name)
{
    if (name[0]) net_request_geocode(name);
}

static void place_btn_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    open_keyboard("Weather place (city name)", g_ns.manual_name, false, place_done);
}

static void wx_refresh_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    net_request_weather();
}

// ---------- UI construction: shared pieces ----------------------------------

static void page_dots(lv_obj_t *tile, int active)
{
    for (int i = 0; i < 3; i++) {
        lv_obj_t *d = make_rect(tile, 400 - 26 + i * 20, 462, 10, 10, i == active ? COL_TEXT : lv_color_hex(0x3A4750));
        lv_obj_set_style_radius(d, LV_RADIUS_CIRCLE, 0);
    }
}

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
    lv_obj_clear_flag(c, LV_OBJ_FLAG_CLICKABLE);
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

// ---------- page 1: gauges ---------------------------------------------------

static void build_battery_scale(lv_obj_t *p)
{
    // Zones per the gauge dial: red 8-10.5, red/green hatched 10.5-12, green 12-15, red 15-16 V
    auto xv = [](float v) { return SCALE_X + (int)((v - DIAL_V_MIN) / (DIAL_V_MAX - DIAL_V_MIN) * SCALE_W); };
    int x1 = xv(10.5f), x2 = xv(12.0f), x3 = xv(15.0f), xe = SCALE_X + SCALE_W;
    make_rect(p, SCALE_X, BAT_BAR_Y, x1 - SCALE_X, SCALE_H, COL_RED);
    for (int x = x1, i = 0; x < x2; x += 8, i++) {
        make_rect(p, x, BAT_BAR_Y, (x + 8 <= x2 ? 8 : x2 - x), SCALE_H, (i & 1) ? COL_GREEN : COL_RED);
    }
    make_rect(p, x2, BAT_BAR_Y, x3 - x2, SCALE_H, COL_GREEN);
    make_rect(p, x3, BAT_BAR_Y, xe - x3, SCALE_H, COL_RED);

    lv_obj_t *icon = make_label(p, &font_icon_44, COL_MUTED);
    lv_label_set_text(icon, FA_BATTERY);
    lv_obj_set_pos(icon, 22, BAT_BAR_Y - 12);

    static const char *const t[5] = {"8", "10", "12", "14", "16"};
    make_ticks(p, BAT_BAR_Y, t);
    mk_bat = make_marker(p);
}

static void build_water_scale(lv_obj_t *p)
{
    make_rect(p, SCALE_X, WAT_BAR_Y, SCALE_W, SCALE_H, COL_TRACK);
    const int n = 57;                                  // blue wedge: thin on the left, full on the right
    for (int i = 0; i < n; i++) {
        int x0 = SCALE_X + i * SCALE_W / n, x1 = SCALE_X + (i + 1) * SCALE_W / n;
        int h  = (int)(SCALE_H * (0.15f + 0.85f * i / (n - 1)) + 0.5f);
        make_rect(p, x0, WAT_BAR_Y + SCALE_H - h, x1 - x0, h, COL_BLUE);
    }
    lv_obj_t *icon = make_label(p, &font_icon_44, COL_MUTED);
    lv_label_set_text(icon, FA_TINT);
    lv_obj_set_pos(icon, 34, WAT_BAR_Y - 12);

    static const char *const t[5] = {"0", "1/4", "1/2", "3/4", "1/1"};
    make_ticks(p, WAT_BAR_Y, t);
    mk_wat = make_marker(p);
}

static void build_gauge_page(lv_obj_t *p)
{
    lv_obj_t *name = make_label(p, &lv_font_montserrat_26, COL_TEXT);
    lv_label_set_text(name, "Globebus");
    lv_obj_set_pos(name, 24, 18);

    lbl_clock = make_label(p, &font_clock_44, COL_TEXT);
    lv_label_set_text(lbl_clock, "--:--");
    lv_obj_align(lbl_clock, LV_ALIGN_TOP_MID, 0, 18);

    lbl_toptron = make_label(p, &lv_font_montserrat_20, COL_MUTED);
    lv_label_set_text(lbl_toptron, "Toptron");
    lv_obj_align(lbl_toptron, LV_ALIGN_TOP_RIGHT, -24, 20);
    lv_obj_update_layout(p);
    dot_status = make_rect(p, 0, 0, 10, 10, COL_OK);
    lv_obj_set_style_radius(dot_status, LV_RADIUS_CIRCLE, 0);
    lv_obj_align_to(dot_status, lbl_toptron, LV_ALIGN_OUT_LEFT_MID, -8, 0);
    lbl_icons = make_label(p, &font_sym_22, COL_MUTED);
    lv_label_set_text(lbl_icons, "");
    lv_obj_align_to(lbl_icons, dot_status, LV_ALIGN_OUT_LEFT_MID, -14, 0);

    lbl_volt = make_label(p, &font_num_88, COL_DIM);
    lv_obj_set_width(lbl_volt, 400);
    lv_obj_set_style_text_align(lbl_volt, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(lbl_volt, "-- V");
    lv_obj_set_pos(lbl_volt, 0, 118);

    lbl_pct = make_label(p, &font_num_88, COL_DIM);
    lv_obj_set_width(lbl_pct, 400);
    lv_obj_set_style_text_align(lbl_pct, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(lbl_pct, "-- %");
    lv_obj_set_pos(lbl_pct, 400, 118);

    build_battery_scale(p);
    build_water_scale(p);

    lbl_state = make_label(p, &lv_font_montserrat_18, COL_MUTED);
    lv_label_set_text(lbl_state, "Hold a selector on the panel");
    lv_obj_set_pos(lbl_state, 24, 432);

    lv_obj_t *gear = lv_btn_create(p);
    lv_obj_set_size(gear, 56, 44);
    lv_obj_set_pos(gear, 800 - 16 - 56, 420);
    lv_obj_set_style_bg_opa(gear, LV_OPA_TRANSP, 0);
    lv_obj_set_style_shadow_width(gear, 0, 0);
    lv_obj_add_event_cb(gear, open_settings_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *gl = make_label(gear, &lv_font_montserrat_26, COL_MUTED);
    lv_label_set_text(gl, LV_SYMBOL_SETTINGS);
    lv_obj_center(gl);

    pill_prov = lv_obj_create(p);
    lv_obj_remove_style_all(pill_prov);
    lv_obj_set_style_bg_color(pill_prov, lv_color_hex(0x3A2F12), 0);
    lv_obj_set_style_bg_opa(pill_prov, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(pill_prov, 12, 0);
    lv_obj_set_style_pad_hor(pill_prov, 12, 0);
    lv_obj_set_style_pad_ver(pill_prov, 3, 0);
    lv_obj_set_size(pill_prov, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
    lv_obj_clear_flag(pill_prov, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_t *pl = make_label(pill_prov, &lv_font_montserrat_16, lv_color_hex(0xF0C565));
    lv_label_set_text(pl, "provisional");
    lv_obj_update_layout(p);
    lv_obj_align_to(pill_prov, gear, LV_ALIGN_OUT_LEFT_MID, -8, 0);
    if (VOLT_CALIBRATED) lv_obj_add_flag(pill_prov, LV_OBJ_FLAG_HIDDEN);

    page_dots(p, 0);
}

// ---------- page 2: weather --------------------------------------------------

static void build_weather_page(lv_obj_t *p)
{
    lv_obj_t *name = make_label(p, &lv_font_montserrat_26, COL_TEXT);
    lv_label_set_text(name, "Globebus");
    lv_obj_set_pos(name, 24, 18);

    lbl_wx_clock = make_label(p, &lv_font_montserrat_28, COL_TEXT);
    lv_label_set_text(lbl_wx_clock, "--:--");
    lv_obj_align(lbl_wx_clock, LV_ALIGN_TOP_MID, 0, 18);

    lbl_wx_place = make_label(p, &lv_font_montserrat_20, COL_MUTED);
    lv_label_set_text(lbl_wx_place, "");
    lv_obj_align(lbl_wx_place, LV_ALIGN_TOP_RIGHT, -24, 22);

    wx_content = lv_obj_create(p);
    lv_obj_remove_style_all(wx_content);
    lv_obj_set_size(wx_content, 800, 380);
    lv_obj_set_pos(wx_content, 0, 70);
    lv_obj_clear_flag(wx_content, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);

    lbl_wx_icon = make_label(wx_content, &font_wx_80, COL_TEXT);
    lv_label_set_text(lbl_wx_icon, FA_CLOUD);
    lv_obj_set_pos(lbl_wx_icon, 110, 34);

    lbl_wx_temp = make_label(wx_content, &font_num_88, COL_TEXT);
    lv_label_set_text(lbl_wx_temp, "--");
    lv_obj_set_pos(lbl_wx_temp, 250, 14);

    lbl_wx_cond = make_label(wx_content, &lv_font_montserrat_22, COL_MUTED);
    lv_label_set_text(lbl_wx_cond, "");
    lv_obj_set_pos(lbl_wx_cond, 254, 116);

    static const char *const det_icons[3] = {FA_WIND, FA_TINT, FA_ARROWSV};
    for (int i = 0; i < 3; i++) {
        lv_obj_t *ic = make_label(wx_content, &font_sym_22, COL_MUTED);
        lv_label_set_text(ic, det_icons[i]);
        lv_obj_set_pos(ic, 560, 30 + i * 36);
        lbl_wx_det[i] = make_label(wx_content, &lv_font_montserrat_20, COL_TEXT);
        lv_label_set_text(lbl_wx_det[i], "--");
        lv_obj_set_pos(lbl_wx_det[i], 596, 30 + i * 36);
    }

    make_rect(wx_content, 40, 170, 720, 1, COL_TRACK);
    for (int d = 0; d < WX_DAYS; d++) {
        int x = 40 + d * 180;
        const lv_font_t *fonts[4] = {&lv_font_montserrat_20, &font_wx_34, &lv_font_montserrat_22, &lv_font_montserrat_18};
        const lv_color_t cols[4] = {COL_MUTED, COL_TEXT, COL_TEXT, COL_MUTED};
        const int ys[4] = {184, 216, 266, 298};
        for (int k = 0; k < 4; k++) {
            lv_obj_t *l = make_label(wx_content, fonts[k], cols[k]);
            lv_obj_set_width(l, 180);
            lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
            lv_label_set_text(l, k == 1 ? FA_CLOUD : "--");
            lv_obj_set_pos(l, x, ys[k]);
            lbl_wx_day[d][k] = l;
        }
    }

    lbl_wx_foot = make_label(p, &lv_font_montserrat_14, COL_DIM);
    lv_label_set_text(lbl_wx_foot, "");
    lv_obj_set_pos(lbl_wx_foot, 24, 438);

    lbl_wx_msg = make_label(p, &lv_font_montserrat_26, COL_MUTED);
    lv_obj_set_width(lbl_wx_msg, 700);
    lv_obj_set_style_text_align(lbl_wx_msg, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(lbl_wx_msg, "No weather yet - turn on Wi-Fi in Settings");
    lv_obj_align(lbl_wx_msg, LV_ALIGN_CENTER, 0, 0);

    lv_obj_add_flag(wx_content, LV_OBJ_FLAG_HIDDEN);
    page_dots(p, 1);
}

static void update_weather_page(const NetState &n)
{
    char buf[64];
    lv_label_set_text(lbl_wx_place, n.place[0] ? n.place : (n.loc_auto ? "Location: automatic" : "No place set"));
    lv_obj_align(lbl_wx_place, LV_ALIGN_TOP_RIGHT, -24, 22);

    if (!n.wx_valid) {
        lv_obj_add_flag(wx_content, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(lbl_wx_msg, LV_OBJ_FLAG_HIDDEN);
        const char *m = !n.wifi_enabled ? "No weather yet - turn on Wi-Fi in Settings"
                      : !n.connected    ? "Waiting for Wi-Fi..."
                      : !n.loc_valid    ? "Finding location..."
                                        : "Fetching weather...";
        lv_label_set_text(lbl_wx_msg, m);
        lv_label_set_text(lbl_wx_foot, "");
        return;
    }
    lv_obj_clear_flag(wx_content, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(lbl_wx_msg, LV_OBJ_FLAG_HIDDEN);

    lv_label_set_text(lbl_wx_icon, wx_icon(n.code, n.is_day));
    snprintf(buf, sizeof(buf), "%.0f\xC2\xB0", n.temp);
    lv_label_set_text(lbl_wx_temp, buf);
    snprintf(buf, sizeof(buf), "%s - feels %.0f\xC2\xB0", wx_text(n.code), n.feels);
    lv_label_set_text(lbl_wx_cond, buf);
    snprintf(buf, sizeof(buf), "%.0f km/h %s", n.wind_kmh, compass(n.wind_dir));
    lv_label_set_text(lbl_wx_det[0], buf);
    snprintf(buf, sizeof(buf), "%d %%", n.humidity);
    lv_label_set_text(lbl_wx_det[1], buf);
    snprintf(buf, sizeof(buf), "%.0f\xC2\xB0 / %.0f\xC2\xB0", n.day[0].tmin, n.day[0].tmax);
    lv_label_set_text(lbl_wx_det[2], buf);

    for (int d = 0; d < WX_DAYS; d++) {
        const WxDay &w = n.day[d];
        lv_label_set_text(lbl_wx_day[d][0], w.label);
        lv_label_set_text(lbl_wx_day[d][1], wx_icon(w.code, true));
        snprintf(buf, sizeof(buf), "%.0f\xC2\xB0 / %.0f\xC2\xB0", w.tmax, w.tmin);
        lv_label_set_text(lbl_wx_day[d][2], buf);
        if (w.pop >= 0) snprintf(buf, sizeof(buf), "%d %%", w.pop);
        else            snprintf(buf, sizeof(buf), "--");
        lv_label_set_text(lbl_wx_day[d][3], buf);
    }

    struct tm t;
    localtime_r(&n.wx_time, &t);
    snprintf(buf, sizeof(buf), "Updated %02d:%02d - open-meteo.com%s", t.tm_hour, t.tm_min,
             n.connected ? "" : " - offline");
    lv_label_set_text(lbl_wx_foot, buf);
}

// ---------- page 3: big clock ------------------------------------------------

static void build_clock_page(lv_obj_t *p)
{
    lbl_big_clock = make_label(p, &font_clock_170, COL_TEXT);
    lv_label_set_text(lbl_big_clock, "--:--");
    lv_obj_align(lbl_big_clock, LV_ALIGN_CENTER, 0, -40);

    lbl_big_date = make_label(p, &lv_font_montserrat_36, COL_MUTED);
    lv_label_set_text(lbl_big_date, "");
    lv_obj_align(lbl_big_date, LV_ALIGN_CENTER, 0, 90);

    page_dots(p, 2);
}

// ---------- settings screen --------------------------------------------------

static lv_obj_t *make_switch_row(lv_obj_t *p, const char *text, int y, lv_event_cb_t cb, lv_obj_t **sw_out)
{
    lv_obj_t *l = make_label(p, &lv_font_montserrat_22, COL_TEXT);
    lv_label_set_text(l, text);
    lv_obj_set_pos(l, 0, y + 8);
    lv_obj_t *sw = lv_switch_create(p);
    lv_obj_set_size(sw, 80, 40);
    lv_obj_set_pos(sw, 330, y);
    lv_obj_add_event_cb(sw, cb, LV_EVENT_VALUE_CHANGED, nullptr);
    *sw_out = sw;
    return l;
}

static void build_clock_tab(lv_obj_t *t)
{
    // time row: [+] HH [-]  :  [+] MM [-]        date row: [+] DD [-]  [+] Mon [-]  [+] YYYY [-]
    struct Field { int x, y, w; };
    const Field fields[5] = {{0, 0, 70}, {250, 0, 70}, {0, 80, 70}, {250, 80, 90}, {520, 80, 110}};
    for (int f = 0; f < 5; f++) {
        lv_obj_t *bp = make_button(t, LV_SYMBOL_PLUS, 60, 50, adj_cb, (void *)(intptr_t)(f * 2));
        lv_obj_set_pos(bp, fields[f].x, fields[f].y);
        lbl_set_f[f] = make_label(t, &lv_font_montserrat_36, COL_TEXT);
        lv_obj_set_width(lbl_set_f[f], fields[f].w);
        lv_obj_set_style_text_align(lbl_set_f[f], LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_pos(lbl_set_f[f], fields[f].x + 64, fields[f].y + 4);
        lv_obj_t *bm = make_button(t, LV_SYMBOL_MINUS, 60, 50, adj_cb, (void *)(intptr_t)(f * 2 + 1));
        lv_obj_set_pos(bm, fields[f].x + 68 + fields[f].w, fields[f].y);
    }
    lv_obj_t *colon = make_label(t, &lv_font_montserrat_36, COL_TEXT);
    lv_label_set_text(colon, ":");
    lv_obj_set_pos(colon, 226, 4);

    lv_obj_t *bset = make_button(t, "Set clock", 200, 50, set_clock_cb, nullptr);
    lv_obj_set_pos(bset, 540, 0);

    lbl_rtc = make_label(t, &lv_font_montserrat_18, COL_MUTED);
    lv_label_set_long_mode(lbl_rtc, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_rtc, 740);
    lv_label_set_text(lbl_rtc, "");
    lv_obj_set_pos(lbl_rtc, 0, 160);
    refresh_set_labels();
}

static void build_wifi_tab(lv_obj_t *t)
{
    make_switch_row(t, "Wi-Fi", 0, wifi_sw_cb, &sw_wifi);
    lv_obj_t *bs = make_button(t, LV_SYMBOL_REFRESH "  Scan", 170, 46, wifi_scan_cb, nullptr);
    lv_obj_set_pos(bs, 570, 0);

    lbl_wifi = make_label(t, &lv_font_montserrat_18, COL_MUTED);
    lv_label_set_long_mode(lbl_wifi, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_wifi, 740);
    lv_label_set_text(lbl_wifi, "");
    lv_obj_set_pos(lbl_wifi, 0, 54);

    list_wifi = lv_list_create(t);
    lv_obj_set_size(list_wifi, 740, 210);
    lv_obj_set_pos(list_wifi, 0, 104);
    lv_obj_set_style_bg_color(list_wifi, COL_PANEL, 0);
    lv_obj_set_style_border_width(list_wifi, 0, 0);
    lv_obj_set_style_text_font(list_wifi, &lv_font_montserrat_20, 0);
}

static void build_ble_tab(lv_obj_t *t)
{
    make_switch_row(t, "Bluetooth (BLE)", 0, ble_sw_cb, &sw_ble);
    lbl_ble = make_label(t, &lv_font_montserrat_18, COL_MUTED);
    lv_label_set_long_mode(lbl_ble, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_ble, 740);
    lv_label_set_text(lbl_ble, "");
    lv_obj_set_pos(lbl_ble, 0, 60);
    lv_obj_t *n = make_label(t, &lv_font_montserrat_16, COL_DIM);
    lv_label_set_long_mode(n, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(n, 740);
    lv_label_set_text(n, "Planned: connect to the EcoFlow and the kiosk app. For now Bluetooth only advertises "
                         "the name \"Globebus\". Off saves power.");
    lv_obj_set_pos(n, 0, 110);
}

static void build_weather_tab(lv_obj_t *t)
{
    make_switch_row(t, "Automatic location (IP)", 0, loc_sw_cb, &sw_loc);
    btn_place = make_button(t, "Set place...", 220, 46, place_btn_cb, nullptr);
    lv_obj_set_pos(btn_place, 0, 60);
    lv_obj_t *br = make_button(t, LV_SYMBOL_REFRESH "  Refresh now", 240, 46, wx_refresh_cb, nullptr);
    lv_obj_set_pos(br, 500, 60);
    lbl_loc = make_label(t, &lv_font_montserrat_18, COL_MUTED);
    lv_label_set_long_mode(lbl_loc, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_loc, 740);
    lv_label_set_text(lbl_loc, "");
    lv_obj_set_pos(lbl_loc, 0, 124);
    lv_obj_t *n = make_label(t, &lv_font_montserrat_16, COL_DIM);
    lv_label_set_long_mode(n, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(n, 740);
    lv_label_set_text(n, "Automatic uses the internet connection's location - on a phone hotspot that can be the "
                         "carrier's city. Setting a place switches to manual.");
    lv_obj_set_pos(n, 0, 200);
}

static void build_display_tab(lv_obj_t *t)
{
    lv_obj_t *l = make_label(t, &lv_font_montserrat_22, COL_TEXT);
    lv_label_set_text(l, "Screen off after");
    lv_obj_set_pos(l, 0, 10);
    dd_timeout = lv_dropdown_create(t);
    lv_dropdown_set_options_static(dd_timeout, TIMEOUT_OPTS);
    lv_dropdown_set_selected(dd_timeout, g_timeout_idx);
    lv_obj_set_width(dd_timeout, 200);
    lv_obj_set_style_text_font(dd_timeout, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_font(lv_dropdown_get_list(dd_timeout), &lv_font_montserrat_22, 0);
    lv_obj_set_pos(dd_timeout, 330, 0);
    lv_obj_add_event_cb(dd_timeout, timeout_dd_cb, LV_EVENT_VALUE_CHANGED, nullptr);
    lv_obj_t *l2 = make_label(t, &lv_font_montserrat_22, COL_TEXT);
    lv_label_set_text(l2, "Keep reading");
    lv_obj_set_pos(l2, 0, 74);
    dd_hold = lv_dropdown_create(t);
    lv_dropdown_set_options_static(dd_hold, HOLD_OPTS);
    lv_dropdown_set_selected(dd_hold, g_hold_idx);
    lv_obj_set_width(dd_hold, 200);
    lv_obj_set_style_text_font(dd_hold, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_font(lv_dropdown_get_list(dd_hold), &lv_font_montserrat_22, 0);
    lv_obj_set_pos(dd_hold, 330, 64);
    lv_obj_add_event_cb(dd_hold, hold_dd_cb, LV_EVENT_VALUE_CHANGED, nullptr);

    lv_obj_t *n = make_label(t, &lv_font_montserrat_16, COL_DIM);
    lv_label_set_long_mode(n, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(n, 740);
    lv_label_set_text(n, "With no touch and no Toptron press for this long, the screen goes black and the backlight "
                         "switches off (saves power and prevents image retention). Touch anywhere or press a "
                         "selector to wake - the waking touch does not press any button.");
    lv_obj_set_pos(n, 0, 134);
    lv_obj_t *n2 = make_label(t, &lv_font_montserrat_16, COL_DIM);
    lv_label_set_long_mode(n2, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(n2, 740);
    lv_label_set_text(n2, "\"Keep reading\" is how long the last measured value stays on the gauge page after you let "
                          "go of the selector. Until release matches the old gauge, whose needle fell back.");
    lv_obj_set_pos(n2, 0, 210);
}

static void build_diag_tab(lv_obj_t *t)
{
    lv_obj_t *brs = make_button(t, "Rescan I2C", 190, 42, rescan_cb, nullptr);
    lv_obj_set_pos(brs, 550, 0);
    lbl_scan = make_label(t, &lv_font_montserrat_14, COL_MUTED);
    lv_label_set_long_mode(lbl_scan, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_scan, 530);
    lv_label_set_text(lbl_scan, "I2C scan: waiting...");
    lv_obj_set_pos(lbl_scan, 0, 4);
    lbl_hist = make_label(t, &lv_font_montserrat_14, lv_color_hex(0xB0BEC5));
    lv_label_set_text(lbl_hist, "No presses yet.");
    lv_obj_set_pos(lbl_hist, 0, 52);
    lbl_diag = make_label(t, &lv_font_montserrat_14, COL_MUTED);
    lv_label_set_text(lbl_diag, "Idle: --");
    lv_obj_set_pos(lbl_diag, 0, 190);
    lbl_netst = make_label(t, &lv_font_montserrat_14, COL_MUTED);
    lv_label_set_long_mode(lbl_netst, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_netst, 740);
    lv_label_set_text(lbl_netst, "Network: --");
    lv_obj_set_pos(lbl_netst, 0, 262);
}

static void build_settings_screen()
{
    scr_set = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(scr_set, COL_BG, 0);
    lv_obj_clear_flag(scr_set, LV_OBJ_FLAG_SCROLLABLE);

    lv_obj_t *back = make_button(scr_set, LV_SYMBOL_LEFT "  Back", 150, 48, back_cb, nullptr);
    lv_obj_set_pos(back, 16, 10);
    lv_obj_t *title = make_label(scr_set, &lv_font_montserrat_28, COL_TEXT);
    lv_label_set_text(title, "Settings");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 18);

    lv_obj_t *tv = lv_tabview_create(scr_set, LV_DIR_TOP, 54);
    lv_obj_set_size(tv, 800, 410);
    lv_obj_set_pos(tv, 0, 70);
    lv_obj_set_style_bg_color(tv, COL_BG, 0);
    lv_obj_t *btns = lv_tabview_get_tab_btns(tv);
    lv_obj_set_style_bg_color(btns, COL_PANEL, 0);
    lv_obj_set_style_text_color(btns, COL_MUTED, 0);
    lv_obj_set_style_text_font(btns, &lv_font_montserrat_20, 0);
    lv_obj_set_style_text_color(btns, COL_TEXT, LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_t *content = lv_tabview_get_content(tv);
    lv_obj_clear_flag(content, LV_OBJ_FLAG_SCROLLABLE);   // tabs by tapping only, no accidental swipes

    lv_obj_t *tabs[6];
    const char *names[6] = {"Clock", "Wi-Fi", "BT", "Weather", "Display", "Diag"};
    for (int i = 0; i < 6; i++) {
        tabs[i] = lv_tabview_add_tab(tv, names[i]);
        lv_obj_set_style_pad_all(tabs[i], 24, 0);
    }
    build_clock_tab(tabs[0]);
    build_wifi_tab(tabs[1]);
    build_ble_tab(tabs[2]);
    build_weather_tab(tabs[3]);
    build_display_tab(tabs[4]);
    build_diag_tab(tabs[5]);
}

// ---------- network state -> UI ---------------------------------------------

static void update_net_ui()
{
    static NetState n;
    static uint32_t last_version = UINT32_MAX;
    static bool last_scanning = false;
    static int  last_scan_count = -1;
    net_get(&n);
    if (n.version == last_version) return;
    last_version = n.version;
    g_ns = n;

    lvgl_port_lock(-1);

    // switches reflect the stored state (setting a state does not fire VALUE_CHANGED)
    if (n.wifi_enabled) lv_obj_add_state(sw_wifi, LV_STATE_CHECKED); else lv_obj_clear_state(sw_wifi, LV_STATE_CHECKED);
    if (n.ble_enabled)  lv_obj_add_state(sw_ble, LV_STATE_CHECKED);  else lv_obj_clear_state(sw_ble, LV_STATE_CHECKED);
    if (n.loc_auto)     lv_obj_add_state(sw_loc, LV_STATE_CHECKED);  else lv_obj_clear_state(sw_loc, LV_STATE_CHECKED);

    char buf[160];
    if (!n.wifi_enabled)     snprintf(buf, sizeof(buf), "Off");
    else if (n.connected)    snprintf(buf, sizeof(buf), "Connected to %s  |  %s  |  signal %d dBm", n.ssid, n.ip, n.rssi);
    else if (n.have_creds)   snprintf(buf, sizeof(buf), "Connecting to %s...", n.ssid);
    else                     snprintf(buf, sizeof(buf), "No network saved - tap Scan and choose one");
    lv_label_set_text(lbl_wifi, n.scanning ? "Scanning..." : buf);

    if (n.scanning != last_scanning || n.scan_count != last_scan_count) {
        last_scanning = n.scanning;
        last_scan_count = n.scan_count;
        lv_obj_clean(list_wifi);
        for (int i = 0; i < n.scan_count; i++) {
            char item[64];
            snprintf(item, sizeof(item), "%s   (%d dBm%s)", n.scan_ssid[i], n.scan_rssi[i], n.scan_open[i] ? ", open" : "");
            lv_obj_t *b = lv_list_add_btn(list_wifi, LV_SYMBOL_WIFI, item);
            lv_obj_set_style_bg_color(b, COL_PANEL, 0);
            lv_obj_set_style_text_color(b, COL_TEXT, 0);
            lv_obj_add_event_cb(b, wifi_item_cb, LV_EVENT_CLICKED, (void *)(intptr_t)i);
        }
    }

    lv_label_set_text(lbl_ble, n.ble_enabled ? "On - advertising as \"Globebus\"" : "Off");

    snprintf(buf, sizeof(buf), "Using: %s%s%s", n.loc_valid ? n.place : "(no location yet)",
             n.loc_valid ? (n.loc_auto ? "  (automatic)" : "  (manual)") : "",
             n.manual_name[0] ? "" : "   |   no manual place saved");
    lv_label_set_text(lbl_loc, buf);

    // main page status icons: Wi-Fi bright when connected, dim when enabled but offline; BT when on
    snprintf(buf, sizeof(buf), "%s%s", n.wifi_enabled ? FA_WIFI : "", n.ble_enabled ? "  " FA_BT : "");
    lv_label_set_text(lbl_icons, buf);
    lv_obj_set_style_text_color(lbl_icons, n.connected ? COL_TEXT : COL_DIM, 0);
    lv_obj_update_layout(lbl_icons);
    lv_obj_align_to(lbl_icons, dot_status, LV_ALIGN_OUT_LEFT_MID, -14, 0);

    snprintf(buf, sizeof(buf), "Network: %s", n.status[0] ? n.status : "--");
    lv_label_set_text(lbl_netst, buf);

    update_weather_page(n);
    lvgl_port_unlock();
}

static void update_rtc_status()
{
    char buf[160];
    struct tm t;
    bool osc = false;
    bool rtc_ok = rtc_read(&t, &osc);
    const char *rtc = rtc_ok ? "RTC running" : (osc ? "RTC not set" : "RTC read error");
    if (g_ns.time_synced) {
        struct tm s;
        localtime_r(&g_ns.last_sync, &s);
        snprintf(buf, sizeof(buf), "%s  |  Internet time synced at %02d:%02d (Europe/Bucharest)  |  "
                                   "no coin cell: time is lost when power is removed", rtc, s.tm_hour, s.tm_min);
    } else {
        snprintf(buf, sizeof(buf), "%s  |  Internet time: not synced (%s)  |  no coin cell: time is lost when power "
                                   "is removed", rtc, g_ns.connected ? "waiting" : "needs Wi-Fi");
    }
    lvgl_port_lock(-1);
    lv_label_set_text(lbl_rtc, buf);
    lvgl_port_unlock();
}

// ---------- Arduino entry points ---------------------------------------------

void setup()
{
    Serial.begin(115200);
    delay(1500);   // give USB-CDC time to enumerate so early logs are not lost
    Serial.println("\n=== Dethleffs panel - Globebus UI (Toptron via burden resistor) ===");

    setenv("TZ", GLOBEBUS_TZ, 1);
    tzset();

    Board *board = new Board();
    g_board = board;
    g_board_name = board->getConfig().name;
    Serial.printf("Initializing board (%s)\n", g_board_name);
    board->init();
    if (!board->begin()) {
        Serial.println("FATAL: board->begin() failed - LCD/touch/IO-expander init error");
        while (true) { delay(1000); }
    }

    system_time_from_rtc();

    {
        Preferences p;
        p.begin("globebus", true);
        int idx = p.getUChar("disp_to", TIMEOUT_DEFAULT);
        int hidx = p.getUChar("hold", HOLD_DEFAULT);
        p.end();
        g_hold_idx = (hidx >= 0 && hidx < (int)(sizeof(HOLD_SECONDS) / sizeof(HOLD_SECONDS[0]))) ? hidx : HOLD_DEFAULT;
        g_timeout_idx = (idx >= 0 && idx < (int)(sizeof(TIMEOUT_MIN) / sizeof(TIMEOUT_MIN[0]))) ? idx : TIMEOUT_DEFAULT;
    }

    lvgl_port_init(board->getLCD(), board->getTouch());
    lvgl_port_lock(-1);
    scr_main = lv_obj_create(nullptr);
    lv_obj_set_style_bg_color(scr_main, COL_BG, 0);
    lv_obj_t *tv = lv_tileview_create(scr_main);
    lv_obj_set_style_bg_color(tv, COL_BG, 0);
    lv_obj_set_scrollbar_mode(tv, LV_SCROLLBAR_MODE_OFF);
    build_gauge_page(lv_tileview_add_tile(tv, 0, 0, LV_DIR_RIGHT));
    build_weather_page(lv_tileview_add_tile(tv, 1, 0, LV_DIR_LEFT | LV_DIR_RIGHT));
    build_clock_page(lv_tileview_add_tile(tv, 2, 0, LV_DIR_LEFT));
    build_settings_screen();
    scr_blank = lv_obj_create(nullptr);             // black, clickable: swallows the waking touch
    lv_obj_set_style_bg_color(scr_blank, lv_color_black(), 0);
    lv_obj_add_flag(scr_blank, LV_OBJ_FLAG_CLICKABLE);
    lv_scr_load(scr_main);
    lvgl_port_unlock();

    net_begin(on_ntp_synced);

    Serial.printf("Setup done. Burden %.0f ohm, press threshold %.1f mV, volts %s. Internal heap free %lu B.\n",
                  BURDEN_OHM, PRESS_THRESHOLD_MV, VOLT_CALIBRATED ? "calibrated" : "PROVISIONAL",
                  (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
    g_rescan_requested = true;
}

void loop()
{
    static uint32_t last_poll_ms = 0, last_diag_ms = 0, last_clock_ms = 0, last_net_ms = 0, last_hold_ui_s = 0;
    static uint32_t last_screen_ms = 0;

    if (g_press.active) {
        float v;
        if (!ads_read_volts(MUX_DIFF_01, PGA_SIGNAL, &v)) return;
        float mv = v * 1000.0f;
        if (mv < PRESS_THRESHOLD_MV) {
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
        struct tm t = {};
        t.tm_hour  = g_set[0];
        t.tm_min   = g_set[1];
        t.tm_mday  = g_set[2];
        t.tm_mon   = g_set[3] - 1;
        t.tm_year  = g_set[4] - 1900;
        t.tm_isdst = -1;
        set_system_time(&t);
        rtc_from_system_time("manual set");
        last_clock_ms = 0;
    }
    if (g_ntp_synced) {
        g_ntp_synced = false;
        rtc_from_system_time("internet time");
        last_clock_ms = 0;
    }

    uint32_t now = millis();
    if (now - last_poll_ms >= IDLE_POLL_MS) {
        last_poll_ms = now;
        float v;
        if (!ads_read_volts(MUX_DIFF_01, PGA_SIGNAL, &v)) {
            // keep the idle counter as it is; a failed read says nothing about the input
        } else if (v * 1000.0f < PRESS_THRESHOLD_MV) {
            if (g_idle_ok < IDLE_CONFIRM_SAMPLES) g_idle_ok++;
        } else if (now < BOOT_IGNORE_MS) {
            Serial.printf("[press] ignored during boot settle: %.2f mV\n", v * 1000.0f);
        } else if (g_idle_ok < IDLE_CONFIRM_SAMPLES) {
            Serial.printf("[press] ignored: input not quiet yet (%.2f mV)\n", v * 1000.0f);
        } else {
            // confirm with back-to-back samples before calling it a press
            float first = v * 1000.0f;
            bool  ok = true;
            for (int i = 1; i < CONFIRM_SAMPLES && ok; i++) {
                ok = ads_read_volts(MUX_DIFF_01, PGA_SIGNAL, &v) && v * 1000.0f >= PRESS_THRESHOLD_MV;
            }
            if (ok) {
                g_idle_ok = 0;
                press_start(first);
                press_sample(v * 1000.0f);
                return;
            }
        }
    }

    if (g_display_ms) {
        uint32_t hold_ms = (uint32_t)HOLD_SECONDS[g_hold_idx] * 1000UL;
        uint32_t age = now - g_display_ms;
        if (hold_ms == 0) {
            clear_value();                        // "until release": press_end published the value, now drop it
        } else if (age >= hold_ms) {
            clear_value();
        } else {
            uint32_t left = (hold_ms - age + 999) / 1000;
            if (left != last_hold_ui_s) {
                last_hold_ui_s = left;
                char t[48];
                snprintf(t, sizeof(t), "Held reading - clears in %lu s", (unsigned long)left);
                set_state_text(t);
            }
        }
    }

    if (now - last_screen_ms >= SCREEN_CHECK_MS) {
        last_screen_ms = now;
        check_screen_timeout();
    }
    if (now - last_net_ms >= NET_UI_PERIOD_MS) {
        last_net_ms = now;
        update_net_ui();
    }
    if (now - last_clock_ms >= CLOCK_PERIOD_MS || last_clock_ms == 0) {
        last_clock_ms = now;
        update_clocks();
    }
    if (now - last_diag_ms >= DIAG_PERIOD_MS) {
        last_diag_ms = now;
        run_diagnostics();
        update_rtc_status();
    }
    delay(5);
}
