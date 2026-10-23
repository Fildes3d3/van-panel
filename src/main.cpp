/*
 * Dethleffs Globebus control panel - firmware: Toptron gauge reader + Globebus UI + Wi-Fi/weather/clock
 * Board : Waveshare ESP32-S3-Touch-LCD-4.3B (M-053)
 *
 * Toptron facts (CLAUDE.md, Stage 4 notes):
 *   - The gauge output exists ONLY while a momentary selector is held (starter/leisure rocker, fresh/grey rocker).
 *   - It is a ~mA gauge drive, not a stiff voltage. The original gauge is 85.8 ohm.
 *   - Battery channels and the clean tank are steady while held; grey decays (dirty sender suspected).
 *   - The original gauge dial: ONE needle, two aligned linear scales - 8/10/12/14/16 V on top and
 *     0/(1/4)/(1/2)/(3/4)/(1/1) below. The calibrated line puts 0 mV at ~9.95 V, not 8 V, so the dial formula can
 *     never show an empty tank (floor 24 %); tanks have their own mapping from measured rod states - TANK_BANDS.
 *
 * Hardware (perfboard, van configuration):
 *   SIG+ --+-- 4.7k -- A0,  100 nF A0 -> board GND
 *        R_burden 100 ohm (final; was 499 ohm interim)
 *   SIG- --+-- 4.7k -- A1,  100 nF A1 -> board GND   (matched pair since the 2026-09-29 perfboard)
 *   Board GND on its own wire to vehicle ground (NOT the Toptron ground / signal -).
 *
 * Measurement (unchanged from the tested Stage 4c v4):
 *   - Polls A0-A1 (differential) every ~50 ms. |V| > PRESS_THRESHOLD_MV = a selector is held.
 *   - While held: ~100 samples/s. The value is shown live (every LIVE_UI_MS, from 0.1 s on, like the old needle);
 *     on release the value from ~0.2 s before release stays on screen. The tank rods need seconds to settle after
 *     a rest (21:08, 2026-10-08: 41.8 mV at the start, 47.0 at 0.5 s, 56.8 at the end), so the end is the best value.
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
 *   101 +-0.256 V. Signal at +-0.256 V with the 100 ohm burden (the 499 ohm interim needed +-1.024 V).
 *   Expected span with 100 ohm: ~0-140 mV (16.5 V on the dial ~ 1.4 mA), well inside +-256 mV.
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
#include "vlog.h"
#include "hist.h"
#include <Preferences.h>
#include <esp_system.h>
#include <esp_core_dump.h>
#include <esp_app_desc.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

using namespace esp_panel::drivers;
using namespace esp_panel::board;

LV_FONT_DECLARE(font_num_88);
LV_FONT_DECLARE(font_num_56);
LV_FONT_DECLARE(font_text_18);
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
static constexpr uint8_t MUX_AIN2    = 0x6;               // AIN2 - GND (supply divider)

struct Pga {
    uint16_t bits;
    float    lsb_v;
};
static constexpr Pga PGA_4V096  = {0x0200, 4.096f / 32768.0f};  // PGA = 001
static constexpr Pga PGA_2V048  = {0x0400, 2.048f / 32768.0f};  // PGA = 010

// Supply divider (2026-10-11): board VIN (the Toptron's fused +12 V) -- 47k -- A2 -- 4.7k -- board GND, no capacitor.
// The Toptron's tank output follows its supply (empty tank +11.7 mV with the charger on, even with the board on an
// isolated power bank), so the supply is measured to correct the tanks. 16 V -> 1.46 V at A2; a 40 V spike -> 3.6 V,
// current held < 1 mA by the 47k. Resistors measured: 46.8 k and 4.71 k (in circuit: A2-GND 4.71 k, VIN-A2 47.5 k).
static constexpr float SUPPLY_R_TOP_K = 46.8f;
static constexpr float SUPPLY_R_BOT_K = 4.71f;
static constexpr float SUPPLY_CAL     = 1.000f;   // set from a meter comparison at VIN
static constexpr float SUPPLY_MIN_V   = 5.0f;     // below this there is no van feed (USB / power bank)
static inline float supply_from_a2(float a2_volts)
{
    return a2_volts * (SUPPLY_R_TOP_K + SUPPLY_R_BOT_K) / SUPPLY_R_BOT_K * SUPPLY_CAL;
}
static constexpr Pga PGA_1V024  = {0x0600, 1.024f / 32768.0f};  // PGA = 011
static constexpr Pga PGA_0V256  = {0x0A00, 0.256f / 32768.0f};  // PGA = 101
static constexpr Pga PGA_SIGNAL = PGA_0V256;                    // 100 ohm burden
static constexpr const char *PGA_SIGNAL_TEXT = "+-0.256 V";
static constexpr float BURDEN_OHM = 100.0f;                     // nominal; fitted 2026-09-29

static constexpr uint32_t CONV_POLL_TIMEOUT_MS = 50;      // 128 SPS conversion is ~7.8 ms

static constexpr float    PRESS_THRESHOLD_MV   = 5.0f;    // "none" reads ~0 through the burden. 1 mV (the old
                                                          // 499 ohm threshold scaled by current) latched a "press"
                                                          // on a small idle offset in the van: stuck on 10.0 V. The
                                                          // smallest real signals are ~11 mV (10.5 V battery) and
                                                          // 19.5 mV (empty tank); 5 mV = 10.2 V on the line.
static constexpr uint32_t MAX_PRESS_MS         = 30000;   // the rockers are momentary: nothing real lasts this long
// Open-input detector. With the 100 ohm burden across SIG+/SIG-, A0 and A1 can never be further apart than the
// burden voltage (~130 mV at 16.5 V, ~0 at rest). In the van (2026-10-08) A0 wandered 200-900 mV from A1 for
// minutes: the A0 path had lost its connection and every press was ignored or rejected. Checked at rest only.
// Only visible on van power: on USB the whole board floats and an open A0 reads ~0 like a good one.
static constexpr uint32_t WIRE_CHECK_MS        = 2000;
static constexpr float    WIRE_LOOSE_MV        = 150.0f;  // |A0 - A1| above this, twice in a row = open
static constexpr float    WIRE_OK_MV           = 20.0f;   // below this for WIRE_OK_CHECKS in a row = connected again
static constexpr int      WIRE_OK_CHECKS       = 10;
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

// Display: live while held, from WIN_START_MS on (skips the rocker's contact transient); WIN_END_MS only bounds the
// early-window value kept for the diagnostics table.
static constexpr uint32_t WIN_START_MS    = 100;
static constexpr uint32_t WIN_END_MS      = 500;
static constexpr uint32_t LIVE_UI_MS      = 250;
static const uint16_t HOLD_SECONDS[] = {0, 5, 15, 60};        // 0 = clear as soon as the selector is released
static const char    *HOLD_OPTS      = "Until release\n5 s\n15 s\n60 s";
static constexpr int  HOLD_DEFAULT   = 1;                      // 5 s
static constexpr int      BACKLIGHT_EXIO   = 2;   // CH422G EXIO2 = backlight enable on this board (per its profile)
static constexpr uint32_t BL_TEST_MS       = 5000;
static constexpr uint32_t BL_REASSERT_MS    = 5000;   // while blanked, re-assert "off" in case a glitch flips it
static constexpr uint32_t ERROR_FRESH_MS = 5UL * 60UL * 1000UL;  // status dot shows recent errors, not a latch

// Volts calibration, 100 ohm burden: V = VOLT_OFFSET + VOLT_PER_MV * mV. Least-squares line through three points
// on the leisure battery, van power, battery voltage read at the terminals while the rocker was held:
//   12.55 V <-> 51.17 mV  charger off            (2026-09-29)
//   14.07 V <-> 81.05 mV  charger on             (2026-09-29, presses 80.99 / 81.10)
//   12.10 V <-> 44.84 mV  charger off, low point (2026-10-22, presses 44.77 / 44.85; meter read to one decimal)
// Residuals +0.06 / -0.01 / -0.05 V. The old two-point line (9.947 + 0.05087 mV) read 0.13 V high at 12.1 V.
// A two-segment fit would hit all three exactly, but its low slope swings +-10 % with the 12.1 V reading's last
// digit, so one straight line is the more honest choice. Outside 12.1-14.1 V it is an extrapolation.
// The same line is used for the car battery (one needle, one gauge).
static constexpr bool  VOLT_CALIBRATED = true;
static constexpr float VOLT_PER_MV     = 0.05330f;    // V per mV
static constexpr float VOLT_OFFSET     = 9.761f;      // V at 0 mV

// Tank level. The senders are 4 metal rods of different lengths (user, 2026-10-08): a common rod plus 3 level
// rods, so a tank can only report a few states, and the current through the water also varies with wetness,
// water and supply. The panel shows the state - 100 / 75 / "<=50" % - not a fine percentage. Both tanks share the
// table (the board cannot tell which is selected). Calibrated on the CLEAN tank only: the grey sender has never
// read reliably (user). Clean readings with the charger OFF, level seen through the tank wall:
//   full (4 rods in water)  55-59 mV   (2026-10-08 settled 58.7; power bank 56.8-58.9)
//   3/4  (3 rods)           42.5-45 mV (2026-10-10/12)
//   1/2  (2 rods)           35.5 mV    (2026-10-13, VIN 12.6-12.7 V)
//   empty (1 rod)           31.0-31.2 mV (2026-10-23, VIN 12.36 V); 34.7 mV on 2026-10-08 (VIN not measured)
// Empty and 1/2 differ by less than the supply moves the reading, so the lowest level rod cannot be told apart:
// everything below the 3/4 band is shown as "<=50 %" (user decision 2026-10-23) - honest, and never "empty" for a
// half tank nor "half" for an empty one without saying so. Band edges sit midway between neighbouring states.
// No supply correction (user decision 2026-10-13): a voltage-proportional correction was tried and proved wrong -
// with the charger on, the same 1/2 tank read 54.9 mV at both 13.28 and 13.67 V. The charger acts more like a fixed
// offset per charger mode (+19.4 mV charging at 13.3-13.7 V, ~+8 mV in maintenance at 12.55 V), cause unknown.
// The bands are for the CHARGER OFF; with the charger on, tanks read high (about one step). The supply is still
// measured during each press and logged, as data for handling the charger later.
struct TankBand { float min_mv; float frac; };
static const TankBand TANK_BANDS[] = {
    {50.0f, 1.00f},      // full
    {39.0f, 0.75f},      // 3/4: midway between 1/2 (35.5) and the lowest 3/4 (42.5)
};                       // below the last band: 1/2 or less ("<=50 %")
static constexpr float TANK_LOW_FRAC = 0.5f;           // marker position for "1/2 or less"
static const char     *TANK_LOW_TEXT = "\xE2\x89\xA4" "50 %";   // U+2264 is in font_num_88

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
static uint32_t  g_bl_test_end = 0;
// Screen/backlight bookkeeping for the diagnostics page: the backlight once stayed on in the van although the
// screen had blanked, and neither path could be made to fail on the bench. These fields say what actually ran.
static bool     g_bl_on         = true;
static uint32_t g_bl_change_ms  = 0;
static time_t   g_bl_change_time = 0;
static bool     g_bl_driver_ok  = true;
static bool     g_bl_exio_ok    = true;
static uint32_t g_reasserts     = 0;              // "off" pushed to the chip again while blanked (see below)
static uint32_t g_exp_cfg_fail  = 0;              // failed forced writes to the expander
static uint32_t g_wakes         = 0;
static uint32_t g_reassert_ms   = 0;

// Last few screen events, so the Diag page still shows what happened before the touch that woke the screen
static constexpr int SCREEN_LOG_ROWS = 5;
struct ScreenEvent {
    time_t   when;
    uint32_t ms;
    char     text[56];
};
static ScreenEvent g_scr_log[SCREEN_LOG_ROWS];
static int         g_scr_log_count = 0;

static void screen_log(const char *fmt, ...)
{
    char buf[56];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    vlog("SCREEN %s", buf);
    for (int i = SCREEN_LOG_ROWS - 1; i > 0; i--) g_scr_log[i] = g_scr_log[i - 1];
    g_scr_log[0].when = time(nullptr);
    g_scr_log[0].ms   = millis();
    strlcpy(g_scr_log[0].text, buf, sizeof(g_scr_log[0].text));
    if (g_scr_log_count < SCREEN_LOG_ROWS) g_scr_log_count++;
    Serial.printf("[screen] %s\n", buf);
}              // backlight test: light stays off until this millis()
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

// Themes for the gauge page (user, 2026-10-23): "Toptron Classic" = the dial scales above; "Duo" = two rectangles
// side by side, battery left (red/yellow/green fill), water right (blue fill), value under each. Both layers are
// built and updated; the theme only decides which one is visible. Saved in NVS "globebus"/"theme".
static const char *THEME_OPTS   = "Toptron Classic\nDuo";
// Night mode (user, 2026-10-23): same layout, every colour mapped to dim red on black by an LVGL colour filter.
// The backlight is on/off only on this board, so colour is the only lever. Auto = sunset to sunrise (forecast).
static const char *NIGHT_OPTS   = "Off\nAuto (sunset)\nAlways";
static int         g_night_idx  = 1;
static lv_obj_t   *dd_night     = nullptr;
static bool        g_night_on   = false;
static bool        g_web_on     = false;          // phone page server started (see web_poll)
static int         g_theme_idx  = 0;
static lv_obj_t   *dd_theme     = nullptr;
static lv_obj_t   *cont_classic = nullptr;
static lv_obj_t   *cont_duo     = nullptr;
static lv_obj_t   *duo_bat_fill = nullptr;
static lv_obj_t   *duo_wat_fill = nullptr;
static lv_obj_t   *duo_lbl_volt = nullptr;
static lv_obj_t   *duo_lbl_pct  = nullptr;

// page 2 (weather)
static lv_obj_t *lbl_wx_clock = nullptr;
static lv_obj_t *lbl_wx_place = nullptr;
static lv_obj_t *lbl_wx_icon  = nullptr;
static lv_obj_t *lbl_wx_temp  = nullptr;
static lv_obj_t *lbl_wx_cond  = nullptr;
static lv_obj_t *lbl_wx_det[4] = {};          // wind, humidity, today min/max, sunrise/sunset
static lv_obj_t *lbl_wx_day[WX_DAYS][4] = {};   // label, icon, temps, pop
static lv_obj_t *lbl_wx_foot  = nullptr;
static lv_obj_t *lbl_wx_msg   = nullptr;
static lv_obj_t *wx_content   = nullptr;

// page 3 (big clock)
static lv_obj_t *lbl_big_clock = nullptr;
static lv_obj_t *lbl_hist_clock = nullptr;           // history page header clock
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
static lv_obj_t *lbl_screen  = nullptr;
static lv_obj_t *lbl_scrlog  = nullptr;

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
    float    supply_v    = NAN;    // supply measured during the press (NAN = no van feed)
    uint32_t supply_ms   = 0;      // millis() of the last supply read during the press
    bool     shown       = false;  // a value has been published for this press
    uint32_t live_ms     = 0;      // millis() of the last live update
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
static float    g_idle_mv    = NAN;               // latest idle A0-A1 at the signal PGA (offset with nothing held)
static float    g_supply_v   = NAN;               // smoothed supply (board VIN), NAN = no van feed
static float    g_supply_raw = NAN;               // last single reading, for the diagnostics page
static uint32_t g_stuck      = 0;                 // "presses" ended by MAX_PRESS_MS
static bool     g_wire_loose = false;             // open-input detector state (see WIRE_LOOSE_MV)
static uint32_t g_wire_events = 0;
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
    static uint32_t last_logged_ms = 0, logged_total = 0;
    if (!last_logged_ms || millis() - last_logged_ms >= 10000) {
        vlog("I2C error %s during %s (total %lu, %lu since last line)", esp_err_to_name(err), where,
             (unsigned long)g_read_errors, (unsigned long)(g_read_errors - logged_total));
        last_logged_ms = millis();
        logged_total = g_read_errors;
    }
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

static void update_frost();
static void rem_tick();

// ---------- night mode -----------------------------------------------------------
static lv_style_t            g_night_style;
static lv_color_filter_dsc_t g_night_filter;

static lv_color_t night_filter_cb(const lv_color_filter_dsc_t *, lv_color_t c, lv_opa_t)
{
    lv_color32_t c32;
    c32.full = lv_color_to32(c);
    uint32_t y = (c32.ch.red * 77u + c32.ch.green * 150u + c32.ch.blue * 29u) >> 8;   // luminance
    return lv_color_make((uint8_t)(y * 160 / 255), (uint8_t)(y * 38 / 255), (uint8_t)(y * 30 / 255));
}

// The filter is not inherited in LVGL 8, so the style goes on every object (labels: main part only).
static void night_walk(lv_obj_t *o, bool on)
{
    static const lv_style_selector_t parts[] = {LV_PART_MAIN, LV_PART_INDICATOR, LV_PART_ITEMS, LV_PART_KNOB,
                                                LV_PART_SELECTED};
    lv_obj_remove_style(o, &g_night_style, LV_PART_ANY | LV_STATE_ANY);   // never twice
    if (on) {
        int n = lv_obj_check_type(o, &lv_label_class) ? 1 : 5;
        for (int i = 0; i < n; i++) lv_obj_add_style(o, &g_night_style, parts[i]);
    }
    uint32_t cnt = lv_obj_get_child_cnt(o);
    for (uint32_t i = 0; i < cnt; i++) night_walk(lv_obj_get_child(o, i), on);
}

static void night_apply(bool on, bool force = false)   // caller holds the LVGL lock
{
    static bool inited = false;
    if (!inited) {
        lv_style_init(&g_night_style);
        lv_color_filter_dsc_init(&g_night_filter, night_filter_cb);
        lv_style_set_color_filter_dsc(&g_night_style, &g_night_filter);
        lv_style_set_color_filter_opa(&g_night_style, LV_OPA_COVER);
        lv_style_set_img_recolor(&g_night_style, lv_color_hex(0x5A1E18));     // images: wave strip, marker
        lv_style_set_img_recolor_opa(&g_night_style, LV_OPA_COVER);
        lv_style_set_bg_img_recolor(&g_night_style, lv_color_hex(0x5A1E18));  // the "<=50 %" stripes
        lv_style_set_bg_img_recolor_opa(&g_night_style, 190);              // keep a hint of the stripes
        inited = true;
    }
    if (on == g_night_on && !force) return;
    bool changed = on != g_night_on;
    g_night_on = on;
    if (scr_main) night_walk(scr_main, on);
    if (scr_set)  night_walk(scr_set, on);
    if (changed) vlog("NIGHT mode %s", on ? "on" : "off");
}

static bool night_wanted()
{
    if (g_night_idx == 0) return false;
    if (g_night_idx == 2) return true;
    if (!time_valid()) return false;
    time_t now = time(nullptr);
    struct tm t;
    localtime_r(&now, &t);
    int m = t.tm_hour * 60 + t.tm_min;
    int sr = 7 * 60, ss = 20 * 60;                                     // without a forecast
    if (g_ns.wx_valid && g_ns.day[0].sunrise_min >= 0 && g_ns.day[0].sunset_min >= 0) {
        sr = g_ns.day[0].sunrise_min;
        ss = g_ns.day[0].sunset_min;
    }
    return m < sr || m >= ss;
}

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
    if (lbl_hist_clock) lv_label_set_text(lbl_hist_clock, hm);
    update_frost();
    night_apply(night_wanted());
    static int last_min = -1;                         // reminders: once a minute
    int min_now = (int)(time(nullptr) / 60);
    if (min_now != last_min) {
        last_min = min_now;
        rem_tick();
    }
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

// Tank fraction for a reading, from the measured bands (see TANK_BANDS).
static float tank_frac(float mv)
{
    for (const TankBand &b : TANK_BANDS) {
        if (mv >= b.min_mv) return b.frac;
    }
    return TANK_LOW_FRAC;
}

static bool tank_is_low(float mv)
{
    return mv < TANK_BANDS[sizeof(TANK_BANDS) / sizeof(TANK_BANDS[0]) - 1].min_mv;
}

// ---------- theme "Duo" -------------------------------------------------------

static constexpr int DUO_W = 210, DUO_H = 260, DUO_Y = 86, DUO_BAT_X = 115, DUO_WAT_X = 475;
static const lv_color_t DUO_RED    = lv_color_hex(0xE5483D);
static const lv_color_t DUO_YELLOW = lv_color_hex(0xE0B030);
static const lv_color_t DUO_GREEN  = lv_color_hex(0x639922);
static const lv_color_t DUO_BLUE   = lv_color_hex(0x3D78D8);

// Battery fill = USABLE capacity of a flooded lead-acid battery (user decision 2026-10-23; both batteries are
// flooded): 12.2 V (about 50 % state of charge, the usual lowest point to discharge to) is empty, 12.7 V (full at
// rest) is full, linear in between - the resting-voltage table is close to linear there (12.25 V ~50 %, 12.45 ~75 %,
// 12.65 ~100 %; Modern Survival Blog, Clever Solar Power, Wind & Solar, Inspire Clean Energy). 12.6 V shows 80 %,
// 12.4 V 40 %. Resting values only: under load it reads low, on charge (or with surface charge) it reads full.
static constexpr float BAT_EMPTY_V = 12.2f;
static constexpr float BAT_FULL_V  = 12.7f;
static float battery_fill(float volts)
{
    return clampf((volts - BAT_EMPTY_V) / (BAT_FULL_V - BAT_EMPTY_V), 0, 1);
}

// "<=50 %" is drawn striped: a 16 x 16 tile of diagonal stripes in two blues, repeated over the fill.
static lv_color_t   g_hatch_px[16 * 16];
static lv_img_dsc_t g_hatch_img;
static void make_hatch()
{
    for (int y = 0; y < 16; y++)
        for (int x = 0; x < 16; x++)
            g_hatch_px[y * 16 + x] = ((x + y) % 16) < 7 ? DUO_BLUE : lv_color_hex(0x2C5AA6);
    g_hatch_img.header.always_zero = 0;
    g_hatch_img.header.w  = 16;
    g_hatch_img.header.h  = 16;
    g_hatch_img.header.cf = LV_IMG_CF_TRUE_COLOR;
    g_hatch_img.data_size = sizeof(g_hatch_px);
    g_hatch_img.data      = (const uint8_t *)g_hatch_px;
}

// Box geometry: with the frost banner shown the boxes start lower (22 px under the banner), same bottom edge.
static constexpr int DUO_Y_BANNER = 114;
// Children of a box are placed inside its border (LVGL 8 content area), so all fill/wave maths use the inner
// height. Using the outer height put the wave 12 px (2 x border) too low - it covered the top of the stripes.
static constexpr int DUO_BORDER   = 6;
static int        g_duo_y      = DUO_Y;
static int        g_duo_h      = DUO_H;
static lv_obj_t  *duo_bat_box  = nullptr;
static lv_obj_t  *duo_wat_box  = nullptr;
static lv_obj_t  *duo_wave     = nullptr;
static lv_obj_t  *duo_bolt     = nullptr;
static bool       g_duo_live   = false;        // battery side shows the live leisure value, not a press

struct DuoFillState { float frac; lv_color_t color; bool striped; bool shown; };
static DuoFillState g_bat_fs = {}, g_wat_fs = {};
static DuoFillState &fill_state(lv_obj_t *fill) { return fill == duo_wat_fill ? g_wat_fs : g_bat_fs; }

// Water surface: a strip one wave period wider than the box, drifting left by one period and repeating. The strip
// is periodic, so the restart is invisible (the first mockup's strip was too short and showed its end).
static constexpr int WAVE_PERIOD = 104;
static constexpr int WAVE_H      = 14;
static lv_img_dsc_t  g_wave_img;
static uint8_t      *g_wave_px = nullptr;
static void make_wave()
{
    const int w = DUO_W + WAVE_PERIOD;
    g_wave_px = (uint8_t *)heap_caps_malloc(w * WAVE_H * 3, MALLOC_CAP_SPIRAM);
    if (!g_wave_px) return;
    lv_color_t c = DUO_BLUE;
    for (int x = 0; x < w; x++) {
        float top = 7.0f - 4.5f * sinf(2.0f * (float)M_PI * x / WAVE_PERIOD);   // 2.5 .. 11.5 px down the strip
        for (int y = 0; y < WAVE_H; y++) {
            uint8_t *px = g_wave_px + (y * w + x) * 3;          // TRUE_COLOR_ALPHA, 16-bit: colour lo, hi, alpha
            px[0] = c.full & 0xFF;
            px[1] = c.full >> 8;
            px[2] = (uint8_t)(clampf((y + 1.0f) - top, 0, 1) * 255.0f);   // 1 px soft edge
        }
    }
    g_wave_img.header.always_zero = 0;
    g_wave_img.header.w  = w;
    g_wave_img.header.h  = WAVE_H;
    g_wave_img.header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA;
    g_wave_img.data_size = w * WAVE_H * 3;
    g_wave_img.data      = g_wave_px;
}

static int duo_inner_h() { return g_duo_h - 2 * DUO_BORDER; }

// The strip sits ON the fill: its bottom edge meets the fill's top, so on the striped "<=50 %" fill the stripes run
// all the way up and the wave starts where they end.
static void wave_place(int fill_h)
{
    if (duo_wave) lv_obj_set_y(duo_wave, duo_inner_h() - fill_h - WAVE_H);
}

static void wave_drift_cb(void *obj, int32_t v) { lv_obj_set_x((lv_obj_t *)obj, v); }

static void wave_run(bool on)
{
    if (!duo_wave || !g_wave_px) return;
    if (!on) {
        lv_anim_del(duo_wave, wave_drift_cb);
        lv_obj_add_flag(duo_wave, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    lv_obj_clear_flag(duo_wave, LV_OBJ_FLAG_HIDDEN);
    if (lv_anim_get(duo_wave, wave_drift_cb)) return;
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, duo_wave);
    lv_anim_set_exec_cb(&a, wave_drift_cb);
    lv_anim_set_values(&a, 0, -WAVE_PERIOD);
    lv_anim_set_time(&a, 3200);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_path_cb(&a, lv_anim_path_linear);
    lv_anim_start(&a);
}

// Fills rise (or fall) to the new level instead of jumping.
static void fill_height_cb(void *obj, int32_t h)
{
    lv_obj_t *f = (lv_obj_t *)obj;
    lv_obj_set_height(f, h);
    lv_obj_align(f, LV_ALIGN_BOTTOM_MID, 0, 0);
    if (f == duo_wat_fill) wave_place(h);
}

static void duo_set_fill(lv_obj_t *fill, float frac, lv_color_t color, bool striped)
{
    DuoFillState &fs = fill_state(fill);
    fs = {frac, color, striped, true};
    int h = (int)(clampf(frac, 0, 1) * duo_inner_h() + 0.5f);
    lv_anim_del(fill, fill_height_cb);
    if (h <= 0) {
        fs.shown = false;
        lv_obj_add_flag(fill, LV_OBJ_FLAG_HIDDEN);
        if (fill == duo_wat_fill) wave_run(false);
        return;
    }
    int from = lv_obj_has_flag(fill, LV_OBJ_FLAG_HIDDEN) ? 0 : lv_obj_get_height(fill);
    lv_obj_set_style_bg_color(fill, color, 0);
    lv_obj_set_style_bg_img_src(fill, striped ? &g_hatch_img : nullptr, 0);
    fill_height_cb(fill, from);
    lv_obj_clear_flag(fill, LV_OBJ_FLAG_HIDDEN);
    if (fill == duo_wat_fill) wave_run(true);
    if (from == h) return;
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, fill);
    lv_anim_set_exec_cb(&a, fill_height_cb);
    lv_anim_set_values(&a, from, h);
    lv_anim_set_time(&a, 700);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_start(&a);
}

static void duo_hide_fill(lv_obj_t *fill)
{
    fill_state(fill).shown = false;
    lv_anim_del(fill, fill_height_cb);
    lv_obj_add_flag(fill, LV_OBJ_FLAG_HIDDEN);
    if (fill == duo_wat_fill) wave_run(false);
}

// Slow pulse of the battery fill while charging.
static void pulse_cb(void *obj, int32_t v) { lv_obj_set_style_bg_opa((lv_obj_t *)obj, (lv_opa_t)v, 0); }
static void bat_pulse(bool on)
{
    if (!on) {
        lv_anim_del(duo_bat_fill, pulse_cb);
        lv_obj_set_style_bg_opa(duo_bat_fill, LV_OPA_COVER, 0);
        return;
    }
    if (lv_anim_get(duo_bat_fill, pulse_cb)) return;
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, duo_bat_fill);
    lv_anim_set_exec_cb(&a, pulse_cb);
    lv_anim_set_values(&a, LV_OPA_COVER, 158);
    lv_anim_set_time(&a, 1000);
    lv_anim_set_playback_time(&a, 1000);
    lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_in_out);
    lv_anim_start(&a);
}

// Frost banner on/off moves the boxes; re-apply the current fills to the new height.
static void duo_layout(bool banner)
{
    int y = banner ? DUO_Y_BANNER : DUO_Y;
    if (y == g_duo_y || !duo_bat_box) return;
    g_duo_y = y;
    g_duo_h = DUO_Y + DUO_H - y;
    lv_obj_t *boxes[2] = {duo_bat_box, duo_wat_box};
    for (lv_obj_t *b : boxes) {
        lv_obj_set_y(b, g_duo_y);
        lv_obj_set_height(b, g_duo_h);
    }
    lv_obj_center(duo_bolt);
    lv_obj_t *fills[2] = {duo_bat_fill, duo_wat_fill};
    for (lv_obj_t *f : fills) {
        DuoFillState fs = fill_state(f);
        if (fs.shown) {
            lv_anim_del(f, fill_height_cb);
            fill_height_cb(f, (int)(clampf(fs.frac, 0, 1) * duo_inner_h() + 0.5f));
        }
    }
}

// Live leisure battery (no rocker needed): the A2 divider reads the Toptron's feed, i.e. the leisure battery minus
// the feed's drop. Offsets: screen lit 0.40 V (2026-10-22: 12.1 V at the terminals, VIN 11.70 V during a press);
// screen dark 0.15 V is an ESTIMATE (VIN rises 0.2-0.3 V when the backlight goes off) - calibrate with a meter.
// Only with a van feed: on USB or a power bank there is no supply reading and the battery side shows "-- V".
static constexpr float LIVE_OFFSET_LIT_V  = 0.40f;
static constexpr float LIVE_OFFSET_DARK_V = 0.15f;
static constexpr float CHARGING_V         = 13.0f;    // above any resting lead-acid voltage
static float live_battery_v()
{
    if (isnan(g_supply_v)) return NAN;
    return g_supply_v + (g_screen_off ? LIVE_OFFSET_DARK_V : LIVE_OFFSET_LIT_V);
}

static void duo_show_live()                      // caller holds the LVGL lock
{
    float v = live_battery_v();
    if (isnan(v) || g_theme_idx != 1) {
        duo_hide_fill(duo_bat_fill);
        bat_pulse(false);
        lv_obj_add_flag(duo_bolt, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(duo_lbl_volt, "-- V");
        lv_obj_set_style_text_color(duo_lbl_volt, COL_DIM, 0);
        g_duo_live = false;
        return;
    }
    float bf = battery_fill(v);
    duo_set_fill(duo_bat_fill, bf, bf < 0.25f ? DUO_RED : (bf < 0.5f ? DUO_YELLOW : DUO_GREEN), false);
    char t[16];
    snprintf(t, sizeof(t), "%.1f V", v);
    lv_label_set_text(duo_lbl_volt, t);
    lv_obj_set_style_text_color(duo_lbl_volt, COL_TEXT, 0);
    bool charging = v >= CHARGING_V;
    bat_pulse(charging);
    if (charging) lv_obj_clear_flag(duo_bolt, LV_OBJ_FLAG_HIDDEN);
    else          lv_obj_add_flag(duo_bolt, LV_OBJ_FLAG_HIDDEN);
    g_duo_live = true;
}

static void apply_theme()
{
    if (!cont_classic || !cont_duo) return;
    if (g_theme_idx == 1) {
        lv_obj_add_flag(cont_classic, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(cont_duo, LV_OBJ_FLAG_HIDDEN);
        if (!g_display_ms) duo_show_live();
    } else {
        if (duo_bolt) duo_show_live();            // theme is Classic: this stops the pulse and clears the side
        lv_obj_add_flag(cont_duo, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(cont_classic, LV_OBJ_FLAG_HIDDEN);
    }
}

// With the charger charging (VIN above 13.0 V, bulk/absorption) the tank output reads high by a roughly fixed
// amount: clean tank at 1/2 read 54.9 mV at 13.28 and 13.67 V vs 35.5 mV with the charger off (2026-10-13) -> -19.4 mV.
// One level, one day of data. Maintenance charging (~12.5 V at VIN, ~+8 mV) cannot be told from voltage: not handled.
static constexpr float TANK_CHARGE_SUPPLY_V  = 13.0f;
static constexpr float TANK_CHARGE_OFFSET_MV = 19.4f;
static float tank_mv(float mv)
{
    float s = g_press.supply_v;
    return (!isnan(s) && s > TANK_CHARGE_SUPPLY_V) ? mv - TANK_CHARGE_OFFSET_MV : mv;
}

// Last shown reading, for the phone page.
static float  g_last_v     = NAN;
static float  g_last_tfill = NAN;
static bool   g_last_low   = false;
static char   g_last_tank[16] = "";
static time_t g_last_at    = 0;

static void show_value(float mv, bool live = false)
{
    float volts = VOLT_OFFSET + VOLT_PER_MV * mv;
    if (volts > PLAUSIBLE_MAX_V) {                // cannot be a real gauge reading - do not display it
        if (live) return;                         // judged once, on the final value
        g_rejected++;
        Serial.printf("[display] REJECTED %.2f mV -> %.2f V (above %.1f V; total %lu)\n", mv, volts,
                      PLAUSIBLE_MAX_V, (unsigned long)g_rejected);
        set_state_text("Ignored an implausible reading");
        return;
    }
    float frac  = (volts - DIAL_V_MIN) / (DIAL_V_MAX - DIAL_V_MIN);
    float tmv   = tank_mv(mv);
    float tfrac = tank_frac(tmv);
    float pct   = tfrac * 100.0f;
    bool  red   = volts < RED_LOW_V || volts > RED_HIGH_V;
    char v[16], p[16];
    snprintf(v, sizeof(v), "%.1f V", volts);
    if (tank_is_low(tmv)) snprintf(p, sizeof(p), "%s", TANK_LOW_TEXT);
    else                  snprintf(p, sizeof(p), "%.0f %%", pct);
    if (!live) {
        g_last_v = volts;
        g_last_tfill = tfrac;
        g_last_low = tank_is_low(tmv);
        snprintf(g_last_tank, sizeof(g_last_tank), "%s", tank_is_low(tmv) ? "<=50 %" : p);
        g_last_at = time(nullptr);
    }

    lvgl_port_lock(-1);
    lv_label_set_text(lbl_volt, v);
    lv_label_set_text(lbl_pct, p);
    lv_obj_set_style_text_color(lbl_volt, red ? COL_WARN : COL_TEXT, 0);
    lv_obj_set_style_text_color(lbl_pct, COL_TEXT, 0);
    place_marker(mk_bat, BAT_BAR_Y, frac);
    place_marker(mk_wat, WAT_BAR_Y, tfrac);
    if (g_theme_idx == 1) {                       // Duo only: no animations running on a hidden layer
        float bf = battery_fill(volts);
        lv_color_t bc = bf < 0.25f ? DUO_RED : (bf < 0.5f ? DUO_YELLOW : DUO_GREEN);
        duo_set_fill(duo_bat_fill, bf, bc, false);
        duo_set_fill(duo_wat_fill, tfrac, DUO_BLUE, tank_is_low(tmv));
        bat_pulse(false);                         // a press shows the pressed channel, not the live battery
        lv_obj_add_flag(duo_bolt, LV_OBJ_FLAG_HIDDEN);
        g_duo_live = false;
        lv_label_set_text(duo_lbl_volt, v);
        lv_label_set_text(duo_lbl_pct, p);
        lv_obj_set_style_text_color(duo_lbl_volt, COL_TEXT, 0);
        lv_obj_set_style_text_color(duo_lbl_pct, COL_TEXT, 0);
    }
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
    duo_hide_fill(duo_wat_fill);
    lv_label_set_text(duo_lbl_pct, "-- %");
    lv_obj_set_style_text_color(duo_lbl_pct, COL_DIM, 0);
    duo_show_live();                              // battery side falls back to the live leisure value
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

// Backlight switching, and why it is done the way it is. Three findings, all from the van:
//
// 1. NEVER read the expander's pins. Reading the CH422G means sending its "read IO" command, which disturbs the
//    output stage - the pins revert and the backlight comes on. A read-back check did exactly that: every 5 s the
//    read switched the light on and the code "corrected" it, 178 times in half an hour.
// 2. The vendor driver does not always write. digitalWrite() compares the new level with its own cached copy of
//    the output register and sends nothing when they match (esp_io_expander_set_level, "Write to reg only when
//    different"). So re-writing "off" is a no-op on the wire: in the van the light came back on by itself while
//    96 such re-asserts all reported success without a single byte reaching the chip.
// 3. Something in the van does change the chip's state behind the driver's back (a restart clears it; the bench
//    never shows it). Two ways it can: the chip resets, and its output register returns to the power-on 0xFF -
//    every pin high, backlight on; or it loses its "all IO output" setting and the pins float. Either way the
//    driver's cache still says "off".
//
// So after switching, and periodically while blanked, push both the output mode and the cached output byte to
// the chip unconditionally, through the driver's low-level write. Reading the *cached* byte is fine - it is
// memory only; the harmful read is the chip's "read IO" command, which this never sends.
// (The driver also skips the IO write when the whole byte would be zero; the LCD reset pin keeps it non-zero.)
static bool expander_force_write()
{
    auto base = (g_board && g_board->getIO_Expander()) ? g_board->getIO_Expander()->getBase() : nullptr;
    auto h = base ? base->getDeviceHandle() : nullptr;
    if (!h || !h->read_output_reg || !h->write_output_reg) return false;

    bool ok = static_cast<esp_expander::CH422G *>(base)->enableAllIO_Output();
    uint32_t out = 0;
    ok = (h->read_output_reg(h, &out) == ESP_OK) && ok;       // cached copy, no I2C traffic
    if (g_bl_on) out |= (1u << BACKLIGHT_EXIO);
    else         out &= ~(1u << BACKLIGHT_EXIO);
    ok = (h->write_output_reg(h, out) == ESP_OK) && ok;       // always sent, unlike digitalWrite()
    if (!ok) g_exp_cfg_fail++;
    return ok;
}

static void backlight_set(bool on)
{
    auto bl = g_board ? g_board->getBacklight() : nullptr;
    auto exp_dev = (g_board && g_board->getIO_Expander()) ? g_board->getIO_Expander()->getBase() : nullptr;

    g_bl_on = on;
    g_bl_driver_ok = bl ? (on ? bl->on() : bl->off()) : false;
    g_bl_exio_ok   = exp_dev ? exp_dev->digitalWrite(BACKLIGHT_EXIO, on ? 1 : 0) : false;
    g_bl_exio_ok   = expander_force_write() && g_bl_exio_ok;

    g_bl_change_ms = millis();
    g_bl_change_time = time(nullptr);
    g_reassert_ms = millis();
    screen_log("backlight %s (driver %s, pin %s)", on ? "on" : "off", g_bl_driver_ok ? "ok" : "FAIL",
               g_bl_exio_ok ? "ok" : "FAIL");
}

// While the screen is blanked, push the "off" state to the chip again every few seconds (see above).
static void backlight_reassert()
{
    if (!g_screen_off || millis() - g_reassert_ms < BL_REASSERT_MS) return;
    g_reassert_ms = millis();
    g_reasserts++;
    if (!expander_force_write()) {
        g_bl_exio_ok = false;
        screen_log("re-assert write FAILED (%lu)", (unsigned long)g_reasserts);
    }
}

static void screen_off()
{
    if (g_screen_off) return;
    lvgl_port_lock(-1);
    scr_prev = lv_scr_act();
    lv_scr_load(scr_blank);
    lvgl_port_unlock();
    delay(60);                                    // let the black frame reach the panel before the light goes out
    backlight_set(false);
    g_screen_off = true;
    screen_log("screen blanked (timeout)");
}

static void screen_on(const char *why)
{
    if (!g_screen_off) return;
    lvgl_port_lock(-1);
    lv_scr_load(scr_prev ? scr_prev : scr_main);
    lv_disp_trig_activity(nullptr);
    lvgl_port_unlock();
    backlight_set(true);
    g_screen_off = false;
    g_wakes++;
    screen_log("woke: %s", why);
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
    vlog("SETTING keep reading %u s (0 = until release)", HOLD_SECONDS[g_hold_idx]);
}

static void bl_test_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
    g_bl_test_end = millis() + BL_TEST_MS;
    if (g_bl_test_end == 0) g_bl_test_end = 1;
    Serial.println("[screen] backlight test: off for 5 s");
}

static void theme_dd_cb(lv_event_t *e)
{
    g_theme_idx = lv_dropdown_get_selected(dd_theme);
    Preferences p;
    p.begin("globebus", false);
    p.putUChar("theme", (uint8_t)g_theme_idx);
    p.end();
    apply_theme();                                // called from an LVGL event: the lock is already held
    vlog("SETTING theme %s", g_theme_idx == 1 ? "Duo" : "Toptron Classic");
}

static void timeout_dd_cb(lv_event_t *e)
{
    g_timeout_idx = lv_dropdown_get_selected(dd_timeout);
    Preferences p;
    p.begin("globebus", false);
    p.putUChar("disp_to", (uint8_t)g_timeout_idx);
    p.end();
    Serial.printf("[screen] timeout set to %u min (0 = never)\n", TIMEOUT_MIN[g_timeout_idx]);
    vlog("SETTING display timeout %u min (0 = never)", TIMEOUT_MIN[g_timeout_idx]);
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
    g_press.supply_v    = g_supply_v;             // until the first read during the press
    g_press.supply_ms   = 0;
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
    if (!g_press.supply_ms || millis() - g_press.supply_ms >= 100) {   // the supply, every 0.1 s while held
        g_press.supply_ms = millis();
        float a2;
        if (ads_read_volts(MUX_AIN2, PGA_2V048, &a2)) {
            float sv = supply_from_a2(a2);
            if (sv < SUPPLY_MIN_V)         g_press.supply_v = NAN;
            else if (isnan(g_press.supply_v)) g_press.supply_v = sv;
            else                           g_press.supply_v += 0.5f * (sv - g_press.supply_v);
        }
    }
    if (t >= WIN_START_MS && (!g_press.shown || millis() - g_press.live_ms >= LIVE_UI_MS)) {
        g_press.shown   = true;
        g_press.live_ms = millis();
        show_value(mv, true);
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
    // value ~0.2 s before release (the release edge itself is excluded); short presses: the last sample
    float end_mv = g_recent_count > END_BACK_SAMPLES ? g_recent[g_recent_head] : g_press.last_mv;
    if (dur >= MIN_PRESS_MS) {
        show_value(end_mv);                       // the settled value stays on screen
    } else {
        Serial.printf("[press] #%lu ignored: %lu ms < %lu ms\n", (unsigned long)g_press.n, (unsigned long)dur,
                      (unsigned long)MIN_PRESS_MS);
        set_state_text(g_display_ms ? "Held reading" : "Hold a selector on the panel");
    }
    g_press.shown = true;

    char a[4][12];
    for (int i = 0; i < 4; i++) fmt_opt(a[i], sizeof(a[i]), g_press.at_mv[i]);
    Serial.printf("[press] #%lu END  %.2f s, %lu samples | first %.2f  @0.2s %s  @0.5s %s  @1s %s  @2s %s  "
                  "peak %.2f  end(-0.2s) %.2f  window %.2f mV\n",
                  (unsigned long)g_press.n, dur / 1000.0f, (unsigned long)g_press.samples, g_press.first_mv, a[0],
                  a[1], a[2], a[3], g_press.peak_mv, end_mv, g_press.win_mv);

    {
        float shown = end_mv;
        float volts = VOLT_OFFSET + VOLT_PER_MV * shown;
        float pct   = tank_frac(tank_mv(shown)) * 100.0f;   // "<=50" is logged as 50; charger-corrected
        vlog("PRESS #%lu %.1fs first %.2f @0.5s %s end %.2f peak %.2f mV -> %.2f V / tank %.0f %%, supply %.2f V",
             (unsigned long)g_press.n, dur / 1000.0f, g_press.first_mv, a[1], end_mv, g_press.peak_mv, volts, pct,
             g_press.supply_v);
        vlog_flush();                             // a reading is worth keeping even if the power is cut next
    }

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
    char s0[16], s1[16], sd[16], sup[40], webaddr[64], line[600];
    if (g_web_on && WiFi.isConnected())
        snprintf(webaddr, sizeof(webaddr), "http://globebus.local  or  http://%s", WiFi.localIP().toString().c_str());
    else
        snprintf(webaddr, sizeof(webaddr), "needs Wi-Fi");
    if (isnan(g_supply_v)) snprintf(sup, sizeof(sup), "-- (no van feed)");
    else                   snprintf(sup, sizeof(sup), "%.2f V (now %.2f V)", g_supply_v, g_supply_raw);
    if (isnan(g_idle_mv)) snprintf(sd, sizeof(sd), "--");
    else                  snprintf(sd, sizeof(sd), "%.3f", g_idle_mv);
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
             "Idle: A0 %s V   A1 %s V (signal- vs board GND)   A0-A1 %s mV   I2C/ADC errors: %lu%s\n"
             "Supply %s   Uptime %02lu:%02lu:%02lu   presses %lu (rejected %lu, stuck %lu)   wire %s (%lu)   free heap %lu B (internal %lu B)   built " __DATE__ " " __TIME__
             "\nVolts %s: V = %.3f + %.5f x mV   tank (charger off): >=%.1f full, >=%.1f 3/4, below <=1/2 mV\nPhone page: %s",
             s0, s1, sd, (unsigned long)g_read_errors, err_when, sup, (unsigned long)(s / 3600),
             (unsigned long)((s / 60) % 60), (unsigned long)(s % 60), (unsigned long)g_press.n,
             (unsigned long)g_rejected, (unsigned long)g_stuck, g_wire_loose ? "OPEN" : "ok",
             (unsigned long)g_wire_events,
             (unsigned long)ESP.getFreeHeap(),
             (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             VOLT_CALIBRATED ? "calibrated" : "PROVISIONAL", VOLT_OFFSET, VOLT_PER_MV,
             TANK_BANDS[0].min_mv, TANK_BANDS[1].min_mv, webaddr);
    char sline[260];
    if (g_bl_change_ms) {
        uint32_t ago = (millis() - g_bl_change_ms) / 1000;
        char when[24] = "";
        if (g_bl_change_time > TIME_VALID_AFTER) {
            struct tm b;
            localtime_r(&g_bl_change_time, &b);
            snprintf(when, sizeof(when), " at %02d:%02d,", b.tm_hour, b.tm_min);
        }
        snprintf(sline, sizeof(sline),
                 "Screen: %s, backlight %s (last switched%s %lu s ago; driver %s, EXIO%d %s)   wakes: %lu",
                 g_screen_off ? "blanked" : "showing", g_bl_on ? "ON" : "off", when, (unsigned long)ago,
                 g_bl_driver_ok ? "ok" : "FAILED", BACKLIGHT_EXIO, g_bl_exio_ok ? "ok" : "FAILED",
                 (unsigned long)g_wakes);
    } else {
        snprintf(sline, sizeof(sline), "Screen: showing, backlight ON (never switched yet)   wakes: %lu",
                 (unsigned long)g_wakes);
    }
    size_t sl = strlen(sline);
    sl += snprintf(sline + sl, sizeof(sline) - sl, "   re-asserts: %lu", (unsigned long)g_reasserts);
    if (g_exp_cfg_fail) sl += snprintf(sline + sl, sizeof(sline) - sl, "   expander write fails: %lu",
                                       (unsigned long)g_exp_cfg_fail);
    static char slog[SCREEN_LOG_ROWS * 80 + 8];
    size_t ll = 0;
    slog[0] = '\0';
    for (int i = 0; i < g_scr_log_count && ll < sizeof(slog) - 80; i++) {
        const ScreenEvent &e = g_scr_log[i];
        char when[16];
        if (e.when > TIME_VALID_AFTER) {
            struct tm t;
            localtime_r(&e.when, &t);
            snprintf(when, sizeof(when), "%02d:%02d:%02d", t.tm_hour, t.tm_min, t.tm_sec);
        } else {
            snprintf(when, sizeof(when), "+%lus", (unsigned long)(e.ms / 1000));
        }
        ll += snprintf(slog + ll, sizeof(slog) - ll, "%s  %s\n", when, e.text);
    }
    lvgl_port_lock(-1);
    lv_label_set_text(lbl_screen, sline);
    lv_label_set_text(lbl_scrlog, slog[0] ? slog : "(no screen events yet)");
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
    if (g_night_on) night_apply(true, true);      // objects created since (Wi-Fi list) get the night colours too
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
    for (int i = 0; i < 4; i++) {
        lv_obj_t *d = make_rect(tile, 400 - 35 + i * 20, 462, 10, 10, i == active ? COL_TEXT : lv_color_hex(0x3A4750));
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

static lv_obj_t *duo_box(lv_obj_t *parent, int x, lv_obj_t **fill)
{
    lv_obj_t *box = lv_obj_create(parent);
    lv_obj_remove_style_all(box);
    lv_obj_set_pos(box, x, g_duo_y);
    lv_obj_set_size(box, DUO_W, g_duo_h);
    lv_obj_set_style_radius(box, 16, 0);
    lv_obj_set_style_clip_corner(box, true, 0);
    lv_obj_set_style_border_color(box, lv_color_white(), 0);
    lv_obj_set_style_border_width(box, DUO_BORDER, 0);
    lv_obj_set_style_border_post(box, true, 0);      // outline drawn over the fill
    lv_obj_clear_flag(box, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    *fill = lv_obj_create(box);
    lv_obj_remove_style_all(*fill);
    lv_obj_set_width(*fill, DUO_W);
    lv_obj_set_style_bg_opa(*fill, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_img_tiled(*fill, true, 0);
    lv_obj_clear_flag(*fill, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(*fill, LV_OBJ_FLAG_HIDDEN);
    return box;
}

static lv_obj_t *duo_value(lv_obj_t *parent, int center_x, const char *text)
{
    lv_obj_t *l = make_label(parent, &font_num_56, COL_DIM);
    lv_obj_set_width(l, 340);
    lv_obj_set_style_text_align(l, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(l, text);
    lv_obj_set_pos(l, center_x - 170, DUO_Y + DUO_H + 12);
    return l;
}

static void build_duo_layer(lv_obj_t *p)
{
    make_hatch();
    cont_duo = lv_obj_create(p);
    lv_obj_remove_style_all(cont_duo);
    lv_obj_set_size(cont_duo, 800, 480);
    lv_obj_clear_flag(cont_duo, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    duo_bat_box = duo_box(cont_duo, DUO_BAT_X, &duo_bat_fill);
    duo_wat_box = duo_box(cont_duo, DUO_WAT_X, &duo_wat_fill);
    make_wave();
    if (g_wave_px) {
        duo_wave = lv_img_create(duo_wat_box);    // above the water fill, under the outline (border_post)
        lv_img_set_src(duo_wave, &g_wave_img);
        lv_obj_clear_flag(duo_wave, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_flag(duo_wave, LV_OBJ_FLAG_HIDDEN);
    }
    duo_bolt = make_label(duo_bat_box, &lv_font_montserrat_48, lv_color_white());
    lv_label_set_text(duo_bolt, LV_SYMBOL_CHARGE);
    lv_obj_center(duo_bolt);
    lv_obj_add_flag(duo_bolt, LV_OBJ_FLAG_HIDDEN);
    duo_lbl_volt = duo_value(cont_duo, DUO_BAT_X + DUO_W / 2, "-- V");
    duo_lbl_pct  = duo_value(cont_duo, DUO_WAT_X + DUO_W / 2, "-- %");
}

// ---------- frost warning ------------------------------------------------------
// From open-meteo's daily minimum: the coming night is tomorrow's minimum (it falls around dawn); between midnight
// and 06:00 it is today's. At or below FROST_C the gauge page shows the banner; Duo's boxes move down to make room.
static constexpr float FROST_C = 0.0f;
static lv_obj_t *banner_frost = nullptr;
static lv_obj_t *lbl_frost    = nullptr;
static bool      g_frost      = false;
static char      g_frost_text[96] = "";
// Low leisure battery (live value in the red band for 2 min; clears in the green band). Takes priority over frost.
static lv_obj_t *banner_low   = nullptr;
static lv_obj_t *lbl_low      = nullptr;
static bool      g_low_bat    = false;

static lv_obj_t *make_banner(lv_obj_t *p, lv_color_t bg, lv_color_t fg, lv_obj_t **lbl)
{
    lv_obj_t *b = lv_obj_create(p);
    lv_obj_remove_style_all(b);
    lv_obj_set_pos(b, 160, 62);
    lv_obj_set_size(b, 480, 30);
    lv_obj_set_style_radius(b, 15, 0);
    lv_obj_set_style_bg_color(b, bg, 0);
    lv_obj_set_style_bg_opa(b, LV_OPA_COVER, 0);
    lv_obj_clear_flag(b, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(b, LV_OBJ_FLAG_HIDDEN);
    *lbl = make_label(b, &font_text_18, fg);
    lv_label_set_text(*lbl, "");
    lv_obj_center(*lbl);
    return b;
}

static void update_banners()                     // caller holds the LVGL lock
{
    if (!banner_frost || !banner_low) return;
    bool low = g_low_bat, frost = g_frost && !low;
    if (low)   lv_obj_clear_flag(banner_low, LV_OBJ_FLAG_HIDDEN);
    else       lv_obj_add_flag(banner_low, LV_OBJ_FLAG_HIDDEN);
    if (frost) lv_obj_clear_flag(banner_frost, LV_OBJ_FLAG_HIDDEN);
    else       lv_obj_add_flag(banner_frost, LV_OBJ_FLAG_HIDDEN);
    duo_layout(low || frost);
}

static void build_frost_banner(lv_obj_t *p)
{
    banner_frost = make_banner(p, lv_color_hex(0x123A5C), lv_color_hex(0xCFE8FF), &lbl_frost);
    banner_low   = make_banner(p, lv_color_hex(0x5C1A16), lv_color_hex(0xFFC9C3), &lbl_low);
}

static void update_frost()                       // caller holds the LVGL lock
{
    if (!banner_frost) return;
    bool show = false;
    long t_round = 0;
    if (g_ns.wx_valid && time_valid()) {
        time_t now = time(nullptr);
        struct tm t;
        localtime_r(&now, &t);
        float tmin = g_ns.day[t.tm_hour < 6 ? 0 : 1].tmin;
        if (!isnan(tmin) && tmin <= FROST_C) {
            show = true;
            t_round = lroundf(tmin);
        }
    }
    static long last_round = LONG_MIN;
    if (show && t_round != last_round) {
        char b[96];
        // U+F2DC snowflake, U+2212 minus, U+00B0 degree, U+00B7 middle dot - all in font_text_18
        snprintf(b, sizeof(b), "\xEF\x8B\x9C  %s%ld\xC2\xB0" "C tonight \xC2\xB7 protect the water tank and pipes",
                 t_round < 0 ? "\xE2\x88\x92" : "", labs(t_round));
        lv_label_set_text(lbl_frost, b);
        lv_obj_center(lbl_frost);
        snprintf(g_frost_text, sizeof(g_frost_text), "%s%ld \xC2\xB0" "C tonight \xC2\xB7 protect the water tank and pipes",
                 t_round < 0 ? "-" : "", labs(t_round));
        last_round = t_round;
    }
    if (show == g_frost) return;
    g_frost = show;
    vlog("FROST warning %s (forecast low %ld C)", show ? "on" : "off", t_round);
    update_banners();
}

// ---------- page 2: leisure battery history ------------------------------------
static constexpr int CH_X = 60, CH_Y = 120, CH_W = 700, CH_H = 280;
static constexpr int HIST_CV_MIN = 1180, HIST_CV_MAX = 1420;      // chart range 11.8 .. 14.2 V
static lv_obj_t           *hist_chart     = nullptr;
static lv_chart_series_t  *hist_ser       = nullptr;
static lv_obj_t           *lbl_hist_sum   = nullptr;
static lv_obj_t           *lbl_hist_empty = nullptr;
static lv_obj_t           *btn_h24        = nullptr;
static lv_obj_t           *btn_h7d        = nullptr;
static lv_obj_t           *lbl_hist_x[5]  = {};
static lv_obj_t           *hist_chg[12]   = {};
static bool                g_hist_7d      = false;
static HistPoint          *g_hist_tmp     = nullptr;

static int hist_cv_y(float cv) { return CH_Y + CH_H - (int)((cv - HIST_CV_MIN) * CH_H / (HIST_CV_MAX - HIST_CV_MIN)); }

static void refresh_history()                   // caller holds the LVGL lock
{
    if (!hist_chart) return;
    if (!g_hist_tmp) g_hist_tmp = (HistPoint *)heap_caps_malloc(HIST_CAP * sizeof(HistPoint), MALLOC_CAP_SPIRAM);
    const int      n_buckets = g_hist_7d ? 336 : 288;              // 30 min or 5 min
    const uint32_t bucket_s  = g_hist_7d ? 1800 : HIST_STEP_S;
    lv_chart_set_point_count(hist_chart, n_buckets);
    lv_chart_set_all_value(hist_chart, hist_ser, LV_CHART_POINT_NONE);
    for (lv_obj_t *r : hist_chg) lv_obj_add_flag(r, LV_OBJ_FLAG_HIDDEN);
    static const char *X24[5] = {"-24 h", "-18 h", "-12 h", "-6 h", "now"};
    static const char *X7[5]  = {"-7 d", "-5 d", "-3.5 d", "-1.75 d", "now"};
    for (int i = 0; i < 5; i++) lv_label_set_text(lbl_hist_x[i], g_hist_7d ? X7[i] : X24[i]);
    lv_obj_set_style_bg_color(btn_h24, g_hist_7d ? COL_BG : COL_BTN, 0);
    lv_obj_set_style_bg_color(btn_h7d, g_hist_7d ? COL_BTN : COL_BG, 0);

    int n = 0;
    time_t now = time(nullptr);
    time_t from = now - (time_t)n_buckets * bucket_s;
    if (time_valid() && g_hist_tmp) n = hist_window(from, now, g_hist_tmp, HIST_CAP);
    if (n == 0) {
        lv_obj_clear_flag(lbl_hist_empty, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(lbl_hist_sum, "");
        lv_chart_refresh(hist_chart);
        return;
    }
    lv_obj_add_flag(lbl_hist_empty, LV_OBJ_FLAG_HIDDEN);

    // bucket averages + charging runs
    static int32_t sum[336];
    static uint16_t cnt[336];
    static bool chg[336];
    memset(sum, 0, sizeof(sum));
    memset(cnt, 0, sizeof(cnt));
    memset(chg, 0, sizeof(chg));
    const HistPoint *lo = nullptr, *hi = nullptr;
    for (int i = 0; i < n; i++) {
        const HistPoint &q = g_hist_tmp[i];
        int b = (int)((time_t)q.t - from) / (int)bucket_s;
        if (b < 0 || b >= n_buckets) continue;
        sum[b] += q.cv;
        cnt[b]++;
        if (q.flags & HIST_CHARGING) chg[b] = true;
        if (!lo || q.cv < lo->cv) lo = &q;
        if (!hi || q.cv > hi->cv) hi = &q;
    }
    for (int b = 0; b < n_buckets; b++) {
        if (cnt[b]) lv_chart_set_value_by_id(hist_chart, hist_ser, b,
                                             LV_CLAMP(HIST_CV_MIN, sum[b] / cnt[b], HIST_CV_MAX));
    }
    int r = 0;
    for (int b = 0; b < n_buckets && r < 12; b++) {
        if (!chg[b]) continue;
        int e = b;
        while (e + 1 < n_buckets && chg[e + 1]) e++;
        int x0 = CH_X + b * CH_W / n_buckets, x1 = CH_X + (e + 1) * CH_W / n_buckets;
        lv_obj_set_pos(hist_chg[r], x0, CH_Y);
        lv_obj_set_size(hist_chg[r], LV_MAX(2, x1 - x0), CH_H);
        lv_obj_clear_flag(hist_chg[r], LV_OBJ_FLAG_HIDDEN);
        r++;
        b = e;
    }
    lv_chart_refresh(hist_chart);

    const HistPoint &last = g_hist_tmp[n - 1];
    char tl[24], th[24], line[128];
    struct tm t;
    time_t tt = lo->t;
    localtime_r(&tt, &t);
    strftime(tl, sizeof(tl), g_hist_7d ? "%a %H:%M" : "%H:%M", &t);
    tt = hi->t;
    localtime_r(&tt, &t);
    strftime(th, sizeof(th), g_hist_7d ? "%a %H:%M" : "%H:%M", &t);
    snprintf(line, sizeof(line), "now %.2f V \xC2\xB7 lowest %.2f V (%s) \xC2\xB7 highest %.2f V (%s)",
             last.cv / 100.0f, lo->cv / 100.0f, tl, hi->cv / 100.0f, th);
    lv_label_set_text(lbl_hist_sum, line);
}

static void hist_range_cb(lv_event_t *e)
{
    g_hist_7d = lv_event_get_user_data(e) != nullptr;
    refresh_history();                           // LVGL event: lock already held
}

static void build_history_page(lv_obj_t *p)
{
    lv_obj_t *name = make_label(p, &lv_font_montserrat_26, COL_TEXT);
    lv_label_set_text(name, "Globebus");
    lv_obj_set_pos(name, 24, 18);
    lbl_hist_clock = make_label(p, &font_clock_44, COL_TEXT);
    lv_label_set_text(lbl_hist_clock, "--:--");
    lv_obj_align(lbl_hist_clock, LV_ALIGN_TOP_MID, 0, 18);

    btn_h24 = lv_btn_create(p);
    btn_h7d = lv_btn_create(p);
    lv_obj_t *btns[2] = {btn_h24, btn_h7d};
    const char *txt[2] = {"24 h", "7 d"};
    for (int i = 0; i < 2; i++) {
        lv_obj_set_size(btns[i], 80, 36);
        lv_obj_set_pos(btns[i], 590 + i * 92, 22);
        lv_obj_set_style_radius(btns[i], 18, 0);
        lv_obj_set_style_shadow_width(btns[i], 0, 0);
        lv_obj_set_style_border_color(btns[i], COL_BTN, 0);
        lv_obj_set_style_border_width(btns[i], 2, 0);
        lv_obj_add_event_cb(btns[i], hist_range_cb, LV_EVENT_CLICKED, i ? (void *)1 : nullptr);
        lv_obj_t *l = make_label(btns[i], &lv_font_montserrat_18, COL_TEXT);
        lv_label_set_text(l, txt[i]);
        lv_obj_center(l);
    }

    // background bands in the battery fill's colours: red < 25 % usable, yellow < 50 %, green above
    const float v25 = BAT_EMPTY_V + 0.25f * (BAT_FULL_V - BAT_EMPTY_V);
    const float v50 = BAT_EMPTY_V + 0.50f * (BAT_FULL_V - BAT_EMPTY_V);
    int y25 = hist_cv_y(v25 * 100), y50 = hist_cv_y(v50 * 100);
    lv_obj_t *band;
    band = make_rect(p, CH_X, CH_Y, CH_W, y50 - CH_Y, DUO_GREEN);
    lv_obj_set_style_bg_opa(band, 26, 0);
    band = make_rect(p, CH_X, y50, CH_W, y25 - y50, DUO_YELLOW);
    lv_obj_set_style_bg_opa(band, 31, 0);
    band = make_rect(p, CH_X, y25, CH_W, CH_Y + CH_H - y25, DUO_RED);
    lv_obj_set_style_bg_opa(band, 31, 0);
    for (lv_obj_t *&r : hist_chg) {
        r = make_rect(p, CH_X, CH_Y, 2, CH_H, DUO_BLUE);
        lv_obj_set_style_bg_opa(r, 26, 0);
        lv_obj_add_flag(r, LV_OBJ_FLAG_HIDDEN);
    }
    const int gv[3] = {14, 13, 12};
    for (int i = 0; i < 3; i++) {
        int y = hist_cv_y(gv[i] * 100);
        make_rect(p, CH_X, y, CH_W, 1, COL_TRACK);
        lv_obj_t *l = make_label(p, &lv_font_montserrat_16, COL_MUTED);
        lv_label_set_text_fmt(l, "%d V", gv[i]);
        lv_obj_set_pos(l, 8, y - 10);
    }
    for (int i = 0; i < 5; i++) {
        lbl_hist_x[i] = make_label(p, &lv_font_montserrat_16, COL_MUTED);
        lv_obj_set_width(lbl_hist_x[i], 80);
        lv_obj_set_style_text_align(lbl_hist_x[i], LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_pos(lbl_hist_x[i], CH_X + i * CH_W / 4 - 40, CH_Y + CH_H + 8);
    }

    hist_chart = lv_chart_create(p);
    lv_obj_remove_style_all(hist_chart);
    lv_obj_set_pos(hist_chart, CH_X, CH_Y);
    lv_obj_set_size(hist_chart, CH_W, CH_H);
    lv_chart_set_type(hist_chart, LV_CHART_TYPE_LINE);
    lv_chart_set_div_line_count(hist_chart, 0, 0);
    lv_chart_set_range(hist_chart, LV_CHART_AXIS_PRIMARY_Y, HIST_CV_MIN, HIST_CV_MAX);
    lv_obj_set_style_line_width(hist_chart, 3, LV_PART_ITEMS);
    lv_obj_set_style_size(hist_chart, 0, LV_PART_INDICATOR);
    lv_obj_clear_flag(hist_chart, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    hist_ser = lv_chart_add_series(hist_chart, COL_TEXT, LV_CHART_AXIS_PRIMARY_Y);

    lbl_hist_empty = make_label(p, &lv_font_montserrat_20, COL_MUTED);
    lv_label_set_text(lbl_hist_empty, "No history yet - it builds up on van power with the clock set");
    lv_obj_align(lbl_hist_empty, LV_ALIGN_TOP_MID, 0, CH_Y + CH_H / 2 - 12);
    lbl_hist_sum = make_label(p, &font_text_18, COL_MUTED);
    lv_label_set_text(lbl_hist_sum, "");
    lv_obj_set_pos(lbl_hist_sum, CH_X, 436);

    page_dots(p, 1);
    refresh_history();
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

    build_frost_banner(p);
    cont_classic = lv_obj_create(p);                 // theme layers: transparent to touch, full page
    lv_obj_remove_style_all(cont_classic);
    lv_obj_set_size(cont_classic, 800, 480);
    lv_obj_clear_flag(cont_classic, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);
    build_duo_layer(p);

    lbl_volt = make_label(cont_classic, &font_num_88, COL_DIM);
    lv_obj_set_width(lbl_volt, 400);
    lv_obj_set_style_text_align(lbl_volt, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(lbl_volt, "-- V");
    lv_obj_set_pos(lbl_volt, 0, 118);

    lbl_pct = make_label(cont_classic, &font_num_88, COL_DIM);
    lv_obj_set_width(lbl_pct, 400);
    lv_obj_set_style_text_align(lbl_pct, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(lbl_pct, "-- %");
    lv_obj_set_pos(lbl_pct, 400, 118);

    build_battery_scale(cont_classic);
    build_water_scale(cont_classic);
    apply_theme();

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

    static const char *const det_icons[4] = {FA_WIND, FA_TINT, FA_ARROWSV, FA_SUN};
    for (int i = 0; i < 4; i++) {
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
    page_dots(p, 2);
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
    if (n.day[0].sunrise_min >= 0 && n.day[0].sunset_min >= 0)
        snprintf(buf, sizeof(buf), "%02d:%02d / %02d:%02d", n.day[0].sunrise_min / 60, n.day[0].sunrise_min % 60,
                 n.day[0].sunset_min / 60, n.day[0].sunset_min % 60);
    else
        snprintf(buf, sizeof(buf), "--");
    lv_label_set_text(lbl_wx_det[3], buf);

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

// ---------- reminders --------------------------------------------------------------
// Three countdowns (user-approved mockup 2026-10-23); interval -/+ in 5-day steps, "Done" restarts the countdown.
// Saved in NVS "globebus": remNd (days), remNt (unix time last done). The countdown starts the first time the
// clock is valid.
struct Reminder {
    const char *name;
    uint16_t    days;
    uint32_t    done;
    lv_obj_t   *lbl_iv;
    lv_obj_t   *lbl_due;
};
static Reminder  g_rem[] = {{"Battery water", 30, 0, nullptr, nullptr},
                            {"Water filter", 90, 0, nullptr, nullptr},
                            {"Gas bottle", 60, 0, nullptr, nullptr}};
static constexpr int REM_N = sizeof(g_rem) / sizeof(g_rem[0]);
static lv_obj_t *rem_strip     = nullptr;
static lv_obj_t *rem_strip_lbl = nullptr;
static int       g_rem_due     = -1;

static void rem_load()
{
    Preferences p;
    p.begin("globebus", true);
    for (int i = 0; i < REM_N; i++) {
        char k[8];
        snprintf(k, sizeof(k), "rem%dd", i);
        g_rem[i].days = p.getUShort(k, g_rem[i].days);
        snprintf(k, sizeof(k), "rem%dt", i);
        g_rem[i].done = p.getULong(k, 0);
    }
    p.end();
}

static void rem_save(int i)
{
    Preferences p;
    p.begin("globebus", false);
    char k[8];
    snprintf(k, sizeof(k), "rem%dd", i);
    p.putUShort(k, g_rem[i].days);
    snprintf(k, sizeof(k), "rem%dt", i);
    p.putULong(k, g_rem[i].done);
    p.end();
}

static long rem_days_left(const Reminder &r)       // <= 0: due (0 today, -n overdue by n days)
{
    long left = (long)r.done + (long)r.days * 86400L - (long)time(nullptr);
    return left > 0 ? (left + 86399) / 86400 : -((-left) / 86400);
}

static void rem_due_text(char *b, size_t n, long d)
{
    if (d > 1)        snprintf(b, n, "in %ld d", d);
    else if (d == 1)  snprintf(b, n, "tomorrow");
    else if (d == 0)  snprintf(b, n, "due today");
    else              snprintf(b, n, "overdue %ld d", -d);
}

static void rem_tick()                           // caller holds the LVGL lock
{
    if (!time_valid()) return;
    g_rem_due = -1;
    for (int i = 0; i < REM_N; i++) {
        Reminder &r = g_rem[i];
        if (!r.done) {                           // first valid clock: start the countdown
            r.done = (uint32_t)time(nullptr);
            rem_save(i);
        }
        long d = rem_days_left(r);
        char b[32];
        if (r.lbl_iv) {
            snprintf(b, sizeof(b), "every %u d", r.days);
            lv_label_set_text(r.lbl_iv, b);
        }
        if (r.lbl_due) {
            rem_due_text(b, sizeof(b), d);
            lv_label_set_text(r.lbl_due, b);
            lv_obj_set_style_text_color(r.lbl_due, d <= 0 ? lv_color_hex(0xF0C565) : COL_MUTED, 0);
        }
        if (d <= 0 && g_rem_due < 0) g_rem_due = i;
    }
    if (!rem_strip) return;
    if (g_rem_due < 0) {
        lv_obj_add_flag(rem_strip, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    char due[24], b[96];
    rem_due_text(due, sizeof(due), rem_days_left(g_rem[g_rem_due]));
    snprintf(b, sizeof(b), "%s \xC2\xB7 %s", g_rem[g_rem_due].name, due);
    lv_label_set_text(rem_strip_lbl, b);
    lv_obj_clear_flag(rem_strip, LV_OBJ_FLAG_HIDDEN);
}

static void rem_done_cb(lv_event_t *e)
{
    intptr_t i = (intptr_t)lv_event_get_user_data(e);
    if (i < 0) i = g_rem_due;                     // the clock page's button: whichever is shown
    if (i < 0 || i >= REM_N) return;
    g_rem[i].done = (uint32_t)time(nullptr);
    rem_save((int)i);
    vlog("REMINDER done: %s (every %u d)", g_rem[i].name, g_rem[i].days);
    rem_tick();
}

static void rem_step_cb(lv_event_t *e)
{
    intptr_t v = (intptr_t)lv_event_get_user_data(e);  // index*2 + (0 = minus, 1 = plus)
    int i = (int)(v / 2);
    int d = g_rem[i].days + ((v & 1) ? 5 : -5);
    g_rem[i].days = (uint16_t)LV_CLAMP(5, d, 365);
    rem_save(i);
    rem_tick();
}

static void build_reminders_tab(lv_obj_t *t)
{
    for (int i = 0; i < REM_N; i++) {
        int y = i * 64;
        lv_obj_t *n = make_label(t, &lv_font_montserrat_22, COL_TEXT);
        lv_label_set_text(n, g_rem[i].name);
        lv_obj_set_pos(n, 0, y + 8);
        lv_obj_t *m = make_button(t, "-", 44, 40, rem_step_cb, (void *)(intptr_t)(i * 2));
        lv_obj_set_pos(m, 200, y);
        g_rem[i].lbl_iv = make_label(t, &lv_font_montserrat_20, COL_MUTED);
        lv_obj_set_width(g_rem[i].lbl_iv, 130);
        lv_obj_set_style_text_align(g_rem[i].lbl_iv, LV_TEXT_ALIGN_CENTER, 0);
        lv_obj_set_pos(g_rem[i].lbl_iv, 250, y + 9);
        lv_obj_t *pl = make_button(t, "+", 44, 40, rem_step_cb, (void *)(intptr_t)(i * 2 + 1));
        lv_obj_set_pos(pl, 386, y);
        g_rem[i].lbl_due = make_label(t, &lv_font_montserrat_20, COL_MUTED);
        lv_obj_set_pos(g_rem[i].lbl_due, 450, y + 9);
        lv_obj_t *d = make_button(t, "Done", 110, 40, rem_done_cb, (void *)(intptr_t)i);
        lv_obj_set_pos(d, 632, y);
    }
    lv_obj_t *note = make_label(t, &lv_font_montserrat_16, COL_DIM);
    lv_label_set_long_mode(note, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(note, 740);
    lv_label_set_text(note, "Change the interval with - / + (5 days a step). Done restarts the countdown. A due reminder "
                            "also shows on the clock page.");
    lv_obj_set_pos(note, 0, 210);
    rem_tick();
}

static void build_clock_page(lv_obj_t *p)
{
    lbl_big_clock = make_label(p, &font_clock_170, COL_TEXT);
    lv_label_set_text(lbl_big_clock, "--:--");
    lv_obj_align(lbl_big_clock, LV_ALIGN_CENTER, 0, -40);

    lbl_big_date = make_label(p, &lv_font_montserrat_36, COL_MUTED);
    lv_label_set_text(lbl_big_date, "");
    lv_obj_align(lbl_big_date, LV_ALIGN_CENTER, 0, 90);

    rem_strip = lv_obj_create(p);
    lv_obj_remove_style_all(rem_strip);
    lv_obj_set_pos(rem_strip, 150, 350);
    lv_obj_set_size(rem_strip, 500, 54);
    lv_obj_set_style_radius(rem_strip, 12, 0);
    lv_obj_set_style_bg_color(rem_strip, COL_BTN, 0);
    lv_obj_set_style_bg_opa(rem_strip, LV_OPA_COVER, 0);
    lv_obj_clear_flag(rem_strip, LV_OBJ_FLAG_SCROLLABLE);
    rem_strip_lbl = make_label(rem_strip, &font_text_18, lv_color_hex(0xF0C565));
    lv_label_set_text(rem_strip_lbl, "");
    lv_obj_align(rem_strip_lbl, LV_ALIGN_LEFT_MID, 22, 0);
    lv_obj_t *done = make_button(rem_strip, "Done", 96, 34, rem_done_cb, (void *)(intptr_t)-1);
    lv_obj_align(done, LV_ALIGN_RIGHT_MID, -10, 0);
    lv_obj_add_flag(rem_strip, LV_OBJ_FLAG_HIDDEN);

    page_dots(p, 3);
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

static void night_dd_cb(lv_event_t *e)
{
    g_night_idx = lv_dropdown_get_selected(dd_night);
    Preferences p;
    p.begin("globebus", false);
    p.putUChar("night", (uint8_t)g_night_idx);
    p.end();
    vlog("SETTING night mode %s", g_night_idx == 0 ? "off" : (g_night_idx == 1 ? "auto" : "always"));
    night_apply(night_wanted());                  // LVGL event: lock already held
}

static lv_obj_t *display_row(lv_obj_t *t, const char *text, int y, const char *opts, int sel, lv_event_cb_t cb)
{
    lv_obj_t *l = make_label(t, &lv_font_montserrat_22, COL_TEXT);
    lv_label_set_text(l, text);
    lv_obj_set_pos(l, 0, y + 10);
    lv_obj_t *dd = lv_dropdown_create(t);
    lv_dropdown_set_options_static(dd, opts);
    lv_dropdown_set_selected(dd, sel);
    lv_obj_set_width(dd, 260);
    lv_obj_set_style_text_font(dd, &lv_font_montserrat_22, 0);
    lv_obj_set_style_text_font(lv_dropdown_get_list(dd), &lv_font_montserrat_22, 0);
    lv_obj_set_pos(dd, 250, y);
    lv_obj_add_event_cb(dd, cb, LV_EVENT_VALUE_CHANGED, nullptr);
    return dd;
}

static void build_display_tab(lv_obj_t *t)
{
    dd_timeout = display_row(t, "Screen off after", 0, TIMEOUT_OPTS, g_timeout_idx, timeout_dd_cb);
    dd_hold    = display_row(t, "Keep reading", 56, HOLD_OPTS, g_hold_idx, hold_dd_cb);
    dd_theme   = display_row(t, "Theme", 112, THEME_OPTS, g_theme_idx, theme_dd_cb);
    dd_night   = display_row(t, "Night mode", 168, NIGHT_OPTS, g_night_idx, night_dd_cb);
    lv_obj_t *bt = make_button(t, "Backlight test (5 s off)", 230, 46, bl_test_cb, nullptr);
    lv_obj_set_pos(bt, 530, 0);
    lv_obj_set_style_text_font(lv_obj_get_child(bt, 0), &lv_font_montserrat_16, 0);

    lv_obj_t *n = make_label(t, &lv_font_montserrat_16, COL_DIM);
    lv_label_set_long_mode(n, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(n, 740);
    lv_label_set_text(n, "Screen off: black screen + backlight off after this long without touch or press; touch or a "
                         "selector wakes it. Keep reading: how long a value stays after you let go. Night mode: dim red "
                         "colours, Auto from sunset to sunrise (forecast; 20:00-07:00 without one).");
    lv_obj_set_pos(n, 0, 230);
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
    lbl_screen = make_label(t, &lv_font_montserrat_14, COL_MUTED);
    lv_label_set_long_mode(lbl_screen, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_screen, 740);
    lv_label_set_text(lbl_screen, "Screen: --");
    lv_obj_set_pos(lbl_screen, 0, 236);

    lbl_scrlog = make_label(t, &lv_font_montserrat_14, COL_DIM);
    lv_label_set_text(lbl_scrlog, "(no screen events yet)");
    lv_obj_set_pos(lbl_scrlog, 0, 258);

    lbl_netst = make_label(t, &lv_font_montserrat_14, COL_MUTED);
    lv_label_set_long_mode(lbl_netst, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_netst, 740);
    lv_label_set_text(lbl_netst, "Network: --");
    lv_obj_set_pos(lbl_netst, 0, 352);
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

    lv_obj_t *tabs[7];
    const char *names[7] = {"Clock", "Wi-Fi", "BT", "Weather", "Display", "Remind", "Diag"};
    for (int i = 0; i < 7; i++) {
        tabs[i] = lv_tabview_add_tab(tv, names[i]);
        lv_obj_set_style_pad_all(tabs[i], 24, 0);
    }
    build_clock_tab(tabs[0]);
    build_wifi_tab(tabs[1]);
    build_ble_tab(tabs[2]);
    build_weather_tab(tabs[3]);
    build_display_tab(tabs[4]);
    build_reminders_tab(tabs[5]);
    build_diag_tab(tabs[6]);
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
    if (n.connected != g_ns.connected) {
        if (n.connected) vlog("WIFI connected to %s (%d dBm)", n.ssid, n.rssi);
        else             vlog("WIFI disconnected");
    }
    if (n.ble_enabled != g_ns.ble_enabled) vlog("BT %s", n.ble_enabled ? "on" : "off");
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

// ---------- phone page --------------------------------------------------------------
// Read-only page on the van Wi-Fi: http://globebus.local (mDNS) or the board's IP (shown in Settings > Diag). No
// password: anyone on the same Wi-Fi can look, nobody can change anything (user decision 2026-10-23).
static WebServer g_web(80);

static const char WEB_PAGE[] PROGMEM = R"HTML(<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>Globebus</title><style>
body{margin:0;background:#141A20;color:#E8EDF1;font-family:sans-serif}h1{font-size:22px;font-weight:500;margin:16px}
#t{float:right;color:#8FA1AD;font-size:16px;margin-top:4px}.row{display:flex;justify-content:space-around;margin:8px 0}
.v{font-size:26px;text-align:center}.s{font-size:13px;color:#8FA1AD;text-align:center}
.card{margin:12px;background:#1C242C;border-radius:12px;padding:8px}.m{margin:12px;font-size:16px}
.n{margin:12px;font-size:13px;color:#8FA1AD}.b{margin:0 12px 8px;padding:8px 12px;border-radius:14px;font-size:15px}
</style></head><body><h1>Globebus<span id="t"></span></h1><div id="ban"></div>
<div class="row"><div><svg id="bb" width="120" height="160"></svg><div class="v" id="bv">--</div><div class="s" id="bs"></div></div>
<div><svg id="wb" width="120" height="160"></svg><div class="v" id="wv">--</div><div class="s" id="ws"></div></div></div>
<div class="card"><div class="s" style="text-align:left">Leisure battery &middot; 24 h</div>
<svg id="hg" viewBox="0 0 300 100" width="100%" height="110" preserveAspectRatio="none"></svg></div>
<div class="m" id="wx"></div><div class="n">Read-only. Refreshes every 30 s.</div><script>
function box(id,f,c,st){var h=Math.round(152*Math.max(0,Math.min(1,f))),y=156-h,s='<defs><clipPath id="c'+id+'"><rect x="4" y="4" width="112" height="152" rx="10"/></clipPath><pattern id="p'+id+'" width="10" height="10" patternUnits="userSpaceOnUse" patternTransform="rotate(45)"><rect width="10" height="10" fill="#2C5AA6"/><rect width="4" height="10" fill="#3D78D8"/></pattern></defs>';
if(h>0)s+='<rect clip-path="url(#c'+id+')" x="4" y="'+y+'" width="112" height="'+h+'" fill="'+(st?'url(#p'+id+')':c)+'"/>';
s+='<rect x="4" y="4" width="112" height="152" rx="10" fill="none" stroke="#fff" stroke-width="4"/>';document.getElementById(id).innerHTML=s}
function $(i){return document.getElementById(i)}
function upd(d){$('t').textContent=d.time||'';
if(d.live){box('bb',d.live.fill,d.live.color,false);$('bv').textContent=d.live.v.toFixed(1)+' V';$('bs').textContent=d.live.charging?'leisure, live, charging':'leisure, live'}
else{box('bb',0,'',false);$('bv').textContent='-- V';$('bs').textContent='no van feed'}
if(d.last){box('wb',d.last.tfill,'#3D78D8',d.last.low);$('wv').textContent=d.last.tank;$('ws').textContent='last reading '+d.last.at}
else{box('wb',0,'',false);$('wv').textContent='-- %';$('ws').textContent='no reading yet'}
$('ban').innerHTML=d.banner?'<div class="b" style="background:'+d.banner.bg+';color:'+d.banner.fg+'">'+d.banner.text+'</div>':'';
var p='',x;for(var i=0;i<d.hist.length;i++){if(d.hist[i]==null)continue;x=i*300/(d.hist.length-1);p+=(p?' L':'M')+x.toFixed(1)+' '+(100-(d.hist[i]-1180)*100/240).toFixed(1)}
$('hg').innerHTML=p?'<path d="'+p+'" fill="none" stroke="#E8EDF1" stroke-width="2"/>':'<text x="150" y="55" fill="#8FA1AD" font-size="11" text-anchor="middle">no history yet</text>';
$('wx').textContent=d.wx?(d.wx.t+'° '+d.wx.cond+' · tonight '+d.wx.tonight+'°'):''}
function go(){fetch('/api').then(function(r){return r.json()}).then(upd).catch(function(){})}go();setInterval(go,30000);
</script></body></html>)HTML";

static void web_api()
{
    String j;
    j.reserve(2400);
    char b[200];
    char hm[8] = "";
    if (time_valid()) {
        time_t now = time(nullptr);
        struct tm t;
        localtime_r(&now, &t);
        snprintf(hm, sizeof(hm), "%02d:%02d", t.tm_hour, t.tm_min);
    }
    snprintf(b, sizeof(b), "{\"time\":\"%s\",", hm);
    j += b;
    float v = live_battery_v();
    if (!isnan(v)) {
        float f = battery_fill(v);
        snprintf(b, sizeof(b), "\"live\":{\"v\":%.2f,\"fill\":%.2f,\"color\":\"%s\",\"charging\":%s},", v, f,
                 f < 0.25f ? "#E5483D" : (f < 0.5f ? "#E0B030" : "#639922"), v >= CHARGING_V ? "true" : "false");
    } else {
        snprintf(b, sizeof(b), "\"live\":null,");
    }
    j += b;
    if (g_last_at) {
        struct tm t;
        localtime_r(&g_last_at, &t);
        snprintf(b, sizeof(b), "\"last\":{\"v\":%.2f,\"tank\":\"%s\",\"tfill\":%.2f,\"low\":%s,\"at\":\"%02d:%02d\"},",
                 g_last_v, g_last_tank, g_last_tfill, g_last_low ? "true" : "false", t.tm_hour, t.tm_min);
    } else {
        snprintf(b, sizeof(b), "\"last\":null,");
    }
    j += b;
    if (g_low_bat && !isnan(v))
        snprintf(b, sizeof(b), "\"banner\":{\"text\":\"Leisure battery low \\u00b7 %.1f V \\u00b7 recharge soon\","
                               "\"bg\":\"#5C1A16\",\"fg\":\"#FFC9C3\"},", v);
    else if (g_frost)
        snprintf(b, sizeof(b), "\"banner\":{\"text\":\"%s\",\"bg\":\"#123A5C\",\"fg\":\"#CFE8FF\"},", g_frost_text);
    else
        snprintf(b, sizeof(b), "\"banner\":null,");
    j += b;
    j += "\"hist\":[";                             // last 24 h in 15-minute buckets, volts x 100
    if (!g_hist_tmp) g_hist_tmp = (HistPoint *)heap_caps_malloc(HIST_CAP * sizeof(HistPoint), MALLOC_CAP_SPIRAM);
    int n = 0;
    time_t now = time(nullptr), from = now - 96 * 900;
    if (time_valid() && g_hist_tmp) n = hist_window(from, now, g_hist_tmp, HIST_CAP);
    for (int k = 0; k < 96; k++) {
        int32_t sum = 0, cnt = 0;
        for (int i = 0; i < n; i++) {
            int bk = (int)(((time_t)g_hist_tmp[i].t - from) / 900);
            if (bk == k) {
                sum += g_hist_tmp[i].cv;
                cnt++;
            }
        }
        if (cnt) snprintf(b, sizeof(b), "%s%ld", k ? "," : "", (long)(sum / cnt));
        else     snprintf(b, sizeof(b), "%snull", k ? "," : "");
        j += b;
    }
    j += "],";
    if (g_ns.wx_valid) {
        time_t now2 = time(nullptr);
        struct tm t;
        localtime_r(&now2, &t);
        snprintf(b, sizeof(b), "\"wx\":{\"t\":%.0f,\"cond\":\"%s\",\"tonight\":%.0f}}", g_ns.temp, wx_text(g_ns.code),
                 g_ns.day[t.tm_hour < 6 ? 0 : 1].tmin);
    } else {
        snprintf(b, sizeof(b), "\"wx\":null}");
    }
    j += b;
    g_web.send(200, "application/json", j);
}

static void web_poll()                           // loop(), at rest
{
    if (!g_web_on) {
        if (!g_ns.connected) return;
        g_web.on("/", HTTP_GET, []() { g_web.send_P(200, "text/html", WEB_PAGE); });
        g_web.on("/api", HTTP_GET, web_api);
        g_web.begin();
        if (MDNS.begin("globebus")) MDNS.addService("http", "tcp", 80);
        g_web_on = true;
        vlog("WEB phone page on http://globebus.local / http://%s", WiFi.localIP().toString().c_str());
    }
    g_web.handleClient();
}

// ---------- persistent log: boot record, periodic sample, USB commands --------------------------------------

static const char *reset_reason_text(esp_reset_reason_t r)
{
    switch (r) {
    case ESP_RST_POWERON:   return "POWER-ON (power was off or applied)";
    case ESP_RST_EXT:       return "EXTERNAL PIN";
    case ESP_RST_SW:        return "SOFTWARE";
    case ESP_RST_PANIC:     return "CRASH (panic)";
    case ESP_RST_INT_WDT:   return "INTERRUPT WATCHDOG";
    case ESP_RST_TASK_WDT:  return "TASK WATCHDOG";
    case ESP_RST_WDT:       return "OTHER WATCHDOG";
    case ESP_RST_DEEPSLEEP: return "DEEP SLEEP";
    case ESP_RST_BROWNOUT:  return "BROWN-OUT (supply dipped too low)";
    case ESP_RST_SDIO:      return "SDIO";
    case ESP_RST_USB:       return "USB (reset or flashing over USB)";
    case ESP_RST_JTAG:      return "JTAG";
    default:                return "UNKNOWN";
    }
}

static void log_boot()
{
    Preferences p;
    p.begin("globebus", false);
    uint32_t boots = p.getULong("boots", 0) + 1;
    p.putULong("boots", boots);
    p.end();
    esp_reset_reason_t r = esp_reset_reason();
    // A dump stays on flash until a newer crash overwrites it, so say whether it belongs to this firmware.
    char dump[140] = "none";
    if (esp_core_dump_image_check() == ESP_OK) {
        static esp_core_dump_summary_t sum;
        if (esp_core_dump_get_summary(&sum) == ESP_OK) {
            char mine[10];
            esp_app_get_elf_sha256(mine, sizeof(mine));
            bool same = strncmp((const char *)sum.app_elf_sha256, mine, sizeof(mine) - 1) == 0;
            snprintf(dump, sizeof(dump), "%s firmware (%.9s), task %s, cause %lu at PC 0x%08lx, address 0x%08lx",
                     same ? "THIS" : "an older", (const char *)sum.app_elf_sha256, sum.exc_task,
                     (unsigned long)sum.ex_info.exc_cause, (unsigned long)sum.exc_pc,
                     (unsigned long)sum.ex_info.exc_vaddr);
        } else {
            snprintf(dump, sizeof(dump), "present (unreadable)");
        }
    }
    vlog("BOOT #%lu reset: %s (%d); firmware built " __DATE__ " " __TIME__ "; crash dump on flash: %s",
         (unsigned long)boots, reset_reason_text(r), (int)r, dump);
}

// Once a minute (and never during a press): how the board and the Toptron input look at rest.
static void log_sample()
{
    float a0 = NAN, a1 = NAN;
    ads_read_volts(MUX_AIN0, PGA_4V096, &a0);
    ads_read_volts(MUX_AIN1, PGA_4V096, &a1);
    // chip temperature: not the cabin's, but it follows it - to see whether the open-A0 fault (2026-10) comes and
    // goes with warming and cooling (a cracked joint) rather than with use
    vlog("SAMPLE supply %.2f V, chip %.1f C, gnd A0 %.1f A1 %.1f mV, idle A0-A1 %.3f mV, i2c errors %lu, stuck %lu, "
         "heap %lu/%lu B, wifi %s %d dBm, screen %s",
         g_supply_v, temperatureRead(), a0 * 1000.0f, a1 * 1000.0f, g_idle_mv, (unsigned long)g_read_errors, (unsigned long)g_stuck,
         (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL), (unsigned long)ESP.getFreeHeap(),
         g_ns.connected ? "on" : "off", g_ns.connected ? g_ns.rssi : 0, g_screen_off ? "dark" : "on");
}

// Once a second at rest. One conversion per call, smoothed, since the divider has no capacitor.
static void supply_update()
{
    float v;
    if (!ads_read_volts(MUX_AIN2, PGA_2V048, &v)) return;
    float s = supply_from_a2(v);
    g_supply_raw = s;
    if (s < SUPPLY_MIN_V) {
        g_supply_v = NAN;
        return;
    }
    g_supply_v = isnan(g_supply_v) ? s : g_supply_v + 0.3f * (s - g_supply_v);
}

static void wire_check()
{
    static int      bad = 0, good = 0;
    static uint32_t since_ms = 0;
    static float    max_mv = 0;
    float a0, a1;
    if (!ads_read_volts(MUX_AIN0, PGA_4V096, &a0) || !ads_read_volts(MUX_AIN1, PGA_4V096, &a1)) return;
    float d = fabsf(a0 - a1) * 1000.0f;
    if (!g_wire_loose) {
        bad = d > WIRE_LOOSE_MV ? bad + 1 : 0;
        if (bad < 2) return;
        g_wire_loose = true;
        g_wire_events++;
        since_ms = millis();
        max_mv = d;
        good = 0;
        vlog("WIRE signal input looks OPEN: A0 %.0f mV, A1 %.0f mV vs board GND (apart %.0f mV), chip %.1f C - "
             "presses ignored", a0 * 1000.0f, a1 * 1000.0f, d, temperatureRead());
        set_state_text("Signal wire loose? Rockers ignored");
        return;
    }
    if (d > max_mv) max_mv = d;
    good = d < WIRE_OK_MV ? good + 1 : 0;
    if (good < WIRE_OK_CHECKS) {
        set_state_text("Signal wire loose? Rockers ignored");   // keep it up; other code may have replaced it
        return;
    }
    g_wire_loose = false;
    bad = 0;
    vlog("WIRE signal input connected again after %lu s (A0/A1 were up to %.0f mV apart), chip %.1f C",
         (unsigned long)((millis() - since_ms) / 1000), max_mv, temperatureRead());
    set_state_text(g_display_ms ? "Held reading" : "Hold a selector on the panel");
}

// Serial commands (115200 baud): log, loginfo, logflush, logclear, help
static void handle_serial()
{
    static char cmd[24];
    static size_t n = 0;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c != '\n' && c != '\r') {
            if (n < sizeof(cmd) - 1) cmd[n++] = c;
            continue;
        }
        if (!n) continue;
        cmd[n] = 0;
        n = 0;
        if (!strcmp(cmd, "log"))           vlog_dump(Serial);
        else if (!strcmp(cmd, "loginfo"))  vlog_info(Serial);
        else if (!strcmp(cmd, "logflush")) { vlog_flush(); vlog_info(Serial); }
        else if (!strcmp(cmd, "logclear")) { vlog_clear(); Serial.println("[log] cleared"); }
        else if (!strcmp(cmd, "logtest"))  vlog_flash_test(Serial, 32 * 1024);
        else Serial.println("commands: log (print the stored log), loginfo, logflush (write pending now), logclear, "
                            "logtest (timed 32 KB scratch write, to check the display)");
    }
}

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
    vlog_begin();
    hist_begin();
    rem_load();
    log_boot();

    {
        Preferences p;
        p.begin("globebus", true);
        int idx = p.getUChar("disp_to", TIMEOUT_DEFAULT);
        int hidx = p.getUChar("hold", HOLD_DEFAULT);
        int tidx = p.getUChar("theme", 0);
        g_theme_idx = (tidx == 1) ? 1 : 0;
        int nidx = p.getUChar("night", 1);
        g_night_idx = (nidx >= 0 && nidx <= 2) ? nidx : 1;
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
    build_history_page(lv_tileview_add_tile(tv, 1, 0, LV_DIR_LEFT | LV_DIR_RIGHT));
    build_weather_page(lv_tileview_add_tile(tv, 2, 0, LV_DIR_LEFT | LV_DIR_RIGHT));
    build_clock_page(lv_tileview_add_tile(tv, 3, 0, LV_DIR_LEFT));
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
        if (millis() - g_press.start_ms > MAX_PRESS_MS) {
            // Not a rocker: a standing signal. End it so the panel does not freeze on one value (and so the rest
            // of loop() runs again); a new press needs the input to go quiet first (g_idle_ok was reset).
            g_stuck++;
            screen_log("input stuck %.2f mV >%lus, ignored (%lu)", mv,
                       (unsigned long)(MAX_PRESS_MS / 1000), (unsigned long)g_stuck);
            press_end();
            return;
        }
        if (mv < PRESS_THRESHOLD_MV) {
            if (++g_press.below >= RELEASE_SAMPLES) press_end();
            return;
        }
        g_press.below = 0;
        press_sample(mv);
        return;   // sample again immediately
    }

    handle_serial();
    vlog_poll(g_screen_off, false);
    {
        static uint32_t last_sample_ms = 0;
        if (millis() - last_sample_ms >= 60000UL || !last_sample_ms) {
            last_sample_ms = millis();
            if (last_sample_ms > BOOT_IGNORE_MS) log_sample();
            else last_sample_ms = 0;
        }
        static uint32_t last_live_ms = 0;            // Duo idle: live leisure battery every 2 s
        if (millis() - last_live_ms >= 2000) {
            last_live_ms = millis();
            static uint32_t low_since = 0;           // low-battery banner: red band for 2 min, clears in green
            float lv = live_battery_v();
            const float v25 = BAT_EMPTY_V + 0.25f * (BAT_FULL_V - BAT_EMPTY_V);
            const float v50 = BAT_EMPTY_V + 0.50f * (BAT_FULL_V - BAT_EMPTY_V);
            bool was_low = g_low_bat;
            if (isnan(lv)) {
                low_since = 0;
                g_low_bat = false;
            } else if (lv < v25) {
                if (!low_since) low_since = millis();
                if (millis() - low_since >= 120000UL) g_low_bat = true;
            } else {
                low_since = 0;
                if (lv >= v50) g_low_bat = false;
            }
            lvgl_port_lock(-1);
            if (g_low_bat) {
                static int last_dv = -1;
                int dv = (int)lroundf(lv * 10);
                if (dv != last_dv) {
                    char b[80];
                    snprintf(b, sizeof(b), "Leisure battery low \xC2\xB7 %.1f V \xC2\xB7 recharge soon", lv);
                    lv_label_set_text(lbl_low, b);
                    lv_obj_center(lbl_low);
                    last_dv = dv;
                }
            }
            if (was_low != g_low_bat) {
                vlog("LOW battery banner %s (live %.2f V)", g_low_bat ? "on" : "off", lv);
                update_banners();
            }
            lvgl_port_unlock();
            if (!g_display_ms && !g_screen_off && g_theme_idx == 1) {
                lvgl_port_lock(-1);
                duo_show_live();
                lvgl_port_unlock();
            }
        }
        static time_t last_hist_slot = 0;            // battery history: one point per 5-minute slot
        if (time_valid() && !isnan(g_supply_v)) {
            time_t slot = time(nullptr) / HIST_STEP_S;
            if (slot != last_hist_slot) {
                last_hist_slot = slot;
                float v = live_battery_v();
                hist_add(time(nullptr), v, v >= CHARGING_V);
                lvgl_port_lock(-1);
                refresh_history();
                lvgl_port_unlock();
            }
        }
        hist_save_if_due();
        web_poll();
        static uint32_t last_supply_ms = 0;
        if (millis() - last_supply_ms >= 1000) {
            last_supply_ms = millis();
            supply_update();
        }
        static uint32_t last_wire_ms = 0;
        if (millis() > BOOT_IGNORE_MS && millis() - last_wire_ms >= WIRE_CHECK_MS) {
            last_wire_ms = millis();
            wire_check();
        }
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
        vlog("TIME set from the internet");
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
            g_idle_mv = v * 1000.0f;
            if (g_idle_ok < IDLE_CONFIRM_SAMPLES) g_idle_ok++;
        } else if (now < BOOT_IGNORE_MS) {
            Serial.printf("[press] ignored during boot settle: %.2f mV\n", v * 1000.0f);
        } else if (g_wire_loose) {
            // an open A0 produces random "readings"; never show them
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

    if (g_bl_test_end) {                          // manual backlight test from Settings > Display
        static bool test_off = false;
        if (!test_off) {
            test_off = true;
            backlight_set(false);
        } else if ((int32_t)(now - g_bl_test_end) >= 0) {
            backlight_set(true);
            test_off = false;
            g_bl_test_end = 0;
        }
    }
    if (now - last_screen_ms >= SCREEN_CHECK_MS) {
        last_screen_ms = now;
        check_screen_timeout();
        backlight_reassert();
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
