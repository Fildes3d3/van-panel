/*
 * Dethleffs Globebus control panel - STAGE 2 firmware: I2C scan
 * Board : Waveshare ESP32-S3-Touch-LCD-4.3B (M-053)
 *
 * Purpose: list every device that ACKs on the board's internal I2C bus.
 * Nothing external is connected. No ADS1115, no Toptron.
 *
 * How the bus is accessed:
 *   ESP32_Display_Panel installs the ESP-IDF *legacy* I2C driver on port 0
 *   (GPIO8 SDA / GPIO9 SCL, 400 kHz) for the GT911 touch and the CH422G expander
 *   (src/drivers/host/esp_panel_host_i2c.cpp). We probe through that same
 *   installed driver with i2c_master_cmd_begin(), which serialises against the
 *   touch polling done by the LVGL task. We never call Wire.begin() and never
 *   install a second driver on these pins (ESP32_Display_Panel issue #243).
 *
 * Probe = START, address byte (write), STOP. No data byte is sent, so no
 * register is written. That matters for the CH422G, whose "addresses" are
 * commands: a command with no data byte changes nothing.
 *
 * Expected (CLAUDE.md, Stage 2): CH422G somewhere in 0x20-0x27 / 0x30-0x3F,
 * PCF85063 RTC at 0x51, GT911 touch at 0x5D (0x14 is its alternate address).
 *
 * Screen shows:
 *   - board profile, chip / flash / PSRAM, uptime, free memory (as Stage 1)
 *   - scan result list + PASS/CHECK verdict
 *   - RESCAN button (the scan runs in loop(), not in the LVGL task)
 *   - CONTENTION TEST: loop() also scans every SCAN_PERIOD_MS. Drag a finger on the
 *     screen (away from the button) so GT911 reads compete for the bus. Each scan logs
 *     how many touch reads happened while it ran; totals every STATS_EVERY scans.
 *   - corner markers and touch readout kept from Stage 1
 */

#include <Arduino.h>
#include <esp_display_panel.hpp>
#include <lvgl.h>
#include <driver/i2c.h>
#include <esp_timer.h>
#include "lvgl_v8_port.h"

using namespace esp_panel::drivers;
using namespace esp_panel::board;

// Port the display library installed. Must match the board profile.
static constexpr i2c_port_t SCAN_PORT = I2C_NUM_0;
#ifdef ESP_PANEL_BOARD_TOUCH_I2C_HOST_ID
static_assert(ESP_PANEL_BOARD_TOUCH_I2C_HOST_ID == 0, "touch I2C host is not port 0");
#endif
#ifdef ESP_PANEL_BOARD_EXPANDER_I2C_HOST_ID
static_assert(ESP_PANEL_BOARD_EXPANDER_I2C_HOST_ID == 0, "expander I2C host is not port 0");
#endif

static constexpr uint8_t ADDR_FIRST = 0x08;   // 0x00-0x07 and 0x78-0x7F are reserved
static constexpr uint8_t ADDR_LAST  = 0x77;

// Max wait per probe, covering both acquiring the driver lock and the transfer.
// Must exceed the longest time loop() can be starved by the higher-priority LVGL task
// (first probe after boot measured at ~79 ms while the first frame renders). With 50 ms,
// ~1 in 10 boots returned ESP_ERR_TIMEOUT on the first probe: the wait had already expired
// when loop() resumed, so the call failed if the driver lock was not free at that instant
// (which task held it is not verified). A genuinely dead bus still times out here.
static constexpr uint32_t PROBE_TIMEOUT_MS = 1000;

static constexpr uint32_t SCAN_PERIOD_MS = 200;   // contention test: continuous scanning
static constexpr uint32_t STATS_EVERY    = 25;    // cumulative summary interval (scans)

static const char *g_board_name = "?";
static lv_obj_t *lbl_uptime   = nullptr;
static lv_obj_t *lbl_mem      = nullptr;
static lv_obj_t *lbl_touch    = nullptr;
static lv_obj_t *lbl_scan_hdr = nullptr;
static lv_obj_t *lbl_scan     = nullptr;
static lv_obj_t *lbl_verdict  = nullptr;
static lv_obj_t *lbl_stats    = nullptr;
static lv_obj_t *touch_dot    = nullptr;
static uint32_t  press_count  = 0;

static volatile bool g_rescan_requested = false;
static uint32_t      g_scan_count       = 0;

// Incremented on every LVGL indev read that reports a press on the screen object.
// Touch is interrupt-driven, so each of these corresponds to one GT911 I2C read.
// (Presses on the RESCAN button go to the button and are not counted.)
static volatile uint32_t g_touch_reads = 0;

struct Stats {
    uint32_t scans              = 0;
    uint32_t bus_errors         = 0;
    uint32_t wrong_count        = 0;   // scans that did not find exactly 26 devices
    uint32_t overlap_scans      = 0;   // scans with >=1 touch read while they ran
    uint32_t overlap_reads      = 0;
    uint32_t slowest_us_idle    = 0;   // excluding scan #1 (boot starvation)
    uint32_t slowest_us_overlap = 0;
};
static Stats g_stats;

// ---------- I2C scan ---------------------------------------------------------

struct ScanResult {
    bool     ack[128]       = {};
    int      n_found        = 0;
    int      n_bus_err      = 0;     // anything other than ACK / NACK
    esp_err_t first_err     = ESP_OK;
    uint8_t  first_err_addr = 0;
    uint32_t duration_ms    = 0;
    uint32_t slowest_us     = 0;     // slowest single probe (lock wait + transfer)
    uint8_t  slowest_addr   = 0;
};

static const char *describe(uint8_t a)
{
    if ((a >= 0x20 && a <= 0x27) || (a >= 0x30 && a <= 0x3F)) return "CH422G IO expander";
    if (a == 0x51) return "PCF85063 RTC";
    if (a == 0x5D) return "GT911 touch";
    if (a == 0x14) return "GT911 touch (alt addr)";
    return "UNEXPECTED";
}

static bool is_expected(uint8_t a)
{
    return strcmp(describe(a), "UNEXPECTED") != 0;
}

// Returns ESP_OK on ACK, ESP_FAIL on NACK, other codes on bus/driver errors.
static esp_err_t probe(uint8_t addr)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (!cmd) return ESP_ERR_NO_MEM;
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t err = i2c_master_cmd_begin(SCAN_PORT, cmd, pdMS_TO_TICKS(PROBE_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    return err;
}

static ScanResult run_scan()
{
    ScanResult r;
    uint32_t t0 = millis();
    for (uint8_t a = ADDR_FIRST; a <= ADDR_LAST; a++) {
        int64_t p0 = esp_timer_get_time();
        esp_err_t err = probe(a);
        uint32_t us = (uint32_t)(esp_timer_get_time() - p0);
        if (us > r.slowest_us) {
            r.slowest_us = us;
            r.slowest_addr = a;
        }
        if (err == ESP_OK) {
            r.ack[a] = true;
            r.n_found++;
        } else if (err != ESP_FAIL) {
            if (r.n_bus_err == 0) {
                r.first_err = err;
                r.first_err_addr = a;
            }
            r.n_bus_err++;
        }
    }
    r.duration_ms = millis() - t0;
    return r;
}

static void report_scan(const ScanResult &r, uint32_t touch_during, bool verbose)
{
    g_scan_count++;

    g_stats.scans++;
    g_stats.bus_errors += r.n_bus_err;
    if (r.n_found != 26) g_stats.wrong_count++;
    if (touch_during) {
        g_stats.overlap_scans++;
        g_stats.overlap_reads += touch_during;
        if (r.slowest_us > g_stats.slowest_us_overlap) g_stats.slowest_us_overlap = r.slowest_us;
    } else if (g_scan_count > 1 && r.slowest_us > g_stats.slowest_us_idle) {
        g_stats.slowest_us_idle = r.slowest_us;
    }

    bool has_exp = false, has_rtc = r.ack[0x51], has_touch = r.ack[0x5D] || r.ack[0x14];
    int n_unexpected = 0;
    for (int a = ADDR_FIRST; a <= ADDR_LAST; a++) {
        if (!r.ack[a]) continue;
        if (strcmp(describe(a), "CH422G IO expander") == 0) has_exp = true;
        if (!is_expected(a)) n_unexpected++;
    }
    bool pass = has_exp && has_rtc && has_touch && n_unexpected == 0 && r.n_bus_err == 0;

    // Anything abnormal always gets the full report
    if (r.n_bus_err || r.n_found != 26 || n_unexpected) verbose = true;

    if (!verbose) {
        Serial.printf("[i2c] scan #%lu: %d found, %d err, slowest 0x%02X %lu us, touch reads during scan %lu\n",
                      (unsigned long)g_scan_count, r.n_found, r.n_bus_err, r.slowest_addr,
                      (unsigned long)r.slowest_us, (unsigned long)touch_during);
    } else {
    // --- serial: full detail ---
    Serial.printf("\n[i2c] scan #%lu: port %d, 0x%02X-0x%02X, %d device(s), %d bus error(s), %lu ms\n",
                  (unsigned long)g_scan_count, (int)SCAN_PORT, ADDR_FIRST, ADDR_LAST,
                  r.n_found, r.n_bus_err, (unsigned long)r.duration_ms);
    for (int a = ADDR_FIRST; a <= ADDR_LAST; a++) {
        if (r.ack[a]) Serial.printf("[i2c]   0x%02X  %s\n", a, describe(a));
    }
    Serial.printf("[i2c]   slowest probe: 0x%02X %lu us\n", r.slowest_addr, (unsigned long)r.slowest_us);
    if (r.n_bus_err) {
        Serial.printf("[i2c]   first bus error at 0x%02X: %s\n", r.first_err_addr, esp_err_to_name(r.first_err));
    }
    Serial.printf("[i2c]   expander:%s rtc:%s touch:%s unexpected:%d -> %s\n",
                  has_exp ? "yes" : "NO", has_rtc ? "yes" : "NO", has_touch ? "yes" : "NO",
                  n_unexpected, pass ? "PASS" : "CHECK");
    Serial.printf("[i2c]   touch reads during scan: %lu\n", (unsigned long)touch_during);
    }

    if (g_stats.scans % STATS_EVERY == 0) {
        Serial.printf("[stats] scans %lu | bus errors %lu | wrong device count %lu | "
                      "touch-overlap scans %lu (%lu reads) | slowest probe idle %lu us, during touch %lu us\n",
                      (unsigned long)g_stats.scans, (unsigned long)g_stats.bus_errors,
                      (unsigned long)g_stats.wrong_count, (unsigned long)g_stats.overlap_scans,
                      (unsigned long)g_stats.overlap_reads, (unsigned long)g_stats.slowest_us_idle,
                      (unsigned long)g_stats.slowest_us_overlap);
    }

    // --- screen: compact ---
    static char list[400];
    size_t len = 0;
    list[0] = '\0';
    for (int a = ADDR_FIRST; a <= ADDR_LAST && len < sizeof(list) - 8; a++) {
        if (r.ack[a]) len += snprintf(list + len, sizeof(list) - len, "0x%02X%s  ", a, is_expected(a) ? "" : "(!)");
    }
    if (r.n_found == 0) snprintf(list, sizeof(list), "(no device answered)");

    lvgl_port_lock(-1);
    lv_label_set_text_fmt(lbl_scan_hdr, "I2C scan #%lu  (port %d, SDA 8 / SCL 9):  %d found, %d bus errors, %lu ms",
                          (unsigned long)g_scan_count, (int)SCAN_PORT, r.n_found, r.n_bus_err,
                          (unsigned long)r.duration_ms);
    lv_label_set_text(lbl_scan, list);
    lv_label_set_text_fmt(lbl_verdict, "%s   expander %s   RTC 0x51 %s   touch %s   unexpected %d",
                          pass ? "PASS" : "CHECK",
                          has_exp ? "ok" : "MISSING", has_rtc ? "ok" : "MISSING",
                          has_touch ? "ok" : "MISSING", n_unexpected);
    lv_obj_set_style_text_color(lbl_verdict, pass ? lv_color_hex(0x66BB6A) : lv_color_hex(0xFF5252), 0);
    lv_label_set_text_fmt(lbl_stats, "Totals: %lu scans, %lu bus errors, %lu touch-overlap scans, "
                          "slowest idle %lu us / touch %lu us",
                          (unsigned long)g_stats.scans, (unsigned long)g_stats.bus_errors,
                          (unsigned long)g_stats.overlap_scans, (unsigned long)g_stats.slowest_us_idle,
                          (unsigned long)g_stats.slowest_us_overlap);
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

    g_touch_reads++;

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
    lv_label_set_text(lbl_scan_hdr, "I2C scan: running...");
    g_rescan_requested = true;   // picked up by loop(); do not scan while holding the LVGL lock
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
    lv_label_set_text(title, "DETHLEFFS PANEL  -  STAGE 2  I2C SCAN");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 30);

    const lv_color_t grey = lv_color_hex(0xB0BEC5);

    lv_obj_t *lbl_board = make_label(scr, &lv_font_montserrat_20, lv_color_hex(0x80DEEA));
    lv_label_set_text_fmt(lbl_board, "Board profile: %s", g_board_name);
    lv_obj_align(lbl_board, LV_ALIGN_TOP_LEFT, 40, 78);

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
    lv_obj_align(lbl_chip, LV_ALIGN_TOP_LEFT, 40, 108);

    // Same sanity check as Stage 1 (XIP copy makes 8 MB PSRAM report ~7.4 MB; threshold is 4 MB)
    if (ESP.getPsramSize() < 4 * 1024 * 1024 || ESP.getFlashChipSize() < 16 * 1024 * 1024) {
        lv_obj_t *warn = make_label(scr, &lv_font_montserrat_16, lv_color_hex(0xFF5252));
        lv_label_set_text(warn, "WARNING: PSRAM/flash size does not match N16R8 - check board JSON");
        lv_obj_align(warn, LV_ALIGN_TOP_LEFT, 40, 172);
    }

    lbl_uptime = make_label(scr, &lv_font_montserrat_16, lv_color_white());
    lv_obj_align(lbl_uptime, LV_ALIGN_TOP_LEFT, 40, 196);

    lbl_mem = make_label(scr, &lv_font_montserrat_16, grey);
    lv_obj_align(lbl_mem, LV_ALIGN_TOP_LEFT, 240, 196);

    lbl_scan_hdr = make_label(scr, &lv_font_montserrat_16, lv_color_hex(0x80DEEA));
    lv_label_set_text(lbl_scan_hdr, "I2C scan: waiting...");
    lv_obj_align(lbl_scan_hdr, LV_ALIGN_TOP_LEFT, 40, 232);

    lbl_scan = make_label(scr, &lv_font_montserrat_20, lv_color_white());
    lv_label_set_long_mode(lbl_scan, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_scan, 720);
    lv_label_set_text(lbl_scan, "");
    lv_obj_align(lbl_scan, LV_ALIGN_TOP_LEFT, 40, 260);

    lbl_verdict = make_label(scr, &lv_font_montserrat_20, grey);
    lv_label_set_text(lbl_verdict, "");
    lv_obj_align(lbl_verdict, LV_ALIGN_TOP_LEFT, 40, 340);

    lbl_stats = make_label(scr, &lv_font_montserrat_16, lv_color_hex(0xB0BEC5));
    lv_label_set_long_mode(lbl_stats, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(lbl_stats, 720);
    lv_label_set_text(lbl_stats, "");
    lv_obj_align(lbl_stats, LV_ALIGN_TOP_LEFT, 40, 372);

    lbl_touch = make_label(scr, &lv_font_montserrat_16, lv_color_hex(0xFFD54F));
    lv_label_set_text(lbl_touch, "Touch: (touch anywhere)");
    lv_obj_align(lbl_touch, LV_ALIGN_BOTTOM_LEFT, 40, -40);

    lv_obj_t *btn = lv_btn_create(scr);
    lv_obj_set_size(btn, 200, 64);
    lv_obj_align(btn, LV_ALIGN_BOTTOM_RIGHT, -50, -40);
    lv_obj_add_event_cb(btn, rescan_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_t *btn_txt = make_label(btn, &lv_font_montserrat_20, lv_color_white());
    lv_label_set_text(btn_txt, "RESCAN");
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
    Serial.println("\n=== Dethleffs panel STAGE 2 (I2C scan) ===");
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

    Serial.println("Setup done. Scanning every 200 ms (contention test); RESCAN forces a full report.");
    g_rescan_requested = true;
}

void loop()
{
    static uint32_t last_scan_ms = 0;
    bool manual = g_rescan_requested;
    if (manual || millis() - last_scan_ms >= SCAN_PERIOD_MS) {
        g_rescan_requested = false;
        last_scan_ms = millis();
        uint32_t t0 = g_touch_reads;
        ScanResult r = run_scan();   // loop task; LVGL task reads GT911 concurrently while touched
        uint32_t touch_during = g_touch_reads - t0;
        report_scan(r, touch_during, manual || g_scan_count == 0);
    }
    delay(5);
}
