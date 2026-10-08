#include "vlog.h"

#include <LittleFS.h>
#include <esp_heap_caps.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <time.h>

static constexpr const char *PART_LABEL     = "spiffs";        // the partition table's unused data partition
static constexpr const char *FILE_CUR       = "/log.txt";
static constexpr const char *FILE_OLD       = "/log.1.txt";
static constexpr size_t      MAX_FILE_BYTES = 1024 * 1024;     // rotate at 1 MB: at most ~2 MB on flash
static constexpr size_t      PENDING_BYTES  = 32 * 1024;       // RAM buffer between flushes (~400 lines)
static constexpr size_t      FLUSH_AT_BYTES = 24 * 1024;       // flush early if this full (unless busy)
static constexpr uint32_t    FLUSH_DARK_MS  = 60UL * 1000;     // screen dark: write every minute
static constexpr uint32_t    FLUSH_LIT_MS   = 60UL * 1000;     // screen on: same - no visible effect (tested)

static bool              s_mounted   = false;
static char             *s_pending   = nullptr;   // lines not yet on flash
static size_t            s_len       = 0;
static char             *s_out       = nullptr;   // copy being written, so vlog() never waits for the flash
static SemaphoreHandle_t s_lock      = nullptr;
static uint32_t          s_dropped   = 0;         // lines lost because the RAM buffer was full
static uint32_t          s_flushes   = 0;
static uint32_t          s_flush_ms  = 0;         // millis() of the last flush
static uint32_t          s_flush_dur = 0;         // duration of the last flush, ms
static bool              s_was_dark  = false;

void vlog_begin()
{
    if (!s_lock) s_lock = xSemaphoreCreateMutex();
    if (!s_pending) s_pending = (char *)heap_caps_malloc(PENDING_BYTES, MALLOC_CAP_SPIRAM);
    if (!s_out) s_out = (char *)heap_caps_malloc(PENDING_BYTES, MALLOC_CAP_SPIRAM);
    // formatOnFail: the partition has never been used, so the first mount formats it (once)
    s_mounted = LittleFS.begin(true, "/littlefs", 4, PART_LABEL);
    Serial.printf("[log] %s, %u of %u bytes used\n", s_mounted ? "mounted" : "MOUNT FAILED",
                  s_mounted ? (unsigned)LittleFS.usedBytes() : 0, s_mounted ? (unsigned)LittleFS.totalBytes() : 0);
    s_flush_ms = millis();
}

void vlog(const char *fmt, ...)
{
    char text[256];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(text, sizeof(text), fmt, ap);
    va_end(ap);

    char stamp[32];
    time_t now = time(nullptr);
    struct tm t;
    if (now > 1700000000 && localtime_r(&now, &t)) strftime(stamp, sizeof(stamp), "%Y-%m-%d %H:%M:%S", &t);
    else snprintf(stamp, sizeof(stamp), "----------  --:--:--");   // clock not set yet (no RTC coin cell)

    char line[320];
    uint32_t ms = millis();
    int n = snprintf(line, sizeof(line), "%s +%lu.%01lus %s\n", stamp, (unsigned long)(ms / 1000),
                     (unsigned long)((ms % 1000) / 100), text);
    if (n <= 0) return;
    if (n >= (int)sizeof(line)) n = sizeof(line) - 1;

    Serial.print("[log] ");
    Serial.write((const uint8_t *)line, n);

    if (!s_lock || !s_pending) return;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    if (s_len + n <= PENDING_BYTES) {
        memcpy(s_pending + s_len, line, n);
        s_len += n;
    } else {
        s_dropped++;
    }
    xSemaphoreGive(s_lock);
}

bool vlog_flush()
{
    if (!s_mounted || !s_lock) return false;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    size_t len = s_len;
    if (len) memcpy(s_out, s_pending, len);
    s_len = 0;
    uint32_t dropped = s_dropped;
    s_dropped = 0;
    xSemaphoreGive(s_lock);
    if (!len && !dropped) return true;

    uint32_t t0 = millis();
    File f = LittleFS.open(FILE_CUR, "a");
    if (f && f.size() + len > MAX_FILE_BYTES) {
        f.close();
        LittleFS.remove(FILE_OLD);
        LittleFS.rename(FILE_CUR, FILE_OLD);
        f = LittleFS.open(FILE_CUR, "a");
    }
    bool ok = (bool)f;
    if (ok) {
        if (dropped) f.printf("(%lu log lines lost: RAM buffer was full)\n", (unsigned long)dropped);
        ok = f.write((const uint8_t *)s_out, len) == len;
        f.close();
    }
    s_flushes++;
    s_flush_ms = millis();
    s_flush_dur = s_flush_ms - t0;
    if (!ok) Serial.println("[log] flash write FAILED");
    return ok;
}

void vlog_poll(bool screen_dark, bool busy)
{
    if (busy || !s_mounted) return;               // never stall the sampling of a held rocker
    uint32_t now = millis();
    bool went_dark = screen_dark && !s_was_dark;
    s_was_dark = screen_dark;
    size_t len;
    xSemaphoreTake(s_lock, portMAX_DELAY);
    len = s_len;
    xSemaphoreGive(s_lock);
    if (!len && !s_dropped) return;
    uint32_t every = screen_dark ? FLUSH_DARK_MS : FLUSH_LIT_MS;
    if (went_dark || len >= FLUSH_AT_BYTES || now - s_flush_ms >= every) vlog_flush();
}

static void dump_file(Stream &out, const char *path)
{
    if (!LittleFS.exists(path)) return;
    File f = LittleFS.open(path, "r");
    if (!f) return;
    uint8_t buf[512];
    while (f.available()) {
        size_t n = f.read(buf, sizeof(buf));
        if (!n) break;
        out.write(buf, n);
    }
    f.close();
}

void vlog_dump(Stream &out)
{
    out.println("=== LOG BEGIN ===");
    if (s_mounted) {
        dump_file(out, FILE_OLD);
        dump_file(out, FILE_CUR);
    }
    if (s_lock) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        if (s_len) {
            out.println("--- not yet on flash ---");
            out.write((const uint8_t *)s_pending, s_len);
        }
        xSemaphoreGive(s_lock);
    }
    out.println("=== LOG END ===");
}

void vlog_clear()
{
    if (s_mounted) {
        LittleFS.remove(FILE_OLD);
        LittleFS.remove(FILE_CUR);
    }
    if (s_lock) {
        xSemaphoreTake(s_lock, portMAX_DELAY);
        s_len = 0;
        s_dropped = 0;
        xSemaphoreGive(s_lock);
    }
}

void vlog_info(Stream &out)
{
    size_t cur = 0, old = 0;
    if (s_mounted) {
        File f;
        if (LittleFS.exists(FILE_CUR) && (f = LittleFS.open(FILE_CUR, "r"))) { cur = f.size(); f.close(); }
        if (LittleFS.exists(FILE_OLD) && (f = LittleFS.open(FILE_OLD, "r"))) { old = f.size(); f.close(); }
    }
    out.printf("[log] %s; files %u + %u B; flash %u / %u B used; pending %u B; flushes %lu "
               "(last %lu s ago, took %lu ms); dropped %lu\n",
               s_mounted ? "mounted" : "NOT mounted", (unsigned)cur, (unsigned)old,
               s_mounted ? (unsigned)LittleFS.usedBytes() : 0, s_mounted ? (unsigned)LittleFS.totalBytes() : 0,
               (unsigned)s_len, (unsigned long)s_flushes, (unsigned long)((millis() - s_flush_ms) / 1000),
               (unsigned long)s_flush_dur, (unsigned long)s_dropped);
}

// Worst case for the display: a large write (block erases included). Uses a scratch file, not the log.
void vlog_flash_test(Stream &out, size_t bytes)
{
    if (!s_mounted) { out.println("[log] not mounted"); return; }
    static uint8_t chunk[1024];
    for (size_t i = 0; i < sizeof(chunk); i++) chunk[i] = (uint8_t)(i * 7 + 3);
    uint32_t t0 = millis();
    File f = LittleFS.open("/flashtest.bin", "w");
    size_t done = 0;
    while (f && done < bytes) {
        size_t n = bytes - done < sizeof(chunk) ? bytes - done : sizeof(chunk);
        if (f.write(chunk, n) != n) break;
        done += n;
    }
    if (f) f.close();
    uint32_t t1 = millis();
    LittleFS.remove("/flashtest.bin");
    out.printf("[log] flash test: wrote %u B in %lu ms, delete %lu ms\n", (unsigned)done, (unsigned long)(t1 - t0),
               (unsigned long)(millis() - t1));
}
