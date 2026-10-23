#include "hist.h"

#include <LittleFS.h>
#include <esp_heap_caps.h>

static constexpr const char *HIST_FILE    = "/hist.bin";
static constexpr uint32_t    HIST_MAGIC   = 0x48535431;     // "HST1"
static constexpr uint32_t    SAVE_EVERY_MS = 30UL * 60 * 1000;

static HistPoint *s_buf    = nullptr;   // ring
static int        s_head   = 0;         // next write index
static int        s_count  = 0;
static bool       s_dirty  = false;
static uint32_t   s_saved_ms = 0;

void hist_begin()
{
    if (!s_buf) s_buf = (HistPoint *)heap_caps_calloc(HIST_CAP, sizeof(HistPoint), MALLOC_CAP_SPIRAM);
    if (!s_buf) return;
    s_saved_ms = millis();
    if (!LittleFS.exists(HIST_FILE)) return;
    File f = LittleFS.open(HIST_FILE, "r");
    if (!f) return;
    uint32_t hdr[3] = {};                               // magic, count, head
    bool ok = f.read((uint8_t *)hdr, sizeof(hdr)) == sizeof(hdr) && hdr[0] == HIST_MAGIC &&
              hdr[1] <= (uint32_t)HIST_CAP && hdr[2] < (uint32_t)HIST_CAP;
    if (ok) ok = f.read((uint8_t *)s_buf, HIST_CAP * sizeof(HistPoint)) == HIST_CAP * sizeof(HistPoint);
    f.close();
    if (ok) {
        s_count = (int)hdr[1];
        s_head  = (int)hdr[2];
    } else {
        memset(s_buf, 0, HIST_CAP * sizeof(HistPoint));
        s_count = s_head = 0;
    }
    Serial.printf("[hist] %s, %d points\n", ok ? "loaded" : "file unreadable, starting empty", s_count);
}

void hist_add(time_t t, float volts, bool charging)
{
    if (!s_buf || isnan(volts)) return;
    HistPoint &p = s_buf[s_head];
    p.t     = (uint32_t)t;
    p.cv    = (int16_t)lroundf(volts * 100.0f);
    p.flags = charging ? HIST_CHARGING : 0;
    p.pad   = 0;
    s_head  = (s_head + 1) % HIST_CAP;
    if (s_count < HIST_CAP) s_count++;
    s_dirty = true;
}

bool hist_save()
{
    if (!s_buf || !s_dirty) return true;
    File f = LittleFS.open(HIST_FILE, "w");
    if (!f) return false;
    uint32_t hdr[3] = {HIST_MAGIC, (uint32_t)s_count, (uint32_t)s_head};
    bool ok = f.write((const uint8_t *)hdr, sizeof(hdr)) == sizeof(hdr) &&
              f.write((const uint8_t *)s_buf, HIST_CAP * sizeof(HistPoint)) == HIST_CAP * sizeof(HistPoint);
    f.close();
    if (ok) s_dirty = false;
    s_saved_ms = millis();
    return ok;
}

void hist_save_if_due()
{
    if (s_dirty && millis() - s_saved_ms >= SAVE_EVERY_MS) hist_save();
}

int hist_window(time_t from, time_t to, HistPoint *out, int max)
{
    if (!s_buf) return 0;
    int n = 0;
    int start = (s_head - s_count + HIST_CAP) % HIST_CAP;
    for (int i = 0; i < s_count && n < max; i++) {
        const HistPoint &p = s_buf[(start + i) % HIST_CAP];
        if ((time_t)p.t >= from && (time_t)p.t <= to) out[n++] = p;
    }
    return n;
}

int hist_count() { return s_count; }
