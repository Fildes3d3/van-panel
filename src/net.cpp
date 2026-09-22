/*
 * Globebus panel - networking task. See net.h for the contract.
 *
 * Services used (no accounts, no API keys):
 *   - Weather:    http://api.open-meteo.com/v1/forecast
 *   - Geocoding:  http://geocoding-api.open-meteo.com/v1/search   (manual place -> lat/lon)
 *   - IP location: http://ip-api.com/json  (automatic place; plain HTTP, city-level, free non-commercial tier;
 *                  on a phone hotspot it may report the carrier's city - hence the manual override)
 *   - Time:       NTP pool.ntp.org / time.cloudflare.com
 * All requests use PLAIN HTTP (verified 2026-09-22: open-meteo answers 200 on http, no redirect). HTTPS was dropped
 * because TLS needs ~30-40 KB of internal DMA RAM, which is not available while Bluetooth (Bluedroid, ~70 KB internal)
 * is on - weather failed with HTTP -1. Only public weather data is fetched and nothing personal is sent; the earlier
 * HTTPS did not verify certificates either. (http_get_json still supports https:// if ever needed.)
 * Wi-Fi password is stored in NVS in plain form (not encrypted) - acceptable for a van panel, noted for the record.
 */
#include "net.h"

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <BLEDevice.h>
#include "esp_sntp.h"
#include <esp_heap_caps.h>

static constexpr uint32_t WX_PERIOD_MS      = 20UL * 60UL * 1000UL;   // weather refresh
static constexpr uint32_t WX_RETRY_MS       = 2UL * 60UL * 1000UL;    // after a failure
static constexpr uint32_t GEO_PERIOD_MS     = 2UL * 60UL * 60UL * 1000UL;  // IP location refresh
static constexpr uint32_t STATUS_POLL_MS    = 500;
static constexpr uint32_t HTTP_TIMEOUT_MS   = 8000;

static NetState          S;
static SemaphoreHandle_t s_mtx;
static void (*s_on_time_synced)() = nullptr;

// Requests from the UI (guarded by s_mtx)
static bool s_req_scan = false, s_req_connect = false, s_req_wifi = false, s_req_ble = false;
static bool s_req_loc = false, s_req_geocode = false, s_req_weather = false;
static bool s_want_wifi = false, s_want_ble = false, s_want_loc_auto = true;
static char s_req_ssid[33], s_req_pass[65], s_req_place[48];
static char s_pass[65];                       // saved password (task-local use)
static volatile bool s_sntp_event = false;

static void lock()   { xSemaphoreTake(s_mtx, portMAX_DELAY); }
static void unlock() { xSemaphoreGive(s_mtx); }
static void bump()   { S.version++; }

static void set_status(const char *fmt, ...)
{
    char buf[96];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    lock();
    strlcpy(S.status, buf, sizeof(S.status));
    bump();
    unlock();
    Serial.printf("[net] %s\n", buf);
}

// ---------- public API -------------------------------------------------------

void net_get(NetState *out)
{
    lock();
    *out = S;
    unlock();
}

void net_set_wifi(bool on)            { lock(); s_want_wifi = on; s_req_wifi = true; unlock(); }
void net_request_scan()               { lock(); s_req_scan = true; unlock(); }
void net_set_ble(bool on)             { lock(); s_want_ble = on; s_req_ble = true; unlock(); }
void net_set_location_auto(bool on)   { lock(); s_want_loc_auto = on; s_req_loc = true; unlock(); }
void net_request_weather()            { lock(); s_req_weather = true; unlock(); }

void net_request_connect(const char *ssid, const char *pass)
{
    lock();
    strlcpy(s_req_ssid, ssid, sizeof(s_req_ssid));
    strlcpy(s_req_pass, pass, sizeof(s_req_pass));
    s_req_connect = true;
    unlock();
}

void net_request_geocode(const char *place_name)
{
    lock();
    strlcpy(s_req_place, place_name, sizeof(s_req_place));
    s_req_geocode = true;
    unlock();
}

const char *wx_text(int c)
{
    switch (c) {
    case 0: return "Clear sky";
    case 1: return "Mainly clear";
    case 2: return "Partly cloudy";
    case 3: return "Overcast";
    case 45: case 48: return "Fog";
    case 51: case 53: case 55: return "Drizzle";
    case 56: case 57: return "Freezing drizzle";
    case 61: return "Light rain";
    case 63: return "Rain";
    case 65: return "Heavy rain";
    case 66: case 67: return "Freezing rain";
    case 71: return "Light snow";
    case 73: return "Snow";
    case 75: return "Heavy snow";
    case 77: return "Snow grains";
    case 80: case 81: return "Rain showers";
    case 82: return "Heavy showers";
    case 85: case 86: return "Snow showers";
    case 95: return "Thunderstorm";
    case 96: case 99: return "Thunderstorm, hail";
    default: return "Unknown";
    }
}

// ---------- persistence ------------------------------------------------------

static void prefs_load()
{
    Preferences p;
    p.begin("globebus", true);
    S.wifi_enabled = p.getBool("wifi_en", false);
    S.ble_enabled  = p.getBool("ble_en", false);
    S.loc_auto     = p.getBool("loc_auto", true);
    p.getString("ssid", S.ssid, sizeof(S.ssid));
    p.getString("pass", s_pass, sizeof(s_pass));
    p.getString("loc_name", S.manual_name, sizeof(S.manual_name));
    float lat = p.getFloat("loc_lat", NAN), lon = p.getFloat("loc_lon", NAN);
    p.end();
    S.have_creds = S.ssid[0] != 0;
    if (!S.loc_auto && !isnan(lat) && !isnan(lon)) {
        S.lat = lat;
        S.lon = lon;
        S.loc_valid = true;
        strlcpy(S.place, S.manual_name, sizeof(S.place));
    }
    s_want_wifi = S.wifi_enabled;
    s_want_ble = S.ble_enabled;
    s_want_loc_auto = S.loc_auto;
}

static void prefs_put_bool(const char *k, bool v)
{
    Preferences p;
    p.begin("globebus", false);
    p.putBool(k, v);
    p.end();
}

// ---------- HTTP helpers -----------------------------------------------------

static bool http_get_json(const String &url, JsonDocument &doc)
{
    HTTPClient http;
    http.setTimeout(HTTP_TIMEOUT_MS);
    http.setConnectTimeout(HTTP_TIMEOUT_MS);
    http.useHTTP10(true);        // no chunked transfer encoding: ArduinoJson parses the raw stream
    WiFiClientSecure tls;
    WiFiClient plain;
    bool ok;
    if (url.startsWith("https://")) {
        tls.setInsecure();
        ok = http.begin(tls, url);
    } else {
        ok = http.begin(plain, url);
    }
    if (!ok) {
        set_status("HTTP begin failed");
        return false;
    }
    int code = http.GET();
    if (code != HTTP_CODE_OK) {
        set_status("HTTP %d from %s", code, url.substring(0, 40).c_str());
        http.end();
        return false;
    }
    DeserializationError e = deserializeJson(doc, http.getStream());
    http.end();
    if (e) {
        set_status("JSON error: %s", e.c_str());
        return false;
    }
    return true;
}

static String url_encode(const char *s)
{
    String out;
    for (; *s; s++) {
        char c = *s;
        if (isalnum((unsigned char)c) || c == '-' || c == '_' || c == '.') out += c;
        else {
            char b[4];
            snprintf(b, sizeof(b), "%%%02X", (unsigned char)c);
            out += b;
        }
    }
    return out;
}

// Day of week 0=Sun..6=Sat (Sakamoto)
static int dow(int y, int m, int d)
{
    static const int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    if (m < 3) y -= 1;
    return (y + y / 4 - y / 100 + y / 400 + t[m - 1] + d) % 7;
}

// ---------- location & weather -----------------------------------------------

static bool geolocate_ip()
{
    JsonDocument doc;
    if (!http_get_json("http://ip-api.com/json/?fields=status,city,lat,lon", doc)) return false;
    if (strcmp(doc["status"] | "", "success") != 0) {
        set_status("IP location failed");
        return false;
    }
    lock();
    S.lat = doc["lat"] | 0.0f;
    S.lon = doc["lon"] | 0.0f;
    strlcpy(S.place, doc["city"] | "?", sizeof(S.place));
    S.loc_valid = true;
    bump();
    unlock();
    set_status("Location (IP): %s", S.place);
    return true;
}

static bool geocode(const char *name)
{
    JsonDocument doc;
    String url = "http://geocoding-api.open-meteo.com/v1/search?count=1&language=en&format=json&name=" + url_encode(name);
    if (!http_get_json(url, doc)) return false;
    JsonVariant r = doc["results"][0];
    if (r.isNull()) {
        set_status("Place not found: %s", name);
        return false;
    }
    float lat = r["latitude"] | NAN, lon = r["longitude"] | NAN;
    char resolved[48];
    snprintf(resolved, sizeof(resolved), "%s", (const char *)(r["name"] | name));
    Preferences p;
    p.begin("globebus", false);
    p.putString("loc_name", resolved);
    p.putFloat("loc_lat", lat);
    p.putFloat("loc_lon", lon);
    p.putBool("loc_auto", false);
    p.end();
    lock();
    strlcpy(S.manual_name, resolved, sizeof(S.manual_name));
    strlcpy(S.place, resolved, sizeof(S.place));
    S.lat = lat;
    S.lon = lon;
    S.loc_valid = true;
    S.loc_auto = false;
    s_want_loc_auto = false;
    bump();
    unlock();
    set_status("Location (manual): %s", resolved);
    return true;
}

static bool fetch_weather()
{
    float lat, lon;
    lock();
    lat = S.lat;
    lon = S.lon;
    unlock();
    char url[400];
    snprintf(url, sizeof(url),
             "http://api.open-meteo.com/v1/forecast?latitude=%.4f&longitude=%.4f"
             "&current=temperature_2m,apparent_temperature,relative_humidity_2m,weather_code,wind_speed_10m,"
             "wind_direction_10m,is_day"
             "&daily=weather_code,temperature_2m_max,temperature_2m_min,precipitation_probability_max"
             "&timezone=auto&forecast_days=%d",
             lat, lon, WX_DAYS);
    JsonDocument doc;
    if (!http_get_json(String(url), doc)) return false;

    JsonObject cur = doc["current"];
    JsonObject dly = doc["daily"];
    if (cur.isNull() || dly.isNull()) {
        set_status("Weather: unexpected response");
        return false;
    }
    static const char *const names[] = {"Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
    lock();
    S.temp     = cur["temperature_2m"] | NAN;
    S.feels    = cur["apparent_temperature"] | NAN;
    S.humidity = cur["relative_humidity_2m"] | -1;
    S.code     = cur["weather_code"] | -1;
    S.wind_kmh = cur["wind_speed_10m"] | NAN;
    S.wind_dir = cur["wind_direction_10m"] | -1;
    S.is_day   = (cur["is_day"] | 1) == 1;
    for (int i = 0; i < WX_DAYS; i++) {
        WxDay &d = S.day[i];
        d.code = dly["weather_code"][i] | -1;
        d.tmax = dly["temperature_2m_max"][i] | NAN;
        d.tmin = dly["temperature_2m_min"][i] | NAN;
        d.pop  = dly["precipitation_probability_max"][i] | -1;
        const char *date = dly["time"][i] | "";
        int y = 0, m = 0, dd = 0;
        if (i == 0) strlcpy(d.label, "Today", sizeof(d.label));
        else if (sscanf(date, "%d-%d-%d", &y, &m, &dd) == 3) strlcpy(d.label, names[dow(y, m, dd)], sizeof(d.label));
        else strlcpy(d.label, "?", sizeof(d.label));
    }
    S.wx_valid = true;
    S.wx_time  = time(nullptr);
    bump();
    unlock();
    set_status("Weather updated (%.1f C, %s)", S.temp, wx_text(S.code));
    return true;
}

// ---------- Wi-Fi / time / BLE -----------------------------------------------

static void sntp_cb(struct timeval *) { s_sntp_event = true; }

static void wifi_apply(bool on)
{
    if (on) {
        WiFi.mode(WIFI_STA);
        WiFi.setAutoReconnect(true);
        if (S.have_creds) {
            WiFi.begin(S.ssid, s_pass);
            set_status("Connecting to %s", S.ssid);
        } else {
            set_status("Wi-Fi on - no network saved");
        }
    } else {
        WiFi.disconnect(true);
        WiFi.mode(WIFI_OFF);
        set_status("Wi-Fi off");
    }
}

// Bluedroid needs a large block of INTERNAL RAM; starting it without enough memory aborts the chip and starves Wi-Fi
// encryption (seen 2026-09-22: BLE_INIT malloc failed -> reboot -> esp-sha/esp-aes alloc failures). Check first.
static constexpr size_t BLE_MIN_INTERNAL_FREE = 70 * 1024;   // conservative; free internal heap is logged for tuning

static bool s_ble_up = false;
static void ble_apply(bool on)
{
    if (on && !s_ble_up) {
        size_t free_int = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
        size_t big_int  = heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL);
        if (free_int < BLE_MIN_INTERNAL_FREE) {
            prefs_put_bool("ble_en", false);
            lock(); S.ble_enabled = false; s_want_ble = false; bump(); unlock();
            set_status("Bluetooth NOT started: %u B internal RAM free (need %u) - switched off",
                       (unsigned)free_int, (unsigned)BLE_MIN_INTERNAL_FREE);
            return;
        }
        BLEDevice::init("Globebus");
        BLEDevice::getAdvertising()->start();
        s_ble_up = true;
        set_status("Bluetooth on (internal RAM %u -> %u B, largest block was %u)", (unsigned)free_int,
                   (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL), (unsigned)big_int);
    } else if (!on && s_ble_up) {
        BLEDevice::getAdvertising()->stop();
        BLEDevice::deinit(false);
        s_ble_up = false;
        set_status("Bluetooth off");
    }
}

static void do_scan()
{
    lock();
    S.scanning = true;
    bump();
    unlock();
    bool was_off = WiFi.getMode() == WIFI_OFF;
    if (was_off) WiFi.mode(WIFI_STA);
    int n = WiFi.scanNetworks(false, false);
    lock();
    S.scan_count = 0;
    for (int i = 0; i < n && S.scan_count < NET_MAX_SCAN; i++) {
        String ssid = WiFi.SSID(i);
        if (ssid.length() == 0) continue;
        bool dup = false;                              // same SSID from several APs: keep the strongest (scan is sorted)
        for (int k = 0; k < S.scan_count; k++) dup |= (ssid == S.scan_ssid[k]);
        if (dup) continue;
        strlcpy(S.scan_ssid[S.scan_count], ssid.c_str(), 33);
        S.scan_rssi[S.scan_count] = WiFi.RSSI(i);
        S.scan_open[S.scan_count] = WiFi.encryptionType(i) == WIFI_AUTH_OPEN;
        S.scan_count++;
    }
    S.scanning = false;
    bump();
    unlock();
    WiFi.scanDelete();
    if (was_off && !S.wifi_enabled) WiFi.mode(WIFI_OFF);
    set_status("Scan: %d networks", n < 0 ? 0 : n);
}

static void net_task(void *)
{
    prefs_load();
    lock(); bump(); unlock();

    sntp_set_time_sync_notification_cb(sntp_cb);
    if (S.wifi_enabled) wifi_apply(true);
    if (S.ble_enabled) ble_apply(true);

    bool     was_connected = false;
    uint32_t next_wx = 0, next_geo = 0, last_poll = 0;

    for (;;) {
        // --- take requests
        bool r_scan, r_conn, r_wifi, r_ble, r_loc, r_geo, r_wx, want_wifi, want_ble, want_auto;
        char ssid[33], pass[65], place[48];
        lock();
        r_scan = s_req_scan; r_conn = s_req_connect; r_wifi = s_req_wifi; r_ble = s_req_ble;
        r_loc = s_req_loc; r_geo = s_req_geocode; r_wx = s_req_weather;
        s_req_scan = s_req_connect = s_req_wifi = s_req_ble = s_req_loc = s_req_geocode = s_req_weather = false;
        want_wifi = s_want_wifi; want_ble = s_want_ble; want_auto = s_want_loc_auto;
        strlcpy(ssid, s_req_ssid, sizeof(ssid));
        strlcpy(pass, s_req_pass, sizeof(pass));
        strlcpy(place, s_req_place, sizeof(place));
        unlock();

        if (r_wifi) {
            prefs_put_bool("wifi_en", want_wifi);
            lock(); S.wifi_enabled = want_wifi; bump(); unlock();
            wifi_apply(want_wifi);
        }
        if (r_conn) {
            Preferences p;
            p.begin("globebus", false);
            p.putString("ssid", ssid);
            p.putString("pass", pass);
            p.putBool("wifi_en", true);
            p.end();
            strlcpy(s_pass, pass, sizeof(s_pass));
            lock();
            strlcpy(S.ssid, ssid, sizeof(S.ssid));
            S.have_creds = true;
            S.wifi_enabled = true;
            s_want_wifi = true;
            bump();
            unlock();
            WiFi.disconnect(false);
            wifi_apply(true);
        }
        if (r_scan) do_scan();
        if (r_ble) {
            prefs_put_bool("ble_en", want_ble);
            lock(); S.ble_enabled = want_ble; bump(); unlock();
            ble_apply(want_ble);
        }
        if (r_loc) {
            prefs_put_bool("loc_auto", want_auto);
            lock(); S.loc_auto = want_auto; bump(); unlock();
            next_geo = 0;
            next_wx = 0;
            if (!want_auto) {      // back to the saved manual place
                Preferences p;
                p.begin("globebus", true);
                float lat = p.getFloat("loc_lat", NAN), lon = p.getFloat("loc_lon", NAN);
                p.end();
                lock();
                S.loc_valid = !isnan(lat) && !isnan(lon);
                if (S.loc_valid) { S.lat = lat; S.lon = lon; strlcpy(S.place, S.manual_name, sizeof(S.place)); }
                bump();
                unlock();
            }
        }

        // --- connection state
        uint32_t now = millis();
        if (now - last_poll >= STATUS_POLL_MS) {
            last_poll = now;
            bool conn = WiFi.status() == WL_CONNECTED;
            lock();
            if (conn != S.connected) bump();
            S.connected = conn;
            if (conn) {
                int rssi = WiFi.RSSI();
                if (abs(rssi - S.rssi) >= 3) bump();
                S.rssi = rssi;
                strlcpy(S.ip, WiFi.localIP().toString().c_str(), sizeof(S.ip));
            } else {
                S.ip[0] = 0;
            }
            unlock();
            if (conn && !was_connected) {
                set_status("Connected to %s (internal RAM free %u B)", S.ssid,
                           (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
                configTzTime(GLOBEBUS_TZ, "pool.ntp.org", "time.cloudflare.com");
                next_wx = 0;
                next_geo = 0;
            }
            if (!conn && was_connected) set_status("Wi-Fi disconnected");
            was_connected = conn;
        }

        if (s_sntp_event) {
            s_sntp_event = false;
            lock();
            S.time_synced = true;
            S.last_sync = time(nullptr);
            bump();
            unlock();
            set_status("Internet time synced");
            if (s_on_time_synced) s_on_time_synced();
        }

        // --- location + weather (only when online)
        if (S.connected) {
            if (r_geo && place[0]) {
                if (geocode(place)) next_wx = 0;
            }
            if (S.loc_auto && (int32_t)(now - next_geo) >= 0) {
                next_geo = now + (geolocate_ip() ? GEO_PERIOD_MS : WX_RETRY_MS);
                next_wx = 0;
            }
            if (r_wx) next_wx = 0;
            if (S.loc_valid && (int32_t)(now - next_wx) >= 0) {
                next_wx = now + (fetch_weather() ? WX_PERIOD_MS : WX_RETRY_MS);
            }
        } else if (r_geo) {
            set_status("Connect to Wi-Fi first to look up a place");
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}

void net_begin(void (*on_time_synced)())
{
    s_on_time_synced = on_time_synced;
    s_mtx = xSemaphoreCreateMutex();
    memset(&S, 0, sizeof(S));
    S.loc_auto = true;
    xTaskCreatePinnedToCore(net_task, "net", 12288, nullptr, 1, nullptr, 0);
}
