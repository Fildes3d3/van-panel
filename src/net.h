/*
 * Globebus panel - networking: Wi-Fi, internet time, weather (open-meteo.com), BLE on/off.
 *
 * Runs as its own FreeRTOS task on core 0 so DNS/HTTP/TLS waits never block the ADC sampling in loop() (core 1).
 * The UI never calls network APIs: it reads a copy of NetState (net_get) and posts requests (net_request_*).
 * All settings persist in NVS (Preferences namespace "globebus").
 */
#pragma once

#include <Arduino.h>
#include <time.h>

// Romania: EET/EEST with EU DST rules (last Sunday of March 03:00 / last Sunday of October 04:00)
#define GLOBEBUS_TZ "EET-2EEST,M3.5.0/3,M10.5.0/4"

static constexpr int NET_MAX_SCAN = 16;
static constexpr int WX_DAYS      = 4;

struct WxDay {
    int   code;         // WMO weather code
    float tmax, tmin;   // deg C
    int   pop;          // max precipitation probability, %
    char  label[8];     // "Today", "Mon", ...
};

struct NetState {
    uint32_t version;           // increments on every change the UI should redraw

    // Wi-Fi
    bool wifi_enabled;
    bool have_creds;
    bool connected;
    char ssid[33];              // saved / connected network
    int  rssi;
    char ip[16];

    // scan
    bool scanning;
    int  scan_count;
    char scan_ssid[NET_MAX_SCAN][33];
    int  scan_rssi[NET_MAX_SCAN];
    bool scan_open[NET_MAX_SCAN];

    // internet time
    bool   time_synced;
    time_t last_sync;

    // BLE
    bool ble_enabled;

    // weather location
    bool  loc_auto;             // true: IP geolocation; false: manual place
    char  manual_name[48];      // manual place as typed/resolved
    char  place[48];            // place currently used for the weather
    float lat, lon;
    bool  loc_valid;

    // weather
    bool   wx_valid;
    time_t wx_time;             // when fetched (local epoch)
    float  temp, feels, wind_kmh;
    int    humidity, wind_dir, code;
    bool   is_day;
    WxDay  day[WX_DAYS];

    char status[96];            // last notable network event / error, for the settings screen
};

// Start the network task. on_time_synced runs in the network task after an NTP sync (write the RTC there or set a flag).
void net_begin(void (*on_time_synced)());

void net_get(NetState *out);                               // thread-safe copy

void net_set_wifi(bool on);
void net_request_scan();
void net_request_connect(const char *ssid, const char *pass);   // saves credentials, connects
void net_set_ble(bool on);
void net_set_location_auto(bool on);
void net_request_geocode(const char *place_name);          // resolve and store a manual place (switches to manual)
void net_request_weather();                                // refresh now

const char *wx_text(int code);                             // English condition text for a WMO code
