#pragma once
/*
 * Leisure-battery history for the history page: one point every 5 minutes, 7 days kept (2016 points, PSRAM),
 * saved to the log file system every 30 minutes and loaded at boot, so a restart does not wipe the chart.
 * Points are only taken with a valid clock and a van feed (the voltage comes from the A2 supply divider).
 */

#include <Arduino.h>
#include <time.h>

struct HistPoint {
    uint32_t t;        // unix time
    int16_t  cv;       // battery volts x 100
    uint8_t  flags;    // bit 0: charging
    uint8_t  pad;
};

static constexpr int      HIST_CAP      = 7 * 24 * 12;   // 5-minute points for 7 days
static constexpr uint32_t HIST_STEP_S   = 5 * 60;
static constexpr uint8_t  HIST_CHARGING = 0x01;

void hist_begin();                                       // allocate + load the saved file (log FS must be mounted)
void hist_add(time_t t, float volts, bool charging);     // append one point
bool hist_save();                                        // write the file now
void hist_save_if_due();                                 // every 30 min, called from loop()
int  hist_window(time_t from, time_t to, HistPoint *out, int max);   // points in [from, to], oldest first
int  hist_count();
