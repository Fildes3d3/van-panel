#pragma once
/*
 * Persistent event log on the board's flash (LittleFS on the otherwise unused "spiffs" partition, 3.4 MB).
 *
 * Lines are collected in a RAM buffer (PSRAM) and written to flash once a minute (and when the screen goes dark).
 * A flash write stalls PSRAM, which feeds the RGB panel, so this was tested on the board: 5 x 32 KB writes
 * (~100 ms each, far more than a normal batch) with the screen lit showed no flicker or shift (2026-10-08).
 * On a sudden power loss the lines of the last minute at most are lost; the next boot line records the power-on.
 *
 * Read on demand over USB: type "log" in a serial terminal (115200 baud). "help" lists the other commands.
 * Two files of up to 1 MB each (current + previous), so the oldest records are dropped first.
 */

#include <Arduino.h>

void   vlog_begin();                                                    // mount (formats an empty partition once)
void   vlog(const char *fmt, ...) __attribute__((format(printf, 1, 2)));   // one timestamped line; any task
void   vlog_poll(bool screen_dark, bool busy);                          // flush policy; call from loop()
bool   vlog_flush();                                                    // write pending lines now
void   vlog_dump(Stream &out);                                          // stored log, oldest first, plus pending
void   vlog_clear();                                                    // delete the stored log
void   vlog_info(Stream &out);                                          // sizes and counters
void   vlog_flash_test(Stream &out, size_t bytes);                      // timed write+delete of a scratch file
