#pragma once
#include <Arduino.h>

// Captures the ESP log stream (everything from log_e/w/i and library logs like
// WiFi events, HTTPClient, esp_jpg_decode) into an in-memory ring buffer, while
// still forwarding to the real UART. The web UI's serial console reads it.

void   logbufBegin();                       // install the esp_log capture hook
size_t logbufRead(char* out, size_t cap);   // copy buffered text oldest->newest; returns bytes

// Printf into the console ring buffer + UART (uptime-prefixed). Our log_i/_w/_e
// are redefined below to route here, because Arduino's ARDUHAL log_x does NOT
// pass through esp_log_set_vprintf and so wouldn't otherwise be captured.
void   logbufPrintf(const char* fmt, ...) __attribute__((format(printf, 1, 2)));

#ifdef log_i
#undef log_i
#endif
#ifdef log_w
#undef log_w
#endif
#ifdef log_e
#undef log_e
#endif
#define log_i(fmt, ...) logbufPrintf("[I] " fmt, ##__VA_ARGS__)
#define log_w(fmt, ...) logbufPrintf("[W] " fmt, ##__VA_ARGS__)
#define log_e(fmt, ...) logbufPrintf("[E] " fmt, ##__VA_ARGS__)
