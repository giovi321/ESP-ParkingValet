#include "logbuf.h"
#include "esp_log.h"
#include <stdarg.h>
#include <string.h>

#define LOGBUF_SIZE 6144

static char            s_buf[LOGBUF_SIZE];
static size_t          s_start = 0, s_len = 0;
static portMUX_TYPE    s_mux  = portMUX_INITIALIZER_UNLOCKED;
static vprintf_like_t  s_prev = nullptr;

static void appendBytes(const char* p, size_t n) {
  portENTER_CRITICAL(&s_mux);
  for (size_t i = 0; i < n; i++) {
    s_buf[(s_start + s_len) % LOGBUF_SIZE] = p[i];
    if (s_len < LOGBUF_SIZE) s_len++;
    else s_start = (s_start + 1) % LOGBUF_SIZE;   // overwrite oldest
  }
  portEXIT_CRITICAL(&s_mux);
}

static int vhook(const char* fmt, va_list ap) {
  char line[256];
  va_list ap2;
  va_copy(ap2, ap);
  int n = vsnprintf(line, sizeof(line), fmt, ap2);
  va_end(ap2);
  if (n > 0) appendBytes(line, (size_t)n < sizeof(line) ? (size_t)n : sizeof(line) - 1);
  return s_prev ? s_prev(fmt, ap) : n;   // forward to the original (UART) logger
}

void logbufBegin() {
  s_prev = esp_log_set_vprintf(vhook);
  // The occasional bad sensor frame makes the JPEG decoder shout; we already
  // skip those frames in cv.cpp, so quiet the tag to keep the console readable.
  esp_log_level_set("esp_jpg_decode", ESP_LOG_NONE);
}

void logbufPrintf(const char* fmt, ...) {
  char line[224];
  va_list ap;
  va_start(ap, fmt);
  int n = vsnprintf(line, sizeof(line), fmt, ap);
  va_end(ap);
  if (n < 0) return;
  if (n > (int)sizeof(line) - 1) n = sizeof(line) - 1;

  char pfx[16];
  int pn = snprintf(pfx, sizeof(pfx), "[%8lu] ", (unsigned long)millis());

  Serial.write((const uint8_t*)pfx, pn);     // still goes to the physical UART
  Serial.write((const uint8_t*)line, n);
  Serial.write((uint8_t)'\n');

  appendBytes(pfx, pn);                       // ...and into the web console ring
  appendBytes(line, n);
  appendBytes("\n", 1);
}

size_t logbufRead(char* out, size_t cap) {
  portENTER_CRITICAL(&s_mux);
  size_t n = s_len <= cap ? s_len : cap;
  size_t startIdx = (s_start + (s_len - n)) % LOGBUF_SIZE;
  size_t first = LOGBUF_SIZE - startIdx;
  if (first > n) first = n;
  memcpy(out, s_buf + startIdx, first);
  memcpy(out + first, s_buf, n - first);
  portEXIT_CRITICAL(&s_mux);
  return n;
}
