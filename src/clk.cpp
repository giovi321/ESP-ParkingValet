#include "clk.h"

void clockBegin() {
  // UTC (offset 0, no DST); Home Assistant / your backend localize. Public NTP pools — the
  // device reaches the internet once it's in STA mode.
  configTime(0, 0, "pool.ntp.org", "time.google.com");
}

bool clockSynced() {
  return time(nullptr) > 1700000000;   // ~2023-11-14; anything earlier = not synced
}

time_t clockEpoch() {
  time_t t = time(nullptr);
  return clockSynced() ? t : 0;
}

String clockIso() {
  if (!clockSynced()) return String("");
  time_t t = time(nullptr);
  struct tm tmv;
  gmtime_r(&t, &tmv);
  char b[24];
  strftime(b, sizeof(b), "%Y-%m-%dT%H:%M:%SZ", &tmv);
  return String(b);
}
