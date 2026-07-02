#include "profile.h"
#include "clk.h"
#include <Preferences.h>
#include <Arduino.h>
#include <math.h>
#include "logbuf.h"   // include last

static const int      PROF_N     = 24;      // hours per day
static const uint16_t PROF_MAGIC = 0x0F19;
static const char*    PROF_NS    = "occprofile";
static const char*    PROF_KEY   = "v1";
static const uint32_t FOLD_MS    = 10UL * 60UL * 1000UL;   // fold at most every 10 min
static const uint32_t SAVE_MS    = 10UL * 60UL * 1000UL;   // persist at most every 10 min
static const float    ALPHA      = 0.10f;                  // slow: averages across days

static const Config* s_cfg = nullptr;
static float    s_free[PROF_N];
static uint32_t s_seen = 0;          // bit i set once hour i has an observation
static bool     s_dirty = false;
static uint32_t s_lastFold = 0, s_lastSave = 0;

struct ProfBlob { uint16_t magic; float free[PROF_N]; uint32_t seen; };

static int hourOfDay() {
  uint32_t e = (uint32_t)clockEpoch();
  if (e == 0) return -1;                                   // clock not NTP-synced yet
  long local = (long)e + (long)(s_cfg ? s_cfg->tzOffsetMin : 0) * 60L;
  if (local < 0) local = 0;
  return (int)((local / 3600L) % 24L);
}

void profileBegin(const Config* cfg) {
  s_cfg = cfg;
  for (int i = 0; i < PROF_N; i++) s_free[i] = 0.0f;
  s_seen = 0;
  Preferences p;
  if (p.begin(PROF_NS, /*readOnly=*/true)) {
    ProfBlob b;
    size_t got = p.getBytes(PROF_KEY, &b, sizeof(b));
    p.end();
    if (got == sizeof(b) && b.magic == PROF_MAGIC) {
      s_seen = b.seen;
      for (int i = 0; i < PROF_N; i++) s_free[i] = isfinite(b.free[i]) ? b.free[i] : 0.0f;
    }
  }
}

void profileUpdate(const CurbResult& r) {
  if (!s_cfg || !r.valid) return;
  if (r.dark || r.warming || r.camera_moved) return;       // skip unreliable frames
  uint32_t now = millis();
  if (s_lastFold && (now - s_lastFold) < FOLD_MS) return;
  int h = hourOfDay();
  if (h < 0) return;                                        // need a real clock
  s_lastFold = now;
  if (!(s_seen & (1u << h))) s_free[h] = r.free_curb_m;     // first observation seeds directly
  else                       s_free[h] += ALPHA * (r.free_curb_m - s_free[h]);
  s_seen |= (1u << h);
  s_dirty = true;
}

void profileLoop() {
  if (!s_dirty) return;
  uint32_t now = millis();
  if (s_lastSave && (now - s_lastSave) < SAVE_MS) return;
  s_lastSave = now;
  ProfBlob b; b.magic = PROF_MAGIC; b.seen = s_seen;
  for (int i = 0; i < PROF_N; i++) b.free[i] = s_free[i];
  Preferences p;
  if (p.begin(PROF_NS, /*readOnly=*/false)) {
    if (p.putBytes(PROF_KEY, &b, sizeof(b)) == sizeof(b)) s_dirty = false;
    p.end();
  }
}

float profileTypicalNow() {
  int h = hourOfDay();
  if (h < 0 || !(s_seen & (1u << h))) return -1.0f;
  return s_free[h];
}
float profileTypicalNextHour() {
  int h = hourOfDay();
  if (h < 0) return -1.0f;
  int n = (h + 1) % PROF_N;
  if (!(s_seen & (1u << n))) return -1.0f;
  return s_free[n];
}
bool profileHasData() {
  int h = hourOfDay();
  return h >= 0 && (s_seen & (1u << h));
}
