#include "spool.h"
#include <LittleFS.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "esp_heap_caps.h"
#include "net.h"
#include "clk.h"
#include "web_server.h"   // webNoteSend
#include "logbuf.h"       // log macros route to the web console (include last)

// ---------------------------------------------------------------------------
// State. The committed entries form a contiguous range [s_head, s_nextSeq) on
// disk: we only ever append at the tail and delete from the head, so a simple
// pair of counters plus a byte tally is enough — no per-drain directory scan.
// ---------------------------------------------------------------------------
static const Config* s_cfg     = nullptr;
static bool          s_ready   = false;   // backend mounted
static bool          s_armed   = false;   // allowed to mount (set after OTA image confirmed)
static bool          s_mountTried = false; // mount attempted (don't retry on failure)
static uint32_t      s_head    = 0;        // seq of the oldest queued entry
static uint32_t      s_nextSeq = 0;        // next seq to assign
static uint32_t      s_count   = 0;        // queued entries
static uint32_t      s_bytes   = 0;        // sum of queued file sizes (logical)
static uint32_t      s_lastDrainMs = 0;

static const char* SDIR        = "/spool";
static const size_t FS_MARGIN  = 8192;     // keep this many bytes free in the FS

static void pathBin(uint32_t seq, char* buf, size_t n) { snprintf(buf, n, "%s/%08u.bin", SDIR, (unsigned)seq); }
static void pathTmp(uint32_t seq, char* buf, size_t n) { snprintf(buf, n, "%s/%08u.tmp", SDIR, (unsigned)seq); }

static size_t fileSize(const char* path) {
  File f = LittleFS.open(path, "r");
  size_t s = f ? f.size() : 0;
  if (f) f.close();
  return s;
}

// Parse "<8 digits>.bin"/".tmp" from a possibly-pathful name. kind: 0=.bin, 1=.tmp, -1=other.
static bool parseName(const char* name, uint32_t* seq, int* kind) {
  const char* slash = strrchr(name, '/');
  const char* b = slash ? slash + 1 : name;
  char* end = nullptr;
  unsigned long v = strtoul(b, &end, 10);
  if (end == b || !end || *end != '.') return false;
  if      (!strcmp(end, ".bin")) *kind = 0;
  else if (!strcmp(end, ".tmp")) *kind = 1;
  else                           *kind = -1;
  *seq = (uint32_t)v;
  return true;
}

// Remove the oldest entry, updating counters, and advance the head to the next
// existing file. Safe to call even if the head file is already gone.
static void deleteHead() {
  char path[40]; pathBin(s_head, path, sizeof(path));
  size_t sz = fileSize(path);
  LittleFS.remove(path);
  if (s_bytes >= sz) s_bytes -= sz; else s_bytes = 0;
  if (s_count) s_count--;
  s_head++;
  while (s_count > 0 && s_head < s_nextSeq) {       // skip any gaps (e.g. after a crash)
    char p[40]; pathBin(s_head, p, sizeof(p));
    if (LittleFS.exists(p)) break;
    s_head++;
  }
  if (s_count == 0) s_head = s_nextSeq;
}

// Mount LittleFS and recover any queued entries. Deferred to the first spool
// activity (NOT called from setup()): a slow or failed first-boot format then
// can't stall the web server, and an OTA'd build boots far enough through
// loop() to be marked valid before any flash format runs. Tries exactly once;
// on failure the spool stays disabled until the next reboot rather than
// hammering the flash every loop.
static bool ensureMounted() {
  if (s_ready) return true;
  if (!s_armed) return false;          // not yet cleared to touch flash (pre-OTA-confirm)
  if (s_mountTried) return false;
  s_mountTried = true;

#ifdef PARKINGVALET_ENABLE_SD
  // Future hardware with free pins: try the SD backend here when backend==auto.
  // This board's camera owns the SD_MMC pins, so SD is never attempted.
#endif
  if (s_cfg && s_cfg->spoolBackend == SPOOL_BACKEND_AUTO)
    log_i("spool: SD unavailable on this board (camera uses SD pins) -> internal flash");

  // Mount LittleFS on the "spiffs" partition (formats on first use).
  if (!LittleFS.begin(/*formatOnFail=*/true, "/littlefs", 10, "spiffs")) {
    log_e("spool: LittleFS mount failed; offline queue disabled until reboot");
    return false;
  }
  if (!LittleFS.exists(SDIR)) LittleFS.mkdir(SDIR);

  // Recover any queued entries: find the committed range and tally bytes; drop
  // half-written ".tmp" leftovers from an interrupted enqueue.
  uint32_t minSeq = 0xFFFFFFFF, maxSeq = 0;
  File dir = LittleFS.open(SDIR);
  if (dir && dir.isDirectory()) {
    for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
      uint32_t seq; int kind;
      bool ok = parseName(f.name(), &seq, &kind);
      size_t sz = f.size();
      f.close();
      if (!ok) continue;
      if (kind == 1) {                     // orphan temp -> remove
        char p[40]; pathTmp(seq, p, sizeof(p));
        LittleFS.remove(p);
        continue;
      }
      if (kind != 0) continue;             // not ours
      if (seq < minSeq) minSeq = seq;
      if (seq > maxSeq) maxSeq = seq;
      s_count++;
      s_bytes += sz;
    }
  }
  if (dir) dir.close();

  if (s_count > 0) { s_head = minSeq; s_nextSeq = maxSeq + 1; }
  else             { s_head = 0;      s_nextSeq = 0; }

  s_ready = true;
  log_i("spool: ready (backend=flash, %u queued, %u bytes, total=%u/%u)",
        (unsigned)s_count, (unsigned)s_bytes,
        (unsigned)LittleFS.usedBytes(), (unsigned)LittleFS.totalBytes());
  return true;
}

void spoolBegin(const Config* cfg) {
  s_cfg = cfg;
  s_ready = false;
  s_mountTried = false;
  s_head = s_nextSeq = s_count = s_bytes = 0;
  // The LittleFS mount is intentionally deferred to ensureMounted() on first
  // use, so setup() and the web server never wait on a flash format and a
  // freshly-OTA'd build can boot far enough to confirm itself valid.
}

void spoolArm() { s_armed = true; }

void spoolEnqueue(const char* event, const uint8_t* jpg, size_t jpgLen,
                  int count, int prevCount, const bool* slots, int nSlots) {
  if (!s_cfg || s_cfg->spoolMode == SPOOL_OFF) return;
  if (!ensureMounted()) return;

  bool storeImg = (s_cfg->spoolMode == SPOOL_PHOTO) && jpg && jpgLen > 0;

  // Build the meta line. Kept short; the receiver gets device/auth from the POST.
  auto buildMeta = [&](bool withImg, String& out) {
    JsonDocument m;
    m["event"] = event;
    m["count"] = count;
    m["prev"]  = prevCount;
    m["ts"]    = (uint32_t)clockEpoch();
    m["iso"]   = clockIso();
    JsonArray sa = m["slots"].to<JsonArray>();
    for (int i = 0; i < nSlots && i < MAX_ROIS; i++) sa.add(slots[i]);
    m["img"]   = withImg ? 1 : 0;
    serializeJson(m, out);
  };

  String meta; buildMeta(storeImg, meta);
  size_t entryBytes = meta.length() + 1 + (storeImg ? jpgLen : 0);

  const uint32_t maxEntries = s_cfg->spoolMaxEntries ? s_cfg->spoolMaxEntries : 1;
  const uint32_t maxBytes   = (uint32_t)(s_cfg->spoolMaxKB ? s_cfg->spoolMaxKB : 1) * 1024UL;

  // Make room: honour the entry cap, the byte cap, and the physical FS budget.
  while (s_count > 0 && s_count >= maxEntries) deleteHead();
  while (s_count > 0 && (s_bytes + entryBytes) > maxBytes) deleteHead();
  while (s_count > 0 && (LittleFS.usedBytes() + entryBytes + FS_MARGIN) > LittleFS.totalBytes()) deleteHead();

  // If a photo entry still can't fit on its own, fall back to count-only so the
  // change is never lost (just the image).
  bool fits = (entryBytes <= maxBytes) &&
              (LittleFS.usedBytes() + entryBytes + FS_MARGIN <= LittleFS.totalBytes());
  if (!fits && storeImg) {
    log_w("spool: photo (%u B) won't fit -> storing count-only for this change", (unsigned)jpgLen);
    storeImg = false;
    buildMeta(false, meta);
    entryBytes = meta.length() + 1;
    fits = (entryBytes <= maxBytes) &&
           (LittleFS.usedBytes() + entryBytes + FS_MARGIN <= LittleFS.totalBytes());
  }
  if (!fits) { log_e("spool: cannot store change (filesystem full)"); return; }

  // Write to a temp file, then rename into place (atomic commit).
  uint32_t seq = s_nextSeq;
  char tmp[40], dst[40];
  pathTmp(seq, tmp, sizeof(tmp));
  pathBin(seq, dst, sizeof(dst));

  File f = LittleFS.open(tmp, "w");
  if (!f) { log_e("spool: open '%s' failed", tmp); return; }
  bool wrote = (f.print(meta) == meta.length());
  wrote = f.write('\n') == 1 && wrote;
  if (wrote && storeImg) wrote = (f.write(jpg, jpgLen) == jpgLen);
  size_t finalSz = f.size();
  f.close();
  if (!wrote) { LittleFS.remove(tmp); log_e("spool: write '%s' failed (FS full?)", tmp); return; }
  if (!LittleFS.rename(tmp, dst)) { LittleFS.remove(tmp); log_e("spool: commit '%s' failed", dst); return; }

  if (s_count == 0) s_head = seq;
  s_nextSeq = seq + 1;
  s_count++;
  s_bytes += finalSz;
  log_i("spool: queued #%u (%s, %u B) -> %u entries / %u B",
        (unsigned)seq, storeImg ? "photo" : "count", (unsigned)finalSz,
        (unsigned)s_count, (unsigned)s_bytes);
}

void spoolDrain() {
  if (!s_cfg || s_cfg->spoolMode == SPOOL_OFF) return;
  if (netIsAP() || WiFi.status() != WL_CONNECTED) return;
  if (!s_cfg->whEnabled || !s_cfg->whUrl[0]) return;   // nowhere to deliver; keep queued
  if (!ensureMounted() || s_count == 0) return;

  uint32_t now = millis();
  uint32_t iv  = s_cfg->minSendIntervalMs;
  if (iv && (uint32_t)(now - s_lastDrainMs) < iv) return;   // don't hammer the receiver
  s_lastDrainMs = now;

  char path[40]; pathBin(s_head, path, sizeof(path));
  File f = LittleFS.open(path, "r");
  if (!f) { deleteHead(); return; }                         // missing head -> skip it

  String metaLine = f.readStringUntil('\n');
  size_t total = f.size();
  size_t imgLen = (total > metaLine.length() + 1) ? (total - metaLine.length() - 1) : 0;

  JsonDocument m;
  if (deserializeJson(m, metaLine) != DeserializationError::Ok) {
    f.close();
    log_w("spool: dropping corrupt entry #%u", (unsigned)s_head);
    deleteHead();
    return;
  }

  const char* event = m["event"] | "count_changed";
  int count = m["count"] | 0;
  int prev  = m["prev"]  | 0;
  uint32_t ts = m["ts"]  | 0;
  const char* iso = m["iso"] | "";
  bool hasImg = (m["img"] | 0) != 0;

  bool slots[MAX_ROIS]; int nSlots = 0;
  for (JsonVariant v : m["slots"].as<JsonArray>()) { if (nSlots >= MAX_ROIS) break; slots[nSlots++] = v.as<bool>(); }

  uint8_t* buf = nullptr;
  if (hasImg && imgLen > 0) {
    buf = (uint8_t*)heap_caps_malloc(imgLen, MALLOC_CAP_SPIRAM);
    if (!buf) buf = (uint8_t*)malloc(imgLen);
    if (buf && f.read(buf, imgLen) != imgLen) { free(buf); buf = nullptr; imgLen = 0; }
    else if (!buf) imgLen = 0;
  }
  f.close();

  uint32_t nowEpoch = (uint32_t)clockEpoch();
  uint32_t ageS = (nowEpoch && ts && nowEpoch > ts) ? (nowEpoch - ts) : 0;

  int code = netSendEvent(*s_cfg, event, buf, imgLen, count, prev, slots, nSlots,
                          ts, iso, /*queued=*/true, ageS);
  if (buf) free(buf);
  webNoteSend(event, count, code);

  if (code >= 200 && code < 400) {
    log_i("spool: delivered #%u (HTTP %d) -> %u left", (unsigned)s_head, code, (unsigned)(s_count - 1));
    deleteHead();
  } else {
    log_w("spool: delivery of #%u failed (%d); %u still queued", (unsigned)s_head, code, (unsigned)s_count);
  }
}

void spoolStats(uint32_t& count, uint32_t& bytes) { count = s_count; bytes = s_bytes; }

void spoolClear() {
  if (!ensureMounted()) { s_head = s_nextSeq = s_count = s_bytes = 0; return; }
  File dir = LittleFS.open(SDIR);
  if (dir && dir.isDirectory()) {
    for (File f = dir.openNextFile(); f; f = dir.openNextFile()) {
      uint32_t seq; int kind;
      bool ok = parseName(f.name(), &seq, &kind);
      f.close();
      if (!ok) continue;
      char p[40];
      if (kind == 0)      { pathBin(seq, p, sizeof(p)); LittleFS.remove(p); }
      else if (kind == 1) { pathTmp(seq, p, sizeof(p)); LittleFS.remove(p); }
    }
  }
  if (dir) dir.close();
  s_head = s_nextSeq = s_count = s_bytes = 0;
  log_i("spool: cleared");
}
