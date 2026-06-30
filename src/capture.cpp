#include "capture.h"
#include "net.h"
#include "clk.h"
#include "features.h"
#include "logbuf.h"

static uint32_t s_lastCapMs = 0;
static const uint32_t CAPTURE_MIN_INTERVAL_MS = 30000;  // throttle: ~one sample / 30s (netPostJson blocks)

void captureMaybeLog(const Config& cfg, const CurbResult& r) {
  if (!cfg.trainCapture || !cfg.captureUrl[0] || !r.valid) return;
  uint32_t now = millis();
  if (s_lastCapMs != 0 && (now - s_lastCapMs) < CAPTURE_MIN_INTERVAL_MS) return;
  s_lastCapMs = now;

  String body; body.reserve(256 + (size_t)r.nCells * 220);
  body += "{\"device\":\""; body += cfg.hostname; body += "\",";
  body += "\"ts\":"; body += String((uint32_t)clockEpoch()); body += ",";
  body += "\"cells\":[";
  bool first = true;
  for (int i = 0; i < r.nCells && i < MAX_CELLS; i++) {
    if (!cfg.cells[i].enabled) continue;   // skip disabled cells (would train as mislabeled-empty)
    const CellResult& s = r.cells[i];
    if (!first) body += ","; first = false;
    body += "{\"i\":"; body += i;
    body += ",\"label\":"; body += (s.occupied ? 1 : 0);          // weak label = committed decision
    body += ",\"score\":"; body += String(s.clfScore, 3);
    body += ",\"f\":[";
    for (int k = 0; k < CLF_NFEAT; k++) { if (k) body += ","; body += String(s.feat[k], 4); }
    body += "]}";
  }
  body += "]}";

  int code = netPostJson(cfg.captureUrl, cfg.captureAuthHeaderName, cfg.captureAuthHeaderValue,
                         true /*tlsInsecure*/, body);
  log_i("capture POST -> HTTP %d (%u cells)", code, (unsigned)r.nCells);
}
