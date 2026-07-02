#include "capture.h"
#include "net.h"
#include "clk.h"
#include "features.h"
#include "logbuf.h"

static uint32_t s_lastCapMs = 0;
static const uint32_t CAPTURE_MIN_INTERVAL_MS  = 30000;  // baseline throttle: ~one sample / 30s
static const uint32_t CAPTURE_HARD_INTERVAL_MS = 5000;   // faster when the engines disagree (hard cases)

void captureMaybeLog(const Config& cfg, const CurbResult& r) {
  if (!cfg.trainCapture || !cfg.captureUrl[0] || !r.valid) return;
  uint32_t now = millis();

  // Hard case: at least one enabled cell where the classifier and the edge engine
  // disagree. These are the frames worth labelling; capture them at a faster cadence
  // so the hand-verified hard-case dataset the trainer reports on fills up quicker.
  int disagreements = 0;
  for (int i = 0; i < r.nCells && i < MAX_CELLS; i++)
    if (cfg.cells[i].enabled && r.cells[i].clfDisagree) disagreements++;
  const bool hard = disagreements > 0;

  uint32_t interval = hard ? CAPTURE_HARD_INTERVAL_MS : CAPTURE_MIN_INTERVAL_MS;
  if (s_lastCapMs != 0 && (now - s_lastCapMs) < interval) return;
  s_lastCapMs = now;

  String body; body.reserve(256 + (size_t)r.nCells * 220);
  body += "{\"device\":\""; body += cfg.hostname; body += "\",";
  body += "\"ts\":"; body += String((uint32_t)clockEpoch()); body += ",";
  body += "\"hard\":"; body += (hard ? 1 : 0); body += ",";   // disagreement-triggered frame
  body += "\"cells\":[";
  bool first = true;
  for (int i = 0; i < r.nCells && i < MAX_CELLS; i++) {
    if (!cfg.cells[i].enabled) continue;   // skip disabled cells (would train as mislabeled-empty)
    const CellResult& s = r.cells[i];
    if (!first) body += ","; first = false;
    body += "{\"i\":"; body += i;
    body += ",\"label\":"; body += (s.occupied ? 1 : 0);          // weak label = committed decision
    body += ",\"score\":"; body += String(s.clfScore, 3);
    body += ",\"d\":"; body += (s.clfDisagree ? 1 : 0);           // per-cell classifier/edge disagreement
    body += ",\"f\":[";
    for (int k = 0; k < CLF_NFEAT; k++) { if (k) body += ","; body += String(s.feat[k], 4); }
    body += "]}";
  }
  body += "]}";

  int code = netPostJson(cfg.captureUrl, cfg.captureAuthHeaderName, cfg.captureAuthHeaderValue,
                         cfg.captureTlsInsecure, body);
  log_i("capture POST -> HTTP %d (%u cells, %s)", code, (unsigned)r.nCells, hard ? "hard" : "routine");
}
