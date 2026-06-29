#include "cv.h"
#include "features.h"
#include "clf.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_heap_caps.h"

void CvEngine::begin(const Config* cfg) {
  _cfg = cfg;
  reset();
}

void CvEngine::reset() {
  for (int i = 0; i < MAX_ROIS; i++) {
    _committed[i]    = false;
    _lastRaw[i]      = false;
    _stableCnt[i]    = 0;
    _baselineEdge[i] = 0.0f;
    _baselineInit[i] = false;
    _lastEdge[i]     = 0.0f;
  }
}

void CvEngine::recalibrate(int index) {
  if (index < 0) { reset(); return; }      // all bays
  if (index >= MAX_ROIS) return;           // out of range -> no-op
  _committed[index]    = false;
  _lastRaw[index]      = false;
  _stableCnt[index]    = 0;
  _baselineEdge[index] = 0.0f;
  _baselineInit[index] = false;            // re-seeds from the next frame (relative mode)
}

void CvEngine::markOccupied(int index) {
  if (index < 0 || index >= MAX_ROIS) return;   // single bay only (no "all occupied")
  _committed[index] = true;
  _lastRaw[index]   = true;
  _stableCnt[index] = 0;
  if (_cfg && _cfg->occupancyMode == OCCUPANCY_RELATIVE) {
    float hys   = constrain(_cfg->hysteresis, 0.0f, 0.9f);
    float delta = (_cfg->relDelta > 0.0f) ? _cfg->relDelta : 1.0f;
    float enter = delta * (1.0f + hys);
    // Put the empty reference one entry-band below the bay's current edge, so the
    // live metric (edge - baseline) sits right at the entry level: the bay reads
    // occupied now, yet drops to empty (and the EMA relearns the true baseline)
    // once the car actually leaves and the edge falls below this reference.
    float base = _lastEdge[index] - enter;
    _baselineEdge[index] = base > 0.0f ? base : 0.0f;
    _baselineInit[index] = true;
  }
}

void CvEngine::snapshotState(CvPersist& o) const {
  o.magic    = CV_PERSIST_MAGIC;
  o.roiCount = (uint16_t)(_cfg ? _cfg->roiCount : 0);
  o.roiSig   = roiSignature();
  for (int i = 0; i < MAX_ROIS; i++) {
    o.baselineEdge[i] = _baselineEdge[i];
    o.baselineInit[i] = _baselineInit[i] ? 1 : 0;
    o.committed[i]    = _committed[i] ? 1 : 0;
  }
}

bool CvEngine::restoreState(const CvPersist& in) {
  if (in.magic != CV_PERSIST_MAGIC) return false;
  if (in.roiSig != roiSignature())  return false;   // ROI geometry changed -> ignore, seed live
  for (int i = 0; i < MAX_ROIS; i++) {
    _baselineEdge[i] = in.baselineEdge[i];
    _baselineInit[i] = in.baselineInit[i] != 0;
    _committed[i]    = in.committed[i] != 0;
    _lastRaw[i]      = _committed[i];                // align debounce with the restored commit
    _stableCnt[i]    = 0;
    _lastEdge[i]     = _baselineEdge[i];             // plausible until the first analyze() runs
  }
  // Adopt the current signature so the first analyze() doesn't see a "changed ROI
  // set" (member starts 0) and reset() away everything we just restored.
  _roiSig = roiSignature();
  return true;
}

uint32_t CvEngine::roiSignature() const {
  // Cheap hash of ROI geometry/count so we can reset state when they change.
  uint32_t h = 2166136261u;
  auto mix = [&](uint32_t v) { h ^= v; h *= 16777619u; };
  mix((uint32_t)_cfg->roiCount);
  for (int i = 0; i < _cfg->roiCount && i < MAX_ROIS; i++) {
    const Roi& r = _cfg->rois[i];
    mix((uint32_t)r.nPoints);
    for (int j = 0; j < r.nPoints && j < MAX_POLY; j++) {
      mix((uint32_t)(r.px[j] * 1000));
      mix((uint32_t)(r.py[j] * 1000));
    }
    // NOTE: `enabled` is deliberately NOT mixed in. It isn't geometry, and a
    // disabled bay keeps its array slot, so an enable/disable toggle leaves every
    // slot's index — and its per-slot state — valid. Hashing it here would reset
    // ALL bays' adaptive baselines on every toggle, which in relative mode makes
    // occupied bays briefly read empty during tuning.
  }
  return h;
}

bool CvEngine::ensureBuffers(int w, int h) {
  if (_luma && _rgb && w == _decW && h == _decH) return true;
  if (_luma) { heap_caps_free(_luma); _luma = nullptr; }
  if (_rgb)  { heap_caps_free(_rgb);  _rgb  = nullptr; }
  _decW = w; _decH = h;
  // A few extra rows of headroom: the JPEG decoder can round output dimensions
  // up to the MCU grid, so allocate slightly more than w*h to be safe.
  size_t px = (size_t)w * (h + 8);
  _rgb  = (uint8_t*)heap_caps_malloc(px * 2, MALLOC_CAP_SPIRAM);
  _luma = (uint8_t*)heap_caps_malloc(px,     MALLOC_CAP_SPIRAM);
  if (!_rgb || !_luma) {
    // fall back to internal RAM if PSRAM alloc failed (small frames only)
    if (!_rgb)  _rgb  = (uint8_t*)heap_caps_malloc(px * 2, MALLOC_CAP_8BIT);
    if (!_luma) _luma = (uint8_t*)heap_caps_malloc(px,     MALLOC_CAP_8BIT);
  }
  return _luma && _rgb;
}

static jpg_scale_t pickScale(int srcW) {
  if (srcW >= 1280) return JPG_SCALE_8X;   // 1280->160, 1600->200
  if (srcW >= 640)  return JPG_SCALE_4X;    // 640->160, 800->200, 1024->256
  if (srcW >= 320)  return JPG_SCALE_2X;    // 320->160, 480->240
  return JPG_SCALE_NONE;
}
static int scaleDiv(jpg_scale_t s) {
  switch (s) { case JPG_SCALE_8X: return 8; case JPG_SCALE_4X: return 4;
               case JPG_SCALE_2X: return 2; default: return 1; }
}

bool CvEngine::analyze(const uint8_t* jpg, size_t len, int srcW, int srcH, CvResult& out) {
  uint32_t t0 = millis();
  out.valid = false;
  out.count = 0;
  out.n = _cfg ? _cfg->roiCount : 0;
  if (!_cfg || srcW <= 0 || srcH <= 0) return false;

  // Skip malformed/truncated frames (missing JPEG SOI/EOI markers) without
  // invoking the decoder — the occasional bad sensor frame would otherwise log
  // "JPG Decompression Failed". These are harmless; we just skip the cycle.
  if (len < 4 || jpg[0] != 0xFF || jpg[1] != 0xD8 ||
      jpg[len - 2] != 0xFF || jpg[len - 1] != 0xD9) {
    return false;
  }

  // Reset per-slot state if the ROI set changed.
  uint32_t sig = roiSignature();
  if (sig != _roiSig) { reset(); _roiSig = sig; }

  jpg_scale_t scale = pickScale(srcW);
  int div = scaleDiv(scale);
  int w = srcW / div, h = srcH / div;
  if (!ensureBuffers(w, h)) return false;
  out.decW = w; out.decH = h;

  if (!jpg2rgb565(jpg, len, _rgb, scale)) return false;

  // RGB565 -> luma
  const uint16_t* px = reinterpret_cast<const uint16_t*>(_rgb);
  const int npx = w * h;
  for (int i = 0; i < npx; i++) {
    uint16_t v = px[i];
    int r = ((v >> 11) & 0x1F) << 3;
    int g = ((v >> 5)  & 0x3F) << 2;
    int b = ( v        & 0x1F) << 3;
    _luma[i] = (uint8_t)((r * 77 + g * 150 + b * 29) >> 8);
  }

  const float globalThr = _cfg->edgeThreshold;
  const float hys = constrain(_cfg->hysteresis, 0.0f, 0.9f);  // keep the exit band positive (>=1.0 would latch occupied)
  const uint8_t stableNeed = _cfg->stableFrames ? _cfg->stableFrames : 1;
  const bool  relative = (_cfg->occupancyMode == OCCUPANCY_RELATIVE);
  const float relDelta = (_cfg->relDelta > 0.0f) ? _cfg->relDelta : 1.0f;  // floor so the band can't collapse

  int count = 0;
  for (int i = 0; i < _cfg->roiCount && i < MAX_ROIS; i++) {
    const Roi& roi = _cfg->rois[i];
    SlotResult& sr = out.slots[i];
    // Effective threshold/delta (also shown in the UI). In relative mode every
    // bay uses the global delta against its own baseline; the per-ROI absolute
    // override applies only in absolute mode.
    sr.threshold = relative ? relDelta : ((roi.threshold > 0.0f) ? roi.threshold : globalThr);

    // Polygon vertices in pixel space + bounding box.
    float vx[MAX_POLY], vy[MAX_POLY];
    int np = roi.nPoints < MAX_POLY ? roi.nPoints : MAX_POLY;
    float minx = 1e9f, miny = 1e9f, maxx = -1e9f, maxy = -1e9f;
    for (int j = 0; j < np; j++) {
      vx[j] = roi.px[j] * w; vy[j] = roi.py[j] * h;
      if (vx[j] < minx) minx = vx[j];  if (vx[j] > maxx) maxx = vx[j];
      if (vy[j] < miny) miny = vy[j];  if (vy[j] > maxy) maxy = vy[j];
    }
    int x0 = constrain((int)floorf(minx), 0, w - 2);
    int y0 = constrain((int)floorf(miny), 0, h - 2);
    int x1 = constrain((int)ceilf(maxx),  x0 + 1, w - 1);
    int y1 = constrain((int)ceilf(maxy),  y0 + 1, h - 1);

    FeatureAccum fa; featureAccumInit(fa);
    for (int y = y0; y < y1; y++) {
      const uint8_t* row = &_luma[y * w];
      const uint8_t* nxt = &_luma[(y + 1) * w];
      for (int x = x0; x < x1; x++) {
        // point-in-polygon (ray casting)
        bool inside = false;
        for (int a = 0, b = np - 1; a < np; b = a++) {
          if (((vy[a] > y) != (vy[b] > y)) &&
              ((float)x < (vx[b] - vx[a]) * ((float)y - vy[a]) / (vy[b] - vy[a]) + vx[a]))
            inside = !inside;
        }
        if (!inside) continue;
        int lum = row[x];
        int gx = abs((int)row[x + 1] - lum);
        int gy = abs((int)nxt[x]     - lum);
        fa.gradSum  += (gx + gy);
        fa.intSum   += lum;
        fa.intSqSum += (uint32_t)lum * (uint32_t)lum;
        fa.cnt++;
        // uniform LBP(8,1): compare 8 neighbours to centre (guard image borders)
        if (x > 0 && x < w - 1 && y > 0 && y < h - 1) {
          const uint8_t* r0 = &_luma[(y - 1) * w];
          uint8_t c = (uint8_t)lum;
          uint8_t code =
            ((r0[x - 1] >= c) << 7) | ((r0[x] >= c) << 6) | ((r0[x + 1] >= c) << 5) |
            ((row[x + 1] >= c) << 4) | ((nxt[x + 1] >= c) << 3) | ((nxt[x] >= c) << 2) |
            ((nxt[x - 1] >= c) << 1) | ((row[x - 1] >= c) << 0);
          uint8_t rot = (uint8_t)((code << 1) | (code >> 7));
          int trans = __builtin_popcount((unsigned)(code ^ rot));
          if (trans <= 2) fa.lbp[__builtin_popcount((unsigned)code)]++;  // uniform -> bin by set-bits 0..8
          else            fa.lbp[9]++;                                    // non-uniform
        }
        // colour from the matching RGB565 pixel
        uint16_t cpx = px[y * w + x];
        int cr = ((cpx >> 11) & 0x1F) << 3, cg = ((cpx >> 5) & 0x3F) << 2, cb = (cpx & 0x1F) << 3;
        int mx = cr > cg ? (cr > cb ? cr : cb) : (cg > cb ? cg : cb);
        int mn = cr < cg ? (cr < cb ? cr : cb) : (cg < cb ? cg : cb);
        fa.satSum_x1000 += mx ? (int64_t)(mx - mn) * 1000 / mx : 0;
        fa.brSum += (cb - cr);
      }
    }
    float edge  = fa.cnt ? (float)fa.gradSum / (float)fa.cnt : 0.0f;
    float meanI = fa.cnt ? (float)fa.intSum  / (float)fa.cnt : 0.0f;
    sr.edge = edge; sr.meanI = meanI;
    _lastEdge[i] = edge;   // remembered for markOccupied()'s re-base math
    featuresFinalize(fa, _baselineEdge[i], sr.feat);
    sr.clfScore = -1.0f;

    if (!roi.enabled) {
      // Keep geometry but do not count; report instantaneous values only.
      sr.rawOccupied = false; sr.occupied = false; sr.baselineEdge = _baselineEdge[i];
      continue;
    }

    // In relative mode, seed the empty baseline on the very first frame so the
    // first decision compares against a real reference, not zero (which would
    // read as instantly occupied). A bay genuinely occupied at boot (or the first
    // time it is enabled) then seeds high and reads empty until its first
    // departure — a deliberate trade for lighting robustness; absolute mode is
    // correct from boot if that matters.
    if (relative && !_baselineInit[i]) { _baselineEdge[i] = edge; _baselineInit[i] = true; }

    // Classifier score whenever a model is embedded — cheap (a few ops). It drives the
    // decision when the classifier engine is selected, and otherwise serves as a live
    // disagreement monitor against the edge engine in the UI.
    sr.clfScore = clfAvailable() ? clfScore(sr.feat) : -1.0f;

    bool raw;
    if (_cfg->occupancyEngine == 1 && sr.clfScore >= 0.0f) {
      // Probability hysteresis around 0.5 (same `hys` fraction as the edge band).
      float p = sr.clfScore;
      float pen = 0.5f + hys * 0.5f;   // enter-occupied threshold
      float pex = 0.5f - hys * 0.5f;   // exit-occupied threshold
      raw = _committed[i] ? (p >= pex) : (p > pen);
    } else {
      // Legacy edge engine: absolute edge, or its rise above the empty baseline.
      float metric = relative ? (edge - _baselineEdge[i]) : edge;
      float enter = sr.threshold * (1.0f + hys);
      float exit  = sr.threshold * (1.0f - hys);
      raw = _committed[i] ? (metric >= exit) : (metric > enter);
    }
    sr.rawOccupied = raw;

    // Debounce: require the raw decision to persist before committing.
    if (raw == _committed[i]) {
      _stableCnt[i] = 0;
    } else {
      if (raw == _lastRaw[i]) {
        if (_stableCnt[i] < 0xFFFF) _stableCnt[i]++;
      } else {
        _stableCnt[i] = 1;
      }
      if (_stableCnt[i] >= stableNeed) {
        _committed[i] = raw;
        _stableCnt[i] = 0;
      }
    }
    _lastRaw[i] = raw;

    // Adaptive empty-edge baseline: track the edge while the bay is committed-
    // empty and stable, and freeze it while occupied so a long-parked car can't
    // pull the reference up. In relative mode this is the live reference the
    // decision subtracts (ambient light drift cancels out); in absolute mode it
    // is just a diagnostic / auto-threshold helper. It adapts in both modes, so
    // toggling to relative finds a warmed-up baseline ready to use.
    if (!_committed[i] && _stableCnt[i] == 0) {
      if (!_baselineInit[i]) { _baselineEdge[i] = edge; _baselineInit[i] = true; }
      else _baselineEdge[i] += _cfg->baselineEma * (edge - _baselineEdge[i]);
    }
    sr.baselineEdge = _baselineEdge[i];

    sr.occupied = _committed[i];
    if (_committed[i]) count++;
  }

  out.count = count;
  out.tookMs = millis() - t0;
  out.valid = true;
  return true;
}
