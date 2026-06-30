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
  for (int i = 0; i < MAX_CELLS; i++) {
    _committed[i]    = false;
    _lastRaw[i]      = false;
    _stableCnt[i]    = 0;
    _baselineEdge[i] = 0.0f;
    _baselineInit[i] = false;
    _lastEdge[i]     = 0.0f;
  }
  _reportedSpaces = -1;
  _pendingSpaces  = -1;
  _pendingCnt     = 0;
}

void CvEngine::recalibrate(int index) {
  if (index < 0) { reset(); return; }       // all cells
  if (index >= MAX_CELLS) return;            // out of range -> no-op
  _committed[index]    = false;
  _lastRaw[index]      = false;
  _stableCnt[index]    = 0;
  _baselineEdge[index] = 0.0f;
  _baselineInit[index] = false;             // re-seeds from the next frame (relative mode)
}

void CvEngine::markOccupied(int index) {
  if (index < 0 || index >= MAX_CELLS) return;   // single cell only (no "all occupied")
  _committed[index] = true;
  _lastRaw[index]   = true;
  _stableCnt[index] = 0;
  if (_cfg && _cfg->occupancyMode == OCCUPANCY_RELATIVE) {
    float hys   = constrain(_cfg->hysteresis, 0.0f, 0.9f);
    float delta = (_cfg->relDelta > 0.0f) ? _cfg->relDelta : 1.0f;
    float enter = delta * (1.0f + hys);
    // Put the empty reference one entry-band below the cell's current edge, so
    // the live metric (edge - baseline) sits right at the entry level: the cell
    // reads occupied now, yet drops to empty (and the EMA relearns the true
    // baseline) once the space clears and the edge falls below this reference.
    float base = _lastEdge[index] - enter;
    _baselineEdge[index] = base > 0.0f ? base : 0.0f;
    _baselineInit[index] = true;
  }
}

void CvEngine::snapshotState(CurbPersist& o) const {
  o.magic     = CURB_PERSIST_MAGIC;
  o.cellCount = (uint16_t)(_cfg ? _cfg->cellCount : 0);
  o.geomSig   = roiSignature();
  for (int i = 0; i < MAX_CELLS; i++) {
    o.baselineEdge[i] = _baselineEdge[i];
    o.baselineInit[i] = _baselineInit[i] ? 1 : 0;
    o.committed[i]    = _committed[i] ? 1 : 0;
  }
  o.carPitchLearned = 0.0f;   // placeholder for Task C4.1
  o.learnSamples    = 0;      // placeholder for Task C4.1
}

bool CvEngine::restoreState(const CurbPersist& in) {
  if (in.magic != CURB_PERSIST_MAGIC) return false;
  if (in.geomSig != roiSignature())   return false;   // geometry changed -> ignore, seed live
  for (int i = 0; i < MAX_CELLS; i++) {
    _baselineEdge[i] = in.baselineEdge[i];
    _baselineInit[i] = in.baselineInit[i] != 0;
    _committed[i]    = in.committed[i] != 0;
    _lastRaw[i]      = _committed[i];                 // align debounce with the restored commit
    _stableCnt[i]    = 0;
    _lastEdge[i]     = _baselineEdge[i];              // plausible until the first analyze() runs
  }
  // Adopt the current signature so the first analyze() doesn't see a "changed
  // geometry" (member starts 0) and reset() away everything we just restored.
  _roiSig = roiSignature();
  return true;
}

uint32_t CvEngine::roiSignature() const {
  // Cheap hash of cell geometry/count so we can reset state when they change.
  uint32_t h = 2166136261u;
  auto mix = [&](uint32_t v) { h ^= v; h *= 16777619u; };
  if (!_cfg) return h;
  mix((uint32_t)_cfg->cellCount);
  for (int i = 0; i < _cfg->cellCount && i < MAX_CELLS; i++) {
    const CurbCell& c = _cfg->cells[i];
    for (int j = 0; j < 4; j++) {
      mix((uint32_t)(c.px[j] * 1000));
      mix((uint32_t)(c.py[j] * 1000));
    }
    // NOTE: `enabled` is deliberately NOT mixed in for the same reason as before:
    // a dead-zone toggle should not drop all cells' adaptive baselines.
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

bool CvEngine::analyze(const uint8_t* jpg, size_t len, int srcW, int srcH, CurbResult& out) {
  uint32_t t0 = millis();
  out.valid = false;
  out.nCells = 0;
  out.est_free_spaces    = 0;     // TODO(T6-T10): full headline computation in Task C3.1
  out.free_curb_m        = 0.0f;
  out.longest_free_run_m = 0.0f;
  out.can_fit            = false;
  out.reliable_range_m   = 0.0f;
  out.occupied_fraction  = 0.0f;
  out.dark               = false;
  if (!_cfg || srcW <= 0 || srcH <= 0) return false;

  // Skip malformed/truncated frames (missing JPEG SOI/EOI markers) without
  // invoking the decoder — the occasional bad sensor frame would otherwise log
  // "JPG Decompression Failed". These are harmless; we just skip the cycle.
  if (len < 4 || jpg[0] != 0xFF || jpg[1] != 0xD8 ||
      jpg[len - 2] != 0xFF || jpg[len - 1] != 0xD9) {
    return false;
  }

  // Reset per-cell state if the geometry changed.
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
  const float hys = constrain(_cfg->hysteresis, 0.0f, 0.9f);  // keep exit band positive
  const uint8_t stableNeed = _cfg->stableFrames ? _cfg->stableFrames : 1;
  const bool  relative = (_cfg->occupancyMode == OCCUPANCY_RELATIVE);
  const float relDelta = (_cfg->relDelta > 0.0f) ? _cfg->relDelta : 1.0f;  // floor so the band can't collapse

  int nCells = (_cfg->cellCount < MAX_CELLS) ? _cfg->cellCount : MAX_CELLS;
  out.nCells = nCells;

  // Per-cell detection loop: reuses the ray-cast accumulator + featuresFinalize +
  // relative-edge/hysteresis/debounce decision primitives from the bay-era engine.
  // Full per-cell pipeline (spatial smoothing, headline aggregation) is Task C2.1/C3.1.
  for (int i = 0; i < nCells; i++) {
    const CurbCell& cell = _cfg->cells[i];
    CellResult& cellRes  = out.cells[i];

    // CurbCell always has 4 quad vertices (no variable nPoints like the old Roi).
    const int np = 4;
    float vx[4], vy[4];
    float minx = 1e9f, miny = 1e9f, maxx = -1e9f, maxy = -1e9f;
    for (int j = 0; j < np; j++) {
      vx[j] = cell.px[j] * w; vy[j] = cell.py[j] * h;
      if (vx[j] < minx) minx = vx[j]; if (vx[j] > maxx) maxx = vx[j];
      if (vy[j] < miny) miny = vy[j]; if (vy[j] > maxy) maxy = vy[j];
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
        // point-in-polygon (ray casting) for the 4-vertex quad
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
          uint8_t c_lum = (uint8_t)lum;
          uint8_t code =
            ((r0[x - 1] >= c_lum) << 7) | ((r0[x] >= c_lum) << 6) | ((r0[x + 1] >= c_lum) << 5) |
            ((row[x + 1] >= c_lum) << 4) | ((nxt[x + 1] >= c_lum) << 3) | ((nxt[x] >= c_lum) << 2) |
            ((nxt[x - 1] >= c_lum) << 1) | ((row[x - 1] >= c_lum) << 0);
          uint8_t rot = (uint8_t)((code << 1) | (code >> 7));
          int trans = __builtin_popcount((unsigned)(code ^ rot));
          if (trans <= 2) fa.lbp[__builtin_popcount((unsigned)code)]++;  // uniform -> bin by set-bits 0..8
          else            fa.lbp[9]++;                                    // non-uniform
        }
        // colour from the matching RGB565 pixel
        uint16_t cpx = px[y * w + x];
        int r5 = ((cpx >> 11) & 0x1F) << 3, g6 = ((cpx >> 5) & 0x3F) << 2, b5 = (cpx & 0x1F) << 3;
        int mx = r5 > g6 ? (r5 > b5 ? r5 : b5) : (g6 > b5 ? g6 : b5);
        int mn = r5 < g6 ? (r5 < b5 ? r5 : b5) : (g6 < b5 ? g6 : b5);
        fa.satSum_x1000 += mx ? (int64_t)(mx - mn) * 1000 / mx : 0;
        fa.brSum += (b5 - r5);
      }
    }
    float edge  = fa.cnt ? (float)fa.gradSum / (float)fa.cnt : 0.0f;
    float meanI = fa.cnt ? (float)fa.intSum  / (float)fa.cnt : 0.0f;
    cellRes.edge = edge; cellRes.meanI = meanI;
    _lastEdge[i] = edge;   // remembered for markOccupied()'s re-base math
    featuresFinalize(fa, _baselineEdge[i], cellRes.feat);
    cellRes.clfScore = -1.0f;
    cellRes.inRange  = cell.enabled;

    if (!cell.enabled) {
      // Keep geometry but do not count; report instantaneous values only.
      cellRes.rawOccupied = false;
      cellRes.occupied    = false;
      cellRes.baselineEdge = _baselineEdge[i];
      cellRes.inRange      = false;
      continue;
    }

    // In relative mode, seed the empty baseline on the very first frame so the
    // first decision compares against a real reference, not zero (which would
    // read as instantly occupied).
    if (relative && !_baselineInit[i]) { _baselineEdge[i] = edge; _baselineInit[i] = true; }

    // Classifier score whenever a model is embedded (cheap: a few ops).
    cellRes.clfScore = clfAvailable() ? clfScore(cellRes.feat) : -1.0f;

    // Effective threshold (no per-cell override in curb model; global only).
    float thr = relative ? relDelta : globalThr;

    bool raw;
    if (_cfg->occupancyEngine == 1 && cellRes.clfScore >= 0.0f) {
      // Probability hysteresis around 0.5 (same `hys` fraction as the edge band).
      float p = cellRes.clfScore;
      float pen = 0.5f + hys * 0.5f;   // enter-occupied threshold
      float pex = 0.5f - hys * 0.5f;   // exit-occupied threshold
      raw = _committed[i] ? (p >= pex) : (p > pen);
    } else {
      // Legacy edge engine: absolute edge, or its rise above the empty baseline.
      float metric = relative ? (edge - _baselineEdge[i]) : edge;
      float enter  = thr * (1.0f + hys);
      float exitT  = thr * (1.0f - hys);
      raw = _committed[i] ? (metric >= exitT) : (metric > enter);
    }
    cellRes.rawOccupied = raw;

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

    // Adaptive empty-edge baseline: track while committed-empty and stable,
    // freeze while occupied. In relative mode this is the live reference the
    // decision subtracts (ambient light drift cancels out).
    if (!_committed[i] && _stableCnt[i] == 0) {
      if (!_baselineInit[i]) { _baselineEdge[i] = edge; _baselineInit[i] = true; }
      else _baselineEdge[i] += _cfg->baselineEma * (edge - _baselineEdge[i]);
    }
    cellRes.baselineEdge = _baselineEdge[i];
    cellRes.occupied     = _committed[i];
  }

  // ── T5: spatial-median + free-gap reducer + headline + light gate + stability ──

  // 1. Width-3 spatial median per strip (enabled cells only), when smoothMode == 1.
  //    Reads pre-smooth labels from a stack snapshot so neighbours are unaffected
  //    by earlier writes in the same pass.  Does NOT touch _committed[] — temporal
  //    debounce state stays the pre-smooth per-cell value for next-frame continuity.
  if (_cfg->smoothMode == 1) {
    for (int si = 0; si < _cfg->stripCount && si < MAX_STRIPS; si++) {
      int  idx[MAX_CELLS]; int ni = 0;
      for (int i = 0; i < nCells; i++) {
        if (_cfg->cells[i].strip == (uint8_t)si && _cfg->cells[i].enabled)
          idx[ni++] = i;
      }
      if (ni < 2) continue;
      bool snap[MAX_CELLS];
      for (int k = 0; k < ni; k++) snap[k] = out.cells[idx[k]].occupied;
      for (int k = 0; k < ni; k++) {
        bool lo = snap[(k > 0)    ? k - 1 : 0];
        bool me = snap[k];
        bool hi = snap[(k < ni-1) ? k + 1 : ni - 1];
        out.cells[idx[k]].occupied = ((lo ? 1 : 0) + (me ? 1 : 0) + (hi ? 1 : 0)) >= 2;
      }
    }
  }

  // 2 + 3. Free-gap run-length + aggregated headline scalars.
  float free_curb_m        = 0.0f;
  float longest_free_run_m = 0.0f;
  float reliable_range_m   = 0.0f;
  float enabled_len        = 0.0f;
  float occupied_len       = 0.0f;
  float luma_sum           = 0.0f;
  int   luma_cnt           = 0;
  int   raw_spaces         = 0;
  const float pitch = (_cfg->carPitchM > 0.0f) ? _cfg->carPitchM : 6.0f;

  // Single pass over all cells: reliable_range_m, fraction accumulators, luma sum.
  for (int i = 0; i < nCells; i++) {
    reliable_range_m += _cfg->cells[i].lenM;
    if (_cfg->cells[i].enabled) {
      enabled_len += _cfg->cells[i].lenM;
      if (out.cells[i].occupied) occupied_len += _cfg->cells[i].lenM;
      luma_sum += out.cells[i].meanI;
      luma_cnt++;
    }
  }

  // Per-strip free-gap run scan.
  for (int si = 0; si < _cfg->stripCount && si < MAX_STRIPS; si++) {
    int idx[MAX_CELLS]; int ni = 0;
    for (int i = 0; i < nCells; i++) {
      if (_cfg->cells[i].strip == (uint8_t)si)
        idx[ni++] = i;
    }
    bool  in_run    = false;
    float run_m     = 0.0f;
    bool  run_front = false;   // run started at k==0 (flush against strip's front boundary)
    for (int k = 0; k < ni; k++) {
      int ci = idx[k];
      bool free_cell = _cfg->cells[ci].enabled && !out.cells[ci].occupied;
      if (free_cell) {
        if (!in_run) { in_run = true; run_m = 0.0f; run_front = (k == 0); }
        run_m += _cfg->cells[ci].lenM;
      } else {
        if (in_run) {
          // Closed by occupied/disabled cell: run does NOT touch strip's last cell.
          // Terminal only if it started at k==0 (flush against front boundary).
          float clr = run_front ? _cfg->clearEndM : _cfg->clearInteriorM;
          free_curb_m += run_m;
          if (run_m > longest_free_run_m) longest_free_run_m = run_m;
          if (run_m >= pitch) {
            int n = (int)floorf((run_m - clr) / pitch);
            raw_spaces += (n > 0 ? n : 0);
          }
          in_run = false; run_m = 0.0f;
        }
      }
    }
    if (in_run) {
      // Run exhausted all strip cells → terminal at the back boundary (clearEndM).
      free_curb_m += run_m;
      if (run_m > longest_free_run_m) longest_free_run_m = run_m;
      if (run_m >= pitch) {
        int n = (int)floorf((run_m - _cfg->clearEndM) / pitch);
        raw_spaces += (n > 0 ? n : 0);
      }
    }
  }

  // 4. Light gate: trip dark flag when mean enabled-cell luma is low.
  float meanLuma = luma_cnt ? (luma_sum / (float)luma_cnt) : 0.0f;
  out.dark = (meanLuma < _cfg->darkLumaThresh);

  // 5. Reported-integer stability hold (reuses stableNeed from the per-cell loop).
  //    Keeps est_free_spaces from flapping ±1; biases to UNDER-count (safe side).
  if (raw_spaces == _reportedSpaces) {
    _pendingCnt = 0;
  } else if (raw_spaces == _pendingSpaces) {
    if (_pendingCnt < 0xFF) _pendingCnt++;
    if (_pendingCnt >= stableNeed) {
      _reportedSpaces = raw_spaces;
      _pendingCnt     = 0;
    }
  } else {
    _pendingSpaces = raw_spaces;
    _pendingCnt    = 1;
  }

  // Headline aggregation.
  out.free_curb_m        = free_curb_m;
  out.longest_free_run_m = longest_free_run_m;
  out.reliable_range_m   = reliable_range_m;
  out.can_fit            = (longest_free_run_m >= (pitch - _cfg->clearInteriorM));
  out.occupied_fraction  = (enabled_len > 0.0f) ? (occupied_len / enabled_len) : 0.0f;
  out.est_free_spaces    = (_reportedSpaces >= 0) ? _reportedSpaces : raw_spaces;

  out.tookMs = millis() - t0;
  out.valid  = true;
  return true;
}
