#include "cv.h"
#include "features.h"
#include "clf.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_heap_caps.h"

// Auto-learn car-pitch refiner tuning (Task C4.1). Named here so the pipeline's
// tunable constants are discoverable in one place instead of inline literals.
static const float    PITCH_BAND_MIN_M     = 3.0f;   // reject noise / motorcycles below this footprint
static const float    PITCH_BAND_MAX_M     = 6.5f;   // reject multi-car blobs above this footprint
static const float    PITCH_LEARN_ALPHA    = 0.05f;  // slow EMA rate toward observed footprint
static const uint16_t PITCH_LEARN_MIN_K    = 30;     // samples before the learned pitch is trusted
static const float    DEFAULT_CAR_PITCH_M  = 6.0f;   // fallback when carPitchM is unset

static_assert(curb_reduce::RD_MAX_CELLS  == MAX_CELLS,  "curb_reduce cell cap must match config_store");
static_assert(curb_reduce::RD_MAX_STRIPS == MAX_STRIPS, "curb_reduce strip cap must match config_store");

// Camera-moved detector tuning.
static const float   CAM_MOVE_DELTA  = 0.5f;    // occupied-fraction jump vs the slow reference
static const uint8_t CAM_MOVE_FRAMES = 8;       // sustained frames before flagging
static const float   CAM_REF_ALPHA   = 0.01f;   // slow reference drift while the scene is stable

void CvEngine::begin(const Config* cfg) {
  _cfg = cfg;
  // Initialise learning state here (not in reset): a geometry/debounce reset must
  // not wipe slowly-accumulated pitch observations.
  _carPitchLearned = 0.0f;
  _learnSamples    = 0;
  for (int i = 0; i < MAX_CELLS; i++) _learnEpisodeActive[i] = false;
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
  _occFracRef  = 0.0f;
  _occRefInit  = false;
  _moveCnt     = 0;
  _cameraMoved = false;
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
  o.carPitchLearned = _carPitchLearned;   // Task C4.1
  o.learnSamples    = _learnSamples;      // Task C4.1
}

bool CvEngine::restoreState(const CurbPersist& in) {
  if (in.magic != CURB_PERSIST_MAGIC) return false;
  if (in.geomSig != roiSignature())   return false;   // geometry changed -> ignore, seed live
  for (int i = 0; i < MAX_CELLS; i++) {
    float b = in.baselineEdge[i];
    _baselineEdge[i] = isfinite(b) ? b : 0.0f;        // never restore a NaN/Inf baseline
    _baselineInit[i] = isfinite(b) && in.baselineInit[i] != 0;
    _committed[i]    = in.committed[i] != 0;
    _lastRaw[i]      = _committed[i];                 // align debounce with the restored commit
    _stableCnt[i]    = 0;
    _lastEdge[i]     = _baselineEdge[i];              // plausible until the first analyze() runs
    // A cell restored occupied is an already-counted parking episode: arm its
    // anti-double-count flag so the first post-reboot analyze() does not re-fold
    // the same parked car into the pitch EMA (learnSamples persists across reboots).
    _learnEpisodeActive[i] = _committed[i];
  }
  // Restore auto-learn state (Task C4.1). The geomSig gate above already guarantees
  // the stored values belong to the same ROI layout; a re-trace drops them correctly.
  _carPitchLearned = in.carPitchLearned;
  _learnSamples    = in.learnSamples;
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

// Stage 1: decode + downscale the JPEG to RGB565, then to a luma buffer.
bool CvEngine::decodeToLuma(const uint8_t* jpg, size_t len, int srcW, int srcH) {
  jpg_scale_t scale = pickScale(srcW);
  int div = scaleDiv(scale);
  int w = srcW / div, h = srcH / div;
  if (!ensureBuffers(w, h)) return false;          // sets _decW/_decH
  if (!jpg2rgb565(jpg, len, _rgb, scale)) return false;

  const uint16_t* px = reinterpret_cast<const uint16_t*>(_rgb);
  const int npx = w * h;
  for (int i = 0; i < npx; i++) {
    uint16_t v = px[i];
    int r = ((v >> 11) & 0x1F) << 3;
    int g = ((v >> 5)  & 0x3F) << 2;
    int b = ( v        & 0x1F) << 3;
    _luma[i] = (uint8_t)((r * 77 + g * 150 + b * 29) >> 8);
  }
  return true;
}

// Stage 2 (per cell): ray-cast accumulate the in-polygon features, finalize the
// feature vector, run the occupied/free decision (classifier or relative edge) with
// hysteresis + debounce, and update the adaptive empty baseline.
void CvEngine::analyzeCell(int i, const DecideParams& dp, CellResult& cellRes) {
  const int w = _decW, h = _decH;
  const uint16_t* px = reinterpret_cast<const uint16_t*>(_rgb);
  const CurbCell& cell = _cfg->cells[i];

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

  // In relative mode, seed the empty baseline on the very first frame (enabled
  // cells) BEFORE finalizing features, so the first decision compares against a
  // real reference (not zero, which would read as instantly occupied) AND feat[1]
  // (edge above baseline) reads ~0 on the seeding frame rather than the full edge
  // energy — otherwise the classifier/capture would see a spurious occupied vote.
  if (dp.relative && cell.enabled && !_baselineInit[i]) { _baselineEdge[i] = edge; _baselineInit[i] = true; _warmupLeft = CV_WARMUP_FRAMES; }

  featuresFinalize(fa, _baselineEdge[i], cellRes.feat);
  cellRes.clfScore = -1.0f;
  cellRes.inRange  = cell.enabled;

  if (!cell.enabled) {
    // Keep geometry but do not count; report instantaneous values only.
    cellRes.rawOccupied = false;
    cellRes.occupied    = false;
    cellRes.baselineEdge = _baselineEdge[i];
    cellRes.inRange      = false;
    return;
  }

  // Classifier score whenever a model is embedded (cheap: a few ops).
  cellRes.clfScore = clfAvailable() ? clfScore(cellRes.feat) : -1.0f;

  // Effective threshold (no per-cell override in curb model; global only).
  float thr = dp.relative ? dp.relDelta : dp.globalThr;

  bool raw;
  if (_cfg->occupancyEngine == 1 && cellRes.clfScore >= 0.0f) {
    // Probability hysteresis around 0.5 (same `hys` fraction as the edge band).
    float p = cellRes.clfScore;
    float pen = 0.5f + dp.hys * 0.5f;   // enter-occupied threshold
    float pex = 0.5f - dp.hys * 0.5f;   // exit-occupied threshold
    raw = _committed[i] ? (p >= pex) : (p > pen);
  } else {
    // Legacy edge engine: absolute edge, or its rise above the empty baseline.
    float metric = dp.relative ? (edge - _baselineEdge[i]) : edge;
    float enter  = thr * (1.0f + dp.hys);
    float exitT  = thr * (1.0f - dp.hys);
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
    if (_stableCnt[i] >= dp.stableNeed) {
      _committed[i] = raw;
      _stableCnt[i] = 0;
    }
  }
  _lastRaw[i] = raw;

  // Adaptive empty-edge baseline: track while committed-empty and stable,
  // freeze while occupied. In relative mode this is the live reference the
  // decision subtracts (ambient light drift cancels out).
  if (!_committed[i] && _stableCnt[i] == 0) {
    if (!_baselineInit[i]) { _baselineEdge[i] = edge; _baselineInit[i] = true; _warmupLeft = CV_WARMUP_FRAMES; }
    else _baselineEdge[i] += dp.emaRate * (edge - _baselineEdge[i]);
  }
  cellRes.baselineEdge = _baselineEdge[i];
  cellRes.occupied     = _committed[i];
}

bool CvEngine::analyze(const uint8_t* jpg, size_t len, int srcW, int srcH, CurbResult& out) {
  uint32_t t0 = millis();
  out.valid = false;
  out.nCells = 0;
  out.nStrips = 0;
  out.est_free_spaces    = 0;
  out.free_curb_m        = 0.0f;
  out.longest_free_run_m = 0.0f;
  out.can_fit            = false;
  out.reliable_range_m   = 0.0f;
  out.occupied_fraction  = 0.0f;
  out.dark               = false;
  out.warming            = false;
  out.camera_moved       = _cameraMoved;
  out.pitch_m            = 0.0f;
  out.pitch_learned_m    = _carPitchLearned;
  out.pitch_samples      = _learnSamples;
  out.pitch_disagree     = false;
  if (!_cfg || srcW <= 0 || srcH <= 0) return false;

  // Skip malformed/truncated frames (missing JPEG SOI/EOI markers) without
  // invoking the decoder — the occasional bad sensor frame would otherwise log
  // "JPG Decompression Failed". These are harmless; we just skip the cycle.
  if (len < 4 || jpg[0] != 0xFF || jpg[1] != 0xD8 ||
      jpg[len - 2] != 0xFF || jpg[len - 1] != 0xD9) {
    return false;
  }

  // Reset per-cell state if the geometry changed. A live re-trace also drops the
  // auto-learned pitch: it was learned under the OLD metric scale, and the NVS
  // geomSig guard (which gates the persisted learner) only fires across reboots —
  // clearing here keeps runtime behaviour consistent with that guard (spec §8).
  uint32_t sig = roiSignature();
  if (sig != _roiSig) {
    reset();
    _carPitchLearned = 0.0f;
    _learnSamples    = 0;
    for (int i = 0; i < MAX_CELLS; i++) _learnEpisodeActive[i] = false;
    _roiSig = sig;
  }

  // Stage 1: decode + downscale.
  if (!decodeToLuma(jpg, len, srcW, srcH)) return false;
  out.decW = _decW; out.decH = _decH;

  const DecideParams dp{
    _cfg->edgeThreshold,
    constrain(_cfg->hysteresis, 0.0f, 0.9f),                 // keep exit band positive
    (_cfg->relDelta > 0.0f) ? _cfg->relDelta : 1.0f,        // floor so the band can't collapse
    constrain(_cfg->baselineEma, 0.0f, 1.0f),               // |1-a|<=1 so the EMA can't diverge
    (_cfg->occupancyMode == OCCUPANCY_RELATIVE),
    (uint8_t)(_cfg->stableFrames ? _cfg->stableFrames : 1),
  };
  const uint8_t stableNeed = dp.stableNeed;

  int nCells = (_cfg->cellCount < MAX_CELLS) ? _cfg->cellCount : MAX_CELLS;
  out.nCells = nCells;

  // Stage 2: per-cell features + occupied/free decision + debounce + baseline.
  for (int i = 0; i < nCells; i++) analyzeCell(i, dp, out.cells[i]);

  // ── Stage 3: spatial-median + free-gap reducer + headline + light gate + stability ──
  // The geometry math now lives in the pure, host-testable curb_reduce.h. Fill its
  // cell view from the config + per-cell decisions and drive the stages from there.

  const int nStrips = (_cfg->stripCount < MAX_STRIPS) ? _cfg->stripCount : MAX_STRIPS;
  for (int i = 0; i < nCells; i++) {
    _rc[i].occupied = out.cells[i].occupied;
    _rc[i].enabled  = _cfg->cells[i].enabled;
    _rc[i].strip    = _cfg->cells[i].strip;
    _rc[i].lenM     = _cfg->cells[i].lenM;
  }

  // 1. Spatial median (smoothMode==1); reflect the smoothed labels back into the
  //    result so the overlay and /api/state show them.  _committed[] is untouched,
  //    so temporal debounce keeps the pre-smooth value for next-frame continuity.
  if (_cfg->smoothMode == 1) {
    curb_reduce::spatialMedian(_rc, nCells, nStrips);
    for (int i = 0; i < nCells; i++) out.cells[i].occupied = _rc[i].occupied;
  }

  // Light-gate luma sum needs per-cell meanI, which is a CV feature, not geometry.
  float luma_sum = 0.0f; int luma_cnt = 0;
  for (int i = 0; i < nCells; i++) {
    if (_cfg->cells[i].enabled) { luma_sum += out.cells[i].meanI; luma_cnt++; }
  }

  // 2. Auto-learn car-pitch refiner (folds one isolated-car observation per episode).
  if (_cfg->pitchLearn) {
    curb_reduce::PitchLearnState ls;
    ls.carPitchLearned = _carPitchLearned;
    ls.learnSamples    = _learnSamples;
    for (int i = 0; i < MAX_CELLS; i++) ls.episodeActive[i] = _learnEpisodeActive[i];
    curb_reduce::PitchLearnParams lp{ _cfg->clearInteriorM, PITCH_BAND_MIN_M, PITCH_BAND_MAX_M,
                                      PITCH_LEARN_ALPHA, 0xFFFF };
    curb_reduce::pitchLearn(_rc, nCells, nStrips, lp, ls);
    _carPitchLearned = ls.carPitchLearned;
    _learnSamples    = ls.learnSamples;
    for (int i = 0; i < MAX_CELLS; i++) _learnEpisodeActive[i] = ls.episodeActive[i];
  }

  // Pitch: learned when gated (pitchLearn on, >= K samples, valid), else configured.
  const float cfgPitch = (_cfg->carPitchM > 0.0f ? _cfg->carPitchM : DEFAULT_CAR_PITCH_M);
  const bool  pitchGated = (_cfg->pitchLearn && _learnSamples >= PITCH_LEARN_MIN_K && _carPitchLearned > 0.0f);
  const float pitch = pitchGated ? _carPitchLearned : cfgPitch;
  out.pitch_m         = pitch;
  out.pitch_learned_m = _carPitchLearned;
  out.pitch_samples   = _learnSamples;
  out.pitch_disagree  = pitchGated && (fabsf(_carPitchLearned - cfgPitch) > 0.15f * cfgPitch);

  // 3. Free-gap run-length + aggregated (and per-strip) headline scalars.
  curb_reduce::Params   rp{ pitch, _cfg->clearInteriorM, _cfg->clearEndM };
  curb_reduce::Aggregate ag;
  curb_reduce::aggregate(_rc, nCells, nStrips, rp, ag);

  // 4. Light gate: trip dark flag when mean enabled-cell luma is low.
  float meanLuma = luma_cnt ? (luma_sum / (float)luma_cnt) : 0.0f;
  out.dark = (luma_cnt > 0 && meanLuma < _cfg->darkLumaThresh);

  // 4b. Warm-up gate: flag low-confidence while a freshly-seeded baseline settles.
  out.warming = (_warmupLeft > 0);
  if (_warmupLeft > 0) _warmupLeft--;

  // 5. Integer stability hold: keep est_free_spaces from flapping +/-1 (under-count safe).
  curb_reduce::SpacesHold hold{ _reportedSpaces, _pendingSpaces, _pendingCnt };
  int reported = curb_reduce::spacesHold(ag.raw_spaces, stableNeed, hold);
  _reportedSpaces = hold.reported; _pendingSpaces = hold.pending; _pendingCnt = hold.cnt;

  // Headline aggregation.
  out.free_curb_m        = ag.free_curb_m;
  out.longest_free_run_m = ag.longest_free_run_m;
  out.reliable_range_m   = ag.reliable_range_m;
  out.can_fit            = (ag.longest_free_run_m >= (pitch - _cfg->clearInteriorM));
  out.occupied_fraction  = (ag.enabled_len > 0.0f) ? (ag.occupied_len / ag.enabled_len) : 0.0f;
  out.est_free_spaces    = reported;

  // Camera-moved detector: a sudden, sustained scene-wide occupancy step (unlike
  // gradual parking) most likely means the camera was nudged and the calibration is
  // now stale. Sticky once tripped; cleared by reset() / mark-all-empty. Skipped
  // while warming (the reference is still being established).
  float occFrac = out.occupied_fraction;
  if (!_occRefInit) { _occFracRef = occFrac; _occRefInit = true; }
  else if (!_cameraMoved && !out.warming) {
    if (fabsf(occFrac - _occFracRef) > CAM_MOVE_DELTA) {
      if (_moveCnt < 0xFF) _moveCnt++;
      if (_moveCnt >= CAM_MOVE_FRAMES) _cameraMoved = true;
    } else {
      _moveCnt = 0;
      _occFracRef += CAM_REF_ALPHA * (occFrac - _occFracRef);
    }
  }
  out.camera_moved = _cameraMoved;

  // Per-strip breakdown (diagnostic; raw per-strip spaces, no separate hold).
  out.nStrips = nStrips;
  for (int si = 0; si < nStrips && si < MAX_STRIPS; si++) {
    out.strips[si].free_curb_m        = ag.strip[si].free_curb_m;
    out.strips[si].longest_free_run_m = ag.strip[si].longest_free_run_m;
    out.strips[si].est_free_spaces    = ag.strip[si].raw_spaces;
    out.strips[si].reliable_range_m   = ag.strip[si].reliable_range_m;
  }

  out.tookMs = millis() - t0;
  out.valid  = true;
  return true;
}
