#include "cv.h"
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
  }
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
    mix((uint32_t)r.enabled);
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
  const float hys = _cfg->hysteresis;
  const uint8_t stableNeed = _cfg->stableFrames ? _cfg->stableFrames : 1;

  int count = 0;
  for (int i = 0; i < _cfg->roiCount && i < MAX_ROIS; i++) {
    const Roi& roi = _cfg->rois[i];
    SlotResult& sr = out.slots[i];
    sr.threshold = (roi.threshold > 0.0f) ? roi.threshold : globalThr;

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

    uint64_t gradSum = 0, intSum = 0;
    uint32_t cnt = 0;
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
        int gx = abs((int)row[x + 1] - (int)row[x]);
        int gy = abs((int)nxt[x]     - (int)row[x]);
        gradSum += (gx + gy);
        intSum  += row[x];
        cnt++;
      }
    }
    float edge  = cnt ? (float)gradSum / (float)cnt : 0.0f;
    float meanI = cnt ? (float)intSum  / (float)cnt : 0.0f;
    sr.edge = edge; sr.meanI = meanI;

    if (!roi.enabled) {
      // Keep geometry but do not count; report instantaneous values only.
      sr.rawOccupied = false; sr.occupied = false; sr.baselineEdge = _baselineEdge[i];
      continue;
    }

    // Hysteresis around the effective threshold.
    float enter = sr.threshold * (1.0f + hys);
    float exit  = sr.threshold * (1.0f - hys);
    bool raw = _committed[i] ? (edge >= exit) : (edge > enter);
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

    // Adaptive empty-edge baseline (diagnostic / auto-threshold helper):
    // track the edge level only while the slot is committed-empty and stable.
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
