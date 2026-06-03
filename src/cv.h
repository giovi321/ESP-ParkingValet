#pragma once
#include <Arduino.h>
#include "config_store.h"

// ---------------------------------------------------------------------------
// On-device classical CV for parking-slot occupancy.
//
// Pipeline (all on the ESP32, nothing leaves the device):
//   JPEG  ->  downscaled RGB565 (img_converters jpg2rgb565)  ->  luma buffer
//   per ROI: normalized edge/gradient energy (primary, lighting-robust)
//            + mean intensity (secondary / diagnostics)
//   occupancy: absolute edge threshold with hysteresis
//   adaptive empty-edge EMA per slot (diagnostics + threshold auto-suggest)
//   debounce: a slot's raw state must hold STABLE_FRAMES cycles before it commits
//   count = number of committed-occupied enabled ROIs
// ---------------------------------------------------------------------------

struct SlotResult {
  float edge;          // current normalized edge energy (mean |gradient|)
  float meanI;         // current mean intensity (0..255)
  float baselineEdge;  // adaptive EMA of edge energy while empty (diagnostic)
  float threshold;     // effective threshold used for this slot
  bool  occupied;      // committed occupancy
  bool  rawOccupied;   // instantaneous (pre-debounce) decision
};

struct CvResult {
  bool       valid;
  int        count;        // committed occupied slots
  int        n;            // number of ROIs evaluated
  int        decW, decH;   // analysis-image dimensions
  uint32_t   tookMs;
  SlotResult slots[MAX_ROIS];
};

class CvEngine {
 public:
  // cfg is a live pointer (re-read every analyze()).
  void begin(const Config* cfg);

  // Analyze one JPEG frame. srcW/srcH are the JPEG dimensions (fb->width/height).
  // Updates internal per-slot state and fills out. Returns false on decode error.
  bool analyze(const uint8_t* jpg, size_t len, int srcW, int srcH, CvResult& out);

  // Forget per-slot debounce/baseline state (call when ROIs change).
  void reset();

 private:
  const Config* _cfg = nullptr;

  // analysis buffers (PSRAM), reallocated when decoded size changes
  uint8_t* _luma = nullptr;
  uint8_t* _rgb  = nullptr;
  int _decW = 0, _decH = 0;

  // per-slot persistent state
  bool     _committed[MAX_ROIS];
  bool     _lastRaw[MAX_ROIS];
  uint16_t _stableCnt[MAX_ROIS];
  float    _baselineEdge[MAX_ROIS];
  bool     _baselineInit[MAX_ROIS];

  uint32_t _roiSig = 0;   // signature of current ROI set, to detect changes

  bool ensureBuffers(int w, int h);
  uint32_t roiSignature() const;
};
