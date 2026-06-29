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
  float feat[16];      // CLF_NFEAT feature vector (see features.h); filled every analyze()
  float clfScore;      // classifier occupied probability [0,1], or -1 if not computed
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

// Per-bay state that must survive a reboot. In relative mode the empty baseline
// is the live reference the decision subtracts; it lives only in RAM and is
// re-seeded from the first frame after boot. Because this device reboots itself
// (offline watchdog / OTA), a bay occupied at reboot would seed an "occupied"
// baseline and read empty until it turns over. Persisting this blob to NVS and
// restoring it on boot keeps the learned reference instead. Plain POD: written
// to NVS verbatim, guarded by magic + the ROI signature (geometry change = drop).
static const uint16_t CV_PERSIST_MAGIC = 0xCB01;
struct CvPersist {
  uint16_t magic;
  uint16_t roiCount;
  uint32_t roiSig;                   // ROI geometry hash; must match to restore
  float    baselineEdge[MAX_ROIS];
  uint8_t  baselineInit[MAX_ROIS];
  uint8_t  committed[MAX_ROIS];
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

  // Re-arm one bay's empty baseline (index < 0 = all bays): clear its committed
  // state and force the baseline to re-seed from the next frame. Backs the
  // per-bay "mark empty now" action, so a single empty bay can be calibrated
  // without needing the whole lot empty at once.
  void recalibrate(int index);

  // Force one bay (index >= 0) to read occupied now. The inverse of "mark empty":
  // in relative mode it lowers the bay's baseline one entry-band below its current
  // edge so the live metric sits at the occupied threshold — fixing a bay that
  // seeded its baseline while occupied and reads empty. Self-heals: once the car
  // leaves, the edge drops below the re-based reference and the EMA relearns the
  // true empty level. (In absolute mode it just commits occupied, best-effort.)
  void markOccupied(int index);

  // Serialize / restore the per-bay baseline state for NVS persistence. restore
  // applies only when the stored ROI signature matches the live geometry (else it
  // returns false and the engine seeds live as before).
  void snapshotState(CvPersist& out) const;
  bool restoreState(const CvPersist& in);

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
  float    _lastEdge[MAX_ROIS];        // most recent per-bay edge (for markOccupied)

  uint32_t _roiSig = 0;   // signature of current ROI set, to detect changes

  bool ensureBuffers(int w, int h);
  uint32_t roiSignature() const;
};
