#pragma once
#include <Arduino.h>
#include "config_store.h"
#include "curb_reduce.h"   // pure free-gap reducer (host-testable)

// ---------------------------------------------------------------------------
// On-device classical CV for curb free-space occupancy.
//
// Pipeline (all on the ESP32, nothing leaves the device):
//   JPEG  ->  downscaled RGB565 (img_converters jpg2rgb565)  ->  luma buffer
//   per CurbCell: normalized edge/gradient energy (primary, lighting-robust)
//                 + mean intensity (secondary / diagnostics)
//   occupancy: absolute edge threshold with hysteresis, or relative to per-cell
//              adaptive empty baseline (lighting-robust)
//   debounce: a cell's raw state must hold stableFrames cycles before commit
//   headline: free-gap run-length -> free_curb_m / est_free_spaces / can_fit
// ---------------------------------------------------------------------------

struct CellResult {
  float feat[16];       // CLF_NFEAT feature vector (capture + diagnostics)
  float clfScore;       // classifier occupied probability [0,1], or -1 if not computed
  float edge, meanI, baselineEdge;
  bool  occupied;       // committed (post-debounce, post-smooth)
  bool  rawOccupied;    // pre-debounce instantaneous decision
  bool  inRange;        // false => excluded far/out-of-range cell
};

struct CurbResult {
  bool     valid;
  int      decW, decH; uint32_t tookMs;
  int      nCells;
  CellResult cells[MAX_CELLS];
  // headline (aggregated across strips, in-range only):
  float    free_curb_m;
  float    longest_free_run_m;
  bool     can_fit;            // longest_free_run_m >= carPitchM - clearInteriorM
  int      est_free_spaces;
  float    reliable_range_m;   // total in-range curb length the camera resolves
  float    occupied_fraction;  // 0..1 over in-range cells
  bool     dark;               // light-confidence gate tripped
  bool     warming;            // baselines freshly (re)seeded: numbers low-confidence until the EMA settles
  // Pitch observability (so the configured-vs-observed disagreement is visible; spec §5):
  float    pitch_m;            // pitch actually used for est_free_spaces this frame
  float    pitch_learned_m;    // auto-learn refiner estimate (0 = not learned yet)
  uint16_t pitch_samples;      // accepted isolated-car observations (gate at >= 30)
  bool     pitch_disagree;     // learned pitch gated AND differs from configured by > 15%
};

// Per-cell state that must survive a reboot (adaptive baselines).
// The new magic (0xCB02 vs old 0xCB01) causes old blobs to be silently ignored.
static const uint16_t CURB_PERSIST_MAGIC = 0xCB02;
struct CurbPersist {
  uint16_t magic;
  uint16_t cellCount;
  uint32_t geomSig;                 // hash of cell geometry; mismatch drops stale baselines
  float    baselineEdge[MAX_CELLS];
  uint8_t  baselineInit[MAX_CELLS];
  uint8_t  committed[MAX_CELLS];
  float    carPitchLearned;         // auto-learn refiner state (Task C4.1)
  uint16_t learnSamples;            // K (gate at >= 30) (Task C4.1)
};

class CvEngine {
 public:
  // cfg is a live pointer (re-read every analyze()).
  void begin(const Config* cfg);

  // Analyze one JPEG frame. srcW/srcH are the JPEG dimensions (fb->width/height).
  // Updates internal per-cell state and fills out. Returns false on decode error.
  bool analyze(const uint8_t* jpg, size_t len, int srcW, int srcH, CurbResult& out);

  // Forget per-cell debounce/baseline state (call when geometry changes).
  void reset();

  // Re-arm one cell's empty baseline (index < 0 = all cells): clear its committed
  // state and force the baseline to re-seed from the next frame.
  void recalibrate(int index);

  // Force one cell (index >= 0) to read occupied now. The inverse of recalibrate:
  // in relative mode it lowers the cell's baseline so the live metric sits at the
  // occupied threshold — fixing a cell that seeded while occupied. Self-heals once
  // the curb clears. (In absolute mode it just commits occupied, best-effort.)
  void markOccupied(int index);

  // Serialize / restore the per-cell baseline state for NVS persistence. restore
  // applies only when the stored geometry signature matches the live geometry
  // (else returns false and the engine seeds live as before).
  void snapshotState(CurbPersist& out) const;
  bool restoreState(const CurbPersist& in);

  // Auto-learn refiner accessors (Task C4.1).
  float    learnedPitch() const { return _carPitchLearned; }
  uint16_t learnSamples() const { return _learnSamples; }

 private:
  const Config* _cfg = nullptr;

  // analysis buffers (PSRAM), reallocated when decoded size changes
  uint8_t* _luma = nullptr;
  uint8_t* _rgb  = nullptr;
  int _decW = 0, _decH = 0;

  // reducer cell view (kept off the analyze() stack; refilled each frame)
  curb_reduce::Cell _rc[MAX_CELLS];

  // per-cell persistent state
  bool     _committed[MAX_CELLS];
  bool     _lastRaw[MAX_CELLS];
  uint16_t _stableCnt[MAX_CELLS];
  float    _baselineEdge[MAX_CELLS];
  bool     _baselineInit[MAX_CELLS];
  float    _lastEdge[MAX_CELLS];    // most recent per-cell edge (for markOccupied)

  uint32_t _roiSig = 0;   // signature of current cell geometry, to detect changes

  // Auto-learn car-pitch refiner (Task C4.1):
  float    _carPitchLearned = 0.0f;        // learned pitch (m); 0 = not yet learned
  uint16_t _learnSamples    = 0;           // accepted isolated-car observations (gate >= 30)
  bool     _learnEpisodeActive[MAX_CELLS]; // anti-double-count: episode active per cell

  // Stability hold for est_free_spaces (T5): keeps the published integer from
  // flapping ±1 as a vehicle passes or light shifts.  Biases to UNDER-count
  // (safe direction) because a parked occluder persists in the committed state.
  int     _reportedSpaces = -1;  // last promoted value (-1 = not yet set)
  int     _pendingSpaces  = -1;  // candidate waiting for stableFrames
  uint8_t _pendingCnt     = 0;   // frames the candidate has held

  // Low-confidence warm-up: armed whenever a cell seeds its empty baseline from an
  // unknown scene (fresh boot without NVS restore, geometry re-trace, mark-empty),
  // counts down over the next frames so the headline is flagged until the adaptive
  // baseline has had a chance to settle (spec: "flagged low-confidence until ...
  // the adaptive EMA warms up").
  static const uint16_t CV_WARMUP_FRAMES = 20;
  uint16_t _warmupLeft = 0;

  bool ensureBuffers(int w, int h);
  uint32_t roiSignature() const;

  // analyze() pipeline stages (see cv.cpp). Kept as private methods so each stage
  // has one job and analyze() reads as an orchestrator.
  // Per-frame decision parameters, computed once and shared by every cell.
  struct DecideParams {
    float   globalThr, hys, relDelta, emaRate;
    bool    relative;
    uint8_t stableNeed;
  };
  // Decode + downscale the JPEG into _rgb/_luma (sets _decW/_decH). False on failure.
  bool decodeToLuma(const uint8_t* jpg, size_t len, int srcW, int srcH);
  // Accumulate features for cell i, run the occupied/free decision + debounce +
  // baseline update, and fill cellRes. Uses _decW/_decH/_luma/_rgb.
  void analyzeCell(int i, const DecideParams& dp, CellResult& cellRes);
};
