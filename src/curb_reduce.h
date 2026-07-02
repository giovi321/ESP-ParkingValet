#pragma once
#include <stdint.h>
#include <math.h>

// ---------------------------------------------------------------------------
// Pure curb free-space reducer: the spatial-median smoothing, free-gap run-length,
// clearance rules, pitch auto-learner and integer stability hold, with NO Arduino /
// esp_camera dependency. Extracted from CvEngine::analyze() so the exact numbers
// published to Home Assistant and the webhooks can be unit-tested on the host
// (see test/test_curb_reduce/). CvEngine fills a Cell[] from its config + per-cell
// decisions and calls these; behaviour is identical to the previous inline code.
// ---------------------------------------------------------------------------

namespace curb_reduce {

static const int RD_MAX_CELLS  = 48;   // must match config_store.h MAX_CELLS
static const int RD_MAX_STRIPS = 4;    // must match config_store.h MAX_STRIPS

struct Cell {
  bool    occupied;   // post-debounce decision (mutated in place by spatialMedian)
  bool    enabled;    // dead-zone mask / in-range gate
  uint8_t strip;      // owning strip index
  float   lenM;       // this cell's ground length (metres)
};

// Gather, in array order, the indices of cells belonging to strip si. When
// enabledOnly is true only enabled cells are collected. Returns the count.
inline int collectStrip(const Cell* c, int n, int si, bool enabledOnly, int* idx) {
  int ni = 0;
  for (int i = 0; i < n; i++) {
    if (c[i].strip != (uint8_t)si) continue;
    if (enabledOnly && !c[i].enabled) continue;
    idx[ni++] = i;
  }
  return ni;
}

// Width-3 spatial median per strip over the enabled-cell sequence, in place on
// c[].occupied. Reads a snapshot first so neighbours are unaffected by earlier
// writes in the same pass. Kills single-cell salt-and-pepper before run-length.
inline void spatialMedian(Cell* c, int n, int nStrips) {
  for (int si = 0; si < nStrips && si < RD_MAX_STRIPS; si++) {
    int idx[RD_MAX_CELLS];
    int ni = collectStrip(c, n, si, /*enabledOnly=*/true, idx);
    if (ni < 2) continue;
    bool snap[RD_MAX_CELLS];
    for (int k = 0; k < ni; k++) snap[k] = c[idx[k]].occupied;
    for (int k = 0; k < ni; k++) {
      bool lo = snap[(k > 0)      ? k - 1 : 0];
      bool me = snap[k];
      bool hi = snap[(k < ni - 1) ? k + 1 : ni - 1];
      c[idx[k]].occupied = ((lo ? 1 : 0) + (me ? 1 : 0) + (hi ? 1 : 0)) >= 2;
    }
  }
}

struct StripAgg {
  float free_curb_m;
  float longest_free_run_m;
  int   raw_spaces;
  float reliable_range_m;   // enabled length in this strip
};

struct Aggregate {
  float    free_curb_m;
  float    longest_free_run_m;
  int      raw_spaces;
  float    reliable_range_m;   // total enabled length
  float    enabled_len;        // == reliable_range_m
  float    occupied_len;
  StripAgg strip[RD_MAX_STRIPS];
};

struct Params {
  float pitch;            // pitch actually used (learned when gated, else configured)
  float clearInteriorM;   // clearance for a gap bounded by parked cars on both sides
  float clearEndM;        // clearance for a gap touching a hard boundary
};

// Free-gap run scan + aggregate scalars (totals and per-strip). occupied[] must be
// the post-median decision. A gap is TERMINAL (uses clearEndM) when it touches a
// hard boundary at either end: the strip's physical start/end or a disabled cell.
// It is INTERIOR (clearInteriorM) only when bounded by an enabled occupied cell on
// both sides. clearEndM > clearInteriorM biases boundary gaps to under-count (safe).
inline void aggregate(const Cell* c, int n, int nStrips, const Params& p, Aggregate& out) {
  out.free_curb_m = 0.0f; out.longest_free_run_m = 0.0f; out.raw_spaces = 0;
  out.reliable_range_m = 0.0f; out.enabled_len = 0.0f; out.occupied_len = 0.0f;
  for (int s = 0; s < RD_MAX_STRIPS; s++) {
    out.strip[s].free_curb_m = 0.0f; out.strip[s].longest_free_run_m = 0.0f;
    out.strip[s].raw_spaces = 0; out.strip[s].reliable_range_m = 0.0f;
  }

  // Enabled length + occupied length (per strip and total).
  for (int i = 0; i < n; i++) {
    if (!c[i].enabled) continue;
    out.enabled_len += c[i].lenM;
    if (c[i].occupied) out.occupied_len += c[i].lenM;
    if (c[i].strip < RD_MAX_STRIPS) out.strip[c[i].strip].reliable_range_m += c[i].lenM;
  }
  out.reliable_range_m = out.enabled_len;

  // Per-strip free-gap run scan.
  for (int si = 0; si < nStrips && si < RD_MAX_STRIPS; si++) {
    int idx[RD_MAX_CELLS];
    int ni = collectStrip(c, n, si, /*enabledOnly=*/false, idx);
    StripAgg& sa = out.strip[si];
    bool  in_run     = false;
    float run_m      = 0.0f;
    bool  front_hard = false;
    for (int k = 0; k < ni; k++) {
      int ci = idx[k];
      bool free_cell = c[ci].enabled && !c[ci].occupied;
      if (free_cell) {
        if (!in_run) {
          in_run = true; run_m = 0.0f;
          front_hard = (k == 0) || !c[idx[k - 1]].enabled;   // strip start or dead-zone wall
        }
        run_m += c[ci].lenM;
      } else if (in_run) {
        bool  back_hard = !c[ci].enabled;                     // dead-zone wall vs parked car
        float clr = (front_hard || back_hard) ? p.clearEndM : p.clearInteriorM;
        sa.free_curb_m += run_m;
        if (run_m > sa.longest_free_run_m) sa.longest_free_run_m = run_m;
        if (run_m >= p.pitch) {
          int nn = (int)floorf((run_m - clr) / p.pitch);
          sa.raw_spaces += (nn > 0 ? nn : 0);
        }
        in_run = false; run_m = 0.0f;
      }
    }
    if (in_run) {   // run reached the strip's physical end (hard boundary)
      sa.free_curb_m += run_m;
      if (run_m > sa.longest_free_run_m) sa.longest_free_run_m = run_m;
      if (run_m >= p.pitch) {
        int nn = (int)floorf((run_m - p.clearEndM) / p.pitch);
        sa.raw_spaces += (nn > 0 ? nn : 0);
      }
    }
    out.free_curb_m += sa.free_curb_m;
    out.raw_spaces  += sa.raw_spaces;
    if (sa.longest_free_run_m > out.longest_free_run_m) out.longest_free_run_m = sa.longest_free_run_m;
  }
}

// --- Auto-learn car-pitch refiner (stateful) -------------------------------
struct PitchLearnState {
  float    carPitchLearned;         // learned pitch (m); 0 = not learned yet
  uint16_t learnSamples;            // accepted isolated-car observations
  bool     episodeActive[RD_MAX_CELLS];  // anti-double-count per cell
};
struct PitchLearnParams {
  float    clearInteriorM;
  float    bandMin;                 // reject footprints below (noise/motorcycles)
  float    bandMax;                 // reject footprints above (multi-car blobs)
  float    alpha;                   // EMA rate toward observed footprint
  uint16_t maxSamples;              // saturation cap for learnSamples
};

// Fold at most one isolated-car observation per new parking episode into the EMA.
inline void pitchLearn(const Cell* c, int n, int nStrips, const PitchLearnParams& p, PitchLearnState& st) {
  // Re-arm every cell that is no longer occupied (car left) so a fresh arrival
  // produces a new observation.
  for (int i = 0; i < n; i++) if (!c[i].occupied) st.episodeActive[i] = false;

  for (int si = 0; si < nStrips && si < RD_MAX_STRIPS; si++) {
    int idx[RD_MAX_CELLS];
    int ni = collectStrip(c, n, si, /*enabledOnly=*/false, idx);
    int k = 0;
    while (k < ni) {
      int ci = idx[k];
      if (c[ci].enabled && c[ci].occupied) {
        int   rstart        = k;
        float footprint     = 0.0f;
        bool  episodeActive = false;
        while (k < ni && c[idx[k]].enabled && c[idx[k]].occupied) {
          footprint += c[idx[k]].lenM;
          if (st.episodeActive[idx[k]]) episodeActive = true;
          k++;
        }
        int rend = k - 1;
        bool isolated = (rstart > 0) && (rend < ni - 1) &&
          (c[idx[rstart - 1]].enabled && !c[idx[rstart - 1]].occupied) &&
          (c[idx[rend + 1]].enabled   && !c[idx[rend + 1]].occupied);
        bool inBand = (footprint >= p.bandMin && footprint <= p.bandMax);
        if (isolated && inBand && !episodeActive) {
          float pitchObs = footprint + p.clearInteriorM;
          if (st.learnSamples == 0) st.carPitchLearned = pitchObs;
          else                      st.carPitchLearned += p.alpha * (pitchObs - st.carPitchLearned);
          if (st.learnSamples < p.maxSamples) st.learnSamples++;
          for (int m = rstart; m <= rend; m++) st.episodeActive[idx[m]] = true;
        }
      } else {
        k++;
      }
    }
  }
}

// --- Integer stability hold (stateful) -------------------------------------
struct SpacesHold { int reported; int pending; uint8_t cnt; };

// Debounce the published integer so it does not flap +/-1 as a vehicle passes.
// Returns the value to publish (reported once promoted, else the raw value).
inline int spacesHold(int raw, uint8_t stableNeed, SpacesHold& st) {
  if (raw == st.reported) {
    st.pending = -1; st.cnt = 0;
  } else if (raw == st.pending) {
    if (st.cnt < 0xFF) st.cnt++;
    if (st.cnt >= stableNeed) { st.reported = raw; st.cnt = 0; }
  } else {
    st.pending = raw; st.cnt = 1;
  }
  return (st.reported >= 0) ? st.reported : raw;
}

}  // namespace curb_reduce
