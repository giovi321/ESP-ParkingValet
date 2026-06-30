#pragma once
#include <Arduino.h>

#define CLF_NFEAT 16

// Accumulated over a bay's in-polygon pixels in one pass (see cv.cpp).
struct FeatureAccum {
  uint32_t cnt;          // pixel count
  uint64_t gradSum;      // sum of |gx|+|gy|
  uint64_t intSum;       // sum of luma
  uint64_t intSqSum;     // sum of luma^2 (for variance)
  uint32_t lbp[10];      // uniform LBP(8,1) histogram (bins 0..8 = #set-bits for uniform codes, bin 9 = non-uniform)
  int64_t  satSum_x1000; // sum of saturation*1000 (integer-friendly)
  int64_t  brSum;        // sum of (b - r)  (each -255..255)
};

inline void featureAccumInit(FeatureAccum& a) { memset(&a, 0, sizeof(a)); }

// Produce the canonical CLF_NFEAT vector. baselineEdge is the cell's adaptive empty
// reference (CellResult.baselineEdge / _baselineEdge[i]).
inline void featuresFinalize(const FeatureAccum& a, float baselineEdge, float* feat) {
  float n = a.cnt ? (float)a.cnt : 1.0f;
  float edge  = (float)a.gradSum / n;
  float meanI = (float)a.intSum  / n;
  float meanSq= (float)a.intSqSum / n;
  float var   = meanSq - meanI * meanI; if (var < 0) var = 0;
  feat[0] = edge;
  feat[1] = edge - baselineEdge;
  feat[2] = meanI;
  feat[3] = sqrtf(var);
  uint32_t lbpTot = 0; for (int k = 0; k < 10; k++) lbpTot += a.lbp[k];
  float lt = lbpTot ? (float)lbpTot : 1.0f;
  for (int k = 0; k < 10; k++) feat[4 + k] = (float)a.lbp[k] / lt;
  feat[14] = (float)a.satSum_x1000 / (n * 1000.0f);
  feat[15] = (float)a.brSum / (n * 255.0f);
}
