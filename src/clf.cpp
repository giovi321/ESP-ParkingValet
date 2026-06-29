#include "clf.h"
#include "clf_model.h"
bool  clfAvailable() { return CLF_MODEL_PRESENT != 0; }
float clfScore(const float* feat) {
  if (!CLF_MODEL_PRESENT) return -1.0f;
  float p = clfPredict(feat);
  if (p < 0) return -1.0f;
  return p > 1.0f ? 1.0f : p;
}
