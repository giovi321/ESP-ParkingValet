#include "clf.h"
#include "clf_model.h"
#include "features.h"      // CLF_NFEAT
#include <Preferences.h>
#include <Arduino.h>
#include <math.h>

// Runtime (hot-swappable) logistic model, persisted to NVS. When present it takes
// priority over any compiled-in clf_model.h, so a freshly-trained model can be
// pushed over the network (POST /api/model) without reflashing.
static const char*    CLF_NS    = "clfmodel";
static const char*    CLF_KEY   = "wb";
static const uint16_t CLF_MAGIC = 0xC1F0;

struct ClfBlob {
  uint16_t magic;
  uint16_t nfeat;
  float    w[CLF_NFEAT];
  float    b;
};

static bool  s_rtPresent = false;
static float s_W[CLF_NFEAT];
static float s_b = 0.0f;

static bool allFinite(const float* w, float b) {
  if (!isfinite(b)) return false;
  for (int k = 0; k < CLF_NFEAT; k++) if (!isfinite(w[k])) return false;
  return true;
}

void clfBegin() {
  Preferences p;
  if (!p.begin(CLF_NS, /*readOnly=*/true)) return;
  ClfBlob blob;
  size_t got = p.getBytes(CLF_KEY, &blob, sizeof(blob));
  p.end();
  if (got == sizeof(blob) && blob.magic == CLF_MAGIC && blob.nfeat == CLF_NFEAT &&
      allFinite(blob.w, blob.b)) {
    for (int k = 0; k < CLF_NFEAT; k++) s_W[k] = blob.w[k];
    s_b = blob.b;
    s_rtPresent = true;
  }
}

bool clfSaveRuntime(const float* w, float b) {
  if (!allFinite(w, b)) return false;
  ClfBlob blob;
  blob.magic = CLF_MAGIC; blob.nfeat = CLF_NFEAT; blob.b = b;
  for (int k = 0; k < CLF_NFEAT; k++) blob.w[k] = w[k];
  Preferences p;
  if (!p.begin(CLF_NS, /*readOnly=*/false)) return false;
  size_t n = p.putBytes(CLF_KEY, &blob, sizeof(blob));
  p.end();
  if (n != sizeof(blob)) return false;
  for (int k = 0; k < CLF_NFEAT; k++) s_W[k] = w[k];
  s_b = b; s_rtPresent = true;
  return true;
}

void clfClearRuntime() {
  Preferences p;
  if (p.begin(CLF_NS, /*readOnly=*/false)) { p.remove(CLF_KEY); p.end(); }
  s_rtPresent = false;
}

bool clfRuntimePresent() { return s_rtPresent; }

bool clfAvailable() { return s_rtPresent || CLF_MODEL_PRESENT != 0; }

float clfScore(const float* feat) {
  if (s_rtPresent) {
    float z = s_b;
    for (int k = 0; k < CLF_NFEAT; k++) z += s_W[k] * feat[k];
    float p = 1.0f / (1.0f + expf(-z));
    return p;   // already in [0,1]
  }
  if (!CLF_MODEL_PRESENT) return -1.0f;
  float p = clfPredict(feat);
  if (p < 0) return -1.0f;
  return p > 1.0f ? 1.0f : p;
}
