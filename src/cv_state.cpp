#include "cv_state.h"
#include <Preferences.h>
#include "config_store.h"   // CFG_NAMESPACE

// Own key inside the config namespace (parallel to CFG_KEY, never overlaps it).
static const char* CVSTATE_KEY = "cvstate";

bool cvStateLoad(CvPersist& out) {
  Preferences p;
  if (!p.begin(CFG_NAMESPACE, /*readOnly=*/true)) return false;
  size_t len = p.getBytesLength(CVSTATE_KEY);
  bool ok = (len == sizeof(CvPersist)) &&
            (p.getBytes(CVSTATE_KEY, &out, sizeof(CvPersist)) == sizeof(CvPersist));
  p.end();
  return ok && out.magic == CV_PERSIST_MAGIC;
}

bool cvStateSave(const CvPersist& in) {
  Preferences p;
  if (!p.begin(CFG_NAMESPACE, /*readOnly=*/false)) return false;
  size_t n = p.putBytes(CVSTATE_KEY, &in, sizeof(CvPersist));
  p.end();
  return n == sizeof(CvPersist);
}
