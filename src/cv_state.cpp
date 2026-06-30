#include "cv_state.h"
#include <Preferences.h>
#include "config_store.h"   // CFG_NAMESPACE

// Own key inside the config namespace (parallel to CFG_KEY, never overlaps it).
static const char* CVSTATE_KEY = "cvstate";

bool cvStateLoad(CurbPersist& out) {
  Preferences p;
  if (!p.begin(CFG_NAMESPACE, /*readOnly=*/true)) return false;
  size_t len = p.getBytesLength(CVSTATE_KEY);
  bool ok = (len == sizeof(CurbPersist)) &&
            (p.getBytes(CVSTATE_KEY, &out, sizeof(CurbPersist)) == sizeof(CurbPersist));
  p.end();
  return ok && out.magic == CURB_PERSIST_MAGIC;
}

bool cvStateSave(const CurbPersist& in) {
  Preferences p;
  if (!p.begin(CFG_NAMESPACE, /*readOnly=*/false)) return false;
  size_t n = p.putBytes(CVSTATE_KEY, &in, sizeof(CurbPersist));
  p.end();
  return n == sizeof(CurbPersist);
}
