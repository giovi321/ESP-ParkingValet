#pragma once
#include <Arduino.h>
#include "config_store.h"
#include "cv.h"

// Capture a fresh camera frame and burn the ROI overlay into it: each bay's
// polygon outline + translucent fill colored by occupancy (green=free,
// red=occupied, grey=disabled), the bay name as a label, and a local-time
// timestamp. Returns a freshly-malloc'd JPEG via *out (free it with free());
// the return value is the JPEG length, or 0 on any failure (no frame, decode
// error, OOM). Runs on the loopTask; takes ~1-2 s. `cv` supplies occupancy.
size_t overlayRenderJpeg(const Config& cfg, const CvResult& cv, uint8_t** out);
