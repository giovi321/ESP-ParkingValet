#pragma once
#include <Arduino.h>
#include "config_store.h"
#include "cv.h"

// Capture a fresh camera frame and burn the curb overlay into it: each cell's
// quad coloured by occupancy (green=free, red=occupied, grey=disabled/out-of-range),
// the strip name as a label, and a local-time timestamp. Returns a freshly-malloc'd
// JPEG via *out (free it with free()); the return value is the JPEG length, or 0 on
// any failure (no frame, decode error, OOM). Runs on the loopTask; takes ~1-2 s.
// `cv` supplies occupancy. Full cell/strip draw loop is implemented in Task C3.2.
size_t overlayRenderJpeg(const Config& cfg, const CurbResult& cv, uint8_t** out);
