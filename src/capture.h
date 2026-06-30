#pragma once
#include "config_store.h"
#include "cv.h"
// When cfg.trainCapture is on, POST per-cell feature vectors + weak labels (the current
// committed decision) to cfg.captureUrl for offline training. Throttled internally; no-op
// when off, when captureUrl is empty, or when the result is invalid.
void captureMaybeLog(const Config& cfg, const CurbResult& r);
