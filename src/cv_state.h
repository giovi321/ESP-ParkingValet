#pragma once
#include "cv.h"

// ---------------------------------------------------------------------------
// Persist the CvEngine's per-cell baselines across reboots.
//
// The blob is stored in the existing config NVS namespace under its own key, so
// a config save / factory-reset of one never disturbs the other. The contents
// are validated by CurbPersist::magic and (at restore time, in the engine) by
// the geometry signature, so a stale or geometry-mismatched blob is ignored.
// ---------------------------------------------------------------------------

bool cvStateLoad(CurbPersist& out);     // false if absent / wrong size / bad magic
bool cvStateSave(const CurbPersist& in);
