#pragma once
#include "cv.h"
#include "config_store.h"

// Learned occupancy profile: a slow per-hour-of-day EMA of free_curb_m, so the
// device can report what is "typical" for the current time and whether the curb is
// busier or quieter than usual. Persisted to NVS (tiny: 24 floats), independent of
// the config blob. Local hour uses the configured tz offset.

void  profileBegin(const Config* cfg);
void  profileUpdate(const CurbResult& r);  // fold this frame into the current hour (throttled, gated)
void  profileLoop();                       // periodic NVS persist (throttled, change-gated)
float profileTypicalNow();                 // learned free_curb_m for the current hour, -1 if unlearned
float profileTypicalNextHour();            // learned free_curb_m for the next hour, -1 if unlearned
bool  profileHasData();                    // the current hour has at least one observation
