#pragma once
#include <Arduino.h>
#include "config_store.h"
#include "cv.h"

// Built-in (synchronous) WebServer on port 80: serves the gzipped SPA, the
// JSON config/state API, a live snapshot, OTA upload, a captive portal in AP
// mode, and Digest-authenticated access to everything under /api, /snapshot
// and /update.
//
// `cfg` and `last` are live pointers owned by main(); the loop updates *last
// every CV cycle, and the API reads/merges *cfg.
void webBegin(Config* cfg, CvResult* last);
void webLoop();

// Record the outcome of a webhook send so the UI can display it.
void webNoteSend(const char* event, int count, int httpCode);
