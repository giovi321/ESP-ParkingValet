#pragma once
#include <Arduino.h>
#include "config_store.h"

// Initialize the OV5640 camera in JPEG mode using the pin map in camera_pins.h
// and the resolution/quality from cfg. Returns true on success.
bool cameraInit(const Config& cfg);

// Re-apply runtime sensor settings (framesize, quality, flip/mirror, image
// controls) without a full re-init. Safe to call after a config change.
void cameraApplySettings(const Config& cfg);

// --- OV5640 autofocus ---
// Apply the configured AF mode (loads the AF firmware on first use). afMode:
// 0 = off/fixed, 1 = focus once, 2 = continuous.
void cameraApplyFocus(const Config& cfg);

// Trigger a single autofocus now (on-demand). Returns false if AF unavailable.
bool cameraFocusNow();

// OV5640 AF firmware status byte (0x10=focused, 0x70=idle), or -1 if no AF.
int cameraFocusStatus();
