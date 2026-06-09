#pragma once
#include <Arduino.h>
#include "config_store.h"

// ---------------------------------------------------------------------------
// Offline spool: store-and-forward queue for count-change webhook events.
//
// Every committed count change is persisted here BEFORE the baseline advances,
// then delivered when the link returns — surviving a connection blip and a
// reboot (including the offline-reboot watchdog). The queue is bounded: when it
// hits the configured entry/byte caps (or the filesystem fills) the OLDEST
// entry is dropped.
//
// Storage backend is configurable (cfg.spoolBackend): "auto" prefers an SD card
// and falls back to internal flash. On this board the camera occupies the
// SD_MMC pins, so SD is unavailable and the spool always lands on internal
// flash (LittleFS on the ~128 KB "spiffs" partition). LittleFS allocates in
// 4 KB blocks, so realistic capacity is ~20 count records or 1-2 photos.
//
// One entry == one file "/spool/<seq>.bin": a single-line meta JSON, a newline,
// then the optional JPEG bytes. Writes go to "<seq>.tmp" and are renamed into
// place so a power loss never leaves a half-written entry in the queue.
// ---------------------------------------------------------------------------

// Mount the backend and recover any queued entries. Call once in setup() after
// configLoad(). Safe to call when cfg->spoolMode == SPOOL_OFF (mounts nothing).
void spoolBegin(const Config* cfg);

// Persist a count-change event. In SPOOL_PHOTO mode the JPEG (jpg/jpgLen) is
// stored too; in SPOOL_COUNT mode the image is ignored. Enforces the caps,
// dropping the oldest entries as needed. No-op when the spool is off.
void spoolEnqueue(const char* event, const uint8_t* jpg, size_t jpgLen,
                  int count, int prevCount, const bool* slots, int nSlots);

// Deliver the oldest queued event if the link is up and the webhook is
// configured. Call from loop(): sends at most one entry per call, throttled by
// cfg.minSendIntervalMs, and removes it only on a 2xx/3xx response.
void spoolDrain();

// Current queue depth (entries) and logical size (bytes) for the UI.
void spoolStats(uint32_t& count, uint32_t& bytes);

// Erase every queued entry.
void spoolClear();
