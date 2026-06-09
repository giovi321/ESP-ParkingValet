#pragma once
#include <Arduino.h>
#include "config_store.h"

// WiFi/network manager: STA with reconnect, AP + captive-portal fallback,
// and the webhook multipart POST.

enum NetMode { NET_BOOT, NET_STA, NET_AP };

struct NetStatus {
  NetMode mode;
  bool    staConnected;
  String  ip;
  int8_t  rssi;
};

// Initialize WiFi subsystem with the given live config.
void netBegin(const Config* cfg);

// Try to join the configured STA network. Blocks up to timeoutMs.
// Returns true on success. With no saved SSID, returns false immediately.
bool netStartSTA(uint32_t timeoutMs = 25000);

// Start SoftAP + captive portal (config mode). fromFailure=true means we fell
// back here because a station join failed (vs. a user/first-time setup), which
// arms the offline-reboot watchdog so the device keeps retrying the real WiFi.
void netStartAP(bool fromFailure = false);

// Must be called frequently from loop(): services DNS (AP) and STA reconnect.
void netLoop();

NetMode  netMode();
bool     netIsAP();
NetStatus netGetStatus();

// Build and POST the webhook event (multipart/form-data with the fields and,
// when jpg/jpgLen are given, the JPEG image part — pass jpg=nullptr/jpgLen=0 to
// omit it for a count-only event). `event` is e.g. "count_changed"/"heartbeat";
// `slots` is per-ROI occupancy.
//
// For replayed (spooled) events, pass the original capture time via origTs
// (UTC epoch) / origIso (ISO8601) and queued=true so the receiver can tell a
// backfill from a live event; queuedAgeS is how long it sat in the queue. Live
// sends use the defaults (current time, queued=false).
//
// Returns the HTTP status code (>0 ok), or a negative error.
int netSendEvent(const Config& cfg, const char* event,
                 const uint8_t* jpg, size_t jpgLen,
                 int count, int prevCount,
                 const bool* slots, int nSlots,
                 uint32_t origTs = 0, const char* origIso = nullptr,
                 bool queued = false, uint32_t queuedAgeS = 0);

// POST a JSON body to an arbitrary URL with an optional auth header (used by the
// stats/telemetry webhook). Returns the HTTP status code (>0 ok), or negative.
int netPostJson(const char* url, const char* authName, const char* authValue,
                bool tlsInsecure, const String& body);
