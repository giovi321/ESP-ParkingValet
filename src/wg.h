#pragma once
#include "config_store.h"

// WireGuard client: joins the homelab WG network as an outbound peer so the web
// UI/OTA are reachable on the tunnel IP. Brings the tunnel up only after WiFi STA
// is connected AND the clock is NTP-synced (handshakes carry TAI64N timestamps).
void        wgBegin(const Config* cfg);
void        wgLoop(uint32_t now);   // call every loop(); cheap, non-blocking
bool        wgIsUp();
const char* wgStateStr();

// Tear the tunnel down and re-arm from the (already-updated) config. Call after a
// config change touched any wg* field: disabling leaves no tunnel up, and edited
// keys/endpoint/AllowedIPs take effect instead of being silently ignored.
void        wgReconfigure();
