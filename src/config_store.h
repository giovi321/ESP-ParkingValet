#pragma once
#include <Arduino.h>
#include <ArduinoJson.h>

// ---------------------------------------------------------------------------
// Persistent configuration model.
//
// Everything the user can tune lives here and is persisted to NVS as a single
// compact JSON blob (key "cfg" in namespace "parkingcam"). The web UI reads it
// via configToJson() and writes partial updates via configMergeJson().
//
// Secrets (WiFi/admin/webhook credentials) are stored in NVS (internal flash).
// They are NEVER emitted over the API (configToJson(..., includeSecrets=false)
// masks them). Enable flash encryption if at-rest protection is required.
// ---------------------------------------------------------------------------

static const int   MAX_ROIS        = 12;
static const int   MAX_POLY        = 8;    // max vertices per slot polygon
static const char* CFG_NAMESPACE   = "parkingcam";
static const char* CFG_KEY         = "cfg";
static const int   CONFIG_VERSION  = 1;

// Default credentials on a fresh device (forces a change on first login).
static const char* DEFAULT_ADMIN_USER = "admin";
static const char* DEFAULT_ADMIN_PASS = "parking";
static const char* DEFAULT_AP_SSID    = "ESP-ParkingValet-Setup";
static const char* DEFAULT_AP_PASS    = "parking1234";   // >= 8 chars for WPA2
static const char* DEFAULT_HOSTNAME   = "esp-parkingvalet";

// Trigger modes
enum TriggerMode : uint8_t {
  TRIG_ANY_CHANGE = 0,   // send whenever the committed count changes
  TRIG_THRESHOLD  = 1,   // send only when the count crosses >= threshold (rising or falling edge)
};

// Offline spool (store & forward): what to persist for each count change.
enum SpoolMode : uint8_t {
  SPOOL_OFF   = 0,   // disabled: count changes are sent live, best-effort (legacy)
  SPOOL_COUNT = 1,   // queue the count-change record only (no image) — deep, durable
  SPOOL_PHOTO = 2,   // queue the photo + record (few fit; see spool.cpp)
};
// Where the spool lives.
enum SpoolBackend : uint8_t {
  SPOOL_BACKEND_AUTO  = 0,   // SD if available, else internal flash
  SPOOL_BACKEND_FLASH = 1,   // internal flash only
};

struct Roi {
  char  name[16];
  int   nPoints;            // 3..MAX_POLY
  float px[MAX_POLY];       // normalized [0..1] polygon vertices (survive resolution changes)
  float py[MAX_POLY];
  float threshold;          // edge-energy occupancy threshold (0 = use global default)
  bool  enabled;
};

struct Config {
  int  version;

  // --- WiFi / network ---
  char staSsid[33];
  char staPass[65];
  char apSsid[33];
  char apPass[65];
  char hostname[33];
  uint16_t offlineRebootMin; // auto-reboot if WiFi stays offline this many minutes (0 = off)

  // --- Web auth ---
  char adminUser[33];
  char adminPass[65];
  bool mustChangePass;       // true until the default password is changed

  // --- Webhook ---
  bool whEnabled;
  char whUrl[200];           // full URL, e.g. http://192.168.1.197:5678/webhook/parking-cam
  char whAuthHeaderName[48]; // e.g. "Authorization" or "X-API-Key" ("" = none)
  char whAuthHeaderValue[200];
  bool whTlsInsecure;        // skip cert validation for https (self-signed internal CA)

  // --- Offline spool (store & forward for count-change webhooks) ---
  uint8_t  spoolMode;        // SpoolMode: 0=off, 1=count-only, 2=photo+count
  uint16_t spoolMaxEntries;  // queue depth cap (drop oldest beyond this)
  uint16_t spoolMaxKB;       // queue size cap in KB (drop oldest beyond this)
  uint8_t  spoolBackend;     // SpoolBackend: 0=auto(SD->flash), 1=flash

  // --- Stats webhook (periodic JSON telemetry: ip, rssi, heap, count, ...) ---
  bool     statsEnabled;
  char     statsUrl[200];
  char     statsAuthHeaderName[48];
  char     statsAuthHeaderValue[200];
  bool     statsTlsInsecure;
  uint32_t statsIntervalS;

  // --- MQTT (native; optional Home Assistant auto-discovery) ---
  bool     mqttEnabled;
  char     mqttHost[128];
  uint16_t mqttPort;
  bool     mqttTls;              // mqtts (TLS)
  bool     mqttTlsInsecure;      // skip cert validation
  char     mqttUser[64];
  char     mqttPass[64];
  char     mqttBaseTopic[48];    // e.g. "parking-valet"
  bool     mqttDiscovery;        // publish HA auto-discovery configs
  char     mqttDiscoveryPrefix[32]; // e.g. "homeassistant"
  uint16_t mqttIntervalS;        // diagnostics refresh interval

  // --- Trigger logic ---
  uint8_t  triggerMode;      // TriggerMode
  int      triggerThreshold; // N (for TRIG_THRESHOLD)
  uint32_t minSendIntervalMs;
  uint32_t heartbeatIntervalS; // 0 = disabled

  // --- CV parameters ---
  uint16_t captureIntervalMs;
  uint8_t  stableFrames;     // debounce: consecutive cycles a slot state must hold
  float    edgeThreshold;    // global default edge-energy occupancy threshold
  float    hysteresis;       // fraction; enter=thr*(1+h), exit=thr*(1-h)
  float    baselineEma;      // EMA rate for adaptive empty-baseline (0..1, small)

  // --- Image / sensor ---
  int  framesize;            // framesize_t value (default SVGA = 9)
  int  jpegQuality;          // 0..63 (lower = better)
  bool vFlip;
  bool hMirror;
  int  brightness;           // -2..2
  int  contrast;             // -2..2
  int  saturation;           // -2..2
  bool awb;                  // auto white balance
  bool aec;                  // auto exposure
  int  afMode;               // OV5640 autofocus: 0=off/fixed, 1=auto once, 2=continuous

  // --- ROIs ---
  int roiCount;
  Roi rois[MAX_ROIS];
};

// Populate cfg with factory defaults.
void configLoadDefaults(Config& cfg);

// Load cfg from NVS; falls back to defaults if missing/invalid. Returns true if
// a stored config was found and loaded.
bool configLoad(Config& cfg);

// Persist cfg to NVS. Returns true on success.
bool configSave(const Config& cfg);

// Erase the stored config (next boot uses defaults).
void configFactoryReset();

// Serialize cfg into a JSON object. When includeSecrets is false, all
// credential fields are masked (empty string) so they are never leaked.
void configToJson(const Config& cfg, JsonObject out, bool includeSecrets);

// Merge a partial JSON object into cfg (only keys present are updated).
// Secret fields are only updated when the incoming value is a non-empty string,
// so a UI round-trip with masked secrets preserves the stored values.
// Returns true if any field changed. Sets *wifiChanged/*camChanged if those
// subsystems need to react.
bool configMergeJson(Config& cfg, JsonObjectConst in, bool* wifiChanged, bool* camChanged, bool* mqttChanged);
