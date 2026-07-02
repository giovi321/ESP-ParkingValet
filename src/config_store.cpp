#include "config_store.h"
#include <Preferences.h>
#include <memory>

// ---- small helpers --------------------------------------------------------

static void setStr(char* dst, size_t cap, const char* src) {
  strlcpy(dst, src ? src : "", cap);
}

static bool isSecretKey(const char* k) {
  return !strcmp(k, "staPass") || !strcmp(k, "apPass") ||
         !strcmp(k, "adminPass") || !strcmp(k, "whAuthHeaderValue") ||
         !strcmp(k, "statsAuthHeaderValue") || !strcmp(k, "mqttPass") ||
         !strcmp(k, "wgPrivateKey") || !strcmp(k, "wgPresharedKey") ||
         !strcmp(k, "captureAuthHeaderValue");
}

// ---- defaults -------------------------------------------------------------

void configLoadDefaults(Config& c) {
  memset(&c, 0, sizeof(Config));
  c.version = CONFIG_VERSION;

  setStr(c.staSsid,  sizeof(c.staSsid),  "");
  setStr(c.staPass,  sizeof(c.staPass),  "");
  setStr(c.apSsid,   sizeof(c.apSsid),   DEFAULT_AP_SSID);
  setStr(c.apPass,   sizeof(c.apPass),   DEFAULT_AP_PASS);
  setStr(c.hostname, sizeof(c.hostname), DEFAULT_HOSTNAME);
  c.offlineRebootMin = 0;
  c.apRetryMin = 0;

  c.wgEnabled = false;
  setStr(c.wgPrivateKey,    sizeof(c.wgPrivateKey),    "");
  setStr(c.wgAddress,       sizeof(c.wgAddress),       "");
  setStr(c.wgPeerPublicKey, sizeof(c.wgPeerPublicKey), "");
  setStr(c.wgEndpointHost,  sizeof(c.wgEndpointHost),  "");
  c.wgEndpointPort = 51820;
  setStr(c.wgAllowedIps,    sizeof(c.wgAllowedIps),    "");
  setStr(c.wgPresharedKey,  sizeof(c.wgPresharedKey),  "");
  c.wgKeepalive = 25;

  setStr(c.adminUser, sizeof(c.adminUser), DEFAULT_ADMIN_USER);
  setStr(c.adminPass, sizeof(c.adminPass), DEFAULT_ADMIN_PASS);
  c.mustChangePass = true;

  c.whEnabled = false;
  setStr(c.whUrl, sizeof(c.whUrl), "http://192.168.1.197:5678/webhook/parking-cam");
  setStr(c.whAuthHeaderName,  sizeof(c.whAuthHeaderName),  "");
  setStr(c.whAuthHeaderValue, sizeof(c.whAuthHeaderValue), "");
  c.whTlsInsecure = true;

  c.spoolMode       = SPOOL_COUNT;          // count-only: durable, deep queue
  c.spoolMaxEntries = 20;
  c.spoolMaxKB      = 96;
  c.spoolBackend    = SPOOL_BACKEND_AUTO;

  c.statsEnabled = false;
  setStr(c.statsUrl, sizeof(c.statsUrl), "");
  setStr(c.statsAuthHeaderName,  sizeof(c.statsAuthHeaderName),  "");
  setStr(c.statsAuthHeaderValue, sizeof(c.statsAuthHeaderValue), "");
  c.statsTlsInsecure = true;
  c.statsIntervalS   = 300;

  c.mqttEnabled = false;
  setStr(c.mqttHost, sizeof(c.mqttHost), "");
  c.mqttPort = 1883;
  c.mqttTls = false;
  c.mqttTlsInsecure = true;
  setStr(c.mqttUser, sizeof(c.mqttUser), "");
  setStr(c.mqttPass, sizeof(c.mqttPass), "");
  setStr(c.mqttBaseTopic, sizeof(c.mqttBaseTopic), "parking-valet");
  c.mqttDiscovery = true;
  setStr(c.mqttDiscoveryPrefix, sizeof(c.mqttDiscoveryPrefix), "homeassistant");
  c.mqttIntervalS = 60;

  c.triggerMode        = TRIG_ANY_CHANGE;
  c.triggerThreshold   = 1;
  c.minSendIntervalMs  = 5000;
  c.heartbeatIntervalS = 0;

  c.captureIntervalMs = 1500;
  c.stableFrames      = 4;
  c.occupancyMode     = OCCUPANCY_RELATIVE;
  c.edgeThreshold     = 12.0f;
  c.relDelta          = 6.0f;
  c.hysteresis        = 0.25f;
  c.baselineEma       = 0.02f;

  c.occupancyEngine = 0;
  c.trainCapture = false;
  setStr(c.captureUrl, sizeof(c.captureUrl), "");
  setStr(c.captureAuthHeaderName, sizeof(c.captureAuthHeaderName), "");
  setStr(c.captureAuthHeaderValue, sizeof(c.captureAuthHeaderValue), "");
  c.captureTlsInsecure = true;

  c.framesize   = 9;     // FRAMESIZE_SVGA (800x600)
  c.jpegQuality = 12;
  c.vFlip       = false;
  c.hMirror     = false;
  c.brightness  = 0;
  c.contrast    = 0;
  c.saturation  = 0;
  c.awb         = true;
  c.aec         = true;
  c.afMode      = 1;     // focus once at boot (best for a fixed scene)
  c.tzOffsetMin = 0;     // UTC by default

  // Curb geometry (empty until user traces strips in the web UI)
  c.stripCount = 0;
  c.cellCount  = 0;

  // Curb tunables
  c.carPitchM      = 6.0f;
  c.clearInteriorM = 1.2f;
  c.clearEndM      = 1.8f;
  c.smoothMode     = 1;
  c.darkLumaThresh = 40.0f;
  c.pitchLearn     = 1;
}

// ---- full (de)serialization (always includes secrets) ---------------------
// serializeSettings emits everything EXCEPT the strips/cells arrays; serializeGeom
// emits only those. They are persisted under separate NVS keys so a large geometry
// can't push the frequently-saved settings blob over the partition limit, but the
// API/backup path (serializeFull / configToJson) still emits one combined object.

static void serializeSettings(const Config& c, JsonObject o) {
  o["version"] = c.version;

  o["staSsid"]  = c.staSsid;
  o["staPass"]  = c.staPass;
  o["apSsid"]   = c.apSsid;
  o["apPass"]   = c.apPass;
  o["hostname"] = c.hostname;
  o["offlineRebootMin"] = c.offlineRebootMin;
  o["apRetryMin"]       = c.apRetryMin;

  o["wgEnabled"]       = c.wgEnabled;
  o["wgPrivateKey"]    = c.wgPrivateKey;
  o["wgAddress"]       = c.wgAddress;
  o["wgPeerPublicKey"] = c.wgPeerPublicKey;
  o["wgEndpointHost"]  = c.wgEndpointHost;
  o["wgEndpointPort"]  = c.wgEndpointPort;
  o["wgAllowedIps"]    = c.wgAllowedIps;
  o["wgPresharedKey"]  = c.wgPresharedKey;
  o["wgKeepalive"]     = c.wgKeepalive;

  o["adminUser"]      = c.adminUser;
  o["adminPass"]      = c.adminPass;
  o["mustChangePass"] = c.mustChangePass;

  o["whEnabled"]          = c.whEnabled;
  o["whUrl"]              = c.whUrl;
  o["whAuthHeaderName"]   = c.whAuthHeaderName;
  o["whAuthHeaderValue"]  = c.whAuthHeaderValue;
  o["whTlsInsecure"]      = c.whTlsInsecure;

  o["spoolMode"]       = c.spoolMode;
  o["spoolMaxEntries"] = c.spoolMaxEntries;
  o["spoolMaxKB"]      = c.spoolMaxKB;
  o["spoolBackend"]    = c.spoolBackend;

  o["statsEnabled"]         = c.statsEnabled;
  o["statsUrl"]             = c.statsUrl;
  o["statsAuthHeaderName"]  = c.statsAuthHeaderName;
  o["statsAuthHeaderValue"] = c.statsAuthHeaderValue;
  o["statsTlsInsecure"]     = c.statsTlsInsecure;
  o["statsIntervalS"]       = c.statsIntervalS;

  o["mqttEnabled"]          = c.mqttEnabled;
  o["mqttHost"]             = c.mqttHost;
  o["mqttPort"]             = c.mqttPort;
  o["mqttTls"]              = c.mqttTls;
  o["mqttTlsInsecure"]      = c.mqttTlsInsecure;
  o["mqttUser"]             = c.mqttUser;
  o["mqttPass"]             = c.mqttPass;
  o["mqttBaseTopic"]        = c.mqttBaseTopic;
  o["mqttDiscovery"]        = c.mqttDiscovery;
  o["mqttDiscoveryPrefix"]  = c.mqttDiscoveryPrefix;
  o["mqttIntervalS"]        = c.mqttIntervalS;

  o["triggerMode"]        = c.triggerMode;
  o["triggerThreshold"]   = c.triggerThreshold;
  o["minSendIntervalMs"]  = c.minSendIntervalMs;
  o["heartbeatIntervalS"] = c.heartbeatIntervalS;

  o["captureIntervalMs"] = c.captureIntervalMs;
  o["stableFrames"]      = c.stableFrames;
  o["occupancyMode"]     = c.occupancyMode;
  o["edgeThreshold"]     = c.edgeThreshold;
  o["relDelta"]          = c.relDelta;
  o["hysteresis"]        = c.hysteresis;
  o["baselineEma"]       = c.baselineEma;

  o["occupancyEngine"]       = c.occupancyEngine;
  o["trainCapture"]          = c.trainCapture;
  o["captureUrl"]            = c.captureUrl;
  o["captureAuthHeaderName"] = c.captureAuthHeaderName;
  o["captureAuthHeaderValue"]= c.captureAuthHeaderValue;
  o["captureTlsInsecure"]    = c.captureTlsInsecure;

  o["framesize"]   = c.framesize;
  o["jpegQuality"] = c.jpegQuality;
  o["vFlip"]       = c.vFlip;
  o["hMirror"]     = c.hMirror;
  o["brightness"]  = c.brightness;
  o["contrast"]    = c.contrast;
  o["saturation"]  = c.saturation;
  o["awb"]         = c.awb;
  o["aec"]         = c.aec;
  o["afMode"]      = c.afMode;
  o["tzOffsetMin"] = c.tzOffsetMin;

  // Curb tunables
  o["carPitchM"]      = c.carPitchM;
  o["clearInteriorM"] = c.clearInteriorM;
  o["clearEndM"]      = c.clearEndM;
  o["smoothMode"]     = c.smoothMode;
  o["darkLumaThresh"] = c.darkLumaThresh;
  o["pitchLearn"]     = c.pitchLearn;
}

static void serializeGeom(const Config& c, JsonObject o) {
  // Curb strips array
  {
    JsonArray arr = o["strips"].to<JsonArray>();
    for (int i = 0; i < c.stripCount && i < MAX_STRIPS; i++) {
      JsonObject s = arr.add<JsonObject>();
      s["name"]       = c.strips[i].name;
      s["realLenM"]   = c.strips[i].realLenM;
      s["realWidthM"] = c.strips[i].realWidthM;
      s["nCells"]     = c.strips[i].nCells;
    }
  }

  // Curb cells array
  {
    JsonArray arr = o["cells"].to<JsonArray>();
    for (int i = 0; i < c.cellCount && i < MAX_CELLS; i++) {
      JsonObject cell = arr.add<JsonObject>();
      JsonArray px = cell["px"].to<JsonArray>();
      for (int j = 0; j < 4; j++) px.add(c.cells[i].px[j]);
      JsonArray py = cell["py"].to<JsonArray>();
      for (int j = 0; j < 4; j++) py.add(c.cells[i].py[j]);
      cell["lenM"]    = c.cells[i].lenM;
      cell["strip"]   = c.cells[i].strip;
      cell["enabled"] = c.cells[i].enabled;
    }
  }
}

static void serializeFull(const Config& c, JsonObject o) {
  serializeSettings(c, o);
  serializeGeom(c, o);
}

static void parseFull(Config& c, JsonObjectConst o) {
  // c is pre-seeded with defaults; only overwrite present keys.
  if (o["staSsid"].is<const char*>())  setStr(c.staSsid,  sizeof(c.staSsid),  o["staSsid"]);
  if (o["staPass"].is<const char*>())  setStr(c.staPass,  sizeof(c.staPass),  o["staPass"]);
  if (o["apSsid"].is<const char*>())   setStr(c.apSsid,   sizeof(c.apSsid),   o["apSsid"]);
  // AP password gates the whole setup surface: accept only empty (keep current) or
  // a valid WPA2 length (>=8). A too-short value is ignored so the AP is never open.
  if (o["apPass"].is<const char*>()) {
    const char* v = o["apPass"]; size_t vl = strlen(v);
    if (vl == 0 || vl >= 8) setStr(c.apPass, sizeof(c.apPass), v);
  }
  if (o["hostname"].is<const char*>()) setStr(c.hostname, sizeof(c.hostname), o["hostname"]);
  c.offlineRebootMin = o["offlineRebootMin"] | c.offlineRebootMin;
  c.apRetryMin       = o["apRetryMin"]       | c.apRetryMin;

  c.wgEnabled = o["wgEnabled"] | c.wgEnabled;
  if (o["wgPrivateKey"].is<const char*>())    setStr(c.wgPrivateKey,    sizeof(c.wgPrivateKey),    o["wgPrivateKey"]);
  if (o["wgAddress"].is<const char*>())       setStr(c.wgAddress,       sizeof(c.wgAddress),       o["wgAddress"]);
  if (o["wgPeerPublicKey"].is<const char*>()) setStr(c.wgPeerPublicKey, sizeof(c.wgPeerPublicKey), o["wgPeerPublicKey"]);
  if (o["wgEndpointHost"].is<const char*>())  setStr(c.wgEndpointHost,  sizeof(c.wgEndpointHost),  o["wgEndpointHost"]);
  c.wgEndpointPort = o["wgEndpointPort"] | c.wgEndpointPort;
  if (o["wgAllowedIps"].is<const char*>())    setStr(c.wgAllowedIps,    sizeof(c.wgAllowedIps),    o["wgAllowedIps"]);
  if (o["wgPresharedKey"].is<const char*>())  setStr(c.wgPresharedKey,  sizeof(c.wgPresharedKey),  o["wgPresharedKey"]);
  c.wgKeepalive = o["wgKeepalive"] | c.wgKeepalive;

  if (o["adminUser"].is<const char*>()) setStr(c.adminUser, sizeof(c.adminUser), o["adminUser"]);
  if (o["adminPass"].is<const char*>()) setStr(c.adminPass, sizeof(c.adminPass), o["adminPass"]);
  c.mustChangePass = o["mustChangePass"] | c.mustChangePass;

  c.whEnabled = o["whEnabled"] | c.whEnabled;
  if (o["whUrl"].is<const char*>())             setStr(c.whUrl,             sizeof(c.whUrl),             o["whUrl"]);
  if (o["whAuthHeaderName"].is<const char*>())  setStr(c.whAuthHeaderName,  sizeof(c.whAuthHeaderName),  o["whAuthHeaderName"]);
  if (o["whAuthHeaderValue"].is<const char*>()) setStr(c.whAuthHeaderValue, sizeof(c.whAuthHeaderValue), o["whAuthHeaderValue"]);
  c.whTlsInsecure = o["whTlsInsecure"] | c.whTlsInsecure;

  c.spoolMode       = o["spoolMode"]       | c.spoolMode;
  c.spoolMaxEntries = o["spoolMaxEntries"] | c.spoolMaxEntries;
  c.spoolMaxKB      = o["spoolMaxKB"]      | c.spoolMaxKB;
  c.spoolBackend    = o["spoolBackend"]    | c.spoolBackend;

  c.statsEnabled = o["statsEnabled"] | c.statsEnabled;
  if (o["statsUrl"].is<const char*>())             setStr(c.statsUrl,             sizeof(c.statsUrl),             o["statsUrl"]);
  if (o["statsAuthHeaderName"].is<const char*>())  setStr(c.statsAuthHeaderName,  sizeof(c.statsAuthHeaderName),  o["statsAuthHeaderName"]);
  if (o["statsAuthHeaderValue"].is<const char*>()) setStr(c.statsAuthHeaderValue, sizeof(c.statsAuthHeaderValue), o["statsAuthHeaderValue"]);
  c.statsTlsInsecure = o["statsTlsInsecure"] | c.statsTlsInsecure;
  c.statsIntervalS   = o["statsIntervalS"]   | c.statsIntervalS;

  c.mqttEnabled = o["mqttEnabled"] | c.mqttEnabled;
  if (o["mqttHost"].is<const char*>())            setStr(c.mqttHost,            sizeof(c.mqttHost),            o["mqttHost"]);
  c.mqttPort        = o["mqttPort"]        | c.mqttPort;
  c.mqttTls         = o["mqttTls"]         | c.mqttTls;
  c.mqttTlsInsecure = o["mqttTlsInsecure"] | c.mqttTlsInsecure;
  if (o["mqttUser"].is<const char*>())            setStr(c.mqttUser,            sizeof(c.mqttUser),            o["mqttUser"]);
  if (o["mqttPass"].is<const char*>())            setStr(c.mqttPass,            sizeof(c.mqttPass),            o["mqttPass"]);
  if (o["mqttBaseTopic"].is<const char*>())       setStr(c.mqttBaseTopic,       sizeof(c.mqttBaseTopic),       o["mqttBaseTopic"]);
  c.mqttDiscovery   = o["mqttDiscovery"]   | c.mqttDiscovery;
  if (o["mqttDiscoveryPrefix"].is<const char*>()) setStr(c.mqttDiscoveryPrefix, sizeof(c.mqttDiscoveryPrefix), o["mqttDiscoveryPrefix"]);
  c.mqttIntervalS   = o["mqttIntervalS"]   | c.mqttIntervalS;

  c.triggerMode        = o["triggerMode"]        | c.triggerMode;
  c.triggerThreshold   = o["triggerThreshold"]   | c.triggerThreshold;
  c.minSendIntervalMs  = o["minSendIntervalMs"]  | c.minSendIntervalMs;
  c.heartbeatIntervalS = o["heartbeatIntervalS"] | c.heartbeatIntervalS;

  c.captureIntervalMs = o["captureIntervalMs"] | c.captureIntervalMs;
  c.stableFrames      = o["stableFrames"]      | c.stableFrames;
  c.occupancyMode     = o["occupancyMode"]     | c.occupancyMode;
  c.edgeThreshold     = o["edgeThreshold"]     | c.edgeThreshold;
  c.relDelta          = o["relDelta"]          | c.relDelta;
  c.hysteresis        = o["hysteresis"]        | c.hysteresis;
  c.baselineEma       = o["baselineEma"]       | c.baselineEma;
  c.hysteresis        = constrain(c.hysteresis,  0.0f, 0.9f);
  c.baselineEma       = constrain(c.baselineEma, 0.0f, 1.0f);   // |1-a|<=1 so the EMA can't diverge

  c.occupancyEngine = o["occupancyEngine"] | c.occupancyEngine;
  c.trainCapture    = o["trainCapture"]    | c.trainCapture;
  if (o["captureUrl"].is<const char*>())             setStr(c.captureUrl,             sizeof(c.captureUrl),             o["captureUrl"]);
  if (o["captureAuthHeaderName"].is<const char*>())  setStr(c.captureAuthHeaderName,  sizeof(c.captureAuthHeaderName),  o["captureAuthHeaderName"]);
  if (o["captureAuthHeaderValue"].is<const char*>()) setStr(c.captureAuthHeaderValue, sizeof(c.captureAuthHeaderValue), o["captureAuthHeaderValue"]);
  c.captureTlsInsecure = o["captureTlsInsecure"] | c.captureTlsInsecure;

  c.framesize   = o["framesize"]   | c.framesize;
  c.jpegQuality = o["jpegQuality"] | c.jpegQuality;
  c.vFlip       = o["vFlip"]       | c.vFlip;
  c.hMirror     = o["hMirror"]     | c.hMirror;
  c.brightness  = o["brightness"]  | c.brightness;
  c.contrast    = o["contrast"]    | c.contrast;
  c.saturation  = o["saturation"]  | c.saturation;
  c.awb         = o["awb"]         | c.awb;
  c.aec         = o["aec"]         | c.aec;
  c.afMode      = o["afMode"]      | c.afMode;
  c.tzOffsetMin = o["tzOffsetMin"] | c.tzOffsetMin;

  // Curb tunables (merge, then clamp to sane ranges so hostile/buggy input can't
  // corrupt the headline numbers or diverge the baseline EMA).
  c.carPitchM      = o["carPitchM"]      | c.carPitchM;
  c.clearInteriorM = o["clearInteriorM"] | c.clearInteriorM;
  c.clearEndM      = o["clearEndM"]      | c.clearEndM;
  c.darkLumaThresh = o["darkLumaThresh"] | c.darkLumaThresh;
  c.carPitchM      = constrain(c.carPitchM,      0.5f, 30.0f);
  c.clearInteriorM = constrain(c.clearInteriorM, 0.0f, 10.0f);
  c.clearEndM      = constrain(c.clearEndM,      0.0f, 10.0f);
  c.darkLumaThresh = constrain(c.darkLumaThresh, 0.0f, 255.0f);
  // smoothMode enum: only 0 (none) and 1 (width-3 median) exist in the firmware.
  c.smoothMode = o["smoothMode"] | c.smoothMode;
  if (c.smoothMode > 1) c.smoothMode = 1;
  // pitchLearn is a uint8 flag; the web UI sends a JSON boolean for its checkbox,
  // and `bool | uint8_t` would silently drop it — accept both forms.
  if (o["pitchLearn"].is<bool>())     c.pitchLearn = o["pitchLearn"].as<bool>() ? 1 : 0;
  else                                c.pitchLearn = o["pitchLearn"] | c.pitchLearn;
  c.pitchLearn = c.pitchLearn ? 1 : 0;

  // Curb strips: pre-seed empty; overwrite only when present, so an old backup
  // without "strips" yields zero strips (user re-traces in the web UI).
  if (o["strips"].is<JsonArrayConst>()) {
    int n = 0;
    memset(c.strips, 0, sizeof(c.strips));
    for (JsonObjectConst s : o["strips"].as<JsonArrayConst>()) {
      if (n >= MAX_STRIPS) break;
      if (s["name"].is<const char*>()) setStr(c.strips[n].name, sizeof(c.strips[n].name), s["name"]);
      c.strips[n].realLenM   = s["realLenM"]   | 0.0f;
      c.strips[n].realWidthM = s["realWidthM"] | 0.0f;
      c.strips[n].nCells     = s["nCells"]     | 0;
      n++;
    }
    c.stripCount = n;
  }

  // Curb cells: pre-seed empty; overwrite only when present.
  if (o["cells"].is<JsonArrayConst>()) {
    int n = 0;
    memset(c.cells, 0, sizeof(c.cells));
    for (JsonObjectConst cell : o["cells"].as<JsonArrayConst>()) {
      if (n >= MAX_CELLS) break;
      // Clamp normalized vertices to [0,1]: a wild coordinate would make the overlay
      // Bresenham loop iterate for minutes (tripping the hang watchdog) and skew the
      // per-cell bounding box; cv.cpp additionally clamps at use, this is at rest.
      if (cell["px"].is<JsonArrayConst>()) {
        JsonArrayConst px = cell["px"].as<JsonArrayConst>();
        for (int j = 0; j < 4; j++) c.cells[n].px[j] = constrain((float)(px[j] | 0.0f), 0.0f, 1.0f);
      }
      if (cell["py"].is<JsonArrayConst>()) {
        JsonArrayConst py = cell["py"].as<JsonArrayConst>();
        for (int j = 0; j < 4; j++) c.cells[n].py[j] = constrain((float)(py[j] | 0.0f), 0.0f, 1.0f);
      }
      c.cells[n].lenM    = constrain((float)(cell["lenM"] | 0.0f), 0.0f, 30.0f);  // negative would corrupt free_curb_m
      int si             = cell["strip"] | (int)0;
      c.cells[n].enabled = cell["enabled"] | true;
      // Orphan cell (strip index past the parsed strip count) is excluded from the
      // per-strip free-gap scan; disable it so it also drops out of reliable_range_m
      // and occupied_fraction rather than silently under-reporting free curb.
      if (si < 0 || si >= c.stripCount) { si = 0; c.cells[n].enabled = false; }
      c.cells[n].strip   = (uint8_t)si;
      n++;
    }
    c.cellCount = n;
  }
}

// ---- NVS load / save ------------------------------------------------------

// Read one NVS blob and merge it into cfg via parseFull. Missing/oversize/corrupt
// blobs are skipped (cfg keeps its current values). Returns true if a blob was
// applied. Legacy single-blob configs (strips/cells embedded in CFG_KEY) still load
// because parseFull picks up whatever keys are present.
static bool loadBlobInto(Preferences& p, const char* key, Config& cfg) {
  size_t len = p.getBytesLength(key);
  if (len == 0 || len > 16384) return false;
  std::unique_ptr<char[]> buf(new char[len + 1]);
  size_t got = p.getBytes(key, buf.get(), len);
  if (got != len) return false;
  buf[len] = 0;
  JsonDocument doc;
  if (deserializeJson(doc, buf.get()) != DeserializationError::Ok) return false;
  parseFull(cfg, doc.as<JsonObjectConst>());
  return true;
}

bool configLoad(Config& cfg) {
  configLoadDefaults(cfg);

  Preferences p;
  if (!p.begin(CFG_NAMESPACE, /*readOnly=*/true)) return false;
  bool anySettings = loadBlobInto(p, CFG_KEY, cfg);       // settings (+ legacy embedded geometry)
  loadBlobInto(p, CFG_GEOM_KEY, cfg);                     // geometry (authoritative when present)
  p.end();
  cfg.version = CONFIG_VERSION;
  return anySettings;
}

bool configSave(const Config& cfg) {
  // Serialize settings and geometry into separate blobs so a large geometry never
  // pushes the frequently-saved settings blob over the NVS partition budget (a
  // single combined blob near MAX_CELLS could exceed it, since nvs_set_blob keeps
  // both the old and new copies during the atomic swap).
  JsonDocument sdoc; serializeSettings(cfg, sdoc.to<JsonObject>());
  JsonDocument gdoc; serializeGeom(cfg,     gdoc.to<JsonObject>());
  String sout; serializeJson(sdoc, sout);
  String gout; serializeJson(gdoc, gout);
  if (sout.isEmpty() || gout.isEmpty()) return false;   // serialization/alloc failure -> don't report success

  Preferences p;
  if (!p.begin(CFG_NAMESPACE, /*readOnly=*/false)) return false;
  size_t n1 = p.putBytes(CFG_KEY,      sout.c_str(), sout.length());
  size_t n2 = p.putBytes(CFG_GEOM_KEY, gout.c_str(), gout.length());
  p.end();
  return n1 == sout.length() && n2 == gout.length();
}

void configFactoryReset() {
  Preferences p;
  if (p.begin(CFG_NAMESPACE, false)) {
    p.clear();
    p.end();
  }
}

// ---- API (de)serialization ------------------------------------------------

void configToJson(const Config& cfg, JsonObject out, bool includeSecrets) {
  serializeFull(cfg, out);
  if (!includeSecrets) {
    out["staPass"]           = "";
    out["apPass"]            = "";
    out["adminPass"]         = "";
    out["whAuthHeaderValue"] = "";
    out["statsAuthHeaderValue"] = "";
    out["mqttPass"] = "";
    out["wgPrivateKey"]   = "";
    out["wgPresharedKey"] = "";
    out["captureAuthHeaderValue"] = "";
  }
}

bool configMergeJson(Config& cfg, JsonObjectConst in, bool* wifiChanged, bool* camChanged,
                     bool* mqttChanged, bool* wgChanged) {
  // Snapshot the bytes we care about for change detection.
  char prevSta[33], prevStaP[65], prevHost[33];
  setStr(prevSta, sizeof(prevSta), cfg.staSsid);
  setStr(prevStaP, sizeof(prevStaP), cfg.staPass);
  setStr(prevHost, sizeof(prevHost), cfg.hostname);
  int prevFs = cfg.framesize, prevQ = cfg.jpegQuality;
  bool prevVf = cfg.vFlip, prevHm = cfg.hMirror;
  int prevB = cfg.brightness, prevC = cfg.contrast, prevS = cfg.saturation;
  bool prevAwb = cfg.awb, prevAec = cfg.aec;
  int prevAf = cfg.afMode;
  char pMqH[128], pMqU[64], pMqP[64], pMqB[48], pMqDP[32];
  setStr(pMqH, sizeof(pMqH), cfg.mqttHost);  setStr(pMqU, sizeof(pMqU), cfg.mqttUser);
  setStr(pMqP, sizeof(pMqP), cfg.mqttPass);  setStr(pMqB, sizeof(pMqB), cfg.mqttBaseTopic);
  setStr(pMqDP, sizeof(pMqDP), cfg.mqttDiscoveryPrefix);
  bool pMqEn = cfg.mqttEnabled, pMqTls = cfg.mqttTls, pMqTi = cfg.mqttTlsInsecure, pMqDisc = cfg.mqttDiscovery;
  int pMqPort = cfg.mqttPort, pMqIv = cfg.mqttIntervalS;
  bool pWgEn = cfg.wgEnabled; int pWgPort = cfg.wgEndpointPort, pWgKa = cfg.wgKeepalive;
  char pWgPk[48], pWgAddr[24], pWgPub[48], pWgHost[64], pWgAip[24], pWgPsk[48];
  setStr(pWgPk, sizeof(pWgPk), cfg.wgPrivateKey);   setStr(pWgAddr, sizeof(pWgAddr), cfg.wgAddress);
  setStr(pWgPub, sizeof(pWgPub), cfg.wgPeerPublicKey); setStr(pWgHost, sizeof(pWgHost), cfg.wgEndpointHost);
  setStr(pWgAip, sizeof(pWgAip), cfg.wgAllowedIps);  setStr(pWgPsk, sizeof(pWgPsk), cfg.wgPresharedKey);

  // parseFull only overwrites present keys; but it would also overwrite secrets
  // with empty strings. For secrets we only apply a non-empty value (masked
  // round-trips send "" and must not wipe the stored value), so strip empty
  // secret keys into a filtered copy first.
  JsonDocument tmp;
  JsonObject t = tmp.to<JsonObject>();
  for (JsonPairConst kv : in) {
    const char* k = kv.key().c_str();
    if (isSecretKey(k)) {
      const char* v = kv.value().is<const char*>() ? kv.value().as<const char*>() : nullptr;
      if (!v || !v[0]) continue;
    }
    t[(const char*)k] = kv.value();
  }
  parseFull(cfg, tmp.as<JsonObjectConst>());

  if (wifiChanged) {
    *wifiChanged = strcmp(prevSta, cfg.staSsid) || strcmp(prevStaP, cfg.staPass) ||
                   strcmp(prevHost, cfg.hostname);
  }
  if (camChanged) {
    *camChanged = prevFs != cfg.framesize || prevQ != cfg.jpegQuality ||
                  prevVf != cfg.vFlip || prevHm != cfg.hMirror ||
                  prevB != cfg.brightness || prevC != cfg.contrast ||
                  prevS != cfg.saturation || prevAwb != cfg.awb || prevAec != cfg.aec ||
                  prevAf != cfg.afMode;
  }
  if (mqttChanged) {
    *mqttChanged = pMqEn != cfg.mqttEnabled || strcmp(pMqH, cfg.mqttHost) || pMqPort != cfg.mqttPort ||
                   pMqTls != cfg.mqttTls || pMqTi != cfg.mqttTlsInsecure ||
                   strcmp(pMqU, cfg.mqttUser) || strcmp(pMqP, cfg.mqttPass) ||
                   strcmp(pMqB, cfg.mqttBaseTopic) || pMqDisc != cfg.mqttDiscovery ||
                   strcmp(pMqDP, cfg.mqttDiscoveryPrefix) || pMqIv != cfg.mqttIntervalS;
  }
  if (wgChanged) {
    *wgChanged = pWgEn != cfg.wgEnabled || pWgPort != cfg.wgEndpointPort || pWgKa != cfg.wgKeepalive ||
                 strcmp(pWgPk, cfg.wgPrivateKey) || strcmp(pWgAddr, cfg.wgAddress) ||
                 strcmp(pWgPub, cfg.wgPeerPublicKey) || strcmp(pWgHost, cfg.wgEndpointHost) ||
                 strcmp(pWgAip, cfg.wgAllowedIps) || strcmp(pWgPsk, cfg.wgPresharedKey);
  }
  return true;
}
