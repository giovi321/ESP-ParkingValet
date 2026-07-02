#include "config_store.h"
#include <Preferences.h>
#include <memory>

// ---- field tables ---------------------------------------------------------
// One table per field shape drives defaults, (de)serialization and the secret
// mask, so a new setting is a single line instead of four coordinated edits.
// Two fields are handled explicitly outside the tables because they need custom
// logic: apPass (WPA2 length guard) and pitchLearn (accept a JSON boolean). The
// strips/cells geometry arrays are structurally different and stay hand-written.

// X(name, defaultCStr, isSecret)
#define CONFIG_STRINGS(X) \
  X(staSsid,                "",                    0) \
  X(staPass,                "",                    1) \
  X(apSsid,                 DEFAULT_AP_SSID,       0) \
  X(hostname,               DEFAULT_HOSTNAME,      0) \
  X(wgPrivateKey,           "",                    1) \
  X(wgAddress,              "",                    0) \
  X(wgPeerPublicKey,        "",                    0) \
  X(wgEndpointHost,         "",                    0) \
  X(wgAllowedIps,           "",                    0) \
  X(wgPresharedKey,         "",                    1) \
  X(adminUser,              DEFAULT_ADMIN_USER,    0) \
  X(adminPass,              DEFAULT_ADMIN_PASS,    1) \
  X(whUrl,                  "http://192.168.1.197:5678/webhook/parking-cam", 0) \
  X(whAuthHeaderName,       "",                    0) \
  X(whAuthHeaderValue,      "",                    1) \
  X(statsUrl,               "",                    0) \
  X(statsAuthHeaderName,    "",                    0) \
  X(statsAuthHeaderValue,   "",                    1) \
  X(mqttHost,               "",                    0) \
  X(mqttUser,               "",                    0) \
  X(mqttPass,               "",                    1) \
  X(mqttBaseTopic,          "parking-valet",       0) \
  X(mqttDiscoveryPrefix,    "homeassistant",       0) \
  X(captureUrl,             "",                    0) \
  X(captureAuthHeaderName,  "",                    0) \
  X(captureAuthHeaderValue, "",                    1)

// X(ctype, name, defaultExpr) — parsed via `c.name = o["name"] | c.name`
#define CONFIG_SCALARS(X) \
  X(uint16_t, offlineRebootMin,  0) \
  X(uint16_t, apRetryMin,        0) \
  X(bool,     wgEnabled,         false) \
  X(uint16_t, wgEndpointPort,    51820) \
  X(uint16_t, wgKeepalive,       25) \
  X(bool,     mustChangePass,    true) \
  X(bool,     whEnabled,         false) \
  X(bool,     whTlsInsecure,     true) \
  X(uint8_t,  spoolMode,         SPOOL_COUNT) \
  X(uint16_t, spoolMaxEntries,   20) \
  X(uint16_t, spoolMaxKB,        96) \
  X(uint8_t,  spoolBackend,      SPOOL_BACKEND_AUTO) \
  X(bool,     statsEnabled,      false) \
  X(bool,     statsTlsInsecure,  true) \
  X(uint32_t, statsIntervalS,    300) \
  X(bool,     mqttEnabled,       false) \
  X(uint16_t, mqttPort,          1883) \
  X(bool,     mqttTls,           false) \
  X(bool,     mqttTlsInsecure,   true) \
  X(bool,     mqttDiscovery,     true) \
  X(uint16_t, mqttIntervalS,     60) \
  X(uint8_t,  triggerMode,       TRIG_ANY_CHANGE) \
  X(int,      triggerThreshold,  1) \
  X(uint32_t, minSendIntervalMs, 5000) \
  X(uint32_t, heartbeatIntervalS,0) \
  X(uint16_t, captureIntervalMs, 1500) \
  X(uint8_t,  stableFrames,      4) \
  X(uint8_t,  occupancyMode,     OCCUPANCY_RELATIVE) \
  X(float,    edgeThreshold,     12.0f) \
  X(float,    relDelta,          6.0f) \
  X(float,    hysteresis,        0.25f) \
  X(float,    baselineEma,       0.02f) \
  X(uint8_t,  occupancyEngine,   0) \
  X(bool,     trainCapture,      false) \
  X(bool,     captureTlsInsecure,true) \
  X(int,      framesize,         9) \
  X(int,      jpegQuality,       12) \
  X(bool,     vFlip,             false) \
  X(bool,     hMirror,           false) \
  X(int,      brightness,        0) \
  X(int,      contrast,          0) \
  X(int,      saturation,        0) \
  X(bool,     awb,               true) \
  X(bool,     aec,               true) \
  X(int,      afMode,            1) \
  X(int16_t,  tzOffsetMin,       0) \
  X(float,    carPitchM,         6.0f) \
  X(float,    clearInteriorM,    1.2f) \
  X(float,    clearEndM,         1.8f) \
  X(uint8_t,  smoothMode,        1) \
  X(float,    darkLumaThresh,    40.0f) \
  X(uint8_t,  pitchLearn,        1)

// ---- small helpers --------------------------------------------------------

static void setStr(char* dst, size_t cap, const char* src) {
  strlcpy(dst, src ? src : "", cap);
}

static bool isSecretKey(const char* k) {
#define X(name, def, sec) if ((sec) && !strcmp(k, #name)) return true;
  CONFIG_STRINGS(X)
#undef X
  return !strcmp(k, "apPass");   // secret, handled outside the table
}

// ---- defaults -------------------------------------------------------------

void configLoadDefaults(Config& c) {
  memset(&c, 0, sizeof(Config));
  c.version = CONFIG_VERSION;

#define X(name, def, sec) setStr(c.name, sizeof(c.name), def);
  CONFIG_STRINGS(X)
#undef X
  setStr(c.apPass, sizeof(c.apPass), DEFAULT_AP_PASS);   // explicit: length-guarded on write
#define X(ty, name, def) c.name = (def);
  CONFIG_SCALARS(X)
#undef X

  // Curb geometry (empty until user traces strips in the web UI)
  c.stripCount = 0;
  c.cellCount  = 0;
}

// ---- full (de)serialization (always includes secrets) ---------------------
// serializeSettings emits everything EXCEPT the strips/cells arrays; serializeGeom
// emits only those. They are persisted under separate NVS keys so a large geometry
// can't push the frequently-saved settings blob over the partition limit, but the
// API/backup path (serializeFull / configToJson) still emits one combined object.

static void serializeSettings(const Config& c, JsonObject o) {
  o["version"] = c.version;

#define X(name, def, sec) o[#name] = c.name;
  CONFIG_STRINGS(X)
#undef X
  o["apPass"] = c.apPass;
#define X(ty, name, def) o[#name] = c.name;
  CONFIG_SCALARS(X)
#undef X
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
  // Strings: apply only when the key is a string (secrets are pre-filtered by
  // configMergeJson so a masked empty value never lands here).
#define X(name, def, sec) if (o[#name].is<const char*>()) setStr(c.name, sizeof(c.name), o[#name]);
  CONFIG_STRINGS(X)
#undef X
  // apPass gates the whole setup surface: accept only empty (keep current) or a
  // valid WPA2 length (>=8) so a too-short value never opens the AP.
  if (o["apPass"].is<const char*>()) {
    const char* v = o["apPass"]; size_t vl = strlen(v);
    if (vl == 0 || vl >= 8) setStr(c.apPass, sizeof(c.apPass), v);
  }
  // Scalars: merge present keys, keep the current value otherwise.
#define X(ty, name, def) c.name = o[#name] | c.name;
  CONFIG_SCALARS(X)
#undef X
  // pitchLearn is a uint8 flag but the UI sends a JSON boolean for its checkbox;
  // `bool | uint8_t` above drops it, so accept the boolean form explicitly.
  if (o["pitchLearn"].is<bool>()) c.pitchLearn = o["pitchLearn"].as<bool>() ? 1 : 0;
  c.pitchLearn = c.pitchLearn ? 1 : 0;
  // Clamp so hostile/buggy input can't corrupt the numbers or diverge the EMA.
  c.hysteresis     = constrain(c.hysteresis,     0.0f, 0.9f);
  c.baselineEma    = constrain(c.baselineEma,    0.0f, 1.0f);   // |1-a|<=1 so the EMA can't diverge
  c.carPitchM      = constrain(c.carPitchM,      0.5f, 30.0f);
  c.clearInteriorM = constrain(c.clearInteriorM, 0.0f, 10.0f);
  c.clearEndM      = constrain(c.clearEndM,      0.0f, 10.0f);
  c.darkLumaThresh = constrain(c.darkLumaThresh, 0.0f, 255.0f);
  if (c.smoothMode > 1) c.smoothMode = 1;   // only 0=none, 1=width-3 median exist

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
#define X(name, def, sec) if (sec) out[#name] = "";
    CONFIG_STRINGS(X)
#undef X
    out["apPass"] = "";
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
