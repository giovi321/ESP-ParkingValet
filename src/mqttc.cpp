#include "mqttc.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "esp_heap_caps.h"
#include "net.h"
#include "clk.h"
#include "overlay.h"

#ifndef PARKINGCAM_VERSION
#define PARKINGCAM_VERSION "0.0.0"
#endif
#if defined(__has_include)
#  if __has_include("build_info.h")
#    include "build_info.h"
#  endif
#endif
#ifndef BUILD_GIT_SHA
#define BUILD_GIT_SHA "dev"
#endif

#include "logbuf.h"   // log macros route to the web console (include last)

static const Config*     s_cfg  = nullptr;
static const CurbResult* s_last = nullptr;
static WiFiClient        s_plain;
static WiFiClientSecure  s_tls;
static PubSubClient      s_mqtt;
static uint32_t s_lastTry = 0;
static uint32_t s_lastPub = 0;
static bool     s_began   = false;

// Per-cell control reuses the CV actions that back the web UI buttons (main.cpp).
extern void cvRecalibrate(int index);   // "mark free": re-seed baseline(s) (index<0 = all)
extern void cvMarkOccupied(int index);  // "mark occupied": force one cell occupied

static volatile bool s_photoReq    = false;   // set by the receive callback, serviced in mqttLoop
static uint32_t      s_lastPhotoMs = 0;
static const uint32_t PHOTO_MIN_MS = 3000;    // rate-limit overlay photos
static uint32_t      s_roiSig      = 0;       // detect strip edits to refresh discovery
static const char*   SETSTATE_IDLE = "-";     // select idle option (ASCII, see options list)

static const char* NODE       = "parkingvalet";   // stable HA object_id / unique_id prefix
static const char* DEVICE_ID  = "esp-parkingvalet";
static const char* GITHUB_URL = "https://github.com/giovi321/ESP-ParkingValet";

// key, friendly name, device_class, unit, entity_category, icon
struct Field { const char* key; const char* name; const char* dclass; const char* unit; const char* ecat; const char* icon; };
static const Field FIELDS[] = {
  // TODO(T6-T10): "count"/"roi_count" rows replaced with curb headline fields in Task C3.5
  {"count",      "Free spaces",  nullptr,           nullptr, nullptr,      "mdi:car"},
  {"rssi",       "Signal",       "signal_strength", "dBm",   "diagnostic", nullptr},
  {"ip",         "IP address",   nullptr,           nullptr, "diagnostic", "mdi:ip-network"},
  {"ssid",       "SSID",         nullptr,           nullptr, "diagnostic", "mdi:wifi"},
  {"uptime_s",   "Uptime",       "duration",        "s",     "diagnostic", nullptr},
  {"heap_free",  "Free heap",    "data_size",       "B",     "diagnostic", nullptr},
  {"psram_free", "Free PSRAM",   "data_size",       "B",     "diagnostic", nullptr},
  {"roi_count",  "Cells",        nullptr,           nullptr, "diagnostic", "mdi:select-group"},
  {"mode",       "WiFi mode",    nullptr,           nullptr, "diagnostic", "mdi:access-point"},
  {"version",    "Firmware",     nullptr,           nullptr, "diagnostic", "mdi:chip"},
  {"build",      "Build",        nullptr,           nullptr, "diagnostic", "mdi:source-commit"},
  {"time",       "Last update",  "timestamp",       nullptr, "diagnostic", nullptr},
};
static const int NFIELDS = sizeof(FIELDS) / sizeof(FIELDS[0]);

static String baseTopic()  { return String(s_cfg->mqttBaseTopic[0] ? s_cfg->mqttBaseTopic : "parking-valet"); }
static String availTopic() { return baseTopic() + "/availability"; }

static String fieldValue(const char* k) {
  // TODO(T6-T10): "count" -> est_free_spaces and "roi_count" -> cell_count
  // repointed minimally here so it builds and emits sane values; full repoint in Task C3.5.
  if (!strcmp(k, "count"))      return String(s_last && s_last->valid ? s_last->est_free_spaces : -1);
  if (!strcmp(k, "rssi"))       return String(WiFi.RSSI());
  if (!strcmp(k, "ip"))         return netGetStatus().ip;
  if (!strcmp(k, "ssid"))       return String(s_cfg->staSsid);
  if (!strcmp(k, "uptime_s"))   return String((uint32_t)(millis() / 1000));
  if (!strcmp(k, "heap_free"))  return String((uint32_t)ESP.getFreeHeap());
  if (!strcmp(k, "psram_free")) return String((uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
  if (!strcmp(k, "roi_count"))  return String(s_cfg->cellCount);   // TODO(T6-T10): key rename in C3.5
  if (!strcmp(k, "mode"))       return netIsAP() ? "ap" : "sta";
  if (!strcmp(k, "version"))    return String(PARKINGCAM_VERSION);
  if (!strcmp(k, "build"))      return String(BUILD_GIT_SHA);
  if (!strcmp(k, "time"))       return clockIso();
  return String("");
}

static void addDevice(JsonObject o) {
  JsonObject dev = o["dev"].to<JsonObject>();
  dev["ids"].to<JsonArray>().add(DEVICE_ID);
  dev["name"] = "ESP-ParkingValet";
  dev["mdl"]  = "ESP32-CAM (OV5640)";
  dev["mf"]   = "giovi321";
  dev["sw"]   = String(PARKINGCAM_VERSION) + " (" + BUILD_GIT_SHA + ")";
  dev["cu"]   = GITHUB_URL;
}

// Cheap hash of strip count + names, to detect strip edits and refresh HA entities.
// (Cell count is not hashed here; bay discovery is per-strip in T1.)
static uint32_t roiSig() {
  uint32_t hsh = 2166136261u;
  auto mix = [&](uint32_t v) { hsh ^= v; hsh *= 16777619u; };
  mix((uint32_t)s_cfg->stripCount);
  for (int i = 0; i < s_cfg->stripCount && i < MAX_STRIPS; i++) {
    const CurbStrip& s = s_cfg->strips[i];
    for (const char* p = s.name; *p; p++) mix((uint8_t)*p);
  }
  return hsh;
}

// Publish one HA discovery config (retained) under <prefix>/<component>/<node>/<obj>/config.
static void publishCfg(const char* component, const String& obj, JsonDocument& d) {
  const String prefix = s_cfg->mqttDiscoveryPrefix[0] ? s_cfg->mqttDiscoveryPrefix : "homeassistant";
  char payload[640];
  size_t n = serializeJson(d, payload, sizeof(payload));
  String topic = prefix + "/" + component + "/" + NODE + "/" + obj + "/config";
  s_mqtt.publish(topic.c_str(), (const uint8_t*)payload, n, true);
}
static void clearCfg(const char* component, const String& obj) {
  const String prefix = s_cfg->mqttDiscoveryPrefix[0] ? s_cfg->mqttDiscoveryPrefix : "homeassistant";
  String topic = prefix + "/" + component + "/" + NODE + "/" + obj + "/config";
  s_mqtt.publish(topic.c_str(), (const uint8_t*)"", 0, true);   // empty retained = remove entity
}

// TODO(T6-T10): per-strip discovery replaces per-bay in Task C3.5. For now
// we adapt the bay entities to iterate over strips so the code compiles and
// emits sane HA entities. Full per-cell/strip repoint happens in C3.5.
static void publishBayDiscovery() {
  if (!s_cfg->mqttDiscovery) return;
  const String base = baseTopic();
  const String avty = availTopic();
  for (int i = 0; i < s_cfg->stripCount && i < MAX_STRIPS; i++) {
    const char* nm = s_cfg->strips[i].name[0] ? s_cfg->strips[i].name : "Strip";
    {
      JsonDocument d;
      d["name"]    = nm;
      d["uniq_id"] = String(NODE) + "_bay" + i;
      d["stat_t"]  = base + "/bay/" + i + "/state";
      d["avty_t"]  = avty;
      d["dev_cla"] = "occupancy";
      addDevice(d.as<JsonObject>());
      publishCfg("binary_sensor", String("bay") + i, d);
    }
    {
      JsonDocument d;
      d["name"]    = String(nm) + " control";
      d["uniq_id"] = String(NODE) + "_bay" + i + "_set";
      d["cmd_t"]   = base + "/bay/" + i + "/set";
      d["stat_t"]  = base + "/bay/" + i + "/setstate";
      d["avty_t"]  = avty;
      JsonArray opt = d["options"].to<JsonArray>();
      opt.add(SETSTATE_IDLE); opt.add("free"); opt.add("occupied");
      addDevice(d.as<JsonObject>());
      publishCfg("select", String("bay") + i + "_set", d);
    }
  }
  // Remove entities for dropped strips (clear up to MAX_STRIPS).
  for (int i = s_cfg->stripCount; i < MAX_STRIPS; i++) {
    clearCfg("binary_sensor", String("bay") + i);
    clearCfg("select", String("bay") + i + "_set");
  }
  // Snapshot camera + two control buttons (published once with the strip configs).
  {
    JsonDocument d;
    d["name"]    = "Snapshot";
    d["uniq_id"] = String(NODE) + "_photo";
    d["t"]       = base + "/photo";
    d["avty_t"]  = avty;
    addDevice(d.as<JsonObject>());
    publishCfg("camera", "photo", d);
  }
  {
    JsonDocument d;
    d["name"]    = "Take photo";
    d["uniq_id"] = String(NODE) + "_take_photo";
    d["cmd_t"]   = base + "/cmd/photo";
    d["pl_prs"]  = "1";
    d["avty_t"]  = avty;
    d["ic"]      = "mdi:camera";
    addDevice(d.as<JsonObject>());
    publishCfg("button", "take_photo", d);
  }
  {
    JsonDocument d;
    d["name"]    = "Mark all free";
    d["uniq_id"] = String(NODE) + "_mark_all_free";
    d["cmd_t"]   = base + "/cmd/mark_all_free";
    d["pl_prs"]  = "1";
    d["avty_t"]  = avty;
    d["ic"]      = "mdi:broom";
    addDevice(d.as<JsonObject>());
    publishCfg("button", "mark_all_free", d);
  }
}

// TODO(T6-T10): publish per-cell/strip state in Task C3.5.
// For now publish per-strip occupancy as a best-effort placeholder.
static void publishBayState() {
  const String base = baseTopic();
  for (int i = 0; i < s_cfg->stripCount && i < MAX_STRIPS; i++) {
    // Use the first cell of this strip as the strip-level occupancy indicator.
    bool occ = false;
    if (s_last && s_last->valid) {
      for (int j = 0; j < s_last->nCells && j < MAX_CELLS; j++) {
        // strip membership is in cfg.cells[j].strip (CurbCell), not CellResult
        if (s_cfg->cells[j].strip == (uint8_t)i) { occ = s_last->cells[j].occupied; break; }
      }
    }
    s_mqtt.publish((base + "/bay/" + i + "/state").c_str(), occ ? "ON" : "OFF", true);
  }
}

static void publishBayIdle() {
  const String base = baseTopic();
  for (int i = 0; i < s_cfg->stripCount && i < MAX_STRIPS; i++)
    s_mqtt.publish((base + "/bay/" + i + "/setstate").c_str(), SETSTATE_IDLE, true);
}

static void publishDiscovery() {
  if (!s_cfg->mqttDiscovery) return;
  const String base   = baseTopic();
  const String avty   = availTopic();
  const String prefix = s_cfg->mqttDiscoveryPrefix[0] ? s_cfg->mqttDiscoveryPrefix : "homeassistant";
  for (int i = 0; i < NFIELDS; i++) {
    const Field& f = FIELDS[i];
    JsonDocument d;
    d["name"]    = f.name;
    d["uniq_id"] = String(NODE) + "_" + f.key;
    d["stat_t"]  = base + "/" + f.key;
    d["avty_t"]  = avty;
    if (f.dclass) d["dev_cla"]      = f.dclass;
    if (f.unit)   d["unit_of_meas"] = f.unit;
    if (f.ecat)   d["ent_cat"]      = f.ecat;
    if (f.icon)   d["ic"]           = f.icon;
    addDevice(d.as<JsonObject>());
    char payload[1024];
    size_t n = serializeJson(d, payload, sizeof(payload));
    String topic = prefix + "/sensor/" + NODE + "/" + f.key + "/config";
    s_mqtt.publish(topic.c_str(), (const uint8_t*)payload, n, true);   // retained
  }
  publishBayDiscovery();
}

static void publishState() {
  const String base = baseTopic();
  for (int i = 0; i < NFIELDS; i++) {
    String v = fieldValue(FIELDS[i].key);
    if (!v.length()) continue;   // skip e.g. time before NTP sync
    s_mqtt.publish((base + "/" + FIELDS[i].key).c_str(), v.c_str(), true);   // retained
  }
  publishBayState();
}

static void onMqttMessage(char* topic, uint8_t* payload, unsigned int len) {
  const String base = baseTopic();
  String t(topic);
  char body[16] = {0};
  unsigned int n = len < sizeof(body) - 1 ? len : sizeof(body) - 1;
  memcpy(body, payload, n);

  if (t == base + "/cmd/photo") { s_photoReq = true; return; }
  if (t == base + "/cmd/mark_all_free") { cvRecalibrate(-1); return; }

  // base/bay/<i>/set — strip-level control (full per-cell repoint in Task C3.5)
  String pre = base + "/bay/";
  if (t.startsWith(pre) && t.endsWith("/set")) {
    String idx = t.substring(pre.length(), t.length() - 4);
    if (idx.length() == 0) return;
    for (size_t k = 0; k < idx.length(); k++) if (!isDigit(idx[k])) return;
    int i = idx.toInt();
    if (i < 0 || i >= MAX_STRIPS) return;
    if (!strcmp(body, "free"))          cvRecalibrate(i);
    else if (!strcmp(body, "occupied")) cvMarkOccupied(i);
    else return;   // ignore the "-" idle echo
    s_mqtt.publish((base + "/bay/" + i + "/setstate").c_str(), SETSTATE_IDLE, true);  // reset so the same pick re-fires
  }
}

static void subscribeCommands() {
  const String base = baseTopic();
  s_mqtt.subscribe((base + "/cmd/photo").c_str());
  s_mqtt.subscribe((base + "/cmd/mark_all_free").c_str());
  s_mqtt.subscribe((base + "/bay/+/set").c_str());
}

static bool connectNow() {
  if (!s_cfg->mqttHost[0]) return false;
  s_mqtt.setServer(s_cfg->mqttHost, s_cfg->mqttPort ? s_cfg->mqttPort : 1883);
  const char* user = s_cfg->mqttUser[0] ? s_cfg->mqttUser : nullptr;
  const char* pass = s_cfg->mqttPass[0] ? s_cfg->mqttPass : nullptr;
  String cid  = String(s_cfg->hostname) + "-" + String((uint32_t)ESP.getEfuseMac(), HEX);
  String avty = availTopic();
  bool ok = s_mqtt.connect(cid.c_str(), user, pass, avty.c_str(), 0, true, "offline");
  if (ok) {
    log_i("MQTT connected to %s:%u", s_cfg->mqttHost, (unsigned)s_cfg->mqttPort);
    s_mqtt.publish(avty.c_str(), "online", true);
    publishDiscovery();
    publishState();
    subscribeCommands();
    publishBayIdle();
    s_roiSig = roiSig();
  } else {
    log_w("MQTT connect to %s:%u failed (rc=%d)", s_cfg->mqttHost, (unsigned)s_cfg->mqttPort, s_mqtt.state());
  }
  return ok;
}

void mqttBegin(const Config* cfg, const CurbResult* last) {
  s_cfg = cfg; s_last = last;
  if (cfg->mqttTls) {
    if (cfg->mqttTlsInsecure) s_tls.setInsecure();
    s_mqtt.setClient(s_tls);
  } else {
    s_mqtt.setClient(s_plain);
  }
  s_mqtt.setBufferSize(2048);
  s_mqtt.setKeepAlive(30);
  s_mqtt.setCallback(onMqttMessage);
  s_began = true;
}

void mqttReconfigure() {
  if (!s_began) return;
  s_mqtt.disconnect();
  if (s_cfg->mqttTls) {
    if (s_cfg->mqttTlsInsecure) s_tls.setInsecure();
    s_mqtt.setClient(s_tls);
  } else {
    s_mqtt.setClient(s_plain);
  }
  s_lastTry = 0;   // reconnect promptly with the new settings
}

// Render the overlay photo and stream it to base/photo. The JPEG is far larger
// than the 2048-byte client buffer, so it MUST be streamed with beginPublish/
// write/endPublish rather than a single publish().
static void publishPhoto() {
  if (!s_mqtt.connected()) return;
  if (!s_last) return;
  uint8_t* jpg = nullptr;
  size_t len = overlayRenderJpeg(*s_cfg, *s_last, &jpg);
  if (!len || !jpg) return;
  String topic = baseTopic() + "/photo";
  if (s_mqtt.beginPublish(topic.c_str(), len, /*retained=*/true)) {
    size_t off = 0;
    bool writeOk = true;
    while (off < len) {
      size_t chunk = (len - off) < 512 ? (len - off) : 512;
      if (s_mqtt.write(jpg + off, chunk) != chunk) {
        log_w("MQTT photo write short at off=%u len=%u", (unsigned)off, (unsigned)chunk);
        writeOk = false;
        break;
      }
      off += chunk;
    }
    s_mqtt.endPublish();
    if (writeOk) log_i("MQTT photo published: %u bytes", (unsigned)len);
  } else {
    log_w("MQTT beginPublish failed for photo (%u bytes)", (unsigned)len);
  }
  free(jpg);
}

void mqttLoop() {
  if (!s_began || !s_cfg->mqttEnabled || !s_cfg->mqttHost[0]) return;
  if (netIsAP() || WiFi.status() != WL_CONNECTED) return;
  uint32_t now = millis();
  if (!s_mqtt.connected()) {
    if (now - s_lastTry < 5000) return;
    s_lastTry = now;
    connectNow();
    return;
  }
  s_mqtt.loop();

  // Refresh per-strip HA entities when the strip set/name changes.
  uint32_t sig = roiSig();
  if (sig != s_roiSig) { s_roiSig = sig; publishBayDiscovery(); publishBayState(); publishBayIdle(); }

  if (s_photoReq && now - s_lastPhotoMs > PHOTO_MIN_MS) {
    s_photoReq = false; s_lastPhotoMs = now;
    publishPhoto();
  }

  uint32_t iv = (s_cfg->mqttIntervalS ? s_cfg->mqttIntervalS : 60) * 1000UL;
  if (now - s_lastPub >= iv) { s_lastPub = now; publishState(); }
}

void mqttPublishNow() {
  if (s_began && s_cfg->mqttEnabled && s_mqtt.connected()) publishState();
}

bool mqttConnected() { return s_began && s_mqtt.connected(); }
