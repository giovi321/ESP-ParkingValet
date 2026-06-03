#include "mqttc.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include "esp_heap_caps.h"
#include "net.h"
#include "clk.h"

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

static const Config*   s_cfg  = nullptr;
static const CvResult* s_last = nullptr;
static WiFiClient        s_plain;
static WiFiClientSecure  s_tls;
static PubSubClient      s_mqtt;
static uint32_t s_lastTry = 0;
static uint32_t s_lastPub = 0;
static bool     s_began   = false;

static const char* NODE       = "parkingvalet";   // stable HA object_id / unique_id prefix
static const char* DEVICE_ID  = "esp-parkingvalet";
static const char* GITHUB_URL = "https://github.com/giovi321/ESP-ParkingValet";

// key, friendly name, device_class, unit, entity_category, icon
struct Field { const char* key; const char* name; const char* dclass; const char* unit; const char* ecat; const char* icon; };
static const Field FIELDS[] = {
  {"count",      "Cars",        nullptr,           nullptr, nullptr,      "mdi:car"},
  {"rssi",       "Signal",      "signal_strength", "dBm",   "diagnostic", nullptr},
  {"ip",         "IP address",  nullptr,           nullptr, "diagnostic", "mdi:ip-network"},
  {"ssid",       "SSID",        nullptr,           nullptr, "diagnostic", "mdi:wifi"},
  {"uptime_s",   "Uptime",      "duration",        "s",     "diagnostic", nullptr},
  {"heap_free",  "Free heap",   "data_size",       "B",     "diagnostic", nullptr},
  {"psram_free", "Free PSRAM",  "data_size",       "B",     "diagnostic", nullptr},
  {"roi_count",  "Slots",       nullptr,           nullptr, "diagnostic", "mdi:select-group"},
  {"mode",       "WiFi mode",   nullptr,           nullptr, "diagnostic", "mdi:access-point"},
  {"version",    "Firmware",    nullptr,           nullptr, "diagnostic", "mdi:chip"},
  {"build",      "Build",       nullptr,           nullptr, "diagnostic", "mdi:source-commit"},
  {"time",       "Last update", "timestamp",       nullptr, "diagnostic", nullptr},
};
static const int NFIELDS = sizeof(FIELDS) / sizeof(FIELDS[0]);

static String baseTopic()  { return String(s_cfg->mqttBaseTopic[0] ? s_cfg->mqttBaseTopic : "parking-valet"); }
static String availTopic() { return baseTopic() + "/availability"; }

static String fieldValue(const char* k) {
  if (!strcmp(k, "count"))      return String(s_last && s_last->valid ? s_last->count : -1);
  if (!strcmp(k, "rssi"))       return String(WiFi.RSSI());
  if (!strcmp(k, "ip"))         return netGetStatus().ip;
  if (!strcmp(k, "ssid"))       return String(s_cfg->staSsid);
  if (!strcmp(k, "uptime_s"))   return String((uint32_t)(millis() / 1000));
  if (!strcmp(k, "heap_free"))  return String((uint32_t)ESP.getFreeHeap());
  if (!strcmp(k, "psram_free")) return String((uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
  if (!strcmp(k, "roi_count"))  return String(s_cfg->roiCount);
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
}

static void publishState() {
  const String base = baseTopic();
  for (int i = 0; i < NFIELDS; i++) {
    String v = fieldValue(FIELDS[i].key);
    if (!v.length()) continue;   // skip e.g. time before NTP sync
    s_mqtt.publish((base + "/" + FIELDS[i].key).c_str(), v.c_str(), true);   // retained
  }
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
  } else {
    log_w("MQTT connect to %s:%u failed (rc=%d)", s_cfg->mqttHost, (unsigned)s_cfg->mqttPort, s_mqtt.state());
  }
  return ok;
}

void mqttBegin(const Config* cfg, const CvResult* last) {
  s_cfg = cfg; s_last = last;
  if (cfg->mqttTls) {
    if (cfg->mqttTlsInsecure) s_tls.setInsecure();
    s_mqtt.setClient(s_tls);
  } else {
    s_mqtt.setClient(s_plain);
  }
  s_mqtt.setBufferSize(2048);
  s_mqtt.setKeepAlive(30);
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
  uint32_t iv = (s_cfg->mqttIntervalS ? s_cfg->mqttIntervalS : 60) * 1000UL;
  if (now - s_lastPub >= iv) { s_lastPub = now; publishState(); }
}

void mqttPublishNow() {
  if (s_began && s_cfg->mqttEnabled && s_mqtt.connected()) publishState();
}

bool mqttConnected() { return s_began && s_mqtt.connected(); }
