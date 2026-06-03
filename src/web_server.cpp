#include "web_server.h"
#include <WebServer.h>
#include <Update.h>
#include <ArduinoJson.h>
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "net.h"
#include "camera.h"
#include "buttons.h"
#include "mqttc.h"
#include "web_ui.h"      // index_html_gz / index_html_gz_len (generated)
#include "logbuf.h"

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

extern int sendStatsNow();   // defined in main.cpp

static WebServer  server(80);
static Config*    g_cfg  = nullptr;
static CvResult*  g_last = nullptr;

static bool     s_rebootPending = false;
static uint32_t s_rebootAt = 0;

static char     s_lastSendEvent[24] = "";
static int      s_lastSendCount = 0;
static int      s_lastSendCode  = 0;
static uint32_t s_lastSendAt    = 0;
static bool     s_otaAuthFail   = false;

// --- helpers ---------------------------------------------------------------

static bool requireAuth() {
  // AP/config mode is gated by the WPA2 AP password (physical proximity), so we
  // skip Digest there to keep first-time setup and captive portals smooth.
  // STA mode is network-exposed, so it always requires Digest auth.
  if (netIsAP()) return true;
  if (g_cfg->adminUser[0] &&
      !server.authenticate(g_cfg->adminUser, g_cfg->adminPass)) {
    server.requestAuthentication(DIGEST_AUTH, "ESP-ParkingValet", "Authentication required");
    return false;
  }
  return true;
}

static void scheduleReboot(uint32_t inMs) {
  s_rebootPending = true;
  s_rebootAt = millis() + inMs;
}

// --- handlers --------------------------------------------------------------

static void handleRoot() {
  // Gating the document itself makes the browser prompt for Digest creds on
  // load (reliable), after which fetch()/img requests reuse them.
  if (!requireAuth()) return;
  server.sendHeader("Content-Encoding", "gzip");
  server.send_P(200, "text/html", (PGM_P)index_html_gz, index_html_gz_len);
}

static void handleState() {
  if (!requireAuth()) return;

  JsonDocument doc;
  doc["version"]        = PARKINGCAM_VERSION;
  doc["build"]          = BUILD_GIT_SHA;
  NetStatus ns = netGetStatus();
  doc["mode"]           = (ns.mode == NET_AP) ? "ap" : "sta";
  doc["ip"]             = ns.ip;
  doc["rssi"]           = ns.rssi;
  doc["ssid"]           = (ns.mode == NET_AP) ? g_cfg->apSsid : g_cfg->staSsid;
  doc["hostname"]       = g_cfg->hostname;
  doc["heapFree"]       = (uint32_t)ESP.getFreeHeap();
  doc["psramFree"]      = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  doc["uptimeS"]        = (uint32_t)(millis() / 1000);
  doc["mustChangePass"] = g_cfg->mustChangePass;
  doc["whEnabled"]      = g_cfg->whEnabled;
  doc["afStatus"]       = cameraFocusStatus();
  doc["mqttEnabled"]    = g_cfg->mqttEnabled;
  doc["mqttConnected"]  = mqttConnected();

  JsonObject cv = doc["cv"].to<JsonObject>();
  cv["valid"]  = g_last->valid;
  cv["count"]  = g_last->count;
  cv["decW"]   = g_last->decW;
  cv["decH"]   = g_last->decH;
  cv["tookMs"] = g_last->tookMs;

  JsonArray slots = doc["slots"].to<JsonArray>();
  for (int i = 0; i < g_cfg->roiCount && i < MAX_ROIS; i++) {
    JsonObject o = slots.add<JsonObject>();
    const Roi& r = g_cfg->rois[i];
    o["name"]    = r.name;
    o["enabled"] = r.enabled;
    JsonArray pts = o["points"].to<JsonArray>();
    for (int j = 0; j < r.nPoints && j < MAX_POLY; j++) {
      JsonObject p = pts.add<JsonObject>();
      p["x"] = r.px[j]; p["y"] = r.py[j];
    }
    if (g_last->valid && i < g_last->n) {
      const SlotResult& sr = g_last->slots[i];
      o["edge"]         = sr.edge;
      o["meanI"]        = sr.meanI;
      o["baselineEdge"] = sr.baselineEdge;
      o["threshold"]    = sr.threshold;
      o["occupied"]     = sr.occupied;
      o["raw"]          = sr.rawOccupied;
    }
  }

  JsonObject ls = doc["lastSend"].to<JsonObject>();
  ls["event"] = s_lastSendEvent;
  ls["count"] = s_lastSendCount;
  ls["code"]  = s_lastSendCode;
  ls["agoS"]  = s_lastSendAt ? (uint32_t)((millis() - s_lastSendAt) / 1000) : -1;

  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

static void handleConfigGet() {
  if (!requireAuth()) return;
  JsonDocument doc;
  configToJson(*g_cfg, doc.to<JsonObject>(), /*includeSecrets=*/false);
  String out;
  serializeJson(doc, out);
  server.send(200, "application/json", out);
}

static void handleConfigPost() {
  if (!requireAuth()) return;
  String body = server.arg("plain");
  if (body.isEmpty()) { server.send(400, "application/json", "{\"ok\":false,\"err\":\"empty body\"}"); return; }

  JsonDocument doc;
  if (deserializeJson(doc, body) != DeserializationError::Ok) {
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"bad json\"}");
    return;
  }

  bool wifiChanged = false, camChanged = false, mqttChanged = false;
  configMergeJson(*g_cfg, doc.as<JsonObjectConst>(), &wifiChanged, &camChanged, &mqttChanged);

  // Clear the forced-change flag once the password differs from the default.
  if (strcmp(g_cfg->adminPass, DEFAULT_ADMIN_PASS) != 0) g_cfg->mustChangePass = false;

  bool saved = configSave(*g_cfg);
  if (camChanged)  { cameraApplySettings(*g_cfg); cameraApplyFocus(*g_cfg); }
  if (mqttChanged) mqttReconfigure();

  JsonDocument resp;
  resp["ok"]     = saved;
  resp["reboot"] = wifiChanged;
  String out; serializeJson(resp, out);
  server.send(saved ? 200 : 500, "application/json", out);

  if (wifiChanged) scheduleReboot(900);   // re-join with new credentials cleanly
}

static void handleSnapshot() {
  if (!requireAuth()) return;
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { server.send(503, "text/plain", "no frame"); return; }
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "image/jpeg", (const char*)fb->buf, fb->len);  // sets Content-Length, no empty-body warning
  esp_camera_fb_return(fb);
}

static void handleLog() {
  if (!requireAuth()) return;
  const size_t cap = 6144;
  char* buf = (char*)malloc(cap + 1);
  if (!buf) { server.send(500, "text/plain", "oom"); return; }
  size_t n = logbufRead(buf, cap);
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/plain", buf, n);
  free(buf);
}

// Download the full config (including secrets) as a JSON backup file.
static void handleBackup() {
  if (!requireAuth()) return;
  JsonDocument doc;
  configToJson(*g_cfg, doc.to<JsonObject>(), /*includeSecrets=*/true);
  String out;
  serializeJson(doc, out);
  server.sendHeader("Content-Disposition", "attachment; filename=\"esp-parkingvalet-config.json\"");
  server.send(200, "application/json", out);
}

// Restore a config backup, then reboot to apply cleanly.
static void handleRestore() {
  if (!requireAuth()) return;
  String body = server.arg("plain");
  JsonDocument doc;
  if (body.isEmpty() || deserializeJson(doc, body) != DeserializationError::Ok) {
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"bad json\"}");
    return;
  }
  bool w = false, c = false, m = false;
  configMergeJson(*g_cfg, doc.as<JsonObjectConst>(), &w, &c, &m);
  bool saved = configSave(*g_cfg);
  server.send(saved ? 200 : 500, "application/json",
              saved ? "{\"ok\":true,\"reboot\":true}" : "{\"ok\":false}");
  if (saved) scheduleReboot(900);
}

static void handleAction() {
  if (!requireAuth()) return;
  String body = server.arg("plain");
  JsonDocument doc;
  deserializeJson(doc, body);
  const char* action = doc["action"] | "";

  if (!strcmp(action, "reboot")) {
    server.send(200, "application/json", "{\"ok\":true}");
    scheduleReboot(600);
  } else if (!strcmp(action, "factory_reset")) {
    server.send(200, "application/json", "{\"ok\":true}");
    configFactoryReset();
    scheduleReboot(600);
  } else if (!strcmp(action, "ap_mode")) {
    server.send(200, "application/json", "{\"ok\":true}");
    delay(300);
    buttonsForceApMode();   // sets RTC flag + restarts
  } else if (!strcmp(action, "test_webhook")) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) { server.send(503, "application/json", "{\"ok\":false,\"err\":\"no frame\"}"); return; }
    bool slots[MAX_ROIS];
    int n = (g_last->valid) ? g_last->n : 0;
    for (int i = 0; i < n && i < MAX_ROIS; i++) slots[i] = g_last->slots[i].occupied;
    int code = netSendEvent(*g_cfg, "test", fb->buf, fb->len,
                            g_last->valid ? g_last->count : 0,
                            g_last->valid ? g_last->count : 0, slots, n);
    esp_camera_fb_return(fb);
    webNoteSend("test", g_last->valid ? g_last->count : 0, code);
    JsonDocument r; r["ok"] = (code > 0 && code < 400); r["code"] = code;
    String out; serializeJson(r, out);
    server.send(200, "application/json", out);
  } else if (!strcmp(action, "test_stats")) {
    int code = sendStatsNow();
    JsonDocument r; r["ok"] = (code > 0 && code < 400); r["code"] = code;
    String out; serializeJson(r, out);
    server.send(200, "application/json", out);
  } else if (!strcmp(action, "af_focus")) {
    bool ok = cameraFocusNow();
    server.send(200, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false,\"err\":\"AF unavailable\"}");
  } else if (!strcmp(action, "test_mqtt")) {
    mqttPublishNow();
    server.send(200, "application/json", mqttConnected() ? "{\"ok\":true}" : "{\"ok\":false,\"err\":\"not connected\"}");
  } else {
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"unknown action\"}");
  }
}

static void handleOtaUpload() {
  HTTPUpload& up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    s_otaAuthFail = false;
    if (g_cfg->adminUser[0] && !server.authenticate(g_cfg->adminUser, g_cfg->adminPass)) {
      s_otaAuthFail = true;
      return;
    }
    log_i("OTA start: %s", up.filename.c_str());
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (s_otaAuthFail) return;
    if (Update.write(up.buf, up.currentSize) != up.currentSize) Update.printError(Serial);
  } else if (up.status == UPLOAD_FILE_END) {
    if (s_otaAuthFail) return;
    if (Update.end(true)) log_i("OTA ok: %u bytes", (unsigned)up.totalSize);
    else Update.printError(Serial);
  }
}

static void handleOtaDone() {
  if (s_otaAuthFail) {
    s_otaAuthFail = false;
    server.requestAuthentication(DIGEST_AUTH, "ESP-ParkingValet", "Authentication required");
    return;
  }
  bool ok = !Update.hasError();
  server.send(200, "application/json", ok ? "{\"ok\":true}" : "{\"ok\":false}");
  if (ok) scheduleReboot(800);
}

static void handleNotFound() {
  // Captive portal: in AP mode, bounce everything to the config page.
  if (netIsAP()) {
    server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
    server.send(302, "text/plain", "");
    return;
  }
  server.send(404, "text/plain", "not found");
}

// --- public ----------------------------------------------------------------

void webBegin(Config* cfg, CvResult* last) {
  g_cfg = cfg;
  g_last = last;

  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/state",  HTTP_GET,  handleState);
  server.on("/api/config", HTTP_GET,  handleConfigGet);
  server.on("/api/config", HTTP_POST, handleConfigPost);
  server.on("/api/action", HTTP_POST, handleAction);
  server.on("/snapshot",   HTTP_GET,  handleSnapshot);
  server.on("/api/log",    HTTP_GET,  handleLog);
  server.on("/api/backup", HTTP_GET,  handleBackup);
  server.on("/api/restore",HTTP_POST, handleRestore);
  server.on("/update",     HTTP_POST, handleOtaDone, handleOtaUpload);

  // Common captive-portal probe endpoints -> redirect to root in AP mode.
  server.on("/generate_204",       HTTP_GET, handleNotFound);
  server.on("/gen_204",            HTTP_GET, handleNotFound);
  server.on("/hotspot-detect.html",HTTP_GET, handleNotFound);
  server.on("/ncsi.txt",           HTTP_GET, handleNotFound);

  server.onNotFound(handleNotFound);
  server.begin();
  log_i("web server started on :80");
}

void webLoop() {
  server.handleClient();
  if (s_rebootPending && (int32_t)(millis() - s_rebootAt) >= 0) {
    log_w("rebooting (scheduled)");
    delay(50);
    ESP.restart();
  }
}

void webNoteSend(const char* event, int count, int httpCode) {
  strlcpy(s_lastSendEvent, event, sizeof(s_lastSendEvent));
  s_lastSendCount = count;
  s_lastSendCode  = httpCode;
  s_lastSendAt    = millis();
}
