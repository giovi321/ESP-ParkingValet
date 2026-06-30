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
#include "spool.h"
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
extern void noteLoopAlive();  // hang-watchdog heartbeat (main.cpp)
extern void otaMarkValidIfPending();  // confirm a pending OTA image before a voluntary reboot (main.cpp)
extern void cvRecalibrate(int index); // "mark empty now": re-seed baselines (index<0 = all cells) (main.cpp)
extern void cvMarkOccupied(int index); // "mark occupied now": force one cell occupied (relative re-base) (main.cpp)

static WebServer   server(80);
static Config*     g_cfg  = nullptr;
static CurbResult* g_last = nullptr;

static bool     s_rebootPending = false;
static uint32_t s_rebootAt = 0;

static char     s_lastSendEvent[24] = "";
static int      s_lastSendCount = 0;
static int      s_lastSendCode  = 0;
static uint32_t s_lastSendAt    = 0;
static bool     s_otaAuthFail   = false;
static bool     s_otaBeginOk    = false;   // Update.begin() succeeded for the current upload
static bool     s_otaOk         = false;   // a complete, plausibly-sized image was flashed OK
static bool     s_camStopped    = false;   // camera deinit'd for an in-progress OTA

// Smallest upload we'll treat as a real firmware image. Guards against a zero- or
// near-empty multipart part reaching Update.end(true) (which, with no bytes
// written, flashes an uninitialized buffer into the partition header). Mirrors the
// web UI's client-side floor; the real image is ~1.2 MB.
static const size_t MIN_OTA_BYTES = 65536;

// --- helpers ---------------------------------------------------------------

// Effective admin username for Digest auth. Never empty: a blank adminUser (which
// the config API/UI can persist) must NOT silently disable auth, so it falls back
// to the factory default. Otherwise one blank-username save would open every
// endpoint — including the firmware-flashing /update — to anyone on the network.
static const char* authUser() {
  return g_cfg->adminUser[0] ? g_cfg->adminUser : DEFAULT_ADMIN_USER;
}

static bool requireAuth() {
  // AP/config mode is gated by the WPA2 AP password (physical proximity), so we
  // skip Digest there to keep first-time setup and captive portals smooth.
  // STA mode is network-exposed, so it always requires Digest auth.
  if (netIsAP()) return true;
  if (!server.authenticate(authUser(), g_cfg->adminPass)) {
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
  uint32_t spQ = 0, spB = 0; spoolStats(spQ, spB);
  doc["spoolMode"]      = g_cfg->spoolMode;
  doc["spoolQueued"]    = spQ;
  doc["spoolBytes"]     = spB;

  // doc["cv"] — diagnostics panel (#sCv: valid/decW/decH/tookMs)
  JsonObject cv = doc["cv"].to<JsonObject>();
  cv["valid"]  = g_last->valid;
  cv["decW"]   = g_last->decW;
  cv["decH"]   = g_last->decH;
  cv["tookMs"] = g_last->tookMs;

  // doc["curb"] — headline (poll() reads st.curb.*: free_curb_m/can_fit/est_free_spaces/reliable_range_m)
  JsonObject curb = doc["curb"].to<JsonObject>();
  curb["free_curb_m"]        = g_last->free_curb_m;
  curb["longest_free_run_m"] = g_last->longest_free_run_m;
  curb["can_fit"]            = g_last->can_fit;
  curb["est_free_spaces"]    = g_last->est_free_spaces;
  curb["reliable_range_m"]   = g_last->reliable_range_m;
  curb["occupied_fraction"]  = g_last->occupied_fraction;
  curb["dark"]               = g_last->dark;

  // doc["cells"] — per-cell array (renderOverlay/renderCellTable reads st.cells[i].occupied etc.)
  JsonArray cells = doc["cells"].to<JsonArray>();
  for (int i = 0; i < g_cfg->cellCount && i < MAX_CELLS; i++) {
    JsonObject o = cells.add<JsonObject>();
    const CurbCell& cell = g_cfg->cells[i];
    o["enabled"] = cell.enabled;
    o["strip"]   = cell.strip;
    o["lenM"]    = cell.lenM;
    JsonArray px = o["px"].to<JsonArray>();
    for (int j = 0; j < 4; j++) px.add(cell.px[j]);
    JsonArray py = o["py"].to<JsonArray>();
    for (int j = 0; j < 4; j++) py.add(cell.py[j]);
    if (g_last->valid && i < g_last->nCells) {
      const CellResult& cr = g_last->cells[i];
      o["edge"]         = cr.edge;
      o["meanI"]        = cr.meanI;
      o["baselineEdge"] = cr.baselineEdge;
      o["occupied"]     = cr.occupied;
      o["raw"]          = cr.rawOccupied;
      o["inRange"]      = cr.inRange;
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
    // TODO(T6-T10): full curb repoint - replace slots with curb fields in Task C3.4
    bool slots[MAX_CELLS];
    int n = g_last->valid ? g_last->nCells : 0;
    for (int i = 0; i < n && i < MAX_CELLS; i++) slots[i] = g_last->cells[i].occupied;
    int count = g_last->valid ? g_last->est_free_spaces : 0;
    int code = netSendEvent(*g_cfg, "test", fb->buf, fb->len, count, count, slots, n);
    esp_camera_fb_return(fb);
    webNoteSend("test", count, code);
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
  } else if (!strcmp(action, "clear_spool")) {
    spoolClear();
    server.send(200, "application/json", "{\"ok\":true}");
  } else if (!strcmp(action, "recalibrate")) {
    int slot = doc["slot"] | -1;   // -1 / absent = all cells; otherwise just that cell
    if (slot < -1 || slot >= MAX_CELLS) { server.send(400, "application/json", "{\"ok\":false,\"err\":\"bad slot\"}"); return; }
    cvRecalibrate(slot);           // re-seed empty baseline(s) from the current view
    server.send(200, "application/json", "{\"ok\":true}");
  } else if (!strcmp(action, "mark_occupied")) {
    int slot = doc["slot"] | -1;   // single cell only (no "all occupied")
    if (slot < 0 || slot >= MAX_CELLS) { server.send(400, "application/json", "{\"ok\":false,\"err\":\"bad slot\"}"); return; }
    cvMarkOccupied(slot);          // force this cell occupied; relative mode re-bases so it sticks
    server.send(200, "application/json", "{\"ok\":true}");
  } else {
    server.send(400, "application/json", "{\"ok\":false,\"err\":\"unknown action\"}");
  }
}

// Bring the camera back after an OTA that did NOT reboot (failed/aborted). The
// camera is stopped at OTA start because its continuous DMA into PSRAM shares
// the cache that flash writes disable, which hard-hangs the chip mid-write.
static void otaRestoreCamera() {
  if (s_camStopped) { s_camStopped = false; cameraInit(*g_cfg); log_i("camera restored after failed OTA"); }
}

static void handleOtaUpload() {
  HTTPUpload& up = server.upload();
  noteLoopAlive();   // loop() is blocked here for the whole upload; keep the hang watchdog fed per chunk
  if (up.status == UPLOAD_FILE_START) {
    s_otaAuthFail = false;
    s_otaBeginOk  = false;
    s_otaOk       = false;
    if (!server.authenticate(authUser(), g_cfg->adminPass)) {
      s_otaAuthFail = true;
      return;
    }
    log_i("OTA start: %s", up.filename.c_str());
    // Stop the camera BEFORE any flash write: its continuous DMA into PSRAM
    // collides with the cache being disabled during flash erase/write and locks
    // up the chip (recoverable only by a power cycle). Restored on failure.
    esp_camera_deinit();
    s_camStopped = true;
    if (Update.isRunning()) Update.abort();   // clear any stale/aborted attempt
    // Latch whether begin() actually succeeded: it can fail (e.g. a 4KB sector-
    // buffer malloc fail) WITHOUT setting Update's error flag, which would
    // otherwise make handleOtaDone report a no-op flash as success.
    s_otaBeginOk = Update.begin(UPDATE_SIZE_UNKNOWN);
    if (!s_otaBeginOk) log_e("OTA begin failed: %s", Update.errorString());
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (s_otaAuthFail || !s_otaBeginOk) return;
    if (Update.write(up.buf, up.currentSize) != up.currentSize)
      log_e("OTA write failed: %s", Update.errorString());
  } else if (up.status == UPLOAD_FILE_END) {
    if (s_otaAuthFail || !s_otaBeginOk) return;
    // Only finalize a plausibly-complete image. Calling end(true) with zero bytes
    // written flashes an uninitialized buffer into the partition header.
    if (up.totalSize >= MIN_OTA_BYTES && Update.end(true)) {
      s_otaOk = true;
      log_i("OTA ok: %u bytes", (unsigned)up.totalSize);
    } else {
      Update.abort();
      log_e("OTA end failed (%u bytes): %s", (unsigned)up.totalSize, Update.errorString());
    }
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    Update.abort();
    otaRestoreCamera();   // upload interrupted -> bring the camera back
    log_w("OTA aborted (upload interrupted)");
  }
}

static void handleOtaDone() {
  if (s_otaAuthFail) {
    s_otaAuthFail = false;
    server.requestAuthentication(DIGEST_AUTH, "ESP-ParkingValet", "Authentication required");
    return;
  }
  if (s_otaOk) {
    server.send(200, "application/json", "{\"ok\":true}");
    scheduleReboot(800);   // camera stays down; the reboot brings it back
  } else {
    otaRestoreCamera();    // failed/rejected update -> restore the camera we stopped at start
    // begin() can fail without latching an Update error, so give a useful reason.
    const char* err = !s_otaBeginOk ? "flash init failed (low memory?)" : Update.errorString();
    JsonDocument r; r["ok"] = false; r["err"] = err;   // surface the reason to the UI
    String out; serializeJson(r, out);
    server.send(200, "application/json", out);
  }
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

void webBegin(Config* cfg, CurbResult* last) {
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
  // Self-heal an orphaned OTA: if an upload stopped the camera but the
  // connection died without delivering UPLOAD_FILE_END/ABORTED (silent RST or
  // WiFi de-auth), neither callback restored it. Recover once we are back here
  // with no active client and no reboot pending. (The hang watchdog in main.cpp
  // covers the harsher case where handleClient() never returns at all.)
  if (s_camStopped && !s_rebootPending && !server.client().connected()) {
    if (Update.isRunning()) Update.abort();   // clear the half-written image (_size=0)
    otaRestoreCamera();
    log_w("camera restored after orphaned OTA upload");
  }
  server.handleClient();
  if (s_rebootPending && (int32_t)(millis() - s_rebootAt) >= 0) {
    otaMarkValidIfPending();   // a voluntary reboot must not revert an image that's been running fine
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
