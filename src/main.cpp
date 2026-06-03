#include <Arduino.h>
#include "esp_camera.h"
#include "config_store.h"
#include "camera.h"
#include "cv.h"
#include "net.h"
#include "buttons.h"
#include "web_server.h"
#include "camera_pins.h"
#include <WiFi.h>
#include <ArduinoJson.h>
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "clk.h"
#include "mqttc.h"
#include "logbuf.h"

#if defined(__has_include)
#  if __has_include("build_info.h")
#    include "build_info.h"
#  endif
#endif
#ifndef BUILD_GIT_SHA
#define BUILD_GIT_SHA "dev"
#endif

static Config   cfg;
static CvEngine cvEngine;
static CvResult lastResult;

static int      lastSentCount   = -1;   // -1 = not initialized (don't send on boot)
static uint32_t lastCaptureMs   = 0;
static uint32_t lastSendMs      = 0;
static uint32_t lastHeartbeatMs = 0;
static int      lastMqttCount   = -2;

// --- status LED (GPIO25 on this board) ------------------------------------
static inline void ledWrite(bool on) {
  digitalWrite(PIN_STATUS_LED, (on != (bool)LED_ACTIVE_LOW) ? HIGH : LOW);
}
static void ledUpdate() {
  static uint32_t t = 0; static bool on = false;
  uint32_t now = millis();
  if (netIsAP()) {                              // AP / config mode: slow blink
    if (now - t >= 500) { t = now; on = !on; ledWrite(on); }
  } else if (WiFi.status() != WL_CONNECTED) {   // (re)connecting: fast blink
    if (now - t >= 120) { t = now; on = !on; ledWrite(on); }
  } else {                                      // connected & running: heartbeat
    ledWrite((now % 3000) < 60);
  }
}

// Decide whether this committed count warrants a webhook, per the trigger rule.
static bool shouldSend(int prev, int count) {
  if (cfg.triggerMode == TRIG_THRESHOLD) {
    int N = cfg.triggerThreshold;
    return (prev < N) != (count < N);   // crossing the threshold in either direction
  }
  return count != prev;                  // TRIG_ANY_CHANGE
}

static void postEvent(const char* event, camera_fb_t* fb, const CvResult& r, int count, int prev) {
  bool slots[MAX_ROIS];
  for (int i = 0; i < r.n && i < MAX_ROIS; i++) slots[i] = r.slots[i].occupied;
  int code = netSendEvent(cfg, event, fb->buf, fb->len, count, prev, slots, r.n);
  webNoteSend(event, count, code);
  lastSendMs = millis();
}

static void maybeSend(camera_fb_t* fb, const CvResult& r) {
  if (!r.valid) return;
  uint32_t now = millis();

  // Heartbeat (independent of count changes).
  if (cfg.whEnabled && cfg.heartbeatIntervalS > 0 &&
      now - lastHeartbeatMs >= cfg.heartbeatIntervalS * 1000UL) {
    lastHeartbeatMs = now;
    postEvent("heartbeat", fb, r, r.count, lastSentCount < 0 ? r.count : lastSentCount);
  }

  // First valid frame: adopt as baseline silently.
  if (lastSentCount < 0) { lastSentCount = r.count; return; }

  if (!cfg.whEnabled) { lastSentCount = r.count; return; }
  if (!shouldSend(lastSentCount, r.count)) return;
  if (now - lastSendMs < cfg.minSendIntervalMs) return;   // rate-limited; retry next cycle

  postEvent("count_changed", fb, r, r.count, lastSentCount);
  lastSentCount = r.count;
}

// --- stats / telemetry webhook ---------------------------------------------
static uint32_t lastStatsMs = 0;

static void buildStatsJson(String& out) {
  JsonDocument d;
  d["device"]   = cfg.hostname;
  d["version"]  = PARKINGCAM_VERSION;
  d["build"]    = BUILD_GIT_SHA;
  NetStatus ns  = netGetStatus();
  d["mode"]     = ns.mode == NET_AP ? "ap" : "sta";
  d["ip"]       = ns.ip;
  d["rssi"]     = ns.rssi;
  d["ssid"]     = ns.mode == NET_AP ? cfg.apSsid : cfg.staSsid;
  d["mac"]      = WiFi.macAddress();
  d["uptime_s"] = (uint32_t)(millis() / 1000);
  d["heap_free"]    = (uint32_t)ESP.getFreeHeap();
  d["psram_free"]   = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  d["reset_reason"] = (int)esp_reset_reason();
  d["roi_count"]    = cfg.roiCount;
  d["count"]        = lastResult.valid ? lastResult.count : -1;
  if (lastResult.valid) {
    d["cv_ms"]    = lastResult.tookMs;
    d["analysis"] = String(lastResult.decW) + "x" + String(lastResult.decH);
  }
  d["webhook_enabled"] = cfg.whEnabled;
  d["ts"]   = (uint32_t)clockEpoch();   // UTC epoch seconds (0 until NTP-synced)
  d["time"] = clockIso();               // ISO8601 UTC string
  serializeJson(d, out);
}

// Build + POST the stats payload now. Non-static so the web "test" action can call it.
int sendStatsNow() {
  String body;
  buildStatsJson(body);
  int code = netPostJson(cfg.statsUrl, cfg.statsAuthHeaderName, cfg.statsAuthHeaderValue,
                         cfg.statsTlsInsecure, body);
  log_i("stats POST %s -> HTTP %d (%u bytes)", cfg.statsUrl, code, (unsigned)body.length());
  return code;
}

static void maybeSendStats() {
  if (!cfg.statsEnabled || !cfg.statsUrl[0]) return;
  if (netIsAP() || WiFi.status() != WL_CONNECTED) return;
  uint32_t now = millis();
  uint32_t interval = (cfg.statsIntervalS ? cfg.statsIntervalS : 300) * 1000UL;
  if (now - lastStatsMs < interval) return;
  lastStatsMs = now;
  sendStatsNow();
}

void setup() {
  Serial.begin(115200);
  delay(200);
  logbufBegin();
  log_i("ESP-ParkingValet %s (%s) booting", PARKINGCAM_VERSION, BUILD_GIT_SHA);

  pinMode(PIN_STATUS_LED, OUTPUT);
  buttonsBegin();
  bool forcedAp = buttonsConsumeForcedAp();

  configLoad(cfg);
  log_i("config: host=%s sta='%s' rois=%d webhook=%s",
        cfg.hostname, cfg.staSsid, cfg.roiCount, cfg.whEnabled ? "on" : "off");

#ifdef PARKINGCAM_BUTTON_DISCOVERY
  buttonsDiscoveryScan();
#endif

  if (!cameraInit(cfg)) {
    log_e("camera init failed (check the OV5640 pin map / wiring)");
  } else {
    log_i("camera OK");
  }

  cvEngine.begin(&cfg);
  lastResult.valid = false;

  netBegin(&cfg);
  bool sta = false;
  if (!forcedAp) sta = netStartSTA();
  if (!sta) {
    bool fromFailure = !forcedAp && cfg.staSsid[0] != 0;   // had creds but couldn't join
    log_i("%s", forcedAp ? "entering AP config mode (button)"
                         : (fromFailure ? "STA join failed -> AP (offline watchdog armed)"
                                        : "no WiFi creds -> AP config mode"));
    netStartAP(fromFailure);
  }

  clockBegin();   // start NTP (syncs once online)
  webBegin(&cfg, &lastResult);
  mqttBegin(&cfg, &lastResult);
  log_i("ready. mode=%s ip=%s", netIsAP() ? "AP" : "STA", netGetStatus().ip.c_str());
}

void loop() {
  buttonsLoop();
  netLoop();
  webLoop();
  ledUpdate();
  maybeSendStats();
  mqttLoop();

  uint32_t now = millis();
  if (now - lastCaptureMs >= cfg.captureIntervalMs) {
    lastCaptureMs = now;
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      CvResult r;
      if (cvEngine.analyze(fb->buf, fb->len, fb->width, fb->height, r)) {
        lastResult = r;
        if (!netIsAP()) maybeSend(fb, r);   // only act on triggers when on the real network
        if (r.valid && r.count != lastMqttCount) { lastMqttCount = r.count; mqttPublishNow(); }
      }
      esp_camera_fb_return(fb);
    }
  }
  delay(2);
}
