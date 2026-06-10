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
#include "esp_ota_ops.h"
#include "clk.h"
#include "mqttc.h"
#include "spool.h"
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

// --- hang watchdog --------------------------------------------------------
// A core-0 task reboots the device if loop() (or a progressing OTA upload)
// stops "beating". This recovers from a wedged web-server upload read on a
// silently-dropped link (half-open socket: the Arduino WebServer spins in
// _uploadReadByte with no timeout), which blocks loop() forever so none of the
// loop()-based timers (scheduled reboot, offline watchdog) can fire. The core
// 5s task-WDT only watches the idle task, not loopTask, so this fills the gap.
static volatile uint32_t s_loopBeat = 0;
void noteLoopAlive() { s_loopBeat = millis(); }
static void hangWatchdogTask(void*) {
  const uint32_t LIMIT_MS = 90000;   // no progress this long -> reboot to recover
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(3000));
    uint32_t beat = s_loopBeat;
    if (beat && (millis() - beat) > LIMIT_MS) {
      ets_printf("\n[hang-wdt] no loop progress for >%us -> restart\n", (unsigned)(LIMIT_MS / 1000));
      esp_restart();
    }
  }
}

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

  // Durably queue the change (delivered now if online, else on reconnect/after a
  // reboot) so it survives an outage. With the spool off, fall back to the
  // legacy live best-effort send.
  if (cfg.spoolMode != SPOOL_OFF) {
    bool slots[MAX_ROIS];
    for (int i = 0; i < r.n && i < MAX_ROIS; i++) slots[i] = r.slots[i].occupied;
    spoolEnqueue("count_changed", fb->buf, fb->len, r.count, lastSentCount, slots, r.n);
    lastSendMs = now;
  } else {
    postEvent("count_changed", fb, r, r.count, lastSentCount);
  }
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
  spoolBegin(&cfg);   // mount the offline queue + recover any pending events
  xTaskCreatePinnedToCore(hangWatchdogTask, "hangwdt", 2048, nullptr, 5, nullptr, 0);   // core 0; loopTask is core 1
  log_i("ready. mode=%s ip=%s", netIsAP() ? "AP" : "STA", netGetStatus().ip.c_str());
}

void loop() {
  noteLoopAlive();   // feed the hang watchdog

  // Confirm a freshly-OTA'd image as good after a short healthy run. This core
  // build has bootloader rollback enabled (CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE),
  // so an OTA image boots in "pending verify" and is reverted on the next reboot
  // unless the app marks itself valid. Until this fires, rollback is the safety
  // net: a build that can't run loop() this long is reverted automatically.
  static uint32_t loopStartMs = 0;
  static bool     otaConfirmed = false;
  if (loopStartMs == 0) loopStartMs = millis();
  if (!otaConfirmed && (millis() - loopStartMs) > 15000) {
    otaConfirmed = true;
    const esp_partition_t* run = esp_ota_get_running_partition();
    esp_ota_img_states_t imgState;
    if (run && esp_ota_get_state_partition(run, &imgState) == ESP_OK &&
        imgState == ESP_OTA_IMG_PENDING_VERIFY) {
      esp_ota_mark_app_valid_cancel_rollback();
      log_i("OTA image confirmed valid (rollback cancelled)");
    }
    spoolArm();   // only now let the spool touch flash — image is committed
  }

  buttonsLoop();
  netLoop();
  webLoop();
  ledUpdate();
  maybeSendStats();
  mqttLoop();
  spoolDrain();   // deliver any queued count changes once the link is back

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
