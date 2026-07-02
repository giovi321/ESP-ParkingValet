#include <Arduino.h>
#include "esp_camera.h"
#include "config_store.h"
#include "camera.h"
#include "cv.h"
#include "cv_state.h"
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
#include "wg.h"
#include "capture.h"
#include "clf.h"
#include "profile.h"
#include "version.h"   // PARKINGCAM_VERSION + BUILD_GIT_SHA (folds in generated build_info.h)

static Config     cfg;
static CvEngine   cvEngine;
static CurbResult lastResult;

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

// Confirm a freshly-OTA'd image as valid (cancel bootloader rollback) if it is
// still pending. Idempotent and safe from any healthy context. Exposed so a
// voluntary reboot (web "reboot", config-change reboot, …) issued inside the 15s
// auto-confirm window below can't revert an image that has been running fine.
void otaMarkValidIfPending() {
  const esp_partition_t* run = esp_ota_get_running_partition();
  esp_ota_img_states_t imgState;
  if (run && esp_ota_get_state_partition(run, &imgState) == ESP_OK &&
      imgState == ESP_OTA_IMG_PENDING_VERIFY) {
    esp_ota_mark_app_valid_cancel_rollback();
    log_i("OTA image confirmed valid (rollback cancelled)");
  }
}

// --- CV baseline persistence (survive reboots) ----------------------------
// The relative-mode empty baseline lives only in RAM and is re-seeded from the
// first frame after boot. Since this device reboots itself (offline watchdog,
// OTA), a cell occupied at reboot would seed an "occupied" baseline and read
// empty until it turns over. We snapshot the baselines to NVS while running
// and restore them on boot. Writes are throttled and change-gated, so a stable
// scene writes nothing and flash wear stays bounded.
static const uint32_t CV_SAVE_INTERVAL_MS = 5UL * 60UL * 1000UL;
static CurbPersist s_cvSaved;              // last blob written (for change detection)
static bool        s_cvSavedValid = false;
static uint32_t    s_cvLastCheckMs = 0;

static bool cvStateDiffers(const CurbPersist& a, const CurbPersist& b) {
  if (a.geomSig != b.geomSig || a.cellCount != b.cellCount) return true;
  // Auto-learn progress: persist it too, else learnSamples/carPitchLearned only
  // reach NVS when a baseline happens to drift, and a reboot rolls them back —
  // possibly below the K>=30 gate, flipping est_free_spaces back to the default.
  if (a.learnSamples != b.learnSamples) return true;
  { float dp = a.carPitchLearned - b.carPitchLearned; if (dp < 0) dp = -dp; if (dp > 0.05f) return true; }
  for (int i = 0; i < MAX_CELLS; i++) {
    if (a.committed[i] != b.committed[i] || a.baselineInit[i] != b.baselineInit[i]) return true;
    float d = a.baselineEdge[i] - b.baselineEdge[i]; if (d < 0) d = -d;
    float ref = a.baselineEdge[i] > 1.0f ? a.baselineEdge[i] : 1.0f;
    if (d > 0.05f * ref) return true;    // baseline drifted > 5%
  }
  return false;
}

// Snapshot the engine baselines and write them to NVS, but only when they moved
// since the last save (force=true overrides, for a deliberate user action).
static void cvStatePersist(bool force) {
  CurbPersist cur; cvEngine.snapshotState(cur);
  if (!force && s_cvSavedValid && !cvStateDiffers(s_cvSaved, cur)) return;
  if (cvStateSave(cur)) { s_cvSaved = cur; s_cvSavedValid = true; }
}

// "Mark empty now": clear committed occupancy and re-arm a cell's adaptive empty
// baseline so it re-seeds from the next frame. index < 0 does every cell; a
// specific index does just that one, so a single empty cell can be calibrated
// without the whole strip being empty at once.
void cvRecalibrate(int index) {
  cvEngine.recalibrate(index);
  cvStatePersist(true);   // persist immediately so the recalibration survives a reboot
  if (index < 0) log_i("CV recalibrated: all baselines re-seed from the current view");
  else           log_i("CV recalibrated: cell %d re-seeds its baseline from the current view", index);
}

// "Mark occupied now": force one cell to read occupied (relative mode re-bases
// its baseline so the reading sticks and then self-heals when the space clears).
void cvMarkOccupied(int index) {
  cvEngine.markOccupied(index);
  cvStatePersist(true);
  log_i("CV: cell %d forced occupied (baseline re-based for relative mode)", index);
}

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

// Decide whether this committed metric warrants a webhook, per the trigger rule.
static bool shouldSend(int prev, int count) {
  if (cfg.triggerMode == TRIG_THRESHOLD) {
    int N = cfg.triggerThreshold;
    return (prev < N) != (count < N);   // crossing the threshold in either direction
  }
  return count != prev;                  // TRIG_ANY_CHANGE
}

static int postEvent(const char* event, camera_fb_t* fb, const CurbResult& r, int prev) {
  CurbEvent ev = {r.free_curb_m, r.est_free_spaces, prev, r.can_fit, r.reliable_range_m, r.occupied_fraction};
  int code = netSendEvent(cfg, event, fb->buf, fb->len, ev);  // live -> queued=false
  webNoteSend(event, ev.estSpaces, code);
  lastSendMs = millis();
  return code;
}

static void maybeSend(camera_fb_t* fb, const CurbResult& r) {
  if (!r.valid) return;
  uint32_t now = millis();

  // Heartbeat (independent of count changes).
  if (cfg.whEnabled && cfg.heartbeatIntervalS > 0 &&
      now - lastHeartbeatMs >= cfg.heartbeatIntervalS * 1000UL) {
    lastHeartbeatMs = now;
    postEvent("heartbeat", fb, r,
              lastSentCount < 0 ? r.est_free_spaces : lastSentCount);
  }

  // First valid frame: adopt as baseline silently.
  if (lastSentCount < 0) { lastSentCount = r.est_free_spaces; return; }

  if (!cfg.whEnabled) { lastSentCount = r.est_free_spaces; return; }
  if (!shouldSend(lastSentCount, r.est_free_spaces)) return;
  if (now - lastSendMs < cfg.minSendIntervalMs) return;   // rate-limited; retry next cycle

  // Deliver the change. When the link is up and nothing is already queued, send
  // it live WITH the freshly-captured photo (queued=false) — the common online
  // case. Fall back to the durable spool only when offline, when a backlog is
  // still draining (so we stay in order behind it), or when the live POST fails.
  if (cfg.spoolMode != SPOOL_OFF) {
    CurbEvent ev = {r.free_curb_m, r.est_free_spaces, lastSentCount, r.can_fit, r.reliable_range_m, r.occupied_fraction};
    uint32_t qCount, qBytes; spoolStats(qCount, qBytes);
    bool sentLive = false;
    if (WiFi.status() == WL_CONNECTED && qCount == 0) {
      int code = postEvent("count_changed", fb, r, lastSentCount);  // live, with photo
      sentLive = (code >= 200 && code < 400);
    }
    if (!sentLive)
      spoolEnqueue("count_changed", fb->buf, fb->len, ev);
    lastSendMs = now;
  } else {
    postEvent("count_changed", fb, r, lastSentCount);
  }
  lastSentCount = r.est_free_spaces;
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
  d["strip_count"]        = cfg.stripCount;
  d["cell_count"]         = cfg.cellCount;
  d["est_free_spaces"]    = lastResult.valid ? lastResult.est_free_spaces    : -1;
  d["free_curb_m"]        = lastResult.valid ? lastResult.free_curb_m        : 0.0f;
  d["can_fit"]            = lastResult.valid && lastResult.can_fit;
  d["reliable_range_m"]   = lastResult.valid ? lastResult.reliable_range_m   : 0.0f;
  d["occupied_fraction"]  = lastResult.valid ? lastResult.occupied_fraction  : 0.0f;
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
  log_i("config: host=%s sta='%s' cells=%d webhook=%s",
        cfg.hostname, cfg.staSsid, cfg.cellCount, cfg.whEnabled ? "on" : "off");

#ifdef PARKINGCAM_BUTTON_DISCOVERY
  buttonsDiscoveryScan();
#endif

  if (!cameraInit(cfg)) {
    log_e("camera init failed (check the OV5640 pin map / wiring)");
  } else {
    log_i("camera OK");
  }

  clfBegin();   // load a hot-swapped runtime classifier from NVS, if one was pushed
  profileBegin(&cfg);   // load the learned per-hour occupancy profile
  cvEngine.begin(&cfg);
  lastResult.valid = false;

  // Restore the per-cell baselines learned before the last reboot, so occupied
  // cells don't read empty until they turn over. Ignored (seed live) on first
  // boot or after a geometry change.
  {
    CurbPersist blob;
    if (cvStateLoad(blob) && cvEngine.restoreState(blob)) {
      s_cvSaved = blob; s_cvSavedValid = true;
      log_i("CV baselines restored from NVS (survived reboot)");
    } else {
      log_i("CV baselines: none stored or geometry changed -> seeding live");
    }
  }

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
  wgBegin(&cfg);
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
    otaMarkValidIfPending();
    spoolArm();   // only now let the spool touch flash — image is committed
  }

  buttonsLoop();
  netLoop();
  wgLoop(millis());
  webLoop();
  ledUpdate();
  maybeSendStats();
  mqttLoop();
  spoolDrain();   // deliver any queued count changes once the link is back

  if (millis() - s_cvLastCheckMs >= CV_SAVE_INTERVAL_MS) {   // throttled, change-gated baseline save
    s_cvLastCheckMs = millis();
    cvStatePersist(false);
  }
  profileLoop();   // throttled, change-gated persist of the learned occupancy profile

  uint32_t now = millis();
  if (now - lastCaptureMs >= cfg.captureIntervalMs) {
    lastCaptureMs = now;
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      static CurbResult r;
      if (cvEngine.analyze(fb->buf, fb->len, fb->width, fb->height, r)) {
        lastResult = r;
        profileUpdate(r);
        captureMaybeLog(cfg, r);
        if (!netIsAP()) maybeSend(fb, r);   // only act on triggers when on the real network
        if (r.valid && r.est_free_spaces != lastMqttCount) {
          lastMqttCount = r.est_free_spaces;
          mqttPublishNow();
        }
      }
      esp_camera_fb_return(fb);
    }
  }
  delay(2);
}
