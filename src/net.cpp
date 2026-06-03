#include "net.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <DNSServer.h>
#include "esp_heap_caps.h"
#include "clk.h"
#include "logbuf.h"   // routes log_i/_w/_e into the web serial console (include last)

static const Config* s_cfg = nullptr;
static NetMode   s_mode = NET_BOOT;
static DNSServer s_dns;
static uint32_t  s_lastReconnect = 0;
static bool      s_apFromFailure = false;   // AP entered as a fallback from a failed join
static uint32_t  s_offlineSince  = 0;       // millis() since we went offline (0 = online)

static const char* MP_BOUNDARY = "----ParkingCamBoundary7f3a91c4";

void netBegin(const Config* cfg) {
  s_cfg = cfg;
  WiFi.persistent(false);          // we manage credentials in NVS ourselves
  WiFi.setAutoReconnect(true);
}

bool netStartSTA(uint32_t timeoutMs) {
  if (!s_cfg || !s_cfg->staSsid[0]) return false;
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(s_cfg->hostname);
  WiFi.begin(s_cfg->staSsid, s_cfg->staPass);

  uint32_t start = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - start) < timeoutMs) {
    delay(200);
  }
  if (WiFi.status() == WL_CONNECTED) {
    s_mode = NET_STA;
    log_i("STA connected: %s  ip=%s rssi=%d",
          s_cfg->staSsid, WiFi.localIP().toString().c_str(), WiFi.RSSI());
    return true;
  }
  log_w("STA connect to '%s' timed out", s_cfg->staSsid);
  return false;
}

void netStartAP(bool fromFailure) {
  s_apFromFailure = fromFailure;
  s_offlineSince = 0;
  WiFi.mode(WIFI_AP);
  const char* pass = (strlen(s_cfg->apPass) >= 8) ? s_cfg->apPass : nullptr; // open if too short
  WiFi.softAP(s_cfg->apSsid, pass);
  IPAddress ip = WiFi.softAPIP();
  s_dns.start(53, "*", ip);        // captive portal: resolve everything to us
  s_mode = NET_AP;
  log_i("AP mode: ssid='%s' ip=%s", s_cfg->apSsid, ip.toString().c_str());
}

// Reboot the device if it has been offline for offlineRebootMin minutes (0=off).
static void offlineWatchdog(uint32_t now) {
  if (!s_cfg || s_cfg->offlineRebootMin == 0) { s_offlineSince = 0; return; }
  if (s_offlineSince == 0) s_offlineSince = now;
  else if (now - s_offlineSince >= (uint32_t)s_cfg->offlineRebootMin * 60000UL) {
    log_w("offline for %u min -> rebooting to recover", (unsigned)s_cfg->offlineRebootMin);
    delay(50);
    ESP.restart();
  }
}

void netLoop() {
  uint32_t now = millis();
  if (s_mode == NET_AP) {
    s_dns.processNextRequest();
    if (s_apFromFailure) offlineWatchdog(now);   // join failed -> keep retrying the real WiFi
    return;
  }
  if (s_mode == NET_STA) {
    if (WiFi.status() != WL_CONNECTED) {
      if (now - s_lastReconnect > 10000) {
        s_lastReconnect = now;
        log_w("STA link down, reconnecting...");
        WiFi.reconnect();
      }
      offlineWatchdog(now);
    } else {
      s_offlineSince = 0;   // online
    }
  }
}

NetMode netMode()  { return s_mode; }
bool    netIsAP()  { return s_mode == NET_AP; }

NetStatus netGetStatus() {
  NetStatus s;
  s.mode = s_mode;
  s.staConnected = (WiFi.status() == WL_CONNECTED);
  if (s_mode == NET_AP) { s.ip = WiFi.softAPIP().toString(); s.rssi = 0; }
  else { s.ip = WiFi.localIP().toString(); s.rssi = WiFi.RSSI(); }
  return s;
}

// --- webhook POST ----------------------------------------------------------

static String buildSlotsJson(const bool* slots, int n) {
  String s = "[";
  for (int i = 0; i < n; i++) { if (i) s += ","; s += slots[i] ? "true" : "false"; }
  s += "]";
  return s;
}

static void appendField(String& head, const char* name, const String& value) {
  head += "--"; head += MP_BOUNDARY; head += "\r\n";
  head += "Content-Disposition: form-data; name=\""; head += name; head += "\"\r\n\r\n";
  head += value; head += "\r\n";
}

int netSendEvent(const Config& cfg, const char* event,
                 const uint8_t* jpg, size_t jpgLen,
                 int count, int prevCount,
                 const bool* slots, int nSlots) {
  if (!cfg.whUrl[0]) return -1000;   // no URL set. The enable toggle is enforced by the
                                     // caller (maybeSend), so "Send test" works even when disabled.
  if (WiFi.status() != WL_CONNECTED)   return -1001;

  uint32_t uptimeS = millis() / 1000;
  String filename = String(cfg.hostname) + "_" + count + "_" + uptimeS + ".jpg";

  String head;
  head.reserve(640);
  appendField(head, "device",     cfg.hostname);
  appendField(head, "event",      event);
  appendField(head, "count",      String(count));
  appendField(head, "prev_count", String(prevCount));
  appendField(head, "slots",      buildSlotsJson(slots, nSlots));
  appendField(head, "ts",         String((uint32_t)clockEpoch()));   // UTC epoch (0 if unsynced)
  appendField(head, "time",       clockIso());                       // ISO8601 UTC
  // Image webhook intentionally carries no diagnostics — just photo + occupancy + time.
  head += "--"; head += MP_BOUNDARY; head += "\r\n";
  head += "Content-Disposition: form-data; name=\"image\"; filename=\"" + filename + "\"\r\n";
  head += "Content-Type: image/jpeg\r\n\r\n";

  String tail = "\r\n--";
  tail += MP_BOUNDARY; tail += "--\r\n";

  size_t total = head.length() + jpgLen + tail.length();
  uint8_t* body = (uint8_t*)heap_caps_malloc(total, MALLOC_CAP_SPIRAM);
  if (!body) body = (uint8_t*)malloc(total);
  if (!body) return -1002;

  size_t off = 0;
  memcpy(body + off, head.c_str(), head.length()); off += head.length();
  memcpy(body + off, jpg, jpgLen);                 off += jpgLen;
  memcpy(body + off, tail.c_str(), tail.length()); off += tail.length();

  bool https = strncmp(cfg.whUrl, "https:", 6) == 0;
  HTTPClient http;
  int code;
  String ctype = String("multipart/form-data; boundary=") + MP_BOUNDARY;

  if (https) {
    WiFiClientSecure secure;
    if (cfg.whTlsInsecure) secure.setInsecure();
    http.begin(secure, cfg.whUrl);
    http.addHeader("Content-Type", ctype);
    if (cfg.whAuthHeaderName[0]) http.addHeader(cfg.whAuthHeaderName, cfg.whAuthHeaderValue);
    code = http.POST(body, total);
    http.end();
  } else {
    WiFiClient client;
    http.begin(client, cfg.whUrl);
    http.addHeader("Content-Type", ctype);
    if (cfg.whAuthHeaderName[0]) http.addHeader(cfg.whAuthHeaderName, cfg.whAuthHeaderValue);
    code = http.POST(body, total);
    http.end();
  }

  heap_caps_free(body);
  log_i("webhook POST '%s' count=%d -> HTTP %d (%u bytes)", event, count, code, (unsigned)total);
  return code;
}

int netPostJson(const char* url, const char* authName, const char* authValue,
                bool tlsInsecure, const String& body) {
  if (!url || !url[0])               return -1000;
  if (WiFi.status() != WL_CONNECTED) return -1001;

  bool https = strncmp(url, "https:", 6) == 0;
  HTTPClient http;
  int code;
  if (https) {
    WiFiClientSecure secure;
    if (tlsInsecure) secure.setInsecure();
    http.begin(secure, url);
    http.addHeader("Content-Type", "application/json");
    if (authName && authName[0]) http.addHeader(authName, authValue);
    code = http.POST(body);
    http.end();
  } else {
    WiFiClient client;
    http.begin(client, url);
    http.addHeader("Content-Type", "application/json");
    if (authName && authName[0]) http.addHeader(authName, authValue);
    code = http.POST(body);
    http.end();
  }
  return code;
}
