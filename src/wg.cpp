#include "wg.h"
#include <esp_wireguard.h>
#include <WiFi.h>
#include "clk.h"
#include "logbuf.h"

static const Config*      s_cfg = nullptr;
static wireguard_ctx_t    s_ctx = {0};
static wireguard_config_t s_wg = ESP_WIREGUARD_CONFIG_DEFAULT();
static bool      s_inited = false;
static bool      s_connected = false;
static bool      s_up = false;
static uint32_t  s_connectedAt = 0;
static uint32_t  s_lastPoll = 0;
static const uint32_t WG_UP_TIMEOUT_MS = 60000;  // reboot if peer never comes up

void wgBegin(const Config* cfg) { s_cfg = cfg; }

static bool wgConfigComplete() {
  return s_cfg && s_cfg->wgEnabled &&
         s_cfg->wgPrivateKey[0] && s_cfg->wgPeerPublicKey[0] &&
         s_cfg->wgEndpointHost[0] && s_cfg->wgAddress[0];
}

const char* wgStateStr() {
  if (!s_cfg || !s_cfg->wgEnabled) return "off";
  if (!s_connected) return clockSynced() ? "connecting" : "wait-clock";
  return s_up ? "up" : "down";
}
bool wgIsUp() { return s_up; }

void wgLoop(uint32_t now) {
  if (!wgConfigComplete()) return;
  if (WiFi.status() != WL_CONNECTED) return;     // only over STA
  if (!clockSynced()) return;                    // NTP gate (TAI64N)

  if (!s_connected) {
    s_wg.private_key          = s_cfg->wgPrivateKey;
    s_wg.public_key           = s_cfg->wgPeerPublicKey;
    s_wg.endpoint             = s_cfg->wgEndpointHost;
    s_wg.port                 = s_cfg->wgEndpointPort;
    s_wg.address              = s_cfg->wgAddress;        // device tunnel IP (bare, e.g. "10.6.0.7")
    s_wg.netmask              = "255.255.255.0";
    s_wg.persistent_keepalive = s_cfg->wgKeepalive;
    if (s_cfg->wgPresharedKey[0]) s_wg.preshared_key = s_cfg->wgPresharedKey;

    if (!s_inited) {
      if (esp_wireguard_init(&s_wg, &s_ctx) != ESP_OK) { log_e("wg init failed"); return; }
      s_inited = true;
    }
    if (esp_wireguard_connect(&s_ctx) != ESP_OK) { log_e("wg connect failed"); return; }
    s_connected = true; s_connectedAt = now;
    log_i("wg: connecting to %s:%u", s_cfg->wgEndpointHost, (unsigned)s_cfg->wgEndpointPort);
    return;
  }

  if (now - s_lastPoll < 5000) return;
  s_lastPoll = now;
  bool up = (esp_wireguardif_peer_is_up(&s_ctx) == ESP_OK);
  if (up && !s_up) log_i("wg: tunnel up");
  s_up = up;

  if (!s_up && (now - s_connectedAt) > WG_UP_TIMEOUT_MS) {
    log_w("wg: peer not up after %lus -> reboot", (unsigned long)(WG_UP_TIMEOUT_MS / 1000));
    delay(50);
    ESP.restart();
  }
}
