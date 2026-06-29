#include "wg.h"
#include <esp_wireguard.h>
#include <WiFi.h>
#include "clk.h"
#include "logbuf.h"

// Split "ip[/bits]" into a bare ip (ipOut) and return the prefix bits (default 32).
static int wgSplitCidr(const char* in, char* ipOut, size_t n) {
  strlcpy(ipOut, in ? in : "", n);
  char* slash = strchr(ipOut, '/');
  int bits = 32;
  if (slash) { *slash = 0; bits = atoi(slash + 1); }
  if (bits < 0) bits = 0; if (bits > 32) bits = 32;
  return bits;
}
// Dotted-decimal netmask for a prefix length.
static void wgCidrToMask(int bits, char* out, size_t n) {
  uint32_t m = bits ? (0xFFFFFFFFu << (32 - bits)) : 0;
  snprintf(out, n, "%u.%u.%u.%u",
           (unsigned)((m >> 24) & 0xFF), (unsigned)((m >> 16) & 0xFF),
           (unsigned)((m >> 8) & 0xFF), (unsigned)(m & 0xFF));
}

static const Config*      s_cfg = nullptr;
static wireguard_ctx_t    s_ctx = {0};
static wireguard_config_t s_wg = ESP_WIREGUARD_CONFIG_DEFAULT();
static char s_addr[24], s_mask[16], s_aip[24], s_amask[16];
static uint8_t   s_downCnt = 0;   // consecutive peer-down polls (debounce the reboot)
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
         s_cfg->wgEndpointHost[0] && s_cfg->wgAddress[0] &&
         s_cfg->wgAllowedIps[0];
}

const char* wgStateStr() {
  if (!s_cfg || !s_cfg->wgEnabled) return "off";
  if (!s_connected) {
    if (WiFi.status() != WL_CONNECTED) return "wait-wifi";
    return clockSynced() ? "connecting" : "wait-clock";
  }
  return s_up ? "up" : "down";
}
bool wgIsUp() { return s_up; }

void wgLoop(uint32_t now) {
  if (!wgConfigComplete()) return;
  if (WiFi.status() != WL_CONNECTED) return;     // only over STA
  if (!clockSynced()) return;                    // NTP gate (TAI64N)

  if (!s_connected) {
    int abits = wgSplitCidr(s_cfg->wgAddress, s_addr, sizeof(s_addr));   // strip CIDR; bare IP for lwIP
    wgCidrToMask(abits, s_mask, sizeof(s_mask));                         // netmask from the address prefix
    s_wg.private_key          = s_cfg->wgPrivateKey;
    s_wg.public_key           = s_cfg->wgPeerPublicKey;
    s_wg.endpoint             = s_cfg->wgEndpointHost;
    s_wg.port                 = s_cfg->wgEndpointPort;
    s_wg.address              = s_addr;
    s_wg.netmask              = s_mask;
    s_wg.persistent_keepalive = s_cfg->wgKeepalive;
    if (s_cfg->wgPresharedKey[0]) s_wg.preshared_key = s_cfg->wgPresharedKey;

    if (!s_inited) {
      if (esp_wireguard_init(&s_wg, &s_ctx) != ESP_OK) { log_e("wg init failed"); return; }
      s_inited = true;
    }
    if (esp_wireguard_connect(&s_ctx) != ESP_OK) { log_e("wg connect failed"); return; }

    // Install the AllowedIPs route so homelab return traffic (incl. replies to whoever
    // opens the web UI) flows back through the tunnel. Without this the handshake is up
    // but the device cannot route to the homelab subnet.
    int pbits = wgSplitCidr(s_cfg->wgAllowedIps, s_aip, sizeof(s_aip));
    wgCidrToMask(pbits, s_amask, sizeof(s_amask));
    if (esp_wireguard_add_allowed_ip(&s_ctx, s_aip, s_amask) != ESP_OK)
      log_w("wg: add_allowed_ip(%s/%d) failed", s_aip, pbits);

    s_connected = true; s_connectedAt = now;
    log_i("wg: connecting to %s:%u", s_cfg->wgEndpointHost, (unsigned)s_cfg->wgEndpointPort);
    return;
  }

  if (now - s_lastPoll < 5000) return;
  s_lastPoll = now;
  bool up = (esp_wireguardif_peer_is_up(&s_ctx) == ESP_OK);
  if (up && !s_up) log_i("wg: tunnel up");
  s_up = up;
  if (up) s_downCnt = 0;
  else if (s_downCnt < 255) s_downCnt++;

  // Reboot only after the bring-up window has passed AND several consecutive down polls,
  // so a single transient blip after the tunnel was up does not bounce the device.
  if (!s_up && (now - s_connectedAt) > WG_UP_TIMEOUT_MS && s_downCnt >= 3) {
    log_w("wg: peer down for %u polls after %lus -> reboot",
          (unsigned)s_downCnt, (unsigned long)(WG_UP_TIMEOUT_MS / 1000));
    delay(50);
    ESP.restart();
  }
}
