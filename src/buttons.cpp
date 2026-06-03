#include "buttons.h"
#include "camera_pins.h"      // PIN_CONFIG_BUTTON (GPIO0)
#include "esp_system.h"
#include "logbuf.h"           // routes log_i/_w/_e into the web serial console (include last)

static const uint32_t FORCE_AP_MAGIC = 0xA9C0FFEEu;
RTC_NOINIT_ATTR static uint32_t s_forceAp;

static bool     s_pressed   = false;
static uint32_t s_pressedAt = 0;
static bool     s_handled   = false;

void buttonsBegin() {
  pinMode(PIN_CONFIG_BUTTON, INPUT_PULLUP);  // BOOT is active-LOW
}

bool buttonsConsumeForcedAp() {
  bool forced = (esp_reset_reason() == ESP_RST_SW) && (s_forceAp == FORCE_AP_MAGIC);
  s_forceAp = 0;   // always clear, so it fires only once
  return forced;
}

void buttonsLoop() {
  bool down = (digitalRead(PIN_CONFIG_BUTTON) == LOW);
  uint32_t now = millis();

  if (down && !s_pressed) {
    s_pressed = true; s_pressedAt = now; s_handled = false;
  } else if (down && s_pressed && !s_handled) {
    if (now - s_pressedAt >= LONGPRESS_MS) {
      s_handled = true;
      log_w("BOOT long-press: rebooting into AP config mode");
      s_forceAp = FORCE_AP_MAGIC;
      delay(50);
      ESP.restart();
    }
  } else if (!down && s_pressed) {
    s_pressed = false;   // released before threshold -> ignore
  }
}

void buttonsForceApMode() {
  s_forceAp = FORCE_AP_MAGIC;
  delay(50);
  ESP.restart();
}

void buttonsDiscoveryScan(uint32_t durationMs) {
  // Only safe, camera-free GPIOs (avoid UART 1/3 and all camera pins).
  const int pins[] = { 0, 4, 13, 16, 17 };
  const int n = sizeof(pins) / sizeof(pins[0]);
  int last[n];
  for (int i = 0; i < n; i++) { pinMode(pins[i], INPUT_PULLUP); last[i] = digitalRead(pins[i]); }
  Serial.println("[btn-discovery] press each physical button; transitions below:");
  uint32_t start = millis();
  while (millis() - start < durationMs) {
    for (int i = 0; i < n; i++) {
      int v = digitalRead(pins[i]);
      if (v != last[i]) {
        Serial.printf("[btn-discovery] GPIO%-2d -> %s\n", pins[i], v == LOW ? "LOW (pressed)" : "HIGH");
        last[i] = v;
      }
    }
    delay(15);
  }
  Serial.println("[btn-discovery] done");
}
