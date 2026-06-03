#include "camera.h"
#include "camera_pins.h"
#include "esp_camera.h"
#include "ESP32_OV5640_AF.h"
#include "logbuf.h"   // routes log_i/_w/_e into the web serial console (include last)

static OV5640 s_ov5640;
static bool   s_afStarted = false;   // chip-id checked
static bool   s_afReady   = false;   // AF firmware downloaded

// Send an AF command via the firmware command register, waiting for the ACK.
static void afCmd(sensor_t* s, uint8_t cmd) {
  s->set_reg(s, OV5640_CMD_ACK, 0xff, 0x01);
  s->set_reg(s, OV5640_CMD_MAIN, 0xff, cmd);
  for (int i = 0; i < 200; i++) {
    if (s->get_reg(s, OV5640_CMD_ACK, 0xff) == 0x00) break;
    delay(5);
  }
}

// Download the AF firmware once (lazy). ~5 KB over SCCB, a second or so.
static bool ensureAfReady() {
  if (s_afReady) return true;
  if (!s_afStarted) return false;
  s_afReady = (s_ov5640.focusInit() == 0);
  log_i("OV5640 AF firmware init: %s", s_afReady ? "ok" : "FAILED");
  return s_afReady;
}

bool cameraInit(const Config& cfg) {
  camera_config_t c = {};
  c.ledc_channel = LEDC_CHANNEL_0;
  c.ledc_timer   = LEDC_TIMER_0;
  c.pin_d0 = CAM_PIN_D0;  c.pin_d1 = CAM_PIN_D1;  c.pin_d2 = CAM_PIN_D2;  c.pin_d3 = CAM_PIN_D3;
  c.pin_d4 = CAM_PIN_D4;  c.pin_d5 = CAM_PIN_D5;  c.pin_d6 = CAM_PIN_D6;  c.pin_d7 = CAM_PIN_D7;
  c.pin_xclk = CAM_PIN_XCLK;
  c.pin_pclk = CAM_PIN_PCLK;
  c.pin_vsync = CAM_PIN_VSYNC;
  c.pin_href  = CAM_PIN_HREF;
  c.pin_sccb_sda = CAM_PIN_SIOD;
  c.pin_sccb_scl = CAM_PIN_SIOC;
  c.pin_pwdn  = CAM_PIN_PWDN;
  c.pin_reset = CAM_PIN_RESET;
  c.xclk_freq_hz = CAM_XCLK_FREQ_HZ;
  // Floor the JPEG quality: values below ~8 produce oversized/malformed frames
  // on the OV5640 that the on-device decoder rejects ("Data format error").
  if (cfg.jpegQuality < 8) log_w("jpegQuality %d too aggressive; using 8", cfg.jpegQuality);
  c.pixel_format = PIXFORMAT_JPEG;
  // Initialize at the largest resolution we allow at runtime (UXGA) so the
  // driver allocates buffers big enough for any choice. set_framesize() can
  // then move freely among smaller sizes; growing PAST the init size would
  // otherwise silently fail and the resolution would appear "stuck".
  c.frame_size   = psramFound() ? FRAMESIZE_UXGA : FRAMESIZE_SVGA;
  c.jpeg_quality = cfg.jpegQuality < 8 ? 8 : cfg.jpegQuality;
  c.grab_mode    = CAMERA_GRAB_LATEST;

  if (psramFound()) {
    c.fb_count    = 2;
    c.fb_location = CAMERA_FB_IN_PSRAM;
  } else {
    c.fb_count    = 1;
    c.fb_location = CAMERA_FB_IN_DRAM;   // no PSRAM: capped at SVGA above
  }

  esp_err_t err = esp_camera_init(&c);
  if (err != ESP_OK) {
    log_e("esp_camera_init failed: 0x%x", err);
    return false;
  }
  cameraApplySettings(cfg);

  // Flush warm-up frames: the first frames right after init are often malformed
  // (the classic startup "JPG Decompression Failed" the CV loop would skip).
  for (int i = 0; i < 3; i++) {
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) esp_camera_fb_return(fb);
    delay(80);
  }

  s_afStarted = s_ov5640.start(esp_camera_sensor_get());   // checks the OV5640 chip id
  cameraApplyFocus(cfg);
  return true;
}

void cameraApplySettings(const Config& cfg) {
  sensor_t* s = esp_camera_sensor_get();
  if (!s) return;
  int rc = s->set_framesize(s, (framesize_t)cfg.framesize);
  int q = cfg.jpegQuality < 8 ? 8 : cfg.jpegQuality;   // floor (see cameraInit)
  log_i("apply framesize=%d quality=%d(eff %d) -> set_framesize rc=%d", cfg.framesize, cfg.jpegQuality, q, rc);
  s->set_quality(s, q);
  s->set_vflip(s, cfg.vFlip ? 1 : 0);
  s->set_hmirror(s, cfg.hMirror ? 1 : 0);
  s->set_brightness(s, cfg.brightness);
  s->set_contrast(s, cfg.contrast);
  s->set_saturation(s, cfg.saturation);
  s->set_whitebal(s, cfg.awb ? 1 : 0);
  s->set_awb_gain(s, cfg.awb ? 1 : 0);
  s->set_exposure_ctrl(s, cfg.aec ? 1 : 0);
}

void cameraApplyFocus(const Config& cfg) {
  if (cfg.afMode == 0) return;            // off / fixed: leave the lens as-is
  if (!ensureAfReady()) return;
  sensor_t* s = esp_camera_sensor_get();
  if (cfg.afMode == 2) { s_ov5640.autoFocusMode(); log_i("AF: continuous"); }
  else                 { afCmd(s, 0x03);           log_i("AF: single"); }
}

bool cameraFocusNow() {
  if (!ensureAfReady()) return false;
  afCmd(esp_camera_sensor_get(), 0x03);
  return true;
}

int cameraFocusStatus() {
  return s_afStarted ? (int)s_ov5640.getFWStatus() : -1;
}
