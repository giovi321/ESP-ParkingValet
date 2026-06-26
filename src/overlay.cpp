#include "overlay.h"
#include "esp_camera.h"
#include "img_converters.h"
#include "esp_heap_caps.h"
#include "clk.h"
#include "font5x7.h"
#include <math.h>
#include "logbuf.h"

// RGB565 packing matching cv.cpp's extraction (R in bits 15..11, G 10..5, B 4..0).
#define RGB565(r, g, b) ((uint16_t)((((r) & 0xF8) << 8) | (((g) & 0xFC) << 3) | ((b) >> 3)))
static const uint16_t COL_FREE = RGB565(46, 204, 113);
static const uint16_t COL_OCC  = RGB565(255, 91, 91);
static const uint16_t COL_DIS  = RGB565(140, 140, 140);
static const uint16_t COL_WHITE = RGB565(255, 255, 255);
static const uint16_t COL_BLACK = RGB565(0, 0, 0);

// Blend src over dst with alpha 0..255 (component-wise on RGB565).
static inline uint16_t blend565(uint16_t dst, uint16_t src, uint8_t a) {
  int dr = (dst >> 11) & 0x1F, dg = (dst >> 5) & 0x3F, db = dst & 0x1F;
  int sr = (src >> 11) & 0x1F, sg = (src >> 5) & 0x3F, sb = src & 0x1F;
  int r = (sr * a + dr * (255 - a)) / 255;
  int g = (sg * a + dg * (255 - a)) / 255;
  int b = (sb * a + db * (255 - a)) / 255;
  return (uint16_t)((r << 11) | (g << 5) | b);
}

static inline void setPx(uint16_t* buf, int w, int h, int x, int y, uint16_t c) {
  if ((unsigned)x < (unsigned)w && (unsigned)y < (unsigned)h) buf[y * w + x] = c;
}
static inline void blendPx(uint16_t* buf, int w, int h, int x, int y, uint16_t c, uint8_t a) {
  if ((unsigned)x < (unsigned)w && (unsigned)y < (unsigned)h) {
    uint16_t* p = &buf[y * w + x];
    *p = blend565(*p, c, a);
  }
}

// 2px Bresenham line (plots a 2x2 block per point for visible thickness).
static void drawLine(uint16_t* buf, int w, int h, int x0, int y0, int x1, int y1, uint16_t c) {
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy;
  for (;;) {
    setPx(buf, w, h, x0, y0, c);   setPx(buf, w, h, x0 + 1, y0, c);
    setPx(buf, w, h, x0, y0 + 1, c); setPx(buf, w, h, x0 + 1, y0 + 1, c);
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}

static bool pointInPoly(const float* vx, const float* vy, int np, float x, float y) {
  bool inside = false;
  for (int a = 0, b = np - 1; a < np; b = a++) {
    if (((vy[a] > y) != (vy[b] > y)) &&
        (x < (vx[b] - vx[a]) * (y - vy[a]) / (vy[b] - vy[a]) + vx[a]))
      inside = !inside;
  }
  return inside;
}

// One 5x7 glyph at scale s (each font pixel -> s x s block).
static void drawChar(uint16_t* buf, int w, int h, int x, int y, char ch, uint16_t c, int s) {
  if (ch < 0x20 || ch > 0x7F) ch = 0x7F;
  const uint8_t* g = FONT5X7[ch - 0x20];
  for (int col = 0; col < 5; col++) {
    uint8_t bits = g[col];
    for (int row = 0; row < 7; row++) {
      if (bits & (1 << row))
        for (int dy = 0; dy < s; dy++)
          for (int dx = 0; dx < s; dx++)
            setPx(buf, w, h, x + col * s + dx, y + row * s + dy, c);
    }
  }
}

// Text with a translucent dark backing rectangle for legibility.
static void drawText(uint16_t* buf, int w, int h, int x, int y, const char* str, uint16_t c, int s) {
  int n = (int)strlen(str);
  int tw = n * 6 * s, th = 7 * s;
  for (int yy = y - s; yy < y + th + s; yy++)
    for (int xx = x - s; xx < x + tw + s; xx++)
      blendPx(buf, w, h, xx, yy, COL_BLACK, 160);
  for (int i = 0; i < n; i++) drawChar(buf, w, h, x + i * 6 * s, y, str[i], c, s);
}

size_t overlayRenderJpeg(const Config& cfg, const CvResult& cv, uint8_t** out) {
  *out = nullptr;
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) { log_w("overlay: no frame"); return 0; }

  // Decode (downscaled so working buffer + output stay bounded, <=800px wide).
  jpg_scale_t scale = (fb->width > 800) ? JPG_SCALE_2X : JPG_SCALE_NONE;
  int div = (scale == JPG_SCALE_2X) ? 2 : 1;
  int w = fb->width / div, h = fb->height / div;
  size_t px = (size_t)w * (h + 8);   // headroom: decoder rounds up to MCU grid
  uint16_t* rgb = (uint16_t*)heap_caps_malloc(px * 2, MALLOC_CAP_SPIRAM);
  if (!rgb) rgb = (uint16_t*)heap_caps_malloc(px * 2, MALLOC_CAP_8BIT);
  if (!rgb) { esp_camera_fb_return(fb); log_w("overlay: rgb OOM"); return 0; }

  bool ok = jpg2rgb565(fb->buf, fb->len, (uint8_t*)rgb, scale);
  esp_camera_fb_return(fb);
  if (!ok) { heap_caps_free(rgb); log_w("overlay: decode failed"); return 0; }

  // Draw each bay.
  for (int i = 0; i < cfg.roiCount && i < MAX_ROIS; i++) {
    const Roi& r = cfg.rois[i];
    int np = r.nPoints < MAX_POLY ? r.nPoints : MAX_POLY;
    if (np < 3) continue;
    float vx[MAX_POLY], vy[MAX_POLY];
    float minx = 1e9f, miny = 1e9f, maxx = -1e9f, maxy = -1e9f, cx = 0, cy = 0;
    for (int j = 0; j < np; j++) {
      vx[j] = r.px[j] * w; vy[j] = r.py[j] * h;
      cx += vx[j]; cy += vy[j];
      if (vx[j] < minx) minx = vx[j]; if (vx[j] > maxx) maxx = vx[j];
      if (vy[j] < miny) miny = vy[j]; if (vy[j] > maxy) maxy = vy[j];
    }
    cx /= np; cy /= np;
    bool occ = (cv.valid && i < cv.n) ? cv.slots[i].occupied : false;
    uint16_t col = !r.enabled ? COL_DIS : (occ ? COL_OCC : COL_FREE);

    // translucent fill
    int x0 = (int)floorf(minx), y0 = (int)floorf(miny);
    int x1 = (int)ceilf(maxx),  y1 = (int)ceilf(maxy);
    if (x0 < 0) x0 = 0; if (y0 < 0) y0 = 0;
    if (x1 > w - 1) x1 = w - 1; if (y1 > h - 1) y1 = h - 1;
    for (int y = y0; y <= y1; y++)
      for (int x = x0; x <= x1; x++)
        if (pointInPoly(vx, vy, np, (float)x, (float)y))
          blendPx(rgb, w, h, x, y, col, 64);

    // outline
    for (int j = 0; j < np; j++) {
      int k = (j + 1) % np;
      drawLine(rgb, w, h, (int)vx[j], (int)vy[j], (int)vx[k], (int)vy[k], col);
    }

    // label near centroid
    if (r.name[0]) {
      int lx = (int)cx - (int)(strlen(r.name) * 6);
      int ly = (int)cy - 7;
      if (lx < 2) lx = 2;
      drawText(rgb, w, h, lx, ly, r.name, COL_WHITE, 2);
    }
  }

  // timestamp, bottom-left
  String ts = clockLocalStamp(cfg.tzOffsetMin);
  if (!ts.length()) ts = String("uptime ") + String((uint32_t)(millis() / 1000)) + "s";
  drawText(rgb, w, h, 6, h - 7 * 2 - 6, ts.c_str(), COL_WHITE, 2);

  // encode
  uint8_t quality = (cfg.jpegQuality >= 8 && cfg.jpegQuality <= 63) ? (uint8_t)cfg.jpegQuality : 12;
  size_t jlen = 0;
  bool enc = fmt2jpg((uint8_t*)rgb, (size_t)w * h * 2, w, h, PIXFORMAT_RGB565, quality, out, &jlen);
  heap_caps_free(rgb);
  if (!enc || !*out) { log_w("overlay: encode failed"); return 0; }
  log_i("overlay: %dx%d -> %u byte jpeg", w, h, (unsigned)jlen);
  return jlen;
}
