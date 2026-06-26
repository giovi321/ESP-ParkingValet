# MQTT Overlay Photo + Per-Bay State & Control — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add an MQTT command that publishes an on-device overlay photo, plus per-bay occupancy sensors and per-bay free/occupied controls, to the existing Home Assistant MQTT integration.

**Architecture:** A new `overlay` module captures a frame and burns the ROI overlay (colored polygons + bay-name labels + local-time timestamp) into a JPEG. `mqttc.cpp` gains a subscribe + receive callback (it runs on the loopTask via `s_mqtt.loop()`, same context as the web handlers, so it calls the existing `cvRecalibrate()`/`cvMarkOccupied()` directly). The big image is streamed with PubSubClient's `beginPublish()/write()/endPublish()`. A web-UI timezone offset feeds the burned-in timestamp.

**Tech Stack:** ESP32 (Arduino-ESP32 core 2.0.17), PlatformIO env `esp32cam`, PubSubClient, ArduinoJson 7, esp32-camera `img_converters` (`jpg2rgb565`, `fmt2jpg`).

## Global Constraints

- Build/compile gate for every task: `pio run -e esp32cam` must succeed (run from repo root `Z:/git/ESP-ParkingValet`).
- No new third-party libraries. Only `ArduinoJson`, `PubSubClient`, esp32-camera (already deps).
- `MAX_ROIS = 12`, `MAX_POLY = 8` (from `config_store.h`). Bay index `i` is 0-based.
- MQTT base topic = `cfg.mqttBaseTopic` (default `parking-valet`); discovery prefix = `cfg.mqttDiscoveryPrefix` (default `homeassistant`); stable node id `NODE = "parkingvalet"`, device id `DEVICE_ID = "esp-parkingvalet"` (already defined in `mqttc.cpp`).
- PubSubClient buffer stays 2048 (`setBufferSize(2048)`); large images MUST use `beginPublish/write/endPublish`, never a single `publish()`.
- Overlay colors match the web UI: free = green `#2ecc71` (46,204,113), occupied = red `#ff5b5b` (255,91,91), disabled = grey (140,140,140).
- The web UI HTML lives in `web-src/index.html`; `tools/pio_prebuild.py` regenerates `src/web_ui.h` from it on every build (do NOT hand-edit `web_ui.h`).
- Settings inputs follow the convention `id="c_<configKey>"`; `fill()` populates every key in the `F[]` array, `collect(keys)` reads them, and each Save button calls `save([...keys])`.
- There is no host unit-test framework. "Tests" are: (a) `pio run -e esp32cam` compiles, and (b) on-device observation steps (flash, then watch MQTT / HA / the photo). On-device steps are marked **(on-device, manual)** and may be deferred to a hardware session, but compile gates are mandatory per task.
- Commit after each task. Branch is `feature/mqtt-photo-overlay-bay-control` (already created).

---

## File Structure

- `src/config_store.h` — **modify**: add `int16_t tzOffsetMin` to `Config`.
- `src/config_store.cpp` — **modify**: default + `serializeFull` + `parseFull` for `tzOffsetMin`.
- `src/clk.h` / `src/clk.cpp` — **modify**: add `String clockLocalStamp(int offsetMin)`.
- `web-src/index.html` — **modify**: timezone-offset field + `F[]` entry + `saveImage` key.
- `src/font5x7.h` — **create**: public-domain 5×7 ASCII font table (data only).
- `src/overlay.h` / `src/overlay.cpp` — **create**: `overlayRenderJpeg()` + raster helpers.
- `src/mqttc.cpp` — **modify**: callback + subscribe; per-bay state, binary_sensor & select discovery; ROI-change refresh + stale cleanup; photo command + streamed publish; camera & control-button discovery.

Dependency order: Task 1 (config) → Task 2 (clk) → Task 3 (web UI) → Task 4 (overlay, needs clk+config) → Task 5 (mqtt state/control) → Task 6 (mqtt photo, needs overlay + Task 5).

---

### Task 1: Config field `tzOffsetMin`

**Files:**
- Modify: `src/config_store.h` (struct `Config`, after the `afMode` image block)
- Modify: `src/config_store.cpp` (`configLoadDefaults`, `serializeFull`, `parseFull`)

**Interfaces:**
- Produces: `Config::tzOffsetMin` (`int16_t`, minutes from UTC, default 0). Consumed by Task 2's caller (overlay) and Task 3 (web UI).

- [ ] **Step 1: Add the field to the struct**

In `src/config_store.h`, inside `struct Config`, the `// --- Image / sensor ---` block ends with `int afMode;`. Add the new field right after it:

```c
  int  afMode;               // OV5640 autofocus: 0=off/fixed, 1=auto once, 2=continuous
  int16_t tzOffsetMin;       // minutes offset from UTC for the burned-in overlay timestamp (no DST)
```

- [ ] **Step 2: Default it**

In `src/config_store.cpp`, in `configLoadDefaults()`, after `c.afMode = 1;` (the line with the `// focus once at boot` comment), add:

```c
  c.tzOffsetMin = 0;     // UTC by default
```

- [ ] **Step 3: Serialize it**

In `src/config_store.cpp`, in `serializeFull()`, after `o["afMode"] = c.afMode;`, add:

```c
  o["tzOffsetMin"] = c.tzOffsetMin;
```

- [ ] **Step 4: Parse it**

In `src/config_store.cpp`, in `parseFull()`, after `c.afMode = o["afMode"] | c.afMode;`, add:

```c
  c.tzOffsetMin = o["tzOffsetMin"] | c.tzOffsetMin;
```

- [ ] **Step 5: Compile**

Run: `pio run -e esp32cam`
Expected: build succeeds (`SUCCESS`), no errors.

- [ ] **Step 6: Commit**

```bash
git add src/config_store.h src/config_store.cpp
git commit -m "feat(config): add tzOffsetMin for the overlay timestamp

giovi321"
```

---

### Task 2: `clockLocalStamp()` helper

**Files:**
- Modify: `src/clk.h` (declaration)
- Modify: `src/clk.cpp` (definition)

**Interfaces:**
- Consumes: `Config::tzOffsetMin` (caller passes the int).
- Produces: `String clockLocalStamp(int offsetMin)` — returns `"YYYY-MM-DD HH:MM:SS"` in local time (UTC + offset), or `""` if the clock is not yet synced.

- [ ] **Step 1: Declare it**

In `src/clk.h`, after the `String clockIso();` line, add:

```c
String clockLocalStamp(int offsetMin);  // "YYYY-MM-DD HH:MM:SS" local (UTC+offsetMin), or "" if unsynced
```

- [ ] **Step 2: Define it**

In `src/clk.cpp`, append at end of file:

```c
String clockLocalStamp(int offsetMin) {
  if (!clockSynced()) return String("");
  time_t t = time(nullptr) + (time_t)offsetMin * 60;
  struct tm tmv;
  gmtime_r(&t, &tmv);   // gmtime on the already-offset epoch = local wall clock
  char b[24];
  strftime(b, sizeof(b), "%Y-%m-%d %H:%M:%S", &tmv);
  return String(b);
}
```

- [ ] **Step 3: Compile**

Run: `pio run -e esp32cam`
Expected: build succeeds.

- [ ] **Step 4: Commit**

```bash
git add src/clk.h src/clk.cpp
git commit -m "feat(clk): add clockLocalStamp() for local-time overlay timestamps

giovi321"
```

---

### Task 3: Web UI timezone-offset field

**Files:**
- Modify: `web-src/index.html` (Image section HTML; `F[]` array; `saveImage` keys)

**Interfaces:**
- Consumes: `cfg.tzOffsetMin` (from `/api/config`).
- Produces: a `tzOffsetMin` key in the POST body saved by the Image card.

- [ ] **Step 1: Add the input to the Image card**

In `web-src/index.html`, in the `<!-- IMAGE -->` section, find the autofocus select line:

```html
      <select id="c_afMode"><option value="0">Off / fixed</option><option value="1">Auto once</option><option value="2">Continuous</option></select>
```

Immediately after it (before the `<div class="btns">` with `saveImage`), insert:

```html
      <label>Timezone offset for photo timestamp (minutes from UTC)</label>
      <input type="number" id="c_tzOffsetMin" step="15" min="-720" max="840">
      <div class="hint">Only affects the timestamp burned into the MQTT overlay photo. Fixed offset, no automatic DST. Examples: <code>60</code> = UTC+1, <code>120</code> = UTC+2, <code>-300</code> = UTC−5.</div>
```

- [ ] **Step 2: Register it in the form-binding array**

In `web-src/index.html`, find the `F` array. Its image line reads:

```js
  "framesize","jpegQuality","brightness","contrast","saturation","vFlip","hMirror","awb","aec","afMode",
```

Replace that line with (append `tzOffsetMin`):

```js
  "framesize","jpegQuality","brightness","contrast","saturation","vFlip","hMirror","awb","aec","afMode","tzOffsetMin",
```

- [ ] **Step 3: Save it from the Image card**

In `web-src/index.html`, find:

```js
$("#saveImage").onclick=()=>save(["framesize","jpegQuality","brightness","contrast","saturation","vFlip","hMirror","awb","aec","afMode"]);
```

Replace with (append `tzOffsetMin`):

```js
$("#saveImage").onclick=()=>save(["framesize","jpegQuality","brightness","contrast","saturation","vFlip","hMirror","awb","aec","afMode","tzOffsetMin"]);
```

- [ ] **Step 4: Compile (regenerates web_ui.h from the HTML)**

Run: `pio run -e esp32cam`
Expected: build succeeds; the prebuild step regenerates `src/web_ui.h` (no manual edit).

- [ ] **Step 5: Verify the field is wired (on-device, manual)**

After flashing a later build: open the web UI → Image tab → set "Timezone offset" to `120` → Save → reload. The field should reload as `120`, and `GET /api/config` should include `"tzOffsetMin":120`.

- [ ] **Step 6: Commit**

```bash
git add web-src/index.html
git commit -m "feat(ui): timezone offset field for the overlay timestamp

giovi321"
```

---

### Task 4: Overlay renderer module

**Files:**
- Create: `src/font5x7.h`
- Create: `src/overlay.h`
- Create: `src/overlay.cpp`

**Interfaces:**
- Consumes: `Config` (`rois`, `roiCount`, `jpegQuality`, `tzOffsetMin`), `CvResult` (`slots[i].occupied`, `valid`, `n`), `clockLocalStamp(int)` (Task 2), esp32-camera `esp_camera_fb_get/return`, `img_converters` `jpg2rgb565`/`fmt2jpg`.
- Produces: `size_t overlayRenderJpeg(const Config& cfg, const CvResult& cv, uint8_t** out)` — captures a fresh frame, returns a malloc'd overlaid JPEG (`free()` it). Returns 0 on any failure. Consumed by Task 6.

- [ ] **Step 1: Create the font table `src/font5x7.h`**

Public-domain 5×7 font, ASCII 0x20–0x7F, column-major (5 columns/char, bit0 = top row):

```c
#pragma once
#include <Arduino.h>

// Classic public-domain 5x7 glyphs, ASCII 0x20..0x7F. 5 bytes per char,
// column-major; bit 0 = top pixel row, bit 6 = bottom (rows 0..6 used).
static const uint8_t FONT5X7[96][5] = {
  {0x00,0x00,0x00,0x00,0x00}, {0x00,0x00,0x5F,0x00,0x00}, {0x00,0x07,0x00,0x07,0x00}, {0x14,0x7F,0x14,0x7F,0x14},
  {0x24,0x2A,0x7F,0x2A,0x12}, {0x23,0x13,0x08,0x64,0x62}, {0x36,0x49,0x55,0x22,0x50}, {0x00,0x05,0x03,0x00,0x00},
  {0x00,0x1C,0x22,0x41,0x00}, {0x00,0x41,0x22,0x1C,0x00}, {0x14,0x08,0x3E,0x08,0x14}, {0x08,0x08,0x3E,0x08,0x08},
  {0x00,0x50,0x30,0x00,0x00}, {0x08,0x08,0x08,0x08,0x08}, {0x00,0x60,0x60,0x00,0x00}, {0x20,0x10,0x08,0x04,0x02},
  {0x3E,0x51,0x49,0x45,0x3E}, {0x00,0x42,0x7F,0x40,0x00}, {0x42,0x61,0x51,0x49,0x46}, {0x21,0x41,0x45,0x4B,0x31},
  {0x18,0x14,0x12,0x7F,0x10}, {0x27,0x45,0x45,0x45,0x39}, {0x3C,0x4A,0x49,0x49,0x30}, {0x01,0x71,0x09,0x05,0x03},
  {0x36,0x49,0x49,0x49,0x36}, {0x06,0x49,0x49,0x29,0x1E}, {0x00,0x36,0x36,0x00,0x00}, {0x00,0x56,0x36,0x00,0x00},
  {0x08,0x14,0x22,0x41,0x00}, {0x14,0x14,0x14,0x14,0x14}, {0x00,0x41,0x22,0x14,0x08}, {0x02,0x01,0x51,0x09,0x06},
  {0x32,0x49,0x79,0x41,0x3E}, {0x7E,0x11,0x11,0x11,0x7E}, {0x7F,0x49,0x49,0x49,0x36}, {0x3E,0x41,0x41,0x41,0x22},
  {0x7F,0x41,0x41,0x22,0x1C}, {0x7F,0x49,0x49,0x49,0x41}, {0x7F,0x09,0x09,0x09,0x01}, {0x3E,0x41,0x49,0x49,0x7A},
  {0x7F,0x08,0x08,0x08,0x7F}, {0x00,0x41,0x7F,0x41,0x00}, {0x20,0x40,0x41,0x3F,0x01}, {0x7F,0x08,0x14,0x22,0x41},
  {0x7F,0x40,0x40,0x40,0x40}, {0x7F,0x02,0x0C,0x02,0x7F}, {0x7F,0x04,0x08,0x10,0x7F}, {0x3E,0x41,0x41,0x41,0x3E},
  {0x7F,0x09,0x09,0x09,0x06}, {0x3E,0x41,0x51,0x21,0x5E}, {0x7F,0x09,0x19,0x29,0x46}, {0x46,0x49,0x49,0x49,0x31},
  {0x01,0x01,0x7F,0x01,0x01}, {0x3F,0x40,0x40,0x40,0x3F}, {0x1F,0x20,0x40,0x20,0x1F}, {0x3F,0x40,0x38,0x40,0x3F},
  {0x63,0x14,0x08,0x14,0x63}, {0x07,0x08,0x70,0x08,0x07}, {0x61,0x51,0x49,0x45,0x43}, {0x00,0x7F,0x41,0x41,0x00},
  {0x02,0x04,0x08,0x10,0x20}, {0x00,0x41,0x41,0x7F,0x00}, {0x04,0x02,0x01,0x02,0x04}, {0x40,0x40,0x40,0x40,0x40},
  {0x00,0x01,0x02,0x04,0x00}, {0x20,0x54,0x54,0x54,0x78}, {0x7F,0x48,0x44,0x44,0x38}, {0x38,0x44,0x44,0x44,0x20},
  {0x38,0x44,0x44,0x48,0x7F}, {0x38,0x54,0x54,0x54,0x18}, {0x08,0x7E,0x09,0x01,0x02}, {0x0C,0x52,0x52,0x52,0x3E},
  {0x7F,0x08,0x04,0x04,0x78}, {0x00,0x44,0x7D,0x40,0x00}, {0x20,0x40,0x44,0x3D,0x00}, {0x7F,0x10,0x28,0x44,0x00},
  {0x00,0x41,0x7F,0x40,0x00}, {0x7C,0x04,0x18,0x04,0x78}, {0x7C,0x08,0x04,0x04,0x78}, {0x38,0x44,0x44,0x44,0x38},
  {0x7C,0x14,0x14,0x14,0x08}, {0x08,0x14,0x14,0x18,0x7C}, {0x7C,0x08,0x04,0x04,0x08}, {0x48,0x54,0x54,0x54,0x20},
  {0x04,0x3F,0x44,0x40,0x20}, {0x3C,0x40,0x40,0x20,0x7C}, {0x1C,0x20,0x40,0x20,0x1C}, {0x3C,0x40,0x30,0x40,0x3C},
  {0x44,0x28,0x10,0x28,0x44}, {0x0C,0x50,0x50,0x50,0x3C}, {0x44,0x64,0x54,0x4C,0x44}, {0x00,0x08,0x36,0x41,0x00},
  {0x00,0x00,0x7F,0x00,0x00}, {0x00,0x41,0x36,0x08,0x00}, {0x08,0x04,0x08,0x10,0x08}, {0x00,0x00,0x00,0x00,0x00},
};
```

- [ ] **Step 2: Create `src/overlay.h`**

```c
#pragma once
#include <Arduino.h>
#include "config_store.h"
#include "cv.h"

// Capture a fresh camera frame and burn the ROI overlay into it: each bay's
// polygon outline + translucent fill colored by occupancy (green=free,
// red=occupied, grey=disabled), the bay name as a label, and a local-time
// timestamp. Returns a freshly-malloc'd JPEG via *out (free it with free());
// the return value is the JPEG length, or 0 on any failure (no frame, decode
// error, OOM). Runs on the loopTask; takes ~1-2 s. `cv` supplies occupancy.
size_t overlayRenderJpeg(const Config& cfg, const CvResult& cv, uint8_t** out);
```

- [ ] **Step 3: Create `src/overlay.cpp`**

```c
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
      int lx = (int)cx - (int)(strlen(r.name) * 6) / 2;
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
```

- [ ] **Step 4: Compile**

Run: `pio run -e esp32cam`
Expected: build succeeds. (Linker pulls in `overlay.cpp`; it is referenced by Task 6, but it compiles standalone now.)

- [ ] **Step 5: Commit**

```bash
git add src/font5x7.h src/overlay.h src/overlay.cpp
git commit -m "feat(overlay): on-device ROI overlay renderer (polygons, labels, timestamp)

giovi321"
```

---

### Task 5: MQTT per-bay state + control (no photo yet)

**Files:**
- Modify: `src/mqttc.cpp`

**Interfaces:**
- Consumes: `cvRecalibrate(int)`, `cvMarkOccupied(int)` (from `main.cpp`, declared `extern` here as in `web_server.cpp`); `s_cfg` (`Config*`), `s_last` (`CvResult*`), `baseTopic()`, `availTopic()`, `addDevice()`, `s_mqtt` (already in the file).
- Produces: subscriptions + callback handling `base/cmd/mark_all_free` and `base/bay/<i>/set`; retained `base/bay/<i>/state` (`ON`/`OFF`); retained `base/bay/<i>/setstate` (`—`); HA discovery for N binary_sensors + N selects with stale-index cleanup; ROI-change refresh. Establishes the `s_photoReq` flag plumbing consumed by Task 6.

- [ ] **Step 1: Add externs, helpers, and state near the top of `mqttc.cpp`**

In `src/mqttc.cpp`, after the existing `static bool s_began = false;` line (end of the static-state block), add:

```c
// Per-bay control reuses the CV actions that back the web UI buttons (main.cpp).
extern void cvRecalibrate(int index);   // "mark free": re-seed baseline(s) (index<0 = all)
extern void cvMarkOccupied(int index);  // "mark occupied": force one bay occupied

static volatile bool s_photoReq    = false;   // set by the receive callback, serviced in mqttLoop
static uint32_t      s_lastPhotoMs = 0;
static const uint32_t PHOTO_MIN_MS = 3000;    // rate-limit overlay photos
static uint32_t      s_roiSig      = 0;       // detect ROI edits to refresh discovery
static const char*   SETSTATE_IDLE = "-";     // select idle option (ASCII, see options list)
```

Add the overlay include with the other includes at the top (after `#include "clk.h"`):

```c
#include "overlay.h"
```

- [ ] **Step 2: Add an ROI signature helper (above `publishDiscovery`)**

In `src/mqttc.cpp`, just before `static void publishDiscovery() {`, add:

```c
// Cheap hash of bay count + names + enabled, to detect ROI edits and refresh
// the per-bay HA entities (editing ROIs does not call mqttReconfigure()).
static uint32_t roiSig() {
  uint32_t hsh = 2166136261u;
  auto mix = [&](uint32_t v) { hsh ^= v; hsh *= 16777619u; };
  mix((uint32_t)s_cfg->roiCount);
  for (int i = 0; i < s_cfg->roiCount && i < MAX_ROIS; i++) {
    const Roi& r = s_cfg->rois[i];
    for (const char* p = r.name; *p; p++) mix((uint8_t)*p);
    mix(r.enabled ? 1u : 0u);
  }
  return hsh;
}
```

- [ ] **Step 3: Add bay discovery + bay state publishers (above `publishState`)**

In `src/mqttc.cpp`, just before `static void publishState() {`, add:

```c
// Publish one HA discovery config (retained) under <prefix>/<component>/<node>/<obj>/config.
static void publishCfg(const char* component, const String& obj, JsonDocument& d) {
  const String prefix = s_cfg->mqttDiscoveryPrefix[0] ? s_cfg->mqttDiscoveryPrefix : "homeassistant";
  char payload[640];
  size_t n = serializeJson(d, payload, sizeof(payload));
  String topic = prefix + "/" + component + "/" + NODE + "/" + obj + "/config";
  s_mqtt.publish(topic.c_str(), (const uint8_t*)payload, n, true);
}
static void clearCfg(const char* component, const String& obj) {
  const String prefix = s_cfg->mqttDiscoveryPrefix[0] ? s_cfg->mqttDiscoveryPrefix : "homeassistant";
  String topic = prefix + "/" + component + "/" + NODE + "/" + obj + "/config";
  s_mqtt.publish(topic.c_str(), (const uint8_t*)"", 0, true);   // empty retained = remove entity
}

// Per-bay occupancy binary_sensors + free/occupied selects, with stale cleanup.
static void publishBayDiscovery() {
  if (!s_cfg->mqttDiscovery) return;
  const String base = baseTopic();
  const String avty = availTopic();
  for (int i = 0; i < s_cfg->roiCount && i < MAX_ROIS; i++) {
    const char* nm = s_cfg->rois[i].name[0] ? s_cfg->rois[i].name : "Bay";
    {
      JsonDocument d;
      d["name"]    = nm;
      d["uniq_id"] = String(NODE) + "_bay" + i;
      d["stat_t"]  = base + "/bay/" + i + "/state";
      d["avty_t"]  = avty;
      d["dev_cla"] = "occupancy";
      addDevice(d.as<JsonObject>());
      publishCfg("binary_sensor", String("bay") + i, d);
    }
    {
      JsonDocument d;
      d["name"]    = String(nm) + " control";
      d["uniq_id"] = String(NODE) + "_bay" + i + "_set";
      d["cmd_t"]   = base + "/bay/" + i + "/set";
      d["stat_t"]  = base + "/bay/" + i + "/setstate";
      d["avty_t"]  = avty;
      JsonArray opt = d["options"].to<JsonArray>();
      opt.add(SETSTATE_IDLE); opt.add("free"); opt.add("occupied");
      addDevice(d.as<JsonObject>());
      publishCfg("select", String("bay") + i + "_set", d);
    }
  }
  for (int i = s_cfg->roiCount; i < MAX_ROIS; i++) {   // remove entities for dropped bays
    clearCfg("binary_sensor", String("bay") + i);
    clearCfg("select", String("bay") + i + "_set");
  }
}

// Per-bay occupancy state (retained) + initial idle select state.
static void publishBayState() {
  const String base = baseTopic();
  for (int i = 0; i < s_cfg->roiCount && i < MAX_ROIS; i++) {
    bool occ = (s_last && s_last->valid && i < s_last->n) ? s_last->slots[i].occupied : false;
    s_mqtt.publish((base + "/bay/" + i + "/state").c_str(), occ ? "ON" : "OFF", true);
  }
}
static void publishBayIdle() {
  const String base = baseTopic();
  for (int i = 0; i < s_cfg->roiCount && i < MAX_ROIS; i++)
    s_mqtt.publish((base + "/bay/" + i + "/setstate").c_str(), SETSTATE_IDLE, true);
}
```

> Note: the select idle option is the plain ASCII string `"-"` (a hyphen). HA requires the published state to be one of the `options`, so the option list and the published idle value both use `"-"`.

- [ ] **Step 4: Call the bay publishers from `publishDiscovery` and `publishState`**

In `src/mqttc.cpp`, at the END of `publishDiscovery()` (after the existing `for` loop over `FIELDS`, before the closing `}`), add:

```c
  publishBayDiscovery();
```

At the END of `publishState()` (after its `for` loop, before the closing `}`), add:

```c
  publishBayState();
```

- [ ] **Step 5: Add the receive callback (above `connectNow`)**

In `src/mqttc.cpp`, just before `static bool connectNow() {`, add:

```c
static void onMqttMessage(char* topic, uint8_t* payload, unsigned int len) {
  const String base = baseTopic();
  String t(topic);
  char body[16] = {0};
  unsigned int n = len < sizeof(body) - 1 ? len : sizeof(body) - 1;
  memcpy(body, payload, n);

  if (t == base + "/cmd/photo") { s_photoReq = true; return; }
  if (t == base + "/cmd/mark_all_free") { cvRecalibrate(-1); return; }

  // base/bay/<i>/set
  String pre = base + "/bay/";
  if (t.startsWith(pre) && t.endsWith("/set")) {
    int i = t.substring(pre.length(), t.length() - 4).toInt();
    if (i < 0 || i >= MAX_ROIS) return;
    if (!strcmp(body, "free"))          cvRecalibrate(i);
    else if (!strcmp(body, "occupied")) cvMarkOccupied(i);
    else return;   // ignore the "-" idle echo
    s_mqtt.publish((base + "/bay/" + i + "/setstate").c_str(), SETSTATE_IDLE, true);  // reset so the same pick re-fires
  }
}

static void subscribeCommands() {
  const String base = baseTopic();
  s_mqtt.subscribe((base + "/cmd/photo").c_str());
  s_mqtt.subscribe((base + "/cmd/mark_all_free").c_str());
  s_mqtt.subscribe((base + "/bay/+/set").c_str());
}
```

- [ ] **Step 6: Register the callback in `mqttBegin` and subscribe in `connectNow`**

In `src/mqttc.cpp`, in `mqttBegin()`, after `s_mqtt.setKeepAlive(30);`, add:

```c
  s_mqtt.setCallback(onMqttMessage);
```

In `connectNow()`, inside the `if (ok) { ... }` block, after the existing `publishState();` line, add:

```c
    subscribeCommands();
    publishBayIdle();
    s_roiSig = roiSig();
```

- [ ] **Step 7: Refresh discovery on ROI edits in `mqttLoop`**

In `src/mqttc.cpp`, in `mqttLoop()`, replace this existing block:

```c
  s_mqtt.loop();
  uint32_t iv = (s_cfg->mqttIntervalS ? s_cfg->mqttIntervalS : 60) * 1000UL;
  if (now - s_lastPub >= iv) { s_lastPub = now; publishState(); }
```

with:

```c
  s_mqtt.loop();

  // Refresh per-bay HA entities when the ROI set/name/enable changes.
  uint32_t sig = roiSig();
  if (sig != s_roiSig) { s_roiSig = sig; publishBayDiscovery(); publishBayState(); publishBayIdle(); }

  uint32_t iv = (s_cfg->mqttIntervalS ? s_cfg->mqttIntervalS : 60) * 1000UL;
  if (now - s_lastPub >= iv) { s_lastPub = now; publishState(); }
```

- [ ] **Step 8: Compile**

Run: `pio run -e esp32cam`
Expected: build succeeds.

- [ ] **Step 9: Verify (on-device, manual)**

Flash. With an MQTT broker configured and HA discovery on:
- `mosquitto_sub -h <broker> -t 'parking-valet/#' -v` shows `parking-valet/bay/0/state ON|OFF` for each bay and `parking-valet/bay/0/setstate -`.
- In HA: each bay appears as an occupancy binary_sensor and a "<name> control" select with options `-/free/occupied`.
- Pick `occupied` on a bay's select → device log shows `CV: bay 0 forced occupied`; the select snaps back to `-`. Pick `free` → log shows `CV recalibrated: bay 0 …`.
- `mosquitto_pub -t parking-valet/cmd/mark_all_free -n` → log shows all baselines re-seeding.

- [ ] **Step 10: Commit**

```bash
git add src/mqttc.cpp
git commit -m "feat(mqtt): per-bay occupancy sensors + free/occupied select controls

giovi321"
```

---

### Task 6: MQTT overlay photo command

**Files:**
- Modify: `src/mqttc.cpp`

**Interfaces:**
- Consumes: `overlayRenderJpeg()` (Task 4), `s_photoReq`/`s_lastPhotoMs`/`PHOTO_MIN_MS` (Task 5), `s_mqtt`, `baseTopic()`, `availTopic()`, `addDevice()`.
- Produces: streamed retained JPEG on `base/photo`; HA camera + "Take photo" + "Mark all free" buttons in discovery; services `s_photoReq` in `mqttLoop`.

- [ ] **Step 1: Add the photo publisher (above `mqttLoop`)**

In `src/mqttc.cpp`, just before `void mqttLoop() {`, add:

```c
// Render the overlay photo and stream it to base/photo. The JPEG is far larger
// than the 2048-byte client buffer, so it MUST be streamed with beginPublish/
// write/endPublish rather than a single publish().
static void publishPhoto() {
  if (!s_mqtt.connected()) return;
  uint8_t* jpg = nullptr;
  size_t len = overlayRenderJpeg(*s_cfg, *s_last, &jpg);
  if (!len || !jpg) return;
  String topic = baseTopic() + "/photo";
  if (s_mqtt.beginPublish(topic.c_str(), len, /*retained=*/true)) {
    size_t off = 0;
    while (off < len) {
      size_t chunk = (len - off) < 512 ? (len - off) : 512;
      s_mqtt.write(jpg + off, chunk);
      off += chunk;
    }
    s_mqtt.endPublish();
    log_i("MQTT photo published: %u bytes", (unsigned)len);
  } else {
    log_w("MQTT beginPublish failed for photo (%u bytes)", (unsigned)len);
  }
  free(jpg);
}
```

> `overlayRenderJpeg` allocates with `fmt2jpg` (plain `malloc`), so it is released with `free()`, not `heap_caps_free()`.

- [ ] **Step 2: Add camera + control-button discovery to `publishBayDiscovery`**

In `src/mqttc.cpp`, at the END of `publishBayDiscovery()` (after the stale-cleanup `for` loop, before the closing `}`), add:

```c
  // Snapshot camera + two control buttons (published once with the bay configs).
  {
    JsonDocument d;
    d["name"]    = "Snapshot";
    d["uniq_id"] = String(NODE) + "_photo";
    d["t"]       = base + "/photo";
    d["avty_t"]  = avty;
    addDevice(d.as<JsonObject>());
    publishCfg("camera", "photo", d);
  }
  {
    JsonDocument d;
    d["name"]    = "Take photo";
    d["uniq_id"] = String(NODE) + "_take_photo";
    d["cmd_t"]   = base + "/cmd/photo";
    d["pl_prs"]  = "1";
    d["avty_t"]  = avty;
    d["ic"]      = "mdi:camera";
    addDevice(d.as<JsonObject>());
    publishCfg("button", "take_photo", d);
  }
  {
    JsonDocument d;
    d["name"]    = "Mark all free";
    d["uniq_id"] = String(NODE) + "_mark_all_free";
    d["cmd_t"]   = base + "/cmd/mark_all_free";
    d["pl_prs"]  = "1";
    d["avty_t"]  = avty;
    d["ic"]      = "mdi:broom";
    addDevice(d.as<JsonObject>());
    publishCfg("button", "mark_all_free", d);
  }
```

- [ ] **Step 3: Service the photo request in `mqttLoop`**

In `src/mqttc.cpp`, in `mqttLoop()`, find the ROI-refresh block added in Task 5:

```c
  uint32_t sig = roiSig();
  if (sig != s_roiSig) { s_roiSig = sig; publishBayDiscovery(); publishBayState(); publishBayIdle(); }
```

Immediately after it, add:

```c
  if (s_photoReq && now - s_lastPhotoMs > PHOTO_MIN_MS) {
    s_photoReq = false; s_lastPhotoMs = now;
    publishPhoto();
  }
```

- [ ] **Step 4: Compile**

Run: `pio run -e esp32cam`
Expected: build succeeds.

- [ ] **Step 5: Verify (on-device, manual)**

Flash.
- In HA: a "Snapshot" camera entity, plus "Take photo" and "Mark all free" buttons appear on the device.
- Press "Take photo" (or `mosquitto_pub -t parking-valet/cmd/photo -n`). The device log shows `overlay: WxH -> N byte jpeg` then `MQTT photo published`. The HA camera shows the photo with colored bay polygons (green free / red occupied / grey disabled), bay-name labels, and a bottom-left timestamp in the configured local time.
- **Color check:** if green/red look swapped or wrong (e.g. green renders blue), the RGB565 byte order differs on encode — fix by byte-swapping in `RGB565()` in `overlay.cpp` (`return __builtin_bswap16(value)`), rebuild, retest.
- Press "Take photo" twice within 3 s → only one image publishes (rate limit).

- [ ] **Step 6: Commit**

```bash
git add src/mqttc.cpp
git commit -m "feat(mqtt): on-demand overlay photo + camera/control-button discovery

giovi321"
```

---

## Self-Review

**Spec coverage:**
- On-demand overlay photo (capture + burn-in + publish) → Task 4 (renderer) + Task 6 (command + streamed publish + camera entity). ✓
- Overlay = colored polygons + fill + bay-name labels + local timestamp → Task 4. ✓
- Per-bay occupancy state (binary_sensor) → Task 5 (`publishBayState` + binary_sensor discovery). ✓
- Per-bay control as a single select (free/occupied) with idle re-fire → Task 5 (select discovery + callback + `setstate` reset). ✓
- "Mark all free" + "Take photo" controls → Tasks 5/6 (buttons + `cmd/*` topics). ✓
- Retained photo → Task 6 (`beginPublish(..., true)`). ✓
- Command-only trigger (no auto-publish on count change) → no change to `main.cpp`'s `maybeSend`. ✓
- Configurable TZ offset in web UI → Tasks 1, 2, 3. ✓
- Discovery refresh on ROI changes + stale cleanup → Task 5 (`roiSig` in `mqttLoop`, `clearCfg`). ✓
- Subscribe + callback on the loopTask → Task 5 (`setCallback`, `subscribeCommands`). ✓

**Placeholder scan:** No TBD/TODO; every code step shows complete code. Font table is fully populated. ✓

**Type consistency:**
- `overlayRenderJpeg(const Config&, const CvResult&, uint8_t**) -> size_t` — declared in `overlay.h` (Task 4 Step 2), called identically in Task 6 Step 1. ✓
- `clockLocalStamp(int) -> String` — declared Task 2 Step 1, used Task 4 Step 3. ✓
- `s_photoReq`, `s_lastPhotoMs`, `PHOTO_MIN_MS`, `s_roiSig`, `SETSTATE_IDLE` — defined Task 5 Step 1, used in Tasks 5/6. ✓
- `publishCfg`/`clearCfg`/`publishBayDiscovery`/`publishBayState`/`publishBayIdle`/`roiSig`/`onMqttMessage`/`subscribeCommands`/`publishPhoto` — all defined before use; `publishBayDiscovery` is extended in Task 6 Step 2 (same function). ✓
- `cfg.tzOffsetMin` is `int16_t`; `clockLocalStamp(int)` widens it safely. ✓

**Known runtime risk (flagged for the verifier, not a plan defect):** RGB565 encode byte-order — Task 6 Step 5 has the one-line fix if colors look wrong. Internal-DRAM headroom for `fmt2jpg`'s output alloc — if it fails, `overlayRenderJpeg` returns 0 and the photo is skipped (logged), no crash; mitigation is the ≤800px width cap already in Task 4.
