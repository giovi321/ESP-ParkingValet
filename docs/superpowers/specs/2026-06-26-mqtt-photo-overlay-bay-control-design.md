# Design: MQTT overlay photo + per-bay state & control

Date: 2026-06-26
Status: Approved (pending spec review)
Scope: firmware (`src/`), one new module, additions to `mqttc.cpp`

## Goal

Expose two new capabilities over MQTT, on top of the existing publish-only MQTT/HA
integration:

1. **On-demand overlay photo** — an MQTT command makes the device capture a frame
   *now*, burn the parking-space overlay into it (polygons colored by occupancy,
   bay-name labels, a timestamp), and publish the JPEG to an MQTT topic that a
   Home Assistant MQTT camera renders directly.
2. **Per-bay state + control** — publish each bay's free/occupied state as its own
   retained MQTT binary_sensor, and accept per-bay commands to mark a bay free
   (recalibrate) or occupied, plus a "mark all free" command. These reuse the
   existing `cvRecalibrate()` / `cvMarkOccupied()` actions that already back the
   web UI buttons.

Home Assistant is the consumer; all new entities attach to the existing
auto-discovered `ESP-ParkingValet` device.

## Confirmed decisions

- **Overlay richness:** polygon outline + translucent fill (green=free,
  red=occupied, grey=disabled, matching the web UI), **plus** bay-name text labels
  **plus** a capture timestamp. Text needs a small bundled bitmap font.
- **Photo topic retained:** yes — the last overlay image survives a broker/HA
  restart. (~50 KB retained payload; the broker must allow a packet that size.)
- **Trigger model:** command-only. No automatic overlay-photo publish on count
  change (the webhook path already ships photos on count change).

## Current architecture (relevant facts)

- `src/mqttc.cpp` is **publish-only** today: retained state topics under the base
  topic + HA discovery for a fixed `FIELDS[]` table. It never `subscribe()`s and
  has no message callback. Buffer is 2048 bytes (`setBufferSize(2048)`).
- The MQTT client is serviced by `mqttLoop()` → `s_mqtt.loop()`, called from the
  main `loop()` on the loopTask — the same context as the web handlers. So an
  MQTT receive callback may call `cvRecalibrate()` / `cvMarkOccupied()` directly
  (no cross-thread races), exactly as the web `/api/action` handler does.
- `cvRecalibrate(int)` and `cvMarkOccupied(int)` live in `main.cpp`, already
  `extern`-declared and used by `web_server.cpp`. Reusable verbatim.
- The web UI overlay is **SVG drawn in the browser** over the raw `/snapshot`
  JPEG — it is *not* burned into the image. ROI polygons are normalized [0..1]
  coords (`Roi.px[]/py[]`, `nPoints`, up to `MAX_POLY=8`; up to `MAX_ROIS=12`).
- Decode/encode helpers are available via `img_converters.h`: `jpg2rgb565()`
  (already used by `cv.cpp`) and `fmt2jpg()` for RGB565 → JPEG.
- `lastResult` (`CvResult`, per-bay `slots[i].occupied`) is updated every capture
  cycle and is already handed to `mqttc` via `mqttBegin(cfg, last)` (stored as
  `s_last`). `s_cfg` holds the live `Config`.
- Time is available via `clk.h`: `clockEpoch()` (UTC seconds, 0 until NTP sync)
  and `clockIso()` (ISO-8601 string).

## MQTT topic surface

`base` = `cfg.mqttBaseTopic` (default `parking-valet`). `i` = 0-based ROI index.

### State — device publishes, retained
| Topic | Payload | Notes |
|---|---|---|
| `base/count` | integer | existing |
| `base/bay/<i>/state` | `ON` / `OFF` | `ON`=occupied. One per configured ROI. |
| `base/photo` | JPEG bytes | overlay image, published on command, **retained** |

### Commands — device subscribes
| Topic | Payload | Action |
|---|---|---|
| `base/cmd/photo` | any | capture + overlay + publish `base/photo` (rate-limited) |
| `base/cmd/mark_all_free` | any | `cvRecalibrate(-1)` |
| `base/bay/<i>/set` | `free` | `cvRecalibrate(i)` |
| `base/bay/<i>/set` | `occupied` | `cvMarkOccupied(i)` |

One wildcard subscription `base/bay/+/set` covers every bay; `base/cmd/photo` and
`base/cmd/mark_all_free` are subscribed explicitly.

## Home Assistant auto-discovery

Added alongside the existing `FIELDS[]` sensor configs, all under the same device
(`addDevice()`), retained, under the discovery prefix:

- **N × binary_sensor** — `dev_cla: occupancy`, `stat_t: base/bay/<i>/state`,
  name = ROI name, `uniq_id: parkingvalet_bay<i>`.
- **N × 2 buttons** — `<name> – mark free` (`cmd_t: base/bay/<i>/set`,
  `payload_press: free`) and `<name> – mark occupied` (`payload_press: occupied`),
  `uniq_id: parkingvalet_bay<i>_free` / `_occ`.
- **1 × button** "Take photo" — `cmd_t: base/cmd/photo`, `payload_press: 1`.
- **1 × button** "Mark all free" — `cmd_t: base/cmd/mark_all_free`.
- **1 × camera** "Snapshot" — `t: base/photo` (raw JPEG, no `image_encoding`),
  `uniq_id: parkingvalet_photo`.

For 12 ROIs that is ~36 entities, all grouped under the one device.

### Discovery refresh on ROI changes

Editing ROIs (count, names, enable) does **not** currently trigger
`mqttReconfigure()`. To keep HA entities in sync, `mqttLoop()` tracks a cheap ROI
signature (hash of `roiCount` + each `name` + `enabled`). When it changes,
re-publish bay discovery + bay state. Discovery is also (re)published on every
(re)connect, as today.

**Stale-entity cleanup:** when `roiCount` shrinks, publish an empty retained
payload to the config topics of bay indices `roiCount..MAX_ROIS-1`
(binary_sensor + both buttons) so HA removes the dropped entities.

## Module: `overlay.cpp` / `overlay.h` (new)

Single public function:

```c
// Capture a fresh frame, burn the ROI overlay in, and return a JPEG.
// *out is malloc'd (free() it); returns JPEG length, or 0 on failure.
size_t overlayRenderJpeg(const Config& cfg, const CvResult& cv, uint8_t** out);
```

Pipeline:
1. `esp_camera_fb_get()` a fresh frame. If null (e.g. camera stopped during OTA),
   return 0.
2. `jpg2rgb565()` into a PSRAM RGB565 buffer, **downscaled** (decoder
   `JPG_SCALE_*`, same `pickScale` idea as `cv.cpp`) to cap working width at
   ~≤800 px regardless of the configured capture framesize. This bounds PSRAM
   (≤ ~960 KB transient at 800×600×2) and output size.
3. For each ROI, map normalized polygon → pixel coords at the decoded size and
   draw onto the RGB565 buffer:
   - edges (2 px, Bresenham) and a translucent interior fill,
   - color: green=free, red=occupied, grey=disabled (web-UI palette:
     `#2ecc71` / `#ff5b5b` / grey),
   - the bay name as a label near the polygon centroid.
4. Draw the timestamp (`clockIso()` if `clockEpoch()>0`, else `uptime <s>s`) in a
   corner.
5. `fmt2jpg(RGB565 → JPEG, cfg.jpegQuality)` → `*out`.
6. Return the fb, free the RGB565 buffer, return the JPEG length.

Text uses a small bundled 5×7 ASCII bitmap font (printable 0x20–0x7E, ~475 bytes),
drawn at 2× scale with a dark filled backing rectangle for legibility. The font
and all draw helpers (`drawLine`, `fillPolyTint`, `drawText`) are `static` inside
`overlay.cpp`.

The renderer reads occupancy from `cv.slots[i].occupied` and geometry/enable from
`cfg.rois[i]`. It does **not** re-run detection — it uses the latest `lastResult`.

## Wiring changes in `mqttc.cpp`

- Add `extern void cvRecalibrate(int); extern void cvMarkOccupied(int);` (mirror
  `web_server.cpp`); include `overlay.h` and `clk.h`.
- State, added in `mqttBegin()`: `s_mqtt.setCallback(onMqttMessage)`.
- `static volatile bool s_photoReq; static uint32_t s_lastPhotoMs; static uint32_t s_roiSig;`
- `onMqttMessage(topic, payload, len)`:
  - `base/cmd/photo` → `s_photoReq = true` (do **not** render inside the receive
    callback).
  - `base/cmd/mark_all_free` → `cvRecalibrate(-1)`.
  - `base/bay/<i>/set` → parse `i`; payload `free` → `cvRecalibrate(i)`,
    `occupied` → `cvMarkOccupied(i)`.
- `connectNow()`: after `publishDiscovery()` + `publishState()`, subscribe to the
  three command topics.
- `publishDiscovery()`: extended to also emit the bay binary_sensors, bay buttons,
  the two control buttons, the camera, and the stale-index cleanup.
- `publishState()`: extended to also publish `base/bay/<i>/state` (`ON`/`OFF`) for
  `i in 0..roiCount`.
- `publishPhoto()` (new): `overlayRenderJpeg()`, then stream with
  `s_mqtt.beginPublish(base/photo, len, /*retained=*/true)` + chunked `write()` +
  `endPublish()` (required — the JPEG far exceeds the 2048-byte buffer). Free the
  buffer. No-op if not connected or render returned 0.
- `mqttLoop()` after `s_mqtt.loop()`:
  - if the ROI signature changed → republish discovery + bay state;
  - if `s_photoReq` and `now - s_lastPhotoMs > PHOTO_MIN_MS` (e.g. 3000) →
    clear the flag, stamp `s_lastPhotoMs`, call `publishPhoto()`.

The render + publish runs on the loopTask (after the receive callback returns),
takes ~1–2 s, and is well under the 90 s hang-watchdog limit. It briefly pauses
capture/analysis, acceptable for an on-demand action.

## Error handling & edge cases

- Camera unavailable (OTA deinit) → `overlayRenderJpeg` returns 0; `publishPhoto`
  skips, no crash.
- PSRAM alloc failure for the RGB565 buffer → return 0, skip.
- Clock not yet synced → timestamp shows uptime instead of a wall-clock time.
- Photo command spam → rate-limited by `PHOTO_MIN_MS`.
- Disabled bays → state published `OFF`, polygon drawn grey.
- Retained ~50 KB photo → the broker's max packet / message size limit must allow
  it (note in user-facing docs).
- TLS (mqtts) → `beginPublish`/`write` work over `WiFiClientSecure`, just slower.

## Out of scope (YAGNI)

- Auto-publishing the overlay photo on count change.
- A `select`/override entity to *force* ground-truth state (the mark commands are
  calibration nudges, not hard overrides — same semantics as the web UI).
- Per-bay edge/threshold values over MQTT (already on the stats/telemetry path).
- Configurable overlay resolution/quality knobs (uses the capped width +
  `cfg.jpegQuality`).

## Files touched

- `src/overlay.h`, `src/overlay.cpp` — new renderer + bundled font.
- `src/mqttc.cpp` — subscriptions, callback, per-bay discovery + state, photo
  publish, ROI-change refresh.
- (no change to `cv.*`, `main.cpp`, or the web UI.)
