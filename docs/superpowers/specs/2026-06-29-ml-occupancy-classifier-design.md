# On-device ML occupancy classifier — design

**Date:** 2026-06-29
**Status:** Approved design (pending final user sign-off on scope/effort)
**Author:** brainstorming session (Claude + Giovanni)

## Problem

The current occupancy detector classifies each parking bay from a **single feature** —
normalized edge/gradient energy vs a threshold (with hysteresis + an N-frame debounce).
A single hand-tuned threshold is the weakest classical method and is not robust to
weather/lighting: shadows, wet asphalt, snow, and night all shift edge energy and cause
misclassification.

Goal: a **weather-robust, on-device** occupancy decision, trained on the device's own
camera data, without moving inference off the device.

## Decisions (and why)

These were settled during brainstorming. Algorithm-level choices were made by Claude
(Giovanni defers ML/CV internals); Giovanni chose the deployment direction.

1. **Inference runs on-device.** (User.) Keeps the device self-contained; no dependency
   on the homelab/network for occupancy.
2. **Training data is harvested from this exact camera.** (User.) Weak-labelled from the
   current edge engine, with hand-correction of the weather-hard frames. A fixed install
   only ever sees one camera, so same-camera data is what matters.
3. **Staged approach, classifier first.** (User.) Phase 1 = a lightweight multi-feature
   trained classifier. Phase 2 (only if Phase 1's measured accuracy is insufficient) =
   a tiny int8 CNN, ideally after moving to an ESP32-S3. See "Phase 2" below.
4. **Replace only the decision step.** Keep ROI polygons, per-bay EMA empty-baseline,
   hysteresis, the N-frame debounce, and the commit/count logic. We swap
   `edge → threshold` for `features → trained classifier`. Surgical, low-risk.
5. **One shared classifier for all bays.** Features are bay-agnostic after per-bay
   normalization, so a single model serves all 12 ROIs. Less data, simpler.
6. **Remote management via a WireGuard client.** (User.) The whole training/operation
   loop — toggling capture mode, reviewing snapshots, recalibrating, switching engines,
   and OTA-ing a retrained model — must be drivable from outside the LAN. The device dials
   **out** and joins the existing homelab WireGuard network as a peer, so its web UI + OTA
   become reachable remotely without exposing anything to the public internet. This is
   independent of the classifier work but is required to make training/iteration remote.

### Why not a CNN now (research-backed)

A background research workflow (4 researchers + synthesis, 2026-06-29) found:

- On a **fixed camera**, a CNN's accuracy edge over a *proper multi-feature* classical
  classifier is only ~3 percentage points. The large (~9 pp) CNN advantage is
  *cross-camera* generalization — which a permanently-mounted device never needs.
- The current failures are **not** a verdict on classical ML — they come from using a
  *single* feature. The literature's robust 94–99% comes from *multi-feature texture*
  descriptors (LBP/LPQ) the device doesn't yet use (it computes `meanI` and has decoded
  colour available but ignores both).
- A CNN *is* feasible here (with `esp-tflite-micro` + ESP-NN a 32×32 int8 net is
  ~8–15 ms/bay, chunked one bay per loop) — but it costs a fiddly TFLite-Micro/ESP-NN
  Arduino integration, int8 quantization with train/serve pixel-parity risk, a
  ~20–40 KB SRAM tensor arena, ~100–160 KB flash, and more labelled data — for ~3 pp.

So Phase 1 captures most of the gain at a tiny fraction of the cost/risk, and the
labelled dataset built for it is the reusable asset if we ever escalate to the CNN.

## Architecture

```
INFERENCE (normal run), per capture cycle, per bay:
  decoded luma + RGB565  ──▶  feature extraction  ──▶  trained classifier  ──▶ raw occupied
                                                                                  │
                              [existing hysteresis + N-frame debounce + commit] ◀─┘ ──▶ count

CAPTURE (training-data collection; UI toggle), per cycle, per bay:
  feature extraction ──▶ log { features, weakLabel (current edge decision), margin,
                               roiSig, frameId, ts }  +  send snapshot
        │
        ▼
  harvest records + snapshots (n8n) → hand-correct the wrong labels (mostly weather-hard)
        │
        ▼
  tools/train_classifier.py  → train + validate → export src/clf_model.h → commit → OTA
```

### Key design choice: log features, not pixels (parity by construction)

The dominant silent-failure risk in on-device ML is **train/serve mismatch** — the model
trains on pixels that don't match what the device produces (JPEG→RGB565 banding,
downscale filter, luma rounding). We eliminate it: the **device computes the feature
vector and logs that exact vector**, and training happens on those literal numbers. No
need to replicate the firmware's pixel pipeline in Python. This is the main reason the
classifier path is materially lower-risk than the CNN path, and why "harvest from your
own camera" composes so cleanly.

## Components

### 1. Feature extraction (firmware, C++ — extends `cv.cpp`)

Per bay, one pass over the bay's pixels (reusing the existing decoded luma + RGB565),
producing a fixed-length vector (~20–30 values), lighting-normalized where possible:

- **Baseline-relative edge density** — current edge energy minus the per-bay learned
  empty baseline (already half-built; reuse).
- **Normalized mean luma** — `meanI` normalized to a per-bay/frame reference to cancel
  global brightness drift.
- **Luma variance / std** — flatness cue (empty asphalt is flat; a car adds structure).
- **Texture histogram** — a coarse **uniform-LBP** histogram (~10 bins) and/or a small
  gradient-orientation histogram (~8 bins). This is the key illumination-invariant
  ingredient the current pipeline lacks.
- **Colour / saturation stats (from RGB565)** — mean saturation + a neutral/blue cue, to
  separate wet (dark, specular), snow (bright, desaturated), and asphalt (neutral grey).

All features are cheap (integer-friendly, single pass) and have **no learned weights** on
the device — they are deterministic transforms, so the same code runs in capture and
inference.

### 2. Classifier (trained offline, embedded as generated C)

- Train **both** an L2-regularized **logistic regression** and a **small
  gradient-boosted-trees** model offline; pick whichever validates best on the held-out
  hard cases. (Expectation: LR is likely sufficient given a modest, partly-correlated
  dataset and resists overfitting; GBDT is the fallback if non-linear feature
  interactions matter.)
- One shared model for all bays. Output = occupied score → fed into the **existing**
  hysteresis + debounce (we do not re-implement temporal smoothing).
- Export the winner to `src/clf_model.h` (generated C: a dot-product + sigmoid for LR, or
  nested if/else for trees via `m2cgen`/`micromlgen`). A few KB; no inference runtime
  library needed; microsecond latency per bay; zero pressure on the 90 s hang-watchdog.

### 3. Capture mechanism (firmware)

- New config flag `trainCapture` (bool, default off). When on, each cycle the device
  POSTs a JSON batch of per-bay records to a capture endpoint (reusing the existing
  webhook auth machinery) **and** sends the matching snapshot, so n8n stores both for
  review. (Alternative if preferred later: write CSV to SD when an SD card is present.)
- Records carry the weak label (the current edge engine's committed decision) + margin so
  high-confidence samples can be auto-accepted and only low-margin / known-hard frames
  need manual correction.

### 4. Offline training pipeline (`tools/train_classifier.py`)

Reads the collected records, fixes class imbalance (class weights / oversampling), trains
LR + GBDT, reports **per-class precision/recall + confusion matrix on the hand-verified
hard cases** (never top-line accuracy), exports the winner to `src/clf_model.h`, and
prints the on-device feature/weight summary. Retraining = rerun the script + OTA.

### 5. Config + UI

- Config: `occupancyEngine` (0 = edge-threshold legacy, 1 = trained classifier),
  `trainCapture` (bool), and capture URL/auth (reuse the webhook pattern).
- UI: an engine selector, a capture toggle, and the per-bay classifier score +
  edge-vs-classifier disagreement shown in the existing Slots table.

### 6. Remote access (WireGuard client)

So the training loop can be driven remotely, the device joins the homelab WireGuard
network as an outbound peer; once the tunnel is up, the existing web server + OTA are
reachable on the device's tunnel IP (lwIP listens on all interfaces — no per-interface
binding needed). **Library/footprint/lifecycle details are being verified against the exact
Arduino-ESP32 2.0.17 / `espressif32@6.9.0` toolchain by a background feasibility check;
this section will be reconciled with its findings.** Working assumptions:

- **Role:** device = WireGuard *client* dialing OUT to the homelab WG endpoint (the device
  is behind NAT and initiates the tunnel). Persistent-keepalive keeps the NAT mapping open.
- **Lifecycle:** bring the tunnel up only in STA mode, **after** WiFi is connected **and**
  NTP has synced (WG handshakes use TAI64N timestamps and fail with a wrong clock). Tear
  down / re-establish on WiFi drop+reconnect. WG is irrelevant in AP fallback mode.
- **Config (NVS; private + preshared keys are secrets, masked in the API like other
  credentials):** `wgEnabled` (bool), `wgPrivateKey` (device, secret), `wgPeerPublicKey`
  (server), `wgEndpointHost`, `wgEndpointPort`, `wgLocalIp` (device address inside the
  tunnel), optional `wgPresharedKey` (secret) and `wgKeepalive` (seconds). The
  config-store already documents that NVS secrets are plaintext unless flash encryption is
  enabled — the WG private key inherits that caveat.
- **Scope note:** this is a self-contained firmware capability (new `wg.*` module + config
  + a small UI card). It is orthogonal to the classifier and can land before, after, or
  alongside it; it is grouped here because the user requires it for *remote* training.

## Data flow & error handling

- **Inference:** classifier score replaces the raw edge decision; everything downstream is
  unchanged. If `occupancyEngine = 0`, or the model header is an empty placeholder, the
  device uses the legacy edge threshold — so it always degrades gracefully to today's
  behaviour rather than breaking.
- **Disagreement monitor:** the legacy edge engine keeps running in parallel; per-bay
  disagreement is logged/surfaced so drift and regressions are visible, and so live A/B is
  possible before fully trusting the classifier.

## Testing

- **Offline:** held-out per-class precision/recall on hand-labelled hard cases
  (night / wet / snow / shadow, both classes).
- **On-device:** disagreement counter (classifier vs edge) + extended per-bay UI metrics;
  confirm `.bin` flash size and free internal SRAM after the build (budget is ample for
  Phase 1 since there is no ML runtime, but measure).
- **Parity:** guaranteed by construction (training consumes device-emitted feature
  vectors), but spot-check a few device-vs-recomputed vectors.

## What Giovanni has to do (effort)

0. One-time: set the WireGuard config (keys + homelab endpoint) so the device is reachable
   remotely. After that, **all the steps below can be done from outside the LAN** — toggling
   capture, reviewing, retraining, and OTA — which is the point of adding WireGuard.
1. Flash a build with capture mode and turn it on; let it collect across conditions. The
   unavoidable cost here is **calendar time** — the model can only learn weather it has
   seen (no snow examples until it snows), regardless of approach.
2. Periodically skim the collected snapshots and **correct only the labels the device got
   wrong** (mostly the weather-hard frames) — a spreadsheet/Telegram review, not labelling
   from scratch.
3. Run one Python script to train + regenerate the model header.
4. OTA the new firmware and flip the engine to "classifier"; watch the disagreement/accuracy.

Expected outcome: substantially better weather robustness at microsecond on-device cost,
with the current engine as an instant fallback.

## Phase 2 (only if Phase 1 is insufficient) — tiny int8 CNN

Documented so the staged plan is concrete; **not built unless Phase 1's measured accuracy
on hard cases is unacceptable.**

- Runtime: `esp-tflite-micro` (TFLM) with **ESP-NN** enabled (the only int8-CNN runtime
  supported on plain ESP32-LX6 + Arduino-ESP32 2.0.17 / IDF 4.4; ESP-NN gives ~10× via
  optimized C, not SIMD). **Not** ESP-DL (needs IDF 5.3+). Register only used ops via
  `MicroMutableOpResolver`.
- Model: one shared 32×32×1 grayscale int8 CNN (Conv s2 1→8→16→32 → GAP → Dense→2),
  ~166 K MACs, <10 KB `.tflite`. Step to 48×48 if 32×32 underfits.
- Scheduling: one bay per `loop()` iteration (~8–15 ms/bay with ESP-NN); tensor arena
  (~20–40 KB) in internal SRAM if it fits, else PSRAM (slower, camera-DMA/flash-cache
  contention). Never run a PSRAM-arena inference during an OTA flash write.
- Strongly prefer migrating to an **ESP32-S3** before committing to a per-bay CNN.

## Risks

- **Data is the real work / dominant accuracy risk.** Local, weather-diverse capture from
  this camera is mandatory; missing conditions = failure in those conditions, any approach.
- **Class imbalance** — a private lot sits mostly-empty or mostly-full for long stretches;
  weak-labeller errors concentrate in one class. Handle with class weights/oversampling and
  judge by per-class P/R, not accuracy.
- **Feature sufficiency** — if the engineered features underfit the hardest cases, escalate
  to Phase 2 (CNN). The dataset built in Phase 1 carries over.
- **Capture-mode load** — POSTing features + snapshots adds traffic while enabled; it is
  off by default and only used during collection.
- **WireGuard on this toolchain** — library maturity for Arduino-ESP32 2.0.17 / IDF 4.4
  and coexistence with esp32-camera must be confirmed (background check in progress); the
  tunnel depends on NTP being synced first; and the WG private key sits in NVS plaintext
  unless flash encryption is enabled.
```
