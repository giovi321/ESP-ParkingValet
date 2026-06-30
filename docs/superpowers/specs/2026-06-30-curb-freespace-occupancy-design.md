# On-street curb free-space occupancy — design

**Date:** 2026-06-30
**Status:** Approved design (pending final user sign-off on the written spec)
**Branch:** `dev-ml` (continues the new-version work; supersedes the fixed-bay occupancy output)
**Builds on:** the WireGuard remote access + the per-region feature/classifier/capture/training
machinery already on `dev-ml` (`features.h`, `cv.cpp` ray-cast accumulator + relative-edge
baseline/hysteresis/debounce, `clf.*`, `capture.*`, `tools/train_classifier.py`).

## Problem

The monitored parking is **on-street, curb-side, with no marked bays** (and one extra spot
*on the sidewalk*, parallel to the curb). Cars of varying length park informally — mostly
parallel, at varying angles and gaps — so the number of free spaces depends on how the cars
pack. The fixed-bay model ("count occupied bays") is the wrong abstraction. The right one is
**free-space estimation along the parkable curb**.

## The reframe (headline output)

Because car length genuinely varies (~3.5–5.8 m) and parking is informal, an exact free-spaces
*count* is false precision. The headline is a robust measurement plus a labelled estimate:

- **`free_curb_m`** — total free curb length in metres (PRIMARY; agnostic to vehicle type).
- **`longest_free_run_m`** — the single most trustworthy derived number.
- **`can_fit`** (bool) — `longest_free_run_m >= one car`; the highest-confidence signal.
- **`est_free_spaces`** — sum over gaps of `floor(...)`, clearly labelled an ESTIMATE, with an
  optional optimistic/pessimistic band (recompute at pitch 5.5 m and 6.7 m).
- **`occupied_fraction`**, per-gap diagnostics, and a **light-confidence flag**.

Realistic accuracy is ~86–96% in daylight, degrading at the far end of the strip and at dusk.
This is a "how much curb is free / is there a spot" sensor, not a per-car ground-truth counter.

## Decisions (with the user's answers)

1. **Curb mode REPLACES the fixed-bay mode.** (User.) A single occupancy model. Per-bay
   polygons, the per-bay MQTT occupancy sensors, and the bay UI are removed. The CV/ML **core**
   (features, classifier slot, capture, training) and WireGuard are RETAINED and applied
   per-cell.
2. **Multiple parkable strips** (1..K). (Derived from: informal-parallel curb + one sidewalk
   spot.) Each strip is independently calibrated; the sidewalk spot is just a short strip.
   Outputs aggregate across strips.
3. **Geometry = informal parallel** (mostly parallel, varying angles/gaps). (User.) Free-length
   is geometry-robust; the car-count is a soft estimate that auto-learn adapts to. NOT marked
   angled/perpendicular stalls.
4. **Hard zone boundaries, both ends in view.** (User.) Both terminal gaps are real "walls" and
   use the end clearance; no FOV-edge partial-run dropping needed for the in-view ends (kept for
   robustness anyway).
5. **Day operation with winter-dusk tolerance.** (User.) A luminance confidence gate flags/
   down-weights numbers when the scene is too dark; daylight + lit dusk report normally. No
   IR/night scope.
6. **Calibration: standard car-pitch default + auto-learn REFINER.** (User chose auto-learn.)
   Auto-learn cannot bootstrap (cold start / outliers), so the foundation is a one-time metric
   anchor (needed for perspective regardless) + a standards pitch default; auto-learn then
   refines the real pitch from observed isolated cars. The user never hand-calibrates car size.

## Architecture

A new module `src/curb.cpp` / `src/curb.h` owns the parkable-area model and the free-space
computation. It does NOT overload the 12-bay arrays — those are removed. It reuses, verbatim:
`FeatureAccum` + `featuresFinalize()` (features.h), the per-polygon ray-cast accumulator loop
and the relative-edge baseline+hysteresis+debounce primitives (today in `cv.cpp`), and the
NVS-persist pattern (`CvPersist`). The per-frame pixel work is the same machinery applied to
more, smaller polygons — total cost comparable to today's 12-ROI pass.

```
SETUP (host-side, browser):
  per strip: user marks 4 image corners of the parkable quad + enters real length L_m (+ approx
  width W_m). Browser computes a ground-plane homography H (4 coplanar correspondences, no
  camera intrinsics) and lays out N = round(L_m / g) cells of EQUAL GROUND length g (~1.0–1.5 m):
  cell i's real corners (i·g,0),((i+1)·g,0),((i+1)·g,W),(i·g,W) → through H → a 4-vertex image
  quad → stored as the normalized px[]/py[] polygons the firmware already consumes. Per-frame
  perspective cost = ZERO; every free cell is exactly g metres. Dead-zone cells (driveways,
  hydrants) get enabled=false.

PER FRAME (device):
  decode JPEG → downscaled luma + RGB565 (existing)
  per strip, per enabled cell:
     ray-cast accumulate → featuresFinalize() → 16-vector
     occupied/free decision vs the cell's OWN empty-asphalt baseline (+ per-cell hysteresis + N-frame debounce)
  → width-3 median smooth the 1-D label chain per strip
  → run-length the FREE cells into gaps (skip masked cells)
  → gap metres → free_curb_m, longest_free_run_m, can_fit, est_free_spaces (per strip, summed)
  → final-integer debounce/hysteresis on est_free_spaces + light-confidence gate
  → publish MQTT (+ overlay)
```

## Components

### 1. Strip + cell model (`curb.h`)
- `MAX_STRIPS` (e.g. 4), `MAX_CELLS` total (e.g. 64) across all strips.
- Per strip: `realLenM`, `realWidthM`, `cellLenM` (g), `nCells`, and the generated cell polygons
  (normalized `px[]/py[]`, reusing the ROI polygon representation, ≤ `MAX_POLY` vertices).
- Per cell: `enabled` (dead-zone mask) + runtime state: `committed`, `baselineEdge` (EMA),
  `baselineInit`, `lastRaw`, `stableCnt` — the same five fields the bay engine kept, sized to the
  cell count.
- Cell-quad generation is **host-side JS** at calibration; the firmware just consumes polygons.

### 2. Per-cell occupied/free
- Reuse the ray-cast loop → `featuresFinalize()` → `feat[16]` per cell.
- **Day-one engine = relative-edge** (each cell vs its own baseline): `metric = edge − baselineEdge`
  with the existing `enter = thr·(1+h)` / `exit = thr·(1−h)` hysteresis + `stableFrames` debounce.
  Per-cell baselines make the decision naturally normalised and sidestep cross-cell absolute-edge
  scaling (which varies with perspective pixel density).
- **OCCUPIED** if: edge-above-baseline high (car body/wheels/panel gaps) **OR** LBP texture
  (`feat[4..13]`) deviates from the cell's learned asphalt histogram **OR** chroma
  (`feat[14]/[15]`) is high. **Colour is a POSITIVE car vote only**; low saturation is NEUTRAL
  (grey/silver/black/white cars are achromatic). **FREE** only when edge AND texture both read
  road-like. Shadows: lean on chroma + LBP + the relative baseline; **never threshold raw luma**.
- **Trained classifier upgrade:** when `occupancyEngine==1 && clfScore>=0`, `clfScore()` is the
  primary `p_occ` with probability hysteresis (the path already exists from Part B). The classifier
  is the per-cell occupied/free model; `trainCapture` logs per-cell vectors (existing machinery).

### 3. Spatial smoothing
- Width-3 median filter on the binary label chain per strip BEFORE run-length (kills single-cell
  salt-and-pepper that would split/invent a gap). Optional 2-state Viterbi (O(4N) ints) as a
  tunable upgrade.

### 4. Free-gap → free length → estimated cars
- Run-length-encode FREE runs per strip (skip masked cells). `G_m = free_cells · g`.
- Convert with **pitch**, not bare footprint:
  - interior gap (parked cars both sides): `n = floor((G_m − clearInteriorM) / carPitchM)`
  - terminal gap (touches a hard zone boundary): `n = floor((G_m − clearEndM) / carPitchM)`
  - discard runs with `G_m < carPitchM`; subtract a half-cell guard at each run end so a
    half-occupied boundary cell cannot fabricate a space; a run touching the FOV edge drops its
    partial terminal cell (not needed for the in-view ends here, kept for robustness).
- Defaults (Config, tunable): `carPitchM=6.0`, `clearInteriorM=1.2`, `clearEndM=1.8`.
- `free_curb_m` = Σ run metres; `est_free_spaces` = Σ n; `longest_free_run_m` = max run;
  `can_fit` = `longest_free_run_m >= carPitchM − clearInteriorM` (≈ one car footprint).

### 5. Calibration
- **Metric anchor (required):** 4 corners + `realLenM` (+ approx `realWidthM`) per strip → the
  homography (host-side) → gaps measured in real metres. This single input eliminates the
  position-dependent car-length problem and powers perspective.
- **Empty-strip seed:** a "mark all free / strip empty" action re-seeds every cell's
  baseline+texture from one known-empty frame (generalises `recalibrate(index<0)`). Required so
  the first frame doesn't read occupied.
- **Auto-learn refiner (the user's choice):** an NVS histogram of the lengths of ISOLATED
  occupied runs (free on both sides, shorter than ~1.5× current pitch to exclude multi-car runs);
  take the lowest dense cluster as the single-car footprint, add `clearInteriorM`, and EMA it
  slowly toward the configured pitch **gated behind K ≥ ~30 samples**. It (i) flags in UI/MQTT
  when the configured pitch disagrees with observation and (ii) tracks slow fleet/camera drift.
  Never bootstraps from nothing; never overrides an explicit "street full → enter N" calibration.
- **Optional "street is full → enter N":** sets `carPitchM = strip_len_m / N` (most accurate;
  optional because a full street may be rare).
- **Recalibrate on camera move:** one action re-runs corner marking and re-seeds cell baselines
  (the homography and baselines are valid only for one fixed pose).

### 6. Perspective (host-side, zero per-frame cost)
- One-time 4-corner planar homography per strip, computed in the browser at calibration
  (`getPerspectiveTransform`/DLT — 4 coplanar points fully determine the 8-DOF H; the curb is a
  single ground plane, so no intrinsics/pose/lens model needed). Cells come out short-in-pixels
  far from the camera, long near it — each equal ground length. No runtime warp.
- Flag far cells whose mapped pixel area drops below a floor as **low-confidence /
  resolution-starved** (optionally merge). Rely on per-cell relative/normalized features, not
  raw cross-cell absolute edge.
- The sidewalk strip is on the raised pavement plane → its own corners + homography (per-strip
  flat-ground assumption holds).

### 7. Output + stability
- **MQTT** (replace the per-bay `count`/occupancy entities; extend the discovery in `mqttc.cpp`):
  `free_curb_m`, `longest_free_run_m`, `can_fit`, `est_free_spaces` (+ optional band), and
  `occupied_fraction`, plus a `confidence`/`dark` flag and optional per-strip / per-gap detail.
- **Three smoothing layers:** (1) per-cell hysteresis + `stableFrames` debounce; (2) spatial
  median per strip; (3) final-integer debounce/hysteresis on `est_free_spaces` so it doesn't flap
  between N and N+1 as a vehicle passes or light shifts. A *parked* occluder persists → biases
  toward UNDER-counting free space (the safe direction).
- **Light-confidence gate:** when scene luma is low (winter dusk / glare), down-weight or flag the
  numbers rather than asserting them. Daylight + lit dusk report normally.

### 8. Persistence
- New NVS blob for per-cell baseline state + the auto-learned pitch, mirroring `CvPersist`, guarded
  by a magic + a strip/cell-geometry signature so re-drawing the strips drops stale baselines.

### 9. UI (web)
- Strip editor: add/remove strips; per strip mark 4 corners, enter real length (+ width); the
  browser previews the generated cells over the snapshot (reusing the overlay, which already draws
  image-space polygons via `overlay.cpp`).
- Dead-zone masking: toggle individual cells off (driveways/hydrants).
- Actions: "mark all empty" (seed baselines), "recalibrate (camera moved)", optional "street full → N".
- Tunables: `cellLenM` (g), `carPitchM`, clearances, smoothing mode, light threshold, engine
  (classical vs trained classifier).
- Live readout: cells coloured occupied/free, free runs highlighted, and the headline numbers.

## Data flow & error handling
- If a strip has no cells / no real length set, it contributes nothing (no crash).
- If `occupancyEngine==1` but no model is embedded (`CLF_MODEL_PRESENT 0`, today's state), the
  classical relative-edge engine runs — day-one works without a trained model.
- Baselines not yet seeded → first frames flagged low-confidence until "mark all empty" or the
  adaptive EMA warms up.
- Dark frames → light-confidence flag; numbers published but marked unreliable.

## Testing
- **Host (pytest):** the gap→metres→cars reducer and the auto-learn histogram/median logic are
  pure functions — unit-test them (run-length, interior vs end clearance, guard bands, discard
  `< pitch`, optimistic/pessimistic band, isolated-run extraction, K-gate). The homography
  cell-layout (browser JS) gets a small JS test: a synthetic quad maps a known car length to ~the
  expected cell span.
- **On-device (deferred to hardware):** build green; then per-cell feature sanity, "mark empty"
  seeding, a known-reference check (a car of known length should span ~`car/g` cells), and free-run
  behaviour as cars come/go. Capture + train a per-cell model later.

## Risks (from research)
- **Resolution floor × perspective:** at ~200×150 analysis a full strip gives only ~5–8 px/cell;
  far cells cover more ground/pixel → distant gaps coarsely quantized, far cars may be too few
  pixels. Mitigate: tall (sidewall) strip; flag/merge resolution-starved far cells; optionally
  decode the strip one JPG scale finer (costs PSRAM/CPU vs the 90 s watchdog).
- **Achromatic cars** defeat the colour cue and a clean panel can read low-edge → a car read as
  FREE = OVER-count (unsafe direction). Texture/LBP + relative-edge baseline must carry the
  negative; colour positive-only.
- **Calibration gates everything and fails silently:** sloppy corners / wrong real length skews the
  scale with no visible error; a nudged camera invalidates both H and baselines. Mitigate: explicit
  recalibrate action + a known-reference sanity check.
- **Quantization & wrong pitch:** ±1 cell per gap edge can flip `floor()` on a tight one-car gap;
  guards cut false positives but can suppress a genuine tight space (under-count). Pitch/clearances
  are site-specific — exposed and tunable; auto-learn refines.
- **Occlusion / double-parking:** a tall van in a near lane occludes far cells; a parked delivery
  vehicle reads occupied → conservative UNDER-count (safe), but a stationary occluder persists.
- **Night/dusk/glare** degrades classical features → the light-confidence gate flags rather than
  asserts.
- **Dead zones can't be inferred** from appearance → user-masked per cell; the mask is static, so
  new no-parking paint / a blocked driveway is mis-reported until updated.

## Phasing (for the plan)
1. Strip/cell model + host-side homography cell layout + UI strip editor + persistence.
2. Per-cell occupied/free (classical) + per-cell baselines + "mark all empty" + spatial median.
3. Free-gap reducer → outputs + MQTT entities + overlay + stability layers + light gate.
4. Calibration: pitch defaults + tunables + auto-learn refiner + recalibrate action.
5. Per-cell capture/training wired to the existing classifier slot (optional upgrade path).
```
