# Curb Free-Space Occupancy — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the fixed-bay occupancy detection with an on-street **curb free-space** model — one-or-more traced curb strips subdivided into cells, per-cell occupied/free reusing the existing CV machinery, free-gap run-length → free curb metres + "is there room?" + a soft estimated count — **while leaving every non-detection platform feature fully intact.**

**Architecture:** Rewrite the occupancy core (`cv.*`, the occupancy `Config` fields, the result struct, the persist blob, the overlay draw loop) in place, reusing the ray-cast accumulator + `featuresFinalize()` + relative-edge/hysteresis/debounce primitives **verbatim, per cell**. Every transport (WiFi/AP/WireGuard/MQTT/webhook/stats/spool/OTA/backup-restore/auth/NTP/snapshot/serial) is **preserved**; only the occupancy *field values* flowing through each are repointed at the documented seams.

**Tech Stack:** C++ / Arduino-ESP32 2.0.17 (PlatformIO `espressif32@6.9.0`); the built-in WebServer + the existing CV/overlay/MQTT/spool stack; host-side browser JS for the strip-tracer + cell layout; existing `tools/train_classifier.py` (per-cell rows). Design spec: `docs/superpowers/specs/2026-06-30-curb-freespace-occupancy-design.md` (read it).

## Global Constraints

- **PRESERVE ALL PLATFORM FEATURES (hard requirement).** These must keep working and stay essentially untouched — change only the occupancy *values* at the seams below, never the transport:
  WiFi STA/AP + captive portal + `apRetryMin` auto-retry + offline-reboot watchdog (`net.cpp:20-138`); WireGuard (`wg.*`, zero occupancy coupling — DO NOT TOUCH); MQTT connection + HA discovery framework + diagnostics sensors + photo transport (`mqttc.cpp`); event webhook + stats telemetry transport + `netPostJson` (`net.cpp:165-265`); offline spool mechanics (`spool.cpp`); OTA (`web_server.cpp` `handleOtaUpload/Done`); backup/restore (`/api/backup`,`/api/restore` — schema-agnostic whole-Config); digest auth; NTP (`clk.*` — DO NOT TOUCH); snapshot/log; the web UI platform cards (WiFi, WireGuard, Watchdog, MQTT, Webhook, Stats, Spool, Image, Admin, OTA, Backup/Restore, Serial console) + the generic `F[]`/`SECRET[]`/`fill`/`collect`/`save` form engine.
- **Occupancy seams (repoint values only; from the touch-point map):**
  - A — `netSendEvent()` multipart fields `count`/`prev_count`/`slots` (`net.cpp:167,177-179`) → curb scalar fields; transport block `165-235` + `netPostJson` UNCHANGED.
  - B — spool meta-JSON keys `count`/`prev`/`slots` (`spool.cpp:153-164`, drain `255-263,277`) → curb keys; framing/caps/drain UNCHANGED.
  - C — stats payload keys `d['count']`/`d['roi_count']` (`main.cpp:228-229`) → curb keys; `netPostJson` + Stats card UNCHANGED.
  - D — MQTT diagnostics `FIELDS[]` rows `count`/`roi_count` (`mqttc.cpp:51/58/70/77`) → curb FIELDS rows; `publishDiscovery/State` UNCHANGED.
  - E — MQTT per-bay binary_sensors + free/occupied selects + `bay/+/set` (`mqttc.cpp:124-156,192-198,250-262`) → REMOVE; publish `can_fit`/`dark` as binary_sensors; `clearCfg()` purges stale retained bay configs; camera/`cmd`/photo UNCHANGED.
  - F — MQTT change-trigger `r.count!=lastMqttCount` (`main.cpp:361`) → fire on a curb headline change.
  - G — `/api/state` `doc.cv.count` + `doc.slots[]` (`web_server.cpp:118-145`) → curb headline + `doc.cells/doc.strips`; `cv.valid/decW/decH/tookMs` kept; transport UNCHANGED.
  - H — `/api/config` `rois` geometry (`config_store.cpp` serialize/parse) → `strips`/cells; endpoint + save engine UNCHANGED.
  - I — `/api/action` `recalibrate`/`mark_occupied` (`web_server.cpp:291-300`) → per-cell/strip seed; `{action,slot}` contract kept.
  - J — webhook trigger predicate `shouldSend()` over occupied count (`main.cpp:150-156`) → curb metric; `minSendIntervalMs`/heartbeat UNCHANGED.
  - K — backup/restore: change schema ONLY in `config_store.cpp` (add `strips`/cells, drop `rois`); new fields ride backup automatically.
  - L — `overlayRenderJpeg(cfg,result,out)` (`overlay.h:12`): retype 2nd param; rewrite only the draw loop (`overlay.cpp:106-146`); decode/blend/encode infra UNCHANGED.
- **Coordinated type-seam (compile-fail-fast, land in ONE task):** the result struct is the 2nd param of `webBegin` (`web_server.h`), `mqttBegin` (`mqttc.h`), and `overlayRenderJpeg` (`overlay.h`), plus `main.cpp`'s `lastResult`. Retyping `CvResult*`→`CurbResult*` must update all four headers + readers together.
- **`MAX_ROIS`→`MAX_CELLS`/`MAX_STRIPS` is a coordinated rename**, not a local edit — it sizes the result struct, the persist arrays, the spool replay buffer (`spool.cpp:161,262-263`), and bounds-checks in `web_server.cpp`/`mqttc.cpp`. No dangling `MAX_ROIS` may remain.
- **`cfg.roiCount` is read as a config-level occupancy field** outside the result struct (`main.cpp:228`, `mqttc.cpp:77`, config serialize/parse) — repoint every read to strip/cell count.
- **Structural decision:** rewrite `cv.*` IN PLACE (keep `CvEngine`, reuse its helper primitives) rather than a separate `curb.cpp` — avoids duplicating the ray-cast/decision helpers. (Deviates from the spec's "curb.cpp"; documented here.)
- **CONFIG_VERSION bump + one-time spool clear** on the schema change: an old bay-era backup or on-disk spool entry must degrade gracefully (no crash) — restoring an old backup yields zero strips (user re-traces), and the spool is cleared once on the upgrade boot so stale `count`/`slots` records don't replay as zero-value curb events.
- **No new secrets** are introduced by the curb model (geometry/pitches aren't secret); if any task adds one it MUST be wired into BOTH the `configToJson` mask and the `parseFull` non-empty secret-merge (mirror `captureAuthHeaderValue`).
- **Build = the gate.** `pio run` must stay green at every task. `src/web_ui.h` is generated from `web-src/index.html` by `tools/gen_web_ui.py` (pre-build); NEVER hand-edit it. Commit messages end with a trailer line `giovi321`. No host test framework except `tools/` pytest.
- **Camera reality (fisheye, high oblique, short range):** the strip is a traced polyline (not a 4-corner quad); cells are laid out host-side with approximate near→far ground-length; out-of-range far cells are excluded; lean on `free_curb_m`/`can_fit`, treat `est_free_spaces` as an estimate.

## Shared Contracts (define ONCE; every task uses these names verbatim)

**Constants (`config_store.h`):** `MAX_STRIPS = 4`, `MAX_CELLS = 48` (total across strips), each cell is a 4-vertex quad. `MAX_ROIS`/`MAX_POLY`/`Roi` are removed.

**Geometry config (`config_store.h`, replaces `Roi`/`rois[]`/`roiCount`):**
```cpp
struct CurbStrip {
  char  name[16];
  float realLenM;     // real-world length of the traced stretch (metres) — the metric anchor
  float realWidthM;   // approx sidewall band height (metres) — diagnostics only
  int   nCells;       // cells generated for this strip
};
struct CurbCell {
  float   px[4], py[4]; // normalized [0..1] quad vertices (host-generated along the strip)
  float   lenM;         // this cell's ground length (metres); equal within a strip
  uint8_t strip;        // owning strip index
  bool    enabled;      // dead-zone mask (driveways/hydrants/the bottom-right car) + in-range gate
};
// in struct Config (replacing the rois block):
CurbStrip strips[MAX_STRIPS];
int       stripCount;
CurbCell  cells[MAX_CELLS];
int       cellCount;
```

**Curb tunables (`config_store.h`, added near the kept CV tunables):**
```cpp
float    carPitchM;       // packed parallel pitch, default 6.0 (footprint + 1 inter-vehicle gap)
float    clearInteriorM;  // default 1.2
float    clearEndM;       // default 1.8
uint8_t  smoothMode;      // 0=none, 1=width-3 median (default 1)
float    darkLumaThresh;  // mean-luma below this => low-confidence/dark flag (default e.g. 40)
uint8_t  pitchLearn;      // 0=off, 1=auto-learn refiner on (default 1)
```
KEEP (reused per cell): `occupancyMode`, `edgeThreshold`, `relDelta`, `hysteresis`, `baselineEma`, `stableFrames`, `occupancyEngine`. KEEP (redefined meaning): `triggerMode`, `triggerThreshold` (now compare a curb metric), `minSendIntervalMs`, `heartbeatIntervalS`. DROP: the per-`Roi` `threshold`.

**Result struct (`cv.h`, replaces `SlotResult`/`CvResult`):**
```cpp
struct CellResult {
  float feat[16];     // CLF_NFEAT vector (capture + diagnostics)
  float clfScore;     // classifier prob or -1
  float edge, meanI, baselineEdge;
  bool  occupied;     // committed (post-debounce, post-smooth)
  bool  rawOccupied;  // pre-debounce
  bool  inRange;      // false => excluded far/out-of-range cell
};
struct CurbResult {
  bool     valid;
  int      decW, decH; uint32_t tookMs;
  int      nCells;
  CellResult cells[MAX_CELLS];
  // headline (aggregated across strips, in-range only):
  float    free_curb_m;
  float    longest_free_run_m;
  bool     can_fit;            // longest_free_run_m >= carPitchM - clearInteriorM
  int      est_free_spaces;
  float    reliable_range_m;   // total in-range curb length the camera resolves
  float    occupied_fraction;  // 0..1 over in-range cells
  bool     dark;               // light-confidence gate tripped
};
```

**Persist blob (`cv.h`, replaces `CvPersist`):**
```cpp
static const uint16_t CURB_PERSIST_MAGIC = 0xCB02;   // NEW magic (was 0xCB01)
struct CurbPersist {
  uint16_t magic;
  uint16_t cellCount;
  uint32_t geomSig;                 // hash of strip/cell geometry; mismatch drops stale baselines
  float    baselineEdge[MAX_CELLS];
  uint8_t  baselineInit[MAX_CELLS];
  uint8_t  committed[MAX_CELLS];
  float    carPitchLearned;         // auto-learn refiner state
  uint16_t learnSamples;            // K (gate at >= 30)
};
```

**Curb output field names (verbatim everywhere — webhook/spool/stats/MQTT/state):**
`free_curb_m`, `longest_free_run_m`, `can_fit`, `est_free_spaces`, `reliable_range_m`, `occupied_fraction`, `dark`.

---

# Phase 0 — Schema + result type + persist (the keystone; one coordinated build)

## Task C0.1: Config geometry + curb tunables + CONFIG_VERSION bump
**Files:** Modify `src/config_store.h`, `src/config_store.cpp`.
**Interfaces:** Produces the Shared-Contracts geometry structs, the curb tunables, `MAX_STRIPS`/`MAX_CELLS`; removes `Roi`/`rois[]`/`roiCount`/`MAX_ROIS`/`MAX_POLY`/per-Roi threshold.

- [ ] **Step 1:** In `config_store.h` replace the `Roi`/`MAX_ROIS`/`MAX_POLY` block and the `rois[]`/`roiCount` Config members with the Shared-Contracts `CurbStrip`/`CurbCell` + `strips[]`/`stripCount`/`cells[]`/`cellCount` + the curb tunables. Drop the per-`Roi` `threshold`. Bump `CONFIG_VERSION` by 1.
- [ ] **Step 2:** In `config_store.cpp` `configLoadDefaults`: `stripCount=0; cellCount=0;` and the curb tunable defaults (`carPitchM=6.0f`, `clearInteriorM=1.2f`, `clearEndM=1.8f`, `smoothMode=1`, `darkLumaThresh=40.0f`, `pitchLearn=1`). Keep all other defaults.
- [ ] **Step 3:** `serializeFull`: replace the `rois` array block (`config_store.cpp:199-211`) with a `strips` array (per strip: name/realLenM/realWidthM/nCells) and a `cells` array (per cell: px[4]/py[4]/lenM/strip/enabled). Emit the curb tunables. Keep every non-occupancy field.
- [ ] **Step 4:** `parseFull` (+ replace `parseRois`): parse `strips`/`cells` back (pre-seed empty; overwrite only when present, so an old backup without `strips` yields zero strips — graceful). Merge the curb tunables with the `o[k] | c.k` pattern. Remove the legacy-rectangle→polygon `parseRois` upgrade.
- [ ] **Step 5:** `pio run` → `[SUCCESS]` (the rest of the tree will not compile yet — that is expected; this task may be committed once `config_store.*` itself is self-consistent and the build is taken green together with C0.2/C0.3 in the coordinated Phase-0 commit). **Phase 0 lands as one green build.**

## Task C0.2: Result + persist types, retype the 4 consumers (coordinated)
**Files:** Modify `src/cv.h`, `src/overlay.h`, `src/mqttc.h`, `src/web_server.h`, and the readers `src/main.cpp`, `src/overlay.cpp`, `src/mqttc.cpp`, `src/web_server.cpp` enough to compile against the new type (full logic repoint is later tasks; here just retype + stub the field reads so it builds).
**Interfaces:** Produces `CellResult`/`CurbResult`/`CurbPersist` (Shared Contracts). All `CvResult*` params become `CurbResult*`.

- [ ] **Step 1:** In `cv.h` replace `SlotResult`/`CvResult`/`CvPersist`/`CV_PERSIST_MAGIC` with the Shared-Contracts `CellResult`/`CurbResult`/`CurbPersist`/`CURB_PERSIST_MAGIC`. Keep the `CvEngine` class declaration (its body is rewritten in Phase 2); update its method signatures to take/produce `CurbResult`.
- [ ] **Step 2:** Retype the 2nd param: `overlay.h` `overlayRenderJpeg(const Config&, const CurbResult&, ...)`; `mqttc.h` `mqttBegin(const Config*, const CurbResult*)`; `web_server.h` `webBegin(const Config*, const CurbResult*)`.
- [ ] **Step 3:** Update `main.cpp` `lastResult` to `CurbResult` and fix the obvious field reads to compile (temporary: headline fields exist; per-cell loop bodies that referenced `slots[i]` get minimally adjusted — full repoint in Phase 3).
- [ ] **Step 4:** Make `overlay.cpp`/`mqttc.cpp`/`web_server.cpp` COMPILE against the new struct (comment/stub the occupancy-field bodies that move to later tasks, leaving a clear `// repointed in Task Cx` marker — but no behavioural placeholder ships; these bodies are fully written in Phase 3). Prefer to fold the real Phase-3 repoint in here if small.
- [ ] **Step 5:** `pio run` → `[SUCCESS]`. Commit Phase 0 (C0.1+C0.2) as one green build:
```bash
git add src/config_store.h src/config_store.cpp src/cv.h src/overlay.h src/mqttc.h src/web_server.h src/main.cpp src/overlay.cpp src/mqttc.cpp src/web_server.cpp
git commit -F - <<'EOF'
feat(curb): schema + result/persist types for curb free-space (replaces bay model)

Replaces Roi/rois/MAX_ROIS with CurbStrip/CurbCell/strips/cells + curb tunables;
SlotResult/CvResult/CvPersist -> CellResult/CurbResult/CurbPersist (+ new magic);
retypes the four CvResult* consumers (overlay/mqtt/web/main) in one coordinated
build. CONFIG_VERSION bumped. Platform transport untouched.

giovi321
EOF
```

# Phase 1 — Strip tracer + host-side cell layout + persistence

## Task C1.1: Web UI strip tracer (replaces the bay polygon editor)
**Files:** Modify `web-src/index.html` (the ROI/bay editor `121-144,576-653`, `#slotTable` `145-149,507-515`, the live headline `100,118-120`); regenerate `src/web_ui.h`.
**Interfaces:** Produces the `strips`/`cells` JSON over `/api/config` (Seam H) and consumes `/api/state` `cells`/`strips` (Seam G). REUSE the `#ovl` SVG + viewBox sync + pointer drag; the generic `save()`/`collect()` engine; the platform cards untouched.

- [ ] **Step 1:** Replace the bay polygon editor with a **strip tracer**: per strip, click a **polyline** of points along the curb + a width handle + a `realLenM` input; a "generate cells" step calls the host-side layout (Task C1.2) to produce the cell quads, previewed over `#ovl`. Add/remove strip controls; per-cell **dead-zone toggle** (click a cell to disable — masks the bottom-right car / driveways); a per-strip **"mark all empty"** button (Seam I).
- [ ] **Step 2:** Replace `#hCount`/`#bigCount` "cars" headline with the curb headline (free_curb_m / can_fit / est_free_spaces / reliable_range_m), and `#slotTable` with a per-cell list (occupied/free/disabled/out-of-range) — both fed from `/api/state` (`st.curb.*`, `st.cells[]`).
- [ ] **Step 3:** Update `F[]`/`fill()`/`loadCfg` to read/write `cfg.strips`/`cfg.cells` (was `cfg.rois`) and the curb tunables; rebrand the Detection card bay→cell and add the curb tunable inputs (`carPitchM`, clearances, `cellLenM` target g, `smoothMode`, `darkLumaThresh`, `pitchLearn`); `saveDetect`/`saveOccEngine` key lists → curb keys. Add `#ovl` CSS classes `free-run`, `out-of-range`, `dead-zone`.
- [ ] **Step 4:** `pio run` (regenerates `web_ui.h`) → `[SUCCESS]`; confirm the `[gen_web_ui] wrote` line. Commit (both files).

## Task C1.2: Host-side cell layout (polyline → cell quads)
**Files:** Modify `web-src/index.html` (JS); regenerate.
**Interfaces:** Produces `cells[]` (normalized quads + `lenM` + `strip` + `enabled`) from a strip's traced polyline + width + `realLenM`. Pure JS, unit-testable.

- [ ] **Step 1: Write the failing JS test** (a small inline test harness or a `tools/test_cell_layout.mjs` run with `node`): given a straight polyline of known image length and `realLenM=24`, `g≈1.5`, it returns 16 cells whose `lenM≈1.5` and whose quads tile the band without gaps/overlap.
- [ ] **Step 2:** Implement `layoutCells(polyline, widthPx, realLenM, gTargetM)`: walk the polyline by arc-length; place cells of ≈`gTargetM` ground length, applying an **approximate monotonic near→far correction** (cells get longer in image-space toward the near/bottom end — derive the near/far scale from the polyline endpoints' vertical position as a fisheye-tolerant heuristic, or from an optional one-car reference mark); offset each cell across the width band to a quad; compute each cell's mapped pixel area and mark cells below a floor (`~6–8 px` along-strip) `enabled=false` + `inRange=false` (usable-range gate). Emit normalized `px[]/py[]`, `lenM`, `strip`.
- [ ] **Step 3:** Run the test → pass. `pio run` (regenerate) → `[SUCCESS]`. Commit.

## Task C1.3: Curb persistence (per-cell baselines)
**Files:** Modify `src/cv_state.cpp`/`.h` call sites in `src/main.cpp` (`78-94,290-298,346-349`) to use `CurbPersist`; `cv_state.*` mechanism is UNCHANGED (size+magic gate auto-drops the old blob).
**Interfaces:** Produces save/restore of `CurbPersist` via the existing `cvStateLoad/Save` plumbing.

- [ ] **Step 1:** Repoint the `CvPersist`→`CurbPersist` references in `main.cpp` (snapshot/restore around the cv engine state). The `cv_state` getBytes/putBytes mechanism is generic; the `len==sizeof` gate rejects the old 0xCB01 blob. Geometry signature mismatch (re-traced strips) drops stale baselines.
- [ ] **Step 2:** `pio run` → `[SUCCESS]`. Commit.

# Phase 2 — Per-cell detection (reuse the CV primitives)

## Task C2.1: Rewrite `CvEngine::analyze()` for strips/cells
**Files:** Modify `src/cv.cpp` (rewrite the per-bay iteration `176-184,247-251,310-317` and `recalibrate`/`markOccupied`/`reset`/`roiSignature`); REUSE the ray-cast accumulator (`185-244`), `featuresFinalize`, and the relative-edge/hysteresis/debounce decision primitives (`259-308`) verbatim per cell.
**Interfaces:** Consumes `Config.cells[]`/`strips[]`; produces `CellResult[]` (occupied/free + feat + baseline) into `CurbResult.cells[]`. Headline aggregation is Task C3.1.

- [ ] **Step 1:** Iterate `i in 0..cfg.cellCount`: for each `enabled` cell, run the existing ray-cast accumulator over the cell quad → `featuresFinalize` → `feat[16]`; decide occupied/free against the cell's own baseline (relative-edge day-one; `clfScore` when `occupancyEngine==1 && clfAvailable()`), with the existing enter/exit hysteresis + `stableFrames` debounce + frozen-while-occupied baseline EMA. Disabled/out-of-range cells: `inRange=false`, skipped.
- [ ] **Step 2:** Spatial smoothing: after per-cell raw labels, apply the width-3 median (per strip, `smoothMode==1`) over `inRange` cells before commit.
- [ ] **Step 3:** `recalibrate(index)`: `index<0` = mark ALL cells empty (re-seed baselines); `index>=0` = re-seed cell `index`. `markOccupied` → repurpose as a per-cell occupied override or drop (keep the signature). `roiSignature()` → `geomSig` over strip/cell geometry.
- [ ] **Step 4:** `pio run` → `[SUCCESS]`. Commit.

# Phase 3 — Free-space reducer, outputs, stability (the seam repoints)

## Task C3.1: Free-gap reducer → headline (`free_curb_m`, `can_fit`, `est_free_spaces`, …)
**Files:** Modify `src/cv.cpp` (aggregate after per-cell labels).
**Interfaces:** Produces the `CurbResult` headline scalars (Shared Contracts).

- [ ] **Step 1:** Per strip, run-length-encode FREE `inRange` cells into gaps; `G_m = Σ cell.lenM`. `n_interior = floor((G_m − clearInteriorM)/carPitchM)`; gaps touching a strip end use `clearEndM` (both ends are hard boundaries per the install); discard `G_m < carPitchM`; half-cell guard at run ends. Aggregate across strips: `free_curb_m`, `longest_free_run_m`, `est_free_spaces = Σ n`, `can_fit = longest_free_run_m >= carPitchM − clearInteriorM`, `reliable_range_m = Σ inRange cell.lenM`, `occupied_fraction`.
- [ ] **Step 2:** Light gate: if mean luma over in-range cells `< darkLumaThresh` → `dark=true`.
- [ ] **Step 3:** Final-integer stability: a small debounce/hysteresis on `est_free_spaces` so it does not flap by ±1 on a passing vehicle (a short EMA/vote with a hold).
- [ ] **Step 4:** `pio run` → `[SUCCESS]`. Commit.

## Task C3.2: Overlay draw loop (cells + free runs + headline) — Seam L
**Files:** Modify `src/overlay.cpp` (`106-146`); decode/blend/encode infra UNCHANGED.
- [ ] Render each cell coloured occupied/free/disabled/out-of-range, highlight FREE runs, draw the headline (`free_curb_m`, `can_fit`) + the reliable-range cutoff. `pio run` → `[SUCCESS]`. Commit.

## Task C3.3: Repoint `/api/state` + `/api/action` — Seams G, I
**Files:** Modify `src/web_server.cpp` (`118-145` handleState; `291-300` recalibrate/mark_occupied; externs `31-32`).
- [ ] `handleState`: replace `cv.count`/`slots[]` with the curb headline + `doc.cells[]`/`doc.strips[]`; keep `cv.valid/decW/decH/tookMs`. `handleAction` `recalibrate`: `slot<0`="mark all empty", `slot>=0`=per-cell seed (bound = cellCount); drop/repurpose `mark_occupied`. Keep the `{action,slot}` contract + dispatcher. `pio run` → `[SUCCESS]`. Commit.

## Task C3.4: Repoint webhook + spool + stats + trigger — Seams A, B, C, F, J
**Files:** Modify `src/net.h`/`net.cpp` (A: `155-236`, drop `buildSlotsJson`), `src/spool.h`/`spool.cpp` (B: `145-164,255-263,277`), `src/main.cpp` (C `228-229`; F `361`; J `shouldSend` `150-208`).
- [ ] **Step 1:** `netSendEvent`: replace `count`/`prevCount`/`slots[]`/`nSlots` params with the curb scalars; emit `appendField` rows `free_curb_m`/`can_fit`/`est_free_spaces`/`reliable_range_m`/`occupied_fraction`; filename off frame/uptime not count; drop `buildSlotsJson`; transport block UNCHANGED. (Alt allowed: one opaque pre-serialized JSON String param.)
- [ ] **Step 2:** `spoolEnqueue`/`spoolDrain`: curb scalar params + curb meta keys; drop the `slots[MAX_ROIS]` buffer + clamps; keep framing/caps/`m[key]|default`. **Add a one-time `spoolClear()` on the CONFIG_VERSION upgrade boot** so stale bay records don't replay as zero-value curb events.
- [ ] **Step 3:** `main.cpp`: stats `buildStatsJson` curb keys (drop `roi_count`/`count` → strip/cell counts + curb headline); MQTT trigger fires on a curb-headline change; `shouldSend()` predicate compares a curb metric (redefine `triggerThreshold` units, keep `minSendIntervalMs`/heartbeat). `test_webhook` (`web_server.cpp:263-276`) builds curb args.
- [ ] **Step 4:** `pio run` → `[SUCCESS]`. Commit.

## Task C3.5: Repoint MQTT entities — Seams D, E
**Files:** Modify `src/mqttc.cpp`/`.h` (retype `s_last`; FIELDS rows `51/58/70/77`; per-bay blocks `124-156,192-198,250-262`; `roiSig` `97-107`; externs `35-36`; `mark_all_free`). Camera/`cmd`/photo UNCHANGED.
- [ ] Drop `count`/`roi_count` FIELDS rows + per-bay binary_sensors/selects/`publishBayIdle` + `bay/+/set`; add curb FIELDS rows (`free_curb_m` m, `longest_free_run_m` m, `est_free_spaces`, `reliable_range_m` m, `occupied_fraction` %) so they ride `publishDiscovery/publishState`; publish `can_fit` + `dark` as `binary_sensor` via `publishCfg`; repoint `roiSig`→geomSig, `mark_all_free`→empty-strip seed; `clearCfg()` purges stale retained bay ids. `pio run` → `[SUCCESS]`. Commit.

# Phase 4 — Calibration + auto-learn

## Task C4.1: Auto-learn pitch refiner
**Files:** Modify `src/cv.cpp` (collect isolated-run lengths into `CurbPersist.carPitchLearned`), `src/config_store.*` (`pitchLearn` already added).
**Interfaces:** Pure function `learnPitch(runsMetres[], configured)` is host-testable — mirror it in `tools/` pytest if a Python port is desired; otherwise unit-reason in C.
- [ ] On each frame, collect lengths of ISOLATED occupied runs (free both sides, `< 1.5×` current pitch); maintain a small histogram in `CurbPersist`; take the lowest dense cluster + `clearInteriorM` → footprint→pitch; EMA toward the configured pitch **gated at `learnSamples >= 30`**; never bootstrap from nothing, never override an explicit pitch. Surface "configured vs observed" disagreement in `/api/state` + MQTT. `pio run` → `[SUCCESS]`. Commit.

## Task C4.2: Recalibrate-on-camera-move + optional "street full → N"
**Files:** Modify `web-src/index.html` (actions) + `src/web_server.cpp` (action branches) + `src/cv.cpp`.
- [ ] Add a "recalibrate (camera moved)" action (re-trace corners + re-seed baselines) and an optional "street full → enter N" (`carPitchM = stripLen/N`) on the existing `/api/action` dispatcher. Regenerate UI. `pio run` → `[SUCCESS]`. Commit.

# Phase 5 — Per-cell capture/training

## Task C5.1: Per-cell training capture
**Files:** Modify `src/capture.cpp`/`.h` (`10-37`): iterate per CELL, emit `cells` key (was `bays`), skip disabled cells; keep the `feat[16]` row shape; `netPostJson` transport + 30s throttle UNCHANGED. `tools/train_classifier.py` keeps `feat[16]` (only the per-row loop / `bays`→`cells` key changes — update if it hard-codes `bays`).
- [ ] `pio run` → `[SUCCESS]`; `cd tools && python -m pytest -q` → green. Commit.

---

## Self-review notes (author)

- **Spec coverage:** strip+cells+homography/polyline → C1.1/C1.2; per-cell detect → C2.1; free-gap reducer + outputs + stability + light gate → C3.1; calibration + auto-learn → C4.x; per-cell capture → C5.1; outputs/MQTT/webhook/stats/state/overlay repoints → C3.2–C3.5; persistence → C1.3; schema/types → C0.x.
- **Preserve guarantee:** every Global-Constraint seam (A–L) maps to a specific task that changes VALUES only; `clk`/`wg`/`clf`/`features`/`cv_state`/`gen_web_ui`/spool-mechanics/auth/OTA/backup-restore-transport/MQTT-discovery-framework are in the fileModMap as `unchanged`.
- **Risk handling:** lossy cross-schema restore + stale spool → CONFIG_VERSION bump + one-time `spoolClear` (C3.4) + documented re-trace; `MAX_ROIS` entanglement + the 4-signature type seam → done atomically in Phase 0 (C0.2); `roiCount` reads → repointed in C3.4/C3.5; trigger redefinition → C3.4; cv-in-place decision → recorded in Global Constraints; new-secret gap → none introduced (noted).
- **Type consistency:** the Shared-Contracts struct/field/constant names are used verbatim by every task; the curb output field names are fixed once and reused at all seams.
