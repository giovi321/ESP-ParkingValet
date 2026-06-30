# Task T1 — Curb Schema + Result/Persist Types (Phase 0 Keystone)

**Date:** 2026-06-30  
**Branch:** dev-ml  
**Status:** COMPLETE — `pio run` green on first successful pass after fix  

---

## What Was Done

Replaced the fixed-bay occupancy model (`Roi/rois/MAX_ROIS`, `SlotResult/CvResult/CvPersist`) with the curb model (`CurbStrip/CurbCell/strips/cells`, `CellResult/CurbResult/CurbPersist`) across the entire source tree. All platform transport (WiFi, WireGuard, MQTT, webhook, stats, spool, OTA, backup-restore, auth, NTP, snapshot) was preserved — only the occupancy data model and its consumer signatures changed.

---

## Files Changed (17 files, 478 ins / 429 del)

| File | Change |
|---|---|
| `src/config_store.h` | Removed `MAX_ROIS=12`, `MAX_POLY=8`, `struct Roi`, `roiCount`, `rois[MAX_ROIS]`. Added `MAX_STRIPS=4`, `MAX_CELLS=48`, `CONFIG_VERSION=2`, `struct CurbStrip`, `struct CurbCell`, `stripCount`, `cellCount`, curb tunables |
| `src/config_store.cpp` | `configLoadDefaults`: zero-init curb geometry + tunable defaults. `serializeFull`: strips/cells/tunables block. Removed `parseRois`. `parseFull`: strips/cells parsing (pre-seed-empty; only overwritten when JSON key present → old backups yield zero strips gracefully) |
| `src/cv.h` | Removed `SlotResult`, `CvResult`, `CvPersist`, `CV_PERSIST_MAGIC=0xCB01`. Added `CellResult`, `CurbResult`, `CurbPersist`, `CURB_PERSIST_MAGIC=0xCB02`. Updated `CvEngine` signature: `analyze(…CurbResult&)`, `snapshotState(CurbPersist&)`, `restoreState(const CurbPersist&)` |
| `src/cv.cpp` | `analyze()` rewired to `CurbResult&`; iterates `cfg->cells[i]` (4-vertex CurbCell), renamed color-channel locals `r5/g6/b5` + `cellRes` to avoid conflict with `CellResult& cr`. Private arrays changed from `[MAX_ROIS]` to `[MAX_CELLS]`. `snapshotState`/`restoreState` operate on `CurbPersist` with `CURB_PERSIST_MAGIC` |
| `src/cv_state.h` | Signatures: `cvStateLoad(CurbPersist&)`, `cvStateSave(const CurbPersist&)` |
| `src/cv_state.cpp` | `sizeof(CurbPersist)` check; `CURB_PERSIST_MAGIC` validation |
| `src/overlay.h` | Signature: `overlayRenderJpeg(const Config&, const CurbResult&, uint8_t**)` |
| `src/overlay.cpp` | Signature updated; bay draw loop stubbed with `TODO(T6-T10)` + `(void)cv;`; timestamp overlay, drawLine, drawText, pointInPoly infrastructure retained for Task C3.2 |
| `src/mqttc.h` | Signature: `mqttBegin(const Config*, const CurbResult*)` |
| `src/mqttc.cpp` | `s_last` typed `const CurbResult*`. `roiSig()` hashes `stripCount`/`strips[i].name`. `publishBayDiscovery/State/Idle` iterate `stripCount/MAX_STRIPS`. Strip membership lookup uses `s_cfg->cells[j].strip` (CurbCell), not CellResult. `MAX_ROIS→MAX_STRIPS` in bay message handler |
| `src/web_server.h` | Signature: `webBegin(Config*, CurbResult*)` |
| `src/web_server.cpp` | `g_last` typed `CurbResult*`. `handleState`: `est_free_spaces`, cell loop over `cellCount/MAX_CELLS` using `CurbCell`/`CellResult` fields. `handleAction test_webhook/recalibrate/mark_occupied`: `MAX_ROIS→MAX_CELLS`, `nCells`, `cells[i].occupied`, `est_free_spaces` |
| `src/capture.h` | Signature: `captureMaybeLog(const Config&, const CurbResult&)` |
| `src/capture.cpp` | `r.nCells`, `cfg.cells[i].enabled`, `r.cells[i]`, `MAX_CELLS`. TODO(T6-T10) for key rename in Task C5.1 |
| `src/main.cpp` | `lastResult: CurbResult`, `s_cvSaved: CurbPersist`. `cvStateDiffers` uses `geomSig/cellCount/MAX_CELLS`. `postEvent/maybeSend` use `r.nCells/r.cells[i].occupied/r.est_free_spaces`. `buildStatsJson` uses `cfg.cellCount`/`lastResult.est_free_spaces`. Loop: `CurbResult r`, trigger on `r.est_free_spaces != lastMqttCount` |
| `src/spool.cpp` | 3× `MAX_ROIS→MAX_CELLS` (lines 161, 262, 263) |
| `src/features.h` | Comment updated: `SlotResult.baselineEdge → CellResult.baselineEdge` |

---

## Shared Contracts Delivered (verbatim)

```cpp
// config_store.h
static const int MAX_STRIPS = 4;
static const int MAX_CELLS  = 48;
static const int CONFIG_VERSION = 2;
struct CurbStrip { char name[16]; float realLenM; float realWidthM; int nCells; };
struct CurbCell  { float px[4], py[4]; float lenM; uint8_t strip; bool enabled; };

// cv.h
struct CellResult  { float feat[16]; float clfScore; float edge, meanI, baselineEdge; bool occupied; bool rawOccupied; bool inRange; };
struct CurbResult  { bool valid; int decW, decH; uint32_t tookMs; int nCells; CellResult cells[MAX_CELLS]; float free_curb_m; float longest_free_run_m; bool can_fit; int est_free_spaces; float reliable_range_m; float occupied_fraction; bool dark; };
static const uint16_t CURB_PERSIST_MAGIC = 0xCB02;
struct CurbPersist { uint16_t magic; uint16_t cellCount; uint32_t geomSig; float baselineEdge[MAX_CELLS]; uint8_t baselineInit[MAX_CELLS]; uint8_t committed[MAX_CELLS]; float carPitchLearned; uint16_t learnSamples; };
```

---

## Build Result

```
RAM:   [==        ]  24.9% (used 81584 bytes from 327680 bytes)
Flash: [=======   ]  65.9% (used 1295285 bytes from 1966080 bytes)
========================= [SUCCESS] Took 16.15 seconds =========================
```

---

## Old-Symbol Grep Result

```
grep -r 'MAX_ROIS|CvResult|SlotResult|CvPersist|roiCount|rois\[' src/
→ 0 matches in 0 files
```

All old symbols fully eliminated from the source tree.

---

## Bug Found and Fixed During Build

`mqttc.cpp` `publishBayState()` referenced `s_last->cells[j].strip`, but `strip` is a field of `CurbCell` (config geometry), not `CellResult` (analysis output). Fixed to `s_cfg->cells[j].strip`. This is correct: the cell→strip mapping is static config, while `CellResult` carries only per-frame analysis outputs.

---

## Deferred Items (TODO markers in code)

All bodies that depend on full curb field semantics are marked `// TODO(T6-T10)`:

| Location | Deferred to |
|---|---|
| `overlay.cpp`: cell/strip draw loop | Task C3.2 |
| `mqttc.cpp`: per-cell/strip state publishing | Task C3.5 |
| `web_server.cpp`: slot loop field semantics | Task C3.x |
| `capture.cpp`: rename "bays" key → "cells" | Task C5.1 |
| `main.cpp`: triggerThreshold units | Task C3.4 |
| `main.cpp`: `maybeSend` slots payload → curb fields | Task C3.4 |
| `buildStatsJson`: rename/repoint stats keys | Task C3.4 |
| `main.cpp`: MQTT trigger on curb headline change | Task C3.4 |

---

## Config Version Bump

`CONFIG_VERSION` bumped from 1 to 2. Old backups without `strips`/`cells` keys yield `stripCount=0 / cellCount=0` gracefully (pre-seed-empty + overwrite-only-when-present pattern in `parseFull`). Old NVS baselines are rejected at restore time because `CURB_PERSIST_MAGIC` changed from `0xCB01` to `0xCB02`.

---

## Key Constraints Preserved

- WiFi / WireGuard / MQTT / webhook / stats / spool / OTA / backup-restore / auth / NTP / snapshot: zero changes
- Secret masking and all non-occupancy `Config` fields: intact
- No hardware flashed
