# Training the on-device occupancy classifier

The device can learn a weather-robust occupancy model from its OWN camera, then run it
on-device. Everything below can be driven remotely once WireGuard is set up
(see "Remote access (WireGuard)" in the README). The firmware is the easy part — the real
cost is collecting weather-diverse data over time.

## 1. Turn on training capture

In the web UI → **Occupancy engine & training** card:
- **Capture URL** — an endpoint that stores each POST as one JSON line (e.g. an n8n webhook
  that appends to a `.jsonl` file). Set the auth header if your endpoint needs one.
- Tick **Training capture** and Save.

The device now POSTs a batch every ~30 s: per bay, the 16-feature vector (`f`), the device's
current decision as a **weak label** (`label`), and the live classifier score (`score`).

## 2. Collect across conditions

Leave it running across **day, night, wet, snow, shadow**. This is mostly waiting — the model
can only learn conditions it has actually seen. The device also sends snapshot photos on
count changes, which you can review while labelling.

## 3. Build the dataset

Concatenate the POSTed batches into one file, `records.jsonl` (one JSON batch per line).

## 4. Correct the labels (only the wrong ones)

Each bay record's `label` is the device's edge-engine guess — correct most of the time. For
the frames it got wrong (the weather-hard ones), add a `"y"` field (0 = empty, 1 = occupied)
to that bay with the true label. The trainer uses `y` when present, else `label`. You only fix
the mistakes, not label from scratch.

## 5. Train

```
cd tools
python -m pip install -r requirements.txt
python train_classifier.py --in /path/to/records.jsonl --out ../src/clf_model.h
```

It prints a held-out **per-class precision/recall + confusion matrix** — judge by those on the
hard cases, NOT top-line accuracy (a mostly-empty lot scores high by guessing "empty"). It
writes `src/clf_model.h` with `CLF_MODEL_PRESENT 1`.

## 6. Flash

```
pio run
```

Then OTA-upload `.pio/build/esp32cam/firmware.bin` via the web UI (**Firmware update**).

## 7. Switch the engine on

In the UI → **Occupancy engine & training** → **Engine = Trained classifier** → Save. The edge
engine stays available as an instant fallback (switch back any time, or it auto-falls-back if
no model is embedded).

## 8. Iterate

As you capture more hard weather, append to `records.jsonl`, correct the new mistakes, retrain,
re-flash. The model improves as the dataset grows.

## Notes

- The current trainer exports a logistic regression (small, exact, microseconds on-device).
  If it underfits the hardest conditions, the design's "Phase 2" (a tiny CNN, ideally on an
  ESP32-S3) is the documented next step — see the design spec.
- Parity is automatic: the model trains on the EXACT feature vectors the device logs, so there
  is no train/serve mismatch to worry about.
