#!/usr/bin/env python3
"""Train the per-cell occupancy classifier from device capture records and emit src/clf_model.h.

Input: a JSONL file; each line is one capture batch as the device POSTs it:
  {"device":..,"ts":..,"cells":[{"i":0,"label":0,"score":-1,"f":[<16 floats>], "y":1(optional)}]}
The training label is the per-cell "y" (hand-corrected) if present, else "label" (the device's
weak edge-engine decision). Trains an L2 logistic regression (standardized, class-balanced),
folds the scaler into raw-feature weights, and writes a clf_model.h whose clfPredict() consumes
the raw 16-float feature vector the firmware produces.
"""
import argparse, json, math, sys
import numpy as np

NFEAT = 16

def load_records(path, with_meta=False):
    """Parse a capture JSONL file into feature matrix X and label vector y.

    Tolerant by design: a single malformed line, non-dict record, or bad cell is
    skipped with a warning instead of aborting the run. A cell is skipped unless
    its 'f' is a 16-long list of finite floats and its label coerces to 0/1.

    with_meta=False (default) returns (X, y) for backward compatibility.
    with_meta=True also returns (groups, verified): 'groups' is a per-sample batch
    key (the record's ts, else its line index) for leakage-aware splitting, and
    'verified' flags samples whose label came from an explicit "y" override.
    """
    X, y, groups, verified = [], [], [], []
    n_skipped = 0
    with open(path) as f:
        for lineno, line in enumerate(f):
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError as e:
                print("warning: skipping malformed JSON line: %s" % e, file=sys.stderr)
                n_skipped += 1
                continue
            if not isinstance(obj, dict):
                print("warning: skipping bad record (not a JSON object) at line %d" % (lineno + 1),
                      file=sys.stderr)
                n_skipped += 1
                continue
            cells = obj.get("cells", [])
            if not isinstance(cells, list):
                print("warning: skipping bad record ('cells' not a list) at line %d" % (lineno + 1),
                      file=sys.stderr)
                n_skipped += 1
                continue
            group = obj.get("ts", lineno)
            for cell in cells:
                if not isinstance(cell, dict):
                    print("warning: skipping bad cell (not a JSON object) at line %d" % (lineno + 1),
                          file=sys.stderr)
                    n_skipped += 1
                    continue
                feat = cell.get("f")
                if not isinstance(feat, list) or len(feat) != NFEAT:
                    continue
                has_y = "y" in cell
                label = cell.get("y", cell.get("label"))
                if label is None:
                    continue
                try:
                    fv = [float(v) for v in feat]
                except (TypeError, ValueError):
                    print("warning: skipping bad cell (non-numeric feature) at line %d" % (lineno + 1),
                          file=sys.stderr)
                    n_skipped += 1
                    continue
                if not all(math.isfinite(v) for v in fv):
                    print("warning: skipping bad cell (non-finite feature) at line %d" % (lineno + 1),
                          file=sys.stderr)
                    n_skipped += 1
                    continue
                try:
                    lab = int(label)
                except (TypeError, ValueError):
                    print("warning: skipping bad cell (non-numeric label) at line %d" % (lineno + 1),
                          file=sys.stderr)
                    n_skipped += 1
                    continue
                if lab not in (0, 1):
                    print("warning: skipping bad cell (label not 0/1) at line %d" % (lineno + 1),
                          file=sys.stderr)
                    n_skipped += 1
                    continue
                X.append(fv)
                y.append(lab)
                groups.append(group)
                verified.append(has_y)
    if n_skipped:
        print("warning: skipped %d bad cell/record item(s)" % n_skipped, file=sys.stderr)
    Xa = np.asarray(X, dtype=float)
    ya = np.asarray(y, dtype=int)
    if with_meta:
        return Xa, ya, np.asarray(groups), np.asarray(verified, dtype=bool)
    return Xa, ya

def _fit_folded(feats, labels):
    """Fit StandardScaler+LogisticRegression; return raw-space (W[16], B)."""
    from sklearn.preprocessing import StandardScaler
    from sklearn.linear_model import LogisticRegression
    scaler = StandardScaler().fit(feats)
    clf = LogisticRegression(C=1.0, class_weight="balanced", max_iter=1000)
    clf.fit(scaler.transform(feats), labels)
    w = clf.coef_[0]; b = float(clf.intercept_[0])
    mean = scaler.mean_; std = scaler.scale_
    std = np.where(std == 0, 1.0, std)   # guard constant features
    # z = sum(w_i (x_i-mean_i)/std_i) + b = sum((w_i/std_i) x_i) + (b - sum(w_i mean_i/std_i))
    W = w / std
    B = b - float(np.sum(w * mean / std))
    return W, B

def emit_model_header(feats, labels, model_kind="logreg", out_path="../src/clf_model.h"):
    if model_kind != "logreg":
        raise ValueError("only 'logreg' export is supported")
    W, B = _fit_folded(feats, labels)
    wlist = ", ".join("%.8ef" % v for v in W)
    lines = [
        "#pragma once",
        "// Generated by tools/train_classifier.py — do not edit by hand.",
        "#include <math.h>",
        "#ifndef CLF_NFEAT",
        "#define CLF_NFEAT 16",
        "#endif",
        "#define CLF_MODEL_PRESENT 1",
        "// Occupied probability in [0,1] for a CLF_NFEAT-long raw feature vector.",
        "static inline float clfPredict(const float* feat) {",
        "  static const float W[16] = { %s };" % wlist,
        "  float z = %.8ef;" % B,
        "  for (int i = 0; i < 16; i++) z += W[i] * feat[i];",
        "  return 1.0f / (1.0f + expf(-z));",
        "}",
    ]
    with open(out_path, "w") as f:
        f.write("\n".join(lines) + "\n")
    return W, B

def _split_indices(labels, groups):
    """Return (train_idx, test_idx). Prefer a group-wise split keyed on the
    record ts/line so near-duplicate frames from one ~30 s batch cannot land in
    both train and test (temporal leakage). Fall back to a stratified row split
    when grouping is unavailable or would leave a split single-class."""
    from sklearn.model_selection import train_test_split, GroupShuffleSplit
    idx = np.arange(len(labels))
    if groups is not None and len(groups) == len(labels) and len(np.unique(groups)) >= 2:
        gss = GroupShuffleSplit(n_splits=1, test_size=0.25, random_state=0)
        tr, te = next(gss.split(idx, labels, groups))
        if len(set(labels[tr].tolist())) >= 2 and len(set(labels[te].tolist())) >= 2:
            return idx[tr], idx[te]
    return train_test_split(idx, test_size=0.25, stratify=labels, random_state=0)


def _report(feats, labels, groups=None, verified=None):
    from sklearn.preprocessing import StandardScaler
    from sklearn.linear_model import LogisticRegression
    from sklearn.metrics import classification_report, confusion_matrix
    # Need two classes AND >=2 members in the least-populated class, else the
    # stratified/held-out split is impossible. Skip the report (never raise) so
    # model emission downstream is never blocked.
    counts = np.bincount(labels, minlength=2)
    if int((counts > 0).sum()) < 2 or int(counts[counts > 0].min()) < 2:
        print("[train] need >=2 samples in each of two classes to evaluate; "
              "skipping held-out report (model still trained on all data).")
        return
    try:
        tr, te = _split_indices(labels, groups)
        Xtr, Xte, ytr, yte = feats[tr], feats[te], labels[tr], labels[te]
        sc = StandardScaler().fit(Xtr)
        clf = LogisticRegression(C=1.0, class_weight="balanced", max_iter=1000).fit(sc.transform(Xtr), ytr)
        yp = clf.predict(sc.transform(Xte))
        grouped = groups is not None and len(groups) == len(labels) and len(np.unique(groups)) >= 2
        print("[train] held-out per-class report (%s):" %
              ("group-wise split, limits temporal leakage" if grouped else "stratified row split"))
        print(classification_report(yte, yp, target_names=["empty", "occupied"],
                                    labels=[0, 1], zero_division=0))
        print("[train] confusion matrix [rows=true, cols=pred]:")
        print(confusion_matrix(yte, yp, labels=[0, 1]))
        # Hand-verified (y-override) subset: these are the only truly trustworthy
        # labels, so report them separately — top-line numbers otherwise just
        # measure agreement with the device's own weak edge-engine labels.
        if verified is not None and len(verified) == len(labels):
            vmask = verified[te]
            if bool(vmask.any()):
                print("[train] hand-verified (y-override) held-out subset — %d samples:" % int(vmask.sum()))
                print(classification_report(yte[vmask], yp[vmask], target_names=["empty", "occupied"],
                                            labels=[0, 1], zero_division=0))
                print("[train] hand-verified confusion matrix [rows=true, cols=pred]:")
                print(confusion_matrix(yte[vmask], yp[vmask], labels=[0, 1]))
            else:
                print("[train] no hand-verified (y-override) samples fell in the held-out split.")
    except Exception as e:
        print("[train] evaluation skipped (%s); model still emitted." % e, file=sys.stderr)

def main(argv=None):
    ap = argparse.ArgumentParser(description="Train occupancy classifier -> clf_model.h")
    ap.add_argument("--in", dest="inp", required=True, help="capture records (.jsonl)")
    ap.add_argument("--out", dest="out", default="../src/clf_model.h", help="output header path")
    args = ap.parse_args(argv)
    feats, labels, groups, verified = load_records(args.inp, with_meta=True)
    if len(feats) == 0:
        print("no usable records in %s" % args.inp, file=sys.stderr); return 1
    print("[train] %d samples, %d occupied / %d empty (%d hand-verified)" %
          (len(labels), int((labels == 1).sum()), int((labels == 0).sum()), int(verified.sum())))
    _report(feats, labels, groups, verified)
    W, B = emit_model_header(feats, labels, "logreg", args.out)
    print("[train] wrote %s (logreg, %d features)" % (args.out, len(W)))
    return 0

if __name__ == "__main__":
    sys.exit(main())
