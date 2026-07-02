import os, sys, json
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from train_classifier import load_records, emit_model_header, _report

def _sigmoid(z): return 1.0 / (1.0 + np.exp(-z))

def test_emit_model_header_has_contract(tmp_path):
    feats = np.zeros((4, 16)); labels = np.array([0, 0, 1, 1])
    feats[2:, 0] = 10.0   # separable on feat[0]
    out = tmp_path / "clf_model.h"
    emit_model_header(feats, labels, "logreg", str(out))
    txt = out.read_text()
    assert "#define CLF_NFEAT 16" in txt
    assert "#define CLF_MODEL_PRESENT 1" in txt
    assert "float clfPredict(const float* feat)" in txt

def test_emitted_weights_separate_classes(tmp_path):
    rng = np.random.RandomState(0)
    empty = rng.normal(0.0, 0.1, size=(50, 16))
    occ   = rng.normal(0.0, 0.1, size=(50, 16)); occ[:, 0] += 5.0   # feat[0] high => occupied
    feats = np.vstack([empty, occ]); labels = np.array([0]*50 + [1]*50)
    W, B = emit_model_header(feats, labels, "logreg", str(tmp_path / "clf_model.h"))
    pe = _sigmoid(empty @ W + B).mean()
    po = _sigmoid(occ   @ W + B).mean()
    assert pe < 0.5 < po

def test_load_records_y_override(tmp_path):
    rec = {"device":"x","ts":1,"cells":[
        {"i":0,"label":0,"f":[0.0]*16},
        {"i":1,"label":0,"y":1,"f":[1.0]*16}]}
    p = tmp_path / "r.jsonl"; p.write_text(json.dumps(rec) + "\n")
    X, y = load_records(str(p))
    assert X.shape == (2, 16)
    assert list(y) == [0, 1]   # second row uses the y=1 override, not label=0

def test_load_records_tolerates_bad_cells(tmp_path):
    lines = [
        json.dumps({"device":"x","ts":1,"cells":[
            {"i":0,"label":0,"f":[0.0]*16},              # good
            {"i":1,"label":1,"f":[1.0]*15 + [None]},     # JS-null feature (NaN) -> skip
        ]}),
        json.dumps({"device":"x","ts":2,"cells":[
            {"i":0,"label":1,"f":[2.0]*16},              # good
            {"i":1,"label":0,"f":[3.0]*8},               # wrong length -> skip
        ]}),
        "this is not json",                              # malformed line -> skip
        json.dumps([1, 2, 3]),                           # top-level array -> skip
        '{"device":"x","ts":3,"cells":[{"i":0,"label":1,"f":[' +
            ",".join(["NaN"] + ["0.0"]*15) + ']}]}',     # bare NaN token -> skip
        json.dumps({"device":"x","ts":4,"cells":[
            {"i":0,"label":"occupied","f":[4.0]*16},     # non-numeric label -> skip
            {"i":1,"label":0,"f":[5.0]*16},              # good
        ]}),
    ]
    p = tmp_path / "r.jsonl"; p.write_text("\n".join(lines) + "\n")
    X, y = load_records(str(p))
    assert X.shape == (3, 16)          # only the 3 well-formed cells survive
    assert list(y) == [0, 1, 0]
    # a file with bad cells still trains end-to-end
    out = tmp_path / "clf_model.h"
    emit_model_header(X, y, "logreg", str(out))
    assert "#define CLF_MODEL_PRESENT 1" in out.read_text()

def test_load_records_meta_flags_verified(tmp_path):
    rec = {"device":"x","ts":7,"cells":[
        {"i":0,"label":0,"f":[0.0]*16},                  # weak label, not verified
        {"i":1,"label":0,"y":1,"f":[1.0]*16}]}           # y-override -> verified
    p = tmp_path / "r.jsonl"; p.write_text(json.dumps(rec) + "\n")
    X, y, groups, verified = load_records(str(p), with_meta=True)
    assert X.shape == (2, 16)
    assert list(verified) == [False, True]
    assert list(groups) == [7, 7]      # both cells share the batch ts as group key

def test_report_single_member_class_does_not_raise(tmp_path):
    # Least-populated class has 1 member -> stratified split is impossible.
    # _report must skip cleanly, and emit_model_header must still write a header.
    feats = np.zeros((5, 16)); labels = np.array([0, 0, 0, 0, 1])
    feats[4, 0] = 10.0
    _report(feats, labels)             # must not raise
    out = tmp_path / "clf_model.h"
    emit_model_header(feats, labels, "logreg", str(out))
    assert "#define CLF_MODEL_PRESENT 1" in out.read_text()
