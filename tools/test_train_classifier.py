import os, sys, json
import numpy as np
sys.path.insert(0, os.path.dirname(__file__))
from train_classifier import load_records, emit_model_header

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
