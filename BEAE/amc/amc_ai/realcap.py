"""Domain-gap validator: does a synth-only AMC model survive real captures?

The only labelled real IQ we have is the AIS stack's aicap_*.bin, and AIS is
GMSK by definition (9600 baud, BT=0.4, h=0.5). So every real burst here has a
known ground-truth class = GMSK. If a model trained purely on synth.py collapses
on air, it shows up as GMSK bursts predicted as anything else.

Length is a confound: aicap bursts are a fixed 896 samples, the trainer uses
2048. To separate the *domain* gap (synth vs real) from a *length* gap we hold
length constant — synth GMSK is also evaluated at 896 — and additionally report
the training length (2048) and a length-matched resample of the real bursts.

Run: python -m amc_ai.realcap [--n 2000]
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import torch

sys.path.insert(0, os.path.expanduser("~/BEWE/BEAE/ais"))
from ais_ai.aicap import day_files, iter_file  # noqa: E402
from ais_ai.model import IQResNet1D            # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from amc_ai.synth import CLASSES, make_clean, impair, to_tensor_layout  # noqa: E402

AIS_DATA = os.path.expanduser("~/BEWE/BEAE/ais/data")
def _current_model():
    d = os.path.expanduser("~/BEWE/BEAE/amc/data/ai_model")
    cur = os.path.join(d, "current.json")
    if os.path.exists(cur):
        import json
        return os.path.join(d, json.load(open(cur))["model"])
    return os.path.join(d, "model_v0003.pt")


MODEL = _current_model()
GMSK = CLASSES.index("GMSK")


def load_model(dev):
    ck = torch.load(MODEL, map_location=dev)
    assert ck["classes"] == CLASSES, "class list drift — checkpoint incompatible"
    m = IQResNet1D(n_classes=len(CLASSES)).to(dev)
    m.load_state_dict(ck["state"])
    m.eval()
    return m


def _resample(iq: np.ndarray, m: int) -> np.ndarray:
    """FFT resample complex64[n] -> [m] (same method as ais dataset)."""
    n = len(iq)
    if m == n:
        return iq
    spec = np.fft.fft(iq)
    if m > n:
        h = n // 2
        out = np.zeros(m, dtype=complex)
        out[:h] = spec[:h]
        out[m - (n - h):] = spec[h:]
    else:
        h = m // 2
        out = np.concatenate([spec[:h], spec[n - (m - h):]])
    return (np.fft.ifft(out) * (m / n)).astype(np.complex64)


def load_real(n_max: int, resample_to: int | None):
    """Real AIS bursts -> float32 [k, 2, L]. All are GMSK ground truth."""
    X = []
    for path in day_files(AIS_DATA, days=3):
        for rec in iter_file(path):
            iq = rec.iq
            if resample_to and len(iq) != resample_to:
                iq = _resample(iq, resample_to)
            X.append(to_tensor_layout(iq))
            if len(X) >= n_max:
                return np.stack(X)
    return np.stack(X) if X else np.zeros((0, 2, 1), np.float32)


def synth_gmsk(k: int, nsamp: int, seed: int) -> np.ndarray:
    rng = np.random.default_rng(seed)
    X = []
    for _ in range(k):
        snr = rng.uniform(10, 20)
        X.append(to_tensor_layout(impair(make_clean("GMSK", nsamp, rng), snr, rng)))
    return np.stack(X)


@torch.no_grad()
def predict(model, dev, X, bs=256):
    preds, confs = [], []
    for i in range(0, len(X), bs):
        xb = torch.from_numpy(X[i:i + bs]).to(dev)
        logits = model(xb)
        p = torch.softmax(logits, 1)
        c, idx = p.max(1)
        preds.append(idx.cpu().numpy())
        confs.append(c.cpu().numpy())
    return np.concatenate(preds), np.concatenate(confs)


def report(name, preds, confs):
    k = len(preds)
    recall = float((preds == GMSK).mean())
    conf_gmsk = float(confs[preds == GMSK].mean()) if (preds == GMSK).any() else 0.0
    hist = np.bincount(preds, minlength=len(CLASSES))
    top = sorted(range(len(CLASSES)), key=lambda i: -hist[i])[:4]
    dist = "  ".join(f"{CLASSES[i]}:{hist[i]}" for i in top if hist[i])
    print(f"\n[{name}]  k={k}")
    print(f"  GMSK recall = {recall*100:5.1f}%   mean conf(GMSK) = {conf_gmsk:.3f}")
    print(f"  top preds: {dist}")
    return recall


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=2000)
    a = ap.parse_args()
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    model = load_model(dev)
    print(f"dev={dev}  model={os.path.basename(MODEL)}  classes={len(CLASSES)}")

    conds = [
        ("synth GMSK @2048 (train len, sanity)", synth_gmsk(a.n, 2048, 1)),
        ("synth GMSK @896  (length control)",    synth_gmsk(a.n, 896, 2)),
        ("REAL AIS  @896  (DOMAIN GAP)",         load_real(a.n, None)),
        ("REAL AIS  @2048 (resampled to train)", load_real(a.n, 2048)),
    ]
    for name, X in conds:
        if len(X) == 0:
            print(f"\n[{name}] no data")
            continue
        preds, confs = predict(model, dev, X)
        report(name, preds, confs)


if __name__ == "__main__":
    main()
