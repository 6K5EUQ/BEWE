"""AMC trainer. Reuses IQResNet1D from the AIS stack unchanged.

Data is generated on the fly rather than dumped to disk: a fresh burst every
step is strictly better than replaying a fixed set, and synthesis is cheaper
than the backward pass. That also means "epoch" here is just a step budget.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time

import numpy as np
import torch
import torch.nn as nn

sys.path.insert(0, os.path.expanduser("~/BEWE/BEAE/ais"))
from ais_ai.model import IQResNet1D  # noqa: E402  (shared architecture)

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from amc_ai.synth import CLASSES, make_batch  # noqa: E402

DATA = os.path.expanduser("~/BEWE/BEAE/amc/data")
MODEL_DIR = os.path.join(DATA, "ai_model")


def _gen(nsamp, per_class, snr, seed):
    X, y, _ = make_batch(per_class, nsamp, snr_range=snr, seed=seed)
    return torch.from_numpy(X), torch.from_numpy(y)


def evaluate(model, dev, nsamp, per_class, snr_points, seed=9999):
    """Per-SNR accuracy + confusion at the lowest and highest SNR."""
    model.eval()
    out = {}
    conf = {}
    with torch.no_grad():
        for s in snr_points:
            X, y = _gen(nsamp, per_class, (s, s), seed + int(s))
            p = model(X.to(dev)).argmax(1).cpu()
            out[s] = float((p == y).float().mean())
            m = np.zeros((len(CLASSES), len(CLASSES)), dtype=int)
            for t, q in zip(y.tolist(), p.tolist()):
                m[t][q] += 1
            conf[s] = m
    model.train()
    return out, conf


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--nsamp", type=int, default=1024)
    ap.add_argument("--steps", type=int, default=1500)
    ap.add_argument("--per-class", type=int, default=16)
    ap.add_argument("--snr-min", type=float, default=-2.0)
    ap.add_argument("--snr-max", type=float, default=22.0)
    ap.add_argument("--lr", type=float, default=2e-3)
    ap.add_argument("--out", default="")
    a = ap.parse_args()

    dev = "cuda" if torch.cuda.is_available() else "cpu"
    torch.manual_seed(0)
    model = IQResNet1D(n_classes=len(CLASSES)).to(dev)
    opt = torch.optim.AdamW(model.parameters(), lr=a.lr, weight_decay=1e-4)
    sched = torch.optim.lr_scheduler.OneCycleLR(opt, max_lr=a.lr, total_steps=a.steps)
    lossf = nn.CrossEntropyLoss(label_smoothing=0.05)

    print(f"dev={dev} classes={len(CLASSES)} nsamp={a.nsamp} steps={a.steps} "
          f"batch={a.per_class * len(CLASSES)}")
    t0 = time.time()
    run = 0.0
    for step in range(1, a.steps + 1):
        X, y = _gen(a.nsamp, a.per_class, (a.snr_min, a.snr_max), seed=step)
        X, y = X.to(dev, non_blocking=True), y.to(dev, non_blocking=True)
        opt.zero_grad(set_to_none=True)
        loss = lossf(model(X), y)
        loss.backward()
        nn.utils.clip_grad_norm_(model.parameters(), 5.0)
        opt.step()
        sched.step()
        run += loss.item()
        if step % 100 == 0:
            print(f"  step {step:5d}/{a.steps}  loss {run / 100:.4f}  "
                  f"{time.time() - t0:6.1f}s")
            run = 0.0

    accs, conf = evaluate(model, dev, a.nsamp, 40, [0, 5, 10, 15, 20])
    print("\n=== accuracy vs SNR ===")
    for s, v in accs.items():
        print(f"  {s:5.1f} dB : {v * 100:5.1f}%")

    os.makedirs(MODEL_DIR, exist_ok=True)
    ver = 1
    while os.path.exists(os.path.join(MODEL_DIR, f"model_v{ver:04d}.pt")):
        ver += 1
    path = a.out or os.path.join(MODEL_DIR, f"model_v{ver:04d}.pt")
    torch.save({"state": model.state_dict(), "classes": CLASSES,
                "nsamp": a.nsamp, "acc": accs}, path)
    meta = {"version": ver, "model": os.path.basename(path), "classes": CLASSES,
            "nsamp": a.nsamp, "acc_by_snr": {str(k): v for k, v in accs.items()}}
    with open(os.path.join(MODEL_DIR, "current.json"), "w") as f:
        json.dump(meta, f, indent=2)
    np.save(os.path.join(MODEL_DIR, f"confusion_v{ver:04d}.npy"),
            np.stack([conf[s] for s in sorted(conf)]))
    print(f"\nsaved {path}  (v{ver})")


if __name__ == "__main__":
    main()
