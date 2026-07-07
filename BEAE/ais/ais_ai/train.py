"""Full retrain from aicap window (manual: python -m ais_ai train --days N).

Always trains from scratch (no incremental fine-tune -> no forgetting), then
calibrates open-set thresholds and atomically publishes model_vNNNN + meta +
current.json for the daemon to hot-swap.
"""
import json
import os
import time

import numpy as np
import torch

from .config import Config
from .dataset import load_bursts, time_split
from .model import IQResNet1D
from .openset import calibrate_thresholds, fit_temperature


def _ece(conf: np.ndarray, correct: np.ndarray, n_bins: int = 10) -> float:
    """Expected Calibration Error: |confidence - accuracy| averaged over
    equal-width confidence bins, weighted by bin size. 0 = perfectly
    calibrated (a 0.99-confidence prediction is right 99% of the time)."""
    edges = np.linspace(0.0, 1.0, n_bins + 1)
    err = 0.0
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (conf > lo) & (conf <= hi) if lo > 0 else (conf >= lo) & (conf <= hi)
        if not m.any():
            continue
        err += (m.mean()) * abs(conf[m].mean() - correct[m].mean())
    return float(err)


def _augment(x: torch.Tensor, cfg: Config, gen: torch.Generator) -> torch.Tensor:
    """Batched GPU augmentation. x [B,2,N] RMS-normalized.
    Phase rotation (carrier phase is random per burst — safe & essential),
    AWGN, small time shift, amplitude scale. NEVER frequency shift / resample
    (CFO and clock ppm are genuine device fingerprints)."""
    B, _, N = x.shape
    dev = x.device
    i, q = x[:, 0], x[:, 1]
    th = torch.rand(B, 1, device=dev, generator=gen) * (2 * torch.pi)
    c, s = torch.cos(th), torch.sin(th)
    i, q = i * c - q * s, i * s + q * c                       # phase rotation
    snr_db = torch.rand(B, 1, device=dev, generator=gen) * cfg.aug_snr_db
    sigma = (10.0 ** (-snr_db / 20.0)) * 0.7071               # vs unit-RMS signal
    i = i + torch.randn(B, N, device=dev, generator=gen) * sigma
    q = q + torch.randn(B, N, device=dev, generator=gen) * sigma
    x = torch.stack([i, q], dim=1)
    if cfg.aug_shift > 0:                                     # integer roll per sample
        shifts = torch.randint(-cfg.aug_shift, cfg.aug_shift + 1, (B,), device=dev, generator=gen)
        idx = (torch.arange(N, device=dev).unsqueeze(0) - shifts.unsqueeze(1)) % N
        x = torch.gather(x, 2, idx.unsqueeze(1).expand(B, 2, N))
    amp = cfg.aug_amp_lo + torch.rand(B, 1, 1, device=dev, generator=gen) * (cfg.aug_amp_hi - cfg.aug_amp_lo)
    return x * amp


@torch.no_grad()
def _logits_all(model, x: torch.Tensor, batch: int, device) -> torch.Tensor:
    model.eval()
    outs = []
    for i in range(0, len(x), batch):
        outs.append(model(x[i:i + batch].to(device, non_blocking=True)).float().cpu())
    return torch.cat(outs) if outs else torch.zeros(0)


def train(cfg: Config, days: int, crop: str = "preamble", log=print):
    log(f"loading aicap window: {days} days, crop={crop}")
    bursts = load_bursts(cfg, days, crop)
    model, meta, _ = _fit(cfg, bursts, crop, log)
    return _publish(cfg, model, meta, days, log)


def _fit(cfg: Config, bursts, crop: str, log=print):
    """Core: split -> train -> calibrate. Returns (model, meta_dict, (va_x, va_y))."""
    t0 = time.time()
    device = "cuda" if torch.cuda.is_available() else "cpu"
    torch.manual_seed(cfg.seed)
    np.random.seed(cfg.seed)

    classes, tr_x, tr_y, va_x, va_y, unk_x = time_split(cfg, bursts)
    if len(classes) < 2:
        raise SystemExit(f"not enough classes (>= {cfg.min_class} bursts each): got {len(classes)}")
    log(f"classes={len(classes)} train={len(tr_x)} val={len(va_x)} "
        f"unknown-negatives={0 if unk_x is None else len(unk_x)}")

    tr_xt = torch.from_numpy(tr_x)          # pinned-host resident; tiny (~100MB)
    tr_yt = torch.from_numpy(tr_y)
    va_xt = torch.from_numpy(va_x)
    va_yt = torch.from_numpy(va_y)
    if device == "cuda":
        tr_xt, tr_yt = tr_xt.pin_memory(), tr_yt.pin_memory()

    # balanced sampling: weight 1/min(count, cap)
    counts = np.bincount(tr_y, minlength=len(classes)).astype(np.float64)
    w = 1.0 / np.minimum(counts, cfg.cap_per_class)[tr_y]
    sample_w = torch.from_numpy(w)
    epoch_len = max(cfg.batch, int(np.minimum(counts, cfg.cap_per_class).sum()))

    model = IQResNet1D(len(classes)).to(device)
    opt = torch.optim.AdamW(model.parameters(), lr=cfg.lr, weight_decay=cfg.weight_decay)
    iters_per_epoch = max(1, epoch_len // cfg.batch)
    total_iters = cfg.max_epochs * iters_per_epoch
    warm = cfg.warmup_epochs * iters_per_epoch
    sched = torch.optim.lr_scheduler.LambdaLR(
        opt, lambda it: (it + 1) / warm if it < warm
        else 0.5 * (1 + np.cos(np.pi * (it - warm) / max(1, total_iters - warm))))
    lossf = torch.nn.CrossEntropyLoss(label_smoothing=cfg.label_smooth)
    gen = torch.Generator(device=device).manual_seed(cfg.seed)
    cpu_gen = torch.Generator().manual_seed(cfg.seed)

    best_acc, best_state, bad = -1.0, None, 0
    amp_dtype = torch.bfloat16 if device == "cuda" else torch.float32
    for ep in range(cfg.max_epochs):
        model.train()
        idx = torch.multinomial(sample_w, epoch_len, replacement=True, generator=cpu_gen)
        for i in range(0, epoch_len - cfg.batch + 1, cfg.batch):
            b = idx[i:i + cfg.batch]
            xb = tr_xt[b].to(device, non_blocking=True)
            yb = tr_yt[b].to(device, non_blocking=True)
            xb = _augment(xb, cfg, gen)
            with torch.autocast(device_type=device, dtype=amp_dtype, enabled=device == "cuda"):
                loss = lossf(model(xb), yb)
            opt.zero_grad(set_to_none=True)
            loss.backward()
            opt.step()
            sched.step()
        # val balanced accuracy
        vl = _logits_all(model, va_xt, cfg.batch, device)
        pred = vl.argmax(1).numpy()
        accs = [float((pred[va_y == c] == c).mean()) for c in range(len(classes)) if (va_y == c).sum()]
        bal = float(np.mean(accs))
        log(f"epoch {ep + 1}: loss={float(loss.detach()):.3f} val_bal_acc={bal:.4f}")
        if bal > best_acc:
            best_acc, bad = bal, 0
            best_state = {k: v.detach().cpu().clone() for k, v in model.state_dict().items()}
        else:
            bad += 1
            if bad >= cfg.patience:
                log(f"early stop at epoch {ep + 1}")
                break
    model.load_state_dict(best_state)

    # ── calibration ────────────────────────────────────────────────────────
    vl = _logits_all(model, va_xt, cfg.batch, device)
    T = fit_temperature(vl, va_yt)
    pv = torch.softmax(vl / T, 1).numpy()
    pred = pv.argmax(1)
    top1 = float((pred == va_y).mean())
    pu = None
    if unk_x is not None and len(unk_x):
        ul = _logits_all(model, torch.from_numpy(unk_x), cfg.batch, device)
        pu = torch.softmax(ul / T, 1).numpy()
    taus, floor = calibrate_thresholds(cfg, pv, va_y, pu, len(classes))
    eff = np.array(taus)[pred]
    false_unknown = float((pv.max(1) < eff).mean())
    unk_fpr = float((pu.max(1) >= np.array(taus)[pu.argmax(1)]).mean()) if pu is not None else -1.0

    # Calibration: does a 0.99-confidence prediction turn out right ~99% of the
    # time? High top1 can hide a model that is systematically over-confident.
    ece = _ece(pv.max(1), (pred == va_y).astype(np.float64))
    # Per-class recall: a class with few bursts can hide behind a good overall
    # top1/bal_acc — surface the worst one so thin classes get flagged.
    per_class_recall = {int(classes[c]): float((pred[va_y == c] == c).mean())
                        for c in range(len(classes)) if (va_y == c).sum()}
    worst_mmsi, worst_recall = min(per_class_recall.items(), key=lambda kv: kv[1])
    log(f"top1={top1:.4f} bal_acc={best_acc:.4f} T={T:.3f} floor={floor:.2f} "
        f"false_unknown={false_unknown:.3f} natural_unknown_fpr={unk_fpr:.3f} "
        f"ece={ece:.4f} worst_class={worst_mmsi}({worst_recall:.3f})")

    meta = {
        "crop": crop, "input_len": int(tr_x.shape[2]),
        "nominal_sr": cfg.nominal_sr, "classes": [int(c) for c in classes],
        "temperature": T, "taus": taus, "tau_floor": floor,
        "val_top1": top1, "val_bal_acc": best_acc,
        "false_unknown": false_unknown, "natural_unknown_fpr": unk_fpr,
        "ece": ece, "per_class_recall": per_class_recall,
        "worst_class_mmsi": worst_mmsi, "worst_class_recall": worst_recall,
        "train_n": int(len(tr_x)), "val_n": int(len(va_x)),
        "trained_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "train_seconds": round(time.time() - t0, 1),
    }
    return model, meta, (va_x, va_y)


def _publish(cfg: Config, model, meta: dict, days: int, log=print):
    os.makedirs(cfg.model_dir, exist_ok=True)
    cur_path = os.path.join(cfg.model_dir, "current.json")
    ver = 1
    if os.path.exists(cur_path):
        try:
            ver = int(json.load(open(cur_path)).get("version", 0)) + 1
        except Exception:
            ver = int(time.time()) % 60000
    mname, jname = f"model_v{ver:04d}.pt", f"meta_v{ver:04d}.json"
    torch.save({k: v.cpu() for k, v in model.state_dict().items()},
               os.path.join(cfg.model_dir, mname))
    meta = {**meta, "version": ver, "days": days}
    json.dump(meta, open(os.path.join(cfg.model_dir, jname), "w"), indent=1)
    tmp = cur_path + ".tmp"
    json.dump({"version": ver, "model": mname, "meta": jname}, open(tmp, "w"))
    os.replace(tmp, cur_path)
    olds = sorted(f for f in os.listdir(cfg.model_dir) if f.startswith("model_v"))
    for f in olds[:-5]:   # keep last 5 versions
        for p in (f, f.replace("model_", "meta_").replace(".pt", ".json")):
            try:
                os.remove(os.path.join(cfg.model_dir, p))
            except OSError:
                pass
    log(f"published v{ver} ({mname})")
    return meta
