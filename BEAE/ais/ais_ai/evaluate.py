"""Evaluation report: per-class P/R, confusion pairs, open-set holdout,
statistical baseline (nearest-centroid Mahalanobis on JSONL hand features)."""
import glob
import json
import os

import numpy as np
import torch

from .config import Config
from .dataset import load_bursts, time_split
from .openset import decide
from .proto import ST_UNKNOWN
from .train import _fit, _logits_all


def _report_closed(classes, va_y, probs, taus, log):
    pred = probs.argmax(1)
    log(f"\n== closed-set (val n={len(va_y)}) ==")
    log(f"top-1 acc: {(pred == va_y).mean():.4f}")
    rows, conf_pairs = [], {}
    for c, mmsi in enumerate(classes):
        sel = va_y == c
        if not sel.sum():
            continue
        rec = float((pred[sel] == c).mean())
        psel = pred == c
        prec = float((va_y[psel] == c).mean()) if psel.sum() else 0.0
        rows.append((mmsi, int(sel.sum()), prec, rec))
        for w in pred[sel][pred[sel] != c]:
            conf_pairs[(mmsi, classes[w])] = conf_pairs.get((mmsi, classes[w]), 0) + 1
    log(f"{'MMSI':>10} {'n':>6} {'prec':>6} {'rec':>6}")
    for mmsi, n, p, r in sorted(rows, key=lambda r: r[3]):
        log(f"{mmsi:>10} {n:>6} {p:>6.3f} {r:>6.3f}")
    top = sorted(conf_pairs.items(), key=lambda kv: -kv[1])[:10]
    if top:
        log("top confused pairs (true -> predicted):")
        for (a, b), n in top:
            log(f"  {a} -> {b}: {n}")
    eff = np.array(taus)[pred]
    log(f"false-UNKNOWN on knowns: {(probs.max(1) < eff).mean():.3f}")


def run_eval(cfg: Config, days: int, holdout_k: int = 0, baseline: bool = False,
             crop: str = "clean", log=print):
    bursts = load_bursts(cfg, days, crop)
    if holdout_k > 0:
        # hold out K mid-frequency eligible classes entirely, retrain, measure
        # how often their bursts route to UNKNOWN (open-set target >= 90%)
        elig = sorted((m for m, (t, x) in bursts.items() if len(t) >= cfg.min_class),
                      key=lambda m: len(bursts[m][0]))
        mid = elig[len(elig) // 2 - holdout_k // 2: len(elig) // 2 - holdout_k // 2 + holdout_k]
        held = {m: bursts.pop(m) for m in mid}
        log(f"held-out MMSIs: {mid}")
        model, meta, (va_x, va_y) = _fit(cfg, bursts, crop, log)
        device = next(model.parameters()).device
        unk_total, unk_routed = 0, 0
        for mmsi, (t, x) in held.items():
            lg = _logits_all(model, torch.from_numpy(x), cfg.batch, device).numpy()
            st = [decide(l, meta["temperature"], meta["taus"], meta["classes"])[0] for l in lg]
            n_unk = sum(1 for s in st if s == ST_UNKNOWN)
            unk_total += len(st); unk_routed += n_unk
            log(f"  held-out {mmsi}: UNKNOWN {n_unk}/{len(st)} ({n_unk / len(st):.2%})")
        log(f"open-set UNKNOWN rate on held-out: {unk_routed / max(1, unk_total):.2%} (target >= 90%)")
        vl = _logits_all(model, torch.from_numpy(va_x), cfg.batch, device)
        probs = torch.softmax(vl / meta["temperature"], 1).numpy()
        _report_closed(meta["classes"], va_y, probs, meta["taus"], log)
    else:
        model, meta, (va_x, va_y) = _fit(cfg, bursts, crop, log)
        device = next(model.parameters()).device
        vl = _logits_all(model, torch.from_numpy(va_x), cfg.batch, device)
        probs = torch.softmax(vl / meta["temperature"], 1).numpy()
        _report_closed(meta["classes"], va_y, probs, meta["taus"], log)
    if baseline:
        baseline_jsonl(cfg, days, log)


def baseline_jsonl(cfg: Config, days: int, log=print):
    """Apples-to-apples statistical baseline: nearest-centroid normalized
    distance on [cfo, fdev_std, clk_ppm] from the daily JSONL, same
    time-split scheme. The DL model should beat this top-1."""
    files = sorted(glob.glob(os.path.join(cfg.data_dir, "ais_*.jsonl")))[-days:]
    per = {}
    for path in files:
        with open(path) as f:
            for line in f:
                try:
                    d = json.loads(line)
                except json.JSONDecodeError:
                    continue
                if not d.get("mmsi") or "cfo" not in d:
                    continue
                per.setdefault(d["mmsi"], []).append(
                    (d["t"], d["cfo"], d.get("fstd", 0.0), d.get("ppm", 0.0)))
    feats = {}
    for mmsi, rows in per.items():
        if len(rows) < cfg.min_class:
            continue
        rows.sort()
        a = np.array(rows, dtype=np.float64)
        feats[mmsi] = (a[:, 0], a[:, 1:4])
    classes = sorted(feats)
    if len(classes) < 2:
        log("baseline: not enough classes")
        return
    cents, stds, va = {}, {}, []
    for mmsi in classes:
        t, x = feats[mmsi]
        cut = t[0] + (t[-1] - t[0]) * (1 - cfg.val_frac)
        tr, vl = x[t < cut - cfg.guard_gap_s * 1000], x[t >= cut]
        if len(tr) < 10 or len(vl) == 0:
            continue
        cents[mmsi] = tr.mean(0)
        stds[mmsi] = np.maximum(tr.std(0), [30.0, 20.0, 0.5])   # noise floors
        va.append((mmsi, vl))
    ok = tot = 0
    for mmsi, vl in va:
        for row in vl:
            d = {m: float((((row - cents[m]) / stds[m]) ** 2).sum()) for m in cents}
            ok += min(d, key=d.get) == mmsi
            tot += 1
    log(f"\n== statistical baseline (CFO/fdev/ppm nearest-centroid) ==")
    log(f"classes={len(cents)} val n={tot} top-1: {ok / max(1, tot):.4f}")
