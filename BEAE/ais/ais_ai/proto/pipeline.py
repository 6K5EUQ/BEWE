"""Train / enroll / evaluate / publish for the residual+prototypical model.

Isolated artifacts under data/ai_model_proto/ (never touches data/ai_model/).
Reuses the existing aicap reader, resampler and the per-class TIME split so results
are directly comparable to the deployed IQResNet1D (data/ai_model/ v1-v4).
"""
import json
import os
import time

import numpy as np
import torch
import torch.nn.functional as F

from ..aicap import day_files, iter_file
from ..dataset import resample_to_nominal
from .model import ProtoEmbed, prototypical_loss
from .residual import extract, N_CHANNELS, N_AUX

EMB = ProtoEmbed.EMB


def proto_dir(cfg):
    return os.path.join(cfg.data_dir, "ai_model_proto")


# ── data ────────────────────────────────────────────────────────────────────
def load_residual(cfg, days, log=print):
    start, length = cfg.crop_preamble_start, cfg.crop_preamble_len
    per = {}
    for path in day_files(cfg.data_dir, days):
        for rec in iter_file(path):
            if abs(rec.out_sr - cfg.nominal_sr) / cfg.nominal_sr > cfg.sr_resample_max:
                continue
            iq = resample_to_nominal(rec.iq, rec.out_sr, cfg)
            r = extract(iq, start, length, cfg.nominal_sr)
            if r is None:
                continue
            per.setdefault(rec.mmsi, []).append((rec.t_ms, r[0], r[1]))
    out = {}
    for mmsi, lst in per.items():
        lst.sort(key=lambda r: r[0])
        out[mmsi] = (np.array([r[0] for r in lst], np.int64),
                     np.stack([r[1] for r in lst]),
                     np.stack([r[2] for r in lst]))
    log(f"loaded residual bursts: {sum(len(v[0]) for v in out.values())} from {len(out)} MMSIs")
    return out


def time_split(cfg, bursts):
    """Per-class time split with guard gap (mirror dataset.time_split). Sub-threshold
    MMSIs become natural open-set negatives."""
    classes = sorted(m for m, (t, f, a) in bursts.items() if len(t) >= cfg.min_class)
    idx = {m: i for i, m in enumerate(classes)}
    trF, trA, trY, vaF, vaA, vaY, unkF, unkA = ([] for _ in range(8))
    for mmsi, (t, f, a) in bursts.items():
        if mmsi not in idx:
            unkF.append(f); unkA.append(a); continue
        cut = t[0] + (t[-1] - t[0]) * (1.0 - cfg.val_frac)
        vm = t >= cut
        tm = t < (cut - cfg.guard_gap_s * 1000)
        if vm.sum() == 0 or tm.sum() == 0:
            k = int(len(t) * (1.0 - cfg.val_frac))
            tm = np.zeros(len(t), bool); tm[:k] = True; vm = ~tm
        y = idx[mmsi]
        trF.append(f[tm]); trA.append(a[tm]); trY.append(np.full(tm.sum(), y))
        vaF.append(f[vm]); vaA.append(a[vm]); vaY.append(np.full(vm.sum(), y))
    z = lambda shape: np.zeros((0, *shape), np.float32)
    cat = lambda l, shape: np.concatenate(l) if l else z(shape)
    L = cfg.crop_preamble_len
    return {
        "classes": classes,
        "tr": (cat(trF, (N_CHANNELS, L)), cat(trA, (N_AUX,)),
               np.concatenate(trY).astype(np.int64) if trY else np.zeros(0, np.int64)),
        "va": (cat(vaF, (N_CHANNELS, L)), cat(vaA, (N_AUX,)),
               np.concatenate(vaY).astype(np.int64) if vaY else np.zeros(0, np.int64)),
        "unk": (cat(unkF, (N_CHANNELS, L)), cat(unkA, (N_AUX,))) if unkF else None,
    }


# ── embed / enroll / classify ───────────────────────────────────────────────
def embed_all(model, feat, aux, device, bs=1024):
    if len(feat) == 0:
        return np.zeros((0, EMB), np.float32)
    model.eval()
    outs = []
    with torch.no_grad():
        for i in range(0, len(feat), bs):
            f = torch.from_numpy(feat[i:i + bs]).to(device)
            a = torch.from_numpy(aux[i:i + bs]).to(device)
            outs.append(model(f, a).cpu().numpy())
    return np.concatenate(outs)


def enroll(emb, y, n_classes, k=None):
    """Prototype per class = L2-normalized mean embedding (optionally of first k bursts)."""
    protos = np.zeros((n_classes, EMB), np.float32)
    for c in range(n_classes):
        e = emb[y == c]
        if k is not None:
            e = e[:k]
        if len(e):
            m = e.mean(0)
            protos[c] = m / (np.linalg.norm(m) + 1e-9)
    return protos


def classify(emb, protos, taus=None):
    """Returns (pred_class, max_sim). UNKNOWN = -1 when taus given and max_sim < tau."""
    if len(emb) == 0:
        return np.zeros(0, np.int64), np.zeros(0, np.float32)
    sim = emb @ protos.T                      # cosine (both unit-norm)
    pred = sim.argmax(1)
    smax = sim[np.arange(len(sim)), pred]
    if taus is not None:
        pred = np.where(smax >= taus[pred], pred, -1)
    return pred, smax


# ── train ───────────────────────────────────────────────────────────────────
def _episode(feat, aux, y, cls_idx, C, S, Q, rng, device, aug):
    pick = rng.choice(len(cls_idx), size=min(C, len(cls_idx)), replace=False)
    fb, ab, yb = [], [], []
    for j, ci in enumerate(pick):
        ii = cls_idx[ci]
        sel = rng.choice(len(ii), size=S + Q, replace=len(ii) < S + Q)
        fb.append(feat[ii[sel]]); ab.append(aux[ii[sel]]); yb.append(np.full(S + Q, j))
    f = np.concatenate(fb); a = np.concatenate(ab); yy = np.concatenate(yb)
    if aug:
        f = f + rng.normal(0, 0.03, f.shape).astype(np.float32)   # light SNR jitter on residual
    return (torch.from_numpy(f).to(device), torch.from_numpy(a).to(device),
            torch.from_numpy(yy).to(device))


def train_embed(cfg, split, log=print, seed=42, C=20, S=5, Q=10, episodes=1500):
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    trF, trA, trY = split["tr"]
    classes = split["classes"]
    n_cls = len(classes)
    # episode-eligible = classes with >= S+Q training bursts (others still enroll few-shot later)
    cls_idx = [np.where(trY == c)[0] for c in range(n_cls)]
    elig = [c for c in range(n_cls) if len(cls_idx[c]) >= S + Q]
    log(f"episode-eligible classes: {len(elig)}/{n_cls} (need >= {S+Q} train bursts)")
    if len(elig) < 2:
        raise RuntimeError("not enough eligible classes to train embedding")
    rng = np.random.default_rng(seed)
    torch.manual_seed(seed)
    model = ProtoEmbed().to(dev)
    opt = torch.optim.AdamW(model.parameters(), lr=cfg.lr, weight_decay=cfg.weight_decay)
    sched = torch.optim.lr_scheduler.CosineAnnealingLR(opt, episodes)
    eidx = [cls_idx[c] for c in elig]
    model.train()
    t0 = time.time()
    for ep in range(episodes):
        f, a, yy = _episode(trF, trA, trY, eidx, C, S, Q, rng, dev, aug=True)
        emb = model(f, a)
        loss, acc = prototypical_loss(emb, yy, S)
        opt.zero_grad(); loss.backward(); opt.step(); sched.step()
        if (ep + 1) % 300 == 0:
            log(f"  ep {ep+1}/{episodes} loss={float(loss.detach()):.3f} ep_acc={float(acc.detach()):.3f}")
    log(f"embed trained in {time.time()-t0:.1f}s ({dev})")
    return model, dev


def calibrate(cfg, va_emb, va_y, protos, unk_emb):
    """Per-class tau = percentile of correct-match cosine sim; raise global floor
    until natural-unknown acceptance <= fpr_target."""
    n_cls = len(protos)
    pred, smax = classify(va_emb, protos)
    taus = np.full(n_cls, cfg.tau_min, np.float32)
    for c in range(n_cls):
        m = (va_y == c) & (pred == c)
        if m.sum() >= 5:
            taus[c] = np.percentile(smax[m], cfg.tau_pctile)
    if unk_emb is not None and len(unk_emb):
        _, us = classify(unk_emb, protos)
        up = (unk_emb @ protos.T).argmax(1)
        floor = cfg.tau_min
        for f in np.linspace(cfg.tau_min, 0.999, 60):
            eff = np.maximum(taus, f)
            fpr = float((us >= eff[up]).mean())
            floor = f
            if fpr <= cfg.fpr_target:
                break
        taus = np.maximum(taus, floor)
    return taus


# ── report ──────────────────────────────────────────────────────────────────
def report(cfg, split, model, dev, log=print, few_k=10):
    classes = split["classes"]; n = len(classes)
    trF, trA, trY = split["tr"]; vaF, vaA, vaY = split["va"]
    tr_emb = embed_all(model, trF, trA, dev)
    va_emb = embed_all(model, vaF, vaA, dev)
    unk = split["unk"]
    unk_emb = embed_all(model, unk[0], unk[1], dev) if unk else None
    protos = enroll(tr_emb, trY, n)
    taus = calibrate(cfg, va_emb, vaY, protos, unk_emb)
    pred, smax = classify(va_emb, protos)                      # closed-set (no threshold)
    top1 = float((pred == vaY).mean()) if len(vaY) else 0.0
    rec = np.array([(pred[vaY == c] == c).mean() if (vaY == c).sum() else 0.0 for c in range(n)])
    bal = float(rec.mean())
    predo, _ = classify(va_emb, protos, taus)
    false_unknown = float((predo == -1).mean()) if len(vaY) else 0.0
    nat_fpr = -1.0
    if unk_emb is not None and len(unk_emb):
        po, _ = classify(unk_emb, protos, taus)
        nat_fpr = float((po != -1).mean())
    # few-shot: enroll each class from only few_k train bursts, re-measure balanced recall
    protos_k = enroll(tr_emb, trY, n, k=few_k)
    predk, _ = classify(va_emb, protos_k)
    reck = np.array([(predk[vaY == c] == c).mean() if (vaY == c).sum() else 0.0 for c in range(n)])
    log(f"classes={n} tr={len(trY)} va={len(vaY)} unk={0 if unk_emb is None else len(unk_emb)}")
    log(f"top1={top1:.4f} bal_acc={bal:.4f} false_unknown={false_unknown:.3f} "
        f"nat_fpr={nat_fpr:.3f} worst_recall={rec.min():.3f}")
    log(f"few-shot(k={few_k}) bal_acc={float(reck.mean()):.4f}  (full-enroll bal_acc={bal:.4f})")
    return {"classes": [int(c) for c in classes], "top1": top1, "bal_acc": bal,
            "false_unknown": false_unknown, "nat_fpr": nat_fpr,
            "worst_recall": float(rec.min()), "few_shot_bal_acc": float(reck.mean()),
            "protos": protos.tolist(), "taus": taus.tolist(),
            "tr_n": int(len(trY)), "va_n": int(len(vaY))}


# ── publish (isolated dir) / orchestrate / autotrain ────────────────────────
def publish(cfg, model, result, days, log=print):
    d = proto_dir(cfg)
    os.makedirs(d, exist_ok=True)
    cur = os.path.join(d, "current.json")
    ver = 1
    if os.path.exists(cur):
        try:
            ver = int(json.load(open(cur)).get("version", 0)) + 1
        except Exception:
            ver = 1
    mname, jname = f"embed_v{ver:04d}.pt", f"protos_v{ver:04d}.json"
    torch.save({k: v.cpu() for k, v in model.state_dict().items()}, os.path.join(d, mname))
    meta = {**result, "version": ver, "days": days, "emb": EMB,
            "crop": "preamble", "n_channels": N_CHANNELS, "n_aux": N_AUX,
            "trained_at": time.strftime("%Y-%m-%dT%H:%M:%S%z")}
    json.dump(meta, open(os.path.join(d, jname), "w"))
    tmp = os.path.join(d, "current.json.tmp")
    json.dump({"version": ver, "model": mname, "protos": jname}, open(tmp, "w"))
    os.replace(tmp, cur)
    for pre in ("embed_v", "protos_v"):
        olds = sorted(f for f in os.listdir(d) if f.startswith(pre))
        for f in olds[:-5]:
            os.remove(os.path.join(d, f))
    log(f"published proto v{ver} ({mname}) -> {d}")
    return ver


def run_train(cfg, days, log=print, do_publish=True, bursts=None):
    if bursts is None:
        bursts = load_residual(cfg, days, log)
    split = time_split(cfg, bursts)
    model, dev = train_embed(cfg, split, log)
    result = report(cfg, split, model, dev, log)
    if do_publish:
        publish(cfg, model, {k: v for k, v in result.items()}, days, log)
    return result


def autotrain(cfg, days, min_classes, log=print):
    """Train only once enough transmitters have accumulated. Cheap to run on a timer:
    if under threshold it just reports and exits without publishing."""
    bursts = load_residual(cfg, days, log)
    elig = [m for m, (t, f, a) in bursts.items() if len(t) >= cfg.min_class]
    log(f"eligible transmitters={len(elig)} / threshold={min_classes} "
        f"(>= {cfg.min_class} bursts each)")
    if len(elig) < min_classes:
        log("not enough data yet -> skip training")
        return None
    return run_train(cfg, days, log, do_publish=True, bursts=bursts)


def status(cfg, log=print):
    d = proto_dir(cfg)
    cur = os.path.join(d, "current.json")
    if not os.path.exists(cur):
        log("no proto model published yet")
        return
    c = json.load(open(cur))
    m = json.load(open(os.path.join(d, c["protos"])))
    log(f"proto current: v{c['version']} ({c['model']})")
    log(f"  classes={len(m['classes'])} top1={m['top1']:.4f} bal_acc={m['bal_acc']:.4f} "
        f"few_shot_bal_acc={m.get('few_shot_bal_acc', -1):.4f} "
        f"false_unknown={m['false_unknown']:.3f} nat_fpr={m['nat_fpr']:.3f} "
        f"trained_at={m.get('trained_at', '?')}")
    vers = sorted(f for f in os.listdir(d) if f.startswith("embed_v"))
    log(f"  versions kept (rollback): {' '.join(vers)}")
