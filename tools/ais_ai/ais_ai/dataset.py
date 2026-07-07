"""Dataset assembly: crop, normalize, time-split, class selection.

Leakage rules (do not weaken):
- default crop excludes the AIS payload (payload bits 8-37 spell the MMSI —
  a payload-seeing model "demodulates" the label instead of fingerprinting).
- split is by TIME per class with a guard gap; adjacent bursts are
  near-duplicates so a random split reports fantasy accuracy.
"""
import numpy as np

from .aicap import day_files, iter_file
from .config import Config


def crop_bounds(cfg: Config, crop: str):
    if crop == "preamble":
        return cfg.crop_preamble_start, cfg.crop_preamble_len
    if crop == "payload":
        return cfg.crop_payload_start, cfg.crop_payload_len
    raise ValueError(f"unknown crop {crop!r}")


def crop_and_norm(iq: np.ndarray, start: int, length: int) -> np.ndarray | None:
    """Crop [start, start+length) and per-burst RMS normalize.
    Returns float32 [2, length] (I, Q rows) or None if unusable."""
    if len(iq) < start + length:
        if len(iq) < start + length // 2:
            return None
        pad = np.zeros(start + length, dtype=np.complex64)
        pad[:len(iq)] = iq
        iq = pad
    seg = iq[start:start + length]
    rms = np.sqrt(np.mean(np.abs(seg) ** 2)) + 1e-12
    seg = (seg / rms).astype(np.complex64)
    return np.stack([seg.real, seg.imag]).astype(np.float32)


def load_bursts(cfg: Config, days: int, crop: str = "preamble"):
    """Read aicap files -> {mmsi: (t_ms sorted asc, x [k,2,N] float32)}."""
    start, length = crop_bounds(cfg, crop)
    per: dict[int, list] = {}
    for path in day_files(cfg.data_dir, days):
        for rec in iter_file(path):
            if abs(rec.out_sr - cfg.nominal_sr) / cfg.nominal_sr > cfg.sr_tol:
                continue   # mixed sample rate — exclude rather than silently mix
            x = crop_and_norm(rec.iq, start, length)
            if x is None:
                continue
            per.setdefault(rec.mmsi, []).append((rec.t_ms, x))
    out = {}
    for mmsi, lst in per.items():
        lst.sort(key=lambda r: r[0])
        t = np.array([r[0] for r in lst], dtype=np.int64)
        x = np.stack([r[1] for r in lst])
        out[mmsi] = (t, x)
    return out


def time_split(cfg: Config, bursts):
    """Per-class time split. Returns (classes, tr_x, tr_y, va_x, va_y, unknown_x).

    classes: sorted MMSIs with >= min_class bursts. Sub-threshold MMSIs become
    natural open-set negatives (unknown_x)."""
    classes = sorted(m for m, (t, x) in bursts.items() if len(t) >= cfg.min_class)
    cls_idx = {m: i for i, m in enumerate(classes)}
    tr_x, tr_y, va_x, va_y, unk = [], [], [], [], []
    for mmsi, (t, x) in bursts.items():
        if mmsi not in cls_idx:
            unk.append(x)
            continue
        cut_t = t[0] + (t[-1] - t[0]) * (1.0 - cfg.val_frac)
        va_mask = t >= cut_t
        tr_mask = t < (cut_t - cfg.guard_gap_s * 1000)   # guard gap
        if va_mask.sum() == 0 or tr_mask.sum() == 0:
            k = int(len(t) * (1.0 - cfg.val_frac))
            tr_mask = np.zeros(len(t), bool); tr_mask[:k] = True
            va_mask = ~tr_mask
        y = cls_idx[mmsi]
        tr_x.append(x[tr_mask]); tr_y.append(np.full(tr_mask.sum(), y))
        va_x.append(x[va_mask]); va_y.append(np.full(va_mask.sum(), y))
    cat = lambda l: np.concatenate(l) if l else np.zeros((0,), np.float32)
    return (classes,
            cat(tr_x), cat(tr_y).astype(np.int64),
            cat(va_x), cat(va_y).astype(np.int64),
            np.concatenate(unk) if unk else None)
