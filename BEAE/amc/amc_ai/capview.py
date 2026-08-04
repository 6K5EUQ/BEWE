"""Inspect the real IQ that BEWE actually sent (amccap_*.bin).

The model is trained on synthetic bursts only. When it disagrees with reality the
useful question is not "which class did it say" but "what does the input look
like" — so this prints measurable properties of each capture next to the model's
answer, and compares them against the synthetic training distribution.

Enable the dump first:  BEWE_AMC_CAP=1 ./BEWE ...
Then:                   python -m amc_ai.capview [--n 40]
"""
from __future__ import annotations

import argparse
import glob
import os
import struct
import sys

import numpy as np

from .config import Config
from .synth import CLASSES, make_clean, impair

HDR = struct.Struct("<IHHIqIHBB")   # mirror of AmcReqHdr (amc_ai.cpp)
assert HDR.size == 28
MAGIC = int.from_bytes(b"AMRQ", "little")


def read_caps(path: str):
    out = []
    with open(path, "rb") as f:
        blob = f.read()
    off = 0
    while off + HDR.size <= len(blob):
        magic, ver, typ, bw, t_ms, sr, n, ch, trig = HDR.unpack_from(blob, off)
        if magic != MAGIC:
            break
        need = HDR.size + 8 * n
        if off + need > len(blob):
            break
        iq = np.frombuffer(blob, dtype=np.float32, count=2 * n,
                           offset=off + HDR.size).copy().view(np.complex64)
        out.append(dict(t_ms=t_ms, bw=bw, sr=sr, n=n, ch=ch, trig=trig, iq=iq))
        off += need
    return out


def props(x: np.ndarray, sr: int):
    """Measurable properties — the same handful for real and synthetic, so the
    two can be compared without trusting the model."""
    x = x / (np.sqrt(np.mean(np.abs(x) ** 2)) + 1e-12)
    a = np.abs(x)
    # envelope variation: ~0 for constant-envelope (FM/FSK), large for AM/QAM
    papr = 20 * np.log10(a.max() / (np.sqrt(np.mean(a ** 2)) + 1e-12) + 1e-12)
    env_cv = a.std() / (a.mean() + 1e-12)
    # occupied bandwidth (99%) as a fraction of sr
    X = np.abs(np.fft.fftshift(np.fft.fft(x))) ** 2
    c = np.cumsum(X) / X.sum()
    lo, hi = np.searchsorted(c, 0.005), np.searchsorted(c, 0.995)
    occ = (hi - lo) / len(x)
    # spectral centroid offset from 0 (is the channel actually centred?)
    f = np.fft.fftshift(np.fft.fftfreq(len(x), 1 / sr))
    cen = float((X * f).sum() / X.sum())
    # DC energy share (RTL LO leakage lands here)
    k0 = len(x) // 2
    dc = float(X[k0 - 1:k0 + 2].sum() / X.sum())
    # instantaneous-frequency spread (FM/FSK family separator)
    ph = np.unwrap(np.angle(x))
    ifr = np.diff(ph) * sr / (2 * np.pi)
    return dict(papr=papr, env_cv=env_cv, occ=occ, cen_khz=cen / 1e3,
                dc=dc, if_std_khz=float(ifr.std()) / 1e3)


def synth_ref(sr: int, nsamp: int, seed=0):
    rng = np.random.default_rng(seed)
    ref = {}
    for k in CLASSES:
        rows = [props(impair(make_clean(k, nsamp, rng), 20, rng), sr) for _ in range(12)]
        ref[k] = {kk: float(np.median([r[kk] for r in rows])) for kk in rows[0]}
    return ref


def main():
    ap = argparse.ArgumentParser(prog="amc_ai.capview")
    ap.add_argument("--n", type=int, default=20, help="most recent N captures")
    ap.add_argument("--file", default=None)
    ap.add_argument("--no-model", action="store_true", help="skip inference")
    a = ap.parse_args()

    cfg = Config()
    path = a.file
    if not path:
        g = sorted(glob.glob(os.path.join(cfg.data_dir, "amccap_*.bin")))
        if not g:
            print(f"no amccap_*.bin in {cfg.data_dir}\n"
                  f"run BEWE with BEWE_AMC_CAP=1 first", file=sys.stderr)
            return 1
        path = g[-1]
    caps = read_caps(path)[-a.n:]
    if not caps:
        print(f"no records in {path}", file=sys.stderr)
        return 1
    print(f"{path}: {len(caps)} records\n")

    reg = None
    if not a.no_model:
        from .infer import ModelRegistry
        reg = ModelRegistry(cfg)

    print(f"{'time':>8} {'ch':>3} {'bw kHz':>7} {'sr kHz':>7} {'occ':>5} "
          f"{'papr':>5} {'envCV':>6} {'cenkHz':>7} {'dc%':>5} {'ifSD':>7}  pred")
    for c in caps:
        p = props(c["iq"], c["sr"])
        pred = ""
        if reg is not None:
            st, c1, p1, c2, p2, mv, _pp = reg.predict(c["iq"], c["sr"])
            pred = f"{CLASSES[c1] if c1 < len(CLASSES) else '?'} {p1*100:.0f}%"
        import time as _t
        ts = _t.strftime("%H:%M:%S", _t.localtime(c["t_ms"] / 1000))
        print(f"{ts:>8} {c['ch']:>3} {c['bw']/1e3:>7.1f} {c['sr']/1e3:>7.1f} "
              f"{p['occ']:>5.3f} {p['papr']:>5.1f} {p['env_cv']:>6.3f} "
              f"{p['cen_khz']:>7.1f} {p['dc']*100:>5.1f} {p['if_std_khz']:>7.1f}  {pred}")

    sr = caps[-1]["sr"]
    n = len(caps[-1]["iq"])
    print("\nsynthetic reference @ SNR 20 dB (median):")
    print(f"{'class':>6} {'occ':>5} {'papr':>5} {'envCV':>6} {'ifSD':>7}")
    for k, v in synth_ref(sr, n).items():
        print(f"{k:>6} {v['occ']:>5.3f} {v['papr']:>5.1f} {v['env_cv']:>6.3f} {v['if_std_khz']:>7.1f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
