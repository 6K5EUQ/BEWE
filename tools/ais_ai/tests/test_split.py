import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
from ais_ai.config import Config
from ais_ai.dataset import time_split


def make_bursts(mmsi, n, t0, dt_ms=5000, length=320):
    t = t0 + np.arange(n, dtype=np.int64) * dt_ms
    x = np.random.default_rng(mmsi).normal(size=(n, 2, length)).astype(np.float32)
    return t, x


def test_guard_gap_no_leakage():
    cfg = Config()
    cfg.min_class = 200
    bursts = {440000001 + i: make_bursts(440000001 + i, 400, 1700000000000)
              for i in range(3)}
    classes, tr_x, tr_y, va_x, va_y, unk = time_split(cfg, bursts)
    assert len(classes) == 3
    for c, mmsi in enumerate(classes):
        t, _ = bursts[mmsi]
        cut = t[0] + (t[-1] - t[0]) * (1 - cfg.val_frac)
        n_tr = int((t < cut - cfg.guard_gap_s * 1000).sum())
        n_va = int((t >= cut).sum())
        assert (tr_y == c).sum() == n_tr and (va_y == c).sum() == n_va
        # max train time + guard <= min val time
        max_tr_t = t[t < cut - cfg.guard_gap_s * 1000].max()
        min_va_t = t[t >= cut].min()
        assert max_tr_t + cfg.guard_gap_s * 1000 <= min_va_t


def test_subthreshold_becomes_unknown():
    cfg = Config()
    cfg.min_class = 200
    bursts = {1: make_bursts(1, 300, 0), 2: make_bursts(2, 300, 0),
              3: make_bursts(3, 50, 0)}   # 3 below threshold
    classes, *_ , unk = time_split(cfg, bursts)
    assert 3 not in classes and unk is not None and len(unk) == 50
