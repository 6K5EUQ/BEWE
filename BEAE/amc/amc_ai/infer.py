"""Model registry + inference for the AMC daemon.

Hot-swap: current.json is polled by mtime, so `python -m amc_ai train` can publish
a new checkpoint while the daemon keeps serving — no restart, no dropped bursts.
"""
from __future__ import annotations

import json
import logging
import os
import threading

import numpy as np
import torch

from .config import Config
from .model import IQResNet1D
from .synth import CLASSES, to_tensor_layout

log = logging.getLogger("amc_ai.infer")


class ModelRegistry:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.lock = threading.Lock()
        self.model = None
        self.version = 0
        self.nsamp = cfg.default_nsamp
        self.classes = list(CLASSES)
        self._mtime = 0.0
        self.dev = "cuda" if torch.cuda.is_available() else "cpu"
        self.maybe_reload(force=True)

    # ── loading ────────────────────────────────────────────────────────────
    def _current_path(self) -> str:
        return os.path.join(self.cfg.model_dir, "current.json")

    def maybe_reload(self, force: bool = False):
        p = self._current_path()
        try:
            mt = os.path.getmtime(p)
        except OSError:
            return
        if not force and mt <= self._mtime:
            return
        try:
            with open(p) as f:
                meta = json.load(f)
            ckpt = os.path.join(self.cfg.model_dir, meta["model"])
            ck = torch.load(ckpt, map_location=self.dev)
            classes = meta.get("classes") or list(CLASSES)
            m = IQResNet1D(n_classes=len(classes)).to(self.dev)
            # AMC checkpoints are dict-wrapped ({"state": ...}), unlike the bare
            # state_dict the AIS stack saves. Accept both so an older/newer
            # trainer does not brick the daemon.
            m.load_state_dict(ck["state"] if isinstance(ck, dict) and "state" in ck else ck)
            m.eval()
            with self.lock:
                self.model = m
                self.version = int(meta.get("version", 0))
                self.nsamp = int(meta.get("nsamp", self.cfg.default_nsamp))
                self.classes = classes
                self._mtime = mt
            log.info("model v%d loaded (%s, nsamp=%d, %d classes, dev=%s)",
                     self.version, meta.get("model"), self.nsamp, len(classes), self.dev)
        except Exception:
            log.exception("model load failed (%s)", p)

    # ── prediction ─────────────────────────────────────────────────────────
    def _crop(self, iq: np.ndarray) -> np.ndarray:
        n = self.nsamp
        if len(iq) == n:
            return iq
        if len(iq) < n:                       # pad: rare, only if BEWE sends short
            out = np.zeros(n, dtype=np.complex64)
            out[:len(iq)] = iq
            return out
        if self.cfg.crop_center:
            off = (len(iq) - n) // 2
            return iq[off:off + n]
        return iq[:n]

    def predict(self, iq: np.ndarray, out_sr: int):
        """Returns (status, cls, conf, cls2, conf2, model_ver, probs)."""
        with self.lock:
            m, ver, classes = self.model, self.version, self.classes
        if m is None:
            from .proto import ST_NO_MODEL, NO_CLASS
            return ST_NO_MODEL, NO_CLASS, 0.0, NO_CLASS, 0.0, 0, []
        from .proto import ST_OK, NO_CLASS
        x = to_tensor_layout(self._crop(iq.astype(np.complex64)))
        t = torch.from_numpy(x).unsqueeze(0).to(self.dev)
        with torch.no_grad():
            p = torch.softmax(m(t), dim=1)[0].cpu().numpy()
        order = np.argsort(p)[::-1]
        c1 = int(order[0])
        c2 = int(order[1]) if len(order) > 1 else -1
        # Guard against a checkpoint whose class list is longer than the one the
        # C++ side knows — an out-of-range index would render as "?" forever.
        if c1 >= len(classes):
            return ST_OK, NO_CLASS, 0.0, NO_CLASS, 0.0, ver, []
        return ST_OK, c1, float(p[c1]), (c2 if c2 >= 0 else NO_CLASS), \
            (float(p[c2]) if c2 >= 0 else 0.0), ver, [float(x) for x in p]
