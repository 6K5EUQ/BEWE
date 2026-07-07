"""ModelRegistry: load current checkpoint, hot-swap on current.json change."""
import json
import logging
import os
import threading

import numpy as np
import torch

from .config import Config
from .dataset import crop_and_norm, crop_bounds
from .model import IQResNet1D
from .proto import ST_ERROR, ST_NO_MODEL

log = logging.getLogger("ais_ai.infer")


class ModelRegistry:
    def __init__(self, cfg: Config):
        self.cfg = cfg
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self._lock = threading.Lock()
        self._model = None
        self._meta = None
        self._cur_mtime = 0.0
        self.maybe_reload()

    @property
    def meta(self):
        return self._meta

    def maybe_reload(self):
        cur = os.path.join(self.cfg.model_dir, "current.json")
        try:
            mtime = os.path.getmtime(cur)
        except OSError:
            return
        if mtime == self._cur_mtime:
            return
        try:
            info = json.load(open(cur))
            meta = json.load(open(os.path.join(self.cfg.model_dir, info["meta"])))
            model = IQResNet1D(len(meta["classes"]))
            state = torch.load(os.path.join(self.cfg.model_dir, info["model"]),
                               map_location="cpu", weights_only=True)
            model.load_state_dict(state)
            model.eval().to(self.device)
            with torch.no_grad():   # warm-up forward (kernel autotune)
                model(torch.zeros(1, 2, meta["input_len"], device=self.device))
            with self._lock:
                self._model, self._meta, self._cur_mtime = model, meta, mtime
            log.info("hot-swapped model v%s (%d classes, val_top1=%.3f)",
                     meta["version"], len(meta["classes"]), meta.get("val_top1", -1))
        except Exception:
            log.exception("model reload failed — keeping previous")

    def predict(self, iq: np.ndarray, out_sr: int):
        """iq complex64 (full capture) -> (status, pred_mmsi, conf_pct, model_ver)."""
        with self._lock:
            model, meta = self._model, self._meta
        if model is None:
            return ST_NO_MODEL, 0, 0, 0
        try:
            start, length = crop_bounds(self.cfg, meta.get("crop", "preamble"))
            x = crop_and_norm(iq, start, length)
            if x is None:
                return ST_ERROR, 0, 0, meta["version"]
            with torch.no_grad():
                logits = model(torch.from_numpy(x).unsqueeze(0).to(self.device))
            from .openset import decide
            st, mmsi, conf = decide(logits[0].float().cpu().numpy(),
                                    meta["temperature"], meta["taus"], meta["classes"])
            return st, mmsi, conf, meta["version"]
        except Exception:
            log.exception("inference failed")
            return ST_ERROR, 0, 0, meta["version"] if meta else 0
