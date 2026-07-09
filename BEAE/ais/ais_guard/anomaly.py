"""AnomalyScorer — TrajNet 예측오차 백분위(0.6) + Grid 희귀도(0.4) 결합 점수 0-100.

traj current.json/grid.npz는 5초 폴링으로 mtime 변경 시 hot-swap (ais_ai/infer.py 패턴).
daemon.py 주입용 시그니처:
  scorer_fn(mmsi:int, track:list[tuple[t_ms,lat,lon,sog,cog]]) -> (float, str) | None
  predict_fn(mmsi:int, track) -> list[(lat,lon)] | None
"""
import json
import logging
import os
import threading
import time

import numpy as np
import torch

from .grid import GridStats, GRID_PATH, MODEL_DIR
from .traj_model import TrajNet, SEQ_IN, SEQ_OUT

log = logging.getLogger("ais_guard.anomaly")

W_TRAJ, W_GRID = 0.6, 0.4


class AnomalyScorer:
    POLL_S = 5.0                                        # 모델 파일 폴링 주기

    def __init__(self, model_dir: str = MODEL_DIR, grid_path: str = GRID_PATH):
        self.model_dir = model_dir
        self.grid_path = grid_path
        self._lock = threading.Lock()
        self._model = None                              # TrajNet | None
        self._pctl = None                               # np.ndarray(99) 정규화오차 백분위
        self._grid = None                               # GridStats | None
        self._ver = 0
        self._cur_mtime = 0.0
        self._grid_mtime = 0.0
        self._last_poll = 0.0
        self._maybe_reload(force=True)

    # ── hot-swap ─────────────────────────────────────────────────────────────
    def _maybe_reload(self, force=False):
        now = time.monotonic()
        if not force and now - self._last_poll < self.POLL_S:
            return
        self._last_poll = now
        cur = os.path.join(self.model_dir, "current.json")
        try:
            mt = os.path.getmtime(cur)
        except OSError:
            mt = 0.0
        if mt and mt != self._cur_mtime:
            try:
                info = json.load(open(cur))
                ck = torch.load(os.path.join(self.model_dir, info["model"]),
                                map_location="cpu", weights_only=True)
                m = TrajNet()
                m.load_state_dict(ck["state_dict"])
                m.eval()
                pctl = np.asarray(ck.get("err_pctl") or [], np.float64)
                with self._lock:
                    self._model, self._pctl = m, pctl
                    self._ver, self._cur_mtime = int(info.get("version", 0)), mt
                log.info("hot-swapped traj model v%d (val_mse=%.4f)",
                         self._ver, ck.get("val_mse", -1.0))
            except Exception:
                log.exception("traj model reload failed — keeping previous")
        try:
            gmt = os.path.getmtime(self.grid_path)
        except OSError:
            gmt = 0.0
        if gmt and gmt != self._grid_mtime:
            try:
                g = GridStats.load(self.grid_path)
                with self._lock:
                    self._grid, self._grid_mtime = g, gmt
                log.info("hot-swapped grid (%d cells)", len(g.cells))
            except Exception:
                log.exception("grid reload failed — keeping previous")

    def _snapshot(self):
        self._maybe_reload()
        with self._lock:
            return self._model, self._pctl, self._grid

    # ── daemon 주입 API ──────────────────────────────────────────────────────
    def scorer_fn(self, mmsi: int, track):
        """(score 0-100, why 한국어) 또는 None (판단 재료 없음)."""
        model, pctl, grid = self._snapshot()
        traj_r = grid_r = None
        if model is not None and len(track) > SEQ_IN + SEQ_OUT:
            err = model.anomaly_error(track)
            if err is not None:
                if pctl is not None and len(pctl):
                    traj_r = float(np.searchsorted(pctl, err)) / 100.0
                else:
                    traj_r = min(1.0, err)              # 백분위 없으면 원시 비율
        if grid is not None and track:
            _t, lat, lon, sog, cog = track[-1]
            grid_r = grid.score_point(lat, lon, sog, cog)
        if traj_r is None and grid_r is None:
            return None
        if traj_r is not None and grid_r is not None:
            score = 100.0 * (W_TRAJ * traj_r + W_GRID * grid_r)
        else:
            score = 100.0 * (traj_r if traj_r is not None else grid_r)
        why = []
        if traj_r is not None:
            why.append(f"예측오차 상위 {100-traj_r*100:.0f}%")
        if grid_r is not None and grid_r >= 0.5:
            why.append("드문 해역·속력·침로")
        return (float(min(max(score, 0.0), 100.0)), ", ".join(why) or "정상")

    def predict_fn(self, mmsi: int, track):
        """미래 8점 [(lat,lon)...] 또는 None (모델 없음/점 부족)."""
        model, _pctl, _grid = self._snapshot()
        if model is None:
            return None
        return model.predict(track)


# ── 모듈 레벨 편의 함수 (기본 인스턴스 지연 생성) ────────────────────────────
_default = None
_default_lock = threading.Lock()


def get_scorer() -> AnomalyScorer:
    global _default
    with _default_lock:
        if _default is None:
            _default = AnomalyScorer()
        return _default


def scorer_fn(mmsi: int, track):
    return get_scorer().scorer_fn(mmsi, track)


def predict_fn(mmsi: int, track):
    return get_scorer().predict_fn(mmsi, track)
