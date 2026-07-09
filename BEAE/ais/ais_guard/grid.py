"""해역 격자 통계 — 0.005° 셀별 (방문수, 속력 평균/표준편차, 침로 8방위 히스토그램).

score_point()는 관측치가 과거 통행 패턴에서 얼마나 벗어나는지 [0,1] 희귀도로 돌려준다.
"""
import glob
import json
import math
import os

import numpy as np

CELL = 0.005                                            # 격자 한 변 (도)
AIS_DIR = os.path.expanduser("~/BEWE/modules/ais")      # HOST가 append하는 일별 jsonl
GUARD_DIR = os.path.expanduser("~/BEWE/BEAE/ais/data/guard")
MODEL_DIR = os.path.join(GUARD_DIR, "model")
GRID_PATH = os.path.join(MODEL_DIR, "grid.npz")

POSITION_TYPES = {1, 2, 3, 18, 19, 27}                  # 위치보고 메시지만
_REF_COUNT = math.log1p(500.0)                          # 방문 500회 이상 = 완전히 흔한 셀
_MIN_STAT = 20                                          # 속력/침로 통계 최소 표본


def day_files(days: int, ais_dir: str = AIS_DIR):
    """최근 days일치 ais_*.jsonl 경로 (오래된 것부터). days<=0 → 전부."""
    files = sorted(glob.glob(os.path.join(ais_dir, "ais_*.jsonl")))
    return files[-days:] if days > 0 else files


def iter_positions(path: str):
    """한 파일에서 유효 위치보고만 yield: (t_ms, mmsi, lat, lon, sog, cog)."""
    try:
        f = open(path, encoding="utf-8", errors="replace")
    except OSError:
        return
    with f:
        for line in f:
            try:
                r = json.loads(line)
            except ValueError:
                continue                                # torn tail — C++가 append 중
            if r.get("crc") != 1 or r.get("ty") not in POSITION_TYPES:
                continue
            lat, lon, mmsi = r.get("lat"), r.get("lon"), r.get("mmsi", 0)
            if lat is None or lon is None or not mmsi:
                continue
            if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
                continue                                # 91/181 = AIS n/a
            if lat == 0.0 and lon == 0.0:
                continue
            yield (int(r.get("t", 0)), int(mmsi), float(lat), float(lon),
                   float(r.get("sog", -1.0)), float(r.get("cog", -1.0)))


def _key(lat: float, lon: float) -> int:
    return (int(math.floor(lat / CELL)) + 20000) * 100000 \
         + (int(math.floor(lon / CELL)) + 40000)


class GridStats:
    """셀별 통계 누산기. 셀 값 = [count, sog_n, sog_sum, sog_sq, cog_hist(8)]."""

    def __init__(self):
        self.cells = {}                                 # key(int) -> float64[12]

    def add(self, lat, lon, sog, cog):
        c = self.cells.get(_key(lat, lon))
        if c is None:
            c = self.cells[_key(lat, lon)] = np.zeros(12, np.float64)
        c[0] += 1
        if sog >= 0.0:
            c[1] += 1; c[2] += sog; c[3] += sog * sog
        if 0.0 <= cog < 360.0:
            c[4 + int(cog / 45.0) % 8] += 1

    def build(self, day_paths):
        """ais_*.jsonl 경로들에서 통계 누적. 누적된 위치보고 수를 반환."""
        n = 0
        for path in day_paths:
            for _t, _mmsi, lat, lon, sog, cog in iter_positions(path):
                self.add(lat, lon, sog, cog)
                n += 1
        return n

    def score_point(self, lat, lon, sog, cog) -> float:
        """관측치 희귀도 [0,1]. 1 = 전례 없는 해역/거동, 0 = 흔함."""
        c = self.cells.get(_key(lat, lon))
        if c is None:
            return 1.0                                  # 통행 이력 없는 셀
        parts = [(0.5, 1.0 - min(1.0, math.log1p(c[0]) / _REF_COUNT))]
        if sog >= 0.0 and c[1] >= _MIN_STAT:
            mean = c[2] / c[1]
            std = max(math.sqrt(max(c[3] / c[1] - mean * mean, 0.0)), 0.5)
            parts.append((0.3, min(1.0, abs(sog - mean) / std / 4.0)))
        if 0.0 <= cog < 360.0:
            hist = c[4:12]
            tot = hist.sum()
            if tot >= _MIN_STAT:
                p = hist[int(cog / 45.0) % 8] / tot
                parts.append((0.2, min(1.0, max(0.0, 1.0 - 8.0 * p))))
        w = sum(p[0] for p in parts)
        return float(sum(wi * ri for wi, ri in parts) / w)

    def save(self, path: str = GRID_PATH):
        os.makedirs(os.path.dirname(path), exist_ok=True)
        keys = np.fromiter(self.cells.keys(), np.int64, len(self.cells))
        vals = (np.stack(list(self.cells.values()))
                if self.cells else np.zeros((0, 12), np.float64))
        tmp = path + ".tmp.npz"                         # .npz로 끝나야 자동 확장자 방지
        np.savez_compressed(tmp, keys=keys, vals=vals)
        os.replace(tmp, path)

    @classmethod
    def load(cls, path: str = GRID_PATH):
        z = np.load(path)
        g = cls()
        g.cells = {int(k): v for k, v in zip(z["keys"], z["vals"])}
        return g
