"""해안선(육지) 판정 — GUARD 충돌 오탐(두 배 사이 육지) 배제용.

C++ 지도와 동일 소스 src/korea_osm_data.hpp(KR_OSM_COAST) 를 파싱해 bbox(마산·진해 근해)
닫힌 링만 캐시(.npz). even-odd 레이캐스팅 point_on_land + 선분 표본 seg_crosses_land.
최초 1회 파싱(수 초) 후 npz 재사용. numpy 만 사용.
"""
import os
import re

import numpy as np

# 전체 링을 써야 even-odd 레이캐스팅(동쪽 무한대) parity 가 정확 — bbox 로 링 자르면 안 됨.
# lat 오름차순 정렬 + 검사 시 lat 밴드만 훑어 속도 확보. (C++ map_view fill_edges 와 동일)
_CACHE = os.path.join(os.path.dirname(__file__), "data", "coastline_kr.npz")
_HDR = os.path.normpath(os.path.join(os.path.dirname(__file__), "..", "..", "..", "src", "korea_osm_data.hpp"))

_edges = None   # (N,4) [latlo, lathi, lonlo, slope], latlo 오름차순


def _build():
    txt = open(_HDR).read()
    body = txt[txt.find("KR_OSM_COAST[]"):]
    toks = re.findall(r"-?\d+\.\d+|NAN", body)
    vals = [float("nan") if t == "NAN" else float(t) for t in toks]
    arr = np.array(vals[: len(vals) // 2 * 2]).reshape(-1, 2)
    edges = []
    ring = []
    def flush():
        if len(ring) >= 3:
            a = np.array(ring)
            if abs(a[0, 0] - a[-1, 0]) < 0.002 and abs(a[0, 1] - a[-1, 1]) < 0.002:
                n = len(a)                              # 전체 링 유지 (레이캐스팅 parity 정확)
                for i in range(n):
                    la1, lo1 = a[i]
                    la2, lo2 = a[(i + 1) % n]
                    if la1 == la2:
                        continue
                    if la1 > la2:
                        la1, la2, lo1, lo2 = la2, la1, lo2, lo1
                    edges.append((la1, la2, lo1, (lo2 - lo1) / (la2 - la1)))
        ring.clear()
    for la, lo in arr:
        if np.isnan(la):
            flush()
        else:
            ring.append((la, lo))
    flush()
    E = np.array(edges, dtype=np.float64)
    E = E[E[:, 0].argsort()]
    os.makedirs(os.path.dirname(_CACHE), exist_ok=True)
    np.savez(_CACHE, edges=E)
    return E


def _load():
    global _edges
    if _edges is not None:
        return _edges
    try:
        if os.path.getmtime(_CACHE) >= os.path.getmtime(_HDR):
            _edges = np.load(_CACHE)["edges"]
            return _edges
    except OSError:
        pass
    try:
        _edges = _build()
    except OSError:
        _edges = np.zeros((0, 4))       # 헤더 없으면 판정 비활성(항상 물)
    return _edges


def point_on_land(lat, lon):
    E = _load()
    if len(E) == 0:
        return False
    m = (E[:, 0] <= lat) & (lat < E[:, 1])
    if not m.any():
        return False
    lon_at = E[m, 2] + E[m, 3] * (lat - E[m, 0])
    return int((lon_at > lon).sum()) & 1 == 1


def seg_crosses_land(la1, lo1, la2, lo2):
    E = _load()
    if len(E) == 0:
        return False
    n = max(4, int(max(abs(la2 - la1), abs(lo2 - lo1)) / 0.0015) + 1)
    for i in range(1, n):
        t = i / n
        if point_on_land(la1 + (la2 - la1) * t, lo1 + (lo2 - lo1) * t):
            return True
    return False
