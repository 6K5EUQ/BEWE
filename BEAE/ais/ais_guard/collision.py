"""충돌(CPA/TCPA)·좌초(위험구역) 물리 판정.

zones.json: {"zones":[{"name":str,"type":"reef|shallow|forbidden",
                       "poly":[[lat,lon],...]}]}  (zones_example.json 참고)
predict_fn(mmsi) -> [(lat,lon),...] (cfg.predict_step_s 간격 미래점) 또는 None —
py-dl 궤적 예측기가 있으면 등속 가정보다 우선한다.
"""
import json
import math
import os

KN2MS = 0.514444
M_PER_DEG_LAT = 111320.0                                # 위도 1도 ≈ 111.32 km


# ── 좌표/속도 ────────────────────────────────────────────────────────────────
def to_local(lat, lon, ref_lat, ref_lon):
    """위경도 → 기준점 로컬 평면 m (등장방형 근사, x=동 y=북)."""
    kx = M_PER_DEG_LAT * math.cos(math.radians(ref_lat))
    return ((lon - ref_lon) * kx, (lat - ref_lat) * M_PER_DEG_LAT)


def vel_ms(sog_kn, cog_deg, nav=None):
    """SOG/COG → (vx,vy) m/s. 저속(<1kt)·정박(nav 1)·계류(nav 5)는 COG 신뢰불가 → 정지."""
    if sog_kn is None or sog_kn < 1.0 or cog_deg is None or cog_deg < 0 or cog_deg >= 360:
        return (0.0, 0.0)
    if nav in (1, 5):
        return (0.0, 0.0)
    v = sog_kn * KN2MS
    r = math.radians(cog_deg)
    return (v * math.sin(r), v * math.cos(r))


def cpa_tcpa(p1, v1, p2, v2):
    """등속 가정 최근접. 반환 (cpa_m, tcpa_s); 상대속도≈0이면 tcpa=-1."""
    dx, dy = p2[0] - p1[0], p2[1] - p1[1]
    dvx, dvy = v2[0] - v1[0], v2[1] - v1[1]
    dv2 = dvx * dvx + dvy * dvy
    if dv2 < 1e-6:
        return math.hypot(dx, dy), -1.0
    t = -(dx * dvx + dy * dvy) / dv2
    if t < 0:
        t = 0.0                                         # 이미 멀어지는 중 → 현재가 최근접
    return math.hypot(dx + dvx * t, dy + dvy * t), t


# ── 선박쌍 충돌 후보 ─────────────────────────────────────────────────────────
def check_pairs(cfg, active, predict_fn=None):
    """활성 트랙 쌍 스캔 → [(trA, trB, cpa_m, tcpa_s, lat, lon)] 경보 후보."""
    out = []
    trs = [tr for tr in active.values() if tr.latest()]
    for i in range(len(trs)):
        for j in range(i + 1, len(trs)):
            a, b = trs[i], trs[j]
            _, la1, lo1, sog1, cog1 = a.latest()
            _, la2, lo2, sog2, cog2 = b.latest()
            v1 = vel_ms(sog1, cog1, getattr(a, "nav", None))
            v2 = vel_ms(sog2, cog2, getattr(b, "nav", None))
            relsp = math.hypot(v2[0] - v1[0], v2[1] - v1[1])
            if relsp < 0.8:                             # 상대속도 <~1.5kt: 평행·동속·정지 → 무시
                continue
            p2 = to_local(la2, lo2, la1, lo1)
            if math.hypot(*p2) > cfg.pair_max_dist_m:
                continue
            cpa = tcpa = None
            if predict_fn is not None:                  # 예측궤적 CPA 우선
                pa, pb = predict_fn(a.mmsi), predict_fn(b.mmsi)
                if pa and pb:
                    best_d, best_k = float("inf"), 0
                    for k in range(min(len(pa), len(pb))):
                        q1 = to_local(pa[k][0], pa[k][1], la1, lo1)
                        q2 = to_local(pb[k][0], pb[k][1], la1, lo1)
                        d = math.hypot(q2[0] - q1[0], q2[1] - q1[1])
                        if d < best_d:
                            best_d, best_k = d, k
                    cpa, tcpa = best_d, (best_k + 1) * cfg.predict_step_s
            if cpa is None:
                cpa, tcpa = cpa_tcpa((0.0, 0.0), v1, p2, v2)
            if tcpa < 0 or tcpa > cfg.tcpa_max_s or cpa > cfg.cpa_warn_m:
                continue                                # 멀어지는 중/먼 미래/충분히 먼 최근접
            # 경보 지점 = 최근접 순간 두 배 위치 중점
            aL = (la1 + v1[1] * tcpa / M_PER_DEG_LAT, lo1 + v1[0] * tcpa / (M_PER_DEG_LAT * math.cos(math.radians(la1))))
            bL = (la2 + v2[1] * tcpa / M_PER_DEG_LAT, lo2 + v2[0] * tcpa / (M_PER_DEG_LAT * math.cos(math.radians(la2))))
            out.append((a, b, cpa, tcpa, (aL[0] + bL[0]) / 2, (aL[1] + bL[1]) / 2))
    return out


# ── 위험구역 ─────────────────────────────────────────────────────────────────
def load_zones(path):
    """zones.json → [{"name","type","poly":[(lat,lon),...]}]. 없거나 깨지면 []."""
    try:
        with open(path, encoding="utf-8") as f:
            doc = json.load(f)
    except (OSError, ValueError):
        return []
    zones = []
    for z in doc.get("zones", []):
        poly = [(float(p[0]), float(p[1])) for p in z.get("poly", [])]
        if len(poly) >= 3:
            zones.append({"name": str(z.get("name", "?")),
                          "type": str(z.get("type", "forbidden")), "poly": poly})
    return zones


def point_in_poly(lat, lon, poly):
    """레이 캐스팅 point-in-polygon."""
    inside = False
    n = len(poly)
    for i in range(n):
        la1, lo1 = poly[i]
        la2, lo2 = poly[(i + 1) % n]
        if (la1 > lat) != (la2 > lat):
            x = lo1 + (lat - la1) / (la2 - la1) * (lo2 - lo1)
            if lon < x:
                inside = not inside
    return inside


def check_zones(cfg, active, zones, predict_fn=None):
    """트랙별 최근접 위험구역 → [(tr, zone, eta_s, lat, lon)]. eta=0이면 이미 내부."""
    if not zones:
        return []
    out = []
    for tr in active.values():
        latest = tr.latest()
        if latest is None:
            continue
        _, lat, lon, sog, cog = latest
        hit = None                                      # (eta, zone, plat, plon)
        for z in zones:
            if point_in_poly(lat, lon, z["poly"]):
                hit = (0.0, z, lat, lon)
                break
        if hit is None:
            pts = _future_points(cfg, tr, predict_fn)
            for eta, plat, plon in pts:
                for z in zones:
                    if point_in_poly(plat, plon, z["poly"]):
                        if hit is None or eta < hit[0]:
                            hit = (eta, z, plat, plon)
                        break
                if hit is not None:
                    break                               # 최초 진입점이 최근접
        if hit is not None:
            out.append((tr, hit[1], hit[0], hit[2], hit[3]))
    return out


def _future_points(cfg, tr, predict_fn):
    """[(eta_s, lat, lon)] 미래 위치 — 예측기 우선, 없으면 등속 외삽."""
    if predict_fn is not None:
        pts = predict_fn(tr.mmsi)
        if pts:
            return [((k + 1) * cfg.predict_step_s, p[0], p[1])
                    for k, p in enumerate(pts) if (k + 1) * cfg.predict_step_s <= cfg.zone_horizon_s]
    _, lat, lon, sog, cog = tr.latest()
    vx, vy = vel_ms(sog, cog)
    if vx == 0.0 and vy == 0.0:
        return []
    ky = 1.0 / M_PER_DEG_LAT
    kx = 1.0 / (M_PER_DEG_LAT * math.cos(math.radians(lat)))
    out = []
    t = cfg.zone_step_s
    while t <= cfg.zone_horizon_s:
        out.append((t, lat + vy * t * ky, lon + vx * t * kx))
        t += cfg.zone_step_s
    return out
