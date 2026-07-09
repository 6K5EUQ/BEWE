"""지도 오버레이 발행 — 위험구역 폴리곤(typ=6) + 데모/가상 항적(typ=7).

경로 꼭짓점은 reco 필드에 "lat,lon;lat,lon;..." 텍스트 패킹(레코드당 ≤6점),
긴 경로는 청크 분할: mmsi=청크 idx, mmsi2=총 청크 수, 이름(msg)은 청크 0에만.
C++ guard_module 이 wire 로 JOIN 에 전파 → ais 지도에 폴리곤/점선 항적으로 렌더.
state=3(CLEAR) 발행 시 해당 aid 오버레이 제거.
"""
import time
import zlib

from .alerts import append_alert, make_doc, trunc_utf8
from .collision import load_zones

TYP_ZONE, TYP_PATH = 6, 7
_PTS_PER_CHUNK = 6                       # "%.5f,%.5f;" ≈ 19B × 6 = 114B ≤ reco 127B

# zones.json type → kind(sev 재활용): 시각 색상 단계 (1=주의 2=위험 3=금지)
_ZONE_KIND = {"reef": 2, "shallow": 2, "forbidden": 3}


def _now_ms():
    return int(time.time() * 1000)


def emit_path(cfg, aid, typ, kind, name, pts, clear=False, vmmsi=0):
    """경로 1개를 청크 분할 발행. pts=[(lat,lon),...]. clear=True 면 제거만.
    vmmsi: 항적 소유 (가상)MMSI — 청크0 선두 "M<mmsi>;" 토큰 (경보↔항적 연결)."""
    t = _now_ms()
    if clear:
        append_alert(cfg, make_doc(t, aid, typ, kind, 3, 0, 1, 0, 0, 0, -1, -1, "", ""))
        return 1
    chunks = [pts[i:i + _PTS_PER_CHUNK] for i in range(0, len(pts), _PTS_PER_CHUNK)]
    for i, ch in enumerate(chunks):
        packed = ";".join(f"{la:.5f},{lo:.5f}" for la, lo in ch)
        if i == 0 and vmmsi:
            packed = f"M{vmmsi};" + packed
        append_alert(cfg, make_doc(t, aid, typ, kind, 1, i, len(chunks),
                                   ch[0][0], ch[0][1], 0, -1, -1,
                                   trunc_utf8(name, 38) if i == 0 else "", packed))
    return len(chunks)


def publish_zones(cfg, log=None):
    """zones.json 전체를 typ=6 으로 발행 (데몬 시작/구역 변경 시)."""
    zones = load_zones(cfg.zones_path)
    for z in zones:
        aid = zlib.crc32(f"Z:{z['name']}".encode())
        emit_path(cfg, aid, TYP_ZONE, _ZONE_KIND.get(z["type"], 3), z["name"], z["poly"])
    if log:
        log.info("zones published: %d", len(zones))
    return len(zones)


# ── 데모: 탐지 유형별 예시 상황 (진해만·가덕수로 — 실측 선박 위치 기준 水域) ──
# 가상 MMSI 999xxxxxx. 라벨/문구는 짧은 영어 (GUI 폰트에 한글 글리프 없음).
def _demo_items(cfg):
    it = []
    # 1) COLLISION: 두 항로 연장선 교점 = (35.1200, 128.6533) — 머리들은 교점 못미침
    a = [(35.096 + 0.005 * i, 128.612 + 0.0086 * i) for i in range(5)]   # NE 진행
    b = [(35.150 - 0.0055 * i, 128.625 + 0.0052 * i) for i in range(6)]  # SE 진행
    it.append(("path", 999000001, "DEMO-A", a))
    it.append(("path", 999000002, "DEMO-B", b))
    it.append(("alert", 1, 3, 999000001, 999000002, 35.1200, 128.6533, 92.0, 140.0, 210.0,
               "CPA 140m 3.5분 후: DEMO-A x DEMO-B", "양 선박 VHF16 호출, 우현 변침 지시"))
    # 2) GROUNDING: REEF-A(35.055~070N 128.640~662E, 진해만 남측 수역)로 남진
    c = [(35.108 - 0.006 * i, 128.648) for i in range(6)]
    it.append(("path", 999000003, "DEMO-C", c))
    it.append(("alert", 2, 2, 999000003, 0, 35.070, 128.648, 78.0, -1.0, 300.0,
               "REEF-A 5분 내 진입: DEMO-C", "즉시 변침 지시, 암초 회피"))
    # 3) ANOMALY: 가덕 서측 수역(35.10N 128.74E) 갈지자
    d = [(35.093, 128.735), (35.100, 128.744), (35.091, 128.750), (35.101, 128.757),
         (35.092, 128.761), (35.100, 128.766), (35.091, 128.760), (35.097, 128.752)]
    it.append(("path", 999000004, "DEMO-D", d))
    it.append(("alert", 3, 2, 999000004, 0, d[-1][0], d[-1][1], 88.0, -1.0, -1.0,
               "이상항적 88점: DEMO-D", "항적 감시, VHF 호출"))
    # 4) SPOOF: 가덕수로(35.04~056N 128.79~80E) 북서진, RF 지문 불일치
    e = [(35.040 + 0.0032 * i, 128.802 - 0.0032 * i) for i in range(6)]
    it.append(("path", 999000005, "DEMO-E", e))
    it.append(("alert", 4, 3, 999000005, 0, e[-1][0], e[-1][1], 95.0, -1.0, -1.0,
               "RF지문 불일치 6/6: DEMO-E", "MMSI 위장 의심 — VHF·레이더 확인"))
    return it


def _demo_aid(kind, key):
    return zlib.crc32(f"D:{kind}:{key}".encode())


def demo(cfg, clear=False):
    """예시 상황 4종 발행/제거. clear=True 면 전부 CLEAR."""
    n = 0
    t = _now_ms()
    for item in _demo_items(cfg):
        if item[0] == "path":
            _, mmsi, name, pts = item
            emit_path(cfg, _demo_aid("P", mmsi), TYP_PATH, 1, name, pts, clear=clear, vmmsi=mmsi)
            n += 1
        else:
            _, typ, sev, mmsi, mmsi2, lat, lon, score, cpa, tcpa, msg, reco = item
            aid = _demo_aid("A", f"{typ}:{mmsi}")
            append_alert(cfg, make_doc(t, aid, typ, sev, 3 if clear else 1, mmsi, mmsi2,
                                       lat, lon, score, cpa, tcpa,
                                       "" if clear else msg, "" if clear else reco))
            n += 1
    return n
