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


def emit_path(cfg, aid, typ, kind, name, pts, clear=False):
    """경로 1개를 청크 분할 발행. pts=[(lat,lon),...]. clear=True 면 제거만."""
    t = _now_ms()
    if clear:
        append_alert(cfg, make_doc(t, aid, typ, kind, 3, 0, 1, 0, 0, 0, -1, -1, "", ""))
        return 1
    chunks = [pts[i:i + _PTS_PER_CHUNK] for i in range(0, len(pts), _PTS_PER_CHUNK)]
    for i, ch in enumerate(chunks):
        packed = ";".join(f"{la:.5f},{lo:.5f}" for la, lo in ch)
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


# ── 데모: 탐지 유형별 예시 상황 (DGS-1/DGS-4 수신권 — 부산·마산 근해) ──────────
# 가상 MMSI 999xxxxxx (실선박과 절대 안 겹침). 항적은 typ=7, 경보는 typ 1~4 실제 포맷.
def _demo_items(cfg):
    it = []
    # 1) 충돌위험: 두 배가 35.020N 128.700E 로 수렴 (교차각 ~90°)
    a = [(34.990 + 0.006 * i, 128.660 + 0.008 * i) for i in range(6)]   # NE 진행
    b = [(35.055 - 0.007 * i, 128.668 + 0.007 * i) for i in range(6)]   # SE 진행
    it.append(("path", 999000001, "[예시] MU-DONG",  a))
    it.append(("path", 999000002, "[예시] SE-JIN 7", b))
    it.append(("alert", 1, 3, 999000001, 999000002, a[-1][0], a[-1][1], 92.0, 140.0, 210.0,
               "[예시] 두 선박 수렴 — CPA 140m/3.5분", "즉시 VHF 호출·변침 권고. 우현 변침 유도, 접근 지속 시 긴급경보."))
    # 2) 좌초위험: 예시 암초 구역(35.08~35.10N 128.60~128.62E)으로 직진
    c = [(35.040 + 0.009 * i, 128.640 - 0.005 * i) for i in range(6)]
    it.append(("path", 999000003, "[예시] HAE-RIM", c))
    it.append(("alert", 2, 2, 999000003, 0, c[-1][0], c[-1][1], 78.0, -1.0, 300.0,
               "[예시] 암초 구역 접근 — 진입까지 약 5분", "위험구역 통보·항로 변경 지시. 5분 내 미변침 시 긴급 단계 격상."))
    # 3) 이상항적: 갈지자+루프 (평소 항로 학습 대비 예측오차 급증)
    d = [(35.115, 128.760), (35.122, 128.775), (35.110, 128.782), (35.121, 128.792),
         (35.108, 128.796), (35.118, 128.806), (35.105, 128.802), (35.112, 128.788)]
    it.append(("path", 999000004, "[예시] BADA 21", d))
    it.append(("alert", 3, 2, 999000004, 0, d[-1][0], d[-1][1], 88.0, -1.0, -1.0,
               "[예시] 이상 항적 — 학습 항로 대비 예측오차 88점", "조타 불능·표류/의도적 배회 가능성. 교신 시도 및 지속 관찰."))
    # 4) 위협선박: 정상 직선 항적이나 RF 지문이 신고 MMSI 와 불일치
    e = [(34.975 + 0.004 * i, 128.820 - 0.006 * i) for i in range(6)]
    it.append(("path", 999000005, "[예시] UNKNOWN-ID", e))
    it.append(("alert", 4, 3, 999000005, 0, e[-1][0], e[-1][1], 95.0, -1.0, -1.0,
               "[예시] RF지문-MMSI 불일치 6/6 버스트 — 위장 의심", "AIS 신원 위장 의심. 타 센서 교차확인·해경 통보 검토."))
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
            emit_path(cfg, _demo_aid("P", mmsi), TYP_PATH, 1, name, pts, clear=clear)
            n += 1
        else:
            _, typ, sev, mmsi, mmsi2, lat, lon, score, cpa, tcpa, msg, reco = item
            aid = _demo_aid("A", f"{typ}:{mmsi}")
            append_alert(cfg, make_doc(t, aid, typ, sev, 3 if clear else 1, mmsi, mmsi2,
                                       lat, lon, score, cpa, tcpa,
                                       "" if clear else msg, "" if clear else reco))
            n += 1
    return n
