"""위협선박 판정 — Match_AI RF 지문 불일치 + CFO/RSSI 정체성 점프 보조 규칙.

불일치 = aist==2 && aim!=mmsi (AI가 '다른 송신기'로 판정한 버스트).
"""
from collections import Counter


def rf_jumps(cfg, tr) -> int:
    """동일 MMSI 연속 버스트 간 CFO/RSSI 급변 횟수 — RF 정체성 점프 의심."""
    n = 0
    prev = None
    for t_ms, cfo, rssi in tr.rf:
        if prev is not None and (abs(cfo - prev[0]) > cfg.cfo_jump_hz
                                 or abs(rssi - prev[1]) > cfg.rssi_jump_db):
            n += 1
        prev = (cfo, rssi)
    return n


def check(cfg, tr):
    """위협이면 (score, n_mis, n_tot, aim_top, n_jump), 아니면 None."""
    buf = list(tr.match)
    if len(buf) < cfg.threat_min_bursts:
        return None
    mis = [aim for aist, aim in buf if aist == 2 and aim and aim != tr.mmsi]
    ratio = len(mis) / len(buf)
    if ratio < cfg.threat_ratio:
        return None
    aim_top = Counter(mis).most_common(1)[0][0]         # 가장 유력한 실제 송신기
    n_jump = rf_jumps(cfg, tr)
    score = min(100.0, ratio * 100.0 + (10.0 if n_jump else 0.0))
    return score, len(mis), len(buf), aim_top, n_jump
