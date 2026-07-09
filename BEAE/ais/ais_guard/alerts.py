"""AlertBook — 경보 상태머신(NEW→UPDATE→CLEAR) + JSONL 발행.

발행처: alerts_live.jsonl (C++ tail) + alerts_YYYYMMDD.jsonl (일별 보관).
aid = crc32("typ:mmsi:mmsi2") — 같은 사건은 같은 aid로 UPDATE/CLEAR.
해제는 연속 clear_ticks 틱 미관측 시(히스테리시스).
"""
import json
import os
import time
import zlib

from .tracker import kst_day

TYP_COLLISION, TYP_GROUNDING, TYP_ANOMALY, TYP_THREAT, TYP_REPORT = 1, 2, 3, 4, 5
ST_NEW, ST_UPDATE, ST_CLEAR = 1, 2, 3

TYP_NAME = {1: "충돌위험", 2: "좌초위험", 3: "이상항적", 4: "위협선박", 5: "일일보고"}
SEV_NAME = {1: "주의", 2: "경고", 3: "긴급"}


def make_aid(typ: int, mmsi: int, mmsi2: int = 0) -> int:
    return zlib.crc32(f"{typ}:{mmsi}:{mmsi2}".encode())


def trunc_utf8(s: str, nbytes: int) -> str:
    """UTF-8 경계 안전 바이트 절단 (C++ char[] 계약)."""
    b = s.encode("utf-8")
    if len(b) <= nbytes:
        return s
    b = b[:nbytes]
    while b and (b[-1] & 0xC0) == 0x80:                 # 멀티바이트 중간이면 후퇴
        b = b[:-1]
    return b.decode("utf-8", "ignore")


def append_alert(cfg, doc: dict):
    """계약 JSON 한 줄을 live + 일별 파일에 append."""
    os.makedirs(cfg.guard_dir, exist_ok=True)
    line = json.dumps(doc, ensure_ascii=False, separators=(",", ":")) + "\n"
    daily = os.path.join(cfg.guard_dir, f"alerts_{kst_day(doc['t'] / 1000.0)}.jsonl")
    for path in (cfg.alerts_live_path, daily):
        with open(path, "a", encoding="utf-8") as f:
            f.write(line)


def make_doc(t_ms, aid, typ, sev, st, mmsi, mmsi2, lat, lon, score, cpa, tcpa, msg, reco):
    return {"v": 1, "t": int(t_ms), "aid": int(aid), "typ": int(typ), "sev": int(sev),
            "st": int(st), "mmsi": int(mmsi), "mmsi2": int(mmsi2),
            "lat": round(float(lat), 6), "lon": round(float(lon), 6),
            "score": round(float(score), 1), "cpa": round(float(cpa), 1),
            "tcpa": round(float(tcpa), 1),
            "msg": trunc_utf8(msg, 95), "reco": trunc_utf8(reco, 127)}


class AlertBook:
    def __init__(self, cfg):
        self.cfg = cfg
        self.active: dict[int, dict] = {}               # aid → 사건 상태
        self.tick = 0
        self.n_emitted = 0
        os.makedirs(cfg.guard_dir, exist_ok=True)

    def observe(self, typ, sev, mmsi, mmsi2, lat, lon, score, cpa, tcpa, msg, reco):
        """이번 틱에 조건이 성립한 사건 보고. NEW/UPDATE 발행은 내부에서 판단."""
        aid = make_aid(typ, mmsi, mmsi2)
        e = self.active.get(aid)
        is_new = e is None
        if is_new:
            e = self.active[aid] = {"typ": typ, "mmsi": mmsi, "mmsi2": mmsi2,
                                    "last_emit": 0.0, "emit_sev": 0, "emit_score": -1e9}
        e.update(tick=self.tick, miss=0, sev=sev, score=score, lat=lat, lon=lon,
                 cpa=cpa, tcpa=tcpa, msg=msg, reco=reco)
        now = time.time()
        if (is_new or sev != e["emit_sev"]                       # 심각도 변화
                or abs(score - e["emit_score"]) >= self.cfg.update_score_delta
                or now - e["last_emit"] >= self.cfg.update_min_interval_s):
            self._emit(aid, e, ST_NEW if is_new else ST_UPDATE)

    def end_tick(self):
        """틱 마감: 미관측 사건 miss 누적, clear_ticks 도달 시 CLEAR."""
        for aid in list(self.active):
            e = self.active[aid]
            if e.get("tick") == self.tick:
                continue
            e["miss"] = e.get("miss", 0) + 1
            if e["miss"] >= self.cfg.clear_ticks:
                self._emit(aid, e, ST_CLEAR)
                del self.active[aid]
        self.tick += 1

    def _emit(self, aid, e, st):
        doc = make_doc(int(time.time() * 1000), aid, e["typ"], e["sev"], st,
                       e["mmsi"], e["mmsi2"], e.get("lat", 0.0), e.get("lon", 0.0),
                       e.get("score", 0.0), e.get("cpa", -1.0), e.get("tcpa", -1.0),
                       e.get("msg", ""), e.get("reco", ""))
        append_alert(self.cfg, doc)
        e["last_emit"] = time.time()
        e["emit_sev"] = e["sev"]
        e["emit_score"] = e["score"]
        self.n_emitted += 1

    def write_status(self, extra: dict | None = None):
        """guard_status.json 원자적 갱신."""
        doc = {"pid": os.getpid(), "time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
               "tick": self.tick, "alerts_active": len(self.active),
               "alerts_emitted": self.n_emitted,
               "active": [{"aid": aid, "typ": e["typ"], "sev": e["sev"],
                           "mmsi": e["mmsi"], "mmsi2": e["mmsi2"],
                           "score": round(e.get("score", 0.0), 1)}
                          for aid, e in self.active.items()]}
        if extra:
            doc.update(extra)
        tmp = self.cfg.status_path + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(doc, f, ensure_ascii=False, indent=1)
        os.replace(tmp, self.cfg.status_path)
