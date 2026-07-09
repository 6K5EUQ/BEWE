"""일일 보고서 report_YYYYMMDD.md 생성 + REPORT 경보 1건 발행.

소스: 같은 날 alerts_YYYYMMDD.jsonl (AlertBook가 남긴 발행 로그).
"""
import json
import os
import time
from collections import Counter
from datetime import datetime, timedelta, timezone

from .alerts import (ST_NEW, SEV_NAME, TYP_NAME, TYP_REPORT,
                     append_alert, make_aid, make_doc)
from .tracker import KST, kst_day


def _load(cfg, day: str) -> list[dict]:
    path = os.path.join(cfg.guard_dir, f"alerts_{day}.jsonl")
    out = []
    try:
        with open(path, encoding="utf-8") as f:
            for line in f:
                try:
                    out.append(json.loads(line))
                except ValueError:
                    pass
    except OSError:
        pass
    return out


def _hhmm(t_ms: int) -> str:
    return datetime.fromtimestamp(t_ms / 1000.0, KST).strftime("%H:%M")


def generate(cfg, day: str | None = None, emit: bool = True) -> str:
    """report_YYYYMMDD.md 작성, emit=True면 REPORT 경보도 발행. 경로 반환."""
    day = day or kst_day()
    alerts = [a for a in _load(cfg, day) if a.get("typ") != TYP_REPORT]
    news = [a for a in alerts if a.get("st") == ST_NEW]
    by_typ = Counter(a["typ"] for a in news)
    by_sev = Counter(a["sev"] for a in news)
    ships: dict[int, dict] = {}                          # mmsi → 요약
    for a in alerts:
        for m in (a.get("mmsi", 0), a.get("mmsi2", 0)):
            if not m:
                continue
            s = ships.setdefault(m, {"n": 0, "max_score": 0.0, "typs": set(), "last_msg": ""})
            if a.get("st") == ST_NEW:
                s["n"] += 1
            s["max_score"] = max(s["max_score"], float(a.get("score", 0.0)))
            s["typs"].add(a["typ"])
            s["last_msg"] = a.get("msg", s["last_msg"])

    d = f"{day[:4]}-{day[4:6]}-{day[6:]}"
    L = [f"# BEWE GUARD 일일 보고서 — {d}", "",
         f"- 생성: {time.strftime('%Y-%m-%dT%H:%M:%S%z')}",
         f"- 경보 발생 {len(news)}건 (심각 {by_sev.get(3, 0)} / 경고 {by_sev.get(2, 0)} / "
         f"정보 {by_sev.get(1, 0)}), 발행 라인 {len(alerts)}줄, 관련 선박 {len(ships)}척", "",
         "## 유형별 발생", "", "| 유형 | 건수 |", "|---|---|"]
    for typ in sorted(TYP_NAME):
        if typ == TYP_REPORT:
            continue
        L.append(f"| {TYP_NAME[typ]} | {by_typ.get(typ, 0)} |")
    L += ["", "## 타임라인 (발생 기준, 최대 100건)", ""]
    if news:
        L += ["| 시각 | 심각도 | 유형 | 내용 |", "|---|---|---|---|"]
        for a in news[:100]:
            L.append(f"| {_hhmm(a['t'])} | {SEV_NAME.get(a['sev'], '?')} "
                     f"| {TYP_NAME.get(a['typ'], '?')} | {a.get('msg', '')} |")
    else:
        L.append("(경보 없음)")
    L += ["", "## 선박별 요약", ""]
    if ships:
        L += ["| MMSI | 경보수 | 최고점수 | 유형 | 마지막 메시지 |", "|---|---|---|---|---|"]
        for m, s in sorted(ships.items(), key=lambda kv: -kv[1]["max_score"]):
            typs = "/".join(TYP_NAME[t] for t in sorted(s["typs"]))
            L.append(f"| {m} | {s['n']} | {s['max_score']:.0f} | {typs} | {s['last_msg']} |")
    else:
        L.append("(해당 선박 없음)")
    L += ["", "## 권고 로그", ""]
    recos = sorted({a.get("reco", "") for a in news if a.get("reco")})
    L += [f"- {r}" for r in recos] or ["(권고 없음)"]
    L.append("")

    os.makedirs(cfg.guard_dir, exist_ok=True)
    path = os.path.join(cfg.guard_dir, f"report_{day}.md")
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        f.write("\n".join(L))
    os.replace(tmp, path)

    if emit:
        msg = (f"{d} 일일요약: 경보 {len(news)}건 "
               f"(긴급 {by_sev.get(3, 0)}/경고 {by_sev.get(2, 0)}), 선박 {len(ships)}척")
        doc = make_doc(int(time.time() * 1000), make_aid(TYP_REPORT, int(day), 0),
                       TYP_REPORT, 1, ST_NEW, 0, 0, 0.0, 0.0,
                       float(len(news)), -1.0, -1.0, msg, path)
        append_alert(cfg, doc)
    return path


def yesterday(day: str) -> str:
    """KST 기준 전날 YYYYMMDD."""
    dt = datetime.strptime(day, "%Y%m%d").replace(tzinfo=KST) - timedelta(days=1)
    return dt.strftime("%Y%m%d")
