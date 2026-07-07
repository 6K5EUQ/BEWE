"""Human-readable status: training-data accumulation + model/daemon state.
Used by `python -m ais_ai status` and the /bewe-ai ais status skill."""
import datetime
import json
import os

from .aicap import data_summary
from .config import Config


def _kst(t_ms):
    if not t_ms:
        return "-"
    return datetime.datetime.fromtimestamp(t_ms / 1000).strftime("%m-%d %H:%M")


def _mb(b):
    return f"{b / 1e6:.1f} MB"


def render(cfg: Config, days: int) -> str:
    s = data_summary(cfg.data_dir, days, cfg.min_class)
    lo, hi = s["t_lo_ms"], s["t_hi_ms"]
    span_min = round((hi - lo) / 60000) if (lo and hi) else 0
    rate = round(s["total_records"] / span_min) if span_min else 0

    L = []
    L.append("=== 학습데이터 현황 (최근 %d일, aicap %d파일) ===" % (days, s["n_files"]))
    L.append("항목\t값")
    L.append("수집 시작\t%s" % _kst(lo))
    L.append("현재\t%s" % _kst(hi))
    L.append("경과\t%d분" % span_min)
    L.append("총 버스트\t%s" % f"{s['total_records']:,}")
    L.append("선박(MMSI)\t%d척" % s["distinct_mmsi"])
    L.append("수집 속도\t~%d버스트/분" % rate)
    L.append("총 용량\t%s" % _mb(s["total_bytes"]))
    L.append("%d버스트↑ 선박\t%d척 (학습 최소 2척)" % (s["min_class"], s["n_ready"]))

    if s["n_files"] > 1:
        L.append("")
        L.append("=== 일자별 ===")
        L.append("날짜\t버스트\t용량")
        for day, rec, byt in s["per_day"]:
            L.append("%s\t%s\t%s" % (day, f"{rec:,}", _mb(byt)))

    L.append("")
    L.append("=== 상위 선박 (>=%d = 학습대상[OK]) ===" % s["min_class"])
    L.append("MMSI\t버스트")
    for m, n in s["top_mmsi"][:12]:
        mark = "  [OK]" if n >= s["min_class"] else ""
        L.append("%d\t%s%s" % (m, f"{n:,}", mark))

    # ── model / daemon ──
    L.append("")
    L.append("=== 모델/데몬 ===")
    try:
        d = json.load(open(cfg.status_path))
        if d.get("no_model"):
            L.append("모델\t없음 (미학습) - /bewe-ai ais update 로 학습")
        else:
            L.append("모델 버전\tv%s" % d.get("model_version"))
            L.append("학습 클래스\t%s척" % d.get("n_classes"))
            L.append("정확도(top1)\t%.1f%%" % (100 * (d.get("val_top1") or 0)))
            L.append("마지막 학습\t%s" % (d.get("last_train") or "-"))
        L.append("데몬 지연(p50/p99)\t%s/%s ms" % (d.get("infer_p50_ms"), d.get("infer_p99_ms")))
        L.append("최근1시간 질의\t%s건" % d.get("bursts_1h"))
    except FileNotFoundError:
        L.append("데몬\t상태파일 없음 (데몬 미가동?)")
    return "\n".join(L)
