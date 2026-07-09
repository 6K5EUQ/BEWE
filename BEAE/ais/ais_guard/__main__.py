import argparse
import json

from .config import GuardConfig


def main():
    ap = argparse.ArgumentParser(prog="ais_guard", description="BEWE GUARD 경보 엔진")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("daemon", help="상시 감시 데몬 (AIS tail → 경보 발행)")

    rp = sub.add_parser("report", help="일일 보고서 생성 (수동 트리거)")
    rp.add_argument("--date", default=None, help="YYYYMMDD (기본: 오늘)")
    rp.add_argument("--no-emit", action="store_true", help="REPORT 경보 발행 생략")

    sp = sub.add_parser("status", help="guard_status.json 표시")
    sp.add_argument("--json", action="store_true", help="원본 JSON 그대로")


    a = ap.parse_args()
    cfg = GuardConfig()

    if a.cmd == "daemon":
        from .daemon import run
        run(cfg)
    elif a.cmd == "report":
        from .report import generate
        print(generate(cfg, a.date, emit=not a.no_emit))
    elif a.cmd == "status":
        try:
            doc = json.load(open(cfg.status_path))
        except (OSError, ValueError):
            print("상태 파일 없음 — 데몬 미실행?")
            return
        if a.json:
            print(json.dumps(doc, ensure_ascii=False, indent=2))
            return
        print(f"guard 데몬 pid={doc.get('pid')} 시각={doc.get('time')}")
        print(f"  트랙 {doc.get('tracks')} (활성 {doc.get('tracks_active')}) "
              f"수신 {doc.get('lines')}줄 (무효 {doc.get('bad_lines')})")
        print(f"  경보 활성 {doc.get('alerts_active')} / 누적발행 {doc.get('alerts_emitted')} "
              f"구역 {doc.get('zones')} scorer={doc.get('scorer')} predictor={doc.get('predictor')}")
        for e in doc.get("active", []):
            print(f"  - aid={e['aid']} typ={e['typ']} sev={e['sev']} "
                  f"mmsi={e['mmsi']} score={e['score']}")


if __name__ == "__main__":
    main()
