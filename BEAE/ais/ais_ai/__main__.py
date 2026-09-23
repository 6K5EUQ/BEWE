import argparse
import json
import sys

from .config import Config


def main():
    ap = argparse.ArgumentParser(prog="ais_ai", description="BEWE AIS RF fingerprint daemon/trainer")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("daemon", help="serve UDS inference + status (no auto-retrain)")

    tp = sub.add_parser("train", help="full retrain from aicap window, publish for hot-swap")
    tp.add_argument("--days", type=int, default=14)
    tp.add_argument("--crop", choices=["clean", "preamble", "payload"], default="clean",
                    help="preamble/payload crops contain payload bits - ablation only, never deploy")
    tp.add_argument("--min-class", type=int, default=None)

    ep = sub.add_parser("eval", help="train in-memory + full report (no publish)")
    ep.add_argument("--days", type=int, default=14)
    ep.add_argument("--crop", choices=["clean", "preamble", "payload"], default="clean")
    ep.add_argument("--holdout-k", type=int, default=0, help="hold out K classes for open-set test")
    ep.add_argument("--baseline-jsonl", action="store_true", help="also run hand-feature baseline")
    ep.add_argument("--min-class", type=int, default=None)

    sp = sub.add_parser("status", help="training-data accumulation + model/daemon state")
    sp.add_argument("--days", type=int, default=14)
    sp.add_argument("--json", action="store_true", help="raw ai_status.json instead of table")

    ip = sub.add_parser("inspect", help="aicap .bin parser statistics")
    ip.add_argument("path")

    a = ap.parse_args()
    cfg = Config()
    if getattr(a, "min_class", None):
        cfg.min_class = a.min_class

    if a.cmd == "daemon":
        from .daemon import run
        run(cfg)
    elif a.cmd == "train":
        if a.crop != "clean":
            print(f"WARNING: {a.crop} crop contains payload bits - ablation only, do not deploy",
                  file=sys.stderr)
        from .train import train
        train(cfg, a.days, a.crop)
    elif a.cmd == "eval":
        from .evaluate import run_eval
        run_eval(cfg, a.days, a.holdout_k, a.baseline_jsonl, a.crop)
    elif a.cmd == "status":
        if a.json:
            try:
                print(json.dumps(json.load(open(cfg.status_path)), indent=2))
            except FileNotFoundError:
                print("no status file - daemon not running?")
        else:
            from .statusview import render
            print(render(cfg, a.days))
    elif a.cmd == "inspect":
        from .aicap import inspect
        st = inspect(a.path)
        pm = st.pop("per_mmsi")
        print(json.dumps(st, indent=2))
        top = sorted(pm.items(), key=lambda kv: -kv[1])[:15]
        print("top MMSIs:", ", ".join(f"{m}:{n}" for m, n in top))


if __name__ == "__main__":
    main()
