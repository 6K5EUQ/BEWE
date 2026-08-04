import argparse
import json
import sys

from .config import Config


def main():
    ap = argparse.ArgumentParser(prog="amc_ai", description="BEWE AMC modulation classifier daemon/trainer")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("daemon", help="serve UDS inference + status (no auto-retrain)")
    sub.add_parser("status", help="print ai_status.json")

    tp = sub.add_parser("train", help="retrain from synthetic bursts and publish for hot-swap")
    tp.add_argument("--epochs", type=int, default=None)
    tp.add_argument("--nsamp", type=int, default=None)

    a = ap.parse_args()
    cfg = Config()

    if a.cmd == "daemon":
        from .daemon import run
        run(cfg)
    elif a.cmd == "status":
        try:
            with open(cfg.status_path) as f:
                print(json.dumps(json.load(f), indent=1))
        except OSError as e:
            print(f"no status: {e}", file=sys.stderr)
            return 1
    elif a.cmd == "train":
        # train.py is the existing standalone trainer; keep it as the single
        # source of training logic rather than duplicating it here.
        from . import train as _t
        argv = []
        if a.epochs is not None:
            argv += ["--epochs", str(a.epochs)]
        if a.nsamp is not None:
            argv += ["--nsamp", str(a.nsamp)]
        sys.argv = ["amc_ai.train"] + argv
        _t.main()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
