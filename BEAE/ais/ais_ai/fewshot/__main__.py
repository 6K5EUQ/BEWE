"""CLI for the experimental residual+prototypical model (isolated from `ais_ai`).

  python -m ais_ai.fewshot eval       # train in-memory + full report, NO publish
  python -m ais_ai.fewshot train      # train + publish to data/ai_model_proto/
  python -m ais_ai.fewshot autotrain  # train+publish only once enough data accumulated
  python -m ais_ai.fewshot status     # current proto model + kept versions (rollback)

Never touches data/ai_model/ (deployed IQResNet1D) or the live UDS daemon.
"""
import argparse

from ..config import Config
from . import pipeline


def main():
    ap = argparse.ArgumentParser(prog="ais_ai.fewshot")
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("train", "eval"):
        p = sub.add_parser(name)
        p.add_argument("--days", type=int, default=14)
    at = sub.add_parser("autotrain")
    at.add_argument("--days", type=int, default=14)
    at.add_argument("--min-classes", type=int, default=10,
                    help="minimum eligible transmitters before training runs")
    sub.add_parser("status")

    a = ap.parse_args()
    cfg = Config()
    if a.cmd == "eval":
        pipeline.run_train(cfg, a.days, do_publish=False)
    elif a.cmd == "train":
        pipeline.run_train(cfg, a.days, do_publish=True)
    elif a.cmd == "autotrain":
        pipeline.autotrain(cfg, a.days, a.min_classes)
    elif a.cmd == "status":
        pipeline.status(cfg)


if __name__ == "__main__":
    main()
