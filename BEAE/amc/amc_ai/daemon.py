"""Daemon wiring: UDS server thread + status writer.

No auto-retrain by design — the operator runs `python -m amc_ai train` and the
registry hot-swaps when current.json changes. Mirror of ais_ai/daemon.py.
"""
import json
import logging
import logging.handlers
import os
import signal
import threading
import time

from . import __version__
from .config import Config
from .infer import ModelRegistry
from .server import Server, Stats


def _setup_logging(cfg: Config):
    os.makedirs(os.path.dirname(cfg.log_path), exist_ok=True)
    h = logging.handlers.RotatingFileHandler(cfg.log_path, maxBytes=5 << 20, backupCount=5)
    h.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s"))
    root = logging.getLogger()
    root.setLevel(logging.INFO)
    root.addHandler(h)
    root.addHandler(logging.StreamHandler())


def _write_status(cfg: Config, reg: ModelRegistry, stats: Stats):
    doc = {
        "pid": os.getpid(), "daemon_version": __version__,
        "time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "no_model": reg.model is None,
        "model_version": reg.version,
        "n_classes": len(reg.classes),
        "nsamp": reg.nsamp,
        "device": reg.dev,
        **stats.snapshot(),
    }
    tmp = cfg.status_path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(doc, f, indent=1)
    os.replace(tmp, cfg.status_path)   # atomic — a reader never sees a half file


def run(cfg: Config | None = None):
    cfg = cfg or Config()
    _setup_logging(cfg)
    log = logging.getLogger("amc_ai.daemon")
    log.info("daemon starting (sock=%s, model_dir=%s)", cfg.sock_path, cfg.model_dir)

    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())

    reg = ModelRegistry(cfg)
    stats = Stats()
    srv = Server(cfg, reg, stats)
    t = threading.Thread(target=srv.serve_forever, args=(stop,), daemon=True, name="uds")
    t.start()

    while not stop.is_set():
        try:
            _write_status(cfg, reg, stats)
        except Exception:
            log.exception("status write failed")
        stop.wait(cfg.status_interval_s)
    log.info("daemon stopping")
    t.join(timeout=3)
    try:
        os.unlink(cfg.sock_path)
    except OSError:
        pass
