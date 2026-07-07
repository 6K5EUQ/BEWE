"""Daemon wiring: UDS server thread + status writer. No auto-retrain by
design (operator runs `python -m ais_ai train` manually; the registry
hot-swaps when current.json changes)."""
import glob
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
    meta = reg.meta or {}
    doc = {
        "pid": os.getpid(), "daemon_version": __version__,
        "time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "no_model": reg.meta is None,
        "model_version": meta.get("version"),
        "n_classes": len(meta.get("classes", [])),
        "val_top1": meta.get("val_top1"), "val_bal_acc": meta.get("val_bal_acc"),
        "natural_unknown_fpr": meta.get("natural_unknown_fpr"),
        "last_train": meta.get("trained_at"), "crop": meta.get("crop"),
        **stats.snapshot(),
    }
    tmp = cfg.status_path + ".tmp"
    with open(tmp, "w") as f:
        json.dump(doc, f, indent=1)
    os.replace(tmp, cfg.status_path)


def _prune_old_caps(cfg: Config, log):
    cut = time.time() - cfg.prune_days * 86400
    for p in glob.glob(os.path.join(cfg.data_dir, "aicap_*.bin")):
        try:
            if os.path.getmtime(p) < cut:
                os.remove(p)
                log.info("pruned old capture %s", os.path.basename(p))
        except OSError:
            pass


def run(cfg: Config | None = None):
    cfg = cfg or Config()
    _setup_logging(cfg)
    log = logging.getLogger("ais_ai.daemon")
    log.info("daemon starting (sock=%s, model_dir=%s)", cfg.sock_path, cfg.model_dir)

    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    signal.signal(signal.SIGINT, lambda *_: stop.set())

    reg = ModelRegistry(cfg)
    stats = Stats()
    srv = Server(cfg, reg, stats)
    t = threading.Thread(target=srv.serve_forever, args=(stop,), daemon=True, name="uds")
    t.start()

    last_prune = 0.0
    while not stop.is_set():
        try:
            _write_status(cfg, reg, stats)
        except Exception:
            log.exception("status write failed")
        if time.time() - last_prune > 6 * 3600:
            last_prune = time.time()
            _prune_old_caps(cfg, log)
        stop.wait(cfg.status_interval_s)
    log.info("daemon stopping")
    t.join(timeout=3)
    try:
        os.unlink(cfg.sock_path)
    except OSError:
        pass
