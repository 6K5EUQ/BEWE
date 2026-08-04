"""Single source of configuration for the AMC daemon/trainer.

The C++ side (src/modules/amc/amc_ai.cpp) derives the same paths from
BEWEPaths::data_dir()+"/BEAE/amc/...". If you move data_dir here you MUST change
it there too and recompile — a mismatch kills the IPC silently (the daemon binds
one socket, BEWE connects to another, and every inference just times out).
"""
import os
from dataclasses import dataclass, field


def _home() -> str:
    return os.path.expanduser("~")


@dataclass
class Config:
    # ── paths ──────────────────────────────────────────────────────────────
    data_dir: str = field(default_factory=lambda: os.path.join(_home(), "BEWE", "BEAE", "amc", "data"))
    sock_path: str = ""
    model_dir: str = ""
    log_path: str = ""
    status_path: str = ""

    # ── inference geometry ─────────────────────────────────────────────────
    # BEWE sends AMC_CAP complex samples (amc_decode.cpp, currently 4096) at
    # whatever out_sr the channel DDC produced. The model wants `nsamp` — we crop
    # here rather than in C++ so retraining with a different nsamp needs no
    # recompile. nsamp is read from the checkpoint's current.json.
    default_nsamp: int = 2048
    # Take the crop from the middle of the burst: the head can still hold the
    # squelch attack ramp and the tail may run past the burst into noise.
    crop_center: bool = True

    # ── daemon ─────────────────────────────────────────────────────────────
    status_interval_s: int = 30
    model_check_interval_s: float = 5.0

    def __post_init__(self):
        if not self.sock_path:
            self.sock_path = os.path.join(self.data_dir, "ai.sock")
        if not self.model_dir:
            self.model_dir = os.path.join(self.data_dir, "ai_model")
        if not self.log_path:
            self.log_path = os.path.join(self.data_dir, "ai_daemon.log")
        if not self.status_path:
            self.status_path = os.path.join(self.data_dir, "ai_status.json")
