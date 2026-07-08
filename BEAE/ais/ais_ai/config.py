"""Single source of configuration for the AIS RF-fingerprint daemon/trainer."""
import os
from dataclasses import dataclass, field


def _home() -> str:
    return os.path.expanduser("~")


@dataclass
class Config:
    # ── paths ──────────────────────────────────────────────────────────────
    data_dir: str = field(default_factory=lambda: os.path.join(_home(), "BEWE", "BEAE", "ais", "data"))
    sock_path: str = ""
    model_dir: str = ""
    log_path: str = ""
    status_path: str = ""

    # ── capture geometry (mirror of ais_decode.cpp constants) ──────────────
    # capture record = 896 complex samples @ ~48 kHz; HDLC payload gate at index 640.
    ai_pre: int = 640          # gate index inside capture
    ai_cap: int = 896          # total capture length

    # crop "preamble" (default, MMSI-leak-free): [gate-248, gate+72) = 320 samples
    #   48 pre-roll tail + ~200 ramp/preamble/flag + 72 margin
    crop_preamble_start: int = 640 - 248
    crop_preamble_len: int = 320
    # crop "payload" (ablation ONLY — includes MMSI bits, never deploy):
    #   [gate-248, 896) = 504 samples (adds first ~51 payload bits incl. MMSI)
    crop_payload_start: int = 640 - 248
    crop_payload_len: int = 504

    nominal_sr: int = 48000
    sr_tol: float = 0.005          # |sr-nominal|/nominal within this → use as-is
    sr_resample_max: float = 0.05  # above sr_tol but within this → resample; beyond → drop

    # ── dataset ────────────────────────────────────────────────────────────
    window_days: int = 14
    min_class: int = 200       # bursts required (train window) to become a class
    cap_per_class: int = 2000  # effective samples per class per epoch
    val_frac: float = 0.15     # last 15% of each class's time range
    guard_gap_s: int = 600     # drop val-adjacent train bursts within this gap

    # ── training ───────────────────────────────────────────────────────────
    batch: int = 256
    lr: float = 3e-3
    weight_decay: float = 1e-2
    max_epochs: int = 60
    patience: int = 10
    warmup_epochs: int = 3
    label_smooth: float = 0.05
    seed: int = 42

    # augmentation
    aug_snr_db: float = 10.0   # additive noise: degrade SNR by U(0, this) dB
    aug_shift: int = 3         # random integer time shift +-samples
    aug_amp_lo: float = 0.7
    aug_amp_hi: float = 1.4

    # ── open-set ───────────────────────────────────────────────────────────
    tau_min: float = 0.50      # global softmax-prob floor
    tau_pctile: float = 5.0    # per-class threshold percentile of correct val probs
    fpr_target: float = 0.10   # max fraction of natural unknowns allowed above threshold

    # ── daemon ─────────────────────────────────────────────────────────────
    status_interval_s: int = 30
    model_check_interval_s: float = 5.0
    prune_days: int = 21       # delete aicap files older than this

    def __post_init__(self):
        if not self.sock_path:
            self.sock_path = os.path.join(self.data_dir, "ai.sock")
        if not self.model_dir:
            self.model_dir = os.path.join(self.data_dir, "ai_model")
        if not self.log_path:
            self.log_path = os.path.join(self.data_dir, "ai_daemon.log")
        if not self.status_path:
            self.status_path = os.path.join(self.data_dir, "ai_status.json")
