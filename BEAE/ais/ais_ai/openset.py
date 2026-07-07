"""Open-set decision: temperature-scaled max-softmax with per-class thresholds."""
import numpy as np
import torch

from .proto import ST_MATCH, ST_UNKNOWN


def fit_temperature(logits: torch.Tensor, labels: torch.Tensor) -> float:
    """Single-scalar temperature minimizing NLL on val logits (LBFGS)."""
    log_t = torch.zeros(1, requires_grad=True)
    opt = torch.optim.LBFGS([log_t], lr=0.1, max_iter=50)
    logits = logits.float().cpu()
    labels = labels.cpu()

    def closure():
        opt.zero_grad()
        loss = torch.nn.functional.cross_entropy(logits / log_t.exp(), labels)
        loss.backward()
        return loss

    opt.step(closure)
    return float(log_t.exp().clamp(0.05, 20.0))


def calibrate_thresholds(cfg, probs_val: np.ndarray, labels_val: np.ndarray,
                         probs_unknown: np.ndarray | None, n_classes: int):
    """Per-class tau = pctile of correct-prediction max-prob; then raise the
    global floor until natural-unknown FPR <= fpr_target. Returns (taus, floor)."""
    pred = probs_val.argmax(1)
    pmax = probs_val.max(1)
    taus = np.full(n_classes, cfg.tau_min, dtype=np.float64)
    for c in range(n_classes):
        sel = (labels_val == c) & (pred == c)
        if sel.sum() >= 10:
            taus[c] = max(cfg.tau_min, float(np.percentile(pmax[sel], cfg.tau_pctile)))
    floor = cfg.tau_min
    if probs_unknown is not None and len(probs_unknown):
        u_pred = probs_unknown.argmax(1)
        u_pmax = probs_unknown.max(1)
        for cand in np.arange(cfg.tau_min, 0.996, 0.005):
            eff = np.maximum(taus[u_pred], cand)
            if float((u_pmax >= eff).mean()) <= cfg.fpr_target:
                floor = float(cand)
                break
        else:
            floor = 0.995
    return np.maximum(taus, floor).tolist(), floor


def decide(logits: np.ndarray, temperature: float, taus, classes):
    """logits [C] -> (status, pred_mmsi, conf_millipct). conf scaled *1000 (3 decimal places)."""
    z = logits / max(temperature, 1e-6)
    z -= z.max()
    p = np.exp(z); p /= p.sum()
    c = int(p.argmax())
    conf = int(round(100000.0 * float(p[c])))
    if float(p[c]) >= taus[c]:
        return ST_MATCH, int(classes[c]), conf
    return ST_UNKNOWN, int(classes[c]), conf
