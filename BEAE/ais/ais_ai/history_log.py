"""Append per-training snapshot to ai_history/train_log.csv and refresh charts.

Called after each `ais_ai train` publish. Not part of the training/inference
path -- safe to fail without affecting the daemon or model files.
"""
import csv
import json
import os

FIELDS = [
    "version", "model_name", "trained_at", "classes", "train_n", "val_n",
    "val_top1", "val_bal_acc", "false_unknown", "natural_unknown_fpr",
    "param_count", "model_bytes", "train_seconds",
]


def _param_count(model_path):
    import torch
    sd = torch.load(model_path, map_location="cpu")
    return int(sum(t.numel() for t in sd.values()))


def record(model_dir, history_dir, version):
    """Read meta_vNNNN.json + model_vNNNN.pt for `version`, append a CSV row,
    then regenerate the trend charts. Idempotent: re-running for an existing
    version overwrites that version's row rather than duplicating it."""
    vtag = f"v{version:04d}"
    meta_path = os.path.join(model_dir, f"meta_{vtag}.json")
    model_path = os.path.join(model_dir, f"model_{vtag}.pt")
    with open(meta_path) as f:
        meta = json.load(f)

    row = {
        "version": version,
        "model_name": f"BEAEv{version}",
        "trained_at": meta.get("trained_at"),
        "classes": len(meta.get("classes", [])),
        "train_n": meta.get("train_n"),
        "val_n": meta.get("val_n"),
        "val_top1": meta.get("val_top1"),
        "val_bal_acc": meta.get("val_bal_acc"),
        "false_unknown": meta.get("false_unknown"),
        "natural_unknown_fpr": meta.get("natural_unknown_fpr"),
        "param_count": _param_count(model_path),
        "model_bytes": os.path.getsize(model_path),
        "train_seconds": meta.get("train_seconds"),
    }

    os.makedirs(history_dir, exist_ok=True)
    csv_path = os.path.join(history_dir, "train_log.csv")

    rows = []
    if os.path.exists(csv_path):
        with open(csv_path, newline="") as f:
            rows = [r for r in csv.DictReader(f) if int(r["version"]) != version]
    rows.append({k: row[k] for k in FIELDS})
    rows.sort(key=lambda r: int(r["version"]))

    with open(csv_path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=FIELDS)
        w.writeheader()
        w.writerows(rows)

    _plot_charts(rows, os.path.join(history_dir, "charts"))
    return csv_path


def _plot_charts(rows, charts_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(charts_dir, exist_ok=True)
    versions = [f"BEAEv{r['version']}" for r in rows]
    top1 = [float(r["val_top1"]) * 100 for r in rows]
    bal_acc = [float(r["val_bal_acc"]) * 100 for r in rows]
    nat_fpr = [float(r["natural_unknown_fpr"]) * 100 for r in rows]
    classes = [int(r["classes"]) for r in rows]
    params = [int(r["param_count"]) for r in rows]
    train_n = [int(r["train_n"]) + int(r["val_n"]) for r in rows]

    fig, axes = plt.subplots(2, 2, figsize=(11, 8))

    ax = axes[0][0]
    ax.plot(versions, top1, marker="o", label="val_top1 (%)")
    ax.plot(versions, bal_acc, marker="s", label="val_bal_acc (%)")
    ax.set_title("Accuracy over model versions")
    ax.set_ylim(0, 105)
    ax.legend()
    ax.grid(True, alpha=0.3)

    ax = axes[0][1]
    ax.plot(versions, nat_fpr, marker="o", color="tab:red")
    ax.set_title("Natural unknown FPR (%) — lower is better")
    ax.grid(True, alpha=0.3)

    ax = axes[1][0]
    ax.bar(versions, classes, color="tab:green")
    ax.set_title("Trained vessel classes")
    ax.grid(True, alpha=0.3, axis="y")

    ax = axes[1][1]
    ax.bar(versions, train_n, color="tab:orange", label="train+val samples")
    ax2 = ax.twinx()
    ax2.plot(versions, params, marker="d", color="tab:purple", label="param count")
    ax.set_title("Data volume vs. model size")
    ax.legend(loc="upper left")
    ax2.legend(loc="upper right")

    fig.tight_layout()
    fig.savefig(os.path.join(charts_dir, "training_trend.png"), dpi=150)
    plt.close(fig)
