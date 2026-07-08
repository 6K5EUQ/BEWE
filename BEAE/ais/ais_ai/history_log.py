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
    "ece", "worst_class_mmsi", "worst_class_recall",
    "epochs_run", "param_count", "model_bytes", "train_seconds",
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
        "ece": meta.get("ece"),
        "worst_class_mmsi": meta.get("worst_class_mmsi"),
        "worst_class_recall": meta.get("worst_class_recall"),
        "epochs_run": meta.get("epochs_run"),
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


_INK = "#3a3a34"
_MUTED = "#6b6a60"
_GRID = "#d8d6ca"
_BLUE = "#2a78d6"     # categorical slot 1 — val_top1
_RED = "#e34948"      # categorical slot 6 — natural_unknown_fpr (status: higher = worse)
_GREEN = "#008300"    # categorical slot 4 — trained classes
_ORANGE = "#eb6834"   # categorical slot 8 — data volume
_VIOLET = "#4a3aa7"   # categorical slot 5 — train time


def _style_axis(ax, title):
    ax.set_title(title, color=_INK, fontsize=12, fontweight="bold", loc="left")
    ax.tick_params(colors=_MUTED, labelsize=9)
    for spine in ("top", "right"):
        ax.spines[spine].set_visible(False)
    for spine in ("left", "bottom"):
        ax.spines[spine].set_color(_GRID)
    ax.grid(True, axis="y", color=_GRID, linewidth=0.8, alpha=0.7)
    ax.set_axisbelow(True)


def _plot_charts(rows, charts_dir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(charts_dir, exist_ok=True)
    versions = [f"v{r['version']}" for r in rows]
    top1 = [float(r["val_top1"]) * 100 for r in rows]
    nat_fpr = [float(r["natural_unknown_fpr"]) * 100 for r in rows]
    classes = [int(r["classes"]) for r in rows]
    train_n = [int(r["train_n"]) + int(r["val_n"]) for r in rows]
    train_s = [float(r["train_seconds"]) for r in rows]
    epochs = [int(r["epochs_run"]) if r.get("epochs_run") else None for r in rows]
    # ece/worst_class_recall are absent from pre-existing rows (added later) —
    # only plot versions that actually have them rather than faking a 0.
    ece_v = [(v, float(r["ece"]) * 100) for v, r in zip(versions, rows) if r.get("ece")]
    worst_v = [(v, float(r["worst_class_recall"]) * 100) for v, r in zip(versions, rows)
               if r.get("worst_class_recall")]

    fig, axes = plt.subplots(2, 4, figsize=(22, 8))
    fig.patch.set_facecolor("white")

    # Closed-set accuracy: values cluster near 100%, so zoom to where the
    # differences actually live instead of wasting 0-90 on empty axis.
    ax = axes[0][0]
    lo = min(top1)
    ax.plot(versions, top1, marker="o", markersize=8, linewidth=2, color=_BLUE)
    for x, y in zip(versions, top1):
        ax.annotate(f"{y:.1f}%", xy=(x, y), xytext=(0, 8), textcoords="offset points",
                    ha="center", fontsize=9, color=_INK)
    ax.set_ylim(max(0, lo - 3), 101.5)
    _style_axis(ax, "Closed-set accuracy (val_top1, %)")

    # Open-set false-positive rate: real risk metric, so the axis must start
    # at 0 — a floating floor here would exaggerate version-to-version noise.
    ax = axes[0][1]
    ax.plot(versions, nat_fpr, marker="o", markersize=8, linewidth=2, color=_RED)
    for x, y in zip(versions, nat_fpr):
        ax.annotate(f"{y:.0f}%", xy=(x, y), xytext=(0, 8), textcoords="offset points",
                    ha="center", fontsize=9, color=_INK)
    ax.set_ylim(0, max(nat_fpr) * 1.2)
    _style_axis(ax, "Unknown-vessel false-positive rate (%, lower=better)")

    # Calibration error: a high top1 can hide a model whose confidence numbers
    # are meaningless (e.g. always says 99.9% regardless of whether it's right).
    # 0 = a stated confidence matches its actual hit rate; axis starts at 0.
    ax = axes[0][2]
    if ece_v:
        vv, ee = zip(*ece_v)
        ax.plot(vv, ee, marker="o", markersize=8, linewidth=2, color=_BLUE)
        for x, y in zip(vv, ee):
            ax.annotate(f"{y:.1f}%", xy=(x, y), xytext=(0, 8), textcoords="offset points",
                        ha="center", fontsize=9, color=_INK)
        ax.set_ylim(0, max(max(ee) * 1.3, 2))
    else:
        ax.text(0.5, 0.5, "no data yet", transform=ax.transAxes, ha="center",
                color=_MUTED, fontsize=10)
    _style_axis(ax, "Calibration error (ECE, %, lower=better)")

    ax = axes[1][0]
    bars = ax.bar(versions, classes, color=_GREEN, width=0.55)
    ax.bar_label(bars, padding=3, fontsize=9, color=_INK)
    ax.set_ylim(0, max(classes) * 1.2)
    _style_axis(ax, "Trained vessel classes (count)")

    # Param count barely moves version to version (< 0.2% drift) — not worth
    # a second axis; drop it and show what actually changes: training data volume.
    # Runs of identical volume across consecutive versions mean new captures
    # weren't reaching training (e.g. the 2026-07 sample-rate-mismatch bug that
    # silently dropped a whole day's bursts) — fade those bars and call it out
    # instead of letting a flat run look like normal data growth.
    ax = axes[1][1]
    stuck = [i for i in range(1, len(train_n)) if train_n[i] == train_n[i - 1]]
    colors = [(_ORANGE + "55") if (i in stuck or i + 1 in stuck) else _ORANGE
              for i in range(len(train_n))]
    bars = ax.bar(versions, train_n, color=colors, width=0.55)
    ax.bar_label(bars, padding=3, fontsize=9, color=_INK,
                 labels=[f"{n:,}" for n in train_n])
    ax.set_ylim(0, max(train_n) * 1.2)
    if stuck:
        lo, hi = versions[stuck[0] - 1], versions[stuck[-1]]
        ax.annotate(f"stuck {lo}–{hi}: new data not reaching training\n(fixed: sample-rate resample)",
                    xy=(0.5, 0.97), xycoords="axes fraction", ha="center", va="top",
                    fontsize=8, color=_MUTED, style="italic")
    _style_axis(ax, "Training data volume (train+val bursts)")

    # Worst-performing class: overall top1/bal_acc are means and can hide one
    # thin-data vessel doing badly — this surfaces that floor directly.
    ax = axes[1][2]
    if worst_v:
        vv, ww = zip(*worst_v)
        bars = ax.bar(vv, ww, color=_RED, width=0.55)
        ax.bar_label(bars, padding=3, fontsize=9, color=_INK, fmt="%.0f%%")
        ax.set_ylim(0, 105)
    else:
        ax.text(0.5, 0.5, "no data yet", transform=ax.transAxes, ha="center",
                color=_MUTED, fontsize=10)
    _style_axis(ax, "Worst single-class recall (%, lower=worse)")

    # Train time scales with data volume (from-scratch each run) — useful to know
    # how the retrain cost grows as more vessels/bursts accumulate.
    ax = axes[0][3]
    bars = ax.bar(versions, train_s, color=_VIOLET, width=0.55)
    ax.bar_label(bars, padding=3, fontsize=9, color=_INK, fmt="%.0fs")
    ax.set_ylim(0, max(train_s) * 1.2)
    _style_axis(ax, "Train time (s, from scratch)")

    # Epochs actually run before early-stop. This is why train time doesn't track
    # data volume: a run that plateaus early stops sooner and finishes faster
    # (fewer epochs), even on a bigger dataset — the real driver of wall-clock.
    ax = axes[1][3]
    ep_v = [(v, e) for v, e in zip(versions, epochs) if e is not None]
    if ep_v:
        vv, ee = zip(*ep_v)
        bars = ax.bar(vv, ee, color=_BLUE, width=0.55)
        ax.bar_label(bars, padding=3, fontsize=9, color=_INK)
        ax.set_ylim(0, max(ee) * 1.25)
    else:
        ax.text(0.5, 0.5, "no data yet", transform=ax.transAxes, ha="center",
                color=_MUTED, fontsize=10)
    _style_axis(ax, "Epochs run (early-stop point)")

    fig.tight_layout()
    fig.savefig(os.path.join(charts_dir, "training_trend.png"), dpi=150, facecolor="white")
    plt.close(fig)
