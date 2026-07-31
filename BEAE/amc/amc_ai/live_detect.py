"""Live AMC monitor. Streams RTL-SDR IQ, watches a few narrowband channels,
and whenever a channel's windowed power rises above its tracked noise floor
(detection filter), it channelizes that window and hands it to the synth-only
AMC model. Prints a timestamped prediction per trigger.

Purpose: catch bursty signals (DMR PTT, etc.) that a one-shot capture misses.

Pipe:  rtl_sdr -f 450e6 -s 2.56e6 -g 40 - | python -m amc_ai.live_detect
numpy only.
"""
from __future__ import annotations

import argparse
import os
import sys
import time

import numpy as np
import torch

sys.path.insert(0, os.path.expanduser("~/BEWE/BEAE/ais"))
from ais_ai.model import IQResNet1D  # noqa: E402
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from amc_ai.synth import CLASSES, to_tensor_layout  # noqa: E402

def _current_model():
    d = os.path.expanduser("~/BEWE/BEAE/amc/data/ai_model")
    cur = os.path.join(d, "current.json")
    if os.path.exists(cur):
        import json
        return os.path.join(d, json.load(open(cur))["model"])
    return os.path.join(d, "model_v0003.pt")


MODEL = _current_model()


def fir_lowpass(cutoff, fs, ntaps=257):
    n = np.arange(ntaps) - (ntaps - 1) / 2
    h = np.sinc(2 * cutoff / fs * n) * np.hamming(ntaps)
    return (h / h.sum()).astype(np.complex64)


class Channel:
    """One watched narrowband channel with a slow noise-floor tracker."""
    def __init__(self, name, f_off, fs, decim, cutoff):
        self.name = name
        self.f_off = f_off
        self.fs = fs
        self.decim = decim
        self.fs_out = fs / decim
        self.h = fir_lowpass(cutoff, fs)
        self.floor = None                    # EMA of low-power windows
        self.hist = np.zeros(len(CLASSES), dtype=int)
        self.phase0 = 0.0                    # carry NCO phase across blocks
        self.n0 = 0

    def process(self, iq, win, hop):
        """Return list of (start_in_out, snr_db, X) for triggered windows."""
        n = np.arange(self.n0, self.n0 + len(iq))
        self.n0 += len(iq)
        sh = iq * np.exp(-2j * np.pi * self.f_off / self.fs * n).astype(np.complex64)
        filt = np.convolve(sh, self.h, mode="same")[::self.decim].astype(np.complex64)
        p = np.abs(filt) ** 2
        starts = list(range(0, len(filt) - win, hop))
        if not starts:
            return []
        pw = np.array([p[s:s + win].mean() for s in starts])
        lo = np.percentile(pw, 10)           # stays near noise even if partly active
        self.floor = lo if self.floor is None else 0.85 * self.floor + 0.15 * lo
        return starts, pw, filt


def load_model(dev):
    ck = torch.load(MODEL, map_location=dev)
    m = IQResNet1D(n_classes=len(CLASSES)).to(dev)
    m.load_state_dict(ck["state"])
    m.eval()
    return m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fs", type=float, default=2.56e6)
    ap.add_argument("--fc", type=float, default=450e6)
    ap.add_argument("--decim", type=int, default=40)          # -> 64 kHz
    ap.add_argument("--cutoff", type=float, default=6000)
    ap.add_argument("--thr-db", type=float, default=8.0)      # detection threshold over floor
    ap.add_argument("--block", type=float, default=0.5)       # seconds per read
    ap.add_argument("--secs", type=float, default=120)        # total run time
    ap.add_argument("--chans", default="449.700,449.850")     # MHz, comma sep
    a = ap.parse_args()

    win, hop = 2048, 1024
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    model = load_model(dev)
    chans = [Channel(f"{c}MHz", (float(c) * 1e6 - a.fc), a.fs, a.decim, a.cutoff)
             for c in a.chans.split(",")]
    print(f"dev={dev} fc={a.fc/1e6}MHz fs={a.fs/1e6}Msps thr={a.thr_db}dB "
          f"watching {[c.name for c in chans]}  sps(DMR)={chans[0].fs_out/4800:.1f}")
    sys.stdout.flush()

    nsamp = int(a.fs * a.block)
    nbytes = nsamp * 2
    t_start = time.time()
    trig_total = 0
    while time.time() - t_start < a.secs:
        buf = sys.stdin.buffer.read(nbytes)
        if len(buf) < nbytes:
            break
        raw = (np.frombuffer(buf, dtype=np.uint8).astype(np.float32) - 127.5) / 127.5
        iq = (raw[0::2] + 1j * raw[1::2]).astype(np.complex64)
        iq -= iq.mean()
        t_rel = time.time() - t_start
        for ch in chans:
            starts, pw, filt = ch.process(iq, win, hop)
            thr = ch.floor * (10 ** (a.thr_db / 10))
            hits = [(s, 10 * np.log10(pw[i] / ch.floor + 1e-12))
                    for i, s in enumerate(starts) if pw[i] > thr]
            if not hits:
                continue
            X = np.stack([to_tensor_layout(filt[s:s + win]) for s, _ in hits])
            with torch.no_grad():
                prob = torch.softmax(model(torch.from_numpy(X).to(dev)), 1).cpu().numpy()
            preds = prob.argmax(1)
            conf = prob.max(1)
            for k in range(len(hits)):
                ch.hist[preds[k]] += 1
            trig_total += len(hits)
            # summarize this block's triggers for the channel
            top = np.bincount(preds, minlength=len(CLASSES))
            lead = CLASSES[top.argmax()]
            print(f"  t={t_rel:6.1f}s [{ch.name}] {len(hits):2d} trig  "
                  f"SNR {hits[0][1]:4.1f}-{max(h[1] for h in hits):4.1f}dB  "
                  f"-> {lead} (conf {conf.mean():.2f})")
            sys.stdout.flush()

    print(f"\n=== summary ({trig_total} triggers over {time.time()-t_start:.0f}s) ===")
    for ch in chans:
        tot = ch.hist.sum()
        if tot == 0:
            print(f"  [{ch.name}] no triggers")
            continue
        order = np.argsort(-ch.hist)
        dist = "  ".join(f"{CLASSES[i]}:{ch.hist[i]}" for i in order if ch.hist[i])
        print(f"  [{ch.name}] {tot} triggers:  {dist}")


if __name__ == "__main__":
    main()
