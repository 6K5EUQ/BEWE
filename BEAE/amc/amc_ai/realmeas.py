"""Live-capture AMC validation. Reads a raw RTL-SDR u8 IQ dump, finds active
narrowband channels, channelizes the strongest, and asks the synth-only model
what it is. Ground truth for this run is operator-supplied (e.g. DMR = 4FSK).

numpy only (no scipy on this host).
"""
from __future__ import annotations

import argparse
import os
import sys

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


def load_u8(path, fs):
    raw = np.fromfile(path, dtype=np.uint8).astype(np.float32)
    raw = (raw - 127.5) / 127.5
    iq = raw[0::2] + 1j * raw[1::2]
    iq -= iq.mean()                       # kill DC spike
    return iq.astype(np.complex64)


def psd(iq, fs, nfft=4096):
    nb = len(iq) // nfft
    acc = np.zeros(nfft)
    w = np.hanning(nfft)
    for i in range(nb):
        b = iq[i * nfft:(i + 1) * nfft] * w
        acc += np.abs(np.fft.fftshift(np.fft.fft(b))) ** 2
    acc /= nb
    f = np.fft.fftshift(np.fft.fftfreq(nfft, 1 / fs))
    return f, 10 * np.log10(acc + 1e-12)


def find_channels(f, p, fc, min_sep_hz=20000, top=8):
    """Peaks above (median + 6 dB), separated by min_sep_hz."""
    thr = np.median(p) + 6.0
    order = np.argsort(-p)
    chosen = []
    for i in order:
        if p[i] < thr:
            break
        fo = f[i]
        if all(abs(fo - c[0]) > min_sep_hz for c in chosen):
            chosen.append((fo, p[i]))
        if len(chosen) >= top:
            break
    return chosen


def fir_lowpass(cutoff, fs, ntaps=257):
    n = np.arange(ntaps) - (ntaps - 1) / 2
    h = np.sinc(2 * cutoff / fs * n) * np.hamming(ntaps)
    return (h / h.sum()).astype(np.float32)


def channelize(iq, fs, f_off, decim, cutoff):
    """Shift f_off to DC, LPF, decimate. Returns (iq_ch, fs_out)."""
    n = np.arange(len(iq))
    sh = iq * np.exp(-2j * np.pi * f_off / fs * n).astype(np.complex64)
    h = fir_lowpass(cutoff, fs)
    filt = np.convolve(sh, h, mode="same")
    return filt[::decim].astype(np.complex64), fs / decim


def bursts(iq_ch, win=2048, hop=1024, top_n=40):
    """Windows above noise floor; if none, the top_n by power (continuous sig)."""
    p = np.abs(iq_ch) ** 2
    starts = list(range(0, len(iq_ch) - win, hop))
    pw = np.array([p[s:s + win].mean() for s in starts])
    floor = np.percentile(pw, 25)
    snr_db = 10 * np.log10(pw / floor + 1e-12)
    thr_db = 6.0
    sel = [i for i in range(len(pw)) if snr_db[i] > thr_db]
    mode = "burst"
    if not sel:                            # continuous / no clear burst -> top-N
        sel = list(np.argsort(-pw)[:top_n])
        mode = "top-power"
    out = [starts[i] for i in sel]
    return out, snr_db[sel], mode


def load_model(dev):
    ck = torch.load(MODEL, map_location=dev)
    m = IQResNet1D(n_classes=len(CLASSES)).to(dev)
    m.load_state_dict(ck["state"])
    m.eval()
    return m


@torch.no_grad()
def classify(model, dev, iq_ch, starts, win=2048):
    X = np.stack([to_tensor_layout(iq_ch[s:s + win]) for s in starts])
    logits = model(torch.from_numpy(X).to(dev))
    prob = torch.softmax(logits, 1).cpu().numpy()
    preds = prob.argmax(1)
    return preds, prob


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cap", default="/tmp/dmr_cap.bin")
    ap.add_argument("--fs", type=float, default=2.56e6)
    ap.add_argument("--fc", type=float, default=450e6)
    ap.add_argument("--decim", type=int, default=50)     # 2.56M/50 = 51.2k
    ap.add_argument("--cutoff", type=float, default=8000)
    ap.add_argument("--chan", type=float, default=None,  help="force channel offset Hz")
    a = ap.parse_args()

    iq = load_u8(a.cap, a.fs)
    print(f"loaded {len(iq)} samples = {len(iq)/a.fs:.1f}s @ {a.fs/1e6:.2f} Msps, fc={a.fc/1e6:.3f} MHz")

    f, p = psd(iq, a.fs)
    ch = find_channels(f, p, a.fc)
    print("\n=== detected channels (offset from center) ===")
    for fo, pw in ch:
        print(f"  {fo/1e3:+9.1f} kHz  ({(a.fc+fo)/1e6:9.4f} MHz)   {pw:6.1f} dB")

    dev = "cuda" if torch.cuda.is_available() else "cpu"
    model = load_model(dev)
    fs_out = a.fs / a.decim
    print(f"\nchannel fs_out = {fs_out/1e3:.1f} kHz   (DMR 4800 baud -> sps={fs_out/4800:.1f})")

    targets = [a.chan] if a.chan is not None else [c[0] for c in ch[:4]]
    for f_off in targets:
        iq_ch, fso = channelize(iq, a.fs, f_off, a.decim, a.cutoff)
        starts, snr, mode = bursts(iq_ch)
        preds, prob = classify(model, dev, iq_ch, starts)
        hist = np.bincount(preds, minlength=len(CLASSES))
        top = sorted(range(len(CLASSES)), key=lambda i: -hist[i])[:5]
        conf = prob.max(1).mean()
        dist = "  ".join(f"{CLASSES[i]}:{hist[i]}" for i in top if hist[i])
        print(f"\n[{f_off/1e3:+.1f} kHz  {(a.fc+f_off)/1e6:.4f} MHz]  {len(starts)} windows ({mode}, SNR {snr.min():.1f}-{snr.max():.1f} dB)")
        print(f"   preds: {dist}")
        print(f"   mean conf = {conf:.3f}")


if __name__ == "__main__":
    main()
