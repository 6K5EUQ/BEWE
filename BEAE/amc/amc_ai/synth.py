"""Synthetic modulated-baseband generator for AMC training.

Why synthesis instead of captures: an AMC label is a physical property of the
waveform, so we can generate it exactly. Real captures are the opposite - the
label has to be inferred, which is the thing we are trying to learn. Captures
are kept for validation (see amc_ai.realcap), never as the primary train set.

Every generator returns complex64 baseband at `sps` samples per symbol, unit
average power, before channel impairments. Impairments are applied separately
so the same clean burst can be replayed at many SNRs.
"""
from __future__ import annotations

import numpy as np

# Class list. Ordering is the label index and must stay stable once a model is
# trained - appending is safe, reordering silently invalidates every checkpoint.
CLASSES = [
    "BPSK", "QPSK", "PSK8", "QAM16", "QAM64",
    "2FSK", "4FSK", "GMSK", "MSK",
    "AM", "FM", "OFDM",
]
CLASS_IDX = {c: i for i, c in enumerate(CLASSES)}


def _rrc(beta: float, sps: int, span: int = 8) -> np.ndarray:
    """Root-raised-cosine pulse. Linear modulations are pulse-shaped in the
    real world; training on rectangular symbols teaches the net a spectrum
    that never occurs on air."""
    n = np.arange(-span * sps / 2, span * sps / 2 + 1, dtype=np.float64)
    t = n / sps
    with np.errstate(divide="ignore", invalid="ignore"):
        num = np.sin(np.pi * t * (1 - beta)) + 4 * beta * t * np.cos(np.pi * t * (1 + beta))
        den = np.pi * t * (1 - (4 * beta * t) ** 2)
        h = num / den
    # Removable singularities at t=0 and t=+-1/(4beta).
    h[np.isclose(t, 0.0)] = 1.0 + beta * (4 / np.pi - 1)
    if beta > 0:
        edge = np.isclose(np.abs(4 * beta * t), 1.0)
        if edge.any():
            h[edge] = (beta / np.sqrt(2)) * (
                (1 + 2 / np.pi) * np.sin(np.pi / (4 * beta))
                - (1 - 2 / np.pi) * np.cos(np.pi / (4 * beta))
            )
    h = np.nan_to_num(h)
    return h / np.sqrt(np.sum(h ** 2))


def _upsample_shape(sym: np.ndarray, sps: int, beta: float, rng) -> np.ndarray:
    up = np.zeros(len(sym) * sps, dtype=np.complex128)
    up[::sps] = sym
    return np.convolve(up, _rrc(beta, sps), mode="same")


# ── linear (memoryless) constellations ────────────────────────────────────
_CONST = {
    "BPSK": np.array([1, -1], dtype=np.complex128),
    "QPSK": np.exp(1j * (np.pi / 4 + np.arange(4) * np.pi / 2)),
    "PSK8": np.exp(1j * np.arange(8) * np.pi / 4),
}


def _qam(m: int) -> np.ndarray:
    k = int(np.sqrt(m))
    lv = np.arange(-(k - 1), k, 2)
    grid = np.array([complex(i, q) for q in lv for i in lv])
    return grid / np.sqrt(np.mean(np.abs(grid) ** 2))


_CONST["QAM16"] = _qam(16)
_CONST["QAM64"] = _qam(64)


def _linear(kind: str, nsym: int, sps: int, rng) -> np.ndarray:
    c = _CONST[kind]
    sym = c[rng.integers(0, len(c), nsym)]
    return _upsample_shape(sym, sps, rng.uniform(0.2, 0.5), rng)


# ── frequency / phase modulations ─────────────────────────────────────────
def _fsk(levels: int, nsym: int, sps: int, rng) -> np.ndarray:
    """Continuous-phase FSK. h is the modulation index; the integral of the
    instantaneous frequency must be continuous or the spectrum is wrong.

    2FSK at h=0.5 *is* MSK, so the binary case must avoid a neighbourhood of
    0.5 - otherwise the two labels describe the same waveform and the model is
    punished for a contradiction in the data (seen as 2FSK->MSK confusion).

    Real FSK modems premod-filter the symbol stream (DMR/dPMR use RRC alpha 0.2,
    others use a gaussian) so the frequency transitions are smooth, not the hard
    rectangular switching of a textbook FSK. Training on rect-only made the net
    file real DMR (RRC-shaped 4FSK) under GMSK/FM instead of 4FSK — the smoothed
    transitions live outside the rect manifold. So the frequency waveform is
    randomly shaped here (none / gaussian / RRC), with unit-DC-gain kernels that
    smooth transitions while preserving each symbol's deviation (hence h)."""
    dev = np.arange(-(levels - 1), levels, 2, dtype=np.float64)
    f = dev[rng.integers(0, levels, nsym)]
    freq = np.repeat(f, sps).astype(np.float64)
    if levels == 2:
        # Leave 2FSK as rectangular switching. Gaussian-shaping a binary CPFSK
        # *is* GMSK, so shaping the 2FSK class collides with GMSK and real AIS
        # (GMSK, h=0.5) then reads as 2FSK. Keep the pre-existing separation.
        h = rng.uniform(0.65, 1.2)
    else:
        # h = modulation index (adjacent-symbol Δf / symbol rate). Real 4FSK is
        # narrow: DMR's ±1944/±648 Hz at 4800 baud is h=0.27, below the old 0.4
        # floor, and real modems premod-filter (DMR = RRC alpha 0.2), so rect-
        # only, h>=0.4 training filed real DMR under FM/GMSK. Widen h down and
        # randomly shape (unit-DC-gain kernels preserve each symbol's deviation).
        h = rng.uniform(0.2, 1.0)
        mode = rng.choice(("none", "gauss", "rrc"))
        if mode == "gauss":
            freq = np.convolve(freq, _gaussian(rng.uniform(0.2, 0.5), sps), mode="same")
        elif mode == "rrc":
            k = _rrc(rng.uniform(0.2, 0.35), sps, span=6)
            freq = np.convolve(freq, k / k.sum(), mode="same")
    inst = freq * (h * np.pi / sps)
    return np.exp(1j * np.cumsum(inst))


def _gaussian(bt: float, sps: int, span: int = 4) -> np.ndarray:
    n = np.arange(-span * sps / 2, span * sps / 2 + 1, dtype=np.float64) / sps
    a = np.sqrt(np.log(2) / 2) / bt
    g = np.exp(-(np.pi ** 2) * (n ** 2) / (a ** 2) / 2)
    return g / g.sum()


def _gmsk(nsym: int, sps: int, rng) -> np.ndarray:
    b = rng.integers(0, 2, nsym) * 2.0 - 1.0
    rect = np.repeat(b, sps)
    shaped = np.convolve(rect, _gaussian(rng.uniform(0.25, 0.5), sps), mode="same")
    # h=0.5 -> pi/2 phase change per symbol, the defining property of GMSK.
    return np.exp(1j * np.cumsum(shaped * (np.pi / 2 / sps)))


def _msk(nsym: int, sps: int, rng) -> np.ndarray:
    b = np.repeat(rng.integers(0, 2, nsym) * 2.0 - 1.0, sps)
    return np.exp(1j * np.cumsum(b * (np.pi / 2 / sps)))


def _am(n: int, sps: int, rng) -> np.ndarray:
    """Analog AM with a band-limited message, not a single tone - a pure tone
    is trivially separable and teaches nothing about real voice channels."""
    msg = _lowpass_noise(n, rng, cutoff=rng.uniform(0.02, 0.08))
    depth = rng.uniform(0.3, 0.9)
    return (1.0 + depth * msg).astype(np.complex128)


def _fm(n: int, sps: int, rng) -> np.ndarray:
    msg = _lowpass_noise(n, rng, cutoff=rng.uniform(0.02, 0.08))
    kf = rng.uniform(0.5, 3.0)
    return np.exp(1j * np.cumsum(msg) * (kf * np.pi / sps))


def _lowpass_noise(n: int, rng, cutoff: float) -> np.ndarray:
    x = rng.standard_normal(n)
    f = np.fft.rfftfreq(n)
    X = np.fft.rfft(x)
    X[f > cutoff] = 0
    y = np.fft.irfft(X, n)
    p = np.max(np.abs(y))
    return y / p if p > 0 else y


def _ofdm(n: int, sps: int, rng) -> np.ndarray:
    """OFDM looks Gaussian in the time domain; the net must key on the cyclic
    prefix correlation, so the CP has to be generated properly."""
    nfft = int(rng.choice([64, 128, 256]))
    ncp = nfft // int(rng.choice([4, 8, 16]))
    used = int(nfft * rng.uniform(0.5, 0.9)) // 2 * 2
    out = []
    while sum(len(b) for b in out) < n:
        sym = np.zeros(nfft, dtype=np.complex128)
        c = _CONST["QAM16"]
        d = c[rng.integers(0, len(c), used)]
        half = used // 2
        sym[1:half + 1] = d[:half]
        sym[-half:] = d[half:]
        t = np.fft.ifft(sym) * np.sqrt(nfft)
        out.append(np.concatenate([t[-ncp:], t]))
    return np.concatenate(out)[:n]


def make_clean(kind: str, n: int, rng) -> np.ndarray:
    """One clean burst of `n` complex samples, unit average power."""
    sps = int(rng.integers(4, 17))          # 4..16 samples/symbol
    nsym = n // sps + 16
    if kind in _CONST:
        x = _linear(kind, nsym, sps, rng)
    elif kind == "2FSK":
        x = _fsk(2, nsym, sps, rng)
    elif kind == "4FSK":
        x = _fsk(4, nsym, sps, rng)
    elif kind == "GMSK":
        x = _gmsk(nsym, sps, rng)
    elif kind == "MSK":
        x = _msk(nsym, sps, rng)
    elif kind == "AM":
        x = _am(n + 64, sps, rng)
    elif kind == "FM":
        x = _fm(n + 64, sps, rng)
    elif kind == "OFDM":
        x = _ofdm(n + 64, sps, rng)
    else:
        raise ValueError(f"unknown class {kind}")
    # Trim transients from pulse-shaping / filter warm-up before slicing.
    if len(x) > n + 32:
        off = int(rng.integers(16, len(x) - n - 1))
        x = x[off:off + n]
    else:
        x = np.resize(x, n)
    p = np.sqrt(np.mean(np.abs(x) ** 2))
    return (x / p if p > 0 else x).astype(np.complex64)


def impair(x: np.ndarray, snr_db: float, rng, *, cfo=True, multipath=True) -> np.ndarray:
    """Channel + receiver impairments. Without these the net learns a clean
    textbook constellation and collapses on air."""
    y = x.astype(np.complex128)
    if multipath and rng.random() < 0.5:
        taps = int(rng.integers(2, 5))
        h = (rng.standard_normal(taps) + 1j * rng.standard_normal(taps)) / np.sqrt(2)
        h[0] = 1.0                              # keep a dominant direct path
        h /= np.sqrt(np.sum(np.abs(h) ** 2))
        y = np.convolve(y, h, mode="same")
    if cfo:
        # Residual carrier offset after the channel filter, in cycles/sample.
        fo = rng.uniform(-0.02, 0.02)
        y *= np.exp(2j * np.pi * fo * np.arange(len(y)))
        y *= np.exp(1j * rng.uniform(0, 2 * np.pi))
    p = np.mean(np.abs(y) ** 2)
    if p <= 0:
        return y.astype(np.complex64)
    npow = p / (10 ** (snr_db / 10.0))
    noise = np.sqrt(npow / 2) * (rng.standard_normal(len(y)) + 1j * rng.standard_normal(len(y)))
    return (y + noise).astype(np.complex64)


def to_tensor_layout(x: np.ndarray) -> np.ndarray:
    """complex64[N] -> float32[2, N], power-normalised (matches IQResNet1D)."""
    p = np.sqrt(np.mean(np.abs(x) ** 2))
    if p > 0:
        x = x / p
    return np.stack([x.real, x.imag]).astype(np.float32)


def make_batch(n_per_class: int, nsamp: int, snr_range=(0, 20), seed=None):
    """Balanced batch across every class, uniform SNR inside `snr_range`."""
    rng = np.random.default_rng(seed)
    X, y, snrs = [], [], []
    for ci, kind in enumerate(CLASSES):
        for _ in range(n_per_class):
            snr = rng.uniform(*snr_range)
            b = impair(make_clean(kind, nsamp, rng), snr, rng)
            X.append(to_tensor_layout(b))
            y.append(ci)
            snrs.append(snr)
    return (np.stack(X), np.array(y, dtype=np.int64), np.array(snrs, dtype=np.float32))
