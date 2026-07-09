"""Residual extraction (A): strip run-to-run nuisances, expose the hardware signature.

Given a gate-aligned preamble-region burst (the C++ capture triggers on the HDLC
payload gate, so every burst is framed consistently), we remove the parts that vary
per transmission and are NOT a stable fingerprint — carrier frequency offset (CFO),
amplitude gain, global phase — and keep the parts that ARE the transmitter's physical
signature: the turn-on transient envelope, phase noise, and GMSK modulation deviations.

Output is a 4-channel real representation over the crop plus a small aux vector.
This is intentionally reconstruction-free (no ideal-GMSK subtraction) for robustness;
a v2 can subtract the known preamble/flag ideal instantaneous-frequency pattern to
isolate pure phase-noise. Crop geometry mirrors dataset.crop_and_norm so the fair
time-split and MMSI-leak-free window are preserved.
"""
import numpy as np

N_CHANNELS = 4   # [I, Q, inst-freq, envelope-shape]
N_AUX = 1        # [cfo_kHz]


def _inst_freq(iq: np.ndarray) -> np.ndarray:
    """FM discriminator: angle(x[n] * conj(x[n-1])) — instantaneous frequency (rad/sample)."""
    d = np.zeros(len(iq), np.float32)
    if len(iq) > 1:
        d[1:] = np.angle(iq[1:] * np.conj(iq[:-1])).astype(np.float32)
    return d


def extract(iq_c: np.ndarray, start: int, length: int, nominal_sr: int):
    """iq_c: complex64 full burst, already resampled to nominal_sr.
    Returns (feat [N_CHANNELS, length] float32, aux [N_AUX] float32) or None.

    Runs IDENTICALLY in training and inference (keep in lockstep)."""
    if len(iq_c) < start + length:
        if len(iq_c) < start + length // 2:
            return None
        pad = np.zeros(start + length, np.complex64)
        pad[:len(iq_c)] = iq_c
        iq_c = pad
    seg = iq_c[start:start + length].astype(np.complex64)
    lo, hi = length // 4, length * 3 // 4          # stable (preamble/flag) region, skip ramp start

    # 1) coarse CFO — mean instantaneous frequency over the stable alternating preamble
    #    (0101... averages to ~0 dev, so the mean ~ carrier offset).
    ifr = _inst_freq(seg)
    cfo_rad = float(np.mean(ifr[lo:hi]))           # rad/sample
    cfo_hz = cfo_rad * nominal_sr / (2.0 * np.pi)

    # 2) derotate CFO
    n = np.arange(length, dtype=np.float32)
    derot = (seg * np.exp(-1j * cfo_rad * n)).astype(np.complex64)

    # 3) RMS normalize (remove gain)
    rms = float(np.sqrt(np.mean(np.abs(derot) ** 2))) + 1e-12
    derot = (derot / rms).astype(np.complex64)

    # 4) global phase align (rotate mean phase of stable region to 0)
    ref = complex(np.mean(derot[lo:hi]))
    if abs(ref) > 1e-9:
        derot = (derot * np.exp(-1j * np.angle(ref))).astype(np.complex64)

    # channels: residual I/Q (nuisance-stripped), inst-freq (phase-noise + GMSK), envelope shape
    ifr_d = _inst_freq(derot)
    amp = np.abs(seg).astype(np.float32)
    env = amp / (float(amp.mean()) + 1e-9) - 1.0   # mean-normalized envelope shape (ramp/PA)
    feat = np.stack([derot.real, derot.imag, ifr_d, env]).astype(np.float32)

    aux = np.array([cfo_hz / 1000.0], np.float32)  # kHz
    if not np.isfinite(feat).all() or not np.isfinite(aux).all():
        return None
    return feat, aux
