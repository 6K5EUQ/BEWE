"""Phase B: wideband IQ -> per-channel energy -> hop events.

Turns a 61.44 MSPS (or any) complex recording into the {rel_index: channel}
map that solve.py consumes. An STFT is used as a lightweight channelizer:
the 80 ISM2G4 channels sit on an exact 1 MHz grid, so each falls cleanly on
a group of FFT bins. (A polyphase filterbank would be a little sharper and is
the natural upgrade; the STFT is enough for 1 MHz-spaced channels.)

Includes make_test_iq() so the whole chain is testable without hardware: it
synthesises tone bursts on chosen channels at chosen times, and detect_*()
must recover them.

Requires numpy.
"""

from dataclasses import dataclass, field

import numpy as np

from .fhss import ISM2G4, Domain


@dataclass
class ChannelizerConfig:
    fs: float                 # sample rate (Hz), e.g. 61.44e6
    fc: float                 # IQ centre frequency (Hz), e.g. 2440.0e6
    domain: Domain = ISM2G4
    fft_size: int = 4096      # STFT window; 61.44e6/4096 ~ 15 kHz/bin
    hop: int = 2048           # STFT hop (samples between frames)
    window: str = "hann"


@dataclass
class Channelizer:
    cfg: ChannelizerConfig
    channel_ids: list[int] = field(default_factory=list)   # observable channel indices
    _bin_lo: list[int] = field(default_factory=list)
    _bin_hi: list[int] = field(default_factory=list)

    def __post_init__(self):
        cfg = self.cfg
        half = cfg.fs / 2.0
        spacing = 1_000_000.0  # ISM2G4 exact 1 MHz
        # FFT bin frequencies (baseband), fftshifted so index 0 = -fs/2
        freqs = np.fft.fftshift(np.fft.fftfreq(cfg.fft_size, d=1.0 / cfg.fs))
        for k in range(cfg.domain.freq_count):
            f_bb = cfg.domain.channel_hz(k) - cfg.fc  # baseband offset
            if abs(f_bb) > half - spacing / 2:
                continue  # channel outside the snapshot
            lo = np.searchsorted(freqs, f_bb - spacing / 2)
            hi = np.searchsorted(freqs, f_bb + spacing / 2)
            if hi <= lo:
                continue
            self.channel_ids.append(k)
            self._bin_lo.append(int(lo))
            self._bin_hi.append(int(hi))

    def energy(self, iq: np.ndarray):
        """STFT energy per observable channel.

        Returns (times_s, energy[n_frames, n_channels], channel_ids) with
        energy in dB relative to its own median floor per channel.
        """
        cfg = self.cfg
        nfft = cfg.fft_size
        win = np.hanning(nfft) if cfg.window == "hann" else np.ones(nfft)
        n_frames = 1 + max(0, (len(iq) - nfft) // cfg.hop)
        n_ch = len(self.channel_ids)
        energy = np.empty((n_frames, n_ch), dtype=np.float64)
        for fidx in range(n_frames):
            start = fidx * cfg.hop
            seg = iq[start:start + nfft]
            if len(seg) < nfft:
                seg = np.concatenate([seg, np.zeros(nfft - len(seg), dtype=seg.dtype)])
            spec = np.fft.fftshift(np.fft.fft(seg * win))
            psd = (spec.real ** 2 + spec.imag ** 2)
            for c in range(n_ch):
                energy[fidx, c] = psd[self._bin_lo[c]:self._bin_hi[c]].sum()
        times = (np.arange(n_frames) * cfg.hop + nfft / 2.0) / cfg.fs
        # per-channel dB above own median (robust floor)
        floor = np.median(energy, axis=0)
        floor[floor <= 0] = 1e-12
        energy_db = 10.0 * np.log10(np.maximum(energy, 1e-12) / floor)
        return times, energy_db, list(self.channel_ids)


@dataclass
class Burst:
    t_start: float
    t_end: float
    channel: int
    peak_db: float


def detect_bursts(times: np.ndarray, energy_db: np.ndarray, channel_ids: list[int],
                  threshold_db: float = 10.0, min_frames: int = 1) -> list[Burst]:
    """Threshold each channel's energy track into contiguous bursts."""
    bursts: list[Burst] = []
    dt = float(times[1] - times[0]) if len(times) > 1 else 0.0
    for c, ch in enumerate(channel_ids):
        track = energy_db[:, c]
        above = track >= threshold_db
        i = 0
        n = len(above)
        while i < n:
            if not above[i]:
                i += 1
                continue
            j = i
            while j < n and above[j]:
                j += 1
            if (j - i) >= min_frames:
                bursts.append(Burst(
                    t_start=float(times[i]),
                    t_end=float(times[j - 1] + dt),
                    channel=ch,
                    peak_db=float(track[i:j].max()),
                ))
            i = j
    bursts.sort(key=lambda b: b.t_start)
    return bursts


def bursts_to_observations(bursts: list[Burst], hop_period_s: float,
                           t0: float | None = None) -> dict[int, int]:
    """Assign each burst a relative hop index from its timestamp.

    hop_period_s = FHSShopInterval * packet_interval (e.g. 500 Hz LoRa 2G4:
    hop 4 * 2000 us = 8 ms). FHSS transmits on exactly one channel per hop, so
    when several bursts round to the same index (spectral leakage of a strong
    tone into adjacent channels) the loudest one is the real channel; the
    others are kept out.
    """
    if not bursts:
        return {}
    if t0 is None:
        t0 = bursts[0].t_start
    best: dict[int, Burst] = {}
    for b in bursts:
        r = int(round((b.t_start - t0) / hop_period_s))
        if r not in best or b.peak_db > best[r].peak_db:
            best[r] = b
    return {r: b.channel for r, b in best.items()}


# ---- synthetic IQ for tests (no hardware) ----------------------------------
def make_test_iq(events, cfg: ChannelizerConfig, duration_s: float,
                 snr_db: float = 20.0, seed: int = 0) -> np.ndarray:
    """Build a complex IQ vector with tone bursts.

    events: iterable of (t_start_s, duration_s, channel_index). Each becomes a
    complex sinusoid at that channel's baseband offset. White noise sets SNR.
    """
    rng = np.random.default_rng(seed)
    n = int(duration_s * cfg.fs)
    noise = (rng.standard_normal(n) + 1j * rng.standard_normal(n)) / np.sqrt(2.0)
    iq = noise.astype(np.complex128)
    amp = 10.0 ** (snr_db / 20.0)
    for (t_start, dur, ch) in events:
        f_bb = cfg.domain.channel_hz(ch) - cfg.fc
        i0 = int(t_start * cfg.fs)
        i1 = min(n, i0 + int(dur * cfg.fs))
        if i1 <= i0:
            continue
        t = np.arange(i1 - i0) / cfg.fs
        iq[i0:i1] += amp * np.exp(2j * np.pi * f_bb * t)
    return iq
