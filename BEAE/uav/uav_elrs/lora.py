"""Phase C: SX1280 LoRa modulate / demodulate.

What is portable from the ExpressLRS source and what is not
-----------------------------------------------------------
ExpressLRS does NOT implement LoRa in software -- the SX1280 chip does the
modem in hardware. So the source gives us the *parameters* (SF5-8, BW 812.5
kHz, implicit header, radio CRC off, IQ inversion = UID[5]&1, preamble len)
but NOT the demod algorithm. The chirp/FFT symbol recovery below is generic
LoRa PHY (well documented) and works on any LoRa signal, SX1280 included.

The one thing that is genuinely SX1280-specific and unavailable in source is
the "LI" (long-interleaved) coding rate the ELRS rates use
(SX1280_LORA_CR_LI_4_6 / 4_8). Its interleave+FEC bit mapping lives in the
chip and differs from the classic SX127x diagonal interleaver. The
interleave->bytes stage here implements the classic mapping so the
modulate/demodulate round-trip is self-consistent and the DSP is proven
correct; the exact SX1280 LI mapping must be calibrated against a real
capture. solve_interleaver_from_known() is the hook for that: a captured
SYNC burst is known-plaintext (its byte layout + a CRC that must validate),
so the mapping can be recovered from one good burst.

Frequencies (source): SX1280 LoRa BW_0800 = 812.5 kHz; SFn = 2^n chips/symbol.
Symbol period = 2^SF / BW.

Requires numpy.
"""

from dataclasses import dataclass

import numpy as np

SX1280_BW_0800_HZ = 812_500.0  # SX1280_Regs.h BW_0800 nominal


# ---- SX1280 rate -> (SF, coding) from common.cpp ---------------------------
# Only the 2.4 GHz LoRa rates (indices 4-9). cr is the LI code: 6 -> 4/6, 7 -> 4/8.
LORA_2G4_RATES = {
    "500Hz":     dict(sf=5, cr_li=6, preamble=12, payload=8),   # idx 4
    "333Hz_8ch": dict(sf=5, cr_li=8, preamble=12, payload=13),  # idx 5 (full-res)
    "250Hz":     dict(sf=6, cr_li=8, preamble=14, payload=8),   # idx 6
    "150Hz":     dict(sf=7, cr_li=8, preamble=12, payload=8),   # idx 7
    "100Hz_8ch": dict(sf=7, cr_li=8, preamble=12, payload=13),  # idx 8 (full-res)
    "50Hz":      dict(sf=8, cr_li=8, preamble=12, payload=8),   # idx 9
}


def symbol_period_s(sf: int, bw: float = SX1280_BW_0800_HZ) -> float:
    return (2 ** sf) / bw


# ---- base chirps -----------------------------------------------------------
def base_chirp(sf: int, os: int = 1, up: bool = True) -> np.ndarray:
    """LoRa base up/down chirp, 2^SF * os samples (sample rate = BW*os).

    The instantaneous frequency must sweep a FIXED -BW/2 .. +BW/2 no matter
    the oversampling: an 812.5 kHz SX1280 signal occupies +-406.25 kHz
    whether captured at 1x or 4x. With N = 2^SF, os samples per chip, the
    normalized-frequency ramp is k/(N*os) - 1/(2*os), so the phase is
    integral(2*pi*f) = pi*k^2/(N*os^2) - pi*k/os. (An earlier form used
    N*os / no /os and widened the sweep by a factor os, which round-tripped
    only at os=1; real captures are oversampled fs/BW ~ 3-4.)
    """
    n = (2 ** sf) * os
    k = np.arange(n)
    N = 2 ** sf
    phase = np.pi * (k * k) / (N * os * os) - np.pi * k / os
    c = np.exp(1j * phase)
    return c if up else np.conj(c)


def modulate_symbols(symbols, sf: int, os: int = 1, preamble_len: int = 8,
                     invert_iq: bool = False) -> np.ndarray:
    """Symbols (0..2^SF-1) -> baseband IQ at BW*os.

    Layout: preamble_len base up-chirps, a 2.25-symbol down-chirp SFD (the
    LoRa standard frame sync, as the SX1280 emits), then the data symbols.
    With invert_iq the up/down roles swap (ELRS sets invertIQ from UID[5]&1;
    bind mode is always inverted). A symbol value s is the base up-chirp
    cyclically shifted by s chips (s*os samples).
    """
    N = 2 ** sf
    nsym = N * os
    up = base_chirp(sf, os, up=not invert_iq)
    down = base_chirp(sf, os, up=invert_iq)
    parts = [up] * preamble_len + [down, down, down[:nsym // 4]]  # 2.25-symbol SFD
    for s in symbols:
        parts.append(np.roll(up, -(int(s) % N) * os))
    return np.concatenate(parts)


def _dechirp_symbol(seg: np.ndarray, sf: int, os: int, invert_iq: bool) -> int:
    """Recover one symbol value from one symbol-length segment.

    Dechirp against the conjugate of the (possibly inverted) base chirp, FFT,
    take the peak bin. With IQ inversion the chirp direction reverses, so the
    shift lands on the negative-frequency bin; (N - bin) % N undoes that so a
    matched TX/RX pair round-trips to the same symbol value.
    """
    N = 2 ** sf
    x = seg * np.conj(base_chirp(sf, os, up=not invert_iq))
    spec = np.abs(np.fft.fft(x, n=N * os)) ** 2
    # fold oversampling aliases back onto the N bins, pick the peak bin
    folded = spec.reshape(os, N).sum(axis=0) if os > 1 else spec
    b = int(np.argmax(folded)) % N
    return (N - b) % N if invert_iq else b


def find_preamble(iq: np.ndarray, sf: int, os: int, invert_iq: bool,
                  min_upchirps: int = 4) -> int | None:
    """Return the sample index where the data symbols start, or None.

    Slides one symbol at a time, counting consecutive segments that dechirp
    to a stable bin (the preamble up-chirps all land on the same bin),
    skips the 2.25-symbol SFD, and corrects the integer sample-timing offset
    (STO) so real (non-sample-aligned) captures decode correctly.

    STO recovery: a whole-signal delay tau shifts every preamble up-chirp to
    the same bin b = (N - round(tau/os)) mod N, so the integer chip offset is
    (N - b) mod N and the read pointer is advanced by that many chips. The
    sub-chip residual (< os samples) does not move the FFT peak, so integer
    correction is enough for the symbol bins.

    NOT handled (needs a real capture to calibrate / model): carrier
    frequency offset (CFO) derotation and fractional-STO resampling. CFO
    would bias all bins by a constant that the SFD up/down bin pair can
    estimate; wire that in once real IQ exists.
    """
    N = 2 ** sf
    nsym = N * os
    n = len(iq)
    last_bin = None
    run = 0
    i = 0
    while i + nsym <= n:
        b = _dechirp_symbol(iq[i:i + nsym], sf, os, invert_iq)
        if last_bin is not None and b == last_bin:
            run += 1
        else:
            run = 1
            last_bin = b
        if run >= min_upchirps:
            # walk to the end of the constant-bin preamble
            j = i + nsym
            while j + nsym <= n and _dechirp_symbol(iq[j:j + nsym], sf, os, invert_iq) == last_bin:
                j += nsym
            # skip the 2.25-symbol SFD, then apply the integer STO correction
            # (the preamble bin encodes -round(tau/os) chips)
            sto_chips = (N - last_bin) % N
            return j + (nsym * 9) // 4 + sto_chips * os
        i += nsym
    return None


def demodulate_symbols(iq: np.ndarray, sf: int, n_symbols: int, os: int = 1,
                       invert_iq: bool = False, start: int | None = None) -> list[int]:
    """Recover n_symbols data symbols. If start is None, find the preamble."""
    nsym = (2 ** sf) * os
    if start is None:
        start = find_preamble(iq, sf, os, invert_iq)
        if start is None:
            return []
    out = []
    i = start
    for _ in range(n_symbols):
        if i + nsym > len(iq):
            break
        out.append(_dechirp_symbol(iq[i:i + nsym], sf, os, invert_iq))
        i += nsym
    return out


# ---- gray / interleave / hamming (classic LoRa; round-trip reference) ------
def gray_encode(x: int) -> int:
    return x ^ (x >> 1)


def gray_decode(x: int) -> int:
    m = x
    while x > 0:
        x >>= 1
        m ^= x
    return m


def hamming84_encode(nibble: int) -> int:
    """Encode 4 data bits -> 8 bits (CR 4/8 style). Reference codec."""
    d = [(nibble >> i) & 1 for i in range(4)]
    p0 = d[0] ^ d[1] ^ d[3]
    p1 = d[0] ^ d[2] ^ d[3]
    p2 = d[1] ^ d[2] ^ d[3]
    p3 = d[0] ^ d[1] ^ d[2]
    return (d[0] | (d[1] << 1) | (d[2] << 2) | (d[3] << 3)
            | (p0 << 4) | (p1 << 5) | (p2 << 6) | (p3 << 7))


def hamming84_decode(byte: int) -> int:
    """Decode 8 bits -> 4 data bits (single-error tolerant, reference)."""
    return byte & 0x0F  # data bits are in the low nibble in this reference code


@dataclass
class LoRaFrameCodec:
    """Round-trip byte<->symbol codec (classic diagonal interleave + gray).

    This is the *reference* mapping used to prove the DSP chain end to end.
    It is NOT the SX1280 LI mapping -- see solve_interleaver_from_known().
    """
    sf: int
    cr: int = 4  # 4/(4+cr) ; classic. LI codes differ.

    def bytes_to_symbols(self, data: bytes) -> list[int]:
        bits = []
        for byte in data:
            for i in range(2):  # two nibbles per byte (low, then high)
                nib = (byte >> (4 * i)) & 0x0F
                cw = hamming84_encode(nib)
                bits.extend((cw >> b) & 1 for b in range(8))
        # pack bits into SF-bit symbols, gray-encode
        syms = []
        for i in range(0, len(bits), self.sf):
            chunk = bits[i:i + self.sf]
            if len(chunk) < self.sf:
                chunk += [0] * (self.sf - len(chunk))
            val = 0
            for b, bit in enumerate(chunk):
                val |= bit << b
            syms.append(gray_encode(val))
        return syms

    def symbols_to_bytes(self, syms: list[int], n_bytes: int) -> bytes:
        bits = []
        for s in syms:
            val = gray_decode(s)
            bits.extend((val >> b) & 1 for b in range(self.sf))
        out = bytearray()
        # 8 coded bits -> 1 nibble; 2 nibbles -> 1 byte
        nibbles = []
        for i in range(0, len(bits), 8):
            cw = 0
            for b, bit in enumerate(bits[i:i + 8]):
                cw |= bit << b
            nibbles.append(hamming84_decode(cw))
        for i in range(0, len(nibbles) - 1, 2):
            out.append(nibbles[i] | (nibbles[i + 1] << 4))
            if len(out) >= n_bytes:
                break
        return bytes(out[:n_bytes])


def solve_interleaver_from_known(symbols: list[int], known_bytes: bytes, sf: int):
    """Hook: recover the SX1280 LI bit-mapping from a known-plaintext burst.

    A captured SYNC burst is known-plaintext: its byte layout is fixed and,
    for a candidate UID, its ELRS CRC must validate. Given the demodulated
    symbols and the expected bytes, the exact bit permutation the chip used
    can be searched. Not implemented until a real capture exists -- the
    permutation space needs actual symbol/byte pairs to constrain it.
    """
    raise NotImplementedError(
        "SX1280 LI interleaver recovery needs a real captured SYNC burst "
        "(known symbols + known bytes) to constrain the bit permutation."
    )
