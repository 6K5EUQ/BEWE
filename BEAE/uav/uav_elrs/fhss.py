"""FHSS: LCG PRNG and hop-sequence build, ported 1:1 from ExpressLRS.

Source:
  lib/FHSS/random.cpp        rng() / rngSeed() / rngN()
  lib/FHSS/FHSS.cpp          domains[], FHSSrandomiseFHSSsequence(),
                             FHSSrandomiseFHSSsequenceBuild()
  lib/FHSS/FHSS.h            FHSS_SEQUENCE_LEN, channel/frequency math

The sequence is fully determined by the 32-bit seed (from uid.uid_to_seed).
This module has no dependency on IQ or hardware; it is pure integer math and
is the reference the solver checks candidate seeds against.
"""

from dataclasses import dataclass

FHSS_SEQUENCE_LEN = 256          # FHSS.h:23
FREQ_SPREAD_SCALE_SX128X = 256   # FHSS.h:20 (non-LR1121)
FREQ_STEP_SX128X = 52_000_000.0 / (2 ** 18)  # SX1280_Regs.h:13 = 198.3642578125 Hz


# ---- LCG PRNG (random.cpp) -------------------------------------------------
# rng(): seed = (214013*seed + 2531011) mod 2^31; return seed >> 16 (0..0x7FFF)
_LCG_M = 2147483648  # 2^31
_LCG_A = 214013
_LCG_C = 2531011


class Rng:
    """Exact port of lib/FHSS/random.cpp. Stateful; seed once, then pull."""

    __slots__ = ("seed",)

    def __init__(self, seed: int = 0):
        self.seed = seed & 0xFFFFFFFF

    def rng(self) -> int:
        # returns 0..0x7FFF
        self.seed = (_LCG_A * self.seed + _LCG_C) % _LCG_M
        return self.seed >> 16

    def rngN(self, max_: int) -> int:
        # returns 0 <= x < max_, with max_ < 256
        return self.rng() % max_


# ---- Regulatory domains (FHSS.cpp domains[] / domainsDualBand[]) ------------
@dataclass(frozen=True)
class Domain:
    name: str
    freq_start: int   # Hz
    freq_stop: int    # Hz
    freq_count: int
    freq_center: int  # Hz

    @property
    def sync_channel(self) -> int:
        # FHSS.cpp:93  sync_channel = freq_count / 2  (integer division)
        return self.freq_count // 2

    @property
    def freq_spread(self) -> int:
        # FHSS.cpp:94  (stop-start)*SCALE/(count-1)  -- integer arithmetic
        return ((self.freq_stop - self.freq_start) * FREQ_SPREAD_SCALE_SX128X) // (self.freq_count - 1)

    @property
    def band_count(self) -> int:
        # FHSS.cpp:95  primaryBandCount = (256/count)*count  -- whole blocks only
        return (FHSS_SEQUENCE_LEN // self.freq_count) * self.freq_count

    def channel_hz(self, channel: int) -> int:
        """Nominal channel centre in Hz (a clean freq_start + N * spacing grid).

        Caveat: the firmware stores freq_start/stop in *register units*
        (FREQ_HZ_TO_REG_VAL = freq / FREQ_STEP, FREQ_STEP = 52 MHz / 2^18 =
        198.3642578125 Hz) and truncates at each step, so the actual emitted RF
        is up to ~327 Hz below this nominal grid (< 0.04% of the 1 MHz channel
        spacing). Negligible for channel selection and for a channelizer whose
        FFT bins are ~15 kHz; use exact_channel_hz() if sub-kHz tuning matters.
        """
        return self.freq_start + (self.freq_spread * channel) // FREQ_SPREAD_SCALE_SX128X

    def exact_channel_hz(self, channel: int) -> int:
        """Firmware-exact RF centre: reproduces the register-domain truncation."""
        reg_start = int(self.freq_start / FREQ_STEP_SX128X)
        reg_stop = int(self.freq_stop / FREQ_STEP_SX128X)
        reg_spread = ((reg_stop - reg_start) * FREQ_SPREAD_SCALE_SX128X) // (self.freq_count - 1)
        reg_ch = reg_start + (reg_spread * channel) // FREQ_SPREAD_SCALE_SX128X
        return int(reg_ch * FREQ_STEP_SX128X)


# Sub-GHz (SX127x / LR1121 primary) -- FHSS.cpp:19-28
DOMAINS_SUBGHZ = {
    "AU915":  Domain("AU915",  915500000, 926900000, 20, 921000000),
    "FCC915": Domain("FCC915", 903500000, 926900000, 40, 915000000),
    "EU868":  Domain("EU868",  863275000, 869575000, 13, 868000000),
    "IN866":  Domain("IN866",  865375000, 866950000,  4, 866000000),
    "AU433":  Domain("AU433",  433420000, 434420000,  3, 434000000),
    "EU433":  Domain("EU433",  433100000, 434450000,  3, 434000000),
    "US433":  Domain("US433",  433250000, 438000000,  8, 434000000),
    "US433W": Domain("US433W", 423500000, 438000000, 20, 434000000),
}

# 2.4 GHz (SX128x) -- FHSS.cpp:45-53
ISM2G4 = Domain("ISM2G4", 2400400000, 2479400000, 80, 2440000000)
DOMAINS_2G4 = {"ISM2G4": ISM2G4}


def build_sequence(seed: int, domain: Domain = ISM2G4) -> list[int]:
    """Exact port of FHSSrandomiseFHSSsequenceBuild (FHSS.cpp:134-175).

    Returns the hop table as channel indices, length = domain.band_count
    (240 for ISM2G4). FHSSptr wraps mod this length, so the on-air hop
    pattern repeats with this period.
    """
    freq_count = domain.freq_count
    sync = domain.sync_channel
    n = domain.band_count

    rng = Rng(seed)
    seq = [0] * n

    # initialize (FHSS.cpp:146-155)
    for i in range(n):
        if i % freq_count == 0:
            seq[i] = sync
        elif i % freq_count == sync:
            seq[i] = 0
        else:
            seq[i] = i % freq_count

    # randomise within each block, never touching the block-start sync slot
    # (FHSS.cpp:157-170)
    for i in range(n):
        if i % freq_count != 0:
            offset = (i // freq_count) * freq_count
            rand = rng.rngN(freq_count - 1) + 1  # 1..freq_count-1
            seq[i], seq[offset + rand] = seq[offset + rand], seq[i]

    return seq


def sequence_frequencies_hz(seq: list[int], domain: Domain = ISM2G4) -> list[int]:
    """Map a channel-index sequence to absolute centre frequencies (Hz)."""
    return [domain.channel_hz(c) for c in seq]
