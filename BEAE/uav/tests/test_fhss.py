"""FHSS: LCG and sequence-build match the ExpressLRS source guarantees."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from uav_elrs.fhss import (FHSS_SEQUENCE_LEN, ISM2G4, Rng, build_sequence,
                           sequence_frequencies_hz)


def test_lcg_matches_c():
    # random.cpp: seed=(214013*seed+2531011)%2^31; return seed>>16
    r = Rng(0)
    seed = 0
    for _ in range(1000):
        seed = (214013 * seed + 2531011) % 2147483648
        assert r.rng() == (seed >> 16)


def test_ism2g4_constants():
    assert ISM2G4.freq_count == 80
    assert ISM2G4.sync_channel == 40
    assert ISM2G4.band_count == 240          # (256//80)*80
    assert ISM2G4.channel_hz(0) == 2400400000
    assert ISM2G4.channel_hz(40) == 2440400000   # sync channel
    assert ISM2G4.channel_hz(79) == 2479400000
    # exact 1 MHz spacing
    for k in range(1, 80):
        assert ISM2G4.channel_hz(k) - ISM2G4.channel_hz(k - 1) == 1_000_000


def test_sequence_guarantees():
    seq = build_sequence(0xDEADBEEF, ISM2G4)
    fc = ISM2G4.freq_count
    sync = ISM2G4.sync_channel
    assert len(seq) == ISM2G4.band_count
    # 1. sync channel at every block start
    for i in range(0, len(seq), fc):
        assert seq[i] == sync
    # 2. no repeats within a block, and each block is a permutation of 0..fc-1
    for b in range(len(seq) // fc):
        block = seq[b * fc:(b + 1) * fc]
        assert sorted(block) == list(range(fc))
    # 3. every channel appears equally (band_count/fc times)
    from collections import Counter
    counts = Counter(seq)
    assert set(counts.values()) == {ISM2G4.band_count // fc}


def test_deterministic():
    assert build_sequence(12345) == build_sequence(12345)
    assert build_sequence(12345) != build_sequence(12346)


def test_frequencies():
    seq = build_sequence(1)
    freqs = sequence_frequencies_hz(seq)
    assert all(2400400000 <= f <= 2479400000 for f in freqs)


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"ok  {name}")
    print("FHSS: all passed")
