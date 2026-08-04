"""Phase C: LoRa DSP round-trip and the full SYNC extraction pipeline.

Proves the chirp/FFT symbol chain and the byte codec are exact inverses, and
that a built-and-modulated SYNC packet demodulates back to bytes whose ELRS
CRC validates and yields the correct UID[4], UID[5]. The byte codec here is
the reference (classic) interleaver, not the SX1280 LI mapping -- see the
note in lora.py -- but the DSP and the packet/CRC/UID chain are exact.
"""

import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from uav_elrs import lora
from uav_elrs.detect import (ChannelizerConfig, Channelizer, bursts_to_observations,
                             detect_bursts, make_test_iq)
from uav_elrs.sync_packet import build_sync_packet, extract_uid_from_sync
from uav_elrs.uid import bind_phrase_to_uid


def test_symbol_round_trip():
    for sf in (5, 6, 7, 8):
        for invert in (False, True):
            N = 2 ** sf
            syms = [0, 1, 2, N - 1, N // 2, 7 % N, 13 % N]
            iq = lora.modulate_symbols(syms, sf, os=1, preamble_len=8,
                                       invert_iq=invert)
            # start=None exercises find_preamble on a sample-aligned signal
            got = lora.demodulate_symbols(iq, sf, len(syms), os=1, invert_iq=invert)
            assert got == [s % N for s in syms], f"sf={sf} inv={invert}: {got} vs {syms}"


def test_oversampled_round_trip():
    """Regression for the base_chirp bug: the chirp must sweep a fixed +-BW/2
    regardless of oversampling, so os=2,3,4 round-trip as well as os=1."""
    for sf in (6, 7):
        N = 2 ** sf
        syms = [1, 2, 3, N - 1, N // 2, 100 % N]
        for os_ in (1, 2, 3, 4):
            for invert in (False, True):
                iq = lora.modulate_symbols(syms, sf, os=os_, preamble_len=8,
                                           invert_iq=invert)
                start = 8 * N * os_ + (9 * N * os_) // 4  # preamble + 2.25-sym SFD
                got = lora.demodulate_symbols(iq, sf, len(syms), os=os_,
                                              invert_iq=invert, start=start)
                assert got == [s % N for s in syms], f"os={os_} inv={invert}: {got}"


def test_sto_recovery():
    """find_preamble recovers integer-chip sample-timing offset (STO).

    Sample-aligned always works; whole-chip offsets (tau a multiple of os) are
    corrected from the preamble bin. Sub-chip offsets (tau not a multiple of
    os) are out of scope for integer STO -- they need fractional resampling,
    which is real-capture calibration work (see find_preamble docstring)."""
    import numpy as np
    sf, os_ = 7, 2
    N = 2 ** sf
    syms = [10, 20, 30, 40, 50]
    iq = lora.modulate_symbols(syms, sf, os=os_, preamble_len=10)
    for tau in (0, os_, os_ * 3, os_ * 8, os_ * 16):  # whole-chip offsets
        d = np.concatenate([np.zeros(tau, dtype=complex), iq])
        got = lora.demodulate_symbols(d, sf, len(syms), os=os_)
        assert got == syms, f"tau={tau}: {got}"


def test_preamble_sync():
    sf = 7
    N = 2 ** sf
    syms = [10, 20, 30, 40]
    iq = lora.modulate_symbols(syms, sf, os=1, preamble_len=10, invert_iq=False)
    got = lora.demodulate_symbols(iq, sf, len(syms), os=1, invert_iq=False)
    assert got == syms


def test_gray_round_trip():
    for x in range(256):
        assert lora.gray_decode(lora.gray_encode(x)) == x


def test_frame_codec_round_trip():
    codec = lora.LoRaFrameCodec(sf=7)
    data = bytes([0x02, 0x05, 0x99, 0x09, 0x00, 0x11, 0x22, 0x00])
    syms = codec.bytes_to_symbols(data)
    back = codec.symbols_to_bytes(syms, len(data))
    assert back == data


def test_full_sync_pipeline():
    """Build a real SYNC packet -> codec -> modulate -> demod -> bytes ->
    extract_uid_from_sync validates the CRC and returns the true UID[4:5]."""
    uid = bind_phrase_to_uid("pipeline-uid")
    pkt = build_sync_packet(uid4=uid[4], uid5=uid[5], fhss_index=17, nonce=99,
                            rf_rate_enum=25, model_id=0xFF, full_res=False)
    sf = 8
    codec = lora.LoRaFrameCodec(sf=sf)
    syms = codec.bytes_to_symbols(pkt)
    iq = lora.modulate_symbols(syms, sf, os=1, preamble_len=8, invert_iq=False)
    got_syms = lora.demodulate_symbols(iq, sf, len(syms), os=1)  # find_preamble
    got_bytes = codec.symbols_to_bytes(got_syms, len(pkt))
    assert got_bytes == pkt
    ext = extract_uid_from_sync(got_bytes)
    assert ext is not None
    assert (ext.uid4, ext.uid5) == (uid[4], uid[5])
    assert ext.fhss_index == 17 and ext.nonce == 99


def test_channelizer_detects_bursts():
    """Phase B DSP: synth tone bursts -> channelizer -> detect -> observations."""
    cfg = ChannelizerConfig(fs=61.44e6, fc=2440.0e6, fft_size=4096, hop=2048)
    hop_period = 8e-3  # 500 Hz LoRa 2G4: hop 4 * 2000 us
    true_channels = [40, 42, 15, 60, 41]  # channels the hops land on
    events = []
    for i, ch in enumerate(true_channels):
        events.append((i * hop_period, 1.5e-3, ch))
    duration = len(true_channels) * hop_period + 0.002
    iq = make_test_iq(events, cfg, duration, snr_db=25.0)
    chz = Channelizer(cfg)
    times, energy_db, ch_ids = chz.energy(iq)
    bursts = detect_bursts(times, energy_db, ch_ids, threshold_db=8.0)
    # every observable true channel should be detected
    seen = {b.channel for b in bursts}
    observable = {c for c in true_channels if c in ch_ids}
    assert observable <= seen, f"missed {observable - seen}"
    obs = bursts_to_observations(bursts, hop_period)
    # observations map indices to the right channels for observable ones
    for i, ch in enumerate(true_channels):
        if ch in ch_ids and i in obs:
            assert obs[i] == ch


if __name__ == "__main__":
    for name in ["test_symbol_round_trip", "test_oversampled_round_trip",
                 "test_sto_recovery", "test_preamble_sync",
                 "test_gray_round_trip", "test_frame_codec_round_trip",
                 "test_full_sync_pipeline", "test_channelizer_detects_bursts"]:
        globals()[name]()
        print(f"ok  {name}")
    print("LORA/DETECT: all passed")
