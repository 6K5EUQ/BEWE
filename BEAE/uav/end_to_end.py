#!/usr/bin/env python3
"""End-to-end demonstration of the whole pipeline, no hardware required.

Runs the full chain against a self-generated example so the logic is visible
and reproducible:

  1. bind phrase -> UID (the secret we pretend not to know)
  2. Phase C: build a real SYNC packet, LoRa-modulate it, demodulate it back,
     and extract UID[4], UID[5] by CRC validation
  3. Phase B: synthesise a wideband snapshot of the true hop pattern (only the
     ~50 MHz we can see), channelise, and detect the hop events
  4. Phase A: from the SYNC-derived UID[4], UID[5] plus the detected hops,
     recover UID[2], UID[3] and confirm the full UID matches the secret

Run:  python end_to_end.py  [bind_phrase]
"""

import sys

import numpy as np

from uav_elrs import fhss, lora
from uav_elrs.detect import (Channelizer, ChannelizerConfig, bursts_to_observations,
                             detect_bursts, make_test_iq)
from uav_elrs.solve import SEED_MASK, simulate_observations, solve_from_sync
from uav_elrs.sync_packet import build_sync_packet, extract_uid_from_sync
from uav_elrs.uid import bind_phrase_to_uid, uid_to_seed


def main():
    phrase = sys.argv[1] if len(sys.argv) > 1 else "recon-target-7"

    # --- 0. the secret ------------------------------------------------------
    uid = bind_phrase_to_uid(phrase)
    seed = uid_to_seed(uid)
    print(f"[secret]  bind phrase = {phrase!r}")
    print(f"[secret]  UID = {uid.hex(' ')}   FHSS seed = 0x{seed:08X}")
    print()

    # --- 1. Phase C: SYNC packet -> LoRa -> demod -> UID[4], UID[5] ----------
    sf = 8  # 50 Hz rate
    pkt = build_sync_packet(uid4=uid[4], uid5=uid[5], fhss_index=23, nonce=77,
                            rf_rate_enum=29)  # 29 = RATE_LORA_2G4_500HZ
    codec = lora.LoRaFrameCodec(sf=sf)
    syms = codec.bytes_to_symbols(pkt)
    iq = lora.modulate_symbols(syms, sf, os=1, preamble_len=12, invert_iq=bool(uid[5] & 1))
    got_syms = lora.demodulate_symbols(iq, sf, len(syms), os=1, invert_iq=bool(uid[5] & 1))
    got_pkt = codec.symbols_to_bytes(got_syms, len(pkt))
    ext = extract_uid_from_sync(got_pkt)
    assert ext is not None, "SYNC CRC did not validate"
    print("[phase C] SYNC packet demodulated, CRC valid")
    print(f"[phase C]   UID[4]=0x{ext.uid4:02X}  UID[5]=0x{ext.uid5:02X}"
          f"  fhss_index={ext.fhss_index}  nonce={ext.nonce}")
    print()

    # --- 2. Phase B: wideband snapshot -> detected hop events ---------------
    # 61.44 MSPS centred at 2440 MHz sees ~50 of the 80 channels.
    cfg = ChannelizerConfig(fs=61.44e6, fc=2440.0e6, fft_size=4096, hop=2048)
    chz = Channelizer(cfg)
    visible = set(chz.channel_ids)
    print(f"[phase B] snapshot sees {len(visible)}/80 channels "
          f"({min(visible)}..{max(visible)})")

    hop_period = 8e-3  # 500 Hz LoRa 2G4: hop interval 4 * 2000 us
    true_seq = fhss.build_sequence(seed)
    n_hops = 60
    events = []
    for r in range(n_hops):
        ch = true_seq[(23 + r) % len(true_seq)]  # start at the SYNC's fhss_index
        if ch in visible:
            events.append((r * hop_period, 1.2e-3, ch))
    iq_wb = make_test_iq(events, cfg, n_hops * hop_period + 0.002, snr_db=22.0)
    times, energy_db, ch_ids = chz.energy(iq_wb)
    bursts = detect_bursts(times, energy_db, ch_ids, threshold_db=8.0)
    obs = bursts_to_observations(bursts, hop_period)
    print(f"[phase B] detected {len(bursts)} bursts -> {len(obs)} hop observations")
    print()

    # --- 3. Phase A: recover UID[2], UID[3] ---------------------------------
    sols = solve_from_sync(obs, ext.uid4, ext.uid5)
    print(f"[phase A] 2^16 search -> {len(sols)} solution(s)")
    if len(sols) == 1:
        s = sols[0]
        rec = bytes([s.uid2345[0], s.uid2345[1], s.uid2345[2], s.uid2345[3]])
        print(f"[phase A]   UID[2..5] = {rec.hex(' ')}  (UID[2] MSB free)")
        # compare against the secret, ignoring UID[0], UID[1] and UID[2] MSB
        ok = (s.seed == (seed & SEED_MASK))
        print(f"\n[result]  recovered seed 0x{s.seed:08X} == secret (mod 2^31): {ok}")
        print("[result]  UID[0], UID[1] are never transmitted -> unknowable by RF")
    else:
        print("[phase A]   ambiguous -- need more hops or wider coverage")


if __name__ == "__main__":
    main()
