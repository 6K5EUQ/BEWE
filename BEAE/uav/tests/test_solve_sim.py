"""Phase A validation: does the LCG seed recover uniquely from partial hops?

This is the gate for the whole approach. We take a real UID, generate its
true hop sequence, keep only the hops that fall in a contiguous channel
window (modelling a ~50 MHz / 61.44 MSPS snapshot over the 79 MHz band), and
check that solve_from_sync recovers exactly one seed. We also sweep coverage
and hop count and print the minimum hops needed for a unique answer.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from uav_elrs.fhss import ISM2G4
from uav_elrs.solve import SEED_MASK, simulate_observations, solve_from_sync
from uav_elrs.uid import bind_phrase_to_uid, uid_to_seed


def _window(n_ch: int) -> set[int]:
    # contiguous block of channels the snapshot can see (e.g. 50 of 80)
    return set(range(n_ch))


def test_unique_recovery_50pct():
    uid = bind_phrase_to_uid("field-test-1")
    seed = uid_to_seed(uid)
    obs = simulate_observations(seed, n_hops=240, observed_channels=_window(50),
                                start_offset=0)
    sols = solve_from_sync(obs, uid[4], uid[5])
    # unique up to the UID[2] MSB free bit (LoRa mode; see Solution docstring)
    assert len(sols) == 1, f"expected unique, got {len(sols)}"
    assert sols[0].seed == (seed & SEED_MASK)
    assert sols[0].uid2345 == (uid[2] & 0x7F, uid[3], uid[4], uid[5])
    assert sols[0].uid2_msb_free


def test_offset_recovered():
    uid = bind_phrase_to_uid("phase-check")
    seed = uid_to_seed(uid)
    obs = simulate_observations(seed, n_hops=240, observed_channels=_window(50),
                                start_offset=137)
    sols = solve_from_sync(obs, uid[4], uid[5])
    assert len(sols) == 1
    assert sols[0].offset == 137


def test_min_hops_table():
    """Sweep coverage x hop-count; print min hops for a unique solution."""
    phrases = ["alpha", "bravo", "charlie", "delta", "echo"]
    print("\n  coverage | min hops for unique UID (median over 5 links)")
    for n_ch in (40, 50, 60, 70):
        needed = []
        for ph in phrases:
            uid = bind_phrase_to_uid(ph)
            seed = uid_to_seed(uid)
            win = _window(n_ch)
            found_at = None
            for n_hops in range(20, 241, 10):
                obs = simulate_observations(seed, n_hops, win, start_offset=0)
                sols = solve_from_sync(obs, uid[4], uid[5])
                if len(sols) == 1 and sols[0].seed == (seed & SEED_MASK):
                    found_at = n_hops
                    break
            needed.append(found_at if found_at is not None else 999)
        needed.sort()
        med = needed[len(needed) // 2]
        print(f"    {n_ch}/80  | {med} hops"
              + ("" if med < 999 else "  (not unique within 240)"))
        # 50-channel coverage must resolve within the 240-hop period
        if n_ch >= 50:
            assert med <= 240, f"{n_ch}ch did not resolve"


if __name__ == "__main__":
    for name in ["test_unique_recovery_50pct", "test_offset_recovered",
                 "test_min_hops_table"]:
        globals()[name]()
        print(f"ok  {name}")
    print("SOLVE: all passed")
