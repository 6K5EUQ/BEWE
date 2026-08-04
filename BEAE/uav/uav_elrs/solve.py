"""Solver: hop observations -> LCG seed -> UID.

The hop table repeats with period N = domain.band_count (240 for ISM2G4) and
FHSSptr advances by exactly 1 per hop. So a set of timed burst detections
becomes a map {relative_hop_index: observed_channel}, some indices missing
(the 38% of hops that fall outside the ~50 MHz snapshot). For each candidate
seed we rebuild the table and ask whether any phase offset makes every
observation line up. The correct seed is the only one that stays consistent
once enough hops are seen; test_solve_sim quantifies how many.

Realistic pipeline: Phase C demodulates one SYNC packet -> UID[4], UID[5]
(pins the low 16 bits of the seed) and fhss_index (pins the phase). The
search then covers only UID[2], UID[3] = 2^16 candidates, seconds in Python.
The fully-blind 2^32 search is provided but is meant to be narrowed first or
run from a compiled helper.

No IQ or hardware here -- operates on the (index, channel) events that
detect.py produces (or that a test simulates).
"""

from dataclasses import dataclass

from .fhss import ISM2G4, Domain, build_sequence
from .uid import candidate_seeds_from_uid_low, seed_to_uid2345


SEED_MASK = 0x7FFFFFFF  # LCG is mod 2^31 -> seed bit 31 has no effect


@dataclass
class Solution:
    seed: int                        # canonical (low 31 bits); bit 31 is a free bit
    offset: int                      # FHSSptr at relative index 0
    uid2345: tuple[int, int, int, int]  # uid2 has its MSB cleared (see below)
    uid2_msb_free: bool = True
    # In LoRa mode the FHSS seed is effectively 31-bit: seed bit 31 = UID[2]
    # bit 7 does not change any hop, and the CRC init does not use UID[2] at
    # all, so UID[2]^0x80 is an equally valid, operationally identical UID.
    # (FLRC mode would disambiguate via its 32-bit sync word; LoRa has none.)


def _positions_by_channel(seq: list[int], n_channels: int) -> list[list[int]]:
    """Index: channel -> list of positions in the sequence holding it."""
    by_ch: list[list[int]] = [[] for _ in range(n_channels)]
    for pos, ch in enumerate(seq):
        by_ch[ch].append(pos)
    return by_ch


def consistent_offsets(seq: list[int], observations: dict[int, int],
                       n_channels: int) -> list[int]:
    """Offsets o such that seq[(o + r) % N] == c for every observed (r, c).

    Anchored on the first observation so each seed costs O(3 * n_obs) instead
    of O(N * n_obs): a channel occurs N/n_channels (=3) times in the table.
    """
    if not observations:
        return []
    n = len(seq)
    items = sorted(observations.items())
    r0, c0 = items[0]
    by_ch = _positions_by_channel(seq, n_channels)
    out = []
    for pos in by_ch[c0]:
        o = (pos - r0) % n
        if all(seq[(o + r) % n] == c for r, c in items):
            out.append(o)
    return out


def solve_uid(observations: dict[int, int], seed_iter, domain: Domain = ISM2G4,
              max_solutions: int = 8) -> list[Solution]:
    """Search seed_iter for seeds consistent with the observations.

    Returns every consistent (seed, offset). len == 1 means the UID is
    uniquely recovered; >1 means ambiguous (need more hops); 0 means no
    candidate in the iterable fits (wrong UID-low, or bad detections).
    Stops early once max_solutions are found (guard against a runaway blind
    search returning millions).
    """
    n_channels = domain.freq_count
    sols: list[Solution] = []
    seen: set[tuple[int, int]] = set()   # (canonical seed, offset) -- dedupe bit-31 twins
    for seed in seed_iter:
        seq = build_sequence(seed, domain)
        for o in consistent_offsets(seq, observations, n_channels):
            canon = seed & SEED_MASK
            key = (canon, o)
            if key in seen:
                continue
            seen.add(key)
            u2, u3, u4, u5 = seed_to_uid2345(canon)  # canon has bit31=0 -> u2 MSB=0
            sols.append(Solution(canon, o, (u2 & 0x7F, u3, u4, u5)))
            if len(sols) >= max_solutions:
                return sols
    return sols


def solve_from_sync(observations: dict[int, int], uid4: int, uid5: int,
                    domain: Domain = ISM2G4) -> list[Solution]:
    """2^16 search once a SYNC packet has pinned UID[4], UID[5]."""
    return solve_uid(observations, candidate_seeds_from_uid_low(uid4, uid5), domain)


def solve_blind(observations: dict[int, int], domain: Domain = ISM2G4,
                seed_range=range(0, 1 << 32), max_solutions: int = 8) -> list[Solution]:
    """Fully-blind 2^32 search. Heavy in pure Python -- narrow with a SYNC
    packet (solve_from_sync) first, or slice seed_range across workers / a
    compiled port. Provided for completeness and for small-range unit tests.
    """
    return solve_uid(observations, iter(seed_range), domain, max_solutions)


# ---- synthetic observation generator (for tests / what-if, no IQ) ----------
def simulate_observations(seed: int, n_hops: int, observed_channels: set[int],
                          start_offset: int = 0,
                          domain: Domain = ISM2G4) -> dict[int, int]:
    """Emulate a wideband snapshot: the true hop pattern, but keep only hops
    that land on a channel the receiver can actually see (observed_channels).

    observed_channels models the ~50 MHz slice: e.g. the 50 contiguous
    channels covered by 61.44 MSPS. Returns {rel_index: channel}.
    """
    seq = build_sequence(seed, domain)
    n = len(seq)
    obs: dict[int, int] = {}
    for r in range(n_hops):
        ch = seq[(start_offset + r) % n]
        if ch in observed_channels:
            obs[r] = ch
    return obs
