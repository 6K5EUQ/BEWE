"""UID derivation, seed round-trip, and candidate coverage."""

import hashlib
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from uav_elrs import OTA_VERSION_ID
from uav_elrs.uid import (bind_phrase_to_uid, candidate_seeds_from_uid_low,
                          crc_initializer, seed_from_uid_low_high,
                          seed_to_uid2345, uid_to_seed)


def test_bind_phrase_matches_configurator():
    # binary_configurator.py:43  md5('-DMY_BINDING_PHRASE="'+phrase+'"')[0:6]
    phrase = "test-drone"
    material = ('-DMY_BINDING_PHRASE="' + phrase + '"').encode()
    expected = hashlib.md5(material).digest()[:6]
    assert bind_phrase_to_uid(phrase) == expected
    assert len(bind_phrase_to_uid(phrase)) == 6


def test_seed_formula():
    uid = bytes([0x11, 0x22, 0x33, 0x44, 0x55, 0x66])
    seed = uid_to_seed(uid)
    # (0x33<<24)|(0x44<<16)|(0x55<<8)|(0x66^4)
    expected = (0x33 << 24) | (0x44 << 16) | (0x55 << 8) | (0x66 ^ OTA_VERSION_ID)
    assert seed == expected


def test_seed_round_trip():
    for uid in [bytes([0, 0, a, b, c, d]) for a, b, c, d in
                [(1, 2, 3, 4), (255, 0, 128, 7), (0xAB, 0xCD, 0xEF, 0x01)]]:
        seed = uid_to_seed(uid)
        assert seed_to_uid2345(seed) == (uid[2], uid[3], uid[4], uid[5])


def test_candidate_seeds_contains_truth():
    uid = bind_phrase_to_uid("my-quad")
    true_seed = uid_to_seed(uid)
    # the 2^16 set for known UID4/UID5 must contain the true seed
    found = False
    for seed in candidate_seeds_from_uid_low(uid[4], uid[5]):
        if seed == true_seed:
            found = True
            break
    assert found
    # and the reconstructed high bytes match
    assert seed_from_uid_low_high(uid[2], uid[3], uid[4], uid[5]) == true_seed


def test_crc_initializer():
    # OtaCrcInitializer = ((UID4<<8)|UID5) ^ (OTA_VERSION_ID<<8)
    assert crc_initializer(0x55, 0x66) == (((0x55 << 8) | 0x66) ^ (OTA_VERSION_ID << 8))


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"ok  {name}")
    print("UID: all passed")
