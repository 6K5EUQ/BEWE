"""UID: bind-phrase -> UID, UID <-> FHSS seed, and seed/UID recovery.

Source:
  python/build_flags.py:39-40, python/binary_configurator.py:43   bind -> UID
  lib/OTA/OTA.cpp:41-45   OtaGetUidSeed()
  lib/OTA/OTA.cpp:28-39   OtaUpdateCrcInitFromUid()

Key facts
  UID is 6 bytes = first 6 bytes of MD5('-DMY_BINDING_PHRASE="' + phrase + '"').
  The FHSS seed uses only UID[2..5]:
      seed = (UID2<<24)|(UID3<<16)|(UID4<<8)|(UID5 ^ OTA_VERSION_ID)
  UID[0], UID[1] never appear on air -- they cannot be recovered from RF at all.
  The CRC initializer uses only UID[4], UID[5].
"""

import hashlib

from . import OTA_VERSION_ID


def bind_phrase_to_uid(phrase: str) -> bytes:
    """bind phrase -> 6-byte UID (binary_configurator.py:33-44).

    Two paths, matching the firmware's generateUID():
      - a comma-separated list of 4..6 numbers in [0,256) is taken as literal
        UID bytes, zero-padded to 6 (binary_configurator.py:34-41);
      - otherwise MD5 over the *entire define string* (line 43).
    """
    parts = [p.strip() for p in phrase.split(",")]
    if 4 <= len(parts) <= 6 and all(p.lstrip("-").isdigit() for p in parts):
        nums = [int(p) for p in parts]
        if all(0 <= n < 256 for n in nums):
            return bytes(nums + [0] * (6 - len(nums)))
    material = b'-DMY_BINDING_PHRASE="' + phrase.encode() + b'"'
    return hashlib.md5(material).digest()[:6]


def uid_to_seed(uid: bytes) -> int:
    """UID -> 32-bit FHSS seed (OtaGetUidSeed, OTA.cpp:41-45)."""
    return (
        (uid[2] << 24)
        + (uid[3] << 16)
        + (uid[4] << 8)
        + (uid[5] ^ OTA_VERSION_ID)
    ) & 0xFFFFFFFF


def seed_to_uid2345(seed: int) -> tuple[int, int, int, int]:
    """Invert OtaGetUidSeed back to (UID2, UID3, UID4, UID5).

    Exact and unique: the seed is a bijection of UID[2..5]. UID[0], UID[1]
    are not part of the seed and stay unknown.
    """
    u2 = (seed >> 24) & 0xFF
    u3 = (seed >> 16) & 0xFF
    u4 = (seed >> 8) & 0xFF
    u5 = (seed & 0xFF) ^ OTA_VERSION_ID
    return u2, u3, u4, u5


def crc_initializer(uid4: int, uid5: int) -> int:
    """OtaCrcInitializer (OTA.cpp:34-38) from UID[4], UID[5].

    OtaCrcInitializer = ((UID4<<8)|UID5) ^ (OTA_VERSION_ID<<8)
    The per-packet CRC seed is this value XOR nonce (0 for SYNC).
    """
    return (((uid4 << 8) | uid5) ^ (OTA_VERSION_ID << 8)) & 0xFFFF


def seed_from_uid_low_high(uid2: int, uid3: int, uid4: int, uid5: int) -> int:
    """Build the seed from the four on-air UID bytes (convenience)."""
    return uid_to_seed(bytes([0, 0, uid2, uid3, uid4, uid5]))


def candidate_seeds_from_uid_low(uid4: int, uid5: int):
    """Yield all 2^16 seeds consistent with a known UID[4], UID[5].

    This is the realistic Phase A/B search once a SYNC packet has been
    demodulated (Phase C): the low 16 bits of the seed are then pinned and
    only UID[2], UID[3] (the high 16 bits) remain unknown.
    """
    low = ((uid4 << 8) + (uid5 ^ OTA_VERSION_ID)) & 0xFFFF
    for high in range(0x10000):
        yield (high << 16) | low
