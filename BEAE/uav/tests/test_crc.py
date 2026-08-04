"""CRC14/CRC16 round-trip and rejection."""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from uav_elrs.crc import (generate_full, generate_std, validate_full,
                          validate_std)
from uav_elrs.packet import PACKET_TYPE_DATA, PACKET_TYPE_SYNC
from uav_elrs.uid import crc_initializer


def _make_std(body7: bytes, crc_init: int, nonce: int) -> bytes:
    ch, cl = generate_std(body7, crc_init, nonce)
    pkt = bytearray(8)
    pkt[:7] = body7
    pkt[0] = (body7[0] & 0b11) | (ch << 2)
    pkt[7] = cl
    return bytes(pkt)


def test_std_round_trip():
    crc_init = crc_initializer(0x55, 0x66)
    # non-SYNC uses the nonce; SYNC forces nonce 0
    body = bytes([PACKET_TYPE_DATA, 0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC])
    for nonce in (0, 1, 42, 255):
        pkt = _make_std(body, crc_init, nonce)
        assert validate_std(pkt, crc_init, nonce)
        # wrong nonce must fail (nonce is folded into the seed)
        assert not validate_std(pkt, crc_init, (nonce + 1) & 0xFF)


def test_std_sync_ignores_nonce():
    crc_init = crc_initializer(0x11, 0x22)
    body = bytes([PACKET_TYPE_SYNC, 0x05, 0x99, 0x09, 0x00, 0x11, 0x22])
    pkt = _make_std(body, crc_init, nonce=0)
    # SYNC validates regardless of the nonce passed
    assert validate_std(pkt, crc_init, nonce=0)
    assert validate_std(pkt, crc_init, nonce=200)


def test_std_corruption_rejected():
    crc_init = crc_initializer(0x55, 0x66)
    body = bytes([PACKET_TYPE_DATA, 1, 2, 3, 4, 5, 6])
    pkt = bytearray(_make_std(body, crc_init, 7))
    pkt[3] ^= 0x01
    assert not validate_std(bytes(pkt), crc_init, 7)


def test_full_round_trip():
    crc_init = crc_initializer(0xAB, 0xCD)
    body = bytes([PACKET_TYPE_DATA, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10])
    for nonce in (0, 3, 200):
        crc = generate_full(body, crc_init, nonce)
        pkt = body + bytes([crc & 0xFF, (crc >> 8) & 0xFF])
        assert validate_full(pkt, crc_init, nonce)
        bad = bytearray(pkt); bad[5] ^= 0x80
        assert not validate_full(bytes(bad), crc_init, nonce)


if __name__ == "__main__":
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
            print(f"ok  {name}")
    print("CRC: all passed")
