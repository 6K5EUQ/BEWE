"""ELRS custom CRC (CRC14 / CRC16), ported 1:1 from lib/CRC/crc.cpp.

ExpressLRS does not use the radio's hardware CRC on LoRa; it appends its own
table-driven CRC over the OTA bytes, seeded by the UID and XORed with the
packet nonce. Getting a valid CRC is the proof that a recovered UID and a
demodulated packet are both correct.

Source:
  lib/CRC/crc.cpp        Crc2Byte::init() / Crc2Byte::calc()
  lib/OTA/OTA.h:189-190  ELRS_CRC14_POLY 0x2E57, ELRS_CRC16_POLY 0x3D65
  lib/OTA/OTA.cpp:516-550 ValidatePacketCrcStd / ...Full  (calc ranges, seeds)
"""

from . import OTA_VERSION_ID

ELRS_CRC14_POLY = 0x2E57  # OTA.h:189  (8-byte "std" packet)
ELRS_CRC16_POLY = 0x3D65  # OTA.h:190  (13-byte "full" packet)

OTA4_PACKET_SIZE = 8
OTA8_PACKET_SIZE = 13
OTA4_CRC_CALC_LEN = 7   # offsetof(OTA_Packet4_s, crcLow)
OTA8_CRC_CALC_LEN = 11  # offsetof(OTA_Packet8_s, crc)


class Crc2Byte:
    """Exact port of lib/CRC/crc.cpp Crc2Byte (MSB-first, table-driven)."""

    __slots__ = ("bits", "poly", "bitmask", "tab")

    def __init__(self, bits: int, poly: int):
        self.bits = bits
        self.poly = poly
        self.bitmask = (1 << bits) - 1
        highbit = 1 << (bits - 1)
        self.tab = [0] * 256
        for i in range(256):
            crc = (i << (bits - 8)) & 0xFFFF
            for _ in range(8):
                crc = ((crc << 1) ^ (poly if (crc & highbit) else 0)) & 0xFFFF
            self.tab[i] = crc

    def calc(self, data: bytes, length: int, crc: int) -> int:
        # crc.cpp:53-60  crc = (crc<<8) ^ tab[((crc>>(bits-8)) ^ byte) & 0xFF]
        shift = self.bits - 8
        for i in range(length):
            idx = ((crc >> shift) ^ data[i]) & 0xFF
            crc = ((crc << 8) ^ self.tab[idx]) & 0xFFFF
        return crc & self.bitmask


# Shared instances (ELRS keeps one static Crc2Byte, re-init on packet size)
_crc14 = Crc2Byte(14, ELRS_CRC14_POLY)
_crc16 = Crc2Byte(16, ELRS_CRC16_POLY)

PACKET_TYPE_SYNC = 0b10  # OTA.h:20


def _nonce_validator(pkt_type: int, nonce: int) -> int:
    # OTA.cpp: nonceValidator = (type == SYNC) ? 0 : OtaNonce
    return 0 if pkt_type == PACKET_TYPE_SYNC else (nonce & 0xFF)


def validate_std(pkt: bytes, crc_init: int, nonce: int) -> bool:
    """Validate an 8-byte packet (CRC14). ValidatePacketCrcStd, OTA.cpp:525-537.

    pkt: 8 bytes. type = pkt[0] & 0b11, crcHigh = pkt[0] >> 2, crcLow = pkt[7].
    """
    if len(pkt) != OTA4_PACKET_SIZE:
        raise ValueError("std packet must be 8 bytes")
    pkt_type = pkt[0] & 0b11
    in_crc = ((pkt[0] >> 2) << 8) + pkt[7]
    body = bytearray(pkt)
    body[0] &= 0b11  # zero crcHigh before calc
    seed = (crc_init ^ _nonce_validator(pkt_type, nonce)) & 0xFFFF
    calc = _crc14.calc(bytes(body), OTA4_CRC_CALC_LEN, seed)
    return in_crc == calc


def validate_full(pkt: bytes, crc_init: int, nonce: int) -> bool:
    """Validate a 13-byte packet (CRC16). ValidatePacketCrcFull, OTA.cpp:517-522.

    pkt: 13 bytes. crc = little-endian uint16 at bytes 11,12. type = pkt[0]&0b11.
    """
    if len(pkt) != OTA8_PACKET_SIZE:
        raise ValueError("full packet must be 13 bytes")
    pkt_type = pkt[0] & 0b11
    in_crc = pkt[11] | (pkt[12] << 8)
    seed = (crc_init ^ _nonce_validator(pkt_type, nonce)) & 0xFFFF
    calc = _crc16.calc(pkt, OTA8_CRC_CALC_LEN, seed)
    return in_crc == calc


def generate_std(body7: bytes, crc_init: int, nonce: int) -> tuple[int, int]:
    """Generate CRC for an 8-byte packet body (first 7 bytes, byte0 crcHigh=0).

    Returns (crcHigh, crcLow) -- crcHigh goes in the high 6 bits of byte0,
    crcLow is byte7. GeneratePacketCrcStd, OTA.cpp:544-550. Used by tests.
    """
    pkt_type = body7[0] & 0b11
    seed = (crc_init ^ _nonce_validator(pkt_type, nonce)) & 0xFFFF
    crc = _crc14.calc(body7, OTA4_CRC_CALC_LEN, seed)
    return (crc >> 8) & 0x3F, crc & 0xFF


def generate_full(body11: bytes, crc_init: int, nonce: int) -> int:
    """Generate the CRC16 for a 13-byte packet body (first 11 bytes).

    Returns the uint16 CRC (little-endian on the wire). Used by tests.
    """
    pkt_type = body11[0] & 0b11
    seed = (crc_init ^ _nonce_validator(pkt_type, nonce)) & 0xFFFF
    return _crc16.calc(body11, OTA8_CRC_CALC_LEN, seed)
