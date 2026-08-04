"""OTA packet layout: type, SYNC fields, RC channels.

Source:
  lib/OTA/OTA.h:56-176   OTA_Packet4_s / OTA_Packet8_s / OTA_Sync_s
  lib/OTA/OTA.cpp:94-116 PackUInt11ToChannels4x10 (inverse implemented here)
  src/tx_main.cpp:381-416 sync packet population, :411-415 model-match XOR

A decoded SYNC packet is the jackpot: it carries UID[4], UID[5] in the clear
(so the CRC init and the low 16 bits of the FHSS seed), plus fhssIndex (the
transmitter's current FHSSptr) and nonce -- which pin the hop-sequence phase
directly, without any search.
"""

from dataclasses import dataclass

PACKET_TYPE_RCDATA = 0b00     # uplink
PACKET_TYPE_DATA = 0b01
PACKET_TYPE_SYNC = 0b10       # uplink
PACKET_TYPE_LINKSTATS = 0b00  # downlink
MODELMATCH_MASK = 0x3F        # OTA.h:27

_TYPE_NAME = {0b00: "RCDATA/LINKSTATS", 0b01: "DATA", 0b10: "SYNC", 0b11: "?"}


def packet_type(pkt: bytes) -> int:
    """Low two bits of byte 0 -- always the packet type for both sizes."""
    return pkt[0] & 0b11


def type_name(pkt: bytes) -> str:
    return _TYPE_NAME[packet_type(pkt)]


@dataclass
class SyncInfo:
    fhss_index: int    # transmitter's current FHSSptr
    nonce: int         # transmitter's OtaNonce
    rf_rate_enum: int  # expresslrs_RFrates_e value
    switch_enc_mode: int
    new_tlm_ratio: int
    gemini_mode: int
    ota_protocol: int
    uid4: int          # UID[4], in the clear
    uid5_raw: int      # UID[5] as transmitted (may be model-match XORed)

    @property
    def uid5_if_no_modelmatch(self) -> int:
        """UID[5] assuming model match is off (the common case).

        With model match on, the low 6 bits are XORed with (~modelId)&0x3f;
        bits 6-7 are never touched. If a candidate UID fails CRC, retry the
        64 model-id possibilities on the low 6 bits.
        """
        return self.uid5_raw


def parse_sync(pkt: bytes) -> SyncInfo:
    """Parse the SYNC payload. Works for both 8- and 13-byte packets:
    the OTA_Sync_s struct sits at bytes 1..6 in both layouts.
    """
    if packet_type(pkt) != PACKET_TYPE_SYNC:
        raise ValueError("not a SYNC packet")
    if len(pkt) not in (8, 13):
        raise ValueError("packet must be 8 or 13 bytes")
    fhss_index = pkt[1]
    nonce = pkt[2]
    rf_rate = pkt[3]
    bits = pkt[4]
    uid4 = pkt[5]
    uid5 = pkt[6]
    return SyncInfo(
        fhss_index=fhss_index,
        nonce=nonce,
        rf_rate_enum=rf_rate,
        switch_enc_mode=bits & 0x01,
        new_tlm_ratio=(bits >> 1) & 0x07,
        gemini_mode=(bits >> 4) & 0x01,
        ota_protocol=(bits >> 5) & 0x03,
        uid4=uid4,
        uid5_raw=uid5,
    )


def unpack_channels_4x10(five_bytes: bytes) -> list[int]:
    """Inverse of PackUInt11ToChannels4x10 (OTA.cpp:94-116).

    5 bytes -> 4x 10-bit channel values. Packing is little-endianish:
      ch0: byte0[0:8] + byte1[0:2]
      ch1: byte1[2:8] + byte2[0:4]
      ch2: byte2[4:8] + byte3[0:6]
      ch3: byte3[6:8] + byte4[0:8]
    """
    if len(five_bytes) != 5:
        raise ValueError("need exactly 5 bytes")
    b = five_bytes
    v0 = b[0] | ((b[1] & 0x03) << 8)
    v1 = (b[1] >> 2) | ((b[2] & 0x0F) << 6)
    v2 = (b[2] >> 4) | ((b[3] & 0x3F) << 4)
    v3 = (b[3] >> 6) | (b[4] << 2)
    return [v0 & 0x3FF, v1 & 0x3FF, v2 & 0x3FF, v3 & 0x3FF]
