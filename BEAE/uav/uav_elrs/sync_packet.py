"""Phase C top level: a demodulated SYNC packet -> validated UID[4], UID[5].

Ties packet + crc + uid together. A SYNC packet carries UID[4] (clear) and
UID[5] (possibly model-match XORed on its low 6 bits). We confirm the
extraction by requiring the ELRS CRC to validate -- that is the only proof
the demodulation and the UID guess are both right. Once UID[4], UID[5] are
confirmed, solve.solve_from_sync narrows UID[2], UID[3] with a 2^16 search
over the observed hops.

Source: model-match XOR at src/tx_main.cpp:411-415 (MODELMATCH_MASK 0x3f).
"""

from dataclasses import dataclass

from . import OTA_VERSION_ID
from .crc import validate_full, validate_std
from .packet import (MODELMATCH_MASK, PACKET_TYPE_SYNC, packet_type,
                     parse_sync)
from .uid import crc_initializer


@dataclass
class SyncExtract:
    uid4: int
    uid5: int            # real UID[5] (model-match undone)
    fhss_index: int      # transmitter's FHSSptr -> pins hop phase directly
    nonce: int
    rf_rate_enum: int
    model_delta: int     # (~modelId)&0x3f that was applied (0 = model match off)
    crc_ok: bool = True


def extract_uid_from_sync(pkt: bytes) -> SyncExtract | None:
    """Validate a candidate SYNC packet and return the real UID[4], UID[5].

    Returns None if it is not a SYNC packet or no model-id makes the CRC
    validate (i.e. the demodulation is wrong or it is a different link).
    """
    if packet_type(pkt) != PACKET_TYPE_SYNC:
        return None
    if len(pkt) not in (8, 13):
        return None
    info = parse_sync(pkt)
    validate = validate_std if len(pkt) == 8 else validate_full

    uid4 = info.uid4
    tx_uid5 = info.uid5_raw
    high2 = tx_uid5 & 0xC0
    low6 = tx_uid5 & 0x3F

    # model_delta = (~modelId)&0x3f in 0..63; real low6 = tx_low6 ^ delta.
    # delta 0 first (model match off, the common case).
    for delta in range(64):
        real_uid5 = high2 | (low6 ^ delta)
        crc_init = crc_initializer(uid4, real_uid5)
        # SYNC packets use nonceValidator = 0
        if validate(pkt, crc_init, nonce=0):
            return SyncExtract(
                uid4=uid4,
                uid5=real_uid5,
                fhss_index=info.fhss_index,
                nonce=info.nonce,
                rf_rate_enum=info.rf_rate_enum,
                model_delta=delta,
            )
    return None


def build_sync_packet(uid4: int, uid5: int, fhss_index: int, nonce: int,
                      rf_rate_enum: int, model_id: int = 0xFF,
                      full_res: bool = False) -> bytes:
    """Build a valid SYNC packet (for tests / round-trip). Mirrors
    tx_main.cpp:381-416 field population + the model-match XOR + CRC.
    """
    from .crc import generate_full, generate_std

    delta = (~model_id) & MODELMATCH_MASK
    tx_uid5 = (uid5 & 0xC0) | ((uid5 & 0x3F) ^ delta)
    bits = 0  # switchEncMode/newTlmRatio/geminiMode/otaProtocol/free all 0
    crc_init = crc_initializer(uid4, uid5)

    if not full_res:
        body = bytearray(8)
        body[0] = PACKET_TYPE_SYNC            # type in low 2 bits, crcHigh=0
        body[1] = fhss_index & 0xFF
        body[2] = nonce & 0xFF
        body[3] = rf_rate_enum & 0xFF
        body[4] = bits
        body[5] = uid4 & 0xFF
        body[6] = tx_uid5 & 0xFF
        crc_high, crc_low = generate_std(bytes(body[:7]), crc_init, nonce=0)
        body[0] |= (crc_high << 2)
        body[7] = crc_low
        return bytes(body)
    else:
        body = bytearray(13)
        body[0] = PACKET_TYPE_SYNC
        body[1] = fhss_index & 0xFF
        body[2] = nonce & 0xFF
        body[3] = rf_rate_enum & 0xFF
        body[4] = bits
        body[5] = uid4 & 0xFF
        body[6] = tx_uid5 & 0xFF
        # bytes 7..10 free (0)
        crc = generate_full(bytes(body[:11]), crc_init, nonce=0)
        body[11] = crc & 0xFF
        body[12] = (crc >> 8) & 0xFF
        return bytes(body)
