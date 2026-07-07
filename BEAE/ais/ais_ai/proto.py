"""Wire protocol — mirror of src/modules/ais/ais_ai.cpp. All little-endian.

Request : u32 len(=28+8n) | u32 'AIRQ' | u16 ver | u16 type | u32 mmsi |
          i64 t_ms | u32 out_sr | u16 n | u8 ch | u8 flags | f32 IQ[2n]
Reply   : u32 len(=18) | u32 'AIRP' | u16 ver | u16 type | u8 status |
          u8 pad | u16 conf_millipct | u16 model_ver | u32 pred_mmsi  (22 bytes total)
          conf_millipct = confidence * 1000 (0..100000, i.e. 3 decimal places on the %)
"""
import struct

MAGIC_REQ = int.from_bytes(b"AIRQ", "little")   # 0x51524941
MAGIC_RSP = int.from_bytes(b"AIRP", "little")   # 0x50524941
VER = 1
TYPE_INFER = 1

# after the len prefix: magic..flags = 28 bytes
REQ_HDR = struct.Struct("<IHHIqIHBB")
assert REQ_HDR.size == 28
RSP = struct.Struct("<IIHHBBHHI")
assert RSP.size == 22

MAX_FRAME = 65536

# reply status codes
ST_NO_MODEL = 0
ST_UNKNOWN = 1
ST_MATCH = 2
ST_ERROR = 3


class FrameError(Exception):
    pass


def parse_request(hdr_payload: bytes):
    """hdr_payload = the `len` bytes that followed the length prefix.
    Returns (mmsi, t_ms, out_sr, n, ch, flags, iq_bytes)."""
    if len(hdr_payload) < REQ_HDR.size:
        raise FrameError("short frame")
    magic, ver, typ, mmsi, t_ms, out_sr, n, ch, flags = REQ_HDR.unpack_from(hdr_payload, 0)
    if magic != MAGIC_REQ or ver != VER or typ != TYPE_INFER:
        raise FrameError(f"bad header magic={magic:#x} ver={ver} type={typ}")
    if len(hdr_payload) != REQ_HDR.size + 8 * n:
        raise FrameError(f"len mismatch n={n} got={len(hdr_payload)}")
    return mmsi, t_ms, out_sr, n, ch, flags, hdr_payload[REQ_HDR.size:]


def build_reply(status: int, conf_millipct: int, model_ver: int, pred_mmsi: int) -> bytes:
    return RSP.pack(18, MAGIC_RSP, VER, TYPE_INFER, status & 0xFF, 0,
                    max(0, min(100000, conf_millipct)), model_ver & 0xFFFF, pred_mmsi)
