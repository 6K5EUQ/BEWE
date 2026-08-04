"""Wire protocol — mirror of src/modules/amc/amc_ai.cpp. All little-endian.

Request : u32 len(=28+8n) | u32 'AMRQ' | u16 ver | u16 type | u32 bw_hz |
          i64 t_ms | u32 out_sr | u16 n | u8 ch | u8 trig | f32 IQ[2n]
Reply   : u32 len(=18) | u32 'AMRP' | u16 ver | u16 type | u8 status | u8 cls |
          u8 cls2 | u8 pad | u16 conf_permille | u16 conf2_permille | u16 model_ver
          (22 bytes total)

conf_permille = probability * 1000, so 0..1000. It MUST be clamped: the AIS
daemon once packed a value past 65535 into its u16 and struct.pack raised inside
the select loop, taking the whole daemon down silently (2026-07-07). A
probability can only exceed 1.0 through a bug, but clamp anyway — a wrong number
is survivable, a dead daemon is not.

cls / cls2 are indices into amc_ai.synth.CLASSES, which must stay in the same
order as AMC_CLASSES in src/modules/amc/amc_meta.hpp. 0xFF = no decision.
"""
import struct

MAGIC_REQ = int.from_bytes(b"AMRQ", "little")
MAGIC_RSP = int.from_bytes(b"AMRP", "little")
VER = 1
TYPE_INFER = 1

# after the len prefix: magic..trig = 28 bytes
REQ_HDR = struct.Struct("<IHHIqIHBB")
assert REQ_HDR.size == 28
RSP = struct.Struct("<IIHHBBBBHHH")
assert RSP.size == 22

MAX_FRAME = 65536

# reply status codes
ST_NO_MODEL = 0
ST_OK = 1
ST_ERROR = 3

NO_CLASS = 0xFF


class FrameError(Exception):
    pass


def parse_request(hdr_payload: bytes):
    """hdr_payload = the `len` bytes that followed the length prefix.
    Returns (bw_hz, t_ms, out_sr, n, ch, trig, iq_bytes)."""
    if len(hdr_payload) < REQ_HDR.size:
        raise FrameError("short frame")
    magic, ver, typ, bw_hz, t_ms, out_sr, n, ch, trig = REQ_HDR.unpack_from(hdr_payload, 0)
    if magic != MAGIC_REQ or ver != VER or typ != TYPE_INFER:
        raise FrameError(f"bad header magic={magic:#x} ver={ver} type={typ}")
    if len(hdr_payload) != REQ_HDR.size + 8 * n:
        raise FrameError(f"len mismatch n={n} got={len(hdr_payload)}")
    return bw_hz, t_ms, out_sr, n, ch, trig, hdr_payload[REQ_HDR.size:]


def _permille(p: float) -> int:
    return max(0, min(1000, int(round(p * 1000.0))))


def build_reply(status: int, cls: int, conf: float, cls2: int, conf2: float, model_ver: int) -> bytes:
    return RSP.pack(18, MAGIC_RSP, VER, TYPE_INFER,
                    status & 0xFF,
                    (cls if 0 <= cls < 255 else NO_CLASS),
                    (cls2 if 0 <= cls2 < 255 else NO_CLASS),
                    0,
                    _permille(conf), _permille(conf2),
                    model_ver & 0xFFFF)
