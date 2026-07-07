"""Defensive reader for aicap_YYYYMMDD.bin written by ais_ai.cpp.

Record: u32 'AIC1' | u32 mmsi | i64 t_ms | u32 out_sr | u16 n | u8 ch | u8 flags
        | f32 IQ[2n]                                    (24-byte header)
Never raises on corrupt/torn data: bad records are skipped by scanning forward
for the next magic; a truncated tail record just ends iteration (C++ may be
mid-append).
"""
import glob
import os
import struct
from dataclasses import dataclass

import numpy as np

MAGIC = int.from_bytes(b"AIC1", "little")   # 0x31434941
HDR = struct.Struct("<IIqIHBB")
assert HDR.size == 24
_MAGIC_BYTES = b"AIC1"

N_MIN, N_MAX = 64, 4096
SR_MIN, SR_MAX = 40000, 60000


@dataclass
class CapRecord:
    mmsi: int
    t_ms: int
    out_sr: int
    ch: int
    flags: int
    iq: np.ndarray  # complex64 [n]


def iter_file(path: str, start_offset: int = 0):
    """Yield CapRecord from one .bin file. Returns (via StopIteration value
    pattern not used) — call stats(path) for skip accounting if needed."""
    with open(path, "rb") as f:
        data = f.read()
    yield from iter_bytes(data, start_offset)


def iter_bytes(data: bytes, start_offset: int = 0):
    off = start_offset
    n_total = len(data)
    while off + HDR.size <= n_total:
        magic, mmsi, t_ms, out_sr, n, ch, flags = HDR.unpack_from(data, off)
        ok = (magic == MAGIC and N_MIN <= n <= N_MAX and SR_MIN <= out_sr <= SR_MAX
              and mmsi > 0)
        if not ok:
            nxt = data.find(_MAGIC_BYTES, off + 1)   # resync
            if nxt < 0:
                return
            off = nxt
            continue
        end = off + HDR.size + 8 * n
        if end > n_total:
            return   # torn tail record — stop, C++ may be mid-append
        iq = np.frombuffer(data, dtype=np.float32, count=2 * n,
                           offset=off + HDR.size).copy()
        if not np.all(np.isfinite(iq)):
            off = end
            continue
        yield CapRecord(mmsi, t_ms, out_sr, ch, flags, iq.view(np.complex64))
        off = end


def day_files(data_dir: str, days: int):
    """Most-recent `days` aicap files, oldest first."""
    files = sorted(glob.glob(os.path.join(data_dir, "aicap_*.bin")))
    return files[-days:] if days > 0 else files


def inspect(path: str):
    """Parser statistics for one file: dict with counts."""
    with open(path, "rb") as f:
        data = f.read()
    per_mmsi, srs, n_rec, skipped = {}, {}, 0, 0
    off = 0
    while off + HDR.size <= len(data):
        magic, mmsi, t_ms, out_sr, n, ch, flags = HDR.unpack_from(data, off)
        if (magic == MAGIC and N_MIN <= n <= N_MAX and SR_MIN <= out_sr <= SR_MAX
                and mmsi > 0 and off + HDR.size + 8 * n <= len(data)):
            n_rec += 1
            per_mmsi[mmsi] = per_mmsi.get(mmsi, 0) + 1
            srs[out_sr] = srs.get(out_sr, 0) + 1
            off += HDR.size + 8 * n
        else:
            nxt = data.find(_MAGIC_BYTES, off + 1)
            if nxt < 0:
                skipped += len(data) - off
                break
            skipped += nxt - off
            off = nxt
    return {"file": path, "bytes": len(data), "records": n_rec,
            "skipped_bytes": skipped, "distinct_mmsi": len(per_mmsi),
            "sr_histogram": srs, "per_mmsi": per_mmsi}
