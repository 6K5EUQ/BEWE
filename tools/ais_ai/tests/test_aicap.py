import struct
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
from ais_ai.aicap import HDR, MAGIC, iter_bytes


def rec(mmsi=440000001, t=1700000000000, sr=48000, n=896):
    iq = np.arange(2 * n, dtype=np.float32)
    return HDR.pack(MAGIC, mmsi, t, sr, n, 3, 0) + iq.tobytes()


def test_roundtrip():
    data = rec() + rec(mmsi=440000002, n=896)
    out = list(iter_bytes(data))
    assert len(out) == 2
    assert out[0].mmsi == 440000001 and out[1].mmsi == 440000002
    assert out[0].iq.dtype == np.complex64 and len(out[0].iq) == 896
    assert out[0].iq[0] == 0 + 1j and out[0].iq[1] == 2 + 3j


def test_torn_tail():
    data = rec() + rec()[:100]   # truncated tail record (C++ mid-append)
    out = list(iter_bytes(data))
    assert len(out) == 1


def test_resync_after_garbage():
    data = rec() + b"\xde\xad\xbe\xef" * 20 + rec(mmsi=440000009)
    out = list(iter_bytes(data))
    assert [r.mmsi for r in out] == [440000001, 440000009]


def test_bad_fields_skipped():
    bad = HDR.pack(MAGIC, 0, 0, 48000, 896, 0, 0) + b"\x00" * (8 * 896)  # mmsi=0
    data = bad + rec()
    out = list(iter_bytes(data))
    assert len(out) == 1 and out[0].mmsi == 440000001


def test_nan_rejected():
    iq = np.full(2 * 896, np.nan, dtype=np.float32)
    bad = HDR.pack(MAGIC, 440000005, 1, 48000, 896, 0, 0) + iq.tobytes()
    out = list(iter_bytes(bad + rec()))
    assert [r.mmsi for r in out] == [440000001]
