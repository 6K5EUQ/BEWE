import struct
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import numpy as np
import pytest
from ais_ai.proto import (MAGIC_REQ, REQ_HDR, RSP, VER, TYPE_INFER,
                          FrameError, build_reply, parse_request)


def make_req(n=896, mmsi=440000001):
    iq = np.zeros(2 * n, dtype=np.float32)
    hdr = REQ_HDR.pack(MAGIC_REQ, VER, TYPE_INFER, mmsi, 1700000000000, 48000, n, 3, 0)
    return hdr + iq.tobytes()


def test_roundtrip():
    frame = make_req()
    mmsi, t_ms, out_sr, n, ch, flags, iq = parse_request(frame)
    assert (mmsi, out_sr, n, ch) == (440000001, 48000, 896, 3)
    assert len(iq) == 8 * 896
    assert len(frame) == 28 + 8 * 896   # C++ len field value


def test_len_mismatch():
    with pytest.raises(FrameError):
        parse_request(make_req()[:-4])


def test_bad_magic():
    frame = bytearray(make_req())
    frame[0] ^= 0xFF
    with pytest.raises(FrameError):
        parse_request(bytes(frame))


def test_reply_layout():
    r = build_reply(2, 88, 7, 440000001)
    assert len(r) == 20
    ln, magic, ver, typ, st, conf, mver, pred = RSP.unpack(r)
    assert ln == 16 and st == 2 and conf == 88 and mver == 7 and pred == 440000001


def test_reply_conf_clamped():
    r = build_reply(2, 250, 1, 1)
    assert RSP.unpack(r)[5] == 100
