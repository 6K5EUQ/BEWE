#!/usr/bin/env python3
"""Mock Match_AI daemon for C++ integration tests (no torch needed).

Usage: python3 mock_daemon.py [match|unknown|mismatch|nomodel|hang]
  match    -> status=2, pred=claimed mmsi, conf=88  (GUI: green "MMSI 88%")
  unknown  -> status=1                              (GUI: "UNKNOWN")
  mismatch -> status=2, pred=claimed+1, conf=77     (GUI: red)
  nomodel  -> status=0                              (GUI: "-")
  hang     -> accept then never reply               (C++ 60ms budget + 5s latch test)
"""
import os
import socket
import struct
import sys
import time

MODE = sys.argv[1] if len(sys.argv) > 1 else "match"
SOCK = os.path.expanduser("~/BEWE/modules/ais/ai.sock")
REQ = struct.Struct("<IHHIqIHBB")   # after len prefix, 28 bytes

try:
    os.unlink(SOCK)
except FileNotFoundError:
    pass
srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
srv.bind(SOCK)
srv.listen(2)
print(f"mock daemon [{MODE}] on {SOCK}")

while True:
    c, _ = srv.accept()
    print("client connected")
    try:
        while True:
            ln_b = c.recv(4, socket.MSG_WAITALL)
            if len(ln_b) < 4:
                break
            (ln,) = struct.unpack("<I", ln_b)
            body = c.recv(ln, socket.MSG_WAITALL)
            if len(body) < ln:
                break
            magic, ver, typ, mmsi, t_ms, sr, n, ch, flags = REQ.unpack_from(body, 0)
            assert magic == int.from_bytes(b"AIRQ", "little"), hex(magic)
            assert ln == 28 + 8 * n, (ln, n)
            if MODE == "hang":
                print(f"req mmsi={mmsi} n={n} -> hanging")
                time.sleep(3600)
            st, pred, conf = {"match": (2, mmsi, 88), "unknown": (1, mmsi, 42),
                              "mismatch": (2, mmsi + 1, 77), "nomodel": (0, 0, 0)}[MODE]
            c.send(struct.pack("<IIHHBBHI", 16, int.from_bytes(b"AIRP", "little"),
                               1, 1, st, conf, 1, pred))
            print(f"req mmsi={mmsi} n={n} sr={sr} ch={ch} -> status={st} pred={pred}")
    except (ConnectionError, AssertionError) as e:
        print("conn end:", e)
    finally:
        c.close()
