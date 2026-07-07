"""UDS server: selectors single-thread loop, multiple connections,
partial-read accumulation, length-prefix framing."""
import collections
import logging
import os
import selectors
import socket
import struct
import time

import numpy as np

from .config import Config
from .infer import ModelRegistry
from .proto import MAX_FRAME, FrameError, build_reply, parse_request, ST_ERROR

log = logging.getLogger("ais_ai.server")


class Stats:
    def __init__(self):
        self.lat_ms = collections.deque(maxlen=2048)
        self.counts = collections.deque(maxlen=100000)   # (t, status)

    def note(self, ms: float, status: int):
        self.lat_ms.append(ms)
        self.counts.append((time.time(), status))

    def snapshot(self):
        lats = sorted(self.lat_ms)
        cut = time.time() - 3600
        recent = [s for t, s in self.counts if t >= cut]
        n = len(recent)
        return {
            "infer_p50_ms": round(lats[len(lats) // 2], 2) if lats else None,
            "infer_p99_ms": round(lats[int(len(lats) * 0.99)], 2) if lats else None,
            "bursts_1h": n,
            "unknown_rate_1h": round(sum(1 for s in recent if s == 1) / n, 3) if n else None,
        }


class Server:
    def __init__(self, cfg: Config, registry: ModelRegistry, stats: Stats):
        self.cfg, self.reg, self.stats = cfg, registry, stats
        self.sel = selectors.DefaultSelector()
        self._last_model_check = 0.0

    def _listen(self):
        path = self.cfg.sock_path
        os.makedirs(os.path.dirname(path), exist_ok=True)
        try:
            os.unlink(path)   # stale socket from a previous run
        except FileNotFoundError:
            pass
        srv = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        srv.bind(path)
        os.chmod(path, 0o600)
        srv.listen(4)
        srv.setblocking(False)
        self.sel.register(srv, selectors.EVENT_READ, ("accept", None))
        log.info("listening on %s", path)
        return srv

    def serve_forever(self, stop_event):
        self._listen()
        bufs = {}
        while not stop_event.is_set():
            for key, _ in self.sel.select(timeout=0.5):
                kind, conn = key.data
                if kind == "accept":
                    c, _ = key.fileobj.accept()
                    c.setblocking(False)
                    bufs[c] = bytearray()
                    self.sel.register(c, selectors.EVENT_READ, ("conn", c))
                    log.info("client connected")
                    continue
                try:
                    data = conn.recv(65536)
                except (BlockingIOError, InterruptedError):
                    continue
                except OSError:
                    data = b""
                if not data:
                    self._drop(conn, bufs, "disconnect")
                    continue
                buf = bufs[conn]
                buf += data
                if not self._drain(conn, buf, bufs):
                    continue
            now = time.time()
            if now - self._last_model_check >= self.cfg.model_check_interval_s:
                self._last_model_check = now
                self.reg.maybe_reload()

    def _drain(self, conn, buf, bufs) -> bool:
        """Process complete frames in buf. False if conn was dropped."""
        while len(buf) >= 4:
            (ln,) = struct.unpack_from("<I", buf, 0)
            if ln < 28 or ln > MAX_FRAME:
                self._drop(conn, bufs, f"bad frame len {ln}")
                return False
            if len(buf) < 4 + ln:
                return True   # wait for more
            frame = bytes(buf[4:4 + ln])
            del buf[:4 + ln]
            t0 = time.perf_counter()
            try:
                mmsi, t_ms, out_sr, n, ch, flags, iq_b = parse_request(frame)
                iq = np.frombuffer(iq_b, dtype=np.float32).copy().view(np.complex64)
                st, pred, conf, mver = self.reg.predict(iq, out_sr)
            except FrameError as e:
                self._drop(conn, bufs, f"frame error: {e}")
                return False
            except Exception:
                log.exception("request handling failed")
                st, pred, conf, mver = ST_ERROR, 0, 0, 0
            ms = (time.perf_counter() - t0) * 1000.0
            self.stats.note(ms, st)
            try:
                conn.sendall(build_reply(st, conf, mver, pred))
            except OSError:
                self._drop(conn, bufs, "send failed")
                return False
        return True

    def _drop(self, conn, bufs, why: str):
        log.info("client dropped: %s", why)
        try:
            self.sel.unregister(conn)
        except Exception:
            pass
        try:
            conn.close()
        except Exception:
            pass
        bufs.pop(conn, None)
