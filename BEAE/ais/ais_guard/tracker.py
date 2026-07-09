"""TrackStore — 오늘자 AIS JSONL tail(자정 롤오버/파일 교체 안전) → MMSI별 항적.

트랙 포인트는 crc==1 & hp==1 만, 정적정보/Match_AI/RF 버퍼는 crc==1 전체에서 갱신.
"""
import json
import os
import time
from collections import deque
from dataclasses import dataclass, field
from datetime import datetime, timedelta, timezone

KST = timezone(timedelta(hours=9))


def kst_day(t: float | None = None) -> str:
    return datetime.fromtimestamp(time.time() if t is None else t, KST).strftime("%Y%m%d")


@dataclass
class Track:
    mmsi: int
    pts: deque = None       # (t_ms, lat, lon, sog, cog) — 유효 위치만
    match: deque = None     # (aist, aim) — Match_AI 결과 있는 버스트만
    rf: deque = None        # (t_ms, cfo_hz, rssi_db)
    nm: str = ""            # 최신 선명
    st: int = 0             # 최신 선종
    last_t_ms: int = 0      # 마지막 수신(위치 무관)

    def latest(self):
        return self.pts[-1] if self.pts else None


class TrackStore:
    def __init__(self, cfg):
        self.cfg = cfg
        self.tracks: dict[int, Track] = {}
        self._path = ""
        self._f = None
        self._buf = b""
        self.n_lines = 0        # 반영된 레코드 수
        self.n_bad = 0          # 파싱 실패/무효 줄 수

    def _today_path(self) -> str:
        return os.path.join(self.cfg.ais_dir, f"ais_{kst_day()}.jsonl")

    def poll(self) -> int:
        """새 줄 소비, 반영 건수 반환. 롤오버 시 이전 파일 잔여분 먼저 드레인."""
        path = self._today_path()
        if self._f is not None and self._path != path:
            n = self._drain()                      # 자정 직전 잔여 줄
            self._f.close()
            self._f = None
            self._buf = b""
            return n + self.poll()
        if self._f is None:
            try:
                self._f = open(path, "rb")
            except OSError:
                return 0                           # 오늘 파일 아직 없음
            self._path = path
        try:                                       # truncate/inode 교체 감지
            st = os.stat(self._path)
            fst = os.fstat(self._f.fileno())
            if st.st_ino != fst.st_ino:
                self._f.close()
                self._f = open(self._path, "rb")
                self._buf = b""
            elif fst.st_size < self._f.tell():
                self._f.seek(0)
                self._buf = b""
        except OSError:
            pass
        return self._drain()

    def _drain(self) -> int:
        n = 0
        while True:
            chunk = self._f.read(1 << 16)
            if not chunk:
                break
            self._buf += chunk
            while True:
                i = self._buf.find(b"\n")
                if i < 0:
                    break                          # 마지막 줄 미완성 — C++ append 중
                line = self._buf[:i]
                self._buf = self._buf[i + 1:]
                if self._ingest(line):
                    n += 1
        return n

    def _ingest(self, raw: bytes) -> bool:
        try:
            d = json.loads(raw)
        except Exception:
            self.n_bad += 1
            return False
        mmsi = int(d.get("mmsi", 0) or 0)
        if mmsi <= 0 or d.get("crc", 0) != 1:
            self.n_bad += 1
            return False
        tr = self.tracks.get(mmsi)
        if tr is None:
            w = self.cfg.track_window
            tr = self.tracks[mmsi] = Track(mmsi, deque(maxlen=w), deque(maxlen=w), deque(maxlen=w))
        t_ms = int(d.get("t", 0))
        tr.last_t_ms = max(tr.last_t_ms, t_ms)
        if d.get("hp") == 1:
            tr.pts.append((t_ms, float(d.get("lat", 0.0)), float(d.get("lon", 0.0)),
                           float(d.get("sog", -1.0)), float(d.get("cog", -1.0))))
        if d.get("nm"):
            tr.nm = str(d["nm"])
        if d.get("st"):
            tr.st = int(d["st"])
        if "aist" in d:
            tr.match.append((int(d["aist"]), int(d.get("aim", 0))))
        if "cfo" in d:
            tr.rf.append((t_ms, float(d["cfo"]), float(d.get("rssi", 0.0))))
        self.n_lines += 1
        return True

    def get_active(self, max_age_s: float, now_ms: int | None = None) -> dict[int, Track]:
        """max_age_s 이내 위치보고가 있는 트랙만."""
        now_ms = now_ms or int(time.time() * 1000)
        cut = now_ms - int(max_age_s * 1000)
        return {m: tr for m, tr in self.tracks.items() if tr.pts and tr.pts[-1][0] >= cut}

    def prune(self, max_age_s: float, now_ms: int | None = None):
        """오래 침묵한 트랙 메모리 정리."""
        now_ms = now_ms or int(time.time() * 1000)
        cut = now_ms - int(max_age_s * 1000)
        for m in [m for m, tr in self.tracks.items() if tr.last_t_ms < cut]:
            del self.tracks[m]
