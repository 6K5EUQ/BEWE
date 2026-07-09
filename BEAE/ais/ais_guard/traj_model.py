"""TrajNet — GRU seq2seq 항적 예측기.

입력 16스텝 피처 [dlat*1e4, dlon*1e4, sog/20, sin(cog), cos(cog), dt/10s]
→ 미래 8스텝 (dlat,dlon) (같은 1e4 스케일). track = [(t_ms,lat,lon,sog,cog), ...].
"""
import math

import numpy as np
import torch
import torch.nn as nn

SEQ_IN, SEQ_OUT = 16, 8
HIDDEN, N_FEAT = 96, 6
POS_SCALE = 1e4                                         # 도 → 1e-4도 단위
_M_LAT = 111320.0                                       # 위도 1도 ≈ 111.32 km


def track_features(track):
    """(feat[N-1,6], pts[N,2]) 또는 None. dt<=0 중복점은 제거."""
    pts = []
    last_t = None
    for t_ms, lat, lon, sog, cog in track:
        if last_t is not None and t_ms <= last_t:
            continue
        last_t = t_ms
        pts.append((t_ms / 1000.0, lat, lon, max(float(sog), 0.0), float(cog)))
    if len(pts) < 2:
        return None
    a = np.asarray(pts, np.float64)                     # [N,5] = t_s,lat,lon,sog,cog
    dt = np.diff(a[:, 0])
    cog = np.deg2rad(a[1:, 4])
    ok = (a[1:, 4] >= 0.0) & (a[1:, 4] < 360.0)         # cog n/a → (0,0)
    feat = np.stack([np.diff(a[:, 1]) * POS_SCALE,
                     np.diff(a[:, 2]) * POS_SCALE,
                     a[1:, 3] / 20.0,
                     np.where(ok, np.sin(cog), 0.0),
                     np.where(ok, np.cos(cog), 0.0),
                     dt / 10.0], 1).astype(np.float32)
    return feat, a[:, 1:3]


class TrajNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.enc = nn.GRU(N_FEAT, HIDDEN, batch_first=True)
        self.dec = nn.GRU(2, HIDDEN, batch_first=True)
        self.head = nn.Linear(HIDDEN, 2)

    def forward(self, x):                               # [B,16,6] → [B,8,2]
        _, h = self.enc(x)
        inp = x[:, -1:, :2]                             # 마지막 관측 델타로 시작
        outs = []
        for _ in range(SEQ_OUT):
            o, h = self.dec(inp, h)
            d = self.head(o)
            outs.append(d)
            inp = d                                     # autoregressive
        return torch.cat(outs, 1)

    @torch.no_grad()
    def predict(self, track):
        """미래 8점 절대좌표 [(lat,lon)*8] 또는 None (점 부족)."""
        fp = track_features(track)
        if fp is None or len(fp[0]) < SEQ_IN:
            return None
        feat, pts = fp
        self.eval()
        d = self(torch.from_numpy(feat[-SEQ_IN:][None]))[0].numpy() / POS_SCALE
        cum = np.cumsum(d.astype(np.float64), 0)
        lat0, lon0 = pts[-1]
        return [(float(lat0 + dl), float(lon0 + dn)) for dl, dn in cum]

    @torch.no_grad()
    def anomaly_error(self, track):
        """직전 시점의 8스텝 예측 vs 실제 — 이동량 정규화 오차(무차원) 또는 None."""
        fp = track_features(track)
        if fp is None or len(fp[0]) < SEQ_IN + SEQ_OUT:
            return None
        pred = self.predict(track[:-SEQ_OUT])
        if pred is None:
            return None
        fut = track[-SEQ_OUT:]
        lat_p, lon_p = track[-SEQ_OUT - 1][1], track[-SEQ_OUT - 1][2]
        m_lon = _M_LAT * max(math.cos(math.radians(lat_p)), 0.05)
        errs, move = [], []
        for (pl, pn), f in zip(pred, fut):
            al, an = f[1], f[2]
            errs.append(math.hypot((pl - al) * _M_LAT, (pn - an) * m_lon))
            move.append(math.hypot((al - lat_p) * _M_LAT, (an - lon_p) * m_lon))
            lat_p, lon_p = al, an
        return float(np.mean(errs) / (np.mean(move) + 30.0))
