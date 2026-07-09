"""TrajNet 학습 — 아카이브 ais_*.jsonl → 항적 윈도우(16+8) → GRU 학습 → publish.

publish는 ais_ai/fewshot/pipeline.py 패턴: data/guard/model/traj_vNNNN.pt +
current.json (버전+1, 최근 5개 보존). 같은 기간의 GridStats도 grid.npz로 갱신.

CLI: python -m ais_guard.train_traj --days N
"""
import argparse
import json
import os
import time

import numpy as np
import torch

from .grid import GridStats, GRID_PATH, MODEL_DIR, day_files, iter_positions
from .traj_model import TrajNet, track_features, SEQ_IN, SEQ_OUT, POS_SCALE, _M_LAT

SEED = 42
GAP_MS = 120_000                                        # 시간갭 > 120s → 항적 분절
STRIDE = 4                                              # 슬라이딩 윈도우 간격
MAX_STEP = 500.0                                        # |dlat,dlon|*1e4 상한 (텔레포트 컷)
MIN_WINDOWS = 200                                       # 학습 최소 윈도우 수
BATCH, LR, WD = 512, 1e-3, 1e-2
MAX_EPOCHS, PATIENCE = 60, 8


# ── 데이터 추출 ──────────────────────────────────────────────────────────────
def extract_tracks(paths, log=print):
    """MMSI별 위치보고 수집 → 시간갭 분절 → 항적 리스트 [(t,lat,lon,sog,cog)...]."""
    per = {}
    for path in paths:
        for t, mmsi, lat, lon, sog, cog in iter_positions(path):
            per.setdefault(mmsi, []).append((t, lat, lon, sog, cog))
    tracks = []
    for mmsi, pts in per.items():
        pts.sort(key=lambda p: p[0])
        seg = [pts[0]]
        for p in pts[1:]:
            if p[0] - seg[-1][0] > GAP_MS:
                if len(seg) > SEQ_IN + SEQ_OUT:
                    tracks.append(seg)
                seg = []
            seg.append(p)
        if len(seg) > SEQ_IN + SEQ_OUT:
            tracks.append(seg)
    log(f"tracks: {len(tracks)} segments from {len(per)} MMSIs")
    return tracks


def make_windows(tracks, log=print):
    """항적 → (X[N,16,6], Y[N,8,2], lat0[N], t[N]). Y는 POS_SCALE 스케일."""
    W = SEQ_IN + SEQ_OUT
    xs, ys, lat0s, ts = [], [], [], []
    for seg in tracks:
        fp = track_features(seg)
        if fp is None or len(fp[0]) < W:
            continue
        feat, pts = fp
        for i in range(0, len(feat) - W + 1, STRIDE):
            w = feat[i:i + W]
            if np.abs(w[:, :2]).max() > MAX_STEP or w[:, 5].max() > 12.0:
                continue                                # 글리치/갭 낀 윈도우 제외
            xs.append(w[:SEQ_IN])
            ys.append(w[SEQ_IN:, :2])
            lat0s.append(pts[i + SEQ_IN, 0])            # 예측 시작점 위도
            ts.append(seg[0][0])
    if not xs:
        return None
    X = np.stack(xs); Y = np.stack(ys)
    log(f"windows: {len(X)}")
    return X, Y, np.asarray(lat0s, np.float64), np.asarray(ts, np.int64)


def time_split(X, Y, lat0, t, val_frac=0.15):
    """최근 시간대 15%를 검증셋으로 (시간 누수 방지)."""
    order = np.argsort(t, kind="stable")
    cut = int(len(order) * (1.0 - val_frac))
    tr, va = order[:cut], order[cut:]
    if len(va) == 0 or len(tr) == 0:                    # 데이터 극소 → 앞뒤 반반
        half = max(1, len(order) // 2)
        tr, va = order[:half], order[half:]
    return (X[tr], Y[tr]), (X[va], Y[va], lat0[va])


# ── 검증 오차 분포 (anomaly.py 정규화용 백분위) ──────────────────────────────
@torch.no_grad()
def val_error_pctl(model, vaX, vaY, va_lat0, device):
    """anomaly_error()와 같은 정의의 정규화 오차 → 1..99 백분위."""
    model.eval()
    errs = []
    for i in range(0, len(vaX), 2048):
        x = torch.from_numpy(vaX[i:i + 2048]).to(device)
        pred = model(x).cpu().numpy().astype(np.float64) / POS_SCALE
        act = vaY[i:i + 2048].astype(np.float64) / POS_SCALE
        m_lon = _M_LAT * np.maximum(np.cos(np.radians(va_lat0[i:i + 2048])), 0.05)
        pc, ac = np.cumsum(pred, 1), np.cumsum(act, 1)  # 절대 오프셋 (도)
        d = pc - ac
        e = np.hypot(d[:, :, 0] * _M_LAT, d[:, :, 1] * m_lon[:, None]).mean(1)
        mv = np.hypot(act[:, :, 0] * _M_LAT, act[:, :, 1] * m_lon[:, None]).mean(1)
        errs.append(e / (mv + 30.0))
    errs = np.concatenate(errs)
    return np.percentile(errs, np.arange(1, 100)).tolist()


# ── 학습 ─────────────────────────────────────────────────────────────────────
def train(X, Y, lat0, t, log=print):
    dev = "cuda" if torch.cuda.is_available() else "cpu"
    torch.manual_seed(SEED)
    (trX, trY), (vaX, vaY, va_lat0) = time_split(X, Y, lat0, t)
    log(f"train={len(trX)} val={len(vaX)} device={dev}")
    model = TrajNet().to(dev)
    opt = torch.optim.AdamW(model.parameters(), lr=LR, weight_decay=WD)
    rng = np.random.default_rng(SEED)
    vX = torch.from_numpy(vaX).to(dev)
    vY = torch.from_numpy(vaY).to(dev)
    best, best_state, bad = float("inf"), None, 0
    t0 = time.time()
    for ep in range(MAX_EPOCHS):
        model.train()
        perm = rng.permutation(len(trX))
        tot = 0.0
        for i in range(0, len(perm), BATCH):
            idx = perm[i:i + BATCH]
            x = torch.from_numpy(trX[idx]).to(dev)
            y = torch.from_numpy(trY[idx]).to(dev)
            loss = torch.nn.functional.mse_loss(model(x), y)
            opt.zero_grad(); loss.backward(); opt.step()
            tot += float(loss.detach()) * len(idx)
        model.eval()
        with torch.no_grad():
            vl = float(torch.nn.functional.mse_loss(model(vX), vY))
        log(f"  ep {ep+1}/{MAX_EPOCHS} train={tot/len(trX):.4f} val={vl:.4f}")
        if vl < best - 1e-4:
            best, bad = vl, 0
            best_state = {k: v.cpu().clone() for k, v in model.state_dict().items()}
        else:
            bad += 1
            if bad >= PATIENCE:
                log(f"early stop at ep {ep+1}")
                break
    model.load_state_dict(best_state)
    log(f"trained in {time.time()-t0:.1f}s val_mse={best:.4f}")
    pctl = val_error_pctl(model, vaX, vaY, va_lat0, dev)
    return model.cpu(), best, pctl, len(trX), len(vaX)


# ── publish (proto/pipeline.py 패턴) ─────────────────────────────────────────
def publish(model, val_mse, pctl, days, n_tr, n_va, log=print):
    os.makedirs(MODEL_DIR, exist_ok=True)
    cur = os.path.join(MODEL_DIR, "current.json")
    ver = 1
    if os.path.exists(cur):
        try:
            ver = int(json.load(open(cur)).get("version", 0)) + 1
        except Exception:
            ver = 1
    mname = f"traj_v{ver:04d}.pt"
    torch.save({"state_dict": model.state_dict(), "err_pctl": pctl,
                "val_mse": val_mse, "days": days, "tr_n": n_tr, "va_n": n_va,
                "trained_at": time.strftime("%Y-%m-%dT%H:%M:%S%z")},
               os.path.join(MODEL_DIR, mname))
    tmp = cur + ".tmp"
    json.dump({"version": ver, "model": mname}, open(tmp, "w"))
    os.replace(tmp, cur)
    olds = sorted(f for f in os.listdir(MODEL_DIR) if f.startswith("traj_v"))
    for f in olds[:-5]:                                 # 최근 5개 보존 (롤백)
        os.remove(os.path.join(MODEL_DIR, f))
    log(f"published traj v{ver} ({mname}) -> {MODEL_DIR}")
    return ver


def main():
    ap = argparse.ArgumentParser(description="GUARD 항적예측 모델 학습")
    ap.add_argument("--days", type=int, default=14, help="최근 N일치 아카이브 사용 (0=전부)")
    args = ap.parse_args()
    paths = day_files(args.days)
    if not paths:
        print("no ais_*.jsonl archives found — skip")
        return
    print(f"archives: {len(paths)} files ({os.path.basename(paths[0])} .. "
          f"{os.path.basename(paths[-1])})")
    tracks = extract_tracks(paths)
    win = make_windows(tracks)
    if win is None or len(win[0]) < MIN_WINDOWS:
        print(f"not enough windows (<{MIN_WINDOWS}) — skip training")
        return
    model, val_mse, pctl, n_tr, n_va = train(*win)
    publish(model, val_mse, pctl, args.days, n_tr, n_va)
    grid = GridStats()                                  # 같은 기간으로 격자 통계 갱신
    n = grid.build(paths)
    grid.save(GRID_PATH)
    print(f"grid: {n} positions, {len(grid.cells)} cells -> {GRID_PATH}")


if __name__ == "__main__":
    main()
