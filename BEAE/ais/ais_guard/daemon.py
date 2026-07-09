"""GUARD 데몬 — AIS tail → tick_s마다 충돌/좌초/위협/이상 평가 → AlertBook.

이상항적 scorer_fn(mmsi, track)->(score, why)|None 과 궤적 predict_fn(mmsi)->
[(lat,lon)]|None 은 주입 인터페이스: run() 인자로 넘기거나, 없으면 패키지 내
anomaly 모듈(py-dl 제공)의 scorer_fn/predict_fn 을 자동 연결. 둘 다 없으면 skip.
"""
import logging
import logging.handlers
import os
import signal
import time

from .alerts import (TYP_ANOMALY, TYP_COLLISION, TYP_GROUNDING, TYP_THREAT, AlertBook)
from .collision import check_pairs, check_zones, load_zones
from .config import GuardConfig
from .tracker import TrackStore, kst_day
from . import overlay, report, threat


def _setup_logging(cfg):
    os.makedirs(cfg.guard_dir, exist_ok=True)
    h = logging.handlers.RotatingFileHandler(cfg.log_path, maxBytes=5 << 20, backupCount=3)
    h.setFormatter(logging.Formatter("%(asctime)s %(levelname)s %(name)s: %(message)s"))
    root = logging.getLogger()
    root.setLevel(logging.INFO)
    root.addHandler(h)
    root.addHandler(logging.StreamHandler())


def _pid_alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def _write_pidfile(cfg):
    """기존 데몬 살아있으면 RuntimeError, 죽은 pidfile은 회수."""
    try:
        old = int(open(cfg.pid_path).read().strip())
        if old != os.getpid() and _pid_alive(old):
            raise RuntimeError(f"guard daemon already running (pid {old})")
    except (OSError, ValueError):
        pass
    os.makedirs(cfg.guard_dir, exist_ok=True)
    with open(cfg.pid_path, "w") as f:
        f.write(str(os.getpid()))


def _load_hooks(tracker, log):
    """py-dl anomaly 모듈 자동 연결. predict는 (mmsi,track)→(mmsi) 어댑터."""
    try:
        from . import anomaly
    except Exception:                                   # torch 미설치 등 — 규칙층만 동작
        log.info("anomaly module unavailable -> DL hooks off")
        return None, None

    def predict_fn(mmsi):
        tr = tracker.tracks.get(mmsi)
        if tr is None or not tr.pts:
            return None
        return anomaly.predict_fn(mmsi, list(tr.pts))

    log.info("DL hooks connected (ais_guard.anomaly)")
    return anomaly.scorer_fn, predict_fn


def _ship(tr) -> str:
    return f"{tr.nm}({tr.mmsi})" if tr.nm else str(tr.mmsi)


def _tcpa_txt(tcpa_s: float) -> str:
    return f"{tcpa_s / 60:.1f}분 후" if tcpa_s >= 0 else "접근 중"


def _evaluate(cfg, tracker, book, zones, scorer_fn, predict_fn, now_ms):
    active = tracker.get_active(cfg.active_max_age_s, now_ms)

    # 1) 선박간 충돌 (CPA/TCPA)
    for a, b, cpa, tcpa, lat, lon in check_pairs(cfg, active, predict_fn):
        m1, m2 = sorted((a.mmsi, b.mmsi))               # aid 안정화
        sev = 3 if cpa <= cfg.cpa_crit_m else 2
        prox = max(0.0, 1.0 - cpa / cfg.cpa_warn_m)
        tfac = max(0.0, 1.0 - max(tcpa, 0.0) / cfg.tcpa_max_s)
        score = min(100.0, 100.0 * (0.6 * prox + 0.4 * tfac))
        msg = f"CPA {cpa:.0f}m {_tcpa_txt(tcpa)}: {_ship(a)} x {_ship(b)}"
        reco = "양 선박 VHF16 호출, 침로·속력 변경 지시"
        book.observe(TYP_COLLISION, sev, m1, m2, lat, lon, score, cpa, tcpa, msg, reco)

    # 2) 위험구역 진입/접근 (좌초)
    for tr, z, eta, lat, lon in check_zones(cfg, active, zones, predict_fn):
        inside = eta <= 0.0
        sev = 3 if inside else 2
        score = 100.0 if inside else min(100.0, 100.0 * (1.0 - eta / cfg.zone_horizon_s))
        what = "진입" if inside else f"{eta / 60:.0f}분 내 진입"
        msg = f"{z['name']} {what}: {_ship(tr)}"
        reco = "즉시 변침 지시, 구역 이탈 유도"
        book.observe(TYP_GROUNDING, sev, tr.mmsi, 0, lat, lon, score, -1.0,
                     0.0 if inside else eta, msg, reco)

    # 3) 위협선박 (Match_AI 불일치)
    for tr in active.values():
        r = threat.check(cfg, tr)
        if r is None:
            continue
        score, n_mis, n_tot, aim_top, n_jump = r
        sev = 3 if n_mis / n_tot >= cfg.threat_crit_ratio else 2
        latest = tr.latest()
        extra = f", RF점프 {n_jump}회" if n_jump else ""
        msg = f"RF지문 불일치 {n_mis}/{n_tot} (추정 {aim_top}){extra}: {_ship(tr)}"
        reco = "MMSI 위장 의심 — VHF·레이더 교차확인"
        book.observe(TYP_THREAT, sev, tr.mmsi, aim_top, latest[1], latest[2],
                     score, -1.0, -1.0, msg, reco)

    # 4) 이상항적 (DL scorer 주입 시)
    if scorer_fn is not None:
        for tr in active.values():
            try:
                r = scorer_fn(tr.mmsi, list(tr.pts))
            except Exception:
                logging.getLogger("ais_guard").exception("scorer_fn failed mmsi=%d", tr.mmsi)
                continue
            if not r:
                continue
            score, why = r
            if score < cfg.anom_score_warn:
                continue
            sev = 3 if score >= cfg.anom_score_crit else 2
            latest = tr.latest()
            msg = f"이상항적 {score:.0f}점: {_ship(tr)} — {why}"
            reco = "항적 감시, 지속 시 VHF 호출"
            book.observe(TYP_ANOMALY, sev, tr.mmsi, 0, latest[1], latest[2],
                         score, -1.0, -1.0, msg, reco)

    book.end_tick()
    return len(active)


def run(cfg: GuardConfig | None = None, scorer_fn=None, predict_fn=None):
    cfg = cfg or GuardConfig()
    _setup_logging(cfg)
    log = logging.getLogger("ais_guard.daemon")
    _write_pidfile(cfg)

    try:                                                # 비대해진 live 파이프 정리
        if os.path.getsize(cfg.alerts_live_path) > cfg.live_trim_bytes:
            open(cfg.alerts_live_path, "w").close()
            log.info("trimmed oversized alerts_live.jsonl")
    except OSError:
        pass

    stop = [False]
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__(0, True))
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__(0, True))

    tracker = TrackStore(cfg)
    if scorer_fn is None and predict_fn is None:
        scorer_fn, predict_fn = _load_hooks(tracker, log)
    book = AlertBook(cfg)
    zones = load_zones(cfg.zones_path)
    log.info("guard daemon start (ais_dir=%s zones=%d scorer=%s predict=%s)",
             cfg.ais_dir, len(zones), scorer_fn is not None, predict_fn is not None)
    try:
        overlay.publish_zones(cfg, log)                 # 위험구역 폴리곤 → 지도 오버레이 전파
    except Exception:
        log.exception("zone publish failed")

    day = kst_day()
    next_tick = next_status = 0.0
    last_prune = time.time()
    n_active = 0
    while not stop[0]:
        tracker.poll()
        now = time.time()
        if now >= next_tick:
            next_tick = now + cfg.tick_s
            try:
                n_active = _evaluate(cfg, tracker, book, zones, scorer_fn, predict_fn,
                                     int(now * 1000))
            except Exception:
                log.exception("evaluate failed")
        if now >= next_status:
            next_status = now + cfg.status_interval_s
            try:
                book.write_status({"tracks": len(tracker.tracks), "tracks_active": n_active,
                                   "lines": tracker.n_lines, "bad_lines": tracker.n_bad,
                                   "zones": len(zones), "day": day,
                                   "scorer": scorer_fn is not None,
                                   "predictor": predict_fn is not None})
            except Exception:
                log.exception("status write failed")
        if now - last_prune > 600:
            last_prune = now
            tracker.prune(cfg.track_prune_s)
        cur = kst_day()
        if cur != day:                                  # 자정 롤오버 → 전날 보고서
            try:
                path = report.generate(cfg, day)
                log.info("daily report -> %s", path)
            except Exception:
                log.exception("daily report failed")
            day = cur
            zones = load_zones(cfg.zones_path)          # 구역 정의 하루 1회 재적재
            try:
                overlay.publish_zones(cfg, log)         # 재적재분 오버레이 재전파
            except Exception:
                log.exception("zone publish failed")
        time.sleep(0.2)

    log.info("guard daemon stopping")
    try:
        os.unlink(cfg.pid_path)
    except OSError:
        pass
