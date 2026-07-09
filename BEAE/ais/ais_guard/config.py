"""GUARD 엔진 단일 설정 (ais_ai/config.py 패턴)."""
import os
from dataclasses import dataclass, field


def _home() -> str:
    return os.path.expanduser("~")


@dataclass
class GuardConfig:
    # ── 경로 ────────────────────────────────────────────────────────────────
    ais_dir: str = field(default_factory=lambda: os.path.join(
        _home(), "BEWE", "modules", "ais"))                 # HOST가 append하는 일별 jsonl
    guard_dir: str = field(default_factory=lambda: os.path.join(
        _home(), "BEWE", "BEAE", "ais", "data", "guard"))
    alerts_live_path: str = ""      # C++가 tail하는 경보 파이프
    zones_path: str = ""            # 위험구역 폴리곤 정의
    status_path: str = ""           # guard_status.json
    model_dir: str = ""             # traj_vNNNN.pt + current.json (py-dl)
    log_path: str = ""
    pid_path: str = ""

    # ── 충돌(CPA/TCPA) ─────────────────────────────────────────────────────
    cpa_warn_m: float = 300.0       # CPA 경고 반경
    cpa_crit_m: float = 100.0       # CPA 심각 반경
    tcpa_max_s: float = 360.0       # 이 시간 내 최근접만 경보
    pair_max_dist_m: float = 20000.0  # 현재 이보다 먼 쌍은 계산 생략
    pair_min_sog_kn: float = 1.0    # 둘 다 이 속력 미만(정박)이면 생략
    predict_step_s: float = 30.0    # predict_fn 궤적 점 간격 가정

    # ── 위험구역(좌초) ──────────────────────────────────────────────────────
    zone_horizon_s: float = 600.0   # 진입 예측 시계
    zone_step_s: float = 30.0       # 등속 외삽 샘플 간격

    # ── 이상항적 ────────────────────────────────────────────────────────────
    anom_score_warn: float = 70.0   # scorer 점수 경보 임계
    anom_score_crit: float = 90.0   # 심각 임계

    # ── 위협선박(Match_AI 불일치) ───────────────────────────────────────────
    threat_min_bursts: int = 5      # 판정 최소 Match_AI 버스트 수
    threat_ratio: float = 0.6       # 불일치 비율 임계
    threat_crit_ratio: float = 0.8  # 심각 임계
    cfo_jump_hz: float = 200.0      # 동일 MMSI 연속 버스트 CFO 급변
    rssi_jump_db: float = 15.0      # 동일 MMSI 연속 버스트 RSSI 급변

    # ── 트래커/루프 ─────────────────────────────────────────────────────────
    track_window: int = 32          # MMSI별 보관 포인트/버스트 수
    tick_s: float = 2.0             # 평가 주기
    active_max_age_s: float = 180.0  # 이 나이 이내 위치만 활성 트랙
    track_prune_s: float = 3600.0   # 오래된 트랙 메모리 정리
    clear_ticks: int = 3            # 해제조건 연속 틱(히스테리시스)
    update_min_interval_s: float = 30.0  # UPDATE 재발행 최소 간격
    update_score_delta: float = 5.0      # 점수 변화 시 즉시 UPDATE
    status_interval_s: float = 10.0
    live_trim_bytes: int = 5 << 20  # 데몬 시작 시 alerts_live 이보다 크면 비움

    def __post_init__(self):
        if not self.alerts_live_path:
            self.alerts_live_path = os.path.join(self.guard_dir, "alerts_live.jsonl")
        if not self.zones_path:
            self.zones_path = os.path.join(self.guard_dir, "zones.json")
        if not self.status_path:
            self.status_path = os.path.join(self.guard_dir, "guard_status.json")
        if not self.model_dir:
            self.model_dir = os.path.join(self.guard_dir, "model")
        if not self.log_path:
            self.log_path = os.path.join(self.guard_dir, "guard_daemon.log")
        if not self.pid_path:
            self.pid_path = os.path.join(self.guard_dir, "guard.pid")
