#pragma once
// ── 위성 협대역 녹음 재절단 (HOST 전용) ──────────────────────────────────────
// 위성 예약은 도플러를 따라가며 넉넉한 폭(기본 200 kHz)으로 1차 기록한다. 패스가
// 끝나면 파일 전체에서 실제 점유 대역폭을 재고, 그 폭 x1.25 로 다시 잘라 같은
// 경로에 덮어쓴다 (.sigmf-meta 의 SR/CF/BW 도 갱신).
//
// 측정 (2026-10-01 DGS-2 58400 패스 2개로 Python 선검증, 둘 다 ~5.5 kHz):
//   1초마다 FFT 16회 평균 → 행별 중앙값으로 정규화 → 5bin 평활 최대가 +3 dB 넘는
//   행만 "신호 행" → 그 평균 PSD 에서 안쪽 절반의 중앙값=바닥, MAD=잡음편차 →
//   바닥 + max(1 dB, 6σ) 를 넘으며 피크에 이어진 구간(2bin 틈 허용) = 점유 대역.
// 신호 행이 5개 미만이면 자르지 않고 원본을 둔다 (대역폭을 잴 수 없다).
#include <string>
#include <cstdint>

namespace SatTrim {

struct Result {
    bool        trimmed      = false;
    int         signal_rows  = 0;
    double      occ_bw_hz    = 0;    // 측정 점유 대역폭
    double      center_off_hz= 0;    // 1차 파일 중심 대비 신호 중심
    uint32_t    sr_out       = 0;
    std::string why;                 // trimmed=false 사유
};

Result trim(const std::string& sigmf_data_path);

} // namespace SatTrim
