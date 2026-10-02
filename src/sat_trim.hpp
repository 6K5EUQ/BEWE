#pragma once
// ── 위성 협대역 녹음 재절단 (HOST 전용) ──────────────────────────────────────
// 위성 예약은 도플러를 따라가며 넉넉한 폭(기본 200 kHz)으로 1차 기록한다. 패스가
// 끝나면 파일 전체에서 실제 점유 대역폭을 재고, 그 폭 x1.25 로 다시 잘라 같은
// 경로에 덮어쓴다 (.sigmf-meta 의 SR/CF/BW 도 갱신).
//
// 측정 (2026-10-01 DGS-2 58400 패스로 Python 선검증 ~5.5 kHz, C++ 로 기존 녹음 재처리):
//   1초 전부 평균(bin ~250 Hz) → bin별 시간축 10%ile 와 행 중앙값으로 정규화 →
//   9bin 평활 최대가 +1 dB 넘는 행만 "신호 행" → 그 평균 PSD 에서 안쪽 절반의
//   중앙값=바닥, MAD=잡음편차 → 바닥 + max(1 dB, 6σ) 를 넘으며 피크에 이어진 구간
//   (2bin 틈 허용) = 점유 대역.
// 자르지 않고 1차 파일을 두는 경우: 신호 행 5개 미만, 또는 신호 행 피크의 60% 이상이
// 그 구간에 머물지 않음(도플러 추적 프레임에서 움직이는 지상 신호·스퓨리어스), 또는
// (dop_fn 이 있을 때) 그 행들의 도플러 변화가 500 Hz 미만(패스 끝 — 지상 반송파와 구별 불가).
// 피크는 캡처 중심 ±1/8 폭에서만 찾는다 (위성은 지정 주파수 근처에 있다).
#include <string>
#include <cstdint>
#include <functional>

namespace SatTrim {

struct Result {
    bool        trimmed      = false;
    int         signal_rows  = 0;
    double      occ_bw_hz    = 0;    // 측정 점유 대역폭
    double      center_off_hz= 0;    // 1차 파일 중심 대비 신호 중심
    uint32_t    sr_out       = 0;
    std::string why;                 // trimmed=false 사유
};

// dop_fn: 녹음 때 채널이 따라간 도플러 (t_unix → Hz). 주면 위성/지상 판별에 쓴다 —
// 패스 양끝은 도플러가 거의 안 변해 고정 지상 반송파도 추적 프레임에서 수십 초
// 머문다 (DGS-1 10/1 09:35 실측: +37 kHz 반송파 11행을 위성으로 잡아 0.73 kHz 로 잘랐다).
Result trim(const std::string& sigmf_data_path,
            const std::function<double(double)>& dop_fn = nullptr);

} // namespace SatTrim
