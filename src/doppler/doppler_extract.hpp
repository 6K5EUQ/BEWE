// ── HIST 스펙트로그램 → 주파수-시간 트랙 ──────────────────────────────────────
//
// 전부 **선형(fftshift된) bin 순서**로 작업한다. 저장 순서는 bin 0 이 DC 라 배열
// 경계에서 주파수가 불연속이고, 그러면 피크검출·게이트·트래커가 전부 대역 중심에서
// 정확히 깨진다. 언시프트는 행 누적 때 두 연속 구간 복사로 공짜에 가깝다.
#pragma once
#include "doppler_types.hpp"
#include <functional>

class HistReader;

namespace Doppler {

using ProgressFn = std::function<void(float, const char*)>;
using CancelFn   = std::function<bool()>;

// 시간 데시메이션 배수. 프레임 안 도플러 번짐을 0.7 bin 아래로 유지한다.
// 최악 도플러율은 최저 LEO 의 TCA: |df/dt| = f*(v/c)/tau_min, tau_min = 500km/7.6kms = 66s.
uint32_t auto_decimate(double bin_hz, double row_rate_hz, double cf_hz);

// 파일 1회 보정 — bin별 잡음바닥/마스크/임계/메인로브폭.
bool calibrate(HistReader& R, const ExtractParams& P, Calib& out,
               const ProgressFn& prog = nullptr, const CancelFn& cancel = nullptr);

// 전 파일 트랙 추출.
bool extract_tracks(HistReader& R, const ExtractParams& P, const Calib& C,
                    std::vector<Candidate>& out, ExtractStats& st,
                    const ProgressFn& prog = nullptr, const CancelFn& cancel = nullptr);

// Meas 박스 안 정밀분석 — row/linear-bin 범위는 뷰어의 Meas 가 이미 그 형태로 준다.
bool refine_in_box(HistReader& R, uint32_t row_lo, uint32_t row_hi,
                   uint32_t lin_lo, uint32_t lin_hi,
                   const ExtractParams& P, const Calib& C, Candidate& out,
                   const ProgressFn& prog = nullptr, const CancelFn& cancel = nullptr);

} // namespace Doppler
