// ── S곡선 적합 + 도플러 판정 ─────────────────────────────────────────────────
//
// 모델: f(t) = A - B*x/sqrt(1+x^2),  x = (t - t_tca)/tau
// 이 파라미터화를 쓰는 이유는 두 가지 성질 때문이다:
//   ① f(t_tca) = A 가 정확히 성립 (TCA 에서 시선속도 0) → 속도를 몰라도 A 가 정지주파수
//   ② 최대 기울기가 B/tau 로 바로 나옴 (TCA 에서)
#pragma once
#include "doppler_types.hpp"

namespace Doppler {

// VarPro 적합. (t_tca, tau) 가 주어지면 (A,B) 에 대해 선형이므로 안쪽은 폐형해,
// 바깥만 2D 격자 + 황금분할. 야코비안·감쇠상수·지역최소 병리가 전부 없다.
bool fit_scurve(const std::vector<TrackPoint>& pts, double cf_hz, double bin_hz,
                double row_rate_hz, SCurveFit& out);

// 게이트 10종 + 점수. 통과면 reason 이 비고 score>0.
// sens=Loose 면 swing_ratio 하한을 완화 (짧은/가장자리 관측 패스 구제).
float score_candidate(Candidate& c, std::string& reason,
                      Sensitivity sens = Sensitivity::Normal);

} // namespace Doppler
