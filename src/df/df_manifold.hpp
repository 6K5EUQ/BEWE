#pragma once
// ── UCA 조향 매니폴드 ─────────────────────────────────────────────────────
//
// 유도 (물리에서. Kraken 식을 옮겨오지 않았다 — 부호가 반대다)
// ----------------------------------------------------------------------
// 나침반 좌표 x^ = East, y^ = North. 시계방향 방위 psi 는 단위벡터
// (sin psi, cos psi) 다. 소자 m 이 시계방향 방위 phi_m, 반경 r 에 있고
// 신호원이 시계방향 방위 beta 에 있으면
//
//   p_m   = r (sin phi_m, cos phi_m)
//   u     = (sin beta, cos beta)                 신호원 쪽 단위벡터
//   u.p_m = r cos(phi_m - beta)
//
// 평면파 E(p,t) = A cos(wt - k.p), k = k n^, n^ = -u (신호원 -> 배열)
// 이므로 E(p_m,t) = A cos(wt + k u.p_m). 통상적인 직교 하향변환
// (2 e^{-jwt} 를 곱하는 쪽, librtlsdr 이 내놓는 규약)을 거치면
//
//   a_m(beta) = exp( +j (2 pi f / c) r cos(phi_m - beta) )      <= 구현할 식
//
// UCA 는 phi_m = 2 pi m / M (시계방향 번호). 따라서 추정기의 argmax 가
// 곧 "안테나 0 기준 시계방향 방위" 다. 360-theta 뒤집기가 필요 없다.
//
// Kraken 과의 관계 (수치로 확인함, df_selftest.cpp 에 단언으로 박아둠)
// ----------------------------------------------------------------------
// Kraken 의 gen_scanning_vectors 는 x=R cos, y=-R sin 에 exp(j2pi(x cos t +
// y sin t)) 를 써서 exp(+j k R cos(2 pi m/M + t)) 가 된다. 부호가 반대이므로
//   theta_kraken = (360 - beta) mod 360
// 즉 Kraken 의 raw theta 는 반시계 방위다. 그 식을 그대로 옮겼으면 방위가
// 거울로 나왔을 것이다.
//
// 남는 불확실성은 수학이 아니라 물리다: 배열이 위에서 볼 때 정말 시계방향인지,
// 그리고 하향변환이 스펙트럼 반전이 아닌지. 둘 다 실측으로만 확정된다.
// 그래서 Sense 를 런타임 설정으로 둔다 — 현장에서 재빌드 없이 뒤집는다.

#include "df_types.hpp"
#include "df_calib.hpp"
#include <complex>
#include <vector>

namespace df {

inline constexpr int kAngleBins = 360;   // 1도 격자

class Manifold {
public:
    // 필요한 파라미터가 바뀌었을 때만 재생성한다. 조향벡터 생성은
    // M*360 개의 exp() 라 싸지 않고, 측정마다 부르게 된다.
    // 기하는 소자 좌표로 받는다 — UCA/ULA/임의 배치가 같은 경로를 탄다.
    //
    // cal 을 주면 그 보정을 곱한 매니폴드를 만든다 (실측 캘리브레이션). 보정이
    // 걸린 테이블은 캐시 비교에도 반영되므로, 캘리브를 갱신하면 다음 ensure 가
    // 알아서 다시 만든다.
    void ensure(double freq_hz, const ArrayGeom& geom, const Calib* cal = nullptr);

    // 이 테이블에 캘리브 보정이 걸려 있는가 (UI 표시용).
    bool calibrated() const { return cal_stamp_ != 0; }

    // 열 우선 접근: bin b (=시계방향 도), 소자 m
    const std::complex<double>* col(int bin) const { return &sv_[(size_t)bin * m_]; }
    int  elements() const { return m_; }
    bool valid()    const { return m_ > 0 && freq_hz_ > 0.0; }

    // 임의 각도(정수 격자 밖)의 조향벡터. 피크 국소 보정에 쓴다.
    void steer(double bearing_deg, std::complex<double>* out) const;

    // 격자엽 지표: 최근접 소자쌍 간격 / (lambda/2). 1 을 넘으면 모호성이 생긴다.
    //
    // 주의 — 이건 ULA 휴리스틱이라 UCA 에서는 양방향으로 틀린다. M=5, r=0.175 의
    // 700 MHz(현 운용점)에서 0.96 을 내놓아 "안전" 이라 하지만, 실제 배열 상관은
    // 정확히 180도에서 0.5732 (= -4.8 dB) 다. 와이어/UI 호환 때문에 남겨두되
    // 판단에는 아래 sidelobe_db() 를 쓸 것.
    double ambiguity_ratio() const;
    double lambda_m() const;

    // 배열 고유 사이드로브 (dB, <=0). 단일 소스 하나만 있어도 Bartlett 스펙트럼에
    // 이만큼의 부엽이 반드시 생긴다 — 즉 이 아래의 "두 번째 봉우리" 는 방출체가
    // 아니라 기하다. ensure() 에서 1회 계산한다 (M중 회전대칭이라 beta0 는
    // [0,360/M) 만 쓸면 되고, m=5 에서 ~0.3 ms).
    double sidelobe_db()  const { return sidelobe_db_; }
    double sidelobe_deg() const { return sidelobe_deg_; }

private:
    std::vector<std::complex<double>> sv_;   // 360 * M, bin-major
    double freq_hz_ = 0.0;
    double gx_[kMaxElements] = {}, gy_[kMaxElements] = {};   // 소자 좌표 (m)
    int    m_ = 0;
    uint32_t cal_stamp_ = 0;   // 반영된 Calib::stamp() (0=무보정). 캐시 비교용
    double sidelobe_db_ = 0.0, sidelobe_deg_ = 0.0;
};

} // namespace df
