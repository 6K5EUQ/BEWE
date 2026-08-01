#include "df_manifold.hpp"

#include <cmath>

namespace df {

namespace {
constexpr double kC       = 299792458.0;
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
constexpr double kTwoPi   = 2.0 * 3.14159265358979323846;
}

double Manifold::lambda_m() const {
    return (freq_hz_ > 0.0) ? kC / freq_hz_ : 0.0;
}

double Manifold::ambiguity_ratio() const {
    if(m_ < 2 || freq_hz_ <= 0.0) return 0.0;
    // 인접 소자 간 현(chord) 길이 = 2 r sin(pi/M). 이게 lambda/2 를 넘으면
    // 조향벡터가 서로 다른 방위에서 반복되기 시작한다 (격자엽).
    const double chord = 2.0 * radius_m_ * std::sin(kTwoPi / (2.0 * m_));
    return chord / (0.5 * lambda_m());
}

void Manifold::steer(double bearing_deg, std::complex<double>* out) const {
    if(m_ <= 0 || freq_hz_ <= 0.0) return;
    const double k    = kTwoPi * freq_hz_ / kC;
    const double beta = bearing_deg * kDeg2Rad;
    const double dir  = (sense_ == Sense::CW) ? 1.0 : -1.0;
    for(int m = 0; m < m_; m++){
        // beta 는 어느 배선이든 "시계방향 방위" 로 고정이다. 배선 방향은 소자
        // 위치 phi_m 에만 걸린다.
        //   CW  : phi_m = +2 pi m / M  ->  cos( 2 pi m/M - beta)
        //   CCW : phi_m = -2 pi m / M  ->  cos( 2 pi m/M + beta)
        // beta 에도 같이 부호를 걸면 cos 가 우함수라 그대로 상쇄되어 CCW 가
        // CW 와 똑같아진다 (실제로 그 버그가 있었고 selftest 가 잡았다).
        const double phi = dir * kTwoPi * m / (double)m_;
        const double ph  = k * radius_m_ * std::cos(phi - beta);
        out[m] = std::complex<double>(std::cos(ph), std::sin(ph));
    }
}

void Manifold::ensure(double freq_hz, double radius_m, int elements, Sense sense){
    if(elements < 1 || elements > kMaxElements || freq_hz <= 0.0 || radius_m <= 0.0) return;
    if(m_ == elements && sense_ == sense
       && std::abs(freq_hz - freq_hz_) < 1.0            // 1 Hz 안쪽이면 같은 것으로 본다
       && std::abs(radius_m - radius_m_) < 1e-9) return;

    freq_hz_  = freq_hz;
    radius_m_ = radius_m;
    m_        = elements;
    sense_    = sense;
    sv_.resize((size_t)kAngleBins * m_);
    for(int b = 0; b < kAngleBins; b++)
        steer((double)b, &sv_[(size_t)b * m_]);

    // ── 배열 고유 사이드로브 ──────────────────────────────────────────────
    // |a(b0+d)^H a(b0)| / M 의 주엽 밖 최댓값. 단일 소스만 있어도 Bartlett
    // 스펙트럼에 이 크기(제곱)의 부엽이 반드시 나타나므로, 이보다 낮은 "두 번째
    // 봉우리" 는 방출체가 아니라 배열 기하다. 그걸 모호성으로 보고하면 깨끗한
    // 측정마다 거짓 경보가 뜬다 (M=5, 700 MHz 에서 180도에 -4.8 dB).
    //
    // 주엽 폭은 kr 에 따라 크게 변하므로 고정 가드(예: 40도)를 쓰면 저주파에서
    // 주엽 자신을 사이드로브로 잰다. 대신 b0 에서 상관이 처음 극소가 되는 지점
    // (첫 골)까지를 주엽으로 보고 그 밖만 훑는다.
    // M중 회전대칭이라 b0 는 [0, 360/M) 만 보면 된다.
    {
        sidelobe_db_ = -99.0; sidelobe_deg_ = 0.0;
        std::vector<std::complex<double>> a1(m_);
        const int b0_max = std::max(1, kAngleBins / m_);
        for(int i = 0; i < b0_max; i++){
            const std::complex<double>* a0 = col(i);
            // 첫 골 찾기
            double prev = 1.0;
            int    null_at = kAngleBins / 2;
            for(int d = 1; d < kAngleBins / 2; d++){
                std::complex<double> acc(0.0, 0.0);
                const std::complex<double>* ad = col((i + d) % kAngleBins);
                for(int k = 0; k < m_; k++) acc += ad[k] * std::conj(a0[k]);
                const double c = std::abs(acc) / m_;
                if(c > prev){ null_at = d; break; }
                prev = c;
            }
            for(int d = null_at; d <= kAngleBins - null_at; d++){
                std::complex<double> acc(0.0, 0.0);
                const std::complex<double>* ad = col((i + d) % kAngleBins);
                for(int k = 0; k < m_; k++) acc += ad[k] * std::conj(a0[k]);
                const double c = std::abs(acc) / m_;
                // Bartlett 전력 = 상관의 제곱이므로 20log10.
                const double db = 20.0 * std::log10(std::max(c, 1e-12));
                if(db > sidelobe_db_){ sidelobe_db_ = db; sidelobe_deg_ = (double)d; }
            }
        }
    }
}

} // namespace df
