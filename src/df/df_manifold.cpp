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
}

} // namespace df
