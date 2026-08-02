#include "df_manifold.hpp"

#include <cmath>

namespace df {

namespace {
constexpr double kC       = 299792458.0;
constexpr double kDeg2Rad = 3.14159265358979323846 / 180.0;
constexpr double kTwoPi   = 2.0 * 3.14159265358979323846;
}

ArrayGeom make_geom(ArrayType t, int elements, double spacing_m, Sense sense){
    ArrayGeom g;
    if(elements < 1) elements = 1;
    if(elements > kMaxElements) elements = kMaxElements;
    g.n = elements;
    const double dir = (sense == Sense::CW) ? 1.0 : -1.0;

    switch(t){
    case ArrayType::Uca: {
        // 소자 m 은 시계방향 방위 2 pi m / M, 반경 spacing_m. 방위 psi 의 단위벡터가
        // (sin psi, cos psi) 이므로 좌표는 r(sin phi, cos phi) 다. 이렇게 두면
        // k*(x sin b + y cos b) = k r cos(phi - b) 로 기존 UCA 식과 정확히 같아진다.
        for(int m = 0; m < elements; m++){
            const double phi = dir * kTwoPi * m / (double)elements;
            g.x[m] = spacing_m * std::sin(phi);
            g.y[m] = spacing_m * std::cos(phi);
        }
        break;
    }
    case ArrayType::Ula: {
        // 동서로 뻗은 등간격 직선. 무게중심이 원점.
        const double x0 = -spacing_m * (elements - 1) / 2.0;
        for(int m = 0; m < elements; m++){ g.x[m] = x0 + spacing_m * m; g.y[m] = 0.0; }
        break;
    }
    case ArrayType::UlaPlus: {
        // M-1 개는 직선, 마지막 하나만 축 밖(북쪽)으로 spacing 만큼. 직선의 대칭을
        // 깨는 게 목적이라 옆으로 뺀 소자 하나면 충분하다.
        const int nl = (elements > 1) ? elements - 1 : 1;
        const double x0 = -spacing_m * (nl - 1) / 2.0;
        for(int m = 0; m < nl; m++){ g.x[m] = x0 + spacing_m * m; g.y[m] = 0.0; }
        if(elements > 1){ g.x[elements-1] = 0.0; g.y[elements-1] = spacing_m; }
        // 무게중심을 원점으로
        double cx = 0, cy = 0;
        for(int m = 0; m < elements; m++){ cx += g.x[m]; cy += g.y[m]; }
        cx /= elements; cy /= elements;
        for(int m = 0; m < elements; m++){ g.x[m] -= cx; g.y[m] -= cy; }
        break;
    }
    case ArrayType::Custom:
    default:
        // 호출측이 좌표를 들고 있다. 프리셋 생성 대상이 아니므로 UCA 로 초기값만.
        for(int m = 0; m < elements; m++){
            const double phi = dir * kTwoPi * m / (double)elements;
            g.x[m] = spacing_m * std::sin(phi);
            g.y[m] = spacing_m * std::cos(phi);
        }
        break;
    }
    return g;
}

double Manifold::lambda_m() const {
    return (freq_hz_ > 0.0) ? kC / freq_hz_ : 0.0;
}

double Manifold::ambiguity_ratio() const {
    if(m_ < 2 || freq_hz_ <= 0.0) return 0.0;
    // 최근접 소자쌍 간격 / (lambda/2). 이게 1 을 넘으면 조향벡터가 서로 다른
    // 방위에서 반복되기 시작한다 (격자엽). UCA 면 인접 현(2 r sin(pi/M)) 이
    // 최근접쌍이므로 예전 식과 같은 값이 나온다.
    double dmin = 1e30;
    for(int a = 0; a < m_; a++)
        for(int b = a+1; b < m_; b++){
            const double dx = gx_[a]-gx_[b], dy = gy_[a]-gy_[b];
            const double d  = std::sqrt(dx*dx + dy*dy);
            if(d > 1e-9 && d < dmin) dmin = d;
        }
    if(dmin > 1e29) return 0.0;
    return dmin / (0.5 * lambda_m());
}

void Manifold::steer(double bearing_deg, std::complex<double>* out) const {
    if(m_ <= 0 || freq_hz_ <= 0.0) return;
    const double k    = kTwoPi * freq_hz_ / kC;
    const double beta = bearing_deg * kDeg2Rad;
    // 나침반 방위 beta 의 신호원 쪽 단위벡터는 (sin beta, cos beta) 다. 소자
    // 좌표와의 내적이 곧 경로차이므로
    //   a_m(beta) = exp(+j k (x_m sin beta + y_m cos beta))
    // 이 식은 배치와 무관하다 — UCA 는 x=r sin(phi), y=r cos(phi) 를 넣으면
    // k r cos(phi - beta) 로 되돌아가는 특수해다. 배선 방향(sense)은 좌표를
    // 만들 때 이미 반영돼 있어 여기서 다시 걸지 않는다.
    const double sb = std::sin(beta), cb = std::cos(beta);
    for(int m = 0; m < m_; m++){
        const double ph = k * (gx_[m]*sb + gy_[m]*cb);
        out[m] = std::complex<double>(std::cos(ph), std::sin(ph));
    }
}

void Manifold::ensure(double freq_hz, const ArrayGeom& g){
    if(g.n < 1 || g.n > kMaxElements || freq_hz <= 0.0) return;
    // 소자가 전부 한 점에 몰려 있으면 방위 정보가 없다.
    { double span = 0;
      for(int m = 0; m < g.n; m++) span += std::abs(g.x[m]) + std::abs(g.y[m]);
      if(span <= 1e-9) return; }

    bool same = (m_ == g.n && std::abs(freq_hz - freq_hz_) < 1.0);   // 1 Hz 안쪽이면 같은 것
    if(same) for(int m = 0; m < g.n; m++)
        if(std::abs(g.x[m]-gx_[m]) > 1e-9 || std::abs(g.y[m]-gy_[m]) > 1e-9){ same = false; break; }
    if(same) return;

    freq_hz_ = freq_hz;
    m_       = g.n;
    for(int m = 0; m < g.n; m++){ gx_[m] = g.x[m]; gy_[m] = g.y[m]; }
    sv_.resize((size_t)kAngleBins * m_);
    for(int b = 0; b < kAngleBins; b++)
        steer((double)b, &sv_[(size_t)b * m_]);

    // ── 배열 고유 사이드로브 ──────────────────────────────────────────────
    // |a(b0+d)^H a(b0)| / M 의 주엽 밖 최댓값. 단일 소스만 있어도 Bartlett
    // 스펙트럼에 이 크기(제곱)의 부엽이 반드시 나타나므로, 이보다 낮은 "두 번째
    // 봉우리" 는 방출체가 아니라 배열 기하다. 그걸 모호성으로 보고하면 깨끗한
    // 측정마다 거짓 경보가 뜬다 (M=5, 700 MHz 에서 180도에 -4.8 dB).
    //
    // 주엽 폭은 배열 개구에 따라 크게 변하므로 고정 가드(예: 40도)를 쓰면 저주파에서
    // 주엽 자신을 사이드로브로 잰다. 대신 b0 에서 상관이 처음 극소가 되는 지점
    // (첫 골)까지를 주엽으로 보고 그 밖만 훑는다.
    //
    // 예전엔 UCA 의 M중 회전대칭을 믿고 b0 를 [0, 360/M) 만 봤다. 임의 배치는
    // 그 대칭이 없다 (ULA 는 축에 대해서만 대칭이고, 방향에 따라 부엽이 전혀
    // 다르다). b0 를 한 바퀴 다 훑되 5도씩 건너뛴다 — 부엽 봉우리는 그보다
    // 훨씬 넓어서 최댓값을 놓치지 않으면서 비용은 1/5 이다.
    {
        sidelobe_db_ = -99.0; sidelobe_deg_ = 0.0;
        const int b0_step = 5;
        for(int i = 0; i < kAngleBins; i += b0_step){
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
