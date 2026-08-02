#include "df_calib.hpp"

#include <algorithm>
#include <cmath>

namespace df {

namespace {
constexpr double kTwoPi = 6.283185307179586;

// 각도차를 (-180, 180] 로
double wrap180(double d){
    while(d <= -180.0) d += 360.0;
    while(d >   180.0) d -= 360.0;
    return d;
}
}

void Calib::set_context(double freq_hz, int elements){
    freq_hz_ = freq_hz;
    m_       = elements;
    stamp_++;
}

void Calib::clear(){ n_ = 0; freq_hz_ = 0.0; m_ = 0; stamp_++; }

void Calib::remove_at(int idx){
    if(idx < 0 || idx >= n_) return;
    for(int i = idx; i < n_-1; i++) pts_[i] = pts_[i+1];
    n_--;
    stamp_++;
}

void Calib::add(double bearing_deg, double snr_db,
                const std::complex<double>* measured,
                const std::complex<double>* theory,
                int elements){
    if(elements < 1 || elements > kMaxElements) return;
    bearing_deg = std::fmod(bearing_deg + 360.0, 360.0);

    CalPoint p;
    p.bearing_deg = bearing_deg;
    p.snr_db      = snr_db;
    // c[m] = x[m] / a[m]. 둘 다 소자 0 위상 0 으로 정규화돼 들어오므로 c[0] 은
    // 실수 양수가 된다. 이론값이 0 에 가까우면(있을 수 없지만) 보정을 1 로 둔다.
    for(int m = 0; m < elements; m++){
        const std::complex<double> a = theory[m];
        p.corr[m] = (std::abs(a) > 1e-12) ? measured[m] / a : std::complex<double>(1.0, 0.0);
    }
    // c[0] 을 정확히 1 로 맞춰 전체 위상 기준을 고정한다 (실측/이론 정규화가
    // 부동소수 오차로 살짝 어긋날 수 있다).
    if(std::abs(p.corr[0]) > 1e-12){
        const std::complex<double> inv = 1.0 / p.corr[0];
        for(int m = 0; m < elements; m++) p.corr[m] *= inv;
    }

    // 같은 방위가 이미 있으면 치환
    for(int i = 0; i < n_; i++){
        if(std::abs(wrap180(pts_[i].bearing_deg - bearing_deg)) < 2.0){
            pts_[i] = p;
            stamp_++;
            return;
        }
    }
    if(n_ >= kMaxCalPoints) return;

    // 방위 순 삽입
    int at = n_;
    for(int i = 0; i < n_; i++) if(pts_[i].bearing_deg > bearing_deg){ at = i; break; }
    for(int i = n_; i > at; i--) pts_[i] = pts_[i-1];
    pts_[at] = p;
    n_++;
    stamp_++;
}

bool Calib::usable_at(double freq_hz, int elements) const {
    if(n_ < 2 || m_ != elements || freq_hz_ <= 0.0 || freq_hz <= 0.0) return false;
    return std::abs(freq_hz - freq_hz_) / freq_hz_ < 0.05;
}

void Calib::correction(double bearing_deg, std::complex<double>* out, int elements) const {
    for(int m = 0; m < elements; m++) out[m] = std::complex<double>(1.0, 0.0);
    if(n_ < 2 || elements != m_) return;

    bearing_deg = std::fmod(bearing_deg + 360.0, 360.0);

    // 방위를 감싸는 두 측정점을 찾는다. 마지막 점과 첫 점 사이도 이어진다
    // (배열은 한 바퀴 도는 게 정상이고, 반쪽만 측정했어도 그 구간을 잇는 편이
    // 갑자기 무보정으로 튀는 것보다 낫다).
    int i0 = n_-1, i1 = 0;
    for(int i = 0; i < n_; i++){
        const int j = (i+1) % n_;
        double a = pts_[i].bearing_deg;
        double b = pts_[j].bearing_deg;
        double span = b - a; if(span <= 0) span += 360.0;
        double off  = bearing_deg - a; if(off < 0) off += 360.0;
        if(off <= span){ i0 = i; i1 = j; break; }
    }
    double a = pts_[i0].bearing_deg;
    double b = pts_[i1].bearing_deg;
    double span = b - a; if(span <= 0) span += 360.0;
    double off  = bearing_deg - a; if(off < 0) off += 360.0;
    const double t = (span > 1e-9) ? (off / span) : 0.0;

    // 진폭은 선형, 위상은 최단호. 복소수를 그대로 선형보간하면 두 끝의 위상이
    // 반대일 때 중간에서 크기가 0 으로 꺼진다 (보정이 소자를 죽여버린다).
    for(int m = 0; m < elements; m++){
        const std::complex<double> c0 = pts_[i0].corr[m];
        const std::complex<double> c1 = pts_[i1].corr[m];
        const double r0 = std::abs(c0), r1 = std::abs(c1);
        const double p0 = std::arg(c0);
        double dp = std::arg(c1) - p0;
        while(dp >  kTwoPi/2) dp -= kTwoPi;
        while(dp < -kTwoPi/2) dp += kTwoPi;
        const double r = r0 + (r1 - r0) * t;
        const double p = p0 + dp * t;
        out[m] = std::polar(r, p);
    }
}

double Calib::worst_dev_db() const {
    double worst = 0.0;
    for(int i = 0; i < n_; i++)
        for(int m = 0; m < m_; m++){
            const double a = std::abs(pts_[i].corr[m]);
            if(a > 1e-12) worst = std::max(worst, std::abs(20.0 * std::log10(a)));
        }
    return worst;
}

} // namespace df
