#include "df_mrc.hpp"

#include <algorithm>
#include <cmath>

namespace df {

namespace {
// 무게 벡터를 단위 2-노름으로. 노름이 붕괴하면 e_0 로 되돌린다.
void normalize(std::complex<double>* w, int m) {
    double s = 0.0;
    for (int i = 0; i < m; i++) s += std::norm(w[i]);
    if (!(s > 1e-30)) {                      // NaN 포함
        for (int i = 0; i < m; i++) w[i] = 0.0;
        w[0] = 1.0;
        return;
    }
    const double inv = 1.0 / std::sqrt(s);
    for (int i = 0; i < m; i++) w[i] *= inv;
}
} // namespace

void Mrc::reset() {
    engaged_ = false;
    m_ = 0;
    center_hz_ = fs_hz_ = 0;
    prev_usable_ = false;
    for (int i = 0; i < kMaxElements; i++) {
        c_[i] = 0.0; p_[i] = 0.0; dc_[i] = 0.0;
        w_target_[i] = 0.0; w_prev_[i] = 0.0;
    }
    w_target_[0] = 1.0;
    w_prev_[0]   = 1.0;
    stats_init_ = false;
    ramp_valid_ = false;
    frames_ = 0;
    warm_ = kMrcWarm;
    dwell_on_ = dwell_off_ = 0;
    gamma_ = 0.0;
    ch0_weak_ = false;
}

void Mrc::set_enabled(bool on) {
    if (on == enabled_) return;
    enabled_ = on;
    if (!on) {
        // 끄는 순간 램프가 e_0 를 향하게 둔다. 결합 자체는 다음 프레임부터
        // 건너뛰지만, 그 프레임의 남은 구간은 부드럽게 빠져나간다.
        engaged_ = false;
    }
}

void Mrc::observe(const std::complex<float>* iq, uint32_t channels,
                  uint32_t samples_per_ch, uint64_t center_hz, uint64_t fs_hz,
                  bool usable) {
    if (!iq || channels < 2) { engaged_ = false; return; }

    int m = (int)channels;
    if (m > kMaxElements) m = kMaxElements;

    // 재튠·샘플레이트 변경·채널수 변경은 조향벡터를 통째로 무효화한다.
    // DAQ 재캘리브(!usable)에서 복귀하는 경계도 마찬가지 — 그 사이 heimdall 이
    // 채널 간 지연/IQ 보정을 다시 잡았으므로 이전 통계는 다른 하드웨어의 것이다.
    const bool ctx_changed = (m != m_) || (center_hz != center_hz_) ||
                             (fs_hz != fs_hz_) || (usable && !prev_usable_);
    if (ctx_changed) {
        const bool was_enabled = enabled_;
        reset();
        enabled_ = was_enabled;
        m_ = m;
        center_hz_ = center_hz;
        fs_hz_ = fs_hz;
    }
    prev_usable_ = usable;

    // 보정 전 프레임은 위상이 틀리다. 통계에 넣으면 가중치가 오염된다.
    if (!usable) { engaged_ = false; return; }
    if (samples_per_ch < (uint32_t)kMrcSnapshots) { engaged_ = false; return; }

    // ── 블록 통계 ────────────────────────────────────────────────────────
    // 블록마다 복소 평균을 빼고 누적한다. 평균제거는 길이 L 창의 고역통과라
    // 응답이 1 - sinc(f L / fs) — L=128, fs=2.4 MSPS 에서 1.9 kHz 에 -20 dB,
    // 20 kHz 위로는 -0.1 dB 다. LO 누설을 죽이는 게 목적이다: 동글이 클럭을
    // 공유해 DC 누설이 채널 간 **코히어런트**라, 그냥 두면 가중치가 방위와
    // 무관한 누설 벡터에 고정된다 (DF 가 dc_guard_hz=2 kHz 로 막는 것과 같은 문제).
    const uint32_t stride = samples_per_ch / (uint32_t)kMrcBlocks;
    std::complex<double> c_f[kMaxElements] = {};
    double               p_f[kMaxElements] = {};
    double               dc_f[kMaxElements] = {};

    for (int b = 0; b < kMrcBlocks; b++) {
        const size_t off = (size_t)b * stride;
        std::complex<double> mean[kMaxElements];
        for (int k = 0; k < m; k++) {
            const std::complex<float>* x = iq + (size_t)k * samples_per_ch + off;
            double sr = 0.0, si = 0.0;
            for (int i = 0; i < kMrcBlockLen; i++) { sr += x[i].real(); si += x[i].imag(); }
            mean[k] = std::complex<double>(sr / kMrcBlockLen, si / kMrcBlockLen);
            dc_f[k] += std::norm(mean[k]);
        }
        for (int i = 0; i < kMrcBlockLen; i++) {
            const std::complex<double> x0 =
                std::complex<double>(iq[off + i].real(), iq[off + i].imag()) - mean[0];
            p_f[0] += std::norm(x0);
            const std::complex<double> x0c = std::conj(x0);
            for (int k = 1; k < m; k++) {
                const std::complex<float>* xk = iq + (size_t)k * samples_per_ch + off;
                const std::complex<double> x =
                    std::complex<double>(xk[i].real(), xk[i].imag()) - mean[k];
                p_f[k] += std::norm(x);
                c_f[k] += x * x0c;
            }
        }
    }
    c_f[0] = p_f[0];   // 정의상

    // ── EMA ──────────────────────────────────────────────────────────────
    // 통계량을 정규화하지 않고 그대로 평균한다. 센 프레임이 평균을 지배하는
    // 게 맞다 — 가중치는 "합칠 게 있었던 순간" 이 정해야 한다. 신호가 사라지면
    // 지수감쇠가 tau 안에 원래대로 끌어내린다.
    if (!stats_init_) {
        for (int k = 0; k < m; k++) { c_[k] = c_f[k]; p_[k] = p_f[k]; dc_[k] = dc_f[k]; }
        stats_init_ = true;
    } else {
        const double a = kMrcAlpha, b1 = 1.0 - kMrcAlpha;
        for (int k = 0; k < m; k++) {
            c_[k] = b1 * c_[k] + a * c_f[k];
            p_[k] = b1 * p_[k] + a * p_f[k];
            dc_[k] = b1 * dc_[k] + a * dc_f[k];
        }
    }
    frames_++;
    if (warm_ > 0) warm_--;

    // ── 코히런스 ─────────────────────────────────────────────────────────
    // gamma = mean_m |c_m|^2 / (p_0 p_m). 잡음만이면 ~1/N = 1.2e-4.
    double g = 0.0;
    int gn = 0;
    for (int k = 1; k < m; k++) {
        const double d = p_[0] * p_[k];
        if (d > 1e-30) { g += std::norm(c_[k]) / d; gn++; }
    }
    gamma_ = gn ? g / gn : 0.0;

    // ch0 이 죽으면 위상 기준 자체가 없다. 그런데 MRC off 경로도 ch0 을 쓰므로
    // 이건 MRC 고장이 아니라 하드웨어 고장이다 — 표시만 하고 결합을 멈춘다.
    double pmed = 0.0;
    { double tmp[kMaxElements]; for (int k = 0; k < m; k++) tmp[k] = p_[k];
      std::sort(tmp, tmp + m); pmed = tmp[m / 2]; }
    ch0_weak_ = (pmed > 1e-30) && (p_[0] < 0.05 * pmed);

    // ── 개시/해제 (드웰 포함) ────────────────────────────────────────────
    if (gamma_ > kMrcGammaOn) { dwell_on_++; dwell_off_ = 0; }
    else if (gamma_ < kMrcGammaOff) { dwell_off_++; dwell_on_ = 0; }
    else { dwell_on_ = dwell_off_ = 0; }

    const bool healthy = enabled_ && !ch0_weak_ && warm_ == 0;
    if (healthy && dwell_on_ >= kMrcDwell) engaged_ = true;
    if (!healthy || dwell_off_ >= kMrcDwell) engaged_ = false;

    solve_weights_();
    build_ramp_();
}

void Mrc::solve_weights_() {
    const int m = m_;
    if (m < 2) { w_target_[0] = 1.0; return; }

    std::complex<double> w[kMaxElements] = {};
    if (!engaged_) {
        // 결합 안 할 땐 목표를 e_0 로 둔다. 램프가 그쪽으로 미끄러진다.
        w[0] = 1.0;
        normalize(w, m);
        for (int k = 0; k < m; k++) w_target_[k] = w[k];
        for (int k = m; k < kMaxElements; k++) w_target_[k] = 0.0;
        return;
    }

    // MRC 가중치는 **신호가 아니라 잡음으로** 나눈다: w_m ∝ a_m / sigma_m^2.
    // c_m/p_m 을 그냥 쓰면 저SNR 극한에서만 맞고 고SNR 에선 뒤집힌다 —
    // p_m -> P|a_m g_m|^2 가 되어 w ∝ 1/g_m, 즉 약한 소자를 **더** 믿게 된다
    // (셀프테스트의 -10 dB 불균형 케이스가 이걸 잡아냈다).
    //
    // x_m = g_m a_m s + nu_m 모델에서
    //     p_m = P g_m^2 + sigma^2,   c_m = P g_m g_0 a_m conj(a_0)
    // 이므로 소자 m 의 신호전력은 |c_m|^2 / (P g_0^2) 이고, 그 P g_0^2 는
    // p_0 - sigma^2 다. sigma^2 가 양변에 있어 자기참조라 몇 번 반복하면 수렴한다
    // (5x5 에 반복 8회는 공짜다).
    double s2 = p_[0];
    for (int k = 1; k < m; k++) s2 = std::min(s2, p_[k]);
    s2 *= 0.5;                                  // 초기값: 최소 채널전력의 절반
    for (int it = 0; it < 8; it++) {
        const double ps0 = p_[0] - s2;
        if (!(ps0 > 1e-30)) break;              // 신호가 없다 — 초기값을 그대로 둔다
        double acc = 0.0; int n = 0;
        for (int k = 0; k < m; k++) {
            const double sm = p_[k] - std::norm(c_[k]) / ps0;
            if (std::isfinite(sm) && sm > 0.0) { acc += sm; n++; }
        }
        if (!n) break;
        s2 = acc / n;
    }

    const double ps0 = p_[0] - s2;
    w[0] = 1.0;
    for (int k = 1; k < m; k++) {
        // 소자별 잡음. 추정이 무너지면 (음수/비유한) 공통 s2 로 폴백한다.
        double sm = (ps0 > 1e-30) ? (p_[k] - std::norm(c_[k]) / ps0) : p_[k];
        if (!std::isfinite(sm) || sm < 1e-30) sm = (s2 > 1e-30) ? s2 : p_[k];
        w[k] = (sm > 1e-30) ? c_[k] / sm : std::complex<double>(0.0, 0.0);
    }
    // w_0 을 실수 양수로 유지하기 위해 ch0 의 잡음으로 같은 정규화를 건다.
    {
        double s0 = (ps0 > 1e-30) ? (p_[0] - std::norm(c_[0]) / ps0) : p_[0];
        if (!std::isfinite(s0) || s0 < 1e-30) s0 = (s2 > 1e-30) ? s2 : p_[0];
        w[0] = (s0 > 1e-30) ? c_[0] / s0 : std::complex<double>(1.0, 0.0);
    }
    for (int k = 0; k < m; k++)
        if (!std::isfinite(w[k].real()) || !std::isfinite(w[k].imag())) w[k] = 0.0;

    // c_0 = p_0 (실수 양수) 이므로 w_0 도 실수 양수다 — 위상 앵커가 구조적으로
    // ch0 에 걸린다. 결합 출력이 ch0 과 같은 반송파 위상을 갖는다는 뜻이고,
    // 그래서 on/off 토글과 프레임 경계가 위상 연속이다.
    normalize(w, m);
    for (int k = 0; k < m; k++) w_target_[k] = w[k];
    for (int k = m; k < kMaxElements; k++) w_target_[k] = 0.0;
}

void Mrc::build_ramp_() {
    const int m = m_;
    ramp_.assign((size_t)kMrcRampSeg * m, std::complex<float>(0.f, 0.f));
    for (int s = 0; s < kMrcRampSeg; s++) {
        const double t = (s + 0.5) / kMrcRampSeg;
        std::complex<double> w[kMaxElements];
        for (int k = 0; k < m; k++) w[k] = (1.0 - t) * w_prev_[k] + t * w_target_[k];
        normalize(w, m);   // 단순 lerp 는 노름이 1 이 아니다 — 중간에 딥이 생긴다
        for (int k = 0; k < m; k++)
            ramp_[(size_t)s * m + k] =
                std::complex<float>((float)w[k].real(), (float)w[k].imag());
    }
    ramp_valid_ = true;
}

void Mrc::combine(const std::complex<float>* iq, uint32_t channels,
                  uint32_t samples_per_ch, std::complex<float>* out) {
    const int m = std::min((int)channels, kMaxElements);
    if (m < 2 || !ramp_valid_) {
        // 결합 불가 — ch0 그대로 복사 (호출자가 여기까지 오면 안 되지만 방어).
        for (uint32_t i = 0; i < samples_per_ch; i++) out[i] = iq[i];
        return;
    }

    const size_t n = samples_per_ch;
    const size_t ramp_n = std::min<size_t>(kMrcRampLen, n);
    const size_t seg_len = ramp_n / kMrcRampSeg;

    // 채널 베이스 포인터를 float 로 잡아 둔다. std::complex<float> 는 연속
    // 인터리브 배열임이 규격으로 보장된다.
    const float* __restrict base[kMaxElements];
    for (int k = 0; k < m; k++)
        base[k] = reinterpret_cast<const float*>(iq + (size_t)k * samples_per_ch);
    float* __restrict o = reinterpret_cast<float*>(out);

    // ── 램프 구간 ────────────────────────────────────────────────────────
    size_t i = 0;
    if (seg_len > 0) {
        for (int s = 0; s < kMrcRampSeg; s++) {
            float wr[kMaxElements], wi[kMaxElements];
            for (int k = 0; k < m; k++) {
                wr[k] = ramp_[(size_t)s * m + k].real();
                wi[k] = ramp_[(size_t)s * m + k].imag();
            }
            const size_t end = i + seg_len;
            for (; i < end; i++) {
                float ar = 0.f, ai = 0.f;
                for (int k = 0; k < m; k++) {
                    const float xr = base[k][2 * i], xi = base[k][2 * i + 1];
                    // conj(w) * x
                    ar += wr[k] * xr + wi[k] * xi;
                    ai += wr[k] * xi - wi[k] * xr;
                }
                o[2 * i] = ar; o[2 * i + 1] = ai;
            }
        }
    }

    // ── 정상 구간 (가중치 고정) ──────────────────────────────────────────
    {
        float wr[kMaxElements], wi[kMaxElements];
        for (int k = 0; k < m; k++) {
            wr[k] = (float)w_target_[k].real();
            wi[k] = (float)w_target_[k].imag();
        }
        for (; i < n; i++) {
            float ar = 0.f, ai = 0.f;
            for (int k = 0; k < m; k++) {
                const float xr = base[k][2 * i], xi = base[k][2 * i + 1];
                ar += wr[k] * xr + wi[k] * xi;
                ai += wr[k] * xi - wi[k] * xr;
            }
            o[2 * i] = ar; o[2 * i + 1] = ai;
        }
    }

    // 다음 프레임의 램프 시작점.
    for (int k = 0; k < kMaxElements; k++) w_prev_[k] = w_target_[k];
}

MrcStatus Mrc::status() const {
    MrcStatus s;
    s.enabled  = enabled_;
    s.engaged  = engaged_;
    s.elements = m_;
    s.gamma    = gamma_;
    s.frames   = frames_;
    s.warm     = warm_;
    s.ch0_weak = ch0_weak_;

    // ||w||=1 기준으로 sum|w_m| 은 소자가 균등하고 위상이 맞으면 sqrt(M),
    // 즉 20log10 이 10log10(M) = 어레이 이득이다. 소자 하나가 죽으면 그만큼
    // 정직하게 내려간다 (4소자면 6.02 dB).
    double sum = 0.0;
    for (int k = 0; k < m_; k++) {
        const double a = std::abs(w_target_[k]);
        s.w_mag[k] = a;
        s.w_deg[k] = std::arg(w_target_[k]) * 180.0 / 3.14159265358979323846;
        s.dc_dbfs[k] = (dc_[k] > 1e-30)
                     ? 10.0 * std::log10(dc_[k] / kMrcBlocks) : -999.0;
        sum += a;
    }
    s.gain_db = (sum > 1e-12) ? 20.0 * std::log10(sum) : 0.0;
    return s;
}

} // namespace df
