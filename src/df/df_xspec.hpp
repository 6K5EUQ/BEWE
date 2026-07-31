#pragma once
// ── Welch 교차스펙트럼 누산기 ─────────────────────────────────────────────
//
// 왜 시간영역 채널라이저를 안 쓰나
// ----------------------------------------------------------------------
// DOA 에 필요한 건 R 뿐이고, **모든 채널에 공통으로 곱해지는 스칼라는 R 을
// 바꾸지 않는다**:
//     x'[n] = e^{j phi[n]} x[n]  =>  x' x'^H = e^{j phi} x x^H e^{-j phi} = x x^H
// NCO 하향변환이 바로 그런 스칼라다. 즉 주파수 이동은 R 에 대해 no-op 이고,
// 데시메이션도 (기댓값상) 샘플 수만 줄인다. 실제로 의미 있는 건 대역 선택뿐.
//
// 그래서 채널당 5개씩 NCO+FIR 을 돌리는 대신, 세그먼트 FFT 를 떠서 채널
// 대역에 드는 빈만 골라 외적을 누산한다. 결과는 같고 (수치 대조 완료),
// 프레임당 1,048,576 x 5 샘플을 필터링하지 않아도 된다.
//
//     X_m^s[k] = FFT_K{ w[n] * y_m[s*hop + n] }[k]
//     B = { k : ch_lo <= f(k) <= ch_hi
//              & |f_b(k)| > dc_guard        (LO 누설 배제)
//              & |f_b(k)| <= 0.45 fs }      (RTL 자체 롤오프 회피)
//     R += sum_s sum_{k in B} X^s[k] X^s[k]^H
//
// 비용은 채널 대역폭과 거의 무관하다. 넓은 채널은 빈이 많아 세그먼트가 적게
// 필요하고, 좁은 채널은 그 반대다 — target_looks 로 예산을 잡는다.

#include "df_types.hpp"
#include <complex>
#include <cstddef>
#include <vector>

namespace df {

class XSpec {
public:
    struct Params {
        double ch_center_hz  = 0;
        double ch_bw_hz      = 0;
        double daq_center_hz = 0;
        double daq_fs_hz     = 0;
        int    elements      = 5;
        double dc_guard_hz   = 2000.0;
        int    target_looks  = 2048;
        int    fft_size      = 8192;   // 시작값. 채널이 좁으면 엔진이 키운다
    };

    ~XSpec();
    XSpec() = default;
    XSpec(const XSpec&) = delete;
    XSpec& operator=(const XSpec&) = delete;

    // 빈 선택까지 계산한다. 실패하면 why 에 이유가 담긴다.
    bool prepare(const Params& p, Status& why);
    void reset();   // R 과 카운터만 초기화 (플랜·버퍼는 유지)

    // 프레임 하나의 기여분을 누산한다. iq 는 channel-major.
    // 세그먼트는 target_looks 를 채울 만큼만 쓴다.
    bool add_frame(const std::complex<float>* iq, size_t samples_per_ch);

    // 정규화된 R 을 out(M*M) 에 쓴다. Re(tr R)/M 이 페이로드 원단위
    // (full scale = 1.0)의 대역 내 평균전력이 되도록 맞춘다 — 그래야
    // power_dbfs 가 의미를 갖는다. PAPR·고유값비는 스케일 무관이라 영향 없다.
    bool snapshot_R(std::complex<double>* out) const;
    int    elements()  const { return m_; }
    int    bins()      const { return (int)bins_.size(); }
    int    fft_size()  const { return k_; }
    int    frames()    const { return frames_; }
    double n_eff()     const;
    double effective_bw_hz() const;

private:
    void destroy_plan();

    std::vector<std::complex<double>> R_;
    std::vector<int>                  bins_;      // 채널에 드는 FFT 빈 인덱스
    std::vector<float>                win_;       // Hann
    Params p_{};
    int    m_ = 0, k_ = 0;
    int    seg_needed_ = 0;      // 프레임당 쓸 세그먼트 수
    int    frames_ = 0;
    long   segs_done_ = 0;
    double win_pow_ = 0.0;       // sum w[n]^2

    void*  fft_in_  = nullptr;   // fftwf_complex[k_]
    void*  fft_out_ = nullptr;   // fftwf_complex[k_ * m_]  (채널별 결과 보관)
    void*  plan_    = nullptr;   // fftwf_plan
};

} // namespace df
