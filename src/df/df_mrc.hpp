#pragma once
// ── MRC (Maximal Ratio Combining) ────────────────────────────────────────
//
// Kraken 5채널을 하나의 IQ 스트림으로 합쳐 링에 넣는다. 동글 5개가 각각 별개
// 칩이라 **수신기 열잡음이 서로 독립**이다. 위상을 맞춰 더하면 신호는 진폭
// M 배로, 잡음은 sqrt(M) 배로 커지므로 SNR 이 10log10(M) = 5소자에서 6.99 dB
// 오른다. 이 이득은 소자 간격과 무관하다 (페이딩 다이버시티와 다른 물건이다 —
// 그쪽은 간격 >= lambda/2 를 요구하지만, 어레이 이득은 위상 정합만 있으면 된다).
//
// 왜 DDC 앞에서 합치나
// ----------------------------------------------------------------------
// 배열 개구가 0.35 m 라 소자 간 최대 지연이 1.17 ns 다. 그 지연이 대역 끝에서
// 위상을 18도 틀어놓는 대역폭이 42.9 MHz 인데 실제 채널은 5~200 kHz 다. 즉
// 협대역 가정이 압도적으로 성립하고, 가중치를 **주파수 무관 스칼라**로 둘 수
// 있다. 그래서 풀레이트에서 5->1 로 먼저 합치면 그 뒤 DDC·복조·녹음·디코더
// 경로가 통째로 무수정이고, 운용 채널이 몇 개든 DDC 비용이 늘지 않는다.
//
// 가중치를 고유분해로 구하지 않는 이유
// ----------------------------------------------------------------------
// 주고유벡터가 교과서 답이지만(그리고 df_estimator 가 이미 그걸 계산한다),
// 여기선 ch0 기준 상관/전력 비가 더 낫다:
//
//     w_m = c_m / p_m,   c_m = E[x_m conj(x_0)],  p_m = E|x_m|^2
//
// R = P a a^H + diag(sigma^2_m) 이면 c_m = P a_m conj(a_0), p_m = P|a_m|^2 +
// sigma^2_m 이므로 w_m ∝ a_m conj(a_0)/(P|a_m|^2 + sigma^2_m) — 저SNR 극한에서
// a_m/sigma^2_m, 즉 소자별 잡음이 다를 때의 **최적 MRC 가중치 그 자체**다.
// 덤으로 w_0 = p_0/p_0 = 1 이 정규화 전에 항상 실수 양수로 나온다. 위상 기준을
// 따로 정할 필요가 없다는 뜻이고, 이건 편의가 아니라 **연속성의 근거**다:
// 결합 출력이 ch0 과 같은 반송파 위상을 갖게 되어 MRC on/off 가 위상 연속이고
// 하류(FM 판별기·AM 포락선·상관기·녹음)가 오늘과 같은 기준을 본다.
//
// 무신호일 때가 이 선택의 진짜 이득이다. 잡음만 있으면 c_m 은 평균 0 이라
// |w_m| ~ 1/sqrt(N) 로 줄고, EMA 를 거치면 -47 dB 다. **임계값 없이 w -> e_0 로
// 수렴한다** — 출력이 ch0 과 0.0001 dB 내로 같아진다. 고유벡터였다면 잡음 하에서
// 스펙트럼이 거의 축퇴라 주고유벡터가 매 프레임 임의 방향으로 튀었을 것이다.
//
// 정규화
// ----------------------------------------------------------------------
// ||w||_2 = 1 을 엄격히 지킨다. 그러면 E|w^H nu|^2 = sigma^2 로 **잡음 전력이
// ch0 과 동일**하다. 워터폴 노이즈플로어·autoscale 분위수·HIST db_min·스퀠치
// 캘리브가 전부 그대로 유효하고, 신호만 올라간다. (V2 의 DAS 는 /N 을 안 해서
// 출력이 N 배로 뜨는데, BEWE 는 이 스트림이 워터폴과 int16 양자화로 가므로
// 그 방식을 쓸 수 없다.) 클리핑 상한은 Cauchy-Schwarz 로 |y| <= sqrt(M) = 2.236,
// x2048 = 4580 vs int16 32767 이라 16 dB 여유가 남는다.

#include "df_types.hpp"
#include <complex>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace df {

// 프레임당 통계 표본. 64 블록 x 128 샘플 = 8192 snapshot.
// 풀레이트(1M 샘플) 대비 227배 싸고, 정확도 손실은 0.0013 dB 다 — 영상관 상관은
// 표본 인덱스에 대한 평균이라 부분집합을 써도 불편추정이고, 늘어나는 건 분산뿐이다.
inline constexpr int    kMrcBlocks   = 64;
inline constexpr int    kMrcBlockLen = 128;
inline constexpr int    kMrcSnapshots = kMrcBlocks * kMrcBlockLen;

// 통계량(가중치가 아니라)의 지수평균. tau = 1/(alpha*2.29Hz) ~ 2.9 s.
// c_m 은 위상자라 이걸 평균해야 wrap 문제가 없다. 가중치를 직접 평균하면
// 반대 위상 두 추정이 상쇄돼 0 이 될 수 있다.
inline constexpr float  kMrcAlpha = 0.15f;

// 프레임 경계 크로스페이드. 가중치는 프레임 경계에서만 바뀌므로 그 지점을
// 미끄러뜨린다. 세그먼트마다 **재정규화**해야 한다 — 단위벡터 둘 사이의 단순
// lerp 는 ||w|| 이 1 이 아니라 중간에 최대 -4.4 dB 딥이 생긴다.
inline constexpr size_t kMrcRampLen = 65536;   // 27.3 ms @ 2.4 MSPS
inline constexpr int    kMrcRampSeg = 256;     // 세그먼트당 256 샘플

// 결합 개시/해제 판정용 평균 크기제곱 코히런스 (ch0 대비).
// 잡음만이면 gamma ~ 1.2e-4 이므로 개시 임계와 400배 떨어져 있다.
// gamma = rho^2/(1+rho)^2 -> on 은 소자당 SNR -5.4 dB, off 는 -7.8 dB.
inline constexpr double kMrcGammaOn  = 0.05;
inline constexpr double kMrcGammaOff = 0.02;
inline constexpr int    kMrcDwell    = 3;   // 연속 프레임 수 (~1.3 s)
inline constexpr int    kMrcWarm     = 8;   // reset 후 결합 개시까지 관측할 프레임

// UI/CLI 표시용 스냅샷.
struct MrcStatus {
    bool     enabled   = false;   // 운용자가 켰는가
    bool     engaged   = false;   // 실제로 결합 중인가
    int      elements  = 0;
    double   gamma     = 0.0;     // 평균 코히런스 (0..1)
    double   gain_db   = 0.0;     // 20log10(sum|w_m|), ||w||=1 기준. 최대 6.99
    uint64_t frames    = 0;       // reset 후 관측 프레임
    int      warm      = 0;       // 남은 워밍업 프레임 (0 이면 준비됨)
    bool     ch0_weak  = false;   // ch0 전력이 붕괴 (하드웨어 고장)
    double   w_mag[kMaxElements] = {};   // |w_m|
    double   w_deg[kMaxElements] = {};   // arg(w_m), 도
    double   dc_dbfs[kMaxElements] = {}; // 채널별 DC 누설 (병리 감시용)
};

class Mrc {
public:
    // 통계·가중치·워밍업을 전부 버린다. 재튠·샘플레이트 변경·채널수 변경·
    // DAQ 재캘리브 복귀·엔진 재시작에서 반드시 부를 것 — 옛 가중치는 그때
    // 물리적으로 틀린 값이 된다.
    void reset();

    void set_enabled(bool on);
    bool enabled() const { return enabled_; }

    // 프레임 하나의 통계를 누적한다 (~0.1 ms). **결합 여부와 무관하게** 매
    // 프레임 부른다 — 그래야 /mrc on 이 이미 수렴한 가중치로 즉시 붙는다.
    // usable=false 인 프레임은 heimdall 이 채널 간 보정을 아직 안 건 상태라
    // 위상이 틀리므로 통계에 넣지 않는다.
    void observe(const std::complex<float>* iq, uint32_t channels,
                 uint32_t samples_per_ch, uint64_t center_hz, uint64_t fs_hz,
                 bool usable);

    // 결합해야 하는가 (enabled + 워밍업 완료 + 코히런스 충족 + 건전성).
    bool engaged() const { return engaged_; }

    // out[i] = sum_m conj(w_m) x_m[i]. out 은 samples_per_ch 개를 담을 수 있어야
    // 한다. 호출자 스레드에서 동기 실행되며 내부 상태만 만지고 입력은 const 다.
    void combine(const std::complex<float>* iq, uint32_t channels,
                 uint32_t samples_per_ch, std::complex<float>* out);

    MrcStatus status() const;

private:
    void solve_weights_();   // c,p -> w_target_ (정규화까지)
    void build_ramp_();      // w_prev_ -> w_target_ 세그먼트 테이블

    bool     enabled_ = false;
    bool     engaged_ = false;

    int      m_ = 0;                 // 현재 채널 수
    uint64_t center_hz_ = 0, fs_hz_ = 0;
    bool     prev_usable_ = false;

    // 누적 통계 (EMA). c_[0] 는 정의상 p_[0] 라 따로 두지 않는다.
    std::complex<double> c_[kMaxElements] = {};
    double               p_[kMaxElements] = {};
    double               dc_[kMaxElements] = {};   // 블록평균 크기의 EMA (감시용)
    bool     stats_init_ = false;

    std::complex<double> w_target_[kMaxElements] = {};
    std::complex<double> w_prev_[kMaxElements]   = {};

    // 램프 테이블: [세그먼트][소자]. 각 세그먼트가 재정규화된 가중치를 담는다.
    std::vector<std::complex<float>> ramp_;
    bool     ramp_valid_ = false;

    uint64_t frames_ = 0;
    int      warm_ = kMrcWarm;
    int      dwell_on_ = 0, dwell_off_ = 0;
    double   gamma_ = 0.0;
    bool     ch0_weak_ = false;
};

} // namespace df
