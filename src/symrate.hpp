#pragma once
// ── 심볼율(전송속도) 자동 추정 ────────────────────────────────────────────────
// 버스트 하나의 복소 기저대역 표본에서 심볼율을 직접 계산한다. 운용자가 커서를
// 두 번 찍어 읽던 값(ui.cpp 의 eid_baud_*)을 자동으로 낸다.
//
// 의존은 FFTW 하나뿐이다 — 코어의 bewe_fft_plan() 을 쓰지 않는다. 그쪽은
// BEWEPaths·bewe_log_push 를 끌어와서 이 헤더만 떼어 컴파일할 수 없게 되고,
// 그러면 tools/ 의 회귀 하네스가 앱 전체를 링크해야 한다. 버스트당 4096점 FFT
// 몇 번이라 wisdom 학습으로 얻을 것도 없다.
//
// ── 변환 둘, 그리고 어느 쪽을 믿을지 ───────────────────────────────────────
// 심볼율은 원신호 스펙트럼에 선으로 안 나온다. 비선형 변환을 거쳐야 심볼 주기가
// 스펙트럼선으로 접히는데, 어떤 변환이 듣는지가 변조에 따라 갈린다.
//
//   A |x|²              진폭이 심볼마다 변하는 것에 반응. 펄스성형된 PSK/QAM 은
//                       심볼 경계에서 포락선이 패이므로 1/T 에 선이 선다.
//                       **정포락선(FSK/GMSK/MSK)에는 완전히 무력하다** — |x| 가 상수다.
//   C g_L(φ')²          순시주파수 φ' = arg(x[n]·conj(x[n-1])) 에 **계단 정합필터**를
//                       걸고 제곱. g_L[n] = (직전 L개 평균) - (직후 L개 평균) 이라
//                       심볼 경계에서만 튀어 임펄스열이 된다. 정포락선을 잡는 건 이것뿐.
//
// 그래서 판정 순서가 물리로 정해진다: **A 가 서면 진폭이 변한 것 = 선형변조**이고
// 그 추정을 쓴다. A 가 침묵하면 정포락선이므로 C 로 넘어간다.
//
// ── C 의 L 을 여러 개 시도하는 이유 ────────────────────────────────────────
// L=1 (그냥 1차 차분) 로는 FSK 가 안 잡힌다. 전이에서의 위상 도약은
// 2·h·2π/sps 라 **과표본이 커질수록 작아지는데 위상잡음은 표본당 일정**하다.
// 1200 baud @48 kHz(sps=40, h=0.35) 에서 도약이 0.11 rad 인데 SNR 6 dB 의 위상잡음이
// 0.35 rad 라 통째로 묻힌다 (회귀에서 참 위치 ratio 가 4.1, 배경과 구분 불가였다).
// L 개를 평균내면 잡음이 √L 로 줄고 계단은 그대로 남는다. 심볼율을 모르는 채로
// 최적 L 을 고를 수 없으므로 몇 개를 돌려 **가장 또렷한 것**을 택한다 — L 이 심볼폭의
// 절반을 넘으면 이웃 전이끼리 상쇄돼 스스로 탈락하므로, 이 선택이 곧 정합이다.
// 실측(x86, 4096점): 최악(A 실패 + L 6개 전부 시도)이 1.5 ms/호출 = 채널당 0.15 %.
// 신호가 잡히면 A 에서 끝나 1/7 수준이다.
//
// ── 배음 요구는 C 에만 건다 (여기서 한 번 틀렸다) ──────────────────────────
// 임펄스열의 스펙트럼은 2배·3배에도 선이 서는 빗살이지만, **선형변조의 순환
// 스펙트럼은 1/T 에만 선이 서고 배음이 없다** (초과대역 α<1). 그래서 배음을
// 전 변환에 요구하면 정작 A 가 맞게 잡은 PSK/QAM 을 전부 버린다 (회귀에서
// QPSK·QAM16 이 통째로 미검출로 떨어졌다).
//
// C 에만 거는 이유는 따로 있다. 정현적으로 변조된 연속 신호(FM 음성)도 C 를
// 흔들지만 그건 선이 하나뿐인 순정현이다. 배음 유무가 "심볼 전이인가, 그냥
// 주기적 변조인가"를 가른다. 이 조건 없이는 700 Hz 톤 FM 이 1400 Hz 심볼율로 잡혔다.
//
// ── L 간 일치를 요구한다 (잡음 배제의 핵심) ────────────────────────────────
// ratio 임계만으로는 잡음을 못 막는다. L 을 6개 돌리며 2040개 bin 에서 최대를
// 고르므로 시행 횟수가 커서 흰잡음도 ratio 4.7 까지 올라오고, 배음 조건마저
// 우연히 통과한다(실측: 흰잡음 L=32 가 ratio 4.74 + harm=1).
// 결정적 차이는 **위치의 일관성**이다. 진짜 심볼율선은 여러 L 이 같은 bin 을
// 짚지만(2FSK 1200: L=16·L=32 가 둘 다 1195.3 Hz), 잡음 피크는 L 마다 딴 데로
// 튄다(16207 / 4066 / 5543 / 14695 / 18070 / 13722 Hz). 그래서 다른 L 이
// ±AGREE_BINS 안에서 거들어 주지 않는 후보는 버린다.
//
// ── 되짚기(1/2·1/3)는 넣지 않는다 ─────────────────────────────────────────
// "최대 피크가 배음 자리에 잡히면 심볼율이 배로 보고된다"는 걱정으로 1/2 자리를
// 되짚는 보정을 넣었다가 뺐다. 두 변환 모두 **기본파가 실제로 가장 강했고**
// (회귀 실측: A 1x=62.5 vs 2x=1.6, C 1x=13.1 vs 2x=8.7), 오히려 그 보정이 실재하는
// 약한 1/2 선(BPSK 21.3)에 걸려 맞는 검출을 절반으로 깎았다. 근거 없는 보정이
// 정상 동작을 깨는 쪽이 실제 위험이라 제거했다.
#include <cmath>
#include <cstddef>
#include <mutex>
#include <vector>
#include <algorithm>
#include <fftw3.h>

namespace symrate {

struct Result {
    float rate_hz = 0.f;   // 추정 심볼율. 0 = 미검출
    float conf    = 0.f;   // 0..1. 선 대 배경비를 사상한 값
};

namespace detail {

// 검출 판정 임계 — 피크가 **국소** 배경의 몇 배(진폭비)여야 "선"으로 보나.
// 합성 회귀 실측(4096점, 국소배경 기준): 흰잡음이 내는 최대 ratio 가 A 3.56 /
// C 3.2~4.74 이고, 참신호 최약체가 QPSK 2400 SNR6 의 5.99 다. 그 사이에 둔다.
// 전 대역 중앙값을 쓰던 시절의 8.0 을 그대로 두면 참신호가 줄줄이 탈락한다.
constexpr double PEAK_RATIO_MIN  = 5.5;
// conf = 1.0 이 되는 비. 이 위는 포화시킨다.
constexpr double PEAK_RATIO_FULL = 20.0;
// 배음으로 인정하는 최소 진폭비. 기본파보다 훨씬 약해도 되지만 배경보다는 확실히 위여야.
constexpr double HARM_RATIO_MIN  = 4.0;
// 심볼율 하한을 bin 수로 — 이보다 낮으면 창 길이가 모자라 못 믿는다.
constexpr int MIN_BINS = 6;
// 배경 추정 창의 반폭(bin). 심볼율선은 창 하나 폭이라 이만큼 넓게 잡아도 중앙값이
// 안 끌려가고, 반대로 변환이 만든 넓은 언덕은 통째로 배경에 흡수된다.
// 전 대역 중앙값을 쓰면 그 언덕이 선으로 오인된다 — 계단 정합필터가 잡음을
// 대역통과로 색칠하는 바람에 **흰잡음만 넣었는데 450 Hz 가 검출됐다.**
constexpr int BG_HALF = 48;
// 배경을 실제로 계산하는 bin 간격. 사이는 선형보간한다 (best_line 주석 참조).
constexpr int BG_STEP = 16;
// L 간 일치로 인정하는 bin 오차. 실측에서 참신호는 여러 L 이 같은 bin(±1)을 짚었다.
constexpr double AGREE_BINS = 2.0;

// FFTW 플래너는 스레드 안전하지 않다. 실행(execute)은 안전하므로 계획 수립만 잠근다.
inline std::mutex& plan_mtx(){ static std::mutex m; return m; }

// r2c 플랜 + 버퍼. 채널 워커가 여러 개 도는데 매번 계획을 세우면 그 뮤텍스가
// 직렬화 지점이 되므로 스레드마다 캐시한다.
struct Plan {
    int n = 0;
    float* in = nullptr;
    fftwf_complex* out = nullptr;
    fftwf_plan p = nullptr;
    ~Plan(){
        if(p){ std::lock_guard<std::mutex> lk(plan_mtx()); fftwf_destroy_plan(p); }
        if(in)  fftwf_free(in);
        if(out) fftwf_free(out);
    }
    bool ensure(int nn){
        if(n == nn && p) return true;
        if(p){ std::lock_guard<std::mutex> lk(plan_mtx()); fftwf_destroy_plan(p); p=nullptr; }
        if(in){ fftwf_free(in); in=nullptr; }
        if(out){ fftwf_free(out); out=nullptr; }
        n = 0;
        float* i2 = fftwf_alloc_real((size_t)nn);
        fftwf_complex* o2 = fftwf_alloc_complex((size_t)(nn/2 + 1));
        if(!i2 || !o2){ if(i2) fftwf_free(i2); if(o2) fftwf_free(o2); return false; }
        fftwf_plan p2;
        {
            std::lock_guard<std::mutex> lk(plan_mtx());
            p2 = fftwf_plan_dft_r2c_1d(nn, i2, o2, FFTW_ESTIMATE);
        }
        if(!p2){ fftwf_free(i2); fftwf_free(o2); return false; }
        in = i2; out = o2; p = p2; n = nn;
        return true;
    }
    // bin 의 전력 (없는 bin 은 0)
    double pw(int b) const {
        if(!out || b < 0 || b > n/2) return 0.0;
        double re = out[b][0], im = out[b][1];
        return re*re + im*im;
    }
    // bin 근방 최대 전력 — 보간 오차만큼 이웃까지 본다
    double pw_near(int b) const {
        double m = 0.0;
        for(int k=b-1;k<=b+1;k++) m = std::max(m, pw(k));
        return m;
    }
    // bin b 주변의 배경 전력 (국소 중앙값). 아래 BG_HALF 주석 참조.
    double local_med(int b, int lo, int hi, int W, std::vector<double>& scratch) const {
        int a = std::max(lo, b-W), z = std::min(hi, b+W);
        if(z <= a) return 0.0;
        scratch.clear();
        scratch.reserve((size_t)(z-a+1));
        for(int k=a;k<=z;k++) scratch.push_back(pw(k));
        std::nth_element(scratch.begin(), scratch.begin()+(ptrdiff_t)(scratch.size()/2),
                         scratch.end());
        return scratch[scratch.size()/2];
    }
};

struct Cand {
    double bin   = -1.0;   // 소수 bin (미검출 -1)
    double ratio = 0.0;    // 피크/배경 진폭비
    double med   = 0.0;    // 배경 전력 중앙값
};

// 한 실수열의 스펙트럼에서 가장 또렷한 선을 찾는다.
inline Cand best_line(Plan& pl, const std::vector<float>& sig, int bin_lo, int bin_hi)
{
    Cand c;
    const int n = (int)sig.size();
    if(!pl.ensure(n)) return c;

    // 평균 제거 — 안 하면 DC 가 창 누설로 저역 bin 을 덮어 심볼율선을 가린다.
    double mean = 0.0;
    for(int i=0;i<n;i++) mean += sig[i];
    mean /= (double)n;

    // Hann 창. 강한 선 하나를 찾는 문제라 누설 억제가 분해능보다 중요하다.
    for(int i=0;i<n;i++){
        double w = 0.5 - 0.5*std::cos(2.0*M_PI*(double)i/(double)(n-1));
        pl.in[i] = (float)((sig[i]-mean)*w);
    }
    fftwf_execute(pl.p);

    const int nb = n/2 + 1;
    if(bin_lo < 1) bin_lo = 1;
    if(bin_hi > nb-2) bin_hi = nb-2;
    if(bin_hi - bin_lo < 8) return c;

    const int nbin = bin_hi - bin_lo + 1;
    std::vector<double> mag((size_t)nbin);
    for(int b=bin_lo;b<=bin_hi;b++) mag[(size_t)(b-bin_lo)] = pl.pw(b);

    // 배경은 **국소** 중앙값이다 (BG_HALF 주석 참조). 전 대역 중앙값을 쓰면
    // 넓은 언덕이 선으로 오인된다.
    //
    // 단 bin 마다 중앙값을 새로 구하면 안 된다 — 2040 bin × 97개 nth_element 를
    // 변환 7개에 대해 돌려 4096점 1회가 **14.8 ms** 였다. Pi5 기지에서 채널당
    // 10%를 넘는다. 배경은 정의상 천천히 변하므로 성긴 격자에서만 구하고 사이는
    // 선형보간한다 (BG_STEP).
    std::vector<double> scratch, grid;
    grid.reserve((size_t)(nbin/BG_STEP + 2));
    for(int b=bin_lo; ; b += BG_STEP){
        int bb = std::min(b, bin_hi);
        grid.push_back(pl.local_med(bb, bin_lo, bin_hi, BG_HALF, scratch));
        if(bb >= bin_hi) break;
    }
    auto bg_at = [&](int b)->double{
        double t = (double)(b - bin_lo) / (double)BG_STEP;
        size_t i = (size_t)t;
        if(i + 1 >= grid.size()) return grid.back();
        double f = t - (double)i;
        return grid[i]*(1.0-f) + grid[i+1]*f;
    };

    int peak = -1; double peak_score = 0.0, peak_med = 0.0;
    for(int b=bin_lo;b<=bin_hi;b++){
        double bg = bg_at(b);
        if(!(bg > 0.0)) continue;
        double s = mag[(size_t)(b-bin_lo)] / bg;
        if(s > peak_score){ peak_score = s; peak = b; peak_med = bg; }
    }
    if(peak < bin_lo+1 || peak > bin_hi-1) return c;

    // med·ratio 는 미검출이어도 채운다 — 안 그러면 "왜 안 잡혔나"를 밖에서 볼 수
    // 없다. bin 이 음수인 것만이 미검출 신호다.
    c.med   = peak_med;
    c.ratio = std::sqrt(peak_score);                 // 전력비 → 진폭비
    if(c.ratio < PEAK_RATIO_MIN) return c;

    // 포물선 보간 (로그 영역) — bin 이하 정밀도. 4096점에서 bin 간격이
    // fs/4096 이라 보간 없이는 48 kHz 기준 ±6 Hz 가 그냥 오차로 남는다.
    double lm = std::log(mag[(size_t)(peak-bin_lo-1)] + 1e-30);
    double lc = std::log(mag[(size_t)(peak-bin_lo)]   + 1e-30);
    double lp = std::log(mag[(size_t)(peak-bin_lo+1)] + 1e-30);
    double den = (lm - 2.0*lc + lp);
    double d = (std::fabs(den) > 1e-12) ? 0.5*(lm - lp)/den : 0.0;
    if(d < -0.5) d = -0.5; else if(d > 0.5) d = 0.5;

    c.bin = (double)peak + d;
    return c;
}

// bin b 를 기본파로 봤을 때 배음(2b, 3b)이 서 있나.
// 2b 가 탐색대역 밖이면 확인할 방법이 없으므로 통과시킨다 (Nyquist 근처 심볼율).
inline bool has_harmonic(const Plan& pl, double bin, int bin_lo, int bin_hi)
{
    std::vector<double> scratch;
    bool checkable = false;
    for(int k=2;k<=3;k++){
        int hb = (int)std::lround(bin*(double)k);
        if(hb > bin_hi || hb > pl.n/2) continue;
        checkable = true;
        double bg = pl.local_med(hb, bin_lo, bin_hi, BG_HALF, scratch);
        if(bg > 0.0 && std::sqrt(pl.pw_near(hb)/bg) >= HARM_RATIO_MIN) return true;
    }
    return !checkable;      // 확인 불가능하면 반증도 없다
}

} // namespace detail

// iq : 인터리브 복소 float (I,Q,I,Q,…)
// n  : 복소 표본 수
// fs : 표본율 Hz
//
// 전제: 디지털 변조 버스트 하나. AM/FM/OFDM 으로 판정된 채널은 호출자가 거른다
// (amc_decode.cpp) — 연속 변조는 심볼 경계 자체가 없어 심볼율이 정의되지 않는다.
inline Result estimate(const float* iq, int n, double fs)
{
    Result r;
    if(!iq || n < 256 || !(fs > 0.0)) return r;

    // 탐색 대역. 하한은 창 길이가 감당하는 최저(MIN_BINS), 상한은 r2c Nyquist
    // 바로 아래. 상한을 좁혀 잡으면 과표본이 덜 된 채널에서 진짜 선을 잘라 버린다.
    const int bin_lo = detail::MIN_BINS;
    const int bin_hi = n/2 - 2;

    // ── A: |x|² ────────────────────────────────────────────────────────────
    std::vector<float> buf((size_t)n);
    for(int i=0;i<n;i++){
        float I = iq[2*i], Q = iq[2*i+1];
        buf[(size_t)i] = I*I + Q*Q;
    }
    static thread_local detail::Plan pl;
    detail::Cand cd = detail::best_line(pl, buf, bin_lo, bin_hi);

    if(cd.bin < 0.0){
        // ── C: 정포락선 — 순시주파수에 계단 정합필터 ────────────────────────
        std::vector<double> phi((size_t)n, 0.0);
        {
            float pi = iq[0], pq = iq[1];
            for(int i=0;i<n;i++){
                float I = iq[2*i], Q = iq[2*i+1];
                phi[(size_t)i] = std::atan2((double)(Q*pi - I*pq),
                                            (double)(I*pi + Q*pq));
                pi = I; pq = Q;
            }
            phi[0] = phi[1];   // 첫 표본은 이전 값이 없다
        }
        // 누적합으로 임의 폭 평균을 O(1) 에 낸다
        std::vector<double> ps((size_t)n + 1, 0.0);
        for(int i=0;i<n;i++) ps[(size_t)i+1] = ps[(size_t)i] + phi[(size_t)i];
        auto mean_of = [&](int a, int b)->double{      // [a,b)
            if(a < 0) a = 0; if(b > n) b = n;
            if(b <= a) return 0.0;
            return (ps[(size_t)b] - ps[(size_t)a]) / (double)(b - a);
        };

        struct LCand { double bin; double ratio; bool harm; };
        std::vector<LCand> cands;
        for(int L : {1, 2, 4, 8, 16, 32}){
            if(L*4 >= n) break;
            for(int i=0;i<n;i++){
                double g = mean_of(i-L, i) - mean_of(i, i+L);
                buf[(size_t)i] = (float)(g*g);
            }
            detail::Cand c2 = detail::best_line(pl, buf, bin_lo, bin_hi);
            if(c2.bin < 0.0) continue;
            // 배음 판정은 반드시 여기서 — 다음 L 이 pl.out 을 덮어쓴다.
            cands.push_back({ c2.bin, c2.ratio,
                              detail::has_harmonic(pl, c2.bin, bin_lo, bin_hi) });
        }

        // 배음이 있고(순정현 배제) 다른 L 이 같은 자리를 짚은 것 중 가장 또렷한 것.
        double best_ratio = 0.0, best_bin = -1.0;
        for(size_t i=0;i<cands.size();i++){
            if(!cands[i].harm) continue;
            int agree = 0;
            for(size_t j=0;j<cands.size();j++)
                if(j != i && std::fabs(cands[j].bin - cands[i].bin) <= detail::AGREE_BINS)
                    agree++;
            if(agree < 1) continue;
            if(cands[i].ratio > best_ratio){
                best_ratio = cands[i].ratio; best_bin = cands[i].bin;
            }
        }
        if(best_bin < 0.0) return r;
        cd.bin = best_bin; cd.ratio = best_ratio;
    }

    r.rate_hz = (float)(cd.bin * fs / (double)n);
    double c = (cd.ratio - detail::PEAK_RATIO_MIN)
             / (detail::PEAK_RATIO_FULL - detail::PEAK_RATIO_MIN);
    r.conf = (float)(c < 0.0 ? 0.0 : (c > 1.0 ? 1.0 : c));
    return r;
}

} // namespace symrate
