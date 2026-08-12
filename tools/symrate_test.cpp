// 심볼율 추정 회귀 하네스 (합성 신호, 하드웨어 불필요).
//   g++ -O2 -I src -o /tmp/symtest tools/symrate_test.cpp -lfftw3f && /tmp/symtest
// 실패 개수를 종료코드로 낸다.
//
// CMake 에 넣지 않는다 — symrate.hpp 가 header-only 라 이렇게 직접 컴파일되고,
// 코어 바이너리에 시험 코드를 끌고 들어가지 않는다.
#include "symrate.hpp"
#include <cstdio>
#include <cstdarg>
#include <random>
#include <complex>

static int g_fail = 0, g_run = 0;

static void check(bool ok, const char* fmt, ...) __attribute__((format(printf,2,3)));
static void check(bool ok, const char* fmt, ...){
    g_run++; if(!ok) g_fail++;
    va_list ap; va_start(ap, fmt);
    printf(ok ? "  PASS  " : "  FAIL  ");
    vprintf(fmt, ap); printf("\n");
    va_end(ap);
}

using cf = std::complex<float>;

// 루트 레이즈드 코사인 펄스성형 — 실제 PSK/QAM 은 이걸 거쳐 나오고, 그래야
// 심볼 경계에서 포락선이 패여 |x|^2 에 선이 선다. 사각펄스로 시험하면
// 추정기가 통과해도 실신호에서 듣는다는 보장이 없다.
static std::vector<float> rrc_taps(int sps, int span, double beta){
    int n = sps*span + 1;
    std::vector<float> h((size_t)n);
    for(int i=0;i<n;i++){
        double t = (double)(i - n/2) / (double)sps;
        double v;
        if(std::fabs(t) < 1e-8){
            v = 1.0 - beta + 4.0*beta/M_PI;
        } else if(beta > 0 && std::fabs(std::fabs(4.0*beta*t) - 1.0) < 1e-6){
            v = beta/std::sqrt(2.0) * ((1.0+2.0/M_PI)*std::sin(M_PI/(4.0*beta))
                                     + (1.0-2.0/M_PI)*std::cos(M_PI/(4.0*beta)));
        } else {
            double num = std::sin(M_PI*t*(1.0-beta)) + 4.0*beta*t*std::cos(M_PI*t*(1.0+beta));
            double den = M_PI*t*(1.0 - std::pow(4.0*beta*t,2.0));
            v = num/den;
        }
        h[(size_t)i] = (float)v;
    }
    return h;
}

// 선형변조 버스트 (BPSK/QPSK/QAM16) — 심볼을 sps 배 업샘플 후 RRC 성형.
static std::vector<float> make_linear(int nsym, int sps, int m_order,
                                      double snr_db, uint32_t seed)
{
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> sym(0, m_order-1);
    std::vector<cf> up((size_t)(nsym*sps), cf(0.f,0.f));
    for(int k=0;k<nsym;k++){
        int s = sym(rng);
        cf c;
        if(m_order == 2)      c = cf(s ? 1.f : -1.f, 0.f);
        else if(m_order == 4) c = cf((s&1)?1.f:-1.f, (s&2)?1.f:-1.f);
        else {                                     // 16QAM
            int i = s & 3, q = (s>>2) & 3;
            c = cf((float)(2*i-3), (float)(2*q-3));
        }
        up[(size_t)(k*sps)] = c;
    }
    auto h = rrc_taps(sps, 8, 0.35);
    int n = (int)up.size();
    std::vector<cf> y((size_t)n, cf(0.f,0.f));
    for(int i=0;i<n;i++){
        cf acc(0.f,0.f);
        for(int j=0;j<(int)h.size();j++){
            int idx = i - j;
            if(idx >= 0 && idx < n) acc += up[(size_t)idx] * h[(size_t)j];
        }
        y[(size_t)i] = acc;
    }
    double p = 0; for(auto& v : y) p += std::norm(v); p /= (double)n;
    double npow = p / std::pow(10.0, snr_db/10.0);
    std::normal_distribution<double> gn(0.0, std::sqrt(npow/2.0));
    std::vector<float> out((size_t)(n*2));
    for(int i=0;i<n;i++){
        out[(size_t)(2*i)]   = y[(size_t)i].real() + (float)gn(rng);
        out[(size_t)(2*i+1)] = y[(size_t)i].imag() + (float)gn(rng);
    }
    return out;
}

// 정포락선 CPFSK 버스트 — |x|^2 가 상수라 전이검출기로만 잡힌다.
static std::vector<float> make_fsk(int nsym, int sps, int m_order,
                                   double dev_norm, double snr_db, uint32_t seed)
{
    std::mt19937 rng(seed);
    std::uniform_int_distribution<int> sym(0, m_order-1);
    int n = nsym*sps;
    std::vector<float> out((size_t)(n*2));
    double ph = 0.0;
    std::normal_distribution<double> gn(0.0, std::sqrt(std::pow(10.0,-snr_db/10.0)/2.0));
    for(int k=0;k<nsym;k++){
        int s = sym(rng);
        double lvl = (double)(2*s - (m_order-1));           // 대칭 편이
        double dph = 2.0*M_PI*dev_norm*lvl/(double)sps;
        for(int i=0;i<sps;i++){
            ph += dph;
            int idx = k*sps + i;
            out[(size_t)(2*idx)]   = (float)(std::cos(ph) + gn(rng));
            out[(size_t)(2*idx+1)] = (float)(std::sin(ph) + gn(rng));
        }
    }
    return out;
}

static void t_rate(const char* name, const std::vector<float>& iq, double fs,
                   double want_hz, double tol_pct)
{
    symrate::Result r = symrate::estimate(iq.data(), (int)(iq.size()/2), fs);
    double err = (want_hz > 0) ? std::fabs(r.rate_hz - want_hz)/want_hz*100.0 : 0.0;
    check(r.rate_hz > 0 && err <= tol_pct,
          "%-28s want=%7.1f got=%7.1f Hz  err=%5.2f%%  conf=%.2f",
          name, want_hz, r.rate_hz, err, r.conf);
}

int main(){
    printf("=== symrate selftest ===\n");
    const double FS = 48000.0;

    // 4096 복소표본 = AMC_CAP. 실제 워커가 넘기는 것과 같은 길이로 시험한다.
    const int NCAP = 4096;

    // sps = FS/baud. AMC 는 fs_out ≈ 4×BW 로 잡으므로 sps 는 대략 4 근처다.
    struct LinCase { const char* nm; int m; double baud; double snr; double tol; };
    LinCase lin[] = {
        {"BPSK  2400 baud SNR20",  2, 2400.0, 20.0, 1.0},
        {"QPSK  9600 baud SNR10",  4, 9600.0, 10.0, 1.0},
        {"QPSK  2400 baud SNR6",   4, 2400.0,  6.0, 2.0},
        {"QAM16 4800 baud SNR15", 16, 4800.0, 15.0, 1.0},
    };
    for(auto& c : lin){
        int sps  = (int)std::lround(FS / c.baud);
        int nsym = NCAP/sps + 16;
        auto iq = make_linear(nsym, sps, c.m, c.snr, 1234);
        iq.resize((size_t)NCAP*2);
        t_rate(c.nm, iq, FS, FS/(double)sps, c.tol);
    }

    struct FskCase { const char* nm; int m; double baud; double snr; double tol; };
    FskCase fsk[] = {
        {"2FSK  1200 baud SNR6",  2, 1200.0,  6.0, 2.0},
        {"2FSK  4800 baud SNR15", 2, 4800.0, 15.0, 1.0},
        {"4FSK  2400 baud SNR15", 4, 2400.0, 15.0, 1.0},
    };
    for(auto& c : fsk){
        int sps  = (int)std::lround(FS / c.baud);
        int nsym = NCAP/sps + 16;
        auto iq = make_fsk(nsym, sps, c.m, 0.35, c.snr, 777);
        iq.resize((size_t)NCAP*2);
        t_rate(c.nm, iq, FS, FS/(double)sps, c.tol);
    }

    // 미검출이어야 하는 것들 — 오검출이 실제 운용에서 더 해롭다.
    {
        std::mt19937 rng(999);
        std::normal_distribution<double> g(0.0,1.0);
        std::vector<float> noise((size_t)NCAP*2);
        for(auto& v : noise) v = (float)g(rng);
        symrate::Result r = symrate::estimate(noise.data(), NCAP, FS);
        check(r.rate_hz == 0.f, "white noise only            got=%.1f Hz (want 0)", r.rate_hz);
    }
    {
        // FM 음성 유사 — 심볼 경계가 없는 연속 변조. 선이 서면 안 된다.
        std::vector<float> iq((size_t)NCAP*2);
        std::mt19937 rng(555);
        std::normal_distribution<double> g(0.0, 0.02);
        double ph = 0.0, mph = 0.0;
        for(int i=0;i<NCAP;i++){
            mph += 2.0*M_PI*700.0/FS;                       // 700 Hz 톤 변조
            ph  += 2.0*M_PI*3000.0*std::sin(mph)/FS;
            iq[(size_t)(2*i)]   = (float)(std::cos(ph) + g(rng));
            iq[(size_t)(2*i+1)] = (float)(std::sin(ph) + g(rng));
        }
        symrate::Result r = symrate::estimate(iq.data(), NCAP, FS);
        check(r.rate_hz == 0.f, "FM tone (no symbols)        got=%.1f Hz (want 0)", r.rate_hz);
    }
    {
        std::vector<float> tiny(64, 0.f);
        symrate::Result r = symrate::estimate(tiny.data(), 32, FS);
        check(r.rate_hz == 0.f, "too short input             got=%.1f Hz (want 0)", r.rate_hz);
        r = symrate::estimate(nullptr, 4096, FS);
        check(r.rate_hz == 0.f, "null input                  got=%.1f Hz (want 0)", r.rate_hz);
    }

    printf("=== %d/%d passed ===\n", g_run-g_fail, g_run);
    return g_fail;
}
