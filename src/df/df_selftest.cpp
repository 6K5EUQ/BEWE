#include "df_selftest.hpp"
#include "df_linalg.hpp"
#include "df_manifold.hpp"
#include "df_estimator.hpp"
#include "heimdall_header.hpp"
#include "df_xspec.hpp"

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <vector>
#include <complex>
#include <random>
#include <algorithm>

namespace df {

using cd = std::complex<double>;

namespace {

constexpr double kPi = 3.14159265358979323846;
int g_fail = 0, g_run = 0, g_verb = 0;

void check(bool ok, const char* fmt, ...) __attribute__((format(printf,2,3)));
void check(bool ok, const char* fmt, ...){
    g_run++;
    if(!ok) g_fail++;
    if(!ok || g_verb > 0){
        va_list ap; va_start(ap, fmt);
        printf(ok ? "  PASS  " : "  FAIL  ");
        vprintf(fmt, ap);
        printf("\n");
        va_end(ap);
    }
}

double ang_err(double a, double b){
    double d = std::fmod(a - b + 540.0, 360.0) - 180.0;
    return std::abs(d);
}

// 이상적인 UCA 스냅샷 하나에서 R 을 만든다. 매니폴드와 같은 물리 모델이지만
// 코드 경로는 다르게 — 여기서 직접 위상을 계산해 매니폴드를 "채점"한다.
void synth_R(cd* R, int m, double radius_m, double freq_hz,
             const double* bearings_cw, const double* amps, int nsrc,
             double noise_var, int nsamp, uint64_t seed, Sense sense){
    std::mt19937_64 rng(seed);
    std::normal_distribution<double> g(0.0, 1.0);
    const double k   = 2.0 * kPi * freq_hz / 299792458.0;
    const double dir = (sense == Sense::CW) ? 1.0 : -1.0;

    for(int i = 0; i < m*m; i++) R[i] = cd(0,0);
    std::vector<cd> x(m);
    const double ns = std::sqrt(noise_var / 2.0);
    for(int n = 0; n < nsamp; n++){
        for(int i = 0; i < m; i++) x[i] = cd(ns*g(rng), ns*g(rng));
        for(int s = 0; s < nsrc; s++){
            const cd sig(amps[s]*g(rng)/std::sqrt(2.0), amps[s]*g(rng)/std::sqrt(2.0));
            const double beta = bearings_cw[s] * kPi / 180.0;
            for(int i = 0; i < m; i++){
                // beta 는 항상 시계방향 방위. 배선 방향은 소자 위치에만 건다.
                const double phi = dir * 2.0 * kPi * i / (double)m;
                const double ph  = k * radius_m * std::cos(phi - beta);
                x[i] += sig * cd(std::cos(ph), std::sin(ph));
            }
        }
        for(int i = 0; i < m; i++)
            for(int j = 0; j < m; j++) R[i*m+j] += x[i] * std::conj(x[j]);
    }
    for(int i = 0; i < m*m; i++) R[i] /= (double)nsamp;
}

// Kraken 의 gen_scanning_vectors 를 그대로 재현한 것. 코드베이스에서 이 식이
// 등장하는 유일한 곳이며, 쓰이는 데가 아니라 theta_kraken = 360 - beta 라는
// 관계를 문서화·검증하기 위한 것이다. GPL 코드를 옮긴 게 아니라 관계를
// 확인하기 위해 공개된 수식을 적어둔 것.
void kraken_sv(int m, double radius_m, double freq_hz, double theta_deg, cd* out){
    const double lambda = 299792458.0 / freq_hz;
    const double r_wl   = radius_m / lambda;
    const double t      = theta_deg * kPi / 180.0;
    for(int i = 0; i < m; i++){
        const double a = 2.0 * kPi * i / (double)m;
        const double x =  r_wl * std::cos(a);
        const double y = -r_wl * std::sin(a);
        const double ph = 2.0 * kPi * (x * std::cos(t) + y * std::sin(t));
        out[i] = cd(std::cos(ph), std::sin(ph));
    }
}

// ── 1. 헤더 레이아웃 ──────────────────────────────────────────────────────
void test_header(){
    printf("[1] heimdall header layout\n");
    check(sizeof(IqHeader) == 1024, "sizeof(IqHeader) == 1024 (got %zu)", sizeof(IqHeader));
    IqHeader h{};
    h.cpi_length = 1048576; h.active_ant_chs = 5;
    check(payload_bytes(h) == 41943040ull, "payload_bytes(1048576,5) == 41943040");
    h.sync_word = kSyncWord;
    check(sync_ok(h), "sync word accepted");
    h.frame_type = FRAME_DATA; h.delay_sync_flag = 1; h.iq_sync_flag = 1; h.noise_source_state = 0;
    check(usable_for_df(h), "calibrated DATA frame is usable");
    h.frame_type = FRAME_CAL;
    check(!usable_for_df(h), "CAL frame rejected");
    h.frame_type = FRAME_DATA; h.noise_source_state = 1;
    check(!usable_for_df(h), "noise source on rejected");
    h.noise_source_state = 0; h.iq_sync_flag = 0;
    check(!usable_for_df(h), "iq_sync_flag clear rejected");
}

// ── 2. Jacobi 고유분해 ────────────────────────────────────────────────────
void test_eigen(){
    printf("[2] hermitian eigensolver\n");
    std::mt19937_64 rng(12345);
    std::normal_distribution<double> g(0.0, 1.0);
    double worst_res = 0.0, worst_ortho = 0.0, worst_order = 0.0;
    for(int trial = 0; trial < 2000; trial++){
        const int m = 2 + (int)(rng() % 7);      // 2..8
        std::vector<cd> A(m*m), B(m*m);
        for(int i = 0; i < m; i++)
            for(int j = 0; j <= i; j++){
                cd v = (i == j) ? cd(g(rng), 0.0) : cd(g(rng), g(rng));
                A[i*m+j] = v; A[j*m+i] = std::conj(v);
            }
        double ev[kMaxElements]; cd V[kMaxElements*kMaxElements];
        hermitian_eigen(A.data(), m, ev, V);

        for(int k = 1; k < m; k++) worst_order = std::max(worst_order, ev[k-1] - ev[k]);
        for(int k = 0; k < m; k++){
            for(int i = 0; i < m; i++){
                cd s(0,0);
                for(int j = 0; j < m; j++) s += A[i*m+j] * V[j*m+k];
                worst_res = std::max(worst_res, std::abs(s - ev[k]*V[i*m+k]));
            }
        }
        for(int p = 0; p < m; p++)
            for(int q = 0; q < m; q++){
                cd s(0,0);
                for(int i = 0; i < m; i++) s += std::conj(V[i*m+p]) * V[i*m+q];
                worst_ortho = std::max(worst_ortho, std::abs(s - cd(p==q?1.0:0.0, 0.0)));
            }
    }
    check(worst_res   < 1e-10, "residual ||A v - lambda v|| < 1e-10 (max %.2e)", worst_res);
    check(worst_ortho < 1e-12, "eigenvectors orthonormal (max dev %.2e)", worst_ortho);
    check(worst_order <= 0.0,  "eigenvalues ascending (max violation %.2e)", worst_order);

    // 축퇴·대각 입력에서 죽지 않아야 한다.
    { cd I[9]={1,0,0, 0,1,0, 0,0,1}; double ev[3]; cd V[9];
      hermitian_eigen(I,3,ev,V);
      check(std::abs(ev[0]-1)<1e-12 && std::abs(ev[2]-1)<1e-12, "identity -> all eigenvalues 1"); }
    { cd Z[4]={0,0,0,0}; double ev[2]; cd V[4];
      hermitian_eigen(Z,2,ev,V);
      check(std::abs(ev[0])<1e-12 && std::abs(ev[1])<1e-12, "zero matrix handled"); }
}

// ── 3. 규약 잠금 ──────────────────────────────────────────────────────────
// 이 프로젝트에서 가장 중요한 테스트. 여기가 통과해야 방위가 시계방향이다.
void test_convention(){
    printf("[3] bearing convention (THE critical test)\n");
    const int    M = 5;
    const double R_M = 0.175, F = 700e6;
    Manifold mf; mf.ensure(F, R_M, M, Sense::CW);

    check(mf.valid() && mf.elements() == M, "manifold built (lambda %.4f m, ambiguity %.3f)",
          mf.lambda_m(), mf.ambiguity_ratio());

    // (a) 합성 평면파를 정확한 시계방향 방위에서 쏘고 되찾는다.
    const double test_b[] = {0, 37, 72, 90, 151, 180, 251, 288, 359};
    double worst = 0.0;
    for(double b : test_b){
        cd Rm[25]; double amp[1] = {1.0}; double br[1] = {b};
        synth_R(Rm, M, R_M, F, br, amp, 1, 1e-6, 4000, 777, Sense::CW);
        Estimate e = estimate_doa(Rm, M, mf, Algo::Music, 1, 4000, 30.0, -99.0);
        worst = std::max(worst, ang_err(e.bearing_deg, b));
        if(g_verb > 0) printf("        beta=%5.1f -> %7.2f  (err %.3f)\n", b, e.bearing_deg, ang_err(e.bearing_deg,b));
    }
    check(worst < 0.5, "MUSIC recovers CW bearing, max error %.3f deg", worst);

    // (b) 거울 실수를 명시적으로 잡는 단언. beta=90 이 270 으로 나오면
    //     매니폴드 부호가 뒤집힌 것이다.
    {
        cd Rm[25]; double amp[1]={1.0}; double br[1]={90.0};
        synth_R(Rm, M, R_M, F, br, amp, 1, 1e-6, 4000, 4242, Sense::CW);
        Estimate e = estimate_doa(Rm, M, mf, Algo::Bartlett, 1, 4000, 30.0, -99.0);
        check(ang_err(e.bearing_deg, 90.0) < 1.0,
              "beta=90 reads 90 (got %.2f) - NOT mirrored to 270", e.bearing_deg);
        check(ang_err(e.bearing_deg, 270.0) > 90.0, "beta=90 is not 270");
    }

    // (c) Kraken 식과의 관계: sv_kraken(360-beta) == a_ours(beta)
    double worst_rel = 0.0;
    for(double b : {0.0, 37.0, 72.0, 151.0, 288.0}){
        cd kv[8]; kraken_sv(M, R_M, F, std::fmod(360.0 - b, 360.0), kv);
        cd ov[8]; mf.steer(b, ov);
        for(int i = 0; i < M; i++) worst_rel = std::max(worst_rel, std::abs(kv[i] - ov[i]));
    }
    check(worst_rel < 1e-9,
          "theta_kraken == 360 - beta  (max |sv_k(360-b) - a(b)| = %.2e)", worst_rel);

    // (d) 전 방위 스윕
    double sweep_worst = 0.0; int sweep_bad = 0;
    for(int b = 0; b < 360; b += 1){
        cd Rm[25]; double amp[1]={1.0}; double br[1]={(double)b};
        synth_R(Rm, M, R_M, F, br, amp, 1, 1e-8, 2000, 900 + b, Sense::CW);
        Estimate e = estimate_doa(Rm, M, mf, Algo::Music, 1, 2000, 30.0, -99.0);
        const double err = ang_err(e.bearing_deg, b);
        sweep_worst = std::max(sweep_worst, err);
        if(err > 1.0) sweep_bad++;
    }
    check(sweep_bad == 0, "full 0..359 sweep, %d bearings off by >1 deg (max %.3f)",
          sweep_bad, sweep_worst);

    // (e) CCW 설정이 실제로 뒤집는지 — 현장 탈출구가 동작하는지 확인
    {
        Manifold cw, ccw;
        cw.ensure(F, R_M, M, Sense::CW);
        ccw.ensure(F, R_M, M, Sense::CCW);
        cd Rm[25]; double amp[1]={1.0}; double br[1]={110.0};
        // 배열이 실제로 CCW 로 배선된 상황을 합성한다.
        synth_R(Rm, M, R_M, F, br, amp, 1, 1e-6, 4000, 31337, Sense::CCW);
        Estimate ecw  = estimate_doa(Rm, M, cw,  Algo::Bartlett, 1, 4000, 30.0, -99.0);
        Estimate eccw = estimate_doa(Rm, M, ccw, Algo::Bartlett, 1, 4000, 30.0, -99.0);
        check(ang_err(eccw.bearing_deg, 110.0) < 1.0,
              "CCW array + Sense::CCW -> 110 (got %.2f)", eccw.bearing_deg);
        check(ang_err(ecw.bearing_deg, 250.0) < 1.0,
              "CCW array + Sense::CW  -> mirrored 250 (got %.2f)", ecw.bearing_deg);
    }
}

// ── 4. 분해능 ─────────────────────────────────────────────────────────────
void test_resolution(){
    printf("[4] two-source resolution\n");
    const int M = 5; const double R_M = 0.175, F = 700e6;
    Manifold mf; mf.ensure(F, R_M, M, Sense::CW);
    double br[2] = {60.0, 150.0}, amp[2] = {1.0, 1.0};
    cd Rm[25];
    synth_R(Rm, M, R_M, F, br, amp, 2, 1e-4, 8000, 555, Sense::CW);
    Estimate e = estimate_doa(Rm, M, mf, Algo::Music, 2, 8000, 30.0, -99.0);
    const double d = std::min(ang_err(e.bearing_deg, 60.0), ang_err(e.bearing_deg, 150.0));
    check(d < 3.0, "MUSIC(d=2) peak lands on one of the two sources (err %.2f)", d);
}

// ── 5. 수락 규칙 ──────────────────────────────────────────────────────────
void test_accept(){
    printf("[5] accept / reject rule\n");
    const int M = 5; const double R_M = 0.175, F = 700e6;
    Manifold mf; mf.ensure(F, R_M, M, Sense::CW);

    // 잡음만: 100% 거부되어야 한다. 임의 각도를 보고하면 최악의 실패다.
    int accepted = 0;
    const int N_TRIAL = 200, NS = 2048;
    for(int t = 0; t < N_TRIAL; t++){
        cd Rm[25];
        synth_R(Rm, M, R_M, F, nullptr, nullptr, 0, 1.0, NS, 1000 + t, Sense::CW);
        Estimate e = estimate_doa(Rm, M, mf, Algo::Music, 1, NS, 30.0, -99.0);
        if(e.ok) accepted++;
    }
    check(accepted == 0, "noise-only rejected %d/%d (false accepts: %d)",
          N_TRIAL - accepted, N_TRIAL, accepted);

    // 소자당 -10 dB 신호: 대부분 수락되어야 한다.
    int det = 0;
    for(int t = 0; t < N_TRIAL; t++){
        cd Rm[25]; double br[1] = {73.0}, amp[1] = {std::sqrt(0.1)};
        synth_R(Rm, M, R_M, F, br, amp, 1, 1.0, NS, 5000 + t, Sense::CW);
        Estimate e = estimate_doa(Rm, M, mf, Algo::Music, 1, NS, 30.0, -99.0);
        if(e.ok && ang_err(e.bearing_deg, 73.0) < 10.0) det++;
    }
    check(det >= (int)(0.95 * N_TRIAL), "-10 dB signal detected %d/%d", det, N_TRIAL);

    // 임계가 1/sqrt(N) 으로 스케일하는지 — look 수를 바꿔도 오경보가 안 늘어야 한다
    for(int ns : {256, 512, 8192}){
        int fa = 0;
        for(int t = 0; t < 100; t++){
            cd Rm[25];
            synth_R(Rm, M, R_M, F, nullptr, nullptr, 0, 1.0, ns, 7000 + t, Sense::CW);
            Estimate e = estimate_doa(Rm, M, mf, Algo::Music, 1, ns, 30.0, -99.0);
            if(e.ok) fa++;
        }
        check(fa == 0, "noise-only at n_eff=%d: %d false accepts", ns, fa);
    }
}

// ── 6. XSpec 전 경로 ──────────────────────────────────────────────────────
// 시간영역 5채널 IQ 를 만들어 XSpec(창 -> FFT -> 빈 선택 -> 외적)에 통과시키고
// 방위가 나오는지 본다. 여기까지 맞아야 "빈 선택 = 채널화" 라는 전제가 성립한다.
void test_xspec(){
    printf("[6] XSpec (Welch cross-spectral) end to end\n");
    const int    M = 5;
    const double R_M = 0.175;
    const double FS = 2.4e6, DAQ_CF = 700e6;
    const double CH_OFF = 312.5e3;              // DAQ 중심에서 벗어난 채널
    const double CH_CF = DAQ_CF + CH_OFF, CH_BW = 25e3;
    const double TRUE_B = 137.0;
    const size_t N = 262144;

    std::mt19937_64 rng(2024);
    std::normal_distribution<double> g(0.0, 1.0);
    std::vector<std::complex<float>> iq((size_t)M * N);

    const double k = 2.0 * kPi * CH_CF / 299792458.0;
    double sv_re[8], sv_im[8];
    for(int m = 0; m < M; m++){
        const double ph = k * R_M * std::cos(2.0*kPi*m/M - TRUE_B*kPi/180.0);
        sv_re[m] = std::cos(ph); sv_im[m] = std::sin(ph);
    }
    // 원하는 신호(대역 내) + 훨씬 센 간섭(대역 밖) + 잡음. 간섭을 제대로
    // 걸러내지 못하면 방위가 간섭 쪽으로 끌려간다.
    const double INT_OFF = -600e3, INT_BW = 200e3, INT_B = 20.0;
    double iv_re[8], iv_im[8];
    const double ki = 2.0 * kPi * (DAQ_CF + INT_OFF) / 299792458.0;
    for(int m = 0; m < M; m++){
        const double ph = ki * R_M * std::cos(2.0*kPi*m/M - INT_B*kPi/180.0);
        iv_re[m] = std::cos(ph); iv_im[m] = std::sin(ph);
    }
    std::complex<double> s_lp(0,0), i_lp(0,0);
    const double a_s = CH_BW / FS, a_i = INT_BW / FS;   // 1차 저역통과로 대역 만들기
    for(size_t n = 0; n < N; n++){
        s_lp += a_s * (std::complex<double>(g(rng), g(rng)) - s_lp);
        i_lp += a_i * (std::complex<double>(g(rng), g(rng)) - i_lp);
        const double t = (double)n / FS;
        const std::complex<double> sc = s_lp * std::polar(1.0, 2.0*kPi*CH_OFF*t)
                                      / std::sqrt(a_s);
        const std::complex<double> ic = i_lp * std::polar(10.0, 2.0*kPi*INT_OFF*t)
                                      / std::sqrt(a_i);      // 20 dB 강함
        for(int m = 0; m < M; m++){
            std::complex<double> v = sc * std::complex<double>(sv_re[m], sv_im[m])
                                   + ic * std::complex<double>(iv_re[m], iv_im[m])
                                   + std::complex<double>(g(rng), g(rng)) * 0.05;
            iq[(size_t)m*N + n] = std::complex<float>((float)v.real(), (float)v.imag());
        }
    }

    XSpec xs; Status why{};
    XSpec::Params p;
    p.ch_center_hz = CH_CF; p.ch_bw_hz = CH_BW;
    p.daq_center_hz = DAQ_CF; p.daq_fs_hz = FS;
    p.elements = M; p.target_looks = 2048; p.fft_size = 8192;
    check(xs.prepare(p, why), "prepare ok (bins=%d K=%d eff_bw=%.0f Hz)",
          xs.bins(), xs.fft_size(), xs.effective_bw_hz());
    check(xs.bins() >= 8, "at least 8 bins in a 25 kHz channel (got %d)", xs.bins());
    check(xs.add_frame(iq.data(), N), "add_frame ok");

    std::complex<double> Rn[64];
    check(xs.snapshot_R(Rn), "snapshot_R ok (n_eff=%.0f)", xs.n_eff());

    Manifold mf; mf.ensure(CH_CF, R_M, M, Sense::CW);
    Estimate e = estimate_doa(Rn, M, mf, Algo::Music, 1, xs.n_eff(), 30.0, -99.0);
    check(e.ok, "accepted (conf %.2f dB, eigSNR %.1f dB, pwr %.1f dBFS)",
          e.confidence_db, e.eig_snr_db, e.power_dbfs);
    check(ang_err(e.bearing_deg, TRUE_B) < 3.0,
          "bearing %.2f vs true %.1f (err %.2f) - out-of-band interferer rejected",
          e.bearing_deg, TRUE_B, ang_err(e.bearing_deg, TRUE_B));

    // 대역 밖 채널은 거부해야 한다.
    { XSpec x2; Status w2{}; XSpec::Params q = p; q.ch_center_hz = DAQ_CF + 3.0e6;
      check(!x2.prepare(q, w2) && w2 == Status::BandOutOfSpan, "out-of-span channel rejected"); }
    // DC 위에 얹힌 채널: LO 누설 가드에 다 먹히면 거부
    { XSpec x2; Status w2{}; XSpec::Params q = p; q.ch_center_hz = DAQ_CF; q.ch_bw_hz = 1000.0;
      q.dc_guard_hz = 2000.0;
      check(!x2.prepare(q, w2) && w2 == Status::DcOverlap, "DC-overlapping channel rejected"); }
}

} // namespace

int run_selftest(int verbosity){
    g_fail = g_run = 0; g_verb = verbosity;
    printf("=== DF selftest ===\n");
    test_header();
    test_eigen();
    test_convention();
    test_resolution();
    test_accept();
    test_xspec();
    printf("=== %d/%d checks passed ===\n", g_run - g_fail, g_run);
    return g_fail;
}

} // namespace df
