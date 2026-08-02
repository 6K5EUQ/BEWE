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
    Manifold mf; mf.ensure(F, make_geom(ArrayType::Uca, M, R_M, Sense::CW));

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
    // 톨러런스는 연속 피크 보정(df_estimator.cpp refine_continuous)이 들어간 뒤
    // 0.5 -> 0.05 로 조였다. 느슨하게 두면 그 개선이 조용히 되돌아가도 안 잡힌다.
    check(worst < 0.05, "MUSIC recovers CW bearing, max error %.3f deg", worst);

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

    // (d2) 격자 셀 *안쪽* 스윕 — 연속 피크 보정의 유일한 증거다.
    //      (d) 는 정수도만 쏘므로 격자점이라 어떤 보간을 써도 맞는다.
    //      옛 방식(의사스펙트럼 3점 포물선)은 MUSIC 의 1/x 봉우리를 담지 못해
    //      여기서 0.199도 RMS / 0.282도 최대로 정수도 쪽에 뭉쳤다. 분모
    //      (a^H Pn a) 위에서 연속 보정하면 1e-5도 아래로 떨어진다.
    {
        double sub_worst = 0.0, sq = 0.0; int n = 0;
        for(int fi = 0; fi < 20; fi++){
            const double frac = fi * 0.05;
            for(double base : {17.0, 100.0, 233.0}){
                const double b = base + frac;
                cd Rm[25]; double amp[1]={1.0}; double br[1]={b};
                synth_R(Rm, M, R_M, F, br, amp, 1, 1e-12, 8000, 5150, Sense::CW);
                Estimate e = estimate_doa(Rm, M, mf, Algo::Music, 1, 8000, 30.0, -99.0);
                const double err = ang_err(e.bearing_deg, b);
                sub_worst = std::max(sub_worst, err);
                sq += err * err; n++;
            }
        }
        const double rms = std::sqrt(sq / (double)std::max(n, 1));
        check(sub_worst < 0.05,
              "sub-degree sweep: max %.4f RMS %.4f deg (grid-parabola was 0.282 / 0.199)",
              sub_worst, rms);
    }

    // (f) UCA 사이드로브 기하 고정.
    //     ambiguity_ratio = 2 r sin(pi/M) / (lambda/2) 는 ULA 휴리스틱이라
    //     UCA 에서 양방향으로 틀린다: 현 운용점(700 MHz)에서 0.96 을 내놓아
    //     "안전"이라 하지만, 실제 배열 상관은 정확히 180도에서 0.5732
    //     (= -4.8 dB) 다. 이 상수를 못박아 두면 매니폴드 수식을 건드렸을 때
    //     기하가 바뀐 걸 바로 잡아낸다.
    {
        double worst_sll = 0.0, at_delta = 0.0;
        cd a0[8], a1[8];
        for(int i = 0; i < (int)(360 / M); i++){          // M중 회전대칭 -> 1/M 만
            const double b0 = (double)i;
            mf.steer(b0, a0);
            for(int j = 40; j <= 320; j++){               // 주엽 밖만
                mf.steer(b0 + j, a1);
                cd acc(0.0, 0.0);
                for(int k = 0; k < M; k++) acc += a1[k] * std::conj(a0[k]);
                const double c = std::abs(acc) / M;
                if(c > worst_sll){ worst_sll = c; at_delta = (double)j; }
            }
        }
        check(std::abs(worst_sll - 0.5732) < 2e-3 && std::abs(at_delta - 180.0) < 1e-9,
              "UCA sidelobe %.4f at %.0f deg (expect 0.5732 @ 180; ambiguity_ratio says %.2f = 'safe')",
              worst_sll, at_delta, mf.ambiguity_ratio());
    }

    // (e) CCW 설정이 실제로 뒤집는지 — 현장 탈출구가 동작하는지 확인
    {
        Manifold cw, ccw;
        cw.ensure(F, make_geom(ArrayType::Uca, M, R_M, Sense::CW));
        ccw.ensure(F, make_geom(ArrayType::Uca, M, R_M, Sense::CCW));
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

    // (f) 좌표 경로가 옛 UCA 폐쇄식과 같은 조향벡터를 내는가.
    // ensure() 가 반경/소자수/sense 대신 좌표를 받도록 바뀌었다 (v15). UCA 는
    // x=r sin(phi), y=r cos(phi) 를 넣으면 k r cos(phi-b) 로 되돌아가는 특수해라
    // 수치가 정확히 같아야 한다 — 어긋나면 기존 배치의 방위가 통째로 틀어진다.
    {
        Manifold mf2; mf2.ensure(F, make_geom(ArrayType::Uca, M, R_M, Sense::CW));
        const double k = 2.0*M_PI*F/299792458.0;
        double worst = 0.0;
        for(int b = 0; b < 360; b += 7){
            const cd* a = mf2.col(b);
            const double beta = b * M_PI / 180.0;
            for(int m = 0; m < M; m++){
                const double phi = 2.0*M_PI*m/(double)M;
                const double ph  = k * R_M * std::cos(phi - beta);   // 옛 식
                worst = std::max(worst, std::abs(a[m] - cd(std::cos(ph), std::sin(ph))));
            }
        }
        check(worst < 1e-9, "coordinate path matches the closed-form UCA (max dev %.2e)", worst);
    }

    // (g) ULA 는 배열 축에 대해 대칭이라 b 와 180-b 를 원리적으로 못 가른다.
    // 이건 버그가 아니라 기하의 성질이고, UI 가 그 경고를 띄우는 근거다.
    // 여기서 확인하는 건 "정말로 같은가" — 같지 않다면 좌표 생성이 틀린 것이다.
    {
        Manifold ula; ula.ensure(F, make_geom(ArrayType::Ula, M, 0.125, Sense::CW));
        double worst = 0.0;
        for(int b = 10; b < 170; b += 13){
            const cd* a1 = ula.col(b);
            const cd* a2 = ula.col(180 - b);
            for(int m = 0; m < M; m++) worst = std::max(worst, std::abs(a1[m] - a2[m]));
        }
        check(worst < 1e-9, "ULA is mirror-degenerate as designed (max dev %.2e)", worst);
    }

    // (h) ULA+1 은 한 소자를 축 밖으로 빼 그 대칭을 깬다. 좌우가 실제로 갈리는지.
    {
        Manifold up; up.ensure(F, make_geom(ArrayType::UlaPlus, M, 0.125, Sense::CW));
        double best_match = 1e30;
        for(int b = 10; b < 170; b += 13){
            const cd* a1 = up.col(b);
            const cd* a2 = up.col(180 - b);
            double d = 0.0;
            for(int m = 0; m < M; m++) d += std::abs(a1[m] - a2[m]);
            best_match = std::min(best_match, d);
        }
        check(best_match > 0.1, "ULA+1 breaks the mirror (closest pair differs by %.3f)", best_match);
    }
}

// ── 4. 분해능 ─────────────────────────────────────────────────────────────
void test_resolution(){
    printf("[4] two-source resolution\n");
    const int M = 5; const double R_M = 0.175, F = 700e6;
    Manifold mf; mf.ensure(F, make_geom(ArrayType::Uca, M, R_M, Sense::CW));
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
    Manifold mf; mf.ensure(F, make_geom(ArrayType::Uca, M, R_M, Sense::CW));

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

    // 임계가 1/sqrt(N) 으로 스케일하는지 — look 수를 바꿔도 오경보가 안 늘어야 한다.
    // 64 는 Config::validate 가 허용하는 target_looks 하한이다. 여기가 가장
    // 위험한 구간이라 반드시 포함해야 한다 — lmin 기반 SNR 지표를 쓰던 시절엔
    // 이 지점에서 순수 잡음에 +0.74 dB SNR 을 보고했다 (잡음 고유값 평균으로
    // 바꾼 이유). 통계 바닥이 여전히 잡아주는지 확인한다.
    for(int ns : {64, 256, 512, 8192}){
        int fa = 0;
        for(int t = 0; t < 100; t++){
            cd Rm[25];
            synth_R(Rm, M, R_M, F, nullptr, nullptr, 0, 1.0, ns, 7000 + t, Sense::CW);
            Estimate e = estimate_doa(Rm, M, mf, Algo::Music, 1, ns, 30.0, -99.0);
            if(e.ok) fa++;
        }
        check(fa == 0, "noise-only at n_eff=%d: %d false accepts", ns, fa);
    }

    // 순수 잡음에서 보고되는 SNR 이 0 dB 아래여야 한다. lmin 을 잡음 추정으로
    // 쓰면 표본 최소 고유값이 아래로 치우쳐 SNR 이 양수로 나온다 — 운용자
    // 임계(snr_threshold_db)가 걸리는 바로 그 값이므로 편향이 직접 위험이다.
    {
        double sum = 0.0; int n = 0;
        for(int t = 0; t < 100; t++){
            cd Rm[25];
            synth_R(Rm, M, R_M, F, nullptr, nullptr, 0, 1.0, 64, 8100 + t, Sense::CW);
            Estimate e = estimate_doa(Rm, M, mf, Algo::Music, 1, 64, 30.0, -99.0);
            sum += e.eig_snr_db; n++;
        }
        const double avg = sum / std::max(n, 1);
        check(avg < 0.0, "noise-only mean eig_snr = %+.2f dB at n_eff=64 (lmin metric gave +0.74)", avg);
    }

    // 소자 하나가 죽은 배열: 방위는 여전히 자신만만하게 나오지만 imbalance 가
    // 서야 한다. 이게 유일한 검출기다 — PAPR 도 고유값비도 이 고장을 못 본다.
    {
        cd Rm[25]; double br[1] = {73.0}, amp[1] = {1.0};
        synth_R(Rm, M, R_M, F, br, amp, 1, 1e-3, 4000, 9100, Sense::CW);
        // 소자 2 의 행/열을 40 dB 낮춘다 (LNA 고장 / 케이블 단선 모사)
        const double g = 1e-4;
        for(int i = 0; i < M; i++){ Rm[i*M + 2] *= g; Rm[2*M + i] *= g; }
        Rm[2*M + 2] *= g;   // 대각은 g^2 이 되도록 한 번 더
        Estimate e = estimate_doa(Rm, M, mf, Algo::Music, 1, 4000, 30.0, -99.0);
        check(e.imbalance, "dead element flagged (diag spread %.0f dB)", e.diag_spread_db);
    }

    // 정상 배열에서는 imbalance 가 서면 안 된다 (오탐 확인).
    {
        cd Rm[25]; double br[1] = {73.0}, amp[1] = {1.0};
        synth_R(Rm, M, R_M, F, br, amp, 1, 1e-3, 4000, 9200, Sense::CW);
        Estimate e = estimate_doa(Rm, M, mf, Algo::Music, 1, 4000, 30.0, -99.0);
        check(!e.imbalance, "healthy array not flagged (diag spread %.1f dB)", e.diag_spread_db);
    }

    // ── SNR 지표가 signal_dim 에 안 흔들려야 한다 ─────────────────────────
    // 잡음 추정을 signal_dim 으로 자르면 "sources (MUSIC)" 슬라이더가
    // Bartlett/Capon 의 수락 임계까지 조용히 움직인다. 특히 d=elements-1 이면
    // 잡음 집합이 한 개로 줄어 옛 lmin 지표(하향 편향)로 그대로 되돌아간다.
    {
        cd Rm[25]; double br[1] = {73.0}, amp[1] = {1.0};
        synth_R(Rm, M, R_M, F, br, amp, 1, 1e-2, 2048, 9300, Sense::CW);
        double worst = 0.0;
        Estimate ref = estimate_doa(Rm, M, mf, Algo::Bartlett, 1, 2048, 30.0, -99.0);
        for(int sd = 1; sd <= M-1; sd++){
            Estimate e = estimate_doa(Rm, M, mf, Algo::Bartlett, sd, 2048, 30.0, -99.0);
            worst = std::max(worst, std::abs(e.eig_snr_db - ref.eig_snr_db));
        }
        check(worst < 1e-9, "eig_snr independent of signal_dim (max drift %.3f dB)", worst);
    }

    // ── 두 번째 소스가 잡음 추정을 오염시키면 안 된다 ────────────────────
    // 잡음 고유값을 평균내면 초과 소스가 섞여 SNR 이 폭락하고 멀쩡한 신호가
    // "No signal" 로 거부된다 (측정: 실제 +17 dB 인데 평균 지표는 +9.1 dB).
    // 중앙값은 소수의 오염값에 안 흔들린다.
    {
        cd R1[25], R2[25];
        double br1[1] = {73.0},          amp1[1] = {1.0};
        double br2[2] = {73.0, 200.0},   amp2[2] = {1.0, 1.0};
        synth_R(R1, M, R_M, F, br1, amp1, 1, 1e-2, 2048, 9400, Sense::CW);
        synth_R(R2, M, R_M, F, br2, amp2, 2, 1e-2, 2048, 9400, Sense::CW);
        Estimate e1 = estimate_doa(R1, M, mf, Algo::Music, 1, 2048, 30.0, -99.0);
        Estimate e2 = estimate_doa(R2, M, mf, Algo::Music, 1, 2048, 30.0, -99.0);
        check(e2.eig_snr_db > e1.eig_snr_db - 3.0,
              "2nd source does not crater SNR (1src %.1f -> 2src %.1f dB)",
              e1.eig_snr_db, e2.eig_snr_db);
    }

    // ── 깨끗한 단일 소스에 거짓 모호성이 뜨면 안 된다 ────────────────────
    // Bartlett 스펙트럼은 사실상 배열 패턴이라 단일 소스만 있어도 배열 고유
    // 사이드로브(M=5/700 MHz 에서 180도 -4.8 dB)가 그대로 보인다. 고정 -20 dB
    // 임계로 자르면 그 기하가 매 측정마다 "대안 방위" 로 보고됐다 (실측 360/360).
    // 임계는 배열 자신의 사이드로브 기준이어야 한다.
    {
        int fired = 0;
        for(int b = 0; b < 360; b += 13){
            cd Rm[25]; double br[1] = {(double)b}, amp[1] = {1.0};
            synth_R(Rm, M, R_M, F, br, amp, 1, 1e-3, 8000, 9500 + b, Sense::CW);
            Estimate e = estimate_doa(Rm, M, mf, Algo::Bartlett, 1, 8000, 30.0, -99.0);
            if(e.alt_n > 0) fired++;
        }
        check(fired == 0, "Bartlett clean single source: %d/28 false alt reports", fired);
    }

    // 진짜 두 번째 방출체는 여전히 잡아야 한다 (임계를 올려 눈이 멀면 안 된다).
    // MUSIC 은 signal_dim 만큼만 신호로 보므로 2소스를 보려면 d=2 여야 한다 —
    // d=1 이면 두 번째 소스가 잡음 부분공간에 들어가 오히려 널이 된다.
    {
        cd Rm[25];
        double br[2] = {73.0, 200.0}, amp[2] = {1.0, 0.75};
        synth_R(Rm, M, R_M, F, br, amp, 2, 1e-3, 8000, 9600, Sense::CW);
        Estimate e = estimate_doa(Rm, M, mf, Algo::Music, 2, 8000, 30.0, -99.0);
        check(e.alt_n > 0, "real 2nd emitter still reported as alt (alt_n=%d)", e.alt_n);
    }

    // 최대 정규화 dB 계약: 어떤 R 이 와도 spectrum_db 는 0 을 넘지 않는다.
    // 바닥(1e-30)을 분자에만 걸던 시절엔 피크가 그 아래로 내려간 병적인 R 에서
    // 전 방위가 양수 dB 로 나왔다.
    {
        cd Rz[25];
        for(int i = 0; i < 25; i++) Rz[i] = cd(0.0, 0.0);
        for(int a = 0; a < 3; a++){
            Estimate e = estimate_doa(Rz, M, mf, (Algo)a, 1, 100, 30.0, -99.0);
            double mxdb = -1e9;
            for(int b = 0; b < kAngleBins; b++) mxdb = std::max(mxdb, (double)e.spectrum_db[b]);
            check(mxdb <= 0.0, "algo %d: zero R -> spectrum_db max %.2f dB (must be <= 0)", a, mxdb);
        }
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

    Manifold mf; mf.ensure(CH_CF, make_geom(ArrayType::Uca, M, R_M, Sense::CW));
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

// ── 7. 매니폴드 캘리브레이션 ──────────────────────────────────────────────
// 실제로 고쳐야 하는 상황을 그대로 만든다: 설정에 적힌 좌표와 물리 배열이
// 다르고, 채널마다 케이블 위상차가 있다. 계산 매니폴드로는 방위가 틀리고,
// 몇 방위를 실측해 보정을 넣으면 맞아야 한다.
void test_calib(){
    printf("[7] manifold calibration\n");
    const int    M = 5;
    const double F = 438e6, R_M = 0.20;

    // 설정에 적힌 (믿고 있는) 배열
    const ArrayGeom believed = make_geom(ArrayType::Uca, M, R_M, Sense::CW);

    // 실제 배열: 반경이 3 cm 크고 소자 두 개가 각도로 어긋나 있다. 줄자로 재고
    // 손으로 단 배열에서 흔한 정도의 오차다.
    ArrayGeom truth = make_geom(ArrayType::Uca, M, R_M + 0.03, Sense::CW);
    { const double a = 8.0 * kPi / 180.0;   // 소자 2 를 8 도 돌린다
      const double x = truth.x[2], y = truth.y[2];
      truth.x[2] = x*std::cos(a) - y*std::sin(a);
      truth.y[2] = x*std::sin(a) + y*std::cos(a); }
    { const double a = -5.0 * kPi / 180.0;
      const double x = truth.x[4], y = truth.y[4];
      truth.x[4] = x*std::cos(a) - y*std::sin(a);
      truth.y[4] = x*std::sin(a) + y*std::cos(a); }

    // 채널별 고정 위상/이득 오차 (동축 길이차 + LNA 편차)
    const cd hw[5] = { cd(1,0), std::polar(1.05, 0.55), std::polar(0.92, -0.9),
                       std::polar(1.11, 1.7), std::polar(0.97, -2.2) };

    Manifold mf_true; mf_true.ensure(F, truth);
    Manifold mf_bel;  mf_bel .ensure(F, believed);

    // 방위 b 에서 배열이 실제로 보는 벡터 (소자 0 위상 0 으로 정규화)
    auto observe = [&](double b, cd* out){
        cd a[kMaxElements];
        mf_true.steer(b, a);
        for(int m = 0; m < M; m++) out[m] = a[m] * hw[m];
        const cd r = std::conj(out[0]) / std::abs(out[0]);
        for(int m = 0; m < M; m++) out[m] *= r;
    };
    // 그 벡터 하나짜리 공분산 (잡음 없는 이상적 관측 — 기하 오차만 보려는 것)
    auto make_R = [&](const cd* x, cd* R){
        for(int i = 0; i < M; i++)
            for(int j = 0; j < M; j++) R[i*M+j] = x[i] * std::conj(x[j]);
    };

    const double probes[] = { 17.0, 73.0, 128.0, 194.0, 251.0, 310.0 };

    // (a) 보정 전: 믿고 있는 매니폴드로는 방위가 틀린다
    double worst_before = 0.0;
    for(double b : probes){
        cd x[kMaxElements], R[kMaxElements*kMaxElements];
        observe(b, x); make_R(x, R);
        Estimate e = estimate_doa(R, M, mf_bel, Algo::Bartlett, 1, 4000, 30.0, -99.0);
        worst_before = std::max(worst_before, ang_err(e.bearing_deg, b));
    }
    check(worst_before > 3.0,
          "uncalibrated array is off by %.1f deg (the error this exists to fix)", worst_before);

    // (b) 12 방위를 30 도 간격으로 실측해 보정을 만든다
    Calib cal;
    cal.set_context(F, M);
    for(int i = 0; i < 12; i++){
        const double b = i * 30.0;
        cd x[kMaxElements], a[kMaxElements];
        observe(b, x);
        mf_bel.steer(b, a);
        const cd r = std::conj(a[0]) / std::abs(a[0]);
        for(int m = 0; m < M; m++) a[m] *= r;
        cal.add(b, 30.0, x, a, M);
    }
    check(cal.count() == 12, "collected %d calibration points", cal.count());

    // (c) 보정 후: 측정점 사이의 방위에서도 맞아야 한다 (probes 는 30 의 배수가
    //     아니므로 전부 보간 구간이다 — 표를 그대로 되읽는 게 아니다)
    Manifold mf_cal; mf_cal.ensure(F, believed, &cal);
    check(mf_cal.calibrated(), "calibration is applied to the manifold");
    double worst_after = 0.0;
    for(double b : probes){
        cd x[kMaxElements], R[kMaxElements*kMaxElements];
        observe(b, x); make_R(x, R);
        Estimate e = estimate_doa(R, M, mf_cal, Algo::Bartlett, 1, 4000, 30.0, -99.0);
        worst_after = std::max(worst_after, ang_err(e.bearing_deg, b));
    }
    check(worst_after < 2.0, "calibrated bearing within %.2f deg (was %.1f)",
          worst_after, worst_before);

    // (d) 주파수가 멀면 거부해야 한다 — 틀린 보정은 무보정보다 나쁘다
    check(!cal.usable_at(F * 1.2, M), "calibration refused 20%% away in frequency");
    check( cal.usable_at(F * 1.01, M), "calibration accepted 1%% away");
    check(!cal.usable_at(F, M-1),      "calibration refused on a different element count");

    // (e) 저장/복원 왕복
    {
        const char* path = "/tmp/bewe_df_calib_selftest.txt";
        check(cal.save(path), "calibration saved");
        Calib rd;
        check(rd.load(path), "calibration loaded");
        cd c0[kMaxElements], c1[kMaxElements];
        double worst = 0.0;
        for(double b = 0; b < 360; b += 7){
            cal.correction(b, c0, M);
            rd .correction(b, c1, M);
            for(int m = 0; m < M; m++) worst = std::max(worst, std::abs(c0[m]-c1[m]));
        }
        check(worst < 1e-6, "round-trip preserves the correction (max dev %.2e)", worst);
        remove(path);
    }
}

int run_selftest(int verbosity){
    g_fail = g_run = 0; g_verb = verbosity;
    printf("=== DF selftest ===\n");
    test_header();
    test_eigen();
    test_convention();
    test_resolution();
    test_accept();
    test_xspec();
    test_calib();
    printf("=== %d/%d checks passed ===\n", g_run - g_fail, g_run);
    return g_fail;
}

} // namespace df
