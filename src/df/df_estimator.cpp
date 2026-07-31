#include "df_estimator.hpp"
#include "df_linalg.hpp"

#include <cmath>
#include <algorithm>

namespace df {

using cd = std::complex<double>;

namespace {

// q = a^H B a. B 는 Hermitian 이므로 결과는 실수여야 하고, 반올림 오차만
// 허수부에 남는다 — real() 로 버린다.
double quad_form(const cd* a, const cd* B, int m){
    double acc = 0.0;
    for(int i = 0; i < m; i++){
        cd s(0.0, 0.0);
        for(int j = 0; j < m; j++) s += B[i*m + j] * a[j];
        acc += (std::conj(a[i]) * s).real();
    }
    return acc;
}

// 고유분해로 역행렬 (Capon 용). R = V diag(l) V^H  =>  R^-1 = V diag(1/l) V^H.
// 대각 로딩으로 조건수를 잡는다 — look 수가 적으면 R 이 특이에 가깝다.
void inverse_via_eigen(const double* eval, const cd* evec, int m,
                       double load, cd* out){
    for(int i = 0; i < m; i++)
        for(int j = 0; j < m; j++) out[i*m + j] = cd(0.0, 0.0);
    for(int k = 0; k < m; k++){
        const double lk = eval[k] + load;
        if(lk <= 0.0) continue;
        const double inv = 1.0 / lk;
        for(int i = 0; i < m; i++)
            for(int j = 0; j < m; j++)
                out[i*m + j] += inv * evec[i*m + k] * std::conj(evec[j*m + k]);
    }
}

double papr_db(const double* p, int n){
    double mx = 0.0, sum = 0.0;
    for(int i = 0; i < n; i++){ mx = std::max(mx, p[i]); sum += p[i]; }
    const double mean = sum / n;
    if(mean <= 0.0 || mx <= 0.0) return 0.0;
    return 10.0 * std::log10(mx / mean);
}

// 정수 격자 피크 주변 3점 포물선 보간. 1도 격자에서 0.1도 수준까지 좁혀준다.
// 참조 구현에는 없는 단계지만 공짜에 가깝고, 격자 양자화가 그대로 방위 오차로
// 남는 걸 막는다.
double refine_peak(const double* p, int n, int k){
    const double ym = p[(k - 1 + n) % n], y0 = p[k], yp = p[(k + 1) % n];
    const double den = ym - 2.0 * y0 + yp;
    if(den == 0.0) return (double)k;
    double d = 0.5 * (ym - yp) / den;
    if(d < -1.0 || d > 1.0) d = 0.0;      // 피크가 아니면 보간하지 않는다
    double r = k + d;
    while(r < 0.0)   r += n;
    while(r >= n)    r -= n;
    return r;
}

} // namespace

Estimate estimate_doa(const cd* R, int m, const Manifold& mf, Algo algo,
                      int signal_dim, double n_eff,
                      double c_papr, double snr_thr_db){
    Estimate e;
    if(m < 2 || m > kMaxElements || !mf.valid() || mf.elements() != m) return e;

    // 전력: Re(tr R)/M. 페이로드가 공칭 +-1.0 이라 이게 곧 dBFS 다.
    double tr = 0.0;
    for(int i = 0; i < m; i++) tr += R[i*m + i].real();
    const double pw = tr / m;
    e.power_dbfs = 10.0 * std::log10(std::max(pw, 1e-30));

    double eval[kMaxElements];
    cd     evec[kMaxElements * kMaxElements];
    hermitian_eigen(R, m, eval, evec);
    for(int i = 0; i < m; i++) e.eval[i] = eval[i];

    const double lmin = std::max(eval[0], 1e-30);
    const double lmax = eval[m - 1];
    e.eig_snr_db = 10.0 * std::log10(std::max((lmax - lmin) / lmin, 1e-30));

    // ── Bartlett 은 알고리즘 선택과 무관하게 항상 계산한다 ────────────────
    // 수락 판정을 Bartlett PAPR 로 하기 때문이다. MUSIC 의 PAPR 은 순수 잡음
    // 에서도 꼬리가 길어(측정상 최대 9 dB) 임계를 잡기 어렵다. Bartlett 은
    // 귀무분포가 좁고 1/sqrt(N) 으로 깔끔하게 줄어 검출기로 훨씬 낫다.
    double pb[kAngleBins];
    for(int b = 0; b < kAngleBins; b++) pb[b] = quad_form(mf.col(b), R, m);
    e.confidence_db = papr_db(pb, kAngleBins);

    double ps[kAngleBins];
    if(algo == Algo::Bartlett){
        for(int b = 0; b < kAngleBins; b++) ps[b] = pb[b];
    } else if(algo == Algo::Capon){
        cd Rinv[kMaxElements * kMaxElements];
        inverse_via_eigen(eval, evec, m, 1e-6 * std::max(tr / m, 1e-30), Rinv);
        for(int b = 0; b < kAngleBins; b++){
            const double d = quad_form(mf.col(b), Rinv, m);
            ps[b] = (d > 0.0) ? 1.0 / d : 0.0;
        }
    } else { // MUSIC
        int d = signal_dim;
        if(d < 1) d = 1;
        if(d > m - 1) d = m - 1;        // 잡음 부분공간이 최소 1차원은 있어야 한다
        // En = 작은 고유값 (m-d) 개의 고유벡터. eval 은 오름차순이므로 앞쪽.
        cd Pn[kMaxElements * kMaxElements];
        for(int i = 0; i < m; i++)
            for(int j = 0; j < m; j++) Pn[i*m + j] = cd(0.0, 0.0);
        for(int k = 0; k < m - d; k++)
            for(int i = 0; i < m; i++)
                for(int j = 0; j < m; j++)
                    Pn[i*m + j] += evec[i*m + k] * std::conj(evec[j*m + k]);
        for(int b = 0; b < kAngleBins; b++){
            const double den = quad_form(mf.col(b), Pn, m);
            ps[b] = (den > 1e-300) ? 1.0 / den : 0.0;
        }
    }
    e.algo_papr_db = papr_db(ps, kAngleBins);

    int kmax = 0;
    for(int b = 1; b < kAngleBins; b++) if(ps[b] > ps[kmax]) kmax = b;
    e.bearing_deg = refine_peak(ps, kAngleBins, kmax);

    double mx = ps[kmax];
    if(mx <= 0.0) mx = 1.0;
    for(int b = 0; b < kAngleBins; b++)
        e.spectrum_db[b] = (float)std::max(10.0 * std::log10(std::max(ps[b], 1e-30) / mx), -40.0);

    // ── 수락 규칙 ────────────────────────────────────────────────────────
    // (1) 통계적 바닥. 잡음만 있을 때 Bartlett PAPR 의 귀무분포는 1/sqrt(n_eff)
    //     로 줄어들므로 상수 하나로 어느 look 수에서나 성립한다. 이걸 통과 못
    //     하면 봉우리가 잡음 요동과 구분되지 않는다 = 임의의 각도다.
    // (2) 운용자 임계. 위를 통과해도 SNR 이 운용자가 정한 값에 못 미치면 거부.
    //     (1)은 "각도가 의미 있는가", (2)는 "쓸 만큼 센가" — 역할이 다르다.
    const double ne = std::max(n_eff, 1.0);
    const double papr_thr = c_papr / std::sqrt(ne);
    e.ok = (e.confidence_db > papr_thr) && (e.eig_snr_db >= snr_thr_db);
    return e;
}

} // namespace df
