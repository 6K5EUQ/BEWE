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

// ── 연속 피크 보정 ────────────────────────────────────────────────────────
// 격자에서 샘플된 의사스펙트럼 위에 포물선을 얹는 대신, 실제 목적함수를 격자
// 밖 조향벡터로 직접 평가해 극점을 찾는다.
//
// 왜 바꿨나. 옛 방식은 ps[] 3점에 포물선을 맞췄는데 ps 는 Capon/MUSIC 에서
// 1/x 꼴이라 봉우리가 극단적으로 뾰족하다. 1도 간격 3점이 그 모양을 전혀 담지
// 못해 추정이 격자점 쪽으로 끌려간다 — 보고 방위가 정수도에 뭉치는 형태로
// 눈에 보인다. 측정: 셀 안을 0.05도 간격으로 쓸었을 때 MUSIC 오차 0.199도 RMS
// (최대 0.282도). 분모(a^H Pn a, a^H R^-1 a)는 반대로 매끄럽고 준2차라 같은
// 3점 포물선이 잘 맞는다. 같은 방법을 dB 영역에 적용하면 0.121도까지밖에 못
// 줄지만, 분모 위에서 하면 1e-5도 아래로 떨어진다.
//
// Bartlett 은 주엽이 51도라 격자 3점으로도 이미 0.0001도 수준이었다 — 손해 없음.
//
// h 를 0.5 -> 0.1 -> 0.02 로 좁히며 3회. 매회 곡률 부호를 확인해 극점이 아니면
// (잡음 평탄면 등) 즉시 멈추고, step 을 +-h 로 잘라 폭주를 막는다. 최악의 경우도
// 거친 격자점에서 0.62도를 못 벗어나므로 옛 방식보다 나쁠 수 없다.
//
// 비용: steer 9회(소자당 sincos) + 2차형식 9회. m=5 에서 ~2 kflop, 측정 1회가
// 1.3초인 것에 비하면 무시할 수준이다.
double refine_continuous(const Manifold& mf, const cd* Q, int m,
                         int kmax, bool maximize){
    double b = (double)kmax;
    const double hs[3] = { 0.5, 0.1, 0.02 };
    cd av[kMaxElements];
    for(int i = 0; i < 3; i++){
        const double h = hs[i];
        mf.steer(b,     av); const double y0 = quad_form(av, Q, m);
        mf.steer(b - h, av); const double ym = quad_form(av, Q, m);
        mf.steer(b + h, av); const double yp = quad_form(av, Q, m);
        const double den = ym - 2.0 * y0 + yp;
        // 최대점이면 아래로 볼록(den<0), 최소점이면 위로 볼록(den>0)이어야 한다.
        if(maximize ? (den >= 0.0) : (den <= 0.0)) break;
        double step = 0.5 * h * (ym - yp) / den;
        if(step < -h) step = -h;
        else if(step > h) step = h;
        b += step;
    }
    while(b < 0.0)         b += kAngleBins;
    while(b >= kAngleBins) b -= kAngleBins;
    return b;
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

    // ── 소자 전력 산포 ────────────────────────────────────────────────────
    // 죽었거나 크게 어긋난 소자는 지금 아무 데서도 안 잡힌다 — 완전한 PAPR 과
    // 완전한 고유값비를 만들어 수락 규칙을 통과하고, 조용히 틀린 방위를 낸다.
    // diag(R) 의 중앙값 대비 배수로 잰다. 임계는 일부러 느슨하게(10배 = 10 dB)
    // 잡았다: 운용자가 채널별 if_gain 을 다르게 줄 수 있기 때문이다.
    // 지금은 거부하지 않고 표시만 한다 — 오탐 비용이 놓침 비용보다 크다.
    {
        double dg[kMaxElements];
        for(int i = 0; i < m; i++) dg[i] = std::max(R[i*m + i].real(), 1e-30);
        double srt[kMaxElements];
        for(int i = 0; i < m; i++) srt[i] = dg[i];
        std::sort(srt, srt + m);
        const double med = srt[m / 2];
        e.diag_spread_db = 10.0 * std::log10(srt[m - 1] / srt[0]);
        e.imbalance = (srt[0] < med / 10.0) || (srt[m - 1] > med * 10.0);
    }

    double eval[kMaxElements];
    cd     evec[kMaxElements * kMaxElements];
    e.eig_sweeps = hermitian_eigen(R, m, eval, evec);
    for(int i = 0; i < m; i++) e.eval[i] = eval[i];

    // MUSIC 모델 차수. MUSIC 분기에서만 쓴다 — 아래 SNR 지표는 일부러 이 값에
    // 의존시키지 않는다 (그러면 "sources (MUSIC)" 슬라이더가 Bartlett/Capon 의
    // 수락 임계까지 조용히 움직인다).
    int d = signal_dim;
    if(d < 1) d = 1;
    if(d > m - 1) d = m - 1;        // 잡음 부분공간이 최소 1차원은 있어야 한다

    // ── 고유값비 SNR ──────────────────────────────────────────────────────
    // 잡음 전력을 최소 고유값 하나가 아니라 **최대를 뺀 나머지의 중앙값**으로
    // 잡는다. 두 가지를 동시에 해결한다.
    //
    // (1) lmin 의 하향 편향. 표본 고유값은 Marchenko-Pastur 로 퍼져 최솟값이
    //     아래로 치우치므로 SNR 이 낙관적으로 나온다. 측정(M=5, 순수 잡음):
    //     N=64 에서 lmin 이 +0.75 dB 를 보고한다 — target_looks 하한이 64 이니
    //     잡음에 방위를 붙이는 실경로다. 중앙값은 같은 조건에서 -2.53 dB.
    // (2) 평균의 오염 취약성. 잡음 고유값을 평균내면 소스가 설정보다 많을 때
    //     신호 고유값이 잡음 추정에 섞여 SNR 이 폭락한다. 측정(2소스, 실제
    //     +17.0 dB): 평균 +9.10 dB (8 dB 과소 → 멀쩡한 신호를 "No signal" 로
    //     거부), 중앙값 +18.18 dB. 중앙값은 소수의 오염값에 안 흔들린다.
    //
    // signal_dim 과 무관하게 항상 m-1 개를 본다 — 그래야 알고리즘을 바꿔도,
    // MUSIC 슬라이더를 만져도 수락 임계가 안 움직인다.
    double ns[kMaxElements];
    const int nk = m - 1;
    for(int i = 0; i < nk; i++) ns[i] = eval[i];   // eval 은 이미 오름차순
    const double nmed = (nk & 1) ? ns[nk/2] : 0.5 * (ns[nk/2 - 1] + ns[nk/2]);
    const double nvar = std::max(nmed, 1e-30);
    const double lmax = eval[m - 1];
    e.eig_snr_db = 10.0 * std::log10(std::max((lmax - nvar) / nvar, 1e-30));

    // ── Bartlett 은 알고리즘 선택과 무관하게 항상 계산한다 ────────────────
    // 수락 판정을 Bartlett PAPR 로 하기 때문이다. MUSIC 의 PAPR 은 순수 잡음
    // 에서도 꼬리가 길어(측정상 최대 9 dB) 임계를 잡기 어렵다. Bartlett 은
    // 귀무분포가 좁고 1/sqrt(N) 으로 깔끔하게 줄어 검출기로 훨씬 낫다.
    double pb[kAngleBins];
    for(int b = 0; b < kAngleBins; b++) pb[b] = quad_form(mf.col(b), R, m);
    e.confidence_db = papr_db(pb, kAngleBins);

    // 목적행렬 Q 와 극점 방향. 격자 스캔과 연속 보정이 같은 Q 를 본다 —
    // 둘이 다른 함수를 보면 보정이 스캔이 찾은 봉우리에서 떨어져 나간다.
    //   Bartlett : Q = R,    a^H R a    를 최대화
    //   Capon    : Q = R^-1, a^H R^-1 a 를 최소화
    //   MUSIC    : Q = Pn,   a^H Pn a   를 최소화
    cd Qbuf[kMaxElements * kMaxElements];
    const cd* Q = R;
    bool maximize = true;

    double ps[kAngleBins];
    if(algo == Algo::Bartlett){
        for(int b = 0; b < kAngleBins; b++) ps[b] = pb[b];
        Q = R; maximize = true;
    } else if(algo == Algo::Capon){
        inverse_via_eigen(eval, evec, m, 1e-6 * std::max(tr / m, 1e-30), Qbuf);
        for(int b = 0; b < kAngleBins; b++){
            const double q = quad_form(mf.col(b), Qbuf, m);
            ps[b] = (q > 0.0) ? 1.0 / q : 0.0;
        }
        Q = Qbuf; maximize = false;
    } else { // MUSIC
        // En = 작은 고유값 (m-d) 개의 고유벡터. eval 은 오름차순이므로 앞쪽.
        for(int i = 0; i < m; i++)
            for(int j = 0; j < m; j++) Qbuf[i*m + j] = cd(0.0, 0.0);
        for(int k = 0; k < m - d; k++)
            for(int i = 0; i < m; i++)
                for(int j = 0; j < m; j++)
                    Qbuf[i*m + j] += evec[i*m + k] * std::conj(evec[j*m + k]);
        for(int b = 0; b < kAngleBins; b++){
            const double den = quad_form(mf.col(b), Qbuf, m);
            ps[b] = (den > 1e-300) ? 1.0 / den : 0.0;
        }
        Q = Qbuf; maximize = false;
    }
    e.algo_papr_db = papr_db(ps, kAngleBins);

    int kmax = 0;
    for(int b = 1; b < kAngleBins; b++) if(ps[b] > ps[kmax]) kmax = b;
    e.bearing_deg = refine_continuous(mf, Q, m, kmax, maximize);

    // 바닥(1e-30)을 분자에만 걸면 피크가 그 아래로 내려간 병적인 R 에서
    // dB 가 양수로 나온다 ("최대 정규화 dB" 계약 위반). 분모에도 같은 바닥을 걸고
    // 결과를 0 dB 로 잘라 계약을 코드로 강제한다.
    const double mx = std::max(ps[kmax], 1e-30);
    for(int b = 0; b < kAngleBins; b++){
        const double db = 10.0 * std::log10(std::max(ps[b], 1e-30) / mx);
        e.spectrum_db[b] = (float)std::min(std::max(db, -40.0), 0.0);
    }

    // ── 대안 방위 (모호집합) ──────────────────────────────────────────────
    // 주엽 밖의 국소최대 중 가장 센 것 2개를 함께 보고한다. 반사체나 2차
    // 방출체가 있으면 그쪽으로 lock 될 수 있고, 그건 숨기는 대신 보여주는 게 맞다.
    //
    // 임계는 **알고리즘마다 다르다** — 단일 소스 하나가 그 스펙트럼에 남기는
    // 부엽의 크기가 다르기 때문이다.
    //
    //  Bartlett : 스펙트럼이 사실상 배열 패턴이라 소스가 하나뿐이어도 배열
    //             기하가 만드는 부엽이 그대로 보인다 (M=5, 700 MHz 에서 180도에
    //             -4.8 dB). 고정 -20 dB 로 자르면 그 기하가 매 측정마다 "모호성"
    //             으로 보고되고 — 실측 360/360 — 경보가 정보를 잃는다. 배열이
    //             스스로 만들 수 있는 수준보다 3 dB 이상 센 것만 후보로 본다.
    //  Capon/MUSIC : 부분공간 억제 덕에 같은 상황에서 부엽이 -20 dB 아래로
    //             내려간다 (실측 오탐 0/28). 여기까지 Bartlett 기준을 적용하면
    //             진짜 2차 방출체를 통째로 놓친다.
    //
    // 주엽 폭도 고정 상수가 아니라 데이터에서 잡는다: kmax 에서 양쪽으로
    // 내려가다 처음 다시 올라가는 지점(첫 골)까지가 주엽이다.
    const double alt_thr_db = (algo == Algo::Bartlett)
                            ? std::min(mf.sidelobe_db() + 3.0, -3.0)
                            : -20.0;
    {
        int lo = kmax, hi = kmax;
        for(int s = 1; s < kAngleBins/2; s++){
            const int a = (kmax - s + kAngleBins) % kAngleBins;
            const int p = (kmax - s + 1 + kAngleBins) % kAngleBins;
            if(ps[a] > ps[p]) break;
            lo = a;
        }
        for(int s = 1; s < kAngleBins/2; s++){
            const int a = (kmax + s) % kAngleBins;
            const int p = (kmax + s - 1) % kAngleBins;
            if(ps[a] > ps[p]) break;
            hi = a;
        }
        auto in_main = [&](int b){
            const int off = (b - lo + kAngleBins) % kAngleBins;
            const int wid = (hi - lo + kAngleBins) % kAngleBins;
            return off <= wid;
        };
        for(int b = 0; b < kAngleBins; b++){
            if(in_main(b)) continue;
            const int pm = (b - 1 + kAngleBins) % kAngleBins;
            const int pp = (b + 1) % kAngleBins;
            if(!(ps[b] >= ps[pm] && ps[b] > ps[pp])) continue;   // 국소최대만
            const double db = 10.0 * std::log10(std::max(ps[b], 1e-30) / mx);
            if(db < alt_thr_db) continue;      // 배열이 스스로 만드는 부엽 수준 이하
            if(db > e.alt_db[0]){
                e.alt_db[1]  = e.alt_db[0];  e.alt_deg[1] = e.alt_deg[0];
                e.alt_db[0]  = db;           e.alt_deg[0] = (double)b;
                if(e.alt_n < 2) e.alt_n++;
            } else if(db > e.alt_db[1]){
                e.alt_db[1] = db;            e.alt_deg[1] = (double)b;
                if(e.alt_n < 2) e.alt_n++;
            }
        }
        // 채택된 대안도 연속 보정해 준다 — 표시 각도가 정수도에 뭉치지 않게.
        for(int i = 0; i < e.alt_n; i++)
            e.alt_deg[i] = refine_continuous(mf, Q, m, (int)e.alt_deg[i], maximize);
    }

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
