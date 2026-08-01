#pragma once
// ── DOA 추정기 ────────────────────────────────────────────────────────────
// 전부 공간상관행렬 R (M x M Hermitian) 과 조향 매니폴드만 받는다. 원시 IQ 는
// 여기까지 오지 않는다.
//
//   Bartlett  P(t) = a^H R a                       Van Trees, Optimum Array Processing ch.6
//   Capon     P(t) = 1 / (a^H R^-1 a)              Capon 1969, Proc. IEEE 57(8)
//   MUSIC     P(t) = 1 / (a^H En En^H a)           Schmidt 1986, IEEE TAP 34(3)

#include "df_types.hpp"
#include "df_manifold.hpp"
#include <complex>

namespace df {

struct Estimate {
    double bearing_deg   = 0;    // 안테나 0 기준, heading offset 적용 전
    double confidence_db = 0;    // Bartlett PAPR — 수락 판정에 쓰는 값
    double algo_papr_db  = 0;    // 보고 알고리즘의 PAPR
    double eig_snr_db    = 0;    // 10log10((lmax - noise_mean)/noise_mean)
    double power_dbfs    = 0;    // Re(tr R)/M
    double eval[kMaxElements] = {};
    float  spectrum_db[kAngleBins] = {};   // 보고 알고리즘, 최대 정규화 dB

    // 주엽 밖 국소최대 상위 2개 (모호집합). alt_db 는 피크 대비 dB (<=0).
    double alt_deg[2] = {};
    double alt_db[2]  = { -999.0, -999.0 };
    int    alt_n      = 0;

    double diag_spread_db = 0;   // max/min diag(R) 비 (dB). 소자 전력 불균형
    bool   imbalance = false;    // 중앙값 대비 10배 밖으로 벗어난 소자가 있다
    int    eig_sweeps = 0;       // Jacobi 스윕 수. <0 이면 실패, 상한이면 미수렴

    bool   ok = false;           // 수락 규칙 통과 여부
};

// R      : M x M 행 우선 Hermitian
// mf     : 채널 중심주파수로 ensure() 된 매니폴드
// n_eff     : 유효 look 수. PAPR 임계가 1/sqrt(n_eff) 로 스케일한다
// c_papr    : 통계적 바닥 상수 (내부용, 운용자 비노출)
// snr_thr_db: 운용자가 DF 탭에서 정한 고유값비 SNR 임계
// 반환값의 ok 가 false 면 bearing_deg 는 신뢰할 수 없다.
Estimate estimate_doa(const std::complex<double>* R, int m,
                      const Manifold& mf, Algo algo, int signal_dim,
                      double n_eff, double c_papr, double snr_thr_db);

} // namespace df
