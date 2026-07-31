#include "df_linalg.hpp"

#include <cmath>
#include <algorithm>

namespace df {

using cd = std::complex<double>;

int hermitian_eigen(const cd* a_in, int m, double* eval, cd* evec){
    if(m < 1 || m > kMaxElements) return -1;

    cd A[kMaxElements * kMaxElements];
    cd V[kMaxElements * kMaxElements];
    for(int i = 0; i < m; i++)
        for(int j = 0; j < m; j++){
            A[i*m + j] = a_in[i*m + j];
            V[i*m + j] = (i == j) ? cd(1.0, 0.0) : cd(0.0, 0.0);
        }

    // 대각합으로 스케일을 잡아 절대 임계 대신 상대 임계를 쓴다.
    double scale = 0.0;
    for(int i = 0; i < m; i++) scale += std::abs(A[i*m + i]);
    if(scale <= 0.0) scale = 1.0;
    const double eps = 1e-30 * scale * scale;

    int sweep = 0;
    const int kMaxSweeps = 60;   // M<=8 이면 보통 5~8 sweep 이면 끝난다
    for(; sweep < kMaxSweeps; sweep++){
        double off = 0.0;
        for(int p = 0; p < m; p++)
            for(int q = p+1; q < m; q++) off += std::norm(A[p*m + q]);
        if(off <= eps) break;

        for(int p = 0; p < m; p++){
            for(int q = p+1; q < m; q++){
                const cd z = A[p*m + q];
                const double az = std::abs(z);
                if(az * az <= eps) continue;

                const double app = A[p*m + p].real();
                const double aqq = A[q*m + q].real();
                const double theta = 0.5 * std::atan2(2.0 * az, app - aqq);
                const double c = std::cos(theta);
                const double s = std::sin(theta);
                // e^{i phi} = z / |z|
                const cd eip = z / az;
                const cd eim = std::conj(eip);

                // 열 갱신: A <- A U,  V <- V U
                //   A[i][p]' =  c*A[i][p] + s*e^{-i phi}*A[i][q]
                //   A[i][q]' = -s*e^{+i phi}*A[i][p] + c*A[i][q]
                for(int i = 0; i < m; i++){
                    const cd aip = A[i*m + p], aiq = A[i*m + q];
                    A[i*m + p] = c * aip + (s * eim) * aiq;
                    A[i*m + q] = -(s * eip) * aip + c * aiq;
                    const cd vip = V[i*m + p], viq = V[i*m + q];
                    V[i*m + p] = c * vip + (s * eim) * viq;
                    V[i*m + q] = -(s * eip) * vip + c * viq;
                }
                // 행 갱신: A <- U^H A
                //   A[p][j]' =  c*A[p][j] + s*e^{+i phi}*A[q][j]
                //   A[q][j]' = -s*e^{-i phi}*A[p][j] + c*A[q][j]
                for(int j = 0; j < m; j++){
                    const cd apj = A[p*m + j], aqj = A[q*m + j];
                    A[p*m + j] = c * apj + (s * eip) * aqj;
                    A[q*m + j] = -(s * eim) * apj + c * aqj;
                }
                // 회전 대상 항은 정확히 0 으로 못 박는다 (누적 오차 방지).
                A[p*m + q] = cd(0.0, 0.0);
                A[q*m + p] = cd(0.0, 0.0);
                A[p*m + p] = cd(A[p*m + p].real(), 0.0);
                A[q*m + q] = cd(A[q*m + q].real(), 0.0);
            }
        }
    }

    int idx[kMaxElements];
    for(int i = 0; i < m; i++){ eval[i] = A[i*m + i].real(); idx[i] = i; }
    std::sort(idx, idx + m, [&](int x, int y){ return eval[x] < eval[y]; });

    double sorted[kMaxElements];
    for(int k = 0; k < m; k++) sorted[k] = eval[idx[k]];
    for(int k = 0; k < m; k++){
        for(int i = 0; i < m; i++) evec[i*m + k] = V[i*m + idx[k]];
        eval[k] = sorted[k];
    }
    return sweep;
}

} // namespace df
