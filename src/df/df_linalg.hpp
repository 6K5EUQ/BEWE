#pragma once
// ── 소형 복소 Hermitian 고유분해 ──────────────────────────────────────────
// MUSIC 이 잡음 부분공간을 뽑는 데 필요하다. 행렬은 M x M, M <= 8 이라
// 순환 Jacobi 로 충분하고 (Golub & Van Loan, Matrix Computations §8.4),
// LAPACK/Eigen 을 끌어들일 이유가 없다. BEWE 는 그 둘 다 링크하지 않는다.
//
// 복소 Hermitian 이라 회전은 두 단계다: 먼저 위상을 돌려 A[p][q] 를 실수로
// 만들고, 그 다음 실대칭 Jacobi 회전을 건다. 하나의 유니터리로 합치면
//   U = [[ c, -s e^{i phi} ],
//        [ s e^{-i phi},  c ]],   tan(2 theta) = 2|z| / (App - Aqq)
// 이고 이게 A[p][q] 를 정확히 0 으로 만든다.

#include "df_types.hpp"
#include <complex>
#include <cstddef>

namespace df {

// 입력 a: 행 우선 M x M Hermitian (상삼각만 맞으면 되지만 전체가 채워져 있다고 본다).
// 출력 eval: 오름차순 고유값 M 개.
//      evec: 행 우선 M x M, 열 k 가 eval[k] 의 고유벡터 (즉 evec[i*M+k]).
// 반환: 수렴에 쓴 sweep 수. 0 이면 입력이 이미 대각.
int hermitian_eigen(const std::complex<double>* a, int m,
                    double* eval, std::complex<double>* evec);

} // namespace df
