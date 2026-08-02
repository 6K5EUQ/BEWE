#pragma once
// ── 배열 매니폴드 캘리브레이션 ────────────────────────────────────────────
//
// 왜 필요한가
// -----------
// Manifold 는 소자 좌표에서 조향벡터를 **계산**한다. 그 계산이 맞으려면 좌표가
// 정확하고, 5 채널의 RF 경로가 동일하며, 안테나끼리 결합하지 않고, 기체 금속이
// 재방사하지 않아야 한다. 현장에서는 넷 다 어긋난다:
//
//   - 좌표: 줄자로 잰 값이라 cm 단위 오차가 남는다
//   - RF 경로: 동축 길이차 1 cm 가 438 MHz 에서 위상 17 도다
//   - 상호결합: 소자가 lambda/2 안쪽에 있으면 서로 끌어당긴다
//   - 산란: 드론 동체·팔·모터가 파면을 휜다
//
// 이 중 어느 것도 좌표 수정으로는 못 없앤다 (방위마다 다르게 나타나기 때문).
// 그래서 **아는 방위에서 실제로 쏴 보고**, 그때 배열이 본 조향벡터를 이론값과
// 비교해 그 차이를 표로 남긴다. 이후 측정은 그 표를 곱한 매니폴드를 쓴다.
//
// 무엇을 저장하나
// ---------------
// 방위 b_i 마다 소자별 복소 보정 c_i[m] = x_i[m] / a(b_i)[m] 이다. x 는 실측
// (주 고유벡터), a 는 이론 조향벡터. 둘 다 소자 0 의 위상이 0 이 되게 정규화한
// 뒤 나누므로 c[0] 은 항상 1 이고, 나머지가 "이 방위에서 이 소자는 이만큼
// 어긋난다" 를 담는다.
//
// 측정점 사이는 보간한다. 진폭은 선형, 위상은 최단호로 — 복소수를 그냥 선형
// 보간하면 두 점의 위상이 반대일 때 크기가 0 으로 꺼진다.
//
// 주파수 의존성
// -------------
// 보정은 주파수마다 다르다 (케이블 위상차는 주파수 비례, 결합·산란은 비선형).
// 그래서 세트마다 측정 주파수를 박아두고, 그 주파수에서 멀면 적용하지 않는다
// — 틀린 보정은 보정이 없는 것보다 나쁘다.

#include "df_types.hpp"
#include <complex>
#include <cstdint>

namespace df {

inline constexpr int kMaxCalPoints = 36;   // 10 도 간격이면 한 바퀴

struct CalPoint {
    double bearing_deg = 0;                       // 운용자가 입력한 참값
    double snr_db      = 0;                       // 그때의 eig_snr — 신뢰도 판단용
    std::complex<double> corr[kMaxElements] = {}; // 소자별 보정 (corr[0] == 1)
};

class Calib {
public:
    // 측정점 추가/치환. 같은 방위(2 도 안쪽)가 이미 있으면 덮어쓴다 — 운용자가
    // 마음에 안 드는 측정을 다시 하는 게 정상 흐름이라 중복을 쌓지 않는다.
    // 방위 순으로 정렬해 둔다 (보간이 이웃을 찾는다).
    void add(double bearing_deg, double snr_db,
             const std::complex<double>* measured,   // 실측 주 고유벡터
             const std::complex<double>* theory,     // 같은 방위의 이론 조향벡터
             int elements);

    void clear();
    void remove_at(int idx);

    int  count() const { return n_; }
    const CalPoint& at(int i) const { return pts_[i]; }

    // 이 세트가 유효한 주파수/소자수. add() 시점의 값이 박힌다.
    void   set_context(double freq_hz, int elements);
    double freq_hz()  const { return freq_hz_; }
    int    elements() const { return m_; }

    // 이 측정이 지금 세트와 같은 조건인가 (주파수 5% 이내, 소자 수 일치).
    // 점 개수는 안 본다 — 두 번째 점을 넣을 때도 참이어야 하기 때문이다.
    bool same_context(double freq_hz, int elements) const;

    // 지금 주파수·소자수에 이 보정을 **적용**해도 되는가. same_context 에 더해
    // 보간에 필요한 최소 점수(2)를 요구한다.
    //
    // 이 둘을 한 함수로 묶었다가 캘리브가 1 점을 못 넘는 버그가 났다: 두 번째
    // capture 에서 "적용 불가(=점 1개)" 를 "다른 세트" 로 읽고 기존 점을 지워
    // 매번 1 점으로 되돌아갔다. 질문이 둘이면 함수도 둘이어야 한다.
    bool usable_at(double freq_hz, int elements) const;

    // 방위 b 의 보정을 out[0..m) 에 쓴다. 측정점이 2 개 미만이면 전부 1 (무보정).
    void correction(double bearing_deg, std::complex<double>* out, int elements) const;

    // 잔차 진단: 각 측정점의 보정 크기(dB) 중 최댓값. 0 에 가까우면 배열이
    // 모델과 잘 맞는다는 뜻이고, 크면 좌표나 배선을 의심해야 한다.
    double worst_dev_db() const;

    // 내용이 바뀔 때마다 증가. Manifold 가 캐시 무효화에 쓴다 (측정점 하나를
    // 다시 잡아도 다음 ensure 가 테이블을 새로 만들어야 한다).
    uint32_t stamp() const { return stamp_; }

    // ── 영속화 ───────────────────────────────────────────────────────────
    // host_state.json 이 아니라 별도 파일이다. 36 점 x 8 소자 x 복소수면
    // host_state 가 통째로 캘리브 계수로 뒤덮이고, 그 파일은 채널·주파수 복원
    // 때문에 자주 다시 쓰인다. 캘리브는 이륙 전에 한 번 잡고 마는 값이라
    // 수명이 전혀 다르다.
    bool save(const char* path) const;
    bool load(const char* path);

private:
    CalPoint pts_[kMaxCalPoints];
    int      n_ = 0;
    double   freq_hz_ = 0.0;
    int      m_ = 0;
    uint32_t stamp_ = 0;
};

} // namespace df
