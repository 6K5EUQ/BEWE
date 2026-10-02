#pragma once
// ── 위성 FSK 복조 + 대표 프레임 + 이전 프레임 비교 (HOST 전용) ─────────────────
// 재절단된 위성 녹음(도플러 보정, 수 kHz 폭)에서 2-FSK 를 복조해 비트열을 뽑는다.
//
// 58400 실측 (2026-10-02, DGS-2 패스 2개로 Python 선검증):
//   2-FSK, h≈0.5 (편이 ±1.3 kHz), 심볼율 5000 bd (수신 클럭 기준 4999.65~4999.86),
//   5000비트 = 1초 프레임이 반복된다. 매초 비트의 ~62% 가 같다(고정부).
//   두 패스(37시간 간격)의 대표 프레임이 신뢰 비트 3091개 중 99.2% 일치했다.
//
// 절차: 신호 구간(1초 전력 > 1.5x 중앙값의 최장 연속) → 50 kHz 로 FFT 재샘플 →
//   FM 판별 → 1심볼 이동평균 → (d/dt)^2 의 스펙트럼 선으로 심볼율(4700~5100) →
//   1초(=심볼율 개) 블록마다 10개 위상 중 |v| 평균이 큰 위상으로 판정 →
//   블록을 원형상관으로 정렬해 다수결 = 대표 프레임 + 비트별 신뢰도(|합|/개수).
#include <string>
#include <vector>
#include <cstdint>
#include <cstring>

namespace SatDemod {

struct Frame {
    double               rate_bd = 0;
    int                  n_frames = 0;
    std::vector<uint8_t> bits;     // 대표 프레임 (frame_len)
    std::vector<uint8_t> conf;     // 비트별 신뢰도 0..9
    double stable_frac() const;    // conf>=5 비율
};

struct Result {
    bool        ok = false;
    std::string why;
    Frame       frame;
};

Result run(const std::string& sigmf_data_path);

// 대표 프레임 파일 (텍스트 3줄: 헤더 / 비트 / 신뢰도)
bool save(const std::string& path, const Frame& f, const std::string& header);
bool load(const std::string& path, Frame& f, std::string* header = nullptr);

struct Compare {
    bool   ok = false;
    int    shift = 0;          // b 를 이만큼 원형 이동하면 a 와 맞는다
    int    polarity = 1;       // -1 이면 비트 반전 관계
    int    n_conf = 0;         // 양쪽 모두 신뢰(>=5) 인 비트 수
    int    n_diff = 0;         // 그중 다른 비트 수
    double agree = 0;          // 1 - n_diff/n_conf
};
Compare compare(const Frame& a, const Frame& b);

// 위성별 "최신 대표 프레임" — Central 이 같은 이름으로 덮어써 보관하고, 기지는 예약
// 준비 때 받아 두었다가 패스 뒤 비교한다. 로컬 캐시: $HOME/BEWE/sat_frames/
std::string frame_dir();
std::string latest_name(int norad);              // SATFRAME_<norad>.txt
inline bool is_frame_file(const char* fn){
    return fn && strncmp(fn, "SATFRAME_", 9) == 0;
}

} // namespace SatDemod
