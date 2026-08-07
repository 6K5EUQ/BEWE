// ── 백그라운드 스캔 워커 ─────────────────────────────────────────────────────
//
// 전 파일 스캔은 실측 5~3800 ms (32768bin x 17853행 v3 미preload 파일이 최악).
// scan_file_db_range 는 8000행(65MB)을 렌더 프레임 안에서 동기 처리하는데 우리는
// 그 6배를 만지므로 **반드시 별도 스레드**여야 한다.
//
// 워커는 HistReader::clone_for_thread() 로 자기 리더를 받는다 — 선해제본은
// shared_ptr 로 공유되므로 수백 MB 재해제가 없다 (실측: 사본 4개에 +0.1 MB).
#pragma once
#include "doppler_types.hpp"
#include "doppler_match.hpp"
#include <string>
#include <vector>

class HistReader;

namespace DopplerScan {

enum class State { Idle, Fetching, Running, Done, Failed, Cancelled };

struct Status {
    State    st = State::Idle;
    float    progress = 0.0f;
    char     stage[32] = {0};
    uint32_t n_tracks = 0;
    std::string err;
};

// 전 파일 스캔 (추출 → 적합 → 점수 → 위성 매칭). reader 는 내부에서 복제한다.
void start_full(const HistReader& reader, const Doppler::ExtractParams& P,
                const DopplerMatch::Params& MP, const std::string& tle_dir);

// Meas 박스 정밀분석. row/lin 범위는 뷰어의 Meas 가 그대로 준다.
void start_refine(const HistReader& reader, uint32_t row_lo, uint32_t row_hi,
                  uint32_t lin_lo, uint32_t lin_hi,
                  const Doppler::ExtractParams& P, const DopplerMatch::Params& MP,
                  const std::string& tle_dir);

void   cancel();
Status status();
bool   busy();

// 결과 스냅샷 (뮤텍스 하 복사). 트랙별 후보는 match_of() 로.
bool results(std::vector<Doppler::Candidate>& tracks);
bool match_of(uint32_t track_id, DopplerMatch::Result& out);

// 1위 후보 이름만. 오버레이 라벨용 — Result 전체(후보 20개)를 복사하지 않는다.
bool top_name_of(uint32_t track_id, std::string& out);

// 모달 닫을 때 반드시 호출 — 워커 join.
void shutdown();

} // namespace DopplerScan
