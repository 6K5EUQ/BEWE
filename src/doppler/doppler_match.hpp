// ── 관측 도플러 트랙 → 위성 후보 순위 ────────────────────────────────────────
//
// 15,800개 카탈로그를 전수 정밀탐색하면 SGP4 호출 1520만회(46초)다. 시간창에만
// 의존하는 기하 게이트를 캐스케이드로 걸어 ~20만회(0.6초)로 줄인다.
#pragma once
#include "doppler_types.hpp"
#include <functional>
#include <string>
#include <vector>

namespace DopplerMatch {

struct Obs {
    double lat_deg = 0;        // +N
    double lon_east_deg = 0;   // +E
    double alt_km = 0;         // repo 에 고도 필드가 없다 — 0 고정 (100m 오차 -> 0.001Hz)
    std::string station;
};

struct Params {
    double el_min_deg = 5.0;   // 지형 고려. 보수적으로 잡는 비용은 후보 300개·0.1초인데
                               // 정답을 놓치는 비용은 전부다.
    double win_pad_s  = 120.0;
    int    max_cand   = 20;
};

struct Cand {
    int         norad = 0;
    std::string name;
    double rms_hz = 0;         // **유일한 순위 키**
    double rms_bins = 0;       // bin 폭이 파일마다 49~3750 Hz 라 정규화 필요
    // rms_k / rms_0. **분리도는 스윙/잔차에 비례하므로 설정이 좌우한다** — 합성
    // 진짜 통과로 실측(2026-08-07, 15812개 카탈로그, 정답은 6개 조건 전부 1위):
    //   403 MHz / 잔차 15 Hz → sep 15.2    464 MHz / 15 Hz → 17.6   (명확)
    //   403 MHz / 잔차 50 Hz → sep  4.7    930 MHz / 100Hz → 5.4
    //   137 MHz / 잔차 15 Hz → sep  5.3    137 MHz / 50 Hz → 1.9   (모호)
    // 137 MHz 대역은 스윙이 403 MHz 의 1/3 이라 구조적으로 분리가 약하다. sep<3 이면
    // "이 궤도면의 이것들" 이 정직한 답이다 — 임의 타이브레이커를 만들지 말 것.
    double sep = 0;
    double f0_fit_hz = 0;
    double max_el_deg = 0, az_tca_deg = 0, el_tca_deg = 0, range_tca_km = 0;
    double dtca_s = 0;         // 예측TCA - 관측TCA. 전 후보가 같은 부호로 치우치면
                               // DSP 가 아니라 **TLE 가 낡은 것**이라는 진단이다.
    double slope_err_pct = 0;  // 같은 궤도면 위성 분리용 (슬랜트/고도 판별)
    double cov = 0;            // 예측 가시창 안에 든 트랙점 비율
    double incl_deg = 0, alt_km = 0, tle_age_days = 0;
};

struct Result {
    std::vector<Cand> cands;
    int      n_stage[5] = {0,0,0,0,0};   // 단계별 생존 수 — "왜 못 찾았나" 를 설명하는 유일한 필드
    double   ms = 0;
    double   tle_age_days = 0;
    std::string tle_src;
    int      n_loaded = 0;
    std::string error;
};

using ProgressFn = std::function<void(float, const char*)>;
using CancelFn   = std::function<bool()>;

// 카탈로그 적재. for_utc 는 **파일 녹화 시각** — 벽시계가 아니다. 가장 가까운
// 에폭의 아카이브본을 고른다.
bool load_catalogue(const std::string& tle_dir, double for_utc, std::string& why);
double catalogue_age_days(double ref_utc);
int    catalogue_size();
const std::string& catalogue_src();

// 트랙 하나에 대한 후보 탐색.
bool match(const Doppler::Candidate& trk, const Obs& obs, const Params& P, Result& out,
           const ProgressFn& prog = nullptr, const CancelFn& cancel = nullptr);

// 일자별 JSONL 아카이브 (~/BEWE/modules/doppler/doppler_YYYYMMDD.jsonl, KST).
// 모듈로 등록하지 않으므로 module_registry 의 자동 Central 푸시에 안 걸린다
// (reg() 순회 + target_modes 0 스킵 — 실측 확인).
bool archive_scan(const Doppler::Candidate& trk, const Obs& obs, const Params& P,
                  const Result& r);
bool archive_confirm(const Doppler::Candidate& trk, const Obs& obs, const Cand& pick, int rank);

} // namespace DopplerMatch
