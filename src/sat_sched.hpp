#pragma once
// ── 위성 패스 자동 예약 녹음 (HOST 전용) ─────────────────────────────────────
// `/sched add sat <NORAD> <CF_Hz> [capture_Hz]` 로 규칙을 걸면, TLE 로 기지 상공 패스
// (고도 0° AOS~LOS)를 예측해 앞으로 24시간 안의 패스를 전부 채널 예약 엔트리로
// 채운다. 규칙을 지울 때까지 매 패스 반복된다.
//
// 녹음은 협대역이다: SDR SR 은 그대로 두고 CF 만 위성 주파수에서 채널폭 이상 비켜
// 맞춘 뒤(DC 회피), 채널 믹서가 TLE 도플러를 따라가며 capture_Hz 폭만 기록한다.
// 패스가 끝나면 SatTrim 이 실제 점유 대역폭을 재서 그 폭으로 다시 자른다.
//
// 규칙은 $HOME/BEWE/sat_sched_<station>.txt 에 한 줄씩 저장 (재시작 복원).
// 궤도원소는 TleCache(assets/tle/archive) 의 최신 leo_ → all_ 순으로 찾는다 —
// Central 이 정본이고 `/tle update` 로 받아 온다.
#include <string>
#include <vector>
#include <cstdint>
#include <ctime>
#include <functional>

class FFTViewer;

namespace SatSched {

struct Rule {
    int      norad  = 0;
    double   cf_hz  = 0;      // 정지(rest) 주파수
    uint32_t cap_hz = 0;      // 1차 캡처 폭 (재절단 전)
};
static constexpr uint32_t DEFAULT_CAP_HZ = 200000;

struct Pass {
    time_t aos = 0, los = 0;
    double max_el = 0;          // deg
};

// 엔트리 target 라벨 — 규칙과 엔트리를 잇는 유일한 키.
std::string target_of(int norad);

void load(const std::string& station);
bool add_rule(const Rule& r, std::string& err);   // 같은 NORAD 는 교체
bool del_rule(int norad);
std::vector<Rule> rules();

// 원소 조회. src_file 에 쓴 스냅샷 경로를 돌려준다.
bool find_elem(int norad, std::string& name, std::string& src_file, double& epoch_age_days,
               std::string& err);

// [t0, t1) 안의 패스. t0 에 이미 떠 있으면 aos=t0 로 잘린 패스가 첫 원소가 된다.
bool predict(int norad, double lat_deg, double lon_e_deg, time_t t0, time_t t1,
             std::vector<Pass>& out, std::string& err);

// 1Hz 메인 루프에서 호출. 10분마다(또는 refresh() 뒤 즉시) 규칙별 24h 패스를
// sched_entries 에 채우고, 끝난 지 1시간 넘은 위성 엔트리를 치운다.
void tick(FFTViewer& v);
void refresh();

// "SAT <n>" 라벨이면 n, 아니면 0.
int norad_of(const char* target);

// t_unix → 관측 도플러 편이(Hz) at cf_hz. 원소가 없으면 빈 함수.
std::function<double(double)> doppler_fn(int norad, double lat_deg, double lon_e_deg, double cf_hz);

} // namespace SatSched
