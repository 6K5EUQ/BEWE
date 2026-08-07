#pragma once
#include "SGP4.h"
#include <string>
#include <functional>
#include <vector>
#include <ctime>

struct TleElem {
    std::string name;
    int         catalog_num     = 0;
    double      semi_major_km   = 0.0;  // derived: visualisation / LEO filter
    bool        is_starlink     = false;
    bool        is_leo          = false;
    elsetrec    satrec;                 // SGP4 initialised record
};

bool tle_load(const std::string& path, std::vector<TleElem>& out);

// ── Central 이 궤도원소 정본이다 ────────────────────────────────────────────
// 기지마다 받으면 space-track 요청 제한(계정당)을 서로 잡아먹고 카탈로그가 기지별로
// 갈린다. Central 이 하루 한 번 모아 두 계열을 만들고 나머지는 받아 캐시한다:
//   leo_YYYYMMDD.txt = LEO 페이로드, Starlink 제외 (도플러 매칭 기본)
//   all_YYYYMMDD.txt = 전 페이로드 + Starlink       (지구본, Starlink 포함 매칭)
// 캐시 위치: assets/tle/archive/
namespace TleCache {
// ui.cpp 가 NetClient 를 물려 준다 — 하위 모듈은 net 계층을 모른다.
void set_requester(std::function<bool(const std::string&)> fn);
bool request(const std::string& filename);      // Central 에 요청 (false=배선 없음)
bool wired();                                   // Central 연결 배선이 되어 있나
std::string dir();                              // assets/tle/archive
// 가장 최근 <prefix>_*.txt 경로. 없으면 빈 문자열.
std::string newest(const char* prefix);
// 오늘부터 back_days 일 전까지 없는 것을 Central 에 요청. 요청한 개수 반환.
int request_recent(const char* prefix, int back_days = 7);
}

void tle_propagate(const TleElem& e, time_t now_utc,
                   double& lat_deg, double& lon_deg, double& alt_km);
