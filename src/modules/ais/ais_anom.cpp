// ── Behavior 이상탐지 규칙층 구현 ──────────────────────────────────────────
// v1: GAP / JUMP / SPEED 3종 (결정적 규칙). ZONE(금지구역)은 폴리곤 config 인프라가
// 필요해 후속 단계로 유보(enum 은 선점).
#include "ais_module.hpp"
#include <unordered_map>
#include <mutex>
#include <cmath>

namespace ais_mod {

namespace {

// ── 규칙 임계 (운영자 조정 여지; 항구 트래픽 기준 보수적) ─────────────────────
constexpr int64_t GAP_MS      = 6 * 60 * 1000;  // 6분 이상 침묵 → 소등 의심
constexpr double  JUMP_KN     = 60.0;           // 함의 속도 이 이상이면 물리적 불가능(순간이동)
constexpr double  SPEED_KN    = 40.0;           // SOG 이 이상이면 과속 (일반 상선/어선 상한 훨씬 넘음)
constexpr int64_t JUMP_MIN_MS = 2000;           // 위치 델타 계산 최소 간격(짧은 간격 노이즈 배제)

struct Prev {
    int64_t t_ms = 0;
    double  lat = 0.0, lon = 0.0;
    bool    has_pos = false;
    bool    underway = false;   // 직전 수신이 항행중(정박/계류 아님 & sog>임계)이었나 — GAP 판정용
};

// 항행중 판정: 정박(1)·계류(5)·좌초(6) 아니고 SOG 가 유의미하게 있음.
inline bool is_underway(const AisRecord& m){
    if(m.nav_status==1 || m.nav_status==5 || m.nav_status==6) return false;
    return m.sog >= 0.5f;   // 0.5kn 미만은 사실상 정지 → 침묵 정상
}

std::mutex g_mtx;
std::unordered_map<uint32_t, Prev> g_prev;   // per-MMSI 직전 상태

// haversine 거리 (해리, nautical miles)
double dist_nm(double lat1, double lon1, double lat2, double lon2){
    constexpr double R_NM = 3440.065;   // 지구 반경(해리)
    double dlat = (lat2-lat1) * M_PI/180.0;
    double dlon = (lon2-lon1) * M_PI/180.0;
    double a = std::sin(dlat/2)*std::sin(dlat/2)
             + std::cos(lat1*M_PI/180.0)*std::cos(lat2*M_PI/180.0)
               *std::sin(dlon/2)*std::sin(dlon/2);
    return 2.0 * R_NM * std::asin(std::min(1.0, std::sqrt(a)));
}

// score: 임계 대비 초과분을 0..1000 으로 정규화 (표시/정렬용). 1000 = 임계의 2배 이상.
uint16_t norm_score(double val, double thr){
    if(thr <= 0) return 0;
    double r = (val - thr) / thr;               // 0 = 임계, 1 = 임계의 2배
    if(r < 0) r = 0; if(r > 1) r = 1;
    return (uint16_t)(r * 1000.0);
}

} // namespace

void host_anom(AisRecord& m){
    m.anom_flag = 0; m.anom_reason = ANOM_NONE; m.anom_score = 0;

    std::lock_guard<std::mutex> lk(g_mtx);
    Prev& p = g_prev[m.mmsi];   // 없으면 기본(t_ms=0) 삽입

    // ── GAP: 직전에 "항행중"이던 배가 장시간 침묵 → 소등 의심 ──
    // 정박/계류선은 원래 송신주기가 길어(수분) 제외. 이동중 배의 침묵만 유의미.
    if(p.t_ms > 0 && p.underway){
        int64_t dt = m.t_ms - p.t_ms;
        if(dt > GAP_MS){
            m.anom_flag = 1;   // watch (재개 시점의 첫 수신에 표시)
            m.anom_reason = ANOM_GAP;
            m.anom_score = norm_score((double)dt, (double)GAP_MS);
        }
    }

    // ── 위치 규칙 (has_pos 인 메시지만) ──
    if(m.has_pos){
        // SPEED: SOG 상한 초과 (sog < 0 = n/a 는 제외; 1023 매핑값도 큰 값이지만 sog 는 kn 스케일)
        if(m.sog >= 0.f && m.sog > (float)SPEED_KN){
            if(m.anom_flag < 2){
                m.anom_flag = 2;   // alert
                m.anom_reason = ANOM_SPEED;
                m.anom_score = norm_score(m.sog, SPEED_KN);
            }
        }
        // JUMP: 직전 위치 대비 함의 속도가 물리적 불가능
        if(p.has_pos && p.t_ms > 0){
            int64_t dt = m.t_ms - p.t_ms;
            if(dt >= JUMP_MIN_MS){
                double nm = dist_nm(p.lat, p.lon, m.lat, m.lon);
                double hours = dt / 3600000.0;
                double implied_kn = nm / hours;
                if(implied_kn > JUMP_KN && m.anom_flag < 2){
                    m.anom_flag = 2;   // alert
                    m.anom_reason = ANOM_JUMP;
                    m.anom_score = norm_score(implied_kn, JUMP_KN);
                }
            }
        }
    }

    // 직전 상태 갱신
    p.t_ms = m.t_ms;
    p.underway = is_underway(m);
    if(m.has_pos){ p.lat = m.lat; p.lon = m.lon; p.has_pos = true; }
}

} // namespace ais_mod
