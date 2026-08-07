// ── 궤도 기하: 관측점 ECEF · TEME→ECEF(위치+속도) · 지평좌표/시선속도 ──────────
//
// SGP4 는 TEME-of-date 로 r[3](km), v[3](km/s) 를 준다. sat_tle.cpp 는 위치만 쓰고
// v 를 버리므로(:137) 시선속도(range-rate) 경로는 이 파일이 처음이다.
//
// **속도 변환의 ω×r 항이 이 기능 전체에서 가장 위험한 줄이다.** OMEGA_E*6900km =
// 0.503 km/s 이고, 403 MHz 에서 676 Hz. 부호가 뒤집히면 1352 Hz 오차인데 이는 진짜
// 위성과 미끼 위성의 분리폭보다 크다. 게다가 다운스트림에서 안 보인다 — 순위만 조용히
// 틀어진다. 그래서 skyfield 대조로 선검증했다 (scratchpad/s1_satgeom.py, 2026-08-07):
//   고도 0.0010deg / 거리 0.0265km / range-rate 0.1875 m/s (403MHz 환산 0.25 Hz).
//   부호를 뒤집으면 722 m/s = 971 Hz — 검증이 이 한 부호를 잡으려고 존재한다.
//
// 좌표계 주의: lon 은 **동경 양수**. HIST 헤더의 station_lon 은 서경 양수이므로
// LongWaterfall::hist_lon_east() 로 한 번 뒤집어서 넘길 것. df_view.cpp:167-170 의
// fabs() 방어는 복사하지 말 것 — 북동반구에서만 우연히 맞는다.
#pragma once
#include <cmath>

struct elsetrec;   // SGP4.h

namespace SatGeom {

// WGS-84 (관측점 전용). 전파는 sat_tle.cpp 가 wgs72 로 하는데 **고치지 말 것** —
// 바꾸면 sat_view 의 모든 위성이 움직인다. 반경 차이 2 m 는 1000 km 슬랜트의 2e-6,
// 도플러로는 0.001 Hz 미만이라 섞어 써도 무해하다.
constexpr double A_E_KM    = 6378.137;
constexpr double FLAT      = 1.0/298.257223563;
constexpr double E2        = FLAT*(2.0-FLAT);          // 6.69437999014e-3
constexpr double OMEGA_E   = 7.292115146706979e-5;     // rad/s, IAU-76/WGS-84
constexpr double C_KM_S    = 299792.458;
constexpr double MU_KM3_S2 = 398600.4418;

// 관측점 측지좌표 → ECEF. lon_e_deg = 동경 양수.
// ECEF 에서 관측점 속도는 정확히 0 이다 — 지구자전 항은 teme_to_ecef 의 속도 쪽에만
// 나타난다. 그래서 버그가 생긴다면 거기다.
void site_ecef(double lat_deg, double lon_e_deg, double alt_km, double r_site[3]);

// TEME-of-date → ECEF(PEF), 위치와 속도 both. 극운동 무시(<0.5", 지표 15 m).
void teme_to_ecef(const double r_teme[3], const double v_teme[3], double jd_ut1,
                  double r_ecef[3], double v_ecef[3]);
// GMST 사전계산판 — 같은 시각 격자를 위성마다 반복 평가하는 핫루프용.
void teme_to_ecef_cs(const double r_teme[3], const double v_teme[3],
                     double cos_gst, double sin_gst,
                     double r_ecef[3], double v_ecef[3]);

struct LookAngle {
    double az_deg;            // 0..360, 북에서 동으로
    double el_deg;            // -90..+90
    double range_km;
    double range_rate_km_s;   // + = 멀어짐
};

// range / range_rate 는 SEZ 회전과 무관하다 — 위경도 오차는 고도 게이트만 나빠지게
// 하고 도플러 점수는 안 건드린다.
void ecef_look(const double r_sat[3], const double v_sat[3], const double r_site[3],
               double lat_deg, double lon_e_deg, LookAngle& out);

// unix time → 완전 율리우스일 (UT1≈UTC, DUT1<0.9s 는 여기서 무의미).
inline double jd_from_unix(double t_unix){ return 2440587.5 + t_unix/86400.0; }

// 고도 el_min 까지 보이는 가시 캡의 지심 반각. 지심-관측점-위성 평면삼각형.
inline double lambda_cap_rad(double r_geo_km, double el_min_rad){
    const double c = A_E_KM*std::cos(el_min_rad)/r_geo_km;
    return (c >= 1.0) ? 0.0 : std::acos(c) - el_min_rad;
}

// 관측점 단위벡터와 위성 ECEF 사이 지심각. v[] 도 SEZ 도 필요 없는 최저가 가시성 대용.
double gamma_rad(const double u_site[3], const double r_sat_ecef[3], double& r_geo_km);

// SGP4 전파 → TEME. scratch 는 호출자 소유 (SGP4 가 satrec 을 변조하므로).
// 근지구(method=='n')는 satrec.t/error 만 쓰므로 위성당 1회 복사 후 재사용 가능하고,
// 심우주(method=='d')는 dspace() 가 atime/xli/xni 적분기 상태를 들고 있어 호출당
// 복사가 필요하다. 캐스케이드 0단계가 심우주를 다 죽이므로 실전에선 안 걸린다.
bool prop_teme(elsetrec& scratch, double t_unix, double r_teme[3], double v_teme[3],
               double& jd_out);

} // namespace SatGeom
