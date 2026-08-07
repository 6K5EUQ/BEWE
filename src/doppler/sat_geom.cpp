#include "sat_geom.hpp"
#include "../SGP4.h"
#include <cmath>

namespace SatGeom {

static constexpr double D2R = M_PI/180.0;
static constexpr double R2D = 180.0/M_PI;

void site_ecef(double lat_deg, double lon_e_deg, double alt_km, double r_site[3]){
    const double la = lat_deg*D2R, lo = lon_e_deg*D2R;
    const double sphi = std::sin(la), cphi = std::cos(la);
    const double N = A_E_KM / std::sqrt(1.0 - E2*sphi*sphi);
    r_site[0] = (N + alt_km)*cphi*std::cos(lo);
    r_site[1] = (N + alt_km)*cphi*std::sin(lo);
    r_site[2] = (N*(1.0 - E2) + alt_km)*sphi;
}

void teme_to_ecef_cs(const double rt[3], const double vt[3],
                     double cg, double sg, double r_ecef[3], double v_ecef[3]){
    // 위치: PEF = ROT3(gst)*TEME — sat_tle.cpp:139-141 이 이미 하는 것과 동일.
    r_ecef[0] =  cg*rt[0] + sg*rt[1];
    r_ecef[1] = -sg*rt[0] + cg*rt[1];
    r_ecef[2] =             rt[2];
    // 속도: v_pef = ROT3(gst)*v_teme - (omega_e x r_pef),  omega_e = (0,0,OMEGA_E)
    //       omega x r = (-w*ry, +w*rx, 0)  ->  빼면 (+w*ry, -w*rx, 0)
    const double a0 =  cg*vt[0] + sg*vt[1];
    const double a1 = -sg*vt[0] + cg*vt[1];
    v_ecef[0] = a0 + OMEGA_E*r_ecef[1];
    v_ecef[1] = a1 - OMEGA_E*r_ecef[0];
    v_ecef[2] =      vt[2];
}

void teme_to_ecef(const double rt[3], const double vt[3], double jd_ut1,
                  double r_ecef[3], double v_ecef[3]){
    const double gst = SGP4Funcs::gstime_SGP4(jd_ut1);
    teme_to_ecef_cs(rt, vt, std::cos(gst), std::sin(gst), r_ecef, v_ecef);
}

void ecef_look(const double r_sat[3], const double v_sat[3], const double r_site[3],
               double lat_deg, double lon_e_deg, LookAngle& out){
    const double rho[3] = { r_sat[0]-r_site[0], r_sat[1]-r_site[1], r_sat[2]-r_site[2] };
    const double rng = std::sqrt(rho[0]*rho[0] + rho[1]*rho[1] + rho[2]*rho[2]);
    out.range_km = rng;
    // 관측점은 ECEF 에서 정지 → v_sat 이 곧 d(rho)/dt.
    out.range_rate_km_s = (rng > 0.0)
        ? (rho[0]*v_sat[0] + rho[1]*v_sat[1] + rho[2]*v_sat[2]) / rng : 0.0;

    const double la = lat_deg*D2R, lo = lon_e_deg*D2R;
    const double sla = std::sin(la), cla = std::cos(la);
    const double slo = std::sin(lo), clo = std::cos(lo);
    // rho_sez = ROT2(90-lat) * ROT3(lon) * rho
    const double S =  sla*clo*rho[0] + sla*slo*rho[1] - cla*rho[2];
    const double E = -slo*rho[0]     + clo*rho[1];
    const double Z =  cla*clo*rho[0] + cla*slo*rho[1] + sla*rho[2];
    out.el_deg = (rng > 0.0) ? std::asin(Z/rng)*R2D : 0.0;
    double az = std::atan2(E, -S)*R2D;
    if(az < 0.0) az += 360.0;
    out.az_deg = az;
}

double gamma_rad(const double u_site[3], const double r_sat_ecef[3], double& r_geo_km){
    const double m = std::sqrt(r_sat_ecef[0]*r_sat_ecef[0]
                             + r_sat_ecef[1]*r_sat_ecef[1]
                             + r_sat_ecef[2]*r_sat_ecef[2]);
    r_geo_km = m;
    if(m <= 0.0) return M_PI;
    double c = (u_site[0]*r_sat_ecef[0] + u_site[1]*r_sat_ecef[1] + u_site[2]*r_sat_ecef[2]) / m;
    if(c >  1.0) c =  1.0;
    if(c < -1.0) c = -1.0;
    return std::acos(c);
}

bool prop_teme(elsetrec& scratch, double t_unix, double r_teme[3], double v_teme[3],
               double& jd_out){
    const double jd = jd_from_unix(t_unix);
    jd_out = jd;
    // sgp4() 는 에폭부터의 경과 '분'을 받는다.
    const double tsince = (jd - scratch.jdsatepoch - scratch.jdsatepochF) * 1440.0;
    if(!SGP4Funcs::sgp4(scratch, tsince, r_teme, v_teme)) return false;
    return scratch.error == 0;
}

} // namespace SatGeom
