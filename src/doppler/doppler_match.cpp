#include "doppler_match.hpp"
#include "sat_geom.hpp"
#include "../sat_tle.hpp"
#include "../SGP4.h"
#include "../bewe_paths.hpp"
#include "../kst_time.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <sys/stat.h>

namespace DopplerMatch {

using namespace SatGeom;
static constexpr double D2R = M_PI/180.0;
static constexpr double R2D = 180.0/M_PI;
static constexpr double V_LEO_KM_S = 7.6;

// ── 카탈로그 ────────────────────────────────────────────────────────────────
static std::vector<TleElem> g_sats;
static std::string          g_src;
static double               g_epoch_med_jd = 0;

static double median_epoch_jd(const std::vector<TleElem>& v){
    if(v.empty()) return 0;
    std::vector<double> e; e.reserve(v.size());
    for(const auto& s : v) e.push_back(s.satrec.jdsatepoch + s.satrec.jdsatepochF);
    std::nth_element(e.begin(), e.begin()+e.size()/2, e.end());
    return e[e.size()/2];
}

double catalogue_age_days(double ref_utc){
    if(g_epoch_med_jd <= 0) return 1e9;
    return jd_from_unix(ref_utc) - g_epoch_med_jd;
}
int catalogue_size(){ return (int)g_sats.size(); }
const std::string& catalogue_src(){ return g_src; }

bool load_catalogue(const std::string& tle_dir, double for_utc, std::string& why){
    why.clear();
    // 아카이브본 중 녹화 시각에 가장 가까운 에폭을 고른다. 오늘 TLE 를 3주 뒤로
    // 역전파하는 것은 3주 된 TLE 를 순전파하는 것과 정확히 똑같이 나쁘다 —
    // Celestrak GP 는 현재 원소만 주므로 과거 파일엔 과거 원소가 필요하다.
    struct Cand { std::string path; double dist; };
    std::vector<Cand> cands;
    const std::string live = tle_dir + "/leo_tle.txt";
    {
        struct stat st{};
        if(stat(live.c_str(), &st) == 0) cands.push_back({live, 1e18});
    }
    const std::string adir = tle_dir + "/archive";
    if(DIR* d = opendir(adir.c_str())){
        while(struct dirent* e = readdir(d)){
            const std::string n = e->d_name;
            if(n.rfind("leo_", 0) != 0 || n.size() < 16) continue;
            cands.push_back({adir + "/" + n, 1e18});
        }
        closedir(d);
    }
    if(cands.empty()){ why = "no TLE catalogue in " + tle_dir; return false; }

    // 각 후보의 중앙 에폭을 재려면 읽어야 한다. 후보가 몇 개 안 되므로 그냥 읽는다.
    double best = 1e300; std::string best_path; std::vector<TleElem> best_v;
    for(Cand& c : cands){
        std::vector<TleElem> v;
        if(!tle_load(c.path, v) || v.empty()) continue;
        const double med = median_epoch_jd(v);
        const double dist = std::fabs(jd_from_unix(for_utc) - med);
        if(dist < best){ best = dist; best_path = c.path; best_v = std::move(v); }
    }
    if(best_v.empty()){ why = "TLE load failed"; return false; }
    g_sats = std::move(best_v);
    g_src  = best_path.substr(best_path.rfind('/')+1);
    g_epoch_med_jd = median_epoch_jd(g_sats);
    return true;
}

// ── 캐스케이드 ──────────────────────────────────────────────────────────────
namespace {

struct Work {
    int      idx;
    double   a_km;        // 반장축
    double   n_rad_s;     // 평균운동
    double   incl_deg;
    double   gamma_min;   // 격자 최소 지심각
    double   t_gmin;      // 그때의 시각
};

// 트랙에서 관측된 최대 |rdot| (km/s). f_obs = f0(1 - rdot/c) 이므로
// rdot_max ~ half_swing/f_center * c.
double observed_rdot_max(const Doppler::Candidate& t){
    if(t.fit.valid && t.fit.f_center_hz > 0)
        return std::fabs(t.fit.half_swing_hz) / t.fit.f_center_hz * C_KM_S;
    const double f0 = 0.5*(t.f_min_hz + t.f_max_hz);
    if(f0 <= 0) return 0;
    return 0.5*(t.f_max_hz - t.f_min_hz) / f0 * C_KM_S;
}

} // anon

bool match(const Doppler::Candidate& trk, const Obs& obs, const Params& P, Result& out,
           const ProgressFn& prog, const CancelFn& cancel){
    out = Result{};
    out.tle_src  = g_src;
    out.n_loaded = (int)g_sats.size();
    if(g_sats.empty()){ out.error = "catalogue empty"; return false; }
    if(trk.pts.size() < 6){ out.error = "track too short"; return false; }

    const double t_tca = trk.fit.valid ? trk.fit.t_tca_utc
                                       : 0.5*(trk.t_start_utc + trk.t_end_utc);
    out.tle_age_days = catalogue_age_days(t_tca);

    const double t0 = trk.t_start_utc - P.win_pad_s;
    const double t1 = trk.t_end_utc   + P.win_pad_s;
    const double W  = t1 - t0;
    const double dur_obs = trk.t_end_utc - trk.t_start_utc;
    const double rdot_obs = observed_rdot_max(trk);
    const double el_min_rad = P.el_min_deg * D2R;
    const double phi_obs = std::fabs(obs.lat_deg);

    double r_site[3];
    site_ecef(obs.lat_deg, obs.lon_east_deg, obs.alt_km, r_site);
    double u_site[3];
    {
        const double m = std::sqrt(r_site[0]*r_site[0]+r_site[1]*r_site[1]+r_site[2]*r_site[2]);
        u_site[0]=r_site[0]/m; u_site[1]=r_site[1]/m; u_site[2]=r_site[2]/m;
    }

    const auto tstart = std::chrono::steady_clock::now();
    out.n_stage[0] = (int)g_sats.size();

    // ── 0단계: 대수적, 전파 0회 ──────────────────────────────────────────
    std::vector<Work> w;
    w.reserve(g_sats.size());
    for(size_t i = 0; i < g_sats.size(); i++){
        const TleElem& s = g_sats[i];
        const double n = s.satrec.no_kozai / 60.0;            // rad/min → rad/s
        if(!(n > 0)) continue;
        const double a = s.semi_major_km;
        if(!(a > A_E_KM)) continue;
        const double incl = s.satrec.inclo * R2D;

        // (i) 도플러 스팬 도달성: 볼 수 있는 최대 시선속도는 궤도속도 + 지구자전 성분.
        //     GEO 는 2*R_e*n 이 1.1 km/s 뿐이라 LEO 관측(12-15 km/s 스팬)에서 즉사한다.
        const double v_orb = n * a;
        if(v_orb + OMEGA_E*A_E_KM < rdot_obs*0.8) continue;

        // (ii) 최대 패스 길이 vs 관측 지속. 550km Starlink 는 780초가 상한이라
        //      900초 트랙이면 1만개가 여기서 죽는다.
        const double c = A_E_KM/a;
        if(c >= 1.0) continue;
        const double T = (2.0*M_PI/n)/M_PI * std::acos(c) * 1.1;
        if(T < dur_obs) continue;

        // (iii) 위도 도달성: 관측점 위도가 궤도경사 + 가시캡 반각 밖이면 절대 안 보인다.
        const double lam = lambda_cap_rad(a, el_min_rad) * R2D;
        if(phi_obs > incl + lam + 1.0) continue;

        w.push_back({(int)i, a, n, incl, 1e9, 0});
    }
    out.n_stage[1] = (int)w.size();

    // GMST 는 시각에만 의존하므로 격자를 한 번 계산해 위성마다 재사용한다
    // (17점 x 3900위성 = 6.6만회 gstime 절약).
    const int NG = 17;
    double gt[NG], gcs[NG], gsn[NG];
    for(int k = 0; k < NG; k++){
        gt[k]  = t0 + W*(double)k/(double)(NG-1);
        const double g = SGP4Funcs::gstime_SGP4(jd_from_unix(gt[k]));
        gcs[k] = std::cos(g); gsn[k] = std::sin(g);
    }
    const double gtca = SGP4Funcs::gstime_SGP4(jd_from_unix(t_tca));
    const double ctca = std::cos(gtca), stca = std::sin(gtca);

    // SGP4 는 satrec 을 변조한다. 근지구(method=='n')는 satrec.t/error 만 쓰므로
    // **위성당 1회 복사 후 시간 샘플 간 재사용**한다 (20만회 복사 → 1.5만회).
    auto prop = [&](const TleElem& s, elsetrec& scratch, double t,
                    double rt[3], double vt[3]) -> bool {
        const double tsince = (jd_from_unix(t) - (s.satrec.jdsatepoch + s.satrec.jdsatepochF))*1440.0;
        if(!SGP4Funcs::sgp4(scratch, tsince, rt, vt)) return false;
        return scratch.error == 0;
    };

    // ── 2a단계: t_tca 1회 탐침 ───────────────────────────────────────────
    // **고도(elevation)를 격자 게이트로 쓰면 안 된다** — 스치는 패스가 임계를 얼마나
    // 짧게 넘을지 상한이 없어 놓칠 수 있다. 지심각 gamma 는 변화율이 (n+omega_e) 로
    // 엄격히 상한이라 스텝 팽창 후 엄밀하다. 35deg = max(n+omega_e)*(W/2).
    std::vector<Work> w2; w2.reserve(w.size()/3+8);
    {
        const double dilate = 35.0;
        double rt[3], vt[3], re[3], ve[3];
        for(Work& q : w){
            if(cancel && cancel()) return false;
            const TleElem& s = g_sats[q.idx];
            elsetrec sc = s.satrec;
            if(!prop(s, sc, t_tca, rt, vt)) continue;
            teme_to_ecef_cs(rt, vt, ctca, stca, re, ve);
            double rg;
            const double g = gamma_rad(u_site, re, rg)*R2D;
            const double lam = lambda_cap_rad(rg, el_min_rad)*R2D;
            if(g < lam + dilate) w2.push_back(q);
        }
    }
    out.n_stage[2] = (int)w2.size();
    if(prog) prog(0.3f, "geometry");

    // ── 2b단계: 격자 최소 gamma ──────────────────────────────────────────
    std::vector<Work> w3; w3.reserve(w2.size()/2+8);
    {
        double rt[3], vt[3], re[3], ve[3];
        for(Work& q : w2){
            if(cancel && cancel()) return false;
            const TleElem& s = g_sats[q.idx];
            elsetrec sc = s.satrec;
            double gmin = 1e9, tmin = t_tca, lam_at = 0;
            for(int k = 0; k < NG; k++){
                if(!prop(s, sc, gt[k], rt, vt)) { gmin = 1e9; break; }
                teme_to_ecef_cs(rt, vt, gcs[k], gsn[k], re, ve);
                double rg;
                const double g = gamma_rad(u_site, re, rg)*R2D;
                if(g < gmin){ gmin = g; tmin = gt[k]; lam_at = lambda_cap_rad(rg, el_min_rad)*R2D; }
            }
            if(gmin > 1e8) continue;
            // 반스텝 팽창: gamma 변화율 상한 (n+omega_e) * (스텝/2)
            const double half = 0.5*W/(double)(NG-1);
            const double dil = (q.n_rad_s + OMEGA_E)*half*R2D;
            if(gmin < lam_at + dil){ q.gamma_min = gmin; q.t_gmin = tmin; w3.push_back(q); }
        }
    }
    out.n_stage[3] = (int)w3.size();
    if(prog) prog(0.6f, "geometry");

    // ── 2c단계: 최근접 정밀화 + 실제 최대고도 게이트 ─────────────────────
    struct Pass { int idx; double t_ca; double max_el; LookAngle la; };
    std::vector<Pass> w4; w4.reserve(w3.size());
    {
        double rt[3], vt[3], re[3], ve[3];
        const double step = W/(double)(NG-1);
        for(Work& q : w3){
            if(cancel && cancel()) return false;
            const TleElem& s = g_sats[q.idx];
            elsetrec sc = s.satrec;
            double bt = q.t_gmin, bel = -90; LookAngle bla{};
            for(int it = 0; it < 6; it++){
                const double h = step/std::pow(2.0, it);
                for(int d = -1; d <= 1; d++){
                    const double t = bt + d*h;
                    if(t < t0 - 60 || t > t1 + 60) continue;
                    if(!prop(s, sc, t, rt, vt)) continue;
                    teme_to_ecef(rt, vt, jd_from_unix(t), re, ve);
                    LookAngle L;
                    ecef_look(re, ve, r_site, obs.lat_deg, obs.lon_east_deg, L);
                    if(L.el_deg > bel){ bel = L.el_deg; bt = t; bla = L; }
                }
            }
            if(bel >= P.el_min_deg) w4.push_back({q.idx, bt, bel, bla});
        }
    }
    out.n_stage[4] = (int)w4.size();
    if(prog) prog(0.8f, "doppler");

    // ── 3단계: 도플러 적합 ───────────────────────────────────────────────
    // f_obs = f0*(1 - rdot/c). |rdot|/c <= 2.5e-5 이므로 둘째 항에만 f0->f0_est 를
    // 대입하면(상대오차 1e-4 x 20kHz = 2Hz) 모델이 f0 에 대해 **단위기울기**로 붕괴한다.
    //   g_k = -f0_est*rdot_k/c,  y_k = f_obs_k - cf
    //   offset = mean(y-g),  r = (y-g) - offset
    // 즉 잔차가 곧 관측·예측 도플러 곡선의 **순수 형상 차이**다. 폐형해, 1패스, 탐색 없음.
    const double f0_est = trk.fit.valid ? trk.fit.f_center_hz
                                        : 0.5*(trk.f_min_hz + trk.f_max_hz);
    const double cf = (double)trk.center_freq_hz;

    // 64점으로 데시메이트 (전 점을 쓰면 위성당 수천 회 SGP4)
    std::vector<const Doppler::TrackPoint*> sub;
    {
        const size_t n = trk.pts.size();
        const size_t want = std::min<size_t>(64, n);
        for(size_t k = 0; k < want; k++) sub.push_back(&trk.pts[n*k/want]);
    }

    std::vector<Cand> cands;
    {
        double rt[3], vt[3], re[3], ve[3];
        for(Pass& pz : w4){
            if(cancel && cancel()) return false;
            const TleElem& s = g_sats[pz.idx];
            elsetrec sc = s.satrec;
            std::vector<double> y, g;
            y.reserve(sub.size()); g.reserve(sub.size());
            int inview = 0;
            bool ok = true;
            for(const Doppler::TrackPoint* p : sub){
                if(!prop(s, sc, p->t_utc, rt, vt)){ ok = false; break; }
                teme_to_ecef(rt, vt, jd_from_unix(p->t_utc), re, ve);
                LookAngle L;
                ecef_look(re, ve, r_site, obs.lat_deg, obs.lon_east_deg, L);
                if(L.el_deg > 0) inview++;
                y.push_back(p->f_hz - cf);
                g.push_back(-f0_est * L.range_rate_km_s / C_KM_S);
            }
            if(!ok || y.size() < 6) continue;
            const double cov = (double)inview/(double)y.size();
            if(cov < 0.80) continue;

            // 가중치는 1. DSP 가 이미 SNR 게이트를 걸었고, 라벨 데이터 없이 가중 방식을
            // 튜닝하면 근거 없는 상수만 늘어난다.
            auto fit_rms = [&](const std::vector<char>& keep, double& off)->double{
                double sd = 0; int n = 0;
                for(size_t i = 0; i < y.size(); i++){ if(!keep[i]) continue; sd += (y[i]-g[i]); n++; }
                if(n < 4) return 1e300;
                off = sd/n;
                double ss = 0;
                for(size_t i = 0; i < y.size(); i++){ if(!keep[i]) continue;
                    const double r = (y[i]-g[i]) - off; ss += r*r; }
                return std::sqrt(ss/n);
            };
            std::vector<char> keep(y.size(), 1);
            double off = 0;
            double rms = fit_rms(keep, off);
            if(rms >= 1e299) continue;
            // Huber 트림 1회 — bin 홉/간섭 교차에 크게 강해진다.
            for(size_t i = 0; i < y.size(); i++)
                if(std::fabs((y[i]-g[i]) - off) > 3.0*rms) keep[i] = 0;
            rms = fit_rms(keep, off);
            if(rms >= 1e299) continue;

            const double f0_fit = cf + off;
            if(std::fabs(f0_fit - f0_est) > 50000.0) continue;
            if(f0_fit < cf - 0.5*(double)trk.sample_rate_hz ||
               f0_fit > cf + 0.5*(double)trk.sample_rate_hz) continue;

            Cand c;
            c.norad = s.catalog_num;
            c.name  = s.name;
            c.rms_hz = rms;
            c.rms_bins = (trk.bin_hz > 0) ? rms/trk.bin_hz : 0;
            c.f0_fit_hz = f0_fit;
            c.max_el_deg = pz.max_el;
            c.az_tca_deg = pz.la.az_deg;
            c.el_tca_deg = pz.la.el_deg;
            c.range_tca_km = pz.la.range_km;
            c.dtca_s = pz.t_ca - t_tca;
            c.cov = cov;
            c.incl_deg = s.satrec.inclo * R2D;
            c.alt_km = s.semi_major_km - A_E_KM;
            c.tle_age_days = jd_from_unix(t_tca) - (s.satrec.jdsatepoch + s.satrec.jdsatepochF);
            // 기울기 오차: TCA 에서 df/dt = f0*v_t^2/(c*rho). 같은 궤도면 위성을 가르는
            // 유일한 지표라 따로 보고한다.
            if(trk.fit.valid && trk.fit.max_slope_hz_s > 0 && pz.la.range_km > 0){
                const double pred = f0_est*(V_LEO_KM_S*V_LEO_KM_S)/(C_KM_S*pz.la.range_km);
                c.slope_err_pct = 100.0*(pred - trk.fit.max_slope_hz_s)/trk.fit.max_slope_hz_s;
            }
            cands.push_back(std::move(c));
        }
    }

    std::sort(cands.begin(), cands.end(),
              [](const Cand& a, const Cand& b){ return a.rms_hz < b.rms_hz; });
    if((int)cands.size() > P.max_cand) cands.resize(P.max_cand);
    for(size_t i = 0; i < cands.size(); i++)
        cands[i].sep = (cands[0].rms_hz > 0) ? cands[i].rms_hz/cands[0].rms_hz : 0.0;
    out.cands = std::move(cands);
    out.ms = std::chrono::duration<double,std::milli>(
                 std::chrono::steady_clock::now()-tstart).count();
    if(prog) prog(1.0f, "done");
    return true;
}

// ── JSONL 아카이브 ──────────────────────────────────────────────────────────
static std::string store_path(){
    std::string d = BEWEPaths::data_dir() + "/modules/doppler";
    ::mkdir((BEWEPaths::data_dir()+"/modules").c_str(), 0755);
    ::mkdir(d.c_str(), 0755);
    time_t now = time(nullptr);
    struct tm tv{}; KST::to_tm(now, tv);
    char b[32]; snprintf(b, sizeof b, "%04d%02d%02d", tv.tm_year+1900, tv.tm_mon+1, tv.tm_mday);
    return d + "/doppler_" + b + ".jsonl";
}
static std::string kst_iso(double t){
    time_t s = (time_t)t; struct tm tv{}; KST::to_tm(s, tv);
    char b[40];
    snprintf(b, sizeof b, "%04d-%02d-%02dT%02d:%02d:%02d+09:00",
             tv.tm_year+1900, tv.tm_mon+1, tv.tm_mday, tv.tm_hour, tv.tm_min, tv.tm_sec);
    return b;
}
static std::string base_name(const std::string& p){
    const size_t s = p.rfind('/');
    return (s == std::string::npos) ? p : p.substr(s+1);
}

bool archive_scan(const Doppler::Candidate& trk, const Obs& obs, const Params& P,
                  const Result& r){
    FILE* f = fopen(store_path().c_str(), "ab");
    if(!f) return false;
    fprintf(f, "{\"kind\":\"scan\",\"ts\":\"%s\",\"ver\":1,", kst_iso((double)time(nullptr)).c_str());
    fprintf(f, "\"src\":{\"file\":\"%s\",\"cf_hz\":%llu,\"sr_hz\":%llu,\"fft\":%u,"
               "\"bin_hz\":%.3f,\"row_rate\":%.4f},",
            base_name(trk.file_path).c_str(),
            (unsigned long long)trk.center_freq_hz, (unsigned long long)trk.sample_rate_hz,
            trk.fft_size, trk.bin_hz, trk.row_rate_hz);
    // lon_e 를 **동경 양수 이름으로** 쓴다 — 서경 함정이 아카이브까지 번지지 않게.
    fprintf(f, "\"obs\":{\"station\":\"%s\",\"lat\":%.5f,\"lon_e\":%.5f,\"alt_km\":%.1f},",
            obs.station.c_str(), obs.lat_deg, obs.lon_east_deg, obs.alt_km);
    fprintf(f, "\"track\":{\"id\":%u,\"t_start\":\"%s\",\"t_end\":\"%s\",\"t_tca\":\"%s\","
               "\"n_pts\":%zu,\"f0_est_hz\":%.1f,\"span_hz\":%.1f,\"snr_med_db\":%.1f,"
               "\"tau_s\":%.1f,\"max_slope_hz_s\":%.2f,\"score\":%.3f},",
            trk.id, kst_iso(trk.t_start_utc).c_str(), kst_iso(trk.t_end_utc).c_str(),
            kst_iso(trk.fit.t_tca_utc).c_str(), trk.pts.size(), trk.fit.f_center_hz,
            trk.f_max_hz - trk.f_min_hz, trk.snr_med_db, trk.fit.tau_s,
            trk.fit.max_slope_hz_s, trk.score);
    fprintf(f, "\"tle\":{\"src\":\"%s\",\"age_days\":%.2f,\"n_loaded\":%d},",
            r.tle_src.c_str(), r.tle_age_days, r.n_loaded);
    fprintf(f, "\"search\":{\"el_min_deg\":%.1f,\"win_pad_s\":%.0f,"
               "\"n_stage\":[%d,%d,%d,%d,%d],\"ms\":%.0f},",
            P.el_min_deg, P.win_pad_s,
            r.n_stage[0], r.n_stage[1], r.n_stage[2], r.n_stage[3], r.n_stage[4], r.ms);
    fprintf(f, "\"cand\":[");
    const int n = std::min<int>(5, (int)r.cands.size());
    for(int i = 0; i < n; i++){
        const Cand& c = r.cands[i];
        // 이름에 큰따옴표가 있으면 JSON 이 깨진다 — 최소 이스케이프.
        std::string nm; for(char ch : c.name){ if(ch=='"'||ch=='\\') nm += '_'; else nm += ch; }
        fprintf(f, "%s{\"rank\":%d,\"norad\":%d,\"name\":\"%s\",\"rms_hz\":%.1f,"
                   "\"rms_bins\":%.3f,\"sep\":%.2f,\"f0_fit_hz\":%.1f,\"max_el_deg\":%.1f,"
                   "\"az_tca_deg\":%.1f,\"el_tca_deg\":%.1f,\"range_tca_km\":%.1f,"
                   "\"dtca_s\":%.1f,\"slope_err_pct\":%.1f,\"cov\":%.2f,"
                   "\"incl_deg\":%.2f,\"alt_km\":%.1f,\"tle_age_days\":%.2f}",
                i ? "," : "", i+1, c.norad, nm.c_str(), c.rms_hz, c.rms_bins, c.sep,
                c.f0_fit_hz, c.max_el_deg, c.az_tca_deg, c.el_tca_deg, c.range_tca_km,
                c.dtca_s, c.slope_err_pct, c.cov, c.incl_deg, c.alt_km, c.tle_age_days);
    }
    fprintf(f, "]}\n");
    fclose(f);
    return true;
}

bool archive_confirm(const Doppler::Candidate& trk, const Obs& obs, const Cand& pick, int rank){
    FILE* f = fopen(store_path().c_str(), "ab");
    if(!f) return false;
    std::string nm; for(char ch : pick.name){ if(ch=='"'||ch=='\\') nm += '_'; else nm += ch; }
    fprintf(f, "{\"kind\":\"confirm\",\"ts\":\"%s\",\"ver\":1,\"op\":\"%s\","
               "\"src\":{\"file\":\"%s\"},\"track\":{\"id\":%u,\"t_tca\":\"%s\"},"
               "\"pick\":{\"norad\":%d,\"name\":\"%s\",\"rank\":%d,\"rms_hz\":%.1f,"
               "\"f0_fit_hz\":%.1f}}\n",
            kst_iso((double)time(nullptr)).c_str(), obs.station.c_str(),
            base_name(trk.file_path).c_str(), trk.id, kst_iso(trk.fit.t_tca_utc).c_str(),
            pick.norad, nm.c_str(), rank, pick.rms_hz, pick.f0_fit_hz);
    fclose(f);
    return true;
}

} // namespace DopplerMatch
