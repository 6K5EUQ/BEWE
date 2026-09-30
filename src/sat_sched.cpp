#include "sat_sched.hpp"
#include "sat_tle.hpp"
#include "doppler/sat_geom.hpp"
#include "fft_viewer.hpp"
#include "bewe_paths.hpp"
#include "kst_time.hpp"
#include <mutex>
#include <map>
#include <set>
#include <cstdio>
#include <cstring>
#include <algorithm>

namespace SatSched {

static std::mutex        g_mtx;
static std::vector<Rule> g_rules;
static std::string       g_path;
static bool              g_force = true;   // 다음 tick 에서 즉시 재계산

std::string target_of(int norad){
    char b[32]; snprintf(b, sizeof b, "SAT %d", norad);
    return b;
}

static void save_locked(){
    if(g_path.empty()) return;
    std::string tmp = g_path + ".tmp";
    FILE* f = fopen(tmp.c_str(), "w");
    if(!f) return;
    for(const auto& r : g_rules)
        fprintf(f, "%d %.0f %u\n", r.norad, r.cf_hz, r.sr_hz);
    fclose(f);
    rename(tmp.c_str(), g_path.c_str());
}

void load(const std::string& station){
    std::string s = station.empty() ? "default" : station;
    for(char& c : s) if(c=='/' || c=='\\') c='_';
    std::lock_guard<std::mutex> lk(g_mtx);
    g_path = BEWEPaths::data_dir() + "/sat_sched_" + s + ".txt";
    g_rules.clear();
    FILE* f = fopen(g_path.c_str(), "r");
    if(!f) return;
    Rule r;
    while(fscanf(f, "%d %lf %u", &r.norad, &r.cf_hz, &r.sr_hz) == 3)
        if(r.norad > 0 && r.cf_hz > 0 && r.sr_hz > 0) g_rules.push_back(r);
    fclose(f);
    bewe_log_push(0, "[SAT] %d rule(s) restored\n", (int)g_rules.size());
    g_force = true;
}

bool add_rule(const Rule& r, std::string& err){
    if(r.norad <= 0 || r.norad > 99999){ err = "invalid NORAD"; return false; }
    if(r.cf_hz < 1e5 || r.cf_hz > 6e9){ err = "invalid CF (Hz)"; return false; }
    if(r.sr_hz < 100000 || r.sr_hz > 61440000){ err = "invalid SR (Hz, 0.1~61.44 MSPS)"; return false; }
    std::lock_guard<std::mutex> lk(g_mtx);
    auto it = std::find_if(g_rules.begin(), g_rules.end(),
                           [&](const Rule& x){ return x.norad == r.norad; });
    if(it != g_rules.end()) *it = r; else g_rules.push_back(r);
    save_locked();
    g_force = true;
    return true;
}

bool del_rule(int norad){
    std::lock_guard<std::mutex> lk(g_mtx);
    auto it = std::find_if(g_rules.begin(), g_rules.end(),
                           [&](const Rule& x){ return x.norad == norad; });
    if(it == g_rules.end()) return false;
    g_rules.erase(it);
    save_locked();
    g_force = true;
    return true;
}

std::vector<Rule> rules(){
    std::lock_guard<std::mutex> lk(g_mtx);
    return g_rules;
}

void refresh(){
    std::lock_guard<std::mutex> lk(g_mtx);
    g_force = true;
}

// ── 궤도원소 ────────────────────────────────────────────────────────────────
static bool find_tle(int norad, TleElem& e, std::string& src){
    for(const char* pre : {"leo", "all"}){
        std::string p = TleCache::newest(pre);
        if(!p.empty() && tle_find(p, norad, e)){ src = p; return true; }
    }
    return false;
}

bool find_elem(int norad, std::string& name, std::string& src_file, double& epoch_age_days,
               std::string& err){
    TleElem e;
    if(!find_tle(norad, e, src_file)){
        err = TleCache::newest("leo").empty() && TleCache::newest("all").empty()
            ? "no TLE snapshot - run /tle update" : "NORAD not in TLE snapshot";
        return false;
    }
    name = e.name;
    epoch_age_days = SatGeom::jd_from_unix((double)time(nullptr))
                   - (e.satrec.jdsatepoch + e.satrec.jdsatepochF);
    return true;
}

// ── 패스 예측 ──────────────────────────────────────────────────────────────
static bool elev_at(const elsetrec& rec, const double site[3], double lat, double lon_e,
                    double t, double& el){
    elsetrec s = rec;   // SGP4 가 satrec 을 변조한다 (심우주는 적분기 상태까지)
    double rt[3], vt[3], jd;
    if(!SatGeom::prop_teme(s, t, rt, vt, jd)) return false;
    double re[3], ve[3];
    SatGeom::teme_to_ecef(rt, vt, jd, re, ve);
    SatGeom::LookAngle la;
    SatGeom::ecef_look(re, ve, site, lat, lon_e, la);
    el = la.el_deg;
    return true;
}

// el(lo) 와 el(hi) 의 부호가 다를 때 0° 교차를 1초 해상도로.
static time_t cross(const elsetrec& rec, const double site[3], double lat, double lon_e,
                    time_t lo, time_t hi, bool rising){
    while(hi - lo > 1){
        time_t mid = lo + (hi - lo)/2;
        double el = 0;
        if(!elev_at(rec, site, lat, lon_e, (double)mid, el)) break;
        if((el > 0) == rising) hi = mid; else lo = mid;
    }
    return hi;
}

bool predict(int norad, double lat, double lon_e, time_t t0, time_t t1,
             std::vector<Pass>& out, std::string& err){
    out.clear();
    TleElem e; std::string src;
    if(!find_tle(norad, e, src)){ err = "NORAD not in TLE snapshot"; return false; }
    double site[3];
    SatGeom::site_ecef(lat, lon_e, 0.0, site);

    // 20초 격자 — 저궤도 패스는 최소 수 분이라 교차를 놓치지 않는다
    // (고도 수 도짜리 스치는 패스만 빠질 수 있다).
    constexpr time_t STEP = 20;
    double el = 0;
    if(!elev_at(e.satrec, site, lat, lon_e, (double)t0, el)){ err = "SGP4 propagation failed"; return false; }
    bool in = el > 0;
    Pass cur; cur.aos = t0; cur.max_el = el;
    time_t prev = t0;
    for(time_t t = t0 + STEP; t <= t1; t += STEP){
        if(!elev_at(e.satrec, site, lat, lon_e, (double)t, el)){ err = "SGP4 propagation failed"; return false; }
        if(!in && el > 0){
            in = true;
            cur.aos = cross(e.satrec, site, lat, lon_e, prev, t, true);
            cur.max_el = el;
        } else if(in && el <= 0){
            in = false;
            cur.los = cross(e.satrec, site, lat, lon_e, prev, t, false);
            out.push_back(cur);
        } else if(in){
            cur.max_el = std::max(cur.max_el, el);
        }
        prev = t;
    }
    if(in && cur.aos == t0 && out.empty()){ err = "never sets in window (GEO?)"; return false; }
    // 창 끝에서 진행 중인 패스는 버린다 — 다음 재계산에서 온전히 잡힌다.
    return true;
}

// ── 예약 엔트리 채우기 ─────────────────────────────────────────────────────
static std::string kst_hms(time_t t){
    struct tm k; KST::to_tm(t, k);
    char b[32]; strftime(b, sizeof b, "%m-%d %H:%M:%S", &k);
    return b;
}

void tick(FFTViewer& v){
    static time_t last = 0;
    static std::set<std::pair<int,time_t>> reported;   // 같은 거부를 매번 찍지 않는다
    const time_t now = time(nullptr);
    std::vector<Rule> rs;
    {
        std::lock_guard<std::mutex> lk(g_mtx);
        if(!g_force && now - last < 600) return;
        g_force = false;
        rs = g_rules;
    }
    last = now;

    const double lat   = v.station_lat;
    const double lon_e = -v.station_lon;   // 내부 규약: 서경 양수
    constexpr time_t HORIZON = 86400;
    const time_t earliest = now + (time_t)FFTViewer::SCHED_PRE_ARM_SEC + 2;

    bool changed = false;
    {
        // 치우기: 규칙이 사라진 대기 엔트리 + 끝난 지 1시간 넘은 위성 엔트리.
        // 활성 엔트리 인덱스가 밀리지 않게 새로 짠다 (Central 복원 경로와 같은 방식).
        std::lock_guard<std::mutex> lk(v.sched_mtx);
        std::vector<FFTViewer::SchedEntry> keep;
        int new_active = -1;
        for(int i = 0; i < (int)v.sched_entries.size(); i++){
            const auto& e = v.sched_entries[i];
            if(i == v.sched_active_idx){ new_active = (int)keep.size(); keep.push_back(e); continue; }
            if(strncmp(e.target, "SAT ", 4) == 0){
                int n = atoi(e.target + 4);
                bool has_rule = std::any_of(rs.begin(), rs.end(), [&](const Rule& r){ return r.norad == n; });
                bool ended = (e.status == FFTViewer::SchedEntry::DONE || e.status == FFTViewer::SchedEntry::FAILED)
                           && e.start_time + (time_t)e.duration_sec < now - 3600;
                if((!has_rule && e.status == FFTViewer::SchedEntry::WAITING) || ended){ changed = true; continue; }
            }
            keep.push_back(e);
        }
        v.sched_entries = std::move(keep);
        v.sched_active_idx = new_active;
    }

    for(const auto& r : rs){
        std::vector<Pass> ps; std::string err;
        if(!predict(r.norad, lat, lon_e, now, now + HORIZON, ps, err)){
            if(reported.insert({r.norad, 0}).second)
                bewe_log_push(0, "[SAT] %d: %s\n", r.norad, err.c_str());
            continue;
        }
        reported.erase({r.norad, 0});
        const std::string tgt = target_of(r.norad);
        for(const auto& p : ps){
            time_t start = std::max(p.aos, earliest);
            if(p.los - start < 10) continue;
            float dur = (float)(p.los - start);
            std::lock_guard<std::mutex> lk(v.sched_mtx);
            bool dup = std::any_of(v.sched_entries.begin(), v.sched_entries.end(),
                [&](const FFTViewer::SchedEntry& e){
                    return tgt == e.target
                        && e.start_time < p.los && p.aos < e.start_time + (time_t)e.duration_sec;
                });
            if(dup) continue;
            const char* why = nullptr;
            if(v.sched_has_overlap(start, dur))                       why = "overlaps another schedule";
            else if((int)v.sched_entries.size() >= MAX_SCHED_ENTRIES) why = "schedule list full";
            if(why){
                if(reported.insert({r.norad, p.aos}).second)
                    bewe_log_push(0, "[SAT] %d pass %s skipped: %s\n",
                                  r.norad, kst_hms(p.aos).c_str(), why);
                continue;
            }
            FFTViewer::SchedEntry e;
            e.start_time   = start;
            e.duration_sec = dur;
            e.freq_mhz     = (float)(r.cf_hz / 1e6);
            e.sr_hz        = r.sr_hz;
            e.bw_khz       = r.sr_hz / 1000.0f;   // sr_hz 를 모르는 구 Central 복원 시 폴백 폭
            e.status       = FFTViewer::SchedEntry::WAITING;
            strncpy(e.operator_name, "SAT", sizeof(e.operator_name)-1);
            strncpy(e.target, tgt.c_str(), sizeof(e.target)-1);
            v.sched_entries.push_back(e);
            changed = true;
            bewe_log_push(0, "[SAT] %d pass AOS %s LOS %s KST (%ds, max %.1f\xC2\xB0) scheduled\n",
                          r.norad, kst_hms(p.aos).c_str(), kst_hms(p.los).c_str(),
                          (int)(p.los - p.aos), p.max_el);
        }
    }
    if(changed) v.broadcast_sched_list();
}

} // namespace SatSched
