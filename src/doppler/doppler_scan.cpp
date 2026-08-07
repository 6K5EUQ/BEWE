#include "doppler_scan.hpp"
#include "doppler_extract.hpp"
#include "doppler_fit.hpp"
#include "../hist_reader.hpp"
#include <atomic>
#include <cstring>
#include <mutex>
#include <thread>
#include <unordered_map>

namespace DopplerScan {

namespace {

std::thread            g_th;
std::mutex             g_mtx;             // 결과 보호
std::atomic<bool>      g_cancel{false};
std::atomic<bool>      g_running{false};
std::atomic<float>     g_prog{0.0f};
std::atomic<int>       g_state{(int)State::Idle};
std::atomic<uint32_t>  g_ntracks{0};
char                   g_stage[32] = {0};
std::mutex             g_stage_mtx;
std::string            g_err;

std::vector<Doppler::Candidate>                     g_tracks;
std::unordered_map<uint32_t, DopplerMatch::Result>  g_match;

void set_stage(const char* s){
    std::lock_guard<std::mutex> lk(g_stage_mtx);
    snprintf(g_stage, sizeof g_stage, "%s", s ? s : "");
}

void join_prev(){
    if(g_th.joinable()){ g_cancel = true; g_th.join(); }
    g_cancel = false;
}

// 공통 몸통. reader 는 이미 워커 소유 사본이다.
void run_job(HistReader reader, Doppler::ExtractParams P, DopplerMatch::Params MP,
             std::string tle_dir, bool refine,
             uint32_t row_lo, uint32_t row_hi, uint32_t lin_lo, uint32_t lin_hi){
    auto cancelled = [](){ return g_cancel.load(); };
    auto progress  = [](float f, const char* s){ g_prog = f; set_stage(s); };

    std::vector<Doppler::Candidate> tracks;
    std::unordered_map<uint32_t, DopplerMatch::Result> matches;
    std::string err;

    do {
        Doppler::Calib C;
        set_stage("calibrate");
        if(!Doppler::calibrate(reader, P, C, progress, cancelled)){
            err = g_cancel ? "cancelled" : "calibration failed"; break;
        }
        if(g_cancel){ err = "cancelled"; break; }

        set_stage("extract");
        Doppler::ExtractStats st;
        if(refine){
            Doppler::Candidate c;
            if(Doppler::refine_in_box(reader, row_lo, row_hi, lin_lo, lin_hi, P, C, c,
                                      progress, cancelled))
                tracks.push_back(std::move(c));
        } else {
            Doppler::extract_tracks(reader, P, C, tracks, st, progress, cancelled);
        }
        if(g_cancel){ err = "cancelled"; break; }

        // 적합 + 점수. 통과 못한 트랙도 남긴다 — reject_reason 을 UI 가 보여준다.
        set_stage("fit");
        const double cf = (double)reader.hdr().center_freq_hz;
        for(size_t i = 0; i < tracks.size(); i++){
            tracks[i].id = (uint32_t)i;
            Doppler::fit_scurve(tracks[i].pts, cf, reader.bin_hz(),
                                reader.hdr().row_rate_hz, tracks[i].fit);
            Doppler::score_candidate(tracks[i], tracks[i].reject_reason);
        }
        {
            std::lock_guard<std::mutex> lk(g_mtx);
            g_tracks = tracks;
        }
        g_ntracks = (uint32_t)tracks.size();
        if(g_cancel){ err = "cancelled"; break; }

        // 점수 통과 트랙만 위성 매칭 — 거부된 트랙에 카탈로그를 돌릴 이유가 없다.
        bool any = false;
        for(const auto& t : tracks) if(t.score > 0.0f){ any = true; break; }
        if(!any) break;

        set_stage("tle");
        double ref = tracks.front().t_start_utc;
        for(const auto& t : tracks) if(t.score > 0.0f){ ref = t.fit.t_tca_utc; break; }
        std::string why;
        if(!DopplerMatch::load_catalogue(tle_dir, ref, MP.include_starlink, why)){ err = why; break; }
        if(g_cancel){ err = "cancelled"; break; }

        set_stage("match");
        DopplerMatch::Obs obs;
        obs.lat_deg      = reader.station_lat();
        obs.lon_east_deg = reader.station_lon_east();
        obs.alt_km       = 0.0;
        obs.station.assign(reader.hdr().station_name,
                           strnlen(reader.hdr().station_name, sizeof(reader.hdr().station_name)));
        for(const auto& t : tracks){
            if(t.score <= 0.0f) continue;
            if(g_cancel){ err = "cancelled"; break; }
            DopplerMatch::Result r;
            if(DopplerMatch::match(t, obs, MP, r, progress, cancelled)){
                DopplerMatch::archive_scan(t, obs, MP, r);
                matches[t.id] = std::move(r);
            }
        }
        {
            std::lock_guard<std::mutex> lk(g_mtx);
            g_match = std::move(matches);
        }
    } while(false);

    {
        std::lock_guard<std::mutex> lk(g_mtx);
        g_err = err;
    }
    g_prog = 1.0f;
    g_state = (int)(g_cancel ? State::Cancelled
                             : (err.empty() ? State::Done : State::Failed));
    set_stage("");
    g_running = false;
}

} // anon

void start_full(const HistReader& reader, const Doppler::ExtractParams& P,
                const DopplerMatch::Params& MP, const std::string& tle_dir){
    join_prev();
    {
        std::lock_guard<std::mutex> lk(g_mtx);
        g_tracks.clear(); g_match.clear(); g_err.clear();
    }
    g_ntracks = 0; g_prog = 0.0f;
    g_state = (int)State::Running; g_running = true;
    g_th = std::thread(run_job, reader.clone_for_thread(), P, MP, tle_dir,
                       false, 0u, 0u, 0u, 0u);
}

void start_refine(const HistReader& reader, uint32_t row_lo, uint32_t row_hi,
                  uint32_t lin_lo, uint32_t lin_hi,
                  const Doppler::ExtractParams& P, const DopplerMatch::Params& MP,
                  const std::string& tle_dir){
    join_prev();
    {
        std::lock_guard<std::mutex> lk(g_mtx);
        g_tracks.clear(); g_match.clear(); g_err.clear();
    }
    g_ntracks = 0; g_prog = 0.0f;
    g_state = (int)State::Running; g_running = true;
    g_th = std::thread(run_job, reader.clone_for_thread(), P, MP, tle_dir,
                       true, row_lo, row_hi, lin_lo, lin_hi);
}

void cancel(){ g_cancel = true; }
bool busy(){ return g_running.load(); }

Status status(){
    Status s;
    s.st = (State)g_state.load();
    s.progress = g_prog.load();
    s.n_tracks = g_ntracks.load();
    { std::lock_guard<std::mutex> lk(g_stage_mtx); snprintf(s.stage, sizeof s.stage, "%s", g_stage); }
    { std::lock_guard<std::mutex> lk(g_mtx); s.err = g_err; }
    return s;
}

bool results(std::vector<Doppler::Candidate>& tracks){
    std::lock_guard<std::mutex> lk(g_mtx);
    if(g_tracks.empty()) return false;
    tracks = g_tracks;
    return true;
}

bool match_of(uint32_t track_id, DopplerMatch::Result& out){
    std::lock_guard<std::mutex> lk(g_mtx);
    auto it = g_match.find(track_id);
    if(it == g_match.end()) return false;
    out = it->second;
    return true;
}

void shutdown(){
    g_cancel = true;
    if(g_th.joinable()) g_th.join();
    g_cancel = false;
    g_running = false;
    g_state = (int)State::Idle;
    std::lock_guard<std::mutex> lk(g_mtx);
    g_tracks.clear(); g_match.clear(); g_err.clear();
}

} // namespace DopplerScan
