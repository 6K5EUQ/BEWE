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
            Doppler::score_candidate(tracks[i], tracks[i].reject_reason, P.sens);
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
            if(DopplerMatch::match(t, obs, MP, r, progress, cancelled))
                matches[t.id] = std::move(r);
        }
        if(g_cancel){ err = "cancelled"; break; }

        // ── 같은 패스를 가리키는 중복 트랙 버리기 ────────────────────────
        // 대역폭이 있는 신호는 위/아래 엣지가 각각 트랙으로 잡힌다. 피크 분리폭이
        // 메인로브에서 유도되므로(실측 732 Hz) 그보다 넓게 벌어진 엣지는 정당하게
        // 별개 피크가 된다 — 추출 단계에서 막을 수 있는 게 아니다.
        // 실측(2026-07-29 DGS-2 464.5MHz): 두 트랙이 4.2 kHz 떨어져 나란히 갔고
        // 둘 다 MALLIGYONG-1 을 1위로 냈다. 시간이 겹치면서 같은 위성을 가리키면
        // 물리적으로 같은 패스이므로, 운용자에게는 신호 하나다.
        //
        // 대표는 score 높은 쪽, 동점이면 곡선 잔차가 작은 쪽, 그래도 같으면 앞선
        // 트랙. 전순서라 한 그룹에서 정확히 하나만 살아남는다 (서로 지우는 일 없음).
        auto better = [&](size_t x, size_t y){
            if(tracks[x].score != tracks[y].score) return tracks[x].score > tracks[y].score;
            const double rx = tracks[x].fit.rms_resid_hz, ry = tracks[y].fit.rms_resid_hz;
            if(rx != ry) return rx < ry;
            return x < y;
        };
        auto top_norad = [&](size_t i)->int{
            if(tracks[i].score <= 0.0f) return 0;
            auto it = matches.find(tracks[i].id);
            if(it == matches.end() || it->second.cands.empty()) return 0;
            return it->second.cands.front().norad;
        };
        std::vector<char> drop(tracks.size(), 0);
        for(size_t a = 0; a < tracks.size(); a++){
            const int na = top_norad(a);
            if(na == 0) continue;
            for(size_t b = 0; b < tracks.size(); b++){
                if(b == a || top_norad(b) != na) continue;
                const double ov = std::min(tracks[a].t_end_utc,   tracks[b].t_end_utc)
                                - std::max(tracks[a].t_start_utc, tracks[b].t_start_utc);
                if(ov <= 0.0) continue;      // 안 겹치면 같은 위성의 다른 패스다
                if(better(b, a)){ drop[a] = 1; break; }
            }
        }
        {
            std::vector<Doppler::Candidate> keep;
            keep.reserve(tracks.size());
            for(size_t i = 0; i < tracks.size(); i++){
                if(drop[i]){ matches.erase(tracks[i].id); continue; }
                keep.push_back(std::move(tracks[i]));
            }
            tracks.swap(keep);
        }
        // 살아남은 것만 아카이브에 남긴다.
        for(const auto& t : tracks){
            auto it = matches.find(t.id);
            if(it != matches.end()) DopplerMatch::archive_scan(t, obs, MP, it->second);
        }
        {
            std::lock_guard<std::mutex> lk(g_mtx);
            g_tracks = tracks;
            g_match  = std::move(matches);
        }
        g_ntracks = (uint32_t)tracks.size();
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

void start_burst(const HistReader& reader, const DopplerMatch::Params& MP,
                 const std::string& tle_dir, Doppler::Sensitivity sens){
    // 버스트(간헐 송신) 프리셋. 큐브샛 비콘은 주기 30~120초로 짧게 쏘아 duty 가 0.1 도
    // 안 되므로 기본 min_occupancy=0.55 로는 전멸한다. duty 대신 절대 점 개수로 거르고,
    // 갭에서 트랙이 끊기지 않게 게이트와 갭 한도를 연다.
    //
    // 실측(2026-08-07, 파일 4개): 트랙은 85->127 로 늘지만 위성 판정은 0/2/1/0 그대로다.
    // mono_frac 과 swing_ratio 가 방어선으로 버틴다 (145MHz 지상신호 실측 스윙 최대
    // 820 Hz vs 그 대역 위성 하한 1451 Hz — 자릿수가 갈린다).
    //
    // apply(sens) 를 burst 필드 설정 *전에* 호출 — Normal/Strict 는 min_dur_s/
    // min_occupancy 를 burst 전용값으로 덮어써야 하므로 순서가 중요하다 (apply 가
    // min_occupancy=0.40/0.70 등으로 먼저 채우고, 아래서 0.05f 로 다시 덮는다).
    Doppler::ExtractParams P;
    P.apply(sens);
    P.burst         = true;
    P.max_gap_s     = 60.0;
    P.min_occupancy = 0.05f;
    P.min_points    = 12;
    P.max_tracks    = 256;      // 트랙이 늘어난다 — 상한이 막으면 뒤쪽을 통째로 놓친다
    start_full(reader, P, MP, tle_dir);
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

bool top_name_of(uint32_t track_id, std::string& out){
    std::lock_guard<std::mutex> lk(g_mtx);
    auto it = g_match.find(track_id);
    if(it == g_match.end() || it->second.cands.empty()) return false;
    out = it->second.cands.front().name;
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
