#include "hist_check.hpp"
#include "fft_viewer.hpp"
#include "central_client.hpp"
#include "long_waterfall.hpp"
#include "mission_push.hpp"
#include "net_protocol.hpp"
#include "bewe_paths.hpp"

#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <set>
#include <vector>
#include <string>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

extern void bewe_log_push(int col, const char* fmt, ...);

namespace HistCheck {
namespace {

FFTViewer*     g_v   = nullptr;
CentralClient* g_cli = nullptr;

std::atomic<bool>       g_running{false};
std::thread             g_thr;
std::mutex              g_q_mtx;
std::condition_variable g_q_cv;
std::deque<std::string> g_queue;          // 대조할 미션 hist dir 목록

// HIST_STAT 응답 대기 (req_id → 결과)
std::mutex              g_stat_mtx;
std::condition_variable g_stat_cv;
PktHistStat             g_stat{};
bool                    g_stat_have = false;
uint8_t                 g_req_id    = 0;

// 이미 세그먼트를 떼어 올린 (원본경로, 시작행) — 같은 구간 반복 export 방지.
std::mutex                          g_done_mtx;
std::set<std::pair<std::string,uint64_t>> g_exported;

// ── 경로 파싱: .../missions/<year>/<code>/hist/<file> ─────────────────────
bool parse_mission_path(const std::string& full, int& year, std::string& code){
    const std::string root = BEWEPaths::missions_root();
    if(full.compare(0, root.size(), root) != 0) return false;
    std::string rest = full.substr(root.size());       // /<year>/<code>/hist/<file>
    if(!rest.empty() && rest[0] == '/') rest.erase(0,1);
    size_t s1 = rest.find('/'); if(s1 == std::string::npos) return false;
    size_t s2 = rest.find('/', s1+1); if(s2 == std::string::npos) return false;
    year = atoi(rest.substr(0, s1).c_str());
    code = rest.substr(s1+1, s2-s1-1);
    return year > 1970 && !code.empty() && code.size() < 8;
}

struct LocalInfo {
    LongWaterfall::FileHeader h{};
    uint64_t rows  = 0;
    uint32_t width = 0;      // 행 바이트수
    off_t    size  = 0;
};

bool read_local(const std::string& path, LocalInfo& out){
    FILE* fp = fopen(path.c_str(), "rb");
    if(!fp) return false;
    size_t rd = fread(&out.h, 1, sizeof(out.h), fp);
    fclose(fp);
    if(rd != sizeof(out.h)) return false;
    if(memcmp(out.h.magic, "BWWF", 4) != 0) return false;
    // HOST 로컬은 항상 raw v3 (v4 압축은 Central 이 한다). v4 면 손댈 게 없다.
    if(out.h.version != LongWaterfall::FILE_VERSION) return false;
    struct stat st{};
    if(stat(path.c_str(), &st) != 0) return false;
    out.width = out.h.fft_input_size ? out.h.fft_input_size : out.h.fft_size;
    if(!out.width) return false;
    out.size = st.st_size;
    out.rows = ((uint64_t)st.st_size - sizeof(out.h)) / out.width;
    return out.rows > 0;
}

// Central 에 stat 질의 후 응답 대기. 반환 false = 무응답(구버전/단절) → 보존.
bool query_central(const char* station, int year, const char* code,
                   uint64_t start_utc, PktHistStat& out, int timeout_ms = 8000){
    if(!g_cli || !g_cli->is_central_connected()) return false;

    PktHistStatReq q{};
    snprintf(q.station, sizeof(q.station), "%s", station);
    q.year = (uint32_t)year;
    snprintf(q.code, sizeof(q.code), "%s", code);
    q.start_utc_unix = start_utc;
    {
        std::lock_guard<std::mutex> lk(g_stat_mtx);
        q.req_id = ++g_req_id; if(q.req_id == 0) q.req_id = ++g_req_id;
        g_stat_have = false;
    }

    // BEWE 패킷 조립 (magic + type + len + payload)
    std::vector<uint8_t> pkt(9 + sizeof(q));
    memcpy(pkt.data(), "BEWE", 4);
    pkt[4] = (uint8_t)PacketType::HIST_STAT_REQ;
    uint32_t plen = (uint32_t)sizeof(q);
    memcpy(pkt.data()+5, &plen, 4);
    memcpy(pkt.data()+9, &q, sizeof(q));
    g_cli->enqueue_relay_broadcast(pkt.data(), pkt.size(), /*no_drop=*/true);

    std::unique_lock<std::mutex> lk(g_stat_mtx);
    if(!g_stat_cv.wait_for(lk, std::chrono::milliseconds(timeout_ms),
                           [&]{ return g_stat_have && g_stat.req_id == q.req_id; }))
        return false;
    out = g_stat;
    return true;
}

// 빠진 구간 [from_row, rows) 를 독립 .bewehist 로 떼어낸다.
// 헤더는 원본을 복사하되 start_utc 를 그 구간 시작 시각으로 민다 — 그래야 Central 에
// 별개 파일로 들어가도 뷰어 시간축이 맞고, 다음 대조에서 원본과 구분된다.
bool export_segment(const std::string& src, const LocalInfo& li,
                    uint64_t from_row, std::string& out_path){
    if(from_row >= li.rows) return false;
    float rr = (li.h.row_rate_hz > 0.5f) ? li.h.row_rate_hz : 5.0f;
    uint64_t seg_start = li.h.start_utc_unix + (uint64_t)(from_row / rr);

    auto slash = src.find_last_of('/');
    std::string dir = (slash == std::string::npos) ? "." : src.substr(0, slash);
    char station[33]={}; memcpy(station, li.h.station_name, 32);
    std::string live = LongWaterfall::build_hist_filename_live(
                          seg_start, li.h.center_freq_hz,
                          (int)li.h.utc_offset_hours, station);
    uint64_t seg_end = li.h.start_utc_unix + (uint64_t)(li.rows / rr);
    std::string name = LongWaterfall::build_hist_filename_finalize(live, seg_end, 0);
    std::string full = dir + "/" + name;
    for(int n=2; access(full.c_str(), F_OK)==0 && n<100; ++n){
        auto p = name.rfind(".bewehist");
        if(p == std::string::npos) break;
        full = dir + "/" + name.substr(0,p) + "_" + std::to_string(n) + ".bewehist";
    }

    FILE* in = fopen(src.c_str(), "rb");
    if(!in) return false;
    FILE* on = fopen(full.c_str(), "wb");
    if(!on){ fclose(in); return false; }

    LongWaterfall::FileHeader h = li.h;
    h.start_utc_unix = seg_start;
    fwrite(&h, 1, sizeof(h), on);

    if(fseeko(in, (off_t)sizeof(h) + (off_t)(from_row * li.width), SEEK_SET) != 0){
        fclose(in); fclose(on); unlink(full.c_str()); return false;
    }
    std::vector<uint8_t> buf(256*1024);
    uint64_t remain = (li.rows - from_row) * li.width;
    bool ok = true;
    while(remain > 0){
        size_t want = (size_t)std::min<uint64_t>(remain, buf.size());
        size_t got  = fread(buf.data(), 1, want, in);
        if(got == 0){ ok = false; break; }
        if(fwrite(buf.data(), 1, got, on) != got){ ok = false; break; }
        remain -= got;
    }
    fclose(in); fclose(on);
    if(!ok){ unlink(full.c_str()); return false; }
    out_path = full;
    return true;
}

// 파일 하나 대조. 반환: 로그용 한 줄.
std::string check_one(const std::string& path, int year, const std::string& code){
    LocalInfo li;
    if(!read_local(path, li)) return path + ": (헤더 불량/빈 파일 — 건너뜀)";

    char station[33]={}; memcpy(station, li.h.station_name, 32);

    PktHistStat st{};
    if(!query_central(station, year, code.c_str(), li.h.start_utc_unix, st))
        return path + ": Central 무응답 — 보존";

    auto base = path.substr(path.find_last_of('/')+1);
    char buf[512];

    if(!st.found){
        // Central 에 아예 없다 → 통파일 업로드 (MissionPush 가 ACK 후 로컬 삭제).
        MissionPush::enqueue(path, MFS_HIST);
        snprintf(buf, sizeof(buf), "%s: Central 에 없음 → 통파일 업로드 (%llu행)",
                 base.c_str(), (unsigned long long)li.rows);
        return buf;
    }
    const uint64_t crows = st.num_rows;   // packed 필드 → 로컬 복사 (참조 바인딩 불가)
    if(crows >= li.rows){
        unlink(path.c_str());
        unlink((path + ".info").c_str());
        snprintf(buf, sizeof(buf), "%s: OK (Central %llu >= 로컬 %llu행) → 로컬 삭제",
                 base.c_str(), (unsigned long long)crows,
                 (unsigned long long)li.rows);
        return buf;
    }

    // 빠진 꼬리만 떼어 올린다. 원본은 다음 대조에서 커버가 증명될 때 지운다 —
    // 세그먼트 ACK 하나만 보고 지우면 업로드가 실패했을 때 유일본이 사라진다.
    {
        std::lock_guard<std::mutex> lk(g_done_mtx);
        if(g_exported.count({path, crows})){
            snprintf(buf, sizeof(buf), "%s: 세그먼트 업로드 진행 중 (%llu행부터) — 대기",
                     base.c_str(), (unsigned long long)crows);
            return buf;
        }
    }
    std::string seg;
    if(!export_segment(path, li, crows, seg)){
        snprintf(buf, sizeof(buf), "%s: 세그먼트 추출 실패 — 보존", base.c_str());
        return buf;
    }
    { std::lock_guard<std::mutex> lk(g_done_mtx); g_exported.insert({path, crows}); }
    MissionPush::enqueue(seg, MFS_HIST);
    snprintf(buf, sizeof(buf),
             "%s: 결손 %llu행 (Central %llu / 로컬 %llu) → 세그먼트 업로드 %s",
             base.c_str(), (unsigned long long)(li.rows - crows),
             (unsigned long long)crows, (unsigned long long)li.rows,
             seg.substr(seg.find_last_of('/')+1).c_str());
    return buf;
}

// 한 미션 hist dir 안의 보존된 .bewehist 전부 대조 (-LIVE 는 제외).
int check_dir(const std::string& dir, int year, const std::string& code, bool log){
    DIR* d = opendir(dir.c_str());
    if(!d) return 0;
    std::vector<std::string> files;
    while(struct dirent* e = readdir(d)){
        const char* n = e->d_name;
        if(n[0] == '.') continue;
        const char* dot = strrchr(n, '.');
        if(!dot || strcmp(dot, ".bewehist") != 0) continue;
        if(strstr(n, "-LIVE.bewehist")) continue;      // 녹화 중 파일은 건드리지 않음
        files.push_back(dir + "/" + n);
    }
    closedir(d);

    // 현재 열려 있는 파일은 절대 제외 (rotate 직후 경합 방지)
    std::string cur = LongWaterfall::current_file_path();
    int n_done = 0;
    for(const auto& f : files){
        if(!cur.empty() && f == cur) continue;
        std::string msg = check_one(f, year, code);
        if(log) bewe_log_push(0, "[HIST] %s\n", msg.c_str());
        else    printf("[HIST] %s\n", msg.c_str());
        n_done++;
    }
    return n_done;
}

void worker(){
    while(g_running.load()){
        std::string dir;
        {
            std::unique_lock<std::mutex> lk(g_q_mtx);
            g_q_cv.wait_for(lk, std::chrono::seconds(2),
                            [&]{ return !g_queue.empty() || !g_running.load(); });
            if(!g_running.load()) break;
            if(g_queue.empty()) continue;
            dir = g_queue.front(); g_queue.pop_front();
        }
        int year = 0; std::string code;
        if(!parse_mission_path(dir + "/x", year, code)) continue;
        check_dir(dir, year, code, /*log=*/true);
    }
}

} // anon

void start(FFTViewer* v, CentralClient* cli){
    g_v = v; g_cli = cli;
    if(g_running.exchange(true)) return;
    g_thr = std::thread(worker);
}

void stop(){
    if(!g_running.exchange(false)) return;
    g_q_cv.notify_all();
    if(g_thr.joinable()) g_thr.join();
}

bool is_active(){ return g_running.load(); }

void on_hist_stat(const uint8_t* bewe_pkt, size_t len){
    if(len < 9 + sizeof(PktHistStat)) return;
    std::lock_guard<std::mutex> lk(g_stat_mtx);
    memcpy(&g_stat, bewe_pkt + 9, sizeof(g_stat));
    g_stat_have = true;
    g_stat_cv.notify_all();
}

void notify_finalized(const std::string& path){
    if(!g_running.load()) return;
    auto slash = path.find_last_of('/');
    if(slash == std::string::npos) return;
    std::string dir = path.substr(0, slash);
    std::lock_guard<std::mutex> lk(g_q_mtx);
    for(const auto& q : g_queue) if(q == dir) return;   // 같은 dir 중복 큐잉 방지
    g_queue.push_back(dir);
    g_q_cv.notify_one();
}

void run_command(const char* args){
    // "/hist check [station] [code]" — 인자는 현재 무시하고 활성 미션 dir 을 대조한다.
    // (station 은 자기 기지 것만 로컬에 있으므로 실질 의미가 없다.)
    (void)args;
    if(!g_v){ printf("[HIST] viewer 없음\n"); return; }
    std::string dir = g_v->active_hist_dir();
    if(dir.empty()){
        printf("[HIST] 활성 미션 없음 — 대조할 대상 없음\n");
        return;
    }
    int year = 0; std::string code;
    if(!parse_mission_path(dir + "/x", year, code)){
        printf("[HIST] 미션 경로 파싱 실패: %s\n", dir.c_str());
        return;
    }
    printf("[HIST] check %04d/%s — %s\n", year, code.c_str(), dir.c_str());
    int n = check_dir(dir, year, code, /*log=*/false);
    if(n == 0) printf("[HIST] 보존된 파일 없음 (전부 Central 도달 증명됨)\n");
}

} // namespace HistCheck
