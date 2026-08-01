#include "long_waterfall.hpp"
#include "fft_viewer.hpp"
#include "bewe_paths.hpp"
#include "net_protocol.hpp"
#include "mission_push.hpp"

#include <atomic>
#include <thread>
#include <mutex>
#include <vector>
#include <string>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <algorithm>
#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>

namespace LongWaterfall {

namespace {

// ── State (file-scope, single host process) ──────────────────────────────
std::atomic<bool>   g_running{false};
std::atomic<int>    g_rotate_req_seq{0};   // ++ when external code calls request_rotate()
int                 g_rotate_seen_seq = 0; // worker tracks last seen
// 현재 열린 파일의 "Central 연결 안정성" 표시. LIVE row stream 이 끊긴 적이 있으면
// finalize 시 file push 로 보완. 연결 내내 안정이면 push 생략 (LIVE tap 만으로 충분).
// open_new_file 마다 false 로 reset. cli_host 의 reconnect 흐름이 mark_dirty() 호출.
std::atomic<bool>   g_file_dirty{false};
std::thread         g_thr;
FFTViewer*          g_v = nullptr;

// 업링크 드롭 카운터 (cli_host 가 CentralClient::drop_count 를 물린다).
// open_new_file 에서 기준값을 찍고, 파일이 열려 있는 동안 증가하면 dirty 로 승격한다.
std::mutex               g_drop_fn_mtx;
std::function<uint64_t()> g_drop_fn;
uint64_t                 g_drop_base = 0;

// 보존된 파일 finalize 알림 (HistCheck).
std::mutex                            g_retain_fn_mtx;
std::function<void(const std::string&)> g_retain_fn;

// Live broadcast callbacks (set by host wiring).
LiveCallbacks       g_live_cb;
std::mutex          g_live_cb_mtx;
// Snapshot of LIVE_START header for join-in-progress (cleared on close).
std::mutex          g_live_state_mtx;
PktLwfLiveStart     g_live_state{};
bool                g_live_state_valid = false;
uint32_t            g_live_row_idx = 0;

// Open file state — accessed only from worker thread except path read.
std::mutex          g_path_mtx;
std::string         g_cur_path;            // empty when no file open
FILE*               g_fp = nullptr;
FileHeader          g_hdr_cur{};

// 캡처 행 → 디스크 행 스테이징 버퍼. 캡처 스레드를 디스크 I/O 로 막지 않으려고
// data_mtx 안에서는 폴딩+양자화만 하고, fwrite 는 락을 놓은 뒤에 한다.
std::vector<uint8_t> g_stage;              // rows_staged * dst_fft 바이트
int                  g_stage_rows = 0;
// 한 번에 처리할 최대 행 수 — 스테이지 상한(=STAGE_MAX_ROWS*fft) 을 묶어둔다.
// 밀리면 다음 루프(20ms)에서 이어 처리하므로 6400 rows/s 까지 따라잡는다.
constexpr int        STAGE_MAX_ROWS = 128;

// Last seen capture index (absolute, not modulo).
int                 g_last_total_ffts = 0;

// 실측 캡처 행레이트 (1초 창). 헤더 row_rate_hz 는 이 값이어야 뷰어 시간축이 맞는다.
// FFTViewer::fft_row_rate_hz 를 쓰지 않는 이유: 그건 net_bcast_worker 가 재는데
// 그 워커는 NetServer 가 있는 HOST 에서만 돈다. LOCAL/GUI-LOCAL 은 0 으로 남아
// 헤더가 5Hz 라고 거짓말하게 된다. 여기서 직접 재면 모든 모드에서 맞다.
float               g_meas_row_rate = 0.f;

// ingest_new_rows 가 data_mtx 안에서 떠 두는 양자화 범위 스냅샷 (워커 스레드 전용).
// quant-rotate 검사가 이 값을 읽어 별도 data_mtx 획득을 없앤다.
float               g_seen_pmin = 0.f, g_seen_pmax = 0.f;

// ── Helpers ──────────────────────────────────────────────────────────────

void close_file_locked(){
    // Live broadcast STOP first (so JOIN closes its mirror file).
    PktLwfLiveStop stop_pkt{};
    bool had_state = false;
    uint32_t rows_snapshot = 0;
    {
        std::lock_guard<std::mutex> lk(g_live_state_mtx);
        if(g_live_state_valid){
            memcpy(stop_pkt.filename, g_live_state.filename, sizeof(stop_pkt.filename));
            had_state = true;
        }
        rows_snapshot = g_live_row_idx;
        g_live_state_valid = false;
        g_live_row_idx = 0;
    }
    if(had_state){
        LiveCallbacks cb_copy;
        { std::lock_guard<std::mutex> lk(g_live_cb_mtx); cb_copy = g_live_cb; }
        if(cb_copy.on_stop) cb_copy.on_stop(stop_pkt);
    }

    if(g_fp){ fflush(g_fp); fclose(g_fp); g_fp = nullptr; }

    // 조각 파일 discard — finalize 대신 버린다. 두 경우:
    //  (1) 0행 (header-only): 정각 rotate 에서 열자마자 닫힌 파일.
    //  (2) 짧은-수명 조각: 정각 rotate 로 연 파일이 스케줄 retune / Central 재연결의
    //      추가 rotate 로 정각 직후 몇십 초 만에 닫힌 경우(실측 12~77초, 예 0000-0001).
    //      종전 가드는 rows==0 만 잡아 몇십~수백 행 조각이 finalize 되어 남았다.
    //  정상 HIST 세그먼트는 정시분할이라 항상 수십 분 이상 → 절대 안 걸린다.
    {
        bool short_stub =
            hist_is_short_stub(g_hdr_cur.start_utc_unix, (uint64_t)time(nullptr));
        if(rows_snapshot == 0 || short_stub){
            std::string discard_path;
            { std::lock_guard<std::mutex> lk(g_path_mtx); discard_path = g_cur_path; g_cur_path.clear(); }
            if(!discard_path.empty()){
                printf("[LongWaterfall] discard %s file: %s\n",
                       rows_snapshot == 0 ? "empty" : "short-stub",
                       discard_path.c_str());
                unlink(discard_path.c_str());
                unlink((discard_path + ".info").c_str());
            }
            g_stage.clear();
            g_stage_rows = 0;
            g_file_dirty.store(false);
            return;
        }
    }

    // Finalize: -LIVE → -<HHMM>Z based on close time.
    std::string finalized_path;
    {
        std::lock_guard<std::mutex> lk(g_path_mtx);
        if(!g_cur_path.empty()){
            auto slash = g_cur_path.find_last_of('/');
            std::string dir  = (slash == std::string::npos) ? "" : g_cur_path.substr(0, slash+1);
            std::string base = (slash == std::string::npos) ? g_cur_path : g_cur_path.substr(slash+1);
            std::string fin  = build_hist_filename_finalize(base, (uint64_t)time(nullptr),
                                                             (int)g_hdr_cur.utc_offset_hours);
            if(fin != base){
                std::string final_full = dir + fin;
                if(rename(g_cur_path.c_str(), final_full.c_str()) == 0){
                    printf("[LongWaterfall] rotate finalize: %s → %s\n", base.c_str(), fin.c_str());
                    finalized_path = final_full;
                } else {
                    fprintf(stderr, "[LongWaterfall] rename failed: %s → %s errno=%d\n",
                            base.c_str(), fin.c_str(), errno);
                    finalized_path = g_cur_path;  // best-effort: original path
                }
            } else {
                finalized_path = g_cur_path;
            }
        }
        g_cur_path.clear();
    }
    g_stage.clear();
    g_stage_rows = 0;
    // 로컬본 처리 (v13.12) — "증명되면 지우고, 아니면 남긴다".
    //
    // CLEAN = 이 파일이 열려 있던 내내 (a) Central 연결이 끊긴 적 없고 (b) 업링크 큐가
    // 프레임을 버린 적도 없다. 그러면 Central 은 같은 행을 전부 받았다 — 로컬본은 잉여다.
    //
    // DIRTY = 둘 중 하나라도 일어났다. 종전엔 여기서 통파일을 곧장 push 했는데, 그건
    // 두 가지로 틀렸다: ① Central 이 멀쩡히 다 받았어도 무조건 올려 낭비였고
    // (실측 2026-07-27 DGS-2: 115MB 중복본), ② 올리는 사본이 5Hz max-hold 라 Central
    // 본보다 품질이 낮았다. 이제는 그냥 로컬에 남겨두고, /hist check 가 Central 의 실제
    // 행수와 대조해 "빠진 만큼만" 올린 뒤 지운다.
    if(!finalized_path.empty()){
        bool was_dirty = g_file_dirty.exchange(false);
        if(was_dirty){
            printf("[LongWaterfall] file finalize DIRTY — retain local for /hist check: %s\n",
                   finalized_path.c_str());
            std::function<void(const std::string&)> fn;
            { std::lock_guard<std::mutex> lk(g_retain_fn_mtx); fn = g_retain_fn; }
            if(fn) fn(finalized_path);
        } else {
            printf("[LongWaterfall] file finalize CLEAN — unlink local (Central has all rows): %s\n",
                   finalized_path.c_str());
            unlink(finalized_path.c_str());
            unlink((finalized_path + ".info").c_str());
        }
    }
}

// Returns true if filename ends with .bewehist (new) or .bewewf (legacy).
static bool is_lwf_filename(const char* n){
    const char* dot = strrchr(n, '.');
    if(!dot) return false;
    return strcmp(dot, ".bewehist") == 0 || strcmp(dot, ".bewewf") == 0;
}

bool open_new_file(uint64_t cf_hz, uint64_t sr_hz, uint32_t fft_size,
                   uint32_t fft_input_size,
                   float dmin, float dmax,
                   float station_lon, float station_lat,
                   const char* station_name){
    // 미션 활성 시 그 미션의 hist 디렉토리 사용. IDLE이면 빈 문자열 → silently
    // 파일 생성 건너뜀 (worker_loop가 500ms마다 재시도, 다음 mission_start
    // request_rotate 호출 시 다시 들어옴).
    std::string dir;
    if(g_v) dir = g_v->active_hist_dir();
    if(dir.empty()){
        // 미션 IDLE → 조용히 skip. worker 가 500ms 후 재시도.
        return false;
    }
    mkdir(BEWEPaths::recordings_dir().c_str(), 0755);

    uint64_t now_utc = (uint64_t)time(nullptr);
    int32_t off_h_now = KST::OFFSET_HOURS;  // KST 강제 (UTC+9)
    std::string fname = build_hist_filename_live(now_utc, cf_hz, (int)off_h_now, station_name);
    // 같은 분에 두 번 시작될 가능성 — 충돌 회피 suffix
    std::string full = dir + "/" + fname;
    for(int n=2; access(full.c_str(), F_OK)==0 && n<100; ++n){
        auto pos = fname.rfind("-LIVE.bewehist");
        if(pos == std::string::npos) break;
        fname = fname.substr(0,pos) + "_" + std::to_string(n) + "-LIVE.bewehist";
        full  = dir + "/" + fname;
    }

    FILE* fp = fopen(full.c_str(), "wb");
    if(!fp){
        fprintf(stderr, "[LongWaterfall] open failed: %s errno=%d\n", full.c_str(), errno);
        return false;
    }
    if(!(dmax > dmin)){ dmin = DEFAULT_DB_MIN; dmax = DEFAULT_DB_MAX; }
    FileHeader h{};
    memcpy(h.magic, "BWWF", 4);
    h.version        = FILE_VERSION;     // 0x0003
    h.fft_size       = fft_size;
    h.sample_rate_hz = sr_hz;
    h.center_freq_hz = cf_hz;
    // v13.12: 프레임당 1행 기록 → 헤더 row_rate 도 실측 캡처 행레이트.
    // (worker_loop 가 g_meas_row_rate 가 잡히기 전에는 파일을 열지 않는다.)
    h.row_rate_hz    = (g_meas_row_rate > 0.5f) ? g_meas_row_rate : DEFAULT_ROW_RATE_HZ;
    h.db_min         = dmin;
    h.db_max         = dmax;
    h.start_utc_unix = now_utc;
    h.station_lon    = station_lon;
    h.fft_input_size = fft_input_size;
    h.utc_offset_hours = off_h_now;
    // v3 fields
    h.station_lat = station_lat;
    if(station_name) strncpy(h.station_name, station_name, sizeof(h.station_name)-1);
    if(fwrite(&h, 1, sizeof(h), fp) != sizeof(h)){
        fclose(fp); return false;
    }
    fflush(fp);

    g_fp = fp;
    g_hdr_cur = h;
    g_stage.assign((size_t)STAGE_MAX_ROWS * fft_size, 0);
    g_stage_rows = 0;
    g_file_dirty.store(false);  // 새 파일 = LIVE tap 안정 가정으로 시작
    { std::lock_guard<std::mutex> lk(g_drop_fn_mtx);
      g_drop_base = g_drop_fn ? g_drop_fn() : 0; }
    {
        std::lock_guard<std::mutex> lk(g_path_mtx);
        g_cur_path = full;
    }
    printf("[LongWaterfall] new file: %s (fft=%u, %.3fMHz, %uMSPS, dB=[%.1f..%.1f], station='%s')\n",
           full.c_str(), fft_size, cf_hz/1e6, (unsigned)(sr_hz/1000000),
           h.db_min, h.db_max, h.station_name);

    // Build live-start packet + broadcast.
    PktLwfLiveStart ls{};
    strncpy(ls.filename, fname.c_str(), sizeof(ls.filename)-1);
    ls.fft_size        = fft_size;
    ls.fft_input_size  = fft_input_size;
    ls.sample_rate_hz  = sr_hz;
    ls.center_freq_hz  = cf_hz;
    ls.row_rate_hz     = h.row_rate_hz;
    ls.db_min          = h.db_min;
    ls.db_max          = h.db_max;
    ls.start_utc_unix  = h.start_utc_unix;
    ls.station_lon     = h.station_lon;
    ls.utc_offset_hours= h.utc_offset_hours;
    ls.station_lat     = h.station_lat;
    // Central 은 FFT_FRAME 을 행으로 기록한다 → 그쪽 파일의 row_rate 는 FFT 행레이트.
    // 아직 미측정(기동 직후)이면 0 → Central 이 row_rate_hz(5Hz)로 폴백.
    ls.fft_row_rate_hz = g_v ? g_v->fft_row_rate_hz.load(std::memory_order_relaxed) : 0.f;
    memcpy(ls.station_name, h.station_name, sizeof(ls.station_name));
    {
        std::lock_guard<std::mutex> lk(g_live_state_mtx);
        g_live_state = ls;
        g_live_state_valid = true;
        g_live_row_idx = 0;
    }
    LiveCallbacks cb_copy;
    { std::lock_guard<std::mutex> lk(g_live_cb_mtx); cb_copy = g_live_cb; }
    if(cb_copy.on_start) cb_copy.on_start(ls);

    return true;
}

// 스테이징된 행들을 디스크에 쓴다. 호출 시점에 data_mtx 를 잡고 있으면 안 된다.
void flush_row_locked(){
    if(!g_fp || g_stage_rows <= 0) return;
    const size_t dst_fft = g_hdr_cur.fft_input_size ? g_hdr_cur.fft_input_size
                                                    : g_hdr_cur.fft_size;
    if(dst_fft == 0){ g_stage_rows = 0; return; }
    fwrite(g_stage.data(), 1, (size_t)g_stage_rows * dst_fft, g_fp);
    fflush(g_fp);

    { std::lock_guard<std::mutex> lk(g_live_state_mtx);
      if(g_live_state_valid) g_live_row_idx += (uint32_t)g_stage_rows; }

    g_stage_rows = 0;
}

// 새 캡처 행을 "프레임당 1행" 으로 스테이지에 담는다 (v13.12: 종전 5Hz max-hold 폐지).
//
// Central 은 FFT_FRAME 을 받는 족족 1행씩 아카이브한다(archive_hist_on_fft). HOST 로컬이
// 5Hz 로 접어 쓰면 같은 시간대의 두 파일이 행수도 눈금도 달라져 서로 비교조차 안 된다.
// 그래서 여기서도 프레임당 1행, 같은 폴딩(pad 묶음 max), 같은 양자화식을 쓴다.
//
// 양자화는 broadcast_fft(net_server.cpp) 와 바이트 단위로 같아야 한다 — 그쪽은 절삭
// 캐스트다. db_to_byte() 는 반올림(+0.5)이라 최대 1 LSB 어긋나므로 여기서 쓰면 안 된다.
//
// data_mtx 를 잡은 채 fwrite 하면 캡처 스레드가 디스크에 물린다. 락 안에서는 폴딩과
// 양자화(순수 연산)만 하고, 쓰기는 호출자가 락을 놓은 뒤 flush_row_locked() 로 한다.
bool ingest_new_rows(FFTViewer* v){
    if(!v || !g_fp) return false;

    const float dmin = g_hdr_cur.db_min;
    const float dmax = g_hdr_cur.db_max;
    float range = dmax - dmin;
    if(!(range > 0.f)) range = 1.f;
    const float inv = 255.f / range;

    std::lock_guard<std::mutex> lk(v->data_mtx);
    // 같은 락 획득에 편승해 양자화 범위 스냅샷 갱신 — 워커 루프의 quant-rotate
    // 검사가 별도 data_mtx 획득 없이 이 값을 읽는다 (감지 지연 동일).
    g_seen_pmin = v->header.power_min;
    g_seen_pmax = v->header.power_max;
    int now = v->total_ffts;
    if(now <= g_last_total_ffts){ return false; }

    // v4.5.3 — 소스는 4× padded (fft_size), HIST 저장은 1× (fft_input_size).
    // 4 bin 묶음 max 로 폴딩 — 좁은 피크 보존. Central 은 양자화 후 max 를 취하는데,
    // db_to_byte 가 단조라 (max 후 양자화) == (양자화 후 max) 로 결과가 같다.
    int src_fft = v->fft_size;
    int dst_fft = v->fft_input_size;
    if(src_fft <= 0 || dst_fft <= 0) return false;
    if(dst_fft != (int)(g_hdr_cur.fft_input_size ? g_hdr_cur.fft_input_size
                                                 : g_hdr_cur.fft_size)) return false;
    int pad = src_fft / dst_fft; if(pad < 1) pad = 1;

    int new_rows = now - g_last_total_ffts;
    // Clamp: if we fell behind by > FFT_HISTORY_ROWS, only the last ring window is valid.
    if(new_rows > FFT_HISTORY_ROWS) new_rows = FFT_HISTORY_ROWS;
    // 스테이지 용량 상한 — 남은 건 다음 루프에서 이어 처리.
    int room = STAGE_MAX_ROWS - g_stage_rows;
    if(room <= 0) return false;
    if(new_rows > room) new_rows = room;

    if((int)g_stage.size() < STAGE_MAX_ROWS * dst_fft)
        g_stage.resize((size_t)STAGE_MAX_ROWS * dst_fft);

    // Capture writes rowp at fi=total_ffts, then increments total_ffts (under data_mtx).
    // So after we observe total_ffts==now, valid rows are at fi for abs_idx in [g_last, now-1].
    int start_abs = g_last_total_ffts;
    if(now - start_abs > new_rows) start_abs = now - new_rows;
    for(int abs_idx = start_abs; abs_idx < start_abs + new_rows; abs_idx++){
        int fi = abs_idx % FFT_HISTORY_ROWS;
        const float* rowp = v->fft_data.data() + (size_t)fi * src_fft;
        uint8_t* out = g_stage.data() + (size_t)g_stage_rows * dst_fft;
        for(int o=0; o<dst_fft; o++){
            float mx;
            if(pad == 1){
                mx = rowp[o];
            } else {
                const float* gp = rowp + (size_t)o * pad;
                mx = gp[0];
                for(int k=1; k<pad; k++){ if(gp[k] > mx) mx = gp[k]; }
            }
            float q = (mx - dmin) * inv;
            if(q < 0.f) q = 0.f;
            if(q > 255.f) q = 255.f;
            out[o] = (uint8_t)q;          // 절삭 — broadcast_fft 와 동일
        }
        g_stage_rows++;
    }
    g_last_total_ffts = start_abs + new_rows;
    return true;
}

// ── Worker loop ──────────────────────────────────────────────────────────
void worker_loop(){
    using clk = std::chrono::steady_clock;
    auto next_flush = clk::now() + std::chrono::milliseconds(200); // 5 Hz default
    // active_hist_dir() 폴링 캐시 (500ms) — 아래 게이트 참조
    clk::time_point s_dir_check{};
    bool s_hist_active = false;

    while(g_running.load(std::memory_order_relaxed)){
        if(!g_v){ std::this_thread::sleep_for(std::chrono::milliseconds(100)); continue; }

        // Rotate requested?
        int req = g_rotate_req_seq.load(std::memory_order_relaxed);
        if(req != g_rotate_seen_seq){
            g_rotate_seen_seq = req;
            // Flush any pending row, close current file.
            flush_row_locked();
            close_file_locked();
            // Reset row counter so we only catch fresh rows after rotate.
            // (avoid one-shot stale-data dump into new file)
            { std::lock_guard<std::mutex> lk(g_v->data_mtx);
              g_last_total_ffts = g_v->total_ffts; }
        }

        // Record while a mission is active. (HIST 는 TM IQ 롤링과 분리 —
        // 풀레이트 IQ 링 없이도 미션 HIST 기록. 미션 종료 시 dir 이 비어
        // 여기서 파일 close, 지연 ≤500ms.)
        // active_hist_dir() 은 mission mutex + string 힙할당이라 20ms 루프에서
        // 매번 부르지 않고 500ms 캐시로 폴링.
        {
            auto nowc = clk::now();
            if(s_dir_check == clk::time_point{} ||
               nowc - s_dir_check >= std::chrono::milliseconds(500)){
                s_dir_check = nowc;
                s_hist_active = !g_v->active_hist_dir().empty();
            }
        }
        if(!s_hist_active){
            if(g_fp){ flush_row_locked(); close_file_locked(); }
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }
        // 캡처 행레이트 실측 (1초 창). 파일이 열려 있든 아니든 계속 잰다.
        // total_ffts 는 1초 경계에서만 필요 — 캡처 스레드 data_mtx 를 50Hz 로 두드리지 않는다.
        {
            static clk::time_point rr_last = clk::now();
            static int             rr_base = -1;
            double el = std::chrono::duration<double>(clk::now() - rr_last).count();
            if(rr_base < 0 || el >= 1.0){
                int cur_total;
                { std::lock_guard<std::mutex> lk(g_v->data_mtx); cur_total = g_v->total_ffts; }
                if(rr_base < 0) rr_base = cur_total;
                if(el >= 1.0){
                    int d = cur_total - rr_base;
                    if(d > 0) g_meas_row_rate = (float)(d / el);
                    rr_last = clk::now(); rr_base = cur_total;
                }
            }
        }

        if(!g_fp){
            // 행레이트를 아직 모르면 파일을 열지 않는다 — 헤더 row_rate 가 틀리면
            // 뷰어 시간축 전체가 그 비율로 어긋나고, 나중에 고칠 수단이 없다.
            if(!(g_meas_row_rate > 0.5f)){
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
                continue;
            }
            uint64_t cf  = g_v->live_cf_hz.load(std::memory_order_relaxed);
            uint64_t sr  = (uint64_t)g_v->header.sample_rate;
            uint32_t fsz, fis;
            float    dmin, dmax;
            { std::lock_guard<std::mutex> lk(g_v->data_mtx);
              // v4.5.3 — HIST 파일은 1× FFT (fft_input_size) 로 저장.
              // 4× zero-pad 는 display 전용 — HIST 디스크 소비 4× 절감.
              fis  = (uint32_t)g_v->fft_input_size;
              fsz  = fis;
              // v13.12 — 양자화 범위를 Central 아카이브와 일치시킨다.
              // header.power_min/max 는 "캡처 양자화 범위" 이지 화면 슬라이더가
              // 아니다 (net_stream.cpp 가 broadcast_fft 에 넘기는 바로 그 값). 그래서
              // v13.3.2 가 고쳤던 "슬라이더 상한 넘는 신호가 255 로 포화" 문제는
              // 여기서 재발하지 않는다. 고정 -120..0 을 쓰면 Central 본과 눈금이
              // 달라져 같은 신호의 dB/SNR 이 서로 다르게 읽힌다.
              dmin = g_v->header.power_min;
              dmax = g_v->header.power_max; }
            if(!(dmax > dmin)){ dmin = DEFAULT_DB_MIN; dmax = DEFAULT_DB_MAX; }
            float    lon = g_v->station_lon;
            float    lat = g_v->station_lat;
            std::string sn = g_v->station_name;     // const std::string copy
            if(cf == 0 || sr == 0 || fsz == 0){
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                continue;
            }
            static int s_open_fail_cnt = 0;
            if(!open_new_file(cf, sr, fsz, fis, dmin, dmax, lon, lat, sn.c_str())){
                s_open_fail_cnt++;
                if(s_open_fail_cnt == 1 || s_open_fail_cnt == 10 ||
                   (s_open_fail_cnt % 120) == 0){
                    fprintf(stderr, "[LWF] open_new_file FAILED x%d in a row\n",
                            s_open_fail_cnt);
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(500));
                continue;
            }
            s_open_fail_cnt = 0;
            // Initialize last_total_ffts so we don't dump pre-existing buffer.
            { std::lock_guard<std::mutex> lk(g_v->data_mtx);
              g_last_total_ffts = g_v->total_ffts; }
            g_stage_rows = 0;
        }

        // 프레임당 1행 스테이징 (data_mtx 짧게) → 락 밖에서 디스크 쓰기.
        ingest_new_rows(g_v);
        flush_row_locked();

        // 업링크 드롭 감시 — FFT_FRAME 이 큐 오버플로로 버려졌으면 Central 아카이브에
        // 그만큼 행이 빠진 것이다. 끊김과 동일하게 dirty 로 올려 로컬본을 보존한다.
        if(g_fp && !g_file_dirty.load(std::memory_order_relaxed)){
            uint64_t now_drops = 0; bool have = false;
            { std::lock_guard<std::mutex> lk(g_drop_fn_mtx);
              if(g_drop_fn){ now_drops = g_drop_fn(); have = true; } }
            if(have && now_drops != g_drop_base){
                printf("[LongWaterfall] uplink drops %llu during file — mark dirty\n",
                       (unsigned long long)(now_drops - g_drop_base));
                g_file_dirty.store(true);
            }
        }

        // 양자화 기준이 바뀌면(오토스케일 재수렴, 수동 /rx autoscale) 파일을 가른다.
        // 한 파일에 두 기준의 행이 섞이면 어느 쪽으로 역산해도 절반이 틀린다 —
        // Central 이 같은 이유로 스트림을 rotate 한다(central_mission_archive.cpp).
        if(g_fp){
            // ingest_new_rows 가 방금 같은 락 안에서 떠 둔 스냅샷 사용 (추가 락 없음)
            float pmin = g_seen_pmin, pmax = g_seen_pmax;
            if(pmax > pmin &&
               (fabsf(pmin - g_hdr_cur.db_min) > 0.01f ||
                fabsf(pmax - g_hdr_cur.db_max) > 0.01f)){
                printf("[LongWaterfall] quant range changed %.1f/%.1f -> %.1f/%.1f — rotate\n",
                       g_hdr_cur.db_min, g_hdr_cur.db_max, pmin, pmax);
                close_file_locked();
                continue;
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // Shutdown: close cleanly.
    if(g_fp){ flush_row_locked(); close_file_locked(); }
}

} // anon

// ── Public API ───────────────────────────────────────────────────────────

void start_worker(FFTViewer* v){
    if(g_running.exchange(true)){
        // Already running → just re-bind viewer pointer (safe if same v).
        g_v = v;
        return;
    }
    g_v = v;
    g_rotate_seen_seq = g_rotate_req_seq.load();
    g_last_total_ffts = 0;
    // 기존 실행에서 crash로 남은 -LIVE.bewehist 파일을 mtime 기준으로 finalize.
    finalize_stale_live_all();
    g_thr = std::thread(worker_loop);
}

void stop_worker(){
    if(!g_running.exchange(false)) return;
    if(g_thr.joinable()) g_thr.join();
    g_v = nullptr;
}

void request_rotate(){
    g_rotate_req_seq.fetch_add(1, std::memory_order_relaxed);
}

// Central 연결이 끊겼을 때 cli_host 가 호출. 현재 열려있는 HIST 파일에 dirty flag set.
// finalize 시점에 dirty 면 MissionPush 로 통파일 push (LIVE tap 동안 누락 row 보완).
void mark_dirty(){
    g_file_dirty.store(true);
}

void set_on_retained(std::function<void(const std::string&)> fn){
    std::lock_guard<std::mutex> lk(g_retain_fn_mtx);
    g_retain_fn = std::move(fn);
}

void set_drop_counter(std::function<uint64_t()> fn){
    std::lock_guard<std::mutex> lk(g_drop_fn_mtx);
    g_drop_fn = std::move(fn);
    g_drop_base = g_drop_fn ? g_drop_fn() : 0;
}

std::string current_file_path(){
    std::lock_guard<std::mutex> lk(g_path_mtx);
    return g_cur_path;
}

void set_live_callbacks(const LiveCallbacks& cbs){
    std::lock_guard<std::mutex> lk(g_live_cb_mtx);
    g_live_cb = cbs;
}

bool snapshot_live_start(::PktLwfLiveStart& out){
    std::lock_guard<std::mutex> lk(g_live_state_mtx);
    if(!g_live_state_valid) return false;
    out = g_live_state;
    return true;
}

// Finalize any "...-LIVE.bewehist" files in `dir` that aren't the active recording.
// Uses file mtime as end time → renames to "...-HHMM.bewehist" (KST).
void finalize_stale_live_in_dir(const std::string& dir,
                                 const std::string& active_basename){
    DIR* d = opendir(dir.c_str());
    if(!d) return;
    struct dirent* de;
    std::vector<std::string> targets;
    while((de = readdir(d)) != nullptr){
        const char* n = de->d_name;
        if(!n || n[0]=='.') continue;
        std::string name = n;
        if(name.rfind("-LIVE.bewehist") == std::string::npos) continue;
        if(!active_basename.empty() && name == active_basename) continue;
        targets.push_back(name);
    }
    closedir(d);
    for(auto& base : targets){
        std::string full = dir + "/" + base;
        struct stat st{};
        if(stat(full.c_str(), &st) != 0) continue;
        uint64_t end_utc = (uint64_t)st.st_mtime;
        std::string fin = build_hist_filename_finalize(base, end_utc, 0);
        if(fin == base) continue;
        std::string new_full = dir + "/" + fin;
        // 충돌 회피: 같은 이름 이미 있으면 _2, _3 ... suffix
        std::string try_full = new_full;
        std::string try_fin  = fin;
        for(int n=2; access(try_full.c_str(), F_OK)==0 && n<100; ++n){
            auto dot = fin.rfind(".bewehist");
            if(dot == std::string::npos) break;
            try_fin  = fin.substr(0, dot) + "_" + std::to_string(n) + ".bewehist";
            try_full = dir + "/" + try_fin;
        }
        if(rename(full.c_str(), try_full.c_str()) == 0){
            printf("[LongWaterfall] stale-LIVE finalize: %s → %s\n",
                   base.c_str(), try_fin.c_str());
        } else {
            fprintf(stderr, "[LongWaterfall] stale-LIVE rename failed: %s → %s errno=%d\n",
                    base.c_str(), try_fin.c_str(), errno);
        }
    }
}

void finalize_stale_live_all(){
    // 현재 recording 중인 파일 basename (skip 대상)
    std::string cur = current_file_path();
    std::string cur_base;
    if(!cur.empty()){
        auto s = cur.find_last_of('/');
        cur_base = (s == std::string::npos) ? cur : cur.substr(s+1);
    }
    finalize_stale_live_in_dir(BEWEPaths::hist_host_dir(), cur_base);
    finalize_stale_live_in_dir(BEWEPaths::hist_join_dir(), cur_base);
    finalize_stale_live_in_dir(BEWEPaths::hist_live_dir(), cur_base);
    // 활성 미션 hist 디렉토리
    if(g_v){
        std::string md = g_v->active_hist_dir();
        if(!md.empty()) finalize_stale_live_in_dir(md, cur_base);
    }
    // 모든 미션 디렉토리 (지난 미션의 stale LIVE도 정리)
    // v3.20.0 layout: missions/<station>/<year>/<code>/hist/ — 3 단계 루프.
    DIR* dr = opendir(BEWEPaths::missions_root().c_str());
    if(!dr) return;
    struct dirent* de;
    while((de = readdir(dr)) != nullptr){
        const char* sname = de->d_name;
        if(!sname || sname[0]=='.') continue;
        // 4자리 숫자(YYYY) 폴더는 legacy — 마이그레이션 안 됐으면 무시.
        if(strlen(sname) == 4){
            bool all_digit = true;
            for(int i=0;i<4;i++) if(sname[i]<'0'||sname[i]>'9'){ all_digit=false; break; }
            if(all_digit) continue;
        }
        std::string sdir = BEWEPaths::missions_root() + "/" + sname;
        DIR* ds = opendir(sdir.c_str());
        if(!ds) continue;
        struct dirent* de_y;
        while((de_y = readdir(ds)) != nullptr){
            const char* yn = de_y->d_name;
            if(!yn || yn[0]=='.') continue;
            if(strlen(yn) != 4) continue;
            bool num = true;
            for(int i=0;i<4;i++) if(yn[i]<'0'||yn[i]>'9'){ num=false; break; }
            if(!num) continue;
            std::string ydir = sdir + "/" + yn;
            DIR* dy = opendir(ydir.c_str());
            if(!dy) continue;
            struct dirent* de2;
            while((de2 = readdir(dy)) != nullptr){
                const char* m = de2->d_name;
                if(!m || m[0]=='.') continue;
                std::string hd = ydir + "/" + m + "/hist";
                struct stat st;
                if(stat(hd.c_str(), &st) == 0 && S_ISDIR(st.st_mode)){
                    finalize_stale_live_in_dir(hd, cur_base);
                }
            }
            closedir(dy);
        }
        closedir(ds);
    }
    closedir(dr);
}

void scan_dir_into_list(::PktLwfList& out){
    memset(&out, 0, sizeof(out));
    std::string dir = BEWEPaths::hist_host_dir();
    // Exclude the file currently being recorded — JOINs see it via LIVE_START
    // (LIVE tab) instead, never as a finished entry in the HOST tab.
    std::string cur_full = current_file_path();
    std::string cur_base;
    if(!cur_full.empty()){
        size_t s = cur_full.find_last_of('/');
        cur_base = (s == std::string::npos) ? cur_full : cur_full.substr(s+1);
    }
    DIR* d = opendir(dir.c_str());
    if(!d) return;

    struct Entry {
        std::string name;
        uint64_t size, start, cf, sr;
        uint32_t fft;
        char     station_name[32];
        float    station_lat, station_lon;
    };
    std::vector<Entry> all;
    struct dirent* de;
    while((de = readdir(d)) != nullptr){
        const char* n = de->d_name;
        if(!n || n[0]=='.') continue;
        if(!is_lwf_filename(n)) continue;
        if(!cur_base.empty() && cur_base == n) continue; // skip active LIVE
        std::string full = dir + "/" + n;
        struct stat st{};
        if(stat(full.c_str(), &st) != 0) continue;
        if((uint64_t)st.st_size < sizeof(FileHeader)) continue;
        FILE* fp = fopen(full.c_str(), "rb");
        if(!fp) continue;
        FileHeader h{};
        if(fread(&h, 1, sizeof(h), fp) != sizeof(h) || memcmp(h.magic, "BWWF", 4) != 0
           || h.version != FILE_VERSION){
            fclose(fp); continue;
        }
        fclose(fp);
        Entry e{};
        e.name  = n;
        e.size  = (uint64_t)st.st_size;
        e.start = h.start_utc_unix;
        e.cf    = h.center_freq_hz;
        e.sr    = h.sample_rate_hz;
        e.fft   = h.fft_size;
        memcpy(e.station_name, h.station_name, sizeof(e.station_name));
        e.station_lat = h.station_lat;
        e.station_lon = h.station_lon;
        all.push_back(e);
    }
    closedir(d);

    // Newest first
    std::sort(all.begin(), all.end(),
              [](const Entry& a, const Entry& b){ return a.start > b.start; });
    if(all.size() > MAX_LWF_FILES) all.resize(MAX_LWF_FILES);

    out.count = (uint16_t)all.size();
    for(size_t i=0; i<all.size(); i++){
        LwfFileEntry& e = out.entries[i];
        memset(&e, 0, sizeof(e));
        strncpy(e.filename, all[i].name.c_str(), sizeof(e.filename)-1);
        e.size_bytes     = all[i].size;
        e.start_utc      = all[i].start;
        e.center_freq_hz = all[i].cf;
        e.sample_rate_hz = all[i].sr;
        e.fft_size       = all[i].fft;
        e.num_rows       = (uint32_t)((all[i].size - sizeof(FileHeader)) / std::max<uint64_t>(1, all[i].fft));
        memcpy(e.station_name, all[i].station_name, sizeof(e.station_name));
        e.station_lat    = all[i].station_lat;
        e.station_lon    = all[i].station_lon;
    }
}

} // namespace LongWaterfall
