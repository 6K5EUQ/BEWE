// Long Waterfall viewer modal.
// Layout: viewer (left, large) + resizable splitter + file list (right, status-style).
// Y=freq (top=high, bottom=low, linear FFT-shifted). X=time (left=old, right=new).
// Wheel = X(time) cursor-anchored zoom; Ctrl+wheel = Y(freq) cursor-anchored zoom.
// Arrow ←/→ = pan one full screen (EID parity).
// File click = select. Right-click = context menu (Info / Delete).
//
// Source files:
//   - HOST 자기 파일: ~/BEWE/recordings/long_waterfall/*.bewewf  (worker가 직접 기록)
//   - JOIN 다운로드: 같은 디렉토리에 host에서 받은 파일을 같은 이름으로 저장

#include "long_waterfall.hpp"
#include "fft_viewer.hpp"
#include "bewe_paths.hpp"
#include "net_protocol.hpp"
#include "net_client.hpp"

#include <imgui.h>
#include <GL/glew.h>

#include <vector>
#include <string>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <algorithm>
#include <mutex>
#include <unordered_map>
#include <functional>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <zstd.h>

namespace {

// ── Jet colormap ─────────────────────────────────────────────────────────
ImU32 jet_color(uint8_t v){
    float t = v / 255.0f;
    float r = std::max(0.0f, std::min(1.0f, 1.5f - std::fabs(4.0f*t - 3.0f)));
    float g = std::max(0.0f, std::min(1.0f, 1.5f - std::fabs(4.0f*t - 2.0f)));
    float b = std::max(0.0f, std::min(1.0f, 1.5f - std::fabs(4.0f*t - 1.0f)));
    return IM_COL32((int)(r*255),(int)(g*255),(int)(b*255),255);
}

// ── Open file state ──────────────────────────────────────────────────────
struct OpenFile {
    std::string path;
    LongWaterfall::FileHeader hdr{};
    uint64_t total_size = 0;
    uint32_t num_rows = 0;
    FILE*    fp = nullptr;
    // mmap 가속: rebuild_texture가 fseek/fread 대신 직접 포인터 액세스.
    // 실패 시(open/mmap 에러) map==nullptr 유지 — fp fallback으로 동작.
    int            fd = -1;
    const uint8_t* map = nullptr;
    size_t         map_size = 0;
    std::vector<uint8_t> rowbuf;   // v3 mmap 실패 시 1행 fread fallback 버퍼
    // ── v4 (블록 zstd) ────────────────────────────────────────────────────
    // get_row() 가 v4 면 block 을 해제해 cache_buf 에 담고 그 안 포인터를 반환.
    // 단일 블록 캐시로 충분: 두 소비자(scan_file_db_range, rebuild_texture)는
    // 반환 포인터를 다음 get_row 전에 소비하고 r 을 단조 증가로 훑는다.
    bool     is_v4 = false;
    uint8_t  v4_flags = 0;      // HIST_V4_FLAG_* (col-delta 등)
    uint32_t block_rows = 0;
    std::vector<LongWaterfall::HistBlockIndex> index;   // num_blocks entries
    int                  cache_block = -1;              // 캐시된 블록 번호 (-1=없음)
    std::vector<uint8_t> cache_buf;                     // block_rows * fft_size (해제본)
    std::vector<uint8_t> frame_buf;                     // mmap 실패 시 압축 프레임 읽기
    // ── v13.3.2: v4 전체 선해제 (open 시 1회) ─────────────────────────────
    // 열 때 전 블록을 풀어 RAM 에 상주시키면 이후 get_row() 는 포인터 반환만 하고
    // zstd 해제가 사라진다 — 팬/줌마다 도는 rebuild_texture 가 순수 메모리 접근이
    // 되어 조작이 매끄러워진다. "열 때 좀 걸려도 이후 안 버벅" 을 택한 것.
    // LIVE(자라는 파일)·과대 파일은 선해제하지 않고 종전 블록캐시로 동작.
    std::vector<uint8_t> full_buf;   // num_rows * fft_size (비면 미사용)
    bool                 full_ready = false;
};
// 선해제 상한 — 이보다 큰 파일은 종전 블록캐시 경로. HIST 뷰어는 8GB+ PC 에서
// 쓰지만 무제한으로 물면 여러 파일을 열 때 누적된다.
static constexpr uint64_t FULL_DECOMP_MAX_BYTES = 768ull * 1024 * 1024;
OpenFile g_open;

// ── Texture ──────────────────────────────────────────────────────────────
GLuint   g_tex = 0;
int      g_tex_w = 1024;
int      g_tex_h = 512;
std::vector<uint32_t> g_pixel_buf;
bool     g_tex_dirty = true;

// View region: t0/t1 in row indices, f0/f1 in linear freq idx (0=lowest)
double   g_t0=0, g_t1=0;
double   g_f0=0, g_f1=0;

// 색 윈도 — 열린 파일 자체를 스캔해 얻는다 (라이브 워터폴과 무관).
// 헤더의 db_min/db_max 는 byte↔dB 눈금자일 뿐, 파일이 실제로 담고 있는 신호
// 범위와 다르다 (파일 오픈 시점의 값이라 stale 하기도 하다). 그래서 전 행의
// 바이트 히스토그램을 세어 실시간 autoscale 과 같은 공식으로 윈도를 잡는다.
float    g_file_db_min = 0.0f;
float    g_file_db_max = 0.0f;
uint64_t g_scanned_rows = 0;      // 스캔한 표본 행수 (진단용)
uint64_t g_scan_cursor  = 0;      // 다음 스캔할 행 인덱스 (서브샘플 커서)
uint64_t g_hist[256] = {0};       // 바이트 히스토그램 (누적)

// (우측 파일목록 패널 제거 — 도달불가 확정 후 삭제. 진입은 미션창에서 파일 선택.)

// ── Measurement region overlay (Ctrl+우클릭 드래그로 생성, BW/시간 측정용) ─
struct Meas {
    bool   selecting = false;        // dragging out a new region
    bool   active    = false;        // region exists
    // Data coords. t in row indices, f in linear freq idx [0, fft_size].
    double t0 = 0, t1 = 0;
    double f0 = 0, f1 = 0;
    // Edit state (for resize/move via left-click)
    enum Edit { EDIT_NONE, EDIT_MOVE, EDIT_L, EDIT_R, EDIT_T, EDIT_B } edit = EDIT_NONE;
    double sv_t0=0, sv_t1=0, sv_f0=0, sv_f1=0;   // saved on edit start
    double mx0=0, my0=0;                          // mouse data coord on edit start
};
Meas g_meas;

// ── 좌드래그 줌 (박스 친 영역으로 바로 확대) + 줌 히스토리 (뒤로가기) ──
struct ZoomDrag {
    bool   active = false;                 // dragging out a zoom box
    double t0=0, t1=0, f0=0, f1=0;          // box in data coords (row idx / freq idx)
    double px0=0, py0=0;                    // screen px where drag started (실수클릭 판정용)
};
ZoomDrag g_zdrag;

struct ViewRect { double t0, t1, f0, f1; };
std::vector<ViewRect> g_zoom_hist;         // 이전 뷰들 (뒤로가기 스택)
void push_zoom_hist(){
    // 현재 뷰를 스택에 저장. 과도한 성장 방지 상한.
    g_zoom_hist.push_back({g_t0, g_t1, g_f0, g_f1});
    if(g_zoom_hist.size() > 64) g_zoom_hist.erase(g_zoom_hist.begin());
}
void pop_zoom_hist(){
    if(g_zoom_hist.empty()) return;
    ViewRect r = g_zoom_hist.back();
    g_zoom_hist.pop_back();
    g_t0 = r.t0; g_t1 = r.t1; g_f0 = r.f0; g_f1 = r.f1;
    g_tex_dirty = true;
}


// 표시용 — 파일명에서 .bewehist/.bewewf 확장자 제거 (공간 절약)
static std::string strip_hist_ext(const std::string& s){
    if(s.size() >= 9 && s.compare(s.size()-9, 9, ".bewehist")==0) return s.substr(0, s.size()-9);
    if(s.size() >= 7 && s.compare(s.size()-7, 7, ".bewewf")==0)   return s.substr(0, s.size()-7);
    return s;
}

// ARCHIVE 스타일 hover tooltip — Station/Frequency/Start/(Stop). is_live이면 Stop 생략.
// 시간은 항상 KST(UTC+9).
static void hist_row_tooltip(const char* station_name, float station_lat, float station_lon,
                              uint64_t cf_hz, uint64_t start_utc, uint64_t end_utc,
                              bool is_live){
    auto fmt_local = [](uint64_t utc) -> std::string {
        struct tm tm_loc; KST::to_tm((time_t)utc, tm_loc);
        char b[24];
        snprintf(b, sizeof(b), "%02d:%02d", tm_loc.tm_hour, tm_loc.tm_min);
        return b;
    };
    if(!ImGui::BeginTooltip()) return;
    if(station_name && station_name[0]){
        ImGui::Text("Station   : %s (%s)",
            station_name,
            LongWaterfall::fmt_lat_lon(station_lat, station_lon).c_str());
    } else {
        ImGui::TextDisabled("Station   : ?");
    }
    ImGui::Text("Frequency : %.1f MHz", (double)cf_hz / 1e6);
    ImGui::Text("Start     : %s", fmt_local(start_utc).c_str());
    if(!is_live){
        ImGui::Text("Stop      : %s", fmt_local(end_utc).c_str());
    }
    ImGui::EndTooltip();
}

// LWF list cached from host (JOIN side)

uint64_t   g_last_known_rows = 0;

// ── Helpers ──────────────────────────────────────────────────────────────
void close_open(){
    if(g_open.map){ munmap((void*)g_open.map, g_open.map_size); }
    if(g_open.fd >= 0){ ::close(g_open.fd); }
    if(g_open.fp){ fclose(g_open.fp); g_open.fp = nullptr; }
    g_open = OpenFile{};
    g_last_known_rows = 0;
    g_tex_dirty = true;
    std::memset(g_hist, 0, sizeof(g_hist));
    g_scanned_rows = 0;
    g_scan_cursor = 0;
    g_zoom_hist.clear();
    g_zdrag = ZoomDrag{};
}

static void preload_full_v4();   // 정의는 get_row 뒤 (블록캐시 경로를 재사용)

bool open_file(const std::string& path){
    close_open();
    FILE* fp = fopen(path.c_str(), "rb");
    if(!fp) return false;
    LongWaterfall::FileHeader h{};
    if(fread(&h, 1, sizeof(h), fp) != sizeof(h) || memcmp(h.magic,"BWWF",4)!=0
       || (h.version != LongWaterfall::FILE_VERSION
           && h.version != LongWaterfall::FILE_VERSION_ZSTD)){
        fclose(fp); return false;
    }
    fseek(fp, 0, SEEK_END);
    uint64_t sz = (uint64_t)ftell(fp);
    fseek(fp, 0, SEEK_SET);
    if(h.fft_size == 0 || sz < sizeof(h)){ fclose(fp); return false; }
    g_open.path = path;
    g_open.hdr = h;
    g_open.total_size = sz;
    g_open.fp = fp;
    // mmap (RDONLY, SHARED) — 실패해도 fp fallback으로 동작.
    g_open.fd = ::open(path.c_str(), O_RDONLY);
    if(g_open.fd >= 0 && sz > 0){
        void* m = mmap(nullptr, sz, PROT_READ, MAP_SHARED, g_open.fd, 0);
        if(m != MAP_FAILED){
            g_open.map = (const uint8_t*)m;
            g_open.map_size = sz;
            // jump-around 패턴: 커널 readahead가 별 도움 안 됨.
            posix_madvise(m, sz, POSIX_MADV_RANDOM);
        } else {
            ::close(g_open.fd); g_open.fd = -1;
        }
    }
    if(h.version == LongWaterfall::FILE_VERSION_ZSTD){
        // v4: 헤더의 authoritative num_rows + footer index. size 산술 안 씀.
        const LongWaterfall::V4Ext& ext = LongWaterfall::v4ext(h);
        const uint32_t nb   = ext.num_blocks;
        const uint64_t ioff = ext.index_offset;
        const size_t   need = (size_t)nb * sizeof(LongWaterfall::HistBlockIndex);
        if(ext.codec != LongWaterfall::HIST_CODEC_ZSTD || ext.block_rows == 0 ||
           nb == 0 || ioff < sizeof(h) || ioff + need > sz){
            close_open(); return false;
        }
        g_open.is_v4      = true;
        g_open.v4_flags   = ext.flags;
        g_open.block_rows = ext.block_rows;
        g_open.num_rows   = (uint32_t)ext.num_rows;
        g_open.index.resize(nb);
        if(g_open.map && ioff + need <= g_open.map_size){
            memcpy(g_open.index.data(), g_open.map + ioff, need);
        } else if(fseek(fp, (long)ioff, SEEK_SET) != 0 ||
                  fread(g_open.index.data(), 1, need, fp) != need){
            close_open(); return false;
        }
    } else {
        g_open.num_rows = (uint32_t)((sz - sizeof(h)) / h.fft_size);
    }
    g_t0 = 0; g_t1 = std::max<uint32_t>(1, g_open.num_rows);
    g_f0 = 0; g_f1 = h.fft_size;
    g_tex_dirty = true;
    g_last_known_rows = g_open.num_rows;
    // v13.3.2 — 종료된 v4 파일은 여기서 전체를 풀어 둔다 (열 때 한 번 비용을 치르고
    // 이후 팬/줌을 매끄럽게). LIVE 는 계속 자라므로 제외 — 종전 블록캐시로 동작.
    if(path.find("-LIVE.bewehist") == std::string::npos) preload_full_v4();
    return true;
}

bool read_header_only(const std::string& path, LongWaterfall::FileHeader& h, uint64_t& size){
    FILE* fp = fopen(path.c_str(), "rb");
    if(!fp) return false;
    if(fread(&h, 1, sizeof(h), fp) != sizeof(h) || memcmp(h.magic,"BWWF",4)!=0
       || (h.version != LongWaterfall::FILE_VERSION
           && h.version != LongWaterfall::FILE_VERSION_ZSTD)){
        fclose(fp); return false;
    }
    fseek(fp, 0, SEEK_END);
    size = (uint64_t)ftell(fp);
    fclose(fp);
    return true;
}

// 헤더+파일크기로부터 행 수. v4 는 헤더의 authoritative num_rows, v3 는 size 산술.
static uint64_t hist_rows_from_header(const LongWaterfall::FileHeader& h, uint64_t file_size){
    if(h.version == LongWaterfall::FILE_VERSION_ZSTD)
        return LongWaterfall::v4ext(h).num_rows;
    if(h.fft_size == 0) return 0;
    return (file_size - sizeof(h)) / h.fft_size;
}

void refresh_size_live(){
    if(!g_open.fp) return;
    if(g_open.is_v4) return;   // v4(완료·압축)는 성장하지 않음 — size 재계산 금지
    struct stat st{};
    if(stat(g_open.path.c_str(), &st) != 0) return;
    uint64_t sz = (uint64_t)st.st_size;
    if(sz == g_open.total_size) return;
    g_open.total_size = sz;
    uint32_t old_rows = g_open.num_rows;
    g_open.num_rows = (uint32_t)((sz - sizeof(LongWaterfall::FileHeader)) / g_open.hdr.fft_size);
    if(g_open.num_rows != old_rows){
        if(g_t1 >= old_rows - 0.5){
            double w = g_t1 - g_t0;
            g_t1 = g_open.num_rows;
            g_t0 = g_t1 - w;
            if(g_t0 < 0) g_t0 = 0;
        }
        // mmap 확장: LIVE 파일은 계속 자라므로 새 크기로 remap.
        if(g_open.fd >= 0){
            if(g_open.map){ munmap((void*)g_open.map, g_open.map_size); g_open.map = nullptr; g_open.map_size = 0; }
            void* m = mmap(nullptr, sz, PROT_READ, MAP_SHARED, g_open.fd, 0);
            if(m != MAP_FAILED){
                g_open.map = (const uint8_t*)m;
                g_open.map_size = sz;
                posix_madvise(m, sz, POSIX_MADV_RANDOM);
            }
        }
        g_tex_dirty = true;
        g_last_known_rows = g_open.num_rows;
    }
}

// 행 r 의 fft_size 바이트 시작 포인터. v3=mmap/fread 직접, v4=해당 블록을 zstd
// 해제해 캐시(cache_buf) 안 포인터 반환. 실패(범위밖/디코드오류) 시 nullptr.
// 반환 포인터는 "다음 get_row 호출 전까지"만 유효 (v4 단일 블록 캐시).
static const uint8_t* get_row(uint32_t r){
    const uint32_t fft_sz = g_open.hdr.fft_size;
    if(fft_sz == 0 || r >= g_open.num_rows) return nullptr;

    if(!g_open.is_v4){
        const uint64_t off = sizeof(LongWaterfall::FileHeader) + (uint64_t)r * fft_sz;
        if(g_open.map && off + fft_sz <= g_open.map_size)
            return g_open.map + off;                 // mmap fast path (syscall 없음)
        if(!g_open.fp) return nullptr;
        if(g_open.rowbuf.size() != fft_sz) g_open.rowbuf.resize(fft_sz);
        if(fseek(g_open.fp, (long)off, SEEK_SET) != 0) return nullptr;
        if(fread(g_open.rowbuf.data(), 1, fft_sz, g_open.fp) != fft_sz) return nullptr;
        return g_open.rowbuf.data();
    }

    // v13.3.2: 선해제본이 있으면 해제 없이 바로 포인터.
    if(g_open.full_ready){
        const size_t off = (size_t)r * fft_sz;
        if(off + fft_sz <= g_open.full_buf.size()) return g_open.full_buf.data() + off;
        return nullptr;
    }

    // v4: 블록 zstd 해제 + 단일 블록 캐시.
    if(g_open.block_rows == 0) return nullptr;
    const uint32_t block = r / g_open.block_rows;
    if(block != (uint32_t)g_open.cache_block){
        if(block >= g_open.index.size()) return nullptr;
        const auto& e = g_open.index[block];
        const uint8_t* src;
        if(g_open.map && e.frame_offset + e.comp_len <= g_open.map_size){
            src = g_open.map + e.frame_offset;
        } else {
            if(!g_open.fp) return nullptr;
            if(g_open.frame_buf.size() < e.comp_len) g_open.frame_buf.resize(e.comp_len);
            if(fseek(g_open.fp, (long)e.frame_offset, SEEK_SET) != 0) return nullptr;
            if(fread(g_open.frame_buf.data(), 1, e.comp_len, g_open.fp) != e.comp_len) return nullptr;
            src = g_open.frame_buf.data();
        }
        const size_t cap  = (size_t)g_open.block_rows * fft_sz;
        const size_t want = (size_t)e.raw_rows * fft_sz;
        if(g_open.cache_buf.size() < cap) g_open.cache_buf.resize(cap);
        const size_t d = ZSTD_decompress(g_open.cache_buf.data(), cap, src, e.comp_len);
        if(ZSTD_isError(d) || d != want){ g_open.cache_block = -1; return nullptr; }
        if(g_open.v4_flags & LongWaterfall::HIST_V4_FLAG_COL_DELTA){
            // un-delta (freq): x[i]=d[i]+x[i-1] per row (블록 로드 1회만).
            for(uint32_t rr = 0; rr < e.raw_rows; rr++){
                uint8_t* row = g_open.cache_buf.data() + (size_t)rr * fft_sz;
                for(uint32_t i = 1; i < fft_sz; i++) row[i] = (uint8_t)(row[i] + row[i-1]);
            }
        }
        g_open.cache_block = (int)block;
    }
    const uint32_t within = r - block * g_open.block_rows;
    return g_open.cache_buf.data() + (size_t)within * fft_sz;
}

// v13.3.2 — v4(zstd) 파일 전체를 한 번에 풀어 RAM 에 올린다 (open 시 1회).
// 성공하면 이후 get_row() 는 압축해제 없이 포인터만 반환하므로 팬/줌마다 도는
// rebuild_texture 가 매끄러워진다. 실패/미적용 시 조용히 종전 블록캐시로 동작.
// LIVE(자라는 파일)에는 쓰지 않는다 — 새 행이 붙으면 선해제본이 곧 낡는다.
static void preload_full_v4(){
    if(!g_open.is_v4 || g_open.full_ready) return;
    if(g_open.num_rows == 0 || g_open.hdr.fft_size == 0) return;
    const uint64_t need = (uint64_t)g_open.num_rows * g_open.hdr.fft_size;
    if(need > FULL_DECOMP_MAX_BYTES) return;      // 과대 — 종전 경로 유지

    std::vector<uint8_t> buf;
    try { buf.resize((size_t)need); }
    catch(const std::bad_alloc&){ return; }       // RAM 부족 — 종전 경로 유지

    // 블록 순서대로 해제 (get_row 의 블록캐시 경로를 그대로 재사용 — full_ready 는
    // 아직 false 라 재귀하지 않는다). 블록당 1회 해제로 전체를 채운다.
    const uint32_t fft_sz = g_open.hdr.fft_size;
    for(uint32_t r = 0; r < g_open.num_rows; r++){
        const uint8_t* p = get_row(r);
        if(!p) return;                            // 손상/실패 — 선해제 포기
        memcpy(buf.data() + (size_t)r * fft_sz, p, fft_sz);
    }
    g_open.full_buf.swap(buf);
    g_open.full_ready = true;
    // 블록캐시는 이제 안 쓰므로 메모리 반환.
    g_open.cache_buf.clear(); g_open.cache_buf.shrink_to_fit();
    g_open.cache_block = -1;
}

// 열린 파일의 dB 색 윈도를 파일 데이터 자체에서 구한다.
// 실시간 autoscale 과 같은 공식: 노이즈플로어 = 하위 15% 분위수 →
// autoscale_db_window() (fft_viewer.hpp) 에 그대로 넘긴다.
// 바이트 히스토그램(256칸)만 누적하므로 분위수·최대값이 O(1) 로 나온다.
//
// 대형 파일 로딩 지연 방지 — 서브샘플 스캔:
//  전 행 대신 행 stride 로 건너뛰며 최대 ~SCAN_ROW_TARGET 행만 스캔. 분위수·피크는
//  통계적으로 거의 동일하나 zstd 압축해제량이 수십분의 1 → 한 프레임에 끝나도 안 멈춤.
//  (프레임 분할은 오히려 색 수렴 중 매 프레임 텍스처 재빌드를 유발해 더 느려서 제거.)
// LIVE 파일은 계속 자라므로 stride 재산출한 채 새 행 커서로 이어 스캔한다.
static constexpr uint64_t SCAN_ROW_TARGET = 8000;  // 목표 스캔 표본 행수
static void scan_file_db_range(){
    if(!g_open.fp || g_open.hdr.fft_size == 0) return;
    const uint32_t fft_sz = g_open.hdr.fft_size;
    if(g_scan_cursor > g_open.num_rows){     // 다른 파일로 교체됨 → 처음부터
        std::memset(g_hist, 0, sizeof(g_hist));
        g_scan_cursor = 0; g_scanned_rows = 0;
    }
    // v13.3.2 — 블록 단위 서브샘플.
    //  v4 압축 파일은 get_row() 가 행 하나를 위해 블록 전체(~2MB)를 zstd 해제하고
    //  블록 1개만 캐시한다. 행을 stride 로 흩뿌리면 매 행이 캐시 미스라 해제 횟수가
    //  표본 행수와 같아진다 (8000회) — 전 행 순차 스캔(블록당 1회)보다 오히려 느리다.
    //  그래서 블록을 건너뛰되 고른 블록 안에서는 연속으로 읽는다:
    //  해제 횟수 = 고른 블록 수(수십), 표본 수는 그대로 유지.
    //  비압축(mmap) 파일은 block_rows==0 → 종전 행 stride 그대로.
    //  비압축은 청크=1 이라 자연히 v13.3.1 의 행 stride 와 동일해진다.
    const uint64_t chunk = (g_open.block_rows > 0) ? g_open.block_rows : 1;
    if(g_scan_cursor < g_open.num_rows){
        // 표본 목표를 맞추는 청크 stride (청크 단위로 건너뜀).
        // 압축: 청크당 block_rows 행을 통째로 세므로 목표 행수를 청크 수로 환산.
        // 비압축: chunk==1 → want_chunks==SCAN_ROW_TARGET → 행 stride 와 동일.
        const uint64_t total_chunks = (g_open.num_rows + chunk - 1) / chunk;
        const uint64_t want_chunks  = (SCAN_ROW_TARGET + chunk - 1) / chunk;
        uint64_t cstride = (want_chunks > 0) ? total_chunks / want_chunks : 1;
        if(cstride < 1) cstride = 1;

        for(uint64_t c = g_scan_cursor / chunk; c * chunk < g_open.num_rows; c += cstride){
            const uint64_t r0 = c * chunk;
            const uint64_t r1 = std::min<uint64_t>(r0 + chunk, g_open.num_rows);
            bool ok = true;
            for(uint64_t r = r0; r < r1; r++){
                const uint8_t* p = get_row((uint32_t)r);
                if(!p){ ok = false; break; }
                // bin 0 은 DC — 실시간 autoscale 도 i=1 부터 누적하므로 동일하게 제외.
                for(uint32_t i = 1; i < fft_sz; i++) g_hist[p[i]]++;
                g_scanned_rows++;
            }
            g_scan_cursor = r0 + cstride * chunk;
            if(!ok) break;
        }
    }

    uint64_t total = 0;
    for(int b = 0; b < 256; b++) total += g_hist[b];
    if(total == 0){                          // 빈 파일 → 헤더 눈금자 그대로
        g_file_db_min = g_open.hdr.db_min;
        g_file_db_max = g_open.hdr.db_max;
        return;
    }
    uint64_t want = (uint64_t)(total * 0.15);
    int noise_b = 0, peak_b = 0;
    uint64_t acc = 0;
    for(int b = 0; b < 256; b++){
        acc += g_hist[b];
        if(acc > want){ noise_b = b; break; }
    }
    for(int b = 255; b >= 0; b--){
        if(g_hist[b]){ peak_b = b; break; }
    }
    const float fmin = g_open.hdr.db_min, fmax = g_open.hdr.db_max;
    float noise = LongWaterfall::byte_to_db((uint8_t)noise_b, fmin, fmax);
    float peak  = LongWaterfall::byte_to_db((uint8_t)peak_b,  fmin, fmax);
    autoscale_db_window(noise, peak, g_file_db_min, g_file_db_max);
}

void rebuild_texture(float view_db_min, float view_db_max){
    if(!g_open.fp) return;
    if(g_pixel_buf.size() != (size_t)g_tex_w * g_tex_h)
        g_pixel_buf.assign((size_t)g_tex_w * g_tex_h, 0);

    int W = g_tex_w, H = g_tex_h;
    double t_span = std::max(1.0, g_t1 - g_t0);
    double f_span = std::max(1.0, g_f1 - g_f0);
    uint32_t fft_sz = g_open.hdr.fft_size;
    if(fft_sz == 0) return;
    int fft_half = (int)fft_sz / 2;

    // byte → dB → 메인 워터폴 윈도 [view_db_min, view_db_max] 재정규화.
    const float fmin = g_open.hdr.db_min;
    const float fmax = g_open.hdr.db_max;
    const float fspan = std::max(1e-3f, fmax - fmin);
    const float vspan_inv = 1.0f / std::max(1.0f, view_db_max - view_db_min);

    // 256-entry color LUT — byte를 최종 픽셀 색으로 한 번에 매핑.
    ImU32 color_lut[256];
    for(int b=0; b<256; b++){
        float db = fmin + (b / 255.0f) * fspan;
        float tt = (db - view_db_min) * vspan_inv;
        if(tt < 0.f) tt = 0.f; else if(tt > 1.f) tt = 1.f;
        color_lut[b] = jet_color((uint8_t)(tt * 255.0f));
    }

    // 가시 linear freq 범위 → 저장 bin 범위 (FFT-shift 풀기).
    // linear: 0=lowest, fft_half=DC, fft_sz-1=highest. storage: 0=DC.
    // bin = (idx<fft_half) ? idx+fft_half : idx-fft_half.
    int lin_lo = (int)std::floor(g_f0); if(lin_lo < 0) lin_lo = 0;
    int lin_hi = (int)std::ceil (g_f1); if(lin_hi > (int)fft_sz) lin_hi = fft_sz;
    if(lin_hi <= lin_lo) lin_hi = lin_lo + 1;
    int br_lo[2], br_hi[2]; int n_br = 0;
    if(lin_hi <= fft_half){
        br_lo[0] = lin_lo + fft_half; br_hi[0] = lin_hi + fft_half; n_br = 1;
    } else if(lin_lo >= fft_half){
        br_lo[0] = lin_lo - fft_half; br_hi[0] = lin_hi - fft_half; n_br = 1;
    } else {
        br_lo[0] = lin_lo + fft_half; br_hi[0] = (int)fft_sz;
        br_lo[1] = 0;                 br_hi[1] = lin_hi - fft_half; n_br = 2;
    }

    std::vector<uint8_t> col_max(fft_sz, 0);  // 호이스팅: 컬럼마다 가시 bin만 zero.

    int rows_total = (int)t_span;
    int rows_per_col_max = 64;
    int rows_step = std::max(1, rows_total / (W * rows_per_col_max));

    // y→bin 매핑은 x 와 무관 — 컬럼마다 재계산하지 않고 1회 계산
    std::vector<int> y_lo(H), y_hi(H);
    for(int y=0; y<H; y++){
        double f_top = g_f1 - (y       / (double)H) * f_span;
        double f_bot = g_f1 - ((y+1.0) / (double)H) * f_span;
        int idx_lo = (int)std::floor(std::min(f_top, f_bot));
        int idx_hi = (int)std::ceil(std::max(f_top, f_bot));
        if(idx_lo < 0) idx_lo = 0;
        if(idx_hi > (int)fft_sz) idx_hi = fft_sz;
        if(idx_hi <= idx_lo) idx_hi = idx_lo + 1;
        y_lo[y] = idx_lo; y_hi[y] = idx_hi;
    }

    for(int x=0; x<W; x++){
        double t_a = g_t0 + (x      / (double)W) * t_span;
        double t_b = g_t0 + ((x+1)  / (double)W) * t_span;
        int ra = (int)std::floor(t_a);
        int rb = (int)std::floor(t_b);
        if(rb <= ra) rb = ra + 1;
        if(ra < 0) ra = 0;
        if(rb > (int)g_open.num_rows) rb = g_open.num_rows;
        if(ra >= rb){
            for(int y=0; y<H; y++) g_pixel_buf[(size_t)y*W + x] = IM_COL32(20,20,25,255);
            continue;
        }
        // 가시 bin 영역만 0으로 reset (max-hold 시작값).
        for(int k=0; k<n_br; k++)
            std::memset(col_max.data() + br_lo[k], 0, (size_t)(br_hi[k] - br_lo[k]));

        int n_sampled = 0;
        for(int r=ra; r<rb; r += rows_step){
            const uint8_t* rb_ptr = get_row((uint32_t)r);   // v3=mmap/fread, v4=블록해제
            if(!rb_ptr) break;
            // 가시 bin만 max-hold (off-screen bin 무시).
            uint8_t* cm_ptr = col_max.data();
            for(int k=0; k<n_br; k++){
                int b0 = br_lo[k], b1 = br_hi[k];
                // 브랜치리스 max — 지배 바이트 루프 SIMD 벡터화 허용 (결과 동일)
                for(int b=b0; b<b1; b++)
                    cm_ptr[b] = std::max(cm_ptr[b], rb_ptr[b]);
            }
            if(++n_sampled >= rows_per_col_max) break;
        }
        // Y axis: max-hold across all bins mapped to each pixel row (avoids
        // missing strong signals when many bins fall into one pixel).
        for(int y=0; y<H; y++){
            uint8_t mx = 0;
            for(int idx = y_lo[y]; idx < y_hi[y]; idx++){
                int bin = (idx < fft_half) ? (idx + fft_half) : (idx - fft_half);
                if(col_max[bin] > mx) mx = col_max[bin];
            }
            g_pixel_buf[(size_t)y*W + x] = color_lut[mx];
        }
    }

    if(!g_tex){
        glGenTextures(1, &g_tex);
        glBindTexture(GL_TEXTURE_2D, g_tex);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    }
    glBindTexture(GL_TEXTURE_2D, g_tex);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, W, H, 0, GL_RGBA, GL_UNSIGNED_BYTE, g_pixel_buf.data());
    glBindTexture(GL_TEXTURE_2D, 0);

    g_tex_dirty = false;
}

// UTC offset (hours) for the file. KST 강제 (UTC+9). 파라미터는 호환을 위해 유지.
int header_utc_offset(const LongWaterfall::FileHeader& /*h*/){
    return KST::OFFSET_HOURS;
}

// Format epoch → "YYYY-MM-DD HH:MM:SS" (KST 기준, off_h 무시).
std::string fmt_local_time(uint64_t utc, int /*off_h_ignored*/){
    struct tm tm_kst; KST::to_tm((time_t)utc, tm_kst);
    char buf[40];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", &tm_kst);
    return buf;
}

std::string fmt_duration_hms(uint64_t total_sec){
    uint64_t h = total_sec / 3600;
    uint64_t m = (total_sec % 3600) / 60;
    uint64_t s = total_sec % 60;
    char buf[24];
    snprintf(buf, sizeof(buf), "%02llu:%02llu:%02llu",
             (unsigned long long)h, (unsigned long long)m, (unsigned long long)s);
    return buf;
}

// Recognize a HIST file by name (.bewehist new, .bewewf legacy, or "wfimg_/HIST_" prefix).
static bool is_hist_filename(const std::string& fn){
    if(fn.size() >= 9 && fn.compare(fn.size()-9, 9, ".bewehist")==0) return true;
    if(fn.size() >= 7 && fn.compare(fn.size()-7, 7, ".bewewf")==0)  return true;
    if(fn.rfind("wfimg_", 0) == 0) return true;
    if(fn.rfind("HIST_",  0) == 0) return true;
    return false;
}

void register_dl_callbacks_once(NetClient* cli){
    static NetClient* s_bound = nullptr;
    if(s_bound == cli) return;
    if(!cli) return;
    s_bound = cli;
    // 다운로드는 기존 FILE_META/FILE_DATA 메커니즘 재사용. 저장 dir만 결정.
    // 다른 file transfer (region/share)와 충돌하지 않도록 HIST 파일명만 리다이렉트.
    auto prev_get_dir = cli->on_get_save_dir;
    cli->on_get_save_dir = [prev_get_dir](const std::string& fn) -> std::string {
        if(is_hist_filename(fn)){
            std::string dir = BEWEPaths::hist_join_dir();
            mkdir(BEWEPaths::recordings_dir().c_str(), 0755);
            mkdir(BEWEPaths::hist_dir().c_str(), 0755);
            mkdir(dir.c_str(), 0755);
            return dir;
        }
        return prev_get_dir ? prev_get_dir(fn) : std::string();
    };
    auto prev_meta = cli->on_file_meta;
    cli->on_file_meta = [prev_meta](const std::string& name, uint64_t total){
        if(is_hist_filename(name)) return;   // HIST 파일은 Archive 패널에 노출되지 않게 chain 차단
        if(prev_meta) prev_meta(name, total);
    };
    auto prev_prog = cli->on_file_progress;
    cli->on_file_progress = [prev_prog](const std::string& name, uint64_t done, uint64_t total){
        if(is_hist_filename(name)) return;   // chain 차단 (패널 제거 후 진행률 표시 없음)
        if(prev_prog) prev_prog(name, done, total);
    };
    // on_file_done도 가로채야 Archive에 'IQ_*.wav 다운로드 완료' 같은 게 안 뜸
    auto prev_done = cli->on_file_done;
    cli->on_file_done = [prev_done](const std::string& path, const std::string& name){
        if(is_hist_filename(name)) return;   // chain 차단
        if(prev_done) prev_done(path, name);
    };

    // LIVE 수신 콜백 (v4.6.0 제거): JOIN은 실시간 hist row 스트림을 받지 않음.
    // Central archive 가 source-of-truth — 미션창에서 수동 다운로드.
}


} // anon

// External entry point for mission_view: open a HIST file in the viewer.
// Returns true on success — caller should set v.lwf_modal_open = true.
// Anon-namespace open_file has internal linkage; in the same TU the unqualified
// call resolves to it via the implicit using-directive on anonymous namespaces.
bool lwf_open_file(const std::string& path){
    return open_file(path);
}

namespace LongWaterfallView {

void draw_modal(FFTViewer& v, NetClient* cli){
    // HIST 모달 새로 열릴 때마다 file panel 기본 open + HOST 탭 활성.
    static bool s_prev_open = false;
    bool first_open = (v.lwf_modal_open && !s_prev_open);
    s_prev_open = v.lwf_modal_open;
    if(!v.lwf_modal_open) return;
    // 미션 모달 위에 떠올라야 ESC가 이 창에 작용.
    if(first_open) ImGui::SetNextWindowFocus();

    register_dl_callbacks_once(cli);

    ImGuiIO& io = ImGui::GetIO();
    constexpr float kBottomBarH = 32.0f;
    float modal_h = io.DisplaySize.y - kBottomBarH;
    if(modal_h < 100.f) modal_h = 100.f;

    ImGui::SetNextWindowPos(ImVec2(0,0));
    ImGui::SetNextWindowSize(ImVec2(io.DisplaySize.x, modal_h));
    ImGui::SetNextWindowBgAlpha(0.97f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.f);
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.05f,0.06f,0.10f,1.f));
    // NoTitleBar — 상단 파란 제목바 제거 (사용자 요청).
    ImGui::Begin("##lwf_modal", &v.lwf_modal_open,
        ImGuiWindowFlags_NoTitleBar|ImGuiWindowFlags_NoResize|ImGuiWindowFlags_NoMove|
        ImGuiWindowFlags_NoCollapse|ImGuiWindowFlags_NoScrollbar);

    // S키 file-panel 토글 제거 (v4.0): mission 창에서 파일 선택해 진입.
    // 우측 file panel 자체도 항상 닫힌 채 — viewer는 viewer 본연만 담당.
    bool modal_focused = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);
    // ESC로 모달 닫기 (titlebar X 없음 보완).
    if(modal_focused && ImGui::IsKeyPressed(ImGuiKey_Escape, false)){
        v.lwf_modal_open = false;
    }

    ImVec2 win_sz = ImGui::GetContentRegionAvail();
    float view_w  = std::max(200.f, win_sz.x);

    // ── Left: viewer ─────────────────────────────────────────────────────
    // 정보 영역만 작은 패딩, 이미지는 child 가장자리까지 꽉 차게.
    ImGui::BeginChild("##lwf_view", ImVec2(view_w, win_sz.y), false);

    // Live refresh size if open file is current LIVE.
    std::string live_path = LongWaterfall::current_file_path();
    bool g_open_loaded = !g_open.path.empty();

    if(!g_open_loaded){
        // (안내문 제거 — 빈 viewer 표시)
    } else {
        // host의 LIVE 파일이면 file size 폴링 (JOIN 측 hist/live/ mirror 는 v4.6.0 에서 제거).
        if(g_open.path == live_path) refresh_size_live();
        // 색 윈도는 파일 데이터에서 — LIVE 로 자라면 새 행만 이어서 스캔.
        {
            float pmin = g_file_db_min, pmax = g_file_db_max;
            scan_file_db_range();
            if(g_file_db_min != pmin || g_file_db_max != pmax) g_tex_dirty = true;
        }
        const auto& h = g_open.hdr;
        int off_h = header_utc_offset(h);
        float row_rate = std::max(1.0f, h.row_rate_hz);
        uint64_t dur_sec = (uint64_t)((float)g_open.num_rows / row_rate);
        uint64_t stop_utc = h.start_utc_unix + dur_sec;

        unsigned fft_disp = h.fft_input_size > 0 ? h.fft_input_size : h.fft_size;
        ImGui::Dummy(ImVec2(0, 4));
        ImGui::Indent(10.0f);
        ImGui::Text("CF : %.3f MHz   SR : %.2f MSPS   FFT : %u   Duration : %s   Size : %.1f MB",
            h.center_freq_hz / 1e6,
            h.sample_rate_hz / 1e6,
            fft_disp,
            fmt_duration_hms(dur_sec).c_str(),
            g_open.total_size / 1048576.0);
        if(h.station_name[0]){
            ImGui::Text("Station : %s (%s)",
                h.station_name,
                LongWaterfall::fmt_lat_lon(h.station_lat, h.station_lon).c_str());
        }
        ImGui::Text("Start : %s", fmt_local_time(h.start_utc_unix, off_h).c_str());
        ImGui::Text("Stop  : %s", fmt_local_time(stop_utc, off_h).c_str());
        ImGui::Text("Color : %.1f / %.1f dB",
            g_file_db_min, g_file_db_max);
        ImGui::Unindent(10.0f);
        ImGui::Dummy(ImVec2(0, 2));
        ImGui::Separator();

        // Image는 viewer child 끝까지 꽉 차게 (FramePadding/ItemSpacing 0).
        ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing,  ImVec2(0,0));
        ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(0,0));
        ImVec2 img_sz = ImGui::GetContentRegionAvail();
        if(img_sz.x >= 64 && img_sz.y >= 64){
            int target_w = std::min(2048, std::max(256, (int)img_sz.x));
            int target_h = std::min(1024, std::max(128, (int)img_sz.y));
            if(target_w != g_tex_w || target_h != g_tex_h){
                g_tex_w = target_w; g_tex_h = target_h;
                g_tex_dirty = true;
            }
            if(g_tex_dirty) rebuild_texture(g_file_db_min, g_file_db_max);

            ImVec2 img_pos = ImGui::GetCursorScreenPos();
            if(g_tex){
                ImGui::Image((ImTextureID)(intptr_t)g_tex, img_sz);
            }
            bool hov = ImGui::IsItemHovered();
            if(hov && io.MouseWheel != 0.f){
                double zf = (io.MouseWheel > 0) ? 0.8 : 1.25;
                ImVec2 mp = io.MousePos;
                if(io.KeyCtrl){
                    double frac_top = (mp.y - img_pos.y) / (double)img_sz.y;
                    double mv = g_f1 - frac_top * (g_f1 - g_f0);
                    double new_span = (g_f1 - g_f0) * zf;
                    g_f1 = mv + frac_top * new_span;
                    g_f0 = g_f1 - new_span;
                    if(g_f0 < 0){ g_f1 -= g_f0; g_f0 = 0; }
                    if(g_f1 > h.fft_size){ g_f0 -= (g_f1 - h.fft_size); g_f1 = h.fft_size; }
                    if(g_f0 < 0) g_f0 = 0;
                    if(g_f1 - g_f0 < 4) g_f1 = g_f0 + 4;
                } else {
                    double frac_x = (mp.x - img_pos.x) / (double)img_sz.x;
                    double mu = g_t0 + frac_x * (g_t1 - g_t0);
                    double new_span = (g_t1 - g_t0) * zf;
                    g_t0 = mu - frac_x * new_span;
                    g_t1 = g_t0 + new_span;
                    if(g_t0 < 0){ g_t1 -= g_t0; g_t0 = 0; }
                    if(g_t1 > g_open.num_rows){ g_t0 -= (g_t1 - g_open.num_rows); g_t1 = g_open.num_rows; }
                    if(g_t0 < 0) g_t0 = 0;
                    if(g_t1 - g_t0 < 4) g_t1 = g_t0 + 4;
                }
                g_tex_dirty = true;
            }
            bool focused = ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows);
            if(focused && ImGui::IsKeyPressed(ImGuiKey_Home, false)){
                g_t0 = 0; g_t1 = std::max<uint32_t>(1, g_open.num_rows);
                g_f0 = 0; g_f1 = h.fft_size;
                g_tex_dirty = true;
            }
            if(focused && !io.WantTextInput){
                // 방향키 = 줌 크기 유지 팬. 좌우=시간, 상하=주파수. 한 번에 화면의 50%.
                double tspan = g_t1 - g_t0;
                double ttotal = (double)g_open.num_rows;
                double tstep = tspan * 0.5;
                if(ImGui::IsKeyPressed(ImGuiKey_LeftArrow, false)){
                    double t0 = g_t0 - tstep;
                    if(t0 < 0) t0 = 0;
                    g_t0 = t0; g_t1 = t0 + tspan;
                    g_tex_dirty = true;
                }
                if(ImGui::IsKeyPressed(ImGuiKey_RightArrow, false)){
                    double t1 = g_t1 + tstep;
                    if(t1 > ttotal) t1 = ttotal;
                    g_t0 = t1 - tspan; g_t1 = t1;
                    g_tex_dirty = true;
                }
                double fspan = g_f1 - g_f0;
                double ftotal = (double)h.fft_size;
                double fstep = fspan * 0.5;
                // Up = 주파수 위로(높은 쪽), Down = 아래로
                if(ImGui::IsKeyPressed(ImGuiKey_UpArrow, false)){
                    double f1 = g_f1 + fstep;
                    if(f1 > ftotal) f1 = ftotal;
                    g_f0 = f1 - fspan; g_f1 = f1;
                    g_tex_dirty = true;
                }
                if(ImGui::IsKeyPressed(ImGuiKey_DownArrow, false)){
                    double f0 = g_f0 - fstep;
                    if(f0 < 0) f0 = 0;
                    g_f0 = f0; g_f1 = f0 + fspan;
                    g_tex_dirty = true;
                }
            }

            // ── Measurement region overlay ─────────────────────────────
            // Ctrl+우클릭 드래그 = 새 영역 / 좌클릭으로 모서리 잡고 늘리기·이동
            // 더블클릭 또는 Del 키 = 영역 제거
            // 표시: 빨간 박스 + Bandwidth(MHz) + Duration(s)
            {
                ImVec2 mp = io.MousePos;
                auto px_to_t = [&](float x){ return g_t0 + (double)(x - img_pos.x) / (double)img_sz.x * (g_t1 - g_t0); };
                auto px_to_f = [&](float y){ return g_f1 - (double)(y - img_pos.y) / (double)img_sz.y * (g_f1 - g_f0); };
                auto t_to_px = [&](double t){ return img_pos.x + (float)((t - g_t0) / (g_t1 - g_t0) * img_sz.x); };
                auto f_to_py = [&](double f){ return img_pos.y + (float)((g_f1 - f) / (g_f1 - g_f0) * img_sz.y); };

                bool ctrl  = io.KeyCtrl;
                bool in_img = (mp.x>=img_pos.x && mp.x<=img_pos.x+img_sz.x &&
                               mp.y>=img_pos.y && mp.y<=img_pos.y+img_sz.y);

                // 시작: Ctrl+우클릭
                if(in_img && ctrl && ImGui::IsMouseClicked(ImGuiMouseButton_Right)){
                    g_meas.selecting = true; g_meas.active = false;
                    g_meas.edit = Meas::EDIT_NONE;
                    g_meas.t0 = g_meas.t1 = px_to_t(mp.x);
                    g_meas.f0 = g_meas.f1 = px_to_f(mp.y);
                }
                if(g_meas.selecting && ImGui::IsMouseDown(ImGuiMouseButton_Right)){
                    g_meas.t1 = px_to_t(mp.x);
                    g_meas.f1 = px_to_f(mp.y);
                    ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
                }
                if(g_meas.selecting && ImGui::IsMouseReleased(ImGuiMouseButton_Right)){
                    g_meas.selecting = false;
                    // normalize so t0<t1, f0<f1
                    if(g_meas.t0 > g_meas.t1) std::swap(g_meas.t0, g_meas.t1);
                    if(g_meas.f0 > g_meas.f1) std::swap(g_meas.f0, g_meas.f1);
                    if((g_meas.t1 - g_meas.t0) > 1.0 && (g_meas.f1 - g_meas.f0) > 1.0)
                        g_meas.active = true;
                }

                // 박스 + BW/Duration 라벨 렌더 (Meas 영역과 좌드래그 줌박스 공용)
                auto draw_region_box = [&](double bt0, double bt1, double bf0, double bf1){
                    float rx0 = t_to_px(bt0), rx1 = t_to_px(bt1);
                    float ry_lo = f_to_py(bf0);                // 낮은 주파수 → 화면 아래
                    float ry_hi = f_to_py(bf1);                // 높은 주파수 → 화면 위
                    if(rx1 < rx0) std::swap(rx0, rx1);
                    if(ry_hi > ry_lo) std::swap(ry_hi, ry_lo);
                    ImDrawList* dlf = ImGui::GetWindowDrawList();
                    dlf->AddRectFilled(ImVec2(rx0, ry_hi), ImVec2(rx1, ry_lo),
                                       IM_COL32(255,60,60,40));
                    dlf->AddRect(ImVec2(rx0, ry_hi), ImVec2(rx1, ry_lo),
                                 IM_COL32(255,80,80,220), 0.f, 0, 1.5f);

                    // Info text: BW (MHz), Duration (s)
                    double f_lo_idx = std::min(bf0, bf1);
                    double f_hi_idx = std::max(bf0, bf1);
                    double bw_mhz = (f_hi_idx - f_lo_idx) / (double)h.fft_size * (h.sample_rate_hz / 1e6);
                    double t_lo = std::min(bt0, bt1);
                    double t_hi = std::max(bt0, bt1);
                    double dur_s = (t_hi - t_lo) / (double)std::max(1.f, h.row_rate_hz);
                    // 60s 초과면 'MM m SS.sss s' 로, 그 이하면 초 단위 그대로.
                    char dur_str[32];
                    if(dur_s >= 60.0){
                        int    mins = (int)(dur_s / 60.0);
                        double secs = dur_s - mins * 60.0;
                        snprintf(dur_str, sizeof(dur_str), "%d m %06.3f s", mins, secs);
                    } else {
                        snprintf(dur_str, sizeof(dur_str), "%.3f s", dur_s);
                    }
                    char info[96];
                    if(bw_mhz > 1.0)
                        snprintf(info, sizeof(info), "BW : %.3f MHz   Duration : %s",
                                 bw_mhz, dur_str);
                    else
                        snprintf(info, sizeof(info), "BW : %.1f kHz   Duration : %s",
                                 bw_mhz * 1000.0, dur_str);
                    ImVec2 ts = ImGui::CalcTextSize(info);
                    float tx = rx0 + 4.f;
                    float ty = ry_hi - ts.y - 4.f;
                    if(ty < img_pos.y + 2) ty = ry_lo + 4.f;
                    dlf->AddRectFilled(ImVec2(tx-3, ty-2), ImVec2(tx+ts.x+3, ty+ts.y+2),
                                       IM_COL32(0,0,0,170));
                    dlf->AddText(ImVec2(tx, ty), IM_COL32(255,200,200,255), info);
                };

                // Render rectangle
                if(g_meas.active || g_meas.selecting)
                    draw_region_box(g_meas.t0, g_meas.t1, g_meas.f0, g_meas.f1);

                // Edit (resize/move) via 좌클릭 — region active 일 때만
                if(g_meas.active && !g_meas.selecting && !ctrl){
                    float rx0 = t_to_px(std::min(g_meas.t0, g_meas.t1));
                    float rx1 = t_to_px(std::max(g_meas.t0, g_meas.t1));
                    float ry_hi = f_to_py(std::max(g_meas.f0, g_meas.f1)); // top
                    float ry_lo = f_to_py(std::min(g_meas.f0, g_meas.f1)); // bottom
                    const float EDGE = 6.f;
                    bool on_l = std::fabs(mp.x - rx0) <= EDGE && mp.y >= ry_hi - EDGE && mp.y <= ry_lo + EDGE;
                    bool on_r = std::fabs(mp.x - rx1) <= EDGE && mp.y >= ry_hi - EDGE && mp.y <= ry_lo + EDGE;
                    bool on_t = std::fabs(mp.y - ry_hi) <= EDGE && mp.x >= rx0 - EDGE && mp.x <= rx1 + EDGE;
                    bool on_b = std::fabs(mp.y - ry_lo) <= EDGE && mp.x >= rx0 - EDGE && mp.x <= rx1 + EDGE;
                    bool inside = (mp.x>=rx0 && mp.x<=rx1 && mp.y>=ry_hi && mp.y<=ry_lo);
                    if(g_meas.edit == Meas::EDIT_NONE){
                        if(on_l)      ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
                        else if(on_r) ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
                        else if(on_t) ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
                        else if(on_b) ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeNS);
                        else if(inside) ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeAll);
                    }
                    if(g_meas.edit == Meas::EDIT_NONE && (inside || on_l || on_r || on_t || on_b) &&
                       ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
                        g_meas.sv_t0 = g_meas.t0; g_meas.sv_t1 = g_meas.t1;
                        g_meas.sv_f0 = g_meas.f0; g_meas.sv_f1 = g_meas.f1;
                        g_meas.mx0 = px_to_t(mp.x);
                        g_meas.my0 = px_to_f(mp.y);
                        if(on_l)      g_meas.edit = Meas::EDIT_L;
                        else if(on_r) g_meas.edit = Meas::EDIT_R;
                        else if(on_t) g_meas.edit = Meas::EDIT_T;
                        else if(on_b) g_meas.edit = Meas::EDIT_B;
                        else          g_meas.edit = Meas::EDIT_MOVE;
                    }
                    if(g_meas.edit != Meas::EDIT_NONE){
                        if(ImGui::IsMouseDown(ImGuiMouseButton_Left)){
                            double dt = px_to_t(mp.x) - g_meas.mx0;
                            double df = px_to_f(mp.y) - g_meas.my0;
                            switch(g_meas.edit){
                                case Meas::EDIT_MOVE:
                                    g_meas.t0 = g_meas.sv_t0 + dt; g_meas.t1 = g_meas.sv_t1 + dt;
                                    g_meas.f0 = g_meas.sv_f0 + df; g_meas.f1 = g_meas.sv_f1 + df;
                                    break;
                                case Meas::EDIT_L: {
                                    double t_min = std::min(g_meas.sv_t0, g_meas.sv_t1) + dt;
                                    double t_max = std::max(g_meas.sv_t0, g_meas.sv_t1);
                                    g_meas.t0 = t_min; g_meas.t1 = t_max;
                                } break;
                                case Meas::EDIT_R: {
                                    double t_min = std::min(g_meas.sv_t0, g_meas.sv_t1);
                                    double t_max = std::max(g_meas.sv_t0, g_meas.sv_t1) + dt;
                                    g_meas.t0 = t_min; g_meas.t1 = t_max;
                                } break;
                                case Meas::EDIT_T: {
                                    double f_min = std::min(g_meas.sv_f0, g_meas.sv_f1);
                                    double f_max = std::max(g_meas.sv_f0, g_meas.sv_f1) + df;
                                    g_meas.f0 = f_min; g_meas.f1 = f_max;
                                } break;
                                case Meas::EDIT_B: {
                                    double f_min = std::min(g_meas.sv_f0, g_meas.sv_f1) + df;
                                    double f_max = std::max(g_meas.sv_f0, g_meas.sv_f1);
                                    g_meas.f0 = f_min; g_meas.f1 = f_max;
                                } break;
                                default: break;
                            }
                        }
                        if(ImGui::IsMouseReleased(ImGuiMouseButton_Left)){
                            g_meas.edit = Meas::EDIT_NONE;
                            // 너무 작아진 영역은 제거
                            double tw = std::fabs(g_meas.t1 - g_meas.t0);
                            double fw = std::fabs(g_meas.f1 - g_meas.f0);
                            if(tw < 1.0 || fw < 1.0) g_meas.active = false;
                        }
                    }
                    // 더블클릭으로 제거 (영역 안에서 좌클릭)
                    if(inside && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)){
                        g_meas.active = false;
                        g_meas.edit = Meas::EDIT_NONE;
                    }
                }
                // Del 키 = 영역 제거 (모달 focus + 입력 중 아닐 때)
                if(g_meas.active && focused && !io.WantTextInput &&
                   ImGui::IsKeyPressed(ImGuiKey_Delete, false)){
                    g_meas.active = false;
                    g_meas.edit = Meas::EDIT_NONE;
                }

                // ── 좌드래그 줌 (박스 친 영역으로 바로 확대) ────────────────
                // Meas 편집(모서리/이동)과 겹치지 않게: Meas 편집 중이 아닐 때만.
                // Ctrl 없이 이미지 위에서 좌클릭 드래그 → 빨간박스 → 놓으면 그 영역으로 줌.
                bool meas_busy = g_meas.selecting || g_meas.edit != Meas::EDIT_NONE;
                if(!ctrl && !meas_busy){
                    if(in_img && ImGui::IsMouseClicked(ImGuiMouseButton_Left)){
                        g_zdrag.active = true;
                        g_zdrag.px0 = mp.x; g_zdrag.py0 = mp.y;
                        g_zdrag.t0 = g_zdrag.t1 = px_to_t(mp.x);
                        g_zdrag.f0 = g_zdrag.f1 = px_to_f(mp.y);
                    }
                    if(g_zdrag.active && ImGui::IsMouseDown(ImGuiMouseButton_Left)){
                        g_zdrag.t1 = px_to_t(mp.x);
                        g_zdrag.f1 = px_to_f(mp.y);
                    }
                    if(g_zdrag.active && ImGui::IsMouseReleased(ImGuiMouseButton_Left)){
                        g_zdrag.active = false;
                        // 실수 클릭 방지 — 화면상 드래그 폭이 양축 모두 임계 미만이면 무시
                        const float MIN_DRAG_PX = 6.f;
                        if(std::fabs(mp.x - g_zdrag.px0) >= MIN_DRAG_PX &&
                           std::fabs(mp.y - g_zdrag.py0) >= MIN_DRAG_PX){
                            double nt0 = std::min(g_zdrag.t0, g_zdrag.t1);
                            double nt1 = std::max(g_zdrag.t0, g_zdrag.t1);
                            double nf0 = std::min(g_zdrag.f0, g_zdrag.f1);
                            double nf1 = std::max(g_zdrag.f0, g_zdrag.f1);
                            if(nt1 - nt0 >= 1.0 && nf1 - nf0 >= 1.0){
                                push_zoom_hist();        // 현재 뷰를 뒤로가기 스택에 저장
                                g_t0 = nt0; g_t1 = nt1;
                                g_f0 = nf0; g_f1 = nf1;
                                g_tex_dirty = true;
                            }
                        }
                    }
                    // 드래그 중 빨간 박스 + BW/Duration 렌더 (Meas와 동일 코드 재사용)
                    if(g_zdrag.active)
                        draw_region_box(g_zdrag.t0, g_zdrag.t1, g_zdrag.f0, g_zdrag.f1);
                }

                // ── 줌 뒤로가기 ── Ctrl+Z 또는 순수 우클릭(드래그 없는 단일 클릭)
                if(focused && !io.WantTextInput && ctrl &&
                   ImGui::IsKeyPressed(ImGuiKey_Z, false)){
                    pop_zoom_hist();
                }
                // 우클릭: Ctrl+우클릭은 Meas 생성이라 제외. 순수 우클릭 = 뒤로가기.
                if(in_img && !ctrl && ImGui::IsMouseReleased(ImGuiMouseButton_Right) &&
                   !g_meas.selecting){
                    ImVec2 dm = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right);
                    if(std::fabs(dm.x) < 4.f && std::fabs(dm.y) < 4.f)
                        pop_zoom_hist();
                }
            }

            if(hov){
                ImVec2 mp = io.MousePos;
                double t = g_t0 + (mp.x - img_pos.x) / img_sz.x * (g_t1 - g_t0);
                double freq_idx = g_f1 - (mp.y - img_pos.y) / img_sz.y * (g_f1 - g_f0);
                double t_sec = t / (double)h.row_rate_hz;
                uint64_t hover_utc = h.start_utc_unix + (uint64_t)t_sec;
                // 날짜는 상단 정보란(Start/Stop)에 있으므로 툴팁은 HH:MM:SS 만.
                struct tm tm_kst; KST::to_tm((time_t)hover_utc, tm_kst);
                char tbuf[16];
                strftime(tbuf, sizeof(tbuf), "%H:%M:%S", &tm_kst);
                double cf_mhz = h.center_freq_hz / 1e6;
                double sr_mhz = h.sample_rate_hz / 1e6;
                double fmhz = cf_mhz + (freq_idx / (double)h.fft_size - 0.5) * sr_mhz;
                // SNR = 커서 지점 dB - 파일 전역 노이즈플로어.
                // scan_file_db_range() 가 하위 15% 분위수를 noise 로 잡고
                // g_file_db_min = noise - 5 로 저장하므로 +5 로 역산한다.
                // 실측(v3 파일 6종, 각 12구간): 파일 내 노이즈플로어 변동 평균 0.33dB,
                // 최대 1.4dB(게인 변경 케이스) — 바이트 양자화(0.17~0.39dB/byte) 수준이라
                // 행 단위 재계산 없이 전역값으로 충분하다.
                int  row_i = (int)t;
                int  lin_i = (int)freq_idx;          // 화면 선형 인덱스 (0=최저, fft/2=DC)
                bool have_snr = false;
                float snr_db = 0.f;
                if(row_i >= 0 && (uint64_t)row_i < g_open.num_rows &&
                   lin_i >= 0 && (uint32_t)lin_i < h.fft_size &&
                   g_file_db_max > g_file_db_min){
                    // FFT-shift 해제 — rebuild_texture() 와 동일 규칙.
                    // linear: 0=lowest, fft_half=DC, fft_sz-1=highest / storage: 0=DC.
                    const int fft_half = (int)h.fft_size / 2;
                    const int bin_i = (lin_i < fft_half) ? lin_i + fft_half : lin_i - fft_half;
                    if(const uint8_t* rp = get_row((uint32_t)row_i)){
                        float d = LongWaterfall::byte_to_db(rp[bin_i], h.db_min, h.db_max);
                        snr_db   = d - (g_file_db_min + 5.0f);
                        have_snr = true;
                    }
                }
                if(have_snr)
                    ImGui::SetTooltip("%s\n%.3fMHz\nSNR %.1fdB", tbuf, fmhz, snr_db);
                else
                    ImGui::SetTooltip("%s\n%.3fMHz", tbuf, fmhz);
            }
        }
        ImGui::PopStyleVar(2);   // ItemSpacing + FramePadding
    }
    ImGui::EndChild();

    ImGui::End();
    ImGui::PopStyleColor();
    ImGui::PopStyleVar();   // WindowRounding

}

void close_modal(){
    close_open();
    if(g_tex){ glDeleteTextures(1, &g_tex); g_tex = 0; }
}

} // namespace LongWaterfallView
