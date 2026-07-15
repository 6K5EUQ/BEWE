#pragma once
// Long-Waterfall (host-side): post-FFT magnitude → 1 byte/bin → disk file,
// rotated on SR/CF/fft_size/IQ-rolling on-off changes. CLI/GUI 공용.
//
// Worker thread polls FFTViewer::fft_data + total_ffts, max-hold compresses
// capture rows down to ~5 row/sec, quantizes float dB to uint8 (db_min..db_max
// → 0..255), appends to .bewewf file under ~/BEWE/recordings/long_waterfall/.
//
// File format (62B header rounded to 64, then raw rows):
//   "BWWF"(4) ver(2) fft_size(4) sample_rate(8) center_freq(8)
//   row_rate_hz(4 float) db_min(4 float) db_max(4 float) start_utc(8) reserved(16)
// Each row = fft_size bytes.

#include <cstdint>
#include <cstdio>
#include <cmath>
#include <ctime>
#include <string>
#include <functional>

#include "net_protocol.hpp"
#include "kst_time.hpp"

class FFTViewer;

namespace LongWaterfall {

// Disk file header — v3 layout (128 B). v2(64B) and earlier rejected by reader.
#pragma pack(push, 1)
struct FileHeader {
    char     magic[4];          // "BWWF"
    uint16_t version;           // 0x0003
    uint32_t fft_size;
    uint64_t sample_rate_hz;
    uint64_t center_freq_hz;
    float    row_rate_hz;       // rows per second (target, e.g. 5.0)
    float    db_min;            // dB → byte 0
    float    db_max;            // dB → byte 255
    uint64_t start_utc_unix;    // file creation, UTC seconds
    float    station_lon;       // signed degrees (v3: filled from FFTViewer.station_lon)
    uint32_t fft_input_size;    // user-set FFT size (= fft_size / FFT_PAD_FACTOR); 0 if unknown
    int32_t  utc_offset_hours;  // host system TZ at file open (tm_gmtoff/3600)
    uint8_t  reserved_v2[6];    // pad to 64 (v2 layout end)
    // ── v3 extension ────
    char     station_name[32];  // null-terminated; "" if unknown
    float    station_lat;       // signed degrees
    uint8_t  reserved_v3[28];   // pad to 128
};
#pragma pack(pop)
static_assert(sizeof(FileHeader) == 128, "LongWaterfall::FileHeader v3 must be 128");

constexpr uint16_t FILE_VERSION      = 0x0003;
constexpr uint16_t FILE_VERSION_ZSTD = 0x0004;   // v4: block-zstd compressed body

// ── v4 압축 컨테이너 (블록 zstd + footer index) ───────────────────────────
// v4 파일은 v3 헤더와 바이트 동일하되 version=0x0004, 그리고 reserved_v3[28] 에
// 아래 확장 필드를 담는다. body = [독립 zstd 프레임 × num_blocks][footer index].
// 프레임 i = rows [i*block_rows, min((i+1)*block_rows, num_rows)) 를 raw(1B/bin)로
// 이어붙인 뒤 통째 압축. reader 는 size 산술 대신 헤더 num_rows 를 신뢰한다.
constexpr uint8_t HIST_CODEC_ZSTD           = 1;
constexpr uint8_t HIST_V4_FLAG_FRAME_CKSUM  = 0x01;
// 각 행을 주파수축 delta(d[i]=x[i]-x[i-1], uint8 wrap)로 변환 후 압축 → 압축비 개선
// (32768폭 실측 3.7→4.7x). reader 는 해제 후 행별 prefix-sum 으로 복원.
constexpr uint8_t HIST_V4_FLAG_COL_DELTA    = 0x02;

#pragma pack(push, 1)
struct V4Ext {                 // FileHeader.reserved_v3 (파일오프셋 100) 위에 overlay
    uint8_t  codec;            // HIST_CODEC_ZSTD
    uint8_t  flags;            // HIST_V4_FLAG_*
    uint32_t block_rows;       // 프레임당 행 수
    uint32_t num_blocks;       // == ceil(num_rows / block_rows)
    uint64_t num_rows;         // authoritative 행 수 (size 산술 대체)
    uint64_t index_offset;     // 파일오프셋: footer index 시작
};
struct HistBlockIndex {        // footer index entry (16B)
    uint64_t frame_offset;     // 파일오프셋: zstd 프레임 시작
    uint32_t comp_len;         // 압축 바이트 길이 (ZSTD_decompress srcSize)
    uint32_t raw_rows;         // 해제 후 행 수 (마지막 블록 부분 + 검증)
};
#pragma pack(pop)
static_assert(sizeof(V4Ext) <= sizeof(FileHeader::reserved_v3),
              "V4Ext must fit in FileHeader::reserved_v3");
static_assert(sizeof(HistBlockIndex) == 16, "HistBlockIndex must be 16 bytes");

// reserved_v3 overlay 접근자 (packed struct — 코드베이스 net-struct 관행과 동일).
inline V4Ext&       v4ext(FileHeader& h){ return *reinterpret_cast<V4Ext*>(h.reserved_v3); }
inline const V4Ext& v4ext(const FileHeader& h){ return *reinterpret_cast<const V4Ext*>(h.reserved_v3); }

// 블록당 ~2MB(raw) 목표. reader 는 헤더 block_rows 를 쓰므로 writer 만 이걸 호출.
inline uint32_t hist_default_block_rows(uint32_t fft_size){
    if(fft_size == 0) return 256;
    uint64_t br = (2ull*1024*1024) / fft_size;
    if(br < 64)   br = 64;
    if(br > 1024) br = 1024;
    return (uint32_t)br;
}

constexpr float DEFAULT_ROW_RATE_HZ = 5.0f;
constexpr float DEFAULT_DB_MIN = -120.0f;
constexpr float DEFAULT_DB_MAX = 0.0f;

// Start the host-side worker. Idempotent: subsequent calls are no-op.
// Worker terminates only on stop_worker(). Records while a mission is active
// (v.active_hist_dir() non-empty; TM IQ 롤링과 독립).
void start_worker(FFTViewer* v);
void stop_worker();

// Trigger a file rotation on the next worker iteration.
// Called from capture/IO or UI on SR/CF/fft_size/IQ on-off events.
void request_rotate();

// 현재 열린 HIST 파일을 "dirty" 표시 — Central 연결 끊김이 발생했음을 의미.
// finalize 시점에 dirty 면 통파일을 MissionPush 로 enqueue (LIVE row 누락분 보완).
// dirty 아니면 LIVE tap 만으로 도달했다고 가정하고 push skip.
void mark_dirty();

// Currently-open file path (empty if worker idle / not recording). Thread-safe snapshot.
std::string current_file_path();

// Scan ~/BEWE/recordings/long_waterfall/ → fill PktLwfList from on-disk headers.
// Skips files that are not valid .bewewf (header missing / wrong magic).
void scan_dir_into_list(::PktLwfList& out);

// Scan `dir` for stale "...-LIVE.bewehist" files (host crashed before close → rename
// never ran) and finalize them using file mtime as end time. Skips `active_basename`
// if it matches the currently-recording file. Safe to call from any thread.
void finalize_stale_live_in_dir(const std::string& dir,
                                 const std::string& active_basename);

// Convenience: finalize stale LIVE files in well-known HIST dirs
// (host dir, join dir, live dir, active mission hist dir). Called from
// start_worker() and on mission start.
void finalize_stale_live_all();

// ── Live broadcast hooks ───────────────────────────────────────────────
// Set by host wiring (cli_host / ui). Worker calls these inside open/flush/close
// so NetServer can fan out LIVE_START / LIVE_ROW / LIVE_STOP to all JOINs.
// Callbacks must be cheap (queue-only); worker thread invokes them directly.
struct LiveCallbacks {
    std::function<void(const ::PktLwfLiveStart&)> on_start;
    std::function<void(const ::PktLwfLiveRowHdr& hdr,
                       const uint8_t* row, uint32_t row_bytes)> on_row;
    std::function<void(const ::PktLwfLiveStop&)>  on_stop;
};
void set_live_callbacks(const LiveCallbacks& cbs);

// Build PktLwfLiveStart from currently open LIVE file header (for new JOIN).
// Returns false if no file is currently open.
bool snapshot_live_start(::PktLwfLiveStart& out);

// Public format constants (so view code can quantize/dequantize identically).
inline uint8_t db_to_byte(float db, float dmin, float dmax){
    if(db <= dmin) return 0;
    if(db >= dmax) return 255;
    float t = (db - dmin) / (dmax - dmin);
    int v = (int)(t * 255.0f + 0.5f);
    if(v < 0) v = 0; else if(v > 255) v = 255;
    return (uint8_t)v;
}
inline float byte_to_db(uint8_t b, float dmin, float dmax){
    return dmin + (dmax - dmin) * (b / 255.0f);
}

// ── Station coord formatter (single source of truth) ─────────────────────
// globe.pick stores east-longitude as negative — same convention preserved
// through .bewehist headers. Always render with this helper to keep tooltip,
// info bar, and globe-click label consistent.
inline std::string fmt_lat_lon(float lat, float lon){
    char b[48];
    snprintf(b, sizeof(b), "%.4f%c %.4f%c",
        fabsf(lat), lat>=0 ? 'N' : 'S',
        fabsf(lon), lon>=0 ? 'W' : 'E');
    return b;
}

// ── Mission-code filename helpers (host + JOIN 공용) ──────────────────────
// 양식: <station>_<MissCode><DD>_<Mon><DD>.<YYYY>_<F.F>MHz_<HHMM>-LIVE.bewehist
// 종료 시 -LIVE → -<HHMM> 로 rename (KST 기준).
// MissCode: A=Jan, B=Feb, ..., L=Dec (alphabet, 'I' 포함).
// HHMM/날짜는 항상 KST(UTC+9) — viewer 상단 Start/Stop 표시와 일치.
// utc_offset_hours 파라미터는 호환성을 위해 남기지만 무시 (KST 강제).
inline char mission_letter(int mon0_11){ return (char)('A' + mon0_11); }
inline const char* month_abbr3(int mon0_11){
    static const char* m[] = {"Jan","Feb","Mar","Apr","May","Jun",
                               "Jul","Aug","Sep","Oct","Nov","Dec"};
    return m[mon0_11];
}
inline std::string sanitize_station_hist(const char* sn){
    std::string s = (sn && sn[0]) ? sn : "host";
    for(auto& c : s){
        unsigned char u = (unsigned char)c;
        bool ok = (u>='0'&&u<='9') || (u>='A'&&u<='Z') || (u>='a'&&u<='z')
               || c=='-' || c=='_' || c=='.';
        if(!ok) c = '_';
    }
    return s;
}
inline std::string build_hist_filename_live(uint64_t start_utc, uint64_t cf_hz,
                                             int /*utc_offset_hours_ignored*/,
                                             const char* station_name = nullptr){
    struct tm tm_loc; KST::to_tm((time_t)start_utc, tm_loc);
    double cf_mhz = (double)cf_hz / 1e6;
    std::string st = sanitize_station_hist(station_name);
    char buf[128];
    snprintf(buf, sizeof(buf),
        "%s_%c%02d_%s%02d.%04d_%.1fMHz_%02d%02d-LIVE.bewehist",
        st.c_str(),
        mission_letter(tm_loc.tm_mon), tm_loc.tm_mday,
        month_abbr3(tm_loc.tm_mon), tm_loc.tm_mday, 1900 + tm_loc.tm_year,
        cf_mhz, tm_loc.tm_hour, tm_loc.tm_min);
    return buf;
}
// Returns finalized basename when given a "...-LIVE.bewehist" basename + end_utc.
// If input doesn't match, returns input unchanged.
// 새 형식: 1600-1624.bewehist (KST 기준, Z 접미사 제거).
// 기존 -HHMMZ.bewehist 파일도 같은 dir에 공존 가능 (둘 다 valid).
inline std::string build_hist_filename_finalize(const std::string& live_name,
                                                 uint64_t end_utc,
                                                 int /*utc_offset_hours_ignored*/){
    auto pos = live_name.rfind("-LIVE.bewehist");
    if(pos == std::string::npos) return live_name;
    struct tm tm_loc; KST::to_tm((time_t)end_utc, tm_loc);
    char tail[24];
    snprintf(tail, sizeof(tail), "-%02d%02d.bewehist",
             tm_loc.tm_hour, tm_loc.tm_min);
    return live_name.substr(0, pos) + tail;
}

} // namespace LongWaterfall

// ── GUI viewer (defined in long_waterfall_view.cpp; not built in headless) ──
class NetClient;
namespace LongWaterfallView {
    void draw_modal(FFTViewer& v, NetClient* cli);  // call once per frame when v.lwf_modal_open
    void close_modal();                             // GL cleanup on shutdown
}
