// ── .bewehist 리더 (인스턴스별 상태, GUI 무관) ────────────────────────────────
//
// long_waterfall_view.cpp 의 file-static `g_open` + `get_row()` 를 클래스로 옮긴 것.
// **왜 옮겨야 했나**: get_row() 는 g_open 의 cache_block/cache_buf/frame_buf/rowbuf 를
// 변조하고 반환 포인터의 계약이 "이 상태에 대한 다음 get_row 전까지 유효" 였다. 소비자가
// 렌더 스레드 하나뿐일 땐 성립했지만, 백그라운드 분석 워커가 붙는 순간 한쪽이 압축을
// 풀고 있는 버퍼를 다른 쪽이 읽는다. 단일 소비자 불변식이라 락으로도 못 고치고 상태를
// 인스턴스로 쪼개는 수밖에 없다.
//
// 스레드 규칙: **한 HistReader 를 두 스레드가 동시에 쓰면 안 된다.** 워커는
// clone_for_thread() 로 자기 것을 받아 쓴다. 선해제본(full_buf)만 shared_ptr 로
// 공유되고 — 이게 없으면 워커가 수백 MB 를 두 번째로 압축 해제한다.
#pragma once
#include "long_waterfall.hpp"
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

class HistReader {
public:
    HistReader() = default;
    ~HistReader();
    HistReader(HistReader&&) noexcept;
    HistReader& operator=(HistReader&&) noexcept;
    HistReader(const HistReader&) = delete;
    HistReader& operator=(const HistReader&) = delete;

    bool open(const std::string& path);
    void close();
    bool is_open() const { return fp_ != nullptr || full_ready(); }

    const LongWaterfall::FileHeader& hdr() const { return hdr_; }
    const std::string& path() const { return path_; }
    uint32_t num_rows()       const { return num_rows_; }
    uint32_t fft_size()       const { return hdr_.fft_size; }
    uint32_t fft_input_size() const { return hdr_.fft_input_size; }
    uint64_t total_size()     const { return total_size_; }
    bool     is_v4()          const { return is_v4_; }
    // v4 블록당 행수 (0 = v3). 서브샘플 스캔이 블록 경계로 청크를 잡을 때 쓴다 —
    // 블록 안에서 건너뛰어봐야 어차피 블록 전체를 해제하므로 이득이 없다.
    uint32_t block_rows()     const { return block_rows_; }
    bool     is_live()        const { return path_.find("-LIVE.bewehist") != std::string::npos; }

    // 제로패딩 배수. 구형 파일은 fft_size 가 fft_input_size 의 4배라 인접 bin 이
    // sinc 상관돼 있다 — 피크 분리폭을 여기서 끌어와야 한다.
    uint32_t osr() const {
        const uint32_t fis = hdr_.fft_input_size;
        return (fis == 0 || hdr_.fft_size < fis) ? 1u : hdr_.fft_size / fis;
    }

    // v3 LIVE 파일 성장 반영. 커진 경우에만 true.
    bool refresh_live();

    // v4 전체 선해제. 성공 시 이후 get_row 는 포인터 반환만 한다.
    bool preload_full(uint64_t max_bytes = 768ull*1024*1024);
    bool full_ready() const { return full_ && !full_->empty(); }

    // 행 r 의 fft_size 바이트. **이 리더의 다음 get_row 호출 전까지만 유효**
    // (선해제본이 있으면 리더 수명 내내 유효). 실패 시 nullptr.
    const uint8_t* get_row(uint32_t r);

    // 워커용 사본. 선해제본은 공유하고, 없으면 파일을 따로 열어 자기 캐시를 갖는다.
    HistReader clone_for_thread() const;

    // ── 기하 단일 진실원 ─────────────────────────────────────────────────
    // 뷰어 곳곳에 흩어져 있던 수식을 여기로 모은다. **fft_input_size 를 쓰면 안 된다** —
    // 행 폭도 주파수 축 분모도 fft_size 다 (hist_check.cpp:98 은 HOST 파일에서 둘이
    // 같아 우연히 맞는 것뿐이고 구형 Central 파일에선 틀린다).
    double bin_hz() const {
        return hdr_.fft_size ? (double)hdr_.sample_rate_hz / (double)hdr_.fft_size : 0.0;
    }
    // 선형(fftshift된) 인덱스 → 절대 주파수. lin=fft_size/2 가 center_freq.
    double freq_hz_of_linear(double lin) const {
        const double N = (double)hdr_.fft_size;
        if(N <= 0) return (double)hdr_.center_freq_hz;
        return (double)hdr_.center_freq_hz + (lin - N*0.5)/N * (double)hdr_.sample_rate_hz;
    }
    // 절대 주파수 → 선형 인덱스 (위의 역).
    double linear_of_freq_hz(double f_hz) const {
        const double N = (double)hdr_.fft_size;
        if(hdr_.sample_rate_hz == 0) return N*0.5;
        return (f_hz - (double)hdr_.center_freq_hz)/(double)hdr_.sample_rate_hz*N + N*0.5;
    }
    // 선형 인덱스 → 저장 인덱스. 저장은 bin 0 = DC 라 배열 경계에서 주파수가 끊긴다.
    // **이 언랩은 저장 접근에만 쓰고 주파수 축에는 쓰지 않는다.**
    uint32_t storage_of_linear(uint32_t lin) const {
        const uint32_t half = hdr_.fft_size/2;
        return (lin < half) ? (lin + half) : (lin - half);
    }
    double utc_of_row(double r) const {
        const double rr = (hdr_.row_rate_hz > 0.0f) ? (double)hdr_.row_rate_hz : 1.0;
        return (double)hdr_.start_utc_unix + r/rr;
    }
    float db(uint8_t b) const { return LongWaterfall::byte_to_db(b, hdr_.db_min, hdr_.db_max); }

    double station_lat() const { return (double)hdr_.station_lat; }
    // 헤더는 서경 양수 — 여기서 한 번만 뒤집는다.
    double station_lon_east() const { return LongWaterfall::hist_lon_east(hdr_.station_lon); }
    bool   has_station_pos() const {
        return !(hdr_.station_lat == 0.0f && hdr_.station_lon == 0.0f);
    }

    // 파일을 열지 않고 헤더만. (뷰어의 read_header_only 대체)
    static bool read_header(const std::string& path, LongWaterfall::FileHeader& h, uint64_t& size);
    static uint64_t rows_from_header(const LongWaterfall::FileHeader& h, uint64_t file_size);

private:
    bool map_file();
    void unmap_file();

    std::string path_;
    LongWaterfall::FileHeader hdr_{};
    uint64_t total_size_ = 0;
    uint32_t num_rows_   = 0;
    FILE*    fp_ = nullptr;
    int      fd_ = -1;
    const uint8_t* map_ = nullptr;
    size_t   map_size_  = 0;
    std::vector<uint8_t> rowbuf_;     // v3 mmap 실패 시 1행 fread fallback

    bool     is_v4_ = false;
    uint8_t  v4_flags_ = 0;
    uint32_t block_rows_ = 0;
    std::vector<LongWaterfall::HistBlockIndex> index_;
    int                  cache_block_ = -1;
    std::vector<uint8_t> cache_buf_;
    std::vector<uint8_t> frame_buf_;

    // 선해제본 — 사본 간 공유. 이게 shared_ptr 인 것이 clone_for_thread 의 핵심.
    std::shared_ptr<const std::vector<uint8_t>> full_;
};
